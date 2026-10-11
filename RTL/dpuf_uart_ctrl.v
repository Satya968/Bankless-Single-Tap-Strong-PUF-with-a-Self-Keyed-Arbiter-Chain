// =============================================================================
// dpuf_uart_ctrl.v  --  CRP transfer + key load, over UART
//
// Derived from fpga/rtl/puf_uart_ctrl.v. Only the command sequencer differs,
// because this design needs a key loaded before it can be compared against the
// Python model.
//
// The receiver and transmitter now live in uart_rx.v / uart_tx.v, shared with
// the PicoBlaze controller (rtl/pb/). They used to be inline here, duplicated
// from the Markov-CBO build; sharing them means one implementation to get right
// and one to test -- and tb_uart.v now covers 771 checks of framing, byte
// order, the present/ack handshake and overrun latching, none of which was
// exercised by any testbench while the code was inline.
//
// PROTOCOL -- opcode + 8 payload bytes, MSB-first
//   0xA5  LOAD KEY    8 bytes follow. No reply.
//   0x5A  EVALUATE    8 bytes follow (the challenge). Replies 1 byte:
//                     bit0 = response, bits 7:1 = 0.
//   other             ignored.
//
// WHY AN OPCODE AND NOT A BARE 8-BYTE STREAM
//   The Markov-CBO build uses byte count alone as the sync, which works only
//   because it has exactly one kind of frame. Two frame types need a tag. An
//   unrecognised opcode is DISCARDED rather than treated as data, which also
//   gives the host a resync tool: send a run of 0x00 and the device returns to
//   the opcode state without emitting anything.
//
// ORDER IS NOT PART OF THIS PUF
//   The Markov-CBO controller carries a warning that challenges must be sent in
//   order, never shuffled, because its responses are a sequence and one dropped
//   byte desynchronises the state permanently. THAT WARNING DOES NOT APPLY HERE.
//   This design is stateless per challenge: E_K(C) is rebuilt from C every
//   query. Challenges may be sent in any order, repeated, or dropped, and the
//   board never needs a reset between runs. That is the random-access property
//   novel/RESULTS.md identifies as the stateful family's deployment blocker.
//
//   The key is the one piece of state, and it persists until reloaded or reset.
//
// 8N1, default 115200 baud @ 100 MHz.
// =============================================================================
`timescale 1ns / 1ps

module dpuf_uart_ctrl #(
    parameter CLK_HZ = 100_000_000,
    parameter BAUD   = 115_200,
    parameter N      = 64,
    // Power-on key. Non-zero so that a board which is never sent a LOAD KEY
    // frame still behaves like a keyed device rather than degenerating to
    // K = 0, which by the absorption lemma is a bare 2-XOR APUF and would look
    // like a working PUF while being trivially modelled.
    parameter [N-1:0] KEY_DEFAULT = 64'hA5A5_5A5A_C3C3_3C3C
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        uart_rx,
    output wire        uart_tx,      // driven by the uart_tx instance below

    output reg [N-1:0] challenge,
    output reg [N-1:0] key,
    output reg         start,
    input  wire        response,
    input  wire        response_valid,
    input  wire        busy
);

    localparam [7:0] OP_KEY  = 8'hA5;
    localparam [7:0] OP_EVAL = 8'h5A;

    // ---- shared UART (see uart_rx.v / uart_tx.v) ----------------------------
    wire [7:0] rx_byte;
    wire       rx_present, rx_overrun, tx_busy;
    reg        rx_ack, tx_go;
    reg  [7:0] tx_data;

    uart_rx #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) u_rx (
        .clk(clk), .rst_n(rst_n), .rx(uart_rx),
        .data(rx_byte), .present(rx_present), .ack(rx_ack), .overrun(rx_overrun)
    );

    uart_tx #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) u_tx (
        .clk(clk), .rst_n(rst_n),
        .data(tx_data), .send(tx_go), .tx(uart_tx), .busy(tx_busy)
    );

    // The sequencer below consumes one byte per `rx_done`, so it takes the
    // arrival EDGE of `present` rather than its level -- otherwise a single
    // byte would be consumed on every cycle it stays asserted.
    reg  rx_present_d;
    wire rx_done = rx_present & ~rx_present_d;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) rx_present_d <= 1'b0;
        else        rx_present_d <= rx_present;
    end

    // ---- command sequencer --------------------------------------------------
    localparam C_OP  = 2'd0,   // waiting for an opcode
               C_ARG = 2'd1,   // collecting the 8 payload bytes
               C_RUN = 2'd2,   // PUF evaluating
               C_TX  = 2'd3;

    reg [1:0]   cstate;
    reg [3:0]   byte_cnt;
    reg [N-1:0] shifter;
    reg         op_is_eval;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cstate     <= C_OP;
            byte_cnt   <= 4'd0;
            start      <= 1'b0;
            tx_go      <= 1'b0;
            rx_ack     <= 1'b0;
            tx_data    <= 8'd0;
            shifter    <= {N{1'b0}};
            challenge  <= {N{1'b0}};
            key        <= KEY_DEFAULT;
            op_is_eval <= 1'b0;
        end else begin
            start  <= 1'b0;
            tx_go  <= 1'b0;
            // uart_rx holds `present` until acknowledged. Acking on the same
            // edge the sequencer consumes the byte keeps exactly one byte in
            // flight, which is what makes the overrun flag meaningful: it can
            // then only fire if the HOST outran the device, never because this
            // sequencer was slow to look.
            rx_ack <= rx_done;

            case (cstate)
                C_OP: if (rx_done) begin
                    if (rx_byte == OP_KEY || rx_byte == OP_EVAL) begin
                        op_is_eval <= (rx_byte == OP_EVAL);
                        byte_cnt   <= 4'd0;
                        cstate     <= C_ARG;
                    end
                    // any other byte: stay here, discard -- host resync path
                end

                C_ARG: if (rx_done) begin
                    shifter <= {shifter[N-9:0], rx_byte};   // MSB-first
                    if (byte_cnt == 4'd7) begin
                        byte_cnt <= 4'd0;
                        if (op_is_eval) begin
                            challenge <= {shifter[N-9:0], rx_byte};
                            start     <= 1'b1;
                            cstate    <= C_RUN;
                        end else begin
                            key    <= {shifter[N-9:0], rx_byte};
                            cstate <= C_OP;          // key load sends no reply
                        end
                    end else byte_cnt <= byte_cnt + 4'd1;
                end

                C_RUN: if (response_valid) begin
                    tx_data <= {7'b0, response};
                    cstate  <= C_TX;
                end

                C_TX: if (!tx_busy) begin
                    tx_go  <= 1'b1;
                    cstate <= C_OP;
                end
            endcase
        end
    end

endmodule
