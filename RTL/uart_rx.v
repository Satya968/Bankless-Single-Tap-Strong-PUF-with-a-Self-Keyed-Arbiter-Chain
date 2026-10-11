// =============================================================================
// uart_rx.v  --  8N1 receiver with a one-byte holding register
//
// Extracted verbatim from the receiver inside dpuf_uart_ctrl.v so that the
// hand-written controller and the PicoBlaze controller share ONE implementation
// rather than two copies that can drift apart. The sampling logic is unchanged;
// only the `present`/`ack` handshake is new, because a soft core polls a status
// bit instead of reacting to a one-cycle pulse.
//
// HANDSHAKE
//   `present` goes high when a byte lands and STAYS high until `ack`. PicoBlaze
//   runs at 2 clocks per instruction and cannot see a single-cycle strobe, so a
//   pulse-only interface would drop bytes at random depending on where in its
//   instruction stream the byte arrived -- a fault that looks like a flaky
//   cable and is not.
//
// OVERRUN IS REPORTED, NOT HIDDEN
//   If a second byte completes while `present` is still set, the new byte is
//   dropped and `overrun` latches. Silently overwriting would corrupt a
//   challenge in a way the host cannot detect: the CRP would simply be wrong,
//   and wrong CRPs are indistinguishable from a PUF with poor reliability --
//   which is exactly the measurement this hardware exists to make.
// =============================================================================
`timescale 1ns / 1ps

module uart_rx #(
    parameter CLK_HZ = 100_000_000,
    parameter BAUD   = 115_200
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       rx,
    output reg  [7:0] data,
    output reg        present,     // a byte is waiting; cleared by `ack`
    input  wire       ack,         // 1-cycle: consumer has taken `data`
    output reg        overrun      // sticky: a byte arrived while one waited
);

    localparam integer DIV  = CLK_HZ / BAUD;
    localparam integer HALF = DIV / 2;

    reg [1:0]  sync;
    reg [15:0] cnt;
    reg [3:0]  bitno;
    reg [7:0]  sr;
    reg        busy;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sync    <= 2'b11;
            cnt     <= 16'd0;
            bitno   <= 4'd0;
            sr      <= 8'd0;
            busy    <= 1'b0;
            data    <= 8'd0;
            present <= 1'b0;
            overrun <= 1'b0;
        end else begin
            sync <= {sync[0], rx};

            if (ack) present <= 1'b0;

            if (!busy) begin
                if (sync[1] == 1'b0) begin           // start bit
                    busy  <= 1'b1;
                    cnt   <= HALF[15:0];             // sample mid-bit
                    bitno <= 4'd0;
                end
            end else begin
                if (cnt != 0) cnt <= cnt - 16'd1;
                else begin
                    cnt <= DIV[15:0];
                    if (bitno == 4'd0) begin
                        bitno <= 4'd1;               // consume the start bit
                    end else if (bitno <= 4'd8) begin
                        sr    <= {sync[1], sr[7:1]}; // LSB first
                        bitno <= bitno + 4'd1;
                    end else begin
                        busy <= 1'b0;
                        // `present && !ack` means the consumer has not taken
                        // the previous byte yet.
                        if (present && !ack) begin
                            overrun <= 1'b1;
                        end else begin
                            data    <= sr;
                            present <= 1'b1;
                        end
                    end
                end
            end
        end
    end

endmodule
