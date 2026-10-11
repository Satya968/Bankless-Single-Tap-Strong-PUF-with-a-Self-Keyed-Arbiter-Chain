// =============================================================================
// uart_tx.v  --  8N1 transmitter
//
// Extracted from the transmitter inside dpuf_uart_ctrl.v so that both
// controllers share one implementation. Logic unchanged; `send`/`busy` replaces
// the internal `tx_go`/`tx_busy` pair, which is the interface a polling soft
// core needs.
//
// CONTRACT: assert `send` for one clock with `data` valid while `busy` is low.
// A `send` raised while busy is IGNORED, not queued -- PicoBlaze polls `busy`
// before writing, so queueing would only hide a program bug behind a buffer.
// =============================================================================
`timescale 1ns / 1ps

module uart_tx #(
    parameter CLK_HZ = 100_000_000,
    parameter BAUD   = 115_200
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire [7:0] data,
    input  wire       send,
    output reg        tx,
    output reg        busy
);

    localparam integer DIV = CLK_HZ / BAUD;

    reg [15:0] cnt;
    reg [3:0]  bitno;
    reg [9:0]  sr;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // sr is reset explicitly: a shift register left without a reset
            // branch makes Vivado infer set+reset on the same element
            // (Synth 8-7137), a genuine sim/hardware mismatch risk. Idle line
            // is high, so all-ones is the right reset value.
            tx    <= 1'b1;
            busy  <= 1'b0;
            cnt   <= 16'd0;
            bitno <= 4'd0;
            sr    <= {10{1'b1}};
        end else if (!busy) begin
            tx <= 1'b1;
            if (send) begin
                sr    <= {1'b1, data, 1'b0};   // stop, data (LSB first), start
                busy  <= 1'b1;
                cnt   <= DIV[15:0];
                bitno <= 4'd0;
            end
        end else begin
            if (cnt != 0) cnt <= cnt - 16'd1;
            else begin
                tx    <= sr[0];
                sr    <= {1'b1, sr[9:1]};
                cnt   <= DIV[15:0];
                bitno <= bitno + 4'd1;
                if (bitno == 4'd10) busy <= 1'b0;
            end
        end
    end

endmodule
