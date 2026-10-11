// =============================================================================
// sdsem_tdc_fpga_top.v -- board top for the bankless single-tap SD-SEM PUF.
// UART <-> sdsem_tdc_top. Same wire protocol as the DKD builds (capture_sdsem.py
// drives it). USE_SIGNKEY: 1 = on-die TDC sign-key (the design), 0 = UART-key
// (=G) bring-up. mask_leds exposes the public stable-stage mask.
//
// KEY_DEFAULT here is G for the bring-up build; in sign-key mode it is unused.
// =============================================================================
`timescale 1ns / 1ps

module sdsem_tdc_fpga_top #(
    parameter N              = 64,
    parameter ROUNDS         = 4,   // >=4 for key security (novel/KEYLEN_RESULTS.md)
    parameter H_BITS         = 4,
    parameter WHITEN         = 1,
    parameter USE_SIGNKEY    = 1,
    parameter [N-1:0] KEY_DEFAULT = 64'hA5A5_5A5A_C3C3_3C3C,
    parameter RST_ACTIVE_LOW = 0
)(
    input  wire clk,
    input  wire rst,
    input  wire uart_rx,
    output wire uart_tx,
    output wire [7:0] mask_leds
);
    wire rst_n = RST_ACTIVE_LOW ? rst : ~rst;
    wire [N-1:0] challenge, uart_key;
    wire         start, response, response_valid, busy;

    dpuf_uart_ctrl #(.CLK_HZ(100_000_000), .BAUD(115_200), .N(N),
                     .KEY_DEFAULT(KEY_DEFAULT)) u_uart (
        .clk(clk), .rst_n(rst_n), .uart_rx(uart_rx), .uart_tx(uart_tx),
        .challenge(challenge), .key(uart_key), .start(start),
        .response(response), .response_valid(response_valid), .busy(busy));

    // power-on pulse to trigger the boot sign-key measurement
    reg [3:0] bc = 0; reg bp = 0;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin bc<=0; bp<=0; end
        else if (bc < 4'd8) begin bc<=bc+1'b1; bp<=(bc==4'd4); end
        else bp<=1'b0;

    wire [N-1:0] mask;
    sdsem_tdc_top #(.N(N), .ROUNDS(ROUNDS), .H_BITS(H_BITS),
                    .WHITEN(WHITEN), .USE_SIGNKEY(USE_SIGNKEY)) u_puf (
        .clk(clk), .rst_n(rst_n), .challenge_in(challenge), .start(start),
        .boot_start(bp), .key_ext(uart_key), .key_ext_valid(start),
        .response_out(response), .response_valid(response_valid),
        .key_mask(mask), .busy(busy));

    assign mask_leds = mask[7:0];
endmodule
