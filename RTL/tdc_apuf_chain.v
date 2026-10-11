// =============================================================================
// tdc_apuf_chain.v -- N-stage PDL switch chain with a DUAL-USE TDC readout.
//
// Replaces the 1-bit arbiter of apuf_chain.v with a carry-chain time-to-digital
// converter (TDC) that measures the analog delay difference between the two
// rails as a multi-bit count. This is the readout that makes the bankless
// sign-key work (novel/signkey_tdc_sim.py): sign(w_i) is recovered from the
// DIFFERENCE of adjacent-probe counts, which needs the analog magnitude a 1-bit
// arbiter cannot give.
//
// DUAL-USE -- one analog primitive, two uses, so area stays near Chen's range:
//   * response  = thermo[MID] : did the carry reach the midpoint = which rail
//                 won = exactly the old arbiter bit. Cheap (one tap).
//   * count     = popcount(thermo) : the full delay measurement, used ONLY at
//                 boot for the sign-key. The popcount adder is boot-amortised.
//
// SYNTHESIS vs SIMULATION
//   Vivado build: real CARRY4 delay line (define nothing). iverilog / behavioural
//   sim: `define TDC_SIM to get a zero-delay stand-in whose response is the same
//   parity function apuf_chain_stub.v uses, so the datapath testbench still runs.
//   The TDC's analog FUNCTION is silicon-only (rails have zero delay in RTL) --
//   the standard FPGA-TDC caveat; place-and-route tunes the carry-tap LSB.
// =============================================================================
`timescale 1ns / 1ps

module tdc_apuf_chain #(
    parameter N     = 64,
    parameter LANE  = 0,
    parameter TAPS  = 32,          // carry taps (coarse TDC is enough: sim shows
    parameter TDCW  = 6,           // t_lsb=1.0 sigma_w keeps >=48 stable bits)
    parameter MID   = TAPS / 2
)(
    input  wire            clk,
    input  wire            rst_n,
    input  wire            launch,
    input  wire [N-1:0]    challenge,
    output wire            response,       // sign bit = which rail won (arbiter eq.)
    output wire [TDCW-1:0] count,          // multi-bit delay (for boot sign-key)
    output wire            resp_valid
);

    (* DONT_TOUCH = "TRUE" *) wire [N:0] path0;
    (* DONT_TOUCH = "TRUE" *) wire [N:0] path1;
    assign path0[0] = launch;
    assign path1[0] = launch;

    genvar i;
    generate
        for (i = 0; i < N; i = i + 1) begin : stage
            (* DONT_TOUCH = "TRUE" *)
            apuf_pdl_stage u_stage (
                .in0 (path0[i]), .in1 (path1[i]), .c (challenge[i]),
                .out0(path0[i+1]), .out1(path1[i+1]));
        end
    endgenerate

    // ---- TDC: rail1 starts the carry, rail0 latches it ---------------------
`ifdef TDC_SIM
    // Zero-delay stand-in for elaboration / datapath sim. The response is the
    // same masked parity apuf_chain_stub.v uses (so golden vectors match); the
    // count is a deterministic function of it (no analog meaning in sim).
    function [63:0] mask_of; input integer lane;
        mask_of = (lane==0)? 64'h0F1E2D3C4B5A6978 :
                  (lane==1)? 64'hFEDCBA9876543210 : 64'h0123456789ABCDEF;
    endfunction
    wire [63:0] lmask = mask_of(LANE);
    wire resp_c = launch ? (^(challenge & lmask[N-1:0])) : 1'b0;
    reg [TDCW-1:0] cnt_s;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) cnt_s <= 0;
        else        cnt_s <= resp_c ? {1'b1,{(TDCW-1){1'b0}}} : {TDCW{1'b0}};
    assign count    = cnt_s;
    assign response = resp_c;
    wire raw_done = launch;
`else
    // Real carry-chain TDC (Xilinx 7-series). rail1 drives CYINIT of a CARRY4
    // string with S=1111 so the carry propagates; rail0's edge latches the
    // thermometer taps. How far the carry got = the delay difference.
    (* DONT_TOUCH = "TRUE" *) wire [TAPS-1:0] cout;
    genvar g;
    generate
        for (g = 0; g < TAPS/4; g = g + 1) begin : carry
            CARRY4 u_c4 (
                .CO    (cout[4*g +: 4]),
                .O     (),
                .CI    (g == 0 ? 1'b0     : cout[4*g-1]),
                .CYINIT(g == 0 ? path1[N] : 1'b0),
                .DI    (4'b0000),
                .S     (4'b1111));
        end
    endgenerate
    // Thermometer latch: clocked by rail0's edge (async, like the arbiter).
    (* ASYNC_REG = "TRUE", DONT_TOUCH = "TRUE" *) reg [TAPS-1:0] thermo;
    always @(posedge path0[N] or negedge rst_n)
        if (!rst_n) thermo <= {TAPS{1'b0}};
        else        thermo <= cout;
    // response = did the carry reach the midpoint (one tap, cheap)
    assign response = thermo[MID];
    // count = popcount(thermo): combinational sum, then registered (boot only).
    integer j; reg [TDCW-1:0] pc_c;
    always @(*) begin
        pc_c = {TDCW{1'b0}};
        for (j = 0; j < TAPS; j = j + 1) pc_c = pc_c + thermo[j];
    end
    reg [TDCW-1:0] pc;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) pc <= 0; else pc <= pc_c;
    assign count = pc;
    wire raw_done = path0[N] | path1[N];
`endif

    // ---- done flag + 2-FF synchroniser (same as apuf_chain) ----------------
    reg [2:0] done_sync;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) done_sync <= 3'b000;
        else        done_sync <= {done_sync[1:0], raw_done};
    end
    assign resp_valid = done_sync[1] & ~done_sync[2];

endmodule
