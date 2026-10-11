// =============================================================================
// tdc_signkey.v -- TDC readout + bankless on-die sign-key (fpga_sdsem build)
//
// Same function as fpga_dpuf/rtl/tdc_signkey.v (validated by
// novel/signkey_tdc_sim.py): a TDC reading the arbiter chain resolves the analog
// delay to a multi-bit count, from which sign(w_i) is recovered as
// sign( count(e_{i-1}) - count(e_i) ). Full-entropy, 100% reproduction on the
// stable mask, ~54 usable bits.
//
// AREA OPTIMISATION vs the fpga_dpuf original (-64 FF)
// ----------------------------------------------------
// The original held the walking-1 probe e_i in a 64-bit shift register `probe`.
// e_i is simply (1 << stage), so it is DERIVED COMBINATIONALLY from the `stage`
// counter here instead of being stored. That removes the 64-bit probe register
// (-64 FF) for the cost of a 6->64 one-hot decoder (a few LUTs). The probe
// sequence is bit-identical, so the extracted key is unchanged.
// =============================================================================
`timescale 1ns / 1ps

module tdc_signkey #(
    parameter N       = 64,
    parameter TDC_W   = 6,        // TDC output bits (2^6 = 64 carry taps)
    parameter MARGIN  = 1         // |count diff| >= MARGIN  => stable stage
)(
    input  wire            clk,
    input  wire            rst_n,
    input  wire [TDC_W-1:0] tdc_count,
    input  wire            tdc_valid,
    input  wire            boot_start,
    output reg  [N-1:0]    probe_challenge,
    output reg             probe_launch,
    output reg  [N-1:0]    key,             // sign(w_i)
    output reg  [N-1:0]    mask,            // 1 = stable stage (public helper data)
    output reg             key_valid,
    output wire            busy
);
    localparam B_IDLE=3'd0, B_SET=3'd1, B_RACE=3'd2, B_LATCH=3'd3, B_STEP=3'd4, B_DONE=3'd5;
    reg [2:0]            st;
    reg [$clog2(N):0]    stage;
    reg signed [TDC_W:0] prev_cnt;          // count on the PREVIOUS probe (e_{i-1})
    reg                  have_prev;

    assign busy = (st != B_IDLE);

    // e_i = one-hot(stage), derived (not stored). -64 FF vs the shift-register
    // form. probe_challenge stays a combinational output, as before.
    always @(*) probe_challenge = ({{(N-1){1'b0}}, 1'b1} << stage);

    wire signed [TDC_W:0]   cur   = {1'b0, tdc_count};
    wire signed [TDC_W+1:0] diff  = cur - prev_cnt;      // ~ 2 w_i / t_lsb
    wire                    this_sign = (diff >= 0);
    wire [TDC_W+1:0]        adiff = this_sign ? diff : -diff;
    wire                    this_mask = (adiff >= MARGIN);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st <= B_IDLE; stage <= 0; prev_cnt <= 0; have_prev <= 1'b0;
            probe_launch <= 1'b0; key <= 0; mask <= 0; key_valid <= 1'b0;
        end else begin
            key_valid <= 1'b0;
            case (st)
                B_IDLE: if (boot_start) begin
                    stage <= 0; have_prev <= 1'b0; st <= B_SET;
                end
                B_SET:  begin probe_launch <= 1'b0; st <= B_RACE; end
                B_RACE: begin probe_launch <= 1'b1; if (tdc_valid) st <= B_LATCH; end
                B_LATCH: begin
                    probe_launch <= 1'b0;
                    if (have_prev) begin
                        // key[j] = sign(w_j) after N steps (shift register, cheap)
                        key  <= {this_sign, key[N-1:1]};
                        mask <= {this_mask, mask[N-1:1]};
                    end
                    prev_cnt  <= cur;
                    have_prev <= 1'b1;
                    st <= B_STEP;
                end
                B_STEP: begin
                    if (stage == N-1) st <= B_DONE;
                    else begin stage <= stage + 1'b1; st <= B_SET; end
                end
                B_DONE: begin key_valid <= 1'b1; st <= B_IDLE; end
                default: st <= B_IDLE;
            endcase
        end
    end
endmodule
