// =============================================================================
// arb_signkey.v -- bankless on-die key from the arbiter chain's own responses
//                  to 63 PUBLIC boot challenges, majority-voted.
//
// Drop-in replacement for tdc_signkey.v (same ports + one extra input,
// race_bit = the chain's sign bit / response).
//
// WHY THIS REPLACES THE TDC SIGN-KEY
//   Two-board silicon (11 Oct 2026) gave 46.98% inter-board response HD. With
//   a 1-bit key cliff (any wrong key bit -> ~50% agreement), that is only
//   reachable if both boards derived the SAME key: the per-stage TDC count
//   differences are dominated by layout-determined (systematic) skew and TDC
//   quantisation, not by random variation. The whole-chain race on a random
//   challenge, by contrast, IS device-unique on silicon (arbiter HD ~47% across
//   boards), so the key is now taken from that.
//
// KEY BIT i (i = 0..62) = majority over VOTES races of the chain's sign bit on
//   public challenge P_i (ROM below, numpy default_rng(0xB007), 63x64 bits).
//   63 <= 65 = APUF weight dimension, so every 63-bit key is reachable by some
//   delay vector: the linear APUF structure does not constrain the key.
// MASK BIT i = 1 if all VOTES races agreed (public helper data: says which bits
//   were unanimous, not their values). key[63] = mask[63] = 0.
//
// TIMING: each race is preceded by PRECHARGE cycles with launch low, so both
//   rails return to 0 and the done edge re-arms. 63 x 15 x ~24 cycles = ~0.23 ms
//   at 100 MHz.
// =============================================================================
`timescale 1ns / 1ps

module arb_signkey #(
    parameter N         = 64,
    parameter TDC_W     = 6,      // unused, kept for port compatibility
    parameter MARGIN    = 1,      // unused, kept for port compatibility
    parameter VOTES     = 15,     // odd
    parameter PRECHARGE = 16      // launch-low cycles before each race
)(
    input  wire             clk,
    input  wire             rst_n,
    input  wire [TDC_W-1:0] tdc_count,     // unused
    input  wire             tdc_valid,     // race done (synchronised)
    input  wire             race_bit,      // chain sign bit = response A(.)
    input  wire             boot_start,
    output reg  [N-1:0]     probe_challenge,
    output reg              probe_launch,
    output reg  [N-1:0]     key,
    output reg  [N-1:0]     mask,
    output reg              key_valid,
    output wire             busy
);
    localparam B_IDLE=3'd0, B_PRE=3'd1, B_RACE=3'd2, B_COUNT=3'd3, B_BIT=3'd4, B_DONE=3'd5;
    localparam VW = $clog2(VOTES+1);

    reg [2:0]    st;
    reg [5:0]    idx;           // key bit 0..62
    reg [VW-1:0] vote_n;        // races done for this bit
    reg [VW-1:0] ones;          // races that returned 1
    reg [4:0]    pre;

    assign busy = (st != B_IDLE);

    // ---- public boot-challenge ROM (63 x 64) --------------------------------
    reg [63:0] boot_chal;
    always @(*) begin
        case (idx)
            6'd0 : boot_chal = 64'hFC4D7B9436A22051;
            6'd1 : boot_chal = 64'hD0B29F4C8292B702;
            6'd2 : boot_chal = 64'h66496112662DC7E6;
            6'd3 : boot_chal = 64'h29A9B244E20DFCE9;
            6'd4 : boot_chal = 64'h823568ACBD092E14;
            6'd5 : boot_chal = 64'hF93EA04AB464D0B8;
            6'd6 : boot_chal = 64'h98C66490303608FA;
            6'd7 : boot_chal = 64'h046589528890B21E;
            6'd8 : boot_chal = 64'h3DF9BE5A8C8DD591;
            6'd9 : boot_chal = 64'hB0FDA58CBDD78116;
            6'd10: boot_chal = 64'hB492C78469EE229B;
            6'd11: boot_chal = 64'h3A3BA26636A33016;
            6'd12: boot_chal = 64'h099E132AE60654DA;
            6'd13: boot_chal = 64'h46D603EA30A1F32A;
            6'd14: boot_chal = 64'h0E833D15CA00A6FE;
            6'd15: boot_chal = 64'h7EB4F9556EC3CF93;
            6'd16: boot_chal = 64'hD6C82B987C4A3894;
            6'd17: boot_chal = 64'h6841716E0906B497;
            6'd18: boot_chal = 64'h70E2B4EE4EA17456;
            6'd19: boot_chal = 64'h0C05A7BE1BB6B5C2;
            6'd20: boot_chal = 64'h6230177AB66E8CED;
            6'd21: boot_chal = 64'h996FF96AEA67E333;
            6'd22: boot_chal = 64'hB165F0E61CF84979;
            6'd23: boot_chal = 64'hAA4A5C54612BC8A9;
            6'd24: boot_chal = 64'h59D23C74A665BFCF;
            6'd25: boot_chal = 64'hD73646B46A73784D;
            6'd26: boot_chal = 64'h062394464E610E68;
            6'd27: boot_chal = 64'h6FE91D0C4EF06C38;
            6'd28: boot_chal = 64'h082903DC815D39EA;
            6'd29: boot_chal = 64'h696068E3AAB4B125;
            6'd30: boot_chal = 64'hCE3DAF1D7DBAE3AB;
            6'd31: boot_chal = 64'hC727E36A1930A68D;
            6'd32: boot_chal = 64'h69AAFE7AFCC42C3C;
            6'd33: boot_chal = 64'h23CB6A090F387931;
            6'd34: boot_chal = 64'h8D62B33CE1242664;
            6'd35: boot_chal = 64'hD0204D68FD625243;
            6'd36: boot_chal = 64'hF09E107B750D51C6;
            6'd37: boot_chal = 64'hF4A267DF3A5DD011;
            6'd38: boot_chal = 64'h3C33BEBBA86F86FF;
            6'd39: boot_chal = 64'h780D2DC2B53D1641;
            6'd40: boot_chal = 64'h78FF98CA8C83595D;
            6'd41: boot_chal = 64'hCC5A115E84F49347;
            6'd42: boot_chal = 64'hF2A90682110AD4C9;
            6'd43: boot_chal = 64'h551C95FAF773E703;
            6'd44: boot_chal = 64'hBF18941B037CF582;
            6'd45: boot_chal = 64'h73D23A8227B90860;
            6'd46: boot_chal = 64'hF7FC8849F085A39D;
            6'd47: boot_chal = 64'h7D0CF14D8BE85619;
            6'd48: boot_chal = 64'h3D98247F61309268;
            6'd49: boot_chal = 64'h27F2C4EF2E3EB13F;
            6'd50: boot_chal = 64'h4A5AAC0A6F5D51E5;
            6'd51: boot_chal = 64'hB59306D5D7F30281;
            6'd52: boot_chal = 64'hEDAC86BFF0C12407;
            6'd53: boot_chal = 64'hEC583036479A3944;
            6'd54: boot_chal = 64'h7AB746AF83F99832;
            6'd55: boot_chal = 64'hC263347228D3989C;
            6'd56: boot_chal = 64'hE1B0EBFED08D6C55;
            6'd57: boot_chal = 64'h28DAFD047EA988BE;
            6'd58: boot_chal = 64'h53EC00E98F000C58;
            6'd59: boot_chal = 64'h2F865E1A1B1E1E72;
            6'd60: boot_chal = 64'h4CF09601C46E9963;
            6'd61: boot_chal = 64'hDF40348FFC7664E7;
            6'd62: boot_chal = 64'hA86138E03D7D9A64;
            default: boot_chal = 64'h0;
        endcase
    end
    always @(*) probe_challenge = boot_chal[N-1:0];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st <= B_IDLE; idx <= 6'd0; vote_n <= 0; ones <= 0; pre <= 0;
            probe_launch <= 1'b0; key <= {N{1'b0}}; mask <= {N{1'b0}};
            key_valid <= 1'b0;
        end else begin
            key_valid <= 1'b0;
            case (st)
                B_IDLE: if (boot_start) begin
                    idx <= 6'd0; vote_n <= 0; ones <= 0; pre <= 0;
                    probe_launch <= 1'b0; st <= B_PRE;
                end
                B_PRE: begin                       // rails discharge, done re-arms
                    probe_launch <= 1'b0;
                    if (pre == PRECHARGE-1) begin pre <= 0; st <= B_RACE; end
                    else pre <= pre + 1'b1;
                end
                B_RACE: begin
                    probe_launch <= 1'b1;
                    if (tdc_valid) st <= B_COUNT;
                end
                B_COUNT: begin
                    probe_launch <= 1'b0;
                    ones   <= ones + race_bit;
                    vote_n <= vote_n + 1'b1;
                    st     <= (vote_n == VOTES-1) ? B_BIT : B_PRE;
                end
                B_BIT: begin
                    key[idx]  <= (ones > (VOTES/2));
                    mask[idx] <= (ones == 0) || (ones == VOTES);
                    ones <= 0; vote_n <= 0;
                    if (idx == 6'd62) st <= B_DONE;
                    else begin idx <= idx + 1'b1; st <= B_PRE; end
                end
                B_DONE: begin key_valid <= 1'b1; st <= B_IDLE; end
                default: st <= B_IDLE;
            endcase
        end
    end
endmodule
