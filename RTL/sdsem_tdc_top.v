// =============================================================================
// sdsem_tdc_top.v -- Bankless single-tap SD-SEM PUF, keyed on-die at boot.
//
//   G   = majority-voted chain responses to 63 public boot challenges
//         (arb_signkey, at boot; replaces the TDC per-stage sign-key, whose key
//          came out IDENTICAL on two boards -- see arb_signkey.v header)
//   S   = M(G) . C                            (chen_clfsr, one clock per step)
//   H   = Dec(S[taps]) + 1                    (chen_sem, SD-SEM: H from STATE)
//   R   = A(S) ^ S[16] ^ S[48]                (SINGLE whitened tap)
//
// A(.) is the chain's sign bit (thermo[MID]). The same chain gives the key at
// boot and the response afterwards -- one analog primitive, two uses, no bank.
//
// USE_SIGNKEY : 1 = on-die key (the design); 0 = key (=G) loaded over UART
//               for datapath bring-up.
// =============================================================================
`timescale 1ns / 1ps

module sdsem_tdc_top #(
    parameter N           = 64,
    parameter ROUNDS      = 4,
    parameter H_BITS      = 4,
    parameter WHITEN      = 1,
    parameter TDCW        = 6,
    parameter USE_SIGNKEY = 1,
    parameter KEY_VOTES   = 15,         // races per key bit at boot (odd)
    parameter LOG_I       = 16,
    parameter LOG_J       = 48,
    parameter [6*H_BITS-1:0] H_TAPS = {6'd31, 6'd56, 6'd45, 6'd36}
)(
    input  wire         clk,
    input  wire         rst_n,
    input  wire [N-1:0] challenge_in,
    input  wire         start,
    input  wire         boot_start,
    input  wire [N-1:0] key_ext,        // = G (bring-up, USE_SIGNKEY=0)
    input  wire         key_ext_valid,
    output reg          response_out,
    output reg          response_valid,
    output wire [N-1:0] key_mask,
    output wire         busy
);
    localparam PRECHARGE = 4;
    localparam S_IDLE=3'd0, S_WARM=3'd1, S_HGET=3'd2, S_STEP=3'd3, S_OPRE=3'd4, S_ORACE=3'd5;

    // ---- shared dual-use arbiter/TDC chain ---------------------------------
    reg  [N-1:0]    chain_challenge;
    reg             chain_launch;
    wire            chain_response;          // sign bit = A(.)
    wire [TDCW-1:0] chain_count;             // multi-bit delay (no longer used for the key)
    wire            chain_valid;

    genvar ki;
    generate
        for (ki = 0; ki < 1; ki = ki + 1) begin : apuf
            (* DONT_TOUCH = "TRUE" *)
            tdc_apuf_chain #(.N(N), .LANE(ki), .TDCW(TDCW)) u_chain (
                .clk(clk), .rst_n(rst_n), .launch(chain_launch),
                .challenge(chain_challenge),
                .response(chain_response), .count(chain_count),
                .resp_valid(chain_valid));
        end
    endgenerate

    // ---- key source (= LFSR configuration G): on-die key OR UART -----------
    wire [N-1:0] active_key;
    wire [N-1:0] sk_probe_challenge;
    wire         sk_probe_launch, sk_busy;
    wire         key_ready;

    generate
    if (USE_SIGNKEY) begin : g_signkey
        wire [N-1:0] sk_mask;
        wire         sk_key_valid;
        arb_signkey #(.N(N), .TDC_W(TDCW), .VOTES(KEY_VOTES)) u_signkey (
            .clk(clk), .rst_n(rst_n),
            .tdc_count(chain_count), .tdc_valid(chain_valid),
            .race_bit(chain_response),
            .boot_start(boot_start),
            .probe_challenge(sk_probe_challenge), .probe_launch(sk_probe_launch),
            .key(active_key), .mask(sk_mask), .key_valid(sk_key_valid),
            .busy(sk_busy));
        assign key_mask = sk_mask;
        reg keyed;
        always @(posedge clk or negedge rst_n)
            if (!rst_n)           keyed <= 1'b0;
            else if (sk_key_valid) keyed <= 1'b1;
        assign key_ready = keyed;
    end else begin : g_extkey
        assign sk_busy = 1'b0; assign sk_probe_launch = 1'b0;
        assign sk_probe_challenge = {N{1'b0}};
        reg [N-1:0] key_reg;
        always @(posedge clk or negedge rst_n)
            if (!rst_n)             key_reg <= {N{1'b0}};
            else if (key_ext_valid) key_reg <= key_ext;
        assign active_key = key_reg;
        assign key_mask   = {N{1'b0}};
        assign key_ready  = 1'b1;
    end
    endgenerate

    // G is the low N-1 bits of the key (Chen's configuration vector width).
    wire [N-2:0] g = active_key[N-2:0];

    // ---- configurable LFSR -------------------------------------------------
    wire [N-1:0] s;
    reg [2:0]    st;
    reg          launch;
    wire lf_load = (st == S_IDLE) && start && key_ready && !sk_busy;
    wire lf_step = (st == S_WARM) || (st == S_STEP);

    chen_clfsr #(.N(N)) u_clfsr (
        .clk(clk), .rst_n(rst_n),
        .load(lf_load), .step(lf_step),
        .seed(challenge_in), .g(g), .s(s));

    // ---- SD-SEM step size (H from the STATE, not the arbiter) --------------
    wire [H_BITS:0] h_now;
    chen_sem #(.N(N), .H_BITS(H_BITS), .H_TAPS(H_TAPS)) u_sem (
        .s(s), .r_mid(chain_response), .r_end(1'b0),
        .mode_state(1'b1), .h(h_now));

    // ---- chain ownership: boot key extraction vs. output race --------------
    always @(*) begin
        if (sk_busy) begin chain_challenge = sk_probe_challenge; chain_launch = sk_probe_launch; end
        else         begin chain_challenge = s;                  chain_launch = launch; end
    end

    wire whitened = WHITEN ? (chain_response ^ s[LOG_I] ^ s[LOG_J])
                           : chain_response;

    // ---- SD-SEM sequencer (load / warm / evolve / output race) -------------
    reg [2:0]  pre_cnt;
    reg [H_BITS:0] step_cnt;
    reg [$clog2(ROUNDS+1)-1:0] round;

    assign busy = (st != S_IDLE) || sk_busy;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st <= S_IDLE; launch <= 1'b0; response_out <= 1'b0; response_valid <= 1'b0;
            round <= 0; pre_cnt <= 0; step_cnt <= 0;
        end else begin
            response_valid <= 1'b0;
            case (st)
                S_IDLE: begin
                    launch <= 1'b0;
                    if (start && key_ready && !sk_busy) begin
                        round <= 0; st <= S_WARM;
                    end
                end
                S_WARM: st <= S_HGET;
                S_HGET: begin
                    if (round == ROUNDS[$clog2(ROUNDS+1)-1:0]) begin
                        pre_cnt <= 0; launch <= 1'b0; st <= S_OPRE;
                    end else begin
                        step_cnt <= h_now; st <= S_STEP;
                    end
                end
                S_STEP: begin
                    if (step_cnt == 1) begin round <= round + 1'b1; st <= S_HGET; end
                    else                     step_cnt <= step_cnt - 1'b1;
                end
                S_OPRE: begin
                    if (pre_cnt == PRECHARGE - 1) begin launch <= 1'b1; st <= S_ORACE; end
                    else                                pre_cnt <= pre_cnt + 1'b1;
                end
                S_ORACE: begin
                    if (chain_valid) begin
                        response_out   <= whitened;
                        response_valid <= 1'b1;
                        launch         <= 1'b0;
                        st             <= S_IDLE;
                    end
                end
                default: st <= S_IDLE;
            endcase
        end
    end
endmodule
