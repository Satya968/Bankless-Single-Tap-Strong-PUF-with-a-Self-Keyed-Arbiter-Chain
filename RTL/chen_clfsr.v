// =============================================================================
// chen_clfsr.v  --  configurable Galois LFSR, Chen et al. Eq. (2) and (3)
//
// S^t = M . S^(t-1), with M's last column populated by the configuration
// vector G. Written out, Eq. (3) gives
//
//     s'[0] = s[n-1]
//     s'[i] = s[i-1] ^ (g[i-1] & s[n-1])          for i = 1 .. n-1
//
// which is the standard Galois form: shift up, and XOR the bit that fell off
// the top into every tapped position. One clock per step.
//
// WHY Eq. (3) AND NOT THE SEC. 3.3 WALK-THROUGH
// ---------------------------------------------
// The paper's worked example writes "s'_1 <- s_3 ^ s'_0" with s'_0 = s_3, which
// would make s'_1 identically zero, and indexes a nonexistent s_4 for n = 4. It
// is not self-consistent with Eq. (3). The equation is taken as authoritative,
// and novel/chen2026.py makes the same choice, so RTL and model agree by
// construction rather than by luck.
//
// M IS INVERTIBLE FOR EVERY G
// ---------------------------
// Row 0 is e_{n-1} and rows 1..n-1 are e_{i-1} + g_i.e_{n-1}, so the matrix is
// unit lower-triangular after a single row permutation: no choice of G can make
// it singular. That matters because a singular update would collapse state
// entropy and look like modelling resistance while being nothing of the kind.
// tb_chen_clfsr.v checks bijectivity numerically rather than trusting this.
// =============================================================================
`timescale 1ns / 1ps

module chen_clfsr #(
    parameter N = 64
)(
    input  wire         clk,
    input  wire         rst_n,
    input  wire         load,        // load `seed` into the state
    input  wire         step,        // advance one cycle
    input  wire [N-1:0] seed,
    input  wire [N-2:0] g,           // configuration; g[i-1] taps position i
    output reg  [N-1:0] s
);

    wire feedback = s[N-1];

    // Next state, Eq. (3). Kept as a wire so the SEM block can look ahead
    // without duplicating the update rule.
    wire [N-1:0] s_next;
    assign s_next[0] = feedback;

    genvar i;
    generate
        for (i = 1; i < N; i = i + 1) begin : tap
            assign s_next[i] = s[i-1] ^ (g[i-1] & feedback);
        end
    endgenerate

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)      s <= {N{1'b0}};
        else if (load)   s <= seed;
        else if (step)   s <= s_next;
    end

endmodule
