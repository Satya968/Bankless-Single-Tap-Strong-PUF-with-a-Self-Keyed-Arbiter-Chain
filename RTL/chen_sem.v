// =============================================================================
// chen_sem.v  --  step-size generator: RM-SEM (the paper) and SD-SEM (ours)
//
// This is the one block the architecture claim is about. Everything else in
// fpga_chen/ is Chen et al. reimplemented; this module is where the two designs
// differ, and it is deliberately the ONLY place they differ, so that a
// utilisation report can attribute the delta to it and to nothing else.
//
//   mode_state = 0   RM-SEM, Chen et al. Eq. (4):  H = Dec(r_mid, r_end) + 1
//                    Two bits from two APUF arbiters. H in {1,2,3,4}.
//   mode_state = 1   SD-SEM:                       H = Dec(S[taps]) + 1
//                    H_BITS bits from the LFSR state. H in {1 .. 2^H_BITS}.
//
// BOTH ARE PRESENT IN ONE BITSTREAM ON PURPOSE
// --------------------------------------------
// The comparison being published is "SD-SEM versus RM-SEM", and a
// reliability/uniformity difference measured across two bitstreams, or worse
// two boards, is confounded by placement and by silicon. One bitstream with a
// mode bit means the two configurations share the same chain, the same
// arbiters, the same routing and the same die. The cost is that the utilisation
// report then contains BOTH, so the area claim cannot be read off this build --
// it needs two synthesis-only runs with the mode bit tied off, and the README
// says so.
//
// WHY THE STATE IS A LEGITIMATE SOURCE FOR H, WHICH IS THE WHOLE ARGUMENT
// -----------------------------------------------------------------------
// S = M.C and M is built from G, the weak-PUF sponge output. So H is still
// key-derived and still challenge-dependent: an attacker without G cannot
// compute it, which is the property RM-SEM was there to provide. What it no
// longer contains is an arbiter output, and that asymmetry is the point:
//
//   RM-SEM   a flipped arbiter changes H, which changes the number of LFSR
//            steps, which desynchronises every later state. Chen et al.
//            Sec. 5.3 calls this "catastrophic error propagation" and it is
//            why 20-79% of challenges have to be screened away.
//   SD-SEM   H is a function of flip-flops. It cannot flip. An arbiter error
//            costs exactly one wrong output bit and cannot propagate.
//
// AND WIDENING IS FREE HERE, WHERE IT WAS NOT FREE FOR CHEN
// ---------------------------------------------------------
// Chen's H is 2 bits because each bit costs an arbiter -- silicon, and noise.
// Read from the state, each bit costs one wire into a multiplexer. H_BITS = 4
// gives a reachable exponent set of {3..33}, 31 affine images of C, where the
// paper has {3..9}, 7 images. The structural attack measured that mixture as
// worth 13.35 pp (1-way 74.87% -> 7-way 61.52%), so it is load-bearing, and it
// is the one lever on the measured frontier that costs nothing.
//
// THE TAPS ARE PUBLIC, AND THAT IS NOT AN OVERSIGHT
// -------------------------------------------------
// H_TAPS is a compile-time constant, so it is in the bitstream and a
// determined attacker reads it out. It does not need to be secret: knowing
// WHICH state bits are read is useless without knowing the state, and the
// state needs G. Making the taps secret would mean storing them, which means
// more weak-PUF bits, which is the cost this design exists to avoid.
// Kerckhoffs applies -- the taps are published in the paper too.
// =============================================================================
`timescale 1ns / 1ps

module chen_sem #(
    parameter N      = 64,
    parameter H_BITS = 4,
    // Tap positions, 6 bits each, FIRST TAP IN THE HIGH BITS. This ordering
    // matches novel/chen2026.py `_step_size_state`, which shifts left as it
    // walks h_taps_wide in order, so h_taps_wide[0] ends up the MSB of H-1.
    // Defaults are the h_bits=4 taps that model emits for n=64
    // (numpy default_rng(0x5EED+64).choice(64, 4, replace=False) = 31,56,45,36).
    // sim/gen_golden_chen.py writes them into golden_sem.txt and the testbench
    // compares against that, so a mismatch here is caught rather than assumed.
    parameter [6*H_BITS-1:0] H_TAPS = {6'd31, 6'd56, 6'd45, 6'd36}
)(
    input  wire [N-1:0]      s,
    input  wire              r_mid,
    input  wire              r_end,
    input  wire              mode_state,   // 1 = SD-SEM, 0 = RM-SEM
    output wire [H_BITS:0]   h             // 1 .. 2^H_BITS
);

    // ---- SD-SEM: gather the tapped state bits, first tap to the MSB --------
    wire [H_BITS-1:0] sel;
    genvar k;
    generate
        for (k = 0; k < H_BITS; k = k + 1) begin : tapsel
            // Tap k (k=0 is the first, i.e. the MSB of `sel`) lives in
            // H_TAPS[6*(H_BITS-1-k) +: 6].
            localparam integer POS = 6 * (H_BITS - 1 - k);
            wire [5:0] idx = H_TAPS[POS +: 6];
            assign sel[H_BITS-1-k] = s[idx];
        end
    endgenerate

    // ---- RM-SEM: the paper's two arbiter bits ------------------------------
    wire [1:0] rm = {r_mid, r_end};

    // Zero-extend the 2-bit RM value to the SD width so one adder serves both.
    // H_BITS >= 2 is required for this to be well-formed; H_BITS = 1 would
    // truncate the paper mode to {1,2} and silently stop being the paper.
    wire [H_BITS-1:0] dec = mode_state ? sel
                                       : {{(H_BITS-2){1'b0}}, rm};

    assign h = {1'b0, dec} + 1'b1;

    // ---- elaboration guards ------------------------------------------------
    // H_TAPS MUST BE OVERRIDDEN WHENEVER H_BITS IS.
    // Its width is 6*H_BITS, so raising H_BITS without supplying new taps
    // zero-extends the default on the left and the FIRST tap silently becomes
    // index 0. At H_BITS = 5 that yields taps {0,31,56,45,36} where the model
    // uses {3,44,36,31,55}: a design that elaborates, simulates, and produces a
    // uniform, unique, perfectly reliable response stream that matches no model
    // and no other build. The distinctness check below does NOT catch that
    // particular case (0 is distinct from the rest), which is exactly why this
    // is stated here rather than left to a test.
    //
    // sim/run_sim.sh passes a matching H_TAPS for every H_BITS it sweeps, and
    // sim/tb_chen_sem.v compares the taps against the model's as TEXT.
    // synthesis translate_off
    integer gi, gj;
    reg [5:0] ti, tj;
    initial begin
        if (H_BITS < 2) begin
            $display({"chen_sem: H_BITS must be >= 2 -- RM-SEM needs two bits ",
                      "and would be silently truncated"});
            $finish;
        end
        for (gi = 0; gi < H_BITS; gi = gi + 1) begin
            ti = H_TAPS[6*(H_BITS-1-gi) +: 6];
            if (ti >= N) begin
                $display("chen_sem: tap %0d is index %0d, outside 0..%0d",
                         gi, ti, N-1);
                $finish;
            end
            // A duplicated tap halves the reachable exponent set without
            // changing anything observable: H stays in range, the responses
            // stay uniform, and the mixture the security argument rests on is
            // quietly narrower than the paper claims.
            for (gj = gi + 1; gj < H_BITS; gj = gj + 1) begin
                tj = H_TAPS[6*(H_BITS-1-gj) +: 6];
                if (ti == tj) begin
                    $display({"chen_sem: taps %0d and %0d are both index %0d ",
                              "-- the exponent mixture is narrower than H_BITS ",
                              "implies"}, gi, gj, ti);
                    $finish;
                end
            end
        end
    end
    // synthesis translate_on

endmodule
