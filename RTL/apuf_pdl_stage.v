// =============================================================================
// apuf_pdl_stage.v  --  one 2x2 APUF switch stage as two hard LUT6 PDLs
//
// WHY TWO LUT6 AND NOT ONE LUT6_2
// -------------------------------
// The paper (Sec. VI-A) describes "a single 6-input, dual-output LUT" per
// stage, i.e. a LUT6_2 driving both rails from O6/O5. Two problems with that
// on real silicon + real tooling:
//
//   1. ASYMMETRY (the design-level problem, and the serious one).
//      Inside a 7-series LUT, O6 is the output of the full 6-input mux tree and
//      O5 comes off one level earlier. So O6 and O5 have DIFFERENT intrinsic
//      propagation delays -- systematically, on every stage, in the same
//      direction. An APUF measures the delay difference between its two rails;
//      a fixed per-stage offset repeated 64 times is a large deterministic bias
//      that swamps the process variation the PUF is supposed to extract. It
//      would push uniformity away from 50% and make every chip resolve the same
//      way -- destroying uniqueness, which is precisely the failure mode we
//      just spent the Python side fixing (delay compensation, apuf.py).
//      Two structurally identical LUT6s give both rails the same internal path.
//
//   2. PLACEMENT (the tooling problem).
//      Vivado transforms LUT6_2 into a LUT6 + LUT5 macro. The macro's internal
//      bel assignment is fixed, so `set_property BEL` on the /LUT6 and /LUT5
//      sub-cells is SILENTLY IGNORED -- the LOC then places the macro at the
//      site's default bel and every stage in a slice collides on A6LUT
//      (Vivado 12-2285). Two plain LUT6 cells are ordinary leaf cells that
//      accept BEL/LOC normally.
//
// Cost: 2 LUTs/stage instead of 1, so 128 LUTs per 64-stage chain rather than
// the paper's ~67. On an XC7A100T (63,400 LUTs) that is irrelevant, and the
// symmetry is worth far more than the area.
//
// PIN MAP (mirrored by LOCK_PINS in the XDC)
//   I0 = in0   I1 = in1   I4 = c   I2 = I3 = I5 = 0
//
// INIT, with index = {I5,I4,I3,I2,I1,I0} = 16*c + 2*in1 + in0:
//   out0 = c ? in1 : in0  ->  INIT[3:0]=1010b, INIT[19:16]=1100b -> ...000C000A
//   out1 = c ? in0 : in1  ->  INIT[3:0]=1100b, INIT[19:16]=1010b -> ...000A000C
// =============================================================================
`timescale 1ns / 1ps

// SELECTING THE PRIMITIVE -- `define USE_LUT62 to build the paper's version.
//
// This is a PREPROCESSOR switch, deliberately, not a parameter or a generate-if.
// Either of those wraps the instance in an extra named block, turning every
// cell path from
//     u_puf/apuf[0].u_chain/stage[0].u_stage/u_r0
// into
//     u_puf/apuf[0].u_chain/stage[0].g_lut6.u_stage/u_r0
// which silently invalidates every LOC/BEL line in the placement XDC -- and an
// XDC path that no longer matches is the exact failure mode the generator's
// no-`-quiet` rule exists to catch loudly. With `ifdef the hierarchy above the
// stage is byte-identical between the two builds; only the leaf names differ
// (u_r0 + u_r1  vs  u_lut), which the generator emits per style.
module apuf_pdl_stage (
    input  wire in0,
    input  wire in1,
    input  wire c,
    output wire out0,
    output wire out1
);

`ifdef USE_LUT62
    // ---- the paper's structure: ONE dual-output LUT drives both rails ------
    // Sec. VI-A "a single 6-input, dual-output LUT"; Table VIII's 134 LUTs for
    // 128 stages; Fig. 7's "two 6LUTs in ONE slice" (= two STAGES per slice).
    // I5 MUST be high or O6 reads the wrong half of INIT. Full derivation and
    // the rail-symmetry trade-off this accepts: apuf_pdl_stage_lut62.v.
    (* DONT_TOUCH = "TRUE" *)
    LUT6_2 #(.INIT(64'h000C_000A_000A_000C)) u_lut (
        .O6 (out0),
        .O5 (out1),
        .I0 (in0), .I1 (in1), .I2 (1'b0), .I3 (1'b0), .I4 (c), .I5 (1'b1)
    );
`else
    // rail 0: out0 = c ? in1 : in0
    (* DONT_TOUCH = "TRUE" *)
    LUT6 #(.INIT(64'h0000_0000_000C_000A)) u_r0 (
        .O  (out0),
        .I0 (in0), .I1 (in1), .I2 (1'b0), .I3 (1'b0), .I4 (c), .I5 (1'b0)
    );

    // rail 1: out1 = c ? in0 : in1   (same structure, complementary INIT)
    (* DONT_TOUCH = "TRUE" *)
    LUT6 #(.INIT(64'h0000_0000_000A_000C)) u_r1 (
        .O  (out1),
        .I0 (in0), .I1 (in1), .I2 (1'b0), .I3 (1'b0), .I4 (c), .I5 (1'b0)
    );
`endif

endmodule
