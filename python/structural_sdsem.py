"""Structure-aware attack on captured SD-SEM CRPs (the security metric).

    py fpga_sdsem/host/structural_sdsem.py caps/sdsem0.npz --train 20000 --test 20000

Runs the CMA-ES structure-aware attacker (novel/chen_structural.py) that jointly
searches the 63-bit key G AND the 128 arbiter delays (191 params) to reproduce
the captured (challenge, response) pairs. A design whose security is real key
entropy flatlines this attacker near 50% on the TEST set; the same attacker
breaks Markov-CBO / SOI / DFO at 95-99%.

MODEL MUST MATCH THE BITSTREAM. The attacker's forward model reads only PUBLIC
config; set it to what the board was built with:
  rm_sem = "state", out_taps = 1, ROUNDS (from the .npz), H_BITS (default 4).
Two details that make the model match sdsem_tdc_top.v exactly:
  * the readout is the chain-END TDC bit (~r_end, depth n), NOT the depth-n/2
    tap that out_taps=1 defaults to -- so out_tap_depths is pinned to [n];
  * whitening taps s[16] ^ s[48] (log_i/log_j) are the Chen defaults.
If the model does not match, a ~50% result is meaningless (the model could not
reproduce the design even given the key), so keep these in sync with the RTL.

Positive control (that the design IS learnable GIVEN the key -- the conditional
break) is in novel/KEYLEN_RESULTS.md (~90%). This script is the negative side:
WITHOUT the key, the structure-aware search must fail.
"""
import argparse
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(ROOT / "novel"))

from chen2026 import Chen2026PUF          # noqa: E402
from chen_structural import attack        # noqa: E402


def load(path):
    d = np.load(path, allow_pickle=True)
    chal = d["challenges"]
    resp = d["responses"]
    if resp.ndim == 1:
        resp = resp[None, :]
    # Enrolled response = per-challenge majority across passes.
    r = (resp.mean(axis=0) >= 0.5).astype(np.uint8) if resp.shape[0] > 1 \
        else resp[0].astype(np.uint8)
    rounds = int(d["rounds"]) if "rounds" in d else 4
    return chal, r, rounds, resp.shape[0]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("npz")
    ap.add_argument("--train", type=int, default=20000)
    ap.add_argument("--test", type=int, default=20000)
    ap.add_argument("--h-bits", type=int, default=4, help="must match the bitstream H_BITS")
    ap.add_argument("--max-iter", type=int, default=1500)
    ap.add_argument("--time-budget", type=float, default=1400.0)
    ap.add_argument("--seed", type=int, default=0)
    a = ap.parse_args()

    chal, resp, rounds, passes = load(a.npz)
    span = a.train + a.test
    if len(chal) < span:
        raise SystemExit(f"{a.npz} has {len(chal):,} CRPs; need {span:,}")

    n = chal.shape[1]
    # Config-only PUF: its random g/d0/d1 are NEVER used -- the attack searches
    # for the silicon's. Only public config is read.
    puf = Chen2026PUF(n_stages=n, seed=a.seed, rm_sem="state",
                      out_taps=1, h_bits=a.h_bits, rounds=rounds)
    puf.out_tap_depths = np.array([n])        # RTL TDC taps the chain END (r_end)

    print(f"structure-aware attack on {a.npz}")
    print(f"  design: SD-SEM state, out_taps=1@depth{n}, H_BITS={a.h_bits}, "
          f"ROUNDS={rounds}, {passes} pass(es)")
    print(f"  attacker searches G(63) + delays(128) = 191 params jointly\n")
    res = attack(puf, chal, resp, a.train, a.test, a.max_iter, a.seed, a.time_budget)
    print(f"  generations   : {res['generations']}")
    print(f"  train accuracy : {100*res['train_acc']:.2f}%")
    print(f"  TEST accuracy  : {100*res['test_acc']:.2f}%   "
          f"({'HOLDS (~50%)' if res['test_acc'] < 0.55 else 'LEAK -- investigate'})")
    print(f"  wall time      : {res['seconds']} s")
    print("\nInterpretation: TEST ~50% => the structure-aware attacker cannot "
          "recover the key; security is the key entropy (see KEYLEN_RESULTS.md). "
          "TEST >> 50% would mean the key leaks structurally -- check ROUNDS>=4.")


if __name__ == "__main__":
    main()
