"""Analyse a bankless SD-SEM capture (.npz from capture_sdsem.py).

    py fpga_sdsem/host/analyze_sdsem.py caps/sdsem0.npz

Self-contained (no external analysis lib). Reports uniformity, reliability/BER
(needs --repeat > 1 capture), per-challenge stability, and a generic-ML sanity
check (logistic regression on the parity features -- should flatline near 50%
without G). Uniqueness needs captures from >= 2 boards; pass several files.
"""
import argparse
import numpy as np

N = 64


def phi(challenges):
    """Standard APUF parity features, +/-1, shape (n, N+1) with bias column."""
    c = challenges.astype(np.int8)
    # Phi_i = prod_{k>=i} (1-2 c_k); compute as reversed cumulative product
    d = 1 - 2 * c
    cum = np.cumprod(d[:, ::-1], axis=1)[:, ::-1]
    return np.hstack([cum, np.ones((c.shape[0], 1))])


def uniformity(resp):
    r = resp.reshape(-1)
    return 100.0 * r.mean()


def reliability(resp3d):
    """resp3d: (repeat, n). Agreement with per-challenge majority vote."""
    maj = (resp3d.mean(axis=0) >= 0.5).astype(np.uint8)
    agree = (resp3d == maj).mean()
    return 100.0 * agree, 100.0 * (1 - agree)


def generic_ml(challenges, resp):
    """Logistic regression accuracy on parity features -- keyless attacker."""
    try:
        from sklearn.linear_model import LogisticRegression
    except Exception:
        return None
    X = phi(challenges)
    y = resp.astype(int)
    n = len(y)
    tr = int(0.7 * n)
    clf = LogisticRegression(max_iter=2000)
    clf.fit(X[:tr], y[:tr])
    return 100.0 * clf.score(X[tr:], y[tr:])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("files", nargs="+", help="one or more .npz captures")
    args = ap.parse_args()

    per_board_resp = []
    for f in args.files:
        d = np.load(f, allow_pickle=True)
        ch = d["challenges"]
        rp = d["responses"]
        signkey = int(d["signkey"]) if "signkey" in d else -1
        rounds = int(d["rounds"]) if "rounds" in d else -1
        print(f"\n=== {f}  (signkey={signkey}, ROUNDS={rounds}) ===")
        if rp.ndim == 2:                      # (repeat, n)
            unif = uniformity(rp)
            rel, ber = reliability(rp)
            print(f"  uniformity   : {unif:.3f}%")
            print(f"  reliability  : {rel:.3f}%   BER {ber:.3f}%")
            maj = (rp.mean(axis=0) >= 0.5).astype(np.uint8)
            flips = (rp != maj).mean(axis=0)
            print(f"  unstable CRPs (flip>0): {100.0 * (flips > 0).mean():.2f}%")
            r0 = maj
        else:                                 # (n,)
            print(f"  uniformity   : {uniformity(rp):.3f}%  (single pass, no BER)")
            r0 = rp
        ml = generic_ml(ch, r0)
        if ml is not None:
            print(f"  keyless logistic-regression accuracy: {ml:.2f}%  "
                  f"(near 50% = no leak without G)")
        per_board_resp.append(r0)

    if len(per_board_resp) >= 2:
        m = np.array(per_board_resp)
        # inter-board Hamming distance over the common challenge set
        k = min(len(x) for x in per_board_resp)
        m = m[:, :k]
        hds = []
        for i in range(len(m)):
            for j in range(i + 1, len(m)):
                hds.append((m[i] != m[j]).mean())
        print(f"\nuniqueness (mean inter-board HD): {100.0 * np.mean(hds):.3f}%  "
              f"(ideal 50%)")


if __name__ == "__main__":
    main()
