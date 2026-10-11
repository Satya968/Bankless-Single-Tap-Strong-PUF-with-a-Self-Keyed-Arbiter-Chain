"""Fast, resumable CRP capture for the bankless SD-SEM board (2M+ CRPs).

    py capture_sdsem_fast.py --port COM5 --signkey --n 2000000 --out caps/board0_2M
    py capture_sdsem_fast.py --port COM5 --signkey --n 2000000 --out caps/board0_2M --resume

Same protocol, same challenge generator (np.random.default_rng(seed)) and same
.npz format as capture_sdsem.py, so analyze_sdsem.py and structural_sdsem.py
read the output unchanged.

WHY IT IS FASTER
  capture_sdsem.py sends one challenge, waits for its byte, then sends the next,
  so every CRP pays a full USB round trip. Here the host writes BATCH challenges
  in one go and then reads BATCH reply bytes. This is safe with dpuf_uart_ctrl.v:
  the PUF answers in < 1 us after the 9th byte, and its reply byte (87 us) is
  sent long before the next 9-byte frame (781 us) has finished arriving, so the
  controller is always back in its opcode state before the next frame starts.
  Throughput is then set by the 115200-baud link: about 0.78 ms per CRP, i.e.
  roughly 26 minutes per million CRPs.

SAFETY
  * Every reply byte must be 0x00 or 0x01. A short read or any other value means
    a byte was lost, so the whole batch is discarded, the link is resynced, and
    the batch is sent again (the design is stateless per challenge, so repeating
    a challenge is harmless).
  * Progress is checkpointed to <out>.partial.npz every --checkpoint CRPs.
    After a crash or unplug, rerun the same command with --resume.

TWO BOARDS: run the identical command (same --n, --seed) on each board and only
change --out, so both boards see exactly the same challenges (needed for
uniqueness).
"""
import argparse
import os
import time

import numpy as np

OP_EVAL = 0x5A
N = 64


def frames_for(challenges):
    """(k, 64) uint8 bits -> one bytes object of k 9-byte EVALUATE frames.

    Bit p of the challenge has weight 2^p, sent MSB-first (as capture_sdsem.py).
    """
    weights = (np.uint64(1) << np.arange(N, dtype=np.uint64))
    vals = (challenges.astype(np.uint64) * weights).sum(axis=1, dtype=np.uint64)
    be = vals.astype(">u8").view(np.uint8).reshape(-1, 8)       # MSB-first bytes
    frames = np.empty((len(challenges), 9), dtype=np.uint8)
    frames[:, 0] = OP_EVAL
    frames[:, 1:] = be
    return frames.tobytes()


class Board:
    def __init__(self, port, baud, timeout):
        import serial
        self.ser = serial.Serial(port, baud, timeout=timeout)
        try:                                   # bigger OS buffers on Windows
            self.ser.set_buffer_size(rx_size=1 << 16, tx_size=1 << 16)
        except Exception:
            pass
        self.resync()

    def resync(self):
        time.sleep(0.05)
        self.ser.reset_input_buffer()
        self.ser.write(b"\x00" * 16)          # non-opcode bytes -> back to C_OP
        self.ser.flush()
        time.sleep(0.05)
        self.ser.reset_input_buffer()

    def evaluate_batch(self, challenges):
        k = len(challenges)
        self.ser.write(frames_for(challenges))
        self.ser.flush()
        reply = self.ser.read(k)
        if len(reply) != k:
            raise IOError(f"short read: {len(reply)}/{k} bytes")
        r = np.frombuffer(reply, dtype=np.uint8)
        if np.any(r > 1):
            raise IOError("unexpected reply byte (link out of sync)")
        return r.copy()

    def close(self):
        self.ser.close()


def save_partial(path, responses, p, i, args):
    """Checkpoint: all responses so far + where to resume (pass p, index i)."""
    np.savez_compressed(path, responses=responses, pass_idx=p, pos=i,
                        n=args.n, seed=args.seed, repeat=args.repeat)


def save_final(path, challenges, responses, args):
    np.savez_compressed(
        path, challenges=challenges,
        responses=(responses[0] if args.repeat == 1 else responses),
        key=np.uint64(0), rounds=args.rounds, seed=args.seed,
        signkey=np.uint8(1 if args.signkey else 0), design="sdsem_tdc")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--n", type=int, default=2_000_000)
    ap.add_argument("--repeat", type=int, default=1)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--rounds", type=int, default=4)
    ap.add_argument("--signkey", action="store_true")
    ap.add_argument("--batch", type=int, default=512,
                    help="challenges written per USB transfer")
    ap.add_argument("--checkpoint", type=int, default=100_000)
    ap.add_argument("--retries", type=int, default=5)
    ap.add_argument("--out", required=True, help="output path (.npz added)")
    ap.add_argument("--resume", action="store_true")
    a = ap.parse_args()
    if not a.signkey:
        raise SystemExit("this script is for the sign-key bitstream; pass --signkey")

    out = a.out[:-4] if a.out.endswith(".npz") else a.out
    partial = out + ".partial.npz"
    os.makedirs(os.path.dirname(out) or ".", exist_ok=True)

    rng = np.random.default_rng(a.seed)
    challenges = rng.integers(0, 2, size=(a.n, N), dtype=np.uint8)
    responses = np.zeros((a.repeat, a.n), dtype=np.uint8)

    # (pass, index) to start from
    start_pass, start_i = 0, 0
    if a.resume and os.path.exists(partial):
        d = np.load(partial)
        if (int(d["seed"]), int(d["n"]), int(d["repeat"])) != (a.seed, a.n, a.repeat):
            raise SystemExit("partial file was made with a different --n/--seed/"
                             "--repeat; use the same values or delete it")
        responses[:] = d["responses"]
        start_pass, start_i = int(d["pass_idx"]), int(d["pos"])
        if start_i >= a.n:
            start_pass, start_i = start_pass + 1, 0
        print(f"resuming at pass {start_pass + 1}, CRP {start_i:,}")

    board = Board(a.port, a.baud, timeout=max(2.0, a.batch * 0.002))
    print(f"capturing {a.n:,} CRPs x {a.repeat} pass(es) on {a.port}, "
          f"batch {a.batch}, seed {a.seed}")
    t0 = time.time()
    try:
        for p in range(start_pass, a.repeat):
            i = start_i if p == start_pass else 0
            next_ckpt = (i // a.checkpoint + 1) * a.checkpoint
            while i < a.n:
                j = min(i + a.batch, a.n)
                for attempt in range(a.retries + 1):
                    try:
                        responses[p, i:j] = board.evaluate_batch(challenges[i:j])
                        break
                    except IOError as e:
                        if attempt == a.retries:
                            raise
                        print(f"  batch {i:,}-{j:,}: {e}; resync and retry")
                        board.resync()
                i = j
                if i >= next_ckpt or i == a.n:
                    save_partial(partial, responses, p, i, a)
                    rate = (i - (start_i if p == start_pass else 0)) / \
                        max(time.time() - t0, 1e-9)
                    eta = (a.n - i + (a.repeat - p - 1) * a.n) / max(rate, 1e-9)
                    print(f"  pass {p + 1}/{a.repeat}  {i:>9,d}/{a.n:,d}  "
                          f"unif {100 * responses[p, :i].mean():.2f}%  "
                          f"{rate:,.0f} CRP/s  ETA {eta / 60:.0f} min", flush=True)
                    next_ckpt += a.checkpoint
            t0 = time.time()
    except KeyboardInterrupt:
        print("\ninterrupted -- progress is in", partial, "(rerun with --resume)")
        raise SystemExit(1)
    finally:
        board.close()

    save_final(out + ".npz", challenges, responses, a)
    if os.path.exists(partial):
        os.remove(partial)
    print(f"\nwrote {out}.npz: {a.n:,} CRPs x {a.repeat} pass(es), "
          f"unif {100 * responses.mean():.2f}%")
    if a.repeat > 1:
        maj = (responses.mean(axis=0) >= 0.5).astype(np.uint8)
        print(f"reliability {100 * (responses == maj).mean():.3f}%  "
              f"BER {100 * (responses != maj).mean():.3f}%")


if __name__ == "__main__":
    main()
