"""Stream CRPs off the bankless single-tap SD-SEM board over UART.

    py fpga_sdsem/host/capture_sdsem.py --port COM5 --self-test
    py fpga_sdsem/host/capture_sdsem.py --port COM5 --n 20000 --repeat 10 --out caps/sdsem0
    py fpga_sdsem/host/capture_sdsem.py --port COM5 --n 200000 --signkey --out caps/sdsem0_sk

DESIGN: fpga_sdsem/rtl/sdsem_tdc_top.v (Chen LFSR in single-tap SD-SEM mode,
keyed by an on-die TDC sign-key -- bankless). Reference model: novel/chen2026.py
with rm_sem="state", out_taps=1.

PROTOCOL (dpuf_uart_ctrl.v, unchanged): opcode byte + 8 payload bytes, MSB-first.
    0xA5  load key (= G)   no reply          (bring-up mode only; USE_SIGNKEY=0)
    0x5A  evaluate         replies 1 byte, bit0 = response

TWO KEY MODES, matching sdsem_tdc_fpga_top.v's USE_SIGNKEY parameter:
  bring-up  (USE_SIGNKEY=0): G loaded over UART -- validates the datapath against
            novel/chen2026.py on real silicon. Pass --key (= G).
  sign-key  (USE_SIGNKEY=1, pass --signkey): G is measured on-die from the readout
            chain's stage signs at power-up. No key is loaded. This is the actual
            bankless PUF; record mask_leds alongside the capture (helper data).

STATELESS per challenge: S = M(G).C is rebuilt from C every query, so challenges
may be sent in any order, repeated, or dropped. --self-test checks that.
"""
import argparse
import numpy as np

OP_KEY = 0xA5
OP_EVAL = 0x5A
DEFAULT_KEY = 0xA5A5_5A5A_C3C3_3C3C     # sdsem_tdc_fpga_top.v KEY_DEFAULT (= G)
N = 64


def pack(bits):
    return int(sum(int(b) << p for p, b in enumerate(bits)))


def to_bytes(value):
    return bytes((value >> (8 * (7 - i))) & 0xFF for i in range(8))


class Board:
    def __init__(self, port, baud=115200, timeout=2.0):
        import serial
        self.ser = serial.Serial(port, baud, timeout=timeout)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

    def resync(self):
        self.ser.write(b"\x00" * 8)
        self.ser.flush()
        self.ser.reset_input_buffer()

    def load_key(self, key):
        self.ser.write(bytes([OP_KEY]) + to_bytes(key))
        self.ser.flush()

    def evaluate(self, challenge_int):
        self.ser.write(bytes([OP_EVAL]) + to_bytes(challenge_int))
        self.ser.flush()
        reply = self.ser.read(1)
        if len(reply) != 1:
            raise TimeoutError("no response byte -- check baud, wiring, and "
                               "that the bitstream is programmed")
        return reply[0] & 1

    def close(self):
        self.ser.close()


def self_test(board, count=200):
    rng = np.random.default_rng(7)
    ch = [pack(rng.integers(0, 2, size=N, dtype=np.uint8)) for _ in range(count)]
    first = [board.evaluate(c) for c in ch]
    second = {i: board.evaluate(ch[i]) for i in range(count)[::-1]}
    mism = sum(first[i] != second[i] for i in range(count))
    ones = sum(first)
    print(f"  determinism / random access : {count - mism}/{count} agree "
          f"({'PASS' if mism == 0 else 'FAIL'})")
    print(f"  uniformity                  : {100.0 * ones / count:.2f}% ones")
    if mism:
        print("  -> responses depend on query ORDER. For this stateless design "
              "that is a bug: check the LFSR loads S<-C per query and G is stable.")
    return mism == 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True, help="e.g. COM5 or /dev/ttyUSB1")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--n", type=int, default=20000, help="CRPs to capture")
    ap.add_argument("--out", default="board_sdsem.npz")
    ap.add_argument("--key", default=f"{DEFAULT_KEY:016x}",
                    help="64-bit G hex (bring-up). Ignored with --signkey.")
    ap.add_argument("--signkey", action="store_true",
                    help="bitstream measures its own G (USE_SIGNKEY=1); do NOT "
                         "load a key. Record mask_leds separately.")
    ap.add_argument("--rounds", type=int, default=4,
                    help="SD-SEM ROUNDS the bitstream was built with (recorded)")
    ap.add_argument("--seed", type=int, default=0, help="challenge RNG seed")
    ap.add_argument("--repeat", type=int, default=1,
                    help="passes over the SAME challenges; >1 needed for reliability")
    ap.add_argument("--self-test", action="store_true")
    args = ap.parse_args()

    key = int(args.key, 16)
    board = Board(args.port, args.baud)
    try:
        board.resync()
        if not args.signkey:
            board.load_key(key)
        else:
            print("sign-key mode: G measured on-die at boot; no UART key load.")

        if args.self_test:
            print(f"self-test on {args.port}"
                  + ("" if args.signkey else f", G {key:016x}"))
            raise SystemExit(0 if self_test(board) else 1)

        rng = np.random.default_rng(args.seed)
        challenges = rng.integers(0, 2, size=(args.n, N), dtype=np.uint8)
        responses = np.empty((args.repeat, args.n), dtype=np.uint8)
        for r in range(args.repeat):
            for i in range(args.n):
                responses[r, i] = board.evaluate(pack(challenges[i]))
                if (i + 1) % 1000 == 0:
                    print(f"  pass {r + 1}/{args.repeat}  {i + 1:>7,d}/{args.n:,d}  "
                          f"unif {100.0 * responses[r, :i + 1].mean():.2f}%", flush=True)
            if args.repeat > 1:
                print(f"  pass {r + 1} done: unif {100.0 * responses[r].mean():.2f}%",
                      flush=True)
    finally:
        board.close()

    out_resp = responses[0] if args.repeat == 1 else responses
    np.savez_compressed(args.out, challenges=challenges, responses=out_resp,
                        key=np.uint64(0 if args.signkey else key),
                        rounds=args.rounds, seed=args.seed,
                        signkey=np.uint8(1 if args.signkey else 0),
                        design="sdsem_tdc")
    print(f"\nwrote {args.out}: {args.n:,d} CRPs x {args.repeat} pass(es), "
          f"unif {100.0 * responses.mean():.2f}%, "
          f"{'sign-key' if args.signkey else f'G {key:016x}'}, ROUNDS={args.rounds}")
    if args.repeat > 1:
        maj = (responses.mean(axis=0) >= 0.5).astype(np.uint8)
        print(f"reliability (agreement with majority): "
              f"{100.0 * (responses == maj).mean():.2f}%  "
              f"BER {100.0 * (responses != maj).mean():.2f}%")


if __name__ == "__main__":
    main()
