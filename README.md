# Bankless Single-Tap Strong PUF

A strong physically unclonable function (PUF) for FPGAs that resists machine-learning
modeling attacks **without** paying for it in bit-error rate, and generates its own
secret key on-die from the same arbiter chain, so no separate weak-PUF "key bank" is
needed.

Implemented and measured on two Digilent Basys-3 boards (Xilinx Artix-7 XC7A35T).
This repository contains the RTL, the Vivado build scripts, the host-side capture
and analysis tools, and the Python models and attacks used in the accompanying
paper (submitted to IEEE ISCAS 2027).

---

## Key idea

In an arbiter-based PUF, to first order

```
BER ≈ p · I_A(f)
```

where `p` is the probability that one arbiter decision flips and `I_A` is how many
arbiter decisions the response is sensitive to. Modeling resistance, however,
depends on sensitivity to the *challenge* bits, which exact digital logic can
provide without adding any arbiter noise. So the design:

1. puts **all nonlinearity in exact, key-configured digital logic**: a configurable
   64-bit Galois LFSR whose step count is read from its own state
   (state-driven step evolution, **SD-SEM**), and
2. makes **one arbiter decision per response** (`I_A = 1`), whitened by two
   LFSR state bits.

The **key** that configures the LFSR is generated at power-up by the same arbiter
chain: it races 63 fixed public challenges 15 times each, and the majority result of
each race is one key bit. Nothing is stored and nothing is loaded.

```
challenge C ──► Galois LFSR M(G) ──► state S ──► single arbiter race A(S) ──► ⊕ S[16] ⊕ S[48] ──► response R
                    ▲                  │  ▲
                    │                  ▼  │ H steps
                    │               SD-SEM: H = Dec(S[31,56,45,36]) + 1
                    │
          key G = majority of A(P_i), i = 0..62   (63 public boot challenges, measured at power-up)
```

---

## Results (silicon, two boards, same bitstream)

| Metric | Board 1 | Board 2 |
|---|---|---|
| Bit-error rate (majority of 5 reads) | **0.180%** | **0.126%** |
| CRPs that ever flipped | 0.63% | 0.43% |
| Challenges screened / discarded | none | none |
| Uniformity (2M CRPs) | 50.00% | 50.00% |
| **Uniqueness** (2M CRPs, board 1 vs board 2) | **50.02%** | |
| Key correct after power cycle | 19 / 19 (99.64 to 99.84% CRP agreement) | |
| Logistic regression (1.8M train / 200k test) | 49.93% | 50.13% |
| MLP 64-64-64 (500k train / 200k test) | 50.21% | 50.16% |
| Structure-aware CMA-ES, 191 params (key + 128 delays) | 50.25% | 50.36% |
| Linked CMA-ES, 130 params (key derived from delays) | 49.74% | 50.47% |
| Linked reliability attack | 50.22% | 50.20% |

Implementation (Vivado 2019.1, after place-and-route, UART excluded):

| Block | LUT | FF |
|---|---|---|
| Delay chain + TDC arbiter | 132 | 35 |
| Galois LFSR | 72 | 64 |
| Key generator (incl. challenge ROM) | 166 | 92 |
| Control | 17 | 18 |
| **PUF core total** | **387** | **209** |

Timing is met at 100 MHz (4.96 ns slack).
One response takes about 46 clock cycles; key generation at boot takes about 0.2 ms.

All measurements are at room temperature and nominal voltage.

---

## Repository layout

```
fpga_sdsem/
├── rtl/
│   ├── sdsem_tdc_fpga_top.v   board top: UART <-> PUF, boot pulse, mask LEDs
│   ├── sdsem_tdc_top.v        PUF core: key source, LFSR, SD-SEM sequencer, readout
│   ├── arb_signkey.v          on-die key generator (63 public challenges x 15 votes)
│   ├── tdc_apuf_chain.v       64-stage delay chain + CARRY4 TDC arbiter
│   ├── apuf_pdl_stage.v       one switch stage (two symmetric LUT6 cells)
│   ├── chen_clfsr.v           key-configured Galois LFSR
│   ├── chen_sem.v             step-count generator (SD-SEM)
│   ├── dpuf_uart_ctrl.v       UART command protocol
│   ├── uart_rx.v, uart_tx.v
│   └── tdc_signkey.v          first-version key generator (superseded, see below)
├── xdc/
│   ├── sdsem.xdc              pins and timing exceptions
│   └── placement_sdsem.xdc    symmetric placement of the delay chain (required)
├── host/
│   ├── capture_sdsem.py       capture CRPs over UART (one challenge at a time)
│   ├── capture_sdsem_fast.py  batched, resumable capture (use for 2M CRPs)
│   ├── analyze_sdsem.py       uniformity, BER, uniqueness, LR attack
│   └── structural_sdsem.py    structure-aware CMA-ES attack on captured CRPs
├── sim/                       iverilog testbenches (not Vivado sources)
├── build_vivado.tcl           one-command area / full build
└── report_util.tcl
novel/                         Python models, ablations and attacks used in the paper
paper/                         ISCAS paper source (IEEEtran)
```

---

## Requirements

- **Hardware:** Digilent Basys-3 (xc7a35tcpg236-1). Two or more boards to measure
  uniqueness.
- **Vivado:** 2019.1 or newer.
- **Python:** 3.9+, with `numpy`, `pyserial`, `scikit-learn`, `cma`, `pandas`.

```bash
pip install numpy pyserial scikit-learn cma pandas
```

---

## Building the bitstream

From `fpga_sdsem/`:

```tcl
# area only (out-of-context synthesis of the PUF core)
vivado -mode batch -source build_vivado.tcl -tclargs area

# full board build -> bitstream
vivado -mode batch -source build_vivado.tcl -tclargs impl
```

Or create a project manually:

- **Top:** `sdsem_tdc_fpga_top`
- **Sources:** all files in `rtl/` (not `sim/`)
- **Constraints:** `xdc/sdsem.xdc`, then `xdc/placement_sdsem.xdc` with processing
  order set to LATE:
  `set_property PROCESSING_ORDER LATE [get_files placement_sdsem.xdc]`
- **Generics:** `USE_SIGNKEY=1 ROUNDS=4 H_BITS=4 WHITEN=1`
  (`USE_SIGNKEY=0` loads the key over UART instead, for datapath bring-up)

The placement file is required: without symmetric placement the two rails are
unbalanced and the PUF loses its uniqueness.

---

## Capturing data

Program the board, wait 2 to 3 seconds for the boot-time key generation, then from
`host/`:

```bash
# reliability set: 40k challenges, 5 reads each
py capture_sdsem_fast.py --port COM5 --signkey --n 40000 --repeat 5 --out caps/board1_rel

# attack set: 2M challenges, 1 read each (about 26 min per million)
py capture_sdsem_fast.py --port COM5 --signkey --n 2000000 --out caps/board1_2M

# power-cycle check: unplug, replug, wait, then
py capture_sdsem_fast.py --port COM5 --signkey --n 5000 --out caps/board1_reboot
```

Use the **same command** (same `--n`, default `--seed`) on every board and change
only `--out`, so all boards see identical challenges. If a capture stops, rerun it
with `--resume`.

UART protocol (115200 8N1): `0x5A` + 8 challenge bytes (MSB first) returns one byte
whose bit 0 is the response; `0xA5` + 8 bytes loads a key (only with `USE_SIGNKEY=0`).

---

## Analysis

```bash
# per-board metrics, and uniqueness when several boards are given
py analyze_sdsem.py caps/board1_rel.npz caps/board2_rel.npz
py analyze_sdsem.py caps/board1_2M.npz  caps/board2_2M.npz

# structure-aware attack (searches the 63 key bits and 128 stage delays)
py structural_sdsem.py caps/board1_rel.npz --train 20000 --test 20000
```

`structural_sdsem.py` needs `chen2026.py` and `chen_structural.py` from `novel/`.

---

## A note on the key generator

The first version (`tdc_signkey.v`) took each key bit as the sign of one stage's
delay, measured with the TDC on walking-one probes. On two boards with the same
bitstream it produced the **same key**, and uniqueness was only 46.98%: single-stage
delay differences are smaller than one TDC step and are dominated by the placement,
which every device shares. The current generator (`arb_signkey.v`) uses whole-chain
races on public challenges, which accumulate the random variation of all 64 stages;
uniqueness is now 50.02%. If you port this design to another FPGA, measure
uniqueness on at least two boards before trusting the key.

---

## Limitations

- Results are at room temperature and nominal voltage; BER and key reproduction over
  temperature and voltage have not been measured yet.
- Uniqueness comes from one pair of boards.
- The security argument is empirical (attacks plus the key-sensitivity analysis),
  not a cryptographic reduction.

---


This design builds on the keyed-LFSR PUF of Chen *et al.*, "A lightweight PUF with
immunity to machine learning attacks based on a weak-PUF-assisted reconfigurable
LFSR and temporal feedback," *ACM TODAES*, vol. 31, no. 5, 2026.

## License

Add a license before making the repository public (for example MIT for the code and
CC BY 4.0 for the paper and figures).
