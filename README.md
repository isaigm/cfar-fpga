# Configurable CFAR Detector (VHDL)

A parameterizable, streaming **Constant False Alarm Rate (CFAR)** radar target
detector written in VHDL, verified bit-exact against a fixed-point Python golden
model with [cocotb](https://www.cocotb.org/), and synthesized with timing closure
on a Xilinx Artix-7 (Basys 3).

The estimator variant (**CA / GO / SO**, with **OS** planned) is selected at
elaboration time through a single generic, so one design covers the whole family
without paying for the modes you don't use.

---

## What is CFAR?

A radar range profile is an array of **power samples**: each sample is the energy
of the echo returned from one distance bin (the *index* encodes distance via echo
time-of-flight, the *value* encodes reflected energy). A target shows up as a peak.

A fixed detection threshold does not work, because the noise/clutter floor varies
along the profile: set it low and clutter triggers false alarms; set it high and
weak targets are missed. **CFAR sets the threshold adaptively, per cell**, to keep
the false-alarm rate constant regardless of the local noise level.

For each **Cell Under Test (CUT)**, CFAR:

1. Skips a few **guard cells** on each side (so target energy leaking into
   neighbours does not corrupt the noise estimate).
2. Estimates the local noise from the surrounding **reference cells**.
3. Computes `threshold = noise_estimate x alpha`.
4. Declares a detection if `CUT > threshold`.

The CUT is evaluated at the **centre** of a sliding window, so the detector has a
fixed fill latency but a throughput of one sample per clock.

```
[ ref x N/2 ][ guard ][ CUT ][ guard ][ ref x N/2 ]
```

---

## Variants

All variants share the sliding window, guard handling, threshold multiply and
comparison. **The only thing that changes is how the reference cells are combined
into the noise estimate:**

| Variant | Noise estimate | Best for | Weakness |
|---------|----------------|----------|----------|
| **CA** (Cell-Averaging) | mean of all `N` reference cells | homogeneous clutter (statistically optimal) | clutter edges; masks on multiple targets |
| **GO** (Greatest-Of) | `max(mean_left, mean_right)` | clutter edges (keeps false alarms bounded at transitions) | masks a target sitting on one reference side |
| **SO** (Smallest-Of) | `min(mean_left, mean_right)` | closely spaced multiple targets | false-alarm blow-up at clutter edges |
| **OS** (Ordered-Statistic) *(planned)* | k-th value of the sorted reference cells | non-homogeneous clutter + multiple targets | needs a sorting network (expensive) |

Selected via the `CFAR_TYPE` generic (`CA`, `GO`, `SO`).

---

## Fixed-point arithmetic

Everything is unsigned integer arithmetic; no dividers, no truncation.

`alpha` is stored as a fixed-point value scaled by `2^ALPHA_FRAC` (Q?.8):

```
ALPHA_FP = round(alpha * 2^ALPHA_FRAC)      # e.g. alpha=12.45 -> 3188  (Pfa=1e-4, N=16)
```

The detection test `CUT > (ref_sum / N) * (ALPHA_FP / 2^ALPHA_FRAC)` is rearranged
to remove both divisions (which are powers of two) by shifting the CUT instead:

```
CUT << (log2(N) + ALPHA_FRAC)  >  ref_sum * ALPHA_FP
```

So the shift amount is `log2(N) + ALPHA_FRAC`. For **CA** the estimate averages all
`N` cells (`log2(N) + 8`); for **GO/SO** it averages **one side** of `N/2` cells, so
the shift is `log2(N/2) + 8`. This comparison is exact and matches the golden model
bit for bit.

> **Note on alpha:** `ALPHA_FP = 3188` is derived from the **CA** formula
> `alpha = N * (Pfa^(-1/N) - 1)`. GO and SO have different alpha-vs-Pfa relationships
> (the statistics of max/min of two averages differ from a single average). RTL and
> golden agree bit-exactly because both use the same alpha, but to hold the target
> Pfa in GO/SO the alpha should be recomputed per variant.

---

## Architecture

- **Sliding window** implemented as a shift register (`window`), one sample in per
  valid clock.
- **Running sums** instead of combinational adder trees: `sum_all` (whole window,
  for CA) and per-side `left_sum` / `right_sum` (for GO/SO) are each maintained
  incrementally (`+= incoming - outgoing`) with dedicated side shift registers.
  This keeps the critical path short regardless of `N`.
- The estimator logic is selected with `if ... generate`, so **only the chosen
  variant is synthesized** — the unused branches do not exist in the fabric.
- `m_valid` deasserts during window fill (edge cells with incomplete neighbourhoods
  are not evaluated).

### Interface

```vhdl
generic (
  CFAR_TYPE  : cfar_t  := CA;   -- CA | GO | SO
  SAMPLE_W   : integer := 16;   -- input sample width (unsigned power)
  N_REF      : integer := 16;   -- total reference cells (power of two)
  N_GUARD    : integer := 2;    -- guard cells per side
  ALPHA_W    : integer := 16;   -- width of ALPHA_FP
  ALPHA_FP   : integer := 3188; -- alpha * 2^ALPHA_FRAC
  ALPHA_FRAC : integer := 8     -- alpha fractional bits
);
port (
  clk, rst : in  std_logic;
  s_data   : in  std_logic_vector(SAMPLE_W-1 downto 0);  -- streaming power samples
  s_valid  : in  std_logic;
  m_cut    : out std_logic_vector(SAMPLE_W-1 downto 0);  -- cell under test (for plotting)
  m_detect : out std_logic;                              -- 1 = target
  m_valid  : out std_logic                               -- 0 during fill / edges
);
```

---

## Verification

Verified with cocotb + GHDL against `cfar_golden.py`, a fixed-point model that is
bit-exact to the RTL (same integer arithmetic, same shift-based comparison). The
same stimulus (thermal noise + a clutter region + three targets, one hidden inside
the clutter) is fed to both DUT and golden, and the per-CUT detection decisions are
compared bit for bit.

```
[CA] PASS: 380 decisions bit-identical to golden
[GO] PASS: 380 decisions bit-identical to golden
[SO] PASS: 380 decisions bit-identical to golden
```

`test_diag.py` additionally dumps a per-CUT report on mismatch (RTL vs golden
estimator, threshold and decision) to localise a fault to a specific datapath stage.

### Run it

```bash
# CFAR_MODE must match the RTL's CFAR_TYPE generic
CFAR_MODE=CA make
CFAR_MODE=GO make
CFAR_MODE=SO make
```

---

## Synthesis (Artix-7, Basys 3)

| Mode | WNS @ 50 MHz | Approx. Fmax |
|------|--------------|--------------|
| CA   | +16.38 ns     | ~276 MHz     |
| GO/SO| +14.98 ns     | ~199 MHz     |

The design closes timing with large margin; no pipelining is needed for CA/GO/SO.
(OS will need a pipelined bitonic sorting network — ~10 serial compare-swap stages
for N=16 — and is planned as a separate stage.)

---

## The self-masking effect

The bundled scenario includes a strong target buried in high-variance clutter. With
narrow guards (`N_GUARD=1`), CA-CFAR **fails to detect it**: the target's own energy
leaks past the single guard cell into the reference cells, inflating the local noise
estimate and pushing the threshold above the (saturated) peak — the target masks
itself.

Widening the guard band (`N_GUARD=2`) puts the target's shoulders inside the guard
cells instead of the reference cells, drops the threshold, and recovers the target.
This is the textbook trade-off between guard width and reference locality, and both
configurations are included to illustrate it.

| N_GUARD=1 (self-masking, 2/3 detected) | N_GUARD=2 (recovered, 3/3 detected) |
|----------------------------------------|-------------------------------------|
| ![self-masking](docs/nguard1.png)      | ![recovered](docs/nguard2.png)      |

---

## Repository layout

```
.
|-- src/
|   |-- cfar_pkg.vhd        -- cfar_t enum (CA/GO/SO/OS)
|   `-- ca_cfar.vhd         -- the configurable detector
|-- golden/
|   `-- cfar_golden.py      -- fixed-point reference model (CA/GO/SO)
|-- tb/
|   |-- test_ca_cfar.py     -- cocotb testbench (variant via CFAR_MODE env var)
|   |-- test_diag.py        -- diagnostic testbench (per-CUT RTL-vs-golden dump)
|   `-- Makefile
|-- docs/
|   |-- nguard1.png
|   `-- nguard2.png
`-- README.md
```

---

## Roadmap

- [x] CA / GO / SO variants, generic-selected
- [x] Running-sum datapath (no combinational adder trees)
- [x] Bit-exact verification vs fixed-point golden (cocotb)
- [x] Timing closure on Artix-7
- [ ] Per-variant alpha (correct Pfa in GO/SO)
- [ ] OS-CFAR with a pipelined bitonic sorting network
- [ ] 2-D (Range-Doppler) CFAR