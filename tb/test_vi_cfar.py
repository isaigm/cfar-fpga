"""
cocotb testbench for the VI-CFAR detector.
Verifies the RTL bit-exact against vi_cfar_golden.py.

    make                 # SIM=ghdl by default

The decision thresholds and per-mode alphas are read from the environment and
passed to BOTH the golden (as function args) and the RTL (as -g generics via the
Makefile), so sweeping them from the Makefile keeps the two sides in lockstep.
"""
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, FallingEdge
import random, math, os
import numpy as np
from vi_cfar_golden import (vi_cfar, N_GUARD as N_GUARD_D,
                            K_VI_FP as K_VI_FP_D, K_MR_FP as K_MR_FP_D,
                            ALPHA_FP_BOTH as A_BOTH_D, ALPHA_FP_HALF as A_HALF_D)

# tunables -- must match the RTL generics (Makefile passes the same values via -g)
N_GUARD       = int(os.environ.get("N_GUARD", N_GUARD_D))
K_VI_FP       = int(os.environ.get("K_VI_FP", K_VI_FP_D))
K_MR_FP       = int(os.environ.get("K_MR_FP", K_MR_FP_D))
ALPHA_FP_BOTH = int(os.environ.get("ALPHA_FP_BOTH", A_BOTH_D))
ALPHA_FP_HALF = int(os.environ.get("ALPHA_FP_HALF", A_HALF_D))


def scenario(n=400, seed=0):
    """Thermal noise + clutter region + three targets (one hidden in clutter)."""
    rnd = random.Random(seed); s = []
    for i in range(n):
        var = 300.0 if 150 < i < 250 else 30.0
        I = rnd.gauss(0, math.sqrt(var)); Q = rnd.gauss(0, math.sqrt(var))
        I += 150.0 * math.exp(-((i - 80) ** 2) / 8.0)
        I += 200.0 * math.exp(-((i - 150) ** 2) / 8.0)
        I += 300.0 * math.exp(-((i - 200) ** 2) / 8.0)
        s.append(min(int(I ** 2 + Q ** 2), 65535))
    return s


def stimulus():
    """Diverse stream that exercises all five VI-CFAR branches."""
    rnd = random.Random(7); x = []
    for sd in range(4): x += scenario(400, sd)              # clean / clutter / targets
    x += [rnd.randint(50, 300) for _ in range(400)]         # homogeneous     -> CA-both
    for _ in range(12):                                     # clutter edges   -> GO
        x += [rnd.randint(40, 120) for _ in range(25)]
        x += [rnd.randint(3000, 6000) for _ in range(25)]
    for _ in range(40):                                     # single spikes   -> single-window CA
        x += [rnd.randint(50, 250) for _ in range(28)]; x += [rnd.randint(40000, 65535)]
    for _ in range(40):                                     # spike pairs     -> SO
        x += [rnd.randint(50, 250) for _ in range(9)];  x += [60000]
        x += [rnd.randint(50, 250) for _ in range(11)]; x += [60000]
        x += [rnd.randint(50, 250) for _ in range(9)]
    return [min(int(v), 65535) for v in x]


@cocotb.test()
async def test_vi_cfar(dut):
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())

    # reset
    dut.rst.value = 1
    dut.s_valid.value = 0
    dut.s_data.value = 0
    for _ in range(5):
        await RisingEdge(dut.clk)
    dut.rst.value = 0
    await RisingEdge(dut.clk)

    # same stimulus to DUT and golden -> fair bit-exact comparison
    x = stimulus()
    detect_g, valid_g = vi_cfar(np.array(x, dtype=np.int64),
                                n_guard=N_GUARD,
                                k_vi_fp=K_VI_FP, k_mr_fp=K_MR_FP,
                                alpha_fp_both=ALPHA_FP_BOTH,
                                alpha_fp_half=ALPHA_FP_HALF)
    gold = list(detect_g[valid_g].astype(int))

    # VI has no sorter: outputs are combinational off the registered front-end,
    # so there is zero pipeline latency and nothing to drain (SORT_LAT = 0).
    hw = []
    for v in x:
        dut.s_data.value = int(v)
        dut.s_valid.value = 1
        await RisingEdge(dut.clk)      # DUT registers the shift/sums
        await FallingEdge(dut.clk)     # combinational outputs settled
        if int(dut.m_valid.value) == 1:
            hw.append(int(dut.m_detect.value))

    n = min(len(hw), len(gold))
    diffs = [k for k in range(n) if hw[k] != gold[k]]
    dut._log.info(f"[VI] HW valid={len(hw)} golden valid={len(gold)} compared={n}  "
                  f"detections HW={sum(hw[:n])} golden={sum(gold[:n])}")
    assert not diffs, f"[VI] mismatch at CUTs {diffs[:10]} of {n}"
    dut._log.info(f"[VI] PASS: {n} decisions bit-identical to golden")