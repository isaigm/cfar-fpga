"""
Diagnostic cocotb testbench: same as test_ca_cfar.py, but on a mismatch it dumps a
per-CUT table (RTL vs golden estimator, threshold and decision) so a fault can be
localised to a specific datapath stage:

    - 'est' differs           -> bug in the window / running sum / estimator
    - 'threshold' differs     -> bug in the multiplier or alpha
    - est & threshold equal,
      'det' differs           -> bug in the comparison / shift

Reads internal RTL signals (dut.estimator, dut.threshold) -- GHDL exposes these.
"""
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, FallingEdge
import random, math, os
import numpy as np
from cfar_golden import ca_cfar, ALPHA_FP, N_REF, ALPHA_FRAC

CFAR_MODE = os.environ.get("CFAR_MODE", "CA")
N_GUARD   = int(os.environ.get("N_GUARD", "2"))


def scenario(n=400, seed=0):
    rnd = random.Random(seed)
    s = []
    for i in range(n):
        var = 300.0 if 150 < i < 250 else 30.0
        I = rnd.gauss(0, math.sqrt(var))
        Q = rnd.gauss(0, math.sqrt(var))
        I += 150.0 * math.exp(-((i - 80) ** 2) / 8.0)
        I += 200.0 * math.exp(-((i - 150) ** 2) / 8.0)
        I += 300.0 * math.exp(-((i - 200) ** 2) / 8.0)
        s.append(min(int(I ** 2 + Q ** 2), 65535))
    return s


def golden_detail(x, i, n_ref, n_guard, mode, alpha_fp, alpha_frac):
    """Recompute the golden's internals at CUT index i (for the report)."""
    half = n_ref // 2
    lead = half + n_guard
    log2n = n_ref.bit_length() - 1
    log2h = half.bit_length() - 1
    older = x[i - lead : i - n_guard]
    newer = x[i + n_guard + 1 : i + lead + 1]
    if mode == 'CA':
        est = int(older.sum()) + int(newer.sum()); shift = log2n + alpha_frac
    elif mode == 'GO':
        est = max(int(older.sum()), int(newer.sum())); shift = log2h + alpha_frac
    else:  # SO
        est = min(int(older.sum()), int(newer.sum())); shift = log2h + alpha_frac
    return int(x[i]), est, est * int(alpha_fp), (int(x[i]) << shift)


@cocotb.test()
async def test_cfar(dut):
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    dut.rst.value = 1
    dut.s_valid.value = 0
    dut.s_data.value = 0
    for _ in range(5):
        await RisingEdge(dut.clk)
    dut.rst.value = 0
    await RisingEdge(dut.clk)

    x = scenario()
    xa = np.array(x, dtype=np.int64)
    detect_g, valid_g = ca_cfar(xa, n_guard=N_GUARD, cfar_type=CFAR_MODE)
    gold = list(detect_g[valid_g].astype(int))
    valid_idx = list(np.where(valid_g)[0])       # bin index of each valid decision

    hw = []
    hw_extra = []
    for v in x:
        dut.s_data.value = v
        dut.s_valid.value = 1
        await RisingEdge(dut.clk)
        await FallingEdge(dut.clk)
        if int(dut.m_valid.value) == 1:
            hw.append(int(dut.m_detect.value))
            # read internal RTL signals for the diagnostic report
            hw_extra.append((int(dut.m_cut.value),
                             int(dut.estimator.value),
                             int(dut.threshold.value)))

    n = min(len(hw), len(gold))
    diffs = [k for k in range(n) if hw[k] != gold[k]]

    if not diffs:
        dut._log.info(f"[{CFAR_MODE}] PASS: {n} decisions bit-identical")
    else:
        dut._log.error(f"[{CFAR_MODE}] {len(diffs)} MISMATCHES. First few:")
        dut._log.error(f"{'bin':>4} {'CUT':>7} | {'RTL:est':>9} {'RTL:thr':>11} {'det':>3} "
                       f"| {'GOLD:est':>9} {'GOLD:thr':>11} {'det':>3}")
        for k in diffs[:8]:
            b = valid_idx[k]
            rtl_cut, rtl_est, rtl_thr = hw_extra[k]
            _, g_est, g_thr, _ = golden_detail(xa, b, N_REF, N_GUARD,
                                               CFAR_MODE, ALPHA_FP, ALPHA_FRAC)
            dut._log.error(f"{b:>4} {rtl_cut:>7} | {rtl_est:>9} {rtl_thr:>11} {hw[k]:>3} "
                           f"| {g_est:>9} {g_thr:>11} {gold[k]:>3}")
        assert False, f"[{CFAR_MODE}] {len(diffs)} mismatches vs golden"