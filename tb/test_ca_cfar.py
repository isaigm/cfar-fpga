"""
cocotb testbench for the configurable CFAR detector.
Verifies the RTL bit-exact against cfar_golden.py, for any variant.

The variant is chosen via the CFAR_MODE environment variable and MUST match the
RTL's CFAR_TYPE generic (CFAR_TYPE is elaboration-time, so the RTL must be
compiled for the same mode being tested).

    CFAR_MODE=CA make      # verify CA mode
    CFAR_MODE=GO make      # verify GO mode
    CFAR_MODE=SO make      # verify SO mode
"""
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, FallingEdge
import random, math, os
import numpy as np
from cfar_golden import ca_cfar, ALPHA_FP as ALPHA_FP_CA, N_REF

CFAR_MODE = os.environ.get("CFAR_MODE", "CA")   # must match RTL generic CFAR_TYPE
N_GUARD   = int(os.environ.get("N_GUARD", "2"))
OS_RANK   = int(os.environ.get("OS_RANK", "3")) # must match RTL generic OS_RANK
# alpha must match the RTL generic ALPHA_FP for the mode under test
ALPHA_FP  = int(os.environ.get("ALPHA_FP", str(ALPHA_FP_CA)))

# Sorter pipeline latency (0 for non-OS). Bitonic depth = clog2(N)*(clog2(N)+1)/2.
_LOGN     = (N_REF - 1).bit_length()
SORT_LAT  = _LOGN * (_LOGN + 1) // 2 if CFAR_MODE == "OS" else 0


def scenario(n=400, seed=0):
    """Thermal noise + clutter region + three targets (one hidden in clutter)."""
    rnd = random.Random(seed)
    s = []
    for i in range(n):
        var = 300.0 if 150 < i < 250 else 30.0       # clutter region
        I = rnd.gauss(0, math.sqrt(var))
        Q = rnd.gauss(0, math.sqrt(var))
        I += 150.0 * math.exp(-((i - 80) ** 2) / 8.0)   # target 1: clean zone
        I += 200.0 * math.exp(-((i - 150) ** 2) / 8.0)  # target 2: clutter edge
        I += 300.0 * math.exp(-((i - 200) ** 2) / 8.0)  # target 3: inside clutter
        s.append(min(int(I ** 2 + Q ** 2), 65535))       # square-law + 16-bit saturation
    return s


@cocotb.test()
async def test_cfar(dut):
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
    x = scenario()
    detect_g, valid_g = ca_cfar(np.array(x, dtype=np.int64),
                                alpha_fp=ALPHA_FP, n_guard=N_GUARD,
                                cfar_type=CFAR_MODE, os_rank=OS_RANK)
    gold = list(detect_g[valid_g].astype(int))   # valid decisions, in order

    # drive the stream, then drain SORT_LAT extra valid cycles to flush the sorter
    # pipeline (0 for non-OS). Drain samples enter after all real data, so their
    # (contaminated) valid outputs land past len(gold) and are dropped by the slice.
    hw = []
    for v in list(x) + [0] * SORT_LAT:
        dut.s_data.value = v
        dut.s_valid.value = 1
        await RisingEdge(dut.clk)      # DUT registers the shift/sum on this edge
        await FallingEdge(dut.clk)     # half a cycle: combinational outputs settled
        if int(dut.m_valid.value) == 1:
            hw.append(int(dut.m_detect.value))

    # compare
    n = min(len(hw), len(gold))
    diffs = [k for k in range(n) if hw[k] != gold[k]]
    dut._log.info(f"[{CFAR_MODE}] HW valid={len(hw)} golden valid={len(gold)} "
                  f"compared={n}  detections HW={sum(hw[:n])} golden={sum(gold[:n])}")
    assert not diffs, f"[{CFAR_MODE}] mismatch at CUTs {diffs[:10]} of {n}"
    dut._log.info(f"[{CFAR_MODE}] PASS: {n} decisions bit-identical to golden")