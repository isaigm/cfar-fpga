"""
Generate the README figures from the fixed-point golden model.

These plots come straight from cfar_golden.py (no RTL / cocotb needed): the RTL is
verified bit-exact to the golden, so the golden's decisions are the RTL's decisions.
Keeping plotting out of the testbench keeps verification fast and CI-friendly.

Produces:
    docs/nguard1.png  -- N_GUARD=1: strong target self-masks inside clutter (2/3)
    docs/nguard2.png  -- N_GUARD=2: wider guards recover it (3/3)

Run:  python docs/make_plots.py       (from the repo root)
"""
import sys, os, math
import numpy as np
import matplotlib
matplotlib.use("Agg")          # headless backend (safe on WSL / no display)
import matplotlib.pyplot as plt

# import the golden model from ../golden
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "golden"))
from cfar_golden import ca_cfar, ALPHA_FP, ALPHA_FRAC, N_REF


def scenario(n=400, seed=0):
    """Thermal noise + clutter region + three targets (one hidden in clutter)."""
    rnd = __import__("random").Random(seed)
    s = []
    for i in range(n):
        var = 300.0 if 150 < i < 250 else 30.0
        I = rnd.gauss(0, math.sqrt(var))
        Q = rnd.gauss(0, math.sqrt(var))
        I += 150.0 * math.exp(-((i - 80) ** 2) / 8.0)
        I += 200.0 * math.exp(-((i - 150) ** 2) / 8.0)
        I += 300.0 * math.exp(-((i - 200) ** 2) / 8.0)
        s.append(min(int(I ** 2 + Q ** 2), 65535))
    return np.array(s, dtype=np.int64)


def threshold_curve(x, n_ref, n_guard, alpha_fp, alpha_frac, cfar_type='CA'):
    """Reconstruct the adaptive threshold (in power units) at every evaluated bin,
    so it can be plotted on the same axis as the signal."""
    half = n_ref // 2
    lead = half + n_guard
    log2n = n_ref.bit_length() - 1
    thr = np.full(len(x), np.nan)
    for i in range(lead, len(x) - lead):
        older = x[i - lead:i - n_guard]
        newer = x[i + n_guard + 1:i + lead + 1]
        est = int(older.sum()) + int(newer.sum())          # CA
        # threshold in power units = (ref_sum/N) * (ALPHA_FP/2^F)
        thr[i] = (est * alpha_fp) / (2.0 ** (log2n + alpha_frac))
    return thr


def make_plot(n_guard, fname, title):
    x = scenario()
    detect, valid = ca_cfar(x, n_guard=n_guard, cfar_type='CA')
    thr = threshold_curve(x, N_REF, n_guard, ALPHA_FP, ALPHA_FRAC, 'CA')
    det_bins = np.where(detect)[0]

    # x axis in TRUE bin numbers (evaluated region starts at 'lead')
    bins = np.arange(len(x))

    plt.figure(figsize=(12, 6))
    plt.plot(bins, x, label="Radar signal (power)", color="tab:blue", alpha=0.7)
    plt.plot(bins, thr, label="Adaptive CA-CFAR threshold", color="tab:orange", linewidth=2)
    for j, d in enumerate(det_bins):
        plt.axvline(x=d, color="red", linestyle="--", alpha=0.5,
                    label="Detection" if j == 0 else "")
        plt.plot(d, x[d], "ro")
    plt.title(title)
    plt.xlabel("Range (distance bins)")
    plt.ylabel("Power amplitude")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    out = os.path.join(os.path.dirname(__file__), fname)
    plt.savefig(out, dpi=110)
    plt.close()
    print(f"{fname}: N_GUARD={n_guard}, {int(detect.sum())} detections at bins {list(det_bins)}")


if __name__ == "__main__":
    make_plot(1, "nguard1.png",
              "CA-CFAR, N_GUARD=1  -- strong target self-masks inside clutter (2/3)")
    make_plot(2, "nguard2.png",
              "CA-CFAR, N_GUARD=2  -- wider guards recover the hidden target (3/3)")
