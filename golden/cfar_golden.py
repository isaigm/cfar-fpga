"""
Configurable CA-CFAR golden model -- fixed-point, bit-exact to the RTL.

Supported variants: 'CA', 'GO', 'SO'  ('OS' is planned).

The testbench does:  from cfar_golden import ca_cfar, ALPHA_FP

Parameters below MUST match the RTL generics.
"""
import numpy as np

SAMPLE_W   = 16      # sample width (unsigned power)
N_REF      = 16      # total reference cells (power of two)
N_GUARD    = 2       # guard cells per side (RTL default is 2)
ALPHA_FRAC = 8       # alpha fractional bits (Q?.8)
PFA        = 1e-4    # design probability of false alarm


def alpha_from_pfa(pfa, n_ref=N_REF):
    """Theoretical CA-CFAR alpha for exponential noise: a = N * (Pfa^(-1/N) - 1).

    NOTE: valid for CA. GO/SO have a different alpha-vs-Pfa relationship for the
    same Pfa, because the statistics of max/min of two averages differ from a
    single average.
    """
    return n_ref * (pfa ** (-1.0 / n_ref) - 1.0)


# Fixed-point alpha consumed by both RTL and testbench. Pfa=1e-4 -> 3188.
ALPHA_FP = round(alpha_from_pfa(PFA) * (1 << ALPHA_FRAC))


def ca_cfar(x, alpha_fp=ALPHA_FP, n_ref=N_REF, n_guard=N_GUARD,
            alpha_frac=ALPHA_FRAC, cfar_type='CA'):
    """
    CFAR detection, bit-exact to the hardware, for any variant.

    x         : unsigned power samples (array of ints).
    cfar_type : 'CA' | 'GO' | 'SO'  (OS not yet implemented).
    Returns   : (detect, valid), boolean arrays the length of x.
                detect[i] -- True if bin i exceeds the adaptive threshold.
                valid[i]  -- False on the edge cells that lack a full
                             neighbourhood (first/last 'lead' cells).

    Scaling (no truncation, identical to the RTL): the /N (or /(N/2)) and the
    /2^F are moved to the CUT side of the comparison as a left shift, so:

        CA:    estimate uses all N reference cells -> shift = log2(N)   + F
        GO/SO: estimate uses ONE side of N/2 cells -> shift = log2(N/2) + F

        CUT << shift  >  estimate * alpha_fp
    """
    x = np.asarray(x, dtype=np.int64)
    half  = n_ref // 2
    lead  = half + n_guard                 # distance CUT -> farthest reference
    L = len(x)
    log2n = n_ref.bit_length() - 1         # log2(N)
    log2h = half.bit_length() - 1          # log2(N/2)

    detect = np.zeros(L, dtype=bool)
    valid  = np.zeros(L, dtype=bool)

    for i in range(lead, L - lead):
        older = x[i - lead : i - n_guard]          # N/2 cells (older side)
        newer = x[i + n_guard + 1 : i + lead + 1]  # N/2 cells (newer side)

        if cfar_type == 'CA':
            est   = int(older.sum()) + int(newer.sum())   # all N cells
            shift = log2n + alpha_frac
        elif cfar_type == 'GO':
            est   = max(int(older.sum()), int(newer.sum()))
            shift = log2h + alpha_frac
        elif cfar_type == 'SO':
            est   = min(int(older.sum()), int(newer.sum()))
            shift = log2h + alpha_frac
        elif cfar_type == 'OS':
            raise NotImplementedError("OS: sorting network not implemented yet")
        else:
            raise ValueError(f"unknown cfar_type: {cfar_type}")

        lhs = int(x[i]) << shift
        rhs = est * int(alpha_fp)
        detect[i] = lhs > rhs
        valid[i]  = True

    return detect, valid


if __name__ == "__main__":
    import math, random

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

    x = scenario()
    for t in ('CA', 'GO', 'SO'):
        d, v = ca_cfar(np.array(x), cfar_type=t)
        print(f"{t}: {int(d.sum())} detections over {int(v.sum())} evaluated cells")