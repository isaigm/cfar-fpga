"""
VI-CFAR golden model -- fixed-point, bit-exact to vi_cfar.vhd.

Composite CFAR (Smith & Varshney): per CUT it computes a variability index (VI)
for each half-window and the mean ratio (MR) between them, then adapts to
CA / GO / SO / single-window CA. All tests are done in the SAME integer,
cross-multiplied form as the RTL (no division, no floats in the decision path),
so the decisions are bit-identical to the hardware.

    from vi_cfar_golden import vi_cfar, ALPHA_FP_BOTH, ALPHA_FP_HALF

Parameters below MUST match the RTL generics.
"""
import numpy as np

SAMPLE_W   = 16      # sample width (unsigned power)
N_REF      = 16      # total reference cells (power of two)
N_GUARD    = 2       # guard cells per side
ALPHA_FRAC = 8       # alpha fractional bits

# --- decision thresholds (match RTL generics) --------------------------------
K_VI_FRAC     = 8
K_MR_FRAC     = 8
K_VI_FP       = 1219    # round(4.76  * 2**K_VI_FRAC)
K_MR_FP       = 462     # round(1.806 * 2**K_MR_FRAC)

# --- per-mode alpha (match RTL generics) -------------------------------------
# N-cell case (CA over both windows) and N/2-cell case (GO / SO / single CA)
# need different scale factors to hold the same Pfa.
ALPHA_FP_BOTH = 3188    # round(N   * (Pfa**(-1/N)   - 1) * 2**ALPHA_FRAC),  N=16, Pfa=1e-4
ALPHA_FP_HALF = 4428    # round(N/2 * (Pfa**(-1/(N/2))- 1) * 2**ALPHA_FRAC),  N/2=8, Pfa=1e-4


def vi_cfar(x, n_ref=N_REF, n_guard=N_GUARD,
            k_vi_fp=K_VI_FP, k_vi_frac=K_VI_FRAC,
            k_mr_fp=K_MR_FP, k_mr_frac=K_MR_FRAC,
            alpha_fp_both=ALPHA_FP_BOTH, alpha_fp_half=ALPHA_FP_HALF,
            alpha_frac=ALPHA_FRAC):
    """
    VI-CFAR detection, bit-exact to the hardware.

    x       : unsigned power samples (array of ints).
    Returns : (detect, valid) boolean arrays the length of x.

    Window mapping (matches the RTL running sums):
        A = leading  (newer) half-window  -> right_sum / sq_right_sum
        B = lagging  (older) half-window  -> left_sum  / sq_left_sum

    Scaling (no truncation, identical to the RTL): the /n_cells and the /2^F are
    moved to the CUT side of the comparison as a left shift, so
        CA-both : n_cells = N     -> shift = log2(N)
        others  : n_cells = N/2   -> shift = log2(N/2)
        CUT << (shift + F)  >  estimate * alpha(mode)
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
        older = x[i - lead : i - n_guard]          # B (lagging)
        newer = x[i + n_guard + 1 : i + lead + 1]  # A (leading)

        # python ints -> exact wide arithmetic, matching the RTL's fixed widths
        SumB = int(older.sum())
        SumA = int(newer.sum())
        SqB  = sum(int(v) * int(v) for v in older)
        SqA  = sum(int(v) * int(v) for v in newer)

        # Step 1: VI test, cross-multiplied  (VI <= K_VI => homogeneous)
        homA = (SqA << (k_vi_frac + log2h)) <= k_vi_fp * SumA * SumA
        homB = (SqB << (k_vi_frac + log2h)) <= k_vi_fp * SumB * SumB

        # Step 2: MR test, cross-multiplied  (same mean if within [1/K_MR, K_MR])
        same_mean = ((SumA << k_mr_frac) <= k_mr_fp * SumB) and \
                    ((SumB << k_mr_frac) <= k_mr_fp * SumA)

        # Step 3: decision mux  (A = right/leading, B = left/lagging)
        if homA and homB:
            if same_mean:                      # homogeneous       -> CA over both (N)
                est, shift, alpha = SumA + SumB, log2n, alpha_fp_both
            else:                              # clutter edge      -> GO (N/2)
                est, shift, alpha = max(SumA, SumB), log2h, alpha_fp_half
        elif homA:                             # interferer in B   -> trust A only (N/2)
            est, shift, alpha = SumA, log2h, alpha_fp_half
        elif homB:                             # interferer in A   -> trust B only (N/2)
            est, shift, alpha = SumB, log2h, alpha_fp_half
        else:                                  # multiple targets  -> SO (N/2)
            est, shift, alpha = min(SumA, SumB), log2h, alpha_fp_half

        # Step 4: detection
        lhs = int(x[i]) << (shift + alpha_frac)
        rhs = est * alpha
        detect[i] = lhs > rhs
        valid[i]  = True

    return detect, valid