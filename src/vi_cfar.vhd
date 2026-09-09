library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;
use work.cfar_pkg.all;

entity vi_cfar is
  generic (
    SAMPLE_W      : integer := 16;
    N_REF         : integer := 16;
    N_GUARD       : integer := 2;
    ALPHA_W       : integer := 16;
    -- VI / MR decision thresholds, fixed-point.
    -- Canonical paper values ~4.76 and ~1.806; tune vs the golden for your N_REF/Pfa.
    K_VI_FP       : integer := 1219;   -- round(4.76  * 2^K_VI_FRAC)
    K_MR_FP       : integer := 462;    -- round(1.806 * 2^K_MR_FRAC)
    -- Per-mode threshold scale factors, fixed-point. The VI-CFAR normalises over a
    -- different cell count depending on the decision, so it needs two alphas:
    ALPHA_FP_BOTH : integer := 3188;   -- N-cell   case: CA over both windows
    ALPHA_FP_HALF : integer := 4428;   -- N/2-cell case: GO / SO / single-window CA
    ALPHA_FRAC    : integer := 8;
    K_VI_FRAC     : integer := 8;
    K_MR_FRAC     : integer := 8
  );
  port (
    clk, rst : in  std_logic;
    s_data   : in  std_logic_vector(SAMPLE_W - 1 downto 0);
    s_valid  : in  std_logic;
    m_cut    : out std_logic_vector(SAMPLE_W - 1 downto 0);
    m_detect : out std_logic;
    m_valid  : out std_logic
  );
end entity;

architecture Behavioral of vi_cfar is
  function sizeof(n : integer) return integer is
  begin
    return integer(ceil(log2(real(n))));
  end sizeof;

  constant TOTAL_SAMPLES : integer := N_REF + 2 * N_GUARD + 1;
  constant SUM_W         : integer := SAMPLE_W + sizeof(TOTAL_SAMPLES);
  constant SQ_W          : integer := 2 * SAMPLE_W;
  constant SQ_SUM_W      : integer := SQ_W + sizeof(TOTAL_SAMPLES);
  constant T_W           : integer := SUM_W + ALPHA_W;
  constant HALF_REF      : integer := N_REF / 2;
  constant CUT_IDX       : integer := HALF_REF + N_GUARD;

  -- Comparison widths. The VI test carries a square (SumA^2) so it is much wider
  -- than the MR test, which is linear in the sums. Keep them separate.
  constant CMP_W    : integer := 2 * SUM_W + sizeof(K_VI_FP);   -- ~53b: K_VI_FP * SumA^2 dominates
  constant MR_CMP_W : integer := SUM_W + sizeof(K_MR_FP);       -- ~30b: K_MR_FP * SumA   dominates

  signal curr_idx     : integer range 0 to TOTAL_SAMPLES := 0;
  signal full         : std_logic;
  signal left_sum     : unsigned(SUM_W - 1 downto 0)    := (others => '0');
  signal right_sum    : unsigned(SUM_W - 1 downto 0)    := (others => '0');
  signal sq_left_sum  : unsigned(SQ_SUM_W - 1 downto 0) := (others => '0');
  signal sq_right_sum : unsigned(SQ_SUM_W - 1 downto 0) := (others => '0');

  signal estimator : unsigned(SUM_W - 1 downto 0);
  signal alpha_sel : unsigned(ALPHA_W - 1 downto 0);      -- per-mode alpha, chosen per CUT
  signal ref_shift : natural range 0 to SUM_W;            -- /N vs /(N/2) normalisation, per CUT
  signal threshold : unsigned(T_W - 1 downto 0);
  signal window    : sample_array_t (0 to TOTAL_SAMPLES - 1)(SAMPLE_W - 1 downto 0) := (others => (others => '0'));
  signal sq_window : sample_array_t (0 to TOTAL_SAMPLES - 1)(SQ_W - 1 downto 0)     := (others => (others => '0'));

  signal homA      : std_logic;   -- window A (right / leading) homogeneous?
  signal homB      : std_logic;   -- window B (left  / lagging) homogeneous?
  signal same_mean : std_logic;   -- leading/lagging means similar?

begin
  full <= '1' when curr_idx >= TOTAL_SAMPLES else '0';

  ---------------------------------------------------------------------------
  -- Step 1: variability-index test per half-window  (VI <= K_VI => homogeneous)
  --   m*SqA * 2^F  <=  K_VI_FP * SumA^2       (m = HALF_REF folded into the shift)
  ---------------------------------------------------------------------------
  homA <= '1' when shift_left(resize(sq_right_sum, CMP_W), K_VI_FRAC + sizeof(HALF_REF))
                   <= resize(K_VI_FP * right_sum * right_sum, CMP_W)
          else '0';
  homB <= '1' when shift_left(resize(sq_left_sum, CMP_W), K_VI_FRAC + sizeof(HALF_REF))
                   <= resize(K_VI_FP * left_sum * left_sum, CMP_W)
          else '0';

  ---------------------------------------------------------------------------
  -- Step 2: mean-ratio test  (1/K_MR <= SumA/SumB <= K_MR => same mean)
  --   cross-multiplied: SumA*2^F <= K_MR_FP*SumB  AND  SumB*2^F <= K_MR_FP*SumA
  --   (the reciprocal bound falls out of the second inequality -- no 1/K_MR needed)
  ---------------------------------------------------------------------------
  same_mean <= '1' when (shift_left(resize(right_sum, MR_CMP_W), K_MR_FRAC) <= resize(K_MR_FP * left_sum,  MR_CMP_W))
                    and (shift_left(resize(left_sum,  MR_CMP_W), K_MR_FRAC) <= resize(K_MR_FP * right_sum, MR_CMP_W))
               else '0';

  ---------------------------------------------------------------------------
  -- Step 3: decision mux -- pick estimator + normalisation (shift) + alpha per CUT.
  --   A = right window (leading), B = left window (lagging).
  ---------------------------------------------------------------------------
  process(all)
  begin
    if homA = '1' and homB = '1' then
      if same_mean = '1' then                 -- homogeneous       -> CA over both windows (N cells)
        estimator <= left_sum + right_sum;
        ref_shift <= sizeof(N_REF);
        alpha_sel <= to_unsigned(ALPHA_FP_BOTH, ALPHA_W);
      else                                     -- clutter edge      -> GO (greatest of, N/2 cells)
        if right_sum > left_sum then
          estimator <= right_sum;
        else
          estimator <= left_sum;
        end if;
        ref_shift <= sizeof(HALF_REF);
        alpha_sel <= to_unsigned(ALPHA_FP_HALF, ALPHA_W);
      end if;
    elsif homA = '1' then                      -- interferer in B   -> trust A (right) only (N/2)
      estimator <= right_sum;
      ref_shift <= sizeof(HALF_REF);
      alpha_sel <= to_unsigned(ALPHA_FP_HALF, ALPHA_W);
    elsif homB = '1' then                      -- interferer in A   -> trust B (left) only (N/2)
      estimator <= left_sum;
      ref_shift <= sizeof(HALF_REF);
      alpha_sel <= to_unsigned(ALPHA_FP_HALF, ALPHA_W);
    else                                       -- multiple targets  -> SO (smallest of, N/2 cells)
      if right_sum < left_sum then
        estimator <= right_sum;
      else
        estimator <= left_sum;
      end if;
      ref_shift <= sizeof(HALF_REF);
      alpha_sel <= to_unsigned(ALPHA_FP_HALF, ALPHA_W);
    end if;
  end process;

  ---------------------------------------------------------------------------
  -- Step 4: adaptive threshold + detection.  threshold = estimator * alpha(mode)
  --   CUT << (shift(mode) + ALPHA_FRAC)  >  threshold
  ---------------------------------------------------------------------------
  threshold <= resize(estimator * alpha_sel, T_W);

  m_cut    <= window(CUT_IDX);
  m_valid  <= '1' when (full = '1' and s_valid = '1') else '0';
  m_detect <= '1' when full = '1'
                   and shift_left(resize(unsigned(window(CUT_IDX)), T_W), ALPHA_FRAC + ref_shift) > threshold
              else '0';

  ---------------------------------------------------------------------------
  -- Front-end: sample window, squares window, running sums (raw and squared).
  ---------------------------------------------------------------------------
  process(clk)
    variable sq_data : unsigned(SQ_W - 1 downto 0);
  begin
    if rising_edge(clk) then
      if rst = '1' then
        curr_idx     <= 0;
        window       <= (others => (others => '0'));
        sq_window    <= (others => (others => '0'));
        left_sum     <= (others => '0');
        right_sum    <= (others => '0');
        sq_left_sum  <= (others => '0');
        sq_right_sum <= (others => '0');
      elsif s_valid = '1' then
        sq_data   := unsigned(s_data) * unsigned(s_data);   -- one multiplier per cell
        window    <= window(1 to TOTAL_SAMPLES - 1) & s_data;
        sq_window <= sq_window(1 to TOTAL_SAMPLES - 1) & std_logic_vector(sq_data);

        right_sum <= right_sum
          + resize(unsigned(s_data), SUM_W)
          - resize(unsigned(window(TOTAL_SAMPLES - HALF_REF)), SUM_W);
        left_sum <= left_sum
          + resize(unsigned(window(HALF_REF)), SUM_W)
          - resize(unsigned(window(0)), SUM_W);

        sq_right_sum <= sq_right_sum
          + resize(sq_data, SQ_SUM_W)
          - resize(unsigned(sq_window(TOTAL_SAMPLES - HALF_REF)), SQ_SUM_W);
        sq_left_sum <= sq_left_sum
          + resize(unsigned(sq_window(HALF_REF)), SQ_SUM_W)
          - resize(unsigned(sq_window(0)), SQ_SUM_W);

        if full = '0' then
          curr_idx <= curr_idx + 1;
        end if;
      end if;
    end if;
  end process;

end architecture;