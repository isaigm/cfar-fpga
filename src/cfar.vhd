library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;
use work.cfar_pkg.all;

entity cfar is
  generic (
    CFAR_TYPE  : cfar_t  := OS;
    OS_RANK    : integer := 3;
    SAMPLE_W   : integer := 16;
    N_REF      : integer := 16;
    N_GUARD    : integer := 2;
    ALPHA_W    : integer := 16;
    ALPHA_FP   : integer := 33298;
    ALPHA_FRAC : integer := 8
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

architecture Behavioral of cfar is
  function sizeof(n : integer) return integer is
  begin
    return integer(ceil(log2(real(n))));
  end sizeof;

  constant TOTAL_SAMPLES : integer := N_REF + 2 * N_GUARD + 1;
  constant SUM_W         : integer := SAMPLE_W + sizeof(TOTAL_SAMPLES);
  constant T_W           : integer := SUM_W + ALPHA_W;
  constant HALF_REF      : integer := N_REF / 2;
  constant CUT_IDX       : integer := HALF_REF + N_GUARD;
  constant SORT_LAT      : integer := sizeof(N_REF) * (sizeof(N_REF) + 1) / 2;

  signal curr_idx          : integer range 0 to TOTAL_SAMPLES := 0;
  signal full              : std_logic;
  signal left_sum          : unsigned(SUM_W - 1 downto 0) := (others => '0');
  signal right_sum         : unsigned(SUM_W - 1 downto 0) := (others => '0');
  signal estimator         : unsigned(SUM_W - 1 downto 0);
  signal threshold         : unsigned(T_W - 1 downto 0);
  signal window            : sample_array_t (0 to TOTAL_SAMPLES - 1)(SAMPLE_W - 1 downto 0) := (others => (others => '0'));
  signal ref_window        : sample_array_t (0 to N_REF - 1)(SAMPLE_W - 1 downto 0);
  signal sorted_ref_window : sample_array_t (0 to N_REF - 1)(SAMPLE_W - 1 downto 0);
  signal cut_delay         : sample_array_t (0 to SORT_LAT - 1)(SAMPLE_W - 1 downto 0) := (others => (others => '0'));
  signal valid_delay       : std_logic_vector(0 to SORT_LAT - 1) := (others => '0');

begin
  full       <= '1' when curr_idx >= TOTAL_SAMPLES else '0';
  threshold  <= resize(estimator * to_unsigned(ALPHA_FP, ALPHA_W), T_W);
  ref_window <= window(0 to HALF_REF - 1) & window(CUT_IDX + N_GUARD + 1  to TOTAL_SAMPLES - 1);

  gen_left_right: if CFAR_TYPE = GO or CFAR_TYPE = SO generate
    m_detect <= '1' when full = '1' and (shift_left(resize(unsigned(window(CUT_IDX)), T_W), ALPHA_FRAC + sizeof(HALF_REF)) > threshold) else '0';
  end generate;

  gen_ca: if CFAR_TYPE = CA generate
    m_detect  <= '1' when full = '1' and (shift_left(resize(unsigned(window(CUT_IDX)), T_W), ALPHA_FRAC + sizeof(N_REF)) > threshold) else '0';
    estimator <= left_sum + right_sum;
  end generate;

  gen_go: if CFAR_TYPE = GO generate
    process(left_sum, right_sum)
    begin
      if left_sum > right_sum then
        estimator <= left_sum;
      else
        estimator <= right_sum;
      end if;
    end process;
  end generate;

  gen_so: if CFAR_TYPE = SO generate
    process(left_sum, right_sum)
    begin
      if left_sum < right_sum then
        estimator <= left_sum;
      else
        estimator <= right_sum;
      end if;
    end process;
  end generate;

  gen_os: if CFAR_TYPE = OS generate
    m_cut    <= cut_delay(0);
    m_valid  <= valid_delay(0);
    m_detect <= '1' when valid_delay(0) = '1' and (shift_left(resize(unsigned(cut_delay(0)), T_W), ALPHA_FRAC) > threshold) else '0';
    
    estimator <= resize(
      unsigned(sorted_ref_window(OS_RANK - 1)),
      SUM_W
    );

    bitonic_network_inst: entity work.bitonic_network
      generic map (
        n     => N_REF,
        width => SAMPLE_W
      )
      port map (
        clk     => clk,
        inputs  => ref_window,
        outputs => sorted_ref_window
      );
  end generate;

  gen_co_ca_go: if CFAR_TYPE = GO or CFAR_TYPE = SO or CFAR_TYPE = CA generate
    m_cut   <= window(CUT_IDX);
    m_valid <= '1' when (full = '1' and s_valid = '1') else '0';
    
    process(clk)
    begin
      if rising_edge(clk) then
        if rst = '1' then
          curr_idx     <= 0;
          window       <= (others => (others => '0'));
          left_sum     <= (others => '0');
          right_sum    <= (others => '0');
        elsif s_valid = '1' then
          window       <= window(1 to TOTAL_SAMPLES - 1) & s_data;
          right_sum <= right_sum
            + resize(unsigned(s_data), SUM_W)
            - resize(unsigned(window(TOTAL_SAMPLES - HALF_REF)), SUM_W);
            
          left_sum <= left_sum
            + resize(unsigned(window(HALF_REF)), SUM_W)
            - resize(unsigned(window(0)), SUM_W);
            
          if full = '0' then
            curr_idx <= curr_idx + 1;
          end if;
        end if;
      end if;
    end process;
  end generate;

  gen_os_proc: if CFAR_TYPE = OS generate
    process(clk)
    begin
      if rising_edge(clk) then
        if rst = '1' then
          curr_idx     <= 0;
          window       <= (others => (others => '0'));
          left_sum     <= (others => '0');
          right_sum    <= (others => '0');
          cut_delay    <= (others => (others => '0'));
          valid_delay  <= (others => '0');
        elsif s_valid = '1' then
          window       <= window(1 to TOTAL_SAMPLES - 1) & s_data;
          cut_delay    <= cut_delay(1 to SORT_LAT - 1) & window(CUT_IDX);
          valid_delay  <= valid_delay(1 to SORT_LAT - 1) & (full and s_valid);

          right_sum <= right_sum
            + resize(unsigned(s_data), SUM_W)
            - resize(unsigned(window(TOTAL_SAMPLES - HALF_REF)), SUM_W);
            
          left_sum <= left_sum
            + resize(unsigned(window(HALF_REF)), SUM_W)
            - resize(unsigned(window(0)), SUM_W);

          if full = '0' then
            curr_idx <= curr_idx + 1;
          end if;
        end if;
      end if;
    end process;
  end generate;

end architecture;