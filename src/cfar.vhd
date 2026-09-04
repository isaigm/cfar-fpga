library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;
  use ieee.math_real.all;
  use work.cfar_pkg.all;
entity cfar is
  generic (
    CFAR_TYPE     : cfar_t  := CA;
    SAMPLE_W      : integer := 16;    
    N_REF         : integer := 16;   
    N_GUARD       : integer := 2;    
    ALPHA_W       : integer := 16;    
    ALPHA_FP      : integer := 3188;   
    ALPHA_FRAC    : integer := 8
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
  constant SUM_W : integer := SAMPLE_W + sizeof(TOTAL_SAMPLES);
  constant T_W: integer := SUM_W + ALPHA_W;
  constant HALF_REF: integer := N_REF / 2;
  constant CUT_IDX: integer := HALF_REF + N_GUARD;
 
  type sample_array_t is array (natural range <>) of std_logic_vector(SAMPLE_W - 1 downto 0);

  
  signal curr_idx: integer range 0 to TOTAL_SAMPLES := 0;
  signal full: std_logic;
  signal sum_all: unsigned(SUM_W - 1 downto 0) := (others => '0');
  signal left_sum: unsigned(SUM_W - 1 downto 0) := (others => '0');
  signal right_sum: unsigned(SUM_W - 1 downto 0) := (others => '0');
  signal estimator: unsigned(SUM_W - 1 downto 0);
  signal threshold: unsigned(T_W - 1 downto 0);
  
  signal window      : sample_array_t(0 to TOTAL_SAMPLES - 1) := (others => (others => '0'));
  
begin
  full    <= '1' when curr_idx >= TOTAL_SAMPLES else '0';
  m_cut   <= window(CUT_IDX);
  m_valid <= '1' when (full = '1' and s_valid = '1') else '0';
  threshold <= resize(estimator * to_unsigned(ALPHA_FP, ALPHA_W), T_W);
  
  gen_left_right: if CFAR_TYPE = GO or CFAR_TYPE = SO generate
    m_detect <= '1' when full = '1' and (shift_left(resize(unsigned(window(CUT_IDX)), T_W), ALPHA_FRAC + sizeof(HALF_REF)) > threshold) else '0';

  end generate;

  gen_ca: if CFAR_TYPE = CA generate
    m_detect <= '1' when full = '1' and (shift_left(resize(unsigned(window(CUT_IDX)), T_W), ALPHA_FRAC + sizeof(N_REF)) > threshold) else '0';
    process(left_sum, right_sum)
      variable sum: unsigned(SUM_W - 1 downto 0);
    begin
      estimator <= left_sum + right_sum;
    end process;
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

  process(clk)
  begin
    if rising_edge(clk) then
      if rst = '1' then
        curr_idx <= 0;
        sum_all <= (others => '0');
        window <= (others => (others => '0'));
        left_sum <= (others => '0');
        right_sum <= (others => '0');
      elsif s_valid = '1' then 
        window <= window(1 to TOTAL_SAMPLES - 1) & s_data;

        sum_all <= sum_all
          + resize(unsigned(s_data), SUM_W)
          - resize(unsigned(window(0)), SUM_W);
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
end architecture;
