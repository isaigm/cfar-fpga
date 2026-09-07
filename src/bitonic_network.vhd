library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;
  use ieee.math_real.all;
  use work.cfar_pkg.all;

entity bitonic_network is
  generic ( N : integer := 16; WIDTH : integer := 16; DIR : std_logic := '1' );
  port (
    clk     : in  std_logic;
    inputs  : in  sample_array_t(0 to N - 1)(WIDTH - 1 downto 0);
    outputs : out sample_array_t(0 to N - 1)(WIDTH - 1 downto 0)
  );
end entity;

architecture rtl of bitonic_network is
  function clog2(x : integer) return integer is
  begin return integer(ceil(log2(real(x)))); end function;
  function bxor(a, b : integer) return integer is
  begin return to_integer(to_unsigned(a, 32) xor to_unsigned(b, 32)); end function;
  function band(a, b : integer) return integer is
  begin return to_integer(to_unsigned(a, 32) and to_unsigned(b, 32)); end function;

  constant LOGN   : integer := clog2(N);
  constant STAGES : integer := LOGN * (LOGN + 1) / 2;

  type grid_t is array (0 to STAGES) of sample_array_t(0 to N - 1)(WIDTH - 1 downto 0);
  signal reg  : grid_t;
  signal comb : grid_t;
begin
  comb(0) <= inputs;
  reg(0)  <= inputs;

  gen_k: for ki in 1 to LOGN generate
    gen_j: for ji in 0 to ki - 1 generate
      constant k : integer := 2 ** ki;
      constant j : integer := 2 ** (ki - 1 - ji);
      constant s : integer := (ki - 1) * ki / 2 + ji + 1;
    begin
      gen_i: for i in 0 to N - 1 generate
        constant p         : integer := bxor(i, j);
        constant i_is_low  : boolean := (band(i, j) = 0);
        constant asc_block : boolean := (band(i, k) = 0);
        constant want_asc  : boolean := asc_block xor (DIR = '0');
        constant take_min  : boolean := (i_is_low = want_asc);
      begin
        gmin: if take_min generate
          comb(s)(i) <= reg(s-1)(i) when unsigned(reg(s-1)(i)) <= unsigned(reg(s-1)(p))
                        else reg(s-1)(p);
        end generate;
        gmax: if not take_min generate
          comb(s)(i) <= reg(s-1)(i) when unsigned(reg(s-1)(i)) >= unsigned(reg(s-1)(p))
                        else reg(s-1)(p);
        end generate;
      end generate;

      process(clk) begin
        if rising_edge(clk) then
          reg(s) <= comb(s);
        end if;
      end process;
    end generate;
  end generate;

  outputs <= reg(STAGES);
end architecture;