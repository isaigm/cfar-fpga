library ieee;
  use ieee.std_logic_1164.all;

package cfar_pkg is

    type cfar_t is (CA, GO, SO, OS);

    type sample_array_t is array (natural range <>) of std_logic_vector;

end package cfar_pkg;