TOPLEVEL_LANG = vhdl
SIM          ?= ghdl
VHDL_SOURCES  = $(PWD)/../src/cfar_pkg.vhd $(PWD)/../src/bitonic_network.vhd $(PWD)/../src/cfar.vhd
TOPLEVEL      = cfar
MODULE        = test_ca_cfar
EXTRA_ARGS   += --std=08

# Variant under test. CFAR_MODE must match the RTL generic CFAR_TYPE, which is why
# we override the RTL generic from the same variable below.
CFAR_MODE ?= CA
OS_RANK   ?= 3

# alpha depends on the estimator statistics. GO/SO reuse the CA value (bit-exact vs
# golden holds; target Pfa does not -- see README). OS needs its own alpha.
ifeq ($(CFAR_MODE),OS)
  ALPHA_FP ?= 33298
else
  ALPHA_FP ?= 3188
endif

export CFAR_MODE
export OS_RANK
export ALPHA_FP

# GHDL applies top-level generic overrides at run time.
SIM_ARGS += -gCFAR_TYPE=$(CFAR_MODE) -gOS_RANK=$(OS_RANK) -gALPHA_FP=$(ALPHA_FP)

# golden model is in ../golden
export PYTHONPATH := $(PWD)/../golden:$(PYTHONPATH)
include $(shell cocotb-config --makefiles)/Makefile.sim