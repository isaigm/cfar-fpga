TOPLEVEL_LANG = vhdl
SIM          ?= ghdl
VHDL_SOURCES  = $(PWD)/../src/cfar_pkg.vhd $(PWD)/../src/ca_cfar.vhd
TOPLEVEL      = ca_cfar
MODULE        = test_ca_cfar
EXTRA_ARGS   += --std=08
# golden model is in ../golden
export PYTHONPATH := $(PWD)/../golden:$(PYTHONPATH)
include $(shell cocotb-config --makefiles)/Makefile.sim
