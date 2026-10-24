# sh_l2 lint and unit simulation (Verilator)
#
#   make lint                      lint sh_l2 with -Wall
#   make sim                       build and run the unit test
#   make sim PARAMS="-GDATA_RD_DEL=2 +define+SH_L2_BYTES=1024" SEED=7
#   make regress                   run a set of configurations
#   make sys                       build and run the system test (L2 in the mesh)
#   make sys_regress               run the system test in several configurations
#
# PARAMS takes -G<tb parameter>=<value> and +define+SH_L2_<name>=<value> /
# +define+SH_NOC_<name>=<value> to override the packages.

ROOT    := $(abspath $(dir $(lastword $(MAKEFILE_LIST)))/../../..)
PKG     := $(ROOT)/design/ip_top/pkg/ip_top_pkg.sv $(ROOT)/design/noc/pkg/sh_noc_pkg.sv \
           $(ROOT)/design/l2/pkg/sh_l2_pkg.sv
RTL     := $(PKG) $(shell find $(ROOT)/design -path '*/rtl/*.sv' | sort)
TB      := $(ROOT)/design/l2/tb/sh_l2_tb_mem.sv $(ROOT)/design/l2/tb/sh_l2_tb.sv
SYS_TB  := $(ROOT)/design/l2/tb/sh_l2_tb_mem.sv $(ROOT)/design/noc/tb/sh_noc_tb_mem.sv \
           $(ROOT)/design/l2/tb/sh_l2_sys_tb.sv
PARAMS  ?=
SEED    ?= 1
OBJ     := obj_dir

.PHONY: lint sim regress sys sys_regress clean

lint:
	verilator --lint-only -Wall --top-module sh_l2 $(PARAMS) $(RTL)

sim:
	verilator --binary --timing --timescale 1ns/1ps -j 0 -Wno-fatal -Wno-lint -Wno-style \
	  --top-module sh_l2_tb $(PARAMS) -Mdir $(OBJ) $(RTL) $(TB) >/dev/null
	$(abspath $(OBJ))/Vsh_l2_tb +verilator+seed+$(SEED)

sys:
	verilator --binary --timing --timescale 1ns/1ps -j 0 -Wno-fatal -Wno-lint -Wno-style \
	  --top-module sh_l2_sys_tb $(PARAMS) -Mdir $(OBJ)_sys $(RTL) $(SYS_TB) >/dev/null
	$(abspath $(OBJ))_sys/Vsh_l2_sys_tb +verilator+seed+$(SEED)

# Small caches (a few sets and ways) so that lines are evicted constantly
SMALL := +define+SH_L2_BYTES=512 +define+SH_L2_WAYS=2

regress:
	@set -e; \
	for p in "$(SMALL)" \
	         "$(SMALL) -GTAG_RD_DEL=1 -GDATA_RD_DEL=1 -GLRU_RD_DEL=1" \
	         "$(SMALL) -GTAG_RD_DEL=3 -GDATA_RD_DEL=2 -GLRU_RD_DEL=1" \
	         "+define+SH_L2_BYTES=256 +define+SH_L2_WAYS=4" \
	         "+define+SH_L2_BYTES=128 +define+SH_L2_WAYS=2 -GSPAN_LINES=8" \
	         "+define+SH_L2_BYTES=2048 +define+SH_L2_WAYS=8 -GSPAN_LINES=96" \
	         "$(SMALL) -GTX_PCT=100 -GRX_PCT=100 -GMEM_DLY=0" \
	         "$(SMALL) -GRX_PCT=20 -GWB_ERR_PCT=30" \
	         "$(SMALL) -GBAD_MOD=0 -GFULL_PCT=40" \
	         "-GNUM_REQ=3000 -GSPAN_LINES=4096" \
	         "-GNUM_REQ=3000 -GSPAN_LINES=40000 -GDATA_RD_DEL=2" \
	         "-GNUM_REQ=3000 -GSPAN_LINES=24 -GSTRIDE=2048 -GTAG_RD_DEL=1 -GLRU_RD_DEL=2" \
	         "$(SMALL) -GPF_RD_DEL=2 -GDATA_RD_DEL=1" \
	         "$(SMALL) +define+SH_L2_PF_DEPTH=2" \
	         "$(SMALL) +define+SH_L2_PF_DEPTH=4 -GPF_RD_DEL=1" \
	         "$(SMALL) -GSPAN_LINES=2 -GBAD_MOD=0" \
	         "$(SMALL) -GSPAN_LINES=6 -GMEM_DLY=100 -GRX_PCT=30" \
	         "$(SMALL) -GMAX_OUT=1"; do \
	  for s in 1 2 3; do \
	    out=$$($(MAKE) -s sim PARAMS="$$p" SEED=$$s OBJ=obj_dir_regress 2>&1); \
	    echo "PARAMS=\"$$p\" SEED=$$s: $$(echo "$$out" | grep -E '^(PASS|FAIL)')"; \
	    echo "$$out" | grep -q '^PASS' || { echo "$$out" | grep -iE 'error|fail' | head -20; exit 1; }; \
	  done; \
	done

sys_regress:
	@set -e; \
	for p in "$(SMALL)" \
	         "$(SMALL) -GL2_RD_DEL=2 -GASYNC=0" \
	         "+define+SH_L2_BYTES=2048 +define+SH_L2_WAYS=4 -GSPAN_LINES=32" \
	         "-GSPAN_LINES=64" \
	         "$(SMALL) +define+SH_NOC_ROWS=3 +define+SH_NOC_COLS=3 +define+SH_NOC_L2_X=0 +define+SH_NOC_L2_Y=0" \
	         "$(SMALL) -GTX_PCT=100 -GRX_PCT=100 -GMEM_DLY=0"; do \
	  for s in 1 2; do \
	    out=$$($(MAKE) -s sys PARAMS="$$p" SEED=$$s OBJ=obj_dir_sregress 2>&1); \
	    echo "PARAMS=\"$$p\" SEED=$$s: $$(echo "$$out" | grep -E '^(PASS|FAIL)')"; \
	    echo "$$out" | grep -q '^PASS' || { echo "$$out" | grep -iE 'error|fail' | head -20; exit 1; }; \
	  done; \
	done

clean:
	rm -rf obj_dir* 
