# sh_noc_mesh lint and simulation (Verilator)
#
#   make lint                      lint the mesh with -Wall
#   make sim                       build and run the random-transaction test
#   make sim PARAMS="-GASYNC=0 +define+SH_NOC_ROWS=2" SEED=7
#   make regress                   run a set of configurations
#
# PARAMS takes -G<tb parameter>=<value> for the implementation parameters and
# +define+SH_NOC_<name>=<value> to override sh_noc_pkg (ROWS, COLS, FLIT_W,
# MAX_LEN, TAG_W).

ROOT    := $(abspath $(dir $(lastword $(MAKEFILE_LIST)))/../../..)
PKG     := $(ROOT)/design/ip_top/pkg/ip_top_pkg.sv $(ROOT)/design/noc/pkg/sh_noc_pkg.sv
RTL     := $(PKG) $(shell find $(ROOT)/design -path '*/rtl/*.sv' | sort)
TB      := $(ROOT)/design/noc/tb/sh_noc_tb_mem.sv $(ROOT)/design/noc/tb/sh_noc_mesh_tb.sv
PARAMS  ?=
SEED    ?= 1
OBJ     := obj_dir

.PHONY: lint sim regress clean

lint:
	verilator --lint-only -Wall --top-module sh_noc_mesh $(PARAMS) $(RTL)

sim:
	verilator --binary --timing --timescale 1ns/1ps -j 0 -Wno-fatal -Wno-lint -Wno-style \
	  --top-module sh_noc_mesh_tb $(PARAMS) -Mdir $(OBJ) $(RTL) $(TB) >/dev/null
	$(abspath $(OBJ))/Vsh_noc_mesh_tb +verilator+seed+$(SEED)

regress:
	@set -e; \
	for p in "" "-GASYNC=0" \
	         "+define+SH_NOC_ROWS=1 +define+SH_NOC_COLS=1" \
	         "+define+SH_NOC_ROWS=1 +define+SH_NOC_COLS=5" \
	         "+define+SH_NOC_ROWS=3 +define+SH_NOC_COLS=5 +define+SH_NOC_MAX_LEN=1 +define+SH_NOC_TAG_W=1" \
	         "+define+SH_NOC_ROWS=2 +define+SH_NOC_COLS=3 +define+SH_NOC_FLIT_W=96 +define+SH_NOC_MAX_LEN=5" \
	         "-GBUF_DEPTH=2 -GCDC_DEPTH=2" \
	         "-GBUF_DEPTH=1 -GCDC_DEPTH=2 -GTX_PCT=100" \
	         "+define+SH_NOC_ROWS=2 +define+SH_NOC_COLS=3 -GRST_LOW=0 -GRST_ASYNC=0" \
	         "-GTX_PCT=100 -GRX_PCT=100" \
	         "-GUNMAPPED_PCT=60 -GRX_PCT=20" \
	         "-GBUF_MEM_RD_DEL=1 -GCDC_MEM_RD_DEL=1" \
	         "-GBUF_MEM_RD_DEL=2 -GCDC_MEM_RD_DEL=3 -GBUF_DEPTH=7" \
	         "-GASYNC=0 -GBUF_MEM_RD_DEL=1 -GCDC_MEM_RD_DEL=2 -GBUF_DEPTH=1 -GCDC_DEPTH=1"; do \
	  for s in 1 2 3; do \
	    out=$$($(MAKE) -s sim PARAMS="$$p" SEED=$$s OBJ=obj_dir_regress 2>&1); \
	    echo "PARAMS=\"$$p\" SEED=$$s: $$(echo "$$out" | grep -E '^(PASS|FAIL)')"; \
	    echo "$$out" | grep -q '^PASS' || { echo "$$out" | grep -iE 'error|fail' | head -20; exit 1; }; \
	  done; \
	done

clean:
	rm -rf obj_dir obj_dir_regress
