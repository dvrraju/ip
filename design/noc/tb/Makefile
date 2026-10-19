# sh_noc_mesh lint and simulation (Verilator)
#
#   make lint                      lint the mesh with -Wall
#   make sim                       build and run the random-traffic test
#   make sim PARAMS="-GROWS=4 -GCOLS=4 -GASYNC=0" SEED=7
#   make regress                   run a set of configurations

ROOT    := $(abspath $(dir $(lastword $(MAKEFILE_LIST)))/../../..)
RTL     := $(shell find $(ROOT)/design -path '*/rtl/*.sv' | sort)
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
	for p in "" "-GASYNC=0" "-GROWS=1 -GCOLS=1" "-GROWS=1 -GCOLS=5" \
	         "-GROWS=4 -GCOLS=4 -GBUF_DEPTH=2 -GCDC_DEPTH=2" \
	         "-GROWS=2 -GCOLS=3 -GRST_LOW=0 -GRST_ASYNC=0" \
	         "-GROWS=4 -GCOLS=4 -GINJ_PCT=100 -GEJ_PCT=100" \
	         "-GBUF_MEM_RD_DEL=1 -GCDC_MEM_RD_DEL=1" \
	         "-GBUF_MEM_RD_DEL=2 -GCDC_MEM_RD_DEL=3 -GBUF_DEPTH=7" \
	         "-GASYNC=0 -GBUF_MEM_RD_DEL=1 -GCDC_MEM_RD_DEL=2 -GBUF_DEPTH=1 -GCDC_DEPTH=1" \
	         "-GROWS=4 -GCOLS=4 -GBUF_MEM_RD_DEL=1 -GBUF_DEPTH=6 -GINJ_PCT=100 -GEJ_PCT=100"; do \
	  for s in 1 2 3; do \
	    out=$$($(MAKE) -s sim PARAMS="$$p" SEED=$$s OBJ=obj_dir_regress 2>&1); \
	    echo "PARAMS=\"$$p\" SEED=$$s: $$(echo "$$out" | grep -E '^(PASS|FAIL)')"; \
	    echo "$$out" | grep -q '^PASS' || { echo "$$out" | grep -iE 'error|fail' | head -20; exit 1; }; \
	  done; \
	done

clean:
	rm -rf obj_dir obj_dir_regress
