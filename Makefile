#=====================================================================
# Makefile - Verilator flow for the SPI Controller IP
#
#   make            # build + run the smoke test
#   make modes      # build + run the CPOL/CPHA/bit-order/DFS sweep
#   make lint       # lint only, no build
#   make wave       # run the smoke test with VCD dump -> tb_spi_controller.vcd
#   make wave-modes # same for the mode sweep
#   make clean
#=====================================================================
VERILATOR ?= verilator

# --binary  : elaborate, generate main(), compile and link in one step
# --timing  : enable #delay / event control in initial blocks (Verilator 5.x)
# -j 0      : use all cores
VFLAGS  = --binary --timing -j 0
VFLAGS += --timescale 1ns/1ps          # RTL files carry no `timescale
VFLAGS += -Wall -Wno-fatal
VFLAGS += -Wno-DECLFILENAME            # spi_cdc_prims.sv holds 3 modules
VFLAGS += -Wno-UNUSEDSIGNAL            # unused status/level bits
VFLAGS += -Wno-BLKSEQ                  # blocking assign in TB clock generators
VFLAGS += -Wno-WIDTHTRUNC -Wno-WIDTHEXPAND

.PHONY: all smoke modes lint wave wave-modes clean

all: smoke

smoke:
	$(VERILATOR) $(VFLAGS) --top-module tb_spi_controller \
	    -f compile.f -o sim_smoke --Mdir obj_smoke
	./obj_smoke/sim_smoke

modes:
	$(VERILATOR) $(VFLAGS) --top-module tb_modes \
	    -f compile_modes.f -o sim_modes --Mdir obj_modes
	./obj_modes/sim_modes

lint:
	$(VERILATOR) --lint-only --timing --timescale 1ns/1ps -Wall -Wno-fatal \
	    -Wno-DECLFILENAME -Wno-UNUSEDSIGNAL -Wno-BLKSEQ \
	    --top-module tb_spi_controller -f compile.f

wave:
	$(VERILATOR) $(VFLAGS) --trace -DDUMP --top-module tb_spi_controller \
	    -f compile.f -o sim_wave --Mdir obj_wave
	./obj_wave/sim_wave
	@echo "  -> tb_spi_controller.vcd   (gtkwave tb_spi_controller.vcd)"

wave-modes:
	$(VERILATOR) $(VFLAGS) --trace -DDUMP --top-module tb_modes \
	    -f compile_modes.f -o sim_wmodes --Mdir obj_wmodes
	./obj_wmodes/sim_wmodes
	@echo "  -> tb_modes.vcd"

clean:
	rm -rf obj_smoke obj_modes obj_wave obj_wmodes *.vcd

