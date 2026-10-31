#=====================================================================
# Makefile - Verilator flow for the SPI Controller IP
#
#   make            # build + run the smoke test
#   make modes      # build + run the CPOL/CPHA/bit-order/DFS sweep
#   make slave      # build + run the slave-mode regression (TB = SPI master)
#   make slave-ivl  # same slave regression on Icarus Verilog (4-state)
#   make lint       # lint only, no build
#   make wave       # run the smoke test with VCD dump -> tb_spi_controller.vcd
#   make wave-modes # same for the mode sweep
#   make wave-slave # same for the slave regression
#   make clean
#=====================================================================
VERILATOR ?= verilator
YOSYS ?= $(HOME)/oss-cad-suite/bin/yosys
IVERILOG  ?= iverilog
VVP       ?= vvp

# --binary  : elaborate, generate main(), compile and link in one step
# --timing  : enable #delay / event control in initial blocks (Verilator 5.x)
# -j 0      : use all cores
VFLAGS  = --binary --timing -j 0
VFLAGS += --timescale 1ns/1ps          # RTL files carry no `timescale
VFLAGS += -Wall -Wno-fatal
VFLAGS += -Wno-DECLFILENAME            # spi_cdc_prims.sv holds 3 modules
VFLAGS += -Wno-UNUSEDSIGNAL            # unused status/level bits
VFLAGS += -Wno-WIDTHTRUNC -Wno-WIDTHEXPAND

.PHONY: all smoke modes slave slave-ivl lint wave wave-modes wave-slave clean

all: smoke

smoke:
	$(VERILATOR) $(VFLAGS) --top-module tb_spi_controller \
	    -f compile.f -o sim_smoke --Mdir obj_smoke
	./obj_smoke/sim_smoke

modes:
	$(VERILATOR) $(VFLAGS) --top-module tb_modes \
	    -f compile_modes.f -o sim_modes --Mdir obj_modes
	./obj_modes/sim_modes

slave:
	$(VERILATOR) $(VFLAGS) --top-module tb_slave \
	    -f compile_slave.f -o sim_slave --Mdir obj_slave
	./obj_slave/sim_slave

# Icarus is 4-state: a flop that is never reset stays X and the checks
# fail. Verilator starts every flop at 0, which hides a missing reset.
slave-ivl:
	mkdir -p obj_ivl
	$(IVERILOG) -g2012 -s tb_slave -o obj_ivl/sim_slave -c compile_slave.f
	$(VVP) -n obj_ivl/sim_slave

lint:
	$(VERILATOR) --lint-only -Wall \
	    -Wno-DECLFILENAME \
	    --top-module spi_controller_top -f rtl.f
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

wave-slave:
	$(VERILATOR) $(VFLAGS) --trace -DDUMP --top-module tb_slave \
	    -f compile_slave.f -o sim_wslave --Mdir obj_wslave
	./obj_wslave/sim_wslave
	@echo "  -> tb_slave.vcd"

clean:
	rm -rf obj_smoke obj_modes obj_slave obj_ivl obj_wave obj_wmodes obj_wslave *.vcd

struct:
		$(YOSYS) -q -p "read_slang -f rtl.f --top spi_controller_top \
		    --blackboxed-module spi_slave_engine; proc; check -nolatches -assert"

r3:
		$(VERILATOR) $(VFLAGS) --top-module tb_r3 \
	 	   -f compile_r3.f -o sim_r3 --Mdir obj_r3
		./obj_r3/sim_r3
regs:
		$(VERILATOR) $(VFLAGS) $(REGS_DEFS) --top-module tb_regs \
		   -f compile_regs.f -o sim_regs --Mdir obj_regs
		./obj_regs/sim_regs
flush:
		$(VERILATOR) $(VFLAGS) --top-module tb_flush \
		    -f compile_flush.f -o sim_flush --Mdir obj_flush
		./obj_flush/sim_flush +seed=1
		./obj_flush/sim_flush +seed=2 +hp=5000 +hr=40000
		./obj_flush/sim_flush +seed=3 +hp=40000 +hr=5000
