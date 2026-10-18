# spi-controller-ip
SPI Controller IP with production-grade CDC architecture — multi-flop, event, and handshake synchronization with full spec documentation
# SPI Controller IP — Multi-Clock-Domain Design with Rigorous CDC Architecture

[![Verilator](https://github.com/daleshpatle/spi-controller-ip/actions/workflows/spi-verilator.yml/badge.svg)](https://github.com/daleshpatle/spi-controller-ip/actions/workflows/spi-verilator.yml)

A company-grade SPI (Serial Peripheral Interface) Controller IP supporting Master and Slave modes, designed with a production-quality Clock Domain Crossing (CDC) architecture and a full documentation suite (specification, architecture diagrams, and verified timing diagrams).

> **Why this project:** Most student SPI cores ignore CDC entirely. This IP treats CDC as a first-class design problem — every domain boundary is identified, classified, and closed with the appropriate synchronization technique, following industry practice (Cummings, SNUG papers).

---

## Features

- SPI Master and Slave modes, all four SPI modes (CPOL/CPHA 00–11)
- Configurable data width and clock divider
- Register-based configuration interface (system clock domain)
- Independent clock domains: System clock, generated SCLK, and external SCLK (slave)
- Interrupt generation with proper event CDC back to the system domain

## CDC Architecture (the core of this design)

| Boundary | Signal class | Technique |
|---|---|---|
| System → SPI domain (config) | Quasi-static multi-bit | Synchronized commit pulse: `cfg_write_pulse` → toggle + 2-FF event synchronizer → `cfg_valid_sync` → gated capture |
| SPI → System (status/events) | Single-bit event | Toggle-based event CDC with return acknowledgment |
| External SCLK ↔ SPI domain (slave) | Multi-bit data | `write_req`/`write_ack` full handshake at the domain boundary |
| Async inputs | Level | 2-FF synchronizers |

Key design decisions documented in the spec:
- **Configuration commit race closure** — a last-IDLE-cycle race on configuration capture is closed using a synchronized commit pulse rather than relying on quasi-static assumptions alone.
- **Quasi-static CDC justification** — where the quasi-static technique *is* used, its validity conditions are stated explicitly per Cliff Cummings' CDC papers.
- **Metastability budgeting** — synchronizer depth choices justified with MTBF reasoning.

## Repository Contents

```
├── rtl/            # Synthesizable SystemVerilog RTL
├── docs/           # Specification, architecture & CDC diagrams, timing diagrams
├── tb/             # Testbench
└── README.md
```

## Documentation Suite

- **Design Specification** — register map, protocol behavior, timing parameters
- **CDC Strategy document** — every crossing identified and classified, with the closure technique per crossing
- **Cycle-accurate timing diagrams** — simulation-verified (not hand-drawn approximations)

## Verification

- Directed, self-checking SystemVerilog testbenches, run in CI on every push
- Mode sweep: 4 SPI modes x MSB/LSB x DFS in {4, 8, 16} through loopback — 24/24 passing
- Back-to-back multi-word transfers, APB error protocol, DMA/interrupt interlock, software reset
- Slave mode: the testbench is the external SPI master — 4 SPI modes x MSB/LSB x DFS in {4, 8, 13, 32} x 7 frame patterns, plus 12.5 MHz SCLK — 240/240 passing on Verilator and Icarus
- Simulators: Verilator 5.x, Icarus Verilog 12.0
- Not yet covered: aborted frames (`FRAME_ERR`), TX data written during a frame, configuration change mid-frame, CDC-aware simulation (random clock ratios)

## Author

**Dalesh Patle** — Design Verification Engineer | M.Tech, IIT Guwahati
[LinkedIn](https://www.linkedin.com/in/dalesh-patle-134b80232)

## Simulation

### Verilator (open source, no licence)

Requires **Verilator 5.x** — `--timing` and `--binary` do not exist in 4.x.

```sh
sudo apt install verilator gtkwave     # Ubuntu 24.04 ships 5.020

make            # smoke test
make modes      # mode / bit-order / DFS sweep
make slave      # slave-mode regression (TB is the SPI master)
make slave-ivl  # same on Icarus Verilog (sudo apt install iverilog)
make lint       # lint only
make wave       # smoke test + tb_spi_controller.vcd
```

### Xcelium

```sh
xrun -f compile.f       -top tb_spi_controller -access +r -l xrun.log
xrun -f compile_modes.f -top tb_modes          -access +r -l xrun_modes.log
```

The testbenches drive APB stimulus a delta after the clock edge
(`@(posedge pclk); #1; psel = 1;`) rather than with non-blocking assignments.
Verilator does not implement the NBA region inside `initial` blocks or tasks
called from them, so `psel <= 1` executes as a blocking assignment and races the
DUT flops on that same edge — which silently drops the first TX word. Driving
after a small delay is simulator-neutral and gives identical results under
Verilator, Icarus and event-driven signoff tools.

## Verification status

| Test | Coverage | Result |
|---|---|---|
| `tb_spi_controller` | Reset values, `PSLVERR` rules, `DFS` range rejection, always-writable `SPI_CTRL`, loopback data integrity, DMA gating and the interrupt interlock, software-reset sequence | **ALL CHECKS PASSED** |
| `tb_modes` | 4 SPI modes x MSB/LSB x DFS in {4, 8, 16} through loopback | **24 / 24** |
| `tb_slave` | Slave mode with the TB as the external SPI master: 4 SPI modes x MSB/LSB x DFS in {4, 8, 13, 32} x 7 frame patterns (multi-word and back-to-back frames, TX underrun, nothing staged, first frame after reset) plus 12.5 MHz SCLK. Checks every MISO word, every RX word, RX level, `DONE` and `TXUNF`, and that `RXOVF`/`FRERR` stay 0 | **240 / 240** |

Cross-verified on Verilator 5.x and Icarus Verilog 12.0.
`verilator --lint-only -Wall` is clean apart from two waived informational
categories (`DECLFILENAME`, `UNUSEDSIGNAL`) and one commented in-line waiver
(`SYNCASYNCNET`: in the slave engine CS_n is both an async clear and data, on purpose).

Slave-mode timing limit: back-to-back TX words need
(DFS − 1) × T_SCLK > ~4 × T_spi_ref_clk. With a 29 MHz `spi_ref_clk`, DFS 4
works up to about 21 MHz SCLK; DFS 8 and larger were checked up to 50 MHz SCLK.
