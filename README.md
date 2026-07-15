# spi-controller-ip
SPI Controller IP with production-grade CDC architecture — multi-flop, event, and handshake synchronization with full spec documentation
# SPI Controller IP — Multi-Clock-Domain Design with Rigorous CDC Architecture

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

- Directed and randomized stimulus in SystemVerilog
- Simulation-verified waveforms for all SPI modes and CDC corner cases (config change during transaction, back-to-back transfers, slave-side asynchronous SCLK)
- Simulator: Mentor QuestaSim

## Author

**Dalesh Patle** — Design Verification Engineer | M.Tech, IIT Guwahati
[LinkedIn](https://www.linkedin.com/in/dalesh-patle-134b80232)
