//======================================================================
// spi_pkg.sv - Shared types and constants for the SPI Controller IP
//======================================================================
package spi_pkg;

  // ---- Register offsets (byte addresses) ----
  localparam logic [7:0] ADDR_CTRL     = 8'h00;
  localparam logic [7:0] ADDR_STAT     = 8'h04;
  localparam logic [7:0] ADDR_TXDATA   = 8'h08;
  localparam logic [7:0] ADDR_RXDATA   = 8'h0C;
  localparam logic [7:0] ADDR_CLKDIV   = 8'h10;
  localparam logic [7:0] ADDR_IE       = 8'h14;
  localparam logic [7:0] ADDR_IS       = 8'h18;
  localparam logic [7:0] ADDR_CSCTRL   = 8'h1C;
  localparam logic [7:0] ADDR_FIFOCTRL = 8'h20;
  localparam logic [7:0] ADDR_FIFOLVL  = 8'h24;

  // ---- Interrupt source bit positions (SPI_IE / SPI_IS) ----
  localparam int IRQ_TXWM  = 0;
  localparam int IRQ_RXWM  = 1;
  localparam int IRQ_TXUNF = 2;
  localparam int IRQ_RXOVF = 3;
  localparam int IRQ_DONE  = 4;
  localparam int IRQ_FRERR = 5;
  localparam int IRQ_N     = 6;

  // ------------------------------------------------------------------
  // Latched configuration bundle (Spec 5.3).
  // Captured in one shot into the SPI domain by the configuration latch.
  // EN is part of this bundle; the IDLE->LOAD start condition is
  // evaluated against the post-capture value (Spec 7.4).
  // ------------------------------------------------------------------
  typedef struct packed {
    logic [5:0] cs_hold;    // SPI_CSCTRL[14:9]
    logic [5:0] cs_setup;   // SPI_CSCTRL[8:3]
    logic       cs_auto;    // SPI_CSCTRL[2]
    logic [1:0] cs_sel;     // SPI_CSCTRL[1:0]
    logic [7:0] clkdiv;     // SPI_CLKDIV[7:0]
    logic [5:0] dfs;        // SPI_CTRL[13:8]  literal frame size
    logic       loopback;   // SPI_CTRL[5]
    logic       lsbfirst;   // SPI_CTRL[4]
    logic       cpha;       // SPI_CTRL[3]
    logic       cpol;       // SPI_CTRL[2]
    logic       mode;       // SPI_CTRL[1]  0=master 1=slave
    logic       en;         // SPI_CTRL[0]
  } cfg_t;

  // Reset value of the latched bundle (mirrors Spec 6.10)
  function automatic cfg_t cfg_reset_value(input logic def_cpol,
                                           input logic def_cpha);
    cfg_t c;
    c.cs_hold  = 6'h3F;
    c.cs_setup = 6'h3F;
    c.cs_auto  = 1'b1;
    c.cs_sel   = 2'd0;
    c.clkdiv   = 8'hFF;
    c.dfs      = 6'd8;
    c.loopback = 1'b0;
    c.lsbfirst = 1'b0;
    c.cpha     = def_cpha;
    c.cpol     = def_cpol;
    c.mode     = 1'b0;
    c.en       = 1'b0;
    return c;
  endfunction

endpackage
