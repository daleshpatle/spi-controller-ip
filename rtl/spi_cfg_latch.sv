//======================================================================
// spi_cfg_latch.sv - Configuration latch (Spec 5.3)
//
// A wide configuration bus cannot be carried by a two-flop synchronizer.
// Instead the whole bundle is captured in ONE SHOT by a parallel register
// in the SPI domain, at a moment when the source is guaranteed stable.
//
// Capture condition:
//     (cfg_valid_sync | cfg_pend) && fsm_idle
//
// cfg_pend (commit-pending flop) is REQUIRED because SPI_CTRL is always
// writable: a commit pulse that arrives mid-frame would otherwise be lost
// and the write silently dropped. cfg_pend arms the commit and holds it
// until the FSM next reaches IDLE.  Latest-value-wins: cfg_bus is sampled
// at capture time, so two writes while busy capture the second value.
//
// en_effective is the POST-CAPTURE enable. The FSM must evaluate
// IDLE->LOAD against this, never against cfg_active.en alone, otherwise a
// frame can launch on a stale EN in the very cycle the capture lands.
//======================================================================
module spi_cfg_latch
  import spi_pkg::*; #(
  parameter logic DEF_CPOL = 1'b0,
  parameter logic DEF_CPHA = 1'b0
)(
  input  logic  spi_clk,
  input  logic  spi_rst_n,

  input  cfg_t  cfg_bus,          // quasi-static, from the bus-domain register file
  input  logic  cfg_valid_sync,   // synchronized commit pulse (Spec 5.2)
  input  logic  fsm_idle,

  output cfg_t  cfg_active,
  output logic  cfg_capture,      // high in the cycle the capture happens
  output logic  en_effective      // post-capture EN, for the FSM start condition
);


  logic cfg_pend;

  assign cfg_capture  = (cfg_valid_sync | cfg_pend) & fsm_idle;
  assign en_effective = cfg_capture ? cfg_bus.en : cfg_active.en;

  always_ff @(posedge spi_clk or negedge spi_rst_n) begin
    if (!spi_rst_n) begin
      cfg_pend   <= 1'b0;
      cfg_active <= cfg_reset_value(DEF_CPOL, DEF_CPHA);
    end else begin
      // arm on the synchronized commit pulse ...
      if (cfg_valid_sync) cfg_pend <= 1'b1;
      // ... and fire when the FSM is idle. This assignment is later in the
      // block, so on a simultaneous arm+fire the clear wins, which is the
      // intended behaviour (capture now, nothing left pending).
      if (cfg_capture) begin
        cfg_active <= cfg_bus;
        cfg_pend   <= 1'b0;
      end
    end
  end

endmodule
