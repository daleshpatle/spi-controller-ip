//======================================================================
// spi_clkgen.sv - Master-mode serial clock generator (Spec 8.1)
//
//   f(SCLK) = f(spi_ref_clk) / (2 * (CLKDIV + 1))     divide 2 .. 512
//
// The factor of two is structural: one reference edge makes the rising
// edge of SCLK and a second makes the falling edge, so no setting can
// produce a serial clock faster than half the reference. Every divide is
// even, so SCLK has an exact 50% duty cycle at all frequencies.
//
// SCLK is PARKED at the CPOL idle level whenever run=0, and the counter
// is held reset, so enabling never produces a runt or half-width pulse.
//======================================================================
module spi_clkgen (
  input  logic       spi_clk,
  input  logic       spi_rst_n,
  input  logic       run,          // asserted only during the TRANSFER state
  input  logic       cpol,
  input  logic       cpha,
  input  logic [7:0] clkdiv,

  output logic       sclk_o,       // the serial clock driven to the pin
  output logic       lead_edge,    // 1-cycle strobe at the leading  SCLK edge
  output logic       trail_edge,   // 1-cycle strobe at the trailing SCLK edge
  output logic       drive_en,     // shift-out strobe  (CPHA=0 -> trailing)
  output logic       sample_en,    // sample-in  strobe (CPHA=0 -> leading)
  output logic       edge_tick     // any SCLK edge this cycle
);

  logic [7:0] cnt;
  logic       sclk_q;

  always_ff @(posedge spi_clk or negedge spi_rst_n) begin
    if (!spi_rst_n) begin
      cnt    <= 8'd0;
      sclk_q <= 1'b0;
    end else if (!run) begin
      cnt    <= 8'd0;
      sclk_q <= cpol;                 // parked at the CPOL idle level
    end else if (cnt == clkdiv) begin
      cnt    <= 8'd0;
      sclk_q <= ~sclk_q;
    end else begin
      cnt    <= cnt + 8'd1;
    end
  end

  assign sclk_o    = sclk_q;
  assign edge_tick = run && (cnt == clkdiv);

  // Leading edge = the transition away from the CPOL idle level.
  assign lead_edge  = edge_tick && (sclk_q == cpol);
  assign trail_edge = edge_tick && (sclk_q != cpol);

  // CPHA = 0 : sample on the leading edge, drive on the trailing edge
  // CPHA = 1 : drive  on the leading edge, sample on the trailing edge
  assign sample_en = cpha ? trail_edge : lead_edge;
  assign drive_en  = cpha ? lead_edge  : trail_edge;

endmodule
