//======================================================================
// spi_cdc_prims.sv - CDC primitives (Spec 5.2)
//   spi_reset_sync : async assert, synchronous release   (Spec 7.7)
//   spi_lvl_sync   : two-flop level synchronizer
//   spi_event_sync : toggle + 2-FF + 2-cycle edge detect  (5 flops/event)
//======================================================================

//----------------------------------------------------------------------
// Reset synchronizer - one instance PER CLOCK DOMAIN.
// Never fan a single instance across two domains (Spec 7.7).
//----------------------------------------------------------------------
module spi_reset_sync (
  input  logic clk,
  input  logic rst_n_in,     // raw asynchronous reset (active low)
  output logic rst_n_out     // async assert, sync deassert
);
  (* ASYNC_REG = "TRUE" *) logic r1, r2;

  always_ff @(posedge clk or negedge rst_n_in) begin
    if (!rst_n_in) {r2, r1} <= 2'b00;
    else           {r2, r1} <= {r1, 1'b1};
  end

  assign rst_n_out = r2;
endmodule


//----------------------------------------------------------------------
// Two-flop level synchronizer. Single-bit only - never use on a bus.
//----------------------------------------------------------------------
module spi_lvl_sync #(
  parameter logic INIT = 1'b0
)(
  input  logic clk,
  input  logic rst_n,
  input  logic d,
  output logic q
);
  (* ASYNC_REG = "TRUE" *) logic s1, s2;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) {s2, s1} <= {INIT, INIT};
    else        {s2, s1} <= {s1, d};
  end

  assign q = s2;
endmodule


//----------------------------------------------------------------------
// Event synchronizer (Spec 5.2)
//   src_pulse (1 cycle) -> toggle -> 2-FF sync -> 2-cycle-wide dst pulse
//   Chain: 1 toggle flop + 2 sync flops + 2 edge-detect flops = 5 flops.
//   The 2-cycle output cannot be missed by a same-clock consumer or by
//   a consumer running on a divide-by-two clock.
//----------------------------------------------------------------------
module spi_event_sync (
  input  logic src_clk,
  input  logic src_rst_n,
  input  logic src_pulse,
  input  logic dst_clk,
  input  logic dst_rst_n,
  output logic dst_pulse
);
  logic tgl;

  always_ff @(posedge src_clk or negedge src_rst_n) begin
    if (!src_rst_n)     tgl <= 1'b0;
    else if (src_pulse) tgl <= ~tgl;
  end

  (* ASYNC_REG = "TRUE" *) logic q1, q2;
  logic qd1, qd2;

  always_ff @(posedge dst_clk or negedge dst_rst_n) begin
    if (!dst_rst_n) {qd2, qd1, q2, q1} <= 4'b0000;
    else            {qd2, qd1, q2, q1} <= {qd1, q2, q1, tgl};
  end

  // two-cycle-wide recovered pulse
  assign dst_pulse = (q2 ^ qd1) | (qd1 ^ qd2);
endmodule
