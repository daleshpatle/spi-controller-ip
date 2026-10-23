//======================================================================
// spi_cdc_prims.sv - CDC primitives (Spec 5.2)
//   spi_reset_sync : async assert, synchronous release   (Spec 7.7)
//   spi_lvl_sync   : two-flop level synchronizer
//   spi_event_sync : toggle + 2-FF + 2-cycle edge detect, with a return
//                    acknowledge and a pending flag (no event is lost)
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
// Event synchronizer with return acknowledge (Spec 5.2)
//   src_pulse (1 cycle) -> toggle -> 2-FF sync -> 2-cycle-wide dst pulse
//   Forward chain: 1 toggle flop + 2 sync flops + 2 edge-detect flops.
//   The 2-cycle output cannot be missed by a same-clock consumer or by
//   a consumer running on a divide-by-two clock.
//
//   Acknowledge: the synchronized toggle (q2) is synchronized back into
//   the source domain (2 flops). A new toggle is sent only after the
//   previous one has come back. A pulse that arrives earlier sets the
//   pending flag and is sent when the acknowledge returns. Without this,
//   two pulses within one destination clock period toggle twice, the
//   destination sees no change and BOTH events are lost (bug B13).
//   Events closer together than the acknowledge round trip are merged
//   into one; every consumer is a sticky bit or a latest-value capture,
//   so merging is harmless by construction.
//
//   src_busy (source domain): an event is in flight or pending.
//----------------------------------------------------------------------
module spi_event_sync (
  input  logic src_clk,
  input  logic src_rst_n,
  input  logic src_pulse,
  output logic src_busy,
  input  logic dst_clk,
  input  logic dst_rst_n,
  output logic dst_pulse
);
  // ---------------- destination: 2-FF sync + 2-cycle edge detect ----------------
  logic tgl;
  (* ASYNC_REG = "TRUE" *) logic q1, q2;
  logic qd1, qd2;

  always_ff @(posedge dst_clk or negedge dst_rst_n) begin
    if (!dst_rst_n) {qd2, qd1, q2, q1} <= 4'b0000;
    else            {qd2, qd1, q2, q1} <= {qd1, q2, q1, tgl};
  end

  // two-cycle-wide recovered pulse
  assign dst_pulse = (q2 ^ qd1) | (qd1 ^ qd2);

  // ---------------- source: toggle, acknowledge, pending ----------------
  (* ASYNC_REG = "TRUE" *) logic a1, a2;   // acknowledge synchronizer (q2 -> src)
  logic pend;
  logic ack_ok;                             // destination has seen the last toggle
  assign ack_ok = (tgl == a2);

  always_ff @(posedge src_clk or negedge src_rst_n) begin
    if (!src_rst_n) begin
      tgl      <= 1'b0;
      pend     <= 1'b0;
      {a2, a1} <= 2'b00;
    end else begin
      {a2, a1} <= {a1, q2};
      if ((src_pulse || pend) && ack_ok) begin
        tgl  <= ~tgl;                       // send (a pending and a new pulse merge)
        pend <= 1'b0;
      end else if (src_pulse) begin
        pend <= 1'b1;                       // previous toggle not yet acknowledged
      end
    end
  end

  assign src_busy = pend || !ack_ok;
endmodule
