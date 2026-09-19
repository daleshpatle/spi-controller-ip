//======================================================================
// spi_async_fifo.sv - Dual-clock FIFO, gray-coded pointers (Spec 5.1)
//
// The ONLY path by which multi-bit data crosses between the bus and SPI
// domains. Each side compares its own pointer against a two-flop
// synchronized copy of the foreign pointer, so the reported level is
// conservative, never optimistic (Spec 6.9).
//
// w_clr / r_clr are SYNCHRONOUS clears used by FIFO flush and by the
// software-reset sequence. Both sides must be cleared - the SWRST
// sequencer does this in each domain (Spec 7.5 steps 2 and 6).
//======================================================================
module spi_async_fifo #(
  parameter int WIDTH = 32,
  parameter int DEPTH = 8                      // power of two
)(
  // write port
  input  logic                       wclk,
  input  logic                       wrst_n,
  input  logic                       w_clr,
  input  logic                       wen,
  input  logic [WIDTH-1:0]           wdata,
  output logic                       wfull,
  output logic [$clog2(DEPTH):0]     wlevel,

  // read port
  input  logic                       rclk,
  input  logic                       rrst_n,
  input  logic                       r_clr,
  input  logic                       ren,
  output logic [WIDTH-1:0]           rdata,
  output logic                       rempty,
  output logic [$clog2(DEPTH):0]     rlevel
);

  localparam int AW = $clog2(DEPTH);

  function automatic [AW:0] bin2gray(input logic [AW:0] b);
    bin2gray = b ^ (b >> 1);
  endfunction

  function automatic [AW:0] gray2bin(input logic [AW:0] g);
    logic [AW:0] b;
    for (int i = AW; i >= 0; i--) b[i] = ^(g >> i);
    gray2bin = b;
  endfunction

  logic [WIDTH-1:0] mem [DEPTH-1:0];

  // ------------------------------------------------------------------
  // All pointer and synchronizer declarations are hoisted here, ahead of
  // both clock-domain blocks. Each side references the OTHER side's gray
  // pointer, so a per-section declaration would forward-reference and is
  // rejected under strict declare-before-use (IEEE 1800 12.5).
  // ------------------------------------------------------------------
  logic [AW:0] wbin, wgray, wbin_nxt, wgray_nxt;
  logic [AW:0] rbin, rgray, rbin_nxt, rgray_nxt;
  (* ASYNC_REG = "TRUE" *) logic [AW:0] wq1_rgray, wq2_rgray;
  (* ASYNC_REG = "TRUE" *) logic [AW:0] rq1_wgray, rq2_wgray;
  logic [AW:0] wr_rbin, rd_wbin;

  // ---------------- write side ----------------

  assign wbin_nxt  = wbin + {{AW{1'b0}}, (wen && !wfull)};
  assign wgray_nxt = bin2gray(wbin_nxt);

  always_ff @(posedge wclk or negedge wrst_n) begin
    if (!wrst_n) begin
      wbin  <= '0;
      wgray <= '0;
    end else if (w_clr) begin
      wbin  <= '0;
      wgray <= '0;
    end else begin
      wbin  <= wbin_nxt;
      wgray <= wgray_nxt;
    end
  end

  always_ff @(posedge wclk) begin
    if (wen && !wfull) mem[wbin[AW-1:0]] <= wdata;
  end

  // synchronize the read pointer into the write domain
  always_ff @(posedge wclk or negedge wrst_n) begin
    if (!wrst_n)      {wq2_rgray, wq1_rgray} <= '0;
    else if (w_clr)   {wq2_rgray, wq1_rgray} <= '0;   // pipeline cleared too
    else              {wq2_rgray, wq1_rgray} <= {wq1_rgray, rgray};
  end

  // Full is REGISTERED, not a continuous assign. A combinational wfull would
  // close a loop wfull -> wen -> wbin_nxt -> wgray_nxt -> wfull, because the
  // writer gates its own enable on full. Registering it is the standard
  // dual-clock FIFO construction and costs nothing in throughput.
  logic wfull_val;
  assign wfull_val = (wgray_nxt == {~wq2_rgray[AW:AW-1], wq2_rgray[AW-2:0]});

  always_ff @(posedge wclk or negedge wrst_n) begin
    if      (!wrst_n) wfull <= 1'b0;
    else if (w_clr)   wfull <= 1'b0;
    else              wfull <= wfull_val;
  end

  assign wr_rbin = gray2bin(wq2_rgray);
  assign wlevel  = wbin - wr_rbin;              // conservative (reads high)

  // ---------------- read side ----------------

  assign rbin_nxt  = rbin + {{AW{1'b0}}, (ren && !rempty)};
  assign rgray_nxt = bin2gray(rbin_nxt);

  always_ff @(posedge rclk or negedge rrst_n) begin
    if (!rrst_n) begin
      rbin  <= '0;
      rgray <= '0;
    end else if (r_clr) begin
      rbin  <= '0;
      rgray <= '0;
    end else begin
      rbin  <= rbin_nxt;
      rgray <= rgray_nxt;
    end
  end

  // synchronize the write pointer into the read domain
  always_ff @(posedge rclk or negedge rrst_n) begin
    if (!rrst_n)      {rq2_wgray, rq1_wgray} <= '0;
    else if (r_clr)   {rq2_wgray, rq1_wgray} <= '0;   // pipeline cleared too
    else              {rq2_wgray, rq1_wgray} <= {rq1_wgray, wgray};
  end

  // Empty is registered for the same reason, and resets ASSERTED.
  logic rempty_val;
  assign rempty_val = (rgray_nxt == rq2_wgray);

  always_ff @(posedge rclk or negedge rrst_n) begin
    if      (!rrst_n) rempty <= 1'b1;
    else if (r_clr)   rempty <= 1'b1;
    else              rempty <= rempty_val;
  end
  assign rdata  = mem[rbin[AW-1:0]];             // first-word-fall-through

  assign rd_wbin = gray2bin(rq2_wgray);
  assign rlevel  = rd_wbin - rbin;               // conservative (reads low)

endmodule
