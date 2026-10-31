//======================================================================
// spi_async_fifo.sv - Dual-clock FIFO, gray-coded pointers (Spec 5.1)
//
// The ONLY path by which multi-bit data crosses between the bus and SPI
// domains. Each side compares its own pointer against a two-flop
// synchronized copy of the foreign pointer, so the reported level is
// conservative, never optimistic (Spec 6.9).
//
// w_clr / r_clr are SYNCHRONOUS clears used only by the software-reset
// sequence. Both sides must be cleared - the SWRST sequencer does this in
// each domain (Spec 7.5 steps 2 and 6) and holds the bus side cleared
// until the SPI side has been cleared too.
//
// FLUSH never clears a pointer (bug B15). A pointer that jumps to 0
// changes several gray bits at once, and the two sides would jump at
// different times, so for a while one side compares against the other
// side's old pointer and reports words that do not exist. Instead the
// READ pointer steps forward to a flush target, one entry per read clock:
// the same single-bit gray step as a normal read. The far side only ever
// sees real pointer values, at most a few cycles late.
//   r_flush  read-domain pulse: discard every word the read side can see.
//            For a FIFO whose flush is written in its read domain (RX).
//   w_flush  write-domain pulse: discard every word written so far. For a
//            FIFO whose flush is written in its write domain (TX). The
//            write pointer at the flush is handed to the read side with a
//            two-phase handshake. Until the read side has caught up, wlevel
//            counts only the words written after the flush. wfull keeps
//            counting the old words until their space is really free.
// While a flush is applied the read side reports empty and level 0.
// An instance uses one of the two flush inputs and ties the other to 0.
//======================================================================
module spi_async_fifo #(
  parameter int WIDTH = 32,
  parameter int DEPTH = 8                      // power of two, 4 or more
)(
  // write port
  input  logic                       wclk,
  input  logic                       wrst_n,
  input  logic                       w_clr,
  input  logic                       w_flush,    // discard everything written so far
  input  logic                       wen,
  input  logic [WIDTH-1:0]           wdata,
  output logic                       wfull,
  output logic [$clog2(DEPTH):0]     wlevel,

  // read port
  input  logic                       rclk,
  input  logic                       rrst_n,
  input  logic                       r_clr,
  input  logic                       r_flush,    // discard everything the read side can see
  output logic                       r_flushed,  // 1 cycle: a flush was taken and every word
                                                 // popped so far belongs to the flushed data
  input  logic                       ren,
  output logic [WIDTH-1:0]           rdata,
  output logic                       rempty,
  output logic [$clog2(DEPTH):0]     rlevel
);

  localparam int AW = $clog2(DEPTH);
  localparam logic [AW:0] PDEPTH = {1'b1, {AW{1'b0}}};   // DEPTH as a pointer distance

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
  // w_flush handshake: request toggle and target (write side), acknowledge
  // toggle (read side), and the synchronizers of the two toggles
  logic        w_freq, r_fack;
  logic [AW:0] w_ftgt;
  (* ASYNC_REG = "TRUE" *) logic rq1_freq, rq2_freq;
  (* ASYNC_REG = "TRUE" *) logic wq1_fack, wq2_fack;
  logic        wq3_fack;

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

  // ---- w_flush: hand the write pointer to the read side ----
  // w_ftgt is loaded in the same cycle as the w_freq toggle and is not
  // changed again until the read side has toggled r_fack back, so the read
  // side always samples a stable value. A flush written while one is still
  // open moves wlevel at once (w_vtgt) and is sent when the open one closes
  // (w_refl). r_fack crosses on three flops, one more than the read
  // pointer: when the request reads as closed, wq2_rgray already shows the
  // read pointer at or past the target.
  logic        w_fopen;                // request sent, not yet acknowledged
  logic [AW:0] w_vtgt;                 // write pointer at the latest flush
  logic        w_fview;                // wlevel counts only words written since w_vtgt
  logic        w_refl;                 // a later flush is waiting to be sent
  assign w_fopen = (w_freq != wq3_fack);

  always_ff @(posedge wclk or negedge wrst_n) begin
    if (!wrst_n) {wq3_fack, wq2_fack, wq1_fack} <= 3'b000;
    else         {wq3_fack, wq2_fack, wq1_fack} <= {wq2_fack, wq1_fack, r_fack};
  end

  always_ff @(posedge wclk or negedge wrst_n) begin
    if (!wrst_n) begin
      w_freq  <= 1'b0;
      w_ftgt  <= '0;
      w_vtgt  <= '0;
      w_fview <= 1'b0;
      w_refl  <= 1'b0;
    end else if (w_clr) begin
      // both pointers restart from 0: nothing to hide and nothing to send.
      // A request that is still open is closed by the read side's r_clr.
      w_vtgt  <= '0;
      w_fview <= 1'b0;
      w_refl  <= 1'b0;
    end else if (w_flush) begin
      w_vtgt  <= wbin;
      w_fview <= 1'b1;
      if (w_fopen) begin
        w_refl  <= 1'b1;                 // send it when the open one closes
      end else begin
        w_freq  <= ~w_freq;              // send now
        w_ftgt  <= wbin;
        w_refl  <= 1'b0;
      end
    end else if (!w_fopen) begin
      if (w_refl) begin
        w_freq  <= ~w_freq;              // send the flush that waited
        w_ftgt  <= w_vtgt;
        w_refl  <= 1'b0;
      end else begin
        w_fview <= 1'b0;                 // the read side has caught up
      end
    end
  end

  // Full is REGISTERED, not a continuous assign. A combinational wfull would
  // close a loop wfull -> wen -> wbin_nxt -> wgray_nxt -> wfull, because the
  // writer gates its own enable on full. Registering it is the standard
  // dual-clock FIFO construction and costs nothing in throughput.
  // Full counts every word still in memory, flushed or not: a flushed
  // word's slot is free only once the read side has stepped past it.
  logic wfull_val;
  assign wfull_val = (wgray_nxt == {~wq2_rgray[AW:AW-1], wq2_rgray[AW-2:0]});

  always_ff @(posedge wclk or negedge wrst_n) begin
    if      (!wrst_n) wfull <= 1'b0;
    else if (w_clr)   wfull <= 1'b0;
    else              wfull <= wfull_val;
  end

  assign wr_rbin = gray2bin(wq2_rgray);
  // Conservative (reads high). After w_flush: only the words written since,
  // counted from w_vtgt or from the read pointer, whichever is later.
  logic [AW:0] wlvl_mem, wlvl_new;
  assign wlvl_mem = wbin - wr_rbin;
  assign wlvl_new = wbin - w_vtgt;
  assign wlevel   = (w_fview && (wlvl_new < wlvl_mem)) ? wlvl_new : wlvl_mem;

  // ---------------- read side ----------------

  // ---- flush: step the read pointer to the target ----
  // Taken when the w_flush request arrives, or on r_flush. The read pointer
  // then steps one entry per clock up to the target, never past the
  // synchronized write pointer, and the read side reports empty meanwhile.
  // If the reader is already past a w_flush target (it popped a word written
  // after the flush before the request arrived), nothing more is discarded.
  logic        r_fopen;                // w_flush request arrived, not yet acknowledged
  logic        r_drain;                // stepping to r_dtgt
  logic        r_fserve;               // this drain serves a w_flush request
  logic [AW:0] r_dtgt;                 // flush target
  logic        r_behind;               // rbin is short of r_dtgt
  logic        r_step;

  always_ff @(posedge rclk or negedge rrst_n) begin
    if (!rrst_n) {rq2_freq, rq1_freq} <= 2'b00;
    else         {rq2_freq, rq1_freq} <= {rq1_freq, w_freq};
  end
  assign r_fopen  = (rq2_freq != r_fack);

  assign r_behind = ((rbin - r_dtgt) >= PDEPTH);
  assign r_step   = r_drain && r_behind && (rbin != rd_wbin);

  always_ff @(posedge rclk or negedge rrst_n) begin
    if (!rrst_n) begin
      r_drain   <= 1'b0;
      r_fserve  <= 1'b0;
      r_dtgt    <= '0;
      r_fack    <= 1'b0;
      r_flushed <= 1'b0;
    end else if (r_clr) begin
      r_drain   <= 1'b0;
      r_fserve  <= 1'b0;
      r_fack    <= rq2_freq;             // the clear empties the FIFO: close an open request
      r_flushed <= 1'b0;
    end else begin
      r_flushed <= 1'b0;
      if (r_drain) begin
        if (r_flush) begin
          r_dtgt   <= rd_wbin;           // another read-side flush: extend the target
        end else if (!r_behind) begin
          r_drain  <= 1'b0;              // target reached
          r_fserve <= 1'b0;
          if (r_fserve) r_fack <= rq2_freq;   // close the w_flush request
        end
      end else if (r_fopen) begin
        // w_ftgt has been stable since the request was sent
        r_drain   <= 1'b1;
        r_fserve  <= 1'b1;
        r_dtgt    <= w_ftgt;
        r_flushed <= ((rbin - w_ftgt) >= PDEPTH) || (rbin == w_ftgt);
      end else if (r_flush) begin
        r_drain   <= 1'b1;
        r_dtgt    <= rd_wbin;
        r_flushed <= 1'b1;
      end
    end
  end

  assign rbin_nxt  = rbin + {{AW{1'b0}}, (r_step || (ren && !rempty))};
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
  logic rempty_q, rempty_val;
  assign rempty_val = (rgray_nxt == rq2_wgray);

  always_ff @(posedge rclk or negedge rrst_n) begin
    if      (!rrst_n) rempty_q <= 1'b1;
    else if (r_clr)   rempty_q <= 1'b1;
    else              rempty_q <= rempty_val;
  end
  // reported empty while a flush is applied
  assign rempty = rempty_q || r_fopen || r_drain;
  assign rdata  = mem[rbin[AW-1:0]];             // first-word-fall-through

  assign rd_wbin = gray2bin(rq2_wgray);
  assign rlevel  = (r_fopen || r_drain) ? '0 : (rd_wbin - rbin);   // conservative (reads low)

endmodule
