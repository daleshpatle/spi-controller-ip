//======================================================================
// spi_slave_engine.sv - Slave-mode shift register and word-level handoff
//
// The shift register is clocked DIRECTLY by the external SCLK - there is
// no synchronizer in the bit-sampling path (Spec 5.1, 7.2). A completed
// word crosses into spi_ref_clk through a toggle/handshake, and the data
// bus is sampled directly because the handshake holds the word stable for
// the exchange. The async RX FIFO's gray pointers are a separate
// mechanism for the later SPI -> bus crossing.
//
// Sampling edge. CPHA=0 samples on the leading edge, CPHA=1 on the
// trailing edge; the leading edge is rising for CPOL=0 and falling for
// CPOL=1. Both cases reduce to the rising edge of
//        sclk_sample = sclk_i ^ cpol ^ cpha
// CPOL and CPHA are quasi-static (captured only while the FSM is idle),
// so this XOR cannot glitch during a frame. In an ASIC flow the XOR sits
// on a clock path and must be implemented with a clock-safe cell and
// constrained accordingly.
//
// DFS snapshot. The bit counter lives in the external-SCLK domain but
// DFS lives in cfg_active (spi_ref_clk). DFS is frozen into dfs_frame on
// the first SCLK edge of each frame, so the counter compares against a
// value that is constant for the whole frame. No multi-bit synchronizer
// is required - and none would be correct, because it would introduce
// bit skew across the DFS field.
//======================================================================
module spi_slave_engine
  import spi_pkg::*; #(
  parameter int DATA_WIDTH = 32
)(
  input  logic                   spi_clk,
  input  logic                   spi_rst_n,

    // Slave uses only dfs, lsbfirst, cpha and cpol; the other fields are
  // master-only or handled in the top level
  /* verilator lint_off UNUSEDSIGNAL */
  input  cfg_t                   cfg,
  /* verilator lint_on UNUSEDSIGNAL */
  input  logic                   swrst_active,

  // external serial pins (slave direction)
  input  logic                   sclk_i,
  input  logic                   mosi_i,
  output logic                   miso_o,
  input  logic                   cs_n_i,

  // TX FIFO read port (SPI side)
  input  logic                   tx_empty,
  input  logic [DATA_WIDTH-1:0]  tx_data,
  output logic                   tx_ren,

  // RX FIFO write port (SPI side)
  input  logic                   rx_full,
  output logic                   rx_wen,
  output logic [DATA_WIDTH-1:0]  rx_data,

  // status / events
  output logic                   busy,
  output logic                   evt_done,
  output logic                   evt_rx_overrun,
  output logic                   evt_tx_underrun,
  output logic                   evt_frame_err
);


  // ------------------------------------------------------------------
  // TX word staging in the spi_ref_clk domain
  // ------------------------------------------------------------------
  logic [DATA_WIDTH-1:0] tx_hold;
  logic                  tx_hold_vld;

  // ------------------------------------------------------------------
  // External-SCLK domain
  // ------------------------------------------------------------------
  logic sclk_sample;
  assign sclk_sample = sclk_i ^ cfg.cpol ^ cfg.cpha;

  logic                  cs_inactive;
  assign cs_inactive = cs_n_i;          // active-low chip-select

  logic [DATA_WIDTH-1:0] rx_sr_x;
  logic [DATA_WIDTH-1:0] tx_sr_x;
  logic [5:0]            bitcnt_x;
  logic [5:0]            dfs_frame;     // DFS frozen on the first edge of a frame
  logic                  rx_active_x;   // 0 until the first sample edge of a frame
  logic                  tx_loaded_x;   // 0 until the first drive edge of a frame
  logic                  tx_pend_x;     // a new word was loaded on the last drive edge
  logic                  tx_pend_data_x; // ...and it came from tx_hold, not the idle value
  logic                  tx_take_tgl_x; // toggles when a staged TX word has really been sent
  logic                  frame_tgl_x;   // toggles on each completed word
  logic [DATA_WIDTH-1:0] hold_x;        // completed word, held stable
  logic                  underrun_tgl_x;

  // DFS for this edge: live cfg on the first edge of a frame, frozen copy after
  logic [5:0] dfs_now;
  assign dfs_now = rx_active_x ? dfs_frame : cfg.dfs;

  logic last_bit_x;                     // this sample edge completes a word
  assign last_bit_x = !cs_inactive && (bitcnt_x == dfs_now - 6'd1);

  logic [DATA_WIDTH-1:0] tx_word;       // next TX word, aligned for shifting out
  assign tx_word = tx_hold_vld
                 ? (cfg.lsbfirst ? tx_hold : (tx_hold << (DATA_WIDTH - 32'(cfg.dfs))))
                 : {DATA_WIDTH{1'b1}};  // idle value on underrun

    // CPHA=0: the master samples the first bit on the first sample edge, straight
  // from tx_word. first_data_x records whether that bit came from staged data or
  // from the idle value, so the first drive edge shifts the SAME word and counts it
  // the same way even if tx_hold is loaded in between (TX FIFO written mid-frame).
  logic                  first_data_x;
  logic [DATA_WIDTH-1:0] tx_first;      // CPHA=0: the word whose first bit is already out
  assign tx_first = first_data_x ? tx_word : {DATA_WIDTH{1'b1}};

  logic [DATA_WIDTH-1:0] tx_cur;        // what MISO is shifting out right now
  assign tx_cur = tx_loaded_x ? tx_sr_x
                : (rx_active_x ? tx_first : tx_word);

  // Per-frame state: async reset on power-on OR CS_n high, constant 0 only.
  // The two toggles: power-on reset only (a CS_n reset would look like an event).
  // CS_n is also used as data (gating, BUSY/FRAME_ERR sync); release timing is
  // met by the master's CS-to-first-SCLK setup time (t_CSS).
  /* verilator lint_off SYNCASYNCNET */
  // ---- receive: sample on the rising edge of sclk_sample ----
  always_ff @(posedge sclk_sample or posedge cs_inactive or negedge spi_rst_n) begin
    if (!spi_rst_n)       {rx_active_x, dfs_frame, bitcnt_x, rx_sr_x} <= '0;
    else if (cs_inactive) {rx_active_x, dfs_frame, bitcnt_x, rx_sr_x} <= '0;
    else begin
      rx_active_x <= 1'b1;
      if (!rx_active_x) dfs_frame <= cfg.dfs;   // freeze DFS for the whole frame
      rx_sr_x  <= cfg.lsbfirst ? {mosi_i, rx_sr_x[DATA_WIDTH-1:1]}
                               : {rx_sr_x[DATA_WIDTH-2:0], mosi_i};
      bitcnt_x <= last_bit_x ? 6'd0 : bitcnt_x + 6'd1;
    end
  end

  // CPHA=0 first word: data or idle, as seen by the master on the first sample edge
  always_ff @(posedge sclk_sample or posedge cs_inactive or negedge spi_rst_n) begin
    if (!spi_rst_n)        first_data_x <= 1'b0;
    else if (cs_inactive)  first_data_x <= 1'b0;
    else if (!rx_active_x) first_data_x <= tx_hold_vld;
  end

  // Word in progress, for FRAME_ERR: set by a sample edge that does not finish a
  // word, cleared by one that does. Power-on reset ONLY - if CS_n cleared it, it
  // would fall at the same moment CS_n rises, and the two equal-length 2-FF
  // synchronizers would deliver both changes in the same spi_clk cycle.
  logic word_open_x;
  always_ff @(posedge sclk_sample or negedge spi_rst_n) begin
    if (!spi_rst_n)        word_open_x <= 1'b0;
    else if (!cs_inactive) word_open_x <= !last_bit_x;
  end

  // completed word and its toggle: power-on reset only
  always_ff @(posedge sclk_sample or negedge spi_rst_n) begin
    if (!spi_rst_n) begin
      frame_tgl_x <= 1'b0;
      hold_x      <= '0;
    end else if (last_bit_x) begin
      hold_x      <= cfg.lsbfirst
                   ? ({mosi_i, rx_sr_x[DATA_WIDTH-1:1]} >> (DATA_WIDTH - 32'(dfs_now)))
                   : ({rx_sr_x[DATA_WIDTH-2:0], mosi_i} & ((1 << dfs_now) - 1));
      frame_tgl_x <= ~frame_tgl_x;      // request: a word is ready
    end
  end

    // ---- transmit: drive on the opposite edge ----
  // Before the first drive edge MISO shows the first bit straight from tx_word.
  // A new word is loaded (bit 0 presented, not advanced) on the drive edge
  // right after the sample edge that completed the previous word, and on the
  // first drive edge of a frame for CPHA=1. For CPHA=0 the first drive edge
  // loads the first word already advanced by one bit.
  logic tx_boundary_x;                  // previous sample edge completed a word
  assign tx_boundary_x = tx_loaded_x && rx_active_x && (bitcnt_x == 6'd0);

  logic [DATA_WIDTH-1:0] tx_shifted;    // tx_cur advanced by one bit
  assign tx_shifted = cfg.lsbfirst ? {1'b1, tx_cur[DATA_WIDTH-1:1]}
                                   : {tx_cur[DATA_WIDTH-2:0], 1'b1};

  always_ff @(negedge sclk_sample or posedge cs_inactive or negedge spi_rst_n) begin
    if (!spi_rst_n)       {tx_loaded_x, tx_pend_x, tx_pend_data_x, tx_sr_x} <= '0;
    else if (cs_inactive) {tx_loaded_x, tx_pend_x, tx_pend_data_x, tx_sr_x} <= '0;
    else begin
      tx_loaded_x <= 1'b1;
      if (tx_boundary_x || (cfg.cpha && !tx_loaded_x)) begin
        tx_sr_x        <= tx_word;              // present bit 0 of a new word
        tx_pend_x      <= 1'b1;                 // sent once the master clocks it
        tx_pend_data_x <= tx_hold_vld;
      end else begin
        tx_sr_x        <= tx_shifted;
        tx_pend_x      <= 1'b0;
      end
    end
  end

  // A word counts as SENT on the drive edge after it was presented: the master
  // sampled its first bit in between. A word presented on the last drive edge
  // of a frame is never clocked, so it stays staged for the next frame.
  // CPHA=0 first word: its first bit is sampled before the first drive edge.
  logic tx_sent_x, tx_sent_data_x;
  assign tx_sent_x      = !cs_inactive && (tx_pend_x || (!cfg.cpha && !tx_loaded_x));
    assign tx_sent_data_x = tx_pend_x ? tx_pend_data_x : first_data_x;

  // sent / underrun toggles: power-on reset only. Underrun = the master clocked
  // a word while no TX data was staged - once per word, not once per bit.
  always_ff @(negedge sclk_sample or negedge spi_rst_n) begin
    if (!spi_rst_n) begin
      tx_take_tgl_x  <= 1'b0;
      underrun_tgl_x <= 1'b0;
    end else if (tx_sent_x) begin
      if (tx_sent_data_x) tx_take_tgl_x  <= ~tx_take_tgl_x;
      else                underrun_tgl_x <= ~underrun_tgl_x;
    end
  end

  /* verilator lint_on SYNCASYNCNET */

  assign miso_o = cs_inactive ? 1'b1
                : (cfg.lsbfirst ? tx_cur[0] : tx_cur[DATA_WIDTH-1]);

  // partial word present - used to flag a framing error at CS deassertion
  logic partial_x;
    assign partial_x = word_open_x;       // not cleared by CS_n (see word_open_x)

  // ------------------------------------------------------------------
  // Handoff into the spi_ref_clk domain
  // ------------------------------------------------------------------
  // The toggle is generated in the external-SCLK domain above, so only the
  // destination half of the event synchronizer is needed here: 2-FF sync
  // plus a ONE-cycle edge detect. Not the 2-cycle output of spi_event_sync
   // (Spec 5.2): word_rdy is the RX FIFO write enable, tx_take clears the TX
  // staging register, and evt_done / evt_rx_overrun / evt_tx_underrun feed
  // toggle-type event synchronizers. All of them act once per cycle, so a
  // 2-cycle pulse would write every word twice and toggle every event twice
  // (the second toggle cancels the first, so a slow pclk could miss it).
  logic word_rdy;
  (* ASYNC_REG = "TRUE" *) logic fq1, fq2;
  logic fqd1;
  always_ff @(posedge spi_clk or negedge spi_rst_n) begin
    if (!spi_rst_n) {fqd1, fq2, fq1} <= 3'b000;
    else            {fqd1, fq2, fq1} <= {fq2, fq1, frame_tgl_x};
  end
  assign word_rdy = fq2 ^ fqd1;             // exactly one spi_clk cycle per word

  // underrun event synchronized the same way (one cycle per toggle)
  (* ASYNC_REG = "TRUE" *) logic uq1, uq2;
  logic uqd1;
  always_ff @(posedge spi_clk or negedge spi_rst_n) begin
    if (!spi_rst_n) {uqd1, uq2, uq1} <= 3'b000;
    else            {uqd1, uq2, uq1} <= {uq2, uq1, underrun_tgl_x};
  end
  assign evt_tx_underrun = uq2 ^ uqd1;

  // "staged TX word was sent" event, synchronized the same way. Back-to-back
  // words need the next word staged before the next word boundary:
  // (DFS-1) SCLK periods > 4 spi_clk periods (DFS 4, spi_clk 29 MHz: SCLK
  // up to ~21 MHz; DFS 8: beyond 50 MHz).
  (* ASYNC_REG = "TRUE" *) logic tq1, tq2;
  logic tqd1;
  always_ff @(posedge spi_clk or negedge spi_rst_n) begin
    if (!spi_rst_n) {tqd1, tq2, tq1} <= 3'b000;
    else            {tqd1, tq2, tq1} <= {tq2, tq1, tx_take_tgl_x};
  end
  logic tx_take;
  assign tx_take = tq2 ^ tqd1;

  // chip-select and partial-word status, synchronized for framing errors
  logic cs_n_sync, cs_n_sync_q, partial_sync;
  spi_lvl_sync #(.INIT(1'b1)) u_cs_sync  (.clk(spi_clk), .rst_n(spi_rst_n),
                                          .d(cs_n_i),    .q(cs_n_sync));
  spi_lvl_sync #(.INIT(1'b0)) u_par_sync (.clk(spi_clk), .rst_n(spi_rst_n),
                                          .d(partial_x), .q(partial_sync));

  always_ff @(posedge spi_clk or negedge spi_rst_n) begin
    if (!spi_rst_n) cs_n_sync_q <= 1'b1;
    else            cs_n_sync_q <= cs_n_sync;
  end

  // FRAME_ERR: chip-select deasserted with a partial word in the shift
  // register. The partial word is discarded - storing it would misalign
  // every subsequent word in the stream (Spec 7.2).
   // Evaluated one spi_clk cycle after the CS_n rise is seen: word_open_x last
  // changed BEFORE CS_n rose, so by then partial_sync has settled.
  logic cs_rise_q;
  always_ff @(posedge spi_clk or negedge spi_rst_n) begin
    if (!spi_rst_n) cs_rise_q <= 1'b0;
    else            cs_rise_q <= cs_n_sync && !cs_n_sync_q;
  end
  assign evt_frame_err = cs_rise_q && partial_sync && !swrst_active;

  // ---- push the completed word into the RX FIFO ----
  assign rx_data        = hold_x;
  assign rx_wen         = word_rdy && !rx_full && !swrst_active;
  assign evt_rx_overrun = word_rdy &&  rx_full && !swrst_active;
  assign evt_done       = word_rdy && !swrst_active;

  // ---- keep the TX holding register fed ----
  always_ff @(posedge spi_clk or negedge spi_rst_n) begin
    if (!spi_rst_n) begin
      tx_hold     <= '0;
      tx_hold_vld <= 1'b0;
    end else if (swrst_active) begin
      tx_hold     <= '0;
      tx_hold_vld <= 1'b0;
    end else begin
      if (!tx_hold_vld && !tx_empty) begin
        tx_hold     <= tx_data;
        tx_hold_vld <= 1'b1;
        end else if (tx_hold_vld && tx_take) begin
        // the staged word has really gone out on MISO; stage the next one
        tx_hold_vld <= 1'b0;
      end
    end
  end

  assign tx_ren = (!tx_hold_vld && !tx_empty && !swrst_active);
  assign busy   = !cs_n_sync;

endmodule
