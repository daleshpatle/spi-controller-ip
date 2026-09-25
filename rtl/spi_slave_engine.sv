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
// DFS lives in cfg_active (spi_ref_clk). DFS is frozen into dfs_frame
// while chip-select is inactive, so the counter compares against a value
// that is constant for the whole frame. No multi-bit synchronizer is
// required - and none would be correct, because it would introduce bit
// skew across the DFS field.
//======================================================================
module spi_slave_engine
  import spi_pkg::*; #(
  parameter int DATA_WIDTH = 32
)(
  input  logic                   spi_clk,
  input  logic                   spi_rst_n,

  input  cfg_t                   cfg,
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
  logic [5:0]            dfs_frame;     // DFS frozen at frame start
  logic                  frame_tgl_x;   // toggles on each completed word
  logic [DATA_WIDTH-1:0] hold_x;        // completed word, held stable
  logic                  tx_started_x;
  logic                  underrun_tgl_x;

  // CS_n is deliberately used two ways: as the ASYNC clear of the SCLK-domain
  // frame logic below (SCLK stops when CS deasserts, so no clock edge is left
  // for a synchronous reset), and as DATA into a 2-FF synchronizer for BUSY /
  // FRAME_ERR. Reset release (CS asserting) is async to SCLK; recovery timing
  // is met by the SPI CS-to-first-SCLK setup time (t_CSS) the master guarantees.
  /* verilator lint_off SYNCASYNCNET */
  // ---- receive: sample on the rising edge of sclk_sample ----
  always_ff @(posedge sclk_sample or posedge cs_inactive) begin
    if (cs_inactive) begin
      rx_sr_x     <= '0;
      bitcnt_x    <= 6'd0;
      dfs_frame   <= cfg.dfs;           // freeze DFS for the whole frame
      frame_tgl_x <= frame_tgl_x;       // preserved across frames
      hold_x      <= hold_x;
    end else begin
      rx_sr_x <= cfg.lsbfirst ? {mosi_i, rx_sr_x[DATA_WIDTH-1:1]}
                              : {rx_sr_x[DATA_WIDTH-2:0], mosi_i};
      if (bitcnt_x == dfs_frame - 6'd1) begin
        bitcnt_x    <= 6'd0;
        hold_x      <= cfg.lsbfirst
                     ? ({mosi_i, rx_sr_x[DATA_WIDTH-1:1]} >> (DATA_WIDTH - 32'(dfs_frame)))
                     : ({rx_sr_x[DATA_WIDTH-2:0], mosi_i} & ((1 << dfs_frame) - 1));
        frame_tgl_x <= ~frame_tgl_x;    // request: a word is ready
      end else begin
        bitcnt_x <= bitcnt_x + 6'd1;
      end
    end
  end

  // ---- transmit: drive on the opposite edge ----
  // At chip-select deassertion the shift register is pre-loaded, so the
  // first bit is already present on MISO before the first clock edge.
  // That is what CPHA=0 requires. For CPHA=1 the first drive edge only
  // presents the bit without advancing (tx_started_x).
  always_ff @(negedge sclk_sample or posedge cs_inactive) begin
    if (cs_inactive) begin
      tx_sr_x        <= tx_hold_vld
                      ? (cfg.lsbfirst ? tx_hold : (tx_hold << (DATA_WIDTH - 32'(cfg.dfs))))
                      : {DATA_WIDTH{1'b1}};       // idle value on underrun
      tx_started_x   <= 1'b0;
      underrun_tgl_x <= underrun_tgl_x;
    end else begin
      if (cfg.cpha && !tx_started_x) begin
        tx_started_x <= 1'b1;                     // present bit 0, do not advance
      end else begin
        tx_sr_x <= cfg.lsbfirst ? {1'b1, tx_sr_x[DATA_WIDTH-1:1]}
                                : {tx_sr_x[DATA_WIDTH-2:0], 1'b1};
      end
      if (!tx_hold_vld) underrun_tgl_x <= ~underrun_tgl_x;
    end
  end

  /* verilator lint_on SYNCASYNCNET */

  assign miso_o = cs_inactive ? 1'b1
                : (cfg.lsbfirst ? tx_sr_x[0] : tx_sr_x[DATA_WIDTH-1]);

  // partial word present - used to flag a framing error at CS deassertion
  logic partial_x;
  assign partial_x = (bitcnt_x != 6'd0);

  // ------------------------------------------------------------------
  // Handoff into the spi_ref_clk domain
  // ------------------------------------------------------------------
  // The toggle is generated in the external-SCLK domain above, so only the
  // destination half of the event synchronizer is needed here: 2-FF sync
  // plus the 2-cycle edge detect of Spec 5.2.
  logic word_rdy;
  (* ASYNC_REG = "TRUE" *) logic fq1, fq2;
  logic fqd1, fqd2;
  always_ff @(posedge spi_clk or negedge spi_rst_n) begin
    if (!spi_rst_n) {fqd2, fqd1, fq2, fq1} <= 4'b0000;
    else            {fqd2, fqd1, fq2, fq1} <= {fqd1, fq2, fq1, frame_tgl_x};
  end
  assign word_rdy = (fq2 ^ fqd1) | (fqd1 ^ fqd2);

  // underrun event synchronized the same way
  (* ASYNC_REG = "TRUE" *) logic uq1, uq2;
  logic uqd1, uqd2;
  always_ff @(posedge spi_clk or negedge spi_rst_n) begin
    if (!spi_rst_n) {uqd2, uqd1, uq2, uq1} <= 4'b0000;
    else            {uqd2, uqd1, uq2, uq1} <= {uqd1, uq2, uq1, underrun_tgl_x};
  end
  assign evt_tx_underrun = (uq2 ^ uqd1) | (uqd1 ^ uqd2);

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
  assign evt_frame_err = (cs_n_sync && !cs_n_sync_q) && partial_sync && !swrst_active;

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
      end else if (tx_hold_vld && word_rdy) begin
        // the frame just consumed the staged word; stage the next one
        tx_hold_vld <= 1'b0;
      end
    end
  end

  assign tx_ren = (!tx_hold_vld && !tx_empty && !swrst_active);
  assign busy   = !cs_n_sync;

endmodule
