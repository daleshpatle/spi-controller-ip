//======================================================================
// spi_regfile.sv - APB slave, register file, interrupt and DMA logic
//
// Zero-wait-state slave: pready is tied high and the core never stalls
// the bus (Spec 7.6). Errors are reported through PSLVERR instead, which
// completes in bounded time and cannot deadlock a shared APB segment.
//
// SPI_CTRL is always writable regardless of FSM state; configuration
// writes update the register file immediately and reach the SPI domain
// only at the next capture (Spec 5.4). The only PSLVERR raised on
// SPI_CTRL is for an out-of-range DFS value (Spec 6.2).
//======================================================================
module spi_regfile
  import spi_pkg::*; #(
  parameter int   DATA_WIDTH     = 32,
  parameter int   FIFO_DEPTH     = 8,
  parameter int   APB_ADDR_WIDTH = 8,
  parameter logic DEF_CPOL       = 1'b0,
  parameter logic DEF_CPHA       = 1'b0
)(
  input  logic                        pclk,
  input  logic                        prst_n,

  // APB
  input  logic                        psel,
  input  logic                        penable,
  input  logic                        pwrite,
  input  logic [APB_ADDR_WIDTH-1:0]   paddr,
  input  logic [31:0]                 pwdata,
  output logic [31:0]                 prdata,
  output logic                        pready,
  output logic                        pslverr,

  // configuration out to the latch
  output cfg_t                        cfg_bus,
  output logic                        cfg_commit,     // 1-cycle commit pulse
  output logic                        swrst_write,

  // TX FIFO write port (bus side)
  output logic                        tx_wen,
  output logic [DATA_WIDTH-1:0]       tx_wdata,
  input  logic                        tx_full,
  input  logic [$clog2(FIFO_DEPTH):0] tx_level,

  // RX FIFO read port (bus side)
  output logic                        rx_ren,
  input  logic [DATA_WIDTH-1:0]       rx_rdata,
  input  logic                        rx_empty,
  input  logic [$clog2(FIFO_DEPTH):0] rx_level,

  // FIFO flush requests
  output logic                        tx_flush,
  output logic                        rx_flush,

  // status in from the SPI domain (already synchronized)
  input  logic                        busy_sync,
  input  logic                        tx_stall_sync,
  input  logic                        rst_busy,

  // events in from the SPI domain (already synchronized 1-cycle pulses)
  input  logic                        evt_done,
  input  logic                        evt_tx_underrun,
  input  logic                        evt_rx_overrun,
  input  logic                        evt_frame_err,

  // outputs
  output logic                        irq,
  output logic                        tx_dreq,
  output logic                        rx_dreq
);


  localparam int LW = $clog2(FIFO_DEPTH);

  // ---------------- register storage ----------------
  logic        r_en, r_mode, r_cpol, r_cpha, r_lsbfirst, r_loopback;
  logic        r_txdmaen, r_rxdmaen;
  logic [5:0]  r_dfs;
  logic [7:0]  r_clkdiv;
  logic [1:0]  r_cs_sel;
  logic        r_cs_auto;
  logic [5:0]  r_cs_setup, r_cs_hold;
  logic [7:0]  r_tx_wm_lvl, r_rx_wm_lvl;
  logic [IRQ_N-1:0] r_ie, r_is;

  // sticky status
  logic s_tx_underrun, s_rx_overrun, s_frame_err;

  // ---------------- APB decode ----------------
  logic acc, wr, rd;
  assign acc = psel && penable;
  assign wr  = acc &&  pwrite;
  assign rd  = acc && !pwrite;

  logic [7:0] off;
  assign off = paddr[7:0];

  logic is_ctrl, is_stat, is_txdata, is_rxdata, is_clkdiv;
  logic is_ie, is_is, is_csctrl, is_fifoctrl, is_fifolvl, is_mapped;

  assign is_ctrl     = (off == ADDR_CTRL);
  assign is_stat     = (off == ADDR_STAT);
  assign is_txdata   = (off == ADDR_TXDATA);
  assign is_rxdata   = (off == ADDR_RXDATA);
  assign is_clkdiv   = (off == ADDR_CLKDIV);
  assign is_ie       = (off == ADDR_IE);
  assign is_is       = (off == ADDR_IS);
  assign is_csctrl   = (off == ADDR_CSCTRL);
  assign is_fifoctrl = (off == ADDR_FIFOCTRL);
  assign is_fifolvl  = (off == ADDR_FIFOLVL);
  assign is_mapped   = is_ctrl | is_stat | is_txdata | is_rxdata | is_clkdiv
                     | is_ie   | is_is   | is_csctrl | is_fifoctrl | is_fifolvl;

  // DFS range check (Spec 6.2): legal 4..32 and not greater than DATA_WIDTH
  logic [5:0] dfs_wr;
  logic       dfs_bad;
  assign dfs_wr  = pwdata[13:8];
  assign dfs_bad = is_ctrl && wr &&
                   ((dfs_wr < 6'd4) || (dfs_wr > 6'd32) || (32'(dfs_wr) > DATA_WIDTH));

  // ---------------- PSLVERR (Spec 7.6) ----------------
  // Each FIFO-port guard is qualified by its OWN address decode. This is
  // what keeps SPI_CTRL - and therefore SWRST - always writable even while
  // RST_BUSY is set. A guard keyed on rst_busy alone would reject the very
  // write that recovers a hung core.
  assign pslverr = acc && (
       (!is_mapped)                                            // unmapped
     || (wr && (is_stat || is_rxdata || is_fifolvl))           // write to RO
     || (rd &&  is_txdata)                                     // read of WO
     || dfs_bad                                                // out-of-range DFS
     || (wr && is_txdata && (tx_full  || rst_busy))            // TX FIFO guard
     || (rd && is_rxdata && (rx_empty || rst_busy))            // RX FIFO guard
  );

  assign pready = 1'b1;                 // never inserts wait states

  // ---------------- FIFO access ----------------
  assign tx_wen   = wr && is_txdata && !tx_full  && !rst_busy;
  assign tx_wdata = pwdata[DATA_WIDTH-1:0];
  assign rx_ren   = rd && is_rxdata && !rx_empty && !rst_busy;

  // ---------------- watermark and DMA ----------------
  logic tx_wm, rx_wm;
  assign tx_wm = (tx_level <= {1'b0, r_tx_wm_lvl[LW-1:0]});   // space available
  assign rx_wm = (rx_level >= {1'b0, r_rx_wm_lvl[LW-1:0]}) && (rx_level != '0);

  assign tx_dreq = tx_wm && r_txdmaen;
  assign rx_dreq = rx_wm && r_rxdmaen;

  // ---------------- register writes ----------------
  logic cfg_wr_hit;
  assign cfg_wr_hit = wr && (is_ctrl || is_clkdiv || is_csctrl) && !pslverr;

  always_ff @(posedge pclk or negedge prst_n) begin
    if (!prst_n) begin
      r_en        <= 1'b0;
      r_mode      <= 1'b0;
      r_cpol      <= DEF_CPOL;
      r_cpha      <= DEF_CPHA;
      r_lsbfirst  <= 1'b0;
      r_loopback  <= 1'b0;
      r_txdmaen   <= 1'b0;
      r_rxdmaen   <= 1'b0;
      r_dfs       <= 6'd8;
      r_clkdiv    <= 8'hFF;
      r_cs_sel    <= 2'd0;
      r_cs_auto   <= 1'b1;
      r_cs_setup  <= 6'h3F;
      r_cs_hold   <= 6'h3F;
      r_tx_wm_lvl <= 8'(FIFO_DEPTH/2);
      r_rx_wm_lvl <= 8'(FIFO_DEPTH/2);
      r_ie        <= '0;
    end else begin
      if (wr && is_ctrl && !pslverr) begin
        r_en       <= pwdata[0];
        r_mode     <= pwdata[1];
        r_cpol     <= pwdata[2];
        r_cpha     <= pwdata[3];
        r_lsbfirst <= pwdata[4];
        r_loopback <= pwdata[5];
        r_txdmaen  <= pwdata[6];
        r_rxdmaen  <= pwdata[7];
        r_dfs      <= dfs_wr;            // range already validated
      end
      if (wr && is_clkdiv && !pslverr) r_clkdiv <= pwdata[7:0];
      if (wr && is_csctrl && !pslverr) begin
        r_cs_sel   <= pwdata[1:0];
        r_cs_auto  <= pwdata[2];
        r_cs_setup <= pwdata[8:3];
        r_cs_hold  <= pwdata[14:9];
      end
      if (wr && is_fifoctrl && !pslverr) begin
        r_tx_wm_lvl <= pwdata[15:8];
        // RX_WM_LVL of 0 would assert permanently and is not permitted
        r_rx_wm_lvl <= (pwdata[23:16] == 8'd0) ? 8'd1 : pwdata[23:16];
      end
      if (wr && is_ie && !pslverr) r_ie <= pwdata[IRQ_N-1:0];
    end
  end

  // SWRST is write-1-to-trigger and always reads back 0, so a
  // read-modify-write of SPI_CTRL can never re-trigger the reset.
  assign swrst_write = wr && is_ctrl && pwdata[14] && !pslverr;

  // FIFO flush: self-clearing, honoured only while the core is idle
  assign tx_flush = wr && is_fifoctrl && pwdata[0] && !busy_sync && !pslverr;
  assign rx_flush = wr && is_fifoctrl && pwdata[1] && !busy_sync && !pslverr;

  // ---------------- commit pulse ----------------
  // One pclk cycle on any write to a latched configuration register.
  // Fires on every such write regardless of FSM state; the configuration
  // latch arms it until the FSM is idle (Spec 5.3).
  logic cfg_commit_q;
  always_ff @(posedge pclk or negedge prst_n) begin
    if (!prst_n) cfg_commit_q <= 1'b0;
    else         cfg_commit_q <= cfg_wr_hit;
  end
  assign cfg_commit = cfg_commit_q;

  // ---------------- interrupt status ----------------
  // SPI_IS latches every source independently of SPI_IE. SPI_IE gates only
  // the irq output, which is what lets polled operation observe events -
  // including error events - with interrupts disabled (Spec 6.6).
  logic [IRQ_N-1:0] is_set;

  always_comb begin
    is_set             = '0;
    // The watermarks are LEVEL conditions, so they are latched while the
    // condition holds. A W1C write cannot clear them until the FIFO has
    // actually been serviced - which is the useful behaviour, since the
    // condition is "action still needed".
    is_set[IRQ_TXWM]   = tx_wm;
    is_set[IRQ_RXWM]   = rx_wm;
    is_set[IRQ_TXUNF]  = evt_tx_underrun;
    is_set[IRQ_RXOVF]  = evt_rx_overrun;
    is_set[IRQ_DONE]   = evt_done;
    is_set[IRQ_FRERR]  = evt_frame_err;
  end

  logic is_w1c;
  assign is_w1c = wr && is_is && !pslverr;

  always_ff @(posedge pclk or negedge prst_n) begin
    if (!prst_n) begin
      r_is          <= '0;
      s_tx_underrun <= 1'b0;
      s_rx_overrun  <= 1'b0;
      s_frame_err   <= 1'b0;
    end else if (swrst_write) begin
      r_is          <= '0;               // SWRST clears SPI_IS (Spec 7.5 step 2)
      s_tx_underrun <= 1'b0;
      s_rx_overrun  <= 1'b0;
      s_frame_err   <= 1'b0;
    end else begin
      for (int i = 0; i < IRQ_N; i++) begin
        if (is_set[i])                       r_is[i] <= 1'b1;
        else if (is_w1c && pwdata[i])        r_is[i] <= 1'b0;
      end
      // sticky SPI_STAT flags track the same sources
      if (evt_tx_underrun)                      s_tx_underrun <= 1'b1;
      else if (is_w1c && pwdata[IRQ_TXUNF])     s_tx_underrun <= 1'b0;
      if (evt_rx_overrun)                       s_rx_overrun  <= 1'b1;
      else if (is_w1c && pwdata[IRQ_RXOVF])     s_rx_overrun  <= 1'b0;
      if (evt_frame_err)                        s_frame_err   <= 1'b1;
      else if (is_w1c && pwdata[IRQ_FRERR])     s_frame_err   <= 1'b0;
    end
  end

  // ---------------- irq with the DMA interlock (Spec 6.6) ----------------
  // The interlock is per-direction, suppresses only the irq contribution,
  // and never affects SPI_IS latching or the error sources.
  assign irq = |(r_is[IRQ_FRERR:IRQ_TXUNF] & r_ie[IRQ_FRERR:IRQ_TXUNF])
             | (r_is[IRQ_TXWM] & r_ie[IRQ_TXWM] & ~r_txdmaen)
             | (r_is[IRQ_RXWM] & r_ie[IRQ_RXWM] & ~r_rxdmaen);

  // ---------------- configuration bundle out ----------------
  always_comb begin
    cfg_bus.en       = r_en;
    cfg_bus.mode     = r_mode;
    cfg_bus.cpol     = r_cpol;
    cfg_bus.cpha     = r_cpha;
    cfg_bus.lsbfirst = r_lsbfirst;
    cfg_bus.loopback = r_loopback;
    cfg_bus.dfs      = r_dfs;
    cfg_bus.clkdiv   = r_clkdiv;
    cfg_bus.cs_sel   = r_cs_sel;
    cfg_bus.cs_auto  = r_cs_auto;
    cfg_bus.cs_setup = r_cs_setup;
    cfg_bus.cs_hold  = r_cs_hold;
  end

  // ---------------- read mux ----------------
  logic [31:0] stat_val;
  always_comb begin
    stat_val        = 32'd0;
    stat_val[0]     = busy_sync;
    stat_val[1]     = (tx_level == '0);
    stat_val[2]     = tx_full;
    stat_val[3]     = rx_empty;
    stat_val[4]     = (32'(rx_level) == FIFO_DEPTH);
    stat_val[5]     = tx_wm;
    stat_val[6]     = rx_wm;
    stat_val[7]     = s_tx_underrun;
    stat_val[8]     = s_rx_overrun;
    stat_val[9]     = s_frame_err;
    stat_val[10]    = tx_stall_sync;
    stat_val[11]    = rst_busy;
  end

  always_comb begin
    prdata = 32'd0;
    unique case (off)
      ADDR_CTRL:     prdata = {17'd0, 1'b0,          // SWRST always reads 0
                               r_dfs, r_rxdmaen, r_txdmaen, r_loopback,
                               r_lsbfirst, r_cpha, r_cpol, r_mode, r_en};
      ADDR_STAT:     prdata = stat_val;
      ADDR_RXDATA:   prdata = (rx_empty || rst_busy) ? 32'd0
                                                     : {{(32-DATA_WIDTH){1'b0}}, rx_rdata};
      ADDR_CLKDIV:   prdata = {24'd0, r_clkdiv};
      ADDR_IE:       prdata = {{(32-IRQ_N){1'b0}}, r_ie};
      ADDR_IS:       prdata = {{(32-IRQ_N){1'b0}}, r_is};
      ADDR_CSCTRL:   prdata = {17'd0, r_cs_hold, r_cs_setup, r_cs_auto, r_cs_sel};
      ADDR_FIFOCTRL: prdata = {8'd0, r_rx_wm_lvl, r_tx_wm_lvl, 6'd0, 2'b00};
      ADDR_FIFOLVL:  prdata = {16'd0,
                               {(8-LW-1){1'b0}}, rx_level,
                               {(8-LW-1){1'b0}}, tx_level};
      default:       prdata = 32'd0;
    endcase
  end

endmodule
