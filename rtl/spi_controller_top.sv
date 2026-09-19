//======================================================================
// spi_controller_top.sv - SPI Controller IP top level
//
// Two primary clock domains bridged by dual-clock asynchronous FIFOs:
//   bus domain (pclk)       : APB slave, register file, interrupt and DMA
//   SPI domain (spi_ref_clk): transfer FSM, shift engine, SCLK generator
// In slave mode the external SCLK forms a third domain, handled locally
// inside spi_slave_engine.
//
// Multi-bit data crosses ONLY through the async FIFOs. Single-bit control
// and status cross as event-synchronized pulses. The wide configuration
// bus is captured, never continuously sampled.
//
// INTEGRATION: presetn and spi_rst_n must be asserted TOGETHER. They are
// separated only so each can be released synchronously to its own clock;
// asserting one alone leaves the async FIFOs with desynchronized pointers.
//======================================================================
module spi_controller_top
  import spi_pkg::*; #(
  parameter int   DATA_WIDTH     = 32,
  parameter int   FIFO_DEPTH     = 8,
  parameter int   NUM_CS         = 4,
  parameter int   APB_ADDR_WIDTH = 8,
  parameter logic DEF_CPOL       = 1'b0,
  parameter logic DEF_CPHA       = 1'b0
)(
  // clocks and resets
  input  logic                      pclk,
  input  logic                      presetn,
  input  logic                      spi_ref_clk,
  input  logic                      spi_rst_n,

  // APB slave
  input  logic                      psel,
  input  logic                      penable,
  input  logic                      pwrite,
  input  logic [APB_ADDR_WIDTH-1:0] paddr,
  input  logic [31:0]               pwdata,
  output logic [31:0]               prdata,
  output logic                      pready,
  output logic                      pslverr,

  // SPI pins (bidirectional; direction follows SPI_CTRL.MODE)
  output logic                      sclk_o,
  input  logic                      sclk_i,
  output logic                      sclk_oe,
  output logic                      mosi_o,
  input  logic                      mosi_i,
  output logic                      mosi_oe,
  output logic                      miso_o,
  input  logic                      miso_i,
  output logic                      miso_oe,
  output logic [NUM_CS-1:0]         cs_n_o,
  input  logic                      cs_n_i,
  output logic                      cs_oe,

  // system
  output logic                      irq,
  output logic                      tx_dreq,
  output logic                      rx_dreq
);


  localparam int LW = $clog2(FIFO_DEPTH);

  // ---------------- per-domain reset synchronizers (Spec 7.7) ----------
  // Two independent instances. Never one instance fanning to both domains.
  logic prst_n_s, spi_rst_n_s;

  spi_reset_sync u_rst_bus (.clk(pclk),        .rst_n_in(presetn),   .rst_n_out(prst_n_s));
  spi_reset_sync u_rst_spi (.clk(spi_ref_clk), .rst_n_in(spi_rst_n), .rst_n_out(spi_rst_n_s));

  // ---------------- interconnect ----------------
  cfg_t  cfg_bus, cfg_active;
  logic  cfg_commit, cfg_valid_sync, cfg_capture, en_effective;
  logic  swrst_write, rst_busy, bus_clr, spi_clr, swrst_active;

  logic                  tx_wen, tx_full, tx_ren, tx_empty;
  logic [DATA_WIDTH-1:0] tx_wdata, tx_rdata;
  logic [LW:0]           tx_level_bus, tx_level_spi;

  logic                  rx_wen, rx_full, rx_ren, rx_empty;
  logic [DATA_WIDTH-1:0] rx_wdata, rx_rdata;
  logic [LW:0]           rx_level_spi, rx_level_bus;

  logic tx_flush, rx_flush;
  logic fsm_idle;

  // ---------------- register file (bus domain) ----------------
  logic busy_sync, tx_stall_sync;
  logic evt_done_bus, evt_txunf_bus, evt_rxovf_bus, evt_frerr_bus;

  spi_regfile #(
    .DATA_WIDTH(DATA_WIDTH), .FIFO_DEPTH(FIFO_DEPTH),
    .APB_ADDR_WIDTH(APB_ADDR_WIDTH), .DEF_CPOL(DEF_CPOL), .DEF_CPHA(DEF_CPHA)
  ) u_regfile (
    .pclk(pclk), .prst_n(prst_n_s),
    .psel(psel), .penable(penable), .pwrite(pwrite), .paddr(paddr),
    .pwdata(pwdata), .prdata(prdata), .pready(pready), .pslverr(pslverr),
    .cfg_bus(cfg_bus), .cfg_commit(cfg_commit), .swrst_write(swrst_write),
    .tx_wen(tx_wen), .tx_wdata(tx_wdata), .tx_full(tx_full), .tx_level(tx_level_bus),
    .rx_ren(rx_ren), .rx_rdata(rx_rdata), .rx_empty(rx_empty), .rx_level(rx_level_bus),
    .tx_flush(tx_flush), .rx_flush(rx_flush),
    .busy_sync(busy_sync), .tx_stall_sync(tx_stall_sync), .rst_busy(rst_busy),
    .evt_done(evt_done_bus), .evt_tx_underrun(evt_txunf_bus),
    .evt_rx_overrun(evt_rxovf_bus), .evt_frame_err(evt_frerr_bus),
    .irq(irq), .tx_dreq(tx_dreq), .rx_dreq(rx_dreq)
  );

  // ---------------- configuration commit: bus -> SPI ----------------
  spi_event_sync u_cfg_sync (
    .src_clk(pclk),        .src_rst_n(prst_n_s),    .src_pulse(cfg_commit),
    .dst_clk(spi_ref_clk), .dst_rst_n(spi_rst_n_s), .dst_pulse(cfg_valid_sync)
  );

  spi_cfg_latch #(.DEF_CPOL(DEF_CPOL), .DEF_CPHA(DEF_CPHA)) u_cfg_latch (
    .spi_clk(spi_ref_clk), .spi_rst_n(spi_rst_n_s),
    .cfg_bus(cfg_bus), .cfg_valid_sync(cfg_valid_sync), .fsm_idle(fsm_idle),
    .cfg_active(cfg_active), .cfg_capture(cfg_capture), .en_effective(en_effective)
  );

  // ---------------- software reset sequencer ----------------
  spi_swrst_ctrl u_swrst (
    .pclk(pclk), .prst_n(prst_n_s), .swrst_write(swrst_write),
    .rst_busy(rst_busy), .bus_clr(bus_clr),
    .spi_clk(spi_ref_clk), .spi_rst_n(spi_rst_n_s),
    .spi_clr(spi_clr), .swrst_active(swrst_active)
  );

  // FIFO flush requests must clear BOTH sides. The bus side clears
  // immediately; the far side is cleared through an event synchronizer.
  logic tx_flush_spi, rx_flush_spi;
  spi_event_sync u_txflush_sync (
    .src_clk(pclk),        .src_rst_n(prst_n_s),    .src_pulse(tx_flush),
    .dst_clk(spi_ref_clk), .dst_rst_n(spi_rst_n_s), .dst_pulse(tx_flush_spi)
  );
  spi_event_sync u_rxflush_sync (
    .src_clk(pclk),        .src_rst_n(prst_n_s),    .src_pulse(rx_flush),
    .dst_clk(spi_ref_clk), .dst_rst_n(spi_rst_n_s), .dst_pulse(rx_flush_spi)
  );

  // ---------------- asynchronous FIFOs ----------------
  spi_async_fifo #(.WIDTH(DATA_WIDTH), .DEPTH(FIFO_DEPTH)) u_tx_fifo (
    .wclk(pclk),        .wrst_n(prst_n_s),    .w_clr(bus_clr | tx_flush),
    .wen(tx_wen),       .wdata(tx_wdata),     .wfull(tx_full), .wlevel(tx_level_bus),
    .rclk(spi_ref_clk), .rrst_n(spi_rst_n_s), .r_clr(spi_clr | tx_flush_spi),
    .ren(tx_ren),       .rdata(tx_rdata),     .rempty(tx_empty), .rlevel(tx_level_spi)
  );

  spi_async_fifo #(.WIDTH(DATA_WIDTH), .DEPTH(FIFO_DEPTH)) u_rx_fifo (
    .wclk(spi_ref_clk), .wrst_n(spi_rst_n_s), .w_clr(spi_clr | rx_flush_spi),
    .wen(rx_wen),       .wdata(rx_wdata),     .wfull(rx_full), .wlevel(rx_level_spi),
    .rclk(pclk),        .rrst_n(prst_n_s),    .r_clr(bus_clr | rx_flush),
    .ren(rx_ren),       .rdata(rx_rdata),     .rempty(rx_empty), .rlevel(rx_level_bus)
  );

  // ---------------- master and slave engines ----------------
  logic                  m_tx_ren, m_rx_wen, m_busy, m_stall, m_idle;
  logic                  m_done, m_rxovf;
  logic [DATA_WIDTH-1:0] m_rx_data;
  logic                  m_sclk, m_mosi;
  logic [NUM_CS-1:0]     m_cs_n;

  spi_master_engine #(.DATA_WIDTH(DATA_WIDTH), .NUM_CS(NUM_CS)) u_master (
    .spi_clk(spi_ref_clk), .spi_rst_n(spi_rst_n_s),
    .cfg(cfg_active), .en_effective(en_effective), .swrst_active(swrst_active),
    .tx_empty(tx_empty | cfg_active.mode), .tx_data(tx_rdata), .tx_ren(m_tx_ren),
    .rx_full(rx_full), .rx_wen(m_rx_wen), .rx_data(m_rx_data),
    .sclk_o(m_sclk), .mosi_o(m_mosi), .miso_i(miso_i), .cs_n_o(m_cs_n),
    .busy(m_busy), .tx_stall(m_stall), .fsm_idle(m_idle),
    .evt_done(m_done), .evt_rx_overrun(m_rxovf)
  );

  logic                  s_tx_ren, s_rx_wen, s_busy;
  logic                  s_done, s_rxovf, s_txunf, s_frerr;
  logic [DATA_WIDTH-1:0] s_rx_data;
  logic                  s_miso;

  spi_slave_engine #(.DATA_WIDTH(DATA_WIDTH)) u_slave (
    .spi_clk(spi_ref_clk), .spi_rst_n(spi_rst_n_s),
    .cfg(cfg_active), .swrst_active(swrst_active),
    .sclk_i(sclk_i), .mosi_i(mosi_i), .miso_o(s_miso), .cs_n_i(cs_n_i),
    .tx_empty(tx_empty | ~cfg_active.mode), .tx_data(tx_rdata), .tx_ren(s_tx_ren),
    .rx_full(rx_full), .rx_wen(s_rx_wen), .rx_data(s_rx_data),
    .busy(s_busy), .evt_done(s_done), .evt_rx_overrun(s_rxovf),
    .evt_tx_underrun(s_txunf), .evt_frame_err(s_frerr)
  );

  // ---------------- engine mux ----------------
  logic slave_mode;
  assign slave_mode = cfg_active.mode;

  assign tx_ren    = slave_mode ? s_tx_ren  : m_tx_ren;
  assign rx_wen    = slave_mode ? s_rx_wen  : m_rx_wen;
  assign rx_wdata  = slave_mode ? s_rx_data : m_rx_data;

  // The transfer FSM is the master FSM; in slave mode it stays in IDLE so
  // the configuration latch is free to capture between frames.
  assign fsm_idle  = slave_mode ? 1'b1 : m_idle;

  logic busy_spi, stall_spi;
  assign busy_spi  = slave_mode ? s_busy : m_busy;
  assign stall_spi = slave_mode ? 1'b0   : m_stall;

  // ---------------- pin direction ----------------
  assign sclk_o  = m_sclk;
  assign mosi_o  = m_mosi;
  assign miso_o  = s_miso;
  assign cs_n_o  = m_cs_n;
  // LOOPBACK holds the external pins idle (Spec 7.5)
  assign sclk_oe = ~slave_mode & ~cfg_active.loopback;
  assign mosi_oe = ~slave_mode & ~cfg_active.loopback;
  assign cs_oe   = ~slave_mode & ~cfg_active.loopback;
  assign miso_oe =  slave_mode & ~cs_n_i;      // released to Hi-Z when not selected

  // ---------------- status: SPI -> bus (level synchronizers) ----------
  spi_lvl_sync u_busy_sync  (.clk(pclk), .rst_n(prst_n_s), .d(busy_spi),  .q(busy_sync));
  spi_lvl_sync u_stall_sync (.clk(pclk), .rst_n(prst_n_s), .d(stall_spi), .q(tx_stall_sync));

  // ---------------- events: SPI -> bus (event synchronizers) ----------
  logic evt_done_spi, evt_rxovf_spi;
  assign evt_done_spi  = slave_mode ? s_done  : m_done;
  assign evt_rxovf_spi = slave_mode ? s_rxovf : m_rxovf;

  spi_event_sync u_done_sync (
    .src_clk(spi_ref_clk), .src_rst_n(spi_rst_n_s), .src_pulse(evt_done_spi),
    .dst_clk(pclk),        .dst_rst_n(prst_n_s),    .dst_pulse(evt_done_bus)
  );
  spi_event_sync u_rxovf_sync (
    .src_clk(spi_ref_clk), .src_rst_n(spi_rst_n_s), .src_pulse(evt_rxovf_spi),
    .dst_clk(pclk),        .dst_rst_n(prst_n_s),    .dst_pulse(evt_rxovf_bus)
  );
  spi_event_sync u_txunf_sync (
    .src_clk(spi_ref_clk), .src_rst_n(spi_rst_n_s), .src_pulse(s_txunf & slave_mode),
    .dst_clk(pclk),        .dst_rst_n(prst_n_s),    .dst_pulse(evt_txunf_bus)
  );
  spi_event_sync u_frerr_sync (
    .src_clk(spi_ref_clk), .src_rst_n(spi_rst_n_s), .src_pulse(s_frerr & slave_mode),
    .dst_clk(pclk),        .dst_rst_n(prst_n_s),    .dst_pulse(evt_frerr_bus)
  );

endmodule
