//======================================================================
// spi_master_engine.sv - Transfer FSM, shift engine, chip-select control
//
// States (Spec 7.1 / FSM diagram):
//   IDLE -> LOAD -> CS_SETUP -> TRANSFER -> CS_HOLD -> DONE -> IDLE
//   TRANSFER -> STALL  (TX FIFO empty at a word boundary, CS held)
//   any state -> IDLE  (software reset)
//
// Frame termination counts SCLK EDGES, not sample strobes: a frame ends
// after exactly 2*DFS edges, which guarantees SCLK is back at its CPOL
// idle level when the frame completes (Spec 8.3).
//======================================================================
module spi_master_engine
  import spi_pkg::*; #(
  parameter int DATA_WIDTH = 32,
  parameter int NUM_CS     = 4
)(
  input  logic                    spi_clk,
  input  logic                    spi_rst_n,

  input  cfg_t                    cfg,            // latched configuration
  input  logic                    en_effective,   // post-capture EN (Spec 7.4)
  input  logic                    swrst_active,   // force to IDLE

  // TX FIFO read port (SPI side)
  input  logic                    tx_empty,
  input  logic [DATA_WIDTH-1:0]   tx_data,
  output logic                    tx_ren,

  // RX FIFO write port (SPI side)
  input  logic                    rx_full,
  output logic                    rx_wen,
  output logic [DATA_WIDTH-1:0]   rx_data,

  // serial pins (master direction)
  output logic                    sclk_o,
  output logic                    mosi_o,
  input  logic                    miso_i,
  output logic [NUM_CS-1:0]       cs_n_o,

  // status / events
  output logic                    busy,
  output logic                    tx_stall,
  output logic                    fsm_idle,
  output logic                    evt_done,
  output logic                    evt_rx_overrun
);



  // Configuration fields unpacked into plain nets. Packed-struct member
  // selects inside always blocks are poorly supported by some tools, and
  // flat nets also make the synthesised logic easier to read.
  logic       c_cpol, c_cpha, c_lsbfirst, c_loopback, c_cs_auto;
  logic [5:0] c_dfs, c_cs_setup, c_cs_hold;
  logic [7:0] c_clkdiv;
  logic [1:0] c_cs_sel;

  assign c_cpol     = cfg.cpol;
  assign c_cpha     = cfg.cpha;
  assign c_lsbfirst = cfg.lsbfirst;
  assign c_loopback = cfg.loopback;
  assign c_cs_auto  = cfg.cs_auto;
  assign c_dfs      = cfg.dfs;
  assign c_cs_setup = cfg.cs_setup;
  assign c_cs_hold  = cfg.cs_hold;
  assign c_clkdiv   = cfg.clkdiv;
  assign c_cs_sel   = cfg.cs_sel;

  typedef enum logic [2:0] {
    S_IDLE, S_LOAD, S_CS_SETUP, S_TRANSFER, S_STALL, S_CS_HOLD, S_DONE
  } state_e;

  state_e state, nstate;

  logic [DATA_WIDTH-1:0] tx_sr, rx_sr;
  logic [9:0]            edge_cnt;         // counts SCLK edges, up to 2*DFS
  logic [6:0]            cs_cnt;
  logic                  cs_active;        // chip-select asserted (active low on pin)
  logic                  run_clk;
  logic                  frame_last_edge;
  logic                  tx_started;

  // ---------------- serial clock ----------------
  logic lead_edge, trail_edge, drive_en, sample_en, edge_tick;

  assign run_clk = (state == S_TRANSFER);

  spi_clkgen u_clkgen (
    .spi_clk    (spi_clk),
    .spi_rst_n  (spi_rst_n),
    .run        (run_clk),
    .cpol       (c_cpol),
    .cpha       (c_cpha),
    .clkdiv     (c_clkdiv),
    .sclk_o     (sclk_o),
    .lead_edge  (lead_edge),
    .trail_edge (trail_edge),
    .drive_en   (drive_en),
    .sample_en  (sample_en),
    .edge_tick  (edge_tick)
  );

  // a frame is complete after exactly 2*DFS edges
  assign frame_last_edge = edge_tick && (edge_cnt == ({4'd0, c_dfs} << 1) - 10'd1);

  // ---------------- MOSI / loopback ----------------
  logic mosi_int, miso_int;
  assign mosi_int = c_lsbfirst ? tx_sr[0] : tx_sr[DATA_WIDTH-1];
  assign mosi_o   = mosi_int;
  // LOOPBACK = 1 ties the transmit datapath to the receive datapath and
  // leaves the external pins idle (Spec 7.5).
  assign miso_int = c_loopback ? mosi_int : miso_i;

  // Next value of the RX shift register. Needed because the final SCLK edge
  // of a frame is a SAMPLE edge for CPHA=1 but a DRIVE edge for CPHA=0, so
  // the assembled word must be taken from rx_sr_nxt rather than assuming a
  // sample lands on the last edge.
  logic [DATA_WIDTH-1:0] rx_sr_nxt;
  always_comb begin
    if (sample_en)
      rx_sr_nxt = c_lsbfirst ? {miso_int, rx_sr[DATA_WIDTH-1:1]}
                             : {rx_sr[DATA_WIDTH-2:0], miso_int};
    else
      rx_sr_nxt = rx_sr;
  end

  // ---------------- next-state logic ----------------
  logic more_data;
  assign more_data = !tx_empty;

  always_comb begin
    nstate = state;
    unique case (state)
      S_IDLE:     if (en_effective && !tx_empty)          nstate = S_LOAD;
      S_LOAD:                                             nstate = S_CS_SETUP;
      S_CS_SETUP: if (!c_cs_auto || cs_cnt >= {1'b0, c_cs_setup})
                                                          nstate = S_TRANSFER;
      S_TRANSFER: if (frame_last_edge) begin
                    if (more_data)         nstate = S_LOAD;
                    else if (c_cs_auto)    nstate = S_CS_HOLD;
                    else                   nstate = S_STALL;
                  end
      S_STALL:    if (more_data)                          nstate = S_LOAD;
                  else if (!en_effective)                 nstate = S_CS_HOLD;
      S_CS_HOLD:  if (!c_cs_auto || cs_cnt >= {1'b0, c_cs_hold})
                                                          nstate = S_DONE;
      S_DONE:                                             nstate = S_IDLE;
      default:                                            nstate = S_IDLE;
    endcase
    if (swrst_active) nstate = S_IDLE;
  end

  // ---------------- state / datapath registers ----------------
  logic [DATA_WIDTH-1:0] rx_word;
  logic                  rx_push;

  always_ff @(posedge spi_clk or negedge spi_rst_n) begin
    if (!spi_rst_n) begin
      state     <= S_IDLE;
      tx_sr     <= '0;
      rx_sr     <= '0;
      edge_cnt  <= '0;
      cs_cnt    <= '0;
      cs_active <= 1'b0;
      rx_push   <= 1'b0;
      rx_word   <= '0;
      tx_started<= 1'b0;
    end else if (swrst_active) begin
      // software reset: flush the shift register and return to IDLE
      state     <= S_IDLE;
      tx_sr     <= '0;
      rx_sr     <= '0;
      edge_cnt  <= '0;
      cs_cnt    <= '0;
      cs_active <= 1'b0;
      rx_push   <= 1'b0;
    end else begin
      state   <= nstate;
      rx_push <= 1'b0;

      unique case (state)
        S_IDLE: begin
          edge_cnt <= '0;
          cs_cnt   <= '0;
          if (c_cs_auto) cs_active <= 1'b0;
        end

        S_LOAD: begin
          // left-align for MSB-first so the transmitted bit is always the
          // top bit of the shift register
          tx_sr    <= c_lsbfirst ? tx_data
                                   : (tx_data << (DATA_WIDTH - 32'(c_dfs)));
          rx_sr      <= '0;
          edge_cnt   <= '0;
          cs_cnt     <= '0;
          tx_started <= 1'b0;
        end

        S_CS_SETUP: begin
          cs_active <= 1'b1;
          cs_cnt    <= cs_cnt + 7'd1;
        end

        S_TRANSFER: begin
          rx_sr <= rx_sr_nxt;
          // CPHA=1 presents bit 0 on the first drive edge without advancing;
          // CPHA=0 already presented it at load time.
          if (drive_en) begin
            if (c_cpha && !tx_started) tx_started <= 1'b1;
            else tx_sr <= c_lsbfirst ? {1'b0, tx_sr[DATA_WIDTH-1:1]}
                                     : {tx_sr[DATA_WIDTH-2:0], 1'b0};
          end
          if (edge_tick) edge_cnt <= edge_cnt + 10'd1;

          if (frame_last_edge) begin
            // assemble the received word, right-aligned
            rx_word <= c_lsbfirst ? (rx_sr_nxt >> (DATA_WIDTH - 32'(c_dfs)))
                                  : (rx_sr_nxt & ((1 << c_dfs) - 1));
            rx_push  <= 1'b1;
            edge_cnt <= '0;
            cs_cnt   <= '0;
          end
        end

        S_STALL: begin
          cs_cnt <= '0;
        end

        S_CS_HOLD: begin
          cs_cnt <= cs_cnt + 7'd1;
        end

        S_DONE: begin
          if (c_cs_auto) cs_active <= 1'b0;
          cs_cnt <= '0;
        end

        default: ;
      endcase
    end
  end

  // ---------------- FIFO handshakes ----------------
  assign tx_ren = (state == S_LOAD);

  // RX overrun: a word was assembled while the RX FIFO was full (Spec 7.5)
  assign rx_wen         = rx_push && !rx_full;
  assign rx_data        = rx_word;
  assign evt_rx_overrun = rx_push &&  rx_full;

  // ---------------- chip-select decode ----------------
  always_comb begin
    for (int i = 0; i < NUM_CS; i++)
      cs_n_o[i] = !(cs_active && (32'(c_cs_sel) == i));
  end

  // ---------------- status ----------------
  assign fsm_idle = (state == S_IDLE);
  assign busy     = (state != S_IDLE);
  // TX_STALL is live status, not an error, and not sticky (Spec 6.3)
  assign tx_stall = (state == S_STALL);
  assign evt_done = (state == S_DONE);

endmodule
