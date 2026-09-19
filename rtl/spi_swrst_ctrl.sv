//======================================================================
// spi_swrst_ctrl.sv - Software reset sequencer (Spec 7.5)
//
// SWRST is a FUNCTIONAL reset sequenced by the FSM, not a reset net
// distributed from the bus domain. Driving a reset net across the
// boundary would create a reset-domain crossing; presetn and spi_rst_n
// remain the only true asynchronous resets in the design.
//
// Sequence:
//   1. RST_BUSY <- 1; APB access to SPI_TXDATA/SPI_RXDATA -> PSLVERR
//   2. bus-domain state cleared (FIFO pointer, its sync pipeline, SPI_IS)
//   3. request crosses to the SPI domain through an event synchronizer
//   4. SCLK stopped and parked at the CPOL idle level
//   5. chip-select deasserted AFTER the clock stop (clean end-of-frame)
//   6. shift register, SPI-side pointers, sync pipelines and dreq cleared
//   7. FSM -> IDLE, BUSY cleared
//   8. completion crosses back; RST_BUSY <- 0
//
// The request is accepted unconditionally in any state: a software reset
// must remain available when the core is hung, which is precisely when a
// precondition such as EN=0 or BUSY=0 could not be met.
//======================================================================
module spi_swrst_ctrl (
  // bus domain
  input  logic pclk,
  input  logic prst_n,
  input  logic swrst_write,      // 1-cycle pulse: write 1 to SPI_CTRL.SWRST
  output logic rst_busy,         // SPI_STAT.RST_BUSY
  output logic bus_clr,          // clear bus-side pointer / pipeline / SPI_IS

  // SPI domain
  input  logic spi_clk,
  input  logic spi_rst_n,
  output logic spi_clr,          // clear SPI-side pointers / pipelines / dreq
  output logic swrst_active      // hold the FSM in reset for the sequence
);

  // ---------------- bus domain: request and RST_BUSY ----------------
  logic req_pulse;
  assign req_pulse = swrst_write;      // accepted unconditionally

  logic done_pulse_bus;

  always_ff @(posedge pclk or negedge prst_n) begin
    if (!prst_n) begin
      rst_busy <= 1'b0;
    end else begin
      if (req_pulse)            rst_busy <= 1'b1;
      else if (done_pulse_bus)  rst_busy <= 1'b0;
    end
  end

  // step 2: clear bus-side state at the moment the request is issued
  assign bus_clr = req_pulse;

  // ---------------- request: bus -> SPI ----------------
  logic req_pulse_spi;
  spi_event_sync u_req_sync (
    .src_clk   (pclk),    .src_rst_n (prst_n),    .src_pulse (req_pulse),
    .dst_clk   (spi_clk), .dst_rst_n (spi_rst_n), .dst_pulse (req_pulse_spi)
  );

  // ---------------- SPI domain: run the sequence ----------------
  // The sequence is short and fully local: assert swrst_active for a few
  // spi_ref_clk cycles so the engines park SCLK, drop chip-select, flush
  // the datapath and return the FSM to IDLE.
  typedef enum logic [1:0] {R_IDLE, R_STOP, R_FLUSH, R_DONE} rstate_e;
  rstate_e rstate;
  logic [2:0] rcnt;
  logic       done_pulse_spi;

  always_ff @(posedge spi_clk or negedge spi_rst_n) begin
    if (!spi_rst_n) begin
      rstate         <= R_IDLE;
      rcnt           <= 3'd0;
      done_pulse_spi <= 1'b0;
    end else begin
      done_pulse_spi <= 1'b0;
      unique case (rstate)
        R_IDLE:  if (req_pulse_spi) begin
                   rstate <= R_STOP;
                   rcnt   <= 3'd0;
                 end
        // step 4/5: stop the clock, then deassert chip-select
        R_STOP:  begin
                   rcnt <= rcnt + 3'd1;
                   if (rcnt == 3'd3) begin
                     rstate <= R_FLUSH;
                     rcnt   <= 3'd0;
                   end
                 end
        // step 6/7: flush pointers and pipelines, FSM to IDLE
        R_FLUSH: begin
                   rcnt <= rcnt + 3'd1;
                   if (rcnt == 3'd2) rstate <= R_DONE;
                 end
        R_DONE:  begin
                   done_pulse_spi <= 1'b1;     // step 8
                   rstate         <= R_IDLE;
                 end
        default: rstate <= R_IDLE;
      endcase
    end
  end

  assign swrst_active = (rstate != R_IDLE);
  assign spi_clr      = (rstate == R_FLUSH);

  // ---------------- completion: SPI -> bus ----------------
  spi_event_sync u_done_sync (
    .src_clk   (spi_clk), .src_rst_n (spi_rst_n), .src_pulse (done_pulse_spi),
    .dst_clk   (pclk),    .dst_rst_n (prst_n),    .dst_pulse (done_pulse_bus)
  );

endmodule
