//======================================================================
// tb_spi_controller.sv - self-checking smoke test
//
// Covers the behaviours the specification is most specific about:
//   reset values, APB PSLVERR rules, DFS range rejection, always-writable
//   SPI_CTRL, loopback data integrity, watermark/DMA gating, the interrupt
//   interlock, and the software-reset sequence.
//======================================================================
`timescale 1ns/1ps

module tb_spi_controller;

  localparam int DW    = 32;
  localparam int DEPTH = 8;
  localparam int NCS   = 4;

  logic pclk = 0, presetn = 0;
  logic spi_ref_clk = 0, spi_rst_n = 0;

  // deliberately unrelated clock periods - the domains are asynchronous
  always #5    pclk        = ~pclk;         // 100 MHz
  always #17   spi_ref_clk = ~spi_ref_clk;  //  ~29 MHz

  logic        psel=0, penable=0, pwrite=0;
  logic [7:0]  paddr=0;
  logic [31:0] pwdata=0, prdata;
  logic        pready, pslverr;

  logic sclk_o, sclk_i=0, sclk_oe, mosi_o, mosi_i=0, mosi_oe;
  logic miso_o, miso_i=0, miso_oe;
  logic [NCS-1:0] cs_n_o;
  logic cs_n_i=1, cs_oe;
  logic irq, tx_dreq, rx_dreq;

  int errors = 0;

  spi_controller_top #(
    .DATA_WIDTH(DW), .FIFO_DEPTH(DEPTH), .NUM_CS(NCS), .APB_ADDR_WIDTH(8)
  ) dut (
    .pclk(pclk), .presetn(presetn), .spi_ref_clk(spi_ref_clk), .spi_rst_n(spi_rst_n),
    .psel(psel), .penable(penable), .pwrite(pwrite), .paddr(paddr),
    .pwdata(pwdata), .prdata(prdata), .pready(pready), .pslverr(pslverr),
    .sclk_o(sclk_o), .sclk_i(sclk_i), .sclk_oe(sclk_oe),
    .mosi_o(mosi_o), .mosi_i(mosi_i), .mosi_oe(mosi_oe),
    .miso_o(miso_o), .miso_i(miso_i), .miso_oe(miso_oe),
    .cs_n_o(cs_n_o), .cs_n_i(cs_n_i), .cs_oe(cs_oe),
    .irq(irq), .tx_dreq(tx_dreq), .rx_dreq(rx_dreq)
  );

  // ---------------- APB tasks ----------------
  task automatic apb_write(input [7:0] a, input [31:0] d, output logic err);
    @(posedge pclk); #1; psel=1; pwrite=1; paddr=a; pwdata=d; penable=0;
    @(posedge pclk); #1; penable=1;
    @(posedge pclk); err = pslverr; #1;
    psel=0; penable=0; pwrite=0;
  endtask

  task automatic apb_read(input [7:0] a, output logic [31:0] d, output logic err);
    @(posedge pclk); #1; psel=1; pwrite=0; paddr=a; penable=0;
    @(posedge pclk); #1; penable=1;
    @(posedge pclk); d = prdata; err = pslverr; #1;
    psel=0; penable=0;
  endtask

  task automatic wr(input [7:0] a, input [31:0] d);
    logic e; apb_write(a,d,e);
  endtask

  task automatic check(input string name, input logic cond);
    if (cond) $display("  PASS  %s", name);
    else begin $display("  FAIL  %s", name); errors++; end
  endtask

  logic [31:0] rd; logic er;

  initial begin
    $display("\n=== SPI Controller IP - smoke test ===\n");
    repeat (5) @(posedge pclk); #1;
    presetn = 1; spi_rst_n = 1;
    repeat (10) @(posedge pclk);

    // -------- 1. reset values (Spec 6.10) --------
    $display("[1] Reset values");
    apb_read(8'h00, rd, er); check("SPI_CTRL   = 0x00000800", rd === 32'h0000_0800);
    apb_read(8'h10, rd, er); check("SPI_CLKDIV = 0x000000FF", rd === 32'h0000_00FF);
    apb_read(8'h1C, rd, er); check("SPI_CSCTRL = 0x00007FFC", rd === 32'h0000_7FFC);
    apb_read(8'h14, rd, er); check("SPI_IE     = 0x00000000", rd === 32'h0);
    apb_read(8'h20, rd, er); check("SPI_FIFOCTRL watermarks = DEPTH/2",
                                   rd[15:8]===8'd4 && rd[23:16]===8'd4);
    apb_read(8'h04, rd, er); check("SPI_STAT TX_EMPTY=1, RX_EMPTY=1, BUSY=0",
                                   rd[1]===1'b1 && rd[3]===1'b1 && rd[0]===1'b0);

    // -------- 2. APB error rules (Spec 7.6) --------
    $display("\n[2] PSLVERR rules");
    apb_read (8'h0C, rd, er); check("read empty RX FIFO -> PSLVERR", er===1'b1);
    check("  and PRDATA = 0", rd === 32'h0);
    apb_write(8'h04, 32'h1, er); check("write to RO SPI_STAT -> PSLVERR", er===1'b1);
    apb_read (8'h08, rd, er);   check("read of WO SPI_TXDATA -> PSLVERR", er===1'b1);
    apb_write(8'h40, 32'h1, er); check("unmapped offset -> PSLVERR", er===1'b1);
    apb_write(8'h00, 32'h0000_0300, er);  // DFS = 3, below the legal minimum
    check("DFS = 3 rejected -> PSLVERR", er===1'b1);
    apb_read(8'h00, rd, er); check("  DFS field unchanged (still 8)", rd[13:8]===6'd8);
    check("  pready tied high throughout", pready===1'b1);

    // -------- 3. SPI_CTRL always writable (locked decision) --------
    $display("\n[3] SPI_CTRL is always writable");
    apb_write(8'h00, 32'h0000_0801, er);  // EN=1, DFS=8
    check("SPI_CTRL write accepted, no PSLVERR", er===1'b0);

    // -------- 4. loopback transfer (Spec 7.5) --------
    $display("\n[4] Loopback data integrity");
    wr(8'h00, 32'h0000_0800);             // EN=0 while configuring
    wr(8'h10, 32'h0000_0001);             // CLKDIV = 1 -> fast SCLK for the test
    wr(8'h1C, 32'h0000_0004);             // CS_AUTO=1, setup/hold = 0
    wr(8'h00, 32'h0000_0821);             // EN=1, LOOPBACK=1, DFS=8
    repeat (20) @(posedge pclk);

    wr(8'h08, 32'h000000A5);
    wr(8'h08, 32'h0000005A);
    wr(8'h08, 32'h000000F0);

    // wait for the frames to complete
    repeat (400) @(posedge pclk);

    apb_read(8'h0C, rd, er);
    check("loopback word 1 = 0xA5", (rd[7:0]===8'hA5) && (er===1'b0));
    apb_read(8'h0C, rd, er);
    check("loopback word 2 = 0x5A", (rd[7:0]===8'h5A) && (er===1'b0));
    apb_read(8'h0C, rd, er);
    check("loopback word 3 = 0xF0", (rd[7:0]===8'hF0) && (er===1'b0));

    apb_read(8'h04, rd, er);
    check("no underrun / overrun / frame error",
          rd[7]===1'b0 && rd[8]===1'b0 && rd[9]===1'b0);
    apb_read(8'h18, rd, er);
    check("DONE_INT latched in SPI_IS", rd[4]===1'b1);

    // -------- 5. DMA gating and the interrupt interlock (Spec 6.6) --------
    $display("\n[5] DMA gating and interlock");
    wr(8'h00, 32'h0000_0800);             // EN=0
    repeat (20) @(posedge pclk);
    check("tx_dreq low while TXDMAEN=0", tx_dreq===1'b0);
    wr(8'h00, 32'h0000_0840);             // TXDMAEN=1
    repeat (20) @(posedge pclk);
    check("tx_dreq asserted with TXDMAEN=1 and TX FIFO empty", tx_dreq===1'b1);
    check("rx_dreq still low (RXDMAEN=0)", rx_dreq===1'b0);

    wr(8'h18, 32'h3F);                    // clear SPI_IS
    wr(8'h14, 32'h01);                    // enable TXWM_INT only
    repeat (20) @(posedge pclk);
    check("TX watermark irq suppressed by TXDMAEN interlock", irq===1'b0);
    apb_read(8'h18, rd, er);
    check("  but SPI_IS still latches TXWM", rd[0]===1'b1);

    wr(8'h00, 32'h0000_0800);             // TXDMAEN=0
    repeat (20) @(posedge pclk);
    check("irq asserts once DMA is disabled", irq===1'b1);
    wr(8'h14, 32'h00);

    // -------- 6. software reset (Spec 7.5) --------
    $display("\n[6] Software reset");
    wr(8'h08, 32'h11);  wr(8'h08, 32'h22);   // queue data
    repeat (5) @(posedge pclk);
    apb_write(8'h00, 32'h0000_4800, er);     // SWRST = 1
    check("SWRST write accepted unconditionally", er===1'b0);

    apb_read(8'h00, rd, er);
    check("SWRST always reads back 0", rd[14]===1'b0);

    begin
      int guard = 0;
      do begin
        apb_read(8'h04, rd, er);
        guard++;
      end while (rd[11] === 1'b1 && guard < 200);
      check("RST_BUSY self-clears", rd[11]===1'b0);
    end

    apb_read(8'h04, rd, er);
    check("TX FIFO emptied by SWRST", rd[1]===1'b1);
    apb_read(8'h10, rd, er);
    check("SPI_CLKDIV preserved across SWRST", rd===32'h0000_0001);
    apb_read(8'h1C, rd, er);
    check("SPI_CSCTRL preserved across SWRST", rd===32'h0000_0004);
    apb_read(8'h18, rd, er);
    // The watermark sources are level conditions: with the FIFO emptied by
    // the reset, TXWM legitimately re-latches at once. Only the latched
    // error and completion sources must be cleared.
    check("SPI_IS error/done sources cleared by SWRST", rd[5:2]===4'h0);

    // -------- summary --------
    $display("\n=====================================");
    if (errors == 0) $display("  ALL CHECKS PASSED");
    else             $display("  %0d CHECK(S) FAILED", errors);
    $display("=====================================\n");
    $finish;
  end

  initial begin
    #4_000_000;
    $display("TIMEOUT");
    $finish;
  end

`ifdef DUMP
  initial begin
    $dumpfile("tb_spi_controller.vcd");
    $dumpvars(0, tb_spi_controller);
  end
`endif

endmodule
