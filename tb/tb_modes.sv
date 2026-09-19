// Sweep all four SPI modes, both bit orders and several frame sizes
// through the loopback path. Verifies the CPOL/CPHA edge logic and the
// DFS-driven frame termination for every combination.
`timescale 1ns/1ps
module tb_modes;
  logic pclk=0, presetn=0, spi_ref_clk=0, spi_rst_n=0;
  always #5  pclk        = ~pclk;
  always #17 spi_ref_clk = ~spi_ref_clk;

  logic psel=0,penable=0,pwrite=0; logic [7:0] paddr=0;
  logic [31:0] pwdata=0, prdata; logic pready, pslverr;
  logic sclk_o,sclk_i=0,sclk_oe,mosi_o,mosi_i=0,mosi_oe,miso_o,miso_i=0,miso_oe;
  logic [3:0] cs_n_o; logic cs_n_i=1, cs_oe, irq, tx_dreq, rx_dreq;
  int errors=0, tests=0;

  spi_controller_top #(.DATA_WIDTH(32),.FIFO_DEPTH(8),.NUM_CS(4)) dut (
    .pclk(pclk),.presetn(presetn),.spi_ref_clk(spi_ref_clk),.spi_rst_n(spi_rst_n),
    .psel(psel),.penable(penable),.pwrite(pwrite),.paddr(paddr),.pwdata(pwdata),
    .prdata(prdata),.pready(pready),.pslverr(pslverr),
    .sclk_o(sclk_o),.sclk_i(sclk_i),.sclk_oe(sclk_oe),.mosi_o(mosi_o),.mosi_i(mosi_i),
    .mosi_oe(mosi_oe),.miso_o(miso_o),.miso_i(miso_i),.miso_oe(miso_oe),
    .cs_n_o(cs_n_o),.cs_n_i(cs_n_i),.cs_oe(cs_oe),
    .irq(irq),.tx_dreq(tx_dreq),.rx_dreq(rx_dreq));

  task automatic wr(input [7:0] a, input [31:0] d);
    @(posedge pclk); #1; psel=1;pwrite=1;paddr=a;pwdata=d;penable=0;
    @(posedge pclk); #1; penable=1;
    @(posedge pclk); #1; psel=0;penable=0;pwrite=0;
  endtask
  task automatic rdr(input [7:0] a, output logic [31:0] d);
    @(posedge pclk); #1; psel=1;pwrite=0;paddr=a;penable=0;
    @(posedge pclk); #1; penable=1;
    @(posedge pclk); d=prdata; #1; psel=0;penable=0;
  endtask

  logic [31:0] v;
  initial begin
    repeat(5) @(posedge pclk); #1; presetn=1; spi_rst_n=1; repeat(10) @(posedge pclk);
    $display("\n=== mode / bit-order / DFS sweep (loopback) ===\n");
    for (int cpol=0; cpol<2; cpol++)
    for (int cpha=0; cpha<2; cpha++)
    for (int lsb=0;  lsb<2;  lsb++)
    for (int di=0;   di<3;   di++) begin
      int dfs; logic [31:0] pat, ctrl;
      dfs = (di==0)?4:((di==1)?8:16);
      pat = (dfs==4)?32'h9 : (dfs==8)?32'hC3 : 32'hBEEF;
      wr(8'h00, 32'h0000_0800);                       // EN=0
      wr(8'h10, 32'h0000_0001);
      wr(8'h1C, 32'h0000_0004);
      ctrl = 32'h21 | (dfs<<8) | (cpol<<2) | (cpha<<3) | (lsb<<4);
      wr(8'h00, ctrl);                                // EN=1, LOOPBACK=1
      repeat(20) @(posedge pclk);
      wr(8'h08, pat);
      repeat(300) @(posedge pclk);
      rdr(8'h0C, v);
      tests++;
      if (v !== pat) begin
        errors++;
        $display("  FAIL  CPOL=%0d CPHA=%0d %s DFS=%2d  sent %h got %h",
                 cpol,cpha,lsb?"LSB":"MSB",dfs,pat,v);
      end else
        $display("  PASS  CPOL=%0d CPHA=%0d %s DFS=%2d  0x%h",
                 cpol,cpha,lsb?"LSB":"MSB",dfs,pat);
      wr(8'h00, 32'h0000_4800);                       // SWRST between cases
      repeat(60) @(posedge pclk);
    end
    $display("\n  %0d/%0d combinations passed", tests-errors, tests);
    if (errors) $display("  %0d FAILURES\n", errors); else $display("  ALL MODES PASS\n");
    $finish;
  end
  initial begin #20_000_000; $display("TIMEOUT"); $finish; end
`ifdef DUMP
  initial begin
    $dumpfile("tb_modes.vcd");
    $dumpvars(0, tb_modes);
  end
`endif

endmodule
