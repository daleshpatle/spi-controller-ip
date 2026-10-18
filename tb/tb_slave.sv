// Slave-mode regression. The testbench is the external SPI master: it drives
// sclk_i / mosi_i / cs_n_i, records what the IP drives on miso_o, then reads
// the received words and the status back over APB.
//
// Every case starts from a full reset, so every case also checks the first
// frame after reset. Cases: 4 SPI modes x MSB/LSB x DFS {4, 8, 13, 32} x 7
// frame patterns at SCLK 5 MHz, plus multi-word frames at 12.5 MHz for
// DFS 4 and 8 - 240 cases. Each case checks every MISO word (TX data, or
// the idle all-ones word once the staged words run out), the RX level and
// every RX word, RX empty afterwards, DONE=1, RXOVF=0, FRERR=0, and
// TXUNF=1 only when the master clocks more words than were staged.
`timescale 1ns/1ps
module tb_slave;
  logic pclk, presetn=1, spi_ref_clk, spi_rst_n=1;   // each case asserts them
  initial begin pclk       = 1'b0; forever #5  pclk        = ~pclk;        end
  initial begin spi_ref_clk = 1'b0; forever #17 spi_ref_clk = ~spi_ref_clk; end

  logic psel=0,penable=0,pwrite=0; logic [7:0] paddr=0;
  logic [31:0] pwdata=0, prdata; logic pready, pslverr;
  logic sclk_o,sclk_i=0,sclk_oe,mosi_o,mosi_i=0,mosi_oe,miso_o,miso_i=0,miso_oe;
  logic [3:0] cs_n_o; logic cs_n_i=1, cs_oe, irq, tx_dreq, rx_dreq;

  spi_controller_top #(.DATA_WIDTH(32),.FIFO_DEPTH(8),.NUM_CS(4)) dut (
    .pclk(pclk),.presetn(presetn),.spi_ref_clk(spi_ref_clk),.spi_rst_n(spi_rst_n),
    .psel(psel),.penable(penable),.pwrite(pwrite),.paddr(paddr),.pwdata(pwdata),
    .prdata(prdata),.pready(pready),.pslverr(pslverr),
    .sclk_o(sclk_o),.sclk_i(sclk_i),.sclk_oe(sclk_oe),.mosi_o(mosi_o),.mosi_i(mosi_i),
    .mosi_oe(mosi_oe),.miso_o(miso_o),.miso_i(miso_i),.miso_oe(miso_oe),
    .cs_n_o(cs_n_o),.cs_n_i(cs_n_i),.cs_oe(cs_oe),
    .irq(irq),.tx_dreq(tx_dreq),.rx_dreq(rx_dreq));

  task automatic wr(input [7:0] a, input [31:0] d, output logic err);
    @(posedge pclk); #1; psel=1;pwrite=1;paddr=a;pwdata=d;penable=0;
    @(posedge pclk); #1; penable=1;
    @(posedge pclk); err=pslverr; #1; psel=0;penable=0;pwrite=0;
  endtask
  task automatic rd(input [7:0] a, output logic [31:0] d, output logic err);
    @(posedge pclk); #1; psel=1;pwrite=0;paddr=a;penable=0;
    @(posedge pclk); #1; penable=1;
    @(posedge pclk); d=prdata; err=pslverr; #1; psel=0;penable=0;
  endtask

  // ---- current case ----
  int cpol, cpha, lsb, dfs;
  bit fast;                             // 0: SCLK 5 MHz, 1: SCLK 12.5 MHz
  logic [31:0] mask;                    // DFS ones = the idle (underrun) word
  string label;                         // printed with every FAIL line

  function automatic string mode_str();
    return $sformatf("CPOL=%0d CPHA=%0d %s", cpol, cpha, (lsb != 0) ? "LSB" : "MSB");
  endfunction

  // SCLK timing. Constant delays only: a variable #delay gives ZERODLY
  // warnings on newer Verilator.
  task automatic half_sclk();    if (fast) #40; else #100; endtask
  task automatic half_sclk_m1(); if (fast) #39; else #99;  endtask

  // One DFS-bit word, as an SPI master does it in this mode.
  // Returns what the master sampled on MISO.
  task automatic xfer(input logic [31:0] w, output logic [31:0] g);
    int i;
    g = '0;
    for (int k = 0; k < dfs; k++) begin
      i = (lsb != 0) ? k : dfs - 1 - k;
      if (cpha == 0) begin
        mosi_i = w[i];                                     // valid before the edge
        half_sclk();    sclk_i = ~cpol[0]; g[i] = miso_o;  // leading edge: both sample
        half_sclk();    sclk_i =  cpol[0];                 // trailing edge: both drive
      end else begin
        sclk_i = ~cpol[0]; #1; mosi_i = w[i];              // leading edge: both drive
        half_sclk_m1(); sclk_i =  cpol[0]; g[i] = miso_o;  // trailing edge: both sample
        half_sclk();
      end
    end
  endtask

  // Frame patterns: CS_n frames x words per frame, words staged in the TX FIFO.
  task automatic pattern(input int p, output int frames, output int words,
                         output int ntx, output bit prime);
    prime = 0;
    case (p)
      0: begin frames = 1; words = 2; ntx = 2; end             // 2-word frame
      1: begin frames = 2; words = 1; ntx = 2; end             // two 1-word frames
      2: begin frames = 2; words = 3; ntx = 6; end             // two 3-word frames
      3: begin frames = 1; words = 3; ntx = 1; end             // 1 staged: TX underrun
      4: begin frames = 1; words = 2; ntx = 2; prime = 1; end  // after a CS_n pulse
      5: begin frames = 2; words = 4; ntx = 8; end             // 8 words: FIFOs full
      default: begin frames = 1; words = 2; ntx = 0; end       // nothing staged
    endcase
  endtask

  task automatic chk(inout int bad, input string name,
                     input logic [31:0] got, input logic [31:0] exp);
    if (got !== exp) begin
      bad++;
      $display("    FAIL  %s: %s got 0x%0h, expected 0x%0h", label, name, got, exp);
    end
  endtask

  logic [31:0] txw [8], rxw [8], miso_seen [8];   // TX data, RX data, MISO as sampled
  int ncase = 0, nbad = 0;

  // One case: reset, configure, stage TX words, clock the frames, check.
  task automatic run_case(input int p, output int bad);
    int frames, words, ntx, total, n;
    bit prime;
    logic [31:0] v, g;
    logic err;
    pattern(p, frames, words, ntx, prime);
    total = frames * words;
    bad   = 0;
    mask  = (dfs == 32) ? 32'hFFFF_FFFF : ((32'h1 << dfs) - 1);
    for (int k = 0; k < 8; k++) begin                      // new data every case
      txw[k] = (32'hC2A9_35F1 ^ (32'(ncase*8 + k) * 32'h9E37_79B9)) & mask;
      rxw[k] = (32'h5B19_A7D3 ^ (32'(ncase*8 + k) * 32'h7F4A_7C15)) & mask;
    end

    // full reset with a real 1->0 edge (also right after power-up, so it works
    // in 2-state simulation with random initial values): every case is also a
    // first-frame-after-reset case
    cs_n_i = 1; mosi_i = 0;
    repeat (3) @(posedge spi_ref_clk); presetn = 0; spi_rst_n = 0;
    sclk_i = cpol[0];                                      // SCLK idle level, set in reset
    repeat (5) @(posedge spi_ref_clk); #1; presetn = 1; spi_rst_n = 1;
    repeat (10) @(posedge pclk);

    wr(8'h00, 32'h3 | (dfs<<8) | (cpol<<2) | (cpha<<3) | (lsb<<4), err);  // EN, slave
    chk(bad, "SPI_CTRL write", 32'(err), 0);
    for (int k = 0; k < ntx; k++) begin
      wr(8'h08, txw[k], err);
      chk(bad, $sformatf("TX write %0d", k+1), 32'(err), 0);
    end
    repeat (40) @(posedge pclk);
    #0.5;    // SPI pins move off the clock grid: a real master is asynchronous, and an
             // exact tie with a clock edge would depend on simulator event order
    if (prime) begin
      cs_n_i = 0; repeat (4) half_sclk(); cs_n_i = 1; repeat (4) half_sclk();
    end

    n = 0;
    for (int f = 0; f < frames; f++) begin
      cs_n_i = 0; repeat (2) half_sclk();
      for (int w = 0; w < words; w++) begin xfer(rxw[n], g); miso_seen[n] = g; n++; end
      repeat (2) half_sclk(); cs_n_i = 1; repeat (8) half_sclk();
    end
    repeat (80) @(posedge pclk);

    for (int k = 0; k < total; k++)
      chk(bad, $sformatf("MISO word %0d", k+1), miso_seen[k], (k < ntx) ? txw[k] : mask);
    rd(8'h24, v, err); chk(bad, "RX level", 32'(v[15:8]), 32'(total));
    rd(8'h18, v, err);
    chk(bad, "DONE",  32'(v[4]), 1);
    chk(bad, "RXOVF", 32'(v[3]), 0);
    chk(bad, "FRERR", 32'(v[5]), 0);
    chk(bad, "TXUNF", 32'(v[2]), (total > ntx) ? 1 : 0);
    for (int k = 0; k < total; k++) begin
      rd(8'h0C, v, err);
      chk(bad, $sformatf("RX word %0d", k+1), err ? 32'hDEAD_BEEF : v, rxw[k]);
    end
    rd(8'h0C, v, err); chk(bad, "RX empty (PSLVERR)", 32'(err), 1);
  endtask

  int b, ngrp, nbad_grp;
  initial begin
    $display("\n=== slave mode: TB is the external SPI master ===\n");
    for (cpol = 0; cpol < 2; cpol++)
    for (cpha = 0; cpha < 2; cpha++)
    for (lsb  = 0; lsb  < 2; lsb++)
    for (int di = 0; di < 4; di++) begin
      dfs = (di == 0) ? 4 : (di == 1) ? 8 : (di == 2) ? 13 : 32;
      ngrp = 0; nbad_grp = 0;
      fast = 0;                                            // SCLK 5 MHz: all patterns
      for (int p = 0; p < 7; p++) begin
        label = $sformatf("%s DFS=%0d pattern %0d", mode_str(), dfs, p);
        run_case(p, b);
        ncase++; ngrp++; if (b != 0) begin nbad++; nbad_grp++; end
      end
      if (dfs <= 8) begin                                  // SCLK 12.5 MHz
        fast = 1;
        label = $sformatf("%s DFS=%0d pattern 2 at 12.5 MHz", mode_str(), dfs);
        run_case(2, b);
        ncase++; ngrp++; if (b != 0) begin nbad++; nbad_grp++; end
      end
      $display("  %s  %s DFS=%2d  %0d/%0d cases", (nbad_grp != 0) ? "FAIL" : "PASS",
               mode_str(), dfs, ngrp - nbad_grp, ngrp);
    end
    $display("\n  %0d/%0d cases passed", ncase - nbad, ncase);
    if (nbad != 0)         $display("  %0d FAILURES\n", nbad);
    else if (ncase != 240) $display("  ONLY %0d OF 240 CASES RAN\n", ncase);
    else                   $display("  ALL SLAVE CASES PASS\n");
    $finish;
  end
  initial begin #20_000_000; $display("TIMEOUT"); $finish; end
`ifdef DUMP
  initial begin
    $dumpfile("tb_slave.vcd");
    $dumpvars(0, tb_slave);
  end
`endif

endmodule
