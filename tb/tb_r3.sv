// tb_r3.sv - self-checking tests for the revision 3 RTL changes (spec Appendix B, items 1-5)
//   +hp=<ps> +hr=<ps>  pclk / spi_ref_clk half periods in ps (default 5000 / 17000: 100 / 29.4 MHz)
//   +test=<name>       all (default) | reset | enrace | csonce | enstop | manual | wm | commit |
//                      swrst2 | slave | slvdone
//   -DNCS=<n>          NUM_CS for the build (default 4); 5 must be rejected at elaboration
// Every test starts from a hardware reset. Last line: RESULT <n> pass <m> fail
// verilator lint_off BLKSEQ
// The clock periods (+hp/+hr) and the SCLK half period are run-time values, so
// newer Verilator (5.046+) cannot prove these delays non-zero (ZERODLY, a speed hint)
// verilator lint_off ZERODLY
`timescale 1ns/1ps
`ifndef NCS
  `define NCS 4
`endif
`ifndef EN_PROBE
  `define EN_PROBE dut.en_spi
`endif
module tb_r3;
  int  hp_ps = 5000, hr_ps = 17000;
  real T;                                               // spi_ref_clk period in ns
  logic pclk = 1'b0, spi_ref_clk = 1'b0, presetn = 1'b1, spi_rst_n = 1'b1;
  initial begin
    void'($value$plusargs("hp=%d", hp_ps));
    forever #(hp_ps / 1000.0) pclk = ~pclk;
  end
  initial begin
    void'($value$plusargs("hr=%d", hr_ps));
    #0.3;                                               // the two clocks are unrelated
    forever #(hr_ps / 1000.0) spi_ref_clk = ~spi_ref_clk;
  end

  logic psel = 0, penable = 0, pwrite = 0; logic [7:0] paddr = 0;
  logic [31:0] pwdata = 0, prdata; logic pready, pslverr;
  logic sclk_o, sclk_oe, mosi_o, mosi_oe, miso_o, miso_oe, cs_oe, irq, tx_dreq, rx_dreq;
  logic sclk_i = 0, mosi_i = 0, cs_n_i = 1, miso_i;
  logic [`NCS-1:0] cs_n_o;
  bit   loop_en = 0;
  assign miso_i = loop_en ? mosi_o : 1'b0;              // wire loop on the pins (master tests)

  spi_controller_top #(.DATA_WIDTH(32), .FIFO_DEPTH(8), .NUM_CS(`NCS)) dut (
    .pclk(pclk), .presetn(presetn), .spi_ref_clk(spi_ref_clk), .spi_rst_n(spi_rst_n),
    .psel(psel), .penable(penable), .pwrite(pwrite), .paddr(paddr), .pwdata(pwdata),
    .prdata(prdata), .pready(pready), .pslverr(pslverr),
    .sclk_o(sclk_o), .sclk_i(sclk_i), .sclk_oe(sclk_oe), .mosi_o(mosi_o), .mosi_i(mosi_i),
    .mosi_oe(mosi_oe), .miso_o(miso_o), .miso_i(miso_i), .miso_oe(miso_oe),
    .cs_n_o(cs_n_o), .cs_n_i(cs_n_i), .cs_oe(cs_oe),
    .irq(irq), .tx_dreq(tx_dreq), .rx_dreq(rx_dreq));

  localparam logic [7:0] CTRL = 8'h00, STAT = 8'h04, TXD = 8'h08, RXD = 8'h0C, CLKDIV = 8'h10,
                         IS = 8'h18, CSCTRL = 8'h1C, FIFOCTRL = 8'h20, FIFOLVL = 8'h24;

  // ---------------- APB ----------------
  task automatic wr_e(input [7:0] a, input [31:0] d, output logic e);
    @(posedge pclk); #0.1; psel = 1; pwrite = 1; paddr = a; pwdata = d; penable = 0;
    @(posedge pclk); #0.1; penable = 1;
    @(posedge pclk); e = pslverr; #0.1; psel = 0; penable = 0; pwrite = 0;
  endtask
  task automatic wr(input [7:0] a, input [31:0] d);
    logic e; wr_e(a, d, e);
  endtask
  task automatic rd(input [7:0] a, output logic [31:0] d);
    @(posedge pclk); #0.1; psel = 1; pwrite = 0; paddr = a; penable = 0;
    @(posedge pclk); #0.1; penable = 1;
    @(posedge pclk); d = prdata; #0.1; psel = 0; penable = 0;
  endtask
  // two writes back to back: SETUP, ACCESS, SETUP, ACCESS (2 pclk apart)
  task automatic wr2(input [7:0] a1, input [31:0] d1, input [7:0] a2, input [31:0] d2);
    @(posedge pclk); #0.1; psel = 1; pwrite = 1; paddr = a1; pwdata = d1; penable = 0;
    @(posedge pclk); #0.1; penable = 1;
    @(posedge pclk); #0.1; penable = 0; paddr = a2; pwdata = d2;
    @(posedge pclk); #0.1; penable = 1;
    @(posedge pclk); #0.1; psel = 0; penable = 0; pwrite = 0;
  endtask

  // ---------------- checking ----------------
  int npass = 0, nfail = 0;
  task automatic check(input string name, input bit ok, input string detail);
    if (ok) begin npass++; $display("  PASS  %s  (%s)", name, detail); end
    else    begin nfail++; $display("  FAIL  %s  (%s)", name, detail); end
  endtask
  function automatic bit near(input real a, input real b);
    return (a > b - 0.25) && (a < b + 0.25);
  endfunction

  // ---------------- pin monitors (master tests) ----------------
  // The master pins are registered on spi_ref_clk, so sampling just after each
  // spi_ref_clk edge records every change at its exact clock edge.
  bit mon = 0;
  realtime cs_fall[$], cs_rise[$], sclk_e[$], oe_rise[$], oe_fall[$];
  realtime en_fall_t;
  logic    idle_at_fall[$];
  int      stale;                                       // 2-state: starts at 0
  logic    exp_idle = 1'b0;
  logic    cs_q, oe_q, sclk_q, en_q;                    // previous values; mon is 0 until they are set
  always @(posedge spi_ref_clk) begin
    #0.1;
    if (mon) begin
      if (cs_q === 1'b1 && cs_n_o[0] === 1'b0) begin cs_fall.push_back($realtime); idle_at_fall.push_back(sclk_o); end
      if (cs_q === 1'b0 && cs_n_o[0] === 1'b1) cs_rise.push_back($realtime);
      if (cs_n_o[0] === 1'b0 && sclk_o !== sclk_q) sclk_e.push_back($realtime);
      if (oe_q === 1'b0 && sclk_oe === 1'b1) oe_rise.push_back($realtime);
      if (oe_q === 1'b1 && sclk_oe === 1'b0) oe_fall.push_back($realtime);
      if (en_q === 1'b1 && `EN_PROBE === 1'b0) en_fall_t = $realtime;
      // SCLK driven while no chip-select is asserted must sit at the CPOL idle level
      if (sclk_oe === 1'b1 && cs_n_o[0] === 1'b1 && sclk_o !== exp_idle) stale++;
    end
    cs_q = cs_n_o[0]; oe_q = sclk_oe; sclk_q = sclk_o; en_q = `EN_PROBE;
  end

  task automatic mon_start(input logic idle);
    cs_fall.delete(); cs_rise.delete(); sclk_e.delete(); oe_rise.delete(); oe_fall.delete();
    idle_at_fall.delete(); stale = 0; exp_idle = idle; mon = 1;
  endtask

  task automatic wait_ref(input int n); repeat (n) @(posedge spi_ref_clk); endtask
  task automatic wait_edges(input int n);
    int k = 0;
    while (sclk_e.size() < n && k < 20000) begin @(posedge spi_ref_clk); k++; end
  endtask
  task automatic wait_cs_rise(input int n);
    int k = 0;
    while (cs_rise.size() < n && k < 20000) begin @(posedge spi_ref_clk); k++; end
  endtask
  task automatic wait_rx_level(input int n);
    logic [31:0] l; int k = 0;
    do begin rd(FIFOLVL, l); k++; end while (l[15:8] != 8'(n) && k < 2000);
  endtask
  task automatic wait_not_busy();
    logic [31:0] st; int k = 0;
    wait_ref(8);
    do begin rd(STAT, st); k++; end while (st[0] && k < 2000);
  endtask

  task automatic reset_dut();
    mon = 0; loop_en = 0; cs_n_i = 1; sclk_i = 0; mosi_i = 0;
    repeat (3) @(posedge spi_ref_clk); presetn = 0; spi_rst_n = 0;
    repeat (4) @(posedge spi_ref_clk); repeat (2) @(posedge pclk); #0.2; presetn = 1; spi_rst_n = 1;
    repeat (6) @(posedge pclk); repeat (6) @(posedge spi_ref_clk);
  endtask

  logic [31:0] v, st, lvl, isr; logic e;
  logic [31:0] w [0:7];

  // ---------------- reset: no SPI pin driven ----------------
  task automatic t_reset();
    $display("[reset] outputs after reset");
    check("reset: no SPI pin driven", {sclk_oe, mosi_oe, cs_oe, miso_oe} === 4'b0000,
          $sformatf("sclk_oe %b mosi_oe %b cs_oe %b miso_oe %b", sclk_oe, mosi_oe, cs_oe, miso_oe));
    check("reset: every chip-select high", cs_n_o === '1, $sformatf("cs_n_o %b", cs_n_o));
  endtask

  // ---------------- EN and configuration in one write ----------------
  task automatic t_enrace();
    $display("[enrace] one CTRL write sets EN = 1 together with CPOL, CPHA, LSBFIRST and DFS");
    wr(CLKDIV, 2); wr(CSCTRL, 32'h4 | (3 << 3) | (3 << 9));           // CS_AUTO, setup 3, hold 3
    wr(TXD, w[0]); wr(TXD, w[1]);                                     // EN = 0: words wait
    loop_en = 1; mon_start(1'b1);
    wr(CTRL, 32'h1 | (1 << 2) | (1 << 3) | (1 << 4) | (12 << 8));     // EN, CPOL 1, CPHA 1, LSB, DFS 12
    wait_edges(48); wait_cs_rise(1); wait_ref(10);
    check("first frame uses the CPOL written with EN (SCLK idle 1 when CS falls)",
          idle_at_fall.size() == 1 && idle_at_fall[0] === 1'b1,
          $sformatf("%0d CS assertion(s), SCLK at CS fall %b", idle_at_fall.size(),
                    idle_at_fall.size() ? idle_at_fall[0] : 1'bx));
    check("SCLK never driven at a stale idle level", stale == 0, $sformatf("%0d ref cycles", stale));
    check("DFS 12 from the same write: 2 words x 24 edges under one chip-select",
          sclk_e.size() == 48 && cs_fall.size() == 1, $sformatf("%0d edges, %0d CS assertion(s)",
          sclk_e.size(), cs_fall.size()));
    rd(RXD, v); check("RX word 1 through the pin loop", v == (w[0] & 32'hFFF), $sformatf("0x%0h", v));
    rd(RXD, v); check("RX word 2 through the pin loop", v == (w[1] & 32'hFFF), $sformatf("0x%0h", v));
    check("EN = 1: SCLK, MOSI and CS outputs driven, MISO not", {sclk_oe, mosi_oe, cs_oe, miso_oe} === 4'b1110,
          $sformatf("%b%b%b%b", sclk_oe, mosi_oe, cs_oe, miso_oe));
    check("idle: SCLK parked at CPOL = 1, chip-selects high", sclk_o === 1'b1 && cs_n_o === '1,
          $sformatf("sclk_o %b cs_n_o %b", sclk_o, cs_n_o));
    mon = 0;
    wr(CTRL, (1 << 2) | (1 << 3) | (1 << 4) | (12 << 8));             // EN = 0
    wait_ref(20);
    check("EN = 0 when idle: SPI outputs released", {sclk_oe, mosi_oe, cs_oe} === 3'b000,
          $sformatf("%b%b%b", sclk_oe, mosi_oe, cs_oe));
  endtask

  // ---------------- CS_SETUP once per chip-select assertion ----------------
  task automatic t_csonce();
    real css, ifg1, ifg2, csh;
    $display("[csonce] 3 queued words, CS_AUTO = 1, CLKDIV 1, CS_SETUP 9, CS_HOLD 5");
    wr(CLKDIV, 1); wr(CSCTRL, 32'h4 | (9 << 3) | (5 << 9));
    for (int k = 0; k < 3; k++) wr(TXD, w[k]);
    loop_en = 1; mon_start(1'b0);
    wr(CTRL, 32'h1 | (8 << 8));
    wait_edges(48); wait_cs_rise(1); wait_ref(10); mon = 0;
    if (sclk_e.size() < 48 || cs_fall.size() < 1 || cs_rise.size() < 1) begin
      check("3 words sent", 0, $sformatf("%0d edges", sclk_e.size()));
    end else begin
      css  = (sclk_e[0]  - cs_fall[0]) / T;
      ifg1 = (sclk_e[16] - sclk_e[15]) / T;
      ifg2 = (sclk_e[32] - sclk_e[31]) / T;
      csh  = (cs_rise[0] - sclk_e[47]) / T;
      check("one chip-select assertion for 3 queued words", cs_fall.size() == 1 && cs_rise.size() == 1,
            $sformatf("%0d falls, %0d rises", cs_fall.size(), cs_rise.size()));
      check("t_CSS = (CS_SETUP + CLKDIV + 1) T = 11 T", near(css, 11.0), $sformatf("%0.2f T", css));
      check("t_IFG word 1 -> 2 = CLKDIV + 2 = 3 T (spec: at most CLKDIV + 3)", near(ifg1, 3.0),
            $sformatf("%0.2f T", ifg1));
      check("t_IFG word 2 -> 3 = 3 T, CS_SETUP not re-applied", near(ifg2, 3.0), $sformatf("%0.2f T", ifg2));
      check("t_CSH = (CS_HOLD + 2) T = 7 T", near(csh, 7.0), $sformatf("%0.2f T", csh));
      for (int k = 0; k < 3; k++) begin
        rd(RXD, v); check($sformatf("RX word %0d", k + 1), v == (w[k] & 32'hFF), $sformatf("0x%0h", v));
      end
    end
  endtask

  // ---------------- EN = 0 in the middle of a burst ----------------
  task automatic t_enstop();
    real csh;
    $display("[enstop] 4 queued 16-bit words, EN cleared during word 2");
    wr(CLKDIV, 3); wr(CSCTRL, 32'h4 | (2 << 3) | (4 << 9));
    for (int k = 0; k < 4; k++) wr(TXD, w[k]);
    loop_en = 1; mon_start(1'b0);
    wr(CTRL, 32'h1 | (16 << 8));
    wait_edges(32 + 10);                                              // inside word 2
    wr(CTRL, (16 << 8));                                              // EN = 0
    wait_cs_rise(1); wait_ref(20); rd(STAT, st); mon = 0;
    check("word in progress completes, no further word is loaded", sclk_e.size() == 64,
          $sformatf("%0d SCLK edges, expected 64", sclk_e.size()));
    csh = (cs_rise.size() && sclk_e.size()) ? (cs_rise[0] - sclk_e[sclk_e.size() - 1]) / T : -1.0;
    check("chip-select deasserted after CS_HOLD: t_CSH = 6 T", near(csh, 6.0), $sformatf("%0.2f T", csh));
    check("outputs released only after chip-select is high",
          oe_fall.size() == 1 && cs_rise.size() == 1 && oe_fall[0] > cs_rise[0],
          $sformatf("OE fell %0d time(s), %0.1f ns after CS rose", oe_fall.size(),
                    (oe_fall.size() && cs_rise.size()) ? oe_fall[0] - cs_rise[0] : -1.0));
    check("FSM idle: BUSY = 0", !st[0], $sformatf("STAT 0x%0h", st));
    rd(FIFOLVL, lvl);
    check("queued TX words preserved (TX level 2), 2 words received", lvl[7:0] == 2 && lvl[15:8] == 2,
          $sformatf("TX %0d RX %0d", lvl[7:0], lvl[15:8]));
    rd(RXD, v); check("RX word 1", v == (w[0] & 32'hFFFF), $sformatf("0x%0h", v));
    rd(RXD, v); check("RX word 2", v == (w[1] & 32'hFFFF), $sformatf("0x%0h", v));
    mon_start(1'b0);
    wr(CTRL, 32'h1 | (16 << 8));                                      // EN = 1 again
    wait_edges(64); wait_cs_rise(1); wait_ref(10); mon = 0;
    rd(FIFOLVL, lvl);
    check("EN = 1 again sends the 2 preserved words", lvl[7:0] == 0 && lvl[15:8] == 2 && sclk_e.size() == 64,
          $sformatf("TX %0d RX %0d, %0d edges", lvl[7:0], lvl[15:8], sclk_e.size()));
    rd(RXD, v); check("RX word 3", v == (w[2] & 32'hFFFF), $sformatf("0x%0h", v));
    rd(RXD, v); check("RX word 4", v == (w[3] & 32'hFFFF), $sformatf("0x%0h", v));
  endtask

  // ---------------- manual chip-select (CS_AUTO = 0) and the STALL exit ----------------
  task automatic t_manual();
    real css, hold_t;
    $display("[manual] CS_AUTO = 0, CS_SETUP 4, CS_HOLD 6: STALL, second word, EN = 0 ends the frame");
    wr(CLKDIV, 1); wr(CSCTRL, (4 << 3) | (6 << 9));                   // CS_AUTO = 0
    loop_en = 1; mon_start(1'b0);
    wr(CTRL, 32'h1 | (8 << 8)); repeat (8) @(posedge pclk); wait_ref(10);   // EN path: ~4 pclk + 5 ref
    check("EN = 1, TX empty: outputs driven, chip-select high", sclk_oe && cs_oe && cs_n_o[0],
          $sformatf("sclk_oe %b cs_oe %b cs_n_o[0] %b", sclk_oe, cs_oe, cs_n_o[0]));
    wr(TXD, w[4]); wait_edges(16); wait_ref(10); rd(STAT, st);
    check("one word, TX empty: TX_STALL with chip-select held low", st[10] && st[0] && !cs_n_o[0],
          $sformatf("TX_STALL %b BUSY %b cs_n_o[0] %b", st[10], st[0], cs_n_o[0]));
    css = (sclk_e.size() && cs_fall.size()) ? (sclk_e[0] - cs_fall[0]) / T : -1.0;
    check("CS_SETUP applied in manual mode: t_CSS = 6 T", near(css, 6.0), $sformatf("%0.2f T", css));
    wr(TXD, w[5]); wait_edges(32); wait_ref(10);
    check("second word under the same chip-select", cs_fall.size() == 1 && cs_rise.size() == 0,
          $sformatf("%0d falls %0d rises", cs_fall.size(), cs_rise.size()));
    wr(IS, 32'h3F);
    wr(CTRL, (8 << 8));                                               // EN = 0 during the STALL
    wait_cs_rise(1); wait_ref(20); rd(STAT, st); rd(IS, isr); mon = 0;
    check("EN = 0 ends the STALL: chip-select high, BUSY = 0, TX_STALL = 0, no SWRST",
          cs_rise.size() == 1 && !st[0] && !st[10], $sformatf("%0d rises, STAT 0x%0h", cs_rise.size(), st));
    hold_t = cs_rise.size() ? (cs_rise[0] - en_fall_t) / T : -1.0;
    check("CS_HOLD applied in manual mode (EN seen to CS high: CS_HOLD + 3 = 9 T)", near(hold_t, 9.0),
          $sformatf("%0.2f T", hold_t));
    check("outputs released after chip-select is high",
          oe_fall.size() == 1 && cs_rise.size() == 1 && oe_fall[0] > cs_rise[0], $sformatf("%0d OE fall(s)", oe_fall.size()));
    check("DONE_INT latched for the chip-select frame", isr[4], $sformatf("SPI_IS 0x%0h", isr));
    rd(RXD, v); check("RX word 1", v == (w[4] & 32'hFF), $sformatf("0x%0h", v));
    rd(RXD, v); check("RX word 2", v == (w[5] & 32'hFF), $sformatf("0x%0h", v));
  endtask

  // ---------------- watermark range and full-width compare (B12) ----------------
  task automatic t_wm();
    $display("[wm] watermark levels 0..FIFO_DEPTH, PSLVERR on out-of-range writes");
    for (int k = 0; k < 3; k++) wr(TXD, w[k]);                        // EN = 0: TX level 3
    wr_e(FIFOCTRL, (8 << 8) | (8 << 16), e); rd(FIFOCTRL, v); rd(STAT, st);
    check("TX_WM_LVL = RX_WM_LVL = FIFO_DEPTH accepted", !e && v[15:8] == 8 && v[23:16] == 8,
          $sformatf("PSLVERR %b, reads TX %0d RX %0d", e, v[15:8], v[23:16]));
    check("TX level 3 <= TX_WM_LVL 8: TX_WM = 1 (full-width compare)", st[5], $sformatf("STAT 0x%0h", st));
    wr(FIFOCTRL, (2 << 8) | (8 << 16)); rd(STAT, st);
    check("TX level 3, TX_WM_LVL 2: TX_WM = 0", !st[5], $sformatf("STAT 0x%0h", st));
    wr(FIFOCTRL, (3 << 8) | (8 << 16)); rd(STAT, st);
    check("TX level 3, TX_WM_LVL 3: TX_WM = 1", st[5], $sformatf("STAT 0x%0h", st));
    wr_e(FIFOCTRL, (9 << 8) | (8 << 16), e); rd(FIFOCTRL, v);
    check("TX_WM_LVL 9 rejected: PSLVERR, register unchanged", e && v[15:8] == 3 && v[23:16] == 8,
          $sformatf("PSLVERR %b, reads TX %0d RX %0d", e, v[15:8], v[23:16]));
    wr_e(FIFOCTRL, (4 << 8) | (0 << 16), e); rd(FIFOCTRL, v);
    check("RX_WM_LVL 0 rejected: PSLVERR, register unchanged", e && v[15:8] == 3 && v[23:16] == 8,
          $sformatf("PSLVERR %b, reads TX %0d RX %0d", e, v[15:8], v[23:16]));
    wr_e(FIFOCTRL, (4 << 8) | (9 << 16), e);
    check("RX_WM_LVL 9 rejected: PSLVERR", e, $sformatf("PSLVERR %b", e));
    wr_e(FIFOCTRL, 32'h1 | (9 << 8) | (8 << 16), e); rd(FIFOLVL, lvl);
    check("rejected write performs no flush", e && lvl[7:0] == 3, $sformatf("PSLVERR %b, TX level %0d", e, lvl[7:0]));
    wr_e(FIFOCTRL, 32'h1 | (0 << 8) | (1 << 16), e); rd(FIFOLVL, lvl); rd(STAT, st);
    check("valid flush with TX_WM_LVL 0: TX level 0, TX_WM = 1", !e && lvl[7:0] == 0 && st[5],
          $sformatf("PSLVERR %b, TX level %0d, STAT 0x%0h", e, lvl[7:0], st));
    // RX side at full depth, through the internal loopback
    wr(FIFOCTRL, (4 << 8) | (8 << 16)); wr(CLKDIV, 0); wr(CSCTRL, 32'h4);
    wr(CTRL, 32'h1 | (1 << 5) | (4 << 8));                            // EN, LOOPBACK, DFS 4
    for (int k = 0; k < 7; k++) wr(TXD, 32'(k));
    wait_rx_level(7); rd(STAT, st);
    check("RX level 7, RX_WM_LVL 8: RX_WM = 0", !st[6], $sformatf("STAT 0x%0h", st));
    wr(TXD, 32'h7); wait_rx_level(8); rd(STAT, st);
    check("RX level 8 = FIFO_DEPTH, RX_WM_LVL 8: RX_WM = 1, RX_FULL = 1", st[6] && st[4], $sformatf("STAT 0x%0h", st));
    wr(CTRL, (1 << 5) | (4 << 8));
  endtask

  // ---------------- two configuration writes back to back (B13) ----------------
  task automatic t_commit();
    int nbad = 0, maxw = 0, nw;
    logic [7:0] cd; logic [5:0] su;
    $display("[commit] 200 pairs of back-to-back writes: CLKDIV, then CSCTRL 2 pclk later");
    for (int i = 0; i < 200; i++) begin
      cd = 8'(i); su = 6'(i * 7 + 1);
      repeat (1 + (i % 7)) @(posedge pclk);             // walk the pclk / ref phase
      wr2(CLKDIV, 32'(cd), CSCTRL, 32'h4 | (32'(su) << 3));
      nw = 0;
      while ((dut.cfg_active.clkdiv !== cd || dut.cfg_active.cs_setup !== su) && nw < 40) begin
        @(posedge pclk); repeat (2) @(posedge spi_ref_clk); nw++;
      end
      if (nw > maxw) maxw = nw;
      if (dut.cfg_active.clkdiv !== cd || dut.cfg_active.cs_setup !== su) nbad++;
    end
    check("every back-to-back write pair reaches the SPI domain", nbad == 0,
          $sformatf("%0d of 200 lost, worst wait %0d steps of 1 pclk + 2 ref edges", nbad, maxw));
  endtask

  // ---------------- two SWRST writes back to back (B13) ----------------
  task automatic t_swrst2();
    int k, act;
    $display("[swrst2] two SWRST writes 2 pclk apart");
    for (int j = 0; j < 3; j++) wr(TXD, w[j]);
    wr2(CTRL, 32'h4800, CTRL, 32'h4800);
    k = 0;
    do begin rd(STAT, st); k++; end while (st[11] && k < 500);
    check("RST_BUSY clears", !st[11], $sformatf("after %0d status reads", k));
    act = 0;
    for (int j = 0; j < 200; j++) begin @(posedge spi_ref_clk); if (dut.swrst_active) act++; end
    rd(STAT, st); rd(FIFOLVL, lvl);
    check("no second reset sequence after RST_BUSY = 0", act == 0 && !st[11],
          $sformatf("%0d ref cycles of reset activity, RST_BUSY %b", act, st[11]));
    check("the reset was performed: TX FIFO empty", lvl[7:0] == 0, $sformatf("TX level %0d", lvl[7:0]));
  endtask

  // ---------------- slave mode: EN gating at frame boundaries ----------------
  int  half;                                            // SCLK half period of the external master, ns
  bit  oe_seen, slv_mon;                                // 2-state: start at 0
  always @(posedge miso_oe) if (slv_mon) oe_seen = 1;

  task automatic xword(input [7:0] wd, output logic [7:0] g);     // mode 0, MSB first, 8 bits
    for (int i = 7; i >= 0; i--) begin
      mosi_i = wd[i]; #(half); sclk_i = 1'b1; g[i] = miso_o; #(half); sclk_i = 1'b0;
    end
  endtask
  task automatic cs_low();  #0.5; cs_n_i = 1'b0; #(2 * half); endtask
  task automatic cs_high(); #(2 * half); cs_n_i = 1'b1; #(8 * half); wait_ref(12); wait_ref(1); endtask

  task automatic t_slave();
    logic [7:0] g0, g1;
    $display("[slave] EN = 0 ignores chip-select; EN changes apply at frame boundaries");
    wr(CTRL, (1 << 1) | (8 << 8));                                    // slave, EN = 0, mode 0, DFS 8
    wr(TXD, w[0]); wr(TXD, w[1]); wait_ref(20);
    slv_mon = 1; oe_seen = 0;
    cs_low(); xword(8'hA1, g0); xword(8'hB2, g1); cs_high();
    rd(FIFOLVL, lvl); rd(IS, isr);
    check("EN = 0: MISO never driven", !oe_seen, "");
    check("EN = 0: no RX word written", lvl[15:8] == 0, $sformatf("RX level %0d", lvl[15:8]));
    check("EN = 0: no TX word used (TX level 2, nothing staged)", lvl[7:0] == 2 && !dut.u_slave.tx_hold_vld,
          $sformatf("TX level %0d, staged %b", lvl[7:0], dut.u_slave.tx_hold_vld));
    check("EN = 0: no event raised (DONE, TXUNF, RXOVF, FRERR)", isr[5:2] == 0, $sformatf("SPI_IS 0x%0h", isr));
    // EN = 1 written while selected: that frame is still ignored
    oe_seen = 0;
    cs_low(); xword(8'hC3, g0); wr(CTRL, 32'h1 | (1 << 1) | (8 << 8)); xword(8'hD4, g1); cs_high();
    rd(FIFOLVL, lvl);
    check("EN = 1 written mid-frame: that frame still ignored", lvl[15:8] == 0 && !oe_seen,
          $sformatf("RX level %0d, MISO driven %b", lvl[15:8], oe_seen));
    // next frame: received
    oe_seen = 0;
    cs_low(); xword(8'h5A, g0); xword(8'h3C, g1); cs_high();
    rd(FIFOLVL, lvl); rd(IS, isr);
    check("next frame: MISO driven, TX words sent", oe_seen && g0 == w[0][7:0] && g1 == w[1][7:0],
          $sformatf("MISO driven %b, 0x%0h 0x%0h", oe_seen, g0, g1));
    check("next frame: 2 RX words, DONE latched", lvl[15:8] == 2 && isr[4], $sformatf("RX %0d, SPI_IS 0x%0h", lvl[15:8], isr));
    rd(RXD, v); check("RX word 1", v == 32'h5A, $sformatf("0x%0h", v));
    rd(RXD, v); check("RX word 2", v == 32'h3C, $sformatf("0x%0h", v));
    // EN = 0 written mid-frame: that frame completes, the next one is ignored
    wr(TXD, w[2]); wr(TXD, w[3]); wait_ref(20);
    cs_low(); xword(8'h11, g0); wr(CTRL, (1 << 1) | (8 << 8)); xword(8'h22, g1); cs_high();
    rd(FIFOLVL, lvl);
    check("EN = 0 written mid-frame: that frame completes", lvl[15:8] == 2 && g0 == w[2][7:0] && g1 == w[3][7:0],
          $sformatf("RX %0d, MISO 0x%0h 0x%0h", lvl[15:8], g0, g1));
    oe_seen = 0;
    cs_low(); xword(8'h33, g0); xword(8'h44, g1); cs_high();
    rd(FIFOLVL, lvl);
    check("following frame ignored", lvl[15:8] == 2 && !oe_seen, $sformatf("RX %0d, MISO driven %b", lvl[15:8], oe_seen));
    // aborted word while off: no FRAME_ERR
    wr(IS, 32'h3F);
    cs_low(); mosi_i = 1'b1; for (int i = 0; i < 3; i++) begin #(half); sclk_i = 1'b1; #(half); sclk_i = 1'b0; end
    cs_high(); rd(IS, isr);
    check("EN = 0: aborted word raises no FRAME_ERR", !isr[5], $sformatf("SPI_IS 0x%0h", isr));
    slv_mon = 0;
  endtask

  // ---------------- slave DONE events closer than one pclk period (B13, SPI -> bus) ----------------
  task automatic t_slvdone();
    logic [7:0] g0, g1;
    int miss = 0, nt = 60;
    int h_save;
    $display("[slvdone] slave, two 8-bit words per frame, SPI_IS.DONE checked after every frame");
    h_save = half;
    // SCLK as fast as the slave limit allows, (DFS-1) x T(SCLK) > 4 x T(ref): two words then fit in
    // one pclk period when pclk is 8x slower, which is where two DONE toggles used to cancel
    half = int'(T * 0.4) + 1;
    wr(CTRL, 32'h1 | (1 << 1) | (8 << 8));
    repeat (10) @(posedge pclk); wait_ref(20);          // EN reaches the slave before the first frame
    for (int i = 0; i < nt; i++) begin
      wr(FIFOCTRL, 32'h2 | (4 << 8) | (4 << 16));        // flush RX
      wr(IS, 32'h3F); wait_ref(1 + i % 5);
      #((i * 13) % 271);                                  // sweep the frame phase against pclk
      cs_low(); xword(8'(i), g0); xword(8'(i + 1), g1); cs_high();
      repeat (4) @(posedge pclk); rd(IS, isr);
      if (!isr[4]) begin miss++; $display("  frame %0d: SPI_IS 0x%0h", i, isr); end
    end
    half = h_save;
    check("DONE latched after every frame", miss == 0, $sformatf("%0d of %0d frames without DONE", miss, nt));
  endtask

  string test = "all";
  initial begin
    void'($value$plusargs("test=%s", test));
    #1; T = 2.0 * hr_ps / 1000.0;
    half = (int'(4.0 * T) > 100) ? int'(4.0 * T) : 100;
    for (int k = 0; k < 8; k++) w[k] = 32'hC2A9_35F1 ^ (32'(k) * 32'h9E37_79B9);
    $display("tb_r3: pclk %0.2f ns, spi_ref_clk %0.2f ns (pclk : ref = %0.2f), NUM_CS %0d, test=%s",
             2.0 * hp_ps / 1000.0, T, (2.0 * hr_ps) / (2.0 * hp_ps), `NCS, test);
    if (test == "all" || test == "reset")   begin reset_dut(); t_reset();   end
    if (test == "all" || test == "enrace")  begin reset_dut(); t_enrace();  end
    if (test == "all" || test == "csonce")  begin reset_dut(); t_csonce();  end
    if (test == "all" || test == "enstop")  begin reset_dut(); t_enstop();  end
    if (test == "all" || test == "manual")  begin reset_dut(); t_manual();  end
    if (test == "all" || test == "wm")      begin reset_dut(); t_wm();      end
    if (test == "all" || test == "commit")  begin reset_dut(); t_commit();  end
    if (test == "all" || test == "swrst2")  begin reset_dut(); t_swrst2();  end
    if (test == "all" || test == "slave")   begin reset_dut(); t_slave();   end
    if (test == "all" || test == "slvdone") begin reset_dut(); t_slvdone(); end
    $display("RESULT %0d pass %0d fail", npass, nfail);
    $finish;
  end
  initial begin #50_000_000; $display("TIMEOUT"); $display("RESULT %0d pass %0d fail", npass, nfail + 1); $finish; end
endmodule
