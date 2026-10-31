// tb_flush.sv - randomized FIFO flush and SWRST stress (bugs B14, B15; tracker FI-03, RS-01)
//   +hp=<ps> +hr=<ps>   pclk / spi_ref_clk half periods (default 5000 / 17000: 100 / 29.4 MHz)
//   +ph=<ps>            spi_ref_clk start offset (default 300)
//   +seed=<n>           random seed (default 1)      +iters=<n>  random steps (default 150)
//   -DDEPTH=<n>         FIFO_DEPTH of the build (default 8; Verilator: +define+DEPTH=<n>)
//   +only=<step>        run only one kind of step (txflush, rxflush, txidle, slave, swrst)
//   +trace              dump the last 600 clock edges of FIFO state at the first failure
//   -DNO_R4_PROBES      leave out the coverage probes of R4 FIFO internals (to run on older RTL)
// Steps, picked at random:
//   txflush  EN = 0: write, flush, write, flush ... then send through the internal loopback:
//            exactly the words written after the last flush must come out, in order, once
//   rxflush  RX words waiting, some read, RX flush: the bus side must show an empty RX FIFO on
//            every read after the flush, latch no RXWM, and only later words come out
//   txidle   TX flush with EN = 1 and the TX FIFO empty: no frame starts
//   slave    slave mode, TX words written (one gets staged), TX flush, more words, one frame
//            from the TB: MISO carries only the words written after the flush (bug B14)
//   swrst    SWRST with data in both FIFOs: levels read 0 during RST_BUSY, everything empty
//            after it, SPI_IS = TXWM only, and the FIFOs work afterwards
// Also checked on every clock: no FIFO level above FIFO_DEPTH, and no FIFO ever holds more
// than FIFO_DEPTH words between its write and read pointers.
// Last line: RESULT <n> pass <m> fail
// verilator lint_off BLKSEQ
// verilator lint_off ZERODLY
`timescale 1ns/1ps
`ifndef DEPTH
  `define DEPTH 8
`endif
module tb_flush;
  localparam int DEPTH = `DEPTH;
  localparam int AW    = $clog2(DEPTH);
  localparam int HALF  = 100;                           // slave SCLK half period, ns

  int hp_ps = 5000, hr_ps = 17000, ph_ps = 300, seed = 1, iters = 150;
  string only = "";                                     // +only=<step>: run one kind of step
  logic pclk = 1'b0, spi_ref_clk = 1'b0, presetn = 1'b1, spi_rst_n = 1'b1;
  initial begin
    void'($value$plusargs("hp=%d", hp_ps));
    forever #(hp_ps / 1000.0) pclk = ~pclk;
  end
  initial begin
    void'($value$plusargs("hr=%d", hr_ps));
    void'($value$plusargs("ph=%d", ph_ps));
    #(ph_ps / 1000.0);
    forever #(hr_ps / 1000.0) spi_ref_clk = ~spi_ref_clk;
  end

  logic psel = 0, penable = 0, pwrite = 0; logic [7:0] paddr = 0;
  logic [31:0] pwdata = 0, prdata; logic pready, pslverr;
  logic sclk_o, sclk_oe, mosi_o, mosi_oe, miso_o, miso_oe, cs_oe, irq, tx_dreq, rx_dreq;
  logic sclk_i = 0, mosi_i = 0, cs_n_i = 1;
  logic [3:0] cs_n_o;

  spi_controller_top #(.DATA_WIDTH(32), .FIFO_DEPTH(DEPTH), .NUM_CS(4)) dut (
    .pclk(pclk), .presetn(presetn), .spi_ref_clk(spi_ref_clk), .spi_rst_n(spi_rst_n),
    .psel(psel), .penable(penable), .pwrite(pwrite), .paddr(paddr), .pwdata(pwdata),
    .prdata(prdata), .pready(pready), .pslverr(pslverr),
    .sclk_o(sclk_o), .sclk_i(sclk_i), .sclk_oe(sclk_oe), .mosi_o(mosi_o), .mosi_i(mosi_i),
    .mosi_oe(mosi_oe), .miso_o(miso_o), .miso_i(1'b0), .miso_oe(miso_oe),
    .cs_n_o(cs_n_o), .cs_n_i(cs_n_i), .cs_oe(cs_oe),
    .irq(irq), .tx_dreq(tx_dreq), .rx_dreq(rx_dreq));

  localparam logic [7:0] CTRL = 8'h00, STAT = 8'h04, TXD = 8'h08, RXD = 8'h0C, CLKDIV = 8'h10,
                         IS = 8'h18, CSCTRL = 8'h1C, FIFOCTRL = 8'h20, FIFOLVL = 8'h24;
  localparam logic [31:0] C_EN = 32'h1, C_MODE = 32'h2, C_LOOP = 32'h20, C_SWRST = 32'h4000;
  localparam logic [31:0] D32 = 32'd32 << 8, D8 = 32'd8 << 8;

  // ---------------- checking ----------------
  int npass = 0, nfail = 0;
  task automatic check(input string name, input bit ok, input string detail);
    if (ok) npass++;
    else begin
      nfail++;
      if (nfail <= 40) $display("  FAIL  %s  (%s)  t=%0t", name, detail, $time);
      if (nfail == 1) dump_trace();
    end
  endtask

  // ---------------- APB ----------------
  task automatic wr_e(input logic [7:0] a, input logic [31:0] d, output logic e);
    @(posedge pclk); #0.1; psel = 1; pwrite = 1; paddr = a; pwdata = d; penable = 0;
    @(posedge pclk); #0.1; penable = 1;
    @(posedge pclk); e = pslverr; #0.1; psel = 0; penable = 0; pwrite = 0;
  endtask
  task automatic rd_e(input logic [7:0] a, output logic [31:0] d, output logic e);
    @(posedge pclk); #0.1; psel = 1; pwrite = 0; paddr = a; penable = 0;
    @(posedge pclk); #0.1; penable = 1;
    @(posedge pclk); d = prdata; e = pslverr; #0.1; psel = 0; penable = 0;
  endtask
  task automatic wr(input logic [7:0] a, input logic [31:0] d);
    logic e;
    wr_e(a, d, e);
    check($sformatf("setup write 0x%02h <- 0x%08h accepted", a, d), e === 1'b0, "PSLVERR");
  endtask
  task automatic rd(input logic [7:0] a, output logic [31:0] d);
    logic e;
    rd_e(a, d, e);
  endtask

  task automatic wait_ref(input int n); repeat (n) @(posedge spi_ref_clk); endtask
  task automatic jitter(); repeat ($urandom_range(7)) @(posedge pclk); endtask
  task automatic wait_stat(input logic [31:0] m, input logic [31:0] val);
    logic [31:0] s; int k;
    k = 0;
    do begin rd(STAT, s); k++; end while ((s & m) != val && k < 20000);
    check($sformatf("SPI_STAT & 0x%0h reaches 0x%0h", m, val), (s & m) == val, $sformatf("0x%08h", s));
  endtask
  task automatic settle(); wait_ref(3 * DEPTH + 30); repeat (10) @(posedge pclk); endtask
  // master idle, every TX word popped, every RX word visible on the bus side: twice in a row.
  // (SPI_STAT alone can say BUSY = 0 for a moment after the engine has started.)
  task automatic wait_quiet();
    int ok, k;
    ok = 0; k = 0;
    while (ok < 2 && k < 20000) begin
      wait_ref(8); repeat (4) @(posedge pclk); k++;
      if (dut.u_master.state == 3'd0 && dut.u_tx_fifo.wbin == dut.u_tx_fifo.rbin &&
          dut.u_rx_fifo.rd_wbin == dut.u_rx_fifo.wbin && !dut.busy_sync) ok++;
      else ok = 0;
    end
    check("engine and FIFOs go quiet", ok == 2, "timeout");
    settle();
  endtask

  logic [31:0] fc;                                      // FIFOCTRL thresholds in use
  task automatic set_fc(input int txwm, input int rxwm);
    fc = (32'(rxwm) << 16) | (32'(txwm) << 8);
    wr(FIFOCTRL, fc);
  endtask

  // ---------------- whitebox monitors ----------------
  // effective TX FIFO pops by an engine, and drain steps
  int pops;                                             // 2-state: starts at 0
  always @(posedge spi_ref_clk) if (dut.u_tx_fifo.ren && !dut.u_tx_fifo.rempty) pops++;
  // no level above FIFO_DEPTH, and at most FIFO_DEPTH words between the pointers
  logic [AW:0] span_tx, span_rx;
  int bad_lvl, bad_span;
  always @(posedge pclk) begin
    if (32'(dut.tx_level_bus) > DEPTH || 32'(dut.rx_level_bus) > DEPTH) bad_lvl++;
    span_tx = dut.u_tx_fifo.wbin - dut.u_tx_fifo.rbin;
    span_rx = dut.u_rx_fifo.wbin - dut.u_rx_fifo.rbin;
    if (!dut.rst_busy && !dut.swrst_active && (32'(span_tx) > DEPTH || 32'(span_rx) > DEPTH)) bad_span++;
  end
  // coverage
  int n_txf, n_refl, n_rxf, n_idle, n_slv, n_rst, n_rej, n_take_past;
`ifndef NO_R4_PROBES                                    // R4 FIFO internals (off for older RTL)
  logic refl_q;
  always @(posedge pclk) begin
    refl_q <= dut.u_tx_fifo.w_refl;
    if (dut.u_tx_fifo.w_refl && !refl_q) n_refl++;
  end
  always @(posedge spi_ref_clk)
    if (!dut.u_tx_fifo.r_clr && !dut.u_tx_fifo.r_drain && dut.u_tx_fifo.r_fopen &&
        !(((dut.u_tx_fifo.rbin - dut.u_tx_fifo.w_ftgt) >= (AW+1)'(DEPTH)) || (dut.u_tx_fifo.rbin == dut.u_tx_fifo.w_ftgt)))
      n_take_past++;
`endif

  // +trace: keep the last 600 clock edges of FIFO state, dump them at the first failure
  bit trace_on = 0;
  string tr [0:599];
  int tr_n;
  initial trace_on = $test$plusargs("trace");
`ifndef NO_R4_PROBES
  always @(posedge pclk or posedge spi_ref_clk)
    if (trace_on) begin
      tr[tr_n % 600] = $sformatf("t=%0t %s | RX w%0d r%0d rd_wbin=%0d wr_rbin=%0d wfull=%b rempty=%b drain=%b tgt=%0d lvl=%0d wen=%b ren=%b flush=%b | TX w%0d r%0d | busy=%b st=%0d",
               $time, pclk ? "P" : "R",
               dut.u_rx_fifo.wbin, dut.u_rx_fifo.rbin, dut.u_rx_fifo.rd_wbin, dut.u_rx_fifo.wr_rbin, dut.u_rx_fifo.wfull,
               dut.u_rx_fifo.rempty, dut.u_rx_fifo.r_drain, dut.u_rx_fifo.r_dtgt, dut.u_rx_fifo.rlevel,
               dut.u_rx_fifo.wen, dut.u_rx_fifo.ren, dut.rx_flush, dut.u_tx_fifo.wbin, dut.u_tx_fifo.rbin,
               dut.busy_sync, dut.u_master.state);
      tr_n++;
    end
`endif

  task automatic dump_trace();
    if (trace_on) for (int k = (tr_n > 600 ? tr_n - 600 : 0); k < tr_n; k++) $display("  TR %s", tr[k % 600]);
    trace_on = 0;
  endtask

  // ---------------- steps ----------------
  // queues are module-level (static): Icarus 12 cannot keep them in automatic tasks
  logic [31:0] q [$];                                   // words expected out, in order
  logic [31:0] sent [$];                                // words sent before an RX flush
  logic [7:0]  exp [$];                                 // MISO bytes expected in a slave frame

  // read every word in the RX FIFO and compare with q
  task automatic rx_compare(input string what);
    logic [31:0] lv, w; logic e; int nbad;
    rd(FIFOLVL, lv);
    check($sformatf("%s: RX level = words expected", what), 32'(lv[15:8]) == q.size(),
          $sformatf("RX level %0d, expected %0d", lv[15:8], q.size()));
    nbad = 0;
    foreach (q[i]) begin rd_e(RXD, w, e); if (e || w !== q[i]) nbad++; end
    check($sformatf("%s: the words come out in order, once", what), nbad == 0,
          $sformatf("%0d of %0d wrong", nbad, q.size()));
    rd_e(RXD, w, e);
    check($sformatf("%s: nothing more in the RX FIFO", what), e === 1'b1, $sformatf("0x%08h", w));
  endtask

  task automatic s_txflush();
    logic [31:0] w, lv, st; logic e; int nseg, k, pops0;
    n_txf++;
    wr(CTRL, C_LOOP | D32);                             // master, EN = 0, internal loopback
    q.delete();
    pops0 = pops;
    nseg = 1 + $urandom_range(2);
    for (int s = 0; s < nseg; s++) begin
      k = $urandom_range(DEPTH);
      for (int i = 0; i < k; i++) begin
        w = $urandom; wr_e(TXD, w, e);
        if (!e) q.push_back(w); else n_rej++;
      end
      if ($urandom_range(1)) jitter();
      wr(FIFOCTRL, fc | 32'h1);
      q.delete();
      rd(FIFOLVL, lv); rd(STAT, st);
      check("TX flush: TX level 0 and TX_EMPTY = 1 on the first reads", lv[7:0] == 8'd0 && st[1] === 1'b1,
            $sformatf("FIFOLVL 0x%08h STAT 0x%08h", lv, st));
    end
    k = $urandom_range(DEPTH);
    for (int i = 0; i < k; i++) begin
      w = $urandom; wr_e(TXD, w, e);
      if (!e) q.push_back(w); else n_rej++;
    end
    rd(FIFOLVL, lv);
    check("TX level = words accepted since the flush", 32'(lv[7:0]) == q.size(),
          $sformatf("TX level %0d, expected %0d", lv[7:0], q.size()));
    settle();
    check("EN = 0: nothing popped by the engine", pops == pops0, $sformatf("%0d pops", pops - pops0));
    wr(CTRL, C_EN | C_LOOP | D32);
    wait_quiet();
    check("loopback: engine popped exactly the words written after the flush", pops - pops0 == q.size(),
          $sformatf("%0d pops, expected %0d", pops - pops0, q.size()));
    rx_compare("TX flush");
    wr(CTRL, C_LOOP | D32);
  endtask

  task automatic s_rxflush();
    logic [31:0] w, lv, st, v; logic e; int k, j, nbad, m;
    n_rxf++;
    set_fc(DEPTH / 2, DEPTH);                           // RXWM only at DEPTH words: never here
    wr(CTRL, C_EN | C_LOOP | D32);
    k = 1 + $urandom_range(DEPTH - 2);
    sent.delete();
    for (int i = 0; i < k; i++) begin w = $urandom; wr(TXD, w); sent.push_back(w); end
    wait_quiet();
    j = $urandom_range(k - 1);
    nbad = 0;
    for (int i = 0; i < j; i++) begin rd_e(RXD, w, e); if (e || w !== sent[i]) nbad++; end
    check("RX words before the flush", nbad == 0, $sformatf("%0d of %0d wrong", nbad, j));
    wr(IS, 32'h3F);
    jitter();
    wr(FIFOCTRL, fc | 32'h2);
    nbad = 0;
    for (int i = 0; i < 4; i++) begin
      rd(FIFOLVL, lv); rd(STAT, st);
      if (lv[15:8] !== 8'd0 || st[3] !== 1'b1 || st[6] !== 1'b0) nbad++;
    end
    check("RX flush: RX level 0, RX_EMPTY 1, RX_WM 0 on every read after it", nbad == 0,
          $sformatf("%0d of 4 reads wrong", nbad));
    rd_e(RXD, w, e);
    check("RX flush: RXDATA read right after it gets PSLVERR", e === 1'b1, $sformatf("0x%08h", w));
    rd(IS, v);
    check("RX flush: no RXWM latched", v[1] === 1'b0, $sformatf("SPI_IS 0x%02h", v));
    m = $urandom_range(DEPTH - 1);
    q.delete();
    for (int i = 0; i < m; i++) begin w = $urandom; wr(TXD, w); q.push_back(w); end
    wait_quiet();
    rx_compare("RX flush");
    wr(CTRL, C_LOOP | D32);
  endtask

  task automatic s_txidle();
    logic [31:0] v, lv; int pops0;
    n_idle++;
    wr(CTRL, C_EN | C_LOOP | D32);
    wait_quiet();
    wr(IS, 32'h3F);
    pops0 = pops;
    jitter();
    wr(FIFOCTRL, fc | 32'h1);
    wait_ref(60);
    rd(IS, v); rd(FIFOLVL, lv);
    check("TX flush with EN = 1, TX FIFO empty: no frame", pops == pops0 && v[4] === 1'b0 && lv === 32'h0,
          $sformatf("%0d pops, SPI_IS 0x%02h, FIFOLVL 0x%08h", pops - pops0, v, lv));
    wr(CTRL, C_LOOP | D32);
  endtask

  // the TB is the external master: one frame of n 8-bit words, mode 0, MSB first
  logic [7:0] got [0:3];
  task automatic s_frame(input int n);
    #(2 * HALF); cs_n_i = 1'b0; #(2 * HALF);
    for (int w = 0; w < n; w++)
      for (int i = 7; i >= 0; i--) begin
        mosi_i = 1'($urandom); #(HALF); sclk_i = 1'b1; got[w][i] = miso_o; #(HALF); sclk_i = 1'b0;
      end
    #(2 * HALF); cs_n_i = 1'b1; #(8 * HALF);
    wait_ref(12); repeat (4) @(posedge pclk);
  endtask

  task automatic s_slave();
    logic [31:0] w, lv; logic e; int k, m, nbad;
    n_slv++;
    wr(CTRL, C_MODE | D8); wait_ref(10);
    wr(CTRL, C_EN | C_MODE | D8);
    repeat (10) @(posedge pclk); wait_ref(20);
    k = $urandom_range(DEPTH - 1);                      // 0: nothing staged before the flush
    for (int i = 0; i < k; i++) wr(TXD, 32'($urandom_range(254)));   // never 0xFF
    wait_ref(10);
    jitter();
    wr(FIFOCTRL, fc | 32'h1);
    // half the time the new words follow the flush at once: the slave may stage one of them
    // before the flush request reaches the SPI side, and must then keep it
    if ($urandom_range(1)) wait_ref(4 * DEPTH + 20);
    m = $urandom_range(2);
    exp.delete();
    for (int i = 0; i < m; i++) begin
      w = 32'($urandom_range(254)); wr(TXD, w); exp.push_back(w[7:0]);
    end
    // the external master waits for the flush to finish: the SPI side discards the k flushed
    // words one per spi_ref_clk, so new words reach the slave after about k + 6 spi_ref_clk
    wait_ref(k + 20);
    s_frame(3);
    while (exp.size() < 3) exp.push_back(8'hFF);
    nbad = 0;
    for (int i = 0; i < 3; i++) if (got[i] !== exp[i]) nbad++;
    check("slave: after a TX flush, MISO carries only the words written after it", nbad == 0,
          $sformatf("MISO %02h %02h %02h, expected %02h %02h %02h", got[0], got[1], got[2], exp[0], exp[1], exp[2]));
    // leave: EN = 0, master mode, both FIFOs empty
    wr(CTRL, C_MODE | D8); wait_ref(10);
    wr(CTRL, C_LOOP | D32); wait_ref(10);
    wr(FIFOCTRL, fc | 32'h3); settle();
    rd(FIFOLVL, lv);
    check("slave: FIFOs empty after the step", lv === 32'h0, $sformatf("FIFOLVL 0x%08h", lv));
    wr(IS, 32'h3F);
  endtask

  task automatic s_swrst();
    logic [31:0] w, lv, st, v, c; logic e; int r, k, nbad, npoll;
    n_rst++;
    set_fc(DEPTH / 2, 1 + $urandom_range(DEPTH - 1));
    wr(CTRL, C_EN | C_LOOP | D32);
    r = $urandom_range(DEPTH - 1);
    for (int i = 0; i < r; i++) wr(TXD, $urandom);
    wait_quiet();
    wr(CTRL, C_LOOP | D32);
    k = $urandom_range(DEPTH);
    for (int i = 0; i < k; i++) wr(TXD, $urandom);
    rd(CTRL, c);
    jitter();
    wr(CTRL, c | C_SWRST);
    nbad = 0; npoll = 0;
    do begin
      rd(FIFOLVL, lv); rd(STAT, st); npoll++;
      if (st[11] && lv !== 32'h0) nbad++;
    end while (st[11] && npoll < 2000);
    check("SWRST: FIFO levels read 0 while RST_BUSY = 1", nbad == 0 && !st[11],
          $sformatf("%0d of %0d reads with data, RST_BUSY %b", nbad, npoll, st[11]));
    settle();
    rd(FIFOLVL, lv); rd(STAT, st); rd(IS, v);
    check("SWRST: both FIFOs empty, SPI_IS = TXWM only",
          lv === 32'h0 && st[1] === 1'b1 && st[3] === 1'b1 && v[5:0] === 6'h01,
          $sformatf("FIFOLVL 0x%08h STAT 0x%08h SPI_IS 0x%02h", lv, st, v));
    // the FIFOs work after the reset
    wr(CTRL, C_EN | C_LOOP | D32);
    q.delete();
    k = $urandom_range(DEPTH - 1);
    for (int i = 0; i < k; i++) begin w = $urandom; wr(TXD, w); q.push_back(w); end
    wait_quiet();
    rx_compare("after SWRST");
    wr(CTRL, C_LOOP | D32);
    wr(IS, 32'h3F);
  endtask

  initial begin
    void'($value$plusargs("seed=%d", seed));
    void'($value$plusargs("iters=%d", iters));
    void'($value$plusargs("only=%s", only));
    void'($urandom(seed));
    $display("tb_flush: FIFO_DEPTH %0d, pclk half %0d ps, spi_ref_clk half %0d ps, offset %0d ps, seed %0d, %0d steps",
             DEPTH, hp_ps, hr_ps, ph_ps, seed, iters);
    // reset: start deasserted, then a real edge once the clocks run
    repeat (3) @(posedge spi_ref_clk); presetn = 1'b0; spi_rst_n = 1'b0;
    repeat (4) @(posedge spi_ref_clk); repeat (2) @(posedge pclk); #0.2; presetn = 1'b1; spi_rst_n = 1'b1;
    repeat (6) @(posedge pclk); repeat (6) @(posedge spi_ref_clk);
    wr(CSCTRL, 32'h4); wr(CLKDIV, 32'h0);
    set_fc(DEPTH / 2, DEPTH / 2);
    for (int it = 0; it < iters; it++) begin
      int step;
      wr(CLKDIV, 32'($urandom_range(2)));
      step = $urandom_range(9);
      if (only == "txflush") step = 0;
      if (only == "rxflush") step = 4;
      if (only == "txidle")  step = 6;
      if (only == "slave")   step = 7;
      if (only == "swrst")   step = 9;
      case (step)
        0, 1, 2, 3: s_txflush();
        4, 5:       s_rxflush();
        6:          s_txidle();
        7, 8:       s_slave();
        default:    s_swrst();
      endcase
    end
    check("no FIFO level above FIFO_DEPTH, on any clock", bad_lvl == 0, $sformatf("%0d clocks", bad_lvl));
    check("never more than FIFO_DEPTH words between a FIFO's pointers", bad_span == 0, $sformatf("%0d clocks", bad_span));
    $display("coverage: txflush %0d (relaunch %0d, writes rejected %0d), rxflush %0d, txidle %0d, slave %0d, swrst %0d, reader past target %0d",
             n_txf, n_refl, n_rej, n_rxf, n_idle, n_slv, n_rst, n_take_past);
    $display("RESULT %0d pass %0d fail", npass, nfail);
    $finish;
  end
  initial begin #200_000_000; $display("TIMEOUT"); $display("RESULT %0d pass %0d fail", npass, nfail + 1); $finish; end
endmodule
