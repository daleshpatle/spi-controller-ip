// tb_regs.sv - self-checking register, APB, status and interrupt tests
//   Tracker rows: RG-01..RG-06, FI-01..FI-05, RS-01, MS-09 (through LOOPBACK), PR-01 / PR-02
//   +test=<name>   all (default) | reset | apb | dfs | txfull | rstbusy | bitbash | status |
//                  flush | w1c | irq | swrst
//   +define+DW=<n> +define+DEPTH=<n>   DATA_WIDTH and FIFO_DEPTH of the build (default 32 / 8)
//   +define+DCPOL=1 +define+DCPHA=1     DEF_CPOL / DEF_CPHA of the build (default 0 / 0)
//   (Icarus: -DDW=<n> and so on)
// Every test starts from a hardware reset. Last line: RESULT <n> pass <m> fail
`timescale 1ns/1ps
`ifndef DW
  `define DW 32
`endif
`ifndef DEPTH
  `define DEPTH 8
`endif
`ifndef DCPOL
  `define DCPOL 0
`endif
`ifndef DCPHA
  `define DCPHA 0
`endif
module tb_regs;
  localparam int   DW       = `DW;
  localparam int   DEPTH    = `DEPTH;
  localparam logic DEF_CPOL = 1'(`DCPOL);
  localparam logic DEF_CPHA = 1'(`DCPHA);
  localparam int   DFS_MAX  = (DW < 32) ? DW : 32;      // largest legal DFS
  localparam int   WM0      = DEPTH / 2;                // watermark reset level
  localparam int   HALF     = 100;                      // slave-mode SCLK half period, ns

  // register offsets
  localparam logic [7:0] CTRL = 8'h00, STAT = 8'h04, TXD = 8'h08, RXD = 8'h0C, CLKDIV = 8'h10,
                         IE = 8'h14, IS = 8'h18, CSCTRL = 8'h1C, FIFOCTRL = 8'h20, FIFOLVL = 8'h24;
  // SPI_CTRL fields
  localparam logic [31:0] C_EN = 32'h1, C_MODE = 32'h2, C_CPOL = 32'h4, C_CPHA = 32'h8,
                          C_LSB = 32'h10, C_LOOP = 32'h20, C_TXDMA = 32'h40, C_RXDMA = 32'h80,
                          C_SWRST = 32'h4000;

  logic pclk, spi_ref_clk, presetn, spi_rst_n;
  initial begin pclk = 1'b0; forever #5 pclk = ~pclk; end
  initial begin spi_ref_clk = 1'b0; #0.3; forever #17 spi_ref_clk = ~spi_ref_clk; end

  logic        psel, penable, pwrite;
  logic [7:0]  paddr;
  logic [31:0] pwdata, prdata;
  logic        pready, pslverr;
  logic        sclk_o, sclk_oe, mosi_o, mosi_oe, miso_o, miso_oe, cs_oe, irq, tx_dreq, rx_dreq;
  logic        sclk_i, mosi_i, cs_n_i;
  logic [3:0]  cs_n_o;

  spi_controller_top #(.DATA_WIDTH(DW), .FIFO_DEPTH(DEPTH), .NUM_CS(4), .APB_ADDR_WIDTH(8),
                       .DEF_CPOL(DEF_CPOL), .DEF_CPHA(DEF_CPHA)) dut (
    .pclk(pclk), .presetn(presetn), .spi_ref_clk(spi_ref_clk), .spi_rst_n(spi_rst_n),
    .psel(psel), .penable(penable), .pwrite(pwrite), .paddr(paddr), .pwdata(pwdata),
    .prdata(prdata), .pready(pready), .pslverr(pslverr),
    .sclk_o(sclk_o), .sclk_i(sclk_i), .sclk_oe(sclk_oe), .mosi_o(mosi_o), .mosi_i(mosi_i),
    .mosi_oe(mosi_oe), .miso_o(miso_o), .miso_i(1'b0), .miso_oe(miso_oe),
    .cs_n_o(cs_n_o), .cs_n_i(cs_n_i), .cs_oe(cs_oe),
    .irq(irq), .tx_dreq(tx_dreq), .rx_dreq(rx_dreq));

  // ---------------- checking ----------------
  int npass = 0, nfail = 0, unexp = 0;
  task automatic check(input string name, input bit ok, input string detail);
    if (ok) begin npass++; $display("  PASS  %s  (%s)", name, detail); end
    else    begin nfail++; $display("  FAIL  %s  (%s)", name, detail); end
  endtask
  task automatic info(input string name, input string detail);
    $display("  INFO  %s  (%s)", name, detail);
  endtask

  // APB rules checked on every cycle of every test
  int pready_low, slverr_idle;                          // 2-state: start at 0
  int dreq_busy;                                        // pclk cycles with a DMA request during RST_BUSY
  always @(posedge pclk) begin
    if (psel && penable && pready !== 1'b1)     pready_low  <= pready_low + 1;
    if (!(psel && penable) && pslverr !== 1'b0) slverr_idle <= slverr_idle + 1;
    if (dut.rst_busy && (tx_dreq || rx_dreq))     dreq_busy   <= dreq_busy + 1;
  end

  // ---------------- APB ----------------
  task automatic wr_e(input logic [7:0] a, input logic [31:0] d, output logic e);
    @(posedge pclk); #0.1; psel = 1'b1; pwrite = 1'b1; paddr = a; pwdata = d; penable = 1'b0;
    @(posedge pclk); #0.1; penable = 1'b1;
    @(posedge pclk); e = pslverr; #0.1; psel = 1'b0; penable = 1'b0; pwrite = 1'b0;
  endtask
  task automatic rd_e(input logic [7:0] a, output logic [31:0] d, output logic e);
    @(posedge pclk); #0.1; psel = 1'b1; pwrite = 1'b0; paddr = a; penable = 1'b0;
    @(posedge pclk); #0.1; penable = 1'b1;
    @(posedge pclk); d = prdata; e = pslverr; #0.1; psel = 1'b0; penable = 1'b0;
  endtask
  // helper accesses: an error here is not expected and is counted
  task automatic wr(input logic [7:0] a, input logic [31:0] d);
    logic e;
    wr_e(a, d, e);
    if (e) begin unexp++; $display("  NOTE  unexpected PSLVERR on write 0x%02h <- 0x%08h", a, d); end
  endtask
  task automatic rd(input logic [7:0] a, output logic [31:0] d);
    logic e;
    rd_e(a, d, e);
    if (e) begin unexp++; $display("  NOTE  unexpected PSLVERR on read 0x%02h", a); end
  endtask

  // ---------------- helpers ----------------
  function automatic logic [31:0] dfs(input int n);
    return 32'(n) << 8;
  endfunction
  function automatic logic [31:0] pat(input int k);         // test data word k
    return (32'hC2A9_35F1 ^ (32'(k + 1) * 32'h9E37_79B9));
  endfunction
  function automatic logic [31:0] mask(input int nbits);
    return (nbits >= 32) ? 32'hFFFF_FFFF : ((32'h1 << nbits) - 32'h1);
  endfunction
  function automatic logic irq_exp(input logic [5:0] isv, input logic [5:0] iev,
                                   input logic txdma, input logic rxdma);
    return (|(isv[5:2] & iev[5:2])) | (isv[0] & iev[0] & ~txdma) | (isv[1] & iev[1] & ~rxdma);
  endfunction

  task automatic wait_ref(input int n); repeat (n) @(posedge spi_ref_clk); endtask
  // poll SPI_STAT until (STAT & m) == val
  task automatic wait_stat(input logic [31:0] m, input logic [31:0] val, output bit ok);
    logic [31:0] s; int k;
    k = 0;
    do begin rd(STAT, s); k++; end while ((s & m) != val && k < 5000);
    ok = ((s & m) == val);
  endtask
  task automatic wait_idle();                              // BUSY = 0 and TX_EMPTY = 1
    bit ok;
    wait_ref(8); wait_stat(32'h3, 32'h2, ok);
    if (!ok) $display("  NOTE  wait_idle timed out");
  endtask
  task automatic wait_rx(input int n);                      // RX level = n
    logic [31:0] l; int k;
    k = 0;
    do begin rd(FIFOLVL, l); k++; end while (32'(l[15:8]) != n && k < 5000);
    if (32'(l[15:8]) != n) $display("  NOTE  wait_rx(%0d) timed out at %0d", n, l[15:8]);
  endtask
  task automatic tx_push(input logic [31:0] d);             // write TXDATA once TX_FULL = 0
    bit ok;
    wait_stat(32'h4, 32'h0, ok); wr(TXD, d);
  endtask
  task automatic loop_on(input int n);                      // master, internal loopback, DFS n
    wr(CSCTRL, 32'h4);                                      // CS_AUTO 1, no setup / hold delay
    wr(CLKDIV, 32'h0);
    wr(CTRL, C_EN | C_LOOP | dfs(n));
  endtask
  task automatic slave_on();                                // slave, mode 0, MSB first, DFS 8
    wr(CTRL, C_MODE | dfs(8));                              // MODE changes only with EN = 0
    wait_ref(10);
    wr(CTRL, C_EN | C_MODE | dfs(8));
    repeat (10) @(posedge pclk); wait_ref(20);              // EN reaches the slave between frames
  endtask
  // one slave frame driven by the TB as the external master: nbits of wd, MSB first
  task automatic s_frame(input logic [7:0] wd, input int nbits);
    #(2 * HALF); cs_n_i = 1'b0; #(2 * HALF);
    for (int i = 7; i > 7 - nbits; i--) begin
      mosi_i = wd[i]; #(HALF); sclk_i = 1'b1; #(HALF); sclk_i = 1'b0;
    end
    #(2 * HALF); cs_n_i = 1'b1; #(8 * HALF);
    wait_ref(12); repeat (4) @(posedge pclk);
  endtask
  // latch all six SPI_IS sources: RXOVF, DONE, RXWM from a master loopback burst that
  // overfills the RX FIFO, TXUNF and FRERR from two slave frames, TXWM from the empty TX FIFO.
  // Leaves: slave mode, EN = 1, RX FIFO full, TX FIFO empty.
  task automatic make_all_sources();
    loop_on(8);
    for (int k = 0; k <= DEPTH; k++) tx_push(pat(k));
    wait_idle(); wait_ref(10);
    slave_on();
    s_frame(8'hA5, 8);                                      // nothing staged -> TX underrun
    s_frame(8'h3C, 3);                                      // CS_n high after 3 bits -> FRAME_ERR
  endtask

  logic [31:0] snap_a [0:6];
  logic [31:0] snap_b [0:6];
  // the R/W registers plus SPI_IS and SPI_FIFOLVL
  task automatic snapshot();
    rd(CTRL, snap_a[0]); rd(CLKDIV, snap_a[1]); rd(IE, snap_a[2]); rd(CSCTRL, snap_a[3]);
    rd(FIFOCTRL, snap_a[4]); rd(IS, snap_a[5]); rd(FIFOLVL, snap_a[6]);
  endtask
  task automatic snapshot_b(output bit same);
    rd(CTRL, snap_b[0]); rd(CLKDIV, snap_b[1]); rd(IE, snap_b[2]); rd(CSCTRL, snap_b[3]);
    rd(FIFOCTRL, snap_b[4]); rd(IS, snap_b[5]); rd(FIFOLVL, snap_b[6]);
    same = 1'b1;
    for (int i = 0; i < 7; i++) if (snap_a[i] !== snap_b[i]) same = 1'b0;
  endtask

  task automatic reset_dut();
    cs_n_i = 1'b1; sclk_i = 1'b0; mosi_i = 1'b0;
    psel = 1'b0; penable = 1'b0; pwrite = 1'b0;
    repeat (3) @(posedge spi_ref_clk); presetn = 1'b0; spi_rst_n = 1'b0;
    repeat (4) @(posedge spi_ref_clk); repeat (2) @(posedge pclk); #0.2; presetn = 1'b1; spi_rst_n = 1'b1;
    repeat (6) @(posedge pclk); repeat (6) @(posedge spi_ref_clk);
  endtask

  // ================= RG-01: reset values =================
  task automatic t_reset();
    logic [31:0] v, ctrl0, fc0, r1 [0:7], r2 [0:7]; logic e; bit same;
    $display("[reset] register values after a hardware reset (spec 6.10)");
    ctrl0 = dfs(8) | (DEF_CPOL ? C_CPOL : 32'h0) | (DEF_CPHA ? C_CPHA : 32'h0);
    fc0   = (32'(WM0) << 16) | (32'(WM0) << 8);
    rd(CTRL, v);     check("SPI_CTRL", v === ctrl0, $sformatf("0x%08h, expected 0x%08h", v, ctrl0));
    rd(STAT, v);     check("SPI_STAT: TX_EMPTY, RX_EMPTY, TX_WM", v === 32'h2A, $sformatf("0x%08h", v));
    rd(CLKDIV, v);   check("SPI_CLKDIV", v === 32'hFF, $sformatf("0x%08h", v));
    rd(IE, v);       check("SPI_IE", v === 32'h0, $sformatf("0x%08h", v));
    rd(IS, v);       check("SPI_IS = TXWM only (level source: empty TX FIFO <= TX_WM_LVL)",
                           v === 32'h1, $sformatf("0x%08h", v));
    rd(CSCTRL, v);   check("SPI_CSCTRL", v === 32'h7FFC, $sformatf("0x%08h", v));
    rd(FIFOCTRL, v); check("SPI_FIFOCTRL: TX_WM_LVL = RX_WM_LVL = FIFO_DEPTH/2", v === fc0,
                           $sformatf("0x%08h, expected 0x%08h", v, fc0));
    rd(FIFOLVL, v);  check("SPI_FIFOLVL", v === 32'h0, $sformatf("0x%08h", v));
    rd_e(RXD, v, e); check("SPI_RXDATA read with the RX FIFO empty: PSLVERR, PRDATA = 0",
                           e === 1'b1 && v === 32'h0, $sformatf("PSLVERR %b, PRDATA 0x%08h", e, v));
    check("irq, tx_dreq, rx_dreq low", {irq, tx_dreq, rx_dreq} === 3'b000,
          $sformatf("irq %b tx_dreq %b rx_dreq %b", irq, tx_dreq, rx_dreq));
    check("SCLK parked at DEF_CPOL, pin not driven", sclk_o === DEF_CPOL && sclk_oe === 1'b0,
          $sformatf("sclk_o %b sclk_oe %b", sclk_o, sclk_oe));
    // reading has no side effect: two passes over every readable register agree
    for (int p = 0; p < 2; p++) begin
      rd(CTRL, v);     if (p == 0) r1[0] = v; else r2[0] = v;
      rd(STAT, v);     if (p == 0) r1[1] = v; else r2[1] = v;
      rd(CLKDIV, v);   if (p == 0) r1[2] = v; else r2[2] = v;
      rd(IE, v);       if (p == 0) r1[3] = v; else r2[3] = v;
      rd(IS, v);       if (p == 0) r1[4] = v; else r2[4] = v;
      rd(CSCTRL, v);   if (p == 0) r1[5] = v; else r2[5] = v;
      rd(FIFOCTRL, v); if (p == 0) r1[6] = v; else r2[6] = v;
      rd(FIFOLVL, v);  if (p == 0) r1[7] = v; else r2[7] = v;
    end
    same = 1'b1;
    for (int i = 0; i < 8; i++) if (r1[i] !== r2[i]) same = 1'b0;
    check("reads have no side effect (two passes over 8 registers agree)", same, "");
  endtask

  // ================= RG-02 / RG-03: APB protocol, decode, access types =================
  task automatic t_apb();
    logic [31:0] v; logic e, s1, s2, s3; int nbad, nadr; bit same;
    $display("[apb] APB protocol, address decode and access types (spec 7.6)");
    // PSLVERR is driven only in the ACCESS phase
    @(posedge pclk); #0.1; psel = 1'b1; pwrite = 1'b0; paddr = 8'h40; penable = 1'b0;
    #1; s1 = pslverr;
    @(posedge pclk); #0.1; penable = 1'b1;
    #1; s2 = pslverr;
    @(posedge pclk); #0.1; psel = 1'b0; penable = 1'b0;
    #1; s3 = pslverr;
    check("unmapped read: PSLVERR low in SETUP, high in ACCESS, low after",
          s1 === 1'b0 && s2 === 1'b1 && s3 === 1'b0, $sformatf("%b %b %b", s1, s2, s3));
    snapshot();
    // every unmapped word offset, read and write
    nbad = 0; nadr = 0;
    for (int a = 8'h28; a <= 8'hFC; a += 4) begin
      rd_e(8'(a), v, e);              if (e !== 1'b1 || v !== 32'h0) nbad++;
      wr_e(8'(a), 32'hFFFF_FFFF, e);  if (e !== 1'b1) nbad++;
      nadr++;
    end
    check("unmapped word offsets 0x28..0xFC: read and write give PSLVERR, PRDATA = 0", nbad == 0,
          $sformatf("%0d offsets, %0d wrong responses", nadr, nbad));
    // offsets that are not word aligned are not registers either
    nbad = 0;
    for (int i = 0; i < 10; i++) begin
      logic [7:0] a;
      case (i)
        0: a = 8'h01; 1: a = 8'h02; 2: a = 8'h03; 3: a = 8'h05; 4: a = 8'h0A;
        5: a = 8'h0F; 6: a = 8'h13; 7: a = 8'h1E; 8: a = 8'h21; default: a = 8'h27;
      endcase
      rd_e(a, v, e);              if (e !== 1'b1 || v !== 32'h0) nbad++;
      wr_e(a, 32'hFFFF_FFFF, e);  if (e !== 1'b1) nbad++;
    end
    check("10 unaligned offsets: read and write give PSLVERR", nbad == 0,
          $sformatf("%0d wrong responses", nbad));
    snapshot_b(same);
    check("rejected writes to unmapped offsets changed no register", same, "");
    // writes to read-only registers
    wr_e(STAT, 32'hFFFF_FFFF, e);    s1 = e;
    wr_e(RXD, 32'hA5A5_A5A5, e);     s2 = e;
    wr_e(FIFOLVL, 32'hFFFF_FFFF, e); s3 = e;
    check("write to SPI_STAT, SPI_RXDATA, SPI_FIFOLVL: PSLVERR", {s1, s2, s3} === 3'b111,
          $sformatf("%b %b %b", s1, s2, s3));
    rd(STAT, v);
    check("  nothing changed: STAT still 0x2A (RX_EMPTY, nothing pushed)", v === 32'h2A,
          $sformatf("STAT 0x%08h", v));
    rd(FIFOLVL, v); check("  FIFO levels still 0", v === 32'h0, $sformatf("FIFOLVL 0x%08h", v));
    // read of the write-only data register
    wr(TXD, 32'h5A);
    rd_e(TXD, v, e);
    check("read of SPI_TXDATA: PSLVERR, PRDATA = 0", e === 1'b1 && v === 32'h0,
          $sformatf("PSLVERR %b, PRDATA 0x%08h", e, v));
    rd(FIFOLVL, v); check("  TX FIFO not popped by the read", v[7:0] === 8'd1, $sformatf("TX level %0d", v[7:0]));
    // the registers themselves accept reads and writes without PSLVERR
    nbad = 0;
    for (int i = 0; i < 7; i++) begin
      logic [7:0] a;
      case (i)
        0: a = CTRL; 1: a = STAT; 2: a = CLKDIV; 3: a = IE; 4: a = IS; 5: a = CSCTRL; default: a = FIFOCTRL;
      endcase
      rd_e(a, v, e); if (e !== 1'b0) nbad++;
      if (a != STAT) begin wr_e(a, v, e); if (e !== 1'b0) nbad++; end    // write back what was read
    end
    rd_e(FIFOLVL, v, e); if (e !== 1'b0) nbad++;
    check("reads of all readable registers and write-backs of the R/W ones: no PSLVERR", nbad == 0,
          $sformatf("%0d errors", nbad));
  endtask

  // ================= RG-03: DFS range, every value =================
  task automatic t_dfs();
    logic [31:0] v, prev, wv; logic e; int nlegal, nill, bad_legal, bad_ill;
    $display("[dfs] SPI_CTRL.DFS 0..63: legal 4..%0d, the rest rejected (spec 6.2)", DFS_MAX);
    nlegal = 0; nill = 0; bad_legal = 0; bad_ill = 0;
    wr(CTRL, C_CPOL | C_LSB | dfs(8));
    for (int n = 0; n < 64; n++) begin
      rd(CTRL, prev);
      // each write also tries to change other CTRL fields, so a rejected write must leave them too
      wv = ((n % 2) ? (C_CPHA | C_LOOP | C_TXDMA) : (C_CPOL | C_LSB | C_RXDMA)) | dfs(n);
      wr_e(CTRL, wv, e); rd(CTRL, v);
      if (n >= 4 && n <= DFS_MAX) begin nlegal++; if (e !== 1'b0 || v !== wv)   bad_legal++; end
      else                        begin nill++;   if (e !== 1'b1 || v !== prev) bad_ill++;   end
    end
    check($sformatf("DFS 4..%0d accepted and read back with the other fields", DFS_MAX), bad_legal == 0,
          $sformatf("%0d values, %0d wrong", nlegal, bad_legal));
    check($sformatf("DFS 0..3 and %0d..63 rejected: PSLVERR, no CTRL field changed", DFS_MAX + 1),
          bad_ill == 0, $sformatf("%0d values, %0d wrong", nill, bad_ill));
    wr(CTRL, dfs(8));
  endtask

  // ================= RG-03: TX full, RX empty =================
  task automatic t_txfull();
    logic [31:0] v; logic e; int nerr, nbad;
    $display("[txfull] TXDATA write with the TX FIFO full, RXDATA read with the RX FIFO empty");
    wr(CTRL, dfs(DFS_MAX));                                   // EN = 0: nothing is sent yet
    nerr = 0;
    for (int k = 0; k < DEPTH; k++) begin wr_e(TXD, pat(k), e); if (e !== 1'b0) nerr++; end
    rd(STAT, v); rd(FIFOLVL, snap_b[0]);
    check($sformatf("%0d writes fill the TX FIFO: no PSLVERR, TX_FULL = 1, TX level %0d", DEPTH, DEPTH),
          nerr == 0 && v[2] === 1'b1 && 32'(snap_b[0][7:0]) == DEPTH,
          $sformatf("%0d errors, STAT 0x%08h, TX level %0d", nerr, v, snap_b[0][7:0]));
    wr_e(TXD, 32'hDEAD_BEEF, e); rd(FIFOLVL, v);
    check("one more write: PSLVERR, TX level unchanged", e === 1'b1 && 32'(v[7:0]) == DEPTH,
          $sformatf("PSLVERR %b, TX level %0d", e, v[7:0]));
    // send everything through the internal loopback: the FIFO must hold exactly the first DEPTH words
    wr(CSCTRL, 32'h4); wr(CLKDIV, 32'h0); wr(CTRL, C_EN | C_LOOP | dfs(DFS_MAX));
    wait_idle(); wait_rx(DEPTH);
    nbad = 0;
    for (int k = 0; k < DEPTH; k++) begin rd(RXD, v); if (v !== (pat(k) & mask(DFS_MAX))) nbad++; end
    check("the FIFO kept the first words in order; the rejected word was not stored", nbad == 0,
          $sformatf("%0d of %0d words wrong", nbad, DEPTH));
    nbad = 0;
    for (int k = 0; k < 3; k++) begin rd_e(RXD, v, e); if (e !== 1'b1 || v !== 32'h0) nbad++; end
    check("3 reads with the RX FIFO empty: PSLVERR, PRDATA = 0", nbad == 0, $sformatf("%0d wrong", nbad));
    tx_push(pat(40)); wait_idle(); wait_rx(1);
    rd_e(RXD, v, e);
    check("  the empty reads did not move the read pointer: next word reads correctly",
          e === 1'b0 && v === (pat(40) & mask(DFS_MAX)), $sformatf("PSLVERR %b, 0x%08h", e, v));
    wr(CTRL, dfs(8));
  endtask

  // ================= RG-03: RST_BUSY rules =================
  task automatic t_rstbusy();
    logic [31:0] v, c, st1, st2, lv; logic e, e1, e2, e3, e4; bit ok;
    $display("[rstbusy] accesses while SPI_STAT.RST_BUSY = 1 (spec 7.5 step 1, 7.6)");
    rd(CTRL, c);
    wr_e(CTRL, c | C_SWRST, e);
    rd(STAT, st1);
    wr_e(TXD, 32'h5A, e1);
    rd_e(RXD, v, e2);
    wr_e(CLKDIV, 32'h33, e3);
    wr_e(CTRL, c, e4);
    rd(STAT, st2);
    check("SWRST written with the current CTRL value: accepted, RST_BUSY = 1",
          e === 1'b0 && st1[11] === 1'b1, $sformatf("PSLVERR %b, STAT 0x%08h", e, st1));
    check("  SPI_TXDATA write while RST_BUSY: PSLVERR", e1 === 1'b1, $sformatf("PSLVERR %b", e1));
    check("  SPI_RXDATA read while RST_BUSY: PSLVERR, PRDATA = 0 (the FIFO is also empty here)",
          e2 === 1'b1 && v === 32'h0, $sformatf("PSLVERR %b, PRDATA 0x%08h", e2, v));
    check("  SPI_CLKDIV and SPI_CTRL stay writable while RST_BUSY", e3 === 1'b0 && e4 === 1'b0,
          $sformatf("PSLVERR %b %b", e3, e4));
    check("  RST_BUSY was still 1 after those accesses", st2[11] === 1'b1, $sformatf("STAT 0x%08h", st2));
    wait_stat(32'h800, 32'h0, ok);
    rd(FIFOLVL, lv); rd(CLKDIV, v);
    check("after RST_BUSY = 0: the rejected TXDATA word was not stored, the CLKDIV write was kept",
          ok && lv === 32'h0 && v === 32'h33, $sformatf("FIFOLVL 0x%08h, CLKDIV 0x%02h", lv, v));
    wr_e(TXD, 32'h5A, e); rd(FIFOLVL, lv);
    check("  TXDATA writes work again", e === 1'b0 && lv[7:0] === 8'd1, $sformatf("PSLVERR %b, TX level %0d", e, lv[7:0]));
    // a write that carries an illegal field is rejected as a whole (spec 7.6 note)
    rd(FIFOCTRL, c);
    wr_e(FIFOCTRL, 32'h1, e); rd(FIFOLVL, lv); rd(FIFOCTRL, v);
    check("bare TX flush (FIFOCTRL = 0x1, so RX_WM_LVL = 0): PSLVERR, no flush, no change",
          e === 1'b1 && lv[7:0] === 8'd1 && v === c, $sformatf("PSLVERR %b, TX level %0d", e, lv[7:0]));
    wr_e(FIFOCTRL, c | 32'h1, e); rd(FIFOLVL, lv);
    check("  flush written with the current thresholds: accepted, TX FIFO empty",
          e === 1'b0 && lv[7:0] === 8'd0, $sformatf("PSLVERR %b, TX level %0d", e, lv[7:0]));
    rd(CTRL, c);
    wr_e(CTRL, C_SWRST, e); rd(STAT, st1); rd(CTRL, v);
    info("bare SWRST write 0x00004000 (DFS field 0)",
         $sformatf("PSLVERR %b, RST_BUSY %b, CTRL 0x%08h -> 0x%08h: %s", e, st1[11], c, v,
                   (e === 1'b1 && st1[11] === 1'b0) ? "rejected with the DFS, no reset" : "reset started"));
    wait_stat(32'h800, 32'h0, ok);
  endtask

  // ================= RG-04: bit-bash =================
  task automatic t_bitbash();
    logic [31:0] v, wv, exp, bg; logic e; int nbad, nw, nlegal, nill; bit ok;
    $display("[bitbash] every R/W field; reserved bits read 0 and ignore writes (spec 6)");
    // SPI_CTRL bits 7:0, DFS 8, reserved 31:15 written as 1
    nbad = 0; nw = 0;
    for (int p = 0; p < 18; p++) begin
      logic [7:0] b;
      if (p == 0) b = 8'h00; else if (p == 1) b = 8'hFF;
      else if (p < 10) b = 8'(1 << (p - 2)); else b = ~8'(1 << (p - 10));
      wr(CTRL, dfs(8));                                       // EN = 0 before each pattern
      wv = 32'(b) | dfs(8);
      wr_e(CTRL, wv | 32'hFFFF_8000, e); rd(CTRL, v); nw++;
      if (e !== 1'b0 || v !== wv) begin nbad++; $display("    CTRL wrote 0x%08h read 0x%08h", wv, v); end
    end
    wr(CTRL, dfs(8));
    check("SPI_CTRL bits 7:0: 0s, 1s, walking 1, walking 0 read back; bits 31:15 read 0", nbad == 0,
          $sformatf("%0d patterns, %0d wrong", nw, nbad));
    rd(CTRL, bg);
    wr_e(CTRL, bg | C_SWRST, e); rd(CTRL, v);
    check("SPI_CTRL.SWRST reads back 0", e === 1'b0 && v === bg, $sformatf("0x%08h", v));
    wait_stat(32'h800, 32'h0, ok);
    // SPI_CLKDIV: all 256 values, bits 31:8 written as 1
    nbad = 0;
    for (int n = 0; n < 256; n++) begin
      wr_e(CLKDIV, 32'(n) | 32'hFFFF_FF00, e); rd(CLKDIV, v);
      if (e !== 1'b0 || v !== 32'(n)) nbad++;
    end
    check("SPI_CLKDIV: all 256 values read back, bits 31:8 read 0", nbad == 0, $sformatf("%0d wrong", nbad));
    wr(CLKDIV, 32'hFF);
    // SPI_IE: all 64 values, bits 31:6 written as 1
    nbad = 0;
    for (int n = 0; n < 64; n++) begin
      wr_e(IE, 32'(n) | 32'hFFFF_FFC0, e); rd(IE, v);
      if (e !== 1'b0 || v !== 32'(n)) nbad++;
    end
    check("SPI_IE: all 64 values read back, bits 31:6 read 0", nbad == 0, $sformatf("%0d wrong", nbad));
    wr(IE, 32'h0);
    // SPI_CSCTRL: every value of each field, the others at their reset value, bits 31:15 as 1
    bg = 32'h7FFC; nbad = 0; nw = 0;
    for (int f = 0; f < 4; f++) begin
      int lsb, w;
      case (f) 0: begin lsb = 0; w = 2; end 1: begin lsb = 2; w = 1; end
               2: begin lsb = 3; w = 6; end default: begin lsb = 9; w = 6; end endcase
      for (int n = 0; n < (1 << w); n++) begin
        exp = (bg & ~(mask(w) << lsb)) | (32'(n) << lsb);
        wr_e(CSCTRL, exp | 32'hFFFF_8000, e); rd(CSCTRL, v); nw++;
        if (e !== 1'b0 || v !== exp) nbad++;
      end
    end
    check("SPI_CSCTRL: every CS_SEL, CS_AUTO, CS_SETUP, CS_HOLD value; bits 31:15 read 0", nbad == 0,
          $sformatf("%0d writes, %0d wrong", nw, nbad));
    wr(CSCTRL, 32'h7FFC);
    // SPI_FIFOCTRL: TX_WM_LVL 0..255 and RX_WM_LVL 0..255, reserved bits 7:2 and 31:24 as 1
    nbad = 0; nlegal = 0; nill = 0;
    for (int side = 0; side < 2; side++)
      for (int n = 0; n < 256; n++) begin
        logic [31:0] prev; bit legal;
        rd(FIFOCTRL, prev);
        if (side == 0) begin exp = (32'(WM0) << 16) | (32'(n) << 8);  legal = (n <= DEPTH);            end
        else           begin exp = (32'(n) << 16) | (32'(WM0) << 8);  legal = (n >= 1 && n <= DEPTH); end
        wr_e(FIFOCTRL, exp | 32'hFF00_00FC, e); rd(FIFOCTRL, v);
        if (legal) begin nlegal++; if (e !== 1'b0 || v !== exp)  nbad++; end
        else       begin nill++;   if (e !== 1'b1 || v !== prev) nbad++; end
      end
    check($sformatf("SPI_FIFOCTRL: TX_WM_LVL 0..%0d and RX_WM_LVL 1..%0d read back, other values PSLVERR and no change",
                    DEPTH, DEPTH), nbad == 0, $sformatf("%0d legal, %0d illegal, %0d wrong", nlegal, nill, nbad));
    exp = (32'(WM0) << 16) | (32'(WM0) << 8);
    wr_e(FIFOCTRL, exp | 32'h3, e); rd(FIFOCTRL, v);
    check("  flush bits 1:0 read back 0", e === 1'b0 && v === exp, $sformatf("0x%08h", v));
  endtask

  // ================= RG-05 / FI-01 / FI-03: SPI_STAT live bits, FIFO levels, flush =================
  task automatic t_status();
    logic [31:0] v, st, lv, fc; logic e; int nbad; bit ok, seen;
    $display("[status] SPI_STAT and SPI_FIFOLVL while the FIFOs fill and drain (spec 6.3, 6.9)");
    // TX side, EN = 0
    nbad = 0;
    for (int n = 1; n <= DEPTH; n++) begin
      wr(TXD, pat(n));
      rd(FIFOLVL, lv); rd(STAT, st);
      if (32'(lv[7:0]) != n || st[1] !== 1'b0 || st[2] !== (n == DEPTH) || st[5] !== (n <= WM0)) begin
        nbad++; $display("    TX level %0d: FIFOLVL 0x%08h STAT 0x%08h", n, lv, st);
      end
    end
    check($sformatf("TX writes 1..%0d: TX level, TX_EMPTY, TX_FULL, TX_WM follow every write", DEPTH),
          nbad == 0, $sformatf("%0d wrong", nbad));
    rd(FIFOCTRL, fc); wr(FIFOCTRL, fc | 32'h1); rd(FIFOLVL, lv); rd(STAT, st);
    check("TX_FLUSH at idle: TX level 0, TX_EMPTY = 1, TX_WM = 1", lv[7:0] === 8'd0 && st[1] === 1'b1 && st[5] === 1'b1,
          $sformatf("FIFOLVL 0x%08h STAT 0x%08h", lv, st));
    // RX side through the internal loopback
    loop_on(8); nbad = 0;
    for (int n = 1; n <= DEPTH; n++) begin
      tx_push(pat(n)); wait_rx(n); rd(STAT, st);
      if (st[3] !== 1'b0 || st[4] !== (n == DEPTH) || st[6] !== (n >= WM0)) begin
        nbad++; $display("    RX level %0d: STAT 0x%08h", n, st);
      end
    end
    check($sformatf("RX words 1..%0d: RX level, RX_EMPTY, RX_FULL, RX_WM follow every word", DEPTH),
          nbad == 0, $sformatf("%0d wrong", nbad));
    // RX overrun: one more word with the RX FIFO full (MS-09 through the loopback)
    tx_push(pat(99)); wait_idle(); wait_ref(10);
    rd(STAT, st); rd(IS, v); rd(FIFOLVL, lv);
    check("word with the RX FIFO full: RX_OVERRUN and RXOVF set, RX level stays FIFO_DEPTH",
          st[8] === 1'b1 && v[3] === 1'b1 && 32'(lv[15:8]) == DEPTH, $sformatf("STAT 0x%08h IS 0x%02h RX level %0d", st, v, lv[15:8]));
    nbad = 0;
    for (int n = 1; n <= DEPTH; n++) begin rd(RXD, v); if (v !== (pat(n) & 32'hFF)) nbad++; end
    rd(STAT, st);
    check("  the first words are intact and the extra word was dropped", nbad == 0 && st[3] === 1'b1,
          $sformatf("%0d wrong, RX_EMPTY %b", nbad, st[3]));
    // BUSY during a slow burst, and a flush written while BUSY is ignored
    wr(CLKDIV, 32'hF); wr(CTRL, C_EN | C_LOOP | dfs(DFS_MAX));
    for (int k = 0; k < 3; k++) wr(TXD, pat(k));
    seen = 1'b0;
    for (int k = 0; k < 400 && !seen; k++) begin rd(STAT, st); if (st[0]) seen = 1'b1; end
    wait_ref(10);                                             // first word popped, level settled
    rd(FIFOLVL, lv); rd(FIFOCTRL, fc);
    wr_e(FIFOCTRL, fc | 32'h1, e); rd(FIFOLVL, v);
    check("BUSY = 1 during a burst", seen, "");
    check("TX_FLUSH written while BUSY: no PSLVERR, ignored (TX level unchanged)",
          e === 1'b0 && v[7:0] === lv[7:0] && lv[7:0] !== 8'd0, $sformatf("TX level %0d -> %0d", lv[7:0], v[7:0]));
    wait_idle(); rd(STAT, st);
    check("BUSY = 0 after the burst", st[0] === 1'b0, $sformatf("STAT 0x%08h", st));
    rd(FIFOCTRL, fc); wr(FIFOCTRL, fc | 32'h2); wait_ref(20); // drop the burst's RX words
    // TX_STALL: manual chip-select runs out of TX data
    wr(CLKDIV, 32'h0); wr(CSCTRL, 32'h0); wr(CTRL, C_EN | C_LOOP | dfs(8));
    wr(TXD, pat(5));
    wait_stat(32'h401, 32'h401, ok); rd(STAT, st);
    check("CS_AUTO = 0, TX empty: TX_STALL = 1, BUSY = 1", ok, $sformatf("STAT 0x%08h", st));
    wr(CTRL, C_LOOP | dfs(8));                               // EN = 0 ends the STALL
    wait_stat(32'h401, 32'h0, ok); rd(STAT, st);
    check("  EN = 0: TX_STALL = 0, BUSY = 0", ok, $sformatf("STAT 0x%08h", st));
    wr(CSCTRL, 32'h7FFC); rd(FIFOCTRL, fc); wr(FIFOCTRL, fc | 32'h2); wait_ref(20);
  endtask

  // ================= FI-03: flush at idle once the FIFO pointers have moved =================
  // A flush (and SWRST) clears the bus side of a FIFO at once and the SPI side a few spi_ref_clk
  // cycles later. In between, neither side may act on the other side's old pointer.
  // Each part starts from a hardware reset and first moves both pointers to FIFO_DEPTH.
  task automatic flush_prep();
    logic [31:0] v;
    reset_dut();
    wr(FIFOCTRL, (32'(DEPTH) << 16) | (32'(WM0) << 8));      // RX_WM_LVL = DEPTH: 2 words do not set RXWM
    loop_on(8);
    for (int i = 0; i < DEPTH; i++) begin tx_push(pat(i)); wait_idle(); wait_rx(1); rd(RXD, v); end
  endtask
  task automatic t_flush();
    logic [31:0] v, fc, lv, st; logic e1, e2; int nbad, k; bit ok;
    $display("[flush] TX_FLUSH and RX_FLUSH at idle after traffic (spec 7.5, 6.8)");
    // TX: one word queued with EN = 0, flush, then watch the bus side
    flush_prep();
    wr(CTRL, C_LOOP | dfs(8)); wr(TXD, pat(20));
    rd(FIFOCTRL, fc); wr(FIFOCTRL, fc | 32'h1);
    nbad = 0;
    for (int i = 0; i < 6; i++) begin
      rd(FIFOLVL, lv); rd(STAT, st);
      if (lv[7:0] !== 8'd0 || st[1] !== 1'b1 || st[2] !== 1'b0) nbad++;
    end
    wr_e(TXD, pat(21), e1); wr_e(TXD, pat(22), e2); rd(FIFOLVL, lv);
    check("TX_FLUSH after traffic: TX level 0, TX_EMPTY = 1, TX_FULL = 0 on every read after it",
          nbad == 0, $sformatf("%0d of 6 reads wrong", nbad));
    check("  TXDATA writes afterwards are accepted and counted", e1 === 1'b0 && e2 === 1'b0 && lv[7:0] === 8'd2,
          $sformatf("PSLVERR %b %b, TX level %0d", e1, e2, lv[7:0]));
    // RX: two words waiting, flush, then watch the bus side
    flush_prep();
    tx_push(pat(30)); tx_push(pat(31)); wait_idle(); wait_rx(2);
    wr(IS, 32'h3F);
    rd(FIFOCTRL, fc); wr(FIFOCTRL, fc | 32'h2);
    nbad = 0;
    for (int i = 0; i < 6; i++) begin
      rd(FIFOLVL, lv); rd(STAT, st);
      if (lv[15:8] !== 8'd0 || st[3] !== 1'b1 || st[6] !== 1'b0) nbad++;
    end
    rd(IS, v);
    check("RX_FLUSH after traffic: RX level 0, RX_EMPTY = 1, RX_WM = 0 on every read after it", nbad == 0,
          $sformatf("%0d of 6 reads showed data", nbad));
    check("  RX_FLUSH latches no RXWM", v[1] === 1'b0, $sformatf("SPI_IS 0x%02h", v));
    // RX again: read RXDATA as soon as the FIFO looks non-empty, then check it is still consistent
    flush_prep();
    tx_push(pat(32)); tx_push(pat(33)); wait_idle(); wait_rx(2);
    rd(FIFOCTRL, fc); wr(FIFOCTRL, fc | 32'h2);
    k = 0;
    do begin rd(STAT, st); k++; end while (st[3] === 1'b1 && k < 6);
    if (st[3] === 1'b0) rd_e(RXD, v, e1); else e1 = 1'b1;
    wait_ref(40); rd(FIFOLVL, lv); rd(STAT, st);
    check("  RXDATA read right after the flush returns nothing, and the FIFO stays empty",
          e1 === 1'b1 && lv[15:8] === 8'd0 && st[3] === 1'b1,
          $sformatf("read PSLVERR %b; 1.4 us later RX level %0d, RX_EMPTY %b", e1, lv[15:8], st[3]));
    // TX_FLUSH with EN = 1 and the TX FIFO empty: nothing may be sent
    flush_prep();
    wr(IS, 32'h3F);
    rd(FIFOCTRL, fc); wr(FIFOCTRL, fc | 32'h1);
    wait_ref(100); rd(IS, v); rd(FIFOLVL, lv);
    check("TX_FLUSH with EN = 1 and the TX FIFO empty: no frame starts, nothing looped back",
          v[4] === 1'b0 && lv === 32'h0, $sformatf("SPI_IS 0x%02h FIFOLVL 0x%08h", v, lv));
    wait_stat(32'h3, 32'h2, ok);
  endtask

  // the IS clearing sequence shared by t_w1c and t_irq: after step s, SPI_IS = is_after(s)
  function automatic logic [5:0] is_after(input int s);
    case (s)
      0: return 6'h3F; 1: return 6'h1F; 2: return 6'h0F; 3: return 6'h07;
      4: return 6'h03; 5: return 6'h01; default: return 6'h00;
    endcase
  endfunction
  task automatic w1c_step(input int s);
    logic [31:0] v;
    case (s)
      1: wr(IS, 32'h20);                                     // FRERR
      2: wr(IS, 32'h10);                                     // DONE
      3: wr(IS, 32'h08);                                     // RXOVF
      4: wr(IS, 32'h04);                                     // TXUNF
      5: begin                                               // RXWM: drain the RX FIFO first
           for (int k = 0; k < DEPTH; k++) rd(RXD, v);
           wr(IS, 32'h02);
         end
      6: begin                                               // TXWM: fill the TX FIFO first
           wr(CTRL, C_MODE | dfs(8)); wait_ref(10);           // EN = 0: the slave stages nothing
           for (int k = 0; k < DEPTH; k++) wr(TXD, pat(k));
           wr(IS, 32'h01);
         end
      default: ;
    endcase
  endtask

  // ================= RG-06 / RG-05: SPI_IS write-1-to-clear, sticky STAT flags =================
  task automatic t_w1c();
    logic [31:0] v, st; logic e;
    $display("[w1c] SPI_IS write-1-to-clear one bit at a time (spec 6.3, 6.6)");
    make_all_sources();
    rd(IS, v); rd(STAT, st);
    check("all six sources latched", v === 32'h3F, $sformatf("SPI_IS 0x%02h", v));
    check("  STAT sticky TX_UNDERRUN, RX_OVERRUN, FRAME_ERR = 1", st[9:7] === 3'b111, $sformatf("STAT 0x%08h", st));
    rd(IS, v); check("reading SPI_IS does not clear it", v === 32'h3F, $sformatf("0x%02h", v));
    wr(IS, 32'h0); rd(IS, v); check("writing 0 clears nothing", v === 32'h3F, $sformatf("0x%02h", v));
    wr_e(IS, 32'hFFFF_FFC0, e); rd(IS, v);
    check("writing 1 to reserved bits 31:6: no PSLVERR, clears nothing", e === 1'b0 && v === 32'h3F,
          $sformatf("PSLVERR %b, 0x%02h", e, v));
    w1c_step(1); rd(IS, v); rd(STAT, st);
    check("W1C FRERR: only bit 5 clears, and STAT.FRAME_ERR with it", v[5:0] === 6'h1F && st[9:7] === 3'b011,
          $sformatf("SPI_IS 0x%02h STAT 0x%08h", v, st));
    w1c_step(2); rd(IS, v); rd(STAT, st);
    check("W1C DONE: only bit 4 clears", v[5:0] === 6'h0F && st[9:7] === 3'b011, $sformatf("SPI_IS 0x%02h", v));
    w1c_step(3); rd(IS, v); rd(STAT, st);
    check("W1C RXOVF: only bit 3 clears, and STAT.RX_OVERRUN with it", v[5:0] === 6'h07 && st[9:7] === 3'b001,
          $sformatf("SPI_IS 0x%02h STAT 0x%08h", v, st));
    w1c_step(4); rd(IS, v); rd(STAT, st);
    check("W1C TXUNF: only bit 2 clears, and STAT.TX_UNDERRUN with it", v[5:0] === 6'h03 && st[9:7] === 3'b000,
          $sformatf("SPI_IS 0x%02h STAT 0x%08h", v, st));
    wr(IS, 32'h02); rd(IS, v);
    check("W1C RXWM while RX level >= RX_WM_LVL: latched again at once (level source)", v[5:0] === 6'h03,
          $sformatf("0x%02h", v));
    w1c_step(5); rd(IS, v);
    check("  RX FIFO drained: W1C RXWM clears it", v[5:0] === 6'h01, $sformatf("0x%02h", v));
    wr(IS, 32'h01); rd(IS, v);
    check("W1C TXWM while TX level <= TX_WM_LVL: latched again at once", v[5:0] === 6'h01, $sformatf("0x%02h", v));
    w1c_step(6); rd(IS, v);
    check("  TX FIFO filled above TX_WM_LVL: W1C TXWM clears it", v[5:0] === 6'h00, $sformatf("0x%02h", v));
  endtask

  // ================= FI-04 / FI-05: irq = IS & IE with the DMA interlock, tx_dreq / rx_dreq =================
  task automatic t_irq();
    logic [31:0] c0, isv, isv2, st; int bad_irq, bad_dreq;
    $display("[irq] irq for every SPI_IE value x TXDMAEN x RXDMAEN, at 7 SPI_IS states (spec 6.6)");
    make_all_sources();
    for (int s = 0; s < 7; s++) begin
      w1c_step(s);
      rd(CTRL, c0); rd(IS, isv);
      bad_irq = 0; bad_dreq = 0;
      for (int dm = 0; dm < 4; dm++) begin
        logic txd, rxd;
        txd = ((dm & 1) != 0); rxd = ((dm & 2) != 0);
        wr(CTRL, (c0 & ~(C_TXDMA | C_RXDMA)) | (txd ? C_TXDMA : 32'h0) | (rxd ? C_RXDMA : 32'h0));
        rd(STAT, st);
        if (tx_dreq !== (st[5] & txd) || rx_dreq !== (st[6] & rxd)) bad_dreq++;
        for (int n = 0; n < 64; n++) begin
          wr(IE, 32'(n));
          if (irq !== irq_exp(isv[5:0], 6'(n), txd, rxd)) bad_irq++;
        end
      end
      wr(IE, 32'h0); wr(CTRL, c0); rd(IS, isv2);
      check($sformatf("SPI_IS 0x%02h: irq right for all 64 IE values x 4 DMA settings", isv[5:0]),
            isv[5:0] === is_after(s) && bad_irq == 0,
            $sformatf("%0d of 256 wrong, SPI_IS expected 0x%02h", bad_irq, is_after(s)));
      check("  tx_dreq = TX_WM & TXDMAEN, rx_dreq = RX_WM & RXDMAEN; IE and DMA leave SPI_IS alone",
            bad_dreq == 0 && isv2 === isv, $sformatf("%0d of 4 wrong, SPI_IS 0x%02h -> 0x%02h", bad_dreq, isv, isv2));
    end
  endtask

  // ================= RS-01 deeper: SWRST preserves configuration, clears status =================
  task automatic t_swrst();
    logic [31:0] v, st, lv, c; logic e; bit ok, same; int k0;
    $display("[swrst] SWRST keeps every configuration register, clears status and FIFOs (spec 7.5)");
    make_all_sources();
    wr(CTRL, C_MODE | dfs(8)); wait_ref(10);                // EN = 0, still slave mode
    wr(CTRL, C_MODE | C_CPHA | C_LSB | C_TXDMA | C_RXDMA | dfs(DFS_MAX - 1));
    wr(CLKDIV, 32'h5A);
    wr(CSCTRL, (32'h2A << 9) | (32'h15 << 3) | 32'h2);       // CS_HOLD 42, CS_SETUP 21, CS_AUTO 0, CS_SEL 2
    wr(FIFOCTRL, (32'(DEPTH - 1) << 16) | (32'h1 << 8));
    wr(IE, 32'h2A);
    for (int k = 0; k < 3; k++) wr(TXD, pat(k));
    rd(STAT, st); rd(FIFOLVL, lv);
    info("before SWRST", $sformatf("STAT 0x%08h FIFOLVL 0x%08h", st, lv));
    snapshot();
    rd(CTRL, c);
    k0 = dreq_busy;
    wr_e(CTRL, c | C_SWRST, e);
    wait_stat(32'h800, 32'h0, ok);
    snapshot_b(same);
    same = 1'b1;
    for (int i = 0; i < 5; i++) if (snap_a[i] !== snap_b[i]) same = 1'b0;
    check("SWRST accepted and RST_BUSY cleared", e === 1'b0 && ok, $sformatf("PSLVERR %b", e));
    check("  CTRL, CLKDIV, IE, CSCTRL, FIFOCTRL thresholds unchanged", same,
          $sformatf("CTRL 0x%08h CLKDIV 0x%02h IE 0x%02h CSCTRL 0x%04h FIFOCTRL 0x%08h",
                    snap_b[0], snap_b[1], snap_b[2], snap_b[3], snap_b[4]));
    rd(IS, v); rd(STAT, st); rd(FIFOLVL, lv);
    check("  SPI_IS cleared; only TXWM latched again (its condition holds)", v === 32'h1, $sformatf("0x%02h", v));
    check("  STAT sticky flags 0, both FIFOs empty, not busy",
          st[9:7] === 3'b000 && st[3] === 1'b1 && st[1] === 1'b1 && st[0] === 1'b0 && lv === 32'h0,
          $sformatf("STAT 0x%08h FIFOLVL 0x%08h", st, lv));
    check("  tx_dreq and rx_dreq low during the whole sequence (spec 7.5 step 6)", dreq_busy == k0,
          $sformatf("%0d pclk cycles with a request while RST_BUSY", dreq_busy - k0));
    check("  afterwards tx_dreq = 1 (TXDMAEN, TX_WM), rx_dreq = 0", tx_dreq === 1'b1 && rx_dreq === 1'b0,
          $sformatf("tx_dreq %b rx_dreq %b", tx_dreq, rx_dreq));
  endtask

  string test = "all";
  initial begin
    presetn = 1'b1; spi_rst_n = 1'b1;
    psel = 1'b0; penable = 1'b0; pwrite = 1'b0; paddr = 8'h0; pwdata = 32'h0;
    sclk_i = 1'b0; mosi_i = 1'b0; cs_n_i = 1'b1;
    void'($value$plusargs("test=%s", test));
    $display("tb_regs: DATA_WIDTH %0d, FIFO_DEPTH %0d, DEF_CPOL %0d, DEF_CPHA %0d, test=%s",
             DW, DEPTH, DEF_CPOL, DEF_CPHA, test);
    if (test == "all" || test == "reset")   begin reset_dut(); t_reset();   end
    if (test == "all" || test == "apb")     begin reset_dut(); t_apb();     end
    if (test == "all" || test == "dfs")     begin reset_dut(); t_dfs();     end
    if (test == "all" || test == "txfull")  begin reset_dut(); t_txfull();  end
    if (test == "all" || test == "rstbusy") begin reset_dut(); t_rstbusy(); end
    if (test == "all" || test == "bitbash") begin reset_dut(); t_bitbash(); end
    if (test == "all" || test == "status")  begin reset_dut(); t_status();  end
    if (test == "all" || test == "flush")   begin t_flush();   end
    if (test == "all" || test == "w1c")     begin reset_dut(); t_w1c();     end
    if (test == "all" || test == "irq")     begin reset_dut(); t_irq();     end
    if (test == "all" || test == "swrst")   begin reset_dut(); t_swrst();   end
    $display("[apb rules, whole run]");
    check("pready = 1 in every ACCESS phase", pready_low == 0, $sformatf("%0d cycles low", pready_low));
    check("PSLVERR low outside the ACCESS phase", slverr_idle == 0, $sformatf("%0d cycles high", slverr_idle));
    check("helper accesses never got PSLVERR", unexp == 0, $sformatf("%0d", unexp));
    $display("RESULT %0d pass %0d fail", npass, nfail);
    $finish;
  end
  initial begin #20_000_000; $display("TIMEOUT"); $display("RESULT %0d pass %0d fail", npass, nfail + 1); $finish; end
endmodule
