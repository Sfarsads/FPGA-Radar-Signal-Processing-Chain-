// radar_tb.sv: self-checking testbench for radar_top.
//
//  * Runs every vector set in vectors/ twice: once with the input valid every cycle (full rate, exercises
//    back-pressure) and once with a random valid pattern (half rate / sparse / bursty).
//  * Checks compressed I/Q, |z|^2, CFAR flags and peak reports bit-exactly against the Python golden model,
//    plus output counts, detection-index continuity, and the AXI-Lite counters (SAT_COUNT vs the model).
//  * Directed AXI-Lite tests: ID, reset values, RW, byte strobes, read-only writes, SLVERR, enable stall
//    (also in the middle of a stream), counter clear, slow bready/rready.
//  * Assertions live in radar_sva.sv (bound by radar_bind.sv); their failure count is checked at the end.
//  * A hand-counted functional-coverage table and throughput/latency numbers are printed at the end.
// Prints "TESTBENCH PASSED" or "TESTBENCH FAILED".
`timescale 1ns/1ps
`ifndef VEC_DIR
  `define VEC_DIR "vectors"
`endif

module radar_tb;
  import radar_pkg::*;
  import radar_tb_pkg::*;

  // Some simulators pad "%02d" with a space instead of zero for single-digit values (seen on
  // Questa 2025.2), which breaks file names like "t00_in_i.hex". Build the two-digit string by
  // hand instead of relying on %02d anywhere test numbers become file names or bin labels.
  function automatic string tt(input int v);
    return (v < 10) ? $sformatf("0%0d", v) : $sformatf("%0d", v);
  endfunction

  localparam string VD     = `VEC_DIR;
  localparam int    L      = STREAM_LEN;
  localparam int    NIN    = L + HOP;                       // stream + one hop of zeros to flush the last block
  localparam int    CMPLIM = VALID_LEN - GUARD - TRAIN;     // CFAR flags are defined for cell index < CMPLIM
  localparam int    MAXT   = 32;

  // register map
  localparam logic [7:0] R_CTRL = 8'h00, R_ALPHA = 8'h04, R_SAT = 8'h08, R_BLK = 8'h0C, R_DET = 8'h10, R_ID = 8'h14;

  localparam int M_FULL = 0, M_HALF = 1, M_SPARSE = 2, M_BURSTY = 3;

  // ---- clock / DUT ---------------------------------------------------------------------------
  logic clk = 1'b0;
  always #5 clk = ~clk;
  logic rst_n = 1'b0;

  logic [31:0] tdata = '0;
  logic        tvalid = 1'b0;
  logic        tready;

  logic                        cmp_v;
  logic signed [DATA_BITS-1:0] cmp_i, cmp_q;
  logic                        mag2_v;
  logic [MAG2_W-1:0]           mag2;
  logic                        det_v, det_flag, det_rep;
  logic [15:0]                 det_idx;

  logic [7:0]  awaddr = '0, araddr = '0;
  logic        awvalid = 1'b0, wvalid = 1'b0, bready = 1'b1, arvalid = 1'b0, rready = 1'b1;
  logic [31:0] wdata = '0;
  logic [3:0]  wstrb = 4'hF;
  logic        awready, wready, bvalid, arready, rvalid;
  logic [1:0]  bresp, rresp;
  logic [31:0] rdata;

  radar_top dut (
    .clk(clk), .rst_n(rst_n),
    .s_axis_tdata(tdata), .s_axis_tvalid(tvalid), .s_axis_tready(tready),
    .m_cmp_valid(cmp_v), .m_cmp_i(cmp_i), .m_cmp_q(cmp_q),
    .m_mag2_valid(mag2_v), .m_mag2(mag2),
    .m_det_valid(det_v), .m_det_idx(det_idx), .m_det_flag(det_flag), .m_det_report(det_rep),
    .s_axil_awaddr(awaddr), .s_axil_awvalid(awvalid), .s_axil_awready(awready),
    .s_axil_wdata(wdata), .s_axil_wstrb(wstrb), .s_axil_wvalid(wvalid), .s_axil_wready(wready),
    .s_axil_bresp(bresp), .s_axil_bvalid(bvalid), .s_axil_bready(bready),
    .s_axil_araddr(araddr), .s_axil_arvalid(arvalid), .s_axil_arready(arready),
    .s_axil_rdata(rdata), .s_axil_rresp(rresp), .s_axil_rvalid(rvalid), .s_axil_rready(rready)
  );

  int cyc = 0;
  always @(posedge clk) cyc <= cyc + 1;

  // ---- functional coverage (hand-counted bins, works in any simulator) ------------------------------------
  localparam int NCOV = 32 + MAXT;
  bit    cov_en  [NCOV];
  bit    cov_hit [NCOV];
  string cov_nm  [NCOV];
  function automatic void cov_def(int i, string n); cov_en[i] = 1'b1; cov_nm[i] = n; endfunction
  function automatic void hit(int i); cov_hit[i] = 1'b1; endfunction
  localparam int C_MODE = 0, C_BP = 4, C_SAT = 5, C_FLAG = 6, C_REP = 7, C_NOREP = 8, C_REP_LO = 9, C_REP_MID = 10,
                 C_REP_HI = 11, C_REP_EARLY = 12, C_REP_LATE = 13, C_RD = 14 /*..19*/, C_WR_CTRL = 20, C_WR_ALPHA = 21,
                 C_WR_RO = 22, C_WR_ERR = 23, C_RD_ERR = 24, C_B_STALL = 25, C_R_STALL = 26, C_STALL_MID = 27,
                 C_CLEAR = 28, C_STROBE = 29, C_TEST = 32;

  initial begin
    for (int i = 0; i < NCOV; i++) begin cov_en[i] = 1'b0; cov_hit[i] = 1'b0; cov_nm[i] = ""; end
    cov_def(C_MODE+0, "input mode: full rate");     cov_def(C_MODE+1, "input mode: random 50%");
    cov_def(C_MODE+2, "input mode: sparse 12%");    cov_def(C_MODE+3, "input mode: bursty");
    cov_def(C_BP, "input back-pressure (tvalid && !tready)");
    cov_def(C_SAT, "datapath saturation observed");
    cov_def(C_FLAG, "CFAR flag raised");            cov_def(C_REP, "peak report raised");
    cov_def(C_NOREP, "run with no peak report");
    cov_def(C_REP_LO, "report in first 16 cells of a block");
    cov_def(C_REP_MID, "report mid block");         cov_def(C_REP_HI, "report in last 16 cells of a block");
    cov_def(C_REP_EARLY, "report in first 512 samples"); cov_def(C_REP_LATE, "report after sample 1024");
    cov_def(C_RD+0, "AXI-Lite read CTRL");   cov_def(C_RD+1, "AXI-Lite read ALPHA_Q");
    cov_def(C_RD+2, "AXI-Lite read SAT_COUNT"); cov_def(C_RD+3, "AXI-Lite read BLOCK_COUNT");
    cov_def(C_RD+4, "AXI-Lite read DETECT_COUNT"); cov_def(C_RD+5, "AXI-Lite read ID");
    cov_def(C_WR_CTRL, "AXI-Lite write CTRL"); cov_def(C_WR_ALPHA, "AXI-Lite write ALPHA_Q");
    cov_def(C_WR_RO, "AXI-Lite write to read-only register");
    cov_def(C_WR_ERR, "AXI-Lite write SLVERR");    cov_def(C_RD_ERR, "AXI-Lite read SLVERR");
    cov_def(C_B_STALL, "bvalid held by slow bready"); cov_def(C_R_STALL, "rvalid held by slow rready");
    cov_def(C_STALL_MID, "enable=0 in the middle of a stream");
    cov_def(C_CLEAR, "counter clear");              cov_def(C_STROBE, "partial byte-strobe write");
  end

  // ---- vectors ---------------------------------------------------------------------------------------
  int n_tests;
  logic [15:0]                 in_i    [0:L-1];
  logic [15:0]                 in_q    [0:L-1];
  logic signed [DATA_BITS-1:0] exp_i   [0:L-1];
  logic signed [DATA_BITS-1:0] exp_q   [0:L-1];
  logic [31:0]                 exp_m2  [0:L-1];
  logic                        exp_flag[0:L-1];
  logic                        exp_rep [0:L-1];
  logic [31:0]                 exp_sat [0:0];
  logic [15:0]                 alpha_t [0:0];
  logic [15:0]                 nt_mem  [0:0];

  task automatic load_test(input int t);
    string p;
    p = $sformatf("%s/t%s_", VD, tt(t));
    $readmemh({p, "in_i.hex"},       in_i);
    $readmemh({p, "in_q.hex"},       in_q);
    $readmemh({p, "exp_i.hex"},      exp_i);
    $readmemh({p, "exp_q.hex"},      exp_q);
    $readmemh({p, "exp_mag2.hex"},   exp_m2);
    $readmemh({p, "exp_flag.hex"},   exp_flag);
    $readmemh({p, "exp_report.hex"}, exp_rep);
    $readmemh({p, "exp_sat.hex"},    exp_sat);
    $readmemh({p, "alpha.hex"},      alpha_t);
  endtask

  // ---- AXI4-Stream driver -----------------------------------------------------------------------------------
  bit  drv_run  = 1'b0;
  int  drv_mode = M_FULL;
  int  sent     = 0;            // samples accepted by the DUT
  int  burst_left = 0;
  bit  burst_on = 1'b0;

  function automatic bit go();
    case (drv_mode)
      M_FULL:   return 1'b1;
      M_HALF:   return ($urandom_range(0, 1) == 1);
      M_SPARSE: return ($urandom_range(0, 7) == 0);
      default: begin
        if (burst_left == 0) begin
          burst_on   = !burst_on;
          burst_left = burst_on ? $urandom_range(1, 300) : $urandom_range(1, 400);
        end
        burst_left--;
        return burst_on;
      end
    endcase
  endfunction

  int nxt;
  always @(posedge clk) begin
    if (!rst_n || !drv_run) begin
      tvalid <= 1'b0;
    end else begin
      nxt = sent;
      if (tvalid && tready) nxt = sent + 1;
      sent <= nxt;
      if (tvalid && !tready) begin
        // hold valid and data until accepted
      end else if (nxt < NIN && go()) begin
        tvalid <= 1'b1;
        tdata  <= (nxt < L) ? {in_q[nxt], in_i[nxt]} : 32'd0;
      end else begin
        tvalid <= 1'b0;
      end
    end
  end

  // ---- scoreboard --------------------------------------------------------------------------------------------
  bit chk_en = 1'b0;
  int ncmp, nm2, ndet, nrep;
  int e_cmp, e_m2, e_flag, e_rep, e_idx, e_extra;
  int cyc_first_cmp;
  int nflag, acc_cnt;
  bit det_seen_idx [0:L-1];

  task automatic sb_reset();
    ncmp = 0; nm2 = 0; ndet = 0; nrep = 0;
    e_cmp = 0; e_m2 = 0; e_flag = 0; e_rep = 0; e_idx = 0; e_extra = 0;
    cyc_first_cmp = -1;
    nflag = 0; acc_cnt = 0;
  endtask

  always @(posedge clk) begin
    if (chk_en) begin
      // accept-side timing marks
      if (tvalid && tready) begin
        acc_cnt = acc_cnt + 1;
      end
      if (tvalid && !tready) hit(C_BP);

      if (cmp_v) begin
        if (cyc_first_cmp < 0) cyc_first_cmp = cyc;
        if (ncmp >= L) e_extra++;
        else if (cmp_i !== exp_i[ncmp] || cmp_q !== exp_q[ncmp]) begin
          e_cmp++;
          if (e_cmp <= 5) $display("  [cmp] idx %0d: got (%0d,%0d) expected (%0d,%0d)", ncmp, cmp_i, cmp_q, exp_i[ncmp], exp_q[ncmp]);
        end
        ncmp++;
      end

      if (mag2_v) begin
        if (nm2 >= L) e_extra++;
        else if ({4'd0, mag2} !== exp_m2[nm2]) begin
          e_m2++;
          if (e_m2 <= 5) $display("  [mag2] idx %0d: got %0d expected %0d", nm2, mag2, exp_m2[nm2]);
        end
        nm2++;
      end

      if (det_v) begin
        if (det_idx != 16'(GUARD + TRAIN + ndet)) begin
          e_idx++;
          if (e_idx <= 5) $display("  [det] index %0d, expected %0d", det_idx, GUARD + TRAIN + ndet);
        end
        ndet++;
        if (det_flag) nflag = nflag + 1;
        if (det_rep) begin
          nrep = nrep + 1;
          if (det_idx[6:0] < 7'd16)      hit(C_REP_LO);
          else if (det_idx[6:0] >= 7'd112) hit(C_REP_HI);
          else                           hit(C_REP_MID);
          if (det_idx < 16'd512)   hit(C_REP_EARLY);
          if (det_idx >= 16'd1024) hit(C_REP_LATE);
        end
        if (det_idx < 16'(CMPLIM)) begin
          if (det_flag !== exp_flag[det_idx]) begin
            e_flag++;
            if (e_flag <= 5) $display("  [flag] cell %0d: got %0d expected %0d", det_idx, det_flag, exp_flag[det_idx]);
          end
          if (det_rep !== exp_rep[det_idx]) begin
            e_rep++;
            if (e_rep <= 5) $display("  [report] cell %0d: got %0d expected %0d", det_idx, det_rep, exp_rep[det_idx]);
          end
        end
      end
    end
  end

  // ---- AXI4-Lite master tasks --------------------------------------------------------------------------------
  bit poll_busy = 1'b0;

  // Timing convention for every task that drives DUT inputs: drive and sample at the NEGEDGE, i.e. away
  // from the active clock edge, so there is no TB/DUT race in any simulator. A signal sampled at a negedge
  // has the value the DUT will see at the next posedge; a handshake completes at that posedge iff valid and
  // ready were both high at the negedge.
  // Handshakes are checked at @(posedge clk) -- the same edge the DUT itself uses to decide
  // whether a transfer occurs -- while every driven signal is set at the preceding @(negedge clk).
  // Driving and checking on opposite edges guarantees a full half-cycle of separation, so there is
  // no ambiguity about whether a just-driven value is visible yet. Checking on the SAME edge type
  // used for driving (tried and discarded here) left a one-event race in some scheduling orders:
  // a delayed ready could be missed because the newly asserted value and the check occurred in the
  // same simulation step, before the value had propagated.
  task automatic axil_write(input logic [7:0] a, input logic [31:0] d, input logic [3:0] strb,
                            input int bready_delay, output logic [1:0] resp);
    bit aw_done, w_done;
    int waited;
    aw_done = 1'b0; w_done = 1'b0; waited = 0; resp = 2'b00;
    @(negedge clk);
    awaddr <= a; awvalid <= 1'b1; wdata <= d; wstrb <= strb; wvalid <= 1'b1;
    bready <= (bready_delay <= 0);
    forever begin
      @(posedge clk);
      if (!aw_done && awready) aw_done = 1'b1;
      if (!w_done  && wready)  w_done  = 1'b1;
      if (aw_done && w_done) begin @(negedge clk); awvalid <= 1'b0; wvalid <= 1'b0; break; end
      waited++;
      if (waited > 200) begin
        @(negedge clk); awvalid <= 1'b0; wvalid <= 1'b0;
        $display("  [axil] write handshake timeout"); break;
      end
    end
    waited = 0;
    forever begin
      @(posedge clk);
      if (bvalid && bready) begin resp = bresp; break; end
      waited++;
      if (waited >= bready_delay) begin @(negedge clk); bready <= 1'b1; end
      if (waited > 200) begin $display("  [axil] no write response"); resp = 2'bxx; break; end
    end
    @(negedge clk); bready <= 1'b1;
    if (bready_delay > 0) hit(C_B_STALL);
    if (a == R_CTRL)  hit(C_WR_CTRL);
    if (a == R_ALPHA) hit(C_WR_ALPHA);
    if (a >= R_SAT && a <= R_ID) hit(C_WR_RO);
    if (resp == 2'b10) hit(C_WR_ERR);
    if (strb != 4'hF)  hit(C_STROBE);
  endtask

  task automatic axil_read(input logic [7:0] a, input int rready_delay, output logic [31:0] d, output logic [1:0] resp);
    int waited;
    waited = 0; d = '0; resp = 2'b00;
    @(negedge clk);
    araddr <= a; arvalid <= 1'b1; rready <= (rready_delay <= 0);
    forever begin
      @(posedge clk);
      if (arvalid && arready) begin @(negedge clk); arvalid <= 1'b0; break; end
      waited++;
      if (waited > 200) begin $display("  [axil] read address timeout"); break; end
    end
    waited = 0;
    forever begin
      @(posedge clk);
      if (rvalid && rready) begin d = rdata; resp = rresp; break; end
      waited++;
      if (waited >= rready_delay) begin @(negedge clk); rready <= 1'b1; end
      if (waited > 200) begin $display("  [axil] no read data"); d = 'x; resp = 2'bxx; break; end
    end
    @(negedge clk); rready <= 1'b1;
    if (rready_delay > 0) hit(C_R_STALL);
    if (a <= R_ID) hit(C_RD + int'(a[7:2]));
    if (resp == 2'b10) hit(C_RD_ERR);
  endtask

  // background reader: hammers the read-only registers while a stream is running
  bit poll_en = 1'b0;
  initial begin
    logic [31:0] d; logic [1:0] r; int a;
    forever begin
      wait (poll_en);
      poll_busy = 1'b1;
      a = $urandom_range(0, 5);
      axil_read(8'(a * 4), $urandom_range(0, 3), d, r);
      repeat ($urandom_range(0, 25)) @(posedge clk);
      poll_busy = 1'b0;
      @(posedge clk);
    end
  end

  // ---- bookkeeping --------------------------------------------------------------------------------------------
  int n_fail = 0, n_pass = 0, n_runs = 0;
  real rate_min = 9.9, rate_max = 0.0;
  int  lat_min = 1 << 30, lat_max = 0;

  task automatic fail(input string msg);
    n_fail++;
    $display("  FAIL: %s", msg);
  endtask

  task automatic expect32(input string what, input logic [31:0] got, input logic [31:0] exp);
    if (got !== exp) fail($sformatf("%s = 0x%08h, expected 0x%08h", what, got, exp));
  endtask

  task automatic do_reset();
    drv_run = 1'b0;
    @(negedge clk);
    rst_n <= 1'b0;
    repeat (6) @(negedge clk);
    rst_n <= 1'b1;
    repeat (2) @(negedge clk);
  endtask

  // ---- one full run of one vector set -------------------------------------------------------------------------------
  task automatic run_one(input int t, input int mode, input bit stall_test, input bit do_clear);
    logic [31:0] d;
    logic [1:0]  r;
    logic [1:0]  resp;
    int   guard, errs_before, sent_before;
    int   c_first, c_512, c_end;
    real  rate;
    string tag;

    tag = $sformatf("t%s mode %0d%s", tt(t), mode, stall_test ? " +stall" : "");
    errs_before = n_fail;
    load_test(t);
    do_reset();
    sb_reset();
    sent = 0; burst_left = 0; burst_on = 1'b0;
    drv_mode = mode;
    hit(C_MODE + mode);

    // program the CFAR multiplier for this vector set and read it back
    axil_write(R_ALPHA, {16'd0, alpha_t[0]}, 4'hF, 0, resp);
    if (resp !== 2'b00) fail("ALPHA_Q write response");
    axil_read(R_ALPHA, 0, d, r);
    expect32("ALPHA_Q readback", d, {16'd0, alpha_t[0]});

    chk_en  = 1'b1;
    drv_run = 1'b1;
    poll_en = (mode == M_HALF || mode == M_BURSTY) && !stall_test;

    // throughput timestamps (full-rate runs): the run thread watches the scoreboard's accept counter
    c_first = -1; c_512 = -1; c_end = -1;
    if (mode == M_FULL && !stall_test) begin
      guard = 0; while (acc_cnt < 1   && guard < 5000)  begin @(posedge clk); guard++; end   c_first = cyc;
      guard = 0; while (acc_cnt < 1024 && guard < 8000)  begin @(posedge clk); guard++; end  c_512   = cyc;
      guard = 0; while (acc_cnt < 2048 && guard < 20000) begin @(posedge clk); guard++; end  c_end   = cyc;
    end

    if (stall_test) begin
      guard = 0;
      while (sent < 700 && guard < 20000) begin @(posedge clk); guard++; end
      axil_write(R_CTRL, 32'd0, 4'hF, 0, resp);             // enable = 0
      sent_before = sent;
      repeat (300) @(posedge clk);
      if (tready !== 1'b0) fail("tready still high with enable=0");
      if (sent != sent_before) fail("samples accepted while enable=0");
      hit(C_STALL_MID);
      axil_write(R_CTRL, 32'd1, 4'hF, 0, resp);             // enable = 1
    end

    guard = 0;
    while (nm2 < L && guard < 120000) begin @(posedge clk); guard++; end
    if (guard >= 120000) fail("timeout waiting for outputs");
    repeat (40) @(posedge clk);
    drv_run = 1'b0;
    poll_en = 1'b0;
    guard = 0;
    while (poll_busy && guard < 400) begin @(posedge clk); guard++; end
    chk_en  = 1'b0;

    // ---- data checks
    if (ncmp != L)          fail($sformatf("compressed outputs: %0d, expected %0d", ncmp, L));
    if (nm2  != L)          fail($sformatf("mag2 outputs: %0d, expected %0d", nm2, L));
    if (ndet != L - 2 * (GUARD + TRAIN)) fail($sformatf("detection entries: %0d, expected %0d", ndet, L - 2 * (GUARD + TRAIN)));
    if (e_cmp  != 0)        fail($sformatf("%0d compressed I/Q mismatches", e_cmp));
    if (e_m2   != 0)        fail($sformatf("%0d |z|^2 mismatches", e_m2));
    if (e_flag != 0)        fail($sformatf("%0d CFAR flag mismatches", e_flag));
    if (e_rep  != 0)        fail($sformatf("%0d peak report mismatches", e_rep));
    if (e_idx  != 0)        fail($sformatf("%0d detection index errors", e_idx));
    if (e_extra != 0)       fail($sformatf("%0d extra outputs beyond the stream", e_extra));

    // ---- register checks (SAT_COUNT against the model)
    axil_read(R_SAT, 0, d, r);   expect32("SAT_COUNT", d, exp_sat[0]);
    if (d > 0) hit(C_SAT);
    axil_read(R_BLK, 0, d, r);   expect32("BLOCK_COUNT", d, 32'(L / HOP));
    axil_read(R_DET, 0, d, r);   expect32("DETECT_COUNT", d, 32'(nrep));

    if (do_clear) begin
      axil_write(R_CTRL, 32'd3, 4'hF, 0, resp);             // enable stays 1, clear counters
      repeat (4) @(posedge clk);
      axil_read(R_SAT, 0, d, r); expect32("SAT_COUNT after clear", d, 32'd0);
      axil_read(R_BLK, 0, d, r); expect32("BLOCK_COUNT after clear", d, 32'd0);
      axil_read(R_DET, 0, d, r); expect32("DETECT_COUNT after clear", d, 32'd0);
      hit(C_CLEAR);
    end

    if (nflag > 0) hit(C_FLAG);
    if (nrep > 0)  hit(C_REP);
    else           hit(C_NOREP);

    // ---- performance numbers (full-rate runs)
    if (mode == M_FULL && !stall_test && c_512 > c_first && c_end > c_512) begin
      rate = 1024.0 / real'(c_end - c_512);      // both points are already back-pressured by the engine
      if (rate < rate_min) rate_min = rate;
      if (rate > rate_max) rate_max = rate;
      if (cyc_first_cmp - c_first < lat_min) lat_min = cyc_first_cmp - c_first;
      if (cyc_first_cmp - c_first > lat_max) lat_max = cyc_first_cmp - c_first;
    end

    n_runs++;
    if (n_fail == errs_before) begin
      n_pass++;
      hit(C_TEST + t);
      $display("PASS  %s  (%0d cmp, %0d cells, %0d reports, sat %0d)", tag, ncmp, ndet, nrep, exp_sat[0]);
    end else begin
      $display("FAIL  %s", tag);
    end
  endtask

  // ---- directed AXI-Lite tests -----------------------------------------------------------------------------------------
  task automatic directed_axil();
    logic [31:0] d;
    logic [1:0]  r;
    $display("-- directed AXI-Lite tests");
    do_reset();
    axil_read(R_ID, 0, d, r);       expect32("ID", d, 32'h52414452);           if (r !== 2'b00) fail("ID resp");
    axil_read(R_CTRL, 0, d, r);     expect32("CTRL reset value", d, 32'd1);
    axil_read(R_ALPHA, 0, d, r);    expect32("ALPHA_Q reset value", d, 32'(ALPHA_Q));
    axil_read(R_SAT, 0, d, r);      expect32("SAT_COUNT reset value", d, 32'd0);
    axil_read(R_BLK, 0, d, r);      expect32("BLOCK_COUNT reset value", d, 32'd0);
    axil_read(R_DET, 0, d, r);      expect32("DETECT_COUNT reset value", d, 32'd0);

    axil_write(R_ALPHA, 32'h0000_1234, 4'hF, 0, r);  if (r !== 2'b00) fail("ALPHA write resp");
    axil_read(R_ALPHA, 0, d, r);    expect32("ALPHA_Q after write", d, 32'h1234);
    axil_write(R_ALPHA, 32'hFFFF_AB00, 4'b0010, 0, r);                        // only byte 1 changes
    axil_read(R_ALPHA, 0, d, r);    expect32("ALPHA_Q after byte-1 strobe write", d, 32'hAB34);
    axil_write(R_ALPHA, 32'h0000_0FF0, 4'hF, 3, r);                           // slow bready
    axil_read(R_ALPHA, 4, d, r);    expect32("ALPHA_Q with slow bready/rready", d, 32'h0FF0);

    axil_write(R_SAT, 32'hDEAD_BEEF, 4'hF, 0, r);  if (r !== 2'b00) fail("RO write should respond OKAY");
    axil_read(R_SAT, 0, d, r);      expect32("SAT_COUNT unchanged by write", d, 32'd0);
    axil_write(R_ID, 32'd0, 4'hF, 0, r);
    axil_read(R_ID, 0, d, r);       expect32("ID unchanged by write", d, 32'h52414452);

    axil_write(8'h40, 32'd1, 4'hF, 0, r);          if (r !== 2'b10) fail("write to unmapped address should be SLVERR");
    axil_read(8'h44, 0, d, r);      if (r !== 2'b10) fail("read of unmapped address should be SLVERR");
    expect32("unmapped read data", d, 32'd0);

    axil_write(R_CTRL, 32'd0, 4'hF, 0, r);                                    // disable
    if (tready !== 1'b0) fail("tready should be low with enable=0");
    axil_read(R_CTRL, 0, d, r);     expect32("CTRL after disable", d, 32'd0);
    axil_write(R_CTRL, 32'd1, 4'hF, 0, r);
    @(posedge clk);
    if (tready !== 1'b1) fail("tready should be high after re-enable");

    do_reset();
    axil_read(R_ALPHA, 0, d, r);    expect32("ALPHA_Q restored by reset", d, 32'(ALPHA_Q));
    $display("-- directed AXI-Lite tests done");
  endtask

  // ---- main sequence ---------------------------------------------------------------------------------------------------------
  initial begin
    $readmemh({VD, "/num_tests.hex"}, nt_mem);
    n_tests = int'(nt_mem[0]);
    if (n_tests < 1 || n_tests > MAXT) begin
      $display("TESTBENCH FAILED: bad num_tests.hex (%0d); run radar_make_vectors.py", n_tests);
      $finish;
    end
    $display("radar_tb: %0d vector sets, stream %0d samples, block %0d, hop %0d", n_tests, L, N_FFT, HOP);
    for (int i = 0; i < n_tests; i++) cov_def(C_TEST + i, $sformatf("vector set t%s passed", tt(i)));
    sb_reset();

    directed_axil();

    for (int t = 0; t < n_tests; t++) begin
      run_one(t, M_FULL,               1'b0, (t == 0));
      run_one(t, 1 + (t % 3),          1'b0, 1'b0);
    end
    run_one(2, M_FULL, 1'b1, 1'b0);          // enable stall in the middle of a stream

    // ---- summary
    begin
      int sva_fails, hits, total;
      sva_fails = int'(radar_tb_pkg::sva_fail_count);
      hits = 0; total = 0;
      $display("\n---- functional coverage ----");
      for (int i = 0; i < NCOV; i++) if (cov_en[i]) begin
        total++;
        if (cov_hit[i]) hits++;
        else $display("  MISSED: %s", cov_nm[i]);
      end
      $display("  %0d / %0d bins hit  (%0d%%)", hits, total, (100 * hits) / total);
      $display("---- performance (full-rate input) ----");
      $display("  steady-state input rate: %0.3f samples/clock (min %0.3f, max %0.3f)  ->  %0.1f MSPS at 100 MHz",
               rate_min, rate_min, rate_max, rate_min * 100.0);
      $display("  latency: block launch -> first compressed sample = %0d cycles (fixed pipeline)", T_OUT0);
      $display("  latency: first accepted input sample -> first compressed output = %0d cycles (includes filling 2 hops)", lat_min);
      $display("---- summary ----");
      $display("  %0d runs, %0d passed, %0d check failures, %0d assertion failures", n_runs, n_pass, n_fail, sva_fails);
      if (n_fail == 0 && sva_fails == 0 && n_pass == n_runs) $display("TESTBENCH PASSED");
      else                                                  $display("TESTBENCH FAILED");
    end
    $finish;
  end
endmodule
