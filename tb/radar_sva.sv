// radar_sva.sv: protocol and datapath assertions for radar_top (bound in radar_bind.sv).
//
// Written in the SVA subset that both Questa and Verilator 5.020 accept: implications, $past,
// $stable, $rose, $isunknown, disable iff. Anything sequence-like (burst lengths, index continuity)
// is a small procedural checker instead of ##N / [*N] sequences.
//
// Every failure prints a message and increments `fails`; the testbench reads it at the end.
module radar_sva
  import radar_pkg::*;
  import radar_tb_pkg::*;
(
  input logic                        clk,
  input logic                        rst_n,

  input logic                        s_axis_tvalid,
  input logic                        s_axis_tready,
  input logic [2*IN_BITS-1:0]        s_axis_tdata,

  input logic                        m_cmp_valid,
  input logic signed [DATA_BITS-1:0] m_cmp_i,
  input logic signed [DATA_BITS-1:0] m_cmp_q,
  input logic                        m_mag2_valid,
  input logic [MAG2_W-1:0]           m_mag2,
  input logic                        m_det_valid,
  input logic [15:0]                 m_det_idx,
  input logic                        m_det_flag,
  input logic                        m_det_report,

  input logic [7:0]                  s_axil_awaddr,
  input logic                        s_axil_awvalid,
  input logic                        s_axil_awready,
  input logic [31:0]                 s_axil_wdata,
  input logic                        s_axil_wvalid,
  input logic                        s_axil_wready,
  input logic [1:0]                  s_axil_bresp,
  input logic                        s_axil_bvalid,
  input logic                        s_axil_bready,
  input logic [7:0]                  s_axil_araddr,
  input logic                        s_axil_arvalid,
  input logic                        s_axil_arready,
  input logic [31:0]                 s_axil_rdata,
  input logic [1:0]                  s_axil_rresp,
  input logic                        s_axil_rvalid,
  input logic                        s_axil_rready,

  input logic                        enable,
  input logic [11:0]                 age,
  input logic [15:0]                 wr_hop,
  input logic [15:0]                 launched,
  input logic [15:0]                 blk_done,
  input logic [7:0]                  sat_inc
);
  function automatic void sva_fail(input string msg);
    sva_fail_count = sva_fail_count + 1;
    $display("[SVA FAIL] %0t %s", $time, msg);
  endfunction
  `define SVA_FAIL(msg) sva_fail(msg)

  // ---- AXI4-Stream input (source rules; checks the testbench driver) ------------------------
  a_axis_hold: assert property (@(posedge clk) disable iff (!rst_n)
      s_axis_tvalid && !s_axis_tready |=> s_axis_tvalid && $stable(s_axis_tdata))
    else `SVA_FAIL("AXI-Stream: tvalid dropped or tdata changed before tready");

  // no acceptance while disabled; accepted samples never overrun a bank that is still being read
  a_no_accept_disabled: assert property (@(posedge clk) disable iff (!rst_n)
      s_axis_tready |-> enable)
    else `SVA_FAIL("tready high while enable=0");

  a_bank_rule: assert property (@(posedge clk) disable iff (!rst_n)
      (wr_hop - blk_done) <= 16'd4)
    else `SVA_FAIL("input bank overrun: writer more than 4 hops ahead of the reader");

  a_launch_has_data: assert property (@(posedge clk) disable iff (!rst_n)
      launched <= wr_hop)
    else `SVA_FAIL("block launched without two complete hops");

  // ---- outputs ---------------------------------------------------------------------------------
  a_cmp_known: assert property (@(posedge clk) disable iff (!rst_n)
      m_cmp_valid |-> !$isunknown({m_cmp_i, m_cmp_q}))
    else `SVA_FAIL("X/Z on compressed output while valid");

  a_mag2_known: assert property (@(posedge clk) disable iff (!rst_n)
      m_mag2_valid |-> !$isunknown(m_mag2))
    else `SVA_FAIL("X/Z on mag2 while valid");

  a_det_known: assert property (@(posedge clk) disable iff (!rst_n)
      m_det_valid |-> !$isunknown({m_det_flag, m_det_report, m_det_idx}))
    else `SVA_FAIL("X/Z on detection outputs while valid");

  a_report_is_flag: assert property (@(posedge clk) disable iff (!rst_n)
      m_det_report |-> m_det_flag)
    else `SVA_FAIL("peak report without CFAR flag");

  a_mag2_follows_cmp: assert property (@(posedge clk) disable iff (!rst_n)
      m_mag2_valid |-> $past(m_cmp_valid, 2))
    else `SVA_FAIL("mag2 valid without compressed sample two cycles earlier");

  a_cmp_after_latency: assert property (@(posedge clk) disable iff (!rst_n)
      m_cmp_valid |-> (age >= 12'(T_OUT0)))
    else `SVA_FAIL("compressed output before the pipeline latency has elapsed");

  // burst shape: every real block produces exactly HOP consecutive valid outputs
  int unsigned run = 0;
  always @(posedge clk) begin
    if (!rst_n) run <= 0;
    else if (m_cmp_valid) run <= run + 1;
    else begin
      if (run != 0 && run != HOP) `SVA_FAIL("compressed output burst is not exactly HOP samples");
      run <= 0;
    end
  end

  // detection index: starts at GUARD+TRAIN and increments by one per valid entry
  logic        det_seen = 1'b0;
  logic [15:0] det_last = '0;
  always @(posedge clk) begin
    if (!rst_n) det_seen <= 1'b0;
    else if (m_det_valid) begin
      if (!det_seen && m_det_idx != 16'(GUARD + TRAIN)) `SVA_FAIL("first detection index is not GUARD+TRAIN");
      if (det_seen && m_det_idx != det_last + 16'd1)    `SVA_FAIL("detection index is not contiguous");
      det_seen <= 1'b1;
      det_last <= m_det_idx;
    end
  end

  // saturation increments per cycle are bounded by the number of saturating points
  a_sat_bound: assert property (@(posedge clk) disable iff (!rst_n)
      sat_inc <= 8'(2*NSTG*6 + 3 + 1))
    else `SVA_FAIL("saturation increment exceeds the number of saturation points");

  // ---- AXI4-Lite (both master and slave rules) -----------------------------------------------------
  a_aw_hold: assert property (@(posedge clk) disable iff (!rst_n)
      s_axil_awvalid && !s_axil_awready |=> s_axil_awvalid && $stable(s_axil_awaddr))
    else `SVA_FAIL("AXI-Lite: AW changed before awready");
  a_w_hold: assert property (@(posedge clk) disable iff (!rst_n)
      s_axil_wvalid && !s_axil_wready |=> s_axil_wvalid && $stable(s_axil_wdata))
    else `SVA_FAIL("AXI-Lite: W changed before wready");
  a_ar_hold: assert property (@(posedge clk) disable iff (!rst_n)
      s_axil_arvalid && !s_axil_arready |=> s_axil_arvalid && $stable(s_axil_araddr))
    else `SVA_FAIL("AXI-Lite: AR changed before arready");
  a_b_hold: assert property (@(posedge clk) disable iff (!rst_n)
      s_axil_bvalid && !s_axil_bready |=> s_axil_bvalid && $stable(s_axil_bresp))
    else `SVA_FAIL("AXI-Lite: bvalid dropped or bresp changed before bready");
  a_r_hold: assert property (@(posedge clk) disable iff (!rst_n)
      s_axil_rvalid && !s_axil_rready |=> s_axil_rvalid && $stable(s_axil_rdata) && $stable(s_axil_rresp))
    else `SVA_FAIL("AXI-Lite: rvalid dropped or rdata changed before rready");
  a_b_known: assert property (@(posedge clk) disable iff (!rst_n)
      s_axil_bvalid |-> !$isunknown(s_axil_bresp))
    else `SVA_FAIL("X on bresp");
  a_r_known: assert property (@(posedge clk) disable iff (!rst_n)
      s_axil_rvalid |-> !$isunknown({s_axil_rdata, s_axil_rresp}))
    else `SVA_FAIL("X on rdata");
  a_no_resp_overlap_w: assert property (@(posedge clk) disable iff (!rst_n)
      s_axil_bvalid |-> !(s_axil_awready || s_axil_wready))
    else `SVA_FAIL("AXI-Lite: slave accepts a new write while a response is pending");

  // ---- cover points (reported by the simulator's coverage report) -------------------------------------------
  c_backpressure: cover property (@(posedge clk) s_axis_tvalid && !s_axis_tready);
  c_output_burst: cover property (@(posedge clk) m_cmp_valid);
  c_report:       cover property (@(posedge clk) m_det_report);
  c_saturation:   cover property (@(posedge clk) sat_inc != 8'd0);
  c_disabled:     cover property (@(posedge clk) !enable);
  c_b_stall:      cover property (@(posedge clk) s_axil_bvalid && !s_axil_bready);
  c_r_stall:      cover property (@(posedge clk) s_axil_rvalid && !s_axil_rready);
  c_slverr_w:     cover property (@(posedge clk) s_axil_bvalid && s_axil_bresp == 2'b10);
  c_slverr_r:     cover property (@(posedge clk) s_axil_rvalid && s_axil_rresp == 2'b10);
endmodule
