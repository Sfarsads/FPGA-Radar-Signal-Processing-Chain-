module radar_cfar
  import radar_pkg::*;
(
  input  logic                        clk,
  input  logic                        rst_n,
  input  logic [ALPHA_W-1:0]          alpha_q,

  input  logic                        v,
  input  logic signed [DATA_BITS-1:0] zr, zi,

  output logic                        m_v,        // |z|^2 stream
  output logic [MAG2_W-1:0]           m_data,

  output logic                        d_v,        // detection stream, one entry per cell under test
  output logic [15:0]                 d_idx,      // index of the cell under test
  output logic                        d_flag,
  output logic                        d_report
);
  // ---- |z|^2 (2 cycles) --------------------------------------------------------------
  logic [2*DATA_BITS-1:0] sq_r, sq_i;
  logic                   v1;
  initial begin sq_r = '0; sq_i = '0; v1 = 1'b0; m_v = 1'b0; m_data = '0; end
  always @(posedge clk) begin
    sq_r <= zr * zr;
    sq_i <= zi * zi;
    v1   <= v;
  end
`ifdef MUT_MAG2RND
  localparam logic [2*DATA_BITS:0] RND = '0;                                                   // MUTATION: |z|^2 truncated, not rounded
`else
  localparam logic [2*DATA_BITS:0] RND = (MAG2_SHIFT > 0) ? ((2*DATA_BITS+1)'(1) << (MAG2_SHIFT-1)) : '0;
`endif
  wire [2*DATA_BITS:0] msum = (2*DATA_BITS+1)'(sq_r) + (2*DATA_BITS+1)'(sq_i) + RND;
  always @(posedge clk) begin
    m_v    <= v1;
    m_data <= MAG2_W'(msum >> MAG2_SHIFT);
  end

  // ---- sliding window ------------------------------------------------------------------
  logic [MAG2_W-1:0] win [0:24];
  logic [SUM_W-1:0]  sum_lead, sum_lag;
  logic [15:0]       cnt;                       // samples received
  initial for (int i = 0; i < 25; i++) win[i] = '0;

  always @(posedge clk) begin
    if (!rst_n) begin
      for (int i = 0; i < 25; i++) win[i] <= '0;
      sum_lead <= '0;
      sum_lag  <= '0;
      cnt      <= '0;
    end else if (m_v) begin
      win[0] <= m_data;
      for (int i = 1; i < 25; i++) win[i] <= win[i-1];
`ifdef MUT_GUARD
      sum_lead <= sum_lead + SUM_W'(m_data) - SUM_W'(win[6]);   // MUTATION: training window one cell too small
`else
      sum_lead <= sum_lead + SUM_W'(m_data) - SUM_W'(win[7]);
`endif
      sum_lag  <= sum_lag  + SUM_W'(win[16]) - SUM_W'(win[24]);
      cnt      <= cnt + 16'd1;
    end
  end

  logic vb;
  always @(posedge clk) vb <= rst_n & m_v;

  // ---- stage B: sum, peak test ------------------------------------------------------------
  wire [MAG2_W-1:0] cut = win[12];
`ifdef MUT_PEAKWIN
  wire is_max = (cut >= win[11]) && (cut >= win[13]);                                         // MUTATION: peak window +/-1 instead of +/-2
`else
  wire is_max = (cut >= win[10]) && (cut >= win[11]) && (cut >= win[13]) && (cut >= win[14]);
`endif

  logic [SUM_W-1:0]  sum_b;
  logic [MAG2_W-1:0] cut_b;
  logic              pk_b, full_b, vb2;
  logic [15:0]       idx_b;
  initial begin sum_b = '0; cut_b = '0; pk_b = 1'b0; full_b = 1'b0; vb2 = 1'b0; idx_b = '0; end
  always @(posedge clk) begin
    sum_b  <= sum_lead + sum_lag;
    cut_b  <= cut;
    pk_b   <= is_max;
    full_b <= (cnt >= 16'd25);                // whole 25-sample window is real data
    idx_b  <= cnt - 16'd13;                   // newest is cnt-1, cell under test is 12 older
    vb2    <= vb;
  end

  // ---- stage C: threshold product ----------------------------------------------------------
  logic [SUM_W+ALPHA_W-1:0] prod_c;
  logic [MAG2_W-1:0]        cut_c;
  logic                     pk_c, full_c, vc;
  logic [15:0]              idx_c;
  initial begin prod_c = '0; cut_c = '0; pk_c = 1'b0; full_c = 1'b0; vc = 1'b0; idx_c = '0; end
  always @(posedge clk) begin
    prod_c <= sum_b * alpha_q;
    cut_c  <= cut_b;
    pk_c   <= pk_b;
    full_c <= full_b;
    idx_c  <= idx_b;
    vc     <= vb2;
  end

  // ---- stage D: compare ----------------------------------------------------------------------
`ifdef MUT_ALPHA
  localparam int AF = ALPHA_FRAC - 1;                  // MUTATION: wrong threshold scaling
`else
  localparam int AF = ALPHA_FRAC;
`endif
  wire [SUM_W+ALPHA_W-1:0] thr = (prod_c + ((SUM_W+ALPHA_W)'(1) << (AF-1))) >> AF;
  wire                     flag = full_c && (SUM_W+ALPHA_W)'(cut_c) > thr;

  initial begin d_v = 1'b0; d_idx = '0; d_flag = 1'b0; d_report = 1'b0; end
  always @(posedge clk) begin
    d_v      <= vc && full_c;
    d_idx    <= idx_c;
    d_flag   <= flag;
    d_report <= flag && pk_c;
  end
endmodule
