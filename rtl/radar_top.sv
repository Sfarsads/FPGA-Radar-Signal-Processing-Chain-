 import radar_pkg::*;
(
  input  logic                        clk,
  input  logic                        rst_n,

  // AXI4-Stream slave: input samples
  input  logic [2*IN_BITS-1:0]        s_axis_tdata,     // {Q, I}
  input  logic                        s_axis_tvalid,
  output logic                        s_axis_tready,

  // compressed pulse (first 128 valid outputs of every block, natural order)
  output logic                        m_cmp_valid,
  output logic signed [DATA_BITS-1:0] m_cmp_i,
  output logic signed [DATA_BITS-1:0] m_cmp_q,

  // |z|^2 and detections
  output logic                        m_mag2_valid,
  output logic [MAG2_W-1:0]           m_mag2,
  output logic                        m_det_valid,
  output logic [15:0]                 m_det_idx,
  output logic                        m_det_flag,
  output logic                        m_det_report,

  // AXI4-Lite slave: control / status
  input  logic [7:0]                  s_axil_awaddr,
  input  logic                        s_axil_awvalid,
  output logic                        s_axil_awready,
  input  logic [31:0]                 s_axil_wdata,
  input  logic [3:0]                  s_axil_wstrb,
  input  logic                        s_axil_wvalid,
  output logic                        s_axil_wready,
  output logic [1:0]                  s_axil_bresp,
  output logic                        s_axil_bvalid,
  input  logic                        s_axil_bready,
  input  logic [7:0]                  s_axil_araddr,
  input  logic                        s_axil_arvalid,
  output logic                        s_axil_arready,
  output logic [31:0]                 s_axil_rdata,
  output logic [1:0]                  s_axil_rresp,
  output logic                        s_axil_rvalid,
  input  logic                        s_axil_rready
);
  initial begin
    for (int s = 0; s < NSTG; s++)
      if (FWD_SHIFTS[s] > 1 || INV_SHIFTS[s] > 1 || FWD_SHIFTS[s] < 0 || INV_SHIFTS[s] < 0)
        $fatal(1, "radar_top: per-stage FFT shifts must be 0 or 1");
    if (DATA_BITS != TW_BITS || DATA_BITS != REF_BITS)
      $fatal(1, "radar_top: this implementation assumes DATA_BITS == TW_BITS == REF_BITS");
  end

  // ---- global time --------------------------------------------------------------------------
  logic [15:0] gt;
  logic [11:0] age;
  always @(posedge clk) begin
    if (!rst_n) begin
      gt  <= '0;
      age <= '0;
    end else begin
      gt  <= gt + 16'd1;
      if (age != 12'hFFF) age <= age + 12'd1;
    end
  end

  // ---- control registers ---------------------------------------------------------------------
  logic                enable;
  logic [ALPHA_W-1:0]  alpha_q;
  logic [7:0]          sat_inc;
  logic                blk_inc;

  radar_regs u_regs (
    .clk(clk), .rst_n(rst_n),
    .s_axil_awaddr(s_axil_awaddr), .s_axil_awvalid(s_axil_awvalid), .s_axil_awready(s_axil_awready),
    .s_axil_wdata(s_axil_wdata), .s_axil_wstrb(s_axil_wstrb), .s_axil_wvalid(s_axil_wvalid), .s_axil_wready(s_axil_wready),
    .s_axil_bresp(s_axil_bresp), .s_axil_bvalid(s_axil_bvalid), .s_axil_bready(s_axil_bready),
    .s_axil_araddr(s_axil_araddr), .s_axil_arvalid(s_axil_arvalid), .s_axil_arready(s_axil_arready),
    .s_axil_rdata(s_axil_rdata), .s_axil_rresp(s_axil_rresp), .s_axil_rvalid(s_axil_rvalid), .s_axil_rready(s_axil_rready),
    .enable(enable), .alpha_q(alpha_q),
    .sat_inc(sat_inc), .blk_inc(blk_inc), .det_inc(m_det_report && m_det_valid)
  );

  // ---- input buffer / block scheduler ---------------------------------------------------------
  logic signed [DATA_BITS-1:0] x1r, x1i;
  logic                        launch;
  logic [15:0]                 wr_hop, launched, blk_done;

  radar_inbuf u_inbuf (
    .clk(clk), .rst_n(rst_n), .enable(enable), .fcnt(gt[7:0]),
    .s_valid(s_axis_tvalid), .s_ready(s_axis_tready), .s_data(s_axis_tdata),
    .xr(x1r), .xi(x1i), .launch(launch),
    .wr_hop(wr_hop), .launched(launched), .blk_done(blk_done)
  );

  // which frames carry real blocks (needed again ~3.3 frames later, at the output)
  logic [7:0] real_hist;
  always @(posedge clk) begin
    if (!rst_n)                real_hist <= '0;
    else if (gt[7:0] == 8'd0)  real_hist[gt[10:8]] <= launch;
  end

  // ---- FFT 1 -------------------------------------------------------------------------------------
  logic signed [DATA_BITS-1:0] f1r, f1i;
  logic [5:0]                  sat_f1;
  radar_fft256 #(.INV(1'b0), .T_IN(T_F1IN)) u_fft1 (
    .clk(clk), .pin0(8'(gt - 16'(T_F1IN))), .age(age),
    .xr(x1r), .xi(x1i), .yr(f1r), .yi(f1i), .sat_n(sat_f1)
  );

  // ---- multiply by the chirp spectrum, conjugate ---------------------------------------------
  logic signed [DATA_BITS-1:0] yr, yi_neg;
  logic [1:0]                  sat_ref;
  radar_refmul u_refmul (
    .clk(clk), .m(8'(gt - 16'(T_F1OUT))), .fr(f1r), .fi(f1i),
    .yr(yr), .yi_neg(yi_neg), .sat_n(sat_ref)
  );

  // ---- reorder to bit-reversed order ---------------------------------------------------------------
  logic signed [DATA_BITS-1:0] b_r, b_i;
  radar_bitrev_buf u_brb (.clk(clk), .gt(gt), .wr_r(yr), .wr_i(yi_neg), .rd_r(b_r), .rd_i(b_i));

  // ---- FFT 2 (inverse transform) ---------------------------------------------------------------------
  logic signed [DATA_BITS-1:0] z_r, z_i;
  logic [5:0]                  sat_f2;
  radar_fft256 #(.INV(1'b1), .T_IN(T_F2IN)) u_fft2 (
    .clk(clk), .pin0(8'(gt - 16'(T_F2IN))), .age(age),
    .xr(b_r), .xi(b_i), .yr(z_r), .yi(z_i), .sat_n(sat_f2)
  );

  // ---- output: conjugate, keep the clean first half of each real block ---------------------------
  localparam logic signed [DATA_BITS-1:0] MINV = {1'b1, {(DATA_BITS-1){1'b0}}};
  localparam logic signed [DATA_BITS-1:0] MAXV = {1'b0, {(DATA_BITS-1){1'b1}}};

  wire [15:0] t_out  = gt - 16'(T_F2OUT);
  wire [7:0]  n_out  = t_out[7:0];
  wire        z_neg_sat = (z_i == MINV);
`ifdef MUT_HALF
  wire        keep = (n_out >= 8'(HOP)) && real_hist[t_out[10:8]] && (int'(age) >= T_F2OUT);   // MUTATION: keep the wrong half
`else
  wire        keep = (n_out <  8'(HOP)) && real_hist[t_out[10:8]] && (int'(age) >= T_F2OUT);
`endif

  initial begin m_cmp_valid = 1'b0; m_cmp_i = '0; m_cmp_q = '0; blk_inc = 1'b0; end
  always @(posedge clk) begin
    m_cmp_valid <= keep;
    m_cmp_i     <= z_r;
`ifdef MUT_NOCONJ
    m_cmp_q     <= z_i;
`else
    m_cmp_q     <= z_neg_sat ? MAXV : -z_i;
`endif
    blk_inc     <= keep && (n_out == 8'(HOP - 1));
  end

  // ---- saturation event count ---------------------------------------------------------------------------
  wire sat_out = z_neg_sat && (int'(age) >= T_F2OUT);
  wire [1:0] sat_ref_g = (int'(age) >= T_WR0) ? sat_ref : 2'd0;
  always @(posedge clk) begin
    if (!rst_n) sat_inc <= '0;
    else        sat_inc <= 8'(sat_f1) + 8'(sat_f2) + 8'(sat_ref_g) + 8'(sat_out);
  end

  // ---- CFAR --------------------------------------------------------------------------------------------------
  radar_cfar u_cfar (
    .clk(clk), .rst_n(rst_n), .alpha_q(alpha_q),
    .v(m_cmp_valid), .zr(m_cmp_i), .zi(m_cmp_q),
    .m_v(m_mag2_valid), .m_data(m_mag2),
    .d_v(m_det_valid), .d_idx(m_det_idx), .d_flag(m_det_flag), .d_report(m_det_report)
  );
endmodule
