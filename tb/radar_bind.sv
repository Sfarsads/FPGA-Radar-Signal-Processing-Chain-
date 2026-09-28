// radar_bind.sv: attaches the assertion module to every radar_top instance.
bind radar_top radar_sva u_sva (
  .clk(clk), .rst_n(rst_n),
  .s_axis_tvalid(s_axis_tvalid), .s_axis_tready(s_axis_tready), .s_axis_tdata(s_axis_tdata),
  .m_cmp_valid(m_cmp_valid), .m_cmp_i(m_cmp_i), .m_cmp_q(m_cmp_q),
  .m_mag2_valid(m_mag2_valid), .m_mag2(m_mag2),
  .m_det_valid(m_det_valid), .m_det_idx(m_det_idx), .m_det_flag(m_det_flag), .m_det_report(m_det_report),
  .s_axil_awaddr(s_axil_awaddr), .s_axil_awvalid(s_axil_awvalid), .s_axil_awready(s_axil_awready),
  .s_axil_wdata(s_axil_wdata), .s_axil_wvalid(s_axil_wvalid), .s_axil_wready(s_axil_wready),
  .s_axil_bresp(s_axil_bresp), .s_axil_bvalid(s_axil_bvalid), .s_axil_bready(s_axil_bready),
  .s_axil_araddr(s_axil_araddr), .s_axil_arvalid(s_axil_arvalid), .s_axil_arready(s_axil_arready),
  .s_axil_rdata(s_axil_rdata), .s_axil_rresp(s_axil_rresp), .s_axil_rvalid(s_axil_rvalid), .s_axil_rready(s_axil_rready),
  .enable(enable), .age(age), .wr_hop(wr_hop), .launched(launched), .blk_done(blk_done), .sat_inc(sat_inc)
);
