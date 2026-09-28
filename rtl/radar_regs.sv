// radar_regs.sv: AXI4-Lite slave with the control / status registers.
//
//   0x00 CTRL         RW  [0] enable (reset 1)   [1] write 1 = clear the three counters (self-clearing)
module radar_regs
  import radar_pkg::*;
(
  input  logic                clk,
  input  logic                rst_n,

  input  logic [7:0]          s_axil_awaddr,
  input  logic                s_axil_awvalid,
  output logic                s_axil_awready,
  input  logic [31:0]         s_axil_wdata,
  input  logic [3:0]          s_axil_wstrb,
  input  logic                s_axil_wvalid,
  output logic                s_axil_wready,
  output logic [1:0]          s_axil_bresp,
  output logic                s_axil_bvalid,
  input  logic                s_axil_bready,
  input  logic [7:0]          s_axil_araddr,
  input  logic                s_axil_arvalid,
  output logic                s_axil_arready,
  output logic [31:0]         s_axil_rdata,
  output logic [1:0]          s_axil_rresp,
  output logic                s_axil_rvalid,
  input  logic                s_axil_rready,

  output logic                enable,
  output logic [ALPHA_W-1:0]  alpha_q,
  input  logic [7:0]          sat_inc,
  input  logic                blk_inc,
  input  logic                det_inc
);
  localparam logic [1:0] OKAY = 2'b00, SLVERR = 2'b10;
  localparam logic [31:0] ID_VALUE = 32'h52414452;

  logic [31:0] sat_cnt, blk_cnt, det_cnt;
  logic        clear_p;

  // ---- write channel ----------------------------------------------------------------------
  logic        aw_seen, w_seen;
  logic [7:0]  aw_addr;
  logic [31:0] w_data;
  logic [3:0]  w_strb;

  assign s_axil_awready = !aw_seen && !s_axil_bvalid;
  assign s_axil_wready  = !w_seen  && !s_axil_bvalid;

  always @(posedge clk) begin
    if (!rst_n) begin
      aw_seen <= 1'b0; w_seen <= 1'b0; s_axil_bvalid <= 1'b0; s_axil_bresp <= OKAY;
      aw_addr <= '0; w_data <= '0; w_strb <= '0;
      enable  <= 1'b1;
      alpha_q <= ALPHA_W'(ALPHA_Q);
      clear_p <= 1'b0;
    end else begin
      clear_p <= 1'b0;
      if (s_axil_awvalid && s_axil_awready) begin aw_seen <= 1'b1; aw_addr <= s_axil_awaddr; end
      if (s_axil_wvalid  && s_axil_wready)  begin w_seen  <= 1'b1; w_data  <= s_axil_wdata; w_strb <= s_axil_wstrb; end
      if (aw_seen && w_seen && !s_axil_bvalid) begin
        s_axil_bvalid <= 1'b1;
        s_axil_bresp  <= OKAY;
        aw_seen       <= 1'b0;
        w_seen        <= 1'b0;
        case (aw_addr[7:2])
          6'd0: begin
            if (w_strb[0]) begin
              enable  <= w_data[0];
              clear_p <= w_data[1];
            end
          end
          6'd1: begin
            if (w_strb[0]) alpha_q[7:0]  <= w_data[7:0];
            if (w_strb[1]) alpha_q[15:8] <= w_data[15:8];
          end
          6'd2, 6'd3, 6'd4, 6'd5: s_axil_bresp <= OKAY;     // read-only registers: write ignored
          default:                s_axil_bresp <= SLVERR;
        endcase
      end
      if (s_axil_bvalid && s_axil_bready) s_axil_bvalid <= 1'b0;
    end
  end

  // ---- counters ---------------------------------------------------------------------------
  always @(posedge clk) begin
    if (!rst_n || clear_p) begin
      sat_cnt <= '0; blk_cnt <= '0; det_cnt <= '0;
    end else begin
      sat_cnt <= sat_cnt + 32'(sat_inc);
      blk_cnt <= blk_cnt + 32'(blk_inc);
      det_cnt <= det_cnt + 32'(det_inc);
    end
  end

  // ---- read channel -----------------------------------------------------------------------
  assign s_axil_arready = !s_axil_rvalid;
  always @(posedge clk) begin
    if (!rst_n) begin
      s_axil_rvalid <= 1'b0; s_axil_rdata <= '0; s_axil_rresp <= OKAY;
    end else begin
      if (s_axil_arvalid && s_axil_arready) begin
        s_axil_rvalid <= 1'b1;
        s_axil_rresp  <= OKAY;
        case (s_axil_araddr[7:2])
          6'd0:    s_axil_rdata <= {30'd0, 1'b0, enable};
          6'd1:    s_axil_rdata <= {16'd0, alpha_q};
          6'd2:    s_axil_rdata <= sat_cnt;
          6'd3:    s_axil_rdata <= blk_cnt;
          6'd4:    s_axil_rdata <= det_cnt;
          6'd5:    s_axil_rdata <= ID_VALUE;
          default: begin s_axil_rdata <= '0; s_axil_rresp <= SLVERR; end
        endcase
      end
      if (s_axil_rvalid && s_axil_rready) s_axil_rvalid <= 1'b0;
    end
  end
endmodule
