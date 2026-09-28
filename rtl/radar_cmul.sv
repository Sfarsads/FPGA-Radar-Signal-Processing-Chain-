
module radar_cmul
  import radar_pkg::*;
#(
  parameter int XW = DATA_BITS,
  parameter int WW = TW_BITS
) (
  input  logic                    clk,
  input  logic signed [XW-1:0]    xr, xi,
  input  logic signed [WW-1:0]    wr, wi,
  output logic signed [XW-1:0]    tr, ti,
  output logic                    sat_r, sat_i
);
  // stage 1: four products
  logic signed [XW+WW-1:0] p_rr, p_ii, p_ri, p_ir;
  always @(posedge clk) begin
    p_rr <= xr * wr;
    p_ii <= xi * wi;
    p_ri <= xr * wi;
    p_ir <= xi * wr;
  end

  // stage 2: combine, round, saturate
  logic signed [47:0] sum_r, sum_i, rnd_r, rnd_i;
  always_comb begin
`ifdef MUT_CMULRND
    sum_r = 48'(p_rr) - 48'(p_ii);                         // MUTATION: no rounding constant
    sum_i = 48'(p_ri) + 48'(p_ir);
    rnd_r = sum_r >>> (WW-1);
    rnd_i = sum_i >>> (WW-1);
`else
    sum_r = 48'(p_rr) - 48'(p_ii);
    sum_i = 48'(p_ri) + 48'(p_ir);
    rnd_r = rshr(sum_r, WW-1);
    rnd_i = rshr(sum_i, WW-1);
`endif
  end

  initial begin tr = '0; ti = '0; sat_r = 1'b0; sat_i = 1'b0; end
  always @(posedge clk) begin
    tr    <= sat_d(rnd_r);
    ti    <= sat_d(rnd_i);
    sat_r <= sat_f(rnd_r);
    sat_i <= sat_f(rnd_i);
  end
endmodule
