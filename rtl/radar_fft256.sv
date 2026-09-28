module radar_fft256
  import radar_pkg::*;
#(
  parameter bit INV  = 1'b0,
  parameter int T_IN = T_F1IN      // global time of input position 0 in frame 0 (for the warm-up gate)
) (
  input  logic                        clk,
  input  logic [7:0]                  pin0,
  input  logic [11:0]                 age,      // clocks since reset, saturating
  input  logic signed [DATA_BITS-1:0] xr, xi,
  output logic signed [DATA_BITS-1:0] yr, yi,
  output logic [5:0]                  sat_n     // saturation events this cycle (all stages)
);
  logic signed [DATA_BITS-1:0] br [0:NSTG];
  logic signed [DATA_BITS-1:0] bi [0:NSTG];
  logic [2:0]                  sn [0:NSTG-1];

  assign br[0] = xr;
  assign bi[0] = xi;

  generate
    for (genvar s = 0; s < NSTG; s++) begin : g_stage
      localparam int LATSUM = stage_lat_sum(s);
      wire [7:0] pin_s = pin0 - 8'(LATSUM);
      wire       warm  = (int'(age) >= T_IN + LATSUM + LM);
      radar_fft_stage #(.S(s), .SH(shift_of(INV, s))) u_stage (
        .clk(clk), .pin(pin_s), .warm(warm),
        .xr(br[s]), .xi(bi[s]), .yr(br[s+1]), .yi(bi[s+1]), .sat_n(sn[s])
      );
    end
  endgenerate

  assign yr = br[NSTG];
  assign yi = bi[NSTG];
  always_comb begin
    sat_n = '0;
    for (int s = 0; s < NSTG; s++) sat_n = sat_n + 6'(sn[s]);
  end
endmodule
