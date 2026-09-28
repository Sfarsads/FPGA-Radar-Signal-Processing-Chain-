// radar_refmul.sv: multiply the FFT1 output by the stored chirp spectrum conj(FFT(chirp)),
// Output is registered, 4 cycles after the input sample.
module radar_refmul
  import radar_pkg::*;
(
  input  logic                        clk,
  input  logic [7:0]                  m,          
  input  logic signed [DATA_BITS-1:0] fr, fi,
  output logic signed [DATA_BITS-1:0] yr, yi_neg,
  output logic [1:0]                  sat_n      
);
  logic signed [REF_BITS-1:0] rom_r [0:255];
  logic signed [REF_BITS-1:0] rom_i [0:255];
  initial begin
    $readmemh(REF_RE_FILE, rom_r);
    $readmemh(REF_IM_FILE, rom_i);
  end

  logic signed [REF_BITS-1:0]  h_r, h_i;
  logic signed [DATA_BITS-1:0] f1r, f1i;
  initial begin h_r = '0; h_i = '0; f1r = '0; f1i = '0; end
  always @(posedge clk) begin
    h_r <= rom_r[m];
    h_i <= rom_i[m];
    f1r <= fr;
    f1i <= fi;
  end

  logic signed [DATA_BITS-1:0] pr, pi;
  logic                        sp_r, sp_i;
  radar_cmul #(.XW(DATA_BITS), .WW(REF_BITS)) u_cmul (
    .clk(clk), .xr(f1r), .xi(f1i), .wr(h_r), .wi(h_i),
    .tr(pr), .ti(pi), .sat_r(sp_r), .sat_i(sp_i)
  );

  localparam logic signed [DATA_BITS-1:0] MINV = {1'b1, {(DATA_BITS-1){1'b0}}};
  localparam logic signed [DATA_BITS-1:0] MAXV = {1'b0, {(DATA_BITS-1){1'b1}}};
  wire neg_sat = (pi == MINV);
  initial begin yr = '0; yi_neg = '0; sat_n = '0; end
  always @(posedge clk) begin
    yr <= pr;
`ifdef MUT_NOCONJ
    yi_neg <= pi;                                     
`else
    yi_neg <= neg_sat ? MAXV : -pi;
`endif
    sat_n <= 2'(sp_r) + 2'(sp_i) + 2'(neg_sat);
  end
endmodule
