module radar_fft_stage
  import radar_pkg::*;
#(
  parameter int S  = 0,
  parameter int SH = 0
) (
  input  logic                        clk,
  input  logic [7:0]                  pin,
  input  logic                        warm,     // count saturation events only once real data is flowing
  input  logic signed [DATA_BITS-1:0] xr, xi,
  output logic signed [DATA_BITS-1:0] yr, yi,
  output logic [2:0]                  sat_n
);
  localparam int D = 1 << S;

  // ---- twiddle ROM (registered read) ----------------------------------------------
  logic signed [TW_BITS-1:0] rom_r [0:127];
  logic signed [TW_BITS-1:0] rom_i [0:127];
  initial begin
    $readmemh(TW_RE_FILE, rom_r);
    $readmemh(TW_IM_FILE, rom_i);
  end

  wire [7:0] jm = pin & 8'(D - 1);                 // index inside the half-group
`ifdef MUT_TWIDX
  wire [6:0] tw_idx = 7'((jm << (7 - S)) + 8'd1);  // MUTATION: twiddle index off by one
`else
  wire [6:0] tw_idx = 7'(jm << (7 - S));
`endif

  logic signed [TW_BITS-1:0] w_r, w_i;
  logic signed [DATA_BITS-1:0] x1r, x1i, x2r, x2i, x3r, x3i;
  initial begin
    w_r = '0; w_i = '0; x1r = '0; x1i = '0; x2r = '0; x2i = '0; x3r = '0; x3i = '0;
  end
  always @(posedge clk) begin
    w_r <= rom_r[tw_idx];
    w_i <= rom_i[tw_idx];
    x1r <= xr;  x1i <= xi;
    x2r <= x1r; x2i <= x1i;
    x3r <= x2r; x3i <= x2i;
  end

  // ---- twiddle multiply (2 cycles) -> t aligned with x3 ---------------------------
  logic signed [DATA_BITS-1:0] tr, ti;
  logic                        sat_tr, sat_ti;
  radar_cmul #(.XW(DATA_BITS), .WW(TW_BITS)) u_cmul (
    .clk(clk), .xr(x1r), .xi(x1i), .wr(w_r), .wi(w_i),
    .tr(tr), .ti(ti), .sat_r(sat_tr), .sat_i(sat_ti)
  );

  // ---- butterfly with feedback delay ---------------------------------------------
  wire [7:0] pin_core = pin - 8'(LM);
  wire       active   = pin_core[S];               // 1 while the "b" half of the group is at the core

  logic [2*DATA_BITS-1:0] fb, dl;
  logic signed [DATA_BITS-1:0] ar, ai;
  assign {ai, ar} = dl;

  logic signed [47:0] sum_r, sum_i, dif_r, dif_i;
  logic signed [DATA_BITS-1:0] top_r, top_i, bot_r, bot_i;
  always_comb begin
    sum_r = rshr(48'(ar) + 48'(tr), SH);
    sum_i = rshr(48'(ai) + 48'(ti), SH);
    dif_r = rshr(48'(ar) - 48'(tr), SH);
    dif_i = rshr(48'(ai) - 48'(ti), SH);
    top_r = sat_d(sum_r);
    top_i = sat_d(sum_i);
    bot_r = sat_d(dif_r);
    bot_i = sat_d(dif_i);
    fb    = active ? {bot_i, bot_r} : {x3i, x3r};
  end

  radar_delay #(.W(2*DATA_BITS), .D(D)) u_dl (.clk(clk), .din(fb), .dout(dl));

  initial begin yr = '0; yi = '0; sat_n = '0; end
  always @(posedge clk) begin
    yr <= active ? top_r : ar;
    yi <= active ? top_i : ai;
    sat_n <= (active && warm)
           ? 3'(sat_tr) + 3'(sat_ti) + 3'(sat_f(sum_r)) + 3'(sat_f(sum_i)) + 3'(sat_f(dif_r)) + 3'(sat_f(dif_i))
           : 3'd0;
  end
endmodule
