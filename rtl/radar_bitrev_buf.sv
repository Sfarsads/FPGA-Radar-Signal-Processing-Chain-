
module radar_bitrev_buf
  import radar_pkg::*;
(
  input  logic                        clk,
  input  logic [15:0]                 gt,
  input  logic signed [DATA_BITS-1:0] wr_r, wr_i,      // written every cycle; index = gt - T_WR0
  output logic signed [DATA_BITS-1:0] rd_r, rd_i       // registered read; position k at gt = T_RD0 + k + 1
);
  logic [2*DATA_BITS-1:0] ram [0:511];
  initial for (int i = 0; i < 512; i++) ram[i] = '0;

  wire [15:0] tw = gt - 16'(T_WR0);
  wire [15:0] tr = gt - 16'(T_RD0);
  wire [8:0]  waddr = {tw[8], tw[7:0]};
  wire [8:0]  raddr = {tr[8], bitrev8(tr[7:0])};

  logic [2*DATA_BITS-1:0] rdata;
  initial rdata = '0;
  always @(posedge clk) begin
    ram[waddr] <= {wr_i, wr_r};
    rdata      <= ram[raddr];
  end
  assign {rd_i, rd_r} = rdata;
endmodule
