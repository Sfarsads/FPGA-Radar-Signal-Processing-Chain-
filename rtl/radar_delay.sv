// radar_delay.sv: fixed delay of exactly D clock cycles (out(t+D) = in(t)), no enable.
// D = 1 is a register; D >= 2 is a circular buffer of D-1 words (read-old-data RAM, maps to M9K).
// Memory is cleared at time zero so simulation never sees X.
module radar_delay #(
  parameter int W = 36,
  parameter int D = 1
) (
  input  logic         clk,
  input  logic [W-1:0] din,
  output logic [W-1:0] dout
);
  generate
    if (D == 1) begin : g_reg
      initial dout = '0;
      always @(posedge clk) dout <= din;
    end else begin : g_ram
      localparam int N  = D - 1;
      localparam int AW = (N > 1) ? $clog2(N) : 1;
      logic [W-1:0] mem [0:N-1];
      logic [AW-1:0] ptr;
      initial begin
        for (int i = 0; i < N; i++) mem[i] = '0;
        ptr  = '0;
        dout = '0;
      end
      always @(posedge clk) begin
        dout     <= mem[ptr];
        mem[ptr] <= din;
        ptr      <= (ptr == AW'(N-1)) ? '0 : ptr + 1'b1;
      end
    end
  endgenerate
endmodule
