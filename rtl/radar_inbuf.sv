module radar_inbuf
  import radar_pkg::*;
(
  input  logic                        clk,
  input  logic                        rst_n,
  input  logic                        enable,
  input  logic [7:0]                  fcnt,       // gt[7:0]: position inside the 256-cycle frame

  input  logic                        s_valid,
  output logic                        s_ready,
  input  logic [2*IN_BITS-1:0]        s_data,     // {Q, I}

  output logic signed [DATA_BITS-1:0] xr, xi,     // FFT1 input (position fcnt-1 of the frame)
  output logic                        launch,     // valid when fcnt == 0: this frame carries a real block

  // exposed for assertions / status
  output logic [15:0]                 wr_hop,     // hops completely written
  output logic [15:0]                 launched,   // blocks launched
  output logic [15:0]                 blk_done    // blocks whose read finished
);
  initial begin
    if (DATA_BITS < IN_BITS) $fatal(1, "radar_inbuf: DATA_BITS must be >= IN_BITS");
  end

  // ---- storage --------------------------------------------------------------------
  logic [2*IN_BITS-1:0] ram [0:511];
  logic [8:0]           waddr;
  initial for (int i = 0; i < 512; i++) ram[i] = '0;

`ifdef MUT_OVERRUN
  assign s_ready = enable;                                   // MUTATION: ignore bank-in-use rule
`else
  // hop w reuses the bank of hop w-4, which must be fully read: blk_done >= w-3
  assign s_ready = enable && ((wr_hop - blk_done) < 16'd4);
`endif
  wire acc = s_valid && s_ready;

  always @(posedge clk) begin
    if (!rst_n) begin
      waddr  <= '0;
      wr_hop <= '0;
    end else if (acc) begin
      ram[waddr] <= s_data;
      waddr      <= waddr + 9'd1;
      if (waddr[6:0] == 7'd127) wr_hop <= wr_hop + 16'd1;
    end
  end

  // ---- frame scheduler -------------------------------------------------------------
  logic       cur_real;
  logic [1:0] cur_bank;
  wire        is_k0 = (fcnt == 8'd0);
  assign launch = is_k0 && enable && ((wr_hop - launched) >= 16'd2);

  always @(posedge clk) begin
    if (!rst_n) begin
      launched <= '0;
      blk_done <= '0;
      cur_real <= 1'b0;
      cur_bank <= '0;
    end else begin
      if (is_k0) begin
        cur_real <= launch;
        cur_bank <= launched[1:0];
        if (launch) launched <= launched + 16'd1;
      end
      if (fcnt == 8'd255 && cur_real) blk_done <= blk_done + 16'd1;
    end
  end

  // ---- bit-reversed block read (registered RAM output) ------------------------------
  wire [1:0] bank_base = is_k0 ? launched[1:0] : cur_bank;
  wire [8:0] raddr     = {bank_base, 7'd0} + {1'b0, bitrev8(fcnt)};   // (128*block + position) mod 512

  logic [2*IN_BITS-1:0] rdata;
  logic                 real_d;
  initial begin rdata = '0; real_d = 1'b0; end
  always @(posedge clk) begin
    rdata  <= ram[raddr];
    real_d <= is_k0 ? launch : cur_real;
  end

  localparam int UP = DATA_BITS - IN_BITS;
  assign xr = real_d ? (DATA_BITS'($signed(rdata[IN_BITS-1:0]))        <<< UP) : '0;
  assign xi = real_d ? (DATA_BITS'($signed(rdata[2*IN_BITS-1:IN_BITS])) <<< UP) : '0;
endmodule
