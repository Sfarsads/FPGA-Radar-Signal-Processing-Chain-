// radar_pkg.sv: shared parameters, timing constants and arithmetic helpers.
// Compile with +incdir+vectors (the .do / Makefile do this).
package radar_pkg;

`include "radar_params.svh"

  // Folder holding the ROM hex files (twiddles, chirp spectrum). Override with +define+VEC_DIR="path".
`ifndef VEC_DIR
  `define VEC_DIR "vectors"
`endif
  localparam string TW_RE_FILE  = {`VEC_DIR, "/tw_re.hex"};
  localparam string TW_IM_FILE  = {`VEC_DIR, "/tw_im.hex"};
  localparam string REF_RE_FILE = {`VEC_DIR, "/ref_re.hex"};
  localparam string REF_IM_FILE = {`VEC_DIR, "/ref_im.hex"};

  // ---- derived widths ------------------------------------------------------------
  localparam int MAG2_W  = 28;     // |I|^2+|Q|^2 after MAG2_SHIFT (max 2^27)
  localparam int ALPHA_W = 16;     // CFAR multiplier register width (Q4.12)
  localparam int SUM_W   = MAG2_W + 4;   // sum of 16 training cells
  localparam int LOG2N   = 8;      // log2(N_FFT)
  localparam int NSTG    = 8;

  // ---- pipeline timing (all in clock cycles; g = global time, frame f starts at g = 256*f) ----
  localparam int LM      = 3;                       // twiddle multiply latency: ROM read + 2 multiplier stages
  localparam int L_FFT   = (N_FFT - 1) + NSTG * (LM + 1);  // FFT input frame start -> natural-order output frame start (287)
  localparam int T_F1IN  = 1;                       // FFT1 input position 0 at g = 256f + 1  (RAM read latency)
  localparam int T_F1OUT = T_F1IN + L_FFT;          // FFT1 output index 0
  localparam int T_WR0   = T_F1OUT + 4;             // ref multiply (3) + conj/negate register (1): first buffer write
  localparam int T_RD0   = T_WR0 + N_FFT;           // bit-reversed read of a frame starts one frame after its write
  localparam int T_F2IN  = T_RD0 + 1;               // FFT2 input position 0
  localparam int T_F2OUT = T_F2IN + L_FFT;          // FFT2 output index 0
  localparam int T_OUT0  = T_F2OUT + 1;             // conjugated output index 0 (registered)

  // Per-stage scaling exponent (0 or 1) of the forward / inverse FFT.
  function automatic int shift_of(input bit inv, input int s);
    return inv ? INV_SHIFTS[s] : FWD_SHIFTS[s];
  endfunction

  // Frame-position offset of FFT stage s: sum of the latencies of stages 0..s-1.
  function automatic int stage_lat_sum(input int s);
    return ((1 << s) - 1) + s * (LM + 1);
  endfunction

  // ---- arithmetic helpers --------------------------------------------------------
  function automatic logic [7:0] bitrev8(input logic [7:0] v);
    logic [7:0] r;
    for (int b = 0; b < 8; b++) r[7-b] = v[b];
`ifdef MUT_BITREV
    r[0] = v[0];           // MUTATION: wrong bit reversal on one bit
`endif
    return r;
  endfunction

  // Saturating conversion of a wide signed value to DATA_BITS.
  function automatic logic signed [DATA_BITS-1:0] sat_d(input logic signed [47:0] x);
    localparam logic signed [47:0] HI =  (48'sd1 <<< (DATA_BITS-1)) - 48'sd1;
    localparam logic signed [47:0] LO = -(48'sd1 <<< (DATA_BITS-1));
`ifdef MUT_NOSAT
    return x[DATA_BITS-1:0];   // MUTATION: wrap instead of saturate
`else
    if (x > HI)      return HI[DATA_BITS-1:0];
    else if (x < LO) return LO[DATA_BITS-1:0];
    else             return x[DATA_BITS-1:0];
`endif
  endfunction

  // 1 when sat_d would clip.
  function automatic logic sat_f(input logic signed [47:0] x);
    localparam logic signed [47:0] HI =  (48'sd1 <<< (DATA_BITS-1)) - 48'sd1;
    localparam logic signed [47:0] LO = -(48'sd1 <<< (DATA_BITS-1));
    return (x > HI) || (x < LO);
  endfunction

  // Arithmetic shift right by s with round-half-up (matches rshift_round in radar_golden.py).
  function automatic logic signed [47:0] rshr(input logic signed [47:0] x, input int s);
    if (s <= 0) return x;
`ifdef MUT_TRUNC
    return x >>> s;            // MUTATION: truncate instead of round
`else
    return (x + (48'sd1 <<< (s-1))) >>> s;
`endif
  endfunction

endpackage
