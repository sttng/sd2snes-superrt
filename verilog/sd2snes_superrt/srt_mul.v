`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// srt_mul: signed 32x32 multiplier for the SuperRT engine.
//
// Delivers the low 48 bits of the product (all the fixed point formats used
// by SuperRT need: FixedMul = p[39:14] sign extended, FixedMul48 = p[45:14],
// colour multiplies = p[15:8]).
//
// a/b are expected to come straight from registers (they become the DSP input
// registers). LAT is the latency in FSM cycles (srt_engine MUL_LAT): an
// operand issued in cycle t can be consumed in cycle t+LAT.
//   LAT = 2: partial products and their carry-save sum are combinational,
//            one register stage (the same structure as srt_mulf)
//   LAT = 3: partial products registered (DSP output registers), then summed
//   LAT = 4: as 3, plus a register stage after the carry-save step
//
// The product is split into four 16/17 bit partial products so each maps to
// one 18x18 embedded multiplier.
//////////////////////////////////////////////////////////////////////////////////
module srt_mul #(
  parameter LAT = 2
)(
  input clk,
  input [31:0] a,
  input [31:0] b,
  output reg [47:0] p
);

wire [15:0] al = a[15:0];
wire [15:0] bl = b[15:0];
wire signed [15:0] ah = a[31:16];
wire signed [15:0] bh = b[31:16];

// p[47:16] = {hh, ll[31:16]} + lh + hl (mod 2^32), p[15:0] = ll[15:0].
// The three-operand sum is done carry-save: one LUT level + one carry chain.
generate if(LAT == 2) begin : g_lat2
  wire [31:0] pp_ll = al * bl;
  wire signed [32:0] pp_lh = $signed({1'b0, al}) * bh;
  wire signed [32:0] pp_hl = ah * $signed({1'b0, bl});
  wire signed [31:0] hh_full = ah * bh;
  wire [31:0] csa_x = {hh_full[15:0], pp_ll[31:16]};
  wire [31:0] csa_y = pp_lh[31:0];
  wire [31:0] csa_z = pp_hl[31:0];
  wire [31:0] csa_s = csa_x ^ csa_y ^ csa_z;
  wire [31:0] csa_c = (csa_x & csa_y) | (csa_x & csa_z) | (csa_y & csa_z);
  always @(posedge clk) begin
    p <= {csa_s + {csa_c[30:0], 1'b0}, pp_ll[15:0]};
  end
end else begin : g_lat34
  reg [31:0] pp_ll;
  reg signed [32:0] pp_lh;
  reg signed [32:0] pp_hl;
  reg [15:0] pp_hh;
  wire signed [31:0] hh_full = ah * bh;
  always @(posedge clk) begin
    pp_ll <= al * bl;
    pp_lh <= $signed({1'b0, al}) * bh;
    pp_hl <= ah * $signed({1'b0, bl});
    pp_hh <= hh_full[15:0];
  end
  wire [31:0] csa_x = {pp_hh, pp_ll[31:16]};
  wire [31:0] csa_y = pp_lh[31:0];
  wire [31:0] csa_z = pp_hl[31:0];
  wire [31:0] csa_s = csa_x ^ csa_y ^ csa_z;
  wire [31:0] csa_c = (csa_x & csa_y) | (csa_x & csa_z) | (csa_y & csa_z);
  if(LAT >= 4) begin : g_extra
    reg [31:0] s_r, c_r;
    reg [15:0] lo_r;
    always @(posedge clk) begin
      s_r <= csa_s;
      c_r <= csa_c;
      lo_r <= pp_ll[15:0];
      p <= {s_r + {c_r[30:0], 1'b0}, lo_r};
    end
  end else begin : g_noextra
    always @(posedge clk) begin
      p <= {csa_s + {csa_c[30:0], 1'b0}, pp_ll[15:0]};
    end
  end
end endgenerate

endmodule

//////////////////////////////////////////////////////////////////////////////////
// srt_mul16: signed 16x16 multiplier (one embedded multiplier), latency LAT
// as srt_mul (2: the DSP output register only).
//////////////////////////////////////////////////////////////////////////////////
module srt_mul16 #(
  parameter LAT = 2
)(
  input clk,
  input [15:0] a,
  input [15:0] b,
  output reg [31:0] p
);

generate if(LAT == 2) begin : g_lat2
  always @(posedge clk) begin
    p <= $signed(a) * $signed(b);
  end
end else if(LAT == 3) begin : g_lat3
  reg [31:0] p1;
  always @(posedge clk) begin
    p1 <= $signed(a) * $signed(b);
    p <= p1;
  end
end else begin : g_lat4
  reg [31:0] p1, p2;
  always @(posedge clk) begin
    p1 <= $signed(a) * $signed(b);
    p2 <= p1;
    p <= p2;
  end
end endgenerate

endmodule

//////////////////////////////////////////////////////////////////////////////////
// srt_mulf: signed 32x32 multiplier with a single register stage, for the
// Newton-Raphson chain (srt_engine "fast lane"). a/b must come straight from
// registers (they become the DSP input registers); the four partial products
// and their carry-save sum are combinational and registered once:
// operands registered at edge N -> p valid after edge N+1, i.e. an operand
// issued in FSM cycle t can be consumed in cycle t+2. Low 48 bits only.
// (= srt_mul with LAT = 2; kept separate so the fast lane stays at latency 2
// when the main lanes are built with MUL_LAT = 3 or 4.)
//////////////////////////////////////////////////////////////////////////////////
module srt_mulf(
  input clk,
  input [31:0] a,
  input [31:0] b,
  output reg [47:0] p
);

wire [15:0] al = a[15:0];
wire [15:0] bl = b[15:0];
wire signed [15:0] ah = a[31:16];
wire signed [15:0] bh = b[31:16];

wire [31:0] pp_ll = al * bl;
wire signed [32:0] pp_lh = $signed({1'b0, al}) * bh;
wire signed [32:0] pp_hl = ah * $signed({1'b0, bl});
wire signed [31:0] hh_full = ah * bh;

wire [31:0] csa_x = {hh_full[15:0], pp_ll[31:16]};
wire [31:0] csa_y = pp_lh[31:0];
wire [31:0] csa_z = pp_hl[31:0];
wire [31:0] csa_s = csa_x ^ csa_y ^ csa_z;
wire [31:0] csa_c = (csa_x & csa_y) | (csa_x & csa_z) | (csa_y & csa_z);

always @(posedge clk) begin
  p <= {csa_s + {csa_c[30:0], 1'b0}, pp_ll[15:0]};
end

endmodule
