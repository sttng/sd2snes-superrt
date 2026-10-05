`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// srt_engine: SuperRT ray tracer in hardware, sized for the mk3's EP4CE15.
//
// SuperRT (c) 2021 Ben Carter / Shironeko Labs, MIT licensed.
// https://github.com/ShironekoBen/superrt
//
// The original chip runs three deeply pipelined ray engines (one command list
// instruction per clock each). That needs about four times the multipliers
// and logic of the EP4CE15, so this is a single sequential engine instead:
// one state machine that walks exactly the same algorithm as the software
// model in src/srt_render.c (pixel loop, ray phases, command list interpreter,
// intersection tests, shading) with three shared pipelined 32x32 multipliers
// and three 16x16 ones. Results are bit-identical to srt_render.c, which is
// bit-identical to the original RTL. Work is skipped or cached only where the
// result provably stays the same (see the plane reciprocal shortcut, the
// reciprocal / plane point / 1/radius caches and isect_miss).
//
// Start: one 'start' pulse latches the frame parameters and renders the 200 x
// 160 frame in raster order. Every pixel (RGB555, R in bits 4:0) is written to
// pix_data with a pix_we pulse; the engine stalls while pix_full is set.
// The command list is read through cmd_addr/cmd_q (synchronous RAM, the
// address is registered by the RAM: cmd_q in cycle t+1 = mem[cmd_addr in t]).
//
// Multiplier latency (MUL_LAT) is 3: operands issued in cycle t are consumed
// in cycle t+3. All waits are written relative to MUL_LAT; MUL_LAT = 4 adds a
// pipeline stage to the multipliers (srt_mul EXTRA) if timing needs it.
//////////////////////////////////////////////////////////////////////////////////
module srt_engine #(
  parameter MUL_LAT = 3
)(
  input clk,
  input start,
  input abort,

  // frame parameters (sampled on 'start')
  input [31:0] i_sx, input [31:0] i_sy, input [31:0] i_sz,
  input [15:0] i_dx, input [15:0] i_dy, input [15:0] i_dz,
  input [15:0] i_xsx, input [15:0] i_xsy, input [15:0] i_xsz,
  input [15:0] i_ysx, input [15:0] i_ysy, input [15:0] i_ysz,
  input [15:0] i_lx, input [15:0] i_ly, input [15:0] i_lz,
  input i_half,                       // half horizontal resolution: even pixels only

  // command buffer read port
  output [8:0] cmd_addr,
  input [63:0] cmd_q,

  // pixel output
  output reg [14:0] pix_data = 15'h0,
  output reg pix_we = 1'b0,
  input pix_full,

  output busy,
  output reg [31:0] cycles = 32'h0
);

localparam [31:0] INF = 32'h7FFFFFFF;
localparam [31:0] MIN_TRACE = 32'hFFF60000;
localparam [31:0] MAX_TRACE = 32'h000A0000;
localparam [1:0] NR_RCP = 2'd0, NR_SQRT = 2'd1, NR_RSQ = 2'd2;

// opcodes
localparam [5:0]
  I_NOP = 6'd0, I_Sphere = 6'd1, I_Plane = 6'd2, I_SphereSub = 6'd3, I_PlaneSub = 6'd4,
  I_SphereAnd = 6'd5, I_PlaneAnd = 6'd6, I_AABB = 6'd7, I_AABBSub = 6'd8, I_AABBAnd = 6'd9,
  I_RegisterHit = 6'd10, I_RegisterHitNoReset = 6'd11, I_Checkerboard = 6'd12,
  I_ResetHitState = 6'd13, I_Jump = 6'd14, I_ResetHitStateAndJump = 6'd15,
  I_Origin = 6'd16, I_Start = 6'd17, I_End = 6'd18;

// states
localparam [6:0]
  S_IDLE = 7'd0,
  S_PIX0 = 7'd1, S_PIX1 = 7'd2, S_PIX2 = 7'd3, S_PIX3 = 7'd4,
  S_PH0 = 7'd5, S_PH1 = 7'd6, S_DEC = 7'd8,
  S_SPH8B = 7'd9, S_SPH1 = 7'd10, S_SPH2 = 7'd11, S_SPH3 = 7'd12, S_SPH4 = 7'd13,
  S_SPH5 = 7'd14, S_SPH6 = 7'd15, S_SPH8 = 7'd16, S_SPH9 = 7'd17,
  S_PLM0 = 7'd18, S_PLM1 = 7'd19, S_PLA = 7'd20, S_PLB = 7'd21,
  S_SKY0 = 7'd22, S_PLD = 7'd23, S_PLF = 7'd25, S_PL10 = 7'd27, S_PL11 = 7'd28,
  S_BX0 = 7'd29, S_BX1 = 7'd30, S_BX2 = 7'd31, S_BX4 = 7'd32, S_BX5 = 7'd33, S_BX6 = 7'd34,
  S_COMB = 7'd35, S_SN0 = 7'd36, S_SN1 = 7'd37, S_SN2 = 7'd38, S_SN3 = 7'd39,
  S_HP0 = 7'd40, S_HP1 = 7'd41, S_CHK = 7'd42, S_RH = 7'd43, S_PHEND = 7'd44,
  S_FIN0 = 7'd45, S_FIN1 = 7'd46, S_FIN3A = 7'd47, S_FIN3 = 7'd48, S_FIN4 = 7'd49,
  S_FIN5 = 7'd50, S_FIN6 = 7'd51, S_FIN7 = 7'd52, S_FIN8 = 7'd53, S_FIN9 = 7'd54,
  S_SKY1 = 7'd55, S_SKY2 = 7'd56, S_SKY3 = 7'd57, S_NEXT = 7'd58,
  S_OUT0 = 7'd59, S_OUT1 = 7'd60, S_OUT2 = 7'd61, S_OUT3 = 7'd62, S_OUT4 = 7'd63,
  S_NR0 = 7'd64, S_NRA = 7'd65, S_NR2 = 7'd66, S_NR3 = 7'd67, S_NR4 = 7'd68, S_NR5 = 7'd69,
  S_NF0 = 7'd70, S_NFA = 7'd71, S_NF2 = 7'd72, S_NF3 = 7'd73, S_NF4 = 7'd74, S_NF5 = 7'd75;

// ---------------------------------------------------------------------------
// helpers
// ---------------------------------------------------------------------------
function [31:0] f40;  // FixedMul: 40 bit product >>> 14
  input [47:0] p;
  f40 = {{6{p[39]}}, p[39:14]};
endfunction

function [31:0] f48;  // FixedMul48 / FixedMul16x16: 48 bit product >>> 14
  input [47:0] p;
  f48 = p[45:14];
endfunction

function [31:0] sx16;
  input [15:0] v;
  sx16 = {{16{v[15]}}, v};
endfunction

function [31:0] cf8_7;
  input [14:0] v;
  cf8_7 = {{10{v[14]}}, v, 7'b0};
endfunction

function [31:0] cf4_7;
  input [10:0] v;
  cf4_7 = {{14{v[10]}}, v, 7'b0};
endfunction

function [15:0] cf2_10;
  input [11:0] v;
  cf2_10 = {v, 4'b0};
endfunction

function [31:0] cf8_12;
  input [19:0] v;
  cf8_12 = {{10{v[19]}}, v, 2'b0};
endfunction

function [31:0] cf8_1;
  input [8:0] v;
  cf8_1 = {{10{v[8]}}, v, 13'b0};
endfunction

function [7:0] cadd;
  input [7:0] x;
  input [7:0] y;
  reg [8:0] s;
  begin
    s = {1'b0, x} + {1'b0, y};
    cadd = s[8] ? 8'hFF : s[7:0];
  end
endfunction

function [15:0] axn;  // AABB face normal: +/-1.0 on one axis
  input dsign;
  input neg;
  axn = (dsign ^ neg) ? 16'hC000 : 16'h4000;
endfunction

function dez;  // direction component is effectively zero
  input [15:0] d;
  dez = (d[15:8] == 8'h00) || (d[15:8] == 8'hFF);
endfunction

function pl_shortcut;  // 8 <= dn < 2^18 (signed), as bit tests instead of compares
  input [31:0] dn;
  pl_shortcut = (dn[31:18] == 14'd0) && (dn[17:3] != 15'd0);
endfunction

function slt;  // signed 32 bit a < b
  input [31:0] a;
  input [31:0] b;
  slt = $signed(a) < $signed(b);
endfunction

// first guess for 1/sqrt(x), by highest set bit in 30..1
function [31:0] seed_tab;
  input [4:0] i;
  case(i)
    5'd1:  seed_tab = 32'h001279a7; 5'd2:  seed_tab = 32'h000d105e;
    5'd3:  seed_tab = 32'h00093cd3; 5'd4:  seed_tab = 32'h0006882f;
    5'd5:  seed_tab = 32'h00049e69; 5'd6:  seed_tab = 32'h00034417;
    5'd7:  seed_tab = 32'h00024f34; 5'd8:  seed_tab = 32'h0001a20b;
    5'd9:  seed_tab = 32'h0001279a; 5'd10: seed_tab = 32'h0000d105;
    5'd11: seed_tab = 32'h000093cd; 5'd12: seed_tab = 32'h00006882;
    5'd13: seed_tab = 32'h000049e6; 5'd14: seed_tab = 32'h00003441;
    5'd15: seed_tab = 32'h000024f3; 5'd16: seed_tab = 32'h00001a20;
    5'd17: seed_tab = 32'h00001279; 5'd18: seed_tab = 32'h00000d10;
    5'd19: seed_tab = 32'h0000093c; 5'd20: seed_tab = 32'h00000688;
    5'd21: seed_tab = 32'h0000049e; 5'd22: seed_tab = 32'h00000344;
    5'd23: seed_tab = 32'h0000024f; 5'd24: seed_tab = 32'h000001a2;
    5'd25: seed_tab = 32'h00000127; 5'd26: seed_tab = 32'h000000d1;
    5'd27: seed_tab = 32'h00000093; 5'd28: seed_tab = 32'h00000068;
    5'd29: seed_tab = 32'h00000049; 5'd30: seed_tab = 32'h00000034;
    default: seed_tab = 32'h00200000;
  endcase
endfunction

// FixedMul(seed, seed): the first Newton-Raphson multiply, precomputed
function [31:0] seed_sq;
  input [4:0] i;
  case(i)
    5'd1: seed_sq = 32'h0155552d;
    5'd2: seed_sq = 32'hfeaaaa5f;
    5'd3: seed_sq = 32'h01555526;
    5'd4: seed_sq = 32'h00aaaa97;
    5'd5: seed_sq = 32'h00555537;
    5'd6: seed_sq = 32'h002aaa98;
    5'd7: seed_sq = 32'h00155544;
    5'd8: seed_sq = 32'h000aaa9f;
    5'd9: seed_sq = 32'h00055551;
    5'd10: seed_sq = 32'h0002aaa4;
    5'd11: seed_sq = 32'h00015554;
    5'd12: seed_sq = 32'h0000aaa7;
    5'd13: seed_sq = 32'h00005553;
    5'd14: seed_sq = 32'h00002aa9;
    5'd15: seed_sq = 32'h00001554;
    5'd16: seed_sq = 32'h00000aaa;
    5'd17: seed_sq = 32'h00000554;
    5'd18: seed_sq = 32'h000002aa;
    5'd19: seed_sq = 32'h00000155;
    5'd20: seed_sq = 32'h000000aa;
    5'd21: seed_sq = 32'h00000055;
    5'd22: seed_sq = 32'h0000002a;
    5'd23: seed_sq = 32'h00000015;
    5'd24: seed_sq = 32'h0000000a;
    5'd25: seed_sq = 32'h00000005;
    5'd26: seed_sq = 32'h00000002;
    5'd27: seed_sq = 32'h00000001;
    5'd28: seed_sq = 32'h00000000;
    5'd29: seed_sq = 32'h00000000;
    5'd30: seed_sq = 32'h00000000;
    default: seed_sq = 32'h00000000;
  endcase
endfunction

function [4:0] hibit;  // highest set bit among 30..1, 0 if none
  input [31:0] v;
  integer i;
  begin
    hibit = 5'd0;
    for(i = 1; i <= 30; i = i + 1)
      if(v[i]) hibit = i[4:0];
  end
endfunction

// ---------------------------------------------------------------------------
// multipliers
// ---------------------------------------------------------------------------
reg [31:0] ma0 = 0, mb0 = 0, ma1 = 0, mb1 = 0, ma2 = 0, mb2 = 0;
wire [47:0] q0, q1, q2;
srt_mul #(.EXTRA(MUL_LAT - 3)) mul0(.clk(clk), .a(ma0), .b(mb0), .p(q0));
srt_mul #(.EXTRA(MUL_LAT - 3)) mul1(.clk(clk), .a(ma1), .b(mb1), .p(q1));
srt_mul #(.EXTRA(MUL_LAT - 3)) mul2(.clk(clk), .a(ma2), .b(mb2), .p(q2));

// three 16x16 lanes (plane denominators)
reg [15:0] mc0 = 0, md0 = 0, mc1 = 0, md1 = 0, mc2 = 0, md2 = 0;
wire [31:0] r0, r1, r2;
srt_mul16 #(.EXTRA(MUL_LAT - 3)) mul3(.clk(clk), .a(mc0), .b(md0), .p(r0));
srt_mul16 #(.EXTRA(MUL_LAT - 3)) mul4(.clk(clk), .a(mc1), .b(md1), .p(r1));
srt_mul16 #(.EXTRA(MUL_LAT - 3)) mul5(.clk(clk), .a(mc2), .b(md2), .p(r2));
// Newton-Raphson fast lane (one value): single register stage, an operand
// issued in cycle t is consumed in cycle t+2 (srt_mulf)
reg [31:0] fa = 0, fb = 0;
wire [47:0] fq;
srt_mulf mulf(.clk(clk), .a(fa), .b(fb), .p(fq));
wire [31:0] fr = {{6{fq[39]}}, fq[39:14]};       // f40

wire [31:0] c0 = {{14{r0[31]}}, r0[31:14]}, c1 = {{14{r1[31]}}, r1[31:14]}, c2 = {{14{r2[31]}}, r2[31:14]};
// three more 16x16 lanes: plane N.D (issued in S_DEC / S_PLA), so that the
// 32x32 lanes can take the plane dot product at the same time
reg [15:0] me0 = 0, mf0 = 0, me1 = 0, mf1 = 0, me2 = 0, mf2 = 0;
wire [31:0] r3, r4, r5;
srt_mul16 #(.EXTRA(MUL_LAT - 3)) mul6(.clk(clk), .a(me0), .b(mf0), .p(r3));
srt_mul16 #(.EXTRA(MUL_LAT - 3)) mul7(.clk(clk), .a(me1), .b(mf1), .p(r4));
srt_mul16 #(.EXTRA(MUL_LAT - 3)) mul8(.clk(clk), .a(me2), .b(mf2), .p(r5));
wire [31:0] c3 = {{14{r3[31]}}, r3[31:14]}, c4 = {{14{r4[31]}}, r4[31:14]}, c5 = {{14{r5[31]}}, r5[31:14]};

wire [31:0] a0 = f40(q0), a1 = f40(q1), a2 = f40(q2);
wire [31:0] b0 = f48(q0), b1 = f48(q1), b2 = f48(q2);
// three-operand sums carry-save: one LUT level + one carry chain
function [31:0] add3;
  input [31:0] x, y, z;
  reg [31:0] cs, cc;
  begin
    cs = x ^ y ^ z;
    cc = (x & y) | (x & z) | (y & z);
    add3 = cs + {cc[30:0], 1'b0};
  end
endfunction
wire [31:0] sumA = add3(a0, a1, a2);
wire [31:0] sumB = add3(b0, b1, b2);
wire [31:0] sumC = add3(c0, c1, c2);
wire [31:0] sumD = add3(c3, c4, c5);         // plane N.D (== FixedMul48 of the 16 bit operands)
wire [15:0] sum16 = sumB[15:0];

// ---------------------------------------------------------------------------
// state
// ---------------------------------------------------------------------------
reg [6:0] state = S_IDLE;
reg [1:0] wcnt = 2'd0;
assign busy = (state != S_IDLE);

// frame parameters
reg [31:0] P_sx = 0, P_sy = 0, P_sz = 0;
reg [15:0] P_xsx = 0, P_xsy = 0, P_xsz = 0, P_ysx = 0, P_ysy = 0, P_ysz = 0;
reg [15:0] Lx = 0, Ly = 0, Lz = 0;
reg P_half = 1'b0;

// pixel loop
reg [7:0] px = 0, py = 0;
reg [15:0] lin_x = 0, lin_y = 0, lin_z = 0;   // direction at the start of the line
reg [15:0] cdx = 0, cdy = 0, cdz = 0;         // camera ray direction

// ray engine
reg [15:0] pdx = 0, pdy = 0, pdz = 0;         // normalised primary direction
reg [31:0] rsx = 0, rsy = 0, rsz = 0;         // current ray
reg [15:0] rdx = 0, rdy = 0, rdz = 0;
reg [1:0] phase = 0;
reg p_hit = 0, p_shadow = 0, s_hit = 0, s_shadow = 0;
reg [31:0] p_x = 0, p_y = 0, p_z = 0, s_x = 0, s_y = 0, s_z = 0;
reg [15:0] p_nx = 0, p_ny = 0, p_nz = 0, s_nx = 0, s_ny = 0, s_nz = 0;
reg [15:0] p_albedo = 0, s_albedo = 0;
reg [7:0] p_refl = 0;
reg [7:0] pr = 0, pg = 0, pb = 0, sr = 0, sg = 0, sb = 0;
reg [15:0] bse = 0;
reg [15:0] sum16_r = 0;                       // shading dot product, registered (timing)                           // N.L (also the phase's ndl)
reg [15:0] sdx = 0, sdy = 0, sdz = 0;         // reflected primary direction
reg [7:0] spec = 0;
reg [7:0] ct_r = 0, ct_g = 0, ct_b = 0;

// execution engine
reg [31:0] sx = 0, sy = 0, sz = 0;
reg [15:0] dx = 0, dy = 0, dz = 0;
reg [31:0] rcpx = 0, rcpy = 0, rcpz = 0;
reg shadow = 0;
reg [31:0] ox = 0, oy = 0, oz = 0;
reg [31:0] sox = 0, soy = 0, soz = 0;         // ray start - origin
reg [31:0] he = 0, hx = 0;                    // hit entry / exit
reg [15:0] hnx = 0, hny = 0, hnz = 0;
reg orh = 0;                                  // object registered hit
reg [31:0] hp_depth = 0;
reg hp_pend = 0;
reg [31:0] hpx = 0, hpy = 0, hpz = 0;
reg [6:0] hp_ret = S_IDLE;
reg reg_hit = 0;
reg [31:0] reg_depth = 0;
reg [31:0] reg_x = 0, reg_y = 0, reg_z = 0;
reg [15:0] reg_nx = 0, reg_ny = 0, reg_nz = 0;
reg [15:0] reg_albedo = 0;
reg [7:0] reg_refl = 0;
reg [15:0] pc = 0;
reg [16:0] icount = 0;
reg [63:0] w = 0;


// intersection
reg [1:0] kind = 0;                           // 0 sphere, 1 plane, 2 aabb
reg [31:0] e_entry = 0, e_exit = 0;
reg [15:0] enx = 0, eny = 0, enz = 0, xnx = 0, xny = 0, xnz = 0;
reg [31:0] objx = 0, objy = 0, objz = 0, rad = 0;
reg insd = 0;
reg [31:0] tx = 0, ty = 0, tz = 0;            // oc / plane delta / sphere normal vector / aabb t0p
reg [31:0] ux = 0, uy = 0, uz = 0;            // aabb t1p
reg [31:0] vx = 0, vy = 0, vz = 0;            // aabb tmin
reg [31:0] zx = 0, zy = 0, zz = 0;            // aabb t0 / tmax
reg [31:0] cpa = 0, dsq = 0, rsq = 0;         // sphere; plane: dot / denom / dn
reg [31:0] dnp = 0, dnn = 0;
reg pl_neg = 0, pl_rneg = 0, pxc_hit = 0;
reg [15:0] nnx = 0, nny = 0, nnz = 0;         // plane normal
reg [31:0] sn_dep = 0;
reg sn_neg = 0;
reg [31:0] ri_rad = 0, ri_val = 0;            // 1 entry cache for 1/radius
reg ri_valid = 0;

// Cache of plane reciprocals: rcp(-dn) for the plane denominators dn, 256
// entries of {valid, dn, rcp}, direct mapped. Both candidate denominators
// (ray outside / inside the plane) are looked up in S_PLD, addressed straight
// from the multiplier sums, while the dot product is being computed.
// (Shadow rays all share one direction, so their plane denominators repeat.)
function [7:0] rc_hash;
  input [31:0] v;
  rc_hash = v[7:0] ^ v[15:8] ^ v[23:16];
endfunction
reg [64:0] rc_mem [0:255];
reg [64:0] rc_qa = 0, rc_qb = 0;
reg rc_we = 0;
reg [7:0] rc_wa = 0;
reg [64:0] rc_wd = 0;

integer rc_i;
initial for(rc_i = 0; rc_i < 256; rc_i = rc_i + 1) rc_mem[rc_i] = 65'd0;
// true dual port RAM: port A reads, port B reads or writes
wire [7:0] rc_aa = rc_hash(sumD);                   // only used when read in S_PLD
wire [7:0] rc_ab = rc_we ? rc_wa : rc_hash(sumC);
always @(posedge clk) rc_qa <= rc_mem[rc_aa];
always @(posedge clk) begin
  if(rc_we) begin
    rc_mem[rc_ab] <= rc_wd;
    rc_qb <= rc_wd;
  end else begin
    rc_qb <= rc_mem[rc_ab];
  end
end
wire rc_hit_a = rc_qa[64] && (rc_qa[63:32] == dnp);
wire rc_hit_b = rc_qb[64] && (rc_qb[63:32] == dnn);

// Per instruction cache of the point on each plane (normal * distance, a
// constant of the instruction), tagged with the instruction's operand bits.
// Read in parallel with the command buffer (same address), so the value is
// there when the instruction is decoded. Power-up contents (all zero) are
// consistent: operands 0 -> point 0.
reg [151:0] pxc [0:511];
reg [151:0] pxc_q = 0;
reg pxc_we = 0;
reg [8:0] pxc_wa = 0;
reg [151:0] pxc_wd = 0;
integer pxc_i;
initial for(pxc_i = 0; pxc_i < 512; pxc_i = pxc_i + 1) pxc[pxc_i] = 152'd0;
always @(posedge clk) begin
  if(pxc_we) pxc[pxc_wa] <= pxc_wd;
  pxc_q <= pxc[cmd_addr];
end

// Newton-Raphson unit (3 lanes)
reg [31:0] nr_in0 = 0, nr_in1 = 0, nr_in2 = 0;
reg [31:0] nr_out0 = 0, nr_out1 = 0, nr_out2 = 0;
reg [31:0] nr_a0 = 0, nr_a1 = 0, nr_a2 = 0;
reg [31:0] nr_h0 = 0, nr_h1 = 0, nr_h2 = 0;
reg [31:0] nr_g0 = 0, nr_g1 = 0, nr_g2 = 0;
reg nr_s0 = 0, nr_s1 = 0, nr_s2 = 0, nr_z0 = 0, nr_z1 = 0, nr_z2 = 0;
reg [1:0] nr_mode = 0;
reg nr_it = 0;
reg [6:0] nr_ret = S_IDLE;

// combinational helpers
// hit state valid (he < hx), kept as a register next to he/hx so that the
// comparison is not in front of the condition / CSG logic
reg hvr = 1'b0;
wire hv = hvr;
`ifdef SRT_SIM_CHECKS
always @(negedge clk) if(!pend_rst && !pend_upd && (hvr !== slt(he, hx))) $display("%t srt_engine: hvr mismatch", $time);
`endif
wire [5:0] dop = cmd_q[5:0];
reg dex;
always @(*) begin
  case(cmd_q[7:6])
    2'd0: dex = 1'b1;
    2'd1: dex = hv;
    2'd2: dex = ~hv;
    default: dex = orh;
  endcase
end

// The RAM address is normally pc; a taken jump (and the start of a ray)
// presents its target directly, so no fetch bubble is needed.
wire fetch_jump = (state == S_DEC) && (wcnt == 2'd0) && !icount[16] &&
                  ((cmd_q[5:0] == I_Jump) || (cmd_q[5:0] == I_ResetHitStateAndJump)) && dex;
wire fetch_zero = (state == S_PH1) && (wcnt == 2'd0);
// pc holds the index of the current instruction (in S_DEC: of the previous
// one when pc_lag is set, i.e. when S_DEC was entered from another state, so
// that no instruction state has to advance pc). The RAM is always presented
// the following instruction's address.
reg pc_lag = 1'b0;
wire [15:0] curpc = pc + {15'd0, pc_lag & (state == S_DEC)};
wire [15:0] nxtpc = pc + ((pc_lag && (state == S_DEC)) ? 16'd2 : 16'd1);
assign cmd_addr = fetch_zero ? 9'd0 : fetch_jump ? cmd_q[16:8] : nxtpc[8:0];

wire nr_rcp = (nr_mode == NR_RCP);
wire nr_rsq = (nr_mode == NR_RSQ);
wire [31:0] nra0 = (nr_rcp & nr_in0[31]) ? (32'd0 - nr_in0) : nr_in0;
wire [31:0] nra1 = (nr_rcp & nr_in1[31]) ? (32'd0 - nr_in1) : nr_in1;
wire [31:0] nra2 = (nr_rcp & nr_in2[31]) ? (32'd0 - nr_in2) : nr_in2;

wire [31:0] nrh0 = nr_rsq ? {nr_in0[31], nr_in0[31:1]} : {1'b0, nra0[31:1]};
wire [31:0] nrh1 = nr_rsq ? {nr_in1[31], nr_in1[31:1]} : {1'b0, nra1[31:1]};
wire [31:0] nrh2 = nr_rsq ? {nr_in2[31], nr_in2[31:1]} : {1'b0, nra2[31:1]};
wire [4:0] nrb0 = hibit(nr_a0), nrb1 = hibit(nr_a1), nrb2 = hibit(nr_a2);

wire prim = ~phase[1];
// half resolution: dither as pixel x / 2, so the doubled pixels alternate
wire [2:0] dith_idx = {py[1:0], P_half ? px[1] : px[0]};
reg [7:0] dith;
always @(*) begin
  case(dith_idx)
    3'd0: dith = 8'd0; 3'd1: dith = 8'd4; 3'd2: dith = 8'd2; 3'd3: dith = 8'd6;
    3'd4: dith = 8'd3; 3'd5: dith = 8'd7; 3'd6: dith = 8'd1; default: dith = 8'd5;
  endcase
end

// temporaries (blocking, only used within one clock)
reg [31:0] t_a, t_b, t_c;
reg [15:0] t16, t16b;
reg t_ins, t_miss, t_hit;
reg [31:0] ne, nhx;
reg [1:0] nsel;
reg nhv;
reg w_and = 0, pend_rst = 0, pend_upd = 0;
reg [31:0] he_n = 0, hx_n = 0;
reg [1:0] hn_sel = 0;
reg [31:0] t_neg;
reg [7:0] t8a, t8b;
reg [31:0] exitd;

localparam [1:0] W1 = MUL_LAT - 1;           // wait after a single issue
localparam [1:0] W2 = MUL_LAT - 2;           // wait after the 2nd back-to-back issue
localparam [1:0] W3 = MUL_LAT - 3;           // wait after the 3rd back-to-back issue

// Newton-Raphson set-up from nr_in*/nr_mode (the work of state S_NR0)
task nr_setup;
  begin
      nr_a0 <= nra0; nr_a1 <= nra1; nr_a2 <= nra2;
      nr_h0 <= nrh0; nr_h1 <= nrh1; nr_h2 <= nrh2;
      nr_s0 <= nr_in0[31]; nr_s1 <= nr_in1[31]; nr_s2 <= nr_in2[31];
      nr_z0 <= (nr_in0 == 32'd0); nr_z1 <= (nr_in1 == 32'd0); nr_z2 <= (nr_in2 == 32'd0);
      nr_it <= 1'b0;
        end
endtask

// an intersection test that missed (entry = INF, exit = 0): only an And
// operation on a valid hit state changes it (to no hit); go straight to the
// next instruction
task isect_miss;
  begin
    if(hv && w_and) begin
      // he/hx are reset in the following S_DEC cycle (keeps this decision
      // logic away from the he/hx registers); hvr is the only part read there
      hvr <= 1'b0; pend_rst <= 1'b1; hp_depth <= INF;
    end else begin
      hp_depth <= he;
    end
    hp_pend <= 1'b1;
    state <= S_DEC;
  end
endtask

// CSG combination of an intersection [ee, ex) with the hit state (S_COMB)
task comb_apply;
  input [31:0] ee;
  input [31:0] ex;
  begin
        ne = he; nhx = hx; nsel = 2'd0; nhv = hv;
        case(w[5:0])
          I_Sphere, I_Plane, I_AABB: begin
            if(slt(ee, ex)) begin
              nhv = 1'b1;
              if(!hv) begin
                ne = ee; nhx = ex; nsel = 2'd1;
              end else begin
                if(slt(ee, he)) begin
                  ne = ee; nsel = 2'd1;
                end
                if(slt(hx, ex)) nhx = ex;
              end
            end
          end
          I_SphereSub, I_PlaneSub, I_AABBSub: begin
            if(hv) begin
              if(!slt(he, ee) && !slt(ex, hx)) begin
                ne = INF; nhx = 32'd0; nhv = 1'b0;
              end else if(slt(ee, he) && slt(he, ex) && !slt(hx, ex)) begin
                ne = ex; nsel = 2'd2; nhv = slt(ex, hx);
              end else if(slt(he, ee) && slt(ee, hx) && !slt(ex, hx)) begin
                nhx = ee;
              end
            end
          end
          default: begin // And
            if(hv) begin
              if(!slt(ee, ex)) begin
                ne = INF; nhx = 32'd0; nhv = 1'b0;
              end else begin
                if(slt(he, ee)) begin
                  ne = ee; nsel = 2'd1;
                end
                if(slt(ex, hx)) nhx = ex;
                // new entry < new exit, from the comparisons above
                if(slt(he, ee) && !slt(ex, hx)) nhv = slt(ee, hx);
                else if(!slt(he, ee) && slt(ex, hx)) nhv = slt(he, ex);
                else nhv = 1'b1;
              end
            end
          end
        endcase
        hvr <= nhv;
        he_n <= ne; hx_n <= nhx; pend_upd <= 1'b1;   // written in the next S_DEC
        hp_depth <= ne; hp_pend <= 1'b1;
        state <= S_DEC;
        // the hit normal is written in the next S_DEC (hn_sel), except for
        // sphere normals which need their own computation (S_SN*)
        hn_sel <= 2'd0;
        if(nsel == 2'd1) begin
          if(kind != 2'd0) begin
            hn_sel <= 2'd1;
          end else if(insd) begin
            hn_sel <= 2'd2;
          end else begin
            sn_dep <= ee; sn_neg <= 1'b0;
            state <= S_SN0;
          end
        end else if(nsel == 2'd2) begin
          if(kind != 2'd0) begin
            hn_sel <= 2'd3;
          end else begin
            sn_dep <= ex; sn_neg <= 1'b1;
            state <= S_SN0;
          end
        end
  end
endtask

always @(posedge clk) begin
  pix_we <= 1'b0;
  pc_lag <= (state != S_DEC);
  rc_we <= 1'b0;
  pxc_we <= 1'b0;
  if(state != S_IDLE) cycles <= cycles + 32'd1;

  if(start | abort) begin
    // pending hit state updates (pend_*, hn_sel) stay pending: they are
    // applied at the next S_DEC, as the state carries over between frames
    wcnt <= 2'd0;
    if(abort) begin
      state <= S_IDLE;
    end else begin
      P_sx <= i_sx; P_sy <= i_sy; P_sz <= i_sz;
      P_xsx <= i_xsx; P_xsy <= i_xsy; P_xsz <= i_xsz;
      P_ysx <= i_ysx; P_ysy <= i_ysy; P_ysz <= i_ysz;
      Lx <= i_lx; Ly <= i_ly; Lz <= i_lz;
      P_half <= i_half;
      lin_x <= i_dx; lin_y <= i_dy; lin_z <= i_dz;
      cdx <= i_dx; cdy <= i_dy; cdz <= i_dz;
      px <= 8'd0; py <= 8'd0;
      cycles <= 32'd0;
      state <= S_PIX0;
    end
  end else if(wcnt != 2'd0) begin
    wcnt <= wcnt - 2'd1;
  end else begin
    case(state)
    S_IDLE: ;

    // ---------------------------------------------------------------- pixel
    // FixedNormalise16Bit of the camera ray
    S_PIX0: begin
      ma0 <= sx16(cdx); mb0 <= sx16(cdx);
      ma1 <= sx16(cdy); mb1 <= sx16(cdy);
      ma2 <= sx16(cdz); mb2 <= sx16(cdz);
      wcnt <= W1; state <= S_PIX1;
    end
    S_PIX1: begin
      nr_in0 <= sx16(sum16); nr_mode <= NR_RSQ; nr_ret <= S_PIX2; state <= S_NF0;
    end
    S_PIX2: begin
      ma0 <= sx16(cdx); mb0 <= sx16(nr_out0[15:0]);
      ma1 <= sx16(cdy); mb1 <= sx16(nr_out0[15:0]);
      ma2 <= sx16(cdz); mb2 <= sx16(nr_out0[15:0]);
      wcnt <= W1; state <= S_PIX3;
    end
    S_PIX3: begin
      pdx <= b0[15:0]; pdy <= b1[15:0]; pdz <= b2[15:0];
      rdx <= b0[15:0]; rdy <= b1[15:0]; rdz <= b2[15:0];
      rsx <= P_sx; rsy <= P_sy; rsz <= P_sz;
      p_hit <= 1'b0; p_shadow <= 1'b0; s_hit <= 1'b0; s_shadow <= 1'b0;
      phase <= 2'd0;
      state <= S_PH0;
    end

    // ---------------------------------------------------------------- phases
    S_PH0: begin
      if(slt(rsx, MIN_TRACE) || slt(MAX_TRACE, rsx) ||
         slt(rsy, MIN_TRACE) || slt(MAX_TRACE, rsy) ||
         slt(rsz, MIN_TRACE) || slt(MAX_TRACE, rsz)) begin
        state <= S_FIN0;
      end else begin
        sx <= rsx; sy <= rsy; sz <= rsz;
        dx <= rdx; dy <= rdy; dz <= rdz;
        sox <= rsx - ox; soy <= rsy - oy; soz <= rsz - oz;
        shadow <= phase[0];
        nr_in0 <= sx16(rdx); nr_in1 <= sx16(rdy); nr_in2 <= sx16(rdz);
        nr_mode <= NR_RCP; nr_ret <= S_PH1; state <= S_NR0;
      end
    end
    S_PH1: begin
      rcpx <= nr_out0; rcpy <= nr_out1; rcpz <= nr_out2;
      pc <= 16'hFFFF; icount <= 17'd0;   // instruction 0 is being read (fetch_zero); pc + pc_lag = 0
      state <= S_DEC;
    end

    // ---------------------------------------------------------------- interpreter
    // invariant in S_DEC: cmd_q = mem[curpc] (the instruction), cmd_addr = curpc + 1
    S_DEC: begin
      w_and <= (dop == I_SphereAnd) || (dop == I_PlaneAnd) || (dop == I_AABBAnd);
      // deferred hit state updates of the previous instruction(s) (resets,
      // COMB, isect_miss). Every instruction passes through here before any
      // state that reads he/hx, so this is the only place he/hx are written
      // (hvr, which S_DEC itself needs, is always updated immediately).
      if(pend_rst) begin
        he <= INF; hx <= 32'd0; pend_rst <= 1'b0;
      end
      if(pend_upd) begin
        he <= he_n; hx <= hx_n; pend_upd <= 1'b0;
      end
      case(hn_sel)
        2'd1: begin hnx <= enx; hny <= eny; hnz <= enz; end
        2'd2: begin hnx <= 16'd0 - dx; hny <= 16'd0 - dy; hnz <= 16'd0 - dz; end
        2'd3: begin hnx <= 16'd0 - xnx; hny <= 16'd0 - xny; hnz <= 16'd0 - xnz; end
        default: ;
      endcase
      hn_sel <= 2'd0;
      // plane products, issued speculatively for every decoded instruction
      // (unused otherwise): (start - point).N, N.D and (-N).D
      ma0 <= sox - pxc_q[95:64]; mb0 <= sx16(cf2_10(cmd_q[19:8]));
      ma1 <= soy - pxc_q[63:32]; mb1 <= sx16(cf2_10(cmd_q[31:20]));
      ma2 <= soz - pxc_q[31:0];  mb2 <= sx16(cf2_10(cmd_q[43:32]));
      me0 <= dx; mf0 <= cf2_10(cmd_q[19:8]);
      me1 <= dy; mf1 <= cf2_10(cmd_q[31:20]);
      me2 <= dz; mf2 <= cf2_10(cmd_q[43:32]);
      mc0 <= dx; md0 <= 16'd0 - cf2_10(cmd_q[19:8]);
      mc1 <= dy; md1 <= 16'd0 - cf2_10(cmd_q[31:20]);
      mc2 <= dz; md2 <= 16'd0 - cf2_10(cmd_q[43:32]);
      if(icount[16]) begin
        state <= S_PHEND;              // runaway command list
      end else begin
        icount <= icount + 17'd1;
        pc <= nxtpc;                   // default: next instruction
        case(dop)
          I_Start: begin
            ox <= 32'd0; oy <= 32'd0; oz <= 32'd0;
            sox <= sx; soy <= sy; soz <= sz;
            if(dex) begin
              pend_rst <= 1'b1; hvr <= 1'b0; orh <= 1'b0; reg_hit <= 1'b0;
            end
          end
          I_Origin: begin
            ox <= cf8_7(cmd_q[22:8]); oy <= cf8_7(cmd_q[37:23]); oz <= cf8_7(cmd_q[52:38]);
            sox <= sx - cf8_7(cmd_q[22:8]);
            soy <= sy - cf8_7(cmd_q[37:23]);
            soz <= sz - cf8_7(cmd_q[52:38]);
          end
          I_Sphere, I_SphereSub, I_SphereAnd: if(dex) begin
            w <= cmd_q; pc <= curpc; state <= S_SPH1;
            kind <= 2'd0;
            objx <= cf8_7(cmd_q[22:8]) + ox;
            objy <= cf8_7(cmd_q[37:23]) + oy;
            objz <= cf8_7(cmd_q[52:38]) + oz;
            tx <= sox - cf8_7(cmd_q[22:8]);     // ray start - centre
            ty <= soy - cf8_7(cmd_q[37:23]);
            tz <= soz - cf8_7(cmd_q[52:38]);
            rad <= cf4_7(cmd_q[63:53]);
          end
          I_Plane, I_PlaneSub, I_PlaneAnd: if(dex) begin
            w <= cmd_q; pc <= curpc;
            kind <= 2'd1;
            nnx <= cf2_10(cmd_q[19:8]); nny <= cf2_10(cmd_q[31:20]); nnz <= cf2_10(cmd_q[43:32]);
            rsq <= cf8_12(cmd_q[63:44]);        // distance
            // ray start - point on the plane (normal * distance, cached per instruction)
            tx <= sox - pxc_q[95:64]; ty <= soy - pxc_q[63:32]; tz <= soz - pxc_q[31:0];
            pxc_hit <= (pxc_q[151:96] == cmd_q[63:8]);
            state <= S_PLB;                     // S_PLB branches to S_PLM0 on a cache miss
          end
          I_AABB, I_AABBSub, I_AABBAnd: if(dex) begin
            w <= cmd_q; pc <= curpc; state <= S_BX1;
            kind <= 2'd2;
            tx <= cf8_1(cmd_q[16:8]) - sox;     // slab planes relative to the ray start
            ty <= cf8_1(cmd_q[25:17]) - soy;
            tz <= cf8_1(cmd_q[34:26]) - soz;
            ux <= cf8_1(cmd_q[43:35]) - sox;
            uy <= cf8_1(cmd_q[52:44]) - soy;
            uz <= cf8_1(cmd_q[61:53]) - soz;
          end
          I_Checkerboard: if(dex) begin
            w <= cmd_q; pc <= curpc; state <= S_CHK;
          end
          I_RegisterHit, I_RegisterHitNoReset: if(dex) begin
            if(hv) begin
              w <= cmd_q; pc <= curpc; state <= S_RH;
            end else begin
              orh <= 1'b0;             // no hit to register: done here
            end
          end
          I_ResetHitState: if(dex) begin
            pend_rst <= 1'b1; hvr <= 1'b0;
          end
          I_Jump, I_ResetHitStateAndJump: begin
            if(dop == I_ResetHitStateAndJump) begin
              pend_rst <= 1'b1; hvr <= 1'b0;
            end
            if(dex) begin
              pc <= cmd_q[23:8];           // target is being read (fetch_jump)
            end
          end
          I_End: if(dex) begin
            state <= S_PHEND;
          end
          default: ;
        endcase
      end
    end

    // resolve the lazily evaluated hit position: start + dir * depth
    S_HP0: begin
      ma0 <= sx16(dx); mb0 <= hp_depth;
      ma1 <= sx16(dy); mb1 <= hp_depth;
      ma2 <= sx16(dz); mb2 <= hp_depth;
      wcnt <= W1; state <= S_HP1;
    end
    S_HP1: begin
      hpx <= sx + a0; hpy <= sy + a1; hpz <= sz + a2;
      hp_pend <= 1'b0;
      state <= hp_ret;
    end

    S_CHK: begin
      if(hp_pend) begin
        hp_ret <= S_CHK; state <= S_HP0;
      end else begin
        if(hpx[14] ^ hpz[14]) begin
          reg_albedo <= w[31:16]; reg_refl <= w[15:8];
        end
        state <= S_DEC;
      end
    end

    S_RH: begin
      if(hv && (~reg_hit || slt(he, reg_depth)) && hp_pend) begin
        hp_ret <= S_RH; state <= S_HP0;
      end else begin
        orh <= 1'b0;
        state <= S_DEC;
        if(hv) begin
          if(~reg_hit || slt(he, reg_depth)) begin
            reg_hit <= 1'b1; reg_depth <= he;
            reg_x <= hpx; reg_y <= hpy; reg_z <= hpz;
            reg_nx <= hnx; reg_ny <= hny; reg_nz <= hnz;
            reg_albedo <= w[31:16]; reg_refl <= w[15:8];
            orh <= 1'b1;
            if(shadow) begin
              state <= S_PHEND;
            end
          end
          if(w[5:0] != I_RegisterHitNoReset) begin
            pend_rst <= 1'b1; hvr <= 1'b0;
          end
        end
      end
    end

    S_PHEND: begin
      if(hp_pend) begin
        hp_ret <= S_PHEND; state <= S_HP0;
      end else begin
        case(phase)
          2'd0: begin
            p_hit <= reg_hit;
            p_x <= reg_x; p_y <= reg_y; p_z <= reg_z;
            p_nx <= reg_nx; p_ny <= reg_ny; p_nz <= reg_nz;
            p_albedo <= reg_albedo; p_refl <= reg_refl;
          end
          2'd1: p_shadow <= reg_hit;
          2'd2: begin
            s_hit <= reg_hit;
            s_x <= reg_x; s_y <= reg_y; s_z <= reg_z;
            s_nx <= reg_nx; s_ny <= reg_ny; s_nz <= reg_nz;
            s_albedo <= reg_albedo;
          end
          default: s_shadow <= reg_hit;
        endcase
        state <= S_FIN0;
      end
    end

    // ---------------------------------------------------------------- sphere
    S_SPH1: begin                      // (start - centre).D, radius^2
      ma0 <= tx; mb0 <= sx16(dx);
      ma1 <= ty; mb1 <= sx16(dy);
      ma2 <= tz; mb2 <= sx16(dz);
      // FixedMul(r, r) for r = v << 7 (v: 11 bit operand) is exactly v * v
      mc0 <= {{5{w[63]}}, w[63:53]}; md0 <= {{5{w[63]}}, w[63:53]};
      state <= S_SPH2;
    end
    S_SPH2: begin                      // |start - centre|^2
      ma0 <= tx; mb0 <= tx;
      ma1 <= ty; mb1 <= ty;
      ma2 <= tz; mb2 <= tz;
      wcnt <= W2; state <= S_SPH4;
    end
    S_SPH4: begin
      cpa <= sumA;                     // -cpa, negated in the next state
      rsq <= r0;
      state <= S_SPH5;
    end
    S_SPH5: begin
      dsq <= sumA;
      cpa <= 32'd0 - cpa;
      ma0 <= cpa; mb0 <= cpa;          // (-cpa)^2 == cpa^2
      state <= S_SPH6;
    end
    S_SPH6: begin
      dnp <= rsq - dsq;
      t_ins = slt(dsq, rsq);
      insd <= t_ins;
      if(cpa[31] && !t_ins) begin
        isect_miss;
      end else begin
        wcnt <= W2; state <= S_SPH8;
      end
    end
    S_SPH8: begin
      dsq <= dsq - a0;                 // distance from centre squared
      nr_in0 <= dnp + a0;              // rsq - (dsq - cpa^2)
      nr_mode <= NR_SQRT; nr_ret <= S_SPH9;
      state <= S_SPH8B;
    end
    S_SPH8B: begin
      // decide from registers; on a hit this is also the Newton-Raphson setup
      if(!slt(dsq, rsq)) begin
        isect_miss;
      end else begin
        nr_setup;
        state <= S_NFA;
      end
    end
    S_SPH9: begin
      t_a = insd ? 32'd0 : (cpa - nr_out0);
      if(t_a[31]) begin
        isect_miss;
      end else begin
        e_entry <= t_a; e_exit <= cpa + nr_out0;
        state <= S_COMB;
      end
    end

    // ---------------------------------------------------------------- plane
    // point on the plane = normal * distance (cache miss: compute + store)
    S_PLM0: begin
      ma0 <= sx16(nnx); mb0 <= rsq;
      ma1 <= sx16(nny); mb1 <= rsq;
      ma2 <= sx16(nnz); mb2 <= rsq;
      wcnt <= W1; state <= S_PLM1;
    end
    S_PLM1: begin
      tx <= sox - a0; ty <= soy - a1; tz <= soz - a2;
      pxc_we <= 1'b1;
      pxc_wa <= pc[8:0];
      pxc_wd <= {w[63:8], a0, a1, a2};
      pxc_hit <= 1'b1;
      state <= S_PLA;
    end
    S_PLA: begin                       // the S_DEC products again, after a point cache miss
      ma0 <= tx; mb0 <= sx16(nnx);
      ma1 <= ty; mb1 <= sx16(nny);
      ma2 <= tz; mb2 <= sx16(nnz);
      me0 <= dx; mf0 <= nnx;
      me1 <= dy; mf1 <= nny;
      me2 <= dz; mf2 <= nnz;
      mc0 <= dx; md0 <= 16'd0 - nnx;
      mc1 <= dy; md1 <= 16'd0 - nny;
      mc2 <= dz; md2 <= 16'd0 - nnz;
      wcnt <= W1; state <= S_PLD;
    end
    S_PLB: begin                       // products issued in S_DEC: plane point cached?
      if(pxc_hit) begin
        wcnt <= W2; state <= S_PLD;
      end else begin
        state <= S_PLM0;               // point not cached: compute it first
      end
    end
    S_PLD: begin                       // products arrive; reciprocal cache read starts
      dnp <= sumD;
      dnn <= sumC;
      cpa <= sumA;                     // dot (negated when used if inside)
      insd <= sumA[31];
      state <= S_PLF;
    end
    S_PLF: begin
      // both reciprocal cache reads arrive; insd (sign of the dot product)
      // selects between them. dot * reciprocal is issued on the fast lane
      // unconditionally (used on a cache hit), as is the Newton-Raphson
      // set-up (used on a miss).
      t_b = insd ? rc_qb[31:0] : rc_qa[31:0];          // cached reciprocal
      fa <= insd ? 32'd0 - cpa : cpa; fb <= t_b;
      pl_rneg <= t_b[31];
      // rcp(denom) is negative for denom = -dn in (-2^18, -8], i.e.
      // dn in [8, 2^18): then only its sign matters (no division needed).
      // Newton-Raphson set-up: |denom| = -dn, or dn when -dn is negative
      // (also right for the wrap of dn = -2^31)
      if(insd) begin
        nnx <= 16'd0 - nnx; nny <= 16'd0 - nny; nnz <= 16'd0 - nnz;
        t_a = 32'd0 - dnn;             // denom
        t_c = dnn;
        t_miss = pl_shortcut(dnn);
        t_hit = rc_hit_b;
      end else begin
        t_a = 32'd0 - dnp;
        t_c = dnp;
        t_miss = pl_shortcut(dnp);
        t_hit = rc_hit_a;
      end
      dsq <= t_a;
      t_neg = t_a[31] ? t_c : t_a;
      nr_a0 <= t_neg; nr_h0 <= {1'b0, t_neg[31:1]};
      nr_s0 <= t_a[31]; nr_z0 <= (t_c == 32'd0);
      nr_mode <= NR_RCP; nr_ret <= S_PL10; nr_it <= 1'b0;
      if(t_miss) begin
        // rcp(denom) is negative: only its sign matters
        if(insd) begin
          e_entry <= 32'd0; e_exit <= INF;
          enx <= 16'd0 - dx; eny <= 16'd0 - dy; enz <= 16'd0 - dz;
          xnx <= 16'd0; xny <= 16'd0; xnz <= 16'd0;
          state <= S_COMB;
        end else begin
          isect_miss;
        end
      end else if(t_hit) begin         // reciprocal cache hit
        wcnt <= 2'd1; state <= S_PL11;
      end else begin
        state <= S_NFA;
      end
    end
    // S_PL10 is only a return address: S_NR5 does its work (pl_result)
    S_PL11: begin                      // fr = dot * rcp(denom) (fast lane)
      if(pl_rneg) begin
        // reciprocal was negative: only its sign matters
        if(insd) begin
          e_entry <= 32'd0; e_exit <= INF;
          enx <= 16'd0 - dx; eny <= 16'd0 - dy; enz <= 16'd0 - dz;
          xnx <= 16'd0; xny <= 16'd0; xnz <= 16'd0;
          state <= S_COMB;
        end else begin
          isect_miss;
        end
      end else if(!fr[31]) begin
        e_entry <= insd ? 32'd0 : fr;
        e_exit <= insd ? fr : INF;
        enx <= nnx; eny <= nny; enz <= nnz;
        xnx <= 16'd0 - nnx; xny <= 16'd0 - nny; xnz <= 16'd0 - nnz;
        state <= S_COMB;
      end else begin
        isect_miss;
      end
    end

    // ---------------------------------------------------------------- AABB
    S_BX1: begin
      ma0 <= tx; mb0 <= rcpx;
      ma1 <= ty; mb1 <= rcpy;
      ma2 <= tz; mb2 <= rcpz;
      state <= S_BX2;
    end
    S_BX2: begin
      ma0 <= ux; mb0 <= rcpx;
      ma1 <= uy; mb1 <= rcpy;
      ma2 <= uz; mb2 <= rcpz;
      wcnt <= W2; state <= S_BX4;
    end
    S_BX4: begin
      zx <= b0; zy <= b1; zz <= b2;    // t0
      state <= S_BX5;
    end
    S_BX5: begin
      vx <= slt(zx, b0) ? zx : b0;     // tmin
      vy <= slt(zy, b1) ? zy : b1;
      vz <= slt(zz, b2) ? zz : b2;
      zx <= slt(b0, zx) ? zx : b0;     // tmax
      zy <= slt(b1, zy) ? zy : b1;
      zz <= slt(b2, zz) ? zz : b2;
      state <= S_BX6;
    end
    S_BX6: begin
      enx <= 16'd0; eny <= 16'd0; enz <= 16'd0;
      xnx <= 16'd0; xny <= 16'd0; xnz <= 16'd0;
      if(slt(zx, zy) && slt(zx, zz)) begin
        exitd = zx; xnx <= axn(dx[15], 1'b0);
      end else if(slt(zy, zz)) begin
        exitd = zy; xny <= axn(dy[15], 1'b0);
      end else begin
        exitd = zz; xnz <= axn(dz[15], 1'b0);
      end
      t_miss = exitd[31]
             || (dez(dx) && (!tx[31] || ux[31]))
             || (dez(dy) && (!ty[31] || uy[31]))
             || (dez(dz) && (!tz[31] || uz[31]));
      if(t_miss) begin
        isect_miss;
      end else begin
        state <= S_COMB;
        e_exit <= exitd;
        if(slt(vy, vx) && slt(vz, vx)) begin
          e_entry <= slt(32'd0, vx) ? vx : 32'd0; enx <= axn(dx[15], 1'b1);
        end else if(slt(vz, vy)) begin
          e_entry <= slt(32'd0, vy) ? vy : 32'd0; eny <= axn(dy[15], 1'b1);
        end else begin
          e_entry <= slt(32'd0, vz) ? vz : 32'd0; enz <= axn(dz[15], 1'b1);
        end
      end
    end

    // ---------------------------------------------------------------- CSG
    S_COMB: begin
      comb_apply(e_entry, e_exit);
    end

    // sphere normal: (start + dir * depth - centre) * rcp(radius)
    S_SN0: begin
      ma0 <= sx16(dx); mb0 <= sn_dep;
      ma1 <= sx16(dy); mb1 <= sn_dep;
      ma2 <= sx16(dz); mb2 <= sn_dep;
      wcnt <= W1; state <= S_SN1;
    end
    S_SN1: begin
      tx <= sx + a0 - objx; ty <= sy + a1 - objy; tz <= sz + a2 - objz;
      if(ri_valid && (ri_rad == rad)) begin
        nr_out0 <= ri_val; state <= S_SN2;
      end else begin
        nr_in0 <= rad; nr_mode <= NR_RCP; nr_ret <= S_SN2; state <= S_NF0;
      end
    end
    S_SN2: begin
      ri_valid <= 1'b1; ri_rad <= rad; ri_val <= nr_out0;
      ma0 <= tx; mb0 <= nr_out0;
      ma1 <= ty; mb1 <= nr_out0;
      ma2 <= tz; mb2 <= nr_out0;
      wcnt <= W1; state <= S_SN3;
    end
    S_SN3: begin
      if(sn_neg) begin
        hnx <= 16'd0 - a0[15:0]; hny <= 16'd0 - a1[15:0]; hnz <= 16'd0 - a2[15:0];
      end else begin
        hnx <= a0[15:0]; hny <= a1[15:0]; hnz <= a2[15:0];
      end
      state <= S_DEC;
    end

    // ---------------------------------------------------------------- shading
    S_FIN0: begin
      if(prim ? p_hit : s_hit) begin
        ma0 <= sx16(prim ? p_nx : s_nx); mb0 <= sx16(Lx);     // N.L
        ma1 <= sx16(prim ? p_ny : s_ny); mb1 <= sx16(Ly);
        ma2 <= sx16(prim ? p_nz : s_nz); mb2 <= sx16(Lz);
        mc0 <= pdx; md0 <= p_nx;                               // D.N (16x16 lanes)
        mc1 <= pdy; md1 <= p_ny;
        mc2 <= pdz; md2 <= p_nz;
        wcnt <= W1; state <= S_FIN1;
      end else begin
        ma0 <= sx16(rdx); mb0 <= sx16(Lx);
        ma1 <= sx16(rdy); mb1 <= sx16(Ly);
        ma2 <= sx16(rdz); mb2 <= sx16(Lz);
        wcnt <= W1; state <= S_SKY0;
      end
    end
    S_FIN1: begin
      bse <= sum16;
      ma0 <= sx16(p_nx); mb0 <= sx16(sumC[15:0]);
      ma1 <= sx16(p_ny); mb1 <= sx16(sumC[15:0]);
      ma2 <= sx16(p_nz); mb2 <= sx16(sumC[15:0]);
      wcnt <= W1; state <= S_FIN3;
    end
    S_FIN3: begin
      sdx <= pdx - {b0[14:0], 1'b0};
      sdy <= pdy - {b1[14:0], 1'b0};
      sdz <= pdz - {b2[14:0], 1'b0};
      if(bse[15] || (prim ? p_shadow : s_shadow)) begin
        spec <= 8'd0; state <= S_FIN8;
      end else begin
        ma0 <= sx16(pdx - {b0[14:0], 1'b0}); mb0 <= sx16(Lx);
        ma1 <= sx16(pdy - {b1[14:0], 1'b0}); mb1 <= sx16(Ly);
        ma2 <= sx16(pdz - {b2[14:0], 1'b0}); mb2 <= sx16(Lz);
        wcnt <= W1; state <= S_FIN3A;
      end
    end
    // the products' sum is registered first: sum -> clamp -> multiplier
    // operands in one cycle was too long for the engine clock
    S_FIN3A: begin
      sum16_r <= sum16;
      state <= S_FIN4;
    end
    S_FIN4: begin
      t16 = sum16_r[15] ? 16'h0000 : (sum16_r[14] ? 16'h3FFF : sum16_r);   // clamp to 0..0x3FFF
      ma0 <= sx16(t16); mb0 <= sx16(t16);
      wcnt <= W1; state <= S_FIN5;
    end
    S_FIN5, S_FIN6: begin
      ma0 <= sx16(b0[15:0]); mb0 <= sx16(b0[15:0]);
      wcnt <= W1; state <= state + 7'd1;
    end
    S_FIN7: begin
      spec <= b0[13:6];
      state <= S_FIN8;
    end
    S_FIN8: begin
      t16 = (!bse[15] && (bse > 16'h3FFF)) ? 16'h3FFF : bse;
      t16b = 16'd0 - t16;
      if(t16[15]) t8a = t16b[15:8];
      else if(prim ? p_shadow : s_shadow) t8a = {1'b0, t16[15:9]};
      else t8a = t16[13:6];
      t_a = {16'h0, prim ? p_albedo : s_albedo};
      ma0 <= {24'h0, t_a[4:0], 3'b000};   mb0 <= {24'h0, t8a};
      ma1 <= {24'h0, t_a[9:5], 3'b000};   mb1 <= {24'h0, t8a};
      ma2 <= {24'h0, t_a[14:10], 3'b000}; mb2 <= {24'h0, t8a};
      wcnt <= W1; state <= S_FIN9;
    end
    S_FIN9: begin
      if(prim) begin
        pr <= cadd(q0[15:8], spec); pg <= cadd(q1[15:8], spec); pb <= cadd(q2[15:8], spec);
      end else begin
        sr <= cadd(q0[15:8], spec); sg <= cadd(q1[15:8], spec); sb <= cadd(q2[15:8], spec);
      end
      state <= S_NEXT;
    end

    // sky
    S_SKY0: begin                      // sum registered first (timing)
      sum16_r <= sum16;
      state <= S_SKY1;
    end
    S_SKY1: begin
      t16 = sum16_r[15] ? 16'h0000 : sum16_r;
      ma0 <= sx16(t16); mb0 <= sx16(t16);
      wcnt <= W1; state <= S_SKY2;
    end
    S_SKY2: begin
      ma0 <= sx16(a0[15:0]); mb0 <= sx16(a0[15:0]);
      wcnt <= W1; state <= S_SKY3;
    end
    S_SKY3: begin
      t16 = rdy + 16'h4000;
      if(t16[15]) t16 = 16'h0000;
      t8a = a0[14:7];                  // sun colour
      t_a = {23'h0, t16[14:7]} + {24'h0, t8a};
      t_b = 32'd190 + {24'h0, t8a};
      t8b = t_a[8] ? 8'hFF : t_a[7:0];
      if(prim) begin
        pr <= t8b; pg <= t8b; pb <= t_b[8] ? 8'hFF : t_b[7:0];
      end else begin
        sr <= t8b; sg <= t8b; sb <= t_b[8] ? 8'hFF : t_b[7:0];
      end
      state <= S_NEXT;
    end

    // next phase
    S_NEXT: begin
      state <= S_OUT0;
      case(phase)
        2'd0: if(p_hit) begin
          if(!bse[15] && (bse != 16'd0)) begin
            phase <= 2'd1; state <= S_PH0;
          end else begin
            p_shadow <= 1'b1;
            if(p_refl != 8'd0) begin
              phase <= 2'd2; state <= S_PH0;
            end
          end
        end
        2'd1: if(p_refl != 8'd0) begin
          phase <= 2'd2; state <= S_PH0;
        end
        2'd2: if(s_hit) begin
          if(!bse[15] && (bse != 16'd0)) begin
            phase <= 2'd3; state <= S_PH0;
          end else begin
            s_shadow <= 1'b1;
          end
        end
        default: ;
      endcase
      // next ray (only used when going to S_PH0)
      if((phase == 2'd1) || ((phase == 2'd0) && (bse[15] || (bse == 16'd0)))) begin
        // reflection
        rsx <= p_x + {{20{p_nx[15]}}, p_nx[15:4]};
        rsy <= p_y + {{20{p_ny[15]}}, p_ny[15:4]};
        rsz <= p_z + {{20{p_nz[15]}}, p_nz[15:4]};
        rdx <= sdx; rdy <= sdy; rdz <= sdz;
      end else begin
        // shadow ray towards the light
        rsx <= (prim ? p_x : s_x) + {{20{Lx[15]}}, Lx[15:4]};
        rsy <= (prim ? p_y : s_y) + {{20{Ly[15]}}, Ly[15:4]};
        rsz <= (prim ? p_z : s_z) + {{20{Lz[15]}}, Lz[15:4]};
        rdx <= Lx; rdy <= Ly; rdz <= Lz;
      end
    end

    // ---------------------------------------------------------------- output
    S_OUT0: begin
      ma0 <= {24'h0, pr}; mb0 <= {24'h0, p_hit ? ~p_refl : 8'hFF};
      ma1 <= {24'h0, pg}; mb1 <= {24'h0, p_hit ? ~p_refl : 8'hFF};
      ma2 <= {24'h0, pb}; mb2 <= {24'h0, p_hit ? ~p_refl : 8'hFF};
      state <= S_OUT1;
    end
    S_OUT1: begin
      ma0 <= {24'h0, sr}; mb0 <= {24'h0, p_hit ? p_refl : 8'h00};
      ma1 <= {24'h0, sg}; mb1 <= {24'h0, p_hit ? p_refl : 8'h00};
      ma2 <= {24'h0, sb}; mb2 <= {24'h0, p_hit ? p_refl : 8'h00};
      wcnt <= W2; state <= S_OUT2;
    end
    S_OUT2: begin
      ct_r <= q0[15:8]; ct_g <= q1[15:8]; ct_b <= q2[15:8];
      state <= S_OUT3;
    end
    S_OUT3: begin
      t8a = cadd(cadd(ct_r, q0[15:8]), dith);
      t8b = cadd(cadd(ct_g, q1[15:8]), dith);
      t_a[7:0] = cadd(cadd(ct_b, q2[15:8]), dith);
      pix_data <= {t_a[7:3], t8b[7:3], t8a[7:3]};
      state <= S_OUT4;
    end
    S_OUT4: if(!pix_full) begin
      pix_we <= 1'b1;
      if(px == (P_half ? 8'd198 : 8'd199)) begin
        px <= 8'd0;
        if(py == 8'd159) begin
          state <= S_IDLE;
        end else begin
          py <= py + 8'd1;
          lin_x <= lin_x + P_ysx; lin_y <= lin_y + P_ysy; lin_z <= lin_z + P_ysz;
          cdx <= lin_x + P_ysx; cdy <= lin_y + P_ysy; cdz <= lin_z + P_ysz;
          state <= S_PIX0;
        end
      end else begin
        // half resolution: skip the odd pixel (the step added twice, 16 bit)
        px <= px + (P_half ? 8'd2 : 8'd1);
        cdx <= cdx + (P_half ? {P_xsx[14:0], 1'b0} : P_xsx);
        cdy <= cdy + (P_half ? {P_xsy[14:0], 1'b0} : P_xsy);
        cdz <= cdz + (P_half ? {P_xsz[14:0], 1'b0} : P_xsz);
        state <= S_PIX0;
      end
    end

    // ---------------------------------------------------------------- Newton-Raphson
    // rcp:   NR(|x|, |x|>>1)^2, sign restored; x == 0 -> 0x7FFFFFFF
    // sqrt:  NR(x, x>>1) * x;                   x == 0 -> 0
    // rsqrt: NR(x, x>>>1)
    S_NR0: begin
      nr_setup;
      state <= S_NRA;
    end
    S_NRA: begin
      nr_g0 <= seed_tab(nrb0); nr_g1 <= seed_tab(nrb1); nr_g2 <= seed_tab(nrb2);
      // first iteration: half * (seed * seed), the square comes from a table
      ma0 <= nr_h0; mb0 <= seed_sq(nrb0);
      ma1 <= nr_h1; mb1 <= seed_sq(nrb1);
      ma2 <= nr_h2; mb2 <= seed_sq(nrb2);
      wcnt <= W1; state <= S_NR3;
    end
    S_NR2: begin
      ma0 <= nr_h0; mb0 <= a0;
      ma1 <= nr_h1; mb1 <= a1;
      ma2 <= nr_h2; mb2 <= a2;
      wcnt <= W1; state <= S_NR3;
    end
    S_NR3: begin
      ma0 <= nr_g0; mb0 <= 32'h00006000 - a0;
      ma1 <= nr_g1; mb1 <= 32'h00006000 - a1;
      ma2 <= nr_g2; mb2 <= 32'h00006000 - a2;
      wcnt <= W1; state <= S_NR4;
    end
    S_NR4: begin
      nr_g0 <= a0; nr_g1 <= a1; nr_g2 <= a2;
      ma0 <= a0; ma1 <= a1; ma2 <= a2;
      if(!nr_it) begin
        nr_it <= 1'b1;
        mb0 <= a0; mb1 <= a1; mb2 <= a2;
        wcnt <= W1; state <= S_NR2;
      end else if(nr_rsq) begin
        nr_out0 <= a0; nr_out1 <= a1; nr_out2 <= a2;
        state <= nr_ret;
      end else begin
        if(nr_rcp) begin
          mb0 <= a0; mb1 <= a1; mb2 <= a2;
        end else begin
          mb0 <= nr_a0; mb1 <= nr_a1; mb2 <= nr_a2;
        end
        wcnt <= W1; state <= S_NR5;
      end
    end
    S_NR5: begin
      if(nr_rcp) begin
        nr_out0 <= nr_z0 ? INF : (nr_s0 ? 32'd0 - a0 : a0);
        nr_out1 <= nr_z1 ? INF : (nr_s1 ? 32'd0 - a1 : a1);
        nr_out2 <= nr_z2 ? INF : (nr_s2 ? 32'd0 - a2 : a2);
        state <= nr_ret;
      end else begin
        nr_out0 <= nr_z0 ? 32'd0 : a0;
        nr_out1 <= nr_z1 ? 32'd0 : a1;
        nr_out2 <= nr_z2 ? 32'd0 : a2;
        state <= nr_ret;
      end
    end

    // ---------------------------------------------------------------- Newton-Raphson, fast lane
    // Same computation as S_NR* for lane 0 only, on the single stage
    // multiplier (2 cycles per dependent multiply instead of MUL_LAT).
    // Used for every single-value reciprocal / square root; the 3 lane
    // reciprocal of the ray direction (S_PH0) uses S_NR*.
    S_NF0: begin
      nr_setup;
      state <= S_NFA;
    end
    S_NFA: begin
      nr_g0 <= seed_tab(nrb0);
      fa <= nr_h0; fb <= seed_sq(nrb0);
      wcnt <= 2'd1; state <= S_NF3;
    end
    S_NF2: begin
      fa <= nr_h0; fb <= fr;
      wcnt <= 2'd1; state <= S_NF3;
    end
    S_NF3: begin
      fa <= nr_g0; fb <= 32'h00006000 - fr;
      wcnt <= 2'd1; state <= S_NF4;
    end
    S_NF4: begin
      nr_g0 <= fr;
      fa <= fr;
      if(!nr_it) begin
        nr_it <= 1'b1;
        fb <= fr;
        wcnt <= 2'd1; state <= S_NF2;
      end else if(nr_rsq) begin
        nr_out0 <= fr;
        state <= nr_ret;
      end else begin
        fb <= nr_rcp ? fr : nr_a0;
        wcnt <= 2'd1; state <= S_NF5;
      end
    end
    S_NF5: begin
      if(nr_ret == S_PL10) begin
        // plane reciprocal: store it in the cache, then dot * rcp
        t_b = nr_z0 ? INF : (nr_s0 ? 32'd0 - fr : fr);
        rc_we <= 1'b1;
        rc_wa <= rc_hash(insd ? dnn : dnp);
        rc_wd <= {1'b1, insd ? dnn : dnp, t_b};
        if(t_b[31]) begin
          if(insd) begin
            e_entry <= 32'd0; e_exit <= INF;
            enx <= 16'd0 - dx; eny <= 16'd0 - dy; enz <= 16'd0 - dz;
            xnx <= 16'd0; xny <= 16'd0; xnz <= 16'd0;
            state <= S_COMB;
          end else begin
            isect_miss;
          end
        end else begin
          fa <= insd ? 32'd0 - cpa : cpa; fb <= t_b;
          pl_rneg <= 1'b0;
          wcnt <= 2'd1; state <= S_PL11;
        end
      end else if(nr_rcp) begin
        nr_out0 <= nr_z0 ? INF : (nr_s0 ? 32'd0 - fr : fr);
        state <= nr_ret;
      end else begin
        nr_out0 <= nr_z0 ? 32'd0 : fr;
        state <= nr_ret;
      end
    end

    default: begin
`ifdef SRT_SIM_CHECKS
      $display("%t srt_engine: undefined state %0d", $time, state);
`endif
      state <= S_IDLE;
    end
    endcase
  end
end

endmodule
