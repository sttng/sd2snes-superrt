// Wrapper for the engine testbench: engine + command buffer RAM (same read timing as superrt.v)
`timescale 1ns / 1ps
module tb_engine_top #(parameter MUL_LAT = 2)(
  input clk,
  input start,
  input [31:0] i_sx, input [31:0] i_sy, input [31:0] i_sz,
  input [15:0] i_dx, input [15:0] i_dy, input [15:0] i_dz,
  input [15:0] i_xsx, input [15:0] i_xsy, input [15:0] i_xsz,
  input [15:0] i_ysx, input [15:0] i_ysy, input [15:0] i_ysz,
  input [15:0] i_lx, input [15:0] i_ly, input [15:0] i_lz,
  input i_half,
  input we, input [8:0] waddr, input [63:0] wdata,
  output [14:0] pix_data, output pix_we, input pix_full,
  output busy, output [31:0] cycles
);
reg [63:0] mem [0:511];
always @(posedge clk) if(we) mem[waddr] <= wdata;
wire [8:0] ra;
reg [8:0] ra_r;
always @(posedge clk) ra_r <= ra;
wire [63:0] q = mem[ra_r];
srt_engine #(.MUL_LAT(MUL_LAT)) eng(.clk(clk), .start(start), .abort(1'b0),
  .i_sx(i_sx), .i_sy(i_sy), .i_sz(i_sz), .i_dx(i_dx), .i_dy(i_dy), .i_dz(i_dz),
  .i_xsx(i_xsx), .i_xsy(i_xsy), .i_xsz(i_xsz), .i_ysx(i_ysx), .i_ysy(i_ysy), .i_ysz(i_ysz),
  .i_lx(i_lx), .i_ly(i_ly), .i_lz(i_lz), .i_half(i_half),
  .cmd_addr(ra), .cmd_q(q), .pix_data(pix_data), .pix_we(pix_we), .pix_full(pix_full),
  .busy(busy), .cycles(cycles));
endmodule
