`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// superrt: SNES side of the SuperRT ray tracing chip for sd2snes mk3.
//
// SuperRT (c) 2021 Ben Carter / Shironeko Labs, MIT licensed.
// https://github.com/ShironekoBen/superrt
//
// The original chip only sees the ROM /CS + /OE lines and address bits A0-A15,
// so every register access is a *read*:
//   $8000-$BE7F  framebuffer window (16000 bytes, upper or lower half of a
//                32000 byte SNES 8bpp tile image; served from PSRAM by the
//                address decoder, see address.v)
//   $BE80-$BEFF  registers. Reading one selects it as the target of the next
//                proxied write (except MapUpperFB/MapLowerFB).
//   $BF00-$BFFF  proxied write: reading $BFxx writes xx to the selected
//                register and selects the next one.
//   $C000-$FFFF  program ROM (PSRAM)
//
// This module holds the register file and the 512 x 64 bit command buffer,
// implements the frame handshake and the 16x16 multiplier, and contains the
// hardware ray tracer (srt_engine.v) with a 2048 pixel output FIFO.
// Everything is exposed to the MCU through mcu_cmd.v (SPI commands 0xA0-0xA6).
//
// Frame handshake (same semantics as RenderFlowController.sv):
//   SNES reads NewFrame while not busy -> read bank flips, busy=1, request
//   pending for the MCU, and the engine starts rendering the frame with the
//   current parameters and command list. The MCU acks the request, streams
//   the RGB555 pixels out of the FIFO (raster order), maps them to the
//   palette, writes SNES tiles into the bank the SNES does not read and
//   signals done -> busy=0.
//   (An MCU that renders in software instead can still read the parameters
//   and command list with A3/A4; the engine then just stalls on a full FIFO.)
//////////////////////////////////////////////////////////////////////////////////
module superrt(
  input clk,
  input eng_clk,                 // ray tracing engine clock (asynchronous to clk)

  // SNES interface
  input [15:0] snes_addr,        // A15-A0 of the current access
  input reg_enable,              // access is in the $BE80-$BFFF window (ROMSEL low)
  input rd_strobe,               // one pulse per SNES read cycle (address stable)
  output reg [7:0] data_out,     // register read data
  output fb_read_bank,           // framebuffer bank the SNES reads from
  output fb_map_lower,           // window shows the lower half (offset 16000)

  // MCU interface
  input [11:0] mcu_ptr,          // byte index for the MCU read streams
  output [7:0] mcu_reg_byte,     // render parameters, byte mcu_ptr
  output [7:0] mcu_cmd_byte,     // command buffer, byte mcu_ptr (big endian words)
  output [7:0] mcu_status,
  input mcu_ack,                 // MCU has taken the frame request
  input mcu_done,                // MCU finished rendering the frame
  output [7:0] mcu_pix_byte,     // pixel FIFO head, low byte if mcu_ptr[0]==0
  input mcu_pix_pop,             // remove the FIFO head
  input mcu_info_snap,           // latch FIFO level / engine state for A6
  output reg [7:0] mcu_info_byte // A6 byte mcu_ptr
);

// register indices (low 7 bits of $BE80-$BEFF)
localparam [6:0]
  R_NewFrame   = 7'h00, R_MapUpperFB = 7'h01, R_MapLowerFB = 7'h02,
  R_RayStartX0 = 7'h03, R_RayStartX1 = 7'h04, R_RayStartX2 = 7'h05, R_RayStartX3 = 7'h06,
  R_RayStartY0 = 7'h07, R_RayStartY1 = 7'h08, R_RayStartY2 = 7'h09, R_RayStartY3 = 7'h0A,
  R_RayStartZ0 = 7'h0B, R_RayStartZ1 = 7'h0C, R_RayStartZ2 = 7'h0D, R_RayStartZ3 = 7'h0E,
  R_RayDirXL   = 7'h0F, R_RayDirXH   = 7'h10, R_RayDirYL   = 7'h11, R_RayDirYH   = 7'h12,
  R_RayDirZL   = 7'h13, R_RayDirZH   = 7'h14,
  R_XStepXL    = 7'h15, R_XStepXH    = 7'h16, R_XStepYL    = 7'h17, R_XStepYH    = 7'h18,
  R_XStepZL    = 7'h19, R_XStepZH    = 7'h1A,
  R_YStepXL    = 7'h1B, R_YStepXH    = 7'h1C, R_YStepYL    = 7'h1D, R_YStepYH    = 7'h1E,
  R_YStepZL    = 7'h1F, R_YStepZH    = 7'h20,
  R_MulAL      = 7'h21, R_MulAH      = 7'h22, R_MulBL      = 7'h23, R_MulBH      = 7'h24,
  R_MulO0      = 7'h25, R_MulO1      = 7'h26, R_MulO2      = 7'h27, R_MulO3      = 7'h28,
  R_CmdAddrL   = 7'h29, R_CmdAddrH   = 7'h2A,
  R_CmdData1   = 7'h2B, R_CmdData2   = 7'h2C, R_CmdData3   = 7'h2D, R_CmdData4   = 7'h2E,
  R_CmdData5   = 7'h2F, R_CmdData6   = 7'h30, R_CmdData7   = 7'h31, R_CmdData8   = 7'h32,
  R_LightXL    = 7'h33, R_LightXH    = 7'h34, R_LightYL    = 7'h35, R_LightYH    = 7'h36,
  R_LightZL    = 7'h37, R_LightZH    = 7'h38,
  R_Status     = 7'h39,
  R_Mode       = 7'h3A;   // sd2snes addition: bit 0 = full horizontal resolution

wire is_proxy = snes_addr[8];      // $BFxx (reg_enable already limits the range)
wire [6:0] reg_idx = snes_addr[6:0];
wire [7:0] wval = snes_addr[7:0];  // proxied write value

reg [6:0] last_reg = 7'h00;

reg [31:0] ray_start_x = 0, ray_start_y = 0, ray_start_z = 0;
reg [15:0] ray_dir_x = 0, ray_dir_y = 0, ray_dir_z = 0;
reg [15:0] xstep_x = 0, xstep_y = 0, xstep_z = 0;
reg [15:0] ystep_x = 0, ystep_y = 0, ystep_z = 0;
reg [15:0] light_x = 0, light_y = 0, light_z = 0;
reg [15:0] mul_a = 0, mul_b = 0;

reg map_lower = 1'b0;
reg read_bank = 1'b0;
reg busy = 1'b0;
reg pending = 1'b0;
// Resolution mode (sd2snes addition, register $BEBA, proxied write):
// bit 0 clear (default, also after configuration) = half horizontal
// resolution - the engine traces every other pixel (100 x 160) and the MCU
// doubles them; set = full 200 x 160. Taken over at NewFrame.
reg full_res = 1'b0;
reg frame_half = 1'b1;

assign fb_read_bank = read_bank;
assign fb_map_lower = map_lower;

// Multiplier: FixedMul(sext(A), sext(B)) - the product of two 16 bit values
// always fits, so this is a plain signed 16x16 multiply >>> 14.
wire signed [31:0] mul_p = $signed(mul_a) * $signed(mul_b);
wire signed [31:0] mul_o = mul_p >>> 14;

// Command buffer: 512 x 64 bit true dual port RAM.
//   port A: SNES writes, MCU reads (A4) when not writing
//   port B: engine reads
reg [63:0] cmdbuf [0:511];
reg [63:0] cmd_shift = 64'h0;
reg [5:0] cmd_bits = 6'd0;
reg [15:0] cmd_addr = 16'h0;
reg cmd_we = 1'b0;
reg [8:0] cmd_waddr;
reg [63:0] cmd_wdata;

wire eng_busy;
wire [8:0] eng_cmd_addr;
reg [63:0] cmd_rdata;       // port A read data (MCU)
reg [63:0] eng_cmd_q;       // port B read data (engine)
wire [8:0] cmd_port_a = cmd_we ? cmd_waddr : mcu_ptr[11:3];

always @(posedge clk) begin
  if(cmd_we) begin
    cmdbuf[cmd_port_a] <= cmd_wdata;
    cmd_rdata <= cmd_wdata;
  end else begin
    cmd_rdata <= cmdbuf[cmd_port_a];
  end
end

always @(posedge eng_clk) eng_cmd_q <= cmdbuf[eng_cmd_addr];   // engine clock domain

// big endian byte order within each word (as sent by the SNES / in CommandBuffer.bin).
// Registered: the RAM output -> byte select -> MCU data register path was too
// long for 96 MHz. The MCU takes the byte at the end of the next SPI byte, many
// clocks after mcu_ptr changed, so the extra stage costs nothing.
reg [7:0] cmd_byte_r = 8'h00;
always @(posedge clk) begin
  case(mcu_ptr[2:0])
    3'd0: cmd_byte_r <= cmd_rdata[63:56];
    3'd1: cmd_byte_r <= cmd_rdata[55:48];
    3'd2: cmd_byte_r <= cmd_rdata[47:40];
    3'd3: cmd_byte_r <= cmd_rdata[39:32];
    3'd4: cmd_byte_r <= cmd_rdata[31:24];
    3'd5: cmd_byte_r <= cmd_rdata[23:16];
    3'd6: cmd_byte_r <= cmd_rdata[15:8];
    default: cmd_byte_r <= cmd_rdata[7:0];
  endcase
end
assign mcu_cmd_byte = cmd_byte_r;

// push n bits of the proxied value into the command shift register
task push_bits;
  input [3:0] n;
  reg [63:0] shifted;
  reg [6:0] total;
  begin
    case(n)
      4'd1: shifted = {cmd_shift[62:0], wval[0]};
      4'd2: shifted = {cmd_shift[61:0], wval[1:0]};
      4'd3: shifted = {cmd_shift[60:0], wval[2:0]};
      4'd4: shifted = {cmd_shift[59:0], wval[3:0]};
      4'd5: shifted = {cmd_shift[58:0], wval[4:0]};
      4'd6: shifted = {cmd_shift[57:0], wval[5:0]};
      4'd7: shifted = {cmd_shift[56:0], wval[6:0]};
      default: shifted = {cmd_shift[55:0], wval[7:0]};
    endcase
    cmd_shift <= shifted;
    total = {1'b0, cmd_bits} + n;
    if(total == 7'd64) begin
      // full word: write it at the current address, then advance.
      // (The original FPGA wrote it one word too far - see notes.)
      cmd_we <= 1'b1;
      cmd_waddr <= cmd_addr[8:0];
      cmd_wdata <= shifted;
      cmd_addr <= cmd_addr + 16'd1;
      cmd_bits <= 6'd0;
    end else begin
      cmd_bits <= total[5:0];
    end
  end
endtask

reg eng_start = 1'b0;

always @(posedge clk) begin
  cmd_we <= 1'b0;
  eng_start <= 1'b0;

  if(mcu_ack) pending <= 1'b0;
  if(mcu_done) busy <= 1'b0;

  if(rd_strobe & reg_enable) begin
    if(is_proxy) begin
      // proxied write to last_reg; select the following register by default
      last_reg <= last_reg + 7'd1;
      case(last_reg)
        R_MulAL:      mul_a[7:0] <= wval;
        R_MulAH:      mul_a[15:8] <= wval;
        R_MulBL:      mul_b[7:0] <= wval;
        R_MulBH:      mul_b[15:8] <= wval;
        R_RayStartX0: ray_start_x[7:0] <= wval;
        R_RayStartX1: ray_start_x[15:8] <= wval;
        R_RayStartX2: ray_start_x[23:16] <= wval;
        R_RayStartX3: ray_start_x[31:24] <= wval;
        R_RayStartY0: ray_start_y[7:0] <= wval;
        R_RayStartY1: ray_start_y[15:8] <= wval;
        R_RayStartY2: ray_start_y[23:16] <= wval;
        R_RayStartY3: ray_start_y[31:24] <= wval;
        R_RayStartZ0: ray_start_z[7:0] <= wval;
        R_RayStartZ1: ray_start_z[15:8] <= wval;
        R_RayStartZ2: ray_start_z[23:16] <= wval;
        R_RayStartZ3: ray_start_z[31:24] <= wval;
        R_RayDirXL:   ray_dir_x[7:0] <= wval;
        R_RayDirXH:   ray_dir_x[15:8] <= wval;
        R_RayDirYL:   ray_dir_y[7:0] <= wval;
        R_RayDirYH:   ray_dir_y[15:8] <= wval;
        R_RayDirZL:   ray_dir_z[7:0] <= wval;
        R_RayDirZH:   ray_dir_z[15:8] <= wval;
        R_XStepXL:    xstep_x[7:0] <= wval;
        R_XStepXH:    xstep_x[15:8] <= wval;
        R_XStepYL:    xstep_y[7:0] <= wval;
        R_XStepYH:    xstep_y[15:8] <= wval;
        R_XStepZL:    xstep_z[7:0] <= wval;
        R_XStepZH:    xstep_z[15:8] <= wval;
        R_YStepXL:    ystep_x[7:0] <= wval;
        R_YStepXH:    ystep_x[15:8] <= wval;
        R_YStepYL:    ystep_y[7:0] <= wval;
        R_YStepYH:    ystep_y[15:8] <= wval;
        R_YStepZL:    ystep_z[7:0] <= wval;
        R_YStepZH:    ystep_z[15:8] <= wval;
        R_LightXL:    light_x[7:0] <= wval;
        R_LightXH:    light_x[15:8] <= wval;
        R_LightYL:    light_y[7:0] <= wval;
        R_LightYH:    light_y[15:8] <= wval;
        R_LightZL:    light_z[7:0] <= wval;
        R_LightZH:    light_z[15:8] <= wval;
        R_Mode:       full_res <= wval[0];
        R_CmdAddrL: begin
          cmd_addr[7:0] <= wval;
          cmd_bits <= 6'd0;
        end
        R_CmdAddrH: begin
          cmd_addr[15:8] <= wval;
          cmd_bits <= 6'd0;
        end
        R_CmdData1: begin push_bits(4'd1); last_reg <= last_reg; end
        R_CmdData2: begin push_bits(4'd2); last_reg <= last_reg; end
        R_CmdData3: begin push_bits(4'd3); last_reg <= last_reg; end
        R_CmdData4: begin push_bits(4'd4); last_reg <= last_reg; end
        R_CmdData5: begin push_bits(4'd5); last_reg <= last_reg; end
        R_CmdData6: begin push_bits(4'd6); last_reg <= last_reg; end
        R_CmdData7: begin push_bits(4'd7); last_reg <= last_reg; end
        R_CmdData8: begin push_bits(4'd8); last_reg <= last_reg; end
        default: ;
      endcase
    end else begin
      // register read: select as proxy target (except the FB mapping
      // registers, which the SNES reads from its HBlank IRQ)
      if((reg_idx != R_MapUpperFB) && (reg_idx != R_MapLowerFB))
        last_reg <= reg_idx;
      case(reg_idx)
        R_NewFrame: begin
          if(~busy) begin
            busy <= 1'b1;
            pending <= 1'b1;
            read_bank <= ~read_bank;
            frame_half <= ~full_res;
            eng_start <= 1'b1;
          end
        end
        R_MapUpperFB: map_lower <= 1'b0;
        R_MapLowerFB: map_lower <= 1'b1;
        default: ;
      endcase
    end
  end
end

// SNES read data
always @(*) begin
  data_out = 8'h00;
  if(~is_proxy) begin
    case(reg_idx)
      R_MulO0:  data_out = mul_o[7:0];
      R_MulO1:  data_out = mul_o[15:8];
      R_MulO2:  data_out = mul_o[23:16];
      R_MulO3:  data_out = mul_o[31:24];
      R_Status: data_out = {7'h00, busy};
      default:  data_out = 8'h00;
    endcase
  end
end

// ---------------------------------------------------------------------------
// ray tracing engine + pixel FIFO
// ---------------------------------------------------------------------------
// The engine runs on its own clock (eng_clk, see main.v / pll.v): the state
// machine does not reach the 96 MHz of the sd2snes system clock.
// Clock domain crossing:
//   start      toggle, synchronised into eng_clk; the frame parameters are
//              sampled by the engine 2-3 eng_clk cycles after NewFrame, while
//              the SNES cannot issue another register access (>200 ns)
//   FIFO       dual clock RAM, Gray coded pointers
//   restart    the engine reports its FIFO write pointer at the frame start
//              (start_ack toggle + frame_base); the read side then drops
//              anything older (frames abandoned by an MCU reset)
//   busy       2 FF synchroniser; cycle count is diagnostic only (unsynchronised)
wire [14:0] eng_pix;
wire eng_pix_we;
wire [31:0] eng_cycles;

function [11:0] bin2gray; input [11:0] b; bin2gray = b ^ (b >> 1); endfunction
function [11:0] gray2bin;
  input [11:0] g;
  integer i;
  begin
    gray2bin[11] = g[11];
    for(i = 10; i >= 0; i = i - 1) gray2bin[i] = gray2bin[i + 1] ^ g[i];
  end
endfunction

reg [11:0] r_ptr = 12'd0;           // FIFO read pointer (clk domain)
reg [11:0] r_gray = 12'd0;

// --- clk domain: start request
reg start_tgl = 1'b0;
always @(posedge clk) if(eng_start) start_tgl <= ~start_tgl;

// --- eng_clk domain
reg [2:0] e_start_sync = 3'b000;
wire e_start = e_start_sync[2] ^ e_start_sync[1];
reg start_ack = 1'b0;
reg [11:0] e_wptr = 12'd0;          // FIFO write pointer (binary, 1 wrap bit)
reg [11:0] e_wptr_gray = 12'd0;
reg [11:0] frame_base = 12'd0;
reg [11:0] e_rgray_s1 = 12'd0, e_rgray_s2 = 12'd0;
wire [11:0] e_rptr = gray2bin(e_rgray_s2);
wire [11:0] e_fill = e_wptr - e_rptr;
reg e_full = 1'b0;
reg [14:0] fifo [0:2047];

always @(posedge eng_clk) begin
  e_start_sync <= {e_start_sync[1:0], start_tgl};
  e_rgray_s1 <= r_gray;
  e_rgray_s2 <= e_rgray_s1;
  if(eng_pix_we) begin
    fifo[e_wptr[10:0]] <= eng_pix;
    e_wptr <= e_wptr + 12'd1;
    e_wptr_gray <= bin2gray(e_wptr + 12'd1);
  end
  // the read pointer seen here lags: stop well before the real limit
  e_full <= (e_fill >= 12'd2032);
  if(e_start) begin
    // the engine restarts in this cycle; a pixel written now is the last of
    // the old frame
    frame_base <= e_wptr + {11'd0, eng_pix_we};
    start_ack <= e_start_sync[1];
  end
end

srt_engine engine(
  .clk(eng_clk), .start(e_start), .abort(1'b0),
  .i_sx(ray_start_x), .i_sy(ray_start_y), .i_sz(ray_start_z),
  .i_dx(ray_dir_x), .i_dy(ray_dir_y), .i_dz(ray_dir_z),
  .i_xsx(xstep_x), .i_xsy(xstep_y), .i_xsz(xstep_z),
  .i_ysx(ystep_x), .i_ysy(ystep_y), .i_ysz(ystep_z),
  .i_lx(light_x), .i_ly(light_y), .i_lz(light_z),
  .i_half(frame_half),
  .cmd_addr(eng_cmd_addr), .cmd_q(eng_cmd_q),
  .pix_data(eng_pix), .pix_we(eng_pix_we), .pix_full(e_full),
  .busy(eng_busy), .cycles(eng_cycles)
);

// --- clk domain: read side
reg [11:0] r_wgray_s1 = 12'd0, r_wgray_s2 = 12'd0;
reg [2:0] r_ack_sync = 3'b000;
reg [1:0] r_busy_sync = 2'b00;
wire r_restarting = (r_ack_sync[2] != start_tgl);   // engine has not taken the start yet
wire [11:0] r_wptr = gray2bin(r_wgray_s2);
wire [11:0] fifo_count = r_restarting ? 12'd0 : (r_wptr - r_ptr);
reg [10:0] fifo_rptr_r = 11'd0;
wire fifo_pop = mcu_pix_pop & (fifo_count != 12'd0);

always @(posedge clk) begin
  r_wgray_s1 <= e_wptr_gray;
  r_wgray_s2 <= r_wgray_s1;
  r_ack_sync <= {r_ack_sync[1:0], start_ack};
  r_busy_sync <= {r_busy_sync[0], eng_busy};
  if(r_ack_sync[2] != r_ack_sync[1]) begin
    // engine restarted; frame_base has been stable for 2+ clk cycles
    r_ptr <= frame_base;
    r_gray <= bin2gray(frame_base);
  end else if(fifo_pop) begin
    r_ptr <= r_ptr + 12'd1;
    r_gray <= bin2gray(r_ptr + 12'd1);
  end
  fifo_rptr_r <= r_ptr[10:0];
end
wire [14:0] fifo_q = fifo[fifo_rptr_r];
wire eng_busy_c = r_busy_sync[1] | r_restarting;

assign mcu_pix_byte = mcu_ptr[0] ? {1'b0, fifo_q[14:8]} : fifo_q[7:0];

// A6: FIFO level (big endian), engine state, cycles of the current/last frame
reg [11:0] info_count = 12'd0;
reg info_busy = 1'b0;
reg [31:0] info_cycles = 32'd0;
always @(posedge clk) begin
  if(mcu_info_snap) begin
    info_count <= fifo_count;
    info_busy <= eng_busy_c;
    info_cycles <= eng_cycles;
  end
end
always @(*) begin
  case(mcu_ptr[2:0])
    3'd0: mcu_info_byte = {4'h0, info_count[11:8]};
    3'd1: mcu_info_byte = info_count[7:0];
    3'd2: mcu_info_byte = {7'h00, info_busy};
    3'd3: mcu_info_byte = info_cycles[31:24];
    3'd4: mcu_info_byte = info_cycles[23:16];
    3'd5: mcu_info_byte = info_cycles[15:8];
    3'd6: mcu_info_byte = info_cycles[7:0];
    default: mcu_info_byte = 8'h00;
  endcase
end

// MCU side
// bit 3: this core has the hardware engine (FIFO / A5 / A6 available)
assign mcu_status = {2'b00, frame_half, eng_busy_c, 1'b1, read_bank, busy, pending};

reg [7:0] reg_byte_r;
always @(*) begin
  case(mcu_ptr[5:0])
    6'd0:  reg_byte_r = ray_start_x[7:0];
    6'd1:  reg_byte_r = ray_start_x[15:8];
    6'd2:  reg_byte_r = ray_start_x[23:16];
    6'd3:  reg_byte_r = ray_start_x[31:24];
    6'd4:  reg_byte_r = ray_start_y[7:0];
    6'd5:  reg_byte_r = ray_start_y[15:8];
    6'd6:  reg_byte_r = ray_start_y[23:16];
    6'd7:  reg_byte_r = ray_start_y[31:24];
    6'd8:  reg_byte_r = ray_start_z[7:0];
    6'd9:  reg_byte_r = ray_start_z[15:8];
    6'd10: reg_byte_r = ray_start_z[23:16];
    6'd11: reg_byte_r = ray_start_z[31:24];
    6'd12: reg_byte_r = ray_dir_x[7:0];
    6'd13: reg_byte_r = ray_dir_x[15:8];
    6'd14: reg_byte_r = ray_dir_y[7:0];
    6'd15: reg_byte_r = ray_dir_y[15:8];
    6'd16: reg_byte_r = ray_dir_z[7:0];
    6'd17: reg_byte_r = ray_dir_z[15:8];
    6'd18: reg_byte_r = xstep_x[7:0];
    6'd19: reg_byte_r = xstep_x[15:8];
    6'd20: reg_byte_r = xstep_y[7:0];
    6'd21: reg_byte_r = xstep_y[15:8];
    6'd22: reg_byte_r = xstep_z[7:0];
    6'd23: reg_byte_r = xstep_z[15:8];
    6'd24: reg_byte_r = ystep_x[7:0];
    6'd25: reg_byte_r = ystep_x[15:8];
    6'd26: reg_byte_r = ystep_y[7:0];
    6'd27: reg_byte_r = ystep_y[15:8];
    6'd28: reg_byte_r = ystep_z[7:0];
    6'd29: reg_byte_r = ystep_z[15:8];
    6'd30: reg_byte_r = light_x[7:0];
    6'd31: reg_byte_r = light_x[15:8];
    6'd32: reg_byte_r = light_y[7:0];
    6'd33: reg_byte_r = light_y[15:8];
    6'd34: reg_byte_r = light_z[7:0];
    6'd35: reg_byte_r = light_z[15:8];
    default: reg_byte_r = 8'h00;
  endcase
end
assign mcu_reg_byte = reg_byte_r;

endmodule
