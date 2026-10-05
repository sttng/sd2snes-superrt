`timescale 1 ns / 1 ns
//////////////////////////////////////////////////////////////////////////////////
// Company: Rehkopf
// Engineer: Rehkopf
//
// Create Date:    01:13:46 05/09/2009
// Design Name:
// Module Name:    address
// Project Name:
// Target Devices:
// Tool versions:
// Description: Address logic w/ SaveRAM masking
//
// Dependencies:
//
// Revision:
// Additional Comments:
//
//////////////////////////////////////////////////////////////////////////////////
module address(
  input CLK,
  input [15:0] featurebits, // peripheral enable/disable
  input [2:0] MAPPER,       // MCU detected mapper
  input [23:0] SNES_ADDR,   // requested address from SNES
  input [7:0] SNES_PA,      // peripheral address from SNES
  input SNES_ROMSEL,        // SNES ROM access
  output [23:0] ROM_ADDR,   // Address to request from SRAM0
  output ROM_HIT,           // enable SRAM0
  output IS_SAVERAM,        // address/CS mapped as SRAM?
  output IS_ROM,            // address mapped as ROM?
  output IS_WRITABLE,       // address somehow mapped as writable area?
  input [23:0] SAVERAM_MASK,
  input [23:0] ROM_MASK,
  output msu_enable,
  output r213f_enable,
  output r2100_hit,
  output snescmd_enable,
  output nmicmd_enable,
  output return_vector_enable,
  output branch1_enable,
  output branch2_enable,
  output branch3_enable,
  // SuperRT
  input srt_fb_bank,          // framebuffer bank currently shown to the SNES
  input srt_fb_lower,         // window maps the lower half of the image
  output srt_reg_enable       // $BE80-$BFFF register window
);

parameter [2:0]
  FEAT_MSU1 = 3,
  FEAT_213F = 4
;

wire [23:0] SRAM_SNES_ADDR;

/* SuperRT memory map (the original chip only decodes A15-A0 and ignores the
   bank byte, so everything mirrors through all banks with /ROMSEL low):
     A14 = 1        program ROM, offset A14-A0 (i.e. $C000-$FFFF = file 4000-7FFF)
     A14 = 0        $8000-$BE7F: framebuffer window (PSRAM)
                    $BE80-$BFFF: registers (superrt.v)
   The two 32000 byte framebuffer banks live in PSRAM at SRT_FB_BASE and
   SRT_FB_BASE + 0x8000; the MCU renders into the bank that is not shown. */
parameter [23:0] SRT_FB_BASE = 24'h800000;

assign IS_ROM = ~SNES_ROMSEL;
assign IS_SAVERAM = 1'b0;
assign IS_WRITABLE = 1'b0;

wire srt_io = ~SNES_ADDR[14];
assign srt_reg_enable = IS_ROM & srt_io
                        & ((SNES_ADDR[13:7] == 7'h7D) | (SNES_ADDR[13:8] == 6'h3F));

wire [14:0] srt_fb_offset = {1'b0, SNES_ADDR[13:0]} + (srt_fb_lower ? 15'd16000 : 15'd0);

assign SRAM_SNES_ADDR = srt_io
                        ? (SRT_FB_BASE | {8'h00, srt_fb_bank, srt_fb_offset})
                        : ({9'h000, SNES_ADDR[14:0]} & ROM_MASK);

assign ROM_ADDR = SRAM_SNES_ADDR;

assign ROM_SEL = 1'b0;

assign ROM_HIT = IS_ROM;

`ifdef SRT_MSU1
assign msu_enable = featurebits[FEAT_MSU1] & (!SNES_ADDR[22] && ((SNES_ADDR[15:0] & 16'hfff8) == 16'h2000));
`else
assign msu_enable = 1'b0; // built without MSU-1
`endif
assign r213f_enable = featurebits[FEAT_213F] & (SNES_PA == 8'h3f);
assign r2100_hit = (SNES_PA == 8'h00);

assign snescmd_enable = ({SNES_ADDR[22], SNES_ADDR[15:9]} == 8'b0_0010101);
assign nmicmd_enable = (SNES_ADDR == 24'h002BF2);
assign return_vector_enable = (SNES_ADDR == 24'h002A6C);
assign branch1_enable = (SNES_ADDR == 24'h002A1F);
assign branch2_enable = (SNES_ADDR == 24'h002A59);
assign branch3_enable = (SNES_ADDR == 24'h002A5E);
endmodule
