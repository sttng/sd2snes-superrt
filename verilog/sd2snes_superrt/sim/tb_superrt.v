// Testbench for the sd2snes "superrt" core (mk3).
//
// Replays a stimulus file produced by the srtemu end-to-end emulator (SNES
// CPU running the SuperRT test ROM + C model of this core + MCU renderer):
//   R aaaaaa vv     SNES read of address aaaaaa, expect vv
//   Q aaaaaa vv     SNES read in the framebuffer window, expect vv
//   S vv            MCU: SPI 0xA0 get status, expect vv
//   P b0 .. b35     MCU: SPI 0xA3 read parameters, expect bytes
//   W wwww...       MCU: SPI 0xA4 command buffer word (512 lines in a row)
//   X n p0 .. pn-1  MCU: read n pixels from the engine FIFO: poll SPI 0xA6
//                   until pixels are available, read them with 0xA5, expect
//                   the RGB555 values p0..
//   U               MSU-1 (core built with SRT_MSU1): enable the MSU-1 feature
//                   bit, read the ID at $2002-$2007 ("S-MSU1"), disable it
//   (S: bit 3 "engine present" must be set; bit 4 "engine busy" is timing
//    dependent and not compared)
//   V               MSU-1 audio path (core built with SRT_MSU1): SNES requests
//                   track 1 at full volume, MCU sees the request (F1/F3), clears
//                   busy, SNES starts play+repeat ($2007), MCU sees the control
//                   request, loads a test tone into the DAC buffer and starts
//                   the DAC; checks the samples coming out of the DAC
//   A / D           MCU: SPI 0xA1 ack / 0xA2 done
//   F aaaaaa vv     MCU: write byte vv to cartridge memory aaaaaa
//   M aaaaaa b*32000  backdoor load of a framebuffer bank (MCU rendered frame)
//
// iverilog -g2005 -DMK3 -o tb tb_superrt.v ip_stubs.v ../*.v && vvp tb +stim=stim.txt +rom=rom.hex
`timescale 1ns / 1ps
module tb;

reg CLKIN = 0;
always #62.5 CLKIN = ~CLKIN;

// SNES
reg [23:0] SNES_ADDR_IN = 24'h000000;
reg SNES_READ_IN = 1, SNES_WRITE_IN = 1, SNES_ROMSEL_IN = 1, SNES_CPU_CLK_IN = 0;
reg SNES_REFRESH = 0, SNES_SYSCLK = 0;
reg [7:0] SNES_PA_IN = 8'hFF;
reg SNES_PARD_IN = 1, SNES_PAWR_IN = 1;
wire [7:0] SNES_DATA;
wire SNES_IRQ, SNES_DATABUS_OE, SNES_DATABUS_DIR;
reg snes_drv = 0;
reg [7:0] snes_dout = 8'h00;
assign SNES_DATA = snes_drv ? snes_dout : 8'hzz;
always #23.28 SNES_SYSCLK = ~SNES_SYSCLK;

// MCU
reg SPI_MOSI = 0, SPI_SS = 1, SPI_SCK = 0;
wire SPI_MISO;
wire MCU_RDY;

// PSRAM
wire [21:0] ROM_ADDR;
wire ROM_1CE, ROM_2CE, ROM_ZZ, ROM_OE, ROM_WE, ROM_BHE, ROM_BLE;
wire [15:0] ROM_DATA;
wire [7:0] RAM_DATA;
wire [18:0] RAM_ADDR;
wire RAM_OE, RAM_WE;
wire DAC_MCLK, DAC_LRCK, DAC_SDOUT, PM6_out, PN6_out;
wire SD_CMD, SD_CLK;

main uut(
  .SNES_CIC_CLK(1'b0), .ROM_ADDR(ROM_ADDR), .ROM_1CE(ROM_1CE), .ROM_2CE(ROM_2CE), .ROM_ZZ(ROM_ZZ),
  .PM6_out(PM6_out), .PN6_out(PN6_out), .PT5_in(1'b0),
  .CLKIN(CLKIN),
  .SNES_ADDR_IN(SNES_ADDR_IN), .SNES_READ_IN(SNES_READ_IN), .SNES_WRITE_IN(SNES_WRITE_IN),
  .SNES_ROMSEL_IN(SNES_ROMSEL_IN), .SNES_DATA(SNES_DATA), .SNES_CPU_CLK_IN(SNES_CPU_CLK_IN),
  .SNES_REFRESH(SNES_REFRESH), .SNES_IRQ(SNES_IRQ), .SNES_DATABUS_OE(SNES_DATABUS_OE),
  .SNES_DATABUS_DIR(SNES_DATABUS_DIR), .SNES_SYSCLK(SNES_SYSCLK),
  .SNES_PA_IN(SNES_PA_IN), .SNES_PARD_IN(SNES_PARD_IN), .SNES_PAWR_IN(SNES_PAWR_IN),
  .ROM_DATA(ROM_DATA), .ROM_OE(ROM_OE), .ROM_WE(ROM_WE), .ROM_BHE(ROM_BHE), .ROM_BLE(ROM_BLE),
  .RAM_DATA(RAM_DATA), .RAM_ADDR(RAM_ADDR), .RAM_OE(RAM_OE), .RAM_WE(RAM_WE),
  .SPI_MOSI(SPI_MOSI), .SPI_MISO(SPI_MISO), .SPI_SS(SPI_SS), .SPI_SCK(SPI_SCK),
  .MCU_RDY(MCU_RDY), .DAC_MCLK(DAC_MCLK), .DAC_LRCK(DAC_LRCK), .DAC_SDOUT(DAC_SDOUT),
  .SD_DAT(4'hF), .SD_CMD(SD_CMD), .SD_CLK(SD_CLK)
);

// ---------------------------------------------------------------- PSRAM model
// byte address = {ROM_ADDR, ROM_1CE (chip select = address bit 1), lane}
// only ROM (0x000000-0x01FFFF) and the framebuffers (0x800000-0x80FFFF) are backed
reg [7:0] mem [0:'h2FFFF];
function [17:0] midx;
  input [23:0] a;
  midx = a[23] ? (18'h20000 | a[15:0]) : {1'b0, a[16:0]};
endfunction
wire [23:0] word_a = {ROM_ADDR, ROM_1CE, 1'b0};
assign ROM_DATA = ROM_WE ? {mem[midx(word_a)], mem[midx(word_a | 24'h1)]} : 16'hzzzz;
always @(*) begin
  if(!ROM_WE) begin
    if(!ROM_BHE) mem[midx(word_a)] = ROM_DATA[15:8];
    if(!ROM_BLE) mem[midx(word_a | 24'h1)] = ROM_DATA[7:0];
  end
end

// ---------------------------------------------------------------- SNES bus
// While the testbench is not doing a cartridge read, the SNES keeps running
// "internal" cycles (WRAM address, /ROMSEL high) so the core has free memory
// slots for MCU accesses, just like on the real console.
reg idle_run = 1, idle_active = 0;
always begin
  if(idle_run) begin
    idle_active = 1;
    SNES_ADDR_IN = 24'h7E0000;
    SNES_ROMSEL_IN = 1;
    SNES_CPU_CLK_IN = 0;
    #186;
    SNES_CPU_CLK_IN = 1;
    #186;
    idle_active = 0;
  end
  #1;
end

task snes_read(input [23:0] a, output [7:0] d);
  begin
    idle_run = 0;
    while(idle_active) #1;
    SNES_ADDR_IN = a;
    SNES_ROMSEL_IN = ~((a[22] | a[15]) & ~(a[23:17] == 7'b0111111));
    SNES_CPU_CLK_IN = 0;
    #186;
    SNES_CPU_CLK_IN = 1;
    SNES_READ_IN = 0;
    #180;
    d = SNES_DATA;
    #6;
    SNES_READ_IN = 1;
    SNES_ROMSEL_IN = 1;
    idle_run = 1;
  end
endtask

task snes_write(input [23:0] a, input [7:0] d);
  begin
    idle_run = 0;
    while(idle_active) #1;
    SNES_ADDR_IN = a;
    SNES_ROMSEL_IN = ~((a[22] | a[15]) & ~(a[23:17] == 7'b0111111));
    SNES_CPU_CLK_IN = 0;
    #186;
    SNES_CPU_CLK_IN = 1;
    SNES_WRITE_IN = 0;
    #40;
    snes_dout = d; snes_drv = 1;
    #146;
    SNES_WRITE_IN = 1;
    #10;
    snes_drv = 0;
    SNES_ROMSEL_IN = 1;
    idle_run = 1;
  end
endtask

// ---------------------------------------------------------------- MCU SPI
localparam SPI_HALF = 24;
task spi_byte(input [7:0] tx, output [7:0] rx);
  integer b;
  begin
    for(b = 7; b >= 0; b = b - 1) begin
      SPI_MOSI = tx[b];
      #SPI_HALF;
      rx[b] = SPI_MISO;
      SPI_SCK = 1;
      #SPI_HALF;
      SPI_SCK = 0;
    end
    #120; // inter-byte gap of the MCU's software loop
  end
endtask

reg [7:0] junk;
task spi_select;   begin SPI_SS = 0; #100; end endtask
task spi_deselect; begin #100; SPI_SS = 1; #200; end endtask
task wait_rdy;     begin #200; while(!MCU_RDY) #10; end endtask

task mcu_cmd1(input [7:0] c);
  begin spi_select; spi_byte(c, junk); spi_deselect; end
endtask

task mcu_set_addr(input [23:0] a);
  begin
    spi_select; wait_rdy;
    spi_byte(8'h00, junk); spi_byte(a[23:16], junk); spi_byte(a[15:8], junk); spi_byte(a[7:0], junk);
    spi_deselect;
  end
endtask

task mcu_write_byte(input [23:0] a, input [7:0] v);
  begin
    mcu_set_addr(a);
    spi_select; spi_byte(8'h98, junk); spi_byte(v, junk); wait_rdy; spi_deselect;
  end
endtask

task mcu_msu_status(output [15:0] st);
  reg [7:0] h, l;
  begin spi_select; spi_byte(8'hF1, junk); spi_byte(8'h00, h); spi_byte(8'h00, l); spi_deselect; st = {h, l}; end
endtask
task mcu_set_msu_status(input [15:0] st);
  begin spi_select; spi_byte(8'hE0, junk); spi_byte(st[7:0], junk); spi_byte(st[15:8], junk); spi_byte(8'h00, junk); spi_deselect; end
endtask
task check8(input [8*24-1:0] what, input [7:0] gotv, input [7:0] expv, input [7:0] mask);
  begin
    checks = checks + 1;
    if((gotv & mask) !== (expv & mask)) begin errors = errors + 1; $display("MSU-1 %0s: got %02x expected %02x (mask %02x)", what, gotv, expv, mask); end
    else $display("MSU-1 %0s: %02x ok", what, gotv);
  end
endtask

`ifdef SRT_MSU1
// DAC output monitor: decode the I2S stream on the DAC_SDOUT/DAC_LRCK pins
// (MSB first, the LSB of a word is shifted out with the next LRCK edge; the bit clock is
// internal to the DAC module, the DAC chip derives it from MCLK)
reg [15:0] i2s_sh = 0;
reg i2s_lr = 0, i2s_take = 0;
integer i2s_words = 0, i2s_nonzero = 0, i2s_max = 0, i2s_val, dac_max = 0, dac_v;
always @(posedge uut.snes_dac.clkin) if(uut.snes_dac.lrck_rising | uut.snes_dac.lrck_falling) begin
  dac_v = uut.snes_dac.vol_sample_sat; if(dac_v < 0) dac_v = -dac_v; if(dac_v > dac_max) dac_max = dac_v;
end
always @(posedge uut.snes_dac.sclk) begin
  i2s_sh = {i2s_sh[14:0], DAC_SDOUT};
  i2s_take = (DAC_LRCK !== i2s_lr);
  if(i2s_take) begin
    i2s_val = $signed(i2s_sh);
    i2s_words = i2s_words + 1;
    if(i2s_val != 0) i2s_nonzero = i2s_nonzero + 1;
    if(i2s_val < 0) i2s_val = -i2s_val;
    if(i2s_val > i2s_max) i2s_max = i2s_val;
  end
  i2s_lr = DAC_LRCK;
end
`endif

// ---------------------------------------------------------------- stimulus
integer fd, r, errors, checks, i, n, k, avail, npix, pix_total;
reg [15:0] pexpv;
reg [7:0] lo, hi;
reg [7:0] info [0:6];
reg [31:0] eng_cycles_last;
reg [47:0] msu_id;
reg [8*4-1:0] op;
reg [23:0] a;
reg [7:0] v, got;
reg [63:0] words [0:511];
reg [63:0] w;
reg [7:0] pexp [0:35];
reg [8*256-1:0] stimfile, romfile;
reg [7:0] romimg [0:'h1FFFF];

initial begin
  errors = 0; checks = 0; pix_total = 0;
  if(!$value$plusargs("stim=%s", stimfile)) stimfile = "stim.txt";
  if(!$value$plusargs("rom=%s", romfile)) romfile = "rom.hex";
  $readmemh(romfile, romimg);
  for(i = 0; i < 'h20000; i = i + 1) mem[i] = romimg[i];
  for(i = 'h20000; i < 'h30000; i = i + 1) mem[i] = 8'h00;

  // FPGA flip-flops power up as 0; give the ones without an initial value a defined state
  #1;
  uut.snes_sd_dma.SD_DMA_STATUSr = 1'b0;
  for(i = 0; i < 512; i = i + 1) uut.snes_superrt.cmdbuf[i] = 64'h0; // M9K powers up cleared
  #3000;
  // the SPI slave's SCK-domain synchroniser powers up as 0 on the FPGA but X in
  // simulation: one dummy message flushes it
  mcu_cmd1(8'hFF);
  // firmware setup: ROM mask, LoROM mapper, no features
  spi_select; spi_byte(8'h10, junk); spi_byte(8'h01, junk); spi_byte(8'hFF, junk); spi_byte(8'hFF, junk); spi_deselect;
  mcu_cmd1(8'h31);
  // a memory read command selects the PSRAM as SD DMA target (as the firmware does)
  mcu_set_addr(24'h000000);
  spi_select; spi_byte(8'h88, junk); wait_rdy; spi_byte(8'h00, junk); spi_deselect;
  spi_select; spi_byte(8'hED, junk); spi_byte(8'h00, junk); spi_byte(8'h00, junk); spi_deselect;

  $display("setup: ROM_MASK=%h MAPPER=%h feat=%h", uut.ROM_MASK, uut.MAPPER, uut.featurebits);
  fd = $fopen(stimfile, "r");
  if(fd == 0) begin $display("cannot open %0s", stimfile); $finish; end
  n = 0;
  while(!$feof(fd)) begin
    r = $fscanf(fd, "%s", op);
    if(r != 1) begin
    end else if(op == "R" || op == "Q") begin
      r = $fscanf(fd, "%h %h", a, v);
      snes_read(a, got);
      checks = checks + 1;
      if(got !== v) begin
        errors = errors + 1;
        if(errors < 30) $display("%0t %0s %06x: got %02x expected %02x", $time, op, a, got, v);
      end
    end else if(op == "S") begin
      r = $fscanf(fd, "%h", v);
      spi_select; spi_byte(8'hA0, junk); spi_byte(8'h00, got); spi_deselect;
      checks = checks + 1;
      if((got[2:0] !== v[2:0]) || (got[3] !== 1'b1)) begin errors = errors + 1; $display("%0t status: got %02x expected %02x", $time, got, v); end
      else $display("%0t status %02x ok", $time, got);
    end else if(op == "X") begin
      r = $fscanf(fd, "%d", npix);
      k = 0;
      while(k < npix) begin
        spi_select; spi_byte(8'hA6, junk);
        for(i = 0; i < 7; i = i + 1) spi_byte(8'h00, info[i]);
        spi_deselect;
        avail = {info[0][3:0], info[1]};
        eng_cycles_last = {info[3], info[4], info[5], info[6]};
        if(avail == 0) begin
          #20000;
        end else begin
          if(avail > npix - k) avail = npix - k;
          spi_select; spi_byte(8'hA5, junk);
          for(i = 0; i < avail; i = i + 1) begin
            spi_byte(8'h00, lo); spi_byte(8'h00, hi);
            r = $fscanf(fd, "%h", pexpv);
            checks = checks + 1;
            if({hi, lo} !== pexpv) begin
              errors = errors + 1;
              if(errors < 30) $display("%0t pixel %0d: got %04x expected %04x", $time, pix_total + k + i, {hi, lo}, pexpv);
            end
          end
          spi_deselect;
          k = k + avail;
        end
      end
      pix_total = pix_total + npix;
      if((pix_total % 16000) == 0) $display("%0t engine pixels streamed (%0d pixels, engine cycles %0d)", $time, pix_total, eng_cycles_last);
    end else if(op == "P") begin
      for(i = 0; i < 36; i = i + 1) r = $fscanf(fd, "%h", pexp[i]);
      spi_select; spi_byte(8'hA3, junk);
      for(i = 0; i < 36; i = i + 1) begin
        spi_byte(8'h00, got);
        checks = checks + 1;
        if(got !== pexp[i]) begin errors = errors + 1; $display("param byte %0d: got %02x expected %02x", i, got, pexp[i]); end
      end
      spi_deselect;
    end else if(op == "W") begin
      r = $fscanf(fd, "%h", words[0]);
      for(i = 1; i < 512; i = i + 1) begin r = $fscanf(fd, "%s", op); r = $fscanf(fd, "%h", words[i]); end
      spi_select; spi_byte(8'hA4, junk);
      for(i = 0; i < 4096; i = i + 1) begin
        spi_byte(8'h00, got);
        w = words[i >> 3];
        checks = checks + 1;
        if(got !== w[63 - 8 * (i & 7) -: 8]) begin
          errors = errors + 1;
          if(errors < 30) $display("cmdbuf byte %0d: got %02x expected %02x", i, got, w[63 - 8 * (i & 7) -: 8]);
        end
      end
      spi_deselect;
      $display("%0t command buffer read back (4096 bytes)", $time);
    end else if(op == "M") begin
      // backdoor: framebuffer bank contents as rendered by the MCU
      r = $fscanf(fd, "%h", a);
      for(i = 0; i < 32000; i = i + 1) begin r = $fscanf(fd, "%h", v); mem[midx(a + i)] = v; end
    end else if(op == "U") begin
`ifdef SRT_MSU1
      spi_select; spi_byte(8'hED, junk); spi_byte(8'h00, junk); spi_byte(8'h08, junk); spi_deselect;
      for(i = 2; i < 8; i = i + 1) begin
        snes_read(24'h002000 + i, got);
        checks = checks + 1;
        msu_id = "S-MSU1";
        if(got !== msu_id[8 * (7 - i) +: 8]) begin
          errors = errors + 1;
          $display("MSU-1 ID byte $%04x: got %02x expected %02x", 16'h2000 + i, got, msu_id[8 * (7 - i) +: 8]);
        end
      end
      spi_select; spi_byte(8'hED, junk); spi_byte(8'h00, junk); spi_byte(8'h00, junk); spi_deselect;
      $display("%0t MSU-1 ID checked", $time);
`else
      $display("core built without MSU-1: U skipped");
`endif
    end else if(op == "V") begin
`ifdef SRT_MSU1
      begin : msu_audio
        reg [15:0] st;
        reg [7:0] th, tl;
        integer smp, w0, nz0;
        // firmware start: MSU-1 feature on (ED), DAC idle, busy cleared (as
        // prepare_audio_track does at start-up)
        spi_select; spi_byte(8'hED, junk); spi_byte(8'h00, junk); spi_byte(8'h08, junk); spi_deselect;
        spi_select; spi_byte(8'hE1, junk); spi_byte(8'h00, junk); spi_deselect;            // dac_pause
        mcu_set_msu_status(16'h2000 | 16'h1000 | 16'h0800 | 16'h0600);                       // clear busy/data busy/error/play/repeat
        snes_read(24'h002000, got); check8("status after init", got, 8'h02, 8'hFF);
        // SNES (ROM code): volume, track 1
        snes_write(24'h002006, 8'hFF);
        snes_write(24'h002004, 8'h01);
        snes_write(24'h002005, 8'h00);
        snes_read(24'h002000, got); check8("status busy", got, 8'h42, 8'hFF);
        // MCU: audio request pending, track number, volume
        mcu_msu_status(st); check8("F1 audio start", st[7:0], 8'h40, 8'h40);
        spi_select; spi_byte(8'hF3, junk); spi_byte(8'h00, th); spi_byte(8'h00, tl); spi_deselect;
        check8("track hi", th, 8'h00, 8'hFF); check8("track lo", tl, 8'h01, 8'hFF);
        spi_select; spi_byte(8'hF4, junk); spi_byte(8'h00, got); spi_deselect; check8("volume", got, 8'hFF, 8'hFF);
        // MCU: track found -> fill DAC buffer (backdoor for the SD DMA: a
        // square wave, +/-8000 on both channels), clear busy + error
        for(smp = 0; smp < 512; smp = smp + 1) begin
          uut.snes_dac.snes_dac_buf.mem[4*smp+0] = (smp & 16) ? 8'h40 : 8'hC0;
          uut.snes_dac.snes_dac_buf.mem[4*smp+1] = (smp & 16) ? 8'h1F : 8'hE0;
          uut.snes_dac.snes_dac_buf.mem[4*smp+2] = (smp & 16) ? 8'h40 : 8'hC0;
          uut.snes_dac.snes_dac_buf.mem[4*smp+3] = (smp & 16) ? 8'h1F : 8'hE0;
        end
        spi_select; spi_byte(8'hE3, junk); spi_byte(8'h00, junk); spi_byte(8'h00, junk); spi_deselect; // dac_reset(0)
        mcu_set_msu_status(16'h0600);
        mcu_set_msu_status(16'h2000 | 16'h0800);
        mcu_msu_status(st); check8("F1 audio start acked", st[7:0], 8'h00, 8'h40);
        snes_read(24'h002000, got); check8("status ready", got, 8'h02, 8'hFF);
        // SNES: play + repeat
        snes_write(24'h002007, 8'h03);
        mcu_msu_status(st); check8("F1 ctrl start play+rpt", st[7:0], 8'h07, 8'h07);
        mcu_set_msu_status(16'h0100);                                   // ack ctrl
        mcu_set_msu_status(16'h0004);                                   // SET_AUDIO_REPEAT
        mcu_set_msu_status(16'h0002);                                   // SET_AUDIO_PLAY
        spi_select; spi_byte(8'hE2, junk); spi_byte(8'h00, junk); spi_deselect; // dac_play
        snes_read(24'h002000, got); check8("status playing", got, 8'h32, 8'hFF);
        w0 = i2s_words; nz0 = i2s_nonzero;
        #4000000;  // 4 ms of audio
        $display("MSU-1 DAC: %0d samples on DAC_SDOUT, %0d non-zero, peak %0d, DAC buffer pointer %0d, volume %0d, internal peak %0d",
                 i2s_words - w0, i2s_nonzero - nz0, i2s_max, uut.snes_dac.dac_address_r, uut.snes_dac.vol_reg, dac_max);
        checks = checks + 1;
        if((i2s_nonzero - nz0) < 100 || i2s_max != dac_max || dac_max != 8000) begin errors = errors + 1; $display("MSU-1 DAC: no audio output"); end
        // MSU-1 off again
        spi_select; spi_byte(8'hE1, junk); spi_byte(8'h00, junk); spi_deselect;
        spi_select; spi_byte(8'hED, junk); spi_byte(8'h00, junk); spi_byte(8'h00, junk); spi_deselect;
      end
`else
      $display("core built without MSU-1: V skipped");
`endif
    end else if(op == "A") begin
      mcu_cmd1(8'hA1);
    end else if(op == "D") begin
      mcu_cmd1(8'hA2);
      $display("%0t frame done", $time);
    end else if(op == "F") begin
      r = $fscanf(fd, "%h %h", a, v);
      mcu_write_byte(a, v);
      checks = checks + 1;
      if(mem[midx(a)] !== v) begin errors = errors + 1; $display("PSRAM write %06x: %02x expected %02x", a, mem[midx(a)], v); end
    end
    n = n + 1;
  end
  $display("stimulus lines %0d, checks %0d, errors %0d", n, checks, errors);
  if(errors == 0) $display("PASS"); else $display("FAIL");
  $finish;
end

endmodule
