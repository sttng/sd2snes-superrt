`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// gsu_fx3 -- pipelined Super FX core for FX3 carts
//
// Same port list as gsu.v.  This core implements ONLY the FX3 behaviour of gsu.v
// (fx3_r == 1): registers at $x000 (window decoding stays in address.v), R15 and
// the other registers readable while GO, STOP zeroes R15 and raises no IRQ, VCR
// reads $52, RON/RAN ignored, 3MB FX map (ROM below PBR $70), MERGE is the
// firmware dispatcher (R0 = 3/4/5 Clear third A/B/C, C2P = no-op).
//
// Why a separate core: gsu.v walks every instruction byte through two 4-state
// FSMs (fetch ADDR/CACHE/HIT/WAIT, execute DECODE/EXECUTE/MEMORY/WAIT) and can
// therefore never retire more than one byte per 4 CLK (21.4 MHz at CLK=85.7 MHz),
// no matter how gsu_clock_en is set.  FX3 software expects roughly 4x a GSU-2.
// This core retires up to one byte per CLK.
//
// Pipeline (one instruction byte per CLK when the code is in cache):
//
//   F : one cache BRAM read in flight; the byte lands in a 2-entry FIFO (D, B),
//       so D decodes from a flop.  The next address is R15's successor or a
//       redirect from D; the GSU's one-byte delay slot covers the extra stage,
//       so taken branches, LOOP and JMP cost no extra cycle.
//   D : prefixes, operand bytes, branches, JMP, LOOP's jump and IBT/IWT R15 are
//       resolved here.  D also decides X's operands (Rs/Rn/R12/R15/imm, #n/#1)
//       and reads them from the register file.
//   X : one adder (ADD/ADC/SUB/SBC/CMP/INC/DEC/LOOP/LINK/R1+1), a one-LUT logic
//       unit, a shift/byte/GETB unit and a registered result for multi-cycle ops
//       (MULT, FMULT, loads, RPIX), selected 4-way and written to the register
//       file at the end of the cycle.  A result needed by the instruction right
//       behind it is forwarded from a registered copy at the START of that
//       instruction's X cycle.  Z/S are kept lazily as the last result + a bit-7
//       flag, so the 16-input zero test is not in the X path.
//
// Multi-cycle X operations hold D.  An instruction that writes R15 from X (an
// ALU op with TO R15, LJMP, a load into R15, ...) holds the delay slot until the
// new R15 is in place, exactly like the old core; fetch behind the delay slot is
// squashed and restarted at the new R15.
//
// PLOT goes through a 256-line write-back plot cache (see BITMAP), which turns
// column-wise drawing (DOOM: 10368 PLOTs per third, 2 pixels per gsu.v flush)
// into whole-row writes.  Flushed at RPIX, CMODE, MERGE and STOP.
//
// The memory side (ROM/RAM request FSMs, store buffer, ROM prefetcher, Clear
// engine) follows gsu.v, minus the GSU-clock waitstates.
//
// Verification (simulation): random-program differential testing against gsu.v
// in FX3 mode (registers, flags, prefix state, RAM image, held byte), and replay
// of GSU jobs captured from DOOM (FX3) in MesenCE, three-way against gsu.v and
// MesenCE's end state.
//
// Savestates, the MCU state window (only registers/SFR are exposed on PGM_DATA)
// and the debug/config interface are not implemented -- as in gsu.v, savestates
// and FX3 do not coexist.  SS_EN, SPEED, FX3 and the config ports are accepted
// and ignored (FX3 is always on, the core always runs at full speed).
//////////////////////////////////////////////////////////////////////////////////
module gsu_fx3(
  input         RST,
  input         CLK,
  input         CE,        // clock enable: 1 = full rate; the Mk.II toggles it (half rate,
                           // 2-cycle paths inside the core, see sd2snes_gsu3/main.ucf).
                           // Pulse inputs (RST, SNES_*_start/_end) must then be 2 CLK long.
  input         pause,
  input         SS_EN,

  input  [23:0] SAVERAM_MASK,
  input  [23:0] ROM_MASK,

  // MMIO interface
  input         ENABLE,
  input         SNES_RD_start,
  input         SNES_WR_start,
  input         SNES_WR_end,
  input  [9:0]  SNES_ADDR,
  input  [7:0]  DATA_IN,
  output        DATA_ENABLE,
  output [7:0]  DATA_OUT,

  // ROM interface
  input         ROM_BUS_RDY,
  output        ROM_BUS_RRQ,
  output        ROM_BUS_WORD,
  output [23:0] ROM_BUS_ADDR,
  input  [15:0] ROM_BUS_RDDATA,

  // RAM interface
  input         RAM_BUS_RDY,
  output        RAM_BUS_RRQ,
  output        RAM_BUS_WRQ,
  output        RAM_BUS_WORD,
  output [18:0] RAM_BUS_ADDR,
  input  [7:0]  RAM_BUS_RDDATA,
  output [7:0]  RAM_BUS_WRDATA,

  input         FX3,

  output        IRQ,
  output        GO,
  output        RON,
  output        RAN,

  input         SPEED,

  input  [9:0]  PGM_ADDR,
  output [7:0]  PGM_DATA,

  input  [7:0]  reg_group_in,
  input  [7:0]  reg_index_in,
  input  [7:0]  reg_value_in,
  input  [7:0]  reg_invmask_in,
  input         reg_we_in,
  input  [7:0]  reg_read_in,
  output [7:0]  config_data_out,

  output        DBG
);


//-------------------------------------------------------------------
// INPUT FLOPS
//-------------------------------------------------------------------
reg [7:0] data_in_r;
reg [9:0] addr_in_r;
reg       enable_r; initial enable_r = 0;
reg [9:0] pgm_addr_r;
always @(posedge CLK) if (CE) begin
  data_in_r  <= DATA_IN;
  addr_in_r  <= SNES_ADDR;
  enable_r   <= ENABLE;
  pgm_addr_r <= PGM_ADDR;
end

//-------------------------------------------------------------------
// ARCHITECTURAL STATE
//-------------------------------------------------------------------
reg [15:0] REG_r [14:0];               // R0..R14 (R15 is r15_r)
initial begin
  REG_r[0] = 0; REG_r[1] = 0; REG_r[2] = 0; REG_r[3] = 0; REG_r[4] = 0;
  REG_r[5] = 0; REG_r[6] = 0; REG_r[7] = 0; REG_r[8] = 0; REG_r[9] = 0;
  REG_r[10] = 0; REG_r[11] = 0; REG_r[12] = 0; REG_r[13] = 0; REG_r[14] = 0;
end
reg [15:0] r15_r;   initial r15_r = 0;

// SFR, split
reg        f_z;     initial f_z  = 0;   // Z/S as written by the SNES (zs_lazy = 0)
reg        zs_lazy; initial zs_lazy = 0;   // Z/S come from the last X result
reg [15:0] zres_q;  initial zres_q = 0;
reg        s7_q;    initial s7_q = 0;      // LOB/HIB: S is bit 7
reg        f_cy;    initial f_cy = 0;
reg        f_s;     initial f_s  = 0;
reg        f_ov;    initial f_ov = 0;
reg        go_r;    initial go_r = 0;
reg        rr_r;    initial rr_r = 0;   // ROM buffer busy (SFR R)
reg        irq_r;   initial irq_r = 0;
reg [1:0]  sfr_il_r; initial sfr_il_r = 0;
// prefix state (SFR ALT1/ALT2/B + SREG/DREG), owned by the D stage
reg        pf_alt1; initial pf_alt1 = 0;
reg        pf_alt2; initial pf_alt2 = 0;
reg        pf_b;    initial pf_b    = 0;
reg [3:0]  pf_sreg; initial pf_sreg = 0;
reg [3:0]  pf_dreg; initial pf_dreg = 0;

reg [7:0]  BRAMR_r; initial BRAMR_r = 0;
reg [7:0]  PBR_r;   initial PBR_r   = 0;
reg [7:0]  ROMBR_r; initial ROMBR_r = 0;
reg [7:0]  CFGR_r;  initial CFGR_r  = 0;
reg [7:0]  SCBR_r;  initial SCBR_r  = 0;
reg [7:0]  CLSR_r;  initial CLSR_r  = 0;
reg [7:0]  SCMR_r;  initial SCMR_r  = 0;
reg        RAMBR_r; initial RAMBR_r = 0;
reg [15:0] CBR_r;   initial CBR_r   = 0;
reg [7:0]  COLR_r;  initial COLR_r  = 0;
reg [7:0]  POR_r;   initial POR_r   = 0;
reg [7:0]  ROMRDBUF_r; initial ROMRDBUF_r = 0;
reg [15:0] RAMADDR_r;  initial RAMADDR_r  = 0;

// Z and S are evaluated lazily from the registered result of the last
// flag-setting instruction: keeps the 16-input NOR out of the X stage
wire        f_z_w = zs_lazy ? ~|zres_q : f_z;
wire        f_s_w = zs_lazy ? (s7_q ? zres_q[7] : zres_q[15]) : f_s;
wire [15:0] SFR_w = {irq_r, 2'b00, pf_b, sfr_il_r, pf_alt2, pf_alt1,
                     1'b0, rr_r, go_r, f_ov, f_s_w, f_cy, f_z_w, 1'b0};

wire [1:0] SCMR_MD = SCMR_r[1:0];
wire [1:0] SCMR_HT = {SCMR_r[5],SCMR_r[2]};
wire       POR_TRS = POR_r[0];
wire       POR_DTH = POR_r[1];
wire       POR_HN  = POR_r[2];
wire       POR_FHN = POR_r[3];
wire       POR_OBJ = POR_r[4];
wire       CFGR_IRQ = CFGR_r[7];

// register file as one flat vector (R15 = r15_r); every read is a plain
// part-select of it, so simulators and synthesis see the same dependencies
wire [255:0] rflat = {r15_r, REG_r[14], REG_r[13], REG_r[12], REG_r[11], REG_r[10], REG_r[9], REG_r[8],
                      REG_r[7], REG_r[6], REG_r[5], REG_r[4], REG_r[3], REG_r[2], REG_r[1], REG_r[0]};
`define RF(n) rflat[{n,4'h0} +: 16]
// single registers as plain wires for the combinational blocks (XST does not accept
// memory elements in an always @(*) sensitivity list)
wire [15:0] r1_w  = REG_r[1];
wire [15:0] r12_w = REG_r[12];
wire [15:0] r13_w = REG_r[13];

//-------------------------------------------------------------------
// CROSS-UNIT SIGNALS (declared early for XST)
//-------------------------------------------------------------------
// memory requesters
reg        fch_rom_rd_r; initial fch_rom_rd_r = 0;   // fetch: cache fill / uncached (ROM)
reg        fch_ram_rd_r; initial fch_ram_rd_r = 0;   // fetch: cache fill / uncached (RAM)
reg        fch_word_r;   initial fch_word_r   = 0;
reg [23:0] fch_addr_r;   initial fch_addr_r   = 0;
reg        prf_rom_rd_r; initial prf_rom_rd_r = 0;
reg [23:0] prf_addr_r;   initial prf_addr_r   = 0;
reg        exe_ram_rd_r; initial exe_ram_rd_r = 0;
reg        exe_word_r;   initial exe_word_r   = 0;
reg [23:0] exe_addr_r;   initial exe_addr_r   = 0;
reg        stb_ram_wr_r; initial stb_ram_wr_r = 0;
reg [23:0] stb_addr_r;   initial stb_addr_r   = 0;
reg [7:0]  stb_data_r;   initial stb_data_r   = 0;
reg        bmp_ram_rd_r; initial bmp_ram_rd_r = 0;
reg [23:0] bmp_addr_r;   initial bmp_addr_r   = 0;
reg        bmf_ram_rd_r; initial bmf_ram_rd_r = 0;
reg        bmf_ram_wr_r; initial bmf_ram_wr_r = 0;
reg [23:0] bmf_addr_r;   initial bmf_addr_r   = 0;
reg [7:0]  bmf_data_r;   initial bmf_data_r   = 0;
reg        clr_ram_wr_r; initial clr_ram_wr_r = 0;

// X -> units
wire       x2c_flush_w;  // cache flush (CACHE w/ change, LJMP) at this edge
wire [15:0] x2c_cbr_w;
reg        x2m_go;       // MERGE clear request
reg [1:0]  x2m_third;

// Clear engine address/data (engine itself at the end)
reg [16:0] fx3_clr_addr_r; initial fx3_clr_addr_r = 0;
reg [10:0] fx3_clr_cnt_r;  initial fx3_clr_cnt_r = 0;
reg [3:0]  fx3_clr_col_r;  initial fx3_clr_col_r = 0;
wire [7:0] clr_byte_w = (~|fx3_clr_cnt_r[5:4]) ? (fx3_clr_cnt_r[0] ? 8'h00 : 8'hFF)
                      : (&fx3_clr_cnt_r[5:4])  ? (fx3_clr_cnt_r[0] ? 8'hFF : 8'h00)
                      :                          8'h00;
wire [18:0] clr_addr_w = {2'b00, fx3_clr_addr_r};

// unit status
reg        stb_busy_r;   initial stb_busy_r = 0;
reg        fx3_clr_busy_r; initial fx3_clr_busy_r = 0;

//-------------------------------------------------------------------
// CACHE RAM
//-------------------------------------------------------------------
reg        cache_mmio_wren_r; initial cache_mmio_wren_r = 0;
reg  [7:0] cache_mmio_wrdata_r;
reg  [8:0] cache_mmio_addr_r;  initial cache_mmio_addr_r = 0;

wire       cache_gsu_wren;
wire [8:0] cache_gsu_addr;
wire [7:0] cache_gsu_wrdata;

wire       cache_wren   = go_r ? cache_gsu_wren   : cache_mmio_wren_r;
wire [8:0] cache_addr   = go_r ? cache_gsu_addr   : cache_mmio_addr_r;
wire [7:0] cache_wrdata = go_r ? cache_gsu_wrdata : cache_mmio_wrdata_r;
wire [7:0] cache_rddata;

// Half-rate builds (always on Mk.II, GSU3_HALF) need every RAM/pipeline register
// of the core to follow CE, so the cache and FMULT are inferred there.
`ifdef MK2
 `define FX3_CE_RAMS
`endif
`ifdef GSU3_HALF
 `define FX3_CE_RAMS
`endif
`ifdef FX3_CE_RAMS
// inferred instead of the gsu_cache IP so the RAM follows CE (write-first, 1 cycle)
reg [7:0] cache_mem [0:511];
reg [7:0] cache_q;
always @(posedge CLK) if (CE) begin
  if (cache_wren) begin cache_mem[cache_addr] <= cache_wrdata; cache_q <= cache_wrdata; end
  else cache_q <= cache_mem[cache_addr];
end
assign cache_rddata = cache_q;
`else
gsu_cache cache (
  .clock(CLK),
  .wren(cache_wren),
  .address(cache_addr),
  .data(cache_wrdata),
  .q(cache_rddata)
);
`endif

reg [31:0] cache_val_r; initial cache_val_r = 0;

//-------------------------------------------------------------------
// MMIO
//-------------------------------------------------------------------
reg        data_enable_r; initial data_enable_r = 0;
reg [7:0]  data_out_r;    initial data_out_r = 0;
reg [7:0]  data_flop_r;   initial data_flop_r = 0;

reg        wb_val_r;  initial wb_val_r = 0;   // SNES write buffer
reg        wb_reg_r;  initial wb_reg_r = 0;
reg        wb_gpr_r;  initial wb_gpr_r = 0;
reg        wb_cok_r;  initial wb_cok_r = 0;
reg [8:0]  wb_addr_r; initial wb_addr_r = 0;
reg [7:0]  wb_data_r; initial wb_data_r = 0;
reg        rb_sfrh_r; initial rb_sfrh_r = 0;  // SFR high byte was read (clears IRQ)

// snes-side register writes, decoded (consumed the cycle after capture)
wire [15:0] mmio_reg = `RF(addr_in_r[4:1]);
wire wb_go      = wb_val_r;
wire wb_gpr_w   = wb_go & wb_gpr_r & wb_addr_r[0];          // high byte -> register write
wire [3:0] wb_gpr_n = wb_addr_r[4:1];
wire wb_r15     = wb_gpr_w & (wb_gpr_n == 4'd15);
wire wb_r14     = wb_gpr_w & (wb_gpr_n == 4'd14);
wire wb_sfr_l   = wb_go & wb_reg_r & (wb_addr_r[7:0] == 8'h30);
wire wb_sfr_h   = wb_go & wb_reg_r & (wb_addr_r[7:0] == 8'h31);
wire wb_pbr     = wb_go & wb_reg_r & (wb_addr_r[7:0] == 8'h34);
wire wb_go_clr  = wb_sfr_l & go_r & ~wb_data_r[5];          // GO 1->0 by the SNES

always @(posedge CLK) if (CE) begin
  if (RST) begin
    data_enable_r <= 0;
    wb_val_r <= 0; wb_reg_r <= 0; wb_gpr_r <= 0; wb_cok_r <= 0;
    rb_sfrh_r <= 0;
    cache_mmio_wren_r <= 0;
    data_flop_r <= 0;
  end
  else begin
    if (enable_r) begin
      if (SNES_RD_start) begin
        if (~|addr_in_r[9:8]) begin
          casex (addr_in_r[7:0])
            8'b000x_xxx0: begin data_out_r <= mmio_reg[7:0];  data_enable_r <= 1; end
            8'b000x_xxx1: begin data_out_r <= mmio_reg[15:8]; data_enable_r <= 1; end
            8'h30: begin data_out_r <= SFR_w[7:0];  data_enable_r <= 1; end
            8'h31: begin data_out_r <= SFR_w[15:8]; data_enable_r <= 1; end
            8'h34: begin data_out_r <= PBR_r;       data_enable_r <= 1; end
            8'h36: begin data_out_r <= ROMBR_r;     data_enable_r <= 1; end
            8'h3B: begin data_out_r <= 8'h52;       data_enable_r <= 1; end
            8'h3C: begin data_out_r <= {7'h00,RAMBR_r}; data_enable_r <= 1; end
            8'h3E: begin data_out_r <= CBR_r[7:0];  data_enable_r <= 1; end
            8'h3F: begin data_out_r <= CBR_r[15:8]; data_enable_r <= 1; end
          endcase
        end
        else begin
          data_enable_r <= 1;
          cache_mmio_addr_r <= {~addr_in_r[8],addr_in_r[7:0]};
        end
      end
      else if (|addr_in_r[9:8]) begin
        // the fourth quarter ($x300-$x3FF) reads $00 in FX3
        data_out_r <= (&addr_in_r[9:8]) ? 8'h00 : cache_rddata;
      end
    end
    else begin
      data_enable_r <= 0;
    end

    rb_sfrh_r <= SNES_RD_start & enable_r & (addr_in_r[9:0] == 10'h031);

    if (SNES_WR_end & enable_r) begin
      wb_val_r  <= 1;
      wb_reg_r  <= ~|addr_in_r[9:8];
      wb_gpr_r  <= ~|addr_in_r[9:5];
      wb_cok_r  <= ~(&addr_in_r[9:8]);
      wb_addr_r <= addr_in_r[8:0];
      wb_data_r <= data_in_r;
    end
    else begin
      wb_val_r <= 0;
    end

    // cache window write
    cache_mmio_wren_r <= 0;
    if (wb_go) begin
      if (~wb_reg_r) begin
        cache_mmio_wren_r   <= wb_cok_r;
        cache_mmio_wrdata_r <= wb_data_r;
        cache_mmio_addr_r   <= {~wb_addr_r[8],wb_addr_r[7:0]};
      end
      else if (wb_gpr_r & ~wb_addr_r[0]) begin
        data_flop_r <= wb_data_r;
      end
    end
  end
end

//-------------------------------------------------------------------
// PIPELINE DECLARATIONS
//-------------------------------------------------------------------
// fetch
parameter FS_IDLE = 3'd0, FS_RUN = 3'd1, FS_MISS = 3'd2, FS_FILL = 3'd3,
          FS_FILLW = 3'd4, FS_UNC = 3'd5, FS_REPLAY = 3'd7;
reg [2:0]  fs_r;      initial fs_r = FS_IDLE;
reg [7:0]  alt_byte_r; initial alt_byte_r = 8'h01;  // byte held across STOP (runs first on restart)

// D stage
reg [7:0]  d_op_r;    initial d_op_r = 0;     // opcode of a multi-byte instruction
reg [1:0]  d_cnt_r;   initial d_cnt_r = 0;    // operand bytes still to come
reg [7:0]  d_lo_r;    initial d_lo_r = 0;

// X stage
parameter XC_ALU = 4'd0, XC_MULT = 4'd1, XC_FMULT = 4'd2, XC_LD = 4'd3, XC_ST = 4'd4,
          XC_GETB = 4'd5, XC_DF = 4'd6, XC_PLOT = 4'd7, XC_RPIX = 4'd8, XC_COLOR = 4'd9,
          XC_CACHE = 4'd10, XC_STOP = 4'd11, XC_LJMP = 4'd12, XC_MERGE = 4'd13, XC_NONE = 4'd15;
parameter XU_ADD = 2'd0, XU_LOG = 2'd1, XU_SHB = 2'd2, XU_MRES = 2'd3;
parameter LO_AND = 3'd0, LO_BIC = 3'd1, LO_OR = 3'd2, LO_XOR = 3'd3, LO_NOT = 3'd4, LO_PASS = 3'd5;
parameter SO_LSR = 4'd0, SO_ASR = 4'd1, SO_DIV2 = 4'd2, SO_ROL = 4'd3, SO_ROR = 4'd4, SO_SWAP = 4'd5,
          SO_SEX = 4'd6, SO_LOB = 4'd7, SO_HIB = 4'd8, SO_GETB = 4'd9, SO_GETBH = 4'd10, SO_GETBL = 4'd11, SO_GETBS = 4'd12;
parameter FC_NONE = 3'd0, FC_ADD = 3'd1, FC_A0 = 3'd2, FC_A15 = 3'd3, FC_MUL = 3'd4;
parameter AS_RS = 3'd0, AS_RN = 3'd1, AS_R12 = 3'd2, AS_R15 = 3'd3, AS_IMM = 3'd4, AS_R1 = 3'd5;
parameter BS_RN = 2'd0, BS_N = 2'd1, BS_ONE = 2'd2;

reg        x_v;      initial x_v = 0;
reg [3:0]  x_cls;    initial x_cls = XC_NONE;
reg        x_alt1;   initial x_alt1 = 0;
reg        x_alt2;   initial x_alt2 = 0;
reg [3:0]  x_dest;   initial x_dest = 0;
reg        x_wa;     initial x_wa = 0;       // writes x_dest
reg        x_r15w;   initial x_r15w = 0;     // ... and x_dest is R15 (from X)
reg [15:0] x_a;      initial x_a = 0;
reg [15:0] x_b;      initial x_b = 0;
reg [15:0] x_r2;     initial x_r2 = 0;
reg [15:0] x_r6;     initial x_r6 = 0;
reg [1:0]  x_fa, x_fb, x_f2, x_f6;           // forward select: 0 as read, 1 wa_q, 2 wb_q
initial begin x_fa = 0; x_fb = 0; x_f2 = 0; x_f6 = 0; end
reg [15:0] x_imm;    initial x_imm = 0;
reg [15:0] x_r15;    initial x_r15 = 0;
reg [7:0]  x_opc;    initial x_opc = 0;
reg        x_ldmode; initial x_ldmode = 0;   // XC_LD/XC_ST: 0 = (Rn), 1 = absolute x_imm
reg [1:0]  x_unit;   initial x_unit = 0;
reg [2:0]  x_lop;    initial x_lop = 0;
reg [3:0]  x_sop;    initial x_sop = 0;
reg        x_sub, x_cin1, x_cinf, x_fzs, x_s7;
reg [2:0]  x_fcy;
reg [1:0]  x_fov;
reg        x_one, x_wstb, x_wrr, x_wdbv, x_wstop, x_wpq, x_wpc;
initial begin x_sub = 0; x_cin1 = 0; x_cinf = 0; x_fzs = 0; x_s7 = 0; x_fcy = 0; x_fov = 0;
              x_one = 0; x_wstb = 0; x_wrr = 0; x_wdbv = 0; x_wstop = 0; x_wpq = 0; x_wpc = 0; end
reg [3:0]  x_st;     initial x_st = 0;       // multi-cycle sub-state
reg        x_fin;    initial x_fin = 0;      // multi-cycle op finishes this cycle
reg        x_finh;   initial x_finh = 0;     // ... finished, but could not retire yet (held)
wire       x_done;

// last register writes (forwarding sources)
reg [15:0] wa_q; initial wa_q = 0;
reg [15:0] wb_q; initial wb_q = 0;

wire d_adv;          // D consumes its byte this cycle
reg        d_redir;  // ... and redirects R15 (taken branch, LOOP, JMP, IWT/IBT R15)
reg [15:0] d_target;

// FETCH
//-------------------------------------------------------------------
// The byte D decodes comes from a flop (d_byte_r), not straight from the cache
// BRAM: a 2-entry FIFO (D, B) is fed by one cache read in flight.  The GSU's
// one-byte delay slot covers the extra stage -- a redirect decided in D is
// presented to the cache right behind the delay slot, so taken branches, LOOP
// and JMP still cost nothing.
wire fetch_rom = (PBR_r < 8'h70);
function [23:0] code_addr;
  input [15:0] a;
  input [7:0]  pbr;
  input [23:0] rmask;
  input [23:0] smask;
  begin
    code_addr = (pbr < 8'h70) ? ((pbr[6] ? {pbr,a} : {1'b0,pbr,a[14:0]}) & rmask)
                              : (24'hE00000 + ({7'h00,pbr[0],a} & smask));
  end
endfunction

reg         d_v_r;    initial d_v_r = 0;
reg  [7:0]  d_byte_r; initial d_byte_r = 8'h01;
reg  [15:0] d_addr_r; initial d_addr_r = 0;
reg         b_v_r;    initial b_v_r = 0;
reg  [7:0]  b_byte_r; initial b_byte_r = 0;
reg  [15:0] b_addr_r; initial b_addr_r = 0;
reg         if_v_r;   initial if_v_r = 0;       // cache read presented at the last edge
reg  [15:0] if_addr_r; initial if_addr_r = 0;
reg  [15:0] lp_r;     initial lp_r = 0;         // last presented address (sequential next = lp_r + 1)
reg         pend_v_r; initial pend_v_r = 0;     // a redirect target is waiting to be presented
reg         pend_skip_r; initial pend_skip_r = 0; // ... after one more sequential byte (the delay slot)
reg  [15:0] pend_t_r; initial pend_t_r = 0;
reg  [15:0] mf_addr_r; initial mf_addr_r = 0;   // missed byte being fetched

wire        d_bv    = go_r & d_v_r;
wire [7:0]  d_byte  = d_byte_r;

wire [15:0] if_off  = if_addr_r - CBR_r;
reg         mf_inwin_r; initial mf_inwin_r = 0;  // missed byte lies in the cache window
wire        if_hit  = ~|if_off[15:9] & cache_val_r[if_addr_r[8:4]];
wire        f_arrive = (fs_r == FS_RUN) & if_v_r & if_hit;
wire        f_miss   = (fs_r == FS_RUN) & if_v_r & ~if_hit;
// fetch is not in the middle of a fill / uncached read (X waits for this
// before it flushes the cache or writes R15)
wire        f_quiet  = (fs_r == FS_RUN) | (fs_r == FS_IDLE);

// line fill / uncached buffer
reg  [3:0]  fill_cnt_r;  initial fill_cnt_r = 0;
reg  [15:0] ubuf_addr_r; initial ubuf_addr_r = 0;
reg  [15:0] ubuf_data_r; initial ubuf_data_r = 0;
reg         ubuf_v_r;    initial ubuf_v_r = 0;
reg  [7:0]  fill_hi_r;   initial fill_hi_r = 0;
reg         fill_wr_r;   initial fill_wr_r = 0;
reg  [8:0]  fill_waddr_r; initial fill_waddr_r = 0;
reg  [7:0]  fill_wdata_r; initial fill_wdata_r = 0;
reg         fill_done_r; initial fill_done_r = 0;
reg  [4:0]  fill_line_r; initial fill_line_r = 0;

wire rom_fch_end, rom_prf_end, ram_fch_end;
reg [15:0] rom_bus_data_r; initial rom_bus_data_r = 0;
reg [15:0] ram_bus_data_r; initial ram_bus_data_r = 0;

wire x_stop_done;
wire x_squash;              // X wrote R15 / flushed the cache this cycle
wire [15:0] x_squash_t;     // ... and fetch continues here

// FIFO push from the uncached path (fills replay through the cache instead)
reg         f_altpush;
reg  [7:0]  f_altbyte;
always @(*) begin
  f_altpush = 0; f_altbyte = rom_bus_data_r[7:0];
  if (fs_r == FS_MISS & mf_inwin_r) f_altpush = 0;
  else if (fs_r == FS_MISS & fetch_rom & ubuf_v_r & (ubuf_addr_r[15:1] == mf_addr_r[15:1])) begin
    f_altpush = 1; f_altbyte = mf_addr_r[0] ? ubuf_data_r[15:8] : ubuf_data_r[7:0];
  end
  else if (fs_r == FS_UNC & fch_rom_rd_r & rom_fch_end) begin
    f_altpush = 1; f_altbyte = mf_addr_r[0] ? rom_bus_data_r[15:8] : rom_bus_data_r[7:0];
  end
  else if (fs_r == FS_UNC & fch_ram_rd_r & ram_fch_end) begin
    f_altpush = 1; f_altbyte = ram_bus_data_r[7:0];
  end
end

wire        f_push    = f_arrive | f_altpush;
wire [7:0]  f_pbyte   = f_arrive ? cache_rddata : f_altbyte;
wire [15:0] f_paddr   = f_arrive ? if_addr_r : mf_addr_r;
wire [1:0]  f_occ     = {1'b0, d_v_r} + {1'b0, b_v_r};
wire [1:0]  f_occ_nx  = f_occ - {1'b0, d_adv} + {1'b0, f_push};
wire        d_redir_now = d_adv & d_redir;
// the byte behind D is already on its way: in B, in flight, or being fetched
// after a miss (presentation stops while a miss is handled)
wire        f_ahead   = b_v_r | if_v_r | ((fs_r != FS_RUN) & (fs_r != FS_IDLE));
wire        f_present = go_r & (fs_r == FS_RUN) & ~f_miss & (f_occ_nx <= 2'd1) & ~x_squash & ~x_stop_done;
wire [15:0] f_seq     = lp_r + 16'd1;
wire [15:0] f_pa      = (d_redir_now & f_ahead) ? d_target : (pend_v_r & ~pend_skip_r) ? pend_t_r : f_seq;
wire        f_replay  = (fs_r == FS_REPLAY);

assign cache_gsu_wren   = fill_wr_r;
assign cache_gsu_addr   = fill_wr_r ? fill_waddr_r : f_replay ? mf_addr_r[8:0] : f_pa[8:0];
assign cache_gsu_wrdata = fill_wdata_r;

always @(posedge CLK) if (CE) begin
  if (RST) begin
    fs_r <= FS_IDLE;
    d_v_r <= 0; b_v_r <= 0; if_v_r <= 0; pend_v_r <= 0;
    fch_rom_rd_r <= 0; fch_ram_rd_r <= 0;
    fill_wr_r <= 0; fill_done_r <= 0;
    ubuf_v_r <= 0;
    alt_byte_r <= 8'h01;
  end
  else begin
    fill_wr_r   <= 0;
    fill_done_r <= 0;
    if (wb_pbr | (x_done & (x_cls == XC_LJMP))) ubuf_v_r <= 0;

    // ---------------- FIFO (D = head, B)
    if (d_adv) begin
      if (b_v_r) begin
        d_byte_r <= b_byte_r; d_addr_r <= b_addr_r;
        b_v_r <= f_push;
        b_byte_r <= f_pbyte; b_addr_r <= f_paddr;
      end
      else begin
        d_v_r <= f_push;
        d_byte_r <= f_pbyte; d_addr_r <= f_paddr;
      end
    end
    else if (~d_v_r) begin
      d_v_r <= f_push;
      d_byte_r <= f_pbyte; d_addr_r <= f_paddr;
    end
    else if (f_push) begin
      b_v_r <= 1;
      b_byte_r <= f_pbyte; b_addr_r <= f_paddr;
    end

    // ---------------- presentation / next address
    if (f_present) begin
      if_v_r <= 1;
      if_addr_r <= f_pa;
      lp_r <= f_pa;
      if (d_redir_now & ~f_ahead)       begin pend_v_r <= 1; pend_skip_r <= 0; pend_t_r <= d_target; end // f_pa was the delay slot
      else if (pend_v_r & pend_skip_r)  pend_skip_r <= 0;                                                 // ditto, for an older redirect
      else                              pend_v_r <= 0;
    end
    else begin
      if (fs_r == FS_RUN) if_v_r <= 0;
      if (d_redir_now) begin pend_v_r <= 1; pend_skip_r <= ~f_ahead; pend_t_r <= d_target; end
    end

    // ---------------- miss handling
    case (fs_r)
      FS_IDLE: begin
        // A code fetch can still be pending when the run ends (STOP retires with an
        // uncached read or a line fill of the byte behind it in flight).  The ROM/RAM
        // bus FSMs keep serving fch_*_rd_r on their own, so drain it here: otherwise
        // the request is re-issued for as long as the GSU sits stopped, and the next
        // run's first miss takes one of those stale completions as its own byte
        // (Star Fox FX3 hung this way: a run entered at $B19F executed the byte of
        // $81BE that the previous run had left in flight).
        if (fch_rom_rd_r & rom_fch_end) fch_rom_rd_r <= 0;
        if (fch_ram_rd_r & ram_fch_end) fch_ram_rd_r <= 0;
        // GO raised: the byte held from the last STOP executes first
        if (go_r & ~wb_go_clr & ~fch_rom_rd_r & ~fch_ram_rd_r) begin
          d_v_r <= 1; d_byte_r <= alt_byte_r; d_addr_r <= r15_r - 16'd1;
          b_v_r <= 0; if_v_r <= 0; pend_v_r <= 0;
          lp_r <= r15_r - 16'd1;
          fs_r <= FS_RUN;
        end
      end
      FS_RUN: begin
        if (~go_r) fs_r <= FS_IDLE;
        else if (f_miss) begin
          mf_addr_r <= if_addr_r;
          mf_inwin_r <= ~|if_off[15:9];
          fs_r <= FS_MISS;
        end
      end
      FS_MISS: begin
        if (mf_inwin_r) begin
          if (fetch_rom | ~stb_busy_r) begin
            fill_cnt_r  <= 0;
            fill_line_r <= mf_addr_r[8:4];
            fch_word_r  <= fetch_rom;
            fch_addr_r  <= code_addr({mf_addr_r[15:4],4'h0}, PBR_r, ROM_MASK, SAVERAM_MASK);
            fch_rom_rd_r <= fetch_rom;
            fch_ram_rd_r <= ~fetch_rom;
            fs_r <= FS_FILL;
          end
        end
        else if (f_altpush) begin
          fs_r <= FS_RUN;                      // served from the uncached word buffer
        end
        else if (fetch_rom | ~stb_busy_r) begin
          fch_word_r  <= fetch_rom;
          fch_addr_r  <= code_addr(fetch_rom ? {mf_addr_r[15:1],1'b0} : mf_addr_r, PBR_r, ROM_MASK, SAVERAM_MASK);
          fch_rom_rd_r <= fetch_rom;
          fch_ram_rd_r <= ~fetch_rom;
          fs_r <= FS_UNC;
        end
      end
      FS_FILL: begin
        if (fch_rom_rd_r & rom_fch_end) begin
          fch_rom_rd_r <= 0;
          fill_wr_r    <= 1;
          fill_waddr_r <= {fill_line_r, fill_cnt_r};
          fill_wdata_r <= rom_bus_data_r[7:0];
          fill_hi_r    <= rom_bus_data_r[15:8];
          fs_r <= FS_FILLW;
        end
        else if (fch_ram_rd_r & ram_fch_end) begin
          fch_ram_rd_r <= 0;
          fill_wr_r    <= 1;
          fill_waddr_r <= {fill_line_r, fill_cnt_r};
          fill_wdata_r <= ram_bus_data_r[7:0];
          fs_r <= FS_FILLW;
        end
      end
      FS_FILLW: begin
        if (fch_word_r & ~fill_cnt_r[0]) begin
          fill_wr_r    <= 1;
          fill_waddr_r <= {fill_line_r, fill_cnt_r | 4'h1};
          fill_wdata_r <= fill_hi_r;
          fill_cnt_r   <= fill_cnt_r | 4'h1;
        end
        else if (&fill_cnt_r) begin
          fill_done_r <= 1;
          fs_r <= FS_REPLAY;
        end
        else begin
          fill_cnt_r   <= fill_cnt_r + 1;
          fch_addr_r   <= code_addr({mf_addr_r[15:4], fill_cnt_r + 4'h1}, PBR_r, ROM_MASK, SAVERAM_MASK);
          fch_rom_rd_r <= fch_word_r;
          fch_ram_rd_r <= ~fch_word_r;
          fs_r <= FS_FILL;
        end
      end
      FS_REPLAY: begin
        // present the missed address again; its line is valid by the time it arrives
        if_v_r <= 1;
        if_addr_r <= mf_addr_r;
        fs_r <= FS_RUN;
      end
      FS_UNC: begin
        if (fch_rom_rd_r & rom_fch_end) begin
          fch_rom_rd_r <= 0;
          ubuf_v_r    <= 1;
          ubuf_addr_r <= {mf_addr_r[15:1],1'b0};
          ubuf_data_r <= rom_bus_data_r;
          fs_r <= FS_RUN;
        end
        else if (fch_ram_rd_r & ram_fch_end) begin
          fch_ram_rd_r <= 0;
          fs_r <= FS_RUN;
        end
      end
      default: fs_r <= FS_IDLE;
    endcase

    // ---------------- squash everything behind D (X wrote R15 / flushed)
    if (x_squash) begin
      // X only squashes while fetch is quiet (RUN); a wrong-path byte missing in
      // this very cycle must not start a miss either
      b_v_r <= 0; if_v_r <= 0;
      fs_r <= FS_RUN;
      if (d_adv) begin
        // only a flush can coincide with D advancing (R15 writers hold D)
        d_v_r <= 0;
        lp_r <= d_addr_r;
        pend_v_r <= d_redir; pend_skip_r <= 1; pend_t_r <= d_target;
      end
      else begin
        // the new address is presented from the (registered) pending target
        pend_v_r <= 1; pend_skip_r <= 0; pend_t_r <= x_squash_t;
      end
    end

    // ---------------- STOP retires / SNES clears GO: keep the D byte for the next run
    if (x_stop_done) begin
      alt_byte_r <= d_byte_r;
      d_v_r <= 0; b_v_r <= 0; if_v_r <= 0;
      fs_r <= FS_IDLE;
    end
    else if (wb_go_clr) begin
      if (d_v_r) alt_byte_r <= d_byte_r;
      d_v_r <= 0; b_v_r <= 0; if_v_r <= 0;
      // a pending code fetch is drained in FS_IDLE (see there), not dropped: one
      // already on the bus would still complete and could be taken by the next run
      fs_r <= FS_IDLE;
    end
  end
end

// cache valid bits / CBR
always @(posedge CLK) if (CE) begin
  if (RST) begin
    cache_val_r <= 0;
    CBR_r <= 0;
  end
  else begin
    if (x2c_flush_w | wb_go_clr | (wb_pbr & ~go_r)) begin
      cache_val_r <= 0;
    end
    else if (fill_done_r) begin
      cache_val_r[fill_line_r] <= 1;
    end
    else if (cache_mmio_wren_r & &cache_mmio_addr_r[3:0]) begin
      cache_val_r[cache_mmio_addr_r[8:4]] <= 1;
    end

    if (wb_go_clr) CBR_r[15:4] <= 0;
    else if (x2c_flush_w) CBR_r[15:4] <= x2c_cbr_w[15:4];
  end
end

// D STAGE
//-------------------------------------------------------------------
wire [7:0] d_opc  = (|d_cnt_r) ? d_op_r : d_byte;    // effective opcode
wire [3:0] d_n    = d_opc[3:0];
wire       d_first = ~|d_cnt_r;

// opcode classes of the effective opcode
wire op_stop   = d_opc == 8'h00;
wire op_nop    = d_opc == 8'h01;
wire op_cache  = d_opc == 8'h02;
wire op_lsr    = d_opc == 8'h03;
wire op_rol    = d_opc == 8'h04;
wire op_br     = (d_opc[7:4] == 4'h0) & (d_opc[3:0] >= 4'h5);
wire op_to     = d_opc[7:4] == 4'h1;
wire op_with   = d_opc[7:4] == 4'h2;
wire op_st     = (d_opc[7:4] == 4'h3) & (d_opc[3:0] <= 4'hB);
wire op_loop   = d_opc == 8'h3C;
wire op_alt1   = d_opc == 8'h3D;
wire op_alt2   = d_opc == 8'h3E;
wire op_alt3   = d_opc == 8'h3F;
wire op_ld     = (d_opc[7:4] == 4'h4) & (d_opc[3:0] <= 4'hB);
wire op_plot   = d_opc == 8'h4C;
wire op_swap   = d_opc == 8'h4D;
wire op_color  = d_opc == 8'h4E;
wire op_not    = d_opc == 8'h4F;
wire op_add    = d_opc[7:4] == 4'h5;
wire op_sub    = d_opc[7:4] == 4'h6;
wire op_merge  = d_opc == 8'h70;
wire op_and    = (d_opc[7:4] == 4'h7) & |d_opc[3:0];
wire op_mult   = d_opc[7:4] == 4'h8;
wire op_sbk    = d_opc == 8'h90;
wire op_link   = (d_opc >= 8'h91) & (d_opc <= 8'h94);
wire op_sex    = d_opc == 8'h95;
wire op_asr    = d_opc == 8'h96;
wire op_ror    = d_opc == 8'h97;
wire op_jmp    = (d_opc >= 8'h98) & (d_opc <= 8'h9D);
wire op_lob    = d_opc == 8'h9E;
wire op_fmult  = d_opc == 8'h9F;
wire op_ibt    = d_opc[7:4] == 4'hA;
wire op_from   = d_opc[7:4] == 4'hB;
wire op_hib    = d_opc == 8'hC0;
wire op_or     = (d_opc[7:4] == 4'hC) & |d_opc[3:0];
wire op_inc    = (d_opc[7:4] == 4'hD) & ~&d_opc[3:0];
wire op_df     = d_opc == 8'hDF;
wire op_dec    = (d_opc[7:4] == 4'hE) & ~&d_opc[3:0];
wire op_getb   = d_opc == 8'hEF;
wire op_iwt    = d_opc[7:4] == 4'hF;

wire d_prefix  = op_alt1 | op_alt2 | op_alt3 | op_with | (op_to & ~pf_b) | (op_from & ~pf_b);
// the instruction ends with this byte
wire d_last    = d_first ? ~(op_br | op_ibt | op_iwt) : (d_cnt_r == 2'd1);

// branch condition (flags are final: the producer is >= 2 bytes ahead)
reg d_cond;
always @(*) begin
  case (d_opc[3:0])
    4'h5: d_cond = 1'b1;
    4'h6: d_cond = (f_s_w == f_ov);
    4'h7: d_cond = (f_s_w != f_ov);
    4'h8: d_cond = ~f_z_w;
    4'h9: d_cond =  f_z_w;
    4'hA: d_cond = ~f_s_w;
    4'hB: d_cond =  f_s_w;
    4'hC: d_cond = ~f_cy;
    4'hD: d_cond =  f_cy;
    4'hE: d_cond = ~f_ov;
    default: d_cond = f_ov;
  endcase
end

// imm of the completed IBT/IWT (only meaningful when d_last)
wire [15:0] d_imm = op_iwt ? {d_byte, d_lo_r} : {{8{d_byte[7]}}, d_byte};

// X-stage write target (the instruction completing in X this cycle)
wire xa_now = x_done & x_wa & ~x_r15w;
wire xb_now = x_done & (x_cls == XC_FMULT) & x_alt1;
function [1:0] fsel;
  input [3:0] n;
  input       xa_now;
  input [3:0] xw_an;
  input       xb_now;
  begin
    if (xa_now & (xw_an == n)) fsel = 2'd1;      // the destination write wins over LMULT's R4
    else if (xb_now & (n == 4'd4)) fsel = 2'd2;
    else fsel = 2'd0;
  end
endfunction

// hazards for values consumed by D itself (LOOP R12/R13, JMP Rn)
wire x_wr12 = x_v & x_wa & (x_dest == 4'd12);
wire x_wr13 = x_v & x_wa & (x_dest == 4'd13);
wire x_wrn  = x_v & ((x_wa & (x_dest == d_n)) | ((x_cls == XC_FMULT) & x_alt1 & (d_n == 4'd4)));
wire d_loop_hz = op_loop & d_first & (x_wr12 | x_wr13);
wire d_jmp_hz  = op_jmp & d_first & ~pf_alt1 & x_wrn;

wire x_free  = ~x_v | x_done;
// X holds the byte behind it: R15 written from X, STOP
wire x_holds = x_v & (x_r15w | (x_cls == XC_STOP));

assign d_adv = d_bv & x_free & ~x_holds & ~d_loop_hz & ~d_jmp_hz & ~pause;

// D-redirect of R15 (value takes effect at the next edge instead of R15+1)
always @(*) begin
  d_redir  = 0;
  d_target = r15_r + {{8{d_byte[7]}}, d_byte};
  if (~d_first & op_br) begin
    d_redir = d_cond;
  end
  else if (d_first & op_loop) begin
    d_redir  = (r12_w != 16'd1);
    d_target = r13_w;
  end
  else if (d_first & op_jmp & ~pf_alt1) begin
    d_redir  = 1;
    d_target = `RF(d_n);
  end
  else if (d_last & ~d_first & (op_ibt | op_iwt) & ~pf_alt1 & ~pf_alt2 & (d_n == 4'd15)) begin
    d_redir  = 1;
    d_target = d_imm;
  end
end

// instruction decode for X (only used when d_adv & d_last & it needs X).
// Besides the class, D decides where X's operands come from and what X does with
// them, so the X stage itself is a fixed datapath with a 4-way result select.
reg        dx_v;
reg [3:0]  dx_cls;
reg [3:0]  dx_dest;
reg        dx_wa;
reg        dx_ldmode;
reg [15:0] dx_imm;
reg [2:0]  dx_as;      // A operand
reg [1:0]  dx_bs;      // B operand
reg [1:0]  dx_unit;
reg [2:0]  dx_lop;
reg [3:0]  dx_sop;
reg        dx_sub, dx_cin1, dx_cinf;
reg        dx_fzs, dx_s7;
reg [2:0]  dx_fcy;
reg [1:0]  dx_fov;
reg        dx_one, dx_wstb, dx_wrr, dx_wdbv, dx_wstop, dx_wpq, dx_wpc;
always @(*) begin
  dx_v = 0; dx_cls = XC_NONE; dx_dest = pf_dreg; dx_wa = 0; dx_ldmode = 0; dx_imm = d_imm;
  dx_as = AS_RS; dx_bs = BS_RN; dx_unit = XU_ADD; dx_lop = LO_PASS; dx_sop = SO_LSR;
  dx_sub = 0; dx_cin1 = 0; dx_cinf = 0; dx_fzs = 0; dx_s7 = 0; dx_fcy = FC_NONE; dx_fov = 2'd0;
  dx_one = 0; dx_wstb = 0; dx_wrr = 0; dx_wdbv = 0; dx_wstop = 0; dx_wpq = 0; dx_wpc = 0;
  if (d_last) begin
    if (op_stop)       begin dx_v = 1; dx_cls = XC_STOP;  dx_wstop = 1; end
    else if (op_cache) begin dx_v = 1; dx_cls = XC_CACHE; dx_wdbv = 1; end
    else if (op_lsr | op_rol | op_asr | op_ror | op_swap | op_sex | op_lob | op_hib) begin
      dx_v = 1; dx_cls = XC_ALU; dx_wa = 1; dx_unit = XU_SHB; dx_fzs = 1; dx_one = 1;
      if (op_lsr)  begin dx_sop = SO_LSR;  dx_fcy = FC_A0; end
      if (op_rol)  begin dx_sop = SO_ROL;  dx_fcy = FC_A15; end
      if (op_asr)  begin dx_sop = pf_alt1 ? SO_DIV2 : SO_ASR; dx_fcy = FC_A0; end
      if (op_ror)  begin dx_sop = SO_ROR;  dx_fcy = FC_A0; end
      if (op_swap) dx_sop = SO_SWAP;
      if (op_sex)  dx_sop = SO_SEX;
      if (op_lob)  begin dx_sop = SO_LOB; dx_s7 = 1; end
      if (op_hib)  begin dx_sop = SO_HIB; dx_s7 = 1; end
    end
    else if (op_to & pf_b)   begin dx_v = 1; dx_cls = XC_ALU; dx_wa = 1; dx_dest = d_n; dx_unit = XU_LOG; dx_one = 1; end            // MOVE
    else if (op_from & pf_b) begin dx_v = 1; dx_cls = XC_ALU; dx_wa = 1; dx_as = AS_RN; dx_unit = XU_LOG; dx_fzs = 1; dx_fov = 2'd2; dx_one = 1; end // MOVES
    else if (op_st | op_sbk) begin dx_v = 1; dx_cls = XC_ST; dx_wstb = 1; end
    else if (op_loop)  begin dx_v = 1; dx_cls = XC_ALU; dx_wa = 1; dx_dest = 4'd12; dx_as = AS_R12; dx_bs = BS_ONE; dx_sub = 1; dx_cin1 = 1; dx_fzs = 1; dx_one = 1; end
    else if (op_ld)    begin dx_v = 1; dx_cls = XC_LD; dx_wa = 1; dx_unit = XU_MRES; end
    else if (op_plot & pf_alt1) begin dx_v = 1; dx_cls = XC_RPIX; dx_wa = 1; dx_as = AS_R1; dx_unit = XU_MRES; dx_fzs = 1; end
    else if (op_plot)  begin dx_v = 1; dx_cls = XC_PLOT; dx_wa = 1; dx_dest = 4'd1; dx_as = AS_R1; dx_bs = BS_ONE; dx_wpq = 1; end
    else if (op_color) begin dx_v = 1; dx_cls = XC_COLOR; dx_one = ~pf_alt1; dx_wpc = pf_alt1; end   // CMODE flushes the plot cache
    else if (op_not)   begin dx_v = 1; dx_cls = XC_ALU; dx_wa = 1; dx_unit = XU_LOG; dx_lop = LO_NOT; dx_fzs = 1; dx_one = 1; end
    else if (op_add)   begin dx_v = 1; dx_cls = XC_ALU; dx_wa = 1; dx_bs = pf_alt2 ? BS_N : BS_RN; dx_cinf = pf_alt1; dx_fzs = 1; dx_fcy = FC_ADD; dx_fov = 2'd1; dx_one = 1; end
    else if (op_sub)   begin dx_v = 1; dx_cls = XC_ALU; dx_wa = ~(pf_alt1 & pf_alt2); dx_bs = (~pf_alt1 & pf_alt2) ? BS_N : BS_RN;
                             dx_sub = 1; dx_cinf = pf_alt1 & ~pf_alt2; dx_cin1 = ~(pf_alt1 & ~pf_alt2); dx_fzs = 1; dx_fcy = FC_ADD; dx_fov = 2'd1; dx_one = 1; end
    else if (op_merge) begin dx_v = 1; dx_cls = XC_MERGE; end
    else if (op_and)   begin dx_v = 1; dx_cls = XC_ALU; dx_wa = 1; dx_bs = pf_alt2 ? BS_N : BS_RN; dx_unit = XU_LOG; dx_lop = pf_alt1 ? LO_BIC : LO_AND; dx_fzs = 1; dx_one = 1; end
    else if (op_or)    begin dx_v = 1; dx_cls = XC_ALU; dx_wa = 1; dx_bs = pf_alt2 ? BS_N : BS_RN; dx_unit = XU_LOG; dx_lop = pf_alt1 ? LO_XOR : LO_OR;  dx_fzs = 1; dx_one = 1; end
    else if (op_mult)  begin dx_v = 1; dx_cls = XC_MULT; dx_wa = 1; dx_bs = pf_alt2 ? BS_N : BS_RN; dx_unit = XU_MRES; dx_fzs = 1; end
    else if (op_link)  begin dx_v = 1; dx_cls = XC_ALU; dx_wa = 1; dx_dest = 4'd11; dx_as = AS_R15; dx_bs = BS_N; dx_one = 1; end
    else if (op_jmp & pf_alt1) begin dx_v = 1; dx_cls = XC_LJMP; dx_wa = 1; dx_dest = 4'd15; dx_wdbv = 1; end
    else if (op_fmult) begin dx_v = 1; dx_cls = XC_FMULT; dx_wa = 1; dx_unit = XU_MRES; dx_fzs = 1; dx_fcy = FC_MUL; end
    else if (op_ibt | op_iwt) begin
      dx_dest = d_n;
      if (pf_alt1)      begin dx_v = 1; dx_cls = XC_LD; dx_wa = 1; dx_ldmode = 1; dx_unit = XU_MRES; end // LMS / LM
      else if (pf_alt2) begin dx_v = 1; dx_cls = XC_ST; dx_ldmode = 1; dx_as = AS_RN; dx_wstb = 1; end // SMS / SM
      else if (d_n != 4'd15) begin dx_v = 1; dx_cls = XC_ALU; dx_wa = 1; dx_as = AS_IMM; dx_unit = XU_LOG; dx_one = 1; end
      if (op_ibt & (pf_alt1 | pf_alt2)) dx_imm = {7'h00, d_byte, 1'b0};
    end
    else if (op_inc)   begin dx_v = 1; dx_cls = XC_ALU; dx_wa = 1; dx_dest = d_n; dx_as = AS_RN; dx_bs = BS_ONE; dx_fzs = 1; dx_one = 1; end
    else if (op_dec)   begin dx_v = 1; dx_cls = XC_ALU; dx_wa = 1; dx_dest = d_n; dx_as = AS_RN; dx_bs = BS_ONE; dx_sub = 1; dx_cin1 = 1; dx_fzs = 1; dx_one = 1; end
    else if (op_df)    begin dx_v = 1; dx_cls = XC_DF; if (~pf_alt1 & pf_alt2) dx_wstb = 1; else dx_wrr = 1; end
    else if (op_getb)  begin dx_v = 1; dx_cls = XC_GETB; dx_wa = 1; dx_unit = XU_SHB; dx_wrr = 1;
                             dx_sop = (pf_alt1 & pf_alt2) ? SO_GETBS : pf_alt2 ? SO_GETBL : pf_alt1 ? SO_GETBH : SO_GETB; end
  end
end

// operand values for the issue
reg [15:0] d_aval; reg [3:0] d_anum; reg d_areg;
always @(*) begin
  d_areg = 1; d_anum = pf_sreg; d_aval = `RF(pf_sreg);
  case (dx_as)
    AS_RN:  begin d_anum = d_n;   d_aval = `RF(d_n); end
    AS_R12: begin d_anum = 4'd12; d_aval = r12_w; end
    AS_R1:  begin d_anum = 4'd1;  d_aval = r1_w; end
    AS_R15: begin d_areg = 0;     d_aval = r15_r; end
    AS_IMM: begin d_areg = 0;     d_aval = dx_imm; end
    default: ;
  endcase
end
wire [15:0] d_bval = (dx_bs == BS_N) ? {12'h000, d_n} : (dx_bs == BS_ONE) ? 16'h0001 : `RF(d_n);
wire        d_breg = (dx_bs == BS_RN);

// D-stage sequential: prefix state, multi-byte collection, X issue
always @(posedge CLK) if (CE) begin
  if (RST) begin
    d_cnt_r <= 0;
    pf_alt1 <= 0; pf_alt2 <= 0; pf_b <= 0; pf_sreg <= 0; pf_dreg <= 0;
    x_v <= 0;
    x_cls <= XC_NONE;
  end
  else begin
    if (x_free & ~d_adv) x_v <= 0;

    if (d_adv) begin
      // ---- multi-byte bookkeeping
      if (d_first) begin
        if (op_br | op_ibt)   begin d_cnt_r <= 1; d_op_r <= d_byte; end
        else if (op_iwt)      begin d_cnt_r <= 2; d_op_r <= d_byte; end
      end
      else begin
        d_cnt_r <= d_cnt_r - 1;
        d_lo_r  <= d_byte;
      end

      // ---- prefix state.  gsu.v computes the next prefix state at the opcode
      // byte and commits it when the instruction completes; between instructions
      // that equals the architectural copy, so this is the same thing.
      if (d_last) begin
        if (d_first & op_alt1)       begin pf_alt1 <= 1; pf_alt2 <= 0; pf_b <= 0; end
        else if (d_first & op_alt2)  begin pf_alt1 <= 0; pf_alt2 <= 1; pf_b <= 0; end
        else if (d_first & op_alt3)  begin pf_alt1 <= 1; pf_alt2 <= 1; pf_b <= 0; end
        else if (d_first & op_to & ~pf_b)   pf_dreg <= d_n;
        else if (d_first & op_with)  begin pf_sreg <= d_n; pf_dreg <= d_n; pf_b <= 1; end
        else if (d_first & op_from & ~pf_b) pf_sreg <= d_n;
        else if (op_br) begin
          // branches leave ALT1/ALT2/B/FROM/TO untouched: a prefix in front of a
          // branch applies to the delay-slot instruction (DOOM relies on this,
          // e.g. FROM R9 ; BRA x ; SUB R5).  gsu.v and MesenCE do the same.
        end
        else begin
          // every other instruction (MOVE/MOVES included) consumes the prefixes
          pf_alt1 <= 0; pf_alt2 <= 0; pf_b <= 0; pf_sreg <= 0; pf_dreg <= 0;
        end
      end

      // ---- issue to X
      x_v <= dx_v;
      if (dx_v) begin
        x_cls    <= dx_cls;
        x_alt1   <= pf_alt1;
        x_alt2   <= pf_alt2;
        x_dest   <= dx_dest;
        x_wa     <= dx_wa;
        x_r15w   <= dx_wa & (dx_dest == 4'd15);
        x_ldmode <= dx_ldmode;
        x_imm    <= dx_imm;
        x_r15    <= r15_r;
        x_opc    <= d_opc;
        x_unit   <= dx_unit; x_lop <= dx_lop; x_sop <= dx_sop;
        x_sub    <= dx_sub;  x_cin1 <= dx_cin1; x_cinf <= dx_cinf;
        x_fzs    <= dx_fzs;  x_s7 <= dx_s7; x_fcy <= dx_fcy; x_fov <= dx_fov;
        x_one    <= dx_one;  x_wstb <= dx_wstb; x_wrr <= dx_wrr; x_wdbv <= dx_wdbv; x_wstop <= dx_wstop; x_wpq <= dx_wpq; x_wpc <= dx_wpc;
        x_a      <= d_aval;  x_fa <= (d_areg & (d_anum != 4'd15)) ? fsel(d_anum, xa_now, x_dest, xb_now) : 2'd0;
        x_b      <= d_bval;  x_fb <= (d_breg & (d_n != 4'd15)) ? fsel(d_n, xa_now, x_dest, xb_now) : 2'd0;
        x_r2     <= REG_r[2]; x_f2 <= fsel(4'd2, xa_now, x_dest, xb_now);
        x_r6     <= REG_r[6]; x_f6 <= fsel(4'd6, xa_now, x_dest, xb_now);
      end
    end
    // SNES write of the SFR high byte (ALT1/ALT2/B) -- only done while stopped
    if (wb_sfr_h) begin
      pf_alt1 <= wb_data_r[0]; pf_alt2 <= wb_data_r[1]; pf_b <= wb_data_r[4];
    end
  end
end

//-------------------------------------------------------------------
// X STAGE
//-------------------------------------------------------------------
// Operands: D places the right values in x_a / x_b (and R2/R6 for PLOT/FMULT);
// a result produced by the instruction retiring in the same cycle D issued is
// taken from its registered copy here (late forwarding, one 2:1 mux level).
wire [15:0] xa  = x_fa[1] ? wb_q : x_fa[0] ? wa_q : x_a;
wire [15:0] xb  = x_fb[1] ? wb_q : x_fb[0] ? wa_q : x_b;
wire [15:0] xr2 = x_f2[1] ? wb_q : x_f2[0] ? wa_q : x_r2;
wire [15:0] xr6 = x_f6[1] ? wb_q : x_f6[0] ? wa_q : x_r6;

// --- adder (ADD/ADC/SUB/SBC/CMP/INC/DEC/LOOP/LINK, PLOT R1+1)
wire [15:0] x_bb  = x_sub ? ~xb : xb;
wire        x_cin = x_cinf ? f_cy : x_cin1;
wire [16:0] x_sum = {1'b0, xa} + {1'b0, x_bb} + {16'h0000, x_cin};
wire        x_sum_ov = (xa[15] ^ x_sum[15]) & (x_bb[15] ^ x_sum[15]);

// --- logic (AND/BIC/OR/XOR/NOT/PASS)
reg [15:0] x_log;
always @(*) begin
  case (x_lop)
    LO_AND:  x_log = xa & xb;
    LO_BIC:  x_log = xa & ~xb;
    LO_OR:   x_log = xa | xb;
    LO_XOR:  x_log = xa ^ xb;
    LO_NOT:  x_log = ~xa;
    default: x_log = xa;
  endcase
end

// --- shift / byte / GETB
reg [15:0] x_shb;
always @(*) begin
  case (x_sop)
    SO_LSR:   x_shb = {1'b0, xa[15:1]};
    SO_ASR:   x_shb = {xa[15], xa[15:1]};
    SO_DIV2:  x_shb = (&xa) ? 16'h0000 : {xa[15], xa[15:1]};
    SO_ROL:   x_shb = {xa[14:0], f_cy};
    SO_ROR:   x_shb = {f_cy, xa[15:1]};
    SO_SWAP:  x_shb = {xa[7:0], xa[15:8]};
    SO_SEX:   x_shb = {{8{xa[7]}}, xa[7:0]};
    SO_LOB:   x_shb = {8'h00, xa[7:0]};
    SO_HIB:   x_shb = {8'h00, xa[15:8]};
    SO_GETB:  x_shb = {8'h00, ROMRDBUF_r};
    SO_GETBH: x_shb = {ROMRDBUF_r, xa[7:0]};
    SO_GETBL: x_shb = {xa[15:8], ROMRDBUF_r};
    default:  x_shb = {{8{ROMRDBUF_r[7]}}, ROMRDBUF_r};   // GETBS
  endcase
end

// --- result
reg  [15:0] x_mres;  initial x_mres  = 0;   // registered result of multi-cycle ops
reg  [15:0] x_mres2; initial x_mres2 = 0;   // LMULT low word
reg         x_mcy;   initial x_mcy   = 0;   // FMULT carry
wire [15:0] x_res = (x_unit == XU_ADD) ? x_sum[15:0] :
                    (x_unit == XU_LOG) ? x_log :
                    (x_unit == XU_SHB) ? x_shb : x_mres;

// --- multipliers
wire [15:0] x_mult_out;
wire [15:0] x_umult_out;
wire [31:0] x_fmult_out;
reg  [15:0] fm_a_r; initial fm_a_r = 0;
reg  [15:0] fm_b_r; initial fm_b_r = 0;
`ifdef MK2
gsu_mult  x_mult (.a(xa[7:0]), .b(xb[7:0]), .p(x_mult_out));
gsu_umult x_umult(.a(xa[7:0]), .b(xb[7:0]), .p(x_umult_out));
`endif
`ifdef MK3
gsu_mult  x_mult (.dataa(xa[7:0]), .datab(xb[7:0]), .result(x_mult_out));
gsu_umult x_umult(.dataa(xa[7:0]), .datab(xb[7:0]), .result(x_umult_out));
`endif
`ifdef FX3_CE_RAMS
// 2-stage signed 16x16 multiplier, inferred (MULT18X18 + regs) so it follows CE
reg signed [31:0] fm_p1, fm_p2;
always @(posedge CLK) if (CE) begin
  fm_p1 <= $signed(fm_a_r) * $signed(fm_b_r);
  fm_p2 <= fm_p1;
end
assign x_fmult_out = fm_p2;
`else
gsu_fmult x_fmult(.clock(CLK), .dataa(fm_a_r), .datab(fm_b_r), .result(x_fmult_out));
`endif

// --- plot geometry (A = R1)
wire [15:0] x_plot_off = {xr2[7:0],5'h00} + xa[7:3];
wire [2:0]  x_plot_idx = ~xa[2:0];
wire [7:0]  x_plot_col = (~x_alt1 & POR_DTH & ~&SCMR_MD) ? {4'h0, ((xa[0] ^ xr2[0]) ? COLR_r[7:4] : COLR_r[3:0])} : COLR_r;
wire        x_plot_vis = POR_TRS | ((SCMR_MD != 2'd3 | POR_FHN) ? (COLR_r[3:0] != 0) : (COLR_r != 0));

// --- plot queue / plot cache status (see BITMAP)
reg  [1:0]  pq_cnt_r;   initial pq_cnt_r = 0;    // plot queue occupancy (0..2)
reg  [7:0]  rpix_col_r; initial rpix_col_r = 0;
reg         rpix_go_r;  initial rpix_go_r = 0;
reg         rpix_done_r; initial rpix_done_r = 0;
reg  [15:0] rpix_off_r;  initial rpix_off_r = 0;
reg  [2:0]  rpix_idx_r;  initial rpix_idx_r = 0;

wire ram_data_end;
wire clr_idle;
wire pc_idle;
wire pc_flush_req;

// --- completion.  Everything here is a flop or a flop-level wait condition, so
// D's advance does not depend on the X datapath.
assign x_done = x_v & ( x_one
                      | x_fin | x_finh
                      | (x_wstb  & ~stb_busy_r)
                      | (x_wrr   & ~rr_r)
                      | (x_wdbv  & d_bv & f_quiet)
                      | (x_wstop & d_bv & ~stb_busy_r & ~rr_r & pc_idle)
                      | (x_wpc   & pc_idle)
                      | (x_wpq   & ~pq_cnt_r[1]) )
                    & (~x_r15w | (d_bv & f_quiet));   // R15 written from X: the delay slot is in D

// --- side effects (all qualified with x_done where they commit)
wire xw_a   = x_wa & ~x_r15w;
wire xw_15  = x_r15w | (x_cls == XC_STOP);
wire [15:0] xw_15d = (x_cls == XC_STOP) ? 16'h0000 : (x_cls == XC_LJMP) ? xa : x_res;
wire x2p_req = x_wa & (x_dest == 4'd14);
assign x2c_flush_w = x_v & x_done & ((x_cls == XC_LJMP) | ((x_cls == XC_CACHE) & (x_r15[15:4] != CBR_r[15:4])));
assign x2c_cbr_w   = (x_cls == XC_LJMP) ? xa : x_r15;

assign x_stop_done = x_done & (x_cls == XC_STOP);
assign x_squash   = x_done & (xw_15 & (x_cls != XC_STOP) | x2c_flush_w);
assign x_squash_t = (x_cls == XC_CACHE) ? d_addr_r + 16'd1 : xw_15d;
assign pc_flush_req = x_v & ((x_cls == XC_RPIX) | (x_cls == XC_STOP) | (x_cls == XC_MERGE) | ((x_cls == XC_COLOR) & x_alt1));

// X multi-cycle sub-state
always @(posedge CLK) if (CE) begin
  if (RST) begin
    x_st <= 0; x_fin <= 0; x_finh <= 0;
    exe_ram_rd_r <= 0;
    rpix_go_r <= 0;
  end
  else begin
    x_fin <= 0;
    // A finished multi-cycle op that writes R15 retires only once its delay slot
    // is in D (see x_done).  x_fin is a one-cycle pulse, so keep it until then --
    // otherwise a delay-slot cache miss that outlasts the op (slow ROM fetch)
    // loses the pulse and X waits forever.
    x_finh <= x_v & ~x_done & (x_fin | x_finh);
    if (~x_v | x_done) begin
      x_st <= 0;
    end
    else begin
      case (x_cls)
        XC_MULT: begin
          x_mres <= x_alt1 ? x_umult_out : x_mult_out;
          x_fin  <= 1;
        end
        XC_FMULT: begin
          case (x_st)
            4'd0: begin fm_a_r <= xa; fm_b_r <= xr6; x_st <= 1; end
            4'd1: x_st <= 2;
            4'd2: x_st <= 3;
            4'd3: begin
              x_mres <= x_fmult_out[31:16]; x_mres2 <= x_fmult_out[15:0]; x_mcy <= x_fmult_out[15];
              x_st <= 4; x_fin <= 1;
            end
            default: ;
          endcase
        end
        XC_LD: begin
          case (x_st)
            4'd0: if (~stb_busy_r) begin
              exe_ram_rd_r <= 1;
              exe_word_r   <= x_ldmode | ~x_alt1;
              exe_addr_r   <= {4'hE, 3'h0, RAMBR_r, x_ldmode ? x_imm : xb};
              x_st <= 1;
            end
            4'd1: if (ram_data_end) begin
              exe_ram_rd_r <= 0;
              x_mres <= (x_ldmode | ~x_alt1) ? ram_bus_data_r : {8'h00, ram_bus_data_r[7:0]};
              x_st <= 2; x_fin <= 1;
            end
            default: ;
          endcase
        end
        XC_RPIX: begin
          case (x_st)
            4'd0: begin rpix_off_r <= x_plot_off; rpix_idx_r <= x_plot_idx; x_st <= 1; end
            4'd1: if (pc_idle) begin
              rpix_go_r <= 1; x_st <= 2;
            end
            4'd2: begin
              rpix_go_r <= 0;
              if (rpix_done_r) begin x_mres <= {8'h00, rpix_col_r}; x_st <= 3; x_fin <= 1; end
            end
            default: ;
          endcase
        end
        XC_MERGE: begin
          case (x_st)
            4'd0: if (pc_idle) begin x_st <= 1; if (~x2m_go) x_fin <= 1; end
            4'd1: x_st <= 2;
            4'd2: if (clr_idle) begin x_st <= 3; x_fin <= 1; end
            default: ;
          endcase
        end
        default: ;
      endcase
    end
  end
end

// MERGE dispatcher: R0 is read straight from the register file -- every older
// instruction has retired by the time MERGE is in X
wire [7:0] x_r0lo = REG_r[0][7:0];
always @(*) begin
  x2m_go    = x_v & (x_cls == XC_MERGE) & (x_st == 4'd0) & pc_idle & ((x_r0lo == 8'd3) | (x_r0lo == 8'd4) | (x_r0lo == 8'd5));
  x2m_third = x_r0lo[1:0] + 2'd1;    // 3->0, 4->1, 5->2
end

//-------------------------------------------------------------------
// ARCHITECTURAL COMMIT (register file, SFR, special registers)
//-------------------------------------------------------------------
reg  prf_pend_r; initial prf_pend_r = 0;
always @(posedge CLK) if (CE) begin
  if (RST) begin
    REG_r[0] <= 0; REG_r[1] <= 0; REG_r[2] <= 0; REG_r[3] <= 0; REG_r[4] <= 0;
    REG_r[5] <= 0; REG_r[6] <= 0; REG_r[7] <= 0; REG_r[8] <= 0; REG_r[9] <= 0;
    REG_r[10] <= 0; REG_r[11] <= 0; REG_r[12] <= 0; REG_r[13] <= 0; REG_r[14] <= 0;
    r15_r <= 0;
    f_z <= 0; f_cy <= 0; f_s <= 0; f_ov <= 0; zs_lazy <= 0;
    go_r <= 0; irq_r <= 0; sfr_il_r <= 0;
    BRAMR_r <= 0; PBR_r <= 0; CFGR_r <= 0; SCBR_r <= 0; CLSR_r <= 0; SCMR_r <= 0;
    COLR_r <= 0; POR_r <= 0;
    ROMBR_r <= 0; RAMBR_r <= 0;
    RAMADDR_r <= 0;
    wa_q <= 0; wb_q <= 0;
  end
  else begin
    if (x_done) begin
      // LMULT: R4 = low word first, then the destination (which wins if it is R4,
      // like gsu.v and MesenCE)
      if ((x_cls == XC_FMULT) & x_alt1) begin
        REG_r[4] <= x_mres2;
        wb_q <= x_mres2;
      end
      if (xw_a) begin
        case (x_dest)
          4'd0 : REG_r[0]  <= x_res;  4'd1 : REG_r[1]  <= x_res;  4'd2 : REG_r[2]  <= x_res;
          4'd3 : REG_r[3]  <= x_res;  4'd4 : REG_r[4]  <= x_res;  4'd5 : REG_r[5]  <= x_res;
          4'd6 : REG_r[6]  <= x_res;  4'd7 : REG_r[7]  <= x_res;  4'd8 : REG_r[8]  <= x_res;
          4'd9 : REG_r[9]  <= x_res;  4'd10: REG_r[10] <= x_res;  4'd11: REG_r[11] <= x_res;
          4'd12: REG_r[12] <= x_res;  4'd13: REG_r[13] <= x_res;  4'd14: REG_r[14] <= x_res;
          default: ;
        endcase
        wa_q <= x_res;
      end
      // flags
      if (x_fzs) begin zres_q <= x_res; s7_q <= x_s7; zs_lazy <= 1; end
      case (x_fcy)
        FC_ADD: f_cy <= x_sum[16];
        FC_A0:  f_cy <= xa[0];
        FC_A15: f_cy <= xa[15];
        FC_MUL: f_cy <= x_mcy;
        default: ;
      endcase
      if (x_fov == 2'd1) f_ov <= x_sum_ov;
      if (x_fov == 2'd2) f_ov <= xa[7];
      // special registers
      case (x_cls)
        XC_COLOR: if (x_alt1) POR_r <= {3'h0, xa[4:0]};
                  else COLR_r <= {POR_FHN ? COLR_r[7:4] : xa[7:4], POR_HN ? xa[7:4] : xa[3:0]};
        XC_DF:    if (~x_alt1 & ~x_alt2) COLR_r <= {POR_FHN ? COLR_r[7:4] : ROMRDBUF_r[7:4], POR_HN ? ROMRDBUF_r[7:4] : ROMRDBUF_r[3:0]};
                  else if (x_alt1 & x_alt2) ROMBR_r <= {1'b0, xa[6:0]};
                  else if (x_alt2) RAMBR_r <= xa[0];
        XC_LJMP:  PBR_r <= {1'b0, xb[6:0]};
        XC_STOP:  go_r <= 0;
        XC_LD:    RAMADDR_r <= x_ldmode ? x_imm : xb;
        XC_ST:    if (x_opc != 8'h90) RAMADDR_r <= x_ldmode ? x_imm : xb;
        default: ;
      endcase
    end

    // R15
    if (x_done & xw_15) r15_r <= xw_15d;
    else if (d_adv) r15_r <= d_redir ? d_target : r15_r + 16'd1;

    // SNES writes (the 65816 only writes registers while the FX is stopped;
    // R15/SFR are how it starts and stops it)
    if (wb_gpr_w) begin
      case (wb_gpr_n)
        4'd0 : REG_r[0]  <= {wb_data_r,data_flop_r};  4'd1 : REG_r[1]  <= {wb_data_r,data_flop_r};
        4'd2 : REG_r[2]  <= {wb_data_r,data_flop_r};  4'd3 : REG_r[3]  <= {wb_data_r,data_flop_r};
        4'd4 : REG_r[4]  <= {wb_data_r,data_flop_r};  4'd5 : REG_r[5]  <= {wb_data_r,data_flop_r};
        4'd6 : REG_r[6]  <= {wb_data_r,data_flop_r};  4'd7 : REG_r[7]  <= {wb_data_r,data_flop_r};
        4'd8 : REG_r[8]  <= {wb_data_r,data_flop_r};  4'd9 : REG_r[9]  <= {wb_data_r,data_flop_r};
        4'd10: REG_r[10] <= {wb_data_r,data_flop_r};  4'd11: REG_r[11] <= {wb_data_r,data_flop_r};
        4'd12: REG_r[12] <= {wb_data_r,data_flop_r};  4'd13: REG_r[13] <= {wb_data_r,data_flop_r};
        4'd14: REG_r[14] <= {wb_data_r,data_flop_r};
        default: begin r15_r <= {wb_data_r,data_flop_r}; go_r <= 1; end
      endcase
    end
    if (wb_go & wb_reg_r) begin
      case (wb_addr_r[7:0])
        8'h30: begin
          f_z <= wb_data_r[1]; f_cy <= wb_data_r[2]; f_s <= wb_data_r[3]; f_ov <= wb_data_r[4];
          zs_lazy <= 0;
          go_r <= wb_data_r[5];
        end
        8'h31: begin irq_r <= wb_data_r[7]; sfr_il_r <= wb_data_r[3:2]; end
        8'h33: BRAMR_r[0] <= wb_data_r[0];
        8'h34: PBR_r <= {1'b0, wb_data_r[6:0]};
        8'h37: {CFGR_r[7],CFGR_r[5]} <= {wb_data_r[7],wb_data_r[5]};
        8'h38: SCBR_r <= wb_data_r;
        8'h39: CLSR_r[0] <= wb_data_r[0];
        8'h3A: SCMR_r[5:0] <= wb_data_r[5:0];
        default: ;
      endcase
    end
    if (rb_sfrh_r) irq_r <= 0;
  end
end

// STORE BUFFER
//-------------------------------------------------------------------
wire        x2s_req  = (x_cls == XC_ST);
wire        x2s_word = x_ldmode | (x_opc == 8'h90) | ~x_alt1;
wire [15:0] x2s_data = xa;
wire [15:0] x2s_addr = x_ldmode ? x_imm : (x_opc == 8'h90) ? RAMADDR_r : xb;
reg [1:0] stb_st_r; initial stb_st_r = 0;   // 0 idle, 1 low byte, 2 high byte
reg [7:0] stb_hi_r; initial stb_hi_r = 0;
reg       stb_word_r; initial stb_word_r = 0;
wire      ram_stb_end;
always @(posedge CLK) if (CE) begin
  if (RST) begin
    stb_st_r <= 0; stb_busy_r <= 0; stb_ram_wr_r <= 0;
  end
  else begin
    case (stb_st_r)
      2'd0: if (x_done & x2s_req) begin
        stb_busy_r   <= 1;
        stb_ram_wr_r <= 1;
        stb_word_r   <= x2s_word;
        stb_addr_r   <= {4'hE, 3'h0, RAMBR_r, x2s_addr};
        stb_data_r   <= x2s_data[7:0];
        stb_hi_r     <= x2s_data[15:8];
        stb_st_r     <= 1;
      end
      2'd1: if (ram_stb_end) begin
        if (stb_word_r) begin
          stb_addr_r[0] <= ~stb_addr_r[0];
          stb_data_r    <= stb_hi_r;
          stb_st_r      <= 2;
        end
        else begin
          stb_ram_wr_r <= 0; stb_busy_r <= 0; stb_st_r <= 0;
        end
      end
      2'd2: if (ram_stb_end) begin
        stb_ram_wr_r <= 0; stb_busy_r <= 0; stb_st_r <= 0;
      end
      default: stb_st_r <= 0;
    endcase
  end
end

//-------------------------------------------------------------------
// ROM PREFETCHER (R14 -> ROMRDBUF, SFR R)
//-------------------------------------------------------------------
reg [1:0] prf_st_r; initial prf_st_r = 0;
always @(posedge CLK) if (CE) begin
  if (RST) begin
    prf_st_r <= 0; rr_r <= 0; prf_pend_r <= 0; prf_rom_rd_r <= 0;
    ROMRDBUF_r <= 0;
  end
  else begin
    // a write to R14 (FX or SNES) (re)starts the buffer fill; RR rises at once
    if ((x_done & x2p_req) | wb_r14) begin
      rr_r <= 1;
      prf_pend_r <= 1;
    end
    case (prf_st_r)
      2'd0: if (prf_pend_r) begin
        prf_pend_r   <= ((x_done & x2p_req) | wb_r14);
        prf_rom_rd_r <= 1;
        prf_addr_r   <= (ROMBR_r[6] ? {ROMBR_r, REG_r[14]} : {1'b0, ROMBR_r, REG_r[14][14:0]}) & ROM_MASK;
        prf_st_r     <= 1;
      end
      2'd1: if (rom_prf_end) begin
        prf_rom_rd_r <= 0;
        ROMRDBUF_r   <= rom_bus_data_r[7:0];
        prf_st_r     <= 0;
        if (~prf_pend_r & ~((x_done & x2p_req) | wb_r14)) rr_r <= 0;
      end
      default: prf_st_r <= 0;
    endcase
  end
end

//-------------------------------------------------------------------
// ROM BUS
//-------------------------------------------------------------------
parameter RS_IDLE = 2'd0, RS_FCH = 2'd1, RS_PRF = 2'd2, RS_END = 2'd3;
reg [1:0]  rom_st_r;  initial rom_st_r = RS_IDLE;
reg        rom_rrq_r; initial rom_rrq_r = 0;
reg [23:0] rom_addr_r; initial rom_addr_r = 0;
reg        rom_word_r; initial rom_word_r = 0;
reg        rom_endfch_r; initial rom_endfch_r = 0;
reg        rom_endprf_r; initial rom_endprf_r = 0;
assign rom_fch_end = rom_endfch_r;
assign rom_prf_end = rom_endprf_r;

always @(posedge CLK) if (CE) begin
  if (RST) begin
    rom_st_r <= RS_IDLE; rom_rrq_r <= 0; rom_endfch_r <= 0; rom_endprf_r <= 0;
  end
  else begin
    rom_endfch_r <= 0; rom_endprf_r <= 0;
    case (rom_st_r)
      RS_IDLE: begin
        if (ROM_BUS_RDY & ~rom_endfch_r & ~rom_endprf_r) begin
          if (fch_rom_rd_r) begin
            rom_rrq_r <= 1; rom_addr_r <= fch_addr_r; rom_word_r <= fch_word_r; rom_st_r <= RS_FCH;
          end
          else if (prf_rom_rd_r) begin
            rom_rrq_r <= 1; rom_addr_r <= prf_addr_r; rom_word_r <= 0; rom_st_r <= RS_PRF;
          end
        end
      end
      RS_FCH, RS_PRF: begin
        rom_rrq_r <= 0;
        if (~rom_rrq_r & ROM_BUS_RDY) begin
          rom_bus_data_r <= ROM_BUS_RDDATA;
          if (rom_st_r == RS_FCH) rom_endfch_r <= 1; else rom_endprf_r <= 1;
          rom_st_r <= RS_IDLE;
        end
      end
      default: rom_st_r <= RS_IDLE;
    endcase
  end
end
assign ROM_BUS_RRQ  = rom_rrq_r;
assign ROM_BUS_WORD = rom_word_r;
assign ROM_BUS_ADDR = rom_addr_r;

//-------------------------------------------------------------------
// RAM BUS
//-------------------------------------------------------------------
parameter MS_IDLE = 1'b0, MS_ACC = 1'b1;
parameter MO_EXE = 3'd0, MO_FCH = 3'd1, MO_STB = 3'd2, MO_BMP = 3'd3, MO_BMFR = 3'd4, MO_BMFW = 3'd5, MO_CLR = 3'd6;
reg        ram_st_r;   initial ram_st_r = MS_IDLE;
reg [2:0]  ram_own_r;  initial ram_own_r = 0;
reg        ram_rrq_r;  initial ram_rrq_r = 0;
reg        ram_wrq_r;  initial ram_wrq_r = 0;
reg [18:0] ram_addr_r; initial ram_addr_r = 0;
reg [7:0]  ram_wdata_r; initial ram_wdata_r = 0;
reg        ram_word_r; initial ram_word_r = 0;   // read: second byte pending
reg        ram_upper_r; initial ram_upper_r = 0;
reg [6:0]  ram_end_r;  initial ram_end_r = 0;    // one-hot end strobe per owner
assign ram_data_end = ram_end_r[MO_EXE];
assign ram_fch_end  = ram_end_r[MO_FCH];
assign ram_stb_end  = ram_end_r[MO_STB];
wire   ram_bmp_end  = ram_end_r[MO_BMP];
wire   ram_bmf_end  = ram_end_r[MO_BMFR] | ram_end_r[MO_BMFW];
wire   ram_clr_end  = ram_end_r[MO_CLR];

always @(posedge CLK) if (CE) begin
  if (RST) begin
    ram_st_r <= MS_IDLE; ram_rrq_r <= 0; ram_wrq_r <= 0; ram_end_r <= 0;
  end
  else begin
    ram_end_r <= 0;
    case (ram_st_r)
      MS_IDLE: begin
        if (RAM_BUS_RDY & ~|ram_end_r) begin
          if (exe_ram_rd_r & ~ram_data_end) begin
            ram_rrq_r <= 1; ram_addr_r <= exe_addr_r[18:0]; ram_word_r <= exe_word_r; ram_upper_r <= 0;
            ram_own_r <= MO_EXE; ram_st_r <= MS_ACC;
          end
          else if (fch_ram_rd_r) begin
            ram_rrq_r <= 1; ram_addr_r <= fch_addr_r[18:0]; ram_word_r <= 0; ram_upper_r <= 0;
            ram_own_r <= MO_FCH; ram_st_r <= MS_ACC;
          end
          else if (stb_ram_wr_r) begin
            ram_wrq_r <= 1; ram_addr_r <= stb_addr_r[18:0]; ram_wdata_r <= stb_data_r; ram_word_r <= 0;
            ram_own_r <= MO_STB; ram_st_r <= MS_ACC;
          end
          else if (bmp_ram_rd_r) begin
            ram_rrq_r <= 1; ram_addr_r <= bmp_addr_r[18:0]; ram_word_r <= 0; ram_upper_r <= 0;
            ram_own_r <= MO_BMP; ram_st_r <= MS_ACC;
          end
          else if (bmf_ram_rd_r) begin
            ram_rrq_r <= 1; ram_addr_r <= bmf_addr_r[18:0]; ram_word_r <= 0; ram_upper_r <= 0;
            ram_own_r <= MO_BMFR; ram_st_r <= MS_ACC;
          end
          else if (bmf_ram_wr_r) begin
            ram_wrq_r <= 1; ram_addr_r <= bmf_addr_r[18:0]; ram_wdata_r <= bmf_data_r; ram_word_r <= 0;
            ram_own_r <= MO_BMFW; ram_st_r <= MS_ACC;
          end
          else if (clr_ram_wr_r) begin
            ram_wrq_r <= 1; ram_addr_r <= clr_addr_w; ram_wdata_r <= clr_byte_w; ram_word_r <= 0;
            ram_own_r <= MO_CLR; ram_st_r <= MS_ACC;
          end
        end
      end
      MS_ACC: begin
        ram_rrq_r <= 0; ram_wrq_r <= 0;
        if (~ram_rrq_r & ~ram_wrq_r & RAM_BUS_RDY) begin
          if (ram_word_r) begin
            // word read: low byte at addr, high byte at addr^1
            ram_word_r <= 0; ram_upper_r <= 1;
            ram_rrq_r <= 1;
            ram_addr_r[0] <= ~ram_addr_r[0];
            ram_bus_data_r[7:0] <= RAM_BUS_RDDATA;
          end
          else begin
            if (ram_upper_r) ram_bus_data_r[15:8] <= RAM_BUS_RDDATA;
            else             ram_bus_data_r[7:0]  <= RAM_BUS_RDDATA;
            ram_upper_r <= 0;
            ram_end_r[ram_own_r] <= 1;
            ram_st_r <= MS_IDLE;
          end
        end
      end
    endcase
  end
end
assign RAM_BUS_RRQ    = ram_rrq_r;
assign RAM_BUS_WRQ    = ram_wrq_r;
assign RAM_BUS_WORD   = 1'b0;
assign RAM_BUS_ADDR   = ram_addr_r;
assign RAM_BUS_WRDATA = ram_wdata_r;

//-------------------------------------------------------------------
// BITMAP: plot cache, write-back, RPIX
//-------------------------------------------------------------------
// gsu.v keeps the GSU's two 8-pixel buffers and flushes one on every change of
// character row -- 2 pixels per flush with an 8-plane read-modify-write when
// pixels are drawn in columns (DOOM's renderer: 10368 PLOTs per third, 5183
// flushes).  Here the pixel buffers are a 256-line direct-mapped write-back cache
// indexed by the screen row (offset[12:5] = Y), each line being one character
// row of 8 pixels at offset {Y, Xchar}.  Lines fill up while the program walks
// down the columns and are written back as whole rows -- no read needed -- when
// another character column claims the row, or at a flush point.
//
// The line data lives in two ways of a pixel BRAM; an evicted line keeps its way
// until the write-back has read it, the new line takes the other way.  The
// write-back computes the planar address with the CURRENT SCMR/SCBR/POR like
// gsu.v.  Flush points (everything written back before the instruction retires):
// RPIX, STOP, MERGE (C2P / Clear) and CMODE.
function [15:0] char_num;
  input [15:0] off;
  input [1:0]  ht;
  begin
    case (ht)
      2'd0: char_num = {off[4:0],4'b0000} + off[12:8];
      2'd1: char_num = {off[4:0],4'b0000} + {off[4:0],2'b00} + off[12:8];
      2'd2: char_num = {off[4:0],4'b0000} + {off[4:0],3'b000} + off[12:8];
      default: char_num = {off[12],off[4],off[11:8],off[3:0]};
    endcase
  end
endfunction
function [15:0] char_shift;
  input [15:0] c;
  input [1:0]  md;
  begin
    case (md)
      2'd0: char_shift = {c,4'h0};
      2'd1: char_shift = {c,5'h00};
      2'd2: char_shift = {c,5'h00};
      default: char_shift = {c,6'h00};
    endcase
  end
endfunction
wire [2:0] bppm1 = {&SCMR_MD, |SCMR_MD, 1'b1};

// ---- plot queue (2 entries) between X and the cache
reg  [15:0] pq_off_r [1:0];
reg  [2:0]  pq_idx_r [1:0];
reg  [7:0]  pq_col_r [1:0];
wire [15:0] pq_off0_w = pq_off_r[0];
initial begin pq_off_r[0] = 0; pq_off_r[1] = 0; pq_idx_r[0] = 0; pq_idx_r[1] = 0; pq_col_r[0] = 0; pq_col_r[1] = 0; end
wire        pq_push = x_done & (x_cls == XC_PLOT) & x_plot_vis;
wire        pq_pop;

always @(posedge CLK) if (CE) begin
  if (RST) pq_cnt_r <= 0;
  else begin
    if (pq_pop) begin
      pq_off_r[0] <= pq_off_r[1]; pq_idx_r[0] <= pq_idx_r[1]; pq_col_r[0] <= pq_col_r[1];
    end
    if (pq_push) begin
      if (pq_cnt_r == 2'd0 | (pq_cnt_r == 2'd1 & pq_pop)) begin
        pq_off_r[0] <= x_plot_off; pq_idx_r[0] <= x_plot_idx; pq_col_r[0] <= x_plot_col;
      end
      else begin
        pq_off_r[1] <= x_plot_off; pq_idx_r[1] <= x_plot_idx; pq_col_r[1] <= x_plot_col;
      end
    end
    pq_cnt_r <= pq_cnt_r + {1'b0, pq_push} - {1'b0, pq_pop};
  end
end

// ---- RAMs (inferred: tag RAM 256 x 14, pixel RAM 2 ways x 256 lines x 8 pixels)
// tag word: {way, xchar[4:0], valid mask[7:0]}
reg  [13:0] pc_tag_mem [0:255];
reg  [7:0]  pc_pix_mem [0:4095];
// simulation-only clear (block RAM powers up as zero on both FPGA families)
// synthesis translate_off
integer ip;
initial begin
  for (ip = 0; ip < 256; ip = ip + 1) pc_tag_mem[ip] = 0;
  for (ip = 0; ip < 4096; ip = ip + 1) pc_pix_mem[ip] = 0;
`ifdef FX3_CE_RAMS
  for (ip = 0; ip < 512; ip = ip + 1) cache_mem[ip] = 0;
  cache_q = 0;
`endif
end
// synthesis translate_on
reg  [7:0]  pc_tag_raddr;
reg  [13:0] pc_tag_q;
reg         pc_tag_we;
reg  [7:0]  pc_tag_waddr;
reg  [13:0] pc_tag_wdata;
reg         pc_pix_we;
reg  [11:0] pc_pix_waddr;
reg  [7:0]  pc_pix_wdata;
reg  [11:0] pc_pix_raddr;
reg  [7:0]  pc_pix_q;
always @(posedge CLK) if (CE) begin
  if (pc_tag_we) pc_tag_mem[pc_tag_waddr] <= pc_tag_wdata;
  pc_tag_q <= pc_tag_mem[pc_tag_raddr];
end
always @(posedge CLK) if (CE) begin
  if (pc_pix_we) pc_pix_mem[pc_pix_waddr] <= pc_pix_wdata;
  pc_pix_q <= pc_pix_mem[pc_pix_raddr];
end

// ---- write-back queue (4 entries): {y, xchar, mask, way}
reg  [7:0]  wq_y_r    [3:0];
reg  [4:0]  wq_x_r    [3:0];
reg  [7:0]  wq_m_r    [3:0];
reg         wq_w_r    [3:0];
reg  [3:0]  wq_v_r;  initial wq_v_r = 0;
reg  [1:0]  wq_rd_r; initial wq_rd_r = 0;    // head
wire        wq_w_rd = wq_w_r[wq_rd_r];
wire [7:0]  wq_y_rd = wq_y_r[wq_rd_r];
reg  [1:0]  wq_wr_r; initial wq_wr_r = 0;    // tail
wire        wq_full = &wq_v_r;
wire        wq_empty = ~|wq_v_r;
wire        wb_pop_w;

// ---- P1 (tag read) / P2 (tag check, pixel write, eviction)
reg         p1_v_r;  initial p1_v_r = 0;      // P2 holds an entry whose tag read is in pc_tag_q
reg  [15:0] p1_off_r; reg [2:0] p1_idx_r; reg [7:0] p1_col_r;
reg         p1_scan_r; initial p1_scan_r = 0; // ... it is a flush-scan entry (no pixel)
reg  [8:0]  pc_lines_r; initial pc_lines_r = 0;   // valid lines
reg         scan_r;  initial scan_r = 0;      // flush scan running
reg  [8:0]  scan_y_r; initial scan_y_r = 0;

wire [7:0]  p2_y   = p1_off_r[12:5];
wire        pc_flush_start = pc_flush_req & ~scan_r & ~|pq_cnt_r & ~p1_v_r & |pc_lines_r;
wire [4:0]  p2_x   = p1_off_r[4:0];
wire [13:0] p2_tag = pc_tag_q;        // P1 never overlaps P2: no read-during-write
wire        p2_way = p2_tag[13];
wire [7:0]  p2_msk = p2_tag[7:0];
wire        p2_hit = ~|p2_msk | (p2_tag[12:8] == p2_x);
wire        wq_has_y = (wq_v_r[0] & (wq_y_r[0] == p2_y)) | (wq_v_r[1] & (wq_y_r[1] == p2_y)) |
                       (wq_v_r[2] & (wq_y_r[2] == p2_y)) | (wq_v_r[3] & (wq_y_r[3] == p2_y));
// P2 needs a write-back slot when it evicts (or flushes) a valid line
wire        p2_evict = p1_v_r & |p2_msk & (p1_scan_r | ~p2_hit);
wire        p2_stall = p2_evict & (wq_full | wq_has_y);
wire        p2_go    = p1_v_r & ~p2_stall;
wire        p1_take  = ~p1_v_r;               // one entry in flight: 2 cycles per pixel
assign      pq_pop   = p1_take & |pq_cnt_r & ~scan_r;

wire        wb_idle_w;
// registered (one cycle late, conservatively cleared by a PLOT entering the
// queue): only X instructions that cannot themselves PLOT wait on it
reg         pc_idle_r; initial pc_idle_r = 1;
always @(posedge CLK) if (CE) pc_idle_r <= ~|pq_cnt_r & ~p1_v_r & ~scan_r & wq_empty & ~|pc_lines_r & wb_idle_w & ~pq_push & ~pc_flush_start;
assign      pc_idle = pc_idle_r;

always @(*) begin
  pc_tag_raddr = p1_v_r ? p2_y : scan_r ? scan_y_r[7:0] : pq_off0_w[12:5];
  pc_tag_we = 0; pc_tag_waddr = p2_y; pc_tag_wdata = 14'h0000;
  pc_pix_we = 0; pc_pix_waddr = {p2_way, p2_y, p1_idx_r}; pc_pix_wdata = p1_col_r;
  if (p2_go) begin
    if (p1_scan_r) begin
      if (|p2_msk) begin pc_tag_we = 1; pc_tag_wdata = {~p2_way, 5'h00, 8'h00}; end
    end
    else if (p2_hit) begin
      pc_tag_we = 1; pc_tag_wdata = {p2_way, p2_x, p2_msk | (8'h01 << p1_idx_r)};
      pc_pix_we = 1; pc_pix_waddr = {p2_way, p2_y, p1_idx_r};
    end
    else begin
      pc_tag_we = 1; pc_tag_wdata = {~p2_way, p2_x, (8'h01 << p1_idx_r)};
      pc_pix_we = 1; pc_pix_waddr = {~p2_way, p2_y, p1_idx_r};
    end
  end
end

always @(posedge CLK) if (CE) begin
  if (RST) begin
    p1_v_r <= 0; pc_lines_r <= 0; scan_r <= 0; wq_v_r <= 0; wq_rd_r <= 0; wq_wr_r <= 0;
  end
  else begin
    // P1 -> P2
    if (p1_take) begin
      if (scan_r) begin
        p1_v_r <= 1; p1_scan_r <= 1; p1_off_r <= {3'b000, scan_y_r[7:0], 5'h00};
        scan_y_r <= scan_y_r + 1;
        if (scan_y_r[7:0] == 8'hFF) scan_r <= 0;
      end
      else if (|pq_cnt_r) begin
        p1_v_r <= 1; p1_scan_r <= 0;
        p1_off_r <= pq_off_r[0]; p1_idx_r <= pq_idx_r[0]; p1_col_r <= pq_col_r[0];
      end
    end
    else if (p2_go) p1_v_r <= 0;
    // start a flush scan once the plot pipe is drained
    if (pc_flush_start) begin
      scan_r <= 1; scan_y_r <= 0;
    end
    // line count
    if (p2_go & ~p1_scan_r & p2_hit & ~|p2_msk) pc_lines_r <= pc_lines_r + 1;
    if (p2_go & p1_scan_r & |p2_msk)            pc_lines_r <= pc_lines_r - 1;
    // write-back queue push / pop
    if (p2_go & p2_evict) begin
      wq_y_r[wq_wr_r] <= p2_y; wq_x_r[wq_wr_r] <= p2_tag[12:8]; wq_m_r[wq_wr_r] <= p2_msk; wq_w_r[wq_wr_r] <= p2_way;
      wq_v_r[wq_wr_r] <= 1; wq_wr_r <= wq_wr_r + 1;
    end
    if (wb_pop_w) begin
      wq_v_r[wq_rd_r] <= 0; wq_rd_r <= wq_rd_r + 1;
    end
  end
end

// ---- write-back engine
// reads the 8 pixels of the head line from the pixel BRAM, then writes (or
// read-modify-writes) the bpp plane bytes
parameter WB_IDLE = 3'd0, WB_PIX = 3'd1, WB_SETUP = 3'd2, WB_RD = 3'd3, WB_RDW = 3'd4, WB_WR = 3'd5, WB_WRW = 3'd6;
reg  [2:0]  wb_st_r;    initial wb_st_r = WB_IDLE;
reg  [3:0]  wb_cnt_r;   initial wb_cnt_r = 0;
reg  [7:0]  wb_pix_r [7:0];
// pixel p plane b at wb_pix_flat[{p,b}]
wire [63:0] wb_pix_flat = {wb_pix_r[7], wb_pix_r[6], wb_pix_r[5], wb_pix_r[4],
                           wb_pix_r[3], wb_pix_r[2], wb_pix_r[1], wb_pix_r[0]};
reg  [2:0]  wb_plane_r; initial wb_plane_r = 0;
reg  [7:0]  wb_old_r;   initial wb_old_r = 0;
reg  [15:0] wb_cshift_r; initial wb_cshift_r = 0;
reg  [7:0]  wb_msk_r;   initial wb_msk_r = 0;
reg  [7:0]  wb_y_r;     initial wb_y_r = 0;
reg         wb_pop_r;   initial wb_pop_r = 0;
assign      wb_pop_w  = wb_pop_r;
assign      wb_idle_w = (wb_st_r == WB_IDLE) & ~wb_pop_r;
wire        wb_full = &wb_msk_r;
wire [23:0] wb_addr = 24'hE00000 + wb_cshift_r + {SCBR_r,10'h000} + {wb_y_r[2:0],1'b0} + {wb_plane_r[2:1], 3'b000, wb_plane_r[0]};
reg  [7:0]  wb_bits;
always @(*) begin
  wb_bits = {wb_pix_flat[56 + wb_plane_r], wb_pix_flat[48 + wb_plane_r], wb_pix_flat[40 + wb_plane_r], wb_pix_flat[32 + wb_plane_r],
             wb_pix_flat[24 + wb_plane_r], wb_pix_flat[16 + wb_plane_r], wb_pix_flat[ 8 + wb_plane_r], wb_pix_flat[ 0 + wb_plane_r]};
  pc_pix_raddr = {wq_w_rd, wq_y_rd, wb_cnt_r[2:0]};
end

always @(posedge CLK) if (CE) begin
  if (RST) begin
    wb_st_r <= WB_IDLE; bmf_ram_rd_r <= 0; bmf_ram_wr_r <= 0; wb_pop_r <= 0;
  end
  else begin
    wb_pop_r <= 0;
    case (wb_st_r)
      WB_IDLE: if (~wq_empty & ~wb_pop_r) begin
        wb_cnt_r <= 0;
        wb_y_r   <= wq_y_r[wq_rd_r];
        wb_msk_r <= wq_m_r[wq_rd_r];
        wb_cshift_r <= char_shift(char_num({3'b000, wq_y_r[wq_rd_r], wq_x_r[wq_rd_r]}, SCMR_HT | {2{POR_OBJ}}), SCMR_MD);
        wb_st_r  <= WB_PIX;
      end
      WB_PIX: begin
        // pixel RAM address = cnt; data for cnt-1 arrives now
        wb_cnt_r <= wb_cnt_r + 1;
        if (wb_cnt_r != 0) wb_pix_r[wb_cnt_r - 1] <= pc_pix_q;
        if (wb_cnt_r == 4'd8) begin
          wb_plane_r <= 0;
          wb_pop_r <= 1;                 // the line's way is free again
          wb_st_r <= wb_full ? WB_WR : WB_RD;
        end
      end
      WB_RD: begin
        bmf_addr_r <= wb_addr; bmf_ram_rd_r <= 1; wb_st_r <= WB_RDW;
      end
      WB_RDW: if (ram_end_r[MO_BMFR]) begin
        bmf_ram_rd_r <= 0; wb_old_r <= ram_bus_data_r[7:0]; wb_st_r <= WB_WR;
      end
      WB_WR: begin
        bmf_addr_r <= wb_addr; bmf_ram_wr_r <= 1;
        bmf_data_r <= (wb_bits & wb_msk_r) | (wb_old_r & ~wb_msk_r);
        wb_st_r <= WB_WRW;
      end
      WB_WRW: if (ram_end_r[MO_BMFW]) begin
        bmf_ram_wr_r <= 0;
        wb_plane_r <= wb_plane_r + 1;
        wb_st_r <= (wb_plane_r == bppm1) ? WB_IDLE : wb_full ? WB_WR : WB_RD;
      end
      default: wb_st_r <= WB_IDLE;
    endcase
  end
end

// RPIX reader (after a flush, straight from the frame buffer like gsu.v)
reg [2:0]  rp_plane_r; initial rp_plane_r = 0;
reg [1:0]  rp_st_r;    initial rp_st_r = 0;
reg [15:0] rp_cshift_r; initial rp_cshift_r = 0;
always @(posedge CLK) if (CE) begin
  if (RST) begin
    rp_st_r <= 0; bmp_ram_rd_r <= 0; rpix_done_r <= 0;
  end
  else begin
    rpix_done_r <= 0;
    rp_cshift_r <= char_shift(char_num(rpix_off_r, SCMR_HT | {2{POR_OBJ}}), SCMR_MD);
    case (rp_st_r)
      2'd0: if (rpix_go_r) begin
        rp_plane_r <= 0; rpix_col_r <= 0; rp_st_r <= 1;
      end
      2'd1: begin
        bmp_addr_r <= 24'hE00000 + rp_cshift_r + {SCBR_r,10'h000} + {rpix_off_r[7:5],1'b0} + {rp_plane_r[2:1], 3'b000, rp_plane_r[0]};
        bmp_ram_rd_r <= 1;
        rp_st_r <= 2;
      end
      2'd2: if (ram_bmp_end) begin
        bmp_ram_rd_r <= 0;
        rpix_col_r[rp_plane_r] <= ram_bus_data_r[rpix_idx_r];
        rp_plane_r <= rp_plane_r + 1;
        if (rp_plane_r == bppm1) begin rp_st_r <= 3; rpix_done_r <= 1; end
        else rp_st_r <= 1;
      end
      default: rp_st_r <= 0;
    endcase
  end
end

//-------------------------------------------------------------------
// FX3 CLEAR ENGINE (MERGE R0=3/4/5)  -- see gsu.v for the layout notes
//-------------------------------------------------------------------
wire fx3_clr_ram_ok  = SAVERAM_MASK[16];
wire fx3_clr_col_end = (fx3_clr_cnt_r == 11'd1151);
assign clr_idle = ~fx3_clr_busy_r;

always @(posedge CLK) if (CE) begin
  if (RST) begin
    fx3_clr_busy_r <= 0; clr_ram_wr_r <= 0;
    fx3_clr_addr_r <= 0; fx3_clr_cnt_r <= 0; fx3_clr_col_r <= 0;
  end
  else if (~fx3_clr_busy_r) begin
    if (x_v & (x_cls == XC_MERGE) & x2m_go & fx3_clr_ram_ok) begin
      fx3_clr_busy_r <= 1;
      clr_ram_wr_r   <= 1;
      fx3_clr_cnt_r  <= 0;
      fx3_clr_col_r  <= 0;
      case (x2m_third)
        2'd0:    fx3_clr_addr_r <= 17'h10000;
        2'd1:    fx3_clr_addr_r <= 17'h12D00;
        default: fx3_clr_addr_r <= 17'h15A00;
      endcase
    end
  end
  else if (~go_r) begin
    clr_ram_wr_r <= 0; fx3_clr_busy_r <= 0;
  end
  else if (ram_clr_end) begin
    if (fx3_clr_col_end) begin
      if (fx3_clr_col_r == 4'd8) begin
        clr_ram_wr_r <= 0; fx3_clr_busy_r <= 0;
      end
      else begin
        fx3_clr_col_r  <= fx3_clr_col_r + 1;
        fx3_clr_cnt_r  <= 0;
        fx3_clr_addr_r <= fx3_clr_addr_r + 17'h81;
      end
    end
    else begin
      fx3_clr_cnt_r  <= fx3_clr_cnt_r + 1;
      fx3_clr_addr_r <= fx3_clr_addr_r + 1;
    end
  end
end

//-------------------------------------------------------------------
// OUTPUTS
//-------------------------------------------------------------------
assign DATA_ENABLE = data_enable_r;
assign DATA_OUT    = data_out_r;
assign GO          = go_r;
assign RON         = SCMR_r[4];
assign RAN         = SCMR_r[3];
assign IRQ         = irq_r;
assign DBG         = 1'b0;
assign config_data_out = 8'h00;

// minimal state readout for the MCU debug interface (registers + SFR)
reg [7:0] pgm_data_r; initial pgm_data_r = 0;
wire [15:0] pgm_reg = `RF(pgm_addr_r[4:1]);
always @(posedge CLK) if (CE) begin
  if (~|pgm_addr_r[9:5]) pgm_data_r <= pgm_addr_r[0] ? pgm_reg[15:8] : pgm_reg[7:0];
  else case (pgm_addr_r[7:0])
    8'h30: pgm_data_r <= SFR_w[7:0];
    8'h31: pgm_data_r <= SFR_w[15:8];
    8'h34: pgm_data_r <= PBR_r;
    8'h36: pgm_data_r <= ROMBR_r;
    8'h37: pgm_data_r <= CFGR_r;
    8'h38: pgm_data_r <= SCBR_r;
    8'h39: pgm_data_r <= CLSR_r;
    8'h3A: pgm_data_r <= SCMR_r;
    8'h3B: pgm_data_r <= 8'h52;
    8'h3C: pgm_data_r <= {7'h00,RAMBR_r};
    8'h3E: pgm_data_r <= CBR_r[7:0];
    8'h3F: pgm_data_r <= CBR_r[15:8];
    8'h40: pgm_data_r <= COLR_r;
    8'h41: pgm_data_r <= POR_r;
    default: pgm_data_r <= 8'hFF;
  endcase
end
assign PGM_DATA = pgm_data_r;

endmodule
