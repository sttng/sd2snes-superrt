// Engine vs. srt_render.c, pixel by pixel.
// usage: tb_engine [-r seed] [-n frames] cmdbuf.bin params...   (params files: 15 ints)
//   -r seed : instead of cmdbuf.bin use random command lists (cmdbuf arg still read as base)
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <string>
#include "Vtb_engine_top.h"
#include "verilated.h"
#include "Vtb_engine_top___024root.h"
static uint64_t st_hist[128], nr_hist[128];
#include "srt_render.h"  // srt_render.c is compiled as C++ by the Verilator makefile

static Vtb_engine_top *top;
static uint64_t cyc = 0;
static void tick() { top->clk = 0; top->eval(); top->clk = 1; top->eval(); cyc++; }

static uint32_t rs = 1;
static uint32_t rnd() { rs ^= rs << 13; rs ^= rs >> 17; rs ^= rs << 5; return rs; }
static uint64_t rbits(int n) { uint64_t v = 0; for(int i = 0; i < n; i++) v = (v << 1) | (rnd() & 1); return v; }

// random but plausible command list
static void gen_random(uint64_t *cmd) {
  int n = 0;
  memset(cmd, 0, 512 * 8);
  auto put = [&](uint64_t w) { if(n < 511) cmd[n++] = w; };
  auto cond = [&]() -> uint64_t { uint32_t r = rnd() % 8; return (uint64_t)(r < 5 ? 0 : r - 4) << 6; };
  auto s15 = [&](int range) -> uint64_t { int v = (int)(rnd() % (2 * range + 1)) - range; return (uint64_t)v & 0x7FFF; }; // 8.7
  auto s9 = [&](int range) -> uint64_t { int v = (int)(rnd() % (2 * range + 1)) - range; return (uint64_t)v & 0x1FF; };  // 8.1
  put(17 | cond());
  int groups = 3 + rnd() % 10;
  for(int g = 0; g < groups; g++) {
    if(rnd() % 4 == 0) put(16 | (s15(600) << 8) | (s15(300) << 23) | (s15(800) << 38));
    int prims = 1 + rnd() % 4;
    for(int p = 0; p < prims; p++) {
      int kind = rnd() % 3, mode = (p == 0) ? 0 : rnd() % 3;
      uint64_t c = (p == 0) ? 0 : cond();
      if(rnd() % 6 == 0) c = cond();
      uint64_t op;
      if(kind == 0) {
        op = mode == 0 ? 1 : mode == 1 ? 3 : 5;
        uint64_t rad = 20 + rnd() % 400;
        if(rnd() % 10 == 0) rad = rbits(11);
        put(op | c | (s15(1200) << 8) | (s15(500) << 23) | (s15(1500) << 38) | (rad << 53));
      } else if(kind == 1) {
        op = mode == 0 ? 2 : mode == 1 ? 4 : 6;
        // normal roughly unit length in 2.10
        int nx = (int)(rnd() % 2049) - 1024, ny = (int)(rnd() % 2049) - 1024, nz = (int)(rnd() % 2049) - 1024;
        if(rnd() % 3 == 0) { nx = 0; ny = (rnd() & 1) ? 1024 : -1024; nz = 0; }
        uint64_t dist = (uint64_t)((int)(rnd() % 40001) - 20000) & 0xFFFFF;
        if(rnd() % 10 == 0) dist = rbits(20);
        put(op | c | ((uint64_t)(nx & 0xFFF) << 8) | ((uint64_t)(ny & 0xFFF) << 20) | ((uint64_t)(nz & 0xFFF) << 32) | (dist << 44));
      } else {
        op = mode == 0 ? 7 : mode == 1 ? 8 : 9;
        int x0 = (int)(rnd() % 120) - 60, y0 = (int)(rnd() % 60) - 30, z0 = (int)(rnd() % 120) - 60;
        int x1 = x0 + rnd() % 40, y1 = y0 + rnd() % 40, z1 = z0 + rnd() % 40;
        if(rnd() % 10 == 0) { x1 = (int)rbits(9); }
        put(op | c | ((uint64_t)(x0 & 0x1FF) << 8) | ((uint64_t)(y0 & 0x1FF) << 17) | ((uint64_t)(z0 & 0x1FF) << 26) |
            ((uint64_t)(x1 & 0x1FF) << 35) | ((uint64_t)(y1 & 0x1FF) << 44) | ((uint64_t)(z1 & 0x1FF) << 53));
      }
    }
    if(rnd() % 3 == 0) put(12 | cond() | (rbits(8) << 8) | (rbits(16) << 16));
    if(rnd() % 8 == 0) put(13 | cond());
    if(rnd() % 8 == 0) { put(14 | cond() | ((uint64_t)(n + 2) << 8)); put(rbits(6) | (rbits(58) << 6)); }
    if(rnd() % 10 == 0) { put(15 | cond() | ((uint64_t)(n + 1) << 8)); }
    put((rnd() % 4 ? 10 : 11) | cond() | (rbits(8) << 8) | (rbits(16) << 16));
    if(rnd() % 12 == 0) put(18 | (uint64_t)((rnd() % 3) + 1) << 6);
    if(rnd() % 20 == 0) put(rbits(64) & ~0x3FULL | (19 + rnd() % 40)); // undefined opcode = nop
  }
  put(18);
}

int main(int argc, char **argv) {
  Verilated::commandArgs(argc, argv);
  int seed = -1, frames = 1, stall = 1, maxerr = 10;
  std::vector<std::string> pfiles;
  const char *cbfile = nullptr;
  for(int i = 1; i < argc; i++) {
    if(!strcmp(argv[i], "-r")) seed = atoi(argv[++i]);
    else if(!strcmp(argv[i], "-n")) frames = atoi(argv[++i]);
    else if(!strcmp(argv[i], "-nostall")) stall = 0;
    else if(!cbfile) cbfile = argv[i];
    else pfiles.push_back(argv[i]);
  }
  static uint64_t cmd[512];
  memset(cmd, 0, sizeof(cmd));
  if(cbfile && strcmp(cbfile, "-")) {
    FILE *f = fopen(cbfile, "rb"); if(!f) { perror(cbfile); return 1; }
    unsigned char b[8]; int n = 0;
    while(n < 512 && fread(b, 1, 8, f) == 8) { uint64_t w = 0; for(int j = 0; j < 8; j++) w = (w << 8) | b[j]; cmd[n++] = w; }
    fclose(f);
  }
  top = new Vtb_engine_top;
  top->start = 0; top->we = 0; top->pix_full = 0;
  for(int i = 0; i < 5; i++) tick();
  if(seed >= 0) rs = 0x9E3779B9u ^ (uint32_t)seed * 2654435761u;
  if(!rs) rs = 1;

  long total_err = 0, total_px = 0;
  uint64_t total_cycles = 0;
  int nf = 0;
  for(size_t pi = 0; pi < pfiles.size(); pi++) {
    long v[15];
    FILE *f = fopen(pfiles[pi].c_str(), "r"); if(!f) { perror(pfiles[pi].c_str()); return 1; }
    for(int i = 0; i < 15; i++) if(fscanf(f, "%ld", &v[i]) != 1) { fprintf(stderr, "bad params\n"); return 1; }
    fclose(f);
    for(int fr = 0; fr < frames; fr++) {
      if(seed >= 0) gen_random(cmd);
      // load command buffer
      for(int i = 0; i < 512; i++) { top->we = 1; top->waddr = i; top->wdata = cmd[i]; tick(); }
      top->we = 0;
      srt_params_t p;
      p.ray_start_x = v[0]; p.ray_start_y = v[1]; p.ray_start_z = v[2];
      p.ray_dir_x = v[3]; p.ray_dir_y = v[4]; p.ray_dir_z = v[5];
      p.xstep_x = v[6]; p.xstep_y = v[7]; p.xstep_z = v[8];
      p.ystep_x = v[9]; p.ystep_y = v[10]; p.ystep_z = v[11];
      p.light_x = v[12]; p.light_y = v[13]; p.light_z = v[14];
      static uint16_t ref[32000];
      srt_frame_begin(&p, cmd);
      for(int y = 0; y < 160; y++) srt_render_line(y, ref + y * 200);

      top->i_sx = p.ray_start_x; top->i_sy = p.ray_start_y; top->i_sz = p.ray_start_z;
      top->i_dx = (uint16_t)p.ray_dir_x; top->i_dy = (uint16_t)p.ray_dir_y; top->i_dz = (uint16_t)p.ray_dir_z;
      top->i_xsx = (uint16_t)p.xstep_x; top->i_xsy = (uint16_t)p.xstep_y; top->i_xsz = (uint16_t)p.xstep_z;
      top->i_ysx = (uint16_t)p.ystep_x; top->i_ysy = (uint16_t)p.ystep_y; top->i_ysz = (uint16_t)p.ystep_z;
      top->i_lx = (uint16_t)p.light_x; top->i_ly = (uint16_t)p.light_y; top->i_lz = (uint16_t)p.light_z;
      top->start = 1; tick(); top->start = 0;
      int n = 0, err = 0;
      uint64_t c0 = cyc;
      while(n < 32000) {
        if(stall) top->pix_full = (rnd() % 16) == 0;
        st_hist[top->rootp->tb_engine_top__DOT__eng__DOT__state + (top->rootp->tb_engine_top__DOT__eng__DOT__wcnt?0:0)]++;
        if(top->rootp->tb_engine_top__DOT__eng__DOT__state==64 && !top->rootp->tb_engine_top__DOT__eng__DOT__wcnt) nr_hist[top->rootp->tb_engine_top__DOT__eng__DOT__nr_ret]++;
        tick();
        if(top->pix_we) {
          uint16_t got = top->pix_data, exp = ref[n] & 0x7FFF;
          if(got != exp) {
            if(err < maxerr) printf("  %s frame %d pixel (%d,%d): got %04x exp %04x\n", pfiles[pi].c_str(), fr, n % 200, n / 200, got, exp);
            err++;
          }
          n++;
        }
        if(cyc - c0 > 400000000ULL) { printf("TIMEOUT\n"); return 2; }
      }
      top->pix_full = 0;
      for(int i = 0; i < 4; i++) tick();
      if(top->busy) { printf("engine still busy after last pixel\n"); err++; }
      printf("%s%s frame %d: %d mismatches, %u engine cycles (%.3f s @ 80 MHz), C: %u isects %u rays\n",
             pfiles[pi].c_str(), seed >= 0 ? " (random)" : "", fr, err, top->cycles, top->cycles / 80e6,
             srt_stats.instructions, srt_stats.rays);
      total_err += err; total_px += 32000; total_cycles += top->cycles; nf++;
    }
  }
  printf("TOTAL: %d frames, %ld pixels, %ld mismatches, avg %.0f cycles/frame\n", nf, total_px, total_err, (double)total_cycles / nf);
  for(int i=0;i<128;i++) if(st_hist[i]) printf("state %3d: %10llu\n", i, (unsigned long long)st_hist[i]);
  for(int i=0;i<128;i++) if(nr_hist[i]) printf("nr ret %3d: %10llu\n", i, (unsigned long long)nr_hist[i]);
  delete top;
  return total_err ? 1 : 0;
}
