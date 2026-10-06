/* SuperRT on sd2snes: platform independent part of the MCU renderer.
   See superrt_core.h */
#include <string.h>
#include "superrt_core.h"

/* On the LPC1756 based mk3 the main RAM is small; keep the larger buffers in
   the AHB RAM there. */
#ifdef SRT_HOST_BUILD
#define SRT_BIGBUF
#else
#include "config.h"
#ifdef CONFIG_MK3_STM32
#define SRT_BIGBUF
#else
#define SRT_BIGBUF IN_AHBRAM
#endif
#endif

/* palette mapping k-d tree (exported by tools/srt_datagen.py):
   4 byte nodes {flags|axis, split, left, right}; flags bit 6/7: left/right
   child is a palette index instead of a node index. */
#define SRT_MAX_TREE_NODES 255
static uint8_t tree[SRT_MAX_TREE_NODES * 4] SRT_BIGBUF;
static uint8_t tree_nodes;
static uint8_t root_leaf;

/* one scan line, and one row of tiles (8 scan lines) in SNES format */
static uint16_t line_rgb[SRT_SCREEN_WIDTH];
static uint8_t tile_row[(SRT_SCREEN_WIDTH / 8) * 64] SRT_BIGBUF;

static uint16_t cache_col = 0xFFFF;
static uint8_t cache_idx;

static inline uint32_t le32(const uint8_t *p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static inline int16_t le16(const uint8_t *p) {
  return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

/* expand a 5 bit channel to 8 bits like the testbed's RGB555Split */
static inline uint8_t expand5(uint16_t v) {
  uint8_t c = (uint8_t)((v & 0x1F) << 3);
  return (c & 0x08) ? (c | 7) : c;
}

uint8_t srt_core_palette_index(uint16_t c) {
  uint8_t ch[3];
  uint8_t n = 0;
  if(c == cache_col) return cache_idx;
  cache_col = c;
  if(!tree_nodes) return cache_idx = root_leaf;
  ch[0] = expand5(c);
  ch[1] = expand5(c >> 5);
  ch[2] = expand5(c >> 10);
  for(;;) {
    const uint8_t *node = &tree[n * 4];
    if(ch[node[0] & 3] <= node[1]) {
      if(node[0] & 0x40) return cache_idx = node[2];
      n = node[2];
    } else {
      if(node[0] & 0x80) return cache_idx = node[3];
      n = node[3];
    }
    if(n >= tree_nodes) return cache_idx = 0; /* corrupt tree */
  }
}

int srt_core_init(void) {
  uint8_t hdr[64];
  uint32_t img, tree_off, off;
  int bank;

  srt_hal_mem_read(SRT_DESC_ADDR, hdr, sizeof(hdr));
  if(memcmp(hdr, "SRTMCU01", 8)) {
    tree_nodes = 0;
    root_leaf = 0;
    return SRT_ERR_NODESC;
  }
  tree_nodes = hdr[8];
  root_leaf = hdr[10];
  tree_off = le32(hdr + 12);
  img = le32(hdr + 16);
  if(tree_nodes > SRT_MAX_TREE_NODES) tree_nodes = SRT_MAX_TREE_NODES;
  if(tree_nodes) srt_hal_mem_read(SRT_DESC_ADDR + tree_off, tree, tree_nodes * 4);
  cache_col = 0xFFFF;

  /* start-up image into both framebuffer banks (reuses the tile buffer) */
  for(off = 0; off < SRT_FB_SIZE; off += sizeof(tile_row)) {
    uint16_t len = (SRT_FB_SIZE - off) < sizeof(tile_row) ? (uint16_t)(SRT_FB_SIZE - off) : (uint16_t)sizeof(tile_row);
    if(img) srt_hal_mem_read(img + off, tile_row, len);
    else memset(tile_row, 0, len);
    for(bank = 0; bank < 2; bank++) {
      srt_hal_mem_write(SRT_FB_BASE + bank * SRT_FB_BANK_SIZE + off, tile_row, len);
    }
  }
  return SRT_OK;
}

void srt_core_parse_params(const uint8_t *raw, srt_params_t *p) {
  p->ray_start_x = (int32_t)le32(raw + 0);
  p->ray_start_y = (int32_t)le32(raw + 4);
  p->ray_start_z = (int32_t)le32(raw + 8);
  p->ray_dir_x = le16(raw + 12);
  p->ray_dir_y = le16(raw + 14);
  p->ray_dir_z = le16(raw + 16);
  p->xstep_x = le16(raw + 18);
  p->xstep_y = le16(raw + 20);
  p->xstep_z = le16(raw + 22);
  p->ystep_x = le16(raw + 24);
  p->ystep_y = le16(raw + 26);
  p->ystep_z = le16(raw + 28);
  p->light_x = le16(raw + 30);
  p->light_y = le16(raw + 32);
  p->light_z = le16(raw + 34);
}

void srt_core_parse_cmdbuf(const uint8_t *raw, uint64_t *cmd) {
  int i, j;
  /* works in place (raw may alias cmd) */
  for(i = 0; i < SRT_CMDBUF_WORDS; i++) {
    uint64_t w = 0;
    for(j = 0; j < 8; j++) w = (w << 8) | raw[i * 8 + j];
    cmd[i] = w;
  }
}

/* convert one scan line (y = 0..7 within the tile row) of RGB555 pixels to
   palette indices and store it in the 25 SNES 8bpp tiles of the row
   (64 bytes per tile: bitplanes 0/1, 2/3, 4/5, 6/7 interleaved per row) */
static void line_to_tiles(int y) {
  int tx, k, i;
  for(tx = 0; tx < SRT_SCREEN_WIDTH / 8; tx++) {
    uint8_t p[8];
    uint8_t *t = &tile_row[tx * 64 + y * 2];
    for(i = 0; i < 8; i++) p[i] = srt_core_palette_index(line_rgb[tx * 8 + i] & 0x7FFF);
    for(k = 0; k < 8; k++) {
      uint8_t v = 0;
      for(i = 0; i < 8; i++) v |= (uint8_t)(((p[i] >> k) & 1) << (7 - i));
      t[(k >> 1) * 16 + (k & 1)] = v;
    }
  }
}

/* half resolution: pixels 0..99 of line_rgb -> each one twice */
static void line_double(void) {
  int x;
  for(x = SRT_SCREEN_WIDTH / 2 - 1; x >= 0; x--) {
    line_rgb[2 * x + 1] = line_rgb[x];
    line_rgb[2 * x] = line_rgb[x];
  }
}

int srt_core_render(const srt_params_t *p, const uint64_t *cmd, int bank, int half) {
  int ty, y;
  uint32_t base = SRT_FB_BASE + (uint32_t)bank * SRT_FB_BANK_SIZE;
  srt_frame_begin(p, cmd);
  for(ty = 0; ty < SRT_SCREEN_HEIGHT / 8; ty++) {
    for(y = 0; y < 8; y++) {
      if(half) {
        srt_render_line_half(ty * 8 + y, line_rgb);
        line_double();
      } else {
        srt_render_line(ty * 8 + y, line_rgb);
      }
      line_to_tiles(y);
      if(srt_hal_poll()) return 1;   /* also serves MSU-1 (at least) once per line */
    }
    srt_hal_mem_write(base + (uint32_t)ty * sizeof(tile_row), tile_row, sizeof(tile_row));
  }
  return 0;
}

int srt_core_render_hw(int bank, int half) {
  int ty, y;
  uint32_t base = SRT_FB_BASE + (uint32_t)bank * SRT_FB_BANK_SIZE;
  for(ty = 0; ty < SRT_SCREEN_HEIGHT / 8; ty++) {
    for(y = 0; y < 8; y++) {
      if(srt_hal_read_pixels(line_rgb, half ? SRT_SCREEN_WIDTH / 2 : SRT_SCREEN_WIDTH)) return 1;
      if(half) line_double();
      line_to_tiles(y);
      /* also serves MSU-1: with a fast engine the MCU rarely waits for pixels
         (srt_hal_read_pixels serves it while waiting), so serve it after
         every line as well */
      if(srt_hal_poll()) return 1;
    }
    srt_hal_mem_write(base + (uint32_t)ty * sizeof(tile_row), tile_row, sizeof(tile_row));
  }
  return 0;
}
