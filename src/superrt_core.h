/* SuperRT on sd2snes: platform independent part of the MCU side.
 *
 * The SNES side (register file, command buffer, frame handshake) and the ray
 * tracing engine live in the FPGA (verilog/sd2snes_superrt). The MCU takes
 * the RGB555 pixels of each frame - streamed from the engine's FIFO, or
 * rendered with the bit accurate software model of the chip (srt_render.c)
 * when the core has no engine - maps them to the 256 colour palette of the ROM
 * and writes SNES 8bpp tiles into the framebuffer bank the SNES is not
 * currently reading.
 *
 * Cartridge memory (PSRAM) layout used by the superrt core:
 *   0x000000  ROM (program at file offset 0x4000-0x7FFF)
 *   0x008000  "SRTMCU01" descriptor + palette mapping tree (in the ROM file)
 *   0x010000  32000 byte start-up image (in the ROM file)
 *   0x800000  framebuffer bank 0 (32000 bytes)
 *   0x808000  framebuffer bank 1
 */
#ifndef SUPERRT_CORE_H
#define SUPERRT_CORE_H

#include <stdint.h>
#include "srt_render.h"

#define SRT_FB_BASE          0x800000L
#define SRT_FB_BANK_SIZE     0x8000L
#define SRT_FB_SIZE          32000
#define SRT_DESC_ADDR        0x008000L
#define SRT_PARAM_BYTES      36
#define SRT_CMDBUF_BYTES     (SRT_CMDBUF_WORDS * 8)

/* FPGA status bits (SPI command 0xA0) */
#define SRT_STATUS_PENDING   0x01
#define SRT_STATUS_BUSY      0x02
#define SRT_STATUS_READBANK  0x04
#define SRT_STATUS_ENGINE    0x08   /* core has the hardware ray tracer */
#define SRT_STATUS_ENG_BUSY  0x10   /* hardware ray tracer is rendering */
#define SRT_STATUS_HALF      0x20   /* pending frame: half horizontal resolution
                                       (the ROM's choice, register $BEBA) */

/* size of the core's pixel FIFO (SPI 0xA5/0xA6) */
#define SRT_FIFO_PIXELS      2046

/* error codes of srt_core_init */
#define SRT_OK               0
#define SRT_ERR_NODESC       1

/* platform hooks, implemented by the firmware (superrt.c) or a test harness */
void srt_hal_mem_read(uint32_t addr, void *buf, uint16_t len);
void srt_hal_mem_write(uint32_t addr, const void *buf, uint16_t len);
/* called between rows of tiles while rendering; return nonzero to abort */
int srt_hal_poll(void);
/* hardware engine path: read n RGB555 pixels (raster order) from the FPGA's
   pixel FIFO, waiting for them as needed; return nonzero to abort */
int srt_hal_read_pixels(uint16_t *buf, uint16_t n);

/* read descriptor + palette tree from cartridge memory, fill both
   framebuffer banks with the start-up image */
int srt_core_init(void);

/* decode the 36 parameter bytes read from the FPGA */
void srt_core_parse_params(const uint8_t *raw, srt_params_t *p);

/* decode 4096 command buffer bytes (big endian words) into the renderer's
   native 64 bit word array */
void srt_core_parse_cmdbuf(const uint8_t *raw, uint64_t *cmd);

/* render a whole frame into framebuffer bank 'bank'; half: only every other
   pixel is traced (100 x 160) and shown twice.
   Returns 0 when complete, nonzero if srt_hal_poll() requested an abort. */
int srt_core_render(const srt_params_t *p, const uint64_t *cmd, int bank, int half);

/* same, but the pixels come from the FPGA's ray tracing engine
   (srt_hal_read_pixels; 100 per line with half set, as the engine was
   told by the ROM); only palette mapping and tile conversion run on the
   MCU */
int srt_core_render_hw(int bank, int half);

/* palette lookup of one RGB555 colour (exposed for tests) */
uint8_t srt_core_palette_index(uint16_t c);

#endif
