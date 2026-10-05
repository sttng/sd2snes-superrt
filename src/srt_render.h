/*
 * SuperRT software renderer
 *
 * Bit-accurate C port of the SuperRT ray tracing chip (Ben Carter, 2021, MIT
 * licensed) for running on a microcontroller (sd2snes mk3 STM32F401) or on a
 * host PC.  The arithmetic follows the SystemVerilog RTL (SRT/ *.sv files), not the
 * C# testbed, so the output matches what the original FPGA produces.
 *
 * All values use the chip's fixed point formats:
 *   positions / depths : signed 18.14 in int32_t
 *   directions         : signed 2.14 in int16_t
 */
#ifndef SRT_RENDER_H
#define SRT_RENDER_H

#include <stdint.h>

#define SRT_SCREEN_WIDTH   200
#define SRT_SCREEN_HEIGHT  160
#define SRT_CMDBUF_WORDS   512

typedef struct {
  int32_t ray_start_x, ray_start_y, ray_start_z; /* camera position, 18.14 */
  int16_t ray_dir_x, ray_dir_y, ray_dir_z;       /* top left ray (unnormalised), 2.14 */
  int16_t xstep_x, xstep_y, xstep_z;             /* per pixel step, 2.14 */
  int16_t ystep_x, ystep_y, ystep_z;             /* per line step, 2.14 */
  int16_t light_x, light_y, light_z;             /* light direction, 2.14 */
} srt_params_t;

/* Optional statistics (instructions dispatched) */
typedef struct {
  uint32_t instructions;
  uint32_t rays;
} srt_stats_t;

extern srt_stats_t srt_stats;

/* Set up a frame. cmdbuf must stay valid while rendering (512 x 64 bit words,
   native integers - i.e. already converted from the big endian file layout) */
void srt_frame_begin(const srt_params_t *params, const uint64_t *cmdbuf);

/* Render one scan line (0..159) into 200 RGB555 pixels (R = bits 4:0),
   including the chip's ordered dither. Lines may be rendered in any order. */
void srt_render_line(int y, uint16_t *out);

/* Half horizontal resolution: the 100 even pixels (x = 0, 2, .. 198) of a
   line, traced as in srt_render_line but dithered as pixel x / 2 */
void srt_render_line_half(int y, uint16_t *out);

/* Render a single pixel */
uint16_t srt_render_pixel(int x, int y);

#endif
