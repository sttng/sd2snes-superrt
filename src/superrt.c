/* SuperRT ray tracing chip support for sd2snes mk3.
 *
 * The FPGA core (verilog/sd2snes_superrt) implements the chip's SNES
 * interface and a hardware ray tracer. This loop waits for frame requests,
 * streams the pixels the FPGA renders, maps them to the palette and writes
 * them as SNES tiles into the framebuffer bank the SNES is not reading
 * (superrt_core.c).
 * With a core without the hardware ray tracer it fetches the render
 * parameters and the command list instead and renders the frame with the
 * software model of the chip (srt_render.c).
 */
#include <string.h>
#include "config.h"
#include "uart.h"
#include "fpga_spi.h"
#include "snes.h"
#include "memory.h"
#include "cli.h"
#include "usbinterface.h"
#include "timer.h"
#include "led.h"
#include "superrt.h"
#include "superrt_core.h"
#include "msu1.h"
#include "smc.h"

extern snes_romprops_t romprops;

#define DBG_SRT while(0)

#ifdef CONFIG_MK3_STM32
static uint64_t srt_cmd[SRT_CMDBUF_WORDS];
#else
static uint64_t srt_cmd[SRT_CMDBUF_WORDS] IN_AHBRAM;
#endif
static uint8_t srt_reset_state;

void srt_hal_mem_read(uint32_t addr, void *buf, uint16_t len) {
  sram_readblock(buf, addr, len);
}

/* PSRAM writes go byte by byte with a handshake each; a whole tile row
   (1600 bytes) is written in chunks with MSU-1 served in between, so that
   the audio buffer is refilled in time also when the engine never makes the
   MCU wait for pixels */
#define SRT_MEM_WRITE_CHUNK 200
static void srt_msu_service(void);
void srt_hal_mem_write(uint32_t addr, const void *buf, uint16_t len) {
  const uint8_t *p = buf;
  while(len) {
    uint16_t n = len > SRT_MEM_WRITE_CHUNK ? SRT_MEM_WRITE_CHUNK : len;
    sram_writeblock((void *)p, addr, n);
    p += n; addr += n; len -= n;
    if(len) srt_msu_service();
  }
}

/* MSU-1 (ROMs with a .msu file, core built with MSU-1): its audio buffer
   holds about 6 ms, so it is served at least once per scan line */
static void srt_msu_service(void) {
  if(romprops.has_msu1) msu1_service();
}

int srt_hal_read_pixels(uint16_t *buf, uint16_t n) {
  srt_msu_service();
  while(n) {
    uint16_t avail = fpga_srt_info(NULL);
    if(!avail) {
      if(srt_hal_poll()) return 1;   /* also serves MSU-1 */
      continue;
    }
    if(avail > n) avail = n;
    fpga_srt_read(FPGA_CMD_SRT_PIXELS, (uint8_t *)buf, avail * 2);
    buf += avail;
    n -= avail;
  }
  return 0;
}

/* called between rows of tiles: keep the console responsive */
int srt_hal_poll(void) {
  srt_msu_service();
  cli_entrycheck();
  srt_reset_state = get_snes_reset_state();
  return srt_reset_state != SNES_RESET_NONE;
}

int superrt_loop(void) {
  uint8_t cmd;
  int res;
  srt_params_t params;
  uint32_t frames = 0;
  tick_t t0;

  /* MSU-1 first: the SNES is already running and may request a track */
  if(romprops.has_msu1) {
    printf("SuperRT: MSU-1 data file found, serving MSU-1\n");
    msu1_start();
  }
  res = srt_core_init();
  printf("SuperRT: init %s\n", res == SRT_OK ? "ok" : "failed (no SRTMCU01 descriptor in ROM)");
  srt_reset_state = SNES_RESET_NONE;

  while(srt_reset_state == SNES_RESET_NONE) {
    uint8_t status;

    srt_reset_state = get_snes_reset_state();
    cmd = snes_get_mcu_cmd();
    if(cmd) {
      switch(cmd) {
        case SNES_CMD_RESET_LOOP_FAIL:
          srt_reset_state = SNES_RESET_SHORT;
          snes_reset_loop();
          break;
        case SNES_CMD_RESET:
          srt_reset_state = SNES_RESET_SHORT;
          snes_reset_pulse();
          break;
        case SNES_CMD_RESET_TO_MENU:
          srt_reset_state = SNES_RESET_LONG;
          break;
        default:
          printf("SuperRT: unsupported cmd: %02x\n", cmd);
          break;
      }
      snes_set_mcu_cmd(0);
    }
    cli_entrycheck();
    usbint_handler();
    srt_msu_service();

    status = fpga_srt_status();
    if(status & SRT_STATUS_PENDING) {
      int bank = (status & SRT_STATUS_READBANK) ? 0 : 1;
      fpga_srt_ack();
      t0 = getticks();
      writeled(1);
      if(status & SRT_STATUS_ENGINE) {
        /* the FPGA started rendering when the SNES requested the frame */
        res = srt_core_render_hw(bank, (status & SRT_STATUS_HALF) != 0);
      } else {
        uint8_t raw[SRT_PARAM_BYTES];
        fpga_srt_read(FPGA_CMD_SRT_PARAMS, raw, sizeof(raw));
        srt_core_parse_params(raw, &params);
        fpga_srt_read(FPGA_CMD_SRT_CMDBUF, (uint8_t *)srt_cmd, SRT_CMDBUF_BYTES);
        srt_core_parse_cmdbuf((uint8_t *)srt_cmd, srt_cmd);
        res = srt_core_render(&params, srt_cmd, bank, (status & SRT_STATUS_HALF) != 0);
      }
      writeled(0);
      /* always release the SNES side, even when aborted by a reset */
      fpga_srt_done();
      frames++;
      DBG_SRT printf("SuperRT: frame %lu -> bank %d in %lu ticks%s\n", frames, bank, (unsigned long)(getticks() - t0), res ? " (aborted)" : "");
      (void)t0;
    }
  }
  printf("SuperRT: %lu frames rendered, reset %s\n", frames, srt_reset_state == SNES_RESET_LONG ? "to menu" : "game");
  if(romprops.has_msu1) return msu1_stop(srt_reset_state);
  return srt_reset_state == SNES_RESET_LONG;
}
