# sd2snes_superrt — SuperRT core for sd2snes mk3

FPGA core for running ROMs made for Ben Carter's
[SuperRT](https://github.com/ShironekoBen/superrt) ray tracing expansion chip
on the sd2snes / FXPAK Pro **mk3**.

The real chip uses a Cyclone V with three deeply pipelined ray tracing
engines. That needs about four times the logic and multipliers of the mk3's
EP4CE15. This core has the chip's **SNES interface** plus **one sequential
ray tracing engine** (`srt_engine.v`). The engine produces the same pixels as
the original chip (bit for bit), but more slowly. The MCU only maps the pixels
to the ROM's palette and writes them as SNES tiles (`../../src/superrt*.c`).

Derived from `sd2snes_obc1`: the OBC1 logic was replaced by `superrt.v`, and
**MSU-1** is a build option, on by default (see Build); the cheat engine /
in-game hook was removed.

## Files

| File | Contents |
|------|----------|
| `superrt.v` | register file, proxied writes, 512 × 64 bit command buffer, multiplier, frame handshake, engine + 2048 pixel FIFO |
| `srt_engine.v` | the ray tracer (pixel loop, ray phases, command list interpreter, sphere/plane/AABB tests, CSG, shading) |
| `srt_mul.v` | pipelined 32×32 (4 embedded multipliers each, 3 instances) and 16×16 (3 instances) multipliers |
| `address.v`, `main.v`, `mcu_cmd.v` | sd2snes glue (memory map, SPI commands `A0`–`A6`) |
| `sim/` | testbenches, see below |

## SNES memory map

As on the original chip, only A15–A0 are decoded, and every access is a
**read**. Everything mirrors through all banks where /ROMSEL is low.

| Address        | Function |
|----------------|----------|
| `$C000-$FFFF`  | program ROM, ROM file offset `$4000-$7FFF` (PSRAM) |
| `$8000-$BE7F`  | framebuffer window: 16000 bytes, the upper or lower half of the 32000 byte SNES 8bpp tile image (PSRAM) |
| `$BE80-$BEFF`  | registers (reading one selects it as the target of the next proxied write; MapUpperFB and MapLowerFB are the exceptions) |
| `$BF00-$BFFF`  | proxied write: reading `$BFxx` writes `xx` to the selected register and selects the next one |

Registers (offset from `$BE80`) are the same as in `SRT/SNESInterface.sv`:

| Offset | Register |
|--------|----------|
| `00` | NewFrame |
| `01` | MapUpperFB |
| `02` | MapLowerFB |
| `03-0E` | RayStart X/Y/Z (32 bit, 18.14) |
| `0F-14` | RayDir |
| `15-1A` | X step |
| `1B-20` | Y step (16 bit, 2.14) |
| `21-24` | multiplier A/B |
| `25-28` | multiplier output (`(A*B)>>14`) |
| `29-2A` | command write address |
| `2B-32` | command data (push 1..8 bits) |
| `33-38` | light direction |
| `39` | status (bit 0 = busy) |

## Frame flow

1. The SNES reads NewFrame while the core is not busy. The framebuffer bank
   shown to the SNES flips, *busy* is set, a request is flagged for the MCU,
   and **the engine starts rendering** with the current parameters and
   command list.
2. The engine writes RGB555 pixels in raster order into a 2048 entry FIFO. It
   stalls while the FIFO is full.
3. The MCU (`src/superrt.c`) sees the request and acknowledges it. It then
   streams the pixels out of the FIFO line by line, maps them to the palette,
   writes SNES tiles into the bank the SNES is **not** reading, and signals
   *done*, which clears busy.

The framebuffer banks are in PSRAM at `0x800000` and `0x808000` (32000 bytes
each, 500 tiles × 64 bytes, tile-major).

### Resolution mode (sd2snes addition)

Register `$BEBA` (index `$3A`, written like the others through the proxy
window): bit 0 clear = **half horizontal resolution (the default**, also after
configuration and for ROMs that never write the register), set = full
resolution. It is taken over at NewFrame. In half resolution the engine traces
only the even pixels (100 × 160; the pixel direction steps by twice the X step)
and the MCU shows each one twice, which roughly halves the render time.
Traced pixels are the same as in full resolution, except that the ordered
dither uses x / 2 so the doubled pixels alternate like normal ones
(`srt_render_line_half` in `src/srt_render.c` is the reference). The test ROM
(`superrt/rom/SRTTest-resmode.patch`) writes the register before every frame
and toggles it with Select.

## MCU commands (SPI)

| Cmd  | Function |
|------|----------|
| `A0` | status byte: bit 0 request pending, bit 1 busy, bit 2 bank read by the SNES, bit 3 hardware engine present, bit 4 engine rendering, bit 5 the pending frame is half horizontal resolution |
| `A1` | acknowledge request |
| `A2` | frame done (clears busy) |
| `A3` | stream render parameters (36 bytes: start X/Y/Z as LE int32, then dir, xstep, ystep, light as LE int16 triples) |
| `A4` | stream command buffer (4096 bytes, 512 big-endian 64 bit words) |
| `A5` | stream pixels from the FIFO: 2 bytes per pixel, RGB555 little endian (R in bits 4:0). Read only as many as `A6` reported. |
| `A6` | engine info, captured when the SPI message starts: FIFO level (2 bytes, big endian), engine busy, engine cycle count of the current or last frame (4 bytes, big endian) |

`A3`/`A4` are there for the software renderer fallback, which the firmware
uses when bit 3 of the status is clear (cores without the engine).

## Engine

* **Bit-exact** with the original RTL:
  multiplier truncations, 16 bit wrap-arounds, Newton-Raphson with the
  chip's seed table.
* **Structure:** one state machine; three 32×32 multipliers (latency 3), six
  16×16 lanes, one 32×32 fast lane (latency 2) for Newton-Raphson chains and
  dot × reciprocal.
* **Result-neutral speed-ups:** seed² from a table; plane reciprocal skipped
  where only its sign matters; reciprocal cache (256 entries) addressed straight
  from the multiplier sums; per-instruction plane point cache; 1/radius cache;
  all plane products issued in the decode state; no fetch bubble on jumps;
  CSG skipped on misses.
* **Clocking:** engine on PLL `c1` (80 MHz), rest at 96 MHz. CDC via start
  toggle, Gray-coded dual-clock FIFO, restart handshake.
* **Timing:** hit-valid flag registered; `he`/`hx` written only in decode
  (pending updates); carry-save sums; plane sign tests as bit tests; shading
  sum registered before the clamp. `main.qsf`: HIGH PERFORMANCE EFFORT +
  physical synthesis.

## Differences from the original chip

* Command words are written at the intended address (the original wrote one
  too far, so jumps landed one instruction early). Same output for the demo.
* Register reads other than multiplier output and status return `$00`.
* Lower frame rate; half resolution mode added.


## Build

The Quartus project is set up the same way as for the other cores
(`main.qsf`, `sd2snes_superrt.qpf`):

    make mk3          # in this directory -> fpga_superrt.bi3

Copy `fpga_superrt.bi3` to `/sd2snes/` on the SD card. The firmware loads it
for ROMs whose header title starts with `SUPERRT` (needs the firmware from this
branch).

Fmax moves by ±3 MHz between builds at this device use.

### Build history

| Build | LEs | Clock | Result |
|-------|-----|-------|--------|
| 2nd | 13 115 | 64 MHz | met, works on hardware |
| 5th | 13 498 | 72 MHz | met, works on hardware |
| 6th | — | 76.8 MHz | met, works on hardware (no MSU-1) |
| 8th | 13 295 | 76.8 MHz | met, works on hardware (MSU-1 back) |
| 13th | 13 933 | 76.8 MHz | 82.3 MHz, works on hardware (half resolution, fast lane) |
| **15th** | **13 997** | **80 MHz** | **85.1 MHz, works on hardware** |

Builds 1, 3, 4, 7, 9, 11, 12 and 14 missed timing. Each was fixed by
restructuring the failing path (see git history).
cFPGA flip-flops do. It also removes a blocking-assignment race in
`spi.v` that only exists in simulation.
