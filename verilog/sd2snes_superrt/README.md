# sd2snes_superrt — SuperRT core for sd2snes mk3

FPGA core for ROMs made for Ben Carter's
[SuperRT](https://github.com/ShironekoBen/superrt) ray tracing chip on the
sd2snes / FXPAK Pro **mk3**.

The original chip has three pipelined engines on a Cyclone V (~4× the EP4CE15).
This core has the chip's SNES interface plus **one sequential engine**
(`srt_engine.v`, 80 MHz). The engine's pixels are bit-identical to the original's. The
MCU maps them to the ROM palette and writes SNES tiles (`../../src/superrt*.c`).
Derived from `sd2snes_obc1`, with MSU-1 (build option, default on) and no cheat engine.

**Status:** the 17th build works on hardware: multiplier latency 2 (12.6 %
fewer cycles than the 15th), 80 MHz engine clock, 13 710 LEs (89 %), engine
Fmax 80.8 MHz, 96 MHz domain 98.4 MHz. Needs the firmware that serves MSU-1
after every line (otherwise the audio crackles once per frame).

Engine time per frame, 15th build → 17th build:

| Mode | Demo start view | 12 test views (avg.) |
|------|-----------------|----------------------|
| Half horizontal resolution (default) | 15.0 → 13.3 M cycles, 5.3 → 6.0 fps | 11.0 → 9.6 M, 7.3 → 8.3 fps |
| Full resolution | 30.0 → 26.5 M cycles, 2.7 → 3.0 fps | 22.0 → 19.2 M, 3.6 → 4.2 fps |

On screen add about 10 ms per frame on the SNES side (command list upload). In
half resolution the MCU's work per frame (pixel stream, palette mapping, tile
writes) now takes about as long as the engine: the demo start view shows a new
frame every 0.19 s (5.2 fps) on hardware, the engine alone needs 0.17 s.

## Files

| File | Contents |
|------|----------|
| `superrt.v` | registers, proxied writes, 512 × 64 bit command buffer, multiplier, frame handshake, clock domain crossing, 2048 pixel FIFO |
| `srt_engine.v` | ray tracer: pixel loop, command interpreter, sphere/plane/AABB, CSG, shading |
| `srt_mul.v` | 32×32 / 16×16 multipliers (latency 2, 3 or 4), `srt_mulf` fast lane |
| `address.v`, `main.v`, `mcu_cmd.v` | sd2snes glue (memory map, SPI commands) |
| `sim/` | testbenches |

## SNES interface

Only A15–A0 are decoded and every access is a read, as on the original.

| Address | Function |
|---------|----------|
| `$C000-$FFFF` | program ROM (file offset `$4000-$7FFF`) |
| `$8000-$BE7F` | framebuffer window: upper / lower half of the 32000 byte 8bpp tile image |
| `$BE80-$BEFF` | registers (a read selects the target of the next proxied write) |
| `$BF00-$BFFF` | proxied write: reading `$BFxx` writes `xx` |

Registers match `SRT/SNESInterface.sv` (`00` NewFrame, `01/02` Map upper/lower
FB, `03-20` ray start/dir/steps, `21-28` multiplier, `29-32` command write,
`33-38` light, `39` status). Addition: **`3A` mode**, bit 0 = full resolution,
default 0 = half, taken over at NewFrame.

**Frame flow:** NewFrame flips the displayed bank, sets busy and starts the
engine. The engine writes RGB555 pixels into the FIFO (stalls when full). The
MCU streams them, writes tiles into the hidden bank (PSRAM `0x800000` /
`0x808000`) and signals done. In half resolution the engine traces even pixels
only (dither uses x / 2) and the MCU doubles them; reference:
`srt_render_line_half` in `src/srt_render.c`.

## MCU commands (SPI)

| Cmd | Function |
|-----|----------|
| `A0` | status: bit 0 pending, 1 busy, 2 SNES bank, 3 engine present, 4 engine rendering, 5 half resolution |
| `A1` / `A2` | acknowledge request / frame done |
| `A3` / `A4` | stream render parameters / command buffer (software fallback for cores without engine) |
| `A5` | stream pixels (RGB555 LE), as many as `A6` reports |
| `A6` | FIFO level, engine busy, cycle count |

## Engine

* **Bit-exact** with `src/srt_render.c` (itself bit-identical to the original
  RTL): multiplier truncations, 16 bit wrap-arounds, Newton-Raphson with the
  chip's seed table.
* **Structure:** one state machine; three 32×32 multipliers, six 16×16 lanes
  and a 32×32 fast lane (Newton-Raphson chains, dot × reciprocal), all with
  latency 2 (`MUL_LAT`; the 32×32 products are summed in the same cycle as the
  embedded multipliers, like the fast lane). `MUL_LAT` 3 / 4 give the
  multipliers more time, bit-exact as well, for about +13 % / +26 % cycles.
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

    make mk3          # -> fpga_superrt.bi3, copy to /sd2snes/

* **Engine clock:** `clk1_multiply_by` / `clk1_divide_by` in `ip/mk3/pll.v`
  and `-multiply_by` / `-divide_by` in `main.sdc` (input 8 MHz). 80 MHz =
  10/1; fallbacks 76.8 MHz = 48/5, 72 MHz = 9/1.
* **Multiplier latency:** `MUL_LAT` at the top of `srt_engine.v`, 2 (default).
  If the multiplier paths fail timing: 3 (the 15th build's multipliers) or 4.
* **MSU-1:** `VERILOG_MACRO "SRT_MSU1=1"` in `main.qsf`; remove to save ~900 LEs.
  The firmware serves MSU-1 from the SuperRT loop.

Fmax moves by ±3 MHz between builds at this device use.

### Build history

| Build | LEs | Clock | Result |
|-------|-----|-------|--------|
| 2nd | 13 115 | 64 MHz | met, works on hardware |
| 5th | 13 498 | 72 MHz | met, works on hardware |
| 6th | — | 76.8 MHz | met, works on hardware (no MSU-1) |
| 8th | 13 295 | 76.8 MHz | met, works on hardware (MSU-1 back) |
| 13th | 13 933 | 76.8 MHz | 82.3 MHz, works on hardware (half resolution, fast lane) |
| 15th | 13 997 | 80 MHz | 85.1 MHz, works on hardware |
| 16th | 13 662 (89 %) | 80 MHz | multiplier latency 2 (−12.6 % cycles): 78.9 MHz, −0.17 ns, one path: command RAM → `mb0` (speculative plane product in decode) |
| **17th** | **13 710 (89 %)** | **80 MHz** | **80.8 MHz, works on hardware** (speculative plane products take the normal from the plane point cache's tag copy, not the command RAM) |

Builds 1, 3, 4, 7, 9, 11, 12, 14 and 16 missed timing. Each was fixed by
restructuring the failing path (see git history).

## Simulation

| Command | Checks |
|---------|--------|
| `sim/engine/run_tests.sh <CommandBuffer.bin> [seeds]` | engine vs `srt_render.c`: 12 cameras, random command lists, random FIFO back-pressure, half mode (Verilator; `MUL_LAT=3 sim/engine/build.sh` for the other latencies) |
| `sim/run_sim.sh <SRTTest.sfc>` | whole core replaying emulator SNES/MCU traffic, 2 engine frames compared pixel by pixel (Verilator); plus MSU-1 ID and audio path; `FULLRES=1` full resolution, `SWPATH=1` software path (Icarus), `MSU=0` core without MSU-1 |
| `sim/engine/prof/prof.sh` | cycles per engine state |

`sim/mksim.py` makes simulation copies with flip-flops starting at 0 and fixes
simulation-only blocking-assignment races in `spi.v` and `msu.v`.
