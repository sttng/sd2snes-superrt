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

* **Bit-exact.** It walks the same algorithm as `src/srt_render.c`, which is
  bit-identical to the original RTL: the 40 and 48 bit multiplier
  truncations, 16 bit wrap-arounds, and the Newton-Raphson reciprocal and
  square root with the chip's seed table.
* **Structure.** A single state machine drives three pipelined 32×32
  multipliers (latency 3), three 16×16 ones, and a 32×32 "fast lane" with a
  single register stage (latency 2) for the dependent chains. The command list is read
  through the second port of the command buffer RAM.
* **Speed-ups** that do not change any result:
  * The first Newton-Raphson multiply (seed²) comes from a table.
  * The plane reciprocal is skipped where only its sign matters.
  * A 256 entry cache holds plane reciprocals. Shadow rays share one
    direction, so their denominators repeat.
  * A per-instruction cache holds each plane's reference point (normal ×
    distance), read together with the instruction.
  * A one-entry cache holds 1/radius.
  * Missed intersections skip the CSG step.
  * RegisterHit without a hit state finishes in the decode cycle.
  * All plane products (dot product on the 32×32 lanes, N·D and (−N)·D on
    two sets of 16×16 lanes) are issued while the instruction is decoded.
    The reciprocal cache is addressed straight from the multiplier sums, so
    the cache result and dot × reciprocal follow in the next state.
  * Single-value reciprocals and square roots (plane, sphere, sphere normal,
    pixel normalisation) run their Newton-Raphson chain on the fast lane,
    2 cycles per dependent multiply instead of 3; a plane reciprocal from
    Newton-Raphson goes straight into the cache and dot × reciprocal. The
    plane result multiply uses the fast lane too.
  * Taken jumps and the start of a ray present the new address to the
    command RAM directly, with no fetch bubble.
  * Sphere radius² comes from a 16×16 lane; shading uses the 16×16 lanes
    for D·N in parallel with N·L.
* **Clock.** The engine has its own clock, PLL output `c1` = 80 MHz (13th build: Fmax 82.3 MHz at 76.8 MHz; fallback 76.8 MHz = `clk1_multiply_by 48` / `clk1_divide_by 5`)
  (`ip/mk3/pll.v`, `main.sdc`). The rest of the core keeps the 96 MHz system
  clock. `superrt.v` holds the clock domain crossing: a start toggle, a
  dual-clock pixel FIFO with Gray-coded pointers, and a restart handshake.
* **Timing work.**
  * The hit-state "valid" flag (he < hx) is a register, so no 32 bit compare
    sits in front of the instruction condition and CSG logic.
  * `he`/`hx` and the hit normal are only written in S_DEC. Resets, CSG
    results and missed-intersection updates are stored as pending updates
    and applied when the next instruction is decoded. Every instruction
    passes through S_DEC before anything reads them, so this costs no
    cycles.
  * Sphere decisions are made from registers.
  * Three-operand sums, including the multipliers' partial products, are
    carry-save.
* **Speed (full resolution).** About 11–34 M cycles per frame depending on the
  view (≈2.3–7 fps at 76.8 MHz). The 12 test cameras average 22.0 M cycles
  (≈3.5 fps; previous version 25.4 M, ≈3.0 fps), and the demo's start view
  takes 30.0 M (≈2.56 fps; previously 35.7 M, ≈2.15 fps).
* **Speed (half resolution, the default).** The 12 test cameras average
  11.0 M cycles (≈7 fps), the demo's start view 15.0 M (≈5.1 fps). The software renderer needed
  several seconds per frame, and the original chip did about 20 fps.
* **State.** Engine registers are not reset between frames, the same as the
  static state in `srt_render.c`. The FPGA flip-flops start at 0.

## Differences from the original chip

* The original wrote each command word one address too far: the write strobe
  and the address increment happened on the same clock, so every jump in the
  command list landed one instruction early. This core writes at the intended
  address. The output is identical for the demo scene, and it saves some work.
* Register reads other than the multiplier output and status return `$00`.
  The original returned framebuffer bytes there, which the SNES ignores.
* The frame rate is lower: see above.
* MSU-1 is optional (default on).

## Build

The Quartus project is set up the same way as for the other cores
(`main.qsf`, `sd2snes_superrt.qpf`):

    make mk3          # in this directory -> fpga_superrt.bi3

Copy `fpga_superrt.bi3` to `/sd2snes/` on the SD card. The firmware loads it
for ROMs whose header title starts with `SUPERRT` (needs the firmware from this
branch).

Quartus history:

| Build | LEs | Engine clock | Engine Fmax (slow 85 °C) |
|-------|-----|--------------|--------------------------|
| 1st | 13 053 / 15 408 (85 %), 25 M9K, 34 multiplier elements | 96 MHz system clock | ≈71 MHz: timing not met |
| 2nd | 13 115 | own clock, 64 MHz | 68.9 MHz: met, works on hardware |
| 3rd | — | 80 MHz | 76.3 MHz: −0.6 ns, all failing paths end in `hx` |
| 4th | — | 80 MHz | 70.4 MHz: −1.7 ns, reciprocal cache RAM → plane result registers, and CSG → pc |
| 5th | 13 498 (88 %) | 72 MHz | 75.0 MHz: met, works on hardware |
| 6th | — | 76.8 MHz | 80.8 MHz: met, works on hardware (cheat engine and audio DAC removed) |
| 7th | — | 80 MHz | not reported (pc_lag, plane point cache hit decided in S_PLB, sdot clamp without a compare) |
| 8th | 13 295 (86 %), 41 M9K | 76.8 MHz | 82.6 MHz: met, works on hardware (MSU-1 + audio DAC back, build option, default on) |
| 9th | 14 416 (94 %), 41 multiplier elements | 80 MHz | 73.5 MHz: −1.1 ns. Plane result → CSG in one state (and the CSG logic twice), plane dot product sign → 32 bit operand muxes; the crowding also cost the 96 MHz domain 0.25 ns |
| 10th | 13 884 (90 %), 47 multiplier elements | 76.8 MHz | 79.5 MHz: met (+0.45 ns; longest paths: multiplier sums → mb0, the sphere/shading operand mux). Both 9th-build paths removed: the plane result goes through S_COMB again, the dot product is issued in S_DEC (N·D on 3 more 16×16 lanes) so its sign is a register |
| 11th | 13 902 (90 %) | 76.8 MHz | engine 77.1 MHz: met (+0.05 ns); 96 MHz domain −0.19 ns: command buffer RAM → byte select → MCU data register (A4, as in the 9th build) |
| 12th | 13 876 (90 %) | 76.8 MHz | 96 MHz domain met; engine 69.4 MHz (−1.4 ns) after a placement change only (77.1 MHz the build before): plane "only the sign matters" compares (`dnn` → isect_miss → `hp_depth`) |
| 13th | 13 933 (90 %), 47 multiplier elements | 76.8 MHz | engine 82.3 MHz, 96 MHz domain 108.5 MHz: met with the high-performance settings; works on hardware (half resolution default, MSU-1) |
| 14th | 13 958 (91 %) | 80 MHz | engine 76.95 MHz: −0.50 ns, only a few paths: shading dot product sum → clamp → multiplier operands (S_FIN4, S_SKY1) |
| current | not yet built | 80 MHz | that sum is registered first (S_FIN3A, S_SKY0: +1 cycle, +0.17 % cycles) |

### MSU-1 build option

`main.qsf` defines `SRT_MSU1` (`set_global_assignment -name VERILOG_MACRO
"SRT_MSU1=1"`). That builds the audio DAC and the MSU-1 registers ($2000-$2007,
active when the ROM has a `.msu` file, as in the other cores) into the core.
The firmware serves MSU-1 from the SuperRT loop: it refills the MSU buffers at
least once per streamed scan line (the audio buffer lasts about 6 ms). With the
software renderer fallback (a core without the engine) audio would stutter.

Without MSU-1 (delete that line) the core has about 900 logic elements fewer.

Fmax moved by ±3 MHz between builds at 85–90 % device use. The 10th build
reached 79.5 MHz, so 80 MHz is not possible with this version. With Fmax above
about 81 MHz the engine could run at 80 MHz (`clk1_multiply_by = 10`,
`clk1_divide_by = 1` in `ip/mk3/pll.v`; `-multiply_by 10 -divide_by 1` in
`main.sdc`); if 76.8 MHz fails, 72 MHz (`9` / `1`).

In the 2nd build the longest paths were:

* the he < hx compare → CSG → he/hx;
* the sphere distance compare → missed-intersection → he/hx / Newton-Raphson
  input;
* the multiplier's second stage.

All three were restructured for the current version.

If `srt_engine_clk` fails timing, change the multiplier in two places:
`clk1_multiply_by` in `ip/mk3/pll.v` and `-multiply_by` of `srt_engine_clk`
in `main.sdc`. The input is 8 MHz, so 9 gives 72 MHz and 8 gives 64 MHz. A
value in between is 76.8 MHz (`clk1_multiply_by = 48`, `clk1_divide_by = 5`;
`-multiply_by 48 -divide_by 5` in `main.sdc`). As a last resort, set
`MUL_LAT` to 4 at the top of `srt_engine.v`. That gives the multipliers an
extra pipeline stage; it is bit-exact as well and costs about 15 % in cycles.

## Simulation

| Test | What it checks |
|------|----------------|
| `sim/engine/run_tests.sh <CommandBuffer.bin> [seeds]` | Verilator: the engine against `srt_render.c`, pixel by pixel. Covers the scene's command list with the 12 test cameras, and random command lists (every opcode, condition, CSG mode, jumps, undefined opcodes; a new list every frame). Random FIFO back-pressure. |
| `sim/run_sim.sh <SRTTest.sfc>` | the whole core (Verilator) replaying SNES bus and MCU traffic recorded by the end-to-end emulator (`superrt/emu`): the SNES program, register writes, NewFrame, then 2 frames rendered by the RTL engine and streamed over SPI. Every pixel is compared with the emulator's, along with register reads, framebuffer reads and PSRAM writes. |
| `SWPATH=1 sim/run_sim.sh <SRTTest.sfc>` | the same with the MCU software path (`A3`/`A4`), Icarus Verilog |
| `sim/engine/prof/prof.sh` | cycles per engine state, for profiling |

`sim/mksim.py` makes simulation copies of the sources in which registers start
at 0, as the FPGA flip-flops do. It also removes a blocking-assignment race in
`spi.v` that only exists in simulation.
