# SuperRT on sd2snes mk3

Runs ROMs for Ben Carter's [SuperRT](https://github.com/ShironekoBen/superrt)
ray tracing chip on an sd2snes / FXPAK Pro mk3 (EP4CE15).

The original chip has three pipelined ray engines on a Cyclone V, about 4× what
the EP4CE15 holds, so the work is split:

* **FPGA** (`verilog/sd2snes_superrt`): the chip's SNES interface (registers,
  command buffer, multiplier, frame handshake, framebuffer window) and one
  sequential ray tracing engine (`srt_engine.v`) that renders into a pixel FIFO.
  MSU-1 is a build option (default on).
* **MCU** (`src/superrt*.c`): streams the pixels, maps them to the ROM's 256
  colour palette, writes SNES tiles into the bank the SNES isn't showing, and
  serves MSU-1.
* `src/srt_render.c`: bit-accurate C model of the chip. Reference for the engine
  and software fallback for cores without the engine (seconds per frame).

The picture is identical to the original chip's, just slower. Engine clock
76.8 MHz, demo start view / average of 12 test views:

| Mode | Demo view | Test views |
|------|-----------|------------|
| Half horizontal resolution (default, 100 × 160, pixels doubled) | ≈5.1 fps | ≈7 fps |
| Full resolution (200 × 160, Select toggles) | ≈2.6 fps | ≈3.5 fps |

The original chip does about 20 fps.

## Status

* **Works on hardware** (13th Quartus build, engine Fmax 82.3 MHz at
  76.8 MHz): half / full resolution, fast lane multiplier, MSU-1 music.
* **Verified in simulation:** `srt_render.c` is bit-identical to the original
  RTL (`rtlref/`). The engine is bit-identical to `srt_render.c` in both
  resolutions (12 cameras, demo view, random command lists, 4-stage multiplier
  variant). The whole core passes replayed SNES/MCU traffic, including the
  MSU-1 audio path down to the DAC pins (`verilog/sd2snes_superrt/sim`).

Details and the Quartus build history: `verilog/sd2snes_superrt/README.md`.

## Getting it running

1. **FPGA core:** build `verilog/sd2snes_superrt` in Quartus, copy
   `fpga_superrt.bi3` to `/sd2snes/`.
2. **Firmware:** build `src` with `CONFIG=config-mk3-stm32` (or `config-mk3`).
3. **ROM:** `superrt/rom/build_rom.sh <superrt checkout>` builds `SRTTest.sfc`
   (needs git, make, gcc, python3 with numpy and pillow; the Windows-only C#
   testbed is not needed). `MSU1=1` adds background music; make the track with
   `superrt/msu/make_msu_track.sh` and keep `SRTTest.sfc`, `SRTTest.msu` and
   `SRTTest-1.pcm` in one folder.

Core, firmware and ROM go together: the half resolution mode needs all three
from the same version.

## ROM requirements

* Header title starting with `SUPERRT` (LoROM).
* At file offset `0x8000` the `SRTMCU01` descriptor from `srt_datagen.py`
  (palette mapping k-d tree + offset of a 32000 byte start-up image).
  `rom/SRTTest-sd2snes.patch` adds both to the test ROM.
* Optional: register `$BEBA` bit 0 selects full resolution (default half).
* For MSU-1 audio on real hardware the ROM must unmute the S-DSP (the test ROM
  uploads a tiny SPC700 program).

## What's where

    verilog/sd2snes_superrt/   FPGA core + engine, sim/ testbenches
    src/srt_render.[ch]        bit-accurate model of the chip
    src/superrt*.[ch]          firmware: palette mapping, tiles, main loop
    superrt/rom/               test ROM build + patches (sd2snes, resolution, MSU-1)
    superrt/msu/               MSU-1 track tool, diagnostic ROMs
    superrt/datagen/           palette / start image / descriptor generator

## Not supported

* sd2snes in-game hooks, cheats, savestates (long reset returns to the menu)
* mk2

## Licences

SuperRT is © 2021 Ben Carter, MIT licence (`LICENSE-SuperRT`); the renderer and
the core's register logic are derived from it. LakeSnes is used as an external
checkout plus `emu/lakesnes.patch`.
