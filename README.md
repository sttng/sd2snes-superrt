# SuperRT on sd2snes mk3

Runs ROMs for Ben Carter's [SuperRT](https://github.com/ShironekoBen/superrt)
real-time ray tracing chip on an sd2snes / FXPAK Pro mk3.

The original chip is three pipelined ray tracing engines on a Cyclone V. That
is about 4× more multipliers, logic and block RAM than the mk3's EP4CE15 has,
so the work is split:

* **FPGA** (`verilog/sd2snes_superrt`): the chip's SNES interface (register
  file, proxied writes, 512 × 64 bit command buffer, multiplier, frame
  handshake, framebuffer window into PSRAM) and **one sequential ray tracing
  engine** (`srt_engine.v`). The engine renders each frame into a pixel FIFO.
* **MCU** (`src/superrt.c`, `src/superrt_core.c`): streams the pixels from
  the FPGA, maps them to the ROM's 256 colour palette, and writes SNES tiles
  into the framebuffer bank the SNES isn't reading.
* `src/srt_render.c` is a bit-accurate C port of the chip. It is the
  reference for the engine, and the firmware falls back to it (a few seconds
  per frame) when the loaded core has no engine.

It is the same picture the original chip produces, just slower: about 2.3–7 fps
instead of about 20 at full resolution. By default the core renders at half
horizontal resolution (100 × 160, each pixel shown twice), about twice as fast:
≈5 fps for the demo's start view, ≈7 fps on average. Select toggles full / half
resolution in the test ROM. MSU-1 works alongside (core build option, default on).

## Status

| Part | Verified how |
|------|--------------|
| Renderer port (`src/srt_render.c`) | bit-identical to a Verilator simulation of the original RTL (`rtlref/`) on 12 camera/light set-ups (all 32000 pixels each) |
| FPGA engine (`srt_engine.v`) | bit-identical to `srt_render.c`: 12 cameras × 2 frames plus 60 frames of random command lists (all opcodes, conditions, CSG modes, jumps), with random FIFO back-pressure; also with the 4-stage multiplier option |
| Whole FPGA core | `verilog/sd2snes_superrt/sim`: emulator traffic replayed against the RTL (Verilator). The RTL engine renders 2 frames of the demo; all 64000 pixels are streamed over SPI and match. Register and framebuffer reads and PSRAM writes are checked too. The software path (A3/A4) passes with Icarus Verilog. |
| SNES ROM + register protocol + MCU loop | end-to-end emulation (`emu/`): SNES CPU (LakeSnes) running the test ROM, a C model of the core (engine + FIFO), and the real MCU code. The demo renders and responds to the joypad, identically on the engine path and the software path. |
| Firmware | builds for `config-mk3-stm32` and `config-mk3` (LPC1756) |
| MSU-1 with SuperRT | core: MSU-1 registers answer in the full core simulation (ID check); firmware: MSU-1 service split out of `msu1_loop` (unchanged behaviour) and called from the SuperRT loop. Fix: the MSU-1 service is started before the (slow) start-image setup and no longer resets the track/data position when the SNES already requested one (the first version answered the ROM's track request with an error, so no music). The MSU-1 test ROM retries the track request if it gets an error. Retry path tested in the emulator with an MSU stub. Verified further: core RTL simulation of the whole MSU-1 audio path (`run_sim.sh` op V: SNES request, MCU handshake, DAC output on the pins) and a firmware co-simulation (`superrt/emu/msu_cosim`: the real `msu1.c` against a model of the core + FatFs/SD offload, SNES emulator running the ROM) which plays `SRTTest-1.pcm` bit-exactly. Diagnostic ROMs: `superrt/msu/diag`. On hardware the music was still silent: SRTTest never loads a sound program, so the S-DSP stays in its reset state with MUTE set, which mutes the console amplifier and with it the cartridge audio (not modelled by emulators). The MSU-1 patch now uploads an 8 byte SPC700 program through the IPL that clears the mute flag. |
| Quartus build / real hardware | software renderer version: works on hardware. Engine version: works on hardware at 64, 72 and 76.8 MHz engine clock. 76.8 MHz with MSU-1: builds and runs on hardware. Fast lane multiplier + shorter plane test: 80 MHz build missed timing (73.5 MHz); restructured version at 76.8 MHz: engine regression, MUL_LAT=4 variant and full core simulation bit-exact; Quartus: timing met (Fmax 79.5 MHz), hardware test pending. |

**Speed:** about 11–34 M engine cycles per frame, depending on the view, at the
engine's 76.8 MHz clock. The demo's start view is 30.0 M cycles (≈2.56 fps), and
the 12 test views average ≈3.5 fps. The MCU's palette mapping and tile writing run while the
engine renders the rest of the frame. The LED is lit while a frame is in
progress.

## What's where

    verilog/sd2snes_superrt/   FPGA core with the ray tracing engine (+ sim/ testbenches)
    src/srt_render.[ch]        bit-accurate software model of the chip (reference + fallback)
    src/superrt_core.[ch]      palette mapping, SNES tile conversion, frame render (portable)
    src/superrt.[ch]           firmware main loop + hardware hooks
    src/smc.c, fpga.h, ...     detection (title "SUPERRT", LoROM) -> fpga_superrt.bi3
    superrt/render/            host builds of the renderer (host_render, batch_render, cmpfb.py)
    superrt/datagen/           srt_datagen.py: palette / palette map / placeholders / MCU data
    superrt/rom/               build_rom.sh + patch for the test ROM
    superrt/emu/               srtemu end-to-end emulator
    superrt/rtlref/            Verilator model of the original RTL (golden reference)

## Getting it running

1. **FPGA core**: `make mk3` in `verilog/sd2snes_superrt` (Quartus), copy
   `fpga_superrt.bi3` to `/sd2snes/` on the SD card.
2. **Firmware**: build `src` with `CONFIG=config-mk3-stm32` (or `config-mk3`)
   as usual; the SuperRT code is only compiled for mk3.
3. **ROM**: `superrt/rom/build_rom.sh <superrt checkout>` builds
   `SRTTest.sfc` (needs git, make, gcc, python3 + numpy + pillow). Nothing from
   the Windows-only C# testbed is needed: `srt_datagen.py` renders the 144
   palette views of the testbed's "PAL Regen" with the C renderer and ports
   its palette generator.

## ROM requirements

Any SuperRT ROM works if

* its header title starts with `SUPERRT` (LoROM), and
* file offset `0x8000` holds the `SRTMCU01` descriptor written by
  `srt_datagen.py`: the palette mapping as a k-d tree (the MCU has no room for
  the 32 KB palette map) and the offset of a 32000 byte start-up image
  (`0x10000` in the test ROM). The SNES never sees these banks because the
  SuperRT memory map ignores the bank byte.

`rom/SRTTest-sd2snes.patch` adds both to `SRTTest.s` (2 `incbin`s in the
otherwise empty ROM1/ROM2 banks).

## Not supported

* sd2snes in-game hooks / button combos / cheats / savestates (the core has
  no cheat engine; return to the menu with a long reset)
* mk2

## Licences

SuperRT is © 2021 Ben Carter, MIT licence (`LICENSE-SuperRT`); the
renderer and the core's register logic are derived from it. LakeSnes is only
used as an unmodified external checkout plus `emu/lakesnes.patch`.
