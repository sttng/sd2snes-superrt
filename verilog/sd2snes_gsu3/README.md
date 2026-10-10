# Super FX 3 (FX3) support for sd2snes

This adds Super FX 3 (FX3) support for example for THE ULTIMATE DOOM
(LoROM cart type `$17`/`$18`). The FX3 runs on a new FPGA core, `fpga_gsu3`, which uses
a pipelined Super FX implementation. Classic Super FX games (Star Fox, Yoshi's Island,
etc.) still use the unchanged `fpga_gsu` core.

## Features

| | Mk3 (Cyclone IV) | Mk2 (Spartan-3 XC3S400) |
|---|---|---|
| FX3 core clock | 85.75 MHz (4x rate - full) | 42.9 MHz (2x rate, clock enable - half) |
| Speed vs. FX2 | ~1.8–5.2× | ~1.3–2.7× |
| Cheat engine | yes | yes |
| MSU-1 | yes | no (no room) |
| In-game hooks / savestates | no (on FX3 carts) | no (on FX3 carts) |

## The FX3 core (`verilog/sd2snes_gsu/gsu_fx3.v`)

- **Pipelined:** fetches up to one instruction byte per clock, with zero-penalty
  branches and a 256-line write-back plot cache.
- **Memory map:**
  - registers at `$7000`, mirrored every `$400`
  - 4 MB linear ROM
  - cart RAM in banks `$70`–`$71`
  - no RON/RAN handover and no FX IRQ; the 65816 polls R15 instead
- **Bus:** the FX gets back-to-back memory slots in S-CPU cycles that don't use the
  bus, plus a faster FX ↔ memory handshake (`main.v`, `address.v`, under `ifdef GSU3`).

## Mk2 specifics

- `GSU3_HALF` (set in `main.v` for Mk2 + GSU3) runs the core with a clock enable on
  every other CLK2 edge.
- `verilog/sd2snes_gsu3/main.ucf` adds a 2-cycle constraint (`TS_gsu_half`, 23.3 ns)
  for paths inside `snes_gsu`. Make sure your ISE project uses this UCF, not
  `../sd2snes_gsu/main.ucf`.
- The cache and FMULT are inferred RAM/multipliers so they follow the clock enable.
- **Last ISE 14.7 result:**
  - all constraints met (`TS_gsu_half` 22.9/23.3 ns, `CLK2` 11.63/11.66 ns)
  - 3,582 of 3,584 slices
- If space runs out again, `GSU3_NOCHEAT` drops the cheat engine as a fallback.

## Firmware (`src/`)

- `smc.c`: detects FX3 carts (cart type `$17`/`$18`, map `$20`) and selects
  `FPGA_GSU3`; RAM size comes from the expansion RAM byte.
- `memory.c`: skips the MSU-1 check for FX3 on Mk2.
- `cheat.c`: NMI/IRQ hooks are disabled on FX3 carts.
- `fpga.h`: adds the `FPGA_GSU3` core; `smc.h` adds the `has_fx3` flag.

## Building

- The root `Makefile` builds `gsu3` for both Mk2 and Mk3.
- **Mk3:** Quartus project `verilog/sd2snes_gsu3/sd2snes_gsu3.qpf`.
- **Mk2:** ISE project `verilog/sd2snes_gsu3/sd2snes_gsu3.xise`. Sources live in
  `../sd2snes_gsu`; the UCF is the local `main.ucf`. `common.mk` picks the local UCF
  when it exists.
