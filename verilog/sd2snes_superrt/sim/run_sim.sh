#!/bin/sh
# run_sim.sh <SRTTest.sfc>
# Replays SNES/MCU traffic recorded by the end-to-end emulator
# (superrt/emu/srtemu) against the actual core RTL.
#
#   default    hardware engine path: the RTL engine renders FRAMES (2) frames,
#              every pixel streamed over SPI (A5/A6) is compared with the
#              emulator's. Needs Verilator >= 5 (--timing); a frame takes
#              about a minute.
#   FULLRES=1  press Select first (ROM built with SRTTest-resmode.patch): full
#              horizontal resolution instead of the default half
#   SWPATH=1   MCU software path (A3/A4 parameter / command list reads),
#              Icarus Verilog, FRAMES (4) frames.
set -e
cd "$(dirname "$0")"
ROM="$1"
[ -f "$ROM" ] || { echo "usage: $0 <SRTTest.sfc>"; exit 1; }
EMU=../../../superrt/emu/srtemu
[ -x $EMU ] || ../../../superrt/emu/build_emu.sh
SWPATH=${SWPATH:-0}
[ "${FULLRES:-0}" = 1 ] && EMU_SCRIPT="2:2:on,10:2:off"   # first frame half, then full
python3 -c "import sys; d=open(sys.argv[1],'rb').read(); open('rom.hex','w').write('\n'.join('%02x'%b for b in d[:0x20000])+'\n')" "$ROM"
if [ "$SWPATH" = 1 ]; then FR=${FRAMES:-4}; else FR=${FRAMES:-2}; fi
SRT_SWPATH=$SWPATH SRT_STIM=stim.txt SRT_STIM_FRAMES=$FR $EMU "$ROM" ${EMU_FRAMES:-60} emu_out 2 "${EMU_SCRIPT:-}" > /dev/null
printf "U\nV\n" >> stim.txt   # MSU-1 ID check + audio path (cores built with SRT_MSU1)
python3 mksim.py
# MSU=0: core built without MSU-1
if [ "${MSU:-1}" = 1 ]; then DEFS="-DMK3 -DSRT_MSU1"; else DEFS="-DMK3"; fi
SRCS="tb_superrt.v ip_stubs.v gen/address.v gen/clk_test.v gen/dac.v gen/msu.v gen/main.v gen/mcu_cmd.v
  gen/sd_dma.v gen/spi.v gen/superrt.v gen/srt_engine.v gen/srt_mul.v"
if [ "$SWPATH" = 1 ]; then
  iverilog -g2005 $DEFS -o tb $SRCS
  vvp -n tb +stim=stim.txt +rom=rom.hex | tail -4
else
  verilator --binary --timing -O2 -j 8 -Wno-fatal -Wno-WIDTH -Wno-lint -Wno-style $DEFS \
    --top-module tb -Mdir vl $SRCS > vl_build.log 2>&1 || { tail -20 vl_build.log; exit 1; }
  ./vl/Vtb +stim=stim.txt +rom=rom.hex | grep -E "streamed|errors|PASS|FAIL|pixel|got|MSU"
fi
