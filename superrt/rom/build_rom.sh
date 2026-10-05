#!/bin/sh
# build_rom.sh <superrt checkout> [workdir]
#
# Builds the SuperRT test ROM for the sd2snes "superrt" core:
#  1. libSFX at the commit used by SuperRT (+ its patch), incl. cc65 tools
#  2. palette, palette map, placeholder images and the MCU descriptor from the
#     scene command buffer (../datagen/srt_datagen.py replaces the Windows
#     testbed's "PAL Regen" / "Write data")
#  3. SRTTest.s with the sd2snes data banks appended (SRTTest-sd2snes.patch),
#     Select toggling full / half horizontal resolution (SRTTest-resmode.patch,
#     RESMODE=0 to leave it out); with MSU1=1 also MSU-1 background music
#     (SRTTest-msu1.patch)
# Output: <workdir>/SRT-SNES/Binaries/SRTTest.sfc
#
# Needs: git, make, gcc/g++, python3 with numpy and pillow
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
SUPERRT="$(cd "$1" && pwd)"
WORK="${2:-$HERE/work}"
[ -f "$SUPERRT/SRT-SNES/Source/SRTTest.s" ] || { echo "usage: $0 <superrt checkout> [workdir]"; exit 1; }
mkdir -p "$WORK"
cd "$WORK"

# 1. libSFX
if [ ! -x libSFX/tools/cc65/bin/ca65 ]; then
  rm -rf libSFX
  git clone https://github.com/Optiroc/libSFX.git libSFX
  (cd libSFX && git checkout 754993beb65540cae2c0f80f7debbb992a053e92 \
     && git submodule update --init --recursive \
     && git apply "$SUPERRT/SRT-SNES/External/libSFX/libSFX SuperRT changes.patch" \
     && make)
fi

# 2. project copy + data
rm -rf SRT-SNES
cp -r "$SUPERRT/SRT-SNES" SRT-SNES
rm -rf SRT-SNES/External/libSFX
ln -s "$WORK/libSFX" SRT-SNES/External/libSFX
(cd "$HERE/../render" && make batch_render)
python3 "$HERE/../datagen/srt_datagen.py" "$HERE/../render/batch_render" \
  SRT-SNES/Data/CommandBuffer.bin SRT-SNES/Data gen
cp gen/MainPal.bin gen/PaletteMap.bin gen/Placeholder.bin gen/Placeholder2.bin gen/SRTMcuData.bin SRT-SNES/Data/

# 3. ROM
(cd SRT-SNES && patch -p2 < "$HERE/SRTTest-sd2snes.patch")
# RESMODE=1 (default): half horizontal resolution by default, Select toggles
# full / half (sd2snes core register $BEBA). RESMODE=0: no resolution
# register writes (the core still defaults to half resolution).
if [ "${RESMODE:-1}" = 1 ]; then (cd SRT-SNES && patch -p2 < "$HERE/SRTTest-resmode.patch"); fi
# MSU1=1: the ROM starts MSU-1 track 1 (SRTTest-1.pcm) as looping background music
if [ "${MSU1:-0}" = 1 ]; then (cd SRT-SNES && patch -p2 < "$HERE/SRTTest-msu1.patch"); fi
(cd SRT-SNES && mkdir -p Binaries && make Binaries/SRTTest.sfc)
echo "ROM: $WORK/SRT-SNES/Binaries/SRTTest.sfc"
