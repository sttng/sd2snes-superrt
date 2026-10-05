#!/bin/sh
# builds obj_dir/Vtb_engine_top (Verilator) with src/srt_render.c as reference
set -e
cd "$(dirname "$0")"
R=../../../../src
verilator --cc --exe --build -O3 -j 4 --top-module tb_engine_top -Wno-fatal -Wno-WIDTH +define+SRT_SIM_CHECKS \
  -CFLAGS "-O2 -I$(cd $R && pwd) -DSRT_HOST_BUILD" \
  tb_engine_top.v ../../srt_engine.v ../../srt_mul.v tb_engine.cpp $(cd $R && pwd)/srt_render.c
