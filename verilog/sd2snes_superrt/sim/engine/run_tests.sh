#!/bin/sh
# Engine regression: srt_engine.v vs src/srt_render.c, pixel by pixel.
#   run_tests.sh <CommandBuffer.bin> [random seeds]
# 1. the scene's command list with the 12 test cameras (2 frames each), and
#    the demo's first view (superrt/tests/demo_start.*)
# 2. random command lists (all opcodes / conditions / CSG modes, jumps,
#    undefined opcodes), a new list every frame
# 3. the same in half horizontal resolution (-h) for the cameras, the demo
#    view and every other seed
set -e
cd "$(dirname "$0")"
[ -x obj_dir/Vtb_engine_top ] || ./build.sh
T=../../../../superrt/tests
CB="$1"; SEEDS="${2:-20}"
fail=0
./obj_dir/Vtb_engine_top -n 2 "$CB" $T/cam*.params | tail -1 || fail=1
./obj_dir/Vtb_engine_top $T/demo_start.cmdbuf $T/demo_start.params | tail -1 || fail=1
./obj_dir/Vtb_engine_top -h -n 1 "$CB" $T/cam*.params | tail -1 || fail=1
./obj_dir/Vtb_engine_top -h $T/demo_start.cmdbuf $T/demo_start.params | tail -1 || fail=1
s=1
while [ $s -le $SEEDS ]; do
  ./obj_dir/Vtb_engine_top -r $s -n 2 - $T/cam$(( s % 12 + 1 )).params | tail -1 || fail=1
  [ $(( s % 2 )) = 0 ] && { ./obj_dir/Vtb_engine_top -h -r $s -n 2 - $T/cam$(( s % 12 + 1 )).params | tail -1 || fail=1; }
  s=$((s + 1))
done
[ $fail = 0 ] && echo "ALL PASS" || { echo "FAILURES"; exit 1; }
