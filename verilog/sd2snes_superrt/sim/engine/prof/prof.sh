#!/bin/sh
# Engine profile: cycles spent per state (Verilator, --public-flat-rw build).
#   prof.sh <cmdbuf.bin> <params>
set -e
A="$(cd "$(dirname "$1")" && pwd)/$(basename "$1")"
B="$(cd "$(dirname "$2")" && pwd)/$(basename "$2")"
cd "$(dirname "$0")"
R="$(cd ../../../../../src && pwd)"
E="$(cd ../../.. && pwd)"
verilator --cc --exe --build -O3 -j 4 --public-flat-rw --top-module tb_engine_top -Wno-fatal -Wno-WIDTH \
  -CFLAGS "-O2 -I$R" ../tb_engine_top.v "$E/srt_engine.v" "$E/srt_mul.v" tbp.cpp "$R/srt_render.c" > /dev/null
./obj_dir/Vtb_engine_top -nostall "$A" "$B" > prof.txt
E="$E" python3 - <<'PY'
import re, os
src = open(os.environ['E'] + '/srt_engine.v').read()
names = {int(v): n for n, v in re.findall(r"(S_\w+) = 7'd(\d+)", src)}
tot = 0; rows = []
for l in open('prof.txt'):
    m = re.match(r'state\s+(\d+):\s+(\d+)', l)
    if m:
        rows.append((int(m.group(2)), names.get(int(m.group(1)), '?'))); tot += int(m.group(2))
    elif 'frame' in l or 'nr ret' in l:
        print(l.strip())
for c, n in sorted(rows, key=lambda x: -x[0]):
    print("%-8s %10d %5.1f%%" % (n, c, 100.0 * c / tot))
print("total", tot)
PY
