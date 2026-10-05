#!/bin/sh
# make_msu_track.sh <audio file> <rom.sfc> [track] [loop point in samples]
# Converts any audio file ffmpeg can read into an MSU-1 track next to the ROM:
#   <rom>-<track>.pcm   "MSU1", loop point (LE32), 44.1 kHz 16 bit stereo LE PCM
#   <rom>.msu           MSU-1 data file (created empty-ish if missing: the
#                       firmware only enables MSU-1 when it exists)
# Needs ffmpeg.
set -e
IN="$1"; ROM="$2"; TRACK="${3:-1}"; LOOP="${4:-0}"
[ -f "$IN" ] && [ -n "$ROM" ] || { echo "usage: $0 <audio file> <rom.sfc> [track] [loop sample]"; exit 1; }
BASE="${ROM%.*}"
OUT="$BASE-$TRACK.pcm"
TMP="$OUT.raw.$$"
ffmpeg -hide_banner -loglevel error -y -i "$IN" -vn -ac 2 -ar 44100 -f s16le -acodec pcm_s16le "$TMP"
python3 - "$TMP" "$OUT" "$LOOP" <<'PY'
import struct, sys
raw = open(sys.argv[1], 'rb').read()
open(sys.argv[2], 'wb').write(b'MSU1' + struct.pack('<I', int(sys.argv[3])) + raw)
print("%s: %.1f s" % (sys.argv[2], len(raw) / 4 / 44100.0))
PY
rm -f "$TMP"
[ -f "$BASE.msu" ] || head -c 512 /dev/zero > "$BASE.msu"
