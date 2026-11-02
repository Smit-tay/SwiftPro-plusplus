#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Jack Sidman Smith
#
# Confirms the host sweep (run.sh, run it first) on the AVR: the originals' legality for 5,340 poses (every L = 0
# point, a 5 deg subgrid, and every point of the "mirror" corner L, R <= 1), built with the original's toolchain (PlatformIO avr-gcc 7.3,
# as tools/kin) and run under simavr, compared point by point with build/limits/sweep-<version>.txt.
#   tools/limits_sweep/avr_check.sh <simavr dir> [upstream dir] [jss.7 repo]
set -eu
sim=$1
up=${2:-$HOME/dev/third-party/uarm}
jss=${3:-$HOME/dev/archive/SwiftProForArduino}
here=$(cd "$(dirname "$0")" && pwd)
out=$here/../../build/limits
pio=${PIO_AVR:-$HOME/.platformio/packages/toolchain-atmelavr/bin}
dir=$(mktemp -d)
trap 'rm -rf "$dir"' EXIT
export LD_LIBRARY_PATH=$(echo "$sim"/simavr/obj-*)
python3 -I - "$dir/points.h" <<'PY'
import sys
pts = set()
for r in range(-60, 261): pts.add((0, r))                                    # L = 0, every R
for l in range(-60, 291, 10):
    for r in range(-60, 261, 10): pts.add((l, r))                            # 5 deg subgrid
for l in range(-60, 3):
    for r in range(-60, 3): pts.add((l, r))                                  # the mirror corner, every point
with open(sys.argv[1], "w") as f:
    f.write("static const int16_t kPoints[][2] PROGMEM = {\n")
    f.write(",\n".join("{%d,%d}" % p for p in sorted(pts)))
    f.write("};\n")
PY
check() {  # $1 = name, $2 = .c, $3 = RAD_TO_DEG, $4 = optional constants header
    local d=$dir/$1
    mkdir -p "$d"
    tr -d '\r' < "$2" > "$d/coord.c"
    { echo '#include <math.h>'; echo '#include <stdbool.h>'; echo '#include <stdint.h>'
      echo 'struct uarm_t { float init_arml_angle, init_armr_angle, init_base_angle; struct { float front_offset, high_offset; } param; };'
      echo 'extern struct uarm_t uarm;'; echo "#define RAD_TO_DEG $3"
      echo 'bool is_angle_legal(float, float, float);'
      if [[ -n ${4:-} ]]; then tr -d '\r' < "$4" | awk '/^#if[ \t]+defined\(UARM_MINI\)/,/^#endif/'; fi; } > "$d/uarm_coord_convert.h"
    "$pio/avr-gcc" -mmcu=atmega2560 -DF_CPU=16000000UL -Os -w -I"$d" -I"$dir" -o "$d/p.elf" "$here/avr_points.c" "$d/coord.c" -lm
    "$sim/simavr/run_avr" -m atmega2560 -f 16000000 "$d/p.elf" 2>/dev/null | sed 's/\x1b\[[0-9;]*m//g' | grep -v '^Loaded' \
        | sed '/^END$/,$d' > "$d/avr.txt"
    python3 -I - "$d/avr.txt" "$out/sweep-$1.txt" "$1" <<'PY'
import sys
avr = {}
for line in open(sys.argv[1]):
    hl, hr, d, t = line.split(); avr[(int(hl) / 2, int(hr) / 2)] = (d, t)
host = {}
for line in open(sys.argv[2]):
    p = line.split()
    if p[0] == "G": host[(float(p[1]), float(p[2]))] = (p[4], p[5])
hostline = {}
for line in open(sys.argv[2]):
    p = line.split()
    if p[0] == "G": hostline[(float(p[1]), float(p[2]))] = [float(v) for v in p[6:9]]
lim = (-20.0, 135.6, -20.0, 119.9) if sys.argv[3] == "jss.7" else (-4.0, 135.6, -4.0, 119.9)
def on_limit(k):  # the round-trip angles (host) lie within 0.01 deg of a limit
    l2, r2, _ = hostline[k]; e = 180 - l2 - r2
    return any(abs(a - b) <= 0.01 for a, b in ((l2, lim[0]), (l2, lim[1]), (r2, lim[2]), (r2, lim[3]), (e, 10.0), (e, 151.0)))
diff = [k for k in sorted(avr) if avr[k] != host[k]]
at_zero = [k for k in diff if k[0] == 0.0 or k[1] == 0.0]
limit = [k for k in diff if k not in at_zero and on_limit(k)]
other = [k for k in diff if k not in at_zero and k not in limit]
row0 = [k for k in avr if (k[0] == 0.0 or k[1] == 0.0) and avr[k][0] == "1"]
mirror = [k for k in avr if avr[k] == ("0", "1")]
print(f"{sys.argv[3]:8} {len(avr)} poses on the AVR, {len(diff)} differ from the host: {len(at_zero)} at L = 0 or R = 0 (acos at the edge of its domain), "
      f"{len(limit)} on a limit, {len(other)} elsewhere")
for k in other: print(f"           elsewhere: L{k[0]:.1f} R{k[1]:.1f}: AVR {avr[k]}, host {host[k]}")
print(f"           AVR: legal poses at L = 0 or R = 0 refused by the round trip: {sum(avr[k][1] == '0' for k in row0)} of {len(row0)}; "
      f"illegal poses accepted by the round trip (mirror): {len(mirror)}")
PY
}
check V4.9.0 "$up/uarm-V4.9.0/src/uarm_coord_convert.c" "(57.29578)"
check V4.10.0 "$up/uarm-V4.10.0/src/uarm_coord_convert.c" "(57.29578)" "$up/uarm-V4.10.0/src/uarm_coord_convert.h"
check jss.7 "$jss/src/uarm_coord_convert.c" "57.295779513082320876798154814105"
