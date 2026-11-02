#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Jack Sidman Smith
#
# COMMANDABLE limits (docs/LIMITS.md): sweeps L and R on a 0.5 deg grid (base fixed at 0 in the -90 frame), and the
# base at the probe pose, through the legality checks of each firmware, on the host, and compares the regions.
#   tools/limits_sweep/run.sh [upstream dir] [jss.7 repo]
#     upstream dir: holds uarm-V4.9.0 and uarm-V4.10.0 (default ~/dev/third-party/uarm)
#     jss.7 repo:   Smit-tay/SwiftProForArduino at 72a2ccf (default ~/dev/archive/SwiftProForArduino)
# Output: a summary on stdout and every point in build/limits/ (sweep-<version>.txt, region.csv).
#
# Each original is a copy of its own src/uarm_coord_convert.c, compiled as C with a shim header that gives it what the
# AVR build had: 32-bit double (every double and libm call becomes float, as avr-gcc's double is float), the
# version's RAD_TO_DEG, and for V4.10.0 its constants (they moved to the header there). Host libm differs from
# avr-libc in the last bits, so a point that lies on a limit to within float noise may differ from the AVR; the
# summary lists those separately.
set -eu
up=${1:-$HOME/dev/third-party/uarm}
jss=${2:-$HOME/dev/archive/SwiftProForArduino}
here=$(cd "$(dirname "$0")" && pwd)
root=$here/../..
out=$root/build/limits
mkdir -p "$out"
dir=$(mktemp -d)
trap 'rm -rf "$dir"' EXIT

shim() {  # $1 = RAD_TO_DEG, $2 = optional header to take the constants block from
    cat <<EOF
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#define double float
#define sqrt sqrtf
#define acos acosf
#define atan atanf
#define sin sinf
#define cos cosf
#define fabs fabsf
#define RAD_TO_DEG $1
struct uarm_t { float init_arml_angle, init_armr_angle, init_base_angle; struct { float front_offset, high_offset; } param; };
extern struct uarm_t uarm;
bool is_angle_legal(float, float, float);
void angle_to_coord(float, float, float, float*, float*, float*);
void coord_to_angle(float, float, float, float*, float*, float*);
void coord_arm2effect(float*, float*, float*);
void coord_effect2arm(float*, float*, float*);
EOF
    if [[ -n ${2:-} ]]; then tr -d '\r' < "$2" | awk '/^#if[ \t]+defined\(UARM_MINI\)/,/^#endif/'; fi
}

build_orig() {  # $1 = name, $2 = .c, $3 = RAD_TO_DEG, $4 = optional constants header
    local d=$dir/$1
    mkdir -p "$d"
    tr -d '\r' < "$2" > "$d/coord.c"
    shim "$3" "${4:-}" > "$d/uarm_coord_convert.h"
    cp "$here/orig_sweep.c" "$d/"
    cc -std=gnu99 -O1 -fsingle-precision-constant -w -I"$d" -o "$d/sweep" "$d/orig_sweep.c" "$d/coord.c" -lm
    "$d/sweep" > "$out/sweep-$1.txt"
}

# V4.9.0 and V4.10.0: the header's "#define RAD_TO_DEG (57.29578)" follows Arduino.h and overrides it
# (uarm_coord_convert.h:6). jss.7 guards it with #ifndef, so Arduino's value stays (Arduino.h:51).
build_orig V4.9.0 "$up/uarm-V4.9.0/src/uarm_coord_convert.c" "(57.29578)"
build_orig V4.10.0 "$up/uarm-V4.10.0/src/uarm_coord_convert.c" "(57.29578)" "$up/uarm-V4.10.0/src/uarm_coord_convert.h"
build_orig jss.7 "$jss/src/uarm_coord_convert.c" "57.295779513082320876798154814105"

c++ -std=c++17 -O1 -Wall -Wextra -I"$root/src" -o "$dir/port" "$here/port_sweep.cpp" "$root/src/kin.cpp" -lm
"$dir/port" > "$out/sweep-pp.txt"

python3 -I "$here/compare.py" "$out"
