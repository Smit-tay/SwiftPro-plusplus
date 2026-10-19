#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Jack Sidman Smith
#
# Compares the original's kinematics with the port's on the ATmega2560 under simavr. The port is its own
# implementation, so values may differ in the last bits; this reports how many differ in bits and in their
# 4-decimal text (what the replies print), and the largest difference. It fails only if NaN-ness or legality differ.
#   tools/kin/run.sh <simavr dir> <original repo> [seeds...]
# The original (a temporary copy of src/uarm_coord_convert.c) is built with its own toolchain (PlatformIO avr-gcc 7.3,
# avr-libc 2.0.0), the port with avr-gcc / avr-libc from PATH. Each prints every result in hex.
set -eu
sim=$1 orig=$2; shift 2
seeds=${*:-1 2 3}
here=$(cd "$(dirname "$0")" && pwd)
pio=${PIO_AVR:-$HOME/.platformio/packages/toolchain-atmelavr/bin}
dir=$(mktemp -d)
cp "$here"/common.h "$here"/orig_main.c "$here"/port_main.cpp "$here"/uarm_coord_convert.h "$dir"/
cp "$orig/src/uarm_coord_convert.c" "$dir/orig_coord.c"
export LD_LIBRARY_PATH=$(echo "$sim"/simavr/obj-*)
run() { "$sim/simavr/run_avr" -m atmega2560 -f 16000000 "$1" 2>/dev/null | sed 's/\x1b\[[0-9;]*m//g' | grep -v '^Loaded' | sed '/^END$/,$d'; }
status=0
for seed in $seeds; do
    (cd "$dir" && python3 "$here/gen.py" "$seed")
    "$pio/avr-gcc" -mmcu=atmega2560 -DF_CPU=16000000UL -Os -I"$dir" -o "$dir/orig.elf" "$dir/orig_main.c" "$dir/orig_coord.c" -lm 2>/dev/null
    avr-g++ -mmcu=atmega2560 -DF_CPU=16000000UL -std=gnu++17 -Os -I"$dir" -I"$here/../../src" -o "$dir/port.elf" \
        "$dir/port_main.cpp" "$here/../../src/kin.cpp" -lm
    run "$dir/orig.elf" > "$dir/orig$seed.txt"
    run "$dir/port.elf" > "$dir/port$seed.txt"
    python3 "$here/compare.py" "$dir/orig$seed.txt" "$dir/port$seed.txt" "seed $seed" || status=1
done
rm -rf "$dir"
exit $status
