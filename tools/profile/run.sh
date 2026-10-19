#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Jack Sidman Smith
#
# Builds tools/profile/profile.cpp with the firmware modules and runs it under simavr.
#   tools/profile/run.sh <simavr dir>
set -eu
sim=$1
here=$(cd "$(dirname "$0")" && pwd)
src=$here/../../src
dir=$(mktemp -d)
avr-g++ -mmcu=atmega2560 -DF_CPU=16000000UL -std=gnu++17 -Os -fno-exceptions -fno-rtti -fno-threadsafe-statics \
    -I"$src" -o "$dir/profile.elf" "$here/profile.cpp" "$src/kin.cpp" "$src/fmt.cpp" "$src/planner.cpp" \
    "$src/stepgen.cpp" "$src/stepper.cpp" "$src/board.cpp" -lm
LD_LIBRARY_PATH=$(echo "$sim"/simavr/obj-*) "$sim/simavr/run_avr" -m atmega2560 -f 16000000 "$dir/profile.elf" \
    2>/dev/null | sed 's/\x1b\[[0-9;]*m//g' | grep -v '^Loaded' | sed '/^END$/,$d'
rm -rf "$dir"
