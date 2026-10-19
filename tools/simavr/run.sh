#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Jack Sidman Smith
#
# Builds the firmware (CMake, build/avr) and the simavr harness, then runs a scripted session against the real image.
#   tools/simavr/run.sh <simavr source dir> [script]      (default script: tools/simavr/session.txt)
# The simavr tree must be built (make -C <dir>/simavr). Host-side only.
set -eu
sim=$1
here=$(cd "$(dirname "$0")" && pwd)
root=$here/../..
script=${2:-$here/session.txt}
cmake -S "$root" -B "$root/build/avr" --toolchain "$root/cmake/avr-gcc.cmake" > /dev/null
cmake --build "$root/build/avr" > /dev/null
elf=$root/build/avr/swiftpro-pp.elf
heap=$(avr-nm "$elf" | awk '$3 == "__bss_end" { print $1 }')
obj=$(echo "$sim"/simavr/obj-*)
mkdir -p "$root/build/simavr"
g++ -std=c++17 -O2 -Wall -Wextra -I"$sim/simavr/sim" -I"$sim/simavr" -o "$root/build/simavr/harness" \
    "$here/harness.cpp" "$obj/libsimavr.a" -lelf
"$root/build/simavr/harness" "$elf" "$heap" "$script"
