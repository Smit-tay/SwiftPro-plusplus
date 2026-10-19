#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Jack Sidman Smith
#
# Prints dtostrf(v, 5, 4) for a list of float32 bit patterns (one hex value per line), one output per line.
#   tools/dtostrf/run.sh <simavr dir> <values.txt> [avr-gcc]
# simavr is built from source (github.com/buserror/simavr, "make build-simavr"); it is not packaged for Fedora.
set -eu
sim=$1 vals=$2 cc=${3:-avr-gcc}
dir=$(mktemp -d)
{ echo '#include <avr/pgmspace.h>'; echo 'static const uint32_t kVals[] PROGMEM = {'
  sed 's/^/0x/; s/$/UL,/' "$vals"; echo '};'; } > "$dir/table.h"
"$cc" -mmcu=atmega2560 -DF_CPU=16000000UL -Os -I"$dir" -o "$dir/dto.elf" "$(dirname "$0")/dtostrf_avr.c"
LD_LIBRARY_PATH=$(echo "$sim"/simavr/obj-*) "$sim/simavr/run_avr" -m atmega2560 -f 16000000 "$dir/dto.elf" 2>/dev/null \
  | sed 's/\x1b\[[0-9;]*m//g' | grep -v '^Loaded' | sed '/^END$/,$d'
rm -rf "$dir"
