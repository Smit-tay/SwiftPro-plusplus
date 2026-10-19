#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Jack Sidman Smith
#
# Writes compile_commands.json for clangd: src/ as AVR code, test/ as host code.
# Run from the repo root: python3 tools/compdb.py. The file holds absolute paths, so it is not committed.

import glob
import json
import os

AVR = ("clang++ --target=avr -mmcu=atmega2560 -isystem /usr/avr/include -DF_CPU=16000000UL -std=gnu++17 -Os "
       "-fno-exceptions -fno-rtti -Wall -Wextra -Wconversion -Wdouble-promotion")
HOST = "g++ -std=c++17 -Wall -Wextra -Wconversion -Wdouble-promotion -Itest"

root = os.getcwd()
avr_files = sorted(glob.glob("src/*.cpp"))
host_files = [f for f in sorted(glob.glob("test/*.cpp")) if f not in avr_files]
db = [{"directory": root, "file": f, "command": f"{AVR} -c {f}"} for f in avr_files]
db += [{"directory": root, "file": f, "command": f"{HOST} -c {f}"} for f in host_files]
with open("compile_commands.json", "w") as out:
    json.dump(db, out, indent=1)
