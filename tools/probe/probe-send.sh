#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Jack Sidman Smith
#
# Send a command file to the arm line by line and log everything it replies.
#
# With picocom (picocom runs this with stdin/stdout on the serial port):
#   picocom -b 115200 --send-cmd "$PWD/probe-send.sh" /dev/ttyACM0
#   wait for "@1", then press C-a C-s and enter the path to probe.txt
#
# Without picocom:
#   stty -F /dev/ttyACM0 115200 raw -echo
#   ./probe-send.sh probe.txt </dev/ttyACM0 >/dev/ttyACM0
#
# Command file syntax:
#   //text      section heading (not sent)
#   //wait N    pause N seconds
#   //burst     send the next two lines back to back, without the pause
#   anything else is sent as one line, followed by PROBE_DELAY seconds of pause
#
# Environment: PROBE_DELAY (default 0.4 s), PROBE_LOG (default probe-<time>.log)

set -u
file=${1:?usage: probe-send.sh <command file>}
delay=${PROBE_DELAY:-0.4}
log=${PROBE_LOG:-probe-$(date +%Y%m%d-%H%M%S).log}

# Copy everything the arm sends to the log and to the terminal.
exec 3<&0
tee -a "$log" <&3 >&2 &
reader=$!

note() { printf '%s\n' "$1" | tee -a "$log" >&2; }

burst=0
while IFS= read -r line || [[ -n $line ]]; do
    line=${line%$'\r'}
    case $line in
        '')         continue ;;
        '//wait '*) sleep "${line#//wait }"; continue ;;
        '//burst')  burst=2; continue ;;
        '//'*)      note ""; note "== ${line#//}"; continue ;;
    esac
    note ">> $line"
    printf '%s\n' "$line"
    if (( burst > 0 )); then
        burst=$((burst - 1))
        (( burst > 0 )) && continue
    fi
    sleep "$delay"
done <"$file"

sleep 2
kill "$reader" 2>/dev/null
note ""
note "== done; log written to $log"
