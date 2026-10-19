#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Jack Sidman Smith
#
# Interactive bring-up runner: sends a command file to the arm, waits for each command's reply, and stops for you
# where the file asks.
#
#   tools/bringup/run.sh <command file> [port]        (port default /dev/ttyACM0)
#
# Opening the port resets the arm (USB DTR), so the firmware boots: the script waits for "@1" before it starts.
# Everything the arm sends is shown and logged with a timestamp to <command file>-<time>.log, next to the file.
#
# Command file syntax:
#   #N <command>     send it, then wait (up to 60 s) for its reply "$N ..."; a move replies when the arm has stopped
#   & #N <command>   send it and go on at once (to interrupt a move: hold, stop)
#   //wait <s>       pause
#   //ask <text>     show the text and wait for Enter (Ctrl-C ends the run)
#   //<text>         a heading, shown and logged
#   blank lines are ignored
set -u
file=${1:?usage: run.sh <command file> [port]}
port=${2:-/dev/ttyACM0}
log=$(dirname "$file")/$(basename "$file" .txt)-$(date +%Y%m%d-%H%M%S).log
raw=$(mktemp)

stty -F "$port" 115200 raw -echo || exit 1
exec 3<>"$port"

# Reader: every line from the arm, timestamped, to the screen and the log; raw copy for reply matching.
(
    while IFS= read -r line <&3; do
        line=${line%$'\r'}
        printf '%s\n' "$line" >> "$raw"
        printf '%s  %s\n' "$(date +%T.%3N)" "$line" | tee -a "$log"
    done
) &
reader=$!
trap 'kill $reader 2>/dev/null; rm -f "$raw"; exec 3>&-' EXIT

note() { printf '%s  %s\n' "$(date +%T.%3N)" "$1" | tee -a "$log"; }

# Waits until a line matching the regex appears after line number $2 of the raw log; false on timeout ($3 seconds).
wait_for() {
    local re=$1 from=$2 limit=$3 t=0
    while (( t < limit * 20 )); do
        tail -n +"$((from + 1))" "$raw" | grep -qE "$re" && return 0
        sleep 0.05
        t=$((t + 1))
    done
    return 1
}

note "== waiting for the arm to boot (@1)"
if ! wait_for '^@1$' 0 15; then
    note "== no @1 within 15 s: check the port and the baud rate"
    exit 1
fi
sleep 0.6  # past the boot beep, as the probe

while IFS= read -r line || [[ -n $line ]]; do
    line=${line%$'\r'}
    case $line in
        '') continue ;;
        '//wait '*) sleep "${line#//wait }" ;;
        '//ask '*)
            note "?? ${line#//ask }"
            printf '   press Enter to go on > ' >&2
            read -r < /dev/tty
            note "-- (Enter)" ;;
        //*) note "== ${line#//}" ;;
        '& '*)
            cmd=${line#& }
            note ">> $cmd   (not waiting)"
            printf '%s\n' "$cmd" >&3 ;;
        *)
            seq=${line%% *}
            seq=${seq#\#}
            from=$(wc -l < "$raw")
            note ">> $line"
            printf '%s\n' "$line" >&3
            if ! wait_for "^\\\$$seq( |$)" "$from" 60; then
                note "== no reply to #$seq within 60 s"
            fi ;;
    esac
done < "$file"
sleep 0.5
note "== done; log in $log"
