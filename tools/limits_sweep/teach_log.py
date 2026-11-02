#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Jack Sidman Smith
#
# TEACHABLE limits (docs/LIMITS.md, procedure in docs/HARDWARE-TEST.md): releases the arm (M2019) and logs P2200
# (B, L, R from the encoders) and P2241 N3 (the wrist servo's feedback on ADC3, raw; UNVERIFIED wiring) at 20 Hz
# while you move the arm by hand, phase by phase. It never commands a move. At the end it asks before locking the
# joints again (M17, which re-reads the encoders and doesn't move).
#
#   python3 -I tools/limits_sweep/teach_log.py [port]          (default /dev/ttyACM0; needs pyserial)
#   python3 -I tools/limits_sweep/teach_log.py [port] --only <phase>[,<phase>...]  (just those phases, in order)
#   python3 -I tools/limits_sweep/teach_log.py [port] --outline  (only the 4-step loop around the L/R region)
#   python3 -I tools/limits_sweep/teach_log.py --summary <csv>  (the summary again, from a saved log)
#
# Output: teach-<time>.csv in the current directory: t (s), phase, B, L, R, elbow (180 - L - R), wrist ADC.
# "Stop" phases record the pose you hold when you press Enter (the median of the last 0.5 s) as that stop.
import csv
import datetime
import re
import statistics
import sys
import threading
import time

PHASES = [  # (name, instructions); a name starting with "stop" records the held pose when you press Enter
    ("sweep-LR", "Move L and R together through everything they can reach: along the folded and the stretched "
                 "edges, up and down, slowly. The arm-to-arm angle couples them, so cover the corners"),
    ("sweep-B", "Turn the base from one end to the other, slowly, at a middle L/R pose"),
    ("sweep-wrist", "Turn the wrist (end effector servo) from one end to the other by hand, gently"),
    ("stop-L-min", "Hold L at its lowest stop (lower arm down as far as it goes, R mid-range)"),
    ("stop-L-max", "Hold L at its highest stop (lower arm up/back as far as it goes, R mid-range)"),
    ("stop-R-min", "Hold R at its stop with the upper arm raised as far as it goes (L mid-range)"),
    ("stop-R-max", "Hold R at its stop with the upper arm lowered as far as it goes (L mid-range)"),
    ("stop-elbow-min", "Hold the arm folded: the two arms as close together as they go (smallest arm-to-arm angle)"),
    ("stop-elbow-max", "Hold the arm stretched: the two arms as far apart as they go (largest arm-to-arm angle)"),
    ("stop-B-min", "Hold the base at one end"),
    ("stop-B-max", "Hold the base at the other end"),
]
OUTLINE = [  # --outline: the (L, R) region's edge as one loop, in shoulder/elbow words
    ("outline-folded", "Close the elbow as far as it goes. Keep it closed and move the shoulder slowly from one "
                       "end of its travel to the other"),
    ("outline-end-1", "Keep the shoulder at that end. Slowly open the elbow all the way"),
    ("outline-stretched", "Keep the elbow fully open and move the shoulder slowly back to the other end"),
    ("outline-end-2", "Keep the shoulder at that end. Slowly close the elbow all the way"),
]
REPLY = re.compile(r"^\$(\d+) ok B(-?[\d.]+) L(-?[\d.]+) R(-?[\d.]+)$")
ADC = re.compile(r"^\$(\d+) ok V(\d+)$")


def summary(rows):
    def ext(name, key):
        vals = [r[key] for r in rows if r[key] is not None and (name is None or r["phase"] == name)]
        return (min(vals), max(vals)) if vals else None
    print("\n== Per joint, every phase (min .. max)")
    for key in ("B", "L", "R", "elbow", "wrist_adc"):
        e = ext(None, key)
        print(f"{key:9} {e[0]:9.2f} .. {e[1]:9.2f}" if e else f"{key:9} none")
    print("\n== Held stops (median of the last 0.5 s before Enter)")
    for r in rows:
        if r["phase"].startswith("stop-") and r.get("held"):
            print(f"{r['phase']:16} B{r['B']:8.2f} L{r['L']:8.2f} R{r['R']:8.2f} elbow{r['elbow']:8.2f} wrist {r['wrist_adc']}")
    print("\n== (L, R) region from sweep-LR and the outline: R range per 5 deg of L (min .. max)")
    lr = [r for r in rows if (r["phase"] == "sweep-LR" or r["phase"].startswith("outline-")) and r["L"] is not None]
    for lo in range(-30, 145, 5):
        rs = [r["R"] for r in lr if lo <= r["L"] < lo + 5]
        if rs:
            print(f"L {lo:4}..{lo + 5:<4} R {min(rs):8.2f} .. {max(rs):8.2f}   ({len(rs)} samples)")


def load(path):
    rows = []
    with open(path) as f:
        for r in csv.DictReader(f):
            row = {"t": float(r["t"]), "phase": r["phase"], "held": r.get("held") == "1"}
            for k in ("B", "L", "R", "elbow", "wrist_adc"):
                row[k] = float(r[k]) if r[k] not in ("", None) else None
            rows.append(row)
    return rows


def main():
    if len(sys.argv) == 3 and sys.argv[1] == "--summary":
        summary(load(sys.argv[2]))
        return
    import serial  # pyserial
    args = sys.argv[1:]
    phases = PHASES
    if "--only" in args:
        i = args.index("--only")
        names = args[i + 1].split(",") if i + 1 < len(args) else []
        known = dict(PHASES)
        if not names or any(n not in known for n in names):
            sys.exit("--only takes phase names from: " + ", ".join(known))
        phases = [(n, known[n]) for n in names]
        del args[i:i + 2]
    if "--outline" in args:
        phases = OUTLINE
        args.remove("--outline")
    port = args[0] if args else "/dev/ttyACM0"
    s = serial.Serial(port, 115200, timeout=0.05)
    print("waiting for the arm to boot (@1) ...")
    t0 = time.time()
    while time.time() - t0 < 15:
        if s.readline().decode(errors="replace").strip() == "@1":
            break
    else:
        sys.exit("no @1 within 15 s")
    time.sleep(0.6)
    s.reset_input_buffer()
    input("SUPPORT THE ARM with one hand: M2019 releases every joint and it will drop. Press Enter to release > ")
    s.write(b"#1 M2019\n")
    time.sleep(0.3)
    rows, lock = [], threading.Lock()
    state = {"phase": "", "run": True, "seq": 10}
    start = time.time()

    def poll():
        pending = {}
        while state["run"]:
            tick = time.time()
            seq = state["seq"]
            state["seq"] += 2
            s.write(f"#{seq} P2200\n#{seq + 1} P2241 N3\n".encode())
            row = {"t": tick - start, "phase": state["phase"], "B": None, "L": None, "R": None, "elbow": None,
                   "wrist_adc": None, "held": False}
            pending[seq], pending[seq + 1] = row, row
            while time.time() - tick < 0.05:
                line = s.readline().decode(errors="replace").strip()
                m = REPLY.match(line)
                if m and int(m.group(1)) in pending:
                    r = pending.pop(int(m.group(1)))
                    r["B"], r["L"], r["R"] = (float(m.group(i)) for i in (2, 3, 4))
                    r["elbow"] = 180.0 - r["L"] - r["R"]
                    continue
                m = ADC.match(line)
                if m and int(m.group(1)) in pending:
                    pending.pop(int(m.group(1)))["wrist_adc"] = float(m.group(2))
            with lock:
                if row["phase"]:
                    rows.append(row)
            for k in [k for k in pending if k < seq - 20]:
                pending.pop(k)  # never answered: keep the row without that reading

    th = threading.Thread(target=poll, daemon=True)
    th.start()
    try:
        for name, text in phases:
            step = f"step {phases.index((name, text)) + 1} of {len(phases)}: "
            print(f"\n[{step}{name}] {text}.")
            state["phase"] = name
            answer = input("   Enter when done (s = skip this phase, q = finish) > ").strip().lower()
            if answer == "q":
                break
            if name.startswith("stop-") and answer != "s":
                now = time.time() - start
                with lock:
                    recent = [r for r in rows if r["phase"] == name and r["L"] is not None and now - r["t"] <= 0.5]
                if recent:
                    held = {"t": now, "phase": name, "held": True}
                    for k in ("B", "L", "R", "elbow", "wrist_adc"):
                        vals = [r[k] for r in recent if r[k] is not None]
                        held[k] = statistics.median(vals) if vals else None
                    with lock:
                        rows.append(held)
                    print(f"   held: B{held['B']:.2f} L{held['L']:.2f} R{held['R']:.2f} elbow{held['elbow']:.2f}")
                else:
                    print("   no readings in the last 0.5 s: not recorded")
    finally:
        state["phase"] = ""
        state["run"] = False
        th.join(timeout=1)
    if input("\nLock the joints again where the arm is now (M17, no motion)? [y/N] > ").strip().lower() == "y":
        s.write(b"#2 M17\n")
        time.sleep(0.3)
    name = f"teach-{datetime.datetime.now():%Y%m%d-%H%M%S}.csv"
    with open(name, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["t", "phase", "B", "L", "R", "elbow", "wrist_adc", "held"])
        w.writeheader()
        for r in rows:
            w.writerow({k: ("" if r.get(k) is None else (1 if r.get(k) is True else 0 if r.get(k) is False else r[k]))
                        for k in w.fieldnames})
    print(f"log: {name} ({len(rows)} rows)")
    summary(rows)


main()
