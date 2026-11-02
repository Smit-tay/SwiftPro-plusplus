#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Jack Sidman Smith
#
# TEACHABLE limits, guided (docs/LIMITS.md): the motors are unlocked (M2019) and you move the arm by hand, one step
# per screen, while the script polls P2200 (B, L, R measured by the encoders) at about 10 Hz. It never moves the arm.
# Works with the original firmware (jss.7) and with SwiftPro++: both answer P2200 from the encoders (PROTOCOL.md 6.4).
#
#   python3 -I tools/limits/teach_limits.py [--port /dev/ttyACM0]      (needs pyserial)
#   python3 -I tools/limits/teach_limits.py --demo                      (a synthetic arm instead of the serial port)
#   python3 -I tools/limits/teach_limits.py --print-screens             (every screen's fixed text, in order, then exit)
#   python3 -I tools/limits/teach_limits.py --report <run dir> [--margin 1.5]   (summary.md and LIMITS-draft.md again)
#
# Safety: Arm.send is the only function that writes to the port. It sends only the commands in ALLOWED, M17 only
# after the last screen, one line at a time, and waits for each reply before the next (the original's RX ring is
# 128 bytes and drops overflow silently, SP-01).
#
# Output: tools/limits/data/<YYYY-MM-DD_HHMM>/
#   raw.csv          every sample: time, step, attempt, accepted, each part in the firmware's frame and the SPEC's
#   stream.csv       the same samples by sensor, written as they arrive (kept if the script dies)
#   unsolicited.txt  every line that was not the reply to the command in flight (@ reports, boot text, late replies)
#   meta.json        firmware replies, part mapping, noise floor, accepted attempts
#   summary.md       per step extremes and dwell, SPEC | SOURCE | MEASURED, the lower arm / forearm boundary
#   LIMITS-draft.md  proposed limits (measured stop minus a margin), for approval; nothing is applied
#
# Frames: P2200 gives B in the user frame (90 = straight ahead), L up from horizontal, R down from horizontal
# (PROTOCOL.md 5, kin.h). TO_SPEC converts each to the frame of uFactory's published ranges. It is the identity:
# whether the SPEC uses the same zeros is UNVERIFIED (LIMITS.md 1), and the summary flags every disagreement.
import argparse
import csv
import datetime
import json
import math
import os
import random
import re
import select
import statistics
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
COUNT = 360.0 / 4096  # one encoder count in degrees (encoder.cpp angle(): AS5600, 12 bits)
PERIOD = 0.1          # poll interval, s

# Every command this script can send. Nothing else, ever: no G commands, no other M commands.
ALLOWED = {
    "M2019": "unlock all motors (step 0)",
    "M17": "re-lock all motors (step 11 only, after you confirm the arm is supported)",
    "P2200": "joint angles measured by the encoders",
    "P2201": "device name",
    "P2203": "firmware version",
    "P2204": "protocol version",
    "P2242": "encoder reference counts",
    "P2244": "encoder health",
}

SENSORS = ("B", "L", "R")  # the fields of P2200, in its order
SENSOR_WORDS = {"B": "base motor reading", "L": "left motor reading", "R": "right motor reading"}
PARTS = ("base", "lower arm", "forearm")  # the hand has no reading (see HAND_NOTE)
SPEC = {"B": (0.0, 180.0), "L": (0.0, 130.0), "R": (0.0, 106.0)}  # LIMITS.md: spec sheet, Developer Manual V1.0.6
SPEC_HAND = (0.0, 180.0)
TO_SPEC = {"B": (1.0, 0.0), "L": (1.0, 0.0), "R": (1.0, 0.0)}  # spec = sign * firmware + offset; UNVERIFIED identity
JSS7 = {"B": (0.0, 180.0), "L": (-20.0, 135.6), "R": (-20.0, 119.9), "elbow": (10.0, 151.0)}  # uarm_coord_convert.c:56-63
TOL = 0.5  # a measured stop and a limit closer than this are not flagged (about 6 counts)

REPLY_ANGLES = re.compile(r"^ok B(-?[\d.]+) L(-?[\d.]+) R(-?[\d.]+)$")


class SafetyError(Exception):
    pass


class Abort(Exception):
    pass


def spec(sensor, v):
    if v is None:
        return None
    sign, offset = TO_SPEC[sensor]
    return sign * v + offset


# ---------------------------------------------------------------------------------------------------------------------
# Screens. Every screen is at most 10 lines: a title, then labelled lines. The fixed text lives here, so that
# --print-screens shows exactly what the run shows.

LEGEND = "Lower arm = the long section from the base up to the elbow. Forearm = the section from the elbow out to the hand."
VIEW = "Stand behind the robot, so the arm points away from you."
STOP_WORD = "A stop is the point where a part will not move further without force."
HAND_NOTE = [
    ("PART", "Hand = the suction cup at the end of the forearm."),
    ("NOTE", "The robot cannot report the hand's angle, so this test skips the hand."),
    ("DON'T", "Do not twist the hand. Its motor may still be powered."),
    ("WHEN", "Press Enter to go on."),
]
PART_WORDS = {
    "base": "Base = the turning joint at the bottom: the whole arm turns on it.",
    "lower arm": "Lower arm = the long section from the base up to the elbow.",
    "forearm": "Forearm = the section from the elbow out to the hand.",
}
FOLDED = "Fold the arm: both arm sections pulled back close to the base. Keep it folded."
EXTENDED = "Extend the arm: both arm sections reaching forward, away from the base, as far as they go. Keep it extended."
TWO_STOPS = "Press Enter when you have touched both stops."
HOLD = "Hold it there a second."

# Steps 2-8: (id, title, pose, part measured, A, B, don't, parts that must stay still)
SWEEPS = [
    (2, "BASE, ARM FOLDED", FOLDED, "base",
     f"Turn the whole arm on its base, slowly, all the way to YOUR LEFT until it stops. {HOLD}",
     f"Then all the way to YOUR RIGHT until it stops. {HOLD}",
     "Do not force it past a stop. Do not move the other parts.", ("lower arm", "forearm")),
    (3, "BASE, ARM EXTENDED", EXTENDED, "base",
     f"Turn the whole arm on its base, slowly, all the way to YOUR LEFT until it stops. {HOLD}",
     f"Then all the way to YOUR RIGHT until it stops. {HOLD}",
     "Do not force it past a stop. Do not move the other parts.", ("lower arm", "forearm")),
    (4, "LOWER ARM, FOREARM FOLDED",
     "Fold the forearm: pull it in towards the lower arm, as close as it goes. Keep it there.", "lower arm",
     f"Tip the lower arm forward, away from you, slowly, until it stops. {HOLD}",
     f"Then tip it back, towards you, until it stops. {HOLD}",
     "Do not force it past a stop. Do not turn the base.", ("base",)),
    (5, "LOWER ARM, FOREARM EXTENDED",
     "Extend the forearm: open the elbow so the forearm points away from the lower arm, as far as it goes. "
     "Keep it there.", "lower arm",
     f"Tip the lower arm forward, away from you, slowly, until it stops. {HOLD}",
     f"Then tip it back, towards you, until it stops. {HOLD}",
     "Do not force it past a stop. Do not turn the base.", ("base",)),
    (6, "FOREARM, FULL RANGE, ARM FULLY EXTENDED",
     "Extend the arm fully: tip the lower arm forward, away from you, until it stops. Hold the lower arm there with "
     "one hand.", "forearm",
     f"With your other hand, close the elbow: bring the forearm in towards the lower arm until it stops. {HOLD}",
     f"Then open the elbow: move the forearm out through its whole range until it stops. {HOLD}",
     "Do not force it past a stop. Do not move the lower arm or the base.", ("base", "lower arm")),
    (7, "FOREARM, LOWER ARM HALFWAY",
     "Hold the lower arm halfway between its two stops: aim for {halfway} on the LIVE line.", "forearm",
     f"Close the elbow: bring the forearm in towards the lower arm until it stops. {HOLD}",
     f"Then open the elbow: move the forearm out, away from the lower arm, until it stops. {HOLD}",
     "Do not force it past a stop. Do not move the lower arm or the base.", ("base", "lower arm")),
    (8, "FOREARM, LOWER ARM AT ITS HIGHEST",
     "Tip the lower arm back, towards you, until it stops. Hold it there.", "forearm",
     f"Close the elbow: bring the forearm in towards the lower arm until it stops. {HOLD}",
     f"Then open the elbow: move the forearm out, away from the lower arm, until it stops. {HOLD}",
     "Do not force it past a stop. Do not move the lower arm or the base.", ("base", "lower arm")),
]
EXPLORE_SECONDS = 60


def title(n, text):
    return f"=== STEP {n} of 11: {text} ==="


def render(head, lines):
    out = [head]
    for label, text in lines:
        if isinstance(text, (list, tuple)):  # A. and B. halves
            for i, t in enumerate(text):
                out.append(f"{label if i == 0 else '':7}{'AB'[i]}. {t}")
        else:
            out.append(f"{label:7}{text}" if label else text)
    assert len(out) <= 10, head
    return out


def screen_safety():
    return render(title(0, "SAFETY"), [
        ("", "SUPPORT THE ARM. When the motors unlock, the arm will sag under its own weight."),
        ("", "Hold it, or rest it on its folded position."),
        ("DO", "Put one hand under the arm now."),
        ("WHEN", "Type YES and press Enter to unlock the motors. Type q to stop without unlocking."),
    ])


def screen_check():
    return render(title(0, "CHECK THE READINGS"), [
        ("POSE", "Any pose. Keep supporting the arm."),
        ("DO", "Move any part of the arm a little, then let it rest."),
        ("VIEW", "Watch the numbers on the LIVE line."),
        ("DON'T", "Do not force any part."),
        ("WHEN", "Do the numbers change? Enter = yes, q = no."),
    ])


def screen_name(i, part):
    return render(title(1, f"NAME THE PARTS ({i} of 3): {part.upper()}"), [
        ("PART", PART_WORDS[part]),
        ("POSE", "Any pose. Keep supporting the arm."),
        ("DO", f"Gently move ONLY the {part} a little, then let go."),
        ("VIEW", VIEW),
        ("DON'T", "Do not move the other parts."),
        ("WHEN", "Press Enter when you have let go."),
    ])


def screen_hand():
    return render(title(1, "NAME THE PARTS: HAND"), HAND_NOTE)


def screen_sweep(step, halfway="the halfway value"):
    n, head, pose, _part, a, b, dont, _still = step
    lines = [("PARTS", LEGEND), ("POSE", pose.format(halfway=halfway)), ("DO", (a, b)), ("VIEW", VIEW)]
    if n == 2:  # the first screen that says "stop"
        lines.append(("STOP", STOP_WORD))
    lines += [("DON'T", dont), ("WHEN", TWO_STOPS)]
    return render(title(n, head), lines)


def screen_explore():
    return render(title(9, "LOWER ARM AND FOREARM TOGETHER"), [
        ("PARTS", LEGEND),
        ("POSE", "Any pose. Keep the base still."),
        ("DO", "Move the lower arm and forearm together to every corner of their range. "
               "Try to make them touch each other's limit."),
        ("VIEW", VIEW),
        ("DON'T", "Do not force any part past a stop. Do not turn the base."),
        ("WHEN", f"The step ends by itself after {EXPLORE_SECONDS} seconds. Enter ends it early."),
    ])


def screen_extra(name):
    return render(title(10, f"EXTRA TEST: {name}"), [
        ("PARTS", LEGEND),
        ("DO", "Do the test you named, slowly."),
        ("VIEW", VIEW),
        ("DON'T", "Do not force any part past a stop."),
        ("WHEN", "Press Enter when you are done."),
    ])


def screen_finish():
    return render(title(11, "LOCK THE MOTORS"), [
        ("", "The motors are still unlocked. Support the arm, then press Enter to re-lock (M17)."),
        ("NOTE", "When the motors lock, the hand may turn back to its last set angle."),
        ("", "Type n and Enter to leave the motors unlocked."),
    ])


EXTRA_ASK = "Type a short name for any other test, or Enter to finish."
ACCEPT = "Enter = accept, r = redo this step"
MORE = "Another 60 seconds? Enter = done, m = more"


def print_screens():
    blocks = [screen_safety(), screen_check() + ["LIVE   (live readings)"]]
    for i, part in enumerate(PARTS, 1):
        blocks.append(screen_name(i, part) + ["LIVE   (live readings)", "",
                      "Changed most: <reading> (by <n>). Next: <reading> (by <n>).",
                      f"So: {part[0].upper() + part[1:]} = <reading>.", ACCEPT])
    blocks.append(screen_hand())
    for step in SWEEPS:
        part = step[3]
        blocks.append(screen_sweep(step) + ["LIVE   (live readings)", "",
                      f"{part[0].upper() + part[1:]}: lowest <n>, highest <n>", ACCEPT])
    blocks.append(screen_explore() + ["LIVE   (live readings, seconds left)", "", "(grid of the lower arm against the "
                  "forearm, 10 degree cells, X = visited)", MORE])
    blocks.append([EXTRA_ASK])
    blocks.append(screen_extra("<name>") + ["LIVE   (live readings)", "", "<part>: lowest <n>, highest <n> (each)",
                                            ACCEPT])
    blocks.append(screen_finish())
    blocks.append(["Margin for the draft limits, in degrees. Suggested 1 to 2. Enter = 1."])
    print(("\n\n" + "-" * 100 + "\n\n").join("\n".join(b) for b in blocks))


# ---------------------------------------------------------------------------------------------------------------------
# The port.

class SerialLink:
    def __init__(self, port):
        import serial  # pyserial
        self.s = serial.Serial(port, 115200, bytesize=8, parity="N", stopbits=1, timeout=0.05, exclusive=True)
        self.buf = b""

    def write(self, data):
        self.s.write(data)
        self.s.flush()

    def readline(self, timeout):
        end = time.monotonic() + max(0.0, timeout)
        while True:
            i = self.buf.find(b"\n")
            if i >= 0:
                line, self.buf = self.buf[:i], self.buf[i + 1:]
                return line.decode(errors="replace").strip()  # boot lines end "\r\n", replies "\n"
            left = end - time.monotonic()
            if left <= 0:
                return None
            self.s.timeout = min(left, 0.05)
            self.buf += self.s.read(max(1, self.s.in_waiting))


class DemoLink:
    """Stands in for the serial port: answers like the firmware, with joint angles that move the way a person would
    for the current step (set_scene). The stops are roughly those of the 2026-10-21 runs."""
    STOP = {"B": (13.4, 163.7), "L": (-14.8, 136.7)}

    def __init__(self):
        self.out = ["device name : SwiftPro", "soft ver : V5.0.0-pp.1", "api ver : V4.0.5", "work mode : 0", "@1",
                    "@5 V1"]
        self.scene = ("rest", {})
        self.t0 = time.monotonic()
        self.pose = {"B": 90.0, "L": 40.0, "R": 60.0}

    @staticmethod
    def r_range(l):  # elbow (180 - L - R) between about 10 closed and 141 open; R's own stops
        return max(-19.3, 39.0 - l), min(115.5, 170.0 - l)

    def set_scene(self, kind, **kw):
        self.scene = (kind, kw)
        self.t0 = time.monotonic()

    @staticmethod
    def path(t, lo, hi):  # middle -> hi, hold, -> lo, hold, -> middle; 12 s
        t %= 12.0
        mid = (lo + hi) / 2
        legs = ((3.0, mid, hi), (1.5, hi, hi), (5.0, hi, lo), (1.5, lo, lo), (1.0, lo, mid))
        for d, a, b in legs:
            if t < d:
                return a + (b - a) * t / d
            t -= d
        return mid

    def angles(self):
        t = time.monotonic() - self.t0
        kind, kw = self.scene
        p = dict(self.pose)
        if kind == "nudge":
            p[kw["sensor"]] += 8.0 * math.sin(2 * math.pi * t / 4.0)
        elif kind == "sweep":
            p.update(kw.get("fixed", {}))
            s = kw["sensor"]
            if s == "R":
                lo, hi = self.r_range(p["L"])
            else:
                lo, hi = self.STOP[s]
            p[s] = self.path(t, lo, hi)
            if s == "L" and "elbow" in kw:  # the forearm keeps its fold while the lower arm moves
                lo, hi = self.r_range(p["L"])
                p["R"] = min(hi, max(lo, 180.0 - p["L"] - kw["elbow"]))
        elif kind == "explore":
            p["L"] = 61.0 + 76.0 * math.sin(0.31 * t)
            p["L"] = min(self.STOP["L"][1], max(self.STOP["L"][0], p["L"]))
            lo, hi = self.r_range(p["L"])
            p["R"] = lo + (hi - lo) * (0.5 + 0.5 * math.sin(1.13 * t))
        return {s: round(v / COUNT + random.choice((-1, 0, 0, 0, 1))) * COUNT for s, v in p.items()}

    def write(self, data):
        m = re.match(r"^#(\d+) (\S+)\n$", data.decode())
        seq, cmd = m.group(1), m.group(2)
        a = self.angles()
        reply = {
            "P2200": f"ok B{a['B']:.4f} L{a['L']:.4f} R{a['R']:.4f}",
            "P2201": "ok VSwiftPro", "P2203": "ok V5.0.0-pp.1", "P2204": "ok V4.0.5",
            "P2242": "ok B2048 L1700 R2300", "P2244": "ok V0", "M2019": "ok", "M17": "ok",
        }[cmd]
        if random.random() < 0.01:
            self.out.append("@6 N0 V0")
        self.out.append(f"${seq} {reply}")

    def readline(self, timeout):
        if self.out:
            return self.out.pop(0)
        time.sleep(min(max(timeout, 0.0), 0.005))
        return None


class Arm:
    def __init__(self, link, unsolicited):
        self.link = link
        self.unsolicited = unsolicited
        self.seq = 0
        self.relock_allowed = False  # set by the last screen only
        self.step = "connect"
        self.t0 = time.time()

    def note(self, line):
        if line:
            self.unsolicited.write(f"{time.time() - self.t0:.3f}\t{self.step}\t{line}\n")
            self.unsolicited.flush()

    def send(self, cmd, timeout=1.0):
        """The only function that writes to the port. One allowed command, then its reply (None on timeout). Lines
        that are not that reply are logged as unsolicited."""
        if cmd not in ALLOWED:
            raise SafetyError(f"refused to send {cmd!r}: not in the allowlist")
        if cmd == "M17" and not self.relock_allowed:
            raise SafetyError("refused to send M17 before the last screen")
        self.seq = self.seq % 9999 + 1
        tag = f"${self.seq}"
        self.link.write(f"#{self.seq} {cmd}\n".encode())
        end = time.monotonic() + timeout
        while True:
            left = end - time.monotonic()
            if left <= 0:
                return None
            line = self.link.readline(left)
            if line is None:
                continue
            if line == tag:  # the original's empty reply (SP-11)
                return ""
            if line.startswith(tag + " "):
                return line[len(tag) + 1:]
            self.note(line)

    def angles(self):
        """P2200 -> {"B", "L", "R"} in the firmware's frame, or None (no reply, or an error such as E26)."""
        r = self.send("P2200")
        m = REPLY_ANGLES.match(r or "")
        if not m:
            if r is not None:
                self.note(f"(reply to P2200) {r}")
            return None
        return {s: float(m.group(i + 1)) for i, s in enumerate(SENSORS)}

    def wait_boot(self, seconds):
        """Opening the port resets the board. Log the boot text; True once "@1" arrives."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            line = self.link.readline(end - time.monotonic())
            if line is None:
                continue
            self.note(line)
            if line == "@1":
                return True
        return False


def port_holders(port):
    """fuser's report of the processes that hold the port, or None if it is free (or fuser is missing)."""
    try:
        r = subprocess.run(["fuser", "-v", port], capture_output=True, text=True, timeout=5)
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return None
    return (r.stderr + r.stdout).strip() if r.returncode == 0 else None


# ---------------------------------------------------------------------------------------------------------------------
# The run.

def ask(prompt, allowed=("",)):
    while True:
        try:
            a = input(prompt + " > ").strip()
        except EOFError:
            raise Abort("input closed")
        if a.lower() in allowed:
            return a.lower()


def enter_pressed():
    r, _, _ = select.select([sys.stdin], [], [], 0)
    if not r:
        return None
    line = sys.stdin.readline()
    if line == "":
        raise Abort("input closed")
    return line.strip().lower()


def fmt(v):
    return "  -  " if v is None else f"{v:.1f}"


class Session:
    def __init__(self, arm, outdir, demo):
        self.arm = arm
        self.outdir = outdir
        self.demo = demo
        self.rows = []      # {"t", "step_id", "step_name", "attempt", "accepted", "B", "L", "R"}
        self.attempts = {}  # step_id -> attempts so far
        self.mapping = {}   # part -> sensor
        self.meta = {"started": datetime.datetime.now().isoformat(timespec="seconds"), "demo": demo,
                     "to_spec": TO_SPEC, "allowed": ALLOWED, "mapping_evidence": {}}
        self.unlocked = False
        self.stream = open(os.path.join(outdir, "stream.csv"), "w", newline="")
        self.stream_w = csv.writer(self.stream)
        self.stream_w.writerow(["time_s", "step_id", "step_name", "attempt", "B", "L", "R"])

    def scene(self, kind, **kw):
        if hasattr(self.arm.link, "set_scene"):
            self.arm.link.set_scene(kind, **kw)

    def value(self, sample, part):
        return spec(self.mapping[part], sample[self.mapping[part]]) if sample else None

    def live(self, sample, focus, stats):
        if not self.mapping:  # before step 1 ends: by sensor
            return "   ".join(f"{SENSOR_WORDS[s][0].upper() + SENSOR_WORDS[s][1:]} {fmt(spec(s, sample[s]) if sample else None)}"
                               for s in SENSORS)
        out = []
        for part in PARTS:
            text = f"{part[0].upper() + part[1:]} {fmt(self.value(sample, part))}"
            if part in focus and part in stats:
                text += f"   (lowest {stats[part][0]:.1f}, highest {stats[part][1]:.1f})"
            out.append(text)
        return "   ".join(out)

    def show(self, lines):
        print("\033[2J\033[H" + "\n".join(lines))

    def capture(self, step_id, step_name, focus=(), seconds=None, extra=None):
        """Polls P2200 every PERIOD until Enter (or seconds), updating the LIVE line in place. Returns (samples,
        the line typed)."""
        self.attempts[step_id] = self.attempts.get(step_id, 0) + 1
        attempt = self.attempts[step_id]
        self.arm.step = f"{step_id}:{step_name}"
        samples, stats, misses = [], {}, 0
        start = time.monotonic()
        typed = None
        while typed is None:
            tick = time.monotonic()
            a = self.arm.angles()
            t = time.time() - self.arm.t0
            row = {"t": t, "step_id": step_id, "step_name": step_name, "attempt": attempt, "accepted": False,
                   "B": None, "L": None, "R": None}
            if a:
                row.update(a)
                samples.append(row)
                for part in focus:
                    v = self.value(a, part)
                    lo, hi = stats.get(part, (v, v))
                    stats[part] = (min(lo, v), max(hi, v))
            else:
                misses += 1
            self.record(row)
            text = self.live(a, focus, stats)
            if extra:
                text += "   " + extra(tick - start, samples)
            if not a:
                text += "   (no reading)"
            sys.stdout.write("\r\033[KLIVE   " + text)
            sys.stdout.flush()
            if seconds is not None and tick - start >= seconds:
                typed = ""
                break
            while time.monotonic() - tick < PERIOD:
                typed = enter_pressed()
                if typed is not None:
                    break
                time.sleep(0.01)
        print()
        if misses:
            print(f"({misses} readings did not arrive.)")
        return samples, typed, attempt

    def record(self, row):
        self.rows.append(row)
        self.stream_w.writerow([f"{row['t']:.3f}", row["step_id"], row["step_name"], row["attempt"]] +
                               ["" if row[s] is None else f"{row[s]:.4f}" for s in SENSORS])
        self.stream.flush()

    def accept(self, step_id, attempt):
        for r in self.rows:
            if r["step_id"] == step_id and r["attempt"] == attempt:
                r["accepted"] = True

    def extremes(self, samples, part):
        vals = [self.value(r, part) for r in samples]
        return (min(vals), max(vals)) if vals else None

    # Step 0 ----------------------------------------------------------------------------------------------------------
    def connect(self):
        for cmd in ("P2201", "P2203", "P2204", "P2242", "P2244"):
            self.meta[cmd] = self.arm.send(cmd)
        if self.meta["P2203"] is None:
            raise Abort("The robot does not answer. Check the cable and the port, then try again.")
        print(f"Robot: {self.meta['P2201']}   firmware {self.meta['P2203']}   protocol {self.meta['P2204']}")
        if (self.meta["P2244"] or "").split(" ")[0] != "ok":
            print(f"Warning: a joint sensor reports a fault ({self.meta['P2244']}).")
            if ask("Enter = go on anyway, q = stop", ("", "q")) == "q":
                raise Abort("Stopped before unlocking.")
        print("Measuring the resting noise for 2 seconds. Do not touch the arm.")
        self.arm.step = "0:rest"
        self.attempts["0-rest"] = 1
        rest = []
        end = time.monotonic() + 2.0
        while time.monotonic() < end:
            tick = time.monotonic()
            a = self.arm.angles()
            row = {"t": time.time() - self.arm.t0, "step_id": "0-rest", "step_name": "rest, motors locked",
                   "attempt": 1, "accepted": True, "B": None, "L": None, "R": None}
            if a:
                row.update(a)
                rest.append(a)
            self.record(row)
            time.sleep(max(0.0, PERIOD - (time.monotonic() - tick)))
        if len(rest) < 5:
            raise Abort("The robot does not report its joint angles (P2200). Stopping before unlocking.")
        self.meta["noise"] = {s: {"span_counts": round((max(r[s] for r in rest) - min(r[s] for r in rest)) / COUNT),
                                  "stdev_counts": round(statistics.pstdev(r[s] for r in rest) / COUNT, 2),
                                  "samples": len(rest)} for s in SENSORS}

    def step0(self):
        self.show(screen_safety())
        if ask("", ("yes", "q")) == "q":
            raise Abort("Stopped without unlocking.")
        self.arm.step = "0:unlock"
        r = self.arm.send("M2019", timeout=3.0)
        self.unlocked = True  # even without a reply: assume the worst
        self.meta["M2019"] = r
        if r is None or not r.startswith("ok"):
            raise Abort(f"Unlocking did not answer ok ({r}).")
        while True:
            self.show(screen_check())
            self.scene("nudge", sensor="L")
            samples, typed, attempt = self.capture("0", "check the readings")
            moved = max((max(r[s] for r in samples) - min(r[s] for r in samples)) for s in SENSORS) if samples else 0
            if typed == "q":
                raise Abort("The numbers do not follow the arm, so this test cannot measure anything. Stopping.")
            if moved < 1.0:
                print(f"The readings changed by only {moved:.1f}. That is noise, not movement.")
                if ask("r = try again, q = stop", ("r", "q")) == "q":
                    raise Abort("The numbers do not follow the arm, so this test cannot measure anything. Stopping.")
                continue
            self.accept("0", attempt)
            self.meta["check_moved_deg"] = round(moved, 2)
            return

    # Step 1 ----------------------------------------------------------------------------------------------------------
    def step1(self):
        while True:
            found = {}
            for i, part in enumerate(PARTS, 1):
                demo_sensor = {"base": "B", "lower arm": "L", "forearm": "R"}[part]
                while True:
                    self.show(screen_name(i, part))
                    self.scene("nudge", sensor=demo_sensor)
                    samples, _typed, attempt = self.capture(f"1-{part}", f"name the parts: {part}")
                    change = {s: (max(r[s] for r in samples) - min(r[s] for r in samples)) if samples else 0.0
                              for s in SENSORS}
                    order = sorted(SENSORS, key=lambda s: -change[s])
                    best, nxt = order[0], order[1]
                    print(f"Changed most: {SENSOR_WORDS[best]} (by {change[best]:.1f}). "
                          f"Next: {SENSOR_WORDS[nxt]} (by {change[nxt]:.1f}).")
                    if change[best] < 2.0:
                        print("Nothing changed enough to tell. Move the part a little more.")
                        ask("Enter = redo this step")
                        continue
                    print(f"So: {part[0].upper() + part[1:]} = {SENSOR_WORDS[best]}.")
                    if ask(ACCEPT, ("", "r")) == "r":
                        continue
                    self.accept(f"1-{part}", attempt)
                    found[part] = best
                    self.meta["mapping_evidence"][part] = {s: round(change[s], 2) for s in SENSORS}
                    break
            if len(set(found.values())) == 3:
                self.mapping = found
                self.meta["mapping"] = found
                break
            print("Two parts changed the same reading, so the parts cannot be told apart. Step 1 starts again.")
            ask("Enter = redo step 1")
        self.show(screen_hand())
        ask("")

    # Steps 2-8 -------------------------------------------------------------------------------------------------------
    def halfway(self):
        vals = [self.value(r, "lower arm") for r in self.rows
                if r["accepted"] and r["step_id"] in ("4", "5") and r["B"] is not None]
        return f"{(min(vals) + max(vals)) / 2:.0f}" if vals else "the halfway value"

    def demo_fixed(self, n):
        # The demo's pose for each step (sensor names: the demo's arm is wired as expected).
        return {2: dict(sensor="B", fixed={"L": 120.0, "R": 45.0}), 3: dict(sensor="B", fixed={"L": 60.0, "R": 0.0}),
                4: dict(sensor="L", fixed={"B": 90.0}, elbow=12.0), 5: dict(sensor="L", fixed={"B": 90.0}, elbow=140.0),
                6: dict(sensor="R", fixed={"L": -14.0}), 7: dict(sensor="R", fixed={"L": 61.0}),
                8: dict(sensor="R", fixed={"L": 136.0})}[n]

    def sweep(self, step):
        n, head, _pose, part, _a, _b, _dont, still = step
        while True:
            self.show(screen_sweep(step, self.halfway()))
            self.scene("sweep", **self.demo_fixed(n))
            samples, _typed, attempt = self.capture(str(n), head.lower(), focus=(part,))
            e = self.extremes(samples, part)
            if not e:
                print("No readings arrived in this step.")
                ask("Enter = redo this step")
                continue
            print(f"{part[0].upper() + part[1:]}: lowest {e[0]:.1f}, highest {e[1]:.1f}")
            if e[1] - e[0] < 5.0:
                print(f"The {part} hardly moved. Press r to redo this step.")
            for other in still:
                o = self.extremes(samples, other)
                if o[1] - o[0] > 5.0:
                    print(f"The {other} moved by {o[1] - o[0]:.1f} during this step. Press r to redo it if that was "
                          f"not meant.")
            if ask(ACCEPT, ("", "r")) == "":
                self.accept(str(n), attempt)
                return

    # Step 9 ----------------------------------------------------------------------------------------------------------
    def visited(self):
        cells = set()
        for r in self.rows:
            if r["B"] is not None and r["step_id"] in ("4", "5", "6", "7", "8", "9") and r["accepted"]:
                cells.add((math.floor(self.value(r, "lower arm") / 10), math.floor(self.value(r, "forearm") / 10)))
        return cells

    def grid(self, live_cells=()):
        cells = self.visited() | set(live_cells)
        xs = [c[0] for c in cells] + [-2, 13]  # at least the lower arm's -20 to 139
        ys = [c[1] for c in cells] + [-3, 11]  # and the forearm's -30 to 119
        out = ["Forearm (rows) against lower arm (columns), 10 degree cells, X = visited, . = not yet"]
        for y in range(max(ys), min(ys) - 1, -1):
            out.append(f"{y * 10:5} |" + "".join("  X " if (x, y) in cells else "  . " for x in range(min(xs), max(xs) + 1)))
        out.append("      +" + "----" * (max(xs) - min(xs) + 1))
        out.append("       " + "".join(f"{x * 10:4}" for x in range(min(xs), max(xs) + 1)) + "   lower arm")
        return out

    def step9(self):
        while True:
            self.show(screen_explore())
            self.scene("explore")

            def left(elapsed, samples):
                return f"{max(0, EXPLORE_SECONDS - elapsed):.0f} seconds left"
            _samples, _typed, attempt = self.capture("9", "lower arm and forearm together", seconds=EXPLORE_SECONDS,
                                                     extra=left)
            self.accept("9", attempt)
            print("\033[2J\033[H" + "\n".join(self.grid()))
            if ask(MORE, ("", "m")) == "":
                return

    # Step 10 ---------------------------------------------------------------------------------------------------------
    def step10(self):
        while True:
            name = ""
            print()
            try:
                name = input(EXTRA_ASK + " > ").strip()
            except EOFError:
                raise Abort("input closed")
            if not name:
                return
            step_id = "10-" + re.sub(r"[^a-z0-9]+", "-", name.lower()).strip("-")
            while True:
                self.show(screen_extra(name))
                self.scene("explore")
                samples, _typed, attempt = self.capture(step_id, f"extra: {name}", focus=PARTS)
                for part in PARTS:
                    e = self.extremes(samples, part)
                    if e:
                        print(f"{part[0].upper() + part[1:]}: lowest {e[0]:.1f}, highest {e[1]:.1f}")
                if ask(ACCEPT, ("", "r")) == "":
                    self.accept(step_id, attempt)
                    break

    # Step 11 ---------------------------------------------------------------------------------------------------------
    def finish(self):
        if not self.unlocked:
            return
        self.scene("rest")
        self.show(screen_finish())
        try:
            a = ask("", ("", "n"))
        except (Abort, KeyboardInterrupt):
            a = "n"
        if a == "n":
            print("The motors are still unlocked.")
            self.meta["M17"] = "not sent"
            return
        self.arm.relock_allowed = True
        self.arm.step = "11:re-lock"
        r = self.arm.send("M17", timeout=3.0)
        self.meta["M17"] = r
        if r is not None and r.startswith("ok"):
            print("The motors are locked.")
            self.unlocked = False
        else:
            print(f"Re-locking did not answer ok ({r}). The motors may still be unlocked.")

    def run(self):
        try:
            self.connect()
            self.step0()
            self.step1()
            for step in SWEEPS:
                self.sweep(step)
            self.step9()
            self.step10()
        except Abort as e:
            print(f"\n{e}")
            self.meta["aborted"] = str(e)
        except KeyboardInterrupt:
            print("\nStopped (Ctrl-C).")
            self.meta["aborted"] = "Ctrl-C"
        finally:
            self.finish()
            self.stream.close()
        margin = 1.0
        try:
            if not self.mapping:  # nothing to propose limits from
                raise EOFError
            m = input("Margin for the draft limits, in degrees. Suggested 1 to 2. Enter = 1. > ").strip()
            margin = float(m) if m else 1.0
        except (EOFError, KeyboardInterrupt, ValueError):
            pass
        save(self.outdir, self.rows, self.meta)
        report(self.outdir, margin)


# ---------------------------------------------------------------------------------------------------------------------
# Output.

def part_column(part):
    return part.replace(" ", "_")


def save(outdir, rows, meta):
    mapping = meta.get("mapping", {})
    with open(os.path.join(outdir, "raw.csv"), "w", newline="") as f:
        w = csv.writer(f)
        head = ["time_s", "step_id", "step_name", "attempt", "accepted"]
        head += [f"{part_column(p)}_raw" for p in PARTS + ("hand",)] + [f"{part_column(p)}_spec" for p in PARTS + ("hand",)]
        head += ["elbow_raw"]
        w.writerow(head)
        for r in rows:
            raw = ["" if (p not in mapping or r[mapping[p]] is None) else f"{r[mapping[p]]:.4f}" for p in PARTS]
            sp = ["" if (p not in mapping or r[mapping[p]] is None) else f"{spec(mapping[p], r[mapping[p]]):.4f}"
                  for p in PARTS]
            elbow = "" if r["L"] is None else f"{180.0 - r['L'] - r['R']:.4f}"
            w.writerow([f"{r['t']:.3f}", r["step_id"], r["step_name"], r["attempt"], int(r["accepted"])] + raw + [""] +
                       sp + [""] + [elbow])
        # Before step 1 ends there is no mapping; then the columns hold nothing and stream.csv has the readings.
    meta = dict(meta)
    meta["finished"] = datetime.datetime.now().isoformat(timespec="seconds")
    with open(os.path.join(outdir, "meta.json"), "w") as f:
        json.dump(meta, f, indent=2)


def load(outdir):
    meta = json.load(open(os.path.join(outdir, "meta.json")))
    rows = []
    with open(os.path.join(outdir, "stream.csv")) as f:
        for r in csv.DictReader(f):
            rows.append({"t": float(r["time_s"]), "step_id": r["step_id"], "step_name": r["step_name"],
                         "attempt": int(r["attempt"]), **{s: float(r[s]) if r[s] else None for s in SENSORS}})
    accepted = set()
    with open(os.path.join(outdir, "raw.csv")) as f:
        for r in csv.DictReader(f):
            if r["accepted"] == "1":
                accepted.add((r["step_id"], int(r["attempt"])))
    for r in rows:
        r["accepted"] = (r["step_id"], r["attempt"]) in accepted
    return rows, meta


def source_ours():
    """SwiftPro++'s limits, read from src/kin.h and src/kin.cpp: {"L": (lo, hi), "B": (lo, hi) user frame,
    "table": [(L, Rmin, Rmax)]}, or None if they cannot be read."""
    try:
        h = open(os.path.join(REPO, "src", "kin.h")).read()
        c = open(os.path.join(REPO, "src", "kin.cpp")).read()
        l = re.search(r"kLimitL\{(-?[\d.]+)f, (-?[\d.]+)f\}", h)
        b = re.search(r"kLimitBase\{(-?[\d.]+)f, (-?[\d.]+)f\}", h)
        l0 = float(re.search(r"kTableL0 = (-?[\d.]+)f", c).group(1))
        step = float(re.search(r"kTableStep = (-?[\d.]+)f", c).group(1))
        body = re.search(r"kTableR\[kTablePoints\]\[2\] = \{(.*?)\n\};", c, re.S).group(1)
        pts = [(int(x), int(y)) for x, y in re.findall(r"\{(-?\d+),\s*(-?\d+)\}", body)]
        return {"L": (float(l.group(1)), float(l.group(2))),
                "B": (float(b.group(1)) + 90.0, float(b.group(2)) + 90.0),
                "table": [(l0 + i * step, lo / 10.0, hi / 10.0) for i, (lo, hi) in enumerate(pts)]}
    except (OSError, AttributeError, ValueError):
        return None


def ours_r(src, l):
    t = src["table"]
    if l <= t[0][0]:
        return t[0][1], t[0][2]
    if l >= t[-1][0]:
        return t[-1][1], t[-1][2]
    for (a, lo0, hi0), (b, lo1, hi1) in zip(t, t[1:]):
        if a <= l <= b:
            f = (l - a) / (b - a)
            return lo0 + f * (lo1 - lo0), hi0 + f * (hi1 - hi0)


def jss7_r(l):
    lo, hi = JSS7["R"]
    return max(lo, 180.0 - l - JSS7["elbow"][1]), min(hi, 180.0 - l - JSS7["elbow"][0])


def flags(measured, limits):
    """Text flags for a measured (lo, hi) against named limit ranges, in both directions."""
    out = []
    if not measured:
        return out
    for name, lim in limits:
        if not lim:
            continue
        if measured[0] < lim[0] - TOL:
            out.append(f"low stop {lim[0] - measured[0]:.1f} beyond {name}")
        elif measured[0] > lim[0] + TOL:
            out.append(f"{name} low {measured[0] - lim[0]:.1f} past the stop")
        if measured[1] > lim[1] + TOL:
            out.append(f"high stop {measured[1] - lim[1]:.1f} beyond {name}")
        elif measured[1] < lim[1] - TOL:
            out.append(f"{name} high {lim[1] - measured[1]:.1f} past the stop")
    return out


def rng(r):
    return "-" if not r else f"{r[0]:.1f} .. {r[1]:.1f}"


def report(outdir, margin):
    rows, meta = load(outdir)
    mapping = meta.get("mapping")
    lines = [f"# Teach limits run {os.path.basename(outdir)}", ""]
    lines += [f"Started {meta.get('started')}, finished {meta.get('finished')}." +
              (" **Demo run: synthetic data.**" if meta.get("demo") else ""), ""]
    if meta.get("aborted"):
        lines += [f"**Stopped early:** {meta['aborted']}", ""]
    lines += ["## Robot", "",
              f"- P2201 (device): `{meta.get('P2201')}`", f"- P2203 (firmware): `{meta.get('P2203')}`",
              f"- P2204 (protocol): `{meta.get('P2204')}`", f"- P2242 (encoder references): `{meta.get('P2242')}`",
              f"- P2244 (encoder health): `{meta.get('P2244')}`",
              f"- M2019 reply: `{meta.get('M2019')}`, M17 reply: `{meta.get('M17')}`",
              "- Boot text and every other unsolicited line: `unsolicited.txt`.", ""]
    lines += ["## Frames", "",
              "Raw = what P2200 reports: base in the user frame (90 = straight ahead), left motor up from horizontal, "
              "right motor down from horizontal (PROTOCOL.md section 5). Spec frame = raw x sign + offset, with "
              f"{json.dumps(meta.get('to_spec'))}: the identity, because whether the published ranges use the same "
              "zeros is UNVERIFIED. Every number below is in the spec frame unless it says raw.", ""]
    lines += ["## Part mapping (step 1)", ""]
    if mapping:
        lines += ["| Part | Reading | Change seen, base / left / right motor reading |", "|---|---|---|"]
        for part in PARTS:
            ev = meta["mapping_evidence"].get(part, {})
            lines.append(f"| {part} | {SENSOR_WORDS[mapping[part]]} | " +
                         " / ".join(f"{ev.get(s, 0):.1f}" for s in SENSORS) + " |")
        lines += ["| hand | none: the firmware cannot read it | - |", ""]
    else:
        lines += ["Step 1 did not finish: no mapping, so nothing below can be named.", ""]
    noise = meta.get("noise")
    lines += ["## Noise floor", "", f"One count = {COUNT:.4f} degrees. At rest with the motors locked, before "
              "unlocking:", ""]
    if noise:
        lines += ["| Reading | Span (counts) | Std dev (counts) | Samples |", "|---|---|---|---|"]
        lines += [f"| {SENSOR_WORDS[s]} | {noise[s]['span_counts']} | {noise[s]['stdev_counts']} | "
                  f"{noise[s]['samples']} |" for s in SENSORS]
    lines.append("")
    misses = sum(1 for r in rows if r["B"] is None)
    lines += [f"Readings that did not arrive: {misses} of {len(rows)}.", ""]
    if not mapping:
        write(outdir, "summary.md", lines)
        return

    def val(r, part):
        return spec(mapping[part], r[mapping[part]])

    acc = [r for r in rows if r["accepted"] and r["B"] is not None]

    # Per step.
    lines += ["## Per step", "",
              "Extremes of the part measured. Dwell = the median of the samples within 0.5 degree of the extreme, and "
              "how long the part stayed there; \"first\" = the end reached first (A on the screen).", "",
              "| Step | Part | Lowest | Dwell (s) | Highest | Dwell (s) | First | Other parts (range) |",
              "|---|---|---|---|---|---|---|---|"]
    step_ids = sorted({r["step_id"] for r in acc if r["step_id"].split("-")[0] not in ("0", "1")},
                      key=lambda s: (int(s.split("-")[0]), s))
    for sid in step_ids:
        sr = [r for r in acc if r["step_id"] == sid]
        sweep = next((s for s in SWEEPS if str(s[0]) == sid), None)
        parts = (sweep[3],) if sweep else (("lower arm", "forearm") if sid == "9" else PARTS)
        for part in parts:
            vals = [(r["t"], val(r, part)) for r in sr]
            lo, hi = min(v for _, v in vals), max(v for _, v in vals)

            def dwell(x):
                near = [(t, v) for t, v in vals if abs(v - x) <= 0.5]
                return statistics.median(v for _, v in near), len(near) * PERIOD
            dlo, dhi = dwell(lo), dwell(hi)
            first = "lowest" if min(t for t, v in vals if v == lo) < min(t for t, v in vals if v == hi) else "highest"
            others = ", ".join(f"{o} {rng((min(val(r, o) for r in sr), max(val(r, o) for r in sr)))}"
                               for o in PARTS if o not in parts)
            lines.append(f"| {sid} {sr[0]['step_name']} | {part} | {lo:.2f} | {dlo[0]:.2f} ({dlo[1]:.1f}) | "
                         f"{hi:.2f} | {dhi[0]:.2f} ({dhi[1]:.1f}) | {first} | {others} |")
    lines.append("")

    # SPEC | SOURCE | MEASURED.
    later = [r for r in acc if r["step_id"].split("-")[0] not in ("0", "1")]
    measured = {p: (min(val(r, p) for r in later), max(val(r, p) for r in later)) if later else None for p in PARTS}
    elbow = (min(180 - r["L"] - r["R"] for r in later), max(180 - r["L"] - r["R"] for r in later)) if later else None
    src = source_ours()
    ours = {"B": src["B"], "L": src["L"],
            "R": (min(t[1] for t in src["table"]), max(t[2] for t in src["table"]))} if src else {}
    lines += ["## SPEC | SOURCE | MEASURED", "",
              "MEASURED = the extremes over every accepted step from 2 on. A hand trace stops at or inside the true "
              f"stop. Flags (difference over {TOL} degree): \"stop beyond X\" = the arm reaches past limit X; \"X past "
              "the stop\" = limit X allows a pose beyond the stop. SOURCE jss.7 = uarm_coord_convert.c:56-63 (base "
              "-90..90 in the code's frame = 0..180 here). SOURCE SwiftPro++ = src/kin.h and src/kin.cpp as they are "
              "now (the 2026-10-21 limits); its forearm row is the table's overall span, which depends on the lower "
              "arm (next section).", "",
              "| Joint | Reading | SPEC | SOURCE jss.7 | SOURCE SwiftPro++ | MEASURED | Flags |", "|---|---|---|---|---|---|---|"]
    for part in PARTS:
        s = mapping[part]
        lim = [("SPEC", SPEC[s]), ("jss.7", JSS7[s]), ("SwiftPro++", ours.get(s))]
        fl = flags(measured[part], lim)
        lines.append(f"| {part} | {SENSOR_WORDS[s]} | {rng(SPEC[s])} | {rng(JSS7[s])} | {rng(ours.get(s))} | "
                     f"{rng(measured[part])} | {'FLAG: ' + '; '.join(fl) if fl else ''} |")
    fl = flags(elbow, [("jss.7", JSS7["elbow"])])
    lines.append(f"| elbow, 180 - left - right (raw) | both | not published | {rng(JSS7['elbow'])} | inside the table | "
                 f"{rng(elbow)} | {'FLAG: ' + '; '.join(fl) if fl else ''} |")
    lines += [f"| hand | none | {rng(SPEC_HAND)} | 0 .. 360 accepted | 0 .. 180 | not readable | |", ""]

    # The lower arm / forearm boundary.
    region = [r for r in acc if r["step_id"].split("-")[0] in ("4", "5", "6", "7", "8", "9", "10")]
    standard = mapping.get("lower arm") == "L" and mapping.get("forearm") == "R"
    lines += ["## Lower arm / forearm boundary", "",
              "From steps 4 to 10, 10 degree bins. Each extreme is compared with the limits at the lower-arm value "
              "of the sample that gave it." + ("" if standard else " The mapping is not lower arm = left motor, "
                                               "forearm = right motor, so the SOURCE columns are left out."), "",
              "| Lower arm bin | Forearm min .. max | Samples | jss.7 allows | SwiftPro++ allows | Flags |",
              "|---|---|---|---|---|---|"]
    bins = {}
    for r in region:
        bins.setdefault(math.floor(val(r, "lower arm") / 10), []).append(r)
    for b in sorted(bins):
        rs = bins[b]
        rmin = min(rs, key=lambda r: val(r, "forearm"))
        rmax = max(rs, key=lambda r: val(r, "forearm"))
        m = (val(rmin, "forearm"), val(rmax, "forearm"))
        j = o = None
        fl = []
        if standard:
            j = (jss7_r(rmin["L"])[0], jss7_r(rmax["L"])[1])
            fl += flags(m, [("jss.7", j)])
            if src:
                o = (ours_r(src, rmin["L"])[0], ours_r(src, rmax["L"])[1])
                fl += flags(m, [("SwiftPro++", o)])
        lines.append(f"| {b * 10} .. {b * 10 + 10} | {rng(m)} | {len(rs)} | {rng(j)} | {rng(o)} | "
                     f"{'FLAG: ' + '; '.join(fl) if fl else ''} |")
    lines += ["", "| Forearm bin | Lower arm min .. max | Samples |", "|---|---|---|"]
    bins = {}
    for r in region:
        bins.setdefault(math.floor(val(r, "forearm") / 10), []).append(r)
    for b in sorted(bins):
        vs = [val(r, "lower arm") for r in bins[b]]
        lines.append(f"| {b * 10} .. {b * 10 + 10} | {min(vs):.1f} .. {max(vs):.1f} | {len(vs)} |")
    lines += ["", "Proposed limits are not here: see `LIMITS-draft.md`, for approval.", ""]
    write(outdir, "summary.md", lines)

    # The draft.
    d = [f"# LIMITS draft from run {os.path.basename(outdir)} (for approval; nothing is applied)", "",
         f"Each limit = the measured stop moved inwards by a margin of {margin:g} degree (rerun with "
         "`--report <dir> --margin <deg>` for another). Spec frame, which is the firmware's frame (identity, "
         "UNVERIFIED). Where a step's trace did not reach the true stop, the limit is inside it: that costs reach, "
         "not safety.", "", "| Joint | Measured stop | Proposed |", "|---|---|---|"]
    for part in PARTS:
        e = measured[part]
        d.append(f"| {part} | {rng(e)} | {rng((e[0] + margin, e[1] - margin) if e else None)} |")
    d.append(f"| hand | not readable | unchanged |")
    d += ["", "Forearm per 10 degree bin of the lower arm (only bins with at least 10 samples):", "",
          "| Lower arm bin | Forearm measured | Proposed |", "|---|---|---|"]
    bins = {}
    for r in region:
        bins.setdefault(math.floor(val(r, "lower arm") / 10), []).append(val(r, "forearm"))
    for b in sorted(bins):
        if len(bins[b]) >= 10:
            m = (min(bins[b]), max(bins[b]))
            d.append(f"| {b * 10} .. {b * 10 + 10} | {rng(m)} | {rng((m[0] + margin, m[1] - margin))} |")
    write(outdir, "LIMITS-draft.md", d + [""])
    print(f"Wrote {os.path.join(outdir, 'summary.md')} and LIMITS-draft.md")


def write(outdir, name, lines):
    with open(os.path.join(outdir, name), "w") as f:
        f.write("\n".join(lines))


def main():
    ap = argparse.ArgumentParser(description="Guided search for the arm's physical joint stops. Never moves the arm.")
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--demo", action="store_true", help="a synthetic arm instead of the serial port")
    ap.add_argument("--print-screens", action="store_true", help="print every screen's fixed text and exit")
    ap.add_argument("--report", metavar="RUN_DIR", help="write summary.md and LIMITS-draft.md again from a run")
    ap.add_argument("--margin", type=float, default=1.0, help="with --report: margin for the draft, degrees")
    args = ap.parse_args()
    if args.print_screens:
        print_screens()
        return
    if args.report:
        report(args.report, args.margin)
        return
    if args.demo:
        link = DemoLink()
    else:
        held = port_holders(args.port)
        if held:
            sys.exit(f"{args.port} is in use:\n{held}\nClose that program and try again. Nothing was sent.")
        try:
            link = SerialLink(args.port)
        except Exception as e:  # pyserial missing, no such port, busy
            held = port_holders(args.port)
            sys.exit(f"Cannot open {args.port}: {e}" + (f"\nIn use by:\n{held}" if held else ""))
    outdir = os.path.join(HERE, "data", datetime.datetime.now().strftime("%Y-%m-%d_%H%M") + ("_demo" if args.demo else ""))
    os.makedirs(outdir, exist_ok=True)
    with open(os.path.join(outdir, "unsolicited.txt"), "w") as unsolicited:
        arm = Arm(link, unsolicited)
        print("Waiting for the robot to start (up to 10 seconds) ...")
        if not arm.wait_boot(10.0):
            print("No start-up message; asking the robot directly.")
        time.sleep(0.5)
        Session(arm, outdir, args.demo).run()
    print(f"Data: {outdir}")


if __name__ == "__main__":
    main()
