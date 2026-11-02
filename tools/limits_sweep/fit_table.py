#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Jack Sidman Smith
#
# Fit the (L, R) limit table from the TEACHABLE logs: piecewise linear R min/max on 5-degree L knots
# (docs/LIMITS.md section 6). The outline is every pose the hand traces passed through; each of its edges is rounded
# inwards to a whole degree, then 1 deg further in. The table, in whole degrees, stays inside that everywhere within
# L's limits and has no dents (R min convex, R max concave in L), so a straight joint-space move between two legal
# poses stays legal all the way. Among those tables it gives up the least reach.
#
#   python3 -I tools/limits_sweep/fit_table.py <dir with the 2026-10-21 teach-*.csv logs> <2026-10-24 run dir>
#     (the run dir: tools/limits/data/2026-10-24_1310, written by tools/limits/teach_limits.py)
# Prints the table and the lines for kin.cpp's kTableR.
import csv
import math
import sys

logs, run = sys.argv[1], sys.argv[2]
L_MIN, L_MAX = -13.8, 135.6  # kLimitL: the table is only used inside it
KNOTS = list(range(-15, 141, 5))
SPREAD = 6  # whole degrees each knot may move from the plain fit while the no-dent fit searches
RUNS = (("teach-20261021-182123.csv", None),
        ("teach-20261021-182616.csv", {"sweep-B", "sweep-LR", "stop-B-min", "stop-B-max"}),
        ("teach-20261021-183403.csv", {"sweep-LR"}))
RUN_STEPS = ("4", "5", "6", "7", "8", "9", "10")  # the steps that move the lower arm and the forearm


def polylines(rows):
    """Consecutive samples (< 0.3 s apart) form lines: the arm passed through every pose between them."""
    segs, prev = [], None
    for t, l, r in rows:
        if prev and t - prev[0] < 0.3:
            segs.append(((prev[1], prev[2]), (l, r)))
        prev = (t, l, r)
    return segs


def teach_logs():
    segs = []
    for name, keep in RUNS:
        rows = []
        for r in csv.DictReader(open(f"{logs}/{name}")):
            if r["held"] == "1" or not r["L"] or (keep and r["phase"] not in keep):
                segs += polylines(rows)
                rows = []
                continue
            rows.append((float(r["t"]), float(r["L"]), float(r["R"])))
        segs += polylines(rows)
    return segs


def teach_run():
    accepted = {(r["step_id"], r["attempt"]) for r in csv.DictReader(open(f"{run}/raw.csv")) if r["accepted"] == "1"}
    segs, rows, key = [], [], None
    for r in csv.DictReader(open(f"{run}/stream.csv")):
        k = (r["step_id"], r["attempt"])
        ok = k in accepted and r["step_id"].split("-")[0] in RUN_STEPS and r["L"]
        if not ok or k != key:
            segs += polylines(rows)
            rows, key = [], k
        if ok:
            rows.append((float(r["time_s"]), float(r["L"]), float(r["R"])))
    return segs + polylines(rows)


def outline(segs):
    """Lowest and highest R on any line, at every 0.05 deg of L (index = L / 0.05)."""
    lo, hi = {}, {}
    for (l0, r0), (l1, r1) in segs:
        a, b = sorted((l0, l1))
        pts = [(i * 0.05, r0 if l1 == l0 else r0 + (r1 - r0) * (i * 0.05 - l0) / (l1 - l0))
               for i in range(math.ceil(a / 0.05), math.floor(b / 0.05) + 1)]
        for l, r in pts + [(l0, r0), (l1, r1)]:
            i = round(l / 0.05)
            lo[i] = min(lo.get(i, 1e9), r)
            hi[i] = max(hi.get(i, -1e9), r)
    return lo, hi


def section(l):
    return min(max(int((l - KNOTS[0]) // 5), 0), len(KNOTS) - 2)


def constraints(edge):
    """Per table section, (f, v): a "min" side must be >= v at fraction f of the section. Only L inside L's limits,
    thinned to the largest v per 0.25 deg of L."""
    cells = {}
    for i, v in edge.items():
        l = i * 0.05
        if not L_MIN - 1e-9 <= l <= L_MAX + 1e-9:
            continue
        k = section(l)
        c = (k, round(l / 0.25))
        if c not in cells or v > cells[c][1]:
            cells[c] = ((l - KNOTS[k]) / 5.0, v)
    out = [[] for _ in range(len(KNOTS) - 1)]
    for (k, _), fv in cells.items():
        out[k].append(fv)
    return out


def inside(cons, k, a, b):
    return all(a + f * (b - a) >= v - 1e-9 for f, v in cons[k])


def plain_fit(cons):
    """The lowest whole-degree knots inside the constraints, any shape: the starting point of the search."""
    g = [None] * len(KNOTS)
    for k, cs in enumerate(cons):
        for f, v in cs:
            i = k if f <= 0.5 else k + 1
            g[i] = max(g[i] if g[i] is not None else -1e9, math.ceil(v))
    top = max(x for x in g if x is not None)
    g = [top if x is None else x for x in g]
    changed = True
    while changed:
        changed = False
        for k, cs in enumerate(cons):
            for f, v in cs:
                if g[k] + f * (g[k + 1] - g[k]) < v - 1e-9:
                    g[k if f < 0.5 else k + 1] += 1
                    changed = True
    return g


def no_dent_fit(cons):
    """Exact search: convex whole-degree knots inside the constraints with the least sum, each within SPREAD of the
    plain fit. State (previous knot, this knot). The first knot (L -15) and the last (L 140) lie outside L's limits:
    the first isn't counted, the last continues the line from L 130-135."""
    base = plain_fit(cons)
    n = len(KNOTS)
    opts = [range(base[i] - SPREAD, base[i] + SPREAD + 1) for i in range(n)]
    best = {(a, b): (b, None) for a in opts[0] for b in opts[1] if inside(cons, 0, a, b)}
    trail = [best]
    for i in range(2, n):
        last = i == n - 1
        nxt = {}
        for (a, b), (cost, _) in best.items():
            for c in ([2 * b - a] if last else opts[i]):
                if a + c - 2 * b < 0 or not inside(cons, i - 1, b, c):
                    continue
                total = cost + (0 if last else c)
                if (b, c) not in nxt or total < nxt[(b, c)][0]:
                    nxt[(b, c)] = (total, (a, b))
        best = nxt
        trail.append(best)
    if not best:
        sys.exit(f"no table without dents within {SPREAD} deg of the plain fit")
    key = min(best, key=lambda k: best[k][0])
    g = [key[1], key[0]]
    for i in range(len(trail) - 1, 0, -1):
        key = trail[i][key][1]
        g.append(key[0])
    return g[::-1]


lo, hi = outline(teach_logs() + teach_run())
r_min = no_dent_fit(constraints({i: math.ceil(v) + 1 for i, v in lo.items()}))
r_max = [-v for v in no_dent_fit(constraints({i: -(math.floor(v) - 1) for i, v in hi.items()}))]

worst = 1e9  # the table's distance from the measured outline, inside L's limits
for i in lo:
    l = i * 0.05
    if L_MIN <= l <= L_MAX:
        k = section(l)
        f = (l - KNOTS[k]) / 5.0
        worst = min(worst, r_min[k] + f * (r_min[k + 1] - r_min[k]) - lo[i],
                    hi[i] - (r_max[k] + f * (r_max[k + 1] - r_max[k])))
dents = sum(r_min[i - 1] + r_min[i + 1] < 2 * r_min[i] or r_max[i - 1] + r_max[i + 1] > 2 * r_max[i]
            for i in range(1, len(KNOTS) - 1))
print(f"closest to the measured outline: {worst:.2f} deg (must be >= 1); dents: {dents}")
print("L      R min   R max")
for k, a, b in zip(KNOTS, r_min, r_max):
    print(f"{k:4}  {a:6}  {b:6}")
print("kTableR:")
cells = [f"{{{a * 10}, {b * 10}}}," for a, b in zip(r_min, r_max)]
for i in range(0, len(cells), 7):
    print("    " + " ".join(f"{c:<13}" for c in cells[i:i + 7]).rstrip())
