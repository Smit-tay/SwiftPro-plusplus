# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Jack Sidman Smith
#
# Compares the sweeps written by run.sh: compare.py <dir with sweep-*.txt>. Prints the summary used in
# docs/LIMITS.md and writes region.csv (every grid point, every check).
#
# "joint" = what a joint command (G2202 N0-2, G2206) accepts at that pose: the original's round trip, our direct check.
# "cart"  = what a Cartesian command (G0, M2222) accepts for the same pose: the round trip in both.
import os
import re
import sys

VERSIONS = ["V4.9.0", "V4.10.0", "jss.7", "pp"]
LIMITS = {  # (L min, L max, R min, R max, elbow min, elbow max) per version, for the float-noise test
    "V4.9.0": (-4.0, 135.6, -4.0, 119.9, 10.0, 151.0),
    "V4.10.0": (-4.0, 135.6, -4.0, 119.9, 10.0, 151.0),
    "jss.7": (-20.0, 135.6, -20.0, 119.9, 10.0, 151.0),
}
NOISE = 0.01  # deg: a round-trip angle this close to a limit is float noise, not a real difference


def pp_limits():
    """SwiftPro++'s limits as kin.h and kin.cpp define them: L's range and the R table (L, R min, R max)."""
    src = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "src")
    h = open(os.path.join(src, "kin.h")).read()
    c = open(os.path.join(src, "kin.cpp")).read()
    l = re.search(r"kLimitL\{(-?[\d.]+)f, (-?[\d.]+)f\}", h)
    l0 = float(re.search(r"kTableL0 = (-?[\d.]+)f", c).group(1))
    step = float(re.search(r"kTableStep = (-?[\d.]+)f", c).group(1))
    body = re.search(r"kTableR\[kTablePoints\]\[2\] = \{(.*?)\n\};", c, re.S).group(1)
    table = [(l0 + i * step, lo / 10, hi / 10) for i, (lo, hi) in
             enumerate((int(a), int(b)) for a, b in re.findall(r"\{(-?\d+),\s*(-?\d+)\}", body))]
    return float(l.group(1)), float(l.group(2)), table


PP_L_MIN, PP_L_MAX, PP_TABLE = pp_limits()


def pp_range(l):  # limit_r (kin.cpp): straight lines between the points, the nearest end's range outside
    if l <= PP_TABLE[0][0]:
        return PP_TABLE[0][1:]
    if l >= PP_TABLE[-1][0]:
        return PP_TABLE[-1][1:]
    for (a, lo0, hi0), (b, lo1, hi1) in zip(PP_TABLE, PP_TABLE[1:]):
        if a <= l <= b:
            f = (l - a) / (b - a)
            return lo0 + f * (lo1 - lo0), hi0 + f * (hi1 - hi0)


def load(path):
    grid, base = {}, {}
    with open(path) as f:
        for line in f:
            k, l, r, b, d, t, l2, r2, b2 = line.split()
            key = (float(l), float(r)) if k == "G" else float(b)
            val = (d == "1", t == "1", float(l2), float(r2), float(b2))
            (grid if k == "G" else base)[key] = val
    return grid, base


def near_limit(v, l2, r2):
    if v == "pp":
        rmin, rmax = pp_range(l2)
        return any(abs(a - b) <= NOISE for a, b in ((l2, PP_L_MIN), (l2, PP_L_MAX), (r2, rmin), (r2, rmax)))
    lmin, lmax, rmin, rmax, emin, emax = LIMITS[v]
    e = 180.0 - l2 - r2
    return any(abs(a - b) <= NOISE for a, b in ((l2, lmin), (l2, lmax), (r2, rmin), (r2, rmax), (e, emin), (e, emax)))


def extent(points):
    if not points:
        return "none"
    ls = [p[0] for p in points]
    rs = [p[1] for p in points]
    return f"L {min(ls):.1f}..{max(ls):.1f}, R {min(rs):.1f}..{max(rs):.1f}, elbow {min(180 - a - b for a, b in points):.1f}..{max(180 - a - b for a, b in points):.1f}"


def main():
    d = sys.argv[1]
    grids, bases = {}, {}
    for v in VERSIONS:
        grids[v], bases[v] = load(f"{d}/sweep-{v}.txt")
    keys = sorted(grids["pp"])
    joint = {v: {k for k in keys if grids[v][k][1]} for v in VERSIONS[:3]}
    joint["pp"] = {k for k in keys if grids["pp"][k][0]}
    cart = {v: {k for k in keys if grids[v][k][1]} for v in VERSIONS}
    direct = {v: {k for k in keys if grids[v][k][0]} for v in VERSIONS}

    print(f"grid: L -30..145, R -30..130, step 0.5, base 0 (-90 frame); {len(keys)} points; mode 0 offsets\n")
    print("== Regions (points, extent)")
    for v in VERSIONS:
        print(f"{v:8} is_angle_legal   {len(direct[v]):6}  {extent(direct[v])}")
        print(f"{v:8} joint commands   {len(joint[v]):6}  {extent(joint[v])}")
        print(f"{v:8} Cartesian        {len(cart[v]):6}  {extent(cart[v])}")

    print("\n== is_angle_legal accepts, the round trip refuses (the original's G2202/G2206 refuse a legal pose)")
    for v in VERSIONS:
        lost = sorted(direct[v] - cart[v])
        noise = [k for k in lost if near_limit(v, grids[v][k][2], grids[v][k][3])]
        real = [k for k in lost if k not in noise]
        print(f"{v:8} {len(lost):5} points: {len(noise)} float noise on a limit, {len(real)} real; real extent: {extent(real)}")
        for k in real[:6]:
            g = grids[v][k]
            print(f"           e.g. L{k[0]:.1f} R{k[1]:.1f} -> IK L{g[2]:.4f} R{g[3]:.4f} B{g[4]:.4f}")
    print("\n== round trip accepts, is_angle_legal refuses (IK lands on another, legal pose)")
    for v in VERSIONS:
        extra = sorted(cart[v] - direct[v])
        print(f"{v:8} {len(extra):5} points; extent {extent(extra)}")

    print("\n== Differences between firmwares (joint commands)")
    pairs = [("V4.9.0", "V4.10.0"), ("V4.9.0", "jss.7"), ("jss.7", "pp"), ("V4.9.0", "pp")]
    for a, b in pairs:
        only_a, only_b = joint[a] - joint[b], joint[b] - joint[a]
        print(f"{a} vs {b}: {len(only_a)} only in {a} ({extent(only_a)}); {len(only_b)} only in {b} ({extent(only_b)})")
    print("pp joint vs pp Cartesian:", len(joint["pp"] - cart["pp"]), "only joint;", len(cart["pp"] - joint["pp"]), "only Cartesian")

    print("\n== R range accepted by joint commands, per L (min..max; '-' none; '*' = not one interval)")
    print(f"{'L':>6} | " + " | ".join(f"{v:>15}" for v in VERSIONS))
    for l in [x / 2 for x in range(-60, 291, 10)]:
        cells = []
        for v in VERSIONS:
            rs = sorted(k[1] for k in joint[v] if k[0] == l)
            if not rs:
                cells.append(f"{'-':>15}")
                continue
            contiguous = all(abs(b - a - 0.5) < 1e-6 for a, b in zip(rs, rs[1:]))
            cells.append(f"{rs[0]:6.1f}..{rs[-1]:6.1f}{'' if contiguous else '*'}".rjust(15))
        print(f"{l:6.1f} | " + " | ".join(cells))

    print("\n== Base at L46 R66 (-90 frame), joint commands")
    for v in VERSIONS:
        bs = sorted(b for b, g in bases[v].items() if (g[1] if v != "pp" else g[0]))
        print(f"{v:8} {bs[0]:.1f}..{bs[-1]:.1f}" if bs else f"{v:8} none")

    print("\n== Map, joint commands, 2.5 deg cells (L across, R down): '#' all four, 'j' jss.7 and pp only,")
    print("   'p' pp only, '!' any other combination, '.' none")
    ls = [x / 2 for x in range(-60, 291, 5)]
    rs = [x / 2 for x in range(-60, 261, 5)]
    ticks = "".join("|" if l % 25 == 0 else " " for l in ls)
    labels = [" "] * (len(ls) + 4)
    for i, l in enumerate(ls):
        if l % 25 == 0:
            for j, ch in enumerate(f"{int(l)}"): labels[i + j] = ch
    print("        " + ticks)
    for r in reversed(rs):
        row = ""
        for l in ls:
            s = tuple((l, r) in joint[v] for v in VERSIONS)
            row += {(True, True, True, True): "#", (False, False, True, True): "j", (False, False, False, True): "p",
                    (False, False, False, False): "."}.get(s, "!")
        print(f"R{r:6.1f} {row}")
    print("        " + ticks)
    print("      L " + "".join(labels).rstrip())

    with open(f"{d}/region.csv", "w") as f:
        f.write("L,R," + ",".join(f"{v}_direct,{v}_trip" for v in VERSIONS) + "\n")
        for k in keys:
            f.write(f"{k[0]},{k[1]}," + ",".join(f"{int(grids[v][k][0])},{int(grids[v][k][1])}" for v in VERSIONS) + "\n")
    print(f"\nall points: {d}/region.csv")


main()
