#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Jack Sidman Smith
#
# Compares two kinematics result files from tools/kin (original vs port). Reports bit and 4-decimal text differences
# and the worst absolute difference; exits 1 if a legality result differs or the port gives NaN where the original
# didn't. The original's NaN where the port has a value is expected for points behind the base axis (Q-57).
import struct, sys
from decimal import Decimal, ROUND_HALF_UP

def value(h):
    return struct.unpack("<f", bytes.fromhex(h)[::-1])[0]

def text(x):  # dtostrf(x, 5, 4), the rule in ARCHITECTURE section 12
    if x != x:
        return "NAN"
    d, q4 = Decimal(x), Decimal("0.0001")
    if d != 0:
        q = Decimal(1).scaleb(d.adjusted() - 7)
        if q > q4:
            return str(d.quantize(q, rounding=ROUND_HALF_UP).quantize(q4))
    return str(d.quantize(q4, rounding=ROUND_HALF_UP))

orig, port, label = sys.argv[1:4]
total = bits = texts = hard = q57 = 0
worst = 0.0
for lo, lp in zip(open(orig), open(port)):
    for a, b in zip(lo.split(), lp.split()):
        if len(a) != 8:  # legality flag
            hard += a != b
            continue
        total += 1
        x, y = value(a), value(b)
        bits += a != b
        texts += text(x) != text(y)
        if x != x and y == y:
            q57 += 1
        elif y != y and x == x:
            hard += 1
        elif x == x:
            worst = max(worst, abs(x - y))
print(f"{label}: {total} values, {bits} differ in bits, {texts} in 4-decimal text, worst {worst:.2g}, "
      f"original NaN only (Q-57) {q57}, mismatches {hard}")
sys.exit(1 if hard else 0)
