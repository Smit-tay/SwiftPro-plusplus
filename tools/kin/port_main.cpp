// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jack Sidman Smith
//
// Prints the port's kinematics results for every row, in the same format as orig_main.c.
#include "common.h"
#include "kin.h"
int main() {
    UCSR0B = _BV(TXEN0);
    const kin::Offsets off{56.65f, 74.55f};
    for (unsigned i = 0; i < sizeof kRows / sizeof kRows[0]; ++i) {
        Row r; memcpy_P(&r, &kRows[i], sizeof r);
        float a = fl(r.v[0]), b = fl(r.v[1]), c = fl(r.v[2]), x, y, z;
        if (r.kind == 'A') {
            kin::forward(a, b, c, &x, &y, &z); hex(x); hex(y); hex(z);
            kin::arm_to_effector(off, &x, &y, &z); hex(x); hex(y); hex(z);
            kin::effector_to_arm(off, &x, &y, &z); hex(x); hex(y); hex(z);
            put(kin::legal(a, b, c) ? '1' : '0');
        } else if (r.kind == 'C') {
            kin::inverse(a, b, c, &x, &y, &z); hex(x); hex(y); hex(z);
        } else if (r.kind == 'S') {
            hex(float(kin::arm_steps(a, b))); hex(float(kin::base_steps(a, c)));
            hex(kin::arm_angle(a, b)); hex(kin::base_angle(a, c));
        } else {
            put(kin::legal(a, b, c) ? '1' : '0');
        }
        put('\n');
    }
    finish();
}
