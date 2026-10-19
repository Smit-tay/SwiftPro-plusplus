// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jack Sidman Smith
/* Prints the original's kinematics results for every row. Built with the original's toolchain. */
#include "common.h"
#include "uarm_coord_convert.h"
int main(void) {
    UCSR0B = _BV(TXEN0);
    uarm.param.front_offset = 56.65f; uarm.param.high_offset = 74.55f;
    for (unsigned i = 0; i < sizeof kRows / sizeof kRows[0]; ++i) {
        struct Row r; memcpy_P(&r, &kRows[i], sizeof r);
        float a = fl(r.v[0]), b = fl(r.v[1]), c = fl(r.v[2]), x, y, z;
        if (r.kind == 'A') {
            angle_to_coord(a, b, c, &x, &y, &z); hex(x); hex(y); hex(z);
            coord_arm2effect(&x, &y, &z); hex(x); hex(y); hex(z);
            coord_effect2arm(&x, &y, &z); hex(x); hex(y); hex(z);
            put(is_angle_legal(a, b, c) ? '1' : '0');
        } else if (r.kind == 'C') {
            coord_to_angle(a, b, c, &x, &y, &z); hex(x); hex(y); hex(z);
        } else if (r.kind == 'S') {
            uarm.init_arml_angle = b; uarm.init_armr_angle = b; uarm.init_base_angle = c;
            get_current_step(a, a, a, &x, &y, &z); hex(x); hex(z);
            get_current_angle(a, a, a, &x, &y, &z); hex(x); hex(z);
        } else {
            put(is_angle_legal(a, b, c) ? '1' : '0');
        }
        put('\n');
    }
    finish();
}
