// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jack Sidman Smith
/* The original's legality on a grid, built on the host with a copy of one version's uarm_coord_convert.c
 * (tools/limits_sweep/run.sh). Per point it prints the direct check, is_angle_legal(L, R, B), and the check a joint
 * command really makes: G2202 and G2206 turn the angles into a Cartesian target (angle_to_coord, coord_arm2effect)
 * and mc_line turns it back (coord_effect2arm, coord_to_angle) before is_angle_legal (jss.7 uarm_protocol.c:279-321,
 * 386-404, motion_control.c:66-84). That is also the check of G0 and M2222 for the same pose.
 *   grid lines:  G <L> <R> <B> <direct 0/1> <round trip 0/1> <L'> <R'> <B'>
 */
#include <stdio.h>
#include "uarm_coord_convert.h"

struct uarm_t uarm;

static void point(char kind, float l, float r, float b) {
    float x, y, z, a2, b2, c2;
    const int direct = is_angle_legal(l, r, b);
    angle_to_coord(l, r, b, &x, &y, &z);
    coord_arm2effect(&x, &y, &z);
    coord_effect2arm(&x, &y, &z);
    coord_to_angle(x, y, z, &a2, &b2, &c2);
    const int trip = is_angle_legal(a2, b2, c2);
    printf("%c %.1f %.1f %.1f %d %d %.4f %.4f %.4f\n", kind, l, r, b, direct, trip, a2, b2, c2);
}

int main(void) {
    uarm.param.front_offset = 56.65f;  /* mode 0 (uarm_common.h:116-117) */
    uarm.param.high_offset = 74.55f;
    for (int i = 0; i <= 350; ++i)        /* L -30 .. 145 */
        for (int j = 0; j <= 320; ++j)    /* R -30 .. 130 */
            point('G', -30.0f + 0.5f * i, -30.0f + 0.5f * j, 0.0f);
    /* Base, -100 .. 100 in the -90 frame, at the probe pose (L46 R66) */
    for (int k = 0; k <= 400; ++k) point('B', 46.0f, 66.0f, -100.0f + 0.5f * k);
    return 0;
}
