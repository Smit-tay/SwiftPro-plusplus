// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jack Sidman Smith
//
// SwiftPro++'s legality on the same grid as orig_sweep.c. Joint commands (G2202 N0-2, G2206) check the angles
// directly (motion.cpp queue_joints); G0 and M2222 check the inverse kinematics of the target. Printed in the same
// columns: direct = the joint commands' check, round trip = the Cartesian commands' check for the same pose.

#include "kin.h"

#include <cstdio>

namespace {

void point(char kind, float l, float r, float b) {
    const kin::Offsets o{56.65f, 74.55f};  // mode 0
    float x, y, z, a2, b2, c2;
    const bool direct = kin::legal(l, r, b);
    kin::forward(l, r, b, &x, &y, &z);
    kin::arm_to_effector(o, &x, &y, &z);
    kin::effector_to_arm(o, &x, &y, &z);
    kin::inverse(x, y, z, &a2, &b2, &c2);
    const bool trip = kin::legal(a2, b2, c2);
    std::printf("%c %.1f %.1f %.1f %d %d %.4f %.4f %.4f\n", kind, static_cast<double>(l), static_cast<double>(r),
                static_cast<double>(b), direct ? 1 : 0, trip ? 1 : 0, static_cast<double>(a2),
                static_cast<double>(b2), static_cast<double>(c2));
}

}  // namespace

int main() {
    for (int i = 0; i <= 350; ++i)
        for (int j = 0; j <= 320; ++j) point('G', -30.0f + 0.5f * static_cast<float>(i), -30.0f + 0.5f * static_cast<float>(j), 0.0f);
    for (int k = 0; k <= 400; ++k) point('B', 46.0f, 66.0f, -100.0f + 0.5f * static_cast<float>(k));
    return 0;
}
