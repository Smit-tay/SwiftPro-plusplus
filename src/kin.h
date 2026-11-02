// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jack Sidman Smith
//
// Kinematics of the Swift Pro arm, from its geometry.
//
// Geometry (the original's constants, uarm_coord_convert.c:44-62): the shoulder joint sits 13.2 mm from the base
// axis and 107.4 mm above the arm-frame origin; the lower arm is 142.07 mm, the upper arm 158.81 mm. The linkage keeps
// the upper arm's angle independent of the lower arm's:
//   a = left motor angle (L): lower arm, measured up from horizontal
//   b = right motor angle (R): upper arm, measured down from horizontal
//   c = base angle in the "-90" frame (0 = straight ahead, positive towards +y)
// Arm frame: the arm's own XYZ. Effector frame: arm frame plus the work mode's front and height offsets.
//
// Changes from the original: the base angle is defined for every point (Q-57; the original had NaN for x < 0), and
// steps are rounded to the nearest step (R-1; the original truncated, a bias of up to one step).

#pragma once

#include <stdint.h>

namespace kin {

constexpr float kStepsPerDegree = 80.0f;  // 32 microsteps x 4.5 gearbox / 1.8 deg (uarm_coord_convert.c:52-54)

struct Offsets {
    float front;   // EEPROM 920 (mode 0 default 56.65)
    float height;  // EEPROM 910 (mode 0 default 74.55)
};

// Joint limits, degrees: the arm's stops found by hand with the joints released (TEACHABLE runs of 2026-10-21 and
// 2026-10-24), each a little inside (Q-82, Q-83, the 2026-10-24 refit). docs/LIMITS.md section 6 derives them; this
// and kin.cpp's R table are the only place they are defined.
struct Range {
    float lo, hi;
};
constexpr Range kLimitL{-13.8f, 135.6f};    // stops -14.79 / 136.73; the top stays at the original's 135.6
constexpr Range kLimitBase{-95.0f, 95.0f};  // the -90 frame; stops -7.56 / 187.65 in the user frame (2026-10-24)

// The range of R at L = a: R's stops move with L, and the elbow's (180 - L - R) lie inside this range. Straight lines
// between points every 5 deg of L (tools/limits_sweep/fit_table.py); outside kLimitL, the nearest end's range.
Range limit_r(float a);

// For a joint move along L from (a, b), up or down: the farthest L it reaches while R = b stays inside the limits all
// the way, and that lies inset inside them (L's and R's). a itself if there is none, or if (a, b) is outside them.
float l_reach(float a, float b, bool up, float inset);

// True if L, R (at that L) and the base lie inside their limits, each range shrunk by inset at both ends. NaN is
// illegal.
bool legal(float a, float b, float c, float inset = 0.0f);

// Joints to the arm frame.
void forward(float a, float b, float c, float* x, float* y, float* z);

// Arm frame to joints (elbow-up solution). Unreachable points give NaN for a and b; legal() refuses them.
void inverse(float x, float y, float z, float* a, float* b, float* c);

// Arm frame <-> effector frame: the front offset along the base direction, the height offset down.
void arm_to_effector(const Offsets& o, float* x, float* y, float* z);
void effector_to_arm(const Offsets& o, float* x, float* y, float* z);

// Steps from the boot angle (init) to angle, to the nearest step, 16 bits. For the base, c is in the -90 frame and
// init_base in the user frame (90 = straight ahead).
int16_t arm_steps(float angle, float init_angle);
int16_t base_steps(float c, float init_base);

// The inverse: the angle after a number of steps from the boot angle.
float arm_angle(float steps, float init_angle);
float base_angle(float steps, float init_base);

}  // namespace kin
