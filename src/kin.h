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

// Joint limits (uarm_coord_convert.c:55-62, 77-101): L -20..135.6, R -20..119.9, 180-L-R in 10..151, base -90..90.
// NaN is illegal.
bool legal(float a, float b, float c);

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
