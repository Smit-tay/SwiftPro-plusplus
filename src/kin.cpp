// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jack Sidman Smith

#include "kin.h"

#include <math.h>

namespace kin {

namespace {

constexpr float kLower = 142.07f;         // lower arm
constexpr float kUpper = 158.81f;         // upper arm
constexpr float kShoulderRadius = 13.2f;  // shoulder joint from the base axis
constexpr float kShoulderHeight = 107.4f; // shoulder joint above the arm-frame origin
constexpr float kDegPerRad = 57.29578f;   // the original's RAD_TO_DEG, as a float (57.295780181884766)
const float kNaN = __builtin_nanf("");    // NAN is a double in avr-libc

constexpr float rad(float deg) { return deg / kDegPerRad; }
constexpr float deg(float r) { return r * kDegPerRad; }

// Direction of the base towards (x, y) in radians. Defined everywhere (Q-57); the original had none for x < 0.
float heading(float x, float y) { return atan2f(y, x); }

}  // namespace

bool legal(float a, float b, float c) {
    if (isnan(a) || isnan(b) || isnan(c)) return false;
    const float elbow = 180.0f - a - b;  // the angle between the two arms
    return a >= -20.0f && a <= 135.6f && b >= -20.0f && b <= 119.9f && elbow >= 10.0f && elbow <= 151.0f &&
           c >= -90.0f && c <= 90.0f;
}

void forward(float a, float b, float c, float* x, float* y, float* z) {
    const float ra = rad(a), rb = rad(b), rc = rad(c);
    // The wrist point relative to the shoulder: out along the lower arm, then along the upper arm (down by b).
    const float reach = kShoulderRadius + kLower * cosf(ra) + kUpper * cosf(rb);
    *z = kShoulderHeight + kLower * sinf(ra) - kUpper * sinf(rb);
    *x = reach * cosf(rc);
    *y = reach * sinf(rc);
}

void inverse(float x, float y, float z, float* a, float* b, float* c) {
    // Planar problem in the arm's vertical plane, relative to the shoulder.
    const float s = sqrtf(x * x + y * y) - kShoulderRadius;
    const float h = z - kShoulderHeight;
    const float d2 = s * s + h * h;
    const float d = sqrtf(d2);
    // Law of cosines: the angle at the shoulder between the target direction and the lower arm.
    const float shoulder = acosf((kLower * kLower + d2 - kUpper * kUpper) / (2.0f * kLower * d));
    const float ra = atan2f(h, s) + shoulder;  // elbow up
    // The upper arm points from the elbow to the target; b is measured downwards.
    const float ex = s - kLower * cosf(ra);
    const float ey = h - kLower * sinf(ra);
    *a = deg(ra);
    *b = deg(atan2f(-ey, ex));
    *c = deg(heading(x, y));
    if (isnan(*a)) *b = kNaN;  // unreachable
}

void arm_to_effector(const Offsets& o, float* x, float* y, float* z) {
    const float h = heading(*x, *y);
    *x += o.front * cosf(h);
    *y += o.front * sinf(h);
    *z -= o.height;
}

void effector_to_arm(const Offsets& o, float* x, float* y, float* z) {
    const float h = heading(*x, *y);
    *x -= o.front * cosf(h);
    *y -= o.front * sinf(h);
    *z += o.height;
}

int16_t arm_steps(float angle, float init_angle) {
    return static_cast<int16_t>(lroundf((angle - init_angle) * kStepsPerDegree));  // R-1: nearest step
}

int16_t base_steps(float c, float init_base) {
    return static_cast<int16_t>(lroundf((c + 90.0f - init_base) * kStepsPerDegree));
}

float arm_angle(float steps, float init_angle) { return init_angle + steps / kStepsPerDegree; }

float base_angle(float steps, float init_base) { return init_base + steps / kStepsPerDegree - 90.0f; }

}  // namespace kin
