// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jack Sidman Smith

#include "kin.h"

#include <math.h>
#if defined(__AVR__)
#include <avr/pgmspace.h>
#endif

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

// R's range in tenths of a degree, {min, max}, at L = kTableL0 + i * kTableStep (docs/LIMITS.md section 6: the
// hand-traced outline of 2026-10-21 and 2026-10-24, each edge rounded inwards to a whole degree and then 1 deg more,
// every straight line between two points inside it). R min is convex and R max concave in L, so the region
// has no dents: a straight joint-space move between two legal poses stays legal all the way.
constexpr float kTableL0 = -15.0f;
constexpr float kTableStep = 5.0f;
constexpr uint8_t kTablePoints = 32;
#if defined(__AVR__)
const int16_t kTableR[kTablePoints][2] PROGMEM = {
#else
const int16_t kTableR[kTablePoints][2] = {
#endif
    {560, 1120},  {510, 1120},  {460, 1120},  {410, 1120},  {360, 1120},  {310, 1120},  {260, 1120},  // L -15..15
    {210, 1120},  {160, 1110},  {110, 1100},  {60, 1090},   {10, 1080},   {-40, 1060},  {-80, 1040},  // 20..50
    {-120, 1020}, {-160, 1000}, {-180, 980},  {-180, 940},  {-180, 900},  {-180, 860},  {-180, 820}, // 55..85
    {-180, 780},  {-180, 730},  {-180, 680},  {-180, 630},  {-180, 580},  {-180, 530},  {-180, 480}, // 90..120
    {-180, 430},  {-180, 380},  {-180, 330},  {-180, 280},                                            // 125..140
};

int16_t raw_r(uint8_t i, uint8_t k) {
#if defined(__AVR__)
    return static_cast<int16_t>(pgm_read_word(&kTableR[i][k]));
#else
    return kTableR[i][k];
#endif
}

float table_r(uint8_t i, uint8_t k) { return raw_r(i, k) * 0.1f; }

// Table section i (L from kTableL0 + i * kTableStep, 5 deg): the L where lo(L) + d <= b <= hi(L) - d, within kLimitL
// shrunk by d, as [*from, *to]. Both conditions are linear in L, so it is one interval; false if it is empty.
bool section(uint8_t i, float b, float d, float* from, float* to) {
    const float s0 = kTableL0 + i * kTableStep;
    float f = fmaxf(s0, kLimitL.lo + d), t = fminf(s0 + kTableStep, kLimitL.hi - d);
    for (uint8_t k = 0; k < 2; ++k) {
        const float v0 = table_r(i, k), v1 = table_r(i + 1, k);
        const float bound = k == 0 ? b - d : b + d;  // lo(L) <= b - d, hi(L) >= b + d
        const bool in0 = k == 0 ? v0 <= bound : v0 >= bound, in1 = k == 0 ? v1 <= bound : v1 >= bound;
        if (in0 && in1) continue;  // the whole section (the common case: no division)
        if (!in0 && !in1) return false;
        const float x = s0 + (bound - v0) * kTableStep / (v1 - v0);  // where the line meets the bound
        if (in0) {
            t = fminf(t, x);
        } else {
            f = fmaxf(f, x);
        }
    }
    *from = f;
    *to = t;
    return f <= t;
}

}  // namespace

Range limit_r(float a) {
    float pos = (a - kTableL0) / kTableStep;
    if (!(pos > 0.0f)) pos = 0.0f;  // NaN too
    if (pos > kTablePoints - 1) pos = kTablePoints - 1;
    uint8_t i = static_cast<uint8_t>(pos);
    if (i > kTablePoints - 2) i = kTablePoints - 2;
    const float t = pos - i;
    const float lo = table_r(i, 0), hi = table_r(i, 1);
    return {lo + (table_r(i + 1, 0) - lo) * t, hi + (table_r(i + 1, 1) - hi) * t};
}

float l_reach(float a, float b, bool up, float inset) {
    constexpr float kTouch = 1e-3f;  // deg: where one section's interval must meet the next's
    const Range r = limit_r(a);
    if (!(a >= kLimitL.lo && a <= kLimitL.hi && b >= r.lo && b <= r.hi)) return a;  // NaN too
    // R's bounds in tenths, rounded outwards: an integer table value compares with them as with b -/+ inset.
    const int16_t lo10 = static_cast<int16_t>(floorf((b - inset) * 10.0f));
    const int16_t hi10 = static_cast<int16_t>(ceilf((b + inset) * 10.0f));
    uint8_t i = static_cast<uint8_t>((a - kTableL0) / kTableStep);
    if (i > kTablePoints - 2) i = kTablePoints - 2;
    float best = a, x = a;
    for (;;) {  // section by section in the direction of the move, while R stays inside its range
        const float s0 = kTableL0 + i * kTableStep, s1 = s0 + kTableStep;
        if (s0 >= kLimitL.lo + inset && s1 <= kLimitL.hi - inset && raw_r(i, 0) <= lo10 && raw_r(i + 1, 0) <= lo10 &&
            raw_r(i, 1) >= hi10 && raw_r(i + 1, 1) >= hi10) {
            best = up ? s1 : s0;  // inset inside at both points, so all along: no arithmetic (most sections)
        } else {
            float f0, t0, fd, td;
            if (!section(i, b, 0.0f, &f0, &t0) || (up ? f0 > x + kTouch : t0 < x - kTouch)) break;
            const float end = up ? t0 : f0;  // where this section's part of the path ends
            if (section(i, b, inset, &fd, &td) && (up ? fd <= end : td >= end)) {
                best = up ? fmaxf(best, fminf(td, end)) : fminf(best, fmaxf(fd, end));
            }
            if (up ? end < s1 - kTouch : end > s0 + kTouch) break;
        }
        if (up ? i == kTablePoints - 2 : i == 0) break;
        x = up ? s1 : s0;
        if (up) {
            ++i;
        } else {
            --i;
        }
    }
    return best;
}

bool legal(float a, float b, float c, float inset) {
    if (isnan(a) || isnan(b) || isnan(c)) return false;
    const auto in = [inset](float v, Range r) { return v >= r.lo + inset && v <= r.hi - inset; };
    return in(a, kLimitL) && in(c, kLimitBase) && in(b, limit_r(a));
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
