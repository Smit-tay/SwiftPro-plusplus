// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jack Sidman Smith

#include "stepgen.h"

#include <math.h>

namespace stepgen {

namespace {

using planner::kAxes;

int32_t block_start[kAxes] = {};  // position (steps) where the executing move started
bool active = false;              // a move is in progress
float done = 0.0f;                // distance covered in it, degrees
float v = 0.0f;                   // speed, deg/s
bool hold_on = false;

// Braking after a stop (Q-60): positions follow p(t) = v0 (t - t^2 / (2 T)) from brake_from, until T.
bool braking = false;
int32_t brake_from[kAxes] = {};
float brake_v0[kAxes] = {};  // steps/s
float brake_t = 0.0f, brake_T = 0.0f;

// The moving average, in exact integers: positions in 1/256 step, and their running sums.
constexpr int32_t kFix = 256;
int32_t window[kSmoothing][kAxes] = {};
int32_t window_sum[kAxes] = {};
uint8_t window_pos = 0;
uint8_t stable = 0;                    // segments in a row whose position didn't change
int32_t sent[kAxes] = {};              // steps sent so far

// sum / (kSmoothing * kFix), rounded to the nearest step.
int32_t mean_steps(int32_t sum) {
    constexpr int32_t d = kSmoothing * kFix;
    return sum >= 0 ? (sum + d / 2) / d : -((-sum + d / 2) / d);
}

float sqrt0(float x) { return x > 0.0f ? sqrtf(x) : 0.0f; }

// Advances the executing move by up to t seconds; returns the time used (less than t only if the move ended).
float advance(const planner::Block& b, float t) {
    const float rem = b.length - done;
    if (rem <= 1e-6f) {
        done = b.length;
        return 0.0f;
    }
    const float a = b.accel;

    if (hold_on) {  // decelerate along the path at a, across the end of the move if need be
        if (v <= 0.0f) {
            v = 0.0f;
            return t;
        }
        const float stop_distance = v * v / (2.0f * a);
        if (stop_distance <= rem) {
            const float dt = t < v / a ? t : v / a;
            done += v * dt - 0.5f * a * dt * dt;
            v -= a * dt;
            if (v < 1e-4f) v = 0.0f;
            return t;
        }
        const float tau = (v - sqrt0(v * v - 2.0f * a * rem)) / a;  // time to reach the end of the move
        if (tau >= t) {
            done += v * t - 0.5f * a * t * t;
            v -= a * t;
            return t;
        }
        done = b.length;
        v -= a * tau;
        return tau;
    }

    float ve = sqrt0(planner::exit2());
    if (ve > b.nominal) ve = b.nominal;
    // Peak speed: the highest speed from which the move can still slow to ve at a, capped by the nominal speed.
    float vp = sqrt0((2.0f * a * rem + v * v + ve * ve) * 0.5f);
    if (vp > b.nominal) vp = b.nominal;
    float ad = a;                // deceleration
    if (v > vp) {                // faster than the plan allows (rounding, a slower exit than expected):
        vp = v;                  // brake now, a little harder than a if necessary, to end exactly at ve
        ad = (v * v - ve * ve) / (2.0f * rem);
        if (ad < a) ad = a;
    }
    const float ta = (vp - v) / a;
    const float da = (vp * vp - v * v) / (2.0f * a);
    const float dd = (vp * vp - ve * ve) / (2.0f * ad);
    float dc = rem - da - dd;
    if (dc < 0.0f) dc = 0.0f;
    const float tc = vp > 0.0f ? dc / vp : 0.0f;
    const float td = (vp - ve) / ad;

    if (t <= ta) {  // accelerating
        done += v * t + 0.5f * a * t * t;
        v += a * t;
        return t;
    }
    done += da;
    v = vp;
    float left = t - ta;
    if (left <= tc) {  // cruising
        done += vp * left;
        return t;
    }
    done += dc;
    left -= tc;
    if (left < td) {  // decelerating
        done += vp * left - 0.5f * ad * left * left;
        v = vp - ad * left;
        if (done > b.length) done = b.length;
        return t;
    }
    done = b.length;  // the move ends inside this segment
    v = ve;
    return ta + tc + td;
}

// At rest and the average has caught up: the last kSmoothing positions are all the same.
bool settled() { return stable >= kSmoothing; }

}  // namespace

void init(const int32_t position[kAxes]) {
    for (uint8_t i = 0; i < kAxes; ++i) {
        block_start[i] = position[i];
        sent[i] = position[i];
        for (uint8_t k = 0; k < kSmoothing; ++k) window[k][i] = position[i] * kFix;
        window_sum[i] = position[i] * kFix * kSmoothing;
    }
    window_pos = 0;
    stable = kSmoothing;
    active = false;
    done = 0.0f;
    v = 0.0f;
    hold_on = false;
}

bool next(Segment* out) {
    if (braking) {
        brake_t += kSegmentS;
        const bool last = brake_t >= brake_T;
        const float t = last ? brake_T : brake_t;
        bool moved = false;
        int32_t at[kAxes];
        for (uint8_t i = 0; i < kAxes; ++i) {
            at[i] = brake_from[i] + static_cast<int32_t>(lroundf(brake_v0[i] * (t - t * t / (2.0f * brake_T))));
            out->steps[i] = static_cast<int16_t>(at[i] - sent[i]);
            moved = moved || at[i] != sent[i];
            sent[i] = at[i];
        }
        if (last) {  // at rest: everything starts from here
            braking = false;
            planner::set_end_position(at);
            init(at);
        }
        return moved || !last;
    }
    float t = kSegmentS;
    while (t > 0.0f) {
        planner::Block* b = planner::current();
        if (b == nullptr) {
            v = 0.0f;
            break;
        }
        if (!active) {
            // A move that starts from rest waits until the smoothing has settled on the exact stop position, so a
            // full stop (a reversal, the end of a streamed move) really reaches its target before turning back.
            if (v == 0.0f && !settled()) break;
            active = true;
            done = 0.0f;
        }
        const float used = advance(*b, t);
        t -= used;
        planner::executing(v, b->length - done);
        if (done >= b->length) {
            for (uint8_t i = 0; i < kAxes; ++i) block_start[i] += b->steps[i];
            planner::discard();
            active = false;
            done = 0.0f;
        } else {
            break;  // the segment ended inside the move (or a hold stopped it)
        }
    }

    // Where the motion is now, then smoothed.
    const planner::Block* b = planner::current();
    int32_t* slot = window[window_pos];
    window_pos = static_cast<uint8_t>((window_pos + 1) % kSmoothing);
    bool same = true;
    bool moved = false;
    for (uint8_t i = 0; i < kAxes; ++i) {
        int32_t p = block_start[i] * kFix;
        if (active && b != nullptr) p += static_cast<int32_t>(lroundf(static_cast<float>(b->steps[i]) * (done / b->length) * kFix));
        const int32_t previous = window[(window_pos + kSmoothing - 2) % kSmoothing][i];
        same = same && p == previous;
        window_sum[i] += p - slot[i];
        slot[i] = p;
        const int32_t target = mean_steps(window_sum[i]);
        const int32_t d = target - sent[i];
        out->steps[i] = static_cast<int16_t>(d);
        sent[i] = target;
        moved = moved || d != 0;
    }
    stable = same ? (stable < kSmoothing ? static_cast<uint8_t>(stable + 1) : stable) : 0;
    return moved || planner::current() != nullptr || !settled();
}

void hold(bool on) { hold_on = on; }

bool holding() { return hold_on; }

void stop(const int32_t at[kAxes], const int16_t last[kAxes], float brake) {
    planner::clear();
    planner::set_end_position(at);
    init(at);
    float peak = 0.0f;  // the fastest joint, deg/s
    for (uint8_t i = 0; i < kAxes; ++i) {
        brake_v0[i] = static_cast<float>(last[i]) / kSegmentS;
        const float deg = fabsf(brake_v0[i]) / 80.0f;
        if (deg > peak) peak = deg;
        brake_from[i] = at[i];
    }
    if (peak <= 0.0f || brake <= 0.0f) return;  // already at rest
    brake_T = peak / brake;  // the fastest joint stops at exactly `brake`
    brake_t = 0.0f;
    braking = true;
}

bool idle() { return !braking && planner::current() == nullptr && settled(); }

const int32_t* position() { return sent; }

}  // namespace stepgen
