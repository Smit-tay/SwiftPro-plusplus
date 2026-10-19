// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jack Sidman Smith
//
// Planner and segment generator, run on the host: exact end positions, limits, smoothing, corners, hold and stop.

#include "../src/planner.h"
#include "../src/stepgen.h"
#include "../src/stepper.h"
#include "check.h"

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <vector>

namespace {

using planner::kAxes;

constexpr float kAccel = 400.0f;  // deg/s^2 per joint (A5)
constexpr float kSpeed = 300.0f;  // deg/s per joint (step-rate cap)

struct Run {
    std::vector<stepgen::Segment> segs;
    int32_t pos[kAxes] = {};
};

// Runs the generator to rest (or n segments), adding the steps up.
void run(Run& r, int max_segments = 100000) {
    stepgen::Segment s;
    for (int k = 0; k < max_segments && stepgen::next(&s); ++k) {
        r.segs.push_back(s);
        for (uint8_t i = 0; i < kAxes; ++i) r.pos[i] += s.steps[i];
    }
}

void reset(const int32_t at[kAxes]) {
    planner::init({kAccel, kSpeed, 0.2f});
    planner::set_end_position(at);
    stepgen::init(at);
}

// Speed of the fastest joint in a segment, deg/s.
float joint_speed(const stepgen::Segment& s, uint8_t i) {
    return static_cast<float>(std::abs(s.steps[i])) / 80.0f / stepgen::kSegmentS;
}

}  // namespace

void test_motion() {
    const int32_t origin[kAxes] = {0, 0, 0};

    // One move lands exactly, within the speed and acceleration limits.
    reset(origin);
    const int32_t t1[kAxes] = {7200, -3600, 1234};  // 90, -45, 15.4 deg
    CHECK(planner::add(t1, 1000.0f * 80.0f / 60.0f));  // F1000 in the original's scale
    Run r;
    run(r);
    CHECK(r.pos[0] == 7200 && r.pos[1] == -3600 && r.pos[2] == 1234);
    CHECK(stepgen::idle());
    float vmax = 0.0f, amax = 0.0f;
    constexpr size_t kWin = 10;  // acceleration over 50 ms, so whole-step rounding doesn't dominate
    for (size_t k = 0; k < r.segs.size(); ++k) {
        for (uint8_t i = 0; i < kAxes; ++i) {
            vmax = std::fmax(vmax, joint_speed(r.segs[k], i));
            if (k >= kWin) {
                float v0 = 0.0f, v1 = 0.0f;  // mean speeds of two windows, kWin segments apart
                for (size_t j = 0; j < kWin / 2; ++j) {
                    v0 += static_cast<float>(r.segs[k - kWin + j].steps[i]);
                    v1 += static_cast<float>(r.segs[k - kWin / 2 + j].steps[i]);
                }
                const float dv = std::fabs(v1 - v0) / (kWin / 2) / 80.0f / stepgen::kSegmentS;
                amax = std::fmax(amax, dv / (kWin / 2 * stepgen::kSegmentS));
            }
        }
    }
    CHECK(vmax <= kSpeed * 1.02f);
    std::printf("motion: peak joint speed %.1f deg/s, peak joint accel %.1f deg/s^2\n", static_cast<double>(vmax),
                static_cast<double>(amax));
    CHECK(amax <= kAccel * 1.1f);  // one step of rounding in the 50 ms window is about 5 %
    // Jerk: with the 40 ms moving average, acceleration builds up over at least 8 segments.
    CHECK(r.segs.size() > 16);
    CHECK(std::abs(r.segs[0].steps[0]) <= 2);

    // Two collinear moves: no stop at the joint between them.
    reset(origin);
    const int32_t a1[kAxes] = {4000, 0, 0}, a2[kAxes] = {8000, 0, 0};
    CHECK(planner::add(a1, 200.0f));
    CHECK(planner::add(a2, 200.0f));
    Run r2;
    run(r2);
    CHECK(r2.pos[0] == 8000);
    int zero_mid = 0;
    for (size_t k = 10; k + 10 < r2.segs.size(); ++k) zero_mid += r2.segs[k].steps[0] == 0;
    CHECK(zero_mid == 0);

    // A reversal must stop in between: the planner allows no speed at the corner.
    reset(origin);
    const int32_t b1[kAxes] = {4000, 0, 0}, b2[kAxes] = {0, 0, 0};
    CHECK(planner::add(b1, 200.0f));
    CHECK(planner::add(b2, 200.0f));
    Run r3;
    run(r3);
    CHECK(r3.pos[0] == 0);
    int32_t peak = 0, p = 0;
    for (const auto& s : r3.segs) {
        p += s.steps[0];
        if (p > peak) peak = p;
    }
    CHECK(peak == 4000);  // reached the corner exactly, no overshoot

    // A 90 degree corner slows down but doesn't stop.
    reset(origin);
    const int32_t c1[kAxes] = {4000, 0, 0}, c2[kAxes] = {4000, 4000, 0};
    CHECK(planner::add(c1, 200.0f));
    CHECK(planner::add(c2, 200.0f));
    Run r4;
    run(r4);
    CHECK(r4.pos[0] == 4000 && r4.pos[1] == 4000);

    // Hold mid-move: it stops, nothing more moves while held, and resume finishes exactly.
    reset(origin);
    const int32_t h1[kAxes] = {16000, 0, 0};
    CHECK(planner::add(h1, 200.0f));
    Run r5;
    run(r5, 60);  // 0.3 s in
    stepgen::hold(true);
    run(r5, 200);
    const int32_t held_at = r5.pos[0];
    CHECK(held_at > 0 && held_at < 16000);
    stepgen::Segment s;
    CHECK(stepgen::next(&s) && s.steps[0] == 0);  // still has work, but held
    r5.pos[0] += s.steps[0];
    stepgen::hold(false);
    run(r5);
    CHECK(r5.pos[0] == 16000);

    // Stop: drops the queue; motion restarts from where the motors are.
    reset(origin);
    CHECK(planner::add(h1, 200.0f));
    Run r6;
    run(r6, 40);
    const int32_t at[kAxes] = {r6.pos[0], 0, 0};
    const int16_t last[kAxes] = {r6.segs.back().steps[0], 0, 0};
    stepgen::stop(at, last, 3.0f * kAccel);  // Q-60: brake at 3x the acceleration
    CHECK(planner::empty());
    const int32_t before = r6.pos[0];
    run(r6);  // braking: keeps going the same way, slowing to rest
    CHECK(stepgen::idle());
    const float v0 = static_cast<float>(last[0]) / stepgen::kSegmentS / 80.0f;  // deg/s
    const float expect = v0 * v0 / (2.0f * 3.0f * kAccel) * 80.0f;              // braking distance, steps
    CHECK(std::fabs(static_cast<float>(r6.pos[0] - before) - expect) <= 3.0f);
    CHECK(!stepgen::next(&s));
    const int32_t back[kAxes] = {0, 0, 0};
    CHECK(planner::add(back, 200.0f));
    run(r6);
    CHECK(r6.pos[0] == 0);

    // The corner speed formula: straight on is unlimited, reversing is 0, in between grows with the opening angle.
    const float ux[kAxes] = {1, 0, 0}, uy[kAxes] = {0, 1, 0}, ud[kAxes] = {0.7071068f, 0.7071068f, 0},
                un[kAxes] = {-1, 0, 0};
    CHECK(planner::corner_speed2(ux, ux, 400.0f, 0.2f) > 1e20f);
    CHECK(planner::corner_speed2(ux, un, 400.0f, 0.2f) == 0.0f);
    CHECK(planner::corner_speed2(ux, uy, 400.0f, 0.2f) < planner::corner_speed2(ux, ud, 400.0f, 0.2f));

    // Many short collinear moves (a G1 line cut into segments) keep cruising.
    reset(origin);
    int32_t tgt[kAxes] = {0, 0, 0};
    Run r7;
    for (int k = 1; k <= 40; ++k) {
        tgt[0] = k * 200;
        tgt[1] = k * 100;
        while (!planner::add(tgt, 150.0f)) {  // queue full: let the generator run a little
            stepgen::Segment seg;
            stepgen::next(&seg);
            r7.segs.push_back(seg);
            for (uint8_t i = 0; i < kAxes; ++i) r7.pos[i] += seg.steps[i];
        }
    }
    run(r7);
    CHECK(r7.pos[0] == 8000 && r7.pos[1] == 4000);
    int stalls = 0;
    for (size_t k = 12; k + 12 < r7.segs.size(); ++k) stalls += r7.segs[k].steps[0] == 0;
    CHECK(stalls == 0);

    // The stepper ISR plays segments exactly: the tally matches, and a segment lasts 10000 ticks (5 ms).
    const int32_t zero[kAxes] = {0, 0, 0};
    stepper::init(zero);
    stepgen::Segment q{{120, -37, 0}};
    CHECK(stepper::queue(q));
    stepgen::Segment q2{{1, 0, -5}};
    CHECK(stepper::queue(q2));
    stepgen::Segment too_many{{200, 0, 0}};
    CHECK(!stepper::queue(too_many));  // above the per-segment cap: refused, not truncated
    stepper::host_advance(100 + 10000 - 1);  // the first ISR loads after 100 ticks; then one segment
    int32_t tally[kAxes];
    stepper::position(tally);
    CHECK(tally[0] == 119 || tally[0] == 120);  // the last event of segment 1 pulses at the next ISR
    stepper::host_advance(20000);
    stepper::position(tally);
    CHECK(tally[0] == 121 && tally[1] == -37 && tally[2] == -5);
    CHECK(!stepper::running());
    // A full run through stepgen and the ISR lands on the target.
    reset(origin);
    stepper::init(origin);
    const int32_t far[kAxes] = {-5000, 3000, 8000};
    CHECK(planner::add(far, 250.0f));
    stepgen::Segment seg;
    for (int k = 0; k < 4000; ++k) {
        while (stepper::room() && stepgen::next(&seg)) CHECK(stepper::queue(seg));
        stepper::host_advance(2000);  // 1 ms
    }
    stepper::position(tally);
    CHECK(tally[0] == -5000 && tally[1] == 3000 && tally[2] == 8000);
}
