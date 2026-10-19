// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jack Sidman Smith
//
// Segment generator: turns the planner's moves into 5 ms segments of whole steps for the stepper ISR.
//
// Each segment advances the executing move along an accelerate / cruise / decelerate profile, recomputed every
// segment from the live speed, the remaining length, the move's nominal speed and the planned exit speed (so a move
// added later can still raise the exit speed). The joint positions this produces are then smoothed by a moving
// average over the last 8 segments (40 ms): acceleration builds up over 40 ms instead of jumping, which limits jerk
// (R-5) without changing where moves end: at rest the average equals the exact target, so final step counts are exact.
// A move that starts from rest waits for the average to settle first, so full stops reach their target exactly;
// corners taken at speed are rounded, by at most the corner tolerance plus the smoothing.
//
// Feed hold (S1000 V0) decelerates along the path, across move boundaries if needed, and waits; resume continues.
// Stop (S1100, Ctrl-X) drops everything and brakes to rest from where the motors actually are (Q-60).

#pragma once

#include <stdint.h>

#include "planner.h"

namespace stepgen {

constexpr uint16_t kSegmentUs = 5000;  // segment length
constexpr float kSegmentS = kSegmentUs * 1e-6f;
constexpr uint8_t kSmoothing = 8;      // segments in the moving average (40 ms)

struct Segment {
    int16_t steps[planner::kAxes];  // signed steps in this segment
};

void init(const int32_t position[planner::kAxes]);  // at rest at position (steps)

// Produces the next segment. Returns false when there is nothing to do (at rest, all steps sent).
bool next(Segment* out);

void hold(bool on);   // S1000 V0 / V1
bool holding();       // a hold is in effect (possibly still decelerating)
// S1100 / Ctrl-X: drop the queue and brake from where the motors are (at, the ISR's tally) and how fast they were
// going (last, steps per 5 ms segment) to rest, all joints together along their current direction, so no joint
// decelerates faster than brake deg/s^2 (Q-60). No smoothing: the stop comes as soon as the deceleration allows.
void stop(const int32_t at[planner::kAxes], const int16_t last[planner::kAxes], float brake);

bool idle();          // at rest and nothing queued
const int32_t* position();  // steps sent so far (what the motors have been told)

}  // namespace stepgen
