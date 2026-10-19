// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jack Sidman Smith
//
// Motion planner: a queue of straight moves in joint space, and the speed at which each move may start.
//
// Units: joint positions in steps (80 per degree), lengths and speeds in degrees and degrees per second, measured as
// the Euclidean length in joint space (the original's F scale, Q-51). Every move is limited so that no single joint
// exceeds the joint acceleration or the joint speed cap.
//
// Speed profile of one move: accelerate, cruise at the nominal speed, decelerate (stepgen executes it, and smooths
// it to limit jerk, R-5). The planner only decides each move's entry speed:
//   - at most the move's nominal speed and the previous move's nominal speed;
//   - at most what the corner between the two moves allows (corner_speed2());
//   - reachable: every move can still slow down to the next move's entry speed (backward pass) and can actually
//     reach its exit speed from its entry speed (forward pass);
//   - the last move in the queue ends at rest.
// The move being executed is never changed; stepgen reports its live speed and remaining length so the forward pass
// starts from the truth.

#pragma once

#include <stdint.h>

namespace planner {

constexpr uint8_t kAxes = 3;      // 0 = L, 1 = R, 2 = base (the original's axis order)
constexpr uint8_t kBlocks = 16;   // queue length (the original: 24 of a larger struct)

struct Block {
    int32_t steps[kAxes];   // signed steps for each joint
    float length;           // joint-space length, degrees
    float unit[kAxes];      // direction, unit vector in degrees
    float nominal;          // cruise speed, deg/s
    float accel;            // acceleration along the move, deg/s^2
    float max_entry2;       // squared entry speed allowed by the corner and the nominal speeds
    float entry2;           // planned squared entry speed
};

struct Limits {
    float joint_accel;  // deg/s^2, per joint (M204, Q-52)
    float joint_speed;  // deg/s, per joint (step-rate cap, Q-25)
    float corner;       // corner tolerance in degrees of joint space (see corner_speed2)
};

void init(const Limits& limits);
void set_limits(const Limits& limits);

bool full();
bool empty();
uint8_t count();

// Adds a move from the end of the queue (or the current position when empty) to target. speed in deg/s.
// Returns false if the queue is full; a zero-length move is accepted and ignored.
bool add(const int32_t target[kAxes], float speed);

// The planned end position of the queue: where the next move starts.
const int32_t* end_position();
void set_end_position(const int32_t pos[kAxes]);  // only while empty (after a stop or a re-sync)

// The move to execute (nullptr if none), and the squared speed at which it must end.
Block* current();
float exit2();
void discard();  // the current move is done
void clear();    // drop everything (S1100); the end position becomes whatever the caller sets next

// stepgen calls this every segment: the executing move's live speed and remaining length, for the forward pass.
void executing(float speed, float remaining);

// The squared speed at which a corner between directions u (in) and v (out) can be taken: a circle tangent to both
// moves that stays within `corner` of the corner point, at acceleration a. Straight on: unlimited; reversing: 0.
float corner_speed2(const float u[kAxes], const float v[kAxes], float a, float corner);

}  // namespace planner
