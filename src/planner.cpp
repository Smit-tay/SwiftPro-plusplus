// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jack Sidman Smith

#include "planner.h"

#include <math.h>

namespace planner {

namespace {

constexpr float kStepsPerDegree = 80.0f;
constexpr float kUnlimited = 1e30f;

Block blocks[kBlocks];
uint8_t tail = 0;   // the executing (or next to execute) block
uint8_t used = 0;   // blocks in the queue
int32_t end_pos[kAxes] = {};
Limits lim{400.0f, 300.0f, 0.2f};

// Live state of the executing block, from stepgen.
float exec_speed2 = 0.0f;
float exec_remaining = 0.0f;

uint8_t index(uint8_t i) { return static_cast<uint8_t>((tail + i) % kBlocks); }

float min2(float a, float b) { return a < b ? a : b; }

// Plans entry speeds (see planner.h). Blocks 1.. used-1 may change; block 0 is executing.
void replan() {
    if (used < 2) return;
    // Backward: each block must be able to slow down to the next one's entry; the last ends at rest.
    float next_entry2 = 0.0f;
    for (uint8_t i = static_cast<uint8_t>(used - 1); i >= 1; --i) {
        Block& b = blocks[index(i)];
        b.entry2 = min2(b.max_entry2, next_entry2 + 2.0f * b.accel * b.length);
        next_entry2 = b.entry2;
    }
    // Forward: each entry must be reachable from the previous block's entry (the executing block: from its live
    // speed over what remains of it).
    const Block& first = blocks[index(0)];
    float reachable2 = exec_speed2 + 2.0f * first.accel * exec_remaining;
    for (uint8_t i = 1; i < used; ++i) {
        Block& b = blocks[index(i)];
        if (b.entry2 > reachable2) b.entry2 = reachable2;
        reachable2 = b.entry2 + 2.0f * b.accel * b.length;
    }
}

}  // namespace

float corner_speed2(const float u[kAxes], const float v[kAxes], float a, float corner) {
    // The corner's opening angle phi is between -u and v, so cos(phi) = -(u . v) and sin(phi/2) = sqrt((1 + u.v)/2).
    // A circle of radius r tangent to both moves passes within r (1 / sin(phi/2) - 1) of the corner; setting that to
    // `corner` gives r = corner * s / (1 - s), and the speed on it at acceleration a is v^2 = a r.
    float dot = 0.0f;
    for (uint8_t i = 0; i < kAxes; ++i) dot += u[i] * v[i];
    if (dot >= 0.99999f) return kUnlimited;  // straight on
    if (dot <= -0.99999f) return 0.0f;       // reversing
    const float s = sqrtf(0.5f * (1.0f + dot));
    return a * corner * s / (1.0f - s);
}

void init(const Limits& limits) {
    lim = limits;
    clear();
    for (int32_t& p : end_pos) p = 0;
}

void set_limits(const Limits& limits) { lim = limits; }

bool full() { return used == kBlocks; }
bool empty() { return used == 0; }
uint8_t count() { return used; }

bool add(const int32_t target[kAxes], float speed) {
    if (full()) return false;
    int32_t delta[kAxes];
    float len2 = 0.0f;
    for (uint8_t i = 0; i < kAxes; ++i) {
        delta[i] = target[i] - end_pos[i];
        const float d = static_cast<float>(delta[i]) / kStepsPerDegree;
        len2 += d * d;
    }
    if (len2 == 0.0f) return true;

    Block& b = blocks[index(used)];
    b.length = sqrtf(len2);
    b.nominal = speed;
    b.accel = kUnlimited;
    for (uint8_t i = 0; i < kAxes; ++i) {
        b.steps[i] = delta[i];
        b.unit[i] = static_cast<float>(delta[i]) / kStepsPerDegree / b.length;
        const float share = fabsf(b.unit[i]);
        if (share > 0.0f) {  // no joint above its own limits
            b.nominal = min2(b.nominal, lim.joint_speed / share);
            b.accel = min2(b.accel, lim.joint_accel / share);
        }
    }

    if (used == 0) {
        b.max_entry2 = 0.0f;  // the arm is at rest
    } else {
        const Block& prev = blocks[index(static_cast<uint8_t>(used - 1))];
        const float corner2 = corner_speed2(prev.unit, b.unit, min2(prev.accel, b.accel), lim.corner);
        b.max_entry2 = min2(corner2, min2(b.nominal * b.nominal, prev.nominal * prev.nominal));
    }
    b.entry2 = b.max_entry2;
    if (used == 0) {
        exec_speed2 = 0.0f;
        exec_remaining = b.length;
    }
    ++used;
    for (uint8_t i = 0; i < kAxes; ++i) end_pos[i] = target[i];
    replan();
    return true;
}

const int32_t* end_position() { return end_pos; }

void set_end_position(const int32_t pos[kAxes]) {
    for (uint8_t i = 0; i < kAxes; ++i) end_pos[i] = pos[i];
}

Block* current() { return used == 0 ? nullptr : &blocks[tail]; }

float exit2() { return used < 2 ? 0.0f : blocks[index(1)].entry2; }

void discard() {
    if (used == 0) return;
    tail = index(1);
    --used;
    if (used > 0) {
        exec_speed2 = blocks[tail].entry2;
        exec_remaining = blocks[tail].length;
    }
}

void clear() {
    tail = 0;
    used = 0;
    exec_speed2 = 0.0f;
    exec_remaining = 0.0f;
}

void executing(float speed, float remaining) {
    exec_speed2 = speed * speed;
    exec_remaining = remaining;
}

}  // namespace planner
