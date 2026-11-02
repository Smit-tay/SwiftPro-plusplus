// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jack Sidman Smith

#include "motion.h"

#include "nvm.h"
#include "planner.h"
#include "stepgen.h"
#include "stepper.h"

#include <math.h>

namespace motion {

namespace {

constexpr float kStepsPerDegree = kin::kStepsPerDegree;
constexpr float kUnitsToDegPerS = 80.0f / 60.0f;  // F is 80 deg/min of joint-space motion per unit (Q-51)
constexpr float kDefaultF = 100.0f;               // Q-55
constexpr float kDefaultA = 5.0f;                 // Q-52: 400 deg/s^2 per joint
constexpr float kMaxA = 50.0f;                    // the largest value you had stored ($120-$122)
constexpr float kJointSpeed = 300.0f;             // deg/s per joint: 24 kHz, inside the ISR's 32 kHz (Q-25)
constexpr float kCorner = 0.2f;                   // corner tolerance, degrees of joint space
constexpr float kBrakeFactor = 3.0f;              // S1100 brakes at 3x the acceleration limit (Q-60; to tune)
constexpr uint16_t kAccelAddr = 960;              // EEPROM: float A, then the marker byte (a free area)
constexpr uint16_t kAccelMarkAddr = 964;
constexpr uint8_t kAccelMark = 0xA5;

// Linear moves: each piece's joint-space chord stays within this of the true path (in degrees of each joint, about
// 0.1 mm at full reach).
constexpr float kLineTolerance = 0.02f;
constexpr float kPieceMax = 20.0f;   // mm
constexpr float kPieceMin = 0.25f;   // mm
constexpr float kJogReach = 1000.0f;   // mm: farther than the arm reaches, the start of a linear jog's search
constexpr float kJogResolution = 0.1f;  // mm: how close the search comes to the edge

float init_angle[3] = {};  // L, R, B (user frame) at tally 0
uint8_t known_mask = 0;
uint8_t locked_mask = 0;   // bit per axis (0 L, 1 R, 2 B)
float accel_a = kDefaultA;
kin::Offsets offs{56.65f, 74.55f};

// A linear move: first checked piece by piece (nothing moves), then cut into the planner piece by piece. Both passes
// run the same cutting, one piece per main-loop pass.
// A linear jog first walks out along its axis to find the edge (kSearching), then is checked and run as any line.
enum class Phase : uint8_t { kIdle, kSearching, kChecking, kRunning };
struct Line {
    Phase phase = Phase::kIdle;
    Result result = Result::kOk;
    float start_joints[3];
    float from[3], to[3];  // effector frame, mm
    float length = 0.0f;   // mm
    float done = 0.0f;     // mm cut so far
    float piece = 5.0f;    // the last piece length, mm
    float joints[3];       // joint angles (L, R, B in the -90 frame) at `done`
    float speed = 0.0f;    // deg/s
    float inset = 0.0f;    // deg, the limits are shrunk by this (jogs)
    float edge = 0.0f;     // kSearching: 0 while walking out, then the first illegal distance found, mm
};
Line line;

void limits() { planner::set_limits({accel_a * 80.0f, kJointSpeed, kCorner}); }

// Joint angles (L, R, B in the -90 frame) <-> tally steps.
void to_steps(const float j[3], int32_t s[3]) {
    s[0] = kin::arm_steps(j[0], init_angle[0]);
    s[1] = kin::arm_steps(j[1], init_angle[1]);
    s[2] = kin::base_steps(j[2], init_angle[2]);
}
void to_angles(const int32_t s[3], float j[3]) {
    j[0] = kin::arm_angle(static_cast<float>(s[0]), init_angle[0]);
    j[1] = kin::arm_angle(static_cast<float>(s[1]), init_angle[1]);
    j[2] = kin::base_angle(static_cast<float>(s[2]), init_angle[2]);
}

void effector_of(const float j[3], float xyz[3]) {
    kin::forward(j[0], j[1], j[2], &xyz[0], &xyz[1], &xyz[2]);
    kin::arm_to_effector(offs, &xyz[0], &xyz[1], &xyz[2]);
}

// Effector point -> joints; false if out of reach or out of the joint limits (shrunk by inset).
bool joints_of(const float xyz[3], float j[3], float inset = 0.0f) {
    float x = xyz[0], y = xyz[1], z = xyz[2];
    kin::effector_to_arm(offs, &x, &y, &z);
    kin::inverse(x, y, z, &j[0], &j[1], &j[2]);
    return kin::legal(j[0], j[1], j[2], inset);
}

float speed_of(float f) { return f * kUnitsToDegPerS; }

bool usable_f(float f) { return !isnan(f) && f > 0.0f; }  // SP-09

// Every move needs all three joints locked, whichever it turns (Q-70, as V4.10.0 motion_control.c:69): a released
// joint doesn't hold the position the move starts from.
bool all_locked() { return locked_mask == 7; }

Result queue_joints(const float j[3], float f) {
    if (!kin::legal(j[0], j[1], j[2])) return Result::kUnreachable;
    int32_t target[3];
    to_steps(j, target);
    planner::add(target, speed_of(f));  // the caller checked busy(): there is room
    return Result::kOk;
}

// One piece of a linear move from line.done: its length (adapted to the path's curvature) and end joints.
// Returns false if the path leaves the joint limits; *len is then the length that was tried.
bool next_piece(float* len, float end_joints[3]) {
    float l = line.piece * 1.5f;
    if (l > kPieceMax) l = kPieceMax;
    const float rest = line.length - line.done;
    for (;;) {
        if (l > rest) l = rest;
        float p1[3], pm[3], j1[3], jm[3];
        for (uint8_t i = 0; i < 3; ++i) {
            const float u = (line.to[i] - line.from[i]) / line.length;
            p1[i] = line.from[i] + u * (line.done + l);
            pm[i] = line.from[i] + u * (line.done + 0.5f * l);
        }
        if (!joints_of(p1, j1, line.inset) || !joints_of(pm, jm, line.inset)) {
            *len = l;
            return false;
        }
        float err = 0.0f;
        for (uint8_t i = 0; i < 3; ++i) {
            const float e = fabsf(jm[i] - 0.5f * (line.joints[i] + j1[i]));
            if (e > err) err = e;
        }
        if (err <= kLineTolerance || l <= kPieceMin) {
            *len = l;
            for (uint8_t i = 0; i < 3; ++i) end_joints[i] = j1[i];
            return true;
        }
        l *= 0.5f;
    }
}

void start_line(const float from[3], const float to[3], float f, const float joints[3]) {
    float d2 = 0.0f;
    for (uint8_t i = 0; i < 3; ++i) {
        line.from[i] = from[i];
        line.to[i] = to[i];
        line.joints[i] = joints[i];
        line.start_joints[i] = joints[i];
        d2 += (to[i] - from[i]) * (to[i] - from[i]);
    }
    line.length = sqrtf(d2);
    line.done = 0.0f;
    line.piece = 5.0f;
    line.speed = speed_of(f);
    line.inset = 0.0f;
}

Result do_line(const float to[3], float f) {
    float from[3], j0[3];
    end_angles(j0);
    effector_of(j0, from);
    float d2 = 0.0f;
    for (uint8_t i = 0; i < 3; ++i) d2 += (to[i] - from[i]) * (to[i] - from[i]);
    if (d2 == 0.0f) return Result::kOk;
    start_line(from, to, f, j0);
    line.phase = Phase::kChecking;
    line.result = Result::kChecking;
    return Result::kChecking;
}

// One piece of the check: true while there is more to check.
void check_step() {
    float l, j[3];
    if (!next_piece(&l, j)) {
        line.phase = Phase::kIdle;
        line.result = Result::kUnreachable;
        return;
    }
    line.done += l;
    line.piece = l;
    for (uint8_t i = 0; i < 3; ++i) line.joints[i] = j[i];
    if (line.done >= line.length) {  // all of it is reachable: start again from the beginning, for real
        line.done = 0.0f;
        line.piece = 5.0f;
        for (uint8_t i = 0; i < 3; ++i) line.joints[i] = line.start_joints[i];
        line.phase = Phase::kRunning;
        line.result = Result::kOk;
    }
}

// One step of a linear jog's search: walk out piece by piece to the first illegal piece, then halve the gap between
// the last legal point and it. The line is then cut back to the edge and checked from the start.
void search_step() {
    if (line.edge == 0.0f) {
        float l, j[3];
        if (!next_piece(&l, j)) {
            line.edge = line.done + l;
            return;
        }
        line.done += l;
        line.piece = l;
        for (uint8_t i = 0; i < 3; ++i) line.joints[i] = j[i];
        if (line.done < line.length) return;
        line.edge = line.length;  // never left the limits (can't happen: kJogReach is beyond reach)
    } else if (line.edge - line.done > kJogResolution) {
        const float mid = 0.5f * (line.done + line.edge);
        float p[3], j[3];
        for (uint8_t i = 0; i < 3; ++i) p[i] = line.from[i] + (line.to[i] - line.from[i]) / line.length * mid;
        if (joints_of(p, j, line.inset)) line.done = mid;
        else line.edge = mid;
        return;
    }
    if (line.done < kJogMin) {
        line.phase = Phase::kIdle;
        line.result = Result::kUnreachable;
        return;
    }
    for (uint8_t i = 0; i < 3; ++i) line.to[i] = line.from[i] + (line.to[i] - line.from[i]) / line.length * line.done;
    line.length = line.done;
    line.done = 0.0f;
    line.piece = 5.0f;
    for (uint8_t i = 0; i < 3; ++i) line.joints[i] = line.start_joints[i];
    line.phase = Phase::kChecking;
}

}  // namespace

void init() {
    float a = 0.0f;
    nvm::read(kAccelAddr, &a, sizeof a);
    accel_a = nvm::read_byte(kAccelMarkAddr) == kAccelMark && a > 0.0f && a <= kMaxA ? a : kDefaultA;
    known_mask = 0;
    locked_mask = 0;
    line.phase = Phase::kIdle;
    const int32_t zero[3] = {0, 0, 0};
    planner::init({accel_a * 80.0f, kJointSpeed, kCorner});
    stepgen::init(zero);
    stepper::init(zero);
    for (float& f : init_angle) f = 0.0f;
}

void sync(uint8_t mask, const float angles[3]) {
    // Only at rest: re-base the given joints on their measured angles (Q-6, M17), keep the others.
    int32_t tally[3];
    stepper::position(tally);
    for (uint8_t i = 0; i < 3; ++i) {
        if ((mask & (1u << i)) == 0) continue;
        init_angle[i] = angles[i];
        tally[i] = 0;
        known_mask = static_cast<uint8_t>(known_mask | (1u << i));
        locked_mask = static_cast<uint8_t>(locked_mask | (1u << i));
        stepper::enable(i, true);
    }
    stepper::set_position(tally);
    planner::set_end_position(tally);
    stepgen::init(tally);
}

bool known() { return known_mask == 7; }

uint8_t known_joints() { return known_mask; }

void hold_all() {
    locked_mask = 7;
    for (uint8_t i = 0; i < 3; ++i) stepper::enable(i, true);
}

bool busy() { return line.phase != Phase::kIdle || planner::full(); }

Result line_result() { return line.result; }

bool poll() {
    bool work = false;
    // A linear move: one piece per pass (each costs two IK solutions).
    if (line.phase == Phase::kSearching) {
        search_step();
        work = true;
    } else if (line.phase == Phase::kChecking) {
        check_step();
        work = true;
    } else if (line.phase == Phase::kRunning && !planner::full()) {
        work = true;
        float l, j[3];
        if (line.done >= line.length || !next_piece(&l, j)) {
            line.phase = Phase::kIdle;  // checked before it started, so this can only be the end
        } else {
            line.done += l;
            line.piece = l;
            for (uint8_t i = 0; i < 3; ++i) line.joints[i] = j[i];
            int32_t target[3];
            to_steps(j, target);
            planner::add(target, line.speed);
            if (line.done >= line.length) line.phase = Phase::kIdle;
        }
    }
    // Keep the stepper's ring full.
    stepgen::Segment s;
    while (stepper::room() && stepgen::next(&s)) {
        stepper::queue(s);
        work = true;
    }
    return work;
}

bool idle() { return line.phase == Phase::kIdle && stepgen::idle() && !stepper::running(); }

Result move_joints(const float a[3], float f) {
    if (!known()) return Result::kNoPosition;
    if (!usable_f(f)) return Result::kBadParam;
    if (!all_locked()) return Result::kReleased;
    float j[3];
    end_angles(j);
    if (!isnan(a[0])) j[0] = a[0];
    if (!isnan(a[1])) j[1] = a[1];
    if (!isnan(a[2])) j[2] = a[2] - 90.0f;
    return queue_joints(j, f);
}

Result move_xyz(const float xyz[3], bool linear, float f) {
    if (!known()) return Result::kNoPosition;
    if (!usable_f(f)) return Result::kBadParam;
    if (!all_locked()) return Result::kReleased;
    float j0[3], here[3], to[3];
    end_angles(j0);
    effector_of(j0, here);
    for (uint8_t i = 0; i < 3; ++i) to[i] = isnan(xyz[i]) ? here[i] : xyz[i];  // Q-54
    if (linear) return do_line(to, f);
    float j[3];
    if (!joints_of(to, j)) return Result::kUnreachable;
    return queue_joints(j, f);
}

Result move_xyz_relative(const float d[3], float f) {
    if (!known()) return Result::kNoPosition;
    if (!usable_f(f)) return Result::kBadParam;
    if (!all_locked()) return Result::kReleased;
    float j0[3], to[3];
    end_angles(j0);
    effector_of(j0, to);
    for (uint8_t i = 0; i < 3; ++i) to[i] += d[i];
    return do_line(to, f);
}

Result move_polar(float s, float r, float h, bool relative, float f) {
    if (!known()) return Result::kNoPosition;
    if (!usable_f(f)) return Result::kBadParam;
    if (!all_locked()) return Result::kReleased;
    float to[3];
    if (relative) {  // from where the queued motion ends: radius + s, heading + r, height + h (uarm_protocol.c:347-366)
        float j0[3], here[3];
        end_angles(j0);
        effector_of(j0, here);
        const float length = sqrtf(here[0] * here[0] + here[1] * here[1]) + s;
        const float heading = (j0[2] + r) / 57.29578f;
        to[0] = length * cosf(heading);
        to[1] = length * sinf(heading);
        to[2] = here[2] + h;
    } else {  // R in the user frame (90 = straight ahead) (uarm_protocol.c:259-262)
        const float heading = (r - 90.0f) / 57.29578f;
        to[0] = s * cosf(heading);
        to[1] = s * sinf(heading);
        to[2] = h;
    }
    return do_line(to, f);
}

Result jog_joint(uint8_t axis, float f) {
    if (!known()) return Result::kNoPosition;
    const float speed = fabsf(f);
    if (!usable_f(speed)) return Result::kBadParam;
    if (!all_locked()) return Result::kReleased;
    float j[3];
    end_angles(j);
    const bool up = f > 0.0f;
    float target;
    if (axis == 2) {
        target = up ? kin::kLimitBase.hi - kJogInset : kin::kLimitBase.lo + kJogInset;
    } else if (axis == 1) {  // R's range at this L
        const kin::Range r = kin::limit_r(j[0]);
        target = up ? r.hi - kJogInset : r.lo + kJogInset;
    } else {  // R's range moves with L: as far as R stays inside it
        target = kin::l_reach(j[0], j[1], up, kJogInset);
    }
    if ((up ? target - j[axis] : j[axis] - target) < kJogMin) return Result::kUnreachable;
    j[axis] = target;
    return queue_joints(j, speed);
}

Result jog_xyz(uint8_t axis, float f) {
    if (!known()) return Result::kNoPosition;
    const float speed = fabsf(f);
    if (!usable_f(speed)) return Result::kBadParam;
    if (!all_locked()) return Result::kReleased;
    float j0[3], from[3], to[3];
    end_angles(j0);
    effector_of(j0, from);
    for (uint8_t i = 0; i < 3; ++i) to[i] = from[i];
    to[axis] += f > 0.0f ? kJogReach : -kJogReach;
    start_line(from, to, speed, j0);
    // Already within kJogInset of a limit: search with the limits themselves, so the jog can still move away from it.
    line.inset = kin::legal(j0[0], j0[1], j0[2], kJogInset) ? kJogInset : 0.0f;
    line.edge = 0.0f;
    line.phase = Phase::kSearching;
    line.result = Result::kChecking;
    return Result::kChecking;
}

Result into_limits(float f) {
    if (!known()) return Result::kNoPosition;
    if (!usable_f(f)) return Result::kBadParam;
    if (!all_locked()) return Result::kReleased;
    float j[3];
    end_angles(j);
    if (kin::legal(j[0], j[1], j[2])) return Result::kOk;
    const auto inside = [](float v, kin::Range r) {
        return v < r.lo ? r.lo + kSettleInset : v > r.hi ? r.hi - kSettleInset : v;
    };
    j[0] = inside(j[0], kin::kLimitL);
    j[1] = inside(j[1], kin::limit_r(j[0]));  // R's range at the corrected L (the elbow's stops are inside it)
    j[2] = inside(j[2], kin::kLimitBase);
    return queue_joints(j, f);
}

void hold(bool on) { stepgen::hold(on); }

void stop() {
    if (line.phase == Phase::kChecking || line.phase == Phase::kSearching) line.result = Result::kOk;  // the command is answered with E25 by the stop
    line.phase = Phase::kIdle;
    int16_t last[3];
    stepper::stop(last);
    int32_t at[3];
    stepper::position(at);
    stepgen::stop(at, last, kBrakeFactor * accel_a * 80.0f);
}

void tally_angles(float out[3]) {
    int32_t s[3];
    stepper::position(s);
    to_angles(s, out);
}

void end_angles(float out[3]) { to_angles(planner::end_position(), out); }

void release(uint8_t joint) {
    const uint8_t axis = joint == 0 ? 2 : static_cast<uint8_t>(joint - 1);  // protocol 0 = base, 1 = L, 2 = R
    locked_mask = static_cast<uint8_t>(locked_mask & ~(1u << axis));
    stepper::enable(axis, false);
}

bool locked(uint8_t joint) {
    const uint8_t axis = joint == 0 ? 2 : static_cast<uint8_t>(joint - 1);
    return (locked_mask & (1u << axis)) != 0;
}

bool set_accel(float a) {
    if (isnan(a) || a <= 0.0f || a > kMaxA) return false;  // SP-37
    accel_a = a;
    limits();
    nvm::write(kAccelAddr, &a, sizeof a);
    nvm::write_byte(kAccelMarkAddr, kAccelMark);
    return true;
}

float accel() { return accel_a; }

void set_offsets(const kin::Offsets& o) { offs = o; }

const kin::Offsets& offsets() { return offs; }

}  // namespace motion
