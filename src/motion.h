// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jack Sidman Smith
//
// Motion commands, between the protocol and the planner.
//
// Position: each joint's angle = its boot angle (from the encoders, or the last re-sync) + its step tally / 80. The
// planner works in tally steps; "the current position" for relative moves and missing axes is the end of the queued
// motion (where the last accepted command will leave the arm).
//
// Moves: refused (E27) while any of the three joints is released, before anything is planned (Q-70).
//   joint moves (G0, G2202 N0-2, G2206): one straight move in joint space; the target is checked against the joint
//     limits first (E22 if out of range, nothing moves);
//   linear moves (G1, G2201, G2204, G2205): a straight line for the effector, cut into joint-space pieces short enough
//     that the path stays within kLineTolerance of the line (R-4). Every piece is checked before anything moves
//     (SP-20), then the pieces are fed to the planner as room allows.
// Speed: F in the original's units, 80 deg/min of joint-space motion (Q-51); modal, F100 after boot (Q-55); F <= 0 is
// E21. Acceleration: per joint, A * 80 deg/s^2 from M204, saved in EEPROM, default A5 (Q-52).

#pragma once

#include <stdint.h>

#include "kin.h"

namespace motion {

enum class Result : uint8_t {
    kOk,
    kBadParam,     // E21
    kUnreachable,  // E22
    kReleased,     // E27: base, L or R is released (M2202, M2019), whichever joints the move turns (Q-70)
    kNoPosition,   // E26: the boot or re-sync encoder read failed, so the position is unknown
    kChecking,     // a linear move is being checked; ask line_result() on later passes
};

void init();  // limits from EEPROM, motors off, position unknown

// Sets the boot angles of the joints in mask (bit 0 L, 1 R, 2 B; base in the user frame) and zeroes their tally.
// Only while idle. Enables those drivers.
void sync(uint8_t mask, const float angles[3]);
bool known();  // all three joints have a valid position
uint8_t known_joints();  // which joints have one (bit 0 L, 1 R, 2 B)
void hold_all();  // enable every driver without a position (boot read failed): the arm holds, moves still get E26

// True while a new motion command can't be taken yet (a linear move is being checked or cut up, or the planner is full).
bool busy();

// The outcome of a linear move that returned kChecking: kChecking until the check is done, then kOk (it runs) or the
// reason it was refused. Nothing moves before the whole line has been checked (SP-20).
Result line_result();
bool poll();   // main loop: feed the planner and the stepper; true if it did anything
bool idle();   // nothing queued or moving

Result move_joints(const float a[3], float f);       // L, R, B (user frame) in degrees; NaN = keep that joint
Result move_xyz(const float xyz[3], bool linear, float f);  // effector frame, mm; NaN = keep that axis (Q-54)
Result move_xyz_relative(const float dxyz[3], float f);    // G2204
Result move_polar(float s, float r, float h, bool relative, float f);  // G2201 / G2205

// Jogs (G2207, G2208; V4.10.0 uarm_protocol.c:408-1329, A4): from where the queued motion ends, as far as the joint
// limits allow, stopping kJogInset inside them. The sign of f is the direction, |f| the speed; f = 0 is kBadParam
// (Q-73). An edge less than kJogMin away is kUnreachable, as in the original. An L jog from outside the limits is
// kUnreachable too (R's range moves with L, so there is no straight way back in; G2209 brings the arm inside). The host pauses a jog with S1000 V0 or
// '!' and ends it with S1100 or Ctrl-X.
constexpr float kJogInset = 1.0f;  // deg, every limit (the original: 1 deg, the elbow 0..0.4)
constexpr float kJogMin = 0.7f;    // deg for joint jogs, mm for linear ones (the original's threshold, :1263)
Result jog_joint(uint8_t axis, float f);  // axis 0 L, 1 R, 2 B: one joint-space move
Result jog_xyz(uint8_t axis, float f);    // axis 0 X, 1 Y, 2 Z: a linear move; kChecking while the edge is found

// G2209 (V4.10.0 uarm_protocol.c:1331-1440, A4): every joint outside its limit (R: its range at the corrected L)
// moves to kSettleInset inside it, in one joint-space move from where the queued motion ends; kOk with no move if none is
// outside. The caller re-syncs from the encoders first.
constexpr float kSettleInset = 2.0f;
Result into_limits(float f);

void hold(bool on);  // S1000
void stop();         // S1100, Ctrl-X

// The tally position as joint angles (L, R, B in the -90 frame) and as effector XYZ, for @3 and lost-step checks.
void tally_angles(float out[3]);
void end_angles(float out[3]);  // where the queued motion ends

void release(uint8_t joint);  // M2202 N0-2 (0 = base, 1 = L, 2 = R, as G2202)
bool locked(uint8_t joint);

// Acceleration (M204): A in the protocol's units (1 = 80 deg/s^2).
bool set_accel(float a);
float accel();

void set_offsets(const kin::Offsets& o);
const kin::Offsets& offsets();

}  // namespace motion
