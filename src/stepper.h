// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jack Sidman Smith
//
// Stepper output: the Timer1 ISR plays 5 ms segments from stepgen as evenly spaced step events, with the joints
// stepping in proportion (Bresenham), and keeps the step tally.
//
// Pins (the original's, cpu_map_atmega2560.h:144-175, 188-196, 540-553):
//   L: STEP PF6, DIR PF7, EN PF2     R: STEP PL3, DIR PL1, EN PK0     B: STEP PF0, DIR PF1, EN PD7
//   micro-step select MS1 PK2, MS2 PK3, MS3 PK4 = 1, 0, 1 (1/32 step, stepper.c:550-553).
//   Enable is active high on these boards (the original sets $4 = 1 at every boot, uarm_peripheral.c:631-646, and then
//   drives EN high to enable, stepper.c:189-191). DIR high = towards fewer steps (planner.c:310, stepper.c:361),
//   inverted for every joint on V3.3.0 boards (board::direction_invert_mask()).
// Timing: Timer1 CTC at 2 MHz (0.5 us). Each event ends the previous pulse and starts its own, which then lasts until
// the next event (at least 31 us; the drivers step on the rising edge). The low time between pulses and the DIR setup
// time are the ISR's own work, a few us. (Q-23 chose an 8 us pulse ended by a busy-wait; that wait cost a quarter of
// the CPU at full speed, so the pulse is ended by the next event instead.) No ISR re-enables interrupts.

#pragma once

#include <stdint.h>

#include "planner.h"
#include "stepgen.h"

namespace stepper {

constexpr uint8_t kRing = 8;                // queued segments: 40 ms
constexpr uint16_t kTicksPerSegment = 10000;  // 5 ms at 2 MHz
constexpr uint8_t kMaxStepsPerSegment = 160;  // 32 kHz; the planner's speed cap keeps it lower

void init(const int32_t position[planner::kAxes]);  // pins, timer, tally = position, motors disabled

bool room();                                   // a segment can be queued
bool queue(const stepgen::Segment& s);         // false if the ring is full or the segment has too many steps
bool running();                                // segments are being played
// Drops queued segments now and stops the timer. last gets the signed steps of the segment that was playing (the
// joints' speed in steps per 5 ms), so the motion layer can brake from that speed (Q-60).
void stop(int16_t last[planner::kAxes]);

void position(int32_t out[planner::kAxes]);    // the step tally (atomic copy, SP-16)
void set_position(const int32_t pos[planner::kAxes]);  // only while stopped (a re-sync)

void enable(uint8_t axis, bool on);            // driver enable (lock) for one joint
bool enabled(uint8_t axis);

// The ISR body. On the AVR, TIMER1_COMPA calls it; on the host, host_advance() does.
void on_timer();

#if !defined(__AVR__)
// Host only: run the ISR for this many 0.5 us ticks of simulated time.
void host_advance(uint32_t ticks);
// Host only: pulses sent to each motor since start-up, signed; set_position() doesn't change it (the simulator's
// motors).
void host_pulses(int32_t out[planner::kAxes]);
#endif

}  // namespace stepper
