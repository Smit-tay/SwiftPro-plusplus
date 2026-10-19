// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jack Sidman Smith
//
// The stepper end effector of work modes 4 (flat) and 5 (standard) (Q-14, SP-26, SP-27, SP-60). UNTESTED ON HARDWARE.
//
// From the original (uarm_peripheral.c:35-170, uarm_timer.c:95-116):
//   DIR PH5, STEP PH6 (the laser's pins), origin switch PK5 (Arduino A13), input with pull-up, pressed = low
//   64 steps per 0.9 deg in mode 4, per 1.8 deg in mode 5
//   a step every 384 us: STEP high for 192 us, low for 192 us
//   positive moves with DIR low ("anticlockwise"); the origin search runs with DIR high, at most 360 deg
//   at the switch the angle is -(effector angle offset, EEPROM 3202), then the effector moves to 90 deg
//   angles 0-360
// Changes: Timer3 instead of Timer4, so the wrist servo's timer is left alone (SP-27); the search runs in the
// background instead of blocking serial input (SP-26); the position is kept in steps and every target is rounded to
// the nearest step (R-17); a search that doesn't find the switch is reported (SP-60, Q-66).

#pragma once

#include <stdint.h>

namespace tool {

enum class Origin : uint8_t { kUnknown, kSearching, kFound, kFailed };

void init();                     // pins and timer idle; not active
void activate(uint8_t mode);     // modes 4/5: pins set up, origin unknown; other modes: deactivate
void deactivate();               // stop; DIR/STEP left low
bool active();

void search(float angle_offset);  // origin search, then a move to 90 deg
Origin origin();
bool busy();                      // searching or moving (replies wait, as the original's effect_ldie)
bool move_to(float angle);        // 0-360; false if out of range. Only when origin() is kFound
float angle();                    // the current angle (steps made so far)
void stop();                      // S1100, M2202 N3

void poll();                      // main loop: finishes the search
void on_timer();                  // the Timer3 ISR body

#if !defined(__AVR__)
void host_advance(uint32_t us);   // run the ISR for this much simulated time
void host_set_switch(bool pressed);
int32_t host_steps();             // steps made, signed (positive = DIR low)
#endif

}  // namespace tool
