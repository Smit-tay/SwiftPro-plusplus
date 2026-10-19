// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jack Sidman Smith
//
// End effector and board inputs, at the original's pins (uarm_peripheral.c, uarm_swift.c):
//   wrist servo  OC4A / PH3, 50 Hz; pulse = (180 - angle) / 180 * 2 ms + 0.5 ms (1.51 ms = 89.1 deg after init)
//   pump         PG4 (pump) and PG5 (valve): on = PG4 1, PG5 0; off = PG4 0, PG5 1; on V3.3.0 boards the valve then
//                pulses 10 times, 50 ms on / 50 ms off (now without blocking, SP-17)
//   gripper      PC7: 0 = catch, 1 = release; PA0 reads whether something is held
//   laser        PH5 and PH6 together
//   limit switch PG3, pull-up, pressed = low
//   power sense  ADC A5 > 100 = motor power present
//   pump sense   ADC A9: < 10 none, <= 70 holding, else running free (P2231)
// Changes: the wrist uses the timer's full resolution, 0.5 us steps (Q-12, about 0.05 deg; the original used 1/1024 of
// the period, 1.76 deg); its angle is reported from what is actually output; release really stops the pulses (Q-7).

#pragma once

#include <stdint.h>

namespace effector {

void init();   // wrist at 89.1 deg, pump off, gripper released, laser off, inputs set up
void poll();   // main loop: the valve sequence

// Wrist.
bool set_wrist(float angle);  // false if outside 0-180 (SP-23)
float wrist();                 // the angle actually output
void attach_wrist(bool on);
bool wrist_attached();

// Pump: 0 off, 1 on. status(): 0 off, 1 on (nothing held), 2 holding (P2231).
void pump(bool on);
uint8_t pump_status();

// Gripper: true = catch. status(): 0 released, 1 caught (PA0 high), 2 holding (PA0 low) (P2232).
void gripper(bool catch_it);
uint8_t gripper_status();

void laser(bool on);
bool laser_on();

bool limit_pressed();   // P2233, @6
bool power_present();   // P2234, @5

#if !defined(__AVR__)
// Host only: the simulator's inputs.
void host_set_pa0(bool high);
void host_set_limit(bool pressed);
bool host_valve_on();
#endif

}  // namespace effector
