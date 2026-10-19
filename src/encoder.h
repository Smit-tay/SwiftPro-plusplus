// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jack Sidman Smith
//
// The three AS5600 joint encoders (address 0x36), read one I2C transaction (about 0.7 ms) per main-loop pass, so the
// main loop never blocks for a whole reading (SP-41 correction: a blocking read can starve the stepper's segment
// buffer; R-16).
//
// Faithful: the angle is the median of 3 samples of the ANGLE register (0x0E/0x0F), and
//   angle = pointB - wrap(reference - raw) * 360 / 4096          (uarm_angle.c:60-80, 136-224)
// with the reference counts from EEPROM 820 (or 800 if the flag at 818 isn't 0xBB) and pointB L 32.758930,
// R 88.980537, base 90.0 (uarm_angle.c:36-41).
// Changes: one I2C transaction per sample, the two ANGLE bytes in one burst (SP-18); a NACK or an impossible value
// is a failure, reported (SP-15); STATUS is checked for MD/ML/MH (SP-42).

#pragma once

#include <stdint.h>

namespace encoder {

enum class Joint : uint8_t { kL = 0, kR = 1, kB = 2 };  // the original's channel order

enum class Job : uint8_t {
    kAngle,   // median of 3 ANGLE samples -> raw()
    kHealth,  // STATUS (0x0B), AGC (0x1A), MAGNITUDE (0x1B/0x1C)
    kWatch,   // kAngle, then STATUS: the lost-step check's reading, with the magnet health it needs (SP-41, SP-42)
};

void init();  // buses idle, reference counts from EEPROM (faithful uarm_angle.c:233-251)

bool start(Joint j, Job job);  // false if a job is running
bool busy();
void poll();                   // one I2C transaction of the running job
void run(Joint j, Job job);    // start and finish (only before the main loop starts, e.g. at boot)

// Results of the last finished job.
bool ok();                // false: a NACK or an impossible reading (SP-15)
uint16_t raw();           // kAngle: median raw count
uint8_t status();         // kHealth, kWatch: STATUS
uint8_t agc();            // kHealth
uint16_t magnitude();     // kHealth

// STATUS bits (AS5600 datasheet; not in the original source, UNVERIFIED): MD magnet detected, ML too weak, MH too strong.
constexpr uint8_t kMD = 0x20;
constexpr uint8_t kML = 0x10;
constexpr uint8_t kMH = 0x08;

// Fault (P2244): no answer, no magnet (SP-42), or the original's test, STATUS & 0x18 == 0x18 (uarm_angle.c:85-98).
bool health_fault(bool ok, uint8_t status);

uint16_t reference(Joint j);              // P2242
void set_reference(Joint j, uint16_t raw);  // M2401 (the caller saves it to EEPROM)
float angle(Joint j, uint16_t raw);       // calculate_current_angle (uarm_angle.c:136-224); base in the user frame

}  // namespace encoder
