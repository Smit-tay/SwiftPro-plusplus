// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jack Sidman Smith
//
// Command handlers and the machine state the protocol layer needs to schedule them.
//
// Built so far: identity and settings (module 2), encoder queries (module 3), motion (module 4), effector (5a), the
// @3 report, buttons and the lost-step watch (5b). Commands not built yet reply E20, as an unknown command does (SP-12).

#pragma once

#include <stdint.h>

#include "fmt.h"
#include "parse.h"

namespace commands {

enum class Status : uint8_t {
    kOk, kE20, kE21, kE22, kE25, kE26,
    kPending,  // not finished: call resume() on later main-loop passes until it returns something else
};

// How a sync command is scheduled.
enum class Kind : uint8_t {
    kMotion,   // goes to the planner; may start while earlier moves run
    kNormal,   // everything else: waits for motion idle (Q-5)
};

// Reads the work mode, offsets and UID from EEPROM (uarm_peripheral.c:327-372), and the boot position from the encoders.
void init();

Kind kind(const parse::Command& c);

// Runs c. A handler may append its reply tail (e.g. " V1", no newline) to tail.
Status execute(const parse::Command& c, fmt::Writer& tail);

// Continue a command that returned kPending, with the same tail. A P/S command (async) and a G/M command (sync) may be
// pending at the same time; each is resumed with its own function. The work itself is done by poll().
Status resume_async(fmt::Writer& tail);
Status resume_sync(fmt::Writer& tail);

// Main loop: one encoder bus step, and the motion pipeline. Returns true if it did anything.
bool poll();

// The next unsolicited event line, without its "\n" (@8, @9, @6, @5, @4); false if none is waiting.
bool next_event(fmt::Writer& out);

// S1100 and Ctrl-X: stop everything now.
void stop();

// True once after the lost-step watch has found a fault (SP-41): the protocol then stops everything, as S1100.
bool take_fault();

// The latest encoder reading of a motion axis (0 L, 1 R, 2 B; base in the -90 frame) and when it was taken, for @3.
// False if there is none, or it failed.
bool measured(uint8_t axis, float* deg, uint32_t* at_ms);

// Faithful reply gate (uarm_protocol.c:175): motion stopped, effector idle, buzzer idle.
bool idle();

// Whether a sync command of this kind may start now (uarm_protocol.c:145-150 plus Q-5).
bool can_start(Kind k);

// M2124 V1: a motion command's reply is sent when it is accepted (Q-32).
bool streaming();

uint8_t work_mode();  // for the boot banner

// The end effector's angle: the wrist servo's as output, or the stepper effector's in modes 4/5 (P2206 N3, @3 R).
float effector_angle();

}  // namespace commands
