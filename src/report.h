// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jack Sidman Smith
//
// The periodic position report (M2120) and the base buttons (@4).
//
// @3 X<x> Y<y> Z<z> R<wrist> B<base> L<l> AR<r> M<j><deg> A<ms> T<ms> S<n>
//   The first seven fields are the original's (faithful uarm_peripheral.c:210-244): the effector position and the
//   joint angles from the step tally (base in the -90 frame), and the wrist angle. Appended (Q-1, Q-33, Q-45): one
//   joint as measured by its encoder, in rotation B, L, R, in the same frame as its tally field, with the age of that
//   reading in ms ("M<j>-" with no A field if there is no good reading); the time since boot in ms; and a report
//   counter, 0-65535 wrapping, that also counts reports skipped because the TX ring was full (Q-21), so the host can
//   see the gap.
// Period: M2120 V<seconds>. V <= 0 stops (faithful uarm_peripheral.c:301-305), as does M2121. Shorter than
// kMinPeriodMs is E21 (Q-47); longer than an hour is E21 too (the original's int ms overflowed at 32.768 s, SP-38).
//
// @4 N<0 Menu|1 Play> V<1 click|2 long press> (SP-44, Q-29, Q-30): Menu on PE7, Play on PE6, pull-ups, pressed = low
// (the V3 and V4.10.0 firmwares' pins, uArmSwift.cpp:67-96, uarm_peripheral.c:931-955 of those trees; UNVERIFIED on
// your board). Sampled every 10 ms; a press or a release needs 3 samples in a row; held 1 s or more is a long press;
// the event is sent on release, only after M2213 V0.

#pragma once

#include <stdint.h>

#include "fmt.h"

namespace report {

// 10 ms (100 Hz, SP-46). A typical line is about 95 bytes, 8.2 ms at 115200 baud; when the link can't keep up, a report
// is skipped (and counted in S), never a reply: a report is sent only if a reply still fits after it.
constexpr uint16_t kMinPeriodMs = 10;
constexpr uint8_t kMaxLength = 160;  // with the "\n" and the NUL

void init();
bool set_period(float seconds);  // M2120; false (E21) if out of range
void stop();                     // M2121

bool due();                      // a report is due now: write() it or skip() it
void write(fmt::Writer& out);    // without the "\n"
void skip();                     // a due report that didn't fit (Q-21)

void set_button_reports(bool on);  // M2213 V0 = on
void poll_buttons();               // every main-loop pass; samples on the 10 ms tick
bool button_event(fmt::Writer& out);  // writes "@4 N<b> V<e>" if one is waiting

#if !defined(__AVR__)
void host_set_button(uint8_t button, bool pressed);  // 0 Menu, 1 Play
#endif

}  // namespace report
