// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jack Sidman Smith
//
// The buzzer on PL5 (the original's pin, uarm_peripheral.c:201), driven by Timer5 toggling OC5C in hardware: no ISR.
// Frequency in Hz and duration in milliseconds (SP-24: the original's duration was right only at 1 kHz). While it
// sounds, sync commands wait and their replies are held, as in the original (uarm_protocol.c:148, :175).

#pragma once

#include <stdint.h>

namespace buzzer {

void init();
bool tone(float hz, uint16_t ms);  // false if hz is outside 16-20000 or ms is 0
void poll();                       // main loop: ends the tone on time
bool idle();

}  // namespace buzzer
