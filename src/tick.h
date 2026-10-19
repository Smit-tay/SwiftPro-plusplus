// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jack Sidman Smith
//
// 1 ms time base on Timer0 (CTC, 16 MHz / 64 / 250 = exactly 1 kHz; Q-13). The original used Timer5 with a
// 0.96 ms period and counters that fired one tick late (SP-25).

#pragma once

#include <stdint.h>

namespace tick {

void init();
uint32_t now_ms();

// Called by the Timer0 ISR on the AVR, and by the simulator once per simulated millisecond.
void on_tick();

}  // namespace tick
