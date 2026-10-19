// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jack Sidman Smith
//
// The ADC, one conversion at a time: AVcc reference, clk/128 (125 kHz, as the original: uarm_swift.c:23), 10 bits,
// about 0.1 ms per reading. Channels 0-15 are the Arduino pins A0-A15.

#pragma once

#include <stdint.h>

namespace adc {

void init();
uint16_t read(uint8_t channel);  // 0-1023

#if !defined(__AVR__)
void host_set(uint8_t channel, uint16_t value);  // the simulator's analog inputs
#endif

}  // namespace adc
