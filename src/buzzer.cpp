// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jack Sidman Smith

#include "buzzer.h"

#include "atomic.h"
#include "tick.h"

#include <math.h>

#if defined(__AVR__)
#include <avr/io.h>
#endif

namespace buzzer {

namespace {

bool sounding = false;
uint32_t start = 0;
uint16_t length = 0;

void silence() {
    sounding = false;
#if defined(__AVR__)
    TCCR5A = 0;  // OC5C disconnected
    TCCR5B = 0;  // timer stopped
    InterruptLock lock;  // PORTL is also written by the stepper ISR (PL1, PL3)
    PORTL = static_cast<uint8_t>(PORTL & ~_BV(PL5));
#endif
}

}  // namespace

void init() {
#if defined(__AVR__)
    {
        InterruptLock lock;
        DDRL = static_cast<uint8_t>(DDRL | _BV(PL5));
    }
#endif
    silence();
}

bool tone(float hz, uint16_t ms) {
    if (isnan(hz) || hz < 16.0f || hz > 20000.0f || ms == 0) return false;
    // CTC with TOP = OCR5A at clk/8: OC5C toggles at each match, so f = 16 MHz / (2 * 8 * (OCR5A + 1)) =
    // 1 MHz / (OCR5A + 1). 16 Hz needs 62499, the 16-bit limit's neighbourhood; 20 kHz needs 49.
    const uint16_t top = static_cast<uint16_t>(lroundf(1000000.0f / hz) - 1);
#if defined(__AVR__)
    TCCR5B = 0;
    TCNT5 = 0;
    OCR5A = top;
    OCR5C = 0;
    TCCR5A = _BV(COM5C0);             // toggle OC5C on compare match
    TCCR5B = _BV(WGM52) | _BV(CS51);  // CTC (TOP = OCR5A), clk/8
#else
    (void)top;
#endif
    sounding = true;
    start = tick::now_ms();
    length = ms;
    return true;
}

void poll() {
    if (sounding && tick::now_ms() - start >= length) silence();
}

bool idle() { return !sounding; }

}  // namespace buzzer
