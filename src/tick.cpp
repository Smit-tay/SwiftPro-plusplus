// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jack Sidman Smith

#include "tick.h"

#include "atomic.h"

#if defined(__AVR__)
#include <avr/interrupt.h>
#include <avr/io.h>
#endif

namespace tick {

namespace {

volatile uint32_t ms = 0;

#if defined(__AVR__)
constexpr uint8_t kTop = 249;  // 16 MHz / 64 / (249 + 1) = 1000 Hz
static_assert(F_CPU / 64 / (kTop + 1) == 1000, "Timer0 tick is not 1 ms");
#endif

}  // namespace

void init() {
    ms = 0;
#if defined(__AVR__)
    TCCR0A = _BV(WGM01);             // CTC, TOP = OCR0A, OC0A/OC0B disconnected
    OCR0A = kTop;
    TCCR0B = _BV(CS01) | _BV(CS00);  // clk / 64
    TIMSK0 = _BV(OCIE0A);
#endif
}

uint32_t now_ms() {
    InterruptLock lock;  // 32-bit read, written by the ISR
    return ms;
}

void on_tick() { ms = ms + 1; }

}  // namespace tick

#if defined(__AVR__)
ISR(TIMER0_COMPA_vect) { tick::on_tick(); }
#endif
