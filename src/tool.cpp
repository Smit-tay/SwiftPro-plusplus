// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jack Sidman Smith

#include "tool.h"

#include "atomic.h"

#include <math.h>

#if defined(__AVR__)
#include <avr/interrupt.h>
#include <avr/io.h>
#endif

namespace tool {

namespace {

constexpr float kStepsPerDegFlat = 64.0f / 0.9f;      // mode 4 (uarm_peripheral.c:84)
constexpr float kStepsPerDegStandard = 64.0f / 1.8f;  // mode 5 (uarm_peripheral.c:87)
// Timer3 CTC at clk/64 (4 us): OCR3A 47 gives 192 us, the original's half step (Timer4 at clk/1024, TCNT preload
// 0x10000 - int(16e6 * 0.0002 / 1024) = 3 ticks of 64 us, uarm_timer.c:95-100).
constexpr uint8_t kOcr = 47;
static_assert((kOcr + 1) * 4 == 192, "192 us per half step");

bool is_active = false;
float steps_per_deg = kStepsPerDegFlat;
Origin state = Origin::kUnknown;
float offset = 0.0f;

// Shared with the ISR.
volatile bool running = false;
volatile bool searching = false;  // stop at the switch
volatile bool hit = false;        // the search stopped at the switch
volatile bool high = false;       // STEP is high (second half of a step)
volatile uint16_t remaining = 0;
volatile int8_t dir = 1;          // +1: DIR low
volatile int32_t position = 0;    // steps from the switch's position (valid once kFound)

#if defined(__AVR__)
void step_pin(bool on) {
    if (on) PORTH = static_cast<uint8_t>(PORTH | _BV(PH6));
    else PORTH = static_cast<uint8_t>(PORTH & ~_BV(PH6));
}
void dir_pin(bool on) {
    InterruptLock lock;
    if (on) PORTH = static_cast<uint8_t>(PORTH | _BV(PH5));
    else PORTH = static_cast<uint8_t>(PORTH & ~_BV(PH5));
}
bool switch_pressed() { return (PINK & _BV(PK5)) == 0; }
void timer(bool on) {
    if (on) {
        TCNT3 = 0;
        TIMSK3 = _BV(OCIE3A);
    } else {
        TIMSK3 = 0;
    }
}
#else
bool host_switch = false, host_timer = false;
int32_t host_count = 0;
uint32_t host_us = 0;
void step_pin(bool on) {
    if (on) host_count += dir;
}
void dir_pin(bool) {}
bool switch_pressed() { return host_switch; }
void timer(bool on) { host_timer = on; }
#endif

void start(uint16_t steps, int8_t d, bool search_mode) {
    timer(false);
    running = false;
    high = false;
    step_pin(false);
    dir = d;
    dir_pin(d < 0);
    remaining = steps;
    searching = search_mode;
    hit = false;
    if (steps == 0) return;
    running = true;
    timer(true);
}

}  // namespace

void init() {
#if defined(__AVR__)
    TCCR3A = 0;
    TCCR3B = _BV(WGM32) | _BV(CS31) | _BV(CS30);  // CTC, clk/64
    OCR3A = kOcr;
    TIMSK3 = 0;
#endif
    is_active = false;
    running = false;
    state = Origin::kUnknown;
}

void activate(uint8_t mode) {
    if (mode != 4 && mode != 5) {
        deactivate();
        return;
    }
    steps_per_deg = mode == 4 ? kStepsPerDegFlat : kStepsPerDegStandard;
    is_active = true;
    state = Origin::kUnknown;
#if defined(__AVR__)
    {
        InterruptLock lock;
        DDRH = static_cast<uint8_t>(DDRH | _BV(PH5) | _BV(PH6));  // steper_init (uarm_peripheral.c:58-64)
        PORTH = static_cast<uint8_t>(PORTH & ~(_BV(PH5) | _BV(PH6)));
        DDRK = static_cast<uint8_t>(DDRK & ~_BV(PK5));             // origin switch, pull-up (:148-149)
        PORTK = static_cast<uint8_t>(PORTK | _BV(PK5));
    }
#endif
}

void deactivate() {
    stop();
    is_active = false;
    state = Origin::kUnknown;
}

bool active() { return is_active; }

void search(float angle_offset) {
    if (!is_active) return;
    offset = angle_offset;
    state = Origin::kSearching;
    start(static_cast<uint16_t>(lroundf(360.0f * steps_per_deg)), -1, true);  // DIR high, 360 deg (:150-152)
}

Origin origin() { return state; }

bool busy() { return running || state == Origin::kSearching; }

bool move_to(float a) {
    if (isnan(a) || a < 0.0f || a > 360.0f) return false;  // the original returned silently with ok (:109)
    int32_t pos;
    {
        InterruptLock lock;
        pos = position;
    }
    const int32_t target = static_cast<int32_t>(lroundf((a + offset) * steps_per_deg));  // R-17: angle = steps / k - offset
    const int32_t delta = target - pos;
    start(static_cast<uint16_t>(delta < 0 ? -delta : delta), delta < 0 ? -1 : 1, false);
    return true;
}

float angle() {
    int32_t pos;
    {
        InterruptLock lock;
        pos = position;
    }
    return static_cast<float>(pos) / steps_per_deg - offset;
}

void stop() {
    timer(false);
    running = false;
    high = false;
    step_pin(false);
    if (state == Origin::kSearching) state = Origin::kUnknown;
}

void poll() {
    if (state != Origin::kSearching || running) return;
    if (!hit) {  // 360 deg without the switch (SP-60)
        state = Origin::kFailed;
        return;
    }
    {
        InterruptLock lock;
        position = 0;  // the switch: angle = -offset (:161)
    }
    state = Origin::kFound;
    move_to(90.0f);  // :162
}

void on_timer() {
    if (!running) return;
    if (high) {
        step_pin(false);
        high = false;
        return;
    }
    if (searching && switch_pressed()) {
        hit = true;
        running = false;
        timer(false);
        return;
    }
    if (remaining == 0) {
        running = false;
        timer(false);
        return;
    }
    --remaining;
    position += dir;
    step_pin(true);
    high = true;
}

#if !defined(__AVR__)
void host_advance(uint32_t us) {
    host_us += us;
    while (host_us >= 192) {
        host_us -= 192;
        if (host_timer) on_timer();
    }
}
void host_set_switch(bool pressed) { host_switch = pressed; }
int32_t host_steps() { return host_count; }
#endif

}  // namespace tool

#if defined(__AVR__)
ISR(TIMER3_COMPA_vect) { tool::on_timer(); }
#endif
