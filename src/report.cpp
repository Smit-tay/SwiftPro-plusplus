// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jack Sidman Smith

#include "report.h"

#include "commands.h"
#include "effector.h"
#include "kin.h"
#include "motion.h"
#include "tick.h"

#include <math.h>

#if defined(__AVR__)
#include <avr/io.h>
#endif

namespace report {

namespace {

constexpr uint32_t kMaxPeriodMs = 3600000;  // an hour

uint32_t period_ms = 0;  // 0 = off
uint32_t next_due = 0;
uint16_t counter = 0;
uint8_t rotation = 0;    // 0 B, 1 L, 2 R

// Buttons: 0 Menu (PE7), 1 Play (PE6).
constexpr uint8_t kSamples = 3;        // in a row, to change state
constexpr uint16_t kLongSamples = 100; // 1 s of 10 ms samples
struct Button {
    bool down = false;
    uint8_t run = 0;       // samples in a row that disagree with down
    uint16_t held = 0;     // samples since it went down
    uint8_t event = 0;     // 0 none, 1 click, 2 long press
};
Button buttons[2];
bool reports_on = false;
uint32_t last_sample = 0;

#if defined(__AVR__)
bool pressed(uint8_t b) { return (PINE & (b == 0 ? _BV(PE7) : _BV(PE6))) == 0; }
#else
bool host_pressed[2] = {};
bool pressed(uint8_t b) { return host_pressed[b]; }
#endif

void sample(Button& b, bool level) {
    if (b.down) ++b.held;
    if (level == b.down) {
        b.run = 0;
        return;
    }
    if (++b.run < kSamples) return;
    b.run = 0;
    b.down = level;
    if (level) {
        b.held = kSamples;  // it has been down since the first of those samples
    } else if (reports_on) {
        b.event = b.held >= kLongSamples ? 2 : 1;
    }
}

}  // namespace

void init() {
    period_ms = 0;
    counter = 0;
    rotation = 0;
    reports_on = false;
    for (Button& b : buttons) b = Button{};
#if defined(__AVR__)
    DDRE = static_cast<uint8_t>(DDRE & ~(_BV(PE6) | _BV(PE7)));
    PORTE = static_cast<uint8_t>(PORTE | _BV(PE6) | _BV(PE7));  // pull-ups: pressed = low
#endif
    last_sample = tick::now_ms();
}

bool set_period(float seconds) {
    if (isnan(seconds)) return false;
    if (seconds <= 0.0f) {  // faithful uarm_peripheral.c:301-305
        stop();
        return true;
    }
    const float ms = roundf(seconds * 1000.0f);
    if (ms < static_cast<float>(kMinPeriodMs) || ms > static_cast<float>(kMaxPeriodMs)) return false;  // Q-47, SP-38
    period_ms = static_cast<uint32_t>(ms);
    next_due = tick::now_ms() + period_ms;  // the first report after one period, as the original
    return true;
}

void stop() { period_ms = 0; }

bool due() { return period_ms != 0 && static_cast<int32_t>(tick::now_ms() - next_due) >= 0; }

namespace {

void advance() {
    ++counter;
    rotation = static_cast<uint8_t>((rotation + 1) % 3);
    next_due += period_ms;
    const uint32_t now = tick::now_ms();
    if (static_cast<int32_t>(now - next_due) >= 0) next_due = now + period_ms;  // fell behind: don't burst
}

}  // namespace

void write(fmt::Writer& out) {
    float a[3];  // L, R, B (-90 frame)
    motion::tally_angles(a);
    float x, y, z;
    kin::forward(a[0], a[1], a[2], &x, &y, &z);
    kin::arm_to_effector(motion::offsets(), &x, &y, &z);
    out.put(FLASH("@3 X"));
    out.f4(x);
    out.put(FLASH(" Y"));
    out.f4(y);
    out.put(FLASH(" Z"));
    out.f4(z);
    out.put(FLASH(" R"));
    out.f4(commands::effector_angle());  // SP-61, Q-68 (the original: the servo's in every mode)
    out.put(FLASH(" B"));
    out.f4(a[2]);
    out.put(FLASH(" L"));
    out.f4(a[0]);
    out.put(FLASH(" AR"));
    out.f4(a[1]);
    // Q-1, Q-33: one measured joint, in rotation B, L, R.
    constexpr char kLetters[3] = {'B', 'L', 'R'};
    constexpr uint8_t kAxes[3] = {2, 0, 1};
    out.put(FLASH(" M"));
    out.put(kLetters[rotation]);
    float deg;
    uint32_t at;
    const uint32_t now = tick::now_ms();
    if (commands::measured(kAxes[rotation], &deg, &at)) {
        out.f4(deg);
        out.put(FLASH(" A"));
        out.u32(now - at);
    } else {
        out.put('-');
    }
    out.put(FLASH(" T"));
    out.u32(now);
    out.put(FLASH(" S"));
    out.u16(counter);
    advance();
}

void skip() { advance(); }

void set_button_reports(bool on) {
    reports_on = on;
    for (Button& b : buttons) b.event = 0;
}

void poll_buttons() {
    const uint32_t now = tick::now_ms();
    if (now - last_sample < 10) return;
    last_sample = now;
    for (uint8_t i = 0; i < 2; ++i) sample(buttons[i], pressed(i));
}

bool button_event(fmt::Writer& out) {
    for (uint8_t i = 0; i < 2; ++i) {
        Button& b = buttons[i];
        if (b.event == 0) continue;
        out.put(FLASH("@4 N"));
        out.u16(i);
        out.put(FLASH(" V"));
        out.u16(b.event);
        b.event = 0;
        return true;
    }
    return false;
}

#if !defined(__AVR__)
void host_set_button(uint8_t button, bool on) { host_pressed[button] = on; }
#endif

}  // namespace report
