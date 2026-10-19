// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jack Sidman Smith
//
// Times firmware functions on the ATmega2560 under simavr (tools/profile/run.sh). Timer4 counts CPU clocks
// (16 MHz, 62.5 ns); each function is timed several times and the worst case is printed in microseconds.
// Host-side measurement only; never flashed.

#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/sleep.h>
#include <stdlib.h>
#include <util/delay.h>

#include "fmt.h"
#include "kin.h"
#include "planner.h"
#include "stepgen.h"
#include "stepper.h"

namespace {

void put(char c) {
    while (!(UCSR0A & _BV(UDRE0))) {}
    UDR0 = c;
}
void put_s(const char* s) {
    while (*s) put(*s++);
}
void report(const char* name, uint16_t cycles) {
    char buf[48];
    fmt::Writer w(buf, sizeof buf);
    w.put(name);
    w.put(' ');
    w.u16(static_cast<uint16_t>(cycles / 16));  // us
    w.put(" us\n");
    put_s(w.text());
}

inline void start() { TCNT4 = 0; }
inline uint16_t stop() { return TCNT4; }

volatile float sink;

}  // namespace

int main() {
    UCSR0B = _BV(TXEN0);
    TCCR4A = 0;
    TCCR4B = _BV(CS40);  // clk/1

    uint16_t worst = 0;
    float a, b, c, x, y, z;
    for (int i = 0; i < 20; ++i) {
        const float px = 150.0f + static_cast<float>(i) * 7.0f, pz = 20.0f + static_cast<float>(i) * 6.0f;
        start();
        kin::inverse(px, 30.0f, pz, &a, &b, &c);
        const uint16_t t = stop();
        if (t > worst) worst = t;
        sink = a + b + c;
    }
    report("kin::inverse", worst);

    worst = 0;
    for (int i = 0; i < 20; ++i) {
        start();
        kin::forward(30.0f + static_cast<float>(i), 60.0f, 10.0f, &x, &y, &z);
        const uint16_t t = stop();
        if (t > worst) worst = t;
        sink = x + y + z;
    }
    report("kin::forward", worst);

    worst = 0;
    for (int i = 0; i < 20; ++i) {
        char buf[24];
        fmt::Writer w(buf, sizeof buf);
        start();
        w.f4(233.6009f + static_cast<float>(i));
        const uint16_t t = stop();
        if (t > worst) worst = t;
    }
    report("fmt::f4 (dtostrf)", worst);

    // The work of one @3 report (report::write): FK, the effector offsets, 8 floats and 3 integers.
    worst = 0;
    for (int i = 0; i < 10; ++i) {
        char buf[160];
        fmt::Writer w(buf, sizeof buf);
        start();
        kin::forward(45.6789f + static_cast<float>(i), 66.0411f, -0.0879f, &x, &y, &z);
        kin::arm_to_effector(kin::Offsets{56.65f, 74.55f}, &x, &y, &z);
        w.put("@3 X");
        w.f4(x);
        w.put(" Y");
        w.f4(y);
        w.put(" Z");
        w.f4(z);
        w.put(" R");
        w.f4(89.1f);
        w.put(" B");
        w.f4(-0.0879f);
        w.put(" L");
        w.f4(45.6789f);
        w.put(" AR");
        w.f4(66.0411f);
        w.put(" MB");
        w.f4(-0.0879f);
        w.put(" A");
        w.u32(3);
        w.put(" T");
        w.u32(1234567);
        w.put(" S");
        w.u16(12345);
        const uint16_t t = stop();
        if (t > worst) worst = t;
        if (i == 0) {
            put_s(w.text());
            put('\n');
        }
    }
    report("@3 report formatting", worst);

    // planner::add with a full queue of short, slightly turning moves (the worst replan).
    const int32_t zero[3] = {0, 0, 0};
    planner::init({400.0f, 300.0f, 0.2f});
    planner::set_end_position(zero);
    stepgen::init(zero);
    worst = 0;
    for (int i = 1; i <= 15; ++i) {
        const int32_t t3[3] = {i * 200, i * 100 + (i % 2) * 20, i * 50};
        start();
        planner::add(t3, 150.0f);
        const uint16_t t = stop();
        if (t > worst) worst = t;
    }
    report("planner::add (15 queued)", worst);

    // stepgen::next while running through those moves.
    worst = 0;
    stepgen::Segment s;
    for (int i = 0; i < 300; ++i) {
        start();
        const bool more = stepgen::next(&s);
        const uint16_t t = stop();
        if (t > worst) worst = t;
        if (!more) break;
    }
    report("stepgen::next", worst);

    // The stepper ISR body with three joints stepping (pulse width included: it waits for 8 us).
    stepper::init(zero);
    stepgen::Segment q{{150, 149, 148}};
    stepper::queue(q);
    TIMSK1 = 0;  // call the body directly instead of from the timer
    worst = 0;
    uint16_t first = 0;
    uint32_t total = 0;
    for (int i = 0; i < 150; ++i) {
        start();
        stepper::on_timer();
        const uint16_t t = stop();
        if (i == 0) first = t;  // this event loads the segment (two divisions)
        else {
            if (t > worst) worst = t;
            total += t;
        }
    }
    report("stepper ISR, loading a segment", first);
    report("stepper ISR, other events (worst)", worst);
    report("stepper ISR, other events (mean)", static_cast<uint16_t>(total / 149));

    put_s("END\n");
    _delay_ms(5);
    cli();
    sleep_cpu();
}
