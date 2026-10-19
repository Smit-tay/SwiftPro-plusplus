// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jack Sidman Smith

#include "stepper.h"

#include "atomic.h"
#include "board.h"

#if defined(__AVR__)
#include <avr/interrupt.h>
#include <avr/io.h>
#endif

namespace stepper {

namespace {

using planner::kAxes;

// A segment as the ISR needs it: step counts per joint, direction bits (1 = towards fewer steps), events.
struct Seg {
    uint8_t steps[kAxes];
    uint8_t dir;
    uint8_t events;    // the largest step count; 0 = an empty 5 ms segment
    uint16_t period;   // ticks between events (computed when queued: no division in the ISR)
    uint8_t extra;     // events that get one tick more, spread evenly
};

Seg ring[kRing];
volatile uint8_t ring_head = 0;  // written by the main loop
volatile uint8_t ring_tail = 0;  // written by the ISR

// ISR state.
Seg cur{};
uint8_t events_left = 0;
uint8_t bres[kAxes] = {};  // Bresenham accumulators
uint16_t base_period = 0;
uint8_t extra = 0;         // events that get one extra tick, spread with an accumulator
uint8_t extra_acc = 0;
uint16_t next_period = 0;  // ticks to the next ISR
volatile bool timer_on = false;
// The tally: steps of the finished segments, plus the steps already done in the current one (computed on request from
// the Bresenham state, so the per-event work has no 32-bit arithmetic). Read under InterruptLock (a memory barrier).
int32_t committed[kAxes] = {};
bool cur_valid = false;  // cur is being played (its steps are not yet in committed)
uint8_t enabled_mask = 0;
uint8_t dir_invert = 0;

uint8_t next_index(uint8_t i) { return static_cast<uint8_t>((i + 1) % kRing); }

#if defined(__AVR__)
// Pin helpers. PORTF and PORTD are in the bit-instruction range; PORTL and PORTK are not, so main-loop writes to them
// are locked (SP-28). The ISR runs with interrupts disabled, so its own writes need no lock.
__attribute__((always_inline)) inline void step_high(uint8_t mask) {
    if (mask & 1) PORTF = static_cast<uint8_t>(PORTF | _BV(PF6));
    if (mask & 2) PORTL = static_cast<uint8_t>(PORTL | _BV(PL3));
    if (mask & 4) PORTF = static_cast<uint8_t>(PORTF | _BV(PF0));
}
__attribute__((always_inline)) inline void step_low() {
    PORTF = static_cast<uint8_t>(PORTF & ~(_BV(PF6) | _BV(PF0)));
    PORTL = static_cast<uint8_t>(PORTL & ~_BV(PL3));
}
__attribute__((always_inline)) inline void set_dir(uint8_t dir) {
    const uint8_t d = static_cast<uint8_t>(dir ^ dir_invert);
    PORTF = static_cast<uint8_t>((PORTF & ~(_BV(PF7) | _BV(PF1))) | ((d & 1) ? _BV(PF7) : 0) | ((d & 4) ? _BV(PF1) : 0));
    PORTL = static_cast<uint8_t>((PORTL & ~_BV(PL1)) | ((d & 2) ? _BV(PL1) : 0));
}
void timer_start() {
    TCNT1 = 0;
    OCR1A = 100;  // first ISR 50 us from now: it loads the segment
    TIFR1 = _BV(OCF1A);
    TIMSK1 = static_cast<uint8_t>(TIMSK1 | _BV(OCIE1A));
}
void timer_stop() { TIMSK1 = static_cast<uint8_t>(TIMSK1 & ~_BV(OCIE1A)); }
__attribute__((always_inline)) inline void timer_period(uint16_t ticks) { OCR1A = static_cast<uint16_t>(ticks - 1); }
#else
void step_high(uint8_t) {}
void step_low() {}
void set_dir(uint8_t) {}
int32_t host_motor[kAxes] = {};
uint32_t host_until_isr = 0;
void timer_start() { host_until_isr = 100; }
void timer_stop() {}
void timer_period(uint16_t) {}
#endif

// Loads the next segment from the ring; false if it is empty.
// Adds the segment that just finished to the committed tally.
void commit() {
    if (!cur_valid) return;
    for (uint8_t i = 0; i < kAxes; ++i)
        committed[i] += (cur.dir & (1u << i)) ? -static_cast<int32_t>(cur.steps[i]) : cur.steps[i];
    cur_valid = false;
}

bool load() {
    commit();
    if (ring_tail == ring_head) return false;
    cur = ring[ring_tail];
    cur_valid = true;
    ring_tail = next_index(ring_tail);
    set_dir(cur.dir);  // DIR settles during the rest of this ISR, before the first STEP edge (several us)
    events_left = cur.events == 0 ? 1 : cur.events;
    base_period = cur.period;
    extra = cur.extra;
    const uint8_t half = static_cast<uint8_t>(cur.events >> 1);
    bres[0] = bres[1] = bres[2] = half;
    extra_acc = 0;
    return true;
}

}  // namespace

// One step event. The previous pulse ends here, this event's pulse starts here and lasts until the next event (at least
// 31 us, as the drivers step on the rising edge); the low time between them is this ISR's own work, a few us.
void on_timer() {
    step_low();
    if (events_left == 0 && !load()) {
        timer_on = false;
        timer_stop();
        return;
    }
    uint8_t mask = 0;
    if (cur.events != 0) {  // Bresenham, unrolled: each joint steps cur.steps[i] times in cur.events events
        uint16_t acc = static_cast<uint16_t>(bres[0] + cur.steps[0]);
        if (acc >= cur.events) {
            acc = static_cast<uint16_t>(acc - cur.events);
            mask |= 1;
        }
        bres[0] = static_cast<uint8_t>(acc);
        acc = static_cast<uint16_t>(bres[1] + cur.steps[1]);
        if (acc >= cur.events) {
            acc = static_cast<uint16_t>(acc - cur.events);
            mask |= 2;
        }
        bres[1] = static_cast<uint8_t>(acc);
        acc = static_cast<uint16_t>(bres[2] + cur.steps[2]);
        if (acc >= cur.events) {
            acc = static_cast<uint16_t>(acc - cur.events);
            mask |= 4;
        }
        bres[2] = static_cast<uint8_t>(acc);
    }
    if (mask != 0) {
        step_high(mask);
#if !defined(__AVR__)
        for (uint8_t i = 0; i < kAxes; ++i)
            if ((mask & (1u << i)) && (enabled_mask & (1u << i))) host_motor[i] += (cur.dir & (1u << i)) ? -1 : 1;
#endif
    }
    uint16_t period = base_period;
    if (extra != 0) {
        extra_acc = static_cast<uint8_t>(extra_acc + extra);
        if (extra_acc >= cur.events) {
            extra_acc = static_cast<uint8_t>(extra_acc - cur.events);
            ++period;
        }
    }
    next_period = period;
    timer_period(period);
    --events_left;
}

void init(const int32_t position[kAxes]) {
    {
        InterruptLock lock;
        ring_head = ring_tail = 0;
        events_left = 0;
        timer_on = false;
        for (uint8_t i = 0; i < kAxes; ++i) committed[i] = position[i];
        cur_valid = false;
    }
    enabled_mask = 0;
    dir_invert = board::direction_invert_mask();
#if defined(__AVR__)
    // Outputs: STEP/DIR/EN for each joint, and the micro-step pins.
    DDRF = static_cast<uint8_t>(DDRF | _BV(PF0) | _BV(PF1) | _BV(PF2) | _BV(PF6) | _BV(PF7));
    {
        InterruptLock lock;  // PORTK/PORTL/DDRK/DDRL are outside the bit-instruction range (SP-28)
        DDRL = static_cast<uint8_t>(DDRL | _BV(PL1) | _BV(PL3));
        DDRK = static_cast<uint8_t>(DDRK | _BV(PK0) | _BV(PK2) | _BV(PK3) | _BV(PK4));
        // 1/32 microstepping: MS1 = 1, MS2 = 0, MS3 = 1 (stepper.c:550-553). Drivers disabled (EN low).
        PORTK = static_cast<uint8_t>((PORTK & ~(_BV(PK0) | _BV(PK3))) | _BV(PK2) | _BV(PK4));
        PORTL = static_cast<uint8_t>(PORTL & ~(_BV(PL1) | _BV(PL3)));
    }
    DDRD = static_cast<uint8_t>(DDRD | _BV(PD7));
    PORTD = static_cast<uint8_t>(PORTD & ~_BV(PD7));
    PORTF = static_cast<uint8_t>(PORTF & ~(_BV(PF0) | _BV(PF1) | _BV(PF2) | _BV(PF6) | _BV(PF7)));
    // Timer1: CTC on OCR1A (mode 4), clk/8 = 2 MHz, interrupt off until a segment is queued.
    TCCR1A = 0;
    TCCR1B = _BV(WGM12) | _BV(CS11);
    TIMSK1 = 0;
#endif
}

bool room() { return next_index(ring_head) != ring_tail; }

bool queue(const stepgen::Segment& s) {
    if (!room()) return false;
    Seg q{};
    uint8_t events = 0;
    for (uint8_t i = 0; i < kAxes; ++i) {
        const int16_t n = s.steps[i];
        const uint16_t mag = static_cast<uint16_t>(n < 0 ? -n : n);
        if (mag > kMaxStepsPerSegment) return false;
        q.steps[i] = static_cast<uint8_t>(mag);
        if (n < 0) q.dir = static_cast<uint8_t>(q.dir | (1u << i));
        if (mag > events) events = static_cast<uint8_t>(mag);
    }
    q.events = events;
    q.period = events == 0 ? kTicksPerSegment : static_cast<uint16_t>(kTicksPerSegment / events);
    q.extra = events == 0 ? 0 : static_cast<uint8_t>(kTicksPerSegment % events);
    ring[ring_head] = q;
    ring_head = next_index(ring_head);
    InterruptLock lock;
    if (!timer_on) {
        timer_on = true;
        timer_start();
    }
    return true;
}

bool running() { return timer_on; }

void stop(int16_t last[kAxes]) {
    InterruptLock lock;
    for (uint8_t i = 0; i < kAxes; ++i)
        last[i] = !cur_valid ? 0 : static_cast<int16_t>((cur.dir & (1u << i)) ? -static_cast<int16_t>(cur.steps[i]) : cur.steps[i]);
    timer_stop();
    timer_on = false;
    ring_head = ring_tail = 0;
    // Keep the steps already made in the segment being played.
    int32_t at[kAxes];
    for (uint8_t i = 0; i < kAxes; ++i) at[i] = committed[i];
    if (cur_valid && cur.events != 0) {
        const uint16_t done = static_cast<uint16_t>(cur.events - events_left);
        for (uint8_t i = 0; i < kAxes; ++i) {
            const int32_t n = static_cast<int32_t>((static_cast<uint32_t>(done) * cur.steps[i] + (cur.events >> 1)) / cur.events);
            at[i] += (cur.dir & (1u << i)) ? -n : n;
        }
    }
    for (uint8_t i = 0; i < kAxes; ++i) committed[i] = at[i];
    cur_valid = false;
    events_left = 0;
    step_low();
}

void position(int32_t out[kAxes]) {
    Seg s;
    uint16_t done = 0;
    bool partial;
    {
        InterruptLock lock;
        for (uint8_t i = 0; i < kAxes; ++i) out[i] = committed[i];
        partial = cur_valid && cur.events != 0;
        if (partial) {
            s = cur;
            done = static_cast<uint16_t>(cur.events - events_left);
        }
    }
    if (!partial) return;
    // Steps after `done` events with the accumulators starting at events / 2: floor((done * steps + events/2) / events).
    for (uint8_t i = 0; i < kAxes; ++i) {
        const int32_t n = static_cast<int32_t>((static_cast<uint32_t>(done) * s.steps[i] + (s.events >> 1)) / s.events);
        out[i] += (s.dir & (1u << i)) ? -n : n;
    }
}

void set_position(const int32_t pos[kAxes]) {
    InterruptLock lock;
    for (uint8_t i = 0; i < kAxes; ++i) committed[i] = pos[i];
    cur_valid = false;
}

void enable(uint8_t axis, bool on) {
    enabled_mask = on ? static_cast<uint8_t>(enabled_mask | (1u << axis)) : static_cast<uint8_t>(enabled_mask & ~(1u << axis));
#if defined(__AVR__)
    InterruptLock lock;  // PORTF/PORTK are also written by the ISR (STEP/DIR) or are extended I/O
    switch (axis) {
        case 0: PORTF = on ? static_cast<uint8_t>(PORTF | _BV(PF2)) : static_cast<uint8_t>(PORTF & ~_BV(PF2)); break;
        case 1: PORTK = on ? static_cast<uint8_t>(PORTK | _BV(PK0)) : static_cast<uint8_t>(PORTK & ~_BV(PK0)); break;
        default: PORTD = on ? static_cast<uint8_t>(PORTD | _BV(PD7)) : static_cast<uint8_t>(PORTD & ~_BV(PD7)); break;
    }
#endif
}

bool enabled(uint8_t axis) { return (enabled_mask & (1u << axis)) != 0; }

#if !defined(__AVR__)
void host_pulses(int32_t out[kAxes]) {
    for (uint8_t i = 0; i < kAxes; ++i) out[i] = host_motor[i];
}

void host_advance(uint32_t ticks) {
    while (timer_on) {
        if (ticks < host_until_isr) {
            host_until_isr -= ticks;
            return;
        }
        ticks -= host_until_isr;
        on_timer();
        host_until_isr = next_period;
    }
}
#endif

}  // namespace stepper

#if defined(__AVR__)
ISR(TIMER1_COMPA_vect) { stepper::on_timer(); }
#endif
