// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jack Sidman Smith

#include "effector.h"

#include "adc.h"
#include "atomic.h"
#include "board.h"
#include "tick.h"

#include <math.h>

#if defined(__AVR__)
#include <avr/io.h>
#endif

namespace effector {

namespace {

// Wrist: Timer4 in phase-and-frequency-correct PWM, TOP = ICR4 = 20000 at clk/8 (0.5 us ticks): a 20 ms period, and
// the high time in microseconds equals OCR4A (uarm_timer.c:115-170, uarm_peripheral.c:12-24).
constexpr uint16_t kTop = 20000;
uint16_t pulse_us = 1510;  // 89.1 deg (uarm_peripheral.c:10)
bool attached = false;

// Pump valve sequence on V3.3.0 boards (uarm_peripheral.c:704-760): 10 pulses, 50 ms on, 50 ms off.
enum class Valve : uint8_t { kIdle, kOn, kOff };
Valve valve = Valve::kIdle;
uint8_t valve_pulses = 0;
uint32_t valve_since = 0;
bool pump_running = false;

#if defined(__AVR__)
void set_bits(volatile uint8_t& r, uint8_t m, bool on) {
    InterruptLock lock;  // ports G and H are shared with code that may run from an ISR later (SP-28)
    r = on ? static_cast<uint8_t>(r | m) : static_cast<uint8_t>(r & ~m);
}
bool pump_pin() { return (PORTG & _BV(PG4)) != 0; }
void pump_pins(bool pump_on, bool valve_on) {
    set_bits(PORTG, _BV(PG4), pump_on);
    set_bits(PORTG, _BV(PG5), valve_on);
}
#else
bool host_pump = false, host_valve = false, host_gripper_released = true, host_laser = false;
bool host_pa0 = true, host_limit = false;
bool pump_pin() { return host_pump; }
void pump_pins(bool pump_on, bool valve_on) {
    host_pump = pump_on;
    host_valve = valve_on;
}
#endif

}  // namespace

void init() {
    pulse_us = 1510;
    attached = false;
    valve = Valve::kIdle;
    pump_running = false;
#if defined(__AVR__)
    // Wrist PWM pin and timer.
    {
        InterruptLock lock;
        DDRH = static_cast<uint8_t>(DDRH | _BV(PH3) | _BV(PH5) | _BV(PH6));  // wrist, laser
        PORTH = static_cast<uint8_t>(PORTH & ~(_BV(PH3) | _BV(PH5) | _BV(PH6)));
    }
    ICR4 = kTop;
    OCR4A = pulse_us;
    TCCR4A = 0;                         // OC4A connected by attach_wrist()
    TCCR4B = _BV(WGM43) | _BV(CS41);    // mode 8 (PFC, TOP = ICR4), clk/8
    // Pump and valve outputs, both off (R-11: the original left them floating until the first pump command).
    DDRG = static_cast<uint8_t>(DDRG | _BV(PG4) | _BV(PG5));
    pump_pins(false, false);
    // Gripper released (PC7 high), PA0 input; limit switch PG3 input with pull-up (uarm_swift.c:25-26).
    DDRC = static_cast<uint8_t>(DDRC | _BV(PC7));
    PORTC = static_cast<uint8_t>(PORTC | _BV(PC7));
    DDRA = static_cast<uint8_t>(DDRA & ~_BV(PA0));
    DDRG = static_cast<uint8_t>(DDRG & ~_BV(PG3));
    PORTG = static_cast<uint8_t>(PORTG | _BV(PG3));
#else
    pump_pins(false, false);
    host_gripper_released = true;
    host_laser = false;
#endif
    attach_wrist(true);  // the original starts the servo at boot (uarm_swift.c:19, uarm_peripheral.c:100-105)
}

void poll() {
    if (valve == Valve::kIdle) return;
    const uint32_t now = tick::now_ms();
    if (now - valve_since < 50) return;
    valve_since = now;
    if (valve == Valve::kOn) {
        valve = Valve::kOff;
        pump_pins(false, false);
    } else if (++valve_pulses >= 10) {
        valve = Valve::kIdle;
        pump_pins(false, true);  // ends with the valve on, as the original
    } else {
        valve = Valve::kOn;
        pump_pins(false, true);
    }
}

bool set_wrist(float angle) {
    if (isnan(angle) || angle < 0.0f || angle > 180.0f) return false;  // SP-23
    // (180 - angle) / 180 * 2000 us + 500 us, to the nearest microsecond (Q-12).
    pulse_us = static_cast<uint16_t>(lroundf((180.0f - angle) / 180.0f * 2000.0f + 500.0f));
#if defined(__AVR__)
    OCR4A = pulse_us;  // double-buffered at TOP in PFC mode: no glitch
#endif
    return true;
}

float wrist() { return 180.0f - (static_cast<float>(pulse_us) - 500.0f) / 2000.0f * 180.0f; }

void attach_wrist(bool on) {
    attached = on;
#if defined(__AVR__)
    if (on) TCCR4A = static_cast<uint8_t>(TCCR4A | _BV(COM4A1));
    else TCCR4A = static_cast<uint8_t>(TCCR4A & ~(_BV(COM4A1) | _BV(COM4A0)));  // Q-7: no pulses, pin low
#endif
}

bool wrist_attached() { return attached; }

void pump(bool on) {
    if (on) {
        valve = Valve::kIdle;
        pump_pins(true, false);
        pump_running = true;
        return;
    }
    if (!pump_running) return;  // off only acts if it was on (uarm_peripheral.c:690)
    pump_running = false;
    pump_pins(false, true);     // vent
    if (board::valve_pulses()) {  // V3.3.0 boards (all straps low)
        valve = Valve::kOn;
        valve_pulses = 0;
        valve_since = tick::now_ms();
    }
}

uint8_t pump_status() {  // uarm_peripheral.c:767-777
    if (!pump_pin()) return 0;
    const uint16_t v = adc::read(9);
    if (v < 10) return 0;
    return v <= 70 ? 2 : 1;
}

void gripper(bool catch_it) {
#if defined(__AVR__)
    set_bits(PORTC, _BV(PC7), !catch_it);
#else
    host_gripper_released = !catch_it;
#endif
}

uint8_t gripper_status() {  // uarm_peripheral.c:794-804
#if defined(__AVR__)
    if (PORTC & _BV(PC7)) return 0;
    return (PINA & _BV(PA0)) ? 1 : 2;
#else
    if (host_gripper_released) return 0;
    return host_pa0 ? 1 : 2;
#endif
}

void laser(bool on) {
#if defined(__AVR__)
    set_bits(PORTH, static_cast<uint8_t>(_BV(PH5) | _BV(PH6)), on);
#else
    host_laser = on;
#endif
}

bool laser_on() {
#if defined(__AVR__)
    return (PORTH & _BV(PH5)) != 0;
#else
    return host_laser;
#endif
}

bool limit_pressed() {
#if defined(__AVR__)
    return (PING & _BV(PG3)) == 0;
#else
    return host_limit;
#endif
}

bool power_present() { return adc::read(5) > 100; }

#if !defined(__AVR__)
void host_set_pa0(bool high) { host_pa0 = high; }
void host_set_limit(bool pressed) { host_limit = pressed; }
bool host_valve_on() { return host_valve; }
#endif

}  // namespace effector
