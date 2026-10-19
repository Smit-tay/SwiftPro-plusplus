// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jack Sidman Smith

#include "pins.h"

#include "atomic.h"

#if defined(__AVR__)
#include <avr/io.h>
#include <avr/pgmspace.h>
#endif

namespace pins {

namespace {

// Arduino pin -> port letter and bit, generated from framework-arduino-avr 5.2.0 variants/mega/pins_arduino.h
// (digital_pin_to_port_PGM, digital_pin_to_bit_mask_PGM). One byte each: the port letter's offset from 'A' in the high
// nibble, the bit in the low nibble.
constexpr uint8_t P(char port, uint8_t bit) { return static_cast<uint8_t>((port - 'A') << 4 | bit); }

#if defined(__AVR__)
const uint8_t kTable[kCount] PROGMEM = {
#else
const uint8_t kTable[kCount] = {
#endif
    P('E', 0), P('E', 1), P('E', 4), P('E', 5), P('G', 5), P('E', 3), P('H', 3), P('H', 4), P('H', 5), P('H', 6),  // 0-9
    P('B', 4), P('B', 5), P('B', 6), P('B', 7), P('J', 1), P('J', 0), P('H', 1), P('H', 0), P('D', 3), P('D', 2),  // 10-19
    P('D', 1), P('D', 0), P('A', 0), P('A', 1), P('A', 2), P('A', 3), P('A', 4), P('A', 5), P('A', 6), P('A', 7),  // 20-29
    P('C', 7), P('C', 6), P('C', 5), P('C', 4), P('C', 3), P('C', 2), P('C', 1), P('C', 0), P('D', 7), P('G', 2),  // 30-39
    P('G', 1), P('G', 0), P('L', 7), P('L', 6), P('L', 5), P('L', 4), P('L', 3), P('L', 2), P('L', 1), P('L', 0),  // 40-49
    P('B', 3), P('B', 2), P('B', 1), P('B', 0), P('F', 0), P('F', 1), P('F', 2), P('F', 3), P('F', 4), P('F', 5),  // 50-59
    P('F', 6), P('F', 7), P('K', 0), P('K', 1), P('K', 2), P('K', 3), P('K', 4), P('K', 5), P('K', 6), P('K', 7),  // 60-69
};

uint8_t entry(uint8_t pin) {
#if defined(__AVR__)
    return pgm_read_byte(&kTable[pin]);
#else
    return kTable[pin];
#endif
}

// Pins the firmware drives (Q-17, docs/ARCHITECTURE.md §2): arm STEP/DIR/EN PF0 PF1 PF2 PF6 PF7 PL1 PL3 PK0 PD7, micro-step
// PK2-PK4, encoder I2C PC1 PC3 (PH2, PH7, PD5, PD6 have no Arduino number), USART0-3 PE0 PE1 PD2 PD3 PH0 PH1 PJ0 PJ1,
// wrist PH3, buzzer PL5.
#if defined(__AVR__)
const uint8_t kOwned[] PROGMEM = {
#else
const uint8_t kOwned[] = {
#endif
    P('F', 0), P('F', 1), P('F', 2), P('F', 6), P('F', 7), P('L', 1), P('L', 3), P('K', 0), P('D', 7),
    P('K', 2), P('K', 3), P('K', 4), P('C', 1), P('C', 3),
    P('E', 0), P('E', 1), P('D', 2), P('D', 3), P('H', 0), P('H', 1), P('J', 0), P('J', 1),
    P('H', 3), P('L', 5),
};
// In work modes 4/5, also the effector stepper's DIR PH5, STEP PH6 and origin switch PK5 (uarm_peripheral.c:61-63, 148).
constexpr uint8_t kOwnedStepper[] = {P('H', 5), P('H', 6), P('K', 5)};

uint8_t owned_entry(uint8_t i) {
#if defined(__AVR__)
    return pgm_read_byte(&kOwned[i]);
#else
    return kOwned[i];
#endif
}

struct Regs {
    volatile uint8_t* port;
    volatile uint8_t* ddr;
    volatile uint8_t* pin;
};

#if defined(__AVR__)
Regs regs(uint8_t letter) {
    switch (letter) {
        case 0: return {&PORTA, &DDRA, &PINA};
        case 1: return {&PORTB, &DDRB, &PINB};
        case 2: return {&PORTC, &DDRC, &PINC};
        case 3: return {&PORTD, &DDRD, &PIND};
        case 4: return {&PORTE, &DDRE, &PINE};
        case 5: return {&PORTF, &DDRF, &PINF};
        case 6: return {&PORTG, &DDRG, &PING};
        case 7: return {&PORTH, &DDRH, &PINH};
        case 9: return {&PORTJ, &DDRJ, &PINJ};
        case 10: return {&PORTK, &DDRK, &PINK};
        default: return {&PORTL, &DDRL, &PINL};
    }
}
#else
HostPort host_ports[12];
// On the host the input level of an output pin is its PORT bit, as on the AVR.
volatile uint8_t host_pin_value;
Regs regs(uint8_t letter) {
    HostPort& h = host_ports[letter];
    host_pin_value = static_cast<uint8_t>((h.port & h.ddr) | (h.in & ~h.ddr));
    return {reinterpret_cast<volatile uint8_t*>(&h.port), reinterpret_cast<volatile uint8_t*>(&h.ddr), &host_pin_value};
}
#endif

void change(volatile uint8_t* r, uint8_t mask, bool on) {
    InterruptLock lock;  // ports F, H, K and L are also written by ISRs or by other modules (SP-28)
    *r = on ? static_cast<uint8_t>(*r | mask) : static_cast<uint8_t>(*r & ~mask);
}

}  // namespace

bool owned(uint8_t pin, bool stepper_effector) {
    if (pin >= kCount) return false;
    const uint8_t e = entry(pin);
    for (uint8_t i = 0; i < sizeof kOwned; ++i)
        if (owned_entry(i) == e) return true;
    if (stepper_effector)
        for (const uint8_t o : kOwnedStepper)
            if (o == e) return true;
    return false;
}

bool write(uint8_t pin, bool high) {  // digitalWrite: the PORT bit, whatever the direction
    if (pin >= kCount) return false;
    const uint8_t e = entry(pin);
    change(regs(static_cast<uint8_t>(e >> 4)).port, static_cast<uint8_t>(1u << (e & 7)), high);
    return true;
}

bool set_output(uint8_t pin, bool output) {  // pinMode OUTPUT / INPUT (INPUT also turns the pull-up off)
    if (pin >= kCount) return false;
    const uint8_t e = entry(pin);
    const Regs r = regs(static_cast<uint8_t>(e >> 4));
    const uint8_t mask = static_cast<uint8_t>(1u << (e & 7));
    InterruptLock lock;
    if (output) {
        *r.ddr = static_cast<uint8_t>(*r.ddr | mask);
    } else {
        *r.ddr = static_cast<uint8_t>(*r.ddr & ~mask);
        *r.port = static_cast<uint8_t>(*r.port & ~mask);
    }
    return true;
}

bool read(uint8_t pin, bool* high) {
    if (pin >= kCount) return false;
    const uint8_t e = entry(pin);
    *high = (*regs(static_cast<uint8_t>(e >> 4)).pin & (1u << (e & 7))) != 0;
    return true;
}

#if !defined(__AVR__)
HostPort& host_port(char letter) { return host_ports[letter - 'A']; }
#endif

}  // namespace pins
