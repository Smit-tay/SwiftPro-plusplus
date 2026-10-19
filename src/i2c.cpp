// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jack Sidman Smith

#include "i2c.h"

#if defined(__AVR__)

#include "atomic.h"

#include <avr/io.h>
#include <util/delay.h>

namespace i2c {

namespace {

// Pin operations for one bus. Port H is outside the bit-instruction range, so its read-modify-writes are locked
// (SP-28: an ISR may write PORTH later, e.g. the stepper effector). Ports C and D compile to single sbi/cbi
// instructions, but are locked too, for one rule everywhere; each lock is a few cycles.
struct Pins {
    volatile uint8_t& ddr;
    volatile uint8_t& port;
    volatile uint8_t& pin;
    uint8_t sda;
    uint8_t scl;
};

Pins pins(Bus b) {
    switch (b) {
        case Bus::kL: return {DDRC, PORTC, PINC, _BV(PC1), _BV(PC3)};  // uarm_iic.c:5-14
        case Bus::kR: return {DDRH, PORTH, PINH, _BV(PH2), _BV(PH7)};  // uarm_iic.c:21-30
        default: return {DDRD, PORTD, PIND, _BV(PD5), _BV(PD6)};      // uarm_iic.c:37-46
    }
}

void set(volatile uint8_t& r, uint8_t m) {
    InterruptLock lock;
    r = static_cast<uint8_t>(r | m);
}

void clear(volatile uint8_t& r, uint8_t m) {
    InterruptLock lock;
    r = static_cast<uint8_t>(r & ~m);
}

// Open drain: low = output with PORT 0; high = input, the pull-up raises the line.
void sda_low(const Pins& p) { set(p.ddr, p.sda); }
void sda_release(const Pins& p) { clear(p.ddr, p.sda); }
bool sda_read(const Pins& p) { return (p.pin & p.sda) != 0; }
void scl_high(const Pins& p) { set(p.port, p.scl); }
void scl_low(const Pins& p) { clear(p.port, p.scl); }

}  // namespace

void init() {
    for (uint8_t i = 0; i < 3; ++i) {
        const Pins p = pins(static_cast<Bus>(i));
        clear(p.port, p.sda);  // SDA's PORT bit stays 0 for good: only DDR moves it
        sda_release(p);
        scl_high(p);
        set(p.ddr, p.scl);
        // Bus clear (I2C specification, "bus clear"; not in the original): a reset in the middle of a transaction can
        // leave an AS5600 holding SDA low (a data bit or an ACK), and the first transaction after boot then fails
        // (SP-62, seen on the arm and reproduced under simavr). Clock until SDA is high while SCL is high (at most a
        // byte and its ACK slot), then START and STOP with SCL kept high: no further clock edge reaches the slave, and
        // START/STOP resets it. Lowering SCL for an ordinary STOP would give the slave another bit to drive.
        _delay_us(5);
        for (uint8_t k = 0; k < 9 && !sda_read(p); ++k) {
            scl_low(p);
            _delay_us(5);
            scl_high(p);
            _delay_us(5);
        }
        sda_low(p);      // START (SCL high)
        _delay_us(5);
        sda_release(p);  // STOP
        _delay_us(5);
    }
}

void start(Bus b) {  // uarm_iic.c:250-259
    const Pins p = pins(b);
    sda_release(p);
    scl_high(p);
    _delay_us(5);
    sda_low(p);
    _delay_us(5);
    scl_low(p);
    _delay_us(2);
}

void stop(Bus b) {  // uarm_iic.c:261-270
    const Pins p = pins(b);
    scl_low(p);
    sda_low(p);
    _delay_us(5);
    scl_high(p);
    _delay_us(5);
    sda_release(p);
    _delay_us(5);
}

bool write(Bus b, uint8_t byte) {  // send_byte (uarm_iic.c:323-345) + wait_ack (:272-295)
    const Pins p = pins(b);
    for (uint8_t i = 0; i < 8; ++i) {
        scl_low(p);
        if (byte & 0x80) sda_release(p);
        else sda_low(p);
        _delay_us(5);
        scl_high(p);
        _delay_us(5);
        scl_low(p);
        byte = static_cast<uint8_t>(byte << 1);
        _delay_us(5);
    }
    // ACK: release SDA, clock high, then poll SDA for a low level up to 250 times, as the original does
    // (uarm_iic.c:275-291). The original ignored the result (SP-15); here it is returned.
    sda_release(p);
    _delay_us(2);
    scl_high(p);
    _delay_us(2);
    bool ack = false;
    for (uint8_t i = 0; i < 250 && !ack; ++i) ack = !sda_read(p);
    scl_low(p);
    _delay_us(2);
    return ack;
}

uint8_t read(Bus b, bool ack) {  // read_byte + ack/nack (uarm_iic.c:297-321, 347-374)
    const Pins p = pins(b);
    uint8_t v = 0;
    sda_release(p);
    _delay_us(2);
    for (uint8_t i = 0; i < 8; ++i) {
        scl_low(p);
        _delay_us(5);
        scl_high(p);
        v = static_cast<uint8_t>(v << 1);
        _delay_us(5);
        if (sda_read(p)) v |= 1;
        _delay_us(5);
    }
    scl_low(p);
    if (ack) sda_low(p);
    else sda_release(p);
    _delay_us(2);
    scl_high(p);
    _delay_us(5);
    scl_low(p);
    _delay_us(2);
    sda_release(p);
    _delay_us(2);
    return v;
}

}  // namespace i2c

#endif  // __AVR__
