// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jack Sidman Smith

#include "nvm.h"

#include "atomic.h"

#if defined(__AVR__)
#include <avr/eeprom.h>
#endif

namespace nvm {

#if defined(__AVR__)

void read(uint16_t addr, void* dst, uint8_t n) {
    eeprom_read_block(dst, reinterpret_cast<const void*>(addr), n);
}

uint8_t read_byte(uint16_t addr) { return eeprom_read_byte(reinterpret_cast<const uint8_t*>(addr)); }

void write_byte(uint16_t addr, uint8_t v) {
    if (read_byte(addr) == v) return;
    while (EECR & _BV(EEPE)) {}  // the previous write, with interrupts on
    EEAR = addr;
    EEDR = v;
    EECR = 0;  // erase + write
    InterruptLock lock;  // EEMPE must be followed by EEPE within 4 cycles
    EECR = _BV(EEMPE);
    EECR = static_cast<uint8_t>(_BV(EEMPE) | _BV(EEPE));
}

void write(uint16_t addr, const void* src, uint8_t n) {
    const uint8_t* s = static_cast<const uint8_t*>(src);
    for (uint8_t i = 0; i < n; ++i) write_byte(static_cast<uint16_t>(addr + i), s[i]);
}

#else

namespace {
uint8_t image[kSize] = {};
bool erased = false;
}  // namespace

uint8_t* host_image() {
    if (!erased) {
        for (uint8_t& b : image) b = 0xFF;
        erased = true;
    }
    return image;
}

void read(uint16_t addr, void* dst, uint8_t n) {
    uint8_t* d = static_cast<uint8_t*>(dst);
    for (uint8_t i = 0; i < n; ++i) d[i] = host_image()[(addr + i) % kSize];
}

uint8_t read_byte(uint16_t addr) { return host_image()[addr % kSize]; }

void write_byte(uint16_t addr, uint8_t v) { host_image()[addr % kSize] = v; }

void write(uint16_t addr, const void* src, uint8_t n) {
    const uint8_t* s = static_cast<const uint8_t*>(src);
    for (uint8_t i = 0; i < n; ++i) write_byte(static_cast<uint16_t>(addr + i), s[i]);
}

#endif

}  // namespace nvm
