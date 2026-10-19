// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jack Sidman Smith

#include "board.h"

#if defined(__AVR__)
#include <avr/io.h>
#endif

namespace board {

namespace {

constexpr uint8_t kStrapMask = 0x78;  // PJ6..PJ3, uarm_peripheral.c:629

const char* version = "V0.0.0";
uint8_t direction_mask = 0;
bool straps_low = false;

#if defined(__AVR__)
uint8_t read_straps() {
    DDRJ = static_cast<uint8_t>(DDRJ & ~kStrapMask);  // inputs, no pull-up (faithful uarm_peripheral.c:630)
    return static_cast<uint8_t>(PINJ & kStrapMask);
}
#else
uint8_t host_straps = 0x40;  // V3.3.1, as on your arm
uint8_t read_straps() { return host_straps; }
#endif

}  // namespace

#if !defined(__AVR__)
void host_set_straps(uint8_t pinj_masked) { host_straps = static_cast<uint8_t>(pinj_masked & kStrapMask); }
#endif

void init() {
    const uint8_t straps = read_straps();
    straps_low = straps == 0;
    switch (straps) {  // faithful uarm_peripheral.c:631-647
        case 0x40: version = "V3.3.1"; direction_mask = 0; break;
        case 0x00: version = "V3.3.0"; direction_mask = 7; break;
        case 0x20: version = "V3.3.2"; direction_mask = 0; break;
        default: version = "V0.0.0"; direction_mask = 0; break;
    }
}

const char* hardware_version() { return version; }

uint8_t direction_invert_mask() { return direction_mask; }

bool valve_pulses() { return straps_low; }

}  // namespace board
