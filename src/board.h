// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jack Sidman Smith
//
// Board identity: the hardware version straps on PJ3-PJ6, read once at boot (faithful uarm_swift.c:11,
// uarm_peripheral.c:628-650).
//
// The original also wrote Grbl settings $3 (direction invert mask) and $4 to EEPROM on every boot from the straps.
// The port has no Grbl settings: the mask is a constant chosen from the straps, and nothing is written.

#pragma once

#include <stdint.h>

namespace board {

void init();  // reads the straps

// "V3.3.0", "V3.3.1", "V3.3.2" or "V0.0.0" (P2202).
const char* hardware_version();

// Stepper direction invert mask: 7 on V3.3.0 boards, else 0 (faithful uarm_peripheral.c:634,638,642,646).
uint8_t direction_invert_mask();

// The pump valve is pulsed after the pump stops: on boards with all straps low (V3.3.0) (uarm_peripheral.c:723).
bool valve_pulses();

#if !defined(__AVR__)
// Host only: the value PINJ & 0x78 will read at the next init().
void host_set_straps(uint8_t pinj_masked);
#endif

}  // namespace board
