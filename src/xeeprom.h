// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jack Sidman Smith
//
// The external I2C EEPROM at 0xA0 on its own bus (PB7/PB6, i2c::Bus::kX), for M2211/M2212 N1 and learning mode
// (parity A9, Q-81; V4.10.0 uarm_peripheral.c:472-620, uarm_iic.c:196-250). 64 KiB, 128-byte pages (the original's
// page arithmetic, uarm_peripheral.c:509-520; the chip type itself is UNVERIFIED, as is its presence on your board).
//
// Changes: a missing ACK is reported (the original replied with whatever it read, SP-15); a write never blocks: the
// chip's write cycle (up to 5 ms) is waited for by polling ready(), where the original slept 5 ms per byte. Each call
// is one bus transaction, about 0.15 ms per byte.

#pragma once

#include <stdint.h>

namespace xeeprom {

constexpr uint32_t kSize = 65536;
constexpr uint8_t kPage = 128;

// n bytes from addr. False if the chip didn't ACK (absent, or still writing).
bool read(uint16_t addr, uint8_t* buf, uint8_t n);
// n bytes to addr, all within one page (the caller splits). False if the chip didn't ACK. The chip then writes them
// internally: ready() is false until it is done.
bool write(uint16_t addr, const uint8_t* buf, uint8_t n);
// The chip ACKs its address: present and not in a write cycle.
bool ready();

}  // namespace xeeprom
