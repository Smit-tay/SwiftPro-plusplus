// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jack Sidman Smith
//
// Bit-banged I2C master, one bus per encoder (faithful pins, uarm_iic.c:5-48), and one for the external EEPROM
// (uarm_iic.c:53-68 in V4.10.0; parity A9, Q-81):
//   L: SDA PC1, SCL PC3     R: SDA PH2, SCL PH7     B (base): SDA PD5, SCL PD6     X (EEPROM): SDA PB7, SCL PB6
// Change (SP-18): SDA is open-drain (released for 1, pulled low for 0), relying on the board's pull-ups (Q-9, measured
// on L). The original drove SDA high as an output. SCL stays push-pull, as in the original (single master). The EEPROM
// bus's pull-ups are UNVERIFIED, so its released SDA also has the AVR's internal pull-up on.
// Timing as the original: 5 us phases, so a byte takes about 0.15 ms.
//
// Each function is one short bus step; the encoder module runs one transaction (START to STOP) per main-loop pass.
// On the host the simulator implements these functions with an AS5600 model (test/sim.cpp).

#pragma once

#include <stdint.h>

namespace i2c {

enum class Bus : uint8_t { kL = 0, kR = 1, kB = 2, kX = 3 };  // the original's channels (ARML 0, ARMR 1, BASE 2)
constexpr uint8_t kBuses = 4;

void init();                         // all buses idle: SDA released, SCL high; a stuck slave is clocked free
void start(Bus b);                   // START or repeated START
void stop(Bus b);
bool write(Bus b, uint8_t byte);     // true if the slave ACKed
uint8_t read(Bus b, bool ack);       // ack: ACK (more bytes follow) or NACK (last byte)

}  // namespace i2c
