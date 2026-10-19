// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jack Sidman Smith
//
// Internal EEPROM, at the original's addresses (uarm_common.h:32-44; ARCHITECTURE section 10).

#pragma once

#include <stdint.h>

namespace nvm {

constexpr uint16_t kSize = 4096;

// Addresses (faithful uarm_common.h).
constexpr uint16_t kAngleReferOld = 800;   // uarm_common.h:33
constexpr uint16_t kAngleReferFlag = 818;  // uarm_common.h:34
constexpr uint16_t kAngleRefer = 820;      // uarm_common.h:32
constexpr uint16_t kWorkMode = 900;        // uarm_common.h:38, 1 byte
constexpr uint16_t kHeightOffset = 910;    // uarm_common.h:39, float
constexpr uint16_t kFrontOffset = 920;     // uarm_common.h:40, float
constexpr uint16_t kUserHeightOffset = 930;  // uarm_common.h:41, float
constexpr uint16_t kUserFrontOffset = 940;   // uarm_common.h:42, float
constexpr uint16_t kBleUid = 1002;         // uarm_common.h:35, 12 bytes
constexpr uint8_t kBleUidLength = 12;
constexpr uint16_t kEffectAngleOffset = 3202;  // uarm_common.h:44, float

void read(uint16_t addr, void* dst, uint8_t n);
uint8_t read_byte(uint16_t addr);

// Writes only the bytes that differ (EEPROM wear). Each byte waits for the previous write with interrupts on and
// blocks interrupts only for the 4-cycle write sequence (SP-39: the original blocked them for up to 3.4 ms per byte).
// A changed byte takes about 3.4 ms, so call this only from commands that save settings.
void write(uint16_t addr, const void* src, uint8_t n);
void write_byte(uint16_t addr, uint8_t v);

#if !defined(__AVR__)
// Host only: the simulated EEPROM, 0xFF (erased) after start-up.
uint8_t* host_image();
#endif

}  // namespace nvm
