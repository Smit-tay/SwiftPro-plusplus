// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jack Sidman Smith

#include "xeeprom.h"

#include "i2c.h"

namespace xeeprom {

namespace {

constexpr uint8_t kAddressWrite = 0xA0;  // EXTERNAL_EEPROM_USER_ADDRESS (uarm_common.h in V4.10.0)
constexpr uint8_t kAddressRead = 0xA1;
constexpr i2c::Bus kBus = i2c::Bus::kX;

// START, the device address, the two address bytes. False (and STOP) on a missing ACK.
bool begin(uint16_t addr) {
    i2c::start(kBus);
    if (i2c::write(kBus, kAddressWrite) && i2c::write(kBus, static_cast<uint8_t>(addr >> 8)) &&
        i2c::write(kBus, static_cast<uint8_t>(addr & 0xFF)))
        return true;
    i2c::stop(kBus);
    return false;
}

}  // namespace

bool read(uint16_t addr, uint8_t* buf, uint8_t n) {
    if (!begin(addr)) return false;
    i2c::start(kBus);
    if (!i2c::write(kBus, kAddressRead)) {
        i2c::stop(kBus);
        return false;
    }
    for (uint8_t i = 0; i < n; ++i) buf[i] = i2c::read(kBus, i + 1 < n);
    i2c::stop(kBus);
    return true;
}

bool write(uint16_t addr, const uint8_t* buf, uint8_t n) {
    if (!begin(addr)) return false;
    bool ok = true;
    for (uint8_t i = 0; i < n && ok; ++i) ok = i2c::write(kBus, buf[i]);
    i2c::stop(kBus);
    return ok;
}

bool ready() {
    i2c::start(kBus);
    const bool ack = i2c::write(kBus, kAddressWrite);
    i2c::stop(kBus);
    return ack;
}

}  // namespace xeeprom
