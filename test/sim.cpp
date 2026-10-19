// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jack Sidman Smith
//
// Host simulator: the real firmware modules, with simulated EEPROM, straps, time and USART0.
// Inputs that no module reads yet are validated and stored, so traces can set them already.

#include "sim.h"

#include "../src/adc.h"
#include "../src/app.h"
#include "../src/board.h"
#include "../src/commands.h"
#include "../src/effector.h"
#include "../src/i2c.h"
#include "../src/nvm.h"
#include "../src/protocol.h"
#include "../src/report.h"
#include "../src/pins.h"
#include "../src/tool.h"
#include "../src/serial.h"
#include "../src/stepper.h"
#include "../src/tick.h"

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <string>

namespace {

std::string out;  // everything transmitted, not yet taken

// AS5600 model, one per bus (L, R, B). Register map from the AS5600 datasheet (UNVERIFIED in the original source):
// 0x0B STATUS, 0x0E/0x0F ANGLE (high, low), 0x1A AGC, 0x1B/0x1C MAGNITUDE (high, low). Reads auto-increment.
struct As5600 {
    uint16_t angle = 0;
    uint8_t status = 0x20;   // MD
    uint8_t agc = 128;       // simulated values for P2246; no hardware meaning
    uint16_t magnitude = 2000;
    bool nack = false;
    // Bus state
    bool expect_address = false;
    bool addressed_write = false;
    bool expect_pointer = false;
    uint8_t pointer = 0;

    uint8_t reg(uint8_t r) const {
        switch (r) {
            case 0x0B: return status;
            case 0x0E: return static_cast<uint8_t>(angle >> 8);
            case 0x0F: return static_cast<uint8_t>(angle & 0xFF);
            case 0x1A: return agc;
            case 0x1B: return static_cast<uint8_t>(magnitude >> 8);
            case 0x1C: return static_cast<uint8_t>(magnitude & 0xFF);
            default: return 0;
        }
    }
};
As5600 chips[3];  // L, R, B: the same order as the motion axes

// Each encoder follows its motor: 4096 counts per 360 deg, 80 steps per deg (28800 steps per turn). A stalled
// encoder keeps its count. Setting enc.raw moves the joint (by hand) to that count.
struct Follow {
    uint16_t raw_base = 0;
    int32_t pulses_base = 0;
    bool stalled = false;
};
Follow follow[3];

int32_t pulses(uint8_t j) {
    int32_t p[3];
    stepper::host_pulses(p);
    return p[j];
}

uint16_t raw_now(uint8_t j) {
    const Follow& f = follow[j];
    if (f.stalled) return f.raw_base;
    const long counts = std::lround(static_cast<double>(pulses(j) - f.pulses_base) * 4096.0 / 28800.0);
    return static_cast<uint16_t>(((f.raw_base + counts) % 4096 + 4096) % 4096);
}

void set_raw(uint8_t j, uint16_t raw) {
    follow[j].raw_base = raw;
    follow[j].pulses_base = pulses(j);
}

As5600& chip(i2c::Bus b) {
    const uint8_t j = static_cast<uint8_t>(b);
    chips[j].angle = raw_now(j);
    return chips[j];
}

// sim key letter -> bus
int bus_of(char letter) { return letter == 'L' ? 0 : letter == 'R' ? 1 : letter == 'B' ? 2 : -1; }

void write_refs(uint16_t b, uint16_t l, uint16_t r) {
    uint8_t* e = nvm::host_image();
    const uint16_t v[3] = {b, l, r};  // struct order b, l, r, little-endian (uarm_angle.c:7-11)
    for (int i = 0; i < 3; ++i) {
        e[nvm::kAngleRefer + 2 * i] = static_cast<uint8_t>(v[i] & 0xFF);
        e[nvm::kAngleRefer + 2 * i + 1] = static_cast<uint8_t>(v[i] >> 8);
    }
    e[nvm::kAngleReferFlag] = 0xBB;
}

void write_float(uint16_t addr, float f) { std::memcpy(nvm::host_image() + addr, &f, sizeof f); }

// Default inputs (sim.h).
void defaults() {
    uint8_t* e = nvm::host_image();
    std::memset(e, 0xFF, nvm::kSize);
    std::memcpy(e + nvm::kBleUid, "000000000000", nvm::kBleUidLength);
    e[nvm::kWorkMode] = 0;
    // Mode 0 default offsets, as stored on your arm (P2220 matched them, probe 2026-10-17; uarm_common.h:90-91).
    write_float(nvm::kFrontOffset, 56.65f);
    write_float(nvm::kHeightOffset, 74.55f);
    write_refs(0, 0, 0);
    board::host_set_straps(0x40);  // V3.3.1
    adc::host_set(5, 512);   // motor power present
    adc::host_set(9, 0);     // pump sense
    effector::host_set_pa0(true);
    effector::host_set_limit(false);
    report::host_set_button(0, false);
    tool::host_set_switch(false);
    for (char l = 'A'; l <= 'L'; ++l)
        if (l != 'I') pins::host_port(l) = pins::HostPort{};
    report::host_set_button(1, false);
    for (As5600& c : chips) c = As5600{};
    for (uint8_t j = 0; j < 3; ++j) {
        follow[j] = Follow{};
        set_raw(j, 0);
    }
}

// Parses "B<n> L<n> R<n>" (validated by is_joint_triple) into v[bus].
void joint_triple(std::string_view s, long v[3]) {
    size_t pos = 0;
    while (pos < s.size()) {
        const char letter = s[pos];
        const size_t sp = s.find(' ', pos);
        const std::string num(s.substr(pos + 1, sp == std::string_view::npos ? std::string_view::npos : sp - pos - 1));
        v[bus_of(letter)] = std::strtol(num.c_str(), nullptr, 10);
        pos = sp == std::string_view::npos ? s.size() : sp + 1;
    }
}

void drain_tx() {
    uint8_t c;
    while (serial::tx_pop(&c)) out += static_cast<char>(c);
}

bool run_loop() {
    bool any = false;
    for (int i = 0; i < 100000 && app::poll(); ++i) any = true;
    drain_tx();
    return any;
}

bool is_int(std::string_view s, long lo, long hi) {
    if (s.empty()) return false;
    const std::string t(s);
    char* end = nullptr;
    const long v = std::strtol(t.c_str(), &end, 10);
    return *end == '\0' && v >= lo && v <= hi;
}

// "B<n> L<n> R<n>" with each n in lo..hi.
bool is_joint_triple(std::string_view s, long lo, long hi) {
    const char letters[3] = {'B', 'L', 'R'};
    size_t pos = 0;
    for (int i = 0; i < 3; ++i) {
        if (pos >= s.size() || s[pos] != letters[i]) return false;
        const size_t sp = s.find(' ', pos);
        const std::string_view num = s.substr(pos + 1, sp == std::string_view::npos ? std::string_view::npos : sp - pos - 1);
        if (!is_int(num, lo, hi)) return false;
        if (i < 2 && sp == std::string_view::npos) return false;
        pos = sp == std::string_view::npos ? s.size() : sp + 1;
    }
    return pos == s.size();
}

}  // namespace

// The i2c bus functions, implemented by the AS5600 model.
namespace i2c {
void init() {}
void start(Bus b) {
    As5600& c = chip(b);
    c.expect_address = true;
    c.expect_pointer = false;
}
void stop(Bus b) {
    As5600& c = chip(b);
    c.expect_address = false;
    c.expect_pointer = false;
}
bool write(Bus b, uint8_t byte) {
    As5600& c = chip(b);
    if (c.nack) return false;
    if (c.expect_address) {
        c.expect_address = false;
        if ((byte >> 1) != 0x36) return false;
        c.addressed_write = (byte & 1) == 0;
        c.expect_pointer = c.addressed_write;
        return true;
    }
    if (c.expect_pointer) {
        c.pointer = byte;
        c.expect_pointer = false;
    }
    return true;
}
uint8_t read(Bus b, bool) {
    As5600& c = chip(b);
    if (c.nack) return 0xFF;  // nobody drives SDA: the pull-up reads 1
    return c.reg(c.pointer++);
}
}  // namespace i2c

bool sim_available() { return true; }

void sim_reset() {
    out.clear();
    defaults();
}

bool sim_set(std::string_view key, std::string_view value) {
    if (key == "hw") {
        if (value == "V3.3.1") board::host_set_straps(0x40);
        else if (value == "V3.3.0") board::host_set_straps(0x00);
        else if (value == "V3.3.2") board::host_set_straps(0x20);
        else if (value == "V0.0.0") board::host_set_straps(0x78);
        else return false;
        return true;
    }
    if (key == "uid") {
        if (value == "none") {
            std::memset(nvm::host_image() + nvm::kBleUid, 0xFF, nvm::kBleUidLength);
            return true;
        }
        if (value.size() != nvm::kBleUidLength) return false;
        std::memcpy(nvm::host_image() + nvm::kBleUid, value.data(), nvm::kBleUidLength);
        return true;
    }
    if (key == "mode") {
        if (!is_int(value, 0, 8)) return false;
        nvm::host_image()[nvm::kWorkMode] = static_cast<uint8_t>(std::atoi(std::string(value).c_str()));
        return true;
    }
    if (key == "enc.ref" || key == "enc.raw" || key == "enc.status") {
        if (!is_joint_triple(value, 0, key == "enc.status" ? 255 : 4095)) return false;
        long v[3] = {};
        joint_triple(value, v);
        if (key == "enc.ref") write_refs(static_cast<uint16_t>(v[2]), static_cast<uint16_t>(v[0]), static_cast<uint16_t>(v[1]));
        for (uint8_t i = 0; i < 3; ++i) {
            if (key == "enc.raw") set_raw(i, static_cast<uint16_t>(v[i]));
            if (key == "enc.status") chips[i].status = static_cast<uint8_t>(v[i]);
        }
        return true;
    }
    if (key == "enc.nack") {
        if (value != "none" && (value.empty() || value.find_first_not_of("BLR") != std::string_view::npos)) return false;
        for (As5600& c : chips) c.nack = false;
        if (value != "none")
            for (const char letter : value) chips[bus_of(letter)].nack = true;
        return true;
    }
    if (key == "enc.slip") {  // <B|L|R><counts>: the encoder jumps by that many counts and goes on following
        if (value.size() < 2 || bus_of(value[0]) < 0 || !is_int(value.substr(1), -4095, 4095)) return false;
        const uint8_t j = static_cast<uint8_t>(bus_of(value[0]));
        const long n = std::strtol(std::string(value.substr(1)).c_str(), nullptr, 10);
        set_raw(j, static_cast<uint16_t>(((raw_now(j) + n) % 4096 + 4096) % 4096));
        return true;
    }
    if (key == "enc.stall") {
        if (value != "none" && value != "B" && value != "L" && value != "R") return false;
        for (uint8_t j = 0; j < 3; ++j) {  // freeze or release at the current count
            const uint16_t now = raw_now(j);
            follow[j].stalled = value != "none" && bus_of(value[0]) == j;
            set_raw(j, now);
        }
        return true;
    }
    if (key == "power" || key == "limit" || key == "gripper.pa0") {
        if (value != "0" && value != "1") return false;
        const bool on = value == "1";
        if (key == "power") adc::host_set(5, on ? 512 : 0);
        else if (key == "limit") effector::host_set_limit(on);
        else effector::host_set_pa0(on);
        return true;
    }
    if (key == "pump.adc") {
        if (!is_int(value, 0, 1023)) return false;
        adc::host_set(9, static_cast<uint16_t>(std::atoi(std::string(value).c_str())));
        return true;
    }
    if (key == "tool.switch") {  // the stepper effector's origin switch (PK5)
        if (value != "0" && value != "1") return false;
        tool::host_set_switch(value == "1");
        return true;
    }
    if (key.size() == 6 && key.rfind("pin.", 0) == 0 && key[4] >= 'A' && key[4] <= 'L' && key[4] != 'I' &&
        key[5] >= '0' && key[5] <= '7') {  // the external level on a port pin, e.g. pin.A0 1
        if (value != "0" && value != "1") return false;
        pins::HostPort& h = pins::host_port(key[4]);
        const uint8_t m = static_cast<uint8_t>(1u << (key[5] - '0'));
        h.in = value == "1" ? static_cast<uint8_t>(h.in | m) : static_cast<uint8_t>(h.in & ~m);
        return true;
    }
    if (key == "button") {
        if (value != "menu down" && value != "menu up" && value != "play down" && value != "play up") return false;
        report::host_set_button(value[0] == 'm' ? 0 : 1, value.substr(5) == "down");
        return true;
    }
    return false;
}

void sim_boot() {
    app::init();
    drain_tx();
}

void sim_rx(std::string_view bytes) {
    for (const char c : bytes) serial::on_rx(0, static_cast<uint8_t>(c));
}

void sim_settle() { run_loop(); }

void sim_advance_ms(uint32_t ms) {
    for (uint32_t i = 0; i < ms; ++i) {
        tick::on_tick();
        run_loop();
        stepper::host_advance(2000);  // 1 ms of Timer1 ticks
        tool::host_advance(1000);
        run_loop();
    }
}

bool sim_idle() { return commands::idle() && protocol::queue_empty(); }

std::string sim_take_tx() {
    drain_tx();
    std::string t;
    t.swap(out);
    return t;
}
