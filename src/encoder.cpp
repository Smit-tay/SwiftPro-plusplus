// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jack Sidman Smith

#include "encoder.h"

#include "i2c.h"
#include "nvm.h"

namespace encoder {

namespace {

constexpr uint8_t kAddressWrite = 0x36 << 1;  // uarm_angle.c:62 (0x36 << 1)
constexpr uint8_t kAddressRead = kAddressWrite | 1;
constexpr uint8_t kRegStatus = 0x0B;  // uarm_angle.c:85
constexpr uint8_t kRegAngle = 0x0E;   // uarm_angle.c:62 (0x0E high, 0x0F low)
constexpr uint8_t kRegAgc = 0x1A;     // AS5600 datasheet (SP-42; not in the original, UNVERIFIED)
constexpr uint8_t kRegMagnitude = 0x1B;

constexpr float kPointB[3] = {32.758930f, 88.980537f, 90.0f};  // L, R, base (uarm_angle.c:36-38)

struct Read {
    uint8_t reg;
    uint8_t count;  // 1 or 2 bytes
};

constexpr Read kAngleReads[3] = {{kRegAngle, 2}, {kRegAngle, 2}, {kRegAngle, 2}};  // 3 samples, median
constexpr Read kHealthReads[3] = {{kRegStatus, 1}, {kRegAgc, 1}, {kRegMagnitude, 2}};
constexpr Read kWatchReads[4] = {{kRegAngle, 2}, {kRegAngle, 2}, {kRegAngle, 2}, {kRegStatus, 1}};

// Bus steps of one read: START, address+W, register, repeated START, address+R, the bytes, STOP.
enum class Step : uint8_t { kStart, kAddrW, kReg, kRestart, kAddrR, kByte0, kByte1, kStop };

uint16_t references[3] = {};

bool running = false;
Joint joint = Joint::kL;
Job job = Job::kAngle;
uint8_t read_index = 0;
Step step = Step::kStart;
uint8_t bytes[2] = {};
uint16_t values[4] = {};
bool failed = false;

bool done_ok = false;
uint16_t done_raw = 0;
uint8_t done_status = 0, done_agc = 0;
uint16_t done_magnitude = 0;

i2c::Bus bus() { return static_cast<i2c::Bus>(joint); }
const Read& current() {
    switch (job) {
        case Job::kAngle: return kAngleReads[read_index];
        case Job::kHealth: return kHealthReads[read_index];
        default: return kWatchReads[read_index];
    }
}
uint8_t read_count() { return job == Job::kWatch ? 4 : 3; }

void finish() {
    running = false;
    done_ok = !failed;
    if (job != Job::kHealth) {
        // Median of three as the original: sum - max - min, in 16 bits (uarm_angle.c:63-79).
        uint16_t max = values[0], min = values[0], sum = values[0];
        for (uint8_t i = 1; i < 3; ++i) {
            if (values[i] > max) max = values[i];
            if (values[i] < min) min = values[i];
            sum = static_cast<uint16_t>(sum + values[i]);
        }
        const uint16_t median = static_cast<uint16_t>(sum - max - min);
        // A 12-bit reading above 4095 is a bus error (R-3); the original returned 0, a plausible wrong angle (SP-15).
        if (median > 4095) done_ok = false;
        done_raw = median;
        if (job == Job::kWatch) done_status = static_cast<uint8_t>(values[3]);
    } else {
        done_status = static_cast<uint8_t>(values[0]);
        done_agc = static_cast<uint8_t>(values[1]);
        done_magnitude = values[2];
    }
}

// After a NACK: release the bus and end the job as failed.
void fail() {
    failed = true;
    step = Step::kStop;
}

}  // namespace

void init() {
    i2c::init();
    // read_angle_reference_param (uarm_angle.c:233-251): struct {b, l, r} of uint16, little-endian.
    const uint16_t addr = nvm::read_byte(nvm::kAngleReferFlag) == 0xBB ? nvm::kAngleRefer : nvm::kAngleReferOld;
    uint8_t buf[6];
    nvm::read(addr, buf, sizeof buf);
    references[static_cast<uint8_t>(Joint::kB)] = static_cast<uint16_t>(buf[0] | buf[1] << 8);
    references[static_cast<uint8_t>(Joint::kL)] = static_cast<uint16_t>(buf[2] | buf[3] << 8);
    references[static_cast<uint8_t>(Joint::kR)] = static_cast<uint16_t>(buf[4] | buf[5] << 8);
    running = false;
    done_ok = false;
}

bool start(Joint j, Job k) {
    if (running) return false;
    running = true;
    joint = j;
    job = k;
    read_index = 0;
    step = Step::kStart;
    failed = false;
    return true;
}

bool busy() { return running; }

namespace {

void bus_step() {
    const Read& r = current();
    switch (step) {
        case Step::kStart:
            i2c::start(bus());
            step = Step::kAddrW;
            break;
        case Step::kAddrW:
            if (i2c::write(bus(), kAddressWrite)) step = Step::kReg;
            else fail();
            break;
        case Step::kReg:
            if (i2c::write(bus(), r.reg)) step = Step::kRestart;
            else fail();
            break;
        case Step::kRestart:
            i2c::start(bus());
            step = Step::kAddrR;
            break;
        case Step::kAddrR:
            if (i2c::write(bus(), kAddressRead)) step = Step::kByte0;
            else fail();
            break;
        case Step::kByte0:
            bytes[0] = i2c::read(bus(), r.count > 1);
            step = r.count > 1 ? Step::kByte1 : Step::kStop;
            break;
        case Step::kByte1:
            bytes[1] = i2c::read(bus(), false);
            step = Step::kStop;
            break;
        case Step::kStop:
            i2c::stop(bus());
            if (failed) {
                finish();
                break;
            }
            values[read_index] = r.count > 1 ? static_cast<uint16_t>(bytes[0] << 8 | bytes[1]) : bytes[0];
            if (++read_index == read_count()) finish();
            else step = Step::kStart;
            break;
    }
}

}  // namespace

// One whole I2C transaction (START to STOP, about 0.7 ms) per call: one bus step per main-loop pass made every step
// pay for a whole pass, and a joint read took about 6 ms (R-16, measured under simavr).
void poll() {
    if (!running) return;
    Step before;
    do {
        before = step;
        bus_step();
    } while (running && before != Step::kStop);
}

void run(Joint j, Job k) {
    start(j, k);
    while (running) poll();
}

bool ok() { return done_ok; }
uint16_t raw() { return done_raw; }
uint8_t status() { return done_status; }
uint8_t agc() { return done_agc; }
uint16_t magnitude() { return done_magnitude; }

bool health_fault(bool answered, uint8_t s) {
    if (!answered) return true;               // SP-42: no ACK
    if ((s & kMD) == 0) return true;          // SP-42: no magnet
    return (s & 0x18) == 0x18;                // faithful uarm_angle.c:85-98
}

uint16_t reference(Joint j) { return references[static_cast<uint8_t>(j)]; }

void set_reference(Joint j, uint16_t raw_count) { references[static_cast<uint8_t>(j)] = raw_count; }

float angle(Joint j, uint16_t raw_count) {
    // calculate_current_angle (uarm_angle.c:136-224), non-UARM_2500 branch.
    int32_t refer = references[static_cast<uint8_t>(j)];
    const int32_t r = raw_count;
    if (refer - r > 2048) refer -= 4096;
    else if (refer - r < -2048) refer += 4096;
    const int32_t offset = refer - r;
    return kPointB[static_cast<uint8_t>(j)] - static_cast<float>(offset) * 360.0f / 4096.0f;
}

}  // namespace encoder
