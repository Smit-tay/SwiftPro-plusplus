// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jack Sidman Smith

#include "commands.h"

#include "adc.h"
#include "board.h"
#include "buzzer.h"
#include "effector.h"
#include "encoder.h"
#include "kin.h"
#include "motion.h"
#include "nvm.h"
#include "pins.h"
#include "report.h"
#include "stepper.h"
#include "tick.h"
#include "tool.h"

#include <math.h>
#include <string.h>

#if defined(__AVR__)
#include <avr/pgmspace.h>
#endif

namespace commands {

bool usable(float v);

namespace {


uint8_t mode = 0;                          // EEPROM 900, one byte (faithful uarm_peripheral.c:333-338)
char uid[nvm::kBleUidLength + 1] = {};     // EEPROM 1002, 12 bytes (faithful uarm_peripheral.c:367-372)
bool motion_done_report = false;           // M2122: report @9 when a move ends
bool stream = false;                       // M2124 (Q-32)
kin::Offsets offsets{};                    // EEPROM 920 / 910 (uarm_peripheral.c:340-352)
constexpr float kDefaultFront = 56.65f;    // mode 0 defaults (uarm_common.h:116-117)
constexpr float kDefaultHeight = 74.55f;
// Default offsets {height, front} per work mode, for M2400 (the Swift Pro block, uarm_common.h:116-138). Mode 7 (user)
// reads EEPROM 930/940 instead.
#if defined(__AVR__)
const kin::Offsets kModeOffsets[9] PROGMEM = {
#else
const kin::Offsets kModeOffsets[9] = {
#endif
    {56.65f, 74.55f},  // 0 normal (suction cup)
    {64.4f, 51.04f},   // 1 laser
    {56.5f, 74.43f},   // 2 3D printer
    {69.5f, 43.0f},    // 3 pen
    {76.5f, 39.0f},    // 4 stepper effector, flat
    {76.0f, 73.4f},    // 5 stepper effector, standard
    {54.5f, 77.2f},    // 6 touch pen
    {56.65f, 74.55f},  // 7 user: 930/940, these if they are unusable (R-7)
    {44.5f, 25.49f},   // 8 test
};
float effect_offset = 0.0f;  // EEPROM 3202, the stepper effector's angle offset (M2412)
bool tool_locked = true;     // M2203 N3 in modes 4/5 (the original's motor_state_bits 0x08)

bool run_flag = false;                     // a move ran since the last @9 (uarm_protocol.c:216-221)
// Board events (uarm_protocol.c:192-214). Both start "off", so a powered arm reports @5 V1 just after boot, as the
// original does. The limit switch is debounced over 10 ms without blocking (SP-17: the original waited 10 ms).
bool power_reported = false, power_pending = false;
bool limit_reported = false, limit_pending = false, limit_seen = false;
uint32_t limit_since = 0, last_sample = 0, last_power = 0;
float feed = 100.0f;                       // modal F, F100 after boot (Q-55)

// A set of encoder reads, joint by joint (the original's order, L, R, B: uarm_protocol.c:1359-1361). There are two:
// one for queries (async), one for re-syncs (sync); poll() gives the bus to one at a time.
struct Reads {
    bool active = false;  // has reads to do or results not yet collected
    bool done = false;    // all reads finished
    encoder::Job job = encoder::Job::kAngle;
    uint16_t number = 0;  // the command (queries)
    uint8_t count = 0;
    uint8_t index = 0;
    encoder::Joint joints[3] = {};
    bool ok[3] = {};
    uint16_t raw[3] = {};
    uint8_t status[3] = {}, agc[3] = {};
    uint16_t magnitude[3] = {};
};
Reads async_reads, sync_reads;
Reads* bus_owner = nullptr;  // the set whose read is on the bus now

void begin_reads(Reads& r, uint16_t number, encoder::Job job, uint8_t count, const encoder::Joint* joints) {
    r = Reads{};
    r.active = true;
    r.number = number;
    r.job = job;
    r.count = count;
    for (uint8_t i = 0; i < count; ++i) r.joints[i] = joints[i];
}

// The lost-step watch (SP-41, Q-26). When the bus is free, the joints are read in turn, B, L, R, at most one read
// started per millisecond. Each reading is kept for @3 (Q-33). A locked joint is checked while motion runs and once
// more after it ends: the encoder must lie within kLostDeg of the range the step tally covered during the read (taken
// just before and just after it), so motion during the read can't cause a false alarm. kLostCount faults in a row on
// one joint stop the arm (as S1100), send @8, and re-sync the tally from the encoders once it has stopped (Q-6).
// A reading that failed, or whose magnet is missing (SP-42), is not checked and resets the count.
// M2125 V0 turns the check off (Q-61); it is on after every boot and not saved. The readings go on either way.
constexpr float kLostDeg = 1.0f;
constexpr uint8_t kLostCount = 2;
constexpr uint8_t kWatchOrder[3] = {2, 0, 1};  // motion axes: B, L, R

struct Measured {
    bool ok = false;
    float deg = 0.0f;  // tally frame: base in the -90 frame
    uint32_t at = 0;   // ms, when the read finished
};
Measured measured_now[3];
bool watch_on = true;      // M2125 (Q-61)
bool watching = false;     // the watch's read is on the bus
uint8_t watch_next = 0;    // index into kWatchOrder
uint8_t watch_axis = 0;
bool watch_check = false;  // this read is to be checked
bool watch_still = false;  // the tally didn't change during the read: a final check
int32_t watch_steps[3] = {};
float watch_before[3] = {};
uint32_t watch_started = 0;
bool check_due[3] = {};    // the axis still needs checking (set while the stepper runs)
uint8_t faults[3] = {};
bool lost = false;         // waiting for take_fault()
bool lost_report = false;  // @8 waiting
uint8_t lost_axis = 0;
float lost_error = 0.0f;
enum class Recover : uint8_t { kNone, kWait, kRead };
Recover recover = Recover::kNone;

bool watch_start() {
    const uint32_t now = tick::now_ms();
    if (now == watch_started) return false;
    watch_started = now;
    watch_axis = kWatchOrder[watch_next];
    watch_next = static_cast<uint8_t>((watch_next + 1) % 3);
    stepper::position(watch_steps);
    const uint8_t joint = watch_axis == 2 ? 0 : static_cast<uint8_t>(watch_axis + 1);
    watch_check = watch_on && check_due[watch_axis] && motion::known() && motion::locked(joint) && recover == Recover::kNone;
    motion::tally_angles(watch_before);
    encoder::start(static_cast<encoder::Joint>(watch_axis), encoder::Job::kWatch);
    watching = true;
    return true;
}

// Q-69 (SP-62): while some locked joint's position is unknown (a failed boot read), take it from the background
// readings as soon as every such joint has a healthy one, at rest, as the boot would have. Released joints wait for
// M2201 or M17.
void auto_sync() {
    if (!motion::idle() || recover != Recover::kNone || lost) return;
    uint8_t mask = 0;
    float angles[3] = {};
    for (uint8_t a = 0; a < 3; ++a) {
        const uint8_t joint = a == 2 ? 0 : static_cast<uint8_t>(a + 1);
        if ((motion::known_joints() & (1u << a)) || !motion::locked(joint)) continue;
        if (!measured_now[a].ok) return;
        angles[a] = measured_now[a].deg + (a == 2 ? 90.0f : 0.0f);  // the readings keep B in the -90 frame
        mask = static_cast<uint8_t>(mask | (1u << a));
    }
    if (mask != 0) motion::sync(mask, angles);
}

void lost_steps(uint8_t a, float error) {
    lost = true;
    lost_report = true;
    lost_axis = a;
    lost_error = error;
    for (uint8_t k = 0; k < 3; ++k) {
        faults[k] = 0;
        check_due[k] = false;
    }
}

void watch_finish() {
    watching = false;
    int32_t steps[3];
    stepper::position(steps);
    watch_still = steps[0] == watch_steps[0] && steps[1] == watch_steps[1] && steps[2] == watch_steps[2];
    const uint8_t a = watch_axis;
    const encoder::Joint j = static_cast<encoder::Joint>(a);
    const bool good = !encoder::health_fault(encoder::ok(), encoder::status());
    Measured& m = measured_now[a];
    m.ok = good;
    m.at = tick::now_ms();
    if (good) m.deg = encoder::angle(j, encoder::raw()) - (j == encoder::Joint::kB ? 90.0f : 0.0f);
    if (!watch_check) {
        if (watch_still) check_due[a] = false;
        return;
    }
    if (!good) {
        faults[a] = 0;
        if (watch_still) check_due[a] = false;
        return;
    }
    float after[3];
    motion::tally_angles(after);
    const float lo = fminf(watch_before[a], after[a]), hi = fmaxf(watch_before[a], after[a]);
    if (m.deg >= lo - kLostDeg && m.deg <= hi + kLostDeg) {
        faults[a] = 0;
        if (watch_still) check_due[a] = false;
        return;
    }
    if (++faults[a] < kLostCount) return;
    lost_steps(a, after[a] - m.deg);
}

// One step of encoder work (main loop). Returns true if it did anything.
bool poll_reads() {
    if (watching) {
        encoder::poll();
        if (!encoder::busy()) {
            watch_finish();
            auto_sync();
        }
        return true;
    }
    if (bus_owner != nullptr) {
        encoder::poll();
        if (encoder::busy()) return true;
        Reads& r = *bus_owner;
        r.ok[r.index] = encoder::ok();
        r.raw[r.index] = encoder::raw();
        r.status[r.index] = encoder::status();
        r.agc[r.index] = encoder::agc();
        r.magnitude[r.index] = encoder::magnitude();
        if (++r.index == r.count) r.done = true;
        bus_owner = nullptr;
        return true;
    }
    Reads* const sets[2] = {&async_reads, &sync_reads};  // queries first
    for (Reads* r : sets) {
        if (r->active && !r->done) {
            encoder::start(r->joints[r->index], r->job);
            bus_owner = r;
            return true;
        }
    }
    return watch_start();
}

// The sync job in progress.
enum class Job : uint8_t { kNone, kResync, kLine, kDelay, kSearch, kReference, kHeight };
Job sync_job = Job::kNone;
uint8_t resync_mask = 0;  // motion axes (bit 0 L, 1 R, 2 B)
uint32_t delay_start = 0, delay_ms = 0;

constexpr encoder::Joint kLRB[3] = {encoder::Joint::kL, encoder::Joint::kR, encoder::Joint::kB};

// save_sys_param (uarm_peripheral.c:387-420): mode 900, front 920, height 910, effector angle offset 3202.
void save_sys() {
    nvm::write_byte(nvm::kWorkMode, mode);
    nvm::write(nvm::kFrontOffset, &offsets.front, sizeof offsets.front);
    nvm::write(nvm::kHeightOffset, &offsets.height, sizeof offsets.height);
    nvm::write(nvm::kEffectAngleOffset, &effect_offset, sizeof effect_offset);
}

// save_user_endoffest (uarm_peripheral.c:422-442): front 940, height 930.
void save_user() {
    nvm::write(nvm::kUserFrontOffset, &offsets.front, sizeof offsets.front);
    nvm::write(nvm::kUserHeightOffset, &offsets.height, sizeof offsets.height);
}

// M2400 (uarm_protocol.c:968-1031): the mode, its default offsets, saved. Changes (SP-57, Q-63): the wrist
// servo keeps its state between servo modes; modes 4/5 release it and start the effector stepper's origin search, and
// leaving them attaches it again.
Status set_mode(uint8_t m) {
    const bool was_tool = tool::active();
    mode = m;
#if defined(__AVR__)
    memcpy_P(&offsets, &kModeOffsets[m], sizeof offsets);
#else
    offsets = kModeOffsets[m];
#endif
    if (m == 7) {  // read_user_endoffest (uarm_peripheral.c:444-464)
        kin::Offsets user{};
        nvm::read(nvm::kUserFrontOffset, &user.front, sizeof user.front);
        nvm::read(nvm::kUserHeightOffset, &user.height, sizeof user.height);
        if (usable(user.front) && usable(user.height)) offsets = user;  // R-7
    }
    motion::set_offsets(offsets);
    save_sys();
    if (m == 4 || m == 5) {
        effector::attach_wrist(false);
        tool::activate(m);
        tool_locked = true;
        tool::search(effect_offset);
        sync_job = Job::kSearch;
        return Status::kPending;
    }
    if (was_tool) {
        tool::deactivate();
        effector::attach_wrist(true);
    }
    return Status::kOk;
}

// N3 lock and release: the wrist servo, or in modes 4/5 the effector stepper (released = stopped).
void lock_effector(bool on) {
    if (!tool::active()) {
        effector::attach_wrist(on);
        return;
    }
    tool_locked = on;
    if (!on) tool::stop();
}

bool effector_locked() { return tool::active() ? tool_locked : effector::wrist_attached(); }

// "V0" / "V1" style single-digit parameter, and nothing after it (Q-53).
bool digit_param(const char* p, char letter, char lo, char hi, uint8_t* value) {
    if (p[0] != letter || p[1] < lo || p[1] > hi || p[2] != '\0') return false;
    *value = static_cast<uint8_t>(p[1] - '0');
    return true;
}

Status start_query(uint16_t number, encoder::Job job, uint8_t count, const encoder::Joint* joints) {
    begin_reads(async_reads, number, job, count, joints);
    return Status::kPending;
}

Reads& query_state = async_reads;

// Results by joint, whatever order they were read in.
uint8_t slot_of(encoder::Joint j) {
    for (uint8_t i = 0; i < query_state.count; ++i)
        if (query_state.joints[i] == j) return i;
    return 0;
}

float joint_angle(encoder::Joint j) {
    const uint8_t i = slot_of(j);
    return encoder::angle(j, query_state.raw[i]);
}

bool all_ok() {
    for (uint8_t i = 0; i < query_state.count; ++i)
        if (!query_state.ok[i]) return false;
    return true;
}

// Formats a finished query (faithful formats: uarm_protocol.c:1355-1460, 1512-1564).
Status finish_query(fmt::Writer& tail) {
    using encoder::Joint;
    const uint16_t n = query_state.number;
    if (n == 2244) {  // mask 4 = L, 2 = R, 1 = B; E26 with the mask when any is set (:1523-1564, 1654-1659)
        uint8_t mask = 0;
        if (encoder::health_fault(query_state.ok[slot_of(Joint::kL)], query_state.status[slot_of(Joint::kL)])) mask |= 4;
        if (encoder::health_fault(query_state.ok[slot_of(Joint::kR)], query_state.status[slot_of(Joint::kR)])) mask |= 2;
        if (encoder::health_fault(query_state.ok[slot_of(Joint::kB)], query_state.status[slot_of(Joint::kB)])) mask |= 1;
        tail.put(" V");
        tail.u16(mask);
        return mask == 0 ? Status::kOk : Status::kE26;
    }
    if (n == 2246) {  // new (SP-42, Q-27, Q-45): per joint STATUS,AGC,MAGNITUDE, or "-" if the chip didn't answer
        const char letters[3] = {'B', 'L', 'R'};
        const Joint order[3] = {Joint::kB, Joint::kL, Joint::kR};
        for (uint8_t k = 0; k < 3; ++k) {
            const uint8_t i = slot_of(order[k]);
            tail.put(' ');
            tail.put(letters[k]);
            if (!query_state.ok[i]) {
                tail.put('-');
                continue;
            }
            tail.u16(query_state.status[i]);
            tail.put(',');
            tail.u16(query_state.agc[i]);
            tail.put(',');
            tail.u16(query_state.magnitude[i]);
        }
        return Status::kOk;
    }
    if (!all_ok()) return Status::kE26;  // SP-15: the original printed a plausible wrong value with "ok"
    switch (n) {
        case 2200:  // " B%s L%s R%s", B in the user frame (:1355-1368)
            tail.put(" B");
            tail.f4(joint_angle(Joint::kB));
            tail.put(" L");
            tail.f4(joint_angle(Joint::kL));
            tail.put(" R");
            tail.f4(joint_angle(Joint::kR));
            return Status::kOk;
        case 2206:  // " V%s" (:1391-1417)
            tail.put(" V");
            tail.f4(joint_angle(query_state.joints[0]));
            return Status::kOk;
        case 2243:  // raw counts " B%d L%d R%d" (:1518-1521)
            tail.put(" B");
            tail.u16(query_state.raw[slot_of(Joint::kB)]);
            tail.put(" L");
            tail.u16(query_state.raw[slot_of(Joint::kL)]);
            tail.put(" R");
            tail.u16(query_state.raw[slot_of(Joint::kR)]);
            return Status::kOk;
        case 2220:
        case 2221: {  // :1419-1460
            float x, y, z;
            const float b = joint_angle(Joint::kB) - 90.0f;
            kin::forward(joint_angle(Joint::kL), joint_angle(Joint::kR), b, &x, &y, &z);
            kin::arm_to_effector(offsets, &x, &y, &z);
            if (n == 2220) {
                tail.put(" X");
                tail.f4(x);
                tail.put(" Y");
                tail.f4(y);
                tail.put(" Z");
                tail.f4(z);
            } else {
                const float length = sqrtf(x * x + y * y);
                tail.put(" S");
                tail.f4(length);
                tail.put(" R");
                tail.f4(b + 90.0f);  // angle_b += 90 (:1451)
                tail.put(" H");
                tail.f4(z);
            }
            return Status::kOk;
        }
        default: return Status::kE20;
    }
}

Status query(uint16_t number, const char* p, fmt::Writer& tail) {
    if (number == 2241) {  // analog input N0-N15 (uarm_protocol.c:1496-1509)
        int16_t ch = 0;
        if (!parse::expect(&p, 'N') || !parse::integer(&p, &ch) || *p != '\0' || ch < 0 || ch > 15) return Status::kE21;
        tail.put(" V");
        tail.u16(adc::read(static_cast<uint8_t>(ch)));
        return Status::kOk;
    }
    if (number == 2240) {  // digital input N0-N69 (uarm_protocol.c:1482-1494); never touches a timer (SP-58)
        int16_t pin = 0;
        bool high = false;
        if (!parse::expect(&p, 'N') || !parse::integer(&p, &pin) || *p != '\0' || pin < 0 ||
            !pins::read(static_cast<uint8_t>(pin), &high))
            return Status::kE21;
        tail.put(high ? " V1" : " V0");
        return Status::kOk;
    }
    if (number != 2206 && p[0] != '\0') return Status::kE21;  // Q-53: these take no parameters
    switch (number) {
        case 2201: tail.put(FLASH(" VSwiftPro")); return Status::kOk;  // uarm_swift.h:13, uarm_protocol.c:1370-1372
        // The version strings carry their own "V" (" %s", uarm_protocol.c:1374-1384).
        case 2202: tail.put(' '); tail.put(board::hardware_version()); return Status::kOk;
        case 2203: tail.put(FLASH(" V5.0.0-pp.1")); return Status::kOk;  // Q-2 (the original: V4.9.1-jss.7)
        case 2204: tail.put(FLASH(" V4.0.5")); return Status::kOk;       // uarm_swift.h:18
        case 2205: tail.put(" V"); tail.put(uid); return Status::kOk;  // :1386-1388; empty if none stored (Q-58)
        case 2400: tail.put(" V"); tail.u16(mode); return Status::kOk;                     // :1566-1568
        case 2242:  // reference counts, " B%d L%d R%d" (:1512-1516)
            tail.put(" B");
            tail.u16(encoder::reference(encoder::Joint::kB));
            tail.put(" L");
            tail.u16(encoder::reference(encoder::Joint::kL));
            tail.put(" R");
            tail.u16(encoder::reference(encoder::Joint::kR));
            return Status::kOk;
        case 2231: tail.put(" V"); tail.u16(effector::pump_status()); return Status::kOk;     // :1462-1464
        case 2232: tail.put(" V"); tail.u16(effector::gripper_status()); return Status::kOk;  // :1466-1468
        case 2233: tail.put(effector::limit_pressed() ? " V1" : " V0"); return Status::kOk;   // :1470-1472
        case 2234: tail.put(effector::power_present() ? " V1" : " V0"); return Status::kOk;   // :1474-1476
        case 2235: tail.put(effector::laser_on() ? " V1" : " V0"); return Status::kOk;        // :1478-1480
        case 2200:
        case 2220:
        case 2221:
        case 2243: return start_query(number, encoder::Job::kAngle, 3, kLRB);
        case 2244:
        case 2246: return start_query(number, encoder::Job::kHealth, 3, kLRB);
        case 2206: {  // N0-N3, and nothing after it (Q-53)
            uint8_t n = 0;
            if (!digit_param(p, 'N', '0', '3', &n)) return Status::kE21;
            if (n == 3) {  // the wrist's angle as output (Q-12), or the stepper effector's (modes 4/5, :1410-1414)
                tail.put(" V");
                tail.f4(effector_angle());
                return Status::kOk;
            }
            // N0 base, N1 L, N2 R
            const encoder::Joint j = n == 0 ? encoder::Joint::kB : n == 1 ? encoder::Joint::kL : encoder::Joint::kR;
            return start_query(number, encoder::Job::kAngle, 1, &j);
        }
        default: return Status::kE20;  // SP-12: the original replied "ok"
    }
}

// Three letter-prefixed numbers, e.g. "X1Y2Z3", each at most 19 characters (the original's buffers, SP-03).
bool three(const char*& p, const char (&letters)[4], float* v) {
    for (uint8_t i = 0; i < 3; ++i)
        if (!parse::expect(&p, letters[i]) || !parse::number(&p, 19, &v[i])) return false;
    return true;
}

// Kinematics queries (uarm_protocol.c:817-905).
Status kinematics(uint16_t number, const char* p, fmt::Writer& tail) {
    float v[3];
    if (number == 2221) {  // B L R (B in the user frame) -> effector X Y Z
        if (!three(p, "BLR", v) || *p != '\0') return Status::kE21;
        float x, y, z;
        kin::forward(v[1], v[2], v[0] - 90.0f, &x, &y, &z);
        kin::arm_to_effector(offsets, &x, &y, &z);
        tail.put(" X");
        tail.f4(x);
        tail.put(" Y");
        tail.f4(y);
        tail.put(" Z");
        tail.f4(z);
        return Status::kOk;
    }
    if (!three(p, "XYZ", v)) return Status::kE21;
    float x = v[0], y = v[1], z = v[2];
    if (number == 2222) {  // reachability; P0 = X Y Z, P1 = polar S R H (R in the user frame)
        int16_t polar = 0;
        if (!parse::expect(&p, 'P') || !parse::integer(&p, &polar) || *p != '\0' || polar < 0 || polar > 1)
            return Status::kE21;
        if (polar == 1) {
            const float length = v[0], heading = (v[1] - 90.0f) / 57.29578f;
            x = length * cosf(heading);
            y = length * sinf(heading);
            z = v[2];
        }
    } else if (*p != '\0') {
        return Status::kE21;
    }
    float a, b, base;
    kin::effector_to_arm(offsets, &x, &y, &z);
    kin::inverse(x, y, z, &a, &b, &base);
    if (number == 2222) {
        tail.put(kin::legal(a, b, base) ? " V1" : " V0");
        return Status::kOk;
    }
    tail.put(" B");  // M2220: no legality check, as the original
    tail.f4(base + 90.0f);
    tail.put(" L");
    tail.f4(a);
    tail.put(" R");
    tail.f4(b);
    return Status::kOk;
}

Status setting(uint16_t number, const char* p) {
    uint8_t v = 0;
    switch (number) {
        case 2122:  // uarm_protocol.c:1154-1158, 549-560
            if (!digit_param(p, 'V', '0', '1', &v)) return Status::kE21;
            motion_done_report = v == 1;
            run_flag = false;
            return Status::kOk;
        case 2213:  // SP-44, Q-29: V0 = buttons report @4, V1 = default function (none in this port)
            if (!digit_param(p, 'V', '0', '1', &v)) return Status::kE21;  // Q-53 (the original accepted anything)
            report::set_button_reports(v == 0);
            return Status::kOk;
        case 2125:  // the lost-step check on/off (Q-61)
            if (!digit_param(p, 'V', '0', '1', &v)) return Status::kE21;
            watch_on = v == 1;
            for (uint8_t& f : faults) f = 0;
            return Status::kOk;
        case 2124:  // SP-45, Q-32
            if (!digit_param(p, 'V', '0', '1', &v)) return Status::kE21;
            stream = v == 1;
            return Status::kOk;
        default: return Status::kE20;
    }
}

Status result_status(motion::Result r) {
    switch (r) {
        case motion::Result::kOk: return Status::kOk;
        case motion::Result::kBadParam: return Status::kE21;
        case motion::Result::kUnreachable: return Status::kE22;
        case motion::Result::kReleased: return Status::kE25;
        case motion::Result::kNoPosition: return Status::kE26;
        case motion::Result::kChecking: return Status::kPending;
    }
    return Status::kE20;
}

// A motion command was accepted (or is being checked): remember it for @9, and wait for a line check.
Status moved(motion::Result r) {
    if (r == motion::Result::kOk || r == motion::Result::kChecking) run_flag = true;
    if (r == motion::Result::kChecking) sync_job = Job::kLine;
    return result_status(r);
}

// Letter-prefixed numbers in a fixed order (G22xx), then an optional F, then the end of the line (Q-53, Q-55).
bool fields(const char* p, const char* letters, uint8_t n, float* v, float* f) {
    for (uint8_t i = 0; i < n; ++i)
        if (!parse::expect(&p, letters[i]) || !parse::number(&p, 19, &v[i])) return false;
    *f = feed;
    if (*p == 'F' && (!parse::expect(&p, 'F') || !parse::number(&p, 19, f))) return false;
    return *p == '\0';
}

// G0/G1 words X, Y, Z, F in any order, each at most once; missing axes are NaN (Q-54), F is modal (Q-55).
bool words(const char* p, float xyz[3], float* f) {
    for (uint8_t i = 0; i < 3; ++i) xyz[i] = NAN;
    *f = feed;
    uint8_t seen = 0;
    while (*p != '\0') {
        const char letter = *p;
        uint8_t i;
        switch (letter) {
            case 'X': i = 0; break;
            case 'Y': i = 1; break;
            case 'Z': i = 2; break;
            case 'F': i = 3; break;
            default: return false;
        }
        if (seen & (1u << i)) return false;
        seen = static_cast<uint8_t>(seen | (1u << i));
        ++p;
        float v;
        if (!parse::number(&p, 19, &v)) return false;
        if (i == 3) *f = v;
        else xyz[i] = v;
    }
    return true;
}

// F is modal: a valid F given with a move becomes the default for the next moves.
void keep_feed(float f) {
    if (f > 0.0f) feed = f;
}

Status gcode(uint16_t number, const char* p) {
    float v[3], f;
    switch (number) {
        case 0:
        case 1: {
            if (!words(p, v, &f)) return Status::kE21;
            const Status s = moved(motion::move_xyz(v, number == 1, f));
            if (s == Status::kOk || s == Status::kPending) keep_feed(f);
            return s;
        }
        case 2201:  // polar S R H, R in the user frame
        case 2205: {  // relative polar
            if (!fields(p, "SRH", 3, v, &f)) return Status::kE21;
            const Status s = moved(motion::move_polar(v[0], v[1], v[2], number == 2205, f));
            if (s == Status::kOk || s == Status::kPending) keep_feed(f);
            return s;
        }
        case 2204: {  // relative X Y Z
            if (!fields(p, "XYZ", 3, v, &f)) return Status::kE21;
            const Status s = moved(motion::move_xyz_relative(v, f));
            if (s == Status::kOk || s == Status::kPending) keep_feed(f);
            return s;
        }
        case 2206: {  // B L R, B in the user frame
            if (!fields(p, "BLR", 3, v, &f)) return Status::kE21;
            const float a[3] = {v[1], v[2], v[0]};
            const Status s = moved(motion::move_joints(a, f));
            if (s == Status::kOk) keep_feed(f);
            return s;
        }
        case 2202: {  // one joint: N0 base, N1 L, N2 R; N3 is the wrist (effector module)
            uint8_t n = 0;
            if (p[0] != 'N' || p[1] < '0' || p[1] > '3') return Status::kE21;
            n = static_cast<uint8_t>(p[1] - '0');
            if (!fields(p + 2, "V", 1, v, &f)) return Status::kE21;
            if (n == 3) {  // the effector; F is not used (uarm_protocol.c:303-306)
                if (tool::active()) {  // modes 4/5: 0-360; the reply waits for the move (effect_ldie, :148, :175)
                    if (tool::origin() != tool::Origin::kFound) return Status::kE26;  // Q-66
                    return tool::move_to(v[0]) ? Status::kOk : Status::kE21;
                }
                return effector::set_wrist(v[0]) ? Status::kOk : Status::kE21;  // 0-180 (SP-23)
            }
            float a[3] = {NAN, NAN, NAN};
            a[n == 0 ? 2 : n - 1] = v[0];
            const Status s = moved(motion::move_joints(a, f));
            if (s == Status::kOk) keep_feed(f);
            return s;
        }
        case 2004: {  // delay before the next sync command; P <= 0 is no delay (Q-50, SP-55)
            int16_t ms = 0;
            if (!parse::expect(&p, 'P') || !parse::integer(&p, &ms) || *p != '\0') return Status::kE21;
            if (ms <= 0) return Status::kOk;
            sync_job = Job::kDelay;
            delay_start = tick::now_ms();
            delay_ms = static_cast<uint32_t>(ms);
            return Status::kPending;
        }
        default: return Status::kE20;
    }
}

// Starts a re-sync of the joints in mask (motion axes: bit 0 L, 1 R, 2 B) from the encoders.
Status resync(uint8_t mask) {
    encoder::Joint joints[3];
    uint8_t n = 0;
    for (uint8_t i = 0; i < 3; ++i)
        if (mask & (1u << i)) joints[n++] = kLRB[i];
    begin_reads(sync_reads, 0, encoder::Job::kAngle, n, joints);
    resync_mask = mask;
    sync_job = Job::kResync;
    return Status::kPending;
}

// Joint numbers as in G2202: 0 base, 1 L, 2 R -> motion axes (0 L, 1 R, 2 B).
uint8_t axis_of(uint8_t joint) { return joint == 0 ? 2 : static_cast<uint8_t>(joint - 1); }

Status mcode(uint16_t number, const char* p, fmt::Writer& tail) {
    uint8_t n = 0;
    switch (number) {
        case 17:  // lock all and re-read the position from the encoders (SP-07, SP-15)
            if (*p != '\0') return Status::kE21;
            lock_effector(true);
            return resync(7);
        case 18:
        case 2019:  // release all, the wrist too (Q-7)
            if (*p != '\0') return Status::kE21;
            for (uint8_t j = 0; j < 3; ++j) motion::release(j);
            lock_effector(false);
            return Status::kOk;
        case 2201:  // lock one joint, re-reading its encoder (SP-08, Q-6); N3 = the wrist
            if (!digit_param(p, 'N', '0', '3', &n)) return Status::kE21;
            if (n == 3) {
                lock_effector(true);
                return Status::kOk;
            }
            return resync(static_cast<uint8_t>(1u << axis_of(n)));
        case 2202:  // release one joint (SP-08); N3 = the wrist, really released (Q-7)
            if (!digit_param(p, 'N', '0', '3', &n)) return Status::kE21;
            if (n == 3) lock_effector(false);
            else motion::release(n);
            return Status::kOk;
        case 2203:  // is the joint locked? N as in G2202 (SP-08; the original mixed up the numbering)
            if (!digit_param(p, 'N', '0', '3', &n)) return Status::kE21;
            tail.put((n == 3 ? effector_locked() : motion::locked(n)) ? " V1" : " V0");
            return Status::kOk;
        case 2231:  // pump (uarm_protocol.c:1245-1258)
            if (!digit_param(p, 'V', '0', '1', &n)) return Status::kE21;
            effector::pump(n == 1);
            return Status::kOk;
        case 2232:  // gripper: V1 catch, V0 release
            if (!digit_param(p, 'V', '0', '1', &n)) return Status::kE21;
            effector::gripper(n == 1);
            return Status::kOk;
        case 2233:  // laser
            if (!digit_param(p, 'V', '0', '1', &n)) return Status::kE21;
            if (tool::active()) return Status::kE20;  // PH5/PH6 drive the effector stepper (SP-61, Q-67)
            effector::laser(n == 1);
            return Status::kOk;
        case 2210: {  // buzzer: F in Hz, T in ms (SP-24)
            float hz;
            int16_t ms;
            if (!parse::expect(&p, 'F') || !parse::number(&p, 9, &hz) || !parse::expect(&p, 'T') ||
                !parse::integer(&p, &ms) || *p != '\0' || ms <= 0)
                return Status::kE21;
            return buzzer::tone(hz, static_cast<uint16_t>(ms)) ? Status::kOk : Status::kE21;
        }
        case 2200:  // moving? Answered at once (Q-56)
            if (*p != '\0') return Status::kE21;
            tail.put(motion::idle() ? " V0" : " V1");
            return Status::kOk;
        case 204: {  // acceleration, A * 80 deg/s^2 per joint, saved (Q-52, SP-37)
            float a;
            if (!parse::expect(&p, 'A') || !parse::number(&p, 19, &a) || *p != '\0') return Status::kE21;
            return motion::set_accel(a) ? Status::kOk : Status::kE21;
        }
        case 2240:    // pin level (uarm_protocol.c:932-947)
        case 2241: {  // pin direction (:949-966)
            int16_t pin = 0, value = 0;
            if (!parse::expect(&p, 'N') || !parse::integer(&p, &pin) || !parse::expect(&p, 'V') ||
                !parse::integer(&p, &value) || *p != '\0' || pin < 0 || pin >= pins::kCount || value < 0 || value > 1)
                return Status::kE21;
            const uint8_t pn = static_cast<uint8_t>(pin);
            if (pins::owned(pn, tool::active())) return Status::kE21;  // Q-17
            if (number == 2240) pins::write(pn, value == 1);
            else pins::set_output(pn, value == 1);
            return Status::kOk;
        }
        case 2400:  // work mode (uarm_protocol.c:968-1031)
            if (!digit_param(p, 'S', '0', '8', &n)) return Status::kE21;
            return set_mode(n);
        case 2401:  // the encoder reference ("point B") from the current position (uarm_angle.c:273-279)
            if (*p != '\0') return Status::kE21;
            begin_reads(sync_reads, 0, encoder::Job::kAngle, 3, kLRB);
            sync_job = Job::kReference;
            return Status::kPending;
        case 2410:  // height offset = the arm's current Z from the encoders (uarm_protocol.c:1039-1058)
            if (*p != '\0') return Status::kE21;
            begin_reads(sync_reads, 0, encoder::Job::kAngle, 3, kLRB);
            sync_job = Job::kHeight;
            return Status::kPending;
        case 2411:    // front offset S (uarm_protocol.c:1060-1076)
        case 2413: {  // height offset H (:1095-1110)
            float v;
            if (!parse::expect(&p, number == 2411 ? 'S' : 'H') || !parse::number(&p, 19, &v) || *p != '\0' ||
                !usable(v))
                return Status::kE21;
            if (number == 2411) offsets.front = v;
            else offsets.height = v;
            motion::set_offsets(offsets);
            save_sys();
            save_user();
            return Status::kOk;
        }
        case 2412: {  // effector angle offset V, then a new origin search in modes 4/5 (uarm_protocol.c:1078-1093)
            float v;
            if (!parse::expect(&p, 'V') || !parse::number(&p, 19, &v) || *p != '\0' || isnan(v) || v < -360.0f ||
                v > 360.0f)
                return Status::kE21;
            effect_offset = v;
            save_sys();
            if (!tool::active()) return Status::kOk;
            tool::search(effect_offset);
            sync_job = Job::kSearch;
            return Status::kPending;
        }
        case 2205: {  // UID: "VUB" + 1-12 characters, any (uarm_protocol.c:689-700 took [0-9.]; SP-56, Q-62)
            if (p[0] != 'V' || p[1] != 'U' || p[2] != 'B') return Status::kE21;
            const char* s = p + 3;
            const size_t len = strlen(s);  // the line assembler has already dropped spaces and upper-cased letters
            if (len == 0 || len > nvm::kBleUidLength) return Status::kE21;
            char buf[nvm::kBleUidLength] = {};
            memcpy(buf, s, len);
            nvm::write(nvm::kBleUid, buf, nvm::kBleUidLength);
            memcpy(uid, buf, nvm::kBleUidLength);
            uid[nvm::kBleUidLength] = '\0';
            return Status::kOk;
        }
        case 2120: {  // @3 period in seconds (uarm_protocol.c:535-547)
            float seconds;
            if (!parse::expect(&p, 'V') || !parse::number(&p, 19, &seconds) || *p != '\0') return Status::kE21;
            return report::set_period(seconds) ? Status::kOk : Status::kE21;
        }
        case 2121:  // uarm_protocol.c:1150-1152
            if (*p != '\0') return Status::kE21;
            report::stop();
            return Status::kOk;
        default:
            if (number >= 2220 && number <= 2222) return kinematics(number, p, tail);
            return setting(number, p);
    }
}

Status scode(uint16_t number, const char* p) {
    uint8_t v = 0;
    if (number == 1000) {  // feed hold / resume
        if (!digit_param(p, 'V', '0', '1', &v)) return Status::kE21;
        motion::hold(v == 0);
        return Status::kOk;
    }
    return Status::kE20;  // S1100 is handled by the protocol (it also answers the queue)
}

}  // namespace

// A stored offset is usable if it's a number of sensible size; erased EEPROM reads as NaN (R-7).
bool usable(float v) { return !isnan(v) && v > -1000.0f && v < 1000.0f; }

void init() {
    adc::init();
    buzzer::init();
    buzzer::tone(1000.0f, 260);  // the boot beep (uarm_swift.c:14)
    effector::init();
    encoder::init();
    motion::init();
    mode = nvm::read_byte(nvm::kWorkMode);
    nvm::read(nvm::kFrontOffset, &offsets.front, sizeof offsets.front);
    nvm::read(nvm::kHeightOffset, &offsets.height, sizeof offsets.height);
    if (mode > 8 || !usable(offsets.front) || !usable(offsets.height)) {  // R-7: erased or damaged EEPROM
        mode = 0;
        offsets = kin::Offsets{kDefaultFront, kDefaultHeight};
    }
    nvm::read(nvm::kEffectAngleOffset, &effect_offset, sizeof effect_offset);
    if (isnan(effect_offset) || effect_offset > 360.0f) effect_offset = 0.0f;  // faithful uarm_peripheral.c:360-362
    nvm::read(nvm::kBleUid, uid, nvm::kBleUidLength);
    uid[nvm::kBleUidLength] = '\0';
    for (char& ch : uid)  // Q-58: an erased UID (0xFF) prints as nothing
        if (static_cast<uint8_t>(ch) == 0xFF) ch = '\0';
    motion::set_offsets(offsets);
    // Modes 4/5: the effector stepper instead of the wrist servo, and its origin search, in the background (SP-26;
    // the original blocked serial input meanwhile, uarm_swift.c:19, 28, protocol.c:77-79).
    tool::init();
    tool_locked = true;
    if (mode == 4 || mode == 5) {
        effector::attach_wrist(false);
        tool::activate(mode);
        tool::search(effect_offset);
    }
    motion_done_report = false;
    report::init();
    for (Measured& m : measured_now) m = Measured{};
    watching = lost = lost_report = false;
    watch_on = true;
    watch_next = 0;
    watch_started = tick::now_ms();
    for (uint8_t k = 0; k < 3; ++k) {
        check_due[k] = false;
        faults[k] = 0;
    }
    recover = Recover::kNone;
    stream = false;
    run_flag = false;
    feed = 100.0f;
    sync_job = Job::kNone;
    power_reported = power_pending = false;
    limit_reported = limit_pending = limit_seen = false;
    limit_since = last_sample = last_power = tick::now_ms();
    async_reads = Reads{};
    sync_reads = Reads{};
    bus_owner = nullptr;
    // The position at boot, from the encoders (faithful uarm_swift.c:21). Done before the main loop, so it may block.
    // If any joint fails, the position stays unknown and moves are refused with E26 until M17 succeeds (SP-15).
    float angles[3];
    bool ok = true;
    for (uint8_t i = 0; i < 3; ++i) {
        encoder::run(kLRB[i], encoder::Job::kAngle);
        ok = ok && encoder::ok();
        angles[i] = encoder::angle(kLRB[i], encoder::raw());
    }
    if (ok) motion::sync(7, angles);
    else motion::hold_all();  // R-9 revised (2026-10-18): hold the arm rather than let it drop; M17 re-reads
}

Kind kind(const parse::Command& c) {
    if (c.family == 'G') {
        switch (c.number) {
            case 0: case 1: case 2201: case 2204: case 2205: case 2206: return Kind::kMotion;
            case 2202: return c.params[0] == 'N' && c.params[1] == '3' ? Kind::kNormal : Kind::kMotion;  // N3: wrist
            default: break;
        }
    }
    return Kind::kNormal;
}

Status execute(const parse::Command& c, fmt::Writer& tail) {
    switch (c.family) {
        case 'P': return query(c.number, c.params, tail);
        case 'M': return mcode(c.number, c.params, tail);
        case 'G': return gcode(c.number, c.params);
        default: return scode(c.number, c.params);
    }
}

Status resume_async(fmt::Writer& tail) {
    if (!async_reads.done) return Status::kPending;
    const Status s = finish_query(tail);
    async_reads.active = false;
    return s;
}

Status resume_sync(fmt::Writer&) {
    switch (sync_job) {
        case Job::kResync: {
            if (!sync_reads.done) return Status::kPending;
            sync_reads.active = false;
            sync_job = Job::kNone;
            float angles[3] = {};
            for (uint8_t k = 0; k < sync_reads.count; ++k) {
                if (!sync_reads.ok[k]) return Status::kE26;  // SP-15: don't start from a bad reading
                const encoder::Joint j = sync_reads.joints[k];
                angles[static_cast<uint8_t>(j)] = encoder::angle(j, sync_reads.raw[k]);
            }
            motion::sync(resync_mask, angles);
            return Status::kOk;
        }
        case Job::kLine: {
            const motion::Result r = motion::line_result();
            if (r == motion::Result::kChecking) return Status::kPending;
            sync_job = Job::kNone;
            return result_status(r);
        }
        case Job::kDelay:
            if (tick::now_ms() - delay_start < delay_ms) return Status::kPending;
            sync_job = Job::kNone;
            return Status::kOk;
        case Job::kSearch:  // M2400 S4/S5, M2412: the search, then the move to 90 deg
            if (tool::origin() == tool::Origin::kSearching || tool::busy()) return Status::kPending;
            sync_job = Job::kNone;
            return tool::origin() == tool::Origin::kFound ? Status::kOk : Status::kE26;  // Q-66
        case Job::kReference:
        case Job::kHeight: {
            if (!sync_reads.done) return Status::kPending;
            sync_reads.active = false;
            const Job job = sync_job;
            sync_job = Job::kNone;
            for (uint8_t k = 0; k < 3; ++k)
                if (!sync_reads.ok[k]) return Status::kE26;  // SP-15; nothing is written
            float angles[3];  // L, R, B (user frame), from the readings, by motion axis (kLRB order)
            if (job == Job::kHeight) {
                for (uint8_t k = 0; k < 3; ++k) angles[k] = encoder::angle(kLRB[k], sync_reads.raw[k]);
                float x, y, z;
                kin::forward(angles[0], angles[1], angles[2] - 90.0f, &x, &y, &z);  // uarm_protocol.c:1043-1049
                offsets.height = z;
                motion::set_offsets(offsets);
                save_sys();
                save_user();
                return Status::kOk;
            }
            // M2401: the readings become the reference, struct order b, l, r (uarm_angle.c:7-11, 273-279).
            uint8_t buf[6];
            const uint16_t refs[3] = {sync_reads.raw[2], sync_reads.raw[0], sync_reads.raw[1]};
            for (uint8_t k = 0; k < 3; ++k) {
                buf[2 * k] = static_cast<uint8_t>(refs[k] & 0xFF);
                buf[2 * k + 1] = static_cast<uint8_t>(refs[k] >> 8);
            }
            nvm::write(nvm::kAngleRefer, buf, sizeof buf);
            nvm::write_byte(nvm::kAngleReferFlag, 0xBB);
            uint8_t mask = 0;
            for (uint8_t k = 0; k < 3; ++k) {
                encoder::set_reference(kLRB[k], sync_reads.raw[k]);
                angles[k] = encoder::angle(kLRB[k], sync_reads.raw[k]);
                if (motion::locked(k == 2 ? 0 : static_cast<uint8_t>(k + 1))) mask = static_cast<uint8_t>(mask | (1u << k));
            }
            motion::sync(mask, angles);  // SP-59, Q-65: the tally follows the new reference
            return Status::kOk;
        }
        case Job::kNone: break;
    }
    return Status::kOk;
}

// Samples the limit switch and motor power once per millisecond.
void poll_events() {
    const uint32_t now = tick::now_ms();
    if (now == last_sample) return;
    last_sample = now;
    const bool limit = effector::limit_pressed();
    if (limit != limit_seen) {
        limit_seen = limit;
        limit_since = now;
    }
    if (limit_seen != limit_reported && now - limit_since >= 10) {
        limit_reported = limit_seen;
        limit_pending = true;
    }
    if (now - last_power >= 10) {
        last_power = now;
        const bool power = effector::power_present();
        if (power != power_reported) {
            power_reported = power;
            power_pending = true;
        }
    }
}

namespace {

// After a lost-step stop: once motion has stopped, re-read the locked joints and re-sync their tally (Q-6).
void poll_recover() {
    if (recover == Recover::kWait && motion::idle()) {
        encoder::Joint joints[3];
        uint8_t n = 0, mask = 0;
        for (uint8_t a = 0; a < 3; ++a) {
            if (!motion::locked(a == 2 ? 0 : static_cast<uint8_t>(a + 1))) continue;
            joints[n++] = kLRB[a];
            mask = static_cast<uint8_t>(mask | (1u << a));
        }
        if (n == 0) {
            recover = Recover::kNone;
            return;
        }
        begin_reads(sync_reads, 0, encoder::Job::kAngle, n, joints);
        resync_mask = mask;
        recover = Recover::kRead;
        return;
    }
    if (recover != Recover::kRead) return;
    if (!sync_reads.active) {  // abandoned by a stop: start again
        recover = Recover::kWait;
        return;
    }
    if (!sync_reads.done) return;
    sync_reads.active = false;
    recover = Recover::kNone;
    float angles[3] = {};
    uint8_t mask = 0;
    for (uint8_t k = 0; k < sync_reads.count; ++k) {
        if (!sync_reads.ok[k]) continue;  // that joint keeps its tally (SP-15)
        const encoder::Joint j = sync_reads.joints[k];
        angles[static_cast<uint8_t>(j)] = encoder::angle(j, sync_reads.raw[k]);
        mask = static_cast<uint8_t>(mask | (1u << static_cast<uint8_t>(j)));
    }
    motion::sync(mask, angles);
}

}  // namespace

bool poll() {
    if (stepper::running())  // the arm is moving
        for (bool& c : check_due) c = true;
    const bool reads = poll_reads();
    poll_recover();
    effector::poll();
    tool::poll();
    buzzer::poll();
    report::poll_buttons();
    poll_events();
    return motion::poll() || reads;
}

bool take_fault() {
    if (!lost) return false;
    lost = false;
    recover = Recover::kWait;
    return true;
}

bool measured(uint8_t axis, float* deg, uint32_t* at) {
    const Measured& m = measured_now[axis];
    *deg = m.deg;
    *at = m.at;
    return m.ok;
}

void stop() {
    motion::stop();
    tool::stop();
    // A sync job is abandoned (its command is answered E25 by the protocol). A read on the bus finishes there.
    sync_job = Job::kNone;
    sync_reads.active = false;
}

// A move's reply also waits for its final lost-step check (SP-41), and for the re-sync after a lost-step stop.
bool checked() { return !check_due[0] && !check_due[1] && !check_due[2] && recover == Recover::kNone && !lost; }

bool idle() { return motion::idle() && buzzer::idle() && !tool::busy() && checked(); }

bool can_start(Kind k) {
    if (!buzzer::idle() || tool::busy()) return false;  // as the original: not while the buzzer or effector runs
    if (recover != Recover::kNone || lost) return false;
    return k == Kind::kMotion ? !motion::busy() : motion::idle() && checked();
}

bool next_event(fmt::Writer& out) {
    if (lost_report) {  // @8 N<joint as G2202> V<tally - encoder, deg> (SP-41, Q-26, Q-45)
        lost_report = false;
        out.put(FLASH("@8 N"));
        out.u16(lost_axis == 2 ? 0 : static_cast<uint8_t>(lost_axis + 1));
        out.put(FLASH(" V"));
        out.f4(lost_error);
        return true;
    }
    if (run_flag && motion_done_report && idle()) {  // @9 (uarm_protocol.c:216-221)
        run_flag = false;
        out.put(FLASH("@9 V0"));
        return true;
    }
    if (limit_pending) {  // @6 (uarm_protocol.c:192-201)
        limit_pending = false;
        out.put(limit_reported ? FLASH("@6 N0 V1") : FLASH("@6 N0 V0"));
        return true;
    }
    if (power_pending) {  // @5 (uarm_protocol.c:203-214)
        power_pending = false;
        out.put(power_reported ? FLASH("@5 V1") : FLASH("@5 V0"));
        return true;
    }
    return report::button_event(out);  // @4
}

bool streaming() { return stream; }

float effector_angle() { return tool::active() ? tool::angle() : effector::wrist(); }

uint8_t work_mode() { return mode; }

}  // namespace commands
