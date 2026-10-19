// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jack Sidman Smith
//
// Parsing of assembled lines: routing ("#N", family), the command word, and the bounded field scanners that replace
// the original's sscanf patterns. Pure functions, host-tested.

#pragma once

#include <stdint.h>

namespace parse {

// Reads a number: optional sign, digits, one '.', no exponent; correctly rounded for host-sized numbers (parse.cpp).
// On success sets *end (if not nullptr) to the first character not used and returns true. False if there is no digit,
// or the number is followed by '.', '+' or '-' (Q-53).
bool read_float(const char* s, const char** end, float* value);

enum class Route : uint8_t {
    kIgnore,       // no reply: empty line, or a line that doesn't start with $ # G M P S (uarm_protocol.c:39-41)
    kSystem,       // "$..." Grbl system command: dropped, reply E20 (Q-36, Q-42)
    kAsync,        // P or S: executes now, replies now
    kSync,         // G or M: sync queue
    kNotCommand,   // after "#N", something other than G M P S: reply E20 (SP-52)
    kBadSequence,  // "#" followed by more than 9 digits: reply E21 without $N (SP-03; the original overflowed)
};

constexpr uint8_t kMaxSeqDigits = 9;  // the original's temp_str[10] (uarm_protocol.c:28)

struct Routed {
    Route route;
    char seq[kMaxSeqDigits + 1];  // digits of "#N", "" if none
    const char* body;             // the line after "#N"
};

Routed route(const char* line);

enum class Word : uint8_t {
    kOk,
    kBadNumber,   // no number after the letter: Grbl STATUS_BAD_NUMBER_FORMAT (gcode.c:120), reply E21 (SP-11)
    kNotInteger,  // a fraction (G2202.5): reply E21 (Q-53; the original ran G2202.5 as G2202)
    kOutOfRange,  // number < 0 or > 65535: the original wrapped it (SP-12 addendum), reply E20
};

struct Command {
    char family;          // 'G', 'M', 'P' or 'S'
    uint16_t number;      // e.g. 2202
    const char* params;   // text after the number
};

// Reads the command word at body[0] (a letter known to be G/M/P/S): the letter and a whole number 0-65535.
Word command(const char* body, Command* out);

// Field scanners, replacing sscanf. Each advances *p past what it read and returns false on a mismatch.

// A literal letter, as in the "N" of "N%d".
bool expect(const char** p, char letter);

// "%[0-9-+.]" then a number read from that copy, as the original does (uarm_protocol.c, e.g. :278-283). The run of
// [0-9+-.] must be 1..max characters, where max is the original's buffer size minus one (SP-03: longer runs
// overflowed it). The number is read from the start of the run; the rest of the run is ignored, as in the original
// ("V1.2.3" gives 1.2).
// Returns false if the run is empty or too long (sscanf no match / SP-03) or has no digit (SP-10).
bool number(const char** p, uint8_t max, float* value);

// "%d" for small integers: optional sign and at least one digit, within int16_t (the AVR's int). Out of range is a
// mismatch (SP-03: the original's int overflow is undefined behaviour).
bool integer(const char** p, int16_t* value);

}  // namespace parse
