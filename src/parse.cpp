// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jack Sidman Smith

#include "parse.h"

namespace parse {

namespace {

constexpr int32_t kIntMin = -32768;   // the AVR's int, which sscanf "%d" fills in the original
constexpr int32_t kIntMax = 32767;

bool is_number_char(char c) { return (c >= '0' && c <= '9') || c == '-' || c == '+' || c == '.'; }

}  // namespace

// Number reading (R-2): an optional sign, digits with at most one '.', no exponent; at least one digit. The result is
// the correctly rounded float whenever the digits form an integer up to 2^24 (about 7 significant digits) and there are
// at most 10 decimals, which covers everything a host sends: the value is one float division by an exact power of ten.
// Longer inputs keep 9 significant digits and are divided in two steps, so the last bit may be off.
// (The original kept 8 digits and scaled by 0.01 and 0.1 steps, which rounds several times.)
// Q-53: a number must end at a character that can't continue it; a second '.' or a stray sign is a format error.
bool read_float(const char* s, const char** end, float* value) {
    constexpr float kPow10[11] = {1e0f, 1e1f, 1e2f, 1e3f, 1e4f, 1e5f, 1e6f, 1e7f, 1e8f, 1e9f, 1e10f};  // all exact
    const char* p = s;
    const bool negative = *p == '-';
    if (*p == '-' || *p == '+') ++p;

    uint32_t digits = 0;  // significant digits kept, as an integer
    uint8_t kept = 0;
    int8_t decimals = 0;  // > 0: divide by 10^decimals; < 0: multiply by 10^-decimals (dropped integer digits)
    bool point = false, any = false;
    for (;; ++p) {
        const char ch = *p;
        if (ch >= '0' && ch <= '9') {
            any = true;
            if (digits == 0 && ch == '0') {  // leading zeros carry no precision
                if (point && decimals < 100) ++decimals;
            } else if (kept < 9) {
                digits = digits * 10u + static_cast<uint8_t>(ch - '0');
                ++kept;
                if (point) ++decimals;
            } else if (!point) {
                --decimals;  // an integer digit beyond the 9 kept ones
            }
        } else if (ch == '.' && !point) {
            point = true;
        } else {
            break;
        }
    }
    if (!any) return false;
    if (*p == '.' || *p == '+' || *p == '-') return false;  // Q-53: "1.2.3", "1-2"

    float v = static_cast<float>(digits);
    if (decimals > 0) {
        if (decimals <= 10) {
            v /= kPow10[decimals];
        } else {
            v /= kPow10[10];
            for (int8_t d = static_cast<int8_t>(decimals - 10); d > 0; --d) v /= 10.0f;
        }
    } else {
        for (int8_t d = decimals; d < 0; ++d) v *= 10.0f;
    }
    *value = negative ? -v : v;
    if (end != nullptr) *end = p;
    return true;
}

Routed route(const char* line) {
    Routed r{Route::kIgnore, {}, line};

    if (line[0] == '$') {  // uarm_protocol.c:29-31; the Grbl system commands are dropped (Q-36, Q-42)
        r.route = Route::kSystem;
        return r;
    }

    const char* p = line;
    if (line[0] == '#') {  // uarm_protocol.c:32-34: sscanf "#%[0-9]", then skip '#' and the digits
        ++p;
        uint8_t n = 0;
        while (*p >= '0' && *p <= '9') {
            if (n == kMaxSeqDigits) {  // SP-03: temp_str[10] would overflow
                r.route = Route::kBadSequence;
                return r;
            }
            r.seq[n++] = *p++;
        }
        r.seq[n] = '\0';
        r.body = p;
        switch (*p) {
            case 'P': case 'S': r.route = Route::kAsync; break;
            case 'G': case 'M': r.route = Route::kSync; break;
            default: r.route = Route::kNotCommand; break;  // SP-52: the original queued it for Grbl
        }
        return r;
    }

    switch (line[0]) {  // uarm_protocol.c:35-41
        case 'P': case 'S': r.route = Route::kAsync; break;
        case 'G': case 'M': r.route = Route::kSync; break;
        default: r.route = Route::kIgnore; break;
    }
    return r;
}

Word command(const char* body, Command* out) {
    out->family = body[0];
    const char* end = nullptr;
    float value = 0.0f;
    if (!read_float(body + 1, &end, &value)) return Word::kBadNumber;
    out->params = end;
    if (value < 0.0f || value >= 65536.0f) return Word::kOutOfRange;  // SP-12 addendum: the original wrapped
    const uint16_t number = static_cast<uint16_t>(value);
    if (static_cast<float>(number) != value) return Word::kNotInteger;  // Q-53
    out->number = number;
    return Word::kOk;
}

bool expect(const char** p, char letter) {
    if (**p != letter) return false;
    ++*p;
    return true;
}

bool number(const char** p, uint8_t max, float* value) {
    const char* start = *p;
    const char* q = start;
    while (is_number_char(*q)) ++q;
    const auto run = q - start;
    if (run == 0 || run > max) return false;  // sscanf no match / SP-03
    // read_float on the copied run: it stops at the first character it can't use, which is inside the run or at its
    // end. Reading the line in place gives the same result, because the run ends at a non-number character.
    if (!read_float(start, nullptr, value)) return false;  // SP-10
    *p = q;
    return true;
}

bool integer(const char** p, int16_t* value) {
    const char* q = *p;
    bool negative = false;
    if (*q == '-' || *q == '+') negative = *q++ == '-';
    int32_t v = 0;
    bool digits = false;
    while (*q >= '0' && *q <= '9') {
        digits = true;
        v = v * 10 + (*q++ - '0');
        if (v > -kIntMin) return false;  // SP-03; leading zeros are fine, as with sscanf
    }
    if (!digits) return false;
    if (negative) v = -v;
    if (v < kIntMin || v > kIntMax) return false;
    *value = static_cast<int16_t>(v);
    *p = q;
    return true;
}

}  // namespace parse
