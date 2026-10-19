// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jack Sidman Smith
//
// Float formatting. The original prints every float with avr-libc's dtostrf(v, 5, 4). On the AVR the port calls
// the same function, so the bytes match by construction: avr-libc 2.0.0 (the original's toolchain) and 2.2.1 print
// identical text for all 5,125 values in test/data/dtostrf.txt (run under simavr, 2026-10-18).
//
// The host build (tests, simulator) implements the rule those 5,125 values follow, and is checked against all of
// them: the exact binary value, rounded half away from zero to 4 decimals, but to no more than 8 significant digits
// (then zero-padded); the sign is kept when the result is zero ("-0.0000"); NaN is "  NAN" and infinity "  INF" or
// " -INF" (right-aligned to width 5).

#include "fmt.h"

#if defined(__AVR__)
#include <stdlib.h>
#else
#include <cmath>
#include <cstdio>
#include <cstring>
#endif

namespace fmt {

Writer::Writer(char* buf, uint8_t capacity) : buf_(buf), cap_(capacity) {
    if (cap_ > 0) buf_[0] = '\0';
}

void Writer::put(char c) {
    if (len_ + 1 >= cap_) {
        overflow_ = true;
        return;
    }
    buf_[len_++] = c;
    buf_[len_] = '\0';
}

void Writer::put(const char* s) {
    while (*s != '\0') put(*s++);
}

void Writer::put(const FlashText* s) {
    const char* p = reinterpret_cast<const char*>(s);
#if defined(__AVR__)
    for (char ch = static_cast<char>(pgm_read_byte(p)); ch != '\0'; ch = static_cast<char>(pgm_read_byte(++p))) put(ch);
#else
    put(p);
#endif
}

void Writer::u16(uint16_t v) { u32(v); }

void Writer::u32(uint32_t v) {
    char digits[10];
    uint8_t n = 0;
    do {
        digits[n++] = static_cast<char>('0' + v % 10);
        v /= 10;
    } while (v != 0);
    while (n > 0) put(digits[--n]);
}

void Writer::i16(int16_t v) {
    if (v < 0) {
        put('-');
        u16(static_cast<uint16_t>(-static_cast<int32_t>(v)));
    } else {
        u16(static_cast<uint16_t>(v));
    }
}

#if defined(__AVR__)

void Writer::f4(float v) {
    char tmp[48];  // the longest output, -FLT_MAX, is 45 characters (test/data/dtostrf.txt)
    dtostrf(static_cast<double>(v), 5, 4, tmp);
    put(tmp);
}

#else

void Writer::f4(float v) {
    if (std::isnan(v)) {
        put("  NAN");
        return;
    }
    if (std::isinf(v)) {
        put(v > 0.0f ? "  INF" : " -INF");
        return;
    }
    // Decimal expansion of the float. 60 decimals is exact for every float >= 2^-60; smaller ones are far below the
    // rounding position, so only their sign matters, and that is handled separately.
    char exact[112];  // up to 39 integer digits, the point and 60 decimals
    std::snprintf(exact, sizeof exact, "%.60f", std::fabs(static_cast<double>(v)));
    const char* point = std::strchr(exact, '.');
    // Digits without the point; ipart digits come first.
    char digits[112];
    uint8_t nd = 0;
    for (const char* c = exact; *c != '\0'; ++c)
        if (*c != '.') digits[nd++] = *c;
    const int ipart = static_cast<int>(point - exact);
    // Position of the first significant digit (index into digits[]), or nd if the value is zero.
    int first = 0;
    while (first < nd && digits[first] == '0') ++first;
    // Keep 4 decimals, but no more than 8 significant digits.
    int keep = ipart + 4;
    if (first < nd && keep - first > 8) keep = first + 8;
    // Round half away from zero on the magnitude: look at the first dropped digit.
    char out[112];
    int n = keep;
    for (int i = 0; i < n; ++i) out[i] = digits[i];
    bool carry = keep < nd && digits[keep] >= '5';
    for (int i = n - 1; carry && i >= 0; --i) {
        if (out[i] == '9') {
            out[i] = '0';
        } else {
            ++out[i];
            carry = false;
        }
    }
    const int ip = ipart;
    for (int i = n; i < ip + 4; ++i) out[i] = '0';  // digits past the 8th significant one print as '0'
    char res[112];
    int r = 0;
    if (std::signbit(v)) res[r++] = '-';
    if (carry) {  // e.g. 99.99995 -> 100.0000
        res[r++] = '1';
    }
    // Integer part without leading zeros (but at least one digit).
    int start = 0;
    while (start < ip - 1 && out[start] == '0') ++start;
    for (int i = start; i < ip; ++i) res[r++] = out[i];
    res[r++] = '.';
    for (int i = ip; i < ip + 4; ++i) res[r++] = out[i];
    res[r] = '\0';
    put(res);
}

#endif

}  // namespace fmt
