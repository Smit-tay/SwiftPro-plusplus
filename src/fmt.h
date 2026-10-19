// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jack Sidman Smith
//
// Text output into a fixed buffer. Running out of room is recorded (overflow()), never silent: the caller checks it
// and reports an error instead of sending a truncated line.

#pragma once

#include <stdint.h>

#if defined(__AVR__)
#include <avr/pgmspace.h>
#endif

namespace fmt {

// Text kept in flash (R-8). Make one with FLASH("..."); on the host it is ordinary text.
struct FlashText;

#if defined(__AVR__)
#define FLASH(s) (reinterpret_cast<const ::fmt::FlashText*>(PSTR(s)))
#else
#define FLASH(s) (reinterpret_cast<const ::fmt::FlashText*>(s))
#endif

class Writer {
public:
    Writer(char* buf, uint8_t capacity);  // capacity includes the terminating NUL

    void put(char c);
    void put(const char* s);
    void put(const FlashText* s);
    void u16(uint16_t v);                      // decimal, as "%u"
    void u32(uint32_t v);                      // decimal, as "%lu"
    void i16(int16_t v);                       // decimal, as "%d"
    void f4(float v);                          // dtostrf(v, 5, 4), byte-exact (see fmt.cpp)

    const char* text() const { return buf_; }
    uint8_t length() const { return len_; }
    bool overflow() const { return overflow_; }

private:
    char* buf_;
    uint8_t cap_;
    uint8_t len_ = 0;
    bool overflow_ = false;
};

}  // namespace fmt
