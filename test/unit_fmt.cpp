// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jack Sidman Smith

#include "../src/fmt.h"
#include "check.h"

#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <string>

namespace {

std::string f4(float v) {
    char buf[64];
    fmt::Writer w(buf, sizeof buf);
    w.f4(v);
    return w.text();
}

}  // namespace

void test_fmt() {
    // Every value avr-libc printed under simavr (test/data/dtostrf.txt).
    std::ifstream in("test/data/dtostrf.txt");
    CHECK(static_cast<bool>(in));
    std::string line;
    int rows = 0, bad = 0;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') continue;
        const size_t sp = line.find(' ');
        const uint32_t bits = static_cast<uint32_t>(std::strtoul(line.substr(0, sp).c_str(), nullptr, 16));
        const std::string want = line.substr(sp + 1);
        float v;
        std::memcpy(&v, &bits, sizeof v);
        const std::string got = f4(v);
        ++rows;
        if (got != want) {
            if (++bad <= 10) std::printf("dtostrf %08x: got \"%s\", want \"%s\"\n", bits, got.c_str(), want.c_str());
        }
    }
    CHECK(rows == 5139);
    CHECK(bad == 0);

    // Integers and text.
    char buf[16];
    fmt::Writer w(buf, sizeof buf);
    w.put("V");
    w.u16(0);
    w.put(' ');
    w.u16(65535);
    w.put(' ');
    w.i16(-32768);
    CHECK_STR(w.text(), "V0 65535 -32768");
    CHECK(!w.overflow());

    // Running out of room is recorded, and the text stays NUL-terminated within the buffer.
    char small[6];
    fmt::Writer s(small, sizeof small);
    s.put("12345");
    CHECK(!s.overflow());
    s.put('6');
    CHECK(s.overflow());
    CHECK_STR(s.text(), "12345");

}
