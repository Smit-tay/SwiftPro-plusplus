// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jack Sidman Smith

#include "../src/line.h"
#include "check.h"

#include <cstdint>
#include <string>

namespace {

// Feeds s and returns the result of the last byte.
line::Result feed(line::Assembler& a, const std::string& s) {
    line::Result r = line::Result::kNone;
    for (const char c : s) r = a.feed(static_cast<uint8_t>(c));
    return r;
}

}  // namespace

void test_line() {
    line::Assembler a;

    // Spaces and control bytes vanish, case folds (faithful protocol.c:95-96, 123-124).
    CHECK(feed(a, "#1 p 2201\n") == line::Result::kLine);
    CHECK_STR(a.text(), "#1P2201");
    CHECK(a.length() == 7);

    // CR ends a line too; CRLF gives a line and then an empty line.
    CHECK(feed(a, "P2201\r") == line::Result::kLine);
    CHECK_STR(a.text(), "P2201");
    CHECK(feed(a, "\n") == line::Result::kLine);
    CHECK_STR(a.text(), "");

    // Comments: (...) inline, ; to end of line (where ')' doesn't end it); '/' is ignored.
    CHECK(feed(a, "P22(comment)01\n") == line::Result::kLine);
    CHECK_STR(a.text(), "P2201");
    CHECK(feed(a, "P2201;(x)abc\n") == line::Result::kLine);
    CHECK_STR(a.text(), "P2201");
    CHECK(feed(a, "P2/201\n") == line::Result::kLine);
    CHECK_STR(a.text(), "P2201");
    // A comment ends with the line.
    CHECK(feed(a, "(open\n") == line::Result::kLine);
    CHECK(feed(a, "P2201\n") == line::Result::kLine);
    CHECK_STR(a.text(), "P2201");

    // Bytes above 0x7F are kept (the original reads uint8_t, protocol.c:65).
    CHECK(feed(a, "P\xC3\n") == line::Result::kLine);
    CHECK(static_cast<uint8_t>(a.text()[1]) == 0xC3);

    // 48 characters: accepted. 49: too long, and the first 48 are kept for "#N" (Q-22).
    const std::string ok48 = "#8P2206N1" + std::string(39, '0');
    CHECK(ok48.size() == 48);
    CHECK(feed(a, ok48 + "\n") == line::Result::kLine);
    CHECK(a.length() == 48);
    CHECK(feed(a, ok48 + "0\n") == line::Result::kTooLong);
    CHECK(a.length() == 48);
    CHECK(std::string(a.text()) == ok48);
    // Spaces don't count.
    CHECK(feed(a, "#8 P2206 N1 " + std::string(39, '0') + "\n") == line::Result::kLine);

    // A line with the lost-bytes marker is refused, and the next line is fine (SP-01).
    std::string lost = "#5P22";
    lost += static_cast<char>(line::kLostMarker);
    CHECK(feed(a, lost + "\n") == line::Result::kLost);
    CHECK(feed(a, "#6P2201\n") == line::Result::kLine);
    CHECK_STR(a.text(), "#6P2201");

    // Nothing is reported before the end of a line.
    CHECK(feed(a, "P2201") == line::Result::kNone);
    CHECK(feed(a, "\n") == line::Result::kLine);
}
