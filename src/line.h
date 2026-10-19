// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jack Sidman Smith
//
// Line assembly: bytes from the RX ring in, complete command lines out.
//
// Faithful to the original (protocol.c:80-129): a line ends at '\n' or '\r'; every byte <= ' ' is dropped, so all
// spaces vanish; '(' ... ')' and ';' ... end of line are comments; '/' is ignored; a-z become A-Z.
// Changes: a line longer than 48 characters is refused (Q-22); a line that lost bytes in the RX ring is refused
// (SP-01, Q-3). The original reported overflow at 79 characters and ran the rest of the line as a new line (SP-13).

#pragma once

#include <stdint.h>

namespace line {

constexpr uint8_t kMaxLength = 48;  // Q-22: libswiftpro's longest command with "#seq" is about 42

// The RX ISR puts this byte in the ring after it had to drop bytes. It can't come from the wire as a command
// byte: the original drops every byte <= ' ' anyway.
constexpr uint8_t kLostMarker = 0x01;

enum class Result : uint8_t {
    kNone,     // no complete line yet
    kLine,     // text() is a complete line (may be empty)
    kTooLong,  // the line had more than kMaxLength characters; text() holds the first kMaxLength
    kLost,     // bytes of this line were lost in the RX ring; text() holds what arrived
};

class Assembler {
public:
    Result feed(uint8_t c);

    // Valid until the next feed().
    const char* text() const { return buf_; }
    uint8_t length() const { return len_; }

private:
    enum class Comment : uint8_t { kNone, kParenthesis, kSemicolon };

    char buf_[kMaxLength + 1] = {};
    uint8_t len_ = 0;
    Comment comment_ = Comment::kNone;
    bool too_long_ = false;
    bool lost_ = false;
    bool done_ = false;  // the previous feed() completed a line; start a new one
};

}  // namespace line
