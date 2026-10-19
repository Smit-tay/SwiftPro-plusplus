// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jack Sidman Smith

#include "line.h"

namespace line {

Result Assembler::feed(uint8_t c) {
    if (done_) {
        len_ = 0;
        buf_[0] = '\0';
        comment_ = Comment::kNone;
        too_long_ = false;
        lost_ = false;
        done_ = false;
    }

    if (c == kLostMarker) {  // SP-01: bytes of this line were dropped
        lost_ = true;
        return Result::kNone;
    }

    if (c == '\n' || c == '\r') {  // faithful protocol.c:81-87
        buf_[len_] = '\0';
        done_ = true;
        if (lost_) return Result::kLost;
        if (too_long_) return Result::kTooLong;
        return Result::kLine;
    }

    if (comment_ != Comment::kNone) {  // faithful protocol.c:88-93
        if (c == ')' && comment_ == Comment::kParenthesis) comment_ = Comment::kNone;
        return Result::kNone;
    }

    if (c <= ' ') return Result::kNone;  // faithful protocol.c:95-96: whitespace and control bytes
    if (c == '/') return Result::kNone;  // faithful protocol.c:97-99: block delete not supported
    if (c == '(') {                      // faithful protocol.c:100-105
        comment_ = Comment::kParenthesis;
        return Result::kNone;
    }
    if (c == ';') {  // faithful protocol.c:106-108
        comment_ = Comment::kSemicolon;
        return Result::kNone;
    }

    if (len_ >= kMaxLength) {  // Q-22 (replaces the 79-character overflow of protocol.c:118-122)
        too_long_ = true;
        return Result::kNone;
    }
    if (c >= 'a' && c <= 'z') c = static_cast<uint8_t>(c - 'a' + 'A');  // faithful protocol.c:123-124
    buf_[len_++] = static_cast<char>(c);
    return Result::kNone;
}

}  // namespace line
