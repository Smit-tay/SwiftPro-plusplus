// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jack Sidman Smith
//
// Lines in, replies out (faithful uarm_protocol.c:23-185, with the approved changes).
//
//   P/S lines (async) run at once and reply at once.
//   G/M lines (sync) wait in a 20-entry FIFO. One starts per main-loop pass when commands::can_start() allows it.
//   Their replies go out in FIFO order, only while commands::idle(); with M2124 V1, a motion command's reply goes
//   out as soon as it was accepted (Q-32).
//
// Replies: "$N " (when the line had "#N") + code + tail + "\n". Codes: ok, E20, E21, E22, E23, E25, E26.
// The queue holds 20 commands that are waiting to run or waiting to reply (SP-53); the next one gets "$N E23"
// (SP-13). A line is taken from the RX ring only when its reply is sure to fit the TX ring.

#pragma once

#include <stdint.h>

namespace protocol {

void init();

// One step of work. Returns true if anything happened (the simulator runs it until it returns false).
bool poll();

// No sync command waiting to run or to reply.
bool queue_empty();

}  // namespace protocol
