// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jack Sidman Smith
//
// Start-up and the main loop. The AVR main() and the host simulator both drive the firmware through these two calls.

#pragma once

namespace app {

// Initialises every module and prints the boot banner. On the AVR, call with interrupts disabled; it enables them.
void init();

// One pass of the main loop. Returns true if anything happened.
bool poll();

}  // namespace app
