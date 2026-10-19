// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jack Sidman Smith

#include "app.h"

#include "board.h"
#include "commands.h"
#include "fmt.h"
#include "protocol.h"
#include "serial.h"
#include "tick.h"

#if defined(__AVR__)
#include <avr/interrupt.h>
#endif

namespace app {

void init() {
    board::init();  // straps first, as the original (uarm_swift.c:11)
    serial::init();
    tick::init();
    commands::init();
    protocol::init();
#if defined(__AVR__)
    sei();  // the original enabled interrupts by accident, inside an EEPROM write (SP-33)
#endif

    // Banner (faithful uarm_swift.c:29-34; the debug "angle offset:" line before it is removed, SP-14).
    // The TX ring is empty here and the banner is about 80 bytes, so it fits. It goes to USB, as the original's
    // replies did before any byte arrived.
    char buf[96];
    fmt::Writer banner(buf, sizeof buf);
    banner.put(FLASH("device name : SwiftPro\r\nsoft ver : V5.0.0-pp.1\r\napi ver : V4.0.5\r\nwork mode : "));  // Q-2
    banner.u16(commands::work_mode());
    banner.put(FLASH("\r\n@1\n"));
    serial::write(serial::kUsb, banner.text(), banner.length());
}

bool poll() { return protocol::poll(); }

}  // namespace app
