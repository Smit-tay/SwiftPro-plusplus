// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jack Sidman Smith
//
// RAII interrupt lock: disables interrupts, restores the previous state (SREG) when it goes out of scope.
// Used for read-modify-write of registers that an ISR also writes (SP-28). No-op on the host.

#pragma once

#include <stdint.h>

#if defined(__AVR__)
#include <avr/interrupt.h>
#include <avr/io.h>

class InterruptLock {
public:
    InterruptLock() : sreg_(SREG) { cli(); }
    ~InterruptLock() { SREG = sreg_; }
    InterruptLock(const InterruptLock&) = delete;
    InterruptLock& operator=(const InterruptLock&) = delete;

private:
    uint8_t sreg_;
};

#else

class InterruptLock {
public:
    InterruptLock() {}   // user-provided, like the AVR version, so a lock variable doesn't count as unused
    ~InterruptLock() {}
    InterruptLock(const InterruptLock&) = delete;
    InterruptLock& operator=(const InterruptLock&) = delete;
};

#endif
