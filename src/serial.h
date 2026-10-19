// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jack Sidman Smith
//
// USART0-3 at 115200 8N1 (kept for Bluetooth, Q-16), each with its own receive ring (R-6), one shared transmit ring.
//
//  - RX overflow is reported, not silent (SP-01, Q-3): the damaged line is replaced by line::kLostMarker, which the
//    line assembler turns into "E23".
//  - TX never blocks (SP-02, Q-21): write() is all-or-nothing and the caller checks tx_room() first. The ring serves one
//    port at a time: a write for another port waits (tx_room() is 0 for it) until the ring has drained, so lines are
//    never split across ports.
//  - Realtime bytes (Q-15): '?' '!' '~' '@' are dropped with no action; 0x18 (Ctrl-X) on any port is a stop request.
//  - RX pins have pull-ups (Q-16), so an unconnected port can't pick up noise.
// The original shared one RX ring between all four ports and sent replies to whichever port received the last byte,
// so two active ports could splice each other's lines (SP-31).

#pragma once

#include <stdint.h>

namespace serial {

constexpr uint8_t kPorts = 4;
constexpr uint8_t kUsb = 0;            // USART0, the USB-serial port
constexpr uint16_t kRxSizeUsb = 256;   // the original: 128 for all four (serial.h:27)
constexpr uint16_t kRxSizeOther = 64;  // USART1-3: one 48-character line at a time is plenty
constexpr uint16_t kTxSize = 256;      // the original: 128 (serial.h:30)

// Empties all rings and clears all state; on the AVR also sets up the four USARTs. Call before sei().
void init();

// Main loop side.
bool read(uint8_t port, uint8_t* c);              // false if that port's ring is empty
uint16_t tx_room(uint8_t port);                   // bytes write() can take now for this port
bool write(uint8_t port, const char* s, uint16_t n);  // all or nothing; false (nothing written) if it doesn't fit
bool write(uint8_t port, const char* s);          // NUL-terminated
bool take_stop_request();                         // true once per Ctrl-X received
uint8_t last_port();                              // the port that received the last byte (for unsolicited events)

// ISR side. Called by the USART ISRs on the AVR, and directly by host tests and the simulator.
void on_rx(uint8_t port, uint8_t c);
bool tx_pop(uint8_t* c);  // next byte to transmit; false if the TX ring is empty

}  // namespace serial
