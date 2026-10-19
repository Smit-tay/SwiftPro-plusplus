// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jack Sidman Smith
//
// Host simulator interface for the golden-trace runner.
//
// The host build of the firmware (Phase 2) implements these functions around the real protocol, motion and
// peripheral modules, with simulated pins, encoders and time. Until then, sim_stub.cpp implements them and
// sim_available() returns false, so every trace is reported as PENDING.
//
// Time is simulated: nothing happens between calls except what sim_settle() and sim_advance_ms() run.

#pragma once

#include <cstdint>
#include <string>
#include <string_view>

// False while only the stub is linked.
bool sim_available();

// Back to power-off state, with every simulated input at its default (see the keys below).
void sim_reset();

// Set a simulated input. Returns false for an unknown key or a bad value (the runner fails the trace).
// Keys and values, all text:
//   hw          V3.3.0 | V3.3.1 | V3.3.2 | V0.0.0       hardware straps on PJ3-PJ6        default V3.3.1
//   uid         <12 characters> | none (erased)        BLE UID in EEPROM 1002            default 000000000000
//   mode        0-8                                    work mode in EEPROM 900           default 0
//   enc.ref     B<n> L<n> R<n>                         reference counts (EEPROM 818/820) default B0 L0 R0
//   enc.raw     B<n> L<n> R<n>                         raw AS5600 counts now             default B0 L0 R0
//   enc.status  B<n> L<n> R<n>                         AS5600 STATUS register (0x0B)     default 32 each (MD only)
//   enc.nack    none | B | L | R | BLR (any subset)    chips that don't ACK              default none
//   enc.stall   none | B | L | R                       from now on this encoder stops following its motor
//   enc.slip    <B|L|R><counts>, e.g. R35 or L-12      that encoder jumps by n counts, then follows again
//   power       0 | 1                                  motor power present (ADC A5 > 100) default 1
//   limit       0 | 1                                  effector limit switch PG3 pressed  default 0
//   pump.adc    0-1023                                 ADC A9                             default 0
//   gripper.pa0 0 | 1                                  PA0 level                          default 1
//   button      menu|play down|up                      base button level                  default up
//   tool.switch 0 | 1                                  stepper effector origin switch PK5 default 0 (open)
//   pin.<P><b>  0 | 1, e.g. pin.A0 1                   external level on port P bit b     default 0
// While the motors move, encoders follow the step tally exactly (rounded to the nearest count),
// except a stalled encoder, which keeps its last value.
bool sim_set(std::string_view key, std::string_view value);

// Power on: runs reset, init and the boot banner. Output goes to the TX buffer.
void sim_boot();

// Bytes arriving on USART0, exactly as given. All of them go through the RX ISR before the main loop runs again, so
// more than 255 bytes at once overflow the RX ring, as a burst from the host would.
void sim_rx(std::string_view bytes);

// Run the main loop until it makes no more progress at the current simulated time.
void sim_settle();

// Advance simulated time by ms milliseconds, running the main loop and all ISRs as the hardware would.
void sim_advance_ms(uint32_t ms);

// True when no motion is planned or running and the sync queue is empty.
bool sim_idle();

// Everything transmitted on USART0 since the last call.
std::string sim_take_tx();
