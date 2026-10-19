// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jack Sidman Smith
//
// Raw access to Arduino pins 0-69 for M2240 (write), M2241 (direction) and P2240 (read) (SP-32, Q-17).
//
// Pin numbers are the Arduino Mega 2560 numbering the original used through digitalWrite/pinMode/digitalRead: the
// table is framework-arduino-avr 5.2.0, variants/mega/pins_arduino.h (the framework of the original's platformio.ini,
// board megaatmega2560). As Arduino: writing 1 to an input pin turns its pull-up on; M2241 V0 makes the pin an input
// with the pull-up off.
// Changes: writes and direction changes to the pins the firmware drives itself are refused (Q-17): the arm's STEP, DIR,
// EN and micro-step pins, the encoder I2C pins, USART0-3, the wrist PWM and the buzzer, and in work modes 4/5 the
// effector stepper's pins and its origin switch. Reads are allowed on every pin and never touch a timer (SP-58: the
// original's digitalRead switched the PWM of pins 6 and 44 off).

#pragma once

#include <stdint.h>

namespace pins {

constexpr uint8_t kCount = 70;

bool owned(uint8_t pin, bool stepper_effector);  // the firmware drives this pin (refused for M2240/M2241)
bool write(uint8_t pin, bool high);              // false if pin >= kCount
bool set_output(uint8_t pin, bool output);       // false if pin >= kCount
bool read(uint8_t pin, bool* high);              // false if pin >= kCount

#if !defined(__AVR__)
// Host only: the simulated ports A-L (index 0 = A), as PORT, DDR and the external input level.
struct HostPort {
    uint8_t port, ddr, in;
};
HostPort& host_port(char letter);
#endif

}  // namespace pins
