// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jack Sidman Smith
//
// Prints dtostrf(v, 5, 4) for every value in table.h on USART0, one per line, then "END". Runs under simavr
// (run_avr -m atmega2560 -f 16000000). Host-side reference only; never flashed.

#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <stdlib.h>
#include <string.h>
#include <util/delay.h>

#include "table.h"  // static const uint32_t kVals[] PROGMEM = { float32 bit patterns };

static void put(char c) {
    while (!(UCSR0A & _BV(UDRE0))) {}
    UDR0 = c;
}

static void put_s(const char* s) {
    while (*s) put(*s++);
}

int main(void) {
    UCSR0B = _BV(TXEN0);
    char b[32];
    for (unsigned i = 0; i < sizeof kVals / sizeof kVals[0]; ++i) {
        uint32_t u = pgm_read_dword(&kVals[i]);
        float f;
        memcpy(&f, &u, 4);
        dtostrf(f, 5, 4, b);
        put_s(b);
        put('\n');
    }
    put_s("END\n");
    _delay_ms(5);  // let the UART finish before the simulation stops
    cli();
    sleep_cpu();
}
