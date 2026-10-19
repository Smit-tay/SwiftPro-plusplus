// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jack Sidman Smith
//
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <stdint.h>
#include <string.h>
#include <util/delay.h>
struct Row { char kind; uint32_t v[3]; };
#include "table.h"
static void put(char c) { while (!(UCSR0A & _BV(UDRE0))) {} UDR0 = c; }
static void hex(float x) { uint32_t r; memcpy(&r, &x, 4); for (int s = 28; s >= 0; s -= 4) put("0123456789abcdef"[(r >> s) & 15]); put(' '); }
static float fl(uint32_t r) { float x; memcpy(&x, &r, 4); return x; }
static void finish(void) { put('E'); put('N'); put('D'); put('\n'); _delay_ms(5); cli(); sleep_cpu(); }
