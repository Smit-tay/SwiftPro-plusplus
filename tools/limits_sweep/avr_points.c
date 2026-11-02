// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jack Sidman Smith
/* The original's legality for a list of poses, on the ATmega2560 under simavr (avr_check.sh), to confirm the host
 * sweep where float details matter. Built with the original's toolchain and a copy of its uarm_coord_convert.c.
 * Prints per point: <L*2> <R*2> <direct 0/1> <round trip 0/1>  (half degrees as integers: no float printing). */
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdbool.h>
#include "points.h"
struct uarm_t { float init_arml_angle, init_armr_angle, init_base_angle; struct { float front_offset, high_offset; } param; };
struct uarm_t uarm;
bool is_angle_legal(float, float, float);
void angle_to_coord(float, float, float, float*, float*, float*);
void coord_to_angle(float, float, float, float*, float*, float*);
void coord_arm2effect(float*, float*, float*);
void coord_effect2arm(float*, float*, float*);
static void put(char c) { while (!(UCSR0A & _BV(UDRE0))) {} UDR0 = c; }
static void num(int v) {
    char b[8]; int n = 0;
    if (v < 0) { put('-'); v = -v; }
    do { b[n++] = (char)('0' + v % 10); v /= 10; } while (v);
    while (n) put(b[--n]);
}
int main(void) {
    UCSR0B = _BV(TXEN0);
    uarm.param.front_offset = 56.65f; uarm.param.high_offset = 74.55f;
    for (unsigned i = 0; i < sizeof kPoints / sizeof kPoints[0]; ++i) {
        const int hl = (int16_t)pgm_read_word(&kPoints[i][0]), hr = (int16_t)pgm_read_word(&kPoints[i][1]);
        const float l = hl * 0.5f, r = hr * 0.5f;
        float x, y, z, a2, b2, c2;
        const bool d = is_angle_legal(l, r, 0.0f);
        angle_to_coord(l, r, 0.0f, &x, &y, &z);
        coord_arm2effect(&x, &y, &z);
        coord_effect2arm(&x, &y, &z);
        coord_to_angle(x, y, z, &a2, &b2, &c2);
        const bool t = is_angle_legal(a2, b2, c2);
        num(hl); put(' '); num(hr); put(' '); put(d ? '1' : '0'); put(' '); put(t ? '1' : '0'); put('\n');
    }
    put('E'); put('N'); put('D'); put('\n'); _delay_ms(5); cli(); sleep_cpu();
}
