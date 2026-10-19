// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jack Sidman Smith

#include "adc.h"

#if defined(__AVR__)
#include <avr/io.h>
#endif

namespace adc {

#if defined(__AVR__)

void init() { ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0); }

uint16_t read(uint8_t channel) {
    channel = static_cast<uint8_t>(channel & 15);
    ADCSRB = static_cast<uint8_t>((ADCSRB & ~_BV(MUX5)) | ((channel & 8) ? _BV(MUX5) : 0));
    ADMUX = static_cast<uint8_t>(_BV(REFS0) | (channel & 7));
    ADCSRA = static_cast<uint8_t>(ADCSRA | _BV(ADSC));
    while (ADCSRA & _BV(ADSC)) {}
    return ADC;
}

#else

namespace {
uint16_t values[16] = {};
}  // namespace

void init() {}
uint16_t read(uint8_t channel) { return values[channel & 15]; }
void host_set(uint8_t channel, uint16_t value) { values[channel & 15] = value; }

#endif

}  // namespace adc
