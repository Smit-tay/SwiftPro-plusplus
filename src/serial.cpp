// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jack Sidman Smith

#include "serial.h"

#include "atomic.h"
#include "line.h"

#if defined(__AVR__)
#include <avr/interrupt.h>
#include <avr/io.h>
#endif

namespace serial {

namespace {

static_assert(kTxSize == 256, "the TX ring uses 8-bit indices");

// A receive ring of a power-of-two size up to 256. head is written by the RX ISR, tail by the main loop.
struct RxRing {
    uint8_t* buf;
    uint8_t mask;
    volatile uint8_t head;
    volatile uint8_t tail;
    bool dropping;  // ISR only: bytes are being dropped until the end of the damaged line (SP-01)

    uint8_t free() const { return static_cast<uint8_t>((tail - head - 1) & mask); }
    void push(uint8_t c) {
        buf[head] = c;
        head = static_cast<uint8_t>((head + 1) & mask);
    }
};

uint8_t rx_usb[kRxSizeUsb];
uint8_t rx_other[kPorts - 1][kRxSizeOther];
RxRing rx[kPorts] = {
    {rx_usb, static_cast<uint8_t>(kRxSizeUsb - 1), 0, 0, false},
    {rx_other[0], static_cast<uint8_t>(kRxSizeOther - 1), 0, 0, false},
    {rx_other[1], static_cast<uint8_t>(kRxSizeOther - 1), 0, 0, false},
    {rx_other[2], static_cast<uint8_t>(kRxSizeOther - 1), 0, 0, false},
};
constexpr uint8_t kReserve = 2;  // room kept for the lost-bytes marker and the line end

uint8_t tx_buf[kTxSize];
volatile uint8_t tx_head = 0;  // written by the main loop
volatile uint8_t tx_tail = 0;  // written by the UDRE ISR
uint8_t tx_port = 0;           // the port the TX ring is serving

volatile uint8_t rx_last = 0;  // the port that received the last byte
volatile bool stop_request = false;

#if defined(__AVR__)
// Bit positions are the same in all four USARTs.
static_assert(U2X1 == U2X0 && U2X2 == U2X0 && U2X3 == U2X0, "USART bit layout");
static_assert(RXEN1 == RXEN0 && TXEN1 == TXEN0 && RXCIE1 == RXCIE0 && UDRIE1 == UDRIE0, "USART bit layout");
static_assert(RXEN3 == RXEN0 && TXEN3 == TXEN0 && RXCIE3 == RXCIE0 && UDRIE3 == UDRIE0, "USART bit layout");

constexpr uint32_t kBaud = 115200;  // faithful config.h:37
// Faithful serial.c:66-71: U2X on, UBRR = ((F_CPU / (4 * baud)) - 1) / 2 = 16, i.e. 117647 baud (+2.1 %).
constexpr uint16_t kUbrr = static_cast<uint16_t>(((F_CPU / (4UL * kBaud)) - 1) / 2);
static_assert(kUbrr == 16, "UBRR differs from the original");

void init_port(volatile uint8_t& ucsra, volatile uint8_t& ucsrb, volatile uint8_t& ubrrh, volatile uint8_t& ubrrl) {
    ubrrh = static_cast<uint8_t>(kUbrr >> 8);
    ubrrl = static_cast<uint8_t>(kUbrr & 0xFF);
    ucsra = static_cast<uint8_t>(ucsra | _BV(U2X0));
    ucsrb = static_cast<uint8_t>(ucsrb | _BV(RXEN0) | _BV(TXEN0) | _BV(RXCIE0));
    // 8N1 is the reset default (faithful serial.c:141).
}

void start_tx() {
    // UCSRnB is also written by the UDRE ISR; USART1-3 are outside the bit-instruction range (SP-28).
    InterruptLock lock;
    switch (tx_port) {
        case 0: UCSR0B = static_cast<uint8_t>(UCSR0B | _BV(UDRIE0)); break;
        case 1: UCSR1B = static_cast<uint8_t>(UCSR1B | _BV(UDRIE1)); break;
        case 2: UCSR2B = static_cast<uint8_t>(UCSR2B | _BV(UDRIE2)); break;
        default: UCSR3B = static_cast<uint8_t>(UCSR3B | _BV(UDRIE3)); break;
    }
}

bool tx_busy() {
    switch (tx_port) {
        case 0: return (UCSR0B & _BV(UDRIE0)) != 0;
        case 1: return (UCSR1B & _BV(UDRIE1)) != 0;
        case 2: return (UCSR2B & _BV(UDRIE2)) != 0;
        default: return (UCSR3B & _BV(UDRIE3)) != 0;
    }
}
#else
void start_tx() {}
bool tx_busy() { return false; }
#endif

}  // namespace

void init() {
    for (RxRing& r : rx) {
        r.head = r.tail = 0;
        r.dropping = false;
    }
    tx_head = tx_tail = 0;
    tx_port = 0;
    rx_last = 0;
    stop_request = false;
#if defined(__AVR__)
    init_port(UCSR0A, UCSR0B, UBRR0H, UBRR0L);
    init_port(UCSR1A, UCSR1B, UBRR1H, UBRR1L);
    init_port(UCSR2A, UCSR2B, UBRR2H, UBRR2L);
    init_port(UCSR3A, UCSR3B, UBRR3H, UBRR3L);
    // Q-16: pull-ups on RXD0 PE0, RXD1 PD2, RXD2 PH0, RXD3 PJ0 (ATmega2560 datasheet pin table; the original uses
    // the hardware USARTs without touching these pins). Called before interrupts are enabled.
    PORTE = static_cast<uint8_t>(PORTE | _BV(PE0));
    PORTD = static_cast<uint8_t>(PORTD | _BV(PD2));
    PORTH = static_cast<uint8_t>(PORTH | _BV(PH0));
    PORTJ = static_cast<uint8_t>(PORTJ | _BV(PJ0));
#endif
}

bool read(uint8_t port, uint8_t* c) {
    RxRing& r = rx[port];
    const uint8_t tail = r.tail;
    if (tail == r.head) return false;
    *c = r.buf[tail];
    r.tail = static_cast<uint8_t>((tail + 1) & r.mask);
    return true;
}

uint16_t tx_room(uint8_t port) {
    if (port != tx_port && (tx_head != tx_tail || tx_busy())) return 0;  // still sending for another port
    return static_cast<uint8_t>(tx_tail - tx_head - 1);
}

bool write(uint8_t port, const char* s, uint16_t n) {
    if (n > tx_room(port)) return false;
    if (n == 0) return true;
    tx_port = port;  // tx_room() guarantees the ring is idle if this changes it
    uint8_t head = tx_head;
    for (uint16_t i = 0; i < n; ++i) {
        tx_buf[head] = static_cast<uint8_t>(s[i]);
        head = static_cast<uint8_t>(head + 1);
    }
    tx_head = head;
    start_tx();
    return true;
}

bool write(uint8_t port, const char* s) {
    uint16_t n = 0;
    while (s[n] != '\0') ++n;
    return write(port, s, n);
}

uint8_t last_port() { return rx_last; }

bool take_stop_request() {
    if (!stop_request) return false;
    stop_request = false;
    return true;
}

void on_rx(uint8_t port, uint8_t c) {
    rx_last = port;
    switch (c) {
        case 0x18: stop_request = true; return;          // Ctrl-X (Q-48)
        case '?': case '!': case '~': case '@': return;  // Q-15: dropped, no action
        default: break;
    }
    RxRing& r = rx[port];
    if (r.dropping) {  // SP-01: drop the rest of the damaged line, then mark it
        if ((c == '\n' || c == '\r') && r.free() >= kReserve) {
            r.push(line::kLostMarker);
            r.push(c);
            r.dropping = false;
        }
        return;
    }
    // Two bytes stay free, so the end of a damaged line can always be marked (SP-01).
    if (r.free() <= kReserve) {
        r.dropping = true;  // this byte is lost
        return;
    }
    r.push(c);
}

bool tx_pop(uint8_t* c) {
    const uint8_t tail = tx_tail;
    if (tail == tx_head) return false;
    *c = tx_buf[tail];
    tx_tail = static_cast<uint8_t>(tail + 1);
    return true;
}

}  // namespace serial

#if defined(__AVR__)
ISR(USART0_RX_vect) { serial::on_rx(0, UDR0); }
ISR(USART1_RX_vect) { serial::on_rx(1, UDR1); }
ISR(USART2_RX_vect) { serial::on_rx(2, UDR2); }
ISR(USART3_RX_vect) { serial::on_rx(3, UDR3); }

ISR(USART0_UDRE_vect) {
    uint8_t c;
    if (serial::tx_pop(&c)) UDR0 = c;
    else UCSR0B = static_cast<uint8_t>(UCSR0B & ~_BV(UDRIE0));
}
ISR(USART1_UDRE_vect) {
    uint8_t c;
    if (serial::tx_pop(&c)) UDR1 = c;
    else UCSR1B = static_cast<uint8_t>(UCSR1B & ~_BV(UDRIE1));
}
ISR(USART2_UDRE_vect) {
    uint8_t c;
    if (serial::tx_pop(&c)) UDR2 = c;
    else UCSR2B = static_cast<uint8_t>(UCSR2B & ~_BV(UDRIE2));
}
ISR(USART3_UDRE_vect) {
    uint8_t c;
    if (serial::tx_pop(&c)) UDR3 = c;
    else UCSR3B = static_cast<uint8_t>(UCSR3B & ~_BV(UDRIE3));
}
#endif
