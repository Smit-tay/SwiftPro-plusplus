// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jack Sidman Smith
//
// The serial rings, driven through the ISR entry points. The module has static state, so the checks run in order and
// leave both rings empty.

#include "../src/line.h"
#include "../src/serial.h"
#include "check.h"

#include <cstdint>
#include <string>

namespace {

void rx(const std::string& s, uint8_t port = 0) {
    for (const char c : s) serial::on_rx(port, static_cast<uint8_t>(c));
}

std::string drain_rx(uint8_t port = 0) {
    std::string out;
    uint8_t c;
    while (serial::read(port, &c)) out += static_cast<char>(c);
    return out;
}

std::string drain_tx() {
    std::string out;
    uint8_t c;
    while (serial::tx_pop(&c)) out += static_cast<char>(c);
    return out;
}

}  // namespace

void test_serial() {
    serial::init();
    // Bytes come out in order.
    rx("#1 P2201\n");
    CHECK(drain_rx() == "#1 P2201\n");

    // Q-15: '?' '!' '~' '@' are dropped wherever they are. Ctrl-X is a stop request, not data (Q-48).
    CHECK(!serial::take_stop_request());
    rx("#2 P22?0!1~@\n");
    serial::on_rx(0, 0x18);
    CHECK(drain_rx() == "#2 P2201\n");
    CHECK(serial::take_stop_request());
    CHECK(!serial::take_stop_request());

    // RX overflow (SP-01): data stops at 253 bytes, keeping 2 free. The rest of that line is dropped, and its line
    // end goes in with the lost-bytes marker in front, so the damaged line is reported, not executed.
    rx(std::string(300, 'A'));
    rx("\n");
    std::string got = drain_rx();
    CHECK(got.size() == 255);
    CHECK(got.substr(0, 253) == std::string(253, 'A'));
    CHECK(static_cast<uint8_t>(got[253]) == line::kLostMarker && got[254] == '\n');
    rx("#3P2201\n");  // back to normal
    CHECK(drain_rx() == "#3P2201\n");
    // Back-to-back: if even the reserve is used, dropping continues until a line end finds room.
    rx(std::string(260, 'A') + "\n");  // fills to 253, marker + end use the reserve
    rx("B\n");                         // no room at all: dropped, still dropping
    got = drain_rx();
    CHECK(got.size() == 255);
    rx("C\n");  // room again: the dropped "B" line is marked
    got = drain_rx();
    CHECK(got.size() == 2 && static_cast<uint8_t>(got[0]) == line::kLostMarker && got[1] == '\n');

    // The assembler turns the marker into kLost.
    line::Assembler a;
    line::Result res = line::Result::kNone;
    for (const char c : std::string(10, 'A') + static_cast<char>(line::kLostMarker) + "\n")
        res = a.feed(static_cast<uint8_t>(c));
    CHECK(res == line::Result::kLost);

    // Each port has its own ring (R-6): interleaved bytes from two ports don't mix.
    for (const char c : std::string("P2201\n")) {
        serial::on_rx(0, static_cast<uint8_t>(c));
        serial::on_rx(1, static_cast<uint8_t>(c == '\n' ? '\n' : static_cast<char>(c + 1)));
    }
    CHECK(drain_rx(0) == "P2201\n");
    CHECK(drain_rx(1) == "Q3312\n");
    CHECK(serial::last_port() == 1);
    // The small rings (64 bytes) also keep 2 bytes for the marker.
    rx(std::string(70, 'A') + "\n", 2);
    got = drain_rx(2);
    CHECK(got.size() == 63 && static_cast<uint8_t>(got[61]) == line::kLostMarker);

    // TX: all or nothing.
    CHECK(serial::tx_room(0) == 255);
    CHECK(serial::write(0, "$1 ok VSwiftPro\n"));
    CHECK(serial::tx_room(0) == 255 - 16);
    // While the ring holds bytes for port 0, port 1 must wait: no line is ever split across ports.
    CHECK(serial::tx_room(1) == 0);
    CHECK(!serial::write(1, "x", 1));
    CHECK(!serial::write(0, std::string(255, 'x').c_str()));  // doesn't fit: nothing written
    CHECK(serial::tx_room(0) == 255 - 16);
    CHECK(drain_tx() == "$1 ok VSwiftPro\n");
    CHECK(serial::tx_room(1) == 255);  // drained: port 1 may use it now
    CHECK(serial::write(1, std::string(255, 'y').c_str()));  // exactly fits
    CHECK(serial::tx_room(1) == 0);
    CHECK(!serial::write(1, "z", 1));
    CHECK(drain_tx() == std::string(255, 'y'));
    CHECK(serial::write(0, "", 0));
    CHECK(serial::tx_room(0) == 255);
}
