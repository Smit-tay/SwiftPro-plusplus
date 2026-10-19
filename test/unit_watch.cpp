// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jack Sidman Smith
//
// The lost-step watch (SP-41) through the simulator, for cases the golden traces can't time: an encoder that jumps
// mid-move and then follows its motor again (a slipping gear or coupling), which a stall test doesn't cover.

#include "check.h"
#include "sim.h"

#include "../src/commands.h"

#include <string>

namespace {

void boot() {
    sim_reset();
    sim_set("mode", "0");
    sim_set("enc.ref", "B1655 L2673 R1664");
    sim_set("enc.raw", "B1654 L2820 R1403");
    sim_boot();
    sim_advance_ms(500);
    sim_take_tx();
}

void run_until_idle() {
    sim_settle();
    for (int ms = 0; ms < 5000 && !sim_idle(); ++ms) sim_advance_ms(1);
}

}  // namespace

void test_watch() {
    // No slip: no @8, the move succeeds.
    boot();
    sim_rx("#1 G2202 N2 V100 F1000\n");
    run_until_idle();
    CHECK(sim_take_tx() == "$1 ok\n");

    // R jumps by 35 counts (3.08 deg) 50 ms into the move, then follows again: caught, E25, tally re-synced.
    boot();
    sim_rx("#2 G2202 N2 V100 F1000\n");
    sim_advance_ms(50);
    CHECK(sim_set("enc.slip", "R35"));
    run_until_idle();
    const std::string out = sim_take_tx();
    CHECK(out.rfind("@8 N2 V-3.", 0) == 0);
    CHECK(out.find("$2 E25\n") != std::string::npos);
    CHECK(commands::idle());

    // Bad values for the sim key itself.
    CHECK(!sim_set("enc.slip", "X1"));
    CHECK(!sim_set("enc.slip", "R"));
}
