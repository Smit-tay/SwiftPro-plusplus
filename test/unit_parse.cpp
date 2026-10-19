// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jack Sidman Smith

#include "../src/parse.h"
#include "check.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>

namespace {

bool rf(const char* s, float* v, const char** end = nullptr) { return parse::read_float(s, end, v); }

}  // namespace

void test_parse() {
    // Number reading (R-2, Q-53). The host's strtof (correctly rounded) is the reference.
    float v = 0.0f;
    const char* end = nullptr;
    CHECK(rf("45", &v) && v == 45.0f);
    CHECK(rf("-1.25", &v) && v == -1.25f);
    CHECK(rf("+3", &v) && v == 3.0f);
    CHECK(rf("007", &v) && v == 7.0f);
    CHECK(rf("45.6789", &v, &end) && v == std::strtof("45.6789", nullptr) && *end == '\0');
    CHECK(rf("-0", &v) && v == 0.0f && std::signbit(v));
    CHECK(rf(".5", &v) && v == 0.5f);
    CHECK(rf("1.", &v) && v == 1.0f);
    CHECK(rf("12F100", &v, &end) && v == 12.0f && *end == 'F');
    CHECK(!rf("", &v));
    CHECK(!rf(".", &v));
    CHECK(!rf("-", &v));
    CHECK(!rf("+-1", &v));
    CHECK(!rf("1.2.3", &v));  // Q-53: a second '.'
    CHECK(!rf("1-2", &v));    // Q-53: a stray sign
    // Exactly the correctly rounded value for host-style numbers: up to 7 significant digits, up to 4 decimals.
    std::srand(20261018);
    int inexact = 0;
    for (int i = 0; i < 20000; ++i) {
        char buf[32];
        const int digits = std::rand() % 10000000;
        const int decimals = std::rand() % 5;
        const int whole = digits;
        std::snprintf(buf, sizeof buf, "%s%d", (std::rand() & 1) ? "-" : "", whole);
        std::string text = buf;
        if (decimals > 0) {
            while (static_cast<int>(text.size()) - (text[0] == '-') <= decimals) text.insert(text[0] == '-' ? 1 : 0, "0");
            text.insert(text.size() - static_cast<size_t>(decimals), ".");
        }
        if (!rf(text.c_str(), &v) || v != std::strtof(text.c_str(), nullptr)) ++inexact;
    }
    CHECK(inexact == 0);
    // Longer numbers: within one ulp.
    CHECK(rf("123456789", &v) && std::fabs(v - 123456789.0f) <= 8.0f);
    CHECK(rf("0.000012345678", &v) && std::fabs(v - 1.2345678e-5f) <= 1e-12f);

    // Routing (uarm_protocol.c:23-71).
    parse::Routed r = parse::route("$I");
    CHECK(r.route == parse::Route::kSystem);
    r = parse::route("#12P2201");
    CHECK(r.route == parse::Route::kAsync);
    CHECK_STR(r.seq, "12");
    CHECK_STR(r.body, "P2201");
    r = parse::route("#3G2202N1V45F100");
    CHECK(r.route == parse::Route::kSync);
    CHECK_STR(r.body, "G2202N1V45F100");
    r = parse::route("#P2201");  // '#' without digits: no "$N", routing continues (faithful)
    CHECK(r.route == parse::Route::kAsync);
    CHECK_STR(r.seq, "");
    CHECK_STR(r.body, "P2201");
    CHECK(parse::route("#5X10").route == parse::Route::kNotCommand);  // SP-52
    CHECK(parse::route("#5$I").route == parse::Route::kNotCommand);   // SP-52
    CHECK(parse::route("#5").route == parse::Route::kNotCommand);     // SP-52
    r = parse::route("#123456789P2201");
    CHECK(r.route == parse::Route::kAsync);
    CHECK_STR(r.seq, "123456789");
    CHECK(parse::route("#1234567890P2201").route == parse::Route::kBadSequence);  // SP-03
    CHECK(parse::route("M2200").route == parse::Route::kSync);
    CHECK(parse::route("S1100").route == parse::Route::kAsync);
    CHECK(parse::route("X10").route == parse::Route::kIgnore);
    CHECK(parse::route("").route == parse::Route::kIgnore);

    // Command word (gcode.c:113-129).
    parse::Command c{};
    CHECK(parse::command("G2202N1V45F100", &c) == parse::Word::kOk);
    CHECK(c.family == 'G' && c.number == 2202);
    CHECK_STR(c.params, "N1V45F100");
    CHECK(parse::command("G2202.5N1", &c) == parse::Word::kNotInteger);  // Q-53 (the original ran it as G2202)
    CHECK(parse::command("G2202.004N1", &c) == parse::Word::kNotInteger);
    CHECK(parse::command("G2202.0N1", &c) == parse::Word::kOk && c.number == 2202);  // ".0" is a whole number
    CHECK(parse::command("M2201.5", &c) == parse::Word::kNotInteger);
    CHECK(parse::command("P", &c) == parse::Word::kBadNumber);
    CHECK(parse::command("MX", &c) == parse::Word::kBadNumber);
    CHECK(parse::command("M67737", &c) == parse::Word::kOutOfRange);  // the original ran it as M2201
    CHECK(parse::command("G-1", &c) == parse::Word::kOutOfRange);
    CHECK(parse::command("M65535", &c) == parse::Word::kOk && c.number == 65535);
    CHECK(parse::command("G0X1Y2Z3F10", &c) == parse::Word::kOk);
    CHECK(c.number == 0);
    CHECK_STR(c.params, "X1Y2Z3F10");

    // Field scanners.
    const char* p = "V45F100";
    CHECK(parse::expect(&p, 'V'));
    CHECK(!parse::expect(&p, 'V'));
    CHECK(parse::number(&p, 19, &v) && v == 45.0f);
    CHECK_STR(p, "F100");
    p = "1.2.3F1";  // Q-53: a malformed number is refused (the original read 1.2 and ignored ".3")
    CHECK(!parse::number(&p, 19, &v));
    p = "-F1";  // a run without a digit: SP-10 (the original replied "ok")
    CHECK(!parse::number(&p, 19, &v));
    p = "F1";  // empty run: sscanf no match
    CHECK(!parse::number(&p, 19, &v));
    p = "1234567890F1";  // 10 characters into a 9-character field: SP-03
    CHECK(!parse::number(&p, 9, &v));
    p = "123456789F1";
    CHECK(parse::number(&p, 9, &v));

    int16_t i = 0;
    p = "1V";
    CHECK(parse::integer(&p, &i) && i == 1);
    CHECK_STR(p, "V");
    p = "-3";
    CHECK(parse::integer(&p, &i) && i == -3);
    p = "+7";
    CHECK(parse::integer(&p, &i) && i == 7);
    p = "0000001";
    CHECK(parse::integer(&p, &i) && i == 1);
    p = "-32768";
    CHECK(parse::integer(&p, &i) && i == -32768);
    p = "32768";
    CHECK(!parse::integer(&p, &i));
    p = "V1";
    CHECK(!parse::integer(&p, &i));
    p = "-";
    CHECK(!parse::integer(&p, &i));
}
