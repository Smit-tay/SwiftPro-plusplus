// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jack Sidman Smith
//
// Kinematics against the original's results on the AVR (test/data/kin.txt, from tools/kin under simavr). The port
// is its own implementation (ARCHITECTURE section 1), so it matches within a tolerance, not bit for bit. The test
// also counts values whose 4-decimal text (what the replies print) differs, and prints the counts.

#include "../src/fmt.h"
#include "../src/kin.h"
#include "check.h"

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace {

float bits(const std::string& h) {
    const uint32_t u = static_cast<uint32_t>(std::strtoul(h.c_str(), nullptr, 16));
    float f;
    std::memcpy(&f, &u, sizeof f);
    return f;
}

std::string f4(float v) {
    char b[64];
    fmt::Writer w(b, sizeof b);
    w.f4(v);
    return w.text();
}

struct Stats {
    double worst = 0.0;
    int values = 0;
    int text_differs = 0;  // 4-decimal text differs
    int nan_mismatch = 0;
};

// Within tol (absolute); NaN only matches NaN. Where the original gave NaN only because x < 0 (Q-57: the port
// computes the real direction), any value is accepted.
bool close(Stats& st, float want, float got, double tol, bool want_nan_ok = false) {
    ++st.values;
    if (want_nan_ok && std::isnan(want)) return true;
    if (std::isnan(want) || std::isnan(got)) {
        if (std::isnan(want) != std::isnan(got)) ++st.nan_mismatch;
        return std::isnan(want) && std::isnan(got);
    }
    const double d = std::fabs(static_cast<double>(want) - static_cast<double>(got));
    if (d > st.worst) st.worst = d;
    if (f4(want) != f4(got)) ++st.text_differs;
    return d <= tol;
}

}  // namespace

void test_kin() {
    std::ifstream in("test/data/kin.txt");
    CHECK(static_cast<bool>(in));
    std::string line;
    int rows = 0, bad = 0, legal_differs = 0, legal_vs_original = 0, step_differs = 0;
    Stats fk, ik, other;
    const kin::Offsets off{56.65f, 74.55f};
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream ss(line);
        std::string kind, i0, i1, i2, colon;
        ss >> kind >> i0 >> i1 >> i2 >> colon;
        std::vector<std::string> o;
        for (std::string t; ss >> t;) o.push_back(t);
        const float a = bits(i0), b = bits(i1), c = bits(i2);
        bool ok = true;
        float x, y, z;
        if (kind == "A") {
            kin::forward(a, b, c, &x, &y, &z);
            ok = close(fk, bits(o[0]), x, 1e-3) && ok;
            ok = close(fk, bits(o[1]), y, 1e-3) && ok;
            ok = close(fk, bits(o[2]), z, 1e-3) && ok;
            const bool behind = x < 0.0f;  // the original's offsets are NaN here (Q-57)
            kin::arm_to_effector(off, &x, &y, &z);
            ok = close(fk, bits(o[3]), x, 1e-3, behind) && ok;
            ok = close(fk, bits(o[4]), y, 1e-3, behind) && ok;
            ok = close(fk, bits(o[5]), z, 1e-3) && ok;
            kin::effector_to_arm(off, &x, &y, &z);
            ok = close(fk, bits(o[6]), x, 1e-3, behind) && ok;
            ok = close(fk, bits(o[7]), y, 1e-3, behind) && ok;
            ok = close(fk, bits(o[8]), z, 1e-3) && ok;
            if ((o[9] == "1") != kin::legal(a, b, c)) ++legal_vs_original;  // Q-83: other limits, by design
        } else if (kind == "C") {
            kin::inverse(a, b, c, &x, &y, &z);
            const float oa = bits(o[0]), ob = bits(o[1]), oc = bits(o[2]);
            // Only where the original's answer is a real arm pose: outside the joint limits, the original's sign
            // fix-ups (uarm_coord_convert.c:150-160) and the elbow-up solution may pick different branches, and both
            // are refused as unreachable (E22) by the legality check.
            if (a < 0.0f) {  // behind the base axis: the original's base angle is NaN, the port's real (Q-57)
                CHECK(std::isnan(oc) && !std::isnan(z) && std::fabs(z) > 90.0f);
            } else if (kin::legal(oa, ob, oc)) {
                ok = close(ik, oa, x, 2e-3) && ok;
                ok = close(ik, ob, y, 2e-3) && ok;
                ok = close(ik, oc, z, 2e-3) && ok;
                if (!kin::legal(x, y, z)) ++legal_differs;
            } else if (kin::legal(x, y, z)) {
                ++legal_differs;
            }
        } else if (kind == "S") {
            // R-1: the port rounds where the original truncated, so they differ by at most one step.
            const int16_t sl = kin::arm_steps(a, b), sb = kin::base_steps(a, c);
            if (std::fabs(bits(o[0]) - static_cast<float>(sl)) > 1.0f) ++step_differs;
            if (std::fabs(bits(o[1]) - static_cast<float>(sb)) > 1.0f) ++step_differs;
            CHECK(sl == static_cast<int16_t>(std::lround((static_cast<double>(a) - static_cast<double>(b)) * 80.0)) ||
                  std::fabs(std::fmod((static_cast<double>(a) - static_cast<double>(b)) * 80.0, 1.0)) > 0.4999);
            ok = close(other, bits(o[2]), kin::arm_angle(a, b), 1e-4) && ok;
            ok = close(other, bits(o[3]), kin::base_angle(a, c), 1e-4) && ok;
        } else {
            if ((o[0] == "1") != kin::legal(a, b, c)) ++legal_vs_original;
        }
        ++rows;
        if (!ok && ++bad <= 5) std::printf("kin row: %s\n", line.c_str());
    }
    CHECK(rows == 1812);
    CHECK(bad == 0);
    CHECK(legal_differs == 0);
    CHECK(fk.nan_mismatch == 0 && ik.nan_mismatch == 0);
    std::printf("kin vs original: FK worst %.2g (text differs %d/%d), IK worst %.2g (text differs %d/%d), "
                "steps off by more than 1: %d/1600, step angles worst %.2g; legality differs (Q-83 limits) %d\n",
                fk.worst, fk.text_differs, fk.values, ik.worst, ik.text_differs, ik.values, step_differs,
                other.worst, legal_vs_original);
    CHECK(step_differs == 0);

    // Your arm, probe 2026-10-17: P2242 refs B1655 L2673 R1664, P2243 raw B1654 L2820 R1403 gave
    // P2200 B89.9121 L45.6789 R66.0411 and P2220 X233.6009 Y-0.3583 Z-10.6346 (mode 0 offsets).
    // Encoder angle = pointB - (ref - raw) * 360 / 4096 (uarm_angle.c:222).
    const float l = 32.758930f - static_cast<float>(2673 - 2820) * 360.0f / 4096.0f;
    const float r = 88.980537f - static_cast<float>(1664 - 1403) * 360.0f / 4096.0f;
    const float base = 90.0f - static_cast<float>(1655 - 1654) * 360.0f / 4096.0f;
    CHECK(f4(base) == "89.9121" && f4(l) == "45.6789" && f4(r) == "66.0411");
    float px, py, pz;
    kin::forward(l, r, base - 90.0f, &px, &py, &pz);  // P2220 passes base - 90 (uarm_protocol.c:1426)
    kin::arm_to_effector(off, &px, &py, &pz);
    CHECK(f4(px) == "233.6009" && f4(py) == "-0.3583" && f4(pz) == "-10.6346");
}

// The joint limits (Q-83, docs/LIMITS.md section 6): this copy of the table must match kin.cpp's, the limits must lie
// inside the original's everywhere (never outside COMMANDABLE), the region must have no dents, and an L jog must end
// where a fine walk along L does.
void test_limits() {
    static const float kTable[32][3] = {  // L, R min, R max: docs/LIMITS.md section 6
        {-15, 56, 112}, {-10, 51, 112}, {-5, 46, 112}, {0, 41, 112},    {5, 36, 112},   {10, 31, 112},  {15, 26, 112},
        {20, 21, 112},  {25, 16, 111},  {30, 11, 110}, {35, 6, 109},    {40, 1, 108},   {45, -4, 106},  {50, -8, 104},
        {55, -12, 102}, {60, -16, 100}, {65, -18, 98}, {70, -18, 94},   {75, -18, 90},  {80, -18, 86},  {85, -18, 82},
        {90, -18, 78},  {95, -18, 73},  {100, -18, 68}, {105, -18, 63}, {110, -18, 58}, {115, -18, 53}, {120, -18, 48},
        {125, -18, 43}, {130, -18, 38}, {135, -18, 33}, {140, -18, 28},
    };
    // No dents: R min convex and R max concave in L, so a straight joint-space move between two legal poses stays legal.
    for (int i = 1; i < 31; ++i) {
        CHECK(kTable[i - 1][1] + kTable[i + 1][1] - 2 * kTable[i][1] >= 0);
        CHECK(kTable[i - 1][2] + kTable[i + 1][2] - 2 * kTable[i][2] <= 0);
    }
    constexpr float e = 0.05f;
    for (const auto& t : kTable) {
        const kin::Range r = kin::limit_r(t[0]);
        CHECK(std::fabs(r.lo - t[1]) < 1e-4f && std::fabs(r.hi - t[2]) < 1e-4f);
        if (t[0] < kin::kLimitL.lo || t[0] > kin::kLimitL.hi) continue;
        CHECK(kin::legal(t[0], t[1] + e, 0.0f) && !kin::legal(t[0], t[1] - e, 0.0f));
        CHECK(kin::legal(t[0], t[2] - e, 0.0f) && !kin::legal(t[0], t[2] + e, 0.0f));
    }
    const kin::Range mid = kin::limit_r(47.5f);  // halfway between the L 45 and 50 points
    CHECK(std::fabs(mid.lo + 6.0f) < 1e-4f && std::fabs(mid.hi - 105.0f) < 1e-4f);
    CHECK(kin::legal(-13.8f + e, 60.0f, 0.0f) && !kin::legal(-13.8f - e, 60.0f, 0.0f));
    CHECK(kin::legal(135.6f - e, 20.0f, 0.0f) && !kin::legal(135.6f + e, 20.0f, 0.0f));
    CHECK(kin::legal(90.0f, 30.0f, -95.0f + e) && !kin::legal(90.0f, 30.0f, -95.0f - e));  // user frame -5
    CHECK(kin::legal(90.0f, 30.0f, 95.0f - e) && !kin::legal(90.0f, 30.0f, 95.0f + e));    // user frame 185
    CHECK(kin::legal(45.6789f, 66.0411f, -0.0879f));  // your probe pose, 2026-10-17
    CHECK(!kin::legal(-14.3f, 80.0f, 0.0f));          // f01e691's taught L: now 0.5 deg from the stop
    CHECK(!kin::legal(std::nanf(""), 60.0f, 0.0f) && !kin::legal(60.0f, std::nanf(""), 0.0f));

    // Inside the original's limits everywhere (L -4..135.6 aside: ours reaches -13.8 as jss.7's -20 did).
    int outside = 0;
    for (float a = kin::kLimitL.lo; a <= kin::kLimitL.hi; a += 0.05f) {
        const kin::Range r = kin::limit_r(a);
        if (r.lo < -20.0f || r.hi > 119.9f || 180.0f - a - r.hi < 10.0f || 180.0f - a - r.lo > 151.0f) ++outside;
    }
    CHECK(outside == 0);

    // l_reach against a walk along L in 0.01 deg steps, from a grid of starts, both ways.
    int starts = 0, differs = 0;
    double worst = 0.0;
    for (float a = kin::kLimitL.lo; a <= kin::kLimitL.hi; a += 3.7f) {
        for (float b = -18.0f; b <= 114.0f; b += 2.9f) {
            if (!kin::legal(a, b, 0.0f)) {
                CHECK(kin::l_reach(a, b, true, 1.0f) == a && kin::l_reach(a, b, false, 1.0f) == a);
                continue;
            }
            for (int up = 0; up < 2; ++up) {
                const float step = up ? 0.01f : -0.01f;
                float want = a;
                for (int n = 1;; ++n) {
                    const float x = a + step * static_cast<float>(n);
                    if (!kin::legal(x, b, 0.0f)) break;
                    if (kin::legal(x, b, 0.0f, 1.0f)) want = x;
                }
                const float got = kin::l_reach(a, b, up != 0, 1.0f);
                const double d = std::fabs(static_cast<double>(got) - static_cast<double>(want));
                if (d > worst) worst = d;
                if (d > 0.011) {
                    if (++differs <= 5) std::printf("l_reach(%g, %g, %d) = %g, walk %g\n", static_cast<double>(a), static_cast<double>(b), up,
                                static_cast<double>(got), static_cast<double>(want));
                }
                if (got != a) CHECK(kin::legal(got, b, 0.0f, 0.999f));
                ++starts;
            }
        }
    }
    std::printf("limits: l_reach vs walk, %d starts, worst %.3g deg\n", starts, worst);
    CHECK(differs == 0);
}
