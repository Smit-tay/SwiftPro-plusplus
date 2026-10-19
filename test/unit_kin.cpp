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
    int rows = 0, bad = 0, legal_differs = 0, step_differs = 0;
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
            if ((o[9] == "1") != kin::legal(a, b, c)) ++legal_differs;
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
            if ((o[0] == "1") != kin::legal(a, b, c)) ++legal_differs;
        }
        ++rows;
        if (!ok && ++bad <= 5) std::printf("kin row: %s\n", line.c_str());
    }
    CHECK(rows == 1812);
    CHECK(bad == 0);
    CHECK(legal_differs == 0);
    CHECK(fk.nan_mismatch == 0 && ik.nan_mismatch == 0);
    std::printf("kin vs original: FK worst %.2g (text differs %d/%d), IK worst %.2g (text differs %d/%d), "
                "steps off by more than 1: %d/1600, step angles worst %.2g\n",
                fk.worst, fk.text_differs, fk.values, ik.worst, ik.text_differs, ik.values, step_differs,
                other.worst);
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
