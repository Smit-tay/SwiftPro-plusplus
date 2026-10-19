// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jack Sidman Smith
//
// Host unit tests: one function per module, run in order.

#include <cstdio>

int g_failures = 0;

void test_line();
void test_parse();
void test_serial();
void test_fmt();
void test_kin();
void test_motion();
void test_watch();

int main() {
    test_line();
    test_parse();
    test_serial();
    test_fmt();
    test_kin();
    test_motion();
    test_watch();
    if (g_failures != 0) {
        std::printf("%d check(s) failed\n", g_failures);
        return 1;
    }
    std::printf("all unit tests passed\n");
    return 0;
}
