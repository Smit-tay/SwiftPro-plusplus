// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jack Sidman Smith
//
// Minimal check macro for the host unit tests.

#pragma once

#include <cstdio>
#include <cstring>

extern int g_failures;

#define CHECK(cond)                                                                \
    do {                                                                           \
        if (!(cond)) {                                                             \
            std::printf("%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #cond);   \
            ++g_failures;                                                          \
        }                                                                          \
    } while (0)

#define CHECK_STR(a, b) CHECK(std::strcmp((a), (b)) == 0)
