// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jack Sidman Smith

#include "app.h"

int main() {
    app::init();
    for (;;) app::poll();
}
