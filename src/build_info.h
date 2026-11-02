// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jack Sidman Smith
//
// Build stamp (R-18): when and from which commit this firmware was built, e.g. "SPP-20261102T143012Z-cf72a17-dirty".
// Generated on every build by cmake/build_info.cmake; read it with P2210, or find it in a flash read-back with
//   avr-objcopy -I ihex -O binary dump.hex dump.bin && strings dump.bin | grep SPP-

#pragma once

#include "fmt.h"

namespace build_info {

// The stamp, kept in flash.
const fmt::FlashText* stamp();

}  // namespace build_info
