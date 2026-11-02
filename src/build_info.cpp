// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jack Sidman Smith
//
// Kept in its own file so that only this file is recompiled when the stamp changes (every build).

#include "build_info.h"

#include "build_stamp.h"  // generated: SPP_BUILD_STAMP

namespace build_info {

const fmt::FlashText* stamp() { return FLASH(SPP_BUILD_STAMP); }

}  // namespace build_info
