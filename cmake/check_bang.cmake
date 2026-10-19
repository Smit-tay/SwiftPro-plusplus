# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Jack Sidman Smith
#
# Fails if the firmware image contains "!!!" (Q-40). The stk500v2 bootloader on the board (Arduino "stk500V2 by MLS",
# 2013) enters its serial monitor when it sees three '!' in a row, which breaks an upload that carries them.
#   cmake -DIMAGE=<file.bin> -P check_bang.cmake

file(READ "${IMAGE}" hex HEX)
set(pos 0)
while(TRUE)
    string(FIND "${hex}" "212121" at)
    if(at EQUAL -1)
        break()
    endif()
    math(EXPR parity "(${pos} + ${at}) % 2")
    if(parity EQUAL 0)
        math(EXPR offset "(${pos} + ${at}) / 2")
        message(FATAL_ERROR "${IMAGE} contains \"!!!\" at byte offset ${offset}: the bootloader would break the upload (Q-40)")
    endif()
    math(EXPR skip "${at} + 1")
    string(SUBSTRING "${hex}" ${skip} -1 hex)
    math(EXPR pos "${pos} + ${skip}")
endwhile()
message(STATUS "${IMAGE}: no \"!!!\" (Q-40)")
