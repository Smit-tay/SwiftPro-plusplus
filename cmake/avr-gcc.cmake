# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Jack Sidman Smith
#
# CMake toolchain file for the ATmega2560 (Arduino Mega 2560 class board in the uArm Swift Pro) with avr-gcc.
#   cmake -B build/avr --toolchain cmake/avr-gcc.cmake
# Tested with avr-gcc 15.2.0 (Fedora). AVR_TOOLCHAIN_PREFIX may point at another install, e.g.
#   -DAVR_TOOLCHAIN_PREFIX=$HOME/.platformio/packages/toolchain-atmelavr/bin/

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR avr)

set(AVR_TOOLCHAIN_PREFIX "" CACHE PATH "Directory holding avr-gcc etc., with a trailing slash (empty: search PATH)")
set(AVR_MCU atmega2560 CACHE STRING "Target MCU")
set(AVR_F_CPU 16000000UL CACHE STRING "CPU clock (the board's crystal)")

set(CMAKE_C_COMPILER   ${AVR_TOOLCHAIN_PREFIX}avr-gcc)
set(CMAKE_CXX_COMPILER ${AVR_TOOLCHAIN_PREFIX}avr-g++)
set(CMAKE_ASM_COMPILER ${AVR_TOOLCHAIN_PREFIX}avr-gcc)
find_program(AVR_OBJCOPY ${AVR_TOOLCHAIN_PREFIX}avr-objcopy REQUIRED)
find_program(AVR_SIZE    ${AVR_TOOLCHAIN_PREFIX}avr-size REQUIRED)

# There is no OS to link test programs against.
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
