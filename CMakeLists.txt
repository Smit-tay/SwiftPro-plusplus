# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Jack Sidman Smith
#
# SwiftPro++ firmware for the uArm Swift Pro (ATmega2560).
#
# Two builds from the same tree:
#
#   Host (native compiler): unit tests and golden protocol traces against the simulator.
#     cmake -B build/host
#     cmake --build build/host
#     ctest --test-dir build/host --output-on-failure
#
#   Firmware (avr-gcc): swiftpro-pp.elf, swiftpro-pp.hex, a size report and the "!!!" check (Q-40).
#     cmake -B build/avr --toolchain cmake/avr-gcc.cmake
#     cmake --build build/avr
#
# Flashing is done by hand (README.md); nothing here talks to the board.

cmake_minimum_required(VERSION 3.20)
project(swiftpro_pp VERSION 5.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Firmware modules shared by both builds. i2c.cpp drives the pins directly and exists only on the AVR; on the host
# the simulator (test/sim.cpp) provides the i2c functions.
set(MODULES
    src/line.cpp src/parse.cpp src/serial.cpp src/fmt.cpp src/tick.cpp src/nvm.cpp src/board.cpp
    src/commands.cpp src/protocol.cpp src/app.cpp src/kin.cpp src/encoder.cpp
    src/planner.cpp src/stepgen.cpp src/stepper.cpp src/motion.cpp src/adc.cpp src/effector.cpp src/buzzer.cpp
    src/report.cpp src/pins.cpp src/tool.cpp)

set(WARNINGS -Wall -Wextra -Wconversion -Wdouble-promotion -Werror)

if(CMAKE_SYSTEM_PROCESSOR STREQUAL "avr")
    # ---------------------------------------------------------------- firmware
    set(AVR_FLAGS -mmcu=${AVR_MCU} -DF_CPU=${AVR_F_CPU} -Os -flto -fno-exceptions -fno-rtti -fno-threadsafe-statics
                  -ffunction-sections -fdata-sections)
    set(CMAKE_CXX_EXTENSIONS ON)  # gnu++17: avr-libc uses GNU extensions

    add_executable(swiftpro-pp src/main.cpp src/i2c.cpp ${MODULES})
    set_target_properties(swiftpro-pp PROPERTIES SUFFIX ".elf")
    target_compile_options(swiftpro-pp PRIVATE ${AVR_FLAGS} ${WARNINGS})
    target_link_options(swiftpro-pp PRIVATE -mmcu=${AVR_MCU} -Os -flto -Wl,--gc-sections)
    target_link_libraries(swiftpro-pp PRIVATE m)

    set(ELF $<TARGET_FILE:swiftpro-pp>)
    set(OUT ${CMAKE_CURRENT_BINARY_DIR}/swiftpro-pp)
    add_custom_command(TARGET swiftpro-pp POST_BUILD
        COMMAND ${AVR_OBJCOPY} -O ihex -R .eeprom ${ELF} ${OUT}.hex
        COMMAND ${AVR_OBJCOPY} -O binary -R .eeprom ${ELF} ${OUT}.bin
        COMMAND ${CMAKE_COMMAND} -DIMAGE=${OUT}.bin -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/check_bang.cmake
        COMMAND ${AVR_SIZE} -C --mcu=${AVR_MCU} ${ELF}
        BYPRODUCTS ${OUT}.hex ${OUT}.bin
        COMMENT "swiftpro-pp.hex, the !!! check and the size report"
        VERBATIM)
else()
    # ---------------------------------------------------------------- host tests
    set(HOST_FLAGS -O1 -g)
    add_library(firmware_host STATIC ${MODULES} test/sim.cpp)
    target_compile_options(firmware_host PRIVATE ${HOST_FLAGS} ${WARNINGS})

    add_executable(unit
        test/unit.cpp test/unit_line.cpp test/unit_parse.cpp test/unit_serial.cpp test/unit_fmt.cpp
        test/unit_kin.cpp test/unit_motion.cpp test/unit_watch.cpp)
    target_compile_options(unit PRIVATE ${HOST_FLAGS} ${WARNINGS})
    target_link_libraries(unit PRIVATE firmware_host)

    add_executable(golden test/golden.cpp)
    target_compile_options(golden PRIVATE ${HOST_FLAGS} ${WARNINGS})
    target_link_libraries(golden PRIVATE firmware_host)

    enable_testing()
    # The unit tests read test/data/ relative to the source root.
    add_test(NAME unit COMMAND unit WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
    file(GLOB TRACES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/test/golden/*.txt)
    list(SORT TRACES)
    foreach(trace ${TRACES})
        get_filename_component(name ${trace} NAME_WE)
        add_test(NAME golden-${name} COMMAND golden ${trace})
    endforeach()
endif()
