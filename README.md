# SwiftPro++

A fresh, modern C++ firmware for the **uFactory uArm Swift Pro** robot arm (ATmega2560, 16 MHz).

This project replaces the [ufactory V3.X and V4.X Grbl-based firmware](https://www.ufactory.cc/download/).
This firmware speaks the same serial protocol, so existing host software like [libswiftpro](https://github.com/Smit-tay/libswiftpro), 
[SwiftRos2](https://github.com/Smit-tay/SwiftRos2), or, hopefully, any other SwiftPro software or scripts keep working.

Status: Extensive testing has been undertaken.  You can view (and control) a Real SwiftPro running this firmware [HERE](https://robot.smithjack.net/).

## Why

The original firmware works, but it carries a CNC controller's baggage and a number of bugs: 
- lines that overflow buffers
- replies that block the arm
- encoder faults reported as good readings
- open-loop motion that never notices a stalled joint
- timing that is off by 4 %. 

This firmware was written from scratch (MIT licence, see [ARCHITECTURE.md §1](docs/ARCHITECTURE.md)), 
using the original's [firmware source](https://github.com/uArm-Developer/SwiftProForArduino) **only** to learn its behaviour. 
It keeps the host contract (nearly 100%) and fixes or improves the rest:

- **Smoother, more accurate motion**: own joint-space planner with corner blending, jerk-limited (S-curve-like)
  profiles, linear moves cut by path error, steps rounded instead of truncated, exact final positions.
- **Closed-loop safety**: the encoders are read continuously; a joint that loses steps or collides stops the arm and is
  reported (`@8`).
- **Position streaming**: `@3` reports at up to 100 Hz with measured joint angles, timestamps and a sequence counter.
- **Optional streamed moves** (`M2124 V1`) so multi-point paths blend instead of stopping at every point.
- **Never blocks**: no busy-waits in the main loop; serial overflow is reported, never silent.
- Base buttons (`@4`), magnet health checks, per-port replies on all four UARTs, and more. The full list of changes,
  each with its evidence and decision, was kept in the project's issue record during development.

## Compared with the original firmware

The original in this case is a (big-fixed) uFactory Grbl-based Swift Pro firmware, (protocol V4.0.5). Measured figures are
marked; the rest come from reading the original's source.

### Size and memory

| | Original (V4.9.1-jss.7) | SwiftPro++ |
|---|---|---|
| Flash (program + initialised data) | 66,060 B (counted from the released hex) | 45,990 B (−30 %) |
| Static RAM (data + bss) | 5,365 B (measured from the linked image) | 4,088 B (−24 %) |
| Left for the stack, of 8,192 B | 2,826 B | 4,104 B |
| Stack actually used | at most about 1 KB (estimate from `-fstack-usage`) | 531 B (measured high-water mark, real image under simavr) |
| String constants | copied to RAM | kept in flash |

### Performance

| | Original | SwiftPro++ |
|---|---|---|
| Position report `@3` | the step tally only; no timestamp, no sequence number; skipped reports invisible | up to 100 Hz (measured on the arm: 239 reports in 2.4 s, none lost), plus one joint measured by its encoder per report, its age, a timestamp and a counter that shows any skipped report |
| Multi-point paths | the arm stops at every point (each move's reply waits for idle) | optional streaming (`M2124 V1`): moves blend through the corners (seen on the arm) |
| Acceleration | trapezoidal: acceleration jumps at every start and stop | jerk-limited: acceleration ramps over 40 ms; a stop still lands on the exact step |
| Linear moves (G1 family) | cut into segments by a rule of length and speed, not of path error; F silently clamped to 100; each move rewrites the acceleration for all later ones | cut by path error (at most 0.02° off the straight line); F as given; acceleration is a separate, saved setting (`M204`) |
| Kinematics | base angle undefined behind the base; steps truncated (up to one step of bias) | defined everywhere; steps rounded. Inverse kinematics round trip 0.06 µm against 5.9 µm (measured on the AVR's float under simavr) |
| Encoder reads | blocking; a three-joint read stops the main loop about 10–12 ms (from the code's delays) | never blocks: one transaction per main-loop pass, 3.6 ms per joint (measured under simavr) |
| Lost steps and collisions | never detected: the arm carries on from a position it isn't at | encoders watched during every move; a joint more than 1° off stops the arm and is reported (`@8`); on the arm, with the base held by hand, it was caught 0.21 s after the move began |
| Time base | 0.96 ms "1 ms" tick, counters one tick late | exact 1 ms |
| Serial input | one 128-byte ring shared by four UARTs; overflow silently splices lines | a 256-byte ring for USB and one per UART; overflow reported (`E23`) |
| Blocking waits | effector origin search up to about 10 s with serial ignored; pump valve 1 s; limit-switch debounce 10 ms; EEPROM writes with interrupts off 3.4 ms per byte | all run in the background; EEPROM writes block interrupts for 4 cycles |
| Stepper interrupt | re-enables interrupts inside itself | never nests; 9 µs per step event (measured under simavr) |

### Bugs fixed and improvements

Faults in the original that this firmware fixes:

- **Crashes and lock-ups**
  - Unbounded `sscanf` into small stack buffers. Lines of 51–79 characters overflowed the 50-byte command buffers and
    could corrupt the planner. `M2205` overflowed a stack buffer.
  - `G2004 P0` wedged the command queue until power-off.
  - `S1100` during a move left Grbl in ALARM: later commands were accepted but never run or answered.
  - More than 20 unanswered commands overwrote a reply that hadn't been sent yet.
- **Wrong or missing replies**
  - Unknown P and S commands, and commands that failed to parse, replied `ok`. Some Grbl status codes produced a
    reply with neither `ok` nor an error code.
  - `E23` had no sequence number, and the tail of an over-long line was executed as a new command.
  - Debug text was sent on the command channel.
- **Encoders**
  - A failed encoder read was silent and gave a plausible wrong angle.
  - The health check caught only a dead bus, never a missing or weak magnet.
  - Encoder values could be torn between their two bytes.
  - The `@3` report read the step counts without protection from the stepper interrupt.
- **Motion**
  - A linear move that ended out of reach ran part of the way before returning `E22`. Now nothing moves.
  - Commands ran as soon as they were dequeued, not after the previous move. `M17`/`M2201` could wipe the motion
    state mid-move.
  - Per-joint lock and unlock were wrong in four ways.
  - Omitted axes used a mixed-unit position.
  - Zero or negative F was accepted.
  - With the configured acceleration, early moves could demand more steps per second than the stepper interrupt
    could produce.
- **Safety**
  - A `#N` line that wasn't a uArm command went to Grbl, where bare axis words moved the arm.
  - Grbl leftovers were reachable from the serial port: `G28`/`G30` moved the arm to stored coordinates, arcs
    ignored errors, `G4` blocked the main loop, `M3`/`M4` broke the wrist's timer, `$` commands rewrote settings, and
    a stray `@` byte entered Grbl's safety-door state and withheld every reply. The automatic position correction
    (`M2123`) moved the arm on its own. All of these are removed.
  - Raw pin commands could drive the step, enable and serial pins. They now refuse pins the firmware uses.
- **Peripherals**
  - The wrist resolution was 1.76° and the angle reported was the one requested, not the one output. It is now 0.05°,
    and the reported angle is the real output.
  - Out-of-range wrist angles wrapped around.
  - The buzzer duration was right only at 1 kHz.
  - `M2400` switched the wrist off and left it off.
  - Reading pin 6 or 44 stopped the wrist or the buzzer.
  - In modes 4/5, the laser drove the effector stepper; a failed origin search went unnoticed; effector steps drifted.
- **Calibration**
  - `M2401` changed the encoder reference but not the position.
- **Housekeeping**
  - Interrupts were enabled by accident, inside an EEPROM write.
  - Grbl settings were rewritten to EEPROM on every boot.
  - Read-modify-write of I/O ports shared with interrupts was not protected.
  - Each reply went to whichever UART had received the last byte.

New features:

- **Base buttons:** the Menu and Play buttons report `@4` events.
- **Encoder detail query:** `P2246` returns each encoder's magnet status, gain and field strength.
- **Streaming replies:** with `M2124 V1`, a move replies as soon as it is accepted.
- **Lost-step reporting:** `@8`, switchable with `M2125`.
- **Boot recovery:** a failed boot position read recovers on its own.

Not addressed:

- **Reset cause:** it can't be reported, because the stock bootloader clears it.
- **Brown-out level:** it stays at 2.7 V. Changing it is a fuse setting, not firmware.

## Hardware

uArm Swift Pro, main board V3.3.x, ATmega2560 with the stock Arduino stk500v2 bootloader. Three steppers (base, left
and right arm motors, 1/32 microstepping, 80 steps per degree), three AS5600 magnetic encoders, a wrist servo, pump,
gripper, laser, limit switch and buzzer. Every pin and constant used is taken from the original source and cited:
[ARCHITECTURE.md §2](docs/ARCHITECTURE.md).

## Building

Requirements: CMake ≥ 3.20, a C++17 host compiler (g++ or clang++), and avr-gcc with avr-libc (tested with avr-gcc
15.2.0).

Host tests (unit tests and the golden protocol traces, against the simulator):

```sh
cmake -B build/host
cmake --build build/host
ctest --test-dir build/host --output-on-failure
```

Firmware:

```sh
cmake -B build/avr --toolchain cmake/avr-gcc.cmake
cmake --build build/avr
```

This produces `build/avr/swiftpro-pp.elf` and `build/avr/swiftpro-pp.hex`, prints the flash and RAM use, and fails if
the image contains `!!!` (the bootloader would break the upload). For another avr-gcc install, add
`-DAVR_TOOLCHAIN_PREFIX=/path/to/bin/`.

For clangd, `python3 tools/compdb.py` (run from the repository root) writes a `compile_commands.json` that treats
`src/` as AVR code and `test/` as host code.

## Flashing

Through the USB port and the board's bootloader, with avrdude:

```sh
avrdude -c wiring -p m2560 -P /dev/ttyACM0 -b 115200 -D -U flash:w:build/avr/swiftpro-pp.hex:i
```

Adjust `-P` to your port. `-D` leaves the erase to the bootloader, as Arduino does. To go back, flash a build of the
original firmware the same way. Don't flash a raw dump of the whole flash through the bootloader: it includes the
bootloader's own section.

**For the first run**, follow [HARDWARE-TEST.md](docs/HARDWARE-TEST.md): queries with motor power off first, then small,
slow single-joint moves.

## Usage

115200 baud, 8N1, lines ending in `\n`. Commands are the uArm protocol V4.0.5, with an optional `#N` sequence number
that is echoed in the reply:

```
> #1 P2203
< $1 ok V5.0.0-pp.1
> #2 G0 X200 Y0 Z100 F100
< $2 ok
> #3 M2120 V0.05
< $3 ok
< @3 X200.0000 Y0.0000 Z100.0000 R89.1000 B0.0000 L… AR… MB0.0000 A4 T15230 S0    (abbreviated)
```

The complete command reference, what is kept, changed, added and dropped, is in [PROTOCOL.md](docs/PROTOCOL.md).

## Documentation

| Document | Contents |
|---|---|
| [DESIGN.md](docs/DESIGN.md) | How the firmware works: modules, main loop, protocol engine, motion pipeline, encoders, concurrency, memory, testing. Start here to work on the code |
| [PROTOCOL.md](docs/PROTOCOL.md) | The serial protocol, command by command, with the original's behaviour and this firmware's |
| [ARCHITECTURE.md](docs/ARCHITECTURE.md) | Hardware map with source citations, timers, EEPROM layout, RAM and stack budget, milestone measurements, design records |
| [HARDWARE-TEST.md](docs/HARDWARE-TEST.md) | Step-by-step plan for the first runs on the arm, and the results |

## Status

- Built: every command in [PROTOCOL.md](docs/PROTOCOL.md) that isn't listed as dropped:
  - protocol engine, identity and settings, encoder queries, kinematics;
  - motion: all move types, hold, stop;
  - effector: wrist, pump, gripper, laser; limit switch, power sense, buzzer;
  - `@3` reports, base buttons, lost-step detection;
  - pin commands, work modes and offsets, encoder reference store, UID;
  - the stepper end effector of modes 4/5.
- Tested on the host (unit tests, 17 golden traces) and on the real firmware image under simavr
  (`tools/simavr/run.sh <simavr source dir>`).
- First bring-up on the arm passed on 2026-10-18: read-only checks, slow and fast moves, hold and stop, linear and
  streamed paths, wrist, pump, release and re-lock, lost-step detection
  ([results](docs/HARDWARE-TEST.md#results)). Not yet tried on hardware: the base
  buttons, the gripper, the laser, the limit switch and the stepper end effector.
- Flash about 46 KB of 248 KB; static RAM 4.1 KB of 8 KB, plus about 530 B of stack (measured).

## Licence

MIT, see [LICENSE](LICENSE). Copyright (c) 2026 Jack Sidman Smith.
