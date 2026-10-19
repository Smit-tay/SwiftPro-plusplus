# ARCHITECTURE — SwiftPro-plusplus

Citations are `file:line` in [Smit-tay/SwiftProForArduino](https://github.com/Smit-tay/SwiftProForArduino) `src/` at commit `72a2ccf`. 
VERIFIED = read in code or measured;
UNVERIFIED = inferred. kept outside this repository.

## 0. Toolchain (VERIFIED, 2026-10-17)

| Tool | Version |
|---|---|
| avr-gcc / avr-g++ | 15.2.0 (Fedora 15.2.0-1.fc44) |
| avr-libc | 2.2.1 |
| host g++ | 16.2.1 |
| clangd | present (`/usr/bin/clangd`) |
| avrdude | present. use for flash. |
| PlatformIO | present; used only to build a scratch copy of the original for measurement |
| simavr | not packaged for Fedora; built from source (github.com/buserror/simavr, commit 5b08da6) in the scratch directory, not installed (your OK, 2026-10-18). Host-side only |

The original builds with PlatformIO's avr-gcc 7.3.0. Its output is **byte-identical to the bug-corrected uArm SwiftPro firmware
`uArmPro_V4.9.1-jss.7` hex** (VERIFIED), so every measurement below describes the current firmware.
Original size (VERIFIED): flash 66,060 B; static RAM 5,365 B (data 780 + bss 4,585); 2,826 B left for the stack.

avr-g++ ships no libstdc++ headers. That suits the no-STL rule: `constexpr`, `static_assert`, `enum class`, templates
and RAII all work. `<type_traits>` and friends are not available,  reimplemented in this code.

Flags: `-mmcu=atmega2560 -DF_CPU=16000000UL -std=gnu++17 -Os -flto -fno-exceptions -fno-rtti  -fstack-usage 
-fno-threadsafe-statics -ffunction-sections -fdata-sections -Wl,--gc-sections -Wall -Wextra -Wconversion -Werror`.

Build with CMake (`CMakeLists.txt`, AVR toolchain file `cmake/avr-gcc.cmake`; host
tests when configured natively). No Arduino core: the few pins we touch are addressed directly through a `constexpr` pin table.

After linking, the build **fails if the flash image contains the bytes `!!!`**  The
stk500v2 bootloader on the Robot may switch to its serial monitor on that sequence during an upload.

## 1. A fresh implementation, not a port

SwiftPro-plusplus is a new firmware under the MIT licence. The original (GPL: `README.md:129-131`; Grbl's GPLv3 headers
in 36 files) is read **only to learn its externally visible behaviour**, which is poorly documented. No code is reused,
including line-by-line translation: every module is written from a description of the behaviour, and tests check that
behaviour against the original (measured replies, simavr runs of the original, cited source lines). Citations in
comments document what the original does; they don't mean the code was carried over.

Consequences:
- Planner and stepper are own design, not Grbl's, or uFactory's. They must reproduce the original's motion behaviour:
  joint-space vs Cartesian moves, F and acceleration semantics, final step counts and the acceptance numbers, not its internal algorithms.
- Float results that come from formulas (kinematics, number parsing) are computed by our own code. Where the
  original's exact bits can't be reproduced without copying its expression order, the port matches it within a
  stated tolerance, and every difference that could show in a 4-decimal reply is reported.
- What is dropped is unchanged: spindle, coolant, probe, homing, limits, control pins, safety door, check mode, arcs,
  dwell, coordinate systems, G28/G30/G92, startup lines, `$` commands and settings, realtime `? ! ~ @`, Grbl messages, the Arduino core, CoreXY and other machine variants.

## 2. Hardware map (VERIFIED, extracted from the original; nothing invented)

### 2.1 Arm axes (Grbl X/Y/Z = L/R/base)

| Joint | STEP | DIR | ENABLE (high = enabled, see note) | Source |
|---|---|---|---|---|
| L (ARML, X) | PF6 | PF7 | PF2 | `cpu_map/cpu_map_atmega2560.h:145-186` |
| R (ARMR, Y) | PL3 | PL1 | PK0 | `:152-191` |
| Base (Z) | PF0 | PF1 | PD7 | `:159-196` |
| Micro-step | MS1 = PK2 = 1, MS2 = PK3 = 0, MS3 = PK4 = 1 → "1/32 step" | | | `:201-212`, `stepper.c` `stepper_init` |

- **Enable polarity:** `$4` (`BITFLAG_INVERT_ST_ENABLE`) is forced to 1 at every boot (`uarm_peripheral.c:629-650`),
  so drivers are enabled by driving the pin **high**. Which driver chip this is: UNVERIFIED. The port reproduces the
  pin levels, not a guess about the chip.
- **Direction invert mask** (`$3`), forced at boot from the hardware-version straps PJ3–PJ6
  (`uarm_peripheral.c:629-650`): V3.3.0 (straps 0000) → 7 (all inverted); V3.3.1/V3.3.2/unknown → 0.
- **Step pulse:** `$0` = 10 µs setting → Timer0 reload `-(((10-2)*16)>>3)` = 16 ticks of 0.5 µs, i.e. **8 µs high**
  (`stepper.c` `st_wake_up`). Idle lock `$1` = 255 (drivers stay enabled).
- **Kinematics constants** (`uarm_coord_convert.c:45-63`): ARM_A 142.07 mm, ARM_B 158.81 mm, Z_BASIC 107.4 mm,
  center offset 13.2 mm, 4.5:1 gearbox, 32 µsteps, so **80 steps/°**. Limits are in PROTOCOL.md §5.
- **Planner scale:** `steps_per_mm` = 6400 on all axes (`defaults/defaults_generic.h:31-33`), so 1 planner unit = 80°.
  max_rate 100000, accel 10·3600 units/min², junction deviation 0.05 (lines 34-48).

### 2.2 Encoders: three AS5600, each on its own bit-banged bus

| Joint | SDA | SCL | Source |
|---|---|---|---|
| L | PC1 | PC3 | `uarm_iic.c:4-12` |
| R | PH2 | PH7 | `:20-28` |
| Base | PD5 | PD6 | `:36-44` |
| (external user EEPROM 0xA0) | PB7 | PB6 | `:53-62` (drop, Q-18) |

Address 0x36. ANGLE register 0x0E/0x0F, read as two separate single-byte transactions. STATUS register 0x0B
(`uarm_angle.c:60-88`). Bit time about 15 µs (~65 kHz), using `delay_us` (`uarm_iic.c:250-730`). Angle =
`pointB − wrap(ref − raw)·360/4096`, with ref from EEPROM 820 (flag 818 = 0xBB, else 800) and
pointB = L 32.758930, R 88.980537, base 90.0 (`uarm_angle.c:33-41,136-232`).

### 2.3 Everything outside the three arm axes

| Feature | Pins | Timer / ISR | Commands | Host uses? | Proposal |
|---|---|---|---|---|---|
| Wrist servo (G2202 N3) | PH3 = OC4A | Timer4 PFC PWM, ICR4 = 20000, ÷8, 50 Hz; no ISR | G2202 N3, P2206 N3, M2201/M2202 N3, M17, M2019, `@3 R` | **yes** (set_wrist, set_servo_angle, reset) | keep; Timer4 is the wrist's alone; |
| Pump + valve | PG4 (pump), PG5 (valve); sense ADC A9 = PK1 | none (pump_tick in main loop; `delay_ms(50)` on V3.3.0) | M2231, P2231 | **yes** | keep; non-blocking valve timing  |
| Gripper (two-finger, vision kit) | PC7 out (low = catch); PA0 in (grip sense) | none | M2232, P2232 | **yes** | keep |
| Laser | PH5 + PH6 out | none | M2233, P2235 | no | keep (trivial) or drop; mutually exclusive with the stepper effector |
| Buzzer | PL5 | Timer2 OVF ISR toggling PL5 at 2·F; callback pointer | M2210, boot beep | yes (set_buzzer) | Timer5 CTC hardware toggle on OC5C/PL5, **no ISR** (PL5 = OC5C per the ATmega2560 pin table: to re-verify against the datasheet before coding). Duration from the 1 ms tick |
| Effector limit switch | PG3 in, pull-up | none (polled, 10 ms blocking debounce) | `@6`, P2233 | yes (on_limit_switch) | keep; non-blocking debounce |
| Motor power sense | ADC A5 = PF5 | none (polled every 256 loops) | `@5`, P2234 | yes (on_power) | keep; poll on the tick (e.g. every 50 ms) |
| Hardware version straps | PJ3–PJ6 in | none | P2202; selects `$3` and the pump valve mode | yes (device info) | keep |
| Stepper end effector (work modes 4/5) | DIR PH5, STEP PH6, origin switch PK5 (pull-up) | Timer4 OVF ISR every 192 µs (time4 reload 3×64 µs) | M2400 S4/S5, G2202 N3, M2412, P2206 N3 | no | keep (Q-14): **Timer3** CTC ISR + non-blocking origin state machine; untested on hardware |
| Work modes 0–8 | n/a | n/a | M2400, P2400 | yes (set_mode) | keep the offsets table (`uarm_common.h:115-139`); 4/5 per Q-14 |
| Raw IO | any Arduino pin 0–69 | n/a | M2240, M2241, P2240, P2241 | yes (ROS services) | allowlist  |
| USART1–3 | PD2/PD3, PH0/PH1, PJ0/PJ1 | RX/UDRE ISRs | all commands | Bluetooth (which USART: UNVERIFIED) | keep all; RX pull-ups on (Q-16) |
| LED effects, buttons, camera | **not present in this firmware** (VERIFIED by grep) | | | | none |

The "nine work modes" are the `uarm_work_mode_e` enum 0–8 (`uarm_common.h:50-60`). Each sets the front and height
offsets of the effector; only modes 4/5 change hardware behaviour (they select the stepper effector).

## 3. Timers and interrupts

### 3.1 Original (VERIFIED from the linked vector table)

| Timer | Owner | ISR (vector) | Rate | Stack (approx.) |
|---|---|---|---|---|
| T0 | Grbl step-pulse reset | TIMER0_OVF (23) | one-shot per step, about 8 µs after the step | small (~10 B) |
| T1 | Grbl stepper | TIMER1_COMPA (17), **re-enables interrupts** | up to ~30 kHz | 21 B frame + 3 B return + nesting |
| T2 | buzzer | TIMER2_OVF (15), calls a function pointer | 2·F while beeping | ~30 B (full call-clobbered save) |
| T3 | unused (`time3_*` never called) | TIMER3_OVF (35) linked, never enabled | n/a | n/a |
| T4 | wrist PWM / stepper effector / Grbl spindle  | TIMER4_OVF (45) via function pointer | 5.2 kHz in modes 4/5 | ~30 B |
| T5 | 1 ms tick (really 0.96 ms) | TIMER5_OVF (50) via function pointer | ~1 kHz while G2004 or M2120 is active | ~30 B |
| — | USART0–3 RX/UDRE | 25/26, 36/37, 51/52, 54/55 | per byte | ~15 B |
| — | PCINT0 (limits, disabled), PCINT2 (control, on PK4!) | 9, 11 | dormant | small |

### 3.2 Proposed

| Timer | Owner | Mode | ISR | Rate | Worst-case ISR stack |
|---|---|---|---|---|---|
| T0 | system tick | CTC, ÷64, OCR0A = 249, exactly 1.000 ms  | TIMER0_COMPA: increments a `uint32_t` ms counter | 1 kHz | ≤ 16 B |
| T1 | stepper | CTC, OCR1A, prescaler per AMASS as in Grbl | TIMER1_COMPA: Bresenham, **no `sei()`** | ≤ ~30 kHz (worst case computed in §6) | ≤ 40 B (measured later with `-fstack-usage` + paint) |
| T2 | free | n/a | none | n/a | n/a |
| T3 | stepper effector (modes 4/5, approved Q-14; untested on hardware) | CTC | TIMER3_COMPA toggles PH6 | ~2.6 kHz while running | ≤ 16 B |
| T4 | wrist servo only | PFC PWM, ICR4 = 20000, ÷8 | none | n/a | n/a |
| T5 | buzzer | CTC, COM5C0 toggle on OC5C/PL5 | none | n/a | n/a |
| USART0–3 | serial (USART1–3 kept for Bluetooth, Q-16; shared rings, faithful routing) | RX complete, UDRE | yes | per byte (≤ 11.5 kHz each) | ≤ 20 B |

**Rules.** No ISR re-enables interrupts, so nesting depth is 1. AVR picks the lowest pending vector number first:
TIMER1_COMPA (17) < TIMER0_COMPA (21) < USART0_RX (25), so the stepper wins arbitration. The stepper's worst-case
latency is therefore the longest other ISR body (USART RX, a few µs) plus the longest `cli` section in main
context (atomic copies, ≤ 2 µs, and EEPROM `EEMPE`/`EEPE` ≤ 1 µs). **No peripheral feature runs
inside, or blocks, the stepper ISR.** None of the peripheral timers needs CPU time except T0 (tiny) and T3
(effector, optional).

Worst-case stack, including ISRs: main-context deepest chain (estimated ≤ 600 B, to be measured) + one ISR
(≤ 40 B + 3 B return). Budget **≤ 1 KB**, enforced by `-fstack-usage` review in every milestone report and by a
stack-paint high-water check (§11).

## 4. Module layout (one header plus one source per module unless noted)

```
src/
  hw.h            constexpr pin/port table, timer constants (citations to the original in comments)
  atomic.h        RAII interrupt-disable guard (saves/restores SREG)
  serial.*        UART0 RX/TX rings, overflow flag, non-blocking write with "room" query
  line.*          line assembly: whitespace/comment strip, upper-case, 79-char limit       [host-tested]
  parse.*         "#N" prefix, family/number, bounded number fields, per-command grammars  [host-tested]
  fmt.*           dtostrf-compatible float formatting, int formatting                        [host-tested]
  kin.*           IK/FK, effector offsets, legality, steps<->angle (float, as original)      [host-tested]
  planner.*       move queue, corner speeds, entry-speed passes (own design)                [host-tested]
  stepgen.*       5 ms segments from the plan, 40 ms moving average                        [host-tested]
  stepper.*       Timer1 ISR, pins, tally                                                   [host-tested]
  motion.*        joint and linear moves, line checking and cutting, lock state, M204       [golden traces]
  i2c.*           bit-banged I2C master x3, open-drain SDA ; AVR only (the simulator models the AS5600)
  encoder.*       AS5600 jobs, one bus step per pass: median-of-3 angle, health 
  effector.*      wrist servo, pump/valve state machine, gripper, laser, (stepper effector)
  buzzer.*        Timer5 OC5C tone + duration
  tick.*          Timer0 1 ms tick, software timers
  nvm.*           internal EEPROM: the uArm parameter layout (§10), safe writes 
  protocol.*      routing, sync queue, reply composition (P/S run at once: no async slot)  [golden traces]
  commands.*      command handlers and the machine state the protocol schedules against
  board.*         hardware version straps (P2202), direction mask
  app.*           init + one main-loop pass; driven by main.cpp on the AVR and by test/sim.cpp on the host
  report.*        @1/@3/@5/@6/@9 producers
  main.cpp        init order, main loop
test/             host g++ unit tests, golden traces (test/golden/*.txt) and their runner (Phase 1)
```

Templates only where they remove real duplication, e.g. a single `BitBangI2c<SdaPort, SdaBit, SclPort, SclBit>`
instead of the original's three copy-pasted drivers. No class hierarchies, no virtual functions.

## 5. Execution model (built, module 4)

```
USART0-3 RX ISRs ─► one RX ring per port ─► line assembler per port ─► router
                                                             ├─ P/S  → run now → reply now (or async pending: encoder query)
                                                             └─ G/M  → sync queue (20) ─► one starts per pass
                                                                                            (sync pending: re-sync, line check, G2004)
   motion: joint move → 1 planner block;  linear move → checked piece by piece, then cut into blocks 
   planner (16 blocks, own design) ─► stepgen (5 ms segments, 40 ms moving average) ─► stepper ring (8) ─► Timer1 ISR
```

Main loop pass (`app::poll` → `protocol::poll`): one encoder bus step, one linear-move piece, refill the stepper ring,
complete finished pending commands, take at most one line, send ready replies and `@9`, start the next sync command.
Measured on the AVR (simavr, `tools/profile/run.sh`): IK 1.26 ms, FK 0.88 ms, `planner::add` with 15 queued 1.7 ms,
`stepgen::next` 1.2 ms per 5 ms segment, `dtostrf` 0.15 ms. The longest pass (a line piece: 2 IK + a planner add) is
about 4.5 ms, well inside the 40 ms of queued segments and the 256-byte RX ring (22 ms at 115200).

## 6. Stepper ISR (built, module 4)

Timer1 CTC at 2 MHz. Each 5 ms segment (≤ 160 steps per joint) is played as evenly spaced events (the period and its
remainder are computed when the segment is queued); joints step in proportion (Bresenham). Each event ends the
previous pulse and starts its own, so a pulse lasts until the next event (≥ 31 µs) and the low time between pulses
is the ISR's own work. Q-23's 8 µs busy-wait was replaced for CPU load (it cost a quarter of the CPU at full speed).
The tally is committed per segment; `position()` adds the steps already made in the current segment from the
Bresenham state, so it is exact at any moment. Measured (simavr): 9 µs per event, 17 µs for an event that starts a
segment. At the per-joint speed cap of 300 °/s (24 kHz) that is under 30 % of the CPU. No ISR re-enables interrupts.

## 7. Moves (built, module 4)

G0, G2202, G2206: one straight move in joint space (as the original). G1, G2201, G2204, G2205: a straight effector
line, cut into pieces whose joint-space chord stays within 0.02° of the true path (R-4), each piece end solved by IK;
the whole line is checked before anything moves . Planner: per-joint acceleration (M204, default A5 = 400 °/s²,
Q-52) and speed cap (300 °/s), corner speed from a tolerance circle (0.2° of joint space), backward/forward passes.
stepgen: a live accelerate/cruise/decelerate profile per 5 ms segment, smoothed by a 40 ms moving average in exact
integers (jerk-limited, R-5); a move that starts from rest waits for the average to settle, so full stops are exact.

## 8. Numeric representation

| Option | ISR | Main (prep, planner, IK) | Notes |
|---|---|---|---|
| **1 (recommended, faithful)** | integer only (Bresenham counters, `uint32_t`) | `float` (avr `double` = `float`) | Proven Grbl split; float ops never run in an ISR |
| 2 | integer | fixed-point everywhere | Large effort, accuracy risk in acos/atan, no real need: IK costs ~1–2 ms per segment on AVR and the planner is far ahead |

All kinematics code uses `float` explicitly, so the host tests (g++, 32-bit `float` with SSE) compute in the same
precision. avr-libc and glibc `acos`/`atan`/`sqrt` can differ in the last ulp, so host golden traces compare
angles/coordinates with a tolerance (≤ 1e-4) plus exact `dtostrf` formatting of the given value. Exact on-target bytes
need simavr.

## 9. Encoders and the `@3` report (Q-1; built, module 5b)

Built: the encoders are read in small steps from the main loop (one bus step per pass). Queries go first, then
re-syncs, and whenever the bus is free the lost-step watch reads the joints in turn, B, L, R (R-12). The watch's
reading has three ANGLE samples plus STATUS. Each reading is kept with its time for `@3` (one measured joint per
report, Q-33) and is checked against the tally range covered during the read. The rest of this section is
the design record.

Driver changes: one transaction per sample (pointer write, repeated start, 2-byte burst) at about
5 µs per bit-time. That is ~45 bit-times, **≈ 0.25 ms per sample**, against ~1.1 ms now. Median-of-3 kept (faithful),
giving ≈ 0.75 ms per joint and ≈ 2.3 ms for three joints (now ~10–12.6 ms). Preemption by ISRs only stretches the
clock, which an I2C master may do. NACK is reported.

`@3` options:

| Option | Content | Cost at 30 Hz | Host impact |
|---|---|---|---|
| a (faithful) | tally (what the planner believes) | none | none |
| b | measured angles replace tally | ~2.3 ms × 30 = 7 % CPU, spread over iterations | X/Y/Z/B/L/AR become measured. Readings lag and include gear backlash; probably what you want for ROS state |
| **c (approved, Q-1)** | tally fields unchanged + 3 measured joints appended, e.g. `… AR%s MB%s ML%s MR%s` | as b | libswiftpro uses tokens 0–6 and ignores extras (`receiver.cc` `parse_at3`), so existing hosts are unaffected; the ROS driver can adopt the new fields later |

Atomic snapshot of the tally in every option.

## 10. Persistent data (internal EEPROM, VERIFIED layout; the port reads and writes the same addresses)

| Address | Size | Content | Source |
|---|---|---|---|
| 800 | 6 | old encoder reference (B, L, R `uint16`), used if flag ≠ 0xBB | `uarm_common.h:33`, `uarm_angle.c` `read_angle_reference_param` |
| 818 | 1 | 0xBB = use 820 | `uarm_common.h:34` |
| 820 | 6 | encoder reference B, L, R (`uint16`, little-endian, struct order b, l, r) | `uarm_common.h:32`, `uarm_angle.c:7-11` |
| 900 | 1 | work mode (low byte of the enum) | `uarm_common.h:38`, `uarm_peripheral.c:327-340` |
| 910 | 4 | height offset (`float`) | `uarm_common.h:39` |
| 920 | 4 | front offset (`float`) | `uarm_common.h:40` |
| 930 | 4 | user height offset | `uarm_common.h:41` |
| 940 | 4 | user front offset | `uarm_common.h:42` |
| 1002 | 12 | UID/BLE MAC text (P2205) | `uarm_common.h:35` |
| 3202 | 4 | effector angle offset (`float`, >360 or NaN → 0) | `uarm_common.h:44` |
| 0, 2049… | | Grbl settings, coordinates, startup lines | `settings.h` (**not used** by the port; left untouched) |
| 1–453 | 452 | UARM_2500 calibration maps | `uarm_angle.c` (**not used**; left untouched) |

Settings that were EEPROM-backed Grbl values become `constexpr`: `$0`=10, `$1`=255, `$2`=0, `$3` from straps,
`$4`=1, `$11`=0.05, steps 6400, max_rate 100000, acceleration **50**·3600   **Note:** if you ever changed
`$` settings on your arm, the original uses your EEPROM values, not these defaults. Please run `$$` on the original
once and verify output (it is safe, and prints only).

## 11. RAM budget (static, estimates; real numbers in every milestone report)

| Item | Original (VERIFIED) | Proposed |
|---|---|---|
| Planner blocks (24) | 984 | ~1,000 |
| Segment ring + stepper block data + prep + ISR state | 36 + 85 + 43 + 35 = 199 | ~200 |
| RX / TX rings | 128 / 128 | 256 / 256 |
| Line assembly | 80 | 80 |
| Sync queue (20) + async slot | 2,040 + 102 | 20 × (80 line + 48 reply + 6) + 1 slot ≈ 2,800. Faithful 79-char lines and full reply text; could shrink to ~1.4 KB if lines are capped at 40 characters (Q-22) |
| Grbl `sys`/`gc_state`/`gc_block`/`settings` | 30 + 63 + 57 + 77 | ~40 (only what the port uses) |
| Calibration maps (unused) | 452 | 0 |
| uArm state, encoder refs, misc | ~300 | ~250 |
| **Total static** | **5,365** | **≈ 4,900** |
| **Left for stack** | 2,826 | ≈ 3,290 |
| Stack budget (main + 1 ISR) | ~0.7–1 KB estimated | ≤ 1 KB, measured |

**Milestones (measured, avr-gcc 15.2.0; `make avr` up to 5b, `cmake --build build/avr` since):**

| Milestone | Flash | Static RAM | Worst-case stack | Notes |
|---|---|---|---|---|
| 1. serial + line + parse (2026-10-18) | 3,798 B | 526 B | ≈ 165 B | RX/TX rings 2 × 256 B. Stack: harness `main` 89 + `parse::command` 18 + `read_float` 17 + 3 return addresses × 3 B ≈ 133 B, plus one ISR (RX: 19 + `on_rx` 4 + `rx_push` 3 + returns ≈ 35 B). ISRs don't nest: none enables interrupts. The harness is not firmware (no hex) |
| 2. fmt + protocol + identity commands + simulator (2026-10-18) | 6,014 B | 2,066 B | ≈ 290 B | Sync queue 20 × 64 B = 1,280 B; rings 512 B; string literals 190 B in `.data` (can move to flash if RAM gets tight). Stack: `protocol::poll` 143 + `emit` 78 + `serial::write` 9 + returns ≈ 255 B, plus one ISR ≈ 35 B. `make avr` builds the real `main.cpp` now, still as an ELF only: no motor pins are driven yet |
| 3. kinematics + encoders (2026-10-18) | 12,952 B | 2,278 B | ≈ 280 B | Flash grew with avr-libc `dtostrf` and libm (float replies are real now). `kin` (rewritten from the geometry, §1) matches the original on the AVR within 0.0024 deg; 3.3 % of values differ in the 4th printed decimal (tools/kin, simavr, 26,436 values). Stack: `poll` 87 + `commands::resume` 39 + `fmt::f4` 57 + `dtostrf` internals (≈ 40, UNVERIFIED) + returns ≈ 240 B, plus one ISR ≈ 35 B. Encoder reads: one bus step (≤ ≈ 0.25 ms) per main-loop pass |
| 4. planner + stepgen + stepper + motion commands (2026-10-18) | 35,036 B | 3,814 B | ≈ 400 B | Planner 16 blocks (≈ 700 B). Stack: `protocol::poll` 74 → `commands`/`motion::poll` → `next_piece` 101 → IK ≈ 45 → libm ≈ 40 (UNVERIFIED) + returns ≈ 350 B, plus one ISR ≈ 50 B. ISR 9 µs/event (measured) |
| 5a. wrist, pump, gripper, laser, limit, power, ADC, buzzer (2026-10-18) | 38,026 B | 3,872 B | ≈ 400 B | No new deep call chains |
| 5b. `@3` report, buttons, lost-step watch, M2125 (2026-10-18) | 42,504 B | 4,000 B | ≈ 460 B (est.) | Deepest chain now the report: `protocol::poll` 74 → `send_report` 171 (its 160 B line buffer; not inlined, so it stays off the motion chain) → `report::write` 55 → `fmt::f4` 57 → `dtostrf` ≈ 40 (UNVERIFIED) + returns ≈ 410 B, plus one ISR ≈ 50 B (`-fstack-usage`, no LTO). Report formatting 3.15 ms (measured, simavr) |
| simavr full-image test (2026-10-18) | 41,890 B | 3,992 B | **528 B measured** | `tools/simavr`: the real ELF with emulated AS5600s on the encoder pins, UART, step/dir counting, stack painted from `__bss_end`. 14.4 s session: boot, identity, encoder queries, joint, Cartesian and linear moves (exact step counts), `@3`, `S1100`, lost steps, buzzer, wrist, release. High-water mark 528 B (the estimate was ≈ 460 B). R-16 applied |
| 5c. pins, work modes and offsets, M2401, M2205, stepper end effector (2026-10-18) | 45,580 B | 4,088 B | **527 B measured** | Effector stepper on Timer3 (CTC clk/64, OCR3A 47: 192 µs half steps, as the original). Mode offset table and owned-pin list in flash. The simavr session covers pins and the effector (exact step counts) |

Stack measurement: `-fstack-usage` per function plus a manual call-graph sum for the deepest chains, and a paint
pattern from the end of `.bss` to SP at boot. A debug build reports the high-water mark (a debug-only `P` command,
needing approval, or read with a debugger: your choice).

## 12. Testing

**Golden traces (Phase 1, built).** `ctest` (host build) runs `test/golden.cpp` on every
`test/golden/*.txt`, one test per trace. A trace sets simulated inputs (`= key value`, keys in `test/sim.h`), boots, sends lines
(`>`), advances simulated time (`~ ms`, `~ idle`) and states the exact output lines (`<`, or `<re` for fields that
vary, such as timestamps). Every expectation carries a provenance tag: `faithful <file:line>`, `measured <log> <id>`. 
The runner lints every trace; the firmware's host build will provide `sim_*` . Until then `test/sim_stub.cpp` is linked and every trace
reports PENDING.

**Change from the Phase 0 plan:** the plan said the traces would be generated by compiling the original's protocol
code on the host. They are written by hand instead, from the cited source lines plus the values measured on your
arm. The original's protocol code is bound to Grbl's globals and to AVR registers throughout, and stubbing all of
that would itself be unverified. The host-compiled original is still used where it is tractable: the kinematics
(`uarm_coord_convert.c`) as the oracle for IK/FK unit tests in Phase 2.

**Float formatting (VERIFIED under simavr, 2026-10-18).** `dtostrf(v, 5, 4)` was run under simavr on 5,125 float32
values: random values across the arm's ranges, values within 1 ulp of every kind of rounding boundary, ties, signed
zero, and values up to 10^6. avr-libc 2.0.0 (the original's toolchain) and 2.2.1 (ours) print identical bytes for
all of them, and all 5,125 match this rule: **the exact binary value, rounded half away from zero to 4 decimals, but
to at most 8 significant digits (zero-padded); a negative value that rounds to zero prints "-0.0000"**. So the port
calls avr-libc's `dtostrf` on the AVR (same bytes as the original, by construction). The host build implements the
rule and is unit-tested against all 5,125 lines of `test/data/dtostrf.txt` (tools: `tools/dtostrf/`).

**Unit tests (Phase 2):** line assembly, parser, reply composition, formatter, IK/FK against the original C, legality,
segment generation, planner math.

**Acceptance baseline (your run, moves the arm):** `tools/acceptance/acceptance-H.txt` and `-L.txt`, one per
acceleration state. Run each straight after a power-cycle, from the probe pose, with `probe-send.sh`.

## 13. Open architecture questions (in addition to Q-1…Q-20 of the issue record)

- **Q-21** **ANSWERED** (skip that `@3` report when TX is full). Skip an `@3` report when TX is full, instead of blocking? (§5)
- **Q-22** **ANSWERED** (cap at 48 characters, `E21` above that). Keep 79-character sync lines (≈ 2.8 KB queue), or cap queued lines at 40 characters (longest libswiftpro command ≈ 42 with `#seq`, so 48) and reply `E21` above that?
- **Q-23** **ANSWERED** (option B: clear the pulse at ISR exit, 8 µs minimum). Step-pulse option B with an 8 µs minimum? (§6)
- **Q-24** **ANSWERED** (option c, see Q-1). `@3` option a, b or c? (§9; same as Q-1)
