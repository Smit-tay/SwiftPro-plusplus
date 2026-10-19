# SwiftPro++ firmware design

This document describes the firmware design and functiona.  This document is specifically intended for a developer who will read,
change or review the code. It describes what is built, the byte-level protocol in [PROTOCOL.md](PROTOCOL.md), and
the hardware map, budgets and measurements in [ARCHITECTURE.md](ARCHITECTURE.md). .

Contents

1. [Principles](#1-principles)
2. [Target and resources](#2-target-and-resources)
3. [Source layout](#3-source-layout)
4. [Start-up](#4-start-up)
5. [The main loop](#5-the-main-loop)
6. [Serial input and output](#6-serial-input-and-output)
7. [Lines and parsing](#7-lines-and-parsing)
8. [The protocol engine](#8-the-protocol-engine)
9. [Commands](#9-commands)
10. [Motion](#10-motion)
11. [Kinematics and frames](#11-kinematics-and-frames)
12. [Encoders and lost-step detection](#12-encoders-and-lost-step-detection)
13. [Reports, events and buttons](#13-reports-events-and-buttons)
14. [Peripherals and persistent settings](#14-peripherals-and-persistent-settings)
15. [Concurrency](#15-concurrency)
16. [Memory and timing](#16-memory-and-timing)
17. [Host build, simulator and tests](#17-host-build-simulator-and-tests)
18. [Working on the code](#18-working-on-the-code)

---

## 1. Principles

- **One cooperative main loop, no blocking.** Every module exposes a `poll()` that does a bounded slice of work and
  returns. Nothing in the main loop waits on hardware (the one exception is boot, before the loop starts). Long jobs,
  such as an encoder read or the check of a linear move, are state machines advanced one step per pass.
- **Interrupts do the minimum.** Five kinds of ISR exist: USART receive, USART transmit, the 1 ms tick, the arm's
  stepper timer, and the effector stepper's timer (modes 4/5 only). None re-enables interrupts, so ISRs never nest.
- **Fixed memory.** No heap, no STL, no exceptions, no RTTI. Every buffer has a fixed size, and running out of room
  is reported (a reply code, a counter, a refused line), never silent.
- **Small, flat modules.** A module is one header plus one source file, a namespace with free functions and
  file-local state. There are no class hierarchies and no abstraction with only one user. Classes are used where an
  object has real instances (`line::Assembler` per port, `fmt::Writer` per buffer) or for RAII (`InterruptLock`).
- **The same code on the host.** Everything except the I2C pin driver compiles natively. Hardware access sits behind
  `#if defined(__AVR__)`, with small host hooks (`host_*` functions) that the simulator drives. Behaviour is tested on
  the PC against golden protocol traces.
- **C++17, carefully.** `constexpr` constants with `static_assert`s on derived values, `enum class` for every state
  and result, `-Wall -Wextra -Wconversion -Wdouble-promotion -Werror`, `float` everywhere (avr `double` is 32 bits
  anyway, and the host must compute in the same precision).

## 2. Target and resources

ATmega2560 at 16 MHz: 256 KB flash (8 KB taken by the bootloader), 8 KB SRAM, 4 KB EEPROM. The stock Arduino
stk500v2 bootloader stays in place. Two of its properties shape the firmware (ARCHITECTURE.md):
it clears `MCUSR`, so the reset cause can't be known; and after a watchdog reset it jumps through RAM, so the
**watchdog must not be used**.

| Resource | Use | Module |
|---|---|---|
| Timer0 | 1 ms tick, CTC, clk/64, OCR0A 249: exactly 1 kHz | `tick` |
| Timer1 | stepper events, CTC at 2 MHz (0.5 µs) | `stepper` |
| Timer3 | stepper end effector (modes 4/5), CTC at clk/64, a half step every 192 µs | `tool` |
| Timer4 | wrist servo PWM on OC4A (PH3), phase-and-frequency correct, TOP 20000 at clk/8: 50 Hz, OCR4A = pulse in µs | `effector` |
| Timer5 | buzzer, CTC toggling OC5C (PL5) in hardware, no ISR | `buzzer` |
| USART0–3 | 115200 8N1; USART0 is the USB port | `serial` |
| ADC | one blocking conversion at a time (~0.1 ms): power sense A5, pump sense A9, A0–A15 queries | `adc` |
| Port pins | steppers, encoder buses (bit-banged I2C), effector outputs, limit switch, buttons, hardware straps | per module |

The stepper drivers are enabled by a high level on EN (VERIFIED from the original's settings and stepper code).
DIR high means negative steps, inverted on V3.3.0 boards. The pin list, with its source citations, is in
ARCHITECTURE.md §2.

## 3. Source layout

```
src/          firmware (all of it builds for AVR; all but i2c.cpp also builds on the host)
test/         host unit tests, the simulator, the golden-trace runner and traces
tools/        measurement and comparison tools (simavr profiling, kinematics and dtostrf comparisons, probes)
cmake/        AVR toolchain file, the "!!!" image check
```

Modules, from the bottom layer up. A module only calls modules below it, with one deliberate exception noted.

| Layer | Modules | Role |
|---|---|---|
| Hardware | `atomic.h`, `tick`, `serial`, `nvm`, `board`, `adc`, `i2c` | timers, UARTs, EEPROM, straps, ADC, I2C pins |
| Pure logic | `line`, `parse`, `fmt`, `kin` | line assembly, parsing, output formatting, kinematics: no hardware, unit-tested |
| Devices | `encoder`, `effector`, `tool`, `buzzer`, `pins` | AS5600 reading; wrist/pump/gripper/laser/limit/power; the stepper end effector of modes 4/5; buzzer; raw Arduino pin access |
| Motion | `planner` → `stepgen` → `stepper`; `motion` on top | move queue and speeds, 5 ms segments, the step ISR; motion commands |
| Application | `report`, `commands`, `protocol`, `app`, `main.cpp` | `@3`/`@4`, command handlers and machine state, the protocol engine, init and loop |

`report` and `commands` call each other: `commands` sets the report period (M2120), and `report::write` asks
`commands::measured()` for the latest encoder reading. That reading belongs to the encoder arbitration in
`commands` (§12).

## 4. Start-up

`main()` calls `app::init()` once, then `app::poll()` forever.

`app::init()` runs with interrupts disabled:

1. `board::init`: reads the hardware-version straps (PJ3–PJ6): version string, direction-invert mask, valve mode.
2. `serial::init`: four USARTs, RX pull-ups, empty rings.
3. `tick::init`: Timer0.
4. `commands::init`:
   - ADC and buzzer; starts the 260 ms boot beep (non-blocking).
   - Effector: wrist attached at 1510 µs, pump and valve off, gripper released.
   - Encoder buses, and the encoder references from EEPROM.
   - Motion: limits and acceleration from EEPROM, motors off, position unknown.
   - Work mode, offsets and UID from EEPROM, with fallbacks for erased or invalid values (R-7).
   - Report and buttons.
   - The boot position: each joint's encoder is read with a blocking `encoder::run()`, which is allowed here because
     the main loop hasn't started. If all three reads succeed, `motion::sync(7, angles)` sets the position and
     enables the drivers. If any read fails, the position stays unknown: moves get `E26` until `M17` succeeds (R-9).
5. `protocol::init`: the queue and the per-port line assemblers.
6. `sei()`, then the boot banner (`device name`, `soft ver`, `api ver`, `work mode`, `@1`) is written to the TX ring
   for USB.

## 5. The main loop

`app::poll()` → `protocol::poll()`, which does, in this order:

1. **Ctrl-X**: if the RX ISR saw 0x18, stop everything (as `S1100`, without a reply).
2. **`commands::poll()`**:
   - one step of encoder work (one I2C transaction, about 0.7 ms);
   - the recovery after a lost-step stop;
   - the pump valve sequence, the buzzer timeout, button sampling;
   - limit-switch and power sampling (once per ms);
   - **`motion::poll()`**: one piece of a linear move (check or feed), then refill the stepper ring from `stepgen`.
3. **Lost-step fault**: if the watch found one, stop everything and fail the queue with `E25`.
4. **Resume pending commands**: a query waiting for encoder reads (async), a sync command waiting for a re-sync, a
   line check or a delay.
5. **Take at most one line**, round-robin over the four ports, and only from a port whose reply is sure to fit in the
   TX ring. No line is taken while an async command is pending.
6. **Pump**:
   - the finished sync replies, in FIFO order, when allowed (§8);
   - one event line (`@8`, `@9`, `@6`, `@5`, `@4`);
   - the `@3` report when due.
7. **Start the next sync command** if none is pending and `commands::can_start()` allows its kind.

Every `poll()` returns whether it made progress. The firmware ignores this. The simulator runs the loop until a pass
makes no progress, which is how it knows that simulated time may advance.

Each pass is short. The longest is about 4.5 ms (one linear-move piece: two inverse-kinematics solutions plus a
planner insert); `@3` formatting is 3.15 ms. The stepper ring holds 40 ms of motion and the USB RX ring 22 ms of input
at full rate, so the loop never starves the motors or loses bytes in normal operation.

## 6. Serial input and output

`serial` owns the four USARTs.

- **Receive**: each port has its own ring (USB 256 B, others 64 B), filled by its RX ISR. When a ring overflows, the
  ISR drops bytes and puts `line::kLostMarker` (0x01) in the ring, using a 2-byte reserve so the marker and the line
  end always fit. The line assembler turns a marked line into `E23` instead of running half a command. The realtime
  bytes `? ! ~ @` are dropped in the ISR; 0x18 sets a stop request.
- **Transmit**: one shared 256 B ring and the UDRE interrupt. `write()` is all-or-nothing and never blocks; callers
  check `tx_room(port)` first. The ring serves one port at a time: for any other port `tx_room()` is 0 until the ring
  has drained. So a line is never interleaved with another port's output.
- **Routing**: replies go to the port the command came from. Unsolicited lines (events, `@3`) go to the port that
  last received a byte.

## 7. Lines and parsing

`line::Assembler` (one per port) turns bytes into lines. It keeps the original's normalisation:

- drops every byte ≤ space;
- strips `( … )` and `; …` comments;
- ignores `/`;
- upper-cases.

It returns `kLine`, `kTooLong` (more than 48 characters, `E21`) or `kLost` (`E23`).

`parse` holds pure functions:

- `route()`: splits off `#N` (up to 9 digits) and classifies the rest:
  - ignore;
  - `$` system command (`E20`);
  - async P/S;
  - sync G/M;
  - not a command;
  - bad sequence number.
- `command()`: reads the command word: family letter and an integer 0–65535 (`G2202.5` is refused).
- Parameter scanners: `expect` (a letter), `number` (a bounded run of `[0-9+-.]` read as a float), `integer` (int16).
- `read_float()`: correctly rounded. Integer digits are divided by an exact power of ten, so `"0.1"` gives the nearest
  float, unlike the original's accumulate-and-scale.

Handlers parse strictly: unexpected trailing text is `E21` (Q-53).

## 8. The protocol engine

`protocol` implements the uArm framing: replies `[$N ]<code>[ tail]\n`, with codes `ok`, `E20`–`E26`.

**Async (P and S)** commands run as soon as their line is taken and reply at once. A command that needs encoder reads
(`P2200`, `P2206`, `P2220`, `P2221`, `P2243`, `P2244`, `P2246`) returns `kPending`; the protocol keeps its sequence
number and port, stops taking lines, and calls `commands::resume_async()` each pass until it completes. `S1100` is
handled here: `commands::stop()`, every queued command answered `E25`, then `ok`. `M2200` ("moving?") is a G/M word
but is answered at once, like a query (Q-56).

**Sync (G and M)** commands go into a 20-entry FIFO. An entry is `{port, seq, text, state, motion}`. While it is
`kWaiting`, `text` holds the line; once it has run, `text` is overwritten with the reply (`kDone`). Entries run in
order, one per pass, when `commands::can_start(kind)` allows:

- **motion** commands start whenever the motion layer can take one (`!motion::busy()`), so moves queue up in the
  planner and blend;
- **normal** commands (everything else) wait until the machine is idle (Q-5);
- nothing starts while the buzzer sounds (faithful).

A sync command can also return `kPending`: a re-sync (`M17`, `M2201`), the whole-path check of a linear move, or a
`G2004` delay. No other sync command starts until it finishes, but queries keep being answered.

**Reply gating.** Finished sync replies are sent in FIFO order, and only when `commands::idle()`: motion stopped,
buzzer silent, final lost-step checks done. With `M2124 V1` (Q-32), a motion command's reply goes out as soon as the
command was accepted; the host then uses `@9` to know when the arm arrives. A full queue answers the next line `E23`
at once; the queue counts entries waiting to run and entries waiting to reply.

**Backpressure.** A line is taken only if a maximal reply (`kReplyMax`, 63 bytes) fits the TX ring for that
port. So a reply is never dropped, and a slow host simply slows down how fast the firmware reads its input.

## 9. Commands

`commands::execute(cmd, tail)` dispatches on family and number to the handlers in `commands.cpp`: queries, G codes,
M codes, settings, kinematics. A handler:

- parses its own parameters;
- appends any reply tail (for example `" V1"`) to the `fmt::Writer`;
- returns a `Status`: `kOk`, `kE20`–`kE26`, or `kPending`.

`commands` also owns the machine-wide state the protocol needs:

- `kind()`: motion or normal;
- `can_start()` and `idle()`: the gates above;
- `next_event()`: the event lines;
- the encoder bus arbitration (§12);
- small settings: modal feed, `@9` reporting, streaming, work mode, offsets, UID.

Unknown or not-yet-built commands return `E20` (the original replied `ok` to anything it didn't know).

## 10. Motion

```
command ──► motion ──► planner (16 blocks) ──► stepgen (5 ms segments) ──► stepper ring (8) ──► Timer1 ISR ──► pins
            checks,     entry speeds,            trapezoid per segment,      evenly spaced events,
            IK, pieces  corner blending          40 ms moving average        Bresenham, tally
```

**Positions.** Each joint's angle is its boot angle (from the encoders at boot or the last re-sync) plus its step
tally / 80. The planner works in tally steps. "The current position" for relative moves and missing axes is the end of
the queued motion, where the last accepted command will leave the arm.

**`motion`** turns commands into planner blocks:

- **Joint moves** (`G0`, `G2202`, `G2206`): the target is checked against the joint limits (`E22`, nothing moves),
  then becomes one straight block in joint space.
- **Linear moves** (`G1`, `G2201`, `G2204`, `G2205`) are straight lines for the effector:
  - the line is checked piece by piece, one piece per pass, and every piece must be reachable before anything moves
    (`E22` otherwise, SP-20);
  - it is then fed to the planner as room allows;
  - pieces are 0.25–20 mm, sized so the joint-space chord stays within 0.02° of the true path (R-4).
- **Feed** F is in the original's units, 80 °/min of joint-space motion (Q-51). It is modal, F100 after boot.
- **Acceleration** is set by `M204 A`, A × 80 °/s² per joint, saved in EEPROM, default A5 (400 °/s²).
- **Joint speed cap**: 300 °/s.

**`planner`** keeps up to 16 blocks `{steps, length, unit vector, nominal speed, accel, max_entry², entry²}`:

- Each block's acceleration and speed are scaled so that no single joint exceeds the per-joint limits.
- The entry speed into a block is limited by both blocks' nominal speeds and by the corner. The corner speed comes
  from a circle tangent to both moves that stays within 0.2° of the corner point (`corner_speed2`). Straight on is
  unlimited; a reversal is zero.
- A backward pass makes every block able to slow down to the next block's entry speed. A forward pass makes every
  entry speed reachable. The last block ends at rest.
- The executing block is never replanned: `stepgen` reports its live speed and the length remaining, and the forward
  pass starts from those.

**`stepgen`** produces one 5 ms segment per call:

1. It advances the executing block along an accelerate / cruise / decelerate profile. The profile is recomputed
   analytically each segment from the live speed, the remaining length, the nominal speed and the current exit speed,
   so a block added later can still raise the exit speed.
2. The resulting joint positions pass through a **moving average over 8 segments (40 ms)**, in integer 1/256-step
   units. Acceleration therefore ramps over 40 ms instead of jumping, which limits jerk (R-5). At rest the average
   equals the exact target, so final step counts are exact.
3. A move that starts from rest waits for the average to settle first, so a reversal doesn't cut the corner.

Each segment is the difference of whole steps from one segment to the next (at most 160 steps per joint).

- **Hold** (`S1000 V0`) decelerates along the path, across block boundaries if needed, and waits; `V1` resumes.
- **Stop** (`S1100`, Ctrl-X, a lost-step fault) drops everything and brakes from where the motors actually are, all
  joints together along their current direction, at 3× the acceleration limit (Q-60). There is no smoothing during
  the stop. The tally stays exact.

**`stepper`** keeps a ring of 8 precomputed segments `{steps[3], dir, events, period, extra}`; the event period and
its remainder are worked out when the segment is queued, not in the ISR. On each Timer1 compare the ISR:

1. ends the previous step pulse;
2. if the segment is finished, loads the next one (sets DIR, commits the finished segment to the tally);
3. runs the unrolled Bresenham for the three joints and raises their STEP pins;
4. sets the time of the next event.

A pulse therefore lasts until the next event (≥ 31 µs); the drivers step on the rising edge. `position()` returns the
committed tally plus the steps already made in the current segment, computed from the Bresenham state, under an
interrupt lock. It is exact at any instant (SP-16). Measured under simavr: 9 µs per event, 17 µs when an event loads
a segment.

## 11. Kinematics and frames

`kin` is a fresh geometric solution, with the arm's dimensions taken from the original:

- lower arm 142.07 mm, upper arm 158.81 mm;
- shoulder 13.2 mm from the base axis and 107.4 mm up.

The functions:

- `forward(L, R, base) → arm XYZ`;
- `inverse(arm XYZ) → L, R, base`: elbow-up, `atan2` plus the law of cosines; unreachable gives NaN;
- `legal()`: the joint limits (L −20…135.6°, R −20…119.9°, 180 − L − R within 10…151°, base −90…90°);
- `arm_to_effector` / `effector_to_arm`: the work mode's front and height offsets;
- steps ↔ angles, rounded to the nearest step (R-1).

Compared with the original on the real AVR's floating point (simavr, 26,436 values), results agree within 0.0024°.
The new inverse kinematics is about 100× more accurate in round trips.

**Frames.** The base angle has two frames, and the protocol uses both:

- the **user frame** (90° = straight ahead): `P2200`, `G2202 N0`, `G2206 B`, encoder angles;
- the **−90 frame** (0 = straight ahead): `@3 B` and all internal motion.

Each conversion is written out where it happens; grep for `- 90.0f`.

## 12. Encoders and lost-step detection

Each joint has an AS5600 on its own bit-banged I2C bus (`i2c`):

- SDA is open-drain; SCL is push-pull.
- Each phase is 5 µs, so a byte takes about 0.15 ms.
- Each `i2c` function is one short bus operation.

**`encoder`** is a state machine that does one I2C transaction (START to STOP, about 0.7 ms) per `poll()` (R-16). Its jobs:

- `kAngle`: the median of three ANGLE samples (a two-byte burst each);
- `kHealth`: STATUS, AGC and MAGNITUDE;
- `kWatch`: three ANGLE samples plus STATUS.

A NACK or an impossible value fails the job, and the failure is reported. The angle conversion is the original's:
`pointB − wrap(reference − raw) × 360/4096`, with the reference counts from EEPROM.

**Arbitration** (`commands::poll_reads`). The bus is shared by three users, in priority order:

1. **queries** (async read set);
2. **re-syncs** (sync read set, also used by the lost-step recovery);
3. **the watch**: whenever the bus is otherwise free, it reads the joints in turn, B, L, R, starting at most one read
   per millisecond (R-12).

A read in progress always finishes before the bus changes hands.

**The watch** serves two purposes:

- **Readings for `@3`**: each successful reading is kept, with its time, for the measured-joint field.
- **Lost-step check** (SP-41, Q-26):
  - **When**: while the stepper runs, every joint is marked "check due".
  - **How**: a due, locked joint with a known position is checked. Its tally is sampled just before and just after the
    read, and the encoder angle must lie within 1.0° of that range. Motion during the read therefore can't cause a
    false alarm.
  - **When the check clears**: the mark clears only after a passing check made while the tally was still. So every
    move ends with one final check per joint, and `commands::idle()`, and with it a move's `ok`, waits for that
    check (R-13).
  - **What isn't checked**: failed readings and readings with no magnet (STATUS MD = 0, SP-42). They reset the fault
    count.
- **Fault**: two failing checks in a row on one joint set a fault. The protocol then stops as `S1100` (the moving and
  queued commands get `E25`), `@8 N<joint> V<tally − encoder>` is sent, and once motion has stopped every locked
  joint's tally is re-read from its encoder. Commands wait until that has finished.
- **Switch**: `M2125 V0` turns the check off; it is on after every boot (Q-61). The readings go on either way.

## 13. Reports, events and buttons

**Events** are produced by `commands::next_event()`, one per main-loop pass, after the replies, in this priority:

- `@8`: lost steps;
- `@9 V0`: motion done, when `M2122 V1` is set;
- `@6 N0 V<0|1>`: limit switch, debounced 10 ms;
- `@5 V<0|1>`: motor power;
- `@4 N<b> V<e>`: buttons.

**`@3`** (`report`): `M2120 V<seconds>`, from 0.01 s to 3600 s. `V ≤ 0` stops it, as does `M2121`. The line starts
with the original's seven fields from the step tally, then:

- one joint measured by its encoder, in turn B, L, R, with the reading's age (`M<j><deg> A<ms>`, or `M<j>-` if there
  is no good reading);
- `T<ms since boot>`;
- `S<counter>`.

Reports are scheduled on the 1 ms tick without drift. A due report is sent only if a full reply still fits after it.
Otherwise it is skipped but still counted in `S`, so reports never delay replies and the host can see the gap (R-14).

**Buttons** (Menu PE7, Play PE6, pressed = low):

- sampled every 10 ms;
- a change of state needs 3 samples in a row;
- held ≥ 1 s is a long press;
- the event is sent on release, only after `M2213 V0` (SP-44).

## 14. Peripherals and persistent settings

- **`effector`**:
  - Wrist servo: 0.5 µs resolution (about 0.05°); its angle is reported from the pulse actually output; release
    really stops the PWM.
  - Pump: with the V3.3.0 valve sequence run as a timed state machine (the original blocked for a second).
  - Gripper, with its sense input; laser; limit switch; motor-power sense (ADC A5 > 100).
  - Pump status from ADC A9.
- **`tool`** (modes 4/5, untested on hardware): the stepper end effector on DIR PH5 / STEP PH6, driven by the
  Timer3 ISR at the original's rate (one step per 384 µs).
  - Origin search: clockwise for up to 360° until the switch on PK5 closes, in the background (the original blocked
    serial input). At the switch the angle is minus the effector angle offset (EEPROM 3202, `M2412`); then the effector
    moves to 90°.
  - Position is kept in steps and targets are rounded (R-17). Until a search succeeds, `G2202 N3` gets `E26`.
  - While it moves, replies wait and no sync command starts, as in the original.
- **`pins`**: `M2240`/`M2241`/`P2240` on Arduino Mega pin numbers 0–69. The table is taken from the Arduino
  framework the original was built with. Pins the firmware drives are refused (Q-17); reads never touch a timer.
- **`buzzer`**: Timer5 toggles the pin in hardware; `poll()` ends the tone on time.
- **`board`**: the straps select the hardware version string, the direction-invert mask (V3.3.0: all joints
  inverted) and whether the valve is pulsed.
- **`nvm`**: EEPROM at the original's addresses (ARCHITECTURE.md §10), so settings stored by the original firmware are
  read as they are. `write()` writes only the bytes that differ and blocks interrupts only for the 4-cycle write
  sequence. It is used only by commands that save settings.

## 15. Concurrency

What runs in interrupt context, and what it shares with the main loop:

| ISR | Touches | Shared with the main loop through |
|---|---|---|
| USARTn RX | that port's RX ring, the stop flag, `last_port` | single-producer / single-consumer ring with `volatile uint8_t` indices: each index has one writer, and 8-bit accesses are atomic |
| USARTn UDRE | the TX ring | the same |
| TIMER0_COMPA | the ms counter | `now_ms()` reads it under `InterruptLock` (32-bit) |
| TIMER1_COMPA | the segment ring, the current segment, the tally, STEP/DIR pins | ring indices; `position()` and `stop()` under `InterruptLock` |
| TIMER3_COMPA | the effector stepper's remaining steps and position, PH6 | `volatile` flags; the position read under `InterruptLock`; the main loop only changes them with the timer interrupt off |

Rules:

- **Read-modify-write of shared ports.** The stepper ISR writes PORTF and PORTL. PORTH, PORTK and PORTL are outside
  the `sbi`/`cbi` range, so even a one-bit change to them is a read-modify-write. Main-loop changes to these ports,
  and to other ports shared between modules (PORTG, the I2C pins), are made under an `InterruptLock`. The lock saves
  and restores SREG, so it nests safely.
- **No nesting.** No ISR re-enables interrupts. Worst-case latency is the longest ISR plus the longest locked region:
  tens of µs.
- **Floats in ISRs.** Floating point never runs in an ISR.

## 16. Memory and timing

Measured with avr-gcc 15.2.0 (`cmake --build build/avr` prints the sizes):

| | |
|---|---|
| Flash | 45,580 B of 253,952 B usable |
| Static RAM | 4,088 B of 8,192 B: the sync queue 20 × 64 B, rings 256 + 3 × 64 + 256 B, planner 16 blocks, stepper ring, buffers |
| Stack | 527 B high-water mark, measured on the real image under simavr over a full session (`tools/simavr`) |
| Stepper ISR | 9 µs per event, 17 µs on segment load (under 30 % CPU at the 300 °/s cap) |
| Main-loop work | IK 1.26 ms, FK 0.88 ms, planner insert 1.7 ms, one segment 1.2 ms, one `@3` 3.15 ms, `dtostrf` 0.15 ms |

The timings are worst cases measured under simavr with `tools/profile/run.sh`. On the real image under simavr
(`tools/simavr`), an encoder sample takes 1.2 ms and a joint reading 3.6 ms. On the real bus these are UNVERIFIED.

## 17. Host build, simulator and tests

The host build compiles every module except `i2c.cpp` into `firmware_host`, together with **`test/sim.cpp`**, the
simulator:

- **Simulated hardware**: an EEPROM image, the straps, the ADC inputs, the limit switch, the gripper sense, the buttons.
- **Encoders**: an AS5600 model per bus, behind the `i2c` functions. Its registers are STATUS, ANGLE, AGC and
  MAGNITUDE, with NACK injection.
- **Motors**: each encoder follows its motor's actual step pulses (`stepper::host_pulses`) unless the trace stalls or
  shifts it (`enc.stall`, `enc.slip`).
- **Time** is simulated. `sim_advance_ms` ticks the 1 ms timer, runs the main loop until it makes no progress, runs
  1 ms of Timer1 events, and runs the loop again.

**Golden traces** (`test/golden/*.txt`, run by `test/golden.cpp`) are scripts of inputs and expected output:

```
= enc.stall L                      set a simulated input
> #6 G2202 N1 V80 F1000            send a line
~ idle                             advance time until idle
<re ^@8 N1 V-?[0-9]+\.[0-9]{4}$    expect a line (exact, or a regex)
< $6 E25
```

Every expectation carries a provenance tag:

- `faithful` with a `file:line` of the original;
- `measured` with a probe log of the real arm;
- `change` or `new` with the label of the decision behind it.

So every byte the firmware sends is traceable to a reason. The full syntax is at the top of `test/golden.cpp`; the
simulator's inputs are listed in `test/sim.h`.

**Unit tests** (`test/unit*.cpp`) cover:

- line assembly and parsing;
- serial ring behaviour;
- `fmt::f4` against 5,139 values printed by avr-libc's `dtostrf` under simavr (byte-exact, including NaN and
  infinity);
- kinematics against the original's output on the AVR;
- planner and stepgen limits (peak speed and acceleration, exact end positions, hold, stop);
- the lost-step watch with a slipping encoder.

**The real image under simavr** (`tools/simavr/run.sh <simavr dir>`) runs `build/avr/swiftpro-pp.elf` on a
simulated ATmega2560. The harness provides:
- the UART, fed at line rate;
- three AS5600 slaves emulated bit by bit on the real encoder pins, so the real I2C driver is exercised;
- step counting on the real STEP/DIR pins, the effector stepper included;
- the hardware straps and the ADC;
- an EEPROM image.

It paints the stack and reports the high-water mark. A scripted session (`tools/simavr/session.txt`, a subset of
the trace syntax) covers boot, identity, encoder queries, moves with exact step counts, `@3`, `S1100`, lost steps,
buzzer, wrist, pins and the stepper end effector. This catches what the host build can't: start-up code, interrupt
setup, pin polarity, I2C timing, flash strings, the real stack. `HARNESS_DEBUG=1` timestamps the output,
`HARNESS_I2C=1` logs bus transactions, `HARNESS_PROFILE=1` samples the PC.

**Tools** (`tools/`): `simavr` (above), `profile` (AVR timings under simavr), `kin` (the original's kinematics against this one, both on
simavr), `dtostrf` (generates the formatting reference), `probe` and `acceptance` (scripts for the real arm, run by
hand).

## 18. Working on the code

**Adding a command:**

1. Add a `case` to the right handler in `commands.cpp`: `query`, `gcode`, `mcode`/`setting`, or `scode`.
2. Parse strictly with `parse::expect` / `number` / `integer` or `digit_param`.
3. Write the tail with the `Writer`, and return a `Status`.
4. If it must wait for hardware, return `kPending` and complete it in `resume_async`/`resume_sync`.
5. If it moves the arm, return `Kind::kMotion` from `kind()`.
6. Add a golden trace block with a provenance tag, and the command to PROTOCOL.md.

**Changing behaviour the host can see:** record the evidence and the decision under a new label first; the traces'
tags must point at it.

**Before committing:**

- `ctest` passes in the host build;
- the AVR build is warning-free;
- the `!!!` check passes;
- if a call chain got deeper, re-check the stack estimate (`-fstack-usage`).
