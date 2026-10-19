# First tests on the arm

A plan for the first runs of SwiftPro++ on the uArm Swift Pro, in three stages, each with what to expect and what to
send back. You flash and run everything; nothing here has been run on hardware yet. Each stage is a command file in
`tools/bringup/`, sent with the probe script you already used:

```sh
picocom -b 115200 --send-cmd "$PWD/tools/probe/probe-send.sh" /dev/ttyACM0
# wait for "@1", then C-a C-s and the path of the stage file; the log goes to probe-<time>.log
```

Send back the log of each stage. If anything below says **stop**, stop there and send the log.

## Before you start

1. **Keep a way back.** The original firmware is in the original repository as
   `hex/uArmPro_V4.9.1-jss.7_release_20260525.hex`; flash it with the same avrdude command. Don't flash `flash-read.hex` through the bootloader: it is a dump of the
   whole flash, bootloader section included.
2. **Build and flash**:
   ```sh
   cmake -B build/avr --toolchain cmake/avr-gcc.cmake && cmake --build build/avr
   avrdude -c wiring -p m2560 -P /dev/ttyACM0 -b 115200 -D -U flash:w:build/avr/swiftpro-pp.hex:i
   ```
   The build fails if the image contains `!!!`, which would break the upload.
3. **EEPROM**: the firmware reads the original's settings where the original kept them: work mode, offsets, encoder
   reference, UID, effector angle offset. It writes nothing at boot. The only new setting is the acceleration
   (`M204`) at address 960, which the original never used (Grbl's area starts at 2049, `settings.h:60-63`).
   Flashing the original back leaves everything as it was.
4. Put the arm in the **probe pose** (about B90 L46 R66, as for the probe and acceptance runs) before powering on.

## Stage 1: read-only (`tools/bringup/stage1-readonly.txt`)

Motor power on, nothing moves. The drivers hold the joints as soon as the boot position has been read, as with the
original.

| Check | Expected | If not |
|---|---|---|
| Boot banner | `device name : SwiftPro`, `soft ver : V5.0.0-pp.1`, `api ver : V4.0.5`, `work mode : <your mode>`, `@1`, then `@5 V1` (motor power) | No banner at all: check the baud rate, then reflash the original and send me the avrdude output |
| Short beep at boot | 1 kHz, 0.26 s | Note it; not critical |
| `P2201`…`P2205`, `P2400` | `ok VSwiftPro`, `ok V3.3.1`, `ok V5.0.0-pp.1`, `ok V4.0.5`, your UID, your mode | — |
| `P2234` | `ok V1` | Motor power not sensed: **stop** |
| `P2244` | `ok V0` (all three encoders healthy) | `E26 V<mask>`: note the mask (4 L, 2 R, 1 B); **stop** before stage 2 |
| `P2246` | `ok B32,<agc>,<mag> L… R…`; 32 = magnet detected, no warning. AGC and magnitude are new information: just report them | 0 or a value with 8 or 16 added: magnet too weak or too strong |
| `P2242`, `P2243`, `P2200`, `P2220` | the same values the original gave in the same pose: your probe log, section B (B ≈ 89.9, L ≈ 45.7, R ≈ 66.0) | `E26`: an encoder read failed, so the I2C timing on your board differs from the simulation; **stop** |
| `M2203 N0/N1/N2` | `ok V1` each (locked) | `V0`: the boot read failed and the drivers were left off; **stop** |
| `@3` (section C) | about five lines, the first seven fields matching `P2200`: `B` is P2200's B − 90, `L` = L, `AR` = R. Then `M<j>` (one measured joint, in turn B, L, R) with its age `A`, plus `T` and `S` | Tally and encoder disagree: **stop** |
| `@3` at 100 Hz (section D) | about 200 lines in 2 s; `S` counts up by 1 (a jump means a skipped report) | Note the gaps |
| Section E | `$40 E22`, `$41 E22`, `$42 E20`, `$43 E21`, `$44 E21`, `$45 E21` | — |
| Section F | `$50 ok V0` (limit switch open), a beep, then `@4 N0 V1` and `@4 N1 V2` after you press the buttons | No `@4`: the button pins (PE7/PE6, from the older firmwares) may differ on your board. Report it; nothing else depends on it |

## Stage 2: slow single-joint moves (`tools/bringup/stage2-motion.txt`)

**The arm moves.** Clear a 300 mm radius and keep a hand on the power switch. Power-cycle into the probe pose first.
Every move is 5–20°, mostly at F20 (about 27 °/s), and in directions that keep the effector at or above its starting
height (the same directions as the acceptance runs).

| Check | Expected | If not |
|---|---|---|
| Direction | Base +5° turns the same way `G2202 N0 V95` did with the original. L +10° lifts the upper arm; R −10° lifts and extends | Wrong direction: **cut the power**. That would be a DIR polarity error on your board version |
| Accuracy | after each move, `P2200` within about 2 encoder counts (0.18°) of the target and of `@3`. One count is 0.088°. First run, 2026-10-18: up to 1.5 counts, L +0.13° after its first move; this looks like gear slack taken up after boot (UNVERIFIED until compared with the original's acceptance runs) | Report the numbers |
| Smoothness | no clunk at start and stop. Acceleration ramps over 40 ms, and the default acceleration is A5 (400 °/s²) | Note anything that sounds or looks rough |
| No `@8` | none during these moves | An `@8` without an obstruction is a false alarm: report the line |
| Hold (5) | the base slows to a stop, `$52 ok V1` (still moving: held), resumes, ends at 110 | — |
| Stop (6) | `ok` at once, the base brakes within a fraction of a second, then `$60 E25`. `P2200` and `@3` still agree | Disagreement means steps were lost while braking at 3× the acceleration (Q-60): report it |

## Stage 3: features (`tools/bringup/stage3-features.txt`)

Run with the interactive runner, which waits for each reply and asks you before every step that needs your hands:

```sh
tools/bringup/run.sh tools/bringup/stage3-features.txt
```

| Check | Expected |
|---|---|
| Test pose | `P2220` ≈ X240 Y0 Z64 (FK from L70 R40 in mode 0 gives X240.10 Z64.27) |
| Square (2) | `P2220` within about 0.2 mm of each corner (X260/220, Y±20, Z64). The effector travels straight edges, not arcs |
| Streaming (3) | five `ok`s at once, the arm runs the square **without stopping at the corners**, then `@9 V0` |
| Wrist, pump, gripper (4) | the wrist turns to 45°, 135°, 90°; `P2206 N3` = `ok V90.0000`; `P2231`/`P2232` as the original (pump: 1 running, 2 holding; gripper: 1 closed empty, 2 holding) |
| Release and re-lock (5) | after you move L by hand, `M2201 N1` re-reads it, and `P2200` and `@3` agree on the new angle |
| Lost steps (6, optional) | holding the base still at F20 gives `@8 N0 V<error>` and `$50 E25`; the arm stops and re-reads its position |
| Back home (7) | the probe pose again |

## Afterwards

If all three stages pass, the next steps are:

- the original's acceptance runs (`tools/acceptance/acceptance-L.txt` and `-H.txt`), so the two firmwares can be
  compared on the same moves;
- a tuning session for acceleration (`M204`) and the lost-step threshold.

The acceptance files carry one comment that no longer applies: in this firmware a G1-family move doesn't change the
acceleration (Q-52), so "state L" is simply the default A5.

## Results

### 2026-10-18, first bring-up (image sha256 a4eb9c08…a9fd0a4c for stage 3; logs kept outside the repository)

| Stage | Result | Notes |
|---|---|---|
| 1 read-only | **pass** | Encoder readings identical to the original's probe log. `@3` at 100 Hz: 239 lines, no gaps in `S`. Buttons not tried |
| 2 slow moves | **pass** | Directions right. Encoder against target up to 1.5 counts (L +0.13° after its first move, likely gear slack, UNVERIFIED). The hold test came too late (script timing); repeated in stage 3 |
| 3, first attempts | aborted | Boot position read failed after a reset; the drivers stayed off and the arm dropped. Cause: I2C bus stuck by the reset (SP-62). Fixed by a bus clear at boot, and R-9 revised so the drivers hold the arm |
| 3 features | **pass** | Hold; test pose; linear square within one base count (0.37 mm); streamed square ran through all four corners without stopping (observed); wrist; pump ran (no effector fitted, gripper untested); release, hand move and re-lock; lost steps caught 0.21 s after the base was held by hand, the move got E25 and the position was recovered |

Open observation (UNVERIFIED): during the R/L moves at F100 back to the probe pose, the base encoder moved 6 counts
(0.53°) with no base command. It is to be compared with the original firmware on the acceptance moves.
