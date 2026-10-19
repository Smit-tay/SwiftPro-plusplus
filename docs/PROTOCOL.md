# PROTOCOL — serial protocol of the original firmware, and what the port does with it

Source: `Smit-tay/SwiftProForArduino` `src/` at commit `72a2ccf` (`V4.9.1-jss.7`, API `V4.0.5`).
Citations are `file:line` in that `src/`. VERIFIED = read in code (or measured); UNVERIFIED = inferred.
Cross-checked against `doc/uArm Swift Pro Protocol V4.0.5_en.pdf` (called "the doc" below), against libswiftpro
(`Smit-tay/libswiftpro`, "L") and against the ROS2 driver (`Smit-tay/SwiftRos2`, `swiftpro_hardware`, "R"; it only
talks through libswiftpro).

**Decision column.**  `drop` = command removed; it then replies `E20` like any unknown command. 
`port` = the fix proposed in the issue record.  Otherwise, the behaviour stays faithful.

---

## 1. Transport

| Item | Original | VERIFIED where |
|---|---|---|
| Port | USART0 at 115200 8N1 (U2X on). USART1–3 also enabled at 115200; all four share one RX ring and one TX ring, and replies go to the UART that last received a byte | `serial.c:65-142, 330-468` |
| RX ring | 128 bytes; overflow drops the byte silently | `serial.h:27`, `serial.c:345-361` |
| TX ring | 128 bytes; writers busy-wait when full | `serial.h:30`, `serial.c:147-178` |
| Realtime bytes, intercepted in the RX ISR wherever they appear | `?` status report, `!` feed hold, `~` cycle start, `@` safety door, `0x18` reset | `config.h` `CMD_*`, `serial.c:338-343` |

**Line assembly** (`protocol.c:80-129`): a line ends at `\n` or `\r`. Every character `<= ' '` is discarded, so
**all spaces vanish**: `G2202 N3 V90 F100` reaches the parser as `G2202N3V90F100`. `(…)` and `;…` comments are
removed, `/` is ignored, and lower case is folded to upper. At 79 characters the firmware prints
`error: E23\n\r\n` (`report.c` `STATUS_OVERFLOW`), and the rest of the line is assembled as a new line .
The main loop assembles at most one line per iteration (`protocol.c:86` `break`).

**Port:** all four USARTs kept for Bluetooth , with RX pull-ups enabled. RX 256 B with overflow detection . Line assembly faithful.


## 2. Framing and routing

`receive_cmd_line()` (`uarm_protocol.c:23-71`):

| Line starts with | Route |
|---|---|
| `$` | Grbl system command, executed immediately; reply `ok\r\n` or `error: …\r\n` with no `$N` (`system.c:105-260`) |
| `#<digits>` | Sequence prefix. The digits are copied to `temp_str[10]` (unbounded), and the reply will start with `$<digits>`. Routing then continues on the rest of the line. A line `#N$…` is **not** a system command: it goes to the sync queue and replies `$N\n`  |
| `P` or `S` (after an optional `#N`) | **Async**: stored in a single slot (`asyn_pack`), executed in the same main-loop pass, reply printed right away. A second P/S line overwrites the slot if the first has not run yet. Since the loop reads one line and then executes, that does not happen in practice (UNVERIFIED) |
| `G` or `M` (after an optional `#N`) | **Sync**: appended to a 20-entry FIFO (`PACK_MAX_SIZE`, `uarm_protocol.h:9`). Full → `E23 buffer full\r\n` without `$N`. Execution rules below |
| anything else | ignored, **no reply** |

**Sync execution** (`parse_cmd_line`, `uarm_protocol.c:134-165`): one queued command per main-loop pass, but only
when `gcode_delay_flag` is clear (G2004 not running), the stepper end effector is idle (`effect_ldie`) and the buzzer
is idle (`beep_ldie`). It does **not** wait for motion to finish.

**Sync reply** (`report_parse_result`, `uarm_protocol.c:175-185`): printed in FIFO order, only when
`sys.state == STATE_IDLE` and the effector and buzzer are idle. So **G/M replies appear only when the arm is idle**
( VERIFIED). P/S replies are printed as soon as they execute, so **reply order may differ from
send order**. The host matches replies by `$N`.

**Reply text** (`add_result_to_report`, `uarm_protocol.c:74-132`):

```
[ "$" N ] [ " " ] CODE [ TAIL ] "\n"
CODE  = "ok" | "E20" | "E21" | "E22" | "E26" | "" (empty)
TAIL  = " V…\n"-style text set by the handler (it includes its own "\n"), or just "\n" when empty
```
The space before CODE appears only when `$N` is present. Without `#N` the reply is `ok\n`. There is no `\r`.

| Internal status | Value | Reply code | Note |
|---|---|---|---|
| `STATUS_OK` / `UARM_CMD_OK` | 0 | `ok` | |
| Grbl 1–11, 21, 22 | | *(nothing)* |
| `STATUS_GCODE_UNSUPPORTED_COMMAND` | 20 | `E20` | |
| `UARM_CMD_ERROR` = `STATUS_UARM_ERROR` | 50 | `E21` | parameter error |
| `UARM_COORD_ERROR` | 51 | `E22` | unreachable target |
| `UARM_ENCODER_ERR0R` | 52 | `E26` | P2244 only |
| any other (12, 23–37, …) | | `E20` | default branch |

The doc also lists E24 (power) and E25 (operation failure); the firmware never emits them (VERIFIED by grep).
E23 appears only in the two malformed forms.

**Port:** framing, `$N`, spacing and FIFO depth 20: faithful. Error mapping: change.

## 3. Boot output (faithful unless noted)

In order (`uarm_swift.c:7-37`, `uarm_peripheral.c:327-385`):
```
angle offset:<dtostrf(effect_angle_offset,5,4)>\r\n     (debug, change: removed)
device name : SwiftPro\r\n
soft ver : V5.0.0-pp.1\r\n                               
api ver : V4.0.5\r\n
work mode : <n>\r\n
@1\n
```
plus possibly `error: EEPROM read fail. Using defaults\r\n` from `settings_init` (dropped with the Grbl settings)
and `['$H'|'$X' to unlock]\r\n` if in ALARM (dropped). A 260-period 1 kHz beep (260 ms) plays at boot
(`uarm_swift.c:14`). In work modes 4/5 the effector origin search starts at boot.

## 4. Unsolicited events

| Event | Format (exact) | Source | When | Host | Decision |
|---|---|---|---|---|---|
| `@1` | `@1\n` | `uarm_swift.c:34` | end of boot | L logs | port |
| `@3` | `@3 X%s Y%s Z%s R%s B%s L%s AR%s\n`, each `%s` = `dtostrf(v,5,4)` | `uarm_peripheral.c:210-244` | every M2120 period, from the main loop | L+R (`on_position`, `on_joint`) | port + append measured joints, then `T<ms> S<seq>`; atomic read . Exact appended format: §10. Skipped (and counted in `S`) when a reply wouldn't fit after it (R-14) |
| `@4` | `@4 N<0 Menu\|1 Play> V<1 click\|2 long press>\n` | V4.0.5 doc only; not in jss.7 | on release, when `M2213 V0`. Menu PE7, Play PE6 (pins from the V3 and V4.10.0 firmwares, UNVERIFIED); 10 ms samples, 3 to change state; held ≥ 1 s = V2 | none | **new** |
| `@5` | `@5 V%d\n` (1 = motor power present, ADC A5 > 100) | `uarm_protocol.c:203-214` | on change, polled every 256 loop passes | L+R | port |
| `@6` | `@6 N0 V%d\n` (1 = effector limit switch PG3 low) | `uarm_protocol.c:192-201` | on change, 10 ms blocking debounce | L+R | port; non-blocking debounce  |
| `@9` | `@9 V0\n` | `uarm_protocol.c:216-221` | idle after any move, if M2122 V1 | L+R | port |
| Grbl | `ALARM: …\r\n`, `[…]\r\n`, `<Idle,MPos:…>\r\n` | `report.c` | alarms, `?` | none | drop |
| debug | `sscanf N\r\n`, `yes\n`, `effect check timeout!\r\n`  | | none | drop |

`@3` fields: X/Y/Z = Cartesian effector position from the **step tally** (not the encoders), R = wrist servo angle
(commanded), B = base joint angle **in the "−90" frame** (`get_current_angle` subtracts 90), L, AR = left and right
joint angles. VERIFIED. Note that B here is 90 less than `P2200`'s B. libswiftpro accepts 4 or 7 tokens.

## 5. Angle frames and units (VERIFIED)

- L/R joint angles: the encoder frame, the same for `P2200`, `G2202 N1/N2`, `G2206 L/R` and `M2220/M2221`.
- Base: the "user" frame (90 = straight ahead) for `P2200 B`, `P2206 N0`, `G2202 N0 V`, `G2206 B`, `M2220/M2221 B`.
  Internally the firmware works in `B−90` (`uarm_protocol.c:292,392,834,860`), and so does `@3 B`.
- Cartesian X/Y/Z: effector frame, i.e. arm frame plus mode offsets (`coord_arm2effect`, `uarm_coord_convert.c:201-220`).
- Polar: S = √(x²+y²), R = base angle in user frame, H = z.
- F: joint-space vector speed, where 1 unit = 6400 steps = 80°. G1/G2201/G2204/G2205 clamp F to 100 and
  overwrite acceleration.
- Joint limits (`uarm_coord_convert.c:56-63`, `is_angle_legal :77-101`): L −20…135.6, R −20…119.9,
  180−L−R in 10…151, base (−90 frame) −90…90.

---

## 6. Command reference

Legend: **A** = async (P/S), **S** = sync (G/M). **Risk:** `E` = writes EEPROM, `C` = calibration, `IO` = raw pin
access, `MV` = moves motors. The **Host** column shows which libswiftpro (L) API and/or ROS2 driver (R) uses it.

### 6.1 G — motion (sync)

| Cmd | Syntax (after space-stripping) | Reply | Behaviour (VERIFIED) | Host | Risk | Decision |
|---|---|---|---|---|---|---|
| G0 | `G0X<f>Y<f>Z<f>F<f>` (Grbl parser; any subset of words) | ok / E22 / E20 / *(empty)* | Absolute effector XYZ. **One joint-space planner block** (path not straight). F modal. Missing axes from the parser position  | L `set_position`, R | MV | port; change  |
| G1 | `G1X…Y…Z…F…` | same | Cartesian-linear: segments = max(10, dist·30/F); F clamped ≤ 100; overwrites acceleration; partial move before E22 | L (option) | MV | port; change  |
| G2004 | `G2004P<int ms>` | ok / E21 | Sets a delay; the **next** sync command waits for it. Starts when dequeued, not after motion. 0.96 ms tick | none | | port;  |
| G2201 | `G2201S<f>R<f>H<f>F<f>` | ok / E21 / E22 / **ok on bad number** | Polar absolute → Cartesian, G1-style segmented (mode 1) | L `set_polar`, R | MV | port; change  |
| G2202 | `G2202N<0-3>V<f>F<f>` | ok / E21 / E22 | N0 base (V−90), N1 L, N2 R: single-joint move as **one joint-space block** (mode 0). Starts from the step-tally angles of the other joints. N3: wrist servo (or stepper effector), immediate, no F used; V 0–360 accepted, servo wraps  | L `set_servo_angle`, `set_wrist`, R | MV | port; change . N3 in modes 4/5: the effector stepper, 0–360 (`E21` outside), `E26` until an origin search succeeds; the reply waits for the move |
| G2204 | `G2204X<f>Y<f>Z<f>F<f>` | as G2201 | Relative Cartesian from the step-tally position; segmented | L `set_position(relative)`, R | MV | port;  |
| G2205 | `G2205S<f>R<f>H<f>F<f>` | as G2201 | Relative polar; segmented | L `set_polar(relative)`, R | MV | port;  |
| G2206 | `G2206B<f>L<f>R<f>F<f>` | as G2202 | Absolute joints (B in user frame); one joint-space block | L `set_joint_angles`, R | MV | port;  |
| G2/G3, G4, G10, G17-19, G20/21, G28/30, G38.x, G40, G43.1/49, G53, G54-59, G61, G80, G90/91, G92, G93/94 | Grbl | | Grbl semantics, several harmful  | none | MV/E | **drop** |

Everything after the G22xx number must match the `sscanf` pattern exactly: fixed order, no extra words. Numbers use
`[0-9+-.]`. Extra trailing text is ignored (VERIFIED: `sscanf` stops matching and the remainder is not checked).

### 6.2 S — system (async)

| Cmd | Syntax | Reply | Behaviour | Host | Decision |
|---|---|---|---|---|---|
| S1000 | `S1000V0` / `S1000V1` | ok / E21 | V0 = feed hold (decelerate, HOLD; sync replies wait), V1 = cycle start (resume) | L `pause/resume_motion`, R | port |
| S1100 | `S1100` (rest ignored) | ok | `mc_reset()`: during motion → ALARM wedge ; when idle → reinit and drop pending RX | L `motion_reset`, R (3 call sites) | **change** |

### 6.3 M — settings and peripherals (sync)

| Cmd | Syntax | Reply | Behaviour (VERIFIED) | Host | Risk | Decision |
|---|---|---|---|---|---|---|
| M17 | `M17` | ok | Enable all 3 drivers, `end_effector_init`, bits = 0x0F, `update_motor_position()` (re-read encoders, zero tally) | L `set_servo_attach(-1)`, `reset`, R | | port; change|
| M18 | `M18` | ok | = M2019 | none | | port |
| M204 | `M204A<f>` | ok / E21 | Stores `$120-$122` = A (·3600) in EEPROM; negative A → ok, ignored ; overridden by next G1  | L `set_acceleration`, R | E | port; |
| M2019 | `M2019` | ok | Disable all 3 drivers, `end_effector_deinit` (wrist keeps PWM), bits = 0 | L `set_servo_detach(-1)`, R | | port; change |
| M2120 | `M2120V<f seconds>` | ok / E21 | Start `@3` every V·1000 "ms" ticks; V ≤ 0 stops | L `set_report_interval`, R | | port: exact 1 ms tick ; V ≤ 0 stops (faithful); 0 < V < 0.01 s or V > 3600 s → `E21` ; the first report one period after the command |
| M2121 | `M2121` | ok | Stop `@3` | none | | port |
| M2122 | `M2122V0/1` | ok / E21 | `@9` on/off (and clears the pending flag) | L `set_motion_report`, R | | port |
| M2123 | `M2123V0/1` | ok / E21 | Closed-loop auto-correction  | none | MV | **drop**  |
| M2200 | `M2200` | `ok V0/1` | 1 if `sys.state != IDLE` **at execution time** (reply printed at idle) | L `get_is_moving` | | port |
| M2201 | `M2201N<0-3>` | ok / E21 | N0–2: enable **all** drivers , set bit (N0→0x04, N1→0x01, N2→0x02); N3: effector init (0x08); then `update_motor_position()` | L `set_servo_attach(id)`, R | | port; change |
| M2202 | `M2202N<0-3>` | ok / E21 | Disable one driver / effector deinit; clear bit | L `set_servo_detach(id)`, R | | port; change |
| M2203 | `M2203N<0-3>` | `ok V0/1` | Reads bits with mismatched mapping N0→0x01 (L), N1→0x02 (R), N2→0x04 (base)  | L `get_servo_attach`, R | | **change** |
| M2205 | `M2205VUB<digits.>` | ok / E21 | Store 12-byte UID to EEPROM 1002 (P2205) | none | E | **keep** . Built: `VUB` + 1–12 characters of any kind (as the line assembler leaves them: no spaces, upper case), NUL-padded; else `E21`  |
| M2210 | `M2210F<f>T<int>` | ok / E21 | Buzzer; T counts half-periods ; blocks the sync queue while sounding | L `set_buzzer`, R | | port; change |
| M2211 | `M2211N1A<int>T<1,2,4>` | `ok V…` / E21 | Read external I2C EEPROM 0xA0 (bit-banged PB6/PB7) | none | | **drop**  |
| M2212 | `M2212N1A<int>T<1,2,4>V<f>` | ok / E21 | Write external EEPROM | none | E | **drop**  |
| M2213 | `M2213V0/1` | ok | jss.7: no-op. V4.0.5 doc: base-button default function on/off | none | | **port**: V0 = buttons report `@4`; V1 = default function, which in this port is none |
| M2215 | `M2215` | ok | Restore all Grbl settings, re-detect hardware version | none | E | **drop**  |
| M2220 | `M2220X<f>Y<f>Z<f>` | `ok B%s L%s R%s` | Inverse kinematics (effector XYZ → angles, B user frame); no legality check | L `coord_to_angles` | | port |
| M2221 | `M2221B<f>L<f>R<f>` | `ok X%s Y%s Z%s` | Forward kinematics | L `angles_to_coord` | | port |
| M2222 | `M2222X<f>Y<f>Z<f>P<0/1>` | `ok V1/V0` / E21 | Reachability; P1 = polar input | L `is_reachable` (P0) | | port |
| M2231 | `M2231V0/1` | ok / E21 | Pump on/off; off → valve sequence  | L `set_pump`, `reset`, R | | port |
| M2232 | `M2232V0/1` | ok / E21 | Gripper V1 = catch (PC7 low), V0 = release (PC7 high) | L `set_gripper`, `reset`, R | | port |
| M2233 | `M2233V0/1` | ok / E21 | Laser PH5+PH6 high/low | none | | port. Built; in modes 4/5 `E20` (PH5/PH6 are the effector stepper's) |
| M2234 | any | ok | no-op (BLE enable) | none | | drop |
| M2240 | `M2240N<0-69>V<0/1>` | ok / E21 | `digitalWrite` any pin  | L `set_digital_output`, R | IO | change . Built: Arduino Mega numbering (framework-arduino-avr 5.2.0 `variants/mega`); firmware-owned pins `E21`  |
| M2241 | `M2241N<0-69>V<0/1>` | ok / E21 | `pinMode` OUTPUT/INPUT | L `set_digital_direction`, R | IO | change . Built: V0 = input with pull-up off, as `pinMode(INPUT)`; firmware-owned pins `E21` |
| M2245, M2304–M2307 | | E20 | listed but fall through to Grbl → E20 | none | | drop (same reply) |
| M2400 | `M2400S<0-8>` | ok / E21 | Set work mode, load default offsets, save to EEPROM 900/910/920; S4/S5 start the origin search ; S7 loads the user offsets 930/940; S8 = test mode | L `set_mode`, R | E | port; . Built: offsets from `uarm_common.h:116-138`; S7 falls back to the mode 0 defaults if 930/940 are unusable (R-7); the wrist keeps its state between servo modes, 4/5 release it and leaving 4/5 re-attaches it; S4/S5 reply after the origin search and the move to 90°, `E26` if the switch isn't found  |
| M2401 | `M2401` | ok | Store the current raw encoder values as the reference ("point B") in EEPROM 818/820 | none | E C | **keep** . Built: three median readings; `E26` and nothing written if one fails; the tally of every locked joint is re-synced to the new reference  |
| M2410 | `M2410` | ok | height offset = current encoder Z; save to 910 and 930/940 | none | E C | **keep** . Built: from three encoder readings, `E26` if one fails |
| M2411 | `M2411S<f>` | ok / E21 | front offset; save | none | E | **keep** . Built: also saved as the user offset (940); a value outside ±1000 or NaN `E21` (R-7) |
| M2412 | `M2412V<f>` | ok / E21 | effector angle offset; save 3202; restarts the origin search | none | E | **keep** . Built: −360…360, else `E21`; in modes 4/5 the reply comes after the new search (`E26` if not found) |
| M2413 | `M2413H<f>` | ok / E21 | height offset; save | none | E | **keep** . Built: also saved as the user offset (930); outside ±1000 or NaN `E21` (R-7) |
| M2500 | `M2500` | ok | Blocking open-loop auto-calibration sweep  | none | MV E C | **drop** |
| M0-M9, M30 | Grbl | | program flow, spindle, coolant | none | | **drop** |

### 6.4 P — queries (async)

| Cmd | Syntax | Reply (exact) | Source of data | Host | Decision |
|---|---|---|---|---|---|
| P2200 | `P2200` | `ok B%s L%s R%s` | encoders (~12.6 ms), B user frame | L `get_joint_angles` | port; change  |
| P2201 | `P2201` | `ok VSwiftPro` | const | L+R `get_device_info` | port |
| P2202 | `P2202` | `ok V3.3.1` (V3.3.0/V3.3.2/V0.0.0 from PJ3–PJ6 straps) | `uarm_peripheral.c:629-650` | L+R | port |
| P2203 | `P2203` | `ok V4.9.1-jss.7` | `uarm_swift.h:17` | L+R | **change: `ok V5.0.0-pp.1` ** |
| P2204 | `P2204` | `ok V4.0.5` | const | L+R | port |
| P2205 | `P2205` | `ok V<12 chars from EEPROM 1002>` | EEPROM | L+R | port |
| P2206 | `P2206N<0-3>` | `ok V%s` | N0–2 encoder (B user frame), N3 wrist commanded angle | L `get_servo_angle` | port;. In modes 4/5 N3 is the effector stepper's angle |
| P2220 | `P2220` | `ok X%s Y%s Z%s` | encoders → FK → effector frame | L `get_position` | port;  |
| P2221 | `P2221` | `ok S%s R%s H%s` | encoders | L `get_polar` | port;  |
| P2231 | `P2231` | `ok V0/1/2` | PG4 state + ADC A9 (<10 → 0, ≤70 → 2, else 1) | L+R `get_pump_status` | port |
| P2232 | `P2232` | `ok V0/1/2` | PC7 high → 0; else PA0 high → 1, low → 2 | L+R `get_gripper_status` | port |
| P2233 | `P2233` | `ok V0/1` | PG3 limit switch | none | port |
| P2234 | `P2234` | `ok V0/1` | ADC A5 > 100 | none | port |
| P2235 | `P2235` | `ok V0/1` | laser PH5 | none | port if laser kept |
| P2240 | `P2240N<0-69>` | `ok V0/1` | `digitalRead` | L+R `get_digital` | change . Built: never touches a timer  |
| P2241 | `P2241N<0-15>` | `ok V<0-1023>` | `analogRead` | L+R `get_analog` | port (read-only; does not change pin direction) |
| P2242 | `P2242` | `ok B%d L%d R%d` | stored reference values | none | port (cheap, useful for calibration audit) |
| P2243 | `P2243` | `ok B%d L%d R%d` | raw encoder values | none | port |
| P2244 | `P2244` | `ok V0` or `E26 V<mask>` (4 = L, 2 = R, 1 = base) | status reg 0x0B & 0x18 == 0x18 → "fail" | L+R `get_encoder_status` | port;  |
| P2400 | `P2400` | `ok V<mode>` | RAM | L `get_mode` | port |

All `%s` floats are `dtostrf(v, 5, 4, …)`: at least 5 wide, 4 decimals (e.g. `90.0000`, `-1.2500`), no plus sign.
This is byte-critical for the host; the port must reproduce avr-libc's `dtostrf` rounding exactly (Phase 1 golden traces).

### 6.5 `$` — Grbl system (immediate, unsequenced)
`$`, `$$`, `$#`, `$G`, `$I`, `$N`, `$x=val`, `$Nx=line`, `$C`, `$X`, `$H`, `$RST=…`. **Drop all** : the port has
no Grbl settings, homing, startup lines or check mode. libswiftpro's `send_raw` cannot reach them .

---

## 7. Peripheral behaviour summary (for byte-compatible ports)

- **Wrist** (`G2202 N3`): `duty_ms = (180−V)/180·2 + 0.5`; Timer4 phase- and frequency-correct PWM, 50 Hz, on OC4A/PH3.
  P2206 N3 / `@3 R` = `180 − (duty_ms − 0.5)/2·180`. Default after init: 1.51 ms (89.1°).
- **Pump** (`M2231`): on → PG4 = 1, PG5 = 0. Off (only if it was on) → PG4 = 0, PG5 = 1. Then, if the PJ3–6 straps are
  non-zero (V3.3.1/V3.3.2), stop with PG5 = 1; on V3.3.0 boards, pulse PG5 10× at 50 ms (blocking, ).
- **Gripper** (`M2232`): PC7 output; V1 → low (catch), V0 → high (release).
- **Laser** (`M2233`): PH5 and PH6 both high/low.
- **Buzzer** (`M2210`): PL5 square wave from a Timer2 ISR toggle.

## 8. Dropped (summary)

Grbl G/M/`$` words beyond G0/G1 ; realtime `? ! ~ @` (`^X` is kept as emergency stop); M2211/M2212/M2234/M2245/
M2304–M2307; M2500 ; reset-cause boot line: bootloader clears `MCUSR`); M2215 ; M2123 closed-loop correction ; UARM_MINI/UARM_2500 variants; the external I2C EEPROM driver; encoder calibration maps.

## 9. Not in the firmware (VERIFIED by grep)

No LED effects, no buttons (`@4` in the doc is "not supported"), no temperature or 3D-print support (`@7`), no
camera or vision support, and no Bluetooth control beyond sharing the serial stream on another UART. The vision
kit's suction cup and two-finger gripper are driven by M2231/M2232 like the standard ones. How the kit's camera and
Arduino connected to the arm is not in this source (UNVERIFIED; possibly USART1–3, ).

## 10. Additions in this port (approved, not in jss.7)

Exact byte formats are fixed in Phase 1 (golden traces), within these decisions.

| Item | Form | Decision |
|---|---|---|
| Lost-step / collision event | `@8 N<joint> V<error deg>`; the moving and queued commands get `$N E25` |  (on by default, 1.0°, 2 consecutive). Built: `@8 N<0 B\|1 L\|2 R> V<tally − encoder, dtostrf 5,4>`; the arm brakes as `S1100`, then the tally of every locked joint is re-read from its encoder . Checked while any joint moves and once when the tally stops (R-13). `M2125 V0` turns the check off, `V1` on; on after every boot, not saved; `ok` / `E21`  |
| Encoder detail query | `P2246` → `ok` + per-joint MD/ML/MH, AGC, magnitude. `P2244` keeps its format and now also flags no-ACK and MD = 0 | |
| Base buttons | `@4 N<0 Menu\|1 Play> V<1 click\|2 long>` after `M2213 V0`; `P2236` dropped |  |
| Streaming replies | `M2124 V1`: a motion command's `$N ok` is sent when accepted into the planner. `V0` (boot default) = faithful idle-gated replies |
| Faster `@3` | `M2120` accepts periods down to 10 ms (R-14). Built: `@3 X Y Z R B L AR M<j><deg> A<ms> T<ms> S<n>`: `M<j>` is one joint measured by its encoder, in rotation B, L, R, in the frame of its tally field (B in the −90 frame), `A` the age of that reading; `M<j>-` and no `A` if there is no good reading. `T` = ms since boot, `S` = report counter since boot (0-65535, wrapping), which also counts skipped reports |
