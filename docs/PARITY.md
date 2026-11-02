# PARITY — every upstream feature, and what SwiftPro++ does with it

Upstream clones (read-only): `~/dev/third-party/uarm/uarm-V4.9.0` (tag V4.9.0, 2020-08-14),
`uarm-V4.10.0` (tag V4.10.0, 2022-06-10), `uarm-v3.1.18-dev` (2017-07-25, Marlin). Citations are `file:line` in
`src/` of the named clone. VERIFIED = read in code; UNVERIFIED = inferred, needs a test.

Verdicts: **port** (do what upstream does), **port-changed** (keep the meaning, fix the implementation),
**have** (SwiftPro++ already does it), **drop** (needs your approval), **n/a** (only for another arm, or no effect).

---

## A0. V4.9.0 → V4.10.0, every change

Size of the diff, ignoring whitespace and CRLF (`git diff -w --ignore-cr-at-eol V4.9.0 V4.10.0 -- src`): 23 files,
+2,002 / −137 lines. Raw it shows +9,076 / −6,051 because V4.10.0 converted `uarm_protocol.c` from CRLF to LF.
Of the 2,002: 366 (`uarm_iic.c`) are compiled out, 39 (`src/README.md`) are a Gitee template, and about 1,000 are the
three jog commands. `gcode.c`, `planner.c`, `limits.c`, `uarm_debug.h` are byte-identical apart from line endings.

The V4.2.0 protocol PDF adds exactly three things to V4.0.5: G2207, G2208 ("Support v4.9.6 or later") and
`E27 Motor disenable`. G2209, P2236, M2124, M2231 V2 and modes 8–11 are in the code but not in any document.

### A0.1 Motion and stop

| # | Change | Where (V4.10.0) | Notes, bugs found | Verdict |
|---|---|---|---|---|
| 1 | Move refused unless base, L and R are all locked: `(motor_state_bits & 0x07) != 0x07` → `E27`. The check comes before any planning, so the step tally doesn't change | `motion_control.c:69`, `uarm_protocol.c:119-124`, `uarm_protocol.h:19` | Strict: also refuses a move that wouldn't turn the released joint. G2209 bypasses it (calls the planner directly). `report.c:95` adds a Grbl-side `STATUS_ENABLE_ERROR = 52` that nothing returns: harmless dead code | **port-changed (A1)**. We already refuse before the tally moves, but reply `E25`, and only when a joint that would turn is released (`motion.cpp:88-94`, `commands.cpp:568`) |
| 2 | `mc_line` with `feed_rate == 0` replies `ok` and moves nothing | `motion_control.c:93, 177-183` | G1 F0 still fails in the parser (`gcode.c:691`, unchanged), so this affects G0 F0 and the uArm moves G2201–G2208 with F0. In V4.9.0, F0 was raised to `MINIMUM_FEED_RATE` 1 mm/min: a move that took hours. Side effect: gcode.c still copies the target into `gc_state.position` (`gcode.c:1054`) | **A3, your call.** We reply `E21` to F ≤ 0 (`motion.cpp:86`, SP-09) |
| 3 | `MINIMUM_FEED_RATE` 1.0 → 0.0 | `config.h:295` | Removes the planner's floor, so F0.0001 now plans an almost endless move instead of a slow one. Has no use once #2 catches F0 | **drop** (we have no such floor and refuse F ≤ 0) |
| 4 | Ctrl-X stop no longer disables the drivers: `mc_reset` calls the new `st_go_idle_2`, which stops Timer1 and leaves the enable pins alone | `motion_control.c:469`, `stepper.c:256-287` | The stop is instant, with no deceleration, so steps can be lost: hence the encoder re-sync in #5 | **have** (our stop brakes to a stop with the drivers locked, `motion.cpp:325-333`) |
| 5 | Ctrl-X in the USART0 RX ISR now also: `plan_reset()`, re-reads the encoders (`update_motor_position`), clears the alarm (`sys_rt_exec_alarm = 0`), forces `STATE_IDLE`, and prints `ok\n` (only if M2123 is off) | `serial.c:364-385` | **Bugs:** (a) the encoder read is about 10 ms of bit-banged I2C inside the ISR: USART0 bytes arriving meanwhile are lost (2-byte FIFO at 11.5 bytes/ms); (b) `update_motor_position` starts with `memset(&sys, 0, …)` (`uarm_peripheral.c:1081`), which clears `print_reset_flag` and `rest_flag` right after they are set, so #8 and the M2123 branch do nothing; (c) `ok` comes from inside the ISR, with no `$N`; (d) USART1–3 still run the old `mc_reset()` alone (`serial.c:422, 457, 492`). Clearing the alarm fixes the ALARM wedge for Ctrl-X, but `S1100` still calls bare `mc_reset()` (`uarm_protocol.c:1500-1502`), so S1100 during motion still wedges in V4.10.0 | **port-changed (A2):** the `ok` reply and the encoder re-sync once stopped, done in the main loop, on every port |
| 6 | `~` (cycle start) prints `ok\n`; `!` (feed hold) prints `ok @9 V0\n`, at once, before the arm has slowed | `serial.c:350-358` | **Bug:** printed from the RX ISR; `serial_write` busy-waits for TX room (`serial.c:160-163`) and the TX interrupt can't run inside the RX ISR, so a `!` or `~` that arrives while the 128-byte TX ring is full hangs the firmware (VERIFIED in code; for example while `@3` streams). `@9` shares a line with `ok`, so a host that reads events at line start doesn't see it (UNVERIFIED which hosts rely on it). USART0 only | **port-changed (A2):** same bytes, sent from the main loop; `@9` text as upstream (proposal in A2) |
| 7 | Main loop: if `restart_flag` (set only by G2209) and idle, re-read the encoders | `protocol.c:174-178` | Part of G2209 (#14) | with G2209 |
| 8 | `print_reset_flag`: the command parsed right after a Ctrl-X gets no result code | `uarm_protocol.c:159`, `protocol.c:179-180`, `main.c:88` | Dead in practice: cleared by the `memset` of #5(b) before it's read. `main.c:89-93` prints `ok` after a reset if `uarm.reset_flag`, which nothing ever sets (only cleared, `uarm_swift.c:26`): dead too. So "ok after reset" comes only from #5 | **drop** (dead code; our aborted commands get `E25`, Q-46) |
| 9 | `check_motor_positon` counter moved into `sys.check_cnt`; M2123 V1 now re-reads the encoders when switched on | `uarm_peripheral.c:1040`, `uarm_protocol.c:1617` | Only matters with M2123, which we dropped (Q-19) | n/a |
| 10 | `stop_*_angle` fields, `st_reset2()`, `update_motor_position2()`, `uarm_device` enum, `check_flag`, `syn_pack_remain` extern, extra includes in `serial.c` | `uarm_common.h`, `stepper.c:555`, `uarm_peripheral.c:1114`, `serial.c:22-45` | Declared but never used (VERIFIED by grep) | n/a |

### A0.2 New commands

| # | Command | Where | What it does (VERIFIED) | Bugs found | Verdict |
|---|---|---|---|---|---|
| 11 | `G2207 N<0-2> F<signed>` Cartesian jog | `uarm_protocol.c:408-1220`, dispatch `:1466` | N0 X, N1 Y, N2 Z. The sign of F is the direction, \|F\| the speed. Starts from the encoder position, works out the farthest reachable point along that axis (first at the arm-to-arm limit 151−2 or 10+1, then at R's and L's limits ±1), and sends a **G1-style linear move** there (`mc_line(1, …)`, so \|F\| is clamped to 100 and acceleration rewritten). The host stops it with `!`/S1000 V0 or Ctrl-X/S1100. If the arm is already within 0.7 mm of the edge it adds 20 or 50 mm to the target **on purpose** so that `mc_line` replies `E22`. F0 → `ok`, no move | `abs()` on floats is the integer `abs`, so "within 0.7" means "within 1.0". The other two axes come from the step tally, the jogged one from the encoders. Linear segments that fall outside the limits are skipped (`continue`, `motion_control.c:145`), not refused. About 800 lines of copy-paste per direction; Y and Z branches differ in small ways that look accidental (e.g. N1 tries L = 26+1, `:768`) | **port-changed (A4):** same syntax and replies; reimplement as "linear move to the workspace boundary along the axis", computed by our IK and legality check |
| 12 | `G2208 N<0-2> F<signed>` joint jog | `uarm_protocol.c:1222-1329` | N0 base → ±90 (−90 frame); N1 L, N2 R → the value that puts the arm-to-arm angle at 10 (F > 0) or 151 + 0.4 (F < 0), clamped to that joint's max −1 and truncated to an integer. One joint-space move (`mc_line(0, …)`). Already at the target (< 0.7°) → target set to 200° so the reply is `E22`. F0 → `ok`, no move | `abs()` as #11. N1 F<0 clamps L with `ARMB_MIN_ANGLE` (R's minimum): the same value (−4) on the Pro, so harmless here. Starts from encoder angles | **port-changed (A4)** |
| 13 | `G2209 F<f>` (undocumented) | `uarm_protocol.c:1331-1440` | For each joint whose **encoder** angle is outside its own limit, move it to that limit ∓2°; the others stay. Then re-reads the encoders at once and again when idle (#7). Replies `ok` | Bypasses `mc_line`: no lock check, no arm-to-arm check. For the out-of-range joint it gives the planner a **change** in degrees where an absolute position is expected, which is right only if that joint's tally is 0 (right after M17 or boot). The immediate `update_motor_position()` clears `sys` (including `sys.state`) while the move is queued. No F validation | **port-changed (A4):** "bring every joint inside its limits (including arm-to-arm), then re-sync"; joint-space move from the measured angles |
| 14 | `P2236 N<0/1>` button state | `uarm_protocol.c:2576-2589`, `:2741`, `uarm_peripheral.c:931-962` | N0 → PE7 (Menu), N1 → PE6 (Play), pull-ups on; reply `ok V1` = released (high), `V0` = pressed. `button_init()` at boot (`uarm_swift.c:14`) | sscanf `%d` accepts e.g. `N01`. Reply polarity is the raw pin level | **port** (we dropped it, SP-44/Q-31; our `@4` already debounces PE6/PE7) |
| 15 | `M2124 V0/V1`: beep (260 ms, 1 kHz) when the position is re-synced | `uarm_protocol.c:1622-1632`, `:2253-2258`, `uarm_peripheral.c:1059-1060` | Default off. Only fires inside `check_motor_positon`, i.e. with M2123 V1 | — | **port-changed (A5):** beep on our own re-syncs (after `@8`, after a stop); move our streaming switch to a free number (A5) |
| 16 | `M2231 V2`: pump "suction" state | `uarm_protocol.c:1971`, `:2338`, `uarm_peripheral.c:764-790` | On the Mini-2 board (straps = 0x10): reverses PG4/PG5. On every other board, including the Pro: PG4 low, PG5 high, PF3 low, the same pins as the valve-open part of `pump_off`, without the valve timer. State `PUMP_STATE_SUCTION` | See #20 for PF3 | **your call (A4 list):** on the Pro it is "open the valve and leave it open"; port as such, or drop |
| 17 | M2400 modes: 8 claw jaw, 9 steering gear, 10 clamp, test moved 8 → 11 | `uarm_common.h:51-58`, `:165-172`, `uarm_protocol.c:2079-2107` | Pro offsets (`#else` branch, the Pro build): 8 = H0 F0, 9 = 10 = 74.55/56.65 (the same as mode 0) | The range check is `line[1] <= '11'`, a multi-character constant (12593), and the mode is `line[1]-'0'`, one digit: `S10` and `S11` run as `S1` (laser), so **modes 10 and 11 cannot be selected**; any character from `0` up, e.g. `S:` or `SA`, is accepted (`uarm_protocol.c:2392`) | **A6** (default: 8, 9, 10 as upstream on the Pro, 11 = test, and S10/S11 actually work) |
| 18 | `M2205` UID scanset `[0-9.]` → `[0-F.]` | `uarm_protocol.c:1748` | `0-F` is the ASCII range `0`…`F`, which includes `: ; < = > ? @`. Still unbounded into `sn_num[13]` | **have** (we accept 1–12 of any character, SP-56/Q-62) |

### A0.3 Peripherals and timing

| # | Change | Where | Notes, bugs found | Verdict |
|---|---|---|---|---|
| 19 | Buzzer: `M2210 T` is now ms at any frequency (half-periods = T·2·F/1000 − 1); Timer2 prescaler chosen by frequency (< 500 Hz: /1024, else /128; the code calls them 2048 and 256, which halves the count so that one interrupt = one half-period); counter widened to 32 bits | `uarm_peripheral.c:181-209`, `uarm_timer.c:43-56` | F0 divides by zero; below about 31 Hz the reload value wraps. Resolution: 1 kHz comes out at 1.008 kHz | **have** (SP-24: hardware toggle on OC5C, F 16–20,000 Hz, T in ms, `buzzer.cpp:45-64`). This settles the A3 question: done |
| 20 | Pump drives **PF3** as an output: high with `M2231 V1`, low on off/suction | `uarm_peripheral.c:740, 755, 769, 785, 798, 802, 855, 862` | On every board, not just the Mini-2. On the Pro board, PF3 is `SER2_FEEDBACK`/ADC3 (your schematic notes): V4.10.0 drives the servo-feedback net high whenever the pump is on (UNVERIFIED: is there a series resistor?) | **drop on the Pro**; record in ISSUES.md as an upstream hazard |
| 21 | Mini-2 board detection: strap mask gets PJ2; straps = 0x10 → "V3.3.3" (or "V3.3.4", unreachable), and the Mini-2 pump polarity | `uarm_peripheral.c:624-653`, `:742-750`, `:804-818` | `PINJ & HARD_MASK == (1<<2)` parses as `PINJ & (HARD_MASK == 4)` = `PINJ & 0` = false, so V3.3.4 is unreachable. Adding PJ2 to the mask changes the Pro's own detection if PJ2 reads high (V3.3.1 would become V0.0.0) (UNVERIFIED: what PJ2 reads on the Pro) | **drop** (Mini-only). Hardware test: read PINJ once to confirm the Pro straps |
| 22 | Mini-2 `pump_off`: blocking `delay_ms(50)` | `uarm_peripheral.c:810` | Mini-only | n/a |
| 23 | `get_pump_status` (P2231) prints `value:<n>      value2:<n>` debug text on the command channel before every reply; new `get_pump_adc()` | `uarm_peripheral.c:896-922` | `while (!(ADCSRA & 1<<ADIF)) pump_adc = ADC;` has no `;` after the loop, so it returns the result of the previous conversion. It rewrites ADMUX/ADCSRA under `analogRead`'s feet | **drop** (we keep the V4.9.0 reply, SP-14) |
| 24 | Encoder I2C: a faster timing variant (1 µs instead of 2–5 µs) | `uarm_iic.c` (366 lines) | All of it inside `#ifdef sort_i2c`, and `sort_i2c` is defined nowhere (`uarm_iic.c:69` is commented out). **No compiled change** | n/a (our encoder driver is our own) |
| 25 | `uarm_angle.c` point-B angles 20.0/92.0 → 19.98/95.89 | `uarm_angle.c:25-26` | Inside `#if defined(UARM_MINI)`. The Pro values (32.758930/88.980537, `:35-36`) are unchanged and match ours (`encoder.cpp:20`) | n/a |
| 26 | `uarm_coord_convert.*`: the arm constants moved from `.c` to `.h` (so G2207 can use them); Mini limits changed (R min 0 → 1, arm-to-arm 10…151 → 18…137) | `uarm_coord_convert.h:8-69` | Pro values unchanged: L −4…135.6, R −4…119.9, arm-to-arm 10…151, base ±90 (`:62-68`). Kinematics code unchanged | n/a (Part B uses these) |
| 27 | Version strings `V4.9.0` → `V4.10.0`; API string still `V4.0.5` | `uarm_swift.h:17-18` | The V4.2.0 PDF is not reflected in `P2204` | n/a (we report our own, Q-2) |
| 28 | `src/README.md`: an untranslated Gitee project template | | | n/a |

### A0.4 Summary for the next steps

- **Already equivalent or better in SwiftPro++:** #4 (drivers stay locked on stop), #18 (UID), #19 (buzzer; A3's
  M2210 question is answered: done).
- **To port, changed:** #1 E27 (A1), #5/#6 stop and realtime replies (A2), #11–#13 G2207–G2209 (A4), #14 P2236 (A4),
  #15 M2124 (A5), #17 modes (A6).
- **Decisions for you:** #2 G0 F0 (`ok` and no move, or our `E21`), #16 M2231 V2 on the Pro.
- **Upstream hazards to record in ISSUES.md, not to port:** #5(a) the encoder read in the ISR, #6 the possible TX
  deadlock in the ISR, #17 S10/S11 unreachable, #20 PF3 driven on the Pro, #23 debug text in P2231, #13 G2209's
  relative/absolute mix-up.

---

## Work items done

Costs are measured on the whole firmware image (avr-gcc 15.2.0, `cmake --build build/avr`); stack is the simavr
high-water mark of `tools/simavr/session.txt`. A1, A2 and A5 were built together, so they share one measurement:
**flash 45,990 → 46,654 B (+664), static RAM 4,088 → 4,107 B (+19), stack 531 → 539 B (+8).**
A7 and A9 step 1 were built together: **flash 49,752 → 51,380 B (+1,628), static RAM 4,137 → 4,161 B (+24), stack
545 B (unchanged).**
A4 was built on its own: **flash 46,654 → 49,700 B (+3,046), static RAM 4,107 → 4,137 B (+30), stack 539 → 545 B
(+6).** Of the flash, about 750 B is `G2209`, 500 B each the two jogs (measured by stubbing each out); the rest is the
parsing, the shared limit table with its inset and the extra line phase. Flash use is 19.6% of the 253,952 B below the
bootloader.

| Item | Source | Syntax and reply | Decision | Tests | Hardware test |
|---|---|---|---|---|---|
| A1 `E27` | V4.10.0 `motion_control.c:69`, `uarm_protocol.c:119-124` | any move (G0, G1, G2201, G2202 N0-2, G2204-G2206) while base, L or R is released → `$N E27`; nothing is planned, the tally doesn't change. Checked after F (`E21`) and before reach (`E22`), as upstream | port (Q-70) | `13-lost-steps.txt` (joint and linear move), simavr session #31 | stage 4, step 1 |
| A2 `!` `~` | V4.10.0 `serial.c:350-358` | `!` anywhere in the stream = feed hold, reply `ok @9 V0`; `~` = resume, reply `ok`. No `$N`; to the port that received the byte; repeats before the main loop takes them count once; of `!` and `~` the last one wins | port-changed: sent from the main loop, not the RX ISR (Q-71) | `03-framing.txt`, `06-stop.txt`, `unit_serial.cpp`, simavr session #51 | stage 4, step 2 |
| A2 Ctrl-X | V4.10.0 `serial.c:364-385` | stop as `S1100` (every queued command `E25`), reply `ok` without `$N` | port-changed (Q-71; replaces Q-48's "no reply") | `06-stop.txt`, simavr session #50 | stage 4, step 3 |
| A2 re-sync | V4.10.0 `serial.c:373, 379`, `protocol.c:174-178` | after every stop (S1100, Ctrl-X, lost steps), once at rest, each locked joint is re-read from its encoder; a failed read leaves that joint's tally. Sync replies wait for it (about 11 ms) | port-changed: in the main loop once stopped, not inside the ISR (Q-71) | `13-lost-steps.txt` (S1100 recovers a stalled L), `06-stop.txt` | stage 4, step 3 |
| A5 `M2124` | V4.10.0 `uarm_protocol.c:1622-1632, 2253-2258`, `uarm_peripheral.c:1046-1060` | `M2124 V0/V1` → `ok` / `E21`; V1 = beep 260 ms at 1 kHz when a re-sync after a stop moves a joint's tally by more than 0.5°. Off after boot, not saved | port-changed: upstream beeps only in the M2123 auto-correction, which we dropped (Q-72) | `13-lost-steps.txt`, `unit_watch.cpp` (timing) | stage 4, step 4 |
| A4 `G2207` | V4.10.0 `uarm_protocol.c:408-1220`, dispatch `:1466` | `G2207 N<0 X\|1 Y\|2 Z> F<signed>` → `ok` when the jog ends; `E21` bad N, F missing or 0; `E22` edge less than 0.7 mm away; `E27` a joint released. Sign of F = direction, \|F\| = speed (Q-51 units), F not kept | port-changed (Q-76): our IK walks the line out to 1° inside every limit (`motion.cpp` `search_step`), then the line is checked and run like `G1`; the original's per-axis formulas (#11) are not ported | `17-jog.txt`, simavr session #53 (search on the AVR, ended by `S1100`) | stage 4, step 5 |
| A4 `G2208` | V4.10.0 `uarm_protocol.c:1222-1329` | `G2208 N<0 B\|1 L\|2 R> F<signed>` → as `G2207`, 0.7° threshold | port-changed (Q-76): every limit 1° inside (the original: 1°, elbow 0–0.4°, base exactly ±90), no truncation to an integer; with the measured limits (Q-83), the L jog follows R's range as L changes | `17-jog.txt` | stage 4, step 5 |
| A4 `G2209` | V4.10.0 `uarm_protocol.c:1331-1440`, `protocol.c:174-178` | `G2209 [F<f>]` → waits for idle, re-reads B, L, R, moves every joint outside its limit and the elbow 2° inside, in one joint-space move; `ok` (also with no move), `E26` read failed, `E27` released, `E21` bad F | port-changed (Q-76): includes the elbow (since Q-83: R into its range at the corrected L); from the measured angles (the original gave the planner a change where it wanted a position, #13) | `17-jog.txt` | stage 4, step 5 (inside the limits: `ok`, no move) |
| A4 `P2236` | V4.10.0 `uarm_protocol.c:2576-2589`, `uarm_peripheral.c:931-962` | `P2236 N<0 Menu\|1 Play>` → `ok V0` pressed, `V1` released (the pin, not debounced); other N `E21` | port (Q-76) | `17-jog.txt` | stage 4, step 6 |
| A6 modes | V4.10.0 `uarm_common.h:45-58, 141-175`, `uarm_protocol.c:2032-2110` | `M2400 S<0-11>` → `ok` / `E21`; `P2400` and the banner report 0–11. Offsets (front, height): 8 claw jaw 0, 0; 9 steering gear and 10 clamp 56.65, 74.55; 11 test 44.5, 25.49. Only the offsets differ between these modes upstream | port-changed (Q-76): S10 and S11 reach their modes (upstream ran them as S1, #17); `S1A` and the like are `E21`. **Compatibility:** a mode 8 saved by jss.7 or V4.9.x (test) loads as claw jaw. Cost: +52 B flash | `16-settings.txt` | none: `M2400` overwrites the offsets saved in EEPROM 910/920, and the traces cover the table |
| A7 (Q-80) | 3.x `Marlin_main.cpp:7642-7646`, `uArmSwift.cpp:285-332, 575-688, 1151-1240, 1454-1458`, `uArmAPI.cpp:686-730` | `G2203`; `M3`/`M5 [P]` laser in mode 1; `M2211 N0` read; `M2410 S`; `M2401 B`; `P2245`. See PROTOCOL.md §6 | as proposed below (A7 table) | `18-v3-eeprom.txt`, simavr session #58-#59 | stage 4, step 7 (read only) |
| A9 step 1 (Q-81) | V4.10.0 `uarm_protocol.c:1773-1860`, `uarm_peripheral.c:472-620`, `uarm_iic.c:53-250` | `M2211`/`M2212 N1 A<0-65524> T<1\|2\|4> [V]`: the external EEPROM at 0xA0 on PB7/PB6 (`i2c::Bus::kX`, `xeeprom.cpp`); `E26` without an ACK. PB6/PB7 leave the raw pin commands | port-changed: no blocking 5 ms per byte, ACKs checked, negative V accepted | `18-v3-eeprom.txt` (simulated chip with page wrap and write cycle), simavr session #57 (no chip: `E26` on the real pins) | stage 4, step 7: **tells whether your board has the chip** |
| A5 `M2126` | — (unused by V3.1.18, V4.9.0, V4.10.0 and both protocol PDFs) | streaming replies, as `M2124` was (Q-32) | moved (Q-72) | `09-streaming.txt` | stage 3, step 3 (script updated) |

Also decided: A3, G0 F0 stays `E21` (Q-73); A0 #16, `M2231 V2` dropped (Q-74).

## Proposals (2026-10-21)

Decided: A7 all as proposed (Q-80, built); A8 `M2500` stays dropped (Q-79); A9 step 1 built (Q-81), steps 2-3
not built, and playback waits for LIMITS.md. Apart from that, nothing in this section is built. Citations without a directory are in `uarm-v3.1.18-dev/lib/Marlin/`. The 3.x
dispatch is `Marlin_main.cpp:7610-7670` (G), `:8205-8330` (M), `:8333-8445` (P). (A plain `grep` skips that file: it
is not valid UTF-8; use `grep -a`.)

### A7. Commands in 3.x that 4.x lacks

| # | Command | 3.x behaviour | 4.x and ours | Proposal | Cost (estimate) |
|---|---|---|---|---|---|
| 1 | `G2203` | waits until every queued move has finished, then `ok` (`Marlin_main.cpp:7642-7646`, `stepper.synchronize()`) | `E20` | **port**: `ok` once idle (a normal sync command already waits for idle, Q-5) | ~30 B |
| 2 | `M3`, `M5 [P<0-255>]` | laser and pen modes only: wait for the previous move, then **M3 = laser off, M5 = laser on at PWM power P** (default 255) on PH5 (`uArmSwift.cpp:285-332`). Swapped from the CNC convention (M3 on, M5 off). Other modes: nothing, `ok` | `E20`; the laser is `M2233 V0/V1` | **port as 3.x**, in mode 1 only (`ok` elsewhere, as 3.x). P > 0 = on, P0 = off, **no PWM yet**: PH5 is OC4C, on the wrist servo's 50 Hz timer, and the laser driver's needs are UNVERIFIED. Real PWM after a hardware check | ~80 B |
| 3 | `M2211 N0 A<addr> T<1\|2\|4>`, `M2212 N0 … V<v>` | read / write the AVR's own EEPROM: byte, int16, float (`uArmSwift.cpp:575-688`, `uArmAPI.cpp:686-760`) | 4.x accepts only `N1` (external EEPROM); we dropped both (Q-18) | **read only** (`M2211 N0`); `M2212 N0` stays refused (`E21`): one wrong write ruins the calibration at 800-821 | ~200 B |
| 4 | `M2245 V<name>`, `M2234 V<0\|1>`, `P2245`; 3.x `M2233` | Bluetooth: set the BLE module's name with AT commands on USART1 (`uArmSwift.cpp:948-978`, `uArmAPI.cpp:397-430`); turn BLE off or on (`uArmService.cpp:67-79`); `P2245` = `ok` with nothing (`:1454-1458`); 3.x `M2233` clears the stored MAC flag | not in 4.x; 4.x `M2233` is the laser | **drop for now**: which USART the BLE module sits on, and whether your board has one, is UNVERIFIED (Q-16). `P2245` → `ok` (free; 3.x hosts may send it). 3.x `M2233` keeps 4.x's meaning (laser). Revisit after a probe | `P2245`: ~20 B |
| 5 | `M2410 S<h>` | `S` sets the height offset; without `S`: current height + 0.3 mm (`uArmSwift.cpp:1214-1240`) | 4.x `M2410` takes no parameters (current Z, no +0.3); explicit height is `M2413 H` | **port `S`** as `M2413 H` (today `M2410 S…` is `E21`, so nothing collides); without `S` keep 4.x's | ~40 B |
| 6 | `M2401 B`, `M2401 V<unlock code>` | `B`: the encoder reference "point B" from the current pose (= our `M2401`). `V<code>`: the **main** reference (point A, EEPROM 800), a factory recalibration (`uArmSwift.cpp:1151-1211`) | 4.x `M2401` = point B, no parameters | **`M2401 B` as an alias**; `V<code>` dropped (a factory calibration from an unknown pose) | ~20 B |
| 7 | `M2300`–`M2303`, `P2250`, 3.x `M2500` | Grove modules; factory test result and MAC; switch the command port to USART2 for an OpenMV camera (`uArmSwift.cpp:1259-1267`) | not in 4.x; 4.x `M2500` is something else (A8) | Grove dropped (Q-78). `P2250`: drop (factory test). 3.x `M2500`: not needed, since every USART already takes commands (Q-16), so an OpenMV on USART2 works without switching | 0 |

### A8. `M2500` (Q-77): a finding that changes the decision

In V4.9.0 and V4.10.0, `M2500` runs `atuo_angle_calibra()` (`uarm_angle.c:324-445` in both). It drives the motors
**open-loop** by pulsing the step pins directly, bypassing the planner, the limits and the encoders, from whatever
pose the arm is in, with each step 400 pulses:

- the base 100 steps one way, then 50 back;
- L 14 steps back, then 63 forward;
- R 32, 33, then 60 steps.

It writes encoder maps to EEPROM 1-453. **The maps are read only by the `UARM_2500` build** (`uarm_angle.c:143-200`:
every use sits under `#if defined(UARM_2500)`). The map comments say 400 pulses = 1.8°, which is that arm's gearing.
On the Swift Pro, 400 pulses = 5° (80 steps/°), so the routine would turn:

| Joint | Steps | Distance on the Pro |
|---|---|---|
| Base | 100 forward | **500°** |
| Base | then 50 back | 250° |
| L | 63 forward | **315°** |

All of this is far past the hard stops, and the maps it saves are never used on the Pro.

**Decided (Q-79): `M2500` stays dropped (`E20`, Q-18), recorded as upstream hazard SP-65 #9.** If you want encoder
linearity calibration on the Pro, that is a new design: sweep each joint inside the measured TEACHABLE limits through
our planner at low speed, record encoder against step tally, and store a correction in free EEPROM. It would come
after LIMITS.md is approved. Your answer 11a predates this finding, so I'm asking again.

### A9. Learning mode (Q-78): proposal

What 3.x does, from the earlier review recorded in ISSUES.md:

- **With `M2213 V1`, the base buttons drive it** (`uArmService.cpp:447-557`):
  - Menu click: release every joint and the wrist, and start recording.
  - Menu click again: stop.
  - Play click while recording: toggle the pump, or the gripper if SW_STATE (PB5) is low.
  - Play click: play once. Play long press: loop. Play click while playing: stop.
- **Recording** (`:1517-1560`): every 50 ms, 10 bytes: L, R, base and wrist as uint16 × 100, then the effector state.
  - Stored in the **external I2C EEPROM at 0xA0** (PB6/PB7), up to 65,520 B: about 6,550 samples, 5.5 min.
  - The end marker is 10 × 0xFF.
- **Playback is unreachable in this source**: the `play()` calls are commented out in every revision (ISSUES.md, the
  V3 button review). Only recording ever worked from this tree.

Proposal, in three steps, each built only after you approve it:

1. **External EEPROM driver, and 4.x `M2211`/`M2212 N1` as its commands** (reverses part of Q-18).
   - The read is also the probe: `M2211 N1 A0 T1` tells whether your board has the chip.
   - Bit-banged on PB6/PB7 like the encoders, page writes, with each write cycle polled so nothing blocks.
   - About 600 B flash.
2. **Recording**, as 3.x, in its format, so a recording from a 3.x arm plays here.
   - Started and stopped by the buttons, only after `M2213 V1`.
   - The samples come from our encoder reads at 20 Hz. The wrist sample is the commanded angle, because the wrist
     servo has no readback (see A10).
   - Recording needs every joint released, as 3.x does.
   - About 800 B flash.
3. **Playback** (new: 3.x's never ran).
   - Before it starts, every joint is locked and re-read (as `M17`).
   - Each sample becomes one joint-space move timed to 50 ms (capped at the joint speed), blended through the
     planner's corner tolerance.
   - Every sample is checked against the joint limits. The first illegal one stops playback, with a beep.
   - The lost-step watch stays on. `S1100`, Ctrl-X or the Play button stop it.
   - Pump and gripper switch at their samples.
   - About 1.5 KB flash and 40 B RAM.

Hardware tests:

- Probe the EEPROM (read only).
- Record about 10 s by hand.
- Play it once at the arm, with a hand on the power switch, inside the TEACHABLE limits.

**Questions:**

- (a) Step 1 now?
- (b) Should playback wait until LIMITS.md is approved? I recommend yes: it is a powered move to wherever your hand
  took the arm.
