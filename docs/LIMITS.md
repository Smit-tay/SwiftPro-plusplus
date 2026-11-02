# LIMITS — joint limits: published, in the code, accepted, reachable by hand

**Status (2026-10-24): done. Margin 1° and a per-L table for R (Q-82) and the numbers of §6 (Q-83) were applied
2026-10-21. The 2026-10-24 refit (§4.2, §6) is approved and applied in `kin.h`/`kin.cpp`: the base −5 … 185, and a
new R table, whole degrees, with no dents. §2 and §3 describe the code before Q-83.**

Four sets of limits:

| Set | What it is | How it is found |
|---|---|---|
| **SPEC** | uFactory's published ranges: base 0–180°, left motor 0–130°, right motor 0–106°, end effector 0–180°, reach 50–320 mm | Swift Pro spec sheet and Developer Manual V1.0.6 |
| **SOURCE** | every limit constant and every legality check in the code | reading V3.1.18-dev, V4.9.0, V4.10.0, jss.7 and SwiftPro++ |
| **COMMANDABLE** | the poses a joint or Cartesian command is actually accepted for | `tools/limits_sweep/run.sh`: each firmware's legality code compiled on the host, swept on a 0.5° grid; `avr_check.sh` confirms it on the AVR |
| **TEACHABLE** | what a person can reach by hand with the joints released (`M2019`) | `tools/limits_sweep/teach_log.py` at the arm |

## 1. Angle frames

All numbers here are in the firmware's frames (PROTOCOL.md §5):

- **L** (left motor, lower arm): up from horizontal. **R** (right motor, upper arm): down from horizontal. Both as
  `P2200` reports them.
- **Elbow**: the angle between the two arms, 180 − L − R. The linkage couples L and R through it.
- **Base**: the user frame, 90 = straight ahead (`P2200 B`, `G2202 N0`). The code checks it in the "−90" frame
  (0 = straight ahead), so the code's −90…90 is 0…180 here.
- **Wrist**: the servo angle, 90 = centre.

Whether the SPEC uses the same zero for L and R is **UNVERIFIED**. One sign that it does: at the folded rest you
observed R = 105.77, against SPEC R max 106. TEACHABLE will show it: if the hand-found stops line up with 0/130 and
0/106, the frames are the same.

## 2. SOURCE

### 2.1 Constants (Swift Pro build)

| Constant | V3.1.18-dev (Marlin) | V4.9.0 | V4.10.0 | jss.7 (72a2ccf) | SwiftPro++ |
|---|---|---|---|---|---|
| L min / max | 0 / 135.6 | −4 / 135.6 | −4 / 135.6 | **−20** / 135.6 | **−20** / 135.6 |
| R min / max | 0 / 119.9 | −4 / 119.9 | −4 / 119.9 | **−20** / 119.9 | **−20** / 119.9 |
| Elbow min / max | 10 / 151 | 10 / 151 | 10 / 151 | 10 / 151 | 10 / 151 |
| Base (user frame) | 0 / 180 | 0 / 180 | 0 / 180 | 0 / 180 | 0 / 180 |
| Wrist | — | 0 / 360 accepted, the servo wraps (SP-23) | same | same | 0 / 180 (SP-23); modes 4/5 stepper 0 / 360 |
| Where | `uArmParams.h:110-115`, `uArmCalibration.cpp:205-230` | `uarm_coord_convert.c:56-63` | `uarm_coord_convert.h:61-68` | `uarm_coord_convert.c:56-63` | `src/kin.cpp:27-32` |

V3.1.18-dev uses its constants only to unwrap encoder readings, never to refuse a move. Its `E_OUT_OF_RANGE` (22,
`uArmParams.h:104`) is defined but never used in this tree, although its changelog says "Return E22 when position
unreachable" (`update.log:2`; UNVERIFIED whether the released hex differs). The Mini (UARM_MINI) and 2500 variants
have other values (V4.10.0 Mini: R min 1, elbow 18–137); they are not the Swift Pro.

jss.7's −20 comes from your commit f01e691 (May 2026), so that a taught pose with L = −14.3 passes.

### 2.2 Legality checks

| Check | V4.x and jss.7 | SwiftPro++ |
|---|---|---|
| `is_angle_legal(L, R, B)`: NaN, L, R, elbow, base, all inclusive | `uarm_coord_convert.c:77-101` (V4.10.0 `:16-40`), identical body in all three | `kin::legal`, the same test with the same constants |
| Joint commands `G2202 N0-2`, `G2206` | FK of the requested angles, then `mc_line`: IK of that point, then `is_angle_legal` on the **IK result** (`uarm_protocol.c:279-321, 386-404`, `motion_control.c:66-84`). So the check is on a round trip, not on the angles asked for (§3.2) | `is_angle_legal` on the requested angles (`motion.cpp` `queue_joints`) |
| Cartesian `G0`, `M2222` | IK, then `is_angle_legal` | the same (`motion.cpp` `joints_of`, `commands.cpp` M2222) |
| Linear `G1`, `G2201`, `G2204`, `G2205` | each segment's IK checked; an illegal segment is **skipped** (`continue`, `motion_control.c:145`), not refused, so the arm cuts the corner | every piece checked before anything moves; any illegal piece refuses the move with `E22` (SP-20) |
| `M2220` (IK query) | none | none (faithful) |
| Grbl soft limits | compiled off (`defaults_generic.h:53`), and `$20=1` is refused without homing (`settings.c:252`); your `$$` dump (2026-10-17) confirms they are off | none (no Grbl) |
| V4.10.0 jogs `G2207`/`G2208`/`G2209` | aim at the limits minus a margin: elbow 151 − 2 or 10 + 1, L/R max − 1 or min + 1, and `G2209` brings a joint to its limit ∓ 2 (`uarm_protocol.c:408-1440`) | not built (A4) |
| Boot, release, re-lock | no check: a pose outside the limits is accepted as the position, and only moves back inside are possible | the same |

## 3. COMMANDABLE (host sweep)

Run: `tools/limits_sweep/run.sh` (host, about 2 s), then `tools/limits_sweep/avr_check.sh <simavr dir>` (about
10 s). The grid is L −30…145 and R −30…130 in 0.5° steps, base 0 (−90 frame), mode 0 offsets: 112,671 poses. Each
pose goes through each firmware's own code: the original's `uarm_coord_convert.c` with AVR float semantics, and
SwiftPro++'s `kin.cpp`. Output: `build/limits/summary.txt`, `region.csv` (every pose, every check).

**On the AVR** (`avr_check.sh`: 5,340 poses with the original's toolchain, avr-gcc 7.3, under simavr): the host
and the AVR disagree on 37 poses (V4.9.0, V4.10.0) and 42 (jss.7). Every one of them is either at exactly L = 0 or
R = 0, or a pose whose round trip lands within 0.01° of a limit, where the last float bit decides. None is anywhere
else. So the regions below hold on the arm, except for which side of a limit an exact boundary pose falls on.

### 3.1 Results

| | V4.9.0 | V4.10.0 | jss.7 | SwiftPro++ |
|---|---|---|---|---|
| Poses `is_angle_legal` accepts | 52,130 | 52,130 | 63,234 | 63,234 |
| Poses a joint command accepts | 53,803 | 53,803 | 64,825 | 63,234 |
| of which **illegal**, moved to a different pose (§3.2) | 1,997 | 1,997 | 1,997 | **0** |
| **Legal** poses refused by a joint command (§3.3), host: at L or R = 0 / on a limit (float noise) | 61 / 263 | 61 / 263 | 61 / 345 | 0 / 0 (a Cartesian command for the same pose: 0 / 213 on a limit) |
| Base range (at L46 R66) | 0…180 | 0…180 | 0…180 | 0…180 |

- V4.9.0 and V4.10.0 accept exactly the same poses.
- jss.7 adds 11,022 grid poses, all with L or R in −20…−4, and drops none.
- SwiftPro++ accepts exactly `is_angle_legal` with jss.7's constants. Compared with jss.7 it refuses the 1,997
  mirror poses (`E22`), and accepts the 406 legal poses that jss.7 refuses on its round trip.

### 3.2 Finding: the original moves to a different pose than the one asked for (new SP-63)

For requests with L ≤ 1 and R ≤ 1 (elbow 209–240°, far outside the limits), the original's IK returns the
other solution for the same point, which is legal. So the command is accepted, and **the arm goes somewhere
else**: `G2206 B90 L-30 R-30` drives to L33.68 R26.32, and `L-25 R-25` to L27.97 R22.03 (host; the AVR agrees,
1,998 such poses). That is true in V4.9.0, V4.10.0 and jss.7. SwiftPro++ checks the requested angles and replies `E22`.

### 3.3 Finding: legal poses refused at L = 0 or R = 0 exactly (new SP-64)

The original IK takes `acos` of a value that rounds just above 1 there, giving NaN. On the AVR, 40 of 204 legal
poses with L or R exactly 0 are refused (`E22`). Which ones depends on rounding. Near those angles the IK also
loses accuracy: L110.5 R0 comes back as R0.028. SwiftPro++'s IK uses `atan2` and has neither problem. Separately, the
round trip makes the original's limits slightly exclusive. A pose exactly on a limit can come back 0.00001° outside
it, so jss.7 refuses, for example, L45 R−16 (elbow exactly 151), which SwiftPro++ accepts.

### 3.4 The (L, R) region

In closed form (all versions, with their own L min and R min):

  L min ≤ L ≤ 135.6,  R min ≤ R ≤ 119.9,  29 − L ≤ R ≤ 170 − L  (from elbow ≤ 151 and elbow ≥ 10)

R accepted by joint commands, per L. 0.5° grid, so 119.9 shows as 119.5. `*` = not one interval: the mirror poses
or the L = 0 holes. The V4.x rows for L < −4 are mirror poses only (§3.2).

| L | V4.9.0 = V4.10.0 | jss.7 | SwiftPro++ |
|---|---|---|---|
| −20 | −30…−9 (mirror only) | −30…119.5 * | 49…119.5 |
| −15 | −30…−14 (mirror only) | −30…119.5 * | 44…119.5 |
| −10 | −30…−19.5 (mirror only) | −30…119.5 * | 39…119.5 |
| −5 | −30…−24.5 (mirror only) | −30…119.5 * | 34…119.5 |
| 0 | −30…119.5 * | −30…119.5 * | 29…119.5 |
| 10 | 19.5…119.5 | 19.5…119.5 | 19…119.5 |
| 20 | 9.5…119.5 | 9.5…119.5 | 9…119.5 |
| 30 | −1…119.5 * | −1…119.5 * | −1…119.5 |
| 40 | −4…119.5 | −11…119.5 | −11…119.5 |
| 50 | −4…119.5 | −19.5…119.5 | −20…119.5 |
| 60 | −4…109.5 | −19.5…109.5 | −20…110 |
| 80 | −4…90 | −19.5…90 | −20…90 |
| 100 | −3.5…69.5 * | −19.5…69.5 * | −20…70 |
| 120 | −3.5…49.5 | −20…49.5 | −20…50 |
| 135 | −4…35 | −20…35 | −20…35 |

(Every 5° in `build/limits/summary.txt`, with a map of the region.)

## 4. TEACHABLE

Procedure: [HARDWARE-TEST.md, "Limits: TEACHABLE"](HARDWARE-TEST.md#limits-teachable). It releases the joints and logs
`P2200` at 20 Hz while you move the arm by hand: L and R together over their whole region, then the base, then the
wrist. Then it records each joint's stop and the elbow's two stops, held one at a time. Nothing is powered beyond
the hand-found boundary.

The wrist servo has no position readback except, possibly, `SER2_FEEDBACK` on PF3/ADC3 (your schematic notes). The
logger records `P2241 N3` (raw ADC) for it. Whether that pin carries the servo's feedback, and its scale, are
UNVERIFIED, so the wrist's TEACHABLE column may stay empty (A10).

### 4.1 Results (2026-10-21, three runs)

The logs are kept outside the repo, in `SwiftPro-plusplus-records/teach/`:

| Run | What it holds |
|---|---|
| `182123` | the full procedure; sweep-LR was quick (44 s) and covered the region only sparsely; stop-B-min skipped |
| `182616` | base sweep and base stops. The phases were swapped by mistake: "sweep-LR" holds the base sweep and "sweep-B" the (folded only) L/R sweep |
| `183403` | one loop around the whole (L, R) outline in 77 s: elbow closed, shoulder end to end, elbow opened, shoulder back. Only its first phase counts; the stop phases after it were passed with Enter and hold nothing |

| Joint | Stop by hand | Notes |
|---|---|---|
| L | −14.79 … 136.73 | the same at every R: a real stop on L |
| R | −19.30 … 115.52 | depends on L (the table, §6) |
| Elbow | 9.31 … 141.32 | open: 141.2–141.3 in two runs, for L −15…50. Closed: 9.3–10.4 for L 80…136, but 11–19 for L 45…80, where R's own stop comes first |
| Base | 13.36 … 163.74 held | the sweeps reached 3.8 … 170.2, inconsistently (3.8, 6.6, 8.6 at the low end): no hard stop at an exact angle. The held stops are used |
| Wrist | none | `P2241 N3` moved 211–227 during the wrist sweep and 189–283 while the other joints moved: it is not the servo's feedback |

Where the hand trace did not reach the stop, the measured edge is inside the true one. The clearest case: R min is
−19.3 at L 88 (run 182123) but only about −11 at L 50…100 in the loop run, and −4.5…−7 near L 95…100. The table
follows the trace, so these places give up reach, never safety.

### 4.2 Results (2026-10-24, guided run)

`tools/limits/teach_limits.py`, one screen per step (SwiftPro++ V5.0.0-pp.1, `P2200` at 10 Hz, 4,783 readings, none
lost; resting noise 0–2 counts). The run's folder, `tools/limits/data/2026-10-24_1310/`, is kept out of git like the
earlier logs; `fit_table.py` needs it.

| Joint | Stop by hand | Notes |
|---|---|---|
| L | −14.8 … 136.7 | the same as 2026-10-21 to 0.01° |
| R | −19.3 … 113.1 | −19.3 reached at every L from 50 to 140; the top at full extension 2.4° below 2026-10-21's 115.5 |
| Elbow | 9.0 … 141.3 | as 2026-10-21 |
| Base | **−7.56 … 187.65** | identical to the count in two steps (arm folded, arm extended), each held 2–9 s: hard stops. You confirmed the base turns that far. **2026-10-21's "held stops" 13.36 / 163.74 were not the stops.** Higher numbers are to your left, standing behind the robot |
| Hand | none | the firmware has no reading of the suction cup's angle |

Gap: with the forearm folded (step 4) the lower arm went no lower than 75.8, so the forearm's top edge for L −10 … 50
rests on the 2026-10-21 traces alone.

## 5. Comparison

Flags: ⚑ = differs from SPEC by more than 1°; ⚠ = COMMANDABLE goes beyond TEACHABLE (a commanded move could drive
into a stop).

| Joint | SPEC | SOURCE (V4.x / jss.7 / ours) | COMMANDABLE (V4.x / jss.7 / ours) | TEACHABLE | Proposed (stop − margin) |
|---|---|---|---|---|---|
| Base | 0…180 | 0…180 all | 0…180 all | −7.6…187.6 ⚑ (2026-10-24; 13.4…163.7 held on 2026-10-21) | −5…185 (2026-10-24) |
| L | 0…130 | −4…135.6 / −20…135.6 / −20…135.6 ⚑ (min below SPEC, max 5.6 above) | −4…135.6 (plus mirror poses, §3.2) / −20…135.6 (plus mirror) / −20…135.6 ⚠ (min 5.2 past the stop) | −14.8…136.7 ⚑ | −13.8…135.6 |
| R | 0…106 | −4…119.9 / −20…119.9 / −20…119.9 ⚑ (max 13.9 above SPEC; you saw 105.77 at the folded rest) | as SOURCE, coupled through the elbow ⚠ (max 4.4, min 0.7 past the stop) | −19.3…115.5 ⚑, depends on L (2026-10-24: −19.3…113.1) | the table, §6 |
| Elbow (180 − L − R) | not published | 10…151 all | 10…151 (the original's slightly exclusive, §3.3) ⚠ (max 9.7 past the stop) | 9.0…141.3, the closed stop depends on L | part of the table, §6 |
| Wrist | 0…180 | 0…360 (wraps) / same / 0…180 | the same | none (ADC3 is not the feedback) | 0…180, unchanged |

| (L, R) region | SPEC | SOURCE | COMMANDABLE | TEACHABLE |
|---|---|---|---|---|
| shape | two independent ranges (no coupling published) | the box L × R cut by 29 − L ≤ R ≤ 170 − L | the same, minus §3.3, plus the original's mirror poses | R min/max per L, §6; not the box-and-band shape: the closed elbow is 11–19° for L 45…80 |

## 6. The limits (approved, Q-83, applied 2026-10-21; refit 2026-10-24)

The aim: the arm may be commanded anywhere it can physically reach, minus a small margin, and every move between
two allowed poses stays allowed all the way. The table surface and the arm's own body are not covered yet.

- **L:** −13.8 … 135.6 (stop + 1 = −13.79; stop − 1 = 135.73, capped at COMMANDABLE's 135.6). Unchanged.
- **Base:** −5 … 185 in the user frame, `kLimitBase{-95, 95}` in the −90 frame (2026-10-24, your values: 2.6° inside
  the stops −7.56 / 187.65). This is wider than the original's 0 … 180: a host that assumes 0 … 180 may need a look.
- **Wrist:** 0 … 180, unchanged.
- **R:** between `R min(L)` and `R max(L)`, straight lines between the 5° points below. This replaces the separate R
  and elbow limits: the elbow stops are inside it (at fixed L, 1° of R is 1° of elbow).

How the R table is built (`tools/limits_sweep/fit_table.py <dir of the 2026-10-21 logs> <2026-10-24 run dir>`
prints it again):
1. **Outline:** consecutive samples (under 0.3 s apart) of all four runs are joined into lines, since the arm passed
   through every pose between them; the outline at each L (every 0.05°) is the lowest and highest R on any line.
2. **Margin (your rule):** each edge is rounded inwards to a whole degree, then moved 1° further in (−19.3 → −18,
   113.1 → 112). The margin is 1.0 to 2.0°.
3. **No dents:** R min is convex in L and R max concave, so the allowed region has no inward notch. `G0`, `G2202` and
   `G2206` check only where a move ends and then move along a straight line in joint space; without dents that line
   stays inside, and so does a blended corner (`M2126 V1`). The 2026-10-21 table had dents: 558 of 200,000 random
   straight moves between legal poses left it; with this table, none.
4. **Least reach given up:** among the whole-degree tables that satisfy 2 and 3, the one with the lowest R min and the
   highest R max (an exact search). The L 140 point lies past L's limit and continues the line from L 130–135.

Result: at least 1.30° from the measured outline everywhere inside L's limits.

| L | R min | R max | | L | R min | R max |
|---|---|---|---|---|---|---|
| −15 | 56 | 112 | | 65 | −18 | 98 |
| −10 | 51 | 112 | | 70 | −18 | 94 |
| −5 | 46 | 112 | | 75 | −18 | 90 |
| 0 | 41 | 112 | | 80 | −18 | 86 |
| 5 | 36 | 112 | | 85 | −18 | 82 |
| 10 | 31 | 112 | | 90 | −18 | 78 |
| 15 | 26 | 112 | | 95 | −18 | 73 |
| 20 | 21 | 112 | | 100 | −18 | 68 |
| 25 | 16 | 111 | | 105 | −18 | 63 |
| 30 | 11 | 110 | | 110 | −18 | 58 |
| 35 | 6 | 109 | | 115 | −18 | 53 |
| 40 | 1 | 108 | | 120 | −18 | 48 |
| 45 | −4 | 106 | | 125 | −18 | 43 |
| 50 | −8 | 104 | | 130 | −18 | 38 |
| 55 | −12 | 102 | | 135 | −18 | 33 |
| 60 | −16 | 100 | | 140 | −18 | 28 |

(The L = −15 and 140 rows only shape the first and last sections; L itself stops at −13.8 and 135.6.)

Against the 2026-10-21 table: R min opens to −18 from L 65 to 140 (4 to 13.5° more reach, the forearm extended with
the lower arm back); R max is 1–2° tighter almost everywhere and 5° near L 50, where the old table's bump was a
dent; R min at L −15 … 50 is about 1° tighter (whole-degree rounding).

What changes for you (2026-10-21):
- **Poses refused that were accepted:** L below −13.8 (jss.7's −20 was added for a taught pose at L −14.3, commit
  f01e691: that pose is now 0.5° from the stop and is refused with E22), and every pose outside the table. Boot,
  release and re-lock still accept any pose, and only moves back inside are possible (§2.2).
- **Cost (measured):** +2,700 B flash: the table (32 × 2 int16 tenths of a degree) is 128 B, the rest is
  `limit_r`, the L jog's arithmetic (`l_reach`) and inlining. RAM unchanged. `kin::legal` takes 0.29 ms (one lookup
  and one interpolation); an L jog's end, `l_reach`, at most 3.6 ms (simavr profile), within the main loop's 4.5 ms.

### 6.1 Where it lives and how it is checked

1. `kin.h`: `kLimitL`, `kLimitBase`, `limit_r(L)`; `kin.cpp`: the R table in flash. `kin::legal`, M2222, every move
   and the jogs read them: `G2208 N1` (L) follows R's range as L changes (`kin::l_reach`, exact on the table's
   straight lines) and is `E22` from outside the limits; `G2208 N2` (R) goes to R's range at the current L; `G2209`
   puts L and the base 2° inside, then R 2° inside its range at that L.
2. `unit_kin.cpp` `test_limits`: its own copy of the table against `kin.cpp`'s, each point ± 0.05°, no dents, a
   point between, L and base ends, your probe pose (legal) and f01e691's L −14.3 (refused), the table inside the
   original's limits at every 0.05° of L, and `l_reach` against a 0.01° walk along L from 1,260 starts both ways
   (worst 0.01°).
3. The host sweep (`run.sh`) now gives ours: L −13.5…135.5, R −18.0…112.0, elbow 12…139 on its 0.5° grid, base
   −95…95 (−90 frame). `compare.py` now reads our limits from `kin.h`/`kin.cpp` for its float-noise test (it had kept
   jss.7's box): the 101 legal poses a Cartesian move refuses are all within 0.01° of an edge.
4. `test/golden/17-jog.txt` follows the new limits (2026-10-24: the L jog ends at 100.96, the R jog at −17, the base
   jog at 184, `G2209` at L 133.6 R 32.4). The simavr session's base moves (40, 160) are inside both ranges; it was
   not rerun on 2026-10-24 (no simavr build on that machine).
5. The differences from SPEC are SP-66 in ISSUES.md.
