# Subsystem 2 — test log

**Subsystem:** Motion control: encoders, wheel speed PI, moves and turns (Buddy 2)  ·  **Student ID:** 2303689  ·  **Code:** `app_program/subsystem2/`

See `testing/README.md` for how to run tests and record results.
Requirement IDs (FR / NFR) are from the Tribunal 2 requirements (`IS10_PreTri2`).
Test cases checked against `refactor-barrc` at `f344714` (23 Nov 2026).

## Test cases

Define a test here before running it. Test IDs never change and are never reused.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S2-M01 | encoder counts per wheel revolution (`S2_ENC_CPR` 415) | FR6 | manual | Mark the tyre; hand-turn each wheel 10 full revolutions; read `total=` in the `s2:` status line before and after | 4150 counts ± 1 % (4109..4191) on both wheels |
| S2-M02 | motor direction and brake per wheel | FR8, FR28 | manual | Wheels off the table: forward, reverse, brake on each motor (`S2_INVERT_RIGHT 1`) | both wheels turn forward for a forward command; brake stops each wheel |
| S2-M03 | straight distance accuracy | FR6, FR8 | manual | 10 × MOVE(d = 500 mm) and 10 × MOVE(d = 1000 mm) on the floor; measure with a tape | each within ±5 % |
| S2-M05 | stall detection | NFR10 (detection time), statechart STALL_FAULT | manual | Drive at 100 mm/s for at least 1 s (past the 500 ms `S2_STALL_ARM_MS` start-up window), then hold one wheel still | `s2: STALL LEFT/RIGHT wheel: … no encoder pulse for ≥ 300 ms`; STALL_FAULT; FAULT_STALL set; SENSOR_FAULT to the VC; motors braked |
| S2-M06 | constant speed over the hump (with S4) | FR23 | manual | Drive over the black and yellow humps 10× each; log `v=` during HUMP_TRAVERSE | no stop; speed within ±20 % of the target over the hump |
| S2-M07 | speed estimate from the encoders | FR6 | manual | Drive MOVE_FOLLOW at 100 and 200 mm/s along a 2 m tape; time it with a stopwatch; read `v=` in the `s2:` line | measured speed within ±10 % of the time-based speed |
| S2-M08 | PI + feed-forward speed control | FR7 | manual | On the floor, straight MOVE (no line following, so no steering): step the target 0 → 150 mm/s and 150 → 80 mm/s; log `v=` and `duty=` every status line, both wheels | each wheel within ±10 % of the target within 1 s of the step; no growing oscillation |
| S2-M10 | speed cap near an obstacle (with S5 / VC) | NFR12 | manual | Drive towards a box; read `v=` from 0.5 s after `vc: something … cm ahead, slowing to 80 mm/s` appears until the robot stops | both wheels ≤ 100 mm/s (NFR12 limit) in every status line |
| S2-M11 | steering message timeout | FR10 (S3 → S2) | manual | During line following, stop the Line & Barcode task so steering messages stop (debugger breakpoint, or a test build that skips `motion_send_steer()`) | steer returns to 0 within 100 ms (`S2_STEER_TIMEOUT_MS`); no runaway turn |
| S2-M12 | Motion telemetry slot on the dashboard (B2, 26 bits, updated every 100 ms, sent in the 500 ms telemetry frame) | FR3, IF to S1 | manual | WiFi build, dashboard open, drive and stop | dashboard Motion panel: speeds, odometer and state match the console `s2:` line |
| S2-T01 | E-STOP flag → brake latency | NFR10 (≤ 100 ms from detection to stop) | manual (logic analyser) | Toggle a spare GPIO where `flg_safety` E-STOP is set; set E-STOP from each source (dashboard, lift, obstacle < 5 cm); measure GPIO edge → GP8..11 all LOW | ≤ 100 ms worst case over 20 trials (design: within one 20 ms Motion cycle) |
| S2-T02 | Motion task period, jitter and WCET with Wi-Fi active | NFR14 | manual (GPIO toggle) | Toggle a spare pin at the start/end of the 20 ms cycle, scope it | period 20 ms ± 1 ms; WCET below 20 ms |

*Method:* **manual** = a person observes the result (console, dashboard, tape measure, scope, logic analyser…).
Turn angles: a TURN command turns about 0.63 × its value on the mat; the VC scales bypass turns by 1.59 for real degrees.
S2-M04 (on-the-spot turn accuracy) and S2-M09 (compass-assisted junction turns) were withdrawn on 23 Nov before being run; their IDs are not reused.

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Result |
|---|---|---|---|
| 2026-11-23 | S2-M01 | `f344714` | Passed |
| 2026-11-23 | S2-M02 | `f344714` | Passed |
| 2026-11-23 | S2-M03 | `f344714` | Passed |
| 2026-11-23 | S2-M05 | `f344714` | Passed |
| 2026-11-23 | S2-M06 | `f344714` | Passed |
| 2026-11-23 | S2-M07 | `f344714` | Passed |
| 2026-11-23 | S2-M08 | `f344714` | Passed |
| 2026-11-23 | S2-M10 | `f344714` | Passed |
| 2026-11-23 | S2-M11 | `f344714` | Passed |
| 2026-11-23 | S2-M12 | `f344714` | Passed |
| 2026-11-23 | S2-T01 | `f344714` | Passed |
| 2026-11-23 | S2-T02 | `f344714` | Passed |
