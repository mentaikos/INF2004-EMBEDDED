# Subsystem 5 — test log

**Subsystem:** Ultrasonic scan & obstacle bypass (Buddy 5)  ·  **Student ID:** 2503076  ·  **Code:** `app_program/subsystem5/`

Tests checked against `main` at `1e4ca46`. Requirement IDs are from `IS10_PreTri2`.
Test IDs never change and are never reused.

## Test cases

| ID | Test | Steps | Pass if you see | Req. |
|---|---|---|---|---|
| S5-U01 | Detect, measure and go round a box (video demo) | 1. WiFi build, dashboard open. 2. Put the 13 x 10 cm box on a straight part of the line. 3. START. 4. Watch the robot and the dashboard until it is following the line again. | Robot **stops** in front of the box. Dashboard width within **±10 %** of the real width (13 cm box → 12–14 cm). It goes round on the open side, **never touches** the box, and is back **on the same line**, following it | FR16, FR26, NFR1, NFR2, NFR3, NFR4, NFR5 |
| S5-AUTO | Automatic checks | 1. Flash the test build. 2. Open the serial monitor. 3. Wait until it stops printing (about 20 s). | Every `[TEST] S5-…` line ends in **PASS** (no FAIL). Covers S5-U03…U26. PC tests S5-H01…H04: `./t` → all PASS | — |
| S5-M02 | Detect, measure, choose a side ×20 | Box on a straight: 5× centred, 5× moved left, 5× moved right, 5× with something blocking one side. | Robot stops every time. Dashboard width within **±10 %** of the real width (13 cm box → 12–14 cm). It goes round on the **open** side | NFR1, NFR2, NFR3 |
| S5-M03 | Go round and back to the line ×20 | Box on a straight part of the track, START, time it with a stopwatch from the stop at the box to back on the line. | Never touches the box. Back on the **same** line and following it, in under 20 s | NFR4, NFR5, FR26 |
| S5-M04 | Too close = instant stop | While the robot drives, quickly put a box about 4 cm in front of it. | Robot stops at once (E-STOP) and does not move again until START | NFR10 |
| S5-M05| No way round | Box on the line with walls (or boxes) close on **both** sides. | Robot backs up a little and scans again, up to 3 times, then stops (FAULT). It never drives into the box | FR18 |
| S5-M06 | No false stop on humps | Drive over both humps 10 times with no box on the track. | No obstacle stop on the humps, 10/10 | NFR1 |
| S5-M07 | Obstacle panel on the dashboard | WiFi build, run one bypass. | Obstacle panel shows the distance, width, side and scan count (not "not sent yet") | FR3, FR24 |

**S5-U01 (changed 2026-11-23).** S5-U01 used to be the template's automatic
"runs in a Micro T-Kernel task" check. It is now the single-run bypass demo
above, to match the S5-U01 video in `videos/README.md`. The automatic check
is still run, now as **S5-U26** (`subsystem5_test.c`). S5-U01 is one run of
what S5-M02 and S5-M03 repeat ×20.

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Tester | Result | Notes |
|---|---|---|---|---|
| 2026-11-23 | S5-U01 | 2503076 | PASS | Video: <https://youtu.be/eVq0_ZQNGTY> |
| 2026-11-23 | S5-AUTO | 2503076 | PASS | |
| 2026-11-23 | S5-M02 | 2503076 | PASS | |
| 2026-11-23 | S5-M03 | 2503076 | PASS | |
| 2026-11-23 | S5-M04 | 2503076 | PASS | |
| 2026-11-23 | S5-M05 | 2503076 | PASS | |
| 2026-11-23 | S5-M06 | 2503076 | PASS | |
| 2026-11-23 | S5-M07 | 2503076 | PASS | |
