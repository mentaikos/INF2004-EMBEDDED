# Subsystem 4 — test log

**Subsystem:** IMU & terrain (Buddy 4: tilt, humps, shock, heading)  ·  **Student ID:** 2500701  ·  **Code:** `app_program/subsystem4/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S4-U01 | message buffer max line = S4_SUMMARY_MAX (template example, kept for S3's example consumer) | IF-03 | auto | `subsystem4_test.c` | `[TEST] S4-U01 PASS` |
| S4-U02 | LED pauses and resumes every 6 reports (template example monitor) | IF-05 | manual | 1. Flash normal image. 2. Watch LED and console ~20 s. | console shows "LED task paused"/"resumed"; LED stops/starts |
| S4-U03 | packed status: 42 bits in 2 words, round trip, negative pitch/rate sign-extend, unused bits 0 | minimum-bit (Week 8) | auto | `subsystem4_test.c` | `[TEST] S4-U03 PASS` |
| S4-U04 | packed status fields clamp, never wrap (pitch ±25.5°, rate ±255°/s, height 31 mm, count 15, fails 3) | minimum-bit | auto | `subsystem4_test.c` | `[TEST] S4-U04 PASS` |
| S4-U05 | VC event words: HUMP_START, HUMP_END(height, 5 bits, clamped), SENSOR_FAULT(FAULT_IMU) | `mbf_vc_evt` (app.h EVT_PACK) | auto | `subsystem4_test.c` | `[TEST] S4-U05 PASS` |
| S4-U06 | integer atan2 (CORDIC) within 0.02°, integer sine within 0.5 % | FR13 (no FPU, no libm) | auto | `subsystem4_test.c` | `[TEST] S4-U06 PASS` |
| S4-U07 | made-up drive over the black (25 mm) and yellow (10 mm) humps: one HUMP_START + one HUMP_END each, height ±3 / ±2 mm | FR (MEASURE), Version C row 14 | auto | `subsystem4_test.c` | `[TEST] S4-U07 PASS` |
| S4-U08 | 2 m of flat track with ±40 mg noise: no false hump | A4 thresholds (+3°) | auto | `subsystem4_test.c` | `[TEST] S4-U08 PASS` |
| S4-U09 | > 2.5 g → one SHOCK, back to LEVEL after 200 ms | FR27 | auto | `subsystem4_test.c` | `[TEST] S4-U09 PASS` |
| S4-U10 | shock while on a hump → HUMP_END(0) so the VC leaves HUMP_TRAVERSE | Version C row 9 | auto | `subsystem4_test.c` | `[TEST] S4-U10 PASS` |
| S4-U11 | 3 failed reads → FAULT + one SENSOR_FAULT; recovery → CALIBRATING | statechart 6.4 | auto | `subsystem4_test.c` | `[TEST] S4-U11 PASS` |
| S4-U12 | sizes: status 32 B unpacked → 8 B packed; height sum int16 (2 B) | Week 13 evidence, Version C row 14 | auto | `subsystem4_test.c` (sizes printed) | `[TEST] S4-U12 PASS` |
| S4-U13 | IMU task runs every 10 ms (95–105 cycles in 1 s), WCET < 1 ms | NFR11, A6 | auto | `subsystem4_test.c` | `[TEST] S4-U13 PASS` |
| S4-U14 | sensor found on Grove 6, calibrated, LEVEL (robot still and flat from power-up) | FR13 | auto (needs sensor) | `subsystem4_test.c` | `[TEST] S4-U14 PASS` |
| S4-H01 | PC simulation: exact two-axle geometry over both humps at 100–400 mm/s, noise, long ramp, flat, shock, fault, packing | design check | auto (PC) | `cd app_program/subsystem4/host_test`, build line in `test_host.c`, run | `ALL TESTS PASSED` |
| S4-M01 | I2C bring-up: both chips answer | hardware, Grove 6 | manual | Flash, read the console at boot | `s4: I2C1 GP26/GP27: accel 0x19 found, mag 0x1E found -> LSM303DLHC ready` |
| S4-M02 | axis mapping | FR13 | manual | Robot flat, then lift the nose ~10° | pitch ≈ +0.0 flat, ≈ +10.0 nose up (if negative: fix `LSM303_ROBOT_*` in `drv_lsm303.h`) |
| S4-M03 | heading and turn rate | FR14, FR15 | manual | Rotate the robot slowly 360° by hand, then once quickly | heading runs 0→359; rate sign matches the turn direction |
| S4-M04 | magnetometer disturbed by the motors (Version C row 12) | assumption | manual | Same heading, motors off vs driving; read `hdg` | note the difference in degrees |
| S4-M05 | tap → SHOCK → E-STOP → motors brake | FR27, NFR10 | manual | Wheels off the table, motors running, tap the board hard | `s4: SHOCK -> E-STOP set`, Motion in E_STOP_BRAKE, wheels stop |
| S4-M06 | unplug Grove 6 → SENSOR_FAULT → recovery | statechart 6.4 | manual | Unplug for 3 s, plug back in, keep still 2 s | state FAULT, then CALIBRATING → LEVEL |
| S4-M07 | real humps, 10 runs each at cruise speed | FR (MEASURE), A4 | manual | Drive over black and yellow hump 10× each, read `s4: HUMP_END height=` | black 25 ± 2 mm, yellow 10 ± 2 mm, no false humps between |
| S4-M08 | measure wheelbase and hump slope lengths | constants | manual | Ruler: front-to-rear axle; each hump's slope length | set `IMU_WHEELBASE_MM`; record slopes here |
| S4-T01 | IMU period, jitter and WCET with Wi-Fi active, before vs after packing | NFR11, A6, Week 13 | manual (GPIO toggle) | Set `S4_DEBUG_TIMING_PIN` to a free pin, scope it; also read `wcet=` in the log | period 10 ms ± 1 tick; WCET well below 10 ms |
| S4-T02 | shock → brake latency | NFR10 (≤ 100 ms), Version C row 11 | manual (logic analyser) | Set `S4_DEBUG_SHOCK_PIN`; tap; measure its edge → GP8..11 all LOW | ≤ 100 ms worst case over 20 trials |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S4-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
