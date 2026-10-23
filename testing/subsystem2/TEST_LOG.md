# Subsystem 2 — test log

**Subsystem:** Motion Control (Buddy 2)  ·  **Student ID:** 2303689  ·  **Code:** `app_program/subsystem2/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S2-U01 | example sample values stay within 0..100 (template example kept for S3/S4) | IF-01 | auto | `subsystem2_test.c` | `[TEST] S2-U01 PASS` |
| S2-U02 | example free slots and free blocks within pool size | IF-02 | auto | `subsystem2_test.c` | `[TEST] S2-U02 PASS` |
| S2-U03 | command word pack/unpack round trip, incl. negative angle and distance, ±180°, −128/+127 cm | `mbf_motion_cmd`, minimum-bit | auto | `subsystem2_test.c` | `[TEST] S2-U03 PASS` |
| S2-U04 | command fields clamped to their ranges; reserved bits stay 0 | `mbf_motion_cmd`, minimum-bit | auto | `subsystem2_test.c` | `[TEST] S2-U04 PASS` |
| S2-U05 | status word pack/unpack round trip; speeds clamped to −64..63; distance wraps at 4096 | `g_motion_status`, minimum-bit | auto | `subsystem2_test.c` | `[TEST] S2-U05 PASS` |
| S2-U06 | wrap-safe distance difference: 4093→3 = +6, 3→4093 = −6 | `g_motion_status` readers | auto | `subsystem2_test.c` | `[TEST] S2-U06 PASS` |
| S2-U07 | encoder 8-bit delta across the wrap (250→3 = 9) | encoder protocol (Part C row 3) | auto | `subsystem2_test.c` | `[TEST] S2-U07 PASS` |
| S2-U08 | MOVE ignored and motors braked while E-STOP is set; STOPPED after E-STOP cleared | NFR9, FR28, `flg_safety` | auto | `subsystem2_test.c` (wheels off the table) | `[TEST] S2-U08 PASS` |
| S2-U09 | `stopped` flag = 1 in STOPPED with both wheels still | Obstacle scan gate (Part C row 6) | auto | `subsystem2_test.c` | `[TEST] S2-U09 PASS` |
| S2-M01 | encoder counts per wheel revolution (sets `S2_ENC_CPR`) | FR8, A2 | manual | Hand-turn each wheel 10 revolutions, read the counts in the debugger / a print | 10 × CPR ± 1 on both wheels |
| S2-M02 | motor direction and brake per wheel | truth table, FR28 | manual | Wheels off the table: forward, reverse, brake on each motor; set `S2_INVERT_*` if needed | correct direction; brake stops the wheel |
| S2-M03 | straight distance accuracy | FR8, A2 | manual | 10 × MOVE(d = 100 cm) on the floor, measure with a tape | each within ±5 cm (±5 %) |
| S2-M04 | turn accuracy | FR8, A2 | manual | 10 × TURN(90°) and 10 × TURN(180°), measure the heading | each within ±5 % |
| S2-M05 | stall detection | statechart 6.2 STALL_FAULT | manual | Hold one wheel while driving at 20 cm/s | STALL_FAULT within 0.3 s (0.5 s at start), FAULT_STALL set, SENSOR_FAULT sent |
| S2-M06 | constant speed over the hump | FR (hold speed over humps) | manual | Drive over the hump in SPEED_FOLLOW, log the speed field | speed returns to target within 0.5 s, no stop |
| S2-T01 | E-STOP → brake latency | NFR10 (≤ 100 ms) | manual (logic analyser) | Set E-STOP from the source task; measure source edge → GP8..11 all LOW | ≤ 100 ms worst case over 20 trials |
| S2-T02 | Motion task period, jitter and WCET with Wi-Fi active, before vs after packing | NFR14, A6 | manual (GPIO toggle) | Toggle a spare pin at the start/end of `motion_cycle`, scope it | period 20 ms ± 1 tick; WCET well below 20 ms |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S2-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
