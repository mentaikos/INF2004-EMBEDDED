# Subsystem 3 — test log

**Subsystem:** Line following & barcode decoding  ·  **Student ID:** 2502193  ·  **Code:** `app_program/subsystem3/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S3-U01 | (example consumer, temporary) statistics self-consistent |  | auto | `subsystem3_test.c` | `[TEST] S3-U01 PASS` |
| S3-U02 | Code 39 table: 44 unique patterns, 3 wide each | FR29 | auto | `subsystem3_test.c` | `[TEST] S3-U02 PASS` |
| S3-U03 | clean `*A*` decodes to LEFT | FR11, FR29 | auto | `subsystem3_test.c` | `[TEST] S3-U03 PASS` |
| S3-U04 | `*A*` read in reverse still decodes | FR11 | auto | `subsystem3_test.c` | `[TEST] S3-U04 PASS` |
| S3-U05 | every command decodes 20/20 with ±25 % width noise | NFR24 | auto | `subsystem3_test.c` | `[TEST] S3-U05 PASS` |
| S3-U06 | damaged barcode is rejected | FR31 | auto | `subsystem3_test.c` | `[TEST] S3-U06 PASS` |
| S3-U07 | valid but unknown barcode → BARCODE_ERR, back to WAIT_FOR_BAR | FR31 | auto | `subsystem3_test.c` | `[TEST] S3-U07 PASS` |
| S3-U08 | ADC trace `*D*` → U_TURN; repeat within 0.5 s ignored | FR12, NFR23 | auto | `subsystem3_test.c` | `[TEST] S3-U08 PASS` |
| S3-U09 | barcode read again after the cool-down | NFR23 | auto | `subsystem3_test.c` | `[TEST] S3-U09 PASS` |
| S3-U10 | partial read discarded after 200 ms without an edge | NFR22 | auto | `subsystem3_test.c` | `[TEST] S3-U10 PASS` |
| S3-U11 | left-only → DRIFTING, steer left; centre → ON_LINE, steer 0 | FR10 | auto | `subsystem3_test.c` | `[TEST] S3-U11 PASS` |
| S3-U12 | JUNCTION after 2 both-black samples, reported once | FR10 | auto | `subsystem3_test.c` | `[TEST] S3-U12 PASS` |
| S3-U13 | LINE_LOST only after > 150 ms white, then LINE_FOUND | FR30, NFR21 | auto | `subsystem3_test.c` | `[TEST] S3-U13 PASS` |
| S3-U14 | ~~binary MQTT status frame~~ **dropped**: Buddy 1 owns the telemetry payload; S3 no longer writes `mbf_tx` | — | — | — | — |
| S3-U15 | task every 20 ms, ISR every 1 ms, bc_ring never overflows | NFR8, NFR16, A6 | auto | `subsystem3_test.c` | `[TEST] S3-U15 PASS` |
| S3-U16 | bc_ring overflow mid-barcode → that barcode discarded, no NAV_CMD, next one read | 8.3 (Part C row 4) | auto | `subsystem3_test.c` | `[TEST] S3-U16 PASS` |
| S3-U17 | edge detector: hysteresis ignores noise; white baseline follows a lighting change | FR9 | auto | `subsystem3_test.c` | `[TEST] S3-U17 PASS` |
| S3-U18 | both white straight from ON_LINE → gap flag, limited right steer, not yet LINE_LOST | 8.3 right-curve mitigation, NFR21 | auto | `subsystem3_test.c` | `[TEST] S3-U18 PASS` |
| S3-U19 | packed status word `g_s3_status`: round-trip at range limits, negative pos/steer sign-extended, out-of-range clamped, bits 31:24 unused | minimum-bit (Week 8) | auto | `subsystem3_test.c` | `[TEST] S3-U19 PASS` |
| S3-U20 | MQTT slot for Buddy 1 (`S1_SLOT_B3`): 8 bits = line(2) bc(2) nav(2) err(1) overflow(1) | minimum-bit, FR3 | auto | `subsystem3_test.c` | `[TEST] S3-U20 PASS` |
| S3-M01 | white calibration on the track | FR9, A3 | manual | power up with IR_RIGHT on white, read console | `s3: calibrated, white=…` |
| S3-M02 | decode every official barcode 20× at three speeds | NFR24, A3 | manual | drive over each barcode, count `s3: barcode` lines | ≥ 19/20 each |
| S3-M03 | GP28 does not follow battery voltage | A8 | manual | unplug IR_RIGHT, vary supply, read `ir_right_raw` | no change |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S3-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
