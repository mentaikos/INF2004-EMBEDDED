# Subsystem 3 — test log

**Subsystem:** Line following & barcode decoding  ·  **Student ID:** 2502193  ·  **Code:** `app_program/subsystem3/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S3-U01 | (example consumer, temporary) statistics self-consistent: min ≤ max, both within 0..100 | IF-01 | auto | `subsystem3_test.c` | `[TEST] S3-U01 PASS` |
| S3-U02 | Code 39 table: 44 unique patterns, 3 wide each | FR29 | auto | `subsystem3_test.c` | `[TEST] S3-U02 PASS` |
| S3-U03 | clean `*A*` decodes to the letter A (the VC maps letters to moves from the MQTT MAP) | FR11, FR29 | auto | `subsystem3_test.c` | `[TEST] S3-U03 PASS` |
| S3-U04 | `*A*` read in reverse (after a U-turn) still decodes to A | FR11 | auto | `subsystem3_test.c` | `[TEST] S3-U04 PASS` |
| S3-U05 | every letter A–Z decodes 20/20 with ±25 % width noise | NFR24 | auto | `subsystem3_test.c` | `[TEST] S3-U05 PASS` |
| S3-U06 | damaged barcode (one wide bar printed narrow) is rejected | FR31 | auto | `subsystem3_test.c` | `[TEST] S3-U06 PASS` |
| S3-U07 | valid Code 39 barcode that is not a letter (`*5*`) → BARCODE_ERR, back to WAIT_FOR_BAR | FR31 | auto | `subsystem3_test.c` | `[TEST] S3-U07 PASS` |
| S3-U08 | ADC trace `*F*` → NAV_CMD; a repeat within the 0.5 s cool-down is ignored | FR12, NFR23 | auto | `subsystem3_test.c` | `[TEST] S3-U08 PASS` |
| S3-U09 | barcode read again after the cool-down | NFR23 | auto | `subsystem3_test.c` | `[TEST] S3-U09 PASS` |
| S3-U10 | a read that stops half way is discarded (no BARCODE_ERR) once S3_BC_READ_MAX_MS (12 s) has passed since its first bar | NFR22 | auto | `subsystem3_test.c` | `[TEST] S3-U10 PASS` |
| S3-U11 | line between the sensors: line under left → steer left; under centre (right) → steer right; both white → ON_LINE, gentle steer to the last side | FR10 | auto | `subsystem3_test.c` | `[TEST] S3-U11 PASS` |
| S3-U12 | JUNCTION after 2 both-black samples, reported once; both white again → ON_LINE | FR10 | auto | `subsystem3_test.c` | `[TEST] S3-U12 PASS` |
| S3-U13 | both white is normal; LINE_LOST only after > S3_LINE_LOST_MS (1000 ms) white (≥ NFR21's 150 ms), then LINE_FOUND on either sensor | FR30, NFR21 | auto | `subsystem3_test.c` | `[TEST] S3-U13 PASS` |
| S3-U14 | ~~binary MQTT status frame~~ **dropped**: Buddy 1 owns the telemetry payload; S3 no longer writes `mbf_tx` | — | — | — | — |
| S3-U15 | task every 20 ms, ISR every S3_SAMPLE_US (0.25 ms, NFR16: ≤ 1 ms), bc_ring never overflows | NFR8, NFR16, A6 | auto | `subsystem3_test.c` | `[TEST] S3-U15 PASS` |
| S3-U16 | bc_ring overflow mid-barcode → that barcode discarded, no NAV_CMD, rest not read as a new barcode, next one read | 8.3 (Part C row 4) | auto | `subsystem3_test.c` | `[TEST] S3-U16 PASS` |
| S3-U17 | edge detector (low-contrast mode): each bar measured against the white next to it; noise and short spikes ignored; white baseline follows a lighting change | FR9 | auto | `subsystem3_test.c` | `[TEST] S3-U17 PASS` |
| S3-U18 | both white after a left touch → gentle left drift (smaller than an edge correction); gap only after 150 ms; lost line keeps the last side for the VC | FR10, 8.3 | auto | `subsystem3_test.c` | `[TEST] S3-U18 PASS` |
| S3-U19 | packed status word `g_s3_status`: round-trip at range limits, negative pos/steer sign-extended, out-of-range clamped, bits 31:24 unused | minimum-bit | auto | `subsystem3_test.c` | `[TEST] S3-U19 PASS` |
| S3-U20 | MQTT slot for Buddy 1 (`S1_SLOT_B3`): 8 bits = line(2) bc(2) nav(2) err(1) overflow(1), MSB first | minimum-bit, FR3 | auto | `subsystem3_test.c` | `[TEST] S3-U20 PASS` |
| S3-U21 | speed range: fast (50 cm/s, narrow 2 ms / wide 5 ms, ±15 % jitter) and slow (narrow 50 ms / wide 150 ms) barcodes each decode 20/20 | NFR24 | auto | `subsystem3_test.c` | `[TEST] S3-U21 PASS` |
| S3-U22 | not used | — | — | — | — |
| S3-U23 | not used | — | — | — | — |
| S3-U24 | not used | — | — | — | — |
| S3-U25 | every letter A–Z decodes to the same letter in both directions, direction reported | FR11 | auto | `subsystem3_test.c` | `[TEST] S3-U25 PASS` |
| S3-U26 | real `Y` read from the robot with a ~2× speed change inside the start `*` decodes, both ways | FR11, NFR24 | auto | `subsystem3_test.c` | `[TEST] S3-U26 PASS` |
| S3-U27 | after a very dark mark (track tape) the learnt black is forgotten after S3_BC_AMP_FORGET_MS (300 ms) of white, so weaker printed bars are seen again | FR9 | auto | `subsystem3_test.c` | `[TEST] S3-U27 PASS` |
| S3-U28 | 20 random marks on the mat never pass the `*` start check (steering limit never engages); the barcode right after them is read | FR11, NFR24 | auto | `subsystem3_test.c` | `[TEST] S3-U28 PASS` |
| S3-U29 | real faint small `Z` trace (thin bars +55..+70) reads as Z, both directions, exactly once | FR11, NFR24 | auto | `subsystem3_test.c` | `[TEST] S3-U29 PASS` |
| S3-U30 | slow pass (~4 cm/s, whole barcode ~8.6 s, longer than the 16-bit clock's 8 s) decodes; a read that stops half way is discarded | NFR22, NFR24 | auto | `subsystem3_test.c` | `[TEST] S3-U30 PASS` |
| S3-U31 | curve: line flickering on the right sensor still builds the bend past S3_BEND_CURVE; after an overshoot it rebuilds towards the side last seen | FR10 | auto | `subsystem3_test.c` | `[TEST] S3-U31 PASS` |
| S3-U32 | real `Z` on a sheet 20 / 38 counts darker than the learnt floor still reads, both directions | FR11, NFR24 | auto | `subsystem3_test.c` | `[TEST] S3-U32 PASS` |
| S3-U33 | real `Z` on a sheet 30 counts brighter than the floor: the floor after the barcode (looks like one more bar) is dropped, Z read once, both directions | FR11, NFR24 | auto | `subsystem3_test.c` | `[TEST] S3-U33 PASS` |
| S3-U34 | small `Y` on a darker sheet: paper edge before the barcode reads as 2 extra elements; they are dropped from the start and Y is read | FR11, NFR24 | auto | `subsystem3_test.c` | `[TEST] S3-U34 PASS` |
| S3-U35 | line tracker trim: repeated left-edge touches learn a left trim (bounded ±S3_TRIM_MAX); a right-edge touch moves it back | FR10 | auto | `subsystem3_test.c` | `[TEST] S3-U35 PASS` |
| S3-M01 | calibration at power-on | FR9 | manual | 1. Power on with the right IR sensor over **white**. 2. Power on again with it over **black**, then press START. | 1: serial `s3: calibrated, white=…`. 2: `s3: calibration failed …` and START refused (`vc: START refused: IR not calibrated`) |
| S3-M02 | read every barcode | FR11, NFR24 | manual | Drive over each barcode on the track 10 times. | Dashboard last-barcode shows the **correct letter** every time; write the count per barcode (e.g. A 10/10) |
| S3-M03 | both directions | FR11 | manual | Drive over a barcode, U-turn, drive back over it. | Same letter both ways (serial says `left->right` / `right->left`) |
| S3-M04 | small, medium and large barcodes | FR11, NFR24 | manual | Put each size on a straight, drive over it 10 times. | Correct letter at least 9/10 for each size |
| S3-M05 | barcode → turn at the next junction | FR12, FR29 | manual | Send a MAP. Put each mapped barcode before a junction, run. | Serial `vc: barcode '…' stored: … at the next junction`, and the robot does that move at the junction, 10/10 per letter |
| S3-M06 | damaged or unknown barcode | FR31 | manual | 1. Cover one bar of a barcode with white paper. 2. Use a letter that is not in the MAP. | 1: serial `s3: BARCODE_ERR …`, robot keeps following the line. 2: robot goes straight on at the junction |
| S3-M07 | read only once | NFR23 | manual | Drive very slowly over one barcode. | Only **one** `s3: BARCODE READ` line for that barcode |
| S3-M08 | line following ×10 | FR10, NFR8 | manual | 10 full laps of the track at normal speed. | Robot never leaves the line on straights or curves, 10/10 |
| S3-M09 | line lost and found | FR30, NFR18, NFR21 | manual | While driving: 1. Lift the front off the line for half a second and put it back. 2. Take the robot off the line and hold it over white for 5 s. | 1: keeps following, no stop. 2: after about 1 s it searches (wiggles); after about 3 more seconds it stops with FAULT |
| S3-M10 | not used | — | — | — | — |
| S3-M11 | line panel on the dashboard | FR3 | manual | WiFi build (`make NET=lwip MQTT=1`), `python tools/s1_dashboard.py`, drive. | Line state and last barcode on the dashboard match what the robot is doing |

All auto tests run in one go: `make TEST=1`, flash `…_test.uf2`, open the serial monitor (115200 baud) and wait ~20 s until it stops printing.

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Tester | Result | Notes |
|---|---|---|---|---|
| YYYY-MM-DD | S3-U01 | | PASS / FAIL | |
| 2026-11-23 | S3-U02 | Livia Poernomo | PASS | host logic test (macOS, kernel/hardware stubbed), not yet on target |
| 2026-11-23 | S3-U03 | Livia Poernomo | PASS | host logic test (macOS, kernel/hardware stubbed), not yet on target |
| 2026-11-23 | S3-U04 | Livia Poernomo | PASS | host logic test (macOS, kernel/hardware stubbed), not yet on target |
| 2026-11-23 | S3-U05 | Livia Poernomo | PASS | host logic test (macOS, kernel/hardware stubbed), not yet on target |
| 2026-11-23 | S3-U06 | Livia Poernomo | PASS | host logic test (macOS, kernel/hardware stubbed), not yet on target |
| 2026-11-23 | S3-U07 | Livia Poernomo | PASS | host logic test (macOS, kernel/hardware stubbed), not yet on target |
| 2026-11-23 | S3-U08 | Livia Poernomo | PASS | host logic test (macOS, kernel/hardware stubbed), not yet on target |
| 2026-11-23 | S3-U09 | Livia Poernomo | PASS | host logic test (macOS, kernel/hardware stubbed), not yet on target |
| 2026-11-23 | S3-U10 | Livia Poernomo | PASS | host logic test (macOS, kernel/hardware stubbed), not yet on target |
| 2026-11-23 | S3-U11 | Livia Poernomo | PASS | host logic test (macOS, kernel/hardware stubbed), not yet on target |
| 2026-11-23 | S3-U12 | Livia Poernomo | PASS | host logic test (macOS, kernel/hardware stubbed), not yet on target |
| 2026-11-23 | S3-U13 | Livia Poernomo | PASS | host logic test (macOS, kernel/hardware stubbed), not yet on target |
| 2026-11-23 | S3-U16 | Livia Poernomo | PASS | host logic test (macOS, kernel/hardware stubbed), not yet on target |
| 2026-11-23 | S3-U17 | Livia Poernomo | PASS | host logic test (macOS, kernel/hardware stubbed), not yet on target |
| 2026-11-23 | S3-U18 | Livia Poernomo | PASS | host logic test (macOS, kernel/hardware stubbed), not yet on target |
| 2026-11-23 | S3-U19 | Livia Poernomo | PASS | host logic test (macOS, kernel/hardware stubbed), not yet on target |
| 2026-11-23 | S3-U20 | Livia Poernomo | PASS | host logic test (macOS, kernel/hardware stubbed), not yet on target |
| 2026-11-23 | S3-U21 | Livia Poernomo | PASS | host logic test (macOS, kernel/hardware stubbed), not yet on target |
| 2026-11-23 | S3-U25 | Livia Poernomo | PASS | host logic test (macOS, kernel/hardware stubbed), not yet on target |
| 2026-11-23 | S3-U26 | Livia Poernomo | PASS | host logic test (macOS, kernel/hardware stubbed), not yet on target |
| 2026-11-23 | S3-U27 | Livia Poernomo | PASS | host logic test (macOS, kernel/hardware stubbed), not yet on target |
| 2026-11-23 | S3-U28 | Livia Poernomo | PASS | host logic test (macOS, kernel/hardware stubbed), not yet on target |
| 2026-11-23 | S3-U29 | Livia Poernomo | PASS | host logic test (macOS, kernel/hardware stubbed), not yet on target |
| 2026-11-23 | S3-U30 | Livia Poernomo | PASS | host logic test (macOS, kernel/hardware stubbed), not yet on target |
| 2026-11-23 | S3-U31 | Livia Poernomo | PASS | host logic test (macOS, kernel/hardware stubbed), not yet on target |
| 2026-11-23 | S3-U32 | Livia Poernomo | PASS | host logic test (macOS, kernel/hardware stubbed), not yet on target |
| 2026-11-23 | S3-U33 | Livia Poernomo | PASS | host logic test (macOS, kernel/hardware stubbed), not yet on target |
| 2026-11-23 | S3-U34 | Livia Poernomo | PASS | host logic test (macOS, kernel/hardware stubbed), not yet on target |
| 2026-11-23 | S3-U35 | Livia Poernomo | PASS | host logic test (macOS, kernel/hardware stubbed), not yet on target |
