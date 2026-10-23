# Integration — test log

Tests that two or more subsystems work together. There is one test (or more) for every interface listed in TEAM.md section 5. Both owners agree on the test and sign off the result.

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| INT-01 | samples flow from S2 to S3 (≥ 5 in 2 s) | IF-01, IF-02 | auto | `subsystem3_test.c` | `[TEST] INT-01 PASS` |
| INT-02 | summary lines flow from S3 to S4 (≥ 2 in 3.5 s) | IF-03 | auto | `subsystem4_test.c` | `[TEST] INT-02 PASS` |
| INT-03 | S3 steering byte reaches S2 Motion Control every 20 ms, none refused (≥ 72 in 1.5 s) | `mbf_steer` (S3 → S2) | auto | `subsystem3_test.c` | `[TEST] INT-03 PASS` |
| INT-04 | S3 telemetry slot reaches the dashboard: frame bits 30..37 change with line state (move tape under sensors) and show NAV of a swiped barcode | IF-S1-01 `s1_tlm_set_slot` (S3 → S1) | manual | build with Buddy 1 merged + `NET=lwip MQTT=1`; watch `tools/s1_dashboard.py` | b3 = line(2) bc(2) nav(2) err(1) ovf(1) as described in `subsystem3.h` |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | INT-01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
