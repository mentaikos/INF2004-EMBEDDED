# Vehicle Controller — test log

**Subsystem:** Vehicle Controller (system statechart, FR1)  ·  **Owner:** team  ·  **Code:** `app_program/vehicle_controller/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| VC-U01 | status word pack/unpack and clamping | `g_vc_status`, minimum-bit | auto | `vc_test.c` | `[TEST] VC-U01 PASS` |
| VC-U02 | barcode command → junction script (advance 5 cm, then TURN −90 / +90 / 180; STRAIGHT keeps following) | FR13, FR9 | auto | `vc_test.c` | `[TEST] VC-U02 PASS` |
| VC-U03 | bypass script geometry from Buddy 5's profile, both sides, clamped to the command ranges | FR19, FR25, `EVT_OBSTACLE_PROFILE` | auto | `vc_test.c` | `[TEST] VC-U03 PASS` |
| VC-U04 | LINE_SEARCH sweep alternates and starts towards the last line side | FR20, NFR18 | auto | `vc_test.c` | `[TEST] VC-U04 PASS` |
| VC-U05 | START hazard check: obstacle within 20 cm blocks START | NFR9, NFR12 | auto | `vc_test.c` | `[TEST] VC-U05 PASS` |
| VC-U06 | INIT → IDLE after the self-test | statechart | auto | `vc_test.c` | `[TEST] VC-U06 PASS` |
| VC-U07 | IDLE ignores driving events; robot stays stopped | FR28, statechart | auto | `vc_test.c` | `[TEST] VC-U07 PASS` |
| VC-U08 | STOP in IDLE stays IDLE | statechart | auto | `vc_test.c` | `[TEST] VC-U08 PASS` |
| VC-M01 | START refused until IR and IMU are calibrated | START guard | manual | Power on, send START at once (MQTT or `vc_post(EVT_START,0)`), then again after 3 s | first refused (`vc: START refused`), second → RUNNING |
| VC-M02 | junction with a pending barcode command | FR13, A7 | manual | Track: barcode A/B/D before a junction | robot turns left / right / U-turns at the junction, then follows |
| VC-M03 | line lost → LINE_SEARCH → found within 3 s; fault after 3 s off the line | NFR18 | manual | Lift the robot off the line while following | sweeps; back on the line → LINE_FOLLOW; otherwise FAULT, `FAULT_LINE_TIMEOUT` |
| VC-M04 | obstacle: stop, scan, bypass, rejoin | FR17–FR20, FR25, NFR5, NFR12 | manual | Box on the line | ≤ 10 cm/s cap, stop, scan, bypass on the clear side, REJOIN_LINE_CONFIRMED, follow; time it (NFR5) |
| VC-M05 | E-STOP → E_STOPPED; START with the hazard gone resumes; STOP → IDLE | NFR9, NFR10 | manual | Dashboard E-STOP / knock / hand at 4 cm | stops; START refused while the hand is there; resumes after |
| VC-M06 | sensor fault → FAULT; STOP resets only when the fault is gone | statechart | manual | Hold a wheel (stall) / unplug the ultrasonic | FAULT; STOP → IDLE (stall) / refused until replugged |
| VC-M07 | hump: speed held, peak height reported at COMPLETE | FR22, A4, A9 | manual | Run over the humps, then send `EVT_MISSION_END` | no braking on the hump; `vc: COMPLETE, peak hump N mm` |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | VC-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
