# Tribunal 1 → code traceability

**Group:** IS10
**Checked against:** `IS10_PostTri1_final.pdf` (Versions A, B, C and the constraint-injection response)
**Code:** branch `refactor-barrc` (logic identical to `56c9e25`)

Each decision the team made in Tribunal 1 is listed with where it lives in the code.
- **Matches:** the code does what the report says.
- **Updated:** the code was changed after the tribunal, for a measured reason. The report is out of date; carry the new value into the Week 13 data-representation table. Don't change the code back.

No code was changed to produce this table.

## 1. Matches

| Tribunal decision | Where in the code |
|---|---|
| Binary MQTT, fixed-length frames, 4-bit header (2-bit type + 2-bit version) | `subsystem1/s1_codec.h` |
| 2-bit commands START / STOP / E-STOP | `S1_CMD_START/STOP/ESTOP`, `s1_codec.h` |
| WiFi-loss bit is the last data bit of telemetry, and stays set until it has been sent (Version C row 5) | bit 137, `s1_codec.h` |
| Telemetry every 500 ms, heartbeat every 1 s | `S1_TELEMETRY_MS`, `S1_HEARTBEAT_MS`, `subsystem1.h` |
| Command IDs; duplicates ignored | `RECENT_IDS`, `subsystem1.c` |
| MQTT Last Will "OFFLINE", retained | `mqtt_utk.c` (`will_msg`, `will_retain = 1`) |
| FAILED ack when a valid command can't be delivered | `S1_ACK_FAILED`, `subsystem1.c` |
| Motion Control is the only task driving the motors; priority 10, every 20 ms | `S2_PRI_MOTION`, `S2_PERIOD_MS`, `motion_control.c` |
| E-STOP brakes immediately with A = B = LOW; Motion doesn't wait for its 20 ms cycle (Version C row 11) | `S2_ESTOP_WAKE 1`, `brake_now()` |
| E-STOP set directly in `flg_safety`; only the VC clears it, on START | `s1_out_estop()` in `subsystem2.c`; `tk_clr_flg` in `task_vc.c` |
| Encoder: two 8-bit wrapping counters in one aligned halfword, written only by the ISR, no kernel calls (Version C row 3) | `ENC_COUNTS`, `encoder.h` |
| Motion status word: speed L/R 7+7, distance 12 (1 mm/LSB), state 3, directions 2, plus the `stopped` bit (Version C row 6) | `S2_STS_*`, `subsystem2.h` |
| `stopped` = STOPPED and both wheels still for 2 cycles (40 ms) | `s2_sts_pack()`, `motion_control.c` |
| Motion state, directions and braking flag as bit-fields in 1 byte | `mf` struct, `motion_control.c` |
| PID integrator clamped to ±I_MAX and stored in 16 bits (Version C row 2) | `H integ`, `S2_I_MAX_PM` |
| Wrap-safe 12-bit distance difference, `s2_dist_diff()` (Version C row 13) | `subsystem2.h`, used by S4 |
| Line sensors read every 20 ms (NFR8 changed from 30 ms) | `S3_PERIOD_MS 20` |
| Line state, barcode state and NAV_CMD 2 bits each; steering 1 byte | `S3_STS_*`, `NAV_BITS` in `app.h` |
| Steer towards the side the line was last seen on, before LINE_LOST | `side_left`, `s3_line_step()` |
| bc_ring holds 64 entries; the ISR stores only edge times; sticky overflow flag sent in telemetry | `S3_BC_RING_SIZE`, `S3_STS_OVF_SHIFT`, `s3_tlm_slot()` |
| On overflow, the barcode is discarded and no NAV_CMD is sent (Version C row 4) | `subsystem3.h:762`, `task_consumer.c` |
| Bar widths from edge times, grouped into narrow and wide; 0.5 s cool-down | `s3_bc_edge()`, `S3_BC_COOLDOWN_MS` |
| IMU priority 13 → 11, every 10 ms, on core 1 like Motion (Version C row 3) | `S4_PRI_IMU`, `S4_PERIOD_MS`, `S4_IMU_PRC` |
| No gyroscope: pitch from the accelerometer, heading and turn rate from the magnetometer | `imu_monitor.c` |
| IMU status 42 bits: state 3, pitch 9 signed, heading 9, rate 9 signed, calibrated 1 / hump 5, count 4, I2C fails 2 (Version C row 14) | `s4_pack.h` |
| Both IMU status words written together under `mtx_s4_status` (`TA_INHERIT`) | `subsystem4.c`, `task_imu.c` |
| Height sum in 0.01 mm steps as int16 (±6300), rounded once at HUMP_END | `hump_sum_cmm`, `IMU_SUM_MAX_CMM` |
| Start threshold +3°, plus wheelbase × sin(peak pitch) | `IMU_PITCH_START_MDEG 3000`, `IMU_WHEELBASE_MM` |
| HUMP_END(0) on any abort after HUMP_START (Version C row 9) | `imu_monitor.c:165`, test S4-U10 |
| Hold speed constant over humps (Buddy 2's Version A position) | VC: no speed change on `EVT_HUMP_START` |
| Obstacle Scan: priority 14, ping every 60 ms, coarse 30/60/90/120/150°, fine 5° steps (25 positions, 5-bit index) | `S5_PRI_MAIN`, `S5_ANGLES 25` |
| Range codes near / far / no-echo distinct (Version B constraint response) | `S5_RANGE_NEAR/FAR/NO_ECHO/UNSCANNED` |
| 3-bit scan state, 3-bit action | `S5_ST_*`, `S5_ACT_*`, `s5_sts_state()` |
| Separate critical stop distance | `S5_CRITICAL_MM 50` (5 cm), `obstacle_scan.h`; ping period `S5_PERIOD_MS 60` |
| SCAN_REQ only after `stopped = 1`; VC approves the bypass; REJOIN_LINE_CONFIRMED uses Line & Barcode observations | `task_vc.c`, `task_scan.c` |
| Events to `mbf_vc_evt` sent with `TMO_POL` (non-blocking) | every `tk_snd_mbf(mbf_vc_evt, …)` |
| Shared data under mutexes with `TA_INHERIT` | `mtx_s1_tlm`, `mtx_s3_status`, `mtx_s4_status`, `mtx_s5_status` |

## 2. Updated after the tribunal (change the document, not the code)

| Report says | Code does | Why | Owner |
|---|---|---|---|
| Motion command word: type 3, speed 6, angle 9 signed, distance 8 signed | Type 3 (5 types, adding `SPEED_LIMIT`), speed 9 (mm/s), angle 9 signed, distance 11 (mm) signed | 6 bits can't hold the cruise speed in mm/s, and 8 bits can't hold a bypass leg in mm. Each width covers its field's valid range, so it still meets the minimum-bit rule. | Buddy 2 |
| S3 telemetry cut to 6 bits | 8 bits: 6 data + BARCODE_ERR + overflow | Version B also decided to send the overflow flag to telemetry, so the slot grew by those bits. | Buddy 3 |
| Barcode sampled every 1 ms (NFR16) | Every 250 µs | At 1 ms, narrow and wide bars overlapped at cruise speed. | Buddy 3 |
| Line loss after 150 ms (NFR21) | 150 ms marks a gap; LINE_LOST after 1 s | 150 ms triggered on normal weaving. | Buddy 3 |
| SHOCK (> 2.5 g) → E-STOP; FAULT_SHOCK 0x80; SHOCK telemetry bit held for 1 s (Version C row 10) | LIFTED / OVERTURNED → E-STOP; `FAULT_LIFTED` 0x0080; lifted bit held for ≥ 1 s in the S4 slot | Professor's update: shock detection removed, lift detection added. Same bit, latch and hold rule. | Buddy 4 |
| HUMP event = one 16-bit word (12 bits) | One 32-bit `mbf_vc_evt` word: 4-bit type + 5-bit height | Version C row 5 asked for one common event format; every buddy uses `EVT_PACK` from `app.h`. | Buddy 4 |
| S4 telemetry: 42-bit status | 43-bit slot = 42 status + lifted bit | The lifted bit replaces the SHOCK bit. (22 Nov: a spare always-zero bit was removed.) | Buddy 4 |
| One shared `g_status` under `mtx_status` | One status word per subsystem, each with one writer: S2 is a single 32-bit word (atomic store), and S3/S4/S5 each have their own `TA_INHERIT` mutex | Removes the cross-buddy read-modify-write risk R1 completely. | All |
| MQTT E-STOP via the "Network task (priority 18)" | Command Handler (priority 16) sets `flg_safety` directly; the radio is the kernel's priority-4 `cyw43` task | No application network task was needed. The dashboard E-STOP is still best effort, because it depends on WiFi. | Buddy 1 |
| Fault-bit list "not yet assigned" (Version C row 5) | Assigned in `app.h` (`FAULT_STALL` … `FAULT_IR_RIGHT`) | Closed. | All |
| 4th 2-bit command encoding undefined (Version C row 5) | `11` in a 2-byte message = INVALID; `11` in a 4-byte message = MAP (barcode letter → command) | Closed; MAP is configuration only. | Buddy 1 |
| 1 ms kernel tick (A6) | 10 ms (`CNF_TIMER_PERIOD`) | Every task's timing is built on 10 ms; don't change it. | Buddy 2 |
| Obstacle detect at 20 cm | 21 cm | The robot coasts; stricter than required. | Buddy 5 |
| Detection → confirmed rejoin < 3 s | About 12–15 s (estimate, still to be timed) | Speeding the bypass up to 3 s would break it; set the requirement from a measured value. | Buddy 5 |
| A9 finish condition unknown | Lifting the robot ends the run | Closed. | Buddy 3 / 5 |

## 2a. Minimum-bit check (22 Nov)

Every shared field was checked against its valid range: the S1 messages, the S2/S3/S4/S5/VC status words, the event payloads, and the telemetry slots. All fields are sized to their range except two, both in the telemetry frame, and both now fixed:

| Where | Before | After |
|---|---|---|
| S5 telemetry slot | 55 bits, `[54:47]` always 0 | 47 bits |
| S4 telemetry slot | 44 bits, bit 0 always 0 ("spare") | 43 bits |
| Telemetry frame | 138 bits → 18 bytes | **129 bits → 17 bytes**, `S1_VERSION` 1 → 2 |

- **Robot behaviour:** unchanged; telemetry is output only. In the firmware, only `s1_codec.c`, `subsystem1.c` and the IMU task's `publish()` changed.
- **Dashboard:** `tools/s1_dashboard.py` was updated to match. A frame built with the firmware's own `s1_codec.c` and slot functions decodes back field-for-field with the dashboard decoder.
- **Old logs:** version-1 frames (18 bytes) are rejected by the new decoder, as the version field intends.

**Kept, with a reason:**

- **S2 speed: 7-bit signed cm/s** (±63), while the commanded maximum is 26.5 cm/s. The field holds the *measured* wheel speed, and the motors' no-load speed hasn't been measured. Measure it, then narrow the field to 6 bits if the no-load speed is ≤ 31 cm/s.
- **Padding to whole bytes** (command, ack and telemetry frames): MQTT payloads are byte strings, so a message can't be shorter than a whole number of bytes.

## 3. Still open (evidence, not code)

The report's open tests are still open: A1 reconnect × 20, A2 move and turn accuracy, A4 thresholds, A5 ultrasonic range, A6 WCET under WiFi, A8 VBAT jumper and the magnetometer-with-motors assumption (Version C row 12). The list in `REQUIREMENTS_AND_ISR_AUDIT.md` section 5 says who records each one.
