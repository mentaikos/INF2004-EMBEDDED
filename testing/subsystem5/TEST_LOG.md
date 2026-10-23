# Subsystem 5 — test log

**Subsystem:** Ultrasonic scan, profiling and recovery  ·  **Student ID:** 2503076  ·  **Code:** `app_program/subsystem5/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S5-U01 | example: runs in a task context | | auto | `subsystem5_test.c` | `[TEST] S5-U01 PASS` |
| S5-U02 | | | manual | 1. … 2. … | |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S5-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |

## Implemented test plan (2026-10-23)

The original template rows above are retained. S5-U01 still checks target
task context; S5-U02 remains reserved. These definitions apply to the new
implementation. `scan_checks.h` runs unchanged on PC and on the target.

| ID | What is tested | Requirement / interface | Method | Expected result |
|---|---|---|---|---|
| S5-U03 | Echo time-of-flight conversion | FR17 | host / target auto | 1166 us -> 20 cm; no pulse -> NO_ECHO |
| S5-U04 | Near / far / missing-echo codes | Week 8 compact ranges | host / target auto | Distinct codes before cm rounding |
| S5-U05 | Every value in every packed bin | Week 8 minimum bits | host / target auto | No neighbouring-bin or guard-byte corruption |
| S5-U06 | Angle, centre PWM and settling maths | FR16, A5 | host / target auto | 30..150 degrees, centre 1500 us, large moves settle longer |
| S5-U07 | Adaptive refinement | FR16 | host / target auto | Refines occupied/uncertain intervals, skips clear regions |
| S5-U08 | Polar geometry / clearance | FR17/25 | host / target auto | Correct width, packed limits and safe side |
| S5-U09 | Unknown side observations | FR18 | host / target auto | NO_ECHO invalidates affected side; neither side -> NO_CLEAR proposal |
| S5-U10 | Robot turning envelope | Week 8 footprint clearance | host / target auto | Rejects a side plan when the turn circle is occupied |
| S5-U11 | Motion stopped guard | Week 8 Part C row 6 | host / target auto | Reject moving request; abort a scan if stopped becomes false |
| S5-U12 | Plan generation approval | Week 8 fresh-plan contract | host / target auto | Old generation rejected, current accepted once |
| S5-U13 | Plan expiry / time wrap | Week 8 fresh-plan contract | host / target auto | Approval at 1-second deadline rejected across 16-bit wrap |
| S5-U14 | Simultaneous rescan / old approval | VC ownership | host / target auto | Rescan invalidates old approval |
| S5-U15 | Invalid echoes / bounded retries | Statechart 6.5 | host / target auto | Fault after 3 invalids; at most 3 retries, >=1 second apart |
| S5-U16 | Fresh stable line observations | FR19/26 | host / target auto | Old repeated observation cannot confirm; 3 fresh samples over 120 ms confirm once |
| S5-U17 | Lost-line relay during stability | FR26 | host / target auto | Resets confirmation counter |
| S5-U18 | Recovery watchdog / time wrap | NFR18 | host / target auto | Fault after 3 seconds, including across timer wrap |
| S5-U19 | Hump transient filter and critical range | NFR10, Week 8 IMU/scan concern | host / target auto | Two transient hump returns suppressed; persistent hazard detected; critical return immediately E-STOPs |
| S5-U20 | Approval during E-STOP | NFR9/10 | host / target auto | Plan invalidated; no bypass transition |
| S5-U21 | Kernel objects created | Micro T-Kernel integration | target auto | Semaphore, flag and mutex IDs positive |
| S5-U22 | CONTINUE requires approval | Statechart 6.5 | host / target auto | Clear scan waits for VC approval |
| S5-U23 | Sensor restored / reset during E-STOP | NFR9 | host / target auto | Restored echoes do not resume movement or clear E-STOP |
| S5-U24 | Stale line samples | Week 8 fresh-line contract | host / target auto | Changing sequence with stale time never confirms |
| S5-U25 | Unknown forward / foreground region | Safe turning envelope | host / target auto | Rejects side approval if the front or a foreground interior bin is unknown |
| S5-H01 | Independent 100-mm flat-board scene | FR17, NFR2 model check | host auto | Visible width 90..110 mm at 180 mm range |
| S5-H02 | Entire scene missing echoes | FR18, invalid ranges | host auto | No valid movement plan |
| S5-B01 | Full SMP firmware, TEST=1 | Link / shared interfaces | cross-build | ELF and UF2 produced with USB CDC and Wi-Fi |
| S5-B02 | Full SMP firmware, TEST=0 | Production build | cross-build | ELF and UF2 produced with USB CDC and Wi-Fi |
| S5-C01 | Strict ARM checks, SMP=0/1 | Portability / Micro T-Kernel API | cross-compile | No Buddy 5 warnings with -Wall -Wextra -Werror |
| S5-M01 | Servo / ultrasonic wiring and calibration | A5 | manual, pending | GP12 50 Hz, centre angle correct, GP5 safe voltage; settle time measured |
| S5-M02 | Distance / width / side selection | NFR1/2/3 | manual, pending | Detection 20/20, width error <=10%, side choice >=19/20 |
| S5-M03 | End-to-end bypass / recovery | NFR4/5 | manual, pending VC integration | Rejoin 20/20; measure detection-to-confirmation <3 seconds |
| S5-M04 | Critical distance -> brake | NFR10 | manual, pending | Scope GP5 and GP8..11, brake <=100 ms with Wi-Fi active |
| S5-M05 | Shared GPIO IRQ / missing echoes | B2/B5 integration | manual, pending | Encoders remain correct, no misattributed/late echoes with motors on/off |
| S5-M06 | STOP / stale plan / fresh line relays | VC integration | manual, pending VC integration | No movement before fresh approval; no rejoin from cached LINE_FOUND |
| S5-M07 | Hump transient filter with IMU | Week 8 IMU + obstacle safety | manual, pending | No false ordinary detection in 10 hump runs; persistent and critical hazards still detected |
| S5-M08 | RAM, WCET, jitter and latency baseline | Week 13 evidence | manual, pending | Equivalent before/after workload measurements; no inferred timing claims |

For M01, measure TRIG-to-TRIG spacing (>=60 ms), the 10-us TRIG pulse,
30-ms timeout and late/stuck-high behaviour with a scope. Measure real servo
settling for 5-, 30-, 60- and 120-degree moves before tuning constants.
M02 uses known boards at 5..100 cm and real obstacles at several angles;
record individual width errors, not only an average. M03/M06 need the VC
or an explicit fixture implementing the documented approval protocol.

No hardware was flashed or observed in this session. On-target S5-U01/U21
and all manual cases remain unrun. Host execution of shared tests does not
constitute an on-target PASS. The original empty template has no unpacked
robot scanner baseline, so Week 13 savings are not claimed. All evidence is
against **HEAD `68da3bd` plus the uncommitted Buddy 5 changes**; the user's
instruction prohibits committing or pushing.

### New execution records

| Date | Test IDs | Revision | Runner | Result | Evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-23 | S5-U03..U20, S5-U22..U25, S5-H01..H02 | `68da3bd` + uncommitted changes | Codex / host GCC | PASS, 24/24 | [Host log](logs/2026-10-23_host.txt) | Shared logic executed on PC; target context/object checks excluded |
| 2026-10-23 | S5-C01 | `68da3bd` + uncommitted changes | ARM GCC 15.2.Rel1 | PASS | [Strict ARM checks](logs/2026-10-23_arm-strict.txt) | All Buddy 5 C files, SMP=0/1 and TEST=0/1, warnings treated as errors |
| 2026-10-23 | S5-B01, S5-B02 | `68da3bd` + uncommitted changes | Existing make build / Pico SDK 2.3.1 | PASS | [Build log](logs/2026-10-23_build.txt) | Both complete SMP ELF/UF2 builds succeeded; no hardware execution |

The [SHA-256 source manifest](logs/2026-10-23_sources.sha256) identifies the
uncommitted Buddy 5 files used for these checks. No source was committed or
pushed. All S5-M cases and on-target S5-U01/U21 remain **PENDING**.
