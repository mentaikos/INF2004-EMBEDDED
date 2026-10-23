# Buddy 5: ultrasonic scanning, profiling and line recovery

Student ID: **2503076**. This replaces the empty subsystem-5 template. All
implementation and test changes are confined to subsystem 5. `app_main.c`
already calls its init/start/test functions, and the makefile automatically
compiles new `.c` files in this folder.

## Report alignment

The Week 6 design review, section 6.5 and task table, assigns Buddy 5 sensing,
planning and monitoring. The **Vehicle Controller (VC) alone sends motion
commands**, and Buddy 2 alone writes motor GPIO. Week 8 retains this ownership
and adds compact data, a coherent echo handoff, fresh plans/line observations,
5-degree angle indices and a confirmed stop before scanning.

| Report requirement | Implementation |
|---|---|
| FR16, coarse then adaptive fine scan | Coarse 30, 60, 90, 120, 150 degrees; refine occupied/uncertain adjacent intervals in 5-degree steps |
| FR17, location, visible width, closest point and clearance | Integer polar projection, foreground extrema, conservative free-wedge chord |
| FR18/24/25, avoidance proposals | STOP, CONTINUE, LEFT, RIGHT, REVERSE_REATTEMPT; insufficient evidence gives NO_CLEAR |
| FR19/26, recovery confirmation | REJOIN_LINE starts a 3-second watchdog; fresh, distinct positive observations over at least 120 ms produce REJOIN_LINE_CONFIRMED |
| NFR12, at most 10 cm/s inside 20 cm | Confirmed obstacle event carries a 10 cm/s cap for VC enforcement; Buddy 5 never writes Motion's command buffer |
| NFR15, 60 ms ultrasonic period | Cyclic-paced task, physical trigger separation at least 60 ms; additional settling on large servo moves |
| NFR9/10, critical stop | Critical range immediately sets Buddy 2's existing E-STOP bit; Buddy 5 never clears it |
| Invalid measurements | Three consecutive invalid echoes report SENSOR_FAULT and latch E-STOP; at most three recovery pings, at least 1 second apart |
| Week 8 IMU / obstacle concern | During a hump, three confirming near samples instead of two reject short reflections; persistent hazards remain detectable and critical ranges are never masked |
| Week 8 stop/approval contract | SCAN_REQ is accepted only with Motion's `stopped` bit; movement during scanning aborts; generation and 1-second TTL checked on approval |
| Week 8 minimum bits | Explicit packed bins/profile/status/event payload; task-only state/action are 3 bits each; ISR uses aligned byte/halfword stores |

The supporting course write-up (Buddy 5, stages 1-5) and briefing (Buddy 5
slide) agree with these deliverables. Document instructions to commit, tag,
submit reports or publish videos are not actions requested in this session.
No commits, tags, pushes, flashing or external messages are made.

## Files and kernel objects

`subsystem5.c` creates objects in phase 1, then starts the task/cyclic in phase
2. `task_scan.c` runs the priority-14 task on `TP_PRC1`, paced by
`tk_cre_cyc` / `tk_wup_tsk` / `tk_slp_tsk`. `drv_ultrasonic.c` owns GP4 TRIG,
GP5 ECHO and GP12 servo PWM (slice 6 A). `obstacle_scan.c` contains the same
hardware-independent integer logic used by the target and host tests.

| Object | Owner / purpose |
|---|---|
| `sem_echo` | Buddy 5; ISR completion, initial count 0, maximum 1, 30 ms wait |
| `flg_scan` | Buddy 5; VC requests and relayed line observations |
| `mtx_s5_status` | Buddy 5; priority-inheritance mutex for snapshot and request metadata |
| 60 ms cyclic handler | Only wakes the scan task; no waits or console output |
| `flg_safety` | Existing Buddy 2 object; Buddy 5 only sets E-STOP |
| VC event/fault objects | Owned by VC; init-time binding through `s5_bind_vc` |

GPIO edges use the existing `subsystem2/gpio_irq.h` dispatcher. No other
buddy's ISR is replaced. ISR/task capture fields are protected with
interrupt-preserving Micro T-Kernel spinlocks. The task stays on core 0,
where the existing initial task registers the shared GPIO IRQ. Never move
only this task/dispatcher to another core without revisiting the drain/arm
protocol. There is no Pico SDK scheduling API or FreeRTOS dependency.

## Formulae and measurement model

- Time of flight: `distance_mm = (echo_us * 343 + 1000) / 2000`, from
  `distance = sound_speed * round_trip_time / 2`, using 343 m/s at room
  temperature. Compare limits in mm before rounding once to cm. A returned
  echo below 50 mm is NEAR; a returned echo beyond 1000 mm is FAR. An absent,
  malformed, late or stuck-high pulse is NO_ECHO, never FAR.
- Sensor angle: `degrees = 30 + 5 * index`, index 0..24. By default 30 points
  right, 90 forward and 150 left. Set `S5_SERVO_REVERSED` after the bench check
  if mounting/wiring reverses this convention.
- Geometry: `x_right = distance * cos(angle)`,
  `y_forward = distance * sin(angle)`. A Q15 sine table gives integer mm
  projections. The foreground band is the closest measured distance plus
  10 cm; visible width is `max(x) - min(x)` over foreground observations.
  Location is the closest visible return's `(x,y)`. This does not estimate
  hidden surfaces or the obstacle's full depth.
- Side clearance: `2 * minimum_side_range * sin(free_fan_angle / 2)`.
  Shrink the fan by 15 degrees for the beam and subtract range error. Missing
  echoes invalidate the affected side; unknown forward/foreground readings
  also block turn approval. The complete coarse endpoints must
  be measured; unscanned interior fine bins are covered by those endpoints
  under the static-scene assumption, rather than interpreted as new echoes.
  Coarse gaps can hide narrow objects; this is a proposal for monitored motion,
  not proof that the whole route is free. Choose a side only when its chord
  exceeds robot width plus margins and the closest return clears
  `ceil(sqrt((width/2)^2 + (length/2)^2)) + margins` for the turn.
- Servo: `pulse_us = min_us + (max_us-min_us)*degrees/180`; PWM divider 125
  and TOP 19999 give `125 MHz / 125 / 20000 = 50 Hz`. Only slice 6 changes.
  Settling is `20 ms + 2 ms * absolute_angle_change`, checked against the
  hardware clock. Five-degree steps can use consecutive 60 ms releases;
  large moves deliberately skip releases. No catch-up trigger bursts.

The timing/trigger and PWM models were checked against the
[HC-SR04 module datasheet](https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf)
and the [RP2040 datasheet](https://datasheets.raspberrypi.org/rp2040/rp2040-datasheet.pdf).
The robot's HC-SR04+ must have a 3.3 V-compatible ECHO, as specified in the
team pin map. Confirm the actual module and servo endpoint calibration.

## VC integration contract

There is **no Vehicle Controller implementation in this checkout**. Buddy 1
is still the LED template. This module therefore exposes the same binding
pattern as Buddies 2-4; creating a new controller is outside Buddy 5's scope.
Without a bound VC, stop/hold events increment `event_drops` and latch E-STOP
so they cannot be silently ignored. No scan/bypass runs automatically.

1. During VC init, after creating its message buffer and fault flag, call
   `s5_bind_vc(mbf_vc_evt, flg_fault)`. Complete binding before any tasks run.
2. On `EVT_OBSTACLE_DETECTED`, apply the 10 cm/s cap, command STOP, and wait
   until `s2_sts_stopped(motion_get_status())` is true. Call `s5_request_scan()`.
3. On `EVT_OBSTACLE_PROFILE`, read `S5_SNAPSHOT` with `s5_get_status()`. Match
   the event's generation to `s5_sts_generation(snapshot.status)` and inspect
   the profile. Approve using `s5_approve_plan(generation)` within 1 second.
   `E_OK` means queued; **wait for status BYPASSING (or FORWARD_WATCH for
   CONTINUE) before commanding movement**. A new scan/cancel/E-STOP or an
   expired plan invalidates approvals. Approvals are single-use.
4. On `EVT_NO_CLEAR`, hold. A reverse/reattempt is a VC proposal only: Buddy 5
   cannot measure rear clearance. VC owns its reverse policy, the reverse
   command and a fresh scan request after the next confirmed stop.
5. Monitor the bypass through fresh forward pings. VC initiates line search
   and calls `s5_rejoin_line()`. While recovering, relay **fresh Buddy 3 line
   samples** using `s5_line_observe(found)` at intervals no longer than 60 ms.
   Use calibrated, on-line observations without a both-white gap as positive
   samples. A latched LINE_FOUND event or cached status alone is insufficient.
   Any LINE_LOST relay cancels stability. Three distinct positive samples
   spanning at least 120 ms confirm recovery. Pre-recovery and stale samples
   cannot confirm it. VC resumes following on `EVT_REJOIN_LINE_CONFIRMED`.
6. VC calls `s5_cancel()` on STOP/mission exit. Recovery timeout or missing
   echo fault sets E-STOP and reports SENSOR_FAULT. Only VC clears E-STOP on
   a fresh START after hazards clear; `s5_reset()` never overrides E-STOP.

Read `subsystem5.h` for event payload bits and the `flg_scan` API. Snapshot
publication precedes event notification, so a higher-priority VC sees the
matching profile. All sends are `TMO_POL`; refused/unbound events are counted.
There is no assigned Buddy 5 MQTT slot in Buddy 1's current template. A future
publisher can read the compact snapshot without calling the network stack
from this task. Fault bits `0x0008` and `0x0040` follow the provisional team
list; confirm final assignments with Buddy 1 before integration.

## Stored data and Week 13 evidence

| Stored field | Range / bits | Representation |
|---|---|---|
| State / action | 8 / 5 values, 3 + 3 bits | Task-private bit-fields |
| Scan angle | 25 positions, 5 bits | Index; never a stored degree integer |
| Distance | Codes 0..3, distances 5..100 cm, 7 bits | Near, far, no-echo, unscanned distinct |
| All scan bins | 175 bits | 22 bytes; unused final bit zero |
| Current range | 7 bits | Private bit-field/status field |
| Pending scan positions | 25 bits | Explicit mask in a 32-bit container |
| Profile | 58 used bits | Two 32-bit words, 8 bytes |
| Closest / y / clearances | 0..100 and sentinels | 7 bits each |
| Visible width | 0..200 cm | 8 bits |
| Closest x | -100..100 cm | Signed 8-bit field, explicit sign extension |
| Visible edge positions | 0..24 | 5 bits each |
| Profile action / valid | 3 + 1 bits | Explicit fields |
| Status | 30 used bits | One 32-bit word |
| Generation / sample sequence | Modulo 256 | One byte each; generation validated with TTL/state |
| All logical deadlines | Modulo 65536 ms | Halfwords, deadlines <=3 seconds |
| ISR rise timestamp | Modulo 65536 us, 16 bits | Aligned halfword; capture <=30 ms |
| ISR pulse width | 0..30000 us, 15 bits | Aligned halfword to avoid ISR sub-byte RMW |
| ISR capture phase | 3 values | Whole byte per ISR rule |
| Event | Existing 4-bit type + <=28-bit payload | One 32-bit word, unchanged shared contract |
| Faults / request flags | Single bits | Kernel event flag APIs |
| Diagnostic counters | Wrapping | 16-bit event-drop and overrun counters |

Calculation intermediates remain full width to avoid truncating products.
The task's physical timestamps use full 32-bit microseconds for scheduler
jitter/servo checks and wrap-safe subtraction. Kernel IDs, spinlocks and API
objects retain their mandated types. Shared profile/request data uses a
priority-inheritance mutex; ISR capture uses the interrupt-preserving lock.

The host reports a 52-byte logic state. Scan bins use 22 bytes versus 100 for
an illustrative 25-element 32-bit array; that is a representation comparison,
**not a measured reduction from the original empty Buddy 5 implementation**.
There is no unpacked robot-scanner baseline here. Do not claim Week 13 RAM,
WCET, jitter or safety-latency improvements without equivalent measured builds.
The new task also needs the standard 2048-byte kernel stack and kernel objects.

## Build and verification

From the repo root, PC tests (GCC; `libm` generates test scenes only):

```sh
gcc -std=c11 -O2 -Wall -Wextra -Werror -DS5_HOST_TEST \
    app_program/subsystem5/host_test/test_host.c \
    app_program/subsystem5/obstacle_scan.c -lm -o /tmp/s5_test
/tmp/s5_test
```

Firmware uses the unchanged build system:

```sh
cd build_make
make TEST=1 -j8
```

Use USB CDC and `CRASH_REPORT=0` to preserve Buddy 2's GP0 encoder. The tests
in `scan_checks.h` are shared by the host runner and on-target `TEST_CHECK`
suite. Host results do not assert that hardware or on-target tests passed.
See `testing/subsystem5/TEST_LOG.md` for evidence and the remaining bench tests.

The default footprint (160 x 200 mm), error margins, critical distance,
foreground-band width, servo endpoints and settle-time constants are explicit
**calibration placeholders** in `obstacle_scan.h`. Room-temperature sound speed,
finite beam width, coarse sampling and 5-degree quantisation limit the model.
Measure NFR1 detection 20/20, NFR2 width error <=10%, NFR3 side selection
>=19/20, NFR4 rejoin 20/20, NFR5 detection-to-rejoin <3 seconds and NFR10 brake
latency <=100 ms. Large servo moves and low bypass speed may make NFR5
unachievable on the actual track; the code does not invent evidence for it.
