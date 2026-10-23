# context.md — IS10 Autonomous Robot Car (INF2004 Embedded Systems)

Hand-over context for a new Claude Code session working in this repository.
Read this whole file before editing code. It combines (1) what the code in this
repo actually contains today and (2) the team's agreed design (Week 6 Design
Review, Week 8 Tribunal 1, post-tribunal gap check, and the "minimum-bit"
change requirement).

Repo: https://github.com/mentaikos/INF2004-EMBEDDED (branch `main`, snapshot
taken at commit `8f63ff0`). Group ID: **IS10**. The person using this session is
**Buddy 2 (Motion Control: motors, encoders, PID)**.

Legend used below: **[CODE]** = verified in the repo; **[DESIGN]** = agreed in
team documents, not yet implemented; **[VERIFY]** = unconfirmed, check before
relying on it; **[DECIDE]** = needs a team decision, do not change silently.

---

## 0. Rules for the assistant

1. **Current state: the application code is still the course example template.**
   `app_program/subsystem1..5/` contains an LED / producer / consumer / monitor
   demo, not robot code. Robot subsystems must be written from scratch,
   following the template's patterns (section 5).
2. **Five developers share this repo.** Each `app_program/subsystemN/` folder
   belongs to one buddy (section 2). Only edit another buddy's folder, or a shared
   file (`app_program/app.h`, `app_main.c`, `config/*`, `lib/*`, `kernel/*`,
   `device/*`), when the user explicitly asks. Then say clearly what changed and
   why, because it affects other people.
3. **Shared interfaces are a contract.** Object names, message layouts, bit
   positions, priorities and pins in sections 6–8 were agreed in tribunal
   documents. Do not rename or re-layout them unilaterally. If a change is needed,
   propose it and flag it as **[DECIDE]**.
4. **Kernel/BSP code (`kernel/`, `lib/`, `device/`, `include/`) is a ported
   micro T-Kernel 3.0 with a custom SMP RP2040 port.** Change it only when
   unavoidable, keep the change minimal, and document it.
5. **Never commit `config/wifi_secrets.h`** (gitignored). Never commit build
   outputs or video files.
6. You cannot flash or run the hardware. When behaviour depends on hardware,
   write the code plus an on-target test (`TEST_CHECK`) and tell the user what to
   measure.
7. Prefer integer / fixed-point maths: the RP2040 has no FPU (`-mfloat-abi=soft`)
   and `tm_printf` has **no `%f`**.

---

## 1. Project summary

An autonomous robot car built on a **Raspberry Pi Pico W mounted on a Cytron
Robo Pico board**, running **micro T-Kernel 3.0 (µT-Kernel)** in C. The car must:

| Pillar | What it does |
|---|---|
| FOLLOW | Track a black line with two digital IR sensors |
| DECODE | Read navigation barcodes beside the line (right IR sensor) → Left / Right / Straight / U-Turn at the next junction |
| MEASURE | Detect humps with the IMU and estimate the peak hump height |
| AVOID | Detect an obstacle with a servo-mounted ultrasonic sensor, profile it, bypass it, rejoin the line |
| REPORT | Publish telemetry over Wi-Fi/MQTT; accept only START / STOP / E-STOP remotely |

Navigation is fully autonomous (FR4). Losing Wi-Fi does **not** stop the robot.

---

## 2. Team and ownership

| Buddy | Student ID | Subsystem | Folder (assumed) |
|---|---|---|---|
| Buddy 1 | 2502039 | Wi-Fi, MQTT, telemetry, command handling | `app_program/subsystem1/` |
| **Buddy 2** | 2303689 | **Motion control: motors, encoders, PID** | **`app_program/subsystem2/`** |
| Buddy 3 | 2502193 | Line following (IR) and barcode decoding | `app_program/subsystem3/` |
| Buddy 4 | 2500701 | IMU: tilt, humps, shock, heading | `app_program/subsystem4/` |
| Buddy 5 | 2503076 | Ultrasonic scanning, obstacle profiling, bypass, line-recovery monitoring | `app_program/subsystem5/` |
| (shared) | — | **Vehicle Controller (VC)**: mission state machine | **no owner or folder yet [DECIDE]** |

**[DECIDE]** The folder ↔ buddy mapping above is assumed from numbering. The
template headers still say `Student ID: <student ID>`. `app_program/subdir.mk`
compiles `app_program/*.c` and `app_program/*/*.c`, so a new folder (for example
`app_program/vehicle/`) can be added. Its `*_init()` / `*_start()` calls would
then need to be added to `usermain()` in `app_main.c` and declared in `app.h`
(shared files, team agreement).

The template refers to `TEAM.md` (pins, priority bands, interfaces) and
`docs/RELEASE.md` (safety rules). **Neither file is in this repo snapshot.**

---

## 3. Hardware

### 3.1 Parts
- Raspberry Pi Pico W (RP2040, dual Cortex-M0+ @ 125 MHz, 264 KB SRAM, CYW43439 radio).
- Cytron Robo Pico carrier: dual H-bridge motor driver, Grove ports, servo headers,
  NeoPixel (GP18), two buttons (GP20, GP21), piezo buzzer (GP22).
- 2 × DC gear motors with **single-channel** Hall encoders (6-pin PH2.0 connector:
  01 encoder GND, 02 encoder A, 03 encoder B (unused), 04 encoder VCC, 05 motor +,
  06 motor −).
- 3 × TCRT5000 / LM393 IR reflective modules (digital DOUT and analog AOUT).
- HC-SR04+ ultrasonic, **3.3 V variant** (powered from 3V3, so no level shifting needed on ECHO).
- SG90 servo (turns the ultrasonic sensor).
- LSM303DLHC IMU (GY-511): accelerometer (I2C addr 0x19) + magnetometer (0x1E). **No gyroscope.**

### 3.2 Pin map [DESIGN, agreed in the Week 6 report and tribunal]

| Signal | GPIO | Function | Owner | Notes |
|---|---|---|---|---|
| ENC_LEFT | GP0 | GPIO input, edge interrupt | B2 | Motor pin 02. GP0 is UART0 TX only if built with `CRASH_REPORT=1` or `CONSOLE=uart`, so **never use those build options** |
| IR_LEFT | GP2 | GPIO input (LM393 DOUT) | B3 | |
| US_TRIG | GP4 | GPIO output | B5 | Grove 3 |
| US_ECHO | GP5 | GPIO input, edge interrupt | B5 | Grove 3 |
| ENC_RIGHT | GP7 | GPIO input, edge interrupt | B2 | Grove 7, **shared cable with GP28 (split cable needed)** |
| MOTOR_R_A / _B | GP8 / GP9 | PWM slice 4 A/B | B2 | Robo Pico M1A / M1B |
| MOTOR_L_A / _B | GP10 / GP11 | PWM slice 5 A/B | B2 | Robo Pico M2A / M2B |
| SERVO_PWM | GP12 | PWM slice 6 A, 50 Hz | B5 | Servo header |
| IR_CENTER | GP16 | GPIO input (LM393 DOUT) | B3 | |
| IMU_SDA / IMU_SCL | GP26 / GP27 | I2C1 | B4 | Grove 6. Leave Grove 5 unused (also carries GP26) |
| IR_RIGHT | GP28 | ADC2 (AOUT) | B3 | Grove 7 shared with GP7. VBAT-sense jumper must stay unsoldered (A8) |
| Radio | GP23, 24, 25, 29 | CYW43439 | kernel | **Never touch** |
| Free | GP1, GP3, GP6, GP13–15, GP17, GP19 (+ GP18/20/21/22 if the onboard parts are unused) | | | GP13 shares PWM slice 6 with the servo (50 Hz) |

All sensors are powered from **3V3**.

### 3.3 Robo Pico motor driver truth table (per motor)
| A | B | Result |
|---|---|---|
| LOW | LOW | **Brake** |
| PWM/HIGH | LOW | Forward* |
| LOW | PWM/HIGH | Backward* |
| HIGH | HIGH | Coast (never used) |

\*Direction depends on wiring. **All four motor pins are PWM** (agreed): forward =
PWM on A with B LOW; reverse = PWM on B with A LOW; brake = both LOW (0 % duty
on both). Keep PWM ≤ 20 kHz.

---

## 4. What the repository contains [CODE]

```
app_program/            APPLICATION (the only place robot code goes)
  app.h                 shared app definitions (team agreement to change)
  app_main.c            usermain(): phase 1 subsystemN_init(), phase 2 subsystemN_start(), optional tests
  test_report.h         TEST_INIT / TEST_CHECK / TEST_SUMMARY macros
  subsystem1..4/        EXAMPLE code (LED, producer, consumer, monitor). To be replaced.
  subsystem5/           empty template
config/                 kernel config (config.h), device config, T-Monitor config, wifi_secrets.example.h
build_make/             makefile + pico_rp2040.mk (the real build). subdir.mk files are tracked source
kernel/, include/, lib/, device/   micro T-Kernel 3.0 + RP2040 SMP port, USB console, cyw43, lwIP, MQTT, drivers
etc/linker/pico_rp2040/ linker script
testing/                TEST_LOG.md per subsystem + integration + system, logs/ folders
videos/README.md        video index (unlisted YouTube links only)
tools/elf2uf2/          host tool built on first make
```
Many other board ports (`iote_*`, `nucleo_*`, `rx*`, `stm32*`) exist and are unused.

### 4.1 Build
```sh
export PICO_SDK_PATH=/path/to/pico-sdk          # needed for USB console, cyw43, lwIP, TinyUSB
# arm-none-eabi-gcc must be on PATH
cd build_make
make -j8                                         # default: SMP=1 CONSOLE=usb_cdc WIFI=cyw43 NET=none MQTT=0 TEST=0
make NET=lwip MQTT=1 -j8                          # with lwIP + MQTT (needs config/wifi_secrets.h)
make TEST=1 -j8                                   # test image: runs subsystemN_test() at boot
```
- The image name encodes the profile, e.g. `mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt_test.uf2`.
- Changing the profile discards old objects (`.build_profile` guard).
- `arm-none-eabi-size` runs after linking. Use it for the Week 13 RAM comparison.
- Console: USB CDC (115200 8N1). `CRASH_REPORT=0` (default) leaves GP0/GP1 free. **Keep it 0** because GP0 is the left encoder.
- Wi-Fi secrets: copy `config/wifi_secrets.example.h` → `config/wifi_secrets.h` (SSID, password, `MQTT_BROKER_IP`, port, client ID, topic).

---

## 5. Platform facts that change how code must be written [CODE]

### 5.1 Dual-core (SMP) by default
- `SMP=1` is the default. Tasks can run on **either core** unless pinned.
  `app_start_task(entry, pri, prc, name)` takes `prc = TP_PRC1` (core 0),
  `TP_PRC2` (core 1) or `0` (any core).
- Kernel service tasks are pinned to core 0: USB console (priority 3), CYW43/lwIP (priority 4).
- **Masking interrupts (DI/EI) only protects against the local core.** Use
  `ISpinLock`/`ISpinUnlock`, `atomic_*` (`<tk/smp.h>`), kernel objects (mutex,
  event flag, message buffer), **or** pin every task and ISR that shares the data
  to the same core.
- **[DECIDE]** The team documents (Part C row 3) say "all tasks and ISRs run on
  core 0, so interrupt masking is enough". That is only true if every robot task
  is created with `TP_PRC1` and GPIO/timer interrupts are enabled on core 0. The
  alternative is to build `SMP=0`. Until decided, write shared-data code that is
  safe on SMP (spinlocks or kernel objects), or pin to `TP_PRC1` and say so.

### 5.2 System tick is 10 ms, not 1 ms
- `config/config.h`: `CNF_TIMER_PERIOD 10`. `tk_dly_tsk`, cyclic and alarm
  handlers have **10 ms resolution**.
- 10 ms and 20 ms periods are fine. **1 ms barcode sampling cannot use a cyclic
  handler.** Use an RP2040 hardware TIMER alarm interrupt (TIMER_ALARMn / TIMER_INTE
  macros exist in `include/sys/sysdepend/cpu/rp2040/sysdef.h`; the kernel tick
  uses SysTick) or another hardware source.
- **[DECIDE]** The team report (A6) assumes a "1 ms µT-Kernel tick". Either change
  `CNF_TIMER_PERIOD` to 1 (team decision; more tick overhead) or update the report.
- For periodic tasks, use the template's pattern: a cyclic handler that calls
  `tk_wup_tsk()`, with the task in a `tk_slp_tsk()` loop (see `subsystem1/task_led.c`).
  Avoid drifting `tk_dly_tsk` loops.

### 5.3 Priorities
- 1 = highest. `CNF_MAX_TSKPRI 32`. Initial task (usermain) = 1. USB console = 3, CYW43/lwIP task = 4.
- The template assigns priority bands per subsystem: S1 12–13, S2 14–15, S3 16–17, S4 18–19, S5 20–21.
- The team design uses different numbers (section 7). **[DECIDE]** Agree one scheme before coding priorities.

### 5.4 Board support library (`<bsp/libbsp.h>`) is minimal
Only these exist: `gpio_set_pin(no, GPIO_MODE_OUT|GPIO_MODE_IN)`, `gpio_set_val`,
`gpio_get_val`, `pwm_set_pin`, `pwm_set_wrap`, `pwm_set_cc`, `pwm_set_enabled`.
**Missing; use registers via `in_w/out_w/set_w/clr_w` from `<tk/syslib.h>`:**
- **PWM clock divider:** `pwm_set_pin` leaves DIV at 1, so PWM = 125 MHz / (wrap+1).
  Motors: wrap 6249 gives 20 kHz. The servo's 50 Hz **needs** the DIV register
  (`PWM_BASE + 0x14*slice + PWM_CHx_DIV`, integer part in bits 11:4), e.g. DIV = 64
  and wrap = 39061.
- **GPIO pull-ups/downs:** `PADS_BANK0_BASE` + `GPIO_PUE`/`GPIO_PDE` macros exist.
- **GPIO edge interrupts:** no API and no register macros. Use `tk_def_int` on
  IO_IRQ_BANK0 (IRQ **13**) and the IO_BANK0 registers (INTR0 at `0x400140F0`,
  PROC0_INTE0 at `0x40014100`, PROC0_INTS0 at `0x40014120`; 4 bits per GPIO:
  LEVEL_LOW, LEVEL_HIGH, EDGE_LOW, EDGE_HIGH; clear edges by writing INTR)
  **[VERIFY against the RP2040 datasheet §2.19.6]**.
  **The encoder ISR (B2, GP0/GP7) and the echo ISR (B5, GP5) share this one
  interrupt**, so the team needs **one shared dispatcher** that checks which pin
  fired (agreed in the shared-resource matrix). **[DECIDE]** where it lives (e.g.
  `app_program/gpio_irq.c`).
- The Pico SDK runtime is **not** linked (only some headers and sources for USB/cyw43). Don't call `gpio_set_irq_enabled_with_callback()` and similar.

### 5.5 Physical timer, ADC, I2C drivers
- `USE_PTMR 0`. If enabled, physical timers use **PWM slices** and would clash with motor/servo PWM. Keep it off.
- **ADC driver** (`DEVCNF_USE_ADC 1`): one conversion per call, waits on an event
  flag, so it is **task context only**, not usable in a 1 ms ISR. Pin init flags are
  all FALSE: set `DEVCONF_ADC_PIN_INIT_2 TRUE` (GP28) in
  `device/adc/sysdepend/rp2040/adc_cnf_sysdep.h`, or read ADC registers directly in
  B3's ISR. Pick one owner of the ADC.
- **I2C driver is OFF** (`DEVCNF_USE_IIC 0`) and its **default pins clash with the
  robot**: `DEVCNF_I2C0_SDA/SCL = 8/9` (the **right motor**) and
  `DEVCNF_I2C1_SDA/SCL = 6/7` (GP7 = **right encoder**). `knl_start_device()`
  (`kernel/sysdepend/pico_rp2040/devinit.c`) only calls `dev_init_i2c(0)` (I2C0,
  "iica"). For the IMU on I2C1 GP26/27: set the I2C1 pins to 26/27, initialise
  **unit 1** ("iicb") instead of unit 0, and make sure I2C0 never claims GP8/9.
  **[VERIFY]** the unit-to-peripheral mapping in `i2c_rp2040.c`. This is Buddy 4's
  area but it **can break Buddy 2's motors**.
- Serial device (`DEVCNF_USE_SER 1`) is the UART0 driver. The console is on USB.

### 5.6 Wi-Fi / lwIP / MQTT
- lwIP runs **NO_SYS=1**. Everything that touches cyw43/lwIP/MQTT must run **only
  on `cyw43_utk_task`** (kernel task, priority 4, core 0, polls about every 2 ms
  but tick-bound to 10 ms).
- Application tasks **must not** call lwIP or `mqtt_*` directly. Data has to be
  handed over (e.g. message buffers `mbf_tx`/`mbf_rx`) and serviced inside that
  task's poll loop (`lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.c`) or in
  `lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c`. This is a kernel-library change owned by Buddy 1.
- `mqtt_utk.c` today is a **demo**: connects to `MQTT_BROKER_IP`, publishes
  "hb N" every 5 s on `MQTT_TOPIC`, subscribes to the same topic and prints what it
  receives. Keep-alive is 60 s.
- The team design's "WiFi/MQTT Network task (priority 18)" corresponds in practice to this kernel task (priority 4).

### 5.7 Kernel object limits (`config/config.h`)
Tasks 32, semaphores 16, event flags 16, mailboxes 12, mutexes 12, message buffers 12,
memory pools 12, fixed pools 12, cyclic handlers 12, alarm handlers 12.
`TASK_STACK_SIZE` = 2048 bytes (`app.h`); `tm_printf` needs about 1 KB.

---

## 6. Coding conventions (copy the template) [CODE]

- Each subsystem has `subsystemN.h` (its **public** interface only: object IDs,
  message formats, priorities, pins) and `.c` files. Other subsystems include it as
  `"../subsystemN/subsystemN.h"`.
- `subsystemN_init()` creates kernel objects only. `subsystemN_start()` creates and
  starts tasks (`app_start_task`). `usermain()` calls **all** inits, then **all**
  starts. Returning from `usermain()` shuts the system down.
- The subsystem that **receives or owns** data creates the kernel object and exports its ID.
- Use `EXPORT` / `LOCAL` / `IMPORT`, T-Kernel types (`ID, ER, INT, UINT, W, UW, H, UH, B, UB, BOOL, FP, SYSTIM`), and `CHECK(er, "what")` for kernel calls.
- Drivers go in the owner's folder as `drv_<name>.c/.h`, called only by the task that owns the peripheral.
- Interrupt handlers (ISR, cyclic, alarm) run in interrupt context: keep them short; **no waits, no `tm_printf`**. Signal tasks with `tk_wup_tsk`, `tk_set_flg`, `tk_sig_sem`.
- Use non-blocking sends where the design says so (`TMO_POL`).
- Hold a mutex for as short a time as possible: copy, unlock, then work.
- Tests: add `TEST_CHECK("S2-U01", cond, "desc")` in `subsystemN_test.c` (built with `TEST=1`), plus a matching row in `testing/subsystemN/TEST_LOG.md`. Integration test IDs are `INT-xx`, system test IDs `SYS-xx`. Never edit or delete old result rows. Always record the commit hash.
- Videos: unlisted YouTube links in `videos/README.md` only.

---

## 7. Agreed architecture [DESIGN]

### 7.1 Tasks
| Task | Owner | Period / trigger | Priority (design) | Purpose |
|---|---|---|---|---|
| Encoder ISR | B2 | GPIO edge (GP0, GP7) | interrupt | Count pulses (+ timestamp) |
| Barcode ADC ISR | B3 | 1 ms hardware timer | interrupt | Sample GP28, store **edge times** in `bc_ring` (≥ 64 entries + overflow flag) |
| Echo ISR | B5 | GPIO edge (GP5) | interrupt | Timestamp echo, signal `sem_echo` |
| **Motion Control** | **B2** | **20 ms** | **10 (highest)** | Speed/distance, PID, MOVE/TURN/STOP, E-STOP enforcement; **only writer of the motor pins** |
| Line & Barcode | B3 | **20 ms** (changed from 30 ms) | 11 | Line state, steering, junction / line lost / found, barcode decode |
| Vehicle Controller | ? | event-driven (`mbf_vc_evt`) | 12 | System statechart; **only task that sends motion commands** |
| IMU Monitor | B4 | 10 ms | 13 (**proposed 11**) | Pitch, heading, humps, shock, IMU fault |
| Obstacle Scan | B5 | 60 ms + on request | 14 | Forward watch, coarse/fine scan, profile, plan, monitor bypass/recovery |
| Command Handler | B1 | event-driven | 16 | Accept only START/STOP/E-STOP; E-STOP → `flg_safety` |
| WiFi/MQTT Network | B1 | 50 ms (in practice the kernel `cyw43_utk_task`, priority 4) | 18 | Only user of cyw43/lwIP |
| Heartbeat & Link Monitor | B1 | 1 s | 19 | Link-loss detection (switching to MQTT PINGREQ / keep-alive) |
| Telemetry Publisher | B1 | 500 ms | 20 (lowest) | Pack status → `mbf_tx` |

**[DECIDE]** Priority order: B4 wants IMU 13 → 11 (rate-monotonic), which collides with Line & Barcode at 11. Also see the template bands (5.3).

### 7.2 Kernel objects and interfaces
| Object | Type | Writer(s) → reader | Content |
|---|---|---|---|
| `mbf_motion_cmd` | message buffer | VC → Motion | MOVE(follow, v), MOVE(d), TURN(angle), STOP, SPEED_LIMIT(v) |
| `mbf_steer` | message buffer | Line&Barcode → Motion | steering correction (Motion keeps only the latest; ignored outside SPEED_FOLLOW) |
| `mbf_vc_evt` | message buffer | L&B, IMU, Obstacle, Motion, Cmd → VC | events (all senders `TMO_POL`; failures counted in telemetry) |
| `flg_safety` | event flag | IMU (SHOCK), Obstacle (TOO_CLOSE), Cmd (MQTT E-STOP) set; **VC clears on START** | E_STOP bit; Motion checks it every cycle |
| `flg_fault` / fault bits | event flag | each owner sets own bits | STALL, ENC_FAULT, IMU_FAULT, NO_ECHO, WIFI_LOST, BARCODE_ERR, LINE_TIMEOUT, SHOCK… **bit positions assigned by Buddy 1 [DECIDE]** |
| `flg_scan` | event flag | VC → Obstacle | SCAN_REQ, BYPASS_APPROVED, REJOIN_LINE, relayed LINE_FOUND/LINE_LOST |
| `flg_link` | event flag | Heartbeat ↔ Network | LINK_UP, RECONNECT |
| `g_status` + `mtx_status` | shared data + mutex (TA_INHERIT) | each task writes **only its own words**, under `mtx_status` | latest status for telemetry and other tasks |
| `enc_counts` | ISR-shared data | Encoder ISR → Motion only | see 8.2 |
| `bc_ring` | ISR ring buffer | Barcode ISR → L&B | edge times + overflow flag |
| `sem_echo` | semaphore | Echo ISR → Obstacle | 30 ms timeout per ping |
| `mbf_tx` / `mbf_rx` | message buffers | Telemetry/Heartbeat → Network; Network → Cmd | outgoing / incoming MQTT payloads |

Events in `mbf_vc_evt`: NAV_CMD, JUNCTION, LINE_LOST, LINE_FOUND, HUMP_START,
HUMP_END(height), OBSTACLE_DETECTED, OBSTACLE_PROFILE, NO_CLEAR, SENSOR_FAULT,
REJOIN_LINE_CONFIRMED, START, STOP, MOTION_DONE (+ MISSION_END).

### 7.3 System statechart (VC)
INIT → (self-test OK) IDLE → (START [IR & IMU calibrated] / clear E_STOP) RUNNING.
RUNNING contains: LINE_FOLLOW (initial), EXECUTE_NAV_CMD (turn at junction →
MOTION_DONE), HUMP_TRAVERSE (HUMP_START/HUMP_END, can also go to OBSTACLE_SCAN),
LINE_SEARCH (timeout 3 s, NFR18), OBSTACLE_SCAN (stop, SCAN_REQ; "no side clear" →
reverse and rescan), BYPASS_OBSTACLE, REJOIN_LINE (exit on REJOIN_LINE_CONFIRMED
from Obstacle Scan; timeout 3 s).
From RUNNING: E_STOP set → E_STOPPED (START [hazard cleared] resumes; STOP → IDLE);
SENSOR_FAULT / encoder stall / LINE_SEARCH or REJOIN timeout → FAULT
(STOP/reset [fault cleared] → IDLE); STOP (MQTT) → IDLE; MISSION_END → COMPLETE
(report peak hump; STOP → IDLE). Wi-Fi loss never leaves RUNNING.
Open: the state to resume in after E_STOPPED **[DECIDE]**; the finish condition (A9).

### 7.4 Key requirements (numbers that code must respect)
- NFR10: stop within **100 ms** of an E-STOP command, shock, obstacle too close, or motor stall (MQTT E-STOP is **best effort**).
- NFR14: Motion Control runs and checks E-STOP every **20 ms**. NFR9: no movement after E-STOP until a new START.
- NFR12: speed ≤ **10 cm/s** when an obstacle is within **20 cm**. NFR5: bypass < 3 s from detection to line reacquired (known conflict with NFR12).
- NFR6 telemetry every 500 ms; NFR17 heartbeat 1 s; NFR7 reconnect ≤ 10 s after detection; NFR19 Wi-Fi timeout 5 s, CONNACK 3 s, 1 s between retries.
- NFR8 line sensors every 20 ms (updated); NFR21 line lost only after > 150 ms of white; NFR22 discard a partial barcode after 200 ms with no edge; NFR23 0.5 s barcode cool-down; NFR24 ≥ 95 % barcode decode in 20 trials; NFR16 barcode sampled every 1 ms.
- NFR11 IMU read every 10 ms. NFR15 ultrasonic triggered every 60 ms.
- FR8 encoder turns Left/Right 90°, U-Turn 180°, straight moves. A2: distance and turns accurate to ±5 %.
- FR27 full stop on sudden impact. FR28 motors never move in a stopped state.

---

## 8. Minimum-bit change requirement (Week 8 injection, Week 13 evidence)

### 8.1 The requirement
Every application data field uses only the bits its valid range needs (C
bit-fields or masks/shifts). Fault conditions are single bits in a shared bitmask.
Applies to motion/encoder data, line/barcode, obstacle, hump, vehicle state, data
shared between tasks and with ISRs, inter-task messages, and **MQTT payloads
(compact binary, not text)**. Read-modify-write on shared packed data must be
race-free (atomic access or synchronisation).

**Week 13 deliverables (in the repo):** data representation table (field, range,
bits, representation); **RAM before vs after**; **message sizes before vs after**;
evidence of no lost functionality and no unacceptable timing degradation.
**Tag an unpacked baseline commit before packing anything.**

### 8.2 Team rules for packing
- The Cortex-M0+ has **no LDREX/STREX**: a bit-field update in a shared word is not atomic.
- **One writer per 32-bit word.** Readers read the whole aligned word, then unpack.
- `g_status` words are still accessed under `mtx_status` (team rule).
- ISR-written data is byte or halfword aligned; **no sub-byte bit-fields written in an ISR**.
- Fault bits use event-flag APIs (`tk_set_flg` / `tk_clr_flg`, atomic and ISR-safe).
- MQTT wire format uses **explicit masks and shifts**, not C bit-fields (bit-field layout is compiler-defined), plus a version field.
- Clamp every value to its range before packing. Sign-extend signed fields when unpacking.
- Calculation intermediates (PID terms, IMU height sum and filters) may stay full width. Stored/shared values are packed, and each persistent full-width variable must be justified individually (Part C row 2).
- Suggested: one shared header (e.g. `app_program/app_types.h`) with all packed types, range macros and `static_assert(sizeof(...))` checks **[DECIDE]**.

### 8.3 Agreed layouts per buddy [DESIGN]
- **Buddy 1:** binary fixed-length telemetry payload with a header, each buddy's minimal fields, and a **Wi-Fi-loss bit at the end**. Commands are 2 bits (START / STOP / E-STOP; the 4th code is reserved/invalid; a command ID is proposed). Link loss is detected by MQTT PINGREQ / keep-alive instead of dashboard acknowledgements. Byte order, signedness, version/length and fault-bit retention are still to be fixed (Part C row 5).
- **Buddy 2:** see section 9.
- **Buddy 3:** line state, barcode state and NAV_CMD 2 bits each; steering 1 byte (signed); telemetry 6 bits; `bc_ring` 64 entries + overflow flag; the ISR stores only edge times. On overflow: discard that barcode, send no NAV_CMD, restart reading (Part C row 4). Bar widths come from **edge times** (narrow/wide grouping), **not encoder counts**. Right-curve mitigation: if both sensors go white straight from ON_LINE, steer right with a limited correction at reduced speed for a short window before LINE_LOST.
- **Buddy 4:** IMU status 42 bits, written only by the IMU (state 3, pitch 9 signed, heading 9, turn rate 9 signed, hump height **5 bits (0–31 mm, 1 mm)**, hump count 4, calibrated 1, I2C fail 2). HUMP event 12 bits. SHOCK and IMU_FAULT bits in `flg_safety`. Height sum kept in 0.01 mm steps (int16), rounded once at HUMP_END. Start threshold **+3°**, plus wheelbase × sin(peak pitch). Shock threshold 2.5 g, with the accelerometer at **±4 g** and ≥ 100 Hz ODR. SHOCK bit latched until telemetry publishes it. On any abort after HUMP_START, send HUMP_END with height 0 (Part C rows 9, 10, 14).
- **Buddy 5:** state 3 bits, action 3 bits; fine scan in 5° steps from 30° to 150° (25 positions, 5 bits); distinct codes for near / out-of-range / no-echo; preserve SCAN_REQ-while-stopped, VC approval and REJOIN_LINE_CONFIRMED. Wait for Motion Control's `stopped` flag before scanning (Part C row 6).
- **VC (proposed):** each `mbf_vc_evt` message is one 32-bit word = 4-bit event type + packed payload.

---

## 9. Buddy 2 — Motion Control specification (the user's subsystem)

Implement in `app_program/subsystem2/`, replacing the producer example. The example's
`sem_slots` / `mpf_samples` / `SAMPLE_MSG` are used by the example subsystems 3 and 4;
coordinate before deleting them (they disappear when the other buddies replace their
examples too).

### 9.1 Planned modules
| File | Purpose | Rules |
|---|---|---|
| `motor_driver.c/.h` | `motor_set(wheel, signed duty)`, `motor_brake_all()` | Only called by Motion Control. Forward = PWM on A, B = 0; reverse = PWM on B, A = 0; brake = A = B = 0; never both high. PWM ≤ 20 kHz (e.g. wrap 6249 at DIV 1). Slices 4 (GP8/9) and 5 (GP10/11) |
| `encoder.c/.h` | Encoder ISR + `encoder_snapshot()` | ISR only increments a byte counter (and optionally a 16-bit µs timestamp); no kernel calls. Uses the shared GPIO-interrupt dispatcher with Buddy 5 (5.4) |
| `motion_control.c` | Motion Control task (statechart 6.2) | Only writer of GP8–GP11. Never blocks. Events sent `TMO_POL` |

### 9.2 Statechart 6.2
- **STOPPED** (initial, safe): A = B = LOW on both motors; PID integrators reset.
- **ACTIVE** superstate, every 20 ms: read the encoder snapshot → speed and distance → PID → PWM → write the g_status word.
  Internal: `SPEED_LIMIT(v)` updates the cap without changing state. A new MOVE/TURN pre-empts the current sub-state.
  - **SPEED_FOLLOW** (`MOVE(follow, v) [E_STOP clear]`): per-wheel PID to v_target plus the steering from `mbf_steer`; v ≤ VC speed limit. **Hold speed constant over humps** (no braking at HUMP_START).
  - **MOVE_DISTANCE** (`MOVE(d) [E_STOP clear]`): straight to target counts, forward or safe reverse; heading hold via the left/right count difference. Counts reached → MOTION_DONE → VC, back to STOPPED.
  - **TURN** (`TURN(angle) [E_STOP clear]`): wheels in opposite directions; target = angle × counts/degree; slow down for the last 10 %. Counts reached → MOTION_DONE → STOPPED.
- **STOP cmd** from any ACTIVE state → brake → STOPPED.
- **E_STOP_BRAKE**: E_STOP bit seen (checked every cycle) → A = B = LOW in the same cycle; motion commands ignored; back to STOPPED only after the VC clears E_STOP on a fresh START.
- **STALL_FAULT**: PWM > 0 and no encoder pulse for **300 ms** (arm only after a start-up window) → brake, set the STALL fault bit, send SENSOR_FAULT → VC; back to STOPPED only on reset by the VC.
- MQTT E-STOP is best effort. Worst-case on-board stop ≈ source period + 20 ms + spin-down; measure it. If it exceeds 100 ms, also wait on `flg_safety` between cycles (`tk_wai_flg` with a timeout of the remaining cycle time).

### 9.3 Packed data owned by Buddy 2
| Field | Range | Bits | Representation |
|---|---|---|---|
| Motion state | 6 states | 3 | bit-field in Motion's private struct (with dir_l 1, dir_r 1, braking 1 → 1 byte) |
| `mbf_motion_cmd` word | — | 26 of 32 | `[2:0]` type (5 cmds), `[8:3]` speed 0–50 cm/s, `[17:9]` angle −180…+180° signed, `[25:18]` distance −128…+127 cm signed |
| Encoder counters | wrapping; < 256 counts per read interval | 8 + 8 | two `uint8_t` in one **aligned halfword**. ISR: byte increment (atomic store). Motion: one halfword read = consistent L/R snapshot; delta = `(uint8_t)(new − old)`; sign from the **commanded direction** (single-channel encoders) |
| Edge timestamp (optional) | wraps every 65 ms | 16 | µs, for low-speed period measurement |
| **g_status motion word** | — | **32** | speed_l 7 (signed cm/s), speed_r 7, **distance 12 (1 mm/LSB, wrapping every 4096 mm)**, state 3, dir_l 1, dir_r 1, **stopped 1**. Written only by Motion, under `mtx_status` |
| Steering in | −31…+31 (agree with B3; B3 lists 1 byte) | 6–8 | from `mbf_steer` |
| Duty per wheel | 0–100 % (or 0–wrap if finer resolution is needed) | 7 (or 10–13) | choose for control quality and justify it |
| STALL, ENC_FAULT | bits | 1 each | fault flag; latched until the VC reset after STOP |
| MOTION_DONE, SENSOR_FAULT | events | 4-bit type | in the `mbf_vc_evt` word |
| PID state | integrator clamped ±I_MAX | 16 stored | per-cycle maths in 32-bit, not stored |

**Readers of distance** compute a wrap-safe **signed** difference:
`dd = ((new - old + 2048) & 0xFFF) - 2048` (tests: 4093→3 = +6 mm, 3→4093 = −6 mm).
Motion keeps the fractional mm internally so converting from counts does not drift.
Buddy 4 reads it every 10 ms; readers must sample at least every ~2 s.
Total distance for the COMPLETE summary needs a separate field (open with Buddy 1).

**`stopped` flag** = state STOPPED **and** both speeds 0 for 2 consecutive cycles
(40 ms). The VC sends SCAN_REQ only when `stopped` = 1.

### 9.4 Encoder protocol (Part C row 3)
Writer: Encoder ISR only. Reader: Motion Control only (Line & Barcode no longer uses
counts). Maximum interval between reads: 40 ms (one missed cycle). If the measured
interval exceeds 40 ms, or a delta exceeds the bound computed from measured
counts/rev × top speed, set **ENC_FAULT** and do not use the value. Other tasks get
distance only from the g_status motion word.

### 9.5 Constants to measure (put them in `subsystem2.h`, with placeholders until measured)
Counts per wheel revolution (single edge), wheel diameter, track width (for
counts/degree), top speed, minimum duty that starts the motor (deadband), PID gains,
spin-down time at cruise speed.

### 9.6 Buddy 2 tests to provide
- S2-U: pack/unpack round-trip of the command word and status word (incl. negative angle/distance, clamping); wrap-safe distance difference tests; encoder delta across a wrap.
- Manual: hand-rotate a wheel 10 revolutions and check the counts; 10 × 1 m straight and 10 × 90°/180° turns within ±5 % (A2); bench test each motor direction and the brake.
- Timing (A6, owned by B2): GPIO-toggle WCET and jitter of each task with Wi-Fi active, before vs after packing; E-STOP→brake latency on a logic analyser.

---

## 10. Open issues and integration gates

1. Robot code not started; all `app_program/` folders are examples or templates.
2. **[DECIDE]** SMP vs core pinning vs SMP=0 (5.1). The design's "core 0 only" safety argument needs `TP_PRC1` everywhere, or spinlocks.
3. **[DECIDE]** 10 ms tick vs the documented 1 ms tick (5.2); how the 1 ms barcode sampling is driven.
4. **[DECIDE]** Priority numbering (template bands vs design; IMU 11 vs Line & Barcode 11).
5. **[DECIDE]** VC owner and folder; resume state after E_STOPPED; finish condition (A9).
6. Shared GPIO IRQ dispatcher for the encoders (B2) and echo (B5): location and API.
7. I2C config: move I2C1 to GP26/27, initialise unit 1, keep I2C0 off GP8/9 (would hijack the right motor).
8. ADC: set the GP28 pin init, or have B3's ISR own the ADC directly; one owner.
9. Fault-bit assignments and the binary MQTT contract (Buddy 1); the handoff from app tasks into `cyw43_utk_task`.
10. Hardware gates: Grove 7 split cable (GP7 + GP28), VBAT jumper open, USB CDC only (no CRASH_REPORT), bench-test motor directions and brake before running.
11. Measurements still missing everywhere: CPR, WCET, RAM baseline, E-STOP latency.
