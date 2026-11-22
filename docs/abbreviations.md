# Project abbreviations (BARR-C:2018 rule 1.5b)

Abbreviations that appear in identifiers, macros and comments in
`app_program/` and `lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.*`.
Add a row here before using a new abbreviation in a name.

## Subsystems and owners

| Abbrev. | Meaning |
|---|---|
| `S1` / `s1_` | Subsystem 1: WiFi / MQTT telemetry (Buddy 1) |
| `S2` / `s2_` | Subsystem 2: Motion control (Buddy 2) |
| `S3` / `s3_` | Subsystem 3: Line following and barcode (Buddy 3) |
| `S4` / `s4_` | Subsystem 4: IMU (Buddy 4) |
| `S5` / `s5_` | Subsystem 5: Obstacle scan (Buddy 5) |
| `VC` / `vc_` | Vehicle Controller (shared) |
| `IF-nn` | Interface number in the Week 6 design review |
| `FR` / `NFR` | Functional / non-functional requirement |

## µT-Kernel object prefixes

| Abbrev. | Meaning |
|---|---|
| `tid_` | task ID |
| `cyc_` | cyclic handler ID |
| `alm_` | alarm handler ID |
| `mtx_` | mutex ID |
| `sem_` | semaphore ID |
| `flg_` | event flag ID |
| `mbf_` | message buffer ID |
| `mbx_` | mailbox ID |
| `mpf_` | fixed-size memory pool ID |
| `tk_`, `tm_` | µT-Kernel API / monitor (console) API |
| `TA_`, `TMO_`, `E_` | kernel attribute, timeout and error-code constants |
| `UB` `UH` `UW` `UD` | unsigned 8 / 16 / 32 / 64-bit (kernel types) |
| `B` `H` `W` `D` | signed 8 / 16 / 32 / 64-bit (kernel types) |
| `INT` `UINT` `BOOL` `ID` `ER` `FP` | kernel int, unsigned, boolean, object ID, error code, function pointer |
| `LOCAL` `EXPORT` `IMPORT` | kernel macros for `static`, global definition, `extern` |
| `_isr` | function that runs in interrupt context (BARR-C 6.5b) |
| `_task` | task entry function (BARR-C 6.4a) |
| `g_` | exported global variable (BARR-C 7.1j) |

## Hardware and protocols

| Abbrev. | Meaning |
|---|---|
| `ADC` | analogue-to-digital converter |
| `GPIO`, `IOB` | general-purpose I/O, I/O bank |
| `IRQ`, `INTE`, `intno` | interrupt request, interrupt enable register, interrupt number |
| `I2C`, `IC_` | I²C bus, RP2040 I²C controller registers |
| `PWM`, `CC` | pulse-width modulation, PWM compare register |
| `IR` | infrared line sensor |
| `IMU` | inertial measurement unit (LSM303 accelerometer + magnetometer) |
| `LSM303`, `DLHC` | the IMU chip and its variant |
| `enc` | wheel encoder |
| `MQTT`, `CONNACK`, `LWT` | MQTT protocol, connect acknowledgement, last will and testament |
| `cyw43`, `utk` | WiFi chip driver, µT-Kernel port layer |
| `WCET` | worst-case execution time |
| `ESTOP` | emergency stop |

## Data and units

| Abbrev. | Meaning |
|---|---|
| `sts` | packed status word |
| `tlm` | telemetry |
| `evt`, `ev` | event |
| `cmd` | command |
| `ack` | acknowledgement |
| `diag`, `dg` | diagnostics |
| `nav` | navigation command (barcode result) |
| `jct` | junction |
| `bc` | barcode |
| `lt` | line tracker state |
| `det` | detector |
| `hdg` | heading |
| `cal` | calibration |
| `tmo` | timeout |
| `ovf` | overflow |
| `sext` | sign-extend |
| `lpf` | low-pass filter |
| `avg` | average |
| `idx` | index |
| `cnt`, `n` | count |
| `dt` | time step |
| `ms`, `us` | milliseconds, microseconds |
| `mm`, `cm` | millimetres, centimetres |
| `mg` | milli-g (acceleration) |
| `dd` | deci-degrees (0.1°) |
| `mdeg` | milli-degrees |
| `dps` | degrees per second |
| `Q8`, `Q14` | fixed point with 8 / 14 fraction bits (256 = 1.0, 16384 = 1.0) |
| `kp`, `ki` | proportional / integral gain |
| `spd` | speed |
| `dist` | distance |
