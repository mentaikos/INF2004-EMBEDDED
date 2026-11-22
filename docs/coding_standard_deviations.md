# BARR-C:2018 conformance and deviation record

**Group:** IS10
**Baseline:** commit `56c9e25` ("fix for iMU"), the commit the BARR-C checklist was run on
**Branch:** `refactor-barrc`
**Scope:** all team C code: `app_program/**` and `lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c/.h` (55 files). µT-Kernel, Pico SDK, lwIP and cyw43 code are out of scope.

BARR-C allows a project to deviate from a rule if the deviation is written down with a reason. This file is that record. Section 1 lists what was fixed in the code. Section 2 lists the rules the team deviates from, and why. Section 3 gives every checklist rule's new status, for updating the checklist report.

**Rule followed for this refactor: the robot's behaviour must not change.** Where a fix would change logic, or rewrite lines another buddy is still working on, the rule is recorded as a deviation instead.

---

## 1. Fixed in the code

| Rule | Change | Where |
|---|---|---|
| 1.1d | Team headers use `static inline` directly instead of the kernel's `Inline` macro. The team's own `#define Inline` copies are removed from the PC-test type headers. | `subsystem2.h`, `encoder.h`, `s4_pack.h`, `subsystem4.h`, `obstacle_scan.h`, `subsystem5.h`, `vehicle_controller.h`, `imu_types.h`, `scan_types.h` |
| 3.3c | Every file ends with `/*** end of file ***/` and a newline. | All 55 files |
| 5.3a | `S3_LINE.pos` is a plain `B` (int8) instead of the signed bit-field `B pos : 3`. It holds −2..+2 and was already alone in byte 1, so the struct layout and size are unchanged. | `subsystem3.h` |
| 6.5b | All 7 interrupt, cyclic and alarm handlers are named `*_isr`: `gpio_irq_isr`, `bc_alarm_isr`, `motion_cyclic_isr`, `imu_cyclic_isr`, `line_barcode_cyclic_isr`, `s5_scan_cyclic_isr`, and `monitor_alarm_isr` (missing from the checklist). | S2, S3, S4, S5 |
| 7.1j | The 3 exported data variables are prefixed with `g_`: `g_s3_diag`, `g_app_stats`, `g_s4_lines_received`. | S3, S4 |
| 7.2c | The exported objects of `subsystem3.c` are now defined at the top of the file. | `subsystem3.c` |
| 8.2d | All 51 `if … else if` chains now end with an `else` (with a comment where nothing needs doing). | S1 2, S2 2, S3 12, S4 9, S5 6, VC 19, `mqtt_utk.c` 1 |
| 8.4c | The 11 `while (1)` task loops are `for (;;)`. | S1, S2, S3, S4, VC |
| 8.4d | The 6 empty loop bodies are `{ /* comment */ }`, including one in `subsystem3_test.c` that the checklist missed. Their `&&` operands are also bracketed (1.4b). | `subsystem3.c`, `drv_lsm303.c`, `subsystem3_test.c` |
| 1.5b | Abbreviations table added. | [`docs/abbreviations.md`](abbreviations.md) |
| — | The stale IMU copy `app_program/vehicle_controller/subsystem4/` is deleted. It was never compiled: the makefile only builds one folder level below `app_program/`. | — |

### How "no behaviour change" was checked

1. **Build:** the firmware was built before and after the refactor, in the normal (`TEST=0`) and self-test (`TEST=1`) configurations, with the `NET=lwip MQTT=1` profile. Two clean builds of the baseline produced identical binaries, so the build is reproducible.
2. **Per-file comparison:** the disassembly and data of every application object file was compared, with the renamed handlers and globals mapped back to the old names.
   - **In the normal build (`TEST=0`), 29 of the 31 object files are byte-for-byte identical.** This covers every change except the two below.
   - **`subsystem3.o` and `task_consumer.o` differ**, only in `s3_line_init`, `s3_line_step` and `line_barcode_task`, the three functions that use `S3_LINE.pos`. The old code read the 3-bit field with shifts; the new code uses a byte load or store. The values are identical for −2..+2.
   - Separately, adding the empty `else {}` blocks inside `s3_line_step` made GCC (`-O2`) order the basic blocks differently. Both versions contain exactly the same instructions (apart from 2 alignment `nop`s), and at `-O1` they compile identically. An empty compound statement has no effect in C.
   - **In the self-test build (`TEST=1`)**, `subsystem3_test.o` also differs. The empty-loop fix at line 461 added two lines, so the `__LINE__` numbers that `TEST_CHECK` prints on a failure moved by +2, and `test_barcode()` got the same kind of block reordering. No test logic changed.
3. **Warnings:** a one-off compile of all team code with `-Wall -Wextra -Wmisleading-indentation -Wparentheses -Wfloat-equal` gives **0 warnings** in both configurations.
4. **PC tests:** the S5 host test passes (26/26), as before. The S4 host test fails one height case (`height 0 mm vs true 25 mm`) **before and after** the refactor; that failure isn't caused by this work.

**Re-test on the robot:** line following (S3), because `s3_line_step` is the only firmware code that changed at the instruction level.

---

## 2. Deviations

| ID | Rules | What the code does | Reason | Mitigation / plan |
|---|---|---|---|---|
| D-01 | 1.1a | Built with `-std=gnu11`, not C99. | The µT-Kernel template and Pico SDK headers are built with GNU C11; the kernel isn't ours to change. | Team code uses no GNU extensions except two `__asm__("dmb")` barriers (1.1c). |
| D-02 | 1.1d, 6.1f | The `LOCAL`, `EXPORT`, `IMPORT` and kernel `Inline` macros are used. The PC-test type headers (`imu_types.h`, `scan_types.h`) define `LOCAL`/`EXPORT`/`IMPORT` the same way so the same source builds on a PC. | These are the µT-Kernel template's conventions (`include/tk/typedef.h`) and every template and kernel file uses them. They map 1:1 to `static` / nothing / `extern`. | Team headers no longer use `Inline` (section 1). |
| D-03 | 1.2a, 1.3b, 3.4a, 3.5a, 8.3a | Template layout: tabs, 8-column indent, `{` at end of line, lines over 80 columns, `break` indented with the case body. | Fixing these touches almost every line and would conflict with buddy work that isn't merged yet (e.g. the S5 width fix). Layout doesn't change generated code. | One formatter pass after all branches are merged: [`docs/barr-c.clang-format`](barr-c.clang-format). Then check the binary is unchanged, as in section 1. |
| D-04 | 1.3a, 1.4b, 3.3a, 8.1a, 8.6a | Some single-statement `if`/`for` bodies have no braces (~490), some `&&`/`\|\|` operands aren't bracketed (~330), there are about 110 lines with two statements, about 170 declarations with commas, and comparisons are written variable-first (`er == E_OK`, ~450). | About 1,500 mechanical edits across every buddy's files: too many to make by hand now, and they'd conflict with unmerged work. | The bugs these rules guard against are covered: **0 warnings** with `-Wmisleading-indentation` (misleading unbraced bodies) and `-Wparentheses` (assignment used as a test). 8.2c (no assignment in an `if` test) conforms. To be applied with `clang-tidy readability-braces-around-statements` and the formatter after merge. |
| D-05 | 1.6a | About 700 casts, mostly without comments. | Most are required by the µT-Kernel API (`(UB*)` for `tm_printf`, `(FP)` for handlers, `(void)` for unused parameters) or are explicit narrowing during bit packing. | Comment any new cast that isn't one of those three kinds. |
| D-06 | 2.2h | Function and module comments are plain block comments, not Doxygen. | Each file and function already has a structured header comment; converting the format adds no information. | Use `/** @brief */` for new code. |
| D-07 | 4.1b | `subsystem1` … `subsystem5` share their first 8 characters. | The course template fixes these folder and file names. | Within a folder, file names are unique. |
| D-08 | 4.2a | 18 `.c` files have no header of the same name. | Convention: each subsystem has **one** public header (`subsystemN.h` / `vehicle_controller.h`). Its other `.c` files (`task_*.c`, `motion_control.c`, `vc_plan.c`, …) are implementation parts of the same module, and the `*_test.c` files are only called from `app_main.c` in `TEST=1` builds. | The public interface of every module is in exactly one header. |
| D-09 | 5.1a | Type names are upper case (`S5_PROFILE`, `VC_DIAG`) without `_t`. | This follows the µT-Kernel naming used by every kernel type (`T_CTSK`, `T_CCYC`, …). The names appear in the interface contract agreed in the tribunal documents. | — |
| D-10 | 5.2a, 5.6a | The µT-Kernel types `UB`/`UH`/`UW`/`W`/`INT`/`BOOL` are used instead of `<stdint.h>` / `bool`. | These are the RTOS API types and are fixed-width typedefs, so the rule's intent (known widths) is met. | — |
| D-11 | 5.2b | `long` in the S4 PC test (`host_test/test_host.c`). | That file runs only on a PC, and `long` is the type of the C library functions it calls (`lroundf`, `labs`, `%ld`). | Not compiled into the firmware. |
| D-12 | 5.4 | Floating point in the IMU maths and the Code 39 fit. | Integer-only versions would have to be re-tuned and re-tested. All literals have the `f` suffix. | `-Wfloat-equal` gives 0 warnings (no float equality tests). |
| D-13 | 6.2a, 8.2b | 13 functions are longer than 100 lines, and some state machines nest `if` more than 2 deep. Eight of the 13 are firmware: `tick()`, `handle_event()`, `script_done()` (VC), `imu_step()`, `imu_task()` (S4), `s3_line_step()`, `barcode_cycle()` (S3) and `s5_make_profile()` (S5). The other five are test functions. | Splitting working, tested state machines is a logic change. This refactor isn't allowed to make one. | Each buddy splits their function when it is next changed for a functional reason. Test-function length has no effect on the robot. |
| D-14 | 6.3a | 33 function-like macros (`S2_CMD_PACK`, `EVT_PACK`, `S3_MS`, …). | Most are used where C needs a constant expression (`case` labels, array sizes, static initialisers), which a function can't provide. | Packing helpers that don't need a constant are already `static inline` functions. |
| D-15 | 6.5c | Cyclic handlers are `EXPORT`, not `static`. | They are registered with `tk_cre_cyc` from `subsystemN.c`, a different file from the handler. | Interrupt handlers registered in the same file are `LOCAL` (`static`). |
| D-16 | 7.1e, 7.1k, 7.1m | Short names (`i`, `n`, `lt`, `bc`), and no `p_` prefix on pointers or `b_` prefix on booleans. | About 800 identifiers, many in the shared interfaces agreed in the tribunal documents. Renaming them would break other buddies' code and the dashboard decoder. | New code follows the rules. Short names are listed in `docs/abbreviations.md`. |
| D-17 | 7.1j | 22 exported µT-Kernel object IDs (`mbf_vc_evt`, `flg_safety`, `tid_imu`, …) have no `g_` prefix. | They are the object names agreed in the tribunal interface documents. Their prefix says what kind of kernel object each one is (see `docs/abbreviations.md`). | All exported data variables now use `g_` (section 1). |
| D-18 | 8.4a | About 80 loops have literal bounds, e.g. `for (i = 0; i < 4; i++)`. | Most count fixed hardware or packet items (2 wheels, 4 bytes, 9 Code 39 elements) or are in test code. | Replace with named constants when the loop is next changed. |

### Advisory rules (BARR-C "preferred"), accepted as they are

- **1.7c:** one forward `goto`, in `drv_lsm303.c`. BARR-C allows forward jumps, so this conforms.
- **1.7d:** 22 uses of `continue`.
- **2.2g/i:** few `NOTE:` / `WARNING:` markers.
- **4.4a:** no team file templates.
- **6.1i:** some driver helpers (`drv_ir_*`) aren't prefixed with their file name.
- **6.2c:** about 100 functions have more than one `return`.

---

## 3. Status of every checklist rule

Legend: **Conforms**, **Fixed** (section 1), **Deviation D-nn** (section 2), **Advisory**.

| Rule | Before | Now |
|---|---|---|
| 1.1a | Partly | Deviation D-01 |
| 1.1c | Conforms | Conforms |
| 1.1d | Does not conform | Fixed (team code) + Deviation D-02 (kernel macros) |
| 1.2a | Does not conform | Deviation D-03 |
| 1.3a | Does not conform | Deviation D-04 |
| 1.3b | Does not conform | Deviation D-03 |
| 1.4b | Does not conform | Deviation D-04 |
| 1.5b | Does not conform | Fixed |
| 1.6a | Does not conform | Deviation D-05 |
| 1.7a/b | Conforms | Conforms |
| 1.7c | Partly | Conforms (forward `goto` allowed) |
| 1.7d | Partly | Advisory |
| 1.8a | Conforms | Conforms |
| 1.8c | Conforms | Conforms |
| 2.1c | Not checked | Conforms (scanned: no commented-out code found) |
| 2.2g/i | Partly | Advisory |
| 2.2h | Does not conform | Deviation D-06 |
| 3.1a | Conforms | Conforms |
| 3.3a | Does not conform | Deviation D-04 |
| 3.3c | Does not conform | Fixed |
| 3.4a | Does not conform | Deviation D-03 |
| 3.5a | Does not conform | Deviation D-03 |
| 3.6a | Conforms | Conforms |
| 4.1a | Conforms | Conforms |
| 4.1b | Does not conform | Deviation D-07 |
| 4.1d | Conforms | Conforms |
| 4.2a | Does not conform | Deviation D-08 |
| 4.2b | Conforms | Conforms |
| 4.3d/f | Conforms | Conforms |
| 4.4a | Partly | Advisory |
| 5.1a | Does not conform | Deviation D-09 |
| 5.2a | Partly | Deviation D-10 |
| 5.2b | Partly | Deviation D-11 (PC test only) |
| 5.3a | Does not conform | Fixed |
| 5.4 | Partly | Deviation D-12 |
| 5.5 | Conforms | Conforms |
| 5.6a | Partly | Deviation D-10 |
| 6.1e/f | Partly | Conforms (team code); kernel `Inline` under D-02 |
| 6.1i | Partly | Advisory |
| 6.2a | Does not conform | Deviation D-13 |
| 6.2c | Partly | Advisory |
| 6.2e | Conforms | Conforms |
| 6.3a | Partly | Deviation D-14 |
| 6.4a | Conforms | Conforms |
| 6.5b | Does not conform | Fixed (7 handlers) |
| 6.5c | Partly | Deviation D-15 |
| 7.1e | Does not conform | Deviation D-16 |
| 7.1j | Partly | Fixed (data variables) + Deviation D-17 (kernel object IDs) |
| 7.1k | Does not conform | Deviation D-16 |
| 7.1m | Does not conform | Deviation D-16 |
| 7.2c | Partly | Fixed |
| 8.1a | Does not conform | Deviation D-04 |
| 8.2c | Conforms | Conforms |
| 8.2b/d | Not checked | 8.2d Fixed (51 chains); 8.2b Deviation D-13 |
| 8.3a | Does not conform | Deviation D-03 |
| 8.3b | Conforms | Conforms |
| 8.4a | Does not conform | Deviation D-18 |
| 8.4c | Does not conform | Fixed |
| 8.4d | Does not conform | Fixed |
| 8.5b | Conforms | Conforms |
| 8.6a | Does not conform | Deviation D-04 |

**Result:** no rule is left as "Does not conform" or "Not checked". Each one now conforms, is fixed, or is a recorded deviation.

## 4. Not changed on purpose (from the requirements and ISR audit)

These items in `REQUIREMENTS_AND_ISR_AUDIT.md` are logic changes, so they are outside this refactor:

- **Barcode ADC wait with no limit** (`bc_alarm_isr`, `subsystem3.c`): now has a braced, commented body (8.4d), but still has no iteration limit. The proposed fix (at most 50 reads, then skip the sample) is for Buddy 3 to make and re-test.
- **Spinlock in the echo ISR** (`drv_ultrasonic.c`): unchanged. Can't spin today (single core, interrupts masked). The proposed `DI`/`EI` replacement is for Buddy 5.
- **Dashboard peak hump / run summary** (`tools/s1_dashboard.py`): Python only, for Buddy 1.
