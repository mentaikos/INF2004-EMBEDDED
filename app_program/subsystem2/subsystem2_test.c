/*
 *	subsystem2_test.c
 *	On-target tests for subsystem 2.  Built only with: make TEST=1
 *	Record results in testing/subsystem2/TEST_LOG.md.
 *
 *	None of these tests turns a motor: S2-U08 only sends a MOVE while
 *	E-STOP is set, which Motion must ignore.  Still, run them with the
 *	wheels off the table.
 */

#include "subsystem2.h"
#include "motor_driver.h"
#include "encoder.h"
#include "../test_report.h"

#if APP_TEST

EXPORT void subsystem2_test(void)
{
	T_RSEM	rsem;
	T_RMPF	rmpf;
	UW	seq, w;
	INT	v, bad = 0;
	BOOL	ok;

	TEST_INIT();

	/* S2-U01 / S2-U02: template example kept for subsystems 3 and 4 */
	for (seq = 0; seq < 40; seq++) {
		v = s2_make_value(seq);
		if (v < 0 || v > 100) bad++;
	}
	TEST_CHECK("S2-U01", bad == 0, "example sample values within 0..100");
	TEST_CHECK("S2-U02",
		   tk_ref_sem(sem_slots, &rsem) == E_OK &&
		   rsem.semcnt >= 0 && rsem.semcnt <= S2_POOL_COUNT &&
		   tk_ref_mpf(mpf_samples, &rmpf) == E_OK &&
		   rmpf.frbcnt <= S2_POOL_COUNT,
		   "example free slots and blocks within pool size");

	/* S2-U03: command word round trip, incl. negative angle/distance */
	w = S2_CMD_PACK(S2_CMD_TURN, 30, -90, 0);
	ok = s2_cmd_type(w) == S2_CMD_TURN && s2_cmd_speed(w) == 30 &&
	     s2_cmd_angle(w) == -90 && s2_cmd_dist(w) == 0;
	w = S2_CMD_PACK(S2_CMD_MOVE_DIST, 20, 0, -100);
	ok = ok && s2_cmd_type(w) == S2_CMD_MOVE_DIST &&
	     s2_cmd_speed(w) == 20 && s2_cmd_dist(w) == -100;
	w = S2_CMD_PACK(S2_CMD_TURN, 0, 180, 127);
	ok = ok && s2_cmd_angle(w) == 180 && s2_cmd_dist(w) == 127;
	w = S2_CMD_PACK(S2_CMD_TURN, 0, -180, -128);
	ok = ok && s2_cmd_angle(w) == -180 && s2_cmd_dist(w) == -128;
	TEST_CHECK("S2-U03", ok, "command word pack/unpack round trip");

	/* S2-U04: values are clamped, nothing spills into other fields */
	w = S2_CMD_PACK(S2_CMD_MOVE_FOLLOW, 70, 250, 200);
	ok = s2_cmd_speed(w) == S2_MAX_SPEED_CM_S && s2_cmd_angle(w) == 180 &&
	     s2_cmd_dist(w) == 127 && (w >> 26) == 0 &&
	     s2_cmd_type(w) == S2_CMD_MOVE_FOLLOW;
	w = S2_CMD_PACK(S2_CMD_SPEED_LIMIT, -5, -300, -200);
	ok = ok && s2_cmd_speed(w) == 0 && s2_cmd_angle(w) == -180 &&
	     s2_cmd_dist(w) == -128 && s2_cmd_type(w) == S2_CMD_SPEED_LIMIT;
	TEST_CHECK("S2-U04", ok, "command fields clamped to their ranges");

	/* S2-U05: status word round trip and clamping */
	w = s2_sts_pack(-12, 34, 4095, S2_ST_STALL_FAULT, 1, 0, 1);
	ok = s2_sts_speed_l(w) == -12 && s2_sts_speed_r(w) == 34 &&
	     s2_sts_dist(w) == 4095 && s2_sts_state(w) == S2_ST_STALL_FAULT &&
	     s2_sts_dir_l(w) == 1 && s2_sts_dir_r(w) == 0 &&
	     s2_sts_stopped(w) == 1;
	w = s2_sts_pack(-100, 100, 4096 + 7, S2_ST_TURN, 0, 1, 0);
	ok = ok && s2_sts_speed_l(w) == -64 && s2_sts_speed_r(w) == 63 &&
	     s2_sts_dist(w) == 7 && s2_sts_state(w) == S2_ST_TURN &&
	     s2_sts_dir_r(w) == 1 && s2_sts_stopped(w) == 0;
	TEST_CHECK("S2-U05", ok, "status word pack/unpack round trip");

	/* S2-U06: wrap-safe distance difference */
	TEST_CHECK("S2-U06",
		   s2_dist_diff(4093, 3) == 6 && s2_dist_diff(3, 4093) == -6 &&
		   s2_dist_diff(100, 100) == 0 && s2_dist_diff(0, 2047) == 2047,
		   "12-bit distance difference across the wrap");

	/* S2-U07: 8-bit encoder delta across the wrap */
	TEST_CHECK("S2-U07",
		   encoder_delta(250, 3) == 9 && encoder_delta(255, 0) == 1 &&
		   encoder_delta(7, 7) == 0,
		   "encoder count delta across the 8-bit wrap");

	/* S2-U08: E-STOP guard.  With E-STOP set a MOVE is ignored and the
	   motors stay braked; after the VC clears it, Motion is STOPPED. */
	tk_set_flg(flg_safety, S2_FLG_ESTOP);
	tk_dly_tsk(3 * S2_PERIOD_MS);
	motion_send_cmd(S2_CMD_PACK(S2_CMD_MOVE_FOLLOW, 20, 0, 0));
	tk_dly_tsk(3 * S2_PERIOD_MS);
	ok = s2_sts_state(motion_get_status()) == S2_ST_E_STOP_BRAKE &&
	     motor_is_braked();
	tk_clr_flg(flg_safety, ~S2_FLG_ESTOP);
	tk_dly_tsk(3 * S2_PERIOD_MS);
	ok = ok && s2_sts_state(motion_get_status()) == S2_ST_STOPPED &&
	     motor_is_braked();
	TEST_CHECK("S2-U08", ok, "MOVE ignored and motors braked while E-STOP set");

	/* S2-U09: "stopped" flag reported when STOPPED and wheels still */
	tk_dly_tsk(S2_ZERO_SPEED_MS + 3 * S2_PERIOD_MS);
	TEST_CHECK("S2-U09", s2_sts_stopped(motion_get_status()) == 1,
		   "stopped flag set in STOPPED with wheels still");

	TEST_SUMMARY("subsystem2");
}

#endif /* APP_TEST */
