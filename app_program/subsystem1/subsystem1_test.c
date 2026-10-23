/*
 *	subsystem1_test.c
 *	On-target tests for subsystem 1.  Built only with: make TEST=1
 *
 *	Each TEST_CHECK prints a PASS/FAIL line on the console.  Save the
 *	console output to testing/subsystem1/logs/ and record the result in
 *	testing/subsystem1/TEST_LOG.md.  Test IDs: S1-U01, S1-U02, ...
 */

#include "subsystem1.h"
#include "s1_codec.h"
#include "../test_report.h"

#if APP_TEST

/* Builds a 2-byte command message: type(2) version(2) cmd(2) id(6) pad(4) */
LOCAL uint8_t *make_cmd(uint8_t *buf, UINT type, UINT ver, UINT cmd, UINT cid)
{
	uint32_t pos = 0;

	buf[0] = buf[1] = 0;
	s1_put_bits(buf, &pos, type, S1_TYPE_BITS);
	s1_put_bits(buf, &pos, ver, S1_VERSION_BITS);
	s1_put_bits(buf, &pos, cmd, 2);
	s1_put_bits(buf, &pos, cid, 6);
	return buf;
}

EXPORT void subsystem1_test(void)
{
	T_RTSK	rtsk;
	T_RMBF	rmbf;
	T_RFLG	rflg;
	uint8_t	buf[S1_TLM_LEN];
	uint64_t slots[4] = { 0x3FFFFFFull, 0xA5u, 0x0F0F0F0F0F0ull, 0x40000000000001ull };
	uint32_t pos;
	uint8_t	id;
	BOOL	ok;

	/* Wait 5 s so there is time to open the serial monitor after a
	   reset; the USB console only appears once the Pico has booted. */
	tm_printf((UB*)"[TEST] subsystem1 tests start in 5 s ...\n");
	tk_dly_tsk(5000);

	TEST_INIT();

	/* S1-U01: kernel objects exist */
	TEST_CHECK("S1-U01",
		   tk_ref_mbf(mbf_s1_tx, &rmbf) == E_OK &&
		   tk_ref_mbf(mbf_s1_rx, &rmbf) == E_OK &&
		   tk_ref_flg(flg_s1_link, &rflg) == E_OK,
		   "mbf_s1_tx, mbf_s1_rx and flg_s1_link created");

	/* S1-U02: the three tasks are running (not dormant) */
	ok = TRUE;
	if (tk_ref_tsk(tid_s1_cmd, &rtsk) != E_OK || rtsk.tskstat == TTS_DMT) ok = FALSE;
	if (tk_ref_tsk(tid_s1_hb,  &rtsk) != E_OK || rtsk.tskstat == TTS_DMT) ok = FALSE;
	if (tk_ref_tsk(tid_s1_tlm, &rtsk) != E_OK || rtsk.tskstat == TTS_DMT) ok = FALSE;
	TEST_CHECK("S1-U02", ok, "Command, Heartbeat and Telemetry tasks started");

	/* S1-U03: telemetry frame is 18 bytes and every slot decodes back
	   unchanged at its fixed bit offset */
	s1_encode_telemetry(buf, 1, slots);
	pos = 0;
	ok  = (s1_get_bits(buf, &pos, S1_TYPE_BITS) == S1_TYPE_TELEMETRY);
	ok &= (s1_get_bits(buf, &pos, S1_VERSION_BITS) == S1_VERSION);
	ok &= (s1_get_bits(buf, &pos, S1_B2_BITS) == slots[0]);
	ok &= (s1_get_bits(buf, &pos, S1_B3_BITS) == slots[1]);
	ok &= (s1_get_bits(buf, &pos, S1_B4_BITS) == slots[2]);
	ok &= (s1_get_bits(buf, &pos, S1_B5_BITS) == slots[3]);
	ok &= (s1_get_bits(buf, &pos, 1) == 1);		/* Wi-Fi loss, last */
	ok &= (pos == S1_TLM_BITS) && (S1_TLM_LEN == 18) &&
	      ((buf[17] & 0x3F) == 0);			/* 6 padding bits */
	TEST_CHECK("S1-U03", ok, "telemetry 18 B, Wi-Fi-loss bit last (137)");

	/* S1-U04: 2-bit commands 00/01/10 decode; anything else is INVALID */
	ok  = (s1_decode_command(make_cmd(buf, S1_TYPE_COMMAND, S1_VERSION, 0, 5), 2, &id) == S1_CMD_START && id == 5);
	ok &= (s1_decode_command(make_cmd(buf, S1_TYPE_COMMAND, S1_VERSION, 1, 6), 2, &id) == S1_CMD_STOP  && id == 6);
	ok &= (s1_decode_command(make_cmd(buf, S1_TYPE_COMMAND, S1_VERSION, 2, 7), 2, &id) == S1_CMD_ESTOP && id == 7);
	ok &= (s1_decode_command(make_cmd(buf, S1_TYPE_COMMAND, S1_VERSION, 3, 8), 2, &id) == S1_CMD_INVALID);	/* code 11 */
	ok &= (s1_decode_command(make_cmd(buf, S1_TYPE_COMMAND, 2, 0, 9), 2, &id) == S1_CMD_INVALID);	/* wrong version */
	ok &= (s1_decode_command(make_cmd(buf, S1_TYPE_ACK, S1_VERSION, 0, 9), 2, &id) == S1_CMD_INVALID);	/* wrong type */
	ok &= (s1_decode_command(make_cmd(buf, S1_TYPE_COMMAND, S1_VERSION, 0, 9), 3, &id) == S1_CMD_INVALID);	/* wrong length */
	TEST_CHECK("S1-U04", ok, "commands 00 START / 01 STOP / 10 E-STOP, rest INVALID");

	/* S1-U05: slot values wider than the slot are masked */
	ok = (s1_tlm_set_slot(S1_SLOT_B3, 0x1FFull) == E_OK);
	{
		UD snap[S1_SLOT_COUNT];
		s1_tlm_snapshot(snap);
		ok &= (snap[S1_SLOT_B3] == 0xFFull);
	}
	ok &= (s1_tlm_set_slot(S1_SLOT_COUNT, 0) == E_PAR);
	s1_tlm_set_slot(S1_SLOT_B3, 0);
	TEST_CHECK("S1-U05", ok, "slot input masked to its bit width");

	/* S1-U06: heartbeat and ack sizes */
	ok  = (s1_encode_heartbeat(buf, 1, 2, 17, 99) == S1_HB_LEN);
	pos = S1_HDR_BITS + 8 + 3 + 16;
	ok &= (s1_get_bits(buf, &pos, 4) == 15);	/* link losses saturate */
	ok &= (s1_encode_ack(buf, S1_ACK_FAILED, 5) == S1_ACK_LEN &&
	       buf[0] == 0xD8 && buf[1] == 0xA0);	/* 11 01 1 000101 00000 */
	TEST_CHECK("S1-U06", ok, "heartbeat 5 bytes, ack 2 bytes");

	TEST_SUMMARY("subsystem1");
}

#endif /* APP_TEST */