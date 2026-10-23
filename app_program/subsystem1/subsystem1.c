/*
 *	subsystem1.c
 *	Subsystem 1 -- WiFi communication, commands and telemetry (Buddy 1).
 *	Student ID: 2502039.
 *
 *	Contents
 *	  1. Kernel objects and shared data
 *	  2. IF-S1-01  telemetry input from the other buddies
 *	  3. Outgoing queue helpers
 *	  4. IF-S1-02  default command hooks (overridden by the real owners)
 *	  5. Network bridge -- hooks called by the radio task (mqtt_utk.c)
 *	  6. Task: Command Handler           (pri 16, event-driven)
 *	  7. Task: Heartbeat & Link Monitor  (pri 19, every 1 s)
 *	  8. Task: Telemetry Publisher       (pri 20, every 500 ms)
 *	  9. subsystem1_init() / subsystem1_start()
 *
 *	Bit packing of the MQTT payloads is in s1_codec.c; the MQTT connection
 *	itself is in lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c.
 */

#include "subsystem1.h"
#include "s1_codec.h"
#include "../../lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.h"

/* ==================================================================== */
/* 1. Kernel objects and shared data                                     */
/* ==================================================================== */
EXPORT ID	mbf_s1_tx;	/* outgoing messages -> network task      */
EXPORT ID	mbf_s1_rx;	/* robot/cmd payloads -> Command Handler  */
EXPORT ID	flg_s1_link;	/* link status bits, below                 */
EXPORT ID	tid_s1_cmd, tid_s1_hb, tid_s1_tlm;
LOCAL  ID	mtx_s1_tlm;	/* protects the telemetry slots            */

#define S1_FLG_LINK_UP		0x01	/* MQTT session up                    */
#define S1_FLG_LINK_LOST	0x02	/* event: link just lost (to monitor) */
#define S1_FLG_WIFI_LOSS	0x04	/* sticky: loss not yet reported      */

LOCAL volatile UW	s1_link_losses;		/* link losses since boot  */
LOCAL volatile UB	s1_mission_state = 1;	/* IDLE                    */

/* One 64-bit word per buddy, so two buddies never write the same word.
   Written by the owners, read by the Telemetry Publisher, always under
   mtx_s1_tlm (a 64-bit store is not atomic on the Cortex-M0+). */
LOCAL UD	tlm_slot[S1_SLOT_COUNT];
LOCAL const UB	slot_bits[S1_SLOT_COUNT] = {
	S1_B2_BITS, S1_B3_BITS, S1_B4_BITS, S1_B5_BITS
};

/* Newest telemetry frame kept while the link is down */
LOCAL UB	pending[S1_TLM_LEN];
LOCAL BOOL	pending_valid;

#define S1_TX_MSG_MAX	(1 + MQTT_UTK_PAYLOAD_MAX)	/* topic + payload */
#define RECENT_IDS	4		/* ids remembered for duplicate check */

/* Milliseconds since boot.  tk_get_otm (operating time) is used, not
   tk_get_tim: system time is a calendar clock that does not start at 0,
   so it cannot give the uptime in the heartbeat. */
LOCAL UW now_ms(void)
{
	SYSTIM t;

	tk_get_otm(&t);
	return (UW)t.lo;
}

/* ==================================================================== */
/* 2. IF-S1-01: telemetry input                                          */
/* ==================================================================== */
EXPORT ER s1_tlm_set_slot(UINT slot, UD bits)
{
	ER er;

	if (slot >= S1_SLOT_COUNT) {
		return E_PAR;
	}
	bits &= (1ull << slot_bits[slot]) - 1u;	/* keep only this slot's bits */

	er = tk_loc_mtx(mtx_s1_tlm, TMO_FEVR);
	if (er < E_OK) {
		return er;
	}
	tlm_slot[slot] = bits;
	tk_unl_mtx(mtx_s1_tlm);
	return E_OK;
}

EXPORT void s1_tlm_snapshot(UD out[S1_SLOT_COUNT])
{
	INT i;

	tk_loc_mtx(mtx_s1_tlm, TMO_FEVR);
	for (i = 0; i < S1_SLOT_COUNT; i++) {
		out[i] = tlm_slot[i];
	}
	tk_unl_mtx(mtx_s1_tlm);
}

EXPORT void s1_set_mission_state(UINT state)
{
	s1_mission_state = (UB)(state & 0x7u);	/* 3 bits */
}

LOCAL BOOL s1_link_up(void)
{
	T_RFLG r;

	return (tk_ref_flg(flg_s1_link, &r) == E_OK &&
		(r.flgptn & S1_FLG_LINK_UP) != 0) ? TRUE : FALSE;
}

/* ==================================================================== */
/* 3. Outgoing queue helpers (non-blocking)                              */
/* ==================================================================== */
LOCAL BOOL s1_tx(UB topic, const UB *payload, INT len)
{
	UB	msg[S1_TX_MSG_MAX];
	INT	i;

	if (len <= 0 || len > MQTT_UTK_PAYLOAD_MAX) {
		return FALSE;
	}
	msg[0] = topic;
	for (i = 0; i < len; i++) {
		msg[1 + i] = payload[i];
	}
	return (tk_snd_mbf(mbf_s1_tx, msg, 1 + len, TMO_POL) == E_OK)
		? TRUE : FALSE;
}

/* frame != NULL: keep it (link down).  frame == NULL: link is back,
   send the kept frame first. */
LOCAL void s1_keep_pending(const UB *frame, INT len)
{
	INT i;

	tk_loc_mtx(mtx_s1_tlm, TMO_FEVR);
	if (frame != NULL) {
		for (i = 0; i < len && i < S1_TLM_LEN; i++) {
			pending[i] = frame[i];
		}
		pending_valid = TRUE;
	} else if (pending_valid) {
		pending_valid = FALSE;
		tk_unl_mtx(mtx_s1_tlm);
		(void)s1_tx(MQTT_UTK_T_TELEMETRY, pending, S1_TLM_LEN);
		return;
	}
	tk_unl_mtx(mtx_s1_tlm);
}

/* ==================================================================== */
/* 4. IF-S1-02: default command hooks                                    */
/*    WEAK: the Vehicle Controller / Motion Control owners replace them  */
/*    with real versions of the same name.  They must never block.       */
/* ==================================================================== */
WEAK_FUNC EXPORT BOOL s1_out_start(void)
{
	tm_printf((UB*)"S1: START -> (no Vehicle Controller yet)\n");
	s1_set_mission_state(2);	/* RUNNING, for stand-alone testing */
	return TRUE;
}

WEAK_FUNC EXPORT BOOL s1_out_stop(void)
{
	tm_printf((UB*)"S1: STOP -> (no Vehicle Controller yet)\n");
	s1_set_mission_state(1);	/* IDLE */
	return TRUE;
}

WEAK_FUNC EXPORT void s1_out_estop(void)
{
	tm_printf((UB*)"S1: E-STOP -> (no flg_safety owner yet)\n");
	s1_set_mission_state(3);	/* E_STOPPED */
}

/* ==================================================================== */
/* 5. Network bridge -- called from cyw43_utk_task() (mqtt_utk.c).       */
/*    Only TMO_POL kernel calls, so the radio task is never blocked.     */
/* ==================================================================== */

/* Next outgoing message from mbf_s1_tx: [topic][payload...] */
EXPORT int s1_net_next_tx(unsigned char *topic, unsigned char *buf, int max)
{
	UB	msg[S1_TX_MSG_MAX];
	INT	n, i;

	n = tk_rcv_mbf(mbf_s1_tx, msg, TMO_POL);
	if (n <= 1) {
		return 0;
	}
	*topic = msg[0];
	n -= 1;
	if (n > max) {
		n = max;
	}
	for (i = 0; i < n; i++) {
		buf[i] = msg[1 + i];
	}
	return n;
}

/* robot/cmd payload -> Command Handler */
EXPORT void s1_net_rx_cmd(const unsigned char *buf, int len)
{
	if (len <= 0 || len > MQTT_UTK_PAYLOAD_MAX) {
		return;
	}
	if (tk_snd_mbf(mbf_s1_rx, (void*)buf, len, TMO_POL) != E_OK) {
		tm_printf((UB*)"S1: mbf_s1_rx full, command dropped\n");
	}
}

/* MQTT session up (1) / down (0) */
EXPORT void s1_net_link(int up)
{
	UB	discard[S1_TX_MSG_MAX];

	if (up) {
		tk_set_flg(flg_s1_link, S1_FLG_LINK_UP);
	} else {
		tk_clr_flg(flg_s1_link, ~S1_FLG_LINK_UP);
		/* drop messages queued for the dead session */
		while (tk_rcv_mbf(mbf_s1_tx, discard, TMO_POL) > 0) {
		}
		/* wake the link monitor; Wi-Fi-loss bit set in next telemetry */
		tk_set_flg(flg_s1_link, S1_FLG_LINK_LOST | S1_FLG_WIFI_LOSS);
	}
}

/* ==================================================================== */
/* 6. Task: Command Handler (pri 16)                                     */
/*    2-bit command: 00 START / 01 STOP -> Vehicle Controller hook,      */
/*    10 E-STOP -> flg_safety hook (never filtered), else ack INVALID.   */
/*    A START/STOP whose 6-bit id was just handled is ignored.           */
/* ==================================================================== */
LOCAL void send_ack(INT result, UB id)
{
	uint8_t	ack[S1_ACK_LEN];

	s1_encode_ack(ack, result, id);
	(void)s1_tx(MQTT_UTK_T_ACK, ack, S1_ACK_LEN);
}

LOCAL void s1_cmd_task(INT stacd, void *exinf)
{
	UB	buf[MQTT_UTK_PAYLOAD_MAX];
	UB	recent[RECENT_IDS];
	INT	n_recent = 0, next = 0, i, n, cmd;
	uint8_t	id;
	BOOL	dup, ok;

	(void)stacd; (void)exinf;

	while (1) {
		n = tk_rcv_mbf(mbf_s1_rx, buf, TMO_FEVR);
		if (n <= 0) {
			continue;
		}

		cmd = s1_decode_command(buf, n, &id);

		if (cmd == S1_CMD_INVALID) {
			tm_printf((UB*)"S1: INVALID command (%d bytes)\n", n);
			send_ack(S1_ACK_INVALID, id);
			continue;
		}
		if (cmd == S1_CMD_ESTOP) {
			s1_out_estop();
			continue;
		}

		dup = FALSE;
		for (i = 0; i < n_recent; i++) {
			if (recent[i] == id) {
				dup = TRUE;
			}
		}
		if (dup) {
			tm_printf((UB*)"S1: duplicate command id %d ignored\n", id);
			continue;
		}

		ok = (cmd == S1_CMD_START) ? s1_out_start() : s1_out_stop();
		if (ok) {
			recent[next] = id;	/* remember only delivered ids */
			next = (next + 1) % RECENT_IDS;
			if (n_recent < RECENT_IDS) n_recent++;
		} else {
			tm_printf((UB*)"S1: %s refused, robot/ack = FAILED\n",
				  cmd == S1_CMD_START ? "START" : "STOP");
			send_ack(S1_ACK_FAILED, id);
		}
	}
}

/* ==================================================================== */
/* 7. Task: Heartbeat & Link Monitor (pri 19)                            */
/*    Heartbeat every 1 s while connected; wakes at once on link loss    */
/*    and counts it.  Reconnection is done by the radio task.            */
/* ==================================================================== */
LOCAL void s1_heartbeat_task(INT stacd, void *exinf)
{
	uint8_t	hb[S1_HB_LEN];
	UB	seq = 0;
	UW	next = now_ms() + S1_HEARTBEAT_MS;
	UINT	ptn;
	INT	wait;
	ER	er;

	(void)stacd; (void)exinf;

	while (1) {
		wait = (INT)(next - now_ms());
		if (wait < 0) {
			wait = 0;
		}

		er = tk_wai_flg(flg_s1_link, S1_FLG_LINK_LOST,
				TWF_ORW | TWF_BITCLR, &ptn, (TMO)wait);
		if (er == E_OK) {
			s1_link_losses++;
			tm_printf((UB*)"S1: link lost (total %d)\n",
				  (INT)s1_link_losses);
			continue;
		}

		next += S1_HEARTBEAT_MS;
		if ((INT)(next - now_ms()) <= 0) {
			next = now_ms() + S1_HEARTBEAT_MS;	/* resync */
		}

		if (!s1_link_up()) {
			continue;
		}
		s1_encode_heartbeat(hb, seq++, s1_mission_state,
				    now_ms() / 1000u, s1_link_losses);
		(void)s1_tx(MQTT_UTK_T_HEARTBEAT, hb, S1_HB_LEN);
	}
}

/* ==================================================================== */
/* 8. Task: Telemetry Publisher (pri 20, every 500 ms)                   */
/*    Copies the slots under the mutex, packs the 18-byte frame outside  */
/*    it.  Wi-Fi-loss bit (bit 137, last) stays 1 until one frame with   */
/*    it has been handed to the network.                                 */
/* ==================================================================== */
LOCAL void s1_telemetry_task(INT stacd, void *exinf)
{
	UD	slots[S1_SLOT_COUNT];
	uint8_t	frame[S1_TLM_LEN];
	T_RFLG	r;
	UW	next = now_ms(), now;
	INT	wait, wifi_loss;

	(void)stacd; (void)exinf;

	while (1) {
		s1_tlm_snapshot(slots);

		tk_ref_flg(flg_s1_link, &r);
		wifi_loss = (r.flgptn & S1_FLG_WIFI_LOSS) ? 1 : 0;
		s1_encode_telemetry(frame, wifi_loss, (const uint64_t*)slots);

		if (r.flgptn & S1_FLG_LINK_UP) {
			s1_keep_pending(NULL, 0);	/* kept frame first */
			if (s1_tx(MQTT_UTK_T_TELEMETRY, frame, S1_TLM_LEN) &&
			    wifi_loss) {
				tk_clr_flg(flg_s1_link, ~S1_FLG_WIFI_LOSS);
			}
		} else {
			s1_keep_pending(frame, S1_TLM_LEN);
		}

		next += S1_TELEMETRY_MS;		/* fixed period */
		now   = now_ms();
		wait  = (INT)(next - now);
		if (wait <= 0) {
			next = now;
			wait = 1;
		}
		tk_dly_tsk((TMO)wait);
	}
}

/* ==================================================================== */
/* 9. init / start                                                       */
/* ==================================================================== */
EXPORT ER subsystem1_init(void)
{
	T_CMBF	cmbf = {0};
	T_CFLG	cflg = {0};
	T_CMTX	cmtx = {0};

	cmtx.mtxatr = TA_INHERIT;		/* priority inheritance */
	mtx_s1_tlm = tk_cre_mtx(&cmtx);
	if (mtx_s1_tlm < E_OK) return mtx_s1_tlm;

	cmbf.mbfatr = TA_TFIFO;
	cmbf.bufsz  = 512;			/* ~20 queued messages */
	cmbf.maxmsz = S1_TX_MSG_MAX;
	mbf_s1_tx = tk_cre_mbf(&cmbf);
	if (mbf_s1_tx < E_OK) return mbf_s1_tx;

	cmbf.bufsz  = 128;
	cmbf.maxmsz = MQTT_UTK_PAYLOAD_MAX;
	mbf_s1_rx = tk_cre_mbf(&cmbf);
	if (mbf_s1_rx < E_OK) return mbf_s1_rx;

	cflg.flgatr  = TA_TFIFO | TA_WMUL;
	cflg.iflgptn = 0;			/* LINK_UP = 0 */
	flg_s1_link = tk_cre_flg(&cflg);
	if (flg_s1_link < E_OK) return flg_s1_link;

	return E_OK;
}

EXPORT ER subsystem1_start(void)
{
	tid_s1_cmd = app_start_task((FP)s1_cmd_task, S1_PRI_CMD, 0,
				    "s1_cmd_task");
	if (tid_s1_cmd < E_OK) return tid_s1_cmd;

	tid_s1_hb = app_start_task((FP)s1_heartbeat_task, S1_PRI_HEARTBEAT, 0,
				   "s1_heartbeat_task");
	if (tid_s1_hb < E_OK) return tid_s1_hb;

	tid_s1_tlm = app_start_task((FP)s1_telemetry_task, S1_PRI_TELEMETRY, 0,
				    "s1_telemetry_task");
	if (tid_s1_tlm < E_OK) return tid_s1_tlm;

#if !TM_MQTT
	tm_printf((UB*)"S1: built without MQTT=1 -- telemetry is not sent\n");
#endif
	return E_OK;
}