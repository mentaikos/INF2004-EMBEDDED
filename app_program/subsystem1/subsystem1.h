/*
 *	subsystem1.h
 *	Subsystem 1 -- WiFi communication, commands and telemetry (Buddy 1).
 *	Student ID: 2502039.
 *
 *	PUBLIC interface: only what other subsystems may use.  Other subsystems
 *	include it as "../subsystem1/subsystem1.h".
 *
 *	Tasks (RTOS task table, Week 6 design review):
 *	  Command Handler           pri 16  event-driven (mbf_s1_rx)
 *	  Heartbeat & Link Monitor  pri 19  every 1 s, and on link loss
 *	  Telemetry Publisher       pri 20  every 500 ms
 *	The WiFi/MQTT network side runs on the framework's radio task
 *	cyw43_utk_task() (pri 4), the only task allowed to call lwIP; see
 *	lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c.
 *
 *	All MQTT payloads are compact binary (change requirement: minimum-bit
 *	data representation).  Every message starts with a 4-bit header:
 *	  2-bit message type, then 2-bit layout version.
 *	Bit layouts are in s1_codec.h.  Code: subsystem1.c (tasks), s1_codec.c
 *	(bit packing), mqtt_utk.c (MQTT connection).
 */

#ifndef SUBSYSTEM1_H
#define SUBSYSTEM1_H

#include "../app.h"

/* ---- Task priorities (1 = highest) ---- */
#define S1_PRI_CMD		16	/* Command Handler          */
#define S1_PRI_HEARTBEAT	19	/* Heartbeat & Link Monitor */
#define S1_PRI_TELEMETRY	20	/* Telemetry Publisher      */

/* ---- Timing (NFR6, NFR17) ---- */
#define S1_TELEMETRY_MS		500
#define S1_HEARTBEAT_MS		1000

/* ---------------------------------------------------------------------
 *  IF-S1-01  Telemetry input (all buddies -> S1)
 *
 *  Each buddy packs its own fields into one integer ("slot") and hands it
 *  to subsystem 1, which places the slots in the fixed 18-byte telemetry
 *  frame.  Only the lowest <width> bits are used; extra bits are masked
 *  off.  Safe to call from any task (protected by mtx_s1_tlm).
 *
 *    slot          owner     bits   frame bits
 *    (header)      Buddy 1     4     0..3   (type 2 + version 2)
 *    S1_SLOT_B2    Buddy 2    26     4..29
 *    S1_SLOT_B3    Buddy 3     8    30..37
 *    S1_SLOT_B4    Buddy 4    44    38..81
 *    S1_SLOT_B5    Buddy 5    55    82..136
 *    (Wi-Fi loss)  Buddy 1     1   137       <- last data bit
 *    (padding)                 6   138..143
 * --------------------------------------------------------------------- */
#define S1_SLOT_B2		0
#define S1_SLOT_B3		1
#define S1_SLOT_B4		2
#define S1_SLOT_B5		3
#define S1_SLOT_COUNT		4

IMPORT ER	s1_tlm_set_slot(UINT slot, UD bits);

/* Mission state for the heartbeat (Vehicle Controller -> S1), 3 bits:
   0 INIT, 1 IDLE, 2 RUNNING, 3 E_STOPPED, 4 FAULT, 5 COMPLETE */
IMPORT void	s1_set_mission_state(UINT state);

/* ---------------------------------------------------------------------
 *  IF-S1-02  Commands out (S1 -> Vehicle Controller / Motion Control)
 *
 *  Called by the Command Handler after a robot/cmd message is decoded.
 *  Subsystem 1 provides WEAK default versions that only print, so it can be
 *  tested alone.  The owners override them with real ones (same name, no
 *  WEAK_FUNC) that:
 *    s1_out_start / s1_out_stop : send EVT_START / EVT_STOP to the Vehicle
 *        Controller through mbf_vc_evt with TMO_POL; return FALSE if the
 *        buffer is full (S1 then publishes robot/ack = FAILED).
 *    s1_out_estop : set E_STOP in flg_safety (bypasses the Vehicle
 *        Controller; Motion Control polls it every 20 ms).
 *  They must never block.
 * --------------------------------------------------------------------- */
IMPORT BOOL	s1_out_start(void);
IMPORT BOOL	s1_out_stop(void);
IMPORT void	s1_out_estop(void);

/* ---- Used by subsystem1_test.c only ---- */
IMPORT ID	mbf_s1_tx;	/* outgoing messages -> network task */
IMPORT ID	mbf_s1_rx;	/* robot/cmd payloads -> Command Handler */
IMPORT ID	flg_s1_link;	/* link status bits */
IMPORT ID	tid_s1_cmd, tid_s1_hb, tid_s1_tlm;
IMPORT void	s1_tlm_snapshot(UD out[S1_SLOT_COUNT]);

#endif /* SUBSYSTEM1_H */