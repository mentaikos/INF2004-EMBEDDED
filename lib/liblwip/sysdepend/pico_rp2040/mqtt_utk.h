/*
 *	mqtt_utk.h
 *	MQTT client for micro T-Kernel (RP2040, NO_SYS=1) -- Buddy 1 / subsystem 1
 *
 *	Everything that calls lwIP runs on cyw43_utk_task() (the radio + network
 *	owner task, docs/RELEASE.md safety rule 6).  Subsystem 1's own tasks never
 *	call lwIP: they exchange bytes with this file only through the three
 *	s1_net_*() hooks below, implemented in app_program/subsystem1/subsystem1.c
 *	(section 5) on top of mbf_s1_tx / mbf_s1_rx and flg_s1_link.
 *
 *	This header uses plain C types only, so it can be included both here
 *	(pico-sdk / lwIP namespace, no tk/tkernel.h) and from app_program/.
 */

#ifndef MQTT_UTK_H
#define MQTT_UTK_H

/* ---- MQTT topics (MQTT documentation, FR3) ---- */
#define MQTT_UTK_TOPIC_CMD        "robot/cmd"        /* subscribe, QoS 1 */
#define MQTT_UTK_TOPIC_TELEMETRY  "robot/telemetry"  /* publish,   QoS 0 */
#define MQTT_UTK_TOPIC_HEARTBEAT  "robot/heartbeat"  /* publish,   QoS 0 */
#define MQTT_UTK_TOPIC_ACK        "robot/ack"        /* publish,   QoS 1 */
#define MQTT_UTK_TOPIC_STATUS     "robot/status"     /* ONLINE / OFFLINE (Last Will), retained */

/* Topic IDs carried in front of every message in mbf_s1_tx */
#define MQTT_UTK_T_TELEMETRY      0
#define MQTT_UTK_T_HEARTBEAT      1
#define MQTT_UTK_T_ACK            2
#define MQTT_UTK_T_COUNT          3

#define MQTT_UTK_PAYLOAD_MAX      32   /* largest binary payload, bytes */

/*
 * Called once per poll from cyw43_utk_task() while the WiFi link is up
 * (see cyw43_utk.c). Starts the MQTT connection if it is idle; does nothing
 * while connecting, connected or waiting out the reconnect back-off.
 */
void mqtt_utk_start(void);

/* ---------------------------------------------------------------------
 *  Hooks implemented by subsystem 1 (subsystem1.c, section 5).
 *  They are called from cyw43_utk_task() and never block (TMO_POL).
 * --------------------------------------------------------------------- */

/* Next outgoing message: writes its topic ID to *topic and its payload to
 * buf (at most max bytes). Returns the payload length, or 0 if none. */
int  s1_net_next_tx(unsigned char *topic, unsigned char *buf, int max);

/* A message arrived on robot/cmd (raw payload bytes). */
void s1_net_rx_cmd(const unsigned char *buf, int len);

/* MQTT session went up (1) or down (0). */
void s1_net_link(int up);

#endif /* MQTT_UTK_H */