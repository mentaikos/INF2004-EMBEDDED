/*
 *	mqtt_utk.c
 *	MQTT client for micro T-Kernel (RP2040, NO_SYS=1) -- Buddy 1 / subsystem 1
 *
 *	Replaces the framework's heartbeat demo.  It implements the MQTT half of
 *	the Buddy 1 statechart (Week 6 design, 6.1):
 *
 *	  IDLE --start--> CONNECTING --CONNACK OK--> CONNECTED
 *	                     | CONNACK timeout 3 s       | keep-alive timeout,
 *	                     v                           v socket error, WiFi drop
 *	                  BACKOFF (1 s) <----------------+
 *	                     |
 *	                     +--> IDLE (cyw43_utk_task calls mqtt_utk_start()
 *	                               again as soon as WiFi is up)
 *
 *	WiFi join / rejoin is handled by cyw43_utk.c.
 *
 *	Runs entirely on cyw43_utk_task(), which already owns the CYW43439 and
 *	lwIP: every function here is either called from that task's poll loop or
 *	from an lwIP callback/timer that the same loop drives through
 *	sys_check_timeouts().  Nothing here includes tk/tkernel.h (namespace
 *	clash with the pico-sdk headers); subsystem 1 is reached only through
 *	the s1_net_*() hooks declared in mqtt_utk.h.
 */

#include "lwip/apps/mqtt.h"
#include "lwip/ip_addr.h"
#include "lwip/netif.h"
#include "lwip/timeouts.h"
#include "wifi_secrets.h"
#include "mqtt_utk.h"
#include <tm/tmonitor.h>

#define MQTT_UTK_KEEPALIVE_S     2     /* NFR20: link loss by keep-alive  */
#define MQTT_UTK_CONNACK_MS      3000  /* NFR19: CONNACK timeout          */
#define MQTT_UTK_BACKOFF_MS      1000  /* NFR19: 1 s between retries      */
#define MQTT_UTK_DRAIN_MS        20    /* how often outgoing data is sent */

typedef enum { ST_IDLE, ST_CONNECTING, ST_CONNECTED, ST_BACKOFF } mqtt_state_t;

static const struct {
	const char	*name;
	u8_t		qos;
} TOPICS[MQTT_UTK_T_COUNT] = {
	[MQTT_UTK_T_TELEMETRY] = { MQTT_UTK_TOPIC_TELEMETRY, 0 },
	[MQTT_UTK_T_HEARTBEAT] = { MQTT_UTK_TOPIC_HEARTBEAT, 0 },
	[MQTT_UTK_T_ACK]       = { MQTT_UTK_TOPIC_ACK,       1 },
};

static mqtt_client_t	*client;
static ip_addr_t	broker_addr;
static mqtt_state_t	state = ST_IDLE;

/* incoming robot/cmd assembly (a publish may arrive in several fragments) */
static u8_t		rx_is_cmd;
static u8_t		rx_buf[MQTT_UTK_PAYLOAD_MAX];
static u16_t		rx_len;
static u8_t		rx_too_long;

static void drain_timeout(void *arg);

/* ------------------------------------------------------------------ */
/* State changes                                                       */
/* ------------------------------------------------------------------ */
static void backoff_done(void *arg)
{
	(void)arg;
	state = ST_IDLE;	/* the poll loop reconnects on its next call */
}

/* Any failure: tell subsystem 1, stop sending, wait 1 s before retrying */
static void go_backoff(const char *why)
{
	sys_untimeout(drain_timeout, NULL);
	sys_untimeout(backoff_done, NULL);

	if (state == ST_CONNECTED) {
		s1_net_link(0);	/* LINK_UP = 0, link loss reported */
	}
	tm_printf((UB*)"mqtt: %s -> RECONNECT_BACKOFF (%d ms)\n",
		  why, MQTT_UTK_BACKOFF_MS);
	state = ST_BACKOFF;
	sys_timeout(MQTT_UTK_BACKOFF_MS, backoff_done, NULL);
}

static void connack_timeout(void *arg)
{
	(void)arg;
	if (state == ST_CONNECTING) {
		mqtt_disconnect(client);	/* abort the attempt */
		go_backoff("CONNACK timeout");
	}
}

/* ------------------------------------------------------------------ */
/* Outgoing: drain subsystem 1's queue every MQTT_UTK_DRAIN_MS          */
/* ------------------------------------------------------------------ */
static void drain_timeout(void *arg)
{
	u8_t	buf[MQTT_UTK_PAYLOAD_MAX];
	u8_t	topic;
	int	len;

	(void)arg;

	if (state != ST_CONNECTED) {
		return;
	}

	/* WiFi dropped: no need to wait for the keep-alive to notice */
	if (netif_default != NULL && !netif_is_link_up(netif_default)) {
		mqtt_disconnect(client);
		go_backoff("WiFi dropped");
		return;
	}

	while ((len = s1_net_next_tx(&topic, buf, sizeof buf)) > 0) {
		if (topic >= MQTT_UTK_T_COUNT) {
			continue;
		}
		if (mqtt_publish(client, TOPICS[topic].name, buf, (u16_t)len,
				 TOPICS[topic].qos, 0, NULL, NULL) != ERR_OK) {
			tm_printf((UB*)"mqtt: publish to %s dropped (buffer full)\n",
				  TOPICS[topic].name);
		}
	}

	sys_timeout(MQTT_UTK_DRAIN_MS, drain_timeout, NULL);
}

/* ------------------------------------------------------------------ */
/* Incoming: robot/cmd -> subsystem 1's Command Handler                 */
/* ------------------------------------------------------------------ */
static void incoming_publish_cb(void *arg, const char *topic, u32_t tot_len)
{
	const char	*want = MQTT_UTK_TOPIC_CMD;
	u32_t		i;

	(void)arg;
	for (i = 0; want[i] != '\0' && topic[i] == want[i]; i++) {
	}
	rx_is_cmd   = (want[i] == '\0' && topic[i] == '\0');
	rx_len      = 0;
	rx_too_long = (tot_len > sizeof rx_buf);
}

static void incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags)
{
	u16_t	i;

	(void)arg;
	if (!rx_is_cmd) {
		return;
	}
	for (i = 0; i < len; i++) {
		if (rx_len < sizeof rx_buf) {
			rx_buf[rx_len++] = data[i];
		} else {
			rx_too_long = 1;
		}
	}
	if ((flags & MQTT_DATA_FLAG_LAST) != 0) {
		/* An oversized command is passed on as a single 0x00 byte
		 * (message type 0), which the Command Handler rejects as
		 * INVALID. */
		if (rx_too_long) {
			rx_buf[0] = 0;
			rx_len = 1;
		}
		s1_net_rx_cmd(rx_buf, rx_len);
		rx_is_cmd = 0;
	}
}

/* ------------------------------------------------------------------ */
/* Connection                                                           */
/* ------------------------------------------------------------------ */
static void connection_cb(mqtt_client_t *c, void *arg, mqtt_connection_status_t status)
{
	(void)c;
	(void)arg;

	if (status == MQTT_CONNECT_ACCEPTED) {
		sys_untimeout(connack_timeout, NULL);
		state = ST_CONNECTED;
		tm_printf((UB*)"mqtt: CONNECTED to " MQTT_BROKER_IP "\n");

		/* entry: subscribe robot/cmd, publish robot/status = ONLINE */
		mqtt_set_inpub_callback(client, incoming_publish_cb,
					incoming_data_cb, NULL);
		(void)mqtt_subscribe(client, MQTT_UTK_TOPIC_CMD, 1, NULL, NULL);
		(void)mqtt_publish(client, MQTT_UTK_TOPIC_STATUS, "ONLINE", 6,
				   1, 1, NULL, NULL);

		s1_net_link(1);		/* LINK_UP = 1 */

		sys_untimeout(drain_timeout, NULL);
		sys_timeout(MQTT_UTK_DRAIN_MS, drain_timeout, NULL);
	} else if (state == ST_CONNECTING || state == ST_CONNECTED) {
		/* refused CONNACK, keep-alive timeout or socket error */
		sys_untimeout(connack_timeout, NULL);
		go_backoff(status == MQTT_CONNECT_TIMEOUT ? "keep-alive timeout"
			   : "connection closed");
	}
}

void mqtt_utk_start(void)
{
	struct mqtt_connect_client_info_t	ci = {0};
	err_t					err;

	if (state != ST_IDLE) {
		return;
	}

	if (client == NULL) {
		client = mqtt_client_new();
		if (client == NULL) {
			tm_printf((UB*)"mqtt: mqtt_client_new() failed\n");
			return;
		}
		if (!ip4addr_aton(MQTT_BROKER_IP, ip_2_ip4(&broker_addr))) {
			tm_printf((UB*)"mqtt: bad MQTT_BROKER_IP\n");
			return;
		}
		IP_SET_TYPE(&broker_addr, IPADDR_TYPE_V4);
	}

	ci.client_id   = MQTT_CLIENT_ID;
	ci.keep_alive  = MQTT_UTK_KEEPALIVE_S;
	ci.will_topic  = MQTT_UTK_TOPIC_STATUS;	/* broker publishes OFFLINE */
	ci.will_msg    = "OFFLINE";		/* if we vanish without notice */
	ci.will_qos    = 1;
	ci.will_retain = 1;

	tm_printf((UB*)"mqtt: MQTT_CONNECTING to " MQTT_BROKER_IP "\n");
	state = ST_CONNECTING;
	err = mqtt_client_connect(client, &broker_addr, MQTT_BROKER_PORT,
				  connection_cb, NULL, &ci);
	if (err != ERR_OK) {
		go_backoff("connect request failed");
		return;
	}
	sys_untimeout(connack_timeout, NULL);
	sys_timeout(MQTT_UTK_CONNACK_MS, connack_timeout, NULL);
}