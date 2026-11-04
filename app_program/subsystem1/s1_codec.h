/*
 *	s1_codec.h
 *	Subsystem 1 -- binary MQTT payload layouts (minimum-bit representation).
 *
 *	Pure C with <stdint.h> only, so it can also be unit-tested on a PC.
 *	Bits are packed most-significant-bit first: bit 0 of a message is the
 *	top bit of byte 0.  Every message starts with a 4-bit header:
 *	  bits 0..1  message type (0 telemetry, 1 heartbeat, 2 command, 3 ack)
 *	  bits 2..3  layout version
 *
 *	TELEMETRY (type 0), 18 bytes, robot -> dashboard, every 500 ms
 *	(fixed length; slot widths agreed at the Week 8 tribunal)
 *	  bits   0..3   header (type, version)
 *	  bits   4..29  Buddy 2 slot (26)
 *	  bits  30..37  Buddy 3 slot (8)
 *	  bits  38..81  Buddy 4 slot (44)
 *	  bits  82..136 Buddy 5 slot (55)
 *	  bit  137      Wi-Fi loss since the last telemetry (1 = yes),
 *	                the LAST data bit of the payload (tribunal decision)
 *	  bits 138..143 padding (0)
 *	If a buddy's width changes, edit S1_Bx_BITS below and bump S1_VERSION;
 *	the frame length is recomputed automatically.
 *
 *	HEARTBEAT (type 1), 5 bytes, robot -> dashboard, every 1 s
 *	  header(4) | seq(8) | mission state(3) | uptime s(16) |
 *	  link losses(4, saturates at 15) | pad(5)
 *
 *	COMMAND (type 2), 2 bytes, dashboard -> robot
 *	  header(4) | command(2): 00 START, 01 STOP, 10 E-STOP (11 invalid) |
 *	  id(6): 0..63, changes with every new command (duplicate filter) |
 *	  pad(4)
 *
 *	MAP (type 2, command code 11), 4 bytes, dashboard -> robot
 *	(1 Nov: barcode letter -> movement mapping, sent before the demo)
 *	  header(4) | command(2) = 11 | id(6) |
 *	  forward(5) | U-turn(5) | left(5) | right(5)
 *	  each letter 0..25 = 'A'..'Z' (26..31 invalid); the four letters
 *	  must be different.  32 bits exactly, no padding.
 *
 *	ACK (type 3), 2 bytes, robot -> dashboard, only on rejection
 *	  header(4) | result(1): 0 INVALID, 1 FAILED | echoed id(6) | pad(5)
 */

#ifndef S1_CODEC_H
#define S1_CODEC_H

#include <stdint.h>

#define S1_TYPE_BITS		2
#define S1_VERSION_BITS		2
#define S1_HDR_BITS		(S1_TYPE_BITS + S1_VERSION_BITS)	/* 4 */
#define S1_VERSION		1	/* 0..3 */

#define S1_TYPE_TELEMETRY	0
#define S1_TYPE_HEARTBEAT	1
#define S1_TYPE_COMMAND		2
#define S1_TYPE_ACK		3

#define S1_HB_LEN		5
#define S1_CMD_LEN		2
#define S1_MAP_LEN		4	/* MAP command (1 Nov) */
#define S1_ACK_LEN		2

#define S1_B2_BITS		26
#define S1_B3_BITS		8
#define S1_B4_BITS		44
#define S1_B5_BITS		55

/* header + four slots + Wi-Fi-loss bit, rounded up to whole bytes */
#define S1_TLM_BITS	(S1_HDR_BITS + S1_B2_BITS + S1_B3_BITS + \
			 S1_B4_BITS + S1_B5_BITS + 1)		/* = 138 */
#define S1_TLM_LEN	((S1_TLM_BITS + 7) / 8)			/* = 18  */

/* 2-bit command codes (tribunal decision) */
#define S1_CMD_START		0	/* 00 */
#define S1_CMD_STOP		1	/* 01 */
#define S1_CMD_ESTOP		2	/* 10 */
#define S1_CMD_INVALID		3	/* 11 in a 2-byte command, also returned for bad messages */
#define S1_CMD_MAP		4	/* 11 in a 4-byte message: letter mapping (1 Nov) */

/* MAP: order of the four letters in the message */
#define S1_MAP_FORWARD		0
#define S1_MAP_UTURN		1
#define S1_MAP_LEFT		2
#define S1_MAP_RIGHT		3
#define S1_MAP_COUNT		4
#define S1_MAP_LETTER_BITS	5

#define S1_ACK_INVALID		0
#define S1_ACK_FAILED		1

/* Generic MSB-first bit writer / reader. pos is a bit index into buf. */
void	 s1_put_bits(uint8_t *buf, uint32_t *pos, uint64_t value, uint32_t nbits);
uint64_t s1_get_bits(const uint8_t *buf, uint32_t *pos, uint32_t nbits);

/* slots[0..3] = Buddy 2..5 packed fields. Returns S1_TLM_LEN. */
int s1_encode_telemetry(uint8_t out[S1_TLM_LEN], int wifi_loss,
			const uint64_t slots[4]);

int s1_encode_heartbeat(uint8_t out[S1_HB_LEN], uint8_t seq,
			uint8_t mission_state, uint32_t uptime_s,
			uint32_t link_losses);

int s1_encode_ack(uint8_t out[S1_ACK_LEN], int result, uint8_t id);

/* Returns S1_CMD_START / STOP / ESTOP and writes *id, or S1_CMD_INVALID
 * if the message is not a valid command of this version. */
int s1_decode_command(const uint8_t *buf, int len, uint8_t *id);

/* 1 Nov: a MAP message.  Returns S1_CMD_MAP and writes *id and
 * letters[S1_MAP_FORWARD..S1_MAP_RIGHT] as 'A'..'Z', or S1_CMD_INVALID
 * (wrong type/version/length/code, a letter > 'Z', or a letter used
 * twice).  s1_decode_command() still handles the 2-byte commands. */
int s1_decode_map(const uint8_t *buf, int len, uint8_t *id,
		  char letters[S1_MAP_COUNT]);

/* Builds a MAP message (dashboard side, and the tests). Returns S1_MAP_LEN. */
int s1_encode_map(uint8_t out[S1_MAP_LEN], uint8_t id,
		  const char letters[S1_MAP_COUNT]);

#endif /* S1_CODEC_H */