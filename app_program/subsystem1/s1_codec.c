/*
 *	s1_codec.c
 *	Subsystem 1 -- pack / unpack the binary MQTT payloads (see s1_codec.h).
 *	Explicit masks and shifts are used instead of C bit-fields, because the
 *	layout of a bit-field struct is compiler-dependent and the dashboard
 *	must decode exactly the same bit positions.
 */

#include "s1_codec.h"

void s1_put_bits(uint8_t *buf, uint32_t *pos, uint64_t value, uint32_t nbits)
{
	uint32_t i;

	for (i = 0; i < nbits; i++) {
		uint32_t bit  = (uint32_t)((value >> (nbits - 1 - i)) & 1u);
		uint32_t byte = *pos >> 3;
		uint32_t sh   = 7 - (*pos & 7);

		buf[byte] = (uint8_t)((buf[byte] & ~(1u << sh)) | (bit << sh));
		(*pos)++;
	}
}

uint64_t s1_get_bits(const uint8_t *buf, uint32_t *pos, uint32_t nbits)
{
	uint64_t v = 0;
	uint32_t i;

	for (i = 0; i < nbits; i++) {
		v = (v << 1) | ((buf[*pos >> 3] >> (7 - (*pos & 7))) & 1u);
		(*pos)++;
	}
	return v;
}

static void put_header(uint8_t *buf, uint32_t *pos, uint32_t type)
{
	s1_put_bits(buf, pos, type, S1_TYPE_BITS);
	s1_put_bits(buf, pos, S1_VERSION, S1_VERSION_BITS);
}

static uint64_t mask(uint32_t nbits)
{
	return (nbits >= 64) ? ~0ull : ((1ull << nbits) - 1u);
}

int s1_encode_telemetry(uint8_t out[S1_TLM_LEN], int wifi_loss,
			const uint64_t slots[4])
{
	static const uint32_t width[4] = { S1_B2_BITS, S1_B3_BITS,
					   S1_B4_BITS, S1_B5_BITS };
	uint32_t pos = 0;
	int i;

	for (i = 0; i < S1_TLM_LEN; i++) {
		out[i] = 0;		/* also zeroes the padding bits */
	}
	put_header(out, &pos, S1_TYPE_TELEMETRY);
	for (i = 0; i < 4; i++) {
		s1_put_bits(out, &pos, slots[i] & mask(width[i]), width[i]);
	}
	/* Wi-Fi-loss bit at the end of the payload (tribunal decision) */
	s1_put_bits(out, &pos, wifi_loss ? 1u : 0u, 1);
	return S1_TLM_LEN;		/* pos == S1_TLM_BITS, rest is padding */
}

int s1_encode_heartbeat(uint8_t out[S1_HB_LEN], uint8_t seq,
			uint8_t mission_state, uint32_t uptime_s,
			uint32_t link_losses, char barcode)
{
	uint32_t pos = 0;
	int i;

	for (i = 0; i < S1_HB_LEN; i++) {
		out[i] = 0;
	}
	if (uptime_s > 0xFFFFu)  uptime_s = 0xFFFFu;	/* ~18 h, saturates */
	if (link_losses > 15u)   link_losses = 15u;

	put_header(out, &pos, S1_TYPE_HEARTBEAT);
	s1_put_bits(out, &pos, seq, 8);
	s1_put_bits(out, &pos, mission_state & 0x7u, 3);
	s1_put_bits(out, &pos, uptime_s, 16);
	s1_put_bits(out, &pos, link_losses, 4);
	/* 4 Nov: last barcode letter in the 5 bits that were padding */
	s1_put_bits(out, &pos, (barcode >= 'A' && barcode <= 'Z') ?
		    (uint32_t)(barcode - 'A' + 1) : 0u, 5);
	return S1_HB_LEN;
}

int s1_encode_ack(uint8_t out[S1_ACK_LEN], int result, uint8_t id)
{
	uint32_t pos = 0;

	out[0] = out[1] = 0;
	put_header(out, &pos, S1_TYPE_ACK);
	s1_put_bits(out, &pos, result ? 1u : 0u, 1);
	s1_put_bits(out, &pos, id & 0x3Fu, 6);
	return S1_ACK_LEN;
}

int s1_decode_command(const uint8_t *buf, int len, uint8_t *id)
{
	uint32_t pos = 0;
	uint32_t type, version, cmd;

	*id = 0;
	if (buf == 0 || len != S1_CMD_LEN) {
		return S1_CMD_INVALID;
	}
	type    = (uint32_t)s1_get_bits(buf, &pos, S1_TYPE_BITS);
	version = (uint32_t)s1_get_bits(buf, &pos, S1_VERSION_BITS);
	cmd     = (uint32_t)s1_get_bits(buf, &pos, 2);
	*id     = (uint8_t)s1_get_bits(buf, &pos, 6);

	if (type != S1_TYPE_COMMAND || version != S1_VERSION) {
		return S1_CMD_INVALID;
	}
	return (int)cmd;	/* 00 START, 01 STOP, 10 E-STOP, 11 invalid */
}

/* 1 Nov: MAP command, header(4) | 11 | id(6) | 4 x letter(5) = 32 bits */
int s1_decode_map(const uint8_t *buf, int len, uint8_t *id,
		  char letters[S1_MAP_COUNT])
{
	uint32_t pos = 0;
	uint32_t type, version, cmd, v;
	int i, j;

	*id = 0;
	if (buf == 0 || len != S1_MAP_LEN) {
		return S1_CMD_INVALID;
	}
	type    = (uint32_t)s1_get_bits(buf, &pos, S1_TYPE_BITS);
	version = (uint32_t)s1_get_bits(buf, &pos, S1_VERSION_BITS);
	cmd     = (uint32_t)s1_get_bits(buf, &pos, 2);
	*id     = (uint8_t)s1_get_bits(buf, &pos, 6);
	if (type != S1_TYPE_COMMAND || version != S1_VERSION || cmd != 3u) {
		return S1_CMD_INVALID;
	}
	for (i = 0; i < S1_MAP_COUNT; i++) {
		v = (uint32_t)s1_get_bits(buf, &pos, S1_MAP_LETTER_BITS);
		if (v > 25u) {
			return S1_CMD_INVALID;		/* not A..Z */
		}
		letters[i] = (char)('A' + v);
		for (j = 0; j < i; j++) {
			if (letters[j] == letters[i]) {
				return S1_CMD_INVALID;	/* one letter, two moves */
			}
		}
	}
	return S1_CMD_MAP;
}

int s1_encode_map(uint8_t out[S1_MAP_LEN], uint8_t id,
		  const char letters[S1_MAP_COUNT])
{
	uint32_t pos = 0;
	int i;

	for (i = 0; i < S1_MAP_LEN; i++) {
		out[i] = 0;
	}
	put_header(out, &pos, S1_TYPE_COMMAND);
	s1_put_bits(out, &pos, 3u, 2);
	s1_put_bits(out, &pos, id & 0x3Fu, 6);
	for (i = 0; i < S1_MAP_COUNT; i++) {
		s1_put_bits(out, &pos, (uint32_t)(letters[i] - 'A') & 0x1Fu,
			    S1_MAP_LETTER_BITS);
	}
	return S1_MAP_LEN;
}