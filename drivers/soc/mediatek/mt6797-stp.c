// SPDX-License-Identifier: GPL-2.0-only
//
// MT6797 ConSys STP full-mode wire codec.
//
// This file intentionally has no BTIF or WMT device probe.  It is a small,
// allocation-free codec that a future STP client can feed from a workqueue
// after the BTIF hard-IRQ callback has copied bytes.  Power, sequencing,
// retransmission and function ownership stay above this layer.

#include <linux/errno.h>
#include <linux/export.h>
#include <linux/module.h>
#include <linux/string.h>

#include <linux/mt6797-stp.h>

#define STP_DELIMITER_BYTE	0x55
#define STP_SYNC_MASK		0xc0
#define STP_SYNC_VALUE		0x80
#define STP_MAX_TASK_TYPE	7
#define STP_LENGTH_HIGH_MASK	0x0f

enum mt6797_stp_decode_state {
	STP_DECODE_HEADER = 0,
	STP_DECODE_PAYLOAD,
	STP_DECODE_CRC,
};

u16 mt6797_stp_crc16(const u8 *data, size_t length)
{
	u16 crc = 0;

	while (length--) {
		u8 bit;

		crc ^= *data++;
		for (bit = 0; bit < 8; bit++)
			crc = (crc >> 1) ^ ((crc & 1) ? 0xa001 : 0);
	}
	return crc;
}
EXPORT_SYMBOL_GPL(mt6797_stp_crc16);

ssize_t mt6797_stp_encode_frame(u8 *out, size_t out_size, u8 packet_type,
					u8 sequence, u8 acknowledgement, bool nak,
					const u8 *payload, u16 payload_length,
					bool delimiter)
{
	size_t needed = MT6797_STP_HEADER_SIZE + payload_length +
			(payload_length ? MT6797_STP_CRC_SIZE : 0) +
			(delimiter ? 2 : 0);
	u8 *header;
	u16 crc;

	if (!out || (!payload && payload_length) || packet_type > STP_MAX_TASK_TYPE ||
		sequence > 7 || acknowledgement > 7 ||
		payload_length > MT6797_STP_MAX_PAYLOAD || out_size < needed)
		return -EINVAL;
	if (delimiter) {
		out[0] = STP_DELIMITER_BYTE;
		out[1] = STP_DELIMITER_BYTE;
	}
	header = out + (delimiter ? 2 : 0);
	header[0] = 0x80 | (sequence << 3) | acknowledgement;
	header[1] = (nak ? 0x80 : 0) | (packet_type << 4) |
		((payload_length >> 8) & STP_LENGTH_HIGH_MASK);
	header[2] = payload_length & 0xff;
	header[3] = header[0] + header[1] + header[2];
	if (!payload_length)
		return needed;
	memcpy(header + MT6797_STP_HEADER_SIZE, payload, payload_length);
	crc = mt6797_stp_crc16(payload, payload_length);
	header[MT6797_STP_HEADER_SIZE + payload_length] = crc & 0xff;
	header[MT6797_STP_HEADER_SIZE + payload_length + 1] = crc >> 8;
	return needed;
}
EXPORT_SYMBOL_GPL(mt6797_stp_encode_frame);

ssize_t mt6797_stp_encode_mandatory_frame(u8 *out, size_t out_size,
						  u8 packet_type,
						  const u8 *payload,
						  u16 payload_length,
						  bool delimiter)
{
	size_t needed = MT6797_STP_HEADER_SIZE + payload_length +
			(payload_length ? MT6797_STP_CRC_SIZE : 0) +
			(delimiter ? 2 : 0);
	u8 *header;

	if (!out || (!payload && payload_length) || packet_type > STP_MAX_TASK_TYPE ||
		payload_length > MT6797_STP_MAX_PAYLOAD || out_size < needed)
		return -EINVAL;
	if (delimiter) {
		out[0] = STP_DELIMITER_BYTE;
		out[1] = STP_DELIMITER_BYTE;
	}
	header = out + (delimiter ? 2 : 0);
	header[0] = 0x80;
	header[1] = (packet_type << 4) |
		((payload_length >> 8) & STP_LENGTH_HIGH_MASK);
	header[2] = payload_length & 0xff;
	/* Mandatory mode deliberately does not use header/CRC checksums. */
	header[3] = 0;
	if (payload_length)
		memcpy(header + MT6797_STP_HEADER_SIZE, payload, payload_length);
	if (payload_length) {
		header[MT6797_STP_HEADER_SIZE + payload_length] = 0;
		header[MT6797_STP_HEADER_SIZE + payload_length + 1] = 0;
	}
	return needed;
}
EXPORT_SYMBOL_GPL(mt6797_stp_encode_mandatory_frame);

ssize_t mt6797_stp_encode_wmt(u8 *out, size_t out_size, u8 opcode,
				      const u8 *params, u16 parameter_length)
{
	if (parameter_length > U16_MAX - 4 ||
		(!params && parameter_length) || out_size < parameter_length + 4)
		return -EINVAL;
	out[0] = 0x01; /* WMT command PDU */
	out[1] = opcode;
	out[2] = parameter_length & 0xff;
	out[3] = parameter_length >> 8;
	if (parameter_length)
		memcpy(out + 4, params, parameter_length);
	return parameter_length + 4;
}
EXPORT_SYMBOL_GPL(mt6797_stp_encode_wmt);

void mt6797_stp_decoder_init(struct mt6797_stp_decoder *decoder)
{
	memset(decoder, 0, sizeof(*decoder));
	decoder->state = STP_DECODE_HEADER;
}
EXPORT_SYMBOL_GPL(mt6797_stp_decoder_init);

void mt6797_stp_decoder_set_mandatory(struct mt6797_stp_decoder *decoder,
					       bool mandatory)
{
	if (!decoder)
		return;
	decoder->mandatory = mandatory;
}
EXPORT_SYMBOL_GPL(mt6797_stp_decoder_set_mandatory);

static void mt6797_stp_decoder_reset(struct mt6797_stp_decoder *decoder)
{
	decoder->header_position = 0;
	decoder->payload_length = 0;
	decoder->payload_position = 0;
	decoder->crc_position = 0;
	decoder->state = STP_DECODE_HEADER;
}

static int mt6797_stp_emit(struct mt6797_stp_decoder *decoder,
				   mt6797_stp_frame_cb callback, void *context,
				   bool has_crc)
{
	struct mt6797_stp_frame frame = {
		.sequence = (decoder->header[0] >> 3) & 7,
		.acknowledgement = decoder->header[0] & 7,
		.packet_type = (decoder->header[1] >> 4) & 7,
		.nak = !!(decoder->header[1] & 0x80),
		.has_crc = has_crc,
		.payload = decoder->payload,
		.payload_length = decoder->payload_length,
	};

	if (!callback)
		return 0;
	return callback(context, &frame);
}

int mt6797_stp_decoder_feed(struct mt6797_stp_decoder *decoder,
				    const u8 *data, size_t length,
				    mt6797_stp_frame_cb callback, void *context)
{
	int frames = 0;

	if (!decoder || (!data && length))
		return -EINVAL;
	while (length--) {
		u8 byte = *data++;

		if (decoder->state == STP_DECODE_HEADER) {
			if (decoder->header_position == 0) {
				if (byte == STP_DELIMITER_BYTE)
					continue;
				if ((byte & STP_SYNC_MASK) != STP_SYNC_VALUE) {
					decoder->bad_frames++;
					continue;
				}
			}
			decoder->header[decoder->header_position++] = byte;
			if (decoder->header_position < MT6797_STP_HEADER_SIZE)
				continue;
			if ((!decoder->mandatory &&
			     (u8)(decoder->header[0] + decoder->header[1] +
				  decoder->header[2]) != decoder->header[3]) ||
			    ((decoder->header[1] >> 4) & 7) > STP_MAX_TASK_TYPE) {
				decoder->bad_frames++;
				mt6797_stp_decoder_reset(decoder);
				continue;
			}
			decoder->payload_length =
				((decoder->header[1] & STP_LENGTH_HIGH_MASK) << 8) |
				decoder->header[2];
			if (decoder->payload_length > MT6797_STP_MAX_PAYLOAD) {
				decoder->bad_frames++;
				mt6797_stp_decoder_reset(decoder);
				continue;
			}
			if (!decoder->payload_length) {
				int ret = mt6797_stp_emit(decoder, callback, context, false);

				mt6797_stp_decoder_reset(decoder);
				if (ret)
					return ret;
				frames++;
			} else {
				decoder->state = STP_DECODE_PAYLOAD;
			}
			continue;
		}

		if (decoder->state == STP_DECODE_PAYLOAD) {
			decoder->payload[decoder->payload_position++] = byte;
			if (decoder->payload_position == decoder->payload_length) {
				decoder->state = STP_DECODE_CRC;
				decoder->crc_position = 0;
			}
			continue;
		}

		decoder->crc[decoder->crc_position++] = byte;
		if (decoder->crc_position == MT6797_STP_CRC_SIZE) {
			u16 actual = decoder->crc[0] | (decoder->crc[1] << 8);
			u16 expected = mt6797_stp_crc16(decoder->payload,
								 decoder->payload_length);
			int ret = 0;

			if (!decoder->mandatory && actual != expected) {
				decoder->bad_frames++;
			} else {
				ret = mt6797_stp_emit(decoder, callback, context, true);
				frames++;
			}
			mt6797_stp_decoder_reset(decoder);
			if (ret)
				return ret;
		}
	}
	return frames;
}
EXPORT_SYMBOL_GPL(mt6797_stp_decoder_feed);

static int __init mt6797_stp_init(void)
{
	return 0;
}
module_init(mt6797_stp_init);

static void __exit mt6797_stp_exit(void)
{
}
module_exit(mt6797_stp_exit);

MODULE_DESCRIPTION("MT6797 ConSys STP wire codec");
MODULE_LICENSE("GPL");
