/* SPDX-License-Identifier: GPL-2.0-only */
/* Wire codec for the MT6797 ConSys STP full-mode transport. */
#ifndef __LINUX_MT6797_STP_H
#define __LINUX_MT6797_STP_H

#include <linux/types.h>

#define MT6797_STP_MAX_PAYLOAD	2048
#define MT6797_STP_HEADER_SIZE	4
#define MT6797_STP_CRC_SIZE	2

struct mt6797_stp_frame {
	u8 sequence;
	u8 acknowledgement;
	u8 packet_type;
	bool nak;
	bool has_crc;
	const u8 *payload;
	u16 payload_length;
};

struct mt6797_stp_decoder {
	u8 header[MT6797_STP_HEADER_SIZE];
	u8 payload[MT6797_STP_MAX_PAYLOAD];
	u16 payload_length;
	u16 payload_position;
	u8 crc[MT6797_STP_CRC_SIZE];
	u8 crc_position;
	u8 header_position;
	u8 state;
	u32 bad_frames;
};

typedef int (*mt6797_stp_frame_cb)(void *context,
					 const struct mt6797_stp_frame *frame);

u16 mt6797_stp_crc16(const u8 *data, size_t length);
ssize_t mt6797_stp_encode_frame(u8 *out, size_t out_size, u8 packet_type,
					u8 sequence, u8 acknowledgement, bool nak,
					const u8 *payload, u16 payload_length,
					bool delimiter);
ssize_t mt6797_stp_encode_wmt(u8 *out, size_t out_size, u8 opcode,
				      const u8 *params, u16 parameter_length);
void mt6797_stp_decoder_init(struct mt6797_stp_decoder *decoder);
int mt6797_stp_decoder_feed(struct mt6797_stp_decoder *decoder,
				    const u8 *data, size_t length,
				    mt6797_stp_frame_cb callback, void *context);

#endif /* __LINUX_MT6797_STP_H */
