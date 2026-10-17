/* SPDX-License-Identifier: GPL-2.0-only */
/* MT6797 BSP BT task carries an H4 byte stream, not STP-sized HCI packets. */
#ifndef __BTMT6797_H4_H
#define __BTMT6797_H4_H

#include <linux/errno.h>
#include <linux/string.h>
#include <linux/types.h>

#define MT6797_H4_MAX_FRAME 1028

struct mt6797_h4 {
	u8 data[MT6797_H4_MAX_FRAME];
	u16 used;
	u16 expected;
	u8 type;
	u8 header;
};

static void mt6797_h4_reset(struct mt6797_h4 *rx)
{
	rx->used = 0;
	rx->expected = 0;
	rx->type = 0;
	rx->header = 0;
}

static int mt6797_h4_feed(struct mt6797_h4 *rx, const u8 *data, size_t size,
			  int (*emit)(void *, u8, const u8 *, size_t), void *context)
{
	while (size) {
		size_t copy;
		unsigned int payload;
		int ret;

		if (!rx->type) {
			rx->type = *data++;
			size--;
			switch (rx->type) {
			case 0x02: /* ACL: handle, little-endian 16-bit data length */
				rx->header = 4;
				break;
			case 0x03: /* SCO: handle, 8-bit data length */
				rx->header = 3;
				break;
			case 0x04: /* Event: event code, 8-bit parameter length */
				rx->header = 2;
				break;
			default:
				mt6797_h4_reset(rx);
				return -EILSEQ;
			}
			rx->expected = rx->header;
		}
		copy = rx->expected - rx->used;
		if (copy > size)
			copy = size;
		memcpy(rx->data + rx->used, data, copy);
		rx->used += copy;
		data += copy;
		size -= copy;
		if (rx->used < rx->expected)
			continue;
		if (rx->used == rx->header) {
			if (rx->type == 0x02)
				payload = rx->data[2] | (rx->data[3] << 8);
			else
				payload = rx->data[rx->header - 1];
			if (payload > sizeof(rx->data) - rx->header) {
				mt6797_h4_reset(rx);
				return -EMSGSIZE;
			}
			rx->expected += payload;
			if (payload)
				continue;
		}
		ret = emit(context, rx->type, rx->data, rx->used);
		mt6797_h4_reset(rx);
		if (ret)
			return ret;
	}
	return 0;
}

#endif
