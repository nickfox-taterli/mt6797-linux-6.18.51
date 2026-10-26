// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2016 MediaTek Inc.
 * Author: Ming Hsiu Tsai <minghsiu.tsai@mediatek.com>
 *         Rick Chang <rick.chang@mediatek.com>
 */

#include <linux/kernel.h>
#include <linux/videodev2.h>
#include <media/jpeg.h>

#include "mtk_jpeg_dec_parse.h"

struct mtk_jpeg_stream {
	u8 *addr;
	u32 size;
	u32 curr;
};

static int read_byte(struct mtk_jpeg_stream *stream)
{
	if (stream->curr >= stream->size)
		return -1;
	return stream->addr[stream->curr++];
}

static int read_word_be(struct mtk_jpeg_stream *stream, u32 *word)
{
	u32 temp;
	int byte;

	byte = read_byte(stream);
	if (byte == -1)
		return -1;
	temp = byte << 8;
	byte = read_byte(stream);
	if (byte == -1)
		return -1;
	*word = (u32)byte | temp;

	return 0;
}

static bool mtk_jpeg_do_parse(struct mtk_jpeg_dec_param *param, u8 *src_addr_va,
			      u32 src_size)
{
	struct mtk_jpeg_stream stream;

	stream.addr = src_addr_va;
	stream.size = src_size;
	stream.curr = 0;

	if (!src_addr_va || read_byte(&stream) != 0xff ||
	    read_byte(&stream) != JPEG_MARKER_SOI)
		return false;

	while (stream.curr < stream.size) {
		int i, byte, marker;
		u32 length, word;

		byte = read_byte(&stream);
		if (byte != 0xff)
			return false;
		do
			byte = read_byte(&stream);
		while (byte == 0xff);
		if (byte <= 0 || byte == JPEG_MARKER_EOI ||
		    byte == JPEG_MARKER_SOS)
			return false;
		if (byte == JPEG_MARKER_TEM)
			continue;

		marker = byte;
		/* Only baseline sequential, 8-bit Huffman frames are supported.
		 * Do not search through unsupported frame entropy for a false SOF0. */
		if (marker >= 0xc0 && marker <= 0xcf &&
		    marker != JPEG_MARKER_SOF0 && marker != JPEG_MARKER_DHT)
			return false;
		if (read_word_be(&stream, &length) || length < 2 ||
		    length - 2 > stream.size - stream.curr)
			return false;
		if (marker != JPEG_MARKER_SOF0) {
			stream.curr += length - 2;
			continue;
		}
		if (length < 8 || read_byte(&stream) != 8 ||
		    read_word_be(&stream, &word) || !word)
			return false;
		param->pic_h = word;
		if (read_word_be(&stream, &word) || !word)
			return false;
		param->pic_w = word;
		param->comp_num = read_byte(&stream);
		if ((param->comp_num != 1 && param->comp_num != 3) ||
		    length != 8 + 3 * param->comp_num)
			return false;
		for (i = 0; i < param->comp_num; i++) {
			int j;

			param->comp_id[i] = read_byte(&stream);
			for (j = 0; j < i; j++)
				if (param->comp_id[j] == param->comp_id[i])
					return false;
			byte = read_byte(&stream);
			param->sampling_w[i] = byte >> 4;
			param->sampling_h[i] = byte & 0x0f;
			param->qtbl_num[i] = read_byte(&stream);
			if (!param->sampling_w[i] || !param->sampling_h[i] ||
			    param->qtbl_num[i] > 3)
				return false;
		}
		return true;
	}

	return false;
}

bool mtk_jpeg_parse(struct mtk_jpeg_dec_param *param, u8 *src_addr_va,
		    u32 src_size)
{
	if (!mtk_jpeg_do_parse(param, src_addr_va, src_size))
		return false;
	if (mtk_jpeg_dec_fill_param(param))
		return false;

	return true;
}
