/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef MT6797_BTIF_RING_H
#define MT6797_BTIF_RING_H

#define BTIF_VFIFO_SIZE 8192U
#define BTIF_VFIFO_WRAP 0x10000U

static inline unsigned int btif_vfifo_advance(unsigned int pointer,
					    unsigned int count)
{
	unsigned int offset = (pointer & 0xffff) + count;

	if (offset >= BTIF_VFIFO_SIZE) {
		offset -= BTIF_VFIFO_SIZE;
		pointer ^= BTIF_VFIFO_WRAP;
	}
	return (pointer & BTIF_VFIFO_WRAP) | offset;
}

static inline int btif_vfifo_pending(unsigned int write, unsigned int read)
{
	int count;

	if ((write | read) & ~(BTIF_VFIFO_WRAP | 0xffffU) ||
	    (write & 0xffff) >= BTIF_VFIFO_SIZE ||
	    (read & 0xffff) >= BTIF_VFIFO_SIZE)
		return -1;
	count = (int)(write & 0xffff) - (int)(read & 0xffff);
	if ((write ^ read) & BTIF_VFIFO_WRAP)
		count += BTIF_VFIFO_SIZE;
	return count >= 0 && count <= BTIF_VFIFO_SIZE ? count : -1;
}
#endif
