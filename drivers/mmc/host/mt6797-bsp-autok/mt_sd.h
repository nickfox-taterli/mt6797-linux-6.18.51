/* SPDX-License-Identifier: GPL-2.0-only */
/* Linux 6.18 host-layout shim for the original MT6797 BSP AutoK. */
#ifndef MT6797_AUTOK_HOST_SHIM_H
#define MT6797_AUTOK_HOST_SHIM_H
#include <linux/io.h>
#include <linux/mmc/host.h>
#include "msdc_reg.h"
#define MSDC_FIFO_SZ 128
#define MSDC_EMMC 0
#define MSDC_SD 1
#define MSDC_SDIO 2
struct msdc_hw {
 unsigned char host_function;
};
struct msdc_host {
 void __iomem *base;
 struct mmc_host *mmc;
 struct msdc_hw *hw;
 unsigned int id;
 unsigned int hclk;
 unsigned int sclk;
 unsigned int tune_latch_ck_cnt;
 int autok_error;
};
/* Verbatim BSP bit-field and synchronized register access helpers. */
static inline unsigned int uffs(unsigned int x)
{
	unsigned int r = 1;

	if (!x)
		return 0;
	if (!(x & 0xffff)) {
		x >>= 16;
		r += 16;
	}
	if (!(x & 0xff)) {
		x >>= 8;
		r += 8;
	}
	if (!(x & 0xf)) {
		x >>= 4;
		r += 4;
	}
	if (!(x & 3)) {
		x >>= 2;
		r += 2;
	}
	if (!(x & 1)) {
		x >>= 1;
		r += 1;
	}
	return r;
}

#define MSDC_READ8(reg)           __raw_readb((const volatile void *)reg)
#define MSDC_READ16(reg)          __raw_readw((const volatile void *)reg)
#define MSDC_READ32(reg)          __raw_readl((const volatile void *)reg)
#define MSDC_WRITE8(reg, val)     mt_reg_sync_writeb(val, reg)
#define MSDC_WRITE16(reg, val)    mt_reg_sync_writew(val, reg)
#define MSDC_WRITE32(reg, val)    mt_reg_sync_writel(val, reg)

#define UNSTUFF_BITS(resp, start, size) \
({ \
	const int __size = size; \
	const u32 __mask = (__size < 32 ? 1 << __size : 0) - 1; \
	const int __off = 3 - ((start) / 32); \
	const int __shft = (start) & 31; \
	u32 __res; \
	__res = resp[__off] >> __shft; \
	if (__size + __shft > 32) \
		__res |= resp[__off-1] << ((32 - __shft) % 32); \
	__res & __mask; \
})

#define MSDC_SET_BIT32(reg, bs) \
	do { \
		volatile unsigned int tv = MSDC_READ32(reg);\
		tv |= (u32)(bs); \
		MSDC_WRITE32(reg, tv); \
	} while (0)

#define MSDC_CLR_BIT32(reg, bs) \
	do { \
		volatile unsigned int tv = MSDC_READ32(reg);\
		tv &= ~((u32)(bs)); \
		MSDC_WRITE32(reg, tv); \
	} while (0)

#define MSDC_SET_FIELD(reg, field, val) \
	do { \
		volatile unsigned int tv = MSDC_READ32(reg); \
		tv &= ~(field); \
		tv |= ((val) << (uffs((unsigned int)field) - 1)); \
		MSDC_WRITE32(reg, tv); \
	} while (0)

#define MSDC_GET_FIELD(reg, field, val) \
	do { \
		volatile unsigned int tv = MSDC_READ32(reg); \
		val = ((tv & (field)) >> (uffs((unsigned int)field) - 1)); \
	} while (0)


#endif
