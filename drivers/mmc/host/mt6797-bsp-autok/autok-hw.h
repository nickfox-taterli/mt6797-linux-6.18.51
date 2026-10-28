/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef MT6797_AUTOK_HW_H
#define MT6797_AUTOK_HW_H
#include <linux/bitops.h>
#include <linux/io.h>
#include "msdc_reg.h"

#define MSDC_FIFO_SZ 128

/* Private state for the MSDC1 SDR104 tuning operation. */
struct mt6797_autok_ctx {
	void __iomem *base;
	unsigned int hclk;
	unsigned int sclk;
	unsigned int tune_latch_ck_cnt;
	int autok_error;
};

/* Keep the BSP's raw 32-bit accesses and full post-write barrier. */
static inline u32 autok_read(void __iomem *reg)
{
	return __raw_readl(reg);
}

static inline void autok_write(void __iomem *reg, u32 value)
{
	__raw_writel(value, reg);
	mb();
}

static inline void autok_set_bits(void __iomem *reg, u32 bits)
{
	autok_write(reg, autok_read(reg) | bits);
}

static inline void autok_clear_bits(void __iomem *reg, u32 bits)
{
	autok_write(reg, autok_read(reg) & ~bits);
}

static inline void autok_set_field(void __iomem *reg, u32 mask, u32 value)
{
	u32 old = autok_read(reg);

	autok_write(reg, (old & ~mask) | (value << __ffs(mask)));
}

static inline u32 autok_get_field(void __iomem *reg, u32 mask)
{
	return (autok_read(reg) & mask) >> __ffs(mask);
}
#endif
