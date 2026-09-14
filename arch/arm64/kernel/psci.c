// SPDX-License-Identifier: GPL-2.0-only
/*
 *
 * Copyright (C) 2013 ARM Limited
 *
 * Author: Will Deacon <will.deacon@arm.com>
 */

#define pr_fmt(fmt) "psci: " fmt

#include <linux/init.h>
#include <linux/arm-smccc.h>
#include <linux/of.h>
#include <linux/smp.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/psci.h>
#include <linux/mm.h>

#include <uapi/linux/psci.h>

#include <asm/cpu_ops.h>
#include <asm/errno.h>
#include <asm/smp_plat.h>

static int __init cpu_psci_cpu_init(unsigned int cpu)
{
	return 0;
}

static int __init cpu_psci_cpu_prepare(unsigned int cpu)
{
	if (!psci_ops.cpu_on) {
		pr_err("no cpu_on method, not booting CPU%d\n", cpu);
		return -ENODEV;
	}

	return 0;
}

#ifdef CONFIG_ARCH_MEDIATEK
/*
 * MT6797's vendor kernel enables cluster-0 coherency before it asks PSCI to
 * release the first secondary CPU.  The register is MCUSYS write-protected,
 * while the CCI400 port is directly accessible from the non-secure kernel.
 * Keep this narrow and keyed to the MT6797 machine so other MediaTek SoCs
 * using the generic PSCI path are unaffected.
 */
#define MT6797_MCUSYS_MP0_AXI_CONFIG		0x1022002c
#define MT6797_CCI400_BASE			0x10390000
#define MT6797_CCI400_STATUS			0x000c
#define MT6797_CCI400_SI4_SNOOP_CONTROL	0x5000
#define MT6797_ACINACTM				(1U << 4)
#define MT6797_CCI_REQ				0x3
#define MTK_SIP_KERNEL_MCUSYS_WRITE		0x82000201

/*
 * The vendor MT6797 port performs this part of the MP2 power-up in the
 * kernel before it calls PSCI for CPU8/CPU9.  The remaining MP2 reset,
 * PLL and core sequence is performed by the existing secure firmware.
 */
#define MT6797_SPM_BASE_ADDR			0x10006000
#define MT6797_SPM_MP2_CPUSYS_PWR_CON		0x0218
#define MT6797_SPM_CPU_EXT_BUCK_ISO		0x0290
#define MT6797_WDT_BASE_ADDR			0x10007000
#define MT6797_WDT_SWSYSRST			0x0018
#define MT6797_IDVFS_BASE_ADDR			0x10222000
#define MT6797_IDVFS_MP2_PLL_CON0		0x04a0
#define MT6797_WDT_SWSYS_RST_PWRAP_SPI_CTL_RST	0x0800
#define MT6797_WDT_SWSYS_RST_KEY			0x88000000
#define MTK_SIP_KERNEL_IDVFS_BIGIDVFSSRAMLDOSET	0xc20003bf

/* The vendor MT6797 I2C6 controller is left clocked for the vproc buck. */
#define MT6797_I2C6_BASE			0x1100e000
#define MT6797_DA9214_ADDR			0x68
#define MT6797_I2C_DATA_PORT			0x000
#define MT6797_I2C_SLAVE_ADDR			0x004
#define MT6797_I2C_INTR_MASK			0x008
#define MT6797_I2C_INTR_STAT			0x00c
#define MT6797_I2C_CONTROL			0x010
#define MT6797_I2C_TRANSFER_LEN			0x014
#define MT6797_I2C_TRANSAC_LEN			0x018
#define MT6797_I2C_TIMING			0x020
#define MT6797_I2C_START			0x024
#define MT6797_I2C_FIFO_STAT			0x030
#define MT6797_I2C_FIFO_ADDR_CLR		0x038
#define MT6797_I2C_IO_CONFIG			0x040
#define MT6797_I2C_HS				0x048
#define MT6797_I2C_SOFTRESET			0x050
#define MT6797_I2C_ACKERR			BIT(1)
#define MT6797_I2C_HS_NACKERR			BIT(2)
#define MT6797_I2C_TRANSAC_COMP			BIT(0)
#define MT6797_I2C_CONTROL_RS			BIT(1)
#define MT6797_I2C_CONTROL_CLK_EXT_EN		BIT(3)
#define MT6797_I2C_CONTROL_DIR_CHANGE		BIT(4)
#define MT6797_I2C_CONTROL_ACKERR_DET_EN	BIT(5)

static void mt6797_smp_init(void)
{
	static bool initialized;
	void __iomem *mcucfg;
	void __iomem *cci;
	struct arm_smccc_res res;
	u32 axi_config;
	u32 snoop_control;
	unsigned int timeout = 100000;

	if (initialized || !of_machine_is_compatible("mediatek,mt6797"))
		return;

	mcucfg = ioremap(MT6797_MCUSYS_MP0_AXI_CONFIG & PAGE_MASK,
			 PAGE_SIZE);
	cci = ioremap(MT6797_CCI400_BASE, 0x10000);
	if (!mcucfg || !cci) {
		pr_err("MT6797 SMP: unable to map MCUSYS/CCI registers\n");
		goto out;
	}

	/* This is mt_smp_init_cpus() from the vendor kernel. */
	axi_config = readl_relaxed(mcucfg +
			(MT6797_MCUSYS_MP0_AXI_CONFIG & ~PAGE_MASK));
	arm_smccc_smc(MTK_SIP_KERNEL_MCUSYS_WRITE,
		      MT6797_MCUSYS_MP0_AXI_CONFIG,
		      axi_config & ~MT6797_ACINACTM,
		      0, 0, 0, 0, 0, &res);
	if ((long)res.a0 < 0)
		pr_warn("MT6797 SMP: MCUSYS_WRITE returned %ld\n",
			(long)res.a0);

	snoop_control = readl_relaxed(cci + MT6797_CCI400_SI4_SNOOP_CONTROL);
	writel_relaxed(snoop_control | MT6797_CCI_REQ,
		       cci + MT6797_CCI400_SI4_SNOOP_CONTROL);
	while ((readl_relaxed(cci + MT6797_CCI400_STATUS) & 1) && timeout--)
		cpu_relax();
	if (!timeout)
		pr_warn("MT6797 SMP: CCI400 change did not settle\n");

	pr_info("MT6797 SMP: MP0_AXI_CONFIG 0x%08x -> 0x%08x, CCI SI4 0x%08x\n",
		axi_config, axi_config & ~MT6797_ACINACTM,
		readl_relaxed(cci + MT6797_CCI400_SI4_SNOOP_CONTROL));

	iounmap(cci);
	iounmap(mcucfg);
	initialized = true;
	return;

out:
	if (cci)
		iounmap(cci);
	if (mcucfg)
		iounmap(mcucfg);
}

static int mt6797_i2c6_transfer(void __iomem *base, u8 addr,
				const u8 *tx, u8 tx_len, u8 *rx, u8 rx_len)
{
	u16 control;
	u16 status;
	unsigned int i;

	if ((!tx_len && !rx_len) || tx_len > 8 || rx_len > 8)
		return -EINVAL;

	/* Match the vendor I2C6 high-speed, push-pull FIFO transaction. */
	control = MT6797_I2C_CONTROL_RS |
		  MT6797_I2C_CONTROL_CLK_EXT_EN |
		  MT6797_I2C_CONTROL_ACKERR_DET_EN;
	if (tx_len && rx_len)
		control |= MT6797_I2C_CONTROL_DIR_CHANGE;

	writew(control, base + MT6797_I2C_CONTROL);
	writew(0x10, base + MT6797_I2C_TIMING);
	writew(0x0201, base + MT6797_I2C_HS);
	writew(0x0000, base + MT6797_I2C_IO_CONFIG);
	writew((addr << 1) | (tx_len ? 0 : 1),
	       base + MT6797_I2C_SLAVE_ADDR);
	writew(readw(base + MT6797_I2C_INTR_MASK) &
	       ~(MT6797_I2C_HS_NACKERR | MT6797_I2C_ACKERR |
		 MT6797_I2C_TRANSAC_COMP),
	       base + MT6797_I2C_INTR_MASK);
	writew(MT6797_I2C_HS_NACKERR | MT6797_I2C_ACKERR |
	       MT6797_I2C_TRANSAC_COMP, base + MT6797_I2C_INTR_STAT);
	writew(0x0001, base + MT6797_I2C_FIFO_ADDR_CLR);
	writew((tx_len & 0xff) | ((rx_len << 8) & 0x1f00),
	       base + MT6797_I2C_TRANSFER_LEN);
	writew((tx_len && rx_len) ? 2 : 1, base + MT6797_I2C_TRANSAC_LEN);

	while (tx_len--)
		writew(*tx++, base + MT6797_I2C_DATA_PORT);

	/* All controller registers must be ready before START. */
	mb();
	writew(0x0001, base + MT6797_I2C_START);

	for (i = 0; i < 100000; i++) {
		status = readw(base + MT6797_I2C_INTR_STAT);
		if (status & (MT6797_I2C_HS_NACKERR | MT6797_I2C_ACKERR))
			break;
		if (status & MT6797_I2C_TRANSAC_COMP)
			goto complete;
		cpu_relax();
	}

	writew(0x0001, base + MT6797_I2C_SOFTRESET);
	writew(MT6797_I2C_HS_NACKERR | MT6797_I2C_ACKERR |
	       MT6797_I2C_TRANSAC_COMP, base + MT6797_I2C_INTR_STAT);
	writew(0x0001, base + MT6797_I2C_FIFO_ADDR_CLR);
	return i == 100000 ? -ETIMEDOUT : -EIO;

complete:
	if (rx_len) {
		if (((readw(base + MT6797_I2C_FIFO_STAT) >> 4) & 0xf) < rx_len)
			return -EIO;
		while (rx_len--)
			*rx++ = readw(base + MT6797_I2C_DATA_PORT) & 0xff;
	}

	return 0;
}

static int mt6797_da9214_read(void __iomem *i2c, u8 reg, u8 *value)
{
	return mt6797_i2c6_transfer(i2c, MT6797_DA9214_ADDR,
				    &reg, 1, value, 1);
}

static int mt6797_da9214_write(void __iomem *i2c, u8 reg, u8 value)
{
	u8 data[2] = { reg, value };

	return mt6797_i2c6_transfer(i2c, MT6797_DA9214_ADDR,
				    data, ARRAY_SIZE(data), NULL, 0);
}

static int mt6797_da9214_enable_buck(void)
{
	void __iomem *i2c;
	u8 page;
	u8 buckb;
	int ret;

	i2c = ioremap(MT6797_I2C6_BASE, PAGE_SIZE);
	if (!i2c)
		return -ENOMEM;

	ret = mt6797_da9214_read(i2c, 0x00, &page);
	if (ret)
		goto out;
	ret = mt6797_da9214_write(i2c, 0x00, page & ~0x0f);
	if (ret)
		goto out;

	ret = mt6797_da9214_read(i2c, 0x5e, &buckb);
	if (ret)
		goto out;
	ret = mt6797_da9214_write(i2c, 0x5e, buckb | BIT(0));
	if (!ret)
		pr_info("MT6797 SMP: DA9214 buck B enabled (0x5e=0x%02x)\n",
			(unsigned int)(buckb | BIT(0)));

out:
	iounmap(i2c);
	return ret;
}

static int mt6797_big_cluster_prepare(void)
{
	static bool prepared;
	void __iomem *spm;
	void __iomem *wdt;
	void __iomem *idvfs;
	struct arm_smccc_res res;
	u32 tmp;

	if (prepared)
		return 0;

	spm = ioremap(MT6797_SPM_BASE_ADDR, PAGE_SIZE);
	wdt = ioremap(MT6797_WDT_BASE_ADDR, PAGE_SIZE);
	idvfs = ioremap(MT6797_IDVFS_BASE_ADDR, PAGE_SIZE);
	if (!spm || !wdt || !idvfs) {
		pr_err("MT6797 SMP: unable to map MP2 power registers\n");
		if (idvfs)
			iounmap(idvfs);
		if (wdt)
			iounmap(wdt);
		if (spm)
			iounmap(spm);
		return -ENOMEM;
	}

	/* Matches vendor cpu_power_on_buck(), before PSCI CPU_ON. */
	writel(readl(spm + MT6797_SPM_MP2_CPUSYS_PWR_CON) | BIT(0),
	       spm + MT6797_SPM_MP2_CPUSYS_PWR_CON);

	/* The vendor sequence deliberately performs this read as a barrier. */
	(void)readl(idvfs + MT6797_IDVFS_MP2_PLL_CON0);

	tmp = readl(wdt + MT6797_WDT_SWSYSRST);
	writel(tmp | MT6797_WDT_SWSYS_RST_KEY |
	       MT6797_WDT_SWSYS_RST_PWRAP_SPI_CTL_RST,
	       wdt + MT6797_WDT_SWSYSRST);

	/* Vendor cpu_power_on_buck(hotplug=1) enables the external vproc buck. */
	if (mt6797_da9214_enable_buck()) {
		pr_err("MT6797 SMP: DA9214 buck B enable failed\n");
		iounmap(idvfs);
		iounmap(wdt);
		iounmap(spm);
		return -EIO;
	}
	udelay(1000);

	writel(readl(spm + MT6797_SPM_CPU_EXT_BUCK_ISO) & ~0x3U,
	       spm + MT6797_SPM_CPU_EXT_BUCK_ISO);

	tmp = readl(wdt + MT6797_WDT_SWSYSRST);
	writel((tmp & ~MT6797_WDT_SWSYS_RST_PWRAP_SPI_CTL_RST) |
	       MT6797_WDT_SWSYS_RST_KEY,
	       wdt + MT6797_WDT_SWSYSRST);

	udelay(240);
	arm_smccc_smc(MTK_SIP_KERNEL_IDVFS_BIGIDVFSSRAMLDOSET,
		      110000, 0, 0, 0, 0, 0, 0, &res);
	if ((long)res.a0 < 0)
		pr_warn("MT6797 SMP: MP2 SRAM LDO SMC returned %ld\n",
			(long)res.a0);
	udelay(240);

	iounmap(idvfs);
	iounmap(wdt);
	iounmap(spm);
	prepared = true;
	pr_info("MT6797 SMP: MP2 power preconditioning complete\n");

	return 0;
}
#endif

static int cpu_psci_cpu_boot(unsigned int cpu)
{
#ifdef CONFIG_ARCH_MEDIATEK
	mt6797_smp_init();
	if (of_machine_is_compatible("mediatek,mt6797") && cpu >= 8) {
		int prep_err = mt6797_big_cluster_prepare();

		if (prep_err)
			return prep_err;
	}
#endif
	phys_addr_t pa_secondary_entry = __pa_symbol(secondary_entry);
	int err = psci_ops.cpu_on(cpu_logical_map(cpu), pa_secondary_entry);
	if (err && err != -EPERM)
		pr_err("failed to boot CPU%d (%d)\n", cpu, err);

	return err;
}

#ifdef CONFIG_HOTPLUG_CPU
static bool cpu_psci_cpu_can_disable(unsigned int cpu)
{
	return !psci_tos_resident_on(cpu);
}

static int cpu_psci_cpu_disable(unsigned int cpu)
{
	/* Fail early if we don't have CPU_OFF support */
	if (!psci_ops.cpu_off)
		return -EOPNOTSUPP;

	/* Trusted OS will deny CPU_OFF */
	if (psci_tos_resident_on(cpu))
		return -EPERM;

	return 0;
}

static void cpu_psci_cpu_die(unsigned int cpu)
{
	/*
	 * There are no known implementations of PSCI actually using the
	 * power state field, pass a sensible default for now.
	 */
	u32 state = PSCI_POWER_STATE_TYPE_POWER_DOWN <<
		    PSCI_0_2_POWER_STATE_TYPE_SHIFT;

	psci_ops.cpu_off(state);
}

static int cpu_psci_cpu_kill(unsigned int cpu)
{
	int err;
	unsigned long start, end;

	if (!psci_ops.affinity_info)
		return 0;
	/*
	 * cpu_kill could race with cpu_die and we can
	 * potentially end up declaring this cpu undead
	 * while it is dying. So, try again a few times.
	 */

	start = jiffies;
	end = start + msecs_to_jiffies(100);
	do {
		err = psci_ops.affinity_info(cpu_logical_map(cpu), 0);
		if (err == PSCI_0_2_AFFINITY_LEVEL_OFF) {
			pr_info("CPU%d killed (polled %d ms)\n", cpu,
				jiffies_to_msecs(jiffies - start));
			return 0;
		}

		usleep_range(100, 1000);
	} while (time_before(jiffies, end));

	pr_warn("CPU%d may not have shut down cleanly (AFFINITY_INFO reports %d)\n",
			cpu, err);
	return -ETIMEDOUT;
}
#endif

const struct cpu_operations cpu_psci_ops = {
	.name		= "psci",
	.cpu_init	= cpu_psci_cpu_init,
	.cpu_prepare	= cpu_psci_cpu_prepare,
	.cpu_boot	= cpu_psci_cpu_boot,
#ifdef CONFIG_HOTPLUG_CPU
	.cpu_can_disable = cpu_psci_cpu_can_disable,
	.cpu_disable	= cpu_psci_cpu_disable,
	.cpu_die	= cpu_psci_cpu_die,
	.cpu_kill	= cpu_psci_cpu_kill,
#endif
};
