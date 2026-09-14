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
#endif

static int cpu_psci_cpu_boot(unsigned int cpu)
{
#ifdef CONFIG_ARCH_MEDIATEK
	mt6797_smp_init();
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
