// SPDX-License-Identifier: GPL-2.0-only
/* Check the board firmware before the standard PSCI path releases CPUs. */
#define pr_fmt(fmt) "mt6797-firmware: " fmt
#include <linux/arm-smccc.h>
#include <linux/cpu.h>
#include <linux/firmware.h>
#include <linux/init.h>
#include <linux/kobject.h>
#include <linux/of.h>
#include <linux/soc/mediatek/mt6797-sip.h>
#include <linux/sysfs.h>

static struct arm_smccc_res boot_caps;

static int __init mt6797_firmware_check(void)
{
	unsigned int cpu;

	if (!of_machine_is_compatible("mediatek,mt6797"))
		return 0;
	arm_smccc_smc(MT6797_SIP_BOOT_CAPS, 0, 0, 0, 0, 0, 0, 0, &boot_caps);
	if (boot_caps.a0 == MT6797_BOOT_CAPS_MAGIC &&
	    boot_caps.a1 == MT6797_BOOT_CAPS_ABI &&
	    (boot_caps.a2 & MT6797_BOOT_CAPS_REQUIRED) == MT6797_BOOT_CAPS_REQUIRED &&
	    boot_caps.a3 == 1)
		return 0;
	/* No raw I2C fallback: an old BL31 cannot safely release MP2. Keep
	 * CPU0 and the recovery console usable, and require a firmware update.
	 */
	for_each_present_cpu(cpu)
		if (cpu != smp_processor_id())
			set_cpu_present(cpu, false);
	pr_crit("BL31 boot ABI mismatch: %lx/%lx/%lx/%lx; secondary CPUs disabled, update tee1\n",
		boot_caps.a0, boot_caps.a1, boot_caps.a2, boot_caps.a3);
	return -EPROTO;
}
early_initcall(mt6797_firmware_check);

static ssize_t boot_abi_show(struct kobject *kobj, struct kobj_attribute *attr,
			     char *buf)
{
	return sysfs_emit(buf, "magic=%lx abi=%lu capabilities=%lx ready=%lu\n",
			  boot_caps.a0, boot_caps.a1, boot_caps.a2, boot_caps.a3);
}
static struct kobj_attribute boot_abi_attr = __ATTR_RO(boot_abi);

static int __init mt6797_firmware_sysfs(void)
{
	struct kobject *kobj;
	int ret;

	if (!of_machine_is_compatible("mediatek,mt6797"))
		return 0;
	kobj = kobject_create_and_add("mt6797", firmware_kobj);
	if (!kobj)
		return -ENOMEM;
	ret = sysfs_create_file(kobj, &boot_abi_attr.attr);
	if (ret)
		kobject_put(kobj);
	return ret;
}
subsys_initcall(mt6797_firmware_sysfs);
