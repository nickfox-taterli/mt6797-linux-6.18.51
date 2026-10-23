// SPDX-License-Identifier: GPL-2.0-only
/* MT6797 BSP EEM detector phases. Voltage recommendations are advisory only. */
#include <linux/cpu.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/nvmem-consumer.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_opp.h>
#include <linux/soc/mediatek/mt6797-cpufreq.h>
#include <linux/soc/mediatek/mt6797-thermal-eem.h>
#include <linux/unaligned.h>
#include <linux/workqueue.h>

#define EEM_ENABLE 0x238
#define EEM_STATUS 0x254
#define EEM_RAW_STATUS 0x258
#define EEM_IRQ_ENABLE 0x25c
#define EEM_BANK 0x400
#define EEM_ERROR_MASK 0x5f00

static const u32 saved_regs[] = {
	0x200, 0x204, 0x208, 0x20c, 0x210, 0x214, 0x218, 0x21c, 0x220,
	0x224, 0x228, 0x22c, 0x230, 0x234, 0x23c, 0x25c, 0x264,
};

struct eem_bank {
	u32 bank, cpu, max_khz, fuse, age_fuse;
	u32 des, tempchar, detchar, agechar, mts, bts, limits, vboot;
	u32 percent[8], dc, age, vop30, vop74, status;
	u32 saved[ARRAY_SIZE(saved_regs)];
	bool overflow, saved_valid;
};

struct mt6797_eem {
	struct device *dev, *thermal, *cpufreq;
	void __iomem *base;
	struct mutex lock;
	struct delayed_work monitor;
	struct eem_bank banks[4];
	u32 ate;
	bool calibrated, monitoring, resume_monitor;
	u64 samples, failures, runs;
	int last_error;
};

static int eem_select(struct mt6797_eem *s, u32 bank,
                      struct mt6797_thermal_bank_cookie *cookie)
{
	return mt6797_thermal_bank_begin(s->thermal, bank, cookie);
}

static int eem_put(struct mt6797_eem *s,
                   struct mt6797_thermal_bank_cookie *cookie)
{
	return mt6797_thermal_bank_end(s->thermal, cookie);
}

static int eem_disable(struct mt6797_eem *s)
{
	struct mt6797_thermal_bank_cookie cookie;
	int ret, error = 0;
	unsigned int i, j;

	for (i = 0; i < ARRAY_SIZE(s->banks); i++) {
		struct eem_bank *b = &s->banks[i];

		if (!b->saved_valid)
			continue;
		ret = eem_select(s, b->bank, &cookie);
		if (ret) {
			s->failures++;
			s->last_error = ret;
			error = error ?: ret;
			continue;
		}
		writel(0, s->base + EEM_IRQ_ENABLE);
		writel(0, s->base + EEM_ENABLE);
		writel(0xffffffff, s->base + EEM_STATUS);
		for (j = 0; j < ARRAY_SIZE(saved_regs); j++)
			writel(b->saved[j], s->base + saved_regs[j]);
		ret = eem_put(s, &cookie);
		if (ret) {
			s->failures++;
			s->last_error = ret;
			error = error ?: ret;
		}
		b->saved_valid = false;
	}
	s->monitoring = false;
	return error;
}

static int eem_save(struct mt6797_eem *s)
{
	unsigned int i, j;
	struct mt6797_thermal_bank_cookie cookie;
	int ret;

	for (i = 0; i < ARRAY_SIZE(s->banks); i++) {
		struct eem_bank *b = &s->banks[i];

		ret = eem_select(s, b->bank, &cookie);
		if (ret)
			return ret;
		if (readl(s->base + EEM_ENABLE)) {
			ret = eem_put(s, &cookie);
			if (ret)
				return ret;
			ret = eem_disable(s);
			return ret ?: -EBUSY;
		}
		for (j = 0; j < ARRAY_SIZE(saved_regs); j++)
			b->saved[j] = readl(s->base + saved_regs[j]);
		b->saved_valid = true;
		ret = eem_put(s, &cookie);
		if (ret)
			return ret;
	}
	return 0;
}

static int eem_phase(struct mt6797_eem *s, struct eem_bank *b,
		      unsigned int phase)
{
	struct mt6797_thermal_bank_cookie cookie;
	int ret;
	u32 p30 = 0, p74 = 0;
	unsigned int i;

	ret = eem_select(s, b->bank, &cookie);
	if (ret)
		return ret;
	for (i = 0; i < 4; i++) {
		p30 |= b->percent[i] << (i * 8);
		p74 |= b->percent[i + 4] << (i * 8);
	}
	writel(0, s->base + EEM_ENABLE);
	writel(b->des, s->base + 0x200);
	writel(b->tempchar, s->base + 0x204);
	writel(b->detchar, s->base + 0x208);
	writel(b->agechar, s->base + 0x20c);
	writel(0x555555, s->base + 0x210);
	writel(0x555555, s->base + 0x214);
	writel(p30, s->base + 0x218);
	writel(p74, s->base + 0x21c);
	writel(b->limits, s->base + 0x220);
	writel(b->vboot, s->base + 0x224);
	writel(0x514, s->base + 0x228);
	writel(0xffff, s->base + 0x22c);
	writel((b->bts << 12) | b->mts, s->base + 0x230);
	writel(0x80000000, s->base + 0x234);
	if (s->ate > 5 || b->overflow)
		writel((readl(s->base + 0x264) & ~15) | 7, s->base + 0x264);
	writel((b->age << 16) | b->dc, s->base + 0x23c);
	writel(0xffffffff, s->base + EEM_STATUS);
	/* Poll RAW status; no unhandled interrupt on the shared thermal block. */
	writel(0, s->base + EEM_IRQ_ENABLE);
	writel(phase, s->base + EEM_ENABLE);
	return eem_put(s, &cookie);
}

static int eem_wait_phase(struct mt6797_eem *s, struct eem_bank *b)
{
	struct mt6797_thermal_bank_cookie cookie;
	int ret;
	unsigned int i;
	u32 status;

	for (i = 0; i < 100; i++) {
		ret = eem_select(s, b->bank, &cookie);
		if (ret)
			return ret;
		status = readl(s->base + EEM_RAW_STATUS);
		b->status = status;
		if (status & (EEM_ERROR_MASK | BIT(0))) {
			b->vop30 = readl(s->base + 0x248);
			b->vop74 = readl(s->base + 0x24c);
			ret = eem_put(s, &cookie);
			return ret ?: (status & EEM_ERROR_MASK ? -EIO : 0);
		}
		ret = eem_put(s, &cookie);
		if (ret)
			return ret;
		usleep_range(50, 100);
	}
	return -ETIMEDOUT;
}

static int eem_measure(void *context)
{
	struct mt6797_eem *s = context;
	struct mt6797_thermal_bank_cookie cookie;
	unsigned int i;
	int ret;

	ret = eem_save(s);
	if (ret)
		return ret;
	/* All four INIT01 measurements finish before any INIT02/MON phase. */
	for (i = 0; i < ARRAY_SIZE(s->banks); i++) {
		ret = eem_phase(s, &s->banks[i], 1);
		if (ret)
			return ret;
	}
	for (i = 0; i < ARRAY_SIZE(s->banks); i++) {
		struct eem_bank *b = &s->banks[i];

		ret = eem_wait_phase(s, b);
		if (ret)
			return ret;
		ret = eem_select(s, b->bank, &cookie);
		if (ret)
			return ret;
		b->dc = -(readl(s->base + 0x240) & 0xffff) & 0xffff;
		b->age = readl(s->base + 0x244) & 0xffff;
		writel(0, s->base + EEM_ENABLE);
		writel(1, s->base + EEM_STATUS);
		ret = eem_put(s, &cookie);
		if (ret)
			return ret;
	}
	for (i = 0; i < ARRAY_SIZE(s->banks); i++) {
		ret = eem_phase(s, &s->banks[i], 5);
		if (ret)
			return ret;
		ret = eem_wait_phase(s, &s->banks[i]);
		if (ret)
			return ret;
	}
	for (i = 0; i < ARRAY_SIZE(s->banks); i++) {
		ret = eem_phase(s, &s->banks[i], 2);
		if (ret)
			return ret;
	}
	s->calibrated = true;
	s->monitoring = true;
	return 0;
}

static void eem_monitor(struct work_struct *work)
{
	struct mt6797_eem *s =
		container_of(to_delayed_work(work), struct mt6797_eem, monitor);
	struct mt6797_thermal_bank_cookie cookie;
	int ret;
	unsigned int i;

	mutex_lock(&s->lock);
	if (!s->monitoring)
		goto unlock;
	for (i = 0; i < ARRAY_SIZE(s->banks); i++) {
		struct eem_bank *b = &s->banks[i];

		ret = eem_select(s, b->bank, &cookie);
		if (ret)
			goto failed;
		b->status = readl(s->base + EEM_RAW_STATUS);
		b->vop30 = readl(s->base + 0x248);
		b->vop74 = readl(s->base + 0x24c);
		if ((b->status & EEM_ERROR_MASK) ||
		    readl(s->base + EEM_ENABLE) != 2)
			s->failures++;
		writel(b->status, s->base + EEM_STATUS);
		ret = eem_put(s, &cookie);
		if (ret)
			goto failed;
	}
	s->samples++;
	schedule_delayed_work(&s->monitor, HZ);
	goto unlock;
failed:
	s->failures++;
	s->last_error = ret;
	s->monitoring = false;
unlock:
	mutex_unlock(&s->lock);
}

static ssize_t state_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct mt6797_eem *s = dev_get_drvdata(dev);
	ssize_t len;
	unsigned int i;

	mutex_lock(&s->lock);
	len = sysfs_emit(
		buf,
		"calibrated=%u monitoring=%u runs=%llu samples=%llu failures=%llu last_error=%d ate=%u voltage_owner=manual_opp automatic_voltage=0\n",
		s->calibrated, s->monitoring, s->runs, s->samples, s->failures,
		s->last_error, s->ate);
	for (i = 0; i < ARRAY_SIZE(s->banks); i++) {
		struct eem_bank *b = &s->banks[i];

		len += sysfs_emit_at(
			buf, len,
			"bank%u dc=%04x age=%04x status=%08x vop30=%08x vop74=%08x MTS=%u BTS=%u percent=%u,%u,%u,%u,%u,%u,%u,%u\n",
			b->bank, b->dc, b->age, b->status, b->vop30, b->vop74,
			b->mts, b->bts, b->percent[0], b->percent[1],
			b->percent[2], b->percent[3], b->percent[4],
			b->percent[5], b->percent[6], b->percent[7]);
	}
	mutex_unlock(&s->lock);
	return len;
}
static DEVICE_ATTR_RO(state);

static ssize_t calibrate_store(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t count)
{
	struct mt6797_eem *s = dev_get_drvdata(dev);
	int ret = 0;

	if (!sysfs_streq(buf, "0") && !sysfs_streq(buf, "1"))
		return -EINVAL;
	cancel_delayed_work_sync(&s->monitor);
	mutex_lock(&s->lock);
	ret = eem_disable(s);
	if (ret)
		goto unlock;
	if (sysfs_streq(buf, "1")) {
		s->calibrated = false;
		ret = mt6797_cpufreq_eem_measure(s->cpufreq, eem_measure, s);
		s->runs++;
		s->last_error = ret;
		if (ret) {
			s->failures++;
			eem_disable(s);
		} else {
			schedule_delayed_work(&s->monitor, HZ);
		}
	}
unlock:
	mutex_unlock(&s->lock);
	return ret ? ret : count;
}
static DEVICE_ATTR_WO(calibrate);

static struct attribute *eem_attrs[] = {
	&dev_attr_state.attr,
	&dev_attr_calibrate.attr,
	NULL,
};
ATTRIBUTE_GROUPS(eem);

static void eem_iounmap(void *base)
{
	iounmap(base);
}

static void eem_put_device(void *dev)
{
	put_device(dev);
}

static struct device *eem_provider(struct device *dev, const char *name)
{
	struct device_node *np = of_parse_phandle(dev->of_node, name, 0);
	struct platform_device *p = np ? of_find_device_by_node(np) : NULL;
	int ret;

	of_node_put(np);
	if (!p)
		return ERR_PTR(-EPROBE_DEFER);
	ret = devm_add_action_or_reset(dev, eem_put_device, &p->dev);
	if (ret)
		return ERR_PTR(ret);
	if (!device_is_bound(&p->dev) || !dev_get_drvdata(&p->dev))
		return ERR_PTR(-EPROBE_DEFER);
	if (!device_link_add(dev, &p->dev, DL_FLAG_AUTOREMOVE_CONSUMER))
		return ERR_PTR(-ENOMEM);
	return &p->dev;
}

static int eem_frequencies(struct eem_bank *b)
{
	struct device *cpu = get_cpu_device(b->cpu);
	unsigned long freq = ULONG_MAX;
	struct dev_pm_opp *opp;
	unsigned int count = 0, frequencies[64], i;

	if (b->bank == 5) {
		/* BSP FY CCI table, samples 0,2,...14, reference max=1118MHz. */
		static const unsigned int khz[] = { 923000, 858000, 754000,
						    637000, 533000, 416000,
						    325000, 234000 };

		for (i = 0; i < 8; i++)
			b->percent[i] = DIV_ROUND_UP(khz[i] * 100, b->max_khz);
		return 0;
	}
	while (count < ARRAY_SIZE(frequencies)) {
		opp = dev_pm_opp_find_freq_floor(cpu, &freq);
		if (IS_ERR(opp))
			break;
		frequencies[count++] = freq / 1000;
		dev_pm_opp_put(opp);
		if (!freq)
			break;
		freq--;
	}
	if (count < 8 || count == ARRAY_SIZE(frequencies) ||
	    frequencies[0] > b->max_khz)
		return -EINVAL;
	for (i = 0; i < 8; i++)
		b->percent[i] = DIV_ROUND_UP(frequencies[i * count / 8] * 100,
					     b->max_khz);
	return 0;
}

static int eem_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mt6797_eem *s;
	struct nvmem_cell *cell;
	struct device_node *np;
	size_t len;
	void *fuses;
	unsigned int i;
	int ret;

	s = devm_kzalloc(dev, sizeof(*s), GFP_KERNEL);
	if (!s)
		return -ENOMEM;
	s->dev = dev;
	mutex_init(&s->lock);
	INIT_DELAYED_WORK(&s->monitor, eem_monitor);
	s->thermal = eem_provider(dev, "mediatek,thermal");
	if (IS_ERR(s->thermal))
		return PTR_ERR(s->thermal);
	s->cpufreq = eem_provider(dev, "mediatek,cpufreq");
	if (IS_ERR(s->cpufreq))
		return PTR_ERR(s->cpufreq);
	np = of_parse_phandle(dev->of_node, "mediatek,thermal", 0);
	s->base = of_iomap(np, 0);
	of_node_put(np);
	if (!s->base)
		return -ENOMEM;
	ret = devm_add_action_or_reset(dev, eem_iounmap, s->base);
	if (ret)
		return ret;
	cell = devm_nvmem_cell_get(dev, "calibration-data");
	if (IS_ERR(cell))
		return PTR_ERR(cell);
	fuses = nvmem_cell_read(cell, &len);
	if (IS_ERR(fuses))
		return PTR_ERR(fuses);
	if (len != 0x44) {
		kfree(fuses);
		return -EINVAL;
	}
	s->ate = (get_unaligned_le32(fuses + 0x2c) >> 4) & 15;
	s->banks[0] =
		(struct eem_bank){ .bank = 0, .cpu = 8, .max_khz = 2496000 };
	s->banks[1] =
		(struct eem_bank){ .bank = 3, .cpu = 4, .max_khz = 2249000 };
	s->banks[2] =
		(struct eem_bank){ .bank = 4, .cpu = 0, .max_khz = 1599000 };
	s->banks[3] = (struct eem_bank){ .bank = 5, .max_khz = 1118000 };
	for (i = 0; i < ARRAY_SIZE(s->banks); i++) {
		struct eem_bank *b = &s->banks[i];
		const u32 fuse_offsets[] = { 0x40, 0x18, 0x20, 0x28 };
		const u32 age_offsets[] = { 0x3c, 0x1c, 0x24, 0x2c };
		u32 fuse = get_unaligned_le32(fuses + fuse_offsets[i]);
		u32 age = get_unaligned_le32(fuses + age_offsets[i]);
		u32 enable = get_unaligned_le32(fuses + (i == 0 ? 4 :
							 i == 1 ? 0x1c :
								  0x24));
		unsigned int dc = (fuse >> 16) & 255;

		if (i == 3)
			enable >>= 8;
		if (!fuse || (enable & 3) != 3) {
			ret = -ENODEV;
			break;
		}
		b->fuse = fuse;
		b->age_fuse = age;
		if (s->ate <= 5 && i && dc < 128) {
			dc = (u8)(((int)dc - 256) / 2);
			b->overflow = true;
		}
		b->des = ((fuse & 255) << 8) | ((fuse >> 8) & 255);
		b->detchar = (dc << 8) | (fuse >> 24);
		b->agechar = i ? ((age >> 24) << 8) : ((age >> 8) & 255) << 8;
		b->tempchar = ((i ? (s->ate > 5 ? 4 : 12) :
				    (s->ate > 5 ? 0x32 : 0x35))
			       << 16) |
			      ((i ? ((age >> 16) & 255) : (age & 255)) << 8) |
			      (i ? 7 : 4);
		b->vboot = i ? 48 : 70;
		b->limits = i ? (80U << 24) | (13 << 16) | 0x1fe :
				(90U << 24) | (50 << 16) | 0x1fe;
		ret = mt6797_thermal_eem_calibration(s->thermal, b->bank,
						     &b->mts, &b->bts);
		if (!ret)
			ret = eem_frequencies(b);
		if (ret)
			break;
	}
	kfree(fuses);
	if (ret)
		return ret;
	platform_set_drvdata(pdev, s);
	dev_info(
		dev,
		"BSP INIT01/INIT02/MON available on explicit low-frequency calibration; manual voltage ownership\n");
	return 0;
}

static void eem_remove(struct platform_device *pdev)
{
	struct mt6797_eem *s = platform_get_drvdata(pdev);

	cancel_delayed_work_sync(&s->monitor);
	mutex_lock(&s->lock);
	eem_disable(s);
	mutex_unlock(&s->lock);
}

static int eem_suspend(struct device *dev)
{
	struct mt6797_eem *s = dev_get_drvdata(dev);
	int ret;

	cancel_delayed_work_sync(&s->monitor);
	mutex_lock(&s->lock);
	s->resume_monitor = s->monitoring;
	ret = eem_disable(s);
	mutex_unlock(&s->lock);
	return ret;
}

static int eem_resume(struct device *dev)
{
	struct mt6797_eem *s = dev_get_drvdata(dev);
	unsigned int i;
	int ret = 0;

	mutex_lock(&s->lock);
	if (!s->resume_monitor)
		goto unlock;
	ret = eem_save(s);
	for (i = 0; !ret && i < ARRAY_SIZE(s->banks); i++) {
		ret = eem_phase(s, &s->banks[i], 5);
		if (!ret)
			ret = eem_wait_phase(s, &s->banks[i]);
	}
	if (ret) {
		s->last_error = ret;
		s->failures++;
		eem_disable(s);
	} else {
		for (i = 0; i < ARRAY_SIZE(s->banks); i++) {
			ret = eem_phase(s, &s->banks[i], 2);
			if (ret) {
				s->last_error = ret;
				s->failures++;
				eem_disable(s);
				goto unlock;
			}
		}
		s->monitoring = true;
		schedule_delayed_work(&s->monitor, HZ);
	}
unlock:
	mutex_unlock(&s->lock);
	return ret;
}
static DEFINE_SIMPLE_DEV_PM_OPS(eem_pm_ops, eem_suspend, eem_resume);

static const struct of_device_id eem_of_match[] = {
	{ .compatible = "mediatek,mt6797-eem" },
	{},
};
MODULE_DEVICE_TABLE(of, eem_of_match);

static struct platform_driver eem_driver = {
	.probe = eem_probe,
	.remove = eem_remove,
	.shutdown = eem_remove,
	.driver = {
		.name = "mt6797-eem",
		.of_match_table = eem_of_match,
		.dev_groups = eem_groups,
		.pm = pm_sleep_ptr(&eem_pm_ops),
	},
};
module_platform_driver(eem_driver);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(
	"MT6797 EEM hardware detector phases with advisory-only voltage output");
