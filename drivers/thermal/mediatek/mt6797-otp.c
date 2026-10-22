// SPDX-License-Identifier: GPL-2.0-only
/* BSP mt_otp PID, using CPUfreq QoS to retain the user's manual voltage OPPs.
 * The autonomous iDVFS voltage controller is deliberately not enabled.
 */
#include <linux/arm-smccc.h>
#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/hrtimer.h>
#include <linux/module.h>
#include <linux/math64.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_qos.h>
#include <linux/soc/mediatek/mt6797-thermal-eem.h>
#include <linux/workqueue.h>

#define OTP_BASE 0x10222440
#define OTP_PERIOD_NS 5000000
#define OTP_MAX_KHZ 2405000
#define OTP_MIN_KHZ 338000

struct mt6797_otp {
	struct device *dev, *thermal;
	struct cpufreq_policy *policy;
	struct freq_qos_request qos;
	struct mutex lock;
	struct hrtimer timer;
	struct work_struct work;
	u32 saved[12], mts, bts, score, frequency;
	unsigned int target, limit;
	u64 samples, failures, max_interval_ns, last_ns, recovery_ns;
	bool enabled, active, programmed;
};

static u32 otp_read(unsigned int offset)
{
	struct arm_smccc_res res;

	arm_smccc_smc(0xc200035f, OTP_BASE + offset, 0, 0, 0, 0, 0, 0, &res);
	return res.a0;
}

static int otp_write(unsigned int offset, u32 value)
{
	struct arm_smccc_res res;

	arm_smccc_smc(0xc200035e, OTP_BASE + offset, value, 0, 0, 0, 0, 0, &res);
	return res.a0 ? -EIO : 0;
}

static int otp_program(struct mt6797_otp *s)
{
	static const u32 settings[] = {
		0, 0x20000, 0xfffff060, 0x40003, 0xff9c, 0xfffe,
		0, 0x27fff, 2, 0xd0000,
	};
	s64 numerator = (s->bts << 4) - ((s->target - 25) << 6);
	u32 adc;
	unsigned int i;
	int ret;

	if (numerator <= 0 || numerator >= 65536 || !s->mts)
		return -ERANGE;
	adc = div_s64(numerator << 6, s->mts);
	if (adc > 4095)
		return -ERANGE;
	ret = otp_write(0, 0);
	for (i = 1; !ret && i < ARRAY_SIZE(settings); i++)
		ret = otp_write(i * 4, settings[i]);
	/* Polling retains BSP 5ms policy without enabling an unhandled IRQ. */
	if (!ret)
		ret = otp_write(0, (adc << 16) | (3 << 4) | BIT(2));
	if (!ret)
		ret = otp_write(0, otp_read(0) | BIT(0));
	if (!ret && (otp_read(0) & (GENMASK(27, 16) | BIT(0))) !=
		    ((adc << 16) | BIT(0)))
		ret = -EIO;
	s->programmed = !ret;
	return ret;
}

static void otp_work(struct work_struct *work)
{
	struct mt6797_otp *s = container_of(work, struct mt6797_otp, work);
	u64 now = ktime_get_ns();
	u32 mux, percent;
	unsigned int limit = OTP_MAX_KHZ;
	int temp, ret = 0;

	cpus_read_lock();
	mutex_lock(&s->lock);
	if (!s->enabled)
		goto unlock;
	if (s->last_ns)
		s->max_interval_ns = max(s->max_interval_ns, now - s->last_ns);
	s->last_ns = now;
	if (!cpu_online(8) && !cpu_online(9)) {
		s->programmed = false;
		goto unlock;
	}
	if (!s->programmed || !(otp_read(0) & BIT(0)))
		ret = otp_program(s);
	if (!ret)
		ret = mt6797_thermal_bank_temp(s->thermal, 0, &temp);
	if (ret) {
		s->failures++;
		limit = OTP_MIN_KHZ;
		goto update;
	}
	mux = otp_read(0x2c);
	ret = otp_write(0x2c, (mux & ~GENMASK(24, 20)) | (5 << 20));
	s->score = otp_read(0x28) & 0xffffff;
	ret |= otp_write(0x2c, (mux & ~GENMASK(24, 20)) | (6 << 20));
	s->frequency = otp_read(0x28) & 0x7ffff;
	ret |= otp_write(0x2c, mux);
	s->samples++;
	if (ret) {
		s->failures++;
		limit = OTP_MIN_KHZ;
		goto update;
	}
	if (s->score && (s->score < 0x2cec || (s->score & BIT(23)))) {
		s->active = true;
		s->recovery_ns = 0;
	}
	if (s->active) {
		/* BSP get_freq_pct(): integer percent*100, 2500MHz reference. */
		percent = (s->frequency >> 12) * 100;
		for (unsigned int i = 0; i < 6; i++)
			if (s->frequency & BIT(11 - i))
				percent += 50 >> i;
		limit = clamp(percent * 250, OTP_MIN_KHZ, OTP_MAX_KHZ);
		if (s->score > 0x30d4 && !(s->score & BIT(23)) &&
		    temp < (s->target - 5) * 1000) {
			if (!s->recovery_ns)
				s->recovery_ns = now;
			if (now - s->recovery_ns >= 2000000000ULL) {
				s->active = false;
				limit = OTP_MAX_KHZ;
			}
		} else {
			s->recovery_ns = 0;
		}
	}
update:
	if (limit != s->limit) {
		ret = freq_qos_update_request(&s->qos, limit);
		if (ret >= 0)
			s->limit = limit;
		else
			s->failures++;
	}
unlock:
	mutex_unlock(&s->lock);
	cpus_read_unlock();
}

static enum hrtimer_restart otp_tick(struct hrtimer *timer)
{
	struct mt6797_otp *s = container_of(timer, struct mt6797_otp, timer);

	queue_work_on(0, system_highpri_wq, &s->work);
	hrtimer_forward_now(timer, ns_to_ktime(OTP_PERIOD_NS));
	return HRTIMER_RESTART;
}

static ssize_t target_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mt6797_otp *s = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%u\n", READ_ONCE(s->target));
}

static ssize_t target_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct mt6797_otp *s = dev_get_drvdata(dev);
	unsigned int target;

	if (kstrtouint(buf, 0, &target) || target < 50 || target > 150)
		return -EINVAL;
	mutex_lock(&s->lock);
	s->target = target;
	s->programmed = false;
	s->recovery_ns = 0;
	mutex_unlock(&s->lock);
	return count;
}
static DEVICE_ATTR_RW(target);

static ssize_t state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mt6797_otp *s = dev_get_drvdata(dev);
	ssize_t len;

	mutex_lock(&s->lock);
	len = sysfs_emit(buf, "enabled=%u programmed=%u target_C=%u mts=%u bts=%u active=%u limit_kHz=%u score=%06x freq=%05x samples=%llu failures=%llu max_interval_ns=%llu actuator=cpufreq_qos voltage_owner=manual_opp autonomous_idvfs=0\n",
			 s->enabled, s->programmed, s->target, s->mts, s->bts,
			 s->active, s->limit, s->score, s->frequency, s->samples,
			 s->failures, s->max_interval_ns);
	mutex_unlock(&s->lock);
	return len;
}
static DEVICE_ATTR_RO(state);

static struct attribute *otp_attrs[] = {
	&dev_attr_target.attr, &dev_attr_state.attr, NULL,
};
ATTRIBUTE_GROUPS(otp);

static void otp_stop(struct mt6797_otp *s)
{
	hrtimer_cancel(&s->timer);
	cancel_work_sync(&s->work);
	mutex_lock(&s->lock);
	s->enabled = false;
	/* Restoring only PID registers never enables the iDVFS actuator. */
	otp_write(0, 0);
	for (unsigned int i = 1; i < ARRAY_SIZE(s->saved); i++)
		if (i != 10)
			otp_write(i * 4, s->saved[i]);
	otp_write(0, s->saved[0] & ~BIT(0));
	s->programmed = false;
	mutex_unlock(&s->lock);
}

static void otp_put_thermal(void *dev)
{
	put_device(dev);
}

static int otp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct platform_device *provider;
	struct device_node *np;
	struct mt6797_otp *s;
	int ret;

	s = devm_kzalloc(dev, sizeof(*s), GFP_KERNEL);
	if (!s)
		return -ENOMEM;
	np = of_parse_phandle(dev->of_node, "mediatek,thermal", 0);
	provider = np ? of_find_device_by_node(np) : NULL;
	of_node_put(np);
	if (!provider)
		return -EPROBE_DEFER;
	s->thermal = &provider->dev;
	ret = devm_add_action_or_reset(dev, otp_put_thermal, s->thermal);
	if (ret)
		return ret;
	if (!device_is_bound(s->thermal) || !dev_get_drvdata(s->thermal))
		return -EPROBE_DEFER;
	if (!device_link_add(dev, s->thermal, DL_FLAG_AUTOREMOVE_CONSUMER))
		return -ENOMEM;
	ret = mt6797_thermal_otp_calibration(s->thermal, &s->mts, &s->bts);
	if (ret)
		return ret;
	s->policy = cpufreq_cpu_get(8);
	if (!s->policy)
		return -EPROBE_DEFER;
	/* The manual CPUfreq policy is the only voltage/frequency actuator. */
	ret = freq_qos_add_request(&s->policy->constraints, &s->qos,
				   FREQ_QOS_MAX, OTP_MAX_KHZ);
	if (ret < 0) {
		cpufreq_cpu_put(s->policy);
		return ret;
	}
	s->dev = dev;
	s->target = 150;
	s->limit = OTP_MAX_KHZ;
	mutex_init(&s->lock);
	INIT_WORK(&s->work, otp_work);
	hrtimer_setup(&s->timer, otp_tick, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	for (unsigned int i = 0; i < ARRAY_SIZE(s->saved); i++)
		if (i != 10)
			s->saved[i] = otp_read(i * 4);
	/* Never replace a different PID/iDVFS owner. */
	if (s->saved[0] & BIT(0)) {
		freq_qos_remove_request(&s->qos);
		cpufreq_cpu_put(s->policy);
		return -EBUSY;
	}
	platform_set_drvdata(pdev, s);
	s->enabled = true;
	hrtimer_start(&s->timer, ns_to_ktime(OTP_PERIOD_NS), HRTIMER_MODE_REL);
	dev_info(dev, "BSP PID target=150C MTS=%u BTS=%u; manual OPP QoS actuator\n",
		 s->mts, s->bts);
	return 0;
}

static void otp_remove(struct platform_device *pdev)
{
	struct mt6797_otp *s = platform_get_drvdata(pdev);

	otp_stop(s);
	freq_qos_remove_request(&s->qos);
	cpufreq_cpu_put(s->policy);
}

static void otp_shutdown(struct platform_device *pdev)
{
	otp_stop(platform_get_drvdata(pdev));
}

static int otp_suspend(struct device *dev)
{
	otp_stop(dev_get_drvdata(dev));
	return 0;
}

static int otp_resume(struct device *dev)
{
	struct mt6797_otp *s = dev_get_drvdata(dev);

	s->enabled = true;
	s->last_ns = 0;
	hrtimer_start(&s->timer, ns_to_ktime(OTP_PERIOD_NS), HRTIMER_MODE_REL);
	return 0;
}
static DEFINE_SIMPLE_DEV_PM_OPS(otp_pm_ops, otp_suspend, otp_resume);

static const struct of_device_id otp_of_match[] = {
	{ .compatible = "mediatek,mt6797-otp" }, {},
};
MODULE_DEVICE_TABLE(of, otp_of_match);

static struct platform_driver otp_driver = {
	.probe = otp_probe,
	.remove = otp_remove,
	.shutdown = otp_shutdown,
	.driver = {
		.name = "mt6797-otp",
		.of_match_table = otp_of_match,
		.dev_groups = otp_groups,
		.pm = pm_sleep_ptr(&otp_pm_ops),
	},
};
module_platform_driver(otp_driver);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MT6797 hardware thermal PID with manual OPP CPUfreq actuator");
