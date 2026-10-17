// SPDX-License-Identifier: GPL-2.0-only
//
// MT6797 STP/WMT diagnostic client.
//
// This is an explicit, root-only bring-up hook.  It never powers ConSys by
// itself and never creates a tty or HCI device.  The operator powers ConSys
// through its existing sysfs control, then requests one bounded WMT query or
// the vendor BTIF-full preamble.  A ConSys reservation prevents a concurrent
// power-off from invalidating the BTIF MMIO transaction.

#include <linux/device.h>
#include <linux/errno.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/unaligned.h>

#include <linux/mt6797-consys.h>
#include <linux/mt6797-stp-session.h>
#include <linux/mt6797-wlan-hif.h>
#include <linux/mt6797-wmt.h>

#define MT6797_STP_DIAG_TIMEOUT_MS	1000
#define MT6797_STP_DIAG_RESPONSE_MAX	32
#define MT6797_WMT_OPCODE_FUNC_CTRL	0x06
#define MT6797_WLAN_WCIR_CHIP_ID	0x0279

enum mt6797_stp_diag_mode {
	MT6797_STP_DIAG_QUERY,
	MT6797_STP_DIAG_FULL,
	MT6797_STP_DIAG_VERSION,
	MT6797_STP_DIAG_PATCH,
	MT6797_STP_DIAG_EMI_TEST,
	MT6797_STP_DIAG_BT_ON,
	MT6797_STP_DIAG_BT_OFF,
	MT6797_STP_DIAG_WIFI_ON,
	MT6797_STP_DIAG_WIFI_OFF,
	MT6797_STP_DIAG_WIFI_HIF,
};

struct mt6797_stp_diag {
	struct device *dev;
	struct device *consys_dev;
	struct mutex lock;
	u8 response[MT6797_STP_DIAG_RESPONSE_MAX];
	u16 response_length;
	int last_result;
	bool busy;
};

static void mt6797_stp_diag_rx(void *context, u8 task, const u8 *payload,
				       u16 length)
{
	struct mt6797_stp_diag *diag = context;

	dev_dbg(diag->dev, "STP RX task=%u length=%u\n", task, length);
}

static int mt6797_stp_diag_query(struct mt6797_stp_diag *diag,
				 struct mt6797_stp_session *session)
{
	static const u8 params[] = { 0x04 };
	static const u8 expected[] = { 0x00, 0x04, 0x11, 0x00, 0x00, 0x00 };
	u16 response_length = 0;
	int ret;

	ret = mt6797_stp_session_wmt(session, 0x04, params, sizeof(params),
					 diag->response, &response_length,
					 sizeof(diag->response),
					 msecs_to_jiffies(MT6797_STP_DIAG_TIMEOUT_MS));
	diag->response_length = response_length;
	if (ret)
		return ret;
	if (response_length != sizeof(expected) ||
	    memcmp(diag->response, expected, sizeof(expected)))
		return -EPROTO;
	return 0;
}

static int mt6797_stp_diag_query_full(struct mt6797_stp_diag *diag,
				      struct mt6797_stp_session *session)
{
	static const u8 params[] = { 0x04 };
	static const u8 expected[] = {
		0x00, 0x04, 0xdf, 0x0e, 0x68, 0x01,
	};
	u16 response_length = 0;
	int ret;

	/* This query deliberately runs after the mode switch, using full STP. */
	ret = mt6797_stp_session_wmt(session, 0x04, params, sizeof(params),
				     diag->response, &response_length,
				     sizeof(diag->response),
				     msecs_to_jiffies(MT6797_STP_DIAG_TIMEOUT_MS));
	diag->response_length = response_length;
	if (ret)
		return ret;
	if (response_length != sizeof(expected) ||
	    memcmp(diag->response, expected, sizeof(expected)))
		return -EPROTO;
	return 0;
}

static int mt6797_stp_diag_function(struct mt6797_stp_diag *diag,
				    struct mt6797_stp_session *session,
				    u8 function, bool enabled)
{
	const u8 params[] = { function, enabled ? 1 : 0 };
	u8 response[MT6797_STP_DIAG_RESPONSE_MAX];
	u16 response_length = 0;
	int ret;

	/* WMT_FUNC_CTRL event carries exactly one status byte. */
	ret = mt6797_stp_session_wmt(session, MT6797_WMT_OPCODE_FUNC_CTRL,
				     params, ARRAY_SIZE(params), response,
				     &response_length, sizeof(response),
				     msecs_to_jiffies(MT6797_STP_DIAG_TIMEOUT_MS));
	diag->response_length = response_length;
	if (ret)
		return ret;
	if (response_length != 1 || response[0])
		return -EPROTO;
	dev_info(diag->dev, "WMT function %u %s acknowledged\n", function,
		 enabled ? "on" : "off");
	return 0;
}

static int mt6797_stp_diag_run(struct mt6797_stp_diag *diag,
				       enum mt6797_stp_diag_mode mode)
{
	static const struct mt6797_stp_session_ops session_ops = {
		.rx_frame = mt6797_stp_diag_rx,
	};
	struct mt6797_stp_host_ops host_ops = { };
	struct mt6797_stp_session *session;
	void *host_context = NULL;
	int ret;
	int release_ret;
	u8 function = 0;
	bool function_enabled = false;
	bool function_powered = false;
	bool function_power_changed = false;
	bool function_on_ack = false;
	bool hif_claimed = false;

	ret = mt6797_consys_stp_acquire(diag->consys_dev, &host_ops,
					&host_context);
	if (ret)
		return ret;
	if (mode == MT6797_STP_DIAG_EMI_TEST) {
		ret = mt6797_consys_emi_patch_begin(host_context);
		if (!ret)
			ret = mt6797_consys_emi_patch_end(host_context);
		goto out_release;
	}

	session = mt6797_stp_session_alloc(diag->dev, &session_ops, diag);
	if (!session) {
		ret = -ENOMEM;
		goto out_release;
	}
	ret = mt6797_stp_session_set_host_ops(session, &host_ops, host_context);
	if (ret)
		goto out_free;
	ret = mt6797_stp_session_start(session, true);
	if (ret)
		goto out_free;
	ret = mt6797_stp_session_wake(session);
	if (ret)
		goto out_stop;

	if (mode == MT6797_STP_DIAG_BT_ON || mode == MT6797_STP_DIAG_BT_OFF ||
	    mode == MT6797_STP_DIAG_WIFI_ON || mode == MT6797_STP_DIAG_WIFI_OFF ||
	    mode == MT6797_STP_DIAG_WIFI_HIF) {
		function = (mode == MT6797_STP_DIAG_BT_ON ||
			    mode == MT6797_STP_DIAG_BT_OFF) ?
			   MT6797_CONSYS_FUNC_BT : MT6797_CONSYS_FUNC_WIFI;
		function_enabled = mode == MT6797_STP_DIAG_BT_ON ||
				 mode == MT6797_STP_DIAG_WIFI_ON ||
				 mode == MT6797_STP_DIAG_WIFI_HIF;
		ret = mt6797_stp_session_configure_btif_full(
			session, msecs_to_jiffies(MT6797_STP_DIAG_TIMEOUT_MS));
		if (!ret && function_enabled) {
			ret = mt6797_consys_radio_power(host_context, function, true,
							&function_power_changed);
			function_powered = !ret && function_power_changed;
		}
		if (!ret)
			ret = mt6797_stp_diag_function(diag, session, function,
						function_enabled);
		function_on_ack = !ret && function_enabled;
		if (!ret && mode == MT6797_STP_DIAG_WIFI_HIF) {
			u32 wcir;

			/* Function-on is the ownership hand-off.  The HIF driver does
			 * not power the radio or infer this state itself. */
			ret = mt6797_wlan_hif_claim(diag->dev);
			if (!ret)
				hif_claimed = true;
			if (!ret)
				ret = mt6797_wlan_hif_wcir_read(diag->dev, &wcir);
			if (!ret) {
				put_unaligned_le32(wcir, diag->response);
				diag->response_length = sizeof(wcir);
				if ((wcir & 0xffff) != MT6797_WLAN_WCIR_CHIP_ID) {
					dev_err(diag->dev,
						"WLAN WCIR chip ID 0x%08x != 0x%04x\n",
						wcir, MT6797_WLAN_WCIR_CHIP_ID);
					ret = -EPROTO;
				} else {
					dev_info(diag->dev,
						 "WLAN HIF WCIR chip ID 0x%04x confirmed\n",
						 wcir & 0xffff);
				}
			}
		}
		if (hif_claimed) {
			release_ret = mt6797_wlan_hif_release(diag->dev);
			hif_claimed = false;
			if (!ret && release_ret)
				ret = release_ret;
		}
		if (mode == MT6797_STP_DIAG_WIFI_HIF && function_on_ack) {
			int off_ret;

			off_ret = mt6797_stp_diag_function(diag, session, function,
							false);
			if (!ret && off_ret)
				ret = off_ret;
			if (function_powered) {
				off_ret = mt6797_consys_radio_power(host_context, function,
								false, NULL);
				if (!ret && off_ret)
					ret = off_ret;
			}
		}
		if (!ret && !function_enabled)
			ret = mt6797_consys_radio_power(host_context, function, false, NULL);
		if (ret && function_powered)
			mt6797_consys_radio_power(host_context, function, false, NULL);
	} else if (mode == MT6797_STP_DIAG_FULL ||
		   mode == MT6797_STP_DIAG_VERSION ||
		   mode == MT6797_STP_DIAG_PATCH) {
		ret = mt6797_stp_session_configure_btif_full(
			session, msecs_to_jiffies(MT6797_STP_DIAG_TIMEOUT_MS));
		if (!ret && mode == MT6797_STP_DIAG_FULL)
			ret = mt6797_stp_diag_query_full(diag, session);
		if (!ret && (mode == MT6797_STP_DIAG_VERSION ||
			     mode == MT6797_STP_DIAG_PATCH)) {
			u32 hw_version;
			u32 fw_version;

			ret = mt6797_wmt_read_version(diag->dev, session,
							   &hw_version, &fw_version);
			if (!ret && mode == MT6797_STP_DIAG_VERSION) {
				put_unaligned_le32(hw_version, diag->response);
				put_unaligned_le32(fw_version, diag->response + 4);
				diag->response_length = 8;
			}
			if (!ret && mode == MT6797_STP_DIAG_PATCH)
				ret = mt6797_wmt_download_patches(diag->dev, session, host_context, hw_version,
							    fw_version);
		}
	} else {
		ret = mt6797_stp_session_prepare_btif_mandatory(session);
		if (ret)
			goto out_stop;
		ret = mt6797_stp_diag_query(diag, session);
	}

out_stop:
	mt6797_stp_session_stop(session);
out_free:
	mt6797_stp_session_free(session);
out_release:
	release_ret = mt6797_consys_stp_release(host_context);
	if (!ret && release_ret)
		ret = release_ret;
	return ret;
}

static ssize_t command_show(struct device *dev, struct device_attribute *attr,
				    char *buf)
{
	return sysfs_emit(buf,
			  "write 'query', 'full', 'version', 'patch', 'emi-test', 'bt-on', 'bt-off', 'wifi-on', 'wifi-off', or 'wifi-hif' to execute one bounded test\n");
}

static ssize_t command_store(struct device *dev, struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct mt6797_stp_diag *diag = dev_get_drvdata(dev);
	enum mt6797_stp_diag_mode mode;
	const char *name;
	int ret;

	if (sysfs_streq(buf, "query")) {
		mode = MT6797_STP_DIAG_QUERY;
		name = "query";
	} else if (sysfs_streq(buf, "full")) {
		mode = MT6797_STP_DIAG_FULL;
		name = "full";
	} else if (sysfs_streq(buf, "version")) {
		mode = MT6797_STP_DIAG_VERSION;
		name = "version";
	} else if (sysfs_streq(buf, "patch")) {
		mode = MT6797_STP_DIAG_PATCH;
		name = "patch";
	} else if (sysfs_streq(buf, "emi-test")) {
		mode = MT6797_STP_DIAG_EMI_TEST;
		name = "emi-test";
	} else if (sysfs_streq(buf, "bt-on")) {
		mode = MT6797_STP_DIAG_BT_ON;
		name = "bt-on";
	} else if (sysfs_streq(buf, "bt-off")) {
		mode = MT6797_STP_DIAG_BT_OFF;
		name = "bt-off";
	} else if (sysfs_streq(buf, "wifi-on")) {
		mode = MT6797_STP_DIAG_WIFI_ON;
		name = "wifi-on";
	} else if (sysfs_streq(buf, "wifi-off")) {
		mode = MT6797_STP_DIAG_WIFI_OFF;
		name = "wifi-off";
	} else if (sysfs_streq(buf, "wifi-hif")) {
		mode = MT6797_STP_DIAG_WIFI_HIF;
		name = "wifi-hif";
	} else {
		return -EINVAL;
	}

	mutex_lock(&diag->lock);
	if (diag->busy) {
		mutex_unlock(&diag->lock);
		return -EBUSY;
	}
	diag->busy = true;
	diag->response_length = 0;
	ret = mt6797_stp_diag_run(diag, mode);
	diag->last_result = ret;
	diag->busy = false;
	mutex_unlock(&diag->lock);

	dev_info(dev, "STP %s result=%d response_length=%u\n",
		 name, ret, diag->response_length);
	return ret ? ret : count;
}

static ssize_t result_show(struct device *dev, struct device_attribute *attr,
				   char *buf)
{
	struct mt6797_stp_diag *diag = dev_get_drvdata(dev);
	int result;

	mutex_lock(&diag->lock);
	result = diag->last_result;
	mutex_unlock(&diag->lock);
	return sysfs_emit(buf, "%d\n", result);
}

static ssize_t response_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct mt6797_stp_diag *diag = dev_get_drvdata(dev);
	ssize_t written = 0;
	u16 length;
	u16 index;

	mutex_lock(&diag->lock);
	length = diag->response_length;
	for (index = 0; index < length && written < PAGE_SIZE - 4; index++)
		written += scnprintf(buf + written, PAGE_SIZE - written,
				     "%s%02x", index ? " " : "", diag->response[index]);
	mutex_unlock(&diag->lock);
	if (written < PAGE_SIZE - 1)
		buf[written++] = '\n';
	return written;
}

static ssize_t busy_show(struct device *dev, struct device_attribute *attr,
				 char *buf)
{
	struct mt6797_stp_diag *diag = dev_get_drvdata(dev);
	bool busy;

	mutex_lock(&diag->lock);
	busy = diag->busy;
	mutex_unlock(&diag->lock);
	return sysfs_emit(buf, "%d\n", busy);
}

static DEVICE_ATTR_RW(command);
static DEVICE_ATTR_RO(result);
static DEVICE_ATTR_RO(response);
static DEVICE_ATTR_RO(busy);

static struct attribute *mt6797_stp_diag_attrs[] = {
	&dev_attr_command.attr,
	&dev_attr_result.attr,
	&dev_attr_response.attr,
	&dev_attr_busy.attr,
	NULL,
};
ATTRIBUTE_GROUPS(mt6797_stp_diag);

static int mt6797_stp_diag_probe(struct platform_device *pdev)
{
	struct mt6797_stp_diag *diag;
	struct platform_device *consys_pdev;
	struct device_node *consys_node;
	int ret;

	consys_node = of_parse_phandle(pdev->dev.of_node, "consys", 0);
	if (!consys_node)
		return dev_err_probe(&pdev->dev, -EINVAL,
				     "missing consys phandle\n");
	consys_pdev = of_find_device_by_node(consys_node);
	of_node_put(consys_node);
	if (!consys_pdev)
		return -EPROBE_DEFER;
	if (!dev_get_drvdata(&consys_pdev->dev)) {
		put_device(&consys_pdev->dev);
		return -EPROBE_DEFER;
	}

	diag = devm_kzalloc(&pdev->dev, sizeof(*diag), GFP_KERNEL);
	if (!diag) {
		put_device(&consys_pdev->dev);
		return -ENOMEM;
	}
	diag->dev = &pdev->dev;
	diag->consys_dev = &consys_pdev->dev;
	mutex_init(&diag->lock);
	diag->last_result = -ENODATA;
	platform_set_drvdata(pdev, diag);
	ret = device_add_groups(&pdev->dev, mt6797_stp_diag_groups);
	if (ret) {
		put_device(&consys_pdev->dev);
		return dev_err_probe(&pdev->dev, ret,
				     "failed to create diagnostic sysfs\n");
	}
	dev_info(&pdev->dev,
		 "MT6797 STP diagnostic ready; power ConSys before command\n");
	return 0;
}

static void mt6797_stp_diag_remove(struct platform_device *pdev)
{
	struct mt6797_stp_diag *diag = platform_get_drvdata(pdev);

	device_remove_groups(&pdev->dev, mt6797_stp_diag_groups);
	if (diag)
		put_device(diag->consys_dev);
}

static const struct of_device_id mt6797_stp_diag_of_match[] = {
	{ .compatible = "mediatek,mt6797-stp-diag" },
	{ }
};
MODULE_DEVICE_TABLE(of, mt6797_stp_diag_of_match);

static struct platform_driver mt6797_stp_diag_driver = {
	.probe = mt6797_stp_diag_probe,
	.remove = mt6797_stp_diag_remove,
	.driver = {
		.name = "mt6797-stp-diag",
		.of_match_table = mt6797_stp_diag_of_match,
	},
};

static DEFINE_MUTEX(mt6797_stp_diag_registration_lock);
static bool mt6797_stp_diag_registered;
static bool mt6797_stp_diag_deferred;
static bool mt6797_stp_diag_enable;

static int mt6797_stp_diag_register(void)
{
	int ret = 0;

	mutex_lock(&mt6797_stp_diag_registration_lock);
	if (!mt6797_stp_diag_registered) {
		ret = platform_driver_register(&mt6797_stp_diag_driver);
		if (!ret) {
			mt6797_stp_diag_registered = true;
			mt6797_stp_diag_enable = true;
		}
	}
	mutex_unlock(&mt6797_stp_diag_registration_lock);
	return ret;
}

static int mt6797_stp_diag_enable_set(const char *val,
					      const struct kernel_param *kp)
{
	bool enable;
	int ret;

	ret = kstrtobool(val, &enable);
	if (ret)
		return ret;
	if (!enable)
		return -EINVAL;
	/*
	 * The parameter is also present for the ordinary opt-in DT.  In that
	 * case the late initcall already registered the driver, so accepting a
	 * redundant write keeps the initramfs hook harmless on a normal image.
	 * Only the explicitly deferred DT is allowed to register at this point.
	 */
	if (!mt6797_stp_diag_deferred) {
		if (READ_ONCE(mt6797_stp_diag_registered)) {
			*(bool *)kp->arg = true;
			return 0;
		}
		return -EINVAL;
	}

	return mt6797_stp_diag_register();
}

static const struct kernel_param_ops mt6797_stp_diag_enable_ops = {
	.set = mt6797_stp_diag_enable_set,
	.get = param_get_bool,
};
module_param_cb(enable, &mt6797_stp_diag_enable_ops,
		&mt6797_stp_diag_enable, 0644);
MODULE_PARM_DESC(enable,
		 "register the deferred STP diagnostic platform driver (0/1)");

/*
 * The driver is an opt-in bring-up hook.  Avoid registering a built-in
 * platform driver on the ordinary headless DT, where no diagnostic device
 * exists.  Besides avoiding needless initcall activity, this keeps the
 * diagnostic object completely dormant until a board DT explicitly opts in.
 *
 * When built in, register at late initcall time.  The diagnostic probe only
 * creates sysfs state and takes a reference to ConSys, but doing that during
 * the ordinary device/deferred-probe window unnecessarily races storage
 * bring-up on this board.  A DT node with mediatek,defer-registration delays
 * the registration until userspace writes the enable parameter after rootfs
 * is mounted.  A module keeps the normal module-init semantics unless the DT
 * explicitly requests the same deferred path.
 */
static int __init mt6797_stp_diag_init(void)
{
	struct device_node *node;

	node = of_find_compatible_node(NULL, NULL, "mediatek,mt6797-stp-diag");
	if (!node)
		return 0;
	mt6797_stp_diag_deferred = of_property_read_bool(node,
							"mediatek,defer-registration");
	of_node_put(node);
	if (mt6797_stp_diag_deferred) {
		pr_info("MT6797 STP diagnostic registration deferred; write enable=1 after rootfs mount\n");
		return 0;
	}

	return mt6797_stp_diag_register();
}
#if IS_BUILTIN(CONFIG_MTK_MT6797_STP_DIAG)
late_initcall(mt6797_stp_diag_init);
#else
module_init(mt6797_stp_diag_init);
#endif

static void __exit mt6797_stp_diag_exit(void)
{
	mutex_lock(&mt6797_stp_diag_registration_lock);
	if (mt6797_stp_diag_registered) {
		platform_driver_unregister(&mt6797_stp_diag_driver);
		mt6797_stp_diag_registered = false;
		mt6797_stp_diag_enable = false;
	}
	mutex_unlock(&mt6797_stp_diag_registration_lock);
}
module_exit(mt6797_stp_diag_exit);

MODULE_DESCRIPTION("MT6797 bounded STP/WMT bring-up diagnostic");
MODULE_LICENSE("GPL");
