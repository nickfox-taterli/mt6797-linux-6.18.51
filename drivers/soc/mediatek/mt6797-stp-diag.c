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

#include <linux/mt6797-consys.h>
#include <linux/mt6797-stp-session.h>

#define MT6797_STP_DIAG_TIMEOUT_MS	1000
#define MT6797_STP_DIAG_RESPONSE_MAX	32

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

static int mt6797_stp_diag_run(struct mt6797_stp_diag *diag, bool full)
{
	static const struct mt6797_stp_session_ops session_ops = {
		.rx_frame = mt6797_stp_diag_rx,
	};
	struct mt6797_stp_host_ops host_ops = { };
	struct mt6797_stp_session *session;
	void *host_context = NULL;
	int ret;
	int release_ret;

	ret = mt6797_consys_stp_acquire(diag->consys_dev, &host_ops,
					&host_context);
	if (ret)
		return ret;

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

	if (full)
		ret = mt6797_stp_session_configure_btif_full(
			session, msecs_to_jiffies(MT6797_STP_DIAG_TIMEOUT_MS));
	else
		ret = mt6797_stp_diag_query(diag, session);

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
	return sysfs_emit(buf, "write 'query' or 'full' to execute one bounded test\n");
}

static ssize_t command_store(struct device *dev, struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct mt6797_stp_diag *diag = dev_get_drvdata(dev);
	bool full;
	int ret;

	if (sysfs_streq(buf, "query"))
		full = false;
	else if (sysfs_streq(buf, "full"))
		full = true;
	else
		return -EINVAL;

	mutex_lock(&diag->lock);
	if (diag->busy) {
		mutex_unlock(&diag->lock);
		return -EBUSY;
	}
	diag->busy = true;
	diag->response_length = 0;
	ret = mt6797_stp_diag_run(diag, full);
	diag->last_result = ret;
	diag->busy = false;
	mutex_unlock(&diag->lock);

	dev_info(dev, "STP %s result=%d response_length=%u\n",
		 full ? "full" : "query", ret, diag->response_length);
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
module_platform_driver(mt6797_stp_diag_driver);

MODULE_DESCRIPTION("MT6797 bounded STP/WMT bring-up diagnostic");
MODULE_LICENSE("GPL");
