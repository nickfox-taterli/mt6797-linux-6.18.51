// SPDX-License-Identifier: GPL-2.0-only
/* Shared MT6797 firmware, STP and radio lifecycle for functional clients. */
#include <linux/device.h>
#include <linux/err.h>
#include <linux/jiffies.h>
#include <linux/kref.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/slab.h>
#include <linux/srcu.h>

#include <linux/mt6797-consys.h>
#include <linux/mt6797-stp-session.h>
#include <linux/mt6797-wmt.h>

struct mt6797_wmt {
	struct device *dev;
	struct device *consys;
	struct device *btif;
	struct mutex lock;
	struct kref ref;
	struct srcu_struct rx_srcu;
	struct mt6797_wmt_client __rcu *clients[4];
	struct mt6797_stp_session *session;
	struct mt6797_wmt_config config;
	struct mt6797_wmt_calibration calibration;
	void *host;
	unsigned int users;
	int error;
	bool removed;
};

struct mt6797_wmt_client {
	struct mt6797_wmt *wmt;
	struct mt6797_wmt_client_ops ops;
	void *context;
	u8 task;
	bool active;
};

static void mt6797_wmt_release(struct kref *ref)
{
	struct mt6797_wmt *wmt = container_of(ref, struct mt6797_wmt, ref);

	cleanup_srcu_struct(&wmt->rx_srcu);
	put_device(wmt->btif);
	put_device(wmt->consys);
	put_device(wmt->dev);
	kfree(wmt);
}

static void mt6797_wmt_receive(void *context, u8 task,
			       const u8 *data, u16 length)
{
	struct mt6797_wmt *wmt = context;
	struct mt6797_wmt_client *client;
	int index;

	if (task >= ARRAY_SIZE(wmt->clients))
		return;
	index = srcu_read_lock(&wmt->rx_srcu);
	client = srcu_dereference(wmt->clients[task], &wmt->rx_srcu);
	if (client && READ_ONCE(client->active) && client->ops.receive)
		client->ops.receive(client->context, data, length);
	srcu_read_unlock(&wmt->rx_srcu, index);
}

/* lock held; session_stop drains RX callbacks before freeing the session. */
static int mt6797_wmt_stop(struct mt6797_wmt *wmt)
{
	int ret = 0;

	if (wmt->session) {
		mt6797_stp_session_stop(wmt->session);
		mt6797_stp_session_free(wmt->session);
		wmt->session = NULL;
	}
	if (wmt->host) {
		ret = mt6797_consys_stp_release(wmt->host);
		wmt->host = NULL;
	}
	wmt->error = ret;
	return ret;
}

static int mt6797_wmt_start(struct mt6797_wmt *wmt)
{
	static const struct mt6797_stp_session_ops ops = {
		.rx_frame = mt6797_wmt_receive,
	};
	struct mt6797_stp_host_ops host_ops;
	int ret;

	ret = mt6797_consys_stp_open(wmt->consys, &host_ops, &wmt->host);
	if (ret)
		return ret;
	wmt->session = mt6797_stp_session_alloc(wmt->dev, &ops, wmt);
	if (!wmt->session) {
		ret = -ENOMEM;
		goto fail;
	}
	ret = mt6797_stp_session_set_host_ops(wmt->session, &host_ops, wmt->host);
	if (ret)
		goto fail;
	ret = mt6797_stp_session_start(wmt->session, true);
	if (ret)
		goto fail;
	ret = mt6797_stp_session_wake(wmt->session);
	if (ret)
		goto fail;
	ret = mt6797_wmt_bootstrap(wmt->dev, wmt->session, wmt->host, &wmt->config,
				   &wmt->calibration);
	if (!ret)
		return 0;
fail:
	mt6797_wmt_stop(wmt);
	return ret;
}

struct mt6797_wmt_client *mt6797_wmt_client_get(struct device *dev, u8 task,
				const struct mt6797_wmt_client_ops *ops, void *context)
{
	struct mt6797_wmt_client *client;
	struct platform_device *pdev;
	struct device_node *node;
	struct mt6797_wmt *wmt;
	int ret;

	if (!dev || (task != MT6797_STP_TASK_BT && task != MT6797_STP_TASK_WIFI))
		return ERR_PTR(-EINVAL);
	node = of_parse_phandle(dev->of_node, "mediatek,wmt", 0);
	if (!node)
		return ERR_PTR(-EINVAL);
	pdev = of_find_device_by_node(node);
	of_node_put(node);
	if (!pdev)
		return ERR_PTR(-EPROBE_DEFER);
	device_lock(&pdev->dev);
	wmt = platform_get_drvdata(pdev);
	if (wmt)
		kref_get(&wmt->ref);
	device_unlock(&pdev->dev);
	if (!wmt) {
		ret = -EPROBE_DEFER;
		goto put_device;
	}
	if (!device_link_add(dev, &pdev->dev, DL_FLAG_AUTOREMOVE_CONSUMER)) {
		ret = -ENOMEM;
		goto put_wmt;
	}
	client = kzalloc(sizeof(*client), GFP_KERNEL);
	if (!client) {
		ret = -ENOMEM;
		goto put_wmt;
	}
	client->wmt = wmt;
	client->task = task;
	client->context = context;
	if (ops)
		client->ops = *ops;
	mutex_lock(&wmt->lock);
	if (wmt->removed)
		ret = -ENODEV;
	else if (rcu_access_pointer(wmt->clients[task]))
		ret = -EBUSY;
	else {
		rcu_assign_pointer(wmt->clients[task], client);
		ret = 0;
	}
	mutex_unlock(&wmt->lock);
	if (!ret) {
		put_device(&pdev->dev);
		return client;
	}
	kfree(client);
put_wmt:
	kref_put(&wmt->ref, mt6797_wmt_release);
put_device:
	put_device(&pdev->dev);
	return ERR_PTR(ret);
}
EXPORT_SYMBOL_GPL(mt6797_wmt_client_get);

int mt6797_wmt_client_open(struct mt6797_wmt_client *client)
{
	struct mt6797_wmt *wmt;
	int ret;

	if (!client)
		return -EINVAL;
	wmt = client->wmt;
	mutex_lock(&wmt->lock);
	if (wmt->removed) {
		ret = -ENODEV;
		goto out;
	}
	if (client->active) {
		ret = -EALREADY;
		goto out;
	}
	if (!wmt->users) {
		ret = mt6797_wmt_start(wmt);
		if (ret)
			goto out;
	} else if (wmt->error) {
		ret = wmt->error;
		goto out;
	}
	ret = mt6797_consys_radio_power(wmt->host, client->task, true, NULL);
	if (ret)
		goto fail;
	/* BSP WLAN probe owns its AHB function; only BT uses WMT FUNC_CTRL. */
	if (client->task == MT6797_STP_TASK_BT) {
		ret = mt6797_wmt_function_control(wmt->dev, wmt->session,
						 client->task, true);
		if (ret) {
			mt6797_consys_radio_power(wmt->host, client->task, false, NULL);
			goto fail;
		}
	}
	WRITE_ONCE(client->active, true);
	wmt->users++;
	goto out;
fail:
	if (!wmt->users)
		mt6797_wmt_stop(wmt);
out:
	mutex_unlock(&wmt->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(mt6797_wmt_client_open);

int mt6797_wmt_client_close(struct mt6797_wmt_client *client)
{
	struct mt6797_wmt *wmt;
	int ret = 0, cleanup_ret;

	if (!client)
		return -EINVAL;
	wmt = client->wmt;
	mutex_lock(&wmt->lock);
	if (!client->active)
		goto out;
	WRITE_ONCE(client->active, false);
	if (wmt->removed)
		goto out;
	if (client->task == MT6797_STP_TASK_BT)
		ret = mt6797_wmt_function_control(wmt->dev, wmt->session,
						 client->task, false);
	cleanup_ret = mt6797_consys_radio_power(wmt->host, client->task, false, NULL);
	if (!ret)
		ret = cleanup_ret;
	if (ret)
		wmt->error = ret;
	if (!--wmt->users) {
		cleanup_ret = mt6797_wmt_stop(wmt);
		if (!ret)
			ret = cleanup_ret;
	}
out:
	mutex_unlock(&wmt->lock);
	synchronize_srcu(&wmt->rx_srcu);
	return ret;
}
EXPORT_SYMBOL_GPL(mt6797_wmt_client_close);

void mt6797_wmt_client_put(struct mt6797_wmt_client *client)
{
	struct mt6797_wmt *wmt;

	if (!client)
		return;
	wmt = client->wmt;
	mt6797_wmt_client_close(client);
	mutex_lock(&wmt->lock);
	RCU_INIT_POINTER(wmt->clients[client->task], NULL);
	mutex_unlock(&wmt->lock);
	synchronize_srcu(&wmt->rx_srcu);
	kfree(client);
	kref_put(&wmt->ref, mt6797_wmt_release);
}
EXPORT_SYMBOL_GPL(mt6797_wmt_client_put);

int mt6797_wmt_client_send(struct mt6797_wmt_client *client,
			   const void *data, size_t length)
{
	struct mt6797_wmt *wmt;
	int ret;

	if (!client || !data || !length || length > 2048)
		return -EINVAL;
	wmt = client->wmt;
	mutex_lock(&wmt->lock);
	if (wmt->removed || !client->active)
		ret = -ESHUTDOWN;
	else if (wmt->error)
		ret = wmt->error;
	else
		ret = mt6797_stp_session_send(wmt->session, client->task, data, length,
					     msecs_to_jiffies(2000));
	mutex_unlock(&wmt->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(mt6797_wmt_client_send);

int mt6797_wmt_wifi_emi_write(struct mt6797_wmt_client *client, u32 offset,
			    const void *data, size_t length)
{
	struct mt6797_wmt *wmt;
	int ret;

	if (!client || client->task != MT6797_STP_TASK_WIFI)
		return -EINVAL;
	wmt = client->wmt;
	mutex_lock(&wmt->lock);
	if (wmt->removed || !client->active)
		ret = -ESHUTDOWN;
	else
		ret = mt6797_consys_wifi_emi_write(wmt->host, offset, data, length);
	mutex_unlock(&wmt->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(mt6797_wmt_wifi_emi_write);

int mt6797_wmt_client_version(struct mt6797_wmt_client *client, u32 *hw, u32 *fw)
{
	struct mt6797_wmt *wmt;
	int ret = 0;

	if (!client || !hw || !fw)
		return -EINVAL;
	wmt = client->wmt;
	mutex_lock(&wmt->lock);
	if (wmt->removed || !client->active)
		ret = -ESHUTDOWN;
	else {
		*hw = wmt->calibration.hw_version;
		*fw = wmt->calibration.fw_version;
	}
	mutex_unlock(&wmt->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(mt6797_wmt_client_version);

int mt6797_wmt_client_cpupcr(struct mt6797_wmt_client *client, u32 *value)
{
	struct mt6797_wmt *wmt;
	int ret;

	if (!client || !value)
		return -EINVAL;
	wmt = client->wmt;
	mutex_lock(&wmt->lock);
	if (wmt->removed || !client->active)
		ret = -ESHUTDOWN;
	else
		ret = mt6797_consys_read_cpupcr(wmt->host, value);
	mutex_unlock(&wmt->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(mt6797_wmt_client_cpupcr);

static struct device *mt6797_wmt_provider(struct device *dev, const char *name)
{
	struct platform_device *pdev;
	struct device_node *node;

	node = of_parse_phandle(dev->of_node, name, 0);
	if (!node)
		return ERR_PTR(-EINVAL);
	pdev = of_find_device_by_node(node);
	of_node_put(node);
	if (!pdev)
		return ERR_PTR(-EPROBE_DEFER);
	if (!dev_get_drvdata(&pdev->dev)) {
		put_device(&pdev->dev);
		return ERR_PTR(-EPROBE_DEFER);
	}
	if (!device_link_add(dev, &pdev->dev, DL_FLAG_AUTOREMOVE_CONSUMER)) {
		put_device(&pdev->dev);
		return ERR_PTR(-ENOMEM);
	}
	return &pdev->dev;
}

static int mt6797_wmt_probe(struct platform_device *pdev)
{
	struct mt6797_wmt *wmt;
	int ret;

	wmt = kzalloc(sizeof(*wmt), GFP_KERNEL);
	if (!wmt)
		return -ENOMEM;
	wmt->consys = mt6797_wmt_provider(&pdev->dev, "consys");
	if (IS_ERR(wmt->consys)) {
		ret = PTR_ERR(wmt->consys);
		goto free;
	}
	wmt->btif = mt6797_wmt_provider(&pdev->dev, "btif");
	if (IS_ERR(wmt->btif)) {
		ret = PTR_ERR(wmt->btif);
		goto put_consys;
	}
	ret = device_property_read_u8(&pdev->dev, "mediatek,antenna-mode",
				      &wmt->config.antenna_mode);
	if (!ret)
		ret = device_property_read_u8(&pdev->dev, "mediatek,fm-transport",
					      &wmt->config.fm_transport);
	if (ret)
		goto put_btif;
	wmt->config.co_clock = device_property_read_bool(&pdev->dev, "mediatek,co-clock");
	ret = init_srcu_struct(&wmt->rx_srcu);
	if (ret)
		goto put_btif;
	wmt->dev = get_device(&pdev->dev);
	kref_init(&wmt->ref);
	mutex_init(&wmt->lock);
	platform_set_drvdata(pdev, wmt);
	dev_info(&pdev->dev, "shared WMT ready; firmware starts with the first radio client\n");
	return 0;
put_btif:
	put_device(wmt->btif);
put_consys:
	put_device(wmt->consys);
free:
	kfree(wmt);
	return ret;
}

static void mt6797_wmt_remove(struct platform_device *pdev)
{
	struct mt6797_wmt *wmt = platform_get_drvdata(pdev);
	struct mt6797_wmt_client *client;
	unsigned int task;

	platform_set_drvdata(pdev, NULL);
	mutex_lock(&wmt->lock);
	wmt->removed = true;
	for (task = 0; task < ARRAY_SIZE(wmt->clients); task++) {
		client = rcu_dereference_protected(wmt->clients[task],
						 lockdep_is_held(&wmt->lock));
		if (client)
			WRITE_ONCE(client->active, false);
	}
	wmt->users = 0;
	mt6797_wmt_stop(wmt);
	mutex_unlock(&wmt->lock);
	kref_put(&wmt->ref, mt6797_wmt_release);
}

static const struct of_device_id mt6797_wmt_of_match[] = {
	{ .compatible = "mediatek,mt6797-wmt" },
	{ }
};
MODULE_DEVICE_TABLE(of, mt6797_wmt_of_match);

static struct platform_driver mt6797_wmt_driver = {
	.probe = mt6797_wmt_probe,
	.remove = mt6797_wmt_remove,
	.driver = {
		.name = "mt6797-wmt",
		.of_match_table = mt6797_wmt_of_match,
	},
};
module_platform_driver(mt6797_wmt_driver);

MODULE_DESCRIPTION("MT6797 shared connectivity firmware and radio lifecycle");
MODULE_LICENSE("GPL");
