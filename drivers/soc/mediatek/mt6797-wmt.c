// SPDX-License-Identifier: GPL-2.0-only
/* Shared MT6797 firmware, STP and radio lifecycle for functional clients. */
#include <linux/device.h>
#include <linux/err.h>
#include <linux/jiffies.h>
#include <linux/interrupt.h>
#include <linux/kref.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/slab.h>
#include <linux/srcu.h>
#include <linux/workqueue.h>

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
	struct device *radios[4];
	struct work_struct recovery_work;
	bool recovery_pending;
	bool reprobing;
	unsigned int recovery_generation;
	unsigned long last_recovery;
	int recovery_result;
	int wake_irq;
	spinlock_t wake_irq_lock;
	bool wake_irq_enabled;
	atomic_t host_wake_pending;
	struct work_struct host_wake_work;
	struct delayed_work idle_work;
	bool psm_enabled;
	bool asleep;
	unsigned long last_activity;
	u64 sleeps, wakes, host_wakes;
	int psm_error;
	struct mt6797_stp_session *session;
	struct mt6797_wmt_config config;
	struct mt6797_wmt_calibration calibration;
	void *host;
	unsigned int users;
	int error;
	bool removed;
};

/* Protect drvdata lookup independently of the supplier device lock. Recovery
 * holds that device lock while rebinding consumers through their normal probe.
 */
static DEFINE_MUTEX(mt6797_wmt_lookup_lock);

#define WMT_PSM_IDLE_MS 200
static int mt6797_wmt_queue_recovery(struct mt6797_wmt *wmt, int reason, bool manual);

static void mt6797_wmt_psm_touch(struct mt6797_wmt *wmt)
{
	WRITE_ONCE(wmt->last_activity, jiffies);
	if (READ_ONCE(wmt->psm_enabled) && !READ_ONCE(wmt->removed))
		mod_delayed_work(system_wq, &wmt->idle_work, msecs_to_jiffies(WMT_PSM_IDLE_MS));
}

static void mt6797_wmt_wake_irq_set(struct mt6797_wmt *wmt, bool enabled)
{
	unsigned long flags;

	if (wmt->wake_irq < 0)
		return;
	spin_lock_irqsave(&wmt->wake_irq_lock, flags);
	if (enabled != wmt->wake_irq_enabled) {
		wmt->wake_irq_enabled = enabled;
		if (enabled)
			enable_irq(wmt->wake_irq);
		else
			disable_irq_nosync(wmt->wake_irq);
	}
	spin_unlock_irqrestore(&wmt->wake_irq_lock, flags);
	if (!enabled)
		synchronize_irq(wmt->wake_irq);
}

/* wmt->lock serializes every power transaction against functional sends. */
static int mt6797_wmt_psm_wake(struct mt6797_wmt *wmt)
{
	bool host_wake;
	int ret;

	mt6797_wmt_wake_irq_set(wmt, false);
	host_wake = atomic_xchg(&wmt->host_wake_pending, 0);
	if (!wmt->asleep && !host_wake)
		return 0;
	ret = mt6797_stp_session_idle(wmt->session, false);
	if (!ret)
		ret = mt6797_stp_session_power(wmt->session,
			host_wake ? MT6797_STP_POWER_HOST_AWAKE : MT6797_STP_POWER_WAKE,
			msecs_to_jiffies(1000));
	if (ret) {
		wmt->psm_error = ret;
		wmt->psm_enabled = false;
		mt6797_wmt_queue_recovery(wmt, ret, false);
		return ret;
	}
	wmt->asleep = false;
	if (host_wake)
		wmt->host_wakes++;
	else
		wmt->wakes++;
	mt6797_wmt_psm_touch(wmt);
	return 0;
}

static void mt6797_wmt_host_wake_work(struct work_struct *work)
{
	struct mt6797_wmt *wmt = container_of(work, struct mt6797_wmt, host_wake_work);

	mutex_lock(&wmt->lock);
	if (!wmt->removed && !wmt->recovery_pending && wmt->session && wmt->users)
		mt6797_wmt_psm_wake(wmt);
	mutex_unlock(&wmt->lock);
}

static irqreturn_t mt6797_wmt_wake_irq(int irq, void *data)
{
	struct mt6797_wmt *wmt = data;
	unsigned long flags;

	spin_lock_irqsave(&wmt->wake_irq_lock, flags);
	if (wmt->wake_irq_enabled) {
		wmt->wake_irq_enabled = false;
		disable_irq_nosync(irq);
	}
	spin_unlock_irqrestore(&wmt->wake_irq_lock, flags);
	atomic_set(&wmt->host_wake_pending, 1);
	if (!READ_ONCE(wmt->removed))
		schedule_work(&wmt->host_wake_work);
	return IRQ_HANDLED;
}

static void mt6797_wmt_idle_work(struct work_struct *work)
{
	struct mt6797_wmt *wmt = container_of(to_delayed_work(work), struct mt6797_wmt, idle_work);
	unsigned long due;
	int ret;

	mutex_lock(&wmt->lock);
	if (wmt->removed || !wmt->psm_enabled || wmt->asleep || !wmt->users ||
	    !wmt->session || wmt->error || wmt->recovery_pending)
		goto out;
	due = READ_ONCE(wmt->last_activity) + msecs_to_jiffies(WMT_PSM_IDLE_MS);
	if (time_before(jiffies, due)) {
		mod_delayed_work(system_wq, &wmt->idle_work, due - jiffies);
		goto out;
	}
	ret = mt6797_stp_session_power(wmt->session, MT6797_STP_POWER_SLEEP,
				       msecs_to_jiffies(1000));
	if (ret) {
		wmt->psm_error = ret;
		wmt->psm_enabled = false;
		mt6797_wmt_queue_recovery(wmt, ret, false);
		goto out;
	}
	wmt->asleep = true;
	wmt->sleeps++;
	ret = mt6797_stp_session_idle(wmt->session, true);
	if (ret && ret != -EBUSY) {
		wmt->psm_error = ret;
		wmt->psm_enabled = false;
		mt6797_wmt_queue_recovery(wmt, ret, false);
		goto out;
	}
	mt6797_wmt_wake_irq_set(wmt, true);
out:
	mutex_unlock(&wmt->lock);
}

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
	for (int task = 0; task < ARRAY_SIZE(wmt->radios); task++)
		put_device(wmt->radios[task]);
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
	mt6797_wmt_psm_touch(wmt);
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

	cancel_delayed_work(&wmt->idle_work);
	mt6797_wmt_wake_irq_set(wmt, false);
	wmt->asleep = false;
	atomic_set(&wmt->host_wake_pending, 0);
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

/* One whole-chip attempt, with no WMT mutex held across consumer callbacks.
 * Driver removal drains HCI/WLAN work and RX before the last client releases
 * STP and ConSys. Reprobe then reloads firmware and restores native devices.
 */
static void mt6797_wmt_recovery_work(struct work_struct *work)
{
	struct mt6797_wmt *wmt = container_of(work, struct mt6797_wmt, recovery_work);
	struct device *radios[4] = {};
	unsigned int task, count = 0;
	int ret = 0, attached;
	bool locked;

	/* Supplier removal may be holding this lock and waiting for this work.
	 * Never wait on it: removal owns teardown if it wins this race.
	 */
	locked = device_trylock(wmt->dev);
	mutex_lock(&wmt->lock);
	if (!locked || wmt->removed) {
		ret = wmt->removed ? -ENODEV : -EBUSY;
		goto finish;
	}
	for (task = 0; task < ARRAY_SIZE(radios); task++) {
		if (wmt->radios[task] && device_is_registered(wmt->radios[task])) {
			radios[task] = get_device(wmt->radios[task]);
			count++;
		}
	}
	if (!count) {
		ret = -ENODEV;
		goto finish;
	}
	wmt->recovery_generation++;
	dev_warn(wmt->dev, "whole-chip recovery %u: quiescing %u radios\n",
		 wmt->recovery_generation, count);
	/* Quiescing is gated by recovery_pending, so retain only errors from
	 * this teardown, not a stale fault from an earlier failed attempt.
	 */
	wmt->error = 0;
	mutex_unlock(&wmt->lock);
	for (task = 0; task < ARRAY_SIZE(radios); task++)
		if (radios[task])
			device_release_driver(radios[task]);
	mutex_lock(&wmt->lock);
	if (wmt->users || wmt->session || wmt->host) {
		ret = -EBUSY;
		goto finish;
	}
	if (wmt->error) {
		ret = wmt->error;
		goto finish;
	}
	wmt->reprobing = true;
	mutex_unlock(&wmt->lock);
	for (task = 0; task < ARRAY_SIZE(radios); task++) {
		if (!radios[task])
			continue;
		attached = device_attach(radios[task]);
		if (attached != 1 && !ret)
			ret = attached < 0 ? attached : -ENODEV;
	}
	if (ret) {
		/* A partial reprobe must not leave one radio using a failed MCU. */
		for (task = 0; task < ARRAY_SIZE(radios); task++)
			if (radios[task])
				device_release_driver(radios[task]);
	}
	mutex_lock(&wmt->lock);
finish:
	wmt->recovery_result = ret;
	wmt->error = ret;
	wmt->reprobing = false;
	wmt->recovery_pending = false;
	mutex_unlock(&wmt->lock);
	if (locked)
		device_unlock(wmt->dev);
	for (task = 0; task < ARRAY_SIZE(radios); task++)
		put_device(radios[task]);
	dev_info(wmt->dev, "whole-chip recovery %u %s: %d\n",
		 wmt->recovery_generation, ret ? "failed" : "complete", ret);
	if (!ret) {
		char *env[] = { "X20_RECOVERY=complete", NULL };

		/* Network policy belongs to userspace; the new netdev has no IP. */
		kobject_uevent_env(&wmt->dev->kobj, KOBJ_CHANGE, env);
	}
}

/* lock held. Terminal errors schedule one attempt, never a reset loop. A
 * failed attempt requires an explicit retry; successful ones have a cooldown.
 */
static int mt6797_wmt_queue_recovery(struct mt6797_wmt *wmt, int reason, bool manual)
{
	if (wmt->removed)
		return -ENODEV;
	if (wmt->recovery_pending)
		return -EALREADY;
	wmt->error = reason;
	if (!manual && (wmt->recovery_result ||
	    (wmt->last_recovery && time_before(jiffies, wmt->last_recovery + 30 * HZ))))
		return -EAGAIN;
	wmt->recovery_pending = true;
	wmt->last_recovery = jiffies;
	if (!schedule_work(&wmt->recovery_work)) {
		wmt->recovery_pending = false;
		return -EBUSY;
	}
	return 0;
}

struct mt6797_wmt_client *mt6797_wmt_client_get(struct device *dev, u8 task,
				const struct mt6797_wmt_client_ops *ops, void *context)
{
	struct mt6797_wmt_client *client;
	struct platform_device *pdev;
	struct device_node *node;
	struct mt6797_wmt *wmt;
	struct device *old_radio = NULL;
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
	mutex_lock(&mt6797_wmt_lookup_lock);
	wmt = platform_get_drvdata(pdev);
	if (wmt)
		kref_get(&wmt->ref);
	mutex_unlock(&mt6797_wmt_lookup_lock);
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
		if (wmt->radios[task] != dev) {
			old_radio = wmt->radios[task];
			wmt->radios[task] = get_device(dev);
		}
		rcu_assign_pointer(wmt->clients[task], client);
		ret = 0;
	}
	mutex_unlock(&wmt->lock);
	put_device(old_radio);
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
	if (wmt->recovery_pending && !wmt->reprobing) {
		ret = -EAGAIN;
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
	ret = mt6797_wmt_psm_wake(wmt);
	if (ret)
		goto fail;
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
	mt6797_wmt_psm_touch(wmt);
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
	if (!wmt->recovery_pending)
		ret = mt6797_wmt_psm_wake(wmt);
	if (!ret && client->task == MT6797_STP_TASK_BT && !wmt->recovery_pending)
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
	else if (wmt->recovery_pending && !wmt->reprobing)
		ret = -EAGAIN;
	else if (wmt->error)
		ret = wmt->error;
	else {
		ret = mt6797_wmt_psm_wake(wmt);
		if (!ret)
			ret = mt6797_stp_session_send(wmt->session, client->task, data, length,
						     msecs_to_jiffies(2000));
		mt6797_wmt_psm_touch(wmt);
	}
	if (ret == -ETIMEDOUT || ret == -EIO || ret == -EPROTO || ret == -ENOBUFS)
		mt6797_wmt_queue_recovery(wmt, ret, false);
	mutex_unlock(&wmt->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(mt6797_wmt_client_send);

int mt6797_wmt_client_recover(struct mt6797_wmt_client *client, int reason)
{
	int ret;

	if (!client || reason >= 0)
		return -EINVAL;
	mutex_lock(&client->wmt->lock);
	ret = mt6797_wmt_queue_recovery(client->wmt, reason, false);
	mutex_unlock(&client->wmt->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(mt6797_wmt_client_recover);

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
	else if (wmt->recovery_pending && !wmt->reprobing)
		ret = -EAGAIN;
	else {
		ret = mt6797_wmt_psm_wake(wmt);
		if (!ret)
			ret = mt6797_consys_wifi_emi_write(wmt->host, offset, data, length);
		mt6797_wmt_psm_touch(wmt);
	}
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
	else if (wmt->recovery_pending && !wmt->reprobing)
		ret = -EAGAIN;
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
	else if (wmt->recovery_pending && !wmt->reprobing)
		ret = -EAGAIN;
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

static ssize_t recovery_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct mt6797_wmt *wmt = dev_get_drvdata(dev);
	const char *state;
	ssize_t length;

	mutex_lock(&wmt->lock);
	state = wmt->recovery_pending ? (wmt->reprobing ? "reprobing" : "quiescing") :
		(wmt->recovery_result ? "failed" : wmt->error ? "fault" : "idle");
	length = sysfs_emit(buf, "%s generation=%u result=%d error=%d users=%u\n", state,
			   wmt->recovery_generation, wmt->recovery_result, wmt->error, wmt->users);
	mutex_unlock(&wmt->lock);
	return length;
}

static ssize_t recovery_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct mt6797_wmt *wmt = dev_get_drvdata(dev);
	int ret;

	if (!sysfs_streq(buf, "recover") &&
	    !(IS_ENABLED(CONFIG_FAULT_INJECTION) && sysfs_streq(buf, "fault")))
		return -EINVAL;
	mutex_lock(&wmt->lock);
	if (sysfs_streq(buf, "fault")) {
		if (!wmt->session || !wmt->users || wmt->recovery_pending)
			ret = -EBUSY;
		else {
			/* Latch a provider failure, not an arbitrary MMIO fault. The
			 * next real radio send exercises automatic error recovery.
			 */
			wmt->error = -EIO;
			ret = 0;
		}
	} else {
		ret = mt6797_wmt_queue_recovery(wmt, -ECANCELED, true);
	}
	mutex_unlock(&wmt->lock);
	return ret ?: count;
}
static DEVICE_ATTR_RW(recovery);

static ssize_t psm_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mt6797_wmt *wmt = dev_get_drvdata(dev);
	ssize_t length;

	mutex_lock(&wmt->lock);
	length = sysfs_emit(buf, "%s %s sleeps=%llu wakes=%llu host_wakes=%llu error=%d irq=%d\n",
		wmt->psm_enabled ? "auto" : "off", wmt->asleep ? "asleep" : "awake",
		wmt->sleeps, wmt->wakes, wmt->host_wakes, wmt->psm_error, wmt->wake_irq);
	mutex_unlock(&wmt->lock);
	return length;
}

static ssize_t psm_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	struct mt6797_wmt *wmt = dev_get_drvdata(dev);
	bool enabled;
	int ret = 0;

	if (sysfs_streq(buf, "auto"))
		enabled = true;
	else if (sysfs_streq(buf, "off"))
		enabled = false;
	else
		return -EINVAL;
	mutex_lock(&wmt->lock);
	if (wmt->removed || wmt->recovery_pending || wmt->error)
		ret = -EBUSY;
	else if (enabled && wmt->wake_irq < 0)
		ret = -EOPNOTSUPP;
	else {
		if (!enabled && wmt->session)
			ret = mt6797_wmt_psm_wake(wmt);
		if (!ret) {
			wmt->psm_enabled = enabled;
			wmt->psm_error = 0;
			if (enabled)
				mt6797_wmt_psm_touch(wmt);
			else
				cancel_delayed_work(&wmt->idle_work);
		}
	}
	mutex_unlock(&wmt->lock);
	return ret ?: count;
}
static DEVICE_ATTR_RW(psm);
static struct attribute *mt6797_wmt_attrs[] = { &dev_attr_recovery.attr, &dev_attr_psm.attr, NULL };
ATTRIBUTE_GROUPS(mt6797_wmt);

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
	spin_lock_init(&wmt->wake_irq_lock);
	atomic_set(&wmt->host_wake_pending, 0);
	INIT_WORK(&wmt->host_wake_work, mt6797_wmt_host_wake_work);
	INIT_DELAYED_WORK(&wmt->idle_work, mt6797_wmt_idle_work);
	INIT_WORK(&wmt->recovery_work, mt6797_wmt_recovery_work);
	wmt->wake_irq = platform_get_irq_optional(pdev, 0);
	if (wmt->wake_irq == -EPROBE_DEFER) {
		ret = wmt->wake_irq;
		goto cleanup_srcu;
	}
	if (wmt->wake_irq >= 0) {
		ret = devm_request_irq(&pdev->dev, wmt->wake_irq, mt6797_wmt_wake_irq,
				       IRQF_NO_AUTOEN, "mt6797-wmt-wake", wmt);
		if (ret)
			goto cleanup_srcu;
	}
	mutex_lock(&mt6797_wmt_lookup_lock);
	platform_set_drvdata(pdev, wmt);
	mutex_unlock(&mt6797_wmt_lookup_lock);
	dev_info(&pdev->dev, "shared WMT ready; firmware starts with the first radio client\n");
	return 0;
cleanup_srcu:
	put_device(wmt->dev);
	cleanup_srcu_struct(&wmt->rx_srcu);
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

	mutex_lock(&mt6797_wmt_lookup_lock);
	platform_set_drvdata(pdev, NULL);
	mutex_unlock(&mt6797_wmt_lookup_lock);
	mutex_lock(&wmt->lock);
	wmt->removed = true;
	wmt->psm_enabled = false;
	mutex_unlock(&wmt->lock);
	mt6797_wmt_wake_irq_set(wmt, false);
	cancel_delayed_work_sync(&wmt->idle_work);
	cancel_work_sync(&wmt->host_wake_work);
	cancel_work_sync(&wmt->recovery_work);
	mutex_lock(&wmt->lock);
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
		.dev_groups = mt6797_wmt_groups,
	},
};
module_platform_driver(mt6797_wmt_driver);

MODULE_DESCRIPTION("MT6797 shared connectivity firmware and radio lifecycle");
MODULE_LICENSE("GPL");
