// SPDX-License-Identifier: GPL-2.0-only
/* Native HCI consumer of the shared MT6797 BTIF/STP/WMT provider.
 * BSP reference: conn_soc/linux/pub/stp_chrdev_bt.c, BT_TASK_INDX H4 stream.
 */
#include <linux/firmware.h>
#include <linux/module.h>
#include <linux/mt6797-stp-session.h>
#include <linux/mt6797-wmt.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/skbuff.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>

#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci_core.h>

#include "btmt6797-h4.h"
#include "btmt6797-compat.h"

#define MT6797_BT_TX_QUEUE_MAX 64
#define MT6797_BT_NVRAM "mediatek/mt6797/BT.bin"

/* BSP libbluetooth_mtk: chip 0x6797, 64-byte NVRAM record. */
struct mt6797_bt_nvram {
	u8 address[6];
	u8 voice[2];
	u8 codec[4];
	u8 radio[6];
	u8 sleep[7];
	u8 feature[2];
	u8 tx_power[3];
	u8 coex[6];
	u8 reserved[28];
} __packed;

struct mt6797_bt {
	struct hci_dev *hdev;
	struct mt6797_wmt_client *client;
	struct sk_buff_head txq;
	struct work_struct tx_work;
	spinlock_t rx_lock; /* Serializes the H4 parser and close's reset. */
	struct mt6797_h4 rx;
	bool running;
	bool failed;
	bool legacy_ble;
};

static int mt6797_bt_receive_frame(void *context, u8 type,
				   const u8 *data, size_t length)
{
	struct mt6797_bt *bt = context;
	struct sk_buff *skb;

	skb = bt_skb_alloc(length, GFP_ATOMIC);
	if (!skb)
		return -ENOMEM;
	hci_skb_pkt_type(skb) = type;
	skb_put_data(skb, data, length);
	if (READ_ONCE(bt->legacy_ble) && type == HCI_EVENT_PKT &&
	    !hci_dev_test_flag(bt->hdev, HCI_USER_CHANNEL))
		mt6797_bt_fixup_capabilities(skb->data, skb->len);
	return hci_recv_frame(bt->hdev, skb);
}

static void mt6797_bt_receive(void *context, const u8 *data, size_t length)
{
	struct mt6797_bt *bt = context;
	unsigned long flags;
	int ret = 0;

	spin_lock_irqsave(&bt->rx_lock, flags);
	if (READ_ONCE(bt->running) && !READ_ONCE(bt->failed)) {
		bt->hdev->stat.byte_rx += length;
		ret = mt6797_h4_feed(&bt->rx, data, length,
				     mt6797_bt_receive_frame, bt);
		if (ret) {
			bt->hdev->stat.err_rx++;
			WRITE_ONCE(bt->failed, true);
		}
	}
	spin_unlock_irqrestore(&bt->rx_lock, flags);
	if (ret) {
		bt_dev_err(bt->hdev, "STP H4 receive failed: %d", ret);
		hci_reset_dev(bt->hdev);
	}
}

static void mt6797_bt_tx_work(struct work_struct *work)
{
	struct mt6797_bt *bt = container_of(work, struct mt6797_bt, tx_work);
	struct sk_buff *skb;

	while (READ_ONCE(bt->running) && !READ_ONCE(bt->failed) &&
	       (skb = skb_dequeue(&bt->txq))) {
		u8 packet[MT6797_H4_MAX_FRAME + 1];
		u8 type = hci_skb_pkt_type(skb);
		int ret;

		packet[0] = type;
		ret = skb_copy_bits(skb, 0, packet + 1, skb->len);
		if (!ret)
			ret = mt6797_wmt_client_send(bt->client, packet, skb->len + 1);
		if (!ret) {
			bt->hdev->stat.byte_tx += skb->len + 1;
			switch (type) {
			case HCI_COMMAND_PKT:
				bt->hdev->stat.cmd_tx++;
				break;
			case HCI_ACLDATA_PKT:
				bt->hdev->stat.acl_tx++;
				break;
			case HCI_SCODATA_PKT:
				bt->hdev->stat.sco_tx++;
				break;
			}
		}
		kfree_skb(skb);
		if (ret) {
			bt->hdev->stat.err_tx++;
			WRITE_ONCE(bt->failed, true);
			bt_dev_err(bt->hdev, "STP H4 transmit failed: %d", ret);
			skb_queue_purge(&bt->txq);
			hci_reset_dev(bt->hdev);
			break;
		}
	}
}

static int mt6797_bt_send(struct hci_dev *hdev, struct sk_buff *skb)
{
	struct mt6797_bt *bt = hci_get_drvdata(hdev);
	unsigned long flags;
	int ret = 0;

	switch (hci_skb_pkt_type(skb)) {
	case HCI_COMMAND_PKT:
	case HCI_ACLDATA_PKT:
	case HCI_SCODATA_PKT:
		break;
	default:
		return -EILSEQ;
	}
	if (!skb->len || skb->len > MT6797_H4_MAX_FRAME)
		return -EMSGSIZE;
	spin_lock_irqsave(&bt->txq.lock, flags);
	if (!READ_ONCE(bt->running) || READ_ONCE(bt->failed)) {
		ret = -ESHUTDOWN;
	} else if (bt->txq.qlen >= MT6797_BT_TX_QUEUE_MAX) {
		ret = -ENOSPC;
	} else {
		__skb_queue_tail(&bt->txq, skb);
		schedule_work(&bt->tx_work);
	}
	spin_unlock_irqrestore(&bt->txq.lock, flags);
	return ret;
}

static int mt6797_bt_open(struct hci_dev *hdev)
{
	struct mt6797_bt *bt = hci_get_drvdata(hdev);
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&bt->rx_lock, flags);
	mt6797_h4_reset(&bt->rx);
	WRITE_ONCE(bt->failed, false);
	WRITE_ONCE(bt->legacy_ble, false);
	WRITE_ONCE(bt->running, true);
	spin_unlock_irqrestore(&bt->rx_lock, flags);
	ret = mt6797_wmt_client_open(bt->client);
	if (ret)
		WRITE_ONCE(bt->running, false);
	return ret;
}

static int mt6797_bt_flush(struct hci_dev *hdev)
{
	struct mt6797_bt *bt = hci_get_drvdata(hdev);

	skb_queue_purge(&bt->txq);
	return 0;
}

static int mt6797_bt_close(struct hci_dev *hdev)
{
	struct mt6797_bt *bt = hci_get_drvdata(hdev);
	unsigned long flags;
	int ret;

	/* Lock against send's enqueue before cancelling the final TX work. */
	spin_lock_irqsave(&bt->txq.lock, flags);
	WRITE_ONCE(bt->running, false);
	spin_unlock_irqrestore(&bt->txq.lock, flags);
	cancel_work_sync(&bt->tx_work);
	skb_queue_purge(&bt->txq);
	ret = mt6797_wmt_client_close(bt->client);
	spin_lock_irqsave(&bt->rx_lock, flags);
	mt6797_h4_reset(&bt->rx);
	spin_unlock_irqrestore(&bt->rx_lock, flags);
	return ret;
}

static int mt6797_bt_command(struct hci_dev *hdev, u16 opcode, u8 length,
			     const void *parameters)
{
	struct sk_buff *skb;
	int ret;

	skb = __hci_cmd_sync(hdev, opcode, length, parameters, HCI_INIT_TIMEOUT);
	if (IS_ERR(skb))
		return PTR_ERR(skb);
	ret = skb->len ? -bt_to_errno(skb->data[0]) : -EPROTO;
	kfree_skb(skb);
	if (ret)
		bt_dev_err(hdev, "BSP command 0x%04x failed: %d", opcode, ret);
	return ret;
}

static int mt6797_bt_set_bdaddr(struct hci_dev *hdev, const bdaddr_t *address)
{
	return mt6797_bt_command(hdev, 0xfc1a, sizeof(*address), address);
}

static bool mt6797_bt_valid_address(const bdaddr_t *address)
{
	static const bdaddr_t bsp_default = {{ 0x01, 0x79, 0x02, 0x46, 0x00, 0x00 }};

	return bacmp(address, BDADDR_ANY) && bacmp(address, BDADDR_NONE) &&
	       bacmp(address, &bsp_default);
}

static int mt6797_bt_setup(struct hci_dev *hdev)
{
	struct mt6797_bt *bt = hci_get_drvdata(hdev);
	const struct mt6797_bt_nvram *nvram;
	struct hci_rp_read_local_version *version;
	struct hci_rp_read_bd_addr *rp;
	const struct firmware *fw;
	struct sk_buff *skb;
	u8 log_enable = 0;
	bdaddr_t address;
	bdaddr_t configured;
	int ret;

	ret = request_firmware(&fw, MT6797_BT_NVRAM, &hdev->dev);
	if (ret)
		return ret;
	if (fw->size != sizeof(*nvram)) {
		ret = -EINVAL;
		goto release;
	}
	nvram = (const void *)fw->data;
	/* BSP preload script reads the controller's address before programming it.
	 * Preserve its eFuse address: the fallback record is not factory identity.
	 * Board-specific public addresses can use the standard MGMT configuration.
	 */
	skb = __hci_cmd_sync(hdev, HCI_OP_READ_BD_ADDR, 0, NULL, HCI_INIT_TIMEOUT);
	if (IS_ERR(skb)) {
		ret = PTR_ERR(skb);
		goto release;
	}
	if (skb->len != sizeof(*rp)) {
		ret = -EPROTO;
	} else {
		rp = (void *)skb->data;
		ret = -bt_to_errno(rp->status);
		bacpy(&address, &rp->bdaddr);
	}
	kfree_skb(skb);
	if (ret)
		goto release;
	baswap(&configured, (const bdaddr_t *)nvram->address);
	if (mt6797_bt_valid_address(&configured))
		bacpy(&address, &configured);
	if (!mt6797_bt_valid_address(&address)) {
		hci_set_quirk(hdev, HCI_QUIRK_INVALID_BDADDR);
		bt_dev_warn(hdev, "controller needs a board-specific public address");
	} else {
		ret = mt6797_bt_set_bdaddr(hdev, &address);
		if (ret)
			goto release;
	}
	/* Exact conn_soc preload sequence, not the newer UART/USB protocol. */
	ret = mt6797_bt_command(hdev, 0xfc79, sizeof(nvram->radio), nvram->radio);
	if (ret)
		goto release;
	ret = mt6797_bt_command(hdev, 0xfc93, sizeof(nvram->tx_power), nvram->tx_power);
	if (ret)
		goto release;
	ret = mt6797_bt_command(hdev, 0xfc7a, sizeof(nvram->sleep), nvram->sleep);
	if (ret)
		goto release;
	ret = mt6797_bt_command(hdev, HCI_OP_RESET, 0, NULL);
	if (ret)
		goto release;
	ret = mt6797_bt_command(hdev, 0xfcbe, sizeof(log_enable), &log_enable);
	if (ret)
		goto release;
	skb = __hci_cmd_sync(hdev, HCI_OP_READ_LOCAL_VERSION, 0, NULL, HCI_INIT_TIMEOUT);
	if (IS_ERR(skb)) {
		ret = PTR_ERR(skb);
		goto release;
	}
	if (skb->len != sizeof(*version)) {
		ret = -EPROTO;
	} else {
		version = (void *)skb->data;
		ret = -bt_to_errno(version->status);
		WRITE_ONCE(bt->legacy_ble, !ret && version->hci_ver == 8 &&
			   le16_to_cpu(version->manufacturer) == 70 &&
			   ((!version->hci_rev && !version->lmp_subver) ||
			    (le16_to_cpu(version->hci_rev) == 0x1317 &&
			     le16_to_cpu(version->lmp_subver) == 0x0630)));
	}
	kfree_skb(skb);
	if (!ret) {
		bt_dev_info(hdev, "BSP MT6797 radio/power/sleep setup complete");
		if (READ_ONCE(bt->legacy_ble)) {
			/* Advertised MWS query returns Unknown Connection Identifier. */
			hci_set_quirk(hdev, HCI_QUIRK_BROKEN_MWS_TRANSPORT_CONFIG);
			bt_dev_warn(hdev, "2016 firmware BLE event compatibility: controller DLE/privacy/ECC unavailable");
		}
	}
release:
	release_firmware(fw);
	return ret;
}

static int mt6797_bt_probe(struct platform_device *pdev)
{
	static const struct mt6797_wmt_client_ops ops = {
		.receive = mt6797_bt_receive,
	};
	struct mt6797_bt *bt;
	struct hci_dev *hdev;
	int ret;

	bt = devm_kzalloc(&pdev->dev, sizeof(*bt), GFP_KERNEL);
	if (!bt)
		return -ENOMEM;
	hdev = hci_alloc_dev();
	if (!hdev)
		return -ENOMEM;
	bt->hdev = hdev;
	skb_queue_head_init(&bt->txq);
	spin_lock_init(&bt->rx_lock);
	INIT_WORK(&bt->tx_work, mt6797_bt_tx_work);
	bt->client = mt6797_wmt_client_get(&pdev->dev, MT6797_STP_TASK_BT, &ops, bt);
	if (IS_ERR(bt->client)) {
		ret = PTR_ERR(bt->client);
		goto free_hdev;
	}
	/* BTIF is an SoC interprocessor H4/STP link, not an exposed UART. */
	hdev->bus = HCI_IPC;
	hdev->open = mt6797_bt_open;
	hdev->close = mt6797_bt_close;
	hdev->flush = mt6797_bt_flush;
	hdev->send = mt6797_bt_send;
	hdev->setup = mt6797_bt_setup;
	hdev->set_bdaddr = mt6797_bt_set_bdaddr;
	hci_set_quirk(hdev, HCI_QUIRK_NON_PERSISTENT_SETUP);
	hci_set_drvdata(hdev, bt);
	SET_HCIDEV_DEV(hdev, &pdev->dev);
	platform_set_drvdata(pdev, bt);
	ret = hci_register_dev(hdev);
	if (!ret)
		return 0;
	mt6797_wmt_client_put(bt->client);
free_hdev:
	hci_free_dev(hdev);
	return dev_err_probe(&pdev->dev, ret, "cannot register STP Bluetooth HCI\n");
}

static void mt6797_bt_remove(struct platform_device *pdev)
{
	struct mt6797_bt *bt = platform_get_drvdata(pdev);

	hci_unregister_dev(bt->hdev);
	cancel_work_sync(&bt->tx_work);
	skb_queue_purge(&bt->txq);
	mt6797_wmt_client_put(bt->client);
	hci_free_dev(bt->hdev);
}

static const struct of_device_id mt6797_bt_of_match[] = {
	{ .compatible = "mediatek,mt6797-bluetooth" },
	{ }
};
MODULE_DEVICE_TABLE(of, mt6797_bt_of_match);

static struct platform_driver mt6797_bt_driver = {
	.probe = mt6797_bt_probe,
	.remove = mt6797_bt_remove,
	.driver = {
		.name = "mt6797-bluetooth",
		.of_match_table = mt6797_bt_of_match,
	},
};
module_platform_driver(mt6797_bt_driver);

MODULE_DESCRIPTION("MediaTek MT6797 Bluetooth HCI over shared BTIF/STP/WMT");
MODULE_LICENSE("GPL");
MODULE_FIRMWARE(MT6797_BT_NVRAM);
