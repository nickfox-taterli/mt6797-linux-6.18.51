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
#include <linux/firmware.h>
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

#define MT6797_STP_DIAG_TIMEOUT_MS	1000
#define MT6797_STP_DIAG_RESPONSE_MAX	32
#define MT6797_WMT_GEN_HVR		0x80000000
#define MT6797_WMT_GEN_FVR		0x80000004
#define MT6797_WMT_GEN_VER_MASK		0x0000ffff
#define MT6797_STP_PATCH_HEADER_SIZE	28
#define MT6797_STP_PATCH_FRAGMENT_SIZE	1000
#define MT6797_STP_PATCH_MAX_SIZE	(256 * 1024)
#define MT6797_STP_PATCH_COUNT		2
#define MT6797_STP_PATCH_HW_VERSION	0x8a00
#define MT6797_STP_PATCH_FW_1		"mediatek/ROMv3_patch_1_1_hdr.bin"
#define MT6797_STP_PATCH_FW_2		"mediatek/ROMv3_patch_1_0_hdr.bin"
#define MT6797_WMT_OPCODE_FUNC_CTRL	0x06
#define MT6797_WLAN_WCIR_CHIP_ID	0x0279

enum mt6797_stp_diag_mode {
	MT6797_STP_DIAG_QUERY,
	MT6797_STP_DIAG_FULL,
	MT6797_STP_DIAG_VERSION,
	MT6797_STP_DIAG_PATCH,
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

static int mt6797_stp_diag_read_reg(struct mt6797_stp_diag *diag,
				     struct mt6797_stp_session *session,
				     u32 address, u32 mask, u32 *value)
{
	u8 params[16] = { 0 };
	u8 response[MT6797_STP_DIAG_RESPONSE_MAX];
	u16 response_length = 0;
	u32 response_address;
	int ret;

	/* WMT_SET_REG_CMD: read, register type, one register, address, mask. */
	params[0] = 0x02;
	params[1] = 0x01;
	params[3] = 0x01;
	put_unaligned_le32(address, params + 4);
	put_unaligned_le32(mask, params + 12);
	ret = mt6797_stp_session_wmt(session, 0x08, params, sizeof(params),
				     response, &response_length, sizeof(response),
				     msecs_to_jiffies(MT6797_STP_DIAG_TIMEOUT_MS));
	if (ret)
		return ret;
	/* status/type/revision/count + address + value */
	if (response_length != 12 || response[0] || response[1] ||
	    response[2] || response[3] != 1)
		return -EPROTO;
	response_address = get_unaligned_le32(response + 4);
	if (response_address != address)
		return -EPROTO;
	*value = get_unaligned_le32(response + 8) & mask;
	return 0;
}

static int mt6797_stp_diag_read_version(struct mt6797_stp_diag *diag,
					struct mt6797_stp_session *session,
					u32 *hw_version, u32 *fw_version)
{
	int ret;

	ret = mt6797_stp_diag_read_reg(diag, session, MT6797_WMT_GEN_HVR,
				       MT6797_WMT_GEN_VER_MASK, hw_version);
	if (ret)
		return ret;
	ret = mt6797_stp_diag_read_reg(diag, session, MT6797_WMT_GEN_FVR,
				       MT6797_WMT_GEN_VER_MASK, fw_version);
	if (ret)
		return ret;
	dev_info(diag->dev, "WMT ROM version hw=0x%04x fw=0x%04x\n",
		 *hw_version, *fw_version);
	return 0;
}

static int mt6797_stp_diag_reg_write(struct mt6797_stp_session *session,
					     const u8 params[16])
{
	static const u8 expected[] = { 0x00, 0x00, 0x00, 0x01 };
	u8 response[MT6797_STP_DIAG_RESPONSE_MAX];
	u16 response_length = 0;
	int ret;

	ret = mt6797_stp_session_wmt(session, 0x08, params, 16,
				     response, &response_length, sizeof(response),
				     msecs_to_jiffies(MT6797_STP_DIAG_TIMEOUT_MS));
	if (ret)
		return ret;
	if (response_length != sizeof(expected) ||
	    memcmp(response, expected, sizeof(expected)))
		return -EPROTO;
	return 0;
}

static int mt6797_stp_diag_mcu_clock(struct mt6797_stp_diag *diag,
					 struct mt6797_stp_session *session, bool enable)
{
	static const u8 enable_params[][16] = {
		{
			0x01, 0x01, 0x00, 0x01,
			0x10, 0x11, 0x02, 0x81,
			0x00, 0x00, 0x00, 0x10,
			0x00, 0x00, 0x00, 0x10,
		},
		{
			0x01, 0x01, 0x00, 0x01,
			0x0c, 0x01, 0x00, 0x80,
			0x40, 0x00, 0x00, 0x00,
			0xc0, 0x00, 0x00, 0x00,
		},
		{
			0x01, 0x01, 0x00, 0x01,
			0x18, 0x11, 0x02, 0x80,
			0x07, 0x00, 0x00, 0x00,
			0x3f, 0x00, 0x00, 0x00,
		},
		{
			0x01, 0x01, 0x00, 0x01,
			0x00, 0x11, 0x02, 0x81,
			0x04, 0x00, 0x00, 0x00,
			0x07, 0x00, 0x00, 0x00,
		},
	};
	static const u8 disable_params[][16] = {
		{
			0x01, 0x01, 0x00, 0x01,
			0x00, 0x11, 0x02, 0x81,
			0x00, 0x00, 0x00, 0x00,
			0x07, 0x00, 0x00, 0x00,
		},
		{
			0x01, 0x01, 0x00, 0x01,
			0x0c, 0x01, 0x00, 0x80,
			0x00, 0x00, 0x00, 0x00,
			0xc0, 0x00, 0x00, 0x00,
		},
		{
			0x01, 0x01, 0x00, 0x01,
			0x10, 0x11, 0x02, 0x81,
			0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x10,
		},
	};
	const u8 (*params)[16] = enable ? enable_params : disable_params;
	size_t count = enable ? ARRAY_SIZE(enable_params) : ARRAY_SIZE(disable_params);
	size_t index;
	int ret;

	for (index = 0; index < count; index++) {
		ret = mt6797_stp_diag_reg_write(session, params[index]);
		if (ret)
			return ret;
	}
	dev_info(diag->dev, "ROMv3 MCU clock %s\n", enable ? "boost enabled" :
		 "boost disabled");
	return 0;
}

static int mt6797_stp_diag_reset(struct mt6797_stp_session *session)
{
	static const u8 params[] = { 0x04 };
	static const u8 expected[] = { 0x00 };
	u8 response[MT6797_STP_DIAG_RESPONSE_MAX];
	u16 response_length = 0;
	int ret;

	ret = mt6797_stp_session_wmt(session, 0x07, params, sizeof(params),
				     response, &response_length, sizeof(response),
				     msecs_to_jiffies(MT6797_STP_DIAG_TIMEOUT_MS));
	if (ret)
		return ret;
	if (response_length != sizeof(expected) ||
	    memcmp(response, expected, sizeof(expected)))
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

static int mt6797_stp_diag_patch_header(const struct firmware *fw,
					      u32 expected_hw, u32 expected_fw,
					      u8 expected_sequence)
{
	const u8 *data;
	u16 hw_version;
	u16 fw_version;
	u8 patch_info;
	u8 sequence;
	u8 patch_count;
	u32 address;
	size_t payload_size;

	if (!fw || fw->size < MT6797_STP_PATCH_HEADER_SIZE ||
		fw->size > MT6797_STP_PATCH_HEADER_SIZE + MT6797_STP_PATCH_MAX_SIZE)
		return -EINVAL;
	data = fw->data;

	hw_version = get_unaligned_be16(data + 20);
	fw_version = get_unaligned_be16(data + 22);
	patch_info = data[24];
	patch_count = patch_info >> 4;
	sequence = patch_info & 0x0f;
	address = get_unaligned_be24(data + 25);
	payload_size = fw->size - MT6797_STP_PATCH_HEADER_SIZE;
	if (patch_count != MT6797_STP_PATCH_COUNT ||
		sequence != expected_sequence || !payload_size ||
		(hw_version != (expected_hw & 0xffff)) ||
		((fw_version ^ (expected_fw & 0xffff)) & 0x00ff))
		return -EPROTO;
	if (!address || address > 0x00ffffff)
		return -ERANGE;

	return 0;
}

static int mt6797_stp_diag_patch_address(struct mt6797_stp_diag *diag,
						 struct mt6797_stp_session *session,
						 const struct firmware *fw)
{
	u8 params[16] = {
		0x01, 0x01, 0x00, 0x01,
		0x08, 0x05, 0x09, 0x02,
		0x00, 0x00, 0x00, 0x00,
		0xff, 0xff, 0xff, 0xff,
	};
	u8 response[MT6797_STP_DIAG_RESPONSE_MAX];
	u16 response_length = 0;
	int ret;

	ret = mt6797_stp_session_wmt(session, 0x08, params, sizeof(params),
				     response, &response_length, sizeof(response),
				     msecs_to_jiffies(MT6797_STP_DIAG_TIMEOUT_MS));
	if (ret)
		return ret;
	if (response_length != 4 || response[0] || response[1] ||
		response[2] || response[3] != 1)
		return -EPROTO;

	/* The vendor command writes the 24-bit image address into bytes 12..15. */
	/* MT6797 selects the partial-patch RAM window at 0x0b2c. */
	params[4] = 0x2c;
	params[5] = 0x0b;
	params[8] = 0x00;
	params[9] = 0x3f;
	params[10] = 0x00;
	params[11] = 0x01;
	params[12] = 0x00;
	memcpy(params + 13, fw->data + 25, 3);
	ret = mt6797_stp_session_wmt(session, 0x08, params, sizeof(params),
				     response, &response_length, sizeof(response),
				     msecs_to_jiffies(MT6797_STP_DIAG_TIMEOUT_MS));
	if (ret)
		return ret;
	if (response_length != 4 || response[0] || response[1] ||
		response[2] || response[3] != 1)
		return -EPROTO;

	dev_info(diag->dev, "ROMv3 patch address 0x%06x accepted\n",
		 get_unaligned_be24(fw->data + 25));
	return 0;
}

static int mt6797_stp_diag_patch_one(struct mt6797_stp_diag *diag,
					     struct mt6797_stp_session *session,
					     const struct firmware *fw,
					     u32 expected_fw,
					     u8 sequence)
{
	u8 *params;
	u8 response[MT6797_STP_DIAG_RESPONSE_MAX];
	u16 response_length;
	size_t payload_size;
	size_t offset;
	u8 fragment;
	int ret;

	ret = mt6797_stp_diag_patch_header(fw, MT6797_STP_PATCH_HW_VERSION,
					  expected_fw, sequence);
	if (ret)
		return ret;
	ret = mt6797_stp_diag_patch_address(diag, session, fw);
	if (ret)
		return ret;

	payload_size = fw->size - MT6797_STP_PATCH_HEADER_SIZE;
	params = kmalloc(1 + MT6797_STP_PATCH_FRAGMENT_SIZE, GFP_KERNEL);
	if (!params)
		return -ENOMEM;
	for (offset = 0; offset < payload_size; offset += MT6797_STP_PATCH_FRAGMENT_SIZE) {
		size_t length = min_t(size_t, MT6797_STP_PATCH_FRAGMENT_SIZE,
					      payload_size - offset);

		fragment = (offset == 0) ? 0x01 : 0x02;
		if (offset + length == payload_size)
			fragment = 0x03;
		params[0] = fragment;
		memcpy(params + 1, fw->data + MT6797_STP_PATCH_HEADER_SIZE + offset,
		       length);
		response_length = 0;
		ret = mt6797_stp_session_wmt(session, 0x01, params, 1 + length,
					     response, &response_length, sizeof(response),
					     msecs_to_jiffies(MT6797_STP_DIAG_TIMEOUT_MS));
		if (ret)
			break;
		if (response_length != 1 || response[0]) {
			ret = -EPROTO;
			break;
		}
		dev_dbg(diag->dev, "ROMv3 patch seq=%u offset=%zu length=%zu\n",
			sequence, offset, length);
	}
	kfree(params);
	if (!ret)
		dev_info(diag->dev, "ROMv3 patch seq=%u transferred payload=%zu\n",
			 sequence, payload_size);
	return ret;
}

static int mt6797_stp_diag_patch(struct mt6797_stp_diag *diag,
					 struct mt6797_stp_session *session,
					 u32 hw_version, u32 fw_version)
{
	static const char * const names[] = {
		MT6797_STP_PATCH_FW_1,
		MT6797_STP_PATCH_FW_2,
	};
	const struct firmware *fw[ARRAY_SIZE(names)] = { };
	unsigned int index;
	unsigned int loaded = 0;
	bool clock_boosted = false;
	int ret;
	int cleanup_ret;

	if (hw_version != MT6797_STP_PATCH_HW_VERSION ||
		((fw_version ^ MT6797_STP_PATCH_HW_VERSION) & 0x00ff))
		return -EPROTO;
	for (index = 0; index < ARRAY_SIZE(names); index++) {
		ret = request_firmware(&fw[index], names[index], diag->dev);
		if (ret)
			goto out_release;
		loaded++;
	}
	ret = mt6797_stp_diag_mcu_clock(diag, session, true);
	if (ret)
		goto out_release;
	clock_boosted = true;
	for (index = 0; index < ARRAY_SIZE(fw); index++) {
		ret = mt6797_stp_diag_patch_one(diag, session, fw[index], fw_version,
							index + 1);
		if (ret)
			goto out_clock;
		/* Vendor init_table_3 resets the target after every patch image. */
		ret = mt6797_stp_diag_reset(session);
		if (ret)
			goto out_clock;
	}
	ret = mt6797_stp_diag_mcu_clock(diag, session, false);
	clock_boosted = false;
	if (ret)
		goto out_release;
	dev_info(diag->dev, "ROMv3 patch set transferred in sequence order\n");
	ret = 0;

out_clock:
	if (clock_boosted) {
		cleanup_ret = mt6797_stp_diag_mcu_clock(diag, session, false);
		if (!ret && cleanup_ret)
			ret = cleanup_ret;
	}
out_release:
	while (loaded)
		release_firmware(fw[--loaded]);
	return ret;
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

			ret = mt6797_stp_diag_read_version(diag, session,
							   &hw_version, &fw_version);
			if (!ret && mode == MT6797_STP_DIAG_VERSION) {
				put_unaligned_le32(hw_version, diag->response);
				put_unaligned_le32(fw_version, diag->response + 4);
				diag->response_length = 8;
			}
			if (!ret && mode == MT6797_STP_DIAG_PATCH)
				ret = mt6797_stp_diag_patch(diag, session, hw_version,
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
			  "write 'query', 'full', 'version', 'patch', 'bt-on', 'bt-off', 'wifi-on', 'wifi-off', or 'wifi-hif' to execute one bounded test\n");
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

/*
 * The driver is an opt-in bring-up hook.  Avoid registering a built-in
 * platform driver on the ordinary headless DT, where no diagnostic device
 * exists.  Besides avoiding needless initcall activity, this keeps the
 * diagnostic object completely dormant until a board DT explicitly opts in.
 *
 * When built in, register at late initcall time.  The diagnostic probe only
 * creates sysfs state and takes a reference to ConSys, but doing that during
 * the ordinary device/deferred-probe window unnecessarily races storage
 * bring-up on this board.  A module keeps the normal module-init semantics.
 */
static int __init mt6797_stp_diag_init(void)
{
	struct device_node *node;
	int ret;

	node = of_find_compatible_node(NULL, NULL, "mediatek,mt6797-stp-diag");
	if (!node)
		return 0;
	of_node_put(node);

	ret = platform_driver_register(&mt6797_stp_diag_driver);
	if (ret)
		pr_err("failed to register STP diagnostic driver: %d\n", ret);
	return ret;
}
#if IS_BUILTIN(CONFIG_MTK_MT6797_STP_DIAG)
late_initcall(mt6797_stp_diag_init);
#else
module_init(mt6797_stp_diag_init);
#endif

static void __exit mt6797_stp_diag_exit(void)
{
	platform_driver_unregister(&mt6797_stp_diag_driver);
}
module_exit(mt6797_stp_diag_exit);

MODULE_DESCRIPTION("MT6797 bounded STP/WMT bring-up diagnostic");
MODULE_LICENSE("GPL");
