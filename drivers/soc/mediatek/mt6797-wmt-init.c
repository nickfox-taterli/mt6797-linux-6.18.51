// SPDX-License-Identifier: GPL-2.0-only
/* Shared MT6797 WMT ROM/firmware initialization, independent of sysfs clients. */
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/firmware.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/unaligned.h>
#include <linux/mt6797-consys.h>
#include <linux/mt6797-stp-session.h>
#include <linux/mt6797-wmt.h>

#define MT6797_WMT_TIMEOUT_MS	2000
#define MT6797_WMT_RESET_TIMEOUT_MS	5000
#define MT6797_WMT_RESPONSE_MAX	32
#define MT6797_WMT_GEN_HVR		0x80000000
#define MT6797_WMT_GEN_FVR		0x80000004
#define MT6797_WMT_GEN_VER_MASK		0x0000ffff
#define MT6797_WMT_PATCH_HEADER_SIZE	28
#define MT6797_WMT_PATCH_FRAGMENT_SIZE	1000
#define MT6797_WMT_PATCH_MAX_SIZE	(256 * 1024)
#define MT6797_WMT_PATCH_COUNT		2
#define MT6797_WMT_PATCH_HW_VERSION	0x8a00
#define MT6797_WMT_PATCH_FW_1		"mediatek/ROMv3_patch_1_1_hdr.bin"
#define MT6797_WMT_PATCH_FW_2		"mediatek/ROMv3_patch_1_0_hdr.bin"

static int mt6797_wmt_read_reg(struct device *dev,
				     struct mt6797_stp_session *session,
				     u32 address, u32 mask, u32 *value)
{
	u8 params[16] = { 0 };
	u8 response[MT6797_WMT_RESPONSE_MAX];
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
				     msecs_to_jiffies(MT6797_WMT_TIMEOUT_MS));
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

int mt6797_wmt_read_version(struct device *dev,
					struct mt6797_stp_session *session,
					u32 *hw_version, u32 *fw_version)
{
	int ret;

	ret = mt6797_wmt_read_reg(dev, session, MT6797_WMT_GEN_HVR,
				       MT6797_WMT_GEN_VER_MASK, hw_version);
	if (ret)
		return ret;
	ret = mt6797_wmt_read_reg(dev, session, MT6797_WMT_GEN_FVR,
				       MT6797_WMT_GEN_VER_MASK, fw_version);
	if (ret)
		return ret;
	dev_info(dev, "WMT ROM version hw=0x%04x fw=0x%04x\n",
		 *hw_version, *fw_version);
	return 0;
}
EXPORT_SYMBOL_GPL(mt6797_wmt_read_version);

static int mt6797_wmt_reg_write(struct mt6797_stp_session *session,
					     const u8 params[16])
{
	static const u8 expected[] = { 0x00, 0x00, 0x00, 0x01 };
	u8 response[MT6797_WMT_RESPONSE_MAX];
	u16 response_length = 0;
	int ret;

	ret = mt6797_stp_session_wmt(session, 0x08, params, 16,
				     response, &response_length, sizeof(response),
				     msecs_to_jiffies(MT6797_WMT_TIMEOUT_MS));
	if (ret)
		return ret;
	if (response_length != sizeof(expected) ||
	    memcmp(response, expected, sizeof(expected)))
		return -EPROTO;
	return 0;
}

static int mt6797_wmt_power_on_dlm(struct mt6797_stp_session *session)
{
	/* Vendor wmt_power_on_dlm_table, without the four-byte WMT header. */
	static const u8 params[][16] = {
		{ 1, 1, 0, 1, 0x60, 0, 0x10, 0x80, 0, 0, 0, 0, 0, 0x0f, 0, 0 },
		{ 1, 1, 0, 1, 0x60, 0, 0x10, 0x80, 0, 0, 0, 0, 0xf0, 0, 0, 0 },
		{ 1, 1, 0, 1, 0x60, 0, 0x10, 0x80, 0, 0, 0, 0, 0x08, 0, 0, 0 },
	};
	unsigned int index;
	int ret;

	for (index = 0; index < ARRAY_SIZE(params); index++) {
		ret = mt6797_wmt_reg_write(session, params[index]);
		if (ret)
			return ret;
	}
	return 0;
}

static int mt6797_wmt_mcu_clock(struct device *dev,
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
		ret = mt6797_wmt_reg_write(session, params[index]);
		if (ret)
			return ret;
	}
	dev_info(dev, "ROMv3 MCU clock %s\n", enable ? "boost enabled" :
		 "boost disabled");
	return 0;
}

static int mt6797_wmt_reset(struct mt6797_stp_session *session)
{
	static const u8 params[] = { 0x04 };
	static const u8 expected[] = { 0x00 };
	u8 response[MT6797_WMT_RESPONSE_MAX];
	u16 response_length = 0;
	int ret;

	ret = mt6797_stp_session_wmt(session, 0x07, params, sizeof(params),
				     response, &response_length, sizeof(response),
				     msecs_to_jiffies(MT6797_WMT_RESET_TIMEOUT_MS));
	if (ret)
		return ret;
	if (response_length != sizeof(expected) ||
	    memcmp(response, expected, sizeof(expected)))
		return -EPROTO;
	return 0;
}

static int mt6797_wmt_patch_header(const struct firmware *fw,
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

	if (!fw || fw->size < MT6797_WMT_PATCH_HEADER_SIZE ||
		fw->size > MT6797_WMT_PATCH_HEADER_SIZE + MT6797_WMT_PATCH_MAX_SIZE)
		return -EINVAL;
	data = fw->data;

	hw_version = get_unaligned_be16(data + 20);
	fw_version = get_unaligned_be16(data + 22);
	patch_info = data[24];
	patch_count = patch_info >> 4;
	sequence = patch_info & 0x0f;
	address = get_unaligned_be24(data + 25);
	payload_size = fw->size - MT6797_WMT_PATCH_HEADER_SIZE;
	if (patch_count != MT6797_WMT_PATCH_COUNT ||
		sequence != expected_sequence || !payload_size ||
		(hw_version != (expected_hw & 0xffff)) ||
		((fw_version ^ (expected_fw & 0xffff)) & 0x00ff))
		return -EPROTO;
	if (!address || address > 0x00ffffff)
		return -ERANGE;

	return 0;
}

static int mt6797_wmt_patch_address(struct device *dev,
						 struct mt6797_stp_session *session,
						 const struct firmware *fw)
{
	u8 params[16] = {
		0x01, 0x01, 0x00, 0x01,
		0x08, 0x05, 0x09, 0x02,
		0x00, 0x00, 0x00, 0x00,
		0xff, 0xff, 0xff, 0xff,
	};
	u8 response[MT6797_WMT_RESPONSE_MAX];
	u16 response_length = 0;
	int ret;

	ret = mt6797_stp_session_wmt(session, 0x08, params, sizeof(params),
				     response, &response_length, sizeof(response),
				     msecs_to_jiffies(MT6797_WMT_TIMEOUT_MS));
	if (ret)
		return ret;
	if (response_length != 4 || response[0] || response[1] ||
		response[2] || response[3] != 1)
		return -EPROTO;

	/* WMT_PATCH_P_ADDRESS_CMD includes a four-byte WMT header. Its
	 * bytes 12..15 are params[8..11] (value), not params[12..15] (mask).
	 * Keep the vendor's full mask and prepend zero to the header address.
	 */
	params[4] = 0x2c;
	params[5] = 0x0b;
	params[8] = 0x00;
	memcpy(params + 9, fw->data + 25, 3);
	ret = mt6797_stp_session_wmt(session, 0x08, params, sizeof(params),
				     response, &response_length, sizeof(response),
				     msecs_to_jiffies(MT6797_WMT_TIMEOUT_MS));
	if (ret)
		return ret;
	if (response_length != 4 || response[0] || response[1] ||
		response[2] || response[3] != 1)
		return -EPROTO;

	dev_info(dev, "ROMv3 patch address 0x%06x accepted\n",
		 get_unaligned_be24(fw->data + 25));
	return 0;
}

static int mt6797_wmt_patch_one(struct device *dev,
					     struct mt6797_stp_session *session,
					     const struct firmware *fw,
					     u32 expected_fw,
					     u8 sequence)
{
	u8 *params;
	u8 response[MT6797_WMT_RESPONSE_MAX];
	u16 response_length;
	size_t payload_size;
	size_t offset;
	u8 fragment;
	int ret;

	ret = mt6797_wmt_patch_header(fw, MT6797_WMT_PATCH_HW_VERSION,
					  expected_fw, sequence);
	if (ret)
		return ret;
	ret = mt6797_wmt_patch_address(dev, session, fw);
	if (ret)
		return ret;

	payload_size = fw->size - MT6797_WMT_PATCH_HEADER_SIZE;
	params = kmalloc(1 + MT6797_WMT_PATCH_FRAGMENT_SIZE, GFP_KERNEL);
	if (!params)
		return -ENOMEM;
	for (offset = 0; offset < payload_size; offset += MT6797_WMT_PATCH_FRAGMENT_SIZE) {
		size_t length = min_t(size_t, MT6797_WMT_PATCH_FRAGMENT_SIZE,
					      payload_size - offset);

		fragment = (offset == 0) ? 0x01 : 0x02;
		if (offset + length == payload_size)
			fragment = 0x03;
		params[0] = fragment;
		memcpy(params + 1, fw->data + MT6797_WMT_PATCH_HEADER_SIZE + offset,
		       length);
		response_length = 0;
		ret = mt6797_stp_session_wmt(session, 0x01, params, 1 + length,
					     response, &response_length, sizeof(response),
					     msecs_to_jiffies(MT6797_WMT_TIMEOUT_MS));
		if (ret)
			break;
		if (response_length != 1 || response[0]) {
			ret = -EPROTO;
			break;
		}
		dev_dbg(dev, "ROMv3 patch seq=%u offset=%zu length=%zu\n",
			sequence, offset, length);
	}
	kfree(params);
	if (!ret)
		dev_info(dev, "ROMv3 patch seq=%u transferred payload=%zu\n",
			 sequence, payload_size);
	return ret;
}

int mt6797_wmt_download_patches(struct device *dev,
					 struct mt6797_stp_session *session,
					 void *host_context,
					 u32 hw_version, u32 fw_version)
{
	static const char * const names[] = {
		MT6797_WMT_PATCH_FW_1,
		MT6797_WMT_PATCH_FW_2,
	};
	const struct firmware *fw[ARRAY_SIZE(names)] = { };
	unsigned int index;
	unsigned int loaded = 0;
	bool clock_boosted = false;
	bool emi_owned = false;
	int ret;
	int cleanup_ret;

	if (hw_version != MT6797_WMT_PATCH_HW_VERSION ||
		((fw_version ^ MT6797_WMT_PATCH_HW_VERSION) & 0x00ff))
		return -EPROTO;
	for (index = 0; index < ARRAY_SIZE(names); index++) {
		ret = request_firmware(&fw[index], names[index], dev);
		if (ret)
			goto out_release;
		loaded++;
		ret = mt6797_wmt_patch_header(fw[index], hw_version,
						 fw_version, index + 1);
		if (ret)
			goto out_release;
	}
	ret = mt6797_consys_emi_patch_begin(host_context);
	if (ret)
		goto out_release;
	emi_owned = true;
	ret = mt6797_wmt_power_on_dlm(session);
	if (ret)
		goto out_release;
	/* Even a failed write can have changed part of the clock setup. */
	clock_boosted = true;
	ret = mt6797_wmt_mcu_clock(dev, session, true);
	if (ret)
		goto out_clock;
	for (index = 0; index < ARRAY_SIZE(fw); index++) {
		ret = mt6797_wmt_patch_one(dev, session, fw[index], fw_version,
							index + 1);
		if (ret)
			goto out_clock;
		/* Vendor init_table_3 resets the target after every patch image. */
		ret = mt6797_wmt_reset(session);
		if (ret)
			goto out_clock;
	}
	ret = mt6797_wmt_mcu_clock(dev, session, false);
	clock_boosted = false;
	if (ret)
		goto out_release;
	dev_info(dev, "ROMv3 patch set transferred in sequence order\n");
	ret = 0;

out_clock:
	if (clock_boosted) {
		cleanup_ret = mt6797_wmt_mcu_clock(dev, session, false);
		if (!ret && cleanup_ret)
			ret = cleanup_ret;
	}
out_release:
	if (emi_owned) {
		cleanup_ret = mt6797_consys_emi_patch_end(host_context);
		if (!ret)
			ret = cleanup_ret;
	}
	while (loaded)
		release_firmware(fw[--loaded]);
	return ret;
}
EXPORT_SYMBOL_GPL(mt6797_wmt_download_patches);

static int mt6797_wmt_expect(struct mt6797_stp_session *session, u8 opcode,
			     const u8 *params, u16 length,
			     const u8 *expected, u16 expected_length)
{
	u8 response[MT6797_WMT_RESPONSE_MAX];
	u16 response_length = 0;
	int ret;

	ret = mt6797_stp_session_wmt(session, opcode, params, length,
				     response, &response_length, sizeof(response),
				     msecs_to_jiffies(MT6797_WMT_TIMEOUT_MS));
	if (ret)
		return ret;
	if (response_length != expected_length ||
	    memcmp(response, expected, expected_length))
		return -EPROTO;
	return 0;
}

int mt6797_wmt_function_control(struct device *dev,
				struct mt6797_stp_session *session,
				u8 function, bool enabled)
{
	const u8 params[] = { function, enabled ? 1 : 0 };
	static const u8 expected[] = { 0 };
	int ret;

	ret = mt6797_wmt_expect(session, 0x06, params, sizeof(params),
				expected, sizeof(expected));
	if (!ret)
		dev_info(dev, "WMT function %u %s acknowledged\n", function,
			 enabled ? "on" : "off");
	return ret;
}
EXPORT_SYMBOL_GPL(mt6797_wmt_function_control);

static int mt6797_wmt_calibrate(struct device *dev,
			       struct mt6797_stp_session *session, void *host_context,
			       struct mt6797_wmt_calibration *calibration)
{
	static const u8 params[] = { 1 };
	bool bt_changed = false, wifi_changed = false;
	int ret, cleanup_ret;

	calibration->length = 0;
	ret = mt6797_consys_radio_power(host_context, MT6797_CONSYS_FUNC_BT,
				       true, &bt_changed);
	if (ret)
		return ret;
	ret = mt6797_consys_radio_power(host_context, MT6797_CONSYS_FUNC_WIFI,
				       true, &wifi_changed);
	if (!ret) {
		ret = mt6797_stp_session_wmt(session, 0x14, params, sizeof(params),
				calibration->data, &calibration->length,
				sizeof(calibration->data),
				msecs_to_jiffies(MT6797_WMT_TIMEOUT_MS));
		/* BSP skips comparison for opcode 0x14 because newer patches append
		 * calibration data. Validate status/subcommand and retain all bytes;
		 * the appended format is firmware-owned, not an invented host ABI.
		 */
		if (!ret && (calibration->length < 2 || calibration->data[0] ||
			     calibration->data[1] != 1))
			ret = -EPROTO;
		if (ret)
			calibration->length = 0;
	}
	if (wifi_changed) {
		cleanup_ret = mt6797_consys_radio_power(host_context,
					 MT6797_CONSYS_FUNC_WIFI, false, NULL);
		if (!ret)
			ret = cleanup_ret;
	}
	if (bt_changed) {
		cleanup_ret = mt6797_consys_radio_power(host_context,
					 MT6797_CONSYS_FUNC_BT, false, NULL);
		if (!ret)
			ret = cleanup_ret;
	}
	if (!ret)
		dev_info(dev, "WMT RF calibration acknowledged, retained %u bytes\n",
			 calibration->length);
	return ret;
}

int mt6797_wmt_post_patch_init(struct device *dev,
			      struct mt6797_stp_session *session,
			      void *host_context, const struct mt6797_wmt_config *config,
			      struct mt6797_wmt_calibration *calibration)
{
	static const u8 status_ok[] = { 0 };
	static const u8 oscillator[] = { 0x08, 0x03 };
	u8 coexistence[] = { 1, 0 };
	u8 fm[] = { 2, 0 }, fm_expected[] = { 0, 0 };
	int ret;

	if (!config || !calibration ||
	    (config->fm_transport != 1 && config->fm_transport != 2))
		return -EINVAL;
	ret = mt6797_wmt_calibrate(dev, session, host_context, calibration);
	if (ret)
		return ret;
	coexistence[1] = config->antenna_mode;
	ret = mt6797_wmt_expect(session, 0x10, coexistence, sizeof(coexistence),
				status_ok, sizeof(status_ok));
	if (ret)
		return ret;
	if (config->co_clock) {
		ret = mt6797_wmt_expect(session, 0x0a, oscillator, sizeof(oscillator),
					status_ok, sizeof(status_ok));
		if (ret)
			return ret;
	}
	fm[1] = fm_expected[1] = config->fm_transport;
	ret = mt6797_wmt_expect(session, 0x05, fm, sizeof(fm),
				fm_expected, sizeof(fm_expected));
	if (!ret)
		dev_info(dev, "WMT board configuration: antenna=%u co-clock=%u FM transport=%u\n",
			 config->antenna_mode, config->co_clock, config->fm_transport);
	return ret;
}
EXPORT_SYMBOL_GPL(mt6797_wmt_post_patch_init);

int mt6797_wmt_bootstrap(struct device *dev, struct mt6797_stp_session *session,
			 void *host_context, const struct mt6797_wmt_config *config,
			      struct mt6797_wmt_calibration *calibration)
{
	u32 hw_version, fw_version;
	int ret;

	if (!dev || !session || !host_context || !config || !calibration)
		return -EINVAL;
	ret = mt6797_stp_session_configure_btif_full(session,
					msecs_to_jiffies(MT6797_WMT_TIMEOUT_MS));
	if (ret)
		return ret;
	ret = mt6797_wmt_read_version(dev, session, &hw_version, &fw_version);
	if (ret)
		return ret;
	calibration->hw_version = hw_version;
	calibration->fw_version = fw_version;
	ret = mt6797_wmt_download_patches(dev, session, host_context, hw_version, fw_version);
	if (ret)
		return ret;
	ret = mt6797_wmt_post_patch_init(dev, session, host_context, config, calibration);
	if (!ret)
		dev_info(dev, "WMT initialization complete\n");
	return ret;
}
EXPORT_SYMBOL_GPL(mt6797_wmt_bootstrap);

MODULE_FIRMWARE(MT6797_WMT_PATCH_FW_1);
MODULE_FIRMWARE(MT6797_WMT_PATCH_FW_2);
MODULE_DESCRIPTION("MT6797 WMT firmware initialization");
MODULE_LICENSE("GPL");
