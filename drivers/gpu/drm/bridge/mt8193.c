// SPDX-License-Identifier: GPL-2.0-only
/*
 * MT8193 control/DDC support. Register protocol, power order and DDC commands
 * follow MediaTek's MT6797 3.18 multibridge and HDMI BSP (Copyright 2015 MTK).
 * The X20 connects CCIR656 data on DPI1; the display producer is separate.
 */
#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/debugfs.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <drm/drm_bridge.h>
#include <drm/drm_connector.h>
#include <drm/drm_edid.h>

#define MT8193_PWR_RESET 0x1100
#define MT8193_PWR_CTRL  0x1104
#define MT8193_SYS_CFG   0x101c
#define MT8193_STATUS    0x0020
#define MT8193_DDC_CTL0  0x1700
#define MT8193_DDC_CTL1  0x1704
#define MT8193_DDC_DATA0 0x1708
#define MT8193_DDC_DATA1 0x170c
#define DDC_TRIGGER BIT(0)
#define DDC_MODE GENMASK(6, 4)
#define DDC_LENGTH GENMASK(10, 8)
#define DDC_ACK GENMASK(23, 16)

struct mt8193 {
	struct i2c_client *client;
	struct regmap *map;
	struct mutex lock;
	struct regulator_bulk_data supplies[4];
	struct gpio_desc *reset, *enable;
	struct drm_bridge bridge;
	bool powered;
	bool test_pattern;
	u64 ddc_blocks, ddc_errors;
};

/* Writes are big endian, reads little endian, just as in BSP mt8193_iic.c.
 * Do not use the BSP >= 0x8000 alias protocol for ordinary register offsets.
 */
static int mt8193_reg_read(void *context, unsigned int reg, unsigned int *val)
{
	struct i2c_client *client = context;
	u8 addr[] = { reg >> 8, reg };
	u8 data[4];
	struct i2c_msg msgs[] = {
		{ .addr = client->addr, .len = 2, .buf = addr },
		{ .addr = client->addr, .flags = I2C_M_RD, .len = 4, .buf = data },
	};
	int ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));

	if (ret != ARRAY_SIZE(msgs))
		return ret < 0 ? ret : -EIO;
	*val = (u32)data[0] | (u32)data[1] << 8 | (u32)data[2] << 16 |
	       (u32)data[3] << 24;
	return 0;
}

static int mt8193_reg_write(void *context, unsigned int reg, unsigned int val)
{
	struct i2c_client *client = context;
	u8 data[] = { reg >> 8, reg, val >> 24, val >> 16, val >> 8, val };
	int ret = i2c_master_send(client, data, sizeof(data));

	return ret == sizeof(data) ? 0 : (ret < 0 ? ret : -EIO);
}

static const struct regmap_config mt8193_regmap_config = {
	.reg_bits = 16,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = 0x7ffc,
	.reg_read = mt8193_reg_read,
	.reg_write = mt8193_reg_write,
	.cache_type = REGCACHE_NONE,
};

static void mt8193_power_off(struct mt8193 *mt)
{
	if (!mt->powered)
		return;
	gpiod_set_value_cansleep(mt->enable, 0);
	gpiod_set_value_cansleep(mt->reset, 1);
	regulator_bulk_disable(ARRAY_SIZE(mt->supplies), mt->supplies);
	mt->powered = false;
	mt->test_pattern = false;
}

static int mt8193_power_on(struct mt8193 *mt)
{
	/* vmipi 1.8 V -> vrf12 1.2 V -> vibr 3.3 V -> vldo28 2.8 V. */
	static const unsigned int delays[] = { 4, 8, 4, 20 };
	unsigned int bus;
	int ret, i;

	if (mt->powered)
		return 0;
	for (i = 0; i < ARRAY_SIZE(mt->supplies); i++) {
		ret = regulator_enable(mt->supplies[i].consumer);
		if (ret) {
			while (i--)
				regulator_disable(mt->supplies[i].consumer);
			return ret;
		}
		msleep(delays[i]);
	}
	mt->powered = true;
	gpiod_set_value_cansleep(mt->reset, 0);
	msleep(20);
	ret = regmap_read(mt->map, 0x1000, &bus);
	if (ret)
		goto fail;
	if (bus == ~0U) {
		ret = -ENODEV;
		goto fail;
	}
	/* BSP HDMI digital domain power, reset, isolation, clock and analog. */
	ret = regmap_update_bits(mt->map, MT8193_PWR_CTRL, BIT(1), BIT(1));
	if (!ret)
		ret = regmap_update_bits(mt->map, MT8193_PWR_RESET, BIT(0), BIT(0));
	if (!ret)
		ret = regmap_update_bits(mt->map, MT8193_PWR_CTRL, BIT(0), 0);
	if (!ret)
		ret = regmap_update_bits(mt->map, MT8193_PWR_CTRL, BIT(2), 0);
	if (!ret)
		ret = regmap_update_bits(mt->map, MT8193_SYS_CFG, BIT(2) | BIT(0), BIT(2) | BIT(0));
	if (!ret)
		ret = regmap_write(mt->map, 0x1500, 0x20);
	if (!ret)
		ret = regmap_update_bits(mt->map, 0x1258, BIT(21), BIT(21));
	if (!ret)
		ret = regmap_write(mt->map, 0x18, 0xff);
	if (ret)
		goto fail;
	gpiod_set_value_cansleep(mt->enable, 1);
	msleep(100);
	dev_info(&mt->client->dev, "BSP power sequence complete, bus_ckcfg=%08x\n", bus);
	return 0;
fail:
	mt8193_power_off(mt);
	return ret;
}

/* Factory pattern from BSP mt8193_ckgen_vfy.c:mt8193_hdmi_on_test().
 * This isolates transmitter/TMDS from SoC DMA. It is a debugfs bring-up test,
 * not a framebuffer or a claim that the MT6797 DPI producer is operational.
 */
static int mt8193_pattern_set(void *data, u64 enable)
{
	struct mt8193 *mt = data;
	static const struct reg_sequence pattern[] = {
		{0x1254, 0x00000323}, {0x101c, 0x00000004},
		{0x1328, 0x00009999}, {0x1334, 0x0020008f},
		{0x1338, 0xd4a88f00}, {0x1344, 0x00008012},
		{0x1348, 0x11ff0000}, {0x1334, 0x0030008f},
		{0x02d4, 0x02}, {0x02d8, 0x80}, {0x0448, 0},
		{0x0450, 2}, {0x0608, 0x80000005}, {0x065c, 0xf},
		{0x0604, 0x40}, {0x061c, 0x00104000},
		{0x0624, 0x02ee07bc}, {0x0628, 0x00030005},
		{0x0630, 0x0080057f}, {0x0634, 0x001002df},
		{0x0638, 0x001002df}, {0x0620, 0x000207ba},
		{0x0600, 1}, {0x060c, 2}, {0x0700, 0x02ee07bc},
		{0x0704, 0x00030005}, {0x0708, 0x01000080},
		{0x070c, 0x02d00500}, {0x0710, 0x00000203},
		{0x0714, 0x00ff8844}, {0x0718, 0x000000ff},
	};
	int ret = 0, i;

	if (enable > 1)
		return -EINVAL;
	mutex_lock(&mt->lock);
	if (!enable) {
		mt8193_power_off(mt);
		ret = mt8193_power_on(mt);
		mt->test_pattern = false;
	} else if (!mt->powered) {
		ret = -EHOSTDOWN;
	} else {
		for (i = 0; i < ARRAY_SIZE(pattern); i++) {
			ret = regmap_write(mt->map, pattern[i].reg, pattern[i].def);
			if (ret)
				break;
			msleep(100);
		}
		mt->test_pattern = !ret;
		if (ret)
			mt8193_power_off(mt);
	}
	mutex_unlock(&mt->lock);
	return ret;
}

static int mt8193_pattern_get(void *data, u64 *value)
{
	struct mt8193 *mt = data;

	mutex_lock(&mt->lock);
	*value = mt->test_pattern;
	mutex_unlock(&mt->lock);
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(mt8193_pattern_fops, mt8193_pattern_get,
			 mt8193_pattern_set, "%llu\n");

static void mt8193_debugfs_remove(void *data)
{
	debugfs_remove_recursive(data);
}

static int mt8193_ddc_command(struct mt8193 *mt, unsigned int mode)
{
	unsigned int val;
	int ret;

	ret = regmap_update_bits(mt->map, MT8193_DDC_CTL1, DDC_MODE | DDC_TRIGGER,
				FIELD_PREP(DDC_MODE, mode) | DDC_TRIGGER);
	if (ret)
		return ret;
	return regmap_read_poll_timeout(mt->map, MT8193_DDC_CTL1, val,
					!(val & DDC_TRIGGER), 100, 20000);
}

static int mt8193_ddc_write(struct mt8193 *mt, u32 data, unsigned int len)
{
	unsigned int val;
	int ret;

	ret = regmap_write(mt->map, MT8193_DDC_DATA0, data);
	if (!ret)
		ret = regmap_update_bits(mt->map, MT8193_DDC_CTL1, DDC_LENGTH,
					FIELD_PREP(DDC_LENGTH, len - 1));
	if (!ret)
		ret = mt8193_ddc_command(mt, 2);
	if (!ret)
		ret = regmap_read(mt->map, MT8193_DDC_CTL1, &val);
	if (!ret && FIELD_GET(DDC_ACK, val) != BIT(len) - 1)
		ret = -ENXIO;
	return ret;
}

/* Lock held; all triggers are bounded and every transfer attempts a STOP.
 * The hardware transfers eight bytes per command, ACKing all but the last.
 */
static int mt8193_read_edid_block(void *data, u8 *buf, unsigned int block, size_t len)
{
	struct mt8193 *mt = data;
	unsigned int val, a, b, i;
	int ret, stop_ret;

	if (block > 3 || len != EDID_LENGTH)
		return -EINVAL;
	if (!mt->powered)
		return -EHOSTDOWN;
	ret = regmap_update_bits(mt->map, MT8193_DDC_CTL0,
		BIT(31) | GENMASK(27, 16) | BIT(1), (260 << 16) | BIT(1));
	if (ret)
		return ret;
	ret = regmap_read(mt->map, MT8193_DDC_CTL1, &val);
	if (ret)
		return ret;
	if (val & DDC_TRIGGER)
		return -EBUSY;
	ret = mt8193_ddc_command(mt, 1);
	if (!ret && block > 1) {
		ret = mt8193_ddc_write(mt, 0x60 | ((block / 2) << 8), 2);
		if (!ret)
			ret = mt8193_ddc_command(mt, 1);
	}
	if (!ret)
		ret = mt8193_ddc_write(mt, 0xa0 | ((block % 2) * 128 << 8), 2);
	if (!ret)
		ret = mt8193_ddc_command(mt, 1);
	if (!ret)
		ret = mt8193_ddc_write(mt, 0xa1, 1);
	for (i = 0; !ret && i < len; i += 8) {
		ret = regmap_update_bits(mt->map, MT8193_DDC_CTL1, DDC_LENGTH,
					FIELD_PREP(DDC_LENGTH, 7));
		if (!ret)
			ret = mt8193_ddc_command(mt, i + 8 == len ? 4 : 5);
		if (!ret)
			ret = regmap_read(mt->map, MT8193_DDC_CTL1, &val);
		if (!ret && FIELD_GET(DDC_ACK, val) != (i + 8 == len ? 0x7f : 0xff))
			ret = -ENXIO;
		if (!ret)
			ret = regmap_read(mt->map, MT8193_DDC_DATA0, &a);
		if (!ret)
			ret = regmap_read(mt->map, MT8193_DDC_DATA1, &b);
		if (!ret) {
			unsigned int j;

			for (j = 0; j < 4; j++) {
				buf[i + j] = a >> (j * 8);
				buf[i + j + 4] = b >> (j * 8);
			}
		}
	}
	stop_ret = mt8193_ddc_command(mt, 3);
	if (!ret)
		ret = stop_ret;
	if (ret)
		mt->ddc_errors++;
	else
		mt->ddc_blocks++;
	return ret;
}

static enum drm_connector_status mt8193_detect(struct drm_bridge *bridge,
					      struct drm_connector *connector)
{
	struct mt8193 *mt = container_of(bridge, struct mt8193, bridge);
	unsigned int status;
	int ret;

	mutex_lock(&mt->lock);
	ret = mt->powered ? regmap_read(mt->map, MT8193_STATUS, &status) : -EHOSTDOWN;
	mutex_unlock(&mt->lock);
	if (ret)
		return connector_status_unknown;
	return status & BIT(0) ? connector_status_connected : connector_status_disconnected;
}

static const struct drm_edid *mt8193_edid_read(struct drm_bridge *bridge,
					      struct drm_connector *connector)
{
	struct mt8193 *mt = container_of(bridge, struct mt8193, bridge);
	const struct drm_edid *edid;

	mutex_lock(&mt->lock);
	edid = drm_edid_read_custom(connector, mt8193_read_edid_block, mt);
	mutex_unlock(&mt->lock);
	return edid;
}

static const struct drm_bridge_funcs mt8193_bridge_funcs = {
	.detect = mt8193_detect,
	.edid_read = mt8193_edid_read,
};

static ssize_t link_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mt8193 *mt = dev_get_drvdata(dev);
	unsigned int status = 0, power = 0;
	int ret;

	mutex_lock(&mt->lock);
	ret = mt->powered ? regmap_read(mt->map, MT8193_STATUS, &status) : -EHOSTDOWN;
	if (!ret)
		ret = regmap_read(mt->map, MT8193_PWR_CTRL, &power);
	if (!ret)
		ret = sysfs_emit(buf, "powered=%u hpd=%u pord=%u status=%08x power=%08x ddc_blocks=%llu ddc_errors=%llu\n",
			mt->powered, !!(status & BIT(0)), !!(status & BIT(1)), status, power,
			mt->ddc_blocks, mt->ddc_errors);
	mutex_unlock(&mt->lock);
	return ret;
}
static DEVICE_ATTR_RO(link_status);

static ssize_t edid_read(struct file *file, struct kobject *kobj,
			const struct bin_attribute *attr, char *buf, loff_t off, size_t count)
{
	struct mt8193 *mt = dev_get_drvdata(kobj_to_dev(kobj));
	u8 data[EDID_LENGTH];
	unsigned int i, block = off / EDID_LENGTH;
	u8 sum = 0;
	int ret;

	/* One block per read; sysfs will call again for subsequent blocks. */
	count = min_t(size_t, count, EDID_LENGTH - off % EDID_LENGTH);
	mutex_lock(&mt->lock);
	ret = mt8193_read_edid_block(mt, data, block, sizeof(data));
	mutex_unlock(&mt->lock);
	if (ret)
		return ret;
	for (i = 0; i < EDID_LENGTH; i++)
		sum += data[i];
	if (sum)
		return -EBADMSG;
	memcpy(buf, data + off % EDID_LENGTH, count);
	return count;
}
static BIN_ATTR_RO(edid, 512);

static struct attribute *mt8193_attrs[] = { &dev_attr_link_status.attr, NULL };
static const struct bin_attribute * const mt8193_bin_attrs[] = { &bin_attr_edid, NULL };
static const struct attribute_group mt8193_group = {
	.attrs = mt8193_attrs, .bin_attrs = mt8193_bin_attrs,
};

static void mt8193_cleanup(void *data)
{
	mt8193_power_off(data);
}

static int mt8193_probe(struct i2c_client *client)
{
	static const char * const names[] = { "vmipi", "vrf12", "vibr", "vldo28" };
	struct device *dev = &client->dev;
	struct mt8193 *mt;
	struct dentry *debugfs;
	int ret, i;

	mt = devm_drm_bridge_alloc(dev, struct mt8193, bridge, &mt8193_bridge_funcs);
	if (IS_ERR(mt))
		return PTR_ERR(mt);
	mt->client = client;
	mutex_init(&mt->lock);
	i2c_set_clientdata(client, mt);
	mt->map = devm_regmap_init(dev, NULL, client, &mt8193_regmap_config);
	if (IS_ERR(mt->map))
		return PTR_ERR(mt->map);
	for (i = 0; i < ARRAY_SIZE(names); i++)
		mt->supplies[i].supply = names[i];
	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(names), mt->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "supplies\n");
	mt->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(mt->reset))
		return dev_err_probe(dev, PTR_ERR(mt->reset), "reset GPIO\n");
	mt->enable = devm_gpiod_get(dev, "enable", GPIOD_OUT_LOW);
	if (IS_ERR(mt->enable))
		return dev_err_probe(dev, PTR_ERR(mt->enable), "5V GPIO\n");
	ret = mt8193_power_on(mt);
	if (ret)
		return dev_err_probe(dev, ret, "power/control access\n");
	ret = devm_add_action_or_reset(dev, mt8193_cleanup, mt);
	if (ret)
		return ret;
	ret = devm_device_add_group(dev, &mt8193_group);
	if (ret)
		return ret;
	debugfs = debugfs_create_dir(dev_name(dev), NULL);
	debugfs_create_file("bsp-pattern-720p50", 0600, debugfs, mt, &mt8193_pattern_fops);
	ret = devm_add_action_or_reset(dev, mt8193_debugfs_remove, debugfs);
	if (ret)
		return ret;
	mt->bridge.of_node = dev->of_node;
	mt->bridge.type = DRM_MODE_CONNECTOR_HDMIA;
	mt->bridge.ops = DRM_BRIDGE_OP_DETECT | DRM_BRIDGE_OP_EDID;
	return devm_drm_bridge_add(dev, &mt->bridge);
}

static int mt8193_suspend(struct device *dev)
{
	struct mt8193 *mt = dev_get_drvdata(dev);

	mutex_lock(&mt->lock);
	mt8193_power_off(mt);
	mutex_unlock(&mt->lock);
	return 0;
}

static int mt8193_resume(struct device *dev)
{
	struct mt8193 *mt = dev_get_drvdata(dev);
	int ret;

	mutex_lock(&mt->lock);
	ret = mt8193_power_on(mt);
	mutex_unlock(&mt->lock);
	return ret;
}
static DEFINE_SIMPLE_DEV_PM_OPS(mt8193_pm, mt8193_suspend, mt8193_resume);

static const struct of_device_id mt8193_match[] = {
	{ .compatible = "mediatek,mt8193" }, { }
};
MODULE_DEVICE_TABLE(of, mt8193_match);
static struct i2c_driver mt8193_driver = {
	.probe = mt8193_probe,
	.driver = {
		.name = "mt8193", .of_match_table = mt8193_match,
		.pm = pm_sleep_ptr(&mt8193_pm),
	},
};
module_i2c_driver(mt8193_driver);
MODULE_DESCRIPTION("MediaTek MT8193 HDMI control and DDC");
MODULE_LICENSE("GPL");
