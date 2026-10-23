// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2015 MediaTek Inc.
 * Author: Hanyi Wu <hanyi.wu@mediatek.com>
 *         Sascha Hauer <s.hauer@pengutronix.de>
 *         Dawei Chien <dawei.chien@mediatek.com>
 *         Louis Yu <louis.yu@mediatek.com>
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/nvmem-consumer.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/thermal.h>
#include <linux/reset.h>
#include <linux/soc/mediatek/mt6797-thermal-eem.h>
#include <linux/soc/mediatek/mt6797-wdt.h>
#include <linux/reboot.h>
#include <linux/ktime.h>
#include <linux/types.h>

#include "../thermal_hwmon.h"

/* AUXADC Registers */
#define AUXADC_CON1_SET_V	0x008
#define AUXADC_CON1_CLR_V	0x00c
#define AUXADC_CON0_V		0x000
#define AUXADC_CON2_V		0x010
#define AUXADC_DATA(channel)	(0x14 + (channel) * 4)

#define APMIXED_SYS_TS_CON0	0x600
#define APMIXED_SYS_TS_CON1	0x604

/* Thermal Controller Registers */
#define TEMP_MONCTL0		0x000
#define TEMP_MONCTL1		0x004
#define TEMP_MONCTL2		0x008
#define TEMP_MONIDET0		0x014
#define TEMP_MONIDET1		0x018
#define TEMP_MSRCTL0		0x038
#define TEMP_MSRCTL1		0x03c
#define TEMP_AHBPOLL		0x040
#define TEMP_AHBTO		0x044
#define TEMP_ADCPNP0		0x048
#define TEMP_ADCPNP1		0x04c
#define TEMP_ADCPNP2		0x050
#define TEMP_ADCPNP3		0x0b4

#define TEMP_ADCMUX		0x054
#define TEMP_ADCEN		0x060
#define TEMP_PNPMUXADDR		0x064
#define TEMP_ADCMUXADDR		0x068
#define TEMP_ADCENADDR		0x074
#define TEMP_ADCVALIDADDR	0x078
#define TEMP_ADCVOLTADDR	0x07c
#define TEMP_RDCTRL		0x080
#define TEMP_ADCVALIDMASK	0x084
#define TEMP_ADCVOLTAGESHIFT	0x088
#define TEMP_ADCWRITECTRL	0x08c
#define TEMP_MSR0		0x090
#define TEMP_MSR1		0x094
#define TEMP_MSR2		0x098
#define TEMP_MSR3		0x0B8

#define TEMP_SPARE0		0x0f0

#define TEMP_ADCPNP0_1          0x148
#define TEMP_ADCPNP1_1          0x14c
#define TEMP_ADCPNP2_1          0x150
#define TEMP_MSR0_1             0x190
#define TEMP_MSR1_1             0x194
#define TEMP_MSR2_1             0x198
#define TEMP_ADCPNP3_1          0x1b4
#define TEMP_MSR3_1             0x1B8

#define PTPCORESEL		0x400

/* This board's PL/LK leave the Linux console offsets zero and use only
 * [0,0x140).  Never claim the tail if a different boot chain owns it.
 * Double-buffer each bank: frequent OTP reads must not evict other banks.
 */
#define MT6797_TRACE_MAGIC	0x32544358 /* XCT2 */
#define MT6797_TRACE_RECORDS	12
#define MT6797_TRACE_WORDS	16
#define MT6797_TRACE_OFFSET	0x800
#define MT6797_TRACE_SIZE	(64 + MT6797_TRACE_RECORDS * 64)
#define MT6797_TRACE_ARCHIVE_OFFSET 0x400

#define TEMP_MONCTL1_PERIOD_UNIT(x)	((x) & 0x3ff)

#define TEMP_MONCTL2_FILTER_INTERVAL(x)	(((x) & 0x3ff) << 16)
#define TEMP_MONCTL2_SENSOR_INTERVAL(x)	((x) & 0x3ff)

#define TEMP_AHBPOLL_ADC_POLL_INTERVAL(x)	(x)

#define TEMP_ADCWRITECTRL_ADC_PNP_WRITE		BIT(0)
#define TEMP_ADCWRITECTRL_ADC_MUX_WRITE		BIT(1)

#define TEMP_ADCVALIDMASK_VALID_HIGH		BIT(5)
#define TEMP_ADCVALIDMASK_VALID_POS(bit)	(bit)

/* MT8173 thermal sensors */
#define MT8173_TS1	0
#define MT8173_TS2	1
#define MT8173_TS3	2
#define MT8173_TS4	3
#define MT8173_TSABB	4

/* AUXADC channel 11 is used for the temperature sensors */
#define MT8173_TEMP_AUXADC_CHANNEL	11

/* The total number of temperature sensors in the MT8173 */
#define MT8173_NUM_SENSORS		5

/* The number of banks in the MT8173 */
#define MT8173_NUM_ZONES		4

/* The number of sensing points per bank */
#define MT8173_NUM_SENSORS_PER_ZONE	4

/* The number of controller in the MT8173 */
#define MT8173_NUM_CONTROLLER		1

/* The calibration coefficient of sensor  */
#define MT8173_CALIBRATION	165

/* Valid temperatures range */
#define MT8173_TEMP_MIN		-20000
#define MT8173_TEMP_MAX		150000
/* Temporary upper range for MT6797 thermal-limit characterization. */
#define MT6797_TEMP_MAX_TEST	180000

/*
 * Layout of the fuses providing the calibration data
 * These macros could be used for MT8183, MT8173, MT2701, and MT2712.
 * MT8183 has 6 sensors and needs 6 VTS calibration data.
 * MT8173 has 5 sensors and needs 5 VTS calibration data.
 * MT2701 has 3 sensors and needs 3 VTS calibration data.
 * MT2712 has 4 sensors and needs 4 VTS calibration data.
 */
#define CALIB_BUF0_VALID_V1		BIT(0)
#define CALIB_BUF1_ADC_GE_V1(x)		(((x) >> 22) & 0x3ff)
#define CALIB_BUF0_VTS_TS1_V1(x)	(((x) >> 17) & 0x1ff)
#define CALIB_BUF0_VTS_TS2_V1(x)	(((x) >> 8) & 0x1ff)
#define CALIB_BUF1_VTS_TS3_V1(x)	(((x) >> 0) & 0x1ff)
#define CALIB_BUF2_VTS_TS4_V1(x)	(((x) >> 23) & 0x1ff)
#define CALIB_BUF2_VTS_TS5_V1(x)	(((x) >> 5) & 0x1ff)
#define CALIB_BUF2_VTS_TSABB_V1(x)	(((x) >> 14) & 0x1ff)
#define CALIB_BUF0_DEGC_CALI_V1(x)	(((x) >> 1) & 0x3f)
#define CALIB_BUF0_O_SLOPE_V1(x)	(((x) >> 26) & 0x3f)
#define CALIB_BUF0_O_SLOPE_SIGN_V1(x)	(((x) >> 7) & 0x1)
#define CALIB_BUF1_ID_V1(x)		(((x) >> 9) & 0x1)

/*
 * Layout of the fuses providing the calibration data
 * These macros could be used for MT7622.
 */
#define CALIB_BUF0_ADC_OE_V2(x)		(((x) >> 22) & 0x3ff)
#define CALIB_BUF0_ADC_GE_V2(x)		(((x) >> 12) & 0x3ff)
#define CALIB_BUF0_DEGC_CALI_V2(x)	(((x) >> 6) & 0x3f)
#define CALIB_BUF0_O_SLOPE_V2(x)	(((x) >> 0) & 0x3f)
#define CALIB_BUF1_VTS_TS1_V2(x)	(((x) >> 23) & 0x1ff)
#define CALIB_BUF1_VTS_TS2_V2(x)	(((x) >> 14) & 0x1ff)
#define CALIB_BUF1_VTS_TSABB_V2(x)	(((x) >> 5) & 0x1ff)
#define CALIB_BUF1_VALID_V2(x)		(((x) >> 4) & 0x1)
#define CALIB_BUF1_O_SLOPE_SIGN_V2(x)	(((x) >> 3) & 0x1)

/*
 * Layout of the fuses providing the calibration data
 * These macros can be used for MT7981 and MT7986.
 */
#define CALIB_BUF0_ADC_GE_V3(x)		(((x) >> 0) & 0x3ff)
#define CALIB_BUF0_DEGC_CALI_V3(x)	(((x) >> 20) & 0x3f)
#define CALIB_BUF0_O_SLOPE_V3(x)	(((x) >> 26) & 0x3f)
#define CALIB_BUF1_VTS_TS1_V3(x)	(((x) >> 0) & 0x1ff)
#define CALIB_BUF1_VTS_TS2_V3(x)	(((x) >> 21) & 0x1ff)
#define CALIB_BUF1_VTS_TSABB_V3(x)	(((x) >> 9) & 0x1ff)
#define CALIB_BUF1_VALID_V3(x)		(((x) >> 18) & 0x1)
#define CALIB_BUF1_O_SLOPE_SIGN_V3(x)	(((x) >> 19) & 0x1)
#define CALIB_BUF1_ID_V3(x)		(((x) >> 20) & 0x1)

enum {
	VTS1,
	VTS2,
	VTS3,
	VTS4,
	VTS5,
	VTSABB,
	MAX_NUM_VTS,
};

enum mtk_thermal_version {
	MTK_THERMAL_V1 = 1,
	MTK_THERMAL_V2,
	MTK_THERMAL_V3,
	MTK_THERMAL_MT6797,
};

/* MT2701 thermal sensors */
#define MT2701_TS1	0
#define MT2701_TS2	1
#define MT2701_TSABB	2

/* AUXADC channel 11 is used for the temperature sensors */
#define MT2701_TEMP_AUXADC_CHANNEL	11

/* The total number of temperature sensors in the MT2701 */
#define MT2701_NUM_SENSORS	3

/* The number of sensing points per bank */
#define MT2701_NUM_SENSORS_PER_ZONE	3

/* The number of controller in the MT2701 */
#define MT2701_NUM_CONTROLLER		1

/* The calibration coefficient of sensor  */
#define MT2701_CALIBRATION	165

/* MT2712 thermal sensors */
#define MT2712_TS1	0
#define MT2712_TS2	1
#define MT2712_TS3	2
#define MT2712_TS4	3

/* AUXADC channel 11 is used for the temperature sensors */
#define MT2712_TEMP_AUXADC_CHANNEL	11

/* The total number of temperature sensors in the MT2712 */
#define MT2712_NUM_SENSORS	4

/* The number of sensing points per bank */
#define MT2712_NUM_SENSORS_PER_ZONE	4

/* The number of controller in the MT2712 */
#define MT2712_NUM_CONTROLLER		1

/* The calibration coefficient of sensor  */
#define MT2712_CALIBRATION	165

#define MT7622_TEMP_AUXADC_CHANNEL	11
#define MT7622_NUM_SENSORS		1
#define MT7622_NUM_ZONES		1
#define MT7622_NUM_SENSORS_PER_ZONE	1
#define MT7622_TS1	0
#define MT7622_NUM_CONTROLLER		1

/* The maximum number of banks */
#define MAX_NUM_ZONES		8

/* The calibration coefficient of sensor  */
#define MT7622_CALIBRATION	165

/* MT8183 thermal sensors */
#define MT8183_TS1	0
#define MT8183_TS2	1
#define MT8183_TS3	2
#define MT8183_TS4	3
#define MT8183_TS5	4
#define MT8183_TSABB	5

/* AUXADC channel  is used for the temperature sensors */
#define MT8183_TEMP_AUXADC_CHANNEL	11

/* The total number of temperature sensors in the MT8183 */
#define MT8183_NUM_SENSORS	6

/* The number of banks in the MT8183 */
#define MT8183_NUM_ZONES               1

/* The number of sensing points per bank */
#define MT8183_NUM_SENSORS_PER_ZONE	 6

/* The number of controller in the MT8183 */
#define MT8183_NUM_CONTROLLER		2

/* The calibration coefficient of sensor  */
#define MT8183_CALIBRATION	153

/* AUXADC channel 11 is used for the temperature sensors */
#define MT7986_TEMP_AUXADC_CHANNEL	11

/* The total number of temperature sensors in the MT7986 */
#define MT7986_NUM_SENSORS		1

/* The number of banks in the MT7986 */
#define MT7986_NUM_ZONES		1

/* The number of sensing points per bank */
#define MT7986_NUM_SENSORS_PER_ZONE	1

/* MT7986 thermal sensors */
#define MT7986_TS1			0

/* The number of controller in the MT7986 */
#define MT7986_NUM_CONTROLLER		1

/* The calibration coefficient of sensor  */
#define MT7986_CALIBRATION		165

/* MT8365 */
#define MT8365_TEMP_AUXADC_CHANNEL 11
#define MT8365_CALIBRATION 164
#define MT8365_NUM_CONTROLLER 1
#define MT8365_NUM_BANKS 1
#define MT8365_NUM_SENSORS 3
#define MT8365_NUM_SENSORS_PER_ZONE 3
#define MT8365_TS1 0
#define MT8365_TS2 1
#define MT8365_TS3 2

struct mtk_thermal;

struct thermal_bank_cfg {
	unsigned int num_sensors;
	const int *sensors;
};

struct mtk_thermal_bank {
	struct mtk_thermal *mt;
	int id;
};

DEFINE_SPINLOCK(mt6797_thermal_eem_lock);
EXPORT_SYMBOL_GPL(mt6797_thermal_eem_lock);

struct mtk_thermal_data {
	s32 num_banks;
	s32 num_sensors;
	s32 auxadc_channel;
	const int *vts_index;
	const int *sensor_mux_values;
	const int *msr;
	const int *adcpnp;
	const int cali_val;
	const int num_controller;
	const int *controller_offset;
	bool need_switch_bank;
	struct thermal_bank_cfg bank_data[MAX_NUM_ZONES];
	enum mtk_thermal_version version;
	u32 apmixed_buffer_ctl_reg;
	u32 apmixed_buffer_ctl_mask;
	u32 apmixed_buffer_ctl_set;
};

struct mtk_thermal {
	struct device *dev;
	void __iomem *thermal_base;
	phys_addr_t thermal_phys;
	struct device *hw_watchdog;
	struct mutex hw_protect_lock;
	int hw_critical_temp;
	void __iomem *auxadc_base, *apmixed_base;
	u32 auxadc_phys, apmixed_phys;
	u32 pm_suspends, pm_resumes, pm_failures;
	int pm_last_error;
	bool pm_suspended;

	struct clk *clk_peri_therm;
	struct clk *clk_auxadc;
	/* lock: for getting and putting banks */
	struct mutex lock;
	/* Protected by the shared thermal/EEM bank spinlock. */
	u64 bank_transactions, bank_hold_max_ns;
	u64 bank_disarm_failures, bank_rearm_failures, bank_restore_retries;
	void __iomem *tc_trace;
	u32 tc_trace_previous[MT6797_TRACE_RECORDS][MT6797_TRACE_WORDS];
	u32 tc_trace_failure[MT6797_TRACE_RECORDS][MT6797_TRACE_WORDS];
	u32 tc_trace_sequence[6], tc_trace_generation, tc_trace_previous_generation;
	u32 tc_trace_failure_generation, tc_trace_failure_reason;
	u64 tc_trace_last_ns[6];
	bool tc_trace_previous_valid;
	bool tc_trace_failure_valid;

	/* Calibration values */
	s32 adc_ge;
	s32 adc_oe;
	s32 degc_cali;
	s32 o_slope;
	s32 o_slope_sign;
	s32 vts[MAX_NUM_VTS];

	const struct mtk_thermal_data *conf;
	struct mtk_thermal_bank banks[MAX_NUM_ZONES];

	int (*raw_to_mcelsius)(struct mtk_thermal *mt, int sensno, s32 raw);
};

/* MT8183 thermal sensor data */
static const int mt8183_bank_data[MT8183_NUM_SENSORS] = {
	MT8183_TS1, MT8183_TS2, MT8183_TS3, MT8183_TS4, MT8183_TS5, MT8183_TSABB
};

static const int mt8183_msr[MT8183_NUM_SENSORS_PER_ZONE] = {
	TEMP_MSR0_1, TEMP_MSR1_1, TEMP_MSR2_1, TEMP_MSR1, TEMP_MSR0, TEMP_MSR3_1
};

static const int mt8183_adcpnp[MT8183_NUM_SENSORS_PER_ZONE] = {
	TEMP_ADCPNP0_1, TEMP_ADCPNP1_1, TEMP_ADCPNP2_1,
	TEMP_ADCPNP1, TEMP_ADCPNP0, TEMP_ADCPNP3_1
};

static const int mt8183_mux_values[MT8183_NUM_SENSORS] = { 0, 1, 2, 3, 4, 0 };
static const int mt8183_tc_offset[MT8183_NUM_CONTROLLER] = {0x0, 0x100};

static const int mt8183_vts_index[MT8183_NUM_SENSORS] = {
	VTS1, VTS2, VTS3, VTS4, VTS5, VTSABB
};

/* MT8173 thermal sensor data */
static const int mt8173_bank_data[MT8173_NUM_ZONES][3] = {
	{ MT8173_TS2, MT8173_TS3 },
	{ MT8173_TS2, MT8173_TS4 },
	{ MT8173_TS1, MT8173_TS2, MT8173_TSABB },
	{ MT8173_TS2 },
};

static const int mt8173_msr[MT8173_NUM_SENSORS_PER_ZONE] = {
	TEMP_MSR0, TEMP_MSR1, TEMP_MSR2, TEMP_MSR3
};

static const int mt8173_adcpnp[MT8173_NUM_SENSORS_PER_ZONE] = {
	TEMP_ADCPNP0, TEMP_ADCPNP1, TEMP_ADCPNP2, TEMP_ADCPNP3
};

static const int mt8173_mux_values[MT8173_NUM_SENSORS] = { 0, 1, 2, 3, 16 };
static const int mt8173_tc_offset[MT8173_NUM_CONTROLLER] = { 0x0, };

static const int mt8173_vts_index[MT8173_NUM_SENSORS] = {
	VTS1, VTS2, VTS3, VTS4, VTSABB
};

/* BSP bank order: BIG, GPU, SOC, CPU-L, CPU-LL, CCI. */
static const int mt6797_bank_data[][2] = {
	{ 0 }, { 3 }, { 1, 2 }, { 1 }, { 1 }, { 1 },
};

static const int mt6797_vts_index[] = {
	VTS1, VTS2, VTS3, VTS4, VTSABB,
};

static const int mt6797_tc_offset[] = { 0x0 };

/* MT2701 thermal sensor data */
static const int mt2701_bank_data[MT2701_NUM_SENSORS] = {
	MT2701_TS1, MT2701_TS2, MT2701_TSABB
};

static const int mt2701_msr[MT2701_NUM_SENSORS_PER_ZONE] = {
	TEMP_MSR0, TEMP_MSR1, TEMP_MSR2
};

static const int mt2701_adcpnp[MT2701_NUM_SENSORS_PER_ZONE] = {
	TEMP_ADCPNP0, TEMP_ADCPNP1, TEMP_ADCPNP2
};

static const int mt2701_mux_values[MT2701_NUM_SENSORS] = { 0, 1, 16 };
static const int mt2701_tc_offset[MT2701_NUM_CONTROLLER] = { 0x0, };

static const int mt2701_vts_index[MT2701_NUM_SENSORS] = {
	VTS1, VTS2, VTS3
};

/* MT2712 thermal sensor data */
static const int mt2712_bank_data[MT2712_NUM_SENSORS] = {
	MT2712_TS1, MT2712_TS2, MT2712_TS3, MT2712_TS4
};

static const int mt2712_msr[MT2712_NUM_SENSORS_PER_ZONE] = {
	TEMP_MSR0, TEMP_MSR1, TEMP_MSR2, TEMP_MSR3
};

static const int mt2712_adcpnp[MT2712_NUM_SENSORS_PER_ZONE] = {
	TEMP_ADCPNP0, TEMP_ADCPNP1, TEMP_ADCPNP2, TEMP_ADCPNP3
};

static const int mt2712_mux_values[MT2712_NUM_SENSORS] = { 0, 1, 2, 3 };
static const int mt2712_tc_offset[MT2712_NUM_CONTROLLER] = { 0x0, };

static const int mt2712_vts_index[MT2712_NUM_SENSORS] = {
	VTS1, VTS2, VTS3, VTS4
};

/* MT7622 thermal sensor data */
static const int mt7622_bank_data[MT7622_NUM_SENSORS] = { MT7622_TS1, };
static const int mt7622_msr[MT7622_NUM_SENSORS_PER_ZONE] = { TEMP_MSR0, };
static const int mt7622_adcpnp[MT7622_NUM_SENSORS_PER_ZONE] = { TEMP_ADCPNP0, };
static const int mt7622_mux_values[MT7622_NUM_SENSORS] = { 0, };
static const int mt7622_vts_index[MT7622_NUM_SENSORS] = { VTS1 };
static const int mt7622_tc_offset[MT7622_NUM_CONTROLLER] = { 0x0, };

/* MT7986 thermal sensor data */
static const int mt7986_bank_data[MT7986_NUM_SENSORS] = { MT7986_TS1, };
static const int mt7986_msr[MT7986_NUM_SENSORS_PER_ZONE] = { TEMP_MSR0, };
static const int mt7986_adcpnp[MT7986_NUM_SENSORS_PER_ZONE] = { TEMP_ADCPNP0, };
static const int mt7986_mux_values[MT7986_NUM_SENSORS] = { 0, };
static const int mt7986_vts_index[MT7986_NUM_SENSORS] = { VTS1 };
static const int mt7986_tc_offset[MT7986_NUM_CONTROLLER] = { 0x0, };

/* MT8365 thermal sensor data */
static const int mt8365_bank_data[MT8365_NUM_SENSORS] = {
	MT8365_TS1, MT8365_TS2, MT8365_TS3
};

static const int mt8365_msr[MT8365_NUM_SENSORS_PER_ZONE] = {
	TEMP_MSR0, TEMP_MSR1, TEMP_MSR2
};

static const int mt8365_adcpnp[MT8365_NUM_SENSORS_PER_ZONE] = {
	TEMP_ADCPNP0, TEMP_ADCPNP1, TEMP_ADCPNP2
};

static const int mt8365_mux_values[MT8365_NUM_SENSORS] = { 0, 1, 2 };
static const int mt8365_tc_offset[MT8365_NUM_CONTROLLER] = { 0 };

static const int mt8365_vts_index[MT8365_NUM_SENSORS] = { VTS1, VTS2, VTS3 };

/*
 * The MT8173 thermal controller has four banks. Each bank can read up to
 * four temperature sensors simultaneously. The MT8173 has a total of 5
 * temperature sensors. We use each bank to measure a certain area of the
 * SoC. Since TS2 is located centrally in the SoC it is influenced by multiple
 * areas, hence is used in different banks.
 *
 * The thermal core only gets the maximum temperature of all banks, so
 * the bank concept wouldn't be necessary here. However, the SVS (Smart
 * Voltage Scaling) unit makes its decisions based on the same bank
 * data, and this indeed needs the temperatures of the individual banks
 * for making better decisions.
 */
static const struct mtk_thermal_data mt8173_thermal_data = {
	.auxadc_channel = MT8173_TEMP_AUXADC_CHANNEL,
	.num_banks = MT8173_NUM_ZONES,
	.num_sensors = MT8173_NUM_SENSORS,
	.vts_index = mt8173_vts_index,
	.cali_val = MT8173_CALIBRATION,
	.num_controller = MT8173_NUM_CONTROLLER,
	.controller_offset = mt8173_tc_offset,
	.need_switch_bank = true,
	.bank_data = {
		{
			.num_sensors = 2,
			.sensors = mt8173_bank_data[0],
		}, {
			.num_sensors = 2,
			.sensors = mt8173_bank_data[1],
		}, {
			.num_sensors = 3,
			.sensors = mt8173_bank_data[2],
		}, {
			.num_sensors = 1,
			.sensors = mt8173_bank_data[3],
		},
	},
	.msr = mt8173_msr,
	.adcpnp = mt8173_adcpnp,
	.sensor_mux_values = mt8173_mux_values,
	.version = MTK_THERMAL_V1,
};

static const struct mtk_thermal_data mt6797_thermal_data = {
	.auxadc_channel = 11,
	.num_banks = ARRAY_SIZE(mt6797_bank_data),
	.num_sensors = ARRAY_SIZE(mt6797_vts_index),
	.vts_index = mt6797_vts_index,
	.num_controller = 1,
	.controller_offset = mt6797_tc_offset,
	.need_switch_bank = true,
	.bank_data = {
		{ .num_sensors = 1, .sensors = mt6797_bank_data[0] },
		{ .num_sensors = 1, .sensors = mt6797_bank_data[1] },
		{ .num_sensors = 2, .sensors = mt6797_bank_data[2] },
		{ .num_sensors = 1, .sensors = mt6797_bank_data[3] },
		{ .num_sensors = 1, .sensors = mt6797_bank_data[4] },
		{ .num_sensors = 1, .sensors = mt6797_bank_data[5] },
	},
	.msr = mt8173_msr,
	.adcpnp = mt8173_adcpnp,
	.sensor_mux_values = mt8173_mux_values,
	.version = MTK_THERMAL_MT6797,
	/* BSP TS_CONFIGURE/TS_TURN_ON: power the analog path before the
	 * first real AUXADC conversion, with the existing 200us settling
	 * delay in mtk_thermal_turn_on_buffer().  PNP writes alone do not
	 * provide that interval before starting conversions.
	 */
	.apmixed_buffer_ctl_reg = APMIXED_SYS_TS_CON1,
	.apmixed_buffer_ctl_mask = (u32)~GENMASK(5, 4),
	.apmixed_buffer_ctl_set = 0,
};

/*
 * The MT2701 thermal controller has one bank, which can read up to
 * three temperature sensors simultaneously. The MT2701 has a total of 3
 * temperature sensors.
 *
 * The thermal core only gets the maximum temperature of this one bank,
 * so the bank concept wouldn't be necessary here. However, the SVS (Smart
 * Voltage Scaling) unit makes its decisions based on the same bank
 * data.
 */
static const struct mtk_thermal_data mt2701_thermal_data = {
	.auxadc_channel = MT2701_TEMP_AUXADC_CHANNEL,
	.num_banks = 1,
	.num_sensors = MT2701_NUM_SENSORS,
	.vts_index = mt2701_vts_index,
	.cali_val = MT2701_CALIBRATION,
	.num_controller = MT2701_NUM_CONTROLLER,
	.controller_offset = mt2701_tc_offset,
	.need_switch_bank = true,
	.bank_data = {
		{
			.num_sensors = 3,
			.sensors = mt2701_bank_data,
		},
	},
	.msr = mt2701_msr,
	.adcpnp = mt2701_adcpnp,
	.sensor_mux_values = mt2701_mux_values,
	.version = MTK_THERMAL_V1,
};

/*
 * The MT8365 thermal controller has one bank, which can read up to
 * four temperature sensors simultaneously. The MT8365 has a total of 3
 * temperature sensors.
 *
 * The thermal core only gets the maximum temperature of this one bank,
 * so the bank concept wouldn't be necessary here. However, the SVS (Smart
 * Voltage Scaling) unit makes its decisions based on the same bank
 * data.
 */
static const struct mtk_thermal_data mt8365_thermal_data = {
	.auxadc_channel = MT8365_TEMP_AUXADC_CHANNEL,
	.num_banks = MT8365_NUM_BANKS,
	.num_sensors = MT8365_NUM_SENSORS,
	.vts_index = mt8365_vts_index,
	.cali_val = MT8365_CALIBRATION,
	.num_controller = MT8365_NUM_CONTROLLER,
	.controller_offset = mt8365_tc_offset,
	.need_switch_bank = false,
	.bank_data = {
		{
			.num_sensors = MT8365_NUM_SENSORS,
			.sensors = mt8365_bank_data
		},
	},
	.msr = mt8365_msr,
	.adcpnp = mt8365_adcpnp,
	.sensor_mux_values = mt8365_mux_values,
	.version = MTK_THERMAL_V1,
	.apmixed_buffer_ctl_reg = APMIXED_SYS_TS_CON0,
	.apmixed_buffer_ctl_mask = (u32) ~GENMASK(29, 28),
	.apmixed_buffer_ctl_set = 0,
};

/*
 * The MT2712 thermal controller has one bank, which can read up to
 * four temperature sensors simultaneously. The MT2712 has a total of 4
 * temperature sensors.
 *
 * The thermal core only gets the maximum temperature of this one bank,
 * so the bank concept wouldn't be necessary here. However, the SVS (Smart
 * Voltage Scaling) unit makes its decisions based on the same bank
 * data.
 */
static const struct mtk_thermal_data mt2712_thermal_data = {
	.auxadc_channel = MT2712_TEMP_AUXADC_CHANNEL,
	.num_banks = 1,
	.num_sensors = MT2712_NUM_SENSORS,
	.vts_index = mt2712_vts_index,
	.cali_val = MT2712_CALIBRATION,
	.num_controller = MT2712_NUM_CONTROLLER,
	.controller_offset = mt2712_tc_offset,
	.need_switch_bank = true,
	.bank_data = {
		{
			.num_sensors = 4,
			.sensors = mt2712_bank_data,
		},
	},
	.msr = mt2712_msr,
	.adcpnp = mt2712_adcpnp,
	.sensor_mux_values = mt2712_mux_values,
	.version = MTK_THERMAL_V1,
};

/*
 * MT7622 have only one sensing point which uses AUXADC Channel 11 for raw data
 * access.
 */
static const struct mtk_thermal_data mt7622_thermal_data = {
	.auxadc_channel = MT7622_TEMP_AUXADC_CHANNEL,
	.num_banks = MT7622_NUM_ZONES,
	.num_sensors = MT7622_NUM_SENSORS,
	.vts_index = mt7622_vts_index,
	.cali_val = MT7622_CALIBRATION,
	.num_controller = MT7622_NUM_CONTROLLER,
	.controller_offset = mt7622_tc_offset,
	.need_switch_bank = true,
	.bank_data = {
		{
			.num_sensors = 1,
			.sensors = mt7622_bank_data,
		},
	},
	.msr = mt7622_msr,
	.adcpnp = mt7622_adcpnp,
	.sensor_mux_values = mt7622_mux_values,
	.version = MTK_THERMAL_V2,
	.apmixed_buffer_ctl_reg = APMIXED_SYS_TS_CON1,
	.apmixed_buffer_ctl_mask = GENMASK(31, 6) | BIT(3),
	.apmixed_buffer_ctl_set = BIT(0),
};

/*
 * The MT8183 thermal controller has one bank for the current SW framework.
 * The MT8183 has a total of 6 temperature sensors.
 * There are two thermal controller to control the six sensor.
 * The first one bind 2 sensor, and the other bind 4 sensors.
 * The thermal core only gets the maximum temperature of all sensor, so
 * the bank concept wouldn't be necessary here. However, the SVS (Smart
 * Voltage Scaling) unit makes its decisions based on the same bank
 * data, and this indeed needs the temperatures of the individual banks
 * for making better decisions.
 */
static const struct mtk_thermal_data mt8183_thermal_data = {
	.auxadc_channel = MT8183_TEMP_AUXADC_CHANNEL,
	.num_banks = MT8183_NUM_ZONES,
	.num_sensors = MT8183_NUM_SENSORS,
	.vts_index = mt8183_vts_index,
	.cali_val = MT8183_CALIBRATION,
	.num_controller = MT8183_NUM_CONTROLLER,
	.controller_offset = mt8183_tc_offset,
	.need_switch_bank = false,
	.bank_data = {
		{
			.num_sensors = 6,
			.sensors = mt8183_bank_data,
		},
	},

	.msr = mt8183_msr,
	.adcpnp = mt8183_adcpnp,
	.sensor_mux_values = mt8183_mux_values,
	.version = MTK_THERMAL_V1,
};

/*
 * MT7986 uses AUXADC Channel 11 for raw data access.
 */
static const struct mtk_thermal_data mt7986_thermal_data = {
	.auxadc_channel = MT7986_TEMP_AUXADC_CHANNEL,
	.num_banks = MT7986_NUM_ZONES,
	.num_sensors = MT7986_NUM_SENSORS,
	.vts_index = mt7986_vts_index,
	.cali_val = MT7986_CALIBRATION,
	.num_controller = MT7986_NUM_CONTROLLER,
	.controller_offset = mt7986_tc_offset,
	.need_switch_bank = true,
	.bank_data = {
		{
			.num_sensors = 1,
			.sensors = mt7986_bank_data,
		},
	},
	.msr = mt7986_msr,
	.adcpnp = mt7986_adcpnp,
	.sensor_mux_values = mt7986_mux_values,
	.version = MTK_THERMAL_V3,
	.apmixed_buffer_ctl_reg = APMIXED_SYS_TS_CON1,
	.apmixed_buffer_ctl_mask = GENMASK(31, 6) | BIT(3),
	.apmixed_buffer_ctl_set = BIT(0),
};

static bool mtk_thermal_temp_is_valid(struct mtk_thermal *mt, int temp)
{
	int max_temp = mt->conf->version == MTK_THERMAL_MT6797 ?
			MT6797_TEMP_MAX_TEST : MT8173_TEMP_MAX;

	return (temp >= MT8173_TEMP_MIN) && (temp <= max_temp);
}

/**
 * raw_to_mcelsius_v1 - convert a raw ADC value to mcelsius
 * @mt:	The thermal controller
 * @sensno:	sensor number
 * @raw:	raw ADC value
 *
 * This converts the raw ADC value to mcelsius using the SoC specific
 * calibration constants
 */
static int raw_to_mcelsius_v1(struct mtk_thermal *mt, int sensno, s32 raw)
{
	s32 tmp;

	raw &= 0xfff;

	tmp = 203450520 << 3;
	tmp /= mt->conf->cali_val + mt->o_slope;
	tmp /= 10000 + mt->adc_ge;
	tmp *= raw - mt->vts[sensno] - 3350;
	tmp >>= 3;

	return mt->degc_cali * 500 - tmp;
}

/* MT6797 uses the V1 fuse layout but also calibrates ADC offset and slope. */
static int raw_to_mcelsius_mt6797(struct mtk_thermal *mt, int sensno, s32 raw)
{
	s64 room, sample, delta;
	int gain, offset, slope;

	if (!raw)
		return 0;

	raw &= 0xfff;
	gain = 10000 + (mt->adc_ge - 512) * 10000 / 4096;
	offset = mt->adc_oe - 512;
	slope = 1663 + (mt->o_slope_sign ? -mt->o_slope * 10 :
					      mt->o_slope * 10);
	room = div_s64((s64)(mt->vts[sensno] + 3350 - offset) * 10000,
		       4096);
	room = div_s64(room * 10000, gain);
	sample = div_s64((s64)(raw - offset) * 10000, 4096);
	sample = div_s64(sample * 10000, gain);
	delta = div_s64((sample - room) * 15, 18);

	return mt->degc_cali * 500 - div_s64(delta * 100000, slope);
}

static int raw_to_mcelsius_v2(struct mtk_thermal *mt, int sensno, s32 raw)
{
	s32 format_1;
	s32 format_2;
	s32 g_oe;
	s32 g_gain;
	s32 g_x_roomt;
	s32 tmp;

	if (raw == 0)
		return 0;

	raw &= 0xfff;
	g_gain = 10000 + (((mt->adc_ge - 512) * 10000) >> 12);
	g_oe = mt->adc_oe - 512;
	format_1 = mt->vts[VTS2] + 3105 - g_oe;
	format_2 = (mt->degc_cali * 10) >> 1;
	g_x_roomt = (((format_1 * 10000) >> 12) * 10000) / g_gain;

	tmp = (((((raw - g_oe) * 10000) >> 12) * 10000) / g_gain) - g_x_roomt;
	tmp = tmp * 10 * 100 / 11;

	if (mt->o_slope_sign == 0)
		tmp = tmp / (165 - mt->o_slope);
	else
		tmp = tmp / (165 + mt->o_slope);

	return (format_2 - tmp) * 100;
}

static int raw_to_mcelsius_v3(struct mtk_thermal *mt, int sensno, s32 raw)
{
	s32 tmp;

	if (raw == 0)
		return 0;

	raw &= 0xfff;
	tmp = 100000 * 15 / 16 * 10000;
	tmp /= 4096 - 512 + mt->adc_ge;
	tmp /= 1490;
	tmp *= raw - mt->vts[sensno] - 2900;

	return mt->degc_cali * 500 - tmp;
}

/**
 * mtk_thermal_get_bank - get bank
 * @bank:	The bank
 *
 * The bank registers are banked, we have to select a bank in the
 * PTPCORESEL register to access it.
 */
/* One atomic transaction owns both selector access and the TC request gate.
 * No waits, mutex acquisitions, formatting or scheduling belong inside it.
 * EEM and diagnostic users use the same exported API, not a second lock.
 */
static int mt6797_bank_begin(struct mtk_thermal *mt, unsigned int bank,
                            struct mt6797_thermal_bank_cookie *cookie)
{
	int ret;
	u32 value;

	cookie->was_armed = false;
	spin_lock_irqsave(&mt6797_thermal_eem_lock, cookie->flags);
	cookie->hold_started_ns = ktime_get_mono_fast_ns();
	if (mt->hw_watchdog) {
		ret = mt6797_wdt_thermal_hold(mt->hw_watchdog, &cookie->was_armed);
		if (ret) {
			mt->bank_disarm_failures++;
			spin_unlock_irqrestore(&mt6797_thermal_eem_lock, cookie->flags);
			if (ret == -EREMOTEIO) {
				dev_emerg(mt->dev, "TC hold failed and entry request state was lost; restarting\n");
				emergency_restart();
			}
			return ret;
		}
	}
	value = readl(mt->thermal_base + PTPCORESEL);
	writel((value & ~0xf) | bank, mt->thermal_base + PTPCORESEL);
	readl(mt->thermal_base + PTPCORESEL);
	return 0;
}

static void mt6797_trace_bank(struct mtk_thermal *mt, u64 started)
{
	u32 words[MT6797_TRACE_WORDS], bank, sequence;
	void __iomem *entry;
	u64 now;
	int i;

	if (!mt->tc_trace || !mt->hw_critical_temp)
		return;
	bank = readl(mt->thermal_base + PTPCORESEL) & 0xf;
	if (bank >= ARRAY_SIZE(mt->tc_trace_last_ns))
		return;
	now = ktime_get_mono_fast_ns();
	if (now - mt->tc_trace_last_ns[bank] < 10 * NSEC_PER_MSEC)
		return;
	mt->tc_trace_last_ns[bank] = now;
	sequence = ++mt->tc_trace_sequence[bank];
	words[0] = sequence;
	words[1] = lower_32_bits(now);
	words[2] = upper_32_bits(now);
	words[3] = bank;
	words[4] = min_t(u64, now - started, U32_MAX);
	words[5] = readl(mt->thermal_base + TEMP_MSR0);
	words[6] = readl(mt->thermal_base + TEMP_MSR1);
	words[7] = readl(mt->thermal_base + TEMP_MSR2);
	words[8] = readl(mt->thermal_base + TEMP_MSR3);
	words[9] = readl(mt->thermal_base + 0x00c);
	/* INTSTS may acknowledge events on read; automatic tracing must not
	 * consume the interrupt handler's evidence or change that protocol.
	 */
	words[10] = U32_MAX;
	words[11] = readl(mt->thermal_base + 0x0c0);
	words[12] = readl(mt->thermal_base + 0x0cc);
	words[13] = readl(mt->thermal_base + TEMP_MSRCTL1);
	words[14] = readl(mt->thermal_base + PTPCORESEL);
	entry = mt->tc_trace + 64 + (bank * 2 + (sequence & 1)) * 64;
	writel(0, entry + 60);
	for (i = 0; i < MT6797_TRACE_WORDS - 1; i++)
		writel(words[i], entry + i * 4);
	/* Last write commits the record. A reset during a write leaves the
	 * other slot intact; readers reject a sequence/commit mismatch.
	 */
	writel(sequence, entry + 60);
	readl(entry + 60);
}

static int mt6797_bank_end(struct mtk_thermal *mt,
                          struct mt6797_thermal_bank_cookie *cookie)
{
	u64 elapsed;
	int ret = 0, retry;

	/* Bounded MMIO under the existing shared bank lock, before rearming.
	 * No extra thread, printk, filesystem I/O or WDT_MODE write.
	 */
	mt6797_trace_bank(mt, cookie->hold_started_ns);
	if (cookie->was_armed) {
		for (retry = 0; retry < 3; retry++) {
			ret = mt6797_wdt_thermal_request(mt->hw_watchdog, true);
			if (!ret)
				break;
			mt->bank_restore_retries++;
		}
	}
	elapsed = ktime_get_mono_fast_ns() - cookie->hold_started_ns;
	mt->bank_transactions++;
	if (cookie->was_armed)
		mt->bank_hold_max_ns = max(mt->bank_hold_max_ns, elapsed);
	if (ret)
		mt->bank_rearm_failures++;
	spin_unlock_irqrestore(&mt6797_thermal_eem_lock, cookie->flags);
	if (ret) {
		/* Continuing with the final hardware protection disarmed is not
		 * a successful temperature read.  Bound recovery and reset.
		 */
		dev_emerg(mt->dev, "TC direct reset request could not be restored: %d; restarting\n", ret);
		emergency_restart();
	}
	return ret;
}

int mt6797_thermal_bank_begin(struct device *dev, unsigned int bank,
                              struct mt6797_thermal_bank_cookie *cookie)
{
	struct mtk_thermal *mt = dev_get_drvdata(dev);

	if (!mt || mt->conf->version != MTK_THERMAL_MT6797 || bank >= mt->conf->num_banks)
		return -EINVAL;
	return mt6797_bank_begin(mt, bank, cookie);
}
EXPORT_SYMBOL_GPL(mt6797_thermal_bank_begin);

int mt6797_thermal_bank_end(struct device *dev,
                            struct mt6797_thermal_bank_cookie *cookie)
{
	struct mtk_thermal *mt = dev_get_drvdata(dev);

	return mt6797_bank_end(mt, cookie);
}
EXPORT_SYMBOL_GPL(mt6797_thermal_bank_end);

static int mtk_thermal_get_bank(struct mtk_thermal_bank *bank,
                               struct mt6797_thermal_bank_cookie *cookie)
{
	struct mtk_thermal *mt = bank->mt;
	u32 val;

	if (!mt->conf->need_switch_bank)
		return 0;
	if (mt->conf->version == MTK_THERMAL_MT6797)
		return mt6797_bank_begin(mt, bank->id, cookie);
	mutex_lock(&mt->lock);
	val = readl(mt->thermal_base + PTPCORESEL);
	writel((val & ~0xf) | bank->id, mt->thermal_base + PTPCORESEL);
	return 0;
}

static int mtk_thermal_put_bank(struct mtk_thermal_bank *bank,
                               struct mt6797_thermal_bank_cookie *cookie)
{
	struct mtk_thermal *mt = bank->mt;

	if (!mt->conf->need_switch_bank)
		return 0;
	if (mt->conf->version == MTK_THERMAL_MT6797)
		return mt6797_bank_end(mt, cookie);
	mutex_unlock(&mt->lock);
	return 0;
}

/**
 * mtk_thermal_bank_temperature - get the temperature of a bank
 * @bank:	The bank
 *
 * The temperature of a bank is considered the maximum temperature of
 * the sensors associated to the bank.
 */
static int mtk_thermal_bank_temperature(struct mtk_thermal_bank *bank)
{
	struct mtk_thermal *mt = bank->mt;
	const struct mtk_thermal_data *conf = mt->conf;
	int i, temp = INT_MIN, max = INT_MIN;
	u32 raw;

	for (i = 0; i < conf->bank_data[bank->id].num_sensors; i++) {
		raw = readl(mt->thermal_base + conf->msr[i]);

		temp = mt->raw_to_mcelsius(
			mt, conf->bank_data[bank->id].sensors[i], raw);

		/*
		 * Depending on the filt/sen intervals and ADC polling time,
		 * we may need up to 60 milliseconds after initialization: this
		 * will result in the first reading containing an out of range
		 * temperature value.
		 * Validate the reading to both address the aforementioned issue
		 * and to eventually avoid bogus readings during runtime in the
		 * event that the AUXADC gets unstable due to high EMI, etc.
		 */
		if (!mtk_thermal_temp_is_valid(mt, temp))
			temp = THERMAL_TEMP_INVALID;

		if (temp > max)
			max = temp;
	}

	return max;
}

static int mtk_read_temp(struct thermal_zone_device *tz, int *temperature)
{
	struct mtk_thermal *mt = thermal_zone_device_priv(tz);
	struct mt6797_thermal_bank_cookie cookie;
	int i, ret, tempmax = INT_MIN;

	for (i = 0; i < mt->conf->num_banks; i++) {
		struct mtk_thermal_bank *bank = &mt->banks[i];

		ret = mtk_thermal_get_bank(bank, &cookie);
		if (ret)
			return ret;
		tempmax = max(tempmax, mtk_thermal_bank_temperature(bank));
		ret = mtk_thermal_put_bank(bank, &cookie);
		if (ret)
			return ret;
	}
	*temperature = tempmax;
	return 0;
}

int mt6797_thermal_bank_temp(struct device *dev, unsigned int id, int *temp)
{
	struct mtk_thermal *mt = dev_get_drvdata(dev);
	struct mt6797_thermal_bank_cookie cookie;
	int ret;

	if (!mt || mt->conf->version != MTK_THERMAL_MT6797 || id >= mt->conf->num_banks)
		return -EINVAL;
	ret = mtk_thermal_get_bank(&mt->banks[id], &cookie);
	if (ret)
		return ret;
	*temp = mtk_thermal_bank_temperature(&mt->banks[id]);
	ret = mtk_thermal_put_bank(&mt->banks[id], &cookie);
	return ret ?: (*temp == THERMAL_TEMP_INVALID ? -EAGAIN : 0);
}
EXPORT_SYMBOL_GPL(mt6797_thermal_bank_temp);

int mt6797_thermal_eem_calibration(struct device *dev, unsigned int bank,
				 u32 *mts, u32 *bts)
{
	static const unsigned int dominant_sensor[] = { 0, 3, 1, 1, 1, 1 };
	struct mtk_thermal *mt = dev_get_drvdata(dev);
	s64 room, t0, t1, t2;
	int gain, offset, slope;

	if (!mt || mt->conf->version != MTK_THERMAL_MT6797 ||
	    bank >= ARRAY_SIZE(dominant_sensor))
		return -EINVAL;
	gain = 10000 + (mt->adc_ge - 512) * 10000 / 4096;
	offset = mt->adc_oe - 512;
	slope = 1663 + (mt->o_slope_sign ? -mt->o_slope * 10 : mt->o_slope * 10);
	/* Preserve the BSP get_thermal_slope_intercept() integer ordering. */
	room = div_s64((s64)(mt->vts[dominant_sensor[bank]] + 3350 - offset) * 10000,
		       4096);
	room = div_s64(room * 10000, gain);
	t0 = div_s64(div_s64(1000000000, gain) * 15, 18);
	*mts = div_s64(t0 * 10, slope);
	t0 = mt->degc_cali * 10 / 2;
	t1 = div_s64((div_s64(1000000000 / 4096, gain) * offset + room * 10) * 15, 18);
	t2 = div_s64(t1 * 100, slope);
	*bts = div_s64((t0 + t2 - 250) * 4, 10);
	return 0;
}
EXPORT_SYMBOL_GPL(mt6797_thermal_eem_calibration);

int mt6797_thermal_otp_calibration(struct device *dev, u32 *mts, u32 *bts)
{
	return mt6797_thermal_eem_calibration(dev, 0, mts, bts);
}
EXPORT_SYMBOL_GPL(mt6797_thermal_otp_calibration);

static const struct thermal_zone_device_ops mtk_thermal_ops = {
	.get_temp = mtk_read_temp,
};

static ssize_t bank_temperatures_show(struct device *dev,
                                     struct device_attribute *attr, char *buf)
{
	struct mtk_thermal *mt = dev_get_drvdata(dev);
	struct mt6797_thermal_bank_cookie cookie;
	ssize_t len = 0;
	int i, temperature, ret;

	for (i = 0; i < mt->conf->num_banks; i++) {
		struct mtk_thermal_bank *bank = &mt->banks[i];

		ret = mtk_thermal_get_bank(bank, &cookie);
		if (ret)
			return ret;
		temperature = mtk_thermal_bank_temperature(bank);
		ret = mtk_thermal_put_bank(bank, &cookie);
		if (ret)
			return ret;
		len += sysfs_emit_at(buf, len, "bank%d=%d\n", i, temperature);
	}
	return len;
}
static DEVICE_ATTR_RO(bank_temperatures);

static ssize_t eem_status_show(struct device *dev,
                              struct device_attribute *attr, char *buf)
{
	struct mtk_thermal *mt = dev_get_drvdata(dev);
	struct mt6797_thermal_bank_cookie cookie;
	ssize_t len = 0;
	int i, ret;

	len += sysfs_emit_at(buf, len, "voltage_control=manual_opp\n");
	for (i = 0; i < mt->conf->num_banks; i++) {
		struct mtk_thermal_bank *bank = &mt->banks[i];
		u32 enable, status, vop30, vop74;

		ret = mtk_thermal_get_bank(bank, &cookie);
		if (ret)
			return ret;
		enable = readl(mt->thermal_base + 0x238);
		status = readl(mt->thermal_base + 0x254);
		vop30 = readl(mt->thermal_base + 0x248);
		vop74 = readl(mt->thermal_base + 0x24c);
		ret = mtk_thermal_put_bank(bank, &cookie);
		if (ret)
			return ret;
		len += sysfs_emit_at(buf, len,
			"bank%d eem_en=0x%08x intsts=0x%08x vop30=0x%08x vop74=0x%08x\n",
			i, enable, status, vop30, vop74);
	}
	return len;
}
static DEVICE_ATTR_RO(eem_status);

static int mt6797_hw_protect(struct mtk_thermal *mt, int temperature)
{
	struct mt6797_thermal_bank_cookie cookie;
	s64 room, delta, raw;
	int gain, offset, slope, i, ret, restore_ret, retry;
	u32 interrupt, thresholds[MAX_NUM_ZONES];
	u32 old_threshold[MAX_NUM_ZONES], old_control[MAX_NUM_ZONES];
	u32 old_interrupt[MAX_NUM_ZONES], selector;
	bool ready, first_setup;

	if (!mt->hw_watchdog)
		return -EOPNOTSUPP;
	mutex_lock(&mt->hw_protect_lock);
	/* Waiting never holds the request off or the shared spinlock. */
	for (retry = 0; retry < 100; retry++) {
		ready = true;
		for (i = 0; i < mt->conf->num_banks; i++) {
			ret = mtk_thermal_get_bank(&mt->banks[i], &cookie);
			if (ret)
				goto unlock;
			ready &= !!(readl(mt->thermal_base + TEMP_MSR0) & 0xfff);
			ret = mtk_thermal_put_bank(&mt->banks[i], &cookie);
			if (ret)
				goto unlock;
		}
		if (ready)
			break;
		usleep_range(1000, 2000);
	}
	if (!ready) {
		ret = -ETIMEDOUT;
		goto unlock;
	}
	gain = 10000 + (mt->adc_ge - 512) * 10000 / 4096;
	offset = mt->adc_oe - 512;
	slope = 1663 + (mt->o_slope_sign ? -mt->o_slope * 10 : mt->o_slope * 10);
	for (i = 0; i < mt->conf->num_banks; i++) {
		int sensor = mt->conf->bank_data[i].sensors[0];

		room = div_s64((s64)(mt->vts[sensor] + 3350 - offset) * 10000, 4096);
		room = div_s64(room * 10000, gain);
		delta = div_s64((s64)(temperature - mt->degc_cali * 500) * slope, 10);
		delta = -div_s64(delta * 18, 15);
		raw = div_s64(delta, 1000) + room * 10;
		raw = div_s64(div_s64(raw * 4096, 10000) * gain, 100000) + offset;
		if (raw < 1 || raw > 4095) {
			ret = -ERANGE;
			goto unlock;
		}
		thresholds[i] = raw;
	}
	first_setup = !mt->hw_critical_temp;
	/* BSP programs all bank thresholds while routing is held off.  The
	 * entire transaction below is MMIO only, under the shared IRQ-safe
	 * bank lock; rollback is also bounded and restores all old banks.
	 */
	ret = mt6797_bank_begin(mt, 0, &cookie);
	if (ret)
		goto unlock;
	selector = readl(mt->thermal_base + PTPCORESEL) & ~0xf;
	for (i = 0; i < mt->conf->num_banks; i++) {
		writel(selector | i, mt->thermal_base + PTPCORESEL);
		old_control[i] = readl(mt->thermal_base + 0x0c0);
		old_threshold[i] = readl(mt->thermal_base + 0x0cc);
		old_interrupt[i] = readl(mt->thermal_base + 0x00c);
	}
	for (i = 0; i < mt->conf->num_banks; i++) {
		writel(selector | i, mt->thermal_base + PTPCORESEL);
		interrupt = old_interrupt[i];
		writel(0, mt->thermal_base + 0x00c);
		writel(0x20000, mt->thermal_base + 0x0c0);
		writel(thresholds[i], mt->thermal_base + 0x0cc);
		writel(interrupt | BIT(31), mt->thermal_base + 0x00c);
		if (readl(mt->thermal_base + 0x0cc) != thresholds[i] ||
		    !(readl(mt->thermal_base + 0x00c) & BIT(31))) {
			ret = -EIO;
			break;
		}
	}
	if (ret) {
		for (i = 0; i < mt->conf->num_banks; i++) {
			writel(selector | i, mt->thermal_base + PTPCORESEL);
			writel(0, mt->thermal_base + 0x00c);
			writel(old_control[i], mt->thermal_base + 0x0c0);
			writel(old_threshold[i], mt->thermal_base + 0x0cc);
			writel(old_interrupt[i], mt->thermal_base + 0x00c);
		}
	} else {
		mt->hw_critical_temp = temperature;
	}
	restore_ret = mt6797_bank_end(mt, &cookie);
	ret = restore_ret ?: ret;
	if (!ret && first_setup) {
		/* Set MODE/routing once, rather than clearing STATUS on reads. */
		ret = mt6797_wdt_thermal_direct(mt->hw_watchdog, true);
		if (ret)
			mt->hw_critical_temp = 0;
	}
unlock:
	mutex_unlock(&mt->hw_protect_lock);
	return ret;
}

static ssize_t hw_critical_temp_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct mtk_thermal *mt = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", READ_ONCE(mt->hw_critical_temp));
}

static ssize_t hw_critical_temp_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct mtk_thermal *mt = dev_get_drvdata(dev);
	int temperature, ret;

	if (kstrtoint(buf, 0, &temperature) || temperature < 10000 || temperature > 150000)
		return -EINVAL;
	ret = mt6797_hw_protect(mt, temperature);
	return ret ? ret : count;
}
static DEVICE_ATTR_RW(hw_critical_temp);

/* Read-only observation of the MT6797 thermal-controller state used to
 * audit the BSP alignment: steady-state hardware filter, per-bank raw
 * sample, and the hardware-direct-reset threshold programming.
 */
static ssize_t tc_hw_status_show(struct device *dev,
                                 struct device_attribute *attr, char *buf)
{
	struct mtk_thermal *mt = dev_get_drvdata(dev);
	struct mt6797_thermal_bank_cookie cookie;
	ssize_t len = 0;
	int i, ret;

	len += sysfs_emit_at(buf, len,
		"msrctl0=0x%05x monctl1=0x%08x monctl2=0x%08x ahbpoll=0x%03x\n",
		readl(mt->thermal_base + TEMP_MSRCTL0),
		readl(mt->thermal_base + TEMP_MONCTL1),
		readl(mt->thermal_base + TEMP_MONCTL2),
		readl(mt->thermal_base + TEMP_AHBPOLL));
	for (i = 0; i < mt->conf->num_banks; i++) {
		struct mtk_thermal_bank *bank = &mt->banks[i];
		u32 raw[4], monintc, intsts, protctl, prottc;

		ret = mtk_thermal_get_bank(bank, &cookie);
		if (ret)
			return ret;
		raw[0] = readl(mt->thermal_base + TEMP_MSR0);
		raw[1] = readl(mt->thermal_base + TEMP_MSR1);
		raw[2] = readl(mt->thermal_base + TEMP_MSR2);
		raw[3] = readl(mt->thermal_base + TEMP_MSR3);
		monintc = readl(mt->thermal_base + 0x00c);
		intsts = readl(mt->thermal_base + 0x010);
		protctl = readl(mt->thermal_base + 0x0c0);
		prottc = readl(mt->thermal_base + 0x0cc);
		ret = mtk_thermal_put_bank(bank, &cookie);
		if (ret)
			return ret;
		len += sysfs_emit_at(buf, len,
			"bank%d raw_msr0=0x%03x raw_msr1=0x%03x raw_msr2=0x%03x raw_msr3=0x%03x monintc=0x%08x intsts=0x%08x protctl=0x%05x prottc=0x%03x\n",
			i, raw[0] & 0xfff, raw[1] & 0xfff, raw[2] & 0xfff, raw[3] & 0xfff,
			monintc, intsts, protctl & 0xfffff, prottc & 0xfff);
	}
	return len;
}
static DEVICE_ATTR_RO(tc_hw_status);

static ssize_t pm_state_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct mtk_thermal *mt = dev_get_drvdata(dev);

	return sysfs_emit(buf, "suspends=%u resumes=%u failures=%u last_error=%d suspended=%u\n",
			 mt->pm_suspends, mt->pm_resumes, mt->pm_failures,
			 mt->pm_last_error, mt->pm_suspended);
}
static DEVICE_ATTR_RO(pm_state);

static ssize_t bank_guard_show(struct device *dev,
                              struct device_attribute *attr, char *buf)
{
	struct mtk_thermal *mt = dev_get_drvdata(dev);
	unsigned long flags;
	u64 count, maximum, disarm, rearm, retries;

	spin_lock_irqsave(&mt6797_thermal_eem_lock, flags);
	count = mt->bank_transactions;
	maximum = mt->bank_hold_max_ns;
	disarm = mt->bank_disarm_failures;
	rearm = mt->bank_rearm_failures;
	retries = mt->bank_restore_retries;
	spin_unlock_irqrestore(&mt6797_thermal_eem_lock, flags);
	return sysfs_emit(buf,
		"transactions=%llu max_hold_ns=%llu disarm_failures=%llu rearm_failures=%llu restore_retries=%llu\n",
		count, maximum, disarm, rearm, retries);
}
static DEVICE_ATTR_RO(bank_guard);

static ssize_t previous_tc_trace_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct mtk_thermal *mt = dev_get_drvdata(dev);
	ssize_t len;
	int i;

	len = sysfs_emit(buf, "available=%d previous_valid=%d generation=%u previous_generation=%u period_ns=10000000\n",
		!!mt->tc_trace, mt->tc_trace_previous_valid,
		mt->tc_trace_generation, mt->tc_trace_previous_generation);
	if (!mt->tc_trace_previous_valid)
		return len;
	for (i = 0; i < MT6797_TRACE_RECORDS; i++) {
		u32 *w = mt->tc_trace_previous[i];
		bool valid = w[0] && w[0] == w[15] && w[3] == i / 2;

		len += sysfs_emit_at(buf, len,
			"slot=%d valid=%d seq=%u mono_ns=%llu bank=%u before_trace_ns=%u msr=%08x,%08x,%08x,%08x monint=%08x intsts=%08x protctl=%08x prottc=%08x msrctl1=%08x selector=%08x commit=%u\n",
			i, valid, w[0], ((u64)w[2] << 32) | w[1], w[3], w[4],
			w[5], w[6], w[7], w[8], w[9], w[10], w[11], w[12],
			w[13], w[14], w[15]);
	}
	return len;
}
static DEVICE_ATTR_RO(previous_tc_trace);

static ssize_t failure_tc_trace_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct mtk_thermal *mt = dev_get_drvdata(dev);
	ssize_t len;
	int i;

	len = sysfs_emit(buf, "valid=%d generation=%u pl_reason=%#x\n",
		mt->tc_trace_failure_valid, mt->tc_trace_failure_generation,
		mt->tc_trace_failure_reason);
	if (!mt->tc_trace_failure_valid)
		return len;
	for (i = 0; i < MT6797_TRACE_RECORDS; i++) {
		u32 *w = mt->tc_trace_failure[i];

		len += sysfs_emit_at(buf, len,
			"slot=%d valid=%d seq=%u mono_ns=%llu bank=%u before_trace_ns=%u msr=%08x,%08x,%08x,%08x monint=%08x intsts=%08x protctl=%08x prottc=%08x msrctl1=%08x selector=%08x commit=%u\n",
			i, !!w[0] && w[0] == w[15] && w[3] == i / 2,
			w[0], ((u64)w[2] << 32) | w[1], w[3], w[4], w[5],
			w[6], w[7], w[8], w[9], w[10], w[11], w[12], w[13],
			w[14], w[15]);
	}
	return len;
}
static DEVICE_ATTR_RO(failure_tc_trace);

static struct attribute *mt6797_thermal_attrs[] = {
	&dev_attr_pm_state.attr,
	&dev_attr_bank_temperatures.attr,
	&dev_attr_eem_status.attr,
	&dev_attr_hw_critical_temp.attr,
	&dev_attr_tc_hw_status.attr,
	&dev_attr_bank_guard.attr,
	&dev_attr_previous_tc_trace.attr,
	&dev_attr_failure_tc_trace.attr,
	NULL,
};
static const struct attribute_group mt6797_thermal_group = {
	.attrs = mt6797_thermal_attrs,
};

/* BSP fast init fills even disabled slots; hardware protection compares all
 * slots, and an uninitialized zero would represent an extremely hot sensor.
 */
static int mt6797_thermal_fast_init(struct mtk_thermal *mt)
{
	static const u32 msr[] = { TEMP_MSR0, TEMP_MSR1, TEMP_MSR2, TEMP_MSR3 };
	u32 value;
	int i, ret;

	writel(0, mt->thermal_base + 0x00c);
	writel(0, mt->thermal_base + TEMP_MONCTL0);
	writel(0x1da1, mt->thermal_base + 0x428);
	writel(1, mt->thermal_base + TEMP_MONCTL1);
	writel(1, mt->thermal_base + TEMP_MONCTL2);
	writel(1, mt->thermal_base + TEMP_AHBPOLL);
	writel(0xff, mt->thermal_base + TEMP_AHBTO);
	writel(0, mt->thermal_base + TEMP_MONIDET0);
	writel(0, mt->thermal_base + TEMP_MONIDET1);
	writel(0, mt->thermal_base + TEMP_MSRCTL0);
	writel(1, mt->thermal_base + TEMP_ADCPNP0);
	writel(2, mt->thermal_base + TEMP_ADCPNP1);
	writel(3, mt->thermal_base + TEMP_ADCPNP2);
	writel(4, mt->thermal_base + 0x0b4);
	writel(mt->thermal_phys + 0x420, mt->thermal_base + TEMP_PNPMUXADDR);
	writel(mt->thermal_phys + 0x420, mt->thermal_base + TEMP_ADCMUXADDR);
	writel(mt->thermal_phys + 0x424, mt->thermal_base + TEMP_ADCENADDR);
	writel(mt->thermal_phys + 0x428, mt->thermal_base + TEMP_ADCVALIDADDR);
	writel(mt->thermal_phys + 0x428, mt->thermal_base + TEMP_ADCVOLTADDR);
	writel(0, mt->thermal_base + TEMP_RDCTRL);
	writel(0x2c, mt->thermal_base + TEMP_ADCVALIDMASK);
	writel(0, mt->thermal_base + TEMP_ADCVOLTAGESHIFT);
	writel(3, mt->thermal_base + TEMP_ADCWRITECTRL);
	writel(15, mt->thermal_base + TEMP_MONCTL0);
	for (i = 0; i < ARRAY_SIZE(msr); i++) {
		ret = readl_poll_timeout_atomic(mt->thermal_base + msr[i], value,
					       (value & 0xfff) == 0xda1, 1, 2000);
		if (ret)
			break;
	}
	writel(0, mt->thermal_base + TEMP_MONCTL0);
	return ret;
}

static int mtk_thermal_init_bank(struct mtk_thermal *mt, int num,
				  u32 apmixed_phys_base, u32 auxadc_phys_base,
				  int ctrl_id)
{
	struct mt6797_thermal_bank_cookie cookie;
	struct mtk_thermal_bank *bank = &mt->banks[num];
	const struct mtk_thermal_data *conf = mt->conf;
	int i, ret;

	int offset = mt->conf->controller_offset[ctrl_id];
	void __iomem *controller_base = mt->thermal_base + offset;

	bank->id = num;
	bank->mt = mt;

	ret = mtk_thermal_get_bank(bank, &cookie);
	if (ret)
		return ret;

	/* bus clock 66M counting unit is 12 * 15.15ns * 256 = 46.540us */
	writel(TEMP_MONCTL1_PERIOD_UNIT(12), controller_base + TEMP_MONCTL1);

	/*
	 * filt interval is 1 * 46.540us = 46.54us,
	 * sen interval is 429 * 46.540us = 19.96ms
	 */
	writel(TEMP_MONCTL2_FILTER_INTERVAL(1) |
			TEMP_MONCTL2_SENSOR_INTERVAL(
				mt->conf->version == MTK_THERMAL_MT6797 &&
				num == 0 ? 46 : 429),
			controller_base + TEMP_MONCTL2);

	/* poll is set to 10u */
	writel(TEMP_AHBPOLL_ADC_POLL_INTERVAL(
		mt->conf->version == MTK_THERMAL_MT6797 ? 781 : 768),
	       controller_base + TEMP_AHBPOLL);

	/*
	 * Temperature sampling control.  MT6797 follows the BSP
	 * THERMAL_CONTROLLER_HW_FILTER=2 setting (tscpu_settings.h):
	 * TEMPMSRCTL0=0x492, i.e. take 2 of 4 samples for steady-state
	 * reads.  The fast-init flow above keeps the BSP single-sample
	 * value (0) while the slots are first filled.
	 */
	writel(mt->conf->version == MTK_THERMAL_MT6797 ? 0x492 : 0x0,
	       controller_base + TEMP_MSRCTL0);

	/* exceed this polling time, IRQ would be inserted */
	writel(0xffffffff, controller_base + TEMP_AHBTO);

	/* number of interrupts per event, 1 is enough */
	writel(0x0, controller_base + TEMP_MONIDET0);
	writel(0x0, controller_base + TEMP_MONIDET1);

	/*
	 * The MT8173 thermal controller does not have its own ADC. Instead it
	 * uses AHB bus accesses to control the AUXADC. To do this the thermal
	 * controller has to be programmed with the physical addresses of the
	 * AUXADC registers and with the various bit positions in the AUXADC.
	 * Also the thermal controller controls a mux in the APMIXEDSYS register
	 * space.
	 */

	/*
	 * this value will be stored to TEMP_PNPMUXADDR (TEMP_SPARE0)
	 * automatically by hw
	 */
	writel(BIT(conf->auxadc_channel), controller_base + TEMP_ADCMUX);

	/* AHB address for auxadc mux selection */
	writel(auxadc_phys_base + AUXADC_CON1_CLR_V,
	       controller_base + TEMP_ADCMUXADDR);

	if (mt->conf->version == MTK_THERMAL_V1 ||
	    mt->conf->version == MTK_THERMAL_MT6797) {
		/* AHB address for pnp sensor mux selection */
		writel(apmixed_phys_base + APMIXED_SYS_TS_CON1,
		       controller_base + TEMP_PNPMUXADDR);
	}

	/* AHB value for auxadc enable */
	writel(BIT(conf->auxadc_channel), controller_base + TEMP_ADCEN);

	/* AHB address for auxadc enable (channel 0 immediate mode selected) */
	writel(auxadc_phys_base + AUXADC_CON1_SET_V,
	       controller_base + TEMP_ADCENADDR);

	/* AHB address for auxadc valid bit */
	writel(auxadc_phys_base + AUXADC_DATA(conf->auxadc_channel),
	       controller_base + TEMP_ADCVALIDADDR);

	/* AHB address for auxadc voltage output */
	writel(auxadc_phys_base + AUXADC_DATA(conf->auxadc_channel),
	       controller_base + TEMP_ADCVOLTADDR);

	/* read valid & voltage are at the same register */
	writel(0x0, controller_base + TEMP_RDCTRL);

	/* indicate where the valid bit is */
	writel(TEMP_ADCVALIDMASK_VALID_HIGH | TEMP_ADCVALIDMASK_VALID_POS(12),
	       controller_base + TEMP_ADCVALIDMASK);

	/* no shift */
	writel(0x0, controller_base + TEMP_ADCVOLTAGESHIFT);

	/* enable auxadc mux write transaction */
	writel(TEMP_ADCWRITECTRL_ADC_MUX_WRITE,
		controller_base + TEMP_ADCWRITECTRL);

	for (i = 0; i < conf->bank_data[num].num_sensors; i++)
		writel(conf->sensor_mux_values[conf->bank_data[num].sensors[i]],
		       mt->thermal_base + conf->adcpnp[i]);

	/* MT6797 starts all banks only after every address/mux is programmed. */
	if (mt->conf->version != MTK_THERMAL_MT6797)
		writel((1 << conf->bank_data[num].num_sensors) - 1,
		       controller_base + TEMP_MONCTL0);

	writel(TEMP_ADCWRITECTRL_ADC_PNP_WRITE |
	       TEMP_ADCWRITECTRL_ADC_MUX_WRITE,
	       controller_base + TEMP_ADCWRITECTRL);

	return mtk_thermal_put_bank(bank, &cookie);
}

static int mt6797_stop_all_banks(struct mtk_thermal *mt)
{
	struct mt6797_thermal_bank_cookie cookie;
	u32 value;
	int i, ret;

	/* MT6797 BSP disables periodic sensing and drains THAHBST0 before
	 * replacing the AHB target addresses. Wait outside the bank spinlock.
	 */
	for (i = 0; i < mt->conf->num_banks; i++) {
		ret = mtk_thermal_get_bank(&mt->banks[i], &cookie);
		if (ret)
			return ret;
		writel(0, mt->thermal_base + TEMP_MONCTL0);
		ret = mtk_thermal_put_bank(&mt->banks[i], &cookie);
		if (ret)
			return ret;
	}
	ret = readl_poll_timeout(mt->thermal_base + 0x418, value,
				 !(value >> 16), 2, 2500);
	if (ret)
		return dev_err_probe(mt->dev, ret,
				     "BSP thermal AHB drain timed out: %#x\n", value);
	return 0;
}

static void mt6797_stop_on_release(void *data)
{
	struct mtk_thermal *mt = data;
	int ret;

	/* Later devres mappings (trace) have already been removed. Clock and
	 * thermal register mappings still exist until after this action.
	 */
	mt->tc_trace = NULL;
	if (mt->hw_watchdog) {
		ret = mt6797_wdt_thermal_request(mt->hw_watchdog, false);
		if (ret) {
			dev_emerg(mt->dev, "cannot stop TC reset request on release: %d\n", ret);
			emergency_restart();
			return;
		}
	}
	ret = mt6797_stop_all_banks(mt);
	if (ret) {
		dev_emerg(mt->dev, "cannot drain TC before clock release: %d\n", ret);
		emergency_restart();
	}
}

static int mt6797_init_all_banks(struct mtk_thermal *mt, u32 apmixed, u32 auxadc,
				  void __iomem *auxadc_base)
{
	struct mt6797_thermal_bank_cookie cookie;
	u32 value;
	int i, ret;

	/* BSP seeds every bank before normal sampling. Mixing one bank's
	 * real AUXADC traffic with another bank's fast-init is not equivalent.
	 */
	for (i = 0; i < mt->conf->num_banks; i++) {
		ret = mtk_thermal_get_bank(&mt->banks[i], &cookie);
		if (ret)
			return ret;
		ret = mt6797_thermal_fast_init(mt);
		if (ret) {
			mtk_thermal_put_bank(&mt->banks[i], &cookie);
			return dev_err_probe(mt->dev, ret,
					     "BSP fast-init timed out in bank%d\n", i);
		}
		ret = mtk_thermal_put_bank(&mt->banks[i], &cookie);
		if (ret)
			return ret;
	}
	ret = mt6797_stop_all_banks(mt);
	if (ret)
		return ret;
	for (i = 0; i < mt->conf->num_banks; i++) {
		ret = mtk_thermal_init_bank(mt, i, apmixed, auxadc, 0);
		if (ret)
			return ret;
	}
	writel(BIT(11), auxadc_base + AUXADC_CON1_SET_V);
	for (i = 0; i < mt->conf->num_banks; i++) {
		ret = mtk_thermal_get_bank(&mt->banks[i], &cookie);
		if (ret)
			return ret;
		writel((1U << mt->conf->bank_data[i].num_sensors) - 1,
		       mt->thermal_base + TEMP_MONCTL0);
		value = readl(mt->thermal_base + TEMP_MSRCTL1);
		writel(value & ~0x10e, mt->thermal_base + TEMP_MSRCTL1);
		ret = mtk_thermal_put_bank(&mt->banks[i], &cookie);
		if (ret)
			return ret;
	}
	return 0;
}

static u64 of_get_phys_base(struct device_node *np)
{
	struct resource res;

	if (of_address_to_resource(np, 0, &res))
		return OF_BAD_ADDR;

	return res.start;
}

static int mtk_thermal_extract_efuse_v1(struct mtk_thermal *mt, u32 *buf)
{
	int i;

	if (!(buf[0] & CALIB_BUF0_VALID_V1))
		return -EINVAL;

	mt->adc_ge = CALIB_BUF1_ADC_GE_V1(buf[1]);

	for (i = 0; i < mt->conf->num_sensors; i++) {
		switch (mt->conf->vts_index[i]) {
		case VTS1:
			mt->vts[VTS1] = CALIB_BUF0_VTS_TS1_V1(buf[0]);
			break;
		case VTS2:
			mt->vts[VTS2] = CALIB_BUF0_VTS_TS2_V1(buf[0]);
			break;
		case VTS3:
			mt->vts[VTS3] = CALIB_BUF1_VTS_TS3_V1(buf[1]);
			break;
		case VTS4:
			mt->vts[VTS4] = CALIB_BUF2_VTS_TS4_V1(buf[2]);
			break;
		case VTS5:
			mt->vts[VTS5] = CALIB_BUF2_VTS_TS5_V1(buf[2]);
			break;
		case VTSABB:
			mt->vts[VTSABB] =
				CALIB_BUF2_VTS_TSABB_V1(buf[2]);
			break;
		default:
			break;
		}
	}

	mt->degc_cali = CALIB_BUF0_DEGC_CALI_V1(buf[0]);
	if (CALIB_BUF1_ID_V1(buf[1]) &
	    CALIB_BUF0_O_SLOPE_SIGN_V1(buf[0]))
		mt->o_slope = -CALIB_BUF0_O_SLOPE_V1(buf[0]);
	else
		mt->o_slope = CALIB_BUF0_O_SLOPE_V1(buf[0]);

	return 0;
}

static int mtk_thermal_extract_efuse_mt6797(struct mtk_thermal *mt, u32 *buf)
{
	int ret = mtk_thermal_extract_efuse_v1(mt, buf);

	if (ret)
		return ret;

	mt->adc_oe = (buf[1] >> 12) & 0x3ff;
	mt->o_slope_sign = (buf[0] >> 7) & 1;
	mt->o_slope = (buf[0] >> 26) & 0x3f;
	if (!(buf[1] & BIT(9)))
		mt->o_slope = 0;

	/* The BSP substitutes defaults for individual bad calibration fields. */
	if (mt->adc_ge < 265 || mt->adc_ge > 758)
		mt->adc_ge = 512;
	if (mt->adc_oe < 265 || mt->adc_oe > 758)
		mt->adc_oe = 512;
	if (mt->degc_cali < 1 || mt->degc_cali > 63)
		mt->degc_cali = 40;
	for (int i = 0; i < mt->conf->num_sensors; i++)
		if (mt->vts[i] < -8 || mt->vts[i] > 484)
			mt->vts[i] = 260;

	return 0;
}

static int mtk_thermal_extract_efuse_v2(struct mtk_thermal *mt, u32 *buf)
{
	if (!CALIB_BUF1_VALID_V2(buf[1]))
		return -EINVAL;

	mt->adc_oe = CALIB_BUF0_ADC_OE_V2(buf[0]);
	mt->adc_ge = CALIB_BUF0_ADC_GE_V2(buf[0]);
	mt->degc_cali = CALIB_BUF0_DEGC_CALI_V2(buf[0]);
	mt->o_slope = CALIB_BUF0_O_SLOPE_V2(buf[0]);
	mt->vts[VTS1] = CALIB_BUF1_VTS_TS1_V2(buf[1]);
	mt->vts[VTS2] = CALIB_BUF1_VTS_TS2_V2(buf[1]);
	mt->vts[VTSABB] = CALIB_BUF1_VTS_TSABB_V2(buf[1]);
	mt->o_slope_sign = CALIB_BUF1_O_SLOPE_SIGN_V2(buf[1]);

	return 0;
}

static int mtk_thermal_extract_efuse_v3(struct mtk_thermal *mt, u32 *buf)
{
	if (!CALIB_BUF1_VALID_V3(buf[1]))
		return -EINVAL;

	mt->adc_ge = CALIB_BUF0_ADC_GE_V3(buf[0]);
	mt->degc_cali = CALIB_BUF0_DEGC_CALI_V3(buf[0]);
	mt->o_slope = CALIB_BUF0_O_SLOPE_V3(buf[0]);
	mt->vts[VTS1] = CALIB_BUF1_VTS_TS1_V3(buf[1]);
	mt->vts[VTS2] = CALIB_BUF1_VTS_TS2_V3(buf[1]);
	mt->vts[VTSABB] = CALIB_BUF1_VTS_TSABB_V3(buf[1]);
	mt->o_slope_sign = CALIB_BUF1_O_SLOPE_SIGN_V3(buf[1]);

	if (CALIB_BUF1_ID_V3(buf[1]) == 0)
		mt->o_slope = 0;

	return 0;
}

static int mtk_thermal_get_calibration_data(struct device *dev,
					    struct mtk_thermal *mt)
{
	struct nvmem_cell *cell;
	u32 *buf;
	size_t len;
	int i, ret = 0;

	/* Start with default values */
	mt->adc_ge = 512;
	mt->adc_oe = 512;
	for (i = 0; i < mt->conf->num_sensors; i++)
		mt->vts[i] = 260;
	mt->degc_cali = 40;
	mt->o_slope = 0;

	cell = nvmem_cell_get(dev, "calibration-data");
	if (IS_ERR(cell)) {
		if (PTR_ERR(cell) == -EPROBE_DEFER)
			return PTR_ERR(cell);
		return 0;
	}

	buf = (u32 *)nvmem_cell_read(cell, &len);

	nvmem_cell_put(cell);

	if (IS_ERR(buf))
		return PTR_ERR(buf);

	if (len < 3 * sizeof(u32)) {
		dev_warn(dev, "invalid calibration data\n");
		ret = -EINVAL;
		goto out;
	}

	switch (mt->conf->version) {
	case MTK_THERMAL_V1:
		ret = mtk_thermal_extract_efuse_v1(mt, buf);
		break;
	case MTK_THERMAL_MT6797:
		ret = mtk_thermal_extract_efuse_mt6797(mt, buf);
		break;
	case MTK_THERMAL_V2:
		ret = mtk_thermal_extract_efuse_v2(mt, buf);
		break;
	case MTK_THERMAL_V3:
		ret = mtk_thermal_extract_efuse_v3(mt, buf);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	if (ret) {
		dev_info(dev, "Device not calibrated, using default calibration values\n");
		ret = 0;
	}

out:
	kfree(buf);

	return ret;
}

static const struct of_device_id mtk_thermal_of_match[] = {
	{
		.compatible = "mediatek,mt6797-thermal",
		.data = &mt6797_thermal_data,
	},
	{
		.compatible = "mediatek,mt8173-thermal",
		.data = (void *)&mt8173_thermal_data,
	},
	{
		.compatible = "mediatek,mt2701-thermal",
		.data = (void *)&mt2701_thermal_data,
	},
	{
		.compatible = "mediatek,mt2712-thermal",
		.data = (void *)&mt2712_thermal_data,
	},
	{
		.compatible = "mediatek,mt7622-thermal",
		.data = (void *)&mt7622_thermal_data,
	},
	{
		.compatible = "mediatek,mt7986-thermal",
		.data = (void *)&mt7986_thermal_data,
	},
	{
		.compatible = "mediatek,mt8183-thermal",
		.data = (void *)&mt8183_thermal_data,
	},
	{
		.compatible = "mediatek,mt8365-thermal",
		.data = (void *)&mt8365_thermal_data,
	}, {
	},
};
MODULE_DEVICE_TABLE(of, mtk_thermal_of_match);

static int mtk_thermal_turn_on_buffer(struct mtk_thermal *mt,
				       void __iomem *apmixed_base)
{
	u32 tmp;

	if (!mt->conf->apmixed_buffer_ctl_reg)
		return 0;

	tmp = readl(apmixed_base + mt->conf->apmixed_buffer_ctl_reg);
	tmp &= mt->conf->apmixed_buffer_ctl_mask;
	tmp |= mt->conf->apmixed_buffer_ctl_set;
	writel(tmp, apmixed_base + mt->conf->apmixed_buffer_ctl_reg);
	udelay(200);
	if (mt->conf->version == MTK_THERMAL_MT6797) {
		tmp = readl(apmixed_base + APMIXED_SYS_TS_CON1);
		if (tmp & GENMASK(5, 4))
			return dev_err_probe(mt->dev, -EIO,
				"TS analog buffer did not power on: %#x\n", tmp);
		dev_info(mt->dev, "TS analog buffer on, 200us settle, TS_CON1=%#x\n", tmp);
	}
	return 0;
}

static void mtk_thermal_release_periodic_ts(struct mtk_thermal *mt,
					    void __iomem *auxadc_base)
{
	int tmp;

	writel(0x800, auxadc_base + AUXADC_CON1_SET_V);
	writel(0x1, mt->thermal_base + TEMP_MONCTL0);
	tmp = readl(mt->thermal_base + TEMP_MSRCTL1);
	writel((tmp & (~0x10e)), mt->thermal_base + TEMP_MSRCTL1);
}

static void mt6797_trace_init(struct mtk_thermal *mt)
{
	void __iomem *sram, *archive;
	unsigned int i, j;
	u32 generation = 0, reason;
	bool archive_owned = true;

	sram = devm_ioremap(mt->dev, 0x12d000, 0xc00);
	if (!sram)
		return;
	if (readl(sram) != 0x43474244 || readl(sram + 4) != 0x40 ||
	    readl(sram + 8) != 0x80 || readl(sram + 12) != 0x10 ||
	    readl(sram + 16) != 0xc0 || readl(sram + 20) != 0x100 ||
	    readl(sram + 24) != 0x40 || readl(sram + 0x2c) ||
	    readl(sram + 0x30)) {
		dev_warn(mt->dev, "TC SRAM trace unavailable: boot console layout has another owner\n");
		return;
	}
	mt->tc_trace = sram + MT6797_TRACE_OFFSET;
	if (readl(mt->tc_trace) == MT6797_TRACE_MAGIC &&
	    readl(mt->tc_trace + 4) == MT6797_TRACE_WORDS &&
	    readl(mt->tc_trace + 8) == MT6797_TRACE_RECORDS) {
		generation = readl(mt->tc_trace + 12);
		for (i = 0; i < MT6797_TRACE_RECORDS; i++)
			for (j = 0; j < MT6797_TRACE_WORDS; j++)
				mt->tc_trace_previous[i][j] =
					readl(mt->tc_trace + 64 + i * 64 + j * 4);
		mt->tc_trace_previous_valid = true;
		mt->tc_trace_previous_generation = generation;
	}
	/* Keep the last exceptional boot separately from the rotating current
	 * boot. A rescue SYSRSTB or normal reboot must not erase that record.
	 * Only claim the audited unused area when blank or already ours.
	 */
	archive = sram + MT6797_TRACE_ARCHIVE_OFFSET;
	if (readl(archive) != MT6797_TRACE_MAGIC) {
		for (i = 0; i < MT6797_TRACE_SIZE; i += 4)
			if (readl(archive + i)) {
				archive_owned = false;
				break;
			}
	} else if (readl(archive + 4) != MT6797_TRACE_WORDS ||
		   readl(archive + 8) != MT6797_TRACE_RECORDS) {
		archive_owned = false;
	}
	reason = readl(sram + 0x40);
	/* Vendor PL may decode dual WDT A0000000 as IRQ-only BIT(2). */
	if (mt->tc_trace_previous_valid &&
	    (reason & (BIT(0) | BIT(2) | BIT(3) | BIT(5)))) {
		dev_warn(mt->dev, "previous TC generation=%u, PL reason=%#x\n",
			 generation, reason);
		for (i = 0; i < MT6797_TRACE_RECORDS; i++) {
			u32 *w = mt->tc_trace_previous[i];

			dev_warn(mt->dev, "previous TC bank=%u seq=%u commit=%u mono_ns=%llu msr=%08x,%08x,%08x,%08x protctl=%x prottc=%x\n",
				 w[3], w[0], w[15], ((u64)w[2] << 32) | w[1],
				 w[5], w[6], w[7], w[8], w[11], w[12]);
		}
		if (archive_owned) {
			writel(0, archive);
			for (i = 4; i < MT6797_TRACE_SIZE; i += 4)
				writel(readl(mt->tc_trace + i), archive + i);
			writel(reason, archive + 16);
			writel(MT6797_TRACE_MAGIC, archive);
			readl(archive);
		}
	}
	if (archive_owned && readl(archive) == MT6797_TRACE_MAGIC) {
		mt->tc_trace_failure_generation = readl(archive + 12);
		mt->tc_trace_failure_reason = readl(archive + 16);
		for (i = 0; i < MT6797_TRACE_RECORDS; i++)
			for (j = 0; j < MT6797_TRACE_WORDS; j++)
				mt->tc_trace_failure[i][j] =
					readl(archive + 64 + i * 64 + j * 4);
		mt->tc_trace_failure_valid = true;
	}
	mt->tc_trace_generation = generation + 1;
	memset_io(mt->tc_trace, 0, MT6797_TRACE_SIZE);
	writel(MT6797_TRACE_WORDS, mt->tc_trace + 4);
	writel(MT6797_TRACE_RECORDS, mt->tc_trace + 8);
	writel(mt->tc_trace_generation, mt->tc_trace + 12);
	writel(MT6797_TRACE_MAGIC, mt->tc_trace);
	readl(mt->tc_trace);
	dev_info(mt->dev, "TC SRAM trace generation=%u previous=%d\n",
		 mt->tc_trace_generation, mt->tc_trace_previous_valid);
}

static int mtk_thermal_probe(struct platform_device *pdev)
{
	int ret, i, ctrl_id;
	struct device_node *auxadc, *apmixedsys, *np = pdev->dev.of_node;
	struct mtk_thermal *mt;
	u64 auxadc_phys_base, apmixed_phys_base;
	struct thermal_zone_device *tzdev;
	void __iomem *apmixed_base, *auxadc_base;
	struct resource *thermal_res;

	mt = devm_kzalloc(&pdev->dev, sizeof(*mt), GFP_KERNEL);
	if (!mt)
		return -ENOMEM;

	mt->conf = of_device_get_match_data(&pdev->dev);

	mt->thermal_base = devm_platform_get_and_ioremap_resource(pdev, 0, &thermal_res);
	if (IS_ERR(mt->thermal_base))
		return PTR_ERR(mt->thermal_base);
	mt->thermal_phys = thermal_res->start;

	ret = mtk_thermal_get_calibration_data(&pdev->dev, mt);
	if (ret)
		return ret;

	mutex_init(&mt->lock);
	mutex_init(&mt->hw_protect_lock);

	mt->dev = &pdev->dev;
	platform_set_drvdata(pdev, mt);
	for (i = 0; i < mt->conf->num_banks; i++) {
		mt->banks[i].id = i;
		mt->banks[i].mt = mt;
	}

	auxadc = of_parse_phandle(np, "mediatek,auxadc", 0);
	if (!auxadc) {
		dev_err(&pdev->dev, "missing auxadc node\n");
		return -ENODEV;
	}

	auxadc_base = of_iomap(auxadc, 0);
	auxadc_phys_base = of_get_phys_base(auxadc);

	of_node_put(auxadc);

	if (auxadc_phys_base == OF_BAD_ADDR) {
		dev_err(&pdev->dev, "Can't get auxadc phys address\n");
		return -EINVAL;
	}

	apmixedsys = of_parse_phandle(np, "mediatek,apmixedsys", 0);
	if (!apmixedsys) {
		dev_err(&pdev->dev, "missing apmixedsys node\n");
		return -ENODEV;
	}

	apmixed_base = of_iomap(apmixedsys, 0);
	apmixed_phys_base = of_get_phys_base(apmixedsys);

	of_node_put(apmixedsys);

	if (apmixed_phys_base == OF_BAD_ADDR) {
		dev_err(&pdev->dev, "Can't get auxadc phys address\n");
		return -EINVAL;
	}
	mt->auxadc_base = auxadc_base;
	mt->apmixed_base = apmixed_base;
	mt->auxadc_phys = auxadc_phys_base;
	mt->apmixed_phys = apmixed_phys_base;
	if (mt->conf->version == MTK_THERMAL_MT6797) {
		struct platform_device *wdt;
		struct device_node *node;

		/* Resolve the supplier BEFORE starting any autonomous AHB traffic.
		 * The old late defer left running banks behind when devres gated
		 * their clocks, then attempted fast-init again on a dirty engine.
		 */
		node = of_parse_phandle(np, "mediatek,watchdog", 0);
		if (node) {
			wdt = of_find_device_by_node(node);
			of_node_put(node);
			if (!wdt)
				return -EPROBE_DEFER;
			if (!device_is_bound(&wdt->dev)) {
				put_device(&wdt->dev);
				return -EPROBE_DEFER;
			}
			mt->hw_watchdog = &wdt->dev;
			if (!device_link_add(&pdev->dev, &wdt->dev,
					     DL_FLAG_AUTOREMOVE_CONSUMER)) {
				put_device(&wdt->dev);
				return -ENOMEM;
			}
			put_device(&wdt->dev);
			ret = mt6797_wdt_thermal_request(mt->hw_watchdog, false);
			if (ret)
				return ret;
		}
	}

	ret = device_reset_optional(&pdev->dev);
	if (ret)
		return ret;

	mt->clk_auxadc = devm_clk_get_enabled(&pdev->dev, "auxadc");
	if (IS_ERR(mt->clk_auxadc)) {
		ret = PTR_ERR(mt->clk_auxadc);
		dev_err(&pdev->dev, "Can't enable auxadc clk: %d\n", ret);
		return ret;
	}

	mt->clk_peri_therm = devm_clk_get_enabled(&pdev->dev, "therm");
	if (IS_ERR(mt->clk_peri_therm)) {
		ret = PTR_ERR(mt->clk_peri_therm);
		dev_err(&pdev->dev, "Can't enable peri clk: %d\n", ret);
		return ret;
	}
	if (mt->conf->version == MTK_THERMAL_MT6797) {
		ret = devm_add_action_or_reset(&pdev->dev, mt6797_stop_on_release, mt);
		if (ret)
			return ret;
	}

	ret = mtk_thermal_turn_on_buffer(mt, apmixed_base);
	if (ret)
		return ret;

	if (mt->conf->version == MTK_THERMAL_MT6797) {
		/* BSP clears immediate and auto channel 11 before bank setup. */
		writel(readl(auxadc_base + AUXADC_CON0_V) & ~BIT(11),
		       auxadc_base + AUXADC_CON0_V);
		writel(BIT(11), auxadc_base + AUXADC_CON1_CLR_V);
	} else if (mt->conf->version != MTK_THERMAL_V1) {
		mtk_thermal_release_periodic_ts(mt, auxadc_base);
	}

	if (mt->conf->version == MTK_THERMAL_MT6797)
		mt->raw_to_mcelsius = raw_to_mcelsius_mt6797;
	else if (mt->conf->version == MTK_THERMAL_V1)
		mt->raw_to_mcelsius = raw_to_mcelsius_v1;
	else if (mt->conf->version == MTK_THERMAL_V2)
		mt->raw_to_mcelsius = raw_to_mcelsius_v2;
	else
		mt->raw_to_mcelsius = raw_to_mcelsius_v3;

	if (mt->conf->version == MTK_THERMAL_MT6797) {
		ret = mt6797_init_all_banks(mt, apmixed_phys_base, auxadc_phys_base,
					  auxadc_base);
		if (ret)
			return ret;
	} else {
		for (ctrl_id = 0; ctrl_id < mt->conf->num_controller; ctrl_id++)
			for (i = 0; i < mt->conf->num_banks; i++) {
				ret = mtk_thermal_init_bank(mt, i, apmixed_phys_base,
							  auxadc_phys_base, ctrl_id);
				if (ret)
					return ret;
			}
	}

	tzdev = devm_thermal_of_zone_register(&pdev->dev, 0, mt,
					      &mtk_thermal_ops);
	if (IS_ERR(tzdev))
		return PTR_ERR(tzdev);

	ret = devm_thermal_add_hwmon_sysfs(&pdev->dev, tzdev);
	if (ret)
		dev_warn(&pdev->dev, "error in thermal_add_hwmon_sysfs");
	if (mt->conf->version == MTK_THERMAL_MT6797) {
		int critical;

		dev_set_drvdata(&pdev->dev, mt);
		if (mt->hw_watchdog) {
			mt6797_trace_init(mt);
			ret = thermal_zone_get_crit_temp(tzdev, &critical);
			if (ret || critical > 150000 || critical < 10000)
				return -EINVAL;
			ret = mt6797_hw_protect(mt, critical);
			if (ret)
				return ret;
			dev_info(&pdev->dev, "TC direct watchdog protection=%dmC\n", critical);
		}
		return devm_device_add_group(&pdev->dev, &mt6797_thermal_group);
	}

	return 0;
}

static int mt6797_restore_thermal(struct mtk_thermal *mt)
{
	int critical = mt->hw_critical_temp;
	int ret;

	/* WDT is a supplier, so its resume has completed before this callback.
	 * Keep its thermal request off until all slots and thresholds are valid.
	 */
	ret = mt6797_wdt_thermal_request(mt->hw_watchdog, false);
	if (ret)
		return ret;
	ret = device_reset_optional(mt->dev);
	if (ret)
		return ret;
	ret = mtk_thermal_turn_on_buffer(mt, mt->apmixed_base);
	if (ret)
		return ret;
	writel(readl(mt->auxadc_base + AUXADC_CON0_V) & ~BIT(11),
	       mt->auxadc_base + AUXADC_CON0_V);
	writel(BIT(11), mt->auxadc_base + AUXADC_CON1_CLR_V);
	ret = mt6797_init_all_banks(mt, mt->apmixed_phys, mt->auxadc_phys,
				    mt->auxadc_base);
	if (ret)
		return ret;
	ret = mt6797_hw_protect(mt, critical);
	if (ret)
		return ret;
	return mt6797_wdt_thermal_request(mt->hw_watchdog, true);
}

static int mtk_thermal_suspend(struct device *dev)
{
	struct mtk_thermal *mt = dev_get_drvdata(dev);
	u32 value;
	int ret, restore_ret;

	if (mt->conf->version != MTK_THERMAL_MT6797 || !mt->hw_watchdog)
		return 0;
	/* Thermal core has stopped polling; OTP/EEM are linked consumers and
	 * have quiesced before us. BSP keeps the clocks on but stops all AHB
	 * engines before powering down the analog buffer.
	 */
	ret = mt6797_wdt_thermal_request(mt->hw_watchdog, false);
	if (ret) {
		mt->pm_failures++;
		mt->pm_last_error = ret;
		return ret;
	}
	ret = mt6797_stop_all_banks(mt);
	if (ret)
		goto restore;
	value = readl(mt->apmixed_base + APMIXED_SYS_TS_CON1);
	writel(value | GENMASK(5, 4), mt->apmixed_base + APMIXED_SYS_TS_CON1);
	if ((readl(mt->apmixed_base + APMIXED_SYS_TS_CON1) & GENMASK(5, 4)) !=
	    GENMASK(5, 4)) {
		ret = -EIO;
		goto restore;
	}
	mt->pm_suspended = true;
	mt->pm_suspends++;
	dev_info(dev, "BSP suspend: banks stopped, AHB drained, TS buffer off\n");
	return 0;

restore:
	mt->pm_failures++;
	mt->pm_last_error = ret;
	/* A failed suspend callback is not resumed by the PM core. Restore
	 * working sampling/protection here before returning the abort reason.
	 */
	restore_ret = mt6797_restore_thermal(mt);
	if (restore_ret) {
		dev_emerg(dev, "cannot restore thermal after aborted suspend: %d\n",
			  restore_ret);
		emergency_restart();
	}
	return ret;
}

static int mtk_thermal_resume(struct device *dev)
{
	struct mtk_thermal *mt = dev_get_drvdata(dev);
	int ret;

	if (!mt->pm_suspended)
		return 0;
	ret = mt6797_restore_thermal(mt);
	if (ret) {
		mt->pm_failures++;
		mt->pm_last_error = ret;
		dev_emerg(dev, "cannot resume thermal sampling/protection: %d\n", ret);
		emergency_restart();
		return ret;
	}
	mt->pm_suspended = false;
	mt->pm_resumes++;
	mt->pm_last_error = 0;
	dev_info(dev, "BSP resume: controller reset, banks seeded, protection=%dmC\n",
		 mt->hw_critical_temp);
	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(mtk_thermal_pm_ops, mtk_thermal_suspend,
			      mtk_thermal_resume);

static struct platform_driver mtk_thermal_driver = {
	.probe = mtk_thermal_probe,
	.driver = {
		.name = "mtk-thermal",
		.of_match_table = mtk_thermal_of_match,
		.pm = pm_sleep_ptr(&mtk_thermal_pm_ops),
	},
};

module_platform_driver(mtk_thermal_driver);

MODULE_AUTHOR("Michael Kao <michael.kao@mediatek.com>");
MODULE_AUTHOR("Louis Yu <louis.yu@mediatek.com>");
MODULE_AUTHOR("Dawei Chien <dawei.chien@mediatek.com>");
MODULE_AUTHOR("Sascha Hauer <s.hauer@pengutronix.de>");
MODULE_AUTHOR("Hanyi Wu <hanyi.wu@mediatek.com>");
MODULE_DESCRIPTION("Mediatek thermal driver");
MODULE_LICENSE("GPL v2");
