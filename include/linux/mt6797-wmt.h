/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __LINUX_MT6797_WMT_H
#define __LINUX_MT6797_WMT_H

#include <linux/types.h>

struct device;
struct mt6797_stp_session;
struct mt6797_wmt_client;

struct mt6797_wmt_client_ops {
	/* Queue received bytes; callbacks must not call blocking WMT APIs. */
	void (*receive)(void *context, const u8 *data, size_t length);
};

struct mt6797_wmt_client *mt6797_wmt_client_get(struct device *dev, u8 task,
				const struct mt6797_wmt_client_ops *ops, void *context);
void mt6797_wmt_client_put(struct mt6797_wmt_client *client);
int mt6797_wmt_client_open(struct mt6797_wmt_client *client);
int mt6797_wmt_client_close(struct mt6797_wmt_client *client);
int mt6797_wmt_client_send(struct mt6797_wmt_client *client,
			   const void *data, size_t length);
int mt6797_wmt_wifi_emi_write(struct mt6797_wmt_client *client, u32 offset,
			    const void *data, size_t length);
int mt6797_wmt_client_version(struct mt6797_wmt_client *client, u32 *hw, u32 *fw);
int mt6797_wmt_client_cpupcr(struct mt6797_wmt_client *client, u32 *value);

#define MT6797_WMT_CALIBRATION_MAX 2044

struct mt6797_wmt_calibration {
	u32 hw_version;
	u32 fw_version;
	u16 length;
	u8 data[MT6797_WMT_CALIBRATION_MAX];
};

struct mt6797_wmt_config {
	u8 antenna_mode;
	u8 fm_transport;
	bool co_clock;
};

/* Caller owns the powered ConSys/STP session for the complete transaction. */
int mt6797_wmt_read_version(struct device *dev,
			    struct mt6797_stp_session *session,
			    u32 *hw_version, u32 *fw_version);
int mt6797_wmt_download_patches(struct device *dev,
				struct mt6797_stp_session *session,
				void *host_context, u32 hw_version, u32 fw_version);
int mt6797_wmt_function_control(struct device *dev,
				struct mt6797_stp_session *session,
				u8 function, bool enabled);
int mt6797_wmt_post_patch_init(struct device *dev,
			      struct mt6797_stp_session *session,
			      void *host_context, const struct mt6797_wmt_config *config,
			      struct mt6797_wmt_calibration *calibration);
int mt6797_wmt_bootstrap(struct device *dev, struct mt6797_stp_session *session,
			 void *host_context, const struct mt6797_wmt_config *config,
			      struct mt6797_wmt_calibration *calibration);

#endif
