/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __LINUX_MT6797_WMT_H
#define __LINUX_MT6797_WMT_H

#include <linux/types.h>

struct device;
struct mt6797_stp_session;

/* Caller owns the powered ConSys/STP session for the complete transaction. */
int mt6797_wmt_read_version(struct device *dev,
			    struct mt6797_stp_session *session,
			    u32 *hw_version, u32 *fw_version);
int mt6797_wmt_download_patches(struct device *dev,
				struct mt6797_stp_session *session,
				void *host_context, u32 hw_version, u32 fw_version);

#endif
