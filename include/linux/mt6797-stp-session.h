/* SPDX-License-Identifier: GPL-2.0-only */
/* Reliable STP/WMT session on top of the MT6797 BTIF byte transport. */
#ifndef __LINUX_MT6797_STP_SESSION_H
#define __LINUX_MT6797_STP_SESSION_H

#include <linux/device.h>
#include <linux/types.h>

struct mt6797_stp_session;

/* Task indices are the values used by the vendor MTKSTP stack. */
#define MT6797_STP_TASK_BT	0
#define MT6797_STP_TASK_FM	1
#define MT6797_STP_TASK_GPS	2
#define MT6797_STP_TASK_WIFI	3
#define MT6797_STP_TASK_WMT	4

struct mt6797_stp_session_ops {
	/* Called from the session workqueue after an in-order STP data frame. */
	void (*rx_frame)(void *context, u8 task, const u8 *payload,
			u16 length);
};

struct mt6797_stp_session *mt6797_stp_session_alloc(
	struct device *owner, const struct mt6797_stp_session_ops *ops,
	void *context);
void mt6797_stp_session_free(struct mt6797_stp_session *session);

/* Claim/release BTIF only. ConSys power and STP mode selection stay above it. */
int mt6797_stp_session_start(struct mt6797_stp_session *session,
				     bool delimiter);
void mt6797_stp_session_stop(struct mt6797_stp_session *session);
int mt6797_stp_session_wake(struct mt6797_stp_session *session);

/* Send one reliable STP data frame and wait for its cumulative ACK. */
int mt6797_stp_session_send(struct mt6797_stp_session *session, u8 task,
				    const u8 *payload, u16 length,
				    unsigned long timeout_jiffies);

/* Send one WMT command and return the matching event parameters. */
int mt6797_stp_session_wmt(struct mt6797_stp_session *session, u8 opcode,
				   const u8 *params, u16 parameter_length,
				   u8 *response, u16 *response_length,
				   size_t response_size,
				   unsigned long timeout_jiffies);

#endif /* __LINUX_MT6797_STP_SESSION_H */
