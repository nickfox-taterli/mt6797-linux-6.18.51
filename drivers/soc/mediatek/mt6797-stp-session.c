// SPDX-License-Identifier: GPL-2.0-only
//
// MT6797 reliable full-mode STP/WMT session.
//
// This is intentionally a library-style module.  It claims BTIF only when a
// higher-level ConSys owner explicitly starts a session.  It does not power
// VCN rails, select an STP mode, or probe the chip by itself.

#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/jiffies.h>
#include <linux/kfifo.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

#include <linux/mt6797-btif.h>
#include <linux/mt6797-stp-session.h>
#include <linux/mt6797-stp.h>

#define MT6797_STP_WINDOW_SIZE		7
#define MT6797_STP_RETRY_LIMIT		10
#define MT6797_STP_ACK_TIMEOUT_MS	180
#define MT6797_STP_RX_FIFO_SIZE		8192
#define MT6797_STP_RESYNC_SIZE		4
#define MT6797_STP_WMT_HEADER_SIZE	4
#define MT6797_STP_FRAME_SIZE		(2 + MT6797_STP_HEADER_SIZE + \
					 MT6797_STP_MAX_PAYLOAD + MT6797_STP_CRC_SIZE)

struct mt6797_stp_tx_slot {
	u8 frame[MT6797_STP_FRAME_SIZE];
	u16 length;
	bool pending;
	bool acked;
};

struct mt6797_stp_session {
	struct device *owner;
	struct mt6797_stp_session_ops ops;
	void *context;
	struct mt6797_stp_host_ops host_ops;
	void *host_context;
	struct mt6797_btif_client_ops btif_ops;

	struct mt6797_stp_decoder decoder;
	DECLARE_KFIFO(rx_fifo, u8, MT6797_STP_RX_FIFO_SIZE);
	spinlock_t rx_lock;
	struct work_struct rx_work;

	/* send_lock serializes synchronous callers; tx_io_lock serializes bytes. */
	struct mutex send_lock;
	struct mutex tx_io_lock;
	struct mutex host_config_lock;
	spinlock_t state_lock;
	wait_queue_head_t ack_wait;
	struct completion response_done;

	struct mt6797_stp_tx_slot slots[8];
	u8 tx_next;
	u8 tx_acked;
	u8 rx_expected;
	u8 rx_ack;
	u8 retries;
	unsigned int outstanding;
	bool retry_requested;
	bool active;
	bool delimiter;
	bool host_enabled;
	int terminal_error;

	bool response_waiting;
	u8 response_opcode;
	u8 response[MT6797_STP_MAX_PAYLOAD];
	u8 wmt_pdu[MT6797_STP_MAX_PAYLOAD];
	u16 response_length;
	int response_status;
};

static bool mt6797_stp_session_active(struct mt6797_stp_session *session)
{
	return READ_ONCE(session->active);
}

static u8 mt6797_stp_next_seq(u8 sequence)
{
	return (sequence + 1) & 7;
}

static void mt6797_stp_set_error(struct mt6797_stp_session *session, int error)
{
	unsigned long flags;

	spin_lock_irqsave(&session->state_lock, flags);
	if (!session->terminal_error)
		session->terminal_error = error;
	spin_unlock_irqrestore(&session->state_lock, flags);
	wake_up_all(&session->ack_wait);
	complete_all(&session->response_done);
}

static void mt6797_stp_btif_rx(void *context, const u8 *data, size_t length)
{
	struct mt6797_stp_session *session = context;

	if (!mt6797_stp_session_active(session) || !length)
		return;
	if (kfifo_in_spinlocked(&session->rx_fifo, data, length,
				       &session->rx_lock) != length)
		mt6797_stp_set_error(session, -ENOBUFS);
	schedule_work(&session->rx_work);
}

static bool mt6797_stp_slot_acked(struct mt6797_stp_session *session,
					  struct mt6797_stp_tx_slot *slot)
{
	unsigned long flags;
	bool done;

	spin_lock_irqsave(&session->state_lock, flags);
	done = slot->acked || session->terminal_error;
	spin_unlock_irqrestore(&session->state_lock, flags);
	return done;
}

static void mt6797_stp_update_frame_ack(struct mt6797_stp_session *session,
						struct mt6797_stp_tx_slot *slot)
{
	u8 *header = slot->frame + (session->delimiter ? 2 : 0);

	header[0] = (header[0] & 0xf8) | session->rx_ack;
	header[3] = header[0] + header[1] + header[2];
}

static void mt6797_stp_process_ack(struct mt6797_stp_session *session,
					   u8 acknowledgement, bool nak)
{
	unsigned long flags;
	u8 sequence;
	bool found = false;

	spin_lock_irqsave(&session->state_lock, flags);
	if (nak)
		session->retry_requested = true;
	sequence = mt6797_stp_next_seq(session->tx_acked);
	while (sequence != session->tx_next) {
		if (sequence == acknowledgement) {
			found = true;
			break;
		}
		sequence = mt6797_stp_next_seq(sequence);
	}
	if (found) {
		sequence = mt6797_stp_next_seq(session->tx_acked);
		while (true) {
			struct mt6797_stp_tx_slot *slot = &session->slots[sequence];

			slot->pending = false;
			slot->acked = true;
			if (session->outstanding)
				session->outstanding--;
			session->tx_acked = sequence;
			if (sequence == acknowledgement)
				break;
			sequence = mt6797_stp_next_seq(sequence);
		}
		session->retries = 0;
		session->retry_requested = false;
	}
	spin_unlock_irqrestore(&session->state_lock, flags);
	wake_up_all(&session->ack_wait);
}

static int mt6797_stp_write(struct mt6797_stp_session *session,
				    const u8 *data, size_t length,
				    unsigned long timeout)
{
	ssize_t written;

	if (!mt6797_stp_session_active(session))
		return -ESHUTDOWN;
	mutex_lock(&session->tx_io_lock);
	written = mt6797_btif_write(session->owner, data, length, timeout);
	mutex_unlock(&session->tx_io_lock);
	if (written == length)
		return 0;
	if (written < 0)
		return written;
	return -EIO;
}

static int mt6797_stp_send_ack(struct mt6797_stp_session *session, u8 ack,
				       bool nak)
{
	u8 frame[MT6797_STP_HEADER_SIZE + 2];
	ssize_t length;

	length = mt6797_stp_encode_frame(frame, sizeof(frame), 0, 0, ack, nak,
					 NULL, 0, session->delimiter);
	if (length < 0)
		return length;
	return mt6797_stp_write(session, frame, length,
				msecs_to_jiffies(MT6797_STP_ACK_TIMEOUT_MS));
}

static int mt6797_stp_send_resync(struct mt6797_stp_session *session)
{
	static const u8 resync[MT6797_STP_RESYNC_SIZE] = { 0x7f, 0x7f, 0x7f, 0x7f };

	return mt6797_stp_write(session, resync, sizeof(resync),
				msecs_to_jiffies(MT6797_STP_ACK_TIMEOUT_MS));
}

static int mt6797_stp_send_pending(struct mt6797_stp_session *session)
{
	u8 sequence;
	int ret;

	spin_lock_irq(&session->state_lock);
	sequence = mt6797_stp_next_seq(session->tx_acked);
	while (sequence != session->tx_next) {
		struct mt6797_stp_tx_slot *slot = &session->slots[sequence];
		u16 length = slot->length;

		if (slot->pending) {
			mt6797_stp_update_frame_ack(session, slot);
			spin_unlock_irq(&session->state_lock);
			ret = mt6797_stp_write(session, slot->frame, length,
					msecs_to_jiffies(MT6797_STP_ACK_TIMEOUT_MS));
			if (ret)
				return ret;
			spin_lock_irq(&session->state_lock);
		}
		sequence = mt6797_stp_next_seq(sequence);
	}
	spin_unlock_irq(&session->state_lock);
	return 0;
}

static void mt6797_stp_wmt_event(struct mt6797_stp_session *session,
					const u8 *payload, u16 length)
{
	unsigned long flags;
	u16 parameter_length;
	bool complete_response = false;

	if (length < MT6797_STP_WMT_HEADER_SIZE)
		return;
	parameter_length = payload[2] | (payload[3] << 8);
	spin_lock_irqsave(&session->state_lock, flags);
	if (session->response_waiting && payload[0] == 2 &&
		payload[1] == session->response_opcode) {
		if (parameter_length != length - MT6797_STP_WMT_HEADER_SIZE) {
			session->response_status = -EPROTO;
		} else if (parameter_length > sizeof(session->response)) {
			session->response_status = -EMSGSIZE;
		} else {
			memcpy(session->response, payload + MT6797_STP_WMT_HEADER_SIZE,
			       parameter_length);
			session->response_length = parameter_length;
			session->response_status = 0;
		}
		session->response_waiting = false;
		complete_response = true;
	}
	spin_unlock_irqrestore(&session->state_lock, flags);
	if (complete_response)
		complete(&session->response_done);
}

static int mt6797_stp_frame(struct mt6797_stp_session *session,
				    const struct mt6797_stp_frame *frame)
{
	unsigned long flags;
	u8 ack;
	bool in_order;

	mt6797_stp_process_ack(session, frame->acknowledgement, frame->nak);
	if (!frame->payload_length && !frame->has_crc)
		return 0;

	spin_lock_irqsave(&session->state_lock, flags);
	in_order = frame->sequence == session->rx_expected;
	ack = session->rx_ack;
	if (in_order) {
		session->rx_ack = frame->sequence;
		session->rx_expected = mt6797_stp_next_seq(frame->sequence);
		ack = session->rx_ack;
	}
	spin_unlock_irqrestore(&session->state_lock, flags);

	if (!in_order) {
		mt6797_stp_send_ack(session, ack, true);
		return 0;
	}
	/* ACK before handing data to a possibly slow client. */
	if (mt6797_stp_send_ack(session, ack, false))
		mt6797_stp_set_error(session, -EIO);
	if (frame->packet_type == MT6797_STP_TASK_WMT)
		mt6797_stp_wmt_event(session, frame->payload, frame->payload_length);
	if (session->ops.rx_frame)
		session->ops.rx_frame(session->context, frame->packet_type,
				      frame->payload, frame->payload_length);
	return 0;
}

static int mt6797_stp_decoder_frame(void *context,
					const struct mt6797_stp_frame *frame)
{
	return mt6797_stp_frame(context, frame);
}

static void mt6797_stp_rx_work(struct work_struct *work)
{
	struct mt6797_stp_session *session =
		container_of(work, struct mt6797_stp_session, rx_work);
	u8 data[256];
	unsigned int length;

	if (!mt6797_stp_session_active(session))
		return;
	while ((length = kfifo_out_spinlocked(&session->rx_fifo, data,
						      sizeof(data), &session->rx_lock))) {
		if (mt6797_stp_decoder_feed(&session->decoder, data, length,
					     mt6797_stp_decoder_frame, session) < 0)
			mt6797_stp_set_error(session, -EPROTO);
		if (!mt6797_stp_session_active(session))
			break;
	}
}

static int mt6797_stp_queue_frame(struct mt6797_stp_session *session,
					  u8 task, const u8 *payload, u16 length,
					  struct mt6797_stp_tx_slot **queued)
{
	struct mt6797_stp_tx_slot *slot;
	unsigned long flags;
	ssize_t encoded;
	u8 sequence;

	spin_lock_irqsave(&session->state_lock, flags);
	if (!session->active || session->terminal_error) {
		spin_unlock_irqrestore(&session->state_lock, flags);
		return session->terminal_error ?: -ESHUTDOWN;
	}
	if (session->outstanding >= MT6797_STP_WINDOW_SIZE) {
		spin_unlock_irqrestore(&session->state_lock, flags);
		return -EAGAIN;
	}
	sequence = session->tx_next;
	slot = &session->slots[sequence];
	encoded = mt6797_stp_encode_frame(slot->frame, sizeof(slot->frame), task,
					 sequence, session->rx_ack, false, payload,
					 length, session->delimiter);
	if (encoded < 0) {
		spin_unlock_irqrestore(&session->state_lock, flags);
		return encoded;
	}
	slot->length = encoded;
	slot->pending = true;
	slot->acked = false;
	session->outstanding++;
	session->tx_next = mt6797_stp_next_seq(sequence);
	*queued = slot;
	spin_unlock_irqrestore(&session->state_lock, flags);
	return 0;
}

static void mt6797_stp_abort_tx(struct mt6797_stp_session *session)
{
	unsigned long flags;
	unsigned int index;

	spin_lock_irqsave(&session->state_lock, flags);
	for (index = 0; index < ARRAY_SIZE(session->slots); index++) {
		session->slots[index].pending = false;
		session->slots[index].acked = false;
	}
	session->outstanding = 0;
	session->tx_acked = (session->tx_next + 7) & 7;
	session->retry_requested = false;
	spin_unlock_irqrestore(&session->state_lock, flags);
}

static int mt6797_stp_send_locked(struct mt6797_stp_session *session,
					   u8 task, const u8 *payload, u16 length,
					   unsigned long timeout)
{
	struct mt6797_stp_tx_slot *slot;
	unsigned long remaining;
	int ret;
	unsigned int attempt;

	ret = mt6797_stp_queue_frame(session, task, payload, length, &slot);
	if (ret)
		return ret;
	for (attempt = 0; attempt <= MT6797_STP_RETRY_LIMIT; attempt++) {
		if (attempt) {
			ret = mt6797_stp_send_resync(session);
			if (!ret)
				ret = mt6797_stp_send_pending(session);
		} else {
			ret = mt6797_stp_write(session, slot->frame, slot->length,
					msecs_to_jiffies(MT6797_STP_ACK_TIMEOUT_MS));
		}
		if (ret)
			break;
		remaining = wait_event_interruptible_timeout(session->ack_wait,
					     mt6797_stp_slot_acked(session, slot) ||
					     READ_ONCE(session->retry_requested),
					     timeout);
		if (mt6797_stp_slot_acked(session, slot)) {
			ret = session->terminal_error;
			if (!ret)
				ret = 0;
			break;
		}
		if (remaining < 0) {
			ret = -ERESTARTSYS;
			break;
		}
		if (!remaining && attempt == MT6797_STP_RETRY_LIMIT) {
			ret = -ETIMEDOUT;
			break;
		}
		spin_lock_irq(&session->state_lock);
		session->retry_requested = false;
		session->retries = attempt + 1;
		spin_unlock_irq(&session->state_lock);
	}
	if (ret)
		mt6797_stp_abort_tx(session);
	return ret;
}

struct mt6797_stp_session *mt6797_stp_session_alloc(
	struct device *owner, const struct mt6797_stp_session_ops *ops,
	void *context)
{
	struct mt6797_stp_session *session;

	if (!owner)
		return NULL;
	session = kzalloc(sizeof(*session), GFP_KERNEL);
	if (!session)
		return NULL;
	session->owner = owner;
	if (ops)
		session->ops = *ops;
	session->context = context;
	session->btif_ops.rx_bytes = mt6797_stp_btif_rx;
	INIT_KFIFO(session->rx_fifo);
	spin_lock_init(&session->rx_lock);
	spin_lock_init(&session->state_lock);
	mutex_init(&session->send_lock);
	mutex_init(&session->tx_io_lock);
	mutex_init(&session->host_config_lock);
	init_waitqueue_head(&session->ack_wait);
	init_completion(&session->response_done);
	INIT_WORK(&session->rx_work, mt6797_stp_rx_work);
	mt6797_stp_decoder_init(&session->decoder);
	return session;
}
EXPORT_SYMBOL_GPL(mt6797_stp_session_alloc);

void mt6797_stp_session_stop(struct mt6797_stp_session *session)
{
	unsigned long flags;
	bool was_active;
	bool host_enabled;
	int ret;

	if (!session)
		return;
	mutex_lock(&session->host_config_lock);
	spin_lock_irqsave(&session->state_lock, flags);
	host_enabled = session->host_enabled;
	spin_unlock_irqrestore(&session->state_lock, flags);
	if (host_enabled && session->host_ops.set_enabled) {
		ret = session->host_ops.set_enabled(session->host_context, false);
		if (ret)
			dev_warn(session->owner,
				 "failed to disable host STP before stop: %d\n", ret);
	}
	spin_lock_irqsave(&session->state_lock, flags);
	was_active = session->active;
	session->active = false;
	session->host_enabled = false;
	session->terminal_error = -ESHUTDOWN;
	session->response_waiting = false;
	spin_unlock_irqrestore(&session->state_lock, flags);
	mutex_unlock(&session->host_config_lock);
	wake_up_all(&session->ack_wait);
	complete_all(&session->response_done);
	if (was_active)
		mt6797_btif_release(session->owner);
	cancel_work_sync(&session->rx_work);
}
EXPORT_SYMBOL_GPL(mt6797_stp_session_stop);

void mt6797_stp_session_free(struct mt6797_stp_session *session)
{
	if (!session)
		return;
	mt6797_stp_session_stop(session);
	kfree(session);
}
EXPORT_SYMBOL_GPL(mt6797_stp_session_free);

int mt6797_stp_session_start(struct mt6797_stp_session *session, bool delimiter)
{
	unsigned long flags;
	int ret;

	if (!session)
		return -EINVAL;
	mutex_lock(&session->host_config_lock);
	spin_lock_irqsave(&session->state_lock, flags);
	if (session->active) {
		spin_unlock_irqrestore(&session->state_lock, flags);
		mutex_unlock(&session->host_config_lock);
		return -EBUSY;
	}
	session->delimiter = delimiter;
	session->terminal_error = 0;
	session->tx_next = 0;
	session->tx_acked = 7;
	session->rx_expected = 0;
	session->rx_ack = 7;
	session->retries = 0;
	session->outstanding = 0;
	session->retry_requested = false;
	session->host_enabled = false;
	memset(session->slots, 0, sizeof(session->slots));
	spin_lock(&session->rx_lock);
	kfifo_reset(&session->rx_fifo);
	spin_unlock(&session->rx_lock);
	mt6797_stp_decoder_init(&session->decoder);
	session->active = true;
	spin_unlock_irqrestore(&session->state_lock, flags);
	mutex_unlock(&session->host_config_lock);
	ret = mt6797_btif_claim(session->owner, &session->btif_ops, session);
	if (ret) {
		spin_lock_irqsave(&session->state_lock, flags);
		session->active = false;
		session->terminal_error = ret;
		spin_unlock_irqrestore(&session->state_lock, flags);
	}
	return ret;
}
EXPORT_SYMBOL_GPL(mt6797_stp_session_start);

int mt6797_stp_session_wake(struct mt6797_stp_session *session)
{
	if (!session || !mt6797_stp_session_active(session))
		return -ENODEV;
	return mt6797_btif_wake(session->owner);
}
EXPORT_SYMBOL_GPL(mt6797_stp_session_wake);

int mt6797_stp_session_set_host_ops(struct mt6797_stp_session *session,
					const struct mt6797_stp_host_ops *ops,
					void *context)
{
	unsigned long flags;

	if (!session || !ops || !ops->set_mode || !ops->set_enabled)
		return -EINVAL;
	mutex_lock(&session->host_config_lock);
	spin_lock_irqsave(&session->state_lock, flags);
	if (session->active || session->host_enabled) {
		spin_unlock_irqrestore(&session->state_lock, flags);
		mutex_unlock(&session->host_config_lock);
		return -EBUSY;
	}
	session->host_ops = *ops;
	session->host_context = context;
	spin_unlock_irqrestore(&session->state_lock, flags);
	mutex_unlock(&session->host_config_lock);
	return 0;
}
EXPORT_SYMBOL_GPL(mt6797_stp_session_set_host_ops);

int mt6797_stp_session_send(struct mt6797_stp_session *session, u8 task,
				    const u8 *payload, u16 length,
				    unsigned long timeout_jiffies)
{
	int ret;

	if (!session || (!payload && length) || !timeout_jiffies)
		return -EINVAL;
	mutex_lock(&session->send_lock);
	ret = mt6797_stp_send_locked(session, task, payload, length,
					     timeout_jiffies);
	mutex_unlock(&session->send_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(mt6797_stp_session_send);

int mt6797_stp_session_wmt(struct mt6797_stp_session *session, u8 opcode,
				   const u8 *params, u16 parameter_length,
				   u8 *response, u16 *response_length,
				   size_t response_size,
				   unsigned long timeout_jiffies)
{
	unsigned long flags;
	unsigned long remaining;
	int ret;

	if (!session || (!params && parameter_length) ||
	    parameter_length > MT6797_STP_MAX_PAYLOAD - MT6797_STP_WMT_HEADER_SIZE ||
	    !response_length || (!response && response_size) || !timeout_jiffies)
		return -EINVAL;
	session->wmt_pdu[0] = 1;
	session->wmt_pdu[1] = opcode;
	session->wmt_pdu[2] = parameter_length & 0xff;
	session->wmt_pdu[3] = parameter_length >> 8;
	if (parameter_length)
		memcpy(session->wmt_pdu + MT6797_STP_WMT_HEADER_SIZE, params,
		       parameter_length);

	mutex_lock(&session->send_lock);
	spin_lock_irqsave(&session->state_lock, flags);
	if (!session->active || session->terminal_error) {
		ret = session->terminal_error ?: -ESHUTDOWN;
		spin_unlock_irqrestore(&session->state_lock, flags);
		goto out_unlock;
	}
	reinit_completion(&session->response_done);
	session->response_opcode = opcode;
	session->response_length = 0;
	session->response_status = -ETIMEDOUT;
	session->response_waiting = true;
	spin_unlock_irqrestore(&session->state_lock, flags);

	ret = mt6797_stp_send_locked(session, MT6797_STP_TASK_WMT, session->wmt_pdu,
					     parameter_length + MT6797_STP_WMT_HEADER_SIZE,
					     timeout_jiffies);
	if (ret < 0) {
		spin_lock_irqsave(&session->state_lock, flags);
		session->response_waiting = false;
		spin_unlock_irqrestore(&session->state_lock, flags);
		goto out_unlock;
	}
	remaining = wait_for_completion_timeout(&session->response_done,
					       timeout_jiffies);
	if (!remaining) {
		ret = -ETIMEDOUT;
		spin_lock_irqsave(&session->state_lock, flags);
		session->response_waiting = false;
		spin_unlock_irqrestore(&session->state_lock, flags);
		goto out_unlock;
	}
	spin_lock_irqsave(&session->state_lock, flags);
	ret = session->response_status;
	if (!ret) {
		if (response_size < session->response_length) {
			ret = -ENOSPC;
		} else if (session->response_length) {
			memcpy(response, session->response, session->response_length);
		}
		if (!ret)
			*response_length = session->response_length;
	}
	spin_unlock_irqrestore(&session->state_lock, flags);

out_unlock:
	mutex_unlock(&session->send_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(mt6797_stp_session_wmt);

static int mt6797_stp_expect_wmt(struct mt6797_stp_session *session,
					const u8 *params, u16 parameter_length,
					const u8 *expected, u16 expected_length,
					unsigned long timeout_jiffies)
{
	u8 response[MT6797_STP_WMT_HEADER_SIZE + 6];
	u16 response_length = 0;
	int ret;

	if (expected_length > sizeof(response))
		return -EINVAL;
	ret = mt6797_stp_session_wmt(session, 0x04, params, parameter_length,
					     response, &response_length,
					     sizeof(response), timeout_jiffies);
	if (ret)
		return ret;
	if (response_length != expected_length ||
	    memcmp(response, expected, expected_length))
		return -EPROTO;
	return 0;
}

int mt6797_stp_session_configure_btif_full(
					struct mt6797_stp_session *session,
					unsigned long timeout_jiffies)
{
	static const u8 query_params[] = { 0x04 };
	static const u8 default_response[] = {
		0x00, 0x04, 0x11, 0x00, 0x00, 0x00,
	};
	static const u8 set_params[] = {
		0x03, 0xdf, 0x0e, 0x68, 0x01,
	};
	static const u8 set_response[] = { 0x00, 0x03 };
	static const u8 full_response[] = {
		0x00, 0x04, 0xdf, 0x0e, 0x68, 0x01,
	};
	struct mt6797_stp_host_ops host_ops;
	void *host_context;
	unsigned long flags;
	bool host_enabled = false;
	int ret;
	int disable_ret;

	if (!session || !timeout_jiffies)
		return -EINVAL;
	mutex_lock(&session->host_config_lock);
	spin_lock_irqsave(&session->state_lock, flags);
	if (!session->active || session->terminal_error) {
		ret = session->terminal_error ?: -ESHUTDOWN;
		spin_unlock_irqrestore(&session->state_lock, flags);
		goto out_unlock;
	}
	host_ops = session->host_ops;
	host_context = session->host_context;
	spin_unlock_irqrestore(&session->state_lock, flags);
	if (!host_ops.set_mode || !host_ops.set_enabled) {
		ret = -EOPNOTSUPP;
		goto out_unlock;
	}

	ret = mt6797_stp_expect_wmt(session, query_params,
					ARRAY_SIZE(query_params), default_response,
					ARRAY_SIZE(default_response), timeout_jiffies);
	if (ret)
		goto out_unlock;
	ret = mt6797_stp_expect_wmt(session, set_params, ARRAY_SIZE(set_params),
					set_response, ARRAY_SIZE(set_response),
					timeout_jiffies);
	if (ret)
		goto out_unlock;

	ret = host_ops.set_mode(host_context, MT6797_STP_BTIF_FULL_MODE);
	if (ret)
		goto out_unlock;
	ret = host_ops.set_enabled(host_context, true);
	if (ret)
		goto out_unlock;
	host_enabled = true;

	/* The vendor sequence requires at least 2 ms; retain its 10 ms margin. */
	msleep(10);
	ret = mt6797_stp_expect_wmt(session, query_params,
					ARRAY_SIZE(query_params), full_response,
					ARRAY_SIZE(full_response), timeout_jiffies);
	if (!ret) {
		spin_lock_irqsave(&session->state_lock, flags);
		session->host_enabled = true;
		spin_unlock_irqrestore(&session->state_lock, flags);
		goto out_unlock;
	}

	/* Do not leave a platform-owned host parser enabled after a failed query. */
	disable_ret = host_ops.set_enabled(host_context, false);
	if (disable_ret)
		dev_warn(session->owner,
			 "failed to roll back host STP enable after init error: %d\n",
			 disable_ret);

out_unlock:
	if (ret && host_enabled) {
		spin_lock_irqsave(&session->state_lock, flags);
		session->host_enabled = false;
		spin_unlock_irqrestore(&session->state_lock, flags);
	}
	mutex_unlock(&session->host_config_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(mt6797_stp_session_configure_btif_full);

static int __init mt6797_stp_session_init(void)
{
	return 0;
}
module_init(mt6797_stp_session_init);

static void __exit mt6797_stp_session_exit(void)
{
}
module_exit(mt6797_stp_session_exit);

MODULE_DESCRIPTION("MT6797 reliable ConSys STP/WMT session");
MODULE_LICENSE("GPL");
