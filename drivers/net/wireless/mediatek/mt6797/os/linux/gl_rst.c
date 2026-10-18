// SPDX-License-Identifier: GPL-2.0-only
/* A firmware assertion requests asynchronous shared WMT radio recovery. */
#include "precomp.h"

BOOLEAN fgIsResetting;
UINT32 g_IsNeedDoChipReset;
static struct work_struct reset_work;
static atomic_t reset_pending = ATOMIC_INIT(0);

static void wlan_reset_work(struct work_struct *work)
{
	int ret;

	WRITE_ONCE(fgIsResetting, TRUE);
	ret = mt6797_wlan_restart();
	WRITE_ONCE(fgIsResetting, FALSE);
	atomic_set(&reset_pending, 0);
	if (ret)
		pr_err("mt6797-wlan: recovery failed: %d\n", ret);
	else
		pr_info("mt6797-wlan: shared radio recovery requested\n");
}

VOID glResetInit(VOID)
{
	INIT_WORK(&reset_work, wlan_reset_work);
	atomic_set(&reset_pending, 0);
	WRITE_ONCE(fgIsResetting, FALSE);
}

VOID glResetUninit(VOID)
{
	cancel_work_sync(&reset_work);
}

BOOLEAN kalIsResetting(VOID)
{
	return READ_ONCE(fgIsResetting);
}

BOOLEAN glResetTrigger(P_ADAPTER_T adapter)
{
	if (atomic_cmpxchg(&reset_pending, 0, 1))
		return FALSE;
	if (!schedule_work(&reset_work)) {
		atomic_set(&reset_pending, 0);
		return FALSE;
	}
	return TRUE;
}

VOID glSendResetRequest(VOID)
{
	glResetTrigger(NULL);
}
