/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __MT6797_DCM_H
#define __MT6797_DCM_H

#include <linux/types.h>

struct device;

/*
 * Serialize a firmware memory transition against MEM/DFS_MEM configuration.
 * The callback runs with the DCM mutex held and reports whether hardware is
 * quiescent. Restore the user's latched DCM settings only when it is safe.
 * An unfinished transition blocks manual MEM writes until recovery succeeds.
 * The caller must hold a managed device link to the DCM supplier.
 */
int mt6797_dcm_memory_transaction(struct device *supplier,
		int (*transition)(void *data, bool *quiescent), void *data);

#endif
