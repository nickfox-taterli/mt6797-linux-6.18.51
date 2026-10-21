/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __MT6797_ARMCORE_DCM_H
#define __MT6797_ARMCORE_DCM_H

#include <linux/types.h>

struct device;

#define MT6797_ARMCORE_DCM_MASK	0x7777
#define MT6797_ARMCORE_DCM_MODE1	0x1111

/*
 * The consumer must hold cpus_read_lock() and a managed device link to the
 * cpufreq supplier. Access shares its mutex, M0 semaphore and bridge helpers.
 * Only ARMPLLDIV_DCMCTL's BSP-owned fields can be written; no PLL access.
 */
int mt6797_armcore_dcm_access(struct device *supplier, u32 *value, bool write);

#endif
