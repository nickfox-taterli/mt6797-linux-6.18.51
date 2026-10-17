/*
* Copyright (C) 2016 MediaTek Inc.
*
* This program is free software: you can redistribute it and/or modify it under the terms of the
* GNU General Public License version 2 as published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
* without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
* See the GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with this program.
* If not, see <http://www.gnu.org/licenses/>.
*/

/* Mainline frequency QoS replaces the BSP PPM calls. CPU hotplug remains with
 * the system policy; this driver never offlines MT6797 CPUs.
 */
#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/mutex.h>
#include <linux/pm_qos.h>
#include "precomp.h"

struct wifi_frequency_vote {
	struct cpufreq_policy *policy;
	struct freq_qos_request minimum;
};
static struct wifi_frequency_vote votes[NR_CPUS];
static DEFINE_MUTEX(votes_lock);

INT_32 kalBoostCpu(UINT_32 core_num)
{
	struct cpufreq_policy *policy;
	struct wifi_frequency_vote *vote;
	unsigned int cpu, index;
	int ret = 0, result;
	bool found = false;

	mutex_lock(&votes_lock);
	if (!core_num) {
		for (cpu = 0; cpu < NR_CPUS; cpu++) {
			vote = &votes[cpu];
			if (!vote->policy)
				continue;
			result = freq_qos_remove_request(&vote->minimum);
			if (result < 0 && !ret)
				ret = result;
			cpufreq_cpu_put(vote->policy);
			vote->policy = NULL;
		}
		goto out;
	}
	cpus_read_lock();
	if (core_num > num_online_cpus()) {
		ret = -EOPNOTSUPP;
		goto unlock_cpus;
	}
	for_each_online_cpu(cpu) {
		policy = cpufreq_cpu_get(cpu);
		if (!policy)
			continue;
		found = true;
		index = cpumask_first(policy->related_cpus);
		vote = &votes[index];
		if (vote->policy) {
			cpufreq_cpu_put(policy);
			continue;
		}
		result = freq_qos_add_request(&policy->constraints, &vote->minimum,
				FREQ_QOS_MIN, min(2000000U, policy->cpuinfo.max_freq));
		if (result < 0) {
			cpufreq_cpu_put(policy);
			if (!ret)
				ret = result;
			continue;
		}
		vote->policy = policy;
	}
	if (!found)
		ret = -EOPNOTSUPP;
unlock_cpus:
	cpus_read_unlock();
out:
	mutex_unlock(&votes_lock);
	return ret;
}
