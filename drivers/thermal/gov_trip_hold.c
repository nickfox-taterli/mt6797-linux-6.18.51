// SPDX-License-Identifier: GPL-2.0-only
/* Fixed trip cooling floors with continuous cooldown confirmation. */

#include <linux/jiffies.h>
#include <linux/slab.h>
#include <linux/thermal.h>

#include "thermal_core.h"

#define TRIP_HOLD_RELEASE_MS 2000

struct trip_hold_state {
	unsigned long release_at;
	bool active;
	bool releasing;
};

static int trip_hold_bind(struct thermal_zone_device *tz)
{
	struct trip_hold_state *states;
	const struct thermal_trip_desc *td;

	states = kcalloc(tz->num_trips, sizeof(*states), GFP_KERNEL);
	if (!states)
		return -ENOMEM;

	/* Preserve reached trips when switching governors inside hysteresis. */
	for_each_trip_desc(tz, td) {
		if (td->trip.temperature != THERMAL_TEMP_INVALID &&
		    tz->temperature >= td->threshold)
			states[thermal_zone_trip_id(tz, &td->trip)].active = true;
	}
	tz->governor_data = states;
	return 0;
}

static void trip_hold_unbind(struct thermal_zone_device *tz)
{
	kfree(tz->governor_data);
	tz->governor_data = NULL;
}

static void trip_hold_manage(struct thermal_zone_device *tz)
{
	struct trip_hold_state *states = tz->governor_data;
	const struct thermal_trip_desc *td;
	struct thermal_instance *instance;
	unsigned long now = jiffies;

	lockdep_assert_held(&tz->lock);

	for_each_trip_desc(tz, td) {
		const struct thermal_trip *trip = &td->trip;
		struct trip_hold_state *state;
		unsigned long target;

		if (trip->type != THERMAL_TRIP_PASSIVE &&
		    trip->type != THERMAL_TRIP_ACTIVE)
			continue;

		state = &states[thermal_zone_trip_id(tz, trip)];
		if (trip->temperature == THERMAL_TEMP_INVALID) {
			state->active = false;
			state->releasing = false;
		} else if (tz->temperature >= trip->temperature) {
			/* Increasing cooling never waits for the release timer. */
			state->active = true;
			state->releasing = false;
		} else if (state->active &&
			   tz->temperature < trip->temperature - trip->hysteresis) {
			if (!state->releasing) {
				state->release_at = now +
					msecs_to_jiffies(TRIP_HOLD_RELEASE_MS);
				state->releasing = true;
			} else if (time_after_eq(now, state->release_at)) {
				state->active = false;
				state->releasing = false;
			}
		} else {
			/* Any reheating above the release point cancels the interval. */
			state->releasing = false;
		}

		list_for_each_entry(instance, &td->thermal_instances, trip_node) {
			target = state->active ? instance->lower : THERMAL_NO_TARGET;
			if (instance->initialized && instance->target == target)
				continue;

			instance->target = target;
			instance->initialized = true;
			scoped_guard(cooling_dev, instance->cdev) {
				instance->cdev->updated = false;
			}
		}
	}

	for_each_trip_desc(tz, td) {
		list_for_each_entry(instance, &td->thermal_instances, trip_node)
			thermal_cdev_update(instance->cdev);
	}
}

static struct thermal_governor thermal_gov_trip_hold = {
	.name = "trip_hold",
	.bind_to_tz = trip_hold_bind,
	.unbind_from_tz = trip_hold_unbind,
	.manage = trip_hold_manage,
};
THERMAL_GOVERNOR_DECLARE(thermal_gov_trip_hold);
