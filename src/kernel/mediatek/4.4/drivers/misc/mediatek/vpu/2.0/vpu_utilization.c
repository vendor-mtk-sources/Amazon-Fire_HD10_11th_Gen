/*
 * Copyright (c) 2017 MediaTek Inc.
 * Author: Mao Lin <Zih-Ling.Lin@mediatek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/devfreq.h>
#include <linux/kernel.h>
#include <linux/ktime.h>
#include <linux/platform_device.h>
#include <linux/pm_opp.h>
#include <linux/timekeeper_internal.h>

#include "vpu_drv.h"
#include "vpu_utilization.h"
#include "vpu_cmn.h"

#define VPU_TIMER_PERIOD_NS 200000000 /* ns = 200 ms */
#define VPU_TIMER_PERIOD_MS 200

static int vpu_sum_usage(struct vpu_device *vpu_device, u32 core_id,
								ktime_t now)
{
	s64 diff;

	diff = ktime_us_delta(now, vpu_device->working_start[core_id]);
	if (diff < 0)
		goto reset;

	if (vpu_device->state[core_id] == VCT_EXECUTING)
		vpu_device->acc_busy[core_id] += (unsigned long) diff;

reset:
	vpu_device->working_start[core_id] = now;

	return 0;
}

int vpu_utilization_compute_enter(struct vpu_device *vpu_device,
							int core_id)
{
	unsigned long flags;
	struct vpu_util *vpu_util = vpu_device->vpu_util;
	int ret = 0;

	spin_lock_irqsave(&vpu_util->lock, flags);
	ret = vpu_sum_usage(vpu_device, core_id, ktime_get());
	if (vpu_device->state[core_id] == VCT_IDLE)
		vpu_device->state[core_id] = VCT_EXECUTING;
	spin_unlock_irqrestore(&vpu_util->lock, flags);

	return ret;
}

int vpu_utilization_compute_leave(struct vpu_device *vpu_device,
							int core_id)
{
	unsigned long flags;
	struct vpu_util *vpu_util = vpu_device->vpu_util;
	int ret = 0;

	spin_lock_irqsave(&vpu_util->lock, flags);
	ret = vpu_sum_usage(vpu_device, core_id, ktime_get());
	if (vpu_device->state[core_id] == VCT_EXECUTING)
		vpu_device->state[core_id] = VCT_IDLE;

	spin_unlock_irqrestore(&vpu_util->lock, flags);

	return ret;
}

int vpu_dvfs_get_usage(const struct vpu_device *vpu_device, int core,
		       unsigned long *total_time, unsigned long *busy_time)
{
	unsigned long flags;
	unsigned long busy = 0;
	unsigned long total;
	struct vpu_util *vpu_util = vpu_device->vpu_util;
	int ret = 0;

	spin_lock_irqsave(&vpu_util->lock, flags);
	busy = max(busy, vpu_device->prev_busy[core]);

	total = vpu_util->prev_total;
	spin_unlock_irqrestore(&vpu_util->lock, flags);

	*busy_time = busy / USEC_PER_MSEC;
	*total_time = total;
	if (*busy_time > *total_time)
		*busy_time = *total_time;

	return ret;
}

static int vpu_timer_get_usage(struct vpu_util *vpu_util)
{
	ktime_t now = ktime_get();
	struct vpu_device *vpu_device = vpu_util->vpu_device;
	unsigned long flags;
	u32 core_id;
	s64 diff;
	int ret = 0;

	spin_lock_irqsave(&vpu_util->lock, flags);
	diff = ktime_ms_delta(now, vpu_util->period_start);
	if (diff < 0) {
		vpu_util->prev_total = 0;
		goto reset_usage;
	}

	vpu_util->prev_total = (unsigned long) diff;

	for (core_id = 0; core_id < MTK_VPU_CORE; core_id++) {
		ret = vpu_sum_usage(vpu_device, core_id, now);
		vpu_device->prev_busy[core_id] = vpu_device->acc_busy[core_id];
	}

reset_usage:
	vpu_util->period_start = now;
	for (core_id = 0; core_id < MTK_VPU_CORE ; core_id++) {
		vpu_device->acc_busy[core_id] = 0;
		vpu_device->working_start[core_id] = vpu_util->period_start;
	}

	spin_unlock_irqrestore(&vpu_util->lock, flags);

#if defined(VPU_MET_READY)
	MET_Events_BusyRate_Trace(vpu_device);
#endif

	return ret;
}

static enum hrtimer_restart vpu_timer_callback(struct hrtimer *timer)
{
	struct vpu_util *vpu_util;
	int ret = 0;

	vpu_util = container_of(timer, struct vpu_util, timer);

	ret = vpu_timer_get_usage(vpu_util);

	if (vpu_util->active) {
		hrtimer_start(&vpu_util->timer, vpu_util->period_time,
			      HRTIMER_MODE_REL);
	}

	return HRTIMER_NORESTART;
}

int vpu_init_util(struct vpu_device *vpu_device)
{
	struct device *dev = vpu_device->dev[0];
	struct vpu_util *vpu_util;
	ktime_t now = ktime_get();
	int core_id;
	int ret = 0;

	vpu_util = devm_kzalloc(dev, sizeof(*vpu_util), GFP_KERNEL);
	if (!vpu_util)
		return -ENOMEM;

	vpu_util->dev = dev;
	vpu_util->vpu_device = vpu_device;

	vpu_util->period_time = ktime_set(0, VPU_TIMER_PERIOD_NS);
	vpu_util->period_start = now;

	for (core_id = 0; core_id < MTK_VPU_CORE ; core_id++) {
		vpu_device->prev_busy[core_id] = 0;
		vpu_device->acc_busy[core_id] = 0;
		vpu_device->working_start[core_id] = vpu_util->period_start;
		vpu_device->state[core_id] = VCT_IDLE;
	}
	vpu_util->prev_total = 0;

	spin_lock_init(&vpu_util->lock);

	vpu_util->active = true;
	hrtimer_init(&vpu_util->timer, CLOCK_MONOTONIC,
		     HRTIMER_MODE_REL);

	vpu_util->timer.function = vpu_timer_callback;
	hrtimer_start(&vpu_util->timer, vpu_util->period_time,
		      HRTIMER_MODE_REL);

	vpu_device->vpu_util = vpu_util;

	return ret;
}

int vpu_deinit_util(struct vpu_device *vpu_device)
{
	struct vpu_util *vpu_util = vpu_device->vpu_util;
	unsigned long flags;
	int ret = 1;

	spin_lock_irqsave(&vpu_util->lock, flags);
	vpu_util->active = false;
	spin_unlock_irqrestore(&vpu_util->lock, flags);

	while (ret > 0)
		ret = hrtimer_cancel(&vpu_util->timer);

	return ret;
}
