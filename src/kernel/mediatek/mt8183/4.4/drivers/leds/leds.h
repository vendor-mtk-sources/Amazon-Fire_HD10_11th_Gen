/*
 * LED Core
 *
 * Copyright 2005 Openedhand Ltd.
 *
 * Author: Richard Purdie <rpurdie@openedhand.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#ifndef __LEDS_H_INCLUDED
#define __LEDS_H_INCLUDED

#include <linux/rwsem.h>
#include <linux/leds.h>

#ifdef CONFIG_SILENT_OTA
extern int silent_ota_bl_is_silent(void);
#endif

static inline void led_set_brightness_async(struct led_classdev *led_cdev,
					enum led_brightness value)
{
	value = min(value, led_cdev->max_brightness);
	led_cdev->brightness = value;

#ifdef CONFIG_SILENT_OTA
	if (silent_ota_bl_is_silent()) {
		pr_info("%s: ignore brightness %d in silent mode\n",
				__func__, value);
		return;
	}
#endif

	if (!(led_cdev->flags & LED_SUSPENDED))
		led_cdev->brightness_set(led_cdev, value);
}

static inline int led_set_brightness_sync(struct led_classdev *led_cdev,
					enum led_brightness value)
{
	int ret = 0;

	led_cdev->brightness = min(value, led_cdev->max_brightness);

#ifdef CONFIG_SILENT_OTA
	if (silent_ota_bl_is_silent()) {
		pr_info("%s: ignore brightness %d in silent mode\n",
				__func__, value);
		return ret;
	}
#endif

	if (!(led_cdev->flags & LED_SUSPENDED))
		ret = led_cdev->brightness_set_sync(led_cdev,
						led_cdev->brightness);
	return ret;
}

static inline int led_get_brightness(struct led_classdev *led_cdev)
{
	return led_cdev->brightness;
}

void led_init_core(struct led_classdev *led_cdev);
void led_stop_software_blink(struct led_classdev *led_cdev);

extern struct rw_semaphore leds_list_lock;
extern struct list_head leds_list;

#endif	/* __LEDS_H_INCLUDED */
