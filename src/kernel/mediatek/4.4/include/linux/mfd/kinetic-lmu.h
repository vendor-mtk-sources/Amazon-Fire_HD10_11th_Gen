/*
 * KINETIC LMU (Lighting Management Unit) Devices
 *
 * Author: Bruce pu <bruce.xm.pu@enskytech.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __KINETIC_LMU_H__
#define __KINETIC_LMU_H__

#include <linux/gpio.h>
#include <linux/notifier.h>
#include <linux/regmap.h>
#include <linux/gpio/consumer.h>

/* Notifier event */
#define LMU_EVENT_MONITOR_DONE		0x01

enum kinetic_lmu_id {
	KTZ8864A,
	LMU_MAX_ID,
};

enum ktz8864a_regulator_id {
	KTZ8864A_BOOST,		/* Boost output */
	KTZ8864A_LDO_POS,	/* Positive display bias output */
	KTZ8864A_LDO_NEG,	/* Negative display bias output */
};

/**
 * struct kinetic_lmu
 *
 * @dev:	Parent device pointer
 * @regmap:	Used for i2c communcation on accessing registers
 * @en_gpio:	GPIO for HWEN pin [Optional]
 * @notifier:	Notifier for reporting hwmon event
 */
struct kinetic_lmu {
	struct device *dev;
	struct regmap *regmap;
	struct gpio_desc *en_gpio;
	struct blocking_notifier_head notifier;
};

#endif //__KINETIC_LMU_H__
