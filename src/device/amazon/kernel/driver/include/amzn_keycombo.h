/*
 * amzn_keycombo.h
 *
 * Copyright 2020 Amazon Technologies, Inc. All Rights Reserved.
 *
 * The code contained herein is licensed under the GNU General Public
 * License Version 2.
 * You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#ifndef __AMZN_KEYCOMBO_H
#define __AMZN_KEYCOMBO_H

#include <linux/notifier.h>

enum COMBO_FUNC {
	COMBO_FUNC_PANIC = 0,
	COMBO_FUNC_POWER_OFF,
	COMBO_FUNC_NUM
};

int register_keycombo_notifier(struct notifier_block *nb);
int unregister_keycombo_notifier(struct notifier_block *nb);
#endif