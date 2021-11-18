/*
* Copyright (C) 2016 MediaTek Inc.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
* See http://www.gnu.org/licenses/gpl-2.0.html for more details.
*/

/*
 *
 * Filename:
 * ---------
 *    mtk_switch_charging.c
 *
 * Project:
 * --------
 *   Android_Software
 *
 * Description:
 * ------------
 *   This Module defines functions of Battery charging
 *
 * Author:
 * -------
 * Wy Chuang
 *
 */
#include <linux/init.h>		/* For init/exit macros */
#include <linux/module.h>	/* For MODULE_ marcros  */
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/power_supply.h>
#include <linux/wakelock.h>
#include <linux/time.h>
#include <linux/mutex.h>
#include <linux/kthread.h>
#include <linux/proc_fs.h>
#include <linux/platform_device.h>
#include <linux/seq_file.h>
#include <linux/scatterlist.h>
#include <linux/suspend.h>
#include <linux/kernel.h>

#include <mt-plat/mtk_boot.h>
#include <mt-plat/battery_metrics.h>
#include <musb_core.h>
#include "mtk_charger_intf.h"
#include "mtk_switch_charging.h"
#include "mtk_battery_internal.h"

static int _uA_to_mA(int uA)
{
	if (uA == -1)
		return -1;
	else
		return uA / 1000;
}

static void _disable_all_charging(struct charger_manager *info)
{
	charger_dev_enable(info->chg1_dev, false);

	if (mtk_is_pe30_running(info)) {
		mtk_pe30_end(info);
		mtk_pe30_set_is_enable(info, true);
	}

	if (mtk_pe20_get_is_enable(info)) {
		mtk_pe20_set_is_enable(info, false);
		if (mtk_pe20_get_is_connect(info))
			mtk_pe20_reset_ta_vchr(info);
	}

	if (mtk_pe_get_is_enable(info)) {
		mtk_pe_set_is_enable(info, false);
		if (mtk_pe_get_is_connect(info))
			mtk_pe_reset_ta_vchr(info);
	}

	if (mtk_pe40_get_is_enable(info)) {
		if (mtk_pe40_get_is_connect(info))
			mtk_pe40_end(info, 3, true);
	}

}

static int adapter_power_detection_by_ocp(struct charger_manager *info)
{
	struct charger_data *pdata = &info->chg1_data;
	int bak_cv_uv = 0, bak_iusb_ua = 0, bak_ichg_ua = 0, bak_mivr = 0;
	int cv_uv = info->data.battery_cv;
	int iusb_ua = info->power_detection.aicl_trigger_iusb;
	int ichg_ua = info->power_detection.aicl_trigger_ichg;
	int mivr_uv = info->power_detection.mivr_detect;

	/* Backup IUSB/ICHG/CV setting */
	charger_dev_get_constant_voltage(info->chg1_dev, &bak_cv_uv);
	charger_dev_get_input_current(info->chg1_dev, &bak_iusb_ua);
	charger_dev_get_charging_current(info->chg1_dev, &bak_ichg_ua);
	charger_dev_get_mivr(info->chg1_dev, &bak_mivr);
	pr_info("%s: backup IUSB[%d] ICHG[%d] CV[%d] MIVR[%d]\n",
			__func__, _uA_to_mA(bak_iusb_ua),
			_uA_to_mA(bak_ichg_ua), bak_cv_uv, bak_mivr);

	/* set higher setting to draw more power */
	pr_info("%s: set IUSB[%d] ICHG[%d] CV[%d] MIVR[%d] for detection\n",
			__func__, _uA_to_mA(iusb_ua),
			_uA_to_mA(ichg_ua), cv_uv,
			(mivr_uv > 0)?mivr_uv:bak_mivr);
	if (mivr_uv > 0)
		charger_dev_set_mivr(info->chg1_dev, mivr_uv);
	charger_dev_enable_input_current_limit_calibration(info->chg1_dev,
		false);
	charger_dev_set_constant_voltage(info->chg1_dev, cv_uv);
	charger_dev_set_input_current(info->chg1_dev, iusb_ua);
	charger_dev_set_charging_current(info->chg1_dev, ichg_ua);
	charger_dev_dump_registers(info->chg1_dev);

	/* Run AICL */
	msleep(50);
	charger_dev_run_aicl(info->chg1_dev,
			&pdata->input_current_limit_by_aicl);
	pr_info("%s: aicl result: %d mA\n", __func__,
			_uA_to_mA(pdata->input_current_limit_by_aicl));

	/* Restore IUB/ICHG/CV setting */
	pr_info("%s: restore IUSB[%d] ICHG[%d] CV[%d] MIVR[%d]\n",
			__func__, _uA_to_mA(bak_iusb_ua),
			_uA_to_mA(bak_ichg_ua), bak_cv_uv, bak_mivr);
	charger_dev_enable_input_current_limit_calibration(info->chg1_dev,
		true);
	charger_dev_set_constant_voltage(info->chg1_dev, bak_cv_uv);
	charger_dev_set_input_current(info->chg1_dev, bak_iusb_ua);
	charger_dev_set_charging_current(info->chg1_dev, bak_ichg_ua);
	charger_dev_dump_registers(info->chg1_dev);

	return pdata->input_current_limit_by_aicl;
}

static int adapter_power_detection(struct charger_manager *info)
{
	struct charger_data *pdata = &info->chg1_data;
	struct power_detection_data *det = &info->power_detection;
	int chr_type = info->chr_type;
	int aicl_ua = 0, rp_curr_ma;
	static const char * const category_text[] = {
		"5W", "7.5W", "9W", "12W", "15W"
	};
	bool wpc_online = false;

	wireless_charger_dev_get_online(get_charger_by_name("wireless_chg"),
		&wpc_online);

	if (wpc_online)
		return 0;

	if (!det->en)
		return 0;

	if (det->iusb_ua)
		goto skip;

	/* Step 1: Determine Type-C adapter by Rp */
	rp_curr_ma = tcpm_inquire_typec_remote_rp_curr(info->tcpc);
	if (rp_curr_ma == 3000) {
		pdata->input_current_limit = 3000000;
		det->iusb_ua = 3000000;
		det->type = ADAPTER_15W;
		goto done;
	} else if (rp_curr_ma == 1500) {
		pdata->input_current_limit = 1500000;
		det->iusb_ua = 1500000;
		det->type = ADAPTER_7_5W;
		goto done;
	}

	if (chr_type != STANDARD_CHARGER)
		return 0;

	/* Step 2: Run AICL for OCP detection on A2C adapter */
	aicl_ua = adapter_power_detection_by_ocp(info);
	if (aicl_ua < 0) {
		pdata->input_current_limit = det->adapter_12w_iusb_lim;
		det->iusb_ua = det->adapter_12w_iusb_lim;
		det->type = ADAPTER_12W;
		pr_info("%s: CV stage or 15W adapter, keep 12W as default\n",
			__func__);
		goto done;
	}

	/* Step 3: Determine adapter power categroy for 5W/9W/12W */
	if (aicl_ua > det->adapter_12w_aicl_min) {
		pdata->input_current_limit = det->adapter_12w_iusb_lim;
		det->iusb_ua = det->adapter_12w_iusb_lim;
		det->type = ADAPTER_12W;
	} else if (aicl_ua > det->adapter_9w_aicl_min) {
		pdata->input_current_limit = det->adapter_9w_iusb_lim;
		det->iusb_ua = det->adapter_9w_iusb_lim;
		det->type = ADAPTER_9W;
	} else {
		pdata->input_current_limit = det->adapter_5w_iusb_lim;
		det->iusb_ua = det->adapter_5w_iusb_lim;
		det->type = ADAPTER_5W;
	}

done:
	bat_metrics_adapter_power(det->type, _uA_to_mA(aicl_ua));
	pr_info("%s: detect %s adapter\n", __func__, category_text[det->type]);
	return 0;

skip:
	if (pdata->thermal_input_current_limit != -1) {
		pr_info("%s: use thermal_input_current_limit, ignore\n",
			__func__);
	} else {
		pdata->input_current_limit = det->iusb_ua;
		pr_info("%s: alread finish: %d mA, skip\n",
			__func__, _uA_to_mA(pdata->input_current_limit));
	}

	return 0;
}

static void swchg_select_charging_current_limit(struct charger_manager *info)
{
	struct charger_data *pdata;
	struct switch_charging_alg_data *swchgalg = info->algorithm_data;
	u32 ichg1_min = 0, aicr1_min = 0;
	int ret;
	bool wpc_online = false;

	wireless_charger_dev_get_online(get_charger_by_name("wireless_chg"),
		&wpc_online);
	pdata = &info->chg1_data;

	mutex_lock(&swchgalg->ichg_aicr_access_mutex);

	/* AICL */
	if (!mtk_is_pe30_running(info) && !mtk_pe20_get_is_connect(info) &&
		!mtk_pe_get_is_connect(info) &&
		!mtk_is_TA_support_pd_pps(info) &&
		!wpc_online)
		charger_dev_run_aicl(info->chg1_dev, &pdata->input_current_limit_by_aicl);

	if (pdata->force_input_current_limit >= 0) {
		pdata->input_current_limit = pdata->force_input_current_limit;
		goto done;
	}

	if (pdata->force_charging_current > 0) {

		pdata->charging_current_limit = pdata->force_charging_current;
		if (pdata->force_charging_current <= 450000) {
			pdata->input_current_limit = 500000;
		} else {
			pdata->input_current_limit = info->data.ac_charger_input_current;
		}
		goto done;
	}

	if (info->usb_unlimited) {
		pdata->input_current_limit = info->data.ac_charger_input_current;
		pdata->charging_current_limit = info->data.ac_charger_current;
		goto done;
	}

	if ((get_boot_mode() == META_BOOT) || ((get_boot_mode() == ADVMETA_BOOT))) {
		pdata->input_current_limit = 200000; /* 200mA */
		goto done;
	}

	if (mtk_is_TA_support_pd_pps(info)) {
		pdata->input_current_limit = info->data.pe40_single_charger_input_current;
		pdata->charging_current_limit = info->data.pe40_single_charger_current;
	} else if (is_typec_adapter(info)) {

		if (tcpm_inquire_typec_remote_rp_curr(info->tcpc) == 3000) {
			pdata->input_current_limit = 3000000;
			pdata->charging_current_limit =
				info->data.ac_charger_current;
		} else if (tcpm_inquire_typec_remote_rp_curr(info->tcpc) == 1500) {
			pdata->input_current_limit = 1500000;
			pdata->charging_current_limit =
				info->data.ac_charger_current;
		} else {
			chr_err("type-C: inquire rp error\n");
			pdata->input_current_limit = 500000;
			pdata->charging_current_limit = 500000;
		}

		chr_err("type-C:%d current:%d\n",
			info->pd_type, tcpm_inquire_typec_remote_rp_curr(info->tcpc));
	} else if (mtk_pdc_check_charger(info) == true) {
		int vbus = 0, cur = 0, idx = 0;

		mtk_pdc_get_setting(info, &vbus, &cur, &idx);
		if (idx != -1) {
		pdata->input_current_limit = cur * 1000;
		pdata->charging_current_limit = info->data.pd_charger_current;
			mtk_pdc_setup(info, idx);
		} else {
			pdata->input_current_limit = info->data.usb_charger_current_configured;
			pdata->charging_current_limit = info->data.usb_charger_current_configured;
		}
		chr_err("[%s]vbus:%d input_cur:%d idx:%d current:%d\n", __func__,
			vbus, cur, idx, info->data.pd_charger_current);

	} else if (info->chr_type == STANDARD_HOST) {
		if (IS_ENABLED(CONFIG_USBIF_COMPLIANCE)) {
			if (info->usb_state == USB_SUSPEND)
				pdata->input_current_limit = info->data.usb_charger_current_suspend;
			else if (info->usb_state == USB_UNCONFIGURED)
				pdata->input_current_limit = info->data.usb_charger_current_unconfigured;
			else if (info->usb_state == USB_CONFIGURED)
				pdata->input_current_limit = info->data.usb_charger_current_configured;
			else
				pdata->input_current_limit = info->data.usb_charger_current_unconfigured;

			pdata->charging_current_limit = pdata->input_current_limit;
		} else {
			pdata->input_current_limit = info->data.usb_charger_current;
			pdata->charging_current_limit = info->data.usb_charger_current;	/* it can be larger */
		}
	} else if (info->chr_type == NONSTANDARD_CHARGER) {
		pdata->input_current_limit = info->data.non_std_ac_charger_current;
		pdata->charging_current_limit = info->data.non_std_ac_charger_current;
	} else if (info->chr_type == STANDARD_CHARGER) {
		pdata->input_current_limit = info->data.ac_charger_input_current;
		pdata->charging_current_limit = info->data.ac_charger_current;
		mtk_pe20_set_charging_current(info, &pdata->charging_current_limit, &pdata->input_current_limit);
		mtk_pe_set_charging_current(info, &pdata->charging_current_limit, &pdata->input_current_limit);
	} else if (info->chr_type == CHARGING_HOST) {
		pdata->input_current_limit = info->data.charging_host_charger_current;
		pdata->charging_current_limit = info->data.charging_host_charger_current;
	} else if (info->chr_type == APPLE_1_0A_CHARGER) {
		pdata->input_current_limit = info->data.apple_1_0a_charger_current;
		pdata->charging_current_limit = info->data.apple_1_0a_charger_current;
	} else if (info->chr_type == APPLE_2_1A_CHARGER) {
		pdata->input_current_limit = info->data.apple_2_1a_charger_current;
		pdata->charging_current_limit = info->data.apple_2_1a_charger_current;
	} else if (info->chr_type == WIRELESS_CHARGER_DEFAULT) {
		pdata->input_current_limit = 500000; /* 500mA */
		pdata->charging_current_limit = info->data.wpc_5w_charger_current;
	} else if (info->chr_type == WIRELESS_CHARGER_5W) {
		pdata->input_current_limit = info->data.wpc_5w_charger_input_current;
		pdata->charging_current_limit = info->data.wpc_5w_charger_current;
	} else if (info->chr_type == WIRELESS_CHARGER_10W) {
		pdata->input_current_limit = info->data.wpc_10w_charger_input_current;
		pdata->charging_current_limit = info->data.wpc_10w_charger_current;
	} else if (info->chr_type == WIRELESS_CHARGER_15W) {
		pdata->input_current_limit = info->data.wpc_15w_charger_input_current;
		pdata->charging_current_limit = info->data.wpc_15w_charger_current;
	}

	/* for throttle wireless charging power */
	if (wpc_online && (pdata->wireless_input_current_limit != -1)) {
		if (pdata->wireless_input_current_limit <
				pdata->input_current_limit) {
			pdata->input_current_limit =
				pdata->wireless_input_current_limit;
		}
	}

	if (info->enable_sw_jeita) {
		if (IS_ENABLED(CONFIG_USBIF_COMPLIANCE) && info->chr_type == STANDARD_HOST)
			pr_debug("USBIF & STAND_HOST skip current check\n");
		else {
			if (info->sw_jeita.sm == TEMP_T0_TO_T1) {
				pdata->charging_current_limit =
				info->data.temp_t0_charging_current_limit;
			}
		}
	}

	if (pdata->thermal_input_power_limit != -1 &&
		wpc_online) {
		struct power_supply *psy;
		int input_power_limit =
				pdata->thermal_input_power_limit;
		int iusb_ua = 0;
		union power_supply_propval wpc_vout;
		enum charger_type chr_type = info->chr_type;
		const int max_input_power_bpp_mw = 5000;
		const int max_input_power_epp_mw = 10000;
		const int max_input_power_bpp_plus_mw = 15000;

		if (chr_type == WIRELESS_CHARGER_15W)
			input_power_limit = min(input_power_limit,
				max_input_power_bpp_plus_mw);
		else if (chr_type == WIRELESS_CHARGER_10W)
			input_power_limit = min(input_power_limit,
				max_input_power_epp_mw);
		else
			input_power_limit = min(input_power_limit,
				max_input_power_bpp_mw);

		psy = power_supply_get_by_name("Wireless");
		if (psy != NULL) {
			ret = power_supply_get_property(psy,
				POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN, &wpc_vout);
			if (!ret) {
				iusb_ua = input_power_limit * 1000 / (wpc_vout.intval / 1000);
				if (iusb_ua < pdata->input_current_limit)
					pdata->input_current_limit = iusb_ua;
			}
		}
	}

	if (pdata->thermal_charging_current_limit != -1)
		if (pdata->thermal_charging_current_limit < pdata->charging_current_limit)
			pdata->charging_current_limit = pdata->thermal_charging_current_limit;

	if (pdata->thermal_input_current_limit != -1)
		if (pdata->thermal_input_current_limit < pdata->input_current_limit)
			pdata->input_current_limit = pdata->thermal_input_current_limit;

	if (mtk_pe40_get_is_connect(info)) {
		if (info->pe4.pe4_input_current_limit != -1 &&
			info->pe4.pe4_input_current_limit < pdata->input_current_limit)
			pdata->input_current_limit = info->pe4.pe4_input_current_limit;

		info->pe4.input_current_limit = pdata->input_current_limit;

		if (info->pe4.pe4_input_current_limit_setting != -1 &&
			info->pe4.pe4_input_current_limit_setting < pdata->input_current_limit)
			pdata->input_current_limit = info->pe4.pe4_input_current_limit_setting;
	}

	/* Apply selected current setting then do adapter power detection */
	charger_dev_set_input_current(info->chg1_dev, pdata->input_current_limit);
	charger_dev_set_charging_current(info->chg1_dev, pdata->charging_current_limit);
	adapter_power_detection(info);

	if (pdata->input_current_limit_by_aicl != -1 && !mtk_is_pe30_running(info) &&
		!mtk_pe20_get_is_connect(info) && !mtk_pe_get_is_connect(info) &&
		!mtk_is_TA_support_pd_pps(info)) {
		if (pdata->input_current_limit_by_aicl < pdata->input_current_limit)
			pdata->input_current_limit = pdata->input_current_limit_by_aicl;
	}
done:
	ret = charger_dev_get_min_charging_current(info->chg1_dev, &ichg1_min);
	if (ret != -ENOTSUPP && pdata->charging_current_limit < ichg1_min)
		pdata->charging_current_limit = 0;

	ret = charger_dev_get_min_input_current(info->chg1_dev, &aicr1_min);
	if (ret != -ENOTSUPP && pdata->input_current_limit < aicr1_min)
		pdata->input_current_limit = 0;

	chr_err("force:%d thermal:%d,%d,%d(mW) pe4:%d,%d,%d setting:%d %d type:%d usb_unlimited:%d usbif:%d usbsm:%d aicl:%d\n",
		_uA_to_mA(pdata->force_charging_current),
		_uA_to_mA(pdata->thermal_input_current_limit),
		_uA_to_mA(pdata->thermal_charging_current_limit),
		pdata->thermal_input_power_limit,
		_uA_to_mA(info->pe4.pe4_input_current_limit),
		_uA_to_mA(info->pe4.pe4_input_current_limit_setting),
		_uA_to_mA(info->pe4.input_current_limit),
		_uA_to_mA(pdata->input_current_limit),
		_uA_to_mA(pdata->charging_current_limit),
		info->chr_type, info->usb_unlimited,
		IS_ENABLED(CONFIG_USBIF_COMPLIANCE), info->usb_state,
		pdata->input_current_limit_by_aicl);

	charger_dev_set_input_current(info->chg1_dev, pdata->input_current_limit);
	charger_dev_set_charging_current(info->chg1_dev, pdata->charging_current_limit);

	/* If AICR < 300mA, stop PE+/PE+20 */
	if (pdata->input_current_limit < 300000) {
		if (mtk_pe20_get_is_enable(info)) {
			mtk_pe20_set_is_enable(info, false);
			if (mtk_pe20_get_is_connect(info))
				mtk_pe20_reset_ta_vchr(info);
		}

		if (mtk_pe_get_is_enable(info)) {
			mtk_pe_set_is_enable(info, false);
			if (mtk_pe_get_is_connect(info))
				mtk_pe_reset_ta_vchr(info);
		}
	}

	/*
	 * If thermal current limit is larger than charging IC's minimum
	 * current setting, enable the charger immediately
	 */
	if (pdata->input_current_limit > aicr1_min && pdata->charging_current_limit > ichg1_min
	    && info->can_charging)
		charger_dev_enable(info->chg1_dev, true);
	mutex_unlock(&swchgalg->ichg_aicr_access_mutex);
}

static void swchg_select_cv(struct charger_manager *info)
{
	u32 constant_voltage;

	if (info->custom_charging_cv != -1) {
		if (info->enable_sw_jeita && info->sw_jeita.cv != 0) {
			if (info->custom_charging_cv > info->sw_jeita.cv)
				constant_voltage = info->sw_jeita.cv;
			else
				constant_voltage = info->custom_charging_cv;
		} else {
			constant_voltage = info->custom_charging_cv;
		}
		chr_err("%s: top_off_mode CV:%duV", __func__,
			constant_voltage);
		charger_dev_set_constant_voltage(info->chg1_dev, constant_voltage);
		return;
	}

	if (info->enable_sw_jeita) {
		if (info->sw_jeita.cv != 0) {
			charger_dev_set_constant_voltage(info->chg1_dev, info->sw_jeita.cv);
			return;
		}
	}

	/* dynamic cv*/
#ifndef CONFIG_MTK_USE_AGING_ZCV
	constant_voltage = info->data.battery_cv;
#else
	/* MTK_USE_AGING_ZCV */
	if (!gm.use_aging_zcv)
		constant_voltage = info->data.battery_cv;
	else
		constant_voltage = info->data.battery_cv_aging;
#endif

	mtk_get_dynamic_cv(info, &constant_voltage);

	charger_dev_set_constant_voltage(info->chg1_dev, constant_voltage);
}

static void swchg_turn_on_charging(struct charger_manager *info)
{
	struct switch_charging_alg_data *swchgalg = info->algorithm_data;
	bool charging_enable = true;

	if (swchgalg->state == CHR_ERROR) {
		charging_enable = false;
		chr_err("[charger]Charger Error, turn OFF charging !\n");
	} else if ((get_boot_mode() == META_BOOT) || ((get_boot_mode() == ADVMETA_BOOT))) {
		charging_enable = false;
		info->chg1_data.input_current_limit = 200000; /* 200mA */
		charger_dev_set_input_current(info->chg1_dev, info->chg1_data.input_current_limit);
		chr_err("[charger]In meta or advanced meta mode, disable charging and set input current limit to 200mA\n");
	} else {
		mtk_pe20_start_algorithm(info);
		mtk_pe_start_algorithm(info);

		swchg_select_charging_current_limit(info);
		if (info->chg1_data.input_current_limit == 0 || info->chg1_data.charging_current_limit == 0) {
			charging_enable = false;
			chr_err("[charger]charging current is set 0mA, turn off charging !\r\n");
		} else {
			swchg_select_cv(info);
		}
	}

	charger_dev_enable(info->chg1_dev, charging_enable);
}

static int mtk_switch_charging_plug_in(struct charger_manager *info)
{
	struct switch_charging_alg_data *swchgalg = info->algorithm_data;

	if (info->enable_sw_safety_timer) {
		if (info->disconnect_duration >=
			info->sw_safety_timer_reset_time) {
			chr_err("%s: Reset SW safety timer\n", __func__);
			info->safety_timeout = false;
			get_monotonic_boottime(&swchgalg->charging_begin_time);
			swchgalg->total_charging_time = 0;
		}
	}
	/* Keep charging state as CHR_BATFULL. */
	if (info->enable_bat_eoc_protect) {
		if (info->bat_eoc_protect) {
			chr_err("%s: Keep charging state full\n", __func__);
			swchgalg->state = CHR_BATFULL;
			info->polling_interval = CHARGING_FULL_INTERVAL;
			battery_main.BAT_STATUS = POWER_SUPPLY_STATUS_CHARGING;
			battery_update(&battery_main);
			charger_dev_do_event(info->chg1_dev, EVENT_EOC, 0);
			return  0;
		}
	}
	swchgalg->state = CHR_CC;
	info->polling_interval = CHARGING_INTERVAL;
	swchgalg->disable_charging = false;
	charger_manager_notifier(info, CHARGER_NOTIFY_START_CHARGING);

	return 0;
}

static int mtk_switch_charging_plug_out(struct charger_manager *info)
{
	info->power_detection.iusb_ua = 0;

	mtk_pe20_set_is_cable_out_occur(info, true);
	mtk_pe_set_is_cable_out_occur(info, true);
	mtk_pe30_plugout_reset(info);
	mtk_pdc_plugout(info);
	mtk_pe40_plugout_reset(info);
	charger_manager_notifier(info, CHARGER_NOTIFY_STOP_CHARGING);
	return 0;
}

static int mtk_switch_charging_do_charging(struct charger_manager *info, bool en)
{
	struct switch_charging_alg_data *swchgalg = info->algorithm_data;

	chr_err("mtk_switch_charging_do_charging en:%d %s\n", en, info->algorithm_name);
	if (en) {
		swchgalg->disable_charging = false;
		swchgalg->state = CHR_CC;
		get_monotonic_boottime(&swchgalg->charging_begin_time);
		charger_manager_notifier(info, CHARGER_NOTIFY_NORMAL);
		mtk_pe40_set_is_enable(info, en);
	} else {
		/* disable might change state , so first */
		_disable_all_charging(info);
		swchgalg->disable_charging = true;
		/* Force state machine to CHR_BATFULL if charging is
		 * disabled by battery EOC protection.
		 */
		if (info->bat_eoc_protect == false) {
			swchgalg->state = CHR_ERROR;
			charger_manager_notifier(info, CHARGER_NOTIFY_ERROR);
		} else {
			swchgalg->state = CHR_BATFULL;
			charger_dev_do_event(info->chg1_dev, EVENT_EOC, 0);
			charger_dev_enable_ir_comp(info->chg1_dev, false);
		}
	}

	return 0;
}

static int mtk_switch_chr_pe40_init(struct charger_manager *info)
{
	swchg_turn_on_charging(info);
	return mtk_pe40_init_state(info);
}

static int mtk_switch_chr_pe40_cc(struct charger_manager *info)
{
	swchg_turn_on_charging(info);
	return mtk_pe40_cc_state(info);
}

/* return false if total charging time exceeds max_charging_time */
static bool mtk_switch_check_charging_time(struct charger_manager *info)
{
	struct switch_charging_alg_data *swchgalg = info->algorithm_data;
	struct timespec time_now;

	if (info->enable_sw_safety_timer) {
		get_monotonic_boottime(&time_now);
		chr_debug("%s: begin: %ld, now: %ld\n", __func__,
			swchgalg->charging_begin_time.tv_sec, time_now.tv_sec);

		if (swchgalg->total_charging_time >= info->data.max_charging_time) {
			chr_err("%s: SW safety timeout: %d sec > %d sec\n",
				__func__, swchgalg->total_charging_time,
				info->data.max_charging_time);
			charger_dev_notify(info->chg1_dev, CHARGER_DEV_NOTIFY_SAFETY_TIMEOUT);
			return false;
		}
	}

	return true;
}

static int mtk_switch_chr_cc(struct charger_manager *info)
{
	bool chg_done = false;
	struct switch_charging_alg_data *swchgalg = info->algorithm_data;
	struct timespec time_now, charging_time;

	/* check bif */
	if (IS_ENABLED(CONFIG_MTK_BIF_SUPPORT)) {
		if (pmic_is_bif_exist() != 1) {
			chr_err("CONFIG_MTK_BIF_SUPPORT but no bif , stop charging\n");
			swchgalg->state = CHR_ERROR;
			charger_manager_notifier(info, CHARGER_NOTIFY_ERROR);
		}
	}

	get_monotonic_boottime(&time_now);
	charging_time = timespec_sub(time_now, swchgalg->charging_begin_time);

	swchgalg->total_charging_time = charging_time.tv_sec;

	if (mtk_pe40_is_ready(info)) {
		chr_err("enter PE4.0!\n");
		swchgalg->state = CHR_PE40_INIT;
		info->pe4.is_connect = true;
		return 1;
	}

	swchg_turn_on_charging(info);

	charger_dev_is_charging_done(info->chg1_dev, &chg_done);
	if (chg_done) {
		swchgalg->state = CHR_BATFULL;
		charger_dev_do_event(info->chg1_dev, EVENT_EOC, 0);
		charger_dev_enable_ir_comp(info->chg1_dev, false);
		chr_err("battery full!\n");
	}

	/* If it is not disabled by throttling,
	 * enable PE+/PE+20, if it is disabled
	 */
	if (info->chg1_data.thermal_input_current_limit != -1 &&
		info->chg1_data.thermal_input_current_limit < 300)
		return 0;

	if (!mtk_pe20_get_is_enable(info)) {
		mtk_pe20_set_is_enable(info, true);
		mtk_pe20_set_to_check_chr_type(info, true);
	}

	if (!mtk_pe_get_is_enable(info)) {
		mtk_pe_set_is_enable(info, true);
		mtk_pe_set_to_check_chr_type(info, true);
	}
	return 0;
}

int mtk_switch_chr_err(struct charger_manager *info)
{
	struct switch_charging_alg_data *swchgalg = info->algorithm_data;

	if (info->enable_sw_jeita) {
		if ((info->sw_jeita.sm == TEMP_BELOW_T0) || (info->sw_jeita.sm == TEMP_ABOVE_T4))
			info->sw_jeita.error_recovery_flag = false;

		if ((info->sw_jeita.error_recovery_flag == false) &&
			(info->sw_jeita.sm != TEMP_BELOW_T0) && (info->sw_jeita.sm != TEMP_ABOVE_T4)) {
			info->sw_jeita.error_recovery_flag = true;
			swchgalg->state = CHR_CC;
			get_monotonic_boottime(&swchgalg->charging_begin_time);
		}
	}

	swchgalg->total_charging_time = 0;

	_disable_all_charging(info);
	return 0;
}

int mtk_switch_chr_full(struct charger_manager *info)
{
	bool chg_done = false;
	struct switch_charging_alg_data *swchgalg = info->algorithm_data;

	swchgalg->total_charging_time = 0;

	/* turn off LED */

	/*
	 * If CV is set to lower value by JEITA,
	 * Reset CV to normal value if temperture is in normal zone
	 */
	swchg_select_cv(info);
	info->polling_interval = CHARGING_FULL_INTERVAL;

	/* Keep charging state at CHR_BATFULL after triggering
	 * battery EOC protection.
	 */
	if (info->bat_eoc_protect)
		return 0;

	charger_dev_is_charging_done(info->chg1_dev, &chg_done);
	if (!chg_done) {
		swchgalg->state = CHR_CC;
		charger_dev_do_event(info->chg1_dev, EVENT_RECHARGE, 0);
		mtk_pe20_set_to_check_chr_type(info, true);
		mtk_pe_set_to_check_chr_type(info, true);
		mtk_pe40_set_is_enable(info, true);
		info->enable_dynamic_cv = true;
		get_monotonic_boottime(&swchgalg->charging_begin_time);
		chr_err("battery recharging!\n");
		info->polling_interval = CHARGING_INTERVAL;
	}

	return 0;
}

int mtk_switch_pe30(struct charger_manager *info)
{
	struct switch_charging_alg_data *swchgalg = info->algorithm_data;

	if (mtk_is_pe30_running(info) == false)
		swchgalg->state = CHR_CC;

	return 0;
}

static int mtk_switch_charging_current(struct charger_manager *info)
{
	swchg_select_charging_current_limit(info);
	return 0;
}

static int mtk_switch_charging_run(struct charger_manager *info)
{
	struct switch_charging_alg_data *swchgalg = info->algorithm_data;
	int ret = 0;

	chr_err("%s [%d %d], timer=%d\n", __func__, swchgalg->state,
		info->pd_type,
		swchgalg->total_charging_time);

	if (mtk_pe30_check_charger(info) == true)
		swchgalg->state = CHR_PE30;

	if (mtk_is_TA_support_pe30(info) == false &&
		mtk_pdc_check_charger(info) == false &&
		mtk_is_TA_support_pd_pps(info) == false) {
		mtk_pe20_check_charger(info);
		mtk_pe_check_charger(info);
	}

	do {
		switch (swchgalg->state) {
			chr_err("mtk_switch_charging_run2 [%d] %d\n", swchgalg->state, info->pd_type);
		case CHR_CC:
			ret = mtk_switch_chr_cc(info);
			break;

		case CHR_PE40_INIT:
			ret = mtk_switch_chr_pe40_init(info);
			break;

		case CHR_PE40_CC:
			ret = mtk_switch_chr_pe40_cc(info);
			break;

		case CHR_BATFULL:
			ret = mtk_switch_chr_full(info);
			break;

		case CHR_ERROR:
			ret = mtk_switch_chr_err(info);
			break;

		case CHR_PE30:
			ret = mtk_switch_pe30(info);
			break;
		}
	} while (ret != 0);
	mtk_switch_check_charging_time(info);

	charger_dev_dump_registers(info->chg1_dev);
	return 0;
}

int charger_dev_event(struct notifier_block *nb, unsigned long event, void *v)
{
	struct charger_manager *info = container_of(nb, struct charger_manager, chg1_nb);
	struct chgdev_notify *data = v;

	pr_err_ratelimited("%s %ld", __func__, event);

	switch (event) {
	case CHARGER_DEV_NOTIFY_EOC:
		charger_manager_notifier(info, CHARGER_NOTIFY_EOC);
		pr_info("%s: end of charge\n", __func__);
		break;
	case CHARGER_DEV_NOTIFY_RECHG:
		charger_manager_notifier(info, CHARGER_NOTIFY_START_CHARGING);
		pr_info("%s: recharge\n", __func__);
		break;
	case CHARGER_DEV_NOTIFY_SAFETY_TIMEOUT:
		info->safety_timeout = true;
		chr_err("%s: safety timer timeout\n", __func__);
		bat_metrics_chg_fault(METRICS_FAULT_SAFETY_TIMEOUT);

		/* If sw safety timer timeout, do not wake up charger thread */
		if (info->enable_sw_safety_timer)
			return NOTIFY_DONE;
		break;
	case CHARGER_DEV_NOTIFY_VBUS_OVP:
		info->vbusov_stat = data->vbusov_stat;
		chr_err("%s: vbus ovp = %d\n", __func__, info->vbusov_stat);
		bat_metrics_chg_fault(METRICS_FAULT_VBUS_OVP);
		break;
	default:
		return NOTIFY_DONE;
	}

	if (info->chg1_dev->is_polling_mode == false)
		_wake_up_charger(info);

	return NOTIFY_DONE;
}


int mtk_switch_charging_init(struct charger_manager *info)
{
	struct switch_charging_alg_data *swch_alg;

	swch_alg = devm_kzalloc(&info->pdev->dev, sizeof(struct switch_charging_alg_data), GFP_KERNEL);
	if (!swch_alg)
		return -ENOMEM;

	info->chg1_dev = get_charger_by_name("primary_chg");
	if (info->chg1_dev)
		chr_err("Found primary charger [%s]\n", info->chg1_dev->props.alias_name);
	else
		chr_err("*** Error : can't find primary charger [%s]***\n", "primary_chg");

	mutex_init(&swch_alg->ichg_aicr_access_mutex);

	info->algorithm_data = swch_alg;
	info->do_algorithm = mtk_switch_charging_run;
	info->plug_in = mtk_switch_charging_plug_in;
	info->plug_out = mtk_switch_charging_plug_out;
	info->do_charging = mtk_switch_charging_do_charging;
	info->do_event = charger_dev_event;
	info->change_current_setting = mtk_switch_charging_current;

	return 0;
}




