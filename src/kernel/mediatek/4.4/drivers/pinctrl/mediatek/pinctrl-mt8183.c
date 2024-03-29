/*
 * Copyright (C) 2017 MediaTek Inc.
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


#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/regmap.h>
#include <dt-bindings/pinctrl/mt65xx.h>
#include "pinctrl-mtk-common.h"
#include "pinctrl-mtk-mt8183.h"
#include <mt-plat/mtk_gpio.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>

/* #define HAS_PUPD */
#define HAS_CONTIN_PUPD_R0R1
/* #define HAS_DISCRE_PUPD_R0R1 */
#define HAS_PULLSEL_PULLEN

static const struct mtk_pin_info mt8183_pin_info_rsel[] = {
	MTK_PIN_INFO(48, 0x0f0, 18, 2, 3),
	MTK_PIN_INFO(49, 0x0f0, 13, 2, 3),
	MTK_PIN_INFO(50, 0x0f0, 10, 2, 4),
	MTK_PIN_INFO(51, 0x0f0, 5, 2, 4),
	MTK_PIN_INFO(81, 0x0f0, 7, 2, 5),
	MTK_PIN_INFO(82, 0x0f0, 5, 2, 5),
	MTK_PIN_INFO(83, 0x0f0, 15, 2, 5),
	MTK_PIN_INFO(84, 0x0f0, 17, 2, 5),
	MTK_PIN_INFO(103, 0x0f0, 20, 2, 6),
	MTK_PIN_INFO(104, 0x0f0, 10, 2, 6),
	MTK_PIN_INFO(105, 0x0f0, 22, 2, 6),
	MTK_PIN_INFO(106, 0x0f0, 12, 2, 6),
};
static const struct mtk_pin_info mt8183_pin_info_eh[] = {
	MTK_PIN_INFO(48, 0x0f0, 20, 3, 3),
	MTK_PIN_INFO(49, 0x0f0, 15, 3, 3),
	MTK_PIN_INFO(50, 0x0f0, 12, 3, 4),
	MTK_PIN_INFO(51, 0x0f0, 7, 3, 4),
	MTK_PIN_INFO(81, 0x0f0, 12, 3, 5),
	MTK_PIN_INFO(82, 0x0f0, 9, 3, 5),
	MTK_PIN_INFO(83, 0x0f0, 19, 3, 5),
	MTK_PIN_INFO(84, 0x0f0, 22, 3, 5),
	MTK_PIN_INFO(103, 0x0f0, 24, 3, 6),
	MTK_PIN_INFO(104, 0x0f0, 14, 3, 6),
	MTK_PIN_INFO(105, 0x0f0, 27, 3, 6),
	MTK_PIN_INFO(106, 0x0f0, 17, 3, 6),
};

/* i2c mode pin set pull r1r0 resistance in rsel register */
static void mtk_rsel_r1r0_set_samereg(struct mtk_pinctrl *pctl,
		const struct mtk_pin_info *rsel_infos,
		unsigned int info_num, unsigned int pin,
		unsigned int r1r0)
{
	unsigned int reg_addr, set_addr, rst_addr;
	unsigned int bit_r0, bit_r1;
	const struct mtk_pin_info *rsel_pin;
	struct regmap *regmap;
	bool find = false;
	unsigned int i;

	for (i = 0; i < info_num; i++) {
		if (pin == rsel_infos[i].pin) {
			find = true;
			break;
		}
	}

	if (find) {
		rsel_pin = rsel_infos + i;
		regmap = pctl->regmap[rsel_pin->ip_num];
		reg_addr = rsel_pin->offset;
		set_addr = rsel_pin->offset + 4;
		rst_addr = rsel_pin->offset + 8;
		bit_r0 = BIT(rsel_pin->bit);
		bit_r1 = BIT(rsel_pin->bit + 1);

		switch (r1r0) {
		case MTK_RSEL_SET_R1R0_00:
			regmap_write(regmap, rst_addr, bit_r0);
			regmap_write(regmap, rst_addr, bit_r1);
			break;
		case MTK_RSEL_SET_R1R0_01:
			regmap_write(regmap, set_addr, bit_r0);
			regmap_write(regmap, rst_addr, bit_r1);
			break;
		case MTK_RSEL_SET_R1R0_10:
			regmap_write(regmap, rst_addr, bit_r0);
			regmap_write(regmap, set_addr, bit_r1);
			break;
		case MTK_RSEL_SET_R1R0_11:
			regmap_write(regmap, set_addr, bit_r0);
			regmap_write(regmap, set_addr, bit_r1);
			break;
		default:
			break;
		}
	}
}

/* i2c mode pin set eh resistance in rsel register */
static void mtk_rsel_eh_drive_set_samereg(struct mtk_pinctrl *pctl,
		const struct mtk_pin_info *rsel_infos,
		unsigned int info_num, unsigned int pin,
		unsigned int e1e0eh)
{
	unsigned int reg_addr, set_addr, rst_addr;
	unsigned int bit_r0, bit_r1, bit_r2;
	const struct mtk_pin_info *rsel_pin;
	struct regmap *regmap;
	bool find = false;
	unsigned int i;

	for (i = 0; i < info_num; i++) {
		if (pin == rsel_infos[i].pin) {
			find = true;
			break;
		}
	}
	if (find) {
		rsel_pin = rsel_infos + i;
		regmap = pctl->regmap[rsel_pin->ip_num];
		reg_addr = rsel_pin->offset;
		set_addr = rsel_pin->offset + 4;
		rst_addr = rsel_pin->offset + 8;
		bit_r0 = BIT(rsel_pin->bit);
		bit_r1 = BIT(rsel_pin->bit + 1);
		bit_r2 = BIT(rsel_pin->bit + 2);

		switch (e1e0eh) {
		case MTK_I2C_EH_DRIVE_000:
			regmap_write(regmap, rst_addr, bit_r0);
			regmap_write(regmap, rst_addr, bit_r1);
			regmap_write(regmap, rst_addr, bit_r2);
			break;
		case MTK_I2C_EH_DRIVE_001:
			regmap_write(regmap, set_addr, bit_r0);
			regmap_write(regmap, rst_addr, bit_r1);
			regmap_write(regmap, rst_addr, bit_r2);
			break;
		case MTK_I2C_EH_DRIVE_010:
			regmap_write(regmap, rst_addr, bit_r0);
			regmap_write(regmap, set_addr, bit_r1);
			regmap_write(regmap, rst_addr, bit_r2);
			break;
		case MTK_I2C_EH_DRIVE_011:
			regmap_write(regmap, set_addr, bit_r0);
			regmap_write(regmap, set_addr, bit_r1);
			regmap_write(regmap, rst_addr, bit_r2);
			break;
		case MTK_I2C_EH_DRIVE_100:
			regmap_write(regmap, rst_addr, bit_r0);
			regmap_write(regmap, rst_addr, bit_r1);
			regmap_write(regmap, set_addr, bit_r2);
			break;
		case MTK_I2C_EH_DRIVE_101:
			regmap_write(regmap, set_addr, bit_r0);
			regmap_write(regmap, rst_addr, bit_r1);
			regmap_write(regmap, set_addr, bit_r2);
			break;
		case MTK_I2C_EH_DRIVE_110:
			regmap_write(regmap, rst_addr, bit_r0);
			regmap_write(regmap, set_addr, bit_r1);
			regmap_write(regmap, set_addr, bit_r2);
			break;
		case MTK_I2C_EH_DRIVE_111:
			regmap_write(regmap, set_addr, bit_r0);
			regmap_write(regmap, set_addr, bit_r1);
			regmap_write(regmap, set_addr, bit_r2);
			break;
		default:
			break;
		}
	}
}

#if defined(HAS_CONTIN_PUPD_R0R1) || defined(HAS_DISCRE_PUPD_R0R1)
/* For type discreate/continuous PUPD + R1 + R0 */
static int mtk_pinctrl_set_gpio_pupd_r1r0(struct mtk_pinctrl *pctl, int pin,
		bool enable, bool isup, unsigned int r1r0)
{
	unsigned int pupd_r1r0 = 0, r0, r1, ret;

	/* For type continuous PUPD + R1 + R0 */
	if (r1r0 == MTK_PUPD_SET_R1R0_01)
		pupd_r1r0 = 1;
	else if (r1r0 == MTK_PUPD_SET_R1R0_10)
		pupd_r1r0 = 2;
	else if (r1r0 == MTK_PUPD_SET_R1R0_11)
		pupd_r1r0 = 3;
	else
		pupd_r1r0 = 0;
	/* HW value 0 for PU, HW value 1 for PD
	 * So need to revert input parametet isup before write to HW
	 */
	if (!isup)
		pupd_r1r0 |= 0x4;

	ret = mtk_pinctrl_update_gpio_value(pctl, pin, pupd_r1r0,
		pctl->devdata->n_pin_pupd_r1r0, pctl->devdata->pin_pupd_r1r0_grps);
	if (ret == 0)
		return ret;

	if (!pctl->devdata->n_pin_pupd)
		return -EPERM;

	/* For type discreate PUPD + R1 + R0 */
	ret = mtk_pinctrl_update_gpio_value(pctl, pin, !isup,
		pctl->devdata->n_pin_pupd, pctl->devdata->pin_pupd_grps);
	if (ret == 0) {
		r0 = r1r0 & 0x1;
		r1 = (r1r0 & 0x2) >> 1;
		mtk_pinctrl_update_gpio_value(pctl, pin, r0,
			pctl->devdata->n_pin_r0, pctl->devdata->pin_r0_grps);
		mtk_pinctrl_update_gpio_value(pctl, pin, r1,
			pctl->devdata->n_pin_r1, pctl->devdata->pin_r1_grps);
		ret = 0;
	}
	return ret;
}

/* For type discreate/continuous PUPD + R1 + R0 */
static int mtk_pinctrl_get_gpio_pupd_r1r0(struct mtk_pinctrl *pctl, int pin)
{
	int ret;
	int bit_pupd, bit_r0, bit_r1;

	/* For type continuous PUPD + R1 + R0 */
	ret = mtk_pinctrl_get_gpio_value(pctl, pin,
		pctl->devdata->n_pin_pupd_r1r0, pctl->devdata->pin_pupd_r1r0_grps);
	if (ret >= 0) {
		/* bit_upd: set for PU, clr for PD, ie, revert HW value */
		bit_pupd = (ret & 0x4) ? 0 : 1;
		bit_r0   = (ret & 0x1) ? MTK_PUPD_R1R0_BIT_R0 : 0;
		bit_r1   = (ret & 0x2) ? MTK_PUPD_R1R0_BIT_R1 : 0;
		return MTK_PUPD_R1R0_BIT_SUPPORT | bit_r1 | bit_r0 | bit_pupd;
	}

	if (!pctl->devdata->n_pin_pupd)
		return -EPERM;

	/* For type discreate PUPD + R1 + R0 */
	bit_pupd = mtk_pinctrl_get_gpio_value(pctl, pin,
		pctl->devdata->n_pin_pupd, pctl->devdata->pin_pupd_grps);
	if (bit_pupd != -EPERM) {
		bit_r1 = mtk_pinctrl_get_gpio_value(pctl, pin,
			pctl->devdata->n_pin_r1, pctl->devdata->pin_r1_grps) ? MTK_PUPD_R1R0_BIT_R1 : 0;
		bit_r0 = mtk_pinctrl_get_gpio_value(pctl, pin,
			pctl->devdata->n_pin_r0, pctl->devdata->pin_r0_grps) ? MTK_PUPD_R1R0_BIT_R0 : 0;
		/* bit_upd: set for PU, clr for PD, ie, revert HW value */
		return MTK_PUPD_R1R0_BIT_SUPPORT | bit_r1 | bit_r0 | !bit_pupd;
	}
	return -EPERM;
}
#endif

#if defined(HAS_PUPD)
/* For type PU+PD */
static int mtk_pinctrl_set_gpio_pu_pd(struct mtk_pinctrl *pctl, int pin,
		bool enable, bool isup, unsigned int r1r0)
{
	if (enable) {
		mtk_pinctrl_set_gpio_value(pctl, pin, isup,
			pctl->devdata->n_pin_pu, pctl->devdata->pin_pu_grps);
		mtk_pinctrl_set_gpio_value(pctl, pin, !isup,
			pctl->devdata->n_pin_pd, pctl->devdata->pin_pd_grps);
	} else {
		mtk_pinctrl_set_gpio_value(pctl, pin, 0,
			pctl->devdata->n_pin_pu, pctl->devdata->pin_pu_grps);
		mtk_pinctrl_set_gpio_value(pctl, pin, 0,
			pctl->devdata->n_pin_pd, pctl->devdata->pin_pd_grps);
	}
	return 0;
}

static int mtk_pinctrl_get_gpio_pu_pd(struct mtk_pinctrl *pctl, int pin)
{
	unsigned int bit_pu = 0, bit_pd = 0;

	bit_pu = mtk_pinctrl_get_gpio_value(pctl, pin,
		pctl->devdata->n_pin_pu, pctl->devdata->pin_pu_grps);
	bit_pd = mtk_pinctrl_get_gpio_value(pctl, pin,
		pctl->devdata->n_pin_pd, pctl->devdata->pin_pd_grps);
	if ((bit_pd != -EPERM) && (bit_pu != -EPERM))
		return bit_pd | (bit_pu << 1);
	else if ((bit_pd == -EPERM) && (bit_pu != -EPERM))
		return bit_pu << 1;
	else if ((bit_pd != -EPERM) && (bit_pu == -EPERM))
		return bit_pd;
	else
		return -EPERM;
}
#endif

#if defined(HAS_PULLSEL_PULLEN)
/* For type PULLSEL + PULLEN */
static int mtk_pinctrl_set_gpio_pullsel_pullen(struct mtk_pinctrl *pctl, int pin,
		bool enable, bool isup, unsigned int r1r0)
{
	mtk_pinctrl_update_gpio_value(pctl, pin, isup,
		pctl->devdata->n_pin_pullsel, pctl->devdata->pin_pullsel_grps);
	mtk_pinctrl_update_gpio_value(pctl, pin, enable,
		pctl->devdata->n_pin_pullen, pctl->devdata->pin_pullen_grps);
	return 0;
}

/* For type PULLSEL + PULLEN */
static int mtk_pinctrl_get_gpio_pullsel_pullen(struct mtk_pinctrl *pctl, int pin)
{
	unsigned int pullsel = 0, pullen = 0;

	pullsel = mtk_pinctrl_get_gpio_value(pctl, pin,
		pctl->devdata->n_pin_pullsel, pctl->devdata->pin_pullsel_grps);
	pullen = mtk_pinctrl_get_gpio_value(pctl, pin,
		pctl->devdata->n_pin_pullen, pctl->devdata->pin_pullen_grps);
	if (pullsel == -EPERM || pullen == -EPERM)
		return -EPERM;
	if (pullen == 0)
		return 0;
	else if ((pullsel == 1) && (pullen == 1))
		return MTK_PUPD_BIT_PU;
	else if ((pullsel == 0) && (pullen == 1))
		return MTK_PUPD_BIT_PD;
	else
		return -EINVAL;
}
#endif

/* Not specifically for gettting pullsel of PULLSEL type,
 * For getting pull-up/pull-down/no-pull + pull-enable/pull-disable
 */
static int mtk_pinctrl_get_gpio_pullsel(struct mtk_pinctrl *pctl, int pin)
{
	int pull_val = 0;

#if defined(HAS_CONTIN_PUPD_R0R1) || defined(HAS_DISCRE_PUPD_R0R1)
	pull_val = mtk_pinctrl_get_gpio_pupd_r1r0(pctl, pin);
	if (pull_val != -EPERM)
		return pull_val;
#endif

#if defined(HAS_PULLSEL_PULLEN) || defined(HAS_PUPD)
	#if defined(HAS_PULLSEL_PULLEN)
	pull_val = mtk_pinctrl_get_gpio_pullsel_pullen(pctl, pin);
	#else /* defined(HAS_PUPD) */
	pull_val = mtk_pinctrl_get_gpio_pu_pd(pctl, pin);
	#endif

	/*pull_val = [pu,pd], 10 is pull up, 01 is pull down*/
	if (pull_val == MTK_PUPD_BIT_PU)
		pull_val = GPIO_PULL_UP;
	else if (pull_val == MTK_PUPD_BIT_PD)
		pull_val = GPIO_PULL_DOWN;
	else if (pull_val == -EPERM)
		pull_val = GPIO_PULL_UNSUPPORTED;
	else
		pull_val = GPIO_NO_PULL;
#endif
	return pull_val;
}

/* Specifically for gettting pullen of PULLSEL type or pull-resistors of PUPD+R0+R1 */
static int mtk_pinctrl_get_gpio_pullen(struct mtk_pinctrl *pctl, int pin)
{
	int pull_val = 0, pull_en = 0;

#if defined(HAS_CONTIN_PUPD_R0R1) || defined(HAS_DISCRE_PUPD_R0R1)
	pull_val = mtk_pinctrl_get_gpio_pupd_r1r0(pctl, pin);
	if (pull_val > 0 && pull_val & MTK_PUPD_R1R0_BIT_SUPPORT) {
		/*pull_val = [r1,r0,pupd], pull disabel 000,001, others enable*/
		if (MTK_PUPD_R1R0_GET_PULLEN(pull_val))
			pull_en = GPIO_PULL_ENABLE;
		else
			pull_en = GPIO_PULL_DISABLE;

		return pull_en;
	} else if (pull_val != -EPERM) {
		return -EINVAL;
	}
#endif

#if defined(HAS_PULLSEL_PULLEN) || defined(HAS_PUPD)
	#if defined(HAS_PULLSEL_PULLEN)
	pull_en = mtk_pinctrl_get_gpio_pullsel_pullen(pctl, pin);
	#else /* defined(HAS_PUPD) */
	pull_en = mtk_pinctrl_get_gpio_pu_pd(pctl, pin);
	#endif

	/*pull_en = [pu,pd], 10,01 pull enabel, others pull disable*/
	if (pull_en & (MTK_PUPD_BIT_PU | MTK_PUPD_BIT_PD))
		pull_en = GPIO_PULL_ENABLE;
	else if (pull_en == -EPERM)
		pull_en = GPIO_PULL_EN_UNSUPPORTED;
	else
		pull_en = GPIO_PULL_DISABLE;
#endif
	return pull_en;

}

static int mtk_pinctrl_set_gpio_pull(struct mtk_pinctrl *pctl,
		int pin, bool enable, bool isup, unsigned int arg)
{
	int ret;

/* #define GPIO_DEBUG */
#ifdef GPIO_DEBUG
	int pull_val;

	pr_info("mtk_pinctrl_set_gpio_pull, pin = %d, enab = %d, sel = %d, arg = %u\n",
		pin, enable, isup, arg);
#endif

#if defined(HAS_CONTIN_PUPD_R0R1) || defined(HAS_DISCRE_PUPD_R0R1)
	ret = mtk_pinctrl_set_gpio_pupd_r1r0(pctl, pin, enable, isup, arg);
	if (ret == 0) {
#ifdef GPIO_DEBUG
		pull_val = mtk_pinctrl_get_gpio_pullsel(pctl, pin);
		pr_info("mtk_pinctrl_get_gpio_pull, pin = %d, enab = %d, sel = %d\n",
			pin,
			((pull_val >= 0) ? MTK_PUPD_R1R0_GET_PULLEN(pull_val) : -1),
			((pull_val >= 0) ? MTK_PUPD_R1R0_GET_PUPD(pull_val) : -1));
#endif
		goto out;
	}
#endif

#if defined(HAS_PULLSEL_PULLEN)
	if (!pctl->devdata->pin_pullsel_grps) {
		ret = -EPERM;
		goto out;
	}

	ret = mtk_pinctrl_set_gpio_pullsel_pullen(pctl, pin, enable, isup, arg);
	if (arg >= MTK_RSEL_SET_R1R0_00 && arg  <= MTK_RSEL_SET_R1R0_11)
			mtk_rsel_r1r0_set_samereg(pctl, mt8183_pin_info_rsel,
				ARRAY_SIZE(mt8183_pin_info_rsel), pin, arg);
#endif

#if defined(HAS_PUPD)
	if (!pctl->devdata->pin_pu_grps) {
		ret = -EPERM;
		goto out;
	}

	ret = mtk_pinctrl_set_gpio_pu_pd(pctl, pin, enable, isup, arg);
#endif

#ifdef GPIO_DEBUG
	if (ret == 0) {
		int enab = -1, sel = -1;

		pull_val = mtk_pinctrl_get_gpio_pullsel(pctl, pin);
		if (pull_val == GPIO_PULL_UP) {
			enab = 1;
			sel = 1;
		} else if (pull_val == GPIO_PULL_DOWN) {
			enab = 1;
			sel = 0;
		} else if (pull_val == GPIO_NO_PULL) {
			enab = 0;
			sel = 0;
		} else if (pull_val == GPIO_PULL_UNSUPPORTED) {
			enab = -1;
			sel = -1;
		}
		pr_info("mtk_pinctrl_get_gpio_pull, pin = %d, enab = %d, sel = %d\n",
			pin, enab, sel);
	}
#endif

out:

	return ret;
}

#ifdef CONFIG_MTK_EIC
int mtk_pinctrl_get_gpio_mode_for_eint(int pin)
{
	return mtk_pinctrl_get_gpio_value(pctl, pin,
		pctl->devdata->n_pin_mode, pctl->devdata->pin_mode_grps);
}
#endif
static const unsigned int mt8183_debounce_data[] = {
	128, 256, 512, 1024, 16384,
	32768, 65536, 131072, 262144, 524288
};

static unsigned int mt8183_spec_debounce_select(unsigned debounce)
{
	return mtk_gpio_debounce_select(mt8183_debounce_data,
		ARRAY_SIZE(mt8183_debounce_data), debounce);
}

int mtk_irq_domain_xlate_fourcell(struct irq_domain *d, struct device_node *ctrlr,
			const u32 *intspec, unsigned int intsize,
			irq_hw_number_t *out_hwirq, unsigned int *out_type)
{
	struct mtk_desc_eint *eint;
	int gpio, mode;

	if (WARN_ON(intsize < 4))
		return -EINVAL;
	*out_hwirq = intspec[0];
	*out_type = intspec[1] & IRQ_TYPE_SENSE_MASK;
	gpio = intspec[2];
	mode = intspec[3];

	eint = &pctl->devdata->pins[gpio].eint;
	eint->eintmux = mode;
	eint->eintnum = intspec[0];

	pr_debug("%s: mtk_pin[%d], eint=%d, mode=%d\n", __func__, gpio, eint->eintnum, eint->eintmux);

	return 0;
}
EXPORT_SYMBOL_GPL(mtk_irq_domain_xlate_fourcell);

struct irq_domain_ops mtk_irq_domain_ops = {
	.xlate = mtk_irq_domain_xlate_fourcell,
};

static const struct mtk_pinctrl_devdata mtk_pinctrl_data = {
	.pins = mtk_pins_mt8183,
	.npins = ARRAY_SIZE(mtk_pins_mt8183),
	.pin_mode_grps = mtk_pin_info_mode,
	.n_pin_mode = ARRAY_SIZE(mtk_pin_info_mode),
	.pin_drv_grps = mtk_pin_info_drv,
	.n_pin_drv = ARRAY_SIZE(mtk_pin_info_mode),
	.pin_smt_grps = mtk_pin_info_smt,
	.n_pin_smt = ARRAY_SIZE(mtk_pin_info_smt),
	.pin_ies_grps = mtk_pin_info_ies,
	.n_pin_ies = ARRAY_SIZE(mtk_pin_info_ies),

#if defined(HAS_PULLSEL_PULLEN)
	.pin_pullsel_grps = mtk_pin_info_pullsel,
	.n_pin_pullsel = ARRAY_SIZE(mtk_pin_info_pullsel),
	.pin_pullen_grps = mtk_pin_info_pullen,
	.n_pin_pullen = ARRAY_SIZE(mtk_pin_info_pullen),
#endif

#if defined(HAS_CONTIN_PUPD_R0R1)
	.pin_pupd_r1r0_grps = mtk_pin_info_pupd_r1r0,
	.n_pin_pupd_r1r0 = ARRAY_SIZE(mtk_pin_info_pupd_r1r0),
#endif

#if defined(HAS_PUPD)
	.pin_pu_grps = mtk_pin_info_pu,
	.n_pin_pu = ARRAY_SIZE(mtk_pin_info_pu),
	.pin_pd_grps = mtk_pin_info_pd,
	.n_pin_pd = ARRAY_SIZE(mtk_pin_info_pd),
#endif

#if defined(HAS_DISCRE_PUPD_R0R1)
	.pin_pupd_grps = mtk_pin_info_pupd,
	.n_pin_pupd = ARRAY_SIZE(mtk_pin_info_pupd),
	.pin_r0_grps = mtk_pin_info_r0,
	.n_pin_r0 = ARRAY_SIZE(mtk_pin_info_r0),
	.pin_r1_grps = mtk_pin_info_r1,
	.n_pin_r1 = ARRAY_SIZE(mtk_pin_info_r1),
#endif

	.pin_dout_grps = mtk_pin_info_dataout,
	.n_pin_dout = ARRAY_SIZE(mtk_pin_info_dataout),
	.pin_din_grps = mtk_pin_info_datain,
	.n_pin_din = ARRAY_SIZE(mtk_pin_info_datain),
	.pin_dir_grps = mtk_pin_info_dir,
	.n_pin_dir = ARRAY_SIZE(mtk_pin_info_dir),
	.pin_i2c_eh_grps = mt8183_pin_info_eh,
	.n_pin_i2c_eh = ARRAY_SIZE(mt8183_pin_info_eh),
	.mtk_pctl_set_pull = mtk_pinctrl_set_gpio_pull,
	.mtk_pctl_get_pull_sel = mtk_pinctrl_get_gpio_pullsel,
	.mtk_pctl_get_pull_en = mtk_pinctrl_get_gpio_pullen,
	.mtk_pctl_set_i2c_eh = mtk_rsel_eh_drive_set_samereg,
	.spec_debounce_select = mt8183_spec_debounce_select,
	.mtk_irq_domain_ops = &mtk_irq_domain_ops,
	.type1_start = 192,
	.type1_end = 192,
	.regmap_num = 9,
	.port_shf = 4,
	.port_mask = 0xf,
	.port_align = 4,
	.eint_offsets = {
		.name = "mt8183_eint",
		.stat      = 0x000,
		.ack       = 0x040,
		.mask      = 0x080,
		.mask_set  = 0x0c0,
		.mask_clr  = 0x100,
		.sens      = 0x140,
		.sens_set  = 0x180,
		.sens_clr  = 0x1c0,
		.soft      = 0x200,
		.soft_set  = 0x240,
		.soft_clr  = 0x280,
		.pol       = 0x300,
		.pol_set   = 0x340,
		.pol_clr   = 0x380,
		.dom_en    = 0x400,
		.dbnc_ctrl = 0x500,
		.dbnc_set  = 0x600,
		.dbnc_clr  = 0x700,
		.port_mask = 7,
		.ports     = 6,
	},
	.ap_num = 212,
	.db_cnt = 13,
};

static int mtk_pinctrl_probe(struct platform_device *pdev)
{
	pr_info("mtk pinctrl probe\n");
	return mtk_pctrl_init(pdev, &mtk_pinctrl_data, NULL);
}

static const struct of_device_id mtk_pctrl_match[] = {
	{
		.compatible = "mediatek,pinctrl",
	}, {
	}
};
MODULE_DEVICE_TABLE(of, mtk_pctrl_match);

static struct platform_driver mtk_pinctrl_driver = {
	.probe = mtk_pinctrl_probe,
	.driver = {
		.name = "mediatek-pinctrl",
		.owner = THIS_MODULE,
		.of_match_table = mtk_pctrl_match,
		.pm = &mtk_eint_pm_ops,
	},
};

static int __init mtk_pinctrl_init(void)
{
	return platform_driver_register(&mtk_pinctrl_driver);
}

/* module_init(mtk_pinctrl_init); */

postcore_initcall(mtk_pinctrl_init);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MediaTek Pinctrl Driver");
MODULE_AUTHOR("Light Hsieh <light.hsieh@mediatek.com>");
