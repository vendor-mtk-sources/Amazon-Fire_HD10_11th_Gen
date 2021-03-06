/*
 * Copyright (c) 2014 MediaTek Inc.
 * Author: James Liao <jamesjj.liao@mediatek.com>
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

#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/clkdev.h>
#include <linux/delay.h>

#include "clk-mtk.h"

#define REG_CON0		0
#define REG_CON1		4

#define CON0_BASE_EN		BIT(0)
#define CON0_PWR_ON		BIT(0)
#define CON0_ISO_EN		BIT(1)
#define CON0_PCW_CHG		BIT(31)

#define AUDPLL_TUNER_EN		BIT(31)

#define POSTDIV_MASK		0x7
#if defined(CONFIG_MACH_MT6799)
#define INTEGER_BITS		8
#define UNIVPLL_DIV			2
#elif defined(CONFIG_MACH_MT6759)
/*#define MT_CCF_BRINGUP*/
#define INTEGER_BITS		8
#define UNIVPLL_DIV			1
#elif defined(CONFIG_MACH_MT6758)
/*#define MT_CCF_BRINGUP*/
#define INTEGER_BITS		8
#define UNIVPLL_DIV			1
#elif defined(CONFIG_MACH_MT6739)
/*#define MT_CCF_BRINGUP*/
#define INTEGER_BITS		7
#define UNIVPLL_DIV			1
#elif defined(CONFIG_MACH_MT6763)
/*#define MT_CCF_BRINGUP*/
#define INTEGER_BITS		8
#define UNIVPLL_DIV			2
#elif defined(CONFIG_MACH_MT8183)
/*#define MT_CCF_BRINGUP*/
#define INTEGER_BITS		8
#define UNIVPLL_DIV			2
#elif defined(CONFIG_MACH_MT6775)
/* #define MT_CCF_BRINGUP */
#define INTEGER_BITS		8
#define UNIVPLL_DIV			1
#else
#define INTEGER_BITS		7
#define UNIVPLL_DIV			1
#endif
/*
 * MediaTek PLLs are configured through their pcw value. The pcw value describes
 * a divider in the PLL feedback loop which consists of 7 bits for the integer
 * part and the remaining bits (if present) for the fractional part. Also they
 * have a 3 bit power-of-two post divider.
 */

struct mtk_clk_pll {
	struct clk_hw	hw;
	void __iomem	*base_addr;
	void __iomem	*pd_addr;
	void __iomem	*pwr_addr;
	void __iomem	*tuner_addr;
	void __iomem	*pcw_addr;
	const struct mtk_pll_data *data;
};

static inline struct mtk_clk_pll *to_mtk_clk_pll(struct clk_hw *hw)
{
	return container_of(hw, struct mtk_clk_pll, hw);
}

static int mtk_pll_is_prepared(struct clk_hw *hw)
{
	struct mtk_clk_pll *pll = to_mtk_clk_pll(hw);

	return (readl(pll->base_addr + REG_CON0) & CON0_BASE_EN) != 0;
}

static unsigned long __mtk_pll_recalc_rate(struct mtk_clk_pll *pll, u32 fin,
		u32 pcw, int postdiv)
{
	int pcwbits = pll->data->pcwbits;
	int pcwfbits;
	u64 vco;
	u8 c = 0;

#if defined(CONFIG_MACH_MT6739)
	int int_bits = pll->data->pcwintbits;
#else
	int int_bits = INTEGER_BITS;
#endif

	/* The fractional part of the PLL divider. */
	pcwfbits = pcwbits > int_bits ? pcwbits - int_bits : 0;

	vco = (u64)fin * pcw;

	if (pcwfbits && (vco & GENMASK(pcwfbits - 1, 0)))
		c = 1;

	vco >>= pcwfbits;

	if (c)
		vco++;

	return ((unsigned long)vco + postdiv - 1) / postdiv;
}

static void mtk_pll_set_rate_regs(struct mtk_clk_pll *pll, u32 pcw,
		int postdiv)
{
	u32 con1, val;

#if defined(CONFIG_MACH_MT6739)
	int pcwchgreg = pll->data->pcwchgreg;
#else
	int pcwchgreg = REG_CON1;
#endif

	/* set postdiv */
	val = readl(pll->pd_addr);
	val &= ~(POSTDIV_MASK << pll->data->pd_shift);
	val |= (ffs(postdiv) - 1) << pll->data->pd_shift;

	/* postdiv and pcw need to set at the same time if on same register */
	if (pll->pd_addr != pll->pcw_addr) {
		writel(val, pll->pd_addr);
		val = readl(pll->pcw_addr);
	}

	/* set pcw */
	val &= ~GENMASK(pll->data->pcw_shift + pll->data->pcwbits - 1,
			pll->data->pcw_shift);
	val |= pcw << pll->data->pcw_shift;
	writel(val, pll->pcw_addr);

	con1 = readl(pll->base_addr + pcwchgreg);

	con1 |= CON0_PCW_CHG;

	writel(con1, pll->base_addr + pcwchgreg);
	if (pll->tuner_addr)
		writel(con1 + 1, pll->tuner_addr);

	udelay(20);
}

/*
 * mtk_pll_calc_values - calculate good values for a given input frequency.
 * @pll:	The pll
 * @pcw:	The pcw value (output)
 * @postdiv:	The post divider (output)
 * @freq:	The desired target frequency
 * @fin:	The input frequency
 *
 */
static void mtk_pll_calc_values(struct mtk_clk_pll *pll, u32 *pcw, u32 *postdiv,
		u32 freq, u32 fin)
{
#if (defined(CONFIG_MACH_MT6763) || (defined(CONFIG_MACH_MT8183)))
	unsigned long fmin = 1500 * MHZ;
	int int_bits = INTEGER_BITS;
#elif defined(CONFIG_MACH_MT6758)
	unsigned long fmin = 2000 * MHZ;
	int int_bits = INTEGER_BITS;
#elif defined(CONFIG_MACH_MT6739)
	unsigned long fmin = pll->data->fmin;
	int int_bits = pll->data->pcwintbits;
#elif defined(CONFIG_MACH_MT6775)
	unsigned long fmin = 2000 * MHZ;
	int int_bits = INTEGER_BITS;
#else
	unsigned long fmin = 1000 * MHZ;
	int int_bits = INTEGER_BITS;
#endif
	const struct mtk_pll_div_table *div_table = pll->data->div_table;
	u64 _pcw;
	u32 val;

	if (freq > pll->data->fmax)
		freq = pll->data->fmax;

	if (div_table) {
		if (freq > div_table[0].freq)
			freq = div_table[0].freq;

		for (val = 0; div_table[val + 1].freq != 0; val++) {
			if (freq > div_table[val + 1].freq)
				break;
		}
		*postdiv = 1 << val;
	} else {
		for (val = 0; val < 5; val++) {
			*postdiv = 1 << val;
			if ((u64)freq * *postdiv >= fmin)
				break;
		}
	}


	/* _pcw = freq * postdiv / fin * 2^pcwfbits */
	_pcw = ((u64)freq << val) << (pll->data->pcwbits - int_bits);
	do_div(_pcw, fin);

	*pcw = (u32)_pcw;
}
#ifdef MT_CCF_BRINGUP
static int mtk_pll_is_prepared_dummy(struct clk_hw *hw)
{
	return 1;
}
static int mtk_pll_set_rate_dummy(struct clk_hw *hw, unsigned long rate,
		unsigned long parent_rate)
{
	return 0;
}
#if 0
static unsigned long mtk_pll_recalc_rate_dummy(struct clk_hw *hw,
		unsigned long parent_rate)
{
	return 0;
}

static long mtk_pll_round_rate_dummy(struct clk_hw *hw, unsigned long rate,
		unsigned long *prate)
{
	return 0;
}
#endif
static int mtk_pll_prepare_dummy(struct clk_hw *hw)
{
	return 0;
}

static void mtk_pll_unprepare_dummy(struct clk_hw *hw)
{
}

#endif
#if ((defined(CONFIG_MACH_MT6799))	\
	|| (defined(CONFIG_MACH_MT6759))	\
	|| (defined(CONFIG_MACH_MT6763))	\
	|| (defined(CONFIG_MACH_MT6758))	\
	|| (defined(CONFIG_MACH_MT8183))	\
	|| (defined(CONFIG_MACH_MT6739))	\
	|| (defined(CONFIG_MACH_MT6775)))
static int mtk_pll_set_rate(struct clk_hw *hw, unsigned long rate,
		unsigned long parent_rate)
{
	struct mtk_clk_pll *pll = to_mtk_clk_pll(hw);
	u32 pcw = 0;
	u32 postdiv;
	unsigned long rate_div = 0;
	const char *name = __clk_get_name(hw->clk);

	/* if univpll, rate << 1 for analog div 2 */
	if (name != NULL && !strcmp(name, "univpll"))
		rate_div = rate * UNIVPLL_DIV;
	#if 0
	else if ((!strcmp(__clk_get_name(hw->clk), "apll1")) || (!strcmp(__clk_get_name(hw->clk), "apll2")))
		rate_div = rate << 2;
	#endif
	else
		rate_div = rate;
	mtk_pll_calc_values(pll, &pcw, &postdiv, rate_div, parent_rate);
	mtk_pll_set_rate_regs(pll, pcw, postdiv);

	return 0;
}

static unsigned long mtk_pll_recalc_rate(struct clk_hw *hw,
		unsigned long parent_rate)
{
	struct mtk_clk_pll *pll = to_mtk_clk_pll(hw);
	u32 postdiv, analogdiv;
	u32 pcw;
	const char *name = __clk_get_name(hw->clk);

	postdiv = (readl(pll->pd_addr) >> pll->data->pd_shift) & POSTDIV_MASK;
	postdiv = 1 << postdiv;

	pcw = readl(pll->pcw_addr) >> pll->data->pcw_shift;
	pcw &= GENMASK(pll->data->pcwbits - 1, 0);

	/* return after analogdiv */
	/* if univpll, analogdiv = 2 */
	/* if apll1/apll2, analogdiv = 4 */
	if (name != NULL && !strcmp(name, "univpll"))
		analogdiv = UNIVPLL_DIV;
	#if 0
	else if ((!strcmp(__clk_get_name(hw->clk), "apll1")) || (!strcmp(__clk_get_name(hw->clk), "apll2")))
		analogdiv = 4;
	#endif
	else
		analogdiv = 1;
	return __mtk_pll_recalc_rate(pll, parent_rate, pcw, postdiv)/analogdiv;
}

static long mtk_pll_round_rate(struct clk_hw *hw, unsigned long rate,
		unsigned long *prate)
{
	struct mtk_clk_pll *pll = to_mtk_clk_pll(hw);
	u32 pcw = 0;
	int postdiv, analogdiv;
	const char *name = __clk_get_name(hw->clk);

	mtk_pll_calc_values(pll, &pcw, &postdiv, rate, *prate);

	/* return after analogdiv */
	/* if univpll, analogdiv = 2 */
	/* if apll1/apll2, analogdiv = 4 */
	if (name != NULL && !strcmp(name, "univpll"))
		analogdiv = UNIVPLL_DIV;
	#if 0
	else if ((!strcmp(__clk_get_name(hw->clk), "apll1")) || (!strcmp(__clk_get_name(hw->clk), "apll2")))
		analogdiv = 4;
	#endif
	else
		analogdiv = 1;
	return __mtk_pll_recalc_rate(pll, *prate, pcw, postdiv)/analogdiv;
}

static int mtk_pll_prepare(struct clk_hw *hw)
{
	struct mtk_clk_pll *pll = to_mtk_clk_pll(hw);
	u32 r;
	const char *name = __clk_get_name(hw->clk);

	/*pr_err("[CCF] %s: %s\n", __func__, __clk_get_name(hw->clk));*/
	if (readl(pll->pwr_addr) & CON0_PWR_ON) {
		/*pr_err("[CCF] %s: %s is already power on\n", __func__, __clk_get_name(hw->clk));*/
	} else {
		/*pr_err("[CCF] %s: %s is power off\n", __func__, __clk_get_name(hw->clk));*/
#if (defined(CONFIG_MACH_MT6763) || (defined(CONFIG_MACH_MT8183)))
		if (name != NULL && !strcmp(name, "univpll"))
			univpll_192m_en(1);
#endif
		r = readl(pll->pwr_addr) | CON0_PWR_ON;

		writel(r, pll->pwr_addr);
		udelay(1);

		r = readl(pll->pwr_addr) & ~CON0_ISO_EN;
		writel(r, pll->pwr_addr);
		udelay(1);

		r = readl(pll->base_addr + REG_CON0);
		r |= pll->data->en_mask;
		writel(r, pll->base_addr + REG_CON0);

		if (pll->tuner_addr) {
			r = readl(pll->tuner_addr) | AUDPLL_TUNER_EN;
			writel(r, pll->tuner_addr);
		}

		udelay(20);

		if (pll->data->flags & HAVE_RST_BAR) {
			r = readl(pll->base_addr + REG_CON0);
			r |= pll->data->rst_bar_mask;
			writel(r, pll->base_addr + REG_CON0);
		}
	}
	return 0;
}

static void mtk_pll_unprepare(struct clk_hw *hw)
{
	struct mtk_clk_pll *pll = to_mtk_clk_pll(hw);
	u32 r;
	const char *name = __clk_get_name(hw->clk);

#if defined(CONFIG_MACH_MT6799) || defined(CONFIG_MACH_MT6759)
	if (name != NULL && !strcmp(name, "univpll")) {
	} else {
		if (readl(pll->pwr_addr) & CON0_PWR_ON) {
			if (pll->data->flags & HAVE_RST_BAR) {
				r = readl(pll->base_addr + REG_CON0);
				r &= ~pll->data->rst_bar_mask;
				writel(r, pll->base_addr + REG_CON0);
			}

			if (pll->tuner_addr) {
				r = readl(pll->tuner_addr) & ~AUDPLL_TUNER_EN;
				writel(r, pll->tuner_addr);
			}

			r = readl(pll->base_addr + REG_CON0);
			r &= ~CON0_BASE_EN;
			writel(r, pll->base_addr + REG_CON0);

			r = readl(pll->pwr_addr) | CON0_ISO_EN;
			writel(r, pll->pwr_addr);

			r = readl(pll->pwr_addr) & ~CON0_PWR_ON;
			writel(r, pll->pwr_addr);
		}
	}
#else
	if (name != NULL && !strcmp(name, "mainpll")) {
	} else {
		if (readl(pll->pwr_addr) & CON0_PWR_ON) {
			if (pll->data->flags & HAVE_RST_BAR) {
				r = readl(pll->base_addr + REG_CON0);
				r &= ~pll->data->rst_bar_mask;
				writel(r, pll->base_addr + REG_CON0);
			}

			if (pll->tuner_addr) {
				r = readl(pll->tuner_addr) & ~AUDPLL_TUNER_EN;
				writel(r, pll->tuner_addr);
			}

			r = readl(pll->base_addr + REG_CON0);
			r &= ~CON0_BASE_EN;
			writel(r, pll->base_addr + REG_CON0);

			#if defined(CONFIG_MACH_MT6775)
			udelay(1);
			#endif

			r = readl(pll->pwr_addr) | CON0_ISO_EN;
			writel(r, pll->pwr_addr);

			#if defined(CONFIG_MACH_MT6775)
			udelay(1);
			#endif

			r = readl(pll->pwr_addr) & ~CON0_PWR_ON;
			writel(r, pll->pwr_addr);
			#if (defined(CONFIG_MACH_MT6763) || (defined(CONFIG_MACH_MT8183)))
			if (name != NULL && !strcmp(name, "univpll"))
				univpll_192m_en(0);
			#endif
		}
	}
#endif
}
#else
static int mtk_pll_set_rate(struct clk_hw *hw, unsigned long rate,
		unsigned long parent_rate)
{
	struct mtk_clk_pll *pll = to_mtk_clk_pll(hw);
	u32 pcw = 0;
	u32 postdiv;

	mtk_pll_calc_values(pll, &pcw, &postdiv, rate, parent_rate);
	mtk_pll_set_rate_regs(pll, pcw, postdiv);

	return 0;
}

static unsigned long mtk_pll_recalc_rate(struct clk_hw *hw,
		unsigned long parent_rate)
{
	struct mtk_clk_pll *pll = to_mtk_clk_pll(hw);
	u32 postdiv;
	u32 pcw;

	postdiv = (readl(pll->pd_addr) >> pll->data->pd_shift) & POSTDIV_MASK;
	postdiv = 1 << postdiv;

	pcw = readl(pll->pcw_addr) >> pll->data->pcw_shift;
	pcw &= GENMASK(pll->data->pcwbits - 1, 0);

	return __mtk_pll_recalc_rate(pll, parent_rate, pcw, postdiv);
}

static long mtk_pll_round_rate(struct clk_hw *hw, unsigned long rate,
		unsigned long *prate)
{
	struct mtk_clk_pll *pll = to_mtk_clk_pll(hw);
	u32 pcw = 0;
	int postdiv;

	mtk_pll_calc_values(pll, &pcw, &postdiv, rate, *prate);

	return __mtk_pll_recalc_rate(pll, *prate, pcw, postdiv);
}

static int mtk_pll_prepare(struct clk_hw *hw)
{
	struct mtk_clk_pll *pll = to_mtk_clk_pll(hw);
	u32 r;

	r = readl(pll->pwr_addr) | CON0_PWR_ON;
	writel(r, pll->pwr_addr);
	udelay(1);

	r = readl(pll->pwr_addr) & ~CON0_ISO_EN;
	writel(r, pll->pwr_addr);
	udelay(1);

	r = readl(pll->base_addr + REG_CON0);
	r |= pll->data->en_mask;
	writel(r, pll->base_addr + REG_CON0);

	if (pll->tuner_addr) {
		r = readl(pll->tuner_addr) | AUDPLL_TUNER_EN;
		writel(r, pll->tuner_addr);
	}

	udelay(20);

	if (pll->data->flags & HAVE_RST_BAR) {
		r = readl(pll->base_addr + REG_CON0);
		r |= pll->data->rst_bar_mask;
		writel(r, pll->base_addr + REG_CON0);
	}

	return 0;
}

static void mtk_pll_unprepare(struct clk_hw *hw)
{
	struct mtk_clk_pll *pll = to_mtk_clk_pll(hw);
	u32 r;

	if (pll->data->flags & HAVE_RST_BAR) {
		r = readl(pll->base_addr + REG_CON0);
		r &= ~pll->data->rst_bar_mask;
		writel(r, pll->base_addr + REG_CON0);
	}

	if (pll->tuner_addr) {
		r = readl(pll->tuner_addr) & ~AUDPLL_TUNER_EN;
		writel(r, pll->tuner_addr);
	}

	r = readl(pll->base_addr + REG_CON0);
	r &= ~CON0_BASE_EN;
	writel(r, pll->base_addr + REG_CON0);

	r = readl(pll->pwr_addr) | CON0_ISO_EN;
	writel(r, pll->pwr_addr);

	r = readl(pll->pwr_addr) & ~CON0_PWR_ON;
	writel(r, pll->pwr_addr);
}
#endif

#if defined(MT_CCF_BRINGUP)
static const struct clk_ops mtk_pll_ops_dummy = {
	.is_enabled	= mtk_pll_is_prepared_dummy,
	.enable		= mtk_pll_prepare_dummy,
	.disable	= mtk_pll_unprepare_dummy,
	.recalc_rate	= mtk_pll_recalc_rate,
	.round_rate	= mtk_pll_round_rate,
	.set_rate	= mtk_pll_set_rate_dummy,
};
#endif
#if ((defined(CONFIG_MACH_MT6799))	\
	|| (defined(CONFIG_MACH_MT6763))	\
	|| (defined(CONFIG_MACH_MT6759))	\
	|| (defined(CONFIG_MACH_MT6758))	\
	|| (defined(CONFIG_MACH_MT8183))	\
	|| (defined(CONFIG_MACH_MT6739))	\
	|| (defined(CONFIG_MACH_MT6775)))
static const struct clk_ops mtk_pll_ops = {
	.is_enabled	= mtk_pll_is_prepared,
	.enable		= mtk_pll_prepare,
	.disable	= mtk_pll_unprepare,
	.recalc_rate	= mtk_pll_recalc_rate,
	.round_rate	= mtk_pll_round_rate,
	.set_rate	= mtk_pll_set_rate,
};
#else
static const struct clk_ops mtk_pll_ops = {
	.is_prepared	= mtk_pll_is_prepared,
	.prepare	= mtk_pll_prepare,
	.unprepare	= mtk_pll_unprepare,
	.recalc_rate	= mtk_pll_recalc_rate,
	.round_rate	= mtk_pll_round_rate,
	.set_rate	= mtk_pll_set_rate,
};
#endif

static struct clk *mtk_clk_register_pll(const struct mtk_pll_data *data,
		void __iomem *base)
{
	struct mtk_clk_pll *pll;
	struct clk_init_data init = {};
	struct clk *clk;
	const char *parent_name = "clk26m";

	pll = kzalloc(sizeof(*pll), GFP_KERNEL);
	if (!pll)
		return ERR_PTR(-ENOMEM);

	pll->base_addr = base + data->reg;
	pll->pwr_addr = base + data->pwr_reg;
	pll->pd_addr = base + data->pd_reg;
	pll->pcw_addr = base + data->pcw_reg;
	if (data->tuner_reg)
		pll->tuner_addr = base + data->tuner_reg;
	pll->hw.init = &init;
	pll->data = data;

	init.name = data->name;
	#if defined(MT_CCF_BRINGUP)
	init.ops = &mtk_pll_ops_dummy;
	#else
	init.ops = &mtk_pll_ops;
	#endif
	init.parent_names = &parent_name;
	init.num_parents = 1;

	clk = clk_register(NULL, &pll->hw);

	if (IS_ERR(clk))
		kfree(pll);

	return clk;
}

void __init mtk_clk_register_plls(struct device_node *node,
		const struct mtk_pll_data *plls, int num_plls, struct clk_onecell_data *clk_data)
{
	void __iomem *base;
	int i;
	struct clk *clk;

	base = of_iomap(node, 0);
	if (!base) {
		pr_err("%s(): ioremap failed\n", __func__);
		return;
	}

	for (i = 0; i < num_plls; i++) {
		const struct mtk_pll_data *pll = &plls[i];

		clk = mtk_clk_register_pll(pll, base);

		if (IS_ERR(clk)) {
			pr_err("Failed to register clk %s: %ld\n",
					pll->name, PTR_ERR(clk));
			continue;
		}

		clk_data->clks[pll->id] = clk;
	}
}
