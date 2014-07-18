/*
 * Copyright 2013 Emilio López
 *
 * Emilio López <emilio@elopez.com.ar>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/clk-provider.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/slab.h>

#define SUN4I_PLL2_ENABLE		31
#define SUN4I_PLL2_POST_DIV		26
#define SUN4I_PLL2_POST_DIV_MASK	0xF
#define SUN4I_PLL2_N			8
#define SUN4I_PLL2_N_MASK		0x7F
#define SUN4I_PLL2_PRE_DIV		0
#define SUN4I_PLL2_PRE_DIV_MASK		0x1F

#define SUN4I_PLL2_OUTPUTS		4

struct sun4i_pll2_clk {
	struct clk_hw hw;
	void __iomem *reg;
};

static inline struct sun4i_pll2_clk *to_sun4i_pll2_clk(struct clk_hw *hw)
{
	return container_of(hw, struct sun4i_pll2_clk, hw);
}

static unsigned long sun4i_pll2_1x_recalc_rate(struct clk_hw *hw,
					    unsigned long parent_rate)
{
	struct sun4i_pll2_clk *clk = to_sun4i_pll2_clk(hw);
	int n, prediv, postdiv;

	u32 val = readl(clk->reg);
	n = (val >> SUN4I_PLL2_N) & SUN4I_PLL2_N_MASK;
	prediv = (val >> SUN4I_PLL2_PRE_DIV) & SUN4I_PLL2_PRE_DIV_MASK;
	postdiv = (val >> SUN4I_PLL2_POST_DIV) & SUN4I_PLL2_POST_DIV_MASK;

	/* 0 is a special case and means 1 */
	if (n == 0)
		n = 1;
	if (prediv == 0)
		prediv = 1;
	if (postdiv == 0)
		postdiv = 1;

	return ((parent_rate * n) / prediv) / postdiv;
}

static unsigned long sun4i_pll2_8x_recalc_rate(struct clk_hw *hw,
					       unsigned long parent_rate)
{
	struct sun4i_pll2_clk *clk = to_sun4i_pll2_clk(hw);
	int n, prediv;

	u32 val = readl(clk->reg);
	n = (val >> SUN4I_PLL2_N) & SUN4I_PLL2_N_MASK;
	prediv = (val >> SUN4I_PLL2_PRE_DIV) & SUN4I_PLL2_PRE_DIV_MASK;

	/* 0 is a special case and means 1 */
	if (n == 0)
		n = 1;
	if (prediv == 0)
		prediv = 1;

	return ((parent_rate * 2 * n) / prediv);
}

static unsigned long sun4i_pll2_4x_recalc_rate(struct clk_hw *hw,
					       unsigned long parent_rate)
{
	return sun4i_pll2_8x_recalc_rate(hw, parent_rate / 2);
}

static unsigned long sun4i_pll2_2x_recalc_rate(struct clk_hw *hw,
					       unsigned long parent_rate)
{
	return sun4i_pll2_8x_recalc_rate(hw, parent_rate / 4);
}

static long sun4i_pll2_1x_round_rate(struct clk_hw *hw, unsigned long rate,
				     unsigned long *parent_rate)
{
	/*
	 * There is only two interesting rates for the audio PLL, the
	 * rest isn't really usable due to accuracy concerns. Therefore,
	 * we specifically round to those rates here
	 */
	if (rate < 22579200)
		return -EINVAL;

	if (rate >= 22579200 && rate < 24576000)
		return 22579200;

	return 24576000;
}

static long sun4i_pll2_8x_round_rate(struct clk_hw *hw, unsigned long rate,
				     unsigned long *parent_rate)
{
	/*
	 * We should account for the postdiv that we're undoing on PLL2x8,
	 * which is always 4 in the usable configurations. The division
	 * by two is done because PLL2x8 also doubles the rate
	 */
	*parent_rate = (rate * 4) / 2;

	return rate;
}

static long sun4i_pll2_4x_round_rate(struct clk_hw *hw, unsigned long rate,
				     unsigned long *parent_rate)
{
	/* PLL2x4 * 2 = PLL2x8 */
	return sun4i_pll2_8x_round_rate(hw, rate * 2, parent_rate);
}

static long sun4i_pll2_2x_round_rate(struct clk_hw *hw, unsigned long rate,
				     unsigned long *parent_rate)
{
	/* PLL2x2 * 4 = PLL2x8 */
	return sun4i_pll2_8x_round_rate(hw, rate * 4, parent_rate);
}

static int sun4i_pll2_set_rate(struct clk_hw *hw, unsigned long rate,
			       unsigned long parent_rate)
{
	struct sun4i_pll2_clk *clk = to_sun4i_pll2_clk(hw);
	u32 val = readl(clk->reg);

	val &= ~(SUN4I_PLL2_N_MASK << SUN4I_PLL2_N);
	val &= ~(SUN4I_PLL2_PRE_DIV_MASK << SUN4I_PLL2_PRE_DIV);
	val &= ~(SUN4I_PLL2_POST_DIV_MASK << SUN4I_PLL2_POST_DIV);

	val |= (21 << SUN4I_PLL2_PRE_DIV) | (4 << SUN4I_PLL2_POST_DIV);

	if (rate == 22579200)
		val |= (79 << SUN4I_PLL2_N);
	else if (rate == 24576000)
		val |= (86 << SUN4I_PLL2_N);
	else
		return -EINVAL;

	writel(val, clk->reg);

	return 0;
}

static struct clk_ops sun4i_pll2_ops_1x = {
	.recalc_rate = sun4i_pll2_1x_recalc_rate,
	.round_rate = sun4i_pll2_1x_round_rate,
	.set_rate = sun4i_pll2_set_rate,
};

static struct clk_ops sun4i_pll2_ops_2x = {
	.recalc_rate = sun4i_pll2_2x_recalc_rate,
	.round_rate = sun4i_pll2_2x_round_rate,
};

static struct clk_ops sun4i_pll2_ops_4x = {
	.recalc_rate = sun4i_pll2_4x_recalc_rate,
	.round_rate = sun4i_pll2_4x_round_rate,
};

static struct clk_ops sun4i_pll2_ops_8x = {
	.recalc_rate = sun4i_pll2_8x_recalc_rate,
	.round_rate = sun4i_pll2_8x_round_rate,
};

static void __init sun4i_pll2_setup(struct device_node *np)
{
	const char *clk_name = np->name, *parent;
	struct clk_onecell_data *clk_data;
	struct sun4i_pll2_clk *pll2;
	struct clk_gate *gate;
	struct clk **clks;
	void __iomem *reg;

	pll2 = kzalloc(sizeof(*pll2), GFP_KERNEL);
	gate = kzalloc(sizeof(*gate), GFP_KERNEL);
	clk_data = kzalloc(sizeof(*clk_data), GFP_KERNEL);
	clks = kcalloc(SUN4I_PLL2_OUTPUTS, sizeof(struct clk *), GFP_KERNEL);
	if (!pll2 || !gate || !clk_data || !clks)
		goto free_mem;

	reg = of_iomap(np, 0);
	parent = of_clk_get_parent_name(np, 0);
	of_property_read_string_index(np, "clock-output-names", 0, &clk_name);

	pll2->reg = reg;
	gate->reg = reg;
	gate->bit_idx = SUN4I_PLL2_ENABLE;

	/* PLL2, also known as PLL2x1 */
	of_property_read_string_index(np, "clock-output-names", 0, &clk_name);
	clks[0] = clk_register_composite(NULL, clk_name, &parent, 1, NULL, NULL,
					 &pll2->hw, &sun4i_pll2_ops_1x,
					 &gate->hw, &clk_gate_ops, 0);
	WARN_ON(IS_ERR(clks[0]));
	parent = clk_name;

	/* PLL2x2, 1/4 the rate of PLL2x8 */
	of_property_read_string_index(np, "clock-output-names", 1, &clk_name);
	clks[1] = clk_register_composite(NULL, clk_name, &parent, 1, NULL, NULL,
					 &pll2->hw, &sun4i_pll2_ops_2x,
					 NULL, NULL, CLK_SET_RATE_PARENT);
	WARN_ON(IS_ERR(clks[1]));

	/* PLL2x4, 1/2 the rate of PLL2x8 */
	of_property_read_string_index(np, "clock-output-names", 2, &clk_name);
	clks[2] = clk_register_composite(NULL, clk_name, &parent, 1, NULL, NULL,
					 &pll2->hw, &sun4i_pll2_ops_4x,
					 NULL, NULL, CLK_SET_RATE_PARENT);
	WARN_ON(IS_ERR(clks[2]));

	/* PLL2x8, double of PLL2 without the post divisor */
	of_property_read_string_index(np, "clock-output-names", 3, &clk_name);
	clks[3] = clk_register_composite(NULL, clk_name, &parent, 1, NULL, NULL,
					 &pll2->hw, &sun4i_pll2_ops_8x,
					 NULL, NULL, CLK_SET_RATE_PARENT);
	WARN_ON(IS_ERR(clks[3]));

	clk_data->clks = clks;
	clk_data->clk_num = SUN4I_PLL2_OUTPUTS;
	of_clk_add_provider(np, of_clk_src_onecell_get, clk_data);

	return;

free_mem:
	kfree(pll2);
	kfree(gate);
	kfree(clk_data);
	kfree(clks);
}
CLK_OF_DECLARE(sun4i_pll2, "allwinner,sun4i-a10-b-pll2-clk", sun4i_pll2_setup);
