// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2022-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/clk.h>
#include <linux/devfreq.h>
#include <linux/interconnect.h>
#include <linux/pm_domain.h>
#include <linux/pm_opp.h>
#include <linux/pm_runtime.h>

#include "iris_core.h"
#include "iris_resources.h"

#define BW_THRESHOLD 50000

int iris_set_icc_bw(struct iris_core *core, unsigned long icc_bw)
{
	unsigned long bw_kbps = 0, bw_prev = 0;
	const struct icc_info *icc_tbl;
	int ret = 0, i;

	icc_tbl = core->iris_platform_data->icc_tbl;

	for (i = 0; i < core->icc_count; i++) {
		if (!strcmp(core->icc_tbl[i].name, "video-mem")) {
			bw_kbps = icc_bw;
			bw_prev = core->power.icc_bw;

			bw_kbps = clamp_t(typeof(bw_kbps), bw_kbps,
					  icc_tbl[i].bw_min_kbps, icc_tbl[i].bw_max_kbps);

			if (abs(bw_kbps - bw_prev) < BW_THRESHOLD && bw_prev)
				return ret;

			core->icc_tbl[i].avg_bw = bw_kbps;

			core->power.icc_bw = bw_kbps;
			break;
		}
	}

	return icc_bulk_set_bw(core->icc_count, core->icc_tbl);
}

int iris_unset_icc_bw(struct iris_core *core)
{
	u32 i;

	core->power.icc_bw = 0;

	for (i = 0; i < core->icc_count; i++) {
		core->icc_tbl[i].avg_bw = 0;
		core->icc_tbl[i].peak_bw = 0;
	}

	return icc_bulk_set_bw(core->icc_count, core->icc_tbl);
}

int iris_opp_set_rate(struct device *dev, unsigned long freq)
{
	struct dev_pm_opp *opp __free(put_opp) =
		devfreq_recommended_opp(dev, &freq, 0);

	if (IS_ERR(opp))
		return PTR_ERR(opp);

	return dev_pm_opp_set_opp(dev, opp);
}

int iris_enable_power_domain_and_clocks(struct iris_core *core, struct iris_power_domain *pd)
{
	int ret, i;

	ret = iris_opp_set_rate(core->dev, ULONG_MAX);
	if (ret)
		return ret;

	for (i = 0; i < pd->pd_cnt; i++) {
		ret = pm_runtime_resume_and_get(pd->dev[i]);
		if (ret < 0)
			goto error;
	}

	ret = clk_bulk_prepare_enable(pd->clk_cnt, pd->clocks);
	if (ret)
		goto error;

	return 0;

error:
	iris_opp_set_rate(core->dev, 0);

	while (--i >= 0)
		pm_runtime_put_sync(pd->dev[i]);

	return ret;
}

void iris_disable_power_domain_and_clocks(struct iris_core *core, struct iris_power_domain *pd)
{
	int i;

	iris_opp_set_rate(core->dev, 0);
	clk_bulk_disable_unprepare(pd->clk_cnt, pd->clocks);

	for (i = 0; i < pd->pd_cnt; i++)
		pm_runtime_put_sync(pd->dev[i]);
}

int iris_genpd_set_hwmode(struct iris_power_domain *pd, bool mode)
{
	int i, ret;

	for (i = 0; i < pd->pd_cnt; i++) {
		ret = dev_pm_genpd_set_hwmode(pd->dev[i], mode);
		if (ret)
			goto error;
	}

	return 0;

error:
	while (--i >= 0)
		dev_pm_genpd_set_hwmode(pd->dev[i], !mode);

	return ret;
}
