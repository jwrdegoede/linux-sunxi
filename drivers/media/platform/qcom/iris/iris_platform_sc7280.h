/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#ifndef __IRIS_PLATFORM_SC7280_H__
#define __IRIS_PLATFORM_SC7280_H__

static const struct bw_info sc7280_bw_table_dec[] = {
	{ ((3840 * 2160) / 256) * 60, 1896000, },
	{ ((3840 * 2160) / 256) * 30,  968000, },
	{ ((1920 * 1080) / 256) * 60,  618000, },
	{ ((1920 * 1080) / 256) * 30,  318000, },
};

static const char * const sc7280_opp_pd_table[] = { "cx" };

static const struct iris_power_domain_data sc7280_ctrl_data = {
	.pd_names = (const char *[]) {
		"venus",
	},
	.pd_cnt = 1,
	.clk_names = (const char *[]) {
		"iface", "core", "bus",
	},
	.clk_cnt = 3,
};

static const struct iris_power_domain_data sc7280_vcodec_data[] = {
	{
		.pd_names = (const char *[]) {
			"vcodec0",
		},
		.pd_cnt = 1,
		.clk_names = (const char *[]) {
			"vcodec_core", "vcodec_bus",
		},
		.clk_cnt = 2,
	},
};

static const char * const sc7280_opp_clk_table[] = {
	"vcodec_core",
	NULL,
};

#endif
