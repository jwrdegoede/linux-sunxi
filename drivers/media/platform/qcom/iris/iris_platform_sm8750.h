/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2025 Linaro Ltd
 */

#ifndef __MEDIA_IRIS_PLATFORM_SM8750_H__
#define __MEDIA_IRIS_PLATFORM_SM8750_H__

static const char * const sm8750_clk_reset_table[] = {
	"bus0", "bus1", "core", "vcodec0_core"
};

static const struct iris_power_domain_data sm8750_ctrl_data = {
	.pd_names = (const char *[]) {
		"venus",
	},
	.pd_cnt = 1,
	.clk_names = (const char *[]) {
		"iface1", "core_freerun", "core",
	},
	.clk_cnt = 3,
};

static const struct iris_power_domain_data sm8750_vcodec_data[] = {
	{
		.pd_names = (const char *[]) {
			"vcodec0",
		},
		.pd_cnt = 1,
		.clk_names = (const char *[]) {
			"iface", "vcodec0_core_freerun", "vcodec0_core",
		},
		.clk_cnt = 3,
	},
};

#endif
