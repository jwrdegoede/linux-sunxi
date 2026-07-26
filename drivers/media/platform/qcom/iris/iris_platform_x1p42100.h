/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#ifndef __IRIS_PLATFORM_X1P42100_H__
#define __IRIS_PLATFORM_X1P42100_H__

static const struct iris_power_domain_data x1p42100_ctrl_data = {
	.pd_names = (const char *[]) {
		"venus",
	},
	.pd_cnt = 1,
	.clk_names = (const char *[]) {
		"iface", "core",
	},
	.clk_cnt = 2,
};

static const struct iris_power_domain_data x1p42100_vcodec_data[] = {
	{
		.pd_names = (const char *[]) {
			"vcodec0",
		},
		.pd_cnt = 1,
		.clk_names = (const char *[]) {
			"vcodec0_core", "vcodec0_bse",
		},
		.clk_cnt = 2,
	},
};

static const char *const x1p42100_opp_clk_table[] = {
	"vcodec0_core",
	"vcodec0_bse",
	NULL,
};

#endif
