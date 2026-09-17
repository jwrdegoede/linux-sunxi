/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef __IRIS_PLATFORM_SM8550_H__
#define __IRIS_PLATFORM_SM8550_H__

static const char * const sm8550_clk_reset_table[] = { "bus" };

static const struct iris_power_domain_data sm8550_ctrl_data = {
	.pd_names = (const char *[]) {
		"venus",
	},
	.pd_cnt = 1,
	.clk_names = (const char *[]) {
		"iface", "core",
	},
	.clk_cnt = 2,
};

static const struct iris_power_domain_data sm8550_vcodec_data[] = {
	{
		.pd_names = (const char *[]) {
			"vcodec0",
		},
		.pd_cnt = 1,
		.clk_names = (const char *[]) {
			"vcodec0_core",
		},
		.clk_cnt = 1,
	},
};

static struct platform_inst_caps platform_inst_cap_sm8550 = {
	.min_frame_width = 96,
	.max_frame_width = 8192,
	.min_frame_height = 96,
	.max_frame_height = 8192,
	.max_mbpf = (8192 * 4352) / 256,
	.mb_cycles_vpp = 200,
	.mb_cycles_fw = 489583,
	.mb_cycles_fw_vpp = 66234,
	.max_frame_rate = MAXIMUM_FPS,
	.max_operating_rate = MAXIMUM_FPS,
};

#endif
