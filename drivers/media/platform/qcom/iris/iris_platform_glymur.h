/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef __IRIS_PLATFORM_GLYMUR_H__
#define __IRIS_PLATFORM_GLYMUR_H__

extern const struct iris_power_domain_data iris_glymur_ctrl_data;
extern const struct iris_power_domain_data iris_glymur_vcodec_data[2];
extern const char * const iris_glymur_clk_reset_table[6];
extern const char * const iris_glymur_opp_clk_table[4];
extern const struct tz_cp_config iris_glymur_tz_cp_config[3];

#endif /* __IRIS_PLATFORM_GLYMUR_H__ */
