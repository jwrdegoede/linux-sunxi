/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2022-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef __IRIS_RESOURCES_H__
#define __IRIS_RESOURCES_H__

struct iris_core;
struct iris_power_domain;

int iris_opp_set_rate(struct device *dev, unsigned long freq);
int iris_unset_icc_bw(struct iris_core *core);
int iris_set_icc_bw(struct iris_core *core, unsigned long icc_bw);
struct device *iris_create_cb_dev(struct iris_core *core, const char *name);
struct device *iris_get_cb_dev(struct iris_inst *inst, enum iris_buffer_type buffer_type);
int iris_enable_power_domain_and_clocks(struct iris_core *core, struct iris_power_domain *pd);
void iris_disable_power_domain_and_clocks(struct iris_core *core, struct iris_power_domain *pd);
int iris_genpd_set_hwmode(struct iris_power_domain *pd, bool mode);

#endif
