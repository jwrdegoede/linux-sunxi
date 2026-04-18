/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Helper code for acquiring and releasing resources (clks, regulators, etc.)
 * for devices with a firmware node following the simple-framebuffer compatible
 * from: Documentation/devicetree/bindings/display/simple-framebuffer.yaml
 *
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 *
 * Based on drivers/gpu/drm/sysfb/simpledrm.c.
 */
#ifndef _SIMPLEFB_RESOURCES_H_
#define _SIMPLEFB_RESOURCES_H_

#include <linux/kconfig.h>

struct clk;
struct device;
struct device_link;
struct regulator;

struct simplefb_resources {
#if defined CONFIG_OF && defined CONFIG_COMMON_CLK
	/* clocks */
	unsigned int clk_count;
	struct clk **clks;
#endif
#if defined CONFIG_OF && defined CONFIG_REGULATOR
	/* regulators */
	unsigned int regulator_count;
	struct regulator **regulators;
#endif
#if defined CONFIG_OF && defined CONFIG_PM_GENERIC_DOMAINS
	/* power-domains */
	int pwr_dom_count;
	struct device **pwr_dom_devs;
	struct device_link **pwr_dom_links;
#endif
};

int simplefb_acquire_resources(struct device *dev, struct simplefb_resources *res);
void simplefb_release_resources(struct simplefb_resources *res);

#endif
