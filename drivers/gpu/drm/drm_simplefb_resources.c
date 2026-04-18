// SPDX-License-Identifier: GPL-2.0-only
/*
 * Helper code for acquiring and releasing resources (clks, regulators, etc.)
 * for devices with a firmware node following the simple-framebuffer compatible
 * from: Documentation/devicetree/bindings/display/simple-framebuffer.yaml
 *
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 *
 * Based on drivers/gpu/drm/sysfb/simpledrm.c.
 */

#include <linux/clk.h>
#include <linux/of_clk.h>
#include <linux/pm_domain.h>
#include <linux/regulator/consumer.h>

#include <drm/simplefb_resources.h>

#if defined CONFIG_OF && defined CONFIG_COMMON_CLK
/*
 * Clock handling code.
 *
 * Here we handle the clocks property of our "simple-framebuffer" dt node.
 * This is necessary so that we can make sure that any clocks needed by
 * the display engine that the bootloader set up for us (and for which it
 * provided a simplefb dt node), stay up, for the life of the simplefb
 * driver.
 *
 * When the driver unloads, we cleanly disable, and then release the clocks.
 *
 * We only complain about errors here, no action is taken as the most likely
 * error can only happen due to a mismatch between the bootloader which set
 * up simplefb, and the clock definitions in the device tree. Chances are
 * that there are no adverse effects, and if there are, a clean teardown of
 * the fb probe will not help us much either. So just complain and carry on,
 * and hope that the user actually gets a working fb at the end of things.
 */
static int simplefb_get_clocks(struct device *dev, struct simplefb_resources *res)
{
	struct clk *clock;
	unsigned int i;
	int ret;

	res->clk_count = of_clk_get_parent_count(dev->of_node);
	if (!res->clk_count)
		return 0;

	res->clks = kzalloc_objs(struct clk *, res->clk_count);
	if (!res->clks)
		return -ENOMEM;

	for (i = 0; i < res->clk_count; ++i) {
		clock = of_clk_get(dev->of_node, i);
		if (IS_ERR(clock)) {
			ret = dev_err_probe(dev, PTR_ERR(clock), "getting clock %u\n", i);
			if (ret == -EPROBE_DEFER)
				goto err;
			continue;
		}
		res->clks[i] = clock;
	}

	return 0;

err:
	while (i--)
		clk_put(res->clks[i]);

	kfree(res->clks);
	return ret;
}

static void simplefb_enable_clocks(struct device *dev, struct simplefb_resources *res)
{
	int ret;

	for (unsigned int i = 0; i < res->clk_count; ++i) {
		ret = clk_prepare_enable(res->clks[i]);
		if (ret)
			dev_err(dev, "Error %pe enabling clock %u\n", ERR_PTR(ret), i);
	}
}

static void simplefb_release_clocks(struct simplefb_resources *res, bool enabled)
{
	for (unsigned int i = 0; i < res->clk_count; ++i) {
		if (enabled)
			clk_disable_unprepare(res->clks[i]);
		clk_put(res->clks[i]);
	}
	kfree(res->clks);
}
#else
static int simplefb_get_clocks(struct device *dev, struct simplefb_resources *res)
{
	return 0;
}
static void simplefb_enable_clocks(struct device *dev, struct simplefb_resources *res) { }
static void simplefb_release_clocks(struct simplefb_resources *res, bool enabled) { }
#endif

#if defined CONFIG_OF && defined CONFIG_REGULATOR

#define SUPPLY_SUFFIX "-supply"

/*
 * Regulator handling code.
 *
 * Here we handle the num-supplies and vin*-supply properties of our
 * "simple-framebuffer" dt node. This is necessary so that we can make sure
 * that any regulators needed by the display hardware that the bootloader
 * set up for us (and for which it provided a simplefb dt node), stay up,
 * for the life of the simplefb driver.
 *
 * When the driver unloads, we cleanly disable, and then release the
 * regulators.
 *
 * We only complain about errors here, no action is taken as the most likely
 * error can only happen due to a mismatch between the bootloader which set
 * up simplefb, and the regulator definitions in the device tree. Chances are
 * that there are no adverse effects, and if there are, a clean teardown of
 * the fb probe will not help us much either. So just complain and carry on,
 * and hope that the user actually gets a working fb at the end of things.
 */
static int simplefb_get_regulators(struct device *dev, struct simplefb_resources *res)
{
	unsigned int count = 0, i = 0;
	struct regulator *regulator;
	struct property *prop;
	const char *p;
	int ret;

	/* Count the number of regulator supplies */
	for_each_property_of_node(dev->of_node, prop) {
		p = strstr(prop->name, SUPPLY_SUFFIX);
		if (p && p != prop->name)
			++count;
	}

	if (!count)
		return 0;

	res->regulators = kzalloc_objs(struct regulator *, count);
	if (!res->regulators)
		return -ENOMEM;

	for_each_property_of_node(dev->of_node, prop) {
		char name[32]; /* 32 is max size of property name */
		size_t len;

		p = strstr(prop->name, SUPPLY_SUFFIX);
		if (!p || p == prop->name)
			continue;
		len = strlen(prop->name) - strlen(SUPPLY_SUFFIX) + 1;
		strscpy(name, prop->name, min(sizeof(name), len));

		regulator = regulator_get_optional(dev, name);
		if (IS_ERR(regulator)) {
			ret = dev_err_probe(dev, PTR_ERR(regulator),
					    "getting regulator %s\n", name);
			if (ret == -EPROBE_DEFER)
				goto err;
			continue;
		}
		res->regulators[i++] = regulator;
	}
	res->regulator_count = i;

	return 0;

err:
	while (i--)
		regulator_put(res->regulators[i]);

	kfree(res->regulators);
	return ret;
}

static void simplefb_enable_regulators(struct device *dev, struct simplefb_resources *res)
{
	int ret;

	for (unsigned int i = 0; i < res->regulator_count; ++i) {
		ret = regulator_enable(res->regulators[i]);
		if (ret)
			dev_err(dev, "Error %pe enabling regulator %u\n", ERR_PTR(ret), i);
	}
}

static void simplefb_release_regulators(struct simplefb_resources *res, bool enabled)
{
	for (unsigned int i = 0; i < res->regulator_count; ++i) {
		if (enabled)
			regulator_disable(res->regulators[i]);
		regulator_put(res->regulators[i]);
	}
	kfree(res->regulators);
}
#else
static int simplefb_get_regulators(struct device *dev, struct simplefb_resources *res)
{
	return 0;
}
static void simplefb_enable_regulators(struct device *dev, struct simplefb_resources *res) { }
static void simplefb_release_regulators(struct simplefb_resources *res, bool enabled) { }
#endif

#if defined CONFIG_OF && defined CONFIG_PM_GENERIC_DOMAINS
/*
 * Generic power domain handling code.
 *
 * Here we handle the power-domains properties of our "simple-framebuffer"
 * dt node. This is only necessary if there is more than one power-domain.
 * A single power-domains is handled automatically by the driver core. Multiple
 * power-domains have to be handled by drivers since the driver core can't know
 * the correct power sequencing. Power sequencing is not an issue for simplefb
 * since the bootloader has put the power domains already in the correct state.
 * simplefb has only to ensure they remain active for its lifetime.
 *
 * When the driver unloads, we detach from the power-domains.
 *
 * We only complain about errors here, no action is taken as the most likely
 * error can only happen due to a mismatch between the bootloader which set
 * up the "simple-framebuffer" dt node, and the PM domain providers in the
 * device tree. Chances are that there are no adverse effects, and if there are,
 * a clean teardown of the fb probe will not help us much either. So just
 * complain and carry on, and hope that the user actually gets a working fb at
 * the end of things.
 */
static void simplefb_detach_genpd(struct simplefb_resources *res)
{
	int i;

	if (res->pwr_dom_count <= 1)
		return;

	for (i = res->pwr_dom_count - 1; i >= 0; i--) {
		if (res->pwr_dom_links[i])
			device_link_del(res->pwr_dom_links[i]);
		if (!IS_ERR_OR_NULL(res->pwr_dom_devs[i]))
			dev_pm_domain_detach(res->pwr_dom_devs[i], true);
	}
	kfree(res->pwr_dom_links);
	kfree(res->pwr_dom_devs);
}

static int simplefb_attach_genpd(struct device *dev, struct simplefb_resources *res)
{
	int i, ret;

	res->pwr_dom_count = of_count_phandle_with_args(dev->of_node, "power-domains",
							"#power-domain-cells");
	/*
	 * Single power-domain devices are handled by driver core nothing to do
	 * here. The same for device nodes without "power-domains" property.
	 */
	if (res->pwr_dom_count <= 1)
		return 0;

	res->pwr_dom_devs = kzalloc_objs(struct device *, res->pwr_dom_count);
	res->pwr_dom_links = kzalloc_objs(struct device_link *, res->pwr_dom_count);
	if (!res->pwr_dom_devs || !res->pwr_dom_links) {
		kfree(res->pwr_dom_links);
		kfree(res->pwr_dom_devs);
		return -ENOMEM;
	}

	for (i = 0; i < res->pwr_dom_count; i++) {
		res->pwr_dom_devs[i] = dev_pm_domain_attach_by_id(dev, i);
		if (IS_ERR(res->pwr_dom_devs[i])) {
			ret = dev_err_probe(dev, PTR_ERR(res->pwr_dom_devs[i]),
					    "attaching PM domain %d\n", i);
			if (ret == -EPROBE_DEFER) {
				simplefb_detach_genpd(res);
				return ret;
			}
			continue;
		}

		res->pwr_dom_links[i] = device_link_add(dev,
							res->pwr_dom_devs[i],
							DL_FLAG_STATELESS |
							DL_FLAG_PM_RUNTIME |
							DL_FLAG_RPM_ACTIVE);
		if (!res->pwr_dom_links[i])
			dev_warn(dev, "failed to link PM domain %d\n", i);
	}

	return 0;
}
#else
static int simplefb_attach_genpd(struct device *dev, struct simplefb_resources *res)
{
	return 0;
}
static void simplefb_detach_genpd(struct simplefb_resources *res) { }
#endif

int simplefb_acquire_resources(struct device *dev, struct simplefb_resources *res)
{
	int ret;

	if (!dev->of_node)
		return 0;

	ret = simplefb_get_clocks(dev, res);
	if (ret)
		return ret;

	ret = simplefb_get_regulators(dev, res);
	if (ret) {
		simplefb_release_clocks(res, false);
		return ret;
	}

	ret = simplefb_attach_genpd(dev, res);
	if (ret) {
		simplefb_release_regulators(res, false);
		simplefb_release_clocks(res, false);
		return ret;
	}

	simplefb_enable_clocks(dev, res);
	simplefb_enable_regulators(dev, res);

	return 0;
}

void simplefb_release_resources(struct simplefb_resources *res)
{
	simplefb_release_regulators(res, true);
	simplefb_release_clocks(res, true);
	simplefb_detach_genpd(res);
}
