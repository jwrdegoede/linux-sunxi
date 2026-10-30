// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2019-2020, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 * Copyright (c) 2024 Linaro Limited
 */

#include <linux/array_size.h>
#include <linux/bits.h>
#include <linux/device.h>
#include <linux/math.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>

#include <asm/byteorder.h>

#define DEFAULT_VOLTAGE_STEPPER_RATE	38400

#define LDO_STEPPER_CTL_REG		0x3b
#define STEP_RATE_MASK			GENMASK(1, 0)

#define LDO_VSET_LB_REG			0x40

#define LDO_ENABLE_REG			0x46
#define ENABLE_BIT			BIT(7)

struct pm8008_regulator {
	struct regmap		*regmap;
	struct regulator_desc	desc;
	unsigned int		base;
};

struct pm8008_regulator_data {
	const char			*name;
	const char			*supply_name;
	unsigned int			base;
	int				min_dropout_uV;
	const struct linear_range	*voltage_range;
	int				n_linear_ranges;
};

struct pm8008_match_data {
	const bool has_stepper_ctl_reg;
	const struct pm8008_regulator_data *regulator_data;
	const int num_regulators;
};

static const struct linear_range pm8008_nldo_ranges[] = {
	REGULATOR_LINEAR_RANGE(528000, 0, 122, 8000),
};

static const struct linear_range pm8008_pldo_ranges[] = {
	REGULATOR_LINEAR_RANGE(1504000, 0, 237, 8000),
};

static const struct linear_range pm8010_nldo_ranges[] = {
	REGULATOR_LINEAR_RANGE(528000, 0, 127, 8000),
};

static const struct linear_range pm8010_pldo_ranges[] = {
	REGULATOR_LINEAR_RANGE(1504000, 0, 255, 8000),
};

static const struct linear_range pm8010_pldo_lv_ranges[] = {
	REGULATOR_LINEAR_RANGE(1800000, 0,  2,  200000),
	REGULATOR_LINEAR_RANGE(2608000, 3,  28, 16000),
	REGULATOR_LINEAR_RANGE(3104000, 29, 30, 96000),
	REGULATOR_LINEAR_RANGE(3312000, 31, 31, 0),
};

#define PM8008_REGULATOR(_name, _supply, _base, _dropout, _range)	\
	{ _name, _supply, _base, _dropout, _range, ARRAY_SIZE(_range) }

static const struct pm8008_regulator_data pm8008_reg_data[] = {
	PM8008_REGULATOR("ldo1", "vdd-l1-l2", 0x4000, 225000, pm8008_nldo_ranges),
	PM8008_REGULATOR("ldo2", "vdd-l1-l2", 0x4100, 225000, pm8008_nldo_ranges),
	PM8008_REGULATOR("ldo3", "vdd-l3-l4", 0x4200, 300000, pm8008_pldo_ranges),
	PM8008_REGULATOR("ldo4", "vdd-l3-l4", 0x4300, 300000, pm8008_pldo_ranges),
	PM8008_REGULATOR("ldo5", "vdd-l5",    0x4400, 200000, pm8008_pldo_ranges),
	PM8008_REGULATOR("ldo6", "vdd-l6",    0x4500, 200000, pm8008_pldo_ranges),
	PM8008_REGULATOR("ldo7", "vdd-l7",    0x4600, 200000, pm8008_pldo_ranges),
};

static const struct pm8008_regulator_data pm8010_reg_data[] = {
	PM8008_REGULATOR("ldo1", "vdd-l1-l2", 0x4000, 172000, pm8010_nldo_ranges),
	PM8008_REGULATOR("ldo2", "vdd-l1-l2", 0x4100, 172000, pm8010_nldo_ranges),
	PM8008_REGULATOR("ldo3", "vdd-l3-l4", 0x4200, 80000, pm8010_pldo_lv_ranges),
	PM8008_REGULATOR("ldo4", "vdd-l3-l4", 0x4300, 80000, pm8010_pldo_lv_ranges),
	PM8008_REGULATOR("ldo5", "vdd-l5",    0x4400, 296000, pm8010_pldo_ranges),
	PM8008_REGULATOR("ldo6", "vdd-l6",    0x4500, 80000, pm8010_pldo_lv_ranges),
	PM8008_REGULATOR("ldo7", "vdd-l7",    0x4600, 296000, pm8010_pldo_ranges),
};

static int pm8008_regulator_set_voltage_sel(struct regulator_dev *rdev, unsigned int sel)
{
	struct pm8008_regulator *preg = rdev_get_drvdata(rdev);
	unsigned int mV;
	__le16 val;
	int ret;

	ret = regulator_list_voltage_linear_range(rdev, sel);
	if (ret < 0)
		return ret;

	mV = DIV_ROUND_UP(ret, 1000);

	val = cpu_to_le16(mV);

	ret = regmap_bulk_write(preg->regmap, preg->base + LDO_VSET_LB_REG,
			&val, sizeof(val));
	if (ret < 0)
		return ret;

	return 0;
}

static int pm8008_regulator_get_voltage_sel(struct regulator_dev *rdev)
{
	struct pm8008_regulator *preg = rdev_get_drvdata(rdev);
	unsigned int uV;
	__le16 val;
	int ret;

	ret = regmap_bulk_read(preg->regmap, preg->base + LDO_VSET_LB_REG,
			&val, sizeof(val));
	if (ret < 0)
		return ret;

	uV = le16_to_cpu(val) * 1000;

	return regulator_map_voltage_linear_range(rdev, uV, INT_MAX);
}

static const struct regulator_ops pm8008_regulator_ops = {
	.list_voltage		= regulator_list_voltage_linear_range,
	.set_voltage_sel	= pm8008_regulator_set_voltage_sel,
	.get_voltage_sel	= pm8008_regulator_get_voltage_sel,
	.enable			= regulator_enable_regmap,
	.disable		= regulator_disable_regmap,
	.is_enabled		= regulator_is_enabled_regmap,
};

static int pm8008_regulator_probe(struct platform_device *pdev)
{
	const struct pm8008_match_data *match_data;
	const struct pm8008_regulator_data *data;
	struct regulator_config config = {};
	const struct platform_device_id *id;
	struct device *dev = &pdev->dev;
	struct pm8008_regulator *preg;
	struct regulator_desc *desc;
	struct regulator_dev *rdev;
	struct regmap *regmap;
	unsigned int val;
	int ret, i;

	id = platform_get_device_id(pdev);
	if (!id)
		return dev_err_probe(dev, -ENODEV, "Missing platform device id\n");

	match_data = (const struct pm8008_match_data *)id->driver_data;
	if (!match_data)
		return dev_err_probe(dev, -ENODATA, "Missing driver match data\n");

	regmap = dev_get_regmap(dev->parent, "secondary");
	if (!regmap)
		return -EINVAL;

	for (i = 0; i < match_data->num_regulators; i++) {
		data = &match_data->regulator_data[i];

		preg = devm_kzalloc(dev, sizeof(*preg), GFP_KERNEL);
		if (!preg)
			return -ENOMEM;

		preg->regmap = regmap;
		preg->base = data->base;

		desc = &preg->desc;

		desc->name = data->name;
		desc->supply_name = data->supply_name;
		desc->of_match = data->name;
		desc->regulators_node = of_match_ptr("regulators");
		desc->ops = &pm8008_regulator_ops;
		desc->type = REGULATOR_VOLTAGE;
		desc->owner = THIS_MODULE;

		desc->linear_ranges = data->voltage_range;
		desc->n_linear_ranges = data->n_linear_ranges;
		desc->n_voltages = linear_range_values_in_range_array(desc->linear_ranges,
								      desc->n_linear_ranges);

		if (match_data->has_stepper_ctl_reg) {
			ret = regmap_read(regmap, preg->base + LDO_STEPPER_CTL_REG, &val);
			if (ret < 0) {
				dev_err(dev, "failed to read step rate: %d\n", ret);
				return ret;
			}
			val &= STEP_RATE_MASK;
			desc->ramp_delay = DEFAULT_VOLTAGE_STEPPER_RATE >> val;
		} else {
			desc->ramp_delay = DEFAULT_VOLTAGE_STEPPER_RATE;
		}

		desc->min_dropout_uV = data->min_dropout_uV;

		desc->enable_reg = preg->base + LDO_ENABLE_REG;
		desc->enable_mask = ENABLE_BIT;

		config.dev = dev->parent;
		config.driver_data = preg;
		config.regmap = regmap;

		rdev = devm_regulator_register(dev, desc, &config);
		if (IS_ERR(rdev)) {
			ret = PTR_ERR(rdev);
			dev_err(dev, "failed to register regulator %s: %d\n",
					desc->name, ret);
			return ret;
		}
	}

	return 0;
}

static const struct pm8008_match_data pm8008_data = {
	.has_stepper_ctl_reg = true,
	.regulator_data = pm8008_reg_data,
	.num_regulators = ARRAY_SIZE(pm8008_reg_data),
};

static const struct pm8008_match_data pm8010_data = {
	.has_stepper_ctl_reg = false,
	.regulator_data = pm8010_reg_data,
	.num_regulators = ARRAY_SIZE(pm8010_reg_data),
};

static const struct platform_device_id pm8008_regulator_id_table[] = {
	{ .name = "pm8008-regulator", .driver_data = (kernel_ulong_t)&pm8008_data },
	{ .name = "pm8010-regulator", .driver_data = (kernel_ulong_t)&pm8010_data },
	{ }
};
MODULE_DEVICE_TABLE(platform, pm8008_regulator_id_table);

static struct platform_driver pm8008_regulator_driver = {
	.driver	= {
		.name = "qcom-pm8008-regulator",
	},
	.probe = pm8008_regulator_probe,
	.id_table = pm8008_regulator_id_table,
};
module_platform_driver(pm8008_regulator_driver);

MODULE_DESCRIPTION("Qualcomm PM8008 PMIC regulator driver");
MODULE_LICENSE("GPL");
