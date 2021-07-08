// SPDX-License-Identifier: GPL-2.0
/* Author: Dan Scally <djrscally@gmail.com> */

#include <linux/dmi.h>
#include <linux/gpio/machine.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/regulator/machine.h>

static const struct dmi_system_id surface_go_2_dmi_table[] = {
	{
		.matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "Microsoft Corporation"),
			DMI_MATCH(DMI_PRODUCT_NAME, "Surface Go 2"),
		},
	},
	{ }
};

static struct regulator_consumer_supply ana_consumer_supplies[] = {
	REGULATOR_SUPPLY("avdd", "i2c-INT347A:00"),
};

static struct regulator_lookup ana_lookup = {
	.device_name = "i2c-INT3472:05",
	.regulator_name = "ANA",
	.init_data = {
		.constraints = {
			.min_uV = 2815200,
			.max_uV = 2815200,
			.apply_uV = 1,
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		},
		.num_consumer_supplies = ARRAY_SIZE(ana_consumer_supplies),
		.consumer_supplies = ana_consumer_supplies,
	},
};

static struct regulator_consumer_supply vsio_consumer_supplies[] = {
	REGULATOR_SUPPLY("dovdd", "i2c-INT347A:00"),
};

static struct regulator_lookup vsio_lookup = {
	.device_name = "i2c-INT3472:05",
	.regulator_name = "VSIO",
	.init_data = {
		.constraints = {
			.min_uV = 1800600,
			.max_uV = 1800600,
			.apply_uV = 1,
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		},
		.num_consumer_supplies = ARRAY_SIZE(vsio_consumer_supplies),
		.consumer_supplies = vsio_consumer_supplies,
	},
};

static struct regulator_consumer_supply core_consumer_supplies[] = {
	REGULATOR_SUPPLY("dvdd", "i2c-INT347A:00"),
};

static struct regulator_lookup core_lookup = {
	.device_name = "i2c-INT3472:05",
	.regulator_name = "CORE",
	.init_data = {
		.constraints = {
			.min_uV = 1200000,
			.max_uV = 1200000,
			.apply_uV = 1,
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		},
		.num_consumer_supplies = ARRAY_SIZE(core_consumer_supplies),
		.consumer_supplies = core_consumer_supplies,
	},
};

static struct gpiod_lookup_table surface_go_2_gpios = {
	.dev_id = "i2c-INT347A:00",
	.table = {
		GPIO_LOOKUP("tps68470-gpio", 9, "reset", GPIO_ACTIVE_LOW),
		GPIO_LOOKUP("tps68470-gpio", 7, "powerdown", GPIO_ACTIVE_LOW)
	}
};

static int __init surface_go_2_init(void)
{
	if (!dmi_check_system(surface_go_2_dmi_table))
		return -EINVAL;

	regulator_add_lookup(&ana_lookup);
	regulator_add_lookup(&vsio_lookup);
	regulator_add_lookup(&core_lookup);
	gpiod_add_lookup_table(&surface_go_2_gpios);

	return 0;
}
device_initcall(surface_go_2_init);
