// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Board info for Acer X86 tablets which ship with Android as the factory image
 * and which have broken DSDT tables. The factory kernels shipped on these
 * devices typically have a bunch of things hardcoded, rather than specified
 * in their DSDT.
 *
 * Copyright (C) 2021-2024 Hans de Goede <hansg@kernel.org>
 */

#include <linux/gpio/machine.h>
#include <linux/platform_device.h>
#include <linux/property.h>

#include "shared-psy-info.h"
#include "x86-android-tablets.h"

/* Acer Iconia One 8 A1-840 (non FHD version) */
static const struct property_entry acer_a1_840_4v20_battery_props[] = {
	PROPERTY_ENTRY_STRING("compatible", "simple-battery"),
	PROPERTY_ENTRY_STRING("device-chemistry", "lithium-ion-polymer"),
	/* The below values were taken from the Android kernel */
	PROPERTY_ENTRY_U32("precharge-current-microamp", 256000),
	PROPERTY_ENTRY_U32("charge-term-current-microamp", 128000),
	PROPERTY_ENTRY_U32("constant-charge-current-max-microamp", 2240000),
	PROPERTY_ENTRY_U32("constant-charge-voltage-max-microvolt", 4192000),
	PROPERTY_ENTRY_U32("factory-internal-resistance-micro-ohms", 150000),
	{ }
};

const struct software_node acer_a1_840_4v20_battery_node = {
	.properties = acer_a1_840_4v20_battery_props,
};

static const struct property_entry acer_a1_840_bq24190_props[] = {
	PROPERTY_ENTRY_REF("monitored-battery", &acer_a1_840_4v20_battery_node),
	PROPERTY_ENTRY_BOOL("omit-battery-class"),
	PROPERTY_ENTRY_BOOL("disable-reset"),
	{ }
};

static const struct software_node acer_a1_840_bq24190_node = {
	.properties = acer_a1_840_bq24190_props,
};

static const struct property_entry acer_a1_840_touchscreen_props[] = {
	PROPERTY_ENTRY_U32("touchscreen-size-x", 800),
	PROPERTY_ENTRY_U32("touchscreen-size-y", 1280),
	{ }
};

static const struct software_node acer_a1_840_touchscreen_node = {
	.properties = acer_a1_840_touchscreen_props,
};

static const struct x86_i2c_client_info acer_a1_840_i2c_clients[] __initconst = {
	{
		/* BQ24297 charger IC */
		.board_info = {
			.type = "bq24297",
			.addr = 0x6b,
			.dev_name = "bq24297",
			.swnode = &acer_a1_840_bq24190_node,
			.platform_data = &bq24190_pdata,
		},
		.adapter_path = "\\_SB_.I2C1",
		.irq_data = {
			.type = X86_ACPI_IRQ_TYPE_GPIOINT,
			.chip = "INT33FC:02",
			.index = 2,
			.trigger = ACPI_EDGE_SENSITIVE,
			.polarity = ACPI_ACTIVE_LOW,
			.con_id = "bq24297_irq",
		},
	}, {
		/* MPU6515 sensors */
		.board_info = {
			.type = "mpu6515",
			.addr = 0x69,
			.dev_name = "mpu6515",
		},
		.adapter_path = "\\_SB_.I2C3",
		.irq_data = {
			.type = X86_ACPI_IRQ_TYPE_APIC,
			.index = 0x47,
			.trigger = ACPI_EDGE_SENSITIVE,
			.polarity = ACPI_ACTIVE_HIGH,
		},
	}, {
		/* FT5416 touchscreen controller */
		.board_info = {
			.type = "edt-ft5x06",
			.addr = 0x38,
			.dev_name = "ft5416",
			.swnode = &acer_a1_840_touchscreen_node,
		},
		.adapter_path = "\\_SB_.I2C4",
		.irq_data = {
			.type = X86_ACPI_IRQ_TYPE_APIC,
			.index = 0x45,
			.trigger = ACPI_EDGE_SENSITIVE,
			.polarity = ACPI_ACTIVE_HIGH,
		},
	}
};

struct gpiod_lookup_table acer_a1_840_int3496_gpios = {
	.dev_id = "intel-int3496",
	.table = {
		GPIO_LOOKUP("INT33FC:02", 1, "mux", GPIO_ACTIVE_HIGH),
		GPIO_LOOKUP("INT33FC:02", 18, "id", GPIO_ACTIVE_HIGH),
		{ }
	},
};

static struct gpiod_lookup_table acer_a1_840_ft5416_gpios = {
	.dev_id = "i2c-ft5416",
	.table = {
		GPIO_LOOKUP("INT33FC:01", 26, "reset", GPIO_ACTIVE_LOW),
		{ }
	},
};

static struct gpiod_lookup_table acer_a1_840_fuel_gauge_gpios = {
	.dev_id = "chtdc_ti_battery",
	.table = {
		GPIO_LOOKUP("INT33FC:02", 10, "charged", GPIO_ACTIVE_HIGH),
		{ }
	},
};

static struct gpiod_lookup_table * const acer_a1_840_gpios[] = {
	&acer_a1_840_int3496_gpios,
	&acer_a1_840_ft5416_gpios,
	&acer_a1_840_fuel_gauge_gpios,
	NULL
};

/* Properties for the Dollar Cove TI PMIC battery MFD child used as fuel-gauge */
static const struct property_entry acer_a1_840_fuel_gauge_props[] = {
	PROPERTY_ENTRY_REF("monitored-battery", &acer_a1_840_4v20_battery_node),
	PROPERTY_ENTRY_STRING_ARRAY_LEN("supplied-from", bq24190_psy, 1),
	{ }
};

static int __init acer_a1_840_init(struct device *dev)
{
	struct device *bat_dev;
	int ret;

	bat_dev = bus_find_device_by_name(&platform_bus_type, NULL, "chtdc_ti_battery");
	if (!bat_dev)
		return dev_err_probe(dev, -EPROBE_DEFER, "getting chtdc_ti_battery dev\n");

	ret = device_create_managed_software_node(bat_dev, acer_a1_840_fuel_gauge_props, NULL);
	put_device(bat_dev);

	return ret;
}

const struct x86_dev_info acer_a1_840_info __initconst = {
	.i2c_client_info = acer_a1_840_i2c_clients,
	.i2c_client_count = ARRAY_SIZE(acer_a1_840_i2c_clients),
	.pdev_info = int3496_pdevs,
	.pdev_count = 1,
	.gpiod_lookup_tables = acer_a1_840_gpios,
	.bat_swnode = &acer_a1_840_4v20_battery_node,
	.init = acer_a1_840_init,
};

/* Acer Iconia One 7 B1-750 has an Android factory image with everything hardcoded */
static const char * const acer_b1_750_mount_matrix[] = {
	"-1", "0", "0",
	"0", "1", "0",
	"0", "0", "1"
};

static const struct property_entry acer_b1_750_bma250e_props[] = {
	PROPERTY_ENTRY_STRING_ARRAY("mount-matrix", acer_b1_750_mount_matrix),
	{ }
};

static const struct software_node acer_b1_750_bma250e_node = {
	.properties = acer_b1_750_bma250e_props,
};

static const struct x86_i2c_client_info acer_b1_750_i2c_clients[] __initconst = {
	{
		/* Novatek NVT-ts touchscreen */
		.board_info = {
			.type = "nt11205-ts",
			.addr = 0x34,
			.dev_name = "NVT-ts",
		},
		.adapter_path = "\\_SB_.I2C4",
		.irq_data = {
			.type = X86_ACPI_IRQ_TYPE_GPIOINT,
			.chip = "INT33FC:02",
			.index = 3,
			.trigger = ACPI_EDGE_SENSITIVE,
			.polarity = ACPI_ACTIVE_LOW,
			.con_id = "NVT-ts_irq",
		},
	}, {
		/* BMA250E accelerometer */
		.board_info = {
			.type = "bma250e",
			.addr = 0x18,
			.swnode = &acer_b1_750_bma250e_node,
		},
		.adapter_path = "\\_SB_.I2C3",
		.irq_data = {
			.type = X86_ACPI_IRQ_TYPE_GPIOINT,
			.chip = "INT33FC:02",
			.index = 25,
			.trigger = ACPI_LEVEL_SENSITIVE,
			.polarity = ACPI_ACTIVE_HIGH,
			.con_id = "bma250e_irq",
		},
	},
};

static struct gpiod_lookup_table acer_b1_750_nvt_ts_gpios = {
	.dev_id = "i2c-NVT-ts",
	.table = {
		GPIO_LOOKUP("INT33FC:01", 26, "reset", GPIO_ACTIVE_LOW),
		{ }
	},
};

static struct gpiod_lookup_table * const acer_b1_750_gpios[] = {
	&acer_b1_750_nvt_ts_gpios,
	&int3496_reference_gpios,
	NULL
};

const struct x86_dev_info acer_b1_750_info __initconst = {
	.i2c_client_info = acer_b1_750_i2c_clients,
	.i2c_client_count = ARRAY_SIZE(acer_b1_750_i2c_clients),
	.pdev_info = int3496_pdevs,
	.pdev_count = 1,
	.gpiod_lookup_tables = acer_b1_750_gpios,
};
