// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2025 Intel Corporation.
 * Copyright (c) 2025 Red Hat, Inc.
 */

#include <linux/acpi.h>
#include <linux/auxiliary_bus.h>
#include <linux/dev_printk.h>
#include <linux/device.h>
#include <linux/gpio/driver.h>
#include <linux/types.h>
#include <linux/usb/usbio.h>

struct usbio_gpio_bank {
	u8 config[USBIO_GPIOSPERBANK];
	u32 bitmap;
};

struct usbio_gpio {
	struct usbio_gpio_bank banks[USBIO_MAX_GPIOBANKS];
	struct gpio_chip gc;
	struct usbio_client *client;
};

static const struct acpi_device_id usbio_gpio_acpi_hids[] = {
	{ "INTC1007" }, /* MTL */
	{ "INTC10B2" }, /* ARL */
	{ "INTC10B5" }, /* LNL */
	{ "INTC10E2" }, /* PTL */
	{ }
};

static bool usbio_gpio_get_bank_and_pin(struct gpio_chip *gc, unsigned int offset,
					struct usbio_gpio_bank **bank_ret, int *pin_ret)
{
	struct usbio_gpio *gpio = gpiochip_get_data(gc);
	struct device *dev = &gpio->client->adev.dev;
	struct usbio_gpio_bank *bank;
	int pin;

	if (offset >= gc->ngpio)
		return false;

	bank = &gpio->banks[offset / USBIO_GPIOSPERBANK];
	pin = offset % USBIO_GPIOSPERBANK;
	if (~bank->bitmap & BIT(pin)) {
		/* The FW bitmap sometimes is invalid, warn and continue */
		dev_warn_once(dev, FW_BUG "GPIO %u is not in FW pins bitmap\n", offset);
	}

	*bank_ret = bank;
	*pin_ret = pin;
	return true;
}

static int usbio_gpio_get_direction(struct gpio_chip *gc, unsigned int offset)
{
	struct usbio_gpio_bank *bank;
	int pin;
	u8 cfg;

	if (!usbio_gpio_get_bank_and_pin(gc, offset, &bank, &pin))
		return -EINVAL;

	cfg = bank->config[pin] & USBIO_GPIO_PINMOD_MASK;
	if (cfg == USBIO_GPIO_PINMOD_INPUT)
		return GPIO_LINE_DIRECTION_IN;
	else if (cfg == USBIO_GPIO_PINMOD_OUTPUT)
		return GPIO_LINE_DIRECTION_OUT;

	return GPIO_LINE_DIRECTION_IN;
}

static int usbio_gpio_get(struct gpio_chip *gc, unsigned int offset)
{
	struct usbio_gpio *gpio = gpiochip_get_data(gc);
	struct usbio_gpio_bank *bank;
	struct usbio_gpio_rw gbuf;
	int pin, ret;

	if (!usbio_gpio_get_bank_and_pin(gc, offset, &bank, &pin))
		return -EINVAL;

	gbuf.bankid = offset / USBIO_GPIOSPERBANK;
	gbuf.pincount  = 1;
	gbuf.pin = pin;

	ret = usbio_control_msg(gpio->client, USBIO_PKTTYPE_GPIO, USBIO_GPIOCMD_READ,
				&gbuf, sizeof(gbuf) - sizeof(gbuf.value),
				&gbuf, sizeof(gbuf));

	return (ret == sizeof(gbuf)) ? (gbuf.value >> pin) & 1 : -EINVAL;
}

static void usbio_gpio_set(struct gpio_chip *gc, unsigned int offset,
		int value)
{
	struct usbio_gpio *gpio = gpiochip_get_data(gc);
	struct usbio_gpio_bank *bank;
	struct usbio_gpio_rw gbuf;
	int pin;

	if (!usbio_gpio_get_bank_and_pin(gc, offset, &bank, &pin))
		return;

	gbuf.bankid = offset / USBIO_GPIOSPERBANK;
	gbuf.pincount  = 1;
	gbuf.pin = pin;
	gbuf.value = value << pin;

	usbio_control_msg(gpio->client, USBIO_PKTTYPE_GPIO, USBIO_GPIOCMD_WRITE,
			  &gbuf, sizeof(gbuf), NULL, 0);
}

static int usbio_gpio_update_config(struct gpio_chip *gc, unsigned int offset,
				    u8 mask, u8 value)
{
	struct usbio_gpio *gpio = gpiochip_get_data(gc);
	struct usbio_gpio_bank *bank;
	struct usbio_gpio_init gbuf;
	int pin;

	if (!usbio_gpio_get_bank_and_pin(gc, offset, &bank, &pin))
		return -EINVAL;

	bank->config[pin] &= ~mask;
	bank->config[pin] |= value;

	gbuf.bankid = offset / USBIO_GPIOSPERBANK;
	gbuf.config = bank->config[pin];
	gbuf.pincount  = 1;
	gbuf.pin = pin;

	return usbio_control_msg(gpio->client, USBIO_PKTTYPE_GPIO, USBIO_GPIOCMD_INIT,
				 &gbuf, sizeof(gbuf), NULL, 0);
}

static int usbio_gpio_direction_input(struct gpio_chip *gc, unsigned int offset)
{
	return usbio_gpio_update_config(gc, offset, USBIO_GPIO_PINMOD_MASK,
					USBIO_GPIO_SET_PINMOD(USBIO_GPIO_PINMOD_INPUT));
}

static int usbio_gpio_direction_output(struct gpio_chip *gc,
		unsigned int offset, int value)
{
	int ret;

	ret = usbio_gpio_update_config(gc, offset, USBIO_GPIO_PINMOD_MASK,
				       USBIO_GPIO_SET_PINMOD(USBIO_GPIO_PINMOD_OUTPUT));
	if (ret)
		return ret;

	usbio_gpio_set(gc, offset, value);
	return 0;
}

static int usbio_gpio_set_config(struct gpio_chip *gc, unsigned int offset,
		unsigned long config)
{
	u8 value;

	switch (pinconf_to_config_param(config)) {
	case PIN_CONFIG_BIAS_PULL_PIN_DEFAULT:
		value = USBIO_GPIO_SET_PINCFG(USBIO_GPIO_PINCFG_DEFAULT);
		break;
	case PIN_CONFIG_BIAS_PULL_UP:
		value = USBIO_GPIO_SET_PINCFG(USBIO_GPIO_PINCFG_PULLUP);
		break;
	case PIN_CONFIG_BIAS_PULL_DOWN:
		value = USBIO_GPIO_SET_PINCFG(USBIO_GPIO_PINCFG_PULLDOWN);
		break;
	case PIN_CONFIG_DRIVE_PUSH_PULL:
		value = USBIO_GPIO_SET_PINCFG(USBIO_GPIO_PINCFG_PUSHPULL);
		break;
	default:
		return -ENOTSUPP;
	}

	return usbio_gpio_update_config(gc, offset, USBIO_GPIO_PINCFG_MASK, value);
}

static int usbio_gpio_probe(struct auxiliary_device *adev,
		const struct auxiliary_device_id *adev_id)
{
	struct usbio_gpio_bank_desc *bank_desc;
	struct device *dev = &adev->dev;
	struct usbio_gpio *gpio;
	int bank, ret;

	bank_desc = dev_get_platdata(dev);
	if (!bank_desc)
		return -EINVAL;

	gpio = devm_kzalloc(dev, sizeof(*gpio), GFP_KERNEL);
	if (!gpio)
		return -ENOMEM;

	gpio->client = auxiliary_get_usbio_client(adev);

	usbio_acpi_bind(gpio->client, usbio_gpio_acpi_hids);

	for (bank = 0; bank < USBIO_MAX_GPIOBANKS && bank_desc[bank].bmap; bank++)
		gpio->banks[bank].bitmap = bank_desc[bank].bmap;

	gpio->gc.label = ACPI_COMPANION(dev) ?
					acpi_dev_name(ACPI_COMPANION(dev)) : dev_name(dev);
	gpio->gc.parent = dev;
	gpio->gc.owner = THIS_MODULE;
	gpio->gc.get_direction = usbio_gpio_get_direction;
	gpio->gc.direction_input = usbio_gpio_direction_input;
	gpio->gc.direction_output = usbio_gpio_direction_output;
	gpio->gc.get = usbio_gpio_get;
	gpio->gc.set = usbio_gpio_set;
	gpio->gc.set_config = usbio_gpio_set_config;
	gpio->gc.base = -1;
	gpio->gc.ngpio = bank * USBIO_GPIOSPERBANK;
	gpio->gc.can_sleep = true;

	auxiliary_set_drvdata(adev, gpio);

	ret = gpiochip_add_data(&gpio->gc, gpio);
	if (ret)
		return ret;

	if (has_acpi_companion(dev))
		acpi_dev_clear_dependencies(ACPI_COMPANION(dev));

	return 0;
}

static void usbio_gpio_remove(struct auxiliary_device *adev)
{
	struct usbio_gpio *gpio = auxiliary_get_drvdata(adev);

	gpiochip_remove(&gpio->gc);
}

static const struct auxiliary_device_id usbio_gpio_id_table[] = {
	{ "usbio.usbio-gpio" },
	{ }
};
MODULE_DEVICE_TABLE(auxiliary, usbio_gpio_id_table);

static struct auxiliary_driver usbio_gpio_driver = {
	.name = USBIO_GPIO_CLIENT,
	.probe = usbio_gpio_probe,
	.remove = usbio_gpio_remove,
	.id_table = usbio_gpio_id_table
};
module_auxiliary_driver(usbio_gpio_driver);

MODULE_DESCRIPTION("Intel USBIO GPIO driver");
MODULE_AUTHOR("Israel Cepeda <israel.a.cepeda.lopez@intel.com>");
MODULE_AUTHOR("Hans de Goede <hansg@kernel.org>");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS("USBIO");
