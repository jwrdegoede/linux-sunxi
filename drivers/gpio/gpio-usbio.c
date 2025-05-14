// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2025 Intel Corporation.
 *
 */

#include <linux/acpi.h>
#include <linux/gpio/driver.h>
#include <linux/usbio.h>

struct usbio_gpio {
	int gpio_banks;
	struct usbio_gpio_bank banks[USBIO_MAX_GPIOBANKS];
	struct gpio_chip gc;
	struct mutex mutex;
	struct usbio_client *client;
};

static const struct acpi_device_id usbio_gpio_acpi_hids[] = {
	{ "INTC1007" }, /* MTL */
	{ "INTC10B2" }, /* ARL */
	{ "INTC10B5" }, /* LNL */
	{ "INTC10E2" }, /* PTL */
	{ }
};

static int usbio_gpio_get_direction(struct gpio_chip *gc, unsigned int offset)
{
	struct usbio_gpio *gpio = gpiochip_get_data(gc);
	struct usbio_gpio_bank *bank;
	int pin;
	u8 cfg;

	if (!gpio || (offset >= gc->ngpio))
		return -EINVAL;

	bank = &gpio->banks[offset / USBIO_GPIOSPERBANK];
	pin = offset % USBIO_GPIOSPERBANK;
	if (~bank->bitmap & BIT(pin))
		return -EINVAL;

	cfg = bank->config[pin] & USBIO_GPIO_PINMOD_MASK;
	if (cfg == USBIO_GPIO_PINMOD_INPUT)
		return GPIO_LINE_DIRECTION_IN;
	else if (cfg == USBIO_GPIO_PINMOD_OUTPUT)
		return GPIO_LINE_DIRECTION_OUT;

	return GPIO_LINE_DIRECTION_IN;
}

static int usbio_gpio_direction_input(struct gpio_chip *gc,
		unsigned int offset)
{
	struct usbio_gpio *gpio = gpiochip_get_data(gc);
	struct usbio_gpio_bank *bank;
	struct usbio_gpio_init gbuf;
	int pin, ret;

	if (!gpio || (offset >= gc->ngpio))
		return -EINVAL;

	bank = &gpio->banks[offset / USBIO_GPIOSPERBANK];
	pin = offset % USBIO_GPIOSPERBANK;
	if (~bank->bitmap & BIT(pin))
		return -EINVAL;

	bank->config[pin] |= USBIO_GPIO_SET_PINMOD(USBIO_GPIO_PINMOD_INPUT);

	mutex_lock(&gpio->mutex);
	gbuf.bankid = offset / USBIO_GPIOSPERBANK;
	gbuf.config = bank->config[pin];
	gbuf.pincount  = 1;
	gbuf.pin = pin;
	ret = usbio_transfer(gpio->client, USBIO_GPIOCMD_INIT,
							&gbuf, sizeof(gbuf), NULL, 0);
	mutex_unlock(&gpio->mutex);

	return ret;
}

static int usbio_gpio_direction_output(struct gpio_chip *gc,
		unsigned int offset, int value)
{
	struct usbio_gpio *gpio = gpiochip_get_data(gc);
	struct usbio_gpio_bank *bank;
	struct usbio_gpio_init gbuf;
	int pin, ret;

	if (!gpio || (offset >= gc->ngpio))
		return -EINVAL;

	bank = &gpio->banks[offset / USBIO_GPIOSPERBANK];
	pin = offset % USBIO_GPIOSPERBANK;
	if (~bank->bitmap & BIT(pin))
		return -EINVAL;

	bank->config[pin] |= USBIO_GPIO_SET_PINMOD(USBIO_GPIO_PINMOD_OUTPUT);

	mutex_lock(&gpio->mutex);
	gbuf.bankid = offset / USBIO_GPIOSPERBANK;
	gbuf.config = bank->config[pin];
	gbuf.pincount  = 1;
	gbuf.pin = pin;
	ret = usbio_transfer(gpio->client, USBIO_GPIOCMD_INIT,
							&gbuf, sizeof(gbuf), NULL, 0);
	mutex_unlock(&gpio->mutex);

	return ret;
}

static int usbio_gpio_get(struct gpio_chip *gc, unsigned int offset)
{
	struct usbio_gpio *gpio = gpiochip_get_data(gc);
	struct usbio_gpio_bank *bank;
	struct usbio_gpio_rw gbuf;
	int pin, ret;

	if (!gpio || (offset >= gc->ngpio))
		return -EINVAL;

	bank = &gpio->banks[offset / USBIO_GPIOSPERBANK];
	pin = offset % USBIO_GPIOSPERBANK;
	if (~bank->bitmap & BIT(pin))
		return -EINVAL;

	mutex_lock(&gpio->mutex);
	gbuf.bankid = offset / USBIO_GPIOSPERBANK;
	gbuf.pincount  = 1;
	gbuf.pin = pin;
	ret = usbio_transfer(gpio->client, USBIO_GPIOCMD_READ,
							&gbuf, sizeof(gbuf) - sizeof(gbuf.value),
							&gbuf, sizeof(gbuf));
	ret = ret == sizeof(gbuf.value) ? (gbuf.value >> pin) & 1 : -EINVAL;
	mutex_unlock(&gpio->mutex);

	return ret;
}

static void usbio_gpio_set(struct gpio_chip *gc, unsigned int offset,
		int value)
{
	struct usbio_gpio *gpio = gpiochip_get_data(gc);
	struct usbio_gpio_bank *bank;
	struct usbio_gpio_rw gbuf;
	int pin;

	if (!gpio || (offset >= gc->ngpio))
		return;

	bank = &gpio->banks[offset / USBIO_GPIOSPERBANK];
	pin = offset % USBIO_GPIOSPERBANK;
	if (~bank->bitmap & BIT(pin))
		return;

	mutex_lock(&gpio->mutex);
	gbuf.bankid = offset / USBIO_GPIOSPERBANK;
	gbuf.pincount  = 1;
	gbuf.pin = pin;
	gbuf.value = value << pin;
	usbio_transfer(gpio->client, USBIO_GPIOCMD_WRITE,
					&gbuf, sizeof(gbuf), NULL, 0);
	mutex_unlock(&gpio->mutex);
}

static int usbio_gpio_set_config(struct gpio_chip *gc, unsigned int offset,
		unsigned long config)
{
	struct usbio_gpio *gpio = gpiochip_get_data(gc);
	struct usbio_gpio_bank *bank;
	int pin;

	if (!gpio || (offset >= gc->ngpio))
		return -EINVAL;

	bank = &gpio->banks[offset / USBIO_GPIOSPERBANK];
	pin = offset % USBIO_GPIOSPERBANK;
	if (~bank->bitmap & BIT(pin))
		return -EINVAL;

	bank->config[pin] = USBIO_GPIO_SET_PINCFG(USBIO_GPIO_PINCFG_DEFAULT);
	switch (pinconf_to_config_param(config)) {
	case PIN_CONFIG_BIAS_PULL_PIN_DEFAULT:
		break;
	case PIN_CONFIG_BIAS_PULL_UP:
		bank->config[pin] |= USBIO_GPIO_SET_PINCFG(USBIO_GPIO_PINCFG_PULLUP);
		break;
	case PIN_CONFIG_BIAS_PULL_DOWN:
		bank->config[pin] |= USBIO_GPIO_SET_PINCFG(USBIO_GPIO_PINCFG_PULLDOWN);
		break;
	case PIN_CONFIG_DRIVE_PUSH_PULL:
		bank->config[pin] |= USBIO_GPIO_SET_PINCFG(USBIO_GPIO_PINCFG_PUSHPULL);
		break;
	default:
		return -ENOTSUPP;
	}

	return 0;
}

static int usbio_gpio_probe(struct auxiliary_device *adev,
		const struct auxiliary_device_id *adev_id)
{
	struct device *dev = &adev->dev;
	struct usbio_gpio *gpio;
	int ret;

	usbio_client_acpi_bind(adev, usbio_gpio_acpi_hids);

	gpio = devm_kzalloc(dev, sizeof(*gpio), GFP_KERNEL);
	if (!gpio)
		return -ENOMEM;

	gpio->client = auxiliary_get_usbio_client(adev);
	gpio->gpio_banks = usbio_gpio_init(gpio->client, gpio->banks,
										sizeof(gpio->banks));
	if (gpio->gpio_banks < 0) {
		devm_kfree(dev, gpio);
		return -EINVAL;
	}

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
	gpio->gc.ngpio = gpio->gpio_banks * USBIO_GPIOSPERBANK;
	gpio->gc.can_sleep = true;
	mutex_init(&gpio->mutex);

	auxiliary_set_drvdata(adev, gpio);

	ret = gpiochip_add_data(&gpio->gc, gpio);
	if (ret) {
		dev_err(dev, "Failed to add gpiochip %d", ret);
		devm_kfree(dev, gpio);
		return ret;
	}

	if (has_acpi_companion(dev))
		acpi_dev_clear_dependencies(ACPI_COMPANION(dev));

	return 0;
}

static void usbio_gpio_remove(struct auxiliary_device *adev)
{
	struct device *dev = &adev->dev;
	struct usbio_gpio *gpio = auxiliary_get_drvdata(adev);

	gpiochip_remove(&gpio->gc);

	mutex_destroy(&gpio->mutex);
	devm_kfree(dev, gpio);
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
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS("USBIO");
