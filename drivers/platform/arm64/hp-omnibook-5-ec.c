// SPDX-License-Identifier: GPL-2.0-only
/*
 * HP OmniBook 5 16 (Snapdragon X2) embedded controller
 *
 * The EC takes commands through a mailbox: wait for an idle status, write
 * the command, wait for it to complete, then read the reply. Command codes
 * are the ones HP's ACPI tables use on this laptop.
 */

#include <linux/cleanup.h>
#include <linux/delay.h>
#include <linux/hwmon.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/sparse-keymap.h>
#include <linux/interrupt.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/unaligned.h>

/* The pending event, cleared by writing 0 (HP's IC10.INTR, field CMD5) */
#define HPEC_EVENT		0x00
#define HPEC_STATUS		0xef
#define HPEC_REQUEST		0xed
#define HPEC_REPLY		0xee
#define HPEC_REPLY_LEN		32

#define HPEC_STATUS_IDLE	0x00
#define HPEC_STATUS_DONE	0xe0
#define HPEC_POLL_TRIES		20
#define HPEC_POLL_US		5000

#define HPEC_OS_FLAG		0x01
#define HPEC_OS_FLAG_LEDS	0x84
#define HPEC_KBD		0x05
#define HPEC_KBD_TIMEOUT_GET	0xfe
#define HPEC_KBD_LEVEL_GET	0xff
#define HPEC_LED		0x06
#define HPEC_LED_GET		0xff
#define HPEC_LED_ON		0x80
#define HPEC_THERMISTOR		0x26
#define HPEC_THERMISTOR_INVALID	0xff
#define HPEC_FAN		0xbc
#define HPEC_FAN_RPM		0x03

/* The four thermistors HP's thermal zones enable, in their order */
static const u8 hpec_thermistors[] = { 0x00, 0x07, 0x04, 0x0a };

static const char * const hpec_thermistor_labels[] = {
	"EC thermistor 1", "EC thermistor 2", "EC thermistor 3", "EC thermistor 4",
};

/* EC events, with HP's WMI hotkey codes from hp-wmi where they exist */
#define HPEC_EVENT_HOT		0x0b
#define HPEC_EVENT_KBD		0x0d
#define HPEC_EVENT_KBD_KEY	0x1d

static const struct key_entry hpec_keymap[] = {
	{ KE_IGNORE, 0x0d, },			/* backlight key, run by the EC */
	{ KE_KEY, 0x11, { KEY_MICMUTE } },	/* F9, WMI 0x270 */
	{ KE_IGNORE, 0x1d, },			/* backlight key, run by the EC */
	{ KE_KEY, 0x1e, { KEY_PROG2 } },	/* F11, WMI 0x21a8 */
	{ KE_IGNORE, 0x30, },			/* fan and thermistor notices */
	{ KE_IGNORE, 0x31, },
	{ KE_IGNORE, 0x32, },
	{ KE_IGNORE, 0x33, },
	{ KE_IGNORE, 0x34, },
	{ KE_IGNORE, 0x35, },
	{ KE_IGNORE, 0x36, },
	{ KE_IGNORE, 0x37, },
	{ KE_IGNORE, 0x38, },
	{ KE_IGNORE, 0x88, },			/* Fn lock, WMI 0x21ab */
	{ KE_END, 0 }
};

/* Keyboard backlight timeout, in EC value order */
static const char * const hpec_kbd_timeouts[] = { "30s", "3min", "always" };

/* EC LEDs 8 and 9, which HP's mute LED WMI methods drive */
static const struct {
	const char *name;
	const char *trigger;
	u8 index;
} hpec_leds[] = {
	{ "platform::mute", "audio-mute", 0x08 },
	{ "platform::micmute", "audio-micmute", 0x09 },
};

struct hpec;

struct hpec_led {
	struct led_classdev cdev;
	struct hpec *ec;
	u8 index;
};

struct hpec {
	struct i2c_client *client;
	struct mutex lock;	/* serializes mailbox commands */
	struct hpec_led leds[ARRAY_SIZE(hpec_leds)];
	struct input_dev *input;
};

static int hpec_wait(struct hpec *ec, u8 status)
{
	int i, ret;

	for (i = 0; i < HPEC_POLL_TRIES; i++) {
		ret = i2c_smbus_read_byte_data(ec->client, HPEC_STATUS);
		if (ret < 0)
			return ret;
		if (ret == status)
			return 0;
		fsleep(HPEC_POLL_US);
	}

	return -ETIMEDOUT;
}

static int hpec_command(struct hpec *ec, u8 cmd, u8 sub, u8 reply[HPEC_REPLY_LEN])
{
	u8 request[] = { cmd, sub };
	int ret;

	guard(mutex)(&ec->lock);

	ret = hpec_wait(ec, HPEC_STATUS_IDLE);
	if (ret)
		return ret;

	ret = i2c_smbus_write_i2c_block_data(ec->client, HPEC_REQUEST,
					     sizeof(request), request);
	if (ret)
		return ret;

	ret = hpec_wait(ec, HPEC_STATUS_DONE);
	if (ret || !reply)
		return ret;

	ret = i2c_smbus_read_i2c_block_data(ec->client, HPEC_REPLY,
					    HPEC_REPLY_LEN, reply);
	if (ret < 0)
		return ret;

	return ret == HPEC_REPLY_LEN ? 0 : -EIO;
}

static int hpec_hwmon_read(struct device *dev, enum hwmon_sensor_types type,
			   u32 attr, int channel, long *val)
{
	struct hpec *ec = dev_get_drvdata(dev);
	u8 reply[HPEC_REPLY_LEN];
	int ret;

	switch (type) {
	case hwmon_fan:
		ret = hpec_command(ec, HPEC_FAN, HPEC_FAN_RPM, reply);
		if (ret)
			return ret;
		*val = get_unaligned_le16(reply);
		return 0;
	case hwmon_temp:
		ret = hpec_command(ec, HPEC_THERMISTOR, hpec_thermistors[channel], reply);
		if (ret)
			return ret;
		if (reply[0] == HPEC_THERMISTOR_INVALID)
			return -ENODATA;
		*val = reply[0] * 1000;
		return 0;
	default:
		return -EOPNOTSUPP;
	}
}

static int hpec_hwmon_read_string(struct device *dev, enum hwmon_sensor_types type,
				  u32 attr, int channel, const char **str)
{
	if (type != hwmon_temp)
		return -EOPNOTSUPP;

	*str = hpec_thermistor_labels[channel];
	return 0;
}

static umode_t hpec_hwmon_is_visible(const void *data, enum hwmon_sensor_types type,
				     u32 attr, int channel)
{
	return 0444;
}

static const struct hwmon_ops hpec_hwmon_ops = {
	.is_visible = hpec_hwmon_is_visible,
	.read = hpec_hwmon_read,
	.read_string = hpec_hwmon_read_string,
};

static const struct hwmon_channel_info * const hpec_hwmon_info[] = {
	HWMON_CHANNEL_INFO(chip, HWMON_C_REGISTER_TZ),
	HWMON_CHANNEL_INFO(fan, HWMON_F_INPUT),
	HWMON_CHANNEL_INFO(temp,
			   HWMON_T_INPUT | HWMON_T_LABEL,
			   HWMON_T_INPUT | HWMON_T_LABEL,
			   HWMON_T_INPUT | HWMON_T_LABEL,
			   HWMON_T_INPUT | HWMON_T_LABEL),
	NULL
};

static const struct hwmon_chip_info hpec_hwmon_chip = {
	.ops = &hpec_hwmon_ops,
	.info = hpec_hwmon_info,
};

static enum led_brightness hpec_led_get(struct led_classdev *cdev)
{
	struct hpec_led *led = container_of(cdev, struct hpec_led, cdev);
	u8 reply[HPEC_REPLY_LEN];

	if (hpec_command(led->ec, HPEC_LED, HPEC_LED_GET, reply))
		return LED_OFF;

	return (reply[1] >> (led->index - hpec_leds[0].index)) & 1;
}

static int hpec_led_set(struct led_classdev *cdev, enum led_brightness value)
{
	struct hpec_led *led = container_of(cdev, struct hpec_led, cdev);
	int ret;

	/* As HP's method does before every change */
	ret = hpec_command(led->ec, HPEC_OS_FLAG, HPEC_OS_FLAG_LEDS, NULL);
	if (ret)
		return ret;

	return hpec_command(led->ec, HPEC_LED,
			    led->index | (value ? HPEC_LED_ON : 0), NULL);
}

static irqreturn_t hpec_irq(int irq, void *data)
{
	struct hpec *ec = data;
	struct device *dev = &ec->client->dev;
	int event;

	scoped_guard(mutex, &ec->lock) {
		event = i2c_smbus_read_byte_data(ec->client, HPEC_EVENT);
		if (event < 0) {
			dev_err_ratelimited(dev, "cannot read EC event: %d\n", event);
			return IRQ_HANDLED;
		}
		i2c_smbus_write_byte_data(ec->client, HPEC_EVENT, 0);
	}

	if (!event)
		return IRQ_HANDLED;

	if (event == HPEC_EVENT_KBD || event == HPEC_EVENT_KBD_KEY)
		sysfs_notify(&dev->kobj, NULL, "kbd_backlight_level");

	if (event == HPEC_EVENT_HOT) {
		dev_crit(dev, "EC reports a critical temperature\n");
		return IRQ_HANDLED;
	}

	if (!sparse_keymap_report_event(ec->input, event, 1, true))
		dev_info_ratelimited(dev, "unknown EC event %#x\n", event);

	return IRQ_HANDLED;
}

static ssize_t kbd_backlight_timeout_show(struct device *dev,
					  struct device_attribute *attr, char *buf)
{
	struct hpec *ec = dev_get_drvdata(dev);
	u8 reply[HPEC_REPLY_LEN];
	int ret;

	ret = hpec_command(ec, HPEC_KBD, HPEC_KBD_TIMEOUT_GET, reply);
	if (ret)
		return ret;

	if (reply[0] >= ARRAY_SIZE(hpec_kbd_timeouts))
		return sysfs_emit(buf, "%#x\n", reply[0]);

	return sysfs_emit(buf, "%s\n", hpec_kbd_timeouts[reply[0]]);
}

static ssize_t kbd_backlight_timeout_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	struct hpec *ec = dev_get_drvdata(dev);
	int i, ret;

	i = sysfs_match_string(hpec_kbd_timeouts, buf);
	if (i < 0)
		return i;

	ret = hpec_command(ec, HPEC_KBD, i, NULL);

	return ret ?: count;
}
static DEVICE_ATTR_RW(kbd_backlight_timeout);

/* The level is set only by the backlight key, inside the EC */
static ssize_t kbd_backlight_level_show(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	struct hpec *ec = dev_get_drvdata(dev);
	u8 reply[HPEC_REPLY_LEN];
	int ret;

	ret = hpec_command(ec, HPEC_KBD, HPEC_KBD_LEVEL_GET, reply);
	if (ret)
		return ret;

	return sysfs_emit(buf, "%u\n", reply[0]);
}
static DEVICE_ATTR_RO(kbd_backlight_level);

static struct attribute *hpec_attrs[] = {
	&dev_attr_kbd_backlight_level.attr,
	&dev_attr_kbd_backlight_timeout.attr,
	NULL
};
ATTRIBUTE_GROUPS(hpec);

static int hpec_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct device *hwmon;
	u8 reply[HPEC_REPLY_LEN];
	struct hpec *ec;
	unsigned int i;
	int ret;

	ec = devm_kzalloc(dev, sizeof(*ec), GFP_KERNEL);
	if (!ec)
		return -ENOMEM;

	ec->client = client;
	i2c_set_clientdata(client, ec);
	ret = devm_mutex_init(dev, &ec->lock);
	if (ret)
		return ret;

	ret = hpec_command(ec, HPEC_FAN, HPEC_FAN_RPM, reply);
	if (ret)
		return dev_err_probe(dev, ret, "EC does not answer\n");

	ec->input = devm_input_allocate_device(dev);
	if (!ec->input)
		return -ENOMEM;

	ec->input->name = "HP OmniBook 5 EC hotkeys";
	ec->input->phys = "hp-omnibook-5-ec/input0";
	ec->input->id.bustype = BUS_I2C;

	ret = sparse_keymap_setup(ec->input, hpec_keymap, NULL);
	if (ret)
		return ret;

	ret = input_register_device(ec->input);
	if (ret)
		return ret;

	hwmon = devm_hwmon_device_register_with_info(dev, "hp_omnibook_ec", ec,
						     &hpec_hwmon_chip, NULL);
	if (IS_ERR(hwmon))
		return PTR_ERR(hwmon);

	for (i = 0; i < ARRAY_SIZE(hpec_leds); i++) {
		struct hpec_led *led = &ec->leds[i];

		led->ec = ec;
		led->index = hpec_leds[i].index;
		led->cdev.name = hpec_leds[i].name;
		led->cdev.default_trigger = hpec_leds[i].trigger;
		led->cdev.max_brightness = 1;
		led->cdev.brightness_get = hpec_led_get;
		led->cdev.brightness_set_blocking = hpec_led_set;

		ret = devm_led_classdev_register(dev, &led->cdev);
		if (ret)
			return ret;
	}

	if (client->irq > 0) {
		ret = devm_request_threaded_irq(dev, client->irq, NULL, hpec_irq,
						IRQF_ONESHOT, "hp-omnibook-5-ec", ec);
		if (ret)
			return dev_err_probe(dev, ret, "cannot request the EC interrupt\n");
	}

	return 0;
}

static const struct of_device_id hpec_of_match[] = {
	{ .compatible = "hp,omnibook-5-16-bf1xxx-ec" },
	{ }
};
MODULE_DEVICE_TABLE(of, hpec_of_match);

static struct i2c_driver hpec_driver = {
	.driver = {
		.name = "hp-omnibook-5-ec",
		.of_match_table = hpec_of_match,
		.dev_groups = hpec_groups,
	},
	.probe = hpec_probe,
};
module_i2c_driver(hpec_driver);

MODULE_DESCRIPTION("HP OmniBook 5 16 embedded controller");
MODULE_LICENSE("GPL");
