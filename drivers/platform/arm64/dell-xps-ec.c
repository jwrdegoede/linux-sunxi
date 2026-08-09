// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2026, Aleksandrs Vinarskis <alex@vinarskis.com>
 */

#include <linux/array_size.h>
#include <linux/dev_printk.h>
#include <linux/device.h>
#include <linux/devm-helpers.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/iio/consumer.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/overflow.h>
#include <linux/pm.h>
#include <linux/types.h>
#include <linux/unaligned.h>
#include <linux/workqueue.h>

#define DELL_XPS_EC_SUSPEND_CMD		0xb9
#define DELL_XPS_EC_SUSPEND_MSG_LEN	64

#define DELL_XPS_EC_TEMP_CMD0		0xfb
#define DELL_XPS_EC_TEMP_CMD1		0x20
#define DELL_XPS_EC_TEMP_CMD3		0x02
#define DELL_XPS_EC_TEMP_MSG_LEN	6
#define DELL_XPS_EC_TEMP_POLL_JIFFIES	msecs_to_jiffies(100)

/*
 * Mapping between IIO channel name (as per schematics) and EC command byte
 */
struct dell_xps_ec_therm {
	const char *name;
	u8 cmd;
};

struct dell_xps_ec_data {
	const struct dell_xps_ec_therm *therms;
	unsigned int num_therms;
	const u8 *profile;
	size_t profile_len;
};

/*
 * Format:
 * - header/unknown (2 bytes)
 * - per-thermistor entries (3 bytes): thermistor_id, param1, param2
 */
static const u8 dell_xps13_9345_thermistor_profile[] = {
	0xff, 0x54,
	0x01, 0x00, 0x2b,	/* sys_therm0 */
	0x02, 0x44, 0x2a,	/* sys_therm1 */
	0x03, 0x44, 0x2b,	/* sys_therm2 */
	0x04, 0x44, 0x28,	/* sys_therm3 */
	0x05, 0x55, 0x2a,	/* sys_therm4 */
	0x06, 0x44, 0x26,	/* sys_therm5 */
	0x07, 0x44, 0x2b,	/* sys_therm6 */
};

static const struct dell_xps_ec_therm dell_xps13_9345_therms[] = {
	/* TODO: 0x01 is sent only occasionally, likely TZ98 or TZ4 */
	{ "sys_therm0", 0x02 },
	{ "sys_therm1", 0x03 },
	{ "sys_therm2", 0x04 },
	{ "sys_therm3", 0x05 },
	{ "sys_therm4", 0x06 },
	{ "sys_therm5", 0x07 },
	{ "sys_therm6", 0x08 },
};

static const struct dell_xps_ec_data dell_xps13_9345_data = {
	.therms = dell_xps13_9345_therms,
	.num_therms = ARRAY_SIZE(dell_xps13_9345_therms),
	.profile = dell_xps13_9345_thermistor_profile,
	.profile_len = sizeof(dell_xps13_9345_thermistor_profile),
};

struct dell_xps_ec {
	struct device *dev;
	struct i2c_client *client;
	const struct dell_xps_ec_data *data;
	struct delayed_work temp_work;
	unsigned int num_therms;
	struct iio_channel *therm_channels[] __counted_by(num_therms);
};

static int dell_xps_ec_suspend_cmd(struct dell_xps_ec *ec, bool suspend)
{
	u8 buf[DELL_XPS_EC_SUSPEND_MSG_LEN] = {};
	int ret;

	buf[0] = DELL_XPS_EC_SUSPEND_CMD;
	buf[1] = suspend ? 0x01 : 0x00;
	/* bytes 2..63 remain zero */

	ret = i2c_master_send(ec->client, buf, sizeof(buf));
	if (ret < 0)
		return ret;

	return 0;
}

static int dell_xps_ec_send_temp(struct dell_xps_ec *ec, u8 cmd_byte,
				 int milli_celsius)
{
	u8 buf[DELL_XPS_EC_TEMP_MSG_LEN];
	u16 deci_celsius;
	int ret;

	/* Convert millicelsius to decicelsius */
	deci_celsius = milli_celsius / 100;

	buf[0] = DELL_XPS_EC_TEMP_CMD0;
	buf[1] = DELL_XPS_EC_TEMP_CMD1;
	buf[2] = cmd_byte;
	buf[3] = DELL_XPS_EC_TEMP_CMD3;
	put_unaligned_le16(deci_celsius, &buf[4]);

	ret = i2c_master_send(ec->client, buf, sizeof(buf));
	if (ret < 0)
		return ret;

	return 0;
}

static void dell_xps_ec_temp_work_fn(struct work_struct *work)
{
	struct dell_xps_ec *ec = container_of(work, struct dell_xps_ec,
					      temp_work.work);
	const struct dell_xps_ec_therm *therms = ec->data->therms;
	int val, ret;
	unsigned int i;

	for (i = 0; i < ec->num_therms; i++) {
		if (!ec->therm_channels[i])
			continue;

		ret = iio_read_channel_processed(ec->therm_channels[i], &val);
		if (ret < 0) {
			dev_err_ratelimited(ec->dev,
					    "Failed to read thermistor %s: %d\n",
					    therms[i].name, ret);
			continue;
		}

		ret = dell_xps_ec_send_temp(ec, therms[i].cmd, val);
		if (ret < 0) {
			dev_err_ratelimited(ec->dev,
					    "Failed to send temp for %s: %d\n",
					    therms[i].name, ret);
		}
	}

	schedule_delayed_work(&ec->temp_work, DELL_XPS_EC_TEMP_POLL_JIFFIES);
}

static irqreturn_t dell_xps_ec_irq_handler(int irq, void *data)
{
	struct dell_xps_ec *ec = data;

	/*
	 * TODO: IRQ is fired on lid-close. Follow Windows example to read out
	 *       the thermistor thresholds and potentially fan speeds.
	 */
	dev_info_ratelimited(ec->dev, "IRQ triggered! (irq=%d)\n", irq);

	return IRQ_HANDLED;
}

static int dell_xps_ec_probe(struct i2c_client *client)
{
	const struct dell_xps_ec_data *data;
	struct device *dev = &client->dev;
	struct dell_xps_ec *ec;
	unsigned int i;
	int ret;

	data = i2c_get_match_data(client);
	if (!data)
		return dev_err_probe(dev, -ENODEV, "No match data\n");

	ec = devm_kzalloc(dev, struct_size(ec, therm_channels, data->num_therms),
			  GFP_KERNEL);
	if (!ec)
		return -ENOMEM;

	ec->dev = dev;
	ec->client = client;
	ec->data = data;
	ec->num_therms = data->num_therms;
	i2c_set_clientdata(client, ec);

	/* Set default thermistor profile */
	ret = i2c_master_send(client, data->profile, data->profile_len);
	if (ret < 0)
		return dev_err_probe(dev, ret, "Failed to set thermistor profile\n");

	/* Get IIO channels for thermistors */
	for (i = 0; i < ec->num_therms; i++) {
		ec->therm_channels[i] =
			devm_iio_channel_get(dev, data->therms[i].name);
		if (IS_ERR(ec->therm_channels[i])) {
			ret = PTR_ERR(ec->therm_channels[i]);
			ec->therm_channels[i] = NULL;
			if (ret == -EPROBE_DEFER)
				return ret;
			dev_warn(dev, "Thermistor %s not available: %d\n",
				 data->therms[i].name, ret);
		}
	}

	/* Start periodic temperature reporting */
	ret = devm_delayed_work_autocancel(dev, &ec->temp_work,
					   dell_xps_ec_temp_work_fn);
	if (ret)
		return ret;
	schedule_delayed_work(&ec->temp_work, DELL_XPS_EC_TEMP_POLL_JIFFIES);
	dev_dbg(dev, "Started periodic temperature reporting to EC every %d ms\n",
		jiffies_to_msecs(DELL_XPS_EC_TEMP_POLL_JIFFIES));

	/* Request IRQ for EC events */
	ret = devm_request_threaded_irq(dev, client->irq, NULL,
					dell_xps_ec_irq_handler,
					IRQF_ONESHOT, dev_name(dev), ec);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to request IRQ\n");

	return 0;
}

/*
 * Notify EC of suspend
 *
 * This will:
 * - Cut power to display/trackpad/keyboard/touchrow, wake-up source still works
 */
static int dell_xps_ec_suspend(struct device *dev)
{
	struct dell_xps_ec *ec = dev_get_drvdata(dev);

	cancel_delayed_work_sync(&ec->temp_work);

	return dell_xps_ec_suspend_cmd(ec, true);
}

/*
 * Notify EC of resume
 *
 * This will undo the suspend actions
 * Without the resume signal, device would wake up but be forced back into
 * suspend by EC within seconds
 */
static int dell_xps_ec_resume(struct device *dev)
{
	struct dell_xps_ec *ec = dev_get_drvdata(dev);
	int ret;

	ret = dell_xps_ec_suspend_cmd(ec, false);
	if (ret)
		return ret;

	schedule_delayed_work(&ec->temp_work, DELL_XPS_EC_TEMP_POLL_JIFFIES);

	return 0;
}

static const struct of_device_id dell_xps_ec_of_match[] = {
	{ .compatible = "dell,xps13-9345-ec", .data = &dell_xps13_9345_data },
	{}
};
MODULE_DEVICE_TABLE(of, dell_xps_ec_of_match);

static const struct i2c_device_id dell_xps_ec_i2c_id[] = {
	{ "dell-xps-ec", (kernel_ulong_t)&dell_xps13_9345_data },
	{}
};
MODULE_DEVICE_TABLE(i2c, dell_xps_ec_i2c_id);

static const struct dev_pm_ops dell_xps_ec_pm_ops = {
	SYSTEM_SLEEP_PM_OPS(dell_xps_ec_suspend, dell_xps_ec_resume)
};

static struct i2c_driver dell_xps_ec_driver = {
	.driver = {
		.name = "dell-xps-ec",
		.of_match_table = dell_xps_ec_of_match,
		.pm = &dell_xps_ec_pm_ops,
	},
	.probe = dell_xps_ec_probe,
	.id_table = dell_xps_ec_i2c_id,
};
module_i2c_driver(dell_xps_ec_driver);

MODULE_AUTHOR("Aleksandrs Vinarskis <alex@vinarskis.com>");
MODULE_DESCRIPTION("Dell XPS 13 9345 Embedded Controller");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS("IIO_CONSUMER");
