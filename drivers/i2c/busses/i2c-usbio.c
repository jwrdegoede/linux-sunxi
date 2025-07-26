// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2025 Intel Corporation.
 * Copyright (c) 2025 Red Hat, Inc.
 */

#include <linux/auxiliary_bus.h>
#include <linux/dev_printk.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/types.h>
#include <linux/usb/usbio.h>

#define I2C_RW_OVERHEAD (sizeof(struct usbio_bulk_packet) + sizeof(struct usbio_i2c_rw))

struct usbio_i2c {
	u32 speed;
	struct i2c_adapter adap;
	struct auxiliary_device *adev;
	struct usbio_i2c_rw *rwbuf;
	bool init_supports_ack_flag;
	u16 txbuf_len;
	u16 rxbuf_len;
};

static const struct acpi_device_id usbio_i2c_acpi_hids[] = {
	{ "INTC1008" }, /* MTL */
	{ "INTC10B3" }, /* ARL */
	{ "INTC10B6" }, /* LNL */
	{ "INTC10E3" }, /* PTL */
	{ }
};

static const u32 usbio_i2c_speeds[] = {
	I2C_MAX_STANDARD_MODE_FREQ,
	I2C_MAX_FAST_MODE_FREQ,
	I2C_MAX_FAST_MODE_PLUS_FREQ,
	I2C_MAX_HIGH_SPEED_MODE_FREQ
};

static void usbio_i2c_uninit(struct i2c_adapter *adap, struct i2c_msg *msg)
{
	struct usbio_i2c *i2c = i2c_get_adapdata(adap);
	struct usbio_i2c_uninit ubuf;

	ubuf.busid = i2c->adev->id;
	ubuf.config = msg->addr;

	usbio_bulk_msg(i2c->adev, USBIO_PKTTYPE_I2C, USBIO_I2CCMD_UNINIT, true,
		       &ubuf, sizeof(ubuf), NULL, 0);
}

static int usbio_i2c_init(struct i2c_adapter *adap, struct i2c_msg *msg)
{
	struct usbio_i2c *i2c = i2c_get_adapdata(adap);
	struct usbio_i2c_init ibuf;
	void *reply_buf;
	u16 reply_len;
	int ret;

	ibuf.busid = i2c->adev->id;
	ibuf.config = msg->addr;
	ibuf.speed = i2c->speed;

	if (i2c->init_supports_ack_flag) {
		reply_buf = &ibuf;
		reply_len = sizeof(ibuf);
	} else {
		reply_buf = NULL;
		reply_len = 0;
	}
	ret = usbio_bulk_msg(i2c->adev, USBIO_PKTTYPE_I2C, USBIO_I2CCMD_INIT, true,
			     &ibuf, sizeof(ibuf), reply_buf, reply_len);
	if (ret != sizeof(ibuf))
		return (ret < 0) ? ret : -EIO;

	return 0;
}

static int usbio_i2c_read(struct i2c_adapter *adap, struct i2c_msg *msg)
{
	struct usbio_i2c *i2c = i2c_get_adapdata(adap);
	u16 rxchunk = i2c->rxbuf_len - I2C_RW_OVERHEAD;
	struct usbio_i2c_rw *rbuf = i2c->rwbuf;
	int ret;

	rbuf->busid = i2c->adev->id;
	rbuf->config = msg->addr;
	rbuf->size = msg->len;

	if (msg->len > rxchunk) {
		/* Need to split the input buffer */
		u16 len = 0;

		do {
			if (msg->len - len < rxchunk)
				rxchunk = msg->len - len;

			ret = usbio_bulk_msg(i2c->adev, USBIO_PKTTYPE_I2C,
					     USBIO_I2CCMD_READ, true,
					     rbuf, len == 0 ? sizeof(*rbuf) : 0,
					     rbuf, sizeof(*rbuf) + rxchunk);
			if (ret < 0)
				return ret;

			memcpy(&msg->buf[len], rbuf->data, rxchunk);
			len += rxchunk;
		} while (msg->len > len);

		goto out_log;
	}

	ret = usbio_bulk_msg(i2c->adev, USBIO_PKTTYPE_I2C, USBIO_I2CCMD_READ, true,
			     rbuf, sizeof(*rbuf), rbuf, sizeof(*rbuf) + msg->len);
	if (ret != sizeof(*rbuf) + msg->len)
		return (ret < 0) ? ret : -EIO;

	memcpy(msg->buf, rbuf->data, msg->len);
out_log:
	dev_dbg(adap->dev.parent, "RD[%d]:%*phN\n", msg->len, msg->len, msg->buf);
	return 0;
}

static int usbio_i2c_write(struct i2c_adapter *adap, struct i2c_msg *msg)
{
	struct usbio_i2c *i2c = i2c_get_adapdata(adap);
	u16 txchunk = i2c->txbuf_len - I2C_RW_OVERHEAD;
	struct usbio_i2c_rw *wbuf = i2c->rwbuf;
	int ret;

	dev_dbg(adap->dev.parent, "WR[%d]:%*phN\n", msg->len, msg->len, msg->buf);

	if (msg->len > txchunk) {
		/* Need to split the output buffer */
		u16 len = 0;

		do {
			wbuf->busid = i2c->adev->id;
			wbuf->config = msg->addr;
			wbuf->size = msg->len;

			memcpy(wbuf->data, &msg->buf[len], txchunk);
			len += txchunk;

			ret = usbio_bulk_msg(i2c->adev, USBIO_PKTTYPE_I2C,
					     USBIO_I2CCMD_WRITE, msg->len == len,
					     wbuf, sizeof(*wbuf) + txchunk,
					     wbuf, sizeof(*wbuf));
			if (ret < 0)
				return ret;

			if (msg->len - len < txchunk)
				txchunk = msg->len - len;
		} while (msg->len > len);

		return 0;
	}

	wbuf->busid = i2c->adev->id;
	wbuf->config = msg->addr;
	wbuf->size = msg->len;
	memcpy(wbuf->data, msg->buf, msg->len);

	ret = usbio_bulk_msg(i2c->adev, USBIO_PKTTYPE_I2C, USBIO_I2CCMD_WRITE, true,
			     wbuf, sizeof(*wbuf) + msg->len, wbuf, sizeof(*wbuf));
	if (ret != sizeof(*wbuf) || wbuf->size != msg->len)
		return (ret < 0) ? ret : -EIO;

	return 0;
}

static int usbio_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg *msgs, int num)
{
	struct usbio_i2c *i2c = i2c_get_adapdata(adap);
	int ret;

	usbio_acquire(i2c->adev);

	ret = usbio_i2c_init(adap, msgs);
	if (ret)
		goto out_release;

	for (int i = 0; i < num; ret = ++i) {
		if (msgs[i].flags & I2C_M_RD)
			ret = usbio_i2c_read(adap, &msgs[i]);
		else
			ret = usbio_i2c_write(adap, &msgs[i]);

		if (ret)
			break;
	}

	usbio_i2c_uninit(adap, msgs);

out_release:
	usbio_release(i2c->adev);
	return ret;
}

static u32 usbio_i2c_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_10BIT_ADDR | I2C_FUNC_SMBUS_EMUL;
}

static const struct i2c_adapter_quirks usbio_i2c_quirks = {
	.flags = I2C_AQ_NO_REP_START,
	.max_read_len = SZ_4K,
	.max_write_len = SZ_4K,
};

/* Use smaller max_*_len settings for chips which do not support split bulk transfers */
static const struct i2c_adapter_quirks usbio_i2c_no_split_transfers_quirks = {
	.flags = I2C_AQ_NO_REP_START,
	.max_read_len = 63 - I2C_RW_OVERHEAD,
	.max_write_len = 63 -  I2C_RW_OVERHEAD,
};

static const struct i2c_algorithm usbio_i2c_algo = {
	.master_xfer = usbio_i2c_xfer,
	.functionality = usbio_i2c_func,
};

static int usbio_i2c_probe(struct auxiliary_device *adev,
		const struct auxiliary_device_id *adev_id)
{
	struct usbio_i2c_bus_desc *i2c_desc;
	struct device *dev = &adev->dev;
	u8 dummy_read_buf;
	struct i2c_msg dummy_read = {
		.addr = 0x08,
		.flags = I2C_M_RD,
		.len = 1,
		.buf = &dummy_read_buf,
	};
	struct usbio_i2c *i2c;
	u32 max_speed;
	int ret;

	i2c_desc = dev_get_platdata(dev);
	if (!i2c_desc)
		return -EINVAL;

	/* Some USBIO chips have caps set to 0, but all chips can do 400KHz */
	if (!i2c_desc->caps)
		max_speed = I2C_MAX_FAST_MODE_FREQ;
	else
		max_speed = usbio_i2c_speeds[i2c_desc->caps & USBIO_I2C_BUS_MODE_CAP_MASK];

	i2c = devm_kzalloc(dev, sizeof(*i2c), GFP_KERNEL);
	if (!i2c)
		return -ENOMEM;

	i2c->adev = adev;

	usbio_acpi_bind(i2c->adev, usbio_i2c_acpi_hids);
	usbio_get_txrxbuf_len(i2c->adev, &i2c->txbuf_len, &i2c->rxbuf_len);

	i2c->rwbuf = devm_kzalloc(dev, max(i2c->txbuf_len, i2c->rxbuf_len), GFP_KERNEL);
	if (!i2c->rwbuf)
		return -ENOMEM;

	i2c->speed = i2c_acpi_find_bus_speed(dev);
	if (!i2c->speed)
		i2c->speed = I2C_MAX_STANDARD_MODE_FREQ;
	else if (i2c->speed > max_speed) {
		dev_warn(dev, "Invalid speed %u adjusting to bus max %u\n",
			 i2c->speed, max_speed);
		i2c->speed = max_speed;
	}

	i2c->adap.owner = THIS_MODULE;
	i2c->adap.class = I2C_CLASS_HWMON;
	i2c->adap.dev.parent = dev;
	i2c->adap.algo = &usbio_i2c_algo;
	i2c->adap.quirks = &usbio_i2c_quirks;

	snprintf(i2c->adap.name, sizeof(i2c->adap.name), "%s.%d",
		 USBIO_I2C_CLIENT, i2c->adev->id);

	device_set_node(&i2c->adap.dev, dev_fwnode(&adev->dev));

	auxiliary_set_drvdata(adev, i2c);
	i2c_set_adapdata(&i2c->adap, i2c);

	/*
	 * Test if USBIO_I2CCMD_INIT commands with the USBIO_PKTFLAG_ACK flag
	 * are supported.
	 */
	usbio_acquire(i2c->adev);
	i2c->init_supports_ack_flag = true;
	ret = usbio_i2c_init(&i2c->adap, &dummy_read);
	if (ret) {
		if (ret != -EPIPE) {
			usbio_release(i2c->adev);
			return ret; /* Unexpected error */
		}

		dev_info(dev, "i2c-init command does not support ack flag\n");
		i2c->init_supports_ack_flag = false;
		/* Chips with this quirk also do not support split bulk transfers */
		i2c->adap.quirks = &usbio_i2c_no_split_transfers_quirks;
	} else {
		/* Continue dummy read to not confuse the USBIO chip */
		usbio_i2c_read(&i2c->adap, &dummy_read);
	}
	usbio_i2c_uninit(&i2c->adap, &dummy_read);
	usbio_release(i2c->adev);

	ret = i2c_add_adapter(&i2c->adap);
	if (ret)
		return ret;

	if (has_acpi_companion(&i2c->adap.dev))
		acpi_dev_clear_dependencies(ACPI_COMPANION(&i2c->adap.dev));

	return 0;
}

static void usbio_i2c_remove(struct auxiliary_device *adev)
{
	struct usbio_i2c *i2c = auxiliary_get_drvdata(adev);

	i2c_del_adapter(&i2c->adap);
}

static const struct auxiliary_device_id usbio_i2c_id_table[] = {
	{ "usbio.usbio-i2c" },
	{ }
};
MODULE_DEVICE_TABLE(auxiliary, usbio_i2c_id_table);

static struct auxiliary_driver usbio_i2c_driver = {
	.name = USBIO_I2C_CLIENT,
	.probe = usbio_i2c_probe,
	.remove = usbio_i2c_remove,
	.id_table = usbio_i2c_id_table
};
module_auxiliary_driver(usbio_i2c_driver);

MODULE_DESCRIPTION("Intel USBIO I2C driver");
MODULE_AUTHOR("Israel Cepeda <israel.a.cepeda.lopez@intel.com>");
MODULE_AUTHOR("Hans de Goede <hansg@kernel.org>");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS("USBIO");
