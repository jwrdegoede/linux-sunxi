// SPDX-License-Identifier: GPL-2.0
/*
 * Intel USBIO Bridge driver
 *
 * Copyright (c) 2025 Intel Corporation.
 *
 */

#include "usbio.h"

static int usbio_control_msg(struct usbio_device *usbio,
		struct usbio_packet_header *pkt, const void *obuf,
		u16 obuf_len, void *ibuf, u16 ibuf_len)
{
	struct usbio_ctrl_packet *cpkt;
	unsigned int pipe;
	u16 cpkt_len = sizeof(*cpkt) + obuf_len;
	u8 request = USB_TYPE_VENDOR | USB_RECIP_DEVICE;
	int ret;

	if (!usbio || !pkt || (!obuf && obuf_len) || (!ibuf && ibuf_len))
		return -EINVAL;

	if (cpkt_len > usbio->ctrlbuf_len) {
		dev_err(usbio->dev, "Packet size error: %u > %u",
				cpkt_len, usbio->ctrlbuf_len);
		return -EMSGSIZE;
	}

	/* Prepare Control Packet Header */
	cpkt = usbio->ctrlbuf;
	cpkt->header.type = pkt->type;
	cpkt->header.cmd = pkt->cmd;
	cpkt->header.flags = pkt->flags;
	cpkt->len = obuf_len;

	/* Copy the data */
	memcpy(cpkt->data, obuf, obuf_len);

	pipe = usb_sndctrlpipe(usbio->udev, usbio->ctrl_pipe);
	ret = usb_control_msg(usbio->udev, pipe, 0, request | USB_DIR_OUT, 0, 0,
							(void *)cpkt, cpkt_len, USBIO_CTRLXFER_TIMEOUT);
	dev_dbg(usbio->dev, "usb control sent: %d", ret);
	dev_dbg(usbio->dev, "\thdr: %*phN data: %*phN", (int)sizeof(*cpkt),
			cpkt, (int)cpkt->len, cpkt->data);

	if (ret < 0 || ret != cpkt_len)
		return -EIO;

	if (pkt->flags & USBIO_PKTFLAG_ACK) {
		cpkt_len = sizeof(*cpkt) + ibuf_len;

		if (cpkt_len > usbio->ctrlbuf_len) {
			dev_err(usbio->dev, "Packet size error: %u > %u",
					cpkt_len, usbio->ctrlbuf_len);
			return -EMSGSIZE;
		}

		pipe = usb_rcvctrlpipe(usbio->udev, usbio->ctrl_pipe);
		ret = usb_control_msg(usbio->udev, pipe, 0, request | USB_DIR_IN,
								0, 0, (void *)cpkt, cpkt_len,
								USBIO_CTRLXFER_TIMEOUT);
		dev_dbg(usbio->dev, "usb control received: %d", ret);
		dev_dbg(usbio->dev, "\thdr: %*phN data: %*phN", (int)sizeof(*cpkt),
				cpkt, (int)cpkt->len, cpkt->data);

		if (ret < sizeof(*cpkt))
			return -EIO;

		ret = -EINVAL;
		if (cpkt->header.type == pkt->type &&
				cpkt->header.cmd == pkt->cmd &&
				cpkt->header.flags & USBIO_PKTFLAG_RSP) {
			if (!(cpkt->header.flags & USBIO_PKTFLAG_ERR)) {
				if (ibuf_len < cpkt->len)
					return -ENOBUFS;
				/* Copy the data */
				memcpy(ibuf, cpkt->data, cpkt->len);
				ret = cpkt->len;
			} else
				dev_err(usbio->dev,
						"Packet error type: %u, cmd: %u, flags: %u",
						cpkt->header.type, cpkt->header.cmd,
						cpkt->header.flags);
		} else
			dev_err(usbio->dev,
					"Unexpected reply type: %u, cmd: %u, flags: %u",
					cpkt->header.type, cpkt->header.cmd,
					cpkt->header.flags);
	} else
		ret = cpkt_len - sizeof(*cpkt);

	return ret;
}

static void usbio_bulk_recv(struct urb *urb)
{
	struct usbio_bulk_packet *bpkt = urb->transfer_buffer;
	struct usbio_device *usbio = urb->context;

	if (!urb->status) {
		if (bpkt->header.flags & USBIO_PKTFLAG_RSP) {
			usbio->rxdat_len = urb->actual_length;
			complete(&usbio->done);
		}
	} else
		dev_err(usbio->dev, "URB error:%d", urb->status);

	usb_submit_urb(usbio->urb, GFP_ATOMIC);
}

static int usbio_bulk_msg(struct usbio_device *usbio,
		struct usbio_packet_header *pkt, bool last, const void *obuf,
		u16 obuf_len, void *ibuf, u16 ibuf_len, int timeout)
{
	struct usbio_bulk_packet *bpkt;
	u16 bpkt_len = sizeof(*bpkt) + obuf_len;
	int ret, act;

	if (!usbio || !pkt || (!obuf && obuf_len) || (!ibuf && ibuf_len))
		return -EINVAL;

	if (bpkt_len > usbio->txbuf_len) {
		dev_err(usbio->dev, "Packet size error: %u > %u",
				bpkt_len, usbio->txbuf_len);
		return -EMSGSIZE;
	}

	if (!obuf_len)
		goto read;

	/* Prepare Bulk Packet Header */
	bpkt = (struct usbio_bulk_packet *)usbio->txbuf;
	bpkt->header.type = pkt->type;
	bpkt->header.cmd = pkt->cmd;
	bpkt->header.flags = last ? pkt->flags : 0;
	bpkt->len = obuf_len;

	/* Copy the data */
	memcpy(bpkt->data, obuf, obuf_len);

	reinit_completion(&usbio->done);
	ret = usb_bulk_msg(usbio->udev, usbio->tx_pipe,
						(void *)bpkt, bpkt_len, &act, timeout);
	dev_dbg(usbio->dev, "usb bulk sent: %u", act);
	dev_dbg(usbio->dev, "\thdr: %*phN data: %*phN", (int)sizeof(*bpkt), bpkt,
				(int)bpkt->len, bpkt->data);

	if (ret || act != bpkt_len)
		return -EIO;

read:
	if (last && pkt->flags & USBIO_PKTFLAG_ACK) {
		bpkt_len = sizeof(*bpkt) + ibuf_len;

		if (bpkt_len > usbio->txbuf_len) {
			dev_err(usbio->dev, "Packet size error: %u > %u",
					bpkt_len, usbio->txbuf_len);
			return -EMSGSIZE;
		}

		ret = wait_for_completion_timeout(&usbio->done, timeout);
		if (!ret)
			return -ETIMEDOUT;

		act = usbio->rxdat_len;
		bpkt = (struct usbio_bulk_packet *)usbio->rxbuf;
		dev_dbg(usbio->dev, "usb bulk received: %u", act);
		dev_dbg(usbio->dev, "\thdr: %*phN data: %*phN", (int)sizeof(*bpkt),
				bpkt, (int)bpkt->len, bpkt->data);

		if (act < sizeof(*bpkt))
			return -EIO;

		ret = -EINVAL;
		if ((bpkt->header.type == pkt->type) &&
				(bpkt->header.cmd == pkt->cmd) &&
				(bpkt->header.flags & USBIO_PKTFLAG_RSP)) {
			ret = -ENODEV;
			if (!(bpkt->header.flags & USBIO_PKTFLAG_ERR)) {
				if (ibuf_len < bpkt->len)
					return -ENOBUFS;
				/* Copy the data */
				memcpy(ibuf, bpkt->data, bpkt->len);
				ret = bpkt->len;
			} else
				dev_err(usbio->dev,
						"Packet error type: %u, cmd: %u, flags: %u",
						bpkt->header.type, bpkt->header.cmd,
						bpkt->header.flags);
		} else
			dev_err(usbio->dev, "Unexpected reply type: %u, cmd: %u",
					bpkt->header.type, bpkt->header.cmd);
	} else
		ret = bpkt_len - sizeof(*bpkt);

	return ret;
}

static int usbio_ctrl_protver(struct usbio_device *usbio)
{
	struct usbio_packet_header pkt = {
		USBIO_PKTTYPE_CTRL,
		USBIO_CTRLCMD_PROTVER,
		USBIO_PKTFLAGS_REQRESP
	};
	struct usbio_protver *prot;
	int ret;

	prot = &usbio->info.protver;
	ret = usbio_control_msg(usbio, &pkt, 0, 0, &prot->ver, sizeof(*prot));
	if (ret != sizeof(*prot)) {
		dev_err(usbio->dev, "CTRL PROTVER failed: %d", ret);
		return -EIO;
	}

	return 0;
}

static int usbio_ctrl_fwver(struct usbio_device *usbio)
{
	struct usbio_packet_header pkt = {
		USBIO_PKTTYPE_CTRL,
		USBIO_CTRLCMD_FWVER,
		USBIO_PKTFLAGS_REQRESP
	};
	struct usbio_fwver *ver;
	int ret;

	ver = &usbio->info.fwver;
	ret = usbio_control_msg(usbio, &pkt, 0, 0, ver, sizeof(*ver));
	if (ret != sizeof(*ver)) {
		dev_err(usbio->dev, "CTRL FWVER failed: %d", ret);
		return -EIO;
	}

	return 0;
}

static int usbio_ctrl_handshake(struct usbio_device *usbio)
{
	struct usbio_packet_header pkt = {
		USBIO_PKTTYPE_CTRL,
		USBIO_CTRLCMD_HS,
		USBIO_PKTFLAGS_REQRESP
	};
	int ret;

	ret = usbio_control_msg(usbio, &pkt, 0, 0, 0, 0);
	if (ret) {
		dev_err(usbio->dev, "CTRL HANDSHAKE failed: %d", ret);
		return -EIO;
	}

	return 0;
}

static void usbio_auxdev_release(struct device *dev)
{
	struct auxiliary_device *adev = to_auxiliary_dev(dev);
	struct usbio_client *client = auxiliary_get_usbio_client(adev);

	client->bridge = NULL;
}

static int usbio_add_client(struct usbio_device *usbio, char *name,
				u8 type, u8 id, void *data)
{
	struct usbio_client *client;
	struct auxiliary_device *adev;
	int ret;

	client = kzalloc(sizeof(*client), GFP_KERNEL);
	if (!client)
		return -ENOMEM;

	client->type = type;
	client->id = id;
	client->bridge = (struct usbio_bridge *)usbio;
	adev = &client->adev;
	adev->name = name;
	adev->id = id;

	adev->dev.parent = usbio->dev;
	adev->dev.platform_data = data;
	adev->dev.release = usbio_auxdev_release;

	ret = auxiliary_device_init(adev);
	if (!ret) {
		if (auxiliary_device_add(adev))
			auxiliary_device_uninit(adev);
	}

	if (!ret)
		list_add_tail(&client->link, &usbio->cli_list);
	else
		kfree(client);

	return ret;
}

static int usbio_ctrl_enumgpios(struct usbio_device *usbio)
{
	struct usbio_packet_header pkt = {
		USBIO_PKTTYPE_CTRL,
		USBIO_CTRLCMD_ENUMGPIO,
		USBIO_PKTFLAGS_REQRESP
	};
	struct usbio_gpio_bank_desc *gpio;
	int ret, i;

	gpio = usbio->gpios;
	ret = usbio_control_msg(usbio, &pkt, 0, 0, gpio, sizeof(usbio->gpios));
	if (ret <= 0 || ret % sizeof(*gpio)) {
		dev_err(usbio->dev, "CTRL ENUMGPIO failed: %d", ret);
		return ret;
	}

	usbio->nr_gpio_banks = ret / sizeof(*gpio);
	dev_info(usbio->dev, "GPIO Banks: %d", usbio->nr_gpio_banks);
	for (i = 0; i < usbio->nr_gpio_banks; i++)
		dev_info(usbio->dev, "\tBank%d[%d] map: %#08x",
					gpio[i].id, gpio[i].pins, gpio[i].bmap);

	if (usbio_add_client(usbio, USBIO_GPIO_CLIENT, USBIO_GPIO, 0, gpio))
		dev_warn(usbio->dev, "Client %s add failed!", USBIO_GPIO_CLIENT);

	return 0;
}

static int usbio_ctrl_enumi2cs(struct usbio_device *usbio)
{
	struct usbio_packet_header pkt = {
		USBIO_PKTTYPE_CTRL,
		USBIO_CTRLCMD_ENUMI2C,
		USBIO_PKTFLAGS_REQRESP
	};
	struct usbio_i2c_bus_desc *i2c;
	int ret, i;

	i2c = usbio->i2cs;
	ret = usbio_control_msg(usbio, &pkt, 0, 0, i2c, sizeof(usbio->i2cs));
	if (ret <= 0 || ret % sizeof(*i2c)) {
		dev_err(usbio->dev, "CTRL ENUMI2C failed: %d", ret);
		return ret;
	}

	usbio->nr_i2c_buses = ret / sizeof(*i2c);
	dev_info(usbio->dev, "I2C Buses: %d", usbio->nr_i2c_buses);
	for (i = 0; i < usbio->nr_i2c_buses; i++) {
		dev_info(usbio->dev, "\tBus%d caps: %#02x", i2c[i].id, i2c[i].caps);
		if (usbio_add_client(usbio, USBIO_I2C_CLIENT, USBIO_I2C, i, &i2c[i]))
			dev_warn(usbio->dev, "Client %s.%d add failed!",
						USBIO_GPIO_CLIENT, i);
	}

	return 0;
}

static int usbio_ctrl_enumspis(struct usbio_device *usbio)
{
	struct usbio_packet_header pkt = {
		USBIO_PKTTYPE_CTRL,
		USBIO_CTRLCMD_ENUMSPI,
		USBIO_PKTFLAGS_REQRESP
	};
	struct usbio_spi_bus_desc *spi;
	int ret, i;

	spi = usbio->spis;
	ret = usbio_control_msg(usbio, &pkt, 0, 0, spi, sizeof(usbio->spis));
	if (ret <= 0 || ret % sizeof(*spi)) {
		dev_err(usbio->dev, "CTRL ENUMSPI failed: %d", ret);
		return ret;
	}

	usbio->nr_spi_buses = ret / sizeof(*spi);
	dev_info(usbio->dev, "SPI Buses: %d", usbio->nr_spi_buses);
	for (i = 0; i < usbio->nr_spi_buses; i++)
		dev_info(usbio->dev, "\tBus%d caps: %#02x", spi[i].id, spi[i].caps);

	return 0;
}

int usbio_gpio_handler(struct usbio_device *usbio, u8 cmd,
		const void *obuf, u16 obuf_len, void *ibuf, u16 ibuf_len)
{
	const struct usbio_gpio_init *init = obuf;
	struct usbio_packet_header pkt = {
		USBIO_PKTTYPE_GPIO,
		cmd,
		ibuf_len ? USBIO_PKTFLAGS_REQRESP : USBIO_PKTFLAG_CMP
	};
	int ret;

	if (!init || init->bankid > usbio->nr_gpio_banks)
		return -EINVAL;

	mutex_lock(&usbio->mutex);
	ret = usbio_control_msg(usbio, &pkt, obuf, obuf_len, ibuf, ibuf_len);
	if (ret > 0)
		ret -= obuf_len;
	mutex_unlock(&usbio->mutex);

	return ret;
}

#define I2C_RW_OVERHEAD (sizeof(struct usbio_bulk_packet) + \
			sizeof(struct usbio_i2c_rw))

int usbio_i2c_handler(struct usbio_device *usbio, u8 cmd,
		const void *obuf, u16 obuf_len, void *ibuf, u16 ibuf_len)
{
	const struct usbio_i2c_init *init = obuf;
	struct usbio_packet_header pkt = {
		USBIO_PKTTYPE_I2C,
		cmd,
		ibuf_len ? USBIO_PKTFLAGS_REQRESP : USBIO_PKTFLAG_CMP
	};
	int ret;

	if (!usbio->tx_pipe || !usbio->rx_pipe)
		return -ENXIO;

	if (!init || init->busid > usbio->nr_i2c_buses)
		return -EINVAL;

	switch (cmd) {
	case USBIO_I2CCMD_INIT:
		struct usbio_i2c_bus_desc *i2c = &usbio->i2cs[init->busid];
		unsigned int mode = i2c->caps & USBIO_I2C_BUS_MODE_CAP_MASK;
		uint32_t max_speed = usbio_i2c_speeds[mode];

		if (init->speed > max_speed) {
			u32 *speed = (u32 *)&init->speed;

			dev_warn(usbio->dev,
						"Invalid speed %u, adjusting to bus max %u",
						*speed, max_speed);
			*speed = max_speed;
		}
		break;

	case USBIO_I2CCMD_WRITE:
		const struct usbio_i2c_rw *i2cwr = obuf;
		u16 txchunk = usbio->txbuf_len - I2C_RW_OVERHEAD;
		u16 wsize = i2cwr->size;

		if (wsize > txchunk) {
			/* Need to split the output buffer */
			struct usbio_i2c_rw *wr;
			u16 len = 0;

			wr = kzalloc(sizeof(*wr) + txchunk, GFP_KERNEL);
			if (!wr) {
				dev_err(usbio->dev, "Failed to allocate i2c txchunk of %u",
						(u16)sizeof(*wr) + txchunk);
				return -ENOMEM;
			}

			memcpy(wr, i2cwr, sizeof(*wr));
			mutex_lock(&usbio->mutex);
			do {
				memcpy(wr->data, &i2cwr->data[len], txchunk);
				len += txchunk;

				ret = usbio_bulk_msg(usbio, &pkt, wsize == len,
										wr, sizeof(*wr) + txchunk, ibuf,
										ibuf_len, USBIO_BULKXFER_TIMEOUT);
				if (ret < 0)
					break;

				if (wsize - len < txchunk)
					txchunk = wsize - len;
			} while (wsize > len);
			mutex_unlock(&usbio->mutex);

			kfree(wr);

			return ret;
		}
		break;

	case USBIO_I2CCMD_READ:
		struct usbio_i2c_rw *i2crd = ibuf;
		u16 rxchunk = usbio->rxbuf_len - I2C_RW_OVERHEAD;
		u16 rsize = i2crd->size;

		if (rsize > rxchunk) {
			/* Need to split the input buffer */
			struct usbio_i2c_rw *rd;
			u16 len = 0;

			rd = kzalloc(sizeof(*rd) + rxchunk, GFP_KERNEL);
			if (!rd) {
				dev_err(usbio->dev, "Failed to allocate i2c rxchunk of %u",
						(u16)sizeof(*rd) + rxchunk);
				return -ENOMEM;
			}

			mutex_lock(&usbio->mutex);
			do {
				if (rsize - len < rxchunk)
					rxchunk = rsize - len;

				ret = usbio_bulk_msg(usbio, &pkt, true, obuf,
									len == 0 ? obuf_len : 0, rd,
									sizeof(*rd) + rxchunk,
									USBIO_BULKXFER_TIMEOUT);
				if (ret < 0)
					break;

				memcpy(&i2crd->data[len], rd->data, rxchunk);
				len += rxchunk;
			} while (rsize > len);
			mutex_unlock(&usbio->mutex);

			if (rsize == len)
				i2crd->size = rd->size;

			kfree(rd);

			return ret < 0 ? ret : sizeof(*i2crd) + i2crd->size;
		}
		break;
	}

	mutex_lock(&usbio->mutex);
	ret = usbio_bulk_msg(usbio, &pkt, true, obuf, obuf_len,
						ibuf, ibuf_len, USBIO_BULKXFER_TIMEOUT);
	mutex_unlock(&usbio->mutex);

	return ret;
}

int usbio_gpio_init(struct usbio_client *client,
		struct usbio_gpio_bank *banks, unsigned int len)
{
	struct usbio_device *usbio;
	struct usbio_gpio_bank_desc *gpio;
	int i;

	if (!client || !banks || !len)
		return -EINVAL;

	if (!client->bridge)
		return -ENODEV;

	usbio = (struct usbio_device *)client->bridge;
	gpio = usbio->gpios;
	for (i = 0; i < len && i < usbio->nr_gpio_banks; i++)
		banks[i].bitmap = gpio[i].bmap;

	return usbio->nr_gpio_banks;
}
EXPORT_SYMBOL_NS_GPL(usbio_gpio_init, "USBIO");

int usbio_transfer(struct usbio_client *client, u8 cmd,
		const void *obuf, u16 obuf_len, void *ibuf, u16 ibuf_len)
{
	struct usbio_device *usbio;
	int ret;

	if (!client)
		return -EINVAL;

	if (!client->bridge)
		return -ENODEV;

	usbio = (struct usbio_device *)client->bridge;
	switch (client->type) {
	case USBIO_GPIO:
		if (USBIO_GPIOCMD_VALID(cmd))
			ret = usbio_gpio_handler(usbio, cmd, obuf, obuf_len,
										ibuf, ibuf_len);
		break;
	case USBIO_I2C:
		if (USBIO_I2CCMD_VALID(cmd))
			ret = usbio_i2c_handler(usbio, cmd, obuf, obuf_len,
										ibuf, ibuf_len);
		break;
	}

	return ret;
}
EXPORT_SYMBOL_NS_GPL(usbio_transfer, "USBIO");

static int usbio_suspend(struct usb_interface *intf, pm_message_t msg)
{
	struct usbio_device *usbio = usb_get_intfdata(intf);

	usb_kill_urb(usbio->urb);

	return 0;
}

static int usbio_resume(struct usb_interface *intf)
{
	struct usbio_device *usbio = usb_get_intfdata(intf);

	return usb_submit_urb(usbio->urb, GFP_KERNEL);
}

static void usbio_disconnect(struct usb_interface *intf)
{
	struct device *dev = &intf->dev;
	struct usbio_device *usbio = usb_get_intfdata(intf);
	struct usbio_client *client, *prev;

	list_for_each_entry_safe_reverse(client, prev, &usbio->cli_list, link) {
		auxiliary_device_delete(&client->adev);
		auxiliary_device_uninit(&client->adev);

		list_del_init(&client->link);
		kfree(client);
	}

	usb_set_intfdata(intf, NULL);
	usb_put_intf(intf);

	kfree(usbio->ctrlbuf);
	kfree(usbio->txbuf);
	kfree(usbio->rxbuf);

	mutex_destroy(&usbio->mutex);
	devm_kfree(dev, usbio);
}

static int usbio_probe(struct usb_interface *intf,
						const struct usb_device_id *id)
{
	struct usb_device *udev = interface_to_usbdev(intf);
	struct usb_endpoint_descriptor *ep_in, *ep_out;
	struct device *dev = &intf->dev;
	struct usbio_device *usbio;
	int ret = -ENOMEM;

	usbio = devm_kzalloc(dev, sizeof(*usbio), GFP_KERNEL);
	if (!usbio)
		return ret;

	usbio->dev = dev;
	usbio->udev = udev;
	usbio->intf = usb_get_intf(intf);
	init_completion(&usbio->done);
	mutex_init(&usbio->mutex);
	INIT_LIST_HEAD(&usbio->cli_list);
	usb_set_intfdata(intf, usbio);

	usbio->ctrl_pipe = usb_endpoint_num(&udev->ep0.desc);
	usbio->ctrlbuf_len = usb_maxpacket(udev, usbio->ctrl_pipe);
	usbio->ctrlbuf = kzalloc(usbio->ctrlbuf_len, GFP_KERNEL);
	if (!usbio->ctrlbuf) {
		dev_err(dev, "Failed to allocate ctrlbuf of %u",
				usbio->ctrlbuf_len);
		goto error;
	}

	dev_dbg(dev, "ep0: %u size: %u\n",
			usb_pipeendpoint(usbio->ctrl_pipe), usbio->ctrlbuf_len);

	/* Find the first bulk-in and bulk-out endpoints */
	ret = usb_find_common_endpoints(intf->cur_altsetting,
									&ep_in, &ep_out, NULL, NULL);
	if (!ret) {
		ret = -ENOMEM;
		usbio->tx_pipe = usb_sndbulkpipe(udev, usb_endpoint_num(ep_out));
		usbio->txbuf_len = usb_endpoint_maxp(ep_out);
		usbio->txbuf = kzalloc(usbio->txbuf_len, GFP_KERNEL);
		if (!usbio->txbuf) {
			dev_err(dev, "Failed to allocate txbuf of %u",
					usbio->txbuf_len);
			goto error;
		}

		usbio->rx_pipe = usb_rcvbulkpipe(udev, usb_endpoint_num(ep_in));
		usbio->rxbuf_len = usb_endpoint_maxp(ep_in);
		usbio->rxbuf = kzalloc(usbio->rxbuf_len, GFP_KERNEL);
		if (!usbio->rxbuf) {
			dev_err(dev, "Failed to allocate rxbuf of %u",
					usbio->rxbuf_len);
			goto error;
		}

		dev_dbg(dev, "ep_out: %#02x size: %u ep_in: %#02x size: %u",
				ep_out->bEndpointAddress, usbio->txbuf_len,
				ep_in->bEndpointAddress, usbio->rxbuf_len);

		usbio->urb = usb_alloc_urb(0, GFP_KERNEL);
		if (!usbio->urb) {
			dev_err(dev, "Failed to allocate usb urb");
			goto error;
		}

		usb_fill_bulk_urb(usbio->urb, udev, usbio->rx_pipe, usbio->rxbuf,
				usbio->rxbuf_len, usbio_bulk_recv, usbio);
		ret = usb_submit_urb(usbio->urb, GFP_KERNEL);
		if (ret) {
			dev_err(dev, "Failed to submit usb urb");
			goto error;
		}
	} else
		dev_warn(dev, "Couldn't find both bulk-in and bulk-out endpoints");

	ret = usbio_ctrl_handshake(usbio);
	if (ret)
		goto error;

	ret = usbio_ctrl_protver(usbio);
	if (ret)
		goto error;

	ret = usbio_ctrl_fwver(usbio);
	if (ret)
		goto error;

	dev_info(dev, "ProtVer(BCD): %02x FwVer: %d.%d.%d.%d",
				usbio->info.protver.ver, usbio->info.fwver.major,
				usbio->info.fwver.minor, usbio->info.fwver.patch,
				usbio->info.fwver.build);

	if (usbio_ctrl_enumgpios(usbio))
		dev_err(dev, "Failed to enum GPIOs");
	if (usbio_ctrl_enumi2cs(usbio))
		dev_err(dev, "Failed to enum I2Cs");
	if (usbio_ctrl_enumspis(usbio))
		dev_err(dev, "Failed to enum SPIs");

	return 0;

error:
	usbio_disconnect(intf);

	return ret;
}

static const struct usb_device_id usbio_table[] = {
	{ USB_DEVICE(0x2AC1, 0x20C1) }, /* Lattice NX40 */
	{ USB_DEVICE(0x2AC1, 0x20C9) }, /* Lattice NX33 */
	{ USB_DEVICE(0x2AC1, 0x20CB) }, /* Lattice NX33U */
	{ USB_DEVICE(0x06CB, 0x0701) }, /* Synaptics Sabre */
	{ }
};
MODULE_DEVICE_TABLE(usb, usbio_table);

static struct usb_driver usbio_driver = {
	.name = "usbio-bridge",
	.probe = usbio_probe,
	.disconnect = usbio_disconnect,
	.suspend = usbio_suspend,
	.resume = usbio_resume,
	.id_table = usbio_table,
	.supports_autosuspend = 1
};
module_usb_driver(usbio_driver);

MODULE_DESCRIPTION("Intel USBIO Bridge driver");
MODULE_AUTHOR("Israel Cepeda <israel.a.cepeda.lopez@intel.com>");
MODULE_LICENSE("GPL");
