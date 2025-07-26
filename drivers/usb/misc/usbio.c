// SPDX-License-Identifier: GPL-2.0
/*
 * Intel USBIO Bridge driver
 *
 * Copyright (c) 2025 Intel Corporation.
 * Copyright (c) 2025 Red Hat, Inc.
 */

#include <linux/acpi.h>
#include <linux/auxiliary_bus.h>
#include <linux/completion.h>
#include <linux/dev_printk.h>
#include <linux/device.h>
#include <linux/lockdep.h>
#include <linux/mutex.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/usb.h>
#include <linux/usb/usbio.h>

/*************************************
 * USBIO Bridge Protocol Definitions *
 *************************************/

/* USBIO Control Commands */
#define USBIO_CTRLCMD_PROTVER	0
#define USBIO_CTRLCMD_FWVER	1
#define USBIO_CTRLCMD_HS	2
#define USBIO_CTRLCMD_ENUMGPIO	16
#define USBIO_CTRLCMD_ENUMI2C	17
#define USBIO_CTRLCMD_ENUMSPI	18

/* USBIO Packet Flags */
#define USBIO_PKTFLAG_ACK	BIT(0)
#define USBIO_PKTFLAG_RSP	BIT(1)
#define USBIO_PKTFLAG_CMP	BIT(2)
#define USBIO_PKTFLAG_ERR	BIT(3)

#define USBIO_PKTFLAGS_REQRESP	(USBIO_PKTFLAG_CMP | USBIO_PKTFLAG_ACK)

#define USBIO_CTRLXFER_TIMEOUT 0
#define USBIO_BULKXFER_TIMEOUT 100

struct usbio_protver {
	uint8_t ver;
} __packed;

struct usbio_fwver {
	uint8_t major;
	uint8_t minor;
	uint16_t patch;
	uint16_t build;
} __packed;

#define USBIO_SPI_BUS_WIRE_CAP_3W	BIT(3) /* 3 wires support */
#define USBIO_SPI_BUS_ADDR_CAP_MASK	0x3
#define USBIO_SPI_BUS_ADDR_CAP_CS0	0 /* CS0 */
#define USBIO_SPI_BUS_ADDR_CAP_CS1	1 /* CS0/1 */
#define USBIO_SPI_BUS_ADDR_CAP_CS2	2 /* CS0/1/2 */
#define USBIO_SPI_BUS_ADDR_CAP_CS3	3 /* CS0/1/2/3 */

struct usbio_spi_bus_desc {
	uint8_t id;
	uint8_t caps;
} __packed;

/***********************************
 * USBIO Bridge Device Definitions *
 ***********************************/
struct usbio_dev_info {
	struct usbio_protver protver;
	struct usbio_fwver fwver;
};

#define MAX_SPIBUSES	5

/**
 * struct usbio_device - the usb device exposing IOs
 *
 * @dev: the device in the usb interface
 * @udev: the detected usb device
 * @intf: the usb interface
 * @ctrl_pipe: the control transfer pipe
 * @ctrlbuf_len: the size of the control transfer pipe
 * @ctrlbuf: the buffer used for control transfers
 * @tx_pipe: the bulk out pipe
 * @txbuf_len: the size of the bulk out pipe
 * @txbuf: the buffer used for bulk out transfers
 * @urb: the urb to read bulk pipe
 * @rx_pipe: the bulk in pipe
 * @rxbuf_len: the size of the bulk in pipe
 * @rxdat_len: the data length at rx buffer
 * @rxbuf: the buffer used for bulk in transfers
 * @info: the device's protocol and firmware information
 * @done: completion object as request is done
 * @mutex: protection against access concurrency
 * @cli_list: device's client list
 */
struct usbio_device {
	struct device *dev;
	struct usb_device *udev;
	struct usb_interface *intf;

	unsigned int ctrl_pipe;
	u16 ctrlbuf_len;
	void *ctrlbuf;

	unsigned int tx_pipe;
	u16 txbuf_len;
	void *txbuf;

	struct urb *urb;
	unsigned int rx_pipe;
	u16 rxbuf_len;
	u16 rxdat_len;
	void *rxbuf;

	struct usbio_dev_info info;

	struct completion done;
	struct mutex mutex;

	struct list_head cli_list;

	unsigned int nr_gpio_banks;
	struct usbio_gpio_bank_desc gpios[USBIO_MAX_GPIOBANKS];

	unsigned int nr_i2c_buses;
	struct usbio_i2c_bus_desc i2cs[USBIO_MAX_I2CBUSES];

	unsigned int nr_spi_buses;
	struct usbio_spi_bus_desc spis[MAX_SPIBUSES];
};

/**
 * struct usbio_client - represents a usbio client
 *
 * @adev: auxiliary device object
 * @bridge: usbio bridge who service the client
 * @link: usbio bridge clients list member
 */
struct usbio_client {
	struct auxiliary_device adev;
	struct usbio_device *bridge;
	struct list_head link;
};

#define adev_to_client(adev) container_of(adev, struct usbio_client, adev)

static int usbio_ctrl_msg(struct usbio_device *usbio, u8 type, u8 cmd,
			  const void *obuf, u16 obuf_len, void *ibuf, u16 ibuf_len)
{
	u8 request = USB_TYPE_VENDOR | USB_RECIP_DEVICE;
	struct usbio_ctrl_packet *cpkt;
	unsigned int pipe;
	u16 cpkt_len;
	int ret;

	lockdep_assert_held(&usbio->mutex);

	if ((obuf_len > (usbio->ctrlbuf_len - sizeof(*cpkt))) ||
	    (ibuf_len > (usbio->ctrlbuf_len - sizeof(*cpkt))))
		return -EMSGSIZE;

	/* Prepare Control Packet Header */
	cpkt = usbio->ctrlbuf;
	cpkt->header.type = type;
	cpkt->header.cmd = cmd;
	if (type == USBIO_PKTTYPE_CTRL || ibuf_len)
		cpkt->header.flags = USBIO_PKTFLAGS_REQRESP;
	else
		cpkt->header.flags = USBIO_PKTFLAG_CMP;
	cpkt->len = obuf_len;

	/* Copy the data */
	memcpy(cpkt->data, obuf, obuf_len);

	pipe = usb_sndctrlpipe(usbio->udev, usbio->ctrl_pipe);
	cpkt_len = sizeof(*cpkt) + obuf_len;
	ret = usb_control_msg(usbio->udev, pipe, 0, request | USB_DIR_OUT, 0, 0,
			      cpkt, cpkt_len, USBIO_CTRLXFER_TIMEOUT);
	dev_dbg(usbio->dev, "control out %d hdr %*phN data %*phN\n", ret,
		(int)sizeof(*cpkt), cpkt, (int)cpkt->len, cpkt->data);

	if (ret != cpkt_len) {
		dev_err(usbio->dev, "USB control out failed: %d\n", ret);
		return (ret < 0) ? ret : -EPROTO;
	}

	if (!(cpkt->header.flags & USBIO_PKTFLAG_ACK))
		return 0;

	pipe = usb_rcvctrlpipe(usbio->udev, usbio->ctrl_pipe);
	cpkt_len = sizeof(*cpkt) + ibuf_len;
	ret = usb_control_msg(usbio->udev, pipe, 0, request | USB_DIR_IN, 0, 0,
			      cpkt, cpkt_len, USBIO_CTRLXFER_TIMEOUT);
	dev_dbg(usbio->dev, "control in %d hdr %*phN data %*phN\n", ret,
		(int)sizeof(*cpkt), cpkt, (int)cpkt->len, cpkt->data);

	if (ret < sizeof(*cpkt)) {
		dev_err(usbio->dev, "USB control in failed: %d\n", ret);
		return (ret < 0) ? ret : -EPROTO;
	}

	if (cpkt->header.type != type || cpkt->header.cmd != cmd ||
	    !(cpkt->header.flags & USBIO_PKTFLAG_RSP)) {
		dev_err(usbio->dev, "Unexpected reply type: %u, cmd: %u, flags: %u\n",
			cpkt->header.type, cpkt->header.cmd, cpkt->header.flags);
		return -EPROTO;
	}

	if (cpkt->header.flags & USBIO_PKTFLAG_ERR)
		return -EREMOTEIO;

	if (ibuf_len < cpkt->len)
		return -ENOSPC;

	memcpy(ibuf, cpkt->data, cpkt->len);
	return cpkt->len;
}

int usbio_control_msg(struct auxiliary_device *adev, u8 type, u8 cmd,
		      const void *obuf, u16 obuf_len, void *ibuf, u16 ibuf_len)
{
	struct usbio_client *client = adev_to_client(adev);
	int ret;

	ret = usbio_acquire(adev);
	if (ret)
		return ret;

	ret = usbio_ctrl_msg(client->bridge, type, cmd, obuf, obuf_len, ibuf, ibuf_len);

	usbio_release(adev);

	return ret;
}
EXPORT_SYMBOL_NS_GPL(usbio_control_msg, "USBIO");

static void usbio_bulk_recv(struct urb *urb)
{
	struct usbio_bulk_packet *bpkt = urb->transfer_buffer;
	struct usbio_device *usbio = urb->context;

	if (!urb->status) {
		if (bpkt->header.flags & USBIO_PKTFLAG_RSP) {
			usbio->rxdat_len = urb->actual_length;
			complete(&usbio->done);
		}
	} else if (urb->status != -ENOENT) {
		dev_err(usbio->dev, "Bulk in error %d\n", urb->status);
	}

	usb_submit_urb(usbio->urb, GFP_ATOMIC);
}

int usbio_bulk_msg(struct auxiliary_device *adev, u8 type, u8 cmd, bool last,
		   const void *obuf, u16 obuf_len, void *ibuf, u16 ibuf_len)
{
	struct usbio_device *usbio = adev_to_client(adev)->bridge;
	struct usbio_bulk_packet *bpkt;
	int ret, act;
	u16 bpkt_len;

	lockdep_assert_held(&usbio->mutex);

	if ((obuf_len > (usbio->txbuf_len - sizeof(*bpkt))) ||
	    (ibuf_len > (usbio->txbuf_len - sizeof(*bpkt))))
		return -EMSGSIZE;

	if (ibuf_len)
		reinit_completion(&usbio->done);

	/* If no data to send, skip to read */
	if (!obuf_len)
		goto read;

	/* Prepare Bulk Packet Header */
	bpkt = usbio->txbuf;
	bpkt->header.type = type;
	bpkt->header.cmd = cmd;
	if (!last)
		bpkt->header.flags = 0;
	else if (ibuf_len)
		bpkt->header.flags = USBIO_PKTFLAGS_REQRESP;
	else
		bpkt->header.flags = USBIO_PKTFLAG_CMP;
	bpkt->len = obuf_len;

	/* Copy the data */
	memcpy(bpkt->data, obuf, obuf_len);

	bpkt_len = sizeof(*bpkt) + obuf_len;
	ret = usb_bulk_msg(usbio->udev, usbio->tx_pipe, bpkt, bpkt_len, &act,
			   USBIO_BULKXFER_TIMEOUT);
	dev_dbg(usbio->dev, "bulk out %d hdr %*phN data %*phN\n", act,
		(int)sizeof(*bpkt), bpkt, (int)bpkt->len, bpkt->data);

	if (ret || act != bpkt_len) {
		dev_err(usbio->dev, "Bulk out failed: %d\n", ret);
		return ret ?: -EPROTO;
	}

	if (!(bpkt->header.flags & USBIO_PKTFLAG_ACK))
		return obuf_len;

read:
	ret = wait_for_completion_timeout(&usbio->done, USBIO_BULKXFER_TIMEOUT);
	if (ret <= 0) {
		dev_err(usbio->dev, "Bulk in wait failed: %d\n", ret);
		return ret ?: -ETIMEDOUT;
	}

	act = usbio->rxdat_len;
	bpkt = usbio->rxbuf;
	dev_dbg(usbio->dev, "bulk in %d hdr %*phN data %*phN\n", act,
		(int)sizeof(*bpkt), bpkt, (int)bpkt->len, bpkt->data);

	/*
	 * Unsupported bulk commands get only an usbio_packet_header with
	 * the error flag set as reply. Return -EPIPE for this case.
	 */
	if (act == sizeof(struct usbio_packet_header) &&
	    (bpkt->header.flags & USBIO_PKTFLAG_ERR))
		return -EPIPE;

	if (act < sizeof(*bpkt)) {
		dev_err(usbio->dev, "Bulk in short read: %d\n", act);
		return -EPROTO;
	}

	if (bpkt->header.type != type || bpkt->header.cmd != cmd ||
	    !(bpkt->header.flags & USBIO_PKTFLAG_RSP)) {
		dev_err(usbio->dev,
			"Unexpected bulk in type 0x%02x cmd 0x%02x flags 0x%02x\n",
			bpkt->header.type, bpkt->header.cmd, bpkt->header.flags);
		return -EPROTO;
	}

	if (bpkt->header.flags & USBIO_PKTFLAG_ERR)
		return -EREMOTEIO;

	if (ibuf_len < bpkt->len)
		return -ENOSPC;

	memcpy(ibuf, bpkt->data, bpkt->len);
	return bpkt->len;
}
EXPORT_SYMBOL_NS_GPL(usbio_bulk_msg, "USBIO");

static int usbio_get_and_lock(struct usbio_device *usbio)
{
	int ret;

	ret = usb_autopm_get_interface(usbio->intf);
	if (ret)
		return ret;

	mutex_lock(&usbio->mutex);
	return 0;
}

static void usbio_unlock_and_put(struct usbio_device *usbio)
{
	mutex_unlock(&usbio->mutex);
	usb_autopm_put_interface(usbio->intf);
}

int usbio_acquire(struct auxiliary_device *adev)
{
	return usbio_get_and_lock(adev_to_client(adev)->bridge);
}
EXPORT_SYMBOL_NS_GPL(usbio_acquire, "USBIO");

void usbio_release(struct auxiliary_device *adev)
{
	usbio_unlock_and_put(adev_to_client(adev)->bridge);
}
EXPORT_SYMBOL_NS_GPL(usbio_release, "USBIO");

void usbio_get_txrxbuf_len(struct auxiliary_device *adev, u16 *txbuf_len, u16 *rxbuf_len)
{
	struct usbio_device *usbio = adev_to_client(adev)->bridge;

	*txbuf_len = usbio->txbuf_len;
	*rxbuf_len = usbio->rxbuf_len;
}
EXPORT_SYMBOL_NS_GPL(usbio_get_txrxbuf_len, "USBIO");

static void usbio_auxdev_release(struct device *dev)
{
	struct auxiliary_device *adev = to_auxiliary_dev(dev);
	struct usbio_client *client = adev_to_client(adev);

	kfree(client);
}

static int usbio_add_client(struct usbio_device *usbio, char *name, u8 id, void *data)
{
	struct usbio_client *client;
	struct auxiliary_device *adev;
	int ret;

	client = kzalloc(sizeof(*client), GFP_KERNEL);
	if (!client)
		return -ENOMEM;

	client->bridge = usbio;
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

	if (ret) {
		dev_err(usbio->dev, "Client %s.%d add failed: %d\n", name, id, ret);
		kfree(client);
		return ret;
	}

	list_add_tail(&client->link, &usbio->cli_list);
	return 0;
}

static int usbio_ctrl_enumgpios(struct usbio_device *usbio)
{
	struct usbio_gpio_bank_desc *gpio = usbio->gpios;
	int ret, i;

	ret = usbio_ctrl_msg(usbio, USBIO_PKTTYPE_CTRL, USBIO_CTRLCMD_ENUMGPIO, NULL, 0,
			     gpio, sizeof(usbio->gpios));
	if (ret < 0 || ret % sizeof(*gpio)) {
		dev_err(usbio->dev, "CTRL ENUMGPIO failed: %d\n", ret);
		return ret;
	}

	usbio->nr_gpio_banks = ret / sizeof(*gpio);
	dev_dbg(usbio->dev, "GPIO Banks: %d\n", usbio->nr_gpio_banks);
	for (i = 0; i < usbio->nr_gpio_banks; i++)
		dev_dbg(usbio->dev, "\tBank%d[%d] map: %#08x\n",
			gpio[i].id, gpio[i].pins, gpio[i].bmap);

	usbio_add_client(usbio, USBIO_GPIO_CLIENT, 0, gpio);
	return 0;
}

static int usbio_ctrl_enumi2cs(struct usbio_device *usbio)
{
	struct usbio_i2c_bus_desc *i2c = usbio->i2cs;
	int ret, i;

	ret = usbio_ctrl_msg(usbio, USBIO_PKTTYPE_CTRL, USBIO_CTRLCMD_ENUMI2C, NULL, 0,
			     i2c, sizeof(usbio->i2cs));
	if (ret < 0 || ret % sizeof(*i2c)) {
		dev_err(usbio->dev, "CTRL ENUMI2C failed: %d\n", ret);
		return ret;
	}

	usbio->nr_i2c_buses = ret / sizeof(*i2c);
	dev_dbg(usbio->dev, "I2C Busses: %d\n", usbio->nr_i2c_buses);
	for (i = 0; i < usbio->nr_i2c_buses; i++) {
		dev_dbg(usbio->dev, "\tBus%d caps: %#02x\n", i2c[i].id, i2c[i].caps);
		usbio_add_client(usbio, USBIO_I2C_CLIENT, i, &i2c[i]);
	}

	return 0;
}

static int usbio_ctrl_enumspis(struct usbio_device *usbio)
{
	struct usbio_spi_bus_desc *spi = usbio->spis;
	int ret, i;

	ret = usbio_ctrl_msg(usbio, USBIO_PKTTYPE_CTRL, USBIO_CTRLCMD_ENUMSPI, NULL, 0,
			     spi, sizeof(usbio->spis));
	if (ret < 0 || ret % sizeof(*spi)) {
		dev_err(usbio->dev, "CTRL ENUMSPI failed: %d\n", ret);
		return ret;
	}

	usbio->nr_spi_buses = ret / sizeof(*spi);
	dev_dbg(usbio->dev, "SPI Busses: %d\n", usbio->nr_spi_buses);
	for (i = 0; i < usbio->nr_spi_buses; i++)
		dev_dbg(usbio->dev, "\tBus%d caps: %#02x\n", spi[i].id, spi[i].caps);

	return 0;
}

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
	struct usbio_device *usbio = usb_get_intfdata(intf);
	struct usbio_client *client, *prev;

	list_for_each_entry_safe_reverse(client, prev, &usbio->cli_list, link) {
		auxiliary_device_delete(&client->adev);
		list_del_init(&client->link);
		auxiliary_device_uninit(&client->adev);
	}

	usb_kill_urb(usbio->urb);
	usb_free_urb(usbio->urb);
}

static int usbio_probe(struct usb_interface *intf, const struct usb_device_id *id)
{
	struct usb_device *udev = interface_to_usbdev(intf);
	struct usb_endpoint_descriptor *ep_in, *ep_out;
	struct device *dev = &intf->dev;
	struct usbio_device *usbio;
	int ret;

	usbio = devm_kzalloc(dev, sizeof(*usbio), GFP_KERNEL);
	if (!usbio)
		return -ENOMEM;

	ret = devm_mutex_init(dev, &usbio->mutex);
	if (ret)
		return ret;

	usbio->dev = dev;
	usbio->udev = udev;
	usbio->intf = intf;
	init_completion(&usbio->done);
	INIT_LIST_HEAD(&usbio->cli_list);
	usb_set_intfdata(intf, usbio);

	usbio->ctrl_pipe = usb_endpoint_num(&udev->ep0.desc);
	usbio->ctrlbuf_len = usb_maxpacket(udev, usbio->ctrl_pipe);
	usbio->ctrlbuf = devm_kzalloc(dev, usbio->ctrlbuf_len, GFP_KERNEL);
	if (!usbio->ctrlbuf)
		return -ENOMEM;

	/* Find the first bulk-in and bulk-out endpoints */
	ret = usb_find_common_endpoints(intf->cur_altsetting, &ep_in, &ep_out,
					NULL, NULL);
	if (ret) {
		dev_err(dev, "Cannot find bulk endpoints: %d\n", ret);
		return ret;
	}

	usbio->tx_pipe = usb_sndbulkpipe(udev, usb_endpoint_num(ep_out));
	usbio->txbuf_len = usb_endpoint_maxp(ep_out);
	usbio->txbuf = devm_kzalloc(dev, usbio->txbuf_len, GFP_KERNEL);
	if (!usbio->txbuf)
		return -ENOMEM;

	usbio->rx_pipe = usb_rcvbulkpipe(udev, usb_endpoint_num(ep_in));
	usbio->rxbuf_len = usb_endpoint_maxp(ep_in);
	usbio->rxbuf = devm_kzalloc(dev, usbio->rxbuf_len, GFP_KERNEL);
	if (!usbio->rxbuf)
		return -ENOMEM;

	usbio->urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!usbio->urb)
		return -ENOMEM;

	usb_fill_bulk_urb(usbio->urb, udev, usbio->rx_pipe, usbio->rxbuf,
			  usbio->rxbuf_len, usbio_bulk_recv, usbio);
	ret = usb_submit_urb(usbio->urb, GFP_KERNEL);
	if (ret)
		return dev_err_probe(dev, ret, "Submitting usb urb\n");

	ret = usbio_get_and_lock(usbio);
	if (ret)
		return ret;

	ret = usbio_ctrl_msg(usbio, USBIO_PKTTYPE_CTRL, USBIO_CTRLCMD_HS, NULL, 0, NULL, 0);
	if (ret < 0)
		goto error;

	ret = usbio_ctrl_msg(usbio, USBIO_PKTTYPE_CTRL, USBIO_CTRLCMD_PROTVER, NULL, 0,
			     &usbio->info.protver, sizeof(struct usbio_protver));
	if (ret < 0)
		goto error;

	ret = usbio_ctrl_msg(usbio, USBIO_PKTTYPE_CTRL, USBIO_CTRLCMD_FWVER, NULL, 0,
			     &usbio->info.fwver, sizeof(struct usbio_fwver));
	if (ret < 0)
		goto error;

	dev_dbg(dev, "ProtVer(BCD): %02x FwVer: %d.%d.%d.%d\n",
		usbio->info.protver.ver, usbio->info.fwver.major,
		usbio->info.fwver.minor, usbio->info.fwver.patch,
		usbio->info.fwver.build);

	usbio_ctrl_enumgpios(usbio);
	usbio_ctrl_enumi2cs(usbio);
	usbio_ctrl_enumspis(usbio);
	usbio_unlock_and_put(usbio);
	return 0;

error:
	usbio_unlock_and_put(usbio);
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

struct usbio_match_ids_walk_data {
	struct acpi_device *adev;
	const struct acpi_device_id *hids;
	const u8 id;
};

static int usbio_match_device_ids(struct acpi_device *adev, void *data)
{
	struct usbio_match_ids_walk_data *wd = data;
	unsigned int id = 0;
	char *uid;

	if (!acpi_match_device_ids(adev, wd->hids)) {
		uid = acpi_device_uid(adev);
		if (uid)
			for (int i = 0; i < strlen(uid); i++)
				if (!kstrtouint(&uid[i], 10, &id))
					break;

		if (!uid || (uid && wd->id == (u8)id)) {
			wd->adev = adev;
			return 1;
		}
	}

	return 0;
}

void usbio_acpi_bind(struct auxiliary_device *adev, const struct acpi_device_id *hids)
{
	struct device *dev = &adev->dev;
	struct acpi_device *parent;
	struct usbio_match_ids_walk_data wd = {
		.adev = NULL,
		.hids = hids,
		.id = adev->id,
	};

	parent = ACPI_COMPANION(dev->parent);
	if (!parent)
		return;

	acpi_dev_for_each_child(parent, usbio_match_device_ids, &wd);
	if (wd.adev)
		ACPI_COMPANION_SET(dev, wd.adev);
}
EXPORT_SYMBOL_NS_GPL(usbio_acpi_bind, "USBIO");

MODULE_DESCRIPTION("Intel USBIO Bridge driver");
MODULE_AUTHOR("Israel Cepeda <israel.a.cepeda.lopez@intel.com>");
MODULE_AUTHOR("Hans de Goede <hansg@kernel.org>");
MODULE_LICENSE("GPL");
