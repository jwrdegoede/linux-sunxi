/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2025 Intel Corporation.
 *
 */

#ifndef _USBIO_H_
#define _USBIO_H_

#include <linux/mutex.h>
#include <linux/i2c.h>
#include <linux/usb.h>
#include <linux/usbio.h>

/*************************************
 * USBIO Bridge Protocol Definitions *
 *************************************/

/* USBIO Packet Type */
#define USBIO_PKTTYPE_CTRL	1
#define USBIO_PKTTYPE_DBG	2
#define USBIO_PKTTYPE_GPIO	USBIO_GPIO
#define USBIO_PKTTYPE_I2C	USBIO_I2C
#define USBIO_PKTTYPE_SPI	USBIO_SPI

/* USBIO Control Commands */
#define USBIO_CTRLCMD_PROTVER	0
#define USBIO_CTRLCMD_FWVER		1
#define USBIO_CTRLCMD_HS		2
#define USBIO_CTRLCMD_ENUMGPIO	16
#define USBIO_CTRLCMD_ENUMI2C	17
#define USBIO_CTRLCMD_ENUMSPI	18

/* USBIO Packet Flags */
#define USBIO_PKTFLAG_ACK	BIT(0)
#define USBIO_PKTFLAG_RSP	BIT(1)
#define USBIO_PKTFLAG_CMP	BIT(2)
#define USBIO_PKTFLAG_ERR	BIT(3)

#define USBIO_PKTFLAGS_REQRESP	(USBIO_PKTFLAG_CMP | USBIO_PKTFLAG_ACK)
#define USBIO_PKTFLAGS_ISRSP	(USBIO_PKTFLAG_CMP | USBIO_PKTFLAG_RSP)

/* USBIO Packet Header */
struct usbio_packet_header {
	uint8_t type;
	uint8_t cmd;
	uint8_t flags;
} __packed;

#define USBIO_CTRLXFER_TIMEOUT 0
#define USBIO_BULKXFER_TIMEOUT 80

/* USBIO Control Transfer Packet */
struct usbio_ctrl_packet {
	struct usbio_packet_header header;
	uint8_t len;
	uint8_t data[] __counted_by(len);
} __packed;

/* USBIO Bulk Transfer Packet */
struct usbio_bulk_packet {
	struct usbio_packet_header header;
	uint16_t len;
	uint8_t data[] __counted_by(len);
} __packed;

struct usbio_protver {
	uint8_t ver;
} __packed;

struct usbio_fwver {
	uint8_t major;
	uint8_t minor;
	uint16_t patch;
	uint16_t build;
} __packed;

struct usbio_gpio_bank_desc {
	uint8_t id;
	uint8_t pins;
	uint32_t bmap;
} __packed;

#define USBIO_I2C_BUS_ADDR_CAP_10B	BIT(3) /* 10bit address support */
#define USBIO_I2C_BUS_MODE_CAP_MASK	0x3
#define USBIO_I2C_BUS_MODE_CAP_SM	0 /* Standard Mode */
#define USBIO_I2C_BUS_MODE_CAP_FM	1 /* Fast Mode */
#define USBIO_I2C_BUS_MODE_CAP_FMP	2 /* Fast Mode+ */
#define USBIO_I2C_BUS_MODE_CAP_HSM	3 /* High-Speed Mode */

struct usbio_i2c_bus_desc {
	uint8_t id;
	uint8_t caps;
} __packed;

uint32_t usbio_i2c_speeds[] = {
	I2C_MAX_STANDARD_MODE_FREQ,
	I2C_MAX_FAST_MODE_FREQ,
	I2C_MAX_FAST_MODE_PLUS_FREQ,
	I2C_MAX_HIGH_SPEED_MODE_FREQ
};

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

int usbio_gpio_handler(struct usbio_device *usbio, u8 cmd,
		const void *obuf, u16 obuf_len, void *ibuf, u16 ibuf_len);

int usbio_i2c_handler(struct usbio_device *usbio, u8 cmd,
		const void *obuf, u16 obuf_len, void *ibuf, u16 ibuf_len);

#endif
