/*
 * Copyright (C) 2014 Boris BREZILLON <b.brezillon.dev@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/mtd/nand.h>
#include <linux/slab.h>

static u8 h27ucg8t2a_read_retry_regs[] = {
	0xcc, 0xbf, 0xaa, 0xab, 0xcd, 0xad, 0xae, 0xaf
};

struct hynix_read_retry {
	u8 *regs;
	u8 values[64];
};

struct hynix_nand {
	struct hynix_read_retry read_retry;
};

int nand_setup_read_retry_hynix(struct mtd_info *mtd, int retry_mode)
{
	struct nand_chip *chip = mtd->priv;
	struct hynix_nand *hynix = chip->manuf_priv;
	int offset = retry_mode * 8;
	int status;
	int i;

	chip->cmdfunc(mtd, 0x36, -1, -1);
	for (i = 0; i < 8; i++) {
		int column = hynix->read_retry.regs[i];
		column |= column << 8;
		chip->cmdfunc(mtd, NAND_CMD_NONE, column, -1);
		chip->write_byte(mtd, hynix->read_retry.values[offset + i]);
	}
	chip->cmdfunc(mtd, 0x16, -1, -1);

	status = chip->waitfunc(mtd, chip);
	if (status & NAND_STATUS_FAIL)
		return -EIO;

	return 0;
}

static void h27ucg8t2a_cleanup(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd->priv;
	kfree(chip->manuf_priv);
}

static int h27ucg8t2a_init(struct mtd_info *mtd, const uint8_t *id)
{
	struct nand_chip *chip = mtd->priv;
	struct hynix_nand *hynix;
	u8 * buf = NULL;
	int i, j;
	int ret;

	buf = kzalloc(1024, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	chip->select_chip(mtd, 0);
	chip->cmdfunc(mtd, NAND_CMD_RESET, -1, -1);
	chip->cmdfunc(mtd, 0x36, 0xff, -1);
	chip->write_byte(mtd, 0x40);
	chip->cmdfunc(mtd, NAND_CMD_NONE, 0xcc, -1);
	chip->write_byte(mtd, 0x4d);
	chip->cmdfunc(mtd, 0x16, -1, -1);
	chip->cmdfunc(mtd, 0x17, -1, -1);
	chip->cmdfunc(mtd, 0x04, -1, -1);
	chip->cmdfunc(mtd, 0x19, -1, -1);
	chip->cmdfunc(mtd, NAND_CMD_READ0, 0x0, 0x200);

	chip->read_buf(mtd, buf, 2);
	if (buf[0] != 0x8 || buf[1] != 0x8) {
		ret = -EINVAL;
		goto leave;
	}
	chip->read_buf(mtd, buf, 1024);

	ret = 0;
	for (j = 0; j < 8; j++) {
		for (i = 0; i < 64; i++) {
			u8 *tmp = buf + (128 * j);
			if ((tmp[i] | tmp[i + 64]) != 0xff) {
				ret = -EINVAL;
				goto leave;
			}
		}
	}

	chip->cmdfunc(mtd, NAND_CMD_RESET, -1, -1);
	chip->cmdfunc(mtd, 0x38, -1, -1);
	chip->select_chip(mtd, -1);

	if (!ret) {
		hynix = kzalloc(sizeof(*hynix), GFP_KERNEL);
		if (!hynix) {
			ret = -ENOMEM;
			goto leave;
		}

		hynix->read_retry.regs = h27ucg8t2a_read_retry_regs;
		memcpy(hynix->read_retry.values, buf, 64);
		chip->manuf_priv = hynix;
		chip->setup_read_retry = nand_setup_read_retry_hynix;
		chip->read_retries = 8;
		chip->manuf_cleanup = h27ucg8t2a_cleanup;
	}

leave:
	kfree(buf);

	return ret;
}

struct hynix_nand_initializer {
	u8 id[6];
	int (*init)(struct mtd_info *mtd, const uint8_t *id);
};

struct hynix_nand_initializer initializers[] = {
	{
		.id = {NAND_MFR_HYNIX, 0xde, 0x94, 0xda, 0x74, 0xc4},
		.init = h27ucg8t2a_init,
	},
};

int hynix_nand_init(struct mtd_info *mtd, const uint8_t *id)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(initializers); i++) {
		struct hynix_nand_initializer *initializer = &initializers[i];
		if (memcmp(id, initializer->id, sizeof(initializer->id)))
			continue;

		return initializer->init(mtd, id);
	}

	return 0;
}
EXPORT_SYMBOL(hynix_nand_init);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Boris BREZILLON <b.brezillon.dev@gmail.com>");
MODULE_DESCRIPTION("Hynix NAND specific code");
