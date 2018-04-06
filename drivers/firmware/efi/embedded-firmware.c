// SPDX-License-Identifier: GPL-2.0
/*
 * Support for extracting embedded firmware for peripherals from EFI code,
 *
 * Copyright (c) 2018 Hans de Goede <hdegoede@redhat.com>
 */

#include <crypto/sha.h>
#include <linux/dmi.h>
#include <linux/efi.h>
#include <linux/efi_embedded_fw.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/vmalloc.h>

struct embedded_fw {
	struct list_head list;
	const char *name;
	void *data;
	size_t length;
};

static LIST_HEAD(found_fw_list);

static const struct dmi_system_id * const embedded_fw_table[] = {
#ifdef CONFIG_TOUCHSCREEN_DMI
	touchscreen_dmi_table,
#endif
	NULL
};

/*
 * Note the efi_check_for_embedded_firmwares() code currently makes the
 * following 2 assumptions. This may needs to be revisited if embedded firmware
 * is found where this is not true:
 * 1) The firmware is only found in EFI_BOOT_SERVICES_CODE memory segments
 * 2) The firmware always starts at an offset which is a multiple of 8 bytes
 */
static int __init efi_check_md_for_embedded_firmware(
	efi_memory_desc_t *md, const struct efi_embedded_fw_desc *desc)
{
	const u64 prefix = *((u64 *)desc->prefix);
	struct sha256_state sctx;
	struct embedded_fw *fw;
	u8 sha256[32];
	u64 i, size;
	void *map;

	size = md->num_pages << EFI_PAGE_SHIFT;
	map = memremap(md->phys_addr, size, MEMREMAP_WB);
	if (!map) {
		pr_err("Error mapping EFI mem at %#llx\n", md->phys_addr);
		return -ENOMEM;
	}

	size -= desc->length;
	for (i = 0; i < size; i += 8) {
		u64 *mem = map + i;

		if (*mem != prefix)
			continue;

		sha256_init_direct(&sctx);
		sha256_update_direct(&sctx, map + i, desc->length);
		sha256_final_direct(&sctx, sha256);
		if (memcmp(sha256, desc->sha256, 32) == 0)
			break;
	}
	if (i >= size) {
		memunmap(map);
		return -ENOENT;
	}

	pr_info("Found EFI embedded fw '%s'\n", desc->name);

	fw = kmalloc(sizeof(*fw), GFP_KERNEL);
	if (!fw) {
		memunmap(map);
		return -ENOMEM;
	}

	fw->data = kmemdup(map + i, desc->length, GFP_KERNEL);
	memunmap(map);
	if (!fw->data) {
		kfree(fw);
		return -ENOMEM;
	}

	fw->name = desc->name;
	fw->length = desc->length;
	list_add(&fw->list, &found_fw_list);

	return 0;
}

void __init efi_check_for_embedded_firmwares(void)
{
	const struct efi_embedded_fw_desc *fw_desc;
	const struct dmi_system_id *dmi_id;
	efi_memory_desc_t *md;
	int i, r;

	for (i = 0; embedded_fw_table[i]; i++) {
		dmi_id = dmi_first_match(embedded_fw_table[i]);
		if (!dmi_id)
			continue;

		fw_desc = dmi_id->driver_data;
		for_each_efi_memory_desc(md) {
			if (md->type != EFI_BOOT_SERVICES_CODE)
				continue;

			r = efi_check_md_for_embedded_firmware(md, fw_desc);
			if (r == 0)
				break;
		}
	}
}

int efi_get_embedded_fw(const char *name, void **data, size_t *size,
			size_t msize)
{
	struct embedded_fw *iter, *fw = NULL;
	void *buf = *data;

	list_for_each_entry(iter, &found_fw_list, list) {
		if (strcmp(name, iter->name) == 0) {
			fw = iter;
			break;
		}
	}

	if (!fw)
		return -ENOENT;

	if (msize && msize < fw->length)
		return -EFBIG;

	if (!buf) {
		buf = vmalloc(fw->length);
		if (!buf)
			return -ENOMEM;
	}

	memcpy(buf, fw->data, fw->length);
	*size = fw->length;
	*data = buf;

	return 0;
}
EXPORT_SYMBOL_GPL(efi_get_embedded_fw);
