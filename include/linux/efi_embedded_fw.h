/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _LINUX_EFI_EMBEDDED_FW_H
#define _LINUX_EFI_EMBEDDED_FW_H

#include <linux/mod_devicetable.h>

/**
 * struct efi_embedded_fw_desc - This struct is used by the EFI embedded-fw
 *                               code to search for embedded firmwares.
 *
 * @name:   Name to register the firmware with if found
 * @prefix: First 8 bytes of the firmware
 * @length: Length of the firmware in bytes including prefix
 * @sha256: SHA256 of the firmware
 */
struct efi_embedded_fw_desc {
	const char *name;
	u8 prefix[8];
	u32 length;
	u8 sha256[32];
};

int efi_get_embedded_fw(const char *name, void **dat, size_t *sz, size_t msize);

#endif
