// SPDX-License-Identifier: GPL-2.0

#include <linux/efi_embedded_fw.h>
#include <linux/property.h>
#include <linux/security.h>
#include <linux/vmalloc.h>

#include "fallback.h"
#include "firmware.h"

int firmware_fallback_platform(struct fw_priv *fw_priv, enum fw_opt opt_flags)
{
#ifdef CONFIG_EFI_EMBEDDED_FIRMWARE
	int rc;

	if (!(opt_flags & FW_OPT_FALLBACK_PLATFORM))
		return -ENOENT;

	rc = security_kernel_load_data(LOADING_FIRMWARE_EFI_EMBEDDED);
	if (rc)
		return rc;

	rc = efi_get_embedded_fw(fw_priv->fw_name, &fw_priv->data,
				 &fw_priv->size);
	if (rc)
		return rc; /* rc == -ENOENT when the fw was not found */

	fw_state_done(fw_priv);
	return 0;
#else
	return -ENOENT;
#endif
}
