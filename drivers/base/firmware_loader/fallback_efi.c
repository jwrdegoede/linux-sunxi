// SPDX-License-Identifier: GPL-2.0

#include <linux/efi_embedded_fw.h>
#include <linux/property.h>
#include <linux/security.h>
#include <linux/vmalloc.h>

#include "fallback.h"
#include "firmware.h"

int fw_get_efi_embedded_fw(struct device *dev, struct fw_priv *fw_priv,
			   enum fw_opt *opt_flags, int ret)
{
	size_t size, max = INT_MAX;
	bool free_on_err = true;
	int rc;

	if (!dev)
		return ret;

	if (!device_property_read_bool(dev, "efi-embedded-firmware"))
		return ret;

	*opt_flags |= FW_OPT_NO_WARN | FW_OPT_NOCACHE | FW_OPT_NOFALLBACK;

	rc = security_kernel_read_file(NULL, READING_FIRMWARE_EFI_EMBEDDED);
	if (rc)
		return rc;

	/* Already populated data member means we're loading into a buffer */
	if (fw_priv->data) {
		max = fw_priv->allocated_size;
		free_on_err = false;
	}

	rc = efi_get_embedded_fw(fw_priv->fw_name, &fw_priv->data, &size, max);
	if (rc) {
		dev_warn(dev, "Firmware %s not in EFI\n", fw_priv->fw_name);
		return ret;
	}

	rc = security_kernel_post_read_file(NULL, fw_priv->data, size,
					    READING_FIRMWARE_EFI_EMBEDDED);
	if (rc) {
		if (free_on_err) {
			vfree(fw_priv->data);
			fw_priv->data = NULL;
		}
		return rc;
	}

	dev_dbg(dev, "using efi-embedded fw %s\n", fw_priv->fw_name);
	fw_priv->size = size;
	fw_state_done(fw_priv);
	return 0;
}
