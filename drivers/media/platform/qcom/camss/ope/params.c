// SPDX-License-Identifier: GPL-2.0
/*
 * ope/params.c
 *
 * CAMSS ISP parameter buffer parser.
 *
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#include <linux/device.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <media/videobuf2-core.h>
#include <media/v4l2-isp.h>

#include "params.h"

int camss_isp_params_apply(struct device *dev,
			   struct vb2_buffer *vb,
			   struct v4l2_isp_params_buffer *scratch,
			   const struct v4l2_isp_params_block_type_info *type_info,
			   const camss_isp_params_handler_fn *handlers,
			   unsigned int num_handlers,
			   void *priv)
{
	const struct v4l2_isp_params_buffer *buf = scratch;
	const void *src;
	unsigned int remaining;
	unsigned int offset = 0;
	int ret;

	ret = v4l2_isp_params_validate_buffer_size(dev, vb,
					v4l2_isp_params_buffer_size(CAMSS_PARAMS_OPE_MAX_PAYLOAD));
	if (ret)
		return ret;

	src = vb2_plane_vaddr(vb, 0);

	if (!src) {
		dev_dbg(dev, "params: buffer has no kernel mapping\n");
		return -EFAULT;
	}

	/*
	 * The payload lives in a vmalloc'd buffer that userspace keeps mapped
	 * while the buffer is queued, so it must not be validated in place:
	 * copy it into kernel-only memory first, then work exclusively on the
	 * copy.  See the v4l2_isp_params_validate_buffer() documentation.
	 */
	memcpy(scratch, src, vb2_get_plane_payload(vb, 0));

	ret = v4l2_isp_params_validate_buffer(dev, vb, buf, type_info, num_handlers);
	if (ret)
		return ret;

	dev_dbg(dev, "params: version=%u data_size=%u\n", buf->version, buf->data_size);

	remaining = buf->data_size;

	while (remaining >= sizeof(struct v4l2_isp_params_block_header)) {
		const union camss_isp_params_block *block =
			(const union camss_isp_params_block *)&buf->data[offset];
		u16 type  = block->header.type;
		u32 bsize = block->header.size;

		if (type < num_handlers && handlers[type])
			handlers[type](priv, block);
		else
			dev_dbg(dev, "params: no handler for block type %u\n", type);

		offset += bsize;
		remaining -= bsize;
	}

	dev_dbg(dev, "params: buffer parsed successfully\n");

	return 0;
}
