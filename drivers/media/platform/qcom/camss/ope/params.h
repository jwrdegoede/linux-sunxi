/* SPDX-License-Identifier: GPL-2.0 */
/*
 * ope/params.h
 *
 * CAMSS ISP parameter buffer parser.
 *
 * Wraps the upstream v4l2_isp_params_validate_buffer() validation and adds
 * a dispatch layer: after validation each block is forwarded to a
 * driver-supplied handler.
 *
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#ifndef CAMSS_PARAMS_H
#define CAMSS_PARAMS_H

#include <linux/string.h>
#include <linux/types.h>
#include <media/v4l2-isp.h>
#include <uapi/linux/qcom-camss-config.h>

#define CAMSS_ISP_PARAMS_FMT_INIT \
	{ .fourcc = V4L2_META_FMT_QCOM_ISP_PARAMS, .depth = 8, .align = 0, .num_planes = 1 }

#define CAMSS_ISP_PARAMS_FL_BLOCK_DIRTY   (1U << V4L2_ISP_PARAMS_FL_DRIVER_FLAGS(0))

struct device;
struct vb2_buffer;
struct camss_isp_fmt;

union camss_isp_params_block {
	struct v4l2_isp_params_block_header header;
	struct camss_params_wb_gain         wb_gain;
	struct camss_params_chroma_enhan   chroma_enhan;
	struct camss_params_color_correct  color_correct;
	struct camss_params_gamma          gamma;
};

typedef void (*camss_isp_params_handler_fn)(void *priv, const union camss_isp_params_block *block);

/**
 * camss_isp_params_copy_block - copy one validated block into driver state
 *
 * @dst:   destination block in the driver's shadow configuration
 * @block: validated source block
 * @size:  size of the destination block
 *
 * Returns true if the payload was copied, false if only the header was.
 */
static inline bool camss_isp_params_copy_block(void *dst,
					       const union camss_isp_params_block *block,
					       size_t size)
{
	struct v4l2_isp_params_block_header *hdr = dst;
	bool header_only = block->header.size == sizeof(block->header);

	if (header_only)
		*hdr = block->header;
	else
		memcpy(dst, block, size);

	hdr->flags |= CAMSS_ISP_PARAMS_FL_BLOCK_DIRTY;

	return !header_only;
}

/**
 * camss_isp_params_apply - validate and dispatch a params buffer
 *
 * @dev:          device for error logging
 * @vb:           the vb2 buffer (used for size validation)
 * @scratch:      kernel-only bounce buffer, at least
 *                v4l2_isp_params_buffer_size(CAMSS_PARAMS_OPE_MAX_PAYLOAD) bytes
 * @type_info:    per-block-type validation info, indexed by block type
 * @handlers:     per-block-type handlers, indexed by block type
 * @num_handlers: number of entries in @type_info and @handlers
 * @priv:         opaque pointer forwarded to each handler
 *
 * Copies the buffer payload into @scratch, calls
 * v4l2_isp_params_validate_buffer_size(), then
 * v4l2_isp_params_validate_buffer(), then walks the validated block stream
 * dispatching each block to its handler.  Validation and dispatch operate
 * only on @scratch, never on the userspace-visible mapping.
 *
 * Returns 0 on success, negative errno on validation failure.
 */
int camss_isp_params_apply(struct device *dev,
			   struct vb2_buffer *vb,
			   struct v4l2_isp_params_buffer *scratch,
			   const struct v4l2_isp_params_block_type_info *type_info,
			   const camss_isp_params_handler_fn *handlers,
			   unsigned int num_handlers,
			   void *priv);

#endif /* CAMSS_PARAMS_H */
