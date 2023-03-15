// SPDX-License-Identifier: GPL-2.0-only
// Copyright (C) 2013 - 2023 Intel Corporation

#include <linux/media-bus-format.h>
#include <linux/types.h>
#include <linux/videodev2.h>
#include <media/media-entity.h>

#include "ipu6.h"
#include "ipu6-bus.h"
#include "ipu6-isys.h"
#include "ipu6-isys-subdev.h"
#include "ipu6-isys-video.h"

unsigned int ipu6_isys_mbus_code_to_bpp(u32 code)
{
	switch (code) {
	case MEDIA_BUS_FMT_RGB888_1X24:
		return 24;
	case MEDIA_BUS_FMT_RGB565_1X16:
	case MEDIA_BUS_FMT_UYVY8_1X16:
	case MEDIA_BUS_FMT_YUYV8_1X16:
		return 16;
	case MEDIA_BUS_FMT_SBGGR12_1X12:
	case MEDIA_BUS_FMT_SGBRG12_1X12:
	case MEDIA_BUS_FMT_SGRBG12_1X12:
	case MEDIA_BUS_FMT_SRGGB12_1X12:
		return 12;
	case MEDIA_BUS_FMT_SBGGR10_1X10:
	case MEDIA_BUS_FMT_SGBRG10_1X10:
	case MEDIA_BUS_FMT_SGRBG10_1X10:
	case MEDIA_BUS_FMT_SRGGB10_1X10:
		return 10;
	case MEDIA_BUS_FMT_SBGGR8_1X8:
	case MEDIA_BUS_FMT_SGBRG8_1X8:
	case MEDIA_BUS_FMT_SGRBG8_1X8:
	case MEDIA_BUS_FMT_SRGGB8_1X8:
		return 8;
	default:
		WARN_ON(1);
		return 8;
	}
}

unsigned int ipu6_isys_mbus_code_to_mipi(u32 code)
{
	switch (code) {
	case MEDIA_BUS_FMT_RGB565_1X16:
		return MIPI_CSI2_DT_RGB565;
	case MEDIA_BUS_FMT_RGB888_1X24:
		return MIPI_CSI2_DT_RGB888;
	case MEDIA_BUS_FMT_UYVY8_1X16:
	case MEDIA_BUS_FMT_YUYV8_1X16:
		return MIPI_CSI2_DT_YUV422_8B;
	case MEDIA_BUS_FMT_SBGGR12_1X12:
	case MEDIA_BUS_FMT_SGBRG12_1X12:
	case MEDIA_BUS_FMT_SGRBG12_1X12:
	case MEDIA_BUS_FMT_SRGGB12_1X12:
		return MIPI_CSI2_DT_RAW12;
	case MEDIA_BUS_FMT_SBGGR10_1X10:
	case MEDIA_BUS_FMT_SGBRG10_1X10:
	case MEDIA_BUS_FMT_SGRBG10_1X10:
	case MEDIA_BUS_FMT_SRGGB10_1X10:
		return MIPI_CSI2_DT_RAW10;
	case MEDIA_BUS_FMT_SBGGR8_1X8:
	case MEDIA_BUS_FMT_SGBRG8_1X8:
	case MEDIA_BUS_FMT_SGRBG8_1X8:
	case MEDIA_BUS_FMT_SRGGB8_1X8:
		return MIPI_CSI2_DT_RAW8;
	default:
		/* return unavailable MIPI data type - 0x3f */
		WARN_ON(1);
		return 0x3f;
	}
}

bool ipu6_isys_is_bayer_format(u32 code)
{
	switch (ipu6_isys_mbus_code_to_mipi(code)) {
	case MIPI_CSI2_DT_RAW8:
	case MIPI_CSI2_DT_RAW10:
	case MIPI_CSI2_DT_RAW12:
		return true;
	}
	return false;
}

u32 ipu6_isys_convert_bayer_order(u32 code, int x, int y)
{
	static const u32 code_map[] = {
		MEDIA_BUS_FMT_SRGGB8_1X8,
		MEDIA_BUS_FMT_SGRBG8_1X8,
		MEDIA_BUS_FMT_SGBRG8_1X8,
		MEDIA_BUS_FMT_SBGGR8_1X8,
		MEDIA_BUS_FMT_SRGGB10_1X10,
		MEDIA_BUS_FMT_SGRBG10_1X10,
		MEDIA_BUS_FMT_SGBRG10_1X10,
		MEDIA_BUS_FMT_SBGGR10_1X10,
		MEDIA_BUS_FMT_SRGGB12_1X12,
		MEDIA_BUS_FMT_SGRBG12_1X12,
		MEDIA_BUS_FMT_SGBRG12_1X12,
		MEDIA_BUS_FMT_SBGGR12_1X12,
	};
	u32 i;

	for (i = 0; i < ARRAY_SIZE(code_map); i++)
		if (code_map[i] == code)
			break;

	if (i == ARRAY_SIZE(code_map)) {
		WARN_ON(1);
		return code;
	}

	return code_map[i ^ (((y & 1) << 1) | (x & 1))];
}

int ipu6_isys_subdev_set_fmt(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state,
			     struct v4l2_subdev_format *format)
{
	struct ipu6_isys_subdev *asd = to_ipu6_isys_subdev(sd);
	struct v4l2_mbus_framefmt *fmt;
	struct v4l2_rect *crop;
	u32 code = asd->supported_codes[0];
	u32 other_pad, other_stream;
	unsigned int i;
	int ret;

	/* No transcoding, source and sink formats must match. */
	if ((sd->entity.pads[format->pad].flags & MEDIA_PAD_FL_SOURCE) &&
	    sd->entity.num_pads > 1) {
		return v4l2_subdev_get_fmt(sd, state, format);
	}
	format->format.width = clamp(format->format.width, IPU6_ISYS_MIN_WIDTH,
				     IPU6_ISYS_MAX_WIDTH);
	format->format.height = clamp(format->format.height,
				      IPU6_ISYS_MIN_HEIGHT,
				      IPU6_ISYS_MAX_HEIGHT);

	for (i = 0; asd->supported_codes[i]; i++) {
		if (asd->supported_codes[i] == format->format.code) {
			code = asd->supported_codes[i];
			break;
		}
	}
	format->format.code = code;
	format->format.field = V4L2_FIELD_NONE;

	/* Store the format and propagate it to the source pad. */
	fmt = v4l2_subdev_state_get_stream_format(state, format->pad,
						  format->stream);
	if (!fmt)
		return -EINVAL;

	*fmt = format->format;

	if (!(sd->entity.pads[format->pad].flags & MEDIA_PAD_FL_SINK))
		return 0;

	/* propagate format to following source pad */
	fmt = v4l2_subdev_state_get_opposite_stream_format(state, format->pad,
							   format->stream);
	if (!fmt)
		return -EINVAL;

	*fmt = format->format;

	ret = v4l2_subdev_routing_find_opposite_end(&state->routing,
						    format->pad,
						    format->stream,
						    &other_pad,
						    &other_stream);
	if (ret)
		return -EINVAL;

	crop = v4l2_subdev_state_get_stream_crop(state, other_pad,
						 other_stream);
	/* reset crop */
	crop->left = 0;
	crop->top = 0;
	crop->width = fmt->width;
	crop->height = fmt->height;

	return 0;
}

int ipu6_isys_subdev_enum_mbus_code(struct v4l2_subdev *sd,
				    struct v4l2_subdev_state *state,
				    struct v4l2_subdev_mbus_code_enum *code)
{
	struct ipu6_isys_subdev *asd = to_ipu6_isys_subdev(sd);
	const u32 *supported_codes = asd->supported_codes;
	u32 index;

	for (index = 0; supported_codes[index]; index++) {
		if (index == code->index) {
			code->code = supported_codes[index];
			return 0;
		}
	}

	return -EINVAL;
}

static int subdev_set_routing(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *state,
			      struct v4l2_subdev_krouting *routing)
{
	static const struct v4l2_mbus_framefmt format = {
		.width = 4096,
		.height = 3072,
		.code = MEDIA_BUS_FMT_SGRBG10_1X10,
		.field = V4L2_FIELD_NONE,
	};
	int ret;

	ret = v4l2_subdev_routing_validate(sd, routing,
					   V4L2_SUBDEV_ROUTING_ONLY_1_TO_1);
	if (ret)
		return ret;

	ret = v4l2_subdev_set_routing_with_fmt(sd, state, routing, &format);
	if (ret)
		return ret;

	return 0;
}

int ipu6_isys_get_stream_pad_fmt(struct v4l2_subdev *sd, u32 pad, u32 stream,
				 struct v4l2_mbus_framefmt *format)
{
	struct v4l2_mbus_framefmt *fmt;
	struct v4l2_subdev_state *state;
	int ret = -EINVAL;

	if (!sd || !format)
		return -EINVAL;

	state = v4l2_subdev_lock_and_get_active_state(sd);
	fmt = v4l2_subdev_state_get_stream_format(state, pad, stream);
	if (fmt) {
		*format = *fmt;
		ret = 0;
	}
	v4l2_subdev_unlock_state(state);

	return ret;
}

int ipu6_isys_get_stream_pad_crop(struct v4l2_subdev *sd, u32 pad, u32 stream,
				  struct v4l2_rect *crop)
{
	struct v4l2_subdev_state *state;
	struct v4l2_rect *rect;
	int ret = -EINVAL;

	if (!sd || !crop)
		return -EINVAL;

	state = v4l2_subdev_lock_and_get_active_state(sd);
	rect = v4l2_subdev_state_get_stream_crop(state, pad, stream);
	if (rect) {
		*crop = *rect;
		ret = 0;
	}
	v4l2_subdev_unlock_state(state);

	return ret;
}

u32 ipu6_isys_get_src_stream_by_src_pad(struct v4l2_subdev *sd, u32 pad)
{
	struct v4l2_subdev_state *state;
	struct v4l2_subdev_route *routes;
	unsigned int i;
	u32 source_stream = 0;

	state = v4l2_subdev_lock_and_get_active_state(sd);
	if (!state)
		return 0;

	routes = state->routing.routes;
	for (i = 0; i < state->routing.num_routes; i++) {
		if (routes[i].source_pad == pad) {
			source_stream = routes[i].source_stream;
			break;
		}
	}

	v4l2_subdev_unlock_state(state);

	return source_stream;
}

int ipu6_isys_subdev_init_cfg(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *state)
{
	struct v4l2_subdev_route route = {
		.sink_pad = 0,
		.sink_stream = 0,
		.source_pad = 1,
		.source_stream = 0,
		.flags = V4L2_SUBDEV_ROUTE_FL_ACTIVE,
	};
	struct v4l2_subdev_krouting routing = {
		.num_routes = 1,
		.routes = &route,
	};

	return subdev_set_routing(sd, state, &routing);
}

int ipu6_isys_subdev_set_routing(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 enum v4l2_subdev_format_whence which,
				 struct v4l2_subdev_krouting *routing)
{
	return subdev_set_routing(sd, state, routing);
}

int ipu6_isys_subdev_init(struct ipu6_isys_subdev *asd,
			  const struct v4l2_subdev_ops *ops,
			  unsigned int nr_ctrls,
			  unsigned int num_pads)
{
	int ret;

	v4l2_subdev_init(&asd->sd, ops);

	asd->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
			 V4L2_SUBDEV_FL_HAS_EVENTS |
			 V4L2_SUBDEV_FL_STREAMS;
	asd->sd.owner = THIS_MODULE;
	asd->sd.entity.function = MEDIA_ENT_F_VID_IF_BRIDGE;

	asd->pad = devm_kcalloc(&asd->isys->adev->auxdev.dev, num_pads,
				sizeof(*asd->pad), GFP_KERNEL);

	if (!asd->pad)
		return -ENOMEM;

	ret = media_entity_pads_init(&asd->sd.entity, num_pads, asd->pad);
	if (ret)
		return ret;

	if (asd->ctrl_init) {
		ret = v4l2_ctrl_handler_init(&asd->ctrl_handler, nr_ctrls);
		if (ret)
			goto out_media_entity_cleanup;

		asd->ctrl_init(&asd->sd);
		if (asd->ctrl_handler.error) {
			ret = asd->ctrl_handler.error;
			goto out_v4l2_ctrl_handler_free;
		}

		asd->sd.ctrl_handler = &asd->ctrl_handler;
	}

	asd->source = -1;

	return 0;

out_v4l2_ctrl_handler_free:
	v4l2_ctrl_handler_free(&asd->ctrl_handler);

out_media_entity_cleanup:
	media_entity_cleanup(&asd->sd.entity);

	return ret;
}

void ipu6_isys_subdev_cleanup(struct ipu6_isys_subdev *asd)
{
	media_entity_cleanup(&asd->sd.entity);
	v4l2_ctrl_handler_free(&asd->ctrl_handler);
}
