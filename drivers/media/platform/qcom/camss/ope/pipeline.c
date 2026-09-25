// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * CAMSS ISP pipeline helper — declarative MC topology builder
 *
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#include <linux/slab.h>

#include <media/media-device.h>
#include <media/media-entity.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include "pipeline.h"

/* -------- Internal helpers -------- */

static enum vfl_devnode_direction isp_caps_to_vfl_dir(u32 caps)
{
	if (caps & (V4L2_CAP_VIDEO_M2M | V4L2_CAP_VIDEO_M2M_MPLANE))
		return VFL_DIR_M2M;
	if (caps & (V4L2_CAP_VIDEO_OUTPUT | V4L2_CAP_VIDEO_OUTPUT_MPLANE |
		    V4L2_CAP_META_OUTPUT | V4L2_CAP_VBI_OUTPUT | V4L2_CAP_SDR_OUTPUT))
		return VFL_DIR_TX;
	return VFL_DIR_RX;
}

static unsigned int isp_count_pads(const struct camss_isp_pad_desc *pads)
{
	unsigned int n = 0;

	if (!pads)
		return 0;
	while (pads[n].flags)
		n++;
	return n;
}

static struct media_entity *isp_pipeline_media_entity(struct camss_isp_pipeline *pipeline,
						      unsigned int idx)
{
	struct camss_isp_pipeline_entity *slot = &pipeline->entities[idx];

	switch (slot->obj_type) {
	case MEDIA_ENTITY_TYPE_VIDEO_DEVICE:
		return &slot->vdev.entity;
	case MEDIA_ENTITY_TYPE_V4L2_SUBDEV:
		return &slot->subdev.entity;
	default:
		return &slot->entity;
	}
}

/* -------- Validation -------- */

static int isp_pipeline_validate(struct device *dev,
				 const struct camss_isp_entity_desc *descs,
				 unsigned int num_entities)
{
	unsigned int i, pi;

	for (i = 0; i < num_entities; i++) {
		const struct camss_isp_pad_desc *pads = descs[i].pads;
		unsigned int num_pads = isp_count_pads(pads);

		for (pi = 0; pi < num_pads; pi++) {
			const struct camss_isp_pad_desc *pad = &pads[pi];
			const struct camss_isp_pad_desc *peer_pad;
			unsigned int peer_num_pads;
			int peer_ent = pad->peer_entity;

			if (peer_ent < 0)
				continue;

			if ((unsigned int)peer_ent >= num_entities) {
				dev_err(dev, "entity[%u].p%u: peer_entity %d out of range\n",
					i, pi, peer_ent);
				return -EINVAL;
			}

			peer_num_pads = isp_count_pads(descs[peer_ent].pads);
			if (pad->peer_pad >= peer_num_pads) {
				dev_err(dev, "entity[%u].p%u: peer_pad %u out of range\n",
					i, pi, pad->peer_pad);
				return -EINVAL;
			}

			peer_pad = &descs[peer_ent].pads[pad->peer_pad];

			/* Links are SOURCE->SINK; reject SOURCE->SOURCE or SINK->SINK */
			if (((pad->flags & MEDIA_PAD_FL_SOURCE) &&
			     (peer_pad->flags & MEDIA_PAD_FL_SOURCE)) ||
			    ((pad->flags & MEDIA_PAD_FL_SINK) &&
			     (peer_pad->flags & MEDIA_PAD_FL_SINK))) {
				dev_err(dev, "entity[%u].p%u -> entity[%d].p%u: invalid\n",
					i, pi, peer_ent, pad->peer_pad);
				return -EINVAL;
			}

			/* A link must be described symmetrically from both ends */
			if ((unsigned int)peer_pad->peer_entity != i ||
			    peer_pad->peer_pad != pi) {
				dev_err(dev, "entity[%u].p%u <-> entity[%d].p%u: mismatch\n",
					i, pi, peer_ent, pad->peer_pad);
				return -EINVAL;
			}
		}
	}

	return 0;
}

/* -------- Allocation / Release -------- */

struct camss_isp_pipeline *camss_isp_pipeline_alloc(unsigned int num_entities)
{
	struct camss_isp_pipeline *pipeline;

	pipeline = kzalloc(struct_size(pipeline, entities, num_entities),
			   GFP_KERNEL);
	if (!pipeline)
		return ERR_PTR(-ENOMEM);

	pipeline->num_entities = num_entities;
	mutex_init(&pipeline->vdev_lock);
	return pipeline;
}

void camss_isp_pipeline_free(struct camss_isp_pipeline *pipeline)
{
	unsigned int i;

	if (!pipeline)
		return;

	/* Release pads and subdevs state */
	for (i = 0; i < pipeline->num_entities; i++) {
		struct camss_isp_pipeline_entity *slot = &pipeline->entities[i];

		if (slot->obj_type == MEDIA_ENTITY_TYPE_V4L2_SUBDEV)
			v4l2_subdev_cleanup(&slot->subdev);

		kfree(slot->pads);
		slot->pads = NULL;
	}

	mutex_destroy(&pipeline->vdev_lock);
	kfree(pipeline);
}

/* -------- Registration -------- */

void camss_isp_pipeline_unregister(struct camss_isp_pipeline *pipeline)
{
	int i;

	/* Unregister entities in reverse order. */
	for (i = (int)pipeline->num_entities - 1; i >= 0; i--) {
		struct camss_isp_pipeline_entity *slot = &pipeline->entities[i];

		if (slot->registered) {
			switch (slot->obj_type) {
			case MEDIA_ENTITY_TYPE_VIDEO_DEVICE:
				if (video_device_pipeline(&slot->vdev))
					video_device_pipeline_stop(&slot->vdev);
				video_unregister_device(&slot->vdev);
				break;
			case MEDIA_ENTITY_TYPE_V4L2_SUBDEV:
				v4l2_device_unregister_subdev(&slot->subdev);
				break;
			case MEDIA_ENTITY_TYPE_BASE:
				media_entity_remove_links(&slot->entity);
				media_device_unregister_entity(&slot->entity);
				slot->entity.name = NULL;
				break;
			}

			slot->registered = false;
		}
	}

	pipeline->v4l2_dev = NULL;
}

static int isp_register_vdev(struct camss_isp_pipeline *pipeline,
			     struct camss_isp_pipeline_entity *slot,
			     const struct camss_isp_entity_desc *desc,
			     struct v4l2_device *v4l2_dev)
{
	struct video_device *vdev = &slot->vdev;
	int ret;

	if (!desc->name || !desc->name[0] || !desc->vdev.fops)
		return -EINVAL;

	strscpy(vdev->name, desc->name, sizeof(vdev->name));
	vdev->vfl_dir     = isp_caps_to_vfl_dir(desc->vdev.caps);
	vdev->v4l2_dev    = v4l2_dev;
	vdev->device_caps = desc->vdev.caps | V4L2_CAP_IO_MC;
	vdev->release     = video_device_release_empty;
	vdev->fops        = desc->vdev.fops;
	vdev->lock        = &pipeline->vdev_lock;
	if (desc->vdev.ioctl_ops)
		vdev->ioctl_ops = desc->vdev.ioctl_ops;
	if (desc->vdev.entity_ops)
		vdev->entity.ops = desc->vdev.entity_ops;

	vdev->entity.obj_type = MEDIA_ENTITY_TYPE_VIDEO_DEVICE;
	vdev->entity.function = desc->function ? desc->function : MEDIA_ENT_F_IO_V4L;

	ret = media_entity_pads_init(&vdev->entity, slot->num_pads, slot->pads);
	if (ret)
		return ret;

	video_set_drvdata(vdev, desc->vdev.drvdata);

	ret = video_register_device(vdev, VFL_TYPE_VIDEO, -1);
	if (ret)
		return ret;

	return 0;
}

static int isp_register_subdev(struct camss_isp_pipeline_entity *slot,
			       const struct camss_isp_entity_desc *desc,
			       struct v4l2_device *v4l2_dev)
{
	struct v4l2_subdev *sd = &slot->subdev;
	int ret;

	if (!desc->name || !desc->name[0] || !desc->subdev.ops)
		return -EINVAL;

	v4l2_subdev_init(sd, desc->subdev.ops);
	strscpy(sd->name, desc->name, sizeof(sd->name));
	sd->entity.function = desc->function ?
			      desc->function : MEDIA_ENT_F_V4L2_SUBDEV_UNKNOWN;
	/* Create a /dev/v4l-subdevN node so userspace can query pad formats */
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	if (desc->subdev.internal_ops)
		sd->internal_ops = desc->subdev.internal_ops;
	if (desc->subdev.entity_ops)
		sd->entity.ops = desc->subdev.entity_ops;

	ret = media_entity_pads_init(&sd->entity, slot->num_pads, slot->pads);
	if (ret)
		return ret;

	ret = v4l2_subdev_init_finalize(sd);
	if (ret)
		return ret;

	ret = v4l2_device_register_subdev(v4l2_dev, sd);
	if (ret) {
		v4l2_subdev_cleanup(sd);
		return ret;
	}

	return 0;
}

static int isp_register_base_entity(struct camss_isp_pipeline_entity *slot,
				    const struct camss_isp_entity_desc *desc,
				    struct v4l2_device *v4l2_dev)
{
	struct media_entity *entity = &slot->entity;
	int ret;

	if (!desc->name || !desc->name[0])
		return -EINVAL;

	entity->obj_type = MEDIA_ENTITY_TYPE_BASE;
	entity->name     = desc->name;
	entity->function = desc->function;

	ret = media_entity_pads_init(entity, slot->num_pads, slot->pads);
	if (ret) {
		entity->name = NULL;
		return ret;
	}

	ret = media_device_register_entity(v4l2_dev->mdev, entity);
	if (ret) {
		entity->name = NULL;
		return ret;
	}

	return 0;
}

static int isp_alloc_pads(struct camss_isp_pipeline_entity *slot,
			  const struct camss_isp_entity_desc *desc)
{
	unsigned int num_pads = isp_count_pads(desc->pads);
	unsigned int i;

	if (!num_pads)
		goto done;

	slot->pads = kcalloc(num_pads, sizeof(*slot->pads), GFP_KERNEL);
	if (!slot->pads)
		return -ENOMEM;

	for (i = 0; i < num_pads; i++)
		slot->pads[i].flags = desc->pads[i].flags;
done:
	slot->num_pads = num_pads;
	return 0;
}

int camss_isp_pipeline_register(struct camss_isp_pipeline *pipeline,
				struct v4l2_device *v4l2_dev,
				const struct camss_isp_entity_desc *descs,
				unsigned int num_entities)
{
	unsigned int i, pi;
	int ret;

	if (WARN_ON(num_entities != pipeline->num_entities))
		return -EINVAL;

	if (WARN_ON(!v4l2_dev || !v4l2_dev->mdev))
		return -EINVAL;

	ret = isp_pipeline_validate(v4l2_dev->dev, descs, num_entities);
	if (ret)
		return ret;

	pipeline->v4l2_dev = v4l2_dev;

	/* Register each entity */
	for (i = 0; i < num_entities; i++) {
		const struct camss_isp_entity_desc *desc = &descs[i];
		struct camss_isp_pipeline_entity *slot = &pipeline->entities[i];

		slot->obj_type = desc->obj_type;

		ret = isp_alloc_pads(slot, desc);
		if (ret)
			goto err_unregister;

		switch (desc->obj_type) {
		case MEDIA_ENTITY_TYPE_VIDEO_DEVICE:
			ret = isp_register_vdev(pipeline, slot, desc, v4l2_dev);
			break;
		case MEDIA_ENTITY_TYPE_V4L2_SUBDEV:
			ret = isp_register_subdev(slot, desc, v4l2_dev);
			break;
		case MEDIA_ENTITY_TYPE_BASE:
		default:
			ret = isp_register_base_entity(slot, desc, v4l2_dev);
			break;
		}
		if (ret)
			goto err_unregister;

		slot->registered = true;
	}

	/* Create links — only from SOURCE side to avoid duplicates */
	for (i = 0; i < num_entities; i++) {
		const struct camss_isp_entity_desc *desc = &descs[i];
		unsigned int num_pads = isp_count_pads(desc->pads);

		for (pi = 0; pi < num_pads; pi++) {
			const struct camss_isp_pad_desc *pad = &desc->pads[pi];
			struct media_entity *src_entity, *sink_entity;
			unsigned int src_pad_idx, sink_pad_idx;
			u32 lflags;

			if (!(pad->flags & MEDIA_PAD_FL_SOURCE))
				continue;
			if (pad->peer_entity < 0)
				continue;

			src_entity   = isp_pipeline_media_entity(pipeline, i);
			sink_entity  = isp_pipeline_media_entity(pipeline,
								 (unsigned int)pad->peer_entity);
			src_pad_idx  = pi;
			sink_pad_idx = pad->peer_pad;

			lflags = pad->link_flags ?
				 pad->link_flags :
				 (MEDIA_LNK_FL_IMMUTABLE | MEDIA_LNK_FL_ENABLED);

			ret = media_create_pad_link(src_entity,  src_pad_idx,
						    sink_entity, sink_pad_idx,
						    lflags);
			if (ret)
				goto err_unregister;
		}
	}

	/* Create /dev/v4l-subdevN nodes for all registered subdevs */
	ret = v4l2_device_register_subdev_nodes(v4l2_dev);
	if (ret)
		goto err_unregister;

	return 0;

err_unregister:
	camss_isp_pipeline_unregister(pipeline);
	return ret;
}
