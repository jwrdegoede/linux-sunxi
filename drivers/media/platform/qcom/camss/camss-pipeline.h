/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * CAMSS ISP pipeline helper — declarative MC topology builder
 *
 * Drivers describe their entire media graph — entities (video devices,
 * subdevs, or base entities), their pads, and the links between them —
 * in a single static descriptor table.  The builder validates the table,
 * allocates and registers all entities, and creates all MC links.
 *
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#ifndef _CAMSS_PIPELINE_H
#define _CAMSS_PIPELINE_H

#include <linux/mutex.h>
#include <media/media-device.h>
#include <media/media-entity.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>

/**
 * struct camss_isp_pad_desc - descriptor for one pad and its optional link
 *
 * @flags:       Pad flags: MEDIA_PAD_FL_SINK, MEDIA_PAD_FL_SOURCE,
 *               MEDIA_PAD_FL_MUST_CONNECT.  A zero @flags value acts as
 *               the sentinel that terminates the pad list.
 * @peer_entity: Index of the peer entity in the descriptor array, or -1
 *               if this pad has no link.
 * @peer_pad:    Pad index on the peer entity to link to.
 * @link_flags:  MC link flags (MEDIA_LNK_FL_*).  Defaults to
 *               MEDIA_LNK_FL_IMMUTABLE | MEDIA_LNK_FL_ENABLED when zero.
 *
 * Links are described from both sides (each endpoint references the other),
 * but the builder only creates each link once — from the SOURCE side.
 */
struct camss_isp_pad_desc {
	u32          flags;
	int          peer_entity;
	unsigned int peer_pad;
	u32          link_flags;
};

/**
 * struct camss_isp_entity_desc - descriptor for one entity in the pipeline
 *
 * @name:      Human-readable entity name (also used as video device name
 *             suffix when @obj_type is MEDIA_ENTITY_TYPE_VIDEO_DEVICE).
 * @obj_type:  MEDIA_ENTITY_TYPE_VIDEO_DEVICE, MEDIA_ENTITY_TYPE_V4L2_SUBDEV,
 *             or MEDIA_ENTITY_TYPE_BASE.
 * @function:  MEDIA_ENT_F_* function identifier.
 * @pads:      Sentinel-terminated (flags == 0) array of pad descriptors.
 *
 * Fields used only for MEDIA_ENTITY_TYPE_VIDEO_DEVICE:
 * @vdev.caps: V4L2_CAP_* device capabilities.
 *             The video device direction (VFL_DIR_RX/TX/M2M) is derived
 *             automatically from @caps by the builder.
 * @vdev.drvdata: Opaque pointer set via video_set_drvdata() after registration.
 * @vdev.fops:      File operations (may be NULL to use kernel defaults).
 * @vdev.ioctl_ops: ioctl operations (may be NULL).
 *
 * Fields used only for MEDIA_ENTITY_TYPE_V4L2_SUBDEV:
 * @subdev.ops: Subdev operations (may be NULL).
 * @subdev.internal_ops: Internal subdev operations (may be NULL).
 */
struct camss_isp_entity_desc {
	const char				*name;
	u32					obj_type;
	u32					function;
	const struct camss_isp_pad_desc		*pads;

	union {
		/* MEDIA_ENTITY_TYPE_VIDEO_DEVICE */
		struct {
			u32					caps;
			void					*drvdata;
			const struct v4l2_file_operations	*fops;
			const struct v4l2_ioctl_ops		*ioctl_ops;
			const struct media_entity_operations	*entity_ops;
		} vdev;
		/* MEDIA_ENTITY_TYPE_V4L2_SUBDEV */
		struct {
			const struct v4l2_subdev_ops		*ops;
			const struct v4l2_subdev_internal_ops	*internal_ops;
			const struct media_entity_operations	*entity_ops;
		} subdev;
	};
};

/**
 * struct camss_isp_pipeline_entity - one registered entity slot
 *
 * Internal to the pipeline; drivers access entities via the accessor helpers.
 *
 * @obj_type: mirrors the descriptor's @obj_type.
 * @pads:     allocated pad array for this entity.
 * @num_pads: number of entries in @pads.
 * @vdev:     valid when @obj_type == MEDIA_ENTITY_TYPE_VIDEO_DEVICE.
 * @subdev:   valid when @obj_type == MEDIA_ENTITY_TYPE_V4L2_SUBDEV.
 * @entity:   valid when @obj_type == MEDIA_ENTITY_TYPE_BASE.
 */
struct camss_isp_pipeline_entity {
	u32			 obj_type;
	struct media_pad	*pads;
	unsigned int		 num_pads;
	union {
		struct video_device  vdev;
		struct v4l2_subdev   subdev;
		struct media_entity  entity;
	};
};

/**
 * struct camss_isp_pipeline - registered ISP pipeline topology
 *
 * Allocate with camss_isp_pipeline_alloc(), register with
 * camss_isp_pipeline_register(), tear down with
 * camss_isp_pipeline_unregister(), free with camss_isp_pipeline_free().
 *
 * @v4l2_dev:     Pointer to the caller-provided V4L2 device.
 * @drv_priv:     Driver-private pointer; not touched by the framework.
 * @num_entities: Number of entries in @entities.
 * @entities:     Per-entity state; flexible array.
 */
struct camss_isp_pipeline {
	struct v4l2_device	*v4l2_dev;
	void			*drv_priv;

	unsigned int		 num_entities;
	struct camss_isp_pipeline_entity entities[] __counted_by(num_entities);
};

/**
 * camss_isp_pipeline_alloc() - allocate a pipeline for @num_entities entities
 *
 * Returns a pointer to the new pipeline or ERR_PTR on failure.
 * Free with camss_isp_pipeline_free() if never registered, or call
 * camss_isp_pipeline_unregister() followed by camss_isp_pipeline_free().
 */
struct camss_isp_pipeline *camss_isp_pipeline_alloc(unsigned int num_entities);

/**
 * camss_isp_pipeline_free() - free an unregistered pipeline
 * @pipeline: pipeline to free (may be NULL)
 */
void camss_isp_pipeline_free(struct camss_isp_pipeline *pipeline);

/**
 * camss_isp_pipeline_register() - validate descriptors and register the graph
 * @pipeline:    pipeline (allocated with camss_isp_pipeline_alloc())
 * @v4l2_dev:    caller-owned and already-registered V4L2 device; its
 *               associated media_device (v4l2_dev->mdev) must also be
 *               initialised and registered before this call.
 * @descs:       array of @num_entities entity descriptors
 * @num_entities: number of entities; must equal pipeline->num_entities
 *
 * Validates the descriptor table (link direction consistency, index bounds),
 * then registers all entities into the provided v4l2_device / media_device
 * and creates all MC pad links.
 *
 * Returns 0 on success or a negative error code.
 */
int camss_isp_pipeline_register(struct camss_isp_pipeline *pipeline,
				struct v4l2_device *v4l2_dev,
				const struct camss_isp_entity_desc *descs,
				unsigned int num_entities);

/**
 * camss_isp_pipeline_unregister() - tear down a registered pipeline
 * @pipeline: pipeline to unregister
 */
void camss_isp_pipeline_unregister(struct camss_isp_pipeline *pipeline);

/**
 * camss_isp_pipeline_get_vdev() - return the video_device for entity @idx
 * @pipeline: registered pipeline
 * @idx:      entity index (must be MEDIA_ENTITY_TYPE_VIDEO_DEVICE)
 *
 * Returns NULL if @idx is out of range or the entity is not a video device.
 */
static inline struct video_device *
camss_isp_pipeline_get_vdev(struct camss_isp_pipeline *pipeline,
			    unsigned int idx)
{
	if (WARN_ON(idx >= pipeline->num_entities))
		return NULL;
	if (WARN_ON(pipeline->entities[idx].obj_type !=
		    MEDIA_ENTITY_TYPE_VIDEO_DEVICE))
		return NULL;
	return &pipeline->entities[idx].vdev;
}

/**
 * camss_isp_pipeline_get_subdev() - return the v4l2_subdev for entity @idx
 * @pipeline: registered pipeline
 * @idx:      entity index (must be MEDIA_ENTITY_TYPE_V4L2_SUBDEV)
 *
 * Returns NULL if @idx is out of range or the entity is not a subdev.
 */
static inline struct v4l2_subdev *
camss_isp_pipeline_get_subdev(struct camss_isp_pipeline *pipeline,
			      unsigned int idx)
{
	if (WARN_ON(idx >= pipeline->num_entities))
		return NULL;
	if (WARN_ON(pipeline->entities[idx].obj_type !=
		    MEDIA_ENTITY_TYPE_V4L2_SUBDEV))
		return NULL;
	return &pipeline->entities[idx].subdev;
}

/**
 * camss_isp_pipeline_get_entity() - return the media_entity for entity @idx
 * @pipeline: registered pipeline
 * @idx:      entity index (must be MEDIA_ENTITY_TYPE_BASE)
 *
 * Returns NULL if @idx is out of range or the entity is not a base entity.
 */
static inline struct media_entity *
camss_isp_pipeline_get_entity(struct camss_isp_pipeline *pipeline,
			      unsigned int idx)
{
	if (WARN_ON(idx >= pipeline->num_entities))
		return NULL;
	if (WARN_ON(pipeline->entities[idx].obj_type !=
		    MEDIA_ENTITY_TYPE_BASE))
		return NULL;
	return &pipeline->entities[idx].entity;
}

#endif /* _CAMSS_PIPELINE_H */
