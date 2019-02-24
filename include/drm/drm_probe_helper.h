// SPDX-License-Identifier: GPL-2.0 OR MIT

#ifndef __DRM_PROBE_HELPER_H__
#define __DRM_PROBE_HELPER_H__

#include <linux/types.h>

struct drm_connector;
struct drm_device;
struct drm_modeset_acquire_ctx;

int drm_helper_probe_single_connector_modes(struct drm_connector
					    *connector, uint32_t maxX,
					    uint32_t maxY);
int drm_helper_probe_detect(struct drm_connector *connector,
			    struct drm_modeset_acquire_ctx *ctx,
			    bool force);
void drm_kms_helper_poll_init(struct drm_device *dev);
void drm_kms_helper_poll_fini(struct drm_device *dev);
bool drm_helper_hpd_irq_event(struct drm_device *dev);
void drm_kms_helper_hotplug_event(struct drm_device *dev);

void drm_kms_helper_poll_disable(struct drm_device *dev);
void drm_kms_helper_poll_enable(struct drm_device *dev);
bool drm_kms_helper_is_poll_worker(void);

/**
 * enum drm_kms_oob_hotplug_event - out-of-band hotplug events
 * @DRM_OOB_HOTPLUG_TYPE_C_DP: DisplayPort over Type-C hotplug event
 */
enum drm_kms_oob_hotplug_event {
	DRM_OOB_HOTPLUG_TYPE_C_DP = 0,
};

int drm_kms_register_oob_hotplug_notifier(struct notifier_block *nb);
int drm_kms_unregister_oob_hotplug_notifier(struct notifier_block *nb);
int drm_kms_call_oob_hotplug_notifier_chain(unsigned long event);

#endif
