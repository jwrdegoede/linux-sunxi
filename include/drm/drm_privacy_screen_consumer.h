/* SPDX-License-Identifier: MIT */
/*
 * Copyright (C) 2020 Red Hat, Inc.
 *
 * Authors:
 * Hans de Goede <hdegoede@redhat.com>
 */

#ifndef __DRM_PRIVACY_SCREEN_CONSUMER_H__
#define __DRM_PRIVACY_SCREEN_CONSUMER_H__

#include <linux/device.h>
#include <drm/drm_connector.h>

struct drm_privacy_screen;

struct drm_privacy_screen *drm_privacy_screen_get(struct device *dev,
						  const char *con_id);
void drm_privacy_screen_put(struct drm_privacy_screen *priv);

int drm_privacy_screen_set_sw_state(struct drm_privacy_screen *priv,
				    enum drm_privacy_screen_status sw_state);
void drm_privacy_screen_get_state(struct drm_privacy_screen *priv,
				  enum drm_privacy_screen_status *sw_state_ret,
				  enum drm_privacy_screen_status *hw_state_ret);

int drm_privacy_screen_register_notifier(struct drm_privacy_screen *priv,
					 struct notifier_block *nb);
int drm_privacy_screen_unregister_notifier(struct drm_privacy_screen *priv,
					   struct notifier_block *nb);

#endif
