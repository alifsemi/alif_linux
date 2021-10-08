/*
 * cdc_kms.c  --  CDC Display Controller Mode Setting
 *
 * Copyright (C) 2017 TES Electronic Solutions GmbH
 * Author: Christian Thaler <christian.thaler@tes-dst.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/clk.h>
#include <linux/mutex.h>

#include <drm/drmP.h>
#include <drm/drm_crtc.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_fb_cma_helper.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_probe_helper.h>

#include <video/display_timing.h>
#include <video/of_display_timing.h>
#include <video/videomode.h>

#include <linux/of_graph.h>
#include <linux/wait.h>

#include "cdc_regs.h"
#include "cdc_drv.h"
#include "cdc_kms.h"
#include "cdc_crtc.h"
#include "cdc_plane.h"
#include "cdc_encoder.h"

/*******************************************************************************
 * Format helper
 *
 * Note that the format id is configuration dependent!
 */
static const struct cdc_format cdc_formats[] = {
	{ 0, DRM_FORMAT_ARGB8888, 32 },
	{ 0, DRM_FORMAT_XRGB8888, 32 },
	{ 1, DRM_FORMAT_RGB888, 24 },
	{ 2, DRM_FORMAT_RGB565, 16 },
	{ 3, DRM_FORMAT_ARGB4444, 16 },
	{ 4, DRM_FORMAT_ARGB1555, 16 },
};

const struct cdc_format *
cdc_format_info(__u32 drm_fourcc)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(cdc_formats); ++i) {
		if (cdc_formats[i].fourcc == drm_fourcc)
			return &cdc_formats[i];
	}
	return NULL;
}

static struct drm_framebuffer *cdc_fb_create(struct drm_device *dev,
	struct drm_file *file_priv, const struct drm_mode_fb_cmd2 *mode_cmd)
{
	struct drm_framebuffer *fb;
	struct drm_gem_cma_object *gem;
	const struct cdc_format *format;

	dev_dbg(dev->dev, "creating frame buffer %dx%d (%08x)\n",
		mode_cmd->width, mode_cmd->height, mode_cmd->pixel_format);

	format = cdc_format_info(mode_cmd->pixel_format);
	if (format == NULL) {
		dev_err(dev->dev, "requested unsupported pixel format %08x\n",
			mode_cmd->pixel_format);
		return ERR_PTR(-EINVAL);
	}

	if (mode_cmd->pitches[0] >= CDC_MAX_PITCH) {
		dev_err(dev->dev, "requested too large pitch of %u\n",
			mode_cmd->pitches[0]);
		return ERR_PTR(-EINVAL);
	}

	fb = drm_gem_fb_create(dev, file_priv, mode_cmd);

	gem = drm_fb_cma_get_gem_obj(fb, 0);

	dev_dbg(dev->dev, "FB addr is 0x%08x\n", gem->paddr);

	return fb;
}

#if 0
static void cdc_output_poll_changed(struct drm_device *dev)
{
	struct cdc_device *cdc = dev->dev_private;

	dev_dbg(dev->dev, "%s\n", __func__);

	if (!cdc->fbdev)
		cdc->early_poll = true;
	else
		drm_fbdev_cma_hotplug_event(cdc->fbdev);
}
#endif

static int cdc_atomic_check(struct drm_device *dev,
	struct drm_atomic_state *state)
{
	int ret;

	dev_dbg(dev->dev, "%s\n", __func__);

	ret = drm_atomic_helper_check(dev, state);
	if (ret < 0)
		return ret;

	return 0;
}

struct cdc_commit {
	struct work_struct work;
	struct drm_device *dev;
	struct drm_atomic_state *state;
	u32 crtcs;
};

static void cdc_atomic_complete(struct cdc_commit *commit)
{
	struct drm_device *dev = commit->dev;
	struct cdc_device *cdc = dev->dev_private;
	struct drm_atomic_state *old_state = commit->state;

	dev_dbg(dev->dev, "%s\n", __func__);

	/* Apply the atomic update. */
	drm_atomic_helper_commit_modeset_disables(dev, old_state);
	drm_atomic_helper_commit_modeset_enables(dev, old_state);
	/* todo: maybe set DRM_PLANE_COMMIT_ACTIVE_ONLY flag here
	 * This will stop the KMS core from sending plane update notifications
	 * for a disabled CRTC. Check if we manually ignore plane updates in
	 * this case.
	 */
	drm_atomic_helper_commit_planes(dev, old_state, 0);

	drm_atomic_helper_wait_for_vblanks(dev, old_state);

	drm_atomic_helper_cleanup_planes(dev, old_state);

	/* todo: check if we can make use of the reference counting anywhere
	 * Maybe use worker threads for the commit? See:
	 * https://lists.freedesktop.org/archives/intel-gfx-trybot/2016-September/008592.html
	 */
	drm_atomic_state_put(old_state);

	/* Complete the commit, wake up any waiter. */
	spin_lock(&cdc->commit.wait.lock);
	cdc->commit.pending = 0;
	wake_up_all_locked(&cdc->commit.wait);
	spin_unlock(&cdc->commit.wait.lock);

	kfree(commit);
}

static void cdc_atomic_work(struct work_struct *work)
{
	struct cdc_commit
	*commit = container_of(work, struct cdc_commit, work);

	cdc_atomic_complete(commit);
}

static int cdc_atomic_commit(struct drm_device *dev,
	struct drm_atomic_state *state, bool async)
{
	struct cdc_device *cdc = dev->dev_private;
	struct cdc_commit *commit;
	int ret;

	dev_dbg(dev->dev, "%s\n", __func__);

	ret = drm_atomic_helper_prepare_planes(dev, state);
	if (ret)
		return ret;

	/* Allocate the commit object. */
	commit = kzalloc(sizeof(*commit), GFP_KERNEL);
	if (commit == NULL)
		return -ENOMEM;

	INIT_WORK(&commit->work, cdc_atomic_work);
	commit->dev = dev;
	commit->state = state;

	/* Wait until all affected CRTCs have completed previous commits and
	 * mark them as pending.
	 */
	if (state->crtcs[0].ptr)
		commit->crtcs = 1;

	spin_lock(&cdc->commit.wait.lock);
	ret = wait_event_interruptible_locked(cdc->commit.wait,
		!(cdc->commit.pending));
	if (ret == 0)
		cdc->commit.pending = 1;
	spin_unlock(&cdc->commit.wait.lock);

	if (ret) {
		kfree(commit);
		return ret;
	}

	/* Swap the state, this is the point of no return. */
	ret = drm_atomic_helper_swap_state(state, true);
	if (ret) {
		kfree(commit);
		return ret;
	}

	drm_atomic_state_get(state);
	if (async)
		schedule_work(&commit->work);
	else
		cdc_atomic_complete(commit);

	return 0;
}

static const struct drm_mode_config_funcs cdc_mode_config_funcs = {
	.fb_create = cdc_fb_create,
	.output_poll_changed = drm_fb_helper_output_poll_changed,
	.atomic_check = cdc_atomic_check,
	.atomic_commit = cdc_atomic_commit,
};

static int cdc_encoders_find_and_init(struct cdc_device *cdc,
	struct of_endpoint *ep)
{
	__u32 enc_type = DRM_MODE_ENCODER_NONE;
	struct device_node *connector = NULL;
	struct device_node *encoder = NULL;
	struct device_node *ep_node = NULL;
	struct device_node *entity_ep_node;
	struct device_node *entity;
	int ret;

	/*
	 * Locate the connected entity and infer its type from the number of
	 * endpoints.
	 */
	entity = of_graph_get_remote_port_parent(ep->local_node);
	if (!entity) {
		dev_err(cdc->dev, "unconnected endpoint %s, skipping\n",
			ep->local_node->full_name);
		return -ENODEV;
	}
	
	if (!of_device_is_available(entity)) {
		dev_dbg(cdc->dev,
			"connected entity %pOF is disabled, skipping\n",
			entity);
		return -ENODEV;
	}
	
	dev_dbg(cdc->dev, "endpoint is connected to %s\n", entity->full_name);

	entity_ep_node = of_graph_get_remote_endpoint(ep->local_node);

	for_each_endpoint_of_node(entity, ep_node)
	{
		if (ep_node == entity_ep_node)
			continue;

		/*
		 * We've found one endpoint other than the input, this must
		 * be an encoder. Locate the connector.
		 */
		encoder = entity;
		connector = of_graph_get_remote_port_parent(ep_node);
		of_node_put(ep_node);

		if (!connector) {
			dev_warn(cdc->dev,
				 "no connector for encoder %s, skipping\n",
				 encoder->full_name);
			of_node_put(entity_ep_node);
			of_node_put(encoder);
			return -ENODEV;
		}

		break;
	}

	of_node_put(entity_ep_node);

	if (!encoder) {
		/*
		 * If no encoder has been found the entity must be the
		 * connector.
		 */
		connector = entity;
	}

	ret = cdc_encoder_init(cdc, enc_type, encoder, connector);

	if (ret && ret != -EPROBE_DEFER)
		dev_warn(cdc->dev,
			 "failed to initialize encoder %s (%d), skipping\n",
			 encoder->full_name, ret);

	of_node_put(encoder);
	of_node_put(connector);

	return ret;
}

static int cdc_encoders_init(struct cdc_device *cdc)
{
	struct device_node *np = cdc->dev->of_node;
	struct device_node *ep_node;

	dev_dbg(cdc->dev, "initializing encoder for %s\n", np->full_name);

	/*
	 * CDC only has one endpoint. Now create the encoder for it.
	 */
	for_each_endpoint_of_node(np, ep_node) {
		struct of_endpoint ep;
		int ret;

		ret = of_graph_parse_endpoint(ep_node, &ep);
		if (ret < 0) {
			of_node_put(ep_node);
			return ret;
		}

		/* Process output pipeline. */
		ret = cdc_encoders_find_and_init(cdc, &ep);
		if (ret < 0) {
			if (ret == -EPROBE_DEFER) {
				of_node_put(ep_node);
				return ret;
			}

			continue;
		}
	}

	return 0;
}

int cdc_modeset_init(struct cdc_device *cdc)
{
	struct drm_device *dev = cdc->ddev;
	/*struct drm_fbdev_cma *fbdev;*/
	int ret;

	dev_dbg(cdc->dev, "%s\n", __func__);

	drm_mode_config_init(dev);

	dev->mode_config.min_width = 0;
	dev->mode_config.min_height = 0;
	dev->mode_config.max_width = CDC_MAX_WIDTH;
	dev->mode_config.max_height = CDC_MAX_HEIGHT;
	dev->mode_config.funcs = &cdc_mode_config_funcs;

	/* Initialize vertical blanking interrupts handling. Start with vblank
	 * disabled for all CRTCs.
	 */
	ret = drm_vblank_init(dev, 1 /* num_crtcs */);
	if (ret < 0) {
		dev_err(cdc->dev, "failed to initialize vblank\n");
		return ret;
	}

	cdc_planes_init(cdc);

	cdc_crtc_create(cdc);

	cdc_encoders_init(cdc);

	drm_mode_config_reset(dev);
	drm_kms_helper_poll_init(dev);

#if 0
	if (dev->mode_config.num_connector) {
		dev_dbg(cdc->dev, "Initializing FBDEV CMA...\n");
		fbdev = drm_fbdev_cma_init(dev, 32, 1);
		dev_dbg(cdc->dev, "Finished FBDEV CMA init call\n");
		if (IS_ERR(fbdev)) {
			dev_err(cdc->dev,
				"could not initialize fbdev cma...\n");
			return PTR_ERR(fbdev);
		}

		cdc->fbdev = fbdev;

		// handle a poll event that occured before FBDEV was ready
		if (cdc->early_poll)
			drm_fbdev_cma_hotplug_event(fbdev);
	} else {
		dev_err(cdc->dev,
			"no connector found, disabling fbdev emulation\n");
	}

	dev_dbg(cdc->dev, "Added FB at 0x%08x\n",
		cdc->ddev->mode_config.fb_base);
#endif

	return 0;
}
