/*
 * cdc_crtc.c  --  CDC Display Controller CRTC
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
#include <drm/drmP.h>
#include <drm/drm_crtc.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_plane_helper.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>

#include "cdc_regs.h"
#include "cdc_drv.h"
#include "cdc_kms.h"
#include "cdc_plane.h"
#include "cdc_hw.h"
#include "cdc_hw_helpers.h"
#include "cdc_deswizzle.h"

static struct cdc_device *to_cdc_dev (struct drm_crtc *c)
{
	return container_of(c, struct cdc_device, crtc);
}

static void cdc_crtc_set_display_timing (struct drm_crtc *crtc)
{
	struct cdc_device *cdc = to_cdc_dev(crtc);
	const struct drm_display_mode *mode = &crtc->state->adjusted_mode;
	bool neg_hsync, neg_vsync, neg_blank, inv_clock;

	dev_dbg(cdc->dev, "%s\n", __func__);

	dev_dbg(cdc->dev, "SETTING UP TIMING:\n");
	dev_dbg(cdc->dev, "\thorizontal:\n");
	dev_dbg(cdc->dev, "\t\tclock: %d kHz\n", mode->crtc_clock);
	dev_dbg(cdc->dev, "\t\twidth: %d\n", mode->crtc_hdisplay);
	dev_dbg(cdc->dev, "\t\thsync_len: %d\n",
		mode->crtc_hsync_end - mode->crtc_hsync_start);
	dev_dbg(cdc->dev, "\t\thbackporch: %d\n",
		mode->crtc_hblank_end - mode->crtc_hsync_end);
	dev_dbg(cdc->dev, "\t\thfrontporch: %d\n",
		mode->crtc_hsync_start - mode->crtc_hdisplay);
	dev_dbg(cdc->dev, "\tvertical:\n");
	dev_dbg(cdc->dev, "\t\theight: %d\n", mode->crtc_vdisplay);
	dev_dbg(cdc->dev, "\t\tvsync_len: %d\n",
		mode->crtc_vsync_end - mode->crtc_vsync_start);
	dev_dbg(cdc->dev, "\t\tvbackporch: %d\n",
		mode->crtc_vblank_end - mode->crtc_vsync_end);
	dev_dbg(cdc->dev, "\t\tvfrontporch: %d\n",
		mode->crtc_vsync_start - mode->crtc_vdisplay);

	neg_hsync = (mode->flags & DRM_MODE_FLAG_NHSYNC) ? true : false;
	neg_vsync = (mode->flags & DRM_MODE_FLAG_NVSYNC) ? true : false;
	neg_blank = cdc->neg_blank;
	inv_clock = cdc->neg_pixclk;

	dev_dbg(cdc->dev, "\tflags:\n");
	dev_dbg(cdc->dev, "\t\thsync polarity:       %s\n", neg_hsync ? "neg" : "pos");
	dev_dbg(cdc->dev, "\t\tvsync polarity:       %s\n", neg_vsync ? "neg" : "pos");
	dev_dbg(cdc->dev, "\t\tblank polarity:       %s\n", neg_blank ? "neg" : "pos");
	dev_dbg(cdc->dev, "\t\tpixel clock polarity: %s\n", inv_clock ? "neg" : "pos");

	cdc_hw_setTiming(cdc,
		mode->crtc_hsync_end - mode->crtc_hsync_start, // hsync
		mode->crtc_hblank_end - mode->crtc_hsync_end,  // hback porch
		mode->crtc_hdisplay,                           // hwidth
		mode->crtc_hsync_start - mode->crtc_hdisplay,  // hfront porch
		mode->crtc_vsync_end - mode->crtc_vsync_start, // vsync
		mode->crtc_vblank_end - mode->crtc_vsync_end,  // vback porch
		mode->crtc_vdisplay,                           // vwidth
		mode->crtc_vsync_start - mode->crtc_vdisplay,  // vfront porch
		neg_hsync, neg_hsync, neg_blank, inv_clock);

	clk_set_rate(cdc->pclk, mode->crtc_clock * 1000);
}

void cdc_crtc_cancel_page_flip (struct drm_crtc *crtc, struct drm_file *file)
{
	struct drm_pending_vblank_event *event;
	struct drm_device *dev = crtc->dev;
	struct cdc_device *cdc = dev->dev_private;
	unsigned long flags;

	/* Destroy the pending vertical blanking event associated with the
	 * pending page flip, if any, and disable vertical blanking interrupts.
	 */
	spin_lock_irqsave(&dev->event_lock, flags);
	event = cdc->event;
	if (event && event->base.file_priv == file) {
		cdc->event = NULL;
		drm_event_cancel_free(dev, &event->base);
		drm_crtc_vblank_put(crtc);
	}
	spin_unlock_irqrestore(&dev->event_lock, flags);
}

static void cdc_crtc_finish_page_flip (struct drm_crtc *crtc)
{
	struct drm_pending_vblank_event *event;
	struct drm_device *dev = crtc->dev;
	struct cdc_device *cdc = dev->dev_private;
	unsigned long flags;

	spin_lock_irqsave(&dev->event_lock, flags);
	event = cdc->event;
	cdc->event = NULL;
	spin_unlock_irqrestore(&dev->event_lock, flags);

	if (event == NULL)
		return;

	spin_lock_irqsave(&dev->event_lock, flags);
	drm_crtc_send_vblank_event(crtc, event);
	wake_up(&cdc->flip_wait);
	spin_unlock_irqrestore(&dev->event_lock, flags);

	drm_crtc_vblank_put(crtc);
}

static bool cdc_crtc_page_flip_pending (struct drm_crtc *crtc)
{
	struct drm_device *dev = crtc->dev;
	struct cdc_device *cdc = dev->dev_private;
	unsigned long flags;
	bool pending;

	spin_lock_irqsave(&dev->event_lock, flags);
	pending = cdc->event != NULL;
	spin_unlock_irqrestore(&dev->event_lock, flags);

	return pending;
}

static void cdc_crtc_wait_page_flip (struct drm_crtc *crtc)
{
	struct drm_device *dev = crtc->dev;
	struct cdc_device *cdc = dev->dev_private;

	if (wait_event_timeout(cdc->flip_wait,
			       !cdc_crtc_page_flip_pending(crtc),
			       msecs_to_jiffies(50)))
		return;

	dev_warn(cdc->dev, "page flip timeout\n");

	cdc_crtc_finish_page_flip(crtc);
}

void cdc_crtc_start (struct drm_crtc *crtc)
{
	struct cdc_device *cdc = to_cdc_dev(crtc);

	dev_dbg(cdc->dev, "%s\n", __func__);

	if (cdc->hw.enabled)
		return;

	cdc_hw_setEnabled(cdc, false);
	cdc_hw_setBackgroundColor(cdc, 0xff0000ff);

	cdc_crtc_set_display_timing(crtc);

	drm_crtc_vblank_on(crtc);

	cdc_hw_setEnabled(cdc, true);
}

void cdc_crtc_stop (struct drm_crtc *crtc)
{
	struct cdc_device *cdc = to_cdc_dev(crtc);

	dev_dbg(cdc->dev, "%s\n", __func__);

	if (!cdc->hw.enabled)
		return;

	cdc_crtc_wait_page_flip(crtc);

	dev_dbg(cdc->dev, "%s: vblank off (crtc idx: %u, num_crtcs: %u)\n",
		__func__, drm_crtc_index(crtc), crtc->dev->num_crtcs);
	drm_crtc_vblank_off(crtc);

	cdc_hw_setEnabled(cdc, false);

	if(cdc->dswz)
		dswz_stop(cdc->dswz);
}

/******************************************************************************
 * drm_crtc_funcs
 */

static void cdc_crtc_enable (struct drm_crtc *crtc, struct drm_crtc_state *old_crtc_state)
{
	struct cdc_device *cdc = to_cdc_dev(crtc);

	dev_dbg(cdc->dev, "%s\n", __func__);

	if (cdc->hw.enabled)
		return;

	cdc_crtc_start(crtc);

	/* Reenable underrun and CRC IRQs. It was maybe disabled to prevent
	 * message flooding. */
	cdc_irq_set(cdc, CDC_IRQ_FIFO_UNDERRUN, true);
	cdc_irq_set(cdc, CDC_IRQ_FIFO_UNDERRUN_WARN, true);
	cdc_irq_set(cdc, CDC_IRQ_CRC_ERROR, true);

	/* Enable line IRQ together with CRTC */
	cdc_irq_set(cdc, CDC_IRQ_LINE, true);
}

/* disable crtc when not in use - more explicit than dpms off */
static void cdc_crtc_disable (struct drm_crtc *crtc)
{
	struct cdc_device *cdc = to_cdc_dev(crtc);

	dev_dbg(cdc->dev, "%s\n", __func__);

	if (!cdc->hw.enabled)
		return;

	cdc_crtc_stop(crtc);

	cdc_irq_set(cdc, CDC_IRQ_FIFO_UNDERRUN, false);
	cdc_irq_set(cdc, CDC_IRQ_FIFO_UNDERRUN_WARN, false);
	cdc_irq_set(cdc, CDC_IRQ_CRC_ERROR, false);

	cdc_irq_set(cdc, CDC_IRQ_LINE, false);
}
;

static bool cdc_crtc_mode_fixup (struct drm_crtc *crtc,
	const struct drm_display_mode *mode, struct drm_display_mode *adjusted_mode)
{
	struct cdc_device *cdc = to_cdc_dev(crtc);

	dev_dbg(cdc->dev, "%s(%p)\n", __func__, crtc);

	return true;
}

static void cdc_crtc_atomic_begin (struct drm_crtc *crtc,
	struct drm_crtc_state *old_crtc_state)
{
	struct drm_pending_vblank_event *event = crtc->state->event;
	struct cdc_device *cdc = to_cdc_dev(crtc);
	struct drm_device *dev = crtc->dev;
	unsigned long flags;

	dev_dbg(cdc->dev, "%s (crtc: %p)\n", __func__, crtc);

	if (event) {
		WARN_ON(drm_crtc_vblank_get(crtc) != 0);

		spin_lock_irqsave(&dev->event_lock, flags);
		cdc->event = event;
		spin_unlock_irqrestore(&dev->event_lock, flags);
	}
}

static void cdc_crtc_atomic_flush (struct drm_crtc *crtc,
	struct drm_crtc_state *old_crtc_state)
{
	struct cdc_device *cdc = to_cdc_dev(crtc);

	dev_dbg(cdc->dev, "%s (crtc: %p)\n", __func__, crtc);

	dev_dbg(cdc->dev, "CRTC primary's crtc(crtc: %p)\n", crtc->primary->crtc);

	if(cdc->dswz) {
		dswz_trigger(cdc->dswz);
	}

	if (cdc->wait_for_vblank) {
		/* Schedule shadow reload for next vblank and wait for it.
		 * We only have one CRTC, so index is 0.
		 */
		cdc_hw_triggerShadowReload(cdc, true);
		drm_wait_one_vblank(crtc->dev, 0);
	} else {
		/* Reload immediately, since vblank is disabled */
		cdc_hw_triggerShadowReload(cdc, false);
	}
}

enum drm_mode_status cdc_crtc_mode_valid(struct drm_crtc *crtc,
	const struct drm_display_mode *mode)
{
	struct cdc_device *cdc = to_cdc_dev(crtc);
	long clk_hz = mode->clock * 1000l;

	if(cdc->max_clock_khz) {
		if(mode->clock > cdc->max_clock_khz)
			return MODE_NOCLOCK;
	}

	if(clk_hz != clk_round_rate(cdc->pclk, clk_hz))
		return MODE_NOCLOCK;

	return MODE_OK;
}

static const struct drm_crtc_helper_funcs crtc_helper_funcs = {
	.atomic_enable = cdc_crtc_enable,
	.disable = cdc_crtc_disable,
	.mode_fixup = cdc_crtc_mode_fixup,
	.mode_valid = cdc_crtc_mode_valid,
	.atomic_begin = cdc_crtc_atomic_begin,
	.atomic_flush = cdc_crtc_atomic_flush,
};

static const struct drm_crtc_funcs crtc_funcs = {
	.reset = drm_atomic_helper_crtc_reset,
	.destroy = drm_crtc_cleanup,
	.set_config = drm_atomic_helper_set_config,
	.page_flip = drm_atomic_helper_page_flip,
	.atomic_duplicate_state = drm_atomic_helper_crtc_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_crtc_destroy_state,
};

void cdc_crtc_irq (struct drm_crtc *crtc)
{
	unsigned long flags;
	struct cdc_device *cdc = to_cdc_dev(crtc);

	drm_crtc_handle_vblank(crtc);
	cdc_crtc_finish_page_flip(crtc);

	/* FIXME HACK for MesseDemo */
	spin_lock_irqsave(&cdc->irq_slck, flags);
	cdc->irq_stat |= 1;
	spin_unlock_irqrestore(&cdc->irq_slck, flags);
	wake_up_interruptible(&cdc->irq_waitq);
}

int cdc_crtc_create (struct cdc_device *cdc)
{
	struct drm_crtc *crtc = &cdc->crtc;
	int ret;

	dev_dbg(cdc->dev, "%s\n", __func__);

	/* TODO: add support for programmable clock? */

	cdc_hw_setEnabled(cdc, false);

	init_waitqueue_head(&cdc->flip_wait);

	/* todo: really always use first plane here? */
	ret = drm_crtc_init_with_planes(cdc->ddev, crtc, &cdc->planes[0].plane,
		&cdc->planes[cdc->hw.layer_count - 1].plane, &crtc_funcs, NULL);
	if (ret < 0) {
		dev_err(cdc->dev, "Error initializing drm_crtc_init: %d\n", ret);
		return ret;
	}

	drm_crtc_helper_add(crtc, &crtc_helper_funcs);

	/* Start with vertical blanking interrupt reporting disabled. */
	drm_crtc_vblank_off(crtc);

	return 0;
}

void cdc_crtc_set_vblank (struct cdc_device *cdc, bool enable)
{
	dev_dbg(cdc->dev, "%s(%d)\n", __func__, enable);

	cdc->wait_for_vblank = enable;

	cdc_irq_set(cdc, CDC_IRQ_LINE, enable);
}
