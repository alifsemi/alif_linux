/*
 * cdc_hdmienc.c  --  CDC Display Controller HDMI Encoder
 *
 * Copyright (C) 2017 TES Electronic Solutions GmbH
 * Author: Christian Thaler <christian.thaler@tes-dst.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <drm/drmP.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_encoder_slave.h>

#include "cdc_drv.h"
#include "cdc_encoder.h"

struct cdc_hdmienc {
	struct cdc_encoder *enc;
	struct device *dev;
	bool enabled;
};

#define to_cdc_hdmienc(e) (to_cdc_encoder(e)->hdmi)
#define to_slave_funcs(e) (to_cdc_encoder(e)->slave.slave_funcs)

static void cdc_hdmienc_disable (struct drm_encoder *encoder)
{
	struct cdc_device *cdc = encoder->dev->dev_private;
	struct cdc_hdmienc *hdmienc = to_cdc_hdmienc(encoder);
	const struct drm_encoder_slave_funcs *sfuncs = to_slave_funcs(encoder);

	dev_dbg(cdc->dev, "%s\n", __func__);

	if (sfuncs->dpms)
		sfuncs->dpms(encoder, DRM_MODE_DPMS_OFF);

	hdmienc->enabled = false;
}

static void cdc_hdmienc_enable (struct drm_encoder *encoder)
{
	struct cdc_device *cdc = encoder->dev->dev_private;
	struct cdc_hdmienc *hdmienc = to_cdc_hdmienc(encoder);
	const struct drm_encoder_slave_funcs *sfuncs = to_slave_funcs(encoder);

	dev_dbg(cdc->dev, "%s\n", __func__);

	if (sfuncs->dpms)
		sfuncs->dpms(encoder, DRM_MODE_DPMS_ON);

	hdmienc->enabled = true;
}

static int cdc_hdmienc_atomic_check (struct drm_encoder *encoder,
	struct drm_crtc_state *crtc_state,
	struct drm_connector_state *conn_state)
{
	struct cdc_device *cdc = encoder->dev->dev_private;
	const struct drm_encoder_slave_funcs *sfuncs = to_slave_funcs(encoder);
	struct drm_display_mode *adjusted_mode = &crtc_state->adjusted_mode;
	const struct drm_display_mode *mode = &crtc_state->mode;

	dev_dbg(cdc->dev, "%s\n", __func__);

	if (sfuncs->mode_fixup == NULL)
		return 0;

	return sfuncs->mode_fixup(encoder, mode, adjusted_mode) ? 0 : -EINVAL;
}

static void cdc_hdmienc_mode_set (struct drm_encoder *encoder,
	struct drm_display_mode *mode, struct drm_display_mode *adjusted_mode)
{
	struct cdc_device *cdc = encoder->dev->dev_private;
	const struct drm_encoder_slave_funcs *sfuncs = to_slave_funcs(encoder);

	dev_dbg(cdc->dev, "%s\n", __func__);

	if (sfuncs->mode_set)
		sfuncs->mode_set(encoder, mode, adjusted_mode);
}

static const struct drm_encoder_helper_funcs encoder_helper_funcs = {
	.mode_set = cdc_hdmienc_mode_set,
	.disable = cdc_hdmienc_disable,
	.enable = cdc_hdmienc_enable,
	.atomic_check = cdc_hdmienc_atomic_check,
};

static void cdc_hdmienc_cleanup (struct drm_encoder *encoder)
{
	struct cdc_hdmienc *hdmienc = to_cdc_hdmienc(encoder);

	if (hdmienc->enabled)
		cdc_hdmienc_disable(encoder);

	drm_encoder_cleanup(encoder);
	put_device(hdmienc->dev);
}

static const struct drm_encoder_funcs encoder_funcs = {
	.destroy = cdc_hdmienc_cleanup,
};

int cdc_hdmienc_init (struct cdc_device *cdc, struct cdc_encoder *enc,
	struct device_node *np)
{
	struct drm_encoder *encoder = cdc_encoder_to_drm_encoder(enc);
	struct drm_i2c_encoder_driver *driver;
	struct i2c_client *i2c_slave;
	struct cdc_hdmienc *hdmienc;
	int ret;

	dev_dbg(cdc->dev, "%s (encoder: %s)\n", __func__, np->full_name);

	hdmienc = devm_kzalloc(cdc->dev, sizeof(*hdmienc), GFP_KERNEL);
	if (hdmienc == NULL)
		return -ENOMEM;

	/* Locate the slave I2C device and driver. */
	i2c_slave = of_find_i2c_device_by_node(np); // todo: must call put_device() when done with returned i2c_client device
	if (!i2c_slave || !i2c_get_clientdata(i2c_slave)) {
		if (i2c_slave)
			dev_dbg(cdc->dev, "Could not get %s's client data\n",
				i2c_slave->name);
		return -EPROBE_DEFER;
	}

	hdmienc->dev = &i2c_slave->dev;

	if (hdmienc->dev->driver == NULL) {
		dev_dbg(cdc->dev, "Could not get %s's driver instance\n",
			i2c_slave->name);
		ret = -EPROBE_DEFER;
		goto error;
	}

	/* Initialize the slave encoder. */
	driver = to_drm_i2c_encoder_driver(to_i2c_driver(hdmienc->dev->driver));
	ret = driver->encoder_init(i2c_slave, cdc->ddev, &enc->slave);
	if (ret < 0)
		goto error;

	ret = drm_encoder_init(cdc->ddev, encoder, &encoder_funcs,
			       DRM_MODE_ENCODER_TMDS, NULL);
	if (ret < 0)
		goto error;

	drm_encoder_helper_add(encoder, &encoder_helper_funcs);

	enc->hdmi = hdmienc;
	hdmienc->enc = enc;

	return 0;

	error: put_device(hdmienc->dev);
	dev_err(cdc->dev, "HDMI encoder initialization failed\n");
	return ret;
}
