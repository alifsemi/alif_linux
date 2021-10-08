/*
 * cdc_hdmienc.h  --  CDC Display Controller HDMI Encoder
 *
 * Copyright (C) 2017 TES Electronic Solutions GmbH
 * Author: Christian Thaler <christian.thaler@tes-dst.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __CDC_HDMIENC_H__
#define __CDC_HDMIENC_H__

int cdc_hdmienc_init (struct cdc_device *cdc, struct cdc_encoder *enc,
	struct device_node *np);

#endif //__CDC_HDMIENC_H__
