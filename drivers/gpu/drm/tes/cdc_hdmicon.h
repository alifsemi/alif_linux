/*
 * cdc_hdmicon.h  --  CDC Display Controller HDMI Connector
 *
 * Copyright (C) 2017 TES Electronic Solutions GmbH
 * Author: Christian Thaler <christian.thaler@tes-dst.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __CDC_HDMICON_H__
#define __CDC_HDMICON_H__

int cdc_hdmi_connector_init (struct cdc_device *cdc, struct cdc_encoder *enc);

#endif //__CDC_HDMICON_H__
