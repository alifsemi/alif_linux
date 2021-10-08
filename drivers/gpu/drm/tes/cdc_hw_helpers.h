/*
 * cdc_hw_helpers.h  --  CDC Display Controller Hardware Interface
 *
 * Copyright (C) 2017 TES Electronic Solutions GmbH
 * Author: Christian Thaler <christian.thaler@tes-dst.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef CDC_HW_HELPERS_H_
#define CDC_HW_HELPERS_H_

#include <linux/types.h>

#include "cdc_drv.h"
#include "cdc_regs.h"

void cdc_hw_setPixelFormat (struct cdc_device *cdc, int layer, u8 format);
void cdc_hw_setBlendMode (struct cdc_device *cdc, int layer,
	cdc_blend_factor a_factor1, cdc_blend_factor a_factor2);
void cdc_hw_setWindow (struct cdc_device *cdc, int layer, u16 startX,
	u16 startY, u16 width, u16 height, s16 pitch);
void cdc_hw_setCBAddress (struct cdc_device *cdc, int layer, dma_addr_t address);
void cdc_hw_layer_setEnabled (struct cdc_device *cdc, int layer, bool enable);
void cdc_hw_resetRegisters (struct cdc_device *cdc);
bool cdc_hw_triggerShadowReload (struct cdc_device *cdc, bool in_vblank);
void cdc_hw_setTiming (struct cdc_device *cdc, u16 a_h_sync, u16 a_h_bPorch,
	u16 a_h_width, u16 a_h_fPorch, u16 a_v_sync, u16 a_v_bPorch,
	u16 a_v_width, u16 a_v_fPorch, bool a_neg_hsync, bool a_neg_vsync,
	bool a_neg_blank, bool a_inv_clk);
void cdc_hw_setEnabled (struct cdc_device *cdc, bool enable);
void cdc_hw_setBackgroundColor (struct cdc_device *cdc, u32 color);
void cdc_hw_layer_setCBSize (struct cdc_device *cdc, int layer, u16 width,
	u16 height, s16 pitch);
void cdc_hw_layer_setConstantAlpha (struct cdc_device *cdc, int layer, u8 alpha);

#endif /* CDC_HW_HELPERS_H_ */
