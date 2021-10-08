/*
 * cdc_hw.c  --  CDC Display Controller Hardware Interface
 *
 * Copyright (C) 2017 TES Electronic Solutions GmbH
 * Author: Christian Thaler <christian.thaler@tes-dst.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/slab.h>
#include "cdc_drv.h"
#include "cdc_crtc.h"
#include "cdc_regs.h"

u32 cdc_read_reg (struct cdc_device *cdc, u32 reg)
{
	return ioread32(cdc->mmio + (reg * 4));
}

void cdc_write_reg (struct cdc_device *cdc, u32 reg, u32 val)
{
	iowrite32(val, cdc->mmio + (reg * 4));
}

static int layer_offset (int layer)
{
	return (layer + 1) * CDC_OFFSET_LAYER;
}

u32 cdc_read_layer_reg (struct cdc_device *cdc, int layer, u32 reg)
{
	return cdc_read_reg(cdc, layer_offset(layer) + reg);
}

void cdc_write_layer_reg (struct cdc_device *cdc, int layer, u32 reg, u32 val)
{
	cdc_write_reg(cdc, layer_offset(layer) + reg, val);
}

static u32 read_reg (struct cdc_device *cdc, u32 reg)
{
	return ioread32(cdc->mmio + (reg * 4));
}

static void write_reg (struct cdc_device *cdc, u32 reg, u32 val)
{
	iowrite32(val, cdc->mmio + (reg * 4));
}

void cdc_irq_set (struct cdc_device *cdc, cdc_irq_type irq, bool enable)
{
	u32 status;

	status = read_reg(cdc, CDC_REG_GLOBAL_IRQ_ENABLE);

	if (enable)
		status |= irq;
	else
		status &= ~(irq);

	write_reg(cdc, CDC_REG_GLOBAL_IRQ_ENABLE, status);
}
