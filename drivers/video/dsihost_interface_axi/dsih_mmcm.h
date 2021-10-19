/* SPDX-License-Identifier: GPL-2.0 */
/**
 * Copyright (C) 2018 Synopsys, Inc.
 *
 * @file dsih_mmcm.h
 * @brief Synopsys MIPI DSI clock functions
 * included as a part of Synopsys MIPI DSI Host controller driver
 *
 * @author Luis Oliveira <luis.oliveira@synopsys.com>
 */

#ifndef MMCM_H__
#define MMCM_H__

#include "includes.h"

#define MMCM_BASE_ADDR 	0xD0007000
#define MMCM_MASK 	0x1000

#define DRP_CLKOUT5_REG_1	(0x0018)
#define DRP_CLKOUT5_REG_2	(0x001c)

#define DRP_CLKOUT0_REG_1	(0x0020)
#define DRP_CLKOUT0_REG_2	(0x0024)

#define DRP_CLKOUT1_REG_1	(0x0028)
#define DRP_CLKOUT1_REG_2	(0x002c)

#define DRP_CLKOUT2_REG_1	(0x0030)
#define DRP_CLKOUT2_REG_2	(0x0034)

#define DRP_CLKOUT3_REG_1	(0x0038)
#define DRP_CLKOUT3_REG_2	(0x003c)

#define DRP_CLKOUT4_REG_1	(0x0040)
#define DRP_CLKOUT4_REG_2	(0x0044)

#define DRP_CLKOUT6_REG_1	(0x0048)
#define DRP_CLKOUT6_REG_2	(0x004c)

#define DRP_CLKFBOUT_REG_1	(0x0050)
#define DRP_CLKFBOUT_REG_2	(0x0054)

#define DRP_DIVLOCK		(0x0058)

#define DRP_LOCK_REG_1		(0x0060)
#define DRP_LOCK_REG_2		(0x0064)
#define DRP_LOCK_REG_3		(0x0068)

#define DRP_POWER_REG		(0x00a0)

#define DRP_DIGFILT_REG_1	(0x0138)
#define DRP_DIGFILT_REG_2	(0x013c)

void mmcm_write_word(struct mipi_dsi_dev *dev, uint32_t reg_address,
			uint32_t data);

void mmcm_write_part(struct mipi_dsi_dev *dev, uint32_t reg_address,
			uint32_t data, uint8_t shift, uint8_t width);

uint32_t mmcm_read_word(struct mipi_dsi_dev *dev, uint32_t reg_address);

uint32_t mmcm_read_part(struct mipi_dsi_dev *dev, uint32_t reg_address,
			   uint8_t shift, uint8_t width);

#endif
