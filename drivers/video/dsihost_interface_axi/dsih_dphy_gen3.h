/* SPDX-License-Identifier: GPL-2.0 */
/**
 * Copyright (C) 2018 Synopsys, Inc.
 *
 * @file dsih_dphy_gen3.h
 * @brief Synopsys MIPI DSI DPHY specific GEN3 functions
 * included as a part of Synopsys MIPI DSI Host controller driver
 *
 * @author Luis Oliveira <luis.oliveira@synopsys.com>
 */

#ifndef GEN3_IF_H__
#define GEN3_IF_H__

#include "includes.h"

enum glueiftester{
	GLUELOGIC = 0x4,
	RX_PHY = 0x2,
	TX_PHY = 0x1,
	RESET = 0x0,
};

#define GEN3_IF_TESTER 0x3c
#define GEN3_IF_SOC_PLL 0x48
#define GEN3_IF_SOC_PLL_EN 0x4C

void gen3_if_write_word(struct mipi_dsi_dev *dev, uint32_t reg_address,
			uint32_t data);

void gen3_if_write_part(struct mipi_dsi_dev *dev, uint32_t reg_address,
			uint32_t data, uint8_t shift, uint8_t width);

uint32_t gen3_if_read_word(struct mipi_dsi_dev *dev, uint32_t reg_address);

uint32_t gen3_if_read_part(struct mipi_dsi_dev *dev, uint32_t reg_address,
			   uint8_t shift, uint8_t width);

#endif /* GEN3_IF_H__ */
