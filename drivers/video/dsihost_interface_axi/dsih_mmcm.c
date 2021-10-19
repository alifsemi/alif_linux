// SPDX-License-Identifier: GPL-2.0
/**
 * Copyright (C) 2018 Synopsys, Inc.
 *
 * @file dsih_mmcm.c
 * @brief Synopsys MIPI DSI clock functions
 * included as a part of Synopsys MIPI DSI Host controller driver
 *
 * @author Luis Oliveira <luis.oliveira@synopsys.com>
 */

#include "dsih_mmcm.h"

/**
 * Write a bit field o a 32-bit word to the DSI Host core
 * @param dev pointer to structure holding the DSI Host core information
 * @param reg_address register offset in core
 * @param data to be written to register
 * @param shift bit shift from the left (system is BIG ENDIAN)
 * @param width of bit field
 */
void mmcm_write_part(struct mipi_dsi_dev *dev, uint32_t reg_address, uint32_t data, uint8_t shift, uint8_t width)
{
	uint32_t mask = (1 << width) - 1;
	uint32_t temp = mmcm_read_word(dev, reg_address);

	temp &= ~(mask << shift);
	temp |= (data & mask) << shift;
	mmcm_write_word(dev, reg_address, temp);
}
/**
 * Write a 32-bit word to the DSI Host core
 * @param dev pointer to structure holding the DSI Host core information
 * @param reg_address offset of register
 * @return 32-bit word value stored in register
 */
uint32_t mmcm_read_word(struct mipi_dsi_dev *dev, uint32_t reg_address)
{
	uint32_t ret;

	if(dev == NULL){
		pr_err("%s:Device is null\n", FUNC_NAME);
		return -ENODEV;
	}

	ret = ioread32(dev->mmcm_addr + reg_address);
	pr_debug("MMCM: READ ADDR %X DATA %X\n", reg_address, ret);
	return ret;
}
/**
 * Write a 32-bit word to the DSI Host core
 * @param dev pointer to structure holding the DSI Host core information
 * @param reg_address offset of register in core
 * @param shift bit shift from the left (system is BIG ENDIAN)
 * @param width of bit field
 * @return bit field read from register
 */
uint32_t mmcm_read_part(struct mipi_dsi_dev *dev, uint32_t reg_address, uint8_t shift, uint8_t width)
{
	return (mmcm_read_word(dev, reg_address) >> shift) & ((1 << width) - 1);
}

/**
 * Write a 32-bit word to the DSI Host core
 * @param dev pointer to structure holding the DSI Host core information
 * @param reg_address register offset in core
 * @param data 32-bit word to be written to register
 */
void mmcm_write_word(struct mipi_dsi_dev *dev, uint32_t reg_address, uint32_t data)
{
	if(dev == NULL){
		pr_err("%s:Device is null\n", FUNC_NAME);
		return;
	}
	pr_debug("MMCM: WRITE ADDR %X DATA %X\n", reg_address, data);
	iowrite32(data,dev->mmcm_addr + reg_address);
}
