// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2018-2019 Synopsys, Inc. and/or its affiliates.
 *
 * Synopsys DesignWare MIPI D-PHY controller driver
 * Core functions
 *
 * Author: Luis Oliveira <luis.oliveira@synopsys.com>
 */

#include "dw-dphy-rx.h"

#define MASK(h,l)   (((~(0U)) << (l)) & (~(0U) >> (32 - 1 - (h))))


#define MIPI_CSI2_N_LANES	2
// Camera Clk configured to 20 Mhz
#define DPHY_FCFG_CLOCK_MHZ	25

#define TX_PLL_13		(0x16A)
#define TX_CB_0			(0x1AA)
#define TX_CB_1			(0x1AB)
#define RX_CLKLANE_LANE_6	(0x307)
#define RX_SYS_1		(0x01F)
#define RX_STARTUP_OVR_2	(0x0E2)
#define RX_STARTUP_OVR_3	(0x0E3)
#define RX_STARTUP_OVR_4	(0x0E4)


#define DPHY_STOPSTATE_LANE0	(1U << 0)
#define DPHY_STOPSTATE_LANE1	(1U << 1)
#define DPHY_STOPSTATE_CLOCK	(1U << 2)

struct range_dphy_gen2 {
	u32 freq;
	u8 hsfregrange;
};

struct range_dphy_gen2 range_gen2[] = {
	{ 80, 0x00 }, { 90, 0x10 }, { 100, 0x20 }, { 110, 0x30 },
	{ 120, 0x01 }, { 130, 0x11 }, { 140, 0x21 }, { 150, 0x31 },
	{ 160, 0x02 }, { 170, 0x12 }, { 180, 0x22 }, { 190, 0x32 },
	{ 205, 0x03 }, { 220, 0x13 }, { 235, 0x23 }, { 250, 0x33 },
	{ 275, 0x04 }, { 300, 0x14 }, { 325, 0x05 }, { 350, 0x15 },
	{ 400, 0x25 }, { 450, 0x06 }, { 500, 0x16 }, { 550, 0x07 },
	{ 600, 0x17 }, { 650, 0x08 }, { 700, 0x18 }, { 750, 0x09 },
	{ 800, 0x19 }, { 850, 0x29 }, { 900, 0x39 }, { 950, 0x0A },
	{ 1000, 0x1A }, { 1050, 0x2A }, { 1100, 0x3A }, { 1150, 0x0B },
	{ 1200, 0x1B }, { 1250, 0x2B }, { 1300, 0x3B }, { 1350, 0x0C },
	{ 1400, 0x1C }, { 1450, 0x2C }, { 1500, 0x3C }, { 1550, 0x0D },
	{ 1600, 0x1D }, { 1650, 0x2D }, { 1700, 0x0E }, { 1750, 0x1E },
	{ 1800, 0x2E }, { 1850, 0x3E }, { 1900, 0x0F }, { 1950, 0x1F },
	{ 2000, 0x2F },
};

struct range_dphy_gen3 {
	u32 freq;
	u8 hsfregrange;
	u32 osc_freq_target;
};

struct range_dphy_gen3 range_gen3[] = {
	{ 80, 0x00, 0x1B6 }, { 90, 0x10, 0x1B6 }, { 100, 0x20, 0x1B6 },
	{ 110, 0x30, 0x1B6 }, { 120, 0x01, 0x1B6 }, { 130, 0x11, 0x1B6 },
	{ 140, 0x21, 0x1B6 }, { 150, 0x31, 0x1B6 }, { 160, 0x02, 0x1B6 },
	{ 170, 0x12, 0x1B6 }, { 180, 0x22, 0x1B6 }, { 190, 0x32, 0x1B6 },
	{ 205, 0x03, 0x1B6 }, { 220, 0x13, 0x1B6 }, { 235, 0x23, 0x1B6 },
	{ 250, 0x33, 0x1B6 }, { 275, 0x04, 0x1B6 }, { 300, 0x14, 0x1B6 },
	{ 325, 0x25, 0x1B6 }, { 350, 0x35, 0x1B6 }, { 400, 0x05, 0x1B6 },
	{ 450, 0x16, 0x1B6 }, { 500, 0x26, 0x1B6 }, { 550, 0x37, 0x1B6 },
	{ 600, 0x07, 0x1B6 }, { 650, 0x18, 0x1B6 }, { 700, 0x28, 0x1B6 },
	{ 750, 0x39, 0x1B6 }, { 800, 0x09, 0x1B6 }, { 850, 0x19, 0x1B6 },
	{ 900, 0x29, 0x1B6 }, { 950, 0x3A, 0x1B6 }, { 1000, 0x0A, 0x1B6 },
	{ 1050, 0x1A, 0x1B6 }, { 1100, 0x2A, 0x1B6 }, { 1150, 0x3B, 0x1B6 },
	{ 1200, 0x0B, 0x1B6 }, { 1250, 0x1B, 0x1B6 }, { 1300, 0x2B, 0x1B6 },
	{ 1350, 0x3C, 0x1B6 }, { 1400, 0x0C, 0x1B6 }, { 1450, 0x1C, 0x1B6 },
	{ 1500, 0x2C, 0x1B6 }, { 1550, 0x3D, 0x10F }, { 1600, 0x0D, 0x118 },
	{ 1650, 0x1D, 0x121 }, { 1700, 0x2E, 0x12A }, { 1750, 0x3E, 0x132 },
	{ 1800, 0x0E, 0x13B }, { 1850, 0x1E, 0x144 }, { 1900, 0x2F, 0x14D },
	{ 1950, 0x3F, 0x155 }, { 2000, 0x0F, 0x15E }, { 2050, 0x40, 0x167 },
	{ 2100, 0x41, 0x170 }, { 2150, 0x42, 0x178 }, { 2200, 0x43, 0x181 },
	{ 2250, 0x44, 0x18A }, { 2300, 0x45, 0x193 }, { 2350, 0x46, 0x19B },
	{ 2400, 0x47, 0x1A4 }, { 2450, 0x48, 0x1AD }, { 2500, 0x49, 0x1B6 }
};

static uint8_t dphy_stopstate(struct dw_dphy_rx *dphy)
{
	uint32_t reg_status = 0;
	uint8_t ret = 0;

	reg_status = dw_dphy_read(dphy, R_CSI2_DPHY_STOPSTATE);

	if(reg_status & 0x1) 		ret |= DPHY_STOPSTATE_LANE0;
	if(reg_status & (0x1 << 1)) 	ret |= DPHY_STOPSTATE_LANE1;
	if(reg_status & (0x1 << 16)) 	ret |= DPHY_STOPSTATE_CLOCK;

	return ret;
}


static int dw_dphy_te_12b_read(struct dw_dphy_rx *dphy, u32 addr);
static void dw_dphy_te_12b_write(struct dw_dphy_rx *dphy, u16 addr, u8 data);

static inline void csi_dphy_write_msk(struct dw_dphy_rx *dphy, u32 address, u32 data, u8 shift,
		       u8 width)
{
	u32 temp = dw_dphy_te_12b_read(dphy, address);
	u32 mask = (1 << width) - 1;

	temp &= ~(mask << shift);
	temp |= (data & mask) << shift;

	dw_dphy_te_12b_write(dphy, address, temp);
}


u8 dw_dphy_setup_config(struct dw_dphy_rx *dphy)
{
#if 0
#if IS_ENABLED(CONFIG_DWC_MIPI_TC_DPHY_GEN3)
	int ret;

	if (dphy->max_lanes == CTRL_4_LANES) {
		dev_vdbg(&dphy->phy->dev, "CONFIG 4L\n");
		return CTRL_4_LANES;
	}
	if (IS_ENABLED(CONFIG_OF)) {
		ret = gpio_request(dphy->config_8l, "config");
		if (ret < 0) {
			dev_vdbg(&dphy->phy->dev,
				 "could not acquire config (err=%d)\n", ret);
			return ret;
		}
		ret = gpio_get_value(dphy->config_8l);
		gpio_free(dphy->config_8l);
	} else {
		ret = dphy->config_8l;
	}

	dev_vdbg(&dphy->phy->dev,
		 "Booting in [%s] mode\n",
		 ret == CTRL_8_LANES ? "8L" : "4+4L");
	return ret;

#endif /* CONFIG_DWC_MIPI_TC_DPHY_GEN3 */
#endif
	return CTRL_4_LANES;
}

#if 0
#if IS_ENABLED(CONFIG_DWC_MIPI_TC_DPHY_GEN3)
void dw_dphy_if_write(struct dw_dphy_rx *dphy, u32 address, u32 data)
{
	writel(data, dphy->dphy1_if_addr + address);

	if (dphy->lanes_config == CTRL_4_LANES)
		return;

	iowrite32(data, dphy->dphy2_if_addr + address);
}

u32 dw_dphy_if_read(struct dw_dphy_rx *dphy, u32 address)
{
	u32 if1 = 0, if2 = 0;

	if1 = readl(dphy->dphy1_if_addr + address);

	if (dphy->lanes_config == CTRL_4_LANES)
		goto end;

	if (dphy->lanes_config == DPHYID)
		goto end;

	if2 = readl(dphy->dphy2_if_addr + address);

	if (if1 != if2)
		dev_vdbg(&dphy->phy->dev,
			 "Values read different for each interface\n");
end:
	return if1;
}
#endif
#endif
void dw_dphy_write(struct dw_dphy_rx *dphy, u32 address, u32 data)
{
#if 0
	iowrite32(data, dphy->base_address + address);

	if (dphy->lanes_config == CTRL_4_LANES)
		return;

	if (address == R_CSI2_DPHY_TST_CTRL0)
		iowrite32(data, dphy->base_address + R_CSI2_DPHY2_TST_CTRL0);
	else if (address == R_CSI2_DPHY_TST_CTRL1)
		iowrite32(data, dphy->base_address + R_CSI2_DPHY2_TST_CTRL1);
#endif
	writel(data, dphy->base_address + address);
}

u32 dw_dphy_read(struct dw_dphy_rx *dphy, u32 address)
{
#if 0
	int dphy1 = 0, dphy2 = 0;

	dphy1 = ioread32(dphy->base_address + address);

	if (dphy->lanes_config == CTRL_4_LANES)
		goto end;

	if (address == R_CSI2_DPHY_TST_CTRL0)
		dphy2 = ioread32(dphy->base_address + R_CSI2_DPHY2_TST_CTRL0);
	else if (address == R_CSI2_DPHY_TST_CTRL1)
		dphy2 = ioread32(dphy->base_address + R_CSI2_DPHY2_TST_CTRL1);
	else
		return -ENODEV;
end:
	return dphy1;
#endif
	u32 val;
	val = readl(dphy->base_address + address);
//printk("dw_dphy_read: addr 0x%x = val 0x%x\n",dphy->base_address + address, val);
	return val;
}

void dw_dphy_write_msk(struct dw_dphy_rx *dev, u32 address, u32 data, u8 shift,
		       u8 width)
{
	u32 temp = dw_dphy_read(dev, address);
	u32 mask = (1 << width) - 1;

	temp &= ~(mask << shift);
	temp |= (data & mask) << shift;
	dw_dphy_write(dev, address, temp);
}

static void dw_dphy_te_12b_write(struct dw_dphy_rx *dphy, u16 addr, u8 data)
{
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 0, 1, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, 0, 16, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, 1, 16, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 1, 1, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, 0x0, 0, 8);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 0, 1, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, 0, 16, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, (u8)(addr >> 8),
			  PHY_TESTDIN, 8);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 1, 1, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 0, 1, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, 1, 16, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 1, 1, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, (u8)(addr & 0xFF), 0, 8);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 0, 1, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, 0, 16, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, (u8)data, 0, 8);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 1, 1, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 0, 1, 1);
}

#if 0
static void dw_dphy_te_8b_write(struct dw_dphy_rx *dphy, u8 addr, u8 data)
{
printk("#alif dw_dphy_te_8b_write addr 0x%x data 0x%x\n",addr,data);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 1, PHY_TESTCLK, 1);
	dw_dphy_write(dphy, R_CSI2_DPHY_TST_CTRL1, addr);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, 1, PHY_TESTEN, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 0, PHY_TESTCLK, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, 0, PHY_TESTEN, 1);
	dw_dphy_write(dphy, R_CSI2_DPHY_TST_CTRL1, data);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 1, PHY_TESTCLK, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 0, PHY_TESTCLK, 1);
}
static void dw_dphy_te_write(struct dw_dphy_rx *dphy, u16 addr, u8 data)
{
	if (dphy->dphy_te_len == BIT12)
		dw_dphy_te_12b_write(dphy, addr, data);
	else
		dw_dphy_te_8b_write(dphy, addr, data);
}
#endif

static int dw_dphy_te_12b_read(struct dw_dphy_rx *dphy, u32 addr)
{
	u8 ret;

//	dw_dphy_write(dphy, R_CSI2_DPHY_SHUTDOWNZ, 0);
//	dw_dphy_write(dphy, R_CSI2_DPHY_RSTZ, 0);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 0, PHY_TESTCLK, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, 0, PHY_TESTEN, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, 1, PHY_TESTEN, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 1, PHY_TESTCLK, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, 0x00, PHY_TESTDIN, 8);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 0, PHY_TESTCLK, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, 0, PHY_TESTEN, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, (u8)(addr >> 8),
			  PHY_TESTDIN, 8);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 1, PHY_TESTCLK, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 0, PHY_TESTCLK, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, 1, PHY_TESTEN, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 1, PHY_TESTCLK, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, (u8)addr, PHY_TESTDIN,
			  8);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 0, PHY_TESTCLK, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, 0x00, 0, PHY_TESTDIN);
	ret = dw_dphy_read_msk(dphy, R_CSI2_DPHY_TST_CTRL1, PHY_TESTDOUT, 8);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, 0, PHY_TESTEN, 1);
//	dw_dphy_write(dphy, R_CSI2_DPHY_RSTZ, 1);
//	dw_dphy_write(dphy, R_CSI2_DPHY_SHUTDOWNZ, 1);

	return ret;
}
#if 0
static int dw_dphy_te_8b_read(struct dw_dphy_rx *dphy, u32 addr)
{
	u8 ret;

	dw_dphy_write(dphy, R_CSI2_DPHY_SHUTDOWNZ, 0);
	dw_dphy_write(dphy, R_CSI2_DPHY_RSTZ, 0);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, 1, PHY_TESTEN, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 1, PHY_TESTCLK, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, addr, PHY_TESTDIN, 8);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 0, PHY_TESTCLK, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, 0, PHY_TESTEN, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, 0, PHY_TESTDIN, 8);
	ret = dw_dphy_read_msk(dphy, R_CSI2_DPHY_TST_CTRL1, PHY_TESTDOUT, 8);
	dw_dphy_write(dphy, R_CSI2_DPHY_RSTZ, 1);
	dw_dphy_write(dphy, R_CSI2_DPHY_SHUTDOWNZ, 1);

	return ret;
}
#endif

int dw_dphy_te_read(struct dw_dphy_rx *dphy, u32 addr)
{
	int ret;

//	if (dphy->dphy_te_len == BIT12)
		ret = dw_dphy_te_12b_read(dphy, addr);
//	else
//		ret = dw_dphy_te_8b_read(dphy, addr);

	return ret;
}

#if 0
#if IS_ENABLED(CONFIG_DWC_MIPI_TC_DPHY_GEN3)
static void dw_dphy_if_init(struct dw_dphy_rx *dphy)
{
	dw_dphy_if_write(dphy, DPHYGLUEIFTESTER, RESET);
	dw_dphy_if_write(dphy, DPHYGLUEIFTESTER, TX_PHY);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 1, PHY_TESTCLR, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 0, PHY_TESTCLR, 1);
	dw_dphy_if_write(dphy, DPHYZCALCTRL, 0);
	dw_dphy_if_write(dphy, DPHYZCALCTRL, 1);
	dw_dphy_if_write(dphy, DPHYGLUEIFTESTER, RESET);
	dw_dphy_if_write(dphy, DPHYGLUEIFTESTER, GLUELOGIC);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 1, PHY_TESTCLR, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 0, PHY_TESTCLR, 1);
	dw_dphy_if_write(dphy, DPHYZCALCTRL, 0);
	dw_dphy_if_write(dphy, DPHYZCALCTRL, 1);
	dw_dphy_if_write(dphy, DPHYGLUEIFTESTER, RESET);
	dw_dphy_if_write(dphy, DPHYGLUEIFTESTER, RX_PHY);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 1, PHY_TESTCLR, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 0, PHY_TESTCLR, 1);
	dw_dphy_if_write(dphy, DPHYZCALCTRL, 0);
	dw_dphy_if_write(dphy, DPHYZCALCTRL, 1);
}
#endif

static void dw_dphy_gen3_12bit_tc_power_up(struct dw_dphy_rx *dphy)
{
#if IS_ENABLED(CONFIG_DWC_MIPI_TC_DPHY_GEN3)
	dw_dphy_if_write(dphy, DPHYGLUEIFTESTER, RESET);
	dw_dphy_if_write(dphy, DPHYGLUEIFTESTER, GLUELOGIC);
#endif
	dw_dphy_te_write(dphy, CFGCLKFREQRANGE_TX, 0x1C);

	/* CLKSEL | UPDATEPLL | SHADOW_CLEAR | SHADOW_CTRL | FORCEPLL */
	dw_dphy_te_write(dphy, BYPASS, 0x3F);

	/* IO_DS3 | IO_DS2 | IO_DS1 | IO_DS0 */
	if (dphy->dphy_freq > 1500)
		dw_dphy_te_write(dphy, IO_DS, 0x0F);
#if IS_ENABLED(CONFIG_DWC_MIPI_TC_DPHY_GEN3)
	dw_dphy_if_write(dphy, DPHYGLUEIFTESTER, RESET);
	dw_dphy_if_write(dphy, DPHYGLUEIFTESTER, RX_PHY);
#endif
}
static void dw_dphy_gen3_8bit_tc_power_up(struct dw_dphy_rx *dphy)
{
	u32 input_freq = dphy->dphy_freq / 1000;
#if IS_ENABLED(CONFIG_DWC_MIPI_TC_DPHY_GEN3)
	dw_dphy_if_write(dphy, DPHYGLUEIFTESTER, RESET);
	dw_dphy_if_write(dphy, DPHYGLUEIFTESTER, GLUELOGIC);
	dw_dphy_te_write(dphy, CFGCLKFREQRANGE_RX, 0x1C);
	dw_dphy_if_write(dphy, DPHYGLUEIFTESTER, RESET);
	dw_dphy_if_write(dphy, DPHYGLUEIFTESTER, RX_PHY);
#endif
	dw_dphy_te_write(dphy, OSC_FREQ_TARGET_RX0_MSB, 0x03);
	dw_dphy_te_write(dphy, OSC_FREQ_TARGET_RX0_LSB, 0x02);
	dw_dphy_te_write(dphy, OSC_FREQ_TARGET_RX1_MSB, 0x03);
	dw_dphy_te_write(dphy, OSC_FREQ_TARGET_RX1_LSB, 0x02);
	dw_dphy_te_write(dphy, OSC_FREQ_TARGET_RX2_MSB, 0x03);
	dw_dphy_te_write(dphy, OSC_FREQ_TARGET_RX2_LSB, 0x02);
	dw_dphy_te_write(dphy, OSC_FREQ_TARGET_RX3_MSB, 0x03);
	dw_dphy_te_write(dphy, OSC_FREQ_TARGET_RX3_LSB, 0x02);
	dw_dphy_te_write(dphy, BANDGAP_CTRL, 0x80);

	if (input_freq < 2000)
		dw_dphy_te_write(dphy, HS_RX_CTRL_LANE0, 0xC0);

	if (input_freq < 1000) {
		dw_dphy_te_write(dphy, HS_RX_CTRL_LANE1, 0xC0);
		dw_dphy_te_write(dphy, HS_RX_CTRL_LANE2, 0xC0);
		dw_dphy_te_write(dphy, HS_RX_CTRL_LANE3, 0xC0);
	}
}
#endif
int dw_dphy_g118_settle(struct dw_dphy_rx *dphy)
{
	u32 input_freq, total_settle, settle_time, byte_clk, lp_time;

	lp_time = dphy->lp_time;
	input_freq = dphy->dphy_freq / 1000;

	settle_time = (8 * (1000000 / (input_freq))) + 115000;
	byte_clk = (8000000 / (input_freq));
	total_settle = (settle_time + lp_time * 1000) / byte_clk;

	if (total_settle > 0xFF)
		total_settle = 0xFF;

	return total_settle;
}

#if 0
static void dw_dphy_pwr_down(struct dw_dphy_rx *dphy)
{
	dw_dphy_write(dphy, R_CSI2_DPHY_RSTZ, 0);

	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 0, PHY_TESTCLK, 1);
	if (dphy->lanes_config == CTRL_8_LANES)
		dw_dphy_write_msk(dphy, R_CSI2_DPHY2_TST_CTRL0, 0, PHY_TESTCLK,
				  1);

	dw_dphy_write(dphy, R_CSI2_DPHY_SHUTDOWNZ, 0);
}

static void dw_dphy_pwr_up(struct dw_dphy_rx *dphy)
{
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 1, PHY_TESTCLK, 1);
	if (dphy->lanes_config == CTRL_8_LANES)
		dw_dphy_write_msk(dphy, R_CSI2_DPHY2_TST_CTRL0, 1, PHY_TESTCLK,
				  1);
	dev_vdbg(&dphy->phy->dev, "DPHY power up.\n");
	dw_dphy_write(dphy, R_CSI2_DPHY_SHUTDOWNZ, 1);
	dw_dphy_write(dphy, R_CSI2_DPHY_RSTZ, 1);
}

static int dw_dphy_gen3_12bit_configure(struct dw_dphy_rx *dphy)
{
	u32 input_freq = dphy->dphy_freq;
	u8 range = 0;

	dev_vdbg(&dphy->phy->dev, "12bit: PHY GEN 3: Freq: %u\n", input_freq);
	for (range = 0; (range < ARRAY_SIZE(range_gen3) - 1) &&
	     ((input_freq / 1000) > range_gen3[range].freq);
	     range++)
		;

	dw_dphy_gen3_12bit_tc_power_up(dphy);
	dw_dphy_te_write(dphy, RX_SYS_1, range_gen3[range].hsfregrange);
	dw_dphy_te_write(dphy, RX_SYS_0, 0x20);
	dw_dphy_te_write(dphy, RX_RX_STARTUP_OVR_2,
			 (u8)range_gen3[range].osc_freq_target);
	dw_dphy_te_write(dphy, RX_RX_STARTUP_OVR_3,
			 (u8)(range_gen3[range].osc_freq_target >> 8));
	dw_dphy_te_write(dphy, RX_RX_STARTUP_OVR_4, 0x01);

	if (dphy->phy_type) {
		dw_dphy_te_write(dphy, RX_RX_STARTUP_OVR_1, 0x01);
		dw_dphy_te_write(dphy, RX_RX_STARTUP_OVR_0, 0x80);
	}

	if (dphy->phy_type || input_freq <= 1500)
		dw_dphy_te_write(dphy, RX_SYS_7, 0x38);

	return 0;
}

static int dw_dphy_gen3_8bit_configure(struct dw_dphy_rx *dphy)
{
	u32 input_freq = dphy->dphy_freq;
	u8 data;
	u8 range = 0;

	dev_vdbg(&dphy->phy->dev, "8bit: PHY GEN 3: Freq: %u\n", input_freq);
	for (range = 0; (range < ARRAY_SIZE(range_gen3) - 1) &&
	     ((input_freq / 1000) > range_gen3[range].freq);
	     range++)
		;

	dw_dphy_te_write(dphy, RX_SKEW_CAL, dw_dphy_g118_settle(dphy));
	data = 1 << 7 | range_gen3[range].hsfregrange;
	dw_dphy_te_write(dphy, HSFREQRANGE_8BIT, data);
	dw_dphy_gen3_8bit_tc_power_up(dphy);
	return 0;
}
#endif

#if 0
static int dw_dphy_gen2_configure(struct dw_dphy_rx *dphy)
{
	u32 input_freq = dphy->dphy_freq;
	u8 data;
	u8 range = 0;

	/* provide an initial active-high test clear pulse in TESTCLR  */
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 1, PHY_TESTCLR, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 0, PHY_TESTCLR, 1);

	dev_vdbg(&dphy->phy->dev, "PHY GEN 2: Freq: %u\n", input_freq);
	for (range = 0; (range < ARRAY_SIZE(range_gen2) - 1) &&
	     ((input_freq / 1000) > range_gen2[range].freq); range++)
		;

	data = range_gen2[range].hsfregrange << 1;
	dw_dphy_te_write(dphy, HSFREQRANGE_8BIT, data);
	return 0;
}
#endif

int alif_dsi_phy_init(void __iomem *dsi);

//static struct dw_dphy_rx *hdphy;

static int32_t alif_csi_dphy_setup(struct dw_dphy_rx *dphy, uint32_t clock_frequency)
{
	uint32_t bitrate = (clock_frequency * 2);
	uint8_t hsfreqrange = 0;
	uint8_t cfgclkfreqrange = 0;
	uint32_t osc_freq_target = 0;
	uint8_t range = 0;
	uint32_t reg_data = 0;
	uint8_t stopstate_check =0;
	uint32_t lp_count = 0;
	void __iomem *expslv1;

	expslv1 = ioremap(0x4903F000, 0x40);

	for(range = 0; (range < ARRAY_SIZE(range_gen3) - 1) &&
		((bitrate/1000000) > range_gen3[range].freq); ++range);

	hsfreqrange = range_gen3[range].hsfregrange;
	osc_freq_target = range_gen3[range].osc_freq_target;

	pr_info("%s hsfreqrange 0x%x osc_freq_target 0x%x "
		"range %d range_gent3.hsfregrange %d\n",
		__func__,
		hsfreqrange, osc_freq_target, range,
		range_gen3[range].hsfregrange);

	//dw_dphy_init(dphy);
	dw_dphy_write(dphy, R_CSI2_DPHY_RSTZ, 0);
	dw_dphy_write(dphy, R_CSI2_DPHY_SHUTDOWNZ, 0);

	//Select rx_testport (bit 4)
	reg_data = readl(expslv1 + 0x38);
	reg_data &= ~(1 << 8); //tx_rx
        writel(reg_data, expslv1 + 0x38); //rx_dphy_ctrl0

	reg_data = readl(expslv1 + 0x38);
	reg_data |= (1 << 4); //rx testport
        writel(reg_data, expslv1 + 0x38); //rx_dphy_ctrl0

	reg_data = dw_dphy_read(dphy, R_CSI2_DPHY_TST_CTRL0);
	reg_data |= BIT(0); //TestClr Enable
	dw_dphy_write(dphy, R_CSI2_DPHY_TST_CTRL0, reg_data);

	reg_data = readl(expslv1 + 0x38);
	reg_data &= ~(1 << 4); //tx testport
        writel(reg_data, expslv1 + 0x38); //rx_dphy_ctrl0

	reg_data = dw_dphy_read(dphy, R_CSI2_DPHY_TST_CTRL0);
	reg_data |= BIT(0); //TestClr Enable
	dw_dphy_write(dphy, R_CSI2_DPHY_TST_CTRL0, reg_data);

	udelay(10);

	reg_data = readl(expslv1 + 0x38);
	reg_data |= (1 << 4); //rx testport
        writel(reg_data, expslv1 + 0x38); //rx_dphy_ctrl0

	reg_data = dw_dphy_read(dphy, R_CSI2_DPHY_TST_CTRL0);
	reg_data &= ~BIT(0); //TestClr Disable
	dw_dphy_write(dphy, R_CSI2_DPHY_TST_CTRL0, reg_data);

	reg_data = readl(expslv1 + 0x38);
	reg_data &= ~(1 << 4); //tx testport
        writel(reg_data, expslv1 + 0x38); //rx_dphy_ctrl0

	reg_data = dw_dphy_read(dphy, R_CSI2_DPHY_TST_CTRL0);
	reg_data &= ~BIT(0); //TestClr Disable
	dw_dphy_write(dphy, R_CSI2_DPHY_TST_CTRL0, reg_data);

	reg_data = readl(expslv1 + 0x38); //rx_dphy_ctrl0
	reg_data &= ~(MASK(22,16)); //hsfreqrange[22:16]
	reg_data |= ((hsfreqrange << 16) & MASK(22,16));
        writel(reg_data, expslv1 + 0x38); //rx_dphy_ctrl0

	csi_dphy_write_msk(dphy, TX_PLL_13, 0x3, 0, 2);
	csi_dphy_write_msk(dphy, TX_CB_1, 0x2, 0, 2);
	csi_dphy_write_msk(dphy, TX_CB_0, 0x2, 5, 2);

	reg_data = readl(expslv1 + 0x38); //rx_dphy_ctrl0
	reg_data |= (1 << 4); //rx testport
        writel(reg_data, expslv1 + 0x38); //rx_dphy_ctrl0

	csi_dphy_write_msk(dphy, RX_CLKLANE_LANE_6, 0x1, 7, 1);

	if((bitrate/1000000) == 80)
		csi_dphy_write_msk(dphy, RX_SYS_1, 0x85, 0, 8);

	csi_dphy_write_msk(dphy, RX_STARTUP_OVR_2, (uint8_t)osc_freq_target, 0, 8);
	csi_dphy_write_msk(dphy, RX_STARTUP_OVR_3, (uint8_t)osc_freq_target >> 8, 0, 4);
	csi_dphy_write_msk(dphy, RX_STARTUP_OVR_4, 0x1, 0, 1);

	cfgclkfreqrange = (DPHY_FCFG_CLOCK_MHZ - 17) * 4;

	reg_data = readl(expslv1 + 0x38); //rx_dphy_ctrl0
	reg_data &= ~(MASK(31,24));//cfgclkfreqrange[31:24]
	reg_data |= ((cfgclkfreqrange << 24) & MASK(31,24));
        writel(reg_data, expslv1 + 0x38); //rx_dphy_ctrl0

	reg_data = readl(expslv1 + 0x38); //rx_dphy_ctrl0
	reg_data &= ~(MASK(13,12)); //Basedir[13:12]
	reg_data |= (((1U << MIPI_CSI2_N_LANES) - 1) << 12);
        writel(reg_data, expslv1 + 0x38); //rx_dphy_ctrl0

	reg_data = readl(expslv1 + 0x3C); //rx_dphy_ctrl1
	reg_data &= ~(MASK(1,0)); //forcerxmode
	reg_data |= ((1U << MIPI_CSI2_N_LANES) - 1);
        writel(reg_data, expslv1 + 0x3C); //rx_dphy_ctrl1

	udelay(1);
	dw_dphy_write(dphy, R_CSI2_DPHY_SHUTDOWNZ, 0x1);
	udelay(1);
	dw_dphy_write(dphy, R_CSI2_DPHY_RSTZ, 0x1);

	stopstate_check = DPHY_STOPSTATE_CLOCK | DPHY_STOPSTATE_LANE0 |
			DPHY_STOPSTATE_LANE1;

	while(dphy_stopstate(dphy) != stopstate_check)
	{
		pr_info("csi_dphy_loop>> ");
		if(lp_count++ < 100)
			mdelay(50);
		else{
			dev_err(&dphy->phy->dev,
				"CSI DPHY STOP STATE != STOPSTATE CHECK! ");
			dev_err(&dphy->phy->dev,
				"lane_status - 0x%x",
				dw_dphy_read(dphy, R_CSI2_DPHY_STOPSTATE));
			return -1;
		}
	}
	reg_data = readl(expslv1 + 0x3C); //rx_dphy_ctrl1
	reg_data &= ~(MASK(1,0)); //forcerxmode
        writel(reg_data, expslv1 + 0x3C); //rx_dphy_ctrl1

	iounmap(expslv1);

	return 0;
}

static int dw_dphy_configure(struct dw_dphy_rx *dphy)
{
//	struct dw_dphy_rx *dphy = hdphy;
	void __iomem *dsi_base;

	dsi_base = ioremap(0x49032000, 0x1000);
	alif_dsi_phy_init(dsi_base);
	iounmap(dsi_base);

	pr_info("alif_csi_dphy_setup ->400000000\n");
	alif_csi_dphy_setup(dphy, 400000000);

//	dw_dphy_pwr_down(dphy);
#if 0
	if (dphy->dphy_gen == GEN3) {
#if IS_ENABLED(CONFIG_DWC_MIPI_TC_DPHY_GEN3)
		dw_dphy_if_init(dphy);
#endif
		if (dphy->dphy_te_len == BIT12)
			dw_dphy_gen3_12bit_configure(dphy);
		else
			dw_dphy_gen3_8bit_configure(dphy);
	} else {
		dw_dphy_gen2_configure(dphy);
	}
	dw_dphy_pwr_up(dphy);
#endif
	return 0;
}
#if 0
#if IS_ENABLED(CONFIG_DWC_MIPI_TC_DPHY_GEN3)
int dw_dphy_if_set_idelay(struct dw_dphy_rx *dphy, u8 dly, u8 cells)
{
	u32 val = 0;

	dw_dphy_if_write(dphy, IDLYCFG, 0);
	dw_dphy_if_write(dphy, IDLYSEL, cells);
	dw_dphy_if_write(dphy, IDLYCNTINVAL, dly);

	/* Pulse Value Set */
	dw_dphy_if_write(dphy, IDLYCFG, 1);
	usleep_range(10, 20);
	dw_dphy_if_write(dphy, IDLYCFG, 0);

	/* Pulse IDELAY CTRL Reset */
	dw_dphy_if_write(dphy, DPHY1REGRSTN, 0);
	usleep_range(10, 20);
	dw_dphy_if_write(dphy, DPHY1REGRSTN, 1);

	/* Get Value*/
	val = dw_dphy_if_read(dphy, IDLYCNTOUTVAL);

	if (val != dly) {
		dev_vdbg(&dphy->phy->dev,
			 "odelay config failed, set %d get %d", dly, val);
		return -EINVAL;
	}

	return 0;
}

int dw_dphy_if_get_idelay(struct dw_dphy_rx *dphy)
{
	return dw_dphy_if_read(dphy, IDLYCNTOUTVAL);
}

int dw_dphy_if_set_idelay_lane(struct dw_dphy_rx *dphy, u8 dly, u8 lane)
{
	int cell;

	switch (lane) {
	case 0:
		for (cell = 3; cell <= 10; cell++)
			dw_dphy_if_set_idelay(dphy, dly, cell);
		break;
	case 1:
		for (cell = 14; cell <= 21; cell++)
			dw_dphy_if_set_idelay(dphy, dly, cell);
		break;
	case 2:
		for (cell = 24; cell <= 31; cell++)
			dw_dphy_if_set_idelay(dphy, dly, cell);
		break;
	case 3:
		for (cell = 34; cell <= 41; cell++)
			dw_dphy_if_set_idelay(dphy, dly, cell);
		break;
	case 4: /* ALL */
		dw_dphy_if_set_idelay(dphy, dly, 0x7F);
		break;
	default:
		dev_err(&dphy->phy->dev, "Lane Value not recognized\n");
		return -1;
	}
	return 0;
}
#endif
#endif
int dw_dphy_init(struct phy *phy)
{
	struct dw_dphy_rx *dphy = phy_get_drvdata(phy);

	dev_vdbg(&dphy->phy->dev, "Init DPHY.\n");
	dw_dphy_write(dphy, R_CSI2_DPHY_RSTZ, 0);
	dw_dphy_write(dphy, R_CSI2_DPHY_SHUTDOWNZ, 0);

	return 0;
}

static int dw_dphy_set_phy_state(struct dw_dphy_rx *dphy, u32 on)
{
	//u8 hs_freq;
	if (on) {
		pr_info("%s -> ON\n", __func__);
		dw_dphy_configure(dphy);
	} else
		pr_info("%s -> OFF\n", __func__);
#if 0
	dphy->lanes_config = dw_dphy_setup_config(dphy);

	if (dphy->dphy_te_len == BIT12)
		hs_freq = RX_SYS_1;
	else
		hs_freq = HSFREQRANGE_8BIT;

	if (on) {
		dw_dphy_configure(dphy);
		dev_vdbg(&dphy->phy->dev,
			 "HS Code: 0X%x\n", dw_dphy_te_read(dphy, hs_freq));
	} else {
		dw_dphy_write(dphy, R_CSI2_DPHY_SHUTDOWNZ, 0);
		dw_dphy_write(dphy, R_CSI2_DPHY_RSTZ, 0);
	}
#endif
	return 0;
}

int dw_dphy_power_on(struct phy *phy)
{
	struct dw_dphy_rx *dphy = phy_get_drvdata(phy);

	return dw_dphy_set_phy_state(dphy, 1);
}

int dw_dphy_power_off(struct phy *phy)
{
	struct dw_dphy_rx *dphy = phy_get_drvdata(phy);

	return dw_dphy_set_phy_state(dphy, 0);
}

int dw_dphy_reset(struct phy *phy)
{
#if 0
	struct dw_dphy_rx *dphy = phy_get_drvdata(phy);

	dw_dphy_write(dphy, R_CSI2_DPHY_RSTZ, 0);
	usleep_range(100, 200);
	dw_dphy_write(dphy, R_CSI2_DPHY_RSTZ, 1);
#endif
	return 0;
}

