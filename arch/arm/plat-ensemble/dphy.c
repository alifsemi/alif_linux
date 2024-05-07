// SPDX-License-Identifier: GPL-2.0
/*
 * Alif DSI DPHY init code
 *
 * Copyright (C) 2022 Alif Semiconductor.
 * Author: Harith George <harith.g@alifsemi.com>
 *
 */
#include <linux/clk.h>
#include <linux/iopoll.h>
#include <linux/mod_devicetable.h>

#define R_DPHY_RSTZ		(0xA0)
#define R_DPHY_STATUS		(0xB0)
#define DPHY_TST_CTRL0		(0xB4)
#define DPHY_TST_CTRL1		(0xB8)

static inline void dphy_write(void __iomem *base, u32 reg, u32 val)
{
	writel(val, base + reg);
}

static inline u32 dphy_read(void __iomem *base, u32 reg)
{
	return readl(base + reg);
}

static inline void dphy_set(void __iomem *base, u32 reg, u32 mask)
{
	dphy_write(base, reg, dphy_read(base, reg) | mask);
}

static inline void dphy_clear(void __iomem *base, u32 reg, u32 mask)
{
	dphy_write(base, reg, dphy_read(base, reg) & ~mask);
}

static inline void dphy_update_bits(void __iomem *base, u32 reg,
				   u32 mask, u32 val)
{
	dphy_write(base, reg, (dphy_read(base, reg) & ~mask) | val);
}

static void dphy_write_part(void __iomem *base, uint32_t reg_address,
		 uint32_t data, uint8_t shift, uint8_t width)
{
	uint32_t mask = 0;
	uint32_t temp = 0;

	mask = (1 << width) - 1;
	temp = dphy_read(base, reg_address);
	temp &= ~(mask << shift);
	temp |= (data & mask) << shift;
	dphy_write(base, reg_address, temp);
}

void dsi_dphy_write(void __iomem *base, uint16_t address, uint8_t data)
{
	dphy_write(base, DPHY_TST_CTRL0, 0);
	dphy_write(base, DPHY_TST_CTRL1, 0);
	dphy_write_part(base, DPHY_TST_CTRL1, 1, 16, 1);
	dphy_write_part(base, DPHY_TST_CTRL0, 1, 1, 1);
	dphy_write_part(base, DPHY_TST_CTRL1, 0x0, 0, 8);
	dphy_write_part(base, DPHY_TST_CTRL0, 0, 1, 1);
	dphy_write(base, DPHY_TST_CTRL1, 0);
	dphy_write(base, DPHY_TST_CTRL1, (uint8_t)(address >> 8));
	dphy_write_part(base, DPHY_TST_CTRL0, 1, 1, 1);
	dphy_write_part(base, DPHY_TST_CTRL0, 0, 1, 1);
	dphy_write_part(base, DPHY_TST_CTRL1, 1, 16, 1);
	dphy_write_part(base, DPHY_TST_CTRL0, 1, 1, 1);
	dphy_write_part(base, DPHY_TST_CTRL1, ((uint8_t)address), 0, 8);
	dphy_write_part(base, DPHY_TST_CTRL0, 0, 1, 1);
	dphy_write_part(base, DPHY_TST_CTRL1, 0, 16, 1);
	dphy_write_part(base, DPHY_TST_CTRL1, data, 0, 8);
	dphy_write_part(base, DPHY_TST_CTRL0, 1, 1, 1);
	dphy_write_part(base, DPHY_TST_CTRL0, 0, 1, 1);
}

uint8_t dsi_dphy_read(void __iomem *base, uint16_t address)
{
	int read_data;

	dphy_write(base, DPHY_TST_CTRL0, 0);
	dphy_write(base, DPHY_TST_CTRL1, 0);

	dphy_write_part(base, DPHY_TST_CTRL1, 1, 16, 1);
	dphy_write_part(base, DPHY_TST_CTRL0, 1, 1, 1);
	dphy_write_part(base, DPHY_TST_CTRL1, 0x00, 0, 8);
	dphy_write_part(base, DPHY_TST_CTRL0, 0, 1, 1);
	dphy_write(base, DPHY_TST_CTRL1, 0);
	dphy_write(base, DPHY_TST_CTRL1, (uint8_t)(address >> 8));
	dphy_write_part(base, DPHY_TST_CTRL0, 1, 1, 1);
	dphy_write_part(base, DPHY_TST_CTRL0, 0, 1, 1);
	dphy_write_part(base, DPHY_TST_CTRL1, 1, 16, 1);
	dphy_write_part(base, DPHY_TST_CTRL0, 1, 1, 1);
	dphy_write_part(base, DPHY_TST_CTRL1, ((uint8_t)address), 0, 8);
	dphy_write_part(base, DPHY_TST_CTRL0, 0, 1, 1);

	read_data = dphy_read(base, DPHY_TST_CTRL1);
	read_data = read_data >> 8;
	return (uint8_t)read_data;
}

int alif_dsi_phy_init(void __iomem *base)
{
	int rd_data;
	uint8_t r_data_8;
	void __iomem *expslv1;
	// Setting PLL to generate freq 125Mhz for 250Mbps
	// m = 0x271; n = 0x3
	int hsfreqrange = 0x33;
	int pll_soc_m_7_0 = 0x71;
	int pll_soc_m_9_8 = 0x2;
	int pll_soc_n = 0x3;
	uint8_t vco_cntrl = 0x1F;
	int below_450Mbps = 1;

	expslv1 = ioremap(0x4903F000, 0x40);

	//Select rx_testport (bit 4)
	writel(0x110, expslv1 + 0x30);

	// Just toggle
	dphy_write(base, DPHY_TST_CTRL0, 0);
	dphy_write(base, DPHY_TST_CTRL0, 1);
	dphy_write(base, DPHY_TST_CTRL0, 0);

	// Wait for 15 ns;
	udelay(1);

	// Select tx_testport
	writel(0x100, expslv1 + 0x30);

	//[22:16] - hsfreqrange[6:0]
	//[31:24] - cfgclkfreqrange[7:0]
	writel((0x20000100 | (hsfreqrange<<16)), expslv1 + 0x30);

	// Set TX register 0x16A: pll_mpll_prog_rw (bits1:0)
	dsi_dphy_write(base, 0x16a, 0x03);

	// Set TX register 0x1AB: cb_sel_vref_lprx_rw (bits 1:0)
	dsi_dphy_write(base, 0x1ab, 0x06);

	// Set TX register 0x1AA: cb_sel_vrefcd_lprx_rw (bits 6:5)
	dsi_dphy_write(base, 0x1aa, 0x53);

	//When operating as master or when in High-Speed BIST modes,
	//for datarates below 450 Mbps, clkdiv_clk_en must be enabled.
	//To do this, write 1'b1 in TX configuration register with address 0x1AC bit [4].
	if(below_450Mbps) {
		r_data_8 = dsi_dphy_read(base, 0x1ac);
		dsi_dphy_write(base, 0x1ac, r_data_8 | (1 << 4));
	}

	// Set TX register 0x402 to set txclk_term_lowcap_lp00_en_ovr_en_rw and
	// txclk_term_lowcap_lp00_en_ovr_rw(bits 1:0)
	dsi_dphy_write(base, 0x402, 0x2);

	// Refer to table ??Supported rise/fall time limits?? on page 114 and configure TX test control registers
	// with appropriate values for the specified rise/fall time.
	// 500 Mbps and ?? 1 Gbps 1 ns 150 ps - 300 ps 1 12??d657 225 - 291
	// Set bits [5:4] of TX control register 0x272 to 2'b01 to enable slew rate calibration.
	dsi_dphy_write(base, 0x270, 0x91);	// DIG_RDWR_TX_SLEW_5
	dsi_dphy_write(base, 0x271, 0x2);	// DIG_RDWR_TX_SLEW_6
	dsi_dphy_write(base, 0x272, 0x11);	// DIG_RDWR_TX_SLEW_7

	//For the disabled lanes, set sr_finished_ovr_en = 1??b1, sr_finished_ovr = 1??b1, srcal_en_ovr_en = 1??b1
	//through test control registers 0x310, 0x50b, 0x70b, 0x90b, 0xbob
	//bit2, bit1, bit0 are given as "reserved" in the Databook; but program as per the requirement.
	//Since you are not using lane2, lane3 - you can disable those bits in the registers for lane2 and lane3.
	dsi_dphy_write(base, 0x90b, 0x0e);	//lane 2 & 3
	dsi_dphy_write(base, 0xb0b, 0x0e);	// DIG_RDWR_TX_LANE2_SLEWRATE_0

	rd_data = readl(expslv1 + 0x10);
	rd_data |= 1UL << 20;		//pll_soc_clksel [21:20] clkext div selection
	writel(rd_data, expslv1 + 0x10);

	// When PHY is a master (tx_rxz=1'b1) the PLL needs to be configured before D-PHY Start-up.
	// PLL shadow control
	rd_data = readl(expslv1 + 0x10);
	rd_data |= 1UL << 4;		// Selection of PLL configuration mechanism
	writel(rd_data, expslv1 + 0x10);

	// Foutmax [MHz] Foutmin [MHz] F VCO max [MHz] F VCO min [MHz] Output division factor P vco_cntrl[5:3]
	// 500           250           4000            2000            8                        010
	// 1000          500           4000            2000            4                        001
	// 24Mhz > fclkin/N > 8 Mhz
	// 24Mhz > 38.4Mhz/N > 8 Mhz
	// pll_soc_m [9:0]  Feedback multiplication ratio M (40 to 625)
	// pll_soc_n [15:12] Input frequency division ratio N (1 to 16)
	// M = 625 = 0x271 ; N = 4 ; n = 4 - 1
	// fout = 500Mhz for 1 G bit rate
	// fout = M/N*1/2*fclkin*1/P = M/N*1/2*38.4Mhz*1/8
	dsi_dphy_write(base, 0x179, pll_soc_m_7_0); // DIG_RDWR_TX_PLL_28
	dsi_dphy_write(base, 0x17a, pll_soc_m_9_8); // DIG_RDWR_TX_PLL_29
	//0 pll_m_ovr_en_rw : PLL feedback divider override enable
	dsi_dphy_write(base, 0x17b, 0x1);	// DIG_RDWR_TX_PLL_30
	//7 pll_n_ovr_en_rw : PLL input divider override enable
	//6:3 pll_n_ovr_rw__3__0__ : PLL input divider override
	dsi_dphy_write(base, 0x178, (0x80|(pll_soc_n<<3))); // DIG_RDWR_TX_PLL_27

	// vco_cntrl[5:0] - pll_vco_cntrl_ovr_rw[5:0], pll_vco_cntrl_ovr_en_rw ?? 0x17b
	// pll_vco_cntrl_ovr_en_rw          : PLL VCO control override enable
	// 6:1 pll_vco_cntrl_ovr_rw__5__0__ : PLL VCO control override
	dsi_dphy_write(base, 0x17b, (0x81 | vco_cntrl<<1)); // DIG_RDWR_TX_PLL_30

	// cpbias_cntrl[6:0] - pll_cpbias_cntrl_rw[6:0] ?? 0x15e
	// 6:0 pll_cpbias_cntrl_rw__6__0__ : PLL Charge Pump Bias Control
	dsi_dphy_write(base, 0x15e, 0x0);	// DIG_RDWR_TX_PLL_1

	// gmp_cntrl[1:0] - pll_gmp_cntrl_rw[1:0] ?? 0x162
	// int_cntrl[5:0] - pll_int_cntrl_rw[5:0] ?? 0x162
	// dphy4txtester_DIG_RDWR_TX_PLL_5
	// 7:2 pll_int_cntrl_rw__5__0__ : PLL Integral Charge Pump control
	// 1:0 pll_gmp_cntrl_rw__1__0__ : PLL GMP Control
	dsi_dphy_write(base, 0x162, 0x11);

	// prop_cntrl[5:0] - pll_prop_cntrl_rw[5:0] : PLL Proportional Charge Pump control
	dsi_dphy_write(base, 0x16e, 0x10);  // DIG_RDWR_TX_PLL_17

	// 2:0 cb_vref_mpll: PLL reference voltage control
	dsi_dphy_write(base, 0x1ad, 0x02); // DIG_RDWR_TX_CB_3

	//shadow registers interface (see ??Initialization?? on page 33 for additional details);
	// Set basedir_n = 1'b0; Set forcerxmode_n = 1'b0;
	writel(0x0, expslv1 + 0x34);
	udelay(1);			 // Wait for 15 ns;
	dphy_write(base, R_DPHY_RSTZ, 0x4);// Set enable_n and enableclk=1'b1;
	udelay(1);			 // Wait 5 ns;
	dphy_write(base, R_DPHY_RSTZ, 0x5);// Set shutdownz=1'b1;
	udelay(1); 			 // Wait 5 ns;
	dphy_write(base, R_DPHY_RSTZ, 0x7);// Set rstz=1'b1;

	// 7 pll_pwron_ovr_rw    : PLL poweron override
	// 6 pll_pwron_ovr_en_rw : PLL poweron override enable control
	dsi_dphy_write(base, 0x16e, 0xd0); // DIG_RDWR_TX_PLL_17

	// Wait for clk and data lanes to be in stop state
	do {
		mdelay(20);
	} while((dphy_read(base, R_DPHY_STATUS) & 0x94) != 0x94);
	return 0;
}
