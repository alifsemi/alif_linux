// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) Alif Semiconductor 2022
 *
 * Author: Harith George <harith.g@alifsemi.com>
 */

#include <linux/clk.h>
#include <linux/iopoll.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>

#include <drm/bridge/dw_mipi_dsi.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_print.h>

/* IP version 1.41 */
#define HWVER_141		0x31343100

/* DSI digital registers & bit definitions */
#define DSI_VERSION		0x00
#define VERSION			GENMASK(31, 8)

#define R_DPHY_RSTZ		(0xA0)
#define R_DPHY_STATUS  	   	(0xB0)
#define DPHY_TST_CTRL0 		(0xB4)
#define DPHY_TST_CTRL1	  	(0xB8)

#define LANE_MIN_KBPS		31250
#define LANE_MAX_KBPS		500000

struct dw_mipi_dsi_alif {
	void __iomem *base;
	struct clk *pllref_clk;
	struct dw_mipi_dsi *dsi;
	u32 hw_version;
	int lane_min_kbps;
	int lane_max_kbps;
};

static inline void dsi_write(struct dw_mipi_dsi_alif *dsi, u32 reg, u32 val)
{
	writel(val, dsi->base + reg);
}

static inline u32 dsi_read(struct dw_mipi_dsi_alif *dsi, u32 reg)
{
	return readl(dsi->base + reg);
}

static inline void dsi_set(struct dw_mipi_dsi_alif *dsi, u32 reg, u32 mask)
{
	dsi_write(dsi, reg, dsi_read(dsi, reg) | mask);
}

static inline void dsi_clear(struct dw_mipi_dsi_alif *dsi, u32 reg, u32 mask)
{
	dsi_write(dsi, reg, dsi_read(dsi, reg) & ~mask);
}

static inline void dsi_update_bits(struct dw_mipi_dsi_alif *dsi, u32 reg,
				   u32 mask, u32 val)
{
	dsi_write(dsi, reg, (dsi_read(dsi, reg) & ~mask) | val);
}

void dsi_write_part(struct dw_mipi_dsi_alif *dsi, uint32_t reg_address,
		 uint32_t data, uint8_t shift, uint8_t width)
{
	uint32_t mask = 0;
	uint32_t temp = 0;

	mask = (1 << width) - 1;
	temp = dsi_read(dsi, reg_address);
	temp &= ~(mask << shift);
	temp |= (data & mask) << shift;
	dsi_write(dsi, reg_address, temp);
}

void dsi_dphy_write(struct dw_mipi_dsi_alif *dsi, uint16_t address, uint8_t data)
{
	dsi_write(dsi, DPHY_TST_CTRL0, 0);
	dsi_write(dsi, DPHY_TST_CTRL1, 0);
	dsi_write_part(dsi, DPHY_TST_CTRL1, 1, 16, 1);
	dsi_write_part(dsi, DPHY_TST_CTRL0, 1, 1, 1);
	dsi_write_part(dsi, DPHY_TST_CTRL1, 0x0, 0, 8);
	dsi_write_part(dsi, DPHY_TST_CTRL0, 0, 1, 1);
	dsi_write(dsi, DPHY_TST_CTRL1, 0);
	dsi_write(dsi, DPHY_TST_CTRL1, (uint8_t)(address >> 8));
	dsi_write_part(dsi, DPHY_TST_CTRL0, 1, 1, 1);
	dsi_write_part(dsi, DPHY_TST_CTRL0, 0, 1, 1);
	dsi_write_part(dsi, DPHY_TST_CTRL1, 1, 16, 1);
	dsi_write_part(dsi, DPHY_TST_CTRL0, 1, 1, 1);
	dsi_write_part(dsi, DPHY_TST_CTRL1, ((uint8_t)address), 0, 8);
	dsi_write_part(dsi, DPHY_TST_CTRL0, 0, 1, 1);
	dsi_write_part(dsi, DPHY_TST_CTRL1, 0, 16, 1);
	dsi_write_part(dsi, DPHY_TST_CTRL1, data, 0, 8);
	dsi_write_part(dsi, DPHY_TST_CTRL0, 1, 1, 1);
	dsi_write_part(dsi, DPHY_TST_CTRL0, 0, 1, 1);
}

uint8_t dsi_dphy_read(struct dw_mipi_dsi_alif *dsi, uint16_t address)
{
	int read_data;

	dsi_write(dsi, DPHY_TST_CTRL0, 0);
	dsi_write(dsi, DPHY_TST_CTRL1, 0);

	dsi_write_part(dsi, DPHY_TST_CTRL1, 1, 16, 1);
	dsi_write_part(dsi, DPHY_TST_CTRL0, 1, 1, 1);
	dsi_write_part(dsi, DPHY_TST_CTRL1, 0x00, 0, 8);
	dsi_write_part(dsi, DPHY_TST_CTRL0, 0, 1, 1);
	dsi_write(dsi, DPHY_TST_CTRL1, 0);
	dsi_write(dsi, DPHY_TST_CTRL1, (uint8_t)(address >> 8));
	dsi_write_part(dsi, DPHY_TST_CTRL0, 1, 1, 1);
	dsi_write_part(dsi, DPHY_TST_CTRL0, 0, 1, 1);
	dsi_write_part(dsi, DPHY_TST_CTRL1, 1, 16, 1);
	dsi_write_part(dsi, DPHY_TST_CTRL0, 1, 1, 1);
	dsi_write_part(dsi, DPHY_TST_CTRL1, ((uint8_t)address), 0, 8);
	dsi_write_part(dsi, DPHY_TST_CTRL0, 0, 1, 1);

	read_data = dsi_read(dsi, DPHY_TST_CTRL1);
	read_data = read_data >> 8;
	printk("dsi_dphy_read: addr 0x%x data 0x%x\n", address, (uint8_t)read_data);
	return (uint8_t)read_data;
}

static int dsi_pll_get_clkout_khz(int clkin_khz, int idf, int ndiv, int odf)
{
	int divisor = idf * odf;

	/* prevent from division by 0 */
	if (!divisor)
		return 0;

	return DIV_ROUND_CLOSEST(clkin_khz * ndiv, divisor);
}

static int dsi_pll_get_params(struct dw_mipi_dsi_alif *dsi,
			      int clkin_khz, int clkout_khz,
			      int *idf, int *ndiv, int *odf)
{
/* DSI wrapper registers & bit definitions */
#define IDF_MIN		1
#define IDF_MAX		7
#define NDIV_MIN	10
#define NDIV_MAX	125
#define ODF_MIN		1
#define ODF_MAX		8

	int i, o, n, n_min, n_max;
	int fvco_min, fvco_max, delta, best_delta; /* all in khz */

	/* Early checks preventing division by 0 & odd results */
	if (clkin_khz <= 0 || clkout_khz <= 0)
		return -EINVAL;

	fvco_min = dsi->lane_min_kbps * 2 * ODF_MAX;
	fvco_max = dsi->lane_max_kbps * 2 * ODF_MIN;

	best_delta = 1000000; /* big started value (1000000khz) */

	for (i = IDF_MIN; i <= IDF_MAX; i++) {
		/* Compute ndiv range according to Fvco */
		n_min = ((fvco_min * i) / (2 * clkin_khz)) + 1;
		n_max = (fvco_max * i) / (2 * clkin_khz);

		/* No need to continue idf loop if we reach ndiv max */
		if (n_min >= NDIV_MAX)
			break;

		/* Clamp ndiv to valid values */
		if (n_min < NDIV_MIN)
			n_min = NDIV_MIN;
		if (n_max > NDIV_MAX)
			n_max = NDIV_MAX;

		for (o = ODF_MIN; o <= ODF_MAX; o *= 2) {
			n = DIV_ROUND_CLOSEST(i * o * clkout_khz, clkin_khz);
			/* Check ndiv according to vco range */
			if (n < n_min || n > n_max)
				continue;
			/* Check if new delta is better & saves parameters */
			delta = dsi_pll_get_clkout_khz(clkin_khz, i, n, o) -
				clkout_khz;
			if (delta < 0)
				delta = -delta;
			if (delta < best_delta) {
				*idf = i;
				*ndiv = n;
				*odf = o;
				best_delta = delta;
			}
			/* fast return in case of "perfect result" */
			if (!delta)
				return 0;
		}
	}

	return 0;
}

static int dw_mipi_dsi_phy_init(void *priv_data)
{
	struct dw_mipi_dsi_alif *dsi = priv_data;
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
	dsi_write(dsi, DPHY_TST_CTRL0, 0);
	dsi_write(dsi, DPHY_TST_CTRL0, 1);
	dsi_write(dsi, DPHY_TST_CTRL0, 0);

	// Wait for 15 ns;
	udelay(1);

	// Select tx_testport
	writel(0x100, expslv1 + 0x30);

	//[22:16] - hsfreqrange[6:0]
	//[31:24] - cfgclkfreqrange[7:0]
	writel((0x20000100 | (hsfreqrange<<16)), expslv1 + 0x30);

	// Set TX register 0x16A: pll_mpll_prog_rw (bits1:0)
	dsi_dphy_write(dsi, 0x16a, 0x03);

	// Set TX register 0x1AB: cb_sel_vref_lprx_rw (bits 1:0)
	dsi_dphy_write(dsi, 0x1ab, 0x06);

	// Set TX register 0x1AA: cb_sel_vrefcd_lprx_rw (bits 6:5)
	dsi_dphy_write(dsi, 0x1aa, 0x53);

	//When operating as master or when in High-Speed BIST modes,
	//for datarates below 450 Mbps, clkdiv_clk_en must be enabled.
	//To do this, write 1'b1 in TX configuration register with address 0x1AC bit [4].
	if(below_450Mbps) {
		r_data_8 = dsi_dphy_read(dsi, 0x1ac);
		dsi_dphy_write(dsi, 0x1ac, r_data_8 | (1 << 4));
	}

	// Set TX register 0x402 to set txclk_term_lowcap_lp00_en_ovr_en_rw and
	// txclk_term_lowcap_lp00_en_ovr_rw(bits 1:0)
	dsi_dphy_write(dsi, 0x402, 0x2);

	// Refer to table ??Supported rise/fall time limits?? on page 114 and configure TX test control registers
	// with appropriate values for the specified rise/fall time.
	// 500 Mbps and ?? 1 Gbps 1 ns 150 ps - 300 ps 1 12??d657 225 - 291
	// Set bits [5:4] of TX control register 0x272 to 2'b01 to enable slew rate calibration.
	dsi_dphy_write(dsi, 0x270, 0x91);	// DIG_RDWR_TX_SLEW_5
	dsi_dphy_write(dsi, 0x271, 0x2);	// DIG_RDWR_TX_SLEW_6
	dsi_dphy_write(dsi, 0x272, 0x11);	// DIG_RDWR_TX_SLEW_7

	//For the disabled lanes, set sr_finished_ovr_en = 1??b1, sr_finished_ovr = 1??b1, srcal_en_ovr_en = 1??b1
	//through test control registers 0x310, 0x50b, 0x70b, 0x90b, 0xbob
	//bit2, bit1, bit0 are given as "reserved" in the Databook; but program as per the requirement.
	//Since you are not using lane2, lane3 - you can disable those bits in the registers for lane2 and lane3.
	dsi_dphy_write(dsi, 0x90b, 0x0e);	//lane 2 & 3
	dsi_dphy_write(dsi, 0xb0b, 0x0e);	// DIG_RDWR_TX_LANE2_SLEWRATE_0

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
	dsi_dphy_write(dsi, 0x179, pll_soc_m_7_0); // DIG_RDWR_TX_PLL_28
	dsi_dphy_write(dsi, 0x17a, pll_soc_m_9_8); // DIG_RDWR_TX_PLL_29
	//0 pll_m_ovr_en_rw : PLL feedback divider override enable
	dsi_dphy_write(dsi, 0x17b, 0x1);	// DIG_RDWR_TX_PLL_30
	//7 pll_n_ovr_en_rw : PLL input divider override enable
	//6:3 pll_n_ovr_rw__3__0__ : PLL input divider override
	dsi_dphy_write(dsi, 0x178, (0x80|(pll_soc_n<<3))); // DIG_RDWR_TX_PLL_27

	// vco_cntrl[5:0] - pll_vco_cntrl_ovr_rw[5:0], pll_vco_cntrl_ovr_en_rw ?? 0x17b
	// pll_vco_cntrl_ovr_en_rw          : PLL VCO control override enable
	// 6:1 pll_vco_cntrl_ovr_rw__5__0__ : PLL VCO control override
	dsi_dphy_write(dsi, 0x17b, (0x81 | vco_cntrl<<1)); // DIG_RDWR_TX_PLL_30

	// cpbias_cntrl[6:0] - pll_cpbias_cntrl_rw[6:0] ?? 0x15e
	// 6:0 pll_cpbias_cntrl_rw__6__0__ : PLL Charge Pump Bias Control
	dsi_dphy_write(dsi, 0x15e, 0x0);	// DIG_RDWR_TX_PLL_1

	// gmp_cntrl[1:0] - pll_gmp_cntrl_rw[1:0] ?? 0x162
	// int_cntrl[5:0] - pll_int_cntrl_rw[5:0] ?? 0x162
	// dphy4txtester_DIG_RDWR_TX_PLL_5
	// 7:2 pll_int_cntrl_rw__5__0__ : PLL Integral Charge Pump control
	// 1:0 pll_gmp_cntrl_rw__1__0__ : PLL GMP Control
	dsi_dphy_write(dsi, 0x162, 0x11);

	// prop_cntrl[5:0] - pll_prop_cntrl_rw[5:0] : PLL Proportional Charge Pump control
	dsi_dphy_write(dsi, 0x16e, 0x10);  // DIG_RDWR_TX_PLL_17

	// 2:0 cb_vref_mpll: PLL reference voltage control
	dsi_dphy_write(dsi, 0x1ad, 0x02); // DIG_RDWR_TX_CB_3

	//shadow registers interface (see ??Initialization?? on page 33 for additional details);
	// Set basedir_n = 1'b0; Set forcerxmode_n = 1'b0;
	writel(0x0, expslv1 + 0x34);
	udelay(1);			 // Wait for 15 ns;
	dsi_write(dsi, R_DPHY_RSTZ, 0x4);// Set enable_n and enableclk=1'b1;
	udelay(1);			 // Wait 5 ns;
	dsi_write(dsi, R_DPHY_RSTZ, 0x5);// Set shutdownz=1'b1;
	udelay(1); 			 // Wait 5 ns;
	dsi_write(dsi, R_DPHY_RSTZ, 0x7);// Set rstz=1'b1;

	// 7 pll_pwron_ovr_rw    : PLL poweron override
	// 6 pll_pwron_ovr_en_rw : PLL poweron override enable control
	dsi_dphy_write(dsi, 0x16e, 0xd0); // DIG_RDWR_TX_PLL_17

	// Wait for clk and data lanes to be in stop state
	do {
		mdelay(20);
		printk("loop->");
	} while((dsi_read(dsi, R_DPHY_STATUS) & 0x94) != 0x94);
	return 0;
}

static int
dw_mipi_dsi_get_lane_mbps(void *priv_data, const struct drm_display_mode *mode,
			  unsigned long mode_flags, u32 lanes, u32 format,
			  unsigned int *lane_mbps)
{
	struct dw_mipi_dsi_alif *dsi = priv_data;
	unsigned int idf, ndiv, odf, pll_in_khz, pll_out_khz;
	int ret, bpp;

	dsi->lane_min_kbps = LANE_MIN_KBPS;
	dsi->lane_max_kbps = LANE_MAX_KBPS;

	pll_in_khz = (unsigned int)(clk_get_rate(dsi->pllref_clk) / 1000);

	/* Compute requested pll out */
	bpp = mipi_dsi_pixel_format_to_bpp(format);

	/* TODO Ideally the max pll_out_khz below should be
	 * calculated based on panel max clk.. Hardcoding to
	 * 500Mhz below
	 */
	pll_out_khz = LANE_MAX_KBPS * bpp / lanes;

	if (pll_out_khz > dsi->lane_max_kbps) {
		pll_out_khz = dsi->lane_max_kbps;
		DRM_WARN("Warning max phy mbps is used\n");
	}
	if (pll_out_khz < dsi->lane_min_kbps) {
		pll_out_khz = dsi->lane_min_kbps;
		DRM_WARN("Warning min phy mbps is used\n");
	}

#if 0
	/* Compute best pll parameters */
	idf = 0;
	ndiv = 0;
	odf = 0;
	ret = dsi_pll_get_params(dsi, pll_in_khz, pll_out_khz,
				 &idf, &ndiv, &odf);
	if (ret)
		DRM_WARN("Warning dsi_pll_get_params(): bad params\n");

	/* Get the adjusted pll out value */
	pll_out_khz = dsi_pll_get_clkout_khz(pll_in_khz, idf, ndiv, odf);
#endif
	/* TODO Ideally set the PLL registers here.. */

	*lane_mbps = pll_out_khz / 1000;

	DRM_DEBUG_DRIVER("pll_in %ukHz pll_out %ukHz lane_mbps %uMHz\n",
			 pll_in_khz, pll_out_khz, *lane_mbps);

printk("#Alif HARD SETTING lane_mbps to 250\n");
	*lane_mbps = 250;
	return 0;
}

static const struct dw_mipi_dsi_phy_ops dw_mipi_dsi_alif_phy_ops = {
	.init = dw_mipi_dsi_phy_init,
	.get_lane_mbps = dw_mipi_dsi_get_lane_mbps,
};

static struct dw_mipi_dsi_plat_data dw_mipi_dsi_alif_plat_data = {
	.max_data_lanes = 2,
	.phy_ops = &dw_mipi_dsi_alif_phy_ops,
};

static const struct of_device_id dw_mipi_dsi_alif_dt_ids[] = {
	{ .compatible = "alif,ensemble-dsi", .data = &dw_mipi_dsi_alif_plat_data, },
	{ },
};
MODULE_DEVICE_TABLE(of, dw_mipi_dsi_alif_dt_ids);

static int dw_mipi_dsi_alif_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct dw_mipi_dsi_alif *dsi;
	struct resource *res;
	int ret;

	dsi = devm_kzalloc(dev, sizeof(*dsi), GFP_KERNEL);
	if (!dsi)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	dsi->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(dsi->base)) {
		ret = PTR_ERR(dsi->base);
		DRM_ERROR("Unable to get dsi registers %d\n", ret);
		return ret;
	}

	dsi->pllref_clk = devm_clk_get(dev, "ref");
	if (IS_ERR(dsi->pllref_clk)) {
		ret = PTR_ERR(dsi->pllref_clk);
		DRM_ERROR("Unable to get pll reference clock: %d\n", ret);
		goto err_clk_get;
	}

	ret = clk_prepare_enable(dsi->pllref_clk);
	if (ret) {
		DRM_ERROR("Failed to enable pllref_clk: %d\n", ret);
		goto err_clk_get;
	}

	dsi->hw_version = dsi_read(dsi, DSI_VERSION) & VERSION;

	if (dsi->hw_version != HWVER_141) {
		ret = -ENODEV;
		DRM_ERROR("bad dsi hardware version\n");
		goto err_dsi_probe;
	}
	dw_mipi_dsi_alif_plat_data.base = dsi->base;
	dw_mipi_dsi_alif_plat_data.priv_data = dsi;

	platform_set_drvdata(pdev, dsi);

	dsi->dsi = dw_mipi_dsi_probe(pdev, &dw_mipi_dsi_alif_plat_data);
	if (IS_ERR(dsi->dsi)) {
		ret = PTR_ERR(dsi->dsi);
		DRM_ERROR("Failed to initialize mipi dsi host: %d\n", ret);
		goto err_dsi_probe;
	}

	return 0;

err_dsi_probe:
	clk_disable_unprepare(dsi->pllref_clk);
err_clk_get:

	return ret;
}

static int dw_mipi_dsi_alif_remove(struct platform_device *pdev)
{
	struct dw_mipi_dsi_alif *dsi = platform_get_drvdata(pdev);

	dw_mipi_dsi_remove(dsi->dsi);
	clk_disable_unprepare(dsi->pllref_clk);
	return 0;
}

static int __maybe_unused dw_mipi_dsi_alif_suspend(struct device *dev)
{
	struct dw_mipi_dsi_alif *dsi = dw_mipi_dsi_alif_plat_data.priv_data;

	clk_disable_unprepare(dsi->pllref_clk);
	return 0;
}

static int __maybe_unused dw_mipi_dsi_alif_resume(struct device *dev)
{
	struct dw_mipi_dsi_alif *dsi = dw_mipi_dsi_alif_plat_data.priv_data;
	int ret;

	ret = clk_prepare_enable(dsi->pllref_clk);
	if (ret) {
		DRM_ERROR("Failed to enable pllref_clk: %d\n", ret);
		return ret;
	}
	return 0;
}

static const struct dev_pm_ops dw_mipi_dsi_alif_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(dw_mipi_dsi_alif_suspend,
				dw_mipi_dsi_alif_resume)
};

static struct platform_driver dw_mipi_dsi_alif_driver = {
	.probe		= dw_mipi_dsi_alif_probe,
	.remove		= dw_mipi_dsi_alif_remove,
	.driver		= {
		.of_match_table = dw_mipi_dsi_alif_dt_ids,
		.name	= "alif-display-dsi",
		.pm = &dw_mipi_dsi_alif_pm_ops,
	},
};

module_platform_driver(dw_mipi_dsi_alif_driver);

MODULE_AUTHOR("Harith George <harith.g@alifsemi.com>");
MODULE_DESCRIPTION("Alif Semiconductor DW MIPI DSI host controller driver");
MODULE_LICENSE("GPL v2");
