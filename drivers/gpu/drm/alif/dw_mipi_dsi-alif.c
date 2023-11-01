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

int alif_dsi_phy_init(void __iomem *dsi);
static int dw_mipi_dsi_phy_init(void *priv_data)
{
	struct dw_mipi_dsi_alif *dsip = priv_data;

	return alif_dsi_phy_init(dsip->base);
}

static int
dw_mipi_dsi_get_lane_mbps(void *priv_data, const struct drm_display_mode *mode,
			  unsigned long mode_flags, u32 lanes, u32 format,
			  unsigned int *lane_mbps)
{
	struct dw_mipi_dsi_alif *dsi = priv_data;
	unsigned int pll_in_khz, pll_out_khz;
	int bpp;

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

	dsi->hw_version = readl(dsi->base + DSI_VERSION) & VERSION;

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
