// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (C) 2023 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 * Author: Yogender Kumar Arya <yogender.kumar@alifsemi.com>
 */

#include <linux/io.h>

#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/export.h>
#include <linux/dma-buf.h>
#include <linux/dma-mapping.h>

// Placeholder for DRM includes.
#include "alif_drm_driver.h"
#include "regs_d2d.h"

/*
 * D2D device IRQ Handler.
 */
static irqreturn_t d2d_irq_handler(int irq, void *arg)
{
	struct d2d_device *d2d = (struct d2d_device *)arg;
	u32 status;
	u32 temp = 0;
	u32 cache_flush = 0;

	status = D2D_REG_READ(d2d, D2_STATUS);
	if (status & D2C_IRQ_ENUM) {
		temp |= D2IRQCTL_CLR_FINISH_ENUM;

		/*
		 * Clear D2D caches as a part of Enumeration finish interrupt
		 * handling.
		 */
		if (d2d->hwrevision & D2_HWREVISION_FEATURES_FB_CACHE)
			cache_flush |= D2C_CACHECTL_FLUSH_FB;
		if (d2d->hwrevision & D2_HWREVISION_FEATURES_TX_CACHE)
			cache_flush |= D2C_CACHECTL_FLUSH_TX;
		D2D_REG_WRITE(d2d, D2_CACHECTL, cache_flush);
	}
	if (status & D2C_IRQ_DLIST)
		temp |= D2IRQCTL_CLR_FINISH_DLIST;
	if (status & D2C_IRQ_BUS_ERROR)
		temp |= D2IRQCTL_CLR_BUS_ERROR;
	temp |= D2IRQCTL_ENABLE_FINISH_ENUM |
		D2IRQCTL_ENABLE_FINISH_DLIST |
		D2IRQCTL_ENABLE_BUS_ERROR;
	/*
	 * Clear and re-enable the interrupts.
	 */
	D2D_REG_WRITE(d2d, D2_IRQCTL, temp);
	return IRQ_HANDLED;
}

int d2d_init_irq(struct d2d_device *d2d)
{
	struct device *dev = &d2d->pdev->dev;
	u32 rc = 0;

	rc = devm_request_irq(dev,
			d2d->irq,
			d2d_irq_handler,
			0,
			dev_name(dev),
			d2d);
	if (rc < 0) {
		dev_dbg(dev, "Failed to register IRQ.\n");
		return rc;
	}
	/*
	 * Enabling all the three source of interrupts on D/AVE-2D device.
	 */
	D2D_REG_WRITE(d2d, D2_IRQCTL,
			D2IRQCTL_ENABLE_FINISH_ENUM	|
			D2IRQCTL_ENABLE_FINISH_DLIST	|
			D2IRQCTL_ENABLE_BUS_ERROR	|
			D2IRQCTL_CLR_FINISH_ENUM	|
			D2IRQCTL_CLR_FINISH_DLIST	|
			D2IRQCTL_CLR_BUS_ERROR);

	return 0;
}

static void d2d_deinit_irq(struct d2d_device *d2d)
{
	struct platform_device *pdev = d2d->pdev;

	D2D_REG_WRITE(d2d, D2_IRQCTL, 0);
	devm_free_irq(&pdev->dev, d2d->irq, d2d);
}

int d2d_read_reg_validate(struct d2d_device *d2d, struct drm_d2d_reg *args)
{
	struct platform_device *pdev = d2d->pdev;

	if (args->offset > D2_MAXREGISTER) {
		dev_dbg(&pdev->dev, "Invalid register offset!\n");
		return -EINVAL;
	}

	args->value = D2D_REG_READ(d2d, args->offset);
	return 0;
}

int d2d_write_reg_validate(struct d2d_device *d2d, struct drm_d2d_reg *args)
{
	struct platform_device *pdev = d2d->pdev;

	if (args->offset > D2_MAXREGISTER) {
		dev_dbg(&pdev->dev, "Invalid register offset!\n");
		return -EINVAL;
	}

	D2D_REG_WRITE(d2d, args->offset, args->value);
	return 0;
}

static int d2d_init_device_config_regs(struct d2d_device *d2d)
{
	struct device *dev = &d2d->pdev->dev;
	u32 temp_reg = 0;
	u32 d2d_type = 0;

	d2d->hwrevision = D2D_REG_READ(d2d, D2_HWREVISION);
	dev_info(dev, "HW revision number - 0x%x", d2d->hwrevision);

	/*
	 * Setup the Perf Counters.
	 */
	d2d_type = (d2d->hwrevision & D2_HWREVISION_HWTYPE_MASK) >>
		D2_HWREVISION_HWTYPE_SHIFT;
	if ((d2d_type !=  D2_HWREVISION_HWTYPE_STANDARD) &&
	    (d2d_type != D2_HWREVISION_HWTYPE_LIGHT)) {
		dev_info(dev, "Unknown D2D device detected. Aborting probe!\n");
		return -ENODEV;
	}

	if (d2d->hwrevision & D2_HWREVISION_FEATURES_PERFCOUNT) {
		/*
		 * Setup the perf counters only when the version of D/AVE HW is
		 * Standard and the perf counters are available in the h/w caps.
		 */
		temp_reg = ((D2PC_NONE << D2_PERFTRIGGER_1_SHIFT) &
				D2_PERFTRIGGER_1_MASK) |
			   ((D2PC_NONE << D2_PERFTRIGGER_2_SHIFT) &
				D2_PERFTRIGGER_2_MASK);
		dev_info(dev, "Setting PERFTRIGGER reg - 0x%x\n", temp_reg);
		D2D_REG_WRITE(d2d, D2_PERFTRIGGER, temp_reg);
	}

	/*
	 * Setup the Cache control Registers.
	 */
	temp_reg = 0;
	if (d2d->hwrevision & D2_HWREVISION_FEATURES_FB_CACHE)
		temp_reg |= D2C_CACHECTL_ENABLE_FB | D2C_CACHECTL_FLUSH_FB;
	if (d2d->hwrevision & D2_HWREVISION_FEATURES_TX_CACHE)
		temp_reg |=  D2C_CACHECTL_ENABLE_TX | D2C_CACHECTL_FLUSH_TX;
	dev_info(dev, "CACHE-CTL reg - 0x%x\n", temp_reg);
	D2D_REG_WRITE(d2d, D2_CACHECTL, temp_reg);

	return 0;
}

u32 d2d_get_clk_rate(struct d2d_device *d2d)
{
	return d2d->clk_freq;
}

static int d2d_device_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *res;
	struct d2d_device *d2d = NULL;
	int rc = 0;
	struct clk *sys_clk = NULL;

	/*
	 * Allocating driver private for D2D.
	 */
	d2d = devm_kzalloc(dev, sizeof(struct d2d_device), GFP_KERNEL);
	if (!d2d)
		return -ENOMEM;

	dev_set_drvdata(dev, d2d);
	d2d->pdev  = pdev;

	sys_clk = devm_clk_get(dev, NULL);
	if (IS_ERR(sys_clk))
		dev_dbg(dev, "Unable to parse CLK!\n");
	else {
		d2d->clk_freq = clk_get_rate(sys_clk);
		dev_info(dev, "GPU 2D Clk Frequency - %d", d2d->clk_freq);
	}

	/*
	 * Get the IRQ for the device.
	 */
	d2d->irq = platform_get_irq(pdev, 0);
	if (d2d->irq < 0) {
		dev_dbg(dev, "Parsing IRQ from DT failed!\n");
		return d2d->irq;
	}
	dev_info(dev, "IRQ number - %d\n", d2d->irq);

	/*
	 * Get the physical I/O range of the device and map it to a virtual
	 * base pointer.
	 */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_dbg(dev, "Failed to get MMIO for D/AVE IP!\n");
		return -ENODEV;
	}
	d2d->phy_base = res->start;
	d2d->size = res->end - res->start + 1;

	d2d->regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(d2d->regs)) {
		dev_dbg(dev, "Failed to map MMIO regs! - %ld\n",
			PTR_ERR(d2d->regs));
		return PTR_ERR(d2d->regs);
	}
	dev_info(dev, "Mapped IO range 0x%x (size - 0x%x) to 0x%x\n",
		d2d->phy_base, d2d->size, (u32)d2d->regs);

	/*
	 * Init the configuration registers for the D2D device.
	 */
	rc = d2d_init_device_config_regs(d2d);
	if (rc != 0) {
		dev_dbg(dev, "Device regs config failed!\n");
		return rc;
	}

	rc = d2d_init_irq(d2d);
	if (rc)
		return rc;

	dev_info(dev, "Init DRM core interface.\n");
	rc = drm_init(&pdev->dev);
	if (rc != 0) {
		dev_dbg(dev, "DRM Init failed. rc - %d", rc);
		return rc;
	}

	return 0;
}

static int d2d_device_remove(struct platform_device *pdev)
{
	struct d2d_device *d2d = platform_get_drvdata(pdev);
	struct alif_dev *alif = d2d->alif;

	drm_free(alif);
	d2d_deinit_irq(d2d);

	devm_kfree(&pdev->dev, d2d);
	dev_set_drvdata(&pdev->dev, NULL);
	return 0;
}

static const struct of_device_id of_d2d_match[] = {
	{ .compatible = D2D_DEVICE_COMPATIBLE, .data = NULL },
	{ }
};
MODULE_DEVICE_TABLE(of, of_d2d_match);

struct platform_driver d2d_platform_driver = {
	.driver = {
		.name = D2D_DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_d2d_match,
	},
	.probe = d2d_device_probe,
	.remove = d2d_device_remove,
};

module_platform_driver(d2d_platform_driver);

MODULE_DESCRIPTION("TES ALIF 2D GPU");
MODULE_AUTHOR("Yogender Kumar Arya <yogender.kumar@alifsemi.com>");
MODULE_LICENSE("GPL");
