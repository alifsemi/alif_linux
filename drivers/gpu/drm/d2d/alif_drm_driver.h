/* SPDX-License-Identifier: GPL-2.0-only */
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

#ifndef _ALIF_DRM_DRIVER_H_
#define _ALIF_DRM_DRIVER_H_

#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/bitops.h>
#include <linux/of_device.h>
#include <linux/clk.h>

#include <drm/drm.h>
#include <drm/drmP.h>
#include <drm/drm_drv.h>
#include <drm/drm_gem_cma_helper.h>

#include "uapi/drm/alif_drm.h"

#define D2D_DEVICE_COMPATIBLE	"tes,d2d-1.0"
#define D2D_DRIVER_NAME		"alif-d2d"

#define D2D_REG_READ(d2d, offset) \
	readl(d2d->regs + (offset << 2))
#define D2D_REG_WRITE(d2d, offset, value) \
	writel(value, d2d->regs +  (offset << 2))

struct alif_dev;

/*
 * Structure to hold the D/AVE-2D data.
 */
struct d2d_device {
	struct alif_dev *alif;
	struct platform_device *pdev;
	/* HW revision Information of the D2D device. */
	u32 hwrevision;
	/* Cached value of clock frequency. */
	u32 clk_freq;
	/* Virtual address of the D2D device after mapping. */
	void __iomem *regs;
	/* Physical address of D2D device on bus. */
	resource_size_t phy_base;
	/* Size of MMIO region for D2D device. */
	u32 size;
	/* IRQ number of the device. */
	int irq;
};

struct alif_dev {
	struct drm_device *drm;

	/* D2D HW config structure. */
	struct d2d_device *d2d;
};

static inline struct alif_dev *to_alif_dev(struct drm_device *drm)
{
	return (struct alif_dev *) drm->dev_private;
}

static inline struct device *to_device(struct alif_dev *alif)
{
	struct platform_device *pdev = alif->d2d->pdev;

	return &(pdev->dev);
}

int drm_init(struct device *dev);
void drm_free(struct alif_dev *alif);

/*
 * IOCTLs for Buffer Object creation and manipulation.
 */
int drm_d2d_gem_create_bo_ioctl(struct drm_device *dev, void *data,
	struct drm_file *file_priv);
int drm_d2d_gem_mmap_bo_ioctl(struct drm_device *dev, void *data,
	struct drm_file *file_priv);

/*
 * IOCTLs for register read/write and generic parameters retreival from
 * driver (clk, perf, ...).
 */
int drm_d2d_read_reg_ioctl(struct drm_device *dev, void *data,
	struct drm_file *file_priv);
int drm_d2d_write_reg_ioctl(struct drm_device *dev, void *data,
	struct drm_file *file_priv);
int drm_d2d_get_params_ioctl(struct drm_device *dev, void *data,
	struct drm_file *file_priv);

int d2d_read_reg_validate(struct d2d_device *d2d, struct drm_d2d_reg *args);
int d2d_write_reg_validate(struct d2d_device *d2d, struct drm_d2d_reg *args);
u32 d2d_get_clk_rate(struct d2d_device *d2d);

#endif /* _ALIF_DRM_DRIVER_H_ */
