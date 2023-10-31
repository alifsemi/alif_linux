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

#include "alif_drm_driver.h"

int drm_d2d_gem_create_bo_ioctl(struct drm_device *dev, void *data,
	struct drm_file *file_priv)
{
	struct drm_d2d_create_bo *params = data;
	struct drm_gem_cma_object *cma;
	int ret = 0;

	cma = drm_gem_cma_create(dev, params->size);
	if (IS_ERR(cma))
		return PTR_ERR(cma);

	params->paddr = cma->paddr;
	ret = drm_gem_handle_create(file_priv, &cma->base, &params->handle);
	drm_gem_object_put_unlocked(&cma->base);
	return ret;
}

int drm_d2d_gem_mmap_bo_ioctl(struct drm_device *dev, void *data,
	struct drm_file *file_priv)
{
	struct drm_d2d_mmap_bo *params = data;
	struct drm_gem_object *gem_obj;

	gem_obj = drm_gem_object_lookup(file_priv, params->handle);
	if (!gem_obj)
		return -EINVAL;

	/* The mmap offset was set up at BO allocation time. */
	params->offset = drm_vma_node_offset_addr(&gem_obj->vma_node);
	drm_gem_object_put_unlocked(gem_obj);
	return 0;
}

int drm_d2d_read_reg_ioctl(struct drm_device *dev, void *data,
	struct drm_file *file_priv)
{
	struct alif_dev *alif = to_alif_dev(dev);
	struct drm_d2d_reg *args = data;

	return d2d_read_reg_validate(alif->d2d, args);
}

int drm_d2d_write_reg_ioctl(struct drm_device *dev, void *data,
	struct drm_file *file_priv)
{
	struct alif_dev *alif = to_alif_dev(dev);
	struct drm_d2d_reg *args = data;

	return d2d_write_reg_validate(alif->d2d, args);
}

int drm_d2d_get_params_ioctl(struct drm_device *dev, void *data,
	struct drm_file *file_priv)
{
	struct alif_dev *alif = to_alif_dev(dev);
	struct drm_d2d_get_params *params = data;
	u32 rc = 0;

	switch (params->param_id) {
	case DRM_D2D_GET_PARAMS_CLK:
		params->value = d2d_get_clk_rate(alif->d2d);
		break;
	default:
		pr_err("Invalid case!\n");
		rc = -EINVAL;
	}
	return rc;
}

static const struct drm_ioctl_desc drm_d2d_ioctls[] = {
	DRM_IOCTL_DEF_DRV(D2D_GEM_CREATE_BO, drm_d2d_gem_create_bo_ioctl,
		DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(D2D_GEM_MMAP_BO, drm_d2d_gem_mmap_bo_ioctl,
		DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(D2D_READ_REG, drm_d2d_read_reg_ioctl,
		DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(D2D_WRITE_REG, drm_d2d_write_reg_ioctl,
		DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(D2D_GET_PARAMS, drm_d2d_get_params_ioctl,
		DRM_RENDER_ALLOW),
};

DEFINE_DRM_GEM_CMA_FOPS(d2d_drm_fops);

static struct drm_driver alif_drm_driver = {
	.driver_features	= (DRIVER_GEM | DRIVER_RENDER),

	.ioctls			= drm_d2d_ioctls,
	.num_ioctls		= ARRAY_SIZE(drm_d2d_ioctls),

	DRM_GEM_CMA_VMAP_DRIVER_OPS,
	.fops			= &d2d_drm_fops,
	.name			= D2D_DRIVER_NAME,
	.desc			= "TES ALIF 2D-GPU",
	.date			= "20230413",
	.major			= 1,
	.minor			= 0,
};

int drm_init(struct device *dev)
{
	struct drm_device *drm;
	struct alif_dev *alif = NULL;
	struct d2d_device *d2d = dev_get_drvdata(dev);
	int rc = 0;

	alif = devm_kzalloc(dev, sizeof(*alif), GFP_KERNEL);
	if (!alif)
		return  -ENOMEM;

	drm = drm_dev_alloc(&alif_drm_driver, dev);
	if (IS_ERR(drm)) {
		dev_dbg(dev,
			"Failed to allocate the DRM Device structure. rc - %d",
			rc);
		return PTR_ERR(drm);
	}
	alif->drm = drm;
	drm->dev_private = alif;

	alif->d2d = d2d;
	d2d->alif = alif;

	alif->drm->irq_enabled = true;
	rc = drm_dev_register(alif->drm, 0);
	if (rc) {
		dev_dbg(dev, "DRM Device registration failed! rc - %d", rc);
		goto dev_put;
	}

	return 0;
dev_put:
	drm_dev_put(drm);
	return rc;
}

void drm_free(struct alif_dev *alif)
{
	struct drm_device *drm = alif->drm;
	struct device *dev = to_device(alif);

	drm_dev_unregister(drm);
	alif->drm->irq_enabled = false;

	drm->dev_private = NULL;
	alif->drm = NULL;

	devm_kfree(dev, alif);
	drm_dev_put(drm);
}
