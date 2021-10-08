/*
 * cdc_drv.c  --  CDC Display Controller DRM driver
 *
 * Copyright (C) 2017 TES Electronic Solutions GmbH
 * Author: Christian Thaler <christian.thaler@tes-dst.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/io.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/pm_runtime.h>
#include <linux/clk.h>

#include <linux/seq_file.h>

#include <drm/drmP.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_fb_cma_helper.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_probe_helper.h>

#include "cdc_regs.h"

#include "cdc_drv.h"
#include "cdc_kms.h"
#include "cdc_crtc.h"
#include "cdc_deswizzle.h"
#include "cdc_hw.h"
#include "cdc_hw_helpers.h"

extern struct platform_driver dswz_driver;

static const struct platform_device_id cdc_id_table[] = {
	{ "cdc", 0 },
	{ },
};

MODULE_DEVICE_TABLE ( platform, cdc_id_table);

static const struct of_device_id cdc_of_table[] = {
	{ .compatible =	"tes,cdc-2.1", .data = NULL },
	{ },
};

MODULE_DEVICE_TABLE ( of, cdc_of_table);

static void cdc_layer_init (struct cdc_device *cdc)
{
	int i;

	cdc->planes = devm_kzalloc(cdc->dev,
		sizeof(*cdc->planes) * cdc->hw.layer_count, GFP_KERNEL);
	for (i = 0; i < cdc->hw.layer_count; ++i) {
		dev_dbg(cdc->dev, "Initializing layer %d\n", i);
		cdc_hw_layer_setEnabled(cdc, i, false);
		cdc->planes[i].hw_idx = i;
		cdc->planes[i].cdc = cdc;
		cdc->planes[i].used = false;
	}
}

static irqreturn_t cdc_irq (int irq, void *arg)
{
	struct cdc_device *cdc = (struct cdc_device *) arg;
	u32 status;

	status = cdc_read_reg(cdc, CDC_REG_GLOBAL_IRQ_STATUS);
	cdc_write_reg(cdc, CDC_REG_GLOBAL_IRQ_CLEAR, status);

	if (status & CDC_IRQ_LINE) {
		cdc_crtc_irq(&cdc->crtc);
		if(cdc->dswz) {
			dswz_retrigger(cdc->dswz);
		}
	}
	if (status & CDC_IRQ_BUS_ERROR) {
		dev_err_ratelimited(cdc->dev, "BUS error IRQ triggered\n");
	}
	if (status & CDC_IRQ_FIFO_UNDERRUN_WARN) {
		// disable underrun IRQ to prevent IRQ flooding
		cdc_irq_set(cdc, CDC_IRQ_FIFO_UNDERRUN_WARN, false);

		dev_err_ratelimited(cdc->dev, "FIFO underrun warn\n");
	}
	if (status & CDC_IRQ_SLAVE_TIMING_NO_SIGNAL) {
		dev_err_ratelimited(cdc->dev, "SLAVE no signal\n");
	}
	if (status & CDC_IRQ_SLAVE_TIMING_NO_SYNC) {
		dev_err_ratelimited(cdc->dev, "SLAVE no sync\n");
	}
	if (status & CDC_IRQ_FIFO_UNDERRUN) {
		// disable underrun IRQ to prevent IRQ flooding
		cdc_irq_set(cdc, CDC_IRQ_FIFO_UNDERRUN, false);

		dev_err_ratelimited(cdc->dev, "FIFO underrun\n");
	}
	if (status & CDC_IRQ_CRC_ERROR) {
		// disable underrun IRQ to prevent IRQ flooding
		cdc_irq_set(cdc, CDC_IRQ_CRC_ERROR, false);

		dev_err_ratelimited(cdc->dev, "CRC error\n");
	}

	return IRQ_HANDLED;
}

bool cdc_init_irq (struct cdc_device *cdc)
{
	struct platform_device *pdev = to_platform_device(cdc->dev);

	int irq;
	int ret;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(cdc->dev, "Could not get platform IRQ number\n");
		return false;
	}

	cdc_write_reg(cdc, CDC_REG_GLOBAL_IRQ_ENABLE, cdc->hw.irq_enabled);
	cdc_write_reg(cdc, CDC_REG_GLOBAL_IRQ_CLEAR, 0xff);

	ret = devm_request_irq(cdc->dev, irq, cdc_irq, 0, dev_name(cdc->dev),
		cdc);
	if (ret < 0) {
		dev_err(cdc->dev, "Failed to register IRQ\n");
		return false;
	}

	return true;
}

#if 0
static void cdc_lastclose (struct drm_device *dev)
{
	struct cdc_device *cdc = dev->dev_private;

	drm_fbdev_cma_restore_mode(cdc->fbdev);
}
#endif

static int cdc_enable_vblank (struct drm_device *dev, unsigned int pipe)
{
	struct cdc_device *cdc = dev->dev_private;
	cdc_crtc_set_vblank(cdc, true);

	return 0;
}

static void cdc_disable_vblank (struct drm_device *dev, unsigned int pipe)
{
	struct cdc_device *cdc = dev->dev_private;
	cdc_crtc_set_vblank(cdc, false);
}

/* TODO: remove when cdc is fixed
 * Enforcing 256 byte pitch
 * */
static int cdc_gem_cma_dumb_create (struct drm_file *file_priv,
	struct drm_device *drm, struct drm_mode_create_dumb *args)
{
	args->pitch = (args->pitch + 255) & ~255;
	args->size = args->pitch * args->height;

	return drm_gem_cma_dumb_create_internal(file_priv, drm, args);
}

#ifdef CONFIG_DEBUG_FS
static int cdc_regs_show(struct seq_file *m, void *arg)
{
	struct drm_info_node *node = (struct drm_info_node *) m->private;
	struct drm_device *dev = node->minor->dev;
	struct cdc_device *cdc = dev->dev_private;
	unsigned i;

	pm_runtime_get_sync(dev->dev);

	/* Show the first few registers */
	for(i=0; i<(CDC_LAYER_SPAN + CDC_LAYER_SPAN * cdc->hw.layer_count); i += 4)
	{
		u32 reg = cdc_read_reg(cdc, i);

		if(i == 0)
			seq_printf(m, "Global:\n");
		else if(i % CDC_LAYER_SPAN == 0)
			seq_printf(m, "Layer %d:\n", i / CDC_LAYER_SPAN);

		seq_printf(m, "%03x: %08x", i * 4, reg);
		reg = cdc_read_reg(cdc, i+1);
		seq_printf(m, " %08x", reg);
		reg = cdc_read_reg(cdc, i+2);
		seq_printf(m, " %08x", reg);
		reg = cdc_read_reg(cdc, i+3);
		seq_printf(m, " %08x\n", reg);
	}

	pm_runtime_put_sync(dev->dev);

	return 0;
}

static int cdc_dump_fb(struct seq_file *m, void *arg)
{
	struct drm_info_node *node = (struct drm_info_node *) m->private;
	struct drm_device *dev = node->minor->dev;
	struct drm_framebuffer *fb;
	int ret;
	int i = 0;

	ret = mutex_lock_interruptible(&dev->mode_config.mutex);
	if(ret)
		return ret;

	ret = mutex_lock_interruptible(&dev->struct_mutex);
	if(ret) {
		mutex_unlock(&dev->mode_config.mutex);
		return ret;
	}

	list_for_each_entry(fb, &dev->mode_config.fb_list, head)
	{
		struct drm_gem_cma_object *obj = drm_fb_cma_get_gem_obj(fb, 0);

		if(i==1)
		{
			seq_write(m, obj->vaddr, 800*600*4); /* FIXME: static size, bad hack! */
		}
		++i;
	}

	mutex_unlock(&dev->struct_mutex);
	mutex_unlock(&dev->mode_config.mutex);

	return 0;
}

static int cdc_mm_show(struct seq_file *m, void *arg)
{
	struct drm_info_node *node = (struct drm_info_node *) m->private;
	struct drm_device *dev = node->minor->dev;
	struct drm_printer p = drm_seq_file_printer(m);

	drm_mm_print(&dev->vma_offset_manager->vm_addr_space_mm, &p);

	return 0;
}

static struct drm_info_list cdc_debugfs_list[] = {
	{ "regs", cdc_regs_show, 0 },
	{ "mm", cdc_mm_show, 0 },
	{ "fb", drm_fb_cma_debugfs_show, 0 },
	{ "fbdump", cdc_dump_fb, 0 },
};

static int cdc_debugfs_init(struct drm_minor *minor)
{
	struct drm_device *dev = minor->dev;
	int ret;

	ret = drm_debugfs_create_files(cdc_debugfs_list,
		ARRAY_SIZE(cdc_debugfs_list),
		minor->debugfs_root, minor);

	if(ret) {
		dev_err(dev->dev, "could not install cdc_debugfs_list\n");
		return ret;
	}

	return ret;
}
#endif

#include "cdc_ioctl.h"

long cdc_ioctl (struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct drm_file *file_priv = filp->private_data;
	struct drm_device *dev;
	struct cdc_device *cdc;
	char stack_data[128];
	unsigned int nr = HACK_IOCTL_NR(cmd);
	unsigned int size;

	dev = file_priv->minor->dev;
	cdc = dev->dev_private;

	if (nr >= 0xe0) {
		size = _IOC_SIZE(cmd);

		if (cmd & IOC_IN) {
			if (copy_from_user(stack_data, (void __user *) arg, size) != 0) {
				return -EFAULT;
			}
		}

		/* TODO: Dispatching */
		switch (nr) {
		case 0xe0: {
			struct hack_set_cb *set_cb =
				(struct hack_set_cb*) stack_data;

			cdc_hw_layer_setCBSize(cdc, 0, set_cb->width,
				set_cb->height, set_cb->pitch);
			cdc_hw_setCBAddress(cdc, 0,
				(unsigned int) set_cb->phy_addr);
			if(cdc->dswz) {
				dswz_set_fb_addr(cdc->dswz, (u32) set_cb->phy_addr);
				dswz_set_mode(cdc->dswz, DSWZ_MODE_DESWIZZLE);
				dswz_retrigger(cdc->dswz);
			}
			cdc_hw_triggerShadowReload(cdc, true);
			break;
		}

		case 0xe1: {
			struct hack_set_winpos *winpos =
				(struct hack_set_winpos*) stack_data;

			cdc_hw_setWindow(cdc, 0, winpos->x, winpos->y,
				winpos->width, winpos->height,
				winpos->width * 4);
			cdc_hw_layer_setEnabled(cdc, 0, true);
			cdc_hw_triggerShadowReload(cdc, true);
			break;
		}

		case 0xe2: {
			struct hack_set_alpha *alpha =
				(struct hack_set_alpha*) stack_data;

			cdc_hw_setBlendMode(cdc, 0,
				CDC_BLEND_PIXEL_ALPHA_X_CONST_ALPHA,
				CDC_BLEND_PIXEL_ALPHA_X_CONST_ALPHA_INV);
			cdc_hw_layer_setConstantAlpha(cdc, 0, alpha->alpha);
			cdc_hw_triggerShadowReload(cdc, true);
			break;
		}

		case 0xe3: {
			unsigned long flags;

			drm_crtc_vblank_get(&cdc->crtc);
			spin_lock_irqsave(&cdc->irq_slck, flags);
			cdc->irq_stat = 0;
			spin_unlock_irqrestore(&cdc->irq_slck, flags);
			wait_event_interruptible(cdc->irq_waitq, cdc->irq_stat);
			drm_crtc_vblank_put(&cdc->crtc);

			break;
		}

		default:
			printk(KERN_ERR "Unknown IOCTL (nr = %u)!\n", nr);
		}

		return 0;
	}

	return drm_ioctl(filp, cmd, arg);
}

static const struct file_operations cdc_fops = {
	.owner = THIS_MODULE,
	.open = drm_open,
	.release = drm_release,
	.unlocked_ioctl = cdc_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = drm_compat_ioctl,
#endif
	.poll = drm_poll,
	.read = drm_read,
	.llseek = no_llseek,
	.mmap =	drm_gem_cma_mmap,
};

static struct drm_driver cdc_driver = {
	.driver_features = DRIVER_GEM | DRIVER_MODESET | DRIVER_ATOMIC,
	.lastclose = drm_fb_helper_lastclose,
	.enable_vblank = cdc_enable_vblank,
	.disable_vblank = cdc_disable_vblank,
	.gem_free_object = drm_gem_cma_free_object,
	.prime_handle_to_fd = drm_gem_prime_handle_to_fd,
	.prime_fd_to_handle = drm_gem_prime_fd_to_handle,
	.gem_prime_import = drm_gem_prime_import,
	.gem_prime_export = drm_gem_prime_export,
	.gem_prime_get_sg_table = drm_gem_cma_prime_get_sg_table,
	.gem_prime_import_sg_table = drm_gem_cma_prime_import_sg_table,
	.gem_prime_vmap = drm_gem_cma_prime_vmap,
	.gem_prime_vunmap = drm_gem_cma_prime_vunmap,
	.gem_prime_mmap = drm_gem_cma_prime_mmap,
	.gem_vm_ops = &drm_gem_cma_vm_ops,
	.dumb_create = cdc_gem_cma_dumb_create,
	.dumb_map_offset = drm_gem_dumb_map_offset,
	.dumb_destroy = drm_gem_dumb_destroy,
#ifdef CONFIG_DEBUG_FS
	.debugfs_init = cdc_debugfs_init,
#endif
	.fops = &cdc_fops,
	.name = "tes-cdc",
	.desc = "TES CDC Display Controller",
	.date = "20190902",
	.major = 1,
	.minor = 0,
};

static int cdc_pm_suspend (struct device *dev)
{
	struct cdc_device *cdc = dev_get_drvdata(dev);

	dev_dbg(dev, "%s\n", __func__);

	/* TODO: Suspend */
	drm_kms_helper_poll_disable(cdc->ddev);

	return 0;
}

static int cdc_pm_resume (struct device *dev)
{
	struct cdc_device *cdc = dev_get_drvdata(dev);

	dev_dbg(dev, "%s\n", __func__);

	/* TODO: Resume */
	drm_kms_helper_poll_enable(cdc->ddev);

	return 0;
}

static const struct dev_pm_ops cdc_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(cdc_pm_suspend, cdc_pm_resume)
};

static int cdc_remove (struct platform_device *pdev)
{
	struct cdc_device *cdc = platform_get_drvdata(pdev);
	struct drm_device *ddev = cdc->ddev;

	/* Turn off vblank processing and irq */
	drm_crtc_vblank_off(&cdc->crtc);

	/* Turn off CRTC */
	cdc_hw_setEnabled(cdc, false);

	drm_dev_unregister(ddev);

	drm_kms_helper_poll_fini(ddev);
	drm_mode_config_cleanup(ddev);

	cdc_write_reg(cdc, CDC_REG_GLOBAL_IRQ_ENABLE, 0x0);
	cdc_hw_setEnabled(cdc, false);

	drm_dev_put(ddev);

	return 0;
}

static int compare_name_dswz(struct device *dev, void *data)
{
	if(!dev->driver)
		return 0;

	return (strstr(dev->driver->name, "dswz") != NULL);
}

static int cdc_probe (struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct device *dswz_dev;
	struct cdc_device *cdc;
	struct drm_device *ddev;
	struct resource *mem;
	int max_clock;
	int ret = 0;

	if (np == NULL) {
		dev_err(&pdev->dev, "no platform data\n");
		return -ENODEV;
	}

	cdc = devm_kzalloc(&pdev->dev, sizeof(*cdc), GFP_KERNEL);
	if (cdc == NULL) {
		dev_err(&pdev->dev, "failed to allocate driver data\n");
		return -ENOMEM;
	}

	init_waitqueue_head(&cdc->commit.wait);

	/* FIXME HACK for MesseDemo   */
	spin_lock_init(&cdc->irq_slck);
	init_waitqueue_head(&cdc->irq_waitq);

	cdc->dev = &pdev->dev;

	platform_set_drvdata(pdev, cdc);

	cdc->pclk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(cdc->pclk)) {
		dev_err(&pdev->dev, "failed to initialize pixel clock\n");
		return PTR_ERR(cdc->pclk);
	}

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	cdc->mmio = devm_ioremap_resource(&pdev->dev, mem);
	if (IS_ERR(cdc->mmio)) {
		return PTR_ERR(cdc->mmio);
	}
	dev_dbg(&pdev->dev, "Mapped IO from 0x%x to 0x%p\n", mem->start,
		cdc->mmio);

	if(of_property_read_s32(pdev->dev.of_node, "max-clock-frequency", &max_clock)) {
		cdc->max_clock_khz = 0;
	}
	else {
		cdc->max_clock_khz = max_clock / 1000; // Hz to kHz
		dev_dbg(&pdev->dev, "Set max pixel clock frequency to %d\n", cdc->max_clock_khz);
	}

	np = of_parse_phandle(pdev->dev.of_node, "memory-region", 0);
	if (np) {
		dev_err(&pdev->dev, "Using reserved memory as CMA pool\n");
		ret = of_reserved_mem_device_init(&pdev->dev);
		if(ret) {
			dev_err(&pdev->dev, "Could not get reserved memory\n");
			return ret;
		}
		of_node_put(np);
	}
	else
		dev_err(&pdev->dev, "Using default CMA pool\n");

	/* DRM/KMS objects */
	ddev = drm_dev_alloc(&cdc_driver, &pdev->dev);
	if (IS_ERR(ddev))
		return PTR_ERR(ddev);

	cdc->ddev = ddev;
	ddev->dev_private = cdc;

	cdc_crtc_set_vblank(cdc, false);
	cdc->hw.enabled = false;
	cdc->hw.irq_enabled = 0;

	/* get the hw configuration */
	{
		cdc_hw_revision_t hwrev;
		cdc_config1_t conf1;
		cdc_config2_t conf2;
		u32 layer_count;

		hwrev.m_data = cdc_read_reg(cdc, CDC_REG_GLOBAL_HW_REVISION);
		layer_count = cdc_read_reg(cdc, CDC_REG_GLOBAL_LAYER_COUNT);
		conf1.m_data = cdc_read_reg(cdc, CDC_REG_GLOBAL_CONFIG1);
		conf2.m_data = cdc_read_reg(cdc, CDC_REG_GLOBAL_CONFIG2);

		cdc->hw.layer_count = layer_count;
		cdc->hw.shadow_regs = conf1.bits.m_shadow_regs;
		cdc->hw.bus_width = 1 << conf2.bits.m_bus_width;

		dev_info(&pdev->dev, "CDC HW ver. %u.%u (rev. %u):\n",
			 hwrev.bits.m_major, hwrev.bits.m_minor,
			 hwrev.bits.m_revision);
		dev_info(&pdev->dev, "\tlayer count: %u\n", cdc->hw.layer_count);
		dev_info(&pdev->dev, "\tbus width: %u byte\n", cdc->hw.bus_width);
	}

	/* Spawn stream sub devices if available */
	ret = devm_of_platform_populate(cdc->dev);

	dswz_dev = device_find_child(cdc->dev, NULL, compare_name_dswz);
	if(dswz_dev) {
		dev_info(&pdev->dev, "\tdeswizzler: yes\n");
		cdc->dswz = (struct dswz_device*) dev_get_drvdata(dswz_dev);
		/* todo: build an aggregate driver? */
	}
	else {
		dev_info(&pdev->dev, "\tdeswizzler: no\n");
	}

	cdc_layer_init(cdc);

	cdc_hw_resetRegisters(cdc);

	cdc_init_irq(cdc);

	ret = cdc_modeset_init(cdc);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to initialize CDC Modeset\n");
		goto error;
	}

	ddev->irq_enabled = 1;

	/* Register the DRM device with the core and the connectors with
	 * sysfs.
	 */
	ret = drm_dev_register(ddev, 0);
	if (ret)
		goto error;

	DRM_INFO("Device %s probed\n", dev_name(&pdev->dev));

	/* DSWZ driver needs retriggering in every frame. So we increase
	 * the use counter of the vblank.
	 */
	if(cdc->dswz)
		drm_crtc_vblank_get(&cdc->crtc);

	/* Setup framebuffer with 16 bits/pixel */
	drm_fbdev_generic_setup(ddev, 16);
	return 0;

error:
	cdc_remove(pdev);

	return 0;
}

static struct platform_driver cdc_platform_driver = {
	.probe = cdc_probe,
	.remove = cdc_remove,
	.driver = {
		.name = "tes-cdc",
		.pm = &cdc_pm_ops,
		.of_match_table = cdc_of_table,
	},
	.id_table = cdc_id_table,
};

static struct platform_driver * const drivers[] = {
        &dswz_driver,
	&cdc_platform_driver,
};

static int cdc_drv_init(void)
{
        return platform_register_drivers(drivers, ARRAY_SIZE(drivers));
}
module_init(cdc_drv_init);

static void cdc_drv_exit(void)
{
        platform_unregister_drivers(drivers, ARRAY_SIZE(drivers));
}
module_exit(cdc_drv_exit);

MODULE_AUTHOR("Christian Thaler <christian.thaler@tes-dst.com>");
MODULE_DESCRIPTION("TES CDC Display Controller DRM Driver");
MODULE_LICENSE("GPL");

