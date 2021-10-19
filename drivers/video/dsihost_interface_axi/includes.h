/* SPDX-License-Identifier: GPL-2.0 */
/**
 * Copyright (C) 2018 Synopsys, Inc.
 *
 * @file includes.h
 * @brief includes file
 * included as a part of Synopsys MIPI DSI Host controller driver
 *
 * @author Luis Oliveira <luis.oliveira@synopsys.com>
 */

#ifndef __INCLUDES_H__
#define __INCLUDES_H__

#include <linux/module.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/kthread.h>
#include <linux/freezer.h>
#include <linux/semaphore.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/of_irq.h>
#include <linux/list.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/proc_fs.h>
#include <linux/fb.h>
#include <linux/mm.h>
#include <linux/uaccess.h>
#include <linux/i2c.h>
#include <linux/reset.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <asm/io.h>

#include "dsih_core.h"

#define TRUE 1
#define FALSE 0

/*
 * this macro only works in C99 so you can disable this, by placing
 * #define FUNC_NAME "MIPI_DSI", or __FUNCTION__
 */
#define FUNC_NAME __func__

/*
 * Frame buffer structure
 */
struct frame_buffer {
    /*
     * defines the current state of a particular video card. Inside
     * fb_info, there exists a fb_ops structure which is a collection of
     * needed functions to make fbdev and fbcon work. fb_info is only
     * visible to the kernel.
     */
    struct fb_info info;

    /* The operation file */
    struct fb_ops ops;
};

/**
 * Main structures to instantiate the driver
 */
struct mipi_dsi_dev {

    /** HW version */
    uint32_t hw_version;
    int position;

    /* timeout for FIFO full */
    int timeout;

    /* device node */
    struct device *parent_dev;

    /* device list */
    struct list_head devlist;

    /* interrupts */
    int irq_core;
    int irq_vif;

    /* spinlock */
    spinlock_t slock;

    /* mutex */
    struct reset_control	*rst_1;
    struct reset_control	*rst_2;
    struct reset_control	*rst_3;

    /* device Tree Information */
    char *device_name;

    /* MIPI DSI Controller */
    void __iomem *core_addr;
    uint32_t core_mem_size;

    /* videobridge */
    void __iomem *vid_bridge_addr;
    uint32_t vid_bridge_mem_size;

    /* GEN 3 IF */
    void __iomem *gen3_if_addr;
    uint32_t gen3_if_mem_size;

    void __iomem *mmcm_addr;
    uint32_t mmcm_mem_size;

    /* framebuffer */
    struct frame_buffer fb;

    /* for sending signals to user space */
    pid_t app_pid;

    dphy_t phy;

    /*
     * number of lanes physically connected to controller
     */
    uint8_t max_lanes;

    /*
     * maximum number of byte clock cycles needed by the PHY to perform
     * the Bus Turn Around operation - REQUIRED
     */
    uint16_t max_bta_cycles;

    /*
     * describe the color mode pin (dpicolorm) whether it is active
     * high or low - REQUIRED
     */
    int color_mode_polarity;

    /*
     * describe the shut down pin (dpishutdn) whether it is
     * active high or low - REQUIRED
     */
    int shut_down_polarity;

    dsih_dpi_video_t dpi_video;
    dsih_dpi_video_t dpi_video_old;
    dsih_cmd_mode_video_t cmd_mode_video;
    dsih_cmd_mode_video_t cmd_mode_video_old;
};

/*
 * dynamic memory allocation
 * Instance 0 will have the total size allocated so far and also the
 * number of calls to this function (number of allocated instances)
 */
struct mem_alloc {
    int instance;
    const char *info;
    size_t size;
    /* the pointer allocated by this instance */
    void *pointer;
    struct mem_alloc *last;
    struct mem_alloc *prev;
};

/**
 * @short Allocate memory for the driver
 * @param[in] into allocation name
 * @param[in] size allocation size
 * @param[in,out] allocated return structure for the allocation,
 * may be NULL
 * @return if successful, the pointer to the new created memory
 * if not, NULL
 */
void *alloc_mem(char *info, size_t size, struct mem_alloc *allocated);

void dsi_platform_init(struct mipi_dsi_dev *dev,
			int display, int video_mode, int lanes);

#endif /* __INCLUDES_H__ */
