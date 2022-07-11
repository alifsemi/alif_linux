/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Alif CPI Host platform device driver
 *
 * Copyright (C) 2022 Alif Semiconductor. All rights reserved.
 * Author: Harith George <harith.g@alifsemi.com>
 *
 * Based on code from Synopsys, Inc.
 */

#ifndef __CPI_PLAT_H_
#define __CPI_PLAT_H_

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_graph.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/dwc/dw-mipi-csi-pltfrm.h>

#define CSI_OF_NODE_NAME	"csi2"

#define to_plat_csi_pipeline(_ep) container_of(_ep, struct plat_csi_pipeline, ep)

#define N_BUFFERS 1


enum plat_csi_subdev_index {
	IDX_SENSOR1,
	IDX_SENSOR2,
	IDX_CSI,
	IDX_CPI,
	IDX_VDEV,
	IDX_MAX,
};

/**
 * struct plat_csi_sensor_info - image data source subdev information
 * @pdata: sensor's atrributes passed as media device's platform data
 * @asd: asynchronous subdev registration data structure
 * @subdev: image sensor v4l2 subdev
 * @host: csi device the sensor is currently linked to
 *
 * This data structure applies to image sensor and the writeback subdevs.
 */
struct plat_csi_sensor_info {
	struct plat_csi_source_info pdata;
	struct v4l2_async_subdev 	asd;
	struct v4l2_subdev 		*subdev;
	struct mipi_csi_dev 		*host;
};

/**
 * This structure represents a chain of media entities, including a data
 * source entity (e.g. an image sensor subdevice), a data capture entity
 * - a video capture device node and any remaining entities.
 */
struct plat_csi_pipeline {
	struct plat_csi_media_pipeline 	ep;
	struct list_head list;
	struct media_entity *vdev_entity;
	struct v4l2_subdev *subdevs[IDX_MAX];
};

struct rx_buffer {
	struct vb2_v4l2_buffer vb;
	struct list_head list;
	dma_addr_t dma_addr;
	void *cpu_addr;
};

struct dmaqueue {
	struct list_head active;
	wait_queue_head_t wq;
};

struct cpi_dev{
	struct platform_device *pdev;
	int irq;
	struct mutex lock;
	spinlock_t slock;
	struct plat_csi_video_entity ve;
	struct v4l2_format format;
	struct v4l2_pix_format pix_format;
	const struct plat_csi_fmt *fmt;
//	unsigned long *alloc_ctx;

	/* Buffer and DMA */
	struct vb2_queue vb_queue;
	int idx;
	int last_idx;
	struct dmaqueue vidq;
	struct rx_buffer dma_buf[N_BUFFERS];
//	struct dma_chan *dma;
//	struct dma_interleaved_template xt;
//	struct data_chunk sgl[1];
	struct media_pad vd_pad;
	struct media_pad subdev_pads[VIDEO_DEV_PADS_NUM];
	struct plat_csi_sensor_info sensor[PLAT_MAX_SENSORS];
	struct v4l2_subdev subdev;
	struct v4l2_subdev *csi_sd;
	struct v4l2_subdev *cpi_sd;
	struct media_device media_dev;
	struct v4l2_device v4l2_dev;
	struct device *dev;
	struct v4l2_async_subdev *async_subdevs[PLAT_MAX_SENSORS];
	struct v4l2_async_notifier subdev_notifier;
	struct media_graph link_setup_graph;
	struct list_head pipelines;
	int num_sensors;
	void __iomem *base_addr;
	void *vframe;
};

static inline struct cpi_dev *
entity_to_plat_csi_mdev(struct media_entity *me)
{
	return me->graph_obj.mdev == NULL ? NULL :
		container_of(me->graph_obj.mdev, struct cpi_dev, media_dev);
}

static inline struct cpi_dev *
notifier_to_plat_csi(struct v4l2_async_notifier *n)
{
	return container_of(n, struct cpi_dev, subdev_notifier);
}

static inline void plat_csi_graph_unlock(struct plat_csi_video_entity *ve)
{
	mutex_unlock(&ve->vdev.entity.graph_obj.mdev->graph_mutex);
}

#endif	/* __CPI_PLAT_ */
