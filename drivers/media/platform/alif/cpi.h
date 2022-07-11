/* SPDX-License-Identifier: GPL-2.0 */
/**
 * Copyright (C) 2018-2019 Synopsys, Inc.
 *
 * Synopsys DesignWare MIPI CSI-2 Host video functions
 *
 * Author: Luis Oliveira <Luis.Oliveira@synopsys.com>
 */

#ifndef __CPI_H__
#define __CPI_H__

#if 0
#include <linux/dmaengine.h>
#include <linux/dma/xilinx_dma.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-dma-contig.h>
#include <media/dwc/dw-mipi-csi-pltfrm.h>
#include <uapi/linux/media-bus-format.h>

#define DW_VDMA_N_BUFFERS	2
#define DW_VIDEO_DEF_FORMAT	V4L2_PIX_FMT_BGR24

struct cpi_cap_pdata {
	char *module_name;
	int test;
};

/** Buffer for video frames */
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

struct cpi_core_dev;

struct cpi_video_format {
	const char *name;
	const char *description;
	u32 fourcc;
	u32 mbus_code;
	u8 depth;
};

struct cpi_video {
	struct video_device video;
	struct v4l2_pix_format format;
	struct v4l2_subdev subdev;
	const struct cpi_video_format *fmtinfo;
	struct media_pad vd_pad;
	struct media_pad subdev_pads[VIDEO_DEV_SD_PADS_NUM];
	struct vb2_queue queue;
	unsigned int sequence;
	unsigned int field;
	struct list_head queued_bufs;
	struct mutex lock;
	spinlock_t queued_lock;
	struct dma_chan *dma;
	unsigned int align;
	struct dma_interleaved_template xt;
	struct data_chunk sgl[1];
};

struct cpi_dma_buffer {
	struct vb2_v4l2_buffer buf;
	struct list_head queue;
	struct cpi_video *video;
};

#define to_cpi_dma_buffer(vb) container_of(vb, struct cpi_dma_buffer, buf)
#define video_to_cpi(cpi_v) container_of(cpi_v, struct cpi_core_dev, video)

struct cpi_video_format *cpi_video_get_format_by_fourcc(u32 fourcc);
struct cpi_video_format *cpi_video_get_format_by_code(u32 mbus_code);
int cpi_video_init(struct cpi_core_dev *cpi_dev);
void cpi_video_exit(struct cpi_core_dev *cpi_dev);
#endif
#endif				/* __DW_VIDEO_H__ */
