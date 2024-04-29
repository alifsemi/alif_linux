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

enum data_type {
	CSI_2_YUV420_8 = 0x18,
	CSI_2_YUV420_10 = 0x19,
	CSI_2_YUV420_8_LEG = 0x1A,
	CSI_2_YUV420_8_SHIFT = 0x1C,
	CSI_2_YUV420_10_SHIFT = 0x1D,
	CSI_2_YUV422_8 = 0x1E,
	CSI_2_YUV422_10 = 0x1F,
	CSI_2_RGB444 = 0x20,
	CSI_2_RGB555 = 0x21,
	CSI_2_RGB565 = 0x22,
	CSI_2_RGB666 = 0x23,
	CSI_2_RGB888 = 0x24,
	CSI_2_RAW6 = 0x28,
	CSI_2_RAW7 = 0x29,
	CSI_2_RAW8 = 0x2A,
	CSI_2_RAW10 = 0x2B,
	CSI_2_RAW12 = 0x2C,
	CSI_2_RAW14 = 0x2D,
	CSI_2_RAW16 = 0x2E,
	CSI_2_RAW20 = 0x2F,
	USER_DEFINED_1 = 0x30,
	USER_DEFINED_2 = 0x31,
	USER_DEFINED_3 = 0x32,
	USER_DEFINED_4 = 0x33,
	USER_DEFINED_5 = 0x34,
	USER_DEFINED_6 = 0x35,
	USER_DEFINED_7 = 0x36,
	USER_DEFINED_8 = 0x37,
};

enum cpi_data_mask {
	CPI_DATA_MASK_16_BIT = 0,
	CPI_DATA_MASK_10_BIT,
	CPI_DATA_MASK_12_BIT,
	CPI_DATA_MASK_14_BIT,
};

enum cpi_data_mode {
	CPI_DATA_MODE_1_BIT = 0,
	CPI_DATA_MODE_2_BIT,
	CPI_DATA_MODE_4_BIT,
	CPI_DATA_MODE_8_BIT,
	CPI_DATA_MODE_16_BIT,
	CPI_DATA_MODE_32_BIT,
	CPI_DATA_MODE_64_BIT,
};

enum cpi_csi_color_mode {
	CPI_COLOR_MODE_CONFIG_IPI16_RAW6 = 0,
	CPI_COLOR_MODE_CONFIG_IPI16_RAW7,
	CPI_COLOR_MODE_CONFIG_IPI16_RAW8,
	CPI_COLOR_MODE_CONFIG_IPI16_RAW10,
	CPI_COLOR_MODE_CONFIG_IPI16_RAW12,
	CPI_COLOR_MODE_CONFIG_IPI16_RAW14,
	CPI_COLOR_MODE_CONFIG_IPI16_RAW16,
	CPI_COLOR_MODE_CONFIG_IPI48_RGB444,
	CPI_COLOR_MODE_CONFIG_IPI48_RGB555,
	CPI_COLOR_MODE_CONFIG_IPI48_RGB565,
	CPI_COLOR_MODE_CONFIG_IPI48_RGB666,
	CPI_COLOR_MODE_CONFIG_IPI48_XRGB888,
	CPI_COLOR_MODE_CONFIG_IPI48_RGBX888,
	CPI_COLOR_MODE_CONFIG_IPI48_RAW32,
	CPI_COLOR_MODE_CONFIG_IPI48_RAW48,
};

struct src_pixfmt_map {
	u32 fourcc;
	enum data_type dt;
	enum cpi_csi_color_mode col_mode;
};

static const struct src_pixfmt_map mappings[] = {
	{ V4L2_PIX_FMT_Y6, CSI_2_RAW6, CPI_COLOR_MODE_CONFIG_IPI16_RAW6 },
	{ V4L2_PIX_FMT_GREY, CSI_2_RAW8, CPI_COLOR_MODE_CONFIG_IPI16_RAW8 },
	/*
	 * HACK - The Following settings set the RAW10 format to captured from
	 * CSI IPI interface in just 8-bits, while the IPI interface is still
	 * 10 bits. This is done to save memory.
	 */
	{ V4L2_PIX_FMT_Y10P, CSI_2_RAW10, CPI_COLOR_MODE_CONFIG_IPI16_RAW8 },
	/*
	 * {V4L2_PIX_FMT_Y10P, CSI_2_RAW10, CPI_COLOR_MODE_CONFIG_IPI16_RAW10},
	 */
	{ V4L2_PIX_FMT_Y16, CSI_2_RAW10, CPI_COLOR_MODE_CONFIG_IPI16_RAW16 },
};

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
