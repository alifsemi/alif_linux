// SPDX-License-Identifier: GPL-2.0
/*
 * Alif Camera Controller CPI device driver
 *
 * Copyright (C) 2022 Alif Semiconductor.
 * Author: Harith George <harith.g@alifsemi.com>
 *
 * Based on sample code from Synopsys
 */

#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/videodev2.h>
#include <media/media-entity.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-dma-contig.h>
#include <media/videobuf2-dma-sg.h>
#include <media/videobuf2-vmalloc.h>

#include "plat.h"
#include "cpi.h"

/* Registers offset for CPI */
#define CPI_CTRL		0x00 /* Control register */
#define CPI_INTR		0x04 /* Interrupt Status register */
#define CPI_INTR_ENA		0x08 /* Interrupt Enable register */
#define CPI_CFG			0x10 /* Configuration register */
#define CPI_FIFO_CTRL		0x14 /* FIFO Control register */
#define CPI_AXI_ERR_STAT	0x18 /* AXI Error Status register */
#define CPI_VIDEO_FCFG		0x28 /* Video Frame Config register */
#define CPI_CSI_CMCFG		0x2C /* MIPI CSI Colour Mode Config */
#define CPI_FRAME_ADDR		0x30 /* Video Frame Start Address */

#define CTRL_FIFO_CLK_SEL	BIT(12)
#define CTRL_SW_RESET		BIT(8)
#define CTRL_SNAPSHOT		BIT(4)
#define CTRL_BUSY		BIT(2)
#define CTRL_START		BIT(0)

#define INTR_HSYNC		BIT(20)
#define INTR_VSYNC		BIT(16)
#define INTR_BRESP_ERR		BIT(6)
#define INTR_OUTFIFO_OVERRUN	BIT(5)
#define INTR_INFIFO_OVERRUN	BIT(4)
#define INTR_STOP		BIT(0)

#define CFG_MIPI_CSI			BIT(0)
#define CFG_CSI_HALT_EN			BIT(1)
#define CFG_WAIT_VSYNC			BIT(4)
#define CFG_VSYNC_EN			BIT(5)
#define CFG_ROW_ROUNDUP			BIT(8)
#define CFG_PCLK_POL			BIT(12)
#define CFG_HSYNC_POL			BIT(13)
#define CFG_VSYNC_POL			BIT(14)
#define CFG_MSB				BIT(20)
#define CFG_CODE10ON8			BIT(24)
#define CFG_DATA_MASK			GENMASK(1, 0)
#define CFG_DATA_MASK_SHIFT		28
#define CFG_DATA_MODE_MASK		GENMASK(2, 0)
#define CFG_DATA_MODE_MASK_SHIFT	16

#define FIFO_RD_WMARK_MASK		GENMASK(4, 0)
#define FIFO_RD_WMARK_SHIFT		0
#define FIFO_WR_WMARK_MASK		GENMASK(4, 0)
#define FIFO_WR_WMARK_SHIFT		8

#define CPI_BUSY_POLL_USEC	10
#define CPI_BUSY_TIMEOUT_USEC	20000
static const struct plat_csi_fmt cpi_formats[] = {
	{
		.name = "BGR888",
		.fourcc = V4L2_PIX_FMT_BGR24,
		.depth = 24,
		.mbus_code = MEDIA_BUS_FMT_RGB888_2X12_LE,
	 }, {
		.name = "RGB565",
		.fourcc = V4L2_PIX_FMT_RGB565,
		.depth = 16,
		.mbus_code = MEDIA_BUS_FMT_RGB565_2X8_BE,
	 }, {
		.name = "GREY",
		.fourcc = V4L2_PIX_FMT_GREY,
		.depth = 8,
		.mbus_code = MEDIA_BUS_FMT_SBGGR8_1X8,
	}, {
		.name = "Y10P",
		.fourcc = V4L2_PIX_FMT_Y10P,
		/*
		 * Hack, Depth should be 10 for Y10P. The depth = 8 ensures
		 * that only 8 Most Significant bits out of 10 bits coming at
		 * IPI interface are taken by the Camera controller and dumped
		 * to the memory. This saves some memory and allows image to
		 * be post-processed with bayer2RGB tool
		 * (supports 8-bit and 16-bit formats only).
		 */
		//.depth = 10,
		.depth = 8,
		.mbus_code = MEDIA_BUS_FMT_SBGGR10_1X10,
	}, {
		.name = "Y12",
		.fourcc = V4L2_PIX_FMT_Y12,
		.depth = 12,
		.mbus_code = MEDIA_BUS_FMT_SBGGR12_1X12,
	}
};

static const struct plat_csi_fmt *cpi_find_format(struct v4l2_format *f)
{
	const struct plat_csi_fmt *fmt = NULL;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(cpi_formats); ++i) {
		fmt = &cpi_formats[i];
		if (fmt->fourcc == f->fmt.pix.pixelformat)
			return fmt;
	}
	return NULL;
}

static const struct plat_csi_fmt *cpi_find_mbus_code(u32 mbus_code)
{
	const struct plat_csi_fmt *fmt = NULL;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(cpi_formats); ++i) {
		fmt = &cpi_formats[i];
		if (fmt->mbus_code == mbus_code)
			return fmt;
	}
	return NULL;
}

/*
 * Video node ioctl operations
 */
static int
cpi_querycap(struct file *file, void *priv, struct v4l2_capability *cap)
{
	struct cpi_dev *cpi = video_drvdata(file);

	strlcpy(cap->driver, "cpi-video-device", sizeof(cap->driver));
	strlcpy(cap->card, "cpi-video-device", sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info), "platform:%s",
		 dev_name(&cpi->pdev->dev));
	return 0;
}

static int
cpi_enum_fmt_vid_cap(struct file *file, void *priv, struct v4l2_fmtdesc *f)
{
	const struct plat_csi_fmt *p_fmt;

	if (f->index >= ARRAY_SIZE(cpi_formats))
		return -EINVAL;

	p_fmt = &cpi_formats[f->index];

	f->pixelformat = p_fmt->fourcc;

	return 0;
}

static int cpi_g_fmt_vid_cap(struct file *file, void *priv,
					struct v4l2_format *f)
{
	struct cpi_dev *dev = video_drvdata(file);

	f->fmt.pix = dev->format.fmt.pix;

	return 0;
}

static int
cpi_try_fmt_vid_cap(struct file *file, void *priv, struct v4l2_format *f)
{
	const struct plat_csi_fmt *fmt;

	fmt = cpi_find_format(f);
	if (!fmt) {
		f->fmt.pix.pixelformat = V4L2_PIX_FMT_Y10P;
		fmt = cpi_find_format(f);
	}

	f->fmt.pix.field = V4L2_FIELD_NONE;
	v4l_bound_align_image(&f->fmt.pix.width, 48, MAX_WIDTH, 2,
			      &f->fmt.pix.height, 32, MAX_HEIGHT, 0, 0);

	f->fmt.pix.bytesperline = (f->fmt.pix.width * fmt->depth) >> 3;
	f->fmt.pix.sizeimage = f->fmt.pix.height * f->fmt.pix.bytesperline;
	f->fmt.pix.colorspace = V4L2_COLORSPACE_SRGB;
	return 0;
}

static int cpi_s_fmt_vid_cap(struct file *file, void *priv,
					struct v4l2_format *set_format)
{
	struct cpi_dev *dev = video_drvdata(file);
	struct v4l2_subdev_format fmt = { 0 };
	const struct plat_csi_fmt *plat_fmt;
	int ret;

	if (vb2_is_busy(&dev->vb_queue))
		return -EBUSY;

	ret = cpi_try_fmt_vid_cap(file, dev, set_format);
	if (ret)
		return ret;

	plat_fmt = cpi_find_format(set_format);

	fmt.format.colorspace = set_format->fmt.pix.colorspace;
	fmt.format.code = plat_fmt->mbus_code;

	fmt.format.width = set_format->fmt.pix.width;
	fmt.format.height = set_format->fmt.pix.height;

	ret = plat_csi_pipeline_call(&dev->ve, set_format, &fmt);

	return 0;
}

static int cpi_enum_framesizes(struct file *file, void *fh,
					struct v4l2_frmsizeenum *fsize)
{
	static const struct v4l2_frmsize_stepwise sizes = {
		48, MAX_WIDTH, 4,
		32, MAX_HEIGHT, 1
	};
	int i;

	if (fsize->index)
		return -EINVAL;
	for (i = 0; i < ARRAY_SIZE(cpi_formats); i++)
		if (cpi_formats[i].fourcc == fsize->pixel_format)
			break;
	if (i == ARRAY_SIZE(cpi_formats))
		return -EINVAL;
	fsize->type = V4L2_FRMSIZE_TYPE_STEPWISE;
	fsize->stepwise = sizes;
	return 0;
}

static int cpi_enum_input(struct file *file, void *priv,
			struct v4l2_input *input)
{
	if (input->index != 0)
		return -EINVAL;

	input->type = V4L2_INPUT_TYPE_CAMERA;
	input->std = 0;
	strcpy(input->name, "Camera");

	return 0;
}

static int cpi_g_input(struct file *file, void *priv, unsigned int *i)
{
	*i = 0;
	return 0;
}

static int cpi_s_input(struct file *file, void *priv, unsigned int i)
{
	if (i != 0)
		return -EINVAL;
	return 0;
}

static int
cpi_vidioc_streamon(struct file *file, void *priv, enum v4l2_buf_type type)
{
	struct cpi_dev *cpi = video_drvdata(file);
	struct media_entity *entity = &cpi->ve.vdev.entity;
	int ret;

	ret = plat_csi_pipeline_call(&cpi->ve, set_stream, 1);
	if (ret < 0)
		return ret;

	ret = media_pipeline_start(entity, &cpi->ve.pipe->mp);
	if (ret < 0)
		return ret;

	vb2_ioctl_streamon(file, priv, type);
	if (!ret)
		return ret;

	return 0;
}

static int
cpi_streamoff(struct file *file, void *priv, enum v4l2_buf_type type)
{
	struct cpi_dev *cpi = video_drvdata(file);
	int ret;

	ret = vb2_ioctl_streamoff(file, priv, type);
	if (ret < 0)
		return ret;

	ret = plat_csi_pipeline_call(&cpi->ve, set_stream, 0);

	media_pipeline_stop(&cpi->ve.vdev.entity);
	return 0;
}

static const struct v4l2_ioctl_ops cpi_ioctl_ops = {
	.vidioc_querycap = cpi_querycap,

	.vidioc_enum_fmt_vid_cap = cpi_enum_fmt_vid_cap,
	.vidioc_g_fmt_vid_cap = cpi_g_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap = cpi_s_fmt_vid_cap,
	.vidioc_enum_framesizes = cpi_enum_framesizes,

	.vidioc_enum_input = cpi_enum_input,
	.vidioc_g_input = cpi_g_input,
	.vidioc_s_input = cpi_s_input,

	.vidioc_reqbufs = vb2_ioctl_reqbufs,
	.vidioc_create_bufs = vb2_ioctl_create_bufs,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_expbuf = vb2_ioctl_expbuf,
	.vidioc_prepare_buf = vb2_ioctl_prepare_buf,
	.vidioc_streamon = cpi_vidioc_streamon,
	.vidioc_streamoff = cpi_streamoff,
};

static int cpi_open(struct file *file)
{
	struct cpi_dev *cpi = video_drvdata(file);
	struct media_entity *me = &cpi->ve.vdev.entity;
	int ret;

	mutex_lock(&cpi->lock);

	ret = v4l2_fh_open(file);
	if (ret < 0)
		goto unlock;

	if (!v4l2_fh_is_singular_file(file))
		goto unlock;

	mutex_lock(&me->graph_obj.mdev->graph_mutex);

	ret = plat_csi_pipeline_call(&cpi->ve, open, me, true);
	if (ret == 0)
		me->use_count++;

	mutex_unlock(&me->graph_obj.mdev->graph_mutex);

	if (!ret)
		goto unlock;

	v4l2_fh_release(file);
unlock:
	mutex_unlock(&cpi->lock);
	return ret;
}

static int
cpi_release(struct file *file)
{
	struct cpi_dev *cpi = video_drvdata(file);
	struct media_entity *entity = &cpi->ve.vdev.entity;

	mutex_lock(&cpi->lock);

	if (v4l2_fh_is_singular_file(file)) {
		plat_csi_pipeline_call(&cpi->ve, close);
		mutex_lock(&entity->graph_obj.mdev->graph_mutex);
		entity->use_count--;
		mutex_unlock(&entity->graph_obj.mdev->graph_mutex);
	}

	_vb2_fop_release(file, NULL);

	mutex_unlock(&cpi->lock);
	return 0;
}

static const struct v4l2_file_operations cpi_fops = {
	.owner = THIS_MODULE,
	.open = cpi_open,
	.release = cpi_release,
	.write = vb2_fop_write,
	.read = vb2_fop_read,
	.poll = vb2_fop_poll,
	.unlocked_ioctl = video_ioctl2,
	.mmap = vb2_fop_mmap,
};

static inline struct rx_buffer *to_rx_buffer(struct vb2_v4l2_buffer *vb2)
{
	return container_of(vb2, struct rx_buffer, vb);
}

static inline void cpi_hw_enable_interrupts(struct cpi_dev *cpi, u32 intr_mask)
{
	u32 val = readl(cpi->base_addr + CPI_INTR_ENA);

	val |= intr_mask;
	writel(val, cpi->base_addr + CPI_INTR_ENA);
}

static inline void cpi_hw_disable_interrupts(struct cpi_dev *cpi,
		uint32_t intr_mask)
{
	u32 val = readl(cpi->base_addr + CPI_INTR_ENA);

	val &= ~intr_mask;
	writel(val, cpi->base_addr + CPI_INTR_ENA);
}

static inline int cpi_hw_setup_buffer(struct cpi_dev *cpi)
{
	if (!cpi->active)
		return -EFAULT;

	writel(cpi->active->dma_addr, cpi->base_addr + CPI_FRAME_ADDR);
	return 0;
}

static void cpi_hw_start_video_capture(struct cpi_dev *cpi)
{
	/* Soft reset CPI */
	writel(0, cpi->base_addr + CPI_CTRL);
	writel(CTRL_SW_RESET, cpi->base_addr + CPI_CTRL);
	writel(0, cpi->base_addr + CPI_CTRL);

	writel(CTRL_START |
		CTRL_SNAPSHOT |
		CTRL_FIFO_CLK_SEL, cpi->base_addr + CPI_CTRL);
}

static int cpi_hw_set_geometry(struct cpi_dev *cpi)
{
	const struct plat_csi_fmt *csi_fmt = cpi->fmt;
	struct v4l2_format *v4l2_fmt = &cpi->format;
	u32 data_mode;
	u32 data_mask;
	u32 val;
	int i;

	/* Fifo ctrl Read watermark 0x8, Write watermark 0x18 */
	val = (0x18 << FIFO_WR_WMARK_SHIFT) | (0x8 << FIFO_RD_WMARK_SHIFT);
	writel(val, cpi->base_addr + CPI_FIFO_CTRL);

	/* Configure Frame */
	val =	(v4l2_fmt->fmt.pix.width & GENMASK(13, 0)) |
		(((v4l2_fmt->fmt.pix.height - 1) & GENMASK(11, 0)) << 16);
	writel(val, cpi->base_addr + CPI_VIDEO_FCFG);

	/* Color config 16bit/32bit */
	switch (csi_fmt->depth) {
	case 1:
		data_mode = CPI_DATA_MODE_1_BIT;
		data_mask = 0;
		break;
	case 2:
		data_mode = CPI_DATA_MODE_2_BIT;
		data_mask = 0;
		break;
	case 4:
		data_mode = CPI_DATA_MODE_4_BIT;
		data_mask = 0;
		break;
	case 8:
		data_mode = CPI_DATA_MODE_8_BIT;
		data_mask = 0;
		break;
	case 10:
		data_mode = CPI_DATA_MODE_16_BIT;
		data_mask = CPI_DATA_MASK_10_BIT;
		break;
	case 12:
		data_mode = CPI_DATA_MODE_16_BIT;
		data_mask = CPI_DATA_MASK_12_BIT;
		break;
	case 14:
		data_mode = CPI_DATA_MODE_16_BIT;
		data_mask = CPI_DATA_MASK_14_BIT;
		break;
	case 16:
	default:
		data_mode = CPI_DATA_MODE_16_BIT;
		data_mask = CPI_DATA_MASK_16_BIT;
		break;
	}

	/* MIPI CSI */
	val = CFG_MIPI_CSI | CFG_CSI_HALT_EN | CFG_WAIT_VSYNC | CFG_ROW_ROUNDUP;
	val |= ((data_mode & CFG_DATA_MODE_MASK) << CFG_DATA_MODE_MASK_SHIFT) |
		((data_mask & CFG_DATA_MASK) << CFG_DATA_MASK_SHIFT);
	writel(val, cpi->base_addr + CPI_CFG);

	/* MIPI CSI Color Mode Configuration */
	for (i = 0; i < ARRAY_SIZE(mappings); i++) {
		if (v4l2_fmt->fmt.pix.pixelformat == mappings[i].fourcc)
			break;
	}
	if (i == ARRAY_SIZE(mappings))
		return -ENOTSUPP;

	writel(mappings[i].col_mode, cpi->base_addr + CPI_CSI_CMCFG);

	return 0;
}

static int cpi_queue_setup(struct vb2_queue *vq, unsigned int *nbuffers,
			unsigned int *nplanes, unsigned int sizes[],
			struct device *alloc_devs[])
{
	struct cpi_dev *cpi = vb2_get_drv_priv(vq);
	unsigned long size;

	size = cpi->format.fmt.pix.sizeimage;
	if (size == 0)
		return -EINVAL;


	if (*nbuffers > N_BUFFERS)
		*nbuffers = N_BUFFERS;
	*nplanes = 1;
	sizes[0] = size;

	cpi->active = NULL;
	return 0;
}

static int cpi_buffer_init(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct rx_buffer *buf = container_of(vbuf, struct rx_buffer, vb);

	buf->dma_addr = 0;
	buf->cpu_addr = NULL;
	INIT_LIST_HEAD(&buf->list);

	return 0;
}

static int cpi_buffer_prepare(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct cpi_dev *cpi = vb2_get_drv_priv(vb->vb2_queue);

	vb2_set_plane_payload(vb, 0, cpi->format.fmt.pix.sizeimage);

	if (vb2_get_plane_payload(vb, 0) > vb2_plane_size(vb, 0))
		return -EINVAL;

	vbuf->field = cpi->format.fmt.pix.field;
	return 0;
}

static void cpi_buffer_queue(struct vb2_buffer *vb)
{
	struct cpi_dev *cpi = vb2_get_drv_priv(vb->vb2_queue);
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct rx_buffer *buf = to_rx_buffer(vbuf);
	unsigned long flags = 0;

	if (vb == NULL) {
		pr_warn("%s:vb2_buffer is null\n", __func__);
		return;
	}

	spin_lock_irqsave(&cpi->slock, flags);
	list_add_tail(&buf->list, &cpi->fb_list_head);
	buf->dma_addr = vb2_dma_contig_plane_dma_addr(vb, 0);
	buf->cpu_addr = vb2_plane_vaddr(vb, 0);

	if (!cpi->active) {
		cpi->active = buf;
		if (vb2_is_streaming(vb->vb2_queue)) {
			cpi_hw_setup_buffer(cpi);

			/* Start capture | Snapshot mode */
			cpi_hw_start_video_capture(cpi);
		}
	}
	spin_unlock_irqrestore(&cpi->slock, flags);

	dev_dbg(&cpi->pdev->dev,
		"Queued buffer: dma_addr - 0x%08x cpu_addr - 0x%08x\n",
		buf->dma_addr,
		(u32)buf->cpu_addr);
}

static int cpi_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct cpi_dev *cpi = vb2_get_drv_priv(vq);

	cpi->sequence = 0;
	spin_lock_irq(&cpi->slock);
	/* Set-up the Camera controller registers as per settings provided. */
	cpi_hw_set_geometry(cpi);

	/* Set-up buffers */
	if (!cpi->active) {
		dev_err(&cpi->pdev->dev, "No Buffers Available to capture!\n");
		return -EINVAL;
	}
	cpi_hw_setup_buffer(cpi);

	cpi_hw_enable_interrupts(cpi, INTR_VSYNC |
		     INTR_BRESP_ERR |
		     INTR_OUTFIFO_OVERRUN |
		     INTR_INFIFO_OVERRUN |
		     INTR_STOP);

	/* Start capture | Snapshot mode */
	cpi_hw_start_video_capture(cpi);
	spin_unlock_irq(&cpi->slock);

	dev_dbg(&cpi->pdev->dev, "cpi: Start capture | Video mode!! 0x%x\n",
			readl(cpi->base_addr + CPI_CTRL));

	return 0;
}

static void cpi_stop_streaming(struct vb2_queue *vq)
{
	struct cpi_dev *cpi = vb2_get_drv_priv(vq);
	struct rx_buffer *buf, *node;
	u32 val;
	int ret = 0;

	/* Release all active buffers. */
	spin_lock_irq(&cpi->slock);
	cpi->active = NULL;
	list_for_each_entry_safe(buf, node, &cpi->fb_list_head, list) {
		list_del_init(&buf->list);
		vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
	}

	/* Disable Interrupts. */
	cpi_hw_disable_interrupts(cpi, INTR_VSYNC |
				INTR_BRESP_ERR |
				INTR_OUTFIFO_OVERRUN |
				INTR_INFIFO_OVERRUN |
				INTR_STOP);
	/* Stop the Camera Controller. */
	writel(0, cpi->base_addr + CPI_CTRL);
	spin_unlock_irq(&cpi->slock);

	/*
	 * Wait for Camera controller to stop.
	 * The CPI_CTRL register will be checked for CTRL_BUSY bit every 10us,
	 * and maximum wait will be 20000 us (20 ms).
	 */
	ret = readl_poll_timeout(cpi->base_addr + CPI_CTRL,
			val, !(val & CTRL_BUSY), CPI_BUSY_POLL_USEC,
			CPI_BUSY_TIMEOUT_USEC);
	if (ret)
		dev_err(&cpi->pdev->dev,
			"Failed to stop the Camera controller.\n");
}

/*
 * VideoBuffer2 operations
 */
static const struct vb2_ops vb2_video_qops = {
	.queue_setup = cpi_queue_setup,
	.buf_init = cpi_buffer_init,
	.buf_prepare = cpi_buffer_prepare,
	.buf_queue = cpi_buffer_queue,
	.start_streaming = cpi_start_streaming,
	.stop_streaming = cpi_stop_streaming,
	.wait_prepare = vb2_ops_wait_prepare,
	.wait_finish = vb2_ops_wait_finish,
};

static int cpi_set_default_fmt(struct cpi_dev *cpi)
{
	struct v4l2_format f = {
		.type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
		.fmt.pix = {
			.width          = 560,
			.height         = 560,
			.field          = V4L2_FIELD_NONE,
			.pixelformat    = V4L2_PIX_FMT_Y10P,
		},
	};
	int ret;

	ret = cpi_try_fmt_vid_cap(NULL, NULL, &f);
	if (ret) return ret;

	cpi->format = f;
	cpi->fmt = cpi_find_format(&f);
	return 0;
}

static int cpi_subdev_registered(struct v4l2_subdev *sd)
{
	struct cpi_dev *cpi = v4l2_get_subdevdata(sd);
	struct vb2_queue *q = &cpi->vb_queue;
	struct video_device *vfd = &cpi->ve.vdev;
	int ret;

	memset(vfd, 0, sizeof(*vfd));
	strlcpy(vfd->name, "alif-cam", sizeof(vfd->name));
	vfd->fops = &cpi_fops;
	vfd->ioctl_ops = &cpi_ioctl_ops;
	vfd->v4l2_dev = sd->v4l2_dev;
	vfd->minor = -1;
	vfd->release = video_device_release_empty;
	vfd->queue = q;
	vfd->device_caps = V4L2_CAP_STREAMING | V4L2_CAP_VIDEO_CAPTURE;


	memset(q, 0, sizeof(*q));
	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	q->io_modes = VB2_MMAP | VB2_USERPTR | VB2_READ;
	//TODO q->io_modes = VB2_MMAP | VB2_DMABUF | VB2_READ;

	q->ops = &vb2_video_qops;
	q->mem_ops = &vb2_dma_contig_memops;

	q->buf_struct_size = sizeof(struct rx_buffer);
	q->drv_priv = cpi;
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	q->lock = &cpi->lock;
	q->dev = &cpi->pdev->dev;

	ret = vb2_queue_init(q);
	if (ret < 0)
		return ret;

	cpi->vd_pad.flags = MEDIA_PAD_FL_SINK;
	ret = media_entity_pads_init(&vfd->entity, 1, &cpi->vd_pad);
	if (ret < 0)
		return ret;

	video_set_drvdata(vfd, cpi);
	cpi->ve.pipe = v4l2_get_subdev_hostdata(sd);

	ret = video_register_device(vfd, VFL_TYPE_GRABBER, -1);
	if (ret < 0) {
		media_entity_cleanup(&vfd->entity);
		cpi->ve.pipe = NULL;
		return ret;
	}

	v4l2_info(sd->v4l2_dev, "Registered %s as /dev/%s\n",
		  vfd->name, video_device_node_name(vfd));

	/* Set the default fmt right after registering */
	cpi_set_default_fmt(cpi);
	return 0;
}

static void cpi_subdev_unregistered(struct v4l2_subdev *sd)
{
	struct cpi_dev *cpi = v4l2_get_subdevdata(sd);

	if (cpi == NULL)
		return;

	mutex_lock(&cpi->lock);

	if (video_is_registered(&cpi->ve.vdev)) {
		video_unregister_device(&cpi->ve.vdev);
		media_entity_cleanup(&cpi->ve.vdev.entity);
		cpi->ve.pipe = NULL;
	}

	mutex_unlock(&cpi->lock);
}

static const struct v4l2_subdev_internal_ops cpi_subdev_internal_ops = {
	.registered = cpi_subdev_registered,
	.unregistered = cpi_subdev_unregistered,
};

static int cpi_s_power(struct v4l2_subdev *sd, int on)
{
	/*
	 * Nothing to do here, All geometry setup is geing done in Set-stream.
	 */
	return 0;
}

static int cpi_s_stream(struct v4l2_subdev *sd, int enable)
{
	/* Not doing much here at the moment
	 * called before sensor s_stream */
	return 0;
}

static int cpi_set_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_format *fmt)
{
	struct cpi_dev *cpi = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *subdev_fmt;
	const struct plat_csi_fmt *plat_csi;
	int ret = 0;

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		subdev_fmt = v4l2_subdev_get_try_format(sd, cfg,
				fmt->pad);
	} else {
		plat_csi = cpi_find_mbus_code(fmt->format.code);
		if (plat_csi == NULL) {
			dev_err(&cpi->pdev->dev,
				"Could not find Media Bus format requested.");
			return -EINVAL;
		}
		cpi->fmt = plat_csi;
		cpi->format.fmt.pix.width = fmt->format.width;
		cpi->format.fmt.pix.height = fmt->format.height;
		cpi->format.fmt.pix.field = fmt->format.field;
		cpi->format.fmt.pix.pixelformat = plat_csi->fourcc;
		cpi->format.fmt.pix.bytesperline =
			(plat_csi->depth * fmt->format.width) >> 3;
		cpi->format.fmt.pix.sizeimage =
			cpi->format.fmt.pix.bytesperline * fmt->format.height;
		cpi->format.fmt.pix.colorspace = fmt->format.colorspace;
		cpi->format.fmt.pix.ycbcr_enc = fmt->format.ycbcr_enc;
		cpi->format.fmt.pix.quantization = fmt->format.quantization;
		cpi->format.fmt.pix.xfer_func = fmt->format.xfer_func;
	}

	return ret;
}

static const struct v4l2_subdev_pad_ops cpi_pad_ops = {
	.set_fmt = cpi_set_fmt,
};

static const struct v4l2_subdev_video_ops cpi_video_ops = {
	.s_stream = cpi_s_stream,
};

static struct v4l2_subdev_core_ops cpi_core_ops = {
	.s_power = cpi_s_power,
};

static struct v4l2_subdev_ops cpi_subdev_ops = {
	.core = &cpi_core_ops,
	.video = &cpi_video_ops,
	.pad = &cpi_pad_ops,
};

static int cpi_create_capture_subdev(struct cpi_dev *cpi)
{
	struct v4l2_subdev *sd = &cpi->subdev;
	int ret;

	v4l2_subdev_init(sd, &cpi_subdev_ops);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	snprintf(sd->name, sizeof(sd->name), "alif-cpi");

	cpi->subdev_pads[VIDEO_DEV_PAD_SINK_CPI].flags =
	    MEDIA_PAD_FL_SINK;
	cpi->subdev_pads[VIDEO_DEV_PAD_SOURCE].flags =
	    MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&sd->entity, VIDEO_DEV_PADS_NUM,
					cpi->subdev_pads);
	if (ret)
		return ret;

	sd->internal_ops = &cpi_subdev_internal_ops;
	sd->owner = THIS_MODULE;
	v4l2_set_subdevdata(sd, cpi);
	return 0;
}

static void cpi_unregister_subdev(struct cpi_dev *cpi)
{
	struct v4l2_subdev *sd = &cpi->subdev;

	v4l2_device_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_set_subdevdata(sd, NULL);
}

static inline void cpi_change_buffer(struct cpi_dev *cpi)
{
	if (cpi->active) {
		struct vb2_v4l2_buffer *vbuf = &cpi->active->vb;
		struct rx_buffer *buf = cpi->active;

		list_del_init(&buf->list);
		vbuf->vb2_buf.timestamp = ktime_get_ns();
		vbuf->sequence = cpi->sequence++;
		vbuf->field = cpi->format.fmt.pix.field;
		vb2_buffer_done(&vbuf->vb2_buf, VB2_BUF_STATE_DONE);
	}

	if (list_empty(&cpi->fb_list_head)) {
		cpi->active = NULL;
	} else {
		cpi->active = list_entry(cpi->fb_list_head.next,
				struct rx_buffer, list);
		cpi_hw_setup_buffer(cpi);
		cpi_hw_start_video_capture(cpi);
	}
}

static irqreturn_t cpi_isr(int irq, void *dev)
{
	struct cpi_dev *cpi = dev;
	struct platform_device *pdev = cpi->pdev;
	u32 capture_error_mask;
	u32 int_st;

	capture_error_mask = INTR_INFIFO_OVERRUN |
		INTR_OUTFIFO_OVERRUN |
		INTR_BRESP_ERR;
	int_st = readl(cpi->base_addr + CPI_INTR) &
		readl(cpi->base_addr + CPI_INTR_ENA);
	writel(int_st, cpi->base_addr + CPI_INTR);

	if (int_st & INTR_HSYNC)
		dev_dbg(&pdev->dev, "H-Sync Interrupt\n");

	if (int_st & INTR_VSYNC)
		dev_dbg(&pdev->dev, "V-Sync Interrupt\n");

	if (int_st & INTR_BRESP_ERR) {
		u32 axi_err_stat =
			readl(cpi->base_addr + CPI_AXI_ERR_STAT);
		dev_err(&pdev->dev,
			"AXI Bus response error, BRESP code - %d\n",
			axi_err_stat & 0x3);
	}

	if (int_st & capture_error_mask)
		dev_err(&pdev->dev, "Frame Capture Error. int_st - 0x%08x",
				int_st);

	if (int_st & INTR_STOP) {
		dev_dbg(&pdev->dev, "Capture complete!");
		cpi_change_buffer(cpi);
	}

	return IRQ_HANDLED;
}

int plat_csi_probe(struct platform_device *pdev, struct cpi_dev *cpi);
static int cpi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int ret = 0;
	struct cpi_dev *cpi;
	struct resource *res;

	{
	/* TODO HACK!! CPI can be probed only after the other pipeline
	 * modules are all probed in..
	 */
		static int kkk = 0;
		kkk++;
		if (kkk < 3)
			return -EPROBE_DEFER;
	}
	if (!dev->of_node)
		return -ENODEV;

	cpi = devm_kzalloc(dev, sizeof(*cpi), GFP_KERNEL);
	if (!cpi)
		return -ENOMEM;

	cpi->pdev = pdev;

	cpi->active = NULL;
	spin_lock_init(&cpi->slock);
	mutex_init(&cpi->lock);
	INIT_LIST_HEAD(&cpi->fb_list_head);

	/* Registers mapping */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENXIO;
	cpi->base_addr = devm_ioremap_resource(dev, res);
	if (IS_ERR(cpi->base_addr)) {
		dev_err(dev, "Base address not set.\n");
		return PTR_ERR(cpi->base_addr);
	}

	cpi->irq = platform_get_irq(pdev, 0);
	if(cpi->irq < 0) {
		dev_err(cpi->dev, "platform_get_irq() failed with %d\n",
			cpi->irq);
		return cpi->irq;
	}

	ret = devm_request_irq(&pdev->dev, cpi->irq, cpi_isr, 0,
		"CPI", cpi);
	if(ret) {
		dev_err(cpi->dev, "devm_request_irq() failed with %d\n",
			ret);
		return ret;
	}

	ret = cpi_create_capture_subdev(cpi);
	if (ret)
		goto end;

	dev_info(dev, "Alif CPI device registered\n");

	plat_csi_probe(pdev, cpi);
	platform_set_drvdata(pdev, cpi);

	return 0;
end:
	dev_err(dev, "Alif CPI not registered!!\n");
	return ret;
}

static int cpi_remove(struct platform_device *pdev)
{
	struct cpi_dev *dev = platform_get_drvdata(pdev);

	cpi_unregister_subdev(dev);
	dev_info(&pdev->dev, "Driver removed\n");

	return 0;
}

static const struct of_device_id cpi_of_match[] = {
	{.compatible = "alif,ensemble-cpi"},
	{}
};

MODULE_DEVICE_TABLE(of, cpi_of_match);

static struct platform_driver __refdata cpi_pdrv = {
	.remove = cpi_remove,
	.probe = cpi_probe,
	.driver = {
		   .name = "alif-cpi",
		   .owner = THIS_MODULE,
		   .of_match_table = cpi_of_match,
		   },
};

module_platform_driver(cpi_pdrv);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Harith George <harith.g@alifsemi.com>");
MODULE_DESCRIPTION("Driver for Camera Parallel Interface Controller");
