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
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/videodev2.h>
#include <media/media-entity.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ioctl.h>
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
		.name = "Y10P",
		.fourcc = V4L2_PIX_FMT_Y10P,
		.depth = 8,
		.mbus_code = MEDIA_BUS_FMT_SBGGR10_1X10,
	},
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
		f->fmt.pix.pixelformat = V4L2_PIX_FMT_RGB565;
		fmt = cpi_find_format(f);
	}

	f->fmt.pix.field = V4L2_FIELD_NONE;
	v4l_bound_align_image(&f->fmt.pix.width, 48, MAX_WIDTH, 2,
			      &f->fmt.pix.height, 32, MAX_HEIGHT, 0, 0);

	f->fmt.pix.bytesperline = (f->fmt.pix.width * fmt->depth) >> 3;
	f->fmt.pix.sizeimage = f->fmt.pix.height * f->fmt.pix.bytesperline;
	f->fmt.pix.colorspace = V4L2_COLORSPACE_DEFAULT;
	return 0;
}

static int cpi_s_fmt_vid_cap(struct file *file, void *priv,
					struct v4l2_format *f)
{
	struct cpi_dev *dev = video_drvdata(file);
	int ret;
	struct v4l2_subdev_format fmt;
	struct v4l2_pix_format *dev_fmt_pix = &dev->format.fmt.pix;

	if (vb2_is_busy(&dev->vb_queue))
		return -EBUSY;

	ret = cpi_try_fmt_vid_cap(file, dev, f);
	if (ret)
		return ret;

	dev->fmt = cpi_find_format(f);
	dev_fmt_pix->pixelformat = f->fmt.pix.pixelformat;
	dev_fmt_pix->width = f->fmt.pix.width;
	dev_fmt_pix->height  = f->fmt.pix.height;
	dev_fmt_pix->bytesperline = dev_fmt_pix->width * (dev->fmt->depth / 8);
	dev_fmt_pix->sizeimage =
			dev_fmt_pix->height * dev_fmt_pix->bytesperline;

	fmt.format.colorspace = V4L2_COLORSPACE_SRGB;
	fmt.format.code = dev->fmt->mbus_code;

	fmt.format.width = dev_fmt_pix->width;
	fmt.format.height = dev_fmt_pix->height;

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
cpi_streamon(struct file *file, void *priv, enum v4l2_buf_type type)
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
	.vidioc_prepare_buf = vb2_ioctl_prepare_buf,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_streamon = cpi_streamon,
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

void fill_buffer(struct cpi_dev *dev, struct rx_buffer *buf,
			int buf_num)
{
	buf->vb.field = dev->format.fmt.pix.field;
	buf->vb.sequence++;
	buf->vb.vb2_buf.timestamp = ktime_get_ns();
	vb2_set_plane_payload(&buf->vb.vb2_buf, 0,
		dev->format.fmt.pix.bytesperline*dev->format.fmt.pix.height);
	vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_DONE);
}

#if 0
static void buffer_copy_process(void *param)
{
	struct cpi_dev *dev = (struct cpi_dev *) param;
	unsigned long flags;
	struct dmaqueue *dma_q = &dev->vidq;
	struct rx_buffer *buf = NULL;

	spin_lock_irqsave(&dev->slock, flags);

	if (!list_empty(&dma_q->active)) {
		buf = list_entry(dma_q->active.next, struct rx_buffer, list);
		list_del(&buf->list);
		//fill_buffer(dev, buf, dev->last_idx, flags);
		fill_buffer(dev, buf, dev->last_idx);
	}

	spin_unlock_irqrestore(&dev->slock, flags);
}
#endif

static inline struct rx_buffer *to_rx_buffer(struct vb2_v4l2_buffer *vb2)
{
	return container_of(vb2, struct rx_buffer, vb);
}

static int queue_setup(struct vb2_queue *vq, unsigned int *nbuffers,
			unsigned int *nplanes, unsigned int sizes[],
			struct device *alloc_devs[])
{
	struct cpi_dev *dev = vb2_get_drv_priv(vq);
	unsigned long size;

	size = dev->format.fmt.pix.sizeimage;
	if (size == 0)
		return -EINVAL;

	*nbuffers = N_BUFFERS;
	*nplanes = 1;
	sizes[0] = size;

	return 0;
}

static int buffer_prepare(struct vb2_buffer *vb)
{
#if 0
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct rx_buffer *buf = to_rx_buffer(vbuf);

	INIT_LIST_HEAD(&buf->list);
#endif
	return 0;
}

static void buffer_queue(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct cpi_dev *cpi = vb2_get_drv_priv(vb->vb2_queue);
	struct rx_buffer *buf = to_rx_buffer(vbuf);
	//struct dmaqueue *vidq = NULL;
	//struct dma_async_tx_descriptor *desc;
	//u32 flags;
	char *dest;

	if (vb == NULL) {
		pr_warn("buffer_queue:vb2_buffer is null\n");
		return;
	}
#if 0
	dev = vb2_get_drv_priv(vb->vb2_queue);
	buf = to_rx_buffer(vbuf);
	vidq = &dev->vidq;

	flags = DMA_PREP_INTERRUPT | DMA_CTRL_ACK;
	dev->xt.dir = DMA_DEV_TO_MEM;
	dev->xt.src_sgl = false;
	dev->xt.dst_inc = false;
	dev->xt.dst_sgl = true;
	dev->xt.dst_start = vb2_dma_contig_plane_dma_addr(vb, 0);
	dev->last_idx = dev->idx;
	dev->idx++;
	if (dev->idx >= N_BUFFERS)
		dev->idx = 0;

	dev->xt.frame_size = 1;
	dev->sgl[0].size = dev->format.fmt.pix.bytesperline;
	dev->sgl[0].icg = 0;
	dev->xt.numf = dev->format.fmt.pix.height;

	desc = dmaengine_prep_interleaved_dma(dev->dma, &dev->xt, flags);
	if (!desc) {
		pr_err("Failed to prepare DMA transfer\n");
		vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
		return;
	}

	desc->callback = buffer_copy_process;
	desc->callback_param = dev;

	spin_lock(&dev->slock);
	list_add_tail(&buf->list, &vidq->active);
	spin_unlock(&dev->slock);

	dmaengine_submit(desc);

	if (vb2_is_streaming(&dev->vb_queue))
		dma_async_issue_pending(dev->dma);
#else
	//u32 dest = vb2_dma_contig_plane_dma_addr(vb, 0);
	dest = vb2_plane_vaddr(&buf->vb.vb2_buf, 0);
	memcpy(dest, cpi->vframe,
		cpi->format.fmt.pix.bytesperline * cpi->format.fmt.pix.height);

	buf = to_rx_buffer(vbuf);
	//vidq = &dev->vidq;
	cpi->last_idx = cpi->idx;
	cpi->idx++;
	if (cpi->idx >= N_BUFFERS)
		cpi->idx = 0;

	fill_buffer(cpi, buf, cpi->last_idx);
#endif
}

static int start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct cpi_dev *cpi = vb2_get_drv_priv(vq);
	uint32_t val;

	/* Start capture | Snapshot mode */
	val = readl(cpi->base_addr + CPI_CTRL);
	val |= (1 << 4) | (1 << 0);
	//val |= (1 << 0);
	writel(val, cpi->base_addr + CPI_CTRL);
	printk("#cpi: start_streaming enter => Start capture | Video mode!! 0x%x\n", val);

	//dma_async_issue_pending(dev->dma);
	return 0;
}

static void stop_streaming(struct vb2_queue *vq)
{
#if 0
	struct cpi_dev *dev = vb2_get_drv_priv(vq);
	struct dmaqueue *dma_q = &dev->vidq;

	/* Stop and reset the DMA engine. */
	dmaengine_terminate_all(dev->dma);

	while (!list_empty(&dma_q->active)) {
		struct rx_buffer *buf;

		buf = list_entry(dma_q->active.next, struct rx_buffer, list);
		if (buf) {
			list_del(&buf->list);
			vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
		}
	}
	list_del_init(&dev->vidq.active);
#endif
}

/*
 * VideoBuffer2 operations
 */
static const struct vb2_ops vb2_video_qops = {
	.queue_setup = queue_setup,
	.buf_prepare = buffer_prepare,
	.buf_queue = buffer_queue,
	.start_streaming = start_streaming,
	.stop_streaming = stop_streaming,
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


	INIT_LIST_HEAD(&cpi->vidq.active);
	init_waitqueue_head(&cpi->vidq.wq);
	memset(q, 0, sizeof(*q));
	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	q->io_modes = VB2_MMAP | VB2_USERPTR;

	q->ops = &vb2_video_qops;
	//TODO q->io_modes = VB2_MMAP | VB2_DMABUF | VB2_READ;
	q->io_modes = VB2_MMAP | VB2_READ;
	//TODO q->mem_ops = &vb2_dma_contig_memops;
	q->mem_ops = &vb2_vmalloc_memops;

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

static void cpi_init_cfg(struct cpi_dev *cpi)
{
	struct v4l2_format f = cpi->format;
	uint32_t val;

	/* Clear camera control register */
	writel(0x0, cpi->base_addr + CPI_CTRL);

	/* Soft reset CPI */
	writel(BIT(8), cpi->base_addr + CPI_CTRL);
	val = readl(cpi->base_addr + CPI_CTRL);
	val &= ~BIT(8);
	writel(val, cpi->base_addr + CPI_CTRL);

	/* MIPI CSI */
	writel(BIT(1) | BIT(4), cpi->base_addr + CPI_CFG);

	/* Set the video frame RAM address into the cpi reg */
	writel(__pa(cpi->vframe), cpi->base_addr + CPI_FRAME_ADDR);

	/* Configure Frame */
	val = (f.fmt.pix.width & GENMASK(13,0)) |
		((f.fmt.pix.height & GENMASK(11,0)) << 16) |
		(1 << 15); //Enable
	writel(val, cpi->base_addr + CPI_VIDEO_FCFG);

	/* Fifo ctrl Read watermark 0xF */
	writel(0xF, cpi->base_addr + CPI_FIFO_CTRL);

	/* Color config 16bit/32bit */
	writel(0x0, cpi->base_addr + CPI_CSI_CMCFG);
}

static int cpi_s_power(struct v4l2_subdev *sd, int on)
{
	struct cpi_dev *cpi = v4l2_get_subdevdata(sd);

	if(on){
		printk("#HGG IN CPI_S_POWER on-> %d\n",on);
		cpi_init_cfg(cpi);
	}
	return 0;
}

static int cpi_s_stream(struct v4l2_subdev *sd, int enable)
{
	/* Not doing much here at the moment
	 * called before sensor s_stream */
	return 0;
}

static const struct v4l2_subdev_video_ops cpi_video_ops = {
	.s_stream = cpi_s_stream,
};

static struct v4l2_subdev_core_ops cpi_core_ops = {
	.s_power = cpi_s_power,
};

static struct v4l2_subdev_ops cpi_subdev_ops = {
	.core = &cpi_core_ops,
	.video = &cpi_video_ops,
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

static irqreturn_t cpi_isr(int irq, void *dev)
{
	struct cpi_dev *cpi = dev;
	static int ratelimit = 0;
	if(ratelimit++ < 10){
		printk("in cpi_isr -> INTR_STATUS = 0x%x\n",readl(cpi->base_addr + 0x4));
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
		printk("Probing Alif CPI !!!\n");
		if(kkk < 4)
			return -EPROBE_DEFER;
	}
	if (!dev->of_node)
		return -ENODEV;

	cpi = devm_kzalloc(dev, sizeof(*cpi), GFP_KERNEL);
	if (!cpi)
		return -ENOMEM;

	cpi->pdev = pdev;

	spin_lock_init(&cpi->slock);
	mutex_init(&cpi->lock);

	/* Registers mapping */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENXIO;
	cpi->base_addr = devm_ioremap_resource(dev, res);
	if (IS_ERR(cpi->base_addr)) {
		dev_err(dev, "Base address not set.\n");
		return PTR_ERR(cpi->base_addr);
	}
	/* TODO HACK - Allocating space only 560x560 bytes at the moment
	*/
	cpi->vframe = devm_kzalloc(dev, 0x230 * 0x230, GFP_KERNEL);
	if (IS_ERR(cpi->vframe)) {
		dev_err(dev, "vfame kmalloc failed.\n");
		return PTR_ERR(cpi->vframe);
	}
#if 0 //A32 DMA not currently available in A0/A1 Silicon
	dev_dbg(&pdev->dev, "Requesting DMA\n");
	cpi->dma = dma_request_slave_channel(&pdev->dev, "vdma0");
	if (cpi->dma == NULL) {
		dev_err(&pdev->dev, "no VDMA channel found\n");
		ret = -ENODEV;
		goto end;
	}
#endif
	cpi->irq = platform_get_irq(pdev, 0);
	if(cpi->irq < 0) {
		dev_err(cpi->dev, "platform_get_irq() failed with %d\n",
			cpi->irq);
		return cpi->irq;
	}


	ret = devm_request_irq(&pdev->dev, cpi->irq, cpi_isr, IRQF_SHARED,
		"CPI", cpi);
	if(ret) {
		dev_err(cpi->dev, "devm_request_irq() failed with %d\n",
			ret);
		return ret;
	}

	ret = cpi_create_capture_subdev(cpi);
	if (ret)
		goto end;


#if 0 //A32 DMA not currently available in A0/A1 Silicon
	vb2_dma_contig_set_max_seg_size(dev, DMA_BIT_MASK(32));
	dev_info(dev, "VIDEOBUF2 DMA CONTIG\n");
#endif
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
