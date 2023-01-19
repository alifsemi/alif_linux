/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2018-2019 Synopsys, Inc. and/or its affiliates.
 *
 * Synopsys DesignWare MIPI CSI-2 Host controller driver
 *
 * Author: Luis Oliveira <Luis.Oliveira@synopsys.com>
 */

#ifndef _DW_MIPI_CSI_H__
#define _DW_MIPI_CSI_H__

#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/phy/phy.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>
#include <linux/ratelimit.h>
#include <linux/reset.h>
#include <linux/videodev2.h>
#include <linux/wait.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/dwc/dw-mipi-csi-pltfrm.h>

/* Advanced features */
#define IPI_DT_OVERWRITE BIT(0)
#define DATA_TYPE_OVERWRITE(dt) (((dt) & GENMASK(5, 0)) << 8)
#define LINE_EVENT_SELECTION(n) ((n) << 16)

enum line_event {
	EVSELAUTO = 0,
	EVSELPROG = 1,
};

#define EN_VIDEO BIT(17)
#define EN_LINE_START BIT(18)
#define EN_NULL BIT(19)
#define EN_BLANKING BIT(20)
#define EN_EMBEDDED BIT(21)
#define IPI_SYNC_EVENT_MODE(n) ((n) << 24)

enum sync_event {
	SYNCEVFSN = 0,
	SYNCEVFS = 1,
};

/* DW MIPI CSI-2 register addresses*/
struct R_CSI2 {
#if 0
	u32 VERSION;	 	//0x0
	u32 N_LANES; 		//0x4
	u32 CTRL_RESETN; 	//0x8
	u32 INTERRUPT; 		//0xc
	u32 DATA_IDS_1;		//0x10
	u32 DATA_IDS_2;
	u32 DATA_IDS_VC_1;
	u32 DATA_IDS_VC_2;
	u32 IPI_MODE;
	u32 IPI_VCID;
	u32 IPI_DATA_TYPE;
	u32 IPI_MEM_FLUSH;
	u32 IPI_HSA_TIME;
	u32 IPI_HBP_TIME;
	u32 IPI_HSD_TIME;
	u32 IPI_HLINE_TIME;
	u32 IPI_SOFTRSTN;
	u32 IPI_ADV_FEATURES;
	u32 IPI_VSA_LINES;
	u32 IPI_VBP_LINES;
	u32 IPI_VFP_LINES;
	u32 IPI_VACTIVE_LINES;
	u32 VC_EXTENSION;
	u32 INT_PHY_FATAL;
	u32 MASK_INT_PHY_FATAL;
	u32 FORCE_INT_PHY_FATAL;
	u32 INT_PKT_FATAL;
	u32 MASK_INT_PKT_FATAL;
	u32 FORCE_INT_PKT_FATAL;
	u32 INT_FRAME_FATAL;
	u32 MASK_INT_FRAME_FATAL;
	u32 FORCE_INT_FRAME_FATAL;
	u32 INT_PHY;
	u32 MASK_INT_PHY;
	u32 FORCE_INT_PHY;
	u32 INT_PKT;
	u32 MASK_INT_PKT;
	u32 FORCE_INT_PKT;
	u32 INT_LINE;
	u32 MASK_INT_LINE;
	u32 FORCE_INT_LINE;
	u32 INT_IPI;
	u32 MASK_INT_IPI;
	u32 FORCE_INT_IPI;
	u32 ST_BNDRY_FRAME_FATAL;
	u32 MSK_BNDRY_FRAME_FATAL;
	u32 FORCE_BNDRY_FRAME_FATAL;
	u32 ST_SEQ_FRAME_FATAL;
	u32 MSK_SEQ_FRAME_FATAL;
	u32 FORCE_SEQ_FRAME_FATAL;
	u32 ST_CRC_FRAME_FATAL;
	u32 MSK_CRC_FRAME_FATAL;
	u32 FORCE_CRC_FRAME_FATAL;
	u32 ST_PLD_CRC_FATAL;
	u32 MSK_PLD_CRC_FATAL;
	u32 FORCE_PLD_CRC_FATAL;
	u32 ST_DATA_ID;
	u32 MSK_DATA_ID;
	u32 FORCE_DATA_ID;
	u32 ST_ECC_CORRECT;
	u32 MSK_ECC_CORRECT;
	u32 FORCE_ECC_CORRECT;
#endif
	u32 VERSION;		//0x0	R	Reserved
	u32 N_LANES;		//0x4	R/W	Lane Configuration Register
	u32 CSI2_RESETN;		//0x8	R/W	CSI Controller Reset Control Register
	u32 INT_ST_MAIN;		//0xC	R	Main Interrupt Status Register
	u32 DATA_IDS_1;		//0x10	R/W	DT Data ID Monitors Configuration Register
	u32 DATA_IDS_VC_1;		//0x30	R/W	VC Data ID Monitors Configuration Register
	u32 PHY_SHUTDOWNZ;		//0x40	R/W	PHY Shutdown Control Register
	u32 DPHY_RSTZ;		//0x44	R/W	PHY Reset Control Register
	u32 PHY_RX;			//0x48	R	PHY RX Signals Status Register
	u32 PHY_STOPSTATE;		//0x4C	R	PHY STOPSTATE Signal Status Register
	u32 PHY_TEST_CTRL0;		//0x50	R/W	PHY Test Control 0 Register
	u32 PHY_TEST_CTRL1;		//0x54	R/W	PHY Test Control 1 Register
	u32 IPI_MODE;		//0x80	R/W	IPI Mode Selection Register
	u32 IPI_VCID;		//0x84	R/W	IPI VC Selection Register
	u32 IPI_DATA_TYPE;		//0x88	R/W	IPI DT Selection Register
	u32 IPI_MEM_FLUSH;		//0x8C	R/W	IPI Memory Flush Control Register
	u32 IPI_HSA_TIME;		//0x90	R/W	IPI HSA Configuration Register
	u32 IPI_HBP_TIME;		//0x94	R/W	IPI HBP Configuration Register
	u32 IPI_HSD_TIME;		//0x98	R/W	IPI HSD Configuration Register
	u32 IPI_HLINE_TIME;		//0x9C	R/W	IPI HLINE Configuration Register
	u32 IPI_SOFTRSTN;		//0xA0	R/W	IPI Reset Control Register
	u32 IPI_ADV_FEATURES;	//0xAC	R/W	IPI Advanced Features Configuration Register
	u32 IPI_VSA_LINES;		//0xB0	R/W	IPI VSA Configuration Register
	u32 IPI_VBP_LINES;		//0xB4	R/W	IPI VBP Configuration Register
	u32 IPI_VFP_LINES;		//0xB8	R/W	IPI VFP Configuration Register
	u32 IPI_VACTIVE_LINES;	//0xBC	R/W	IPI VACTIVE Configuration Register
	u32 VC_EXTENSION;		//0xC8	R/W	VC Extension Configuration Register
	u32 PHY_CAL	;		//0xCC	R	PHY CALIBRATION Signal Status Register
	u32 INT_ST_PHY_FATAL;	//0xE0	R	PHY Packet Discarded Interrupt Status Register
	u32 INT_MSK_PHY_FATAL;	//0xE4	R/W	PHY Packet Discarded Interrupt Mask Register
	u32 INT_FORCE_PHY_FATAL;	//0xE8	R/W	PHY Packet Discarded Interrupt Force Register
	u32 INT_ST_PKT_FATAL;	//0xF0	R	PHY Packet Construction Interrupt Status Register
	u32 INT_MSK_PKT_FATAL;	//0xF4	R/W	PHY Packet Construction Interrupt Mask Register
	u32 INT_FORCE_PKT_FATAL;	//0xF8	R/W	PHY Packet Construction Interrupt Force Register
	u32 INT_ST_PHY;		//0x110	R	PHY Interrupt Status Register
	u32 INT_MSK_PHY;		//0x114	R/W	PHY Interrupt Mask Register
	u32 INT_FORCE_PHY;		//0x118	R/W	PHY Interrupt Force Register
	u32 INT_ST_LINE;		//0x130	R	PHY Line Construction Interrupt Status Register
	u32 INT_MSK_LINE;		//0x134	R/W	PHY Line Construction Interrupt Mask Register
	u32 INT_FORCE_LINE;		//0x138	R/W	PHY Line Construction Interrupt Force Register
	u32 INT_ST_IPI_FATAL;	//0x140	R	IPI Interface Interrupt Status Register
	u32 INT_MSK_IPI_FATAL;	//0x144	R/W	IPI Interface Interrupt Mask Register
	u32 INT_FORCE_IPI_FATAL;	//0x148	R/W	IPI Interface Interrupt Force Register
	u32 INT_ST_BNDRY_FRAME_FATAL;	//0x280	R	Frame Boundary Error Interrupt Status Register
	u32 INT_MSK_BNDRY_FRAME_FATAL;   //0x284	R	Frame Boundary Error Interrupt Mask Register
	u32 INT_FORCE_BNDRY_FRAME_FATAL; //0x288	R	Frame Boundary Error Interrupt Force Register
	u32 INT_ST_SEQ_FRAME_FATAL;	//0x290	R	Frame Sequence Error Interrupt Status Register
	u32 INT_MSK_SEQ_FRAME_FATAL;	//0x294	R	Frame Sequence Error Interrupt Mask Register
	u32 INT_FORCE_SEQ_FRAME_FATAL;   //0x298	R	Frame Sequence Error Interrupt Force Register
	u32 INT_ST_CRC_FRAME_FATAL;	//0x2A0	R	Frame CRC Error Interrupt Status Register
	u32 INT_MSK_CRC_FRAME_FATAL;	//0x2A4	R	Frame CRC Error Interrupt Mask Register
	u32 INT_FORCE_CRC_FRAME_FATAL;   //0x2A8	R	Frame CRC Error Interrupt Force Register
	u32 INT_ST_PLD_CRC_FATAL;	//0x2B0	R	Frame Payload Error Interrupt Status Register
	u32 INT_MSK_PLD_CRC_FATAL;	//0x2B4	R	Frame Payload Error Interrupt Mask Register
	u32 INT_FORCE_PLD_CRC_FATAL;	//0x2B8	R	Frame Payload Error Interrupt Force Register
	u32 INT_ST_DATA_ID;		//0x2C0	R	DT Error Interrupt Status Register
	u32 INT_MSK_DATA_ID;		//0x2C4	R	DT Error Interrupt Mask Register
	u32 INT_FORCE_DATA_ID;		//0x2C8	R	DT Error Interrupt Force Register
	u32 INT_ST_ECC_CORRECT;		//0x2D0	R	ECC Interrupt Status Register
	u32 INT_MSK_ECC_CORRECT;		//0x2D4	R	ECC Interrupt Mask Register
	u32 INT_FORCE_ECC_CORRECT;	//0x2D8	R	ECC Interrupt Force Register
	u32 SCRAMBLING;		//0x300	R/W	Descrambling Control Register
	u32 SCRAMBLING_SEED1;		//0x304	R/W	Descrambling Seed Configuration Lane 0 Register
	u32 SCRAMBLING_SEED2;		//0x308	R/W	Descrambling Seed Configuration Lane 1 Register
};

/* Interrupt Masks */
struct interrupt_type {
	u32 PHY_FATAL;
	u32 PKT_FATAL;
	u32 FRAME_FATAL;
	u32 PHY;
	u32 PKT;
	u32 LINE;
	u32 IPI;
	u32 BNDRY_FRAME_FATAL;
	u32 SEQ_FRAME_FATAL;
	u32 CRC_FRAME_FATAL;
	u32 PLD_CRC_FATAL;
	u32 DATA_ID;
	u32 ECC_CORRECTED;
};

/* IPI Data Types */
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

/* DWC MIPI CSI-2 output types */
enum output {
	IPI_OUT = 0,
	IDI_OUT = 1,
	BOTH_OUT = 2
};

/* IPI color components */
enum color_mode {
	COLOR48 = 0,
	COLOR16 = 1
};

/* IPI cut through */
enum cut_through {
	CTINACTIVE = 0,
	CTACTIVE = 1
};

/* IPI output types */
enum ipi_output {
	CAMERA_TIMING = 0,
	AUTO_TIMING = 1
};

/* Format template */
struct mipi_fmt {
	u32 mbus_code;
	u8 depth;
};

struct mipi_dt {
	u32 hex;
	char *name;
};

/* CSI specific configuration */
struct csi_data {
	u32 num_lanes;
	u32 dphy_freq;
	u32 pclk;
	u32 fps;
	u32 bpp;
	u32 output;
	u32 ipi_mode;
	u32 ipi_adv_features;
	u32 ipi_cut_through;
	u32 ipi_color_mode;
	u32 ipi_auto_flush;
	u32 virtual_ch;
	u32 hsa;
	u32 hbp;
	u32 hsd;
	u32 htotal;
	u32 vsa;
	u32 vbp;
	u32 vfp;
	u32 vactive;
};

/* Structure to embed device driver information */
struct dw_csi {
	struct v4l2_subdev sd;
	struct video_device vdev;
	struct v4l2_device v4l2_dev;
	struct device *dev;
	struct media_pad pads[CSI_PADS_NUM];
	struct mipi_fmt *fmt;
	struct v4l2_mbus_framefmt format;
	void __iomem *base_address;
	void __iomem *demo;
	void __iomem *csc;
	int ctrl_irq_number;
	int demosaic_irq;
	struct csi_data hw;
	struct reset_control *rst;
	struct phy *phy;
	struct dw_csih_pdata *config;
	struct mutex lock; /* protect resources sharing */
	spinlock_t slock; /* interrupt handling lock */
	u8 ipi_dt;
	u8 index;
	u8 hw_version_major;
	u16 hw_version_minor;
};

static inline struct dw_csi *sd_to_mipi_csi_dev(struct v4l2_subdev *sdev)
{
	return container_of(sdev, struct dw_csi, sd);
}

void dw_mipi_csi_reset(struct dw_csi *csi_dev);
int dw_mipi_csi_mask_irq_power_off(struct dw_csi *csi_dev);
int dw_mipi_csi_hw_stdby(struct dw_csi *csi_dev);
void dw_mipi_csi_set_ipi_fmt(struct dw_csi *csi_dev);
void dw_mipi_csi_start(struct dw_csi *csi_dev);
void dw_mipi_csi_start_ipi(struct dw_csi *csi_dev, int enable);
int dw_mipi_csi_irq_handler(struct dw_csi *csi_dev);
void dw_mipi_csi_get_version(struct dw_csi *csi_dev);
int dw_mipi_csi_specific_mappings(struct dw_csi *csi_dev);
void dw_mipi_csi_fill_timings(struct dw_csi *dev,
			      struct v4l2_subdev_format *fmt);
void dw_mipi_csi_dump(struct dw_csi *csi_dev);

#if IS_ENABLED(CONFIG_DWC_MIPI_TC_DPHY_GEN3)
int dw_csi_create_capabilities_sysfs(struct platform_device *pdev);
#endif

static inline void dw_mipi_csi_write(struct dw_csi *dev,
				     u32 address, u32 data)
{
	writel(data, dev->base_address + address);
}

static inline u32 dw_mipi_csi_read(struct dw_csi *dev, u32 address)
{
	return readl(dev->base_address + address);
}

#endif /*_DW_MIPI_CSI_H__ */
