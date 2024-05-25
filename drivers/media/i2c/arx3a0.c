// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2022 Alif Semiconductor
 *
 * Written by Harith George
 * Heavily based on AR0521 driver by Krzysztof Hałasa
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <linux/pm_runtime.h>

#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

/* External clock (extclk) frequencies */
#define ARX3A0_EXTCLK_MIN		 	(6 * 1000 * 1000)
#define ARX3A0_EXTCLK_MAX			(48 * 1000 * 1000)

/* PLL and PLL2 */
#define ARX3A0_PLL_MIN				(320 * 1000 * 1000)
#define ARX3A0_PLL_MAX				(1280 * 1000 * 1000)

/* Effective pixel clocks, the registers may be DDR */
#define ARX3A0_PIXEL_CLOCK_RATE			(78 * 1000 * 1000)
#define ARX3A0_PIXEL_CLOCK_MIN			(77 * 1000 * 1000)
#define ARX3A0_PIXEL_CLOCK_MAX			(80 * 1000 * 1000)

#define ARX3A0_WIDTH_MIN			0u
#define ARX3A0_WIDTH_MAX			560u
#define ARX3A0_HEIGHT_MIN			0u
#define ARX3A0_HEIGHT_MAX			560u

#define ARX3A0_WIDTH_BLANKING_MIN		572u
#define ARX3A0_HEIGHT_BLANKING_MIN		38u /* must be even */
#define ARX3A0_TOTAL_WIDTH_MIN			560u

/* ARX3A0 registers */
#define ARX3A0_REG_VT_PIX_CLK_DIV		0x0300
#define ARX3A0_REG_FRAME_LENGTH_LINES		0x0340

#define ARX3A0_REG_CHIP_ID			0x3000
#define ARX3A0_REG_COARSE_INTEGRATION_TIME	0x3012
#define ARX3A0_REG_EXTRA_DELAY			0x3018

#define ARX3A0_REG_RESET			0x301A
#define ARX3A0_REG_RESET_DEFAULTS		0x0018
#define ARX3A0_REG_RESET_GROUP_PARAM_HOLD	BIT(15)
#define ARX3A0_REG_RESET_STREAM			BIT(2)
#define ARX3A0_REG_RESET_RESTART		BIT(1)
#define ARX3A0_REG_RESET_INIT			BIT(0)

#define ARX3A0_REG_GLOBAL_GAIN			0x305E

#define ARX3A0_REG_TEST_PATTERN_MODE		0x3070

#define ARX3A0_REG_SERIAL_FORMAT		0x31AE
#define ARX3A0_REG_SERIAL_FORMAT_MIPI		0x0200

/* ARX3A0 Camera Sensor CHIP-ID registers */
#define ARX3A0_CHIP_ID_REGISTER			0x3000
#define ARX3A0_CHIP_ID_REGISTER_VALUE		0x0353

/* ARX3A0 Camera Sensor registers index */
#define ARX3A0_SOFTWARE_RESET_REGISTER		0x0103
#define ARX3A0_MODE_SELECT_REGISTER		0x0100
#define ARX3A0_MIPI_CONFIG_REGISTER		0x31BE

#define be		cpu_to_be16

#define REGS_ENTRY(a)	{(a), ARRAY_SIZE(a)}
#define REGS(...)	REGS_ENTRY(((const __be16[]){__VA_ARGS__}))

static const struct initial_reg {
	const __be16 *data; /* data[0] is register address */
	unsigned int count;
} initial_regs_raw10[] = {

	/*LOAD= PLL_360Fps_80MHz_80MHz_20MHz*/
	REGS(be(0x300), be(0x000A)),
	REGS(be(0x302), be(0x0001)),
	REGS(be(0x304), be(0x0101)),
	REGS(be(0x306), be(0x2828)),
	REGS(be(0x308), be(0x000A)),
	REGS(be(0x30A), be(0x0001)),

	/* MIPI_TIMING_10bit */
	REGS(be(0x31B0),
	     be(0x0047),   /* 31B0: frame_preamble */
	     be(0x0026),   /* 31B2: line_preamble */
	     be(0x328C),   /* 31B4: */
	     be(0x32E8),   /* 31B6: */
	     be(0x1C12),   /* 31B8: */
	     be(0x1452),   /* 31BA: */
	     be(0x8488)),  /* 31BC: */

	/*LOAD= Analog_Setup_Recommended*/
	REGS(be(0x3ED0), be(0x0748)), /* 3ED0: TXLO from atest/sf bin settings */
	REGS(be(0x3ED6), be(0x3136)), /* 3ED6: Txlatch fr cfpn rows/vln bias */
	REGS(be(0x3EDC), be(0x1020)), /* 3EDC: over range for rst and under range for sig */
	REGS(be(0x3EDE), be(0x1D2A)), /* 3EDE: over range for sig and col dec clk settings */
	REGS(be(0x3EE0), be(0x282A)),
	REGS(be(0x3EE2), be(0x2821)),
	REGS(be(0x3EC8), be(0x0401)),
	REGS(be(0x3ED2), be(0x3903)), /* 3ED2: Ramp offset */
	REGS(be(0x3EC0), be(0x0011)), /* 3EC0: SFbin/SH mode settings */
	REGS(be(0x3ECA), be(0x826F)), /* 3ECA: CDAC/Txlo2/RSTGHI/RSTGLO settings */
	REGS(be(0x3EBC), be(0xA8AA)), /* 3EBC: Bias currents for FSC/ECL */
	REGS(be(0x3EC4), be(0x1000)),
	REGS(be(0x3EBA), be(0x0044)),

	/*LOAD= Corrections_Recommended*/
	REGS(be(0x3ED0), be(0x0745)),  /* 3ED0: TXLO from atest/sf bin settings */
	REGS(be(0x3ED4), be(0x0016)),  /* 3ED4: TXLO open loop/row driver settings */
	REGS(be(0x3EC6), be(0x80F2)),  /* 3EC6: VLN and clk gating controls */
	REGS(be(0x3ED8), be(0x55FF)),  /* 3ED8: Ramp step setting for 10 bit 400 Mhz */
	REGS(be(0x3EE6), be(0x8000)),
	REGS(be(0x30D2), be(0x0000)),  /* 30D2: CRM/CC: enable crm on Visible and CC rows */
	REGS(be(0x31E0), be(0x00F1)),
	REGS(be(0x31E6), be(0xA35F)),
	REGS(be(0x3180), be(0x9096)),  /* FDOC:fdoc settings */
	REGS(be(0x3120), be(0x0001)),
	REGS(be(0x301E), be(0x002A)),  /* PEDESTAL+2 :+2 is a workaround for 10bit mode +0.5 rounding */

	/*LOAD= Pixel_Timing_Recommended_10bit*/
	REGS(be(0x3D00),
	     /* 3D00 */ be(0x0436), be(0x435A), be(0xFFFF), be(0xFFFF),
	     /* 3D08 */ be(0x2180), be(0x0005), be(0x108F), be(0x0802),
	     /* 3D10 */ be(0x5248), be(0x801B), be(0x006F), be(0x8269),
	     /* 3D18 */ be(0x6A82), be(0x5148), be(0x5A80), be(0x5902),
	     /* 3D20 */ be(0x8082), be(0x3060), be(0x8567), be(0x5C20),
	     /* 3D28 */ be(0x4880), be(0x0284), be(0x6084), be(0x5C91),
	     /* 3D30 */ be(0x5980), be(0x5883), be(0x6462), be(0x8056),
	     /* 3D38 */ be(0x8162), be(0x8422), be(0x20A2), be(0x2220)),
	REGS(be(0x3D40),
	     /* 3D40 */ be(0x804B), be(0x8110), be(0x0943), be(0x9243),
	     /* 3D48 */ be(0x8050), be(0x9A4B), be(0x8563), be(0x8363),
	     /* 3D50 */ be(0x8422), be(0x20A2), be(0x61C6), be(0x6F99),
	     /* 3D58 */ be(0x3009), be(0x1FF6), be(0x20ED), be(0x0874),
	     /* 3D60 */ be(0x8230), be(0x609B), be(0x3060), be(0x4600),
	     /* 3D68 */ be(0x3783), be(0x7070), be(0x8040), be(0x4A44),
	     /* 3D70 */ be(0x8003), be(0x0086), be(0x4588), be(0x46BA),
	     /* 3D78 */ be(0x0300), be(0xFFD7), be(0x4688), be(0x4588),
	     /* 3D80 */ be(0x4492), be(0x4A9B), be(0x4070), be(0x8040),
	     /* 3D88 */ be(0x4AAD), be(0x0070), be(0xAE47), be(0x8547),
	     /* 3D90 */ be(0xAD78), be(0x6B85), be(0x6A80), be(0x6984),
	     /* 3D98 */ be(0x6B8A), be(0x6B80), be(0x6980), be(0x6A85),
	     /* 3DA0 */ be(0x7C93), be(0x846B), be(0x8465), be(0x46FF),
	     /* 3DA8 */ be(0xAA65), be(0x9C79), be(0x4A00), be(0x2180),
	     /* 3DB0 */ be(0x44AC), be(0x7070), be(0x2180), be(0x0005),
	     /* 3DB8 */ be(0x108F), be(0x0802), be(0x5248), be(0x801B),
	     /* 3DC0 */ be(0x006F), be(0x8269), be(0x6A82), be(0x5148),
	     /* 3DC8 */ be(0x5A80), be(0x5902), be(0x8082), be(0x3060),
	     /* 3DD0 */ be(0x8567), be(0x5C20), be(0x4880), be(0x0284),
	     /* 3DD8 */ be(0x6084), be(0x5C91), be(0x5980), be(0x5883),
	     /* 3DE0 */ be(0x6462), be(0x8056), be(0x8162), be(0x8422),
	     /* 3DE8 */ be(0x209C), be(0x2220), be(0x514B), be(0x8110),
	     /* 3DF0 */ be(0x0943), be(0x9843), be(0x8050), be(0x8B51),
	     /* 3DF8 */ be(0x8D4B), be(0x9063), be(0x8363), be(0x8422)),
	REGS(be(0x3E00),
	     /* 3E00 */ be(0x209C), be(0x61D3), be(0x1FB6), be(0x20ED),
	     /* 3E08 */ be(0x3809), be(0x524B), be(0x0014), be(0x4580),
	     /* 3E10 */ be(0x4681), be(0x3060), be(0x9D30), be(0x6083),
	     /* 3E18 */ be(0x4645), be(0x0017), be(0x8170), be(0x7070),
	     /* 3E20 */ be(0x7070), be(0x7070), be(0x7070), be(0x7070),
	     /* 3E28 */ be(0x7070), be(0x7070), be(0x7070), be(0x7070),
	     /* 3E30 */ be(0x7070), be(0x7070), be(0x7070), be(0x7070),
	     /* 3E38 */ be(0x7070), be(0x7070), be(0x7070), be(0x7070),
	     /* 3E40 */ be(0x7070), be(0x7070), be(0x7070), be(0x7070),
	     /* 3E48 */ be(0x7070), be(0x7070), be(0x7070), be(0x7070),
	     /* 3E50 */ be(0x7070), be(0x7070), be(0x7070), be(0x7070),
	     /* 3E58 */ be(0x7070), be(0x7070), be(0x7070), be(0x7070),
	     /* 3E60 */ be(0x7070), be(0x7070), be(0x7070), be(0x7070),
	     /* 3E68 */ be(0x7070), be(0x7070), be(0x7070), be(0x7070),
	     /* 3E70 */ be(0x7070), be(0x7070), be(0x7070), be(0x7070),
	     /* 3E78 */ be(0x7070), be(0x7070), be(0x7070), be(0x7070),
	     /* 3E80 */ be(0x7070), be(0x7070), be(0x7070), be(0x7070),
	     /* 3E88 */ be(0x7070), be(0x7070), be(0x7070), be(0x7070),
	     /* 3E90 */ be(0x7070), be(0x7070), be(0x7070), be(0x7070),
	     /* 3E98 */ be(0x7070), be(0x7070), be(0x7070), be(0x7070),
	     /* 3EA0 */ be(0x7070), be(0x7070), be(0x7070), be(0x7070),
	     /* 3EA8 */ be(0x7070), be(0x7070), be(0x7070), be(0x7070),
	     /* 3EB0 */ be(0x7070), be(0x7070), be(0x7070)),

	REGS(be(0x0104), be(0x0001)), /* Group_Parameter_Hold hold reg inserts */
	REGS(be(0x0344), be(0x0004)),
	REGS(be(0x0346), be(0x0004)),
	REGS(be(0x0348), be(0x0233)),
	REGS(be(0x034A), be(0x0233)),
	REGS(be(0x034C), be(0x0233)),
	REGS(be(0x034E), be(0x0230)),

	REGS(be(0x3040), be(0x0041)), /* Y_ODD_INC */
	REGS(be(0x30A4), be(0x0001)), /* Y_EVEN_INC */
	REGS(be(0x0342), be(0x02F8)),
	REGS(be(0x0340), be(0x0248)),
	REGS(be(0x3012), be(0x0107)),
	REGS(be(0x0112), be(0x0A0A)), /* RAW10 */
	REGS(be(0x0202), be(0x366A)),
	REGS(be(0x0300), be(0x000A)),
	REGS(be(0x0302), be(0x0001)),
	REGS(be(0x0304), be(0x0101)),
	REGS(be(0x0306), be(0x2828)),
	REGS(be(0x0308), be(0x000A)),
	REGS(be(0x030A), be(0x0001)),
	REGS(be(0x0340), be(0xA47B),
		be(0x02F8),		/* line_length_pck */
		be(0x0004),		/* X_ADDR_START */
		be(0x0004),		/* Y_ADDR_START */
		be(0x0233),		/* X_ADDR_END */
		be(0x0233),		/* Y_ADDR_END */
		be(0x0230),		/* X_OUTPUT_SIZE */
		be(0x0230)),		/* Y_OUTPUT_SIZE */
	REGS(be(0x0382), be(0x0001)),
	REGS(be(0x0386), be(0x0001)),
	REGS(be(0x0400), be(0x0000)),
	REGS(be(0x0402), be(0x0000)),
	REGS(be(0x0404), be(0x0010)),

	REGS(be(0x3000), be(0x0353)),
	REGS(be(0x3002), be(0x0004)),
	REGS(be(0x3004), be(0x0004)),
	REGS(be(0x3006), be(0x0233)),
	REGS(be(0x3008), be(0x0233)),
	REGS(be(0x300A), be(0xA47B)),
	REGS(be(0x300C), be(0x02F8)),

	REGS(be(0x3012), be(0x366A)),
	REGS(be(0x3018), be(0x0000)),
	REGS(be(0x301A), be(0x001C)),
	REGS(be(0x301C), be(0x0001)),
	REGS(be(0x301D), be(0x0000)),
	REGS(be(0x301E), be(0x002A)),
	REGS(be(0x3021), be(0x0000)),
	REGS(be(0x3022), be(0x0000)),
	REGS(be(0x3023), be(0x0000)),
	REGS(be(0x3026), be(0xFFFF)),
	REGS(be(0x3028), be(0x0004)),
	REGS(be(0x3032), be(0x0100)),
	REGS(be(0x303E), be(0x0000)),
	REGS(be(0x3040), be(0x0041)),
	REGS(be(0x3044), be(0x10C0)),
	REGS(be(0x3046), be(0x0608)),
	REGS(be(0x3048), be(0x0008)),
	REGS(be(0x304A), be(0x0060)),
	REGS(be(0x304C), be(0x0200)),
	REGS(be(0x305E), be(0x2000)), /* Init value for Global Gain */
	REGS(be(0x3064), be(0x5840)),
	REGS(be(0x3068), be(0x0000)),
	REGS(be(0x306A), be(0x0000)),
	REGS(be(0x306E), be(0x9000)),
	REGS(be(0x3070), be(0x0000)),
	REGS(be(0x3072), be(0x0000)),
	REGS(be(0x3074), be(0x0000)),
	REGS(be(0x3076), be(0x0000)),
	REGS(be(0x3078), be(0x0000)),
	REGS(be(0x307A), be(0x0000)),
	REGS(be(0x307E), be(0x0020)),
	REGS(be(0x3088), be(0x0001)),
	REGS(be(0x30A0), be(0x0001)),
	REGS(be(0x30A2), be(0x0001)),
	REGS(be(0x30A4), be(0x0001)),
	REGS(be(0x30A6), be(0x0001)),
	REGS(be(0x30AA), be(0x0000)),
	REGS(be(0x30B0), be(0x0400)),
	REGS(be(0x30BC), be(0x0000)),
	REGS(be(0x30BE), be(0x0000)),
	REGS(be(0x30C0), be(0x2000)),
	REGS(be(0x30C2), be(0x0000)),
	REGS(be(0x30E8), be(0x0000)),
	REGS(be(0x30EA), be(0x0000)),
	REGS(be(0x30EC), be(0x5AE7)),
	REGS(be(0x30F8), be(0x0033)),
	REGS(be(0x30FA), be(0xFC4C)), /* GPIO0, GPIO1 */
	REGS(be(0x3120), be(0x0001)), /* GAIN dither enable */
	REGS(be(0x3122), be(0x0007)),
	REGS(be(0x3124), be(0x01A7)),
	REGS(be(0x3126), be(0x0000)),
	REGS(be(0x3128), be(0x01CF)),
	REGS(be(0x312A), be(0x4567)),
	REGS(be(0x312C), be(0x89AB)),
	REGS(be(0x312E), be(0xCDEF)),
	REGS(be(0x3152), be(0x0010)),
	REGS(be(0x3154), be(0x3200)),
	REGS(be(0x3156), be(0xC8F7)),
	REGS(be(0x3158), be(0x0000)),
	REGS(be(0x315A), be(0x0000)),
	REGS(be(0x315C), be(0x0000)),
	REGS(be(0x315E), be(0x0000)),
	REGS(be(0x3160), be(0x00EC)),
	REGS(be(0x3162), be(0x0317)),
	REGS(be(0x3164), be(0x0000)),
	REGS(be(0x0104), be(0x0000)), /* Group_Parameter Hold - Insert register values */
};

static const struct initial_reg initial_regs_raw8[] = {
	/* LOAD= OTPM */
	REGS(be(0x304C), be(0x3000)),
	REGS(be(0x304A), be(0x0070)),

	/*LOAD= Analog_Setup_Recommended*/
	REGS(be(0x3ED0), be(0x0748)),
	REGS(be(0x3ED6), be(0x3136)),
	REGS(be(0x3EDC), be(0x1020)),
	REGS(be(0x3EDE), be(0x1D2A)),
	REGS(be(0x3EE0), be(0x282A)),
	REGS(be(0x3EE2), be(0x2821)),
	REGS(be(0x3EC8), be(0x0401)),
	REGS(be(0x3ED2), be(0x3903)),
	REGS(be(0x3EC0), be(0x0011)),
	REGS(be(0x3ECA), be(0x826F)),
	REGS(be(0x3EBC), be(0xA8AA)),
	REGS(be(0x3EC4), be(0x1000)),
	REGS(be(0x3EBA), be(0x0044)),

	/*LOAD= Pixel_Timing_Recommended_10bit*/
	REGS(be(0x3D00),
	     /* 3D00 */ be(0x0436), be(0x435A), be(0xFFFF), be(0xFFFF),
	     /* 3D08 */ be(0x2180), be(0x0005), be(0x108F), be(0x0802),
	     /* 3D10 */ be(0x5248), be(0x801B), be(0x006F), be(0x8269),
	     /* 3D18 */ be(0x6A82), be(0x5148), be(0x5A80), be(0x5902),
	     /* 3D20 */ be(0x8082), be(0x3060), be(0x8567), be(0x5C20),
	     /* 3D28 */ be(0x4880), be(0x0284), be(0x6084), be(0x5C91),
	     /* 3D30 */ be(0x5980), be(0x5883), be(0x6462), be(0x8056),
	     /* 3D38 */ be(0x8162), be(0x8422), be(0x20A2), be(0x2220)),
	REGS(be(0x3D40),
	     /* 3D40 */ be(0x804B), be(0x8110), be(0x0943), be(0x9243),
	     /* 3D48 */ be(0x8050), be(0x9A4B), be(0x8563), be(0x8363),
	     /* 3D50 */ be(0x8422), be(0x20A2), be(0x61C6), be(0x6F99),
	     /* 3D58 */ be(0x3009), be(0x1FF6), be(0x20ED), be(0x0874),
	     /* 3D60 */ be(0x8230), be(0x609B), be(0x3060), be(0x4600),
	     /* 3D68 */ be(0x3783), be(0x7070), be(0x8040), be(0x4A44),
	     /* 3D70 */ be(0x8003), be(0x0086), be(0x4588), be(0x46BA),
	     /* 3D78 */ be(0x0300), be(0xFFD7), be(0x4688), be(0x4588),
	     /* 3D80 */ be(0x4492), be(0x4A9B), be(0x4070), be(0x8040),
	     /* 3D88 */ be(0x4AAD), be(0x0070), be(0xAE47), be(0x8547),
	     /* 3D90 */ be(0xAD78), be(0x6B85), be(0x6A80), be(0x6984),
	     /* 3D98 */ be(0x6B8A), be(0x6B80), be(0x6980), be(0x6A85),
	     /* 3DA0 */ be(0x7C93), be(0x846B), be(0x8465), be(0x46FF),
	     /* 3DA8 */ be(0xAA65), be(0x9C79), be(0x4A00), be(0x2180),
	     /* 3DB0 */ be(0x44AC), be(0x7070), be(0x2180), be(0x0005),
	     /* 3DB8 */ be(0x108F), be(0x0802), be(0x5248), be(0x801B),
	     /* 3DC0 */ be(0x006F), be(0x8269), be(0x6A82), be(0x5148),
	     /* 3DC8 */ be(0x5A80), be(0x5902), be(0x8082), be(0x3060),
	     /* 3DD0 */ be(0x8567), be(0x5C20), be(0x4880), be(0x0284),
	     /* 3DD8 */ be(0x6084), be(0x5C91), be(0x5980), be(0x5883),
	     /* 3DE0 */ be(0x6462), be(0x8056), be(0x8162), be(0x8422),
	     /* 3DE8 */ be(0x209C), be(0x2220), be(0x514B), be(0x8110),
	     /* 3DF0 */ be(0x0943), be(0x9843), be(0x8050), be(0x8B51),
	     /* 3DF8 */ be(0x8D4B), be(0x9063), be(0x8363), be(0x8422)),
	REGS(be(0x3E00),
	     /* 3E00 */ be(0x209C), be(0x61D3), be(0x1FB6), be(0x20ED),
	     /* 3E08 */ be(0x3809), be(0x524B), be(0x0014), be(0x4580),
	     /* 3E10 */ be(0x4681), be(0x3060), be(0x9D30), be(0x6083),
	     /* 3E18 */ be(0x4645), be(0x0017), be(0x8170), be(0x7070),
	     /* 3E20 */ be(0x7070), be(0x7070), be(0x7070), be(0x7070),
	     /* 3E28 */ be(0x7070), be(0x7070), be(0x7070), be(0x7070),
	     /* 3E30 */ be(0x7070), be(0x7070), be(0x7070), be(0x7070),
	     /* 3E38 */ be(0x7070), be(0x7070), be(0x7070), be(0x7070),
	     /* 3E40 */ be(0x7070), be(0x7070), be(0x7070), be(0x7070),
	     /* 3E48 */ be(0x7070), be(0x7070), be(0x7070), be(0x7070),
	     /* 3E50 */ be(0x7070), be(0x7070), be(0x7070), be(0x7070),
	     /* 3E58 */ be(0x7070), be(0x7070), be(0x7070), be(0x7070),
	     /* 3E60 */ be(0x7070), be(0x7070), be(0x7070), be(0x7070),
	     /* 3E68 */ be(0x7070), be(0x7070), be(0x7070), be(0x7070),
	     /* 3E70 */ be(0x7070), be(0x7070), be(0x7070), be(0x7070),
	     /* 3E78 */ be(0x7070), be(0x7070), be(0x7070), be(0x7070),
	     /* 3E80 */ be(0x7070), be(0x7070), be(0x7070), be(0x7070),
	     /* 3E88 */ be(0x7070), be(0x7070), be(0x7070), be(0x7070),
	     /* 3E90 */ be(0x7070), be(0x7070), be(0x7070), be(0x7070),
	     /* 3E98 */ be(0x7070), be(0x7070), be(0x7070), be(0x7070),
	     /* 3EA0 */ be(0x7070), be(0x7070), be(0x7070), be(0x7070),
	     /* 3EA8 */ be(0x7070), be(0x7070), be(0x7070), be(0x7070),
	     /* 3EB0 */ be(0x7070), be(0x7070), be(0x7070)),

	/*LOAD= Corrections_Recommended*/
	REGS(be(0x3ED0), be(0x0745)),
	REGS(be(0x3ED4), be(0x0016)),
	REGS(be(0x3EC6), be(0x80F2)),
	REGS(be(0x3ED8), be(0x55FF)),
	REGS(be(0x3EE6), be(0x8000)),
	REGS(be(0x30D2), be(0x0000)),
	REGS(be(0x31E0), be(0x00F1)),
	REGS(be(0x31E6), be(0xA35F)),
	REGS(be(0x3180), be(0x9096)),
	REGS(be(0x3120), be(0x0001)),
	REGS(be(0x301E), be(0x002A)),

	/*LOAD= PLL_360Fps_80MHz_80MHz_20MHz*/
	REGS(be(0x0300), be(0x000A)),
	REGS(be(0x0302), be(0x0001)),
	REGS(be(0x0304), be(0x0101)),
	REGS(be(0x0306), be(0x2828)),
	REGS(be(0x0308), be(0x000A)),
	REGS(be(0x030A), be(0x0001)),

	/* MIPI_TIMING_10bit */
	REGS(be(0x31B0),
	     be(0x0048),   /* 31B0: frame_preamble */
	     be(0x0027),   /* 31B2: line_preamble */
	     be(0x424C),   /* 31B4: */
	     be(0x3308),   /* 31B6: */
	     be(0x1C13),   /* 31B8: */
	     be(0x1450),   /* 31BA: */
	     be(0x8488)),  /* 31BC: */

	/* Group_Parameter_Hold - hold reg inserts */
	REGS(be(0x0104), be(0x0001)),
	REGS(be(0x034C), be(0x0230)),
	REGS(be(0x034E), be(0x0230)),

	REGS(be(0x3012), be(0x0304)),
	REGS(be(0x0112), be(0x0808)), /* RAW8 */

	REGS(be(0x0300), be(0x000C)),
	REGS(be(0x0302), be(0x0001)),
	REGS(be(0x0304), be(0x0101)),
	REGS(be(0x0306), be(0x2030)),
	REGS(be(0x0308), be(0x0008)),
	REGS(be(0x030A), be(0x0001)),
	REGS(be(0x0340), be(0xA78E),
		be(0x02EA),		/* line_length_pck */
		be(0x0004),		/* X_ADDR_START */
		be(0x0004),		/* Y_ADDR_START */
		be(0x0234),		/* X_ADDR_END */
		be(0x0233),		/* Y_ADDR_END */
		be(0x0230),		/* X_OUTPUT_SIZE */
		be(0x0230)),		/* Y_OUTPUT_SIZE */
	REGS(be(0x0382), be(0x0001)),
	REGS(be(0x0386), be(0x0001)),
	REGS(be(0x0400), be(0x0000)),
	REGS(be(0x0402), be(0x0000)),
	REGS(be(0x0404), be(0x0010)),

	REGS(be(0x3000), be(0x0353)),
	REGS(be(0x3002), be(0x0004)),
	REGS(be(0x3004), be(0x0004)),
	REGS(be(0x3006), be(0x0233)),
	REGS(be(0x3008), be(0x0234)),
	REGS(be(0x300A), be(0xA78E)),
	REGS(be(0x300C), be(0x02EA)),

	REGS(be(0x3012), be(0x0304)),
	REGS(be(0x3018), be(0x0000)),
	REGS(be(0x301A), be(0x001C)),
	REGS(be(0x301C), be(0x0001)),
	REGS(be(0x301D), be(0x0000)),
	REGS(be(0x301E), be(0x002A)),
	REGS(be(0x3021), be(0x0000)),
	REGS(be(0x3023), be(0x0000)),
	REGS(be(0x3026), be(0xFFFF)),
	REGS(be(0x3028), be(0x0004)),
	REGS(be(0x3032), be(0x0100)),
	REGS(be(0x303E), be(0x0000)),
	REGS(be(0x3040), be(0x0041)),
	REGS(be(0x3044), be(0x10C0)),
	REGS(be(0x3046), be(0x0608)),
	REGS(be(0x3048), be(0x0008)),
	REGS(be(0x304A), be(0x0070)),
	REGS(be(0x304C), be(0x3000)),
	REGS(be(0x305E), be(0x2040)),
	REGS(be(0x3064), be(0x5840)),
	REGS(be(0x3068), be(0x0000)),
	REGS(be(0x306E), be(0x9080)),
	REGS(be(0x3070), be(0x0000)),
	REGS(be(0x3072), be(0x0000)),
	REGS(be(0x3074), be(0x0000)),
	REGS(be(0x3076), be(0x0000)),
	REGS(be(0x3078), be(0x0000)),
	REGS(be(0x307A), be(0x0000)),
	REGS(be(0x307E), be(0x0020)),
	REGS(be(0x3088), be(0x0001)),
	REGS(be(0x30A0), be(0x0001)),
	REGS(be(0x30A2), be(0x0001)),
	REGS(be(0x30A4), be(0x0001)),
	REGS(be(0x30A6), be(0x0001)),
	REGS(be(0x30AA), be(0x0000)),
	REGS(be(0x30B0), be(0x0400)),
	REGS(be(0x30BC), be(0x0000)),
	REGS(be(0x30BE), be(0x0000)),
	REGS(be(0x30C0), be(0x2000)),
	REGS(be(0x30C2), be(0x0000)),
	REGS(be(0x30E8), be(0x0000)),
	REGS(be(0x30EA), be(0x0000)),
	REGS(be(0x30EC), be(0x5AE7)),
	REGS(be(0x30F8), be(0x0033)),
	REGS(be(0x30FA), be(0xFC4C)), /* GPIO0, GPIO1 */
	REGS(be(0x3120), be(0x0001)), /* GAIN dither enable */
	REGS(be(0x3122), be(0x0007)),
	REGS(be(0x3124), be(0x01A7)),
	REGS(be(0x3126), be(0x0000)),
	REGS(be(0x3128), be(0x01CF)),
	REGS(be(0x312A), be(0x4567)),
	REGS(be(0x312C), be(0x89AB)),
	REGS(be(0x312E), be(0xCDEF)),
	REGS(be(0x3152), be(0x0010)),
	REGS(be(0x3154), be(0x3200)),
	REGS(be(0x3156), be(0xC8F7)),
	REGS(be(0x3158), be(0x0000)),
	REGS(be(0x315A), be(0x0000)),
	REGS(be(0x315C), be(0x0000)),
	REGS(be(0x315E), be(0x0000)),
	REGS(be(0x3160), be(0x00EC)),
	REGS(be(0x3162), be(0x0317)),
	REGS(be(0x3164), be(0x0000)),
	REGS(be(0x3166), be(0x0327)),
	REGS(be(0x3168), be(0x0000)),
	REGS(be(0x3170), be(0x236E)),
	REGS(be(0x3172), be(0x0201)),
	REGS(be(0x3174), be(0x0000)),
	REGS(be(0x3176), be(0x1000)),
	REGS(be(0x317A), be(0x416E)),
	REGS(be(0x31AE), be(0x0202)),
	REGS(be(0x3730), be(0x0000)),
	REGS(be(0x3F3C), be(0x0003)),
	/* Group_Parameter Hold - Release */
	REGS(be(0x0104), be(0x0000)),
};

static const char * const arx3a0_supply_names[] = {
	"vdd_io",	/* I/O (1.8V) supply */
	"vdd",		/* Core, PLL and MIPI (1.2V) supply */
	"vaa",		/* Analog (2.7V) supply */
};

struct arx3a0_ctrls {
	struct v4l2_ctrl_handler handler;
	struct {
		struct v4l2_ctrl *hblank;
		struct v4l2_ctrl *vblank;
	};
	struct v4l2_ctrl *pixrate;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *test_pattern;
};

struct arx3a0_dev {
	struct i2c_client *i2c_client;
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct clk *extclk;
	u32 extclk_freq;

	struct regulator *supplies[ARRAY_SIZE(arx3a0_supply_names)];
	struct gpio_desc *reset_gpio;
	struct gpio_desc *power_gpio;

	/* lock to protect all members below */
	struct mutex lock;

	struct v4l2_mbus_framefmt fmt;
	struct arx3a0_ctrls ctrls;
	unsigned int lane_count;
	u16 total_width;
	u16 total_height;
	u16 pll_pre;
	u16 pll_mult;
	u16 pll_pre2;
	u16 pll_mult2;
	bool streaming;
};

static inline struct arx3a0_dev *to_arx3a0_dev(struct v4l2_subdev *sd)
{
	return container_of(sd, struct arx3a0_dev, sd);
}

static inline struct v4l2_subdev *ctrl_to_sd(struct v4l2_ctrl *ctrl)
{
	return &container_of(ctrl->handler, struct arx3a0_dev,
			     ctrls.handler)->sd;
}

static u32 div64_round(u64 v, u32 d)
{
	return div_u64(v + (d >> 1), d);
}

static u32 div64_round_up(u64 v, u32 d)
{
	return div_u64(v + d - 1, d);
}

/* Data must be BE16, the first value is the register address */
static int arx3a0_read_reg(struct arx3a0_dev *sensor, u16 addr, u16* val)
{
	struct i2c_client *client = sensor->i2c_client;
	struct i2c_msg msg[2];
	int ret;
	u8 buf[2];

	buf[0] = addr >> 8;
	buf[1] = addr & 0xff;

	msg[0].addr = client->addr;
	msg[0].flags = client->flags;
	msg[0].buf = buf;
	msg[0].len = sizeof(buf);

	msg[1].addr = client->addr;
	msg[1].flags = client->flags | I2C_M_RD;
	msg[1].buf = buf;
	msg[1].len = 2;

	ret = i2c_transfer(client->adapter, msg, 2);

	if (ret < 0) {
		v4l2_err(&sensor->sd, "%s: I2C read error\n", __func__);
		return ret;
	}
	*val = (buf[0] << 8) | buf[1];

	return 0;
}


/* Data must be BE16, the first value is the register address */
static int arx3a0_write_regs(struct arx3a0_dev *sensor, const __be16 *data,
			     unsigned int count)
{
	struct i2c_client *client = sensor->i2c_client;
	struct i2c_msg msg;
	int ret;

	msg.addr = client->addr;
	msg.flags = client->flags;
	msg.buf = (u8 *)data;
	msg.len = count * sizeof(*data);
	ret = i2c_transfer(client->adapter, &msg, 1);

	if (ret < 0) {
		v4l2_err(&sensor->sd, "%s: I2C write error\n", __func__);
		return ret;
	}

	return 0;
}

static int arx3a0_write_reg(struct arx3a0_dev *sensor, u16 reg, u16 val)
{
	__be16 buf[2] = {be(reg), be(val)};

	return arx3a0_write_regs(sensor, buf, 2);
}

static int arx3a0_write_reg8(struct arx3a0_dev *sensor, u16 reg, u8 val)
{
	struct i2c_client *client = sensor->i2c_client;
	struct i2c_msg msg;
	int ret;
	char data[3];

	data[0]  = (reg >> 8) & 0xFF;
	data[1]  = reg & 0xFF;
	data[2]  = val;

	msg.addr = client->addr;
	msg.flags = client->flags;
	msg.buf = (u8 *)data;
	msg.len = 3;
	ret = i2c_transfer(client->adapter, &msg, 1);

	if (ret < 0) {
		v4l2_err(&sensor->sd, "%s: I2C write error\n", __func__);
		return ret;
	}
	return 0;
}

static int arx3a0_set_geometry(struct arx3a0_dev *sensor)
{
	/* All dimensions are unsigned 12-bit integers */
	u16 x = (ARX3A0_WIDTH_MAX - sensor->fmt.width) / 2;
	u16 y = ((ARX3A0_HEIGHT_MAX - sensor->fmt.height) / 2) & ~1;
	__be16 regs[] = {
		be(ARX3A0_REG_FRAME_LENGTH_LINES),
		be(sensor->total_height),
		be(sensor->total_width),
		be(x),
		be(y),
		be(x + sensor->fmt.width - 1),
		be(y + sensor->fmt.height - 1),
		be(sensor->fmt.width),
		be(sensor->fmt.height)
	};

	return arx3a0_write_regs(sensor, regs, ARRAY_SIZE(regs));
}

static u32 calc_pll(struct arx3a0_dev *sensor, int num, u32 freq, u16 *pre_ptr,
		    u16 *mult_ptr)
{
	u16 pre = 1, mult = 1, new_pre;
	u32 pll = ARX3A0_PLL_MAX + 1;

	for (new_pre = 1; new_pre < 64; new_pre++) {
		u32 new_pll;
		u32 new_mult = div64_round_up((u64)freq * new_pre,
					      sensor->extclk_freq);

		if (new_mult < 32)
			continue; /* Minimum value */
		if (new_mult > 254)
			break; /* Maximum, larger pre won't work either */
		if (sensor->extclk_freq * (u64)new_mult < ARX3A0_PLL_MIN *
		    new_pre)
			continue;
		if (sensor->extclk_freq * (u64)new_mult > ARX3A0_PLL_MAX *
		    new_pre)
			break; /* Larger pre won't work either */
		new_pll = div64_round_up(sensor->extclk_freq * (u64)new_mult,
					 new_pre);
		if (new_pll < pll) {
			pll = new_pll;
			pre = new_pre;
			mult = new_mult;
		}
	}

	pll = div64_round(sensor->extclk_freq * (u64)mult, pre);
	*pre_ptr = pre;
	*mult_ptr = mult;
	return pll;
}

#define DIV 4
static void arx3a0_calc_mode(struct arx3a0_dev *sensor)
{
	unsigned int speed_mod = 4 / sensor->lane_count; /* 1 with 4 DDR lanes */
	u16 total_width = max(sensor->fmt.width + ARX3A0_WIDTH_BLANKING_MIN,
			      ARX3A0_TOTAL_WIDTH_MIN);
	u16 total_height = sensor->fmt.height + ARX3A0_HEIGHT_BLANKING_MIN;

	/* Calculate approximate pixel clock first */
	u64 pix_clk = ARX3A0_PIXEL_CLOCK_RATE;

	/* PLL1 drives pixel clock - dual rate */
	pix_clk = calc_pll(sensor, 1, pix_clk * (DIV / 2), &sensor->pll_pre,
			   &sensor->pll_mult);
	pix_clk = div64_round(pix_clk, (DIV / 2));
	calc_pll(sensor, 2, pix_clk * (DIV / 2) * speed_mod, &sensor->pll_pre2,
		 &sensor->pll_mult2);

	sensor->total_width = total_width;
	sensor->total_height = total_height;
}
#if 0
static int arx3a0_write_mode(struct arx3a0_dev *sensor)
{
	__be16 pll_regs[] = {
		be(ARX3A0_REG_VT_PIX_CLK_DIV),
		/* 0x300 */ be(4), /* vt_pix_clk_div = number of bits / 2 */
		/* 0x302 */ be(1), /* vt_sys_clk_div */
		/* 0x304 */ be((sensor->pll_pre2 << 8) | sensor->pll_pre),
		/* 0x306 */ be((sensor->pll_mult2 << 8) | sensor->pll_mult),
		/* 0x308 */ be(8), /* op_pix_clk_div = 2 * vt_pix_clk_div */
		/* 0x30A */ be(1)  /* op_sys_clk_div */
	};
	int ret;

	/* Stop streaming for just a moment */
	ret = arx3a0_write_reg(sensor, ARX3A0_REG_RESET,
			       ARX3A0_REG_RESET_DEFAULTS);
	if (ret)
		return ret;

	ret = arx3a0_set_geometry(sensor);
	if (ret)
		return ret;

	ret = arx3a0_write_regs(sensor, pll_regs, ARRAY_SIZE(pll_regs));
	if (ret)
		return ret;

	ret = arx3a0_write_reg(sensor, ARX3A0_REG_COARSE_INTEGRATION_TIME,
			       sensor->ctrls.exposure->val);
	if (ret)
		return ret;

	ret = arx3a0_write_reg(sensor, ARX3A0_REG_RESET,
			       ARX3A0_REG_RESET_DEFAULTS |
			       ARX3A0_REG_RESET_STREAM);
	if (ret)
		return ret;

	ret = arx3a0_write_reg(sensor, ARX3A0_REG_TEST_PATTERN_MODE,
			       sensor->ctrls.test_pattern->val);
	return ret;
}
#endif

static int arx3a0_set_stream(struct arx3a0_dev *sensor, bool on)
{
	if (on) {
		arx3a0_write_reg8(sensor, ARX3A0_MODE_SELECT_REGISTER, 0x01);
		return 0;
	} else {
		arx3a0_write_reg8(sensor, ARX3A0_MODE_SELECT_REGISTER, 0x00);
		return 0;
	}
}

static void arx3a0_adj_fmt(struct v4l2_mbus_framefmt *fmt)
{
	fmt->width = clamp(ALIGN(fmt->width, 4), ARX3A0_WIDTH_MIN,
			   ARX3A0_WIDTH_MAX);
	fmt->height = clamp(ALIGN(fmt->height, 4), ARX3A0_HEIGHT_MIN,
			    ARX3A0_HEIGHT_MAX);
	/*
	 * Media bus Format - SBGGR10_1X10 is for color filter arrays in image
	 * sensors that capture image in Bayer pattern. In bayer pattern, each
	 * pixel captures only one color component (red, green or blue), and
	 * neighboring pixels capture different color components.
	 * The 10_1X10 part represents that each pixel captured by sensor is
	 * 10-bit wide or bit-depth of each pixel.
	 * The 1X10 suggests that the data is transmitted 10-bits in a single
	 * data stream.
	 * Media bus format - SBGGR8_1X8 is also for color filter arrays in
	 * image sensors that capture image in Bayer pattern.
	 * The 8_1X8 part signifies that each captured pixel is 8-bit wide, and
	 * is transmitted as 8-bits per data stream.
	 */
	if (fmt->code != MEDIA_BUS_FMT_SBGGR10_1X10 &&
	    fmt->code != MEDIA_BUS_FMT_SBGGR8_1X8)
		fmt->code = MEDIA_BUS_FMT_SBGGR10_1X10;
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_SRGB;
	fmt->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_XFER_FUNC_DEFAULT;
}

static int arx3a0_set_initial_regs(struct arx3a0_dev *sensor)
{
	unsigned int cnt;

	if (sensor->fmt.code == MEDIA_BUS_FMT_SBGGR10_1X10) {
		for (cnt = 0; cnt < ARRAY_SIZE(initial_regs_raw10); cnt++)
			if (arx3a0_write_regs(sensor,
					initial_regs_raw10[cnt].data,
					initial_regs_raw10[cnt].count))
				return -1;
	} else if (sensor->fmt.code == MEDIA_BUS_FMT_SBGGR8_1X8) {
		for (cnt = 0; cnt < ARRAY_SIZE(initial_regs_raw8); cnt++)
			if (arx3a0_write_regs(sensor,
					initial_regs_raw8[cnt].data,
					initial_regs_raw8[cnt].count))
				return -1;
	} else {
		return -1;
	}

	return 0;
}

static int arx3a0_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *format)
{
	struct arx3a0_dev *sensor = to_arx3a0_dev(sd);
	struct v4l2_mbus_framefmt *fmt;

	mutex_lock(&sensor->lock);

	if (format->which == V4L2_SUBDEV_FORMAT_TRY)
		fmt = v4l2_subdev_get_try_format(&sensor->sd, cfg, 0
						 /* pad */);
	else
		fmt = &sensor->fmt;

	format->format = *fmt;

	mutex_unlock(&sensor->lock);
	return 0;
}

static int arx3a0_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *format)
{
	struct arx3a0_dev *sensor = to_arx3a0_dev(sd);
	int ret = 0;

	arx3a0_adj_fmt(&format->format);

	mutex_lock(&sensor->lock);

	if (format->which == V4L2_SUBDEV_FORMAT_TRY) {
		struct v4l2_mbus_framefmt *fmt;

		fmt = v4l2_subdev_get_try_format(sd, cfg, 0 /* pad */);
		*fmt = format->format;
	} else {
		if (sensor->fmt.code != format->format.code) {
			sensor->fmt = format->format;
			ret = arx3a0_set_initial_regs(sensor);
		}
		arx3a0_calc_mode(sensor);
	}

	mutex_unlock(&sensor->lock);
	return ret;
}

static int arx3a0_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = ctrl_to_sd(ctrl);
	struct arx3a0_dev *sensor = to_arx3a0_dev(sd);
	int ret;

	/* v4l2_ctrl_lock() locks our own mutex */

	switch (ctrl->id) {
	case V4L2_CID_HBLANK:
	case V4L2_CID_VBLANK:
		sensor->total_width = sensor->fmt.width +
			sensor->ctrls.hblank->val;
		sensor->total_height = sensor->fmt.width +
			sensor->ctrls.vblank->val;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	/* access the sensor only if it's powered up */
	/*
	 * if (!pm_runtime_get_if_in_use(&sensor->i2c_client->dev))
	 *	return 0;
	 */

	switch (ctrl->id) {
	case V4L2_CID_HBLANK:
	case V4L2_CID_VBLANK:
		ret = arx3a0_set_geometry(sensor);
		break;
	case V4L2_CID_EXPOSURE:
		ret = arx3a0_write_reg(sensor,
				       ARX3A0_REG_COARSE_INTEGRATION_TIME,
				       ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = arx3a0_write_reg(sensor, ARX3A0_REG_TEST_PATTERN_MODE,
				       ctrl->val);
		break;
	}

	/* pm_runtime_put(&sensor->i2c_client->dev); */
	return ret;
}

static const struct v4l2_ctrl_ops arx3a0_ctrl_ops = {
	.s_ctrl = arx3a0_s_ctrl,
};

static const char * const test_pattern_menu[] = {
	"Disabled",
	"Solid monochrome",
	"Monochrome bars",
	"Monochrome vertical fade bars"
};

static int arx3a0_init_controls(struct arx3a0_dev *sensor)
{
	const struct v4l2_ctrl_ops *ops = &arx3a0_ctrl_ops;
	struct arx3a0_ctrls *ctrls = &sensor->ctrls;
	struct v4l2_ctrl_handler *hdl = &ctrls->handler;
	int ret;

	v4l2_ctrl_handler_init(hdl, 32);

	/* We can use our own mutex for the ctrl lock */
	hdl->lock = &sensor->lock;

	/* Manual gain */
/*	ctrls->gain = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_GAIN, 0, 511, 1, 0);
	ctrls->red_balance = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_RED_BALANCE,
					       -512, 511, 1, 0);
	ctrls->blue_balance = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_BLUE_BALANCE,
						-512, 511, 1, 0);

	v4l2_ctrl_cluster(3, &ctrls->gain);
*/
	ctrls->hblank = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_HBLANK,
					  ARX3A0_WIDTH_BLANKING_MIN, 4094, 1,
					  ARX3A0_WIDTH_BLANKING_MIN);
	ctrls->vblank = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_VBLANK,
					  ARX3A0_HEIGHT_BLANKING_MIN, 4094, 2,
					  ARX3A0_HEIGHT_BLANKING_MIN);
	v4l2_ctrl_cluster(2, &ctrls->hblank);

	/* Read-only */
	ctrls->pixrate = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_PIXEL_RATE,
					   ARX3A0_PIXEL_CLOCK_MIN,
					   ARX3A0_PIXEL_CLOCK_MAX, 1,
					   ARX3A0_PIXEL_CLOCK_RATE);

	/* Manual exposure time */
	ctrls->exposure = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_EXPOSURE, 0,
					    65535, 1, 360);

	ctrls->test_pattern = v4l2_ctrl_new_std_menu_items(hdl, ops,
					V4L2_CID_TEST_PATTERN,
					ARRAY_SIZE(test_pattern_menu) - 1,
					0, 0, test_pattern_menu);

	if (hdl->error) {
		ret = hdl->error;
		goto free_ctrls;
	}

	sensor->sd.ctrl_handler = hdl;
	return 0;

free_ctrls:
	v4l2_ctrl_handler_free(hdl);
	return ret;
}


static int arx3a0_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct arx3a0_dev *sensor = to_arx3a0_dev(sd);
	int i;

	clk_disable_unprepare(sensor->extclk);

	/* de-assert RESET signal */
	if (sensor->reset_gpio)
		gpiod_set_value(sensor->reset_gpio, 1);

	for (i = ARRAY_SIZE(arx3a0_supply_names) - 1; i >= 0; i--) {
		if (sensor->supplies[i])
			regulator_disable(sensor->supplies[i]);
	}
	return 0;
}

static int arx3a0_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct arx3a0_dev *sensor = to_arx3a0_dev(sd);
	unsigned int cnt;
	u16 val;
	int ret;

	for (cnt = 0; cnt < ARRAY_SIZE(arx3a0_supply_names); cnt++)
		if (sensor->supplies[cnt]) {
			ret = regulator_enable(sensor->supplies[cnt]);
			if (ret < 0)
				goto off;

			usleep_range(1000, 1500); /* min 1 ms */
		}

	ret = clk_prepare_enable(sensor->extclk);
	if (ret < 0) {
		v4l2_err(&sensor->sd, "error enabling sensor clock\n");
		goto off;
	}

	usleep_range(1000, 1500); /* min 1 ms */

	if (sensor->reset_gpio)
		/* assert RESET signal */
		gpiod_set_value(sensor->reset_gpio, 1);

	usleep_range(3000, 4000); /* min 45000 clocks */
	gpiod_set_value(sensor->reset_gpio, 0);

	if (sensor->power_gpio)
		gpiod_set_value(sensor->power_gpio, 1);
	usleep_range(1000, 1500); /* min 1 ms */

	if (sensor->reset_gpio)
		gpiod_set_value(sensor->reset_gpio, 0);

	mdelay(100);

	dev_info(dev, "Power-ON Initial reset and configuration!!!\n");
	arx3a0_read_reg(sensor, ARX3A0_CHIP_ID_REGISTER, &val);
	if (val != ARX3A0_CHIP_ID_REGISTER_VALUE) {
		dev_info(dev, "Unknown Chip-ID - 0x%x", val);
		goto off;
	}

	arx3a0_write_reg(sensor, ARX3A0_SOFTWARE_RESET_REGISTER, 0x01);
	mdelay(1);

	/* Putting sensor in standby mode */
	arx3a0_write_reg(sensor, ARX3A0_MODE_SELECT_REGISTER, 0x00);

	/* Putting sensor in LP-11 state on Standby mode. */
	arx3a0_read_reg(sensor, ARX3A0_MIPI_CONFIG_REGISTER, &val);
	arx3a0_write_reg(sensor, ARX3A0_MIPI_CONFIG_REGISTER,  (1 << 7) | val);

	ret = arx3a0_set_initial_regs(sensor);
	if (ret)
		goto off;

	/* Start streaming. */
	arx3a0_write_reg(sensor, ARX3A0_MODE_SELECT_REGISTER, 1);

	mdelay(50);

	/* Suspend any stream. */
	arx3a0_write_reg(sensor, ARX3A0_MODE_SELECT_REGISTER, 0);

	return 0;
off:
	arx3a0_power_off(dev);
	return ret;
}

static int arx3a0_enum_mbus_code(struct v4l2_subdev *sd,
			         struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct arx3a0_dev *sensor = to_arx3a0_dev(sd);

	if (code->index)
		return -EINVAL;

	code->code = sensor->fmt.code;
	return 0;
}

static int arx3a0_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct arx3a0_dev *sensor = to_arx3a0_dev(sd);
	int ret;

	mutex_lock(&sensor->lock);

	ret = arx3a0_set_stream(sensor, enable);
	if (!ret)
		sensor->streaming = enable;

	mutex_unlock(&sensor->lock);
	/* pm_runtime_put(&sensor->i2c_client->dev); */
	return ret;
}

static const struct v4l2_subdev_core_ops arx3a0_core_ops = {
	.log_status = v4l2_ctrl_subdev_log_status,
};

static const struct v4l2_subdev_video_ops arx3a0_video_ops = {
	.s_stream = arx3a0_s_stream,
};

static const struct v4l2_subdev_pad_ops arx3a0_pad_ops = {
	.enum_mbus_code = arx3a0_enum_mbus_code,
	.get_fmt = arx3a0_get_fmt,
	.set_fmt = arx3a0_set_fmt,
};

static const struct v4l2_subdev_ops arx3a0_subdev_ops = {
	.core = &arx3a0_core_ops,
	.video = &arx3a0_video_ops,
	.pad = &arx3a0_pad_ops,
};
/*
static int __maybe_unused arx3a0_suspend(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct arx3a0_dev *sensor = to_arx3a0_dev(sd);

	if (sensor->streaming)
		arx3a0_set_stream(sensor, 0);

	return 0;
}

static int __maybe_unused arx3a0_resume(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct arx3a0_dev *sensor = to_arx3a0_dev(sd);

	if (sensor->streaming)
		return arx3a0_set_stream(sensor, 1);

	return 0;
}
*/
static int arx3a0_probe(struct i2c_client *client)
{
	struct v4l2_fwnode_endpoint ep = {
		.bus_type = V4L2_MBUS_CSI2_DPHY
	};
	struct device *dev = &client->dev;
	struct fwnode_handle *endpoint;
	struct arx3a0_dev *sensor;
	unsigned int cnt;
	int ret;

	sensor = devm_kzalloc(dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	sensor->i2c_client = client;
	sensor->fmt.width = ARX3A0_WIDTH_MAX;
	sensor->fmt.height = ARX3A0_HEIGHT_MAX;

	endpoint = fwnode_graph_get_endpoint_by_id(dev_fwnode(dev), 0, 0,
						   FWNODE_GRAPH_ENDPOINT_NEXT);
	if (!endpoint) {
		dev_err(dev, "endpoint node not found\n");
		return -EINVAL;
	}

	ret = v4l2_fwnode_endpoint_parse(endpoint, &ep);
	fwnode_handle_put(endpoint);
	if (ret) {
		dev_err(dev, "could not parse endpoint\n");
		return ret;
	}

	if (ep.bus_type != V4L2_MBUS_CSI2_DPHY) {
		dev_err(dev, "invalid bus type, must be MIPI CSI2\n");
		return -EINVAL;
	}

	sensor->lane_count = ep.bus.mipi_csi2.num_data_lanes;
	switch (sensor->lane_count) {
	case 1:
	case 2:
		break;
	default:
		dev_err(dev, "invalid number of MIPI data lanes\n");
		return -EINVAL;
	}

	/* Get master clock (extclk) */
	sensor->extclk = devm_clk_get(dev, "extclk");
	if (IS_ERR(sensor->extclk)) {
		dev_err(dev, "failed to get extclk\n");
		return PTR_ERR(sensor->extclk);
	}

	sensor->extclk_freq = clk_get_rate(sensor->extclk);

	if (sensor->extclk_freq < ARX3A0_EXTCLK_MIN ||
	    sensor->extclk_freq > ARX3A0_EXTCLK_MAX) {
		dev_err(dev, "extclk frequency out of range: %u Hz\n",
			sensor->extclk_freq);
		return -EINVAL;
	}

	/* Request optional reset pin (usually active low) and assert it */
	sensor->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						     GPIOD_OUT_HIGH);

	/* Request optional power pin. */
	sensor->power_gpio = devm_gpiod_get_optional(dev, "power",
						GPIOD_OUT_LOW);
	v4l2_i2c_subdev_init(&sensor->sd, client, &arx3a0_subdev_ops);

	sensor->sd.flags = V4L2_SUBDEV_FL_HAS_DEVNODE;

#if defined(CONFIG_MEDIA_CONTROLLER)
	sensor->pad.flags = MEDIA_PAD_FL_SOURCE;
	sensor->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sensor->sd.entity, 1, &sensor->pad);
	if (ret)
		return ret;
#endif

	for (cnt = 0; cnt < ARRAY_SIZE(arx3a0_supply_names); cnt++) {
		struct regulator *supply = devm_regulator_get(dev,
						arx3a0_supply_names[cnt]);

		if (IS_ERR(supply)) {
			dev_info(dev, "no %s regulator found: %li\n",
				 arx3a0_supply_names[cnt], PTR_ERR(supply));
			return PTR_ERR(supply);
		}
		sensor->supplies[cnt] = supply;
	}

	mutex_init(&sensor->lock);

	ret = arx3a0_init_controls(sensor);
	if (ret)
		goto entity_cleanup;

	arx3a0_adj_fmt(&sensor->fmt);

	ret = v4l2_async_register_subdev(&sensor->sd);
	if (ret)
		goto free_ctrls;

	/* Turn on the device and enable runtime PM */
	ret = arx3a0_power_on(&client->dev);
	if (ret)
		goto disable;

	/* TODO: Enable PM */
	/*
	 * pm_runtime_set_active(&client->dev);
	 * pm_runtime_enable(&client->dev);
	 * pm_runtime_idle(&client->dev);
	 */
	return 0;

disable:
	v4l2_async_unregister_subdev(&sensor->sd);
free_ctrls:
	v4l2_ctrl_handler_free(&sensor->ctrls.handler);
entity_cleanup:
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sensor->sd.entity);
#endif
	mutex_destroy(&sensor->lock);
	return ret;
}

static int arx3a0_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct arx3a0_dev *sensor = to_arx3a0_dev(sd);

	v4l2_async_unregister_subdev(&sensor->sd);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sensor->sd.entity);
#endif
	v4l2_ctrl_handler_free(&sensor->ctrls.handler);
	/* TODO: Enable PM */
	/*
	 * pm_runtime_disable(&client->dev);
	 * if (!pm_runtime_status_suspended(&client->dev))
	 *	arx3a0_power_off(&client->dev);
	 * pm_runtime_set_suspended(&client->dev);
	 */
	mutex_destroy(&sensor->lock);
	return 0;
}

static const struct dev_pm_ops arx3a0_pm_ops = {
/*	SET_SYSTEM_SLEEP_PM_OPS(arx3a0_suspend, arx3a0_resume)*/
	SET_RUNTIME_PM_OPS(arx3a0_power_off, arx3a0_power_on, NULL)
};
static const struct of_device_id arx3a0_dt_ids[] = {
	{.compatible = "onnn,arx3a0"},
	{}
};
MODULE_DEVICE_TABLE(of, arx3a0_dt_ids);

static struct i2c_driver arx3a0_i2c_driver = {
	.driver = {
		.name  = "arx3a0",
		/* TODO: Enable PM */
		/* .pm = &arx3a0_pm_ops, */
		.of_match_table = arx3a0_dt_ids,
	},
	.probe_new = arx3a0_probe,
	.remove = arx3a0_remove,
};

module_i2c_driver(arx3a0_i2c_driver);

MODULE_DESCRIPTION("ARX3A0 MIPI Camera subdev driver");
MODULE_AUTHOR("Harith George <harith.g@alifsemi.com>");
MODULE_LICENSE("GPL v2");
