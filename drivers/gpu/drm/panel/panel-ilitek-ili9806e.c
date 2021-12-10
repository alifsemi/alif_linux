// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2021, Alif Semiconductor
 *
 * Author: Harith George <harith.g@alifsemi.com>
 * Heavily borrowed from other ilitek panel drivers
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/fb.h>
#include <linux/kernel.h>
#include <linux/module.h>

#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>

#include <video/mipi_display.h>

struct ili9806e {
	struct drm_panel	panel;
	struct mipi_dsi_device	*dsi;

	struct backlight_device *backlight;
	struct regulator	*power;
	struct gpio_desc	*reset;
};

enum ili9806e_op {
	ILI9806E_SWITCH_PAGE,
	ILI9806E_COMMAND,
};

struct ili9806e_instr {
	enum ili9806e_op	op;

	union arg {
		struct cmd {
			u8	cmd;
			u8	data;
		} cmd;
		u8	page;
	} arg;
};

#define ILI9806E_SWITCH_PAGE_INSTR(_page)	\
	{					\
		.op = ILI9806E_SWITCH_PAGE,	\
		.arg = {			\
			.page = (_page),	\
		},				\
	}

#define ILI9806E_COMMAND_INSTR(_cmd, _data)		\
	{						\
		.op = ILI9806E_COMMAND,		\
		.arg = {				\
			.cmd = {			\
				.cmd = (_cmd),		\
				.data = (_data),	\
			},				\
		},					\
	}

static const struct ili9806e_instr ili9806e_init[] = {
	ILI9806E_SWITCH_PAGE_INSTR(1),
	ILI9806E_COMMAND_INSTR(0x08, 0x10), /* Output SDA 		*/
	ILI9806E_COMMAND_INSTR(0x21, 0x01), /* DE = 1 Active		*/
	ILI9806E_COMMAND_INSTR(0x30, 0x01), /* Resolution 480 X 800	*/
	ILI9806E_COMMAND_INSTR(0x31, 0x00), /* Inversion setting	*/
	ILI9806E_COMMAND_INSTR(0x40, 0x14), /* BT 15 			*/
	ILI9806E_COMMAND_INSTR(0x41, 0x33), /* avdd +5.2v,avee-5.2v 	*/
	ILI9806E_COMMAND_INSTR(0x42, 0x02), /* VGL=DDVDH+VCIP -DDVDL,VGH=2DDVDL-VCIP */
	ILI9806E_COMMAND_INSTR(0x43, 0x09), /* SET VGH clamp level 	*/
	ILI9806E_COMMAND_INSTR(0x44, 0x06), /* SET VGL clamp level	*/
	ILI9806E_COMMAND_INSTR(0x50, 0x70), /* VREG1			*/
	ILI9806E_COMMAND_INSTR(0x51, 0x70), /* VREG2			*/
	ILI9806E_COMMAND_INSTR(0x52, 0x00), /* Flicker MSB		*/
	ILI9806E_COMMAND_INSTR(0x53, 0x48), /* Flicker LSB = VCOM	*/
	ILI9806E_COMMAND_INSTR(0x60, 0x07),
	ILI9806E_COMMAND_INSTR(0x61, 0x00),
	ILI9806E_COMMAND_INSTR(0x62, 0x08),
	ILI9806E_COMMAND_INSTR(0x63, 0x00),
	ILI9806E_COMMAND_INSTR(0xA0, 0x00),
	ILI9806E_COMMAND_INSTR(0xA1, 0x03),
	ILI9806E_COMMAND_INSTR(0xA2, 0x09),
	ILI9806E_COMMAND_INSTR(0xA3, 0x0D),
	ILI9806E_COMMAND_INSTR(0xA4, 0x06),
	ILI9806E_COMMAND_INSTR(0xA5, 0x16),
	ILI9806E_COMMAND_INSTR(0xA6, 0x09),
	ILI9806E_COMMAND_INSTR(0xA7, 0x08),
	ILI9806E_COMMAND_INSTR(0xA8, 0x03),
	ILI9806E_COMMAND_INSTR(0xA9, 0x07),
	ILI9806E_COMMAND_INSTR(0xAA, 0x06),
	ILI9806E_COMMAND_INSTR(0xAB, 0x05),
	ILI9806E_COMMAND_INSTR(0xAC, 0x0D),
	ILI9806E_COMMAND_INSTR(0xAD, 0x2C),
	ILI9806E_COMMAND_INSTR(0xAE, 0x26),
	ILI9806E_COMMAND_INSTR(0xAF, 0x00),
	ILI9806E_COMMAND_INSTR(0xC0, 0x00),
	ILI9806E_COMMAND_INSTR(0xC1, 0x04),
	ILI9806E_COMMAND_INSTR(0xC2, 0x0B),
	ILI9806E_COMMAND_INSTR(0xC3, 0x0F),
	ILI9806E_COMMAND_INSTR(0xC4, 0x09),
	ILI9806E_COMMAND_INSTR(0xC5, 0x18),
	ILI9806E_COMMAND_INSTR(0xC6, 0x07),
	ILI9806E_COMMAND_INSTR(0xC7, 0x08),
	ILI9806E_COMMAND_INSTR(0xC8, 0x05),
	ILI9806E_COMMAND_INSTR(0xC9, 0x09),
	ILI9806E_COMMAND_INSTR(0xCA, 0x07),
	ILI9806E_COMMAND_INSTR(0xCB, 0x05),
	ILI9806E_COMMAND_INSTR(0xCC, 0x0C),
	ILI9806E_COMMAND_INSTR(0xCD, 0x2D),
	ILI9806E_COMMAND_INSTR(0xCE, 0x28),
	ILI9806E_COMMAND_INSTR(0xCF, 0x00),

	/* Change to Page 6 CMD for GIP timing */
	ILI9806E_SWITCH_PAGE_INSTR(6),
	ILI9806E_COMMAND_INSTR(0x00, 0x21),
	ILI9806E_COMMAND_INSTR(0x01, 0x09),
	ILI9806E_COMMAND_INSTR(0x02, 0x00),
	ILI9806E_COMMAND_INSTR(0x03, 0x00),
	ILI9806E_COMMAND_INSTR(0x04, 0x01),
	ILI9806E_COMMAND_INSTR(0x05, 0x01),
	ILI9806E_COMMAND_INSTR(0x06, 0x80),
	ILI9806E_COMMAND_INSTR(0x07, 0x05),
	ILI9806E_COMMAND_INSTR(0x08, 0x02),
	ILI9806E_COMMAND_INSTR(0x09, 0x80),
	ILI9806E_COMMAND_INSTR(0x0A, 0x00),
	ILI9806E_COMMAND_INSTR(0x0B, 0x00),
	ILI9806E_COMMAND_INSTR(0x0C, 0x0A),
	ILI9806E_COMMAND_INSTR(0x0D, 0x0A),
	ILI9806E_COMMAND_INSTR(0x0E, 0x00),
	ILI9806E_COMMAND_INSTR(0x0F, 0x00),
	ILI9806E_COMMAND_INSTR(0x10, 0xE0),
	ILI9806E_COMMAND_INSTR(0x11, 0xE4),
	ILI9806E_COMMAND_INSTR(0x12, 0x04),
	ILI9806E_COMMAND_INSTR(0x13, 0x00),
	ILI9806E_COMMAND_INSTR(0x14, 0x00),
	ILI9806E_COMMAND_INSTR(0x15, 0xC0),
	ILI9806E_COMMAND_INSTR(0x16, 0x08),
	ILI9806E_COMMAND_INSTR(0x17, 0x00),
	ILI9806E_COMMAND_INSTR(0x18, 0x00),
	ILI9806E_COMMAND_INSTR(0x19, 0x00),
	ILI9806E_COMMAND_INSTR(0x1A, 0x00),
	ILI9806E_COMMAND_INSTR(0x1B, 0x00),
	ILI9806E_COMMAND_INSTR(0x1C, 0x00),
	ILI9806E_COMMAND_INSTR(0x1D, 0x00),
	ILI9806E_COMMAND_INSTR(0x20, 0x01),
	ILI9806E_COMMAND_INSTR(0x21, 0x23),
	ILI9806E_COMMAND_INSTR(0x22, 0x45),
	ILI9806E_COMMAND_INSTR(0x23, 0x67),
	ILI9806E_COMMAND_INSTR(0x24, 0x01),
	ILI9806E_COMMAND_INSTR(0x25, 0x23),
	ILI9806E_COMMAND_INSTR(0x26, 0x45),
	ILI9806E_COMMAND_INSTR(0x27, 0x67),
	ILI9806E_COMMAND_INSTR(0x30, 0x01),
	ILI9806E_COMMAND_INSTR(0x31, 0x11),
	ILI9806E_COMMAND_INSTR(0x32, 0x00),
	ILI9806E_COMMAND_INSTR(0x33, 0xEE),
	ILI9806E_COMMAND_INSTR(0x34, 0xFF),
	ILI9806E_COMMAND_INSTR(0x35, 0xCB),
	ILI9806E_COMMAND_INSTR(0x36, 0xDA),
	ILI9806E_COMMAND_INSTR(0x37, 0xAD),
	ILI9806E_COMMAND_INSTR(0x38, 0xBC),
	ILI9806E_COMMAND_INSTR(0x39, 0x76),
	ILI9806E_COMMAND_INSTR(0x3A, 0x67),
	ILI9806E_COMMAND_INSTR(0x3B, 0x22),
	ILI9806E_COMMAND_INSTR(0x3C, 0x22),
	ILI9806E_COMMAND_INSTR(0x3D, 0x22),
	ILI9806E_COMMAND_INSTR(0x3E, 0x22),
	ILI9806E_COMMAND_INSTR(0x3F, 0x22),
	ILI9806E_COMMAND_INSTR(0x40, 0x22),
	ILI9806E_COMMAND_INSTR(0x52, 0x10),
	ILI9806E_COMMAND_INSTR(0x53, 0x10),

	/* Change to Page 7 */
	ILI9806E_SWITCH_PAGE_INSTR(7),
	ILI9806E_COMMAND_INSTR(0x18, 0x1D),
	ILI9806E_COMMAND_INSTR(0x26, 0xB2),
	ILI9806E_COMMAND_INSTR(0x02, 0x77),
	ILI9806E_COMMAND_INSTR(0xE1, 0x79),
	ILI9806E_COMMAND_INSTR(0x17, 0x22),

	/* Change to Page 0 */
	ILI9806E_SWITCH_PAGE_INSTR(0),
	ILI9806E_COMMAND_INSTR(0x3A, 0x70), /* 24BIT	*/
};

static inline struct ili9806e *panel_to_ili9806e(struct drm_panel *panel)
{
	return container_of(panel, struct ili9806e, panel);
}

/*
 * It is organised by page, with each page having its own set of
 * registers, and the first page looks like it's holding the standard
 * DCS commands.
 *
 * So before any attempt at sending a command or data, we have to be
 * sure if we're in the right page or not.
 */
static int ili9806e_switch_page(struct ili9806e *ctx, u8 page)
{
	u8 buf[6] = { 0xff, 0xff, 0x98, 0x06, 0x04, page };
	int ret;

	ret = mipi_dsi_dcs_write_buffer(ctx->dsi, buf, sizeof(buf));
	if (ret < 0)
		return ret;

	return 0;
}

static int ili9806e_send_cmd_data(struct ili9806e *ctx, u8 cmd, u8 data)
{
	u8 buf[2] = { cmd, data };
	int ret;

	ret = mipi_dsi_dcs_write_buffer(ctx->dsi, buf, sizeof(buf));
	if (ret < 0)
		return ret;

	return 0;
}

static int ili9806e_prepare(struct drm_panel *panel)
{
	struct ili9806e *ctx = panel_to_ili9806e(panel);
	unsigned int i;
	int ret;

	mdelay(10);
	/* And reset it */
	gpiod_set_value(ctx->reset, 1);
	msleep(20);

	gpiod_set_value(ctx->reset, 0);
	msleep(20);

	for (i = 0; i < ARRAY_SIZE(ili9806e_init); i++) {
		const struct ili9806e_instr *instr = &ili9806e_init[i];

		if (instr->op == ILI9806E_SWITCH_PAGE)
			ret = ili9806e_switch_page(ctx, instr->arg.page);
		else if (instr->op == ILI9806E_COMMAND)
			ret = ili9806e_send_cmd_data(ctx, instr->arg.cmd.cmd,
						      instr->arg.cmd.data);

		if (ret)
			return ret;
	}
	return 0;
}

static int ili9806e_enable(struct drm_panel *panel)
{
	struct ili9806e *ctx = panel_to_ili9806e(panel);

	mipi_dsi_dcs_exit_sleep_mode(ctx->dsi);

	msleep(120);

	mipi_dsi_dcs_set_display_on(ctx->dsi);

	backlight_enable(ctx->backlight);

	return 0;
}

static int ili9806e_disable(struct drm_panel *panel)
{
	struct ili9806e *ctx = panel_to_ili9806e(panel);

	backlight_disable(ctx->backlight);
	return mipi_dsi_dcs_set_display_off(ctx->dsi);
}

static int ili9806e_unprepare(struct drm_panel *panel)
{
	struct ili9806e *ctx = panel_to_ili9806e(panel);

	mipi_dsi_dcs_enter_sleep_mode(ctx->dsi);
	gpiod_set_value(ctx->reset, 1);

	return 0;
}

/* This panel supports one data lane at maximum speed 850Mbps or
two data lanes at maximum speed 500Mbps */
static const struct drm_display_mode focuslcd_fw405_default_mode = {
	.clock		= 10000,
	.vrefresh	= 60,

	.hdisplay	= 480,
	.hsync_start	= 480 + 10,
	.hsync_end	= 480 + 10 + 20,
	.htotal		= 480 + 10 + 20 + 30,

	.vdisplay	= 800,
	.vsync_start	= 800 + 10,
	.vsync_end	= 800 + 10 + 10,
	.vtotal		= 800 + 10 + 10 + 20,
};

static int ili9806e_get_modes(struct drm_panel *panel)
{
	struct drm_connector *connector = panel->connector;
	struct ili9806e *ctx = panel_to_ili9806e(panel);
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(panel->drm, &focuslcd_fw405_default_mode);
	if (!mode) {
		dev_err(&ctx->dsi->dev, "failed to add mode %ux%ux@%u\n",
			focuslcd_fw405_default_mode.hdisplay,
			focuslcd_fw405_default_mode.vdisplay,
			focuslcd_fw405_default_mode.vrefresh);
		return -ENOMEM;
	}

	drm_mode_set_name(mode);

	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(connector, mode);

	panel->connector->display_info.width_mm = 62;
	panel->connector->display_info.height_mm = 110;

	return 1;
}

static const struct drm_panel_funcs ili9806e_funcs = {
	.prepare	= ili9806e_prepare,
	.unprepare	= ili9806e_unprepare,
	.enable		= ili9806e_enable,
	.disable	= ili9806e_disable,
	.get_modes	= ili9806e_get_modes,
};

static int ili9806e_dsi_probe(struct mipi_dsi_device *dsi)
{
	struct device_node *np;
	struct ili9806e *ctx;
	int ret;

	ctx = devm_kzalloc(&dsi->dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;
	mipi_dsi_set_drvdata(dsi, ctx);
	ctx->dsi = dsi;

	drm_panel_init(&ctx->panel);
	ctx->panel.dev = &dsi->dev;
	ctx->panel.funcs = &ili9806e_funcs;

	ctx->reset = devm_gpiod_get(&dsi->dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->reset)) {
		dev_err(&dsi->dev, "Couldn't get our reset GPIO\n");
		// TODO: Return if reset gpio not set
		//return PTR_ERR(ctx->reset);
	}

	np = of_parse_phandle(dsi->dev.of_node, "backlight", 0);
	if (np) {
		ctx->backlight = of_find_backlight_by_node(np);
		of_node_put(np);

		if (!ctx->backlight)
			return -EPROBE_DEFER;
	}
	ret = drm_panel_add(&ctx->panel);
	if (ret < 0)
		return ret;

	dsi->mode_flags = MIPI_DSI_MODE_VIDEO_BURST;
	dsi->format = MIPI_DSI_FMT_RGB565;
	dsi->lanes = 2;

	return mipi_dsi_attach(dsi);
}

static int ili9806e_dsi_remove(struct mipi_dsi_device *dsi)
{
	struct ili9806e *ctx = mipi_dsi_get_drvdata(dsi);

	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);

	if (ctx->backlight)
		put_device(&ctx->backlight->dev);

	return 0;
}

static const struct of_device_id ili9806e_of_match[] = {
	{ .compatible = "focuslcd,fw405" },
	{ }
};
MODULE_DEVICE_TABLE(of, ili9806e_of_match);

static struct mipi_dsi_driver ili9806e_dsi_driver = {
	.probe		= ili9806e_dsi_probe,
	.remove		= ili9806e_dsi_remove,
	.driver = {
		.name		= "ili9806e-dsi",
		.of_match_table	= ili9806e_of_match,
	},
};
module_mipi_dsi_driver(ili9806e_dsi_driver);

MODULE_AUTHOR("Harith George <harith.g@alifsemi.com>");
MODULE_DESCRIPTION("Ilitek ILI9806E Controller Driver");
MODULE_LICENSE("GPL v2");
