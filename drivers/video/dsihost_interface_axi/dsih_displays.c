// SPDX-License-Identifier: GPL-2.0
/**
 * Copyright (C) 2018 Synopsys, Inc.
 *
 * @file dsih_displays.c
 * @brief included as a part of MIPI DSI Host driver
 * included as a part of Synopsys MIPI DSI Host controller driver
 *
 * @author Luis Oliveira <luis.oliveira@synopsys.com>
 */

#include "dsih_displays.h"
#include "dsih_api.h"
#include "dsih_hal.h"
#include "dsih_dphy.h"

#ifdef DISPLAYS

void copy_dpi_param_changes(dsih_dpi_video_t * from_param,
			    dsih_dpi_video_t * to_param)
{
	to_param->no_of_lanes = from_param->no_of_lanes;
	to_param->non_continuous_clock = from_param->non_continuous_clock;
	to_param->virtual_channel = from_param->virtual_channel;
	to_param->video_mode = from_param->video_mode;
	to_param->byte_clock = from_param->byte_clock;
	to_param->pixel_clock = from_param->pixel_clock;
	to_param->color_coding = from_param->color_coding;
	to_param->is_18_loosely = from_param->is_18_loosely;
	to_param->h_polarity = from_param->h_polarity;
	to_param->h_active_pixels = from_param->h_active_pixels;
	to_param->h_sync_pixels = from_param->h_sync_pixels;
	to_param->h_back_porch_pixels = from_param->h_back_porch_pixels;
	to_param->h_total_pixels = from_param->h_total_pixels;
	to_param->v_polarity = from_param->v_polarity;
	to_param->v_active_lines = from_param->v_active_lines;
	to_param->v_sync_lines = from_param->v_sync_lines;
	to_param->v_back_porch_lines = from_param->v_back_porch_lines;
	to_param->v_total_lines = from_param->v_total_lines;
	to_param->max_hs_to_lp_cycles = from_param->max_hs_to_lp_cycles;
	to_param->max_lp_to_hs_cycles = from_param->max_lp_to_hs_cycles;
	to_param->max_clk_hs_to_lp_cycles = from_param->max_clk_hs_to_lp_cycles;
	to_param->max_clk_lp_to_hs_cycles = from_param->max_clk_lp_to_hs_cycles;
	to_param->eotp_rx_en = from_param->eotp_rx_en;
	to_param->eotp_tx_en = from_param->eotp_tx_en;
	to_param->no_of_chunks = from_param->no_of_chunks;
	to_param->null_packet_size = from_param->null_packet_size;
	to_param->dpi_lp_cmd_en = from_param->dpi_lp_cmd_en;
}

void copy_edpi_param_changes(dsih_cmd_mode_video_t * from_param,
			     dsih_cmd_mode_video_t * to_param)
{
	to_param->color_coding = from_param->color_coding;
	to_param->virtual_channel = from_param->virtual_channel;
	to_param->lp = from_param->lp;
	to_param->te = from_param->te;
	to_param->bta = from_param->bta;
	to_param->h_start = from_param->h_start;
	to_param->h_active_pixels = from_param->h_active_pixels;
	to_param->packet_size = from_param->packet_size;
	to_param->v_start = from_param->v_start;
	to_param->v_active_lines = from_param->v_active_lines;
	to_param->no_of_lanes = from_param->no_of_lanes;
	to_param->byte_clock = from_param->byte_clock;
	to_param->pixel_clock = from_param->pixel_clock;
}

/****************************************/
/*    look up tables                    */
/****************************************/
int pre_video_mode(struct mipi_dsi_dev *dev, unsigned mode, int lanes)
{
	dsih_dpi_video_t *param;

	if (dev == NULL)
		return -ENODEV;

	param = &dev->dpi_video;

	switch (mode) {

		case TREMOLO_S:
		{
				param->display_type = TREMOLO_S;
				/* 960 x 540 | BURST | PClk 100MHz | ByteClk 54MHz| 24bpp */
				param->no_of_lanes = lanes;
				param->virtual_channel = 0;
				param->color_coding = COLOR_CODE_24BIT;
				param->byte_clock = 432000 / 8;	/* KHz  */
				param->video_mode = VIDEO_BURST_WITH_SYNC_PULSES;
				param->receive_ack_packets = 0;	/* enable receiving of ack packets */
				param->pixel_clock = 85000;	/* dpi_clock */
				param->is_18_loosely = 0;
				param->data_en_polarity = 1;

				param->h_polarity = 0;
				param->h_active_pixels = 960;
				param->h_sync_pixels = 20;	/* min 4 pixels */
				param->h_back_porch_pixels = 20;
				param->h_total_pixels = 4000;
				param->v_active_lines = 540;
				param->v_polarity = 0;
				param->v_sync_lines = 2;
				param->v_back_porch_lines = 2;
				param->v_total_lines = 546;

				param->no_of_chunks = 0;
				param->null_packet_size = 0;
				param->dpi_lp_cmd_en = 0;
				param->eotp_rx_en = 0;
				param->eotp_tx_en = 0;
			return 0;
		}
		case TREMOLO_M:
		{
				param->display_type = TREMOLO_M;
				/* 800 x 480 | BURST | PClk 100MHz | ByteClk 54MHz| 24bpp */
				param->no_of_lanes = lanes;
				param->virtual_channel = 0;
				param->color_coding = COLOR_CODE_24BIT;
				param->byte_clock = 432000 / 8;	/* KHz  */
				param->video_mode = VIDEO_BURST_WITH_SYNC_PULSES;
				param->receive_ack_packets = 0;	/* enable receiving of ack packets */
				param->pixel_clock = 85000;	/* dpi_clock */
				param->is_18_loosely = 0;
				param->data_en_polarity = 1;

				param->h_polarity = 0;
				param->h_active_pixels = 800;
				param->h_sync_pixels = 20;	/* min 4 pixels */
				param->h_back_porch_pixels = 20;
				param->h_total_pixels = 4000;

				param->v_active_lines = 480;
				param->v_polarity = 0;
				param->v_sync_lines = 2;
				param->v_back_porch_lines = 2;
				param->v_total_lines = 500;

				param->no_of_chunks = 0;
				param->null_packet_size = 0;
				param->dpi_lp_cmd_en = 0;
				param->eotp_rx_en = 0;
				param->eotp_tx_en = 0;
			return 0;
		}
		case SHARP_LS045K3SX02:
		{
				param->display_type = SHARP_LS045K3SX02;
				/* 800 x 1280 | BURST | PClk 100MHz | ByteClk 56MHz| 24bpp */
				param->no_of_lanes = lanes;
				param->virtual_channel = 0;
				param->color_coding = COLOR_CODE_24BIT;
				param->byte_clock = 498000/8;	/* KHz  */
				param->video_mode = VIDEO_BURST_WITH_SYNC_PULSES;
				param->receive_ack_packets = 0;	/* enable receiving of ack packets */
				param->pixel_clock = 85000;	/* dpi_clock */
				param->is_18_loosely = 0;
				param->data_en_polarity = 1;

				param->h_polarity = 0;
				param->h_active_pixels = 800;
				param->h_sync_pixels = 2;
				param->h_back_porch_pixels = 4;
				param->h_total_pixels = 2500;

				param->v_active_lines = 1280;
				param->v_polarity = 0;
				param->v_sync_lines = 2;
				param->v_back_porch_lines = 3;
				param->v_total_lines = 1284;
				/* Intel specifig config */
				param->max_hs_to_lp_cycles = 52;
				param->max_lp_to_hs_cycles = 66;
				param->max_clk_hs_to_lp_cycles = 84;
				param->max_clk_lp_to_hs_cycles = 114;
				param->no_of_chunks = 0;
				param->null_packet_size = 0;
				param->dpi_lp_cmd_en = 0;
			return 0;
		}
		case RPI_DISPLAY:
		{
				/* 800 x 480 | NON_BURST | PClk 85MHz | ByteClk 56MHz| 24bpp */
				param->display_type = RPI_DISPLAY;
				param->no_of_lanes = lanes;
				param->virtual_channel = 0;
				param->color_coding = COLOR_CODE_24BIT;
				param->byte_clock = 664000/8;	/* KHz  */
				param->video_mode = VIDEO_NON_BURST_WITH_SYNC_PULSES;
				param->receive_ack_packets = 0;	/* enable receiving of ack packets */
				param->pixel_clock = 25000;	/* dpi_clock */
				param->is_18_loosely = 0;
				param->data_en_polarity = 1;

				param->h_polarity = 0;
				param->h_active_pixels = 800;
				param->h_sync_pixels = 50;	/* min 4 pixels */
				param->h_back_porch_pixels = 87;
				param->h_total_pixels = 3417;

				param->v_active_lines = 480;
				param->v_polarity = 0;
				param->v_sync_lines = 20;
				param->v_back_porch_lines = 7;
				param->v_total_lines = 509;

				param->eotp_rx_en = 1;
				param->eotp_tx_en = 1;
				param->no_of_chunks = 1;
				param->null_packet_size = 1;
				param->dpi_lp_cmd_en = 0;
			return 0;
		}
		case SNPS_DSI_DEVICE:
		{
				/* 800 x 480 | NON_BURST | PClk 85MHz | ByteClk 56MHz| 24bpp */
				param->display_type = SNPS_DSI_DEVICE;
				param->no_of_lanes = lanes;
				param->virtual_channel = 0;
				param->color_coding = COLOR_CODE_24BIT;
				param->byte_clock = 664000/8;	/* KHz  */
				param->video_mode = VIDEO_NON_BURST_WITH_SYNC_PULSES;
				param->receive_ack_packets = 0;	/* enable receiving of ack packets */
				param->pixel_clock = 25000;	/* dpi_clock */
				param->is_18_loosely = 0;
				param->data_en_polarity = 1;

				param->h_polarity = 0;
				param->h_active_pixels = 800;
				param->h_sync_pixels = 50;	/* min 4 pixels */
				param->h_back_porch_pixels = 87;
				param->h_total_pixels = 3417;

				param->v_active_lines = 480;
				param->v_polarity = 0;
				param->v_sync_lines = 20;
				param->v_back_porch_lines = 7;
				param->v_total_lines = 509;

				param->eotp_rx_en = 1;
				param->eotp_tx_en = 1;
				param->no_of_chunks = 1;
				param->null_packet_size = 1;
				param->dpi_lp_cmd_en = 0;
			return 0;
		}
		default:
			dev_err(dev->parent_dev, "Invalid configuration" "\n");
			break;
	}
	return 1;
}

int pre_command_mode(struct mipi_dsi_dev *dev, unsigned mode, int lanes)
{
	dsih_cmd_mode_video_t *param;

	if (dev == NULL)
		return -ENODEV;

	param = &dev->cmd_mode_video;

	switch (mode) {
	case TREMOLO_S:
			param->display_type = TREMOLO_S;
			/* 960 x 540 | Command Mode | 24bpp */
			param->no_of_lanes = lanes;
			param->pixel_clock = 85000;	/* dpi_clock */
			param->color_coding = COLOR_CODE_24BIT;
			param->virtual_channel = 0;
			param->lp = 0;
			param->te = 0;
			param->h_start = 0;
			param->h_active_pixels = 960;
			param->v_start = 0;
			param->v_active_lines = 540;
			param->byte_clock = 432000 / 8;	/* KHz  */
		return 0;
	case TREMOLO_M:
			param->display_type = TREMOLO_M;
			/* 800 x 480 | Command Mode | 24bpp */
			param->no_of_lanes = lanes;
			param->pixel_clock = 85000;	/* dpi_clock */
			param->virtual_channel = 0;
			param->color_coding = COLOR_CODE_24BIT;
			param->h_active_pixels = 800;
			param->v_active_lines = 480;
			param->lp = 0;
			param->te = 0;
			param->h_start = 0;
			param->v_start = 0;
			param->byte_clock = 432000 / 8;	/* KHz  */
		return 0;
	case RPI_DISPLAY:
			param->display_type = RPI_DISPLAY;
			param->byte_clock = 664000 / 8;	/* KHz  */
			param->no_of_lanes = lanes;
			param->pixel_clock = 25000;	/* dpi_clock */
			param->virtual_channel = 0;
			param->color_coding = COLOR_CODE_24BIT;
			param->h_active_pixels = 800;
			param->v_active_lines = 480;
			param->lp = 0;
			param->te = 0;
			param->h_start = 0;
			param->v_start = 0;
		return 0;
	case SNPS_DSI_DEVICE:
			param->display_type = SNPS_DSI_DEVICE;
			param->byte_clock = 664000 / 8;	/* KHz  */
			param->no_of_lanes = lanes;
			param->pixel_clock = 25000;	/* dpi_clock */
			param->virtual_channel = 0;
			param->color_coding = COLOR_CODE_24BIT;
			param->h_active_pixels = 800;
			param->v_active_lines = 480;
			param->lp = 0;
			param->te = 0;
			param->bta = 1;
			param->h_start = 0;
			param->v_start = 0;
		return 0;
	default:
		dev_err(dev->parent_dev, "Invalid configuration\n");
		break;
	}
	return 1;
}

int dsi_screen_init(struct mipi_dsi_dev *dev, int screen, int video_mode)
{
	uint8_t dsi_command_param[9] = { 0 };

	switch (screen)	{
		case TREMOLO_S:
		{
			pr_info("%s:Tremolo S\n", FUNC_NAME);
			/* wakeup renesas tremolo S DSI display */
			/* send commands in lp - thats why HS clock is not called */
			mipi_dsih_cmd_mode(dev, 1);

			dsi_command_param[0] = 0x11;
			dsi_command_param[1] = 0x0;
			mipi_dsih_gen_wr_packet(dev, 0, 0x5, dsi_command_param, 2);

			dsi_command_param[0] = 0xB2;
			dsi_command_param[1] = 0x1;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x3;
			dsi_command_param[1] = 0x0;
			dsi_command_param[2] = 0xB3;
			dsi_command_param[3] = 0x21;
			dsi_command_param[4] = 0x8;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 5);

			dsi_command_param[0] = 0x3;
			dsi_command_param[1] = 0x0;
			dsi_command_param[2] = 0xB3;
			dsi_command_param[3] = 0x10;
			dsi_command_param[4] = 0x0;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 5);

			dsi_command_param[0] = 0x7;
			dsi_command_param[1] = 0x0;
			dsi_command_param[2] = 0x92;
			dsi_command_param[3] = 0x6;
			dsi_command_param[4] = 0x1;
			dsi_command_param[5] = 0x4;
			dsi_command_param[6] = 0x0;
			dsi_command_param[7] = 0x3;
			dsi_command_param[8] = 0xC0;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 9);

			dsi_command_param[0] = 0x7;
			dsi_command_param[1] = 0x0;
			dsi_command_param[2] = 0x8B;
			dsi_command_param[3] = 0x3;
			dsi_command_param[4] = 0x2;
			dsi_command_param[5] = 0x6;
			dsi_command_param[6] = 0x0;
			dsi_command_param[7] = 0x2;
			dsi_command_param[8] = 0x1C;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 9);

			dsi_command_param[0] = 0xA0;
			dsi_command_param[1] = 0x1E;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0xA2;
			dsi_command_param[1] = 0x8B;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0xA4;
			dsi_command_param[1] = 0x0;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0xA6;
			dsi_command_param[1] = 0x8B;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0xA8;
			dsi_command_param[1] = 0x0;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0xAA;
			dsi_command_param[1] = 0x1;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x48;
			dsi_command_param[1] = 0x3;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x49;
			dsi_command_param[1] = 0x0;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1C;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x4B;
			dsi_command_param[1] = 0x0;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x4E;
			dsi_command_param[1] = 0x1;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x3A;
			dsi_command_param[1] = 0x7;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x36;
			dsi_command_param[1] = 0x0;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x82;
			dsi_command_param[1] = 0x40;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x7;
			dsi_command_param[1] = 0x0;
			dsi_command_param[2] = 0x2A;
			dsi_command_param[3] = 0x0;
			dsi_command_param[4] = 0x0;
			dsi_command_param[5] = 0x0;
			dsi_command_param[6] = 0x0;
			dsi_command_param[7] = 0x3;
			dsi_command_param[8] = 0xBF;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 9);

			dsi_command_param[0] = 0x7;
			dsi_command_param[1] = 0x0;
			dsi_command_param[2] = 0x2B;
			dsi_command_param[3] = 0x0;
			dsi_command_param[4] = 0x0;
			dsi_command_param[5] = 0x0;
			dsi_command_param[6] = 0x0;
			dsi_command_param[7] = 0x2;
			dsi_command_param[8] = 0x1B;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 9);

			dsi_command_param[0] = 0x4;
			dsi_command_param[1] = 0x0;
			dsi_command_param[2] = 0x4C;
			dsi_command_param[3] = 0x3;
			dsi_command_param[4] = 0x1;
			dsi_command_param[5] = 0x1;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x4;
			dsi_command_param[1] = 0x0;
			dsi_command_param[2] = 0x4C;
			dsi_command_param[3] = 0x0;
			dsi_command_param[4] = 0x1;
			dsi_command_param[5] = 0x0;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x4;
			dsi_command_param[1] = 0x0;
			dsi_command_param[2] = 0x4C;
			dsi_command_param[3] = 0x1;
			dsi_command_param[4] = 0x1;
			dsi_command_param[5] = 0x1;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x4;
			dsi_command_param[1] = 0x0;
			dsi_command_param[2] = 0x4C;
			dsi_command_param[3] = 0x4;
			dsi_command_param[4] = 0x1;
			dsi_command_param[5] = 0x0;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x4;
			dsi_command_param[1] = 0x0;
			dsi_command_param[2] = 0x4C;
			dsi_command_param[3] = 0x5;
			dsi_command_param[4] = 0x1;
			dsi_command_param[5] = 0x14;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x4;
			dsi_command_param[1] = 0x0;
			dsi_command_param[2] = 0x4C;
			dsi_command_param[3] = 0x6;
			dsi_command_param[4] = 0x1;
			dsi_command_param[5] = 0x24;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x4;
			dsi_command_param[1] = 0x0;
			dsi_command_param[2] = 0x4C;
			dsi_command_param[3] = 0x10;
			dsi_command_param[4] = 0x1;
			dsi_command_param[5] = 0xD7;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x4;
			dsi_command_param[1] = 0x0;
			dsi_command_param[2] = 0x4C;
			dsi_command_param[3] = 0x11;
			dsi_command_param[4] = 0x1;
			dsi_command_param[5] = 0x2;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x4;
			dsi_command_param[1] = 0x0;
			dsi_command_param[2] = 0x4C;
			dsi_command_param[3] = 0x12;
			dsi_command_param[4] = 0x1;
			dsi_command_param[5] = 0x0;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x4;
			dsi_command_param[1] = 0x0;
			dsi_command_param[2] = 0x4C;
			dsi_command_param[3] = 0x13;
			dsi_command_param[4] = 0x1;
			dsi_command_param[5] = 0x55;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x4;
			dsi_command_param[1] = 0x0;
			dsi_command_param[2] = 0x4C;
			dsi_command_param[3] = 0x14;
			dsi_command_param[4] = 0x1;
			dsi_command_param[5] = 0x1;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x4;
			dsi_command_param[1] = 0x0;
			dsi_command_param[2] = 0x4C;
			dsi_command_param[3] = 0x15;
			dsi_command_param[4] = 0x1;
			dsi_command_param[5] = 0x70;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x4;
			dsi_command_param[1] = 0x0;
			dsi_command_param[2] = 0x4C;
			dsi_command_param[3] = 0x16;
			dsi_command_param[4] = 0x1;
			dsi_command_param[5] = 0x1F;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x4;
			dsi_command_param[1] = 0x0;
			dsi_command_param[2] = 0x4C;
			dsi_command_param[3] = 0x17;
			dsi_command_param[4] = 0x1;
			dsi_command_param[5] = 0x0F;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x4;
			dsi_command_param[1] = 0x0;
			dsi_command_param[2] = 0x4C;
			dsi_command_param[3] = 0x18;
			dsi_command_param[4] = 0x1;
			dsi_command_param[5] = 0x0F;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x4;
			dsi_command_param[1] = 0x0;
			dsi_command_param[2] = 0x4C;
			dsi_command_param[3] = 0x19;
			dsi_command_param[4] = 0x1;
			dsi_command_param[5] = 0x2;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x4;
			dsi_command_param[1] = 0x0;
			dsi_command_param[2] = 0x4C;
			dsi_command_param[3] = 0x1A;
			dsi_command_param[4] = 0x1;
			dsi_command_param[5] = 0x2;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x4;
			dsi_command_param[1] = 0x0;
			dsi_command_param[2] = 0x4C;
			dsi_command_param[3] = 0x1B;
			dsi_command_param[4] = 0x1;
			dsi_command_param[5] = 0xA0;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x4;
			dsi_command_param[1] = 0x0;
			dsi_command_param[2] = 0x4C;
			dsi_command_param[3] = 0x20;
			dsi_command_param[4] = 0x1;
			dsi_command_param[5] = 0x20;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x4;
			dsi_command_param[1] = 0x0;
			dsi_command_param[2] = 0x4C;
			dsi_command_param[3] = 0x21;
			dsi_command_param[4] = 0x1;
			dsi_command_param[5] = 0x5;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x4;
			dsi_command_param[1] = 0x0;
			dsi_command_param[2] = 0x4C;
			dsi_command_param[3] = 0x22;
			dsi_command_param[4] = 0x1;
			dsi_command_param[5] = 0x10;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x4;
			dsi_command_param[1] = 0x0;
			dsi_command_param[2] = 0x4C;
			dsi_command_param[3] = 0x23;
			dsi_command_param[4] = 0x1;
			dsi_command_param[5] = 0x12;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x4;
			dsi_command_param[1] = 0x0;
			dsi_command_param[2] = 0x4C;
			dsi_command_param[3] = 0x24;
			dsi_command_param[4] = 0x1;
			dsi_command_param[5] = 0x12;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x4;
			dsi_command_param[1] = 0x0;
			dsi_command_param[2] = 0x4C;
			dsi_command_param[3] = 0x25;
			dsi_command_param[4] = 0x1;
			dsi_command_param[5] = 0x0D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x4;
			dsi_command_param[1] = 0x0;
			dsi_command_param[2] = 0x4C;
			dsi_command_param[3] = 0x26;
			dsi_command_param[4] = 0x1;
			dsi_command_param[5] = 0x0B;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x4;
			dsi_command_param[1] = 0x0;
			dsi_command_param[2] = 0x4C;
			dsi_command_param[3] = 0x27;
			dsi_command_param[4] = 0x1;
			dsi_command_param[5] = 0x10;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x4;
			dsi_command_param[1] = 0x0;
			dsi_command_param[2] = 0x4C;
			dsi_command_param[3] = 0x28;
			dsi_command_param[4] = 0x1;
			dsi_command_param[5] = 0x2;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x4;
			dsi_command_param[1] = 0x0;
			dsi_command_param[2] = 0x4C;
			dsi_command_param[3] = 0x29;
			dsi_command_param[4] = 0x1;
			dsi_command_param[5] = 0x2;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x4;
			dsi_command_param[1] = 0x0;
			dsi_command_param[2] = 0x4C;
			dsi_command_param[3] = 0x2A;
			dsi_command_param[4] = 0x1;
			dsi_command_param[5] = 0x2;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x4;
			dsi_command_param[1] = 0x0;
			dsi_command_param[2] = 0x4C;
			dsi_command_param[3] = 0x2B;
			dsi_command_param[4] = 0x1;
			dsi_command_param[5] = 0x0;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x4;
			dsi_command_param[1] = 0x0;
			dsi_command_param[2] = 0x4C;
			dsi_command_param[3] = 0x2C;
			dsi_command_param[4] = 0x1;
			dsi_command_param[5] = 0x5;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x4;
			dsi_command_param[1] = 0x0;
			dsi_command_param[2] = 0x4C;
			dsi_command_param[3] = 0x2D;
			dsi_command_param[4] = 0x1;
			dsi_command_param[5] = 0x10;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x4;
			dsi_command_param[1] = 0x0;
			dsi_command_param[2] = 0x4C;
			dsi_command_param[3] = 0x2E;
			dsi_command_param[4] = 0x1;
			dsi_command_param[5] = 0x12;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x4;
			dsi_command_param[1] = 0x0;
			dsi_command_param[2] = 0x4C;
			dsi_command_param[3] = 0x2F;
			dsi_command_param[4] = 0x1;
			dsi_command_param[5] = 0x12;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x4;
			dsi_command_param[1] = 0x0;
			dsi_command_param[2] = 0x4C;
			dsi_command_param[3] = 0x30;
			dsi_command_param[4] = 0x1;
			dsi_command_param[5] = 0x0D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x4;
			dsi_command_param[1] = 0x0;
			dsi_command_param[2] = 0x4C;
			dsi_command_param[3] = 0x31;
			dsi_command_param[4] = 0x1;
			dsi_command_param[5] = 0x0B;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x4;
			dsi_command_param[1] = 0x0;
			dsi_command_param[2] = 0x4C;
			dsi_command_param[3] = 0x32;
			dsi_command_param[4] = 0x1;
			dsi_command_param[5] = 0x10;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x4;
			dsi_command_param[1] = 0x0;
			dsi_command_param[2] = 0x4C;
			dsi_command_param[3] = 0x33;
			dsi_command_param[4] = 0x1;
			dsi_command_param[5] = 0x2;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x4;
			dsi_command_param[1] = 0x0;
			dsi_command_param[2] = 0x4C;
			dsi_command_param[3] = 0x34;
			dsi_command_param[4] = 0x1;
			dsi_command_param[5] = 0x2;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x4;
			dsi_command_param[1] = 0x0;
			dsi_command_param[2] = 0x4C;
			dsi_command_param[3] = 0x35;
			dsi_command_param[4] = 0x1;
			dsi_command_param[5] = 0x2;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x4;
			dsi_command_param[1] = 0x0;
			dsi_command_param[2] = 0x4C;
			dsi_command_param[3] = 0x50;
			dsi_command_param[4] = 0x1;
			dsi_command_param[5] = 0x0A;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x4;
			dsi_command_param[1] = 0x0;
			dsi_command_param[2] = 0x4C;
			dsi_command_param[3] = 0x53;
			dsi_command_param[4] = 0x1;
			dsi_command_param[5] = 0x40;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x4;
			dsi_command_param[1] = 0x0;
			dsi_command_param[2] = 0x4C;
			dsi_command_param[3] = 0x54;
			dsi_command_param[4] = 0x1;
			dsi_command_param[5] = 0x42;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x4;
			dsi_command_param[1] = 0x0;
			dsi_command_param[2] = 0x4C;
			dsi_command_param[3] = 0x55;
			dsi_command_param[4] = 0x1;
			dsi_command_param[5] = 0x41;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x4;
			dsi_command_param[1] = 0x0;
			dsi_command_param[2] = 0x4C;
			dsi_command_param[3] = 0x56;
			dsi_command_param[4] = 0x1;
			dsi_command_param[5] = 0x2C;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x4;
			dsi_command_param[1] = 0x0;
			dsi_command_param[2] = 0x4C;
			dsi_command_param[3] = 0x59;
			dsi_command_param[4] = 0x1;
			dsi_command_param[5] = 0x8A;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x4;
			dsi_command_param[1] = 0x0;
			dsi_command_param[2] = 0x4C;
			dsi_command_param[3] = 0x5A;
			dsi_command_param[4] = 0x1;
			dsi_command_param[5] = 0x1;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x4;
			dsi_command_param[1] = 0x0;
			dsi_command_param[2] = 0x4C;
			dsi_command_param[3] = 0x5B;
			dsi_command_param[4] = 0x1;
			dsi_command_param[5] = 0x0;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x4;
			dsi_command_param[1] = 0x0;
			dsi_command_param[2] = 0x4C;
			dsi_command_param[3] = 0x5C;
			dsi_command_param[4] = 0x1;
			dsi_command_param[5] = 0x1E;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x4;
			dsi_command_param[1] = 0x0;
			dsi_command_param[2] = 0x4C;
			dsi_command_param[3] = 0x5D;
			dsi_command_param[4] = 0x1;
			dsi_command_param[5] = 0x0A;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x4;
			dsi_command_param[1] = 0x0;
			dsi_command_param[2] = 0x4C;
			dsi_command_param[3] = 0x5E;
			dsi_command_param[4] = 0x1;
			dsi_command_param[5] = 0x32;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x4;
			dsi_command_param[1] = 0x0;
			dsi_command_param[2] = 0x4C;
			dsi_command_param[3] = 0x5F;
			dsi_command_param[4] = 0x1;
			dsi_command_param[5] = 0x30;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x4;
			dsi_command_param[1] = 0x0;
			dsi_command_param[2] = 0x4C;
			dsi_command_param[3] = 0x62;
			dsi_command_param[4] = 0x1;
			dsi_command_param[5] = 0x70;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x4;
			dsi_command_param[1] = 0x0;
			dsi_command_param[2] = 0x4C;
			dsi_command_param[3] = 0x63;
			dsi_command_param[4] = 0x1;
			dsi_command_param[5] = 0x30;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x4;
			dsi_command_param[1] = 0x0;
			dsi_command_param[2] = 0x4C;
			dsi_command_param[3] = 0x66;
			dsi_command_param[4] = 0x1;
			dsi_command_param[5] = 0xAE;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x4;
			dsi_command_param[1] = 0x0;
			dsi_command_param[2] = 0x4C;
			dsi_command_param[3] = 0x67;
			dsi_command_param[4] = 0x1;
			dsi_command_param[5] = 0x30;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x4;
			dsi_command_param[1] = 0x0;
			dsi_command_param[2] = 0x4C;
			dsi_command_param[3] = 0x70;
			dsi_command_param[4] = 0x1;
			dsi_command_param[5] = 0x4;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x4;
			dsi_command_param[1] = 0x0;
			dsi_command_param[2] = 0x4C;
			dsi_command_param[3] = 0x71;
			dsi_command_param[4] = 0x1;
			dsi_command_param[5] = 0x4;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x4;
			dsi_command_param[1] = 0x0;
			dsi_command_param[2] = 0x4C;
			dsi_command_param[3] = 0x72;
			dsi_command_param[4] = 0x1;
			dsi_command_param[5] = 0x2;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x4;
			dsi_command_param[1] = 0x0;
			dsi_command_param[2] = 0x4C;
			dsi_command_param[3] = 0x73;
			dsi_command_param[4] = 0x1;
			dsi_command_param[5] = 0x18;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x4;
			dsi_command_param[1] = 0x0;
			dsi_command_param[2] = 0x4C;
			dsi_command_param[3] = 0x76;
			dsi_command_param[4] = 0x1;
			dsi_command_param[5] = 0x0;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x4;
			dsi_command_param[1] = 0x0;
			dsi_command_param[2] = 0x4C;
			dsi_command_param[3] = 0x79;
			dsi_command_param[4] = 0x1;
			dsi_command_param[5] = 0x30;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x4;
			dsi_command_param[1] = 0x0;
			dsi_command_param[2] = 0x4C;
			dsi_command_param[3] = 0x82;
			dsi_command_param[4] = 0x1;
			dsi_command_param[5] = 0x1;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x4;
			dsi_command_param[1] = 0x0;
			dsi_command_param[2] = 0x4C;
			dsi_command_param[3] = 0x83;
			dsi_command_param[4] = 0x1;
			dsi_command_param[5] = 0x0;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x4;
			dsi_command_param[1] = 0x0;
			dsi_command_param[2] = 0x4C;
			dsi_command_param[3] = 0x84;
			dsi_command_param[4] = 0x1;
			dsi_command_param[5] = 0xFC;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x4;
			dsi_command_param[1] = 0x0;
			dsi_command_param[2] = 0x4C;
			dsi_command_param[3] = 0x86;
			dsi_command_param[4] = 0x1;
			dsi_command_param[5] = 0x10;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x4;
			dsi_command_param[1] = 0x0;
			dsi_command_param[2] = 0x4C;
			dsi_command_param[3] = 0x88;
			dsi_command_param[4] = 0x1;
			dsi_command_param[5] = 0x10;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x4;
			dsi_command_param[1] = 0x0;
			dsi_command_param[2] = 0x4C;
			dsi_command_param[3] = 0x8A;
			dsi_command_param[4] = 0x1;
			dsi_command_param[5] = 0x4;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x4;
			dsi_command_param[1] = 0x0;
			dsi_command_param[2] = 0x4C;
			dsi_command_param[3] = 0x8B;
			dsi_command_param[4] = 0x1;
			dsi_command_param[5] = 0x10;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x4;
			dsi_command_param[1] = 0x0;
			dsi_command_param[2] = 0x4C;
			dsi_command_param[3] = 0x8C;
			dsi_command_param[4] = 0x1;
			dsi_command_param[5] = 0x0;;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x4;
			dsi_command_param[1] = 0x0;
			dsi_command_param[2] = 0x4C;
			dsi_command_param[3] = 0x8D;
			dsi_command_param[4] = 0x1;
			dsi_command_param[5] = 0xFC;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x4;
			dsi_command_param[1] = 0x0;
			dsi_command_param[2] = 0x4C;
			dsi_command_param[3] = 0x8F;
			dsi_command_param[4] = 0x1;
			dsi_command_param[5] = 0x0;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x4;
			dsi_command_param[1] = 0x0;
			dsi_command_param[2] = 0x4C;
			dsi_command_param[3] = 0x91;
			dsi_command_param[4] = 0x1;
			dsi_command_param[5] = 0x0;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x4;
			dsi_command_param[1] = 0x0;
			dsi_command_param[2] = 0x4C;
			dsi_command_param[3] = 0x93;
			dsi_command_param[4] = 0x1;
			dsi_command_param[5] = 0x0;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x4;
			dsi_command_param[1] = 0x0;
			dsi_command_param[2] = 0x4C;
			dsi_command_param[3] = 0x94;
			dsi_command_param[4] = 0x1;
			dsi_command_param[5] = 0x0;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x4;
			dsi_command_param[1] = 0x0;
			dsi_command_param[2] = 0x4C;
			dsi_command_param[3] = 0x95;
			dsi_command_param[4] = 0x1;
			dsi_command_param[5] = 0x0;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x4;
			dsi_command_param[1] = 0x0;
			dsi_command_param[2] = 0x4C;
			dsi_command_param[3] = 0x96;
			dsi_command_param[4] = 0x1;
			dsi_command_param[5] = 0xFC;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x4;
			dsi_command_param[1] = 0x0;
			dsi_command_param[2] = 0x4C;
			dsi_command_param[3] = 0x98;
			dsi_command_param[4] = 0x1;
			dsi_command_param[5] = 0x0;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x4;
			dsi_command_param[1] = 0x0;
			dsi_command_param[2] = 0x4C;
			dsi_command_param[3] = 0x9A;
			dsi_command_param[4] = 0x1;
			dsi_command_param[5] = 0x0;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x4;
			dsi_command_param[1] = 0x0;
			dsi_command_param[2] = 0x4C;
			dsi_command_param[3] = 0x9C;
			dsi_command_param[4] = 0x1;
			dsi_command_param[5] = 0x0;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x4;
			dsi_command_param[1] = 0x0;
			dsi_command_param[2] = 0x4C;
			dsi_command_param[3] = 0x9D;
			dsi_command_param[4] = 0x1;
			dsi_command_param[5] = 0x0;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x81;
			dsi_command_param[1] = 0x0;
			mipi_dsih_gen_wr_packet(dev, 0, 0x5, dsi_command_param, 2);

			dsi_command_param[0] = 0x4;
			dsi_command_param[1] = 0x0;
			dsi_command_param[2] = 0x4C;
			dsi_command_param[3] = 0x2;
			dsi_command_param[4] = 0x1;
			dsi_command_param[5] = 0x0;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			if (video_mode == COMMAND_MODE) {
				dsi_command_param[0] = 0x40;
				dsi_command_param[1] = 0x01;
				mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param,
							2);
			} else if (video_mode == VIDEO_MODE) {
				dsi_command_param[0] = 0x40;
				dsi_command_param[1] = 0x0;
				mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param,
							2);
			}
		return 0;
		}
		case TREMOLO_M:
		{
			pr_info("%s:Tremolo M\n", FUNC_NAME);
			/* wakeup renesas tremolo M DSI display */
			mipi_dsih_cmd_mode(dev, 1);
			/* sleepout */
			dsi_command_param[0] = 0x11;
			mipi_dsih_dcs_wr_cmd(dev, 0, dsi_command_param, 1);
			/* 7.0 inch wvga panel timming */
			dsi_command_param[0] = 0x07;
			dsi_command_param[1] = 0x00;
			dsi_command_param[2] = 0x92;
			dsi_command_param[3] = 0x25;
			dsi_command_param[4] = 0x08;
			dsi_command_param[5] = 0xa3;
			dsi_command_param[6] = 0x00;
			dsi_command_param[7] = 0x03;
			dsi_command_param[8] = 0x20;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 9);

			dsi_command_param[0] = 0x07;
			dsi_command_param[1] = 0x00;
			dsi_command_param[2] = 0x8b;
			dsi_command_param[3] = 0x0a;
			dsi_command_param[4] = 0x02;
			dsi_command_param[5] = 0x04;
			dsi_command_param[6] = 0x00;
			dsi_command_param[7] = 0x01;
			dsi_command_param[8] = 0xe0;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 9);

			dsi_command_param[0] = 0xa0;
			dsi_command_param[1] = 0x0c;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0xa2;
			dsi_command_param[1] = 0x41;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0xa4;
			dsi_command_param[1] = 0x00;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0xa6;
			dsi_command_param[1] = 0x41;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0xa8;
			dsi_command_param[1] = 0x00;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0xaa;
			dsi_command_param[1] = 0x00;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			/* tremolo-m dbi-type_c setting */
			dsi_command_param[0] = 0x48;
			dsi_command_param[1] = 0x03;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x49;
			dsi_command_param[1] = 0x00;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x4a;
			dsi_command_param[1] = 0x1c;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x4b;
			dsi_command_param[1] = 0x00;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x4e;
			dsi_command_param[1] = 0x01;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			/* tremolo-m setting */
			dsi_command_param[0] = 0x3a;
			dsi_command_param[1] = 0x07;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

	/* remove DSI image inversion */
			dsi_command_param[0] = 0x36;
			dsi_command_param[1] = 0x40;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x82;
			dsi_command_param[1] = 0x40;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x07;
			dsi_command_param[1] = 0x00;
			dsi_command_param[2] = 0x2a;
			dsi_command_param[3] = 0x00;
			dsi_command_param[4] = 0x00;
			dsi_command_param[5] = 0x00;
			dsi_command_param[6] = 0x00;
			dsi_command_param[7] = 0x03;
			dsi_command_param[8] = 0x1f;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 9);

			dsi_command_param[0] = 0x07;
			dsi_command_param[1] = 0x00;
			dsi_command_param[2] = 0x2b;
			dsi_command_param[3] = 0x00;
			dsi_command_param[4] = 0x00;
			dsi_command_param[5] = 0x00;
			dsi_command_param[6] = 0x00;
			dsi_command_param[7] = 0x01;
			dsi_command_param[8] = 0xdf;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 9);

			/* tremolo-m agcps setting */
			dsi_command_param[0] = 0x56;
			dsi_command_param[1] = 0x48;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x6c;
			dsi_command_param[1] = 0x96;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			/* wvga lcd power-up sequence */
			dsi_command_param[0] = 0x04;
			dsi_command_param[1] = 0x00;
			dsi_command_param[2] = 0x4c;
			dsi_command_param[3] = 0x03;
			dsi_command_param[4] = 0x01;
			dsi_command_param[5] = 0x01;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4a;
			dsi_command_param[1] = 0x1d;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x04;
			dsi_command_param[1] = 0x00;
			dsi_command_param[2] = 0x4c;
			dsi_command_param[3] = 0x00;
			dsi_command_param[4] = 0x01;
			dsi_command_param[5] = 0x00;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4a;
			dsi_command_param[1] = 0x1d;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x04;
			dsi_command_param[1] = 0x00;
			dsi_command_param[2] = 0x4c;
			dsi_command_param[3] = 0x01;
			dsi_command_param[4] = 0x01;
			dsi_command_param[5] = 0x01;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x04;
			dsi_command_param[1] = 0x00;
			dsi_command_param[2] = 0x4c;
			dsi_command_param[3] = 0x04;
			dsi_command_param[4] = 0x01;
			dsi_command_param[5] = 0x00;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x04;
			dsi_command_param[1] = 0x00;
			dsi_command_param[2] = 0x4c;
			dsi_command_param[3] = 0x05;
			dsi_command_param[4] = 0x01;
			dsi_command_param[5] = 0x14;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x04;
			dsi_command_param[1] = 0x00;
			dsi_command_param[2] = 0x4c;
			dsi_command_param[3] = 0x06;
			dsi_command_param[4] = 0x01;
			dsi_command_param[5] = 0x24;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x04;
			dsi_command_param[1] = 0x00;
			dsi_command_param[2] = 0x4c;
			dsi_command_param[3] = 0x10;
			dsi_command_param[4] = 0x01;
			dsi_command_param[5] = 0xD7;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x04;
			dsi_command_param[1] = 0x00;
			dsi_command_param[2] = 0x4c;
			dsi_command_param[3] = 0x11;
			dsi_command_param[4] = 0x01;
			dsi_command_param[5] = 0x00;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x04;
			dsi_command_param[1] = 0x00;
			dsi_command_param[2] = 0x4c;
			dsi_command_param[3] = 0x12;
			dsi_command_param[4] = 0x01;
			dsi_command_param[5] = 0x00;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x04;
			dsi_command_param[1] = 0x00;
			dsi_command_param[2] = 0x4c;
			dsi_command_param[3] = 0x13;
			dsi_command_param[4] = 0x01;
			dsi_command_param[5] = 0x55;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x04;
			dsi_command_param[1] = 0x00;
			dsi_command_param[2] = 0x4c;
			dsi_command_param[3] = 0x14;
			dsi_command_param[4] = 0x01;
			dsi_command_param[5] = 0x01;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x04;
			dsi_command_param[1] = 0x00;
			dsi_command_param[2] = 0x4c;
			dsi_command_param[3] = 0x15;
			dsi_command_param[4] = 0x01;
			dsi_command_param[5] = 0x70;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x04;
			dsi_command_param[1] = 0x00;
			dsi_command_param[2] = 0x4c;
			dsi_command_param[3] = 0x16;
			dsi_command_param[4] = 0x01;
			dsi_command_param[5] = 0x1E;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x04;
			dsi_command_param[1] = 0x00;
			dsi_command_param[2] = 0x4c;
			dsi_command_param[3] = 0x17;
			dsi_command_param[4] = 0x01;
			dsi_command_param[5] = 0x25;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x04;
			dsi_command_param[1] = 0x00;
			dsi_command_param[2] = 0x4c;
			dsi_command_param[3] = 0x18;
			dsi_command_param[4] = 0x01;
			dsi_command_param[5] = 0x25;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x04;
			dsi_command_param[1] = 0x00;
			dsi_command_param[2] = 0x4c;
			dsi_command_param[3] = 0x19;
			dsi_command_param[4] = 0x01;
			dsi_command_param[5] = 0x02;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x04;
			dsi_command_param[1] = 0x00;
			dsi_command_param[2] = 0x4c;
			dsi_command_param[3] = 0x1A;
			dsi_command_param[4] = 0x01;
			dsi_command_param[5] = 0x02;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x04;
			dsi_command_param[1] = 0x00;
			dsi_command_param[2] = 0x4c;
			dsi_command_param[3] = 0x1B;
			dsi_command_param[4] = 0x01;
			dsi_command_param[5] = 0xA0;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x04;
			dsi_command_param[1] = 0x00;
			dsi_command_param[2] = 0x4c;
			dsi_command_param[3] = 0x20;
			dsi_command_param[4] = 0x01;
			dsi_command_param[5] = 0x2F;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x04;
			dsi_command_param[1] = 0x00;
			dsi_command_param[2] = 0x4c;
			dsi_command_param[3] = 0x21;
			dsi_command_param[4] = 0x01;
			dsi_command_param[5] = 0x0F;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x04;
			dsi_command_param[1] = 0x00;
			dsi_command_param[2] = 0x4c;
			dsi_command_param[3] = 0x22;
			dsi_command_param[4] = 0x01;
			dsi_command_param[5] = 0x0F;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x04;
			dsi_command_param[1] = 0x00;
			dsi_command_param[2] = 0x4c;
			dsi_command_param[3] = 0x23;
			dsi_command_param[4] = 0x01;
			dsi_command_param[5] = 0x0F;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x04;
			dsi_command_param[1] = 0x00;
			dsi_command_param[2] = 0x4c;
			dsi_command_param[3] = 0x24;
			dsi_command_param[4] = 0x01;
			dsi_command_param[5] = 0x0F;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x04;
			dsi_command_param[1] = 0x00;
			dsi_command_param[2] = 0x4c;
			dsi_command_param[3] = 0x25;
			dsi_command_param[4] = 0x01;
			dsi_command_param[5] = 0x0F;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x04;
			dsi_command_param[1] = 0x00;
			dsi_command_param[2] = 0x4c;
			dsi_command_param[3] = 0x26;
			dsi_command_param[4] = 0x01;
			dsi_command_param[5] = 0x0F;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x04;
			dsi_command_param[1] = 0x00;
			dsi_command_param[2] = 0x4c;
			dsi_command_param[3] = 0x27;
			dsi_command_param[4] = 0x01;
			dsi_command_param[5] = 0x00;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x04;
			dsi_command_param[1] = 0x00;
			dsi_command_param[2] = 0x4c;
			dsi_command_param[3] = 0x28;
			dsi_command_param[4] = 0x01;
			dsi_command_param[5] = 0x02;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x04;
			dsi_command_param[1] = 0x00;
			dsi_command_param[2] = 0x4c;
			dsi_command_param[3] = 0x29;
			dsi_command_param[4] = 0x01;
			dsi_command_param[5] = 0x02;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x04;
			dsi_command_param[1] = 0x00;
			dsi_command_param[2] = 0x4c;
			dsi_command_param[3] = 0x2A;
			dsi_command_param[4] = 0x01;
			dsi_command_param[5] = 0x02;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x04;
			dsi_command_param[1] = 0x00;
			dsi_command_param[2] = 0x4c;
			dsi_command_param[3] = 0x2B;
			dsi_command_param[4] = 0x01;
			dsi_command_param[5] = 0x0F;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x04;
			dsi_command_param[1] = 0x00;
			dsi_command_param[2] = 0x4c;
			dsi_command_param[3] = 0x2C;
			dsi_command_param[4] = 0x01;
			dsi_command_param[5] = 0x0F;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x04;
			dsi_command_param[1] = 0x00;
			dsi_command_param[2] = 0x4c;
			dsi_command_param[3] = 0x2D;
			dsi_command_param[4] = 0x01;
			dsi_command_param[5] = 0x0F;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x04;
			dsi_command_param[1] = 0x00;
			dsi_command_param[2] = 0x4c;
			dsi_command_param[3] = 0x2E;
			dsi_command_param[4] = 0x01;
			dsi_command_param[5] = 0x0F;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x04;
			dsi_command_param[1] = 0x00;
			dsi_command_param[2] = 0x4c;
			dsi_command_param[3] = 0x2F;
			dsi_command_param[4] = 0x01;
			dsi_command_param[5] = 0x0F;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x04;
			dsi_command_param[1] = 0x00;
			dsi_command_param[2] = 0x4c;
			dsi_command_param[3] = 0x30;
			dsi_command_param[4] = 0x01;
			dsi_command_param[5] = 0x0F;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x04;
			dsi_command_param[1] = 0x00;
			dsi_command_param[2] = 0x4c;
			dsi_command_param[3] = 0x31;
			dsi_command_param[4] = 0x01;
			dsi_command_param[5] = 0x0F;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x04;
			dsi_command_param[1] = 0x00;
			dsi_command_param[2] = 0x4c;
			dsi_command_param[3] = 0x32;
			dsi_command_param[4] = 0x01;
			dsi_command_param[5] = 0x00;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x04;
			dsi_command_param[1] = 0x00;
			dsi_command_param[2] = 0x4c;
			dsi_command_param[3] = 0x33;
			dsi_command_param[4] = 0x01;
			dsi_command_param[5] = 0x02;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x04;
			dsi_command_param[1] = 0x00;
			dsi_command_param[2] = 0x4c;
			dsi_command_param[3] = 0x34;
			dsi_command_param[4] = 0x01;
			dsi_command_param[5] = 0x02;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x04;
			dsi_command_param[1] = 0x00;
			dsi_command_param[2] = 0x4c;
			dsi_command_param[3] = 0x35;
			dsi_command_param[4] = 0x01;
			dsi_command_param[5] = 0x02;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x04;
			dsi_command_param[1] = 0x00;
			dsi_command_param[2] = 0x4c;
			dsi_command_param[3] = 0x50;
			dsi_command_param[4] = 0x01;
			dsi_command_param[5] = 0x0C;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x04;
			dsi_command_param[1] = 0x00;
			dsi_command_param[2] = 0x4c;
			dsi_command_param[3] = 0x53;
			dsi_command_param[4] = 0x01;
			dsi_command_param[5] = 0x42;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x04;
			dsi_command_param[1] = 0x00;
			dsi_command_param[2] = 0x4c;
			dsi_command_param[3] = 0x54;
			dsi_command_param[4] = 0x01;
			dsi_command_param[5] = 0x42;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x04;
			dsi_command_param[1] = 0x00;
			dsi_command_param[2] = 0x4c;
			dsi_command_param[3] = 0x55;
			dsi_command_param[4] = 0x01;
			dsi_command_param[5] = 0x41;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x04;
			dsi_command_param[1] = 0x00;
			dsi_command_param[2] = 0x4c;
			dsi_command_param[3] = 0x56;
			dsi_command_param[4] = 0x01;
			dsi_command_param[5] = 0x14;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x04;
			dsi_command_param[1] = 0x00;
			dsi_command_param[2] = 0x4c;
			dsi_command_param[3] = 0x59;
			dsi_command_param[4] = 0x01;
			dsi_command_param[5] = 0x88;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x04;
			dsi_command_param[1] = 0x00;
			dsi_command_param[2] = 0x4c;
			dsi_command_param[3] = 0x5A;
			dsi_command_param[4] = 0x01;
			dsi_command_param[5] = 0x01;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x04;
			dsi_command_param[1] = 0x00;
			dsi_command_param[2] = 0x4c;
			dsi_command_param[3] = 0x5B;
			dsi_command_param[4] = 0x01;
			dsi_command_param[5] = 0x00;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x04;
			dsi_command_param[1] = 0x00;
			dsi_command_param[2] = 0x4c;
			dsi_command_param[3] = 0x5C;
			dsi_command_param[4] = 0x01;
			dsi_command_param[5] = 0x02;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x04;
			dsi_command_param[1] = 0x00;
			dsi_command_param[2] = 0x4c;
			dsi_command_param[3] = 0x5D;
			dsi_command_param[4] = 0x01;
			dsi_command_param[5] = 0x0C;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x04;
			dsi_command_param[1] = 0x00;
			dsi_command_param[2] = 0x4c;
			dsi_command_param[3] = 0x5E;
			dsi_command_param[4] = 0x01;
			dsi_command_param[5] = 0x1C;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x04;
			dsi_command_param[1] = 0x00;
			dsi_command_param[2] = 0x4c;
			dsi_command_param[3] = 0x5F;
			dsi_command_param[4] = 0x01;
			dsi_command_param[5] = 0x27;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x04;
			dsi_command_param[1] = 0x00;
			dsi_command_param[2] = 0x4c;
			dsi_command_param[3] = 0x62;
			dsi_command_param[4] = 0x01;
			dsi_command_param[5] = 0x49;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x04;
			dsi_command_param[1] = 0x00;
			dsi_command_param[2] = 0x4c;
			dsi_command_param[3] = 0x63;
			dsi_command_param[4] = 0x01;
			dsi_command_param[5] = 0x27;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x04;
			dsi_command_param[1] = 0x00;
			dsi_command_param[2] = 0x4c;
			dsi_command_param[3] = 0x66;
			dsi_command_param[4] = 0x01;
			dsi_command_param[5] = 0x76;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x04;
			dsi_command_param[1] = 0x00;
			dsi_command_param[2] = 0x4c;
			dsi_command_param[3] = 0x67;
			dsi_command_param[4] = 0x01;
			dsi_command_param[5] = 0x27;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x04;
			dsi_command_param[1] = 0x00;
			dsi_command_param[2] = 0x4c;
			dsi_command_param[3] = 0x70;
			dsi_command_param[4] = 0x01;
			dsi_command_param[5] = 0x01;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x04;
			dsi_command_param[1] = 0x00;
			dsi_command_param[2] = 0x4c;
			dsi_command_param[3] = 0x71;
			dsi_command_param[4] = 0x01;
			dsi_command_param[5] = 0x0E;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x04;
			dsi_command_param[1] = 0x00;
			dsi_command_param[2] = 0x4c;
			dsi_command_param[3] = 0x72;
			dsi_command_param[4] = 0x01;
			dsi_command_param[5] = 0x02;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x04;
			dsi_command_param[1] = 0x00;
			dsi_command_param[2] = 0x4c;
			dsi_command_param[3] = 0x73;
			dsi_command_param[4] = 0x01;
			dsi_command_param[5] = 0x0C;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x04;
			dsi_command_param[1] = 0x00;
			dsi_command_param[2] = 0x4c;
			dsi_command_param[3] = 0x76;
			dsi_command_param[4] = 0x01;
			dsi_command_param[5] = 0x0C;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x04;
			dsi_command_param[1] = 0x00;
			dsi_command_param[2] = 0x4c;
			dsi_command_param[3] = 0x79;
			dsi_command_param[4] = 0x01;
			dsi_command_param[5] = 0x30;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x04;
			dsi_command_param[1] = 0x00;
			dsi_command_param[2] = 0x4c;
			dsi_command_param[3] = 0x82;
			dsi_command_param[4] = 0x01;
			dsi_command_param[5] = 0x00;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x04;
			dsi_command_param[1] = 0x00;
			dsi_command_param[2] = 0x4c;
			dsi_command_param[3] = 0x83;
			dsi_command_param[4] = 0x01;
			dsi_command_param[5] = 0x00;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x04;
			dsi_command_param[1] = 0x00;
			dsi_command_param[2] = 0x4c;
			dsi_command_param[3] = 0x84;
			dsi_command_param[4] = 0x01;
			dsi_command_param[5] = 0xFC;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x04;
			dsi_command_param[1] = 0x00;
			dsi_command_param[2] = 0x4c;
			dsi_command_param[3] = 0x86;
			dsi_command_param[4] = 0x01;
			dsi_command_param[5] = 0x00;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x04;
			dsi_command_param[1] = 0x00;
			dsi_command_param[2] = 0x4c;
			dsi_command_param[3] = 0x88;
			dsi_command_param[4] = 0x01;
			dsi_command_param[5] = 0x00;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x04;
			dsi_command_param[1] = 0x00;
			dsi_command_param[2] = 0x4c;
			dsi_command_param[3] = 0x8A;
			dsi_command_param[4] = 0x01;
			dsi_command_param[5] = 0x00;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x04;
			dsi_command_param[1] = 0x00;
			dsi_command_param[2] = 0x4c;
			dsi_command_param[3] = 0x8B;
			dsi_command_param[4] = 0x01;
			dsi_command_param[5] = 0x00;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x04;
			dsi_command_param[1] = 0x00;
			dsi_command_param[2] = 0x4c;
			dsi_command_param[3] = 0x8C;
			dsi_command_param[4] = 0x01;
			dsi_command_param[5] = 0x00;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x04;
			dsi_command_param[1] = 0x00;
			dsi_command_param[2] = 0x4c;
			dsi_command_param[3] = 0x8D;
			dsi_command_param[4] = 0x01;
			dsi_command_param[5] = 0xFC;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x04;
			dsi_command_param[1] = 0x00;
			dsi_command_param[2] = 0x4c;
			dsi_command_param[3] = 0x8F;
			dsi_command_param[4] = 0x01;
			dsi_command_param[5] = 0x00;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x04;
			dsi_command_param[1] = 0x00;
			dsi_command_param[2] = 0x4c;
			dsi_command_param[3] = 0x91;
			dsi_command_param[4] = 0x01;
			dsi_command_param[5] = 0x00;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x04;
			dsi_command_param[1] = 0x00;
			dsi_command_param[2] = 0x4c;
			dsi_command_param[3] = 0x93;
			dsi_command_param[4] = 0x01;
			dsi_command_param[5] = 0x00;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x04;
			dsi_command_param[1] = 0x00;
			dsi_command_param[2] = 0x4c;
			dsi_command_param[3] = 0x94;
			dsi_command_param[4] = 0x01;
			dsi_command_param[5] = 0x00;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x04;
			dsi_command_param[1] = 0x00;
			dsi_command_param[2] = 0x4c;
			dsi_command_param[3] = 0x95;
			dsi_command_param[4] = 0x01;
			dsi_command_param[5] = 0x00;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x04;
			dsi_command_param[1] = 0x00;
			dsi_command_param[2] = 0x4c;
			dsi_command_param[3] = 0x96;
			dsi_command_param[4] = 0x01;
			dsi_command_param[5] = 0xFC;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x04;
			dsi_command_param[1] = 0x00;
			dsi_command_param[2] = 0x4c;
			dsi_command_param[3] = 0x98;
			dsi_command_param[4] = 0x01;
			dsi_command_param[5] = 0x00;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x04;
			dsi_command_param[1] = 0x00;
			dsi_command_param[2] = 0x4c;
			dsi_command_param[3] = 0x9A;
			dsi_command_param[4] = 0x01;
			dsi_command_param[5] = 0x00;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x04;
			dsi_command_param[1] = 0x00;
			dsi_command_param[2] = 0x4c;
			dsi_command_param[3] = 0x9C;
			dsi_command_param[4] = 0x01;
			dsi_command_param[5] = 0x00;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			dsi_command_param[0] = 0x04;
			dsi_command_param[1] = 0x00;
			dsi_command_param[2] = 0x4c;
			dsi_command_param[3] = 0x9D;
			dsi_command_param[4] = 0x01;
			dsi_command_param[5] = 0x00;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

			/* tremolo-m dpi output on */
			dsi_command_param[0] = 0x81;
			dsi_command_param[1] = 0x00;
			mipi_dsih_gen_wr_packet(dev, 0, 0x05, dsi_command_param, 2);

			dsi_command_param[0] = 0x04;
			dsi_command_param[1] = 0x00;
			dsi_command_param[2] = 0x4C;
			dsi_command_param[3] = 0x02;
			dsi_command_param[4] = 0x01;
			dsi_command_param[5] = 0x00;
			mipi_dsih_gen_wr_packet(dev, 0, 0x39, dsi_command_param, 6);

			dsi_command_param[0] = 0x4A;
			dsi_command_param[1] = 0x1D;
			mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param, 2);

				if (video_mode == COMMAND_MODE) {
					dsi_command_param[0] = 0x40;
					dsi_command_param[1] = 0x01;
					mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param,
								2);
				} else if (video_mode == VIDEO_MODE) {
					dsi_command_param[0] = 0x40;
					dsi_command_param[1] = 0x0;
					mipi_dsih_gen_wr_packet(dev, 0, 0x15, dsi_command_param,
								2);
				}
		}
		return 0;
		case SHARP_LS045K3SX02:
		{
			pr_info("%s:SHARP LS045K3SX02\n", FUNC_NAME);

			mipi_dsih_cmd_mode(dev, 1);

			mipi_dsih_write_word(dev,0x6c,0x00001105);
			mipi_dsih_write_word(dev,0x70,0x00004036);
			mipi_dsih_write_word(dev,0x70,0x005A5AFC);
			mipi_dsih_write_word(dev,0x70,0x38555AF5);
			mipi_dsih_write_word(dev,0x6c,0x00001105);
			mdelay(120);
			mipi_dsih_write_word(dev,0x6c,0x00002905);

			//dsi_command_param[0] = 0x11;
			//dsi_command_param[1] = 0x0;
			//mipi_dsih_gen_wr_packet(dev, 0, 0x5, dsi_command_param, 2);

			//mdelay(1);

			//dsi_command_param[0] = 0x11;
			//dsi_command_param[1] = 0x0;
			//mipi_dsih_gen_wr_packet(dev, 0, 0x5, dsi_command_param, 2);

			//mdelay(150);

			//dsi_command_param[0] = 0x29;
			//dsi_command_param[1] = 0x0;
			//mipi_dsih_gen_wr_packet(dev, 0, 0x5, dsi_command_param, 2);
		}
		return 0;
		case RPI_DISPLAY:
		{
			pr_info("%s:RPI DISPLAY\n", FUNC_NAME);

			mipi_dsih_cmd_mode(dev, 1);
			mdelay(800);
			/* 1002 0300 0000 */
			{
				dsi_command_param[0] = 0x06;
				dsi_command_param[1] = 0x00;

				dsi_command_param[3] = 0x02;
				dsi_command_param[2] = 0x10;

				dsi_command_param[5] = 0x00;
				dsi_command_param[4] = 0x03;

				dsi_command_param[7] = 0x00;
				dsi_command_param[6] = 0x00;

			}
			mipi_dsih_gen_wr_packet(dev, 0, 0x29, dsi_command_param, 8);
			udelay(500);
			/* 6401 0500 0000 */
			{
				dsi_command_param[0] = 0x06;
				dsi_command_param[1] = 0x00;

				dsi_command_param[3] = 0x01;
				dsi_command_param[2] = 0x64;

				dsi_command_param[5] = 0x00;
				dsi_command_param[4] = 0x05;

				dsi_command_param[7] = 0x00;
				dsi_command_param[6] = 0x00;
			}
			mipi_dsih_gen_wr_packet(dev, 0, 0x29, dsi_command_param, 8);
			udelay(500);
			/* 6801 0500 0000 */
			{
				dsi_command_param[0] = 0x06;
				dsi_command_param[1] = 0x00;

				dsi_command_param[3] = 0x01;
				dsi_command_param[2] = 0x68;

				dsi_command_param[5] = 0x00;
				dsi_command_param[4] = 0x05;

				dsi_command_param[7] = 0x00;
				dsi_command_param[6] = 0x00;
			}
			mipi_dsih_gen_wr_packet(dev, 0, 0x29, dsi_command_param, 8);
			udelay(500);
			/* 4401 0000 0000 */
			{
				dsi_command_param[0] = 0x06;
				dsi_command_param[1] = 0x00;

				dsi_command_param[3] = 0x01;
				dsi_command_param[2] = 0x44;

				dsi_command_param[5] = 0x00;
				dsi_command_param[4] = 0x00;

				dsi_command_param[7] = 0x00;
				dsi_command_param[6] = 0x00;
			}
			mipi_dsih_gen_wr_packet(dev, 0, 0x29, dsi_command_param, 8);
			udelay(500);
			/* 4801 0000 0000 */
			{
				dsi_command_param[0] = 0x06;
				dsi_command_param[1] = 0x00;

				dsi_command_param[3] = 0x01;
				dsi_command_param[2] = 0x48;

				dsi_command_param[5] = 0x00;
				dsi_command_param[4] = 0x00;

				dsi_command_param[7] = 0x00;
				dsi_command_param[6] = 0x00;
			}
			mipi_dsih_gen_wr_packet(dev, 0, 0x29, dsi_command_param, 8);
			udelay(500);
			/* 1401 0300 0000 */
			{
				dsi_command_param[0] = 0x06;
				dsi_command_param[1] = 0x00;

				dsi_command_param[3] = 0x01;
				dsi_command_param[2] = 0x14;

				dsi_command_param[5] = 0x00;
				dsi_command_param[4] = 0x03;

				dsi_command_param[7] = 0x00;
				dsi_command_param[6] = 0x00;
			}
			mipi_dsih_gen_wr_packet(dev, 0, 0x29, dsi_command_param, 8);
			udelay(500);
			/* 5004 0000 0000 */
			{
				dsi_command_param[0] = 0x06;
				dsi_command_param[1] = 0x00;

				dsi_command_param[3] = 0x04;
				dsi_command_param[2] = 0x50;

				dsi_command_param[5] = 0x00;
				dsi_command_param[4] = 0x00;

				dsi_command_param[7] = 0x00;
				dsi_command_param[6] = 0x00;
			}
			mipi_dsih_gen_wr_packet(dev, 0, 0x29, dsi_command_param, 8);
			udelay(500);
			/* 2004 5001 1000 */
			{
				dsi_command_param[0] = 0x06;
				dsi_command_param[1] = 0x00;

				dsi_command_param[3] = 0x04;
				dsi_command_param[2] = 0x20;

				dsi_command_param[5] = 0x01;
				dsi_command_param[4] = 0x50;

				dsi_command_param[7] = 0x00;
				dsi_command_param[6] = 0x10;
			}
			mipi_dsih_gen_wr_packet(dev, 0, 0x29, dsi_command_param, 8);
			udelay(500);
			/* 6404 0F04 0000 */
			{
				dsi_command_param[0] = 0x06;
				dsi_command_param[1] = 0x00;

				dsi_command_param[3] = 0x04;
				dsi_command_param[2] = 0x64;

				dsi_command_param[5] = 0x04;
				dsi_command_param[4] = 0x0F;

				dsi_command_param[7] = 0x00;
				dsi_command_param[6] = 0x00;
			}
			mipi_dsih_gen_wr_packet(dev, 0, 0x29, dsi_command_param, 8);
			udelay(500);
			/* 0401 0100 0000 */
			{
				dsi_command_param[0] = 0x06;
				dsi_command_param[1] = 0x00;

				dsi_command_param[3] = 0x01;
				dsi_command_param[2] = 0x04;

				dsi_command_param[5] = 0x00;
				dsi_command_param[4] = 0x01;

				dsi_command_param[7] = 0x00;
				dsi_command_param[6] = 0x00;
			}
			mipi_dsih_gen_wr_packet(dev, 0, 0x29, dsi_command_param, 8);
			udelay(500);
			/* 0402 0100 0000 */
			{
				dsi_command_param[0] = 0x06;
				dsi_command_param[1] = 0x00;


				dsi_command_param[3] = 0x02;
				dsi_command_param[2] = 0x04;

				dsi_command_param[5] = 0x00;
				dsi_command_param[4] = 0x01;

				dsi_command_param[7] = 0x00;
				dsi_command_param[6] = 0x00;
			}
			mipi_dsih_gen_wr_packet(dev, 0, 0x29, dsi_command_param, 8);
			mdelay(100);
		}
		return 0;
		case SNPS_DSI_DEVICE:
		{
			mipi_dsih_eotp_tx(dev, 1);
			mipi_dsih_eotp_rx(dev, 1);

		}
		return 0;
		default:
			pr_info("%s:Screen not recognized!\n", FUNC_NAME);
	}
	return 1;
}

#endif
