// SPDX-License-Identifier: GPL-2.0
/**
 * Copyright (C) 2018 Synopsys, Inc.
 *
 * @file dsih_framebuffer.c
 * @brief Synopsys MIPI framebuffer functions
 * included as a part of Synopsys MIPI DSI Host controller driver
 *
 * @author Luis Oliveira <luis.oliveira@synopsys.com>
 */

#include "dsih_framebuffer.h"
#include "dsih_ioctl.h"
#include "dsih_api.h"
#include "dsih_dphy.h"
#include "dsih_hal.h"
#include "dsih_video.h"
#include "dsih_displays.h"

struct mipi_dsi_dev *pdev_array[2];
struct mipi_dsi_dev *pdev;
int fb_count = 0;

void init_frame_buffer(struct mipi_dsi_dev *dev)
{

	if (dev == NULL) {
		pr_err("%s:mipi_dsi_dev is NULL", FUNC_NAME);
		return;
	}
	// Init fb_fix_screeninfo
	strcpy(dev->fb.info.fix.id, "MIPI DSI");
	dev->fb.info.fix.type = FB_TYPE_PACKED_PIXELS;
	dev->fb.info.fix.visual = FB_VISUAL_TRUECOLOR;
	dev->fb.info.fix.xpanstep = 0;	// No HW panning
	dev->fb.info.fix.ypanstep = 0;	// No HW panning
	dev->fb.info.fix.ywrapstep = 0;	// No HW ywrap
	dev->fb.info.fix.accel = FB_ACCEL_NONE;	// No hardware accelerator
	dev->fb.info.fix.capabilities = FB_CAP_FOURCC;

	// Init fb_var_screeninfo
	dev->fb.info.var.xres = 640;
	dev->fb.info.var.yres = 480;
	dev->fb.info.var.xres_virtual = 640;
	dev->fb.info.var.yres_virtual = 480;
	dev->fb.info.var.xoffset = 0;
	dev->fb.info.var.yoffset = 0;
	dev->fb.info.var.bits_per_pixel = 16;	// Default RGB565
	dev->fb.info.var.grayscale = 0;
	dev->fb.info.var.activate = FB_ACTIVATE_NOW;
	dev->fb.info.var.height = -1;
	dev->fb.info.var.width = -1;
	dev->fb.info.var.pixclock = 20000;	// in pico-seconds
	dev->fb.info.var.left_margin = 64;
	dev->fb.info.var.right_margin = 64;
	dev->fb.info.var.upper_margin = 32;
	dev->fb.info.var.lower_margin = 32;
	dev->fb.info.var.hsync_len = 64;
	dev->fb.info.var.vsync_len = 2;
	dev->fb.info.var.vmode = FB_VMODE_NONINTERLACED;

	// Configure the operations
	dev->fb.ops.owner = THIS_MODULE;
	dev->fb.ops.fb_open = fb_mipi_open;
	dev->fb.ops.fb_release = fb_mipi_release;
	dev->fb.ops.fb_read = fb_mipi_read;
	dev->fb.ops.fb_write = fb_mipi_write;
	dev->fb.ops.fb_check_var = fb_mipi_check_var;
	dev->fb.ops.fb_set_par = fb_mipi_set_par;
	dev->fb.ops.fb_setcolreg = fb_mipi_setcolreg;
	dev->fb.ops.fb_setcmap = fb_mipi_setcmap;
	dev->fb.ops.fb_blank = fb_mipi_blank;
	dev->fb.ops.fb_pan_display = fb_mipi_pan_display;
	dev->fb.ops.fb_fillrect = fb_mipi_fillrect;
	dev->fb.ops.fb_copyarea = fb_mipi_copyarea;
	dev->fb.ops.fb_imageblit = fb_mipi_imageblit;
	dev->fb.ops.fb_cursor = fb_mipi_cursor;
	//dev->fb.ops.fb_rotate = fb_mipi_rotate;
	dev->fb.ops.fb_sync = fb_mipi_sync;
	dev->fb.ops.fb_ioctl = fb_mipi_ioctl;
	dev->fb.ops.fb_get_caps = fb_mipi_get_caps;
	dev->fb.ops.fb_destroy = fb_mipi_destroy;

	// Configure the info structure
	dev->fb.info.fbops = &dev->fb.ops;

	// Update the internal device pointer
	pdev_array[fb_count] = dev;
	fb_count++;
}

int bus32_write(uint32_t offset, uint32_t data)
{

	iowrite32(data, (void *)(pdev->core_addr + offset));
	return 0;
}

int bus32_read(uint32_t offset, uint32_t * data)
{
	*data = ioread32((void *)(pdev->core_addr + offset));
	return 0;
}

int fb_mipi_ioctl(struct fb_info *info, unsigned int cmd, unsigned long arg)
{
	static uint8_t rx_en = 0;
	static uint8_t ack_en = 0;
	static uint8_t eotp_tx_en = 0;
	static uint8_t eotp_rx_en = 0;
	static uint8_t ecc_en = 0;
	static uint8_t cmd_lp = 0;
	static uint8_t ready_dly = 4;
	int ret = 0;
	int param = 0;
	int tmp = 0;
	int i = 0;

	fb_ioctl_data fb_data;
	dsi_basic_config_t dsi_config;
	dsih_cmd_mode_video_t *cmd_video_params;
	cmd_write_t cmd_write;
	cmd_read_t cmd_read;
	tear_t tear;
	dphy_t dphy;
	presp_t presp;

	pdev = pdev_array[info->node];

	if (pdev == NULL) {
		pr_err("%s:mipi_dsi_dev device is NULL\n", FUNC_NAME);
		return -EINVAL;
	}

	disable_irq(pdev->irq_core);
	disable_irq(pdev->irq_vif);

	switch (cmd) {

	case FB_MIPI_CORE_READ:
	{
		copy_from_user(&fb_data, (void __user *)arg,
			       sizeof(fb_ioctl_data));
		ret = bus32_read(fb_data.address, &fb_data.value);

		if (ret) {
			pr_info("%s:READ:  reg 0x%08x [-EIO]\n",
				FUNC_NAME, fb_data.address);

			enable_irq(pdev->irq_core);
			enable_irq(pdev->irq_vif);

			return ret;
		}
		copy_to_user((void __user *)arg, &fb_data,
			     sizeof(fb_ioctl_data));

		/*pr_info("%s:READ:  reg 0x%08x - value 0x%08x\n",
			FUNC_NAME, fb_data.address, fb_data.value);
		*/
		break;
	}
	case FB_MIPI_CORE_WRITE:
	{
		copy_from_user(&fb_data, (void __user *)arg,
			       sizeof(fb_ioctl_data));
		ret = bus32_write(fb_data.address, fb_data.value);
		if (ret) {
			enable_irq(pdev->irq_core);
			enable_irq(pdev->irq_vif);
			return ret;
		}

		copy_to_user((void __user *)arg, &fb_data,
			     sizeof(fb_ioctl_data));
		break;
	}
	case FB_MIPI_BASE_ADDR:
	{
		ret = (uint32_t) pdev->core_addr;
		copy_to_user((void __user *)arg, &ret, sizeof(ret));
		break;
	}
	case FB_MIPI_READY_CLK_DLY:
	{
		copy_from_user(&ready_dly, (void __user *)arg, sizeof(int));
		mipi_dsih_dphy_delay(pdev, ready_dly);
		break;
	}
	case FB_MIPI_VIF_TEST:
	{
		copy_from_user(&param, (void __user *)arg, sizeof(int));
		video_if_test_mode(pdev, param);
		break;
	}
	case FB_MIPI_VIF_HRES:
	{
		copy_from_user(&param, (void __user *)arg, sizeof(int));
		video_if_h_active(pdev, param);
		break;
	}
	case FB_MIPI_VIF_HBP:
	{
		copy_from_user(&param, (void __user *)arg, sizeof(int));
		video_if_horizontal_blanking(pdev, param);
		break;
	}
	case FB_MIPI_VIF_HSYNC:
	{
		copy_from_user(&param, (void __user *)arg, sizeof(int));
		video_if_horizontal_sync(pdev, param);
		break;
	}
	case FB_MIPI_VIF_LTIME:
	{
		copy_from_user(&param, (void __user *)arg, sizeof(int));
		video_if_line_time(pdev, param);
		break;
	}
	case FB_MIPI_VIF_FPS:
	{
		copy_from_user(&param, (void __user *)arg, sizeof(int));
		video_if_fps(pdev, param);
		break;
	}
	case FB_MIPI_VIF_VRES:
	{
		copy_from_user(&param, (void __user *)arg, sizeof(int));
		video_if_v_active(pdev, param);
		break;
	}
	case FB_MIPI_VIF_VBP:
	{
		copy_from_user(&param, (void __user *)arg, sizeof(int));
		video_if_vertical_blanking(pdev, param);
		break;
	}
	case FB_MIPI_VIF_VFP:
	{
		copy_from_user(&param, (void __user *)arg, sizeof(int));
		video_if_vertical_front_porch(pdev, param);
		break;
	}
	case FB_MIPI_VIF_VSYNC:
	{
		copy_from_user(&param, (void __user *)arg, sizeof(int));
		video_if_vertical_sync(pdev, param);
		break;
	}
	case FB_MIPI_VIF_PSEL:
	{
		copy_from_user(&param, (void __user *)arg, sizeof(int));
		video_if_phase(pdev, param);
		break;
	}
	case FB_MIPI_VIF_BPP:
	{
		copy_from_user(&param, (void __user *)arg, sizeof(int));
		video_if_set_color_mode(pdev, param);
		break;
	}
	case FB_MIPI_VIF_PCONF:
	{
		copy_from_user(&param, (void __user *)arg, sizeof(int));
		video_if_set_color_mode(pdev, param);
		break;
	}
	case FB_MIPI_VIF_EDPI:
	{
		copy_from_user(&param, (void __user *)arg, sizeof(int));
		video_if_edpi(pdev, param);
		break;
	}
	case FB_MIPI_VIF_TE:
	{
		copy_from_user(&param, (void __user *)arg, sizeof(int));
		video_if_wait_for_tearing(pdev, param);
		break;
	}
	case FB_MIPI_VIF_PRE:
	{
		copy_from_user(&param, (void __user *)arg, sizeof(int));
		video_if_config(pdev, param);
		break;
	}
	case FB_MIPI_PATTERN_GENERATOR:
	{
		copy_from_user(&param, (void __user *)arg, sizeof(int));

		switch (param) {
		case 0:
			stop_video_pattern(pdev);
			mipi_dsih_hal_int_mask_1(pdev, 0xffffffff);
			break;
		case 1:
			mipi_dsih_hal_int_mask_1(pdev, 0xffffff7f);
			start_video_pattern(pdev, 0, 0);
			break;
		case 2:
			mipi_dsih_hal_int_mask_1(pdev, 0xffffff7f);
			start_video_pattern(pdev, 1, 0);
			break;
		case 3:
			mipi_dsih_hal_int_mask_1(pdev, 0xffffff7f);
			start_video_pattern(pdev, 0, 1);
			break;
		default:
			pr_err("%s:Option not valid!\n", FUNC_NAME);
			break;
		}

		break;
	}
	case FB_MIPI_DPI_WRITE:
	{
		copy_from_user(&(pdev->dpi_video), (void __user *)arg,
			       sizeof(dsih_dpi_video_t));
		if (dsi_get_transition_times(pdev) == FALSE) {
			pr_err("error configuring video - freq out of range\n");
			copy_dpi_param_changes(&pdev->dpi_video_old,
					       &pdev->dpi_video);
		} else {
			ret = mipi_dsih_dpi_video(pdev);

			if (ret) {
				pr_err("error configuring video %d\n", ret);
				copy_dpi_param_changes(&pdev->dpi_video_old,
						       &pdev->dpi_video);
			} else {
				mipi_dsih_reset_controller(pdev);
				copy_dpi_param_changes(&pdev->dpi_video,
						       &pdev->dpi_video_old);
			}
		}
		break;
	}
	case FB_MIPI_DPI_READ:
	{
		copy_to_user((void __user *)arg, &(pdev->dpi_video),
			     sizeof(dsih_dpi_video_t));
		break;
	}
	case FB_MIPI_DPI_PRE_CONFIG:
	{
		copy_from_user(&dsi_config, (void __user *)arg,
			       sizeof(dsi_basic_config_t));
		pre_video_mode(pdev, dsi_config.screen, dsi_config.lanes);

		if (dsi_get_transition_times(pdev) == FALSE) {
			pr_err("error configuring video - freq out of range\n");
			copy_dpi_param_changes(&pdev->dpi_video_old, &pdev->dpi_video);
		} else {
			ret = mipi_dsih_dpi_video(pdev);
			if (ret) {
				pr_err("error configuring video %d\n", ret);
				copy_dpi_param_changes(&pdev->dpi_video_old, &pdev->dpi_video);
			} else {
				mipi_dsih_reset_controller(pdev);
				copy_dpi_param_changes(&pdev->dpi_video, &pdev->dpi_video_old);
			}
		}
		break;
	}
	case FB_MIPI_EDPI_WRITE:
	{
		copy_from_user(&(pdev->cmd_mode_video), (void __user *)arg,
			       sizeof(dsih_cmd_mode_video_t));

		ret =
		    mipi_dsih_dphy_configure(pdev,
					     pdev->cmd_mode_video.no_of_lanes,
					     pdev->cmd_mode_video.byte_clock *
					     8);
		if (!ret)
			pr_err("PHY configuration error\n");
		ret = mipi_dsih_edpi_video(pdev);
		if (!ret)
			pr_err("eDPI configuration error\n");
		else {
			mipi_dsih_reset_controller(pdev);
			copy_edpi_param_changes(&pdev->cmd_mode_video,
						&pdev->cmd_mode_video_old);
		}
		break;
	}
	case FB_MIPI_EDPI_READ:
	{
		copy_to_user((void __user *)arg, &(pdev->cmd_mode_video),
			     sizeof(dsih_cmd_mode_video_t));
		break;
	}
	case FB_MIPI_EDPI_PRE_CONFIG:
	{
		copy_from_user(&param, (void __user *)arg, sizeof(int));
		pre_command_mode(pdev, param, pdev->cmd_mode_video.no_of_lanes);

		ret =
		    mipi_dsih_dphy_configure(pdev,
					     pdev->cmd_mode_video.no_of_lanes,
					     pdev->cmd_mode_video.byte_clock *
					     8);
		if (!ret)
			pr_err("PHY configuration error\n");
		ret = mipi_dsih_edpi_video(pdev);
		if (!ret)
			pr_err("eDPI configuration error\n");
		else {
			mipi_dsih_reset_controller(pdev);
			copy_edpi_param_changes(&pdev->cmd_mode_video,
						&pdev->cmd_mode_video_old);
		}

		break;
	}
	case FB_MIPI_PLATFORM_INIT:
	{
		copy_from_user(&dsi_config, (void __user *)arg,
			       sizeof(dsi_basic_config_t));
		dsi_platform_init(pdev, dsi_config.screen,
				  dsi_config.video_mode, dsi_config.lanes);
		break;
	}
	case FB_MIPI_DCS_WRITE:
	{
		copy_from_user(&cmd_write, (void __user *)arg,
			       sizeof(cmd_write_t));

		mipi_dsih_cmd_mode(pdev, 1);

		/*********** debug *********/
		pr_debug("vc %d ", cmd_write.vc);
		for (i=0;i<=cmd_write.length;i++)
			pr_debug("cmd_buffer[%d] %x ", i, cmd_write.cmd_buffer[i]);
		pr_debug("length %d \n", cmd_write.length);
		/*** vc, opcode, payload ***/

		mipi_dsih_dcs_wr_cmd(pdev, cmd_write.vc, cmd_write.cmd_buffer,
				     cmd_write.length);

		break;
	}
	case FB_MIPI_GEN_WRITE:
	{
		copy_from_user(&cmd_write, (void __user *)arg,
			       sizeof(cmd_write_t));
		mipi_dsih_cmd_mode(pdev, 1);

		/*********** debug *********/
		pr_debug("vc %d ", cmd_write.vc);
		for (i=0;i<=cmd_write.length;i++)
			pr_debug("cmd_buffer[%d] %x ", i,
				 cmd_write.cmd_buffer[i]);
		pr_debug("length %d \n", cmd_write.length);
		/*** vc, opcode, payload ***/

		mipi_dsih_gen_wr_cmd(pdev, cmd_write.vc, cmd_write.cmd_buffer,
				     cmd_write.length);
		break;
	}
	case FB_MIPI_PKT_WRITE:
	{
		copy_from_user(&cmd_write, (void __user *)arg,
			       sizeof(cmd_write_t));

		mipi_dsih_cmd_mode(pdev, 1);

		/*********** debug *********/
		pr_debug("vc %d ", cmd_write.vc);
		for (i=0;i<=cmd_write.length;i++)
			pr_debug("cmd_buffer[%d] %x ", i,
				 cmd_write.cmd_buffer[i]);
		pr_debug("data_type 0x%X \n", cmd_write.data_type);
		pr_debug("length %d \n", cmd_write.length);
		/*** vc, opcode, payload ***/

		mipi_dsih_gen_wr_packet(pdev, cmd_write.vc, cmd_write.data_type,
					cmd_write.cmd_buffer, cmd_write.length);

		break;
	}
	case FB_MIPI_READ_DCS:
	{
		copy_from_user(&cmd_read, (void __user *)arg,
			       sizeof(cmd_read_t));
		cmd_read.read_bytes =
		    mipi_dsih_dcs_rd_cmd(pdev, cmd_read.vc, cmd_read.command,
					 cmd_read.bytes_to_read,
					 cmd_read.cmd_buffer);

		copy_to_user((void __user *)arg, &cmd_read, sizeof(cmd_read_t));
		break;
	}
	case FB_MIPI_READ_GEN:
	{
		copy_from_user(&cmd_read, (void __user *)arg,
			       sizeof(cmd_read_t));

		cmd_read.read_bytes =
		    mipi_dsih_gen_rd_cmd(pdev, cmd_read.vc,
					 (unsigned char *)cmd_read.
					 param, cmd_read.length,
					 cmd_read.bytes_to_read,
					 cmd_read.cmd_buffer);

		copy_to_user((void __user *)arg, &cmd_read, sizeof(cmd_read_t));
		break;
	}
	case FB_MIPI_READ_PACKET:
	{
		copy_from_user(&cmd_read, (void __user *)arg,
			       sizeof(cmd_read_t));

		cmd_read.read_bytes =
		    mipi_dsih_gen_rd_packet(pdev, cmd_read.vc, cmd_read.command,
					    cmd_read.param[1],
					    cmd_read.param[0],
					    cmd_read.bytes_to_read,
					    cmd_read.cmd_buffer);

		copy_to_user((void __user *)arg, &cmd_read, sizeof(cmd_read_t));
		break;
	}
	case FB_MIPI_RX:
	{
		copy_from_user(&rx_en, (void __user *)arg, sizeof(int));

		mipi_dsih_enable_rx(pdev, rx_en);
		break;
	}
	case FB_MIPI_ACK:
	{
		copy_from_user(&ack_en, (void __user *)arg, sizeof(int));
		rx_en = ack_en ? 1 : rx_en;
		mipi_dsih_peripheral_ack(pdev, ack_en);
		mipi_dsih_enable_rx(pdev, rx_en);
		break;
	}
	case FB_MIPI_PHY_PWR:
	{
		copy_from_user(&param, (void __user *)arg, sizeof(int));
		if (param == 2)
			mipi_dsih_reset_phy(pdev);
		else
			mipi_dsih_shutdown_phy(pdev, !param);
		break;
	}
	case FB_MIPI_CORE_PWR:
	{
		copy_from_user(&param, (void __user *)arg, sizeof(int));
		if (param == 2)
			mipi_dsih_reset_controller(pdev);
		else
			mipi_dsih_shutdown_controller(pdev, !param);
		break;
	}
	case FB_MIPI_BRIDGE_PWR:
	{
		copy_from_user(&param, (void __user *)arg, sizeof(int));
		if (param == 1)
			reset_control_deassert(pdev->rst_2);
		else
			reset_control_assert(pdev->rst_2);
		break;
	}
	case FB_MIPI_CMD_HS:
	{
		copy_from_user(&cmd_lp, (void __user *)arg, sizeof(int));

		if (!cmd_lp)
			mipi_dsih_dphy_enable_hs_clk(pdev, 1);

		mipi_dsih_dcs_cmd_lp_transmission(pdev, cmd_lp, cmd_lp, cmd_lp);
		mipi_dsih_gen_cmd_lp_transmission(pdev, cmd_lp, cmd_lp, cmd_lp);
		break;
	}
	case FB_MIPI_ECC:
	{
		copy_from_user(&ecc_en, (void __user *)arg, sizeof(int));

		rx_en = ecc_en ? 1 : rx_en;
		mipi_dsih_ecc_rx(pdev, ecc_en);
		mipi_dsih_enable_rx(pdev, rx_en);
		break;
	}
	case FB_MIPI_EOTP_TX:
	{
		copy_from_user(&eotp_tx_en, (void __user *)arg, sizeof(int));

		rx_en = eotp_tx_en ? 1 : rx_en;
		mipi_dsih_eotp_tx(pdev, eotp_tx_en);
		mipi_dsih_enable_rx(pdev, rx_en);
		break;
	}
	case FB_MIPI_EOTP_RX:
	{
		copy_from_user(&eotp_rx_en, (void __user *)arg, sizeof(int));

		rx_en = eotp_rx_en ? 1 : rx_en;
		mipi_dsih_eotp_rx(pdev, eotp_rx_en);
		mipi_dsih_enable_rx(pdev, rx_en);
		break;
	}
	case FB_MIPI_CMODE:
	{
		copy_from_user(&param, (void __user *)arg, sizeof(int));

		mipi_dsih_hal_dpi_color_mode_pol(pdev, param);
		break;
	}
	case FB_MIPI_RESETTRIG:
	{
		tmp = (mipi_dsih_active_mode(pdev) == 2);
		/* disable video mode */
		mipi_dsih_cmd_mode(pdev, tmp);
		/* reset trigger is 1000 in the D-PHY */
		ret = mipi_dsih_dphy_escape_mode_trigger(pdev, 0x8);
		if (ret)
			pr_err("error %d", ret);

		mdelay(10);
		mipi_dsih_video_mode(pdev, tmp);
		break;
	}
	case FB_MIPI_HSCLK:
	{
		copy_from_user(&param, (void __user *)arg, sizeof(int));

		mipi_dsih_dphy_enable_hs_clk(pdev, tmp);
		break;
	}
	case FB_MIPI_ULPSSAFE:
	{
		copy_from_user(&param, (void __user *)arg, sizeof(int));

		if (param == 0) {
			/* make sure phy is awake before exiting ulps    */
			mipi_dsih_dphy_wakeup_pll(pdev);

			//enable clk_lane transmission
			mipi_dsih_dphy_enable_hs_clk(pdev, 1);

			ret = 0x00;
			ret = mipi_dsih_dphy_ulps_data_lanes(pdev, 0);
			ret = mipi_dsih_dphy_ulps_clk_lane(pdev, 0);
			//Disable command mode
			mipi_dsih_cmd_mode(pdev, 0);
			mdelay(10);
			//Reset PHY
			mipi_dsih_reset_phy(pdev);
			mdelay(10);
			//Reset Controller
			mipi_dsih_reset_controller(pdev);
		} else if (param == 1) {	//ON MODE
			mipi_dsih_cmd_mode(pdev, 1);	//Enable command mode
			ret = 0x00;
			tmp = 0;
			while (ret != 0x01 && tmp < 5000) {
				//Check if generic command FIFO is empty
				ret =
				    mipi_dsih_read_part(pdev,
							R_DSI_HOST_CMD_PKT_STATUS,
							0, 1);
				tmp++;
			}
			ret = 0x00;
			tmp = 0;
			while (ret != 0x01 && tmp < 5000) {
				//Check if DBI command FIFO is empty
				ret =
				    mipi_dsih_read_part(pdev,
							R_DSI_HOST_CMD_PKT_STATUS,
							8, 1);
				tmp++;
			}
			//disable clk_lane transmission
			mipi_dsih_dphy_enable_hs_clk(pdev, 0);

			ret = mipi_dsih_dphy_ulps_data_lanes(pdev, 1);
			ret = mipi_dsih_dphy_ulps_clk_lane(pdev, 1);

			mipi_dsih_reset_controller(pdev);
		}
		break;
	}
	case FB_MIPI_TEAR:
	{
		copy_from_user(&tear, (void __user *)arg, sizeof(tear_t));

		if (tear.status == 1) {
			/* on */
			cmd_video_params->te = 1;
			mipi_dsih_tear_effect_ack(pdev, 1);
		} else {
			/* off */
			cmd_video_params->te = 0;
			mipi_dsih_tear_effect_ack(pdev, 0);
		}
		pr_debug("TEAR EFFECT ack\n");
		break;
	}
	case FB_MIPI_PHY:
	{
		copy_from_user(&dphy, (void __user *)arg, sizeof(dphy_t));
		mipi_dsih_dphy_configure(pdev, dphy.lanes, dphy.output_freq);
		pdev->phy.lanes = dphy.lanes;
		pdev->phy.output_freq = dphy.output_freq;
		break;
	}
	case FB_MIPI_PRESP:
	{
		copy_from_user(&presp, (void __user *)arg, sizeof(presp_t));

		switch (presp.type) {
			case 1:
			mipi_dsih_presp_timeout_low_power_write(pdev, presp.time);
			break;

			case 2:
			mipi_dsih_presp_timeout_low_power_read(pdev, presp.time);
			break;

			case 3:
			mipi_dsih_presp_timeout_high_speed_write(pdev, presp.time);
			break;

			case 4:
			mipi_dsih_presp_timeout_high_speed_read(pdev, presp.time);
			break;

			case 5:
			mipi_dsih_presp_timeout_bta(pdev, presp.time);
			break;

			default:
			pr_err("presp type not valid\n");
			break;
		}
		break;
	}
	case FB_MIPI_VESA:
	{
		copy_from_user(&param, (void __user *)arg, sizeof(int));
		if (param == 1)
			mipi_dsih_write_word(pdev, R_DSI_HOST_DSC_PARAMETER, 1);
		else
			mipi_dsih_write_word(pdev, R_DSI_HOST_DSC_PARAMETER, 0);
		break;
	}
	case FB_MIPI_AULPS:
	{
		copy_from_user(&param, (void __user *)arg, sizeof(int));
		if (param == 1) {
			mipi_dsih_hal_auto_ulps_mode(pdev, 1);
		} else {
			mipi_dsih_hal_auto_ulps_mode(pdev, 0);
		}
		break;
	}
	default:
		pr_err("%s:IOCTL unknown 0x%X!\n", FUNC_NAME, cmd);
		break;
	}
	enable_irq(pdev->irq_core);
	enable_irq(pdev->irq_vif);
	return 0;
}

int fb_mipi_open(struct fb_info *info, int user)
{
	return 0;
}

int fb_mipi_release(struct fb_info *info, int user)
{
	return 0;
}

ssize_t fb_mipi_read(struct fb_info * info, char __user * buf, size_t count,
	     loff_t * ppos)
{
	return 0;
}

ssize_t fb_mipi_write(struct fb_info * info, const char __user * buf, size_t count,
	      loff_t * ppos)
{
	return count;
}

/********************************************************************
 * Miscellaneous functions - They should be used only after the HW is
 * configured.
 ********************************************************************/
int fb_mipi_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
	return 0;
}

int fb_mipi_set_par(struct fb_info *info)
{
	return 0;
}

int fb_mipi_setcolreg(unsigned regno, unsigned red, unsigned green, unsigned blue,
		          unsigned transp, struct fb_info *info)
{
	uint32_t *pal = info->pseudo_palette;
	uint32_t cr = red >> (16 - info->var.red.length);
	uint32_t cg = green >> (16 - info->var.green.length);
	uint32_t cb = blue >> (16 - info->var.blue.length);
	uint32_t value;

	if (regno >= 16)
		return -EINVAL;

	value = (cr << info->var.red.offset) |
	    (cg << info->var.green.offset) | (cb << info->var.blue.offset);
	if (info->var.transp.length > 0) {
		u32 mask = (1 << info->var.transp.length) - 1;
		mask <<= info->var.transp.offset;
		value |= mask;
	}
	pal[regno] = value;
	return 0;
}

int fb_mipi_setcmap(struct fb_cmap *cmap, struct fb_info *info)
{
	return 0;
}

int fb_mipi_blank(int blank, struct fb_info *info)
{
	return 0;
}

int fb_mipi_pan_display(struct fb_var_screeninfo *var, struct fb_info *info)
{
	return 0;
}
void fb_mipi_fillrect(struct fb_info *info, const struct fb_fillrect *rect)
{
	return;
}

void fb_mipi_copyarea(struct fb_info *info, const struct fb_copyarea *region)
{
	return;
}

void fb_mipi_imageblit(struct fb_info *info, const struct fb_image *image)
{
	return;
}

int fb_mipi_cursor(struct fb_info *info, struct fb_cursor *cursor)
{
	return 0;
}

void fb_mipi_rotate(struct fb_info *info, int angle)
{
	return;
}

int fb_mipi_sync(struct fb_info *info)
{
	return 0;
}

int fb_mipi_mmap(struct fb_info *info, struct vm_area_struct *vma)
{
	return 0;
}

void fb_mipi_get_caps(struct fb_info *info, struct fb_blit_caps *caps,
		 struct fb_var_screeninfo *var)
{
	return;
}

void fb_mipi_destroy(struct fb_info *info)
{
	if (info->screen_base)
		iounmap(info->screen_base);
}
