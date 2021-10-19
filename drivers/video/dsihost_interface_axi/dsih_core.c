// SPDX-License-Identifier: GPL-2.0
/**
 * Copyright (C) 2018 Synopsys, Inc.
 *
 * @file dsih_core.c
 * @brief Synopsys MIPI DSI driver API
 * included as a part of Synopsys MIPI DSI Host controller driver
 *
 * @author Luis Oliveira <luis.oliveira@synopsys.com>
 */

#include "includes.h"
#include "dsih_framebuffer.h"
#include "dsih_video.h"
#include "dsih_displays.h"
#include "dsih_api.h"
#include "dsih_dphy.h"
#include "dsih_hal.h"

/** @short License information */
MODULE_LICENSE("GPL v2");
/** @short Author information */
MODULE_AUTHOR("Luis Oliveira <lolivei@synopsys.com");
/** @short Device description */
MODULE_DESCRIPTION("MIPI DSI Host module driver");
/** @short Device version */
MODULE_VERSION("1.40");

/**
 * @short List of driver arguments
 */
int phy_g118 = 0;
module_param(phy_g118, int, 0644);
MODULE_PARM_DESC(phy_g118,
				 "DPHY is Gen3 8 bits (1) or 12 bits (0) the debug traces.");

/**
 * @short List of the devices
 * Linked list that contains the installed devices
 */
static LIST_HEAD(devlist_global);

/**
 * @short List of allocated memory
 * Linked list that contains the allocated memory
 */
static struct mem_alloc *alloc_list;

/**
* @short Init DSI parameters
* @param[in] dev MIPI DSI device
* @return none
*/
uint8_t mipi_dsi_init_param(struct mipi_dsi_dev *dev)
{
	dphy_t *phy = &dev->phy;
	dsih_dpi_video_t *video = &dev->dpi_video;
	dsih_cmd_mode_video_t *edpi_video = &dev->cmd_mode_video;
	uint8_t ret;

	pr_info("%s:DSI initialization\n", FUNC_NAME);
#ifdef GEN_3
	phy->reference_freq = 24000; /* [KHz] */
#else
	phy->reference_freq = 27000; /* [KHz] */
#endif

	if (dev == NULL)
	{
		pr_err("Dev Null\n");
		return -ENODEV;
	}

	dev->max_lanes = 4;
	dev->max_bta_cycles = 4095;
	dev->color_mode_polarity = 1;
	dev->shut_down_polarity = 1;

	mipi_dsih_presp_timeout_low_power_write(dev, 0);
	mipi_dsih_presp_timeout_low_power_read(dev, 0);
	mipi_dsih_presp_timeout_high_speed_write(dev, 0);
	mipi_dsih_presp_timeout_high_speed_read(dev, 0);
	mipi_dsih_presp_timeout_bta(dev, 8);

	/*
	 * Open instance first
	 * - to make sure addresses and other
	 * attributes are correct
	 */
	ret = mipi_dsih_open(dev, COLOR_CODE_24BIT);

	if (ret != TRUE)
		return ret;

	/* initialise DPI video params */
	video->no_of_lanes = 1;
	video->non_continuous_clock = 0;
	video->virtual_channel = 0;
	video->video_mode = 4;  /* invalid value */
	video->byte_clock = 0;  /* KHz  */
	video->pixel_clock = 0; /* dpi_clock KHz */
	video->is_18_loosely = 0;
	video->h_polarity = 0;
	video->h_active_pixels = 0; /* hadr */
	video->h_sync_pixels = 0;
	video->h_back_porch_pixels = 0; /* hbp */
	video->h_total_pixels = 0;		/* hfp */
	video->v_polarity = 0;
	video->v_active_lines = 0; /* vadr */
	video->v_sync_lines = 0;
	video->v_back_porch_lines = 0; /* vbp */
	video->v_total_lines = 0;	  /* vfp */
	/* as of 1.20a */
	video->max_hs_to_lp_cycles = 50;	  /* value for max freq */
	video->max_lp_to_hs_cycles = 153;	 /* value for max freq */
	video->max_clk_hs_to_lp_cycles = 66;  /* value for max freq */
	video->max_clk_lp_to_hs_cycles = 181; /* value for max freq */

	/* Initialise eDPI params */

	edpi_video->virtual_channel = 0;
	edpi_video->lp = 0;
	edpi_video->te = 0;
	edpi_video->bta = 0;
	edpi_video->h_start = 0;
	edpi_video->h_active_pixels = 0;
	edpi_video->packet_size = 0;
	edpi_video->v_start = 0;
	edpi_video->v_active_lines = 0;

	return TRUE;
}

/**
* @short Start DSI platform
* @param[in] dev MIPI DSI device
* @param[in] display Type of display
* @param[in] video_mode Video mode or command mode
* @param[in] lanes number of lanes
* @return none
*/
void dsi_platform_init(struct mipi_dsi_dev *dev, int display, int video_mode,
					   int lanes)
{
	dsi_screen_init(dev, display, video_mode);

	switch (video_mode)
	{
	case COMMAND_MODE:
		/* command mode */
		pr_debug("%s:Command Mode\n", FUNC_NAME);
		if (!pre_command_mode(dev, display, lanes))
		{
			pr_debug("%s:eDPI Video\n", FUNC_NAME);
			if (!mipi_dsih_edpi_video(dev))
				dev_err(dev->parent_dev,
						"error configuring video\n");

			copy_edpi_param_changes(&dev->cmd_mode_video,
									&dev->cmd_mode_video_old);
		}
		video_if_config(dev, display);
		video_if_edpi(dev, 1);
		break;
	case VIDEO_MODE:
		/* video mode */
		pr_debug("%s:Video Mode\n", FUNC_NAME);
		if (!pre_video_mode(dev, display, lanes))
		{
			pr_debug("%s:DPI Video\n", FUNC_NAME);
			pr_debug("%s:Using %d lanes\n", FUNC_NAME, lanes);
			if (mipi_dsih_dpi_video(dev))
				dev_err(dev->parent_dev,
						"error configuring video\n");

			copy_dpi_param_changes(&dev->dpi_video,
								   &dev->dpi_video_old);
		}
		video_if_config(dev, display);
		pre_video_mode(dev, display, lanes);
		break;
	default:
		dev_err(dev->parent_dev, "Invalid mode\n");
		break;
	}
	mipi_dsih_reset_controller(dev);
}

/*********************************************************/
//Interrupts

/**
* @short DSI interrupt handler
* @param[in] irq IRQ
* @param[in] dev_id device ID
* @return Error code
*/
irqreturn_t dwc_mipi_dsi_handler(int irq, void *dev_id)
{

	struct mipi_dsi_dev *dev = NULL;
	uint32_t status_0;
	uint32_t status_1;

	if (dev_id == NULL)
		return IRQ_NONE;

	dev = dev_id;

	//READ REGISTERr
	status_0 = mipi_dsih_hal_int_status_0(dev, 0xffffffff);
	status_1 = mipi_dsih_hal_int_status_1(dev, 0xffffffff);

	dev_info_ratelimited(dev->parent_dev, "IRQ 0 %X IRQ 1 %X\n", status_0,
						 status_1);

	return IRQ_HANDLED;
}

/**
* @short Video bridge interrupt handler
* @param[in] irq IRQ
* @param[in] dev_id device ID
* @return Error code
*/
irqreturn_t videobridge_handler(int irq, void *dev_id)
{
	struct mipi_dsi_dev *dev = NULL;

	if (dev_id == NULL)
		return IRQ_NONE;

	dev = dev_id;

	return IRQ_HANDLED;
}

/*********************************************************/

/**
 * @short Map memory blocks
 * @param[in,out] main MIPI DSI structure
 * @return Return -ENOMEM if one of the blocks is not mapped and 0 if all
 * blocks are mapped successful.
 */
int map_memory_blocks(struct platform_device *pdev, struct mipi_dsi_dev *dev)
{

	struct resource *mem = NULL;

	// Device tree information: Base addresses & mapping
	// Map DWC MIPI DSI Core
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (mem == NULL)
	{
		dev_err(&pdev->dev, "Base address of the device is not set.\n"
							"Refer to device tree.\n");
		return -ENXIO;
	}
	dev->core_mem_size = resource_size(mem);
	if (!request_mem_region(mem->start, resource_size(mem), "dwc_mipi_dsi"))
	{
		dev_err(dev->parent_dev,
				"%s:Unable to request mem region for DWC MIPI DSI\n",
				FUNC_NAME);
		return -ENOMEM;
	}
	dev->core_addr = ioremap(mem->start, resource_size(mem));
	if (!dev->core_addr)
	{
		dev_err(dev->parent_dev, "%s:Unable to map resource\n",
				FUNC_NAME);
		return -ENOMEM;
	}
	// Map Video Bridge
	mem = NULL;
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (mem == NULL)
	{
		dev_err(&pdev->dev, "Base address of the device is not set.\n"
							"Refer to device tree.\n");
		return -ENXIO;
	}
	dev->vid_bridge_mem_size = resource_size(mem);
	if (!request_mem_region(mem->start, resource_size(mem), "video_bridge"))
	{
		dev_err(dev->parent_dev,
				"%s:Unable to request mem region for Video Bridge\n",
				FUNC_NAME);
		return -ENOMEM;
	}
	dev->vid_bridge_addr = ioremap(mem->start, resource_size(mem));
	if (!dev->vid_bridge_addr)
	{
		dev_err(dev->parent_dev,
				"%s:Unable to map video_bridge_base_addr resource\n",
				FUNC_NAME);
		return -ENOMEM;
	}
#ifdef GEN_3
	// Map GEN 3 Interface
	mem = NULL;
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	if (mem == NULL)
	{
		dev_err(&pdev->dev, "Base address of the device is not set.\n"
							"Refer to device tree.\n");
		return -ENXIO;
	}
	dev->gen3_if_mem_size = resource_size(mem);
	if (!request_mem_region(mem->start, resource_size(mem), "gen3"))
	{
		dev_err(dev->parent_dev,
				"%s:Unable to request mem region for Gen3\n",
				FUNC_NAME);
		return -ENOMEM;
	}
	dev->gen3_if_addr = ioremap(mem->start, resource_size(mem));
	if (!dev->gen3_if_addr)
	{
		dev_err(dev->parent_dev,
				"%s:Unable to map gen3_base_addr resource\n",
				FUNC_NAME);
		return -ENOMEM;
	}
#endif
	// Map MMCM Interface

	mem = NULL;
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 3);
	if (mem == NULL)
	{
		dev_err(&pdev->dev, "Base address of the device is not set.\n"
							"Refer to device tree.\n");
		return -ENXIO;
	}
	dev->mmcm_mem_size = resource_size(mem);
	if (!request_mem_region(mem->start, resource_size(mem), "mcmm"))
	{
		dev_err(dev->parent_dev,
				"%s:Unable to request mem region for MMCM\n",
				FUNC_NAME);
		return -ENOMEM;
	}
	dev->mmcm_addr = ioremap(mem->start, resource_size(mem));
	if (!dev->mmcm_addr)
	{
		dev_err(dev->parent_dev,
				"%s:Unable to map mmcm addr resource\n",
				FUNC_NAME);
		return -ENOMEM;
	}

	return 0;
}

/**
 * @short Release memory blocks
 * @param[in,out] main MIPI DSI structure
 * @return Void
 */
void release_memory_blocks(struct mipi_dsi_dev *dev)
{

	release_mem_region((uint32_t)dev->core_addr, dev->core_mem_size);
	iounmap(dev->core_addr);

	release_mem_region((uint32_t)dev->vid_bridge_addr, dev->vid_bridge_mem_size);
	iounmap(dev->vid_bridge_addr);
#ifdef GEN_3
	release_mem_region((uint32_t)dev->gen3_if_addr, dev->gen3_if_mem_size);
	iounmap(dev->gen3_if_addr);
#endif
	release_mem_region((uint32_t)dev->mmcm_addr, dev->mmcm_mem_size);
	iounmap(dev->mmcm_addr);
}

/**
 * @short Register interrupts
 * @param[in,out] main MIPI DSI structure
 * @return Return error if one of the requests is not successful. Does not
 * stop the execution if on of the interrupts is not registered, although the
 * error is returned at the end of the function.
 */
int register_interrupts(struct mipi_dsi_dev *dev)
{
	int ret = 0;
	//Core Interrupts
	ret = request_irq(dev->irq_core,
					  dwc_mipi_dsi_handler, IRQF_SHARED,
					  "dwc_mipi_dsi_handler", dev);
	if (ret)
	{
		pr_err("%s:Could not register dwc_mipi_dsi interrupt\n",
			   FUNC_NAME);
	}
	//Bridge Interrupts
	ret = request_irq(dev->irq_vif,
					  videobridge_handler, IRQF_SHARED,
					  "videobridge_handler", dev);
	if (ret)
	{
		pr_err("%s:Could not register videobridge interrupt\n",
			   FUNC_NAME);
	}
	return ret;
}

/**
 * @short Unregister interrupts
 * @param[in,out] main MIPI DSI structure
 * @return Void
 */
void release_interrupts(struct mipi_dsi_dev *dev)
{
	free_irq(dev->irq_vif, dev);
	free_irq(dev->irq_core, dev);
}

/**
 * @short Allocate memory
 * @param[in] info String to associate with memory allocated
 * @param[in] size Size of the memory to allocate
 * @param[in,out] allocated Pointer to the structure that contains the info
 * about the allocation
 * @return Void
 */
void *alloc_mem(char *info, size_t size, struct mem_alloc *allocated)
{
	struct mem_alloc *new = NULL;
	int *return_pnt = NULL;

	// first time
	if (alloc_list == NULL)
	{
		alloc_list = kzalloc(sizeof(struct mem_alloc), GFP_KERNEL);
		if (!alloc_list)
		{
			printk(KERN_ERR "%s:Couldn't create alloc_list\n",
				   FUNC_NAME);
			return NULL;
		}
		alloc_list->instance = 0;
		alloc_list->info = "allocation list - instance 0";
		alloc_list->size = 0;
		alloc_list->pointer = NULL;
		alloc_list->last = NULL;
		alloc_list->prev = NULL;
	}
	// alloc pretended memory
	return_pnt = kzalloc(size, GFP_KERNEL);
	if (!return_pnt)
	{
		printk(KERN_ERR "%s:Couldn't allocate memory: %s\n",
			   FUNC_NAME, info);
		return NULL;
	}
	// alloc memory for the infostructure
	new = kzalloc(sizeof(struct mem_alloc), GFP_KERNEL);
	if (!new)
	{
		printk(KERN_ERR "%s:Couldn't allocate memory for the "
						"alloc_mem\n",
			   FUNC_NAME);
		kfree(return_pnt);
		return NULL;
	}

	new->instance = ++alloc_list->instance;
	new->info = info;
	new->size = size;
	alloc_list->size += size;
	new->pointer = return_pnt;
	if (alloc_list->last == NULL)
	{
		new->prev = alloc_list; // First instance
	}
	else
	{
		new->prev = alloc_list->last;
	}
	alloc_list->last = new;
	new->last = new;

	return return_pnt;
}

/**
 * @short Free all memory
 * This was implemented this way so that all memory allocated was
 * de-allocated and to avoid memory leaks.
 * @return Void
 */
void free_all_mem(void)
{
	if (alloc_list != NULL)
	{
		pr_debug("%s:Total size allocated: %d\n",
				 FUNC_NAME, alloc_list->size);

		while (alloc_list->instance != 0)
		{
			struct mem_alloc *this;
			this = alloc_list->last;
			// cut this from list
			alloc_list->last = this->prev;
			alloc_list->instance--;
			alloc_list->size -= this->size;
			// free allocated memory
			kfree(this->pointer);
			// free this memory
			pr_debug("%s:Freeing: %s\n",
					 FUNC_NAME, this->info);
			kfree(this);
		}
		pr_debug("%s:Total end size: %d\n",
				 FUNC_NAME, alloc_list->size);
		kfree(alloc_list);
		alloc_list = NULL;
	}
}

/**
 * @short Initialization routine - Entry point of the driver
 * @param[in] pdev pointer to the platform device structure
 * @return 0 on success and a negative number on failure
 * Refer to Linux errors.
 */
static int mipi_dsi_init(struct platform_device *pdev)
{
	int ret = 0;
	struct mipi_dsi_dev *dev = NULL;

	pr_info("****************************************\n");
	pr_info("%s:Installing SNPS MIPI DSI module\n", FUNC_NAME);
	pr_info("****************************************\n");

	pr_info("%s:Device registration\n", FUNC_NAME);
	dev = alloc_mem("MIPI DSI Device", sizeof(struct mipi_dsi_dev), NULL);
	if (!dev)
	{
		pr_err("%s:Could not allocated mipi_dsi_dev\n", FUNC_NAME);
		return -ENOMEM;
	}
	// Zero the device
	memset(dev, 0, sizeof(struct mipi_dsi_dev));

	// Update the device node
	dev->parent_dev = &pdev->dev;

	dev->device_name = "MIPI_DSI";
	pr_info("%s:Driver's name '%s'\n", FUNC_NAME, dev->device_name);

	// Map memory blocks
	pr_debug("%s:Map memory blocks\n", FUNC_NAME);
	if (map_memory_blocks(pdev, dev) < 0)
	{
		pr_err("%s:Map memory blocks failed\n", FUNC_NAME);
		goto free_mem;
	}

	dev->rst_1 = devm_reset_control_get_optional_exclusive(&pdev->dev,
														   "rst_dsi");
	if (IS_ERR(dev->rst_1))
	{
		if (PTR_ERR(dev->rst_1) == -EPROBE_DEFER)
			return -EPROBE_DEFER;
	}
	else
	{
		reset_control_deassert(dev->rst_1);
	}

	dev->rst_2 = devm_reset_control_get_optional_exclusive(&pdev->dev,
														   "rst_bridge");
	if (IS_ERR(dev->rst_2))
	{
		if (PTR_ERR(dev->rst_2) == -EPROBE_DEFER)
			return -EPROBE_DEFER;
	}
	else
	{
		reset_control_deassert(dev->rst_2);
	}

	dev->rst_3 = devm_reset_control_get_optional_exclusive(&pdev->dev,
														   "rst_clks");
	if (IS_ERR(dev->rst_3))
	{
		if (PTR_ERR(dev->rst_3) == -EPROBE_DEFER)
			return -EPROBE_DEFER;
	}
	else
	{
		reset_control_deassert(dev->rst_3);
	}

#ifdef PHY_2
	dev->rst_4 = devm_reset_control_get_optional_exclusive(&pdev->dev,
														   "rst_dsi");
	if (IS_ERR(dev->rst_1))
	{
		if (PTR_ERR(dev->rst_1) == -EPROBE_DEFER)
			return -EPROBE_DEFER;
	}
	else
	{
		reset_control_deassert(dev->rst_1);
	}

	dev->rst_5 = devm_reset_control_get_optional_exclusive(&pdev->dev,
														   "rst_bridge");
	if (IS_ERR(dev->rst_2))
	{
		if (PTR_ERR(dev->rst_2) == -EPROBE_DEFER)
			return -EPROBE_DEFER;
	}
	else
	{
		reset_control_deassert(dev->rst_2);
	}
#endif
	// Get IRQ numbers from device tree
	pr_info("%s:Get IRQ numbers\n", FUNC_NAME);
	dev->irq_core = platform_get_irq(pdev, 0);
	if (dev->irq_core <= 0)
	{
		pr_err("%s:IRQ number %d invalid.\n", FUNC_NAME, 0);
	}

	dev->irq_vif = platform_get_irq(pdev, 1);
	if (dev->irq_vif <= 0)
	{
		pr_err("%s:IRQ number %d invalid.\n", FUNC_NAME, 1);
	}

	// Now that everything is fine, let's add it to device list
	list_add_tail(&dev->devlist, &devlist_global);

	// Configure Frame Buffer
	pr_info("%s:Init Frame Buffer\n", FUNC_NAME);
	init_frame_buffer(dev);

	ret = register_framebuffer(&dev->fb.info);
	if (ret < 0)
	{
		pr_err("%s:register framebuffer device failed\n", FUNC_NAME);
		goto free_mem;
	}

	pr_debug("fb %d registered!\n", dev->fb.info.node);

	// Register interrupts
	register_interrupts(dev);

	// INIT
	mipi_dsi_init_param(dev);

	// Configure Video bridge
	video_if_config(dev, SNPS_DSI_DEVICE);
	video_if_data_source(dev, 0x01);
	video_if_data_target(dev, 0x01);

#ifdef GEN_3
	dev->phy.is_g118 = phy_g118;
#endif

	/* Configure platform */
#ifdef VIDEO_MODE
	dsi_platform_init(dev, SNPS_DSI_DEVICE, VIDEO_MODE, 1);
#endif
#ifdef COMMAND_MODE
	dsi_platform_init(dev, SNPS_DSI_DEVICE, COMMAND_MODE, 1);
#endif
	/* Un-mask interrupts */
	mipi_dsih_hal_int_mask_0(dev, 0xffffffff);
	mipi_dsih_hal_int_mask_1(dev, 0xffffffff);

	video_if_adjust_zoom_autocenter(dev);
	video_if_test_mode(dev, 0); /* Configure test mode */
#ifdef START_VPG
	start_video_pattern(dev, 1, 0);
#endif

	return ret;

free_mem:
	release_memory_blocks(dev);
	free_all_mem();

	return ret;
}

/*
 * @short Exit routine - Exit point of the driver
 * @param[in] pdev pointer to the platform device structure
 * @return 0 on success and a negative number on failure
 * Refer to Linux errors.
 */
static int mipi_dsi_exit(struct platform_device *pdev)
{
	struct mipi_dsi_dev *dev;
	struct list_head *list;

	pr_info("**************************************\n");
	pr_info("%s:Removing SNPS MIPI DSI module\n", FUNC_NAME);
	pr_info("**************************************\n");

	while (!list_empty(&devlist_global))
	{
		list = devlist_global.next;
		list_del(list);
		dev = list_entry(list, struct mipi_dsi_dev, devlist);

		pr_info("Unregistering fb %d!\n", dev->fb.info.node);
		mipi_dsih_hal_power(dev, 0);

		// Unregister interrupts
		pr_info("%s:Release interrupts\n", FUNC_NAME);
		release_interrupts(dev);

		// Unregister framebuffer
		pr_info("%s:Unregister framebuffer\n", FUNC_NAME);
		unregister_framebuffer(&dev->fb.info);

		// Release memory blocks
		pr_info("%s:Release memory blocks\n", FUNC_NAME);
		release_memory_blocks(dev);

		dev = NULL;
	}
	free_all_mem();
	return 0;
}

/*
 * @short of_device_id structure
 */
static const struct of_device_id dw_mipi_dsi[] = {
	{.compatible = "snps,dw-mipi-dsi"},
	{}};

MODULE_DEVICE_TABLE(of, dw_mipi_dsi);

/**
 * @short Platform driver structure
 */
static struct platform_driver __refdata dwc_mipi_dsi_pdrv = {
	.remove = mipi_dsi_exit,
	.probe = mipi_dsi_init,
	.driver = {
		.name = "snps,dw-mipi-dsi",
		.owner = THIS_MODULE,
		.of_match_table = dw_mipi_dsi,
	},
};

module_platform_driver(dwc_mipi_dsi_pdrv);
