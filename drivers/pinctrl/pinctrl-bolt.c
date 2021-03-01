// SPDX-License-Identifier: GPL-2.0-only
/*
 * Driver for pin control for bolt Alif Semiconductor
 * Heavily based off other pinctrl drivers
 *
 * Copyright (C) 2021 Alif Semiconductor
 *
 * Author: Harith George <harith.g@alifsemi.com>
 */
#include <linux/err.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/slab.h>
#include <linux/regmap.h>

#include "pinctrl-utils.h"
#include "core.h"
#include "pinmux.h"

#define MAX_GPIO_BANKS		4
#define MAX_NB_GPIO_PER_BANK	32

/* Number of io pads that can be configured */
#define BOLT_NUM_IOS	116
/* Port 0 is Analog, Digital starts from Port 1 */
#define BOLT_DIGITAL_IO 20

static const struct pinctrl_pin_desc bolt_pins[] = {
	/* Port 0 */
	PINCTRL_PIN(0, "P0_0"),
	PINCTRL_PIN(1, "P0_1"),
	PINCTRL_PIN(2, "P0_2"),
	PINCTRL_PIN(3, "P0_3"),
	PINCTRL_PIN(4, "P0_4"),
	PINCTRL_PIN(5, "P0_5"),
	PINCTRL_PIN(6, "P0_6"),
	PINCTRL_PIN(7, "P0_7"),
	PINCTRL_PIN(8, "P0_8"),
	PINCTRL_PIN(9, "P0_9"),
	PINCTRL_PIN(10, "P0_10"),
	PINCTRL_PIN(11, "P0_11"),
	PINCTRL_PIN(12, "P0_12"),
	PINCTRL_PIN(13, "P0_13"),
	PINCTRL_PIN(14, "P0_14"),
	PINCTRL_PIN(15, "P0_15"),
	PINCTRL_PIN(16, "P0_16"),
	PINCTRL_PIN(17, "P0_17"),
	PINCTRL_PIN(18, "P0_18"),
	PINCTRL_PIN(19, "P0_19"),
	/* Port 1 */
	PINCTRL_PIN(20, "P1_0"),
	PINCTRL_PIN(21, "P1_1"),
	PINCTRL_PIN(22, "P1_2"),
	PINCTRL_PIN(23, "P1_3"),
	PINCTRL_PIN(24, "P1_4"),
	PINCTRL_PIN(25, "P1_5"),
	PINCTRL_PIN(26, "P1_6"),
	PINCTRL_PIN(27, "P1_7"),
	PINCTRL_PIN(28, "P1_8"),
	PINCTRL_PIN(29, "P1_9"),
	PINCTRL_PIN(30, "P1_10"),
	PINCTRL_PIN(31, "P1_11"),
	PINCTRL_PIN(32, "P1_12"),
	PINCTRL_PIN(33, "P1_13"),
	PINCTRL_PIN(34, "P1_14"),
	PINCTRL_PIN(35, "P1_15"),
	PINCTRL_PIN(36, "P1_16"),
	PINCTRL_PIN(37, "P1_17"),
	PINCTRL_PIN(38, "P1_18"),
	PINCTRL_PIN(39, "P1_19"),
	PINCTRL_PIN(40, "P1_20"),
	PINCTRL_PIN(41, "P1_21"),
	PINCTRL_PIN(42, "P1_22"),
	PINCTRL_PIN(43, "P1_23"),
	PINCTRL_PIN(44, "P1_24"),
	PINCTRL_PIN(45, "P1_25"),
	PINCTRL_PIN(46, "P1_26"),
	PINCTRL_PIN(47, "P1_27"),
	PINCTRL_PIN(48, "P1_28"),
	PINCTRL_PIN(49, "P1_29"),
	PINCTRL_PIN(50, "P1_30"),
	PINCTRL_PIN(51, "P1_31"),
	/* Port 2 */
	PINCTRL_PIN(52, "P2_0"),
	PINCTRL_PIN(53, "P2_1"),
	PINCTRL_PIN(54, "P2_2"),
	PINCTRL_PIN(55, "P2_3"),
	PINCTRL_PIN(56, "P2_4"),
	PINCTRL_PIN(57, "P2_5"),
	PINCTRL_PIN(58, "P2_6"),
	PINCTRL_PIN(59, "P2_7"),
	PINCTRL_PIN(60, "P2_8"),
	PINCTRL_PIN(61, "P2_9"),
	PINCTRL_PIN(62, "P2_10"),
	PINCTRL_PIN(63, "P2_11"),
	PINCTRL_PIN(64, "P2_12"),
	PINCTRL_PIN(65, "P2_13"),
	PINCTRL_PIN(66, "P2_14"),
	PINCTRL_PIN(67, "P2_15"),
	PINCTRL_PIN(68, "P2_16"),
	PINCTRL_PIN(69, "P2_17"),
	PINCTRL_PIN(70, "P2_18"),
	PINCTRL_PIN(71, "P2_19"),
	PINCTRL_PIN(72, "P2_20"),
	PINCTRL_PIN(73, "P2_21"),
	PINCTRL_PIN(74, "P2_22"),
	PINCTRL_PIN(75, "P2_23"),
	PINCTRL_PIN(76, "P2_24"),
	PINCTRL_PIN(77, "P2_25"),
	PINCTRL_PIN(78, "P2_26"),
	PINCTRL_PIN(79, "P2_27"),
	PINCTRL_PIN(80, "P2_28"),
	PINCTRL_PIN(81, "P2_29"),
	PINCTRL_PIN(82, "P2_30"),
	PINCTRL_PIN(83, "P2_31"),
	/* Port 3 */
	PINCTRL_PIN(84, "P3_0"),
	PINCTRL_PIN(85, "P3_1"),
	PINCTRL_PIN(86, "P3_2"),
	PINCTRL_PIN(87, "P3_3"),
	PINCTRL_PIN(88, "P3_4"),
	PINCTRL_PIN(89, "P3_5"),
	PINCTRL_PIN(90, "P3_6"),
	PINCTRL_PIN(91, "P3_7"),
	PINCTRL_PIN(92, "P3_8"),
	PINCTRL_PIN(93, "P3_9"),
	PINCTRL_PIN(94, "P3_10"),
	PINCTRL_PIN(95, "P3_11"),
	PINCTRL_PIN(96, "P3_12"),
	PINCTRL_PIN(97, "P3_13"),
	PINCTRL_PIN(98, "P3_14"),
	PINCTRL_PIN(99, "P3_15"),
	PINCTRL_PIN(100, "P3_16"),
	PINCTRL_PIN(101, "P3_17"),
	PINCTRL_PIN(102, "P3_18"),
	PINCTRL_PIN(103, "P3_19"),
	PINCTRL_PIN(104, "P3_20"),
	PINCTRL_PIN(105, "P3_21"),
	PINCTRL_PIN(106, "P3_22"),
	PINCTRL_PIN(107, "P3_23"),
	/* Port 4 */
	PINCTRL_PIN(108, "P4_0"),
	PINCTRL_PIN(109, "P4_1"),
	PINCTRL_PIN(110, "P4_2"),
	PINCTRL_PIN(111, "P4_3"),
	PINCTRL_PIN(112, "P4_4"),
	PINCTRL_PIN(113, "P4_5"),
	PINCTRL_PIN(114, "P4_6"),
	PINCTRL_PIN(115, "P4_7"),
};


struct bolt_pin {
	unsigned long pin_no;
	unsigned long mux;
	unsigned long padctrl;
	unsigned long testval;
};

struct bolt_pinctrl {
	struct device		*dev;
	struct pinctrl_dev 	*pctl;
	void __iomem 		*pinmux_base;
	void __iomem 		*padctrl_base;
	unsigned int group_index;
	struct mutex mutex;
};

/* Read Enable */
#define BOLT_PINCONF_REN		BIT(0)
/* Schmitt Trigger */
#define BOLT_PINCONF_SCHMITT		BIT(1)
/* Slew Rate [1 = Fast, 0 = Slow (Half rate)] */
#define BOLT_PINCONF_SR			BIT(2)
/* Driver Disabled State Control
 * P2 P1
 *  0  0	Z (Normal operation)
 *  0  1 	Weak 1 (Pull up)
 *  1  0	Weak 0 (Pull down)
 *  1  1	Repeater (Bus keeper)
*/
#define BOLT_DSC_SHIFT			3
#define BOLT_DSC_MASK			0x3
#define BOLT_PINCONF_DSC(x)		((x & BOLT_DSC_MASK) << BOLT_DSC_SHIFT)
#define BOLT_PINCONF_DSC_Z		BOLT_PINCONF_DSC(0)
#define BOLT_PINCONF_DSC_PU		BOLT_PINCONF_DSC(1)
#define BOLT_PINCONF_DSC_PD		BOLT_PINCONF_DSC(2)
#define BOLT_PINCONF_DSC_REP		BOLT_PINCONF_DSC(3)
/* Output Drive Strength
 * E2 E1
 *  0  0	2 mA
 *  0  1 	4 mA
 *  1  0	8 mA
 *  1  1	12 mA
 */
#define BOLT_ODS_SHIFT			5
#define BOLT_ODS_MASK			0x3
#define BOLT_PINCONF_DRIVE_STRENGTH(x)	((x & BOLT_ODS_MASK) << BOLT_ODS_SHIFT)
#define BOLT_PINCONF_DS_2MA		BOLT_PINCONF_DRIVE_STRENGTH(0)
#define BOLT_PINCONF_DS_4MA		BOLT_PINCONF_DRIVE_STRENGTH(1)
#define BOLT_PINCONF_DS_8MA		BOLT_PINCONF_DRIVE_STRENGTH(2)
#define BOLT_PINCONF_DS_12MA		BOLT_PINCONF_DRIVE_STRENGTH(3)

/* DRV [1 = Push Pull, 0 = Open Drain] */
#define BOLT_PINCONF_DRV		BIT(7)
#define BOLT_NO_PAD_CTL  0x80000000

static inline const struct group_desc *bolt_pinctrl_find_group_by_name(
				struct pinctrl_dev *pctldev,
				const char *name)
{
	const struct group_desc *grp = NULL;
	int i;

	for (i = 0; i < pctldev->num_groups; i++) {
		grp = pinctrl_generic_get_group(pctldev, i);
		if (grp && !strcmp(grp->name, name))
			break;
	}

	return grp;
}

static void bolt_pin_dbg_show(struct pinctrl_dev *pctldev, struct seq_file *s,
		   unsigned offset)
{
	seq_printf(s, "%s", dev_name(pctldev->dev));
}

static int bolt_dt_node_to_map(struct pinctrl_dev *pctldev,
			struct device_node *np,
			struct pinctrl_map **map, unsigned *num_maps)
{
	struct bolt_pinctrl *ipctl = pinctrl_dev_get_drvdata(pctldev);
	const struct group_desc *grp;
	struct pinctrl_map *new_map;
	struct device_node *parent;
	struct bolt_pin *pin;
	int map_num = 1;
	int i, j;

	/*
	 * first find the group of this node and check if we need create
	 * config maps for pins
	 */
	grp = bolt_pinctrl_find_group_by_name(pctldev, np->name);
	if (!grp) {
		dev_err(ipctl->dev, "unable to find group for node %pOFn\n", np);
		return -EINVAL;
	}

	for (i = 0; i < grp->num_pins; i++) {
		pin = &((struct bolt_pin *)(grp->data))[i];
		if (!(pin->padctrl & BOLT_NO_PAD_CTL))
			map_num++;
	}

	new_map = kmalloc_array(map_num, sizeof(struct pinctrl_map),
				GFP_KERNEL);
	if (!new_map)
		return -ENOMEM;

	*map = new_map;
	*num_maps = map_num;

	/* create mux map */
	parent = of_get_parent(np);
	if (!parent) {
		kfree(new_map);
		return -EINVAL;
	}
	new_map[0].type = PIN_MAP_TYPE_MUX_GROUP;
	new_map[0].data.mux.function = parent->name;
	new_map[0].data.mux.group = np->name;
	of_node_put(parent);

	/* create config map */
	new_map++;
	for (i = j = 0; i < grp->num_pins; i++) {
		pin = &((struct bolt_pin *)(grp->data))[i];

		/*
		 * We only create config maps for SCU pads or MMIO pads that
		 * are not using the default config(a.k.a IMX_NO_PAD_CTL)
		 */
		if (pin->padctrl & BOLT_NO_PAD_CTL)
			continue;

		new_map[j].type = PIN_MAP_TYPE_CONFIGS_PIN;
		new_map[j].data.configs.group_or_pin =
					pin_get_name(pctldev, pin->pin_no);

		new_map[j].data.configs.configs = &pin->padctrl;
		new_map[j].data.configs.num_configs = 1;

		j++;
	}

	dev_dbg(pctldev->dev, "maps: function %s group %s num %d\n",
		(*map)->data.mux.function, (*map)->data.mux.group, map_num);

	return 0;
}

static void bolt_dt_free_map(struct pinctrl_dev *pctldev,
				struct pinctrl_map *map, unsigned num_maps)
{
	kfree(map);
}

static const struct pinctrl_ops bolt_pctl_ops = {
	.get_groups_count = pinctrl_generic_get_group_count,
	.get_group_name = pinctrl_generic_get_group_name,
	.get_group_pins = pinctrl_generic_get_group_pins,
	.pin_dbg_show = bolt_pin_dbg_show,
	.dt_node_to_map = bolt_dt_node_to_map,
	.dt_free_map = bolt_dt_free_map,
};

static int get_mux_offset(int pin_id, u32 *offset, u32 *bitshift)
{
	u32 portpin;
	u32 off, bs;

	/* GPIO port 0 is in analog domain, and port 4 is VBAT domain */
	if(pin_id < 20){
		return -EINVAL;
	}
	else if(pin_id < 52){
		portpin = pin_id - 20;
		off = 0x10;
	}
	else if(pin_id < 84){
		portpin = pin_id - 52;
		off = 0x20;
	}
	else if(pin_id < 108){
		portpin = pin_id - 84;
		off = 0x30;
	}
	else {
		/* Port 4 in the VBAT domain, pins till 115 */
		//if(pin_id < 116)
		return -EINVAL;
	}
	off += (portpin / 8) * 4;
	bs = (portpin % 8) * 4;

//printk("###HGG: muxoffset %d offset 0x%x, bitshift %d\n", pin_id, off, bs);
	*offset = off;
	*bitshift = bs;

	return 0;
}

static int bolt_pmx_set_one_pin(struct bolt_pinctrl *ipctl,
				    struct bolt_pin *pin)
{
	unsigned int pin_id;
	int ret;
	u32 offset, bitshift;

	pin_id = pin->pin_no;
	ret = get_mux_offset(pin_id, &offset, &bitshift);
	if(ret){
		dev_err(ipctl->dev, "Unsupported pin %d\n",pin_id);
		return -EINVAL;
	}

//printk("###HGG: bolt_pmx_set_one_pin %d, pinmux_base 0x%px, offset 0x%x, bitshift %d mux %ld\n", pin_id, ipctl->pinmux_base, offset, bitshift, pin->mux);
//	val = readl(ipctl->pinmux_base + offset);
//	val |= (pin->mux) << bitshift;
//	writel(pin->mux, ipctl->pinmux_base + offset);

	dev_dbg(ipctl->dev, "write: offset 0x%x val 0x%lx\n",
		offset, pin->mux);

	return 0;
}

static int bolt_pmx_set(struct pinctrl_dev *pctldev, unsigned selector,
		       unsigned group)
{
	struct bolt_pinctrl *ipctl = pinctrl_dev_get_drvdata(pctldev);
	struct function_desc *func;
	struct group_desc *grp;
	struct bolt_pin *pin;
	unsigned int npins;
	int i, err;

	/*
	 * Configure the mux mode for each pin in the group for a specific
	 * function.
	 */
	grp = pinctrl_generic_get_group(pctldev, group);
	if (!grp)
		return -EINVAL;

	func = pinmux_generic_get_function(pctldev, selector);
	if (!func)
		return -EINVAL;

	npins = grp->num_pins;

	dev_dbg(ipctl->dev, "enable function %s group %s\n",
		func->name, grp->name);

	for (i = 0; i < npins; i++) {
		pin = &((struct bolt_pin *)(grp->data))[i];
		err = bolt_pmx_set_one_pin(ipctl, pin);
		if (err)
			return err;
	}
	return 0;
}

struct pinmux_ops bolt_pmx_ops = {
	.get_functions_count = pinmux_generic_get_function_count,
	.get_function_name = pinmux_generic_get_function_name,
	.get_function_groups = pinmux_generic_get_function_groups,
	.set_mux = bolt_pmx_set,
};

static int bolt_pinconf_get_config(struct pinctrl_dev *pctldev, unsigned pin, u32 *val)
{
	//struct bolt_pinctrl *info = pinctrl_dev_get_drvdata(pctldev);
	u32 offset;

	offset = (pin - BOLT_DIGITAL_IO) * 4;
//	*val = readl_relaxed(info->padctrl_base + offset);
//printk("###########HGG pinconf_get addr 0x%px \n", info->padctrl_base + offset);
	*val = 0;

	return 0;
}

static void bolt_pinconf_set_config(struct pinctrl_dev *pctldev, unsigned pin, u32 val)
{
	//struct bolt_pinctrl *info = pinctrl_dev_get_drvdata(pctldev);
	u32 offset;

	offset = (pin - BOLT_DIGITAL_IO) * 4;
//	writel_relaxed(val, info->padctrl_base + offset);
//printk("###########HGG pinconf_set addr 0x%px val 0x%x\n", info->padctrl_base + offset, val);
}

static int bolt_pinconf_set(struct pinctrl_dev *pctldev, unsigned pin_id,
			unsigned long *configs, unsigned num_configs)
{
	int i;
	unsigned long config;
	struct bolt_pinctrl *info = pinctrl_dev_get_drvdata(pctldev);
	unsigned int arg, param, val;

	/* Padconf registers not available for the Analog pins */
	if(pin_id < BOLT_DIGITAL_IO || pin_id >= BOLT_NUM_IOS)
		return -ENOTSUPP;

	/* for each config */
	for (i = 0; i < num_configs; i++) {
		config = configs[i];
		val = 0;
		param = pinconf_to_config_param(config);
		arg = pinconf_to_config_argument(config);
		dev_dbg(info->dev,
			"%s:%d, pin_id=%d, config=0x%lx",
			__func__, __LINE__, pin_id, config);
		switch(param) {
		case PIN_CONFIG_BIAS_BUS_HOLD:
			if(arg) val |= BOLT_PINCONF_DSC_REP;
			break;
		case PIN_CONFIG_BIAS_HIGH_IMPEDANCE:
			if(arg) val |= BOLT_PINCONF_DSC_Z;
			break;
		case PIN_CONFIG_BIAS_PULL_UP:
			if(arg) val |= BOLT_PINCONF_DSC_PU;
			break;
		case PIN_CONFIG_BIAS_PULL_DOWN:
			if(arg) val |= BOLT_PINCONF_DSC_PD;
			break;
		case PIN_CONFIG_INPUT_SCHMITT:
			if(arg) val |= BOLT_PINCONF_SCHMITT;
			break;
		case PIN_CONFIG_DRIVE_OPEN_DRAIN:
			if(arg) val &= ~BOLT_PINCONF_DRV;
			break;
		case PIN_CONFIG_DRIVE_PUSH_PULL:
			if(arg) val |= BOLT_PINCONF_DRV;
			break;
		case PIN_CONFIG_INPUT_ENABLE:
			if(arg) val |= BOLT_PINCONF_REN;
			break;
		case PIN_CONFIG_SLEW_RATE:
			if(arg) val |= BOLT_PINCONF_SR;
			break;
		case PIN_CONFIG_DRIVE_STRENGTH:
			switch(arg){
			case 2:   /* 2mA */
				val = 0;
				break;
			case 4:   /* 4mA */
				val = 1;
				break;
			case 8:   /* 8mA */
				val = 2;
				break;
			case 12:  /* 12mA */
				val = 3;
				break;
			}
			val = (val & BOLT_DSC_MASK) << BOLT_DSC_SHIFT;
			break;
		default:
			return -ENOTSUPP;
		}
		bolt_pinconf_set_config(pctldev, pin_id, val);
	}
	return 0;
}

static int bolt_pinconf_get(struct pinctrl_dev *pctldev,
			     unsigned pin_id, unsigned long *config)
{
	//struct bolt_pinctrl *info = pinctrl_dev_get_drvdata(pctldev);
	unsigned int param = pinconf_to_config_param(*config);
	unsigned int arg = 0;
	u32 reg;
	int ret;

	/* Padconf registers not available for the Analog pins */
	if (pin_id < BOLT_DIGITAL_IO || pin_id >= BOLT_NUM_IOS)
		return -ENOTSUPP;

	ret =  bolt_pinconf_get_config(pctldev, pin_id, &reg);
	if (ret)
		return -EIO;

	switch (param) {
	case PIN_CONFIG_BIAS_BUS_HOLD:
		if (!(reg & BOLT_PINCONF_DSC_REP))
			return -EINVAL;
		arg = 1;
		break;
	case PIN_CONFIG_BIAS_HIGH_IMPEDANCE:
		if (!(reg & BOLT_PINCONF_DSC_Z))
			return -EINVAL;
		arg = 1;
		break;
	case PIN_CONFIG_BIAS_PULL_UP:
		if (!(reg & BOLT_PINCONF_DSC_PU))
			return -EINVAL;
		arg = 1;
		break;
	case PIN_CONFIG_BIAS_PULL_DOWN:
		if (!(reg & BOLT_PINCONF_DSC_PD))
			return -EINVAL;
		arg = 1;
		break;
	case PIN_CONFIG_INPUT_SCHMITT:
		if (!(reg & BOLT_PINCONF_SCHMITT))
			return -EINVAL;
		arg = 1;
		break;
	case PIN_CONFIG_DRIVE_OPEN_DRAIN:
		if (reg & BOLT_PINCONF_DRV)
			return -EINVAL;
		arg = 1;
		break;
	case PIN_CONFIG_DRIVE_PUSH_PULL:
		if (!(reg & BOLT_PINCONF_DRV))
			return -EINVAL;
		arg = 1;
		break;
	case PIN_CONFIG_INPUT_ENABLE:
		if (!(reg & BOLT_PINCONF_REN))
			return -EINVAL;
		arg = 1;
		break;
	case PIN_CONFIG_SLEW_RATE:
		if(!(reg & BOLT_PINCONF_SR))
			return -EINVAL;
		arg = 1;
		break;
	case PIN_CONFIG_DRIVE_STRENGTH:
		reg = (reg >> BOLT_DSC_SHIFT) & BOLT_DSC_MASK;
		switch(reg){
		case 0: /* 2mA */
			arg = 2;
			break;
		case 1: /* 4mA */
			arg = 4;
			break;
		case 2: /* 8mA */
			arg = 8;
			break;
		case 3: /* 12mA */
			arg = 12;
			break;
		}
		break;
	default:
		return -ENOTSUPP;
	}
	*config = pinconf_to_config_packed(param, arg);
	return 0;
}

static void bolt_pinconf_dbg_show(struct pinctrl_dev *pctldev,
				   struct seq_file *s, unsigned pin_id)
{
/*	struct group_desc *grp;
	unsigned long config;
	const char *name;
	//int i, ret;
*/
//	mutex_unlock(&pctldev->mutex);
//	st_pinconf_get(pctldev, pin_id, &config);
//	mutex_lock(&pctldev->mutex);
//	seq_printf();
/*	if (group >= pctldev->num_groups)
		return;

	seq_puts(s, "\n");
	grp = pinctrl_generic_get_group(pctldev, group);
	if (!grp)
		return;
	for (i = 0; i < grp->num_pins; i++) {
		struct imx_pin *pin = &((struct imx_pin *)(grp->data))[i];

		name = pin_get_name(pctldev, pin->pin);
		ret = imx_pinconf_get(pctldev, pin->pin, &config);
		if (ret)
			return;
		seq_printf(s, "  %s: 0x%lx\n", name, config);
	}
*/
}

static const struct pinconf_ops bolt_conf_ops = {
	.pin_config_get		= bolt_pinconf_get,
	.pin_config_set		= bolt_pinconf_set,
	.pin_config_dbg_show	= bolt_pinconf_dbg_show,
};

/*
 * Each pin represented in fsl,pins consists of a number of u32 PIN_FUNC_ID
 * and 1 u32 CONFIG, the total size is PIN_FUNC_ID + CONFIG for each pin.
 * For generic_pinconf case, there's no extra u32 CONFIG.
 *
 * PIN_FUNC_ID format:
 * Default:
 *     <mux_reg conf_reg input_reg mux_mode input_val>
 * SHARE_MUX_CONF_REG:
 *     <mux_conf_reg input_reg mux_mode input_val>
 * IMX_USE_SCU:
 *	<pin_id mux_mode>
 */
#define PIN_SIZE 8

void bolt_pinctrl_parse_pin(struct bolt_pinctrl *info,
				       unsigned int *pin_id, struct bolt_pin *pin,
				       const __be32 **list_p,
				       struct device_node *np)
{
	const __be32 *list = *list_p;
	u32 val;

	val = be32_to_cpu(*list++);

	pin->pin_no = *pin_id = val & 0xffff;
	pin->mux = (val >> 16) & 0xf;
	pin->padctrl = be32_to_cpu(*list++);

//printk("#####HGG: pinctrl_parse_pin pin %ld mux %ld padctrl 0x%lx\n",
//		pin->pin_no, pin->mux, pin->padctrl);

	*list_p = list;

	dev_dbg(info->dev, "%ld: 0x%lx 0x%08lx", pin->pin_no,
			pin->mux, pin->padctrl);
}
EXPORT_SYMBOL_GPL(bolt_pinctrl_parse_pin);


static int bolt_pinctrl_parse_groups(struct device_node *np,
				    struct group_desc *grp,
				    struct bolt_pinctrl *ipctl,
				    u32 index)
{
	struct bolt_pin *pin;
	int size, pin_size;
	const __be32 *list;
	int i;

	dev_dbg(ipctl->dev, "group(%d): %pOFn\n", index, np);

	pin_size = PIN_SIZE;

	/* Initialise group */
	grp->name = np->name;

	list = of_get_property(np, "pinmux", &size);
	if (!list) {
		dev_err(ipctl->dev,
			"no pins property in node %pOF\n", np);
		return -EINVAL;
	}

	/* we do not check return since it's safe node passed down */
	if (!size || size % pin_size) {
		dev_err(ipctl->dev, "Invalid pins property in node %pOF\n", np);
		return -EINVAL;
	}

	grp->num_pins = size / pin_size;
	grp->data = devm_kcalloc(ipctl->dev, grp->num_pins,
				sizeof(struct bolt_pin), GFP_KERNEL);
	grp->pins = devm_kcalloc(ipctl->dev, grp->num_pins,
				sizeof(unsigned int), GFP_KERNEL);
	if (!grp->pins || !grp->data)
		return -ENOMEM;

//printk("##HGG bolt_pinctrl_parse_groups: grp->num_pins %d, size 0x%x pin_size 0x%x\n",
//	 grp->num_pins, size, pin_size);

	for (i = 0; i < grp->num_pins; i++) {
		pin = &((struct bolt_pin *)(grp->data))[i];
		bolt_pinctrl_parse_pin(ipctl, &grp->pins[i], pin, &list, np);
	}

	return 0;
}

static int bolt_pinctrl_probe_dt(struct platform_device *pdev,
				struct bolt_pinctrl *ipctl)
{
	struct device_node *np = pdev->dev.of_node;
	struct device_node *child;
	struct pinctrl_dev *pctl = ipctl->pctl;
	struct function_desc *func;
	struct group_desc *grp;
	u32 i = 0;

	if (!np)
		return -ENODEV;

	func = devm_kzalloc(&pdev->dev, sizeof(*func),
				GFP_KERNEL);
	if (!func)
		return -ENOMEM;

	mutex_lock(&ipctl->mutex);
	radix_tree_insert(&pctl->pin_function_tree, i, func);
	mutex_unlock(&ipctl->mutex);

	pctl->num_functions = 1;
	ipctl->group_index = 0;
	pctl->num_groups = of_get_child_count(np);
	if (pctl->num_groups == 0) {
		dev_err(ipctl->dev, "no groups defined in %pOF\n", np);
		return 0;
	}

	/* Initialise function */
	func->name = np->name;
	func->num_group_names = pctl->num_groups;
	func->group_names = devm_kcalloc(ipctl->dev, func->num_group_names,
					 sizeof(char *), GFP_KERNEL);
	if (!func->group_names)
		return -ENOMEM;

	for_each_child_of_node(np, child) {
		func->group_names[i] = child->name;

		grp = devm_kzalloc(ipctl->dev, sizeof(struct group_desc),
				   GFP_KERNEL);
		if (!grp) {
			of_node_put(child);
			return -ENOMEM;
		}
		mutex_lock(&ipctl->mutex);
		radix_tree_insert(&pctl->pin_group_tree,
				ipctl->group_index++, grp);
		mutex_unlock(&ipctl->mutex);

		bolt_pinctrl_parse_groups(child, grp, ipctl, i++);
	}
	return 0;
}

#if 0
/*
 * bolt_free_resources() - free memory used by this driver
 * @info: info driver instance
 */
static void bolt_free_resources(struct bolt_pinctrl *ipctl)
{
	if (ipctl->pctl)
		pinctrl_unregister(ipctl->pctl);
}
#endif

static int bolt_pctl_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct bolt_pinctrl *info;
	struct pinctrl_desc *pctl_desc;

	if (!pdev->dev.of_node) {
		dev_err(&pdev->dev, "device node not found.\n");
		return -EINVAL;
	}
	pctl_desc = devm_kzalloc(&pdev->dev, sizeof(*pctl_desc), GFP_KERNEL);
	if(!pctl_desc)
	       return -ENOMEM;

	info = devm_kzalloc(&pdev->dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;
	info->dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	info->pinmux_base = devm_ioremap_resource(&pdev->dev, res);
	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	info->padctrl_base = devm_ioremap_resource(&pdev->dev, res);
#if 0
	info->syscon = syscon_regmap_lookup_by_phandle(pdev->dev.of_node,
							"syscon");
	if (IS_ERR(info->syscon)) {
		dev_err(&pdev->dev, "unable to get syscon\n");
		return PTR_ERR(info->syscon);
	}
#endif
	platform_set_drvdata(pdev, info);

	pctl_desc->name		= dev_name(&pdev->dev);
	pctl_desc->pins		= bolt_pins;
	pctl_desc->npins	= ARRAY_SIZE(bolt_pins);
	pctl_desc->owner	= THIS_MODULE;
	pctl_desc->pctlops	= &bolt_pctl_ops;
	pctl_desc->pmxops	= &bolt_pmx_ops;
	pctl_desc->confops	= &bolt_conf_ops;

	info->pctl = devm_pinctrl_register(&pdev->dev, pctl_desc, info);
	if (IS_ERR(info->pctl)){
		dev_err(&pdev->dev, "Failed pinctrl registration\n");
		return PTR_ERR(info->pctl);
	}
	if(bolt_pinctrl_probe_dt(pdev, info)){
		dev_err(&pdev->dev, "fail to probe dt properties\n");
	}

	dev_info(&pdev->dev, "Bolt pinctrl initialized\n");
	return 0;
}

static const struct of_device_id bolt_pctl_of_match[] = {
	{ .compatible = "alif,pinctrl-bolt" },
	{ }
};


static struct platform_driver bolt_pctl_driver = {
	.driver = {
		.name = "bolt-pinctrl",
		.of_match_table = bolt_pctl_of_match,
	},
	.probe = bolt_pctl_probe,
};


static int __init bolt_pctl_init(void){

	return platform_driver_register(&bolt_pctl_driver);
}
arch_initcall(bolt_pctl_init);

