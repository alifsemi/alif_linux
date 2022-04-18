// SPDX-License-Identifier: GPL-2.0
/*
 * Counter driver for the Alif Semiconductor utimer IP
 * Copyright (C) 2022 Harith George
 *
 * This driver supports the utimer counter.
 */
#include <linux/bitops.h>
#include <linux/counter.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/iio/iio.h>
#include <linux/iio/types.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/types.h>
#include <linux/platform_device.h>

#define UT_NUM_COUNTERS 	16

struct utimer_cnt {
	struct counter_device counter;
	struct platform_device *pdev;
	void __iomem *base;
};
#define UT_CNTR_CTRL		0x0

#define CNTR_EN			BIT(0)
#define CNTR_START		BIT(1)
#define CNTR_TYPE_SHIFT		2
#define CNTR_DIR		BIT(8)

#define UT_START_SRC		0x4
#define UT_STOP_SRC		0x8
#define UT_CLEAR_SRC		0xc
#define UT_UP_SRC		0x10
#define UT_DOWN_SRC		0x14
#define UT_TRIG_CAPTURE_SRC_A	0x30
#define UT_TRIG_CAPTURE_SRC_B	0x34
#define UT_DMA_CLEAR_SRC_A	0x38
#define UT_DMA_CLEAR_SRC_B	0x3c
#define UT_CMP_CTRL_A		0x50
#define UT_CMP_CTRL_B		0x54
#define UT_BUF_OP_CTRL		0x58
#define UT_CNTR			0x80
#define UT_CNTR_PTR		0x84
#define UT_CNTR_PTR_BUF1	0x88
#define UT_CNTR_PTR_BUF2	0x8c
#define UT_CPTR_A		0x90
#define UT_CPTR_A_BUF1		0x94
#define UT_CPTR_A_BUF2		0x98
#define UT_CPTR_B		0xa0
#define UT_CPTR_B_BUF1		0xa4
#define UT_CPTR_B_BUF2		0xa8
#define UT_CMP_A		0xb0
#define UT_CMP_A_BUF1		0xb4
#define UT_CMP_A_BUF2		0xb8
#define UT_CMP_B		0xc0
#define UT_CMP_B_BUF1		0xc4
#define UT_CMP_B_BUF2		0xc8
#define UT_DT_UP_0		0xd0
#define UT_DT_UPBUF1_0		0xd4
#define UT_DT_DOWN_0		0xd8
#define UT_DT_DOWNBUF1_0	0xdc
#define UT_CHAN_STATUS		0xf4

#define UT_CAPTURE_A		BIT(0)
#define UT_CAPTURE_B		BIT(1)
#define UT_OVERFLOW		BIT(6)
#define UT_UNDERFLOW		BIT(7)
#define UT_CNTR_RUNNING		BIT(14)
#define UT_CNTR_DIR		BIT(15)
#define UT_COMPARE_A_UP		BIT(16)
#define UT_COMPARE_A_DOWN	BIT(17)
#define UT_COMPARE_B_UP		BIT(18)
#define UT_COMPARE_B_DOWN	BIT(19)
#define UT_DRV_A_B_1		BIT(29)
#define UT_DRV_A_B_0		BIT(30)

#define UT_CHAN_INT		0xf8
#define UT_CHAN_INT_MASK	0xfc
#define UT_DUTY_CYCLE_CTRL	0x100

#define DC_ENABLE_A		BIT(0)
#define DC_FORCE_A		BIT(1)
#define DC_OVERFLOW_A		BIT(4)
#define DC_UNDERFLOW_A		BIT(5)

#define DC_ENABLE_B		BIT(8)
#define DC_FORCE_B		BIT(9)
#define DC_OVERFLOW_B		BIT(12)
#define DC_UNDERFLOW_B		BIT(13)

#define UT_DEAD_TIME_CTRL	0x104

#define DT_EN			BIT(0)
#define DT_BUF_EN		BIT(1)

#define UT_GLB_CNTR_START	0x110
#define UT_GLB_CNTR_STOP	0x114
#define UT_GLB_CNTR_CLEAR	0x118
#define UT_GLB_CNTR_RUNNING	0x11c
#define UT_GLB_DRIVER_OEN	0x128


enum utimer_count_function {
	UT_COUNT_FUNCTION_INCREASE = 0,
	UT_COUNT_FUNCTION_DECREASE,
};

static enum counter_count_function utimer_count_functions_list[] = {
	[UT_COUNT_FUNCTION_INCREASE] = COUNTER_COUNT_FUNCTION_INCREASE,
	[UT_COUNT_FUNCTION_DECREASE] = COUNTER_COUNT_FUNCTION_DECREASE,
};

enum utimer_synapse_action {
	UT_SYN_ACT_NONE = 0,
	UT_SYN_ACT_RISING_EDGE,
	UT_SYN_ACT_FALLING_EDGE,
	UT_SYN_ACT_BOTH_EDGES,
	UT_SYN_ACT_A_RISING_B_0,
	UT_SYN_ACT_A_RISING_B_1,
	UT_SYN_ACT_A_FALLING_B_0,
	UT_SYN_ACT_A_FALLING_B_1,
	UT_SYN_ACT_B_RISING_A_0,
	UT_SYN_ACT_B_RISING_A_1,
	UT_SYN_ACT_B_FALLING_A_0,
	UT_SYN_ACT_B_FALLING_A_1,
};

enum ut_count_mode {
	UT_CNT_MODE_SAWTOOTH = 0,
	UT_CNT_MODE_TRIANGLE,
};

static int ut_get_mode(struct counter_device *counter,
	struct counter_count *count)
{
	const int id = (count->id + 1);
	struct utimer_cnt *ut = counter->priv;
	void __iomem *base = ut->base + 0x1000 * id;
	unsigned int mode, ctrl;

	ctrl = readl(base + UT_CNTR_CTRL);
	mode = (ctrl >> 2) & 0x7;
	/* 000: sawtooth
	   001: sawtooth one shot
	   100: triangle (buf at trough)
	   101: triangle (buf at trough and crest)
	   110: triangle (fixed buf op) */

	return mode;
}

static int ut_get_dir(struct counter_device *counter,
			struct counter_count *count)
{
	struct utimer_cnt *ut = counter->priv;
	void __iomem *base = ut->base + 0x1000 * (count->id + 1);
	unsigned int ud_flag, dir;

	/* U/D flag: 0 = up, 1 = down */
	ud_flag = readl(base + UT_CNTR_CTRL) & CNTR_DIR;

	dir = (ud_flag) ? COUNTER_COUNT_DIRECTION_BACKWARD :
			COUNTER_COUNT_DIRECTION_FORWARD;
	return dir;
}

static int ut_get_ptr(struct counter_device *counter,
			struct counter_count *count)
{
	struct utimer_cnt *ut = counter->priv;
	void __iomem *base = ut->base + 0x1000 * (count->id + 1);
	unsigned int ptr;

	/* Get Counter Pointer Register */
	ptr = readl(base + UT_CNTR_PTR);

	return ptr;
}

static int ut_count_read(struct counter_device *counter,
	struct counter_count *count, struct counter_count_read_value *val)
{
	struct utimer_cnt *ut = counter->priv;
	void __iomem *base = ut->base + 0x1000 * (count->id + 1);
	unsigned int cntr;
	unsigned long current_cnt;

	cntr = readl(base + UT_CNTR);
	current_cnt = (unsigned long) cntr;
	counter_count_read_value_set(val, COUNTER_COUNT_POSITION, &current_cnt);

	return 0;
}

static int ut_count_write(struct counter_device *counter,
	struct counter_count *count, struct counter_count_write_value *val)
{
	struct utimer_cnt *ut = counter->priv;
	void __iomem *base = ut->base + 0x1000 * (count->id + 1);
	unsigned int ctrl, err;
	unsigned long start;

	/* Stop counter before writing value*/
	ctrl = readl(base + UT_CNTR_CTRL);
	ctrl &= ~CNTR_START;
	writel(ctrl, base + UT_CNTR_CTRL);

	err = counter_count_write_value_get(&start, COUNTER_COUNT_POSITION,
					    val);
	if (err)
		return err;

	/* Set counter start value */
	writel((unsigned int)start, base + UT_CNTR);
	return 0;
}

static int ut_function_get(struct counter_device *counter,
	struct counter_count *count, size_t *function)
{
	unsigned int mode;
	enum counter_count_direction dir;

	mode = ut_get_mode(counter, count);
	dir = ut_get_dir(counter, count);

	if(dir == COUNTER_COUNT_DIRECTION_FORWARD)
		*function = UT_COUNT_FUNCTION_INCREASE;
	else
		*function = UT_COUNT_FUNCTION_DECREASE;

	return 0;
}

static int ut_function_set(struct counter_device *counter,
	struct counter_count *count, size_t function)
{
	return 0;
}

static int ut_action_get(struct counter_device *counter,
	struct counter_count *count, struct counter_synapse *synapse,
	size_t *action)
{
	size_t function = 0;
	int err;

	err = ut_function_get(counter, count, &function);
	if (err)
		return err;

	/* Default action mode */
	*action = UT_SYN_ACT_NONE;

	/* Determine action mode based on current count function mode */
	switch (function) {
	case UT_COUNT_FUNCTION_INCREASE:
		*action = UT_SYN_ACT_FALLING_EDGE;
		break;
	case UT_COUNT_FUNCTION_DECREASE:
		*action = UT_SYN_ACT_BOTH_EDGES;
		break;
	}

	return 0;
}

static const struct counter_ops utimer_ops = {
	.count_read = ut_count_read,
	.count_write = ut_count_write,
	.function_get = ut_function_get,
	.function_set = ut_function_set,
	.action_get = ut_action_get,
};

static int utimer_count_mode_get(struct counter_device *counter,
	struct counter_count *count, size_t *cnt_mode)
{
	unsigned int mode;

	mode = ut_get_mode(counter, count);

	/* Map count mode to Generic Counter count mode */
	switch (mode) {
	case 0:
		*cnt_mode = COUNTER_COUNT_MODE_NORMAL;
		break;
	case 1:
		*cnt_mode = COUNTER_COUNT_MODE_RANGE_LIMIT;
		break;
	case 2:
		*cnt_mode = COUNTER_COUNT_MODE_NON_RECYCLE;
		break;
	case 3:
		*cnt_mode = COUNTER_COUNT_MODE_MODULO_N;
		break;
	}
	return 0;
}

static int utimer_count_mode_set(struct counter_device *counter,
	struct counter_count *count, size_t cnt_mode)
{
	struct utimer_cnt *ut = counter->priv;
	void __iomem *base = ut->base + 0x1000 * (count->id + 1);
	unsigned int val;

	/* TODO: Map the modes to the right mode */
	/* Write mode to Counter Control Register */
	val = readl(base + UT_CNTR_CTRL);
	val |= cnt_mode << CNTR_TYPE_SHIFT;
	writel(val, base + UT_CNTR_CTRL);

	return 0;
}

static struct counter_count_enum_ext utimer_cnt_mode_enum = {
	.items = counter_count_mode_str,
	.num_items = ARRAY_SIZE(counter_count_mode_str),
	.get = utimer_count_mode_get,
	.set = utimer_count_mode_set
};

static ssize_t utimer_count_direction_read(struct counter_device *counter,
	struct counter_count *count, void *priv, char *buf)
{
	enum counter_count_direction dir;

	dir = ut_get_dir(counter, count);

	return sprintf(buf, "%s\n", counter_count_direction_str[dir]);
}

static ssize_t utimer_count_enable_read(struct counter_device *counter,
	struct counter_count *count, void *private, char *buf)
{
	struct utimer_cnt *ut = counter->priv;
	void __iomem *base = ut->base + 0x1000 * (count->id + 1);
	unsigned int val;

	/* Get Counter Pointer Register */
	val = readl(base + UT_CNTR_CTRL);
	val &= BIT(0);

	return sprintf(buf, "%u\n", val);
}

static ssize_t utimer_count_enable_write(struct counter_device *counter,
	struct counter_count *count, void *private, const char *buf, size_t len)
{
	struct utimer_cnt *ut = counter->priv;
	void __iomem *base = ut->base + 0x1000 * (count->id + 1);
	unsigned int enable, val;
	int ret;

	ret = kstrtouint(buf, 0, &enable);
	if (ret)
		return ret;
	enable &= BIT(0);

	/* Set enable value to bit 0 of Counter Control Register */
	val = readl(base + UT_CNTR_CTRL);
	val |= enable;
	writel(enable, base + UT_CNTR_CTRL);

	return len;
}

static ssize_t utimer_count_ptr_read(struct counter_device *counter,
	struct counter_count *count, void *private, char *buf)
{
	return sprintf(buf, "%u\n", ut_get_ptr(counter, count));
}

static ssize_t utimer_count_ptr_write(struct counter_device *counter,
	struct counter_count *count, void *private, const char *buf, size_t len)
{
	struct utimer_cnt *ut = counter->priv;
	void __iomem *base = ut->base + 0x1000 * (count->id + 1);
	unsigned int ptr;
	int ret;

	ret = kstrtouint(buf, 0, &ptr);
	if (ret)
		return ret;

	/* TODO: Set count register to be below the PTR value ??*/
	/* Set Counter Pointer Register */
	writel(ptr, base + UT_CNTR_PTR);

	return len;
}

static ssize_t utimer_count_running_read(struct counter_device *counter,
	struct counter_count *count, void *private, char *buf)
{
	struct utimer_cnt *ut = counter->priv;
	int channel = count->id;
	void __iomem *base = ut->base;
	unsigned int val;
	int running = 0;

	val = readl(base + UT_GLB_CNTR_RUNNING);
	if(val & (1 << channel))
		running = 1;

	return sprintf(buf, "%u\n", running);
}

static ssize_t utimer_count_running_write(struct counter_device *counter,
	struct counter_count *count, void *private, const char *buf, size_t len)
{
	struct utimer_cnt *ut = counter->priv;
	int channel = count->id;
	void __iomem *base = ut->base;
	int err;
	unsigned int val;
	bool running;

	err = kstrtobool(buf, &running);
	if (err)
		return err;

	if(running){
		/* Start the corresponding channel counter */
		val = readl(base + UT_GLB_CNTR_START);
		val |= (1 << channel);
		writel(val, base + UT_GLB_CNTR_START);
	}
	else {
		/* Stop the corresponding channel counter */
		val = readl(base + UT_GLB_CNTR_STOP);
		val |= (1 << channel);
		writel(val, base + UT_GLB_CNTR_STOP);
	}
	return len;
}


#define	UT_EVENTS_IN_FOR_CHANNEL(ch) 		\
	{					\
	.id = 12 + ((ch) * 2),			\
	.name = "Channel " #ch " event A"	\
	},					\
	{					\
	.id = 12 + ((ch) * 2) + 1,		\
	.name = "Channel " #ch " event B"	\
	}

static struct counter_signal utimer_signals[] = {
	{ .id = 0, .name = "trigger 0" }, /* qec0_z */
	{ .id = 1, .name = "trigger 1" }, /* qec1_z */
	{ .id = 2, .name = "trigger 2" }, /* qec2_z */
	{ .id = 3, .name = "trigger 3" }, /* qec3_z */
	{ .id = 4, .name = "glb events out 3" },
	{ .id = 5, .name = "glb events out 2" },
	{ .id = 6, .name = "glb events out 1" },
	{ .id = 7, .name = "glb events out 0" },
	{ .id = 8, .name = "comp4_filter_out" },
	{ .id = 9, .name = "comp3_filter_out" },
	{ .id = 10, .name = "comp2_filter_out" },
	{ .id = 11, .name = "comp1_filter_out" },
	UT_EVENTS_IN_FOR_CHANNEL(0),
	UT_EVENTS_IN_FOR_CHANNEL(1),
	UT_EVENTS_IN_FOR_CHANNEL(2),
	UT_EVENTS_IN_FOR_CHANNEL(3),
	UT_EVENTS_IN_FOR_CHANNEL(4),
	UT_EVENTS_IN_FOR_CHANNEL(5),
	UT_EVENTS_IN_FOR_CHANNEL(6),
	UT_EVENTS_IN_FOR_CHANNEL(7),
	UT_EVENTS_IN_FOR_CHANNEL(8),
	UT_EVENTS_IN_FOR_CHANNEL(9),
	UT_EVENTS_IN_FOR_CHANNEL(10),
	UT_EVENTS_IN_FOR_CHANNEL(11),
	UT_EVENTS_IN_FOR_CHANNEL(12),
	UT_EVENTS_IN_FOR_CHANNEL(13),
	UT_EVENTS_IN_FOR_CHANNEL(14),
	UT_EVENTS_IN_FOR_CHANNEL(15),
};

static enum counter_synapse_action ut_synapse_trigger_actions[] = {
	[UT_SYN_ACT_NONE] = COUNTER_SYNAPSE_ACTION_NONE,
	[UT_SYN_ACT_RISING_EDGE] = COUNTER_SYNAPSE_ACTION_RISING_EDGE,
	[UT_SYN_ACT_FALLING_EDGE] = COUNTER_SYNAPSE_ACTION_FALLING_EDGE,
	[UT_SYN_ACT_BOTH_EDGES] = COUNTER_SYNAPSE_ACTION_BOTH_EDGES
};

static enum counter_synapse_action ut_synapse_chan_event_actions[] = {
	[UT_SYN_ACT_NONE] = COUNTER_SYNAPSE_ACTION_NONE,
	[UT_SYN_ACT_A_RISING_B_0] = COUNTER_SYNAPSE_ACTION_BOTH_EDGES,
	[UT_SYN_ACT_A_RISING_B_1] = COUNTER_SYNAPSE_ACTION_BOTH_EDGES,
	[UT_SYN_ACT_A_FALLING_B_0] = COUNTER_SYNAPSE_ACTION_BOTH_EDGES,
	[UT_SYN_ACT_A_FALLING_B_1] = COUNTER_SYNAPSE_ACTION_BOTH_EDGES,
	[UT_SYN_ACT_B_RISING_A_0] = COUNTER_SYNAPSE_ACTION_BOTH_EDGES,
	[UT_SYN_ACT_B_RISING_A_1] = COUNTER_SYNAPSE_ACTION_BOTH_EDGES,
	[UT_SYN_ACT_B_FALLING_A_0] = COUNTER_SYNAPSE_ACTION_BOTH_EDGES,
	[UT_SYN_ACT_B_FALLING_A_1] = COUNTER_SYNAPSE_ACTION_BOTH_EDGES,
};

static enum counter_synapse_action ut_synapse_glb_event_actions[] = {
	[UT_SYN_ACT_NONE] = COUNTER_SYNAPSE_ACTION_NONE,
	[UT_SYN_ACT_RISING_EDGE] = COUNTER_SYNAPSE_ACTION_RISING_EDGE,
};

/* The same 4 trigger input signals are shared between all
 * the 16 channels.
 */
#define UT_COUNT_SYNAPSE_TRIGGER(id) {					\
	.actions_list = ut_synapse_trigger_actions,			\
	.num_actions = ARRAY_SIZE(ut_synapse_trigger_actions),		\
	.signal = utimer_signals + id					\
}

#define UT_COUNT_SYNAPSE_GLB_EVENT(id) {				\
	.actions_list = ut_synapse_glb_event_actions,			\
	.num_actions = ARRAY_SIZE(ut_synapse_glb_event_actions),	\
	.signal = utimer_signals + id					\
}

#define UT_COUNT_SYNAPSE_CHAN_EVENT(ch) {				\
	.actions_list = ut_synapse_chan_event_actions,			\
	.num_actions = ARRAY_SIZE(ut_synapse_chan_event_actions),	\
	.signal = utimer_signals + 12 + (2 * ch)			\
}

#define UT_COUNT_SYNAPSES_CHANNEL(ch)	{	\
	UT_COUNT_SYNAPSE_TRIGGER(0), UT_COUNT_SYNAPSE_TRIGGER(1),	\
	UT_COUNT_SYNAPSE_TRIGGER(2), UT_COUNT_SYNAPSE_TRIGGER(3),	\
	UT_COUNT_SYNAPSE_GLB_EVENT(4), UT_COUNT_SYNAPSE_GLB_EVENT(5),	\
	UT_COUNT_SYNAPSE_GLB_EVENT(6), UT_COUNT_SYNAPSE_GLB_EVENT(7),	\
	UT_COUNT_SYNAPSE_GLB_EVENT(8), UT_COUNT_SYNAPSE_GLB_EVENT(9),	\
	UT_COUNT_SYNAPSE_GLB_EVENT(10), UT_COUNT_SYNAPSE_GLB_EVENT(11),	\
	UT_COUNT_SYNAPSE_CHAN_EVENT(ch),				\
}

static struct counter_synapse utimer_count_synapses[][13] = {
	UT_COUNT_SYNAPSES_CHANNEL(0),
	UT_COUNT_SYNAPSES_CHANNEL(1),
	UT_COUNT_SYNAPSES_CHANNEL(2),
	UT_COUNT_SYNAPSES_CHANNEL(3),
	UT_COUNT_SYNAPSES_CHANNEL(4),
	UT_COUNT_SYNAPSES_CHANNEL(5),
	UT_COUNT_SYNAPSES_CHANNEL(6),
	UT_COUNT_SYNAPSES_CHANNEL(7),
	UT_COUNT_SYNAPSES_CHANNEL(8),
	UT_COUNT_SYNAPSES_CHANNEL(9),
	UT_COUNT_SYNAPSES_CHANNEL(10),
	UT_COUNT_SYNAPSES_CHANNEL(11),
	UT_COUNT_SYNAPSES_CHANNEL(12),
	UT_COUNT_SYNAPSES_CHANNEL(13),
	UT_COUNT_SYNAPSES_CHANNEL(14),
	UT_COUNT_SYNAPSES_CHANNEL(15),
};

static const struct counter_count_ext utimer_count_ext[] = {
	{
		.name = "ceiling",
		.read = utimer_count_ptr_read,
		.write = utimer_count_ptr_write
	},
	COUNTER_COUNT_ENUM("count_mode", &utimer_cnt_mode_enum),
	COUNTER_COUNT_ENUM_AVAILABLE("count_mode", &utimer_cnt_mode_enum),
	{
		.name = "direction",
		.read = utimer_count_direction_read
	},
	{
		.name = "enable",
		.read = utimer_count_enable_read,
		.write = utimer_count_enable_write
	},
	{
		.name = "running",
		.read = utimer_count_running_read,
		.write = utimer_count_running_write
	},
};

#define UTIMER_COUNT(_id, _cntname) {					\
	.id = (_id),							\
	.name = (_cntname),						\
	.functions_list = utimer_count_functions_list,			\
	.num_functions = ARRAY_SIZE(utimer_count_functions_list),	\
	.synapses = utimer_count_synapses[(_id)],			\
	.num_synapses =	13, /* 4 trig + 1 Quad channel + 8 glb event */	\
	.ext = utimer_count_ext,					\
	.num_ext = ARRAY_SIZE(utimer_count_ext)				\
}

static struct counter_count utimer_counts[] = {
	UTIMER_COUNT(0, "Channel 1"),
	UTIMER_COUNT(1, "Channel 2"),
	UTIMER_COUNT(2, "Channel 3"),
	UTIMER_COUNT(3, "Channel 4"),
	UTIMER_COUNT(4, "Channel 5"),
	UTIMER_COUNT(5, "Channel 6"),
	UTIMER_COUNT(6, "Channel 7"),
	UTIMER_COUNT(7, "Channel 8"),
	UTIMER_COUNT(8, "Channel 9"),
	UTIMER_COUNT(9, "Channel 10"),
	UTIMER_COUNT(10, "Channel 11"),
	UTIMER_COUNT(11, "Channel 12"),
	UTIMER_COUNT(12, "Channel 13"),
	UTIMER_COUNT(13, "Channel 14"),
	UTIMER_COUNT(14, "Channel 15"),
	UTIMER_COUNT(15, "Channel 16"),
};

static int utimer_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct utimer_cnt *ut;
	struct resource *io;
	void __iomem *base;
	int i;

	/* This also allocates driver data structure */
	ut = devm_kzalloc(dev, sizeof(*ut), GFP_KERNEL);
	if (!ut)
		return -ENOMEM;

	platform_set_drvdata(pdev, ut);
	io = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!io) {
		dev_err(dev, "Failed to get memory region\n");
		return -ENODEV;
	}
	ut->pdev = pdev;
	ut->base = devm_ioremap(dev, io->start, resource_size(io));
	if (IS_ERR(ut->base))
		return PTR_ERR(ut->base);

	/* Initialize Counter device and driver data */
	ut->counter.name = dev_name(dev);
	ut->counter.parent = dev;
	ut->counter.ops = &utimer_ops;
	ut->counter.counts = utimer_counts;
	ut->counter.num_counts = ARRAY_SIZE(utimer_counts);
	ut->counter.signals = utimer_signals;
	ut->counter.num_signals = ARRAY_SIZE(utimer_signals);
	ut->counter.priv = ut;

	for (i = 0; i < UT_NUM_COUNTERS; i++) {
		base = ut->base + 0x1000 * (i + 1);
		/* Enable starting/stopping/clearing via
		 * programming (ie via processor writes)
		 * to each channel */
		writel(0x80000000, base + UT_START_SRC);
		writel(0x80000000, base + UT_STOP_SRC);
		writel(0x80000000, base + UT_CLEAR_SRC);
		writel(0x0, base + UT_CHAN_INT_MASK);
	}
	/* Turn off all channel output enables */
	writel(0xffffffff, ut->base + UT_GLB_DRIVER_OEN);

	/* Register Counter device */
	return devm_counter_register(dev, &ut->counter);
}

static const struct of_device_id utimer_of_match[] = {
	{ .compatible = "alif,alif-utimer", },
	{ /* sentinel */},
};
MODULE_DEVICE_TABLE(of, utimer_of_match);

static struct platform_driver utimer_driver = {
	.probe = utimer_probe,
	.driver = {
		.name = "utimer",
		.of_match_table = utimer_of_match,
	},
};
module_platform_driver(utimer_driver);

MODULE_AUTHOR("Harith George <harith.g@alifsemi.com>");
MODULE_DESCRIPTION("Alif utimer driver");
MODULE_LICENSE("GPL v2");
