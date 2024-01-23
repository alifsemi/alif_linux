// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Defines machines for Alif Ensemble
 *
 * Copyright (c) 2023 Alif Semiconductor
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/sizes.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <linux/of.h>
#include <linux/of_platform.h>

static const char *const ensemble_dt_match[] __initconst = {
	"alif,ensemble",
	NULL
};

DT_MACHINE_START(ENSEMBLE_DT, "Alif Ensemble")
	/* Maintainer: Harith George <harith.g@alifsemi.com> */
	.dt_compat      = ensemble_dt_match,
MACHINE_END
