/* SPDX-License-Identifier: GPL-2.0-only */
/*
 *  arch/arm/mach-ensemble/include/mach/memory.h
 *
 *  Copyright (C) 2023 Alif Semiconductor
 *
 *  Changelog:
 *   31-Aug-2023 Harith George  Initial addition
 */
#ifndef __ASM_ARCH_MEMORY_H
#define __ASM_ARCH_MEMORY_H

/*
 * Sparsemem support.  Each section is 8MB.
 * Max addressable range for A32 is 0xE000_0000.
 * But still that is 32 physmem bits.
 */
#define MAX_PHYSMEM_BITS	32
#define SECTION_SIZE_BITS	24

#endif
