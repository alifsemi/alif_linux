/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright (C) 2023 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 * Author: Yogender Kumar Arya <yogender.kumar@alifsemi.com>
 */

#ifndef _UAPI_ALIF_DRM_H_
#define _UAPI_ALIF_DRM_H_

#include "drm.h"

#if defined(__cplusplus)
extern "C" {
#endif

struct drm_d2d_create_bo {
	__u32 size;
	__u32 flags;
	/** Returned GEM handle for the BO. */
	__u32 handle;
	/** Physical address of contiguous buffer. */
	__u32 paddr;
};

struct drm_d2d_mmap_bo {
	/** Handle for the object being mapped. */
	__u32 handle;
	__u32 flags;
	/** offset into the drm node to use for subsequent mmap call. */
	__u64 offset;
};

struct drm_d2d_reg {
	/** Offset of register w.r.t the base. */
	__u32 offset;
	/** Value of the register. */
	__u32 value;
};

enum {
	DRM_D2D_GET_PARAMS_CLK = 0,
};

struct drm_d2d_get_params {
	__u32 param_id;
	__u32 value;
};

/*
 * IOCTL types
 */
enum {
	DRM_D2D_GEM_CREATE_BO = 0,
	DRM_D2D_GEM_MMAP_BO,
	DRM_D2D_READ_REG,
	DRM_D2D_WRITE_REG,
	DRM_D2D_GET_PARAMS,
};

#define DRM_IOCTL_D2D_GEM_CREATE_BO \
	DRM_IOWR(DRM_COMMAND_BASE + DRM_D2D_GEM_CREATE_BO, \
		struct drm_d2d_create_bo)
#define DRM_IOCTL_D2D_GEM_MMAP_BO \
	DRM_IOWR(DRM_COMMAND_BASE + DRM_D2D_GEM_MMAP_BO, \
		struct drm_d2d_mmap_bo)
#define DRM_IOCTL_D2D_READ_REG	\
	DRM_IOWR(DRM_COMMAND_BASE + DRM_D2D_READ_REG, struct drm_d2d_reg)
#define DRM_IOCTL_D2D_WRITE_REG	\
	DRM_IOWR(DRM_COMMAND_BASE + DRM_D2D_WRITE_REG, struct drm_d2d_reg)
#define DRM_IOCTL_D2D_GET_PARAMS \
	DRM_IOWR(DRM_COMMAND_BASE + DRM_D2D_GET_PARAMS, \
		struct drm_d2d_get_params)

#if defined(__cplusplus)
}
#endif

#endif /* _UAPI_ALIF_DRM_H_ */
