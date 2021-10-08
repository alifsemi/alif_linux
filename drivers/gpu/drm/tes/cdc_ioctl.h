/*
 * cdc_ioctl.h  --  CDC Display Controller ioctl hacks
 *
 * Copyright (C) 2017 TES Electronic Solutions GmbH
 * Author: Christian Thaler <christian.thaler@tes-dst.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef CDC_IOCTL_H_
#define CDC_IOCTL_H_

struct hack_set_cb {
	void *phy_addr;
	int width;
	int pitch;
	int height;
};

struct hack_set_winpos {
	int x;
	int y;
	int width;
	int height;
};

struct hack_set_alpha {
	int alpha;
};

#define HACK_IOCTL_BASE                  'h'
#define HACK_IO(nr)                      _IO(HACK_IOCTL_BASE,nr)
#define HACK_IOR(nr,type)                _IOR(HACK_IOCTL_BASE,nr,type)
#define HACK_IOW(nr,type)                _IOW(HACK_IOCTL_BASE,nr,type)
#define HACK_IOWR(nr,type)               _IOWR(HACK_IOCTL_BASE,nr,type)
#define HACK_IOCTL_NR(n)                 _IOC_NR(n)

#define HACK_IOCTL_VERSION               HACK_IO(0x00)

#define HACK_IOCTL_SET_CB                HACK_IOW(0xe0, hack_set_cb)
#define HACK_IOCTL_SET_WINPOS            HACK_IOW(0xe1, hack_set_winpos)
#define HACK_IOCTL_SET_ALPHA             HACK_IOW(0xe2, hack_set_alpha)
#define HACK_IOCTL_WAIT_VSYNC            HACK_IO( 0xe3)

#endif /* CDC_IOCTL_H_ */
