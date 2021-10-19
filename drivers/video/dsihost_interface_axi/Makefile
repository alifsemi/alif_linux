ccflags-y += -O0 -g -Wall -Wframe-larger-than=4096 \
	-DV_INTERFACE \
	-DDISPLAYS \
	-DGEN_3 \
	-DTESTCHIP \
	-DVESASUPPORT \
	-DVBRIDGE \
	-DCOMMAND_MODE \
	-DAUTOULPS

obj-m += dw-dsih-module.o
dw-dsih-module-objs := dsih_framebuffer.o \
	dsih_video.o \
	dsih_displays.o \
	dsih_api.o \
	dsih_dphy.o \
	dsih_dphy_gen3.o \
	dsih_mmcm.o \
	dsih_dsc.o \
	dsih_hal.o  \
	dsih_core.o

