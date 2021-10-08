cdc-y := cdc_drv.o \
         cdc_kms.o \
         cdc_crtc.o \
         cdc_plane.o \
         cdc_lvdscon.o \
         cdc_encoder.o \
         cdc_hw.o \
         cdc_hw_helpers.o \
         cdc_deswizzle.o
ccflags-$(CONFIG_DRM_TES_CDC) += -DDISABLE_ASSERTIONS -DDEBUG

obj-$(CONFIG_DRM_TES_CDC) += cdc.o

