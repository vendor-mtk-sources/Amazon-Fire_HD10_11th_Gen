#
# Makefile for misc devices that really don't fit anywhere else.
#


obj-y += jd936x_wxga_dsi_vdo.o

ccflags-$(CONFIG_MTK_LCM) += -I$(srctree)/drivers/misc/mediatek/lcm/inc
ccflags-$(CONFIG_MTK_LEDS) += -I$(srctree)/drivers/misc/mediatek/leds/ \
				-I$(srctree)/drivers/misc/mediatek/leds/$(MTK_PLATFORM)