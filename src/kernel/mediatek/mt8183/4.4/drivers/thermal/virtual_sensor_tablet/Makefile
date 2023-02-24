# SPDX-License-Identifier: GPL-2.0
#
# virtual sensor thermal specific Makefile
#
THERMAL_CHIP_DRIVER_DIR := $(srctree)/drivers/misc/mediatek/thermal/$(MTK_PLATFORM)
ccflags-y  += -I$(THERMAL_CHIP_DRIVER_DIR)/inc
ccflags-y  += -I$(srctree)/drivers/misc/mediatek/base/power/$(MTK_PLATFORM)
ccflags-y  += -I$(srctree)/drivers/misc/mediatek/gpu/hal/
ccflags-y  += -I$(srctree)/drivers/misc/mediatek/pmic/include/
ccflags-y  += -I$(srctree)/drivers/misc/mediatek/thermal/common/inc/
ccflags-y  += -I$(srctree)/drivers/misc/mediatek/include/
ccflags-y  += -I$(srctree)/drivers/misc/mediatek/include/mt-plat
ccflags-y  += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include/
ccflags-y  += -I$(srctree)/drivers/thermal/

# platform thermal drivers
obj-y += virtual_sensor_tablet_thermal.o

# platform interface
obj-y += virtual_sensor_interface.o

# governors
obj-y += virtual_sensor_gov.o

# cooling devices
obj-y += virtual_sensor_cooler.o

#thermal sensors
obj-y += virtual_sensor_thermal_sensor.o
