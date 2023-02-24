#
# Makefile for Bosch sensor driver.
#

obj-$(CONFIG_BOSCH_DRIVER_LOG_FUNC) += bs_log.o
ifeq ($(CONFIG_BOSCH_DRIVER_LOG_FUNC),y)
	EXTRA_CFLAGS += -DBOSCH_DRIVER_LOG_FUNC
endif

obj-$(CONFIG_SENSORS_BMA456)    += bma4xy_driver.o bma4.o bma456.o bma4xy_i2c.o
