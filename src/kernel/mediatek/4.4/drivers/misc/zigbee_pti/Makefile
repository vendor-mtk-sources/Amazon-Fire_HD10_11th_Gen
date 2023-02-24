#abc123 needs to share PTI UART with MCU logging.
#This is specific to abc123, so the pinmux switching
#logic in zigbee_pti driver must be specific to abc123
ifeq ($(TARGET_PRODUCT), abc123)
	ccflags-$(CONFIG_AMZ_ZIGBEE_PTI)	:= -DPTI_HAS_SHARED_UART_PINS
endif

obj-$(CONFIG_AMZ_ZIGBEE_PTI)	+= zigbee_pti.o