MTK_PLATFORM := $(subst ",,$(CONFIG_MTK_PLATFORM))
subdir-ccflags-y += -Werror -I$(srctree)/drivers/misc/mediatek/include
subdir-ccflags-y += -Werror -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include
subdir-ccflags-y += -Werror -I$(srctree)/drivers/misc/mediatek/include/mt-plat
subdir-ccflags-y += -Werror -I$(srctree)/drivers/misc/mediatek/usb_c/tcpc/inc

obj-$(CONFIG_AMAZON_SIGN_OF_LIFE) += sign_of_life.o
obj-$(CONFIG_AMAZON_SIGN_OF_LIFE_MTK) += sign_of_life_mtk.o
ifeq ($(or $(strip $(CONFIG_AMAZON_METRICS_LOG)), $(strip $(CONFIG_AMAZON_MINERVA_METRICS_LOG))),y)
	obj-y += amazon_logger.o
endif
obj-$(CONFIG_AMAZON_METRICS_LOG_TEST)	+= amazon_logger_test.o
obj-$(CONFIG_INPUT_AMZN_KEYCOMBO)	+= amzn_keycombo.o
obj-$(CONFIG_AMAZON_INTEGRITY_MODULE)	+= amzn_integrity_module.o
obj-$(CONFIG_AMAZON_LD) += amzn_ld.o
obj-$(CONFIG_AMAZON_LD_SWITCH) += amzn_ld_switch_init.o
obj-$(CONFIG_AMZN_RT_TRACE)	+= amzn_rt_trace.o
obj-$(CONFIG_AMAZON_DSP_FRAMEWORK) += dspframework/
