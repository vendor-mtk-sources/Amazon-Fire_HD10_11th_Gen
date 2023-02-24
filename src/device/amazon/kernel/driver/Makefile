#
# Copyright (C) 2020 Amazon Inc.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See http://www.gnu.org/licenses/gpl-2.0.html for more details.
#

mkfile_path := $(abspath $(lastword $(MAKEFILE_LIST)))
cur_dir := $(patsubst %/,%,$(dir $(mkfile_path)))

$(shell mkdir -p $(CURDIR)/include/linux)
$(shell cp $(cur_dir)/include/amzn_*.h $(CURDIR)/include/linux/)

obj-$(CONFIG_AMZN_SIGN_OF_LIFE) += amzn_sign_of_life.o
obj-$(CONFIG_AMZN_SIGN_OF_LIFE_RTC) += amzn_sign_of_life_rtc_impl.o
obj-$(CONFIG_AMZN_METRICS_LOG) += amzn_logger.o
obj-$(CONFIG_AMZN_METRICS_LOG_TEST) += amzn_logger_test.o
obj-$(CONFIG_AMZN_INPUT_KEYCOMBO)	+= amzn_keycombo.o
obj-$(CONFIG_AMZN_IDME) 		+= amzn_idme.o
obj-$(CONFIG_AMZN_DRV_TEST) 		+= amzn_drv_test.o
