/*
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.
 * All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 */

#ifdef CONFIG_MTK_BOOT
#include "../../../misc/mediatek/include/mt-plat/mtk_boot_common.h"

static inline bool fts_is_boot_mode_supported(void) {
    return true;
}

static inline bool fts_is_kpoc_boot_mode(void) {
    if (get_boot_mode() == KERNEL_POWER_OFF_CHARGING_BOOT)
        return true;

    return false;
}
#else
static inline bool fts_is_boot_mode_supported(void) {
    return false;
}

static inline bool fts_is_kpoc_boot_mode(void) {
    return false;
}
#endif
