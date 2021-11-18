/*
* Copyright (C) 2015 MediaTek Inc.
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.
* If not, see <http://www.gnu.org/licenses/>.
*/


/*******************************************************************************
 *
 * Filename:
 * ---------
 *   AudDrv_Gpio.h
 *
 * Project:
 * --------
 *   MT6797Audio Driver Gpio control
 *
 * Description:
 * ------------
 *   Audio gpio control
 *
 * Author:
 * -------
 *   George
 *
 *------------------------------------------------------------------------------
 *
 *
 *******************************************************************************/

#ifndef _AUDDRV_GPIO_H_
#define _AUDDRV_GPIO_H_

/*****************************************************************************
 *                     C O M P I L E R   F L A G S
 *****************************************************************************/


/*****************************************************************************
 *                E X T E R N A L   R E F E R E N C E S
 *****************************************************************************/
#include "mtk-auddrv-def.h"
#include "mtk-soc-digital-type.h"

/*****************************************************************************
 *                         D A T A   T Y P E S
 *****************************************************************************/


/*****************************************************************************
 *                         M A C R O
 *****************************************************************************/


/*****************************************************************************
 *                 FUNCTION       D E F I N I T I O N
 *****************************************************************************/
#include <linux/gpio.h>

// macros defined for EXT AMP
#define EXT_AMP_CTRL0_BIT (0x1 << 0)
#define EXT_AMP_CTRL1_BIT (0x1 << 1)
#define EXT_AMP_CTRL_MASK (0xF)

#define GPIO_EXT_AMP_ON(base)        (base)
#define GPIO_EXT_AMP_OFF(base)       (base + 1)
#define GPIO_EXT_AMP_CTRL0_ON(base)  (base + 2)
#define GPIO_EXT_AMP_CTRL0_OFF(base) (base + 3)
#define GPIO_EXT_AMP_CTRL1_ON(base)  (base + 4)
#define GPIO_EXT_AMP_CTRL1_OFF(base) (base + 5)
#define GPIO_EXT_AMP_CTRLP_ON(base)  (base + 6)
#define GPIO_EXT_AMP_CTRLP_OFF(base) (base + 7)

void AudDrv_GPIO_probe(void *dev);

int AudDrv_GPIO_Request(bool _enable, enum soc_aud_digital_block _usage);

int AudDrv_GPIO_SMARTPA_Select(int mode);
int AudDrv_GPIO_TDM_Select(int mode);

int AudDrv_GPIO_I2S_Select(int bEnable);
int AudDrv_GPIO_EXTAMP_Select(int bEnable, int mode);
int AudDrv_GPIO_EXTAMP2_Select(int bEnable, int mode);
int AudDrv_GPIO_EXT_SPK_AMP_Select(int bEnable, int mode);
int AudDrv_GPIO_EXT_HP_AMP_Select(int bEnable, int mode);
int AudDrv_GPIO_RCVSPK_Select(int bEnable);
int AudDrv_GPIO_HPDEPOP_Select(int bEnable);

int audio_drv_gpio_aud_clk_pull(bool high);
#endif
