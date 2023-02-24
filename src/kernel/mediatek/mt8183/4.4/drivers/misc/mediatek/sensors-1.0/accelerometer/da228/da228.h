/* Core header for MiraMEMS 3-Axis Accelerometer's driver.
 *
 * da228.h - Linux kernel modules for MiraMEMS 3-Axis Accelerometer
 *
 * Copyright (C) 2020 MiraMEMS Sensing Technology Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __DA228_CORE_H__
#define __DA228_CORE_H__

#define CORE_VER                            "1.0.0_2020-08-28-14:56:30"
#define DA228_BUFSIZE                      256
#define DA228_STK_TEMP_SOLUTION            0
#define DA228_OFFSET_TEMP_SOLUTION         0
#if DA228_OFFSET_TEMP_SOLUTION
#define DA228_AUTO_CALIBRATE               0
#else
#define DA228_AUTO_CALIBRATE               0
#endif /* !DA228_OFFSET_TEMP_SOLUTION */
#if DA228_AUTO_CALIBRATE
#define DA228_SUPPORT_FAST_AUTO_CALI       0
#else
#define DA228_SUPPORT_FAST_AUTO_CALI       0
#endif
#define DA228_SUPPORT_MULTI_LAYOUT         0
#define YZ_CROSS_TALK_ENABLE               0
#define DA228_OFFSET_LEN                   9
#define DA228_OFFSET_MAX                   200
#define DA228_OFFSET_CUS                   130
#define DA228_OFFSET_SEN                   1024

/* Register define for NSA asic */
#define NSA_REG_SPI_I2C                 0x00
#define NSA_REG_WHO_AM_I                0x01
#define NSA_REG_ACC_X_LSB               0x02
#define NSA_REG_ACC_X_MSB               0x03
#define NSA_REG_ACC_Y_LSB               0x04
#define NSA_REG_ACC_Y_MSB               0x05
#define NSA_REG_ACC_Z_LSB               0x06
#define NSA_REG_ACC_Z_MSB               0x07
#define NSA_REG_G_RANGE                 0x0f
#define NSA_REG_ODR_AXIS_DISABLE        0x10
#define NSA_REG_POWERMODE_BW            0x11
#define NSA_REG_SWAP_POLARITY           0x12
#define NSA_REG_FIFO_CTRL               0x14
#define NSA_REG_INTERRUPT_SETTINGS1     0x16
#define NSA_REG_INTERRUPT_SETTINGS2     0x17
#define NSA_REG_INTERRUPT_MAPPING1      0x19
#define NSA_REG_INTERRUPT_MAPPING2      0x1a
#define NSA_REG_INTERRUPT_MAPPING3      0x1b
#define NSA_REG_INT_PIN_CONFIG          0x20
#define NSA_REG_INT_LATCH               0x21
#define NSA_REG_ACTIVE_DURATION         0x27
#define NSA_REG_ACTIVE_THRESHOLD        0x28
#define NSA_REG_TAP_DURATION            0x2A
#define NSA_REG_TAP_THRESHOLD           0x2B
#define NSA_REG_CUSTOM_OFFSET_X         0x38
#define NSA_REG_CUSTOM_OFFSET_Y         0x39
#define NSA_REG_CUSTOM_OFFSET_Z         0x3a
#define NSA_REG_ENGINEERING_MODE        0x7f
#define NSA_REG_SENSITIVITY_TRIM_X      0x80
#define NSA_REG_SENSITIVITY_TRIM_Y      0x81
#define NSA_REG_SENSITIVITY_TRIM_Z      0x82
#define NSA_REG_COARSE_OFFSET_TRIM_X    0x83
#define NSA_REG_COARSE_OFFSET_TRIM_Y    0x84
#define NSA_REG_COARSE_OFFSET_TRIM_Z    0x85
#define NSA_REG_FINE_OFFSET_TRIM_X      0x86
#define NSA_REG_FINE_OFFSET_TRIM_Y      0x87
#define NSA_REG_FINE_OFFSET_TRIM_Z      0x88
#define NSA_REG_SENS_COMP               0x8c
#define NSA_REG_MEMS_OPTION             0x8f
#define NSA_REG_CHIP_INFO               0xc0
#define NSA_REG_CHIP_INFO_SECOND        0xc1
#define NSA_REG_MEMS_OPTION_SECOND      0xc7
#define NSA_REG_SENS_COARSE_TRIM        0xd1

#define DA228_ODR_50HZ                  0x06
#define DA228_ODR_125HZ                 0x07
#define DA228_ODR_250HZ                 0x08

#define DA228_RANGE_4G                  0x01
#define DA228_BW_1_2                    0x3e
#define DA228_SW_RESET                  0x24
#define DA228_X_Y_SWAP                  0x01
#define DA228_Z_POLARITY_REVERSE        0x02
#define DA228_Y_POLARITY_REVERSE        0x04
#define DA228_X_POLARITY_REVERSE        0x08


/*----------------------------------------------------------------------------*/
extern char *idme_get_sensorcal(void);

#endif    /* __DA228_CORE_H__ */


