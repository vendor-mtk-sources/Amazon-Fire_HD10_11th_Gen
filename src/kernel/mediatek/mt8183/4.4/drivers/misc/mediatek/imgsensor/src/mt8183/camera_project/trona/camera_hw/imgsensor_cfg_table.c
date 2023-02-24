/*
 * Copyright (C) 2017 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include "kd_imgsensor.h"

#include "mclk/mclk.h"
#include "regulator/regulator.h"
#include "gpio/gpio.h"

#include "imgsensor_hw.h"
#include "imgsensor_cfg_table.h"

enum IMGSENSOR_RETURN (*hw_open[IMGSENSOR_HW_ID_MAX_NUM]) (struct IMGSENSOR_HW_DEVICE **) = {
imgsensor_hw_mclk_open, imgsensor_hw_regulator_open, imgsensor_hw_gpio_open};

struct IMGSENSOR_HW_CFG imgsensor_custom_config[] = {
	{
	 IMGSENSOR_SENSOR_IDX_MAIN,
	 IMGSENSOR_I2C_DEV_0,
	 {
	  {IMGSENSOR_HW_PIN_MCLK, IMGSENSOR_HW_ID_MCLK},
	  {IMGSENSOR_HW_PIN_AVDD, IMGSENSOR_HW_ID_GPIO},
	  {IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
	  {IMGSENSOR_HW_PIN_DVDD, IMGSENSOR_HW_ID_REGULATOR},
	  {IMGSENSOR_HW_PIN_PDN, IMGSENSOR_HW_ID_GPIO},
	  {IMGSENSOR_HW_PIN_RST, IMGSENSOR_HW_ID_GPIO},
	  {IMGSENSOR_HW_PIN_NONE, IMGSENSOR_HW_ID_NONE},
	  },
	 },
	{
	 IMGSENSOR_SENSOR_IDX_SUB,
	 IMGSENSOR_I2C_DEV_1,
	 {
	  {IMGSENSOR_HW_PIN_MCLK, IMGSENSOR_HW_ID_MCLK},
	  {IMGSENSOR_HW_PIN_AVDD, IMGSENSOR_HW_ID_REGULATOR},
	  {IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
	  {IMGSENSOR_HW_PIN_DVDD, IMGSENSOR_HW_ID_REGULATOR},
	  {IMGSENSOR_HW_PIN_PDN, IMGSENSOR_HW_ID_GPIO},
	  {IMGSENSOR_HW_PIN_RST, IMGSENSOR_HW_ID_GPIO},
	  {IMGSENSOR_HW_PIN_NONE, IMGSENSOR_HW_ID_NONE},
	  },
	 },
	{
	 IMGSENSOR_SENSOR_IDX_MAIN2,
	 IMGSENSOR_I2C_DEV_1,
	 {
	  {IMGSENSOR_HW_PIN_MCLK, IMGSENSOR_HW_ID_MCLK},
	  {IMGSENSOR_HW_PIN_AVDD, IMGSENSOR_HW_ID_REGULATOR},
	  {IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
	  {IMGSENSOR_HW_PIN_DVDD, IMGSENSOR_HW_ID_GPIO},
	  {IMGSENSOR_HW_PIN_PDN, IMGSENSOR_HW_ID_GPIO},
	  {IMGSENSOR_HW_PIN_RST, IMGSENSOR_HW_ID_GPIO},
	  {IMGSENSOR_HW_PIN_NONE, IMGSENSOR_HW_ID_NONE},
	  },
	 },
	{
	 IMGSENSOR_SENSOR_IDX_SUB2,
	 IMGSENSOR_I2C_DEV_1,
	 {
	  {IMGSENSOR_HW_PIN_MCLK, IMGSENSOR_HW_ID_MCLK},
	  {IMGSENSOR_HW_PIN_AVDD, IMGSENSOR_HW_ID_REGULATOR},
	  {IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
	  {IMGSENSOR_HW_PIN_DVDD, IMGSENSOR_HW_ID_GPIO},
	  {IMGSENSOR_HW_PIN_PDN, IMGSENSOR_HW_ID_GPIO},
	  {IMGSENSOR_HW_PIN_RST, IMGSENSOR_HW_ID_GPIO},
	  {IMGSENSOR_HW_PIN_NONE, IMGSENSOR_HW_ID_NONE},
	  },
	 },

	{IMGSENSOR_SENSOR_IDX_NONE}
};

struct IMGSENSOR_HW_CFG imgsensor_custom_config_later[] = {
	{
	 IMGSENSOR_SENSOR_IDX_MAIN,
	 IMGSENSOR_I2C_DEV_0,
	 {
	  {IMGSENSOR_HW_PIN_MCLK, IMGSENSOR_HW_ID_MCLK},
	  {IMGSENSOR_HW_PIN_AVDD, IMGSENSOR_HW_ID_REGULATOR},
	  {IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
	  {IMGSENSOR_HW_PIN_DVDD, IMGSENSOR_HW_ID_REGULATOR},
	  {IMGSENSOR_HW_PIN_PDN, IMGSENSOR_HW_ID_GPIO},
	  {IMGSENSOR_HW_PIN_RST, IMGSENSOR_HW_ID_GPIO},
	  {IMGSENSOR_HW_PIN_NONE, IMGSENSOR_HW_ID_NONE},
	  },
	 },
	{
	 IMGSENSOR_SENSOR_IDX_SUB,
	 IMGSENSOR_I2C_DEV_1,
	 {
	  {IMGSENSOR_HW_PIN_MCLK, IMGSENSOR_HW_ID_MCLK},
	  {IMGSENSOR_HW_PIN_AVDD, IMGSENSOR_HW_ID_REGULATOR},
	  {IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
	  {IMGSENSOR_HW_PIN_DVDD, IMGSENSOR_HW_ID_REGULATOR},
	  {IMGSENSOR_HW_PIN_PDN, IMGSENSOR_HW_ID_GPIO},
	  {IMGSENSOR_HW_PIN_RST, IMGSENSOR_HW_ID_GPIO},
	  {IMGSENSOR_HW_PIN_NONE, IMGSENSOR_HW_ID_NONE},
	  },
	 },
	{
	 IMGSENSOR_SENSOR_IDX_MAIN2,
	 IMGSENSOR_I2C_DEV_1,
	 {
	  {IMGSENSOR_HW_PIN_MCLK, IMGSENSOR_HW_ID_MCLK},
	  {IMGSENSOR_HW_PIN_AVDD, IMGSENSOR_HW_ID_REGULATOR},
	  {IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
	  {IMGSENSOR_HW_PIN_DVDD, IMGSENSOR_HW_ID_GPIO},
	  {IMGSENSOR_HW_PIN_PDN, IMGSENSOR_HW_ID_GPIO},
	  {IMGSENSOR_HW_PIN_RST, IMGSENSOR_HW_ID_GPIO},
	  {IMGSENSOR_HW_PIN_NONE, IMGSENSOR_HW_ID_NONE},
	  },
	 },
	{
	 IMGSENSOR_SENSOR_IDX_SUB2,
	 IMGSENSOR_I2C_DEV_1,
	 {
	  {IMGSENSOR_HW_PIN_MCLK, IMGSENSOR_HW_ID_MCLK},
	  {IMGSENSOR_HW_PIN_AVDD, IMGSENSOR_HW_ID_REGULATOR},
	  {IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
	  {IMGSENSOR_HW_PIN_DVDD, IMGSENSOR_HW_ID_GPIO},
	  {IMGSENSOR_HW_PIN_PDN, IMGSENSOR_HW_ID_GPIO},
	  {IMGSENSOR_HW_PIN_RST, IMGSENSOR_HW_ID_GPIO},
	  {IMGSENSOR_HW_PIN_NONE, IMGSENSOR_HW_ID_NONE},
	  },
	 },

	{IMGSENSOR_SENSOR_IDX_NONE}
};

struct IMGSENSOR_HW_POWER_SEQ platform_power_sequence[] = {
#ifdef MIPI_SWITCH
	{
	 IMGSENSOR_SENSOR_IDX_NAME_SUB,
	 {
	  {
	   IMGSENSOR_HW_PIN_MIPI_SWITCH_EN,
	   IMGSENSOR_HW_PIN_STATE_LEVEL_0,
	   0,
	   IMGSENSOR_HW_PIN_STATE_LEVEL_HIGH,
	   0},
	  {
	   IMGSENSOR_HW_PIN_MIPI_SWITCH_SEL,
	   IMGSENSOR_HW_PIN_STATE_LEVEL_HIGH,
	   0,
	   IMGSENSOR_HW_PIN_STATE_LEVEL_0,
	   0},
	  }
	 },
	{
	 IMGSENSOR_SENSOR_IDX_NAME_MAIN2,
	 {
	  {
	   IMGSENSOR_HW_PIN_MIPI_SWITCH_EN,
	   IMGSENSOR_HW_PIN_STATE_LEVEL_0,
	   0,
	   IMGSENSOR_HW_PIN_STATE_LEVEL_HIGH,
	   0},
	  {
	   IMGSENSOR_HW_PIN_MIPI_SWITCH_SEL,
	   IMGSENSOR_HW_PIN_STATE_LEVEL_0,
	   0,
	   IMGSENSOR_HW_PIN_STATE_LEVEL_0,
	   0},
	  }
	 },
#endif

	{NULL}
};

/* Legacy design */
struct IMGSENSOR_HW_POWER_SEQ sensor_power_sequence[] = {
#if defined(HI556_MIPI_RAW)
	{
	 SENSOR_DRVNAME_HI556_MIPI_RAW,
	 {
	  {RST, Vol_Low, 0, Vol_Low},
	  {AVDD, Vol_Low, 0, Vol_Low},
	  {DOVDD, Vol_1800, 1, Vol_Low},
	  {AVDD, Vol_High, 5, Vol_Low},
	  {DVDD, Vol_1200, 5, Vol_Low},
	  {SensorMCLK, Vol_High, 0, Vol_Low},
	  {RST, Vol_High, 2, Vol_Low}
	  },
	 },
#endif
#if defined(HI556SEC_MIPI_RAW)
	{
	 SENSOR_DRVNAME_HI556SEC_MIPI_RAW,
	 {
	  {RST, Vol_Low, 0, Vol_Low},
	  {AVDD, Vol_Low, 0, Vol_Low},
	  {DOVDD, Vol_1800, 1, Vol_Low},
	  {AVDD, Vol_High, 5, Vol_Low},
	  {DVDD, Vol_1200, 5, Vol_Low},
	  {SensorMCLK, Vol_High, 0, Vol_Low},
	  {RST, Vol_High, 2, Vol_Low}
	  },
	 },
#endif
#if defined(GC5035_MIPI_RAW)
	{
	 SENSOR_DRVNAME_GC5035_MIPI_RAW,
	 {
	  {RST, Vol_Low, 0, Vol_Low},
	  {AVDD, Vol_Low, 0, Vol_Low},
	  {DOVDD, Vol_1800, 1, Vol_Low},
	  {DVDD, Vol_1200, 5, Vol_Low},
	  {AVDD, Vol_High, 5, Vol_Low},
	  {SensorMCLK, Vol_High, 0, Vol_Low},
	  {RST, Vol_High, 2, Vol_Low}
	  },
	 },
#endif
#if defined(GC5035SEC_MIPI_RAW)
	{
	 SENSOR_DRVNAME_GC5035SEC_MIPI_RAW,
	 {
	  {RST, Vol_Low, 0, Vol_Low},
	  {AVDD, Vol_Low, 0, Vol_Low},
	  {DOVDD, Vol_1800, 1, Vol_Low},
	  {DVDD, Vol_1200, 5, Vol_Low},
	  {AVDD, Vol_High, 5, Vol_Low},
	  {SensorMCLK, Vol_High, 0, Vol_Low},
	  {RST, Vol_High, 2, Vol_Low}
	  },
	 },
#endif
#if defined(OV02B_MIPI_RAW)
	{
	 SENSOR_DRVNAME_OV02B_MIPI_RAW,
	 {
	  {RST, Vol_Low, 0, Vol_Low},
	  {PDN, Vol_Low, 0, Vol_Low},
	  {DOVDD, Vol_1800, 1, Vol_Low},
	  {AVDD, Vol_2800, 5, Vol_Low},
	  {SensorMCLK, Vol_High, 0, Vol_Low},
	  {PDN, Vol_High, 4, Vol_Low},
	  {RST, Vol_High, 2, Vol_Low}
	  },
	 },
#endif
#if defined(OV02BSEC_MIPI_RAW)
	{
	 SENSOR_DRVNAME_OV02BSEC_MIPI_RAW,
	 {
	  {RST, Vol_Low, 0, Vol_Low},
	  {PDN, Vol_Low, 0, Vol_Low},
	  {DOVDD, Vol_1800, 1, Vol_Low},
	  {AVDD, Vol_2800, 5, Vol_Low},
	  {SensorMCLK, Vol_High, 0, Vol_Low},
	  {PDN, Vol_High, 4, Vol_Low},
	  {RST, Vol_High, 2, Vol_Low}
	  },
	 },
#endif
#if defined(GC02M1TR_MIPI_RAW)
	{
	 SENSOR_DRVNAME_GC02M1TR_MIPI_RAW,
	 {
	  {RST, Vol_Low, 0, Vol_Low},
	  {DOVDD, Vol_1800, 1, Vol_Low},
	  {AVDD, Vol_2800, 5, Vol_Low},
	  {RST, Vol_High, 2, Vol_Low},
	  {SensorMCLK, Vol_High, 0, Vol_Low}
	  },
	 },
#endif
#if defined(GC02M1TRSEC_MIPI_RAW)
	{
	 SENSOR_DRVNAME_GC02M1TRSEC_MIPI_RAW,
	 {
	  {RST, Vol_Low, 0, Vol_Low},
	  {DOVDD, Vol_1800, 1, Vol_Low},
	  {AVDD, Vol_2800, 5, Vol_Low},
	  {RST, Vol_High, 2, Vol_Low},
	  {SensorMCLK, Vol_High, 0, Vol_Low}
	  },
	 },
#endif

	/* add new sensor before this line */
	{NULL,},
};

struct IMGSENSOR_HW_POWER_SEQ sensor_power_sequence_later[] = {
#if defined(HI556_MIPI_RAW)
	{
	 SENSOR_DRVNAME_HI556_MIPI_RAW,
	 {
	  {RST, Vol_Low, 0, Vol_Low},
	  {DOVDD, Vol_1800, 1, Vol_Low},
	  {AVDD, Vol_2800, 5, Vol_Low},
	  {DVDD, Vol_1200, 5, Vol_Low},
	  {SensorMCLK, Vol_High, 0, Vol_Low},
	  {RST, Vol_High, 2, Vol_Low}
	  },
	 },
#endif
#if defined(HI556SEC_MIPI_RAW)
	{
	 SENSOR_DRVNAME_HI556SEC_MIPI_RAW,
	 {
	  {RST, Vol_Low, 0, Vol_Low},
	  {DOVDD, Vol_1800, 1, Vol_Low},
	  {AVDD, Vol_2800, 5, Vol_Low},
	  {DVDD, Vol_1200, 5, Vol_Low},
	  {SensorMCLK, Vol_High, 0, Vol_Low},
	  {RST, Vol_High, 2, Vol_Low}
	  },
	 },
#endif
#if defined(GC5035_MIPI_RAW)
	{
	 SENSOR_DRVNAME_GC5035_MIPI_RAW,
	 {
	  {RST, Vol_Low, 0, Vol_Low},
	  {DOVDD, Vol_1800, 1, Vol_Low},
	  {DVDD, Vol_1200, 5, Vol_Low},
	  {AVDD, Vol_2800, 5, Vol_Low},
	  {SensorMCLK, Vol_High, 0, Vol_Low},
	  {RST, Vol_High, 2, Vol_Low}
	  },
	 },
#endif
#if defined(GC5035SEC_MIPI_RAW)
	{
	 SENSOR_DRVNAME_GC5035SEC_MIPI_RAW,
	 {
	  {RST, Vol_Low, 0, Vol_Low},
	  {DOVDD, Vol_1800, 1, Vol_Low},
	  {DVDD, Vol_1200, 5, Vol_Low},
	  {AVDD, Vol_2800, 5, Vol_Low},
	  {SensorMCLK, Vol_High, 0, Vol_Low},
	  {RST, Vol_High, 2, Vol_Low}
	  },
	 },
#endif
#if defined(OV02B_MIPI_RAW)
	{
	 SENSOR_DRVNAME_OV02B_MIPI_RAW,
	 {
	  {RST, Vol_Low, 0, Vol_Low},
	  {PDN, Vol_Low, 0, Vol_Low},
	  {DOVDD, Vol_1800, 1, Vol_Low},
	  {AVDD, Vol_2800, 5, Vol_Low},
	  {SensorMCLK, Vol_High, 0, Vol_Low},
	  {PDN, Vol_High, 4, Vol_Low},
	  {RST, Vol_High, 2, Vol_Low}
	  },
	 },
#endif
#if defined(OV02BSEC_MIPI_RAW)
	{
	 SENSOR_DRVNAME_OV02BSEC_MIPI_RAW,
	 {
	  {RST, Vol_Low, 0, Vol_Low},
	  {PDN, Vol_Low, 0, Vol_Low},
	  {DOVDD, Vol_1800, 1, Vol_Low},
	  {AVDD, Vol_2800, 5, Vol_Low},
	  {SensorMCLK, Vol_High, 0, Vol_Low},
	  {PDN, Vol_High, 4, Vol_Low},
	  {RST, Vol_High, 2, Vol_Low}
	  },
	 },
#endif
#if defined(GC02M1TR_MIPI_RAW)
	{
	 SENSOR_DRVNAME_GC02M1TR_MIPI_RAW,
	 {
	  {RST, Vol_Low, 0, Vol_Low},
	  {DOVDD, Vol_1800, 1, Vol_Low},
	  {AVDD, Vol_2800, 5, Vol_Low},
	  {RST, Vol_High, 2, Vol_Low},
	  {SensorMCLK, Vol_High, 0, Vol_Low}
	  },
	 },
#endif
#if defined(GC02M1TRSEC_MIPI_RAW)
	{
	 SENSOR_DRVNAME_GC02M1TRSEC_MIPI_RAW,
	 {
	  {RST, Vol_Low, 0, Vol_Low},
	  {DOVDD, Vol_1800, 1, Vol_Low},
	  {AVDD, Vol_2800, 5, Vol_Low},
	  {RST, Vol_High, 2, Vol_Low},
	  {SensorMCLK, Vol_High, 0, Vol_Low}
	  },
	 },
#endif

	/* add new sensor before this line */
	{NULL,},
};
