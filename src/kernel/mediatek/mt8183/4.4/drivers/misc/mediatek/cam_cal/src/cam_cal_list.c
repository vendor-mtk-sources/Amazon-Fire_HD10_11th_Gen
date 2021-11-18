/*
* Copyright (C) 2016 MediaTek Inc.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
* See http://www.gnu.org/licenses/gpl-2.0.html for more details.
*/
#include <linux/kernel.h>

#include <linux/of.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include "cam_cal_list.h"
#include "kd_imgsensor.h"

/*Common EEPRom Driver*/
#include "common/BRCB032GWZ_3/BRCB032GWZ_3.h"
#include "common/cat24c16/cat24c16.h"
#include "common/GT24c32a/GT24c32a.h"


#define CAM_CAL_DEBUG
#ifdef CAM_CAL_DEBUG
/*#include <linux/log.h>*/
#include <linux/kern_levels.h>
#define PFX "cam_cal_list"

#define CAM_CALINF(format, args...)     pr_info(PFX "[%s] " format, __func__, ##args)
#define CAM_CALDB(format, args...)      pr_debug(PFX "[%s] " format, __func__, ##args)
#define CAM_CALERR(format, args...)     pr_info(format, ##args)
#else
#define CAM_CALINF(x, ...)
#define CAM_CALDB(x, ...)
#define CAM_CALERR(x, ...)
#endif


#define MTK_MAX_CID_NUM 4
unsigned int mtkCidList[MTK_MAX_CID_NUM] = {
	0x010b00ff,/*Single MTK Format*/
	0x020b00ff,/*Double MTK Format in One OTP/EEPRom - Legacy*/
	0x030b00ff,/*Double MTK Format in One OTP/EEPRom*/
	0x040b00ff /*Double MTK Format in One OTP/EEPRom V1.4*/
};

#define IDME_OF_FRONT_CAM_OTP "/idme/front_cam_otp"
#define IDME_OF_REAR_CAM_OTP "/idme/rear_cam_otp"

/* otp data is 2048 bytes at most,
  but needs to save as string, so need 2048*2=4096bytes */
#define CAM_OTP_DATA_LEN 2048

/* mtk otp data length */
#define CAM_MTK_OTP_LEN 1895
unsigned char front_cam_otp[CAM_OTP_DATA_LEN];
unsigned char rear_cam_otp[CAM_OTP_DATA_LEN];


struct stCAM_CAL_FUNC_STRUCT g_camCalCMDFunc[] = {
	{CMD_BRCB032GWZ, brcb032gwz_selective_read_region},
	{CMD_CAT24C16, cat24c16_selective_read_region},
	{CMD_GT24C32A, gt24c32a_selective_read_region},

	/*      ADD before this line */
	{0, 0} /*end of list*/
};

struct stCAM_CAL_LIST_STRUCT g_camCalList[] = {
/*Below is commom sensor */
#if defined(CONFIG_CAM_OTP_DATA_USE_IDME)
	{HI556_SENSOR_ID, 0xA0, CMD_AUTO, cam_cal_check_mtk_cid, cam_cal_rear_read_region},
	{HI556SEC_SENSOR_ID, 0xA0, CMD_AUTO, cam_cal_check_mtk_cid, cam_cal_rear_read_region},
	{OV02B_SENSOR_ID, 0xA8, CMD_AUTO, cam_cal_check_mtk_cid, cam_cal_front_read_region},
	{OV02BSEC_SENSOR_ID, 0xA8, CMD_AUTO, cam_cal_check_mtk_cid, cam_cal_front_read_region},
	{GC5035_SENSOR_ID, 0xA0, CMD_AUTO, cam_cal_check_mtk_cid, cam_cal_rear_read_region},
	{GC5035SEC_SENSOR_ID, 0xA0, CMD_AUTO, cam_cal_check_mtk_cid, cam_cal_rear_read_region},
	{GC02M1TR_SENSOR_ID, 0xA8, CMD_AUTO, cam_cal_check_mtk_cid, cam_cal_front_read_region},
	{GC02M1TRSEC_SENSOR_ID, 0xA8, CMD_AUTO, cam_cal_check_mtk_cid, cam_cal_front_read_region},
#endif
	{HI1336_SENSOR_ID, 0xA0, CMD_AUTO, cam_cal_check_mtk_cid, 0},
	{GC08A3_SENSOR_ID, 0xA2, CMD_AUTO, cam_cal_check_mtk_cid, 0},
	{IMX338_SENSOR_ID, 0xA0, CMD_AUTO, cam_cal_check_mtk_cid, 0},
	{S5K4E6_SENSOR_ID, 0xA8, CMD_AUTO, cam_cal_check_mtk_cid, 0},
	{IMX386_SENSOR_ID, 0xA0, CMD_AUTO, cam_cal_check_mtk_cid, 0},
	{S5K3M3_SENSOR_ID, 0xA0, CMD_AUTO, cam_cal_check_mtk_cid, 0},
	{S5K2L7_SENSOR_ID, 0xA0, CMD_AUTO, cam_cal_check_mtk_cid, 0},
	{IMX398_SENSOR_ID, 0xA0, CMD_AUTO, cam_cal_check_double_eeprom, 0},
	{IMX318_SENSOR_ID, 0xA0, CMD_AUTO, cam_cal_check_mtk_cid, 0},
	{OV8858_SENSOR_ID, 0xA8, CMD_AUTO, cam_cal_check_mtk_cid, 0},
	{IMX350_SENSOR_ID, 0xA0, CMD_AUTO, cam_cal_check_mtk_cid, 0},
	{S5K3P8SX_SENSOR_ID, 0xA0, CMD_AUTO, cam_cal_check_double_eeprom, 0},
	{IMX386_MONO_SENSOR_ID, 0xA0, CMD_AUTO, cam_cal_check_mtk_cid, 0},
	{IMX576_SENSOR_ID, 0xA2, CMD_AUTO, cam_cal_check_double_eeprom, 0},
/*99 */
	{IMX258_SENSOR_ID, 0xA0, CMD_AUTO, cam_cal_check_mtk_cid, 0},
	{IMX258_MONO_SENSOR_ID, 0xA0, CMD_AUTO, cam_cal_check_mtk_cid, 0},
/*97*/
	{OV23850_SENSOR_ID, 0xA0, CMD_AUTO, cam_cal_check_mtk_cid, 0},
	{OV23850_SENSOR_ID, 0xA8, CMD_AUTO, cam_cal_check_mtk_cid, 0},
	{S5K3M2_SENSOR_ID, 0xA0, CMD_AUTO, cam_cal_check_mtk_cid, 0},
/*39*/
	{OV13870_SENSOR_ID, 0xA8, CMD_AUTO, cam_cal_check_mtk_cid, 0},
	{OV8856_SENSOR_ID, 0xA0, CMD_AUTO, cam_cal_check_mtk_cid, 0},
/*55*/
	{S5K2P8_SENSOR_ID, 0xA2, CMD_AUTO, cam_cal_check_mtk_cid, 0},
	{S5K2P8_SENSOR_ID, 0xA0, CMD_AUTO, cam_cal_check_mtk_cid, 0},
	{OV8858_SENSOR_ID, 0xA2, CMD_AUTO, cam_cal_check_mtk_cid, 0},
/* Others */
	{S5K2X8_SENSOR_ID, 0xA0, CMD_AUTO, cam_cal_check_mtk_cid, 0},
	{IMX376_SENSOR_ID, 0xA2, CMD_AUTO, cam_cal_check_mtk_cid, 0},
	{IMX214_SENSOR_ID, 0xA0, CMD_AUTO, cam_cal_check_mtk_cid, 0},
	{IMX214_MONO_SENSOR_ID, 0xA0, CMD_AUTO, cam_cal_check_mtk_cid, 0},
	/*  ADD before this line */
	{0, 0, CMD_NONE, 0, 0} /*end of list*/
};


unsigned int cam_cal_get_sensor_list(struct stCAM_CAL_LIST_STRUCT **ppCamcalList)

{
	if (ppCamcalList == NULL)
		return 1;

	*ppCamcalList = &g_camCalList[0];
	return 0;
}


unsigned int cam_cal_get_func_list(struct stCAM_CAL_FUNC_STRUCT **ppCamcalFuncList)
{
	if (ppCamcalFuncList == NULL)
		return 1;

	*ppCamcalFuncList = &g_camCalCMDFunc[0];
	return 0;
}

static int idme_cam_otp_checksum(unsigned char *data)
{
	int i;
	unsigned int cal_sum = 0;
	unsigned char *p = (char *)&cal_sum;

	if (data == NULL) {
		CAM_CALDB("%s data is null\n", __func__);
		return -1;
	}

	/* calculating sum, adding all otp data by byte */
	for (i = 0; i < CAM_MTK_OTP_LEN - 4; i++)
		cal_sum += data[i];

	/* checksum */
	for (i = 0; i < 4; i++) {
		if (p[i] != data[CAM_MTK_OTP_LEN - 1 - i]) {
			CAM_CALDB("%s camera otp checksum failed\n", __func__);
			return -1;
		}
	}

	return 0;
}

static int idme_get_front_cam_otp(unsigned char *data)
{
	struct device_node *ap = NULL;
	const unsigned char *idme_data = NULL;
	char buf[3] = {0};
	int len, i;
	int ret = 0;

	if (data == NULL) {
		CAM_CALDB("%s data is null\n", __func__);
		return -1;
	}

	ap = of_find_node_by_path(IDME_OF_FRONT_CAM_OTP);
	if (ap) {
		idme_data = (const unsigned char *)of_get_property(ap, "value", &len);
		if (likely(len > 0 && len <= CAM_OTP_DATA_LEN*2)) {
			for (i = 0; i < (CAM_MTK_OTP_LEN*2 - 1); i += 2) {
				buf[0] = idme_data[i];
				buf[1] = idme_data[i + 1];
				ret = kstrtou8(buf, 16, data+(i/2));
				if (ret)
					CAM_CALDB("%s kstrtou8 failed, i=%d\n", __func__, i);
			}
		} else {
			CAM_CALDB("%s front cam otp len err=%d\n", __func__, len);
			return -1;
		}
	} else {
		CAM_CALDB("%s of_find_node_by_path failed\n", __func__);
		return -1;
	}

	return ret;
}

static int idme_get_rear_cam_otp(unsigned char *data)
{
	struct device_node *ap = NULL;
	const unsigned char *idme_data = NULL;
	char buf[3] = {0};
	int len, i;
	int ret = 0;

	if (data == NULL) {
		CAM_CALDB("%s data is null\n", __func__);
		return -1;
	}

	ap = of_find_node_by_path(IDME_OF_REAR_CAM_OTP);
	if (ap) {
		idme_data = (const unsigned char *)of_get_property(ap, "value", &len);
		if (likely(len > 0 && len <= CAM_OTP_DATA_LEN*2)) {
			for (i = 0; i < (CAM_MTK_OTP_LEN*2 - 1); i += 2) {
				buf[0] = idme_data[i];
				buf[1] = idme_data[i + 1];
				ret = kstrtou8(buf, 16, data+(i/2));
				if (ret)
					CAM_CALDB("%s kstrtou8 failed, i=%d\n", __func__, i);
			}
		} else {
			CAM_CALDB("%s rear cam otp len err=%d\n", __func__, len);
			return -1;
		}
	} else {
		CAM_CALDB("%s of_find_node_by_path failed\n", __func__);
		return -1;
	}

	return ret;
}

unsigned int cam_cal_front_read_region(
	struct i2c_client *client, unsigned int addr,
	unsigned char *data, unsigned int size)
{
	int err = 0;

	if (data == NULL)
		return 0;

	if ((addr + size) > CAM_MTK_OTP_LEN)
		return 0;

	err = idme_get_front_cam_otp(front_cam_otp);
	if (err == 0 && idme_cam_otp_checksum(front_cam_otp) == 0) {
		memcpy(data, front_cam_otp+addr, size);
		return size;
	}

	return 0;
}

unsigned int cam_cal_rear_read_region(
	struct i2c_client *client, unsigned int addr,
	unsigned char *data, unsigned int size)
{
	int err = 0;

	if (data == NULL)
		return 0;

	if ((addr + size) > CAM_MTK_OTP_LEN)
		return 0;

	err = idme_get_rear_cam_otp(rear_cam_otp);
	if (err == 0 && idme_cam_otp_checksum(front_cam_otp) == 0) {
		memcpy(data, rear_cam_otp+addr, size);
		return size;
	}

	return 0;
}

unsigned int cam_cal_check_mtk_cid(struct i2c_client *client, cam_cal_cmd_func readCamCalData)
{
	unsigned int calibrationID = 0, ret = 0;
	int j = 0;

	if (readCamCalData != NULL) {
		readCamCalData(client, 1, (unsigned char *)&calibrationID, 4);
		CAM_CALDB("calibrationID = %x\n", calibrationID);
	}

	if (calibrationID != 0)
		for (j = 0; j < MTK_MAX_CID_NUM; j++) {
			CAM_CALDB("mtkCidList[%d] == %x\n", j, calibrationID);
			if (mtkCidList[j] == calibrationID) {
				ret = 1;
				break;
			}
		}

	CAM_CALDB("ret =%d\n", ret);
	return ret;
}

unsigned int cam_cal_check_double_eeprom(struct i2c_client *client, cam_cal_cmd_func readCamCalData)
{
	unsigned int calibrationID = 0, ret = 1;

	CAM_CALDB("start cam_cal_check_double_eeprom !\n");
	if (readCamCalData != NULL) {
		CAM_CALDB("readCamCalData != NULL !\n");
		readCamCalData(client, 1, (unsigned char *)&calibrationID, 4);
		CAM_CALDB("calibrationID = %x\n", calibrationID);
	}

	return ret;
}



