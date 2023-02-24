/************************************************************
 *
 * file: p9415.c
 *
 * Description: P9415 Wireless Power Charger Driver
 *
 *------------------------------------------------------------
 *
 * Copyright (c) 2018, Integrated Device Technology Co., Ltd.
 * Copyright (C) 2019 Amazon.com Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be a reference
 * to you, but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *************************************************************/

#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/debugfs.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/power_supply.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/of.h>
#include <linux/delay.h>
#include <linux/regmap.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/switch.h>
#include <linux/firmware.h>
#include <linux/input.h>
#include <linux/fb.h>
#include <linux/notifier.h>
#include <linux/rtc.h>

#include <mt-plat/charger_type.h>
#include <mt-plat/charger_class.h>
#include <mtk_charger_intf.h>
#include <mt-plat/mtk_boot_common.h>

#include "p9415.h"

#define DEV_AUTH_RETRY	3
#define SEND_CMD_RETRY	3
#define VSWITCH_RETRY	3

#define DEFAULT_EPT_WORK_DELAY	30000	/* 30s */
#define DOCK_EVENT_DEBOUNCE	5000	/* 5s */
#define TIMEOUT_ON_STEP_CHARGING	30	/* 30s */

enum fod_type {
	TYPE_UNKNOWN = 0,
	TYPE_BPP,
	TYPE_EPP,
	TYPE_BPP_PLUS,
	TYPE_MAX
};


static int p9415_enable_charge_flow(struct p9415_dev *chip, bool en);
static bool p9415_get_pg_irq_status(struct p9415_dev *chip);
static int p9415_update_charge_type(struct p9415_dev *chip,
		enum charger_type type);
static void p9415_write_fod(struct p9415_dev *chip,
		enum charger_type chr_type);
static void p9415_enable_fast_charging(struct p9415_dev *chip);
static void p9415_init_adaptive_current_limit_work(struct p9415_dev *chip,
	int initial_delay);

static int p9415_read(void *data, u16 reg, u8 *val)
{
	unsigned int temp = 0;
	int rc = 0;
	struct p9415_dev *di = (struct p9415_dev *)data;

	rc = regmap_read(di->regmap, reg, &temp);
	if (rc >= 0)
		*val = (u8)temp;

	return rc;
}

static int p9415_read4(void *data, u16 reg, uint32_t *val)
{
	int rc = 0;
	struct p9415_dev *di = (struct p9415_dev *)data;
	uint8_t *rd = (uint8_t *) val;
	uint16_t idx = 0;

	for (idx = 0; idx < 4; idx++) {
		rc = di->bus.read(di, reg, rd);
		if (rc < 0) {
			dev_err(di->dev, "%s: error: %d, reg: %04x\n",
				__func__, rc, reg);
			return rc;
		}
		reg++;
		rd++;
	}
	return rc;
}

static int p9415_write(void *data, u16 reg, u8 val)
{
	int rc = 0;
	struct p9415_dev *di = (struct p9415_dev *)data;

	rc = regmap_write(di->regmap, reg, val);
	if (rc < 0) {
		dev_err(di->dev, "%s: error: %d, reg: %04x, val: %02x\n",
			__func__, rc, reg, val);
	}
	return rc;
}

static int p9415_write4(void *data, u16 reg, uint32_t val)
{
	int rc = 0;
	struct p9415_dev *di = (struct p9415_dev *)data;
	uint16_t idx;
	uint8_t *wr = (uint8_t *) &val;

	for (idx = 0; idx < 4; idx++) {
		rc = di->bus.write(di, reg, *wr);
		if (rc < 0) {
			dev_err(di->dev, "%s: error: %d, reg: %04x, val:%02x\n",
				__func__, rc, reg, *wr);
			return rc;
		}
		reg++;
		wr++;
	}
	return rc;
}

static int p9415_read_buffer(void *data, u16 reg, u8 *buf, u32 size)
{
	struct p9415_dev *di = (struct p9415_dev *)data;

	return regmap_bulk_read(di->regmap, reg, buf, size);
}

static int p9415_write_buffer(void *data, u16 reg, u8 *buf, u32 size)
{
	int rc = 0;
	u8 val = 0;
	struct p9415_dev *di = (struct p9415_dev *)data;

	while (size--) {
		val = *buf;
		rc = di->bus.write(di, reg, val);
		if (rc < 0) {
			dev_err(di->dev, "%s: error: %d, reg: %04x, val: %02x\n",
				__func__, rc, reg, val);
			return rc;
		}
		reg++;
		buf++;
	}

	return rc;
}

static void p9415_read_fw_ver(struct p9415_dev *chip)
{
	chip->bus.read_buf(chip, REG_CUST_CODE, chip->fw_ver.val, 6);
}

static ssize_t p9415_fw_ver(struct p9415_dev *chip)
{
	union fw_ver *fw_ver = &chip->fw_ver;

	p9415_read_fw_ver(chip);

	dev_info(chip->dev, "%s: FW Ver:%04x.%04x.%02x.%02x\n",
		__func__, fw_ver->cust_code, fw_ver->proj_id,
		fw_ver->major_rev, fw_ver->minor_rev);

	return 0;
}

static u64 p9415_get_vrect_adc(struct p9415_dev *chip)
{
	u8 buf[2] = {0};
	u64 vrect = 0;

	chip->bus.read_buf(chip, REG_ADC_VRECT, buf, 2);
	/* vrect = val*1.25*21000/4095 mV, val = REG_ADC_VRECT bit0-11 */
	buf[1] &= 0xf;
	vrect = (u64)(buf[0] | (buf[1] << 8));
	vrect = vrect * 21 * 125 * 10 / 4095;
	dev_dbg(chip->dev, "%s: vrect: %ld mV\n", __func__, vrect);

	return vrect;
}

static u16 p9415_get_iout_adc(struct p9415_dev *chip)
{
	u8 buf[2] = {0};
	u16 iout = 0;

	chip->bus.read_buf(chip, REG_RX_IOUT, buf, 2);
	iout = buf[0] | (buf[1] << 8);
	dev_dbg(chip->dev, "%s: iout: %d mA\n", __func__, iout);

	return iout;
}

static u16 p9415_get_iout_raw_adc(struct p9415_dev *chip)
{
	u8 buf[2] = {0};
	u16 iout = 0;

	chip->bus.read_buf(chip, REG_RX_IOUT_RAW, buf, 2);
	iout = buf[0] | (buf[1] << 8);
	dev_dbg(chip->dev, "%s: iout: %d mA\n", __func__, iout);

	return iout;
}

static u16 p9415_get_vout_adc(struct p9415_dev *chip)
{
	u8 buf[2] = {0};
	u32 vout = 0;

	chip->bus.read_buf(chip, REG_ADC_VOUT, buf, 2);
	/* vout = val / 4095 * 10 * 2.1 (V) */
	vout = (buf[0] | (buf[1] << 8)) * 21 * 1000 / 4095;
	dev_dbg(chip->dev, "%s: vout: %d mV\n", __func__, vout);

	return vout;
}

static u16 p9415_get_vout(struct p9415_dev *chip)
{
	u8 buf[2] = {0};
	u16 value = 0;
	u16 vout = 0;

	chip->bus.read_buf(chip, REG_VOUT_SET, buf, 2);
	value = buf[0] | (buf[1] << 8);
	/* vout = (value * 84) / 10 + 2800 */
	vout = (value * 84) / 10 + 2800;

	return vout;
}

static int p9415_set_vout(struct p9415_dev *chip, int vout)
{
	int ret = 0;
	u16 value = 0;

	if ((vout >= SET_VOUT_MIN) && (vout <= SET_VOUT_MAX)) {
		dev_info(chip->dev, "%s: Set vout: %d mv\n", __func__, vout);
		value = (vout - 2800) * 10 / 84;
		dev_info(chip->dev, "%s: vout adc code:%d 0x%x\n",
			__func__, value, value);
		ret = chip->bus.write_buf(chip, REG_VOUT_SET,
				(u8 *) &value, 2);
	} else {
		dev_err(chip->dev, "%s: Set vout parameter error!\n", __func__);
		ret = -EINVAL;
	}

	if (ret)
		dev_err(chip->dev, "%s: Failed to set vout!\n", __func__);

	return ret;
}

static int p9415_get_temp_adc(struct p9415_dev *chip)
{
	u8 buf[2] = {0};
	int ret = 0;
	/* The default temp value is -177.1°C */
	int temp = P9415_DIE_TEMP_DEFAULT;

	if (p9415_get_pg_irq_status(chip))
		ret = chip->bus.read_buf(chip, REG_ADC_TEMP, buf, 2);
	else
		goto out;

	if (ret) {
		dev_err(chip->dev, "%s: Failed to read temp: %d\n",
			__func__, ret);
		goto out;
	}

	/* temp = (val * 7487 - 17710000)/100000 */
	temp = ((int)(buf[0] | (buf[1] << 8)) * 7487 - 17710000) / 100000;

out:
	dev_dbg(chip->dev, "%s: temp:%d degrees C\n", __func__, temp);

	return temp;
}

static u8 p9415_get_tx_signal_strength(struct p9415_dev *chip)
{
	int ret = 0;
	u8 ss = 0;

	ret = chip->bus.read(chip, REG_SS, &ss);
	if (ret) {
		dev_err(chip->dev, "%s: Failed to read tx signal strength: %d\n",
			__func__, ret);
		goto out;
	}

	dev_dbg(chip->dev, "%s: tx signal strength:%d\n", __func__, ss);
out:

	return ss;
}

static u16 p9415_get_calibration_value(struct p9415_dev *chip)
{
	u8 buf[2] = {0};
	u16 cal = 0;

	chip->bus.read_buf(chip, REG_GAIN1, buf, 2);
	cal = buf[0] | (buf[1] << 8);

	return cal;
}

/*	start_address:
 *	0: for FW
 *	0x5E00: for configuration table
 */
static int p9415_mtp_update(struct p9415_dev *chip,
	const char *mtp_name, const char *bin_name,
	uint32_t start_address)
{
	uint8_t val;
	int ret;
	uint16_t retry;
	struct idtp9415pgmtype fw_section;
	int idx, code_position, current_code_length;
	const struct firmware *fw;
	const struct firmware *fw_mtp;
	const uint8_t *firmware = NULL;
	uint8_t *mtp_downloader = NULL;
	uint32_t size = 0;
	uint32_t mtp_size = 0;
	const uint32_t section_size = 128;	/* 128 bytes */
	const uint32_t header_size = 8;		/* 8 bytes */

	dev_info(chip->dev, "%s ++\n", __func__);

	disable_irq(chip->pg_num);
	disable_irq(chip->int_num);

	mutex_lock(&chip->sys_lock);

	if (request_firmware(&fw_mtp, mtp_name, chip->dev)) {
		dev_err(chip->dev, "%s: [MTP]Cannot find MTP:%s\n",
			__func__, mtp_name);
		ret = -EAGAIN;
		goto out_unlock;
	} else {
		dev_info(chip->dev, "%s: [MTP]Found MTP:%s\n",
			__func__, mtp_name);
		mtp_size = fw_mtp->size;
		mtp_downloader = (uint8_t *) fw_mtp->data;
	}
	if (request_firmware(&fw, bin_name, chip->dev)) {
		dev_err(chip->dev, "%s: [MTP]Cannot find bin:%s\n",
			__func__, bin_name);
		ret = -EAGAIN;
		goto out_release_mtp;
	} else {
		size = fw->size;
		firmware = fw->data;
		dev_info(chip->dev, "%s: [MTP]Found bin:%s\n",
			__func__, bin_name);
	}

	/* Update by adb */
	if (!chip->wpc_en) {
		pinctrl_select_state(chip->wpc_pinctrl, chip->wpc_enable);
		usleep_range(10000, 20000);
	}
	pinctrl_select_state(chip->wpc_pinctrl, chip->fw_dl_enable);
	chip->fw_dl_en = true;
	usleep_range(10000, 20000);

	dev_info(chip->dev, "%s: [MTP]starting\n", __func__);

	chip->bus.write(chip, 0x3000, 0x5A);
	usleep_range(10000, 20000);

	chip->bus.write(chip, 0x3004, 0);
	usleep_range(10000, 20000);
	chip->bus.write(chip, 0x3008, 0x9);
	usleep_range(10000, 20000);
	chip->bus.write(chip, 0x300C, 0x5);
	usleep_range(10000, 20000);
	chip->bus.write(chip, 0x300D, 0x1D);
	usleep_range(10000, 20000);

	chip->bus.write(chip, 0x3040, 0x11);

	usleep_range(10000, 20000);

	chip->bus.write(chip, 0x3040, 0x10);

	usleep_range(10000, 20000);

	/* programming downloader to 0x0800 */
	dev_info(chip->dev, "%s: [MTP]Prog downloader\n", __func__);
	dev_info(chip->dev, "%s: [MTP]MTP size: %d\n", __func__, mtp_size);
	ret = chip->bus.write_buf(chip, 0x0800, mtp_downloader, mtp_size);
	if (ret) {
		dev_err(chip->dev, "%s: [MTP]Fail to prog downloader\n",
			__func__);
		ret = -EAGAIN;
		goto out;
	}

	chip->bus.write(chip, 0x0400, 0);
	chip->bus.write(chip, 0x3048, 0xD0);
	/* There will be no acknowledge. */
	chip->bus.write(chip, 0x3040, 0x80);

	/* wait 100ms to reset IC */
	msleep(100);

	/*
	 * downloading bin file to 0x0400
	 * 128 bytes per section
	 * 0x0400 = 0x1 to start downloading
	 *
	 * read 0x0401
	 * status code:
	 * 0x02 = OK
	 * 0x04 = MTP Write Error
	 * 0x08 = Check Sum Error
	 */

	dev_info(chip->dev, "%s: [MTP]Downloading bin file\n", __func__);
	dev_info(chip->dev, "%s: size:%d\n", __func__, size);
	for (code_position = 0; code_position < size;
			code_position += section_size) {
		fw_section.status = 0;
		fw_section.startAddr = code_position + start_address;
		fw_section.dataChksum = fw_section.startAddr;
		memcpy(fw_section.dataBuf, firmware + code_position,
			section_size);

		/* calculate checksum and code length */
		current_code_length = size -
			(code_position/section_size)*section_size;
		if (current_code_length > section_size)
			current_code_length = section_size;

		fw_section.codeLength = current_code_length;

		dev_dbg(chip->dev, "%s: code length=0x%x\n",
					__func__, fw_section.codeLength);
		for (idx = 0; idx < current_code_length; idx++)
			fw_section.dataChksum += fw_section.dataBuf[idx];
		fw_section.dataChksum += fw_section.codeLength;

		dev_info(chip->dev, "%s: [MTP](0x%x)chk-sum:0x%04x\n",
					__func__,
					code_position, fw_section.dataChksum);


		dev_info(chip->dev, "%s: [MTP]download section 0x%x\n",
			__func__, code_position);
		ret = chip->bus.write_buf(chip, 0x0400, (uint8_t *) &fw_section,
					(fw_section.codeLength + header_size));
		if (ret) {
			dev_err(chip->dev, "%s: [MTP]Fail to write bin (%d)\n",
				__func__, code_position);
			ret = -EAGAIN;
			goto out;
		}

		/* trigger downloader to start downloading */
		ret = chip->bus.write(chip, 0x0400, 0x1);
		if (ret) {
			dev_err(chip->dev, "%s: [MTP]Fail to starting downloading\n",
				__func__);
			ret = -EAGAIN;
			goto out;
		}

		retry = 10;
		do {
			usleep_range(20000, 30000);
			val = 0;
			chip->bus.read(chip, 0x0401, &val);
		} while ((val & 1) && --retry > 0);

		if (val != 2) {
			switch (val) {
			case 4:
				dev_err(chip->dev, "%s: [MTP]MTP Write Error\n",
					__func__);
				break;
			case 8:
				dev_err(chip->dev, "%s: [MTP]Check Sum Error\n",
					__func__);
				break;
			default:
				dev_err(chip->dev, "%s: [MTP]Unknown error\n",
					__func__);
			}
			ret = -EAGAIN;
			goto out;
		}
	}
	dev_info(chip->dev, "%s: [MTP]complete downloading\n", __func__);
	/* complete downloading */

	chip->bus.write(chip, 0x3000, 0x5A);
	usleep_range(10000, 20000);
	chip->bus.write(chip, 0x3048, 0x10);
	usleep_range(10000, 20000);


	ret = 0;

out:
	pinctrl_select_state(chip->wpc_pinctrl, chip->fw_dl_disable);
	if (!chip->wpc_en) {
		dev_info(chip->dev, "%s: Powerdown P9415\n", __func__);
		pinctrl_select_state(chip->wpc_pinctrl, chip->wpc_disable);
	}
	chip->fw_dl_en = false;
	release_firmware(fw);
out_release_mtp:
	release_firmware(fw_mtp);
out_unlock:
	mutex_unlock(&chip->sys_lock);

	enable_irq(chip->int_num);
	enable_irq(chip->pg_num);

	dev_info(chip->dev, "%s --\n", __func__);
	return ret;
}

/*	start_address:
 *	0x5F00: for calibration value
 */
static int p9415_mtp_update_data(struct p9415_dev *chip,
	const char *mtp_name, const char *data, const uint32_t length,
	uint32_t start_address)
{
	uint8_t val;
	int ret;
	uint16_t retry;
	struct idtp9415pgmtype fw_section;
	int idx, code_position, current_code_length;
	const struct firmware *fw_mtp;
	uint8_t *mtp_downloader = NULL;
	uint32_t size = length;
	uint32_t mtp_size = 0;
	const uint32_t section_size = 128;	/* 128 bytes */
	const uint32_t header_size = 8;		/* 8 bytes */

	dev_info(chip->dev, "%s ++\n", __func__);

	disable_irq(chip->pg_num);
	disable_irq(chip->int_num);

	mutex_lock(&chip->sys_lock);

	if (request_firmware(&fw_mtp, mtp_name, chip->dev)) {
		dev_err(chip->dev, "%s: [MTP]Cannot find MTP:%s\n",
			__func__, mtp_name);
		ret = -EAGAIN;
		goto out_unlock;
	} else {
		dev_info(chip->dev, "%s: [MTP]Found MTP:%s\n",
			__func__, mtp_name);
		mtp_size = fw_mtp->size;
		mtp_downloader = (uint8_t *) fw_mtp->data;
	}

	/* Update by adb */
	if (!chip->wpc_en) {
		pinctrl_select_state(chip->wpc_pinctrl, chip->wpc_enable);
		usleep_range(10000, 20000);
	}
	pinctrl_select_state(chip->wpc_pinctrl, chip->fw_dl_enable);
	chip->fw_dl_en = true;
	usleep_range(10000, 20000);

	dev_info(chip->dev, "%s: [MTP]starting\n", __func__);

	chip->bus.write(chip, 0x3000, 0x5A);
	usleep_range(10000, 20000);

	chip->bus.write(chip, 0x3004, 0);
	usleep_range(10000, 20000);
	chip->bus.write(chip, 0x3008, 0x9);
	usleep_range(10000, 20000);
	chip->bus.write(chip, 0x300C, 0x5);
	usleep_range(10000, 20000);
	chip->bus.write(chip, 0x300D, 0x1D);
	usleep_range(10000, 20000);

	chip->bus.write(chip, 0x3040, 0x11);

	usleep_range(10000, 20000);

	chip->bus.write(chip, 0x3040, 0x10);

	usleep_range(10000, 20000);

	/* programming downloader to 0x0800 */
	dev_info(chip->dev, "%s: [MTP]Prog downloader\n", __func__);
	dev_info(chip->dev, "%s: [MTP]MTP size: %d\n", __func__, mtp_size);
	ret = chip->bus.write_buf(chip, 0x0800, mtp_downloader, mtp_size);
	if (ret) {
		dev_err(chip->dev, "%s: [MTP]Fail to prog downloader\n",
			__func__);
		ret = -EAGAIN;
		goto out;
	}

	chip->bus.write(chip, 0x0400, 0);
	chip->bus.write(chip, 0x3048, 0xD0);
	/* There will be no acknowledge. */
	chip->bus.write(chip, 0x3040, 0x80);

	/* wait 100ms to reset IC */
	msleep(100);

	/*
	 * downloading bin file to 0x0400
	 * 128 bytes per section
	 * 0x0400 = 0x1 to start downloading
	 *
	 * read 0x0401
	 * status code:
	 * 0x02 = OK
	 * 0x04 = MTP Write Error
	 * 0x08 = Check Sum Error
	 */

	dev_info(chip->dev, "%s: [MTP]Downloading bin file\n", __func__);
	dev_info(chip->dev, "%s: size:%d\n", __func__, size);
	for (code_position = 0; code_position < size;
			code_position += section_size) {
		fw_section.status = 0;
		fw_section.startAddr = code_position + start_address;
		fw_section.dataChksum = fw_section.startAddr;
		memcpy(fw_section.dataBuf, data + code_position,
			section_size);

		/* calculate checksum and code length */
		current_code_length = size -
			(code_position/section_size)*section_size;
		if (current_code_length > section_size)
			current_code_length = section_size;

		fw_section.codeLength = current_code_length;

		dev_dbg(chip->dev, "%s: code length=0x%x\n",
					__func__, fw_section.codeLength);
		for (idx = 0; idx < current_code_length; idx++)
			fw_section.dataChksum += fw_section.dataBuf[idx];
		fw_section.dataChksum += fw_section.codeLength;

		dev_info(chip->dev, "%s: [MTP](0x%x)chk-sum:0x%04x\n",
					__func__,
					code_position, fw_section.dataChksum);


		dev_info(chip->dev, "%s: [MTP]download section 0x%x\n",
			__func__, code_position);
		ret = chip->bus.write_buf(chip, 0x0400, (uint8_t *) &fw_section,
					(fw_section.codeLength + header_size));
		if (ret) {
			dev_err(chip->dev, "%s: [MTP]Fail to write bin (%d)\n",
				__func__, code_position);
			ret = -EAGAIN;
			goto out;
		}

		/* trigger downloader to start downloading */
		ret = chip->bus.write(chip, 0x0400, 0x1);
		if (ret) {
			dev_err(chip->dev, "%s: [MTP]Fail to starting downloading\n",
				__func__);
			ret = -EAGAIN;
			goto out;
		}

		retry = 10;
		do {
			usleep_range(20000, 30000);
			val = 0;
			chip->bus.read(chip, 0x0401, &val);
		} while ((val & 1) && --retry > 0);

		if (val != 2) {
			switch (val) {
			case 4:
				dev_err(chip->dev, "%s: [MTP]MTP Write Error\n",
					__func__);
				break;
			case 8:
				dev_err(chip->dev, "%s: [MTP]Check Sum Error\n",
					__func__);
				break;
			default:
				dev_err(chip->dev, "%s: [MTP]Unknown error\n",
					__func__);
			}
			ret = -EAGAIN;
			goto out;
		}
	}
	dev_info(chip->dev, "%s: [MTP]complete downloading\n", __func__);
	/* complete downloading */

	chip->bus.write(chip, 0x3000, 0x5A);
	usleep_range(10000, 20000);
	chip->bus.write(chip, 0x3048, 0x10);
	usleep_range(10000, 20000);


	ret = 0;

out:
	pinctrl_select_state(chip->wpc_pinctrl, chip->fw_dl_disable);
	if (!chip->wpc_en) {
		dev_info(chip->dev, "%s: Powerdown P9415\n", __func__);
		pinctrl_select_state(chip->wpc_pinctrl, chip->wpc_disable);
	}
	chip->fw_dl_en = false;

	release_firmware(fw_mtp);
out_unlock:
	mutex_unlock(&chip->sys_lock);

	enable_irq(chip->int_num);
	enable_irq(chip->pg_num);

	dev_info(chip->dev, "%s --\n", __func__);
	return ret;
}

static ssize_t p9415_version_show(struct device *dev,
		struct device_attribute *attr,
		char *buffer)
{
	u8 ver[] = { 0xff };
	struct i2c_client *client = to_i2c_client(dev);
	struct p9415_dev *chip = (struct p9415_dev *)i2c_get_clientdata(client);

	mutex_lock(&chip->sys_lock);
	chip->bus.read_buf(chip, REG_CHIP_REV, ver, 1);
	mutex_unlock(&chip->sys_lock);

	return scnprintf(buffer, PAGE_SIZE, "%02x\n", ver[0]);
}

static ssize_t p9415_id_show(struct device *dev,
		struct device_attribute *attr,
		char *buffer)
{
	u8 id[] = { 0xff, 0xff };
	struct i2c_client *client = to_i2c_client(dev);
	struct p9415_dev *chip = (struct p9415_dev *)i2c_get_clientdata(client);

	mutex_lock(&chip->sys_lock);
	if (chip->wpc_en) {
		chip->bus.read_buf(chip, REG_CHIP_ID, id, 2);
	} else {
		/* Reading by adb */
		pinctrl_select_state(chip->wpc_pinctrl, chip->wpc_enable);
		usleep_range(10000, 20000);
		chip->bus.read_buf(chip, REG_CHIP_ID, id, 2);
		pinctrl_select_state(chip->wpc_pinctrl, chip->wpc_disable);
	}
	mutex_unlock(&chip->sys_lock);

	return scnprintf(buffer, PAGE_SIZE, "%04x\n",
				id[0] | (id[1] << 8));
}
static ssize_t p9415_pg_stat_show(struct device *dev,
		struct device_attribute *attr,
		char *buffer)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9415_dev *chip = (struct p9415_dev *)i2c_get_clientdata(client);
	uint32_t pg = 0;

	if (p9415_get_pg_irq_status(chip))
		pg = 1;
	else
		pg = 0;
	return scnprintf(buffer, PAGE_SIZE, "%d\n", pg);
}

static ssize_t p9415_fw_ver_show(struct device *dev,
		struct device_attribute *attr,
		char *buffer)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9415_dev *chip = (struct p9415_dev *)i2c_get_clientdata(client);
	union fw_ver *fw_ver = &chip->fw_ver;

	mutex_lock(&chip->sys_lock);
	if (chip->wpc_en) {
		p9415_read_fw_ver(chip);
	} else {
		/* Reading by adb */
		pinctrl_select_state(chip->wpc_pinctrl, chip->wpc_enable);
		usleep_range(10000, 20000);
		p9415_read_fw_ver(chip);
		pinctrl_select_state(chip->wpc_pinctrl, chip->wpc_disable);
	}
	mutex_unlock(&chip->sys_lock);

	return scnprintf(buffer, PAGE_SIZE, "%04x.%04x.%02x.%02x\n",
		fw_ver->cust_code, fw_ver->proj_id,
		fw_ver->major_rev, fw_ver->minor_rev);
}

static void p9415_wait_iout(struct p9415_dev *chip, uint16_t ma)
{
	uint16_t iout;
	uint16_t count = 0;

	/* wait iout <= mA to avoid OCP */
	do {
		msleep(100);
		iout = p9415_get_iout_adc(chip);
	} while (iout >= ma && ((++count) < (5000/100)));
}
static void p9415_wait_vout(struct p9415_dev *chip, uint16_t mv)
{
	uint16_t vout;
	uint16_t count = 0;

	/* wait vout >= mV */
	do {
		msleep(100);
		vout = p9415_get_vout_adc(chip);
	} while (vout <= mv && ((++count) < (5000/100)));
}

static ssize_t p9415_power_switch_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9415_dev *chip = (struct p9415_dev *)i2c_get_clientdata(client);
	struct charger_manager *cm =
		(struct charger_manager *) chip->consumer->cm;
	enum charger_type charger = CHARGER_UNKNOWN;
	uint16_t power;

	/* Wireless charger is not ready. */
	if (!p9415_get_pg_irq_status(chip))
		return -EINVAL;

	if (kstrtou16(buf, 0, &power))
		return -EINVAL;

	if (power == 15)
		charger = WIRELESS_CHARGER_15W;
	else if (power == 10)
		charger = WIRELESS_CHARGER_10W;
	else
		charger = WIRELESS_CHARGER_5W;

	if (cm->chr_type == charger)
		goto out;

	if (cm->chr_type == WIRELESS_CHARGER_5W) {
		if (charger == WIRELESS_CHARGER_10W) {
			p9415_set_vout(chip, chip->vout_10w);
			p9415_wait_vout(chip, chip->vout_10w - 500);
		} else {
			p9415_set_vout(chip, chip->vout_15w);
			p9415_wait_vout(chip, chip->vout_15w - 1000);
		}

		p9415_update_charge_type(chip, charger);
	} else if (cm->chr_type == WIRELESS_CHARGER_15W) {
		p9415_update_charge_type(chip, charger);

		if (charger == WIRELESS_CHARGER_10W) {
			p9415_wait_iout(chip, 1100);
			p9415_set_vout(chip, chip->vout_10w);
			p9415_wait_vout(chip, chip->vout_10w - 500);
		} else {
			p9415_wait_iout(chip, 1000);
			p9415_set_vout(chip, 5000);
		}
	} else {
		/* WIRELESS_CHARGER_10W */
		if (charger == WIRELESS_CHARGER_5W) {
			p9415_update_charge_type(chip, charger);
			p9415_wait_iout(chip, 1000);
			p9415_set_vout(chip, 5000);
		} else {
			p9415_set_vout(chip, chip->vout_15w);
			p9415_wait_vout(chip, chip->vout_15w - 1000);
			p9415_update_charge_type(chip, charger);
		}
	}

	p9415_write_fod(chip, charger);

out:
	return count;
}

static ssize_t p9415_vrect_adc_show(struct device *dev,
		struct device_attribute *attr,
		char *buffer)
{
	u64 vrect = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct p9415_dev *chip = (struct p9415_dev *)i2c_get_clientdata(client);


	mutex_lock(&chip->sys_lock);
	if (p9415_get_pg_irq_status(chip))
		vrect = p9415_get_vrect_adc(chip);
	mutex_unlock(&chip->sys_lock);

	return scnprintf(buffer, PAGE_SIZE, "%ld\n", vrect);
}

static ssize_t p9415_iout_show(struct device *dev,
		struct device_attribute *attr,
		char *buffer)
{
	u16 iout = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct p9415_dev *chip = (struct p9415_dev *)i2c_get_clientdata(client);


	mutex_lock(&chip->sys_lock);
	if (p9415_get_pg_irq_status(chip))
		iout = p9415_get_iout_adc(chip);
	mutex_unlock(&chip->sys_lock);

	return scnprintf(buffer, PAGE_SIZE, "%d\n", iout);
}

static ssize_t p9415_iout_raw_show(struct device *dev,
		struct device_attribute *attr,
		char *buffer)
{
	u16 iout = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct p9415_dev *chip = (struct p9415_dev *)i2c_get_clientdata(client);


	mutex_lock(&chip->sys_lock);
	if (p9415_get_pg_irq_status(chip))
		iout = p9415_get_iout_raw_adc(chip);
	mutex_unlock(&chip->sys_lock);

	return scnprintf(buffer, PAGE_SIZE, "%d\n", iout);
}

static ssize_t p9415_vout_adc_show(struct device *dev,
		struct device_attribute *attr,
		char *buffer)
{
	int vout = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct p9415_dev *chip = (struct p9415_dev *)i2c_get_clientdata(client);

	mutex_lock(&chip->sys_lock);
	if (p9415_get_pg_irq_status(chip))
		vout = p9415_get_vout_adc(chip);
	mutex_unlock(&chip->sys_lock);

	return scnprintf(buffer, PAGE_SIZE, "%d\n", vout);
}

static ssize_t p9415_vout_set_show(struct device *dev,
		struct device_attribute *attr,
		char *buffer)
{
	int vout = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct p9415_dev *chip = (struct p9415_dev *)i2c_get_clientdata(client);


	mutex_lock(&chip->sys_lock);
	if (p9415_get_pg_irq_status(chip))
		vout = p9415_get_vout(chip);
	mutex_unlock(&chip->sys_lock);

	return scnprintf(buffer, PAGE_SIZE, "%d\n", vout);
}

static ssize_t p9415_vout_set_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t count)
{
	int ret = 0, vout = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct p9415_dev *chip = (struct p9415_dev *)i2c_get_clientdata(client);

	mutex_lock(&chip->sys_lock);
	ret = kstrtoint(buf, 10, &vout);
	if (ret < 0) {
		dev_err(chip->dev, "%s: kstrtoint failed! ret:%d\n",
			__func__, ret);
		goto out;
	}
	p9415_set_vout(chip, vout);

out:
	mutex_unlock(&chip->sys_lock);
	return count;
}

static ssize_t p9415_temp_adc_show(struct device *dev,
		struct device_attribute *attr,
		char *buffer)
{
	int temp = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct p9415_dev *chip = (struct p9415_dev *)i2c_get_clientdata(client);

	mutex_lock(&chip->sys_lock);
	if (p9415_get_pg_irq_status(chip))
		temp = p9415_get_temp_adc(chip);
	mutex_unlock(&chip->sys_lock);

	return scnprintf(buffer, PAGE_SIZE, "%d\n", temp);
}

static ssize_t p9415_tx_signal_strength_show(struct device *dev,
		struct device_attribute *attr,
		char *buffer)
{
	u8 ss;
	struct i2c_client *client = to_i2c_client(dev);
	struct p9415_dev *chip = (struct p9415_dev *)i2c_get_clientdata(client);

	mutex_lock(&chip->sys_lock);
	if (p9415_get_pg_irq_status(chip))
		ss = p9415_get_tx_signal_strength(chip);
	mutex_unlock(&chip->sys_lock);

	return scnprintf(buffer, PAGE_SIZE, "%d\n", ss);
}

static ssize_t p9415_tx_id_show(struct device *dev,
		struct device_attribute *attr,
		char *buffer)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9415_dev *chip = (struct p9415_dev *)i2c_get_clientdata(client);
	uint16_t tx_id = 0;
	int32_t ret;

	if (!p9415_get_pg_irq_status(chip))
		return -EINVAL;

	ret = chip->bus.read_buf(chip, REG_TX_ID, (uint8_t *) &tx_id, 2);
	if (ret)
		return -EINVAL;
	return scnprintf(buffer, PAGE_SIZE, "0x%04x\n", tx_id);
}

static ssize_t p9415_irq_stat_show(struct device *dev,
		struct device_attribute *attr,
		char *buffer)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9415_dev *chip = (struct p9415_dev *)i2c_get_clientdata(client);
	uint32_t irq_stat = 0;
	int32_t ret;

	if (!p9415_get_pg_irq_status(chip))
		return -EINVAL;

	ret = chip->bus.read_buf(chip, REG_STATUS, (uint8_t *) &irq_stat, 4);
	if (ret)
		return -EINVAL;
	return scnprintf(buffer, PAGE_SIZE, "0x%08x\n", irq_stat);
}

static ssize_t p9415_id_authen_status_show(struct device *dev,
		struct device_attribute *attr,
		char *buffer)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9415_dev *chip = (struct p9415_dev *)i2c_get_clientdata(client);

	return scnprintf(buffer, PAGE_SIZE, "%d\n", chip->tx_id_authen_status);
}

static ssize_t p9415_dev_authen_status_show(struct device *dev,
		struct device_attribute *attr,
		char *buffer)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9415_dev *chip = (struct p9415_dev *)i2c_get_clientdata(client);

	return scnprintf(buffer, PAGE_SIZE, "%d\n", chip->tx_dev_authen_status);
}

static ssize_t p9415_over_reason_show(struct device *dev,
		struct device_attribute *attr,
		char *buffer)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9415_dev *chip = (struct p9415_dev *)i2c_get_clientdata(client);

	return scnprintf(buffer, PAGE_SIZE, "%u\n", chip->over_reason);
}

static ssize_t p9415_fod_regs_show(struct device *dev,
		struct device_attribute *attr,
		char *buffer)
{
	u16 reg = 0x00;
	u8 val = 0;
	ssize_t len = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct p9415_dev *chip = (struct p9415_dev *)i2c_get_clientdata(client);
	uint16_t idx_coe;
	struct p9415_fodcoeftype *fod_coe;

	for (reg = REG_FOD_COEF_ADDR; reg <= REG_FOD_DUMP_MAX; reg++) {
		chip->bus.read(chip, reg, &val);
		len += scnprintf(buffer+len, PAGE_SIZE-len,
			"fod reg:0x%02x=0x%02x\n", reg, val);
	}

	fod_coe = chip->bpp_5w_fod;
	len += scnprintf(buffer+len, PAGE_SIZE-len, "BPP:\n");
	for (idx_coe = 0; idx_coe < FOD_COEF_ARRY_LENGTH; idx_coe++)
		len += scnprintf(buffer+len, PAGE_SIZE-len,
					"FOD%d:0x%02x%02x\n", idx_coe,
					fod_coe[idx_coe].offs,
					fod_coe[idx_coe].gain);

	fod_coe = chip->epp_10w_fod;
	len += scnprintf(buffer+len, PAGE_SIZE-len, "EPP:\n");
	for (idx_coe = 0; idx_coe < FOD_COEF_ARRY_LENGTH; idx_coe++)
		len += scnprintf(buffer+len, PAGE_SIZE-len,
					"FOD%d:0x%02x%02x\n", idx_coe,
					fod_coe[idx_coe].offs,
					fod_coe[idx_coe].gain);

	fod_coe = chip->bpp_plus_15w_fod;
	len += scnprintf(buffer+len, PAGE_SIZE-len, "BPP+:\n");
	for (idx_coe = 0; idx_coe < FOD_COEF_ARRY_LENGTH; idx_coe++)
		len += scnprintf(buffer+len, PAGE_SIZE-len,
					"FOD%d:0x%02x%02x\n", idx_coe,
					fod_coe[idx_coe].offs,
					fod_coe[idx_coe].gain);

	return len;
}

static ssize_t p9415_fod_regs_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9415_dev *chip = (struct p9415_dev *)i2c_get_clientdata(client);
	int cnt = 0;
	uint32_t fod[FOD_COEF_ARRY_LENGTH] = { 0 };
	int ret = 0;
	uint16_t idx = 0;
	enum fod_type type = TYPE_UNKNOWN;
	struct p9415_fodcoeftype *fod_coe;

	mutex_lock(&chip->sys_lock);
	cnt = sscanf(buf, "%u %x %x %x %x %x %x",
			&type,
			&fod[0], &fod[1], &fod[2], &fod[3],
			&fod[4], &fod[5]);
	if (cnt > 7) {
		mutex_unlock(&chip->sys_lock);
		return -EINVAL;
	}

	switch (type) {
	case TYPE_BPP:
		fod_coe = chip->bpp_5w_fod;
		break;
	case TYPE_EPP:
		fod_coe = chip->epp_10w_fod;
		break;
	case TYPE_BPP_PLUS:
		fod_coe = chip->bpp_plus_15w_fod;
		break;
	default:
		mutex_unlock(&chip->sys_lock);
		return -EINVAL;
	}

	mutex_lock(&chip->fod_lock);
	for (idx = 0; idx < (cnt-1); idx++) {

		fod_coe[idx].gain = fod[idx] & 0xFF;
		fod_coe[idx].offs = (fod[idx] >> 8) & 0xFF;
		dev_info(chip->dev, "%s: 0x%x 0x%x 0x%x\n", __func__,
			fod[idx], fod_coe[idx].offs, fod_coe[idx].gain);

		if (p9415_get_pg_irq_status(chip)) {
			ret = chip->bus.write_buf(chip,
						REG_FOD_COEF_ADDR + idx*2,
						(uint8_t *) &fod[idx], 2);
			if (ret) {
				mutex_unlock(&chip->fod_lock);
				mutex_unlock(&chip->sys_lock);
				return -EINVAL;
			}
		}
	}
	mutex_unlock(&chip->sys_lock);
	mutex_unlock(&chip->fod_lock);
	return count;
}

static ssize_t p9415_regs_dump_show(struct device *dev,
		struct device_attribute *attr,
		char *buffer)
{
	u16 reg = 0x00;
	u8 val = 0;
	ssize_t len = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct p9415_dev *chip = (struct p9415_dev *)i2c_get_clientdata(client);

	mutex_lock(&chip->sys_lock);
	for (reg = 0x00; reg <= REG_DUMP_MAX; reg++) {
		chip->bus.read(chip, reg, &val);
		len += scnprintf(buffer+len, PAGE_SIZE-len,
			"reg:0x%02x=0x%02x\n", reg, val);
	}
	mutex_unlock(&chip->sys_lock);

	return len;
}

static ssize_t p9415_reg_show(struct device *dev,
		struct device_attribute *attr,
		char *buffer)
{
	u8 buf[REG_DUMP_MAX] = {0};
	int ret = 0;
	int i = 0;
	int len = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct p9415_dev *chip = (struct p9415_dev *)i2c_get_clientdata(client);

	if (!p9415_get_pg_irq_status(chip))
		return -ENODEV;
	ret = chip->bus.read_buf(chip, chip->reg.addr, buf, chip->reg.size);
	if (ret) {
		dev_err(chip->dev, "%s: Failed to read reg: %d\n",
			__func__,  ret);
		return ret;
	}
	for (i = 0; i < chip->reg.size; i++)
		len += scnprintf(buffer + len, PAGE_SIZE - len,
			"addr:0x%04x = 0x%02x\n",
			chip->reg.addr + i, buf[i]);

	return len;
}

static ssize_t p9415_reg_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t count)
{
	int ret = 0;
	int i = 0;
	u8 regs_data[REG_DUMP_MAX] = {0};
	char *tmp_data = NULL;
	char *reg_data = NULL;
	struct i2c_client *client = to_i2c_client(dev);
	struct p9415_dev *chip = (struct p9415_dev *)i2c_get_clientdata(client);

	if (!chip->reg.size) {
		dev_err(chip->dev, "%s: invalid parameters\n", __func__);
		return -EINVAL;
	}
	if (!p9415_get_pg_irq_status(chip))
		return -ENODEV;
	tmp_data = kzalloc(strlen(buf) + 1, GFP_KERNEL);
	if (!tmp_data)
		return -ENOMEM;
	strncpy(tmp_data, buf, strlen(buf));
	while (tmp_data && i < chip->reg.size) {
		reg_data = strsep(&tmp_data, " ");
		if (*reg_data) {
			ret = kstrtou8(reg_data, 0, &regs_data[i]);
			if (ret)
				break;
			i++;
		}
	}

	if (i != chip->reg.size || ret) {
		ret = -EINVAL;
		goto out;
	}

	ret = chip->bus.write_buf(chip, chip->reg.addr, regs_data,
		chip->reg.size);
	if (ret) {
		dev_err(chip->dev,
			"%s: Failed to write reg: %d\n", __func__,  ret);
		goto out;
	}
	ret = count;
out:
	kfree(tmp_data);
	return ret;
}

static ssize_t p9415_addr_show(struct device *dev,
		struct device_attribute *attr,
		char *buffer)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9415_dev *chip = (struct p9415_dev *)i2c_get_clientdata(client);

	return scnprintf(buffer, PAGE_SIZE, "addr:0x%04x size:%d\n",
			chip->reg.addr, chip->reg.size);
}

static ssize_t p9415_addr_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t count)
{
	unsigned int data[2] = {0};
	struct i2c_client *client = to_i2c_client(dev);
	struct p9415_dev *chip = (struct p9415_dev *)i2c_get_clientdata(client);

	if (sscanf(buf, "%x %x", &data[0], &data[1]) != 2)
		return -EINVAL;
	chip->reg.addr = data[0];
	chip->reg.size = data[1];

	return count;
}

static ssize_t p9415_fw_update_show(struct device *dev,
		struct device_attribute *attr,
		char *buffer)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9415_dev *chip = (struct p9415_dev *)i2c_get_clientdata(client);

	return scnprintf(buffer, PAGE_SIZE, "%d\n", chip->fw_dl_en);
}

static ssize_t p9415_config_update_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9415_dev *chip = (struct p9415_dev *)i2c_get_clientdata(client);
	char config_name[128] = { 0 };
	char mtp_name[128] = { 0 };
	int cnt = 0;
	const uint32_t config_table_addr = 0x5E00;

	cnt = sscanf(buf, "%s %s", mtp_name, config_name);
	dev_info(chip->dev, "%s: MTP:%s\n", __func__, mtp_name);
	dev_info(chip->dev, "%s: P9415 Target configuration table:%s\n",
		__func__, config_name);

	if (cnt != 2) {
		dev_err(chip->dev, "%s: Format:echo <mtp_name> <config_name> > config_update\n",
					__func__);
		return count;
	}

	p9415_mtp_update(chip, mtp_name, config_name, config_table_addr);
	return count;
}

static ssize_t p9415_fw_update_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9415_dev *chip = (struct p9415_dev *)i2c_get_clientdata(client);
	char fw_name[128] = { 0 };
	char mtp_name[128] = { 0 };
	int cnt = 0;

	cnt = sscanf(buf, "%s %s", mtp_name, fw_name);
	dev_info(chip->dev, "%s: MTP:%s\n", __func__, mtp_name);
	dev_info(chip->dev, "%s: P9415 Target FW:%s\n", __func__, fw_name);

	if (cnt != 2) {
		dev_err(chip->dev, "%s: Format:echo <mtp_name> <firmware_name> > fw_update\n",
					__func__);
		return count;
	}

	p9415_mtp_update(chip, mtp_name, fw_name, 0);
	return count;
}

static ssize_t p9415_iout_cal_show(struct device *dev,
		struct device_attribute *attr,
		char *buffer)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9415_dev *chip = (struct p9415_dev *)i2c_get_clientdata(client);
	uint32_t cal = 0;

	mutex_lock(&chip->sys_lock);
	if (p9415_get_pg_irq_status(chip))
		cal = p9415_get_calibration_value(chip);
	mutex_unlock(&chip->sys_lock);
	return scnprintf(buffer, PAGE_SIZE, "%d\n", cal);
}

static ssize_t p9415_iout_cal_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9415_dev *chip = (struct p9415_dev *)i2c_get_clientdata(client);
	char mtp_name[128] = { 0 };
	uint32_t cal = 0;
	int cnt = 0;
	union calibration_data cal_data;
	uint32_t index = 0;
	const uint32_t default_cal = 4096;
	const uint32_t section_size = 128;

	cnt = sscanf(buf, "%s %d", mtp_name, &cal);
	dev_info(chip->dev, "%s: MTP:%s\n", __func__, mtp_name);
	dev_info(chip->dev, "%s: Calibration value:%d (0x%x)\n", __func__,
		cal, cal);

	if (cnt != 2) {
		dev_err(chip->dev, "%s: Format:echo <mtp_name> <calibration value> > iout_cal\n",
					__func__);
		return count;
	}

	if (cal == 0)
		cal = default_cal;

	cal_data.I_Gain1 = cal;
	cal_data.I_Gain2 = cal;
	cal_data.I_Gain3 = cal;
	cal_data.I_Offset1 = 0;
	cal_data.I_Offset2 = 0;
	cal_data.I_Offset3 = 0;
	cal_data.I_CoeffThd1 = 550;
	cal_data.I_CoeffThd2 = 650;
	cal_data.I_CoeffThd3 = 750;
	cal_data.I_CoeffThd4 = 850;
	cal_data.I_Gain_Thd_H = 4301;
	cal_data.I_Gain_Thd_L = 3891;
	/* 128 bytes per section. Initialize to 0xFF to empty data byte. */
	for (index = 24; index < section_size; index++)
		cal_data.raw_data[index] = 0xFF;

	p9415_mtp_update_data(chip, mtp_name, (uint8_t *) &cal_data,
		128, 0x5F00);
	return count;
}

static ssize_t p9415_fw_dl_en_show(struct device *dev,
		struct device_attribute *attr,
		char *buffer)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9415_dev *chip = (struct p9415_dev *)i2c_get_clientdata(client);

	return scnprintf(buffer, PAGE_SIZE, "%d\n", chip->fw_dl_en);
}

static ssize_t p9415_fw_dl_en_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9415_dev *chip = (struct p9415_dev *)i2c_get_clientdata(client);
	uint32_t en = 0;

	if (kstrtouint(buf, 0, &en))
		return -EINVAL;

	if (en == 1) {
		chip->fw_dl_en = true;
		pinctrl_select_state(chip->wpc_pinctrl, chip->fw_dl_enable);
	} else {
		chip->fw_dl_en = false;
		pinctrl_select_state(chip->wpc_pinctrl, chip->fw_dl_disable);
	}
	return count;
}

static ssize_t p9415_en_chk_tx_cap_show(struct device *dev,
		struct device_attribute *attr,
		char *buffer)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9415_dev *chip = (struct p9415_dev *)i2c_get_clientdata(client);

	return scnprintf(buffer, PAGE_SIZE, "%d\n", chip->en_chk_tx_cap);
}

static ssize_t p9415_en_chk_tx_cap_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9415_dev *chip = (struct p9415_dev *)i2c_get_clientdata(client);
	uint32_t en = 0;

	if (kstrtouint(buf, 0, &en))
		return -EINVAL;

	if (en == 1)
		chip->en_chk_tx_cap = true;
	else
		chip->en_chk_tx_cap = false;
	return count;
}


static DEVICE_ATTR(version, 0444, p9415_version_show, NULL);
static DEVICE_ATTR(id, 0444, p9415_id_show, NULL);
static DEVICE_ATTR(pg_stat, 0444, p9415_pg_stat_show, NULL);
static DEVICE_ATTR(fw_ver, 0444, p9415_fw_ver_show, NULL);
static DEVICE_ATTR(power_switch, 0644, NULL, p9415_power_switch_store);
static DEVICE_ATTR(vrect_adc, 0444, p9415_vrect_adc_show, NULL);
static DEVICE_ATTR(iout_adc, 0444, p9415_iout_show, NULL);
static DEVICE_ATTR(iout_raw_adc, 0444, p9415_iout_raw_show, NULL);
static DEVICE_ATTR(vout_adc, 0444, p9415_vout_adc_show, NULL);
static DEVICE_ATTR(vout_set, 0644, p9415_vout_set_show,
	p9415_vout_set_store);
static DEVICE_ATTR(temp_adc, 0444, p9415_temp_adc_show, NULL);
static DEVICE_ATTR(tx_signal_strength, 0444, p9415_tx_signal_strength_show,
	NULL);
static DEVICE_ATTR(tx_id, 0444, p9415_tx_id_show, NULL);
static DEVICE_ATTR(irq_stat, 0444, p9415_irq_stat_show, NULL);
static DEVICE_ATTR(id_authen_status, 0444, p9415_id_authen_status_show, NULL);
static DEVICE_ATTR(dev_authen_status, 0444, p9415_dev_authen_status_show, NULL);
static DEVICE_ATTR(over_reason, 0444, p9415_over_reason_show, NULL);
static DEVICE_ATTR(fod_regs, 0644, p9415_fod_regs_show, p9415_fod_regs_store);
static DEVICE_ATTR(registers_dump, 0444, p9415_regs_dump_show, NULL);
static DEVICE_ATTR(reg, 0644, p9415_reg_show, p9415_reg_store);
static DEVICE_ATTR(addr, 0644, p9415_addr_show, p9415_addr_store);
static DEVICE_ATTR(fw_dl_en, 0644, p9415_fw_dl_en_show, p9415_fw_dl_en_store);
static DEVICE_ATTR(config_update, 0200, NULL, p9415_config_update_store);
static DEVICE_ATTR(fw_update, 0644, p9415_fw_update_show,
	p9415_fw_update_store);
static DEVICE_ATTR(iout_cal, 0644, p9415_iout_cal_show, p9415_iout_cal_store);
static DEVICE_ATTR(en_chk_tx_cap, 0644, p9415_en_chk_tx_cap_show,
	p9415_en_chk_tx_cap_store);

static struct attribute *p9415_sysfs_attrs[] = {
	&dev_attr_version.attr,
	&dev_attr_id.attr,
	&dev_attr_pg_stat.attr,
	&dev_attr_fw_ver.attr,
	&dev_attr_power_switch.attr,
	&dev_attr_vrect_adc.attr,
	&dev_attr_iout_adc.attr,
	&dev_attr_iout_raw_adc.attr,
	&dev_attr_vout_adc.attr,
	&dev_attr_vout_set.attr,
	&dev_attr_temp_adc.attr,
	&dev_attr_tx_signal_strength.attr,
	&dev_attr_tx_id.attr,
	&dev_attr_irq_stat.attr,
	&dev_attr_id_authen_status.attr,
	&dev_attr_dev_authen_status.attr,
	&dev_attr_over_reason.attr,
	&dev_attr_fod_regs.attr,
	&dev_attr_registers_dump.attr,
	&dev_attr_reg.attr,
	&dev_attr_addr.attr,
	&dev_attr_fw_dl_en.attr,
	&dev_attr_config_update.attr,
	&dev_attr_fw_update.attr,
	&dev_attr_iout_cal.attr,
	&dev_attr_en_chk_tx_cap.attr,
	NULL,
};

static const struct attribute_group p9415_sysfs_group_attrs = {
	.attrs = p9415_sysfs_attrs,
};

static const struct of_device_id match_table[] = {
	{.compatible = "IDT,idt_wireless_power",},
	{.compatible = "IDT,p9415",},
	{},
};

MODULE_DEVICE_TABLE(of, match_table);

static const struct i2c_device_id p9415_dev_id[] = {
	{"idt_wireless_power", 0},
	{"p9415", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, p9415_dev_id);

/* first step: define regmap_config */
static const struct regmap_config p9415_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.max_register = 0xFFFF,
};

/*
 * The header implicitly provides the size of the message
 * contained in the Packet. The number of bytes in a message
 * is calculated from the value contained in the header of
 * the Packet.
 * Header  Message Size  Comment
 * 0x00...0x1F  1 + (Header - 0) / 32    1 * 32 messages(size 1)
 * 0x20...0x7F  2 + (Header - 32) / 16   6 * 16 messages(size 2...7)
 * 0x80...0xDF  8 + (Header - 128) / 8   12 * 8 messages(size 8...19)
 * 0xE0...0xFF  20 + (Header - 224) / 4  8 * 4 messages(size 20...27)
 */
static int p9415_extract_packet_size(u8 hdr)
{
	if (hdr < 0x20)
		return 1;
	if (hdr < 0x80)
		return (2 + ((hdr - 0x20) >> 4));
	if (hdr < 0xe0)
		return (8 + ((hdr - 0x80) >> 3));
	return (20 + ((hdr - 0xe0) >> 2));
}

static void p9415_write_fod(struct p9415_dev *chip,
	enum charger_type chr_type)
{
	int ret;
	u8 *fod_data = NULL;
	u8 fod_read[FOD_COEF_PARAM_LENGTH];


	if (chr_type == WIRELESS_CHARGER_15W &&
			chip->bpp_plus_15w_fod_num == FOD_COEF_PARAM_LENGTH)
		fod_data = (u8 *)(chip->bpp_plus_15w_fod);
	else if (chr_type == WIRELESS_CHARGER_10W &&
			chip->epp_10w_fod_num == FOD_COEF_PARAM_LENGTH)
		fod_data = (u8 *)(chip->epp_10w_fod);
	else if ((chr_type == WIRELESS_CHARGER_5W ||
			chr_type == WIRELESS_CHARGER_DEFAULT)
			&& chip->bpp_5w_fod_num == FOD_COEF_PARAM_LENGTH)
		fod_data = (u8 *)(chip->bpp_5w_fod);

	if (fod_data == NULL)
		goto no_fod_data;

	/*
	 * Manual power switch or automatic power switch may call this function
	 * at the same time, so add fod mutex lock to prevent concurrent access.
	 */
	mutex_lock(&chip->fod_lock);
	dev_info(chip->dev, "%s: chr_type: %d, writing FOD.\n",
		__func__, chr_type);
	ret = chip->bus.write_buf(chip, REG_FOD_COEF_ADDR, fod_data, FOD_COEF_PARAM_LENGTH);
	if (ret) {
		dev_err(chip->dev, "%s: Failed to write fod data: %d\n",
			__func__, ret);
		goto out;
	}

	ret = chip->bus.read_buf(chip, REG_FOD_COEF_ADDR, fod_read, FOD_COEF_PARAM_LENGTH);
	if (ret) {
		dev_err(chip->dev, "%s: Failed to read fod data: %d\n",
			__func__, ret);
		goto out;
	}

	if (memcmp(fod_data, fod_read, FOD_COEF_PARAM_LENGTH) == 0)
		goto out;

	dev_warn(chip->dev, "%s: compare error, chr_type:%d, fod read:%d %d, %d %d, %d %d, %d %d, %d %d, %d %d",
			__func__, chr_type, fod_read[0], fod_read[1],
			fod_read[2], fod_read[3], fod_read[4], fod_read[5],
			fod_read[6], fod_read[7], fod_read[8], fod_read[9],
			fod_read[10], fod_read[11]);

out:
	mutex_unlock(&chip->fod_lock);
	return;
no_fod_data:
	dev_warn(chip->dev, "%s: Fod data not set.\n", __func__);
}

static void p9415_device_auth_req(struct p9415_dev *chip)
{
	int ret;

	/* set device authentication req */
	ret = chip->bus.write(chip, REG_COMMAND, SEND_DEVICE_AUTH);
	if (ret)
		dev_err(chip->dev, "%s: Failed to write command reg: %d\n",
			__func__, ret);
}

static void p9415_get_tx_capability(struct p9415_dev *chip)
{
	uint8_t header = 0;
	int length;
	uint8_t data_list[16] = { 0 };
	int ret = 0;
	uint32_t tx_max_iout;

	ret = chip->bus.read(chip, REG_BCHEADER_ADDR, &header);
	if (ret) {
		dev_err(chip->dev, "%s: Failed to read bcheader addr\n",
			__func__);
		return;
	}

	length = p9415_extract_packet_size(header);
	ret = chip->bus.read_buf(chip, REG_BCDATA_ADDR, data_list, length);
	if (ret) {
		dev_err(chip->dev, "%s: Failed to read bcdata addr\n",
			__func__);
		return;
	}

	/* handle tx capability */
	if (header == 0x3F &&  data_list[0] == 0x63) {
		chip->tx_max_vbridge = data_list[1]*1000;	/* mV */

		/* Follow the rule defining PPV (Potential Power Value)
		 * in Qi spec. Each step is 0.5W.
		 */
		chip->tx_max_power = data_list[2]/2;

		tx_max_iout = chip->tx_max_power*1000*10000 / chip->vout_15w;
		tx_max_iout = (tx_max_iout + 5)/10*1000;

		dev_info(chip->dev, "%s: Tx max. vbridge: %d mV\n",
			__func__, chip->tx_max_vbridge);
		dev_info(chip->dev, "%s: Tx max. power: %d W\n",
			__func__, chip->tx_max_power);
		dev_info(chip->dev, "%s: Tx max. iout: %d mA\n",
			__func__, tx_max_iout/1000);

		if (tx_max_iout < 1500000) {
			chip->step_load.step_max_ua = tx_max_iout;
			chip->adaptive_current_limit.max_current_limit =
				tx_max_iout/1000;
		} else {
			chip->step_load.step_max_ua =
				chip->step_load.bpp_plus_max_ua;
			chip->adaptive_current_limit.max_current_limit =
				chip->adaptive_current_limit.bpp_plus_max_ma;
		}

		p9415_enable_fast_charging(chip);
	}
}

static void p9415_sendpkt(struct p9415_dev *chip, struct propkt_type *pkt)
{
	/* include header by +1 */
	int length = p9415_extract_packet_size(pkt->header) + 1;
	int ret = 0;

	/*  write data into proprietary packet buffer */
	ret = chip->bus.write_buf(chip, REG_PROPPKT_ADDR, (u8 *)pkt, length);
	if (ret) {
		dev_err(chip->dev, "%s: Failed to write proprietary packet buffer\n",
					__func__);
		return;
	}

	/* send proprietary packet */
	ret = chip->bus.write(chip, REG_COMMAND, SENDPROPP);
	if (ret)
		dev_err(chip->dev, "%s: Failed to write command %d\n",
			__func__, SENDPROPP);
}

/* To request Tx max. vbridge and Tx max. power */
static void p9415_request_tx_capability(struct p9415_dev *chip)
{
	struct propkt_type propkt;

	propkt.header = PROPRIETARY18;
	propkt.cmd = BC_GET_TX_CAP;

	dev_info(chip->dev, "%s: start\n", __func__);

	p9415_sendpkt(chip, &propkt);
}

static int p9415_send_eop(struct p9415_dev *chip, u8 reason)
{
	int ret;

	dev_info(chip->dev,
			"%s: Send EOP reason=%d\n", __func__, reason);
	ret = chip->bus.write(chip, REG_EPT, reason);
	if (ret) {
		dev_err(chip->dev,
			"%s: Failed to write EPT: %d\n", __func__, ret);
		goto out;
	}
	ret = chip->bus.write(chip, REG_COMMAND, SENDEOP);
	if (ret) {
		dev_err(chip->dev,
			"%s: Failed to send EOP: %d\n", __func__, ret);
		goto out;
	}

out:
	return ret;
}

static void p9415_over_handle(struct p9415_dev *chip, uint32_t irq_src)
{
	u8 reason = 0;

	dev_warn(chip->dev, "%s: Received OVER INT: 0x%02x\n",
				__func__, irq_src);
	if (irq_src & P9415_INT_OV_TEMP)
		reason = EOP_OVER_TEMP;
	else if (irq_src & P9415_INT_OV_VOLT)
		reason = EOP_OVER_VOLT;
	else
		reason = EOP_OVER_CURRENT;
	dev_warn(chip->dev, "%s: over_reason:%d\n", __func__, reason);

	chip->over_reason = reason;
	p9415_send_eop(chip, reason);
}

static void p9415_enable_fast_charging(struct p9415_dev *chip)
{
	cancel_delayed_work_sync(&chip->fast_charging_work);

	atomic_set(&chip->vswitch_done, 0);
	atomic_set(&chip->vswitch_retry, 0);

	schedule_delayed_work(&chip->fast_charging_work, 0);
}

static void p9415_convert_intflag(struct p9415_dev *chip,
	uint32_t intflag)
{
	if (intflag & P9415_INT_NTCOVERTEMP)
		dev_info(chip->dev, "%s: NTCOVERTEMP\n", __func__);
	if (intflag & P9415_INT_VRECTON)
		dev_info(chip->dev, "%s: VRECTON\n", __func__);
	if (intflag & P9415_INT_VSWITCH_FAILED)
		dev_info(chip->dev, "%s: VSWITCH_FAILED\n", __func__);
	if (intflag & P9415_INT_SLEEP_MODE)
		dev_info(chip->dev, "%s: SLEEP_MODE\n", __func__);
	if (intflag & P9415_INT_ID_AUTH_SUCCESS)
		dev_info(chip->dev, "%s: ID_AUTH_SUCCESS\n", __func__);
	if (intflag & P9415_INT_ID_AUTH_FAIL)
		dev_info(chip->dev, "%s: ID_AUTH_FAIL\n", __func__);
	if (intflag & P9415_INT_SEND_PKT_SUCCESS)
		dev_info(chip->dev, "%s: BCSUCCESS\n", __func__);
	if (intflag & P9415_INT_SEND_PKT_TIMEOUT)
		dev_info(chip->dev, "%s: BCTIMEOUT\n", __func__);
	if (intflag & P9415_INT_DEVICE_AUTH_SUCCESS)
		dev_info(chip->dev, "%s: DEV_AUTH_SUCCESS\n", __func__);
	if (intflag & P9415_INT_DEVICE_AUTH_FAIL)
		dev_info(chip->dev, "%s: DEV_AUTH_FAIL\n", __func__);
	if (intflag & P9415_INT_LDO_OFF)
		dev_info(chip->dev, "%s: LDO OFF\n", __func__);
	if (intflag & P9415_INT_LDO_ON)
		dev_info(chip->dev, "%s: LDO ON\n", __func__);
	if (intflag & P9415_INT_MODE_CHANGE)
		dev_info(chip->dev, "%s: MODE_CHANGE\n", __func__);
	if (intflag & P9415_INT_TX_DATA_RECV)
		dev_info(chip->dev, "%s: DATA RECV\n", __func__);
	if (intflag & P9415_INT_VSWITCHSUCCESS)
		dev_info(chip->dev, "%s: VSWITCH_SUCCESS\n", __func__);
	if (intflag & P9415_INT_OV_TEMP)
		dev_info(chip->dev, "%s: OTP\n", __func__);
	if (intflag & P9415_INT_OV_VOLT)
		dev_info(chip->dev, "%s: OVP\n", __func__);
	if (intflag & P9415_INT_OV_CURRENT)
		dev_info(chip->dev, "%s: OCP\n", __func__);
}

static void p9415_write_ovp(struct p9415_dev *chip, uint8_t ovp)
{
	chip->bus.write(chip, REG_OVP_SEL, ovp);
}

irqreturn_t p9415_int_handler(int irq, void *ptr)
{
	struct p9415_dev *chip = ptr;
	int ret = 0;
	uint32_t intflag = 0;
	union fw_ver *fw_ver = &chip->fw_ver;

	p9415_read_fw_ver(chip);
	if ((fw_ver->cust_code != 0x25) ||
		(fw_ver->cust_code == 0x25 && fw_ver->major_rev == 0x1 &&
		fw_ver->minor_rev < 0xb)) {
		if (!p9415_get_pg_irq_status(chip))
			goto out;
	}

	ret = p9415_read4(chip, REG_INTFLAG, &intflag);
	if (ret) {
		dev_err(chip->dev, "%s: Failed to read INTFLAG\n", __func__);
		goto out;
	}
	dev_info(chip->dev, "%s: INTFLAG:0x%04x\n", __func__, intflag);
	p9415_convert_intflag(chip, intflag);

	ret = p9415_write4(chip, REG_INT_CLEAR, intflag);
	if (ret) {
		dev_err(chip->dev, "%s: Fail to clear IRQ\n", __func__);
		goto out;
	}

	ret = chip->bus.write(chip, REG_COMMAND, CLRINT);
	if (ret) {
		dev_err(chip->dev, "%s: Failed to reset INT\n", __func__);
		goto out;
	}

	if (intflag & P9415_INT_VRECTON) {
		uint8_t ss;
		uint64_t vrect;

		ss = p9415_get_tx_signal_strength(chip);
		vrect = p9415_get_vrect_adc(chip);
		dev_info(chip->dev, "%s: Received VRECTON, online (%d %dmV)\n",
			__func__, ss, vrect);
	}

	if (intflag & P9415_INT_ID_AUTH_FAIL) {
		chip->tx_authen_complete = true;
		if (chip->rx_mode == RX_MODE_BPP &&
			chip->charger == WIRELESS_CHARGER_DEFAULT &&
			cancel_delayed_work(&chip->bpp_switch_work))
			schedule_delayed_work(&chip->bpp_switch_work, 0);
	}

	if (intflag & P9415_INT_ID_AUTH_SUCCESS &&
		!chip->tx_id_authen_status) {
		chip->tx_id_authen_status = true;
		chip->dev_auth_retry = 0;
		p9415_device_auth_req(chip);
	}

	if (intflag & P9415_INT_DEVICE_AUTH_FAIL) {
		if (chip->dev_auth_retry++ < DEV_AUTH_RETRY) {
			dev_info(chip->dev, "%s: dev auth fail, retry = %d\n",
				__func__, chip->dev_auth_retry);
			p9415_device_auth_req(chip);
		} else {
			chip->tx_authen_complete = true;
			if (cancel_delayed_work(&chip->bpp_switch_work))
				schedule_delayed_work(&chip->bpp_switch_work, 0);
		}
	}

	if (intflag & P9415_INT_DEVICE_AUTH_SUCCESS &&
		!chip->tx_dev_authen_status) {
		chip->tx_dev_authen_status = true;
		chip->tx_authen_complete = true;
		chip->send_cmd_retry = 0;
		/*
		 * Initialize step_max_ua according
		 * according tx capability or
		 * bpp_plus_max_ua.
		 */
		if (chip->rx_mode == RX_MODE_BPP) {
			if (chip->en_chk_tx_cap)
				p9415_request_tx_capability(chip);
			else {
				struct adaptive_current_limit *adaptive =
					&chip->adaptive_current_limit;

				chip->step_load.step_max_ua =
					chip->step_load.bpp_plus_max_ua;
				adaptive->max_current_limit =
					adaptive->bpp_plus_max_ma;
				p9415_enable_fast_charging(chip);
			}
		}
	}

	if (intflag & P9415_INT_TX_DATA_RECV)
		p9415_get_tx_capability(chip);

	if (intflag & P9415_INT_SEND_PKT_TIMEOUT) {
		chip->send_cmd_retry++;
		if (chip->send_cmd_retry < SEND_CMD_RETRY)
			p9415_request_tx_capability(chip);
		else if (cancel_delayed_work(&chip->bpp_switch_work))
			schedule_delayed_work(&chip->bpp_switch_work, 0);
	}

	if (intflag & P9415_INT_VSWITCHSUCCESS) {
		uint64_t vrect;

		p9415_write_ovp(chip, OVP_SEL_1_21V);
		cancel_delayed_work_sync(&chip->bpp_switch_work);
		cancel_delayed_work_sync(&chip->fast_charging_work);

		vrect = p9415_get_vrect_adc(chip);
		dev_info(chip->dev, "%s: Vrect:%dmV\n", __func__, vrect);

		atomic_set(&chip->vswitch_done, 1);
		schedule_delayed_work(&chip->fast_charging_work, 0);
	}

	if (intflag & P9415_INT_VSWITCH_FAILED)
		atomic_inc(&chip->vswitch_retry);

	if (intflag & P9415_INT_LIMIT_MASK)
		p9415_over_handle(chip, intflag);

out:
	return IRQ_HANDLED;
}

static void p9415_attached_vbus(struct p9415_dev *chip)
{
	uint32_t inten = 0;
	uint64_t vrect;

	/* Update INTEN.
	 * Rx FW has default enabled interrupts and some of
	 * them are not required.
	 * Only enable required interrupt source.
	 */
	p9415_read4(chip, REG_INTEN, &inten);
	inten |= (P9415_INT_VSWITCHSUCCESS |
		P9415_INT_VSWITCH_FAILED);

	inten &= ~(P9415_INT_LDO_OFF | P9415_INT_LDO_ON |
		P9415_INT_MODE_CHANGE | P9415_INT_SLEEP_MODE |
		P9415_INT_NTCOVERTEMP);
	p9415_write4(chip, REG_INTEN, inten);

	p9415_read4(chip, REG_INTEN, &inten);
	dev_info(chip->dev, "%s: INTEN:0x%x\n", __func__, inten);

	vrect = p9415_get_vrect_adc(chip);
	dev_info(chip->dev, "%s: Vrect:%dmV\n", __func__, vrect);

	p9415_enable_charge_flow(chip, true);
}

static void p9415_detached_vbus(struct p9415_dev *chip)
{
	dev_dbg(chip->dev, "%s\n", __func__);
	p9415_enable_charge_flow(chip, false);
}

static bool p9415_get_pg_irq_status(struct p9415_dev *chip)
{
	return gpiod_get_value(chip->pg_gpio);
}

static irqreturn_t p9415_pg_handler(int irq, void *ptr)
{
	bool vbus_en = false;
	struct p9415_dev *chip = ptr;

	mutex_lock(&chip->irq_lock);
	vbus_en = p9415_get_pg_irq_status(chip);
	dev_info(chip->dev, "%s: vbus_en = %d\n", __func__, vbus_en);
	if (vbus_en)
		p9415_attached_vbus(chip);
	else
		p9415_detached_vbus(chip);
	mutex_unlock(&chip->irq_lock);

	return IRQ_HANDLED;
}

static int p9415_parse_dt(struct p9415_dev *chip)
{
	struct device_node *dt = chip->client->dev.of_node;
	struct i2c_client *client = chip->client;
	const struct of_device_id *match = NULL;
	struct adaptive_current_limit *adaptive =
		&chip->adaptive_current_limit;
	int ret;
	u8 fod_data[FOD_COEF_PARAM_LENGTH];

	if (!dt) {
		dev_err(chip->dev, "%s: Device does not have associated DT data\n",
				__func__);
		return -EINVAL;
	}

	dev_err(chip->dev, "%s: Device have associated DT data\n", __func__);

	match = of_match_device(match_table, &client->dev);
	if (!match) {
		dev_err(chip->dev, "%s: Unknown device model\n", __func__);
		return -EINVAL;
	}
	of_property_read_u16(dt, "vout_10w", &chip->vout_10w);
	of_property_read_u16(dt, "vout_15w", &chip->vout_15w);

	of_property_read_u32_array(dt, "wpc-mivr", chip->wpc_mivr,
			ARRAY_SIZE(chip->wpc_mivr));

	chip->en_chk_tx_cap = of_property_read_bool(dt,
		"en_chk_tx_cap");

	chip->bpp_5w_fod_num = of_property_count_elems_of_size(dt, "bpp-5w-fod", sizeof(u8));
	if (chip->bpp_5w_fod_num != FOD_COEF_PARAM_LENGTH) {
		dev_err(chip->dev, "%s: Incorrect num of 5w fod data",
			__func__);
		chip->bpp_5w_fod_num = 0;
	} else {
		ret = of_property_read_u8_array(dt, "bpp-5w-fod", fod_data, sizeof(fod_data));
		if (ret == 0) {
			memcpy(chip->bpp_5w_fod, fod_data, sizeof(fod_data));
			dev_info(chip->dev, "%s: 5w fod data:%x %x, %x %x, %x %x, %x %x, %x %x, %x %x",
					__func__,
					chip->bpp_5w_fod[0].gain, chip->bpp_5w_fod[0].offs,
					chip->bpp_5w_fod[1].gain, chip->bpp_5w_fod[1].offs,
					chip->bpp_5w_fod[2].gain, chip->bpp_5w_fod[2].offs,
					chip->bpp_5w_fod[3].gain, chip->bpp_5w_fod[3].offs,
					chip->bpp_5w_fod[4].gain, chip->bpp_5w_fod[4].offs,
					chip->bpp_5w_fod[5].gain, chip->bpp_5w_fod[5].offs);
		} else
			dev_err(chip->dev, "%s: Failed to parse bpp-5w-fod.\n",
				__func__);
	}

	chip->epp_10w_fod_num = of_property_count_elems_of_size(dt, "epp-10w-fod", sizeof(u8));
	if (chip->epp_10w_fod_num != FOD_COEF_PARAM_LENGTH) {
		dev_err(chip->dev, "%s: Incorrect num of 10w fod data",
			__func__);
		chip->epp_10w_fod_num = 0;
	} else {
		ret = of_property_read_u8_array(dt, "epp-10w-fod", fod_data, sizeof(fod_data));
		if (ret == 0) {
			memcpy(chip->epp_10w_fod, fod_data, sizeof(fod_data));
			dev_info(chip->dev, "%s: 10w fod data:%x %x, %x %x, %x %x, %x %x, %x %x, %x %x",
					__func__,
					chip->epp_10w_fod[0].gain, chip->epp_10w_fod[0].offs,
					chip->epp_10w_fod[1].gain, chip->epp_10w_fod[1].offs,
					chip->epp_10w_fod[2].gain, chip->epp_10w_fod[2].offs,
					chip->epp_10w_fod[3].gain, chip->epp_10w_fod[3].offs,
					chip->epp_10w_fod[4].gain, chip->epp_10w_fod[4].offs,
					chip->epp_10w_fod[5].gain, chip->epp_10w_fod[5].offs);
		} else
			dev_err(chip->dev, "%s: Failed to parse epp-10w-fod.\n",
				__func__);
	}

	chip->bpp_plus_15w_fod_num = of_property_count_elems_of_size(dt,
		"bpp-plus-15w-fod", sizeof(u8));
	if (chip->bpp_plus_15w_fod_num != FOD_COEF_PARAM_LENGTH) {
		dev_err(chip->dev, "%s: Incorrect num of 15w fod data",
			__func__);
		chip->bpp_plus_15w_fod_num = 0;
	} else {
		ret = of_property_read_u8_array(dt, "bpp-plus-15w-fod",
				fod_data, sizeof(fod_data));
		if (ret == 0) {
			memcpy(chip->bpp_plus_15w_fod, fod_data,
				sizeof(fod_data));
			dev_info(chip->dev, "%s: 15w fod data:%x %x, %x %x, %x %x, %x %x, %x %x, %x %x",
					__func__,
					chip->bpp_plus_15w_fod[0].gain, chip->bpp_plus_15w_fod[0].offs,
					chip->bpp_plus_15w_fod[1].gain, chip->bpp_plus_15w_fod[1].offs,
					chip->bpp_plus_15w_fod[2].gain, chip->bpp_plus_15w_fod[2].offs,
					chip->bpp_plus_15w_fod[3].gain, chip->bpp_plus_15w_fod[3].offs,
					chip->bpp_plus_15w_fod[4].gain, chip->bpp_plus_15w_fod[4].offs,
					chip->bpp_plus_15w_fod[5].gain, chip->bpp_plus_15w_fod[5].offs);
		} else
			dev_err(chip->dev, "%s: Failed to parse bpp-plus-15w-fod.\n",
				__func__);
	}

	ret = of_property_read_s32(dt, "throttle_threshold",
				&chip->throttle_threshold);
	if (ret == 0) {
		dev_info(chip->dev, "%s: BPP+ power throttle threshold %dC\n",
			__func__, chip->throttle_threshold);

		ret = of_property_read_s32(dt, "throttle_hysteresis_threshold",
					&chip->throttle_hysteresis_threshold);
		if (ret)
			chip->throttle_hysteresis_threshold =
				chip->throttle_threshold - 3;
	} else {
		dev_info(chip->dev, "%s: Not support BPP+ power throttle\n",
			__func__);
		chip->throttle_threshold = -1;
	}

	ret = of_property_read_u32(dt, "EPT_work_delay",
				&chip->EPT_work_delay);
	if (ret < 0)
		chip->EPT_work_delay = DEFAULT_EPT_WORK_DELAY;
	dev_info(chip->dev, "%s: EPT_work_delay: %d ms\n", __func__,
		chip->EPT_work_delay);

	of_property_read_s32(dt, "start_ma", &chip->step_load.start_ma);
	of_property_read_s32(dt, "step_ma", &chip->step_load.step_ma);
	of_property_read_s32(dt, "bpp_plus_max_current",
		&chip->step_load.bpp_plus_max_ua);
	of_property_read_u32(dt, "step_interval",
		&chip->step_load.step_interval);

	of_property_read_s32(dt, "adaptive_start_soc", &adaptive->start_soc);
	of_property_read_s32(dt, "adaptive_interval", &adaptive->interval);
	of_property_read_s32(dt, "adaptive_bpp_plus_max_ma",
		&adaptive->bpp_plus_max_ma);
	of_property_read_s32(dt, "adaptive_min_current_limit",
		&adaptive->min_current_limit);
	of_property_read_s32(dt, "adaptive_margin", &adaptive->margin);
	of_property_read_s32(dt, "adaptive_start_ma", &adaptive->start_ma);
	return 0;
}

static int p9415_vout_enable(struct charger_device *chg_dev, bool en)
{
	struct p9415_dev *chip = dev_get_drvdata(&chg_dev->dev);
	uint16_t vout = 0;

	dev_info(chip->dev, "%s: %s\n", __func__, en ? "true" : "false");

	if (chip->vout_en == en) {
		dev_info(chip->dev, "%s: is_enabled status same as %d\n",
				__func__, en);
		return 0;
	}

	if (p9415_get_pg_irq_status(chip)) {
		vout = p9415_get_vout_adc(chip);

		if (!chip->vout_en && en) {
			chip->bus.write(chip, REG_COMMAND, LDOTGL);
			p9415_attached_vbus(chip);
		} else if (!en && vout) {
			chip->bus.write(chip, REG_COMMAND, LDOTGL);
			p9415_detached_vbus(chip);
		} else
			dev_err(chip->dev, "%s: state is not sync", __func__);
	} else {
		dev_info(chip->dev, "%s: PG isn't ready\n", __func__);
	}

	return 0;
}

static int p9415_set_wpc_en(struct charger_device *chg_dev, bool en)
{
	struct p9415_dev *chip = dev_get_drvdata(&chg_dev->dev);

	dev_info(chip->dev, "%s: %s\n", __func__, en ? "true" : "false");

	if (chip->wpc_en == en) {
		dev_info(chip->dev, "%s: is_enabled status same as %d\n",
				__func__, en);
		return 0;
	}
	chip->wpc_en = en;

	if (en) {
		pinctrl_select_state(chip->wpc_pinctrl, chip->wpc_enable);
	} else {
		usleep_range(10000, 20000);
		pinctrl_select_state(chip->wpc_pinctrl, chip->wpc_disable);
	}

	return 0;
}

static int p9415_set_vout_en(struct charger_device *chg_dev, bool en)
{
	struct p9415_dev *chip = dev_get_drvdata(&chg_dev->dev);

	dev_info(chip->dev, "%s: %s\n", __func__, en ? "true" : "false");

	if (!chip->support_sleep_en) {
		dev_dbg(chip->dev, "%s: Not support sleep pin control.\n",
				__func__);
		p9415_vout_enable(chg_dev, en);
		return 0;
	}

	if (chip->vout_en == en) {
		dev_info(chip->dev, "%s: is_enabled status same as %d\n",
				__func__, en);
		return 0;
	}
	chip->vout_en = en;

	if (en)
		pinctrl_select_state(chip->wpc_pinctrl, chip->sleep_disable);
	else
		pinctrl_select_state(chip->wpc_pinctrl, chip->sleep_enable);

	return 0;
}

static int p9415_request_io_port(struct p9415_dev *chip)
{
	struct i2c_client *client = chip->client;

	chip->wpc_pinctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR(chip->wpc_pinctrl)) {
		dev_err(chip->dev, "%s: Cannot find pinctrl for WPC\n",
			__func__);
		goto PINCTRL_ERR;
	}

	chip->pg_gpio = devm_gpiod_get_optional(chip->dev,
						"p9415-pg", GPIOD_IN);
	if (IS_ERR_OR_NULL(chip->pg_gpio)) {
		dev_err(chip->dev, "%s: Failed to request GPIO for PG\n",
			__func__);
		goto PINCTRL_ERR;
	}
	dev_info(chip->dev, "%s: Success request pg-gpio\n", __func__);

	/* handle pinctrl for nEN */
	chip->wpc_en = true;
	chip->wpc_enable = pinctrl_lookup_state(chip->wpc_pinctrl,
		"wpc_enable");
	if (IS_ERR(chip->wpc_enable)) {
		dev_err(chip->dev, "%s: Cannot find pinctrl wpc_enable!\n",
			__func__);
		goto PINCTRL_ERR;
	}

	chip->wpc_disable = pinctrl_lookup_state(chip->wpc_pinctrl,
		"wpc_disable");
	if (IS_ERR(chip->wpc_disable)) {
		dev_err(chip->dev, "%s: Cannot find pinctrl wpc_disable!\n",
			__func__);
		goto PINCTRL_ERR;
	}

	/* handle EPP_DISABLE */
	chip->epp_en = false;
	chip->epp_enable = pinctrl_lookup_state(chip->wpc_pinctrl,
		"epp_enable");
	if (IS_ERR(chip->epp_enable)) {
		dev_err(chip->dev, "%s: Cannot find pinctrl epp_enable!\n",
			__func__);
		goto PINCTRL_ERR;
	}
	chip->epp_disable = pinctrl_lookup_state(chip->wpc_pinctrl,
		"epp_disable");
	if (IS_ERR(chip->epp_disable)) {
		dev_err(chip->dev, "%s: Cannot find pinctrl epp_disable!\n",
			__func__);
		goto PINCTRL_ERR;
	}

	/* handle FW download enable pin*/
	chip->fw_dl_enable = pinctrl_lookup_state(chip->wpc_pinctrl,
		"fw_dl_enable");
	if (IS_ERR(chip->fw_dl_enable)) {
		dev_err(chip->dev, "%s: Cannot find pinctrl fw_dl_enable!\n",
			__func__);
		goto PINCTRL_ERR;
	}
	chip->fw_dl_disable = pinctrl_lookup_state(chip->wpc_pinctrl,
		"fw_dl_disable");
	if (IS_ERR(chip->fw_dl_disable)) {
		dev_err(chip->dev, "%s: Cannot find pinctrl fw_dl_disable!\n",
			__func__);
		goto PINCTRL_ERR;
	}

	/* handle sleep pin */
	chip->sleep_enable = pinctrl_lookup_state(chip->wpc_pinctrl,
		"sleep_enable");
	chip->sleep_disable = pinctrl_lookup_state(chip->wpc_pinctrl,
		"sleep_disable");
	if (IS_ERR_OR_NULL(chip->sleep_enable) ||
			IS_ERR_OR_NULL(chip->sleep_disable)) {
		chip->support_sleep_en = false;
		dev_info(chip->dev, "%s: Not support sleep pin control.\n",
			__func__);
	} else {
		chip->support_sleep_en = true;
		dev_info(chip->dev, "%s: Support sleep pin control.\n",
			__func__);
	}
	return 0;

PINCTRL_ERR:
	return -ENODEV;
}

static int p9415_register_irq(struct p9415_dev *chip)
{
	int ret = 0;
	struct device_node *of_node = chip->dev->of_node;

	chip->int_num = irq_of_parse_and_map(of_node, 0);
	if (chip->int_num <= 0)
		dev_err(chip->dev, "%s: Unable to get irq for INT\n", __func__);
	else
		dev_info(chip->dev, "%s: int_num:%d\n",
			__func__, chip->int_num);
	ret = devm_request_threaded_irq(chip->dev,
				chip->int_num, NULL, p9415_int_handler,
				IRQF_TRIGGER_LOW | IRQF_ONESHOT, IDT_INT,
				chip);
	if (ret == 0)
		disable_irq_nosync(chip->int_num);
	else
		dev_err(chip->dev, "%s: Request irq failed for int\n",
			__func__);

	chip->pg_num = irq_of_parse_and_map(of_node, 1);
	if (chip->pg_num <= 0)
		dev_err(chip->dev, "%s: Unable to get irq for PG\n", __func__);
	else
		dev_info(chip->dev, "%s: pg_num:%d\n", __func__, chip->pg_num);
	ret = devm_request_threaded_irq(chip->dev,
			chip->pg_num, NULL, p9415_pg_handler,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING |
			IRQF_ONESHOT, IDT_PG, chip);
	if (ret == 0)
		disable_irq_nosync(chip->pg_num);
	else
		dev_err(chip->dev, "%s: Request irq failed for PG\n", __func__);
	return ret;
}

static int p9415_update_charge_online(struct p9415_dev *chip, bool online)
{
	int ret;
	union power_supply_propval propval;

	struct power_supply *chg_psy = power_supply_get_by_name("charger");

	ret = power_supply_get_property(chg_psy,
		POWER_SUPPLY_PROP_ONLINE, &propval);
	if (ret < 0) {
		dev_err(chip->dev, "%s: get psy online failed, ret = %d\n",
			__func__, ret);
		return ret;
	}

	if (propval.intval == online) {
		dev_info(chip->dev, "%s: psy online status same as %d\n",
				__func__, online);
		return ret;
	}

	propval.intval = online;
	ret = power_supply_set_property(chg_psy,
		POWER_SUPPLY_PROP_ONLINE, &propval);
	if (ret < 0)
		dev_err(chip->dev, "%s: set psy online failed, ret = %d\n",
			__func__, ret);

	dev_info(chip->dev, "%s: online = %d\n", __func__, online);
	return ret;
}

static int p9415_set_charger_mivr(struct p9415_dev *chip,
		struct charger_device *chg1_dev, u32 uV)
{
	int ret = 0;

	ret = charger_dev_set_mivr(chg1_dev, uV);
	if (ret < 0)
		dev_err(chip->dev, "%s: Failed to set mivr\n", __func__);

	return ret;
}

static int p9415_set_wpc_mivr(struct p9415_dev *chip,
	enum  charger_type type)
{
	int ret = 0;
	bool is_en = false;
	bool power_path_en = !(type == CHARGER_UNKNOWN);
	struct charger_device *chg1_dev = get_charger_by_name("primary_chg");

	ret = charger_dev_is_powerpath_enabled(chg1_dev, &is_en);
	if (ret < 0) {
		dev_err(chip->dev, "%s: Fail to get power path\n", __func__);
		return ret;
	}

	if (is_en != power_path_en)
		charger_dev_enable_powerpath(chg1_dev, power_path_en);

	switch (type) {
	case WIRELESS_CHARGER_5W:
	case WIRELESS_CHARGER_DEFAULT:
		p9415_set_charger_mivr(chip, chg1_dev,
			chip->wpc_mivr[CHARGE_5W_MODE]);
		break;
	case WIRELESS_CHARGER_10W:
		p9415_set_charger_mivr(chip, chg1_dev,
			chip->wpc_mivr[CHARGE_10W_MODE]);
		break;
	case WIRELESS_CHARGER_15W:
		p9415_set_charger_mivr(chip, chg1_dev,
			chip->wpc_mivr[CHARGE_15W_MODE]);
		break;
	default:
		break;
	}

	return ret;
}

static int p9415_update_charge_type(struct p9415_dev *chip,
			enum charger_type type)
{
	int ret;
	union power_supply_propval propval;

	struct power_supply *chg_psy = power_supply_get_by_name("charger");
	struct charger_manager *cm =
		(struct charger_manager *) chip->consumer->cm;

	ret = power_supply_get_property(chg_psy,
		POWER_SUPPLY_PROP_CHARGER_TYPE, &propval);
	if (ret < 0) {
		dev_err(chip->dev, "%s: get psy type failed, ret = %d\n",
			__func__, ret);
		return ret;
	}

	if (propval.intval == type) {
		dev_info(chip->dev, "%s: psy type is same as %d, not need update\n",
			__func__, type);
		return ret;
	}

	p9415_set_wpc_mivr(chip, type);

	chip->charger = type;
	propval.intval = type;
	ret = power_supply_set_property(chg_psy,
		POWER_SUPPLY_PROP_CHARGER_TYPE, &propval);
	if (ret < 0)
		dev_err(chip->dev, "%s: set psy type failed, ret = %d\n",
			__func__, ret);

	power_supply_changed(chip->wpc_psy);

	if (atomic_read(&chip->online) &&
		cm->chr_type != CHARGER_UNKNOWN)
		cm->chr_type = type;

	_wake_up_charger(cm);

	dev_info(chip->dev, "%s: type = %d\n", __func__, type);
	return ret;
}

static int p9415_get_online(struct charger_device *chg_dev, bool *stat)
{
	struct p9415_dev *chip = dev_get_drvdata(&chg_dev->dev);

	*stat = atomic_read(&chip->online);
	dev_dbg(chip->dev, "%s: get online status: %d\n", __func__, *stat);

	return 0;
}

static int p9415_get_temp(struct charger_device *chg_dev)
{
	struct p9415_dev *chip = dev_get_drvdata(&chg_dev->dev);

	return p9415_get_temp_adc(chip);
}

static int p9415_force_enable_charge(struct charger_device *chg_dev, bool en)
{
	struct p9415_dev *chip = dev_get_drvdata(&chg_dev->dev);

	dev_info(chip->dev, "%s\n", __func__);
	mutex_lock(&chip->irq_lock);
	p9415_enable_charge_flow(chip, en);
	mutex_unlock(&chip->irq_lock);

	return 0;
}

static void p9415_switch_set_state(struct p9415_dev *chip,
	enum dock_state_type set_state)
{
	int cur_state;

	mutex_lock(&chip->switch_lock);
	cur_state = switch_get_state(&chip->dock_state);
	if (cur_state != set_state) {
		dev_info(chip->dev, "%s: dock state changed: %d -> %d\n",
			__func__, cur_state, set_state);
		switch_set_state(&chip->dock_state, set_state);
	} else
		dev_info(chip->dev, "%s: state is same as %d, not need update\n",
			__func__, set_state);

	mutex_unlock(&chip->switch_lock);
}

static void dump_sw_fod_record(struct p9415_dev *chip)
{
	if (chip->sw_fod_count > 0) {
		uint16_t idx, max_count;
		struct rtc_time tm;

		dev_info(chip->dev, "%s: sw_fod_count:%d\n", __func__,
			chip->sw_fod_count);
		max_count = min(chip->sw_fod_count, SW_FOD_RECORD_SIZE);
		for (idx = 0; idx < max_count; idx++) {
			rtc_time_to_tm(chip->sw_fod_time_record[idx].tv_sec,
				&tm);
			dev_info(chip->dev, "%s: SW FOD record%d: %d-%02d-%02d %02d:%02d:%02d.%09lu UTC\n",
				__func__, idx,
				tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
				tm.tm_hour, tm.tm_min, tm.tm_sec,
				chip->sw_fod_time_record[idx].tv_nsec);
		}
	}
}

static void p9415_clean_connection_state(struct p9415_dev *chip)
{
	struct charger_manager *cm =
		(struct charger_manager *) chip->consumer->cm;
	struct charger_data *pdata;

	cancel_delayed_work(&chip->EPT_work);
	cancel_delayed_work_sync(&chip->bpp_switch_work);
	cancel_delayed_work_sync(&chip->fast_charging_work);
	cancel_delayed_work(&chip->step_charging_work);
	cancel_delayed_work(&chip->adaptive_current_limit_work);

	chip->tx_id_authen_status = false;
	chip->tx_dev_authen_status = false;
	chip->tx_authen_complete = false;
	chip->tx_max_vbridge = 0;
	chip->tx_max_power = 0;
	chip->rx_mode = 0;
	chip->cm_cap_en = false;
	chip->on_step_charging = false;
	atomic_set(&chip->vswitch_done, 0);

	/* Reset to a lower level. */
	cm->data.wpc_15w_charger_input_current =
		chip->step_load.start_ma * 1000;
	pdata = &cm->chg1_data;
	pdata->wireless_input_current_limit = -1;

	chip->adaptive_current_limit.on_adaptive = false;
	chip->prev_input_current = -1;

	dump_sw_fod_record(chip);
}

static void p9415_set_cm_cap_enable(struct p9415_dev *chip, bool en)
{

	int ret = 0;

	dev_info(chip->dev, "%s: %s\n", __func__, en ? "true" : "false");

	if (chip->cm_cap_en == en) {
		dev_info(chip->dev, "%s: is_enabled status same as %d\n",
				__func__, en);
		return;
	}

	chip->cm_cap_en = en;
	if (en)
		ret = chip->bus.write(chip, REG_CM_CAP_EN_ADDR, EN_CM_CAP);
	else
		ret = chip->bus.write(chip, REG_CM_CAP_EN_ADDR, DIS_CM_CAP);
	if (ret)
		dev_err(chip->dev, "%s: Failed to enable cm cap: %d\n",
				__func__, ret);
}

static int p9415_enable_charge_flow(struct p9415_dev *chip, bool en)
{
	uint8_t rx_mode = 0;
	enum charger_type charger = CHARGER_UNKNOWN;

	dev_info(chip->dev, "%s: %s\n", __func__, en ? "true" : "false");

	if (chip->is_enabled == en) {
		dev_info(chip->dev, "%s: is_enabled status same as %d\n",
				__func__, en);
		return 0;
	}
	chip->is_enabled = en;
	chip->vout_en = en;
	atomic_set(&chip->online, en);

	if (en) {
		p9415_switch_set_state(chip, TYPE_DOCKED);
		p9415_set_cm_cap_enable(chip, true);
		p9415_update_charge_online(chip, true);
		p9415_read(chip, REG_SYS_Mode, &rx_mode);
		if (rx_mode == RX_MODE_BPP)
			dev_info(chip->dev, "%s: BPP mode\n", __func__);
		else if (rx_mode == RX_MODE_EPP)
			dev_info(chip->dev, "%s: EPP mode\n", __func__);
		else
			dev_info(chip->dev, "%s: rx_mode=0x%02x\n",
				__func__, rx_mode);
		dev_info(chip->dev, "%s: id_auth=%d, dev_auth=%d\n",
			__func__,
			chip->tx_id_authen_status, chip->tx_dev_authen_status);
		if (rx_mode == RX_MODE_EPP) {
			charger = WIRELESS_CHARGER_10W;
		} else {
			charger = WIRELESS_CHARGER_DEFAULT;

			schedule_delayed_work(&chip->bpp_switch_work,
				msecs_to_jiffies(10000));
		}
		chip->rx_mode = rx_mode;

		if (charger == WIRELESS_CHARGER_15W)
			p9415_set_vout(chip, chip->vout_15w);

		p9415_update_charge_type(chip, charger);
		p9415_write_fod(chip, charger);
	} else {
		p9415_clean_connection_state(chip);
		p9415_switch_set_state(chip, TYPE_UNDOCKED);
		p9415_update_charge_online(chip, false);
		p9415_update_charge_type(chip, CHARGER_UNKNOWN);
	}

	return 0;
}

static int p9415_do_algorithm(struct charger_device *chg_dev, void *data)
{
	struct p9415_dev *chip = dev_get_drvdata(&chg_dev->dev);
	struct charger_manager *info = (struct charger_manager *)data;
	struct charger_data *pdata;
	int battery_temperature = info->battery_temperature;
	bool wpc_online;
	int throttle_threshold = chip->throttle_threshold;
	int throttle_hysteresis_threshold = chip->throttle_hysteresis_threshold;

	mutex_lock(&chip->irq_lock);
	wpc_online = atomic_read(&chip->online);
	if (!wpc_online)
		goto out;

	if ((throttle_threshold > 0) &&
		(chip->charger == WIRELESS_CHARGER_15W)) {
		pdata = &info->chg1_data;
		if (battery_temperature >= throttle_threshold) {
			pdata->wireless_input_current_limit = 1000000;
		} else if (pdata->wireless_input_current_limit > 0) {
			if (battery_temperature <= throttle_hysteresis_threshold)
				pdata->wireless_input_current_limit = -1;
			else
				dev_info(chip->dev, "%s: wait battery temperature decreasing\n",
					__func__);
		}
	}

	if (chip->charger == WIRELESS_CHARGER_15W &&
		chip->on_step_charging == true) {
		struct timespec now_time;
		unsigned long total_time_on_step_charging;
		struct charger_manager *cm =
			(struct charger_manager *) chip->consumer->cm;

		getrawmonotonic(&now_time);
		total_time_on_step_charging =
			now_time.tv_sec - chip->start_step_chg_time.tv_sec;
		dev_info(chip->dev, "%s: start:%d  now:%d\n", __func__,
			chip->start_step_chg_time.tv_sec, now_time.tv_sec);
		if (total_time_on_step_charging >= TIMEOUT_ON_STEP_CHARGING) {
			dev_info(chip->dev, "%s: Step charging timeout. Reschedule switching.\n",
				__func__);

			getrawmonotonic(&chip->start_step_chg_time);
			schedule_delayed_work(&chip->step_charging_work,
				msecs_to_jiffies(chip->step_load.step_interval)
				);
		} else {
			dev_info(chip->dev, "%s: On step charging - level:%dmA\n",
				__func__,
				cm->data.wpc_15w_charger_input_current/1000);
		}
	}

out:
	mutex_unlock(&chip->irq_lock);
	return 0;
}

static struct charger_ops p9415_chg_ops = {
	.get_temp = p9415_get_temp,
	.get_wpc_online = p9415_get_online,
	.do_wpc_algorithm = p9415_do_algorithm,
	.force_enable_wpc_charge = p9415_force_enable_charge,
	.set_vout_en = p9415_set_vout_en,
	.set_wpc_en = p9415_set_wpc_en,
};

static struct p9415_desc p9415_default_desc = {
	.chg_dev_name = "wireless_chg",
	.alias_name = "p9415",
};


static int p9415_chager_device_register(struct p9415_dev *chip)
{
	int ret = 0;

	chip->desc = &p9415_default_desc;
	chip->chg_props.alias_name = chip->desc->alias_name;
	chip->chg_dev = charger_device_register(chip->desc->chg_dev_name,
		chip->dev, chip, &p9415_chg_ops, &chip->chg_props);

	if (IS_ERR_OR_NULL(chip->chg_dev)) {
		ret = PTR_ERR(chip->chg_dev);
		return ret;
	}

	return 0;
}

static bool p9415_get_attach_status(struct p9415_dev *chip)
{
	int ret = 0;
	uint32_t intflag = 0;
	uint32_t status = 0;
	uint32_t flag = 0;

	/* powergood not high, tx is not attached */
	if (!p9415_get_pg_irq_status(chip))
		goto dettached;

	dev_info(chip->dev, "%s: power good online\n", __func__);

	/* check triggered interrupt if tx is just attached */
	ret = p9415_read4(chip, REG_INTFLAG, &intflag);
	if (ret) {
		dev_err(chip->dev, "%s: Failed to read INTFLAG\n", __func__);
		goto dettached;
	}
	dev_info(chip->dev, "%s: boot INTFLAG: 0x%04x\n", __func__, intflag);

	/* check status register */
	ret = p9415_read4(chip, REG_STATUS, &status);
	if (ret) {
		dev_err(chip->dev, "%s: Failed to read STATUS\n", __func__);
		goto dettached;
	}
	dev_info(chip->dev, "%s: boot STATUS: 0x%04x\n", __func__, status);

	flag = intflag | status;
	p9415_convert_intflag(chip, flag);

	/* clear interrupt */
	if (intflag) {
		ret = p9415_write4(chip, REG_INT_CLEAR, intflag);
		if (ret) {
			dev_err(chip->dev, "%s: Failed to clear IRQ\n",
				__func__);
			goto dettached;
		}

		ret = chip->bus.write(chip, REG_COMMAND, CLRINT);
		if (ret) {
			dev_err(chip->dev, "%s: Failed to reset INT\n",
				__func__);
			goto dettached;
		}
	}

	/* handler initial state */
	if (flag) {
		if (flag & P9415_INT_ID_AUTH_FAIL)
			chip->tx_authen_complete = true;

		if (flag & P9415_INT_ID_AUTH_SUCCESS)
			chip->tx_id_authen_status = true;
		if (flag & P9415_INT_DEVICE_AUTH_SUCCESS) {
			chip->tx_dev_authen_status = true;
			chip->tx_authen_complete = true;
			/* only KPOC could be here. */
			if (flag & P9415_INT_VSWITCHSUCCESS)
				atomic_set(&chip->vswitch_done, 1);
		}
	}

	/* set true if RX attached TX works well */
	atomic_set(&chip->online, true);
	return true;

dettached:
	atomic_set(&chip->online, false);
	return false;

}

static void p9415_determine_init_status(struct work_struct *work)
{
	struct p9415_dev *chip = container_of(work, struct p9415_dev,
						wpc_init_work.work);

	if (p9415_get_pg_irq_status(chip))
		atomic_set(&chip->online, true);
	else
		atomic_set(&chip->online, false);

	if (!is_mtk_charger_init_done())
		goto out;

	if (p9415_get_attach_status(chip))
		p9415_attached_vbus(chip);

	enable_irq(chip->int_num);
	enable_irq(chip->pg_num);

	if (p9415_get_pg_irq_status(chip) && chip->tx_id_authen_status) {
		if (chip->tx_dev_authen_status) {
			if (atomic_read(&chip->vswitch_done) == 1) {
				/* only KPOC mode could be here */
				p9415_set_vout(chip, chip->vout_15w);
				p9415_update_charge_type(chip,
					WIRELESS_CHARGER_15W);
				p9415_write_fod(chip, WIRELESS_CHARGER_15W);
				p9415_init_adaptive_current_limit_work(chip, 0);
			} else {
				if (chip->en_chk_tx_cap)
					p9415_request_tx_capability(chip);
				else
					p9415_enable_fast_charging(chip);
			}
		} else {
			p9415_device_auth_req(chip);
		}
	}

	dev_info(chip->dev, "%s: wpc init successfully.\n", __func__);
	return;

out:
	dev_info(chip->dev, "%s: mtk_charger not init done.\n", __func__);
	schedule_delayed_work(&chip->wpc_init_work, READY_DETECT_TIME);
}

/* Enable Fast Charging Voltage */
static void p9415_enable_fc_voltage(struct p9415_dev *chip, uint16_t mv)
{
	chip->bus.write_buf(chip, REG_FC_VOLTAGE, (uint8_t *) &mv, 2);
	chip->bus.write(chip, REG_COMMAND, VSWITCH);
	dev_info(chip->dev, "%s: switch to %d mV\n", __func__, mv);
}

/* @initial_delay : Unit in ms */
static void p9415_init_adaptive_current_limit_work(struct p9415_dev *chip,
	int initial_delay)
{
	struct adaptive_current_limit *adaptive =
		&chip->adaptive_current_limit;

	adaptive->current_index = 0;

	adaptive->fill_count = 0;
	memset(adaptive->ibus, 0, sizeof(int)*IBUS_BUFFER_SIZE);

	schedule_delayed_work(&chip->adaptive_current_limit_work,
		msecs_to_jiffies(initial_delay));
}

static void p9415_do_fast_charging_work(struct work_struct *work)
{
	struct p9415_dev *chip = container_of(work, struct p9415_dev,
						fast_charging_work.work);
	struct charger_manager *cm =
		(struct charger_manager *) chip->consumer->cm;
	int ui_soc;

	if (atomic_read(&chip->vswitch_done) == 1) {
		if (chip->charger != WIRELESS_CHARGER_15W) {
			cm->data.wpc_15w_charger_input_current = 500 * 1000;

			ui_soc = battery_get_bat_uisoc();
			if (ui_soc < chip->adaptive_current_limit.start_soc) {
				chip->on_step_charging = true;
				getrawmonotonic(&chip->start_step_chg_time);
				schedule_delayed_work(&chip->step_charging_work,
					msecs_to_jiffies(
					chip->step_load.step_interval));
			}
			p9415_update_charge_type(chip, WIRELESS_CHARGER_15W);
			p9415_write_fod(chip, WIRELESS_CHARGER_15W);
			p9415_init_adaptive_current_limit_work(chip, 0);
		}
	} else {
		p9415_enable_fc_voltage(chip, chip->vout_15w);

		if (atomic_read(&chip->vswitch_retry) < VSWITCH_RETRY) {
			atomic_inc(&chip->vswitch_retry);
			schedule_delayed_work(&chip->fast_charging_work,
				msecs_to_jiffies(1000));
		} else if (cancel_delayed_work(&chip->bpp_switch_work)) {
			schedule_delayed_work(&chip->bpp_switch_work, 0);
		}
	}
}

static void p9415_update_current_setting(struct p9415_dev *chip)
{
	struct charger_manager *cm =
		(struct charger_manager *) chip->consumer->cm;
	int input_current;

	if (!chip->bat_psy)
		chip->bat_psy = power_supply_get_by_name("battery");

	if (chip->charger == WIRELESS_CHARGER_DEFAULT)
		input_current = 500 * 1000;
	else if (chip->charger == WIRELESS_CHARGER_5W)
		input_current = cm->data.wpc_5w_charger_input_current;
	else if (chip->charger == WIRELESS_CHARGER_15W)
		input_current = cm->data.wpc_15w_charger_input_current;
	else if (chip->charger == WIRELESS_CHARGER_10W)
		input_current = cm->data.wpc_10w_charger_input_current;
	else
		return;

	/* Skip update if the same as previous. */
	if (input_current == chip->prev_input_current)
		return;

	if (cm->change_current_setting)
		cm->change_current_setting(cm);

	if (chip->bat_psy)
		power_supply_changed(chip->bat_psy);
	chip->prev_input_current = input_current;
}

static void p9415_bpp_switch_work(struct work_struct *work)
{
	struct p9415_dev *chip = container_of(work, struct p9415_dev,
						bpp_switch_work.work);

	if ((chip->rx_mode == RX_MODE_BPP) &&
			(chip->charger == WIRELESS_CHARGER_DEFAULT)) {
		p9415_update_charge_type(chip, WIRELESS_CHARGER_5W);
		p9415_update_current_setting(chip);
	}
}

static void p9415_EPT_work(struct work_struct *work)
{
	struct p9415_dev *chip = container_of(work, struct p9415_dev,
						EPT_work.work);

	if (p9415_get_pg_irq_status(chip)) {
		dev_info(chip->dev, "%s: Fake FOD send EPT\n", __func__);
		p9415_send_eop(chip, EOP_OVER_TEMP);
	} else {
		dev_info(chip->dev, "%s: Fake FOD skip send EPT\n", __func__);
	}
}

static void p9415_step_charging_work(struct work_struct *work)
{
	struct p9415_dev *chip = container_of(work, struct p9415_dev,
						step_charging_work.work);
	struct charger_manager *cm =
		(struct charger_manager *) chip->consumer->cm;
	struct step_load *step_load = &chip->step_load;
	int ua_now = 0;
	int ua_target = 0;
	int ua_max = 0;

	if (!p9415_get_pg_irq_status(chip))
		return;

	ua_now = cm->data.wpc_15w_charger_input_current;
	ua_max = step_load->step_max_ua;
	if (ua_now < ua_max) {
		ua_target = ua_now + step_load->step_ma * 1000;
		if (ua_target > ua_max)
			ua_target = ua_max;
	} else {
		chip->on_step_charging = false;
		dev_info(chip->dev, "%s: Complete step charging.\n", __func__);
		return;
	}

	dev_info(chip->dev, "%s: target current:%dmA\n", __func__,
		ua_target/1000);
	cm->data.wpc_15w_charger_input_current = ua_target;
	p9415_update_current_setting(chip);
	if (p9415_get_pg_irq_status(chip))
		schedule_delayed_work(&chip->step_charging_work,
			msecs_to_jiffies(chip->step_load.step_interval));
	else
		dev_info(chip->dev, "%s: stop step charging.\n", __func__);
}

static void p9415_simulate_fod(struct p9415_dev *chip)
{
	uint16_t idx;
	uint16_t target_record;

	if (!atomic_read(&chip->online)) {
		dev_info(chip->dev, "%s: wireless charger is offline\n",
			__func__);
		return;
	}

	mutex_lock(&chip->fod_lock);
	target_record = chip->sw_fod_count % SW_FOD_RECORD_SIZE;
	getnstimeofday(&chip->sw_fod_time_record[target_record]);
	dev_info(chip->dev, "%s: Trigger fake FOD\n", __func__);
	for (idx = 0; idx < FOD_COEF_PARAM_LENGTH; idx++)
		p9415_write(chip, REG_FOD_COEF_ADDR + idx, 0x10);
	chip->sw_fod_count++;
	mutex_unlock(&chip->fod_lock);

	if (!delayed_work_pending(&chip->EPT_work)) {
		schedule_delayed_work(&chip->EPT_work,
			msecs_to_jiffies(chip->EPT_work_delay));
	}
}

static bool p9415_is_need_adaptive_current_limit(struct p9415_dev *chip)
{
	if (!p9415_get_pg_irq_status(chip) ||
		chip->charger != WIRELESS_CHARGER_15W) {
		dev_info(chip->dev, "%s: Exit adaptive current limiter.\n",
			__func__);
		return false;
	}

	return true;
}

static bool p9415_skip_to_next_work(struct p9415_dev *chip)
{
	struct adaptive_current_limit *adaptive =
		&chip->adaptive_current_limit;
	int ui_soc;

	if (adaptive->on_adaptive)
		return false;

	if (chip->on_step_charging)
		return true;

	ui_soc = battery_get_bat_uisoc();
	if (ui_soc >= adaptive->start_soc)
		adaptive->on_adaptive = true;
	else
		return true;

	return false;
}

#define FILTER_THRESHOLD	200
static void p9415_adaptive_current_limit_work(struct work_struct *work)
{
	struct p9415_dev *chip = container_of(work, struct p9415_dev,
		adaptive_current_limit_work.work);
	struct adaptive_current_limit *adaptive =
		&chip->adaptive_current_limit;
	struct charger_manager *cm =
		(struct charger_manager *) chip->consumer->cm;
	struct charger_device *chg_dev = chip->prim_chg_dev;

	uint32_t index, current_index;
	int ibus = -1, sum, target_current_limit = -1, margin;
	static int avg;

	/* check conditions should not do adaptive current limit. */
	if (!p9415_is_need_adaptive_current_limit(chip))
		return;

	/* check conditions to re-schedule to next work directly. */
	if (p9415_skip_to_next_work(chip))
		goto done;

	if (charger_dev_get_ibus(chg_dev, &ibus) < 0) {
		dev_err(chip->dev, "%s: Fail to read ibus\n", __func__);
		goto done;
	}
	ibus = ibus/1000;
	current_index = adaptive->current_index;
	adaptive->ibus[current_index] = ibus;
	/* calculate the nex index */
	adaptive->current_index = ((current_index + 1) % IBUS_BUFFER_SIZE);

	/* To reset average value */
	if (adaptive->fill_count == 0)
		avg = 0;
	/* To feed initial buffer data. */
	if (adaptive->fill_count < (IBUS_BUFFER_SIZE-1)) {
		adaptive->fill_count++;
		goto done;
	}

	/* Start to filter after average value is ready. */
	if (avg != 0) {
		if (abs((ibus - avg)) > FILTER_THRESHOLD) {
			if (charger_dev_get_ibus(chg_dev, &ibus) >= 0) {
				ibus = ibus/1000;
				/* Replace with new reading. */
				adaptive->ibus[current_index] = ibus;
			} else {
				dev_err(chip->dev, "%s: Fail to read ibus (%d).\n",
					__func__, __LINE__);
			}
		}
	}

	margin = adaptive->margin;
	for (sum = 0, index = 0; index < IBUS_BUFFER_SIZE; index++)
		sum += adaptive->ibus[index];

	avg = sum/IBUS_BUFFER_SIZE;

	target_current_limit = (avg/100)*100 + margin;

	if (target_current_limit > adaptive->max_current_limit)
		target_current_limit = adaptive->max_current_limit;

	if (target_current_limit < adaptive->min_current_limit)
		target_current_limit = adaptive->min_current_limit;

done:
	dev_info(chip->dev, "%s: IBUS[%d] TARGET[%d]\n",
		__func__, ibus, target_current_limit);

	if (p9415_get_pg_irq_status(chip)) {
		if (target_current_limit > 0) {
			cm->data.wpc_15w_charger_input_current =
				target_current_limit * 1000;
			p9415_update_current_setting(chip);
		}
		schedule_delayed_work(&chip->adaptive_current_limit_work,
			msecs_to_jiffies(adaptive->interval));
	} else {
		dev_info(chip->dev, "%s: Exit adaptive current limiter.\n",
			__func__);
		cm->data.wpc_15w_charger_input_current =
			adaptive->start_ma * 1000;
		adaptive->on_adaptive = false;
	}
}

static int p9415_psy_set_property(struct power_supply *psy,
	enum power_supply_property psp, const union power_supply_propval *val)
{
	struct p9415_dev *chip = power_supply_get_drvdata(psy);
	int input_current_limit = val->intval;

	switch (psp) {
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		/*  Only support zero input current limit to send zero FOD. */
		if (p9415_get_pg_irq_status(chip) && input_current_limit == 0)
			p9415_simulate_fod(chip);
		break;
	default:
		return -EINVAL;
	};
	return 0;
}

static int p9415_psy_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val)
{
	struct p9415_dev *chip = power_supply_get_drvdata(psy);

	union power_supply_propval propval;
	struct power_supply *chg_psy = power_supply_get_by_name("charger");
	enum charger_type charger;
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = atomic_read(&chip->online);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		if (chg_psy == NULL) {
			dev_err(chip->dev, "%s: Not find psy for charger\n",
						__func__);
			return -EINVAL;
		}
		ret = power_supply_get_property(chg_psy,
			POWER_SUPPLY_PROP_CHARGER_TYPE, &propval);
		if (ret < 0) {
			dev_err(chip->dev, "%s: get psy online failed.\n",
				__func__);
			return -EINVAL;
		}

		charger = propval.intval;
		if (charger == WIRELESS_CHARGER_10W)
			val->intval = chip->vout_10w;
		else if (charger == WIRELESS_CHARGER_15W)
			val->intval = chip->vout_15w;
		else
			val->intval = 5000;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		if (!atomic_read(&chip->online))
			return -ENODEV;

		val->intval = p9415_get_temp_adc(chip);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

int p9415_property_is_writeable(struct power_supply *psy,
		enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		return 1;
	default:
		return 0;
	}
}

static enum power_supply_property p9415_psy_properties[] = {
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_TEMP,
};

static int p9415_pm_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9415_dev *chip = (struct p9415_dev *)i2c_get_clientdata(client);

	if (device_may_wakeup(chip->dev))
		enable_irq_wake(chip->int_num);
	disable_irq(chip->pg_num);
	disable_irq(chip->int_num);
	dev_info(chip->dev, "%s\n", __func__);

	return 0;
}

static int p9415_pm_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9415_dev *chip = i2c_get_clientdata(client);

	dev_info(chip->dev, "%s\n", __func__);
	enable_irq(chip->pg_num);
	enable_irq(chip->int_num);
	if (device_may_wakeup(chip->dev))
		disable_irq_wake(chip->int_num);

	return 0;
}

static SIMPLE_DEV_PM_OPS(p9415_pm_ops,
			p9415_pm_suspend, p9415_pm_resume);

static int p9415_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct p9415_dev *chip;
	int ret = 0;
	struct adaptive_current_limit *adaptive;

	dev_info(&client->dev, "%s: enter.\n", __func__);
	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	i2c_set_clientdata(client, chip);

	dev_info(&client->dev, "%s: chip.\n", __func__);
	chip->regmap = regmap_init_i2c(client, &p9415_regmap_config);
	if (!chip->regmap) {
		dev_err(&client->dev, "%s: parent regmap is missing\n",
			__func__);
		ret = -EINVAL;
		goto out_kfree;
	}

	dev_info(&client->dev, "%s: regmap.\n", __func__);
	chip->client = client;
	chip->dev = &client->dev;

	chip->bus.read = p9415_read;
	chip->bus.write = p9415_write;
	chip->bus.read_buf = p9415_read_buffer;
	chip->bus.write_buf = p9415_write_buffer;
	chip->is_hv_adapter = false;
	chip->tx_id_authen_status = false;
	chip->tx_dev_authen_status = false;
	chip->tx_authen_complete = false;
	chip->tx_max_vbridge = 0;
	chip->tx_max_power = 0;
	atomic_set(&chip->vswitch_done, 0);
	chip->is_enabled = false;
	chip->over_reason = 0;
	chip->tx_adapter_type = ADAPTER_UNKNOWN;
	chip->reg.addr = 0x0000;
	chip->reg.size = 1;
	chip->dev_auth_retry = 0;
	chip->tx_authen_complete = false;
	chip->vout_10w = 9000;
	chip->vout_15w = 10000;
	chip->vout_en = true;
	chip->sw_fod_count = 0;
	chip->step_load.start_ma = 500;	/* 500mA */
	chip->step_load.step_ma = 250;	/* 250mA */
	chip->step_load.step_max_ua = 1500 * 1000;	/* 1500mA */
	chip->step_load.bpp_plus_max_ua = 1500 * 1000;	/* 1500mA */
	chip->step_load.step_interval = 1500;	/* 1500ms */
	chip->on_step_charging = false;
	memset(&chip->start_step_chg_time, 0, sizeof(struct timespec));

	adaptive = &chip->adaptive_current_limit;
	adaptive->start_soc = 90;
	adaptive->interval = 10000;	/* 10s */
	adaptive->bpp_plus_max_ma = 1500;	/* 1500mA */
	adaptive->max_current_limit = 1500;	/* 1500mA */
	adaptive->min_current_limit = 400;	/* 400mA */
	adaptive->margin = 200;	/* 200mA */
	adaptive->start_ma = 500;	/* 500mA */
	memset(adaptive->ibus, 0, sizeof(int)*IBUS_BUFFER_SIZE);

	device_init_wakeup(chip->dev, true);
	mutex_init(&chip->sys_lock);
	mutex_init(&chip->irq_lock);
	mutex_init(&chip->fod_lock);
	mutex_init(&chip->switch_lock);

	ret = p9415_parse_dt(chip);
	if (ret < 0) {
		dev_err(chip->dev, "%s: Parse dts failed, ret: %d\n",
			__func__, ret);
		goto out_remap;
	}

	chip->wpc_desc.name = "Wireless";
	chip->wpc_desc.type = POWER_SUPPLY_TYPE_WIRELESS;
	chip->wpc_desc.properties = p9415_psy_properties;
	chip->wpc_desc.num_properties = ARRAY_SIZE(p9415_psy_properties);
	chip->wpc_desc.set_property = p9415_psy_set_property;
	chip->wpc_desc.get_property = p9415_psy_get_property;
	chip->wpc_desc.property_is_writeable = p9415_property_is_writeable,
	chip->wpc_cfg.drv_data = chip;
	chip->wpc_psy = power_supply_register(chip->dev, &chip->wpc_desc,
		&chip->wpc_cfg);
	if (IS_ERR(chip->wpc_psy)) {
		dev_err(chip->dev, "%s: Failed to register power supply: %ld\n",
			__func__, PTR_ERR(chip->wpc_psy));
		ret = PTR_ERR(chip->wpc_psy);
		goto out_remap;
	}

	chip->dock_state.name = "dock";
	chip->dock_state.index = 0;
	chip->dock_state.state = TYPE_UNDOCKED;
	ret = switch_dev_register(&chip->dock_state);
	if (ret) {
		dev_err(chip->dev, "%s: switch_dev_register dock_state Fail\n",
			__func__);
		goto out_wpc_psy;
	}

	ret = p9415_request_io_port(chip);
	if (ret < 0) {
		dev_err(chip->dev, "%s: Failed request IO port, ret: %d\n",
			__func__, ret);
		goto out_switch;
	}

	INIT_DELAYED_WORK(&chip->wpc_init_work, p9415_determine_init_status);
	INIT_DELAYED_WORK(&chip->fast_charging_work,
		p9415_do_fast_charging_work);
	INIT_DELAYED_WORK(&chip->bpp_switch_work,
		p9415_bpp_switch_work);
	INIT_DELAYED_WORK(&chip->EPT_work,
		p9415_EPT_work);
	INIT_DELAYED_WORK(&chip->step_charging_work, p9415_step_charging_work);
	INIT_DELAYED_WORK(&chip->adaptive_current_limit_work,
		p9415_adaptive_current_limit_work);

	chip->prim_chg_dev = get_charger_by_name("primary_chg");
	if (chip->prim_chg_dev == NULL)
		goto out_switch;

	chip->consumer = charger_manager_get_by_name(chip->dev,
		"wireless-charger");
	if (chip->consumer == NULL)
		goto out_switch;

	ret = p9415_register_irq(chip);
	if (ret < 0) {
		dev_err(chip->dev, "%s: Failed reqister irq, ret: %d\n",
			__func__, ret);
		goto out_switch;
	}

	ret = p9415_chager_device_register(chip);
	if (ret < 0) {
		dev_err(chip->dev, "%s: chager device reqister failed, ret: %d\n",
			__func__, ret);
		goto out_switch;
	}

	if (p9415_get_pg_irq_status(chip))
		p9415_fw_ver(chip);

	ret = sysfs_create_group(&client->dev.kobj, &p9415_sysfs_group_attrs);
	if (ret != 0) {
		dev_dbg(chip->dev, "%s: ERROR: sysfs_create_group() failed.\n",
			__func__);
		goto out_charger;
	}

	schedule_delayed_work(&chip->wpc_init_work, 0);

	dev_info(chip->dev, "%s: Successfully.\n", __func__);
	return 0;

out_charger:
	if (chip->chg_dev)
		charger_device_unregister(chip->chg_dev);
out_switch:
	switch_dev_unregister(&chip->dock_state);
out_wpc_psy:
	power_supply_unregister(chip->wpc_psy);
out_remap:
	regmap_exit(chip->regmap);
	device_init_wakeup(chip->dev, false);
	mutex_destroy(&chip->irq_lock);
	mutex_destroy(&chip->sys_lock);
	mutex_destroy(&chip->fod_lock);
	mutex_destroy(&chip->switch_lock);
out_kfree:
	devm_kfree(&client->dev, chip);

	return ret;
}

static int p9415_remove(struct i2c_client *client)
{
	struct p9415_dev *chip = (struct p9415_dev *)i2c_get_clientdata(client);

	if (chip->chg_dev)
		charger_device_unregister(chip->chg_dev);
	mutex_destroy(&chip->irq_lock);
	mutex_destroy(&chip->sys_lock);
	mutex_destroy(&chip->fod_lock);
	mutex_destroy(&chip->switch_lock);
	sysfs_remove_group(&client->dev.kobj, &p9415_sysfs_group_attrs);
	device_init_wakeup(chip->dev, false);
	regmap_exit(chip->regmap);
	switch_dev_unregister(&chip->dock_state);
	power_supply_unregister(chip->wpc_psy);

	return 0;
}

static void p9415_shutdown(struct i2c_client *client)
{
	struct p9415_dev *chip = (struct p9415_dev *)i2c_get_clientdata(client);
	unsigned int boot_mode = get_boot_mode();

	if (boot_mode == KERNEL_POWER_OFF_CHARGING_BOOT
			|| boot_mode == LOW_POWER_OFF_CHARGING_BOOT) {
		if (p9415_get_pg_irq_status(chip) &&
				chip->charger == WIRELESS_CHARGER_15W)
			p9415_set_vout(chip, chip->vout_10w);
		return;
	}

	mutex_lock(&chip->irq_lock);
	if (!atomic_read(&chip->online))
		goto out;

	if (chip->charger == WIRELESS_CHARGER_15W)
		p9415_set_vout(chip, chip->vout_10w);

out:
	mutex_unlock(&chip->irq_lock);
}

static struct i2c_driver p9415_driver = {
	.driver = {
		.name = "p9415",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(match_table),
		.pm = &p9415_pm_ops,
	},
	.probe = p9415_probe,
	.remove = p9415_remove,
	.shutdown = p9415_shutdown,
	.id_table = p9415_dev_id,
};

static int __init p9415_driver_init(void)
{
	return i2c_add_driver(&p9415_driver);
}

static void __exit p9415_driver_exit(void)
{
	i2c_del_driver(&p9415_driver);
}

late_initcall(p9415_driver_init);
module_exit(p9415_driver_exit);

MODULE_AUTHOR("roy.luo@idt.com, simon.song.df@renesas.com");
MODULE_DESCRIPTION("P9415 Wireless Power Charger Monitor driver");
MODULE_LICENSE("GPL v2");
