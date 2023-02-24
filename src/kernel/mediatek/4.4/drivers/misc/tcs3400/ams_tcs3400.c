/*
 * Device driver for monitoring ambient light intensity in (lux), RGB, and
 * color temperature (in kelvin) within the AMS-TAOS TCS family of devices.
 *
 * Copyright (c) 2016, AMS-TAOS USA, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 */

#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mutex.h>
#include <linux/unistd.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/pm.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/kthread.h>
#include <linux/freezer.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_device.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/i2c/ams_tcs3400.h>


static u8 const tcs3400_ids[] = {
	0x90, /* TCS34001 & TCS34005*/
	0x93, /* TCS34003 & TCS34007*/
};

static char const *tcs3400_names[] = {
	"tcs34001/tcs34005",
	"tcs34003/tcs34007",
};

static u8 const restorable_regs[] = {
	TCS3400_ALS_TIME,
	TCS3400_ALS_MINTHRESHLO,
	TCS3400_ALS_MINTHRESHHI,
	TCS3400_ALS_MAXTHRESHLO,
	TCS3400_ALS_MAXTHRESHHI,
	TCS3400_PERSISTENCE,
	TCS3400_CONFIG,
	TCS3400_CONTROL,
	TCS3400_REG_AUX,
	TCS3400_ENABLE,
};

static u8 const als_gains[] = { 1, 4, 16, 64 };
#define ALS_NUM_CH    4
#define ALS_CH_SIZE   (sizeof(u8) * 2)

static int tcs3400_i2c_blk_read(struct tcs3400_chip *chip, u8 reg, u8 *val,
		int size)
{
	s32 ret;
	struct i2c_client *client = chip->client;

	reg += I2C_ADDR_OFFSET;
	ret = i2c_smbus_read_i2c_block_data(client, reg, size, val);
	if (ret < 0)
		dev_err(&client->dev,
			"%s: failed at address %x (%d bytes)\n", __func__,
			reg, size);

	return ret;
}

static int tcs3400_i2c_blk_write(struct tcs3400_chip *chip, u8 reg, u8 *val,
		int size)
{
	s32 ret;
	struct i2c_client *client = chip->client;

	ret =  i2c_smbus_write_i2c_block_data(client, reg, size, val);
	if (ret < 0)
		dev_err(&client->dev, "%s: failed 2X at address %x (%d bytes)\n",
				__func__, reg, size);

	return ret;
}

static int tcs3400_i2c_read(struct tcs3400_chip *chip, u8 reg, u8 *val)
{
	int ret;
	s32 read;
	struct i2c_client *client = chip->client;

	reg += I2C_ADDR_OFFSET;
	ret = i2c_smbus_write_byte(client, reg);
	if (ret < 0) {
		dev_err(&client->dev, "%s: failed 2x to write register %x\n",
			__func__, reg);
		return ret;
	}

	read = i2c_smbus_read_byte(client);
	if (read < 0) {
		dev_err(&client->dev, "%s: failed read from register %x\n",
			__func__, reg);
		return ret;
	}

	*val = (u8) read;
	return 0;
}

int tcs3400_i2c_write(struct tcs3400_chip *chip, u8 reg, u8 val)
{
	int ret;
	struct i2c_client *client = chip->client;

	reg += I2C_ADDR_OFFSET;
	ret = i2c_smbus_write_byte_data(client, reg, val);
	if (ret < 0)
		dev_err(&client->dev, "%s: failed to write register %x err= %d\n",
				__func__, reg, ret);

	return ret;
}

static int tcs3400_flush_regs(struct tcs3400_chip *chip)
{
	int i;
	int rc;
	u8 reg;
	u8 en_tmp = 0;

	/* In case an interrupt just happened, clear it as
	 * we're about to change settings anyway.
	 */
	tcs3400_i2c_write(chip, TCS3400_CMD_ALL_INT_CLR, 0x00);
	/* Get the neable reg so we can put it back when we're done. */
	tcs3400_i2c_read(chip, TCS3400_ENABLE, &en_tmp);
	tcs3400_i2c_write(chip, TCS3400_ENABLE, 0x00);


	for (i = 0; i < ARRAY_SIZE(restorable_regs); i++) {
		reg = restorable_regs[i];
		rc = tcs3400_i2c_write(chip, reg, chip->shadow[reg]);
		if (rc) {
			dev_err(&chip->client->dev, "%s: err on reg 0x%02x\n",
					__func__, reg);
			break;
		}
	}
	/* Put it back the way we found it */
	tcs3400_i2c_write(chip, TCS3400_ENABLE, en_tmp);

	return rc;
}

static int tcs3400_update_enable_reg(struct tcs3400_chip *chip)
{
	return tcs3400_i2c_write(chip, TCS3400_ENABLE,
			chip->shadow[TCS3400_ENABLE]);
}

static void tcs3400_calc_cpl(struct tcs3400_chip *chip)
{
	s32 dgf;
	s32 cpl = 0;
	u32 sat = 0;
	u8 atime = 256 - chip->shadow[TCS3400_ALS_TIME];
	u8  gain = als_gains[(chip->shadow[TCS3400_CONTROL] & 0x3)];

	if (chip->params.d_factor < 1)
		chip->params.d_factor = 1;
	dgf = chip->params.d_factor;

	cpl = (atime * gain);
	cpl *= INTEGRATION_CYCLE;
	cpl /= dgf;
	if (cpl < 1)
		cpl = 1;

	sat = min_t(u32, TCS3400_MAX_ALS_VALUE, (u32) atime << 10);
	sat = sat * 8 / 10; /* 80% */

	chip->als_inf.saturation = sat;
	chip->als_inf.cpl = (u32)cpl;
}

static int tcs3400_irq_clr(struct tcs3400_chip *chip, u8 int2clr)
{
	int ret, ret2;

	ret = i2c_smbus_write_byte_data(chip->client, int2clr, 0x00);
	if (ret < 0) {
		ret2 = i2c_smbus_write_byte_data(chip->client, int2clr, 0x00);
		if (ret2 < 0)
			dev_err(&chip->client->dev, "%s: failed 2x, int to clr %02x\n",
					__func__, int2clr);

		return ret2;
	}

	return ret;
}

static int tcs3400_als_enable(struct tcs3400_chip *chip, int on)
{
	int rc = 0;

	if (on) {
		tcs3400_calc_cpl(chip);
		tcs3400_irq_clr(chip, TCS3400_CMD_ALL_INT_CLR);
		chip->shadow[TCS3400_ENABLE] |= (TCS3400_EN_PWR_ON |
			TCS3400_EN_ALS | TCS3400_EN_ALS_IRQ);
		chip->shadow[TCS3400_REG_AUX] |= TCS3400_EN_ALS_SAT_IRQ;
		rc = tcs3400_update_enable_reg(chip);
		if (rc) {
			pr_err("Write to en failed\n");
			return rc;
		}
		chip->als_enabled = 1;
	} else {
		tcs3400_irq_clr(chip, TCS3400_CMD_ALL_INT_CLR);
		chip->shadow[TCS3400_ENABLE] = 0x00;
		rc = tcs3400_update_enable_reg(chip);
		chip->als_enabled = 0;
	}
	return rc;
}

static int tcs3400_update_als_thres(struct tcs3400_chip *chip, bool on_enable)
{
	s32 ret;
	u16 deltaP = chip->params.als_deltap;
	u16 from, to, cur;
	u16 saturation = chip->als_inf.saturation;

	cur = chip->als_inf.clear_raw;

	if (on_enable) {
		/* move deltaP far away from current position to force an irq */
		from = to = cur > (saturation / 2) ? 0 : saturation;
	} else {
		deltaP = cur * deltaP / 100;
		if (!deltaP)
			deltaP = 1;

		if (cur > deltaP)
			from = cur - deltaP;
		else
			from = 0;

		if (cur < (saturation - deltaP))
			to = cur + deltaP;
		else
			to = saturation;
	}
	from = cpu_to_le16(from);
	to = cpu_to_le16(to);
	memcpy(&chip->shadow[TCS3400_ALS_MINTHRESHLO], &from, ALS_CH_SIZE);
	memcpy(&chip->shadow[TCS3400_ALS_MAXTHRESHLO],   &to, ALS_CH_SIZE);

	ret = tcs3400_i2c_blk_write(chip, TCS3400_ALS_MINTHRESHLO,
			&chip->shadow[TCS3400_ALS_MINTHRESHLO],
			(TCS3400_ALS_MAXTHRESHHI - TCS3400_ALS_MINTHRESHLO) + 1);

	return (ret < 0) ? ret : 0;
}

static int tcs3400_max_als_value(struct tcs3400_chip *chip)
{
	int val;

	val = 256 - chip->shadow[TCS3400_ALS_TIME];
	val = ((val) * 1024);
	val = val > MAX_ALS_VALUE ? MAX_ALS_VALUE : val;
	return val;
}

static int tcs3400_set_als_gain(struct tcs3400_chip *chip, int gain)
{
	u8 ctrl_reg;
	bool current_state = chip->als_enabled;

	tcs3400_als_enable(chip, 0);

	switch (gain) {
		case 1:
			ctrl_reg = AGAIN_1;
			break;
		case 4:
			ctrl_reg = AGAIN_4;
			break;
		case 16:
			ctrl_reg = AGAIN_16;
			break;
		case 64:
			ctrl_reg = AGAIN_64;
			break;
		default:
			dev_err(&chip->client->dev, "%s: wrong als gain %d\n",
					__func__, gain);
			return -EINVAL;
	}

	chip->params.als_gain = ctrl_reg;
	chip->shadow[TCS3400_CONTROL] &= ~0x3;
	chip->shadow[TCS3400_CONTROL] |= ctrl_reg;

	tcs3400_flush_regs(chip);
	tcs3400_calc_cpl(chip);
	if (current_state)
		tcs3400_als_enable(chip, 1);

	dev_info(&chip->client->dev, "%s: new als gain %d\n",
			__func__, als_gains[ctrl_reg]);

	return 0;
}


static void tcs3400_inc_gain(struct tcs3400_chip *chip)
{
	int rc;
	u8 gain = chip->shadow[TCS3400_CONTROL] & 0x3;
	s8 idx;

	if ((gain >= (ARRAY_SIZE(als_gains) - 1)))
		return;
	for (idx = 0; idx <= (ARRAY_SIZE(als_gains) - 1); idx++) {
		if ((als_gains[idx] == 0) || (idx <= gain))
			continue;
		else if (idx > gain) {
			gain = idx;
			break;
		}
	}

	rc = tcs3400_set_als_gain(chip, als_gains[gain]);
	if (rc == 0)
		tcs3400_calc_cpl(chip);
}

static void tcs3400_dec_gain(struct tcs3400_chip *chip)
{
	int rc;
	u8 gain = chip->shadow[TCS3400_CONTROL] & 0x3;
	s8 idx;

	if ((gain <= 0))
		return;
	for (idx = (ARRAY_SIZE(als_gains) - 1); idx >= 0; idx--) {
		if ((als_gains[idx] == 0) || (idx >= gain))
			continue;
		else if (idx < gain) {
			gain = idx;
			break;
		}
	}

	rc = tcs3400_set_als_gain(chip, als_gains[gain]);
	if (rc == 0)
		tcs3400_calc_cpl(chip);
}

static void tcs3400_autogain(struct tcs3400_chip *chip)
{
	if (!chip->params.als_auto_gain) {
		if (chip->als_inf.clear_raw <= TCS3400_MIN_ALS_VALUE) {
			dev_dbg(&chip->client->dev, "%s: darkness (%d <= %d)\n",
				__func__, chip->als_inf.clear_raw, TCS3400_MIN_ALS_VALUE);
		}
		if (chip->als_inf.clear_raw >= chip->als_inf.saturation) {
			dev_dbg(&chip->client->dev, "%s: digital saturation (%d >= %d)\n",
				__func__, chip->als_inf.clear_raw, chip->als_inf.saturation);
		}
		if (chip->in_asat) {
			dev_dbg(&chip->client->dev,
				"%s: analog saturation - gain: %d, als_itime (ms): %d\n",
				 __func__, chip->params.als_gain, chip->atime_ms);
		}
	} else {
		/* 5% of max counts for given ATIME */
		u32 autogain_increment_threshold = (tcs3400_max_als_value(chip) / 20);

		if ((chip->als_inf.clear_raw >= chip->als_inf.saturation) ||
			chip->in_asat) {
			tcs3400_dec_gain(chip);
		} else if (chip->als_inf.clear_raw < autogain_increment_threshold) {
			tcs3400_inc_gain(chip);
		}
	}
}

static int tcs3400_get_lux(struct tcs3400_chip *chip)
{
	s32 rp1 = 0, gp1 = 0, bp1 = 0, cp1 = 0;
	s32 rp2 = 0, gp2 = 0, bp2 = 0;
	u32 cpl = 0;
	s32 r_coef = chip->params.r_coef;
	s32 g_coef = chip->params.g_coef;
	s32 b_coef = chip->params.b_coef;
	s32 ir;
	s32 cct = 0;
	u32 sat = 0;
	s32 _lux1;
	u32 temp_lux;

	tcs3400_calc_cpl(chip);

	/* read latest ADC values */
	rp1 = (chip->als_inf.red_raw);
	gp1 = (chip->als_inf.green_raw);
	bp1 = (chip->als_inf.blue_raw);
	cp1 = (chip->als_inf.clear_raw );
	ir  = (chip->als_inf.ir);
	cpl = (chip->als_inf.cpl);
	cpl = cpl ? cpl : 1;

	sat = chip->als_inf.saturation;
	if (chip->shadow[TCS3400_CONTROL] == 0x03)
		ir = 0; /*ir no longer a factor */

	/* remove ir from counts*/
	rp1 -= ir;
	gp1 -= ir;
	bp1 -= ir;
	cp1 -= ir;

	/*  x Coef */
	rp2 = (r_coef * rp1);
	gp2 = (g_coef * gp1);
	bp2 = (b_coef * bp1);

	_lux1 = (rp2 + gp2 + bp2);
	temp_lux = _lux1 / cpl;

	/* In the unlikely event IR really was higher
	 * than red content, set to 1 to avoid div err
	 */
	if (rp1 == 0)
		rp1 = 1;

	cct = ((chip->params.ct_coef * (u32)bp1) / (u32)rp1)
			+ chip->params.ct_offset;

    tcs3400_autogain(chip);

	// return old value if negative
	if (temp_lux < 0)
		return chip->als_inf.lux;
	chip->als_inf.lux = (u16) temp_lux;
	chip->als_inf.cct = (u16) cct;

	return 0;
}

static int tcs3400_pltf_power_on(struct tcs3400_chip *chip)
{
	int rc = 0;

	if (chip->pdata->platform_power)
		rc = chip->pdata->platform_power(&chip->client->dev, POWER_ON);
	chip->unpowered = rc != 0;
	return rc;
}

static int tcs3400_pltf_power_off(struct tcs3400_chip *chip)
{
	int rc = 0;

	if (chip->pdata->platform_power) {
		rc = chip->pdata->platform_power(&chip->client->dev, POWER_OFF);
		chip->unpowered = rc == 0;
	} else {
		chip->unpowered = false;
	}
	return rc;
}

static void tcs3400_get_als(struct tcs3400_chip *chip)
{
	u8 *buf = &chip->shadow[TCS3400_CLR_CHANLO];

	/* extract raw channel data */
	/* IR collection mode */

	if (chip->ir_toggle_mode == 0)
		chip->als_inf.clear_raw =
			le16_to_cpup((const __le16 *) &buf[0]);
	else if (chip->ir_toggle_mode == 1)
		chip->als_inf.ir_passband =
			le16_to_cpup((const __le16 *) &buf[0]);
	else if (chip->ir_toggle_mode == 2) {
			/*Toggle between clear and IR*/
		if (chip->ir_toggle_state == 0x00) {
			chip->als_inf.clear_raw =
			le16_to_cpup((const __le16 *) &buf[0]);
			chip->ir_toggle_state = 0x80;
		} else {
			chip->als_inf.ir_passband =
				le16_to_cpup((const __le16 *) &buf[0]);
			chip->ir_toggle_state = 0x00;
		}
		tcs3400_i2c_write(chip, TCS3400_IR_TOGGLE,
				chip->ir_toggle_state);
	}

	chip->als_inf.clear_raw = le16_to_cpup((const __le16 *) &buf[0]);
	chip->als_inf.red_raw   = le16_to_cpup((const __le16 *) &buf[2]);
	chip->als_inf.green_raw = le16_to_cpup((const __le16 *) &buf[4]);
	chip->als_inf.blue_raw  = le16_to_cpup((const __le16 *) &buf[6]);

	/* Computed IR always available */
	chip->als_inf.ir = (chip->als_inf.red_raw + chip->als_inf.green_raw
		+ chip->als_inf.blue_raw - chip->als_inf.clear_raw + 1) / 2;

	/* Since '.ir' is u16, it would be zero anyway
	 * - but follow good practice.
	 */
	if (chip->als_inf.ir < 1)
		chip->als_inf.ir = 0;
}

static int tcs3400_read_all(struct tcs3400_chip *chip)
{
	int ret = 0;

	ret = tcs3400_i2c_blk_read(chip, TCS3400_CLR_CHANLO,
			&chip->shadow[TCS3400_CLR_CHANLO],
			ALS_CH_SIZE * ALS_NUM_CH);

	return (ret < 0) ? ret : 0;
}

static void tcs3400_report_als(struct tcs3400_chip *chip)
{
	int lux, rc;

	if (chip->a_idev) {
		rc = tcs3400_get_lux(chip);
		if (!rc) {
			lux = chip->als_inf.lux;
			input_report_abs(chip->a_idev, ABS_MISC, lux);
			input_sync(chip->a_idev);
		}

		/* if persistence is non-zero then update the thresholds */
		if (chip->shadow[TCS3400_PERSISTENCE] & 0x0F)
			tcs3400_update_als_thres(chip, !!rc);
	}
}

static int tcs3400_irq_handler(struct tcs3400_chip *chip)
{
	u8 status;

	tcs3400_i2c_read(chip, TCS3400_STATUS,
		&chip->shadow[TCS3400_STATUS]);
	status = chip->shadow[TCS3400_STATUS];

	mutex_lock(&chip->lock);

	if (status == 0) {
		mutex_unlock(&chip->lock);
		return 0; /*not our interrupt*/
	}

	do {
		/*if we hre our irq was triggered so clear this device */
		tcs3400_i2c_write(chip, TCS3400_CMD_ALL_INT_CLR, 0x00);
		if (status & TCS3400_ST_ALS_SAT)
			chip->in_asat = true;
		else
			chip->in_asat = false;

		/* ALS */
		if (status & TCS3400_ST_ALS_IRQ) {
			if (status & TCS3400_ST_ALS_VALID) {
				tcs3400_read_all(chip);
				tcs3400_get_als(chip);
				tcs3400_report_als(chip);
			} else if (chip->in_asat) {
				tcs3400_autogain(chip);
			}
		}

		tcs3400_i2c_read(chip, TCS3400_STATUS,
				&chip->shadow[TCS3400_STATUS]);
		status = chip->shadow[TCS3400_STATUS];
	} while (status & TCS3400_ST_ALS_IRQ);

	mutex_unlock(&chip->lock);
	return 1; /*we scheduled the interrupt*/
}

static irqreturn_t tcs3400_irq(int irq, void *handle)
{
	struct tcs3400_chip *chip = handle;
	struct device *dev = &chip->client->dev;

	if (chip->in_suspend) {
		dev_info(dev, "%s: in suspend\n", __func__);
		chip->irq_pending = 1;
		/* device signaled this */
		tcs3400_i2c_write(chip, TCS3400_CMD_ALL_INT_CLR, 0x00);
		goto bypass;
	}
	/* Data updates in a seperate thread*/
	//schedule_work(&chip->work_thread);
	tcs3400_irq_handler(chip);
bypass:
	return IRQ_HANDLED;
}

static void tcs3400_set_defaults(struct tcs3400_chip *chip)
{
	u8 *sh = chip->shadow;
	struct device *dev = &chip->client->dev;

	if (chip->pdata) {
		dev_info(dev, "%s: Loading pltform data\n", __func__);
		chip->params.als_time = chip->pdata->parameters.als_time;
		chip->atime_ms =
			(256 - chip->params.als_time) * INTEGRATION_CYCLE / 1000;
		chip->params.persist = chip->pdata->parameters.persist;
		chip->params.als_gain = chip->pdata->parameters.als_gain;
		chip->params.als_auto_gain = chip->pdata->parameters.als_auto_gain;
		chip->params.als_deltap = chip->pdata->parameters.als_deltap;
		chip->params.d_factor = chip->pdata->parameters.d_factor;
		chip->params.r_coef = chip->pdata->parameters.r_coef;
		chip->params.g_coef = chip->pdata->parameters.g_coef;
		chip->params.b_coef = chip->pdata->parameters.b_coef;
		chip->params.ct_coef = chip->pdata->parameters.ct_coef;
		chip->params.ct_offset = chip->pdata->parameters.ct_offset;
	} else {
		dev_info(dev, "%s: use defaults\n", __func__);
		chip->params.persist = ALS_PERSIST(1);
		chip->params.als_time = 0xee; /* 51ms */
		chip->params.als_gain = AGAIN_16;
		chip->atime_ms = 51;
		chip->params.als_auto_gain = 0;
		chip->params.als_deltap = 10;
		chip->params.d_factor = D_Factor;
		chip->params.r_coef = R_Coef;
		chip->params.g_coef = G_Coef;
		chip->params.b_coef = B_Coef;
		chip->params.ct_coef = CT_Coef;
		chip->params.ct_offset = CT_Offset;
	}

	sh[TCS3400_CONTROL] &= ~0x3;
	sh[TCS3400_CONTROL] |= chip->params.als_gain;
	sh[TCS3400_ALS_TIME] = chip->params.als_time;
	sh[TCS3400_PERSISTENCE] = chip->params.persist;
	sh[TCS3400_CONFIG] = 0x40; //reserve bit 6 must be set

	tcs3400_flush_regs(chip);
}

static ssize_t tcs3400_device_als_lux(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tcs3400_chip *chip = dev_get_drvdata(dev);

	mutex_lock(&chip->lock);
	tcs3400_read_all(chip);
	tcs3400_get_als(chip);
	tcs3400_get_lux(chip);
	mutex_unlock(&chip->lock);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_inf.lux);
}

static ssize_t tcs3400_lux_table_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tcs3400_chip *chip = dev_get_drvdata(dev);

	int k = 0;

	mutex_lock(&chip->lock);
	k = snprintf(buf + k, PAGE_SIZE - k, "%d,%d,%d,%d,%d,%d\n",
		chip->params.d_factor, chip->params.r_coef,
		chip->params.g_coef,   chip->params.b_coef,
		chip->params.ct_coef,  chip->params.ct_offset);
	mutex_unlock(&chip->lock);
	return k;
}

static ssize_t tcs3400_lux_table_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct tcs3400_chip *chip = dev_get_drvdata(dev);
	u32 d_factor, r_coef, g_coef, b_coef, ct_coef, ct_offset;

	if (6 != sscanf(buf, "%10d,%10d,%10d,%10d,%10d,%10d",
		&d_factor, &r_coef, &g_coef, &b_coef, &ct_coef, &ct_offset))
		return -EINVAL;
	mutex_lock(&chip->lock);
	chip->params.d_factor = d_factor;
	chip->params.r_coef = r_coef;
	chip->params.g_coef = g_coef;
	chip->params.b_coef = b_coef;
	chip->params.b_coef = ct_coef;
	chip->params.b_coef = ct_offset;
	mutex_unlock(&chip->lock);
	return size;
}

static ssize_t tcs3400_als_asat_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tcs3400_chip *chip = dev_get_drvdata(dev);
	int count;

	mutex_lock(&chip->lock);
	count = snprintf(buf, PAGE_SIZE, "%d\n", chip->in_asat);
	mutex_unlock(&chip->lock);

	return count;
}

static ssize_t tcs3400_als_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tcs3400_chip *chip = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_enabled);
}

static ssize_t tcs3400_als_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct tcs3400_chip *chip = dev_get_drvdata(dev);
	long value;

	if (kstrtoul(buf, 10, &value))
		return -EINVAL;

	mutex_lock(&chip->lock);
	if (value) {
		tcs3400_calc_cpl(chip);
		tcs3400_als_enable(chip, 1);
	}
	else {
		tcs3400_als_enable(chip, 0);
	}

	mutex_unlock(&chip->lock);
	return size;
}

static ssize_t tcs3400_als_gain_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tcs3400_chip *chip = dev_get_drvdata(dev);
	int count;

	mutex_lock(&chip->lock);
	count = snprintf(buf, PAGE_SIZE, "%d (%s)\n",
			als_gains[chip->params.als_gain],
			chip->params.als_auto_gain ? "auto" : "manual");

	mutex_unlock(&chip->lock);

	return count;
}

static ssize_t tcs3400_als_red_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tcs3400_chip *chip = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_inf.red_raw);
}

static ssize_t tcs3400_als_green_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tcs3400_chip *chip = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_inf.green_raw);
}

static ssize_t tcs3400_als_blue_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tcs3400_chip *chip = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_inf.blue_raw);
}

static ssize_t tcs3400_als_clear_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tcs3400_chip *chip = dev_get_drvdata(dev);
	int count;

	mutex_lock(&chip->lock);
	count = snprintf(buf, PAGE_SIZE, "%d\n", chip->als_inf.clear_raw);
	mutex_unlock(&chip->lock);

	return count;
}

static ssize_t tcs3400_ir_passband_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tcs3400_chip *chip = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_inf.ir_passband);
}

static ssize_t tcs3400_als_cct_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tcs3400_chip *chip = dev_get_drvdata(dev);
	int count;

	mutex_lock(&chip->lock);
	tcs3400_read_all(chip);
	tcs3400_get_als(chip);
	tcs3400_get_lux(chip);
	count = snprintf(buf, PAGE_SIZE, "%d\n", chip->als_inf.cct);
	mutex_unlock(&chip->lock);
	return count;
}

static ssize_t tcs3400_als_gain_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct tcs3400_chip *chip = dev_get_drvdata(dev);
	unsigned long gain;
	int rc;

	rc = kstrtoul(buf, 10, &gain);
	if (rc)
		return -EINVAL;

	/* limit gains explicitly */
	if (gain != 0 && gain != 1 && gain != 4 && gain != 16 && gain != 64)
		return -EINVAL;

	mutex_lock(&chip->lock);
	if (gain) {
		chip->params.als_auto_gain = false;
		rc = tcs3400_set_als_gain(chip, gain);
		if (!rc)
			tcs3400_calc_cpl(chip);
	} else {
		chip->params.als_auto_gain = true;
	}
	mutex_unlock(&chip->lock);
	return size;
}

static ssize_t tcs3400_als_persist_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tcs3400_chip *chip = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n",
			(((chip->shadow[TCS3400_PERSISTENCE]) & 0x0f)));
}

static ssize_t tcs3400_als_persist_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct tcs3400_chip *chip = dev_get_drvdata(dev);
	long persist;
	int rc;

	rc = kstrtoul(buf, 10, &persist);
	if (rc)
		return -EINVAL;

	mutex_lock(&chip->lock);
	chip->shadow[TCS3400_PERSISTENCE] &= 0xF0;
	chip->shadow[TCS3400_PERSISTENCE] |= ((u8) persist & 0x0F);
	tcs3400_flush_regs(chip);
	mutex_unlock(&chip->lock);
	return size;
}

static ssize_t tcs3400_als_itime_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tcs3400_chip *chip = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d (ms)\n", chip->atime_ms);
}

static ssize_t tcs3400_als_itime_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct tcs3400_chip *chip = dev_get_drvdata(dev);
	bool current_state = chip->als_enabled;
	long itime = 0;
	int als_count, als_time, rc;
	u8 val;

	rc = kstrtoul(buf, 10, &itime);
	if (rc)
		return -EINVAL;
	if (itime > (0xFF * INTEGRATION_CYCLE / SCALER))
		return -EINVAL;

	mutex_lock(&chip->lock);
	tcs3400_als_enable(chip, 0);

	als_count = AW_TIME_MS(itime); //Assume value in ms
	als_time = (als_count * INTEGRATION_CYCLE) / 1000;
	chip->atime_ms = als_time;
	val = 256 - als_count;
	chip->shadow[TCS3400_ALS_TIME] = val;
	tcs3400_flush_regs(chip);
	tcs3400_calc_cpl(chip);
	if (current_state)
		tcs3400_als_enable(chip, 1);

	mutex_unlock(&chip->lock);
	return size;
}

static ssize_t tcs3400_ir_toggle_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tcs3400_chip *chip = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", chip->ir_toggle_mode);
}

static ssize_t tcs3400_ir_toggle_mode_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct tcs3400_chip *chip = dev_get_drvdata(dev);
	bool current_state = chip->als_enabled;
	long toggle_mode;
	int rc;

	rc = kstrtoul(buf, 10, &toggle_mode);
	if ((toggle_mode < 0) || (toggle_mode > 2))
		return -EINVAL;

	mutex_lock(&chip->lock);
	tcs3400_als_enable(chip, 0);

	chip->ir_toggle_mode = (u8)toggle_mode;

	if (chip->ir_toggle_mode == 0)
		chip->ir_toggle_state = 0x00;
	else if (chip->ir_toggle_mode == 1)
		chip->ir_toggle_state = 0x80;
	else if (chip->ir_toggle_mode == 2) {
		/* Toggle between clear and IR */
		if (chip->ir_toggle_state == 0x00)
			chip->ir_toggle_state = 0x80;
		else
			chip->ir_toggle_state = 0x00;
	}

	tcs3400_i2c_write(chip, TCS3400_IR_TOGGLE, chip->ir_toggle_state);
	if (current_state)
		tcs3400_als_enable(chip, 1);

	mutex_unlock(&chip->lock);
	return size;
}

static struct device_attribute als_attrs[] = {
	__ATTR(als_itime, 0660, tcs3400_als_itime_show,
		tcs3400_als_itime_store),
	__ATTR(als_lux, 0440, tcs3400_device_als_lux, NULL),
	__ATTR(als_red, 0440, tcs3400_als_red_show, NULL),
	__ATTR(als_green, 0440, tcs3400_als_green_show, NULL),
	__ATTR(als_blue, 0440, tcs3400_als_blue_show, NULL),
	__ATTR(als_clear, 0440, tcs3400_als_clear_show, NULL),
	__ATTR(als_cct, 0440, tcs3400_als_cct_show, NULL),
	__ATTR(als_asat, 0440, tcs3400_als_asat_show, NULL),
	__ATTR(als_gain, 0660, tcs3400_als_gain_show,
		tcs3400_als_gain_store),
	__ATTR(als_lux_table, 0660, tcs3400_lux_table_show,
		tcs3400_lux_table_store),
	__ATTR(als_enable, 0660, tcs3400_als_enable_show,
		tcs3400_als_enable_store),
	__ATTR(als_persist, 0660, tcs3400_als_persist_show,
		tcs3400_als_persist_store),
	__ATTR(ir_toggle_mode, 0660, tcs3400_ir_toggle_mode_show,
		tcs3400_ir_toggle_mode_store),
	__ATTR(ir_passband, 0440, tcs3400_ir_passband_show, NULL),
};

static int tcs3400_add_sysfs_interfaces(struct device *dev,
		struct device_attribute *a, int size)
{
	int i;

	for (i = 0; i < size; i++)
		if (device_create_file(dev, a + i))
			goto undo;
	return 0;
undo:
	for (; i >= 0; i--)
		device_remove_file(dev, a + i);
	dev_err(dev, "%s: failed to create sysfs interface\n", __func__);
	return -ENODEV;
}

static void tcs3400_remove_sysfs_interfaces(struct device *dev,
		struct device_attribute *a, int size)
{
	int i;

	for (i = 0; i < size; i++)
		device_remove_file(dev, a + i);
}

static int tcs3400_get_id(struct tcs3400_chip *chip, u8 *id, u8 *rev)
{
	int ret;

	ret = tcs3400_i2c_read(chip, TCS3400_REVID, rev);
	if (ret)
		return ret;
	chip->shadow[TCS3400_REVID] = *rev;
	ret = tcs3400_i2c_read(chip, TCS3400_CHIPID, id);
	if (ret)
		return ret;
	chip->shadow[TCS3400_CHIPID] = *id;
	return 0;
}

static int tcs3400_power_on(struct tcs3400_chip *chip)
{
	int rc;

	rc = tcs3400_pltf_power_on(chip);
	if (rc)
		return rc;
	dev_info(&chip->client->dev, "%s: chip was off, restoring regs\n",
			__func__);
	return tcs3400_flush_regs(chip);
}

static int tcs3400_als_idev_open(struct input_dev *idev)
{
	struct tcs3400_chip *chip = dev_get_drvdata(&idev->dev);
	int rc;

	dev_info(&idev->dev, "%s\n", __func__);
	mutex_lock(&chip->lock);
	if (chip->unpowered) {
		rc = tcs3400_power_on(chip);
		if (rc)
			goto chip_on_err;
	}
	rc = tcs3400_als_enable(chip, 1);
	if (rc)
		tcs3400_pltf_power_off(chip);
chip_on_err:
	mutex_unlock(&chip->lock);
	return rc;
}

static void tcs3400_als_idev_close(struct input_dev *idev)
{
	struct tcs3400_chip *chip = dev_get_drvdata(&idev->dev);

	dev_info(&idev->dev, "%s\n", __func__);
	mutex_lock(&chip->lock);
	tcs3400_als_enable(chip, 0);
	tcs3400_pltf_power_off(chip);
	mutex_unlock(&chip->lock);
}

#ifdef CONFIG_OF
int tcs3400_init_dt(struct tcs3400_i2c_platform_data *pdata)
{
	struct device_node *np = pdata->of_node;
	const char *a_str;
	u32 val;

	if (!pdata->of_node)
		return 0;

	if (!of_property_read_string(np, "als_name", &a_str))
		pdata->als_name = a_str;

	if (!of_property_read_u32(np, "als_persist", &val))
		pdata->parameters.persist = val;

	if (!of_property_read_u32(np, "als_gain", &val))
		pdata->parameters.als_gain = val;

	if (!of_property_read_u32(np, "als_auto_gain", &val))
		pdata->parameters.als_auto_gain = val;

	if (!of_property_read_u32(np, "als_time", &val))
		pdata->parameters.als_time = val;

	if (!of_property_read_u32(np, "als_deltap", &val))
		pdata->parameters.als_deltap = val;

	if (!of_property_read_u32(np, "als_can_wake", &val))
		pdata->als_can_wake = (val == 0) ? false : true;

	if (!of_property_read_u32(np, "dgf", &val))
		pdata->parameters.d_factor = val;

	if (!of_property_read_u32(np, "ct_coef", &val))
		pdata->parameters.ct_coef = val;

	if (!of_property_read_u32(np, "ct_offset", &val))
		pdata->parameters.ct_offset = val;

	if (!of_property_read_u32(np, "r_coef", &val))
		pdata->parameters.r_coef = val;

	if (!of_property_read_u32(np, "g_coef", &val))
		pdata->parameters.g_coef = val;

	if (!of_property_read_u32(np, "b_coef", &val))
		pdata->parameters.b_coef = val;

	return 0;
}
#endif

#ifdef CONFIG_OF
static struct of_device_id tcs3400_i2c_dt_ids[] = {
	{ .compatible = "ams,tcs3400" },
	{}

};
MODULE_DEVICE_TABLE(of, tcs3400_i2c_dt_ids);
#endif

static void tcs3400_update_thread(struct work_struct *update)
{
	struct tcs3400_chip *chip
		= container_of(update, struct tcs3400_chip, work_thread);
	tcs3400_irq_handler(chip);
}

static int tcs3400_probe(struct i2c_client *client,
		const struct i2c_device_id *idp)
{
	int i, ret;
	u8 id, rev;
	struct device *dev = &client->dev;
	static struct tcs3400_chip *chip;
	struct tcs3400_i2c_platform_data *pdata = dev->platform_data;
	bool powered = 0;
	unsigned long default_irq_trigger = 0;

	pr_info("\nTCS3400: probe()\n");
#ifdef CONFIG_OF
	if (!pdata) {
		pdata = devm_kzalloc(dev, sizeof(struct tcs3400_i2c_platform_data),
				GFP_KERNEL);
		if (!pdata)
			return -ENOMEM;

		if (of_match_device(tcs3400_i2c_dt_ids, &client->dev)) {
			pdata->of_node = client->dev.of_node;
			ret = tcs3400_init_dt(pdata);
			if (ret)
				return ret;
		}
	}
#endif

	if (!i2c_check_functionality(client->adapter,
		I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(dev, "%s: i2c smbus byte data unsupported\n", __func__);
		ret = -EOPNOTSUPP;
		goto init_failed;
	}
	if (!pdata) {
		dev_err(dev, "%s: platform data required\n", __func__);
		ret = -EINVAL;
		goto init_failed;
	}

	if (!(pdata->als_name) || client->irq < 0) {
		dev_err(dev, "%s: no reason to run.\n", __func__);
		ret = -EINVAL;
		goto init_failed;
	}

	if (pdata->platform_init) {
		ret = pdata->platform_init();
		if (ret)
			goto init_failed;
	}
	if (pdata->platform_power) {
		ret = pdata->platform_power(dev, POWER_ON);
		if (ret) {
			dev_err(dev, "%s: pltf power on failed\n", __func__);
			goto pon_failed;
		}
		powered = true;
	}
	chip = kzalloc(sizeof(struct tcs3400_chip), GFP_KERNEL);
	if (!chip) {
		ret = -ENOMEM;
		goto malloc_failed;
	}
	chip->client = client;
	chip->pdata = pdata;
	i2c_set_clientdata(client, chip);

	ret = tcs3400_get_id(chip, &id, &rev);
	if (ret)
		goto id_failed;

	dev_info(dev, "%s: device id:%02x device rev:%02x\n",
			__func__, id, rev);

	for (i = 0; i < ARRAY_SIZE(tcs3400_ids); i++) {
		if (id == tcs3400_ids[i])
			break;
	}
	if (i < ARRAY_SIZE(tcs3400_names)) {
		dev_info(dev, "%s: '%s rev. %d' detected\n",
			__func__, tcs3400_names[i], rev);
		chip->device_index = i;
	} else {
		dev_err(dev, "%s: not supported chip id\n", __func__);
		ret = -EOPNOTSUPP;
		goto id_failed;
	}

	mutex_init(&chip->lock);
	tcs3400_set_defaults(chip);
	ret = tcs3400_flush_regs(chip);
	if (ret)
		goto flush_regs_failed;
	if (pdata->platform_power) {
		pdata->platform_power(dev, POWER_OFF);
		powered = false;
		chip->unpowered = true;
	}

	if (!pdata->als_name)
		goto bypass_als_idev;
	chip->a_idev = input_allocate_device();
	if (!chip->a_idev) {
		dev_err(dev, "%s: no memory for input_dev '%s'\n", __func__,
				pdata->als_name);
		ret = -ENODEV;
		goto input_a_alloc_failed;
	}
	chip->a_idev->name = pdata->als_name;
	chip->a_idev->id.bustype = BUS_I2C;
	set_bit(EV_ABS, chip->a_idev->evbit);
	set_bit(ABS_MISC, chip->a_idev->absbit);
	input_set_abs_params(chip->a_idev, ABS_MISC, 0, 65535, 0, 0);
	chip->a_idev->open = tcs3400_als_idev_open;
	chip->a_idev->close = tcs3400_als_idev_close;
	dev_set_drvdata(&chip->a_idev->dev, chip);
	ret = input_register_device(chip->a_idev);
	if (ret) {
		input_free_device(chip->a_idev);
		goto input_a_alloc_failed;
	}
	ret = tcs3400_add_sysfs_interfaces(&chip->a_idev->dev, als_attrs,
			ARRAY_SIZE(als_attrs));
	if (ret)
		goto input_a_sysfs_failed;

bypass_als_idev:
	INIT_WORK(&chip->work_thread, tcs3400_update_thread);

	/* Initialize IRQ & Handler */
	default_irq_trigger =
		irqd_get_trigger_type(irq_get_irq_data(client->irq));
	dev_info(dev, "Requesting IRQ %d\n", client->irq);
	ret = request_threaded_irq(client->irq, NULL, &tcs3400_irq,
			default_irq_trigger | IRQF_SHARED |
			IRQF_ONESHOT, dev_name(dev), chip);
	if (ret) {
		dev_info(dev, "Failure to request irq %d\n", client->irq);
		goto irq_register_fail;
	}

	dev_info(dev, "Probe ok.\n");
	return 0;

irq_register_fail:
	if (chip->a_idev) {
		tcs3400_remove_sysfs_interfaces(&chip->a_idev->dev, als_attrs,
				ARRAY_SIZE(als_attrs));
input_a_sysfs_failed:
		input_unregister_device(chip->a_idev);
	}
input_a_alloc_failed:
id_failed:
flush_regs_failed:
	i2c_set_clientdata(client, NULL);
	kfree(chip);
malloc_failed:
	if (powered && pdata->platform_power)
		pdata->platform_power(dev, POWER_OFF);
pon_failed:
	if (pdata->platform_teardown)
		pdata->platform_teardown(dev);
init_failed:
	dev_err(dev, "Probe failed.\n");

	return ret;
}

static int tcs3400_suspend(struct device *dev)
{
	struct tcs3400_chip *chip = dev_get_drvdata(dev);
	struct tcs3400_i2c_platform_data *pdata = dev->platform_data;

	dev_info(dev, "%s\n", __func__);
	mutex_lock(&chip->lock);
	chip->in_suspend = 1;
	if (chip->a_idev && chip->a_idev->users) {
		if (pdata->als_can_wake) {
			dev_info(dev, "set wake on als\n");
			chip->wake_irq = 1;
		} else {
			dev_info(dev, "als off\n");
			tcs3400_als_enable(chip, 0);
		}
	}
	if (chip->wake_irq) {
		irq_set_irq_wake(chip->client->irq, 1);
	} else if (!chip->unpowered) {
		dev_info(dev, "powering off\n");
		tcs3400_pltf_power_off(chip);
	}
	mutex_unlock(&chip->lock);
	return 0;
}

static int tcs3400_resume(struct device *dev)
{
	struct tcs3400_chip *chip = dev_get_drvdata(dev);
	bool als_on;
	int rc = 0;

	mutex_lock(&chip->lock);
	als_on = chip->a_idev && chip->a_idev->users;
	chip->in_suspend = 0;
	dev_info(dev, "%s: powerd %d, als: needed %d  enabled %d", __func__,
			!chip->unpowered, als_on, chip->als_enabled);
	if (chip->wake_irq) {
		irq_set_irq_wake(chip->client->irq, 0);
		chip->wake_irq = 0;
	}
	if (chip->unpowered && als_on) {
		dev_info(dev, "powering on\n");
		rc = tcs3400_power_on(chip);
		if (rc)
			goto err_power;
	}
	if (als_on && !chip->als_enabled)
		(void) tcs3400_als_enable(chip, 1);
	if (chip->irq_pending) {
		dev_info(dev, "%s: pending interrupt\n", __func__);
		chip->irq_pending = 0;
		(void) tcs3400_report_als(chip);
		enable_irq(chip->client->irq);
	}
err_power:
	mutex_unlock(&chip->lock);
	return 0;
}

static int tcs3400_remove(struct i2c_client *client)
{
	struct tcs3400_chip *chip = i2c_get_clientdata(client);

	free_irq(client->irq, chip);
	if (chip->a_idev) {
		tcs3400_remove_sysfs_interfaces(&chip->a_idev->dev, als_attrs,
				ARRAY_SIZE(als_attrs));
		input_unregister_device(chip->a_idev);
	}
	if (chip->pdata->platform_teardown)
		chip->pdata->platform_teardown(&client->dev);
	i2c_set_clientdata(client, NULL);
	kfree(chip);
	return 0;
}

static struct i2c_device_id tcs3400_idtable[] = {
	{ "tcs3400", 0 },
	{}
};

MODULE_DEVICE_TABLE(i2c, tcs3400_idtable);

static const struct dev_pm_ops tcs3400_pm_ops = {
	.suspend = tcs3400_suspend,
	.resume = tcs3400_resume,
};

static struct i2c_driver tcs3400_driver = {
	.driver = {
		.name = "tcs3400",
		.pm = &tcs3400_pm_ops,
		.of_match_table = tcs3400_i2c_dt_ids,
	},
	.id_table = tcs3400_idtable,
	.probe = tcs3400_probe,
	.remove = tcs3400_remove,
};

module_i2c_driver(tcs3400_driver);

MODULE_AUTHOR("AMS AOS Software<cs.americas@ams.com>");
MODULE_DESCRIPTION("AMS-TAOS tcs3400 ambient light sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("2.2.0");
