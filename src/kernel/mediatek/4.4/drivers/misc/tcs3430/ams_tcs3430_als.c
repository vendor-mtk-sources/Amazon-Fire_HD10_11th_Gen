/*
*****************************************************************************
* Copyright by ams AG                                                       *
* All rights are reserved.                                                  *
*                                                                           *
* IMPORTANT - PLEASE READ CAREFULLY BEFORE COPYING, INSTALLING OR USING     *
* THE SOFTWARE.                                                             *
*                                                                           *
* THIS SOFTWARE IS PROVIDED FOR USE ONLY IN CONJUNCTION WITH AMS PRODUCTS.  *
* USE OF THE SOFTWARE IN CONJUNCTION WITH NON-AMS-PRODUCTS IS EXPLICITLY    *
* EXCLUDED.                                                                 *
*                                                                           *
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       *
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT         *
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS         *
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT  *
* OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,     *
* SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT          *
* LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,     *
* DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY     *
* THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT       *
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE     *
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.      *
*****************************************************************************
*/

/*! \file
* \brief Device driver for monitoring ambient light intensity in (lux)
* functionality within the AMS-TAOS TCS3430 family of devices.
*/

#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/slab.h>

#include <linux/i2c/ams_tcs3430.h>
#include "ams_i2c.h"

//#define LUX_DBG

#define GAIN1         0
#define GAIN4         1
#define GAIN16        2
#define GAIN64        3
#define GAIN128       3 //GAIN128 requires HGAIN to be set
#define ALS_NUM_CH    4
#define ALS_CH_SIZE   (sizeof(u8) * 2)

static u16 const als_gains[] = {
	1,    4,
	16,   64,
	128,
};

static u8 const restorable_als_regs[] = {
	TCS3430_REG_ATIME,
	TCS3430_REG_WTIME,
	TCS3430_REG_PERS,
	TCS3430_REG_CFG0,
	TCS3430_REG_CFG1,
	TCS3430_REG_CFG2,
};

static int tcs3430_flush_als_regs(struct tcs3430_chip *chip)
{
	unsigned i;
	int rc;
	u8 reg;

	for (i = 0; i < ARRAY_SIZE(restorable_als_regs); i++) {
		reg = restorable_als_regs[i];
		rc = ams_i2c_write(chip->client, chip->shadow,
				reg, chip->shadow[reg]);
		if (rc) {
			dev_err(&chip->client->dev, "%s: err on reg 0x%02x\n",
					__func__, reg);
			break;
		}
	}

	return rc;
}

static int tcs3430_max_als_value(struct tcs3430_chip *chip)
{
	int val;

	val = chip->shadow[TCS3430_REG_ATIME];
	if (val > 63)
		val = 0xFFFF;
	else
		val = ((val + 1) * 1023);
	return val;
}

static void tcs3430_get_als(struct tcs3430_chip *chip)
{
	u8 *sh = chip->shadow;

	/* extract raw channel data */
	chip->als_inf.z_raw =
		le16_to_cpup((const __le16 *)&sh[TCS3430_REG_CH0DATAL]);
	chip->als_inf.y_raw =
		le16_to_cpup((const __le16 *)&sh[TCS3430_REG_CH1DATAL]);
	chip->als_inf.ir_raw =
		le16_to_cpup((const __le16 *)&sh[TCS3430_REG_CH2DATAL]);
	chip->als_inf.x_raw =
		le16_to_cpup((const __le16 *)&sh[TCS3430_REG_CH3DATAL]);
}

int tcs3430_read_als(struct tcs3430_chip *chip)
{
	int ret;

	ret = ams_i2c_blk_read(chip->client, TCS3430_REG_CH0DATAL,
			&chip->shadow[TCS3430_REG_CH0DATAL],
			ALS_NUM_CH * ALS_CH_SIZE);

	if (ret >= 0) {
		tcs3430_get_als(chip);
		ret = 0;
	}

	return ret;
}

static void tcs3430_calc_cpl(struct tcs3430_chip *chip)
{
	u64 cpl;
	u32 sat;
	u8 atime;
	u8 gain_idx;

	atime = chip->shadow[TCS3430_REG_ATIME];

	cpl = atime;
	cpl *= INTEGRATION_CYCLE;
	gain_idx = chip->shadow[TCS3430_REG_CFG1] & TCS3430_MASK_AGAIN;
	gain_idx >>= TCS3430_SHIFT_AGAIN;
	cpl *= als_gains[gain_idx];
	if ( chip->shadow[TCS3430_REG_CFG2] & TCS3430_MASK_HGAIN )
		cpl *= 2;
	cpl *= DGF_SCALE_DEFAULT;
	//Optimization if coefficients are scaled by 1000,
	//if (chip->params.coef_scale != 1000) {
	//	// Possible overflow if coefficients are scaled > 1000
	//	cpl *= chip->params.coef_scale; //rgb coeff scaling factor
	//	do_div(cpl,(chip->params.dgf * 1000)); //Atime usec -> msec
	//} else {
	//	do_div(cpl,chip->params.dgf); //Atime usec -> msec
	//}

	sat = min_t(u32, TCS3430_MAX_ALS_VALUE, (u32) atime << 10);

	chip->als_inf.cpl = (u32) cpl;
	chip->als_inf.saturation = TENTH_FRACTION_OF_VAL(sat, 8);
}

int tcs3430_configure_als_mode(struct tcs3430_chip *chip, u8 state)
{
	struct i2c_client *client = chip->client;
	u8 *sh = chip->shadow;

	if (state) // Turning on ALS
	{
		chip->shadow[TCS3430_REG_ATIME] = chip->params.als_time;
		tcs3430_calc_cpl(chip);

		/* set PERS.apers to 2 consecutive ALS values out of range */
		chip->shadow[TCS3430_REG_PERS] &= (~TCS3430_MASK_APERS);
		chip->shadow[TCS3430_REG_PERS] |= 0x02;

		tcs3430_flush_als_regs(chip);

		ams_i2c_modify(client, sh, TCS3430_REG_INTENAB,
				TCS3430_AIEN, TCS3430_AIEN);
		ams_i2c_modify(client, sh, TCS3430_REG_ENABLE,
				TCS3430_WEN | TCS3430_AEN | TCS3430_PON,
				TCS3430_WEN | TCS3430_AEN | TCS3430_PON);
		chip->als_enabled = true;
	}
	else  // Turning off ALS
	{
		// Disable ALS, Wait and ALS Interrupt
		ams_i2c_modify(client, sh, TCS3430_REG_INTENAB,
				TCS3430_AIEN, 0);
		ams_i2c_modify(client, sh, TCS3430_REG_ENABLE,
				TCS3430_WEN | TCS3430_AEN, 0);
		chip->als_enabled = false;

		// If nothing else is enabled set PON = 0;
		if(!(sh[TCS3430_REG_ENABLE] & TCS3430_EN_ALL))
			ams_i2c_modify(client, sh, TCS3430_REG_ENABLE,
					TCS3430_PON, 0);
	}

	return 0;
}

static int tcs3430_set_als_gain(struct tcs3430_chip *chip, int gain)
{
	int rc;
	u8 ctrl_reg;
	u8 saved_enable;
	u8 hgain = 0;

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
		case 128:
			ctrl_reg = AGAIN_64;
			hgain = 1;
			break;
		default:
			dev_err(&chip->client->dev, "%s: wrong als gain %d\n",
					__func__, gain);
			return -EINVAL;
	}

	ctrl_reg <<= TCS3430_SHIFT_AGAIN;
	// Turn off ALS, so that new ALS gain value will take effect at start of
	// new integration cycle.
	// New ALS gain value will then be used in next lux calculation.
	ams_i2c_read(chip->client, TCS3430_REG_ENABLE, &saved_enable);
	ams_i2c_write(chip->client, chip->shadow, TCS3430_REG_ENABLE, 0);
	rc = ams_i2c_modify(chip->client, chip->shadow, TCS3430_REG_CFG1,
			TCS3430_MASK_AGAIN, ctrl_reg);

	/* It is required for bit 2 in CFG2 to always be set.
	 * The code below uses TCS3430_MASK_CFG2_REQUIRED for this purpose.
	 */
	rc = ams_i2c_modify(chip->client, chip->shadow, TCS3430_REG_CFG2,
			(TCS3430_MASK_HGAIN | TCS3430_MASK_CFG2_REQUIRED),
			((hgain << TCS3430_SHIFT_HGAIN) | TCS3430_MASK_CFG2_REQUIRED));
	ams_i2c_write(chip->client, chip->shadow,
			TCS3430_REG_ENABLE, saved_enable);

	if (rc >= 0) {
		chip->params.als_gain =
			(chip->shadow[TCS3430_REG_CFG1] & TCS3430_MASK_AGAIN) >> TCS3430_SHIFT_AGAIN;
		chip->params.als_gain += hgain;
		dev_info(&chip->client->dev, "%s: new als gain %d\n",
				__func__, chip->params.als_gain);
	}

	return rc;
}

static void tcs3430_inc_gain(struct tcs3430_chip *chip)
{
	int rc;
	u8 gain_idx = (chip->shadow[TCS3430_REG_CFG1] & TCS3430_MASK_AGAIN) >> TCS3430_SHIFT_AGAIN;
	s8 idx;
	u8 gain = als_gains[gain_idx];
	if ((chip->shadow[TCS3430_REG_CFG2] & TCS3430_MASK_HGAIN) &&
			(gain == 64)) {
		gain = 128;
		gain_idx = als_gains[ARRAY_SIZE(als_gains) - 1];
	}

	if (gain >= als_gains[(ARRAY_SIZE(als_gains) - 1)])
		return;
	for (idx = 0; idx <= (ARRAY_SIZE(als_gains) - 1); idx++) {
		if ((als_gains[idx] == 0) || (idx <= gain_idx)) continue;
		else if (idx > gain_idx) {
			gain_idx = idx;
			break;
		}
	}

	dev_info(&chip->client->dev, "Autogain INC to %d\n", als_gains[gain_idx]);
	rc = tcs3430_set_als_gain(chip, als_gains[gain_idx]);
	if (rc == 0)
		tcs3430_calc_cpl(chip);
}

static void tcs3430_dec_gain(struct tcs3430_chip *chip)
{
	int rc;
	u8 gain_idx = (chip->shadow[TCS3430_REG_CFG1] & TCS3430_MASK_AGAIN) >> TCS3430_SHIFT_AGAIN;
	s8 idx;
	u8 gain = als_gains[gain_idx];
	if ((chip->shadow[TCS3430_REG_CFG2] & TCS3430_MASK_HGAIN) &&
			(gain == 64)) {
		gain = 128;
		gain_idx = als_gains[ARRAY_SIZE(als_gains) - 1];
	}

	if (gain <= als_gains[0])
		return;
	for (idx = (ARRAY_SIZE(als_gains) - 1); idx >= 0; idx--) {
		if ((als_gains[idx] == 0) || (idx >= gain_idx)) continue;
		else if (idx < gain_idx) {
			gain_idx = idx;
			break;
		}
	}

	dev_info(&chip->client->dev, "Autogain DEC to %d\n", als_gains[gain_idx]);
	rc = tcs3430_set_als_gain(chip, als_gains[gain_idx]);
	if (rc == 0)
		tcs3430_calc_cpl(chip);
}

int tcs3430_get_lux(struct tcs3430_chip *chip)
{

	int quintile = (tcs3430_max_als_value(chip) / 5);

	/* Auto gain moved to end of function */

	/* use time in ms get scaling factor */
	//tcs3430_calc_cpl(chip);

	if (!chip->params.als_auto_gain) {
		if (chip->als_inf.z_raw <= TCS3430_MIN_ALS_VALUE) {
			dev_info(&chip->client->dev, "%s: darkness (%d <= %d)\n",
				__func__, chip->als_inf.z_raw, TCS3430_MIN_ALS_VALUE);
		} else if (chip->als_inf.z_raw >= chip->als_inf.saturation) {
			dev_info(&chip->client->dev, "%s: saturation (%d >= %d\n",
				__func__, chip->als_inf.z_raw, chip->als_inf.saturation);
		}
	} else {
		if (chip->als_inf.z_raw < quintile) {
			tcs3430_inc_gain(chip);
			tcs3430_flush_als_regs(chip);
		} else if (chip->als_inf.z_raw >= chip->als_inf.saturation) {
			tcs3430_dec_gain(chip);
			tcs3430_flush_als_regs(chip);
		}
	}
	chip->als_inf.cct = 0;
	chip->als_inf.lux = 0;

	return 0;
}

int tcs3430_update_als_thres(struct tcs3430_chip *chip, bool on_enable)
{
	s32 ret;
	u16 deltaP = chip->params.als_deltaP;
	u32 from, to, cur;
	u32 saturation = chip->als_inf.saturation;

	cur = chip->als_inf.z_raw;

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
	memcpy(&chip->shadow[TCS3430_REG_AILTL], &from, ALS_CH_SIZE);
	memcpy(&chip->shadow[TCS3430_REG_AIHTL],   &to, ALS_CH_SIZE);

	ret = ams_i2c_reg_blk_write(chip->client, TCS3430_REG_AILTL,
			&chip->shadow[TCS3430_REG_AILTL],
			(TCS3430_REG_AIHTH - TCS3430_REG_AILTL) + 1);

	return (ret < 0) ? ret : 0;
}

void tcs3430_report_als(struct tcs3430_chip *chip)
{
	int lux;
	int rc;

	if (chip->a_idev) {
		rc = tcs3430_get_lux(chip);
		if (!rc) {
			lux = chip->als_inf.lux;
			input_report_abs(chip->a_idev, ABS_MISC, lux);
			input_sync(chip->a_idev);
			tcs3430_update_als_thres(chip, 0);
		} else {
			tcs3430_update_als_thres(chip, 1);
		}
	}
}

/*****************/
/* ABI Functions */
/*****************/

static ssize_t tcs3430_device_als_lux(struct device *dev,
                                      struct device_attribute *attr,
                                      char *buf)
{
	struct tcs3430_chip *chip = dev_get_drvdata(dev);

	AMS_MUTEX_LOCK(&chip->lock);

	tcs3430_read_als(chip);
	tcs3430_get_lux(chip);

	AMS_MUTEX_UNLOCK(&chip->lock);

	return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_inf.lux);
}

static ssize_t tcs3430_lux_coef_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct tcs3430_chip *chip = dev_get_drvdata(dev);
	int k = 0;

	AMS_MUTEX_LOCK(&chip->lock);

	AMS_MUTEX_UNLOCK(&chip->lock);

	return k;
}

static ssize_t tcs3430_lux_coef_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	return 0;
}

static ssize_t tcs3430_als_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tcs3430_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_enabled);
}

static ssize_t tcs3430_als_enable_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct tcs3430_chip *chip = dev_get_drvdata(dev);
	bool value;

	if (strtobool(buf, &value))
		return -EINVAL;

	if (value)
		tcs3430_configure_als_mode(chip, 1);
	else
		tcs3430_configure_als_mode(chip, 0);

	return size;
}

static ssize_t tcs3430_auto_gain_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tcs3430_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%s\n",
			chip->params.als_auto_gain ? "auto" : "manual");
}

static ssize_t tcs3430_auto_gain_enable_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct tcs3430_chip *chip = dev_get_drvdata(dev);
	bool value;

	if (strtobool(buf, &value))
		return -EINVAL;

	if (value)
		chip->params.als_auto_gain = true;
	else
		chip->params.als_auto_gain = false;

	return size;
}

static ssize_t tcs3430_als_gain_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tcs3430_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d (%s)\n",
			als_gains[(chip->params.als_gain)],
			chip->params.als_auto_gain ? "auto" : "manual");
}

static ssize_t tcs3430_als_gain_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	unsigned long gain;
	int i = 0;
	int rc;
	struct tcs3430_chip *chip = dev_get_drvdata(dev);

	rc = kstrtoul(buf, 10, &gain);

	if (rc)
		return -EINVAL;
	if (gain != 0   && gain != 1   && gain != 4   &&
			gain != 16  &&  gain != 64 && gain != 128)
		return -EINVAL;

	while (i < ARRAY_SIZE(als_gains)) {
		if (gain == als_gains[i])
			break;
		i++;
	}

	if (i >= ARRAY_SIZE(als_gains)) {
		dev_err(&chip->client->dev, "%s: wrong als gain %d\n",
				__func__, (int)gain);
		return -EINVAL;
	}

	AMS_MUTEX_LOCK(&chip->lock);

	if (gain) {
		chip->params.als_auto_gain = false;
		rc = tcs3430_set_als_gain(chip, als_gains[i]);
		if (!rc)
			tcs3430_calc_cpl(chip);
	} else {
		chip->params.als_auto_gain = true;
	}
	tcs3430_flush_als_regs(chip);

	AMS_MUTEX_UNLOCK(&chip->lock);

	return rc ? -EIO : size;
}

static ssize_t tcs3430_als_z_raw_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tcs3430_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_inf.z_raw);
}

static ssize_t tcs3430_als_y_raw_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tcs3430_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_inf.y_raw);
}

static ssize_t tcs3430_als_cpl_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tcs3430_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_inf.cpl);
}

static ssize_t tcs3430_als_ir_raw_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tcs3430_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_inf.ir_raw);
}

static ssize_t tcs3430_als_x_raw_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tcs3430_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_inf.x_raw);
}

static ssize_t tcs3430_als_cct_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tcs3430_chip *chip = dev_get_drvdata(dev);
	tcs3430_read_als(chip);
	tcs3430_get_lux(chip);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_inf.cct);
}

static ssize_t tcs3430_als_persist_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tcs3430_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n",
			(((chip->shadow[TCS3430_REG_PERS]) & TCS3430_MASK_APERS)));
}

static ssize_t tcs3430_als_persist_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	long persist;
	int rc;
	struct tcs3430_chip *chip = dev_get_drvdata(dev);

	rc = kstrtoul(buf, 10, &persist);
	if (rc)
		return -EINVAL;

	AMS_MUTEX_LOCK(&chip->lock);
	chip->shadow[TCS3430_REG_PERS] &= ~TCS3430_MASK_APERS;
	chip->shadow[TCS3430_REG_PERS] |= ((u8)persist & TCS3430_MASK_APERS);

	tcs3430_flush_als_regs(chip);

	AMS_MUTEX_UNLOCK(&chip->lock);
	return size;
}

static ssize_t tcs3430_als_atime_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tcs3430_chip *chip = dev_get_drvdata(dev);
	int t;

	t = chip->shadow[TCS3430_REG_ATIME];
	t += 1; //t = INTEGRATION_CYCLE if atime == 0
	t *= INTEGRATION_CYCLE;
	return snprintf(buf, PAGE_SIZE, "%dms (%dus)\n", t / 1000, t);
}

static ssize_t tcs3430_als_atime_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	long atime;
	int rc;
	struct tcs3430_chip *chip = dev_get_drvdata(dev);

	rc = kstrtoul(buf, 10, &atime);
	if (rc)
		return -EINVAL;
	atime = AW_TIME_MS(atime); //Assume value in ms

	AMS_MUTEX_LOCK(&chip->lock);

	chip->shadow[TCS3430_REG_ATIME] = (u8) atime;
	chip->params.als_time = chip->shadow[TCS3430_REG_ATIME];
	tcs3430_calc_cpl(chip);
	tcs3430_flush_als_regs(chip);

	AMS_MUTEX_UNLOCK(&chip->lock);

	return size;
}

static ssize_t tcs3430_als_wtime_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int t;
	u8 wlongcurr;
	struct tcs3430_chip *chip = dev_get_drvdata(dev);

	AMS_MUTEX_LOCK(&chip->lock);

	t = chip->shadow[TCS3430_REG_WTIME];

	wlongcurr = chip->shadow[TCS3430_REG_CFG0] & TCS3430_MASK_WLONG;
	if (wlongcurr)
		t *= 12;

	t *= WAIT_CYCLE;
	t /= 1000;

	AMS_MUTEX_UNLOCK(&chip->lock);

	return snprintf(buf, PAGE_SIZE, "%d (in ms)\n", t);
}

static ssize_t tcs3430_als_wtime_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct tcs3430_chip *chip = dev_get_drvdata(dev);
	unsigned long wtime;
	int wlong;
	int rc;

	rc = kstrtoul(buf, 10, &wtime);
	if (rc)
		return -EINVAL;

	wtime *= 1000;
	if (wtime > (256 * WAIT_CYCLE))
	{
		wlong = 1;
		wtime /= 12;
	}
	else
		wlong = 0;
	wtime /= WAIT_CYCLE;

	AMS_MUTEX_LOCK(&chip->lock);

	chip->shadow[TCS3430_REG_WTIME] = (u8) wtime;
	if (wlong)
		chip->shadow[TCS3430_REG_CFG0] |= TCS3430_MASK_WLONG;
	else
		chip->shadow[TCS3430_REG_CFG0] &= ~TCS3430_MASK_WLONG;

	tcs3430_flush_als_regs(chip);

	AMS_MUTEX_UNLOCK(&chip->lock);
	return size;
}


static ssize_t tcs3430_als_deltaP_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tcs3430_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE,
			"%d (in %%)\n", chip->params.als_deltaP);
}

static ssize_t tcs3430_als_deltaP_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	unsigned long deltaP;
	int rc;
	struct tcs3430_chip *chip = dev_get_drvdata(dev);

	rc = kstrtoul(buf, 10, &deltaP);
	if (rc || deltaP > 100)
		return -EINVAL;
	AMS_MUTEX_LOCK(&chip->lock);
	chip->params.als_deltaP = deltaP;
	AMS_MUTEX_UNLOCK(&chip->lock);
	return size;
}

static ssize_t tcs3430_als_adc_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tcs3430_chip *chip = dev_get_drvdata(dev);

	tcs3430_get_lux(chip);

	return snprintf(buf, PAGE_SIZE, "lux: %d, cct: %d, x: %d, y: %d, "
			"z: %d, ir: %d\n",
			chip->als_inf.lux,
			chip->als_inf.cct,
			chip->als_inf.x_raw,
			chip->als_inf.y_raw,
			chip->als_inf.z_raw,
			chip->als_inf.ir_raw);
}

static ssize_t tcs3430_als_adc_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct tcs3430_chip *chip = dev_get_drvdata(dev);
	u32 z, y, ir, x;

	if (4 != sscanf(buf, "%10d,%10d,%10d,%10d", &x, &y, &z, &ir))
		return -EINVAL;

	AMS_MUTEX_LOCK(&chip->lock);

	chip->als_inf.x_raw = x;
	chip->als_inf.y_raw = y;
	chip->als_inf.z_raw = z;
	chip->als_inf.ir_raw = ir;

	AMS_MUTEX_UNLOCK(&chip->lock);
	return size;
}

struct device_attribute tcs3430_als_attrs[] = {
	__ATTR(als_atime,         0664, tcs3430_als_atime_show,
		tcs3430_als_atime_store),
	__ATTR(als_wtime,         0664, tcs3430_als_wtime_show,
		tcs3430_als_wtime_store),
	__ATTR(als_lux,           0444, tcs3430_device_als_lux,
		NULL),
	__ATTR(als_gain,          0664, tcs3430_als_gain_show,
		tcs3430_als_gain_store),
	__ATTR(als_cpl,           0444, tcs3430_als_cpl_show,
		NULL),
	__ATTR(als_cct,           0444, tcs3430_als_cct_show,
		NULL),
	__ATTR(als_z,           0444, tcs3430_als_z_raw_show,
		NULL),
	__ATTR(als_y,           0444, tcs3430_als_y_raw_show,
		NULL),
	__ATTR(als_ir,           0444, tcs3430_als_ir_raw_show,
		NULL),
	__ATTR(als_x,           0444, tcs3430_als_x_raw_show,
		NULL),
	__ATTR(als_thresh_deltaP, 0664, tcs3430_als_deltaP_show,
		tcs3430_als_deltaP_store),
	__ATTR(als_auto_gain,     0664, tcs3430_auto_gain_enable_show,
		tcs3430_auto_gain_enable_store),
	__ATTR(als_lux_coef,      0664, tcs3430_lux_coef_show,
		tcs3430_lux_coef_store),
	__ATTR(als_enable,   0664, tcs3430_als_enable_show,
		tcs3430_als_enable_store),
	__ATTR(als_persist,       0664, tcs3430_als_persist_show,
		tcs3430_als_persist_store),
	__ATTR(als_adc,           0664, tcs3430_als_adc_show,
		tcs3430_als_adc_store),
};

int tcs3430_als_attrs_size = ARRAY_SIZE(tcs3430_als_attrs);
