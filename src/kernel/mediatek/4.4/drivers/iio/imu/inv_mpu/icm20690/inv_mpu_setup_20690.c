/*
* Copyright (C) 2017-2019 InvenSense, Inc.
*
* This software is licensed under the terms of the GNU General Public
* License version 2, as published by the Free Software Foundation, and
* may be copied, distributed, and modified under those terms.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*/
#define pr_fmt(fmt) "inv_mpu: " fmt
#include "../inv_mpu_iio.h"

static int inv_calc_engine_dur(struct inv_mpu_state *st,
				struct inv_engine_info *ei)
{
	if (!ei->running_rate)
		return -EINVAL;
	if (st->mode_1k_on)
		ei->dur = ei->base_time_1k / ei->orig_rate;
	else if (st->mode_vr_on)
		ei->dur = ei->base_time_vr / ei->orig_rate;
	else
		ei->dur = ei->base_time / ei->orig_rate;
	ei->dur *= ei->divider;

	return 0;
}

static int inv_setup_eis(struct inv_mpu_state *st)
{
	int res;

	if (st->chip_config.eis_enable)
		res = inv_plat_single_write(st, REG_AMA_CTRL_NEW_1,
							BIT_ODR_DELAY_TIME_EN);
	else
		res = inv_plat_single_write(st, REG_AMA_CTRL_NEW_1, 0);

	return res;
}

static int inv_turn_on_fifo(struct inv_mpu_state *st)
{
	u8 int_en, fifo_en, mode, user;
	int r;
	char data;
	int div, sampling_rate;

	r = inv_plat_single_write(st, REG_FIFO_EN, 0);
	if (r)
		return r;
	r = inv_plat_single_write(st, REG_USER_CTRL, BIT_FIFO_RST | st->i2c_dis);
	if (r)
		return r;
	//dummy fifo reg read
	inv_plat_read(st, REG_FIFO_R_W, 1, &data);
	fifo_en = 0;
	int_en = 0;

	if (st->gesture_only_on && (!st->batch.timeout)) {
		if (st->chip_config.stationary_detect_enable)
			st->gesture_int_count = STATIONARY_DELAY_THRESHOLD;
		else
			st->gesture_int_count = WOM_DELAY_THRESHOLD;
		int_en |= BIT_WOM_ALL_INT_EN;
	}
	if (st->batch.timeout) {
		if (!st->batch.fifo_wm_th)
			int_en = BIT_DATA_RDY_EN;
	} else {
		int_en = BIT_DATA_RDY_EN;
	}
	if (st->sensor[SENSOR_GYRO].on)
		fifo_en |= BITS_GYRO_FIFO_EN;

	if (st->sensor[SENSOR_ACCEL].on)
		fifo_en |= BIT_ACCEL_FIFO_EN;
	if (st->sensor[SENSOR_TEMP].on)
		fifo_en |= BITS_TEMP_FIFO_EN;
	if (st->sensor[SENSOR_COMPASS].on)
		fifo_en |= BIT_SLV_0_FIFO_EN;
	r = inv_plat_single_write(st, REG_FIFO_EN, fifo_en);
	if (r)
		return r;
	r = inv_plat_single_write(st, REG_INT_ENABLE, int_en);
	if (r)
		return r;
	if (st->gesture_only_on && (!st->batch.timeout))
		mode = BIT_ACCEL_INTEL_EN | BIT_ACCEL_INTEL_MODE;
	else
		mode = 0;
	if (st->ois.en)
		mode |= BIT_ACCEL_FCHOICE_OIS_B;
	r = inv_plat_single_write(st, REG_ACCEL_INTEL_CTRL, mode);
	if ((st->eng_info[ENGINE_GYRO].running_rate > MAX_COMPASS_RATE) &&
				st->chip_config.compass_enable) {
		if (st->mode_1k_on) {
			div = 0;
		} else if (st->mode_vr_on) {
			div = 1;
		} else {
			if (st->chip_config.gyro_enable)
				div = st->eng_info[ENGINE_GYRO].divider - 1;
			else
				div = st->eng_info[ENGINE_ACCEL].divider - 1;
			if (st->chip_config.eis_enable)
				div = EIS_DIVIDER - 1;
		}
		sampling_rate = BASE_SAMPLE_RATE/(1+div);
		st->mag_divider = sampling_rate/MAX_COMPASS_RATE - 1;
		r = inv_plat_single_write(st, REG_I2C_SLV4_CTRL,
						st->mag_divider);
		if (r)
			return r;
		r = inv_plat_single_write(st, REG_I2C_MST_DELAY_CTRL,
						BIT_DELAY_ES_SHADOW |
						BIT_I2C_SLV1_DELAY_EN |
						BIT_I2C_SLV0_DELAY_EN);
	} else {
		r = inv_plat_single_write(st, REG_I2C_SLV4_CTRL, 0);
		if (r)
			return r;
		r = inv_plat_single_write(st, REG_I2C_MST_DELAY_CTRL,
						BIT_DELAY_ES_SHADOW);
	}
	if (r)
		return r;

	/* make data ready interrupt happens after aux i2c access */
	if (st->chip_config.compass_enable && st->mag_divider == 0)
		r = inv_plat_single_write(st, REG_I2C_MST_CTRL, BIT_WAIT_FOR_ES);
	else
		r = inv_plat_single_write(st, REG_I2C_MST_CTRL, 0);
	if (r)
		return r;

	/* The number of mag samples to be dropped
	 * in addtion to ts_algo->first_sample
	 */
	st->mag_first_drop_cnt = st->mag_divider;

	user = BIT_FIFO_EN;
	if (st->sensor[SENSOR_COMPASS].on)
		user |= BIT_I2C_MST_EN;
	r = inv_plat_single_write(st, REG_USER_CTRL, user | st->i2c_dis);

	inv_setup_eis(st);

	return r;
}

/*
 *  inv_reset_fifo() - Reset FIFO related registers.
 */
int inv_reset_fifo(struct inv_mpu_state *st, bool turn_off)
{
	int r, i;
	struct inv_timestamp_algo *ts_algo = &st->ts_algo;

	r = inv_turn_on_fifo(st);
	if (r)
		return r;

	ts_algo->last_run_time = get_time_ns();
	ts_algo->reset_ts = ts_algo->last_run_time;

	st->last_temp_comp_time = ts_algo->last_run_time;
	st->left_over_size = 0;
	for (i = 0; i < SENSOR_NUM_MAX; i++) {
		st->sensor[i].calib_flag = 0;
		st->sensor[i].sample_calib = 0;
		st->sensor[i].time_calib = ts_algo->last_run_time;
	}

	ts_algo->calib_counter = 0;

	return 0;
}

static int inv_turn_on_engine(struct inv_mpu_state *st)
{
	u8 v, w;
	int res;
	unsigned int wait_ms;
	struct inv_timestamp_algo *ts_algo = &st->ts_algo;

	w = 0;
	if (st->chip_config.gyro_enable | st->chip_config.accel_enable) {
		if (!st->chip_config.gyro_enable)
			w |= BIT_PWR_GYRO_STBY;
		if (!st->chip_config.accel_enable)
			w |= BIT_PWR_ACCEL_STBY;
	} else if (st->chip_config.compass_enable) {
		w |= BIT_PWR_GYRO_STBY;
	} else {
		w = (BIT_PWR_GYRO_STBY | BIT_PWR_ACCEL_STBY);
	}

	res = inv_plat_read(st, REG_PWR_MGMT_2, 1, &v);
	if (res)
		return res;
	res = inv_plat_single_write(st, REG_PWR_MGMT_2, w);
	pr_debug("turn on engine REG %X\n", w);
	if (res)
		return res;

	wait_ms = 0;
	if (st->chip_config.gyro_enable && (v & BIT_PWR_GYRO_STBY)) {
		/* enabling gyro */
		wait_ms = INV_ICM20690_GYRO_START_TIME;
	}
	if (st->chip_config.accel_enable && (v & BIT_PWR_ACCEL_STBY)) {
		/* enabling accel */
		if (INV_ICM20690_ACCEL_START_TIME > wait_ms)
			wait_ms = INV_ICM20690_ACCEL_START_TIME;
	}

	/* nb of first drop samples */
	if (wait_ms < FIRST_DROP_SAMPL_MS)
		wait_ms = FIRST_DROP_SAMPL_MS;

	/* assume st->smplrt_div is set before coming here */
	ts_algo->first_sample = wait_ms / (st->smplrt_div + 1);
	if (ts_algo->first_sample == 0)
		ts_algo->first_sample = 1;
	if (st->mode_1k_on) {
		if (ts_algo->first_sample < MODE_1K_INIT_SAMPLE)
			ts_algo->first_sample = MODE_1K_INIT_SAMPLE;
	}
	if (st->mode_vr_on) {
		if (ts_algo->first_sample < MODE_VR_INIT_SAMPLE)
			ts_algo->first_sample = MODE_VR_INIT_SAMPLE;
	}
	pr_debug("first_sample= %d\n", ts_algo->first_sample);

	if (st->chip_config.has_compass) {
		if (st->chip_config.compass_enable)
			res = st->slave_compass->resume(st);
		else
			res = st->slave_compass->suspend(st);
		if (res)
			return res;
	}
	return res;
}

static int inv_setup_dmp_rate(struct inv_mpu_state *st)
{
	int i;

	for (i = 0; i < SENSOR_NUM_MAX; i++) {
		if (st->sensor[i].on) {
			st->cntl |= st->sensor[i].output;
			st->sensor[i].dur =
				st->eng_info[st->sensor[i].engine_base].dur;
			st->sensor[i].div = 1;
			pr_debug("dur=%d, i=%d\n", st->sensor[i].dur, i);
		}
	}

	return 0;
}

/*
 *  inv_set_lpf_gyro_cycle_avgcfg() - set gyro cycle mode and
 * set avg filter config based on rate.
 */
static int inv_set_lpf_gyro_cycle_avgfltr(struct inv_mpu_state *st, int rate)
{
	const short odr[] = {30, 50, 100, 200, 500};
	const int fltrconfig[] = {
		BIT_AVG_FILTER_30HZ,
		BIT_AVG_FILTER_50HZ,
		BIT_AVG_FILTER_100HZ,
		BIT_AVG_FILTER_200HZ,
		BIT_AVG_FILTER_500HZ
	};

	int i, result, data;

	i = 0;
	while ((rate > odr[i]) && (i < ARRAY_SIZE(fltrconfig) - 1))
		i++;
	data = (BIT_GYRO_CYCLE_EN | fltrconfig[i]);
	result = inv_plat_single_write(st, REG_LP_MODE_CTRL, data);
	return result;
}

/*
 *  inv_set_lpf() - set low pass filer based on fifo rate.
 */
static int inv_set_lpf(struct inv_mpu_state *st, int rate)
{
	const short hz[] = {188, 98, 42, 20, 10, 5};
	const int   d[] = {INV_FILTER_188HZ, INV_FILTER_98HZ,
			INV_FILTER_42HZ, INV_FILTER_20HZ,
			INV_FILTER_10HZ, INV_FILTER_5HZ};
	int i, h, data, result;

	if (st->chip_config.eis_enable || st->ois.en ||
		st->mode_1k_on || st->mode_vr_on) {
		h = (rate >> 1);
		i = 0;
		while ((h < hz[i]) && (i < ARRAY_SIZE(d) - 1))
			i++;
		data = d[i];
		data |= (EXT_SYNC_SET | BIT_FIFO_COUNT_REC);
		result = inv_plat_single_write(st, REG_CONFIG, data);
		if (result)
			return result;

		st->chip_config.lpf = data;
		result = inv_plat_single_write(st, REG_LP_MODE_CTRL, 0);
		st->gyro_lp_mode = 0;
	} else {
		result = inv_set_lpf_gyro_cycle_avgfltr(st, rate);
		if (result)
			return result;
		data = (BIT_FIFO_COUNT_REC);
		result = inv_plat_single_write(st, REG_CONFIG, data | 3);
		st->gyro_lp_mode = 1;
	}

	return result;
}

static int inv_set_div(struct inv_mpu_state *st, int a_d, int g_d)
{
	int result, div;

	if (st->chip_config.gyro_enable)
		div = g_d;
	else
		div = a_d;
	if (st->chip_config.eis_enable)
		div = EIS_DIVIDER - 1;

	pr_debug("div= %d\n", div);
	st->smplrt_div = div;
	result = inv_plat_single_write(st, REG_SAMPLE_RATE_DIV, div);

	return result;
}

static int inv_set_batch(struct inv_mpu_state *st)
{
	int res = 0;
	u32 w;

	if (st->batch.timeout) {
		w = st->batch.timeout * st->eng_info[ENGINE_GYRO].running_rate
					* st->batch.pk_size / 1000;
		if (w > FIFO_SIZE)
			w = FIFO_SIZE;
	} else {
		w = 0;
	}
	if (st->batch.pk_size)
		w /= st->batch.pk_size;
	st->batch.fifo_wm_th = w;
	pr_debug("running= %d, pksize=%d, to=%d w=%d\n",
		st->eng_info[ENGINE_GYRO].running_rate,
		st->batch.pk_size, st->batch.timeout, w);
	res = inv_plat_single_write(st, REG_FIFO_WM_TH, w & 0xff);
	if (res)
		return res;
	res = inv_plat_single_write(st, REG_FIFO_WM_TH_HI, (w >> 8) & 0xff);

	return res;
}

static int inv_set_rate(struct inv_mpu_state *st)
{
	int g_d, a_d, result, i;

	result = inv_setup_dmp_rate(st);
	if (result)
		return result;

	g_d = st->eng_info[ENGINE_GYRO].divider - 1;
	a_d = st->eng_info[ENGINE_ACCEL].divider - 1;
	result = inv_set_div(st, a_d, g_d);
	if (result)
		return result;
	result = inv_set_lpf(st, st->eng_info[ENGINE_GYRO].running_rate);
	st->batch.pk_size = 0;
	for (i = 0; i < SENSOR_NUM_MAX; i++) {
		if (st->sensor[i].on)
			st->batch.pk_size +=  st->sensor[i].sample_size;
	}

	inv_set_batch(st);

	return result;
}

static int inv_determine_engine(struct inv_mpu_state *st)
{
	int i;
	bool a_en, g_en, c_en;
	int compass_rate, accel_rate, gyro_rate;

	a_en = false;
	g_en = false;
	c_en = false;
	compass_rate = MPU_INIT_SENSOR_RATE;
	gyro_rate = MPU_INIT_SENSOR_RATE;
	accel_rate = MPU_INIT_SENSOR_RATE;
	/* loop the streaming sensors to see which engine needs to be turned on
		*/
	for (i = 0; i < SENSOR_NUM_MAX; i++) {
		if (st->sensor[i].on) {
			a_en |= st->sensor[i].a_en;
			g_en |= st->sensor[i].g_en;
			c_en |= st->sensor[i].c_en;
			if (st->sensor[i].c_en)
				compass_rate =
				    max(compass_rate, st->sensor[i].rate);
		}
	}
	if (st->ois.en)
		g_en = true;

	st->mag_divider = 0;
	st->mag_first_drop_cnt = 0;

	if (st->chip_config.eis_enable)
		st->sensor[SENSOR_TEMP].on = true;
	else
		st->sensor[SENSOR_TEMP].on = false;

	if (st->chip_config.eis_enable) {
		g_en = true;
		st->eis.frame_count = 0;
		st->eis.fsync_delay = 0;
		st->eis.gyro_counter = 0;
		st->eis.voting_count = 0;
		st->eis.voting_count_sub = 0;
		gyro_rate = BASE_SAMPLE_RATE;
	} else {
		st->eis.eis_triggered = false;
		st->eis.prev_state = false;
	}

	if (compass_rate > MAX_COMPASS_RATE)
		compass_rate = MAX_COMPASS_RATE;
	st->chip_config.compass_rate = compass_rate;
	accel_rate = st->sensor[SENSOR_ACCEL].rate;
	gyro_rate  = max(gyro_rate, st->sensor[SENSOR_GYRO].rate);

	if (compass_rate < MIN_COMPASS_RATE)
		compass_rate = MIN_COMPASS_RATE;
	st->ts_algo.clock_base = ENGINE_ACCEL;
	if (c_en && (!g_en) && (!a_en)) {
		a_en = true;
		accel_rate = compass_rate;
	}
	if (g_en) {
		/* gyro engine needs to be fastest */
		if (a_en)
			gyro_rate = max(gyro_rate, accel_rate);
		if (c_en) {
			if (gyro_rate < compass_rate)
				gyro_rate = max(gyro_rate, compass_rate);
		}
		accel_rate = gyro_rate;
		compass_rate = gyro_rate;
		st->ts_algo.clock_base = ENGINE_GYRO;
		st->sensor[SENSOR_COMPASS].engine_base = ENGINE_GYRO;
	} else if (a_en) {
		/* accel engine needs to be fastest if gyro engine is off */
		if (c_en) {
			if (accel_rate < compass_rate)
				accel_rate = max(accel_rate, compass_rate);
		}
		compass_rate = accel_rate;
		gyro_rate = accel_rate;
		st->ts_algo.clock_base = ENGINE_ACCEL;
		st->sensor[SENSOR_COMPASS].engine_base = ENGINE_ACCEL;
		st->sensor[SENSOR_TEMP].on = true;
	} else if (c_en) {
		gyro_rate = compass_rate;
		accel_rate = compass_rate;
	}

	st->eng_info[ENGINE_GYRO].running_rate = gyro_rate;
	st->eng_info[ENGINE_ACCEL].running_rate = accel_rate;
	st->eng_info[ENGINE_I2C].running_rate = compass_rate;
	if ((gyro_rate > MPU_DEFAULT_DMP_FREQ) ||
					(accel_rate > MPU_DEFAULT_DMP_FREQ)) {
		if ((gyro_rate == MODE_VR_RATE) ||
				(accel_rate == MODE_VR_RATE))
			st->mode_vr_on = true;
		else
			st->mode_1k_on = true;
	} else {
		st->mode_1k_on = false;
		st->mode_vr_on = false;
	}

	if (st->chip_config.eis_enable) {
		st->eng_info[ENGINE_GYRO].divider = EIS_DIVIDER;
		st->eng_info[ENGINE_ACCEL].divider = EIS_DIVIDER;
		st->eng_info[ENGINE_I2C].divider = EIS_DIVIDER;
	} else if (st->mode_1k_on) {
		st->eng_info[ENGINE_GYRO].divider = 1;
		st->eng_info[ENGINE_ACCEL].divider = 1;
		st->eng_info[ENGINE_I2C].divider = 1;
		// need to update rate and div for 1khz mode
		for (i = 0; i < SENSOR_L_NUM_MAX; i++) {
			if (st->sensor_l[i].on) {
				if (st->sensor_l[i].rate)
					st->sensor_l[i].div =
						((BASE_SAMPLE_RATE /
						st->eng_info[ENGINE_GYRO].divider) /
						(st->sensor_l[i].rate));
				else
					st->sensor_l[i].div = 0xffff;
			}
		}
	} else if (st->mode_vr_on) {
		st->eng_info[ENGINE_GYRO].divider = 2;
		st->eng_info[ENGINE_ACCEL].divider = 2;
		st->eng_info[ENGINE_I2C].divider = 2;
		// need to update rate and div for VR mode
		for (i = 0; i < SENSOR_L_NUM_MAX; i++) {
			if (st->sensor_l[i].on) {
				if (st->sensor_l[i].rate)
					st->sensor_l[i].div =
						((BASE_SAMPLE_RATE /
						st->eng_info[ENGINE_GYRO].divider) /
						(st->sensor_l[i].rate));
				else
					st->sensor_l[i].div = 0xffff;
			}
		}
	} else {
		st->eng_info[ENGINE_GYRO].divider =
			(BASE_SAMPLE_RATE / MPU_DEFAULT_DMP_FREQ) *
			(MPU_DEFAULT_DMP_FREQ /
			st->eng_info[ENGINE_GYRO].running_rate);
		st->eng_info[ENGINE_ACCEL].divider =
			(BASE_SAMPLE_RATE / MPU_DEFAULT_DMP_FREQ) *
			(MPU_DEFAULT_DMP_FREQ /
			st->eng_info[ENGINE_ACCEL].running_rate);
		st->eng_info[ENGINE_I2C].divider =
			(BASE_SAMPLE_RATE / MPU_DEFAULT_DMP_FREQ) *
				(MPU_DEFAULT_DMP_FREQ /
					st->eng_info[ENGINE_I2C].running_rate);
	}

	for (i = 0; i < SENSOR_L_NUM_MAX; i++)
		st->sensor_l[i].counter = 0;
	for (i = 0; i < SENSOR_NUM_MAX; i++)
		st->sensor[i].ts_adj = 0;

	inv_calc_engine_dur(st, &st->eng_info[ENGINE_GYRO]);
	inv_calc_engine_dur(st, &st->eng_info[ENGINE_ACCEL]);

	pr_debug("gen: %d aen: %d cen: %d grate: %d arate: %d\n",
				g_en, a_en, c_en, gyro_rate, accel_rate);

	st->chip_config.gyro_enable = g_en;
	st->chip_config.accel_enable = a_en;
	st->chip_config.compass_enable = c_en;

	if (c_en)
		st->chip_config.slave_enable = 1;
	else
		st->chip_config.slave_enable = 0;

	st->cycle_on = (!st->eis.eis_triggered)
				&& (!st->chip_config.gyro_enable)
				&& (!st->chip_config.compass_enable)
				&& (!st->ois.en)
				&& (!st->mode_1k_on)
				&& (!st->mode_vr_on);

	return 0;
}

/*
 *  set_inv_enable() - enable function.
 */
int set_inv_enable(struct iio_dev *indio_dev)
{
	int result;
	struct inv_mpu_state *st = iio_priv(indio_dev);

	result = inv_switch_power_in_lp(st, true);
	if (result)
		return result;

	inv_stop_interrupt(st);
	inv_determine_engine(st);

	result = inv_set_rate(st);
	if (result) {
		pr_err("inv_set_rate error\n");
		return result;
	}
	result = inv_turn_on_engine(st);
	if (result) {
		pr_err("inv_turn_on_engine error\n");
		return result;
	}
	result = inv_reset_fifo(st, false);
	if (result)
		return result;

	result = inv_switch_power_in_lp(st, false);
	if (result)
		return result;

	if ((!st->chip_config.gyro_enable) &&
		(!st->chip_config.accel_enable) &&
		(!st->chip_config.compass_enable) && (!st->ois.en)) {
		inv_set_power(st, false);
		return 0;
	}

	return result;
}

static int inv_save_interrupt_config(struct inv_mpu_state *st)
{
	int res;

	res = inv_plat_read(st, REG_INT_ENABLE, 1, &st->int_en);

	return res;
}

int inv_stop_interrupt(struct inv_mpu_state *st)
{
	int res;

	res = inv_save_interrupt_config(st);
	if (res)
		return res;

	res = inv_plat_single_write(st, REG_INT_ENABLE, 0);

	return res;
}

int inv_restore_interrupt(struct inv_mpu_state *st)
{
	int res;

	res = inv_plat_single_write(st, REG_INT_ENABLE, st->int_en);

	return res;
}

int inv_stop_stream_interrupt(struct inv_mpu_state *st)
{
	return inv_stop_interrupt(st);
}

int inv_restore_stream_interrupt(struct inv_mpu_state *st)
{
	return inv_restore_interrupt(st);
}

