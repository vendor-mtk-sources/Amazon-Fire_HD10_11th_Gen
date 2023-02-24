/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 Hi556SEC_mipi_raw_Sensor.c
 *
 * Project:
 * --------
 *	 ALPS
 *
 * Description:
 * ------------
 *	 Source code of Sensor driver
 *
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/


#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/atomic.h>
#include <linux/types.h>

#include "hi556secmipiraw_Sensor.h"

#define PFX "hi556sec_camera_sensor"
#define LOG_INF(format, args...)    \
	pr_info(PFX "[%s] " format, __func__, ##args)
#define LOG_DBG(format, args...)    \
	pr_debug(PFX "[%s] " format, __func__, ##args)

#define MULTI_WRITE 1
static DEFINE_SPINLOCK(imgsensor_drv_lock);

#define per_frame 1

static struct imgsensor_info_struct imgsensor_info = {
	.sensor_id = HI556SEC_SENSOR_ID,
	.checksum_value = 0x55e2a82f,
	.pre = {
		.pclk = 176000000,
		.linelength = 2816,
		.framelength = 2083,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2592,
		.grabwindow_height = 1944,
		.mipi_data_lp2hs_settle_dc = 85,
		/* following for GetDefaultFramerateByScenario() */
		.max_framerate = 300,
		.mipi_pixel_rate = 176000000,
	},
	.cap = {
		.pclk = 176000000,
		.linelength = 2816,
		.framelength = 2083,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2592,
		.grabwindow_height = 1944,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
		.mipi_pixel_rate = 176000000,	/* 880Mx2/10 */
	},
	.cap1 = {
		/* capture for PIP 24fps relative information, capture1 mode must use same framelength, linelength with Capture mode for shutter calculate */
		.pclk = 176000000,
		.linelength = 2816,
		.framelength = 4166,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2592,
		.grabwindow_height = 1944,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 150,
		.mipi_pixel_rate = 176000000,
	},
	.normal_video = {
		.pclk = 176000000,
		.linelength = 2816,
		.framelength = 2083,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2592,
		.grabwindow_height = 1944,
		.mipi_data_lp2hs_settle_dc = 85,
		/* following for GetDefaultFramerateByScenario() */
		.max_framerate = 300,
		.mipi_pixel_rate = 176000000,
	},
	.hs_video = {
		.pclk = 176000000,
		.linelength = 2816,
		.framelength = 520,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 640,
		.grabwindow_height = 480,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 1200,
		.mipi_pixel_rate = 44000000,	/* 220Mx2/10 */
	},
	.slim_video = {
		.pclk = 176000000,
		.linelength = 2816,
		.framelength = 2083,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1280,
		.grabwindow_height = 720,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
		.mipi_pixel_rate = 88000000,
	},

	.margin = 2,
	.min_shutter = 6,
	.max_frame_length = 0x7FFF,
#if per_frame
	.ae_shut_delay_frame = 0,
	.ae_sensor_gain_delay_frame = 0,
	.ae_ispGain_delay_frame = 2,
#else
	.ae_shut_delay_frame = 0,
	.ae_sensor_gain_delay_frame = 1,
	.ae_ispGain_delay_frame = 2,
#endif
	.ihdr_support = 0,      /* 1, support; 0,not support */
	.ihdr_le_firstline = 0, /* 1,le first ; 0, se first */
	.sensor_mode_num = 5,	/* support sensor mode num */

	.cap_delay_frame = 3,
	.pre_delay_frame = 3,
	.video_delay_frame = 3,
	.hs_video_delay_frame = 3,	/* enter high speed video  delay frame num */
	.slim_video_delay_frame = 3,	/* enter slim video delay frame num */

	.isp_driving_current = ISP_DRIVING_6MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_OPHY_NCSI2,	/* 0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2 */
	.mipi_settle_delay_mode = 0,		/* 0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL */
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_Gb,
	.mclk = 24,
	.mipi_lane_num = SENSOR_MIPI_2_LANE,
	.i2c_addr_table = {0x50, 0xff},
	.i2c_speed = 400,
};

static struct imgsensor_struct imgsensor = {
	.mirror = IMAGE_NORMAL,
	.sensor_mode = IMGSENSOR_MODE_INIT,
	.shutter = 0x0100,
	.gain = 0xe0,
	.dummy_pixel = 0,
	.dummy_line = 0,
	/* full size current fps : 24fps for PIP, 30fps for Normal or ZSD */
	.current_fps = 300,
	.autoflicker_en = KAL_FALSE,
	.test_pattern = KAL_FALSE,
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,
	.ihdr_en = 0,
	.i2c_write_id = 0x50,
};

/* Sensor output window information */
static SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[5] = {
	{ 2592, 1944, 0, 0, 2592, 1944, 2592, 1944, 0, 0, 2592, 1944, 0, 0, 2592, 1944},    /* preview */
	{ 2592, 1944, 0, 0, 2592, 1944, 2592, 1944, 0, 0, 2592, 1944, 0, 0, 2592, 1944},    /* capture */
	{ 2592, 1944, 0, 0, 2592, 1944, 2592, 1944, 0, 0, 2592, 1944, 0, 0, 2592, 1944},    /* video */
	{ 2592, 1944, 16, 12, 2560, 1920, 640, 480, 0, 0, 640, 480, 0, 0, 640, 480},        /* high speed video */
	{ 2592, 1944, 16, 252, 2560, 1440, 1280, 720, 0, 0, 1280, 720, 0, 0, 1280, 720},    /* slim video */
};

#if MULTI_WRITE
#define I2C_BUFFER_LEN 765

static kal_uint16 hi556sec_table_write_cmos_sensor(
					kal_uint16 *para, kal_uint32 len)
{
	char puSendCmd[I2C_BUFFER_LEN];
	kal_uint32 tosend, IDX;
	kal_uint16 addr = 0, addr_last = 0, data;

	tosend = 0;
	IDX = 0;
	while (len > IDX) {
		addr = para[IDX];

		{
			puSendCmd[tosend++] = (char)(addr >> 8);
			puSendCmd[tosend++] = (char)(addr & 0xFF);
			data = para[IDX + 1];
			puSendCmd[tosend++] = (char)(data >> 8);
			puSendCmd[tosend++] = (char)(data & 0xFF);
			IDX += 2;
			addr_last = addr;
		}

		if ((I2C_BUFFER_LEN - tosend) < 4 ||
			len == IDX ||
			addr != addr_last) {
			iBurstWriteReg_multi(puSendCmd, tosend,
				imgsensor.i2c_write_id,
				4, imgsensor_info.i2c_speed);

			tosend = 0;
		}
	}
	return 0;
}
#endif

static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte = 0;
	char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };

	iReadRegI2C(pu_send_cmd, 2, (u8 *)&get_byte, 1, imgsensor.i2c_write_id);

	return get_byte;
}

static void write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
	char pu_send_cmd[4] = {(char)(addr >> 8),
		(char)(addr & 0xFF), (char)(para >> 8), (char)(para & 0xFF)};

	iWriteRegI2C(pu_send_cmd, 4, imgsensor.i2c_write_id);
}

static void write_cmos_sensor_8(kal_uint32 addr, kal_uint32 para)
{
	char pu_send_cmd[4] = {(char)(addr >> 8),
		(char)(addr & 0xFF), (char)(para & 0xFF)};

	iWriteRegI2C(pu_send_cmd, 3, imgsensor.i2c_write_id);
}

static void set_dummy(void)
{
	LOG_DBG("dummyline = %d, dummypixels = %d\n", imgsensor.dummy_line, imgsensor.dummy_pixel);
	write_cmos_sensor(0x0006, imgsensor.frame_length & 0xFFFF);
	write_cmos_sensor(0x0008, imgsensor.line_length & 0xFFFF);

}	/* set_dummy */

static kal_uint32 return_sensor_id(void)
{
	return ((read_cmos_sensor(0x0F16) << 8) | read_cmos_sensor(0x0F17));
}

static void set_max_framerate(UINT16 framerate, kal_bool min_framelength_en)
{
	kal_uint32 frame_length = imgsensor.frame_length;

	LOG_DBG("framerate = %d, min framelength should enable = %d\n", framerate, min_framelength_en);

	frame_length = imgsensor.pclk / framerate / imgsensor.line_length * 10;
	spin_lock(&imgsensor_drv_lock);
	imgsensor.frame_length = (frame_length > imgsensor.min_frame_length) ?
			frame_length : imgsensor.min_frame_length;
	imgsensor.dummy_line = imgsensor.frame_length -
		imgsensor.min_frame_length;

	if (imgsensor.frame_length > imgsensor_info.max_frame_length) {
		imgsensor.frame_length = imgsensor_info.max_frame_length;
		imgsensor.dummy_line = imgsensor.frame_length -
			imgsensor.min_frame_length;
	}
	if (min_framelength_en)
		imgsensor.min_frame_length = imgsensor.frame_length;

	spin_unlock(&imgsensor_drv_lock);
	set_dummy();
}	/* set_max_framerate */

static void write_shutter(kal_uint32 shutter)
{
	kal_uint32 realtime_fps = 0;

	spin_lock(&imgsensor_drv_lock);

	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);

	shutter = (shutter < imgsensor_info.min_shutter) ?
		imgsensor_info.min_shutter : shutter;
	shutter = (shutter >
		(imgsensor_info.max_frame_length - imgsensor_info.margin)) ?
		(imgsensor_info.max_frame_length - imgsensor_info.margin) :
		shutter;
	if (imgsensor.autoflicker_en) {
		realtime_fps = imgsensor.pclk / (imgsensor.line_length * imgsensor.frame_length) * 10;
		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146, 0);
		else
			write_cmos_sensor(0x0006, imgsensor.frame_length);

	} else{
		/* Extend frame length */
		write_cmos_sensor(0x0006, imgsensor.frame_length);
	}

	/* Update Shutter */
	write_cmos_sensor_8(0x0073, ((shutter & 0xFF0000) >> 16));
	write_cmos_sensor(0x0074, shutter & 0x00FFFF);
	LOG_DBG("shutter =%d, framelength =%d", shutter, imgsensor.frame_length);
}	/* write_shutter */

/*************************************************************************
 * FUNCTION
 *	set_shutter
 *
 * DESCRIPTION
 *	This function set e-shutter of sensor to change exposure time.
 *
 * PARAMETERS
 *	iShutter : exposured lines
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static void set_shutter(kal_uint32 shutter)
{
	unsigned long flags;

	LOG_DBG("set_shutter");
	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	write_shutter(shutter);
}	/* set_shutter */

static kal_uint16 gain2reg(const kal_uint16 gain)
{
	kal_uint16 reg_gain = 0x0000;

	reg_gain = gain / 4 - 16;

	return (kal_uint16)reg_gain;
}

/*************************************************************************
 * FUNCTION
 *	set_gain
 *
 * DESCRIPTION
 *	This function is to set global gain to sensor.
 *
 * PARAMETERS
 *	iGain : sensor global gain(base: 0x40)
 *
 * RETURNS
 *	the actually gain set to sensor.
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint16 set_gain(kal_uint16 gain)
{
	kal_uint16 reg_gain;

	/* 0x350A[0:1], 0x350B[0:7] AGC real gain */
	/* [0:3] = N meams N /16 X    */
	/* [4:9] = M meams M X        */
	/* Total gain = M + N /16 X   */

	if (gain < BASEGAIN || gain > 16 * BASEGAIN) {
		LOG_DBG("Error gain setting");

		if (gain < BASEGAIN)
			gain = BASEGAIN;
		else if (gain > 16 * BASEGAIN)
			gain = 16 * BASEGAIN;
	}

	reg_gain = gain2reg(gain);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.gain = reg_gain;
	spin_unlock(&imgsensor_drv_lock);
	LOG_DBG("gain = %d , reg_gain = 0x%x\n ", gain, reg_gain);

	reg_gain = reg_gain & 0x00FF;
	write_cmos_sensor_8(0x0077, reg_gain);

	return gain;

}	/* set_gain */

#if 0
static void ihdr_write_shutter_gain(kal_uint16 le,
				kal_uint16 se, kal_uint16 gain)
{
	LOG_DBG("le:0x%x, se:0x%x, gain:0x%x\n", le, se, gain);
	if (imgsensor.ihdr_en) {
		spin_lock(&imgsensor_drv_lock);
		if (le > imgsensor.min_frame_length - imgsensor_info.margin)
			imgsensor.frame_length = le + imgsensor_info.margin;
		else
			imgsensor.frame_length = imgsensor.min_frame_length;
		if (imgsensor.frame_length > imgsensor_info.max_frame_length)
			imgsensor.frame_length =
				imgsensor_info.max_frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (le < imgsensor_info.min_shutter)
			le = imgsensor_info.min_shutter;
		if (se < imgsensor_info.min_shutter)
			se = imgsensor_info.min_shutter;
		/* Extend frame length first */
		write_cmos_sensor(0x0006, imgsensor.frame_length);
		write_cmos_sensor(0x3502, (le << 4) & 0xFF);
		write_cmos_sensor(0x3501, (le >> 4) & 0xFF);
		write_cmos_sensor(0x3500, (le >> 12) & 0x0F);
		write_cmos_sensor(0x3508, (se << 4) & 0xFF);
		write_cmos_sensor(0x3507, (se >> 4) & 0xFF);
		write_cmos_sensor(0x3506, (se >> 12) & 0x0F);
		set_gain(gain);
	}
}
#endif

#if 0
static void set_mirror_flip(kal_uint8 image_mirror)
{
	LOG_DBG("image_mirror = %d", image_mirror);

	switch (image_mirror) {
	case IMAGE_NORMAL:
		write_cmos_sensor(0x0000, 0x0000);
		break;
	case IMAGE_H_MIRROR:
		write_cmos_sensor(0x0000, 0x0100);

		break;
	case IMAGE_V_MIRROR:
		write_cmos_sensor(0x0000, 0x0200);

		break;
	case IMAGE_HV_MIRROR:
		write_cmos_sensor(0x0000, 0x0300);

		break;
	default:
		LOG_DBG("Error image_mirror setting");
		break;
	}

}
#endif
/*************************************************************************
 * FUNCTION
 *	night_mode
 *
 * DESCRIPTION
 *	This function night mode of sensor.
 *
 * PARAMETERS
 *	bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static void night_mode(kal_bool enable)
{
	/*No Need to implement this function*/
}	/*	night_mode	*/

#if MULTI_WRITE
kal_uint16 addr_data_pair_init_hi556sec[] = {
	/* 0x0a00, 0x0000, */	/* stream off */
	0x0e00, 0x0102,
	0x0e02, 0x0102,
	0x0e0c, 0x0100,
	0x2000, 0x4031,
	0x2002, 0x8400,
	0x2004, 0x12b0,
	0x2006, 0xe292,
	0x2008, 0x12b0,
	0x200a, 0xe2b0,
	0x200c, 0x12b0,
	0x200e, 0xfce4,
	0x2010, 0x40b2,
	0x2012, 0x0250,
	0x2014, 0x8070,
	0x2016, 0x40b2,
	0x2018, 0x01b8,
	0x201a, 0x8072,
	0x201c, 0x93d2,
	0x201e, 0x00bd,
	0x2020, 0x248b,
	0x2022, 0x0900,
	0x2024, 0x7312,
	0x2026, 0x43d2,
	0x2028, 0x00bd,
	0x202a, 0x12b0,
	0x202c, 0xe60e,
	0x202e, 0x12b0,
	0x2030, 0xe64c,
	0x2032, 0x12b0,
	0x2034, 0xeb28,
	0x2036, 0x12b0,
	0x2038, 0xe662,
	0x203a, 0x12b0,
	0x203c, 0xe878,
	0x203e, 0x12b0,
	0x2040, 0xfc3a,
	0x2042, 0x12b0,
	0x2044, 0xfc62,
	0x2046, 0x12b0,
	0x2048, 0xfc7a,
	0x204a, 0x12b0,
	0x204c, 0xe2ca,
	0x204e, 0x12b0,
	0x2050, 0xe67c,
	0x2052, 0x12b0,
	0x2054, 0xe7dc,
	0x2056, 0xb3a2,
	0x2058, 0x0a84,
	0x205a, 0x2402,
	0x205c, 0x12b0,
	0x205e, 0xe9c8,
	0x2060, 0x4382,
	0x2062, 0x7322,
	0x2064, 0x4392,
	0x2066, 0x7326,
	0x2068, 0x12b0,
	0x206a, 0xf946,
	0x206c, 0x12b0,
	0x206e, 0xe438,
	0x2070, 0x12b0,
	0x2072, 0xe4a0,
	0x2074, 0x12b0,
	0x2076, 0xe4ee,
	0x2078, 0x12b0,
	0x207a, 0xe534,
	0x207c, 0x12b0,
	0x207e, 0xe586,
	0x2080, 0x12b0,
	0x2082, 0xe6ca,
	0x2084, 0x12b0,
	0x2086, 0xe6f0,
	0x2088, 0x12b0,
	0x208a, 0xe722,
	0x208c, 0x4392,
	0x208e, 0x731c,
	0x2090, 0x12b0,
	0x2092, 0xea52,
	0x2094, 0x12b0,
	0x2096, 0xfc8e,
	0x2098, 0x12b0,
	0x209a, 0xe5a2,
	0x209c, 0x93c2,
	0x209e, 0x8294,
	0x20a0, 0x2009,
	0x20a2, 0x0b00,
	0x20a4, 0x7302,
	0x20a6, 0x0258,
	0x20a8, 0x4382,
	0x20aa, 0x7902,
	0x20ac, 0x0900,
	0x20ae, 0x7308,
	0x20b0, 0x12b0,
	0x20b2, 0xf946,
	0x20b4, 0x12b0,
	0x20b6, 0xe342,
	0x20b8, 0x12b0,
	0x20ba, 0xe370,
	0x20bc, 0x12b0,
	0x20be, 0xfd10,
	0x20c0, 0x4292,
	0x20c2, 0x8292,
	0x20c4, 0x7114,
	0x20c6, 0x421f,
	0x20c8, 0x7316,
	0x20ca, 0xc312,
	0x20cc, 0x100f,
	0x20ce, 0x821f,
	0x20d0, 0x8298,
	0x20d2, 0x4f82,
	0x20d4, 0x7334,
	0x20d6, 0x0f00,
	0x20d8, 0x7302,
	0x20da, 0x12b0,
	0x20dc, 0xe6b8,
	0x20de, 0x43b2,
	0x20e0, 0x7a06,
	0x20e2, 0x12b0,
	0x20e4, 0xfcb0,
	0x20e6, 0x9392,
	0x20e8, 0x7114,
	0x20ea, 0x2011,
	0x20ec, 0x0b00,
	0x20ee, 0x7302,
	0x20f0, 0x0258,
	0x20f2, 0x4382,
	0x20f4, 0x7902,
	0x20f6, 0x0800,
	0x20f8, 0x7118,
	0x20fa, 0x12b0,
	0x20fc, 0xeb4c,
	0x20fe, 0x0900,
	0x2100, 0x7112,
	0x2102, 0x12b0,
	0x2104, 0xe3e6,
	0x2106, 0x0b00,
	0x2108, 0x7302,
	0x210a, 0x0036,
	0x210c, 0x3fec,
	0x210e, 0x0b00,
	0x2110, 0x7302,
	0x2112, 0x0036,
	0x2114, 0x4392,
	0x2116, 0x7902,
	0x2118, 0x4292,
	0x211a, 0x7100,
	0x211c, 0x82be,
	0x211e, 0x12b0,
	0x2120, 0xe74a,
	0x2122, 0x12b0,
	0x2124, 0xea8e,
	0x2126, 0x12b0,
	0x2128, 0xead4,
	0x212a, 0x12b0,
	0x212c, 0xe3e6,
	0x212e, 0x930f,
	0x2130, 0x27da,
	0x2132, 0x12b0,
	0x2134, 0xfcc8,
	0x2136, 0x3fb0,
	0x2138, 0x12b0,
	0x213a, 0xe5be,
	0x213c, 0x12b0,
	0x213e, 0xe5e6,
	0x2140, 0x3f70,
	0x2142, 0x4030,
	0x2144, 0xf770,
	0x2146, 0x12b0,
	0x2148, 0xee16,
	0x214a, 0x12b0,
	0x214c, 0xee5c,
	0x214e, 0x12b0,
	0x2150, 0xeeac,
	0x2152, 0x12b0,
	0x2154, 0xeeea,
	0x2156, 0x12b0,
	0x2158, 0xef06,
	0x215a, 0x12b0,
	0x215c, 0xef28,
	0x215e, 0x12b0,
	0x2160, 0xef98,
	0x2162, 0x12b0,
	0x2164, 0xfb2a,
	0x2166, 0x12b0,
	0x2168, 0xfa60,
	0x216a, 0x12b0,
	0x216c, 0xfb4c,
	0x216e, 0x12b0,
	0x2170, 0xf294,
	0x2172, 0xd392,
	0x2174, 0x7102,
	0x2176, 0x4130,
	0x2178, 0x120b,
	0x217a, 0x120a,
	0x217c, 0x1209,
	0x217e, 0x1208,
	0x2180, 0x1207,
	0x2182, 0x4f0c,
	0x2184, 0x4e08,
	0x2186, 0x4d47,
	0x2188, 0x425f,
	0x218a, 0x00de,
	0x218c, 0x4f4e,
	0x218e, 0x430f,
	0x2190, 0x421a,
	0x2192, 0x8308,
	0x2194, 0x421b,
	0x2196, 0x830a,
	0x2198, 0x8e0a,
	0x219a, 0x7f0b,
	0x219c, 0x4a0e,
	0x219e, 0x4b0f,
	0x21a0, 0x4c3a,
	0x21a2, 0x4c3b,
	0x21a4, 0x4a0c,
	0x21a6, 0x4b0d,
	0x21a8, 0x8e0c,
	0x21aa, 0x7f0d,
	0x21ac, 0x284a,
	0x21ae, 0x425f,
	0x21b0, 0x00de,
	0x21b2, 0x4f4a,
	0x21b4, 0x430b,
	0x21b6, 0xb0f2,
	0x21b8, 0x0010,
	0x21ba, 0x00dd,
	0x21bc, 0x2040,
	0x21be, 0x4349,
	0x21c0, 0x421e,
	0x21c2, 0x7560,
	0x21c4, 0x421f,
	0x21c6, 0x7578,
	0x21c8, 0x12b0,
	0x21ca, 0xefc4,
	0x21cc, 0x4a0c,
	0x21ce, 0x4b0d,
	0x21d0, 0x8e0c,
	0x21d2, 0x7f0d,
	0x21d4, 0x2c01,
	0x21d6, 0x4359,
	0x21d8, 0x434f,
	0x21da, 0x93c2,
	0x21dc, 0x8294,
	0x21de, 0x2001,
	0x21e0, 0x435f,
	0x21e2, 0xf94f,
	0x21e4, 0x434e,
	0x21e6, 0x93c8,
	0x21e8, 0x0000,
	0x21ea, 0x2001,
	0x21ec, 0x435e,
	0x21ee, 0xfe4f,
	0x21f0, 0x4fc8,
	0x21f2, 0x0000,
	0x21f4, 0x9347,
	0x21f6, 0x2013,
	0x21f8, 0x4a82,
	0x21fa, 0x7540,
	0x21fc, 0x4b82,
	0x21fe, 0x7574,
	0x2200, 0x934f,
	0x2202, 0x2428,
	0x2204, 0x93c2,
	0x2206, 0x00cd,
	0x2208, 0x2407,
	0x220a, 0x425f,
	0x220c, 0x806d,
	0x220e, 0x5f82,
	0x2210, 0x7542,
	0x2212, 0x5f82,
	0x2214, 0x7544,
	0x2216, 0x3c1e,
	0x2218, 0x425f,
	0x221a, 0x806c,
	0x221c, 0x3ff8,
	0x221e, 0x4a82,
	0x2220, 0x754a,
	0x2222, 0x4b82,
	0x2224, 0x7576,
	0x2226, 0x934f,
	0x2228, 0x2415,
	0x222a, 0x93c2,
	0x222c, 0x00cd,
	0x222e, 0x2412,
	0x2230, 0x425f,
	0x2232, 0x806d,
	0x2234, 0x5f82,
	0x2236, 0x754c,
	0x2238, 0x5f82,
	0x223a, 0x754e,
	0x223c, 0x3c0b,
	0x223e, 0x435f,
	0x2240, 0x3fd7,
	0x2242, 0x421e,
	0x2244, 0x8308,
	0x2246, 0x421f,
	0x2248, 0x830a,
	0x224a, 0x8a0e,
	0x224c, 0x7b0f,
	0x224e, 0x4e0a,
	0x2250, 0x4f0b,
	0x2252, 0x3fb1,
	0x2254, 0x4137,
	0x2256, 0x4138,
	0x2258, 0x4139,
	0x225a, 0x413a,
	0x225c, 0x413b,
	0x225e, 0x4130,
	0x2260, 0x425f,
	0x2262, 0x00d8,
	0x2264, 0xf37f,
	0x2266, 0x421e,
	0x2268, 0x0086,
	0x226a, 0x12b0,
	0x226c, 0xefc4,
	0x226e, 0x4e82,
	0x2270, 0x8308,
	0x2272, 0x4f82,
	0x2274, 0x830a,
	0x2276, 0x4292,
	0x2278, 0x0088,
	0x227a, 0x7316,
	0x227c, 0x425f,
	0x227e, 0x806f,
	0x2280, 0x4f82,
	0x2282, 0x7542,
	0x2284, 0x93c2,
	0x2286, 0x00cc,
	0x2288, 0x244d,
	0x228a, 0x521f,
	0x228c, 0x828c,
	0x228e, 0x4f82,
	0x2290, 0x7544,
	0x2292, 0x93c2,
	0x2294, 0x00cd,
	0x2296, 0x240d,
	0x2298, 0x425f,
	0x229a, 0x806f,
	0x229c, 0x425e,
	0x229e, 0x806e,
	0x22a0, 0x4f0d,
	0x22a2, 0x5e0d,
	0x22a4, 0x4d82,
	0x22a6, 0x754c,
	0x22a8, 0x521f,
	0x22aa, 0x828e,
	0x22ac, 0x5e0f,
	0x22ae, 0x4f82,
	0x22b0, 0x754e,
	0x22b2, 0x425f,
	0x22b4, 0x00f3,
	0x22b6, 0xf37f,
	0x22b8, 0x403d,
	0x22ba, 0x82ee,
	0x22bc, 0x421e,
	0x22be, 0x00f4,
	0x22c0, 0x12b0,
	0x22c2, 0xefd0,
	0x22c4, 0x93c2,
	0x22c6, 0x00cd,
	0x22c8, 0x2023,
	0x22ca, 0x434d,
	0x22cc, 0x403e,
	0x22ce, 0x8306,
	0x22d0, 0x403f,
	0x22d2, 0x82ee,
	0x22d4, 0x12b0,
	0x22d6, 0xf978,
	0x22d8, 0x93c2,
	0x22da, 0x00cd,
	0x22dc, 0x2011,
	0x22de, 0x4382,
	0x22e0, 0x733e,
	0x22e2, 0x93c2,
	0x22e4, 0x00cd,
	0x22e6, 0x240b,
	0x22e8, 0x421e,
	0x22ea, 0x82ee,
	0x22ec, 0x421f,
	0x22ee, 0x82f0,
	0x22f0, 0x821e,
	0x22f2, 0x82f2,
	0x22f4, 0x721f,
	0x22f6, 0x82f4,
	0x22f8, 0x2c02,
	0x22fa, 0x4392,
	0x22fc, 0x733e,
	0x22fe, 0x4130,
	0x2300, 0x435d,
	0x2302, 0x403e,
	0x2304, 0x8307,
	0x2306, 0x403f,
	0x2308, 0x82f2,
	0x230a, 0x12b0,
	0x230c, 0xf978,
	0x230e, 0x3fe7,
	0x2310, 0x425f,
	0x2312, 0x00f2,
	0x2314, 0xf37f,
	0x2316, 0x403d,
	0x2318, 0x82f2,
	0x231a, 0x421e,
	0x231c, 0x00f0,
	0x231e, 0x12b0,
	0x2320, 0xefd0,
	0x2322, 0x3fd3,
	0x2324, 0x521f,
	0x2326, 0x828e,
	0x2328, 0x3fb2,
	0x232a, 0x421e,
	0x232c, 0x7540,
	0x232e, 0x421f,
	0x2330, 0x7574,
	0x2332, 0x12b0,
	0x2334, 0xefc4,
	0x2336, 0x4e82,
	0x2338, 0x8346,
	0x233a, 0x4f82,
	0x233c, 0x8348,
	0x233e, 0x93c2,
	0x2340, 0x00c1,
	0x2342, 0x2403,
	0x2344, 0x42d2,
	0x2346, 0x834b,
	0x2348, 0x8310,
	0x234a, 0x4130,
	0x234c, 0x4292,
	0x234e, 0x7320,
	0x2350, 0x8344,
	0x2352, 0x93c2,
	0x2354, 0x00c1,
	0x2356, 0x2001,
	0x2358, 0x4130,
	0x235a, 0x12b0,
	0x235c, 0xfb64,
	0x235e, 0x12b0,
	0x2360, 0xfbe8,
	0x2362, 0x3ffa,
	0x2364, 0x120b,
	0x2366, 0x120a,
	0x2368, 0x1209,
	0x236a, 0x1208,
	0x236c, 0x421e,
	0x236e, 0x7314,
	0x2370, 0x421f,
	0x2372, 0x7336,
	0x2374, 0x12b0,
	0x2376, 0xefc4,
	0x2378, 0x4e0c,
	0x237a, 0x4f0d,
	0x237c, 0x4e82,
	0x237e, 0x8338,
	0x2380, 0x4f82,
	0x2382, 0x833a,
	0x2384, 0x421f,
	0x2386, 0x0196,
	0x2388, 0x4f0e,
	0x238a, 0x430f,
	0x238c, 0x4c0a,
	0x238e, 0x4d0b,
	0x2390, 0x8e0a,
	0x2392, 0x7f0b,
	0x2394, 0x4a0e,
	0x2396, 0x4b0f,
	0x2398, 0x803e,
	0x239a, 0x0102,
	0x239c, 0x730f,
	0x239e, 0x281b,
	0x23a0, 0x4039,
	0x23a2, 0x0101,
	0x23a4, 0x8329,
	0x23a6, 0x4982,
	0x23a8, 0x7320,
	0x23aa, 0x421e,
	0x23ac, 0x7540,
	0x23ae, 0x421f,
	0x23b0, 0x7574,
	0x23b2, 0x12b0,
	0x23b4, 0xefc4,
	0x23b6, 0x4e82,
	0x23b8, 0x8332,
	0x23ba, 0x4f82,
	0x23bc, 0x8334,
	0x23be, 0x4348,
	0x23c0, 0x5039,
	0x23c2, 0xfffa,
	0x23c4, 0x490c,
	0x23c6, 0x430d,
	0x23c8, 0x5e0c,
	0x23ca, 0x6f0d,
	0x23cc, 0x8a0c,
	0x23ce, 0x7b0d,
	0x23d0, 0x2804,
	0x23d2, 0x4358,
	0x23d4, 0x3c02,
	0x23d6, 0x4a09,
	0x23d8, 0x3fe5,
	0x23da, 0x48c2,
	0x23dc, 0x834b,
	0x23de, 0x4138,
	0x23e0, 0x4139,
	0x23e2, 0x413a,
	0x23e4, 0x413b,
	0x23e6, 0x4130,
	0x23e8, 0x120b,
	0x23ea, 0x120a,
	0x23ec, 0x93c2,
	0x23ee, 0x8294,
	0x23f0, 0x2021,
	0x23f2, 0x93c2,
	0x23f4, 0x8310,
	0x23f6, 0x2404,
	0x23f8, 0x43b2,
	0x23fa, 0x7574,
	0x23fc, 0x43b2,
	0x23fe, 0x7540,
	0x2400, 0x93c2,
	0x2402, 0x834b,
	0x2404, 0x2417,
	0x2406, 0x421e,
	0x2408, 0x8332,
	0x240a, 0x421f,
	0x240c, 0x8334,
	0x240e, 0x532e,
	0x2410, 0x630f,
	0x2412, 0x4e82,
	0x2414, 0x8332,
	0x2416, 0x4f82,
	0x2418, 0x8334,
	0x241a, 0x421a,
	0x241c, 0x8338,
	0x241e, 0x421b,
	0x2420, 0x833a,
	0x2422, 0x4a0c,
	0x2424, 0x4b0d,
	0x2426, 0x8e0c,
	0x2428, 0x7f0d,
	0x242a, 0x2c04,
	0x242c, 0x4a82,
	0x242e, 0x8332,
	0x2430, 0x4b82,
	0x2432, 0x8334,
	0x2434, 0x413a,
	0x2436, 0x413b,
	0x2438, 0x4130,
	0x243a, 0x93c2,
	0x243c, 0x00c3,
	0x243e, 0x240e,
	0x2440, 0x93c2,
	0x2442, 0x00c1,
	0x2444, 0x200b,
	0x2446, 0x43d2,
	0x2448, 0x834a,
	0x244a, 0x93d2,
	0x244c, 0x00bf,
	0x244e, 0x2403,
	0x2450, 0x43c2,
	0x2452, 0x834c,
	0x2454, 0x4130,
	0x2456, 0x43d2,
	0x2458, 0x834c,
	0x245a, 0x4130,
	0x245c, 0x43c2,
	0x245e, 0x834a,
	0x2460, 0x3ff4,
	0x2462, 0x93c2,
	0x2464, 0x834c,
	0x2466, 0x2408,
	0x2468, 0xb3e2,
	0x246a, 0x00c2,
	0x246c, 0x2403,
	0x246e, 0x43d2,
	0x2470, 0x00c8,
	0x2472, 0x4130,
	0x2474, 0x43c2,
	0x2476, 0x00c8,
	0x2478, 0x4130,
	0x247a, 0x93c2,
	0x247c, 0x834a,
	0x247e, 0x2406,
	0x2480, 0x403f,
	0x2482, 0x00c3,
	0x2484, 0x4fe2,
	0x2486, 0x8330,
	0x2488, 0xc3df,
	0x248a, 0x0000,
	0x248c, 0x4130,
	0x248e, 0x93c2,
	0x2490, 0x834a,
	0x2492, 0x2403,
	0x2494, 0x42d2,
	0x2496, 0x8330,
	0x2498, 0x00c3,
	0x249a, 0x421e,
	0x249c, 0x7314,
	0x249e, 0x421f,
	0x24a0, 0x7336,
	0x24a2, 0x12b0,
	0x24a4, 0xefc4,
	0x24a6, 0x4e82,
	0x24a8, 0x831c,
	0x24aa, 0x4f82,
	0x24ac, 0x831e,
	0x24ae, 0x4130,
	0x24b0, 0x93c2,
	0x24b2, 0x834c,
	0x24b4, 0x2408,
	0x24b6, 0xb3e2,
	0x24b8, 0x00c2,
	0x24ba, 0x2403,
	0x24bc, 0x43c2,
	0x24be, 0x00c8,
	0x24c0, 0x4130,
	0x24c2, 0x43d2,
	0x24c4, 0x00c8,
	0x24c6, 0x4130,
	0x24c8, 0x93c2,
	0x24ca, 0x834c,
	0x24cc, 0x2408,
	0x24ce, 0xb3e2,
	0x24d0, 0x00c2,
	0x24d2, 0x2403,
	0x24d4, 0x43d2,
	0x24d6, 0x00c8,
	0x24d8, 0x3c02,
	0x24da, 0x43c2,
	0x24dc, 0x00c8,
	0x24de, 0x43c2,
	0x24e0, 0x8294,
	0x24e2, 0x4130,
	0x24e4, 0x43c2,
	0x24e6, 0x834b,
	0x24e8, 0x4130,
	0x24ea, 0x403e,
	0x24ec, 0x00c2,
	0x24ee, 0x421f,
	0x24f0, 0x7312,
	0x24f2, 0xf07f,
	0x24f4, 0x000c,
	0x24f6, 0x5f4f,
	0x24f8, 0x5f4f,
	0x24fa, 0xdfce,
	0x24fc, 0x0000,
	0x24fe, 0xf0fe,
	0x2500, 0x000f,
	0x2502, 0x0000,
	0x2504, 0x4130,
	0x2506, 0x4f82,
	0x2508, 0x7334,
	0x250a, 0x0f00,
	0x250c, 0x7300,
	0x250e, 0x4130,
	0x2510, 0x421f,
	0x2512, 0x0196,
	0x2514, 0x503f,
	0x2516, 0x0006,
	0x2518, 0x4f82,
	0x251a, 0x832e,
	0x251c, 0x93c2,
	0x251e, 0x00c1,
	0x2520, 0x242d,
	0x2522, 0x425e,
	0x2524, 0x00c2,
	0x2526, 0xc35e,
	0x2528, 0x425f,
	0x252a, 0x8310,
	0x252c, 0xdf4e,
	0x252e, 0x4ec2,
	0x2530, 0x00c2,
	0x2532, 0x934f,
	0x2534, 0x2006,
	0x2536, 0x9382,
	0x2538, 0x730a,
	0x253a, 0x27fd,
	0x253c, 0xd3d2,
	0x253e, 0x00c2,
	0x2540, 0x3c1f,
	0x2542, 0x12b0,
	0x2544, 0xfd86,
	0x2546, 0x12b0,
	0x2548, 0xfdbc,
	0x254a, 0x421f,
	0x254c, 0x8342,
	0x254e, 0x933f,
	0x2550, 0x380a,
	0x2552, 0x932f,
	0x2554, 0x3803,
	0x2556, 0x12b0,
	0x2558, 0xff36,
	0x255a, 0x3ff0,
	0x255c, 0x12b0,
	0x255e, 0xff52,
	0x2560, 0x12b0,
	0x2562, 0xff9c,
	0x2564, 0x3feb,
	0x2566, 0x12b0,
	0x2568, 0xfe06,
	0x256a, 0x12b0,
	0x256c, 0xfe36,
	0x256e, 0x12b0,
	0x2570, 0xfe68,
	0x2572, 0x12b0,
	0x2574, 0xfec0,
	0x2576, 0x12b0,
	0x2578, 0xfefe,
	0x257a, 0x3fe0,
	0x257c, 0x0900,
	0x257e, 0x7328,
	0x2580, 0x12b0,
	0x2582, 0xffc2,
	0x2584, 0x4130,
	0x2586, 0x421e,
	0x2588, 0x7314,
	0x258a, 0x421f,
	0x258c, 0x7336,
	0x258e, 0x12b0,
	0x2590, 0xefc4,
	0x2592, 0x4e82,
	0x2594, 0x832a,
	0x2596, 0x4f82,
	0x2598, 0x832c,
	0x259a, 0xb3e2,
	0x259c, 0x00c2,
	0x259e, 0x2407,
	0x25a0, 0x9382,
	0x25a2, 0x7318,
	0x25a4, 0x27fd,
	0x25a6, 0x9392,
	0x25a8, 0x7318,
	0x25aa, 0x27fd,
	0x25ac, 0x4130,
	0x25ae, 0x9392,
	0x25b0, 0x7318,
	0x25b2, 0x27fd,
	0x25b4, 0x9382,
	0x25b6, 0x7318,
	0x25b8, 0x27fd,
	0x25ba, 0x4130,
	0x25bc, 0x120b,
	0x25be, 0x120a,
	0x25c0, 0x421e,
	0x25c2, 0x7300,
	0x25c4, 0x421f,
	0x25c6, 0x733c,
	0x25c8, 0x12b0,
	0x25ca, 0xefc4,
	0x25cc, 0x4e82,
	0x25ce, 0x8324,
	0x25d0, 0x4f82,
	0x25d2, 0x8326,
	0x25d4, 0x421c,
	0x25d6, 0x8344,
	0x25d8, 0x430d,
	0x25da, 0x421a,
	0x25dc, 0x831c,
	0x25de, 0x421b,
	0x25e0, 0x831e,
	0x25e2, 0x8c0a,
	0x25e4, 0x7d0b,
	0x25e6, 0x8a0e,
	0x25e8, 0x7b0f,
	0x25ea, 0x2c04,
	0x25ec, 0x4292,
	0x25ee, 0x8324,
	0x25f0, 0x8342,
	0x25f2, 0x3c06,
	0x25f4, 0x421b,
	0x25f6, 0x8324,
	0x25f8, 0x821b,
	0x25fa, 0x831c,
	0x25fc, 0x4b82,
	0x25fe, 0x8342,
	0x2600, 0x413a,
	0x2602, 0x413b,
	0x2604, 0x4130,
	0x2606, 0x4382,
	0x2608, 0x8336,
	0x260a, 0x50b2,
	0x260c, 0xfffd,
	0x260e, 0x8342,
	0x2610, 0x4382,
	0x2612, 0x7336,
	0x2614, 0x43a2,
	0x2616, 0x7314,
	0x2618, 0x403e,
	0x261a, 0x7542,
	0x261c, 0x4ea2,
	0x261e, 0x833e,
	0x2620, 0x403f,
	0x2622, 0x7544,
	0x2624, 0x4fa2,
	0x2626, 0x8340,
	0x2628, 0x429e,
	0x262a, 0x8320,
	0x262c, 0x0000,
	0x262e, 0x429f,
	0x2630, 0x8322,
	0x2632, 0x0000,
	0x2634, 0x4130,
	0x2636, 0x421e,
	0x2638, 0x8324,
	0x263a, 0x421f,
	0x263c, 0x8326,
	0x263e, 0x821e,
	0x2640, 0x8346,
	0x2642, 0x721f,
	0x2644, 0x8348,
	0x2646, 0x2c0a,
	0x2648, 0x4382,
	0x264a, 0x7574,
	0x264c, 0x4382,
	0x264e, 0x7540,
	0x2650, 0x421f,
	0x2652, 0x831c,
	0x2654, 0x821f,
	0x2656, 0x8346,
	0x2658, 0x4f82,
	0x265a, 0x8336,
	0x265c, 0x12b0,
	0x265e, 0xfcea,
	0x2660, 0x0b00,
	0x2662, 0x7302,
	0x2664, 0x0578,
	0x2666, 0x4130,
	0x2668, 0x4392,
	0x266a, 0x7318,
	0x266c, 0x4292,
	0x266e, 0x833e,
	0x2670, 0x7542,
	0x2672, 0x4292,
	0x2674, 0x8340,
	0x2676, 0x7544,
	0x2678, 0x421e,
	0x267a, 0x8342,
	0x267c, 0x4e0f,
	0x267e, 0x5f0f,
	0x2680, 0x7f0f,
	0x2682, 0xe33f,
	0x2684, 0x521e,
	0x2686, 0x8332,
	0x2688, 0x621f,
	0x268a, 0x8334,
	0x268c, 0x4e82,
	0x268e, 0x8332,
	0x2690, 0x4f82,
	0x2692, 0x8334,
	0x2694, 0x421d,
	0x2696, 0x8336,
	0x2698, 0x522d,
	0x269a, 0x4d0c,
	0x269c, 0x4c0d,
	0x269e, 0x5d0d,
	0x26a0, 0x7d0d,
	0x26a2, 0xe33d,
	0x26a4, 0x8c0e,
	0x26a6, 0x7d0f,
	0x26a8, 0x2c04,
	0x26aa, 0x4c82,
	0x26ac, 0x8332,
	0x26ae, 0x4d82,
	0x26b0, 0x8334,
	0x26b2, 0x4292,
	0x26b4, 0x8334,
	0x26b6, 0x7574,
	0x26b8, 0x4292,
	0x26ba, 0x8332,
	0x26bc, 0x7540,
	0x26be, 0x4130,
	0x26c0, 0x421e,
	0x26c2, 0x8342,
	0x26c4, 0x4e0f,
	0x26c6, 0x5f0f,
	0x26c8, 0x7f0f,
	0x26ca, 0xe33f,
	0x26cc, 0x521e,
	0x26ce, 0x832a,
	0x26d0, 0x621f,
	0x26d2, 0x832c,
	0x26d4, 0x4e82,
	0x26d6, 0x832a,
	0x26d8, 0x4f82,
	0x26da, 0x832c,
	0x26dc, 0x421c,
	0x26de, 0x832e,
	0x26e0, 0x430d,
	0x26e2, 0x8c0e,
	0x26e4, 0x7d0f,
	0x26e6, 0x2c04,
	0x26e8, 0x4c82,
	0x26ea, 0x832a,
	0x26ec, 0x4d82,
	0x26ee, 0x832c,
	0x26f0, 0x4292,
	0x26f2, 0x832c,
	0x26f4, 0x7336,
	0x26f6, 0x4292,
	0x26f8, 0x832a,
	0x26fa, 0x7314,
	0x26fc, 0x4130,
	0x26fe, 0x4292,
	0x2700, 0x7320,
	0x2702, 0x8328,
	0x2704, 0x421e,
	0x2706, 0x8336,
	0x2708, 0x4e0f,
	0x270a, 0x821f,
	0x270c, 0x8342,
	0x270e, 0x4f82,
	0x2710, 0x833c,
	0x2712, 0x421d,
	0x2714, 0x8328,
	0x2716, 0x8f0d,
	0x2718, 0x4d82,
	0x271a, 0x8328,
	0x271c, 0x922d,
	0x271e, 0x3402,
	0x2720, 0x42a2,
	0x2722, 0x8328,
	0x2724, 0x4292,
	0x2726, 0x8328,
	0x2728, 0x7320,
	0x272a, 0x931e,
	0x272c, 0x3803,
	0x272e, 0x4e0f,
	0x2730, 0x12b0,
	0x2732, 0xfd06,
	0x2734, 0x4130,
	0x2736, 0x0b00,
	0x2738, 0x7302,
	0x273a, 0x0578,
	0x273c, 0x4392,
	0x273e, 0x7318,
	0x2740, 0x4292,
	0x2742, 0x8334,
	0x2744, 0x7574,
	0x2746, 0x4292,
	0x2748, 0x8332,
	0x274a, 0x7540,
	0x274c, 0x12b0,
	0x274e, 0xfcea,
	0x2750, 0x4130,
	0x2752, 0x120b,
	0x2754, 0x120a,
	0x2756, 0x403a,
	0x2758, 0x7314,
	0x275a, 0x403b,
	0x275c, 0x7336,
	0x275e, 0x4a2e,
	0x2760, 0x4b2f,
	0x2762, 0x12b0,
	0x2764, 0xefc4,
	0x2766, 0x4e0c,
	0x2768, 0x4f0d,
	0x276a, 0x421e,
	0x276c, 0x8342,
	0x276e, 0x4e0f,
	0x2770, 0x5f0f,
	0x2772, 0x7f0f,
	0x2774, 0xe33f,
	0x2776, 0x5e0c,
	0x2778, 0x6f0d,
	0x277a, 0x4d8b,
	0x277c, 0x0000,
	0x277e, 0x4c8a,
	0x2780, 0x0000,
	0x2782, 0x421c,
	0x2784, 0x8332,
	0x2786, 0x421d,
	0x2788, 0x8334,
	0x278a, 0x5e0c,
	0x278c, 0x6f0d,
	0x278e, 0x4d82,
	0x2790, 0x7574,
	0x2792, 0x4c82,
	0x2794, 0x7540,
	0x2796, 0x413a,
	0x2798, 0x413b,
	0x279a, 0x4130,
	0x279c, 0x421e,
	0x279e, 0x7300,
	0x27a0, 0x421f,
	0x27a2, 0x733c,
	0x27a4, 0x12b0,
	0x27a6, 0xefc4,
	0x27a8, 0x803e,
	0x27aa, 0x0003,
	0x27ac, 0x730f,
	0x27ae, 0x2ff6,
	0x27b0, 0x12b0,
	0x27b2, 0xfcea,
	0x27b4, 0x0b00,
	0x27b6, 0x7300,
	0x27b8, 0x0002,
	0x27ba, 0x0b00,
	0x27bc, 0x7302,
	0x27be, 0x0320,
	0x27c0, 0x4130,
	0x27c2, 0x4292,
	0x27c4, 0x7542,
	0x27c6, 0x8320,
	0x27c8, 0x4292,
	0x27ca, 0x7544,
	0x27cc, 0x8322,
	0x27ce, 0x4130,
	0x27fe, 0xe000,
	0x3000, 0x60f8,
	0x3002, 0x187f,
	0x3004, 0x7060,
	0x3006, 0x0114,
	0x3008, 0x60b0,
	0x300a, 0x1473,
	0x300c, 0x0013,
	0x300e, 0x140f,
	0x3010, 0x0040,
	0x3012, 0x100f,
	0x3014, 0x60f8,
	0x3016, 0x187f,
	0x3018, 0x7060,
	0x301a, 0x0114,
	0x301c, 0x60b0,
	0x301e, 0x1473,
	0x3020, 0x0013,
	0x3022, 0x140f,
	0x3024, 0x0040,
	0x3026, 0x000f,
	0x0b00, 0x0000,
	0x0b02, 0x0045,
	0x0b04, 0xb405,
	0x0b06, 0xc403,
	0x0b08, 0x0081,
	0x0b0a, 0x8252,
	0x0b0c, 0xf814,
	0x0b0e, 0xc618,
	0x0b10, 0xa828,
	0x0b12, 0x002c,
	0x0b14, 0x4068,
	0x0b16, 0x0000,
	0x0f30, 0x6e25,
	0x0f32, 0x7067,
	0x0954, 0x0009,
	0x0956, 0x0000,
	0x0958, 0xbb80,
	0x095a, 0x5140,
	0x0c00, 0x1110,
	0x0c02, 0x0011,
	0x0c04, 0x0000,
	0x0c06, 0x0200,
	0x0c10, 0x0040,
	0x0c12, 0x0040,
	0x0c14, 0x0040,
	0x0c16, 0x0040,
	0x0a10, 0x4000,
	0x3068, 0xffff,
	0x306a, 0xffff,
	0x006c, 0x0300,
	0x005e, 0x0200,
	0x000e, 0x0100,
	0x0e0a, 0x0001,
	0x004a, 0x0100,
	0x004c, 0x0000,
	0x000c, 0x0022,
	0x0008, 0x0b00,
	0x005a, 0x0202,
	0x0012, 0x000e,
	0x0018, 0x0a31,
	0x0022, 0x0008,
	0x0028, 0x0017,
	0x0024, 0x0028,
	0x002a, 0x002d,
	0x0026, 0x0030,
	0x002c, 0x07c7,
	0x002e, 0x1111,
	0x0030, 0x1111,
	0x0032, 0x1111,
	0x0006, 0x0823,
	0x0116, 0x07b6,
	0x0a22, 0x0000,
	0x0a12, 0x0a20,
	0x0a14, 0x0798,
	0x003e, 0x0000,
	0x0074, 0x080e,
	0x0070, 0x0407,
	0x0002, 0x0000,
	0x0a02, 0x0000,
	0x0a24, 0x0100,
	0x0046, 0x0000,
	0x0076, 0x0000,
	0x0060, 0x0000,
	0x0062, 0x0530,
	0x0064, 0x0500,
	0x0066, 0x0530,
	0x0068, 0x0500,
	0x0122, 0x0300,
	0x015a, 0xff08,
	0x0804, 0x0200,
	0x005c, 0x0100,
	0x0a1a, 0x0800,
	0x004c, 0x0000,
	0x004e, 0x0100, /* PFC_enable   */
	0x0040, 0x0000, /* Fsync_Normal */
	0x0042, 0x0100, /* Fsync_Normal */
	0x003e, 0x0000  /* Fsync_Normal */
};
#endif

static void sensor_init(void)
{
	LOG_INF("E\n");
#if MULTI_WRITE
	hi556sec_table_write_cmos_sensor(
		addr_data_pair_init_hi556sec,
		sizeof(addr_data_pair_init_hi556sec) /
		sizeof(kal_uint16));
#else
	/* write_cmos_sensor(0x0a00, 0x0000); */	/* stream off */
	write_cmos_sensor(0x0e00, 0x0102);
	write_cmos_sensor(0x0e02, 0x0102);
	write_cmos_sensor(0x0e0c, 0x0100);
	write_cmos_sensor(0x2000, 0x7400);
	write_cmos_sensor(0x2002, 0x001c);
	write_cmos_sensor(0x2004, 0x0242);
	write_cmos_sensor(0x2006, 0x0942);
	write_cmos_sensor(0x2008, 0x7007);
	write_cmos_sensor(0x200a, 0x0fd9);
	write_cmos_sensor(0x200c, 0x0259);
	write_cmos_sensor(0x200e, 0x7008);
	write_cmos_sensor(0x2010, 0x160e);
	write_cmos_sensor(0x2012, 0x0047);
	write_cmos_sensor(0x2014, 0x2118);
	write_cmos_sensor(0x2016, 0x0041);
	write_cmos_sensor(0x2018, 0x00d8);
	write_cmos_sensor(0x201a, 0x0145);
	write_cmos_sensor(0x201c, 0x0006);
	write_cmos_sensor(0x201e, 0x0181);
	write_cmos_sensor(0x2020, 0x13cc);
	write_cmos_sensor(0x2022, 0x2057);
	write_cmos_sensor(0x2024, 0x7001);
	write_cmos_sensor(0x2026, 0x0fca);
	write_cmos_sensor(0x2028, 0x00cb);
	write_cmos_sensor(0x202a, 0x009f);
	write_cmos_sensor(0x202c, 0x7002);
	write_cmos_sensor(0x202e, 0x13cc);
	write_cmos_sensor(0x2030, 0x019b);
	write_cmos_sensor(0x2032, 0x014d);
	write_cmos_sensor(0x2034, 0x2987);
	write_cmos_sensor(0x2036, 0x2766);
	write_cmos_sensor(0x2038, 0x0020);
	write_cmos_sensor(0x203a, 0x2060);
	write_cmos_sensor(0x203c, 0x0e5d);
	write_cmos_sensor(0x203e, 0x181d);
	write_cmos_sensor(0x2040, 0x2066);
	write_cmos_sensor(0x2042, 0x20c4);
	write_cmos_sensor(0x2044, 0x5000);
	write_cmos_sensor(0x2046, 0x0005);
	write_cmos_sensor(0x2048, 0x0000);
	write_cmos_sensor(0x204a, 0x01db);
	write_cmos_sensor(0x204c, 0x025a);
	write_cmos_sensor(0x204e, 0x00c0);
	write_cmos_sensor(0x2050, 0x0005);
	write_cmos_sensor(0x2052, 0x0006);
	write_cmos_sensor(0x2054, 0x0ad9);
	write_cmos_sensor(0x2056, 0x0259);
	write_cmos_sensor(0x2058, 0x0618);
	write_cmos_sensor(0x205a, 0x0258);
	write_cmos_sensor(0x205c, 0x2266);
	write_cmos_sensor(0x205e, 0x20c8);
	write_cmos_sensor(0x2060, 0x2060);
	write_cmos_sensor(0x2062, 0x707b);
	write_cmos_sensor(0x2064, 0x0fdd);
	write_cmos_sensor(0x2066, 0x81b8);
	write_cmos_sensor(0x2068, 0x5040);
	write_cmos_sensor(0x206a, 0x0020);
	write_cmos_sensor(0x206c, 0x5060);
	write_cmos_sensor(0x206e, 0x3143);
	write_cmos_sensor(0x2070, 0x5081);
	write_cmos_sensor(0x2072, 0x025c);
	write_cmos_sensor(0x2074, 0x7800);
	write_cmos_sensor(0x2076, 0x7400);
	write_cmos_sensor(0x2078, 0x001c);
	write_cmos_sensor(0x207a, 0x0242);
	write_cmos_sensor(0x207c, 0x0942);
	write_cmos_sensor(0x207e, 0x0bd9);
	write_cmos_sensor(0x2080, 0x0259);
	write_cmos_sensor(0x2082, 0x7008);
	write_cmos_sensor(0x2084, 0x160e);
	write_cmos_sensor(0x2086, 0x0047);
	write_cmos_sensor(0x2088, 0x2118);
	write_cmos_sensor(0x208a, 0x0041);
	write_cmos_sensor(0x208c, 0x00d8);
	write_cmos_sensor(0x208e, 0x0145);
	write_cmos_sensor(0x2090, 0x0006);
	write_cmos_sensor(0x2092, 0x0181);
	write_cmos_sensor(0x2094, 0x13cc);
	write_cmos_sensor(0x2096, 0x2057);
	write_cmos_sensor(0x2098, 0x7001);
	write_cmos_sensor(0x209a, 0x0fca);
	write_cmos_sensor(0x209c, 0x00cb);
	write_cmos_sensor(0x209e, 0x009f);
	write_cmos_sensor(0x20a0, 0x7002);
	write_cmos_sensor(0x20a2, 0x13cc);
	write_cmos_sensor(0x20a4, 0x019b);
	write_cmos_sensor(0x20a6, 0x014d);
	write_cmos_sensor(0x20a8, 0x2987);
	write_cmos_sensor(0x20aa, 0x2766);
	write_cmos_sensor(0x20ac, 0x0020);
	write_cmos_sensor(0x20ae, 0x2060);
	write_cmos_sensor(0x20b0, 0x0e5d);
	write_cmos_sensor(0x20b2, 0x181d);
	write_cmos_sensor(0x20b4, 0x2066);
	write_cmos_sensor(0x20b6, 0x20c4);
	write_cmos_sensor(0x20b8, 0x50a0);
	write_cmos_sensor(0x20ba, 0x0005);
	write_cmos_sensor(0x20bc, 0x0000);
	write_cmos_sensor(0x20be, 0x01db);
	write_cmos_sensor(0x20c0, 0x025a);
	write_cmos_sensor(0x20c2, 0x00c0);
	write_cmos_sensor(0x20c4, 0x0005);
	write_cmos_sensor(0x20c6, 0x0006);
	write_cmos_sensor(0x20c8, 0x0ad9);
	write_cmos_sensor(0x20ca, 0x0259);
	write_cmos_sensor(0x20cc, 0x0618);
	write_cmos_sensor(0x20ce, 0x0258);
	write_cmos_sensor(0x20d0, 0x2266);
	write_cmos_sensor(0x20d2, 0x20c8);
	write_cmos_sensor(0x20d4, 0x2060);
	write_cmos_sensor(0x20d6, 0x707b);
	write_cmos_sensor(0x20d8, 0x0fdd);
	write_cmos_sensor(0x20da, 0x86b8);
	write_cmos_sensor(0x20dc, 0x50e0);
	write_cmos_sensor(0x20de, 0x0020);
	write_cmos_sensor(0x20e0, 0x5100);
	write_cmos_sensor(0x20e2, 0x3143);
	write_cmos_sensor(0x20e4, 0x5121);
	write_cmos_sensor(0x20e6, 0x7800);
	write_cmos_sensor(0x20e8, 0x3140);
	write_cmos_sensor(0x20ea, 0x01c4);
	write_cmos_sensor(0x20ec, 0x01c1);
	write_cmos_sensor(0x20ee, 0x01c0);
	write_cmos_sensor(0x20f0, 0x01c4);
	write_cmos_sensor(0x20f2, 0x2700);
	write_cmos_sensor(0x20f4, 0x3d40);
	write_cmos_sensor(0x20f6, 0x7800);
	write_cmos_sensor(0x20f8, 0xffff);
	write_cmos_sensor(0x27fe, 0xe000);
	write_cmos_sensor(0x3000, 0x60f8);
	write_cmos_sensor(0x3002, 0x187f);
	write_cmos_sensor(0x3004, 0x7060);
	write_cmos_sensor(0x3006, 0x0114);
	write_cmos_sensor(0x3008, 0x60b0);
	write_cmos_sensor(0x300a, 0x1473);
	write_cmos_sensor(0x300c, 0x0013);
	write_cmos_sensor(0x300e, 0x140f);
	write_cmos_sensor(0x3010, 0x0040);
	write_cmos_sensor(0x3012, 0x100f);
	write_cmos_sensor(0x3014, 0x60f8);
	write_cmos_sensor(0x3016, 0x187f);
	write_cmos_sensor(0x3018, 0x7060);
	write_cmos_sensor(0x301a, 0x0114);
	write_cmos_sensor(0x301c, 0x60b0);
	write_cmos_sensor(0x301e, 0x1473);
	write_cmos_sensor(0x3020, 0x0013);
	write_cmos_sensor(0x3022, 0x140f);
	write_cmos_sensor(0x3024, 0x0040);
	write_cmos_sensor(0x3026, 0x000f);
	write_cmos_sensor(0x0b00, 0x0000);
	write_cmos_sensor(0x0b02, 0x0045);
	write_cmos_sensor(0x0b04, 0xb405);
	write_cmos_sensor(0x0b06, 0xc403);
	write_cmos_sensor(0x0b08, 0x0081);
	write_cmos_sensor(0x0b0a, 0x8252);
	write_cmos_sensor(0x0b0c, 0xf814);
	write_cmos_sensor(0x0b0e, 0xc618);
	write_cmos_sensor(0x0b10, 0xa828);
	write_cmos_sensor(0x0b12, 0x002c);
	write_cmos_sensor(0x0b14, 0x4068);
	write_cmos_sensor(0x0b16, 0x0000);
	write_cmos_sensor(0x0f30, 0x6e25);
	write_cmos_sensor(0x0f32, 0x7067);
	write_cmos_sensor(0x0954, 0x0009);
	write_cmos_sensor(0x0956, 0x0000);
	write_cmos_sensor(0x0958, 0xbb80);
	write_cmos_sensor(0x095a, 0x5140);
	write_cmos_sensor(0x0c00, 0x1110);
	write_cmos_sensor(0x0c02, 0x0011);
	write_cmos_sensor(0x0c04, 0x0000);
	write_cmos_sensor(0x0c06, 0x0200);
	write_cmos_sensor(0x0c10, 0x0040);
	write_cmos_sensor(0x0c12, 0x0040);
	write_cmos_sensor(0x0c14, 0x0040);
	write_cmos_sensor(0x0c16, 0x0040);
	write_cmos_sensor(0x0a10, 0x4000);

	write_cmos_sensor(0x0c08, 0x01c0);	/* 9_21 added */
	write_cmos_sensor(0x0c0a, 0x01c0);
	write_cmos_sensor(0x0c0c, 0x01c0);
	write_cmos_sensor(0x0c0e, 0x01c0);

	write_cmos_sensor(0x3068, 0xf800);
	write_cmos_sensor(0x306a, 0xf876);
	write_cmos_sensor(0x006c, 0x0000);
	write_cmos_sensor(0x005e, 0x0200);
	write_cmos_sensor(0x000e, 0x0100);
	write_cmos_sensor(0x0e0a, 0x0001);
	write_cmos_sensor(0x004a, 0x0100);
	write_cmos_sensor(0x004c, 0x0000);
	write_cmos_sensor(0x004e, 0x0100);
	write_cmos_sensor(0x000c, 0x0022);
	write_cmos_sensor(0x0008, 0x0b00);
	write_cmos_sensor(0x005a, 0x0202);
	write_cmos_sensor(0x0012, 0x000e);
	write_cmos_sensor(0x0018, 0x0a31);
	write_cmos_sensor(0x0022, 0x0008);
	write_cmos_sensor(0x0028, 0x0017);
	write_cmos_sensor(0x0024, 0x0028);
	write_cmos_sensor(0x002a, 0x002d);
	write_cmos_sensor(0x0026, 0x0030);
	write_cmos_sensor(0x002c, 0x07c7);
	write_cmos_sensor(0x002e, 0x1111);
	write_cmos_sensor(0x0030, 0x1111);
	write_cmos_sensor(0x0032, 0x1111);
	write_cmos_sensor(0x0006, 0x07bc);
	write_cmos_sensor(0x0a22, 0x0000);
	write_cmos_sensor(0x0a12, 0x0a20);
	write_cmos_sensor(0x0a14, 0x0798);
	write_cmos_sensor(0x003e, 0x0000);
	write_cmos_sensor(0x0074, 0x080e);
	write_cmos_sensor(0x0070, 0x0407);
	write_cmos_sensor(0x0002, 0x0000);
	write_cmos_sensor(0x0a02, 0x0100);
	write_cmos_sensor(0x0a24, 0x0100);
	write_cmos_sensor(0x0046, 0x0000);
	write_cmos_sensor(0x0076, 0x0000);
	write_cmos_sensor(0x0060, 0x0000);
	write_cmos_sensor(0x0062, 0x0530);
	write_cmos_sensor(0x0064, 0x0500);
	write_cmos_sensor(0x0066, 0x0530);
	write_cmos_sensor(0x0068, 0x0500);
	write_cmos_sensor(0x0122, 0x0300);
	write_cmos_sensor(0x015a, 0xff08);
	write_cmos_sensor(0x0804, 0x0200);
	write_cmos_sensor(0x005c, 0x0182);
	write_cmos_sensor(0x0a1a, 0x0800);
#endif
}

#if MULTI_WRITE
kal_uint16 addr_data_pair_preview_hi556sec[] = {
	/* 0x0a00, 0x0000, */	/* stream off */
	0x0b0a, 0x8252,
	0x0f30, 0x6e25,
	0x0f32, 0x7067,
	0x004a, 0x0100,
	0x004c, 0x0000,
	0x000c, 0x0022,
	0x0008, 0x0b00,
	0x005a, 0x0202,
	0x0012, 0x000e,
	0x0018, 0x0a31,
	0x0022, 0x0008,
	0x0028, 0x0017,
	0x0024, 0x0028,
	0x002a, 0x002d,
	0x0026, 0x0030,
	0x002c, 0x07c7,
	0x002e, 0x1111,
	0x0030, 0x1111,
	0x0032, 0x1111,
	0x0006, 0x0823,
	0x0116, 0x07b6,
	0x0a22, 0x0000,
	0x0a12, 0x0a20,
	0x0a14, 0x0798,
	0x003e, 0x0000,
	0x0074, 0x0821,
	0x0070, 0x0411,
	0x0804, 0x0200,
	0x0a04, 0x014a,
	0x090c, 0x0fdc,
	0x090e, 0x002d,
	0x0040, 0x0000,
	0x0042, 0x0100,
	0x003e, 0x0000,
	0x0126, 0x00f9,
	0x0902, 0x4319,
	0x0914, 0xc10a,
	0x0916, 0x071f,
	0x0918, 0x0408,
	0x091a, 0x0c0d,
	0x091c, 0x0f09,
	0x091e, 0x0a00,
	0x0958, 0xbb80,
};
#endif

static void preview_setting(void)
{
#if MULTI_WRITE
	hi556sec_table_write_cmos_sensor(
		addr_data_pair_preview_hi556sec,
		sizeof(addr_data_pair_preview_hi556sec) /
		sizeof(kal_uint16));
#else
	/* write_cmos_sensor(0x0a00, 0x0000); */	/* stream off */
	write_cmos_sensor(0x0b0a, 0x8252);
	write_cmos_sensor(0x0f30, 0x6e25);
	write_cmos_sensor(0x0f32, 0x7067);
	write_cmos_sensor(0x004a, 0x0100);
	write_cmos_sensor(0x004c, 0x0000);
	write_cmos_sensor(0x004e, 0x0100);	/* perframe enable */
	write_cmos_sensor(0x000c, 0x0022);
	write_cmos_sensor(0x0008, 0x0b00);
	write_cmos_sensor(0x005a, 0x0202);
	write_cmos_sensor(0x0012, 0x000e);
	write_cmos_sensor(0x0018, 0x0a31);
	write_cmos_sensor(0x0022, 0x0008);
	write_cmos_sensor(0x0028, 0x0017);
	write_cmos_sensor(0x0024, 0x0028);
	write_cmos_sensor(0x002a, 0x002d);
	write_cmos_sensor(0x0026, 0x0030);
	write_cmos_sensor(0x002c, 0x07c7);
	write_cmos_sensor(0x002e, 0x1111);
	write_cmos_sensor(0x0030, 0x1111);
	write_cmos_sensor(0x0032, 0x1111);
	/* write_cmos_sensor(0x0006, 0x0801); */
	write_cmos_sensor(0x0006, 0x0823);
	write_cmos_sensor(0x0a22, 0x0000);
	write_cmos_sensor(0x0a12, 0x0a20);
	write_cmos_sensor(0x0a14, 0x0798);
	write_cmos_sensor(0x003e, 0x0000);
	/* write_cmos_sensor(0x0074, 0x07ff); */
	/* write_cmos_sensor(0x0070, 0x0411); */
	write_cmos_sensor(0x0804, 0x0200);
	write_cmos_sensor(0x0a04, 0x014a);
	write_cmos_sensor(0x090c, 0x0fdc);
	write_cmos_sensor(0x090e, 0x002d);
	/* =============================================== *
	 *             mipi 2 lane 880Mbps                 *
	 * =============================================== */
	write_cmos_sensor(0x0902, 0x4319);
	write_cmos_sensor(0x0914, 0xc10a);
	write_cmos_sensor(0x0916, 0x071f);
	write_cmos_sensor(0x0918, 0x0408);
	write_cmos_sensor(0x091a, 0x0c0d);
	write_cmos_sensor(0x091c, 0x0f09);
	write_cmos_sensor(0x091e, 0x0a00);
#endif
}

#if MULTI_WRITE
kal_uint16 addr_data_pair_capture_fps_hi556sec[] = {
	/* 0x0a00, 0x0000, */	/* stream off */
	0x0b0a, 0x8252,
	0x0f30, 0x6e25,
	0x0f32, 0x7067,
	0x004a, 0x0100,
	0x004c, 0x0000,
	0x000c, 0x0022,
	0x0008, 0x0b00,
	0x005a, 0x0202,
	0x0012, 0x000e,
	0x0018, 0x0a31,
	0x0022, 0x0008,
	0x0028, 0x0017,
	0x0024, 0x0028,
	0x002a, 0x002d,
	0x0026, 0x0030,
	0x002c, 0x07c7,
	0x002e, 0x1111,
	0x0030, 0x1111,
	0x0032, 0x1111,
	0x0006, 0x1046,
	0x0116, 0x07b6,
	0x0a22, 0x0000,
	0x0a12, 0x0a20,
	0x0a14, 0x0798,
	0x003e, 0x0000,
	0x0074, 0x1044,
	0x0070, 0x0822,
	0x0804, 0x0200,
	0x0a04, 0x014a,
	0x090c, 0x0fdc,
	/* 0x090e, 0x002d, */
	0x0040, 0x0000,
	0x0042, 0x0100,
	0x003e, 0x0000,
	0x0126, 0x00f9,
	0x0902, 0x4319,
	0x0914, 0xc10a,
	0x0916, 0x071f,
	0x0918, 0x0408,
	0x091a, 0x0c0d,
	0x091c, 0x0f09,
	0x091e, 0x0a00,
	0x0958, 0xbb80,
};

kal_uint16 addr_data_pair_capture_30fps_hi556sec[] = {
	/* 0x0a00, 0x0000, */	/* stream off */
	0x0b0a, 0x8252,
	0x0f30, 0x6e25,
	0x0f32, 0x7067,
	0x004a, 0x0100,
	0x004c, 0x0000,
	0x000c, 0x0022,
	0x0008, 0x0b00,
	0x005a, 0x0202,
	0x0012, 0x000e,
	0x0018, 0x0a31,
	0x0022, 0x0008,
	0x0028, 0x0017,
	0x0024, 0x0028,
	0x002a, 0x002d,
	0x0026, 0x0030,
	0x002c, 0x07c7,
	0x002e, 0x1111,
	0x0030, 0x1111,
	0x0032, 0x1111,
	0x0006, 0x0823,
	0x0116, 0x07b6,
	0x0a22, 0x0000,
	0x0a12, 0x0a20,
	0x0a14, 0x0798,
	0x003e, 0x0000,
	0x0074, 0x0821,
	0x0070, 0x0411,
	0x0804, 0x0200,
	0x0a04, 0x014a,
	0x090c, 0x0fdc,
	0x090e, 0x002d,
	0x0040, 0x0000,
	0x0042, 0x0100,
	0x003e, 0x0000,
	0x0126, 0x00f9,
	0x0902, 0x4319,
	0x0914, 0xc10a,
	0x0916, 0x071f,
	0x0918, 0x0408,
	0x091a, 0x0c0d,
	0x091c, 0x0f09,
	0x091e, 0x0a00,
	0x0958, 0xbb80,
};
#endif

static void capture_setting(kal_uint16 currefps)
{
#if MULTI_WRITE
	if (currefps == 300) {
	hi556sec_table_write_cmos_sensor(
		addr_data_pair_capture_30fps_hi556sec,
		sizeof(addr_data_pair_capture_30fps_hi556sec) /
		sizeof(kal_uint16));
	} else {
	hi556sec_table_write_cmos_sensor(
		addr_data_pair_capture_fps_hi556sec,
		sizeof(addr_data_pair_capture_fps_hi556sec) /
		sizeof(kal_uint16));
	}
#else
	if (currefps == 300) {
		LOG_DBG("capture_setting fps = 300\n");
		write_cmos_sensor(0x0b0a, 0x8252);
		write_cmos_sensor(0x0f30, 0x6e25);
		write_cmos_sensor(0x0f32, 0x7067);
		write_cmos_sensor(0x004a, 0x0100);
		write_cmos_sensor(0x004c, 0x0000);
		write_cmos_sensor(0x004e, 0x0100);	/* perframe enable */
		write_cmos_sensor(0x000c, 0x0022);
		write_cmos_sensor(0x0008, 0x0b00);
		write_cmos_sensor(0x005a, 0x0202);
		write_cmos_sensor(0x0012, 0x000e);
		write_cmos_sensor(0x0018, 0x0a31);
		write_cmos_sensor(0x0022, 0x0008);
		write_cmos_sensor(0x0028, 0x0017);
		write_cmos_sensor(0x0024, 0x0028);
		write_cmos_sensor(0x002a, 0x002d);
		write_cmos_sensor(0x0026, 0x0030);
		write_cmos_sensor(0x002c, 0x07c7);
		write_cmos_sensor(0x002e, 0x1111);
		write_cmos_sensor(0x0030, 0x1111);
		write_cmos_sensor(0x0032, 0x1111);
		write_cmos_sensor(0x0006, 0x0823);
		write_cmos_sensor(0x0a22, 0x0000);
		write_cmos_sensor(0x0a12, 0x0a20);
		write_cmos_sensor(0x0a14, 0x0798);
		write_cmos_sensor(0x003e, 0x0000);
		/* write_cmos_sensor(0x0074, 0x07ff); */
		/* write_cmos_sensor(0x0070, 0x0411); */
		write_cmos_sensor(0x0804, 0x0200);
		write_cmos_sensor(0x0a04, 0x014a);
		write_cmos_sensor(0x090c, 0x0fdc);
		write_cmos_sensor(0x090e, 0x002d);
		/* =============================================== *
		 *             mipi 2 lane 880Mbps                 *
		 * =============================================== */
		write_cmos_sensor(0x0902, 0x4319);
		write_cmos_sensor(0x0914, 0xc10a);
		write_cmos_sensor(0x0916, 0x071f);
		write_cmos_sensor(0x0918, 0x0408);
		write_cmos_sensor(0x091a, 0x0c0d);
		write_cmos_sensor(0x091c, 0x0f09);
		write_cmos_sensor(0x091e, 0x0a00);
	} else	{
		LOG_DBG("capture_setting fps not 300\n");
		/* Sensor Information                 *
		 * Sensor	  : Hi-556            *
		 * Date		  : 2016-10-19        *
		 * Customer        : MTK_validation   *
		 * Image size	  : 2592x1944         *
		 * MCLK		  : 24MHz             *
		 * MIPI speed(Mbps): 880Mbps x 2Lane  *
		 * Frame Length	  : 4166              *
		 * Line Length	  : 2816              *
		 * Max Fps	  : 15.0fps           *
		 * Pixel order	  : Green 1st (=GB)   *
		 * X/Y-flip	  : X-flip            *
		 * BLC offset	  : 64code            */

		write_cmos_sensor(0x0b0a, 0x8252);
		write_cmos_sensor(0x0f30, 0x6e25);
		write_cmos_sensor(0x0f32, 0x7067);
		write_cmos_sensor(0x004a, 0x0100);
		write_cmos_sensor(0x004c, 0x0000);
		write_cmos_sensor(0x004e, 0x0100); /* perframe enable */
		write_cmos_sensor(0x000c, 0x0022);
		write_cmos_sensor(0x0008, 0x0b00);
		write_cmos_sensor(0x005a, 0x0202);
		write_cmos_sensor(0x0012, 0x000e);
		write_cmos_sensor(0x0018, 0x0a31);
		write_cmos_sensor(0x0022, 0x0008);
		write_cmos_sensor(0x0028, 0x0017);
		write_cmos_sensor(0x0024, 0x0028);
		write_cmos_sensor(0x002a, 0x002d);
		write_cmos_sensor(0x0026, 0x0030);
		write_cmos_sensor(0x002c, 0x07c7);
		write_cmos_sensor(0x002e, 0x1111);
		write_cmos_sensor(0x0030, 0x1111);
		write_cmos_sensor(0x0032, 0x1111);
		write_cmos_sensor(0x0006, 0x1046);
		write_cmos_sensor(0x0a22, 0x0000);
		write_cmos_sensor(0x0a12, 0x0a20);
		write_cmos_sensor(0x0a14, 0x0798);
		write_cmos_sensor(0x003e, 0x0000);
		write_cmos_sensor(0x0074, 0x1044);
		write_cmos_sensor(0x0070, 0x0822);
		write_cmos_sensor(0x0804, 0x0200);
		write_cmos_sensor(0x0a04, 0x014a);
		write_cmos_sensor(0x090c, 0x0fdc);
		/* write_cmos_sensor(0x090e, 0x002d); */
		/* =============================================== *
		 *             mipi 2 lane 880Mbps                 *
		 * =============================================== */
		write_cmos_sensor(0x0902, 0x4319);
		write_cmos_sensor(0x0914, 0xc10a);
		write_cmos_sensor(0x0916, 0x071f);
		write_cmos_sensor(0x0918, 0x0408);
		write_cmos_sensor(0x091a, 0x0c0d);
		write_cmos_sensor(0x091c, 0x0f09);
		write_cmos_sensor(0x091e, 0x0a00);
	}
#endif
}

#if MULTI_WRITE
kal_uint16 addr_data_pair_normal_video_hi556sec[] = {
	/* 0x0a00, 0x0000, */	/* stream off */
	0x0b0a, 0x8252,
	0x0f30, 0x6e25,
	0x0f32, 0x7067,
	0x004a, 0x0100,
	0x004c, 0x0000,
	0x000c, 0x0022,
	0x0008, 0x0b00,
	0x005a, 0x0202,
	0x0012, 0x000e,
	0x0018, 0x0a31,
	0x0022, 0x0008,
	0x0028, 0x0017,
	0x0024, 0x0028,
	0x002a, 0x002d,
	0x0026, 0x0030,
	0x002c, 0x07c7,
	0x002e, 0x1111,
	0x0030, 0x1111,
	0x0032, 0x1111,
	0x0006, 0x0823,
	0x0116, 0x07b6,
	0x0a22, 0x0000,
	0x0a12, 0x0a20,
	0x0a14, 0x0798,
	0x003e, 0x0000,
	0x0074, 0x0821,
	0x0070, 0x0411,
	0x0804, 0x0200,
	0x0a04, 0x014a,
	0x090c, 0x0fdc,
	0x090e, 0x002d,
	0x0040, 0x0000,
	0x0042, 0x0100,
	0x003e, 0x0000,
	0x0126, 0x00f9,
	0x0902, 0x4319,
	0x0914, 0xc10a,
	0x0916, 0x071f,
	0x0918, 0x0408,
	0x091a, 0x0c0d,
	0x091c, 0x0f09,
	0x091e, 0x0a00,
	0x0958, 0xbb80,
};
#endif

static void normal_video_setting(void)
{
#if MULTI_WRITE
	hi556sec_table_write_cmos_sensor(
			addr_data_pair_normal_video_hi556sec,
			sizeof(addr_data_pair_normal_video_hi556sec) /
			sizeof(kal_uint16));
#else
	/* write_cmos_sensor(0x0a00, 0x0000); */	/* stream off */
	write_cmos_sensor(0x0b0a, 0x8252);
	write_cmos_sensor(0x0f30, 0x6e25);
	write_cmos_sensor(0x0f32, 0x7067);
	write_cmos_sensor(0x004a, 0x0100);
	write_cmos_sensor(0x004c, 0x0000);
	write_cmos_sensor(0x004e, 0x0100);	/* perframe enable */
	write_cmos_sensor(0x000c, 0x0022);
	write_cmos_sensor(0x0008, 0x0b00);
	write_cmos_sensor(0x005a, 0x0202);
	write_cmos_sensor(0x0012, 0x000e);
	write_cmos_sensor(0x0018, 0x0a31);
	write_cmos_sensor(0x0022, 0x0008);
	write_cmos_sensor(0x0028, 0x0017);
	write_cmos_sensor(0x0024, 0x0028);
	write_cmos_sensor(0x002a, 0x002d);
	write_cmos_sensor(0x0026, 0x0030);
	write_cmos_sensor(0x002c, 0x07c7);
	write_cmos_sensor(0x002e, 0x1111);
	write_cmos_sensor(0x0030, 0x1111);
	write_cmos_sensor(0x0032, 0x1111);
	/* write_cmos_sensor(0x0006, 0x0801); */
	write_cmos_sensor(0x0006, 0x0823);
	write_cmos_sensor(0x0a22, 0x0000);
	write_cmos_sensor(0x0a12, 0x0a20);
	write_cmos_sensor(0x0a14, 0x0798);
	write_cmos_sensor(0x003e, 0x0000);
	/* write_cmos_sensor(0x0074, 0x07ff); */
	/* write_cmos_sensor(0x0070, 0x0411); */
	write_cmos_sensor(0x0804, 0x0200);
	write_cmos_sensor(0x0a04, 0x014a);
	write_cmos_sensor(0x090c, 0x0fdc);
	write_cmos_sensor(0x090e, 0x002d);
	/* =============================================== *
	 *             mipi 2 lane 880Mbps                 *
	 * =============================================== */
	write_cmos_sensor(0x0902, 0x4319);
	write_cmos_sensor(0x0914, 0xc10a);
	write_cmos_sensor(0x0916, 0x071f);
	write_cmos_sensor(0x0918, 0x0408);
	write_cmos_sensor(0x091a, 0x0c0d);
	write_cmos_sensor(0x091c, 0x0f09);
	write_cmos_sensor(0x091e, 0x0a00);
#endif
}

#if MULTI_WRITE
kal_uint16 addr_data_pair_hs_video_hi556sec[] = {
	0x0b0a, 0x8252,
	0x0f30, 0x6e25,
	0x0f32, 0x7267,
	0x004a, 0x0100,
	0x004c, 0x0000,
	0x004e, 0x0100,	/* perframe enable */
	0x000c, 0x0022,
	0x0008, 0x0b00,
	0x005a, 0x0208,
	0x0012, 0x0018,
	0x0018, 0x0a27,
	0x0022, 0x0008,
	0x0028, 0x0017,
	0x0024, 0x002e,
	0x002a, 0x0033,
	0x0026, 0x003c,
	0x002c, 0x07bb,
	0x002e, 0x1111,
	0x0030, 0x1111,
	0x0032, 0x7711,
	0x0006, 0x021F,	/* changed for 1ms Vblank */
	0x0a22, 0x0100,
	0x0a12, 0x0280,
	0x0a14, 0x01e0,
	0x003e, 0x0000,
	0x0074, 0x0206,
	0x0070, 0x0103,
	0x0804, 0x0200,
	0x0a04, 0x016a,
	0x090c, 0x0270,
	0x090e, 0x000c,
	0x0902, 0x4319,
	0x0914, 0xc103,
	0x0916, 0x0207,
	0x0918, 0x0302,
	0x091a, 0x0406,
	0x091c, 0x0903,
	0x091e, 0x0300
};
#endif

static void hs_video_setting(void)
{
	/* Sensor Information                *
	 * Sensor	  : hi-556           *
	 * Date		  : 2016-10-19       *
	 * Customer        : MTK_validation  *
	 * Image size	  : 640x480          *
	 * MCLK		  : 24MHz            *
	 * MIPI speed(Mbps): 220Mbps x 2Lane *
	 * Frame Length	  : 520              *
	 * Line Length	  : 2816             *
	 * Max Fps	  : 120.19fps        *
	 * Pixel order	  : Green 1st (=GB)  *
	 * X/Y-flip	  : X-flip           *
	 * BLC offset	  : 64code           */

#if MULTI_WRITE
	hi556sec_table_write_cmos_sensor(
		addr_data_pair_hs_video_hi556sec,
		sizeof(addr_data_pair_hs_video_hi556sec) /
		sizeof(kal_uint16));
#else
	write_cmos_sensor(0x0b0a, 0x8252);
	write_cmos_sensor(0x0f30, 0x6e25);
	write_cmos_sensor(0x0f32, 0x7267);
	write_cmos_sensor(0x004a, 0x0100);
	write_cmos_sensor(0x004c, 0x0000);
	write_cmos_sensor(0x004e, 0x0100);	/* perframe enable */
	write_cmos_sensor(0x000c, 0x0022);
	write_cmos_sensor(0x0008, 0x0b00);
	write_cmos_sensor(0x005a, 0x0208);
	write_cmos_sensor(0x0012, 0x0018);
	write_cmos_sensor(0x0018, 0x0a27);
	write_cmos_sensor(0x0022, 0x0008);
	write_cmos_sensor(0x0028, 0x0017);
	write_cmos_sensor(0x0024, 0x002e);
	write_cmos_sensor(0x002a, 0x0033);
	write_cmos_sensor(0x0026, 0x003c);
	write_cmos_sensor(0x002c, 0x07bb);
	write_cmos_sensor(0x002e, 0x1111);
	write_cmos_sensor(0x0030, 0x1111);
	write_cmos_sensor(0x0032, 0x7711);
	write_cmos_sensor(0x0006, 0x021F);	/* changed for 1ms Vblank */
	write_cmos_sensor(0x0a22, 0x0100);
	write_cmos_sensor(0x0a12, 0x0280);
	write_cmos_sensor(0x0a14, 0x01e0);
	write_cmos_sensor(0x003e, 0x0000);
	write_cmos_sensor(0x0074, 0x0206);
	write_cmos_sensor(0x0070, 0x0103);
	write_cmos_sensor(0x0804, 0x0200);
	write_cmos_sensor(0x0a04, 0x016a);
	write_cmos_sensor(0x090c, 0x0270);
	write_cmos_sensor(0x090e, 0x000c);
	/* =============================================== *
	 *             mipi 2 lane 220Mbps                 *
	 * =============================================== */
	write_cmos_sensor(0x0902, 0x4319);
	write_cmos_sensor(0x0914, 0xc103);
	write_cmos_sensor(0x0916, 0x0207);
	write_cmos_sensor(0x0918, 0x0302);
	write_cmos_sensor(0x091a, 0x0406);
	write_cmos_sensor(0x091c, 0x0903);
	write_cmos_sensor(0x091e, 0x0300);
#endif
}

#if MULTI_WRITE
kal_uint16 addr_data_pair_slim_video_hi556sec[] = {
	0x0b0a, 0x8252,
	0x0f30, 0x6e25,
	0x0f32, 0x7167,
	0x004a, 0x0100,
	0x004c, 0x0000,
	0x004e, 0x0100,	/* perframe enable */
	0x000c, 0x0022,
	0x0008, 0x0b00,
	0x005a, 0x0204,
	0x0012, 0x001c,
	0x0018, 0x0a23,
	0x0022, 0x0008,
	0x0028, 0x0017,
	0x0024, 0x0122,
	0x002a, 0x0127,
	0x0026, 0x012c,
	0x002c, 0x06cb,
	0x002e, 0x1111,
	0x0030, 0x1111,
	0x0032, 0x3311,
	0x0006, 0x0823,
	0x0a22, 0x0000,
	0x0a12, 0x0500,
	0x0a14, 0x02d0,
	0x003e, 0x0000,
	0x0074, 0x0821,
	0x0070, 0x0411,
	0x0804, 0x0200,
	0x0a04, 0x016a,
	0x090e, 0x0010,
	0x090c, 0x09c0,
	0x0902, 0x4319,
	0x0914, 0xc106,
	0x0916, 0x040e,
	0x0918, 0x0304,
	0x091c, 0x0e06,
	0x091a, 0x0708,
	0x091e, 0x0300
};
#endif

static void slim_video_setting(void)
{
	/* Sensor Information                 *
	 * Sensor	  : hi-556            *
	 * Date		  : 2016-10-19        *
	 * Customer        : MTK_validation   *
	 * Image size	  : 1280x720          *
	 * MCLK		  : 24MHz             *
	 * MIPI speed(Mbps): 440Mbps x 2Lane  *
	 * Frame Length	  : 2083              *
	 * Line Length	  : 2816              *
	 * Max Fps	  : 30.0fps           *
	 * Pixel order	  : Green 1st (=GB)   *
	 * X/Y-flip	  : X-flip            *
	 * BLC offset	  : 64code            */

#if MULTI_WRITE
	hi556sec_table_write_cmos_sensor(
		addr_data_pair_slim_video_hi556sec,
		sizeof(addr_data_pair_slim_video_hi556sec) /
		sizeof(kal_uint16));
#else
	write_cmos_sensor(0x0b0a, 0x8252);
	write_cmos_sensor(0x0f30, 0x6e25);
	write_cmos_sensor(0x0f32, 0x7167);
	write_cmos_sensor(0x004a, 0x0100);
	write_cmos_sensor(0x004c, 0x0000);
	write_cmos_sensor(0x004e, 0x0100);	/* perframe enable */
	write_cmos_sensor(0x000c, 0x0022);
	write_cmos_sensor(0x0008, 0x0b00);
	write_cmos_sensor(0x005a, 0x0204);
	write_cmos_sensor(0x0012, 0x001c);
	write_cmos_sensor(0x0018, 0x0a23);
	write_cmos_sensor(0x0022, 0x0008);
	write_cmos_sensor(0x0028, 0x0017);
	write_cmos_sensor(0x0024, 0x0122);
	write_cmos_sensor(0x002a, 0x0127);
	write_cmos_sensor(0x0026, 0x012c);
	write_cmos_sensor(0x002c, 0x06cb);
	write_cmos_sensor(0x002e, 0x1111);
	write_cmos_sensor(0x0030, 0x1111);
	write_cmos_sensor(0x0032, 0x3311);
	write_cmos_sensor(0x0006, 0x0823);
	write_cmos_sensor(0x0a22, 0x0000);
	write_cmos_sensor(0x0a12, 0x0500);
	write_cmos_sensor(0x0a14, 0x02d0);
	write_cmos_sensor(0x003e, 0x0000);
	write_cmos_sensor(0x0074, 0x0821);
	write_cmos_sensor(0x0070, 0x0411);
	write_cmos_sensor(0x0804, 0x0200);
	write_cmos_sensor(0x0a04, 0x016a);
	write_cmos_sensor(0x090e, 0x0010);
	write_cmos_sensor(0x090c, 0x09c0);
	/* =============================================== *
	 *             mipi 2 lane 440Mbps                 *
	 * =============================================== */
	write_cmos_sensor(0x0902, 0x4319);
	write_cmos_sensor(0x0914, 0xc106);
	write_cmos_sensor(0x0916, 0x040e);
	write_cmos_sensor(0x0918, 0x0304);
	write_cmos_sensor(0x091c, 0x0e06);
	write_cmos_sensor(0x091a, 0x0708);
	write_cmos_sensor(0x091e, 0x0300);
#endif
}

static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;

	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			*sensor_id = return_sensor_id() + 1;
			if (*sensor_id == imgsensor_info.sensor_id) {
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id, *sensor_id);
				return ERROR_NONE;
			}
			LOG_INF("Read sensor id fail, write id: 0x%x, id: 0x%x\n", imgsensor.i2c_write_id, *sensor_id);
			retry--;
		} while (retry > 0);
		i++;
		retry = 2;
	}

	if (*sensor_id != imgsensor_info.sensor_id) {
		/* if Sensor ID is not correct, Must set *sensor_id to 0xFFFFFFFF */
		*sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}

	return ERROR_NONE;
}

/*************************************************************************
 * FUNCTION
 *	open
 *
 * DESCRIPTION
 *	This function initialize the registers of CMOS sensor
 *
 * PARAMETERS
 *	None
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 open(void)
{
	/* kal_uint8 i = 0; */
	/* kal_uint8 retry = 2; */
	kal_uint32 sensor_id = 0;

	LOG_INF("[open]: PLATFORM:MT8183,MIPI 2LANE\n");
	LOG_DBG("preview 1296*972@30fps,360Mbps/lane;"
		"capture 2592*1944@30fps,880Mbps/lane\n");

	get_imgsensor_id(&sensor_id);
#if 0
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			sensor_id = return_sensor_id();
			if (sensor_id == imgsensor_info.sensor_id) {
				LOG_DBG("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id, sensor_id);
				break;
			}
			LOG_DBG("Read sensor id fail, write id: 0x%x, id: 0x%x\n", imgsensor.i2c_write_id, sensor_id);
			retry--;
		} while (retry > 0);
		i++;
		if (sensor_id == imgsensor_info.sensor_id)
			break;
	    retry = 2;
	}
#endif
	if (imgsensor_info.sensor_id != sensor_id)
		return ERROR_SENSOR_CONNECT_FAIL;

	/* initail sequence write in  */
	sensor_init();

	spin_lock(&imgsensor_drv_lock);
	imgsensor.autoflicker_en = KAL_FALSE;
	imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.dummy_pixel = 0;
	imgsensor.dummy_line = 0;
	imgsensor.ihdr_en = 0;
	imgsensor.test_pattern = KAL_FALSE;
	imgsensor.current_fps = imgsensor_info.pre.max_framerate;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}	/* open */

static kal_uint32 close(void)
{
	LOG_INF("E\n");
	return ERROR_NONE;
}	/* close */

/*************************************************************************
 * FUNCTION
 * preview
 *
 * DESCRIPTION
 *	This function start the sensor preview.
 *
 * PARAMETERS
 *	*image_window : address pointer of pixel numbers in one period of HSYNC
 *  *sensor_config_data : address pointer of line numbers in one period of VSYNC
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_DBG("E");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.current_fps = imgsensor_info.pre.max_framerate;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	preview_setting();
	return ERROR_NONE;
}	/* preview */

/*************************************************************************
 * FUNCTION
 *	capture
 *
 * DESCRIPTION
 *	This function setup the CMOS sensor in capture MY_OUTPUT mode
 *
 * PARAMETERS
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;

	if (imgsensor.current_fps == imgsensor_info.cap.max_framerate)	{
		imgsensor.pclk = imgsensor_info.cap.pclk;
		imgsensor.line_length = imgsensor_info.cap.linelength;
		imgsensor.frame_length = imgsensor_info.cap.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	} else {
		/* PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 20M */
		imgsensor.pclk = imgsensor_info.cap1.pclk;
		imgsensor.line_length = imgsensor_info.cap1.linelength;
		imgsensor.frame_length = imgsensor_info.cap1.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	}

	spin_unlock(&imgsensor_drv_lock);
	LOG_DBG("Capture fps:%d\n", imgsensor.current_fps);
	capture_setting(imgsensor.current_fps);

	return ERROR_NONE;

}	/* capture() */

static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
	imgsensor.pclk = imgsensor_info.normal_video.pclk;
	imgsensor.line_length = imgsensor_info.normal_video.linelength;
	imgsensor.frame_length = imgsensor_info.normal_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
	imgsensor.current_fps = 300;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	normal_video_setting();
	return ERROR_NONE;
}	/* normal_video */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	imgsensor.pclk = imgsensor_info.hs_video.pclk;
	/* imgsensor.video_mode = KAL_TRUE; */
	imgsensor.line_length = imgsensor_info.hs_video.linelength;
	imgsensor.frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	hs_video_setting();

	return ERROR_NONE;
}    /* hs_video */

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
		      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
	imgsensor.pclk = imgsensor_info.slim_video.pclk;
	imgsensor.line_length = imgsensor_info.slim_video.linelength;
	imgsensor.frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	slim_video_setting();

	return ERROR_NONE;
}    /* slim_video */

static kal_uint32 get_resolution(
		MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
	sensor_resolution->SensorFullWidth =
		imgsensor_info.cap.grabwindow_width;
	sensor_resolution->SensorFullHeight =
		imgsensor_info.cap.grabwindow_height;

	sensor_resolution->SensorPreviewWidth =
		imgsensor_info.pre.grabwindow_width;
	sensor_resolution->SensorPreviewHeight =
		imgsensor_info.pre.grabwindow_height;

	sensor_resolution->SensorVideoWidth =
		imgsensor_info.normal_video.grabwindow_width;
	sensor_resolution->SensorVideoHeight =
		imgsensor_info.normal_video.grabwindow_height;

	sensor_resolution->SensorHighSpeedVideoWidth =
		imgsensor_info.hs_video.grabwindow_width;
	sensor_resolution->SensorHighSpeedVideoHeight =
		imgsensor_info.hs_video.grabwindow_height;

	sensor_resolution->SensorSlimVideoWidth =
		imgsensor_info.slim_video.grabwindow_width;
	sensor_resolution->SensorSlimVideoHeight =
		imgsensor_info.slim_video.grabwindow_height;
	return ERROR_NONE;
}    /* get_resolution */

static kal_uint32 get_info(MSDK_SCENARIO_ID_ENUM scenario_id,
			MSDK_SENSOR_INFO_STRUCT *sensor_info,
			MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_DBG("scenario_id = %d\n", scenario_id);

	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorInterruptDelayLines = 4; /* not use */
	sensor_info->SensorResetActiveHigh = FALSE; /* not use */
	sensor_info->SensorResetDelayCount = 5; /* not use */

	sensor_info->SensroInterfaceType =
	imgsensor_info.sensor_interface_type;
	sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
	sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;
	sensor_info->SensorOutputDataFormat =
		imgsensor_info.sensor_output_dataformat;

	sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame;
	sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame;
	sensor_info->VideoDelayFrame =
		imgsensor_info.video_delay_frame;
	sensor_info->HighSpeedVideoDelayFrame =
		imgsensor_info.hs_video_delay_frame;
	sensor_info->SlimVideoDelayFrame =
		imgsensor_info.slim_video_delay_frame;

	sensor_info->SensorMasterClockSwitch = 0; /* not use */
	sensor_info->SensorDrivingCurrent =
		imgsensor_info.isp_driving_current;
	/* The frame of setting shutter default 0 for TG int */
	sensor_info->AEShutDelayFrame =
		imgsensor_info.ae_shut_delay_frame;
	/* The frame of setting sensor gain */
	sensor_info->AESensorGainDelayFrame =
		imgsensor_info.ae_sensor_gain_delay_frame;
	sensor_info->AEISPGainDelayFrame =
		imgsensor_info.ae_ispGain_delay_frame;
	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine =
		imgsensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum =
		imgsensor_info.sensor_mode_num;

	sensor_info->SensorMIPILaneNumber =
		imgsensor_info.mipi_lane_num;
	sensor_info->SensorClockFreq = imgsensor_info.mclk;
	sensor_info->SensorClockDividCount = 3; /* not use */
	sensor_info->SensorClockRisingCount = 0;
	sensor_info->SensorClockFallingCount = 2; /* not use */
	sensor_info->SensorPixelClockCount = 3; /* not use */
	sensor_info->SensorDataLatchCount = 2; /* not use */

	sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->SensorWidthSampling = 0;    /* 0 is default 1x */
	sensor_info->SensorHightSampling = 0;    /* 0 is default 1x */
	sensor_info->SensorPacketECCOrder = 1;

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
	    sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
	    sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

	    sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
				imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
	break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
	    sensor_info->SensorGrabStartX = imgsensor_info.cap.startx;
	    sensor_info->SensorGrabStartY = imgsensor_info.cap.starty;

	    sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.cap.mipi_data_lp2hs_settle_dc;
	break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
	    sensor_info->SensorGrabStartX = imgsensor_info.normal_video.startx;
	    sensor_info->SensorGrabStartY = imgsensor_info.normal_video.starty;

	    sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.normal_video.mipi_data_lp2hs_settle_dc;
	break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
	    sensor_info->SensorGrabStartX = imgsensor_info.hs_video.startx;
	    sensor_info->SensorGrabStartY = imgsensor_info.hs_video.starty;
	    sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.hs_video.mipi_data_lp2hs_settle_dc;
	break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
	    sensor_info->SensorGrabStartX = imgsensor_info.slim_video.startx;
	    sensor_info->SensorGrabStartY = imgsensor_info.slim_video.starty;
	    sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.slim_video.mipi_data_lp2hs_settle_dc;
	break;
	default:
	    sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
	    sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

	    sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
	break;
	}

	return ERROR_NONE;
}    /* get_info */

static kal_uint32 control(MSDK_SCENARIO_ID_ENUM scenario_id,
			MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_DBG("scenario_id = %d\n", scenario_id);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.current_scenario_id = scenario_id;
	spin_unlock(&imgsensor_drv_lock);
	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		LOG_DBG("preview\n");
		preview(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
	case MSDK_SCENARIO_ID_CAMERA_ZSD:
		capture(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		normal_video(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
	hs_video(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
	    slim_video(image_window, sensor_config_data);
		break;
	default:
		preview(image_window, sensor_config_data);
		return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}	/* control() */

static kal_uint32 set_video_mode(UINT16 framerate)
{
	LOG_DBG("framerate = %d ", framerate);
	/* SetVideoMode Function should fix framerate */
	if (framerate == 0)
		/* Dynamic frame rate */
		return ERROR_NONE;
	spin_lock(&imgsensor_drv_lock);

	if ((framerate == 30) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 296;
	else if ((framerate == 15) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 146;
	else
		imgsensor.current_fps = 10 * framerate;
	spin_unlock(&imgsensor_drv_lock);
	set_max_framerate(imgsensor.current_fps, 1);
	set_dummy();
	return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable,
			UINT16 framerate)
{
	LOG_DBG("enable = %d, framerate = %d ", enable, framerate);
	spin_lock(&imgsensor_drv_lock);
	if (enable)
		imgsensor.autoflicker_en = KAL_TRUE;
	else /* Cancel Auto flick */
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}

static kal_uint32 set_max_framerate_by_scenario(
			MSDK_SCENARIO_ID_ENUM scenario_id,
			MUINT32 framerate)
{
	kal_uint32 frame_length;

	LOG_DBG("scenario_id = %d, framerate = %d\n",
				scenario_id, framerate);

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
	    frame_length = imgsensor_info.pre.pclk / framerate * 10 /
			imgsensor_info.pre.linelength;
	    spin_lock(&imgsensor_drv_lock);
	    imgsensor.dummy_line = (frame_length >
			imgsensor_info.pre.framelength) ?
			(frame_length - imgsensor_info.pre.framelength) : 0;
	    imgsensor.frame_length = imgsensor_info.pre.framelength +
			imgsensor.dummy_line;
	    imgsensor.min_frame_length = imgsensor.frame_length;
	    spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
	break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		if (framerate == 0)
			return ERROR_NONE;
	    frame_length = imgsensor_info.normal_video.pclk /
			framerate * 10 / imgsensor_info.normal_video.linelength;
	    spin_lock(&imgsensor_drv_lock);
	    imgsensor.dummy_line = (frame_length >
			imgsensor_info.normal_video.framelength) ?
		(frame_length - imgsensor_info.normal_video.framelength) : 0;
	    imgsensor.frame_length = imgsensor_info.normal_video.framelength +
			imgsensor.dummy_line;
	    imgsensor.min_frame_length = imgsensor.frame_length;
	    spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
	break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		if (imgsensor.current_fps ==
				imgsensor_info.cap1.max_framerate) {
		frame_length = imgsensor_info.cap1.pclk / framerate * 10 /
				imgsensor_info.cap1.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length >
			imgsensor_info.cap1.framelength) ?
			(frame_length - imgsensor_info.cap1.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.cap1.framelength +
				imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		} else {
			if (imgsensor.current_fps !=
				imgsensor_info.cap.max_framerate)
			LOG_DBG("fps %d fps not support,use cap: %d fps!\n",
			framerate, imgsensor_info.cap.max_framerate/10);
			frame_length = imgsensor_info.cap.pclk /
				framerate * 10 / imgsensor_info.cap.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length >
				imgsensor_info.cap.framelength) ?
			(frame_length - imgsensor_info.cap.framelength) : 0;
			imgsensor.frame_length =
				imgsensor_info.cap.framelength +
				imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
		}
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
	break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
	    frame_length = imgsensor_info.hs_video.pclk /
			framerate * 10 / imgsensor_info.hs_video.linelength;
	    spin_lock(&imgsensor_drv_lock);
	    imgsensor.dummy_line = (frame_length >
			imgsensor_info.hs_video.framelength) ? (frame_length -
			imgsensor_info.hs_video.framelength) : 0;
	    imgsensor.frame_length = imgsensor_info.hs_video.framelength +
			imgsensor.dummy_line;
	    imgsensor.min_frame_length = imgsensor.frame_length;
	    spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
	break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
	    frame_length = imgsensor_info.slim_video.pclk /
			framerate * 10 / imgsensor_info.slim_video.linelength;
	    spin_lock(&imgsensor_drv_lock);
	    imgsensor.dummy_line = (frame_length >
			imgsensor_info.slim_video.framelength) ? (frame_length -
			imgsensor_info.slim_video.framelength) : 0;
	    imgsensor.frame_length =
			imgsensor_info.slim_video.framelength +
			imgsensor.dummy_line;
	    imgsensor.min_frame_length = imgsensor.frame_length;
	    spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
	break;
	default:  /* coding with  preview scenario by default */
	    frame_length = imgsensor_info.pre.pclk / framerate * 10 /
						imgsensor_info.pre.linelength;
	    spin_lock(&imgsensor_drv_lock);
	    imgsensor.dummy_line = (frame_length >
			imgsensor_info.pre.framelength) ?
			(frame_length - imgsensor_info.pre.framelength) : 0;
	    imgsensor.frame_length = imgsensor_info.pre.framelength +
				imgsensor.dummy_line;
	    imgsensor.min_frame_length = imgsensor.frame_length;
	    spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
	    LOG_DBG("error scenario_id = %d, we use preview scenario\n",
				scenario_id);
	break;
	}
	return ERROR_NONE;
}

static kal_uint32 get_default_framerate_by_scenario(
				MSDK_SCENARIO_ID_ENUM scenario_id,
				MUINT32 *framerate)
{
	LOG_DBG("scenario_id = %d\n", scenario_id);

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
	    *framerate = imgsensor_info.pre.max_framerate;
	break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
	    *framerate = imgsensor_info.normal_video.max_framerate;
	break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
	    *framerate = imgsensor_info.cap.max_framerate;
	break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
	    *framerate = imgsensor_info.hs_video.max_framerate;
	break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
	    *framerate = imgsensor_info.slim_video.max_framerate;
	break;
	default:
	break;
	}

	return ERROR_NONE;
}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
	LOG_DBG("set_test_pattern_mode enable: %d", enable);

	if (enable) {
		LOG_DBG("enter color bar");
		/* 0x5E00[8]: 1 enable,  0 disable */
		/* 0x5E00[1:0]; 00 Color bar, 01 Random Data, 10 Square, 11 BLACK */
		write_cmos_sensor(0x0a04, 0x0143);
		write_cmos_sensor(0x0200, 0x0002);
	} else {
		/* 0x5E00[8]: 1 enable,  0 disable */
		/* 0x5E00[1:0]; 00 Color bar, 01 Random Data, 10 Square, 11 BLACK */
		write_cmos_sensor(0x0a04, 0x0142);
		write_cmos_sensor(0x0200, 0x0000);
	}
	spin_lock(&imgsensor_drv_lock);
	imgsensor.test_pattern = enable;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}

static kal_uint32 streaming_control(kal_bool enable)
{
	int streamingReg = 0, retry = 0;

	LOG_INF("streaming_enable(0=Sw Standby,1=streaming): %d\n", enable);

	if (enable) {
		while (retry < 6) {
			LOG_INF("Retry %d time(s)\n", retry);
			write_cmos_sensor(0x0a00, 0x0100); /* stream on */
			usleep_range(82000, 82000);
			streamingReg = read_cmos_sensor(0x0a00);
			if (streamingReg == 0x01)
				break;

			retry++;
		}
	} else {
		while (retry < 6) {
			LOG_INF("Retry %d time(s)\n", retry);
			write_cmos_sensor(0x0a00, 0x0000); /* stream off */
			usleep_range(82000, 82000);
			streamingReg = read_cmos_sensor(0x0a00);
			if (streamingReg == 0x00)
				break;

			retry++;
		}
	}
	LOG_INF("Stream On/ Off Reg 0x0A00 = 0x%X \n", streamingReg);

	usleep_range(10000, 10000);

	return ERROR_NONE;
}

static kal_uint32 feature_control(
			MSDK_SENSOR_FEATURE_ENUM feature_id,
			UINT8 *feature_para, UINT32 *feature_para_len)
{
	UINT16 *feature_return_para_16 = (UINT16 *) feature_para;
	UINT16 *feature_data_16 = (UINT16 *) feature_para;
	UINT32 *feature_return_para_32 = (UINT32 *) feature_para;
	UINT32 *feature_data_32 = (UINT32 *) feature_para;
	INT32 *feature_return_para_i32 = (INT32 *) feature_para;
	unsigned long long *feature_data =
		(unsigned long long *) feature_para;
	kal_uint32 rate;

	SENSOR_WINSIZE_INFO_STRUCT *wininfo;
	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data =
		(MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

	LOG_DBG("feature_id = %d\n", feature_id);
	switch (feature_id) {
	case SENSOR_FEATURE_GET_PERIOD:
	    *feature_return_para_16++ = imgsensor.line_length;
	    *feature_return_para_16 = imgsensor.frame_length;
	    *feature_para_len = 4;
	break;
	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
	    *feature_return_para_32 = imgsensor.pclk;
	    *feature_para_len = 4;
	break;
	case SENSOR_FEATURE_SET_ESHUTTER:
	    set_shutter(*feature_data);
	break;
	case SENSOR_FEATURE_SET_NIGHTMODE:
	    night_mode((BOOL) * feature_data);
	break;
	case SENSOR_FEATURE_SET_GAIN:
	    set_gain((UINT16) *feature_data);
	break;
	case SENSOR_FEATURE_SET_FLASHLIGHT:
	break;
	case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
	break;
	case SENSOR_FEATURE_SET_REGISTER:
	    write_cmos_sensor(sensor_reg_data->RegAddr,
						sensor_reg_data->RegData);
	break;
	case SENSOR_FEATURE_GET_REGISTER:
	    sensor_reg_data->RegData =
				read_cmos_sensor(sensor_reg_data->RegAddr);
	break;
	case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
	    *feature_return_para_32 = LENS_DRIVER_ID_DO_NOT_CARE;
	    *feature_para_len = 4;
	break;
	case SENSOR_FEATURE_SET_VIDEO_MODE:
	    set_video_mode(*feature_data);
	break;
	case SENSOR_FEATURE_CHECK_SENSOR_ID:
	    get_imgsensor_id(feature_return_para_32);
	break;
	case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
	    set_auto_flicker_mode((BOOL)*feature_data_16,
			*(feature_data_16+1));
	break;
	case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
	    set_max_framerate_by_scenario(
			(MSDK_SCENARIO_ID_ENUM)*feature_data,
			*(feature_data+1));
	break;
	case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
	    get_default_framerate_by_scenario(
			(MSDK_SCENARIO_ID_ENUM)*(feature_data),
			(MUINT32 *)(uintptr_t)(*(feature_data+1)));
	break;
	case SENSOR_FEATURE_SET_TEST_PATTERN:
	    set_test_pattern_mode((BOOL)*feature_data);
	break;
	case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
	    *feature_return_para_32 = imgsensor_info.checksum_value;
	    *feature_para_len = 4;
	break;
	case SENSOR_FEATURE_GET_SENSOR_ID:
		*feature_return_para_32 = imgsensor_info.sensor_id;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_FRAMERATE:
	    LOG_DBG("current fps :%d\n", (UINT32)*feature_data);
	    spin_lock(&imgsensor_drv_lock);
	    imgsensor.current_fps = *feature_data;
	    spin_unlock(&imgsensor_drv_lock);
	break;

	case SENSOR_FEATURE_SET_HDR:
	    LOG_DBG("ihdr enable :%d\n", (BOOL)*feature_data);
	    spin_lock(&imgsensor_drv_lock);
	    imgsensor.ihdr_en = (BOOL)*feature_data;
	    spin_unlock(&imgsensor_drv_lock);
	break;
	case SENSOR_FEATURE_GET_CROP_INFO:
	    LOG_DBG("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n",
				(UINT32)*feature_data);

	    wininfo = (SENSOR_WINSIZE_INFO_STRUCT *)
			(uintptr_t)(*(feature_data+1));

		switch (*feature_data_32) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			memcpy((void *)wininfo,
				(void *)&imgsensor_winsize_info[1],
				sizeof(SENSOR_WINSIZE_INFO_STRUCT));
		break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			memcpy((void *)wininfo,
				(void *)&imgsensor_winsize_info[2],
				sizeof(SENSOR_WINSIZE_INFO_STRUCT));
		break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			memcpy((void *)wininfo,
				(void *)&imgsensor_winsize_info[3],
				sizeof(SENSOR_WINSIZE_INFO_STRUCT));
		break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			memcpy((void *)wininfo,
				(void *)&imgsensor_winsize_info[4],
				sizeof(SENSOR_WINSIZE_INFO_STRUCT));
		break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			memcpy((void *)wininfo,
				(void *)&imgsensor_winsize_info[0],
				sizeof(SENSOR_WINSIZE_INFO_STRUCT));
		break;
		}
	break;
	case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
	    LOG_DBG("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",
			(UINT16)*feature_data, (UINT16)*(feature_data+1),
			(UINT16)*(feature_data+2));
	#if 0
	    ihdr_write_shutter_gain((UINT16)*feature_data,
			(UINT16)*(feature_data+1), (UINT16)*(feature_data+2));
	#endif
	break;
	case SENSOR_FEATURE_GET_TEMPERATURE_VALUE:
		*feature_return_para_i32 = 0;
		*feature_para_len = 4;
	break;
	case SENSOR_FEATURE_SET_STREAMING_SUSPEND:
		LOG_INF("SENSOR_FEATURE_SET_STREAMING_SUSPEND\n");
		streaming_control(KAL_FALSE);
	break;
	case SENSOR_FEATURE_SET_STREAMING_RESUME:
		LOG_INF("SENSOR_FEATURE_SET_STREAMING_RESUME, shutter:%llu\n", *feature_data);
		if (*feature_data != 0)
			set_shutter(*feature_data);
		streaming_control(KAL_TRUE);
		break;
	case SENSOR_FEATURE_GET_MIPI_PIXEL_RATE:
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			rate = imgsensor_info.cap.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			rate = imgsensor_info.normal_video.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			rate = imgsensor_info.hs_video.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			rate = imgsensor_info.slim_video.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			rate = imgsensor_info.pre.mipi_pixel_rate;
			break;
		default:
			rate = 0;
			break;
		}
		*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = rate;
		break;
	default:
		break;
	}

	return ERROR_NONE;
}    /* feature_control() */

static SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close
};

UINT32 HI556SEC_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc != NULL)
		*pfFunc = &sensor_func;
	return ERROR_NONE;
}	/* HI556SEC_MIPI_RAW_SensorInit */
