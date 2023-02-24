 /************************************************************
  *
  * file: p9415.h
  *
  * Description: Interface of P9415 to AP access included file
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

#ifndef __IDTP9415_H__
#define __IDTP9415_H__

/* RX -> TX */
#define PROPRIETARY18		0x18
#define PROPRIETARY28		0x28
#define PROPRIETARY38		0x38
#define PROPRIETARY48		0x48
#define PROPRIETARY58		0x58

/* bits mask */
#define BIT0			0x01
#define BIT1			0x02
#define BIT2			0x04
#define BIT3			0x08
#define BIT4			0x10
#define BIT5			0x20
#define BIT6			0x40
#define BIT7			0x80
#define BIT8			(1 << 8)
#define BIT9			(1 << 9)
#define BIT10			(1 << 10)
#define BIT11			(1 << 11)
#define BIT12			(1 << 12)
#define BIT13			(1 << 13)
#define BIT14			(1 << 14)
#define BIT15			(1 << 15)
#define BIT16			(1 << 16)
#define BIT17			(1 << 17)
#define BIT18			(1 << 18)
#define BIT19			(1 << 19)

/* used registers define */
#define REG_CHIP_ID		0x0000
#define REG_CHIP_REV	0x0002

/* format of release firmware:
 * <cust_code>.<proj_id>.<major_rev>.<minor_rev>
 */
#define REG_CUST_CODE		0x0004	/* 2 bytes */
#define REG_PROJ_ID			0x0006	/* 2 bytes */
#define REG_FW_MAJOR_REV	0x0008	/* 1 byte */
#define REG_FW_MINOR_REV	0x0009	/* 1 byte */

#define REG_BC_Timeout	0x0022

#define REG_INTCLR		0x0028
#define REG_INT_CLEAR	REG_INTCLR

#define REG_SSStatus	0x002C
#define REG_STATUS		REG_SSStatus

#define REG_INTFLAG		0x0030
#define REG_INTR		REG_INTFLAG

#define REG_INTEN		0x0034
#define REG_INTR_EN		REG_INTEN

#define REG_CHG_VAL		0x003A
#define REG_CHG_STATUS	REG_CHG_VAL

#define REG_EPT_VAL		0x003B
#define REG_EPT			REG_EPT_VAL

#define REG_ADC_VOUT	0x003C
#define REG_VOUT_SET	0x003E
#define REG_VRECT_ADJ	0x003F
#define REG_ADC_VRECT	0x0040
#define REG_RX_IOUT		0x0044
#define REG_RX_IOUT_RAW	0x0064
#define REG_ADC_TEMP	0x0046
#define REG_OP_FREQ		0x0048
#define REG_ILIM_SET	0x004A
/* Signal Strength Register */
#define REG_SS			0x004B
#define REG_SYS_Mode	0x004D
#define REG_COMMAND		0x004E
/* Proprietary Packet Header Register, PPP_Header (0x50) */
#define REG_PROPPKT_ADDR	0x0050
/* PPP Data Value Register(0X51, 0x52, 0x53, 0x54, 0x55) */
#define REG_PPPDATA_ADDR	0x0051
/* Back Channel Packet Register (0x58) */
#define REG_BCHEADER_ADDR	0x0058
/* Back Channel Packet Register (0x59, 0x5A, 0x5B, 0x5C) */
#define REG_BCDATA_ADDR		0x0059
/* Communocation CAP Enable */
#define REG_CM_CAP_EN_ADDR	0x00B6
/* FOD parameters addr, 16 bytes */
#define REG_FOD_COEF_ADDR	0x0068
#define REG_FOD_DUMP_MAX	0x0073
#define REG_FC_VOLTAGE		0x0078
#define REG_TX_ID			0x011A	/* 2 bytes */
#define REG_OVP_SEL			0x014A	/* 1 bytes */
#define REG_DUMP_MAX		0x0084

#define REG_GAIN1			0x03F2 /* 2 bytes */
#define REG_GAIN2			0x03F4 /* 2 bytes */
#define REG_GAIN3			0x03F6 /* 2 bytes */

#define IDT_FAST_CHARING_EN	0
#define IDT_PROGRAM_FIRMWARE_EN	1

#define CHARGER_VOUT_10W	9000
#define CHARGER_VOUT_5W		6500

#define READY_DETECT_TIME	(50*HZ/1000)

#define SWITCH_DETECT_TIME	(300*HZ/1000)
#define SWITCH_10W_VTH_L	8500
#define SWITCH_10W_VTH_H	9500
#define SWITCH_5W_VTH_L		6000
#define SWITCH_5W_VTH_H		7000
#define SWITCH_VOLTAGE_COUNT	6
#define DIS_CM_CAP	0x00
#define EN_CM_CAP	0XF0

/* bitmap for system operating mode 0x4D */
#define RX_MODE_BPP		BIT0
#define RX_MODE_EPP		(BIT3|BIT0)

/* bitmap for SSCmnd register 0x4e */
#define VSWITCH			BIT7
/*
 * If AP sets this bit to "1" then IDTP9415 M0 clears the interrupt
 * corresponding to the bit(s) which has a value of "1"
 */
#define CLRINT			BIT5

/*
 * If AP sets this bit to 1 then IDTP9415 M0 sends the Charge Status packet
 * (defined in the Battery Charge Status Register) to TX
 * and then sets this bit to 0
 */
#define CHARGE_STAT		BIT4

/*
 * If AP sets this bit to "1" then IDTP9415 M0 sends the End of Power packet
 * (define in the End of Power Transfer Register) to Tx
 * and then sets this bit to "0"
 */
#define SENDEOP			BIT3

/*
 * If AP sets this bit to 1 then IDTP9415 M0 start the device authintication
 */
#define SEND_DEVICE_AUTH		BIT2

/*
 * If AP sets this bit to "1" then IDTP9415 M0 toggles LDO output once
 * (from on to off, or from off to on), and then sets this bit to "0"
 */
#define LDOTGL			BIT1
/* If AP sets this bit to "1" then IDTP9415 M0 sends the Proprietary Packet */
#define SENDPROPP		BIT0

/* bitmap for interrupt register 0x2C */
#define P9415_INT_NTCOVERTEMP		BIT19
#define P9415_INT_VRECTON			BIT18
#define P9415_INT_VSWITCH_FAILED	BIT15
#define P9415_INT_SLEEP_MODE		BIT14
#define P9415_INT_ID_AUTH_SUCCESS	BIT13
#define P9415_INT_ID_AUTH_FAIL		BIT12
#define P9415_INT_SEND_PKT_SUCCESS	BIT11
#define P9415_INT_SEND_PKT_TIMEOUT	BIT10
#define P9415_INT_DEVICE_AUTH_SUCCESS	BIT9
#define P9415_INT_DEVICE_AUTH_FAIL	BIT8
#define P9415_INT_LDO_OFF			BIT7
#define P9415_INT_LDO_ON			BIT6
#define P9415_INT_MODE_CHANGE		BIT5
#define P9415_INT_TX_DATA_RCVD		BIT4
#define P9415_INT_TX_DATA_RECV		P9415_INT_TX_DATA_RCVD
#define P9415_INT_VSWITCHSUCCESS	BIT3
#define P9415_INT_OV_TEMP			BIT2
#define P9415_INT_OV_VOLT			BIT1
#define P9415_INT_OV_CURRENT		BIT0
#define P9415_INT_LIMIT_MASK		(P9415_INT_OV_TEMP | \
					P9415_INT_OV_VOLT | \
					P9415_INT_OV_CURRENT)

/* bitmap for customer command */
#define BC_NONE			0x00
#define BC_SET_FREQ		0x03
#define BC_GET_FREQ		0x04
#define BC_READ_FW_VER	0x05
#define BC_READ_Iin		0x06
#define BC_READ_Vin		0x07
#define BC_SET_Vin		0x0a
#define BC_ADAPTER_TYPE	0x0b
#define BC_RESET		0x0c
#define BC_READ_I2C		0x0d
#define BC_WRITE_I2C	0x0e
#define BC_VI2C_INIT	0x10
#define BC_GET_TX_CAP	0x63	/* Get Tx capbility */

#define IDT_INT			"p9415-int"
#define IDT_PG			"p9415-pg"

#define SET_VOUT_MAX	12500
#define SET_VOUT_MIN	3500

/* End of Power packet types */
#define EOP_OVER_TEMP		0x03
#define EOP_OVER_VOLT		0x04
#define EOP_OVER_CURRENT	0x05

#define FOD_COEF_ARRY_LENGTH	6
#define FOD_COEF_PARAM_LENGTH	12

#define P9415_DIE_TEMP_DEFAULT	-177

#define OVP_SEL_0_18V	0x0	/* 18.04V */
#define OVP_SEL_1_21V	0x1	/* 21V */

/* Adapter Type */
enum adapter_list {
	ADAPTER_UNKNOWN	= 0x00,
	ADAPTER_SDP		= 0x01,
	ADAPTER_CDP		= 0x02,
	ADAPTER_DCP		= 0x03,
	ADAPTER_QC20	= 0x05,
	ADAPTER_QC30	= 0x06,
	ADAPTER_PD		= 0x07,
};

enum charge_mode {
	CHARGE_5W_MODE,
	CHARGE_10W_MODE,
	CHARGE_15W_MODE,
	CHARGE_MODE_MAX,
};

struct input_dev;
struct notifier_block;

struct idtp9415pgmtype {	/* write to structure at SRAM address 0x0400 */
	/* Read/Write by both 9415 and 9415 host */
	u16 status;
	/* OTP image address of the current packet */
	u16 startAddr;
	/* The size of the OTP image data in the current packet */
	u16 codeLength;
	/* Checksum of the current packet */
	u16 dataChksum;
	/* OTP image data of the current packet */
	u8	dataBuf[128];
};

/* proprietary packet type */
struct propkt_type {
	u8 header;	/* Packet type */
	u8 cmd;		/* Back channel command */
	u8 msg[5];	/* Send data buffer */
};

struct access_func {
	int (*read)(void *data, u16 reg, u8 *val);
	int (*write)(void *data, u16 reg, u8 val);
	int (*read_buf)(void *data, u16 reg, u8 *buf, u32 size);
	int (*write_buf)(void *data, u16 reg, u8 *buf, u32 size);
};

struct p9415_desc {
	const char *chg_dev_name;
	const char *alias_name;
};

struct p9415_switch_voltage {
	int voltage_low;
	int voltage_target;
	int voltage_high;
};

/*
 * In each RPP (Received Power Packet), RX will use
 * 'RPP = calcuatedPower * gain + offset' to report the power.
 */
struct p9415_fodcoeftype {
	u8 gain;
	u8 offs;
};

struct p9415_reg {
	u16 addr;
	u8  size;
};

union fw_ver {
	uint8_t val[6];
	struct {
		uint16_t cust_code;
		uint16_t proj_id;
		uint8_t major_rev;
		uint8_t minor_rev;
	};
};

union calibration_data {
	uint8_t raw_data[128];
	struct {
		uint16_t I_Gain1;
		uint16_t I_Gain2;
		uint16_t I_Gain3;
		int16_t I_Offset1;
		int16_t I_Offset2;
		int16_t I_Offset3;
		uint16_t I_CoeffThd1;
		uint16_t I_CoeffThd2;
		uint16_t I_CoeffThd3;
		uint16_t I_CoeffThd4;
		uint16_t I_Gain_Thd_H;
		uint16_t I_Gain_Thd_L;
		uint8_t *data;
	};
};

#define SW_FOD_RECORD_SIZE	4

struct step_load {
	int start_ma;
	int step_ma;
	int step_max_ua;
	int bpp_plus_max_ua;
	uint32_t step_interval;	/* ms */
};

#define IBUS_BUFFER_SIZE 6
struct adaptive_current_limit {
	int start_soc;
	uint32_t current_index;
	int fill_count;
	bool on_adaptive;

	int interval;
	int margin;
	int bpp_plus_max_ma;
	int max_current_limit;
	int min_current_limit;
	int start_ma;
	int ibus[IBUS_BUFFER_SIZE];
};

struct p9415_dev {
	char *name;
	struct i2c_client *client;
	struct device *dev;
	struct regmap *regmap;
	struct access_func bus;
	struct p9415_desc *desc;
	struct charger_device *chg_dev;
	struct charger_properties chg_props;
	struct mutex sys_lock;
	struct mutex irq_lock;
	struct mutex fod_lock;
	struct mutex switch_lock;
	struct power_supply_desc wpc_desc;
	struct power_supply_config wpc_cfg;
	struct delayed_work wpc_init_work;
	struct delayed_work fast_charging_work;
	struct delayed_work bpp_switch_work;
	struct p9415_reg reg;
	struct power_supply *wpc_psy;

	int int_gpio;
	struct gpio_desc *pg_gpio;
	int int_num;
	int pg_num;
	atomic_t online;
	struct pinctrl *wpc_pinctrl;
	struct pinctrl_state *wpc_enable;
	struct pinctrl_state *wpc_disable;
	struct pinctrl_state *epp_enable;
	struct pinctrl_state *epp_disable;
	struct pinctrl_state *fw_dl_enable;
	struct pinctrl_state *fw_dl_disable;
	struct pinctrl_state *sleep_enable;
	struct pinctrl_state *sleep_disable;
	bool epp_en;
	bool vout_en;
	bool support_sleep_en;
	bool fw_dl_en;
	bool wpc_en;
	/* communication cap enable */
	bool cm_cap_en;

	uint16_t vout_10w;
	uint16_t vout_15w;

	enum charger_type charger;
	struct charger_consumer *consumer;

	/* fastcharge switch */
	bool tx_id_authen_status;
	bool tx_dev_authen_status;
	bool tx_authen_complete;
	int wpc_mivr[CHARGE_MODE_MAX];
	struct p9415_fodcoeftype bpp_5w_fod[FOD_COEF_ARRY_LENGTH];
	struct p9415_fodcoeftype epp_10w_fod[FOD_COEF_ARRY_LENGTH];
	struct p9415_fodcoeftype bpp_plus_15w_fod[FOD_COEF_ARRY_LENGTH];
	struct switch_dev dock_state;
	bool is_hv_adapter;
	bool is_enabled;
	u8 tx_adapter_type;
	u8 over_reason;
	u8 bpp_5w_fod_num;
	u8 epp_10w_fod_num;
	u8 bpp_plus_15w_fod_num;
	u8 dev_auth_retry;
	atomic_t vswitch_retry;
	atomic_t vswitch_done;
	u8 send_cmd_retry;

	bool en_chk_tx_cap;
	uint16_t tx_max_vbridge;
	u8 tx_max_power;
	u8 rx_mode;

	union fw_ver fw_ver;

	int sw_fod_count;
	struct timespec sw_fod_time_record[SW_FOD_RECORD_SIZE];

	int throttle_threshold;
	int throttle_hysteresis_threshold;

	struct delayed_work EPT_work;
	uint32_t EPT_work_delay;	/* unite: ms */

	struct step_load step_load;
	bool on_step_charging;
	struct delayed_work step_charging_work;
	struct timespec start_step_chg_time;

	struct delayed_work adaptive_current_limit_work;
	struct charger_device *prim_chg_dev;
	struct adaptive_current_limit adaptive_current_limit;
	struct power_supply *bat_psy;
	int prev_input_current;
};

enum dock_state_type {
	TYPE_DOCKED = 7,
	TYPE_UNDOCKED = 8,
};

enum led_mode {
	LED_CONSTANT_ON = 99,
	LED_CONSTANT_OFF = 100,
};

#endif
