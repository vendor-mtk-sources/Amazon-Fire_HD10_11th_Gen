/*
 * Copyright (C) 2019 Amazon.com
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

/*
 * bu64982Aaf voice coil motor driver
 *
 *
 */

#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/fs.h>

#include "lens_info.h"


#define AF_DRVNAME "BU64982AF_DRV"

#define AF_DEBUG
#ifdef AF_DEBUG
#define LOG_INF(format, args...) pr_debug(AF_DRVNAME " [%s] " format, __func__, ##args)
#else
#define LOG_INF(format, args...)
#endif

#define BU64982AF_PS_EN	0x80
#define BU64982AF_PS_STANDBY	0

#define BU64982AF_EN_ZEROCUR	0
#define BU64982AF_EN_CONSTCUR	0x40

#define BU64982AF_W_OUTPUT_CUR	0
#define BU64982AF_W_SRC	0x8
#define BU64982AF_W_SET_CUR	0x10

#define BU64982AF_M_DIRECT	0
#define BU64982AF_M_ISRC	0x4

#define BU64982AF_D_MSB_MASK	0x3

#define BU64982AF_D_LSB_MASK	0xff

#define BU64982AF_MACRO_DEF	1023

static struct i2c_client *g_pstAF_I2Cclient;
static int *g_pAF_Opened;
static spinlock_t *g_pAF_SpinLock;
static int g_i4DriverStatus;

static unsigned long g_u4AF_INF;
static unsigned long g_u4AF_MACRO = BU64982AF_MACRO_DEF;
static unsigned long g_u4TargetPosition;
static unsigned long g_u4CurrPosition;

static int bu64982_ReadCur_Direct(unsigned short *a_pu2Result)
{
	int i4RetValue = 0;
	char pBuff[2];

	char puSendCmd[1] = {(char)(BU64982AF_PS_EN | BU64982AF_EN_CONSTCUR |
				BU64982AF_W_OUTPUT_CUR | BU64982AF_M_DIRECT)};

	i4RetValue = i2c_master_send(g_pstAF_I2Cclient, puSendCmd, 1);

	if (i4RetValue < 0) {
		g_i4DriverStatus++;
		LOG_INF("I2C read failed - %d!!\n", g_i4DriverStatus);
		return -1;
	}

	i4RetValue = i2c_master_recv(g_pstAF_I2Cclient, pBuff, 2);

	if (i4RetValue < 0) {
		g_i4DriverStatus++;
		LOG_INF("I2C read failed - %d!!\n", g_i4DriverStatus);
		return -1;
	}

	*a_pu2Result = (((u16)(pBuff[0] & BU64982AF_D_MSB_MASK)) << 8) + pBuff[1];

	return 0;
}

static int bu64982_SetCur_Direct(u16 a_u2Data)
{
	int i4RetValue = 0;

	char puSendCmd[2] = {(char)(((a_u2Data >> 8) & BU64982AF_D_MSB_MASK) |
				BU64982AF_PS_EN | BU64982AF_EN_CONSTCUR |
				BU64982AF_W_OUTPUT_CUR | BU64982AF_M_DIRECT),
				(char)(a_u2Data & BU64982AF_D_LSB_MASK)};

	i4RetValue = i2c_master_send(g_pstAF_I2Cclient, puSendCmd, 2);

	if (i4RetValue < 0) {
		g_i4DriverStatus++;
		LOG_INF("I2C read failed - %d!!\n", g_i4DriverStatus);
		return -1;
	}

	return 0;
}

static inline int getAFInfo(__user struct stAF_MotorInfo *pstMotorInfo)
{
	struct stAF_MotorInfo stMotorInfo;

	stMotorInfo.u4MacroPosition = g_u4AF_MACRO;
	stMotorInfo.u4InfPosition = g_u4AF_INF;
	stMotorInfo.u4CurrentPosition = g_u4CurrPosition;
	stMotorInfo.bIsSupportSR = 1;

	stMotorInfo.bIsMotorMoving = 1;

	if (*g_pAF_Opened >= 1)
		stMotorInfo.bIsMotorOpen = 1;
	else
		stMotorInfo.bIsMotorOpen = 0;

	if (copy_to_user(pstMotorInfo, &stMotorInfo, sizeof(struct stAF_MotorInfo)))
		LOG_INF("copy to user failed when getting motor information\n");

	return 0;
}

static inline int moveAF(unsigned long a_u4Position)
{
	int ret = 0;
	unsigned short InitPos;

	if (g_i4DriverStatus > 2) /* I2C failed */
		return -EINVAL;

	if ((a_u4Position > g_u4AF_MACRO) || (a_u4Position < g_u4AF_INF)) {
		LOG_INF("out of range\n");
		return -EINVAL;
	}

	LOG_INF("Move to Pos %lu\n", a_u4Position);

	if (*g_pAF_Opened == 1) {
		ret = bu64982_ReadCur_Direct(&InitPos);

		if (ret == 0) {
			LOG_INF("Init Pos %d\n", InitPos);

			spin_lock(g_pAF_SpinLock);
			g_u4CurrPosition = (unsigned long)InitPos;
			spin_unlock(g_pAF_SpinLock);

		} else {
			spin_lock(g_pAF_SpinLock);
			g_u4CurrPosition = 0;
			spin_unlock(g_pAF_SpinLock);
		}

		spin_lock(g_pAF_SpinLock);
		*g_pAF_Opened = 2;
		spin_unlock(g_pAF_SpinLock);
	}

	if (g_u4CurrPosition == a_u4Position)
		return 0;

	spin_lock(g_pAF_SpinLock);
	g_u4TargetPosition = a_u4Position;
	spin_unlock(g_pAF_SpinLock);

	if (bu64982_SetCur_Direct((unsigned short)g_u4TargetPosition) == 0) {
		spin_lock(g_pAF_SpinLock);
		g_u4CurrPosition = (unsigned long)g_u4TargetPosition;
		spin_unlock(g_pAF_SpinLock);
	} else {
		LOG_INF("set I2C failed when moving the motor\n");
		ret = -1;
	}

	return ret;
}

static inline int setAFInf(unsigned long a_u4Position)
{
	spin_lock(g_pAF_SpinLock);
	g_u4AF_INF = a_u4Position;
	spin_unlock(g_pAF_SpinLock);
	return 0;
}

static inline int setAFMacro(unsigned long a_u4Position)
{
	spin_lock(g_pAF_SpinLock);
	g_u4AF_MACRO = a_u4Position;
	spin_unlock(g_pAF_SpinLock);
	return 0;
}

long bu64982af_Ioctl(struct file *a_pstFile, unsigned int a_u4Command, unsigned long a_u4Param)
{
	long i4RetValue = 0;

	switch (a_u4Command) {
	case AFIOC_G_MOTORINFO:
		i4RetValue = getAFInfo((__user struct stAF_MotorInfo *) (a_u4Param));
		break;

	case AFIOC_T_MOVETO:
		i4RetValue = moveAF(a_u4Param);
		break;

	case AFIOC_T_SETINFPOS:
		i4RetValue = setAFInf(a_u4Param);
		break;

	case AFIOC_T_SETMACROPOS:
		i4RetValue = setAFMacro(a_u4Param);
		break;

	default:
		LOG_INF("No CMD\n");
		i4RetValue = -EPERM;
		break;
	}

	return i4RetValue;
}

/* Main jobs: */
/* 1.Deallocate anything that "open" allocated in private_data. */
/* 2.Shut down the device on last close. */
/* 3.Only called once on last time. */
int bu64982af_Release(struct inode *a_pstInode, struct file *a_pstFile)
{
	if (*g_pAF_Opened == 2) {
		char puSendCmd[2];

		LOG_INF("Wait\n");
		puSendCmd[0] = (char)(0x00);
		puSendCmd[1] = (char)(0x00);
		i2c_master_send(g_pstAF_I2Cclient, puSendCmd, 2);
	}

	if (*g_pAF_Opened) {
		LOG_INF("Free\n");

        LOG_INF("Current Position:%lu\n", g_u4CurrPosition);
        if (g_u4AF_INF < g_u4CurrPosition) {
            int CntStep = g_u4CurrPosition - g_u4AF_INF;
            int Position = g_u4CurrPosition;

            while (CntStep > 0) {
                bu64982_SetCur_Direct((unsigned short)Position);
                mdelay(10);
                Position -= 10;
                LOG_INF("Current Position:%d\n", Position);
                CntStep -= 10;
            }
        }

		spin_lock(g_pAF_SpinLock);
		*g_pAF_Opened = 0;
		spin_unlock(g_pAF_SpinLock);
	}

	g_i4DriverStatus = 0;

	return 0;
}

int bu64982af_SetI2Cclient(struct i2c_client *pstAF_I2Cclient, spinlock_t *pAF_SpinLock, int *pAF_Opened)
{
	g_pstAF_I2Cclient = pstAF_I2Cclient;
	g_pAF_SpinLock = pAF_SpinLock;
	g_pAF_Opened = pAF_Opened;

	return 1;
}
