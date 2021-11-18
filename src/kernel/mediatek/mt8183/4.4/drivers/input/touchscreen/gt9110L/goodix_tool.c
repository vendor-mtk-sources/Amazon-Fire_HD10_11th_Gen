/*
 * Goodix GT9xx touchscreen driver
 *
 * Copyright  (C)  2016 - 2017 Goodix. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be a reference
 * to you, when you are integrating the GOODiX's CTP IC into your system,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * Version: 2.8.0.1
 * Release Date: 2017/11/24
 */

#include "gt9xx.h"

#define DATA_LENGTH_UINT	512
#define CMD_HEAD_LENGTH		(sizeof(struct st_cmd_head) - sizeof(u8 *))
static char procname[20] = {0};

#pragma pack(1)
struct st_cmd_head {
	u8	wr;		/*write read flag 0:R 1:W 2:PID 3:*/
	u8	flag;		/*0:no need flag/int 1: need flag 2:need int*/
	u8	flag_addr[2];	/*flag address*/
	u8	flag_val;	/*flag val*/
	u8	flag_relation;	/*flag_val:flag 0:not equal 1:equal 2:> 3:<*/
	u16	circle;		/*polling cycle*/
	u8	times;		/*plling times*/
	u8	retry;		/*I2C retry times*/
	u16	delay;		/*delay before read or after write*/
	u16	data_len;	/*data length*/
	u8	addr_len;	/*address length*/
	u8	addr[2];	/*address*/
	u8	res[3];		/*reserved*/
	u8	*data; };	/*data pointer*/
#pragma pack()
struct st_cmd_head cmd_head;

static struct i2c_client *gt_client;
static struct proc_dir_entry *goodix_proc_entry;

static ssize_t goodix_tool_read(struct file *, char __user *, size_t, loff_t *);
static ssize_t goodix_tool_write(struct file *, const char __user *,
	size_t, loff_t *);
static const struct file_operations gtp_proc_ops = {
	.owner = THIS_MODULE,
	.read = goodix_tool_read,
	.write = goodix_tool_write,
};

/* static s32 goodix_tool_write(struct file *filp,
 * const char __user *buff, unsigned long len, void *data);
 */
/*static s32 goodix_tool_read( char *page, char
 **start, off_t off, int count, int *eof, void *data );
 */
static s32 (*tool_i2c_read)(u8 *, u16);
static s32 (*tool_i2c_write)(u8 *, u16);

static s32 DATA_LENGTH = (s32)0;
static s8 IC_TYPE[16] = "GT9XX";

static void tool_set_proc_name(char *procname)
{
	snprintf(procname, 20, "gmnode"); /* modify for moto */
}

static s32 tool_i2c_read_no_extra(u8 *buf, u16 len)
{
	s32 ret = -1;
	s32 i = 0;
	struct goodix_ts_data *ts = i2c_get_clientdata(i2c_connect_client);

	for (i = 0; i < cmd_head.retry; i++) {
		ret = gtp_i2c_read(ts->client, buf, len + GTP_ADDR_LENGTH);
		if (ret > 0)
			break;
	}
	return ret;
}

static s32 tool_i2c_write_no_extra(u8 *buf, u16 len)
{
	s32 ret = -1;
	s32 i = 0;
	struct goodix_ts_data *ts = i2c_get_clientdata(i2c_connect_client);

	for (i = 0; i < cmd_head.retry; i++) {
		ret = gtp_i2c_write(ts->client, buf, len);
		if (ret > 0)
			break;
	}

	return ret;
}

static s32 tool_i2c_read_with_extra(u8 *buf, u16 len)
{
	s32 ret = -1;
	u8 pre[2] = {0x0f, 0xff};
	u8 end[2] = {0x80, 0x00};

	tool_i2c_write_no_extra(pre, 2);
	ret = tool_i2c_read_no_extra(buf, len);
	tool_i2c_write_no_extra(end, 2);

	return ret;
}

static s32 tool_i2c_write_with_extra(u8 *buf, u16 len)
{
	s32 ret = -1;
	u8 pre[2] = {0x0f, 0xff};
	u8 end[2] = {0x80, 0x00};

	tool_i2c_write_no_extra(pre, 2);
	ret = tool_i2c_write_no_extra(buf, len);
	tool_i2c_write_no_extra(end, 2);

	return ret;
}

static void register_i2c_func(void)
{
	/* if (!strncmp(IC_TYPE, "GT818", 5)
	 *  || !strncmp(IC_TYPE, "GT816", 5)
	 *  || !strncmp(IC_TYPE, "GT811", 5)
	 *  || !strncmp(IC_TYPE, "GT818F", 6)
	 *  || !strncmp(IC_TYPE, "GT827", 5)
	 *  || !strncmp(IC_TYPE,"GT828", 5)
	 *  || !strncmp(IC_TYPE, "GT813", 5))
	 */
	if (strncmp(IC_TYPE, "GT8110", 6) &&
	    strncmp(IC_TYPE, "GT8105", 6) &&
	    strncmp(IC_TYPE, "GT801", 5) &&
	    strncmp(IC_TYPE, "GT800", 5) &&
	    strncmp(IC_TYPE, "GT801PLUS", 9) &&
	    strncmp(IC_TYPE, "GT811", 5) &&
	    strncmp(IC_TYPE, "GTxxx", 5) &&
	    strncmp(IC_TYPE, "GT9XX", 5)) {
		tool_i2c_read = tool_i2c_read_with_extra;
		tool_i2c_write = tool_i2c_write_with_extra;
		TP_LOGD("I2C function: with pre and end cmd!\n");
	} else {
		tool_i2c_read = tool_i2c_read_no_extra;
		tool_i2c_write = tool_i2c_write_no_extra;
		TP_LOGD("I2C function: without pre and end cmd!\n");
	}
}

static void unregister_i2c_func(void)
{
	tool_i2c_read = NULL;
	tool_i2c_write = NULL;
	TP_LOGD("I2C function: unregister i2c transfer function!\n");
}

s32 init_wr_node(struct i2c_client *client)
{
	s32 i;

	gt_client = client;
	memset(&cmd_head, 0, sizeof(cmd_head));
	cmd_head.data = NULL;

	i = 6;
	while ((!cmd_head.data) && i) {
		cmd_head.data = kzalloc(i * DATA_LENGTH_UINT, GFP_KERNEL);
		if (cmd_head.data)
			break;
		i--;
	}
	if (i) {
		DATA_LENGTH = i * DATA_LENGTH_UINT - GTP_ADDR_LENGTH;
		TP_LOGD("Alloc memory size:%d.\n", DATA_LENGTH);
	} else {
		TP_LOGE("Apply for memory failed.\n");
		return FAIL;
	}

	cmd_head.addr_len = 2;
	cmd_head.retry = 5;

	register_i2c_func();

	tool_set_proc_name(procname);
	goodix_proc_entry = proc_create(procname, 0664, NULL, &gtp_proc_ops);
	if (!goodix_proc_entry) {
		TP_LOGE("Couldn't create proc entry!\n");
		return FAIL;
	}

	TP_LOGD("Create proc entry success!\n");
	return SUCCESS;
}

void uninit_wr_node(void)
{
	kfree(cmd_head.data);
	cmd_head.data = NULL;
	unregister_i2c_func();
	remove_proc_entry(procname, NULL);
}

static u8 relation(u8 src, u8 dst, u8 rlt)
{
	u8 ret = 0;

	switch (rlt) {
	case 0:
		ret = (src != dst) ? true : false;
		break;

	case 1:
		ret = (src == dst) ? true : false;
		TP_LOGD("equal:src:0x%02x  dst:0x%02x  ret:%d.\n",
			src, dst, (s32)ret);
		break;

	case 2:
		ret = (src > dst) ? true : false;
		break;

	case 3:
		ret = (src < dst) ? true : false;
		break;

	case 4:
		ret = (src & dst) ? true : false;
		break;

	case 5:
		ret = (!(src | dst)) ? true : false;
		break;

	default:
		ret = false;
		break;
	}

	return ret;
}

/*******************************************************
 * Function:
 *	Comfirm function.
 * Input:
 *	None.
 * Output:
 *	Return write length.
 ********************************************************/
static u8 comfirm(void)
{
	s32 i = 0;
	u8 buf[32];

	memcpy(buf, cmd_head.flag_addr, cmd_head.addr_len);

	for (i = 0; i < cmd_head.times; i++) {
		if (tool_i2c_read(buf, 1) <= 0) {
			TP_LOGE("Read flag data failed!\n");
			return FAIL;
		}
		if (true == relation(buf[GTP_ADDR_LENGTH],
		 cmd_head.flag_val, cmd_head.flag_relation)) {
			TP_LOGD("value at flag addr:0x%02x.\n",
				buf[GTP_ADDR_LENGTH]);
			TP_LOGD("flag value:0x%02x.\n", cmd_head.flag_val);
			break;
		}

		msleep(cmd_head.circle);
	}

	if (i >= cmd_head.times) {
		TP_LOGE("Can't get the continue flag!\n");
		return FAIL;
	}

	return SUCCESS;
}

ssize_t goodix_tool_write(struct file *filp, const char __user *buff,
			  size_t len, loff_t *off)
{
	s32 ret = 0;
	struct goodix_ts_data *ts = i2c_get_clientdata(gt_client);

	ret = copy_from_user(&cmd_head, buff, min(CMD_HEAD_LENGTH, len));
	if (ret) {
		TP_LOGE("copy_from_user failed.\n");
		return -EPERM;
	}

	TP_LOGD("[Operation]wr: %02X\n", cmd_head.wr);
	TP_LOGD("[Flag]flag: %02X,addr: %02X%02X,value: %02X,relation: %02X\n",
		cmd_head.flag, cmd_head.flag_addr[0],
		cmd_head.flag_addr[1], cmd_head.flag_val,
		cmd_head.flag_relation);
	TP_LOGD("[Retry]circle: %d,times: %d,retry: %d, delay: %d\n",
		(s32)cmd_head.circle,
		(s32)cmd_head.times, (s32)cmd_head.retry,
		(s32)cmd_head.delay);

	if (cmd_head.wr == 1) {
		if (cmd_head.data_len > DATA_LENGTH) {
			TP_LOGE("Tool write failed data too long\n");
			return -EPERM;
		}
		ret = copy_from_user(&cmd_head.data[GTP_ADDR_LENGTH],
				     &buff[CMD_HEAD_LENGTH],
				     cmd_head.data_len);
		if (ret) {
			TP_LOGE("copy_from_user failed.\n");
			return -EPERM;
		}
		memcpy(&cmd_head.data[GTP_ADDR_LENGTH - cmd_head.addr_len],
		       cmd_head.addr, cmd_head.addr_len);

		GTP_DEBUG_ARRAY(cmd_head.data, cmd_head.data_len +
				cmd_head.addr_len);

		if (cmd_head.flag == 1) {
			if (comfirm() == FAIL) {
				TP_LOGE("[WRITE]Comfirm fail!\n");
				return -EPERM;
			}
		} else if (cmd_head.flag == 2) {
			/*Need interrupt!*/
		}
		if (tool_i2c_write(&cmd_head.data[GTP_ADDR_LENGTH -
				   cmd_head.addr_len], cmd_head.data_len +
				   cmd_head.addr_len) <= 0) {
			TP_LOGE("[WRITE]Write data failed!\n");
			return -EPERM;
		}

		GTP_DEBUG_ARRAY(&cmd_head.data[GTP_ADDR_LENGTH -
				cmd_head.addr_len],
				cmd_head.data_len + cmd_head.addr_len);
		if (cmd_head.delay)
			msleep(cmd_head.delay);
	} else if (cmd_head.wr == 3) {
		if (cmd_head.data_len > DATA_LENGTH) {
			TP_LOGE("Tool write failed data too long\n");
			return -EPERM;
		}
		ret = copy_from_user(&cmd_head.data[0], &buff[CMD_HEAD_LENGTH],
				     cmd_head.data_len);
		if (ret) {
			TP_LOGE("copy_from_user failed.\n");
			return -EPERM;
		}
		memcpy(IC_TYPE, cmd_head.data, cmd_head.data_len);

		register_i2c_func();
	} else if (cmd_head.wr == 5) {
		/*memcpy(IC_TYPE, cmd_head.data, cmd_head.data_len);*/
	} else if (cmd_head.wr == 7) {/*disable irq!*/
		gtp_work_control_enable(i2c_get_clientdata(gt_client), false);

		if (ts->pdata->esd_protect)
			gtp_esd_off(ts);
	} else if (cmd_head.wr == 9) {/*enable irq!*/
		gtp_work_control_enable(i2c_get_clientdata(gt_client), true);

		if (ts->pdata->esd_protect)
			gtp_esd_on(ts);
	} else if (cmd_head.wr == 17) {
		if (cmd_head.data_len > DATA_LENGTH) {
			TP_LOGE("Tool write failed data too long\n");
			return -EPERM;
		}
		ret = copy_from_user(&cmd_head.data[GTP_ADDR_LENGTH],
				     &buff[CMD_HEAD_LENGTH],
				     cmd_head.data_len);
		if (ret) {
			TP_LOGD("copy_from_user failed.\n");
			return -EPERM;
		}
		if (cmd_head.data[GTP_ADDR_LENGTH]) {
			TP_LOGD("gtp enter rawdiff.\n");
			set_bit(RAW_DATA_MODE, &ts->flags);
		} else {
			clear_bit(RAW_DATA_MODE, &ts->flags);
			TP_LOGD("gtp leave rawdiff.\n");
		}
	} else if (cmd_head.wr == 19) {
		/* add new command: reset guitar */
		gtp_reset_guitar(gt_client, 20);
	}
#ifdef CONFIG_TOUCHSCREEN_GT9XX_UPDATE
	else if (cmd_head.wr == 11) {/*Enter update mode!*/
		if (gup_enter_update_mode(gt_client) == FAIL)
			return -EPERM;
	} else if (cmd_head.wr == 13) {/*Leave update mode!*/
		gup_leave_update_mode(gt_client);
	} else if (cmd_head.wr == 15) {/*Update firmware!*/
		show_len = 0;
		total_len = 0;
		if (cmd_head.data_len > DATA_LENGTH) {
			TP_LOGE("Tool write failed data too long\n");
			return -EPERM;
		}
		memset(cmd_head.data, 0, DATA_LENGTH);
		ret = copy_from_user(cmd_head.data,
				     &buff[CMD_HEAD_LENGTH],
				     min((unsigned long)cmd_head.data_len,
				     (len - CMD_HEAD_LENGTH)));
		if (ret) {
			TP_LOGD("copy_from_user failed.\n");
			return -EPERM;
		}

		if (gup_update_proc((void *)cmd_head.data) == FAIL)
			return -EPERM;
	}
#endif

	return len;
}

/*******************************************************
 * Function:
 *	Goodix tool read function.
 * Input:
 *	standard proc read function param.
 * Output:
 *	Return read length.
 ********************************************************/
ssize_t goodix_tool_read(struct file *file, char __user *page,
			 size_t size, loff_t *ppos)
{
	s32 ret = 0;

	if (*ppos) {
		/* ADB call again
		 * TP_LOGD("[HEAD]wr: %d\n", cmd_head.wr);
		 * TP_LOGD(
		 * "[PARAM]size: %d, *ppos: %d\n", size, (int)*ppos);
		 * TP_LOGD(
		 * "[TOOL_READ]ADB call again, return it.\n");
		 */
		*ppos = 0;
		return 0;
	}

	if (cmd_head.wr % 2) {
		return -EPERM;
	} else if (!cmd_head.wr) {
		u16 len, data_len, loc, addr;

		if (cmd_head.flag == 1) {
			if (comfirm() == FAIL) {
				TP_LOGE("[READ]Comfirm fail!\n");
				return -EPERM;
			}
		} else if (cmd_head.flag == 2) {
			/*Need interrupt!*/
		}

		if (cmd_head.delay)
			msleep(cmd_head.delay);

		data_len = cmd_head.data_len;
		addr = (cmd_head.addr[0] << 8) + cmd_head.addr[1];
		loc = 0;

		while (data_len > 0) {
			len = data_len > DATA_LENGTH ? DATA_LENGTH : data_len;
			cmd_head.data[0] = (addr >> 8) & 0xFF;
			cmd_head.data[1] = (addr & 0xFF);
			if (tool_i2c_read(cmd_head.data, len) <= 0) {
				TP_LOGE("[READ]Read data failed!\n");
				return -EPERM;
			}
			ret = simple_read_from_buffer(&page[loc], size, ppos,
					&cmd_head.data[GTP_ADDR_LENGTH], len);
			if (ret < 0)
				return ret;
			loc += len;
			addr += len;
			data_len -= len;
		}
		return cmd_head.data_len;
	} else if (cmd_head.wr == 2) {
		ret = simple_read_from_buffer(page, size, ppos,
					      IC_TYPE, sizeof(IC_TYPE));
		return ret;
	}
#ifdef CONFIG_TOUCHSCREEN_GT9XX_UPDATE
	else if (cmd_head.wr == 4) {
		u8 progress_buf[4];

		progress_buf[0] = show_len >> 8;
		progress_buf[1] = show_len & 0xff;
		progress_buf[2] = total_len >> 8;
		progress_buf[3] = total_len & 0xff;

		ret = simple_read_from_buffer(page, size, ppos,
					      progress_buf, 4);
		return ret;
	}
#endif
	else if (cmd_head.wr == 6) {
		/*Read error code!*/
	} else if (cmd_head.wr == 8) {	/*Read driver version*/
		ret = simple_read_from_buffer(page, size, ppos,
					      GTP_DRIVER_VERSION,
					      strlen(GTP_DRIVER_VERSION));
		return ret;
	}

	return -EPERM;
}
