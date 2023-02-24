/*
 * adf_debug.c
 *
 * debugfs include operate of log/cli/state
 *
 * Copyright 2020-2021 Amazon Technologies, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#include "adf/adf_status.h"
#include "adf/adf_common.h"


#ifndef DSP_CORE_NUM
#define DSP_CORE_NUM (1)
#endif

typedef int (*adf_mem_cpy) (uintptr_t, uintptr_t, int);

static adf_mem_cpy adf_dbg_mem_read;
static adf_mem_cpy adf_dbg_mem_write;
static int adfDspCoreNo[DSP_CORE_NUM];

static int adf_debug_show(struct seq_file *file, void *data)
{
	int ret = 0;
	adfFwHdr_secInfo_t *logInfo;
	adfDspPriv_t *adfDspPriv = NULL;
	int *coreNo = file->private;

	if (!coreNo)
		return -EINVAL;

	adfDspPriv = adfLoad_getDspPriv(*coreNo);
	if (adfDspPriv) {
		mutex_lock(&adfDspPriv->adfDspLock);
		if (adfDspPriv->dspHeader == NULL)
			seq_printf(file, "Warning! invalid dspHeader\n");
		else if (adfDspPriv->dspHeader->magic == ADF_FW_MAGIC) {
			logInfo = adfLoad_GetSecInfo(adfDspPriv->dspHeader, "LOG");
			if (logInfo) {
				memset(logInfo->data, 0, logInfo->size);
				ret = adf_dbg_mem_read((uintptr_t)logInfo->addr, (uintptr_t)logInfo->data, logInfo->size);
				if (!ret)
					seq_printf(file, "Warning! Failed to read logData!\n");
				else
					adfLog_dump(logInfo->data, file);
			}
		}
		mutex_unlock(&adfDspPriv->adfDspLock);
	}
	return 0;
}

static int adf_debug_open(struct inode *inode, struct file *file)
{
	return single_open(file, adf_debug_show, inode->i_private);
}

static ssize_t adf_debug_write(struct file *file, const char __user *userbuf,
				size_t count, loff_t *ppos)
{
	int err;
	adfFwHdr_secInfo_t *staInfo;
	adfFwHdr_secInfo_t *cliInfo;
	adfDspPriv_t *adfDspPriv = NULL;
	char *buf = NULL;
	struct seq_file *seqFile = file->private_data;
	int *coreNo = seqFile->private;

	if (!coreNo)
		return -EINVAL;

	buf = kmalloc(count + 1, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;
	memset(buf, 0, count + 1);

	if (copy_from_user(buf, userbuf, count)) {
		kfree(buf);
		return -EFAULT;
	}

	adfDspPriv = adfLoad_getDspPriv(*coreNo);
	if (adfDspPriv) {
		mutex_lock(&adfDspPriv->adfDspLock);
		if (adfDspPriv->dspHeader == NULL)
			pr_err("Warning! invalid dspHeader???\n");
		else if (adfDspPriv->dspHeader->magic == ADF_FW_MAGIC) {
			staInfo = adfLoad_GetSecInfo(adfDspPriv->dspHeader, "STA");
			cliInfo = adfLoad_GetSecInfo(adfDspPriv->dspHeader, "CLI");

			/* we have to transfer the sta/cli data and header individually as the lower layer may get the data before it is fully sent */
			if ((adf_dbg_mem_read((uintptr_t)staInfo->addr, (uintptr_t)staInfo->data, staInfo->size) == 0) ||
				(adf_dbg_mem_read((uintptr_t)cliInfo->addr, (uintptr_t)cliInfo->data, cliInfo->size) == 0)) {
				pr_err("%s read staData/cliData failed! count %lx\n", __func__, (long unsigned int)count);
				mutex_unlock(&adfDspPriv->adfDspLock);
				kfree(buf);
				return count;
			}
			/* always send cli cmd to dsp no matter which state it is */
			adfCli_send(cliInfo->data, (char *)buf, count);
			err = adf_dbg_mem_write((uintptr_t)cliInfo->addr, (uintptr_t)cliInfo->data, cliInfo->size);
			if (err)
				pr_err("%s write cliData failed! count %lx\n", __func__, (long unsigned int)count);
		}
		mutex_unlock(&adfDspPriv->adfDspLock);
	}
	kfree(buf);
	return count;
}

static struct file_operations adfDebugFops = {
	.owner = THIS_MODULE,
	.open = adf_debug_open,
	.read = seq_read,
	.write = adf_debug_write,
};

static int adf_state_show(struct seq_file *file, void *data)
{
	int ret = 0;
	adfState_t dspState = ADF_STATE_DEF;
	adfFwHdr_secInfo_t *staInfo = NULL;
	adfDspPriv_t *adfDspPriv = NULL;
	int *coreNo = file->private;

	if (!coreNo)
		return -EINVAL;

	adfDspPriv = adfLoad_getDspPriv(*coreNo);
	if (adfDspPriv) {
		mutex_lock(&adfDspPriv->adfDspLock);
		if (adfDspPriv->dspHeader == NULL)
			seq_printf(file, "Warning! invalid dspHeader\n");
		else if (adfDspPriv->dspHeader->magic == ADF_FW_MAGIC) {
			staInfo = adfLoad_GetSecInfo(adfDspPriv->dspHeader, "STA");
			if (staInfo) {
				ret  = adf_dbg_mem_read((uintptr_t)staInfo->addr, (uintptr_t)staInfo->data, staInfo->size);
				if (ret) {
					dspState = adfState_get(staInfo->data);
					seq_printf(file, "%02x\n", dspState);
				} else
					seq_printf(file, "Warning! failed to read state from dsp!\n");
			}
		}
		mutex_unlock(&adfDspPriv->adfDspLock);
	}
	return 0;
}

static int adf_state_open(struct inode *inode, struct file *file)
{
	return single_open(file, adf_state_show, inode->i_private);
}

static ssize_t adf_state_write(struct file *file, const char __user *userbuf,
				size_t count, loff_t *ppos)
{
	int ret = 0;
	adfFwHdr_secInfo_t *staInfo = NULL;
	adfDspPriv_t *adfDspPriv = NULL;
	char *buf = NULL;
	struct seq_file *seqFile = file->private_data;
	int *coreNo = seqFile->private;
	uint8_t sta;

	if (!coreNo)
		return -EINVAL;

	buf = kmalloc(count + 1, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;
	memset(buf, 0, count + 1);

	if (copy_from_user(buf, userbuf, count)) {
		kfree(buf);
		return -EFAULT;
	}

	adfDspPriv = adfLoad_getDspPriv(*coreNo);
	if (adfDspPriv) {
		mutex_lock(&adfDspPriv->adfDspLock);
		if (adfDspPriv->dspHeader == NULL)
			pr_err("Warning! invalid dspHeader???\n");
		else if (adfDspPriv->dspHeader->magic == ADF_FW_MAGIC) {
			staInfo = adfLoad_GetSecInfo(adfDspPriv->dspHeader, "STA");
			if (staInfo == NULL) {
				mutex_unlock(&adfDspPriv->adfDspLock);
				kfree(buf);
				return count;
			}
			if (kstrtou8(buf, 16, &sta) == 0) {
				ret = adf_dbg_mem_read((uintptr_t)staInfo->addr, (uintptr_t)staInfo->data, staInfo->size);
				if (!ret) {
					mutex_unlock(&adfDspPriv->adfDspLock);
					kfree(buf);
					return count;
				}
				adfState_set(staInfo->data, sta);
				ret = adf_dbg_mem_write((uintptr_t)staInfo->addr, (uintptr_t)staInfo->data, staInfo->size);
				if (ret)
					pr_err("Warning! state write to dsp fail!\n");
				else
					pr_info("Set dsp state to 0x%02x\n", sta);
			} else {
				pr_err("Warning! invalid state input!\n");
			}
		}
		mutex_unlock(&adfDspPriv->adfDspLock);
	}
	kfree(buf);
	return count;
}

static struct file_operations adfStateFops = {
	.owner = THIS_MODULE,
	.open = adf_state_open,
	.read = seq_read,
	.write = adf_state_write,
};

int adf_debug_fs_init(void *debugReadFunc, void *debugWriteFunc)
{
	struct dentry *adfDebugDir = NULL;
	struct dentry *adfDebugFile = NULL;
	char debugFileName[12] = {0};
	int i = 0;

	if (debugReadFunc == NULL || debugWriteFunc == NULL) {
		pr_err("Invalid param read func %p, write func %p\n", debugReadFunc, debugWriteFunc);
		return -EINVAL;
	}
	adf_dbg_mem_read = debugReadFunc;
	adf_dbg_mem_write = debugWriteFunc;

	if (DSP_CORE_NUM >= 10 || DSP_CORE_NUM <= 0) {
		pr_err("Invalid param dsp core num %d\n", DSP_CORE_NUM);
		return -EINVAL;
	}

	adfDebugDir = debugfs_create_dir("adf_dbg_fs", NULL);
	if (!adfDebugDir)
		return -ENOMEM;

	for (i = 0; i < DSP_CORE_NUM; i++) {
		adfDspCoreNo[i] = i;
		snprintf(debugFileName, sizeof(debugFileName), "adf_debug_%d", i);
		adfDebugFile = debugfs_create_file(debugFileName,
						0644,
						adfDebugDir,
						&adfDspCoreNo[i],
						&adfDebugFops);
		if (!adfDebugFile)
				goto fail;

		snprintf(debugFileName, sizeof(debugFileName), "adf_state_%d", i);
		adfDebugFile = debugfs_create_file(debugFileName,
							0644,
							adfDebugDir,
							&adfDspCoreNo[i],
							&adfStateFops);
		if (!adfDebugFile)
				goto fail;
	}
	return 0;

fail:
	debugfs_remove_recursive(adfDebugDir);
	return -ENOMEM;
}

void *adf_debug_query(uint8_t coreNo, char **cliCmds, uint8_t cliCmdNum, void *logFliter)
{
	adfFwHdr_secInfo_t *staInfo = NULL;
	adfFwHdr_secInfo_t *cliInfo = NULL;
	adfFwHdr_secInfo_t *logInfo = NULL;
	adfState_t dspState = ADF_STATE_DEF;
	adfDspPriv_t *adfDspPriv = NULL;
	void *ret = NULL;
	int i = 0;

	adfDspPriv = adfLoad_getDspPriv(coreNo);
	if (adfDspPriv) {
		mutex_lock(&adfDspPriv->adfDspLock);
		if (adfDspPriv->dspHeader == NULL)
			pr_err("Warning! invalid dspHeader %s, %d\n", __FILE__, __LINE__);
		else if (adfDspPriv->dspHeader->magic == ADF_FW_MAGIC) {
			/* send cli command to trigger */
			staInfo = adfLoad_GetSecInfo(adfDspPriv->dspHeader, "STA");
			cliInfo = adfLoad_GetSecInfo(adfDspPriv->dspHeader, "CLI");
			logInfo = adfLoad_GetSecInfo(adfDspPriv->dspHeader, "LOG");
			if (staInfo == NULL || cliInfo == NULL || logInfo == NULL) {
				mutex_unlock(&adfDspPriv->adfDspLock);
				return ret;
			}

			if (adf_dbg_mem_read((uintptr_t)staInfo->addr, (uintptr_t)staInfo->data, staInfo->size) != 0)
				dspState = adfState_get(staInfo->data);

			if (dspState < ADF_STATE_RUN || dspState >= ADF_STATE_ERR) {
				pr_err("Warning! invalid ADF state 0x%02x\n", dspState);
				mutex_unlock(&adfDspPriv->adfDspLock);
				return ret;
			}

			if (adf_dbg_mem_read((uintptr_t)cliInfo->addr, (uintptr_t)cliInfo->data, cliInfo->size) != 0) {

				for (i = 0; i < cliCmdNum; i++) {
					adfCli_send(cliInfo->data, *(cliCmds + i), strlen(*(cliCmds + i)));
					adf_dbg_mem_write((uintptr_t)cliInfo->addr, (uintptr_t)cliInfo->data, cliInfo->size);
					/* sleep instead of check ack to aviod dsp crash or not run cause kernel hang here */
					msleep (100);
				}
			} else
				pr_err("Warning! can't send adf cli command\n");

			memset(logInfo->data, 0, logInfo->size);
			if (adf_dbg_mem_read((uintptr_t)logInfo->addr, (uintptr_t)logInfo->data, logInfo->size) != 0) {
				ret = adfLog_query(logInfo->data, logFliter);
			} else
				pr_err("Warning! can't read adf log\n");

		}
		mutex_unlock(&adfDspPriv->adfDspLock);
	}
	return ret;
}
