/*
 * log.c
 *
 * log management for all kinds of modules on all DSP platforms
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

#define TAG "ADF_LOG"

DEFINE_MUTEX(adfLogLock);

typedef void* (*adf_log_fliter) (char *logLine);

/*
 * adfLog_calWaterMark()
 * ----------------------------------------
 * caculate adf ring buffer water mark
 *
 * Input:
 *   adfRingbuf_t *rbuf      - input adf ring buffer
 * Return:
 *   none
 */
static void adfLog_calWaterMark(adfRingbuf_t *rbuf)
{
	char check = 0;

	ADF_LOG_I(TAG, "Calculate water marker of data!\n");
	if (rbuf->wp < rbuf->limit) {
		rbuf->rp = rbuf->wp + 1;
		adfRingbuf_read(rbuf, &check, 1);
		if (check == 0)
			rbuf->rp = rbuf->base;
	} else
		rbuf->rp = rbuf->base;
}

/*
 * adfLog_dump()
 * ----------------------------------------
 * Dump log to debugfs file
 *
 * Input:
 *   void *logAddr           - address of log buffer
 *   struct seq_file *file   - debugfs file
 * Return:
 *   int
 */
int adfLog_dump(void *logAddr, struct seq_file *file)
{
	adfRingbuf_t *rbuf = NULL;
	adfRingbuf_t rbufDSP;
	int32_t remain = 0;
	char *logBuf = NULL;

	if (logAddr == NULL || file == NULL) {
		ADF_LOG_E(TAG, "Invalid param logaddr is %p, file is %p\n", logAddr, file);
		return -EINVAL;
	}
	mutex_lock(&adfLogLock);
	rbuf = ((adfRingbuf_t *) logAddr);
	/* back up the log ringbuf, copy it back later */
	/* to avoid influence to dsp side when operate share memory directly */
	memcpy(&rbufDSP, rbuf, sizeof(adfRingbuf_t));
	/* if dsp have uart, it will keep reading data from dsp side, ringbuf size is 0 */
	/* host side need find water mark of data used to be stored in ringbuf */
	if (adfRingbuf_getUsedSize(rbuf) == 0)
		adfLog_calWaterMark(rbuf);
	remain = adfRingbuf_getUsedSize(rbuf);
	if (remain > 0) {
		/* dump to debugfs file */
		/* read all log to tmp buffer */
		logBuf = (char *)kmalloc(remain + 1, GFP_KERNEL);
		if (logBuf == NULL) {
			pr_err("Failed to allocate kernel memory for log buffer\n");
			return -ENOMEM;
		}
		memset(logBuf, 0, remain + 1);
		adfRingbuf_read(rbuf, logBuf, remain);
		seq_printf(file, "%s", logBuf);
		kfree(logBuf);
	}
	/* dump should not change the content of rbuf */
	memcpy(rbuf, &rbufDSP, sizeof(adfRingbuf_t));
	mutex_unlock(&adfLogLock);
	return 0;
}

/*
 * adfLog_print()
 * ----------------------------------------
 * print all dsp log to kernel msg
 *
 * Input:
 *   void *logAddr      - address of log buffer
 * Return:
 *   int
 */
int adfLog_print(void *logAddr)
{
	adfRingbuf_t *rbuf = NULL;
	adfRingbuf_t rbufDSP;
	int32_t remain = 0;
	char *logBuf = NULL;
	char *logStr = NULL;
	char *logLine = NULL;
	char seps[] = "\r\n";

	if (logAddr == NULL) {
		pr_err("Invalid param logaddr is %p\n", logAddr);
		return -EINVAL;
	}
	mutex_lock(&adfLogLock);
	rbuf = (adfRingbuf_t *) logAddr;
	/* back up the log ringbuf, copy it back later */
	/* to avoid influence to dsp side when operate share memory directly */
	memcpy(&rbufDSP, rbuf, sizeof(adfRingbuf_t));
	/* if dsp have uart, it will keep reading data from dsp side, ringbuf size is 0 */
	/* host side need find water mark of data used to be stored in ringbuf */
	if (adfRingbuf_getUsedSize(rbuf) == 0)
		adfLog_calWaterMark(rbuf);
	remain = adfRingbuf_getUsedSize(rbuf);
	if (remain > 0) {
		/* read all log to tmp buffer */
		logBuf = (char *)kmalloc(remain + 1, GFP_KERNEL);
		if (logBuf == NULL) {
			pr_err("Failed to allocate kernel memory for log buffer\n");
			return -ENOMEM;
		}
		memset(logBuf, 0, remain + 1);
		adfRingbuf_read(rbuf, logBuf, remain);
		/* split log to line & print to kernel log */
		logStr = logBuf;
		logLine = strsep(&logStr, seps);
		while (logLine != NULL) {
			if (*logLine != '\0')
				pr_info("%s\n", logLine);
			logLine = strsep(&logStr, seps);
		}
		kfree(logBuf);
	}
	/* print should not change the content of rbuf */
	memcpy(rbuf, &rbufDSP, sizeof(adfRingbuf_t));
	mutex_unlock(&adfLogLock);
	return 0;
}

/*
 * adfLog_query()
 * ----------------------------------------
 * Query log to get useful information
 *
 * Input:
 *   void *logAddr           - address of log buffer
 *   void *logFliter         - adf log filter func ptr
 * Return:
 *   void* info get from dsp log
 */
void *adfLog_query(void *logAddr, void *logFliter)
{
	adfRingbuf_t *rbuf = NULL;
	adfRingbuf_t rbufDSP;
	int32_t remain = 0;
	char *logBuf = NULL;
	char *logStr = NULL;
	char *logLine = NULL;
	char seps[] = "\r\n";
	adf_log_fliter fliter = (adf_log_fliter)logFliter;
	void *ret = NULL;

	if (logAddr == NULL) {
		pr_err("Invalid param logaddr is %p\n", logAddr);
		return ret;
	}
	mutex_lock(&adfLogLock);
	rbuf = (adfRingbuf_t *)logAddr;
	/* back up the log ringbuf, copy it back later */
	/* to avoid influence to dsp side when operate share memory directly */
	memcpy(&rbufDSP, rbuf, sizeof(adfRingbuf_t));
	/* if dsp have uart, it will keep reading data from dsp side, ringbuf size is 0 */
	/* host side need find water mark of data used to be stored in ringbuf */
	if (adfRingbuf_getUsedSize(rbuf) == 0)
		adfLog_calWaterMark(rbuf);
	remain = adfRingbuf_getUsedSize(rbuf);
	if (remain > 0) {
		/* read all log to tmp buffer */
		logBuf = (char *)kmalloc(remain + 1, GFP_KERNEL);
		if (logBuf == NULL) {
			pr_err("Failed to allocate kernel memory for log buffer\n");
			return ret;
		}
		memset(logBuf, 0, remain + 1);
		adfRingbuf_read(rbuf, logBuf, remain);
		/* split log to line & send to fliter */
		logStr = logBuf;
		logLine = strsep(&logStr, seps);
		while (logLine != NULL) {
			if (*logLine != '\0')
				ret = fliter(logLine);
			logLine = strsep(&logStr, seps);
		}
		kfree(logBuf);
	}
	/* query should not change the content of rbuf */
	memcpy(rbuf, &rbufDSP, sizeof(adfRingbuf_t));
	mutex_unlock(&adfLogLock);
	return ret;
}
