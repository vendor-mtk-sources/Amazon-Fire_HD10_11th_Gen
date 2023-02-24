/*
 * cli.c
 *
 * cli management for all kinds of modules on all DSP platforms
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

#define TAG "ADF_CLI"

DEFINE_MUTEX(adfCliLock);

/*
 * adfCli_send()
 * ----------------------------------------
 * send cli cmd to cli buffer
 *
 * Input:
 *   void *cliAddr      - address of cli buffer
 *   void *inBuf        - address of input buffer
 *   int32_t inBufLen   - length of input buffer
 * Return:
 *   int32_t
 */
int32_t adfCli_send(void *cliAddr, const char *inBuf, int32_t inBufLen)
{
	int i = 0;
	adfRingbuf_t *rbuf = NULL;
	char *paramP = NULL;
	long paramLen = 0;

	rbuf = (adfRingbuf_t *)cliAddr;
	if (rbuf) {
		mutex_lock(&adfCliLock);
		adfRingbuf_reset(rbuf);
		rbuf->u.cli.argc = 0;
		rbuf->u.cli.rc = 0;
		paramP = (char *)inBuf;
		for (i = 0; i < inBufLen; i++) {
			if ((*(inBuf + i) == ' ') || (*(inBuf + i) == '\t')
			|| (*(inBuf + i) == '\r') || (*(inBuf + i) == '\n')) {
				paramLen = (uintptr_t)(inBuf + i) - (uintptr_t)(paramP);
				if (paramLen > 0) {
					adfRingbuf_write(rbuf, paramP, (uint32_t)paramLen);
					adfRingbuf_write(rbuf, "\0", 1);
					rbuf->u.cli.argc += 1;
					paramP = (char *)inBuf + i + 1;
				}
			}
		}
		if (rbuf->u.cli.argc > 0)
			rbuf->u.cli.rc = 1;
		mutex_unlock(&adfCliLock);
	}
	return inBufLen;
}
