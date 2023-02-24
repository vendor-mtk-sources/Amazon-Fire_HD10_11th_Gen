/*
 * load.h
 *
 * The ADF FW loading feature with AMZN common ADF format (header).
 *
 * Copyright 2020-2021 Amazon Technologies, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#ifndef _ADF_LOAD_H_
#define _ADF_LOAD_H_

#include "fwheader.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	adfFwHdr_t *dspHeader;         /* dsp header info */
	struct mutex adfDspLock;       /* mutex to protect dsp private data */
} adfDspPriv_t;

typedef bool (*adfLoad_binCpy)(void *dest, void *src, size_t len);
typedef bool (*adfLoad_binRead)(void *src, void *dest, size_t len);

adfDspPriv_t *adfLoad_getDspPriv(uint8_t coreNo);
adfFwHdr_secInfo_t *adfLoad_GetSecInfo(adfFwHdr_t *dspHeader, char *secName);
bool adfLoad_checkMagic(const uint8_t *data);
int32_t adfLoad_init(void *src, void *dst, uint32_t size, adfLoad_binCpy binCpy, uint8_t coreNo);
bool adfLoad_initCheck(void *src, void *dst, adfLoad_binRead binRead);
bool adspLoad_checkMagic(const uint8_t *data);
int32_t adspLoad_init(void *src, void *dst, uint32_t size, adfLoad_binCpy binCpy);

#ifdef __cplusplus
}
#endif

#endif /* _ADF_LOAD_H_ */
