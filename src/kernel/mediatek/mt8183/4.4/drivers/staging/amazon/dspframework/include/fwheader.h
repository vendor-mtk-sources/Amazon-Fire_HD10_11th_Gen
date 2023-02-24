/*
 * fwheader.h
 *
 * the fw header for all DSP platforms, used by host code
 *
 * Copyright 2020-2021 Amazon Technologies, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#ifndef _ADF_FWHEADER_H_
#define _ADF_FWHEADER_H_

#ifdef __cplusplus
extern "C" {
#endif

/* AMZN */
#define ADF_FW_MAGIC         ((uint32_t)0x4E5A4D41)

/* the FW header is 0x80 + secInfos */
#define ADF_FW_HDRLEN        0x80

/* the size is 0x20, reserved may be used as 32/64-bit pointer */
typedef struct {
	uint8_t name[4];
	uint32_t addr;
	uint32_t size;
	uint32_t offset;
	uint32_t *data;
	/* 2 for 64 bit, 3 for 32 bit */
	uint32_t reserved[2];
} adfFwHdr_secInfo_t;

typedef struct {
	/* 0x00~0x20: basic info */
	uint32_t magic;
	uint32_t seccnt;
	uint32_t version[2];
	uint8_t chip[16];
	/* 0x20~0x80: reserved */
	uint32_t reserved[24];
	/* 0x80~: section info, buf or bin */
	adfFwHdr_secInfo_t secarray[];
} adfFwHdr_t;

/* AMZN */
#define ADSP_FW_MAGIC        ((uint32_t)0x414D5A4E)

/* the FW header is 0x80 + binInfos */
#define ADSP_FW_HDRLEN       0x80

/* struct defined for AMZN header and DSP FW */
typedef enum {
	DSPBIN_NORMAL = 0,
	DSPBIN_MODEL
} adfFwHdr_binType_t;

typedef struct {
	uint32_t offset;
	uint32_t size;
	uint32_t addr;
	uint32_t flag;
} adfFwHdr_binInfo_t;

typedef struct {
	/* 0x00~0x20: basic info */
	uint32_t magic;
	uint32_t bincnt;
	uint32_t version[2];
	uint8_t chip[16];
	/* 0x20~0x50: buf addr/len info */
	uint32_t capAddr;
	uint32_t capLen;
	uint32_t plyAddr;
	uint32_t plyLen;
	uint32_t staAddr;
	uint32_t staLen;
	uint32_t cliAddr;
	uint32_t cliLen;
	uint32_t logAddr;
	uint32_t logLen;
	uint32_t ipcAddr;
	uint32_t ipcLen;
	/* 0x50~0x80: reserved */
	uint32_t reserved[12];
	/* binary info array */
	adfFwHdr_binInfo_t binarray[];
} adfFwHdr_old_t;

#ifdef __cplusplus
}
#endif

#endif  /* _ADF_FWHEADER_H_ */
