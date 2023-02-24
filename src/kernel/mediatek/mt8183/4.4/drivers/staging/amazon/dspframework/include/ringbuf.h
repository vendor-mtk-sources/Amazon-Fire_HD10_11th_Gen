/*
 * ringbuf.h
 *
 * the ring buffer management for all kinds of modules on all DSP platforms
 *
 * Copyright 2020-2021 Amazon Technologies, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#ifndef _ADF_RINGBUF_H_
#define _ADF_RINGBUF_H_

#ifdef __cplusplus
extern "C" {
#endif

#ifndef CFG_RINGBUF_USE_PTR
#define CFG_RINGBUF_USE_PTR  0
#endif

/*
 * ADF_RINGBUF_GET_PARAM()
 * ----------------------------------------
 * get the parameter of the ring buffer,
 * note that we may use ptr or offset for base/limit/wp/rp
 *
 * Input:
 *   adfRingbuf_t *rBuf - the pointer to the ring buffer struct
 *   m                  - the member (param) of the struct to be get
 * Return:
 *   the parameter of the ring buffer
 */
#if (CFG_RINGBUF_USE_PTR == 1)
#define ADF_RINGBUF_GET_PARAM(rBuf, m)   ((char *)(rBuf)->m)
#else
#define ADF_RINGBUF_GET_PARAM(rBuf, m)   ((char *)(rBuf) + (rBuf)->m)
#endif

/* noted, considering the efficiency, if we do not use pointer,
 * then wp/rp is not offset based on "base", it's the offset based on "struct"!
 * moreover, we need to use int32_t but not uint32_t because
 * the struct may be behind the buffer!
 */
typedef struct {
	volatile int32_t argc;         /* the argument count, serve for CLI */
	volatile int32_t rc;           /* >0 means valid cli coming,<=0 is the return value of the cli cmd */
} adfRingbuf_cli_t;

typedef struct {
	volatile uint32_t bootReason;  /* the boot reason, serve for LOG */
	uint32_t rsvd;
} adfRingbuf_log_t;

typedef struct {
#if (CFG_RINGBUF_USE_PTR == 1)
	char *base;                    /* the start address of the buffer */
	char *limit;                   /* the end address of the buffer */
	volatile char *wp;             /* the write pointer */
	volatile char *rp;             /* the read pointer */
#else
	int32_t base;                  /* offset from the struct to the buffer */
	int32_t limit;                 /* offset from the struct to the buf end */
	volatile int32_t wp;           /* offset from the struct to the wp */
	volatile int32_t rp;           /* offset from the struct to the rp */
#endif
	int32_t drop;                  /* the dropped bytes */
	int32_t dropunit;              /* the drop size when wp is overlapped rp */
	union {
		uint32_t rsvd[2];          /* keep 16 bytes aligned */
		adfRingbuf_cli_t cli;
		adfRingbuf_log_t log;
	} u;
} adfRingbuf_t;

bool adfRingbuf_checkValid(adfRingbuf_t *rBuf, void *buf, int32_t size);
int32_t adfRingbuf_getUsedSize(adfRingbuf_t *rBuf);
int32_t adfRingbuf_getFreeSize(adfRingbuf_t *rBuf);
int32_t adfRingbuf_read(adfRingbuf_t *rBuf, char *data, int32_t len);
int32_t adfRingbuf_flush(adfRingbuf_t *rBuf, int32_t len);
int32_t adfRingbuf_reserve(adfRingbuf_t *rBuf, int32_t len);
int32_t adfRingbuf_write(adfRingbuf_t *rBuf, const char *data, int32_t len);
int32_t adfRingbuf_add(adfRingbuf_t *rBuf, int32_t len);
int32_t adfRingbuf_apply(adfRingbuf_t *rBuf, char *ptr);
int32_t adfRingbuf_reset(adfRingbuf_t *rBuf);
int32_t adfRingbuf_init(adfRingbuf_t *rBuf, void *buf,
	 int32_t size, int32_t dropunit);
int32_t adfRingbuf_deinit(adfRingbuf_t *rBuf);

#ifdef __cplusplus
}
#endif

#endif /* _ADF_RINGBUF_H_ */
