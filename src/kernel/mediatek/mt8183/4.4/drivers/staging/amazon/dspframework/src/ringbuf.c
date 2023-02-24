/*
 * ringbuf.c
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
#include "adf/adf_status.h"
#include "adf/adf_common.h"

/*
 * ADF_RINGBUF_MOVE_PTR()
 * ----------------------------------------
 * get the remaining bytes in the ring buffer,
 * return 0 if the ring buffer is empty or invalid
 * noted, rewrite as macro function to speed up execution
 * as inline does not work sometimes
 *
 * Noted, ptr/base/limit can be either char* or uint32_t
 *
 * Input:
 *   char *ptr         - the original pointer to do the movement
 *   int32_t offset    - the movement offset
 *   char *base        - the start address of the ring buffer
 *   char *limit       - the end address of the ring buffer
 * Return:
 *   the moved pointer
 */
#define ADF_RINGBUF_MOVE_PTR(ptr, offset, base, limit) \
	(((limit) - (ptr) > (offset)) ? \
	 ((ptr) + (offset)) : ((ptr) + (offset) - ((limit) - (base))))

/*
 * ADF_RINGBUF_IN_RANGE()
 * ----------------------------------------
 * check whether the given pointer is in the range of the rBuf
 *
 * Noted, ptr/base/limit can be either char* or uint32_t
 *
 * Input:
 *   char *ptr         - the pointer to check the range
 *   char *base        - the start address of the ring buffer
 *   char *limit       - the end address of the ring buffer
 * Return:
 *   true or false
 */
#define ADF_RINGBUF_IN_RANGE(ptr, base, limit) \
	(((ptr) >= (base)) && ((ptr) < (limit)))

/*
 * ADF_RINGBUF_CALC_RW_SIZE()
 * ADF_RINGBUF_CALC_WR_SIZE()
 * ----------------------------------------
 * calculate the buffer size between the start and the end pointer,
 * can be used to calculate free size or used size
 * note that, considering the empty and full issue,
 * so in WR_SIZE, we need to use < but not <=
 *
 * Input:
 *   char *ptrS        - the start pointer for the size calculation
 *   char *ptrE        - the end pointer for the size calculation
 *   char *base        - the start address of the ring buffer
 *   char *limit       - the end address of the ring buffer
 * Return:
 *   the size between start and end point in the ring buffer
 */
#define ADF_RINGBUF_CALC_RW_SIZE(ptrS, ptrE, base, limit) \
	(((ptrS) <= (ptrE)) ? \
	 ((ptrE) - (ptrS)) :  \
	 (((limit) - (ptrS)) + ((ptrE) - (base))))
#define ADF_RINGBUF_CALC_WR_SIZE(ptrS, ptrE, base, limit) \
	(((ptrS) < (ptrE)) ?  \
	 ((ptrE) - (ptrS)) :  \
	 (((limit) - (ptrS)) + ((ptrE) - (base))))

/*
 * _adfRingbuf_checkDrop()
 * ----------------------------------------
 * check the free size and do drop if necessary.
 * this func will be called during write operation
 *
 * Input:
 *   adfRingbuf_t *rBuf - the pointer to the ring buffer struct
 *   int32_t len        - the expected data size
 * Return:
 *   the valid length after dropped
 */
static int32_t _adfRingbuf_checkDrop(adfRingbuf_t *rBuf, int32_t len)
{
	int32_t available;
	int32_t dropLen;

	available = ADF_RINGBUF_CALC_WR_SIZE(rBuf->wp, rBuf->rp,
		rBuf->base, rBuf->limit);
	if (len >= available) {
		if (rBuf->dropunit > 0) {
			/* drop 1 more dropunit at max to prevent
			 * the potential frequent buffer full case
			 */
			dropLen = len - available + rBuf->dropunit - 1;
			dropLen = dropLen - (dropLen % rBuf->dropunit);
			rBuf->drop += dropLen;
			rBuf->rp = ADF_RINGBUF_MOVE_PTR(rBuf->rp, dropLen,
				rBuf->base, rBuf->limit);
		} else {
			/* wp should not catch-up rp even though buffer is full,
			 * so reserve 4 bytes at least
			 */
			len = available - sizeof(int32_t);
		}
	}
	return len;
}

/*
 * adfRingbuf_checkValid()
 * ----------------------------------------
 * check whether the target ringBuf is valid (initialized)
 *
 * Input:
 *   adfRingbuf_t* rBuf - the pointer to the ring buffer struct
 *   void* base         - the base address of the buffer
 *   int32_t size       - the size of the buffer
 * Return:
 *   the remaining data size in the ring buffer
 */
bool adfRingbuf_checkValid(adfRingbuf_t *rBuf, void *buf, int32_t size)
{
	if ((ADF_RINGBUF_GET_PARAM(rBuf, base) != (char *)buf) ||
		(rBuf->limit - rBuf->base != size))
		return false;
	if (ADF_RINGBUF_IN_RANGE(rBuf->rp, rBuf->base, rBuf->limit) &&
		ADF_RINGBUF_IN_RANGE(rBuf->wp, rBuf->base, rBuf->limit))
		return true;
	return false;
}

/*
 * adfRingbuf_getUsedSize()
 * ----------------------------------------
 * get the remaining bytes in the ring buffer,
 * return 0 if the ring buffer is empty or invalid
 *
 * Input:
 *   adfRingbuf_t *rBuf - the pointer to the ring buffer struct
 * Return:
 *   the remaining data size in the ring buffer
 */
int32_t adfRingbuf_getUsedSize(adfRingbuf_t *rBuf)
{
	if (rBuf == NULL)
		return 0;
	return ADF_RINGBUF_CALC_RW_SIZE(rBuf->rp, rBuf->wp,
		rBuf->base, rBuf->limit);
}

/*
 * adfRingbuf_getFreeSize()
 * ----------------------------------------
 * get the available bytes in the ring buffer,
 * return 0 if the ring buffer is full or invalid
 *
 * Input:
 *   adfRingbuf_t *rBuf - the pointer to the ring buffer struct
 * Return:
 *   the remaining data size in the ring buffer
 */
int32_t adfRingbuf_getFreeSize(adfRingbuf_t *rBuf)
{
	if (rBuf == NULL)
		return 0;
	return ADF_RINGBUF_CALC_WR_SIZE(rBuf->wp, rBuf->rp, rBuf->base, rBuf->limit);
}

/*
 * adfRingbuf_read()
 * ----------------------------------------
 * read data from the ring buffer,
 * note that the rp will not be moved!
 * you have to call flush after read operation.
 * if the remaining data is less than the expected length,
 * then return all the remaining data.
 *
 * Input:
 *   adfRingbuf_t *rBuf - the pointer to the ring buffer struct
 *   char *data         - the pointer to the read data buffer
 *   int32_t len        - the expected read data size
 * Return:
 *   the real read data size, may be different to expected read size
 */
int32_t adfRingbuf_read(adfRingbuf_t *rBuf, char *data, int32_t len)
{
	int32_t remain;
	int32_t lenToEnd;
	char *pBase;
	char *pRp;

	if ((rBuf == NULL) || (data == NULL) || (len == 0))
		return 0;
	remain = ADF_RINGBUF_CALC_RW_SIZE(rBuf->rp, rBuf->wp,
		rBuf->base, rBuf->limit);
	if (len > remain)
		len = remain;
	lenToEnd = rBuf->limit - rBuf->rp;
	pBase = ADF_RINGBUF_GET_PARAM(rBuf, base);
	pRp = ADF_RINGBUF_GET_PARAM(rBuf, rp);

	if (lenToEnd >= len)
		memcpy(data, pRp, len);
	else {
		memcpy(data, pRp, lenToEnd);
		memcpy(&data[lenToEnd], pBase, len - lenToEnd);
	}

	return len;
}

/*
 * adfRingbuf_flush()
 * ----------------------------------------
 * flush data from the ring buffer,
 * if the remaining data is less than the expected length,
 * then flush all the remaining data.
 *
 * Input:
 *   adfRingbuf_t *rBuf - the pointer to the ring buffer struct
 *   int32_t len        - the expected flush size
 * Return:
 *   the real flushed size, should be always equal to the expected flush size
 */
int32_t adfRingbuf_flush(adfRingbuf_t *rBuf, int32_t len)
{
	int32_t remain;

	if ((rBuf == NULL) || (len == 0))
		return 0;
	remain = ADF_RINGBUF_CALC_RW_SIZE(rBuf->rp, rBuf->wp,
		rBuf->base, rBuf->limit);
	if (len > remain)
		len = remain;
	rBuf->rp = ADF_RINGBUF_MOVE_PTR(rBuf->rp, len, rBuf->base, rBuf->limit);

	return len;
}

/*
 * adfRingbuf_reserve()
 * ----------------------------------------
 * reserve data in the ring buffer,
 * if the free data is less than the expected length,
 * then reserve all the data.
 * Noted, reserve is just an opposite operation of flush
 *
 * Input:
 *   adfRingbuf_t *rBuf - the pointer to the ring buffer struct
 *   int32_t len        - the expected reserve size
 * Return:
 *   the real flushed size, should be always equal to the expected flush size
 */
int32_t adfRingbuf_reserve(adfRingbuf_t *rBuf, int32_t len)
{
	int32_t available;

	if ((rBuf == NULL) || (len == 0))
		return 0;
	available = ADF_RINGBUF_CALC_WR_SIZE(rBuf->wp, rBuf->rp,
		rBuf->base, rBuf->limit);
	if (len >= available)
		len = available - (available % rBuf->dropunit);
	rBuf->rp = ADF_RINGBUF_MOVE_PTR(rBuf->rp, rBuf->limit - rBuf->base - len,
		rBuf->base, rBuf->limit);

	return len;
}

/*
 * adfRingbuf_write()
 * ----------------------------------------
 * write data to the ring buffer,
 * if the remaining space is less than the expected length,
 * then flush dpsz bytes old data and accumulate it into dropped size count.
 *
 * Input:
 *   adfRingbuf_t *rBuf - the pointer to the ring buffer struct
 *   const char *data   - the pointer to the write data buffer
 *   int32_t len        - the expected write data size
 * Return:
 *   the real write data size, may be different to expected write size
 */
int32_t adfRingbuf_write(adfRingbuf_t *rBuf, const char *data, int32_t len)
{
	int32_t lenToEnd;
	char *pBase;
	char *pWp;

	if ((rBuf == NULL) || (data == NULL) || (len == 0))
		return 0;
	len = _adfRingbuf_checkDrop(rBuf, len);
	lenToEnd = rBuf->limit - rBuf->wp;
	pBase = ADF_RINGBUF_GET_PARAM(rBuf, base);
	pWp = ADF_RINGBUF_GET_PARAM(rBuf, wp);

	if (lenToEnd >= len)
		memcpy(pWp, data, len);
	else {
		memcpy(pWp, data, lenToEnd);
		memcpy(pBase, &data[lenToEnd], len - lenToEnd);
	}
	rBuf->wp = ADF_RINGBUF_MOVE_PTR(rBuf->wp, len, rBuf->base, rBuf->limit);

	return len;
}

/*
 * adfRingbuf_add()
 * ----------------------------------------
 * move the wp to add data into the ring buffer.
 * This API is primarily for the HW ringbuf,
 * we need to sync the WP from HW register to SW pointer
 * Note that, the overlap rule is different because this is for HW ringbuf!!!
 * The write data cannot be cutted!!!
 *
 * Input:
 *   adfRingbuf_t *rBuf - the pointer to the ring buffer struct
 *   int32_t len        - the expected add data size
 * Return:
 *   the real added size, should be always equal to the expected add size
 */
int32_t adfRingbuf_add(adfRingbuf_t *rBuf, int32_t len)
{
	if ((rBuf == NULL) || (len == 0))
		return 0;
	len = _adfRingbuf_checkDrop(rBuf, len);
	rBuf->wp = ADF_RINGBUF_MOVE_PTR(rBuf->wp, len, rBuf->base, rBuf->limit);

	return len;
}

/*
 * adfRingbuf_apply()
 * ----------------------------------------
 * apply a new wp, actually this function is similar to adfRingbuf_add(),
 * it will not actually write any data into the ring buffer,
 * it just handle the pointer.
 * different to copy the pointer directly,
 * we will think about the drop case here.
 *
 * Input:
 *   adfRingbuf_t *rBuf - the pointer to the ring buffer struct
 *   char *ptr          - the new wp going to be applied
 * Return:
 *   the real added size, should be always equal to the expected add size
 */
int32_t adfRingbuf_apply(adfRingbuf_t *rBuf, char *ptr)
{
	int32_t len;
#if (CFG_RINGBUF_USE_PTR == 1)
	char *new = ptr;
#else
	int32_t new = ptr - (char *)rBuf;
#endif

	if ((rBuf == NULL) || (new == rBuf->wp) ||
		!ADF_RINGBUF_IN_RANGE(new, rBuf->base, rBuf->limit))
		return 0;
	len = ADF_RINGBUF_CALC_RW_SIZE(rBuf->wp, new, rBuf->base, rBuf->limit);
	len = _adfRingbuf_checkDrop(rBuf, len);
	rBuf->wp = ADF_RINGBUF_MOVE_PTR(rBuf->wp, len, rBuf->base, rBuf->limit);

	return len;
}

/*
 * adfRingbuf_reset()
 * ----------------------------------------
 * reset the ring buffer, reset the rp/wp/drop and clean the whole buffer
 * note that, we will not do memset here
 *
 * Input:
 *   adfRingbuf_t *rBuf - the pointer to the ring buffer struct
 * Return:
 *   int32_t
 */
int32_t adfRingbuf_reset(adfRingbuf_t *rBuf)
{
	/* if ring buffer is valid, then do reset */
#if (CFG_RINGBUF_USE_PTR == 1)
	if ((rBuf == NULL) || (rBuf->base == NULL) || (rBuf->limit == NULL))
		return ADF_STATUS_NOT_SUPPORTED;
#else
	if ((rBuf == NULL) || (rBuf->base == 0) || (rBuf->limit == 0))
		return ADF_STATUS_NOT_SUPPORTED;
#endif

	rBuf->rp = rBuf->base;
	rBuf->wp = rBuf->base;
	rBuf->drop = 0;
	return ADF_STATUS_OK;
}

/*
 * adfRingbuf_init()
 * ----------------------------------------
 * init the ring buffer
 * note that, we will not do memset here
 *
 * Input:
 *   adfRingbuf_t *rBuf - the pointer to the ring buffer struct
 *   void *base         - the base address of the buffer
 *   int32_t size       - the size of the buffer
 *   int32_t dropunit   - the drop unit size when wp is overlapped rp,
 *                        dropunit == 0 means never drop!
 * Return:
 *   int32_t
 */
int32_t adfRingbuf_init(adfRingbuf_t *rBuf, void *buf,
	int32_t size, int32_t dropunit)
{
	/* confirm the buffer base and size are valid */
	if ((rBuf == NULL) || (buf == NULL) || (size == 0) || ((size & 0x3) != 0))
		return ADF_STATUS_NOT_SUPPORTED;

	/* init the whole log buffer and reset the wp/rp/dropunit */
#if (CFG_RINGBUF_USE_PTR == 1)
	rBuf->base = (char *)buf;
#else
	rBuf->base = (char *)buf - (char *)rBuf;
#endif
	rBuf->limit = rBuf->base + size;
	rBuf->dropunit = dropunit;
	memset(rBuf->u.rsvd, 0, sizeof(rBuf->u.rsvd));
	return adfRingbuf_reset(rBuf);
}

/*
 * adfRingbuf_deinit()
 * ----------------------------------------
 * destroy the ring buffer, but we won't free any buffer here
 *
 * Input:
 *   adfRingbuf_t *rBuf - the pointer to the ring buffer struct
 * Return:
 *   int32_t
 */
int32_t adfRingbuf_deinit(adfRingbuf_t *rBuf)
{
	/* confirm the ringbuf is valid */
	if (rBuf == NULL)
		return ADF_STATUS_NOT_SUPPORTED;
	/* we don't have to reset the ringBuf parameters here! */
	/* memset(rBuf, 0, sizeof(adfRingbuf_t)); */
	return ADF_STATUS_OK;
}
