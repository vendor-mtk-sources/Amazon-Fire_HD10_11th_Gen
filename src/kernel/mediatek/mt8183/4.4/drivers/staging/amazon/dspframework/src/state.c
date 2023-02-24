/*
 * state.c
 *
 * the state machine management for all DSP platforms
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

#define TAG "ADF_STA"

/* adf state is the first 4 bytes of staData,
 * and we will another reserve 12 bytes for 16-byte-align
 */
static volatile adfState_t *adfState;
static volatile adfState_t *adfStateLast;
static volatile uint32_t *adfStateOpt;

static spinlock_t adfStateLock = __SPIN_LOCK_UNLOCKED(adfStateLock);

/*
 * _adfState_addrConv()
 * ----------------------------------------
 * convert shared mem address to state values
 *
 * Input:
 *   void *addr        - state memory address
 * Return:
 *   int32_t
 */
static int32_t _adfState_addrConv(void *addr)
{
	int32_t retVal = 0;
	if (addr) {
		adfState = (volatile adfState_t *)addr;
		adfStateLast = (volatile adfState_t *)(addr + sizeof(uint32_t));
		adfStateOpt = (volatile uint32_t *)(addr + 2 * sizeof(uint32_t));
	} else {
		adfState = NULL;
		adfStateLast = NULL;
		adfStateOpt = NULL;
		retVal = -1;
	}
	return retVal;
}

/*
 * adfState_getLast()
 * ----------------------------------------
 * get the last state of the DSP, check DSP last status
 *
 * Input:
 *   void *addr        - state memory address
 * Return:
 *   last state of DSP
 */
adfState_t adfState_getLast(void *addr)
{
	adfState_t dspState = ADF_STATE_DEF;

	spin_lock(&adfStateLock);
	if (_adfState_addrConv(addr) >= 0)
		dspState = *adfStateLast;
	spin_unlock(&adfStateLock);
	return dspState;
}

/*
 * adfState_get()
 * ----------------------------------------
 * get the state of the DSP, check DSP status
 *
 * Input:
 *   void* addr        - state memory address
 * Return:
 *   the current state of DSP
 */
adfState_t adfState_get(void *addr)
{
	adfState_t dspState = ADF_STATE_DEF;

	spin_lock(&adfStateLock);
	if (_adfState_addrConv(addr) >= 0)
		dspState = *adfState;
	spin_unlock(&adfStateLock);
	return dspState;
}

/*
 * adfState_set()
 * ----------------------------------------
 * set the state of the DSP, set DSP status
 *
 * Input:
 *   void *addr        - state memory address
 *   adfState_t state  - the new state of DSP
 * Return:
 *   None
 */
void adfState_set(void *addr, adfState_t state)
{
	spin_lock(&adfStateLock);
	if (_adfState_addrConv(addr) >= 0) {
		if (*adfState != state) {
			*adfStateLast = *adfState;
			*adfState = state;
		}
	}
	spin_unlock(&adfStateLock);
}

/*
 * adfState_getOpt()
 * ----------------------------------------
 * get the special flags (opt) of the DSP
 *
 * Input:
 *   void* addr        - state memory address
 * Return:
 *   the option of the DSP state
 */
uint32_t adfState_getOpt(void *addr)
{
	uint32_t dspStateopt = ADF_OPT_NONE;

	spin_lock(&adfStateLock);
	if (_adfState_addrConv(addr) >= 0)
		dspStateopt = *adfStateOpt;
	spin_unlock(&adfStateLock);
	return dspStateopt;
}

/*
 * adfState_setOpt()
 * ----------------------------------------
 * set the special flags (opt) of the DSP
 *
 * Input:
 *   void* addr        - state memory address
 *   uint32_t opt      - the option ID of DSP
 * Return:
 *   None
 */
void adfState_setOpt(void *addr, uint32_t opt)
{
	spin_lock(&adfStateLock);
	if (_adfState_addrConv(addr) >= 0)
		*adfStateOpt = opt;
	spin_unlock(&adfStateLock);
}
