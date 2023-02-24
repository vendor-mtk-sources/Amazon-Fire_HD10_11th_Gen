/*
 * state.h
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
#ifndef _ADF_STATE_H_
#define _ADF_STATE_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
	ADF_STATE_DEF  = 0x00,
	ADF_STATE_INIT = 0x10,
	ADF_STATE_RUN  = 0x20,
	ADF_STATE_LPWR = 0x40,
	ADF_STATE_VAD  = 0x41,
	ADF_STATE_ERR  = 0xF0
} adfState_t;

/* the mask of the main/sub state */
#define ADF_STATE_MAIN_MASK         (0xF0)
#define ADF_STATE_SUB_MASK          (0x0F)

/* wakeword magic number */
#define ADF_OPT_WW_MAGIC            (0xCAFEBEEF)
#define ADF_OPT_NONE                (0x00000000)

adfState_t adfState_getLast(void *addr);
adfState_t adfState_get(void *addr);
void adfState_set(void *addr, adfState_t state);
uint32_t adfState_getOpt(void *addr);
void adfState_setOpt(void *addr, uint32_t opt);

#ifdef __cplusplus
}
#endif

#endif  /* _ADF_STATE_H_ */
