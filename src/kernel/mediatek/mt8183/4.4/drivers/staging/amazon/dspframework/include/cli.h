/*
 * cli.h
 *
 * the CLI module for all DSP platforms, common code
 *
 * Copyright 2020-2021 Amazon Technologies, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#ifndef _ADF_CLI_H_
#define _ADF_CLI_H_

#ifdef __cplusplus
extern "C" {
#endif

int32_t adfCli_send(void *cliAddr, const char *inBuf, int32_t inBufLen);

#ifdef __cplusplus
}
#endif

#endif  /* _ADF_CLI_H_ */
