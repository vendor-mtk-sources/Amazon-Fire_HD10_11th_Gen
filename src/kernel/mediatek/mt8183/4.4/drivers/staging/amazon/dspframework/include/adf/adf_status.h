/*
 * adf_status.h
 *
 * The definition of ADF status return code.
 *
 * Copyright 2020-2021 Amazon Technologies, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#ifndef _ADF_STATUS_H_
#define _ADF_STATUS_H_

/* we just reserve the OK/General_Err here */
typedef enum {
	/**
	 * @name General
	 * @brief Basic status returns.
	 *        Reserved value[0 to -99]
	 * @{
	 */

	/** Operation completed successfully */
	ADF_STATUS_OK = 0,
	/** Unspecified run-time error */
	ADF_STATUS_GENERAL_ERROR = -1,
	ADF_STATUS_NOT_SUPPORTED = -6,

	/* @} */
} adf_status_t;

#endif /* _ADF_STATUS_H_ */
