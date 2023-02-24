/******************************************************************************
 * Copyright 2020 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 *
 *         file: motor_hades_comms.h
 *  description: Motor hades device communication header file
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 ******************************************************************************
 */

#pragma once

/* Includes ------------------------------------------------------------------- */
#include <linux/types.h>
#include <linux/device.h>
#include "hds_common.h"
#include "hds_host.h"

typedef enum {
   COMP_RECEIVED = 0,
   COMP_TIMEOUT
} CompletionStatus;

typedef enum spi_status {
    SPI_OK = 0,
    SPI_ERROR = 1,
} SPIStatus;

struct hds_completion {
    unsigned long flag;
    wait_queue_head_t wait_queue;
};

struct hds_data {
    struct hds_completion cts;
    int response_timeout_us;
    HDS_HOST_HANDLE hds;
    struct mutex transfer_lock;
};

/* Definitions ---------------------------------------------------------------- */
/* Hades */
#define HADES_POST_SOF_DELAY_US     50
#define HADES_POST_EOF_DELAY_US     1000


/* Function prototypes -------------------------------------------------------- */
/**
 *  \brief      PHY layer start-of-frame handler
 *  \param[IN]  context custom context parameter (struct device pointer)
 */
void Motor_Hades_SOF(void *context);

/**
 *  \brief      PHY layer end-of-frame handler
 *  \param[IN]  context custom context parameter (struct device pointer)
 */
void Motor_Hades_EOF(void *context);

/**
 *  \brief      PHY layer BLOCKING write handler
 *  \param[IN]  context custom context parameter (struct device pointer)
 *  \param[IN]  buf write buffer
 *  \param[IN]  len length of buffer
 */
hds_phy_status_t Motor_Hades_PHY_Write(void          *context,
				       const uint8_t *buf,
				       size_t         len);

/**
 *  \brief      PHY layer BLOCKING read handler
 *  \param[IN]  context custom context parameter (struct device pointer)
 *  \param[IN]  buf read buffer
 *  \param[IN]  len length of buffer
 */
hds_phy_status_t Motor_Hades_PHY_Read(void    *context,
				      uint8_t *buf,
				      size_t   len);

/**
 *  \brief      PHY layer blocking handler to read the CTS state
 *  \param[IN]  context custom context parameter (struct device pointer)
 */
hds_phy_cts_t Motor_Hades_CTS_Read(void *context);

/**
 *  \brief      Write to motor MCU
 *  \param[IN]  hds_data pointer
 *  \param[IN]  cmd command ID
 *  \param[IN]  buf write buffer
 *  \param[IN]  len bytes to write (length of write buffer)
 */
hds_retcode_t Motor_Write(struct hds_data *hd,
			  uint16_t         cmd,
			  const uint8_t   *buf,
			  uint16_t         len,
			  hds_tl_status_t *tl_status);
/**
 *  \brief      Read from motor MCU
 *  \param[IN]  hds_data pointer
 *  \param[IN]  cmd command ID
 *  \param[IN]  buf read buffer
 *  \param[OUT] len the length read from motor MCU
 *  \param[IN]  max_len maximum length allowed to read
 */
hds_retcode_t Motor_Read(struct hds_data *hd,
			 uint16_t         cmd,
			 uint8_t         *buf,
			 uint16_t        *len,
			 uint16_t         max_len,
			 hds_tl_status_t *tl_status);

uint32_t motion_motor_comms_checksum(const uint8_t *buf, size_t len);

void InitCompletion(struct hds_completion *hds_completion);

void SignalCompletion(struct hds_completion *hds_completion);

CompletionStatus WaitForCompletionTimeout(struct hds_completion *hds_completion, int timeout_us);
