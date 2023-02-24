/******************************************************************************
 * Copyright 2020 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 *
 *         file: hds_host.h
 *  description: Hades (HDS) host communication protocol framework
 *               header file
 *      details: The framework handles all the protocol logic. It requires
 *               the user supply the PHY-level functions and invoke specific
 *               callbacks (e.g. PHY read/write completion) when appropriate.
 *
 *               For more details, please visit the abc123 wiki under
 *               Software -> Mobility Platform -> Robotics Platform Softare ->
 *               Inter-MCU Communication Protocol -> Hades
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

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------- */
#include "hds_common.h"

/* Struct/Enums/Typedefs ------------------------------------------------------ */
/**
 *  \brief      Opaque struct definition for the hades host comms framework
 */
typedef void *HDS_HOST_HANDLE;

/**
 *  \brief      Hades host communication stats
 */
typedef struct {
    size_t      num_chksum_errors;  /* /< Number of checksum errors */
    uint32_t    err_recvd_chksum;   /* /< Last error (received checksum) */
    uint32_t    err_calc_chksum;    /* /< Last error (expected checksum) */
    size_t      num_timeout_errors; /* /< Number of timeout errors */
} hds_host_stats_t;

/**
 *  \brief      PHY layer blocking or non-blocking write handler
 *  \note       Use the `is_phy_blocking` flag to indicate whether the provided
 *              handler is blocking (true) or non-blocking (false)
 *  \param[IN]  context custom context parameter
 *  \param[IN]  buf data buffer to write
 *  \param[IN]  len number of bytes to write
 *  \return     HDS_PHY_STATUS_OK if successful, HDS_PHY_STATUS_ERROR otherwise
 */
typedef hds_phy_status_t (*hds_phy_write)(void          *context,
					  const uint8_t *buf,
					  size_t         len);

/**
 *  \brief      PHY layer blocking or non-blocking read handler
 *  \note       Use the `is_phy_blocking` flag to indicate whether the provided
 *              handler is blocking (true) or non-blocking (false)
 *  \param[IN]  context custom context parameter
 *  \param[OUT] buf data buffer to store read data
 *  \param[IN]  len number of bytes to read
 *  \return     HDS_PHY_STATUS_OK if successful, HDS_PHY_STATUS_ERROR otherwise
 */
typedef hds_phy_status_t (*hds_phy_read)(void       *context,
					 uint8_t    *buf,
					 size_t      len);

/**
 *  \brief      PHY layer blocking handler to read the CTS state
 *  \note       User determines how to handle timeouts
 *  \param[IN]  context custom context parameter
 *  \return     HDS_PHY_CTS_ENABLE, HDS_PHY_CTS_DISABLE, or HDS_PHY_CTS_TIMEOUT
 */
typedef hds_phy_cts_t (*hds_phy_cts_read)(void *context);

/**
 *  \brief      PHY layer start-of-frame handler
 *  \details    This should handle the triggering of a PHY layer start of
 *              frame. For example, hades-over-SPI requires chip-select to
 *              be asserted (driven low).
 *  \note       It is up to the user to handle the appropriate delay, which is
 *              device-specific, after triggering an SOF
 *  \param[IN]  context custom context parameter
 */
typedef void (*hds_phy_sof)(void *context);

/**
 *  \brief      PHY layer end-of-frame handler
 *  \details    This should handle the triggering of a PHY layer end of
 *              frame. For example, hades-over-SPI requires chip-select to
 *              be deasserted (driven high).
 *  \note       It is up to the user to handle the appropriate delay, which is
 *              device-specific, after triggering an EOF
 *  \param[IN]  context custom context parameter
 */
typedef void (*hds_phy_eof)(void *context);

/**
 *  \brief      Calculate a 32-bit checksum number
 *  \details    The checksum algorithm is application-dependent and is up to
 *              the user to decide on what algorithm to use
 *  \note       Unless bounded by a hardware-specific implementation, it is
 *              recommended that the function provided is shared code between
 *              the host implementation and the device implementation.
 *  \param[IN]  buf data buffer to run CRC calculation
 *  \param[IN]  len length of buffer in bytes
 *  \return     32-bit checksum result
 */
typedef uint32_t (*hds_ll_checksum)(const uint8_t *buf, size_t len);

/**
 *  \brief      Data structure for user-supplied hades host framework handlers
 */
typedef struct {
    bool                is_phy_blocking; /* /< PHY read/write is blocking flag */
    hds_phy_write       phy_write;       /* /< PHY write handler */
    hds_phy_read        phy_read;        /* /< PHY read handler */
    hds_phy_cts_read    phy_cts_read;    /* /< PHY CTS state read handler */
    hds_phy_sof         phy_sof;         /* /< PHY trigger start-of-frame handler */
    hds_phy_eof         phy_eof;         /* /< PHY trigger end-of-frame handler */
    hds_ll_checksum     ll_chksum;       /* /< Link layer checksum calculation handler */
} hds_host_handlers_t;


/* Function prototypes -------------------------------------------------------- */
/* ***** Init/denit API calls ***** */
/**
 *  \brief      Initialize the hades host handle
 *  \param[OUT] handle hades host handle
 *  \param[IN]  phy_context custom PHY context parameter (may be NULL), which
 *              is passed into the user-provided handlers
 *  \param[IN]  handlers struct of user-defined handlers
 *  \param[IN]  max_data_bytes the maximum transport layer payload size
 *                             expected which must be a value between 1 and
 *                             65531
 *  \return     HDS_RETCODE_SUCCESS if successful, HDS_RETCODE_FAILURE
 *              otherwise
 */
hds_retcode_t hds_host_init(HDS_HOST_HANDLE *handle,
			    void                *phy_context,
			    hds_host_handlers_t *handlers,
			    uint16_t             max_data_bytes);

/**
 *  \brief      Deinitialize the hades host handle
 *  \param[IN]  handle hades host handle
 *  \return     HDS_RETCODE_SUCCESS if successful, HDS_RETCODE_FAILURE
 *              otherwise
 */
hds_retcode_t hds_host_deinit(HDS_HOST_HANDLE handle);


/* ***** Physical Layer API calls ***** */
/**
 *  \brief      Signal that the physical layer write has completed
 *  \details    This should be called in the system's write done callback. If
 *              the PHY read/write handlers are blocking calls, this should not
 *              be called. User is responsible for handling timeouts for
 *              non-blocking writes by calling this function appropriately.
 *  \param[IN]  handle hades host handle
 *  \param[IN]  status HDS_PHY_STATUS_OK if successful, otherwise, user may
 *              provide HDS_PHY_STATUS_ERROR or HDS_PHY_STATUS_TIMEOUT
 */
void hds_host_phy_write_done(HDS_HOST_HANDLE handle, hds_phy_status_t status);

/**
 *  \brief      Signal that the physical layer read has completed
 *  \details    This should be called in the system's read done callback. If
 *              the PHY read/write handlers are blocking calls, this should not
 *              be called. User is responsible for handling timeouts for
 *              non-blocking reads by calling this function appropriately.
 *  \param[IN]  handle hades host handle
 *  \param[IN]  status HDS_PHY_STATUS_OK if successful, otherwise, user may
 *              provide HDS_PHY_STATUS_ERROR or HDS_PHY_STATUS_TIMEOUT
 */
void hds_host_phy_read_done(HDS_HOST_HANDLE handle, hds_phy_status_t status);


/* ***** Hades Framework API calls ***** */
/**
 *  \brief      Hades host-to-device write (blocking)
 *  \param[IN]  handle hades host handle
 *  \param[IN]  channel the channel ID
 *  \param[IN]  buf data buffer to write
 *  \param[IN]  len number of bytes to write
 *  \param[OUT] status the transport layer return status
 *  \return     Hades library return code
 */
hds_retcode_t hds_host_write(HDS_HOST_HANDLE    handle,
			     uint16_t           channel,
			     const uint8_t     *buf,
			     uint16_t           len,
			     hds_tl_status_t   *status);

/**
 *  \brief      Hades host-to-device read (blocking)
 *  \param[IN]  handle hades host handle
 *  \param[IN]  channel the channel ID
 *  \param[OUT] buf data buffer to store read data
 *  \param[OUT] len number of bytes read
 *  \param[IN]  max_len maximum number of bytes allowed
 *  \param[OUT] status the transport layer return status
 *  \return     Hades library return code
 */
hds_retcode_t hds_host_read(HDS_HOST_HANDLE     handle,
			    uint16_t            channel,
			    uint8_t            *buf,
			    uint16_t           *len,
			    uint16_t            max_len,
			    hds_tl_status_t    *status);


/* ***** Stats API calls ***** */
/**
 *  \brief      Get current hades comms stats
 *  \param[IN]  handle hades host handle
 *  \param[OUT] stats stats on the hades host handle
 */
void hds_host_get_stats(HDS_HOST_HANDLE   handle,
			hds_host_stats_t *stats);


#ifdef __cplusplus
} /* extern "C" */
#endif

