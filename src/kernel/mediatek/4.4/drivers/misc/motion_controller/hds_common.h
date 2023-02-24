/******************************************************************************
 * Copyright 2020 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 *
 *         file: hds_common.h
 *  description: Hades (HDS) communication protocol common header file
 *      details: For more details, please visit the abc123 wiki under
 *               Software -> Mobility Platform -> Robotics Platform Software ->
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
#include <linux/types.h>

/* Definitions ---------------------------------------------------------------- */
/* ***** Link Layer ***** */
#define HDS_LL_CHKSUM_SIZE_B        (sizeof(uint32_t))

/* ***** Transport layer ***** */
#define HDS_MAX_TL_PAYLOAD_B        ((uint16_t)65531)


/* Struct/Enums/Typedefs ------------------------------------------------------ */
/* ***** General ***** */
/**
 *  \brief      Hades library return codes
 */
typedef enum {
    HDS_RETCODE_SUCCESS,        /* /< Success */
    HDS_RETCODE_FAILURE,        /* /< Failure */
    HDS_RETCODE_BAD_PARAMS,     /* /< Bad parameters */
    HDS_RETCODE_BAD_RESPONSE,   /* /< Bad response from device */
    HDS_RETCODE_BAD_CHKSUM,     /* /< Bad checksum */
    HDS_RETCODE_PHY_ERROR,      /* /< PHY layer error */
    HDS_RETCODE_TIMEOUT,        /* /< Timeout */
    HDS_RETCODE_NO_DATA,        /* /< No data available */
    HDS_RETCODE_TL_FAILURE,     /* /< Transaction layer failure */
} hds_retcode_t;


/* ***** Physical Layer Enumerations ***** */
/**
 *  \brief      Hades PHY layer status codes
 */
typedef enum {
    HDS_PHY_STATUS_OK,      /* /< PHY handler returned with no errors */
    HDS_PHY_STATUS_ERROR,   /* /< PHY handler returned with errors */
    HDS_PHY_STATUS_BUSY,    /* /< PHY handler is busy */
    HDS_PHY_STATUS_TIMEOUT, /* /< PHY handler timed out */
} hds_phy_status_t;

/**
 *  \brief      Hades PHY layer CTS (clear-to-send) states
 */
typedef enum {
    HDS_PHY_CTS_DISABLE,    /* /< Disabled (no data to send to host) */
    HDS_PHY_CTS_ENABLE,     /* /< Enabled (data to send to host) */
    HDS_PHY_CTS_TIMEOUT,    /* /< Timeout (device failed to respond) */
} hds_phy_cts_t;


/* ***** Link Layer Enumerations ***** */
/**
 *  \brief      Hades link layer subframe types
 */
enum {
    HDS_LL_SUBFRAME_REQUEST         = 0x00, /* /< See wiki for details */
    HDS_LL_SUBFRAME_RESPONSE        = 0x01, /* /< See wiki for details */
    HDS_LL_SUBFRAME_DATA            = 0x02, /* /< Unused; see wiki for details */
};

/**
 *  \brief      Hades link layer direction types
 */
enum {
    HDS_LL_DIRECTION_GET            = 0x00, /* /< Send data from device to host */
    HDS_LL_DIRECTION_SET            = 0x01, /* /< Send data from host to device */
};

/**
 *  \brief      Hades link layer status byte definitions
 *  \note       Status is sent from device back to host during a response
 *              subframe
 */
enum {
    HDS_LL_STATUS_OK                = 0x00, /* /< Subframe received with no errors */
    HDS_LL_STATUS_BAD_CHECKSUM      = 0x01, /* /< Bad checksum received */
    HDS_LL_STATUS_INVALID_REQUEST   = 0x02, /* /< Bad request subframe received */
    HDS_LL_STATUS_INVALID_RESPONSE  = 0x03, /* /< Bad response subframe received */
    HDS_LL_STATUS_TIMEOUT           = 0x04, /* /< Subframe timeout */
};
typedef uint8_t hds_ll_status_t;


/* ***** Transport Layer Enumerations ***** */
/**
 *  \brief      Hades transport layer direction types
 */
enum {
    HDS_TL_DIRECTION_READ           = 0x00, /* /< Read request from host */
    HDS_TL_DIRECTION_WRITE          = 0x01, /* /< Write request from host */
};

/**
 *  \brief      Hades transport layer status byte definitions
 */
enum {
    HDS_TL_STATUS_OK                          = 0x00, /* /< No errors from host read/write */
    HDS_TL_STATUS_NO_DATA                     = 0x01, /* /< No data for host to read */
    HDS_TL_STATUS_ACCESS_DENIED               = 0x02, /* /< MCU read or write not allowed */
    HDS_TL_STATUS_WRITE_FAILED                = 0x03, /* /< MCU write failed */
    HDS_TL_STATUS_INVALID                     = 0x04, /* /< Invalid parameter */
    HDS_TL_STATUS_NOT_HOMING                  = 0x05, /* /< MCU not homing'ed */
    HDS_TL_STATUS_STALL                       = 0x06, /* /< MCU in stall status */
    HDS_TL_STATUS_ABC                         = 0x07, /* /< MCU in active backdrive status */
    HDS_TL_STATUS_SHUTDOWN                    = 0x08, /* /< MCU in shutdown status */
    HDS_TL_STATUS_BAD_NORM_TBL                = 0x09, /* /< Bad noramalization table */
    HDS_TL_STATUS_BAD_OTA_HOMING_TBL          = 0x0A, /* /< Bad ota homing table */
    HDS_TL_STATUS_NORM_TBL_NOT_AVAIL          = 0x0B, /* /< Norm table is not available */
    HDS_TL_STATUS_OTA_HOMING_TBL_NOT_AVAIL    = 0x0C, /* /< Ota homing table is not available */
    HDS_TL_STATUS_NO_RESPONSE                 = 0xFF, /* /< No response from host/device */
};
typedef uint8_t hds_tl_status_t;


/* ***** Transport Layer Header ***** */
/**
 *  \brief      Hades transport layer header
 */
typedef struct __attribute__((packed))
{
    uint16_t            channel;        /* /< Channel ID */
    uint16_t            length;         /* /< Application payload length */
    uint8_t             direction;      /* /< Read/Write */
    hds_tl_status_t     tl_status;      /* /< Transport layer status byte */
} hds_tl_header_t;


/* ***** Link Layer Header ***** */
/**
 *  \brief      Hades link layer header
 */
typedef struct __attribute__((packed))
{
    uint8_t             subframe_type;  /* /< Request or Response */
    uint8_t             direction;      /* /< GET or SET */
    uint16_t            length;         /* /< Data subframe length (with checksum) */
    hds_ll_status_t     ll_status;      /* /< Link layer status byte */
} hds_ll_header_t;


/* ***** Link Layer Data Structures ***** */
/**
 *  \brief      Hades link layer request subframe
 */
typedef struct __attribute__((packed))
{
    hds_ll_header_t     ll_header;      /* /< Link layer header */
    hds_tl_header_t     tl_header;      /* /< Transport layer header */
    uint32_t            chksum;         /* /< Link layer checksum */
} hds_ll_request_t;

/**
 *  \brief      Hades link layer response subframe
 */
typedef struct __attribute__((packed))
{
    hds_ll_header_t     ll_header;      /* /< Link layer header */
    hds_tl_header_t     tl_header;      /* /< Transport layer header */
    uint32_t            chksum;         /* /< Link layer checksum */
} hds_ll_response_t;


#ifdef __cplusplus
} /* extern "C" */
#endif

