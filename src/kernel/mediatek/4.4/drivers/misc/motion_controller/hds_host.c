/******************************************************************************
 * Copyright 2020 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 *
 *         file: hds_host.c
 *  description: Hades (HDS) host communication protocol framework
 *               source file
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

/* Includes ------------------------------------------------------------------- */
#include <linux/slab.h>
#include "fsm.h"        /* Handles the link-layer state machine */
#include "hds_host.h"

/* Struct/Enums/Typedefs ------------------------------------------------------ */
typedef struct {
    /* PHY layer ============================== */
    void                      *phy_context;       /* User-provided PHY context */
    /* Buffers */
    uint8_t                   *request_buf;       /* Request subframe packet buffer */
    uint8_t                   *response_buf;      /* Response subframe packet buffer */
    uint8_t                   *data_buf;          /* Data subframe packet buffer */
    uint16_t                   data_buf_len;      /* Max length of the data buffer */
    /* Flags */
    volatile hds_phy_status_t  phy_read_status;   /* PHY read status flag */
    volatile hds_phy_status_t  phy_write_status;  /* PHY write status flag */

    /* Link Layer ============================= */
    FSM_HANDLE                 ll_fsm;            /* Link layer state machine */

    /* PHY/LL/APP handlers ==================== */
    hds_host_handlers_t        handlers;          /* User-provided PHY handlers and */
						  /* user-specific callbacks */
    uint8_t                   *app_buf;           /* Application read buf */
    uint16_t                   app_len;           /* Application read buf length */
    uint16_t                   app_max_len;       /* Max length allowed for app read buf */
    hds_retcode_t              rc;                /* Last return code */

    /* Comm stats ============================= */
    hds_host_stats_t           stats;             /* Hades comms stats */
} hds_host_t;

enum {
    HDS_LL_STATE_INACTIVE,                  /* Inactive; frame not started */
    HDS_LL_STATE_IDLE,                      /* Idle; SOF sent */
    HDS_LL_STATE_REQ_GET_PENDING,           /* Request GET send pending */
    HDS_LL_STATE_REQ_SET_PENDING,           /* Request SET send pending */
    HDS_LL_STATE_RESP_GET_RECV_PENDING,     /* Response GET receive pending */
    HDS_LL_STATE_RESP_SET_RECV_PENDING,     /* Response SET receive pending */
    HDS_LL_STATE_DATA_RECV_PENDING,         /* Data receive pending */
    HDS_LL_STATE_DATA_SEND_PENDING,         /* Data send pending */

    HDS_LL_NUM_STATES
};

enum {
    HDS_LL_EVENT_SOF,                       /* Start of frame */
    HDS_LL_EVENT_REQ_GET,                   /* Request GET write */
    HDS_LL_EVENT_REQ_SET,                   /* Request SET write */
    HDS_LL_EVENT_CTS_ENABLED,               /* CTS enabled (device ready) */
    HDS_LL_EVENT_TIMEOUT,                   /* CTS timeout detected */
    HDS_LL_EVENT_READ_DONE,                 /* PHY read completed */
    HDS_LL_EVENT_ERROR,                     /* Error occurred */

    HDS_LL_NUM_EVENTS
};


/* Private function prototypes ------------------------------------------------ */
/* ***** General ***** */
static hds_retcode_t priv_hds_host_ll_fsm_init(hds_host_t *hds);

/* ***** FSM Entry/Exit/Transition callbacks ***** */
static void priv_state_inactive_entry_cb(void *context);
static void priv_state_inactive_exit_cb(void *context);

/* ***** Helper functions: BLOCKING ***** */
static hds_retcode_t priv_req_get(hds_host_t   *hds,
				  uint16_t      channel);
static hds_retcode_t priv_req_set_with_data(hds_host_t      *hds,
					    uint16_t         channel,
					    const uint8_t   *buf,
					    uint16_t         len);
static hds_retcode_t priv_read_response(hds_host_t      *hds,
					hds_tl_status_t *status);
static hds_retcode_t priv_check_get_response(hds_host_t *hds);
static hds_retcode_t priv_read_data(hds_host_t *hds, uint8_t *buf);

/* ***** Common helper functions ***** */
static bool priv_validate_response(hds_host_t *hds);
static bool priv_validate_data(hds_host_t *hds);


/* Function definitions ------------------------------------------------------- */
hds_retcode_t hds_host_init(HDS_HOST_HANDLE * handle,
			    void                *phy_context,
			    hds_host_handlers_t *handlers,
			    uint16_t             max_data_bytes)
{
    hds_host_t *hds;

    if (!handle || !handlers)
    {
	return HDS_RETCODE_FAILURE;
    }

    if (!handlers->phy_write || !handlers->phy_read ||
	    !handlers->phy_cts_read || !handlers->phy_sof ||
	    !handlers->phy_eof || !handlers->ll_chksum ||
	    (max_data_bytes == 0) || (max_data_bytes > HDS_MAX_TL_PAYLOAD_B))
    {
	return HDS_RETCODE_FAILURE;
    }

    /* Allocate required dynamic memory for hades device handle */
    hds = kcalloc(1, sizeof(hds_host_t), GFP_KERNEL);
    if (!hds)
    {
	return HDS_RETCODE_FAILURE;
    }

    /* Allocate memory for link layer request buffer */
    hds->request_buf = kmalloc(sizeof(hds_ll_request_t), GFP_KERNEL);
    if (!hds->request_buf)
    {
	goto cleanup;
    }

    /* Allocate memory for link layer response buffer */
    hds->response_buf = kmalloc(sizeof(hds_ll_response_t), GFP_KERNEL);
    if (!hds->response_buf)
    {
	goto cleanup;
    }

    /* Save off the maximum data subframe packet size */
    hds->data_buf_len = (max_data_bytes * sizeof(uint8_t)) + /* Data */
			HDS_LL_CHKSUM_SIZE_B;                /* Checksum */

    /* Allocate memory for link layer data buffer and checksum */
    hds->data_buf = kmalloc(hds->data_buf_len, GFP_KERNEL);
    if (!hds->data_buf)
    {
	goto cleanup;
    }

    /* Create the link layer FSM */
    if (priv_hds_host_ll_fsm_init(hds) != HDS_RETCODE_SUCCESS)
    {
	goto cleanup;
    }

    /* Reset stats */
    memset(&hds->stats, 0, sizeof(hds->stats));

    /* Save the PHY context and is-blocking flag */
    hds->phy_context                = phy_context;
    hds->handlers.is_phy_blocking   = handlers->is_phy_blocking;

    /* Save the user-specific function handlers */
    hds->handlers.phy_write         = handlers->phy_write;
    hds->handlers.phy_read          = handlers->phy_read;
    hds->handlers.phy_cts_read      = handlers->phy_cts_read;
    hds->handlers.phy_sof           = handlers->phy_sof;
    hds->handlers.phy_eof           = handlers->phy_eof;
    hds->handlers.ll_chksum         = handlers->ll_chksum;

    /* Return hades host handle OUT parameter */
    *handle = (hds_host_t *)hds;

    return HDS_RETCODE_SUCCESS;

cleanup:
    hds_host_deinit(hds);
    return HDS_RETCODE_FAILURE;
}


hds_retcode_t hds_host_deinit(HDS_HOST_HANDLE handle)
{
    hds_host_t *hds = (hds_host_t *) handle;

    if (!handle)
    {
	return HDS_RETCODE_FAILURE;
    }

    fsm_deinit(hds->ll_fsm);
    kfree(hds->data_buf);
    kfree(hds->response_buf);
    kfree(hds->request_buf);
    kfree(hds);

    return HDS_RETCODE_SUCCESS;
}


/* ***** Physical Layer API calls ***** */
void hds_host_phy_write_done(HDS_HOST_HANDLE handle, hds_phy_status_t status)
{
    hds_host_t *hds = (hds_host_t *) handle;

    if (!handle)
    {
	return;
    }

    hds->phy_write_status = status;
}

void hds_host_phy_read_done(HDS_HOST_HANDLE handle, hds_phy_status_t status)
{
    int state_num;
    const char *state_name;
    bool read_valid = false;
    hds_host_t *hds = (hds_host_t *)handle;

    if (!handle)
    {
	return;
    }

    /* Save and check the phy read status */
    hds->phy_read_status = status;
    if (hds->phy_read_status != HDS_PHY_STATUS_OK)
    {
	hds->rc = HDS_RETCODE_PHY_ERROR;
	return;
    }

    /* Get current state to determine what type of read event occurred */
    fsm_get_current_state(hds->ll_fsm, &state_num, &state_name);

    /* Response or data read validation */
    switch (state_num)
    {
	case HDS_LL_STATE_RESP_GET_RECV_PENDING:
	    read_valid = priv_validate_response(hds);
	    if (read_valid)
	    {
		hds->rc = priv_check_get_response(hds);
		if (hds->rc != HDS_RETCODE_SUCCESS)
		{
		    read_valid = false;
		}
	    }
	    break;

	case HDS_LL_STATE_RESP_SET_RECV_PENDING:
	    read_valid = priv_validate_response(hds);
	    break;

	case HDS_LL_STATE_DATA_RECV_PENDING:
	    read_valid = priv_validate_data(hds);
	    break;

	default:
	    hds->rc = HDS_RETCODE_FAILURE;
	    break;
    }

    /* Update state machine accordingly */
    if (read_valid)
    {
	fsm_update_with_event(hds->ll_fsm, HDS_LL_EVENT_READ_DONE, handle);
    } else
    {
	fsm_update_with_event(hds->ll_fsm, HDS_LL_EVENT_ERROR, handle);
    }
}


/* ***** Hades Framework API calls ***** */
hds_retcode_t hds_host_write(HDS_HOST_HANDLE    handle,
			     uint16_t           channel,
			     const uint8_t     *buf,
			     uint16_t           len,
			     hds_tl_status_t   *status)
{
    hds_retcode_t rc = HDS_RETCODE_SUCCESS;
    hds_host_t *hds = (hds_host_t *) handle;

    if (!handle || !buf || !status)
    {
	return HDS_RETCODE_FAILURE;
    }

    /* Sanity check: make sure write length won't exceed the internal */
    /* data buf length (which is already properly bounded on init) */
    if (len > (hds->data_buf_len - HDS_LL_CHKSUM_SIZE_B))
    {
	return HDS_RETCODE_BAD_PARAMS;
    }

    /* Start frame */
    fsm_update_with_event(hds->ll_fsm, HDS_LL_EVENT_SOF, handle);

    /* +------+ +--------+ */
    /* | Req  | |  Data  | */
    /* +---+------+-+--------+------------+ */
    /* +------+ */
    /* | Resp | */
    /* +-----------------------+------+---+ */
    /*  */
    /* Send request SET, along with data */
    rc = priv_req_set_with_data(hds, channel, buf, len);
    if (rc != HDS_RETCODE_SUCCESS)
    {
	goto eof;
    }

    /* Read response */
    rc = priv_read_response(hds, status);

eof:
    return rc;
}


hds_retcode_t hds_host_read(HDS_HOST_HANDLE     handle,
			    uint16_t            channel,
			    uint8_t            *buf,
			    uint16_t           *len,
			    uint16_t            max_len,
			    hds_tl_status_t    *status)
{
    hds_retcode_t rc = HDS_RETCODE_SUCCESS;
    hds_host_t *hds = (hds_host_t *) handle;

    if (!handle || !buf || !len || !status)
    {
	return HDS_RETCODE_FAILURE;
    }

    /* Extra sanity checks */
    /* TODO do we need to check upper bound??? */
    if ((max_len == 0) ||
	    (max_len > (hds->data_buf_len - HDS_LL_CHKSUM_SIZE_B)))
    {
	return HDS_RETCODE_FAILURE;
    }

    /* Save max length allowed during this transaction */
    hds->app_max_len = max_len;

    /* Start frame, set read length to zero initially */
    *len = 0;
    fsm_update_with_event(hds->ll_fsm, HDS_LL_EVENT_SOF, handle);

    /* +------+ */
    /* | Req  | */
    /* +---+------+-----------------------+ */
    /* +------+ +--------+ */
    /* | Resp | |  Data  | */
    /* +------------+------+-+--------+---+ */
    /* Send request GET */
    rc = priv_req_get(hds, channel);
    if (rc != HDS_RETCODE_SUCCESS)
    {
	goto eof;
    }

    /* Read response */
    rc = priv_read_response(hds, status);
    if (rc != HDS_RETCODE_SUCCESS)
    {
	goto eof;
    }

    /* Read Data */
    rc = priv_read_data(hds, buf);
    *len = hds->app_len;

eof:
    return rc;
}

void hds_host_get_stats(HDS_HOST_HANDLE   handle,
			hds_host_stats_t *stats)
{
    hds_host_t *hds = (hds_host_t *)handle;

    if (!handle || !stats)
    {
	return;
    }

    memcpy(stats, &hds->stats, sizeof(hds->stats));
}


/* Private function definitions ----------------------------------------------- */
/* ***** General Helper functions ***** */
/**
 *  \brief      Initialize the link layer state machine
 *  \param[IN]  hds hades host handle
 *  \return     HDS_RETCODE_SUCCESS if successful, HDS_RETCODE_FAILURE
 *              otherwise
 */
static hds_retcode_t priv_hds_host_ll_fsm_init(hds_host_t *hds)
{
    int rc;
    int state;

    rc = fsm_init(&hds->ll_fsm, "Hades Host Link Layer V1.0",
		  HDS_LL_STATE_INACTIVE, HDS_LL_NUM_STATES, HDS_LL_NUM_EVENTS);
    if (rc != FSM_RETCODE_SUCCESS)
    {
	return HDS_RETCODE_FAILURE;
    }

    /* Add states */
    fsm_add_state(hds->ll_fsm,
		  HDS_LL_STATE_INACTIVE, "Inactive",
		  priv_state_inactive_entry_cb,
		  priv_state_inactive_exit_cb);
    fsm_add_state(hds->ll_fsm,
		  HDS_LL_STATE_IDLE, "Idle",
		  FSM_NO_ENTRY_CALLBACK,
		  FSM_NO_EXIT_CALLBACK);
    fsm_add_state(hds->ll_fsm,
		  HDS_LL_STATE_REQ_GET_PENDING, "Req GET Pending",
		  FSM_NO_ENTRY_CALLBACK,
		  FSM_NO_EXIT_CALLBACK);
    fsm_add_state(hds->ll_fsm,
		  HDS_LL_STATE_REQ_SET_PENDING, "Req SET Pending",
		  FSM_NO_ENTRY_CALLBACK,
		  FSM_NO_EXIT_CALLBACK);
    fsm_add_state(hds->ll_fsm,
		  HDS_LL_STATE_RESP_GET_RECV_PENDING, "Resp GET Recv Pending",
		  FSM_NO_ENTRY_CALLBACK,
		  FSM_NO_EXIT_CALLBACK);
    fsm_add_state(hds->ll_fsm,
		  HDS_LL_STATE_RESP_SET_RECV_PENDING, "Resp SET Recv Pending",
		  FSM_NO_ENTRY_CALLBACK,
		  FSM_NO_EXIT_CALLBACK);
    fsm_add_state(hds->ll_fsm,
		  HDS_LL_STATE_DATA_RECV_PENDING, "Data Recv Pending",
		  FSM_NO_ENTRY_CALLBACK,
		  FSM_NO_EXIT_CALLBACK);
    fsm_add_state(hds->ll_fsm,
		  HDS_LL_STATE_DATA_SEND_PENDING, "Data Send Pending",
		  FSM_NO_ENTRY_CALLBACK,
		  FSM_NO_EXIT_CALLBACK);

    /* Add events */
    fsm_add_event(hds->ll_fsm,
		  HDS_LL_EVENT_SOF, "SOF");
    fsm_add_event(hds->ll_fsm,
		  HDS_LL_EVENT_REQ_GET, "Req GET");
    fsm_add_event(hds->ll_fsm,
		  HDS_LL_EVENT_REQ_SET, "Req SET");
    fsm_add_event(hds->ll_fsm,
		  HDS_LL_EVENT_CTS_ENABLED, "CTS");
    fsm_add_event(hds->ll_fsm,
		  HDS_LL_EVENT_TIMEOUT, "Timeout");
    fsm_add_event(hds->ll_fsm,
		  HDS_LL_EVENT_READ_DONE, "Read Done");
    fsm_add_event(hds->ll_fsm,
		  HDS_LL_EVENT_ERROR, "General Error");

    /* Add transitions */
    fsm_add_transition(hds->ll_fsm,
		       HDS_LL_STATE_INACTIVE,
		       HDS_LL_EVENT_SOF,
		       HDS_LL_STATE_IDLE,
		       FSM_NO_TRANSITION_CALLBACK);
    fsm_add_transition(hds->ll_fsm,
		       HDS_LL_STATE_IDLE,
		       HDS_LL_EVENT_REQ_GET,
		       HDS_LL_STATE_REQ_GET_PENDING,
		       FSM_NO_TRANSITION_CALLBACK);
    fsm_add_transition(hds->ll_fsm,
		       HDS_LL_STATE_IDLE,
		       HDS_LL_EVENT_REQ_SET,
		       HDS_LL_STATE_REQ_SET_PENDING,
		       FSM_NO_TRANSITION_CALLBACK);
    /* CTS-enabled transitions */
    fsm_add_transition(hds->ll_fsm,
		       HDS_LL_STATE_REQ_GET_PENDING,
		       HDS_LL_EVENT_CTS_ENABLED,
		       HDS_LL_STATE_RESP_GET_RECV_PENDING,
		       FSM_NO_TRANSITION_CALLBACK);
    fsm_add_transition(hds->ll_fsm,
		       HDS_LL_STATE_REQ_SET_PENDING,
		       HDS_LL_EVENT_CTS_ENABLED,
		       HDS_LL_STATE_DATA_SEND_PENDING,
		       FSM_NO_TRANSITION_CALLBACK);
    fsm_add_transition(hds->ll_fsm,
		       HDS_LL_STATE_DATA_SEND_PENDING,
		       HDS_LL_EVENT_CTS_ENABLED,
		       HDS_LL_STATE_RESP_SET_RECV_PENDING,
		       FSM_NO_TRANSITION_CALLBACK);
    /* Read-done transitions */
    fsm_add_transition(hds->ll_fsm,
		       HDS_LL_STATE_RESP_GET_RECV_PENDING,
		       HDS_LL_EVENT_READ_DONE,
		       HDS_LL_STATE_DATA_RECV_PENDING,
		       FSM_NO_TRANSITION_CALLBACK);
    fsm_add_transition(hds->ll_fsm,
		       HDS_LL_STATE_DATA_RECV_PENDING,
		       HDS_LL_EVENT_READ_DONE,
		       HDS_LL_STATE_INACTIVE,
		       FSM_NO_TRANSITION_CALLBACK);
    fsm_add_transition(hds->ll_fsm,
		       HDS_LL_STATE_RESP_SET_RECV_PENDING,
		       HDS_LL_EVENT_READ_DONE,
		       HDS_LL_STATE_INACTIVE,
		       FSM_NO_TRANSITION_CALLBACK);
    /* Timeout transitions */
    fsm_add_transition(hds->ll_fsm,
		       HDS_LL_STATE_REQ_GET_PENDING,
		       HDS_LL_EVENT_TIMEOUT,
		       HDS_LL_STATE_INACTIVE,
		       FSM_NO_TRANSITION_CALLBACK);
    fsm_add_transition(hds->ll_fsm,
		       HDS_LL_STATE_REQ_SET_PENDING,
		       HDS_LL_EVENT_TIMEOUT,
		       HDS_LL_STATE_INACTIVE,
		       FSM_NO_TRANSITION_CALLBACK);
    fsm_add_transition(hds->ll_fsm,
		       HDS_LL_STATE_DATA_SEND_PENDING,
		       HDS_LL_EVENT_TIMEOUT,
		       HDS_LL_STATE_INACTIVE,
		       FSM_NO_TRANSITION_CALLBACK);
    /* General error transitions */
    for (state = 0; state < HDS_LL_NUM_STATES; state++)
    {
	fsm_add_transition(hds->ll_fsm,
			   state,
			   HDS_LL_EVENT_ERROR,
			   HDS_LL_STATE_INACTIVE,
			   FSM_NO_TRANSITION_CALLBACK);
    }

    return HDS_RETCODE_SUCCESS;
}

/* ***** FSM Entry/Exit/Transition callbacks ***** */
/**
 *  \brief      On-entry callback for HDS_LL_STATE_INACTIVE
 *  \param[IN]  context the hades device handle
 */
static void priv_state_inactive_entry_cb(void *context)
{
    hds_host_t *hds = (hds_host_t *) context;

    if (!context)
    {
	return;
    }

    /* Trigger end-of-frame when entering inactive state */
    hds->handlers.phy_eof(hds->phy_context);
}

/**
 *  \brief      On-exit callback for HDS_LL_STATE_INACTIVE
 *  \param[IN]  context the hades device handle
 */
static void priv_state_inactive_exit_cb(void *context)
{
    hds_host_t *hds = (hds_host_t *)context;

    if (!context)
    {
	return;
    }

    /* Trigger start-of-frame when entering inactive state, clear flags */
    hds->phy_read_status = HDS_PHY_STATUS_OK;
    hds->phy_write_status = HDS_PHY_STATUS_OK;
    hds->handlers.phy_sof(hds->phy_context);
}

/* ***** Helper functions: BLOCKING ***** */
/**
 *  \brief      Send a request GET to device
 *  \param[IN]  hds hades host handle
 *  \param[IN]  channel the channel ID
 *  \return     Hades library return code
 */
static hds_retcode_t priv_req_get(hds_host_t   *hds,
				  uint16_t      channel)
{
    hds_phy_status_t phy_status;
    hds_ll_request_t *req;

    if (!hds)
    {
	return HDS_RETCODE_FAILURE;
    }

    req = (hds_ll_request_t *)hds->request_buf;

    /* Prepare request GET */
    *req = (hds_ll_request_t)
    {
	.ll_header.subframe_type    = HDS_LL_SUBFRAME_REQUEST,
	.ll_header.direction        = HDS_LL_DIRECTION_GET,
	.ll_header.length           = 0,
	.ll_header.ll_status        = HDS_LL_STATUS_OK,

	.tl_header.channel          = channel,
	.tl_header.length           = 0,
	.tl_header.direction        = HDS_TL_DIRECTION_READ,
	.tl_header.tl_status        = HDS_TL_STATUS_OK,
    };
    req->chksum = hds->handlers.ll_chksum((const uint8_t *)req,
					  (sizeof(hds_ll_request_t) -
					   HDS_LL_CHKSUM_SIZE_B));

    /* Write request GET */
    hds->phy_write_status = HDS_PHY_STATUS_BUSY;
    phy_status = hds->handlers.phy_write(hds->phy_context,
					 (const uint8_t *)req,
					 sizeof(hds_ll_request_t));

    /* Make sure write call succeeded */
    if (phy_status != HDS_PHY_STATUS_OK)
    {
	fsm_update_with_event(hds->ll_fsm, HDS_LL_EVENT_ERROR, hds);
	return HDS_RETCODE_PHY_ERROR;
    }

    /* Update FSM */
    fsm_update_with_event(hds->ll_fsm, HDS_LL_EVENT_REQ_GET, hds);

    if (hds->handlers.is_phy_blocking)
    {
	hds_host_phy_write_done(hds, HDS_PHY_STATUS_OK);
    } else
    {
	/* Wait for the write to complete */
	while (hds->phy_write_status == HDS_PHY_STATUS_BUSY);

	/* Error out if PHY write did not complete successfully */
	if (hds->phy_write_status != HDS_PHY_STATUS_OK)
	{
	    fsm_update_with_event(hds->ll_fsm, HDS_LL_EVENT_ERROR, hds);
	    return HDS_RETCODE_PHY_ERROR;
	}
    }

    return HDS_RETCODE_SUCCESS;
}


/**
 *  \brief      Send a request SET, and then data, to device
 *  \param[IN]  hds hades host handle
 *  \param[IN]  channel the channel ID
 *  \param[IN]  buf data buffer to write
 *  \param[IN]  len number of bytes to write
 *  \return     Hades library return code
 */
static hds_retcode_t priv_req_set_with_data(hds_host_t      *hds,
					    uint16_t         channel,
					    const uint8_t   *buf,
					    uint16_t         len)
{
    /* Local variables */
    uint32_t chksum;
    hds_phy_cts_t cts;
    hds_phy_status_t phy_status;
    hds_ll_request_t *req;
    uint8_t *data_buf;

    if (!hds || !buf)
    {
	return HDS_RETCODE_FAILURE;
    }
    req = (hds_ll_request_t *)hds->request_buf;
    data_buf = hds->data_buf;

    /* Prepare request SET */
    *req = (hds_ll_request_t)
    {
	.ll_header.subframe_type    = HDS_LL_SUBFRAME_REQUEST,
	.ll_header.direction        = HDS_LL_DIRECTION_SET,
	.ll_header.length           = len + HDS_LL_CHKSUM_SIZE_B,
	.ll_header.ll_status        = HDS_LL_STATUS_OK,

	.tl_header.channel          = channel,
	.tl_header.length           = len,
	.tl_header.direction        = HDS_TL_DIRECTION_WRITE,
	.tl_header.tl_status        = HDS_TL_STATUS_OK,
    };
    req->chksum = hds->handlers.ll_chksum((const uint8_t *)req,
					  (sizeof(hds_ll_request_t) -
					   HDS_LL_CHKSUM_SIZE_B));

    /* Write request SET */
    hds->phy_write_status = HDS_PHY_STATUS_BUSY;
    phy_status = hds->handlers.phy_write(hds->phy_context,
					 (const uint8_t *)req,
					 sizeof(hds_ll_request_t));
    fsm_update_with_event(hds->ll_fsm, HDS_LL_EVENT_REQ_SET, hds);

    /* Make sure write call succeeded */
    if (phy_status != HDS_PHY_STATUS_OK)
    {
	fsm_update_with_event(hds->ll_fsm, HDS_LL_EVENT_ERROR, hds);
	return HDS_RETCODE_PHY_ERROR;
    }

    /* While waiting for PHY to complete (if non-blocking), prepare the */
    /* data buffer in parallel */
    memcpy(data_buf, buf, len);
    chksum = hds->handlers.ll_chksum(data_buf, len);
    data_buf[len + 0] = (chksum >>  0) & 0xFF;
    data_buf[len + 1] = (chksum >>  8) & 0xFF;
    data_buf[len + 2] = (chksum >> 16) & 0xFF;
    data_buf[len + 3] = (chksum >> 24) & 0xFF;

    if (hds->handlers.is_phy_blocking)
    {
	hds_host_phy_write_done(hds, HDS_PHY_STATUS_OK);
    } else
    {
	/* Wait for the write to complete */
	while (hds->phy_write_status == HDS_PHY_STATUS_BUSY);

	/* Error out if PHY write did not complete successfully */
	if (hds->phy_write_status != HDS_PHY_STATUS_OK)
	{
	    fsm_update_with_event(hds->ll_fsm, HDS_LL_EVENT_ERROR, hds);
	    return HDS_RETCODE_PHY_ERROR;
	}
    }

    /* Wait for device to be ready to receive data */
    while (((cts = hds->handlers.phy_cts_read(hds->phy_context)) ==
		HDS_PHY_CTS_DISABLE));
    if (cts == HDS_PHY_CTS_TIMEOUT)
    {
	/* Update stats */
	hds->stats.num_timeout_errors++;

	/* Update state machine with TIMEOUT event */
	fsm_update_with_event(hds->ll_fsm, HDS_LL_EVENT_TIMEOUT, hds);
	return HDS_RETCODE_TIMEOUT;
    }

    /* Update state machine with CTS ENABLED event */
    fsm_update_with_event(hds->ll_fsm, HDS_LL_EVENT_CTS_ENABLED, hds);

    /* Write data */
    hds->phy_write_status = HDS_PHY_STATUS_BUSY;
    phy_status = hds->handlers.phy_write(hds->phy_context, data_buf,
				req->ll_header.length);
    if (phy_status != HDS_PHY_STATUS_OK)
    {
	fsm_update_with_event(hds->ll_fsm, HDS_LL_EVENT_ERROR, hds);
	return HDS_RETCODE_PHY_ERROR;
    }

    if (hds->handlers.is_phy_blocking)
    {
	hds_host_phy_write_done(hds, HDS_PHY_STATUS_OK);
    } else
    {
	/* Wait for the write to complete */
	while (hds->phy_write_status == HDS_PHY_STATUS_BUSY);

	/* Error out if PHY write did not complete successfully */
	if (hds->phy_write_status != HDS_PHY_STATUS_OK)
	{
	    fsm_update_with_event(hds->ll_fsm, HDS_LL_EVENT_ERROR, hds);
	    return HDS_RETCODE_PHY_ERROR;
	}
    }

    return HDS_RETCODE_SUCCESS;
}


/**
 *  \brief      Blocking call to read response from device
 *  \param[IN]  hds hades host handle
 *  \param[OUT] status the transport layer return status
 *  \return     Hades library return code
 */
static hds_retcode_t priv_read_response(hds_host_t      *hds,
					hds_tl_status_t *status)
{
    hds_phy_cts_t cts;
    hds_phy_status_t phy_status;
    hds_ll_response_t *resp;

    if (!hds || !status)
    {
	return HDS_RETCODE_FAILURE;
    }

    resp = (hds_ll_response_t *)hds->response_buf;
    /* Set transport layer status to no response initially. Will */
    /* get updated later if a response is received */
    *status = HDS_TL_STATUS_NO_RESPONSE;

    /* Wait for device to be ready to respond */
    while (((cts = hds->handlers.phy_cts_read(hds->phy_context)) ==
		HDS_PHY_CTS_DISABLE));
    if (cts == HDS_PHY_CTS_TIMEOUT)
    {
	/* Update stats */
	hds->stats.num_timeout_errors++;

	/* Update state machine with TIMEOUT event */
	fsm_update_with_event(hds->ll_fsm, HDS_LL_EVENT_TIMEOUT, hds);
	return HDS_RETCODE_TIMEOUT;
    }

    /* Update state machine with CTS ENABLED event */
    fsm_update_with_event(hds->ll_fsm, HDS_LL_EVENT_CTS_ENABLED, hds);

    /* Read response */
    hds->phy_read_status = HDS_PHY_STATUS_BUSY;
    phy_status = hds->handlers.phy_read(hds->phy_context, (uint8_t *)resp,
					sizeof(hds_ll_response_t));

    /* Make sure PHY had no issues */
    if (phy_status != HDS_PHY_STATUS_OK)
    {
	fsm_update_with_event(hds->ll_fsm, HDS_LL_EVENT_ERROR, hds);
	return HDS_RETCODE_PHY_ERROR;
    }

    if (hds->handlers.is_phy_blocking)
    {
	hds_host_phy_read_done(hds, HDS_PHY_STATUS_OK);
    } else
    {
	/* Wait for read to be done */
	while (hds->phy_read_status == HDS_PHY_STATUS_BUSY);

	/* Error out if PHY read did not complete successfully */
	if (hds->phy_read_status != HDS_PHY_STATUS_OK)
	{
	    fsm_update_with_event(hds->ll_fsm, HDS_LL_EVENT_ERROR, hds);
	    hds->rc = HDS_RETCODE_PHY_ERROR;
	    return hds->rc;
	}
    }

    /* Return the transport layer status to the OUT parameter */
    *status = resp->tl_header.tl_status;

    /* Mark return code failure if status was not okay */
    if (*status != HDS_TL_STATUS_OK)
    {
	hds->rc = HDS_RETCODE_TL_FAILURE;
    } else
    {
	hds->rc = HDS_RETCODE_SUCCESS;
    }

    /* Return read return code */
    return hds->rc;
}


/**
 *  \brief      Read the response subframe
 *  \param[IN]  hds hades host handle
 *  \return     Hades library return code
 */
static hds_retcode_t priv_check_get_response(hds_host_t *hds)
{
    uint16_t tl_len;
    hds_ll_response_t *resp;

    if (!hds)
    {
	return HDS_RETCODE_FAILURE;
    }

    resp = (hds_ll_response_t *)hds->response_buf;

    /* Check that there is data to be read */
    tl_len = resp->tl_header.length;
    if (tl_len == 0)
    {
	return HDS_RETCODE_NO_DATA;
    }

    /* If the length exceeds the max application buffer length, error out */
    if (tl_len > hds->app_max_len)
    {
	return HDS_RETCODE_BAD_PARAMS;
    }

    /* If the link-layer length exceeds the max data buffer length, error out */
    if (resp->ll_header.length > hds->data_buf_len)
    {
	return HDS_RETCODE_BAD_PARAMS;
    }

    return HDS_RETCODE_SUCCESS;
}

/**
 *  \brief      Read the data subframe
 *  \param[IN]  hds hades host handle
 *  \param[IN]  buf buffer to save read data
 *  \return     Hades library return code
 */
static hds_retcode_t priv_read_data(hds_host_t *hds, uint8_t *buf)
{
    hds_phy_status_t   phy_status;
    hds_ll_response_t *resp;

    if (!hds)
    {
	return HDS_RETCODE_FAILURE;
    }

    resp = (hds_ll_response_t *)hds->response_buf;

    /* Save off application buffer pointer, reset length to zero */
    hds->app_buf = buf;
    hds->app_len = 0;

    /* Read data */
    hds->phy_read_status = HDS_PHY_STATUS_BUSY;
    phy_status = hds->handlers.phy_read(hds->phy_context, hds->data_buf,
					resp->ll_header.length);
    if (phy_status != HDS_PHY_STATUS_OK)
    {
	fsm_update_with_event(hds->ll_fsm, HDS_LL_EVENT_ERROR, hds);
	return HDS_RETCODE_PHY_ERROR;
    }

    if (hds->handlers.is_phy_blocking)
    {
	hds_host_phy_read_done(hds, HDS_PHY_STATUS_OK);
    } else
    {
	/* Wait for read to be done */
	while (hds->phy_read_status == HDS_PHY_STATUS_BUSY);

	/* Error out if PHY read did not complete successfully */
	if (hds->phy_read_status != HDS_PHY_STATUS_OK)
	{
	    fsm_update_with_event(hds->ll_fsm, HDS_LL_EVENT_ERROR, hds);
	    hds->rc = HDS_RETCODE_PHY_ERROR;
	}
    }

    /* Return read return code */
    return hds->rc;
}

/* ***** Common helper functions ***** */
/**
 *  \brief      Validate the response subframe via the checksum/subframe type
 *  \param[IN]  hds hades host handle
 *  \return     true if response is valid, false otherwise
 */
static bool priv_validate_response(hds_host_t *hds)
{
    uint32_t chksum;
    hds_ll_response_t *resp;

    if (!hds)
    {
	return false;
    }

    resp = (hds_ll_response_t *)hds->response_buf;

    /* Validate response checksum */
    chksum = hds->handlers.ll_chksum((const uint8_t *)resp,
				     (sizeof(hds_ll_response_t) -
				      HDS_LL_CHKSUM_SIZE_B));

    if (chksum != resp->chksum)
    {
	/* Update stats */
	hds->stats.num_chksum_errors++;
	hds->stats.err_recvd_chksum = resp->chksum;
	hds->stats.err_calc_chksum = chksum;

	hds->rc = HDS_RETCODE_BAD_CHKSUM;
	return false;
    }

    /* Make sure subframe is valid */
    if (resp->ll_header.subframe_type != HDS_LL_SUBFRAME_RESPONSE)
    {
	hds->rc = HDS_RETCODE_BAD_RESPONSE;
	return false;
    }

    hds->rc = HDS_RETCODE_SUCCESS;
    return true;
}

/**
 *  \brief      Validate the data subframe via the checksum
 *  \param[IN]  hds hades host handle
 *  \return     true if response is valid, false otherwise
 */
static bool priv_validate_data(hds_host_t *hds)
{
    uint16_t tl_len;
#if 1
    uint32_t chksum;
    uint32_t data_chksum;
#endif
    uint8_t *data_buf;
    hds_ll_response_t *resp;

    if (!hds)
    {
	return false;
    }


    data_buf = hds->data_buf;
    resp = (hds_ll_response_t *)hds->response_buf;

    /* Determine transport layer payload length */
    tl_len = resp->tl_header.length;

    /* Validate data checksum */
    chksum = hds->handlers.ll_chksum(data_buf, tl_len);
    data_chksum = (data_buf[tl_len + 0] <<  0) +
		  (data_buf[tl_len + 1] <<  8) +
		  (data_buf[tl_len + 2] << 16) +
		  (data_buf[tl_len + 3] << 24);
    if (chksum != data_chksum)
    {
	/* Update stats */
	hds->stats.num_chksum_errors++;
	hds->stats.err_recvd_chksum = resp->chksum;
	hds->stats.err_calc_chksum = chksum;

	hds->rc = HDS_RETCODE_BAD_CHKSUM;
	return false;
    }

    /* If valid, copy data over to application buffer */
    memcpy(hds->app_buf, data_buf, tl_len);
    hds->app_len = tl_len;

    hds->rc = HDS_RETCODE_SUCCESS;
    return true;
}
