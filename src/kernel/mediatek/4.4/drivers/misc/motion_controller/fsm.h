/******************************************************************************
 * Copyright 2020 Amazon.com, Inc. or its affliliates. All Rights Reserved.
 *
 *         file: fsm.h
 *  description: Finite State Machine (FSM) framework header file.
 *
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
#include <stddef.h>


/* Definitions ---------------------------------------------------------------- */
#define FSM_NO_ENTRY_CALLBACK       NULL
#define FSM_NO_EXIT_CALLBACK        NULL
#define FSM_NO_TRANSITION_CALLBACK  NULL

#define FSM_MAX_STRING_LEN          50      /* Includes NULL character */

/* Struct/Enums --------------------------------------------------------------- */
/**
 * \brief       Opaque struct definition for the finite state machine framework
 */
typedef void *FSM_HANDLE;

/**
 *  \brief      Enumeration of FSM library return codes
 */
typedef enum {
    FSM_RETCODE_SUCCESS,    /* /< Success */
    FSM_RETCODE_FAILURE,    /* /< Failure */
} fsm_retcode_t;

/**
 *  \brief      FSM callback function
 *  \param[IN]  context any data context for the state machine
 */
typedef void (*fsm_callback)(void *context);

/**
 *  \brief      Struct to hold state information
 */
struct fsm_state_entry {
    int         state_num;
    const char *state_name;
    fsm_callback entry_callback;
    fsm_callback exit_callback;
};

/**
 *  \brief      Struct to hold event information
 */
struct fsm_event_entry {
    int         event_num;
    const char *event_name;
};

/**
 *  \brief      Struct to hold transition information
 */
struct fsm_transition_entry {
    int            start_state_num;
    int            event_num;
    int            next_state_num;
    fsm_callback   transition_callback;
};


/* Public function prototypes ------------------------------------------------- */
/**
 *  \brief      Initialization function for creating a finite state machine
 *  \param[OUT] handle the finite state machine handle
 *  \param[IN]  name human-readable string representation of the state machine
 *  \param[IN]  initial_state_num the initial state of the state machine
 *  \param[IN]  max_states maximum number of states
 *  \param[IN]  max_events maximum number of events
 *  \return     FSM_RETCODE_SUCCESS if successful, FSM_RETCODE_FAILURE
 *              otherwise
 */
fsm_retcode_t fsm_init(FSM_HANDLE * handle,
		       const char   *name,
		       int           initial_state_num,
		       int           max_states,
		       int           max_events);

/**
 *  \brief      De-initialization function for a finite state machine
 *  \note       This function handles all the freeing of any and all dynamic
 *              memory allocation when fsm_init() was called
 *  \param[IN]  handle the finite state machine handle
 */
fsm_retcode_t fsm_deinit(FSM_HANDLE handle);


/**
 *  \brief      Reset the state machine to initial state
 *
 *  \param[IN]  handle the finite state machine handle
 */
fsm_retcode_t fsm_reset(FSM_HANDLE handle);



/**
 *  \brief      Add a state to the finite state machine
 *  \param[IN]  handle the finite state machine handle
 *  \param[IN]  state_num number representation of the state
 *  \param[IN]  state_name human-readable string representation of the state
 *  \param[IN]  state_entry_callback callback when entering state (can be NULL)
 *  \param[IN]  state_exit_callback callback when entering state (can be NULL)
 *  \return     FSM_RETCODE_SUCCESS if successful, FSM_RETCODE_FAILURE
 *              otherwise (e.g. exceeds maximum number of states)
 */
fsm_retcode_t fsm_add_state(FSM_HANDLE      handle,
			    int             state_num,
			    const char     *state_name,
			    fsm_callback    state_entry_callback,
			    fsm_callback    state_exit_callback);

/**
 *  \brief      Add an event to the finite state machine
 *  \param[IN]  handle the finite state machine handle
 *  \param[IN]  event_num number representation of the event
 *  \param[IN]  event_name human-readable string representation of the event
 *  \return     FSM_RETCODE_SUCCESS if successful, FSM_RETCODE_FAILURE
 *              otherwise (e.g. exceeds maximum number of events)
 */
fsm_retcode_t fsm_add_event(FSM_HANDLE   handle,
			    int          event_num,
			    const char  *event_name);

/**
 *  \brief      Add a state-to-state transition to the finite state machine
 *  \param[IN]  handle the finite state machine handle
 *  \param[IN]  start_state_num number representation of the starting state
 *  \param[IN]  event_num number representation of the event triggering
 *                        the transition
 *  \param[IN]  next_state_num number representation of the next state
 *  \param[IN]  transition_callback callback when transitioning states
 *  \return     FSM_RETCODE_SUCCESS if successful, FSM_RETCODE_FAILURE
 *              otherwise
 */
fsm_retcode_t fsm_add_transition(FSM_HANDLE     handle,
				 int            start_state_num,
				 int            event_num,
				 int            next_state_num,
				 fsm_callback   transition_callback);

/**
 *  \brief      Get current state
 *  \param[IN]  handle the finite state machine handle
 *  \param[OUT] curr_state_num the current state number representation
 *  \param[OUT] curr_state_name the current state string representation
 */
fsm_retcode_t fsm_get_current_state(FSM_HANDLE   handle,
				    int         *curr_state_num,
				    const char **curr_state_name);

/**
 *  \brief      Get last event
 *  \param[IN]  handle the finite state machine handle
 *  \param[OUT] last_event_num the last event number representation. -1 if no event has happend yet
 *  \param[OUT] last_event_name the last event string representation. NULL if no event has happened yet
 */
fsm_retcode_t fsm_get_last_event(FSM_HANDLE   handle,
				    int         *last_event_num,
				    const char **last_event_name);



/**
 *  \brief      Get the name of an event
 *  \param[IN]  handle the finite state machine handle
 *  \param[IN]  event_num the event number representation
 *  \param[OUT] event_name string representation of the event
 */
fsm_retcode_t fsm_get_event_name(FSM_HANDLE     handle,
				 int            event_num,
				 const char   **event_name);

/**
 *  \brief      Get FSM name
 *  \param[IN]  handle the finite state machine handle
 *  \param[OUT] name the user-determined state machine name
 */
fsm_retcode_t fsm_get_name(FSM_HANDLE handle, const char **name);

/**
 *  \brief      Update the finite state machine given an event (and context)
 *  \param[IN]  handle the finite state machine handle
 *  \param[IN]  event_num number representation of the event triggering
 *                        the transition
 *  \param[IN]  context any data context for the state machine (may be NULL)
 *  \return     FSM_RETCODE_SUCCESS if successfully transitioned to the
 *              specified next state; FSM_RETCODE_FAILURE otherwise.
 */
fsm_retcode_t fsm_update_with_event(FSM_HANDLE  handle,
				    int         event_num,
				    void       *context);

#if defined(__x86_64__)
fsm_retcode_t fsm_print(FSM_HANDLE handle);
#else
#define fsm_print(fsm)
#endif

#ifdef __cplusplus
} /* extern "C" */
#endif
