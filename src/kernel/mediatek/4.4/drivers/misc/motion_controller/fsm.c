/******************************************************************************
 * Copyright 2020 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 *
 *         file: fsm.c
 *  description: Finite State Machine (FSM) framework source file.
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

/* Includes ------------------------------------------------------------------- */
#include <linux/slab.h>
#include "fsm.h"

/* Definitions ---------------------------------------------------------------- */
#define FSM_STATE_UNDEFINED     (-1)
#define FSM_EVENT_UNDEFINED     (-1)
#define FSM_LUT_IDX(state,      \
		    event,      \
		    max_events) ((state) * (max_events) + (event))

/* Struct/Enums --------------------------------------------------------------- */
typedef struct {
    int              state;     /* FSM state */
    char            *name;      /* FSM state name */
    fsm_callback     on_entry;  /* FSM state on-entry callback */
    fsm_callback     on_exit;   /* FSM state on-exit callback */
} fsm_state_t;

typedef struct {
    int              event;     /* FSM event */
    char            *name;      /* FSM event name */
} fsm_event_t;

typedef struct {
    char            *name;              /* FSM name */
    int              curr_state;        /* Current state of the state machine */
    int              last_event;        /* Last event of the state machine */

    int              initial_state;     /* Initial state of state machine */

    int              max_states;        /* Maximum number of states */
    fsm_state_t     *states;            /* Array of all states */

    int              max_events;        /* Maximum number of events */
    fsm_event_t     *events;            /* Array of all events */

    int             *next_state_lut;    /* Next State lookup table given */
					/* 1) state number and 2) event number */
    fsm_callback * transition_lut;    
					/* 1) state number and 2) event number */
} fsm_state_machine_t;


/* Private function prototypes ------------------------------------------------ */
void priv_fsm_cleanup(FSM_HANDLE handle);


/* Public function definitions ------------------------------------------------ */
fsm_retcode_t fsm_init(FSM_HANDLE * handle,
		       const char   *name,
		       int           initial_state_num,
		       int           max_states,
		       int           max_events)
{
    int s;
    int e;
    fsm_state_machine_t *fsm;

    /* Sanity checks */
    if (!handle || (max_states <= 0) || (max_events <= 0) ||
	    (initial_state_num >= max_states))
    {
	return FSM_RETCODE_FAILURE;
    }

    /* Create the finite state machine data structure */
    fsm = kmalloc(sizeof(fsm_state_machine_t), GFP_KERNEL);
    if (!fsm)
    {
	return FSM_RETCODE_FAILURE;
    }

    /* Store the state machine name, if valid */
    if (name)
    {
	fsm->name = kstrndup(name, FSM_MAX_STRING_LEN, GFP_KERNEL);
	if (!fsm->name)
	{
	    goto cleanup;
	}
    } else
    {
	fsm->name = NULL;
    }

    /* Set the initial state */
    fsm->curr_state = initial_state_num;
    fsm->initial_state = initial_state_num;
    fsm->last_event = FSM_EVENT_UNDEFINED;

    /* Set the maximum number of states and events and allocate */
    /* space for the state and event arrays */
    fsm->max_states = max_states;
    fsm->max_events = max_events;
    fsm->states = kcalloc(max_states, sizeof(fsm_state_t), GFP_KERNEL);
    if (!fsm->states)
    {
	goto cleanup;
    }
    fsm->events = kcalloc(max_events, sizeof(fsm_event_t), GFP_KERNEL);
    if (!fsm->events)
    {
	goto cleanup;
    }

    /* Mark all states and events as "undefined" */
    for (s = 0; s < max_states; s++)
    {
	fsm->states[s].state = FSM_STATE_UNDEFINED;
    }
    for (e = 0; e < max_events; e++)
    {
	fsm->events[e].event = FSM_EVENT_UNDEFINED;
    }

    /* Create the uninitialized next-state and event callback lookup tables */
    fsm->next_state_lut = kmalloc(max_states * max_events * sizeof(int),
				    GFP_KERNEL);
    if (!fsm->next_state_lut)
    {
	goto cleanup;
    }
    fsm->transition_lut = kcalloc(max_states * max_events,
				  sizeof(fsm_callback), GFP_KERNEL);
    if (!fsm->transition_lut)
    {
	goto cleanup;
    }

    for (s = 0; s < max_states; s++)
    {
	for (e = 0; e < max_events; e++)
	{
	    fsm->next_state_lut[FSM_LUT_IDX(s, e, max_events)] =
		FSM_STATE_UNDEFINED;
	}
    }

    /* Return the finite state machine in the OUT parameter */
    *handle = (fsm_state_machine_t *)fsm;
    return FSM_RETCODE_SUCCESS;

cleanup:
    priv_fsm_cleanup(*handle);
    return FSM_RETCODE_FAILURE;
}


fsm_retcode_t fsm_deinit(FSM_HANDLE handle)
{
    if (!handle)
    {
	return FSM_RETCODE_FAILURE;
    }

    priv_fsm_cleanup(handle);

    return FSM_RETCODE_SUCCESS;
}

fsm_retcode_t fsm_reset(FSM_HANDLE handle)
{
    fsm_state_machine_t *fsm;

    if (!handle)
    {
	return FSM_RETCODE_FAILURE;
    }

    fsm = (fsm_state_machine_t *)handle;

    fsm->curr_state = fsm->initial_state;
    fsm->last_event = FSM_EVENT_UNDEFINED;

    return FSM_RETCODE_SUCCESS;
}


fsm_retcode_t fsm_add_state(FSM_HANDLE      handle,
			    int             state_num,
			    const char     *state_name,
			    fsm_callback    state_entry_callback,
			    fsm_callback    state_exit_callback)
{
    fsm_state_machine_t *fsm;

    if (!handle)
    {
	return FSM_RETCODE_FAILURE;
    }

    fsm = (fsm_state_machine_t *)handle;

    if ((state_num >= fsm->max_states) || (state_num < 0) ||
	    (fsm->states[state_num].state != FSM_STATE_UNDEFINED))
    {
	return FSM_RETCODE_FAILURE;
    }

    if (state_name)
    {
	fsm->states[state_num].name = kstrndup(state_name, FSM_MAX_STRING_LEN,
						GFP_KERNEL);
	if (!fsm->states[state_num].name)
	{
	    return FSM_RETCODE_FAILURE;
	}
    } else
    {
	fsm->states[state_num].name = NULL;
    }
    fsm->states[state_num].state = state_num;
    fsm->states[state_num].on_entry = state_entry_callback;
    fsm->states[state_num].on_exit = state_exit_callback;

    return FSM_RETCODE_SUCCESS;
}


fsm_retcode_t fsm_add_event(FSM_HANDLE   handle,
			    int          event_num,
			    const char  *event_name)
{
    fsm_state_machine_t *fsm;

    if (!handle)
    {
	return FSM_RETCODE_FAILURE;
    }

    fsm = (fsm_state_machine_t *)handle;

    if ((event_num >= fsm->max_events) || (event_num < 0)
	    || (fsm->events[event_num].event != FSM_EVENT_UNDEFINED))
    {
	return FSM_RETCODE_FAILURE;
    }

    if (event_name)
    {
	fsm->events[event_num].name = kstrndup(event_name, FSM_MAX_STRING_LEN,
						GFP_KERNEL);
	if (!fsm->events[event_num].name)
	{
	    return FSM_RETCODE_FAILURE;
	}
    } else
    {
	fsm->events[event_num].name = NULL;
    }
    fsm->events[event_num].event = event_num;

    return FSM_RETCODE_SUCCESS;
}


fsm_retcode_t fsm_add_transition(FSM_HANDLE     handle,
				 int            start_state_num,
				 int            event_num,
				 int            next_state_num,
				 fsm_callback   transition_callback)
{
    fsm_state_machine_t *fsm;
    int idx;

    if (!handle)
    {
	return FSM_RETCODE_FAILURE;
    }

    fsm = (fsm_state_machine_t *)handle;
    idx = FSM_LUT_IDX(start_state_num, event_num, fsm->max_events);
    if ((start_state_num >= fsm->max_states) ||
	    (event_num >= fsm->max_events) ||
	    (next_state_num >= fsm->max_states) ||
	    (fsm->next_state_lut[idx] != FSM_STATE_UNDEFINED))
    {
	return FSM_RETCODE_FAILURE;
    }

    fsm->next_state_lut[idx] = next_state_num;
    fsm->transition_lut[idx] = transition_callback;

    return FSM_RETCODE_SUCCESS;
}

fsm_retcode_t fsm_get_last_event(FSM_HANDLE   handle,
				 int         *last_event_num,
				 const char **last_event_name)
{
    fsm_state_machine_t *fsm;

    if (!handle)
    {
	return FSM_RETCODE_FAILURE;
    }

    fsm = (fsm_state_machine_t *)handle;
    *last_event_num = fsm->last_event;

    /* No event has happened yet */
    if (fsm->last_event < 0)
    {
	*last_event_name = NULL;
    } else
    {
	*last_event_name = fsm->events[fsm->last_event].name;
    }
    return FSM_RETCODE_SUCCESS;
}


fsm_retcode_t fsm_get_current_state(FSM_HANDLE   handle,
				    int         *curr_state_num,
				    const char **curr_state_name)
{
    fsm_state_machine_t *fsm;

    if (!handle)
    {
	return FSM_RETCODE_FAILURE;
    }

    fsm = (fsm_state_machine_t *)handle;
    *curr_state_num = fsm->curr_state;
    *curr_state_name = fsm->states[fsm->curr_state].name;
    return FSM_RETCODE_SUCCESS;
}


fsm_retcode_t fsm_get_event_name(FSM_HANDLE     handle,
				 int            event_num,
				 const char   **event_name)
{
    fsm_state_machine_t *fsm;

    if (!handle)
    {
	return FSM_RETCODE_FAILURE;
    }

    fsm = (fsm_state_machine_t *)handle;
    *event_name = fsm->events[event_num].name;
    return FSM_RETCODE_SUCCESS;
}


fsm_retcode_t fsm_get_name(FSM_HANDLE handle, const char **name)
{
    fsm_state_machine_t *fsm;

    if (!handle)
    {
	return FSM_RETCODE_FAILURE;
    }

    fsm = (fsm_state_machine_t *)handle;
    *name = fsm->name;
    return FSM_RETCODE_SUCCESS;
}


fsm_retcode_t fsm_update_with_event(FSM_HANDLE  handle,
				    int         event_num,
				    void       *context)
{
    fsm_state_machine_t *fsm;
    int idx, next_state;
    fsm_callback curr_state_exit_cb, transition_cb, next_state_entry_cb;

    if (!handle)
    {
	return FSM_RETCODE_FAILURE;
    }

    fsm = (fsm_state_machine_t *)handle;
    if ((event_num < 0) || (event_num >= fsm->max_events))
    {
	return FSM_RETCODE_FAILURE;
    }

    idx = FSM_LUT_IDX(fsm->curr_state, event_num, fsm->max_events);
    next_state = fsm->next_state_lut[idx];

    if (next_state == FSM_STATE_UNDEFINED)
    {
	/* No transition available */
	return FSM_RETCODE_SUCCESS;
    }

    /* With a valid transition, save the transition callback */
    transition_cb = fsm->transition_lut[idx];

    /* Only set these if we are changing states */
    curr_state_exit_cb = NULL;
    next_state_entry_cb = NULL;
    if (fsm->curr_state != next_state)
    {
	curr_state_exit_cb = fsm->states[fsm->curr_state].on_exit;
	next_state_entry_cb = fsm->states[next_state].on_entry;
	fsm->last_event = event_num;
    }

    /* Current state exit callback */
    /* Note: Will be forced to NULL if not changing states */
    if (curr_state_exit_cb)
    {
	curr_state_exit_cb(context);
    }

    /* Event callback */
    if (transition_cb)
    {
	transition_cb(context);
    }

    /* Next state entry callback */
    /* Note: Will be forced to NULL if not changing states */
    if (next_state_entry_cb)
    {
	next_state_entry_cb(context);
    }

    /* Update state machine to next state */
    fsm->curr_state = next_state;

    return FSM_RETCODE_SUCCESS;
}

/* Private function definitions ----------------------------------------------- */
void priv_fsm_cleanup(FSM_HANDLE handle)
{
    int i;
    fsm_state_machine_t *fsm;

    if (!handle)
    {
	return;
    }

    fsm = (fsm_state_machine_t *)handle;

    /* Free the next-state and event callback lookup tables */
    if (fsm->next_state_lut)
    {
	kfree(fsm->next_state_lut);
    }
    if (fsm->transition_lut)
    {
	kfree(fsm->transition_lut);
    }

    /* Free the states and events arrays */
    if (fsm->states)
    {
	for (i = 0; i < fsm->max_states; i++)
	{
	    if (fsm->states[i].name)
	    {
		kfree(fsm->states[i].name);
	    }
	}
	kfree(fsm->states);
    }
    if (fsm->events)
    {
	for (i = 0; i < fsm->max_events; i++)
	{
	    if (fsm->events[i].name)
	    {
		kfree(fsm->events[i].name);
	    }
	}
	kfree(fsm->events);
    }

    /* Free the state machine name, if it exists */
    if (fsm->name)
    {
	kfree(fsm->name);
    }

    /* Free the finite state machine data structure */
    kfree(fsm);
}
