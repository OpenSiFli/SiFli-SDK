/**
 ****************************************************************************************
 *
 * @file display_task.h
 *
 * @brief This file contains definitions related to the Display module
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

#ifndef DISPLAY_TASK_H_
#define DISPLAY_TASK_H_

/**
 ****************************************************************************************
 * @addtogroup DISPLAY Display module
 *@{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"     // SW configuration

#if (DISPLAY_SUPPORT)

#include "rwip_task.h"       // Task definition


/*
 * STATES
 ****************************************************************************************
 */
/// Possible states of the DISPLAY task
enum DISPLAY_STATE
{
    /// IDLE state
    DISPLAY_IDLE,
    /// Number of states.
    DISPLAY_STATE_MAX
};

/*
 * MESSAGES
 ****************************************************************************************
 */
/// Message API of the DISPLAY task
/*@TRACE*/
enum display_msg_id
{
    DISPLAY_MSG_ID_FIRST = TASK_FIRST_MSG(TASK_ID_DISPLAY),

#if DISPLAY_PERIODIC
    DISPLAY_SCREEN_TO,
#endif // DISPLAY_PERIODIC

    DISPLAY_MSG_ID_LAST
};

/*
 * TASK DESCRIPTOR DECLARATIONS
 ****************************************************************************************
 */

/// DISPLAY task descriptor
extern const struct ke_task_desc TASK_DESC_DISPLAY;

#endif //DISPLAY_SUPPORT

/// @} DISPLAY

#endif // DISPLAY_TASK_H_

