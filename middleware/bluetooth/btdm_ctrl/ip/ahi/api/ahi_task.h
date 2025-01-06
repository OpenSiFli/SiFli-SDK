/**
 ****************************************************************************************
 *
 * @file ahi_task.h
 *
 * @brief This file contains definitions related to the Application Host Interface
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

#ifndef AHI_TASK_H_
#define AHI_TASK_H_

/**
 ****************************************************************************************
 * @addtogroup AHI Application Host Interface
 *@{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"     // SW configuration

#if (AHI_TL_SUPPORT)

#include "rwip_task.h"       // Task definition
#include "rwble_hl.h"        // BLE HL default include

/*
 * INSTANCES
 ****************************************************************************************
 */
/// Maximum number of instances of the AHI task
#define AHI_IDX_MAX 1

/*
 * STATES
 ****************************************************************************************
 */
/// Possible states of the AHI task
enum AHI_STATE
{
    /// TX IDLE state
    AHI_TX_IDLE,
    /// TX ONGOING state
    AHI_TX_ONGOING,
    /// Number of states.
    AHI_STATE_MAX
};

/*
 * MESSAGES
 ****************************************************************************************
 */
/// Message API of the AHI task
/*@TRACE*/
enum ahi_msg_id
{
    AHI_MSG_ID_FIRST = TASK_FIRST_MSG(TASK_ID_AHI),

    AHI_MSG_ID_LAST
};

/*
 * TASK DESCRIPTOR DECLARATIONS
 ****************************************************************************************
 */

#endif //AHI_TL_SUPPORT

/// @} AHI

#endif // AHI_TASK_H_

