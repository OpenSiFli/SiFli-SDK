/**
 ****************************************************************************************
 *
 * @file ahi_int_task.h
 *
 * @brief This file contains definitions related to the Application Host Interface in the same core
 *
 * Copyright (C) Sifli 2020-2020
 *
 *
 ****************************************************************************************
 */

#ifndef AHI_INT_TASK_H_
#define AHI_INT_TASK_H_

/**
 ****************************************************************************************
 * @addtogroup AHI_INT Application Host Interface in the same core
 *@{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"     // SW configuration

#if (AHI_INT_SUPPORT)

#include "rwip_task.h"       // Task definition
#include "rwble_hl.h"        // BLE HL default include

/*
 * INSTANCES
 ****************************************************************************************
 */
/// Maximum number of instances of the AHI task
#define AHI_INT_IDX_MAX 1

/*
 * STATES
 ****************************************************************************************
 */
/// Possible states of the AHI INT task
enum AHI_INT_STATE
{
    /// TX IDLE state
    AHI_INT_TX_IDLE,
    /// Number of states.
    AHI_INT_STATE_MAX
};

/*
 * MESSAGES
 ****************************************************************************************
 */
/// Message API of the AHI INT task
/*@TRACE*/
enum ahi_int_msg_id
{
    AHI_INT_MSG_ID_FIRST = TASK_FIRST_MSG(TASK_ID_AHI_INT),

    AHI_INT_MSG_ID_LAST
};

/*
 * TASK DESCRIPTOR DECLARATIONS
 ****************************************************************************************
 */

#endif //AHI_INT_SUPPORT

/// @} AHI_INT

#endif // AHI_INT_TASK_H_

