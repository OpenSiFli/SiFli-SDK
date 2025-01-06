/**
 ****************************************************************************************
 *
 * @file ahi_int_task.c
 *
 * @brief This file contains definitions related to the Application Host Interface in same core
 *
 * Copyright (C) Sifli 2020-2020
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup AHI_INT
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"     // SW configuration

#if (AHI_INT_SUPPORT)


#include "ahi_int.h"
#include "ahi_int_task.h"
#include "ke_msg.h"          // kernel message defines
#include "gapm.h"
#include "co_utils.h"        // core utility functions

/*
 * DEFINES
 ****************************************************************************************
 */



/*
 * MESSAGE HANDLERS
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @brief Function called to send a message through UART.
 *
 * @param[in]  msgid   U16 message id from ke_msg.
 * @param[in] *param   Pointer to parameters of the message in ke_msg.
 * @param[in]  dest_id Destination task id.
 * @param[in]  src_id  Source task ID.
 *
 * @return             Kernel message state, must be KE_MSG_NO_FREE.
 *****************************************************************************************
 */
__STATIC int ahi_msg_send_handler(ke_msg_id_t const msgid,
                                  void *param,
                                  ke_task_id_t const dest_id,
                                  ke_task_id_t const src_id)
{
    //extract the ke_msg pointer from the param passed
    struct ke_msg *msg = ke_param2msg(param);

    // Update source and dest task number with task identifiers
    msg->src_id  = gapm_get_id_from_task(msg->src_id);
    msg->dest_id = gapm_get_id_from_task(msg->dest_id);

    // Callback user.
    ahi_int_msg_callback(msgid, param, msg->dest_id,  msg->src_id);

    //always treat consumed
    return KE_MSG_CONSUMED;
}

/*
 * TASK DESCRIPTOR DEFINITIONS
 ****************************************************************************************
 */

/// Specifies the message handlers that are common to all states.
KE_MSG_HANDLER_TAB(ahi_int)
{

    /** Default handler for AHI TX message, this entry has to be put first as table is
        parsed from end to start by Kernel */
    {KE_MSG_DEFAULT_HANDLER, (ke_msg_func_t)ahi_msg_send_handler},
};

/// Defines the placeholder for the states of all the task instances.
ke_state_t ahi_int_state[AHI_INT_IDX_MAX];

/// AHI task descriptor
const struct ke_task_desc TASK_DESC_AHI_INT = {ahi_int_msg_handler_tab, ahi_int_state, AHI_INT_IDX_MAX,  ARRAY_LEN(ahi_int_msg_handler_tab)};



#endif //AHI_TL_SUPPORT

/// @} AHI
