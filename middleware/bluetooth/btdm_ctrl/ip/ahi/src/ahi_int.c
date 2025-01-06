/**
 ****************************************************************************************
 *
 * @file ahi_int.c
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
#include "ke_event.h"
#include "ke_task.h"
#include "ke_mem.h"
#include "co_bt.h"           // BT standard definitions
#include "co_list.h"
#include "gapm.h"

/*
 * DEFINES
 ****************************************************************************************
 */

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */


///AHI Environment context structure
struct ahi_int_env_tag
{
    // Callback for user
    ke_msg_func_t callback;
};


/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
extern const struct ke_task_desc TASK_DESC_AHI_INT;


/// AHI environment context
struct ahi_int_env_tag ahi_int_env;



/*
 * LOCAL FUNCTION DECLARTIONS
 ****************************************************************************************
 */


/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */


/*
 * EXPORTED FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void ahi_callback_init(ke_msg_func_t callback)
{
    ahi_int_env.callback = callback;
}

void ahi_int_init(void)
{
    // Create AHI Task
    ke_task_create(TASK_AHI_INT, &TASK_DESC_AHI_INT);

    // Initialize AHI task to idle state
    ke_state_set(TASK_AHI_INT, AHI_INT_TX_IDLE);

}

uint32_t ahi_int_is_ready(void)
{
    return ahi_int_env.callback ? 1 : 0;
}


void ahi_int_msg_callback(ke_msg_id_t const msgid, void *param, ke_task_id_t const dest_id, ke_task_id_t const src_id)
{

    if (ahi_int_env.callback)
        ahi_int_env.callback(msgid, param, dest_id, src_id);
}

void ahi_int_msg_send(void const *param_ptr)
{
    //extract the ke_msg pointer from the param passed
    struct ke_msg *msg = ke_param2msg(param_ptr);

    // Update source and dest task identifiers with task number
    msg->src_id  = gapm_get_task_from_id(msg->src_id);
    msg->dest_id = gapm_get_task_from_id(msg->dest_id);

    if (KE_TYPE_GET(msg->dest_id) == TASK_NONE)
    {
        // error handler for None task case.
        msg->dest_id = TASK_GAPM;
        msg->param_len = msg->id;
        msg->src_id = GAPM_UNKNOWN_TASK_MSG;
    }

    ke_msg_send(param_ptr);

}



#endif //AHI_INT_SUPPORT

/// @} AHI

