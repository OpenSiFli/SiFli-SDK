/**
 ****************************************************************************************
 *
 * @file isoohci.h
 *
 * @brief Declaration of the functions used for Isochronous Payload over HCI Transport layer
 *
 * Copyright (C) RivieraWaves 2009-2017
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup ISOOHCI
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"

#if (BLE_ISOOHCI)

#include <string.h>

#include "rwip.h"
#include "isoohci_int.h"

#include "dbg_swdiag.h"
#include "ke_mem.h"
#include "ke_event.h"

#include "co_list.h"
#include "h4tl.h"
#include "arch.h"

/*
 * DEFINES
 ****************************************************************************************
 */


/*
 * ENUMERATIONS
 ****************************************************************************************
 */


/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */


/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// ISO over HCI environment data
struct isoohci_env_tag isoohci_env;


/*
 * LOCAL FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/*
 * FUNCTION DEFINITION
 ****************************************************************************************
 */

void isoohci_init(bool is_reset)
{
    if (!is_reset)
    {
        // Set isochronous management defer handler
        ke_event_callback_set(KE_EVENT_ISOOHCI_OUT_DEFER, &isoohci_out_defer_handler);
        ke_event_callback_set(KE_EVENT_ISOOHCI_IN_DEFER, &isoohci_in_defer_handler);

        // Initialize TX queue
        co_list_init(&isoohci_env.hci_wait_tx_queue);
        co_list_init(&isoohci_env.hci_in_tx_queue);

        // Initialize environment variable
        memset(&(isoohci_env.p_in),  0, sizeof(isoohci_env.p_in));
        memset(&(isoohci_env.p_out), 0, sizeof(isoohci_env.p_out));
    }
    else
    {
        // Clean-up open channel
        for (uint8_t channel = 0 ; channel < BLE_ACTIVITY_MAX ; channel++)
        {
            if (isoohci_env.p_in[channel] != NULL)
            {
                struct isoohci_in_info *p_in_env = isoohci_env.p_in[channel];

                // Get the first buffer from the input queue
                isoohci_buffer_t *p_in_buf = (isoohci_buffer_t *) co_list_pop_front(&(p_in_env->in_queue));

                while (p_in_buf != NULL)
                {
                    // Release buffer
                    ke_free(p_in_buf);

                    // Move onto the next buffer
                    p_in_buf = (isoohci_buffer_t *) co_list_pop_front(&(p_in_env->in_queue));
                }

                // If there is a buffer under construction
                if (p_in_env->current_buf != NULL)
                {
                    ke_free(p_in_env->current_buf);
                }

                // If there is a buffer in use by ISOAL
                if (p_in_env->buf_in_use != NULL)
                {
                    ke_free(p_in_env->buf_in_use);
                }

                ke_free(isoohci_env.p_in[channel]);
                isoohci_env.p_in[channel] = NULL;
            }

            if (isoohci_env.p_out[channel] != NULL)
            {
                ke_free(isoohci_env.p_out[channel]);
                isoohci_env.p_out[channel] = NULL;
            }
        }
    }

    // Initialize message table index to 0
    isoohci_env.in_sdu_idx = 0;
    isoohci_env.in_sdu_send_idx = 0;
}


const struct data_path_itf *isoohci_itf_get(uint8_t  direction)
{
    const struct data_path_itf *res = NULL;

    // load the interface
    switch (direction)
    {
    case ISO_SEL_TX:
    {
        res = &isoohci_in_itf;
    }
    break;
    case ISO_SEL_RX:
    {
        res = &isoohci_out_itf;
    }
    break;
    default:         { /* Nothing to do */    } break;
    }

    return res;
}

#endif //(BLE_ISOOHCI)

/// @} ISOOHCI
