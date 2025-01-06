/**
 ****************************************************************************************
 *
 * @file l2cm.c
 *
 * @brief L2CAP Manager implementation.
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 *
 ****************************************************************************************
 */
/**
 ****************************************************************************************
 * @addtogroup L2CM
 * @{
 ****************************************************************************************
 */
#include "rwip_config.h"

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#if (BLE_L2CM)
#include <string.h>
#include "l2cm_int.h"
#include "../l2cc/l2cc_int.h" // Internal API is required

#include "l2cc.h"
#include "ke_event.h"
/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
///L2CM environment structures table to be in retention area
struct l2cm_env_tag l2cm_env;


/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
#if (BLE_L2CC)
static void l2cm_l2cap_tx_handler(void)
{
    ke_event_clear(KE_EVENT_L2CAP_TX);
    uint32_t cur_state = l2cm_env.con_tx_state;

    if (cur_state)
    {
        uint8_t conidx;

        // browse all connection
        for (conidx = 0 ; (conidx < BLE_CONNECTION_MAX) && (l2cm_env.nb_buffer_avail > 0) ; conidx++)
        {
            // check if waiting for TX
            if (cur_state & (1 << conidx))
            {
                // send data buffers
                l2cc_data_send(conidx, l2cm_env.nb_buffer_avail);
            }
        }

        // check if activity is finished or not
        if ((l2cm_env.con_tx_state != 0) && (l2cm_env.nb_buffer_avail > 0))
        {
            ke_event_set(KE_EVENT_L2CAP_TX);
        }
    }
}
#endif // (BLE_L2CC)

void l2cm_event_init()
{
    ke_event_callback_set(KE_EVENT_L2CAP_TX, &l2cm_l2cap_tx_handler);
}

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */
/* function starts */
void l2cm_init(bool reset)
{
#if (BLE_L2CC)
    if (!reset)
    {
        // Create HCI TX kernel event
        ke_event_callback_set(KE_EVENT_L2CAP_TX, &l2cm_l2cap_tx_handler);
    }

    ke_event_clear(KE_EVENT_L2CAP_TX);
#endif // (BLE_L2CC)

    /* initialize global variables */
    memset(&l2cm_env, 0, sizeof(l2cm_env));


    // initialize L2CC task
    l2cc_init(reset);
}

void l2cm_create(uint8_t conidx)
{
    /* Start the L2CC process */
    l2cc_create(conidx);
}

void l2cm_cleanup(uint8_t conidx)
{
    /* clear the tx bit */
    l2cm_tx_status(conidx, false);
    /* cleanup the L2CC task at conidx */
    l2cc_cleanup(conidx, false);

    // only recovery if no connection
    if (l2cc_conn_count_get() == 0)
    {
        // Recovery nb completed
        l2cm_env.nb_buffer_avail = l2cm_env.le_acl_total_nb_acl_pkt;
    }
}

/* function starts */
void l2cm_set_link_layer_buff_size(uint16_t pkt_len, uint16_t nb_acl)
{
    l2cm_env.le_acl_data_pkt_len = pkt_len;
    l2cm_env.le_acl_total_nb_acl_pkt = nb_acl;
    l2cm_env.nb_buffer_avail = l2cm_env.le_acl_total_nb_acl_pkt;
}

uint16_t l2cm_get_nb_buffer_available(void)
{
    return l2cm_env.nb_buffer_avail;
}

void l2cm_tx_status(uint8_t conidx, bool busy)
{
    // mark tx activity busy or not for specific connection
    if (busy)
    {
        // check if state is updated
        if ((l2cm_env.con_tx_state & ((uint32_t)(1 << conidx))) == 0)
        {
            if (l2cm_env.nb_buffer_avail > 0)
            {
                // inform that TX has to be performed
                l2cm_env.con_tx_state |= ((uint32_t)(1 << conidx));
                ke_event_set(KE_EVENT_L2CAP_TX);
                return;
            }
        }
        l2cm_env.con_tx_state |= ((uint32_t)(1 << conidx));
    }
    else
    {
        l2cm_env.con_tx_state &= ~((uint32_t)(1 << conidx));
    }
}

#endif // #if (BLE_L2CM)
/// @} L2CM
