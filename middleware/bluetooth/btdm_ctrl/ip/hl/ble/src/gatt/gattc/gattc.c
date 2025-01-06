/**
 ****************************************************************************************
 *
 * @file gattc.c
 *
 * @brief Generic Attribute Profile Controller implementation.
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup GATTC
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"

#if (BLE_CENTRAL || BLE_PERIPHERAL)
#include "co_math.h"
#include <stdint.h>

#include "gattc_int.h"
#include "gattc_task.h"
#include "gapc.h"

#include "l2cc_task.h"

#include "co_utils.h"


#include "ke_mem.h"
#include "ke_timer.h"
#include "ke_task.h"

/* ATT Client and server */
#include "attm.h"
#include "../attc/attc.h" // Access to internal API required
#include "../atts/atts.h" // Access to internal API required

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
struct gattc_env_tag *gattc_env[GATTC_IDX_MAX];

/// GATT Controller task descriptor
extern const struct ke_task_desc TASK_DESC_GATTC;

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/**
 * Clean-up operation parameters
 *
 * @param[in] conidx   Connection index
 * @param[in] op_type  Operation Type
 */
static void gattc_operation_cleanup(uint8_t conidx, uint8_t op_type)
{
#if (BLE_ATTC)
    if (op_type == GATTC_OP_CLIENT)
    {
        // clean-up read response list
        while (!(co_list_is_empty(&(gattc_env[conidx]->client.rsp_list))))
        {
            struct co_list_hdr *head = co_list_pop_front(&(gattc_env[conidx]->client.rsp_list));
            ke_msg_free((struct ke_msg const *)head);
        }
    }
    else if (op_type == GATTC_OP_SDP)
    {
        // clean-up SDP data
        while (!(co_list_is_empty(&(gattc_env[conidx]->client.sdp_data))))
        {
            struct co_list_hdr *head = co_list_pop_front(&(gattc_env[conidx]->client.sdp_data));
            ke_free(head);
        }
    }
#endif // (BLE_ATTC)

    // clean-up operation
    if (gattc_env[conidx]->operation[op_type] != NULL)
    {
        // protection to double free of operation if message already present in heap.
        if (!ke_msg_in_queue(gattc_env[conidx]->operation[op_type]))
        {
            KE_MSG_FREE(gattc_env[conidx]->operation[op_type]);
        }
        gattc_env[conidx]->operation[op_type] = NULL;
    }
}



void gattc_init(bool reset)
{
    // Index
    uint8_t conidx;

    if (!reset)
    {
        // Create GATT task
        ke_task_create(TASK_GATTC, &TASK_DESC_GATTC);

        // Initialize GATT controllers environment variable
        for (conidx = 0; conidx < GATTC_IDX_MAX; conidx++)
        {
            gattc_env[conidx] = NULL;
        }
    }
    // Initialize GATT controllers
    for (conidx = 0; conidx < GATTC_IDX_MAX; conidx++)
    {
        /* clean environment variables */
        gattc_cleanup(conidx, false);
    }
}

void gattc_create(uint8_t conidx)
{
    // allocate environment variable for connection
    gattc_env[conidx] = (struct gattc_env_tag *)ke_malloc(sizeof(struct gattc_env_tag), KE_MEM_ENV);
    ASSERT_ERR(gattc_env[conidx] != NULL);

    // clean-up structure
    memset(gattc_env[conidx], 0, sizeof(struct gattc_env_tag));
#if (BLE_ATTC)
    // initialize lists
    co_list_init(&(gattc_env[conidx]->client.rsp_list));
    co_list_init(&(gattc_env[conidx]->client.reg_evt));
    co_list_init(&(gattc_env[conidx]->client.sdp_data));
#endif // (BLE_ATTC)

#if (BLE_ATTS)
    // initialize lists
    co_list_init(&gattc_env[conidx]->server.prep_wr_req_list);
    co_list_init(&gattc_env[conidx]->server.rsp);
    co_list_init(&gattc_env[conidx]->server.pdu_queue);
    gattc_env[conidx]->server.read_cache = NULL;
#endif // (BLE_ATTS)

    // This will be changed according to the MTU negotiation
    gattc_env[conidx]->mtu_size      = (uint16_t)ATT_DEFAULT_MTU;
    gattc_env[conidx]->trans_timeout = false;
    gattc_env[conidx]->mtu_exch      = false;

    // put current task in connection ready state.
    ke_state_set(KE_BUILD_ID(TASK_GATTC, conidx), GATTC_READY);

    // forbid any ATTS request until connection is completed
    gattc_update_state(conidx, GATTC_CONNECTED, true);
}

void gattc_con_enable(uint8_t conidx)
{
    // Connection procedure is finished, allows ATTS peer requests
    gattc_update_state(conidx, GATTC_CONNECTED, false);

#if (BLE_ATTS)
    // resume PDU procedure execution
    atts_process_pdu(conidx);
#endif // (BLE_ATTS)
}


void gattc_cleanup(uint8_t conidx, bool disconnect)
{
    // cleanup environment variable for connection
    if (gattc_env[conidx] != NULL)
    {
        uint8_t i;
        for (i = 0 ; i < GATTC_OP_MAX; i++)
        {
            if (disconnect)
            {
                gattc_send_complete_evt(conidx, i, GAP_ERR_DISCONNECTED);
            }
            else
            {
                //  clean-up parameters if needed
                gattc_operation_cleanup(conidx, i);
            }

#if (BLE_ATTC)
            // clean-up registered client
            while (!(co_list_is_empty(&(gattc_env[conidx]->client.reg_evt))))
            {
                struct co_list_hdr *head = co_list_pop_front(&(gattc_env[conidx]->client.reg_evt));
                ke_free(head);
            }
#endif // (BLE_ATTC)
        }

#if (BLE_ATTS)
        /* free temporary data  used for prepare write */
        atts_clear_prep_data(conidx);

        // clean-up message under ATTS queue
        while (!(co_list_is_empty(&(gattc_env[conidx]->server.pdu_queue))))
        {
            struct co_list_hdr *msg = co_list_pop_front(&(gattc_env[conidx]->server.pdu_queue));
            ke_msg_free((struct ke_msg *)msg);
        }

        // Clear Response data
        atts_clear_rsp_data(conidx);

        /* free read attribute cache */
        atts_clear_read_cache(conidx);
#endif // (BLE_ATTS)

        ke_free(gattc_env[conidx]);
        gattc_env[conidx] = NULL;
    }

    // clear timers
#if (BLE_ATTC)
    ke_timer_clear(GATTC_CLIENT_RTX_IND, KE_BUILD_ID(TASK_GATTC, conidx));
#endif // (BLE_ATTC)
#if (BLE_ATTS)
    ke_timer_clear(GATTC_SERVER_RTX_IND, KE_BUILD_ID(TASK_GATTC, conidx));
#endif // (BLE_ATTS)
    /* set state to Free state */
    ke_state_set(KE_BUILD_ID(TASK_GATTC, conidx), GATTC_FREE);

}

uint16_t gattc_get_mtu(uint8_t idx)
{
    return (gattc_env[idx]->mtu_size);
}


void gattc_set_mtu(uint8_t idx, uint16_t mtu)
{
    // Provides protection to invalid MTU set by peer
    gattc_env[idx]->mtu_size = co_max(L2C_MIN_LE_MTUSIG, co_min(mtu, GAP_MAX_LE_MTU));
    // mark MTU exchanged
    gattc_env[idx]->mtu_exch = true;

    // Inform application that MTU Has Been changed
    struct gattc_mtu_changed_ind *mtu_ind = KE_MSG_ALLOC(GATTC_MTU_CHANGED_IND, gapc_get_dest_task(idx),
                                            KE_BUILD_ID(TASK_GATTC, idx), gattc_mtu_changed_ind);
    mtu_ind->mtu = gattc_env[idx]->mtu_size;

    // fill sequence number if needed
#if (BLE_ATTC)
    if (gattc_get_operation(idx, GATTC_OP_CLIENT) == GATTC_MTU_EXCH)
    {
        // Force clear mtu for speicial case
        ke_timer_clear(GATTC_CLIENT_RTX_IND, KE_BUILD_ID(TASK_GATTC, idx));
        // retrieve operation
        struct gattc_exc_mtu_cmd *cmd = GATT_OPERATION_CMD(idx, GATTC_OP_CLIENT, gattc_exc_mtu_cmd);
        mtu_ind->seq_num = cmd->seq_num;
        gattc_send_complete_evt(idx, GATTC_OP_CLIENT, ATT_ERR_NO_ERROR);
    }
    else
#endif //(BLE_ATTC)
    {
        mtu_ind->seq_num = 0;
    }

    ke_msg_send(mtu_ind);
}


void gattc_send_complete_evt(uint8_t conidx, uint8_t op_type, uint8_t status)
{
    if (gattc_env[conidx]->operation[op_type] != NULL)
    {
        // send command completed event
        struct gattc_cmp_evt *cmp_evt = KE_MSG_ALLOC(GATTC_CMP_EVT, gattc_get_requester(conidx, op_type),
                                        KE_BUILD_ID(TASK_GATTC, conidx), gattc_cmp_evt);

        /* fill up the parameters */
        cmp_evt->operation = *((uint8_t *) gattc_env[conidx]->operation[op_type]);
        cmp_evt->status = status;
        cmp_evt->seq_num = ((struct gattc_op_cmd *) gattc_env[conidx]->operation[op_type])->seq_num;

        /* send the complete event */
        ke_msg_send(cmp_evt);
    }

    // clean-up operation parameters
    gattc_operation_cleanup(conidx, op_type);

    // set state to ready
    gattc_update_state(conidx, (1 << op_type), false);
}


void gattc_send_error_evt(uint8_t conidx, uint8_t operation, uint16_t seq_num, const ke_task_id_t requester, uint8_t status)
{
    // prepare command completed event with error status
    struct gattc_cmp_evt *cmp_evt = KE_MSG_ALLOC(GATTC_CMP_EVT,
                                    requester, KE_BUILD_ID(TASK_GATTC, conidx), gattc_cmp_evt);

    cmp_evt->operation = operation;
    cmp_evt->status = status;
    cmp_evt->seq_num = seq_num;

    // send event
    ke_msg_send(cmp_evt);
}

uint8_t gattc_get_operation(uint8_t conidx, uint8_t op_type)
{
    // by default no operation
    uint8_t ret = GATTC_NO_OP;

    ASSERT_ERR(conidx < GATTC_IDX_MAX);
    ASSERT_ERR(op_type < GATTC_OP_MAX);

    // check if an operation is registered
    if (gattc_env[conidx]->operation[op_type] != NULL)
    {
        // operation code if first by of an operation command
        ret = (*((uint8_t *) gattc_env[conidx]->operation[op_type]));
    }

    return ret;
}

uint16_t gattc_get_op_seq_num(uint8_t conidx, uint8_t op_type)
{
    // by default no operation
    uint16_t ret = 0;

    ASSERT_ERR(conidx < GATTC_IDX_MAX);
    ASSERT_ERR(op_type < GATTC_OP_MAX);

    // check if an operation is registered
    if (gattc_env[conidx]->operation[op_type] != NULL)
    {
        // operation code if first by of an operation command
        ret = ((struct gattc_op_cmd *) gattc_env[conidx]->operation[op_type])->seq_num;
    }

    return ret;
}

void *gattc_get_operation_ptr(uint8_t conidx, uint8_t op_type)
{
    ASSERT_ERR(conidx  < GATTC_IDX_MAX);
    ASSERT_ERR(op_type < GATTC_OP_MAX);
    // return operation pointer
    return gattc_env[conidx]->operation[op_type];
}

void gattc_set_operation_ptr(uint8_t conidx, uint8_t op_type, void *op)
{
    ASSERT_ERR(conidx  < GATTC_IDX_MAX);
    ASSERT_ERR(op_type < GATTC_OP_MAX);
    // set operation pointer
    gattc_env[conidx]->operation[op_type] = op;
}

bool gattc_reschedule_operation(uint8_t conidx, uint8_t op_type)
{
    bool ret = false;

    ASSERT_ERR(conidx  < GATTC_IDX_MAX);
    ASSERT_ERR(op_type < GATTC_OP_MAX);
    // check if operation not null
    if (gattc_env[conidx]->operation[op_type] != NULL)
    {
        ret = true;
        // request kernel to reschedule operation with operation source ID
        ke_msg_forward(gattc_env[conidx]->operation[op_type],
                       KE_BUILD_ID(TASK_GATTC, conidx), ke_msg_src_id_get(gattc_env[conidx]->operation[op_type]));
    }

    return ret;
}


ke_task_id_t gattc_get_requester(uint8_t conidx, uint8_t op_type)
{
    ke_task_id_t ret = 0;
    ASSERT_ERR(conidx  < GATTC_IDX_MAX);
    ASSERT_ERR(op_type < GATTC_OP_MAX);

    // check if an operation is registered
    if (gattc_env[conidx]->operation[op_type] != NULL)
    {
        // retrieve operation requester.
        ret = ke_msg_src_id_get(gattc_env[conidx]->operation[op_type]);
    }

    return ret;
}


void gattc_update_state(uint8_t conidx, ke_state_t state, bool busy)
{
    ke_state_t old_state  = ke_state_get(KE_BUILD_ID(TASK_GATTC, conidx));
    if (busy)
    {
        // set state to busy
        ke_state_set(KE_BUILD_ID(TASK_GATTC, conidx), old_state | state);
    }
    else
    {
        // set state to idle
        ke_state_set(KE_BUILD_ID(TASK_GATTC, conidx), old_state & ~(state));
    }
}


#endif /* #if (BLE_CENTRAL || BLE_PERIPHERAL) */
/// @} GATTC
