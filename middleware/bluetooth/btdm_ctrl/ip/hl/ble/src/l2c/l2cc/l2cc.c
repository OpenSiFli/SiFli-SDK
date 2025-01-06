/**
 ****************************************************************************************
 *
 * @file l2cc.c
 *
 * @brief L2CAP Controller implementation.
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup L2CC
 * @{
 ****************************************************************************************
 */
#include "rwip_config.h"

#if (BLE_L2CC)

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "l2cc_int.h"
#include "l2cc_pdu_int.h"
#include "l2cc_task.h"
#include "l2cc_lecb.h"
#include "../l2cm/l2cm_int.h" // Internal API required

#include "ke_timer.h"
#include "ke_mem.h"

#include "gapc.h"
#include "hci.h"

#include "co_math.h"
#include "co_utils.h"

#if BLE_EMB_PRESENT
    #include "ble_util_buf.h"     // stack buffering
    #include "em_map.h"
#endif //#if BLE_EMB_PRESENT

#include <string.h>

#include "dbg.h"

/*
 * TASK DESCRIPTOR DECLARATIONS
 ****************************************************************************************
 */



/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// L2CAP environments pool
struct l2cc_env_tag *l2cc_env[L2CC_IDX_MAX];

/// L2CC task descriptor
extern const struct ke_task_desc TASK_DESC_L2CC;

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void l2cc_init(bool reset)
{
    // Index
    uint8_t conidx;

    if (!reset)
    {
        // Create L2CC task
        ke_task_create(TASK_L2CC, &TASK_DESC_L2CC);

        // Initialize env structure
        for (conidx = 0 ; conidx < L2CC_IDX_MAX ; conidx++)
        {
            l2cc_env[conidx] = NULL;
        }
    }
    else
    {
        // Initialize L2Cap controllers
        for (conidx = 0; conidx < L2CC_IDX_MAX; conidx++)
        {
            // clean environment variables
            l2cc_cleanup(conidx, true);
        }
    }
}

void l2cc_create(uint8_t conidx)
{
    struct l2cc_env_tag *env;

    ASSERT_ERR(l2cc_env[conidx] == NULL);

    // set device into a non free state
    ke_state_set(KE_BUILD_ID(TASK_L2CC, conidx), L2CC_READY);


    // Allocate environment variable for connection
    env = (struct l2cc_env_tag *)ke_malloc(sizeof(struct l2cc_env_tag), KE_MEM_ENV);

    ASSERT_ERR(env != NULL);

    if (env != NULL)
    {
        /// initialize operation pointer
        uint8_t i;
        for (i = 0 ; i < L2CC_OP_MAX ; i++)
        {
            env->operation[i] = NULL;
        }

        // Clean-up structure
        co_list_init(&env->tx_queue);
        env->rx_buffer = NULL;

#if (BLE_LECB)
        // Initialization of the message list
        co_list_init(&env->lecb_list);
#endif //  (BLE_LECB)

        // Initialization of message reception
        env->rx_cursor = 0;
        env->rx_exp_len    = L2C_HEADER_LEN;
    }

    l2cc_env[conidx] = env;
}

void l2cc_cleanup(uint8_t conidx, bool reset)
{
    struct l2cc_env_tag *env = l2cc_env[conidx];

    // cleanup environment variable for connection
    if (l2cc_env[conidx] != NULL)
    {
        /// clean-up operations
        ke_timer_clear(L2CC_SIGNALING_TRANS_TO_IND, KE_BUILD_ID(TASK_L2CC, conidx));

        uint8_t i;
        for (i = 0 ; i < L2CC_OP_MAX ; i++)
        {
            if (env->operation[i] != NULL)
            {
                /// initialize operation pointer
                if (!reset)
                {
                    // completed operation
                    l2cc_op_complete(conidx, i, GAP_ERR_DISCONNECTED);
                }
                else
                {
                    struct ke_msg *cmd = ke_param2msg(env->operation[i]);
                    // clean-up operation parameters
                    ke_msg_free(cmd);
                }

            }
        }

#if (BLE_LECB)
        // free environment variable
        while (!co_list_is_empty(&(env->lecb_list)))
        {
            struct l2cc_lecb_info *lecb = (struct l2cc_lecb_info *) co_list_pick(&(env->lecb_list));

            l2cc_lecb_free(conidx, lecb, !reset);
        }
#endif // (BLE_LECB)

        // clean-up message which is received
        if (env->rx_buffer != NULL)
        {
            ke_free(env->rx_buffer);
        }

        // free environment variable
        while (!co_list_is_empty(&(env->tx_queue)))
        {
            struct ke_msg *msg = (struct ke_msg *) co_list_pop_front(&(env->tx_queue));

            if (!reset)
            {
                if (msg->id == L2CC_PDU_SEND_CMD)
                {
                    struct l2cc_pdu_send_cmd *pdu = (struct l2cc_pdu_send_cmd *) ke_msg2param(msg);
                    l2cc_send_cmp_evt(conidx, L2CC_PDU_SEND, msg->src_id, GAP_ERR_DISCONNECTED, pdu->pdu.chan_id, 0);
                }
#if (RW_DEBUG)
                else if (msg->id == L2CC_DBG_PDU_SEND_CMD)
                {
                    struct l2cc_dbg_pdu_send_cmd *dbg_pdu = (struct l2cc_dbg_pdu_send_cmd *) ke_msg2param(msg);
                    l2cc_send_cmp_evt(conidx, L2CC_DBG_PDU_SEND, msg->src_id, GAP_ERR_DISCONNECTED, co_read16p(dbg_pdu->pdu.data), 0);
                }
#endif // (RW_DEBUG)
                else
                {
                    // not expected
                    ASSERT_WARN(0, conidx, msg->id);
                }
            }

            ke_free(msg);
        }

        ke_free(l2cc_env[conidx]);
        l2cc_env[conidx] = NULL;
    }

    // Set Free state
    ke_state_set(KE_BUILD_ID(TASK_L2CC, conidx), L2CC_FREE);
}

uint8_t l2cc_conn_count_get()
{
    uint8_t count = 0;
    for (uint8_t i = 0; i < L2CC_IDX_MAX; i++)
    {
        if (l2cc_env[i] != NULL)
        {
            count++;
        }
    }
    return count;
}

void l2cc_update_state(uint8_t conidx, ke_state_t state, bool busy)
{
    // change state only if there is no ongoing disconnect.
    ke_state_t old_state  = ke_state_get(KE_BUILD_ID(TASK_L2CC, conidx));
    if (busy)
    {
        // set state to busy
        ke_state_set(KE_BUILD_ID(TASK_L2CC, conidx), old_state | state);
    }
    else
    {
        // set state to idle
        ke_state_set(KE_BUILD_ID(TASK_L2CC, conidx), old_state & ~(state));
    }
}

void l2cc_op_complete(uint8_t conidx, uint8_t op_type, uint8_t status)
{
    struct l2cc_env_tag *env = l2cc_env[conidx];
    if ((env != NULL) && (op_type < L2CC_OP_MAX))
    {
        void *operation    = env->operation[op_type];

        if (operation != NULL)
        {
            struct ke_msg *cmd = ke_param2msg(operation);

            // send command completed event
            l2cc_send_cmp_evt(conidx, *((uint8_t *) operation), cmd->src_id, status, L2C_CID_LE_SIGNALING, 0);

            // clean-up operation parameters
            ke_msg_free(cmd);

            env->operation[op_type] = NULL;
        }

        // set state to ready
        l2cc_update_state(conidx, (1 << op_type), false);
    }
}


void l2cc_send_cmp_evt(uint8_t conidx, uint8_t operation, const ke_task_id_t requester, uint8_t status, uint16_t cid,
                       uint16_t credit)
{
    // prepare command completed event with error status
    struct l2cc_cmp_evt *cmp_evt = KE_MSG_ALLOC(L2CC_CMP_EVT, requester, KE_BUILD_ID(TASK_L2CC, conidx), l2cc_cmp_evt);

    cmp_evt->operation = operation;
    cmp_evt->status    = status;
    cmp_evt->credit    = credit;
    cmp_evt->cid       = cid;

    // send event
    ke_msg_send(cmp_evt);
}

uint8_t l2cc_get_operation(uint8_t conidx, uint8_t op_type)
{
    struct l2cc_env_tag *env = l2cc_env[conidx];
    // by default no operation
    uint8_t ret = L2CC_NO_OP;

    ASSERT_ERR(conidx < L2CC_IDX_MAX);
    ASSERT_ERR(op_type < L2CC_OP_MAX);

    // check if an operation is registered
    if (env->operation[op_type] != NULL)
    {
        // operation code if first by of an operation command
        ret = (*((uint8_t *) env->operation[op_type]));
    }

    return ret;
}

void *l2cc_get_operation_ptr(uint8_t conidx, uint8_t op_type)
{
    struct l2cc_env_tag *env = l2cc_env[conidx];
    ASSERT_ERR(conidx  < L2CC_IDX_MAX);
    ASSERT_ERR(op_type < L2CC_OP_MAX);
    // return operation pointer
    return env->operation[op_type];
}

void l2cc_set_operation_ptr(uint8_t conidx, uint8_t op_type, void *op)
{
    struct l2cc_env_tag *env = l2cc_env[conidx];
    ASSERT_ERR(conidx  < L2CC_IDX_MAX);
    ASSERT_ERR(op_type < L2CC_OP_MAX);
    // set operation pointer
    env->operation[op_type] = op;
}


void l2cc_data_send(uint8_t conidx, uint8_t nb_buffer)
{
    struct l2cc_env_tag *env = l2cc_env[conidx];

    if (env != NULL)
    {
        // L2CC Environment
        struct ke_msg *pkt = (struct ke_msg *) co_list_pick(&(env->tx_queue));

        bool process_pdu = (pkt != NULL) && (nb_buffer > 0);

        while (process_pdu)
        {
            // Status of the request
            uint8_t  status = GAP_ERR_NO_ERROR;
            // Length of the packet
            uint16_t tx_length = 0;
            // Allocate TX Buffer
            uint8_t *buffer;
            // Packet Boundary flag
            uint8_t  pb_flag = PBF_CONT_HL_FRAG;

            // ************ Allocate a TX Buffer
#if BLE_EMB_PRESENT
            uint16_t buf_ptr = ble_util_buf_acl_tx_alloc();
            // This should not happen
            ASSERT_ERR(buf_ptr != 0);
            // Retrieve buffer
            buffer = (uint8_t *)(EM_BASE_ADDR + ((uint32_t)buf_ptr));
#else // (BLE_HOST_PRESENT)
            // allocate buffer in message heap
            buffer = ke_malloc(l2cm_get_buffer_size() + HCI_ACL_HDR_LEN + HCI_TRANSPORT_HDR_LEN, KE_MEM_KE_MSG);
            // move pointer to payload beginning
            buffer += HCI_ACL_HDR_LEN + HCI_TRANSPORT_HDR_LEN;
#endif // (BLE_EMB_PRESENT) / (BLE_HOST_PRESENT)

            // ************* Pack PDU into allocated buffer
            switch (pkt->id)
            {
            case L2CC_PDU_SEND_CMD:
            {
                struct l2cc_pdu_send_cmd *pdu_cmd = (struct l2cc_pdu_send_cmd *) ke_msg2param(pkt);
                struct l2cc_pdu *pdu = (struct l2cc_pdu *) & (pdu_cmd->pdu);

                // calculate LLID - an error here
                pb_flag    = ((pdu_cmd->offset == 0) ? PBF_1ST_NF_HL_FRAG : PBF_CONT_HL_FRAG);

                // Pack the PDU
                status = l2cc_pdu_pack(pdu, &(pdu_cmd->offset), &tx_length, buffer, &pb_flag);

                // Check if another packet need to be sent or if an error has occurred
                if ((status != GAP_ERR_NO_ERROR) || (pdu->payld_len == 0))
                {
                    // send command completed event
                    l2cc_send_cmp_evt(conidx, L2CC_PDU_SEND, pkt->src_id, status, pdu->chan_id, 0);

                    co_list_pop_front(&env->tx_queue);

                    ke_msg_free(pkt);
                    process_pdu = false;
                }
            }
            break;

#if (BLE_LECB)
            case L2CC_LECB_SDU_SEND_CMD:
            {
                struct l2cc_lecb_sdu_send_cmd *sdu_cmd = (struct l2cc_lecb_sdu_send_cmd *) ke_msg2param(pkt);
                struct l2cc_sdu *sdu = (struct l2cc_sdu *) & (sdu_cmd->sdu);

                struct l2cc_lecb_info *lecb = l2cc_lecb_find(conidx, L2CC_LECB_PEER_CID, sdu->cid);

                // size of segment that have to be transmitted
                uint16_t rem_seg_len = 0;
                // size of segment already sent
                uint16_t tx_seg_len = 0;

                if ((lecb == NULL) || (!GETB(lecb->state, L2CC_LECB_CONNECTED)))
                {
                    status = L2C_ERR_INVALID_CID;
                }
                else
                {
                    // New SDU to send
                    if (sdu_cmd->offset == 0)
                    {
                        rem_seg_len = co_min(sdu->length + L2C_SDU_LEN, lecb->peer_mps);
                    }
                    else
                    {
                        tx_seg_len = ((sdu_cmd->offset + L2C_SDU_LEN) % lecb->peer_mps);

                        // Recalculate length to be copied
                        rem_seg_len = co_min(sdu->length - sdu_cmd->offset, (lecb->peer_mps - tx_seg_len));
                    }

                    // New Segment to send
                    if (tx_seg_len == 0)
                    {
                        ASSERT_ERR(lecb->peer_credit != 0);

                        // Decrease credit
                        lecb->peer_credit--;
                        sdu->credit++;
                    }
                }

                if (status == GAP_ERR_NO_ERROR)
                {
                    // Pack the PDU
                    pb_flag    = ((tx_seg_len == 0) ? PBF_1ST_NF_HL_FRAG : PBF_CONT_HL_FRAG);
                    status     = l2cc_lecb_pdu_pack(conidx, sdu, &tx_length,
                                                    buffer, &(sdu_cmd->offset), rem_seg_len, pb_flag);
                }

                // Check if another packet need to be sent or if an error has occurred
                if ((status != GAP_ERR_NO_ERROR) || (sdu_cmd->offset == sdu->length))
                {
                    uint8_t cid = (lecb != NULL) ? lecb->local_cid : sdu->cid;

                    // send command completed event
                    l2cc_send_cmp_evt(conidx, L2CC_LECB_SDU_SEND, pkt->src_id, status, cid, sdu->credit);

                    co_list_pop_front(&env->tx_queue);

                    ke_msg_free(pkt);

                    if (lecb != NULL)
                    {
                        lecb->tx_sdu = NULL;
                    }
                    process_pdu = false;
                }
                else if (lecb->peer_credit == 0)
                {
                    // mark that buffer is waiting for new credits
                    SETB(lecb->state, L2CC_LECB_TX_WAIT, true);

                    // remove message from tx queue
                    co_list_pop_front(&env->tx_queue);
                    process_pdu = false;
                }
            }
            break;
#endif // (BLE_LECB)

#if (RW_DEBUG)
            case L2CC_DBG_PDU_SEND_CMD:
            {
                struct l2cc_dbg_pdu_send_cmd *dbg_pdu_cmd = (struct l2cc_dbg_pdu_send_cmd *) ke_msg2param(pkt);
                struct l2cc_dbg_pdu *dbg_pdu = (struct l2cc_dbg_pdu *) & (dbg_pdu_cmd->pdu);

                // Pack the PDU
                status = l2cc_dbg_pdu_pack(dbg_pdu, &tx_length, buffer, &(dbg_pdu_cmd->offset), &pb_flag);

                // Check if another packet need to be sent or if an error has occurred
                if ((status != GAP_ERR_NO_ERROR) || (dbg_pdu->length == 0))
                {
                    // send command completed event
                    l2cc_send_cmp_evt(conidx, L2CC_DBG_PDU_SEND, pkt->src_id, status, co_read16p(dbg_pdu->data), 0);

                    co_list_pop_front(&env->tx_queue);

                    ke_msg_free(pkt);
                    process_pdu = false;
                }
            }
            break;
#endif // (RW_DEBUG)

            default:
            {
                ASSERT_INFO(0, pkt->id, conidx);
            }
            break;
            }


            if (status == GAP_ERR_NO_ERROR)
            {
                // Allocate the message for L2CC task
                struct hci_acl_data *data_tx = KE_MSG_ALLOC(HCI_ACL_DATA, gapc_get_conhdl(conidx), KE_BUILD_ID(TASK_L2CC, conidx),
                                               hci_acl_data);

                // Fill packet length
                data_tx->length      = tx_length;

                // create an assert when the tx length is equals to zero
                ASSERT_INFO(tx_length != 0, tx_length, pkt->id);

                // Fill buffer pointer
#if (BLE_EMB_PRESENT)
                data_tx->buf_ptr = ((uint32_t)buffer) - EM_BASE_ADDR;
#else // (BLE_HOST_PRESENT)
                data_tx->buf_ptr = ((uint32_t)buffer);
#endif // (BLE_EMB_PRESENT) / (BLE_HOST_PRESENT)

                // Fill connection handle + packet boundary and broadcast flags
                SETF(data_tx->conhdl_pb_bc_flag, HCI_ACL_HDR_HDL,     gapc_get_conhdl(conidx));
                SETF(data_tx->conhdl_pb_bc_flag, HCI_ACL_HDR_PB_FLAG, pb_flag);
                SETF(data_tx->conhdl_pb_bc_flag, HCI_ACL_HDR_BC_FLAG, BCF_P2P);

                //Trace the tx packet
                TRC_REQ_L2CAP_TX(GETF(data_tx->conhdl_pb_bc_flag, HCI_ACL_HDR_HDL), data_tx->length, data_tx->buf_ptr);

                // Send message
                hci_send_2_controller(data_tx);

                // Decrement the number of available buffers
                l2cm_buffer_acquire();
                nb_buffer--;
            }
            else
            {
                // Just ignore to send the packet
#if (BLE_EMB_PRESENT)
                ble_util_buf_acl_tx_free(buf_ptr);
#else// (BLE_HOST_PRESENT)
                ke_free(buffer - (HCI_ACL_HDR_LEN + HCI_TRANSPORT_HDR_LEN));
#endif // (BLE_EMB_PRESENT) / (BLE_HOST_PRESENT)
            }

            // Check if there are other available buffers
            if (nb_buffer == 0)
            {
                process_pdu = false;
            }
        }

        // set if tx buffer queue is empty or not
        l2cm_tx_status(conidx, ! co_list_is_empty(&(env->tx_queue)));
    }
}



#endif //(BLE_L2CC)

/// @} L2CC
