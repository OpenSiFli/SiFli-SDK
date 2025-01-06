/**
 ****************************************************************************************
 *
 * @file attc.c
 *
 * @brief Attribute Client implementation.
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup ATTC
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"

#if (BLE_CENTRAL || BLE_PERIPHERAL)

#include <stdint.h>
#include <string.h>

#include "co_error.h"
#include "co_utils.h"
#include "co_math.h"

#include "ke_timer.h"
#include "ke_mem.h"
#include "ke_msg.h"
#include "ke_task.h"

#include "attm.h"
#include "attc.h"
#include "gapm.h"
#include "gapc.h"

#include "l2cc_pdu.h"
#include "l2cc_task.h"
#include "../gattc/gattc_int.h" // Access to Internal API required

#if (BLE_ATTC)


/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Get the event message destination
 *
 * @param[in] conidx         Connection index
 * @param[in] handle         Attribute handle
 *
 * @return task identifier
 *
 ****************************************************************************************
 */
static ke_task_id_t attc_get_event_dest(uint8_t conidx, uint16_t handle)
{
    struct attc_register_evt *reg_evt =
        (struct attc_register_evt *) co_list_pick(&(gattc_env[conidx]->client.reg_evt));
    // browse list of event to check if range already registered
    while (reg_evt != NULL)
    {
        // check if element is in expected range
        if ((reg_evt->start_hdl <= handle) && (reg_evt->end_hdl >= handle))
        {
            return reg_evt->task;
        }

        // go to next element
        reg_evt = (struct attc_register_evt *) reg_evt->hdr.next;
    }

    return gapc_get_dest_task(conidx);
}


/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void attc_send_hdl_cfm(uint8_t conidx)
{
    struct l2cc_att_hdl_val_cfm *cfm = ATTC_ALLOCATE_ATT_REQ(conidx,
                                       L2C_CODE_ATT_HDL_VAL_CFM, l2cc_att_hdl_val_cfm, 0);

    /* send the indication confirmation  message */
    attc_send_att_req(conidx, cfm);
}

void attc_send_execute(uint8_t conidx, uint8_t flag)
{
    struct l2cc_att_exe_wr_req *req = ATTC_ALLOCATE_ATT_REQ(conidx,
                                      L2C_CODE_ATT_EXE_WR_REQ, l2cc_att_exe_wr_req, 0);

    /* fill up the parameters */
    req->flags = flag;

    /* send the execute write message */
    attc_send_att_req(conidx, req);
}

void attc_send_att_req(uint8_t conidx, void *pdu)
{
    // retrieve PDU pointer
    struct l2cc_pdu_send_cmd *pkt = L2CC_PDU_TO_CMD(pdu);

    /* do not start the timer for write command, mtu exchange and
     * write characteristic cancellation
     */
    if (!((pkt->pdu.data.code  == L2C_CODE_ATT_SIGN_WR_CMD)
            || (pkt->pdu.data.code  == L2C_CODE_ATT_WR_CMD)
            || (pkt->pdu.data.code  == L2C_CODE_ATT_HDL_VAL_CFM)))
    {
        /* start the timer; will destroy the link if it expires */
        ke_timer_set(GATTC_CLIENT_RTX_IND, KE_BUILD_ID(TASK_GATTC, conidx), ATT_TRANS_RTX);
    }

    /* send the L2CC message */
    l2cc_pdu_send(pdu);
}

void attc_send_read_ind(uint8_t conidx)
{
    // Check if read response can be sent
    if (!co_list_is_empty(&(gattc_env[conidx]->client.rsp_list)))
    {
        // retrieve operation
        struct gattc_read_cmd *read_cmd = (struct gattc_read_cmd *) gattc_get_operation_ptr(conidx, GATTC_OP_CLIENT);
        // get first pdu element
        struct ke_msg *msg = (struct ke_msg *) co_list_pick(&(gattc_env[conidx]->client.rsp_list));
        struct l2cc_pdu_recv_ind *pdu = (struct l2cc_pdu_recv_ind *) ke_msg2param(msg);
        // retrieve total length of data read
        uint16_t length = pdu->offset;
        uint16_t cursor = 0;

        // Create Read Indication message
        struct gattc_read_ind *read_ind =  KE_MSG_ALLOC_DYN(GATTC_READ_IND,
                                           gattc_get_requester(conidx, GATTC_OP_CLIENT), KE_BUILD_ID(TASK_GATTC, conidx),
                                           gattc_read_ind, length);

        // fill in parameters
        read_ind->length = length;
        read_ind->handle = read_cmd->req.simple.handle;
        read_ind->offset = read_cmd->req.simple.offset;

        if (pdu->pdu.data.code == L2C_CODE_ATT_RD_BY_TYPE_RSP)
        {
            // retrieve data info
            struct l2cc_att_rd_by_type_rsp *rd_data = &(pdu->pdu.data.rd_by_type_rsp);

            // Copy data
            memcpy(&(read_ind->value[cursor]), &(rd_data->data[ATT_HANDLE_LEN]), (rd_data->each_len - ATT_HANDLE_LEN));

            // update cursor
            cursor += (rd_data->each_len - ATT_HANDLE_LEN);

            // Update cursor offset value
            if (msg->hdr.next != NULL)
            {
                read_ind->offset -= (rd_data->each_len - ATT_HANDLE_LEN);
            }

            // go to next element
            msg = (struct ke_msg *) msg->hdr.next;
            // free pdu
            KE_MSG_FREE(pdu);
            // update next pdu value
            pdu = (struct l2cc_pdu_recv_ind *) ke_msg2param(msg);
        }

        while (msg != NULL)
        {
            // retrieve data info
            struct l2cc_att_rd_rsp *read_data = &(pdu->pdu.data.rd_rsp);

            // Copy data
            memcpy(&(read_ind->value[cursor]), read_data->value, read_data->value_len);

            // update cursor
            cursor += read_data->value_len;

            // Update cursor offset value
            if (msg->hdr.next != NULL)
            {
                read_ind->offset -= read_data->value_len;
            }

            // go to next element
            msg = (struct ke_msg *) msg->hdr.next;
            pdu = (struct l2cc_pdu_recv_ind *) ke_msg2param(msg);

            // free received pdus
            KE_MSG_FREE(L2CC_PDU_TO_CMD(read_data));
        }

        co_list_init(&(gattc_env[conidx]->client.rsp_list));

        ke_msg_send(read_ind);
    }
}


/*
 * PDU HANDLERS
 ****************************************************************************************
 */



/**
 ****************************************************************************************
 * @brief Handles response of MTU exchange
 * GATT Client response.
 * HL Message: GATT_EXC_MTU_RSP
 *
 * @param[in] conidx    Connection Index
 * @param[in] rsp       Pointer to the parameters of the message.
 *
 *
 ****************************************************************************************
 */





static int attc_exc_mtu_rsp_handler(uint8_t conidx, struct l2cc_att_mtu_rsp const *rsp)
{
    // Maximal MTU
    if (gattc_get_operation(conidx, GATTC_OP_CLIENT) == GATTC_MTU_EXCH)
    {
        uint16_t max_mtu = gapm_get_max_mtu();

        // Update MTU size
        gattc_set_mtu(conidx, co_min((uint32_t)max_mtu, (uint32_t)(rsp->mtu_size)));
#if 0
        switch (gattc_get_operation(conidx, GATTC_OP_CLIENT))
        {
        case GATTC_MTU_EXCH:
        {
            /* send the GATT command complete event to upper layer */
            gattc_send_complete_evt(conidx, GATTC_OP_CLIENT, ATT_ERR_NO_ERROR);
        }
        break;

        default:
        {
            /* send the GATT command complete event to upper layer */
            gattc_send_complete_evt(conidx, GATTC_OP_CLIENT, GAP_ERR_PROTOCOL_PROBLEM);
        }
        break;
        }
#endif
    }
    return (KE_MSG_CONSUMED);
}


/**
 ****************************************************************************************
 * @brief Handles find information response from ATTC.
 *
 * @param[in] conidx    Connection Index
 * @param[in] rsp       Pointer to the parameters of the message.
 *
 ****************************************************************************************
 */
static int attc_find_info_rsp_handler(uint8_t conidx, struct l2cc_att_find_info_rsp const *rsp)
{
    // check operation (first element of operation structure contains operation code
    switch (gattc_get_operation(conidx, GATTC_OP_CLIENT))
    {
    case GATTC_DISC_DESC_CHAR:
    {
        // retrieve operation
        struct gattc_disc_cmd *disc_cmd = GATT_OPERATION_CMD(conidx, GATTC_OP_CLIENT, gattc_disc_cmd);

        uint8_t uuid_len = ((rsp->format == ATT_FORMAT_16BIT_UUID)
                            ? ATT_UUID_16_LEN : ATT_UUID_128_LEN);
        uint16_t cursor = 0;
        uint16_t handle = ATT_INVALID_HANDLE;

        while (cursor < rsp->data_len)
        {
            struct gattc_disc_char_desc_ind *char_desc = KE_MSG_ALLOC_DYN(GATTC_DISC_CHAR_DESC_IND,
                    gattc_get_requester(conidx, GATTC_OP_CLIENT),
                    KE_BUILD_ID(TASK_GATTC, conidx), gattc_disc_char_desc_ind,
                    uuid_len);

            // retrieve handle
            handle = co_read16p(&(rsp->data[cursor]));
            cursor += ATT_HANDLE_LEN;
            char_desc->attr_hdl = handle;

            // retrieve uuid
            char_desc->uuid_len = uuid_len;
            memcpy(&(char_desc->uuid[0]), &(rsp->data[cursor]), uuid_len);
            cursor += uuid_len;

            // send the characteristics indication
            ke_msg_send(char_desc);
        }
        // increment search handle cursor
        handle ++;

        /* stop the search if the end has been detected */
        if ((handle > co_min(disc_cmd->end_hdl, ATT_MAX_ATTR_HDL))
                || (handle == ATT_INVALID_HANDLE))
        {
            /* send the GATT command complete event to upper layer */
            gattc_send_complete_evt(conidx, GATTC_OP_CLIENT, ATT_ERR_ATTRIBUTE_NOT_FOUND);
        }
        else
        {
            // continue discovery execution
            disc_cmd->start_hdl = handle;
            // put discovery command updated on the queue
            gattc_reschedule_operation(conidx, GATTC_OP_CLIENT);
        }
    }
    break;

    default:
    {
        /* send discovery complete event with error */
        gattc_send_complete_evt(conidx, GATTC_OP_CLIENT, GAP_ERR_INVALID_PARAM);
    }
    break;
    }
    return (KE_MSG_CONSUMED);
}


/**
 ****************************************************************************************
 * @brief Handles find by type response from ATTC.
 * GATT Server command.
 *
 * @param[in] conidx    Connection Index
 * @param[in] rsp       Pointer to the parameters of the message.
 *
 ****************************************************************************************
 */
static int attc_find_by_type_rsp_handler(uint8_t conidx, struct l2cc_att_find_by_type_rsp const *rsp)
{
    /* handling for all services discovery
     * Algorithm:
     * 1. check number of responses
     * 2. complete the list of items to send to upper layer
     *    GATTC_DISC_SVC_IND with the data in them
     * 3. check if all services has been found
     *    if no, continue search with latest found handle
     */
    switch (gattc_get_operation(conidx, GATTC_OP_CLIENT))
    {
    case GATTC_DISC_BY_UUID_SVC:
    {
        // retrieve operation
        struct gattc_disc_cmd *disc_cmd = GATT_OPERATION_CMD(conidx, GATTC_OP_CLIENT, gattc_disc_cmd);
        uint16_t cursor = 0;
        uint16_t end_handle = ATT_MAX_ATTR_HDL;

        // loop until all element are extracted
        while (cursor < rsp->data_len)
        {
            struct gattc_disc_svc_ind *svc_ind = KE_MSG_ALLOC_DYN(GATTC_DISC_SVC_IND,
                                                 gattc_get_requester(conidx, GATTC_OP_CLIENT), KE_BUILD_ID(TASK_GATTC, conidx), gattc_disc_svc_ind,
                                                 disc_cmd->uuid_len);

            // retrieve start handle
            svc_ind->start_hdl = co_read16p(&(rsp->data[cursor]));
            cursor += ATT_HANDLE_LEN;
            // retrieve end handle
            svc_ind->end_hdl   = co_read16p(&(rsp->data[cursor]));
            cursor += ATT_HANDLE_LEN;
            end_handle = svc_ind->end_hdl;

            // update message with searched uuid
            svc_ind->uuid_len = disc_cmd->uuid_len;
            memcpy(&(svc_ind->uuid[0]), &(disc_cmd->uuid[0]), disc_cmd->uuid_len);

            // send the svc found indication
            ke_msg_send(svc_ind);
        }

        // increment search handle cursor
        end_handle++;

        /* stop the search if the end has been detected */
        if ((end_handle > co_min(disc_cmd->end_hdl, ATT_MAX_ATTR_HDL))
                || (end_handle == ATT_INVALID_HANDLE))
        {
            /* send the GATT command complete event to upper layer */
            gattc_send_complete_evt(conidx, GATTC_OP_CLIENT, ATT_ERR_ATTRIBUTE_NOT_FOUND);
        }
        else
        {
            // continue discovery execution
            disc_cmd->start_hdl = end_handle;
            // put discovery command updated on the queue
            gattc_reschedule_operation(conidx, GATTC_OP_CLIENT);
        }
    }
    break;

    default:
    {
        /* send discovery complete event with error */
        gattc_send_complete_evt(conidx, GATTC_OP_CLIENT, GAP_ERR_INVALID_PARAM);
    }
    break;
    }
    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles read by type response from ATTC.
 * GATT Server command.
 *
 * @param[in] conidx    Connection Index
 * @param[in] rsp       Pointer to the parameters of the message.
 *
 ****************************************************************************************
 */
static int attc_rd_by_type_rsp_handler(uint8_t conidx, struct l2cc_att_rd_by_type_rsp const *rsp)
{
    uint16_t cursor = 0;
    uint16_t last_handle = ATT_INVALID_HANDLE;
    int msg_status = KE_MSG_CONSUMED;

    // retrieve current operation
    switch (gattc_get_operation(conidx, GATTC_OP_CLIENT))
    {
    // discover characteristic
    case GATTC_DISC_ALL_CHAR:
    case GATTC_DISC_BY_UUID_CHAR:
    {
        // retrieve operation
        struct gattc_disc_cmd *disc_cmd = GATT_OPERATION_CMD(conidx, GATTC_OP_CLIENT, gattc_disc_cmd);
        // retrieve UUID length
        uint8_t uuid_len = ((rsp->each_len > (ATT_HANDLE_LEN + ATT_PROP_LEN +
                                              ATT_HANDLE_LEN + ATT_UUID_16_LEN))
                            ? ATT_UUID_128_LEN : ATT_UUID_16_LEN);
        // UUID offset in payload
        uint8_t uuid_offset = ATT_HANDLE_LEN + ATT_PROP_LEN + ATT_HANDLE_LEN;


        // loop until all element are extracted
        while (cursor < rsp->data_len)
        {
            uint16_t attr_hdl;
            // retrieve attribute handle
            attr_hdl = co_read16p(&(rsp->data[cursor]));
            last_handle = attr_hdl;

            // check if we get expected attribute value
            if ((disc_cmd->operation == GATTC_DISC_ALL_CHAR) // any one
                    || attm_uuid_comp((uint8_t *) & (rsp->data[cursor + uuid_offset]), uuid_len,
                                      (uint8_t *) disc_cmd->uuid, disc_cmd->uuid_len))
            {
                /* discovered characteristics */
                struct gattc_disc_char_ind *char_ind = KE_MSG_ALLOC_DYN(GATTC_DISC_CHAR_IND,
                                                       gattc_get_requester(conidx, GATTC_OP_CLIENT), KE_BUILD_ID(TASK_GATTC, conidx), gattc_disc_char_ind,
                                                       uuid_len);

                // copy attribute handle
                char_ind->attr_hdl = attr_hdl;
                cursor += ATT_HANDLE_LEN;

                // retrieve attribute property
                char_ind->prop = rsp->data[cursor];
                cursor += ATT_PROP_LEN;

                // retrieve pointed attribute handle
                char_ind->pointer_hdl = co_read16p(&(rsp->data[cursor]));
                cursor += ATT_HANDLE_LEN;
                last_handle = char_ind->attr_hdl;

                // retrieve attribute UUID
                char_ind->uuid_len = uuid_len;
                memcpy(char_ind->uuid, &(rsp->data[cursor]), uuid_len);
                cursor += uuid_len;

                // send discovered characteristic
                ke_msg_send(char_ind);
            }
            else
            {
                // increment cursor
                cursor += rsp->each_len;
            }
        }

        // increment search handle cursor
        last_handle++;

        /* stop the search if the end has been detected */
        if ((last_handle > co_min(disc_cmd->end_hdl, ATT_MAX_ATTR_HDL))
                || (last_handle == ATT_INVALID_HANDLE))
        {
            /* send the GATT command complete event to upper layer */
            gattc_send_complete_evt(conidx, GATTC_OP_CLIENT, ATT_ERR_ATTRIBUTE_NOT_FOUND);
        }
        else
        {
            // continue discovery execution
            disc_cmd->start_hdl = last_handle;
            // put discovery command updated on the queue
            gattc_reschedule_operation(conidx, GATTC_OP_CLIENT);
        }
    }
    break;

    // discover included service
    case GATTC_DISC_INCLUDED_SVC:
    {
        // retrieve operation
        struct gattc_disc_cmd *disc_cmd = GATT_OPERATION_CMD(conidx, GATTC_OP_CLIENT, gattc_disc_cmd);

        // search included services
        bool include_uuid = false;
        // retrieve UUID length
        uint8_t uuid_len;
        struct gattc_disc_svc_incl_ind *inc_svc_ind = NULL;

        // check if data includes UUID
        if (rsp->data_len % 8 == 0)
        {
            /* determine the type of UUID */
            include_uuid = true;
        }

        // retrieve UUID length
        uuid_len = (include_uuid ? ATT_UUID_16_LEN : ATT_UUID_128_LEN);

        while (cursor < rsp->data_len)
        {
            /* discovered included service */
            inc_svc_ind = KE_MSG_ALLOC_DYN(GATTC_DISC_SVC_INCL_IND,
                                           gattc_get_requester(conidx, GATTC_OP_CLIENT), KE_BUILD_ID(TASK_GATTC, conidx), gattc_disc_svc_incl_ind,
                                           uuid_len);

            // retrieve attribute handle
            inc_svc_ind->attr_hdl = co_read16p(&(rsp->data[cursor]));
            cursor += ATT_HANDLE_LEN;

            // retrieve svc start handle
            inc_svc_ind->start_hdl = co_read16p(&(rsp->data[cursor]));
            cursor += ATT_HANDLE_LEN;

            // retrieve svc end handle
            inc_svc_ind->end_hdl = co_read16p(&(rsp->data[cursor]));
            cursor += ATT_HANDLE_LEN;

            // UUID is included, retrieve it
            if (include_uuid)
            {
                // retrieve service UUID
                inc_svc_ind->uuid_len = uuid_len;
                memcpy(inc_svc_ind->uuid, &(rsp->data[cursor]), uuid_len);
                cursor += uuid_len;
            }
            // stop processing data, it will be finished elsewhere
            else
            {
                // store indication to be reuse later
                co_list_push_back(&(gattc_env[conidx]->client.rsp_list), &(ke_param2msg(inc_svc_ind)->hdr));
                break;
            }

            // keep last handle
            last_handle = inc_svc_ind->attr_hdl;

            // send discovered included services
            ke_msg_send(inc_svc_ind);
        }

        // UUID is not included, retrieve it by reading service attribute
        if (!include_uuid && (inc_svc_ind != NULL))
        {
            /* This means that UUID 128 requires to be read in service attribute
             * value */

            /* send read request command */
            struct l2cc_att_rd_req *read = ATTC_ALLOCATE_ATT_REQ(conidx,
                                           L2C_CODE_ATT_RD_REQ, l2cc_att_rd_req, 0);

            /* read service  */
            read->handle   = inc_svc_ind->start_hdl;

            /* send the message */
            attc_send_att_req(conidx, read);

            /* message is consumed to not change GATT state*/
            return (KE_MSG_CONSUMED);
        }

        // increment search handle cursor
        last_handle++;

        /* stop the search if the end has been detected */
        if ((last_handle > co_min(disc_cmd->end_hdl, ATT_MAX_ATTR_HDL))
                || (last_handle == ATT_INVALID_HANDLE))
        {
            /* send the GATT command complete event to upper layer */
            gattc_send_complete_evt(conidx, GATTC_OP_CLIENT, ATT_ERR_ATTRIBUTE_NOT_FOUND);
        }
        else
        {
            // continue discovery execution
            disc_cmd->start_hdl = last_handle;
            // put discovery command updated on the queue
            gattc_reschedule_operation(conidx, GATTC_OP_CLIENT);
        }
    }
    break;

    case GATTC_READ_BY_UUID:
    {
        if (rsp->each_len < (gattc_get_mtu(conidx) - ATT_EACHLEN_LEN - ATT_CODE_LEN))
        {
            // Only send information of first parameter read
            struct gattc_read_ind *read_ind =  KE_MSG_ALLOC_DYN(GATTC_READ_IND,
                                               gattc_get_requester(conidx, GATTC_OP_CLIENT), KE_BUILD_ID(TASK_GATTC, conidx), gattc_read_ind,
                                               rsp->each_len - ATT_HANDLE_LEN);

            // retrieve attribute handle
            read_ind->handle = co_read16p(&(rsp->data[cursor]));
            cursor += ATT_HANDLE_LEN;

            // retrieve value of attribute
            read_ind->length = rsp->each_len - ATT_HANDLE_LEN;
            read_ind->offset = 0;
            memcpy(read_ind->value, &(rsp->data[cursor]), read_ind->length);
            cursor += read_ind->length;

            // send read value for founded attribute
            ke_msg_send(read_ind);

            /* stop the search only first read is sent*/
            /* send the GATT command complete event to upper layer */
            gattc_send_complete_evt(conidx, GATTC_OP_CLIENT, ATT_ERR_NO_ERROR);
        }
        else
        {
            struct l2cc_pdu_recv_ind *pdu = L2CC_PDU_TO_IND(rsp);
            // update command parameter to set them as if it's a normal read operation
            struct gattc_read_cmd *read_cmd = GATT_OPERATION_CMD(conidx, GATTC_OP_CLIENT, gattc_read_cmd);

            // retrieve attribute handle
            read_cmd->req.simple.handle = co_read16p(&(rsp->data[cursor]));
            read_cmd->req.simple.offset = rsp->each_len - ATT_HANDLE_LEN;
            read_cmd->req.simple.length = 0;

            pdu->offset = rsp->each_len - ATT_HANDLE_LEN;

            // put pdu message at end of queue
            co_list_push_back(&(gattc_env[conidx]->client.rsp_list), &(ke_param2msg(pdu)->hdr));
            msg_status = KE_MSG_NO_FREE;

            // reschedule operation
            gattc_reschedule_operation(conidx, GATTC_OP_CLIENT);
        }
    }
    break;

    default:
    {
        /* send discovery complete event with error */
        gattc_send_complete_evt(conidx, GATTC_OP_CLIENT, GAP_ERR_INVALID_PARAM);
    }
    break;
    }
    return msg_status;
}

/**
 ****************************************************************************************
 * @brief Handles read by group type response from ATTC.
 * GATT Server command.
 *
 * @param[in] conidx    Connection Index
 * @param[in] rsp       Pointer to the parameters of the message.
 *
 ****************************************************************************************
 */
static int attc_rd_by_grp_type_rsp_handler(uint8_t conidx, struct l2cc_att_rd_by_grp_type_rsp const *rsp)
{
    uint16_t nb_resp = (rsp->data_len / rsp->each_len);

    /* handling for all services discovery
     * Algorithm:
     * 1. check number of responses
     * 2. complete the list of items to send to upper layer
     *    GATTC_DISC_SVC_IND with the data in them
     * 3. check if the desired service is already there
     *      - if yes, send GATTC_CMP_EVT to upper layer
     *      - if no, continue Service discovery
     * 4. send to peer device
     */
    if ((gattc_get_operation(conidx, GATTC_OP_CLIENT) == GATTC_DISC_ALL_SVC) && (nb_resp > 0))
    {
        // retrieve operation
        struct gattc_disc_cmd *disc_cmd = GATT_OPERATION_CMD(conidx, GATTC_OP_CLIENT, gattc_disc_cmd);
        uint16_t cursor = 0;
        uint16_t end_handle = ATT_INVALID_HANDLE;
        uint8_t uuid_len = ((rsp->each_len != 20) ? ATT_UUID_16_LEN : ATT_UUID_128_LEN);
        bool svc_match = false;

        // loop until all element are extracted
        while ((cursor < rsp->data_len) && (!svc_match) && (end_handle != ATT_MAX_ATTR_HDL))
        {
            struct gattc_disc_svc_ind *svc_ind = KE_MSG_ALLOC_DYN(GATTC_DISC_SVC_IND,
                                                 gattc_get_requester(conidx, GATTC_OP_CLIENT), KE_BUILD_ID(TASK_GATTC, conidx), gattc_disc_svc_ind,
                                                 uuid_len);

            // retrieve start handle
            svc_ind->start_hdl = co_read16p(&(rsp->data[cursor]));
            cursor += ATT_HANDLE_LEN;
            // retrieve end handle
            svc_ind->end_hdl   = co_read16p(&(rsp->data[cursor]));
            cursor += ATT_HANDLE_LEN;
            end_handle = svc_ind->end_hdl;

            // retrieve service UUID
            svc_ind->uuid_len = uuid_len;
            memcpy(svc_ind->uuid, &(rsp->data[cursor]), uuid_len);
            cursor += uuid_len;

            // check if a service matches expected UUID
            if (attm_uuid_comp(svc_ind->uuid, uuid_len, disc_cmd->uuid, disc_cmd->uuid_len))
            {
                svc_match = true;
            }

            // send the svc found indication
            ke_msg_send(svc_ind);
        }

        // increment search handle cursor
        end_handle++;

        /* stop the search if the end has been detected
         * or a matching services has been found. */
        if (svc_match || (end_handle > co_min(disc_cmd->end_hdl, ATT_MAX_ATTR_HDL))
                || (end_handle == ATT_INVALID_HANDLE))
        {
            /* send the GATT command complete event to upper layer */
            gattc_send_complete_evt(conidx, GATTC_OP_CLIENT, ATT_ERR_NO_ERROR);
        }
        else
        {
            // continue discovery execution
            disc_cmd->start_hdl = end_handle;
            // put discovery command updated on the queue
            gattc_reschedule_operation(conidx, GATTC_OP_CLIENT);
        }
    }
    else
    {
        /* send the GATT command complete event to upper layer */
        gattc_send_complete_evt(conidx, GATTC_OP_CLIENT, GAP_ERR_PROTOCOL_PROBLEM);
    }
    return (KE_MSG_CONSUMED);
}


/**
 ****************************************************************************************
 * @brief Handles read response or read blob response from ATTC.
 * GATT Server command.length
 *
 * @param[in] conidx    Connection Index
 * @param[in] rsp       Pointer to the parameters of the message.
 *
 * @return If the message was consumed or not.
 *
 ****************************************************************************************
 */
static int attc_rd_rsp_handler(uint8_t conidx, struct l2cc_att_rd_rsp *rsp)
{
    /* message is consumed */
    int msg_status = KE_MSG_CONSUMED;

    switch (gattc_get_operation(conidx, GATTC_OP_CLIENT))
    {
    case GATTC_DISC_INCLUDED_SVC:
    {
        /* for the read request with 128-bit UUID */
        uint16_t last_handle;
        // retrieve operation
        struct gattc_disc_cmd *disc_cmd = GATT_OPERATION_CMD(conidx, GATTC_OP_CLIENT, gattc_disc_cmd);
        // retrieve prepared response
        struct gattc_disc_svc_incl_ind *svc_incl_ind =
            (struct gattc_disc_svc_incl_ind *) ke_msg2param(
                (struct ke_msg *) co_list_pop_front(&(gattc_env[conidx]->client.rsp_list)));

        memcpy(svc_incl_ind->uuid, &rsp->value, ATT_UUID_128_LEN);
        svc_incl_ind->uuid_len = ATT_UUID_128_LEN;

        /* send the message */
        ke_msg_send(svc_incl_ind);

        // retrieve last handle
        last_handle = svc_incl_ind->attr_hdl + 1;

        /* stop the search if the end has been detected */
        if ((last_handle > co_min(disc_cmd->end_hdl, ATT_MAX_ATTR_HDL))
                || (last_handle == ATT_INVALID_HANDLE))
        {
            /* send the GATT command complete event to upper layer */
            gattc_send_complete_evt(conidx, GATTC_OP_CLIENT, ATT_ERR_ATTRIBUTE_NOT_FOUND);
        }
        else
        {
            // continue discovery execution
            disc_cmd->start_hdl = last_handle;
            // put read by type in queue
            gattc_reschedule_operation(conidx, GATTC_OP_CLIENT);
        }
    }
    break;

    case GATTC_READ:
    case GATTC_READ_LONG:
    case GATTC_READ_BY_UUID:
    {
        // retrieve operation
        struct gattc_read_cmd *read_cmd = GATT_OPERATION_CMD(conidx, GATTC_OP_CLIENT, gattc_read_cmd);
        struct l2cc_pdu_recv_ind *pdu = L2CC_PDU_TO_IND(rsp);

        // use offset to store total length of attribute read
        pdu->offset = 0;

        // put ATT READ RSP message at end of queue
        co_list_push_back(&(gattc_env[conidx]->client.rsp_list), &(ke_param2msg(pdu)->hdr));

        // update received value length if closed to size that should be read
        if (read_cmd->req.simple.length != 0)
        {
            rsp->value_len = co_min(rsp->value_len, read_cmd->req.simple.length);
        }

        // increment size of the data with received PDU length
        pdu = (struct l2cc_pdu_recv_ind *)
              ke_msg2param((struct ke_msg *)co_list_pick(&(gattc_env[conidx]->client.rsp_list)));
        pdu->offset += rsp->value_len;

        /* message memory managed elsewhere */
        msg_status = KE_MSG_NO_FREE;

        // check if value of peer attribute is greater than max supported attribute length
        if (pdu->offset > ATT_MAX_VALUE)
        {
            /* send the GATT command complete event to upper layer  because insufficient resources*/
            gattc_send_complete_evt(conidx, GATTC_OP_CLIENT, ATT_ERR_INSUFF_RESOURCE);
        }
        // check if read is finished.
        else if ((rsp->value_len < (gattc_get_mtu(conidx) - 1))
                 || ((rsp->value_len >= read_cmd->req.simple.length)
                     && (read_cmd->req.simple.length != 0)))
        {
            // read is finished perform read - send element read
            attc_send_read_ind(conidx);

            /* send the GATT command complete event to upper layer */
            gattc_send_complete_evt(conidx, GATTC_OP_CLIENT, ATT_ERR_NO_ERROR);
        }
        else
        {
            // continue read execution
            read_cmd->req.simple.offset += rsp->value_len;
            if (read_cmd->req.simple.length != 0)
            {
                read_cmd->req.simple.length -= rsp->value_len;
            }

            // put read command updated on the queue
            gattc_reschedule_operation(conidx, GATTC_OP_CLIENT);
        }
    }
    break;

    default:
    {
        /* send the GATT command complete event to upper layer */
        gattc_send_complete_evt(conidx, GATTC_OP_CLIENT, GAP_ERR_PROTOCOL_PROBLEM);
    }
    break;

    }

    return (msg_status);
}

/**
 ****************************************************************************************
 * @brief Handles read multiple response from ATTC.
 * GATT Server command.
 *
 * @param[in] conidx    Connection Index
 * @param[in] rsp       Pointer to the parameters of the message.
 *
 ****************************************************************************************
 */
static int attc_rd_mult_rsp_handler(uint8_t conidx, struct l2cc_att_rd_mult_rsp const *rsp)
{
    switch (gattc_get_operation(conidx, GATTC_OP_CLIENT))
    {
    case GATTC_READ_MULTIPLE:
    {
        // retrieve operation
        struct gattc_read_cmd *read_cmd = GATT_OPERATION_CMD(conidx, GATTC_OP_CLIENT, gattc_read_cmd);
        uint8_t i;
        uint16_t cursor = 0;
        uint8_t status = ATT_ERR_NO_ERROR;
        for (i = 0 ; i < read_cmd->nb ; i++)
        {
            struct gattc_read_ind *read_ind =  KE_MSG_ALLOC_DYN(GATTC_READ_IND,
                                               gattc_get_requester(conidx, GATTC_OP_CLIENT),  KE_BUILD_ID(TASK_GATTC, conidx), gattc_read_ind,
                                               read_cmd->req.multiple[i].len);

            // retrieve attribute handle
            read_ind->handle = read_cmd->req.multiple[i].handle;

            // retrieve value of attribute
            read_ind->length = co_min(read_cmd->req.multiple[i].len, rsp->value_len - cursor);
            read_ind->offset = 0;
            memcpy(read_ind->value, &(rsp->value[cursor]), read_ind->length);
            cursor += read_ind->length;

            // check if length is as expected
            if (read_ind->length != read_cmd->req.multiple[i].len)
            {
                status = GATT_ERR_INVALID_ATT_LEN;
            }

            // send read value for founded attribute
            ke_msg_send(read_ind);
        }

        // check if all data has been read
        if (cursor !=  rsp->value_len)
        {
            status = GATT_ERR_INVALID_ATT_LEN;
        }

        /* send the GATT command complete event to upper layer */
        gattc_send_complete_evt(conidx, GATTC_OP_CLIENT, status);
    }
    break;

    default:
    {
        /* send the GATT command complete event to upper layer */
        gattc_send_complete_evt(conidx, GATTC_OP_CLIENT, GAP_ERR_PROTOCOL_PROBLEM);
    }
    break;
    }
    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles write response from ATTC.
 * GATT Server command.
 *
 * @param[in] conidx    Connection Index
 * @param[in] rsp       Pointer to the parameters of the message.
 *
 ****************************************************************************************
 */
static int attc_wr_rsp_handler(uint8_t conidx, struct l2cc_att_wr_rsp const *rsp)
{
    switch (gattc_get_operation(conidx, GATTC_OP_CLIENT))
    {
    case GATTC_WRITE:
    {
        /* send the GATT command complete event to upper layer */
        gattc_send_complete_evt(conidx, GATTC_OP_CLIENT, ATT_ERR_NO_ERROR);
    }
    break;

    default:
    {
        /* send the GATT command complete event to upper layer */
        gattc_send_complete_evt(conidx, GATTC_OP_CLIENT, GAP_ERR_PROTOCOL_PROBLEM);
    }
    break;
    }
    return (KE_MSG_CONSUMED);
}


/**
 ****************************************************************************************
 * @brief Handles prepare write response from ATTC.
 * GATT Server command.
 *
 * @param[in] conidx    Connection Index
 * @param[in] rsp       Pointer to the parameters of the message.
 *
 ****************************************************************************************
 */
static int attc_prep_wr_rsp_handler(uint8_t conidx, struct l2cc_att_prep_wr_rsp const *rsp)
{
    switch (gattc_get_operation(conidx, GATTC_OP_CLIENT))
    {
    case GATTC_WRITE:
    {
        // retrieve operation
        struct gattc_write_cmd *write_cmd = GATT_OPERATION_CMD(conidx, GATTC_OP_CLIENT, gattc_write_cmd);
        write_cmd->cursor = rsp->offset - write_cmd->offset;

        // check if receive data is equals to sent data and if received length never
        // exceed sent length
        if ((memcmp((char const *) &rsp->value[0],
                    (char const *) &write_cmd->value[write_cmd->cursor], rsp->value_len) != 0)
                || ((write_cmd->cursor + rsp->value_len) > write_cmd->length))
        {
            // set that an error occurs
            write_cmd->cursor = GATT_WRITE_ERROR_CODE;
        }
        else
        {
            // increment cursor
            write_cmd->cursor += rsp->value_len;
        }

        // put wrote command updated on the queue
        gattc_reschedule_operation(conidx, GATTC_OP_CLIENT);
    }
    break;

    default:
    {
        /* send the GATT command complete event to upper layer */
        gattc_send_complete_evt(conidx, GATTC_OP_CLIENT, GAP_ERR_PROTOCOL_PROBLEM);
    }
    break;
    }
    return (KE_MSG_CONSUMED);
}


/**
 ****************************************************************************************
 * @brief Handles execute write response from ATTC.
 * GATT Server command.
 *
 * @param[in] conidx    Connection Index
 * @param[in] rsp       Pointer to the parameters of the message.
 *
 ****************************************************************************************
 */
static int attc_exe_wr_rsp_handler(uint8_t conidx, struct l2cc_att_exe_wr_rsp const *rsp)
{
    switch (gattc_get_operation(conidx, GATTC_OP_CLIENT))
    {
    case GATTC_WRITE:
    {
        // retrieve operation
        struct gattc_write_cmd *write_cmd = GATT_OPERATION_CMD(conidx, GATTC_OP_CLIENT, gattc_write_cmd);

        // inform the upper layer that write is finished (with or without error)
        gattc_send_complete_evt(conidx, GATTC_OP_CLIENT,
                                ((write_cmd->cursor == write_cmd->length) ? GAP_ERR_NO_ERROR
                                 : GATT_ERR_WRITE));
    }
    break;
    case GATTC_EXEC_WRITE:
    {
        // inform the upper layer that write execution is finished
        gattc_send_complete_evt(conidx, GATTC_OP_CLIENT, GAP_ERR_NO_ERROR);
    }
    break;
    default:
    {
        /* send the GATT command complete event to upper layer */
        gattc_send_complete_evt(conidx, GATTC_OP_CLIENT, GAP_ERR_PROTOCOL_PROBLEM);
    }
    break;
    }
    return (KE_MSG_CONSUMED);
}


/**
 ****************************************************************************************
 * @brief Handles data from ATTC (Notification or indication).
 *
 * @param[in] conidx    Connection Index
 * @param[in] evt       Pointer to the parameters of the message.

 *
 ****************************************************************************************
 */
static int attc_hdl_val_ntf_ind_handler(uint8_t conidx, struct l2cc_att_hdl_val_ind *evt)
{
    struct gattc_event_ind *ind;

    // allocate indication
    ind = KE_MSG_ALLOC_DYN(((evt->code == L2C_CODE_ATT_HDL_VAL_NTF) ? GATTC_EVENT_IND : GATTC_EVENT_REQ_IND),
                           attc_get_event_dest(conidx, evt->handle),
                           KE_BUILD_ID(TASK_GATTC, conidx), gattc_event_ind, evt->value_len);

    // fill in parameters
    ind->type = ((evt->code == L2C_CODE_ATT_HDL_VAL_NTF) ? GATTC_NOTIFY : GATTC_INDICATE);
    ind->length = evt->value_len;

    ind->handle = evt->handle;
    memcpy(ind->value, evt->value, evt->value_len);

    /* send the message */
    ke_msg_send(ind);

    return (KE_MSG_CONSUMED);
}




/**
 ****************************************************************************************
 * @brief Handles reception of an error response from attribute client
 *
 * @param[in] conidx    Connection Index
 * @param[in] param     Pointer to the parameters of the message.
 *
 ****************************************************************************************
 */
static int attc_err_rsp_handler(uint8_t conidx, struct l2cc_att_err_rsp *rsp)
{
    /* send the GATT command complete event to upper layer */
    gattc_send_complete_evt(conidx, GATTC_OP_CLIENT, rsp->reason);

    return (KE_MSG_CONSUMED);
}



/// The default PDU handlers
const struct att_pdu_handler attc_handlers[] =
{
    { L2C_CODE_ATT_MTU_RSP, (att_func_t) attc_exc_mtu_rsp_handler },
    { L2C_CODE_ATT_FIND_INFO_RSP, (att_func_t) attc_find_info_rsp_handler },
    { L2C_CODE_ATT_FIND_BY_TYPE_RSP, (att_func_t) attc_find_by_type_rsp_handler },
    { L2C_CODE_ATT_RD_BY_TYPE_RSP, (att_func_t) attc_rd_by_type_rsp_handler },
    { L2C_CODE_ATT_RD_BY_GRP_TYPE_RSP, (att_func_t) attc_rd_by_grp_type_rsp_handler },
    { L2C_CODE_ATT_RD_RSP, (att_func_t) attc_rd_rsp_handler },
    { L2C_CODE_ATT_RD_BLOB_RSP, (att_func_t) attc_rd_rsp_handler },
    { L2C_CODE_ATT_RD_MULT_RSP, (att_func_t) attc_rd_mult_rsp_handler },

    { L2C_CODE_ATT_WR_RSP, (att_func_t) attc_wr_rsp_handler },
    { L2C_CODE_ATT_PREP_WR_RSP, (att_func_t) attc_prep_wr_rsp_handler },
    { L2C_CODE_ATT_EXE_WR_RSP, (att_func_t) attc_exe_wr_rsp_handler },
    { L2C_CODE_ATT_ERR_RSP, (att_func_t) attc_err_rsp_handler },
    { L2C_CODE_ATT_HDL_VAL_IND, (att_func_t) attc_hdl_val_ntf_ind_handler },
    { L2C_CODE_ATT_HDL_VAL_NTF, (att_func_t) attc_hdl_val_ntf_ind_handler },
};
#endif /* (BLE_ATTC) */


int attc_l2cc_pdu_recv_handler(uint8_t conidx, struct l2cc_pdu_recv_ind *param)
{
    // means that message cannot be handled by ATTS module
    int msg_status = ATT_PDU_HANDLER_NOT_FOUND;

#if (BLE_ATTC)
    uint8_t cursor;
    att_func_t fhandler = NULL;

    // search PDU Handler
    for (cursor = 0 ; cursor < (sizeof(attc_handlers) / sizeof(struct att_pdu_handler)) ; cursor++)
    {
        if (attc_handlers[cursor].pdu_id == param->pdu.data.code)
        {
            fhandler = attc_handlers[cursor].handler;
        }
    }

    // execute function handler
    if (fhandler != NULL)
    {
        // check if transaction timer can be stopped
        if ((param->pdu.data.code != L2C_CODE_ATT_HDL_VAL_IND)
                && (param->pdu.data.code != L2C_CODE_ATT_HDL_VAL_NTF))
        {
            /* Clear Client transaction timer */
            ke_timer_clear(GATTC_CLIENT_RTX_IND, KE_BUILD_ID(TASK_GATTC, conidx));
        }

        // execute PDU Handler
        msg_status = fhandler(conidx, &(param->pdu.data.code));
    }
#else // (!BLE_ATTC)
    switch (param->pdu.data.code)
    {
    case L2C_CODE_ATT_MTU_RSP:
    case L2C_CODE_ATT_FIND_INFO_RSP:
    case L2C_CODE_ATT_FIND_BY_TYPE_RSP:
    case L2C_CODE_ATT_RD_BY_TYPE_RSP:
    case L2C_CODE_ATT_RD_BY_GRP_TYPE_RSP:
    case L2C_CODE_ATT_RD_RSP:
    case L2C_CODE_ATT_RD_BLOB_RSP:
    case L2C_CODE_ATT_RD_MULT_RSP:
    case L2C_CODE_ATT_WR_RSP:
    case L2C_CODE_ATT_PREP_WR_RSP:
    case L2C_CODE_ATT_EXE_WR_RSP:
    case L2C_CODE_ATT_ERR_RSP:
    case L2C_CODE_ATT_HDL_VAL_IND:
    {
        struct l2cc_att_err_rsp *err_rsp = L2CC_ATT_PDU_ALLOC_DYN(conidx, L2C_CODE_ATT_ERR_RSP,
                                           KE_BUILD_ID(TASK_GATTC, conidx), l2cc_att_err_rsp, 0);

        // retrieve error code
        err_rsp->op_code = param->pdu.data.code;
        // Fill Packet
        err_rsp->handle = 0;
        err_rsp->reason = ATT_ERR_INSUFF_RESOURCE;

        /* send the L2Cap message */
        l2cc_pdu_send(err_rsp);
    }
    break;

    default: /* Do Nothing - ignore packet */
        break;
    };
#endif // (BLE_ATTC)

    return (msg_status);
}

#endif // (BLE_CENTRAL || BLE_PERIPHERAL)

/// @} ATTC
