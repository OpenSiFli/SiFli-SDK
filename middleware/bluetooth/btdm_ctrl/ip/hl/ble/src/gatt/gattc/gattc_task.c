/**
 ****************************************************************************************
 *
 * @file gattc_task.c
 *
 * @brief Generic Attribute Profile Controller Task implementation.
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup GATTCTASK
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
#include "co_utils.h"

/* kernel task */
#include "ke_task.h"
#include "ke_mem.h"
/* GATTC task structures */
#include "gattc_task.h"
#include "gattc_int.h"
/* GAP module */
#include "gap.h"
#include "gapm.h"
#include "gapc.h"
/* GATTM functions */
#include "gattm.h"
/* ATTM Database functions */
#include "attm.h"
/* GATT error codes */
#include "../gatt.h"
#include "gattc_int.h"

/* ATT Client and server */
#include "../attc/attc.h" // Access to internal API required
#include "../atts/atts.h" // Access to internal API required

/* L2CAP management */
#include "l2cc.h"
#include "l2cc_pdu.h"



/*
 * DEFINES
 ****************************************************************************************
 */

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @brief Check if current operation can be processed or not.
 * if it can be proceed, initialize an operation request.
 * If a command complete event with error code can be triggered.
 *
 * Function returns how the message should be handled by message handler.
 *
 * @param[in] conidx        Connection Index
 * @param[in] op_type       Operation type.
 * @param[in] op_msg        Requested operation message (note op_msg cannot be null)
 * @param[in] supp_ops      Supported operations array.
 *                          Latest array value shall be GATTC_NO_OP.
 *
 * @return operation can be executed if message status equals KE_MSG_NO_FREE,
 * else nothing to do, just exit from the handler.
 ****************************************************************************************
 */
__STATIC int gattc_process_op(uint8_t conidx, uint8_t op_type, void *op_msg, enum gattc_operation *supp_ops)
{
    ASSERT_ERR(op_type < GATTC_OP_MAX);
    // Returned message status
    int msg_status = KE_MSG_CONSUMED; // Reset State
    // Current process state
    uint8_t  state = ke_state_get(KE_BUILD_ID(TASK_GATTC, conidx));
    uint8_t  operation = *((uint8_t *)op_msg);
    uint16_t seq_num = ((struct gattc_op_cmd *) op_msg)->seq_num;
    uint8_t  status = GAP_ERR_COMMAND_DISALLOWED;

    /* no operation on going or requested operation is current on going operation. */
    if ((state != GATTC_FREE))
    {
        if (gattc_get_operation_ptr(conidx, op_type) != op_msg)
        {
            status = GAP_ERR_NO_ERROR;

            // check what to do with command if an operation is ongoing.
            if ((state & (1 << op_type)) != GATTC_READY)
            {
                // operation are queued by default
                // save it for later.
                msg_status = KE_MSG_SAVED;
            }
            else
            {
                // check if operation is suported
                while (*supp_ops != GATTC_NO_OP)
                {
                    // operation supported by command
                    if (operation == *supp_ops)
                    {
                        break;
                    }
                    // check next operation
                    else
                    {
                        supp_ops++;
                    }
                }

                // operation not supported
                if (*supp_ops == GATTC_NO_OP)
                {
                    status = GAP_ERR_INVALID_PARAM;
                }
                else if (gattc_env[conidx]->trans_timeout)
                {
                    status = GAP_ERR_COMMAND_DISALLOWED;
                }
                else
                {
                    // message memory will be managed by GAPM
                    msg_status = KE_MSG_NO_FREE;

                    // store operation
                    gattc_set_operation_ptr(conidx, op_type, op_msg);

                    // set state to busy
                    gattc_update_state(conidx, (1 << op_type), true);
                }
            }
        }
        else
        {
            // message memory managed by GAPC
            msg_status = KE_MSG_NO_FREE;
            status = GAP_ERR_NO_ERROR;
        }
    }

    // if an error detected, send command completed with error status
    if (status != GAP_ERR_NO_ERROR)
    {
        gattc_send_error_evt(conidx, operation, seq_num, ke_msg_src_id_get(op_msg), status);
    }

    return msg_status;
}



#if (BLE_ATTC)
/**
 ****************************************************************************************
 * @brief Handles request for MTU exchange
 * GATT Client command.
 * HL Message: GATT_EXC_MTU_REQ
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GATTM).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 *
 ****************************************************************************************
 */
__STATIC int gattc_exc_mtu_cmd_handler(ke_msg_id_t const msgid, struct gattc_exc_mtu_cmd *param,
                                       ke_task_id_t const dest_id, ke_task_id_t const src_id)
{

    // list of handler supported operations
    enum gattc_operation supp_ops[] = {GATTC_MTU_EXCH, GATTC_NO_OP};
    // Current connection index
    uint8_t conidx = KE_IDX_GET(dest_id);
    // check if operation can be executed
    int msg_status = gattc_process_op(conidx, GATTC_OP_CLIENT, param, supp_ops);

    // Operation can be handled
    if (msg_status == KE_MSG_NO_FREE)
    {
        // Try to exchange MTU size if not already done
        //if (!gattc_env[conidx]->mtu_exch)
        {
            struct l2cc_att_mtu_req *req = ATTC_ALLOCATE_ATT_REQ(conidx, L2C_CODE_ATT_MTU_REQ, l2cc_att_mtu_req, 0);
            req->mtu_size = gapm_get_max_mtu();
            /* send the message request */
            attc_send_att_req(conidx, req);
        }
#if 0
        else
        {
            // request already done
            gattc_send_complete_evt(conidx, GATTC_OP_CLIENT, GAP_ERR_COMMAND_DISALLOWED);
        }
#endif
    }

    return msg_status;
}



/**
 ****************************************************************************************
 * @brief Handles service discovery request
 * GATT Client command.
 * HL Message: GATT_DISC_SVC_REQ
 * operation:
 *      DISC_ALL_SERVICE
 *      DISC_BY_UUID
 *      DISC_BY_RANGE
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GATT).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 *
 ****************************************************************************************
 */
__STATIC int gattc_disc_cmd_handler(ke_msg_id_t const msgid, struct gattc_disc_cmd *param,
                                    ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    // list of handler supported operations
    enum gattc_operation supp_ops[] = {GATTC_DISC_ALL_SVC, GATTC_DISC_BY_UUID_SVC, GATTC_DISC_INCLUDED_SVC,
                                       GATTC_DISC_ALL_CHAR, GATTC_DISC_BY_UUID_CHAR, GATTC_DISC_DESC_CHAR, GATTC_NO_OP
                                      };
    // Current connection index
    uint8_t conidx = KE_IDX_GET(dest_id);
    // check if operation can be executed
    int msg_status = gattc_process_op(conidx, GATTC_OP_CLIENT, param, supp_ops);

    // Operation can be handled
    if (msg_status == KE_MSG_NO_FREE)
    {
        /* handle checking */
        if ((param->start_hdl > param->end_hdl))
        {
            /* send discovery complete event with error */
            gattc_send_complete_evt(conidx, GATTC_OP_CLIENT, GAP_ERR_INVALID_PARAM);
        }
        // will only process 2 or 16 UUID size for discovery all services
        else if ((!((param->uuid_len == ATT_UUID_128_LEN)
                    || (param->uuid_len == ATT_UUID_32_LEN)
                    || (param->uuid_len == ATT_UUID_16_LEN))))
        {
            // send an error, it is either 2 or 16
            gattc_send_complete_evt(conidx, GATTC_OP_CLIENT, GATT_ERR_INVALID_TYPE_IN_SVC_SEARCH);
        }
        else
        {
            /* check action according to requested operation */
            switch (param->operation)
            {
            case GATTC_DISC_BY_UUID_SVC:
            {
                /* find by type value for service discovery by UUID */
                uint8_t uuid_len = ((param->uuid_len == ATT_UUID_16_LEN)
                                    ? ATT_UUID_16_LEN : ATT_UUID_128_LEN);

                struct l2cc_att_find_by_type_req *find_by_type = ATTC_ALLOCATE_ATT_REQ(conidx,
                        L2C_CODE_ATT_FIND_BY_TYPE_REQ, l2cc_att_find_by_type_req, uuid_len);

                /* fill up the parameters */
                find_by_type->shdl     = param->start_hdl;
                find_by_type->ehdl     = param->end_hdl;

                /* searching for entries for primary service type */
                find_by_type->type     = (uint16_t) ATT_DECL_PRIMARY_SERVICE;


                // for a 32 bits UUID, convert it to a BT 128 bits UUID to convey it over the air
                if (param->uuid_len == ATT_UUID_32_LEN)
                {
                    find_by_type->val_len  = ATT_UUID_128_LEN;
                    attm_convert_to128(find_by_type->val, param->uuid, param->uuid_len);
                }
                else
                {
                    find_by_type->val_len  = param->uuid_len;
                    memcpy(&(find_by_type->val[0]), &(param->uuid[0]), param->uuid_len);
                }

                /* send the message */
                attc_send_att_req(conidx, find_by_type);
            }
            break;

            case GATTC_DISC_ALL_CHAR:
            case GATTC_DISC_BY_UUID_CHAR:
            {
                /* read by type request for characteristic discovery */
                struct l2cc_att_rd_by_type_req *read_by_type = ATTC_ALLOCATE_ATT_REQ(conidx,
                        L2C_CODE_ATT_RD_BY_TYPE_REQ, l2cc_att_rd_by_type_req, ATT_UUID_16_LEN);

                /* fill up the parameters */
                read_by_type->shdl   = param->start_hdl;
                read_by_type->ehdl   = param->end_hdl;

                /* searching for entries for "include" type */
                co_write16p(read_by_type->uuid, ATT_DECL_CHARACTERISTIC);
                read_by_type->uuid_len    = ATT_UUID_16_LEN;

                /* send the message */
                attc_send_att_req(conidx, read_by_type);
            }
            break;
            case GATTC_DISC_INCLUDED_SVC:
            {
                /* read by type request for included service discovery */
                struct l2cc_att_rd_by_type_req *read_by_type = ATTC_ALLOCATE_ATT_REQ(conidx,
                        L2C_CODE_ATT_RD_BY_TYPE_REQ, l2cc_att_rd_by_type_req, ATT_UUID_16_LEN);

                /* fill up the parameters */
                read_by_type->shdl   = param->start_hdl;
                read_by_type->ehdl   = param->end_hdl;

                /* searching for entries for "include" type */
                co_write16p(read_by_type->uuid, ATT_DECL_INCLUDE);
                read_by_type->uuid_len    = ATT_UUID_16_LEN;

                /* send the message */
                attc_send_att_req(conidx, read_by_type);
            }
            break;

            case GATTC_DISC_ALL_SVC:
            {
                /* read by group type request for discover all services */
                struct l2cc_att_rd_by_grp_type_req *read_by_grp_type = ATTC_ALLOCATE_ATT_REQ(conidx,
                        L2C_CODE_ATT_RD_BY_GRP_TYPE_REQ, l2cc_att_rd_by_grp_type_req, ATT_UUID_16_LEN);


                /* fill up the parameters */
                read_by_grp_type->shdl              = param->start_hdl;
                read_by_grp_type->ehdl              = param->end_hdl;
                co_write16p(read_by_grp_type->uuid, ATT_DECL_PRIMARY_SERVICE);
                read_by_grp_type->uuid_len          = ATT_UUID_16_LEN;

                /* send the message */
                attc_send_att_req(conidx, read_by_grp_type);
            }
            break;
            case GATTC_DISC_DESC_CHAR:
            {
                struct l2cc_att_find_info_req *info_req = ATTC_ALLOCATE_ATT_REQ(conidx,
                        L2C_CODE_ATT_FIND_INFO_REQ, l2cc_att_find_info_req, 0);

                /* fill up the parameters */
                info_req->shdl   = param->start_hdl;
                info_req->ehdl   = param->end_hdl;

                /* send the information request message */
                attc_send_att_req(conidx, info_req);
            }
            break;

            default: /* Nothing to do, could not happen */
                break;
            }
        }
    }

    return (msg_status);
}

/**
 ****************************************************************************************
 * @brief Handles read command.
 * GATT Client command.
 * HL Message: GATTC_READ_CMD
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GATT).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 *
 ****************************************************************************************
 */
__STATIC int gattc_read_cmd_handler(ke_msg_id_t const msgid, struct gattc_read_cmd *param,
                                    ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    // list of handler supported operations
    enum gattc_operation supp_ops[] = {GATTC_READ, GATTC_READ_LONG, GATTC_READ_BY_UUID,
                                       GATTC_READ_MULTIPLE, GATTC_NO_OP
                                      };
    // Current connection index
    uint8_t conidx = KE_IDX_GET(dest_id);
    // check if operation can be executed
    int msg_status = gattc_process_op(conidx, GATTC_OP_CLIENT, param, supp_ops);

    // Operation can be handled
    if (msg_status == KE_MSG_NO_FREE)
    {
        /* check action according to requested operation */
        switch (param->operation)
        {
        case GATTC_READ_BY_UUID:
        {
            // check that it's not continuation of UUID read
            if (co_list_is_empty(&(gattc_env[conidx]->client.rsp_list)))
            {
                // check if parameters correctly set.
                if ((param->req.by_uuid.start_hdl > param->req.by_uuid.end_hdl))
                {
                    /* send discovery complete event with error */
                    gattc_send_complete_evt(conidx, GATTC_OP_CLIENT, GAP_ERR_INVALID_PARAM);
                }
                // will only process 2 or 16 UUID size
                else if (!((param->req.by_uuid.uuid_len == ATT_UUID_128_LEN)
                           || (param->req.by_uuid.uuid_len == ATT_UUID_32_LEN)
                           || (param->req.by_uuid.uuid_len == ATT_UUID_16_LEN)))
                {
                    // send an error, it is either 2 or 16
                    gattc_send_complete_evt(conidx, GATTC_OP_CLIENT, GATT_ERR_INVALID_TYPE_IN_SVC_SEARCH);
                }
                else
                {
                    uint8_t uuid_len = ((param->req.by_uuid.uuid_len == ATT_UUID_16_LEN)
                                        ? ATT_UUID_16_LEN : ATT_UUID_128_LEN);

                    struct l2cc_att_rd_by_type_req *read = ATTC_ALLOCATE_ATT_REQ(conidx,
                                                           L2C_CODE_ATT_RD_BY_TYPE_REQ, l2cc_att_rd_by_type_req, uuid_len);

                    /* fill up the message parameters */
                    read->shdl      = param->req.by_uuid.start_hdl;
                    read->ehdl      = param->req.by_uuid.end_hdl;

                    // for a 32 bits UUID, convert it to a BT 128 bits UUID to convey it over the air
                    if (param->req.by_uuid.uuid_len == ATT_UUID_32_LEN)
                    {
                        read->uuid_len  = ATT_UUID_128_LEN;
                        attm_convert_to128(read->uuid, param->req.by_uuid.uuid, param->req.by_uuid.uuid_len);
                    }
                    else
                    {
                        read->uuid_len  = param->req.by_uuid.uuid_len;
                        memcpy(&read->uuid[0], param->req.by_uuid.uuid, read->uuid_len);
                    }

                    /* send message */
                    attc_send_att_req(conidx, read);
                }
                break;
            }
        }
        // no break

        case GATTC_READ:
        {
            // if offset = 0, perform a simple ATT read request
            if (param->req.simple.offset == 0)
            {
                /* read request PDU */
                struct l2cc_att_rd_req *read = ATTC_ALLOCATE_ATT_REQ(conidx,
                                               L2C_CODE_ATT_RD_REQ, l2cc_att_rd_req, 0);

                /* read service  */
                read->handle   = param->req.simple.handle;

                /* send the message */
                attc_send_att_req(conidx, read);
                break;
            }// else perform a read blob
        }
        // no break here.

        case GATTC_READ_LONG:
        {
            struct l2cc_att_rd_blob_req *blob_req = ATTC_ALLOCATE_ATT_REQ(conidx,
                                                    L2C_CODE_ATT_RD_BLOB_REQ, l2cc_att_rd_blob_req, 0);

            /* fill up the parameters */
            blob_req->offset   = param->req.simple.offset;
            blob_req->handle   = param->req.simple.handle;

            /* send the blob request */
            attc_send_att_req(conidx, blob_req);
        }
        break;

        case GATTC_READ_MULTIPLE:
        {
            uint16_t i;

            struct l2cc_att_rd_mult_req *mult_req = ATTC_ALLOCATE_ATT_REQ(conidx,
                                                    L2C_CODE_ATT_RD_MULT_REQ, l2cc_att_rd_mult_req, sizeof(uint16_t) * param->nb);

            // number of handles to read
            mult_req->nb_handles   = param->nb;
            for (i = 0; i < mult_req->nb_handles; i++)
            {
                mult_req->handles[i] = param->req.multiple[i].handle;
            }

            // send the message
            attc_send_att_req(conidx, mult_req);
        }
        break;

        default: /* Nothing to do, could not happen */
            break;
        }
    }

    return (msg_status);
}

/**
 ****************************************************************************************
 * @brief Handles write command.
 * GATT Client command.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GATT).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 *
 ****************************************************************************************
 */
__STATIC int gattc_write_cmd_handler(ke_msg_id_t const msgid, struct gattc_write_cmd *param,
                                     ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    // list of handler supported operations
    enum gattc_operation supp_ops[] = {GATTC_WRITE, GATTC_WRITE_NO_RESPONSE, GATTC_WRITE_SIGNED, GATTC_NO_OP};
    // Current connection index
    uint8_t conidx = KE_IDX_GET(dest_id);
    // check if operation can be executed
    int msg_status = gattc_process_op(conidx, GATTC_OP_CLIENT, param, supp_ops);

    // Operation can be handled
    if (msg_status == KE_MSG_NO_FREE)
    {
        /* check action according to requested operation */
        switch (param->operation)
        {
        case GATTC_WRITE_NO_RESPONSE:
        {
            uint16_t value_len = co_min(param->length, (gattc_get_mtu(conidx) - 3));

            struct l2cc_att_wr_cmd *wr_cmd = ATTC_ALLOCATE_ATT_REQ(conidx,
                                             L2C_CODE_ATT_WR_CMD, l2cc_att_wr_cmd, value_len);

            /* fill up the parameters */
            wr_cmd->handle    = param->handle;
            // compute prepared write value len
            wr_cmd->value_len = value_len;

            memcpy(&wr_cmd->value[0], &param->value[0], wr_cmd->value_len);

            /* send the message */
            attc_send_att_req(conidx, wr_cmd);
        }
        break;
        case GATTC_WRITE_SIGNED:
        {
            uint8_t status = GATT_ERR_SIGNED_WRITE;

            /* Put a checker if the characteristic authentication
             * bit is enabled, and bonding has been done
             */
            // Check if link is bonded - CSRK should exist; also at least unauth level is set from prior bond
            if (gapc_is_sec_set(conidx, GAPC_LK_BONDED) && (gapc_lk_sec_lvl_get(conidx) > GAP_LK_NO_AUTH))
            {
                // Compute prepared write value length (Signature + Sign Counter length are included)
                uint16_t length = param->length + ATT_SIGNED_PDU_VAL_OFFSET + ATT_SIGNATURE_LEN;

                if (length <= gattc_get_mtu(conidx))
                {
                    status = GAP_ERR_NO_ERROR;

                    struct gapc_sign_cmd *cmd = KE_MSG_ALLOC_DYN(GAPC_SIGN_CMD,
                                                KE_BUILD_ID(TASK_GAPC, conidx), dest_id,
                                                gapc_sign_cmd, length);

                    // Message is sign includes full PDU
                    cmd->operation = GAPC_SIGN_PACKET;
                    // Value length + SignCounter length
                    cmd->byte_len  = param->length + SMPC_SIGN_COUNTER_LEN + ATT_SIGNED_PDU_VAL_OFFSET;

                    // Construct the signed PDU - Code
                    cmd->msg[0] = (uint8_t)L2C_CODE_ATT_SIGN_WR_CMD;
                    // Attribute handle
                    co_write16p(&cmd->msg[1], param->handle);

                    // Value
                    memcpy(&cmd->msg[ATT_SIGNED_PDU_VAL_OFFSET], &param->value[0], param->length);
                    // The SignCounter and the MAC value will be added by the SMP

                    ke_msg_send(cmd);
                }
            }

            if (status != GAP_ERR_NO_ERROR)
            {
                /* send error indication to upper layer */
                gattc_send_complete_evt(conidx, GATTC_OP_CLIENT, status);
            }
        }
        break;

        case GATTC_WRITE:
        {
            // check if a simple write can be performed
            if (param->auto_execute && (param->length <= (gattc_get_mtu(conidx) - 3))
                    && (param->offset == 0))
            {
                struct l2cc_att_wr_req *wr_req = ATTC_ALLOCATE_ATT_REQ(conidx,
                                                 L2C_CODE_ATT_WR_REQ, l2cc_att_wr_req, param->length);

                /* fill up the parameters */
                wr_req->handle    = param->handle;

                // compute prepared write value len
                wr_req->value_len = param->length;
                memcpy(&wr_req->value[0], &param->value[0], wr_req->value_len);

                /* send the message */
                attc_send_att_req(conidx, wr_req);
            }
            else
            {
                // write isn't finished
                if (param->cursor < param->length)
                {
                    // Value length
                    uint16_t value_len = co_min((param->length - param->cursor), (gattc_get_mtu(conidx) - 5));

                    struct l2cc_att_prep_wr_req *prep_wr = ATTC_ALLOCATE_ATT_REQ(conidx,
                                                           L2C_CODE_ATT_PREP_WR_REQ, l2cc_att_prep_wr_req, value_len);

                    /* fill up the parameters */
                    prep_wr->handle   = param->handle;
                    prep_wr->offset   = param->offset + param->cursor;

                    // compute prepared write value len
                    prep_wr->value_len = value_len;

                    // copy the first prep_wr->val_len size to prepare write message
                    memcpy(&prep_wr->value[0], &param->value[param->cursor], prep_wr->value_len);

                    /* send the message */
                    attc_send_att_req(conidx, prep_wr);
                }
                // write is finished or an error occurs
                else
                {
                    if (param->auto_execute)
                    {
                        /* send the execute write message */
                        attc_send_execute(conidx,
                                          ((param->cursor == param->length) ? ATT_EXECUTE_ALL_PREPARED_WRITES
                                           : ATT_CANCEL_ALL_PREPARED_WRITES));
                    }
                    else
                    {
                        // inform the upper layer that write is finished (with or without error)
                        gattc_send_complete_evt(conidx, GATTC_OP_CLIENT,
                                                ((param->cursor == param->length) ? GAP_ERR_NO_ERROR
                                                 : GATT_ERR_WRITE));
                    }
                }
            }
        }
        break;
        default: /* Nothing to do, can not happen */
            break;
        }
    }

    return (msg_status);
}



/**
 ****************************************************************************************
 * @brief Handles reception of Indication confirm message.
 * GATT Client command.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GATT).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 *
 ****************************************************************************************
 */
__STATIC int gattc_event_cfm_handler(ke_msg_id_t const msgid, struct gattc_event_cfm *param,
                                     ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    // Current connection index
    uint8_t conidx = KE_IDX_GET(dest_id);

    // check if link is established
    if (ke_state_get(dest_id) != GATTC_FREE)
    {
        // send attribute indication confirmaton
        attc_send_hdl_cfm(conidx);
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles cancel of write execution command request.
 * GATT Client command.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GATT).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 *
 ****************************************************************************************
 */
__STATIC int gattc_execute_write_cmd_handler(ke_msg_id_t const msgid, struct gattc_execute_write_cmd *param,
        ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    // list of handler supported operations
    enum gattc_operation supp_ops[] = {GATTC_EXEC_WRITE, GATTC_NO_OP};
    // Current connection index
    uint8_t conidx = KE_IDX_GET(dest_id);
    // check if operation can be executed
    int msg_status = gattc_process_op(conidx, GATTC_OP_CLIENT, param, supp_ops);

    // Operation can be handled
    if (msg_status == KE_MSG_NO_FREE)
    {
        // request prepare write
        attc_send_execute(conidx, param->execute);
    }

    return (msg_status);
}



/**
 ****************************************************************************************
 * @brief Handles Registration to peer device events (Indication/Notification).
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GATTC).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 *
 ****************************************************************************************
 */
__STATIC int gattc_reg_to_peer_evt_cmd_handler(ke_msg_id_t const msgid,
        struct gattc_reg_to_peer_evt_cmd *param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
    // list of handler supported operations
    enum gattc_operation supp_ops[] = {GATTC_REGISTER, GATTC_UNREGISTER, GATTC_NO_OP};
    // Current connection index
    uint8_t conidx = KE_IDX_GET(dest_id);
    // check if operation can be executed
    int msg_status = gattc_process_op(conidx, GATTC_OP_CLIENT, param, supp_ops);

    // Operation can be handled
    if (msg_status == KE_MSG_NO_FREE)
    {
        uint8_t status = GAP_ERR_NO_ERROR;
        struct attc_register_evt *reg_evt =
            (struct attc_register_evt *) co_list_pick(&(gattc_env[conidx]->client.reg_evt));
        // browse list of event to check if range already registered
        while (reg_evt != NULL)
        {
            if ((reg_evt->start_hdl == param->start_hdl)
                    && (reg_evt->end_hdl == param->end_hdl)
                    && (reg_evt->task == src_id))
            {
                // match index found
                break;
            }

            // go to next element
            reg_evt = (struct attc_register_evt *) reg_evt->hdr.next;
        }

        if (param->operation == GATTC_UNREGISTER)
        {
            // registered event founded
            if (reg_evt != NULL)
            {
                //unregister event
                co_list_extract(&(gattc_env[conidx]->client.reg_evt), &(reg_evt->hdr));
                ke_free(reg_evt);
            }
            else
            {
                // not possible to unregister
                status = GAP_ERR_INVALID_PARAM;
            }
        }
        else
        {
            // registered event founded
            if (reg_evt != NULL)
            {
                // not possible to register, already in list
                status = GAP_ERR_INVALID_PARAM;
            }
            // registered event founded
            else
            {
                // allocate registerd element
                reg_evt = (struct attc_register_evt *) ke_malloc(sizeof(struct attc_register_evt), KE_MEM_ATT_DB);
                // register event
                reg_evt->start_hdl = param->start_hdl;
                reg_evt->end_hdl = param->end_hdl;
                reg_evt->task = src_id;

                // insert element in registration list
                co_list_push_back(&(gattc_env[conidx]->client.reg_evt), &(reg_evt->hdr));
            }
        }
        /* send discovery complete event with error */
        gattc_send_complete_evt(conidx, GATTC_OP_CLIENT, status);
    }

    return (msg_status);
}
/**
 ****************************************************************************************
 * @brief Handles the Service Discovery Procedure Command
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GATTC).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 *
 ****************************************************************************************
 */
__STATIC int gattc_sdp_svc_disc_cmd_handler(ke_msg_id_t const msgid,
        struct gattc_sdp_svc_disc_cmd *param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
    enum gattc_operation supp_ops[] = {GATTC_SDP_DISC_SVC, GATTC_SDP_DISC_SVC_ALL, GATTC_SDP_DISC_CANCEL, GATTC_NO_OP};
    // Current connection index
    uint8_t conidx = KE_IDX_GET(dest_id);
    // check if operation can be executed
    int msg_status = gattc_process_op(conidx, GATTC_OP_SDP, param, supp_ops);

    // Operation can be handled
    if (msg_status == KE_MSG_NO_FREE)
    {
        if (param->operation == GATTC_SDP_DISC_CANCEL)
        {
            // nothing to cancel
            gattc_send_complete_evt(conidx, GATTC_OP_SDP, GAP_ERR_COMMAND_DISALLOWED);
        }
        // parameter sanity check
        else if ((param->start_hdl == ATT_INVALID_HDL)
                 || (param->start_hdl > param->end_hdl)
                 || ((param->operation == GATTC_SDP_DISC_SVC) && (param->uuid_len != ATT_UUID_16_LEN)
                     && (param->uuid_len != ATT_UUID_32_LEN) && (param->uuid_len != ATT_UUID_128_LEN)))
        {
            // Invalid parameters
            gattc_send_complete_evt(conidx, GATTC_OP_SDP, GAP_ERR_INVALID_PARAM);
        }
        else
        {
            // Send a Service Discovery
            struct gattc_disc_cmd *svc_disc =
                KE_MSG_ALLOC_DYN(GATTC_DISC_CMD, dest_id, dest_id, gattc_disc_cmd, ATT_UUID_128_LEN);

            // With a specific UUID
            if (param->operation == GATTC_SDP_DISC_SVC)
            {
                svc_disc->operation = GATTC_DISC_BY_UUID_SVC;
                svc_disc->uuid_len  = param->uuid_len;
                memcpy(svc_disc->uuid, param->uuid, param->uuid_len);
            }
            // All Services
            else
            {
                svc_disc->operation = GATTC_DISC_ALL_SVC;
                svc_disc->uuid_len  = ATT_UUID_16_LEN;
                memset(svc_disc->uuid, 0, ATT_UUID_16_LEN);
            }
            svc_disc->start_hdl = param->start_hdl;
            svc_disc->end_hdl   = param->end_hdl;

            ke_msg_send(svc_disc);
        }
    }
    // operation saved to be executed later
    else if (msg_status == KE_MSG_SAVED)
    {
        // if a cancel is requested, request to cancel on-going operation.
        if (param->operation == GATTC_SDP_DISC_CANCEL)
        {
            // retrieve on-going operation
            struct gattc_sdp_svc_disc_cmd *op =
                (struct gattc_sdp_svc_disc_cmd *) gattc_get_operation_ptr(conidx, GATTC_OP_SDP);
            if (op != NULL)
            {
                // mark it canceled / start handle = 0
                op->start_hdl = ATT_INVALID_HDL;
            }
            // free cancel request
            msg_status = KE_MSG_CONSUMED;
        }
    }

    return (msg_status);
}



/**
 ****************************************************************************************
 * @brief Service Discovery Procedure - Handle end of a discovery command
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GATTC).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 *
 ****************************************************************************************
 */
__STATIC int gattc_cmp_evt_handler(ke_msg_id_t const msgid, struct gattc_cmp_evt *evt,
                                   ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint8_t conidx = KE_IDX_GET(dest_id);


    struct gattc_sdp_svc_disc_cmd *cmd =
        (struct gattc_sdp_svc_disc_cmd *) gattc_get_operation_ptr(conidx, GATTC_OP_SDP);

    // Check if a SDP operation is on-going
    if ((cmd != NULL) && ((cmd->operation == GATTC_SDP_DISC_SVC) || (cmd->operation == GATTC_SDP_DISC_SVC_ALL)))
    {
        uint8_t status = GAP_ERR_NO_ERROR;
        // check if operation has been canceled.
        if (cmd->start_hdl == ATT_INVALID_HDL)
        {
            status = GAP_ERR_CANCELED;
        }
        else
        {
            bool svc_search = false;
            uint8_t disc_type = GATTC_NO_OP;

            // service indication put on head of the list
            struct ke_msg *msg = (struct ke_msg *) co_list_pick(&(gattc_env[conidx]->client.sdp_data));
            struct gattc_sdp_svc_ind *svc = NULL;
            // check that message is valid
            if ((msg != NULL) && (msg->id == GATTC_SDP_SVC_IND))
            {
                svc = (struct gattc_sdp_svc_ind *) ke_msg2param(msg);
            }

            switch (evt->operation)
            {
            case GATTC_DISC_DESC_CHAR:
            {
                // Send Service Indication
                if (svc != NULL)
                {
                    // remove message from SDP data list
                    co_list_pop_front(&(gattc_env[conidx]->client.sdp_data));
                    // inform that message has been received
                    ke_msg_send(svc);
                    svc = NULL;
                }
                else
                {
                    // an error occurs during discovery
                    status = GAP_ERR_PROTOCOL_PROBLEM;
                }
            }
            // no break
            case GATTC_DISC_ALL_SVC:
            case GATTC_DISC_BY_UUID_SVC:
            {
                // search info about service
                svc_search = true;
            }
            break;
            case GATTC_DISC_INCLUDED_SVC:
            {
                disc_type = GATTC_DISC_ALL_CHAR;
            }
            break;
            case GATTC_DISC_ALL_CHAR:
            {
                disc_type = GATTC_DISC_DESC_CHAR;
            }
            break;

            default: /* Nothing to do, Ignore the message. */
                break;
            }

            // search for services
            if (svc_search)
            {
                // check that services are available
                while (! co_list_is_empty(&(gattc_env[conidx]->client.sdp_data)))
                {
                    // take first element of the list
                    msg = (struct ke_msg *) co_list_pop_front(&(gattc_env[conidx]->client.sdp_data));

                    if (msg->id != GATTC_DISC_SVC_IND)
                    {
                        // an error occurs during discovery
                        status = GAP_ERR_PROTOCOL_PROBLEM;
                        // free message
                        ke_msg_free(msg);
                        break;
                    }
                    else
                    {
                        struct gattc_disc_svc_ind *svc_ind = (struct gattc_disc_svc_ind *) ke_msg2param(msg);
                        uint16_t nb_hdl = svc_ind->end_hdl - svc_ind->start_hdl;
                        // calculate service size to allocate.
                        uint32_t svc_msg_size = sizeof(struct gattc_sdp_svc_ind)
                                                + (nb_hdl * sizeof(union gattc_sdp_att_info));

                        // check that memory block can be allocated
                        if (!ke_check_malloc(svc_msg_size, KE_MEM_KE_MSG))
                        {
                            // not enough resources in the system
                            status = GAP_ERR_INSUFF_RESOURCES;
                            // free message
                            ke_msg_free(msg);
                            break;
                        }

                        // allocate service
                        svc = KE_MSG_ALLOC_DYN(GATTC_SDP_SVC_IND,
                                               gattc_get_requester(conidx, GATTC_OP_SDP), dest_id,
                                               gattc_sdp_svc_ind, (nb_hdl * sizeof(union gattc_sdp_att_info)));

                        svc->start_hdl = svc_ind->start_hdl;
                        svc->end_hdl   = svc_ind->end_hdl;
                        svc->uuid_len  = svc_ind->uuid_len;
                        memcpy(svc->uuid, svc_ind->uuid, svc_ind->uuid_len);

                        // free message
                        ke_msg_free(msg);

                        // empty service, trigger service indication
                        if (nb_hdl == 0)
                        {
                            ke_msg_send(svc);
                            svc = NULL;
                        }
                        else
                        {
                            // put service message in front of sdp data
                            co_list_push_front(&(gattc_env[conidx]->client.sdp_data),
                                               &(((struct ke_msg *) ke_param2msg(svc))->hdr));
                            // search for included service
                            disc_type = GATTC_DISC_INCLUDED_SVC;
                            break;
                        }
                    }
                }
            }

            // discovery to perform
            if (disc_type != GATTC_NO_OP)
            {
                if (svc != NULL)
                {
                    // Send a Characteristic Discovery
                    struct gattc_disc_cmd *disc =
                        KE_MSG_ALLOC_DYN(GATTC_DISC_CMD, dest_id, dest_id, gattc_disc_cmd, ATT_UUID_16_LEN);

                    disc->operation = disc_type;
                    disc->start_hdl = svc->start_hdl + 1;
                    disc->end_hdl   = svc->end_hdl;
                    disc->uuid_len  = ATT_UUID_16_LEN;
                    memset(disc->uuid, 0, ATT_UUID_16_LEN);
                    ke_msg_send(disc);
                }
                else
                {
                    // unexpected discovery state
                    status = GAP_ERR_PROTOCOL_PROBLEM;
                }
            }
        }

        // check if discovery is finished
        if ((status != GAP_ERR_NO_ERROR)
                || (co_list_is_empty(&(gattc_env[conidx]->client.sdp_data))))
        {
            gattc_send_complete_evt(conidx, GATTC_OP_SDP, status);
        }
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Service Discovery Procedure - Handle service reception
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GATTC).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 *
 ****************************************************************************************
 */
__STATIC int gattc_disc_svc_ind_handler(ke_msg_id_t const msgid, struct gattc_disc_svc_ind *ind,
                                        ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint8_t conidx = KE_IDX_GET(dest_id);
    uint8_t opcode = gattc_get_operation(conidx, GATTC_OP_SDP);

    // Check if a SDP operation is on-going
    if ((opcode == GATTC_SDP_DISC_SVC) || (opcode == GATTC_SDP_DISC_SVC_ALL))
    {
        // put service info in SDP queue
        co_list_push_back(&(gattc_env[conidx]->client.sdp_data), &(ke_param2msg(ind)->hdr));
    }

    return (KE_MSG_NO_FREE);
}


/**
 ****************************************************************************************
 * @brief Service Discovery Procedure - Handle included service reception
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GATTC).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 *
 ****************************************************************************************
 */
__STATIC int gattc_disc_svc_incl_ind_handler(ke_msg_id_t const msgid, struct gattc_disc_svc_incl_ind *ind,
        ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint8_t conidx = KE_IDX_GET(dest_id);
    uint8_t opcode = gattc_get_operation(conidx, GATTC_OP_SDP);

    // Check if a SDP operation is on-going
    if ((opcode == GATTC_SDP_DISC_SVC) || (opcode == GATTC_SDP_DISC_SVC_ALL))
    {
        // service indication put on head of the list
        struct ke_msg *msg = (struct ke_msg *) co_list_pick(&(gattc_env[conidx]->client.sdp_data));
        // check that message is valid
        if ((msg != NULL) && (msg->id == GATTC_SDP_SVC_IND))
        {
            struct gattc_sdp_svc_ind *svc = (struct gattc_sdp_svc_ind *) ke_msg2param(msg);

            // check if attribute handle is within service attribute range
            if ((ind->attr_hdl > svc->start_hdl) && (ind->attr_hdl <= svc->end_hdl))
            {
                // retrieve attribute index
                uint16_t idx = ind->attr_hdl - (svc->start_hdl + 1);
                // fill included service attribute info
                svc->info[idx].att_type          = GATTC_SDP_INC_SVC;
                svc->info[idx].inc_svc.start_hdl = ind->start_hdl;
                svc->info[idx].inc_svc.end_hdl   = ind->end_hdl;
                svc->info[idx].inc_svc.uuid_len  = ind->uuid_len;
                memcpy(svc->info[idx].inc_svc.uuid, ind->uuid, ind->uuid_len);
            }
        }
    }

    return (KE_MSG_CONSUMED);
}


/**
 ****************************************************************************************
 * @brief Service Discovery Procedure - Handle Characteristic info reception
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GATTC).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 *
 ****************************************************************************************
 */
__STATIC int gattc_disc_char_ind_handler(ke_msg_id_t const msgid, struct gattc_disc_char_ind *ind,
        ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint8_t conidx = KE_IDX_GET(dest_id);
    uint8_t opcode = gattc_get_operation(conidx, GATTC_OP_SDP);

    // Check if a SDP operation is on-going
    if ((opcode == GATTC_SDP_DISC_SVC) || (opcode == GATTC_SDP_DISC_SVC_ALL))
    {
        // service indication put on head of the list
        struct ke_msg *msg = (struct ke_msg *) co_list_pick(&(gattc_env[conidx]->client.sdp_data));
        // check that message is valid
        if ((msg != NULL) && (msg->id == GATTC_SDP_SVC_IND))
        {
            struct gattc_sdp_svc_ind *svc = (struct gattc_sdp_svc_ind *) ke_msg2param(msg);

            // check if attribute handle is within service attribute range
            if ((ind->attr_hdl > svc->start_hdl) && (ind->attr_hdl <= svc->end_hdl))
            {
                // retrieve attribute index
                uint16_t idx = ind->attr_hdl - (svc->start_hdl + 1);
                // fill attribute info
                svc->info[idx].att_type        = GATTC_SDP_ATT_CHAR;
                svc->info[idx].att_char.handle = ind->pointer_hdl;
                svc->info[idx].att_char.prop   = ind->prop;

                // check if value handle is within service attribute range
                if ((ind->pointer_hdl > svc->start_hdl) && (ind->pointer_hdl <= svc->end_hdl))
                {
                    // retrieve attribute index
                    idx = ind->pointer_hdl - (svc->start_hdl + 1);
                    // fill value info
                    svc->info[idx].att_type        = GATTC_SDP_ATT_VAL;
                    svc->info[idx].att.uuid_len = ind->uuid_len;
                    memcpy(svc->info[idx].att.uuid, ind->uuid, ind->uuid_len);
                }
            }
        }
    }

    return (KE_MSG_CONSUMED);
}


/**
 ****************************************************************************************
 * @brief Service Discovery Procedure - Handle attribute descriptioon reception
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GATTC).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 *
 ****************************************************************************************
 */
__STATIC int gattc_disc_char_desc_ind_handler(ke_msg_id_t const msgid, struct gattc_disc_char_desc_ind *ind,
        ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint8_t conidx = KE_IDX_GET(dest_id);
    uint8_t opcode = gattc_get_operation(conidx, GATTC_OP_SDP);

    // Check if a SDP operation is on-going
    if ((opcode == GATTC_SDP_DISC_SVC) || (opcode == GATTC_SDP_DISC_SVC_ALL))
    {
        // service indication put on head of the list
        struct ke_msg *msg = (struct ke_msg *) co_list_pick(&(gattc_env[conidx]->client.sdp_data));
        // check that message is valid
        if ((msg != NULL) && (msg->id == GATTC_SDP_SVC_IND))
        {
            struct gattc_sdp_svc_ind *svc = (struct gattc_sdp_svc_ind *) ke_msg2param(msg);

            // check if attribute handle is within service attribute range
            if ((ind->attr_hdl > svc->start_hdl) && (ind->attr_hdl <= svc->end_hdl))
            {
                // retrieve attribute index
                uint16_t idx = ind->attr_hdl - (svc->start_hdl + 1);

                // only update field which are unknown: Attribute descriptors
                if (svc->info[idx].att_type == GATTC_SDP_NONE)
                {
                    // fill attribute info
                    svc->info[idx].att_type      = GATTC_SDP_ATT_DESC;
                    svc->info[idx].att.uuid_len = ind->uuid_len;
                    memcpy(svc->info[idx].att.uuid, ind->uuid, ind->uuid_len);
                }
            }
        }
    }

    return (KE_MSG_CONSUMED);
}
#endif // (BLE_ATTC)

#if (BLE_ATTS)
/**
 ****************************************************************************************
 * @brief Send event to peer device command request
 * GATT Server command.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GATT).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 *
 ****************************************************************************************
 */
__STATIC int gattc_send_evt_cmd_handler(ke_msg_id_t const msgid, struct gattc_send_evt_cmd *param,
                                        ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    // list of handler supported operations
    enum gattc_operation supp_ops[] = {GATTC_NOTIFY, GATTC_INDICATE, GATTC_SVC_CHANGED,  GATTC_NO_OP};
    // Current connection index
    uint8_t conidx = KE_IDX_GET(dest_id);
    // check if operation can be executed
    int msg_status = gattc_process_op(conidx, GATTC_OP_SERVER, param, supp_ops);

    // Operation can be handled
    if (msg_status == KE_MSG_NO_FREE)
    {
        // check if value in cache is still valid
        if ((gattc_env[conidx]->server.read_cache != NULL)
                && (gattc_env[conidx]->server.read_cache->handle == param->handle))
        {
            atts_clear_read_cache(conidx);
        }

        // request to send event
        uint8_t status = atts_send_event(conidx, param);

        // complete the request if not possible to send event
        if (status != GAP_ERR_NO_ERROR)
        {
            gattc_send_complete_evt(conidx, GATTC_OP_SERVER, status);
        }

    }

    return (msg_status);
}

/**
 ****************************************************************************************
 * @brief Handles a send Service Changed indication command.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC int gattc_send_svc_changed_cmd_handler(ke_msg_id_t const msgid,
        struct gattc_send_svc_changed_cmd *param,
        ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint8_t conidx = KE_IDX_GET(dest_id);

    // Check the current state of the GATTC instance
    if (ke_state_get(dest_id) != GATTC_FREE)
    {

        uint8_t status   = GAP_ERR_INVALID_PARAM;

        // Check the provided parameters
        if ((param->svc_shdl <= param->svc_ehdl) &&
                (param->svc_shdl != 0x0000))
        {
            // check if service changed feature is present
            // or check if client configuration is enabled or not
            if ((!gapm_svc_chg_en()) || (!gapc_svc_chg_ccc_get(conidx)))
            {
                status  = GAP_ERR_NOT_SUPPORTED;
            }
            else
            {
                // Attribute handle
                uint16_t att_handle = gattm_svc_get_start_hdl() + GATT_IDX_SVC_CHANGED;
                // Value
                struct gatt_svc_changed att_val = {param->svc_shdl, param->svc_ehdl};

                // Update the attribute value in the database
                attm_att_set_value(att_handle, sizeof(struct gatt_svc_changed), 0, (uint8_t *)&att_val);

                // Send the indication
                struct gattc_send_evt_cmd *req = KE_MSG_ALLOC_DYN(GATTC_SEND_EVT_CMD,
                                                 dest_id, src_id, gattc_send_evt_cmd, sizeof(struct gatt_svc_changed));

                // Fill in the parameter structure
                req->operation = GATTC_SVC_CHANGED;
                req->seq_num   = param->seq_num;
                req->handle    = att_handle;
                req->length    = sizeof(struct gatt_svc_changed);
                memcpy(req->value, &att_val, req->length);
                // Send the event
                ke_msg_send(req);
                status  = GAP_ERR_NO_ERROR;
            }
        }

        if (status != GAP_ERR_NO_ERROR)
        {
            // Send a CMP_EVT message to the application
            struct gattc_cmp_evt *cmp_evt = KE_MSG_ALLOC(GATTC_CMP_EVT,
                                            src_id, dest_id,
                                            gattc_cmp_evt);

            cmp_evt->operation = GATTC_SVC_CHANGED;
            cmp_evt->status     = status;
            cmp_evt->seq_num    = param->seq_num;

            ke_msg_send(cmp_evt);
        }
    }
    else
    {
        gattc_send_error_evt(conidx, param->operation, param->seq_num, src_id, GAP_ERR_COMMAND_DISALLOWED);
    }

    // Message is consumed
    return (KE_MSG_CONSUMED);
}


/**
 ****************************************************************************************
 * @brief Handles reception of write command confirmation.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC int gattc_write_cfm_handler(ke_msg_id_t const msgid,
                                     struct gattc_write_cfm *param,
                                     ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    // Check the current state of the GATTC instance and if a write response should be sent
    if (ke_state_get(dest_id) != GATTC_FREE)
    {
        uint8_t conidx = KE_IDX_GET(dest_id);

        // Get PDU in top of message queue
        struct ke_msg *pdu_msg = (struct ke_msg *) co_list_pick(&(gattc_env[conidx]->server.pdu_queue));

        if (pdu_msg != NULL)
        {
            struct l2cc_pdu *pdu = &(((struct l2cc_pdu_recv_ind *) ke_msg2param(pdu_msg))->pdu);
            bool continue_process = true;

            switch (pdu->data.code)
            {
            case L2C_CODE_ATT_WR_REQ:
            {
                // Send write response to peer device
                atts_write_rsp_send(KE_IDX_GET(dest_id), param->handle, param->status);
            }
            // no break
            case L2C_CODE_ATT_WR_CMD:
            case L2C_CODE_ATT_SIGN_WR_CMD:
            {
                // clean-up received PDU
                ke_msg_free((struct ke_msg *) co_list_pop_front(&(gattc_env[conidx]->server.pdu_queue)));
            }
            break;

            case L2C_CODE_ATT_EXE_WR_REQ:
            {

                if (param->status != GAP_ERR_NO_ERROR)
                {
                    // Send error response to peer device
                    atts_send_error(conidx, L2C_CODE_ATT_EXE_WR_REQ, param->handle, param->status);

                    // clear operation data
                    atts_clear_prep_data(conidx);

                    // clean-up received PDU
                    ke_msg_free((struct ke_msg *) co_list_pop_front(&(gattc_env[conidx]->server.pdu_queue)));
                }
                else
                {
                    // Nothing to do, let atts procedure to continue
                }
            }
            break;

            default:
            {
                ASSERT_INFO(0, conidx, pdu->data.code);
                continue_process = false;
                /* Ignore message */
            }
            break;

            }

            if (continue_process)
            {
                // Continue procedure execution or check if something else has to be processed
                atts_process_pdu(conidx);
            }
        }

        // check if value in cache is still valid
        if ((param->status == GAP_ERR_NO_ERROR)
                && (gattc_env[conidx]->server.read_cache != NULL)
                && (gattc_env[conidx]->server.read_cache->handle == param->handle))
        {
            atts_clear_read_cache(conidx);
        }
    }

    // Message is consumed
    return (KE_MSG_CONSUMED);
}


/**
 ****************************************************************************************
 * @brief Handles reception of attribute information confirmation.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC int gattc_att_info_cfm_handler(ke_msg_id_t const msgid,
                                        struct gattc_att_info_cfm *param,
                                        ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    // Check the current state of the GATTC instance
    if (ke_state_get(dest_id) != GATTC_FREE)
    {
        uint8_t conidx = KE_IDX_GET(dest_id);

        // Get PDU in top of message queue
        struct ke_msg *pdu_msg = (struct ke_msg *) co_list_pick(&(gattc_env[conidx]->server.pdu_queue));

        if (pdu_msg != NULL)
        {
            struct l2cc_pdu *pdu = &(((struct l2cc_pdu_recv_ind *) ke_msg2param(pdu_msg))->pdu);

            // check that a prepare write is on-going
            switch (pdu->data.code)
            {
            case L2C_CODE_ATT_PREP_WR_REQ:
            {
                // modifying attribute not supported
                if (param->status == ATT_ERR_NO_ERROR)
                {
                    // store attribute length - add RI bit field to ensure that 0 length value are accepted
                    pdu->payld_len = param->length | PERM(RI, ENABLE);
                }
                else
                {
                    // Send error response to peer device
                    atts_send_error(conidx, L2C_CODE_ATT_PREP_WR_REQ, param->handle, param->status);
                    // clean-up received PDU
                    ke_msg_free((struct ke_msg *) co_list_pop_front(&(gattc_env[conidx]->server.pdu_queue)));
                }

                // Continue procedure execution or check if something else has to be processed
                atts_process_pdu(conidx);
            }
            break;

            default:
            {
                ASSERT_INFO(0, conidx, pdu->data.code);
                /* Ignore message */
            }
            break;
            }
        }
    }

    // Message is consumed
    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of read command confirmation.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC int gattc_read_cfm_handler(ke_msg_id_t const msgid,
                                    struct gattc_read_cfm *param,
                                    ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    int msg_status = KE_MSG_CONSUMED;

    // check connection state
    if (ke_state_get(dest_id) != GATTC_FREE)
    {
        uint8_t conidx = KE_IDX_GET(dest_id);

        // Get PDU in top of message queue
        struct ke_msg *pdu_msg = (struct ke_msg *) co_list_pick(&(gattc_env[conidx]->server.pdu_queue));

        // check if a PDU operation is on-going
        if (pdu_msg != NULL)
        {
            // check if a value is already present in cache
            if (gattc_env[conidx]->server.read_cache != NULL)
            {
                ke_msg_free(ke_param2msg(gattc_env[conidx]->server.read_cache));
            }

            // update cache with value
            gattc_env[conidx]->server.read_cache = param;
            msg_status = KE_MSG_NO_FREE;

            // continue PDU procedure execution
            atts_process_pdu(conidx);
        }
    }

    // Message is consumed
    return msg_status;
}

#endif // (BLE_ATTS)



#if (BLE_ATTS) || (BLE_ATTC)

/**
 ****************************************************************************************
 * @brief Handles timeout operations from ATTC/ATTS for upper layer.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_APP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 *
 ****************************************************************************************
 */
__STATIC int gattc_timeout_handler(ke_msg_id_t const msgid, void const *param,
                                   ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint8_t state = ke_state_get(dest_id);
    uint8_t conidx = KE_IDX_GET(dest_id);
    if (state != GATTC_FREE)
    {
        // check that event not already sent and current connection not used for a discovery
        if ((!gattc_env[conidx]->trans_timeout) && (!gapc_is_disc_connection(conidx)))
        {
            ke_msg_send_basic(GATTC_TRANSACTION_TO_ERROR_IND, gapc_get_dest_task(conidx), dest_id);
        }

        // mark that a transaction timeout occurs, it means that next request will be
        // rejected for this connection
        gattc_env[conidx]->trans_timeout = true;

        switch (msgid)
        {
#if (BLE_ATTC)
        case GATTC_CLIENT_RTX_IND:
        {
            /* send the GATT command complete event to upper layer */
            gattc_send_complete_evt(conidx, GATTC_OP_CLIENT, GAP_ERR_TIMEOUT);
        }
        break;
#endif // (BLE_ATTC)

#if (BLE_ATTS)
        case GATTC_SERVER_RTX_IND:
        {
            /* send the GATT command complete event to upper layer */
            gattc_send_complete_evt(conidx, GATTC_OP_SERVER, GAP_ERR_TIMEOUT);
        }
        break;
#endif // (BLE_ATTS)
        default: /* Nothing to do */
            break;
        }

    }

    /* message is consumed */
    return (KE_MSG_CONSUMED);
}


#endif // (BLE_ATTS) || (BLE_ATTC)


/**
 ****************************************************************************************
 * @brief Handles signature data generated from SMP.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GATTC).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 *
 ****************************************************************************************
 */
__STATIC int gapc_sign_ind_handler(ke_msg_id_t const msgid, struct gapc_sign_ind *param,
                                   ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint8_t state = ke_state_get(dest_id);
    uint8_t conidx = KE_IDX_GET(dest_id);
    if (state != GATTC_FREE)
    {
        switch (param->operation)
        {
#if(BLE_ATTC)
        case GAPC_SIGN_PACKET:
        {
            if (gattc_get_operation(conidx, GATTC_OP_CLIENT) == GATTC_WRITE_SIGNED)
            {
                // Retrieve operation
                struct gattc_write_cmd *write_cmd = GATT_OPERATION_CMD(conidx, GATTC_OP_CLIENT, gattc_write_cmd);
                // Value length
                uint16_t value_len = param->byte_len - ATT_SIGNED_PDU_VAL_OFFSET;

                /* status is ATT_ERR_NO_ERROR */
                struct l2cc_att_sign_wr_cmd *sgn_wr = ATTC_ALLOCATE_ATT_REQ(conidx,
                                                      L2C_CODE_ATT_SIGN_WR_CMD, l2cc_att_sign_wr_cmd, value_len);


                /* fill up the parameters */
                sgn_wr->value_len  = value_len;
                sgn_wr->handle     = write_cmd->handle;

                /* copy message */
                memcpy(&sgn_wr->value, &param->signed_msg[ATT_SIGNED_PDU_VAL_OFFSET], sgn_wr->value_len);

                /* send the message */
                attc_send_att_req(conidx, sgn_wr);

                gattc_send_complete_evt(conidx, GATTC_OP_CLIENT, GAP_ERR_NO_ERROR);
            }
            else
            {
                /* send the GATT command complete event to upper layer */
                gattc_send_complete_evt(conidx, GATTC_OP_CLIENT, GAP_ERR_PROTOCOL_PROBLEM);
            }

        }
        break;
#endif // (BLE_ATTC)

#if (BLE_ATTS)
        case GAPC_SIGN_CHECK:
        {
            atts_write_signed_cfm(conidx, param->byte_len, param->signed_msg);
        }
        break;
#endif // (BLE_ATTS)
        default: /* Not expected, ignore message */
            break;
        }
    }

    /* message is consumed */
    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the SMPC_CMP_EVT messqge
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GATTC).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC int gapc_cmp_evt_handler(ke_msg_id_t const msgid, struct gapc_cmp_evt *evt,
                                  ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint8_t state = ke_state_get(dest_id);
    uint8_t conidx = KE_IDX_GET(dest_id);
    if (state != GATTC_FREE)
    {
        switch (evt->operation)
        {
#if (BLE_ATTS)
        case GAPC_SIGN_CHECK:
        {
            // signature check fails
            if (evt->status != GAP_ERR_NO_ERROR)
            {
                // terminate att write signed procedure
                // clean-up received PDU
                ke_msg_free((struct ke_msg *) co_list_pop_front(&(gattc_env[conidx]->server.pdu_queue)));

                // Check if something has to be completed
                atts_process_pdu(conidx);
            }
        }
        break;
#endif // (BLE_ATTS)
        default: /* Not expected, ignore message */
            break;
        }
    }

    /* message is consumed */
    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles data packet from L2CAP.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GATTC).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 *
 ****************************************************************************************
 */
__STATIC int l2cc_pdu_recv_ind_handler(ke_msg_id_t const msgid, struct l2cc_pdu_recv_ind *param,
                                       ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint8_t state = ke_state_get(dest_id);
    int msg_status = KE_MSG_CONSUMED;

    if (state != GATTC_FREE)
    {
        /* retrieve the record index from dest_id */
        uint8_t conidx = KE_IDX_GET(dest_id);

        if (param->status == GAP_ERR_NO_ERROR)
        {
            // request ATTS to process message.
            msg_status = atts_l2cc_pdu_recv_handler(conidx, param);

            // message not handled by Server side.
            if (msg_status == ATT_PDU_HANDLER_NOT_FOUND)
            {
                // request ATTC to process message.
                msg_status = attc_l2cc_pdu_recv_handler(conidx, param);
            }

            // no handler found, ignore the message
            if (msg_status == ATT_PDU_HANDLER_NOT_FOUND)
            {
                msg_status = KE_MSG_CONSUMED;
            }
        }
        else if ((param->status == L2C_ERR_INVALID_PDU) || (param->status == L2C_ERR_UNKNOWN_PDU))
        {
            // verify if the command bit is set onto op_code
            if (GETB(param->pdu.data.code, ATT_OPCODE_CMD_FLAG) == 0)
            {
                // it's a request which shall be responded
                struct l2cc_att_err_rsp *err_msg = L2CC_ATT_PDU_ALLOC(conidx, L2C_CODE_ATT_ERR_RSP,
                                                   KE_BUILD_ID(TASK_GATTC, conidx), l2cc_att_err_rsp);
                err_msg->handle = 0;
                err_msg->op_code = param->pdu.data.code;
                err_msg->reason = (param->status == L2C_ERR_UNKNOWN_PDU) ? ATT_ERR_REQUEST_NOT_SUPPORTED : ATT_ERR_INVALID_PDU;

                // Send message to L2CAP
                l2cc_pdu_send(err_msg);
            }
            // else the message shall be ignored since it's a unsupported command
        }
    }

    /* message is consumed */
    return (msg_status);
}

/**
 ****************************************************************************************
 * @brief Handles data send response from L2CAP.
 *  this is used to know when an indication has been correctly sent
 *
 * @param[in] evt     Id of the message received.
 * @param[in] evt     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC int l2cc_cmp_evt_handler(ke_msg_id_t const msgid, struct l2cc_cmp_evt *evt,
                                  ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint8_t state = ke_state_get(dest_id);
#if (BLE_ATTC || BLE_ATTS)
    uint8_t conidx = KE_IDX_GET(dest_id);
#endif // (BLE_ATTC || BLE_ATTS)

    if (state != GATTC_FREE)
    {
#if (BLE_ATTC)
        switch (gattc_get_operation(conidx, GATTC_OP_CLIENT))
        {
        case GATTC_WRITE_NO_RESPONSE:
        case GATTC_WRITE_SIGNED:
        {
            // inform the upper layer that operation has been successfully sent over the air
            gattc_send_complete_evt(conidx, GATTC_OP_CLIENT, evt->status);
        }
        break;
        default:
        {
            /* Do Nothing */
        }
        break;
        }
#endif // (BLE_ATTC)

#if (BLE_ATTS)
        if (gattc_get_operation(conidx, GATTC_OP_SERVER) == GATTC_NOTIFY)
        {
            /* Notification ready to be sent */
            gattc_send_complete_evt(conidx, GATTC_OP_SERVER, evt->status);
        }
#endif // (BLE_ATTS)
    }

    return (KE_MSG_CONSUMED);
}


/**
 ****************************************************************************************
 * @brief Default message handler that shall reply that ATTS/ATTC not present if
 *        corresponding handler not compiled.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_APP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 *
 ****************************************************************************************
 */
__STATIC int gattc_default_msg_handler(ke_msg_id_t const msgid, void const *param,
                                       ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    // Check if it's a GATTC message
    if (MSG_T(msgid) == TASK_ID_GATTC)
    {
        // prepare unknown message indication
        struct gattc_unknown_msg_ind *p_ind = KE_MSG_ALLOC(GATTC_UNKNOWN_MSG_IND,
                                              src_id, dest_id, gattc_unknown_msg_ind);

        p_ind->unknown_msg_id = msgid;

        // send event
        ke_msg_send(p_ind);

    }

    /* message is consumed */
    return (KE_MSG_CONSUMED);
}



#if (BLE_ATTS)
/**
 ****************************************************************************************
 * @brief Consumes attribute write in database message.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_APP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 *
 ****************************************************************************************
 */
__STATIC int gattc_write_req_ind_handler(ke_msg_id_t const msgid, struct gattc_write_req_ind *param,
        ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    // Connection Index
    uint8_t conidx = KE_IDX_GET(src_id);

    // Only the Service Changed CCC Descriptor can be written in the GATT service
    uint8_t state = ke_state_get(dest_id);
    if (state != GATTC_FREE)
    {
        // Status
        uint8_t status = ATT_ERR_APP_ERROR;
        // Get the received value
        uint16_t ccc_value = co_read16p(&(param->value[0]));

        /*
         * Valid values for the descriptor are:
         *    - 0x0000: Disable sending of indications
         *    - 0x0002: Enable sending of indications
         */
        if ((ccc_value == ATT_CCC_STOP_NTFIND) ||
                (ccc_value == ATT_CCC_START_IND))
        {
            // Inform the application about the new configuration
            struct gattc_svc_changed_cfg *ind = KE_MSG_ALLOC(GATTC_SVC_CHANGED_CFG_IND,
                                                gapc_get_dest_task(conidx),
                                                KE_BUILD_ID(TASK_GATTC, conidx),
                                                gattc_svc_changed_cfg);

            ind->ind_cfg = ccc_value;

            // Send the indication
            ke_msg_send(ind);

            // set value
            gapc_svc_chg_ccc_set(conidx, (ccc_value == ATT_CCC_START_IND));
            status = ATT_ERR_NO_ERROR;
        }

        // Send the write response
        struct gattc_write_cfm *cfm = KE_MSG_ALLOC(GATTC_WRITE_CFM,
                                      KE_BUILD_ID(TASK_GATTC, conidx), TASK_GATTM,
                                      gattc_write_cfm);

        cfm->status = status;
        cfm->handle = param->handle;
        ke_msg_send(cfm);

    }

    // Message is consumed
    return (KE_MSG_CONSUMED);
}



/**
 ****************************************************************************************
 * @brief Receive a request to get information about an attribute handle to write
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC int gattc_att_info_req_ind_handler(ke_msg_id_t const msgid, struct gattc_att_info_req_ind const *param,
        ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    // Current state
    uint8_t state = ke_state_get(dest_id);

    // Only the Service Changed CCC Descriptor can be written in the GATT service
    if (state != GATTC_FREE)
    {
        uint16_t length = 0;
        uint8_t status = ATT_ERR_NO_ERROR;

        if (param->handle == GATT_GET_ATT_HANDLE(GATT_IDX_SVC_CHANGED_CFG))
        {
            length = sizeof(uint16_t);
        }
        else
        {
            // Not supported
            status = ATT_ERR_APP_ERROR;
        }

        // send back response to ATTS
        struct gattc_att_info_cfm *cfm = KE_MSG_ALLOC(GATTC_ATT_INFO_CFM, src_id, dest_id, gattc_att_info_cfm);
        cfm->handle = param->handle;
        cfm->status = status;
        cfm->length = length;

        ke_msg_send(cfm);
    }

    /* message is consumed */
    return (KE_MSG_CONSUMED);
}



/**
 ****************************************************************************************
 * @brief handles request to read GATT database
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_APP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 *
 ****************************************************************************************
 */
__STATIC int gattc_read_req_ind_handler(ke_msg_id_t const msgid, struct gattc_write_req_ind *param,
                                        ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    // Connection Index
    uint8_t conidx = KE_IDX_GET(src_id);
    // Status
    uint8_t status = ATT_ERR_APP_ERROR;

    // Send the write response
    struct gattc_read_cfm *cfm = KE_MSG_ALLOC_DYN(GATTC_READ_CFM,
                                 KE_BUILD_ID(TASK_GATTC, conidx), KE_BUILD_ID(TASK_GATTC, conidx),
                                 gattc_read_cfm, sizeof(uint16_t));

    cfm->handle = param->handle;
    cfm->length = 0;

    // Only the Service Changed CCC Descriptor can be written in the GATT service
    if (param->handle == GATT_GET_ATT_HANDLE(GATT_IDX_SVC_CHANGED_CFG))
    {
        // retrieve CCC value
        cfm->length = sizeof(uint16_t);
        co_write16p(cfm->value, (gapc_svc_chg_ccc_get(conidx) ? ATT_CCC_START_IND : ATT_CCC_STOP_NTFIND));
        status = ATT_ERR_NO_ERROR;
    }

    cfm->status = status;

    ke_msg_send(cfm);

    // Message is consumed
    return (KE_MSG_CONSUMED);
}
#endif // (BLE_ATTS)


/*
 * TASK VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// The default message handlers
KE_MSG_HANDLER_TAB(gattc)
{
    // note: first message is latest message checked by kernel so default is put on top.
    { KE_MSG_DEFAULT_HANDLER, (ke_msg_func_t) gattc_default_msg_handler },

#if (BLE_ATTC)
    { GATTC_EXC_MTU_CMD, (ke_msg_func_t) gattc_exc_mtu_cmd_handler },
    { GATTC_DISC_CMD, (ke_msg_func_t) gattc_disc_cmd_handler },
    { GATTC_READ_CMD, (ke_msg_func_t) gattc_read_cmd_handler },
    { GATTC_WRITE_CMD, (ke_msg_func_t) gattc_write_cmd_handler },
    { GATTC_EVENT_CFM, (ke_msg_func_t) gattc_event_cfm_handler },

    { GATTC_EXECUTE_WRITE_CMD, (ke_msg_func_t) gattc_execute_write_cmd_handler },
    { GATTC_REG_TO_PEER_EVT_CMD, (ke_msg_func_t) gattc_reg_to_peer_evt_cmd_handler },
    { GATTC_CLIENT_RTX_IND, (ke_msg_func_t) gattc_timeout_handler },

    { GATTC_SDP_SVC_DISC_CMD, (ke_msg_func_t) gattc_sdp_svc_disc_cmd_handler },
    { GATTC_CMP_EVT, (ke_msg_func_t) gattc_cmp_evt_handler },
    { GATTC_DISC_SVC_IND, (ke_msg_func_t) gattc_disc_svc_ind_handler },
    { GATTC_DISC_SVC_INCL_IND, (ke_msg_func_t) gattc_disc_svc_incl_ind_handler },
    { GATTC_DISC_CHAR_IND, (ke_msg_func_t) gattc_disc_char_ind_handler },
    { GATTC_DISC_CHAR_DESC_IND, (ke_msg_func_t) gattc_disc_char_desc_ind_handler },
#endif // (BLE_ATTC)

#if (BLE_ATTS)
    { GATTC_READ_CFM, (ke_msg_func_t) gattc_read_cfm_handler },
    { GATTC_WRITE_CFM, (ke_msg_func_t) gattc_write_cfm_handler },
    { GATTC_ATT_INFO_CFM, (ke_msg_func_t) gattc_att_info_cfm_handler },
    { GATTC_SEND_SVC_CHANGED_CMD, (ke_msg_func_t) gattc_send_svc_changed_cmd_handler },
    { GATTC_SEND_EVT_CMD, (ke_msg_func_t) gattc_send_evt_cmd_handler },
    { GATTC_SERVER_RTX_IND, (ke_msg_func_t) gattc_timeout_handler },

    { GATTC_WRITE_REQ_IND, (ke_msg_func_t) gattc_write_req_ind_handler },
    { GATTC_ATT_INFO_REQ_IND, (ke_msg_func_t) gattc_att_info_req_ind_handler },
    { GATTC_READ_REQ_IND, (ke_msg_func_t) gattc_read_req_ind_handler },
#endif // (BLE_ATTS)

#ifndef SMP_SLIM
    { GAPC_SIGN_IND, (ke_msg_func_t) gapc_sign_ind_handler },
#endif // !GAPM_SLIM
    { GAPC_CMP_EVT, (ke_msg_func_t) gapc_cmp_evt_handler },
    { L2CC_PDU_RECV_IND, (ke_msg_func_t) l2cc_pdu_recv_ind_handler },
    { L2CC_CMP_EVT, (ke_msg_func_t) l2cc_cmp_evt_handler },
};

/// GATT task instance.
ke_state_t gattc_state[GATTC_IDX_MAX];


/// GATTC task descriptor
const struct ke_task_desc TASK_DESC_GATTC = {gattc_msg_handler_tab, gattc_state, GATTC_IDX_MAX, ARRAY_LEN(gattc_msg_handler_tab)};


#endif /* #if (BLE_CENTRAL || BLE_PERIPHERAL) */
/// @} GATTCTASK
