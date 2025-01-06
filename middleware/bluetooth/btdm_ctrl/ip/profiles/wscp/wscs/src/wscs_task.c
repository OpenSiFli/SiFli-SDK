/**
 ****************************************************************************************
 *
 * @file wscs_task.c
 *
 * @brief Weight SCale Profile Sensor Task Implementation.
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 * $ Rev $
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup WSCSTASK
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"

#if (BLE_WSC_SERVER)
#include "wsc_common.h"

#include "gapc.h"
#include "gattc_task.h"
#include "attm.h"
#include "wscs.h"
#include "wscs_task.h"
#include "prf_utils.h"

#include "ke_mem.h"
#include "co_utils.h"
/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref WSCS_ENABLE_REQ message.
 * @param[in] msgid Id of the message received
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC int wscs_enable_req_handler(ke_msg_id_t const msgid,
                                     struct wscs_enable_req *param,
                                     ke_task_id_t const dest_id,
                                     ke_task_id_t const src_id)
{
    // Get the address of the environment
    struct wscs_env_tag *wscs_env = PRF_ENV_GET(WSCS, wscs);
    uint8_t conidx = KE_IDX_GET(src_id);
    // Status
    uint8_t status = PRF_ERR_REQ_DISALLOWED;
    uint8_t state = ke_state_get(dest_id);


    if (state == WSCS_IDLE)
    {
        wscs_env->prfl_ind_cfg[conidx] = param->ind_cfg;
        status = GAP_ERR_NO_ERROR;
    }

    // send completed information to APP task that contains error status
    struct wscs_enable_rsp *cmp_evt = KE_MSG_ALLOC(WSCS_ENABLE_RSP, src_id, dest_id, wscs_enable_rsp);
    cmp_evt->status     = status;

    ke_msg_send(cmp_evt);

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the read request from peer device
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC int gattc_read_req_ind_handler(ke_msg_id_t const msgid,
                                        struct gattc_read_req_ind const *param,
                                        ke_task_id_t const dest_id,
                                        ke_task_id_t const src_id)
{
    int msg_status = KE_MSG_CONSUMED;
    uint8_t state = ke_state_get(dest_id);
    // check that task is in idle state
    if (state == WSCS_IDLE)
    {
        // Get the address of the environment
        struct wscs_env_tag *wscs_env = PRF_ENV_GET(WSCS, wscs);
        uint8_t conidx = KE_IDX_GET(src_id);
        uint8_t att_idx = param->handle - wscs_env->shdl;

        // Send data to peer device
        struct gattc_read_cfm *cfm = NULL;

        uint8_t status = ATT_ERR_NO_ERROR;

        switch (att_idx)
        {
        case WSCS_IDX_FEAT_VAL:
        {
            // Fill data
            cfm = KE_MSG_ALLOC_DYN(GATTC_READ_CFM, src_id, dest_id, gattc_read_cfm, sizeof(uint32_t));
            cfm->length = sizeof(uint32_t);
            co_write32p(cfm->value, wscs_env->feature);
        }
        break;

        case WSCS_IDX_MEAS_CCC:
        {
            // Fill data
            cfm = KE_MSG_ALLOC_DYN(GATTC_READ_CFM, src_id, dest_id, gattc_read_cfm, sizeof(uint16_t));
            cfm->length = sizeof(uint16_t);
            co_write16p(cfm->value, wscs_env->prfl_ind_cfg[conidx]);
        }
        break;

        default:
        {
        } break;
        }

        if (cfm == NULL)
        {
            cfm = KE_MSG_ALLOC(GATTC_READ_CFM, src_id, dest_id, gattc_read_cfm);
            cfm->length = 0;
            status = ATT_ERR_REQUEST_NOT_SUPPORTED;
        }

        cfm->handle = param->handle;
        cfm->status = status;

        // Send value to peer device.
        ke_msg_send(cfm);
    }
    // else process it later
    else
    {
        msg_status = KE_MSG_SAVED;
    }
    return (msg_status);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the attribute info request message.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC int gattc_att_info_req_ind_handler(ke_msg_id_t const msgid,
        struct gattc_att_info_req_ind *param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
    // Get the address of the environment
    struct wscs_env_tag *wscs_env = PRF_ENV_GET(WSCS, wscs);
    uint8_t att_idx = param->handle - wscs_env->shdl;
    struct gattc_att_info_cfm *cfm;

    //Send write response
    cfm = KE_MSG_ALLOC(GATTC_ATT_INFO_CFM, src_id, dest_id, gattc_att_info_cfm);
    cfm->handle = param->handle;

    // check if it's a client configuration char
    if (att_idx == WSCS_IDX_MEAS_CCC)
    {
        // CCC attribute length = 2
        cfm->length = sizeof(uint16_t);
        cfm->status = GAP_ERR_NO_ERROR;
    }
    else // not expected request
    {
        cfm->length = 0;
        cfm->status = ATT_ERR_WRITE_NOT_PERMITTED;
    }

    ke_msg_send(cfm);

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref WSCS_MEAS_INDICATE_CMD message.
 * @brief Send MEASUREMENT INDICATION to the connected peer case of CCC enabled
 * @param[in] msgid Id of the message received.
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC int wscs_meas_indicate_cmd_handler(ke_msg_id_t const msgid,
        struct wscs_meas_indicate_cmd *param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
    // Message status
    int msg_status = KE_MSG_CONSUMED;
    // State
    uint8_t state = ke_state_get(dest_id);
    // Connection index
    uint8_t conidx = KE_IDX_GET(dest_id);
    uint16_t length;
    uint8_t status = ATT_ERR_NO_ERROR;

    // Get the address of the environment
    struct wscs_env_tag *wscs_env = PRF_ENV_GET(WSCS, wscs);

    // check that task is in idle state
    if (state == WSCS_IDLE)
    {
        if (wscs_env->prfl_ind_cfg[conidx] & CCC_IND_ENABLED)
        {
            // Allocate the GATT notification message
            struct gattc_send_evt_cmd *ind;
            // allocate the maximum block for indication
            uint8_t value[WSCS_MEAS_IND_SIZE];

            length = 0;

            // Mask off any illegal bits in the flags field
            param->flags &= WSC_FLAGS_MASK;
            // If the Weight indicates Measurement Unsuccessful - 0xFFFF
            // The the flags field should not include BMI and Height

            if (param->weight == WS_MEASUREMENT_UNSUCCESSFUL)
            {
                param->flags &= (WS_MEAS_FLAGS_UNITS_IMPERIAL | WS_MEAS_FLAGS_TIMESTAMP_PRESENT | WS_MEAS_FLAGS_USER_ID_PRESENT);
            }
            else
            {
                // If Body Composition Service is included then Mandatory to have
                // both the Height and BMI fields present (in flags) otherwise
                // reject command.

                if ((wscs_env->bcs_included) && (!(param->flags & WS_MEAS_FLAGS_BMI_PRESENT)))
                {
                    status = PRF_ERR_INVALID_PARAM;
                }
            }
            ///**************************************************************
            /// Encode the Fields of the Weight Measurement
            /// if the Application provide flags and fields which do not correspond
            /// to the features declared by the server, we adjust the flags field to
            /// ensure we only Indicate with fields compatible with our features.
            /// Thus the flags fields is encoded last as it will be modifed by checks on
            /// features.
            ///********************************************************************
            if (status == ATT_ERR_NO_ERROR)
            {
                length++;
                // Mandatory weight
                // pack measured value in database
                co_write16p(&value[length], param->weight);
                length += sizeof(uint16_t);

                // Time stamp if present and feature enabled
                if (param->flags & WS_MEAS_FLAGS_TIMESTAMP_PRESENT)
                {
                    if (wscs_env->feature & WSC_TIME_STAMP_SUPPORTED)
                    {
                        length += prf_pack_date_time(&value[length], &param->time_stamp);
                    }
                    else
                    {
                        // If Time-Stamp is not supported in the features - it should not be transmitted
                        param->flags &= ~WS_MEAS_FLAGS_TIMESTAMP_PRESENT;
                    }
                }

                /// User Id if present and Feature Enabled. If the Multiple users fields is enabled in the
                /// features and not present in the flags field - then an "UNKNOWN/GUEST" user is indicated.
                if (param->flags & WS_MEAS_FLAGS_USER_ID_PRESENT)
                {
                    if (wscs_env->feature & WSC_MULTIPLE_USERS_SUPPORTED)
                        value[length++] = param->user_id;
                    else
                        param->flags &= ~WS_MEAS_FLAGS_USER_ID_PRESENT;
                }
                else if (wscs_env->feature & WSC_MULTIPLE_USERS_SUPPORTED)
                {
                    value[length++] = WS_MEAS_USER_ID_UNKNOWN_USER;
                    param->flags |= WS_MEAS_FLAGS_USER_ID_PRESENT;
                }

                /// BMI & Height if present and enabled in the features.
                if (param->flags & WS_MEAS_FLAGS_BMI_PRESENT)
                {
                    if (wscs_env->feature & WSC_BMI_SUPPORTED_SUPPORTED)
                    {
                        co_write16p(&value[length], param->bmi);
                        length += sizeof(uint16_t);
                        co_write16p(&value[length], param->height);
                        length += sizeof(uint16_t);
                    }
                    else
                    {
                        param->flags &= ~WS_MEAS_FLAGS_BMI_PRESENT;
                    }
                }
                // Finally store the flags in Byte 0 - the flags may have been changed in pre-ceeding steps.
                value[0] = param->flags;

                // Allocate the GATT notification message
                ind = KE_MSG_ALLOC_DYN(GATTC_SEND_EVT_CMD,
                                       KE_BUILD_ID(TASK_GATTC, conidx),
                                       dest_id,
                                       gattc_send_evt_cmd, length);

                // Fill in the parameter structure
                ind->operation = GATTC_INDICATE;
                ind->handle = wscs_env->shdl + WSCS_IDX_MEAS_IND;
                // Pack Measurement record
                ind->length = length;
                memcpy(ind->value, &value[0], length);

                // Send the event
                ke_msg_send(ind);

                // go to busy state
                ke_state_set(dest_id, WSCS_OP_INDICATE);
            }
            else
            {
                // Send a command complete to the App, indicate msg has wrong parameters.
                wscs_send_cmp_evt(wscs_env,
                                  conidx,
                                  WSCS_MEAS_INDICATE_CMD_OP_CODE,
                                  status);
            }
        }
        else
        {
            /// Send a command complete to the App indicate msg could not be sent.
            wscs_send_cmp_evt(wscs_env,
                              conidx,
                              WSCS_MEAS_INDICATE_CMD_OP_CODE,
                              PRF_ERR_IND_DISABLED);
        }

    }
    // else process it later
    else
    {
        msg_status = KE_MSG_SAVED;
    }
    return (msg_status);
}


/**
 ****************************************************************************************
 * @brief Handles reception of the @ref GATTC_WRITE_REQ_IND message.
 * @param[in] msgid Id of the message received.
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC int gattc_write_req_ind_handler(ke_msg_id_t const msgid,
        struct gattc_write_req_ind const *param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
    // Get the address of the environment
    struct wscs_env_tag *wscs_env = PRF_ENV_GET(WSCS, wscs);
    uint8_t conidx = KE_IDX_GET(src_id);
    // Message status
    uint8_t msg_status = KE_MSG_CONSUMED;
    // Status
    uint8_t status = GAP_ERR_NO_ERROR;

    // Check the connection handle
    if (wscs_env != NULL)
    {
        uint8_t state = ke_state_get(dest_id);
        // check that task is in idle state
        if (state == WSCS_IDLE)
        {
            uint8_t att_idx = param->handle - wscs_env->shdl;

            // CSC Measurement Characteristic, Client Characteristic Configuration Descriptor
            if (att_idx == WSCS_IDX_MEAS_CCC)
            {
                uint16_t ntf_cfg;

                // Get the value
                co_write16p(&ntf_cfg, param->value[0]);
                wscs_env->prfl_ind_cfg[conidx] = ntf_cfg;

                // Inform the HL about the new configuration
                struct wscs_wr_ccc_ind *ind = KE_MSG_ALLOC(WSCS_WR_CCC_IND,
                                              prf_dst_task_get(&wscs_env->prf_env, conidx),
                                              prf_src_task_get(&wscs_env->prf_env, conidx),
                                              wscs_wr_ccc_ind);

                ind->ind_cfg = ntf_cfg;

                ke_msg_send(ind);

            }
            else
            {
                status = PRF_ERR_INVALID_PARAM;
            }

            // Send the write response to the peer device
            struct gattc_write_cfm *cfm = KE_MSG_ALLOC(
                                              GATTC_WRITE_CFM, src_id, dest_id, gattc_write_cfm);
            cfm->handle = param->handle;
            cfm->status = status;
            ke_msg_send(cfm);
        }
        else
        {
            msg_status = KE_MSG_SAVED;
        }
    }
    // else drop the message

    return (int)msg_status;
}

/**
 ****************************************************************************************
 * @brief Handles @ref GATT_NOTIFY_CMP_EVT message meaning that an indication
 * has been correctly sent to peer device (but not confirmed by peer device).
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC int gattc_cmp_evt_handler(ke_msg_id_t const msgid,  struct gattc_cmp_evt const *param,
                                   ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint8_t conidx = KE_IDX_GET(src_id);
    // Get the address of the environment
    struct wscs_env_tag *wscs_env = PRF_ENV_GET(WSCS, wscs);

    uint8_t state = ke_state_get(dest_id);

    if (wscs_env != NULL)
    {
        // Check if a connection exists
        switch (param->operation)
        {
        case (GATTC_INDICATE):
        {
            // Inform the application that a procedure has been completed
            if (state == WSCS_OP_INDICATE)
            {
                wscs_send_cmp_evt(wscs_env,
                                  conidx,
                                  WSCS_MEAS_INDICATE_CMD_OP_CODE,
                                  param->status);

                ke_state_set(dest_id, WSCS_IDLE);
            }

        }
        break;
        // else ignore the message
        default:
        {
            ASSERT_ERR(0);
        }
        break;
        }
    }

    return (KE_MSG_CONSUMED);
}

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// Specifies the default message handlers
KE_MSG_HANDLER_TAB(wscs)
{
    {WSCS_ENABLE_REQ, (ke_msg_func_t) wscs_enable_req_handler},
    /// Send a WSC Measurement to the peer device (Indication)
    {WSCS_MEAS_INDICATE_CMD, (ke_msg_func_t) wscs_meas_indicate_cmd_handler},

    {GATTC_READ_REQ_IND, (ke_msg_func_t) gattc_read_req_ind_handler},
    {GATTC_ATT_INFO_REQ_IND, (ke_msg_func_t) gattc_att_info_req_ind_handler},
    {GATTC_WRITE_REQ_IND, (ke_msg_func_t) gattc_write_req_ind_handler},
    {GATTC_CMP_EVT, (ke_msg_func_t) gattc_cmp_evt_handler},
};

void wscs_task_init(struct ke_task_desc *task_desc)
{
    // Get the address of the environment
    struct wscs_env_tag *wscs_env = PRF_ENV_GET(WSCS, wscs);

    task_desc->msg_handler_tab = wscs_msg_handler_tab;
    task_desc->msg_cnt         = ARRAY_LEN(wscs_msg_handler_tab);
    task_desc->state           = wscs_env->state;
    task_desc->idx_max         = WSCS_IDX_MAX;
}

#endif //(BLE_WSC_SERVER)

/// @} WSCSTASK
