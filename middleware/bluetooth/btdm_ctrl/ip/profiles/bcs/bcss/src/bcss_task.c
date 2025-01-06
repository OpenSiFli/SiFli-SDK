/**
 ****************************************************************************************
 *
 * @file bcss_task.c
 *
 * @brief Body Composition Service Task Implementation.
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 * $ Rev $
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup BCSSTASK
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"

#if 1 //(BLE_BCS_SERVER)
#include "rwble_hl_error.h"
#include "gapc.h"
#include "gattc.h"
#include "gattc_task.h"
#include "attm.h"
#include "bcss.h"
#include "bcss_task.h"
#include "bcs_common.h"
#include "prf_utils.h"

#include "ke_mem.h"
#include "co_utils.h"
/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref BCSS_ENABLE_REQ message.
 * @param[in] msgid Id of the message received
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC int bcss_enable_req_handler(ke_msg_id_t const msgid,
                                     struct bcss_enable_req *param,
                                     ke_task_id_t const dest_id,
                                     ke_task_id_t const src_id)
{
    // Get the address of the environment
    struct bcss_env_tag *bcss_env = PRF_ENV_GET(BCSS, bcss);
    uint8_t conidx = KE_IDX_GET(src_id);
    // Status
    uint8_t status = PRF_ERR_REQ_DISALLOWED;
    uint8_t state = ke_state_get(dest_id);


    if (state == BCSS_IDLE)
    {
        bcss_env->prfl_ind_cfg[conidx] = param->ind_cfg;
        status = GAP_ERR_NO_ERROR;
    }

    // send completed information to APP task that contains error status
    struct bcss_enable_rsp *cmp_evt = KE_MSG_ALLOC(BCSS_ENABLE_RSP, src_id, dest_id, bcss_enable_rsp);
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
    if (state == BCSS_IDLE)
    {
        // Get the address of the environment
        struct bcss_env_tag *bcss_env = PRF_ENV_GET(BCSS, bcss);
        uint8_t conidx = KE_IDX_GET(src_id);
        uint8_t att_idx = param->handle - bcss_env->shdl;

        // Send data to peer device
        struct gattc_read_cfm *cfm = NULL;

        uint8_t status = ATT_ERR_NO_ERROR;

        switch (att_idx)
        {
        case BCSS_IDX_FEAT_VAL:
        {
            // Fill data
            cfm = KE_MSG_ALLOC_DYN(GATTC_READ_CFM, src_id, dest_id, gattc_read_cfm, sizeof(uint32_t));
            cfm->length = sizeof(uint32_t);
            co_write32p(cfm->value, bcss_env->feature);
        }
        break;

        case BCSS_IDX_MEAS_CCC:
        {
            // Fill data
            cfm = KE_MSG_ALLOC_DYN(GATTC_READ_CFM, src_id, dest_id, gattc_read_cfm, sizeof(uint16_t));
            cfm->length = sizeof(uint16_t);
            co_write16p(cfm->value, bcss_env->prfl_ind_cfg[conidx]);
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
    struct bcss_env_tag *bcss_env = PRF_ENV_GET(BCSS, bcss);
    uint8_t att_idx = param->handle - bcss_env->shdl;
    struct gattc_att_info_cfm *cfm;

    //Send write response
    cfm = KE_MSG_ALLOC(GATTC_ATT_INFO_CFM, src_id, dest_id, gattc_att_info_cfm);
    cfm->handle = param->handle;

    // check if it's a client configuration char
    if (att_idx == BCSS_IDX_MEAS_CCC)
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
 * @brief Handles reception of the @ref BCSS_MEAS_INDICATE_CMD message.
 * @brief Send MEASUREMENT INDICATION to the connected peer case of CCC enabled
 * @param[in] msgid Id of the message received.
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC int bcss_meas_indicate_cmd_handler(ke_msg_id_t const msgid,
        struct bcss_meas_indicate_cmd *param,
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

    // Get the address of the environment
    struct bcss_env_tag *bcss_env = PRF_ENV_GET(BCSS, bcss);

    // check that task is in idle state
    if (state == BCSS_IDLE)
    {
        if (bcss_env->prfl_ind_cfg[conidx] & CCC_IND_ENABLED)
        {
            // Allocate the GATT notification message

            struct gattc_send_evt_cmd *ind;
            // allocate the maxim block for indication
            uint8_t value[BCSS_MEAS_IND_SIZE];
            uint16_t max_payload = gattc_get_mtu(conidx) - 3;
            uint16_t next_segment_flags = 0;
            length = 0;

            /// Mask off any illegal bits in the flags field
            param->flags &=  BCS_MEAS_FLAGS_VALID;

            if (param->body_fat_percent == BCS_MEASUREMENT_UNSUCCESSFUL)
            {
                // If measurement was unsuccessfull - only time_stamp and user Id are valid fields.
                param->flags &=  BCS_MEAS_FLAGS_TIMESTAMP_PRESENT | BCS_MEAS_FLAGS_USER_ID_PRESENT;
            }

            ///**************************************************************
            /// Encode the Fields of the  Measurement
            /// if the Application provide flags and fields which do not correspond
            /// to the features declared by the server, we adjust the flags field to
            /// ensure we only Indicate with fields compatible with our features.
            /// Thus the flags fields is encoded last as it will be modifed by checks on
            /// features.
            ///
            /// If the Fields provided require 2 ATT_HANDLE_VALUE_INDs. The following will
            /// always be contained in the first message (if selected in flags field).
            ///          1/ Flags
            ///          2/ Body Fat (mandatory)
            ///          3/ Time Stamp
            ///          4/ User Id
            ///          5/ Basal Metabolism
            ///          6/ Muscle Percentage
            ///          7/ Muscle Mass
            ///          8/ Fat Free Mass ( total 20 Bytes)
            ///********************************************************************
            length += sizeof(uint16_t); // Flags is 16bit

            /// Mandatory Body Fat Percentage
            co_write16p(&value[length], param->body_fat_percent);
            length += sizeof(uint16_t);

            /// Time stamp if present and feature enabled
            /// We always include the Time-Stamp and User Id in the first message - if segmenting.

            if (param->flags & BCS_MEAS_FLAGS_TIMESTAMP_PRESENT)
            {
                // Tmp disabled during test
                //if (bcss_env->feature & BCS_TIME_STAMP_SUPPORTED)
                {
                    length += prf_pack_date_time(&value[length], &param->time_stamp);
                }
#if 0 // TEMP DISABLED during test
                else
                {
                    /// If Time-Stamp is not supported in the features - it should not be transmitted
                    param->flags &= ~BCS_MEAS_FLAGS_TIMESTAMP_PRESENT;
                }
#endif
            }

            /// User Id if present and Feature Enabled. If the Multiple users fields is enabled in the
            /// features and not present in the flags field - then an "UNKNOWN/GUEST" user is indicated.
            if (param->flags & BCS_MEAS_FLAGS_USER_ID_PRESENT)
            {
                if (bcss_env->feature & BCS_MULTIPLE_USERS_SUPPORTED)
                    value[length++] = param->user_id;
                else
                    param->flags &= ~BCS_MEAS_FLAGS_USER_ID_PRESENT;
            }
#if 0 // May impact conformance
            else if (bcss_env->feature & BCS_MULTIPLE_USERS_SUPPORTED)
            {
                value[length++] = BCS_MEAS_USER_ID_UNKNOWN_USER;
                param->flags |= BCS_MEAS_FLAGS_USER_ID_PRESENT;
            }
#endif
            /// Basal Metabolism if present and enabled in the features.
            if (param->flags & BCS_MEAS_FLAGS_BASAL_METAB_PRESENT)
            {
                if (bcss_env->feature & BCS_BASAL_METAB_SUPPORTED)
                {
                    co_write16p(&value[length], param->basal_metab);
                    length += sizeof(uint16_t);
                }
                else
                {
                    param->flags &= ~BCS_MEAS_FLAGS_BASAL_METAB_PRESENT;
                }
            }

            if (param->flags & BCS_MEAS_FLAGS_MUSCLE_PERCENT_PRESENT)
            {
                if (bcss_env->feature & BCS_MUSCLE_PERCENTAGE_SUPPORTED)
                {
                    co_write16p(&value[length], param->muscle_percent);
                    length += sizeof(uint16_t);
                }
                else
                {
                    param->flags &= ~BCS_MEAS_FLAGS_MUSCLE_PERCENT_PRESENT;
                }
            }

            if (param->flags & BCS_MEAS_FLAGS_MUSCLE_MASS_PRESENT)
            {
                if (bcss_env->feature & BCS_MUSCLE_MASS_SUPPORTED)
                {
                    co_write16p(&value[length], param->muscle_mass);
                    length += sizeof(uint16_t);
                }
                else
                {
                    param->flags &= ~BCS_MEAS_FLAGS_MUSCLE_MASS_PRESENT;
                }
            }

            if (param->flags & BCS_MEAS_FLAGS_FAT_FREE_MASS_PRESENT)
            {
                if (bcss_env->feature & BCS_FAT_FREE_MASS_SUPPORTED)
                {
                    co_write16p(&value[length], param->fat_free_mass);
                    length += sizeof(uint16_t);
                }
                else
                {
                    param->flags &= ~BCS_MEAS_FLAGS_FAT_FREE_MASS_PRESENT;
                }
            }

            if (param->flags & BCS_MEAS_FLAGS_SOFT_LEAN_MASS_PRESENT)
            {
                if (bcss_env->feature & BCS_SOFT_LEAN_MASS_SUPPORTED)
                {
                    if ((length + sizeof(uint16_t)) <= max_payload)
                    {
                        co_write16p(&value[length], param->soft_lean_mass);
                        length += sizeof(uint16_t);
                    }
                    else
                    {
                        next_segment_flags |= BCS_MEAS_FLAGS_SOFT_LEAN_MASS_PRESENT;
                    }
                }
                else
                {
                    param->flags &= ~BCS_MEAS_FLAGS_SOFT_LEAN_MASS_PRESENT;
                }
            }

            if (param->flags & BCS_MEAS_FLAGS_BODY_WATER_MASS_PRESENT)
            {
                if (bcss_env->feature & BCS_BODY_WATER_MASS_SUPPORTED)
                {
                    if ((length + sizeof(uint16_t)) <= max_payload)
                    {
                        co_write16p(&value[length], param->body_water_mass);
                        length += sizeof(uint16_t);
                    }
                    else
                    {
                        next_segment_flags |= BCS_MEAS_FLAGS_BODY_WATER_MASS_PRESENT;
                    }
                }
                else
                {
                    param->flags &= ~BCS_MEAS_FLAGS_BODY_WATER_MASS_PRESENT;
                }
            }

            if (param->flags & BCS_MEAS_FLAGS_IMPEDANCE_PRESENT)
            {
                if (bcss_env->feature & BCS_IMPEDANCE_SUPPORTED)
                {
                    if ((length + sizeof(uint16_t)) <= max_payload)
                    {
                        co_write16p(&value[length], param->impedance);
                        length += sizeof(uint16_t);
                    }
                    else
                    {
                        next_segment_flags |= BCS_MEAS_FLAGS_IMPEDANCE_PRESENT;
                    }
                }
                else
                {
                    param->flags &= ~BCS_MEAS_FLAGS_IMPEDANCE_PRESENT;
                }
            }

            if (param->flags & BCS_MEAS_FLAGS_WEIGHT_PRESENT)
            {
                if (bcss_env->feature & BCS_WEIGHT_SUPPORTED)
                {
                    if ((length + sizeof(uint16_t)) <= max_payload)
                    {
                        co_write16p(&value[length], param->weight);
                        length += sizeof(uint16_t);
                    }
                    else
                    {
                        next_segment_flags |= BCS_MEAS_FLAGS_WEIGHT_PRESENT;
                    }
                }
                else
                {
                    param->flags &= ~BCS_MEAS_FLAGS_WEIGHT_PRESENT;
                }
            }

            if (param->flags & BCS_MEAS_FLAGS_HEIGHT_PRESENT)
            {
                if (bcss_env->feature & BCS_HEIGHT_SUPPORTED)
                {
                    if ((length + sizeof(uint16_t)) <= max_payload)
                    {
                        co_write16p(&value[length], param->height);
                        length += sizeof(uint16_t);
                    }
                    else
                    {
                        next_segment_flags |= BCS_MEAS_FLAGS_HEIGHT_PRESENT;
                    }
                }
                else
                {
                    param->flags &= ~BCS_MEAS_FLAGS_HEIGHT_PRESENT;
                }
            }

            /// Finally store the flags in Byte 0 - the flags may have been changed in pre-ceeding steps.
            if (next_segment_flags)
            {
                /// Indicate this is a multipacket measurement.
                /// remove flags for fields which will not be transmitted in first segment
                param->flags |= BCS_MEAS_FLAGS_MULTIPACKET_MEAS;

                co_write16p(&value[0], (param->flags &= ~next_segment_flags));
                param->flags = next_segment_flags |= BCS_MEAS_FLAGS_MULTIPACKET_MEAS;
                bcss_env->meas_cmd_msg[conidx] = ke_param2msg(param);

                msg_status = KE_MSG_NO_FREE;
            }
            else
            {
                co_write16p(&value[0], param->flags);
                bcss_env->meas_cmd_msg[conidx] = 0;
            }

            // Allocate the GATT notification message
            ind = KE_MSG_ALLOC_DYN(GATTC_SEND_EVT_CMD,
                                   KE_BUILD_ID(TASK_GATTC, conidx),
                                   dest_id,
                                   gattc_send_evt_cmd, length);

            // Fill in the parameter structure
            ind->operation = GATTC_INDICATE;
            ind->handle = bcss_env->shdl + BCSS_IDX_MEAS_IND;
            // Pack Measurement record
            ind->length = length;
            // data
            memcpy(ind->value, &value[0], length);

            // Send the event
            ke_msg_send(ind);

            // go to busy state
            ke_state_set(dest_id, BCSS_OP_INDICATE);
        }
        else
        {
            /// Send a command complete to the App indicate msg could not be sent.
            bcss_send_cmp_evt(bcss_env,
                              conidx,
                              BCSS_MEAS_INDICATE_CMD_OP_CODE,
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
    struct bcss_env_tag *bcss_env = PRF_ENV_GET(BCSS, bcss);
    uint8_t conidx = KE_IDX_GET(src_id);
    // Message status
    uint8_t msg_status = KE_MSG_CONSUMED;
    // Status
    uint8_t status = GAP_ERR_NO_ERROR;

    // Check the connection handle
    if (bcss_env != NULL)
    {
        uint8_t state = ke_state_get(dest_id);
        // check that task is in idle state
        if (state == BCSS_IDLE)
        {
            uint8_t att_idx = param->handle - bcss_env->shdl;

            // CSC Measurement Characteristic, Client Characteristic Configuration Descriptor
            if (att_idx == BCSS_IDX_MEAS_CCC)
            {
                uint16_t ntf_cfg;

                // Get the value
                co_write16p(&ntf_cfg, param->value[0]);
                bcss_env->prfl_ind_cfg[conidx] = ntf_cfg;

                // Inform the HL about the new configuration
                struct bcss_wr_ccc_ind *ind = KE_MSG_ALLOC(BCSS_WR_CCC_IND,
                                              prf_dst_task_get(&bcss_env->prf_env, conidx),
                                              prf_src_task_get(&bcss_env->prf_env, conidx),
                                              bcss_wr_ccc_ind);

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
    struct bcss_env_tag *bcss_env = PRF_ENV_GET(BCSS, bcss);

    uint8_t state = ke_state_get(dest_id);

    if (bcss_env != NULL)
    {
        // Check if a connection exists
        switch (param->operation)
        {
        case (GATTC_INDICATE):
        {
            if (state == BCSS_OP_INDICATE)
            {
                if (bcss_env->meas_cmd_msg[conidx])
                {
                    /// If this is the CFM to the first part of a segmented message we must Tx the second part.
                    if (bcss_env->prfl_ind_cfg[conidx] & CCC_IND_ENABLED)
                    {
                        /// Allocate the GATT notification message
                        struct gattc_send_evt_cmd *ind;
                        struct bcss_meas_indicate_cmd *cmd = ke_msg2param(bcss_env->meas_cmd_msg[conidx]);
                        /// allocate the maxim block for indication
                        uint8_t value[BCSS_MEAS_IND_SIZE];
                        uint8_t length = 0;


                        ///**************************************************************
                        ///
                        /// Following fields will not be present - allways in first segment
                        ///          3/ Time Stamp
                        ///          4/ User Id
                        ///          5/ Basal Metabolism
                        ///          6/ Muscle Percentage
                        ///          7/ Muscle Mass
                        ///          8/ Fat Free Mass ( total 20 Bytes)
                        ///********************************************************************
                        co_write16p(&value[length], cmd->flags);
                        length += sizeof(uint16_t); /// Flags is 16bit

                        /// Mandatory Body Fat Percentage
                        co_write16p(&value[length], cmd->body_fat_percent);
                        length += sizeof(uint16_t);

                        if (cmd->flags & BCS_MEAS_FLAGS_SOFT_LEAN_MASS_PRESENT)
                        {
                            co_write16p(&value[length], cmd->soft_lean_mass);
                            length += sizeof(uint16_t);
                        }

                        if (cmd->flags & BCS_MEAS_FLAGS_BODY_WATER_MASS_PRESENT)
                        {
                            co_write16p(&value[length], cmd->body_water_mass);
                            length += sizeof(uint16_t);
                        }

                        if (cmd->flags & BCS_MEAS_FLAGS_IMPEDANCE_PRESENT)
                        {
                            co_write16p(&value[length], cmd->impedance);
                            length += sizeof(uint16_t);
                        }

                        if (cmd->flags & BCS_MEAS_FLAGS_WEIGHT_PRESENT)
                        {
                            co_write16p(&value[length], cmd->weight);
                            length += sizeof(uint16_t);
                        }

                        if (cmd->flags & BCS_MEAS_FLAGS_HEIGHT_PRESENT)
                        {
                            co_write16p(&value[length], cmd->height);
                            length += sizeof(uint16_t);
                        }

                        /// Finally store the flags in Byte 0 - the flags may have been changed in pre-ceeding steps.
                        co_write16p(&value[0], cmd->flags);
                        // Allocate the GATT notification message
                        ind = KE_MSG_ALLOC_DYN(GATTC_SEND_EVT_CMD,
                                               KE_BUILD_ID(TASK_GATTC, conidx),
                                               dest_id,
                                               gattc_send_evt_cmd, length);

                        /// Fill in the parameter structure
                        ind->operation = GATTC_INDICATE;
                        ind->handle = bcss_env->shdl + BCSS_IDX_MEAS_IND;
                        ind->length = length;
                        /// data
                        memcpy(ind->value, &value[0], length);

                        /// Send the event
                        ke_msg_send(ind);

                    }
                    ke_msg_free(bcss_env->meas_cmd_msg[conidx]);
                    bcss_env->meas_cmd_msg[conidx] = 0;
                }
                else
                {
                    /// Inform the application that a procedure has been completed
                    bcss_send_cmp_evt(bcss_env,
                                      conidx,
                                      BCSS_MEAS_INDICATE_CMD_OP_CODE,
                                      param->status);

                    ke_state_set(dest_id, BCSS_IDLE);
                }


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
KE_MSG_HANDLER_TAB(bcss)
{
    {BCSS_ENABLE_REQ, (ke_msg_func_t) bcss_enable_req_handler},
    /// Send a BCS Measurement to the peer device (Indication)
    {BCSS_MEAS_INDICATE_CMD, (ke_msg_func_t) bcss_meas_indicate_cmd_handler},

    {GATTC_READ_REQ_IND, (ke_msg_func_t) gattc_read_req_ind_handler},
    {GATTC_ATT_INFO_REQ_IND, (ke_msg_func_t) gattc_att_info_req_ind_handler},
    {GATTC_WRITE_REQ_IND, (ke_msg_func_t) gattc_write_req_ind_handler},
    {GATTC_CMP_EVT, (ke_msg_func_t) gattc_cmp_evt_handler},
};

void bcss_task_init(struct ke_task_desc *task_desc)
{
    // Get the address of the environment
    struct bcss_env_tag *bcss_env = PRF_ENV_GET(BCSS, bcss);

    task_desc->msg_handler_tab = bcss_msg_handler_tab;
    task_desc->msg_cnt         = ARRAY_LEN(bcss_msg_handler_tab);
    task_desc->state           = bcss_env->state;
    task_desc->idx_max         = BCSS_IDX_MAX;
}

#endif //(BLE_BCS_SERVER)

/// @} BCSTASK
