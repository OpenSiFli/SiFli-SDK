/**
 ****************************************************************************************
 *
 * @file udsc_task.c
 *
 * @brief User Data Service Collector Task implementation.
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 * $ Rev $
 *
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @addtogroup UDSCTASK
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"

#if (BLE_UDS_CLIENT)
#include "uds_common.h"
#include "udsc_task.h"
#include "udsc.h"

#include "gap.h"
#include "attm.h"
#include "gattc_task.h"
#include "ke_mem.h"
#include "co_utils.h"


/*
 * STRUCTURES
 ****************************************************************************************
 */

/// State machine used to retrieve User Data Service Service characteristics information
const struct prf_char_def udsc_uds_char[UDSC_CHAR_UDS_MAX] =
{
    [UDSC_CHAR_UDS_DB_CHG_INC] = {
        ATT_CHAR_DATABASE_CHANGE_INCREMENT,
        ATT_MANDATORY,
        (ATT_CHAR_PROP_RD | ATT_CHAR_PROP_WR | ATT_CHAR_PROP_NTF)
    },

    [UDSC_CHAR_UDS_USER_INDEX] = {
        ATT_CHAR_USER_INDEX,
        ATT_MANDATORY,
        (ATT_CHAR_PROP_RD)
    },

    [UDSC_CHAR_UDS_USER_CTRL_PT]   = {
        ATT_CHAR_USER_CONTROL_POINT,
        ATT_MANDATORY,
        (ATT_CHAR_PROP_WR | ATT_CHAR_PROP_IND)
    },

    /// Strings: utf8s
    [UDSC_CHAR_UDS_FIRST_NAME]   = {
        ATT_CHAR_FIRST_NAME,
        ATT_OPTIONAL,
        (ATT_CHAR_PROP_RD | ATT_CHAR_PROP_WR | ATT_CHAR_PROP_EXT_PROP)
    },

    [UDSC_CHAR_UDS_LAST_NAME]   = {
        ATT_CHAR_LAST_NAME,
        ATT_OPTIONAL,
        (ATT_CHAR_PROP_RD | ATT_CHAR_PROP_WR | ATT_CHAR_PROP_EXT_PROP)
    },

    [UDSC_CHAR_UDS_EMAIL_ADDRESS]   = {
        ATT_CHAR_EMAIL_ADDRESS,
        ATT_OPTIONAL,
        (ATT_CHAR_PROP_RD | ATT_CHAR_PROP_WR | ATT_CHAR_PROP_EXT_PROP)
    },

    [UDSC_CHAR_UDS_LANGUAGE]   = {
        ATT_CHAR_LANGUAGE,
        ATT_OPTIONAL,
        (ATT_CHAR_PROP_RD | ATT_CHAR_PROP_WR | ATT_CHAR_PROP_EXT_PROP)
    },

    /// Date
    [UDSC_CHAR_UDS_DATE_OF_BIRTH]   = {
        ATT_CHAR_DATE_OF_BIRTH,
        ATT_OPTIONAL,
        (ATT_CHAR_PROP_RD | ATT_CHAR_PROP_WR)
    },

    [UDSC_CHAR_UDS_DATE_OF_THRESHOLD_ASSESSMENT]   = {
        ATT_CHAR_DATE_OF_THRESHOLD_ASSESSMENT,
        ATT_OPTIONAL,
        (ATT_CHAR_PROP_RD | ATT_CHAR_PROP_WR)
    },

    /// uint16
    [UDSC_CHAR_UDS_WEIGHT]   = {
        ATT_CHAR_WEIGHT,
        ATT_OPTIONAL,
        (ATT_CHAR_PROP_RD | ATT_CHAR_PROP_WR)
    },

    [UDSC_CHAR_UDS_HEIGHT]   = {
        ATT_CHAR_HEIGHT,
        ATT_OPTIONAL,
        (ATT_CHAR_PROP_RD | ATT_CHAR_PROP_WR)
    },

    [UDSC_CHAR_UDS_WAIST_CIRCUMFERENCE]   = {
        ATT_CHAR_WAIST_CIRCUMFERENCE,
        ATT_OPTIONAL,
        (ATT_CHAR_PROP_RD | ATT_CHAR_PROP_WR)
    },

    [UDSC_CHAR_UDS_HIP_CIRCUMFERENCE]   = {
        ATT_CHAR_HIP_CIRCUMFERENCE,
        ATT_OPTIONAL,
        (ATT_CHAR_PROP_RD | ATT_CHAR_PROP_WR)
    },

    /// uint8
    [UDSC_CHAR_UDS_AGE]   = {
        ATT_CHAR_AGE,
        ATT_OPTIONAL,
        (ATT_CHAR_PROP_RD | ATT_CHAR_PROP_WR)
    },

    [UDSC_CHAR_UDS_GENDER]   = {
        ATT_CHAR_GENDER,
        ATT_OPTIONAL,
        (ATT_CHAR_PROP_RD | ATT_CHAR_PROP_WR)
    },

    [UDSC_CHAR_UDS_VO2_MAX]   = {
        ATT_CHAR_VO2_MAX,
        ATT_OPTIONAL,
        (ATT_CHAR_PROP_RD | ATT_CHAR_PROP_WR)
    },

    [UDSC_CHAR_UDS_MAX_HEART_RATE]   = {
        ATT_CHAR_MAX_HEART_RATE,
        ATT_OPTIONAL,
        (ATT_CHAR_PROP_RD | ATT_CHAR_PROP_WR)
    },

    [UDSC_CHAR_UDS_RESTING_HEART_RATE]   = {
        ATT_CHAR_RESTING_HEART_RATE,
        ATT_OPTIONAL,
        (ATT_CHAR_PROP_RD | ATT_CHAR_PROP_WR)
    },

    [UDSC_CHAR_UDS_MAXIMUM_RECOMMENDED_HEART_RATE]   = {
        ATT_CHAR_MAXIMUM_RECOMMENDED_HEART_RATE,
        ATT_OPTIONAL,
        (ATT_CHAR_PROP_RD | ATT_CHAR_PROP_WR)
    },

    [UDSC_CHAR_UDS_AEROBIC_THRESHHOLD]   = {
        ATT_CHAR_AEROBIC_THRESHHOLD,
        ATT_OPTIONAL,
        (ATT_CHAR_PROP_RD | ATT_CHAR_PROP_WR)
    },

    [UDSC_CHAR_UDS_ANAEROBIC_THRESHHOLD]   = {
        ATT_CHAR_ANAEROBIC_THRESHHOLD,
        ATT_OPTIONAL,
        (ATT_CHAR_PROP_RD | ATT_CHAR_PROP_WR)
    },

    [UDSC_CHAR_UDS_SPORT_TYPE_FOR_AEROBIC_AND_ANAEROBIC_THRESHOLDS]   = {
        ATT_CHAR_SPORT_TYPE_FOR_AEROBIC_AND_ANAEROBIC_THRESHOLDS,
        ATT_OPTIONAL,
        (ATT_CHAR_PROP_RD | ATT_CHAR_PROP_WR)
    },

    [UDSC_CHAR_UDS_FAT_BURN_HEART_RATE_LOWER_LIMIT]   = {
        ATT_CHAR_FAT_BURN_HEART_RATE_LOWER_LIMIT,
        ATT_OPTIONAL,
        (ATT_CHAR_PROP_RD | ATT_CHAR_PROP_WR)
    },

    [UDSC_CHAR_UDS_FAT_BURN_HEART_RATE_UPPER_LIMIT]   = {
        ATT_CHAR_FAT_BURN_HEART_RATE_UPPER_LIMIT,
        ATT_OPTIONAL,
        (ATT_CHAR_PROP_RD | ATT_CHAR_PROP_WR)
    },

    [UDSC_CHAR_UDS_AEROBIC_HEART_RATE_LOWER_LIMIT]   = {
        ATT_CHAR_AEROBIC_HEART_RATE_LOWER_LIMIT,
        ATT_OPTIONAL,
        (ATT_CHAR_PROP_RD | ATT_CHAR_PROP_WR)
    },

    [UDSC_CHAR_UDS_AEROBIC_HEART_RATE_UPPER_LIMIT]   = {
        ATT_CHAR_AEROBIC_HEART_RATE_UPPER_LIMIT,
        ATT_OPTIONAL,
        (ATT_CHAR_PROP_RD | ATT_CHAR_PROP_WR)
    },

    [UDSC_CHAR_UDS_ANAEROBIC_HEART_RATE_LOWER_LIMIT]   = {
        ATT_CHAR_ANAEROBIC_HEART_RATE_LOWER_LIMIT,
        ATT_OPTIONAL,
        (ATT_CHAR_PROP_RD | ATT_CHAR_PROP_WR)
    },

    [UDSC_CHAR_UDS_ANAEROBIC_HEART_RATE_UPPER_LIMIT]   = {
        ATT_CHAR_ANAEROBIC_HEART_RATE_UPPER_LIMIT,
        ATT_OPTIONAL,
        (ATT_CHAR_PROP_RD | ATT_CHAR_PROP_WR)
    },

    [UDSC_CHAR_UDS_TWO_ZONE_HEART_RATE_LIMIT]   = {
        ATT_CHAR_TWO_ZONE_HEART_RATE_LIMIT,
        ATT_OPTIONAL,
        (ATT_CHAR_PROP_RD | ATT_CHAR_PROP_WR)
    },

    /// set
    [UDSC_CHAR_UDS_FIVE_ZONE_HEART_RATE_LIMITS]   = {
        ATT_CHAR_FIVE_ZONE_HEART_RATE_LIMITS,
        ATT_OPTIONAL,
        (ATT_CHAR_PROP_RD | ATT_CHAR_PROP_WR)
    },

    [UDSC_CHAR_UDS_THREE_ZONE_HEART_RATE_LIMITS]   = {
        ATT_CHAR_THREE_ZONE_HEART_RATE_LIMITS,
        ATT_OPTIONAL,
        (ATT_CHAR_PROP_RD | ATT_CHAR_PROP_WR)
    },

    /// user defined
    [UDSC_CHAR_UDS_USER_DEFINED_1]   = {
        ATT_CHAR_FIVE_ZONE_HEART_RATE_LIMITS,
        ATT_OPTIONAL,
        (ATT_CHAR_PROP_RD | ATT_CHAR_PROP_WR)
    },

    [UDSC_CHAR_UDS_USER_DEFINED_2]   = {
        ATT_CHAR_THREE_ZONE_HEART_RATE_LIMITS,
        ATT_OPTIONAL,
        (ATT_CHAR_PROP_RD | ATT_CHAR_PROP_WR)
    },
    [UDSC_CHAR_UDS_USER_DEFINED_3]   = {
        ATT_CHAR_FIVE_ZONE_HEART_RATE_LIMITS,
        ATT_OPTIONAL,
        (ATT_CHAR_PROP_RD | ATT_CHAR_PROP_WR)
    },

    [UDSC_CHAR_UDS_USER_DEFINED_4]   = {
        ATT_CHAR_THREE_ZONE_HEART_RATE_LIMITS,
        ATT_OPTIONAL,
        (ATT_CHAR_PROP_RD | ATT_CHAR_PROP_WR)
    },
};

/// State machine used to retrieve User Data Service Service characteristic description information
const struct prf_char_desc_def udsc_uds_char_desc[UDSC_DESC_UDS_MAX] =
{
    /// Database Change Increment Client config
    [UDSC_DESC_UDS_DB_CHG_INC_CCC] = {
        ATT_DESC_CLIENT_CHAR_CFG,
        ATT_MANDATORY,
        UDSC_CHAR_UDS_DB_CHG_INC
    },

    /// User Control Point Client config
    [UDSC_DESC_UDS_USER_CTRL_PT_CCC] = {
        ATT_DESC_CLIENT_CHAR_CFG,
        ATT_MANDATORY,
        UDSC_CHAR_UDS_USER_CTRL_PT
    },

    /// String Extended Properties
    [UDSC_DESC_UDS_FIRST_NAME_EXT] = {
        ATT_DESC_CHAR_EXT_PROPERTIES,
        ATT_OPTIONAL,
        UDSC_CHAR_UDS_FIRST_NAME
    },

    /// String Extended Properties
    [UDSC_DESC_UDS_LAST_NAME_EXT] = {
        ATT_DESC_CHAR_EXT_PROPERTIES,
        ATT_OPTIONAL,
        UDSC_CHAR_UDS_LAST_NAME
    },

    /// String Extended Properties
    [UDSC_DESC_UDS_EMAIL_ADDRESS_EXT] = {
        ATT_DESC_CHAR_EXT_PROPERTIES,
        ATT_OPTIONAL,
        UDSC_CHAR_UDS_EMAIL_ADDRESS
    },

    /// String Extended Properties
    [UDSC_DESC_UDS_LANGUAGE_EXT] = {
        ATT_DESC_CHAR_EXT_PROPERTIES,
        ATT_OPTIONAL,
        UDSC_CHAR_UDS_LANGUAGE
    },
};


/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref GATTC_SDP_SVC_IND_HANDLER message.
 * The handler stores the found service details for service discovery.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC int gattc_sdp_svc_ind_handler(ke_msg_id_t const msgid,
                                       struct gattc_sdp_svc_ind const *ind,
                                       ke_task_id_t const dest_id,
                                       ke_task_id_t const src_id)
{

    uint8_t conidx = KE_IDX_GET(dest_id);
    uint8_t state = ke_state_get(dest_id);

    if (state == UDSC_DISCOVERING_SVC)
    {
        /// User Data Service Collector Role Task Environment
        struct udsc_env_tag *udsc_env = PRF_ENV_GET(UDSC, udsc);

        if (udsc_env != NULL)
        {

            ASSERT_INFO(udsc_env != NULL, dest_id, src_id);
            ASSERT_INFO(udsc_env->env[conidx] != NULL, dest_id, src_id);

            if (attm_uuid16_comp((unsigned char *)ind->uuid, ind->uuid_len, ATT_SVC_USER_DATA))
            {
                // Retrieve WS characteristics and descriptors
                prf_extract_svc_info(ind, UDSC_CHAR_UDS_MAX, &udsc_uds_char[0],  &udsc_env->env[conidx]->uds.chars[0],
                                     UDSC_DESC_UDS_MAX, &udsc_uds_char_desc[0], &udsc_env->env[conidx]->uds.descs[0]);

                //Even if we get multiple responses we only store 1 range
                udsc_env->env[conidx]->uds.svc.shdl = ind->start_hdl;
                udsc_env->env[conidx]->uds.svc.ehdl = ind->end_hdl;

                udsc_env->env[conidx]->nb_svc++;
            }
        }
    }

    return (KE_MSG_CONSUMED);
}
/**
 ****************************************************************************************
 * @brief Handles reception of the @ref UDSC_ENABLE_REQ message.
 * @param[in] msgid Id of the message received.
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */

__STATIC int udsc_enable_req_handler(ke_msg_id_t const msgid,
                                     struct udsc_enable_req *param,
                                     ke_task_id_t const dest_id,
                                     ke_task_id_t const src_id)
{
    /// Status
    uint8_t status     = GAP_ERR_NO_ERROR;
    /// Get connection index
    uint8_t conidx = KE_IDX_GET(dest_id);

    uint8_t state = ke_state_get(dest_id);
    /// User Data Service Collector Role Task Environment
    struct udsc_env_tag *udsc_env = PRF_ENV_GET(UDSC, udsc);

    ASSERT_INFO(udsc_env != NULL, dest_id, src_id);
    if ((state == UDSC_IDLE) && (udsc_env->env[conidx] == NULL))
    {
        /// allocate environment variable for task instance
        udsc_env->env[conidx] = (struct udsc_cnx_env *) ke_malloc(sizeof(struct udsc_cnx_env), KE_MEM_ATT_DB);
        memset(udsc_env->env[conidx], 0, sizeof(struct udsc_cnx_env));

        /// Start discovering
        if (param->con_type == PRF_CON_DISCOVERY)
        {
            /// Start discovery with User Data Service
            prf_disc_svc_send(&(udsc_env->prf_env), conidx, ATT_SVC_USER_DATA);

            udsc_env->env[conidx]->last_uuid_req = ATT_SVC_USER_DATA;
            udsc_env->env[conidx]->last_svc_req  = ATT_SVC_USER_DATA;

            /// Go to DISCOVERING SERVICE state
            ke_state_set(dest_id, UDSC_DISCOVERING_SVC);
        }
        //normal connection, get saved att details
        else
        {
            udsc_env->env[conidx]->uds = param->uds;

            //send APP confirmation that can start normal connection to TH
            udsc_enable_rsp_send(udsc_env, conidx, GAP_ERR_NO_ERROR);
        }
    }
    else if (state != UDSC_FREE)
    {
        // The message will be forwarded towards the good task instance
        status = PRF_ERR_REQ_DISALLOWED;
    }

    if (status != GAP_ERR_NO_ERROR)
    {
        // The request is disallowed (profile already enabled for this connection, or not enough memory, ...)
        udsc_enable_rsp_send(udsc_env, conidx, status);
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref UDSC_RD_CHAR_REQ  message from the application.
 * @brief To read the Feature Characteristic in the peer server.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC int udsc_rd_char_req_handler(ke_msg_id_t const msgid,
                                      struct udsc_rd_char_req *param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id)
{
    uint8_t state = ke_state_get(dest_id);
    uint8_t status = PRF_ERR_REQ_DISALLOWED;
    // Message status
    uint8_t msg_status = KE_MSG_CONSUMED;
    // Get the address of the environment
    struct udsc_env_tag *udsc_env = PRF_ENV_GET(UDSC, udsc);
    // Get connection index
    uint8_t conidx = KE_IDX_GET(dest_id);

    if (state == UDSC_IDLE)
    {
        ASSERT_INFO(udsc_env != NULL, dest_id, src_id);
        // environment variable not ready
        if (udsc_env->env[conidx] == NULL)
        {
            status = PRF_APP_ERROR;
        }
        else
        {
            // Attribute Handle
            uint16_t search_hdl = ATT_INVALID_SEARCH_HANDLE;
            // Service
            struct prf_svc *svc = &udsc_env->env[conidx]->uds.svc;

            if (param->char_idx < UDSC_CHAR_UDS_MAX)
            {
                search_hdl = udsc_env->env[conidx]->uds.chars[param->char_idx].val_hdl;
            }
            // Check if handle is viable
            if ((search_hdl != ATT_INVALID_SEARCH_HANDLE) && (svc != NULL))
            {
                // Force the operation value
                udsc_env->env[conidx]->op_pending = UDSC_READ_CHAR_OP_CODE;
                // keep the characteristic in operation
                udsc_env->env[conidx]->last_char_code = param->char_idx;
                // Send read request
                prf_read_char_send(&(udsc_env->prf_env), conidx, svc->shdl, svc->ehdl, search_hdl);
                // Go to the Busy state
                ke_state_set(dest_id, UDSC_BUSY);

                status = GAP_ERR_NO_ERROR;
            }
            else
            {
                status =  PRF_ERR_INEXISTENT_HDL;
            }
        }
    }
    else if (state == UDSC_FREE)
    {
        status = GAP_ERR_DISCONNECTED;
    }
    else
    {
        // Another procedure is pending, keep the command for later
        msg_status = KE_MSG_SAVED;
        status = GAP_ERR_NO_ERROR;
    }

    if (status != GAP_ERR_NO_ERROR)
    {
        struct udsc_rd_char_rsp *rsp = KE_MSG_ALLOC(UDSC_RD_CHAR_RSP,
                                       prf_dst_task_get(&(udsc_env->prf_env), conidx),
                                       dest_id,
                                       udsc_rd_char_rsp);

        rsp->status = status;
        rsp->char_idx = param->char_idx;

        ke_msg_send(rsp);
    }

    return (int)msg_status;
}


/**
 ****************************************************************************************
 * @brief Handles reception of the @ref UDSC_RD_CHAR_CCC_REQ  message from the application.
 * @brief To read the CCC value of the Measurement characteristic in the peer server.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC int udsc_rd_char_ccc_req_handler(ke_msg_id_t const msgid,
        struct udsc_rd_char_ccc_req *param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
    uint8_t state = ke_state_get(dest_id);
    uint8_t status = PRF_ERR_REQ_DISALLOWED;
    // Message status
    uint8_t msg_status = KE_MSG_CONSUMED;
    // Get the address of the environment
    struct udsc_env_tag *udsc_env = PRF_ENV_GET(UDSC, udsc);
    // Get connection index
    uint8_t conidx = KE_IDX_GET(dest_id);

    if (state == UDSC_IDLE)
    {
        ASSERT_INFO(udsc_env != NULL, dest_id, src_id);
        // environment variable not ready
        if (udsc_env->env[conidx] == NULL)
        {
            status = PRF_APP_ERROR;
        }
        else
        {
            // Attribute Handle
            uint16_t search_hdl = ATT_INVALID_SEARCH_HANDLE;
            // Service
            struct prf_svc *svc = &udsc_env->env[conidx]->uds.svc;

            if (param->char_idx == UDSC_CHAR_UDS_DB_CHG_INC)
            {
                search_hdl = udsc_env->env[conidx]->uds.descs[UDSC_DESC_UDS_DB_CHG_INC_CCC].desc_hdl;
            }
            else if (param->char_idx == UDSC_CHAR_UDS_USER_CTRL_PT)
            {
                search_hdl = udsc_env->env[conidx]->uds.descs[UDSC_DESC_UDS_USER_CTRL_PT_CCC].desc_hdl;
            }

            // Check if handle is viable
            if ((search_hdl != ATT_INVALID_SEARCH_HANDLE) && (svc != NULL))
            {
                // Force the operation value
                udsc_env->env[conidx]->op_pending = UDSC_READ_CCC_OP_CODE;
                // keep the characteristic in operation
                udsc_env->env[conidx]->last_char_code = param->char_idx;
                // Send read request
                prf_read_char_send(&(udsc_env->prf_env), conidx, svc->shdl, svc->ehdl, search_hdl);
                // Go to the Busy state
                ke_state_set(dest_id, UDSC_BUSY);

                status = GAP_ERR_NO_ERROR;
            }
            else
            {
                status =  PRF_ERR_INEXISTENT_HDL;
            }
        }
    }
    else if (state == UDSC_FREE)
    {
        status = GAP_ERR_DISCONNECTED;
    }
    else
    {
        // Another procedure is pending, keep the command for later
        msg_status = KE_MSG_SAVED;
        status = GAP_ERR_NO_ERROR;
    }

    if (status != GAP_ERR_NO_ERROR)
    {
        struct udsc_rd_char_ccc_rsp *rsp = KE_MSG_ALLOC(UDSC_RD_CHAR_CCC_RSP,
                                           prf_dst_task_get(&(udsc_env->prf_env), conidx),
                                           dest_id,
                                           udsc_rd_char_ccc_rsp);

        rsp->status = status;
        rsp->char_idx = param->char_idx;

        ke_msg_send(rsp);
    }

    return (int)msg_status;
}


/**
 ****************************************************************************************
 * @brief Handles reception of the @ref UDSC_WR_CHAR_CCC_CMD message.
 * Allows the application to write new CCC values to a Characteristic in the peer server
 * @param[in] msgid Id of the message received.
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC int udsc_wr_char_ccc_cmd_handler(ke_msg_id_t const msgid,
        struct udsc_wr_char_ccc_cmd *param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
    uint8_t state = ke_state_get(dest_id);
    uint8_t conidx = KE_IDX_GET(dest_id);
    // Status
    uint8_t status     = GAP_ERR_NO_ERROR;
    // Message status
    uint8_t msg_status = KE_MSG_CONSUMED;

    // Get the address of the environment
    struct udsc_env_tag *udsc_env = PRF_ENV_GET(UDSC, udsc);
    // Get connection index

    if (state == UDSC_IDLE)
    {

        ASSERT_INFO(udsc_env != NULL, dest_id, src_id);
        // environment variable not ready
        if (udsc_env->env[conidx] == NULL)
        {
            status = PRF_APP_ERROR;
        }
        else
        {
            // Attribute Handle
            uint16_t handle = ATT_INVALID_SEARCH_HANDLE;
            // Service
            struct prf_svc *svc = &udsc_env->env[conidx]->uds.svc;

            if (param->char_idx == UDSC_CHAR_UDS_DB_CHG_INC)
            {
                handle = udsc_env->env[conidx]->uds.descs[UDSC_DESC_UDS_DB_CHG_INC_CCC].desc_hdl;
            }
            else if (param->char_idx == UDSC_CHAR_UDS_USER_CTRL_PT)
            {
                handle = udsc_env->env[conidx]->uds.descs[UDSC_DESC_UDS_USER_CTRL_PT_CCC].desc_hdl;
            }

            // Check if handle is viable
            if ((handle != ATT_INVALID_SEARCH_HANDLE) && (svc != NULL))
            {
                // Force the operation value
                udsc_env->env[conidx]->op_pending = UDSC_WRITE_CCC_OP_CODE;
                // keep the characteristic in operation
                udsc_env->env[conidx]->last_char_code = param->char_idx;

                // Send the write request
                prf_gatt_write_ntf_ind(&(udsc_env->prf_env), conidx, handle, param->ccc);

                // Go to the Busy state
                ke_state_set(dest_id, UDSC_BUSY);

                status = GAP_ERR_NO_ERROR;
            }
            else
            {
                status = PRF_ERR_INEXISTENT_HDL;
            }
        }

        if (status != GAP_ERR_NO_ERROR)
        {
            udsc_send_cmp_evt(udsc_env, conidx, UDSC_WRITE_CCC_OP_CODE, status);
        }
    }
    else if (state == UDSC_FREE)
    {
        status = GAP_ERR_DISCONNECTED;
    }
    else
    {
        // Another procedure is pending, keep the command for later
        msg_status = KE_MSG_SAVED;
        status = GAP_ERR_NO_ERROR;
    }

    return (int)msg_status;
}


/**
 ****************************************************************************************
 * @brief Handles reception of the @ref UDSC_WR_CHAR_UTF8_CMD message.
 * Allows the application to write new CCC values to a Characteristic in the peer server
 * @param[in] msgid Id of the message received.
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC int udsc_wr_char_utf8_cmd_handler(ke_msg_id_t const msgid,
        struct udsc_wr_char_utf8_cmd *param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
    uint8_t state = ke_state_get(dest_id);
    uint8_t conidx = KE_IDX_GET(dest_id);
    // Status
    uint8_t status     = GAP_ERR_NO_ERROR;
    // Message status
    uint8_t msg_status = KE_MSG_CONSUMED;

    // Get the address of the environment
    struct udsc_env_tag *udsc_env = PRF_ENV_GET(UDSC, udsc);
    // Get connection index

    if (state == UDSC_IDLE)
    {

        ASSERT_INFO(udsc_env != NULL, dest_id, src_id);
        // environment variable not ready
        if (udsc_env->env[conidx] == NULL)
        {
            status = PRF_APP_ERROR;
        }
        else
        {
            // Attribute Handle
            uint16_t handle = ATT_INVALID_SEARCH_HANDLE;
            // Service
            struct prf_svc *svc = &udsc_env->env[conidx]->uds.svc;
            // Length
            uint8_t length;
            uint8_t *data_ptr;

            length = param->utf_name.length;
            data_ptr = &param->utf_name.name[0];

#ifndef USE_EXTRA_CHARS
            if (param->char_idx <= UDSC_CHAR_UDS_LANGUAGE)
#endif
            {
                handle = udsc_env->env[conidx]->uds.chars[param->char_idx].val_hdl;
            }

            // Check if handle is viable
            if ((handle != ATT_INVALID_SEARCH_HANDLE) && (svc != NULL))
            {
                // Force the operation value
                udsc_env->env[conidx]->op_pending = UDSC_WRITE_CHAR_OP_CODE;
                // keep the characteristic in operation
                udsc_env->env[conidx]->last_char_code = param->char_idx;

                // Send the write request
                prf_gatt_write(&(udsc_env->prf_env), conidx, handle, data_ptr, length, GATTC_WRITE);

                // Go to the Busy state
                ke_state_set(dest_id, UDSC_BUSY);

                status = GAP_ERR_NO_ERROR;
            }
            else
            {
                status = PRF_ERR_INEXISTENT_HDL;
            }
        }

        if (status != GAP_ERR_NO_ERROR)
        {
            udsc_send_cmp_evt(udsc_env, conidx, UDSC_WRITE_CHAR_OP_CODE, status);
        }
    }
    else if (state == UDSC_FREE)
    {
        status = GAP_ERR_DISCONNECTED;
    }
    else
    {
        // Another procedure is pending, keep the command for later
        msg_status = KE_MSG_SAVED;
        status = GAP_ERR_NO_ERROR;
    }

    return (int)msg_status;
}


/**
 ****************************************************************************************
 * @brief Handles reception of the @ref UDSC_WR_CHAR_CMD message.
 * Allows the application to write new CCC values to a Characteristic in the peer server
 * @param[in] msgid Id of the message received.
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC int udsc_wr_char_cmd_handler(ke_msg_id_t const msgid,
                                      struct udsc_wr_char_cmd *param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id)
{
    uint8_t state = ke_state_get(dest_id);
    uint8_t conidx = KE_IDX_GET(dest_id);
    // Status
    uint8_t status     = GAP_ERR_NO_ERROR;
    // Message status
    uint8_t msg_status = KE_MSG_CONSUMED;

    // Get the address of the environment
    struct udsc_env_tag *udsc_env = PRF_ENV_GET(UDSC, udsc);
    // Get connection index

    if (state == UDSC_IDLE)
    {

        ASSERT_INFO(udsc_env != NULL, dest_id, src_id);
        // environment variable not ready
        if (udsc_env->env[conidx] == NULL)
        {
            status = PRF_APP_ERROR;
        }
        else
        {
            // Attribute Handle
            uint16_t handle = ATT_INVALID_SEARCH_HANDLE;
            // Service
            struct prf_svc *svc = &udsc_env->env[conidx]->uds.svc;
            // Length
            uint8_t length;
            uint8_t data[sizeof(union char_val)];
            uint8_t *data_ptr = &data[0];
            uint8_t char_idx = param->char_idx;
            length = 0;

            if (((char_idx >= UDSC_CHAR_UDS_DATE_OF_BIRTH) && (char_idx <= UDSC_CHAR_UDS_THREE_ZONE_HEART_RATE_LIMITS)) ||
                    (char_idx == UDSC_CHAR_UDS_DB_CHG_INC))
            {
                handle = udsc_env->env[conidx]->uds.chars[char_idx].val_hdl;
            }

            // Check if handle is viable
            if ((handle != ATT_INVALID_SEARCH_HANDLE) && (svc != NULL))
            {
                // Force the operation value
                udsc_env->env[conidx]->op_pending = UDSC_WRITE_CHAR_OP_CODE;
                // keep the characteristic in operation
                udsc_env->env[conidx]->last_char_code = char_idx;

                if (char_idx == UDSC_CHAR_UDS_DB_CHG_INC)
                {
                    // DB CHG Inc
                    co_write32p(&data[0], param->value.db_chg_inc);
                    length = sizeof(uint32_t);
                }
                else if (char_idx == UDSC_CHAR_UDS_FIVE_ZONE_HEART_RATE_LIMITS)
                {
                    // Set
                    data[0] = param->value.set[0];
                    data[1] = param->value.set[1];
                    data[2] = param->value.set[2];
                    data[3] = param->value.set[3];
                    length = 4;
                }
                else if (char_idx == UDSC_CHAR_UDS_THREE_ZONE_HEART_RATE_LIMITS)
                {
                    // Set
                    data[0] = param->value.set[0];
                    data[1] = param->value.set[1];
                    length = 2;
                }
                else if ((char_idx >= UDSC_CHAR_UDS_DATE_OF_BIRTH) && (char_idx <= UDSC_CHAR_UDS_DATE_OF_THRESHOLD_ASSESSMENT))
                {
                    // Date
                    /// Characteristic Date
                    co_write16p(&data[0], param->value.date.year);
                    data[2] = param->value.date.month;
                    data[3] = param->value.date.day;
                    length = 4;
                }
                else if ((char_idx >= UDSC_CHAR_UDS_WEIGHT) && (char_idx <= UDSC_CHAR_UDS_HIP_CIRCUMFERENCE))
                {
                    // uint16
                    co_write16p(&data[0], param->value.weight);
                    length = 2;
                }
                else if (((char_idx >= UDSC_CHAR_UDS_AGE) && (char_idx <= UDSC_CHAR_UDS_TWO_ZONE_HEART_RATE_LIMIT)) || (char_idx == UDSC_CHAR_UDS_USER_INDEX))
                {
                    // uint8
                    data[0] = param->value.age;
                    length = 1;
                }
                else
                {
                    // other???
                    co_write32p(&data[0], param->value.db_chg_inc);
                    length = 4;
                }

                // Send the write request
                prf_gatt_write(&(udsc_env->prf_env), conidx, handle, data_ptr, length, GATTC_WRITE);

                // Go to the Busy state
                ke_state_set(dest_id, UDSC_BUSY);

                status = GAP_ERR_NO_ERROR;
            }
            else
            {
                status = PRF_ERR_INEXISTENT_HDL;
            }
        }

        if (status != GAP_ERR_NO_ERROR)
        {
            udsc_send_cmp_evt(udsc_env, conidx, UDSC_WRITE_CHAR_OP_CODE, status);
        }
    }
    else if (state == UDSC_FREE)
    {
        status = GAP_ERR_DISCONNECTED;
    }
    else
    {
        // Another procedure is pending, keep the command for later
        msg_status = KE_MSG_SAVED;
        status = GAP_ERR_NO_ERROR;
    }

    return (int)msg_status;
}



/**
 ****************************************************************************************
 * @brief Handles reception of the @ref UDSC_WR_USER_CTRL_PT_CMD message.
 * Allows the application to write new CCC values to a Characteristic in the peer server
 * @param[in] msgid Id of the message received.
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC int udsc_wr_user_ctrl_pt_cmd_handler(ke_msg_id_t const msgid,
        struct udsc_wr_user_ctrl_pt_cmd *param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
    uint8_t state = ke_state_get(dest_id);
    uint8_t conidx = KE_IDX_GET(dest_id);
    // Status
    uint8_t status     = GAP_ERR_NO_ERROR;
    // Message status
    uint8_t msg_status = KE_MSG_CONSUMED;

    // Get the address of the environment
    struct udsc_env_tag *udsc_env = PRF_ENV_GET(UDSC, udsc);
    // Get connection index

    if (state == UDSC_IDLE)
    {

        ASSERT_INFO(udsc_env != NULL, dest_id, src_id);
        // environment variable not ready
        if (udsc_env->env[conidx] == NULL)
        {
            status = PRF_APP_ERROR;
        }
        else
        {
            // Attribute Handle
            uint16_t handle = ATT_INVALID_SEARCH_HANDLE;
            // Service
            struct prf_svc *svc = &udsc_env->env[conidx]->uds.svc;
            // Length
            uint8_t length;
            uint8_t data[UDS_USER_CTRL_PT_MAX_LEN];
            uint8_t *data_ptr = &data[0];

            handle = udsc_env->env[conidx]->uds.chars[UDSC_CHAR_UDS_USER_CTRL_PT].val_hdl;

            // Check if handle is viable
            if ((handle != ATT_INVALID_SEARCH_HANDLE) && (svc != NULL))
            {
                // Force the operation value
                udsc_env->env[conidx]->op_pending = UDSC_WRITE_CHAR_OP_CODE;
                // keep the characteristic in operation
                udsc_env->env[conidx]->last_char_code = UDSC_CHAR_UDS_USER_CTRL_PT;

                data[0] = param->op_code;
                if (param->op_code == UDS_OP_CODE_REGISTER_NEW_USER)
                {
                    length = 3; // op-code and consent code
                    co_write16p(&data[1], param->consent);
                }
                else if (param->op_code == UDS_OP_CODE_CONSENT)
                {
                    length = 4; // op-code, user_id and consent code
                    data[1] = param->user_id;
                    co_write16p(&data[2], param->consent);
                }
                else if (param->op_code == UDS_OP_CODE_DELETE_USER_DATA)
                {
                    length = 1; // op-code
                }
                else
                {
                    // reserved commands - for the future extension

                    if (param->length >= UDS_USER_CTRL_PT_MAX_LEN - 1) //sizeof(op-code)
                    {
                        param->length = 0;
                    }
                    length = 1 + param->length;
                    if (param->length)
                    {
                        memcpy(&data[1], &param->parameter[0], param->length);
                    }
                }

                // Send the write request
                prf_gatt_write(&(udsc_env->prf_env), conidx, handle, data_ptr, length, GATTC_WRITE);

                // Go to the Busy state
                ke_state_set(dest_id, UDSC_BUSY);

                status = GAP_ERR_NO_ERROR;
            }
            else
            {
                status = PRF_ERR_INEXISTENT_HDL;
            }
        }

        if (status != GAP_ERR_NO_ERROR)
        {
            udsc_send_cmp_evt(udsc_env, conidx, UDSC_WRITE_CHAR_OP_CODE, status);
        }
    }
    else if (state == UDSC_FREE)
    {
        status = GAP_ERR_DISCONNECTED;
    }
    else
    {
        // Another procedure is pending, keep the command for later
        msg_status = KE_MSG_SAVED;
        status = GAP_ERR_NO_ERROR;
    }

    return (int)msg_status;
}


/**
 ****************************************************************************************
 * @brief Handles reception of the @ref GATTC_CMP_EVT message.
 * @param[in] msgid Id of the message received.
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC int gattc_cmp_evt_handler(ke_msg_id_t const msgid,
                                   struct gattc_cmp_evt const *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id)
{
    // Get the address of the environment
    struct udsc_env_tag *udsc_env = PRF_ENV_GET(UDSC, udsc);
    // Status
    uint8_t status;

    if (udsc_env != NULL)
    {
        uint8_t conidx = KE_IDX_GET(dest_id);
        uint8_t state = ke_state_get(dest_id);

        if (state == UDSC_DISCOVERING_SVC)
        {
            if ((param->status == GAP_ERR_NO_ERROR) || (param->status == ATT_ERR_ATTRIBUTE_NOT_FOUND))
            {
                // Check service (if found)
                if (udsc_env->env[conidx]->nb_svc)
                {
                    // Check service (mandatory)
                    status = prf_check_svc_char_validity(UDSC_CHAR_UDS_MAX,
                                                         udsc_env->env[conidx]->uds.chars,
                                                         udsc_uds_char);

                    // Check Descriptors (mandatory)
                    if (status == GAP_ERR_NO_ERROR)
                    {
                        status = prf_check_svc_char_desc_validity(UDSC_DESC_UDS_MAX,
                                 udsc_env->env[conidx]->uds.descs,
                                 udsc_uds_char_desc,
                                 udsc_env->env[conidx]->uds.chars);
                    }
                }
            }

            // Raise an ESP_ENABLE_REQ complete event.
            ke_state_set(dest_id, UDSC_IDLE);

            udsc_enable_rsp_send(udsc_env, conidx, param->status);
        }
        else if (state == UDSC_BUSY)
        {
            uint8_t op_pending = udsc_env->env[conidx]->op_pending;
            // retrieve the characteristic in operation
            uint8_t char_idx = udsc_env->env[conidx]->last_char_code;

            status = param->status;
            switch (param->operation)
            {
            case GATTC_READ:
            {
                // Send the complete event status
                if (status != GAP_ERR_NO_ERROR)
                {
                    switch (op_pending)
                    {
                    case UDSC_READ_CHAR_OP_CODE :
                    {
                        struct udsc_rd_char_rsp *rsp = KE_MSG_ALLOC(UDSC_RD_CHAR_RSP,
                                                       prf_dst_task_get(&(udsc_env->prf_env), conidx),
                                                       dest_id,
                                                       udsc_rd_char_rsp);

                        rsp->status = status;
                        rsp->char_idx = char_idx;

                        ke_msg_send(rsp);
                    }
                    break;

                    case UDSC_READ_CCC_OP_CODE :
                    {
                        struct udsc_rd_char_ccc_rsp *rsp = KE_MSG_ALLOC(UDSC_RD_CHAR_CCC_RSP,
                                                           prf_dst_task_get(&(udsc_env->prf_env), conidx),
                                                           dest_id,
                                                           udsc_rd_char_ccc_rsp);

                        rsp->status = status;
                        rsp->char_idx = char_idx;

                        ke_msg_send(rsp);
                    }
                    break;
                    default:
                        break;
                    }
                    udsc_env->env[conidx]->op_pending = 0;
                    // ke_msg_free(ke_param2msg(udsc_env->env[conidx]->operation));
                    // udsc_env->env[conidx]->operation = NULL;
                    ke_state_set(dest_id, UDSC_IDLE);
                }
            }
            break;

            case GATTC_WRITE:
            case GATTC_WRITE_NO_RESPONSE:
            {
                // Send the complete event status
                udsc_send_cmp_evt(udsc_env, conidx, op_pending, param->status);
                ke_state_set(dest_id, UDSC_IDLE);

            }
            break;

            case GATTC_REGISTER:
            {
                if (status != GAP_ERR_NO_ERROR)
                {
                    // Send the complete event error status
                    udsc_send_cmp_evt(udsc_env, conidx, GATTC_REGISTER, param->status);
                }
                // Go to connected state
                ke_state_set(dest_id, UDSC_IDLE);
            }
            break;

            case GATTC_UNREGISTER:
            {
                // Do nothing
            } break;

            default:
            {
                ASSERT_ERR(0);
            }
            break;
            }
        }
    }
    // else ignore the message

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref GATTC_READ_IND message.
 * Generic event received after every simple read command sent to peer server.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC int gattc_read_ind_handler(ke_msg_id_t const msgid,
                                    struct gattc_read_ind const *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id)
{
    // Get the address of the environment
    struct udsc_env_tag *udsc_env = PRF_ENV_GET(UDSC, udsc);

    if (ke_state_get(dest_id) == UDSC_BUSY)
    {
        uint8_t conidx = KE_IDX_GET(dest_id);
        uint8_t op_pending;
        uint8_t status = GAP_ERR_NO_ERROR;

        ASSERT_INFO(udsc_env != NULL, dest_id, src_id);
        ASSERT_INFO(udsc_env->env[conidx] != NULL, dest_id, src_id);

        // Find the op_pending -  and check it is a valid read code.
        op_pending = udsc_env->env[conidx]->op_pending;
        // retrieve the characteristic in operation
        uint8_t char_idx = udsc_env->env[conidx]->last_char_code;

        if ((op_pending >= UDSC_READ_CHAR_OP_CODE) && (op_pending <= UDSC_READ_CCC_OP_CODE))
        {
            switch (op_pending)
            {
            case UDSC_READ_CHAR_OP_CODE:
            {
                if (char_idx <= UDSC_CHAR_UDS_LANGUAGE)
                {
                    //utf8
                    uint16_t length = param->length;
                    struct udsc_rd_char_utf8_rsp *rsp = KE_MSG_ALLOC_DYN(UDSC_RD_CHAR_UTF8_RSP,
                                                        prf_dst_task_get(&(udsc_env->prf_env), conidx),
                                                        dest_id,
                                                        udsc_rd_char_utf8_rsp, length);
                    rsp->status = status;
                    rsp->char_idx = char_idx;
                    rsp->utf_name.length = param->length;
                    for (int i = 0; i < rsp->utf_name.length; i++)
                    {
                        rsp->utf_name.name[i] = param->value[i];
                    }
                    ke_msg_send(rsp);
                }
                else
                {
                    struct udsc_rd_char_rsp *rsp = KE_MSG_ALLOC(UDSC_RD_CHAR_RSP,
                                                   prf_dst_task_get(&(udsc_env->prf_env), conidx),
                                                   dest_id,
                                                   udsc_rd_char_rsp);

                    rsp->status = status;
                    rsp->char_idx = char_idx;

                    if (char_idx == UDSC_CHAR_UDS_DB_CHG_INC)
                    {
                        // DB CHG Inc
                        rsp->value.db_chg_inc = co_read32p(&param->value[0]);
                    }
                    else if (char_idx == UDSC_CHAR_UDS_FIVE_ZONE_HEART_RATE_LIMITS)
                    {
                        // Set
                        rsp->value.set[0] = param->value[0];
                        rsp->value.set[1] = param->value[1];
                        rsp->value.set[2] = param->value[2];
                        rsp->value.set[3] = param->value[3];
                    }
                    else if (char_idx == UDSC_CHAR_UDS_THREE_ZONE_HEART_RATE_LIMITS)
                    {
                        // Set
                        rsp->value.set[0] = param->value[0];
                        rsp->value.set[1] = param->value[1];
                    }
                    else if ((char_idx >= UDSC_CHAR_UDS_DATE_OF_BIRTH) && (char_idx <= UDSC_CHAR_UDS_DATE_OF_THRESHOLD_ASSESSMENT))
                    {
                        // Date
                        /// Characteristic Date
                        rsp->value.date.year  = co_read16p(&param->value[0]);
                        rsp->value.date.month = param->value[2];
                        rsp->value.date.day   = param->value[3];
                    }
                    else if ((char_idx >= UDSC_CHAR_UDS_WEIGHT) && (char_idx <= UDSC_CHAR_UDS_HIP_CIRCUMFERENCE))
                    {
                        // uint16
                        rsp->value.weight = co_read16p(&param->value[0]);
                    }
                    else if (((char_idx >= UDSC_CHAR_UDS_AGE) && (char_idx <= UDSC_CHAR_UDS_TWO_ZONE_HEART_RATE_LIMIT)) || (char_idx == UDSC_CHAR_UDS_USER_INDEX))
                    {
                        // uint8
                        rsp->value.age = param->value[0];
                    }
                    else
                    {
                        // other???
                        rsp->value.db_chg_inc = co_read32p(&param->value[0]);
                    }

                    ke_msg_send(rsp);
                }
            }
            break;

            case UDSC_READ_CCC_OP_CODE :
            {
                struct udsc_rd_char_ccc_rsp *rsp = KE_MSG_ALLOC(UDSC_RD_CHAR_CCC_RSP,
                                                   prf_dst_task_get(&(udsc_env->prf_env), conidx),
                                                   dest_id,
                                                   udsc_rd_char_ccc_rsp);

                rsp->status = status;
                rsp->char_idx = char_idx;
                rsp->ccc     = co_read16p(&param->value[0]);

                ke_msg_send(rsp);
            }
            break;

            default:
            {
                ASSERT_ERR(0);
            }
            break;
            }
            ke_state_set(dest_id, UDSC_IDLE);
        }


    }
    // else drop the message

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref GATTC_EVENT_IND message.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC int gattc_event_ind_handler(ke_msg_id_t const msgid,
                                     struct gattc_event_ind const *param,
                                     ke_task_id_t const dest_id,
                                     ke_task_id_t const src_id)
{
    uint8_t conidx = KE_IDX_GET(dest_id);
    // Get the address of the environment
    struct udsc_env_tag *udsc_env = PRF_ENV_GET(UDSC, udsc);

    if (udsc_env != NULL)
    {
        switch (param->type)
        {
        case (GATTC_NOTIFY):
        {
            if (param->handle == udsc_env->env[conidx]->uds.chars[UDSC_CHAR_UDS_DB_CHG_INC].val_hdl)
            {
                struct udsc_db_chg_inc_ind *ind = KE_MSG_ALLOC(UDSC_DB_CHG_INC_IND,
                                                  prf_dst_task_get(&udsc_env->prf_env, conidx),
                                                  prf_src_task_get(&udsc_env->prf_env, conidx),
                                                  udsc_db_chg_inc_ind);
                ind->value = co_read32p(&param->value[0]);
                ke_msg_send(ind);
            }
            else
            {
                /// Ignore the Notify as Att-Handle is invalid
            }
        } break;

        case (GATTC_INDICATE):
        {
            if (param->handle == udsc_env->env[conidx]->uds.chars[UDSC_CHAR_UDS_USER_CTRL_PT].val_hdl)
            {
                struct udsc_user_ctrl_pt_ind *ind;
                uint16_t length = 0;
                if (param->length > 3)
                {
                    length = param->length - 3;
                }

                if (length >= UDS_USER_CTRL_PT_MAX_LEN - 3) //sizeof(op-code+length)
                {
                    length = UDS_USER_CTRL_PT_MAX_LEN - 3;
                }

                ind = KE_MSG_ALLOC_DYN(UDSC_USER_CTRL_PT_IND,
                                       prf_dst_task_get(&udsc_env->prf_env, conidx),
                                       prf_src_task_get(&udsc_env->prf_env, conidx),
                                       udsc_user_ctrl_pt_ind, length);
                // decode message
                ind->resp_code = param->value[0]; // response op-code =0x20
                ind->req_op_code = param->value[1]; // requested op-code
                ind->resp_value = param->value[2];
                ind->user_id = UDS_USER_ID_UNKNOWN_USER;
                if ((ind->req_op_code == UDS_OP_CODE_REGISTER_NEW_USER) && (ind->resp_value == UDS_OP_RESPONSE_SUCCESS))
                {
                    ind->user_id = param->value[3];
                    if (length)
                    {
                        length--;
                    }
                }

                if (length)
                {
                    memcpy(&ind->parameter[0], &param->value[3], length);
                }

                ind->length = length;

                ke_msg_send(ind);
            }

            // confirm that indication has been correctly received
            struct gattc_event_cfm *cfm = KE_MSG_ALLOC(GATTC_EVENT_CFM, src_id, dest_id, gattc_event_cfm);
            cfm->handle = param->handle;
            ke_msg_send(cfm);
            // else drop the message
        }
        break;

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
KE_MSG_HANDLER_TAB(udsc)
{
    {UDSC_ENABLE_REQ, (ke_msg_func_t)udsc_enable_req_handler},

    {GATTC_SDP_SVC_IND, (ke_msg_func_t)gattc_sdp_svc_ind_handler},
    {GATTC_CMP_EVT, (ke_msg_func_t)gattc_cmp_evt_handler},

    {GATTC_READ_IND, (ke_msg_func_t)gattc_read_ind_handler},
    {GATTC_EVENT_IND, (ke_msg_func_t)gattc_event_ind_handler},
    {GATTC_EVENT_REQ_IND, (ke_msg_func_t)gattc_event_ind_handler},

    ///*** UDS CHARACTERISTIC READ REQUESTS
    {UDSC_RD_CHAR_REQ, (ke_msg_func_t)udsc_rd_char_req_handler},
    /// Read the CCC of a DB_CHANGE & USER_CTRL_PT characteristic
    {UDSC_RD_CHAR_CCC_REQ, (ke_msg_func_t)udsc_rd_char_ccc_req_handler},
    ///***  CHARACTERISTIC WRITE COMMAND
    /// Write the CCC of a measurement sensor characteristic
    {UDSC_WR_CHAR_CMD, (ke_msg_func_t)udsc_wr_char_cmd_handler},
    /// Write the CCC of a measurement sensor characteristic
    {UDSC_WR_CHAR_UTF8_CMD, (ke_msg_func_t)udsc_wr_char_utf8_cmd_handler},

    ///***  CHARACTERISTIC/DESCRIPTOR WRITE REQUESTS
    /// Write the CCC of a measurement sensor characteristic
    {UDSC_WR_USER_CTRL_PT_CMD, (ke_msg_func_t)udsc_wr_user_ctrl_pt_cmd_handler},

    /// Write the CCC of a DB_CHANGE & USER_CTRL_PT characteristic
    {UDSC_WR_CHAR_CCC_CMD, (ke_msg_func_t)udsc_wr_char_ccc_cmd_handler},
};

void udsc_task_init(struct ke_task_desc *task_desc)
{
    // Get the address of the environment
    struct udsc_env_tag *udsc_env = PRF_ENV_GET(UDSC, udsc);

    task_desc->msg_handler_tab = udsc_msg_handler_tab;
    task_desc->msg_cnt         = ARRAY_LEN(udsc_msg_handler_tab);
    task_desc->state           = udsc_env->state;
    task_desc->idx_max         = UDSC_IDX_MAX;
}


#endif //(BLE_UDS_CLIENT)

/// @} UDSCTASK
