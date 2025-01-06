/**
 ****************************************************************************************
 *
 * @file scpps_task.c
 *
 * @brief Scan Parameters Profile Server Task Implementation.
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 *
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @addtogroup SCPPSTASK
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"

#if (BLE_SP_SERVER)
#include "gap.h"
#include "gattc_task.h"
#include "attm.h"

#include "scpps.h"
#include "scpps_task.h"
#include "scpp_common.h"

#include "prf_utils.h"

#include "ke_mem.h"
#include "co_utils.h"

/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref SCPPS_ENABLE_REQ message.
 * The handler enables the Scan Parameters Profile Server Role.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC int scpps_enable_req_handler(ke_msg_id_t const msgid,
                                      struct scpps_enable_req const *param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id)
{
    int msg_status = KE_MSG_SAVED;
    uint8_t state = ke_state_get(dest_id);

    // check state of the task
    if (state == SCPPS_IDLE)
    {
        struct scpps_env_tag *scpps_env = PRF_ENV_GET(SCPPS, scpps);

        // an error occurs, trigg it.
        struct scpps_enable_rsp *rsp = KE_MSG_ALLOC(SCPPS_ENABLE_RSP, src_id,
                                       dest_id, scpps_enable_rsp);

        msg_status  = KE_MSG_CONSUMED;
        rsp->conidx = param->conidx;

        if ((param->conidx > BLE_CONNECTION_MAX)
                || (gapc_get_conhdl(param->conidx) == GAP_INVALID_CONHDL))
        {
            rsp->status = (param->conidx > BLE_CONNECTION_MAX) ? GAP_ERR_INVALID_PARAM : PRF_ERR_REQ_DISALLOWED;
        }
        else
        {
            // restore Bond Data
            scpps_env->ntf_cfg[param->conidx] = param->ntf_cfg;
            rsp->status = GAP_ERR_NO_ERROR;
        }

        ke_msg_send(rsp);
    }

    return msg_status;
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref SCPPS_SCAN_REFRESH_SEND_REQ message.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC int scpps_scan_refresh_send_req_handler(ke_msg_id_t const msgid,
        struct scpps_scan_refresh_send_req const *param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{

    int msg_status = KE_MSG_SAVED;
    uint8_t state = ke_state_get(dest_id);

    // check state of the task
    if (state == SCPPS_IDLE)
    {
        uint8_t status = GAP_ERR_NO_ERROR;
        struct scpps_env_tag *scpps_env = PRF_ENV_GET(SCPPS, scpps);

        msg_status = KE_MSG_CONSUMED;

        // Check provided values
        if ((param->conidx < BLE_CONNECTION_MAX)
                && ((scpps_env->features & SCPPS_SCAN_REFRESH_SUP) != 0)
                && ((scpps_env->ntf_cfg[param->conidx] & PRF_CLI_START_NTF) != 0))
        {

            // Allocate the GATT notification message
            struct gattc_send_evt_cmd *refresh_ntf = KE_MSG_ALLOC_DYN(GATTC_SEND_EVT_CMD,
                    KE_BUILD_ID(TASK_GATTC, param->conidx), dest_id,
                    gattc_send_evt_cmd, sizeof(uint8_t));

            // Fill in the parameter structure
            refresh_ntf->operation = GATTC_NOTIFY;
            refresh_ntf->handle = scpps_env->start_hdl + SCPS_IDX_SCAN_REFRESH_VAL;
            // pack measured value in database
            refresh_ntf->length = sizeof(uint8_t);
            refresh_ntf->value[0] = SCPP_SERVER_REQUIRES_REFRESH;

            // send notification to peer device
            ke_msg_send(refresh_ntf);
            // wait for event to be taken in account in Lower layers.
            ke_state_set(dest_id, SCPPS_BUSY);
        }
        // invalid connection index
        else if (param->conidx >= BLE_CONNECTION_MAX)
        {
            status = GAP_ERR_INVALID_PARAM;
        }
        // feature not supported
        else if ((scpps_env->features & SCPPS_SCAN_REFRESH_SUP) == 0)
        {
            status = PRF_ERR_FEATURE_NOT_SUPPORTED;
        }
        // notification not configured
        else // ((scpps_env->ntf_cfg[param->conidx] & PRF_CLI_START_NTF) == 0)
        {
            status = PRF_ERR_NTF_DISABLED;
        }

        if (status != GAP_ERR_NO_ERROR)
        {
            scpps_scan_refresh_rsp_send(scpps_env, param->conidx, status);
        }
    }

    return msg_status;
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

    struct gattc_att_info_cfm *cfm = KE_MSG_ALLOC(GATTC_ATT_INFO_CFM, src_id, dest_id, gattc_att_info_cfm);;
    struct scpps_env_tag *scpps_env = PRF_ENV_GET(SCPPS, scpps);

    //Send att_info response
    cfm->handle = param->handle;

    // Scan Refresh Notification Configuration
    if (param->handle == scpps_env->start_hdl + SCPS_IDX_SCAN_REFRESH_NTF_CFG)
    {
        cfm->length = 1;
        cfm->status = GAP_ERR_NO_ERROR;
    }
    else
    {
        cfm->length = 0;
        cfm->status = ATT_ERR_WRITE_NOT_PERMITTED;
    }

    ke_msg_send(cfm);

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref GL2C_CODE_ATT_WR_CMD_IND message.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC int gattc_write_req_ind_handler(ke_msg_id_t const msgid,
        struct gattc_write_req_ind const *param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
    // Status for write response
    uint8_t status = PRF_APP_ERROR;
    struct scpps_env_tag *scpps_env = PRF_ENV_GET(SCPPS, scpps);
    uint8_t conidx = KE_IDX_GET(src_id);
    struct gattc_write_cfm *cfm;


    // Scan Interval Window Value
    if (param->handle == scpps_env->start_hdl + SCPS_IDX_SCAN_INTV_WD_VAL)
    {
        uint16_t le_scan_intv;
        uint16_t le_scan_window;

        // Extract value
        le_scan_intv = co_read16p(&param->value[0]);
        le_scan_window = co_read16p(&param->value[sizeof(uint16_t)]);

        // Check interval and window validity
        if ((le_scan_window <= le_scan_intv)
                && (le_scan_window <= SCPPS_SCAN_WINDOW_MAX)
                && (le_scan_window >= SCPPS_SCAN_WINDOW_MIN)
                && (le_scan_intv <= SCPPS_SCAN_INTERVAL_MAX)
                && (le_scan_intv >= SCPPS_SCAN_INTERVAL_MIN))

        {
            // Inform APP of Scan Interval Window change
            struct scpps_scan_intv_wd_ind *ind = KE_MSG_ALLOC(SCPPS_SCAN_INTV_WD_IND,
                                                 prf_dst_task_get(&(scpps_env->prf_env), conidx),
                                                 dest_id, scpps_scan_intv_wd_ind);

            ind->conidx                      = conidx;
            ind->scan_intv_wd.le_scan_intv   = le_scan_intv;
            ind->scan_intv_wd.le_scan_window = le_scan_window;

            ke_msg_send(ind);
            status = GAP_ERR_NO_ERROR;
        }
    }
    // Scan Refresh Notification Configuration
    else if (param->handle == scpps_env->start_hdl + SCPS_IDX_SCAN_REFRESH_NTF_CFG)
    {
        // Check if Scan Refresh Characteristic is supported
        if ((scpps_env->features & SCPPS_SCAN_REFRESH_SUP) != 0)
        {
            // Extract value before check
            uint16_t ntf_cfg = co_read16p(&param->value[0]);

            // Only update configuration if value for stop or notification enable
            if ((ntf_cfg == PRF_CLI_STOP_NTFIND) || (ntf_cfg == PRF_CLI_START_NTF))
            {
                // Inform APP of configuration change
                struct scpps_scan_refresh_ntf_cfg_ind *ind = KE_MSG_ALLOC(SCPPS_SCAN_REFRESH_NTF_CFG_IND,
                        prf_dst_task_get(&(scpps_env->prf_env), conidx),
                        dest_id, scpps_scan_refresh_ntf_cfg_ind);

                // Update state value in environment
                scpps_env->ntf_cfg[conidx] = (uint8_t)ntf_cfg;

                ind->conidx = conidx;
                ind->ntf_cfg = scpps_env->ntf_cfg[conidx];
                ke_msg_send(ind);
                status = GAP_ERR_NO_ERROR;
            }
        }
    }

    //Send write response
    cfm = KE_MSG_ALLOC(GATTC_WRITE_CFM, src_id, dest_id, gattc_write_cfm);
    cfm->handle = param->handle;
    cfm->status = status;
    ke_msg_send(cfm);

    return (KE_MSG_CONSUMED);
}


/**
 ****************************************************************************************
 * @brief Handles reception of the @ref GATTC_READ_REQ_IND message.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC int gattc_read_req_ind_handler(ke_msg_id_t const msgid, struct gattc_read_req_ind const *param,
                                        ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint8_t conidx = KE_IDX_GET(src_id);
    struct scpps_env_tag *scpps_env = PRF_ENV_GET(SCPPS, scpps);

    struct gattc_read_cfm *cfm = KE_MSG_ALLOC_DYN(GATTC_READ_CFM, src_id, dest_id, gattc_read_cfm, sizeof(uint16_t));

    cfm->handle = param->handle;

    // Scan Refresh Notification Configuration
    if ((param->handle == scpps_env->start_hdl + SCPS_IDX_SCAN_REFRESH_NTF_CFG)
            && ((scpps_env->features & SCPPS_SCAN_REFRESH_SUP) != 0))
    {
        // Extract value before check
        uint16_t ntf_cfg = (scpps_env->ntf_cfg[conidx] == PRF_CLI_START_NTF)
                           ? PRF_CLI_START_NTF : PRF_CLI_STOP_NTFIND;

        co_write16p(&cfm->value[0], ntf_cfg);
        cfm->length = sizeof(uint16_t);
        cfm->status = GAP_ERR_NO_ERROR;
    }
    else
    {
        cfm->length = 0;
        cfm->status = PRF_APP_ERROR;
    }

    ke_msg_send(cfm);

    return (KE_MSG_CONSUMED);
}


/**
 ****************************************************************************************
 * @brief Handles @ref GATTC_CMP_EVT for GATTC_NOTIFY message meaning that Measurement
 * notification has been correctly sent to peer device (but not confirmed by peer device).
 * *
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
    uint8_t state = ke_state_get(dest_id);

    // check state of the task
    if ((state == SCPPS_BUSY) && (param->operation == GATTC_NOTIFY))
    {
        // send refresh response and go back to idle state
        scpps_scan_refresh_rsp_send(PRF_ENV_GET(SCPPS, scpps), KE_IDX_GET(src_id), param->status);
        ke_state_set(dest_id, SCPPS_IDLE);
    }

    return (KE_MSG_CONSUMED);
}


/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// Default State handlers definition
KE_MSG_HANDLER_TAB(scpps)
{
    {SCPPS_ENABLE_REQ, (ke_msg_func_t) scpps_enable_req_handler},
    {SCPPS_SCAN_REFRESH_SEND_REQ, (ke_msg_func_t) scpps_scan_refresh_send_req_handler},
    {GATTC_ATT_INFO_REQ_IND, (ke_msg_func_t) gattc_att_info_req_ind_handler},
    {GATTC_WRITE_REQ_IND, (ke_msg_func_t) gattc_write_req_ind_handler},
    {GATTC_READ_REQ_IND, (ke_msg_func_t) gattc_read_req_ind_handler},
    {GATTC_CMP_EVT, (ke_msg_func_t) gattc_cmp_evt_handler},
};

void scpps_task_init(struct ke_task_desc *task_desc)
{
    // Get the address of the environment
    struct scpps_env_tag *scpps_env = PRF_ENV_GET(SCPPS, scpps);

    task_desc->msg_handler_tab = scpps_msg_handler_tab;
    task_desc->msg_cnt         = ARRAY_LEN(scpps_msg_handler_tab);
    task_desc->state           = scpps_env->state;
    task_desc->idx_max         = SCPPS_IDX_MAX;
}

#endif /* #if (BLE_SP_SERVER) */

/// @} SCPPSTASK
