/**
 ****************************************************************************************
 *
 * @file sibles_task.c
 *
 * @brief Device Information Service Task implementation.
 *
 * Copyright (C) Sifli 2019-2022
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup SIBLESTASK
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"

#if (BLE_APP_PRESENT)
#if (BLE_SIBLE_SERVER)
#include "co_utils.h"
#include "gap.h"
#include "gattc_task.h"
#include "sibles.h"
#include "sibles_task.h"
#include "prf_utils.h"
#include "attm_db.h"
#include "ke_mem.h"
#include "dbg_trc.h"
#include "app.h"
#include "gattc.h"
#include "l2cc_pdu.h"
#include "l2cc.h"
#include "hci.h"

#include "gapm_int.h"
/*
 * GLOBAL VARIABLES
 ****************************************************************************************
 */

sibles_task_init_handler _sibles_task_init_handler = _sibles_task_init;

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

#ifndef SOC_BF0_BCPU
void sifli_mbox_bcpu2hcpu(uint8_t *param)
{
    ke_msg_send(param);
}
#endif

uint8_t g_search_svc_len = 0;
uint8_t *g_search_svc = NULL;

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
    ke_state_t state = ke_state_get(dest_id);

    if (state == SIBLES_IDLE)
    {
        struct sibles_env_tag *sibles_env = PRF_ENV_GET(SIBLES, sibles);
        // Check value in already present in service
        struct sibles_val_elmt *val = (struct sibles_val_elmt *) co_list_pick(&(sibles_env->values));
        // loop until value found
        while (val != NULL)
        {
            // value is present in service
            if (val->hdl == param->handle - sibles_env->start_hdl)
            {
                break;
            }
            val = (struct sibles_val_elmt *)val->hdr.next;
        }

        if (val != NULL)
        {
            // Send value to peer device.
            struct gattc_read_cfm *cfm = KE_MSG_ALLOC_DYN(GATTC_READ_CFM, src_id, dest_id, gattc_read_cfm, val->length);
            cfm->handle = param->handle;
            cfm->status = ATT_ERR_NO_ERROR;
            cfm->length = val->length;
            memcpy(cfm->value, val->data, val->length);
            ke_msg_send(cfm);
        }
        else
        {
            // request value to application
            sibles_env->req_hdl    = param->handle;
            sibles_env->req_conidx = KE_IDX_GET(src_id);
#ifndef PLF_UART2_OVERMBOX
            struct sibles_value_req_ind *req_ind = KE_MSG_ALLOC(SIBLES_VALUE_REQ_IND,
                                                   prf_dst_task_get(&(sibles_env->prf_env), KE_IDX_GET(src_id)),
                                                   dest_id, sibles_value_req_ind);
#else
            struct sibles_value_req_ind *req_ind = KE_MSG_ALLOC(SIBLES_VALUE_REQ_IND,
                                                   gapm_get_AHI_task_id(), prf_dst_task_get(&(sibles_env->prf_env), KE_IDX_GET(src_id)),
                                                   sibles_value_req_ind);

#endif
            req_ind->hdl = param->handle;
            sifli_mbox_bcpu2hcpu((uint8_t *)req_ind);
            // Put Service in a busy state
            ke_state_set(dest_id, SIBLES_BUSY);
        }

    }
    // postpone request if profile is in a busy state - required for multipoint
    else if (state == SIBLES_BUSY)
    {
        msg_status = KE_MSG_SAVED;
    }

    return (msg_status);
}


/**
 ****************************************************************************************
 * @brief Handles reception of the @ref SIBLES_SET_VALUE_REQ message.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC int sibles_set_value_req_handler(ke_msg_id_t const msgid,
        struct sibles_value const *param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
    struct sibles_env_tag *sibles_env = PRF_ENV_GET(SIBLES, sibles);
    struct sibles_value_ack *rsp;

    // Check value in already present in service
    struct sibles_val_elmt *val = (struct sibles_val_elmt *) co_list_pick(&(sibles_env->values));
    // loop until value found
    while (val != NULL)
    {
        // if value already present, remove old one
        if (val->hdl == param->hdl)
        {
            co_list_extract(&(sibles_env->values), &(val->hdr));
            ke_free(val);
            break;
        }
        val = (struct sibles_val_elmt *)val->hdr.next;
    }

    // allocate value data
#ifndef PLF_UART2_OVERMBOX
    rsp = KE_MSG_ALLOC(SIBLES_SET_VALUE_RSP, src_id, dest_id, sibles_value_ack);
#else
    rsp = KE_MSG_ALLOC(SIBLES_SET_VALUE_RSP, gapm_get_AHI_task_id(), dest_id, sibles_value_ack);
#endif
    rsp->hdl = param->hdl;
    val = (struct sibles_val_elmt *) ke_malloc(sizeof(struct sibles_val_elmt) + param->length, KE_MEM_ATT_DB);
    if (val)
    {
        val->hdl = param->hdl;
        val->length = param->length;
        memcpy(val->data, param->data, param->length);
        // insert value into the list
        co_list_push_back(&(sibles_env->values), &(val->hdr));
        rsp->status = GAP_ERR_NO_ERROR;
    }
    else
        rsp->status = ATT_ERR_APP_ERROR;

    // send response to application
    sifli_mbox_bcpu2hcpu((uint8_t *)rsp);
    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the value confirmation from application
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC int sibles_value_req_cfm_handler(ke_msg_id_t const msgid,
        struct sibles_value const *param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
    ke_state_t state = ke_state_get(dest_id);

    if (state == SIBLES_BUSY)
    {
        struct sibles_env_tag *sibles_env = PRF_ENV_GET(SIBLES, sibles);

        // chack if application provide correct value
        if (sibles_env->req_hdl == param->hdl)
        {
            // Send value to peer device.
            struct gattc_read_cfm *cfm = KE_MSG_ALLOC_DYN(GATTC_READ_CFM, KE_BUILD_ID(TASK_GATTC, sibles_env->req_conidx), dest_id, gattc_read_cfm, param->length);
            cfm->handle = param->hdl;
            cfm->status = ATT_ERR_NO_ERROR;
            cfm->length = param->length;
            memcpy(cfm->value, param->data, param->length);
            ke_msg_send(cfm);
        }
        else
        {
            // application error, value provided by application is not the expected one
            struct gattc_read_cfm *cfm = KE_MSG_ALLOC(GATTC_READ_CFM, KE_BUILD_ID(TASK_GATTC, sibles_env->req_conidx), dest_id, gattc_read_cfm);
            cfm->handle = param->hdl;
            cfm->status = ATT_ERR_APP_ERROR;
            ke_msg_send(cfm);
        }

        // return to idle state
        ke_state_set(dest_id, SIBLES_IDLE);
    }
    // else ignore request if not in busy state

    return (KE_MSG_CONSUMED);
}



extern struct gattc_env_tag *gattc_env[BLE_CONNECTION_MAX];

// Reponse directly for write command.
uint8_t gatt_get_current_gatt_code(uint8_t conidx)
{
    struct ke_msg *pdu_msg = (struct ke_msg *) co_list_pick(&(gattc_env[conidx]->server.pdu_queue));
    if (pdu_msg)
    {
        struct l2cc_pdu *pdu = &(((struct l2cc_pdu_recv_ind *) ke_msg2param(pdu_msg))->pdu);
        return pdu->data.code;
    }
    return 0;
}

/**
****************************************************************************************
* @brief Handles reception of the @ref  GATTC_WRITE_REQ_IND message.
* The message is redirected from TASK_SVC because at profile enable, the ATT handle is
* register for TASK_FINDT. In the handler, an ATT Write Response/Error Response should
* be sent for ATT protocol, but Alert Level Characteristic only supports WNR so no
* response PDU is needed.
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

    int msg_status = KE_MSG_CONSUMED;
    //int msg_status = KE_MSG_NO_FREE;

    uint8_t conn_idx = KE_IDX_GET(src_id);
    uint8_t gatt_code, is_cmd = 0;
    struct sibles_env_tag *sibles_env = PRF_ENV_GET(SIBLES, sibles);
    struct sibles_value_write_ind *req_ind = KE_MSG_ALLOC_DYN(SIBLES_VALUE_WRITE_IND,
            gapm_get_AHI_task_id(), dest_id, sibles_value_write_ind, param->length);

    sibles_env->req_conidx = conn_idx;
    gatt_code = gatt_get_current_gatt_code(conn_idx);
    if (gatt_code == L2C_CODE_ATT_WR_CMD || gatt_code == L2C_CODE_ATT_SIGN_WR_CMD)
        is_cmd = 1;
    req_ind->offset = param->offset;
    req_ind->hdl = param->handle;
    req_ind->length = param->length;
    req_ind->is_cmd = is_cmd;
    memcpy(req_ind->data, param->value, param->length);

    //ke_msg_forward_new_id(param, SIBLES_VALUE_WRITE_IND, TASK_AHI, dest_id);
    sifli_mbox_bcpu2hcpu((uint8_t *)req_ind);

    if (is_cmd)
    {
        struct gattc_write_cfm *cfm = KE_MSG_ALLOC(GATTC_WRITE_CFM, KE_BUILD_ID(TASK_GATTC, sibles_env->req_conidx), dest_id, gattc_write_cfm);

        // Fill in the parameter structure
        cfm->handle = param->handle;
        cfm->status = 0;
        ke_msg_send(cfm);
    }

    return (msg_status);
}



__STATIC int sibles_value_write_cfm_handler(ke_msg_id_t const msgid,
        struct sibles_value_ack const *param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
    // Allocate write confirmation message.
    struct sibles_env_tag *sibles_env = PRF_ENV_GET(SIBLES, sibles);
    struct gattc_write_cfm *cfm = KE_MSG_ALLOC(GATTC_WRITE_CFM, KE_BUILD_ID(TASK_GATTC, sibles_env->req_conidx), dest_id, gattc_write_cfm);

    // Fill in the parameter structure
    cfm->handle = param->hdl;
    cfm->status = param->status;

    // Send the message
    ke_msg_send(cfm);

    return (KE_MSG_CONSUMED);
}

__STATIC int sibles_svc_reg_req_handler(ke_msg_id_t const msgid,
                                        struct sibles_svc_reg_req const *param,
                                        ke_task_id_t const dest_id,
                                        ke_task_id_t const src_id)
{
    // DB Creation Statis
    uint8_t status = ATT_ERR_NO_ERROR;
    uint16_t start_hdl = 0;
    // Get the address of the environment
    struct attm_desc *att_ptr = (struct attm_desc *)(&param->att_db);
    struct sibles_env_tag *env = PRF_ENV_GET(SIBLES, sibles);
#ifndef PLF_UART2_OVERMBOX
    struct sibles_svc_rsp *rsp = KE_MSG_ALLOC(SIBLES_SVC_RSP, src_id, dest_id, sibles_svc_rsp);
#else
    struct sibles_svc_rsp *rsp = KE_MSG_ALLOC(SIBLES_SVC_RSP, gapm_get_AHI_task_id(), dest_id, sibles_svc_rsp);
#endif
    status = attm_svc_create_db(&start_hdl, param->svc_uuid, NULL,
                                param->attm_entries, NULL, prf_src_task_get(&(env->prf_env), 0), att_ptr,
                                (param->sec_lvl & (PERM_MASK_SVC_DIS | PERM_MASK_SVC_AUTH | PERM_MASK_SVC_EKS | PERM_MASK_SVC_MI)));

    rsp->start_hdl = start_hdl;
    rsp->status = status;
    sifli_mbox_bcpu2hcpu((uint8_t *)rsp);

    return (KE_MSG_CONSUMED);
}


__STATIC int sibles_svc_reg128_req_handler(ke_msg_id_t const msgid,
        struct sibles_svc_reg128_req const *param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
    // DB Creation Statis
    uint8_t status = ATT_ERR_NO_ERROR;
    uint16_t start_hdl = 0;
    //M0 didn't support empty strcutre but M33 used it.
    struct attm_desc_128 *att_ptr = (struct attm_desc_128 *)(&param->att_db);
    // Get the address of the environment
    struct sibles_env_tag *env = PRF_ENV_GET(SIBLES, sibles);
#ifndef PLF_UART2_OVERMBOX
    struct sibles_svc_rsp *rsp = KE_MSG_ALLOC(SIBLES_SVC_RSP, src_id, dest_id, sibles_svc_rsp);
#else
    struct sibles_svc_rsp *rsp = KE_MSG_ALLOC(SIBLES_SVC_RSP, gapm_get_AHI_task_id(), dest_id, sibles_svc_rsp);
#endif
    status = attm_svc_create_db_128(&start_hdl, param->svc_uuid, NULL,
                                    param->attm_entries, NULL, prf_src_task_get(&(env->prf_env), 0), att_ptr,
                                    (param->sec_lvl & (PERM_MASK_SVC_DIS | PERM_MASK_SVC_AUTH | PERM_MASK_SVC_EKS | PERM_MASK_SVC_UUID_LEN | PERM_MASK_SVC_MI)));

    rsp->start_hdl = start_hdl;
    rsp->status = status;
    sifli_mbox_bcpu2hcpu((uint8_t *)rsp);
    return (KE_MSG_CONSUMED);
}

__STATIC int sibles_att_update_handler(ke_msg_id_t const msgid,
                                       struct sibles_update_att_perm_req const *param,
                                       ke_task_id_t const dest_id,
                                       ke_task_id_t const src_id)
{
    uint8_t status = ATT_ERR_NO_ERROR;

#ifndef PLF_UART2_OVERMBOX
    struct sibles_update_att_perm_rsp *rsp = KE_MSG_ALLOC(SIBLES_UPDATE_ATT_PERM_RSP, src_id, dest_id, sibles_update_att_perm_rsp);
#else
    struct sibles_update_att_perm_rsp *rsp = KE_MSG_ALLOC(SIBLES_UPDATE_ATT_PERM_RSP, gapm_get_AHI_task_id(), dest_id, sibles_update_att_perm_rsp);
#endif
    status = attm_att_update_perm(param->handle, param->access_mask, param->perm);

    rsp->handle = param->handle;
    rsp->status = status;
    sifli_mbox_bcpu2hcpu((uint8_t *)rsp);
    return (KE_MSG_CONSUMED);
}

// Need add the handler of message handler.
__STATIC int silbes_svc_search_req_handler(ke_msg_id_t const msgid,
        struct sibles_svc_search_req const *param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
    int msg_status = KE_MSG_CONSUMED;
    // search service first, just implment uuid search
    if (ke_state_get(dest_id) == SIBLES_IDLE)
    {
        struct sibles_env_tag *env = PRF_ENV_GET(SIBLES, sibles);

        struct gattc_disc_cmd *svc_disc =
            KE_MSG_ALLOC_DYN(GATTC_DISC_CMD,
                             KE_BUILD_ID(TASK_GATTC, param->conn_idx),
                             prf_src_task_get(&(env->prf_env), KE_IDX_GET(src_id)),
                             gattc_disc_cmd, ATT_UUID_128_LEN);
        svc_disc->operation = GATTC_DISC_BY_UUID_SVC;
        svc_disc->start_hdl = 0x1;
        svc_disc->end_hdl = 0xFFFF;
        svc_disc->uuid_len = param->len;
        if (g_search_svc)
        {
            free(g_search_svc);
        }

        g_search_svc_len = param->len;
        g_search_svc = malloc(param->len);
        memcpy(g_search_svc, param->svc_uuid, param->len);
        memcpy(svc_disc->uuid, param->svc_uuid, param->len);
        ke_msg_send(svc_disc);
        ke_state_set(dest_id, SIBLES_BUSY);
        return msg_status;

    }
    else if (ke_state_get(dest_id) == SIBLES_BUSY)
    {
        msg_status = KE_MSG_SAVED;
    }

    return msg_status;
}

__STATIC int silbes_disc_svc_ind_handler(ke_msg_id_t const msgid,
        struct gattc_disc_svc_ind const *param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
    struct sibles_env_tag *env = PRF_ENV_GET(SIBLES, sibles);

    /// Notify upper layer the start handle.
    struct sibles_disc_svc_ind *ind = KE_MSG_ALLOC(SIBLES_DISC_SVC_IND, gapm_get_AHI_task_id(), dest_id, sibles_disc_svc_ind);
    ind->start_hdl = param->start_hdl;
    ind->end_hdl = param->end_hdl;
    ind->uuid_len = param->uuid_len;
    memcpy(ind->uuid, param->uuid, param->uuid_len);
    env->svc = (struct gattc_disc_svc_ind *)param;

    sifli_mbox_bcpu2hcpu((uint8_t *)ind);

    /// request the char
    struct gattc_disc_cmd *char_disc =
        KE_MSG_ALLOC_DYN(GATTC_DISC_CMD,
                         KE_BUILD_ID(TASK_GATTC, KE_IDX_GET(src_id)),
                         dest_id,
                         gattc_disc_cmd, ATT_UUID_16_LEN);

    char_disc->operation = GATTC_DISC_ALL_CHAR;
    char_disc->start_hdl = param->start_hdl;
    char_disc->end_hdl = param->end_hdl;
    char_disc->uuid_len  = ATT_UUID_16_LEN;
    memset(char_disc->uuid, 0, ATT_UUID_16_LEN);
    ke_msg_send(char_disc);

    /// env has recorded the svc info for the following searching.
    return (KE_MSG_NO_FREE);

}

__STATIC int silbes_disc_char_ind_handler(ke_msg_id_t const msgid,
        struct gattc_disc_char_ind const *param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
    struct sibles_disc_char_ind *ind = KE_MSG_ALLOC(SIBLES_DISC_CHAR_IND, gapm_get_AHI_task_id(), dest_id, sibles_disc_char_ind);
    ind->attr_hdl = param->attr_hdl;
    ind->pointer_hdl = param->pointer_hdl;
    ind->prop = param->prop;
    ind->uuid_len = param->uuid_len;
    memcpy(ind->uuid, param->uuid, param->uuid_len);
    sifli_mbox_bcpu2hcpu((uint8_t *)ind);
    return (KE_MSG_CONSUMED);
}

__STATIC uint8_t sibles_desc_check_via_16bit(uint8_t *uuid)
{
    // high byte of descriptor is 0x29
    uint16_t h_uuid = uuid[1];
    if (h_uuid == 0x29)
        return 1;
    return 0;
}


__STATIC int silbes_disc_char_desc_ind_handler(ke_msg_id_t const msgid,
        struct gattc_disc_char_desc_ind const *param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
    if (param->uuid_len == 2 && sibles_desc_check_via_16bit((uint8_t *)param->uuid))
    {
        struct sibles_disc_char_desc_ind *ind = KE_MSG_ALLOC(SIBLES_DISC_CHAR_DESC_IND,
                                                gapm_get_AHI_task_id(), dest_id,
                                                sibles_disc_char_desc_ind);
        // Only notify user with descriptor uuid
        // There may happen a race condition that remote device use 32/128b uuid
        ind->attr_hdl = param->attr_hdl;
        ind->uuid_len = param->uuid_len;
        memcpy(ind->uuid, param->uuid, param->uuid_len);
        sifli_mbox_bcpu2hcpu((uint8_t *)ind);
    }

    return (KE_MSG_CONSUMED);
}


__STATIC int silbes_register_peer_svc_handler(ke_msg_id_t const msgid,
        struct sibles_register_notify_req_t const *param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
    struct sibles_env_tag *env = PRF_ENV_GET(SIBLES, sibles);

    struct gattc_reg_to_peer_evt_cmd *cmd = KE_MSG_ALLOC(GATTC_REG_TO_PEER_EVT_CMD,
                                            KE_BUILD_ID(TASK_GATTC, KE_IDX_GET(src_id)),
                                            prf_src_task_get(&(env->prf_env), KE_IDX_GET(src_id)),
                                            gattc_reg_to_peer_evt_cmd);
    cmd->operation = GATTC_REGISTER;
    cmd->seq_num = param->seq_num;
    cmd->start_hdl = param->hdl_start;
    cmd->end_hdl   = param->hdl_end;
    ke_msg_send(cmd);

    return (KE_MSG_CONSUMED);

}


__STATIC int silbes_unregister_peer_svc_handler(ke_msg_id_t const msgid,
        struct sibles_register_notify_req_t const *param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
    struct sibles_env_tag *env = PRF_ENV_GET(SIBLES, sibles);

    struct gattc_reg_to_peer_evt_cmd *cmd = KE_MSG_ALLOC(GATTC_REG_TO_PEER_EVT_CMD,
                                            KE_BUILD_ID(TASK_GATTC, KE_IDX_GET(src_id)),
                                            prf_src_task_get(&(env->prf_env), KE_IDX_GET(src_id)),
                                            gattc_reg_to_peer_evt_cmd);
    cmd->operation = GATTC_UNREGISTER;
    cmd->seq_num = param->seq_num;
    cmd->start_hdl = param->hdl_start;
    cmd->end_hdl   = param->hdl_end;
    ke_msg_send(cmd);

    return (KE_MSG_CONSUMED);
}

__STATIC int silbes_value_write_req_handler(ke_msg_id_t const msgid,
        struct sibles_value_write_req_t const *param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
    uint8_t operation[] = {GATTC_WRITE, GATTC_WRITE_NO_RESPONSE};
    struct sibles_env_tag *env = PRF_ENV_GET(SIBLES, sibles);

    struct gattc_write_cmd *wr_char = KE_MSG_ALLOC_DYN(GATTC_WRITE_CMD,
                                      KE_BUILD_ID(TASK_GATTC, KE_IDX_GET(src_id)), prf_src_task_get(&(env->prf_env), 0),
                                      gattc_write_cmd, param->length);

    // Offset
    wr_char->offset         = 0x0000;
    // cursor always 0
    wr_char->cursor         = 0x0000;
    // Write Type
    wr_char->operation       = operation[param->write_type];

    wr_char->seq_num = param->seq_num;
    // Characteristic Value attribute handle
    wr_char->handle         = param->handle;
    // Value Length
    wr_char->length         = param->length;
    // Auto Execute
    wr_char->auto_execute   = true;
    // Value
    memcpy(&wr_char->value[0], param->value, param->length);

    // Send the message
    ke_msg_send(wr_char);
    return (KE_MSG_CONSUMED);
}

__STATIC int silbes_event_ind_handler(ke_msg_id_t const msgid,
                                      struct gattc_event_ind const *param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id)
{
    struct sibles_env_tag *env = PRF_ENV_GET(SIBLES, sibles);
    if (param->type == GATTC_INDICATE)
    {
        struct gattc_event_cfm *cfm = KE_MSG_ALLOC(GATTC_EVENT_CFM,
                                      KE_BUILD_ID(TASK_GATTC, KE_IDX_GET(src_id)),
                                      prf_src_task_get(&(env->prf_env), 0),
                                      gattc_event_cfm);

        cfm->handle = param->handle;
        ke_msg_send((void const *)cfm);
    }

    struct gattc_event_ind *ind = KE_MSG_ALLOC_DYN(SIBLES_EVENT_IND,
                                  gapm_get_AHI_task_id(), dest_id,
                                  gattc_event_ind, param->length);
    ind->type = param->type;
    ind->length = param->length;
    ind->handle = param->handle;
    memcpy(ind->value, param->value, param->length);
    sifli_mbox_bcpu2hcpu((uint8_t *)ind);
    return (KE_MSG_CONSUMED);
}

__STATIC int silbes_value_read_req_handler(ke_msg_id_t const msgid,
        struct sibles_value_read_req_t const *param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
    uint8_t operation[] = {GATTC_READ, GATTC_READ_LONG, GATTC_READ_BY_UUID, GATTC_READ_MULTIPLE};
    struct sibles_env_tag *env = PRF_ENV_GET(SIBLES, sibles);
    struct gattc_read_cmd *req  = KE_MSG_ALLOC(GATTC_READ_CMD, KE_BUILD_ID(TASK_GATTC, KE_IDX_GET(src_id)),
                                  prf_src_task_get(&(env->prf_env), 0), gattc_read_cmd);
    //request type
    req->operation                      = operation[param->read_type]; // Only support GATTC_READ now.
    req->nb                             = 1;
    req->seq_num                        = param->seq_num;
    req->req.simple.offset              = param->offset;
    req->req.simple.length              = param->length;
    req->req.simple.handle              = param->handle;

    //send request to GATT
    ke_msg_send(req);
    return (KE_MSG_CONSUMED);
}

__STATIC int silbes_value_read_ind_handler(ke_msg_id_t const msgid,
        struct gattc_read_ind const *param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
    ke_msg_forward_new_id(param, SIBLES_VALUE_READ_RSP, gapm_get_AHI_task_id(), dest_id);
    return (KE_MSG_NO_FREE);
}



/****************************************************************************************
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
    struct sibles_env_tag *sibles_env = PRF_ENV_GET(SIBLES, sibles);
    struct gattc_att_info_cfm *cfm;
    struct attm_elmt elmt = ATT_ELEMT_INIT;
    uint8_t status = attmdb_get_attribute(param->handle, &elmt);

    //Send write response
    cfm = KE_MSG_ALLOC(GATTC_ATT_INFO_CFM, src_id, dest_id, gattc_att_info_cfm);
    cfm->handle = param->handle;
    cfm->length = sizeof(uint16_t);

    if (status == ATT_ERR_NO_ERROR)
    {
        if ((elmt.info.att->perm & PERM_MASK_WRITE_REQ) == 0)
            status = ATT_ERR_WRITE_NOT_PERMITTED;
    }
    cfm->status = status;
    ke_msg_send(cfm);

    return (KE_MSG_CONSUMED);
}


__STATIC int sibles_value_ind_req_handler(ke_msg_id_t const msgid,
        struct sibles_value const *param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
    struct sibles_env_tag *sibles_env = PRF_ENV_GET(SIBLES, sibles);
    struct gattc_send_evt_cmd *sibles_ntf = KE_MSG_ALLOC_DYN(GATTC_SEND_EVT_CMD,
                                            KE_BUILD_ID(TASK_GATTC, TASK_IDX_GET(dest_id)), prf_src_task_get(&(sibles_env->prf_env), 0),
                                            gattc_send_evt_cmd, param->length);

    // Fill in the parameter structure
    sibles_ntf->operation = GATTC_INDICATE;
    sibles_ntf->handle = param->hdl;
    // pack measured value in database
    sibles_ntf->length = param->length;
    memcpy(sibles_ntf->value, param->data, param->length);

    // send notification to peer device
    ke_msg_send(sibles_ntf);
    return (KE_MSG_CONSUMED);
}

__STATIC int sibles_value_ntf_ind_handler(ke_msg_id_t const msgid,
        struct sibles_value const *param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
    struct sibles_env_tag *sibles_env = PRF_ENV_GET(SIBLES, sibles);
    struct gattc_send_evt_cmd *sibles_ntf = KE_MSG_ALLOC_DYN(GATTC_SEND_EVT_CMD,
                                            KE_BUILD_ID(TASK_GATTC, TASK_IDX_GET(dest_id)), prf_src_task_get(&(sibles_env->prf_env), 0),
                                            gattc_send_evt_cmd, param->length);

    // Fill in the parameter structure
    sibles_ntf->operation = GATTC_NOTIFY;
    sibles_ntf->handle = param->hdl;
    // pack measured value in database
    sibles_ntf->length = param->length;
    memcpy(sibles_ntf->value, param->data, param->length);

    // send notification to peer device
    ke_msg_send(sibles_ntf);
    return (KE_MSG_CONSUMED);
}

__STATIC int gattc_cmp_evt_handler(ke_msg_id_t const msgid,  struct gattc_cmp_evt const *param,
                                   ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    struct sibles_env_tag *sibles_env = PRF_ENV_GET(SIBLES, sibles);

    switch (param->operation)
    {
    case GATTC_NOTIFY:
    case GATTC_INDICATE:
    {
        struct sibles_value_ack *rsp;
#ifndef PLF_UART2_OVERMBOX
        rsp = KE_MSG_ALLOC(SIBLES_VALUE_IND_RSP, prf_src_task_get(&(sibles_env->prf_env), 0), TASK_ID_APP, sibles_value_ack);
#else
        rsp = KE_MSG_ALLOC(SIBLES_VALUE_IND_RSP, gapm_get_AHI_task_id(), dest_id, sibles_value_ack);
#endif
        rsp->hdl = 0;   // Not used.
        rsp->status = param->status;

        sifli_mbox_bcpu2hcpu((uint8_t *)rsp);
        break;
    }
#if (BLE_ATTC)
    case GATTC_WRITE:
    case GATTC_WRITE_NO_RESPONSE:
    {
        struct sibles_value_ack *rsp;
#ifndef PLF_UART2_OVERMBOX
        rsp = KE_MSG_ALLOC(SIBLES_WRTIE_VALUE_RSP, prf_src_task_get(&(sibles_env->prf_env), 0), TASK_ID_APP, sibles_value_ack);
#else
        rsp = KE_MSG_ALLOC(SIBLES_WRTIE_VALUE_RSP, gapm_get_AHI_task_id(), dest_id, sibles_value_ack);
#endif
        rsp->hdl = 0;   // Not used.
        rsp->status = param->status;

        sifli_mbox_bcpu2hcpu((uint8_t *)rsp);
        break;
    }
    case GATTC_DISC_BY_UUID_SVC:
    {
        if (!(param->status == GAP_ERR_NO_ERROR ||
                // svc has value means uuid already found
                (param->status == ATT_ERR_ATTRIBUTE_NOT_FOUND && (sibles_env->svc != NULL))))
        {
            struct sibles_svc_search_rsp *rsp =  KE_MSG_ALLOC(SIBLES_SVC_SEARCH_RSP,
                                                 gapm_get_AHI_task_id(), dest_id, sibles_svc_search_rsp);
            rsp->status = param->status;

            if (g_search_svc)
            {
                rsp->len = g_search_svc_len;
                memcpy(rsp->svc_uuid, g_search_svc, g_search_svc_len);

                g_search_svc_len = 0;
                free(g_search_svc);
                g_search_svc = NULL;
            }

            sifli_mbox_bcpu2hcpu((uint8_t *)rsp);
            ke_state_set(dest_id, SIBLES_IDLE);
        }
        break;
    }
    case GATTC_DISC_ALL_CHAR:
    {
        if ((param->status == GAP_ERR_NO_ERROR) || (param->status == ATT_ERR_ATTRIBUTE_NOT_FOUND))
        {
            /// Send a Descriptor Discovery
            struct gattc_disc_cmd *desc_disc =
                KE_MSG_ALLOC_DYN(GATTC_DISC_CMD,
                                 KE_BUILD_ID(TASK_GATTC, KE_IDX_GET(src_id)),
                                 dest_id,
                                 gattc_disc_cmd, ATT_UUID_16_LEN);

            desc_disc->operation = GATTC_DISC_DESC_CHAR;
            desc_disc->start_hdl = sibles_env->svc->start_hdl + 1;
            desc_disc->end_hdl = sibles_env->svc->end_hdl;
            desc_disc->uuid_len  = ATT_UUID_16_LEN;

            memset(desc_disc->uuid, 0, ATT_UUID_16_LEN);

            // Reset the Char_Idx back to zero. So Descriptor discovery begins at the first char.
            //envc_env->env[conidx]->char_idx = 0;
            ke_msg_send(desc_disc);

        }
        else
        {
            struct sibles_svc_search_rsp *rsp =  KE_MSG_ALLOC(SIBLES_SVC_SEARCH_RSP,
                                                 gapm_get_AHI_task_id(), dest_id, sibles_svc_search_rsp);
            rsp->status = param->status;

            if (g_search_svc)
            {
                rsp->len = g_search_svc_len;
                memcpy(rsp->svc_uuid, g_search_svc, g_search_svc_len);

                g_search_svc_len = 0;
                free(g_search_svc);
                g_search_svc = NULL;
            }

            sifli_mbox_bcpu2hcpu((uint8_t *)rsp);
            if (sibles_env->svc)
            {
                KE_MSG_FREE(sibles_env->svc);
                sibles_env->svc = NULL;
            }
            ke_state_set(dest_id, SIBLES_IDLE);
            //notify upper layer search failed
        }
        break;
    }
    case GATTC_DISC_DESC_CHAR:
    {
        struct sibles_svc_search_rsp *rsp =  KE_MSG_ALLOC(SIBLES_SVC_SEARCH_RSP,
                                             gapm_get_AHI_task_id(), dest_id, sibles_svc_search_rsp);
        if ((param->status == GAP_ERR_NO_ERROR) || (param->status == ATT_ERR_ATTRIBUTE_NOT_FOUND))
        {
            rsp->status = GAP_ERR_NO_ERROR;
        }
        else
        {
            rsp->status = param->status;
        }

        rsp->len = sibles_env->svc->uuid_len;
        memcpy(rsp->svc_uuid, sibles_env->svc->uuid, sibles_env->svc->uuid_len);

        if (g_search_svc)
        {
            g_search_svc_len = 0;
            free(g_search_svc);
            g_search_svc = NULL;
        }

        sifli_mbox_bcpu2hcpu((uint8_t *)rsp);
        if (sibles_env->svc)
        {
            KE_MSG_FREE(sibles_env->svc);
            sibles_env->svc = NULL;
        }
        ke_state_set(dest_id, SIBLES_IDLE);
        break;
    }
    case GATTC_REGISTER:
    {
        struct sibles_register_notify_rsp_t *rsp = KE_MSG_ALLOC(SIBLES_REGISTER_NOTIFY_RSP,
                gapm_get_AHI_task_id(), dest_id, sibles_register_notify_rsp_t);
        rsp->status = param->status;
        rsp->seq_num = param->seq_num;
        sifli_mbox_bcpu2hcpu((uint8_t *)rsp);
        break;
    }
#endif // BLE_ATTC
    default:
        break;
    }
    return (KE_MSG_CONSUMED);
}

__STATIC int sibles_adv_data_handler(ke_msg_id_t const msgid,  struct sibles_value const *param,
                                     ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    struct sibles_env_tag *sibles_env = PRF_ENV_GET(SIBLES, sibles);

    if (sibles_env->adv_data)
        ke_free(sibles_env->adv_data);
    sibles_env->adv_data = ke_malloc(sizeof(struct sibles_value) + param->length, KE_MEM_ENV);
    memcpy(sibles_env->adv_data, param, sizeof(struct sibles_value) + param->length);
    {
        struct sibles_value_ack *rsp;
#ifndef PLF_UART2_OVERMBOX
        rsp = KE_MSG_ALLOC(SIBLES_CMD_RSP, prf_src_task_get(&(sibles_env->prf_env), 0), TASK_ID_APP, sibles_value_ack);
#else
        rsp = KE_MSG_ALLOC(SIBLES_CMD_RSP, gapm_get_AHI_task_id(), dest_id, sibles_value_ack);
#endif
        rsp->hdl = 0;   // Not used.
        rsp->status = 0;
        sifli_mbox_bcpu2hcpu((uint8_t *)rsp);
    }
    return (KE_MSG_CONSUMED);
}

__STATIC int sibles_scan_rsp_handler(ke_msg_id_t const msgid,  struct sibles_value const *param,
                                     ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    struct sibles_env_tag *sibles_env = PRF_ENV_GET(SIBLES, sibles);

    if (sibles_env->scan_rsp)
        ke_free(sibles_env->scan_rsp);

    sibles_env->scan_rsp = ke_malloc(sizeof(struct sibles_value) + param->length, KE_MEM_ENV);
    memcpy(sibles_env->scan_rsp, param, sizeof(struct sibles_value) + param->length);
    {
        struct sibles_value_ack *rsp;
#ifndef PLF_UART2_OVERMBOX
        rsp = KE_MSG_ALLOC(SIBLES_CMD_RSP, prf_src_task_get(&(sibles_env->prf_env), 0), TASK_ID_APP, sibles_value_ack);
#else
        rsp = KE_MSG_ALLOC(SIBLES_CMD_RSP, gapm_get_AHI_task_id(), dest_id, sibles_value_ack);
#endif
        rsp->hdl = 0;   // Not used.
        rsp->status = 0;
        sifli_mbox_bcpu2hcpu((uint8_t *)rsp);
    }
    return (KE_MSG_CONSUMED);
}

__STATIC int sibles_adv_cmd_handler(ke_msg_id_t const msgid,  uint8_t *param,
                                    ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    struct sibles_env_tag *sibles_env = PRF_ENV_GET(SIBLES, sibles);
    //appm_update_adv_state(1);
    {
        struct sibles_value_ack *rsp;
#ifndef PLF_UART2_OVERMBOX
        rsp = KE_MSG_ALLOC(SIBLES_CMD_RSP, prf_src_task_get(&(sibles_env->prf_env), 0), TASK_ID_APP, sibles_value_ack);
#else
        rsp = KE_MSG_ALLOC(SIBLES_CMD_RSP, gapm_get_AHI_task_id(), dest_id, sibles_value_ack);
#endif
        rsp->hdl = 0;   // Not used.
        rsp->status = 0;
        sifli_mbox_bcpu2hcpu((uint8_t *)rsp);
    }
    return (KE_MSG_CONSUMED);
}

__STATIC int sibles_name_handler(ke_msg_id_t const msgid,  struct sibles_value const *param,
                                 ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    struct sibles_env_tag *sibles_env = PRF_ENV_GET(SIBLES, sibles);

    app_env.dev_name_len = param->length;
    memcpy(app_env.dev_name, param->data, param->length);
    {
        struct sibles_value_ack *rsp;
#ifndef PLF_UART2_OVERMBOX
        rsp = KE_MSG_ALLOC(SIBLES_CMD_RSP, prf_src_task_get(&(sibles_env->prf_env), 0), TASK_ID_APP, sibles_value_ack);
#else
        rsp = KE_MSG_ALLOC(SIBLES_CMD_RSP, gapm_get_AHI_task_id(), dest_id, sibles_value_ack);
#endif
        rsp->hdl = 0;   // Not used.
        rsp->status = 0;
        sifli_mbox_bcpu2hcpu((uint8_t *)rsp);
    }
    return (KE_MSG_CONSUMED);
}

uint8_t sibles_get_adv_data(uint8_t *data)
{
    struct sibles_env_tag *sibles_env = PRF_ENV_GET(SIBLES, sibles);
    uint8_t r = 0;

    if (sibles_env->adv_data)
    {
        memcpy(data, sibles_env->adv_data->data, sibles_env->adv_data->length);
        r = sibles_env->adv_data->length;
    }
    return r;
}

uint8_t sibles_get_scan_rsp(uint8_t *data)
{
    struct sibles_env_tag *sibles_env = PRF_ENV_GET(SIBLES, sibles);
    uint8_t r = 0;

    if (sibles_env->scan_rsp)
    {
        memcpy(data, sibles_env->scan_rsp->data, sibles_env->scan_rsp->length);
        r = sibles_env->scan_rsp->length;
    }
    return r;
}

__STATIC int silbes_hci_send_handler(ke_msg_id_t const msgid,
                                     struct gattc_event_ind const *param,
                                     ke_task_id_t const dest_id,
                                     ke_task_id_t const src_id)
{
    struct sibles_env_tag *env = PRF_ENV_GET(SIBLES, sibles);
    struct gattc_event_ind *ind = KE_MSG_ALLOC_DYN(SIBLES_EVENT_IND,
                                  gapm_get_AHI_task_id(), dest_id,
                                  gattc_event_ind, param->length);
    ind->type = param->type;
    ind->length = param->length;
    ind->handle = param->handle;
    memcpy(ind->value, param->value, param->length);
    sifli_mbox_bcpu2hcpu((uint8_t *)ind);
    return (KE_MSG_CONSUMED);
}

__STATIC int silbes_wlan_coex_enable_req(ke_msg_id_t const msgid,
        struct sibles_wlan_coex_enable_req_t const *param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
    // send HCI command
    struct hci_wlan_coex_set_filter_channel_cmd_t *wlan_enable = KE_MSG_ALLOC(HCI_COMMAND, 0,
            0xFC83, hci_wlan_coex_set_filter_channel_cmd_t);
    //wlan_enable->enable = param->enable;
    /* send the message */
    hci_send_2_controller(wlan_enable);

    return (KE_MSG_CONSUMED);
}

__STATIC int silbes_trc_cfg_req(ke_msg_id_t const msgid,
                                struct sibles_trc_cfg_req_t const *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id)
{
    // send HCI command
    uint8_t idx = TASK_IDX_GET(dest_id);
    uint16_t offset = idx == 0 ? 0x100 : 0;

    // set into local host
#if (TRACER_PRESENT)
    dbg_trc_cfg_received_via_hci(param->mask);
#endif

    // to avoid 52 crash
    if (idx == 0)
    {
        struct hci_dbg_set_trc_cfg_cmd_t *cfg = KE_MSG_ALLOC(HCI_COMMAND, 0,
                                                HCI_DBG_SET_TRC_CFG_CMD_OPCODE - offset, hci_dbg_set_trc_cfg_cmd_t);
        //wlan_enable->enable = param->enable;
        cfg->cfg = param->mask;
        /* send the message */
        hci_send_2_controller(cfg);
    }
    return (KE_MSG_CONSUMED);
}

extern struct gapm_env_tag gapm_env;

__STATIC int sibles_set_static_random_addr_req_handler(ke_msg_id_t const msgid,
        struct sibles_random_addr const *param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
    // Check if address is a static random address
    if (param->addr[5] & 0xC0)
    {
        GAPM_F_SET(gapm_env.cfg_flags, ADDR_TYPE, 1);
        memcpy(&(gapm_env.addr), &param->addr[0], GAP_BD_ADDR_LEN);
    }
    else
    {
        GAPM_F_SET(gapm_env.cfg_flags, ADDR_TYPE, 0);
    }
    return (KE_MSG_CONSUMED);
}


__STATIC int  sibles_enable_dbg_handler(ke_msg_id_t const msgid,
                                        uint8_t const *param,
                                        ke_task_id_t const dest_id,
                                        ke_task_id_t const src_id)
{
    uint8_t idx = TASK_IDX_GET(dest_id);
    uint16_t offset = idx == 0 ? 0x100 : 0;

    // Check if address is a static random address
    /* send query LE event mask */
    struct hci_dbg_reserved_32_cmd_t *dbg_cfg = KE_MSG_ALLOC(HCI_COMMAND, 0, HCI_DBG_RESERVED1_CMD_OPCODE - offset, hci_dbg_reserved_32_cmd_t);

    dbg_cfg->reserved[0] = *param++;
    dbg_cfg->reserved[1] = *param++;
    dbg_cfg->reserved[2] = *param++;
    dbg_cfg->reserved[3] = *param++;
    /* send the event mask */
    hci_send_2_controller(dbg_cfg);
    return (KE_MSG_CONSUMED);
}



__STATIC int  sibles_ch_bd_handler(ke_msg_id_t const msgid,
                                   struct sibles_ch_bd_addr_t const *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id)
{
    uint8_t idx = TASK_IDX_GET(dest_id);
    uint16_t offset = idx == 0 ? 0x100 : 0;

    struct hci_dbg_reserved_64_cmd_t *dbg_cfg = KE_MSG_ALLOC(HCI_COMMAND, 0, HCI_DBG_RESERVED6_CMD_OPCODE - offset, hci_dbg_reserved_64_cmd_t);

    dbg_cfg->reserved[0] = param->addr_type;
    dbg_cfg->reserved[1] = param->addr_method;
    if (param->addr_method == 2)
    {
        memcpy(&dbg_cfg->reserved[2], &param->addr, BD_ADDR_LEN);
    }
    /* send the event mask */
    hci_send_2_controller(dbg_cfg);
    return (KE_MSG_CONSUMED);
}


__STATIC int
hci_cmd_comp_event_handler(ke_msg_id_t const msgid,
                           void const *param,
                           ke_task_id_t const dest_id,
                           ke_task_id_t const src_id)
{
    int return_status = KE_MSG_CONSUMED;
#if (CFG_BT_HOST)
    if (src_id == HCI_DBG_RESERVED6_EXT_CMD_OPCODE
            || src_id == HCI_DBG_RESERVED6_CMD_OPCODE)
    {
        struct hci_basic_cmd_cmp_evt const *evt = (struct hci_basic_cmd_cmp_evt const *)param;
        struct sibles_ch_bd_addr_rsp_t *rsp =  KE_MSG_ALLOC(SIBLES_CH_BD_ADDR_RSP,
                                               gapm_get_AHI_task_id(), dest_id, sibles_ch_bd_addr_rsp_t);
        rsp->status = evt->status;
        sifli_mbox_bcpu2hcpu((uint8_t *)rsp);
    }
#endif // CFG_BT_HOST
    return return_status;
}


/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// Default State handlers definition
KE_MSG_HANDLER_TAB(sibles)
{
    // Peer try to read
    {GATTC_READ_REQ_IND, (ke_msg_func_t)gattc_read_req_ind_handler},
    {SIBLES_SET_VALUE_REQ, (ke_msg_func_t)sibles_set_value_req_handler},
    {SIBLES_VALUE_REQ_CFM, (ke_msg_func_t)sibles_value_req_cfm_handler},

    // Peer try to write
    {GATTC_WRITE_REQ_IND, (ke_msg_func_t)gattc_write_req_ind_handler},
    {GATTC_ATT_INFO_REQ_IND, (ke_msg_func_t) gattc_att_info_req_ind_handler},
    {SIBLES_VALUE_WRITE_CFM, (ke_msg_func_t)sibles_value_write_cfm_handler},

    // Write to peer
    {GATTC_CMP_EVT, (ke_msg_func_t)gattc_cmp_evt_handler},
    {SIBLES_VALUE_IND_REQ, (ke_msg_func_t)sibles_value_ind_req_handler},

    // GAP commands
    {SIBLES_ADV_DATA_REQ, (ke_msg_func_t)sibles_adv_data_handler},
    {SIBLES_SCAN_RSP_REQ, (ke_msg_func_t)sibles_scan_rsp_handler},
    {SIBLES_ADV_CMD_REQ, (ke_msg_func_t)sibles_adv_cmd_handler},
    {SIBLES_NAME_REQ, (ke_msg_func_t)sibles_name_handler},

    {SIBLES_SVC_REG_REQ, (ke_msg_func_t)sibles_svc_reg_req_handler},
    {SIBLES_SVC_REG128_REQ, (ke_msg_func_t)sibles_svc_reg128_req_handler},
    {SIBLES_UPDATE_ATT_PERM_REQ, (ke_msg_func_t)sibles_att_update_handler},

#if (BLE_ATTC)
    {SIBLES_SVC_SEARCH_REQ, (ke_msg_func_t)silbes_svc_search_req_handler},
    {GATTC_DISC_SVC_IND, (ke_msg_func_t)silbes_disc_svc_ind_handler},
    {GATTC_DISC_CHAR_IND, (ke_msg_func_t)silbes_disc_char_ind_handler},
    {GATTC_DISC_CHAR_DESC_IND, (ke_msg_func_t)silbes_disc_char_desc_ind_handler},

    {SIBLES_REGISTER_NOTIFY_REQ, (ke_msg_func_t)silbes_register_peer_svc_handler},
    {SIBLES_UNREGISTER_NOTIFY_REQ, (ke_msg_func_t)silbes_unregister_peer_svc_handler},
    {SIBLES_VALUE_WRITE_REQ, (ke_msg_func_t)silbes_value_write_req_handler},
    {GATTC_EVENT_IND, (ke_msg_func_t)silbes_event_ind_handler},
    {GATTC_EVENT_REQ_IND, (ke_msg_func_t)silbes_event_ind_handler},
    {SIBLES_VALUE_READ_REQ, (ke_msg_func_t)silbes_value_read_req_handler},
    {GATTC_READ_IND, (ke_msg_func_t)silbes_value_read_ind_handler},
#endif // BLE_ATTC
    {SIBLES_WLAN_COEX_ENABLE_REQ, (ke_msg_func_t)silbes_wlan_coex_enable_req},
    {SIBLES_TRC_CFG_REQ, (ke_msg_func_t)silbes_trc_cfg_req},
    {SIBLES_VALUE_NTF_IND, (ke_msg_func_t)sibles_value_ntf_ind_handler},
    {SIBLES_SET_STATIC_RANDOM_ADDR_REQ, (ke_msg_func_t)sibles_set_static_random_addr_req_handler},
    {SIBLES_ENABLE_DBG, (ke_msg_func_t)sibles_enable_dbg_handler},
#if (CFG_BT_HOST)
    {SIBLES_CH_BD_ADDR, (ke_msg_func_t)sibles_ch_bd_handler},
#endif //CFG_BT_HOST

    {HCI_CMD_CMP_EVENT, (ke_msg_func_t)hci_cmd_comp_event_handler},
};

void _sibles_task_init(struct ke_task_desc *task_desc)
{
    // Get the address of the environment
    struct sibles_env_tag *env = PRF_ENV_GET(SIBLES, sibles);

    task_desc->msg_handler_tab = sibles_msg_handler_tab;
    task_desc->msg_cnt         = ARRAY_LEN(sibles_msg_handler_tab);
    task_desc->state           = env->state;
    task_desc->idx_max         = SIBLES_IDX_MAX;
}

#endif //BLE_SIBLE_SERVER
#endif //BLE_APP_PRESENT

/// @} SIBLESTASK
