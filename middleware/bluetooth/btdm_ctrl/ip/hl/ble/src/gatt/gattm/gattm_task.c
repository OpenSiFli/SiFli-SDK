/**
 ****************************************************************************************
 *
 * @file gattm_task.c
 *
 * @brief Generic Attribute Profile Manager Task implementation.
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup GATTMTASK
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
/* software configuration */
#include "rwip_config.h"

#if (BLE_CENTRAL || BLE_PERIPHERAL)
#include "co_utils.h"

/* GATTM module */
#include "gattm.h"
#include "gattm_int.h"
/* GATTC task */
#include "gattc_task.h"
/* GATTC module */
#include "gattc.h"
/* GAP module */
#include "gap.h"
/* ATTM API */
#include "attm.h"
/* GAPM */
#include "gapm.h"
/* ATTM DB*/
#include "attm.h"
/*
 * Defines
 ****************************************************************************************
 */
/// Check if GATTM message id is a request or a response
#define GATTM_IS_REQUEST(msgid) ((((msgid) - GATTM_ADD_SVC_REQ) % 2) == 0)

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Default messages handler for GATTM task
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 *
 ****************************************************************************************
 */
__STATIC int gattm_default_msg_handler(ke_msg_id_t const msgid, void *param,
                                       ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    // Check if it's a GATTM message
    if (MSG_T(msgid) == TASK_ID_GATTM)
    {
        // prepare unknown message indication
        struct gattm_unknown_msg_ind *p_ind = KE_MSG_ALLOC(GATTM_UNKNOWN_MSG_IND,
                                              src_id, dest_id, gattm_unknown_msg_ind);

        p_ind->unknown_msg_id = msgid;

        // send event
        ke_msg_send(p_ind);
    }

    /* message is consumed */
    return (KE_MSG_CONSUMED);
}

#if (BLE_EXT_ATT_DB)
/**
***************************************************************************************
* @brief Add Service into database request. This message shall be used to allocate a
* buffer that will be use to describe a service in attribute database. Then requester
* shall describe database using GATTM_ADD_ATTRIBUTE_REQ message.
*
* If start handle is set to zero (invalid attribute handle), ATTM task automatically
* search free handle block matching with number of attributes to reserve. Else, according
* to start handle, ATTM task checks if attributes to reserve are not overlapping part of
* existing database.
*
* Finally it allocates buffer that:
*  - Describe the database (Block insert in database linked list sorted by start handle)
*  - Contains attributes configurations and their values.
*
* @param[in] msgid     Id of the message received.
* @param[in] param     Pointer to the parameters of the message.
* @param[in] dest_id   ID of the receiving task instance (TASK_GATTM).
* @param[in] src_id    ID of the sending task instance.
*
* @return If the message was consumed or not.
****************************************************************************************
*/
__STATIC int gattm_add_svc_req_handler(ke_msg_id_t const msgid, struct gattm_add_svc_req *param,
                                       ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    struct gattm_add_svc_rsp *add_svc_rsp =
        KE_MSG_ALLOC(GATTM_ADD_SVC_RSP, src_id, dest_id, gattm_add_svc_rsp);

#if(AHI_TL_SUPPORT)
    // for external application which AHI, update task Number from task ID
    if (KE_TYPE_GET(param->svc_desc.task_id) == TASK_ID_AHI)
    {
        param->svc_desc.task_id = gapm_get_task_from_id(param->svc_desc.task_id);
    }
#endif // (AHI_TL_SUPPORT)

    /* Allocate Service structure */
    add_svc_rsp->status = attmdb_add_service(&(param->svc_desc));
    /* update start handle */
    add_svc_rsp->start_hdl = param->svc_desc.start_hdl;

    /* send command response message. */
    ke_msg_send(add_svc_rsp);

    /* message is consumed */
    return (KE_MSG_CONSUMED);
}


/**
***************************************************************************************
* @brief Retrieve service attribute permissions:
*
*   - PERM_SVC_NONE    : No specific permissions
*   - PERM_SVC_UNAUTH  : Just work pairing required
*   - PERM_SVC_AUTH    : MITM pairing required
*   - PERM_SVC_AUTZ    : Authorize mode required
*   - PERM_SVC_HIDE    : Service Hidden (cannot be browsed, read, write)
*   - PERM_SVC_EKS     : Specific encryption key size required
*   - PERM_SVC_DISABLE : Service Disable (can be browsed but cannot be used)
*
* @param[in] msgid     Id of the message received.
* @param[in] param     Pointer to the parameters of the message.
* @param[in] dest_id   ID of the receiving task instance (TASK_GATTM).
* @param[in] src_id    ID of the sending task instance.
*
* @return If the message was consumed or not.
****************************************************************************************
*/
__STATIC int
gattm_svc_get_permission_req_handler(ke_msg_id_t const msgid, struct gattm_svc_get_permission_req *param,
                                     ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    struct gattm_svc_get_permission_rsp *svc_get_perm_rsp =
        KE_MSG_ALLOC(GATTM_SVC_GET_PERMISSION_RSP, src_id, dest_id, gattm_svc_get_permission_rsp);

    /* Get service permissions */
    svc_get_perm_rsp->status =
        attm_svc_get_permission(param->start_hdl, &(svc_get_perm_rsp->perm));

    /* update handle */
    svc_get_perm_rsp->start_hdl = param->start_hdl;

    /* send command response message. */
    ke_msg_send(svc_get_perm_rsp);

    /* message is consumed */
    return (KE_MSG_CONSUMED);
}

/**
***************************************************************************************
* @brief Modify service attribute permissions:
*
*   - PERM_SVC_NONE    : No specific permissions
*   - PERM_SVC_UNAUTH  : Just work pairing required
*   - PERM_SVC_AUTH    : MITM pairing required
*   - PERM_SVC_AUTZ    : Authorize mode required
*   - PERM_SVC_HIDE    : Service Hidden (cannot be browsed, read, write)
*   - PERM_SVC_EKS     : Specific encryption key size required
*   - PERM_SVC_DISABLE : Service Disable (can be browsed but cannot be used)
*
* @param[in] msgid     Id of the message received.
* @param[in] param     Pointer to the parameters of the message.
* @param[in] dest_id   ID of the receiving task instance (TASK_GATTM).
* @param[in] src_id    ID of the sending task instance.
*
* @return If the message was consumed or not.
****************************************************************************************
*/
__STATIC int
gattm_svc_set_permission_req_handler(ke_msg_id_t const msgid, struct gattm_svc_set_permission_req *param,
                                     ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    struct gattm_svc_set_permission_rsp *svc_set_perm_rsp =
        KE_MSG_ALLOC(GATTM_SVC_SET_PERMISSION_RSP, src_id, dest_id, gattm_svc_set_permission_rsp);

    /* Modify service attribute permissions */
    svc_set_perm_rsp->status = attm_svc_set_permission(param->start_hdl, param->perm);

    /* update handle */
    svc_set_perm_rsp->start_hdl = param->start_hdl;

    /* send command response message. */
    ke_msg_send(svc_set_perm_rsp);

    /* message is consumed */
    return (KE_MSG_CONSUMED);
}

/**
***************************************************************************************
* @brief  Retrieve attribute permissions
*
* @param[in] msgid     Id of the message received.
* @param[in] param     Pointer to the parameters of the message.
* @param[in] dest_id   ID of the receiving task instance (TASK_GATTM).
* @param[in] src_id    ID of the sending task instance.
*
* @return If the message was consumed or not.
****************************************************************************************
*/
__STATIC int
gattm_att_get_permission_req_handler(ke_msg_id_t const msgid, struct gattm_att_get_permission_req *param,
                                     ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    struct attm_elmt elmt = ATT_ELEMT_INIT;
    struct gattm_att_get_permission_rsp *get_att_perm_rsp =
        KE_MSG_ALLOC(GATTM_ATT_GET_PERMISSION_RSP, src_id, dest_id, gattm_att_get_permission_rsp);

    /* Retrieve attribute permissions */
    get_att_perm_rsp->status = attmdb_att_get_permission(param->handle, &(get_att_perm_rsp->perm), PERM_MASK_ALL, 0, &elmt);
    get_att_perm_rsp->ext_perm  = (elmt.info.att->info.max_length & PERM_MASK_EKS);

    /* update handle */
    get_att_perm_rsp->handle = param->handle;

    /* send command response message. */
    ke_msg_send(get_att_perm_rsp);

    /* message is consumed */
    return (KE_MSG_CONSUMED);
}

/**
***************************************************************************************
* @brief Modify attribute permissions
*
* @param[in] msgid     Id of the message received.
* @param[in] param     Pointer to the parameters of the message.
* @param[in] dest_id   ID of the receiving task instance (TASK_GATTM).
* @param[in] src_id    ID of the sending task instance.
*
* @return If the message was consumed or not.
****************************************************************************************
*/
__STATIC int
gattm_att_set_permission_req_handler(ke_msg_id_t const msgid, struct gattm_att_set_permission_req *param,
                                     ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    struct gattm_att_set_permission_rsp *set_att_perm_rsp =
        KE_MSG_ALLOC(GATTM_ATT_SET_PERMISSION_RSP, src_id, dest_id, gattm_att_set_permission_rsp);

    /* Modify attribute permissions */
    set_att_perm_rsp->status = attm_att_set_permission(param->handle, param->perm, param->ext_perm);

    /* update handle */
    set_att_perm_rsp->handle = param->handle;

    /* send command response message. */
    ke_msg_send(set_att_perm_rsp);

    /* message is consumed */
    return (KE_MSG_CONSUMED);
}

/**
***************************************************************************************
* @brief  Get attribute value
*
* @param[in] msgid     Id of the message received.
* @param[in] param     Pointer to the parameters of the message.
* @param[in] dest_id   ID of the receiving task instance (TASK_GATTM).
* @param[in] src_id    ID of the sending task instance.
*
* @return If the message was consumed or not.
****************************************************************************************
*/
__STATIC int gattm_att_get_value_req_handler(ke_msg_id_t const msgid, struct gattm_att_get_value_req *param,
        ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint16_t length = 0;
    uint8_t *value;
    uint8_t status;
    struct gattm_att_get_value_rsp *get_att_val_rsp;

    /*  Retrieve attribute value informations. */
    status = attm_get_value(param->handle, &length, &value);

    get_att_val_rsp = KE_MSG_ALLOC_DYN(GATTM_ATT_GET_VALUE_RSP, src_id, dest_id, gattm_att_get_value_rsp, length);

    /* update length */
    get_att_val_rsp->length = length;

    /* copy attribute value */
    memcpy(&(get_att_val_rsp->value[0]), value, length);

    /* update status */
    get_att_val_rsp->status = status;
    /* update handle */
    get_att_val_rsp->handle = param->handle;

    /* send command response message. */
    ke_msg_send(get_att_val_rsp);

    /* message is consumed */
    return (KE_MSG_CONSUMED);
}

/**
***************************************************************************************
* @brief Modify attribute value
*
* @param[in] msgid     Id of the message received.
* @param[in] param     Pointl2cc_data_send_rsp_handlerer to the parameters of the message.
* @param[in] dest_id   ID of the receiving task instance (TASK_GATTM).
* @param[in] src_id    ID of the sending task instance.
*
* @return If the message was consumed or not.
****************************************************************************************
*/
__STATIC int
gattm_att_set_value_req_handler(ke_msg_id_t const msgid, struct gattm_att_set_value_req *param,
                                ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    struct gattm_att_set_value_rsp *set_att_val_rsp =
        KE_MSG_ALLOC(GATTM_ATT_SET_VALUE_RSP, src_id, dest_id, gattm_att_set_value_rsp);

    /* Modify attribute value */
    set_att_val_rsp->status =
        attm_att_set_value(param->handle, param->length, 0, &(param->value[0]));

    /* update handle */
    set_att_val_rsp->handle = param->handle;

    /* send command response message. */
    ke_msg_send(set_att_val_rsp);

    /* message is consumed */
    return (KE_MSG_CONSUMED);
}

#if (RW_DEBUG)
/**
***************************************************************************************
* @brief DEBUG ONLY: Destruct Attribute database
*
* @param[in] msgid     Id of the message received.
* @param[in] param     Pointer to the parameters of the message.
* @param[in] dest_id   ID of the receiving task instance (TASK_GATTM).
* @param[in] src_id    ID of the sending task instance.
*
* @return If the message was consumed or not.
****************************************************************************************
*/
__STATIC int gattm_destroy_db_req_handler(ke_msg_id_t const msgid, struct gattm_destroy_db_req *param,
        ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    struct gattm_destroy_db_rsp *destroy_db_rsp =
        KE_MSG_ALLOC(GATTM_DESTROY_DB_RSP, src_id, dest_id, gattm_destroy_db_rsp);

    /* Destroy database */
    attmdb_destroy();

    /* Modify gap and gatt service start handle */
    gapm_set_svc_start_hdl(param->gap_hdl);
    gattm_env.svc_start_hdl = param->gatt_hdl;

    /* send command response message. */
    destroy_db_rsp->status = ATT_ERR_NO_ERROR;
    ke_msg_send(destroy_db_rsp);

    /* message is consumed */
    return (KE_MSG_CONSUMED);
}

/**
***************************************************************************************
* @brief DEBUG ONLY: Get information about services in attribute database
*
* @param[in] msgid     Id of the message received.
* @param[in] param     Pointer to the parameters of the message.
* @param[in] dest_id   ID of the receiving task instance (TASK_GATTM).
* @param[in] src_id    ID of the sending task instance.
*
* @return If the message was consumed or not.
****************************************************************************************
*/
__STATIC int gattm_svc_get_list_req_handler(ke_msg_id_t const msgid, void *param,
        ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint8_t nb_svc;
    struct gattm_svc_get_list_rsp *get_svc_list_rsp;

    /* retrieve number of services. */
    nb_svc = attmdb_get_nb_svc();

    /* allocate response message */
    get_svc_list_rsp = KE_MSG_ALLOC_DYN(GATTM_SVC_GET_LIST_RSP, src_id, dest_id, gattm_svc_get_list_rsp,
                                        (nb_svc * sizeof(struct gattm_svc_info)));

    /* set number of services */
    get_svc_list_rsp->nb_svc = nb_svc;
    get_svc_list_rsp->status = ATT_ERR_NO_ERROR;

    /* retrieve services informations */
    attmdb_get_svc_info(&(get_svc_list_rsp->svc[0]));

    /* send command response message. */
    ke_msg_send(get_svc_list_rsp);

    /* message is consumed */
    return (KE_MSG_CONSUMED);
}

/**
 ***************************************************************************************
 * @brief DEBUG ONLY: Retrieve information of attribute in DB
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GATTM).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC int gattm_att_get_info_req_handler(ke_msg_id_t const msgid, struct gattm_att_get_info_req *param,
        ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    struct attm_elmt elmt = ATT_ELEMT_INIT;
    struct gattm_att_get_info_rsp *att_info_rsp = KE_MSG_ALLOC(GATTM_ATT_GET_INFO_RSP,
            src_id, dest_id, gattm_att_get_info_rsp);
    att_info_rsp->handle = param->handle;
    // Retrieve UUID
    att_info_rsp->status = attmdb_get_attribute(param->handle, &elmt);

    if (att_info_rsp->status == ATT_ERR_NO_ERROR)
    {
        attmdb_get_uuid(&elmt, &(att_info_rsp->uuid_len), &(att_info_rsp->uuid[0]), false, false);
        // Retrieve permission
        attmdb_att_get_permission(param->handle, &(att_info_rsp->perm), PERM_MASK_ALL, 0, &elmt);

        att_info_rsp->ext_perm  = PERM_GET(elmt.info.att->info.max_length, EKS);
    }

    /* send command response message. */
    ke_msg_send(att_info_rsp);

    /* message is consumed */
    return (KE_MSG_CONSUMED);
}
#endif /* (RW_DEBUG) */
#endif // (BLE_EXT_ATT_DB)



/// The default message handlers
KE_MSG_HANDLER_TAB(gattm)
{
    // note: first message is latest message checked by kernel so default is put on top.
    { KE_MSG_DEFAULT_HANDLER, (ke_msg_func_t) gattm_default_msg_handler },

#if (BLE_EXT_ATT_DB)
    { GATTM_ADD_SVC_REQ, (ke_msg_func_t) gattm_add_svc_req_handler },
    { GATTM_SVC_GET_PERMISSION_REQ, (ke_msg_func_t) gattm_svc_get_permission_req_handler},
    { GATTM_SVC_SET_PERMISSION_REQ, (ke_msg_func_t) gattm_svc_set_permission_req_handler},
    { GATTM_ATT_GET_PERMISSION_REQ, (ke_msg_func_t) gattm_att_get_permission_req_handler},
    { GATTM_ATT_SET_PERMISSION_REQ, (ke_msg_func_t) gattm_att_set_permission_req_handler},
    { GATTM_ATT_GET_VALUE_REQ, (ke_msg_func_t) gattm_att_get_value_req_handler},
    { GATTM_ATT_SET_VALUE_REQ, (ke_msg_func_t) gattm_att_set_value_req_handler},
#if (RW_DEBUG)
    { GATTM_DESTROY_DB_REQ, (ke_msg_func_t) gattm_destroy_db_req_handler},
    { GATTM_SVC_GET_LIST_REQ, (ke_msg_func_t) gattm_svc_get_list_req_handler},
    { GATTM_ATT_GET_INFO_REQ, (ke_msg_func_t) gattm_att_get_info_req_handler},
#endif /* (RW_DEBUG) */
#endif /* (BLE_EXT_ATT_DB) */
};

/// GATT Manager task instance.
ke_state_t gattm_state[GATTM_IDX_MAX];

/// GATTM task descriptor
const struct ke_task_desc TASK_DESC_GATTM = {gattm_msg_handler_tab, gattm_state, GATTM_IDX_MAX, ARRAY_LEN(gattm_msg_handler_tab)};



#endif /* #if (BLE_CENTRAL || BLE_PERIPHERAL) */
/// @} GATTMTASK
