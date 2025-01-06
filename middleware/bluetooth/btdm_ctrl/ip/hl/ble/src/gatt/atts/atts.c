/**
 ****************************************************************************************
 *
 * @file atts.c
 *
 * @brief Attribute Server implementation.
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup ATTS
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"

#if (BLE_CENTRAL || BLE_PERIPHERAL)
#include <stdlib.h>
#include "atts.h"
#include "co_error.h"
#include "co_utils.h"
#include "co_math.h"

#include "ke_mem.h"
#include "ke_timer.h"

#include "attm_db.h" // Access to the internal database is required

#include "gap.h"
#include "gapm.h"
#include "gapc.h"

#include "gattc_task.h"
#include "../gattc/gattc_int.h" // Access to Internal API required

#include "l2cc.h"
#include "l2cm.h"
#include "attm.h"

#include <stddef.h>
#if (BLE_ATTS)


/*
 * DEFINES
 ****************************************************************************************
 */


/// Access operation to attribute element
enum
{
    /// Read type of access to element
    ATT_READ_ACCESS = 0x00,
    /// Write request type of access to element
    ATT_WRITE_ACCESS,
    /// Write command type of access to element
    ATT_WRITE_COMMAND_ACCESS,
    /// Write signed type of access to element
    ATT_WRITE_SIGNED_ACCESS,
    /// Notify type of access to element
    ATT_NOTIFY_ACCESS,
    /// Indication type of access to element
    ATT_INDICATE_ACCESS,

    ATT_MAX_ACCESS
};
/*
 * MACRO DEFINITIONS
 ****************************************************************************************
 */


/// Allocate a PDU packet for a specific PDU type.
#define ATTS_ALLOCATE_PDU(conidx, opcode, pdu_type, value_len)\
    L2CC_ATT_PDU_ALLOC_DYN(conidx, opcode, KE_BUILD_ID(TASK_GATTC, conidx), pdu_type, value_len)

/// retrieve remaining length parameter
#define ATTS_REMAINING_LEN(pdu)\
    ((uint16_t*) ((uint8_t*) pdu - (offsetof(struct l2cc_pdu, data))))

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// temporary data for find info response
struct atts_find_info_rsp
{
    /// List header for chaining
    struct co_list_hdr hdr;
    /// attribute handle
    uint16_t handle;
    /// attribute UUID length
    uint8_t uuid_len;
    /// Attribute UUID
    uint8_t uuid[ATT_UUID_128_LEN];
};

/// temporary data for find by type response
struct atts_find_by_type_rsp
{
    /// List header for chaining
    struct co_list_hdr hdr;
    /// start handle
    uint16_t shdl;
    /// end handle
    uint16_t ehdl;
};

/// temporary data for read by type response
struct atts_rd_by_type_rsp
{
    /// List header for chaining
    struct co_list_hdr hdr;
    /// value handle
    uint16_t  handle;
    /// value length
    uint8_t   length;
    /// A set of two or more values.
    uint8_t   value[__ARRAY_EMPTY];
};

/// temporary data for read by group type response
struct atts_rd_by_grp_type_rsp
{
    /// List header for chaining
    struct co_list_hdr hdr;
    /// start handle
    uint16_t  shdl;
    /// end handle
    uint16_t  ehdl;
    /// value length
    uint8_t   length;
    /// A set of two or more values.
    uint8_t   value[__ARRAY_EMPTY];
};

/// temporary data for read multiple response
struct atts_rd_mult_rsp
{
    /// List header for chaining
    struct co_list_hdr hdr;
    /// value length
    uint16_t   length;
    /// A set of two or more values.
    uint8_t    value[__ARRAY_EMPTY];
};


/// Prepare write data part structure - mapped to the kernel message for optimization of data copy
struct atts_prep_data
{
    /// List header for chaining
    struct co_list_hdr hdr;
    /// total length of data present in prepare write, used to check that size limit not exceeded
    uint16_t total_len;
    /// current length of the attribute
    uint16_t cur_len;
    /// data offset
    uint16_t offset;
    /// length of the partial data
    uint16_t len;
    /// PDU information
    uint8_t  value[__ARRAY_EMPTY];
};


/// Attribute right info
struct atts_right
{
    /// Right mode Mask
    uint16_t mode_mask;
    /// Right Permission Mask
    uint16_t perm_mask;
    /// Default error status is right disabled
    uint8_t err_status;
};

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// Attribute Right info according to access
static const struct atts_right atts_rights[] =
{
    [ATT_READ_ACCESS]          = {PERM_MASK_RD,            PERM_MASK_RP, ATT_ERR_READ_NOT_PERMITTED},
    [ATT_WRITE_ACCESS]         = {PERM_MASK_WRITE_REQ,     PERM_MASK_WP, ATT_ERR_WRITE_NOT_PERMITTED},
    [ATT_WRITE_COMMAND_ACCESS] = {PERM_MASK_WRITE_COMMAND, PERM_MASK_WP, ATT_ERR_WRITE_NOT_PERMITTED},
    [ATT_WRITE_SIGNED_ACCESS]  = {PERM_MASK_WRITE_SIGNED,  PERM_MASK_WP, ATT_ERR_WRITE_NOT_PERMITTED},
    [ATT_NOTIFY_ACCESS]        = {PERM_MASK_NTF,           PERM_MASK_NP, ATT_ERR_REQUEST_NOT_SUPPORTED},
    [ATT_INDICATE_ACCESS]      = {PERM_MASK_IND,           PERM_MASK_IP, ATT_ERR_REQUEST_NOT_SUPPORTED},
};

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @brief Retrieve the destination task of event according to attribute handle
 *
* @param[in] conidx   connection index
* @param[in] handle   The handle of the attribute
*
* @return The destination task
 ****************************************************************************************
 */
static uint16_t atts_get_dest_task(uint8_t conidx, uint16_t handle)
{
    struct attm_svc *svc = attmdb_get_service(handle);
    ASSERT_INFO(svc != NULL, conidx, handle);
    // retrieve destination task identifier
    uint16_t dest = svc->svc.task_id;
    if (PERM_GET(svc->svc.perm, SVC_MI))
    {
        dest = KE_BUILD_ID(dest, conidx);
    }
    return dest;
}

/**
 ****************************************************************************************
 * @brief Checks if the length and offset passed as parameters are compliant with the
 * specified handle properties
 *
 * @param[in] elmt pointer to the attribute to be checked
 * @param[in] offset Offset to be verified
 * @param[in] length Length to be verified
 *
 * @return @ref ATT_ERR_NO_ERROR if length and offset are valid, otherwise the relevant
 * ATT error code (@ref ATT_ERR_INVALID_OFFSET or @ref ATT_ERR_INVALID_ATTRIBUTE_VAL_LEN)
 *
 *****************************************************************************************
 */
static uint8_t atts_check_length_and_offset(struct attm_elmt *elmt, uint16_t length, uint16_t offset)
{
    uint16_t max_length = 0;
    uint8_t  status = attmdb_get_max_len(elmt, &max_length);

    // If attribute handle is invalid, the handling will be done later so return no error
    if (status == ATT_ERR_NO_ERROR)
    {
        // Verify offset
        if (offset >= max_length)
            return ATT_ERR_INVALID_OFFSET;

        // Verify length to be updated
        if ((offset + length) > max_length)
            return ATT_ERR_INVALID_ATTRIBUTE_VAL_LEN;
    }

    return (status);
}


/**
 ****************************************************************************************
 * @brief Retrieve attribute and checks if a peer device has enough permission rights
 *        to perform the access to the specified attribute
 *
 * @param[in]  conidx   Index of the connection with the peer device
 * @param[in]  access   Type of access to be performed (read, write, indication/notify)
 * @param[in]  handle   Handle of the attribute to be accessed
 * @param[out] elmt Attribute pointer to return
 *
 * @return @ref ATT_ERR_NO_ERROR if access is permitted, otherwise the ATT error code to
 * be put in the error response frame
 *
 *****************************************************************************************
 */
static uint8_t atts_get_att_chk_perm(uint8_t conidx, uint8_t access, uint16_t handle,
                                     struct attm_elmt *elmt)
{
    uint8_t  status = ATT_ERR_NO_ERROR;
    bool eks_req = false;
    uint16_t sec_lvl;

    do
    {
        // ensure that access is valid
        if (access > ATT_MAX_ACCESS)
        {
            // Sanity check: No other access types are defined
            ASSERT_INFO(0, handle, conidx);
            status = ATT_ERR_REQUEST_NOT_SUPPORTED;
            break;
        }

        // perform discovery and retrieve permission info
        status = attmdb_att_get_permission(handle, &sec_lvl,
                                           atts_rights[access].mode_mask,
                                           atts_rights[access].perm_mask,
                                           elmt);

        switch (status)
        {
        // encryption key size required
        case ATT_ERR_INSUFF_ENC_KEY_SIZE:
        {
            eks_req = true;
            status = ATT_ERR_NO_ERROR;
        }
        break;
        // request not supported used default error code
        case ATT_ERR_REQUEST_NOT_SUPPORTED:
        {
            status = atts_rights[access].err_status;
        }
        break;
        default: /* Nothing to do */
            break;
        }


        // check if attribute not  found or service disabled
        if (status != ATT_ERR_NO_ERROR)
        {
            break;
        }

        // When attribute requires pairing
        if (sec_lvl >= PERM_RIGHT_UNAUTH)
        {
            // NOTE: the checking is done according to CSA3 - by priority
            if (gapc_lk_sec_lvl_get(conidx) == GAP_LK_NO_AUTH)
            {
                status = ATT_ERR_INSUFF_AUTHEN;
                break;
            }

            // check if link is encrypted and EKS must be in such good level
            // encryption is not for SIGNED WRITE ACCESS
            if (access != ATT_WRITE_SIGNED_ACCESS)
            {
                // encryption must be enabled
                if (!gapc_is_sec_set(conidx, GAPC_LK_ENCRYPTED))
                {
                    // check if LTK exchanged during pairing
                    if (gapc_is_sec_set(conidx, GAPC_LK_LTK_PRESENT))
                    {
                        status = ATT_ERR_INSUFF_ENC;
                    }
                    else
                    {
                        status = ATT_ERR_INSUFF_AUTHEN;
                    }
                    break;
                }

                //check encryption key size if attribute requires maximum
                if (eks_req && (gapc_enc_keysize_get(conidx) < ATT_SEC_ENC_KEY_SIZE))
                {
                    status = ATT_ERR_INSUFF_ENC_KEY_SIZE;
                    break;
                }
            }

            // check if connection has enough authentication level
            if (gapc_lk_sec_lvl_get(conidx) < sec_lvl)
            {
                status = ATT_ERR_INSUFF_AUTHEN;
                break;
            }
        }
    }
    while (0);

    return (status);
}

/**
 ****************************************************************************************
 * @brief Searches for the end handle of the attribute pointed by start_hdl.
 * @note This function does not check the validity of the start handle, which
 * has to be performed prior to call.
 *
 * @param[in] start_hdl  Handle of attribute for which the end handle is searched.
 * @param[in] elmt       Current element used to search end
 *
 * @return The end handle of the attribute. If the attribute is not a grouping attribute,
 * the start handle is returned. If no other attribute with the same type is found after
 * the start handle, last handle of database is returned.
 *
 ****************************************************************************************
 */
static uint16_t atts_find_end(uint16_t start_hdl, struct attm_elmt *elmt)
{
    // by default a descriptor contains only one handle.
    uint16_t end_hdl = start_hdl;

    // service group, just need to provide end handle
    if (elmt->service)
    {
        end_hdl = elmt->info.svc->end_hdl;
    }
    else
    {
        if (attmdb_uuid16_comp(elmt, ATT_DECL_CHARACTERISTIC))
        {
            struct attm_elmt next = ATT_ELEMT_INIT;
            struct attm_svc *svc = attmdb_get_service(start_hdl);
            bool found = false;
            start_hdl++;
            // Loop while end of characteristic not found
            while (!found && (start_hdl <= svc->svc.end_hdl) && (start_hdl > svc->svc.start_hdl))
            {
                // load next attribute (shall exist according to conditions, so no need
                // to check status)
                attmdb_get_attribute(start_hdl, &next);

                // check if it's end of attribute
                if ((attmdb_uuid16_comp(&next, ATT_DECL_CHARACTERISTIC))
                        || attmdb_uuid16_comp(&next, ATT_DECL_INCLUDE)
                        || attmdb_uuid16_comp(&next, ATT_INVALID_UUID))
                {
                    found = true;
                }
                else
                {
                    start_hdl++;
                }
            }

            // end of char found.
            if (found)
            {
                // previous handle was end of characteristic group
                end_hdl = start_hdl - 1;
            }
            // end of service.
            else
            {
                end_hdl = svc->svc.end_hdl;
            }
        }
    }

    return end_hdl;
}


/**
 ****************************************************************************************
 * @brief Retrieve value of the attribute. if ATT_ERR_REQUEST_NOT_SUPPORTED is triggered,
 * it means that a read request has been triggered to application in order to retrieve data
 *
 * Note: if data is present in read attribute cache, this function return data pointer
 * present in cache.
 *
 * @param[in]     conidx     Connection index
 * @param[in]     handle     Handle of attribute value
 * @param[out]    length     Current size of the attribute
 * @param[out]    value      Value pointer of the attribute
 *
 * @return Error code
 *   - @ref ATT_ERR_NO_ERROR               if an attribute was found (handle returned in start_hdl)
 *   - @ref ATT_ERR_REQUEST_NOT_SUPPORTED  If value is not available without external read request
 *   - Any other error that can be triggered by application
 ****************************************************************************************
 */
static uint8_t atts_get_value(uint8_t conidx, uint16_t handle,
                              uint16_t *length, uint8_t **value)
{
    uint8_t status = ATT_ERR_NO_ERROR;
    bool found = false;

    // check value is in cache
    if (gattc_env[conidx]->server.read_cache != NULL)
    {
        if (gattc_env[conidx]->server.read_cache->handle == handle)
        {
            found = true;
            status = gattc_env[conidx]->server.read_cache->status;

            // force error code
            if (status == ATT_ERR_REQUEST_NOT_SUPPORTED)
            {
                status = ATT_ERR_APP_ERROR;
            }

            if (status == ATT_ERR_NO_ERROR)
            {
                // retrieve data from cache
                *length = gattc_env[conidx]->server.read_cache->length;
                *value = gattc_env[conidx]->server.read_cache->value;
            }
            else
            {
                // cleanup cache
                atts_clear_read_cache(conidx);
            }
        }
        else
        {
            // cleanup cache
            atts_clear_read_cache(conidx);
        }
    }

    if (!found)
    {
        status = attm_get_value(handle, length, value);
    }

    if (status == ATT_ERR_REQUEST_NOT_SUPPORTED)
    {
        // send read to attribute value from upper layers
        struct gattc_read_req_ind *rd_req_ind = KE_MSG_ALLOC(GATTC_READ_REQ_IND,
                                                atts_get_dest_task(conidx, handle), KE_BUILD_ID(TASK_GATTC, conidx),
                                                gattc_read_req_ind);

        // Fill the message parameters
        rd_req_ind->handle   = handle;

        // And send the message
        ke_msg_send(rd_req_ind);
    }

    return (status);
}

/**
 ****************************************************************************************
 * @brief Searches for next element having the specified UUID in the attribute database,
 * starting from the start_hdl and up to the end_hdl, find end handle and retrieve attribute value.
 * If found, the start_hdl is updated with the handle of the found UUID.
 * @note This function does not check the validity of the start and stop handles, which
 * has to be performed prior to call.
 *
 * @param[in]     conidx     Connection index
 * @param[in|out] start_hdl  Pointer to the handle used as searching starting point. When
 *                           the searched UUID is found, this value is updated to
 *                           indicate the handle of the found UUID to the caller.
 * @param[in|out] end_hdl    Handle used as searching ending point, fill to characteristic end handle if found
 * @param[in]     uuid_len   UUID length
 * @param[in]     uuid       Pointer to the UUID pattern to find
 * @param[out]    elmt       Attribute pointer to return
 * @param[out]    length     Current size of the attribute
 * @param[out]    value      Value pointer of the attribute
 *
 *
 * @return Error code
 *   - @ref ATT_ERR_NO_ERROR               if an attribute was found (handle returned in start_hdl)
 *   - @ref ATT_ERR_ATTRIBUTE_NOT_FOUND    if no attribute was found.
 *   - @ref ATT_ERR_REQUEST_NOT_SUPPORTED  If value is not available without external read request
 *   - @see atts_get_att_chk_perm for other error codes
 *
 ****************************************************************************************
 */
static uint8_t atts_find_value_by_uuid(uint8_t conidx, uint16_t *start_hdl, uint16_t *end_hdl,
                                       uint8_t uuid_len, uint8_t *uuid, struct attm_elmt *elmt,
                                       uint16_t *length, uint8_t **value)
{
    uint8_t att_uuid_len;
    uint8_t att_uuid[ATT_UUID_128_LEN];
    bool found = false;
    uint8_t status = ATT_ERR_NO_ERROR;
    // store the start handle if attribute is not found
    uint16_t find_start_hdl = *start_hdl;

    do
    {
        status = attmdb_get_next_att(start_hdl, elmt);

        /* end of database - attribute not found */
        if (status != ATT_ERR_NO_ERROR)
        {
            *start_hdl = *end_hdl;
            break;
        }
        else if (*start_hdl <= *end_hdl)
        {
            // retrieve attribute UUID
            status = attmdb_get_uuid(elmt, &(att_uuid_len), att_uuid, false, false);
            ASSERT_INFO(status == ATT_ERR_NO_ERROR, *start_hdl, status);

            // compare UUIDs
            if (attm_uuid_comp(uuid, uuid_len, att_uuid, att_uuid_len))
            {
                found = true;
            }
            /* check next handle if no uuid found */
            else
            {
                (*start_hdl)++;
            }
        }
    }
    while (!found
            && (*start_hdl <= *end_hdl)
            && (*start_hdl != ATT_INVALID_HANDLE));


    if (!found)
    {
        status     = ATT_ERR_ATTRIBUTE_NOT_FOUND;
        // restore value of the start handle
        *start_hdl = find_start_hdl;
    }
    else
    {
        // retrieve attribute and check that all attributes within the range are readable
        status = atts_get_att_chk_perm(conidx, ATT_READ_ACCESS, *start_hdl, elmt);

        // Get the end handle of the found attribute
        *end_hdl = atts_find_end(*start_hdl, elmt);

        if (status == ATT_ERR_NO_ERROR)
        {
            // retrieve value information
            status = atts_get_value(conidx, *start_hdl, length, value);
        }
    }

    return (status);
}


/**
 ****************************************************************************************
 * @brief Update the value of the specified attribute.
 *
 * After updating the value, the function sends an indication to the upper layer profile
 * or protocol task registered for the specified attribute.
 *
 * @param[in] conidx   connection index
 * @param[in] value    Pointer to the data to be written
 * @param[in] length   The length to be written
 * @param[in] offset   The offset at which the data has to be written
 * @param[in] handle   The handle of the attribute that has to be written
 ****************************************************************************************
 */
static void atts_update_elmt(uint8_t conidx, uint8_t *value, uint16_t length,
                             uint16_t offset, uint16_t handle)
{
    // send write indication to set task anyway
    struct gattc_write_req_ind *wr_ind = KE_MSG_ALLOC_DYN(GATTC_WRITE_REQ_IND,
                                         atts_get_dest_task(conidx, handle), KE_BUILD_ID(TASK_GATTC, conidx),
                                         gattc_write_req_ind, length);

    // Fill the message parameters
    wr_ind->handle   = handle;
    wr_ind->length   = length;
    wr_ind->offset   = offset;

    memcpy(&wr_ind->value[0], value, length);

    // And send the message
    ke_msg_send(wr_ind);
}

/**
 ****************************************************************************************
 * @brief Check if attribute handle is present in prepare write list.
 *
 * @param[in] conidx            connection index
 * @param[in] handle            connection handle
 *
 * @return True if present, False else.
 ****************************************************************************************
 */
static bool atts_hdl_present_in_prep_data(uint8_t conidx, uint16_t handle)
{
    bool ret = false;

    struct atts_prep_data *elem = (struct atts_prep_data *) co_list_pick(&(gattc_env[conidx]->server.prep_wr_req_list));

    // check if handle present in prepare write list
    while ((elem != NULL) && !(ret))
    {
        struct l2cc_pdu_recv_ind *p_ind = (struct l2cc_pdu_recv_ind *)elem->value;
        struct l2cc_att_prep_wr_req *p_req = &p_ind->pdu.data.prep_wr_req;

        // check handle value
        if (handle == p_req->handle)
        {
            ret = true;
        }
        else
        {
            elem = (struct atts_prep_data *)  elem->hdr.next;
        }
    }
    return ret;
}

/**
 ****************************************************************************************
 * @brief Store temporary data.
 *
 * @param[in] conidx            connection index
 * @param[in] buf               pointer to buffer data
 * @param[in] len               length of the buffer
 * @param[in] offset            offset value
 * @param[in] handle            connection handle
 * @param[in] current_len       Attribute current length
 *
 * @return If the data can be stored or not.
 ****************************************************************************************
 */
static uint8_t atts_store_prep_data(uint8_t conidx, struct ke_msg *msg, uint16_t len,
                                    uint16_t offset, uint16_t handle, uint16_t current_len)
{
    uint8_t status = ATT_ERR_NO_ERROR;
    struct atts_prep_data *first = NULL;
    struct atts_prep_data *prep;
    uint16_t total_len = len;

    // check if prepare write size do not take to much memory
    if (!co_list_is_empty(&gattc_env[conidx]->server.prep_wr_req_list))
    {
        first = (struct atts_prep_data *) co_list_pick(&gattc_env[conidx]->server.prep_wr_req_list);
        if (first->total_len  + len > ATT_MAX_VALUE)
        {
            // reject not enough resources
            status = ATT_ERR_INSUFF_RESOURCE;
        }
        else
        {
            // keep total length information
            total_len = first->total_len  + len;
        }
    }

    if (status == ATT_ERR_NO_ERROR)
    {
        prep = (struct atts_prep_data *)msg;

        // Fill in the prepared write structure
        prep->len     = len;
        prep->offset  = offset;
        prep->cur_len = current_len;

        // sort list by handles
        struct atts_prep_data *elem = (struct atts_prep_data *) co_list_pick(&(gattc_env[conidx]->server.prep_wr_req_list));

        // check if it's end of list / list is empty
        while (elem != NULL)
        {
            struct l2cc_pdu_recv_ind *p_ind = (struct l2cc_pdu_recv_ind *)elem->value;
            struct l2cc_att_prep_wr_req *p_req = &p_ind->pdu.data.prep_wr_req;

            // ensure that list is sorted by handles
            if (handle < p_req->handle)
            {
                // insert element before next handle greater than current handle
                co_list_insert_before((&gattc_env[conidx]->server.prep_wr_req_list),
                                      (struct co_list_hdr *)elem, (struct co_list_hdr *) prep);
                break;
            }

            elem = (struct atts_prep_data *)  elem->hdr.next;
        }

        // put element at end of the list
        if (elem == NULL)
        {
            // Push new element in the list
            co_list_push_back((&gattc_env[conidx]->server.prep_wr_req_list),
                              (struct co_list_hdr *) prep);
        }

        // update total prepare write queue length
        first = (struct atts_prep_data *) co_list_pick(&gattc_env[conidx]->server.prep_wr_req_list);
        first->total_len = total_len;
    }

    return (status);
}

/**
 ****************************************************************************************
 * @brief Send a PDU Attribute packet through L2CAP
 *
 * @param[in] pdu        PDU Packet
 ****************************************************************************************
 */
static void atts_send_pdu(void *pdu)
{
    /* send the L2CC message */
    l2cc_pdu_send(pdu);
}



/*
 * EXTERNAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Send an attribute error response to peer.
 *
 * @param[in] conidx            Index of the connection with the peer device
 * @param[in] opcode            failing operation code
 * @param[in] uuid              attribute UUID
 * @param[in] error             error code
 ****************************************************************************************
 */
void atts_send_error(uint8_t conidx, uint8_t opcode, uint16_t uuid, uint8_t error)
{
    struct l2cc_att_err_rsp *err_rsp = ATTS_ALLOCATE_PDU(conidx,
                                       L2C_CODE_ATT_ERR_RSP, l2cc_att_err_rsp, L2C_MIN_LE_MTUSIG);

    // retrieve error code
    err_rsp->op_code = opcode;
    // Fill Packet
    err_rsp->handle = uuid;
    err_rsp->reason = error;

    /* send the L2Cap message */
    atts_send_pdu(err_rsp);
}



void atts_write_signed_cfm(uint8_t conidx, uint16_t length, uint8_t *sign_data)
{
    struct attm_elmt elmt = ATT_ELEMT_INIT;
    uint16_t handle = co_read16p(&(sign_data[1]));

    /* re-check permissions to verify that permissions are OK */
    uint8_t status = atts_get_att_chk_perm(conidx, ATT_WRITE_SIGNED_ACCESS,
                                           handle, &elmt);

    if (status == ATT_ERR_NO_ERROR)
    {
        // Get the length of useful value
        uint16_t data_len = length - ATT_SIGNED_PDU_VAL_OFFSET - ATT_SIGNATURE_LEN;

        // sanity check
        if (length < (ATT_SIGNED_PDU_VAL_OFFSET + ATT_SIGNATURE_LEN))
        {
            data_len = 0;
        }

        // Update the entry
        atts_update_elmt(conidx, &sign_data[ATT_SIGNED_PDU_VAL_OFFSET], data_len, 0x0000, handle);
    }
}


uint8_t atts_send_event(uint8_t conidx, struct gattc_send_evt_cmd *event)
{
    struct attm_elmt  elmt = ATT_ELEMT_INIT;
    // check permission
    uint8_t status = atts_get_att_chk_perm(conidx,
                                           ((event->operation == GATTC_NOTIFY) ? ATT_NOTIFY_ACCESS : ATT_INDICATE_ACCESS),
                                           event->handle, &elmt);

    /* retrieve attribute */
    if (status == ATT_ERR_NO_ERROR)
    {
        /* ensure value is within negotiated MTU (3 = Code length + Handle length) */
        uint16_t value_len = co_min(event->length, (gattc_get_mtu(conidx) - (ATT_CODE_LEN + ATT_HANDLE_LEN)));

        struct l2cc_att_hdl_val_ind *hdl_val_ind = ATTS_ALLOCATE_PDU(conidx,
                ((event->operation == GATTC_NOTIFY) ? L2C_CODE_ATT_HDL_VAL_NTF : L2C_CODE_ATT_HDL_VAL_IND),
                l2cc_att_hdl_val_ind, value_len);

        /* element handle */
        hdl_val_ind->handle = event->handle;

        /* ensure value is within negotiated MTU */
        hdl_val_ind->value_len = value_len;

        /* copy the content to value */
        memcpy(&(hdl_val_ind->value[0]), &(event->value[0]), hdl_val_ind->value_len);

        /* construct and send the PDU */
        atts_send_pdu(hdl_val_ind);

        /* specific to indication */
        if (event->operation != GATTC_NOTIFY)
        {
            /* start the timer */
            ke_timer_set(GATTC_SERVER_RTX_IND, KE_BUILD_ID(TASK_GATTC, conidx), ATT_TRANS_RTX);
        }
    }

    return (status);
}

void atts_clear_prep_data(uint8_t conidx)
{
    // cleanup prepare write structure
    while (!co_list_is_empty(&(gattc_env[conidx]->server.prep_wr_req_list)))
    {
        struct atts_prep_data *prep = (struct atts_prep_data *)
                                      co_list_pop_front(&(gattc_env[conidx]->server.prep_wr_req_list));
        ke_free(prep);
    }
}


void atts_clear_rsp_data(uint8_t conidx)
{
    // cleanup response data list
    while (!co_list_is_empty(&(gattc_env[conidx]->server.rsp)))
    {
        struct co_list_hdr *data = co_list_pop_front(&(gattc_env[conidx]->server.rsp));
        ke_free(data);
    }
}

void atts_clear_read_cache(uint8_t conidx)
{
    // clean-up read cache
    if (gattc_env[conidx]->server.read_cache != NULL)
    {
        ke_msg_free(ke_param2msg(gattc_env[conidx]->server.read_cache));
        gattc_env[conidx]->server.read_cache = NULL;
    }
}

void atts_write_rsp_send(uint8_t conidx, uint16_t atthdl, uint8_t status)
{
    if (status == ATT_ERR_NO_ERROR)
    {
        struct l2cc_att_wr_rsp *wr_rsp = ATTS_ALLOCATE_PDU(conidx,
                                         L2C_CODE_ATT_WR_RSP, l2cc_att_wr_rsp, 0);
        // send the write response
        atts_send_pdu(wr_rsp);
    }
    else
    {
        atts_send_error(conidx, L2C_CODE_ATT_WR_REQ, atthdl, status);
    }
}



/*
 * PDU HANDLERS
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @brief Handles the MTU exchange request and constructs response.
 *
 * @param[in] conidx        Index of the connection with the peer device
 * @param[in] req           Message request
 *
 * @return If the message was consumed or not.
 *
 ****************************************************************************************
 */
static int atts_mtu_exc_req(uint8_t conidx, struct l2cc_att_mtu_req *req)
{
    // check if MTU already exchanged
    // (!gattc_env[conidx]->mtu_exch)
    {
        // Maximal MTU
        uint16_t max_mtu = gapm_get_max_mtu();

        struct l2cc_att_mtu_rsp *mtu_rsp = ATTS_ALLOCATE_PDU(conidx,
                                           L2C_CODE_ATT_MTU_RSP, l2cc_att_mtu_rsp, 0);

        // Set the MTU value
        gattc_set_mtu(conidx, co_min(gapm_get_max_mtu(), req->mtu_size));

        // Send the device MTU
        // mtu_rsp->mtu_size = max_mtu;
        if (req->mtu_size > max_mtu)
        {
            mtu_rsp->mtu_size = max_mtu;
        }
        else
        {
            mtu_rsp->mtu_size = req->mtu_size;
        }

        atts_send_pdu(mtu_rsp);
    }
#if 0
    else
    {
        atts_send_error(conidx, L2C_CODE_ATT_MTU_REQ, 0, ATT_ERR_INVALID_PDU);
    }
#endif

    /* construct and send the PDU */
    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles read info request and constructs response.
 *
 * @param[in] conidx        Index of the connection with the peer device
 * @param[in] req           Message request
 *
 * @return If the message was consumed or not.
 *
 ****************************************************************************************
 */
static int atts_find_info_req(uint8_t conidx, struct l2cc_att_find_info_req *req)
{
    uint8_t  status   = ATT_ERR_NO_ERROR;
    uint16_t max_len = gattc_get_mtu(conidx) - (ATT_CODE_LEN + ATT_FORMAT_LEN);
    // Compute the length available in the buffer for the attribute values
    uint16_t remaining_len = max_len;
    // found UUID length
    uint8_t  length = 0x00;

    // Perform initial checks
    if ((req->shdl > req->ehdl) || (req->shdl == ATT_INVALID_SEARCH_HANDLE))
    {
        // start and end handles are invalid
        status = ATT_ERR_INVALID_HANDLE;
    }
    else
    {
        uint16_t handle = req->shdl;
        struct attm_elmt elmt = ATT_ELEMT_INIT;

        do
        {
            /* retrieve first visible attribute from handle to end of DB */
            status = attmdb_get_next_att(&handle, &elmt);

            // no attributes found.
            if (status != ATT_ERR_NO_ERROR)
            {
                status = ATT_ERR_ATTRIBUTE_NOT_FOUND;
                break;
            }

            // check if element found
            if (handle <= req->ehdl)
            {
                struct atts_find_info_rsp *info = (struct atts_find_info_rsp *)
                                                  ke_malloc(sizeof(struct atts_find_info_rsp), KE_MEM_KE_MSG);

                // retrieve UUID in AIR format
                status = attmdb_get_uuid(&elmt, &(info->uuid_len), info->uuid, false, true);
                ASSERT_INFO(status == ATT_ERR_NO_ERROR, handle, status);
                // store handle value
                info->handle = handle;

                // store first found UUID length
                if (length == 0)
                {
                    length = info->uuid_len;
                }

                // Check if the new attribute UUID has the same length as the previous ones
                if (length != info->uuid_len)
                {
                    // drop latest found info
                    ke_free(info);
                    // stop loop
                    break;
                }
                else
                {
                    // put response at end of the list
                    co_list_push_back(&(gattc_env[conidx]->server.rsp), &(info->hdr));
                    remaining_len -= (length + 2);
                    handle++;
                }
            }
        }
        while ((handle <= req->ehdl)
                && (handle != ATT_INVALID_HANDLE)
                && (remaining_len >= (length + 2)));
    }

    // At this point, we have either prepared the find info response, or we got an
    // error. In the first case, we send the prepared response, otherwise we send an error
    // response with the current status
    if (!co_list_is_empty(&(gattc_env[conidx]->server.rsp)))
    {
        uint16_t cursor = 0;
        struct atts_find_info_rsp *info;
        // Allocate Response
        struct l2cc_att_find_info_rsp *rsp = ATTS_ALLOCATE_PDU(conidx,
                                             L2C_CODE_ATT_FIND_INFO_RSP, l2cc_att_find_info_rsp, (max_len - remaining_len));

        rsp->format = (length == ATT_UUID_16_LEN) ? ATT_FORMAT_16BIT_UUID : ATT_FORMAT_128BIT_UUID;
        rsp->data_len += (max_len - remaining_len);

        // browse list
        info = (struct atts_find_info_rsp *) co_list_pick(&(gattc_env[conidx]->server.rsp));

        while (info != NULL)
        {
            co_write16(&(rsp->data[cursor]), info->handle);
            cursor += ATT_HANDLE_LEN;
            memcpy(&(rsp->data[cursor]), info->uuid, info->uuid_len);
            cursor += info->uuid_len;
            // go to next element
            info = (struct atts_find_info_rsp *) info->hdr.next;
        }

        // Send the PDU
        atts_send_pdu(rsp);
    }
    else
    {
        if (status == ATT_ERR_NO_ERROR)
        {
            // By default, we consider we don't find any attribute
            status = ATT_ERR_ATTRIBUTE_NOT_FOUND;
        }

        // No response was prepared, so we send an error response indicating the reason
        // that caused the issue.
        atts_send_error(conidx, req->code, req->shdl, status);
    }

    return (KE_MSG_CONSUMED);
}


/**
 ****************************************************************************************
 * @brief Handles find by type request and constructs response.
 *
 * @param[in] conidx        Index of the connection with the peer device
 * @param[in] req           Message request
 *
 * @return If the message was consumed or not.
 *
 ****************************************************************************************
 */
static int atts_find_by_type_req(uint8_t conidx, struct l2cc_att_find_by_type_req *req)
{
    uint8_t status = ATT_ERR_NO_ERROR;
    struct attm_elmt elmt = ATT_ELEMT_INIT;
    uint16_t handle = req->shdl;

    // retrieve stored parameters to continue execution
    uint16_t *remaining_len = ATTS_REMAINING_LEN(req);

    // initialization value
    if (*remaining_len == 0)
    {
        *remaining_len = gattc_get_mtu(conidx) - ATT_CODE_LEN;

        // Perform initial checks
        if ((req->shdl > req->ehdl) || (req->shdl == ATT_INVALID_SEARCH_HANDLE))
        {
            // start and end handles are invalid
            status = ATT_ERR_INVALID_HANDLE;
        }
    }

    if (status == ATT_ERR_NO_ERROR)
    {
        uint16_t end_hdl;

        do
        {
            uint8_t *value;
            uint16_t length;
            end_hdl = req->ehdl;

            // Search the UUID
            status = atts_find_value_by_uuid(conidx, &handle, &end_hdl, ATT_UUID_16_LEN,
                                             (uint8_t *) & (req->type), &elmt, &length, &value);

            // attribute not found, exit
            if (status != ATT_ERR_NO_ERROR)
            {
                // stop loop because waiting for upper layer to answer read request or attribute not found
                if ((status == ATT_ERR_ATTRIBUTE_NOT_FOUND) || (status == ATT_ERR_REQUEST_NOT_SUPPORTED))
                {
                    break;
                }

                // An attribute cannot be read, go to next one
                handle = end_hdl + 1;
                continue;
            }

            // specific check if trying to found a service
            if ((((req->type == ATT_DECL_PRIMARY_SERVICE) || (req->type == ATT_DECL_SECONDARY_SERVICE))
                    && attm_uuid_comp(req->val, req->val_len, value, length))
                    // if not a service, verify if value and value len is matching
                    || ((length == req->val_len) && (memcmp(req->val, value, req->val_len) == 0)))
            {
                struct atts_find_by_type_rsp *find =
                    (struct atts_find_by_type_rsp *) ke_malloc(sizeof(struct atts_find_by_type_rsp), KE_MEM_KE_MSG);

                // Copy the attribute range into the PDU structure
                find->shdl = handle;
                find->ehdl = end_hdl;
                remaining_len -= (2 * ATT_HANDLE_LEN);

                // put element at end of the list
                co_list_push_back(&(gattc_env[conidx]->server.rsp), &(find->hdr));

                // clear read cache
                atts_clear_read_cache(conidx);
            }

            // continue search
            handle = end_hdl + 1;
        }
        // loop until handle end not reach and there are still data that can be put in response
        while ((handle <= req->ehdl) && (end_hdl != req->ehdl) && (*remaining_len >= (ATT_HANDLE_LEN * 2)));

    }

    // At this point, all read have been perform, answer can be triggered
    if (status != ATT_ERR_REQUEST_NOT_SUPPORTED)
    {
        uint16_t size = co_list_size(&(gattc_env[conidx]->server.rsp));

        if (size != 0)
        {
            uint16_t cursor = 0;
            struct atts_find_by_type_rsp *val;

            struct l2cc_att_find_by_type_rsp *rsp = ATTS_ALLOCATE_PDU(conidx,
                                                    L2C_CODE_ATT_FIND_BY_TYPE_RSP, l2cc_att_find_by_type_rsp, size * (2 * ATT_HANDLE_LEN));

            // fill data length for packer
            rsp->data_len =  size * (2 * ATT_HANDLE_LEN);
            // browse list
            val = (struct atts_find_by_type_rsp *) co_list_pick(&(gattc_env[conidx]->server.rsp));

            while (val != NULL)
            {
                // copy element value
                co_write16p(&(rsp->data[cursor]), val->shdl);
                cursor += ATT_HANDLE_LEN;
                co_write16p(&(rsp->data[cursor]), val->ehdl);
                cursor += ATT_HANDLE_LEN;

                // next element
                val = (struct atts_find_by_type_rsp *) val->hdr.next;
            }

            // Send the PDU
            atts_send_pdu(rsp);
        }
        else
        {
            /* find by type problem */
            atts_send_error(conidx, req->code, handle, ATT_ERR_ATTRIBUTE_NOT_FOUND);
        }
    }
    else
    {
        // store current state of discovery
        req->shdl = handle;
    }

    // check if operation is finish or if waiting for atts response
    return ((status == ATT_ERR_REQUEST_NOT_SUPPORTED) ? KE_MSG_NO_FREE : KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles read by type request and constructs response.
 *
 * @param[in] conidx        Index of the connection with the peer device
 * @param[in] req           Message request
 *
 * @return If the message was consumed or not.
 *
 ****************************************************************************************
 */
static int atts_read_by_type_req(uint8_t conidx, struct l2cc_att_rd_by_type_req *req)
{
    uint8_t status = ATT_ERR_NO_ERROR;
    struct attm_elmt elmt = ATT_ELEMT_INIT;
    uint16_t handle = req->shdl;

    // retrieve stored parameters to continue execution
    uint16_t *remaining_len = ATTS_REMAINING_LEN(req);
    uint16_t elmt_length = 0;

    // initialization value
    if (*remaining_len == 0)
    {
        *remaining_len = gattc_get_mtu(conidx) - ATT_EACHLEN_LEN - ATT_CODE_LEN;

        // Perform initial checks
        if ((req->shdl > req->ehdl) || (req->shdl == ATT_INVALID_SEARCH_HANDLE))
        {
            // start and end handles are invalid
            status = ATT_ERR_INVALID_HANDLE;
        }
    }

    // retrieve value length.
    if (!co_list_is_empty(&(gattc_env[conidx]->server.rsp)))
    {
        elmt_length = ((struct atts_rd_by_type_rsp *) co_list_pick(&(gattc_env[conidx]->server.rsp)))->length;
    }

    if (status == ATT_ERR_NO_ERROR)
    {
        uint16_t end_hdl;

        do
        {
            uint8_t *value;
            uint16_t length;

            end_hdl = req->ehdl;

            // Search the UUID
            status = atts_find_value_by_uuid(conidx, &handle, &end_hdl, req->uuid_len,
                                             (uint8_t *) & (req->uuid), &elmt, &length, &value);

            if (status != ATT_ERR_NO_ERROR)
            {
                // Attribute not found, so stop the search
                break;
            }

            // check that element size is ok.
            if ((elmt_length != length) && (elmt_length != 0))
            {
                // End of search because length not fit
                break;
            }
            else
            {
                // calculate value length : min(length, remain - ATT_HANDLE_LEN, 255 - ATT_HANDLE_LEN)
                uint16_t val_length = co_min(length,
                                             co_min(*remaining_len - ATT_HANDLE_LEN, 255 - ATT_HANDLE_LEN));

                struct atts_rd_by_type_rsp *rd =
                    (struct atts_rd_by_type_rsp *) ke_malloc(sizeof(struct atts_rd_by_type_rsp) + val_length, KE_MEM_KE_MSG);

                // save element length
                elmt_length = length;


                // Copy the attribute range into the PDU structure
                rd->handle = handle;
                rd->length = (uint8_t) val_length;
                memcpy(rd->value, value, val_length);

                *remaining_len -= ATT_HANDLE_LEN + val_length;
                co_list_push_back(&(gattc_env[conidx]->server.rsp), &(rd->hdr));

                // check if cache should be cleaned-up
                if (val_length == length)
                {
                    atts_clear_read_cache(conidx);
                }

                // continue search for next handle
                handle = end_hdl + 1;
            }
        }

        // loop until handle end not reach and there are still data that can be put in response
        while ((handle <= req->ehdl) && (end_hdl != req->ehdl) && (*remaining_len >= (elmt_length + ATT_HANDLE_LEN)));
    }

    // At this point, all read have been perform, answer can be triggered
    if (status != ATT_ERR_REQUEST_NOT_SUPPORTED)
    {
        uint16_t size = co_list_size(&(gattc_env[conidx]->server.rsp));

        if (size != 0)
        {
            uint16_t cursor = 0;
            struct atts_rd_by_type_rsp *val =
                (struct atts_rd_by_type_rsp *) co_list_pick(&(gattc_env[conidx]->server.rsp));
            // calculate data length
            uint16_t data_len = size * (ATT_HANDLE_LEN + val->length);

            struct l2cc_att_rd_by_type_rsp *rsp = ATTS_ALLOCATE_PDU(conidx,
                                                  L2C_CODE_ATT_RD_BY_TYPE_RSP, l2cc_att_rd_by_type_rsp, data_len);

            // fill data length for packer
            rsp->data_len = data_len;
            rsp->each_len = (ATT_HANDLE_LEN + val->length);

            // browse list
            while (val != NULL)
            {
                // copy element value
                co_write16p(&(rsp->data[cursor]), val->handle);
                cursor += ATT_HANDLE_LEN;
                memcpy(&(rsp->data[cursor]), val->value, val->length);
                cursor += val->length;

                // next element
                val = (struct atts_rd_by_type_rsp *) val->hdr.next;
            }

            // Send the PDU
            atts_send_pdu(rsp);
        }
        else
        {
            /* find by type problem */
            atts_send_error(conidx, req->code, handle, status);
        }
    }
    else
    {
        // store current state of discovery
        req->shdl = handle;
    }

    // check if operation is finish or if waiting for atts response
    return ((status == ATT_ERR_REQUEST_NOT_SUPPORTED) ? KE_MSG_NO_FREE : KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles read by group type request and constructs response.
 *
 * @param[in] conidx        Index of the connection with the peer device
 * @param[in] req           Message request
 *
 * @return If the message was consumed or not.
 *
 ****************************************************************************************
 */
static int atts_read_by_grp_type_req(uint8_t conidx, struct l2cc_att_rd_by_grp_type_req *req)
{
    uint8_t status = ATT_ERR_NO_ERROR;
    struct attm_elmt elmt = ATT_ELEMT_INIT;
    uint16_t handle = req->shdl;

    // retrieve stored parameters to continue execution
    uint16_t *remaining_len = ATTS_REMAINING_LEN(req);
    uint16_t elmt_length = 0;

    // initialization value
    if (*remaining_len == 0)
    {
        *remaining_len = gattc_get_mtu(conidx) - ATT_CODE_LEN - ATT_EACHLEN_LEN;

        // Perform initial checks
        if ((req->shdl > req->ehdl) || (req->shdl == ATT_INVALID_SEARCH_HANDLE))
        {
            // start and end handles are invalid
            status = ATT_ERR_INVALID_HANDLE;
        }
    }

    // retrieve value length.
    if (!co_list_is_empty(&(gattc_env[conidx]->server.rsp)))
    {
        elmt_length = ((struct atts_rd_by_type_rsp *) co_list_pick(&(gattc_env[conidx]->server.rsp)))->length;
    }

    if (status == ATT_ERR_NO_ERROR)
    {
        uint16_t end_hdl;

        do
        {
            uint8_t *value;
            uint16_t length;
            end_hdl = req->ehdl;

            // Search the UUID
            status = atts_find_value_by_uuid(conidx, &handle, &end_hdl, req->uuid_len,
                                             (uint8_t *) & (req->uuid), &elmt, &length, &value);

            if (status != ATT_ERR_NO_ERROR)
            {
                // Attribute not found, so stop the search
                break;
            }

            // check that element size is ok.
            if ((elmt_length != length) && (elmt_length != 0))
            {
                // End of search because length not fit
                break;
            }
            else
            {
                // calculate value length : min(lenght, remain - 2*ATT_HANDLE_LEN, 255 - 2*ATT_HANDLE_LEN)
                uint16_t val_length = co_min(length,
                                             co_min(*remaining_len - (2 * ATT_HANDLE_LEN), 255 - (2 * ATT_HANDLE_LEN)));

                struct atts_rd_by_grp_type_rsp *rd =
                    (struct atts_rd_by_grp_type_rsp *) ke_malloc(sizeof(struct atts_rd_by_grp_type_rsp) + val_length, KE_MEM_KE_MSG);

                // save element length
                elmt_length = length;

                // Copy the attribute range into the PDU structure
                rd->shdl = handle;
                rd->ehdl = end_hdl;
                rd->length = (uint8_t) val_length;
                memcpy(rd->value, value, val_length);

                *remaining_len -= (2 * ATT_HANDLE_LEN) + val_length;
                co_list_push_back(&(gattc_env[conidx]->server.rsp), &(rd->hdr));

                // check if cache should be cleaned-up
                if (val_length == length)
                {
                    atts_clear_read_cache(conidx);
                }

                // continue search for next handle
                handle = end_hdl + 1;
            }
        }
        // loop until handle end not reach and there are still data that can be put in response
        while ((handle <= req->ehdl) && (end_hdl != req->ehdl) && (*remaining_len >= (elmt_length + (2 * ATT_HANDLE_LEN))));
    }

    // At this point, all read have been perform, answer can be triggered
    if (status != ATT_ERR_REQUEST_NOT_SUPPORTED)
    {
        uint16_t size = co_list_size(&(gattc_env[conidx]->server.rsp));

        if (size != 0)
        {
            uint16_t cursor = 0;
            struct atts_rd_by_grp_type_rsp *val =
                (struct atts_rd_by_grp_type_rsp *) co_list_pick(&(gattc_env[conidx]->server.rsp));
            // calculate data length
            uint16_t data_len = size * ((2 * ATT_HANDLE_LEN) + val->length);

            struct l2cc_att_rd_by_grp_type_rsp *rsp = ATTS_ALLOCATE_PDU(conidx,
                    L2C_CODE_ATT_RD_BY_GRP_TYPE_RSP, l2cc_att_rd_by_grp_type_rsp, data_len);

            // fill data length for packer
            rsp->data_len = data_len;
            rsp->each_len = ((2 * ATT_HANDLE_LEN) + val->length);

            // browse list
            while (val != NULL)
            {
                // copy element value
                co_write16p(&(rsp->data[cursor]), val->shdl);
                cursor += ATT_HANDLE_LEN;
                co_write16p(&(rsp->data[cursor]), val->ehdl);
                cursor += ATT_HANDLE_LEN;
                memcpy(&(rsp->data[cursor]), val->value, val->length);
                cursor += val->length;

                // next element
                val = (struct atts_rd_by_grp_type_rsp *) val->hdr.next;
            }

            // Send the PDU
            atts_send_pdu(rsp);
        }
        else
        {
            /* find by type problem */
            atts_send_error(conidx, req->code, handle, status);
        }
    }
    else
    {
        // store current state of discovery
        req->shdl = handle;
    }

    // check if operation is finish or if waiting for atts response
    return ((status == ATT_ERR_REQUEST_NOT_SUPPORTED) ? KE_MSG_NO_FREE : KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles read request and constructs response.
 *
 * @param[in] conidx        Index of the connection with the peer device
 * @param[in] req           Message request
 *
 * @return If the message was consumed or not.
 *
 **********************uint8_t******************************************************************
 */
static int atts_read_req(uint8_t conidx, struct l2cc_att_rd_req *req)
{
    uint8_t status = ATT_ERR_NO_ERROR;
    struct attm_elmt elmt = ATT_ELEMT_INIT;

    /* retrieve attribute +  check permission */
    status = atts_get_att_chk_perm(conidx, ATT_READ_ACCESS, req->handle, &elmt);

    if (status == ATT_ERR_NO_ERROR)
    {
        uint8_t *value;
        uint16_t length;

        status = atts_get_value(conidx, req->handle, &length, &value);

        if (status == ATT_ERR_NO_ERROR)
        {
            // Ensure that the length is not higher than negociated MTU (1 = Code length)
            uint16_t value_len = co_min(length, gattc_get_mtu(conidx) - ATT_CODE_LEN);
            struct l2cc_att_rd_rsp *rd_rsp = ATTS_ALLOCATE_PDU(conidx, L2C_CODE_ATT_RD_RSP,
                                             l2cc_att_rd_rsp, value_len);

            // Prepare the PDU structure
            rd_rsp->value_len = value_len;
            // Copy the attribute data into the response buffer
            memcpy(&rd_rsp->value[0], value, rd_rsp->value_len);
            /* construct and send PDU */
            atts_send_pdu(rd_rsp);

            // all data read, clean-up cache
            if ((value_len == length) && (value_len != (gattc_get_mtu(conidx) - ATT_CODE_LEN)))
            {
                atts_clear_read_cache(conidx);
            }
        }
    }

    if ((status != ATT_ERR_NO_ERROR) && (status != ATT_ERR_REQUEST_NOT_SUPPORTED))
    {
        /* read request not allowed */
        atts_send_error(conidx, req->code, req->handle, status);
    }

    // check if operation is finish or if waiting for atts response
    return ((status == ATT_ERR_REQUEST_NOT_SUPPORTED) ? KE_MSG_NO_FREE : KE_MSG_CONSUMED);
}


/**
 ****************************************************************************************
 * @brief Handles read blob request and constructs response.
 *
 * @param[in] conidx        Index of the connection with the peer device
 * @param[in] req           Message request
 *
 * @return If the message was consumed or not.
 *
 ****************************************************************************************
 */
static int atts_read_blob_req(uint8_t conidx, struct l2cc_att_rd_blob_req *req)
{
    uint8_t status = ATT_ERR_NO_ERROR;
    struct attm_elmt elmt = ATT_ELEMT_INIT;

    /* retrieve attribute +  check permission */
    status = atts_get_att_chk_perm(conidx, ATT_READ_ACCESS, req->handle, &elmt);

    if (status == ATT_ERR_NO_ERROR)
    {
        uint8_t *value;
        uint16_t length;
        uint16_t value_len = 0;

        status = atts_get_value(conidx, req->handle, &length, &value);

        if (status == ATT_ERR_NO_ERROR)
        {
            if (req->offset > length)
            {
                // Send invalid offset error
                status = ATT_ERR_INVALID_OFFSET;
            }
            else
            {
                // retrieve value length
                value_len = co_min(length - (req->offset), gattc_get_mtu(conidx) - ATT_CODE_LEN);
            }
        }

        if (status == ATT_ERR_NO_ERROR)
        {
            struct l2cc_att_rd_blob_rsp *rd_blob_rsp;

            rd_blob_rsp = ATTS_ALLOCATE_PDU(conidx, L2C_CODE_ATT_RD_BLOB_RSP,
                                            l2cc_att_rd_blob_rsp, length);

            if (status == ATT_ERR_NO_ERROR)
            {
                // Prepare the PDU structure
                rd_blob_rsp->value_len = value_len;

                // Copy the attribute data into the response buffer
                memcpy(&rd_blob_rsp->value[0], &(value[req->offset]), rd_blob_rsp->value_len);

                /* construct and send PDU */
                atts_send_pdu(rd_blob_rsp);
            }

            // all data read, clean-up cache
            if (((value_len + req->offset) == length) && (value_len != (gattc_get_mtu(conidx) - ATT_CODE_LEN)))
            {
                atts_clear_read_cache(conidx);
            }
        }
    }

    if ((status != ATT_ERR_NO_ERROR) && (status != ATT_ERR_REQUEST_NOT_SUPPORTED))
    {
        /* read request not allowed */
        atts_send_error(conidx, req->code, req->handle, status);
    }

    // check if operation is finish or if waiting for atts response
    return ((status == ATT_ERR_REQUEST_NOT_SUPPORTED) ? KE_MSG_NO_FREE : KE_MSG_CONSUMED);
}


/**
 ****************************************************************************************
 * @brief Handles read multiple handle request and constructs response.
 *
 * @param[in] conidx        Index of the connection with the peer device
 * @param[in] req           Message request
 *
 * @return If the message was consumed or not.
 *
 ****************************************************************************************
 */
static int atts_read_mult_req(uint8_t conidx, struct l2cc_att_rd_mult_req *req)
{
    uint8_t status = ATT_ERR_NO_ERROR;
    struct attm_elmt elmt = ATT_ELEMT_INIT;

    // retrieve stored parameters to continue execution
    uint16_t cursor = co_list_size(&(gattc_env[conidx]->server.rsp));
    uint16_t *remaining_len = ATTS_REMAINING_LEN(req);

    // initialization
    if (*remaining_len == 0)
    {
        // MTU - 1;
        *remaining_len = gattc_get_mtu(conidx) - ATT_CODE_LEN;
    }

    // Put as many attributes as possible
    while ((req->nb_handles > cursor)
            && (*remaining_len > 0)
            && (status == ATT_ERR_NO_ERROR))
    {
        uint16_t length;
        uint8_t *value;

        /* retrieve attribute +  check permission */
        status = atts_get_att_chk_perm(conidx, ATT_READ_ACCESS, req->handles[cursor], &elmt);

        // exit if permission not ok
        if (status != ATT_ERR_NO_ERROR)
        {
            break;
        }

        // retrieve value information
        status = atts_get_value(conidx, req->handles[cursor], &length, &value);

        if (status == ATT_ERR_NO_ERROR)
        {
            // retrieve value length
            uint16_t value_len = co_min(length, *remaining_len);;
            // allocate partial response
            struct atts_rd_mult_rsp *val =
                (struct atts_rd_mult_rsp *) ke_malloc(sizeof(struct atts_rd_mult_rsp) + value_len, KE_MEM_KE_MSG);

            // copy value in partial response
            val->length = value_len;
            memcpy(val->value, value, value_len);

            // put partial response in response list
            co_list_push_back(&(gattc_env[conidx]->server.rsp), &(val->hdr));
            // current value in cache can be removed
            atts_clear_read_cache(conidx);

            *remaining_len -= value_len;
            cursor++;
        }
    }

    // At this point, all read have been perform, answer can be triggered
    if (status == ATT_ERR_NO_ERROR)
    {
        uint16_t value_len = (gattc_get_mtu(conidx) - ATT_CODE_LEN) - *remaining_len;
        uint16_t count_len = 0;
        struct atts_rd_mult_rsp *val;

        // Allocate Response
        struct l2cc_att_rd_mult_rsp *rsp = ATTS_ALLOCATE_PDU(conidx,
                                           L2C_CODE_ATT_RD_MULT_RSP, l2cc_att_rd_mult_rsp, value_len);

        rsp->value_len = value_len;

        // browse list
        val = (struct atts_rd_mult_rsp *) co_list_pick(&(gattc_env[conidx]->server.rsp));

        while (val != NULL)
        {
            memcpy(&(rsp->value[count_len]), val->value, val->length);
            count_len += val->length;
            // next element
            val = (struct atts_rd_mult_rsp *) val->hdr.next;
        }

        // Send the PDU
        atts_send_pdu(rsp);
    }


    if ((status != ATT_ERR_NO_ERROR) && (status != ATT_ERR_REQUEST_NOT_SUPPORTED))
    {
        /* read request not allowed */
        atts_send_error(conidx, req->code, req->handles[cursor], status);
    }

    // check if operation is finish or if waiting for atts response
    return ((status == ATT_ERR_REQUEST_NOT_SUPPORTED) ? KE_MSG_NO_FREE : KE_MSG_CONSUMED);
}


/**
 ****************************************************************************************
 * @brief Handles write no response and updates the attribute server.
 *
 * @param[in] conidx        Index of the connection with the peer device
 * @param[in] cmd           Message command
 *
 * @return If the message was consumed or not.
 *
 ****************************************************************************************
 */
static int atts_write_cmd(uint8_t conidx, struct l2cc_att_wr_cmd *cmd)
{
    struct attm_elmt elmt = ATT_ELEMT_INIT;
    int msg_status = KE_MSG_CONSUMED;

    /* retrieve attribute +  check permission */
    if ((atts_get_att_chk_perm(conidx, ATT_WRITE_COMMAND_ACCESS, cmd->handle, &elmt) == ATT_ERR_NO_ERROR))
    {
        // Check validity of the length and offset
        // If bad length, simply ignore the request
        if (atts_check_length_and_offset(&elmt, cmd->value_len, 0) == ATT_ERR_NO_ERROR)
        {
            // request write to be performed
            atts_update_elmt(conidx, cmd->value, cmd->value_len, 0, cmd->handle);
            // wait confirmation of upper layer before freeing the message
            msg_status = KE_MSG_NO_FREE;
        }
    }

    return (msg_status);
}

/**
 ****************************************************************************************
 * @brief Handles write response and constructs response.
 *
 * @param[in] conidx        Index of the connection with the peer device
 * @param[in] req           Message request
 *
 * @return If the message was consumed or not.
 *
 ****************************************************************************************
 */
static int atts_write_req(uint8_t conidx, struct l2cc_att_wr_req *req)
{
    uint8_t status = ATT_ERR_NO_ERROR;
    struct attm_elmt elmt = ATT_ELEMT_INIT;
    int msg_status = KE_MSG_CONSUMED;

    /* retrieve attribute +  check permission */
    status = atts_get_att_chk_perm(conidx, ATT_WRITE_ACCESS, req->handle, &elmt);

    if (status == ATT_ERR_NO_ERROR)
    {
        // Check validity of the length and offset
        status = atts_check_length_and_offset(&elmt, req->value_len, 0);
    }

    if (status == ATT_ERR_NO_ERROR)
    {
        //CRI: the update elem could return error if the profile task checked and sent error pdu
        //or could return ok, meaning the profile check went ok and response is sent here.

        // Update the attribute and allow write response sent here
        atts_update_elmt(conidx, req->value, req->value_len, 0, req->handle);
        //write/error response will be sent after check in profile

        msg_status = KE_MSG_NO_FREE;
    }
    else
    {
        /* read request not allowed */
        atts_send_error(conidx, req->code, req->handle, status);
    }

    return msg_status;
}


/**
 ****************************************************************************************
 * @brief Handles signed write command.
 *
 * @param[in] conidx        Index of the connection with the peer device
 * @param[in] cmd           Message command
 *
 * @return If the message was consumed or not.
 *
 ****************************************************************************************
 */
static int atts_write_signed(uint8_t conidx, struct l2cc_att_sign_wr_cmd *cmd)
{
    uint16_t actual_len;
    uint8_t status = ATT_ERR_NO_ERROR;
    struct attm_elmt elmt = ATT_ELEMT_INIT;
    int msg_status = KE_MSG_CONSUMED;

    // remove the signature length
    actual_len = (cmd->value_len - ATT_SIGNATURE_LEN);

    /* retrieve attribute +  check permission */
    status = atts_get_att_chk_perm(conidx, ATT_WRITE_SIGNED_ACCESS, cmd->handle, &elmt);

    if (status == ATT_ERR_NO_ERROR)
    {
        // Check validity of the length and offset
        status = atts_check_length_and_offset(&elmt, actual_len, 0);
    }

    // There is no error, check the signing part
    if (status == ATT_ERR_NO_ERROR)
    {
        // Allocate the signature check message for the SMP
        struct gapc_sign_cmd *sign_cmd = KE_MSG_ALLOC_DYN(GAPC_SIGN_CMD,
                                         KE_BUILD_ID(TASK_GAPC, conidx), KE_BUILD_ID(TASK_GATTC, conidx),
                                         gapc_sign_cmd, cmd->value_len + ATT_SIGNED_PDU_VAL_OFFSET);

        // Fill in the parameter structure
        sign_cmd->operation = GAPC_SIGN_CHECK;
        sign_cmd->byte_len  = cmd->value_len + ATT_SIGNED_PDU_VAL_OFFSET;

        sign_cmd->msg[0] = L2C_CODE_ATT_SIGN_WR_CMD;
        co_write16p(&sign_cmd->msg[1], cmd->handle);
        memcpy(&sign_cmd->msg[ATT_SIGNED_PDU_VAL_OFFSET], cmd->value, cmd->value_len);

        // Send the message
        ke_msg_send(sign_cmd);

        // wait end of signature verification procedure and write confirm by upper layers
        msg_status = KE_MSG_NO_FREE;
    }

    return msg_status;
}
/**
 ****************************************************************************************
 * @brief Handles prepare write request and constructs response.
 *
 * @param[in] conidx        Index of the connection with the peer device
 * @param[in] req           Message request
 *
 * @return If the message was consumed or not.
 *
 ****************************************************************************************
 */
static int atts_prepare_write_req(uint8_t conidx, struct l2cc_att_prep_wr_req *req)
{
    struct attm_elmt elmt = ATT_ELEMT_INIT;
    uint8_t status = ATT_ERR_NO_ERROR;
    uint16_t *current_length = ATTS_REMAINING_LEN(req);
    int msg_status = KE_MSG_CONSUMED;

    // prevent checking attribute permission twice
    if (*current_length == 0)
    {
        /* retrieve attribute +  check permission */
        status = atts_get_att_chk_perm(conidx, ATT_WRITE_ACCESS, req->handle, &elmt);
    }

    // Send an error response if required
    if (status == ATT_ERR_NO_ERROR)
    {
        // check if attribute is present in attributes values
        if (!atts_hdl_present_in_prep_data(conidx, req->handle) && (*current_length == 0))
        {
            // request upper layer to provide information about data to put in prepare write queue
            struct gattc_att_info_req_ind *info_req_ind = KE_MSG_ALLOC(GATTC_ATT_INFO_REQ_IND,
                    atts_get_dest_task(conidx, req->handle), KE_BUILD_ID(TASK_GATTC, conidx),
                    gattc_att_info_req_ind);

            // Fill the message parameters
            info_req_ind->handle   = req->handle;

            // And send the message
            ke_msg_send(info_req_ind);

            msg_status = KE_MSG_NO_FREE;
        }
        else
        {
            // message is reused in prepare write list, first remove from queue
            struct ke_msg *pdu_msg = (struct ke_msg *) co_list_pop_front(&(gattc_env[conidx]->server.pdu_queue));

            // Store the write parameters in the queue
            status = atts_store_prep_data(conidx, pdu_msg, req->value_len, req->offset, req->handle,
                                          PERM_GET(*current_length, MAX_LEN));

            if (status == ATT_ERR_NO_ERROR)
            {
                struct l2cc_att_prep_wr_rsp *prep_wr_rsp = ATTS_ALLOCATE_PDU(conidx,
                        L2C_CODE_ATT_PREP_WR_RSP, l2cc_att_prep_wr_rsp, req->value_len);

                // Prepare PDU structure
                prep_wr_rsp->handle = req->handle;
                prep_wr_rsp->value_len = req->value_len;
                memcpy(&prep_wr_rsp->value[0], req->value, prep_wr_rsp->value_len);
                prep_wr_rsp->offset = req->offset;

                /* construct and send the PDU */
                atts_send_pdu(prep_wr_rsp);

                msg_status = KE_MSG_SAVED;
            }
            else
            {
                // message not saved, push it in front of queue
                co_list_push_front(&(gattc_env[conidx]->server.pdu_queue), &(pdu_msg->hdr));
            }
        }
    }

    if (status != ATT_ERR_NO_ERROR)
    {
        atts_send_error(conidx, req->code, req->handle, status);
    }

    return msg_status;
}


/**
 ****************************************************************************************
 * @brief Handles execute write request and constructs response.
 *
 * @param[in] conidx        Index of the connection with the peer device
 * @param[in] req           Message request
 *
 * @return If the message was consumed or not.
 *
 ****************************************************************************************
 */
static int atts_execute_write_req(uint8_t conidx, struct l2cc_att_exe_wr_req *req)
{
    uint16_t *remaining_len = ATTS_REMAINING_LEN(req);
    int msg_status = KE_MSG_CONSUMED;
    uint8_t status = ATT_ERR_NO_ERROR;

    // Perform the correct action
    switch (req->flags)
    {
    case ATT_EXECUTE_ALL_PREPARED_WRITES:

        if (*remaining_len == 0)
        {
            // First check the validity of length and offset for all prepared writes
            if (!co_list_is_empty(&(gattc_env[conidx]->server.prep_wr_req_list)))
            {
                uint16_t handle = ATT_INVALID_HANDLE;
                uint16_t att_length = 0;

                struct atts_prep_data *prep = (struct atts_prep_data *) co_list_pick(&(gattc_env[conidx]->server.prep_wr_req_list));
                struct atts_prep_data *first_blk = NULL;

                // check if list is empty
                while (prep != NULL)
                {
                    struct attm_elmt elmt = ATT_ELEMT_INIT;
                    struct l2cc_pdu_recv_ind *p_ind = (struct l2cc_pdu_recv_ind *)prep->value;
                    struct l2cc_att_prep_wr_req *p_req = &p_ind->pdu.data.prep_wr_req;

                    // retrieve attribute.
                    status = attmdb_get_attribute(p_req->handle, &elmt);
                    // sanity check
                    ASSERT_INFO(status == ATT_ERR_NO_ERROR, p_req->handle, status);

                    // Check validity of the length and offset
                    status = atts_check_length_and_offset(&elmt, prep->len, prep->offset);
                    // ensure that this check is performed once.
                    *remaining_len = 1;
                    if (status == ATT_ERR_NO_ERROR)
                    {
                        // list is sorted by handle, now verify that data to write is valid:
                        // check that data to write is not internally overwritten or contains
                        // some hole.
                        if (handle != p_req->handle)
                        {
                            handle = p_req->handle;
                            // retrieve attribute length
                            att_length = prep->cur_len;

                            // keep first block of attribute value
                            first_blk = prep;
                            first_blk->total_len = 0;
                        }

                        // check that new offset is expected offset
                        if (prep->offset > att_length)
                        {
                            status = ATT_ERR_INVALID_OFFSET;
                        }
                        else
                        {
                            // check if attribute is not modified twice
                            if ((prep->offset < att_length) && first_blk != prep)
                            {
                                // update attribute length
                                prep->cur_len = att_length;
                                // update block size
                                first_blk = prep;
                                first_blk->total_len = prep->len;
                            }
                            else
                            {
                                // calculate new size of the attribute
                                att_length = prep->offset + prep->len;
                                // Update total length
                                first_blk->total_len += prep->len;
                            }
                        }
                    }

                    if (status != ATT_ERR_NO_ERROR)
                    {

                        atts_send_error(conidx, req->code, p_req->handle, status);

                        // Clear the prepare write entry
                        atts_clear_prep_data(conidx);

                        // stop execution
                        break;
                    }

                    prep = (struct atts_prep_data *) prep->hdr.next;
                }
            }
        }

        if (!co_list_is_empty(&(gattc_env[conidx]->server.prep_wr_req_list)))
        {
            uint16_t cursor = 0;
            struct atts_prep_data *prep = (struct atts_prep_data *)co_list_pick(&(gattc_env[conidx]->server.prep_wr_req_list));
            struct atts_prep_data *next = NULL;
            struct l2cc_pdu_recv_ind *p_ind = (struct l2cc_pdu_recv_ind *)prep->value;
            struct l2cc_att_prep_wr_req *p_req = &p_ind->pdu.data.prep_wr_req;
            // prepare write indication of a full handle
            struct gattc_write_req_ind *wr_ind = KE_MSG_ALLOC_DYN(GATTC_WRITE_REQ_IND,
                                                 atts_get_dest_task(conidx, p_req->handle), KE_BUILD_ID(TASK_GATTC, conidx),
                                                 gattc_write_req_ind, prep->total_len);

            // Fill the message parameters
            wr_ind->handle   = p_req->handle;
            wr_ind->length   = prep->total_len;
            wr_ind->offset   = prep->offset;

            // loop until all handle value prepared - handles also zero length attributes
            while ((prep != NULL) && ((cursor < wr_ind->length) || (wr_ind->length == 0)))
            {
                struct l2cc_pdu_recv_ind *p_ind = (struct l2cc_pdu_recv_ind *)prep->value;
                struct l2cc_att_prep_wr_req *p_req = &p_ind->pdu.data.prep_wr_req;

                if (wr_ind->handle == p_req->handle)
                {
                    // copy partial value
                    memcpy(&wr_ind->value[cursor], p_req->value, prep->len);
                    // update cursor
                    cursor += prep->len;
                    // go to next element
                    next = (struct atts_prep_data *) prep->hdr.next;
                    // pop the attribute element
                    co_list_pop_front(&(gattc_env[conidx]->server.prep_wr_req_list));
                    // free element
                    ke_free(prep);
                    prep = next;
                }
                else
                {
                    break;
                }
            }

            // And send the message
            ke_msg_send(wr_ind);

            // prepare write will continue later, wait for upper layer confirmation
            msg_status = KE_MSG_NO_FREE;
            break;
        }
    // no break

    case ATT_CANCEL_ALL_PREPARED_WRITES:
    default:
    {
        // Clear the prepare write entry
        atts_clear_prep_data(conidx);

        if (status == ATT_ERR_NO_ERROR)
        {
            struct l2cc_att_exe_wr_rsp *exe_wr_rsp = ATTS_ALLOCATE_PDU(conidx,
                    L2C_CODE_ATT_EXE_WR_RSP, l2cc_att_exe_wr_rsp, L2C_MIN_LE_MTUSIG);

            // Construct and send the PDU
            atts_send_pdu(exe_wr_rsp);
        }
    }
    break;
    }

    return (msg_status);
}


/**
 ****************************************************************************************
 * @brief Handles reception of Handle Value Confirmation PDU.
 *
 * @param[in] conidx        Index of the connection with the peer device
 * @param[in] cfm           Message confirmation
 *
 * @return If the message was consumed or not.
 *
 ****************************************************************************************
 */
static int atts_hdl_value_cfm(uint8_t conidx, struct l2cc_att_hdl_val_cfm *cfm)
{
    // clear timer if receive a confirmation message
    uint8_t op = gattc_get_operation(conidx, GATTC_OP_SERVER);

    if ((op == GATTC_INDICATE) || (op == GATTC_SVC_CHANGED))
    {
        // clear timeout of indication confirmation
        ke_timer_clear(GATTC_SERVER_RTX_IND, KE_BUILD_ID(TASK_GATTC, conidx));

        /* Indicate that command is correctly completed */
        gattc_send_complete_evt(conidx, GATTC_OP_SERVER, GAP_ERR_NO_ERROR);
    }
    // else ignore the packet

    return (KE_MSG_CONSUMED);
}



/// The default PDU handlers
const struct att_pdu_handler atts_handlers[] =
{
    { L2C_CODE_ATT_MTU_REQ, (att_func_t) atts_mtu_exc_req },
    { L2C_CODE_ATT_FIND_INFO_REQ, (att_func_t) atts_find_info_req },
    { L2C_CODE_ATT_FIND_BY_TYPE_REQ, (att_func_t) atts_find_by_type_req },
    { L2C_CODE_ATT_RD_BY_TYPE_REQ, (att_func_t) atts_read_by_type_req },
    { L2C_CODE_ATT_RD_BY_GRP_TYPE_REQ, (att_func_t) atts_read_by_grp_type_req },
    { L2C_CODE_ATT_RD_REQ, (att_func_t) atts_read_req },
#ifndef ATT_SLIM
    { L2C_CODE_ATT_RD_BLOB_REQ, (att_func_t) atts_read_blob_req },
    { L2C_CODE_ATT_RD_MULT_REQ, (att_func_t) atts_read_mult_req },
#endif // ATT_SLIM
    { L2C_CODE_ATT_WR_CMD, (att_func_t) atts_write_cmd },
    { L2C_CODE_ATT_WR_REQ, (att_func_t) atts_write_req },
#ifndef ATT_SLIM
    { L2C_CODE_ATT_SIGN_WR_CMD, (att_func_t) atts_write_signed },
#endif // ATT_SLIM
    { L2C_CODE_ATT_PREP_WR_REQ, (att_func_t) atts_prepare_write_req },
    { L2C_CODE_ATT_EXE_WR_REQ, (att_func_t) atts_execute_write_req },
};

void atts_process_pdu(uint8_t conidx)
{
    // get head of the pdu queue
    struct ke_msg *pdu_msg = (struct ke_msg *) co_list_pick(&(gattc_env[conidx]->server.pdu_queue));

    while (pdu_msg != NULL)
    {
        struct l2cc_pdu *pdu = &(((struct l2cc_pdu_recv_ind *) ke_msg2param(pdu_msg))->pdu);
        int status;
        int8_t cursor;
        att_func_t fhandler = NULL;

        // search PDU Handler
        for (cursor = ((sizeof(atts_handlers) / sizeof(struct att_pdu_handler)) - 1) ; cursor >= 0  ; cursor--)
        {
            if (atts_handlers[cursor].pdu_id == pdu->data.code)
            {
                fhandler = atts_handlers[cursor].handler;
                break;
            }
        }

        ASSERT_INFO(fhandler != NULL, conidx, pdu->data.code);

        // execute PDU Handler
        status = fhandler(conidx, &(pdu->data.code));

        switch (status)
        {
        // ATTS execution is over
        case KE_MSG_CONSUMED:
        {
            // Clear Response data
            atts_clear_rsp_data(conidx);

            // clean-up received PDU
            ke_msg_free((struct ke_msg *) co_list_pop_front(&(gattc_env[conidx]->server.pdu_queue)));
        }
        // no break
        case KE_MSG_SAVED://  ATTS execution is over but message is used for later
        {
            // look at next message
            pdu_msg = (struct ke_msg *) co_list_pick(&(gattc_env[conidx]->server.pdu_queue));
        }
        break;
        // ATTS execution is over but message is used for later
        case KE_MSG_NO_FREE:
        {
            pdu_msg = NULL;
            // do nothing and return
        }
        break;

        default:
        {
            ASSERT_INFO(0, conidx, status);
        }
        break;
        }
    }
}
#endif /* (BLE_ATTS) */



int atts_l2cc_pdu_recv_handler(uint8_t conidx, struct l2cc_pdu_recv_ind *param)
{
    // means that message cannot be handled by ATTS module
    int msg_status = ATT_PDU_HANDLER_NOT_FOUND;

#if (BLE_ATTS)

    // specific handling of value confirm message
    if (param->pdu.data.code == L2C_CODE_ATT_HDL_VAL_CFM)
    {
        // Indication confirmation
        msg_status = atts_hdl_value_cfm(conidx, &(param->pdu.data.hdl_val_cfm));
    }
    else
    {
        int8_t cursor;

        // search PDU Handler
        for (cursor = ((sizeof(atts_handlers) / sizeof(struct att_pdu_handler)) - 1) ; cursor >= 0  ; cursor--)
        {
            if (atts_handlers[cursor].pdu_id == param->pdu.data.code)
            {
                // check if ATTS is ready to execute request, it can do it only if new pdu is put on head of queue
                bool ready_to_execute = co_list_is_empty(&(gattc_env[conidx]->server.pdu_queue));
                // message saved if connection confirmation message has not been received yet, ATTS Not Ready
                if (ke_state_get(KE_BUILD_ID(TASK_GATTC, conidx)) & GATTC_CONNECTED)
                {
                    ready_to_execute = false;
                }
                // save PDU at end of the list
                co_list_push_back(&(gattc_env[conidx]->server.pdu_queue), &(ke_param2msg(param)->hdr));

                // this parameter is used as a temporary variable in handlers to recover state.
                // Init it to zero when starting process execution
                param->pdu.payld_len = 0;
                // mark message to not be freed
                msg_status = KE_MSG_NO_FREE;

                // if message can be process
                if (ready_to_execute)
                {
                    atts_process_pdu(conidx);
                }

                break;
            }
        }
    }

#else // (!BLE_ATTS)
    switch (param->pdu.data.code)
    {
    case L2C_CODE_ATT_HDL_VAL_CFM:
    case L2C_CODE_ATT_MTU_REQ:
    case L2C_CODE_ATT_FIND_INFO_REQ:
    case L2C_CODE_ATT_FIND_BY_TYPE_REQ:
    case L2C_CODE_ATT_RD_BY_TYPE_REQ:
    case L2C_CODE_ATT_RD_BY_GRP_TYPE_REQ:
    case L2C_CODE_ATT_RD_REQ:
    case L2C_CODE_ATT_RD_BLOB_REQ:
    case L2C_CODE_ATT_RD_MULT_REQ:
    case L2C_CODE_ATT_WR_CMD:
    case L2C_CODE_ATT_WR_REQ:
    case L2C_CODE_ATT_SIGN_WR_CMD:
    case L2C_CODE_ATT_PREP_WR_REQ:
    case L2C_CODE_ATT_EXE_WR_REQ:
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
        msg_status = KE_MSG_CONSUMED;
    }
    break;

    default: /* Do Nothing - ignore packet */
        break;
    };
#endif // (BLE_ATTS)

    return (msg_status);
}


#endif // (BLE_CENTRAL || BLE_PERIPHERAL)

/// @} ATTS
