/**
 ****************************************************************************************
 *
 * @file attm.c
 *
 * @brief Attribute Manager implementation.
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup ATTM
 * @{
 ****************************************************************************************
 */
#include "rwip_config.h"

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#if (BLE_CENTRAL || BLE_PERIPHERAL)
#include <stdint.h>
#include "attm.h"
#include "co_utils.h"
#include "ke_mem.h"
#include "gattm.h"

#include "../gattm/gattm_int.h" // Access to the internal variable required


/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */


/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/**
 * Convert attribute permissions to attribute properties
 *
 * @param[in] perm Attribute permission
 *
 * @return attribute properties
 */
static uint8_t attmdb_perm2prop(uint16_t perm)
{
    return PERM_GET(perm, PROP);
}

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */


bool attm_uuid_comp(uint8_t *uuid_a, uint8_t uuid_a_len,
                    uint8_t *uuid_b, uint8_t uuid_b_len)
{
    bool match_id  = false;

    // Both are 16 bits UUIDs
    if ((uuid_a_len == ATT_UUID_16_LEN) && (uuid_b_len == ATT_UUID_16_LEN))
    {
        // compare both 16 bits uuids
        if (memcmp(uuid_a, uuid_b, ATT_UUID_16_LEN) == 0)
        {
            match_id = true;
        }
    }
    // Both are 32 bits  UUIDs
    else if ((uuid_a_len == ATT_UUID_32_LEN) && (uuid_b_len == ATT_UUID_32_LEN))
    {
        // compare both 32 bits uuids
        if (memcmp(uuid_a, uuid_b, ATT_UUID_32_LEN) == 0)
        {
            match_id = true;
        }
    }
    // One of UUIDs is a 128 bits UUID
    else if ((uuid_a_len == ATT_UUID_128_LEN) || (uuid_b_len == ATT_UUID_128_LEN))
    {
        uint8_t uuid128_a[ATT_UUID_128_LEN];
        uint8_t uuid128_b[ATT_UUID_128_LEN];

        // convert both uuid to 128 bits
        attm_convert_to128(uuid128_a, uuid_a, uuid_a_len);
        attm_convert_to128(uuid128_b, uuid_b, uuid_b_len);

        // compare both 128 bits uuids
        if (memcmp(uuid128_a, uuid128_b, ATT_UUID_128_LEN) == 0)
        {
            match_id = true;
        }
    }

    return match_id;
}

bool attm_uuid16_comp(uint8_t *uuid_a, uint8_t uuid_a_len, uint16_t uuid_b)
{
    return attm_uuid_comp(uuid_a, uuid_a_len, (uint8_t *)&uuid_b, ATT_UUID_16_LEN);
}

bool attm_is_bt16_uuid(uint8_t *uuid)
{
    uint8_t auc_128UUIDBase[ATT_UUID_128_LEN] = ATT_BT_UUID_128;

    /* place the UUID on 12th and 13th location of UUID */
    memcpy(&(auc_128UUIDBase[12]), &(uuid[12]), ATT_UUID_16_LEN);

    return attm_uuid_comp(auc_128UUIDBase, ATT_UUID_128_LEN, uuid, ATT_UUID_128_LEN);
}

bool attm_is_bt32_uuid(uint8_t *uuid)
{
    uint8_t auc_128UUIDBase[ATT_UUID_128_LEN] = ATT_BT_UUID_128;

    /* place the UUID on last 4 bits location of 32 BITS UUID */
    memcpy(&(auc_128UUIDBase[12]), &(uuid[12]), ATT_UUID_32_LEN);

    return attm_uuid_comp(auc_128UUIDBase, ATT_UUID_128_LEN, uuid, ATT_UUID_128_LEN);
}


void attm_convert_to128(uint8_t *uuid128, uint8_t *uuid, uint8_t uuid_len)
{
    uint8_t auc_128UUIDBase[ATT_UUID_128_LEN] = ATT_BT_UUID_128;
    uint8_t cursor = 0;

    if ((uuid_len == ATT_UUID_32_LEN) || (uuid_len == ATT_UUID_16_LEN))
    {
        /* place the UUID on 12th to 15th location of UUID */
        cursor = 12;
    }
    else
    {
        /* we consider it's 128 bits UUID */
        uuid_len  = ATT_UUID_128_LEN;
    }

    /* place the UUID on 12th to 15th location of UUID */
    memcpy(&(auc_128UUIDBase[cursor]), uuid, uuid_len);

    /* update value */
    memcpy(&uuid128[0], &auc_128UUIDBase[0], ATT_UUID_128_LEN);
}


#if (BLE_ATTS)

uint8_t attm_svc_create_db(uint16_t *shdl, uint16_t uuid, uint8_t *cfg_flag, uint8_t max_nb_att,
                           uint8_t *att_tbl, ke_task_id_t const dest_id,
                           const struct attm_desc *att_db, uint8_t svc_perm)
{
    uint8_t nb_att = 0;
    uint8_t i;
    uint8_t status = ATT_ERR_NO_ERROR;
    struct gattm_svc_desc *svc_desc;
    svc_perm |= PERM(SVC_MI, ENABLE);

    //Compute number of attributes and maximal payload size
    for (i = 1; i < max_nb_att; i++)
    {
        // check within db_cfg flag if attribute is enabled or not
        if ((cfg_flag == NULL) || (((cfg_flag[i / 8] >> (i % 8)) & 1) == 1))
        {
            // increment number of attribute to add
            nb_att++;
        }
    }

    // Allocate service information
    svc_desc = (struct gattm_svc_desc *) ke_malloc(
                   sizeof(struct gattm_svc_desc) + (sizeof(struct gattm_att_desc) * (nb_att)), KE_MEM_NON_RETENTION);

    // Initialize service info
    svc_desc->start_hdl = *shdl;
    svc_desc->nb_att = nb_att;
    svc_desc->task_id = dest_id;
    // ensure that service has a 16 bits UUID
    svc_desc->perm = (svc_perm & ~(PERM_MASK_SVC_UUID_LEN));
    // copy UUID
    memcpy(svc_desc->uuid, &uuid, ATT_UUID_16_LEN);

    // Set Attribute parameters
    nb_att = 0;
    for (i = 1; i < max_nb_att; i++)
    {
        // check within db_cfg flag if attribute is enabled or not
        if ((cfg_flag == NULL) || (((cfg_flag[i / 8] >> (i % 8)) & 1) == 1))
        {
            // fill attribute configuration

            // ensure that service has a 16 bits UUID
            svc_desc->atts[nb_att].max_len  = att_db[i].max_size;
            svc_desc->atts[nb_att].ext_perm = (att_db[i].ext_perm & ~PERM_MASK_UUID_LEN);
            svc_desc->atts[nb_att].perm     = att_db[i].perm;
            // copy UUID
            memcpy(svc_desc->atts[nb_att].uuid, &(att_db[i].uuid), ATT_UUID_16_LEN);

            // increment number of attributes
            nb_att++;
        }
    }

    // add service in database
    status = attmdb_add_service(svc_desc);

    // if service added
    if (status == ATT_ERR_NO_ERROR)
    {
        // return start handle
        *shdl = svc_desc->start_hdl;

        // update attribute table mapping
        nb_att = 0;
        for (i = 0; (i < max_nb_att) && (att_tbl != NULL); i++)
        {
            // check within db_cfg flag if attribute is enabled or not
            if ((cfg_flag == NULL) || (((cfg_flag[i / 8] >> (i % 8)) & 1) == 1))
            {
                //Save handle offset in attribute table
                att_tbl[i] = *shdl + nb_att;
                // increment number of attributes
                nb_att++;
            }
        }
    }

    // free service description
    ke_free(svc_desc);

    return (status);
}

uint8_t attm_svc_create_db_128(uint16_t *shdl, const uint8_t *uuid, uint8_t *cfg_flag, uint8_t max_nb_att,
                               uint8_t *att_tbl, ke_task_id_t const dest_id,
                               const struct attm_desc_128 *att_db, uint8_t svc_perm)
{
    uint8_t nb_att = 0;
    uint8_t i;
    uint8_t status = ATT_ERR_NO_ERROR;
    struct gattm_svc_desc *svc_desc;
    svc_perm |= PERM(SVC_MI, ENABLE);

    //Compute number of attributes and maximal payload size
    for (i = 1; i < max_nb_att; i++)
    {
        // check within db_cfg flag if attribute is enabled or not
        if ((cfg_flag == NULL) || (((cfg_flag[i / 8] >> (i % 8)) & 1) == 1))
        {
            // increment number of attribute to add
            nb_att++;
        }
    }

    // Allocate service information
    svc_desc = (struct gattm_svc_desc *) ke_malloc(
                   sizeof(struct gattm_svc_desc) + (sizeof(struct gattm_att_desc) * (nb_att)), KE_MEM_NON_RETENTION);

    // Initialize service info
    svc_desc->start_hdl = *shdl;
    svc_desc->nb_att = nb_att;
    svc_desc->task_id = dest_id;
    // ensure that service has a 16 bits UUID
    svc_desc->perm = svc_perm;

    // copy UUID
    memcpy(svc_desc->uuid, uuid,
           ((PERM_GET(svc_perm, SVC_UUID_LEN) == PERM_UUID_16) ? ATT_UUID_16_LEN
            : ((PERM_GET(svc_perm, SVC_UUID_LEN) == PERM_UUID_32) ? ATT_UUID_32_LEN
               : ATT_UUID_128_LEN)));

    // Set Attribute parameters
    nb_att = 0;
    for (i = 1; i < max_nb_att; i++)
    {
        // check within db_cfg flag if attribute is enabled or not
        if ((cfg_flag == NULL) || (((cfg_flag[i / 8] >> (i % 8)) & 1) == 1))
        {
            // fill attribute configuration

            // ensure that service has a 16 bits UUID
            svc_desc->atts[nb_att].max_len  = att_db[i].max_size;
            svc_desc->atts[nb_att].ext_perm = att_db[i].ext_perm;
            svc_desc->atts[nb_att].perm     = att_db[i].perm;
            // copy UUID
            memcpy(svc_desc->atts[nb_att].uuid, &(att_db[i].uuid),
                   ((PERM_GET(att_db[i].ext_perm, UUID_LEN) == PERM_UUID_16) ? ATT_UUID_16_LEN
                    : ((PERM_GET(att_db[i].ext_perm, UUID_LEN) == PERM_UUID_32) ? ATT_UUID_32_LEN
                       : ATT_UUID_128_LEN)));

            // increment number of attributes
            nb_att++;
        }
    }

    // add service in database
    status = attmdb_add_service(svc_desc);

    // if service added
    if (status == ATT_ERR_NO_ERROR)
    {
        // return start handle
        *shdl = svc_desc->start_hdl;

        // update attribute table mapping
        nb_att = 0;
        for (i = 0; (i < max_nb_att) && (att_tbl != NULL); i++)
        {
            // check within db_cfg flag if attribute is enabled or not
            if ((cfg_flag == NULL) || (((cfg_flag[i / 8] >> (i % 8)) & 1) == 1))
            {
                //Save handle offset in attribute table
                att_tbl[i] = *shdl + nb_att;
                // increment number of attributes
                nb_att++;
            }
        }
    }

    // free service description
    ke_free(svc_desc);

    return (status);
}


uint8_t attm_reserve_handle_range(uint16_t *start_hdl, uint8_t nb_att)
{
    struct gattm_svc_desc svc_desc;
    uint8_t status;

    // create a fake service description containing handle range to allocate
    svc_desc.start_hdl = *start_hdl;
    svc_desc.nb_att    = nb_att - 1;

    // use check service handle to verify if the handle range can be allocated
    status = attmdb_svc_check_hdl(&svc_desc);

    // update service start handle that should be used for service allocation
    *start_hdl = svc_desc.start_hdl;

    //return if service range can be allocated or not.
    return (status);
}

uint8_t attm_att_set_value(uint16_t handle, att_size_t length, att_size_t offset, uint8_t *value)
{
    struct attm_elmt elmt = ATT_ELEMT_INIT;
    uint8_t status = attmdb_get_attribute(handle, &elmt);

    if (status == ATT_ERR_NO_ERROR)
    {
        // value can not be set for following parameters
        if (attmdb_uuid16_comp(&elmt, ATT_DECL_PRIMARY_SERVICE)
                || attmdb_uuid16_comp(&elmt, ATT_DECL_SECONDARY_SERVICE)
                || attmdb_uuid16_comp(&elmt, ATT_DECL_CHARACTERISTIC)
                || attmdb_uuid16_comp(&elmt, ATT_DECL_INCLUDE)
                || attmdb_uuid16_comp(&elmt, ATT_DESC_CHAR_EXT_PROPERTIES)
                || attmdb_uuid16_comp(&elmt, ATT_DESC_CLIENT_CHAR_CFG)
                || attmdb_uuid16_comp(&elmt, ATT_DESC_SERVER_CHAR_CFG))
        {
            status = ATT_ERR_REQUEST_NOT_SUPPORTED;
        }
        else
        {

            // check if data present in attribute
            if (PERM_GET(elmt.info.att->info.max_length, RI) == 0)
            {
                struct attm_att_value *val = (struct attm_att_value *)
                                             (((uint8_t *)elmt.info.att) + PERM_GET(elmt.info.att->info.offset, VAL_OFFSET));


                if ((offset) > val->length)
                {
                    status = ATT_ERR_INVALID_OFFSET;
                }
                /* check if value length is not too big */
                if ((length + offset) > val->max_length)
                {
                    status = ATT_ERR_INVALID_ATTRIBUTE_VAL_LEN;
                }
                /* update attribute value + length */
                else
                {
                    // copy data
                    memcpy(&(val->value[offset]), value, length);
                    val->length = length;
                }
            }
            // attribute value cannot be set
            else
            {
                status = ATT_ERR_REQUEST_NOT_SUPPORTED;
            }
        }
    }

    return (status);
}


uint8_t attm_get_value(uint16_t handle, att_size_t *length, uint8_t **value)
{
    uint8_t uuid_len;
    uint8_t uuid[ATT_UUID_128_LEN];

    struct attm_elmt elmt = ATT_ELEMT_INIT;
    uint8_t status = attmdb_get_attribute(handle, &elmt);
    struct attm_att_value *val;

    if (status == ATT_ERR_NO_ERROR)
    {
        if (attmdb_uuid16_comp(&elmt, ATT_DECL_PRIMARY_SERVICE)
                || attmdb_uuid16_comp(&elmt, ATT_DECL_SECONDARY_SERVICE))

        {
            // Retrieve service UUID
            status = attmdb_get_uuid(&elmt, &uuid_len, uuid, true, true);

            if (status == ATT_ERR_NO_ERROR)
            {
                *length = uuid_len;
                // copy uuid
                memcpy(gattm_env.db.temp_val, uuid, uuid_len);
                *value = gattm_env.db.temp_val;
            }
        }
        else if (attmdb_uuid16_comp(&elmt, ATT_DECL_CHARACTERISTIC))
        {
            // retrieve targeted characteristic value
            struct attm_elmt char_val = ATT_ELEMT_INIT;
            // value handle is just after characteristic.
            handle = handle + 1;
            status = attmdb_get_attribute(handle, &char_val);

            if (status == ATT_ERR_NO_ERROR)
            {
                // Retrieve targeted UUID
                attmdb_get_uuid(&char_val, &uuid_len, uuid, false, true);
                // set value length: prop + handle + UUID Length
                *length = ATT_PROP_LEN + ATT_HANDLE_LEN + uuid_len;
                // copy property
                *gattm_env.db.temp_val = attmdb_perm2prop(char_val.info.att->perm);
                // copy handle
                memcpy(&(gattm_env.db.temp_val[ATT_PROP_LEN]),  &(handle), ATT_HANDLE_LEN);
                // copy UUID
                memcpy(&(gattm_env.db.temp_val[ATT_HANDLE_LEN + ATT_PROP_LEN]), uuid, uuid_len);
                *value = gattm_env.db.temp_val;
            }
            else
            {
                // Database not correctly initialized by Application
                status = ATT_ERR_APP_ERROR;
            }
        }
        else if (attmdb_uuid16_comp(&elmt, ATT_DECL_INCLUDE))
        {
            // retrieve value pointer
            val = (struct attm_att_value *)(((uint8_t *)elmt.info.att) +  PERM_GET(elmt.info.att->info.offset, VAL_OFFSET));
            // copy included service value
            memcpy(&handle, val->value, sizeof(uint16_t));
            // retrieve targeted service value
            struct attm_elmt svc_val = ATT_ELEMT_INIT;
            status = attmdb_get_attribute(handle, &svc_val);

            if (status == ATT_ERR_NO_ERROR)
            {
                // Retrieve targeted UUID
                attmdb_get_uuid(&svc_val, &uuid_len, uuid, true, true);
                // set value length: start handle + end handle + UUID Length
                *length = 2 * ATT_HANDLE_LEN + uuid_len;
                // copy start handle
                memcpy(&(gattm_env.db.temp_val[0]),  &(svc_val.info.svc->start_hdl), ATT_HANDLE_LEN);
                // copy end handle
                memcpy(&(gattm_env.db.temp_val[ATT_HANDLE_LEN]),  &(svc_val.info.svc->end_hdl), ATT_HANDLE_LEN);
                // copy UUID
                memcpy(&(gattm_env.db.temp_val[2 * ATT_HANDLE_LEN]), uuid, uuid_len);
                *value = gattm_env.db.temp_val;
            }
            else
            {
                // Database not correctly initialized by Application
                status = ATT_ERR_APP_ERROR;
            }
        }
        // extended char properties in contains 2 bytes present in attribute description
        else if (attmdb_uuid16_comp(&elmt, ATT_DESC_CHAR_EXT_PROPERTIES))
        {
            // retrieve value pointer
            val = (struct attm_att_value *)(((uint8_t *)elmt.info.att) + PERM_GET(elmt.info.att->info.offset, VAL_OFFSET));

            *length = sizeof(uint16_t);
            *value  = val->value;
        }
        else
        {
            // check if data present in attribute
            if (PERM_GET(elmt.info.att->info.max_length, RI) == 0)
            {
                // retrieve value pointer
                val = (struct attm_att_value *)(((uint8_t *)elmt.info.att) +  PERM_GET(elmt.info.att->info.offset, VAL_OFFSET));

                *value  = val->value;
                *length = val->length;
            }
            // attribute value cannot be read in database
            else
            {
                status = ATT_ERR_REQUEST_NOT_SUPPORTED;
            }
        }
    }

    return (status);
}

uint8_t attm_att_set_permission(uint16_t handle, uint16_t perm, uint16_t ext_perm)
{
    struct attm_elmt elmt = ATT_ELEMT_INIT;
    uint8_t status = attmdb_get_attribute(handle, &elmt);

    if (status == ATT_ERR_NO_ERROR)
    {
        if (attmdb_uuid16_comp(&elmt, ATT_DECL_PRIMARY_SERVICE)
                || attmdb_uuid16_comp(&elmt, ATT_DECL_SECONDARY_SERVICE)
                || attmdb_uuid16_comp(&elmt, ATT_DECL_CHARACTERISTIC)
                || attmdb_uuid16_comp(&elmt, ATT_DECL_INCLUDE)
                || attmdb_uuid16_comp(&elmt, ATT_DESC_CHAR_EXT_PROPERTIES))

        {
            status = ATT_ERR_REQUEST_NOT_SUPPORTED;
        }
        else
        {
            struct attm_att_desc *att = elmt.info.att;

            /* update attribute permissions */
            att->perm = perm;

            /* update attribute extended permissions */
            if (PERM_GET(ext_perm, EKS))
            {
                att->info.max_length |= PERM_MASK_EKS;
            }
            else
            {
                att->info.max_length &= ~PERM_MASK_EKS;
            }
        }
    }

    return (status);
}

uint8_t attm_att_update_perm(uint16_t handle, uint16_t access_mask, uint16_t perm)
{
    struct attm_elmt elmt = ATT_ELEMT_INIT;
    uint8_t status = attmdb_get_attribute(handle, &elmt);

    if (status == ATT_ERR_NO_ERROR)
    {
        if (attmdb_uuid16_comp(&elmt, ATT_DECL_PRIMARY_SERVICE)
                || attmdb_uuid16_comp(&elmt, ATT_DECL_SECONDARY_SERVICE)
                || attmdb_uuid16_comp(&elmt, ATT_DECL_CHARACTERISTIC)
                || attmdb_uuid16_comp(&elmt, ATT_DECL_INCLUDE)
                || attmdb_uuid16_comp(&elmt, ATT_DESC_CHAR_EXT_PROPERTIES))

        {
            status = ATT_ERR_REQUEST_NOT_SUPPORTED;
        }
        else
        {
            /* update attribute permissions (UUID Length not included) */
            elmt.info.att->perm = (elmt.info.att->perm & (~access_mask)) | perm;
        }
    }

    return (status);
}

uint8_t attm_svc_set_permission(uint16_t handle, uint8_t perm)
{
    uint8_t status = ATT_ERR_NO_ERROR;
    struct attm_svc *current_svc = attmdb_get_service(handle);

    /* if handle doesn't exists, trigger an error. */
    if (current_svc == NULL)
    {
        status = ATT_ERR_INVALID_HANDLE;
    }
    else
    {
        /* update attribute permissions */
        current_svc->svc.perm = (perm & ~PERM_MASK_SVC_UUID_LEN)
                                | PERM_GET(current_svc->svc.perm, SVC_UUID_LEN);
    }

    return (status);
}

uint8_t attm_svc_get_permission(uint16_t handle, uint8_t *perm)
{
    uint8_t status = ATT_ERR_NO_ERROR;
    struct attm_svc *current_svc = attmdb_get_service(handle);

    /* if handle doesn't exists, trigger an error. */
    if (current_svc == NULL)
    {
        status = ATT_ERR_INVALID_HANDLE;
    }
    else
    {
        /* update parameters to return */
        *perm = (current_svc->svc.perm & ~PERM_MASK_SVC_UUID_LEN);
    }

    return (status);
}

void attm_init(bool reset)
{
    if (!reset)
    {
        // Initialize DB pointers
        gattm_env.db.svcs = NULL;
        gattm_env.db.cache = NULL;
    }
    else
    {
        /* when initializing database, clear database */
        attmdb_destroy();
    }
}
#endif // (BLE_ATTS)
#endif // #if (BLE_CENTRAL || BLE_PERIPHERAL)
/// @} ATTM
