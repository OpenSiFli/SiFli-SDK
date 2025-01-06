/**
 ****************************************************************************************
 *
 * @file attm_db.c
 *
 * @brief Attribute Server database.
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup ATTDB
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"
#if (BLE_ATTS)
#include <stdlib.h>
#include "ke_mem.h"
#include "co_math.h"

#include "attm.h"
#include "attm_db.h"

#include "gap.h"
#include "gapm.h"
#include "gattm.h"

#include "../gattm/gattm_int.h" // Access to the internal variable required

/*
 * DEFINES
 ****************************************************************************************
 */

/// retrieve attribute value using handle in a service
#define ATT_GET_ELMT(_svc, _hdl) \
    (&((_svc)->atts[(_hdl) - ((_svc)->svc.start_hdl + 1)]))

// Perform UUID comparison
#define ATTR_COMP_UUID(_att, _uuid_len, _uuid)\
   (attm_uuid16_comp((_att).uuid, _uuid_len, (_uuid)))

/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

/**
 * Retrieve Length of memory block to allocate according to service description.
 * this function perform a sanity check of service and attributes UUIDs
 *
 * @param[in|out] svc_desc Service information to add in DB
 * @param[out]    size     Length of memory block to allocate
 *
 * @return status of operation
 */
static uint8_t attmdb_svc_calc_len(struct gattm_svc_desc *svc_desc, uint16_t *size)
{
    uint8_t i;
    uint8_t status = ATT_ERR_NO_ERROR;
    // check service uuid length
    uint8_t uuid_len = ATT_UUID_LEN(PERM_GET(svc_desc->perm, SVC_UUID_LEN));
    *size = sizeof(struct attm_svc);

    // 32 bits UUID
    if (uuid_len == ATT_UUID_32_LEN)
    {
        // 4 bytes required for 32 bits UUID in SVC memory block
        *size += ATT_UUID_32_LEN;
    }
    // 128 bits UUID
    else if (uuid_len == ATT_UUID_128_LEN)
    {
        // check if its not a 16 bits UUID into a 128 bits UUID
        if (attm_is_bt16_uuid(svc_desc->uuid))
        {
            // change uuid length
            svc_desc->perm &= ~PERM_MASK_SVC_UUID_LEN;
            svc_desc->perm |= PERM_UUID_16 << PERM_POS_SVC_UUID_LEN;

            // Retrieve UUID 16 and set it in UUID parameter
            memcpy(&(svc_desc->uuid[0]), &(svc_desc->uuid[12]), ATT_UUID_16_LEN);
        }
        else if (attm_is_bt32_uuid(svc_desc->uuid))
        {
            // 4 bytes required for 32 bits UUID in SVC memory block
            *size += ATT_UUID_32_LEN;

            // change uuid length
            svc_desc->perm &= ~PERM_MASK_SVC_UUID_LEN;
            svc_desc->perm |= PERM_UUID_32 << PERM_POS_SVC_UUID_LEN;

            // Retrieve UUID 32 and set it in UUID parameter
            memcpy(&(svc_desc->uuid[0]), &(svc_desc->uuid[12]), ATT_UUID_32_LEN);
        }
        else
        {
            // 16 bytes required for 128 bits UUID in SVC memory block
            *size += ATT_UUID_128_LEN;
        }
    }
    //
    else if (uuid_len == 0)
    {
        status = GAP_ERR_INVALID_PARAM;
    }

    // 2. Browse each attributes
    for (i = 0 ; (i < svc_desc->nb_att) && (status == ATT_ERR_NO_ERROR); i++)
    {
        uuid_len = ATT_UUID_LEN(PERM_GET(svc_desc->atts[i].ext_perm, UUID_LEN));

        // increment size of memory block
        *size += sizeof(struct attm_att_desc);

        // 2.1 Attributes shall not be services (primary or secondary)
        if ((uuid_len == 0) || ATTR_COMP_UUID(svc_desc->atts[i], uuid_len, ATT_DECL_PRIMARY_SERVICE)
                || ATTR_COMP_UUID(svc_desc->atts[i], uuid_len, ATT_DECL_SECONDARY_SERVICE))
        {
            status = GAP_ERR_INVALID_PARAM;
            break;
        }

        // 2.2 check permissions of know attributes (CCC, SCC, EXT, CharDecl, ...)

        // Client and Service Configuration
        else if ((ATTR_COMP_UUID(svc_desc->atts[i], uuid_len, ATT_DESC_CLIENT_CHAR_CFG))
                 || (ATTR_COMP_UUID(svc_desc->atts[i], uuid_len, ATT_DESC_SERVER_CHAR_CFG)))
        {
            // Read without authentication and Write perm mandatory
            if ((PERM_GET(svc_desc->atts[i].perm, RD) != PERM_RIGHT_ENABLE)
                    || (PERM_GET(svc_desc->atts[i].perm, WRITE_REQ) == PERM_RIGHT_DISABLE))
            {
                status = GATT_ERR_INVALID_PERM;
                break;
            } // note, other perm will be resets.
        }
        // 2.3 Allocate data for included service and extended properties
        else if (ATTR_COMP_UUID(svc_desc->atts[i], uuid_len, ATT_DECL_INCLUDE)
                 || ATTR_COMP_UUID(svc_desc->atts[i], uuid_len, ATT_DESC_CHAR_EXT_PROPERTIES))
        {
            // Allocate memory for having value in service memory block
            // Ensure that size can be allocated into a 16 bits boundary
            *size += CO_ALIGN2_HI(sizeof(struct attm_att_value) + sizeof(uint16_t));
        }

        // 2.4 According to UUID size and value max size and permissions calculate size
        //     needed for the attribute.
        else if (!ATTR_COMP_UUID(svc_desc->atts[i], uuid_len, ATT_DECL_CHARACTERISTIC))
        {
            // 32 bits UUID
            if (uuid_len == ATT_UUID_32_LEN)
            {
                // 4 bytes required for 32 bits UUID in SVC memory block
                *size += ATT_UUID_32_LEN;
            }
            // 128 bits UUID
            else if (uuid_len == ATT_UUID_128_LEN)
            {
                // check if its not a 16 bits UUID into a 128 bits UUID
                if (attm_is_bt16_uuid(svc_desc->atts[i].uuid))
                {
                    // change uuid length
                    svc_desc->atts[i].ext_perm &= ~PERM_MASK_UUID_LEN;
                    svc_desc->atts[i].ext_perm |= PERM_UUID_16 << PERM_POS_UUID_LEN;

                    // Retrieve UUID 16 and set it in UUID parameter
                    memcpy(&(svc_desc->atts[i].uuid[0]), &(svc_desc->atts[i].uuid[12]), ATT_UUID_16_LEN);
                }
                else if (attm_is_bt32_uuid(svc_desc->atts[i].uuid))
                {
                    // 4 bytes required for 32 bits UUID in SVC memory block
                    *size += ATT_UUID_32_LEN;

                    // change uuid length
                    svc_desc->atts[i].ext_perm &= ~PERM_MASK_UUID_LEN;
                    svc_desc->atts[i].ext_perm |= PERM_UUID_32 << PERM_POS_UUID_LEN;

                    // Retrieve UUID 32 and set it in UUID parameter
                    memcpy(&(svc_desc->atts[i].uuid[0]), &(svc_desc->atts[i].uuid[12]), ATT_UUID_32_LEN);
                }
                else
                {
                    // 16 bytes required for 128 bits UUID in SVC memory block
                    *size += ATT_UUID_128_LEN;
                }
            }

            // check if value shall be put in
            if (PERM_GET(svc_desc->atts[i].ext_perm, RI) == 0)
            {
                // Allocate memory for having value in service memory block
                // Ensure that size can be allocated into a 16 bits boundary
                *size += CO_ALIGN2_HI(sizeof(struct attm_att_value) + PERM_GET(svc_desc->atts[i].max_len, MAX_LEN));
            }
        }
    }

    return (status);
}

/**
 * Initialize service information in allocated service memory block
 *
 * @param[in]     svc_desc Service information to add in DB
 * @param[in|out] new_svc  New Allocated Service pointer
 * @param[in]     size     Length of memory block allocated
 * @param[in]     calc_hdl Calculate handle offsets
 *
 */
static void attmdb_svc_init(struct gattm_svc_desc *svc_desc, struct attm_svc *new_svc, uint16_t size, bool calc_hdl)
{
    uint8_t i;
    uint8_t uuid_len;
    // payload cursor
    uint8_t *p_cursor;
    struct attm_att_value *att_value;

    // put payload cursor at end of data.
    p_cursor = ((uint8_t *) new_svc) + size;

    // 1. Initialize service information
    new_svc->svc.start_hdl   = svc_desc->start_hdl;
    // service attribute should be taken in account
    new_svc->svc.end_hdl     = svc_desc->start_hdl + svc_desc->nb_att;
    new_svc->svc.task_id     = svc_desc->task_id;
    new_svc->svc.perm        = svc_desc->perm;
    new_svc->svc.ext_perm    = 0;

    // set UUID value
    uuid_len = ATT_UUID_LEN(PERM_GET(svc_desc->perm, SVC_UUID_LEN));
    switch (uuid_len)
    {
    case ATT_UUID_16_LEN:
    {
        // Copy 16 bits UUID
        memcpy(&(new_svc->svc.uuid), &(svc_desc->uuid[0]), ATT_UUID_16_LEN);
    }
    break;
    case ATT_UUID_32_LEN:
    {
        // put UUID at end of memory block
        p_cursor -= ATT_UUID_32_LEN;
        // copy UUID
        memcpy(p_cursor, &(svc_desc->uuid[0]), ATT_UUID_32_LEN);
        // set uuid offset pointer
        new_svc->svc.uuid = (uint16_t)(((uint32_t) p_cursor) - ((uint32_t) & (new_svc->svc)));
    }
    break;
    case ATT_UUID_128_LEN:
    {
        // put UUID at end of memory block
        p_cursor -= ATT_UUID_128_LEN;
        // copy UUID
        memcpy(p_cursor, &(svc_desc->uuid[0]), ATT_UUID_128_LEN);
        // set uuid offset pointer
        new_svc->svc.uuid = (uint16_t)(((uint32_t) p_cursor) - ((uint32_t) & (new_svc->svc)));
    }
    break;
    default: /* Nothing to do, cannot happen */
        break;
    }

    // 2. Browse all attribute to initialize attribute information in database
    for (i = 0 ; i < svc_desc->nb_att; i++)
    {
        // set UUID value
        uuid_len = ATT_UUID_LEN(PERM_GET(svc_desc->atts[i].ext_perm, UUID_LEN));
        switch (uuid_len)
        {
        case ATT_UUID_16_LEN:
        {
            // Copy 16 bits UUID
            memcpy(&(new_svc->atts[i].uuid), &(svc_desc->atts[i].uuid[0]), ATT_UUID_16_LEN);
        }
        break;
        case ATT_UUID_32_LEN:
        {
            // put UUID at end of memory block
            p_cursor -= ATT_UUID_32_LEN;
            // copy UUID
            memcpy(p_cursor, &(svc_desc->atts[i].uuid[0]), ATT_UUID_32_LEN);
            // set uuid offset pointer
            new_svc->atts[i].uuid = (uint16_t)(((uint32_t) p_cursor) - ((uint32_t) & (new_svc->atts[i])));
        }
        break;
        case ATT_UUID_128_LEN:
        {
            // put UUID at end of memory block
            p_cursor -= ATT_UUID_128_LEN;
            // copy UUID
            memcpy(p_cursor, &(svc_desc->atts[i].uuid[0]), ATT_UUID_128_LEN);
            // set uuid offset pointer
            new_svc->atts[i].uuid = (uint16_t)(((uint32_t) p_cursor) - ((uint32_t) & (new_svc->atts[i])));
        }
        break;
        default: /* Nothing to do, cannot happen */
            break;
        }

        // copy permissions
        new_svc->atts[i].perm = svc_desc->atts[i].perm;

        // for following attributes, max_length field contains a value
        if (ATTR_COMP_UUID(svc_desc->atts[i], uuid_len, ATT_DECL_CHARACTERISTIC))
        {
            // value handle is just after characteristic definition
            new_svc->atts[i].info.max_length = 0;
        }
        else if (ATTR_COMP_UUID(svc_desc->atts[i], uuid_len, ATT_DECL_INCLUDE)
                 || ATTR_COMP_UUID(svc_desc->atts[i], uuid_len, ATT_DESC_CHAR_EXT_PROPERTIES))
        {
            // value present in database, put data at end of memory block
            p_cursor -= CO_ALIGN2_HI(sizeof(struct attm_att_value) + sizeof(uint16_t));
            att_value = (struct attm_att_value *)p_cursor;

            // set value (include service handle or extended properties value)
            att_value->max_length = sizeof(uint16_t);
            att_value->length     = sizeof(uint16_t);
            memcpy(att_value->value, &(svc_desc->atts[i].max_len), sizeof(uint16_t));

            // set uuid offset pointer
            new_svc->atts[i].info.offset = (uint16_t)(((uint32_t) p_cursor) - ((uint32_t) & (new_svc->atts[i])));
        }
        else if (ATTR_COMP_UUID(svc_desc->atts[i], uuid_len, ATT_DESC_CLIENT_CHAR_CFG)
                 || ATTR_COMP_UUID(svc_desc->atts[i], uuid_len, ATT_DESC_SERVER_CHAR_CFG))
        {
            // value not present in database
            new_svc->atts[i].info.max_length = PERM_MASK_RI | sizeof(uint16_t) | PERM_GET(svc_desc->atts[i].ext_perm, EKS);
        }
        else
        {
            // check if value is present in database or a read request indication will be triggered to get value
            if (PERM_GET(svc_desc->atts[i].ext_perm, RI))
            {
                // read indication will be triggered, just copy maximum length
                new_svc->atts[i].info.max_length = PERM_GET(svc_desc->atts[i].max_len, MAX_LEN)
                                                   | (svc_desc->atts[i].ext_perm & ~PERM_MASK_MAX_LEN);
            }
            else
            {
                // value present in database, put data at end of memory block
                p_cursor -= CO_ALIGN2_HI(sizeof(struct attm_att_value) + PERM_GET(svc_desc->atts[i].max_len, MAX_LEN));
                att_value = (struct attm_att_value *)p_cursor;

                // Set attribute value information
                att_value->max_length = PERM_GET(svc_desc->atts[i].max_len, MAX_LEN);
                att_value->length = 0;

                // set uuid offset pointer
                new_svc->atts[i].info.offset  = (uint16_t)(((uint32_t) p_cursor) - ((uint32_t) & (new_svc->atts[i])));
                new_svc->atts[i].info.offset |= (svc_desc->atts[i].ext_perm & ~PERM_MASK_MAX_LEN);
            }
        }
    }
}


/**
 * Insert service into service list.
 *
 * @param[in] svc Service to insert in service list.
 */
static void attmdb_svc_insert(struct attm_svc *svc)
{
    struct attm_svc **current_svc = &(gattm_env.db.svcs);
    bool found = false;

    // browse service list.
    while ((*current_svc != NULL) && (!found))
    {
        // new service could be insert before browsed service
        if (svc->svc.start_hdl < (*current_svc)->svc.start_hdl)
        {
            // insert element in list
            svc->next = *current_svc;
            *current_svc = svc;
            found = true;
        }
        // continue with next service
        else
        {
            current_svc = &((*current_svc)->next);
        }
    }

    // no service found, put service at current pointer (end of service list)
    if (!found)
    {
        *current_svc = svc;
        (*current_svc)->next = NULL;
    }
}


/*
 * EXPORTED FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

uint8_t attmdb_svc_check_hdl(struct gattm_svc_desc *svc_desc)
{
    bool found = false;
    uint8_t status = ATT_ERR_NO_ERROR;
    struct attm_svc **current_svc = &(gattm_env.db.svcs);

    /* Service must start at a specific handle */
    if (svc_desc->start_hdl != ATT_INVALID_HDL)
    {
        /* search where service shall be put in database since database is a linked list
         * of services sorted by handle. */
        while ((!found) && (*current_svc != NULL))
        {
            /* if start handle is less than next services handles, current service
             * is found */
            if (svc_desc->start_hdl <= (*current_svc)->svc.end_hdl)
            {
                found = true;
            }
            /* else continue to search in database */
            else
            {
                current_svc = &((*current_svc)->next);
            }
        }

        if (found)
        {
            /* if new database override existing elements, trigger an error. */
            if ((svc_desc->start_hdl + (svc_desc->nb_att)) >= (*current_svc)->svc.start_hdl)
            {
                status = ATT_ERR_INVALID_HANDLE;
            }
        }
    }
    else
    {
        /* increment start handle to be a valid handle. */
        svc_desc->start_hdl += 1;

        /* search first available block of handles in database. */
        while ((!found) && (*current_svc != NULL))
        {
            /* block of free attribute handle is found */
            if ((svc_desc->start_hdl + (svc_desc->nb_att)) < (*current_svc)->svc.start_hdl)
            {
                found = true;
            }
            else
            {
                /* set new start handle to be after current service */
                svc_desc->start_hdl = (*current_svc)->svc.end_hdl + 1;
                /* update database cursor pointer to continue database search */
                current_svc = &((*current_svc)->next);
            }
        }
    }

    return (status);
}


uint8_t attmdb_add_service(struct gattm_svc_desc *svc_desc)
{
    /* Calculate total number of attributes. */
    uint8_t status = ATT_ERR_NO_ERROR;
    uint16_t svc_mem_size;
    struct attm_svc *new_svc;
    bool calc_hdl = (svc_desc->start_hdl == 0);

    do
    {
        // -----------------------------------------------------------------------------------
        // First Step, check validity of service to create and calculate the memory block to
        // allocate.
        // -----------------------------------------------------------------------------------

        // Check that service handle not already used by another service and calculate start handle.
        status = attmdb_svc_check_hdl(svc_desc);

        // exit if an error occurs
        if (status != ATT_ERR_NO_ERROR) break;

        // calculate memory block that shall be used to initialize service.
        status = attmdb_svc_calc_len(svc_desc, &svc_mem_size);

        // exit if an error occurs
        if (status != ATT_ERR_NO_ERROR) break;

        // -----------------------------------------------------------------------------------
        // Allocate Memory Block
        // -----------------------------------------------------------------------------------
        // allocate buffer structure
        new_svc = (struct attm_svc *) ke_malloc(svc_mem_size, KE_MEM_ATT_DB);


        // -----------------------------------------------------------------------------------
        // Initialize service memory block with attribute informations.
        // -----------------------------------------------------------------------------------
        attmdb_svc_init(svc_desc, new_svc, svc_mem_size, calc_hdl);

        // -----------------------------------------------------------------------------------
        // Insert service into service list
        // -----------------------------------------------------------------------------------
        attmdb_svc_insert(new_svc);
    }
    while (0);

    return (status);
}



void attmdb_destroy(void)
{
    struct attm_svc *current_svc = gattm_env.db.svcs;
    struct attm_svc *next_svc;

    /* browse all database and free all services. */
    while (current_svc != NULL)
    {
        next_svc = current_svc->next;
        ke_free(current_svc);
        current_svc = next_svc;
    }

    /* set database empty */
    gattm_env.db.svcs = NULL;
    gattm_env.db.cache = NULL;
}


struct attm_svc *attmdb_get_service(uint16_t handle)
{
    bool found = false;
    struct attm_svc *current_svc = gattm_env.db.svcs;

    // verify if searched can be speed up using cached service variable.
    if (gattm_env.db.cache != NULL)
    {
        /* check if it's in cached service or after */
        if (handle >= gattm_env.db.cache->svc.start_hdl)
        {
            /* handle is in current service */
            if (handle <= gattm_env.db.cache->svc.end_hdl)
            {
                found = true;
                current_svc = gattm_env.db.cache;
            }
            /* if handle is greater than last handle in cached service,
             * search in rest of database */
            else
            {
                current_svc = gattm_env.db.cache->next;
            }
        }
    }

    /* Browse rest of service list */
    while ((!found) && (current_svc != NULL))
    {
        /* check in following services */
        if (handle >= current_svc->svc.start_hdl)
        {
            /* handle is in current service - and service visible */
            if ((handle <= current_svc->svc.end_hdl) && (!PERM_GET(current_svc->svc.ext_perm, EXT_SVC_HIDE)))
            {
                found = true;
                /* save found service in cache variable to speed-up next search */
                gattm_env.db.cache = current_svc;
            }
            /* else try in next services*/
            else
            {
                current_svc = current_svc->next;
            }
        }
        /* handle not present in database */
        else
        {
            current_svc = NULL;
        }
    }

    /* return found service. */
    return current_svc;
}

uint8_t attmdb_svc_visibility_set(uint16_t handle, bool hide)
{
    uint8_t status = ATT_ERR_INVALID_HANDLE;
    bool found = false;
    struct attm_svc *current_svc = gattm_env.db.svcs;

    /* Browse rest of service list */
    while ((!found) && (current_svc != NULL))
    {
        /* check in following services */
        if (handle >= current_svc->svc.start_hdl)
        {
            /* handle is in current service */
            if (handle <= current_svc->svc.end_hdl)
            {
                found = true;
            }
            /* else try in next services*/
            else
            {
                current_svc = current_svc->next;
            }
        }
        /* handle not present in database */
    }

    if (found)
    {
        // Set service visibility
        PERM_SET(current_svc->svc.ext_perm, EXT_SVC_HIDE, hide);

        // clean-up cache to ensure that it does not point to current service.
        gattm_env.db.cache = NULL;

        status = GAP_ERR_NO_ERROR;
    }

    return status;
}

uint8_t attmdb_get_attribute(uint16_t handle, struct attm_elmt *elmt)
{
    uint8_t status = ATT_ERR_INVALID_HANDLE;
    struct attm_svc *current_svc = attmdb_get_service(handle);

    if (current_svc != NULL)
    {
        status = ATT_ERR_NO_ERROR;

        // service description
        if (current_svc->svc.start_hdl == handle)
        {
            /* retrieve service */
            elmt->info.svc = &(current_svc->svc);
            elmt->service = true;
        }
        else
        {
            /* retrieve attribute */
            elmt->info.att = ATT_GET_ELMT(current_svc, handle);
            elmt->service = false;
        }
    }

    return (status);
}

uint8_t attmdb_get_next_att(uint16_t *handle, struct attm_elmt *elmt)
{
    uint8_t status = ATT_ERR_INVALID_HANDLE;
    bool found = false;

    struct attm_svc *current_svc = gattm_env.db.svcs;

    // verify if searched can be speed up using cached service variable.
    if (gattm_env.db.cache != NULL)
    {
        /* check if it's in cached service or after */
        if (*handle >= gattm_env.db.cache->svc.start_hdl)
        {
            /* handle is available in current service */
            if ((*handle <= gattm_env.db.cache->svc.end_hdl))
            {
                found = true;
                current_svc = gattm_env.db.cache;
            }
            /* if handle is greater than last handle in cached service,
             * search in rest of database */
            else
            {
                current_svc = gattm_env.db.cache->next;
            }
        }
    }

    while ((!found) && (current_svc != NULL))
    {
        /* check in following services */
        if (*handle >= current_svc->svc.start_hdl)
        {
            /* handle is in current service and service not hidden. */
            if ((*handle <= current_svc->svc.end_hdl) && (!PERM_GET(current_svc->svc.ext_perm, EXT_SVC_HIDE)))
            {
                found = true;
            }
            /* else try in next services*/
            else
            {
                current_svc = current_svc->next;
            }
        }
        /* current service contains attribute with handle greater than handle given
         * in parameter */
        else
        {
            // current service is visible
            if (!PERM_GET(current_svc->svc.ext_perm, EXT_SVC_HIDE))
            {
                found = true;
                /* set new handle as first service handle. */
                *handle = current_svc->svc.start_hdl;
            }
            // current service is hidden, ignore it
            else
            {
                current_svc = current_svc->next;
            }
        }
    }

    if (found)
    {
        status = ATT_ERR_NO_ERROR;
        // service description
        if (current_svc->svc.start_hdl == *handle)
        {
            /* retrieve service */
            elmt->info.svc = &(current_svc->svc);
            elmt->service = true;
        }
        else
        {
            /* retrieve attribute */
            elmt->info.att = ATT_GET_ELMT(current_svc, *handle);
            elmt->service = false;
        }

        /* save found service in cache variable to speed-up next search */
        gattm_env.db.cache = current_svc;
    }

    /* return found attribute */
    return (status);
}


bool attmdb_uuid16_comp(struct attm_elmt *elmt, uint16_t uuid16)
{
    bool match = false;

    // check for an attribute which is not a service contains 16 bits UUOD
    if (!elmt->service && (elmt->info.att != NULL)
            && (PERM_GET(elmt->info.att->perm, UUID_LEN) == PERM_UUID_16))
    {
        // compare UUIDs
        if (elmt->info.att->uuid == uuid16)
        {
            match = true;
        }
    }
    // service attribute
    else if (elmt->service && (elmt->info.svc != NULL))
    {
        // check if primary or secondary service match
        if ((PERM_IS_SET(elmt->info.svc->perm, SVC_SECONDARY, DISABLE) && (uuid16 == ATT_DECL_PRIMARY_SERVICE))
                || (PERM_IS_SET(elmt->info.svc->perm, SVC_SECONDARY, ENABLE) && (uuid16 == ATT_DECL_SECONDARY_SERVICE)))
        {
            match = true;
        }
    }
    return match;
}



uint8_t attmdb_get_max_len(struct attm_elmt *elmt, att_size_t *length)
{
    uint8_t status = ATT_ERR_NO_ERROR;

    // not valid for Write only properties
    if (attmdb_uuid16_comp(elmt, ATT_DECL_PRIMARY_SERVICE)
            || attmdb_uuid16_comp(elmt, ATT_DECL_SECONDARY_SERVICE)
            || attmdb_uuid16_comp(elmt, ATT_DECL_CHARACTERISTIC)
            || attmdb_uuid16_comp(elmt, ATT_DECL_INCLUDE)
            || attmdb_uuid16_comp(elmt, ATT_DESC_CHAR_EXT_PROPERTIES))
    {
        status = ATT_ERR_REQUEST_NOT_SUPPORTED;
    }
    else
    {
        // check if data present in attribute
        if (PERM_GET(elmt->info.att->info.max_length, RI) == 0)
        {
            // retrieve value pointer
            struct attm_att_value *val =
                (struct attm_att_value *)(((uint8_t *)elmt->info.att) +  PERM_GET(elmt->info.att->info.offset, VAL_OFFSET));

            *length = val->max_length;
        }
        // attribute value cannot be read in database
        else
        {
            *length = PERM_GET(elmt->info.att->info.max_length, MAX_LEN);
        }
    }

    return (status);
}


uint8_t attmdb_get_uuid(struct attm_elmt *elmt, uint8_t *uuid_len, uint8_t *uuid, bool srv_uuid, bool air)
{
    uint8_t status = ATT_ERR_NO_ERROR;
    uint8_t *uuid_ptr = NULL;
    *uuid_len = 0;

    /* if handle doesn't exists, trigger an error. */
    if (elmt->info.att == NULL)
    {
        status = ATT_ERR_INVALID_HANDLE;
    }
    else
    {
        // specific service handling
        if (elmt->service)
        {
            if (srv_uuid)
            {
                *uuid_len = ATT_UUID_LEN(PERM_GET(elmt->info.svc->perm, SVC_UUID_LEN));

                switch (*uuid_len)
                {
                case ATT_UUID_16_LEN:
                {
                    uuid_ptr = (uint8_t *)(&elmt->info.svc->uuid);
                }
                break;
                case ATT_UUID_32_LEN:
                {
                    uuid_ptr = ((uint8_t *)elmt->info.svc) + elmt->info.svc->uuid;
                }
                break;
                case ATT_UUID_128_LEN:
                {
                    uuid_ptr = ((uint8_t *)elmt->info.svc) + elmt->info.svc->uuid;
                }
                break;
                default: /* Can not happen */
                    break;
                }
            }
            else
            {
                uint16_t uuid_val = ATT_DECL_PRIMARY_SERVICE;
                // retrieve if service is primary or secondary
                if (PERM_GET(elmt->info.svc->perm, SVC_SECONDARY))
                {
                    uuid_val = ATT_DECL_SECONDARY_SERVICE;
                }

                *uuid_len = ATT_UUID_16_LEN;
                // Copy UUID
                memcpy(uuid, &uuid_val, *uuid_len);
            }
        }
        // attribute handling
        else
        {
            *uuid_len = ATT_UUID_LEN(PERM_GET(elmt->info.att->info.max_length, UUID_LEN));

            switch (*uuid_len)
            {
            case ATT_UUID_16_LEN:
            {
                uuid_ptr = (uint8_t *)(&elmt->info.att->uuid);
            }
            break;
            case ATT_UUID_32_LEN:
            {
                uuid_ptr = ((uint8_t *)elmt->info.att) + elmt->info.att->uuid;
            }
            break;
            case ATT_UUID_128_LEN:
            {
                uuid_ptr = ((uint8_t *)elmt->info.att) + elmt->info.att->uuid;
            }
            break;
            default: /* Can not happen */
                break;
            }
        }
    }

    if (uuid_ptr != NULL)
    {
        if (air && (*uuid_len == ATT_UUID_32_LEN))
        {
            // convert to 128 bits UUID
            attm_convert_to128(uuid, uuid_ptr, *uuid_len);
            *uuid_len = ATT_UUID_128_LEN;
        }
        else
        {
            // Copy UUID
            memcpy(uuid, uuid_ptr, *uuid_len);
        }
    }

    return (status);
}


uint8_t attmdb_att_get_permission(uint16_t handle, uint16_t *perm, uint16_t mode_mask,
                                  uint16_t perm_mask, struct attm_elmt *elmt)
{
    uint8_t status = ATT_ERR_NO_ERROR;

    do
    {
        // load attribute value only if attribute not loaded
        if (elmt->info.att == NULL)
        {
            // load attribute info
            status = attmdb_get_attribute(handle, elmt);
        }

        if (status != ATT_ERR_NO_ERROR)
        {
            break;
        }

        if (attmdb_uuid16_comp(elmt, ATT_DECL_PRIMARY_SERVICE)
                || attmdb_uuid16_comp(elmt, ATT_DECL_SECONDARY_SERVICE)
                || attmdb_uuid16_comp(elmt, ATT_DECL_CHARACTERISTIC)
                || attmdb_uuid16_comp(elmt, ATT_DECL_INCLUDE)
                || attmdb_uuid16_comp(elmt, ATT_DESC_CHAR_EXT_PROPERTIES))

        {
            // Those attributes are read only
            if (mode_mask == 0)
            {
                *perm =  PERM(RD, ENABLE) | PERM(RP, NO_AUTH);
            }
            else if (mode_mask == PERM_MASK_RD)
            {
                *perm = PERM_RIGHT_NO_AUTH;
            }
            else
            {
                status = ATT_ERR_REQUEST_NOT_SUPPORTED;
            }
        }
        else
        {
            /* set parameters to return (without UUID length info) */
            *perm = elmt->info.att->perm;

            // check if mode is supported
            if ((mode_mask != PERM_MASK_ALL) && ((*perm & mode_mask) == 0))
            {
                status = ATT_ERR_REQUEST_NOT_SUPPORTED;
                break;
            }
            // if permission mask is set
            else if (mode_mask != PERM_MASK_ALL)
            {
                uint8_t svc_perm = 0;

                // retrieve service permissions
                status = attm_svc_get_permission(handle, &svc_perm);

                if (status != ATT_ERR_NO_ERROR)
                {
                    break;
                }

                // check if encryption key size is required
                if (PERM_GET(elmt->info.att->info.max_length, EKS) || PERM_GET(svc_perm, SVC_EKS))
                {
                    status = ATT_ERR_INSUFF_ENC_KEY_SIZE;
                }

                switch (mode_mask)
                {
                case PERM_MASK_RD:
                {
                    *perm = PERM_GET(*perm, RP);
                }
                break;
                case PERM_MASK_WRITE_COMMAND:
                case PERM_MASK_WRITE_REQ:
                case PERM_MASK_WRITE_SIGNED:
                {
                    *perm = PERM_GET(*perm, WP);
                }
                break;
                case PERM_MASK_IND:
                {
                    *perm = PERM_GET(*perm, IP);
                }
                break;
                case PERM_MASK_NTF:
                {
                    *perm = PERM_GET(*perm, NP);
                }
                break;

                default:
                {
                    ASSERT_INFO(0, handle, mode_mask);
                }
                break;
                }

                // service is disable, not authorized to do anything on attribute
                if (PERM_GET(svc_perm, SVC_DIS))
                {
                    status = ATT_ERR_INSUFF_AUTHOR;
                }
                else
                {
                    // retrieve service authentication level
                    svc_perm = PERM_GET(svc_perm, SVC_AUTH);
                    // if service permission requires more access, use service
                    // authentication permission requirements
                    if (svc_perm > *perm)
                    {
                        *perm = svc_perm;
                    }
                }
            }
            // else just return permission
        }
    }
    while (0);

    return (status);
}


#if (RW_DEBUG)

uint8_t attmdb_get_nb_svc(void)
{
    uint8_t nb_svc = 0;
    struct attm_svc *current_svc = gattm_env.db.svcs;

    /* count number of services. */
    while (current_svc != NULL)
    {
        /* go to next service */
        current_svc = current_svc->next;
        nb_svc++;
    }

    return nb_svc;
}


void attmdb_get_svc_info(struct gattm_svc_info *svc_info)
{
    uint8_t nb_svc = 0;
    struct attm_svc *current_svc = gattm_env.db.svcs;

    /* count number of services. */
    while (current_svc != NULL)
    {
        /* retrieve service information */
        svc_info[nb_svc].start_hdl = current_svc->svc.start_hdl;
        svc_info[nb_svc].end_hdl   = current_svc->svc.end_hdl;

        svc_info[nb_svc].task_id   = gapm_get_id_from_task(current_svc->svc.task_id);
        svc_info[nb_svc].perm      = current_svc->svc.perm & ~(PERM_MASK_SVC_UUID_LEN | PERM_MASK_SVC_MI);

        /* go to next service */
        current_svc = current_svc->next;
        nb_svc++;
    }
}
#endif /* (RW_DEBUG) */


#endif // (BLE_ATTS)

/// @} ATTDB
