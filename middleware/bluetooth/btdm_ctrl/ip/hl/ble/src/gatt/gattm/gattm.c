/**
 ****************************************************************************************
 *
 * @file gattm.c
 *
 * @brief Generic Attribute Profile Manager implementation.
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup GATTM
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"

#if (BLE_CENTRAL || BLE_PERIPHERAL)
#include "gattm.h"
#include "gattm_int.h"

#include "../gatt.h"
#include "../gattc/gattc_int.h" // Access to Internal API required

#include "co_math.h"

#include "attm.h"
#include "gapm.h"
#include "l2cc_pdu.h"

/*
 * DEFINES
 ****************************************************************************************
 */

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
struct gattm_env_tag gattm_env;

/// GATT Manager task descriptor
extern const struct ke_task_desc TASK_DESC_GATTM;

#if (BLE_ATTS)

/// GATT Attribute database description
static const struct attm_desc gattm_att_db[GATT_IDX_NUMBER] =
{
    // GATT_IDX_PRIM_SVC - GATT service
    [GATT_IDX_PRIM_SVC]         = {ATT_DECL_PRIMARY_SERVICE, PERM(RD, ENABLE), 0, 0},
    // GATT_IDX_CHAR_SVC_CHANGED - Service Changed declaration
    [GATT_IDX_CHAR_SVC_CHANGED] = {ATT_DECL_CHARACTERISTIC,  PERM(RD, ENABLE), 0, 0},
    // GATT_IDX_SVC_CHANGED - Service Changed definition
    [GATT_IDX_SVC_CHANGED]      = {ATT_CHAR_SERVICE_CHANGED, (PERM(RD, ENABLE) | PERM(IND, ENABLE)), 0, sizeof(struct gatt_svc_changed)},
    // GATT_IDX_SVC_CHANGED_CFG - Service Changed Client Characteristic Configuration Descriptor
    [GATT_IDX_SVC_CHANGED_CFG]  = {ATT_DESC_CLIENT_CHAR_CFG, PERM(RD, ENABLE) | PERM(WRITE_REQ, ENABLE), 0, sizeof(uint16_t)},
};

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */


uint16_t gattm_svc_get_start_hdl(void)
{
    return gattm_env.svc_start_hdl;
}
#endif /* BLE_ATTS */

void gattm_init(bool reset)
{
    if (!reset)
    {
        // Create GATT task
        ke_task_create(TASK_GATTM, &TASK_DESC_GATTM);
    }

    // Initialize GATT controllers
    gattc_init(reset);

    // set state to IDLE
    ke_state_set(TASK_GATTM, GATTM_IDLE);
}

uint8_t gattm_init_attr(uint16_t start_hdl, bool svc_chg_en)
{
    // Initialization successful ?
    uint8_t status;
    uint32_t feat = GATT_DB_DEFAULT_FEAT | (svc_chg_en ? GATT_DB_SVC_CHG_FEAT : 0);

    // Initialize Start Handle
    gattm_env.svc_start_hdl = start_hdl;

    // Add the GATT service in the DB
    status = attm_svc_create_db(&(gattm_env.svc_start_hdl), ATT_SVC_GENERIC_ATTRIBUTE, (uint8_t *) &feat, GATT_IDX_NUMBER,
                                NULL, TASK_GATTC, &gattm_att_db[0], PERM(SVC_MI, ENABLE));

    // initialize service changed value
    if ((status == ATT_ERR_NO_ERROR) && (gapm_svc_chg_en()))
    {
        // Initial Service Changed Characteristic value
        struct gatt_svc_changed svc_chg = {0x0001, 0xFFFF};

        // Initialize the Service Changed Client Characteristic Cfg. value
        status = attm_att_set_value(GATT_GET_ATT_HANDLE(GATT_IDX_SVC_CHANGED),
                                    sizeof(struct gatt_svc_changed), 0, (uint8_t *)&svc_chg);
    }

    return (status);
}

void gattm_create(uint8_t conidx)
{
    /* prepare resources for connection */
    gattc_create(conidx);
}

void gattm_cleanup(uint8_t conidx)
{
    /* clean environment variables of connection */
    gattc_cleanup(conidx, true);
}

#endif /* #if (BLE_CENTRAL || BLE_PERIPHERAL) */
/// @} GATTM
