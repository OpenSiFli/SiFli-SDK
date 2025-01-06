/**
 ****************************************************************************************
 *
 * @file scpps.c
 *
 * @brief Scan Parameters Profile Server implementation.
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup SCPPS
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
#include "attm.h"
#include "scpps.h"
#include "scpps_task.h"
#include "prf_utils.h"

#include "ke_mem.h"

/*
 * SCPP ATTRIBUTES DEFINITION
 ****************************************************************************************
 */

/// Full SCPS Database Description - Used to add attributes into the database
const struct attm_desc scpps_att_db[SCPS_IDX_NB] =
{
    // Scan Parameters Service Declaration
    [SCPS_IDX_SVC]                      =   {ATT_DECL_PRIMARY_SERVICE, PERM(RD, ENABLE), 0, 0},

    // Scan Interval Window Characteristic Declaration
    [SCPS_IDX_SCAN_INTV_WD_CHAR]        =   {ATT_DECL_CHARACTERISTIC, PERM(RD, ENABLE), 0, 0},
    // Scan Interval Window Characteristic Value
    [SCPS_IDX_SCAN_INTV_WD_VAL]         =   {ATT_CHAR_SCAN_INTV_WD, PERM(WRITE_COMMAND, ENABLE), PERM(RI, ENABLE), sizeof(struct scpp_scan_intv_wd)},

    // Scan Refresh Characteristic Declaration
    [SCPS_IDX_SCAN_REFRESH_CHAR]        =   {ATT_DECL_CHARACTERISTIC, PERM(RD, ENABLE), 0, 0},
    // Scan Refresh Characteristic Value
    [SCPS_IDX_SCAN_REFRESH_VAL]         =   {ATT_CHAR_SCAN_REFRESH, PERM(NTF, ENABLE),  PERM(RI, ENABLE), 0},
    // Scan Refresh Characteristic - Client Characteristic Configuration Descriptor
    [SCPS_IDX_SCAN_REFRESH_NTF_CFG]     =   {ATT_DESC_CLIENT_CHAR_CFG, PERM(RD, ENABLE) | PERM(WRITE_REQ, ENABLE), 0, 0},
};

/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Initialization of the SCPPS module.
 * This function performs all the initializations of the Profile module.
 *  - Creation of database (if it's a service)
 *  - Allocation of profile required memory
 *  - Initialization of task descriptor to register application
 *      - Task State array
 *      - Number of tasks
 *      - Default task handler
 *
 * @param[out]    env        Collector or Service allocated environment data.
 * @param[in|out] start_hdl  Service start handle (0 - dynamically allocated), only applies for services.
 * @param[in]     app_task   Application task number.
 * @param[in]     sec_lvl    Security level (AUTH, EKS and MI field of @see enum attm_value_perm_mask)
 * @param[in]     param      Configuration parameters of profile collector or service (32 bits aligned)
 *
 * @return status code to know if profile initialization succeed or not.
 ****************************************************************************************
 */
static uint8_t scpps_init(struct prf_task_env *env, uint16_t *start_hdl, uint16_t app_task, uint8_t sec_lvl,  struct scpps_db_cfg *params)
{
    struct scpps_env_tag *scpps_env = NULL;
    // Status
    uint8_t status = GAP_ERR_NO_ERROR;
    // Service content flag
    uint8_t cfg_flag = SCPPS_CFG_FLAG_MANDATORY_MASK;

    // Check if multiple instances
    if ((params->features & SCPPS_SCAN_REFRESH_SUP) != 0)
    {
        cfg_flag |= SCPPS_CFG_FLAG_SCAN_REFRESH_MASK;
    }

    //Create SCPS in the DB
    //------------------ create the attribute database for the profile -------------------
    status = attm_svc_create_db(start_hdl, ATT_SVC_SCAN_PARAMETERS, (uint8_t *)&cfg_flag,
                                SCPS_IDX_NB, NULL, env->task, scpps_att_db,
                                (sec_lvl & (PERM_MASK_SVC_DIS | PERM_MASK_SVC_AUTH | PERM_MASK_SVC_EKS)));

    //-------------------- Update profile task information  ---------------------
    if (status == ATT_ERR_NO_ERROR)
    {
        //-------------------- allocate memory required for the profile  ---------------------

        // allocate SCPPS required environment variable
        scpps_env = (struct scpps_env_tag *) ke_malloc(sizeof(struct scpps_env_tag), KE_MEM_ATT_DB);
        memset(scpps_env, 0, sizeof(struct scpps_env_tag));

        env->env = (prf_env_t *) scpps_env;
        scpps_env->start_hdl = *start_hdl;
        scpps_env->features  = params->features;
        scpps_env->prf_env.app_task = app_task
                                      | (PERM_GET(sec_lvl, SVC_MI) ? PERM(PRF_MI, ENABLE) : PERM(PRF_MI, DISABLE));
        scpps_env->prf_env.prf_task = env->task | PERM(PRF_MI, DISABLE);

        // initialize environment variable
        env->id                     = TASK_ID_SCPPS;
        scpps_task_init(&(env->desc));

        // service is ready, go into an Idle state
        ke_state_set(env->task, SCPPS_IDLE);
    }
    else if (scpps_env != NULL)
    {
        ke_free(scpps_env);
    }

    return (status);
}
/**
 ****************************************************************************************
 * @brief Destruction of the SCPPS module - due to a reset for instance.
 * This function clean-up allocated memory (attribute database is destroyed by another
 * procedure)
 *
 * @param[in|out]    env        Collector or Service allocated environment data.
 ****************************************************************************************
 */
static void scpps_destroy(struct prf_task_env *env)
{
    struct scpps_env_tag *scpps_env = (struct scpps_env_tag *) env->env;

    // free profile environment variables
    env->env = NULL;
    ke_free(scpps_env);
}

/**
 ****************************************************************************************
 * @brief Handles Connection creation
 *
 * @param[in|out]    env        Collector or Service allocated environment data.
 * @param[in]        conidx     Connection index
 ****************************************************************************************
 */
static void scpps_create(struct prf_task_env *env, uint8_t conidx)
{
    /* Nothing to do */
}

/**
 ****************************************************************************************
 * @brief Handles Disconnection
 *
 * @param[in|out]    env        Collector or Service allocated environment data.
 * @param[in]        conidx     Connection index
 * @param[in]        reason     Detach reason
 ****************************************************************************************
 */
static void scpps_cleanup(struct prf_task_env *env, uint8_t conidx, uint8_t reason)
{
    struct scpps_env_tag *scpps_env = (struct scpps_env_tag *) env->env;

    ASSERT_ERR(conidx < BLE_CONNECTION_MAX);
    // force notification config to zero when peer device is disconnected
    scpps_env->ntf_cfg[conidx] = 0;
}

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// SCPPS Task interface required by profile manager
const struct prf_task_cbs scpps_itf =
{
    (prf_init_fnct) scpps_init,
    scpps_destroy,
    scpps_create,
    scpps_cleanup,
};


/*
 * GLOBAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

const struct prf_task_cbs *scpps_prf_itf_get(void)
{
    return &scpps_itf;
}



void scpps_scan_refresh_rsp_send(struct scpps_env_tag *scpps_env, uint8_t conidx, uint8_t status)
{
    struct scpps_scan_refresh_send_rsp *rsp = KE_MSG_ALLOC(SCPPS_SCAN_REFRESH_SEND_RSP,
            prf_dst_task_get(&(scpps_env->prf_env), conidx),
            prf_src_task_get(&(scpps_env->prf_env), conidx),
            scpps_scan_refresh_send_rsp);

    rsp->conidx = conidx;
    rsp->status = status;

    ke_msg_send(rsp);
}

#endif // (BLE_SP_SERVER)

/// @} SCPPS
