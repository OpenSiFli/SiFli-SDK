/**
 ****************************************************************************************
 *
 * @file wscs.c
 *
 * @brief Weight SCale Profile Sensor implementation.
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 * $ Rev $
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup WSCS
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"

#if (BLE_WSC_SERVER)
#include "wsc_common.h"

#include "gap.h"
#include "gattc_task.h"
#include "attm.h"
#include "wscs.h"
#include "wscs_task.h"
#include "prf_utils.h"
#include "ke_mem.h"
#include "co_utils.h"

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
// default read perm
#define RD_P   (PERM(RD, ENABLE)        | PERM(RP, NO_AUTH))
// default write perm
#define WR_P   (PERM(WRITE_REQ, ENABLE) | PERM(WP, NO_AUTH))
// ind perm
#define IND_P  (PERM(IND, ENABLE)       | PERM(IP, NO_AUTH))
// enable Read Indication (1 = Value not present in Database)
#define EXT_P  (PERM(RI, ENABLE)        )


/// Full WSCS Database Description - Used to add attributes into the database
static const struct attm_desc wscs_att_db[WSCS_IDX_NB] =
{
    /// ATT Index                 | ATT UUID                         | Permission        | EXT PERM | MAX ATT SIZE
    // Weight SCale Service Declaration
    [WSCS_IDX_SVC]             =   {ATT_DECL_PRIMARY_SERVICE, RD_P, 0, 0},

    [WSCS_IDX_INC_SVC]         =   {ATT_DECL_INCLUDE, RD_P, 0, 0},
    // Weight SCale Feature
    [WSCS_IDX_FEAT_CHAR]       =   {ATT_DECL_CHARACTERISTIC, RD_P, 0, 0},
    // Descriptor Value Changed Characteristic Value
    [WSCS_IDX_FEAT_VAL]        =   {ATT_CHAR_WEIGHT_SCALE_FEATURE, RD_P, EXT_P, WSCS_FEAT_VAL_SIZE},

    // Weight SCale Measurement
    [WSCS_IDX_MEAS_CHAR]       =   {ATT_DECL_CHARACTERISTIC, RD_P, 0, 0},
    // Weight SCale Measurement Characteristic Value
    [WSCS_IDX_MEAS_IND]        =   {ATT_CHAR_WEIGHT_MEASUREMENT, IND_P, 0, WSCS_MEAS_IND_SIZE},
    // Weight SCale Measurement Characteristic - Client Characteristic Configuration Descriptor
    [WSCS_IDX_MEAS_CCC]        =   {ATT_DESC_CLIENT_CHAR_CFG, RD_P | WR_P, EXT_P, sizeof(uint16_t)},
};

/*
 * EXPORTED FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Initialization of the WSCS module.
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
static uint8_t wscs_init(struct prf_task_env *env, uint16_t *start_hdl, uint16_t app_task, uint8_t sec_lvl, struct wscs_db_cfg *param)
{
    //------------------ create the attribute database for the profile -------------------
    // Service content flag
    uint8_t cfg_flag = (1 << WSCS_IDX_NB) - 1;
    // DB Creation Status
    uint8_t status = ATT_ERR_NO_ERROR;



    // Add service in the database
    struct attm_desc wscs_db[WSCS_IDX_NB];
    uint8_t i;

    for (i = 0; i < WSCS_IDX_NB; i++)
        memcpy(&wscs_db[i], &wscs_att_db[i], sizeof(struct attm_desc));

    if (param->bcs_start_hdl)
        wscs_db[WSCS_IDX_INC_SVC].max_size = param->bcs_start_hdl;


    status = attm_svc_create_db(start_hdl, ATT_SVC_WEIGHT_SCALE, (uint8_t *)&cfg_flag,
                                WSCS_IDX_NB, NULL, env->task, &wscs_db[0],
                                (sec_lvl & (PERM_MASK_SVC_DIS | PERM_MASK_SVC_AUTH | PERM_MASK_SVC_EKS)) | PERM(SVC_MI, ENABLE));


    // Check if an error has occured
    if (status == ATT_ERR_NO_ERROR)
    {
        // Allocate WSCS required environment variable
        struct wscs_env_tag *wscs_env =
            (struct wscs_env_tag *) ke_malloc(sizeof(struct wscs_env_tag), KE_MEM_ATT_DB);

        // Initialize WSCS environment

        env->env           = (prf_env_t *) wscs_env;
        wscs_env->shdl     = *start_hdl;
        wscs_env->feature  = param->feature;

        if (param->bcs_start_hdl)
            wscs_env->bcs_included = 0x01;
        else
            wscs_env->bcs_included = 0x00;

        // Mono Instantiated APP task
        wscs_env->prf_env.app_task    = app_task
                                        | (PERM_GET(sec_lvl, SVC_MI) ? PERM(PRF_MI, ENABLE) : PERM(PRF_MI, DISABLE));
        // Multi Instantiated task ???depend on Multiple user support Feature
        wscs_env->prf_env.prf_task    = env->task
                                        |  PERM(PRF_MI, ENABLE);
///             | ( param->feature & WSC_MULTIPLE_USERS_SUPPORTED) ? PERM(PRF_MI, ENABLE) : PERM(PRF_MI, DISABLE);

        // initialize environment variable
        env->id                     = TASK_ID_WSCS;
        wscs_task_init(&(env->desc));

        // Initialize CCC per connection
        memset(wscs_env->prfl_ind_cfg, 0, BLE_CONNECTION_MAX);
        // If we are here, database has been fulfilled with success, go to idle state
        for (uint8_t idx = 0; idx < BLE_CONNECTION_MAX ; idx++)
        {
            /* Put WSCS in disabled state */
            ke_state_set(KE_BUILD_ID(env->task, idx), WSCS_FREE);
        }

    }

    return (status);
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
static void wscs_cleanup(struct prf_task_env *env, uint8_t conidx, uint8_t reason)
{
    struct wscs_env_tag *wscs_env = (struct wscs_env_tag *) env->env;

    // clean-up environment variable allocated for task instance
    wscs_env->prfl_ind_cfg[conidx] = 0;

    /* Put WSCS in disabled state */
    ke_state_set(KE_BUILD_ID(env->task, conidx), WSCS_FREE);
}

/**
 ****************************************************************************************
 * @brief Destruction of the WSCS module - due to a reset for instance.
 * This function clean-up allocated memory (attribute database is destroyed by another
 * procedure)
 *
 * @param[in|out]    env        Collector or Service allocated environment data.
 ****************************************************************************************
 */
static void wscs_destroy(struct prf_task_env *env)
{
    struct wscs_env_tag *wscs_env = (struct wscs_env_tag *) env->env;

    // cleanup environment variable for each task instances

    // free profile environment variables
    env->env = NULL;
    ke_free(wscs_env);
}

/**
 ****************************************************************************************
 * @brief Handles Connection creation
 *
 * @param[in|out]    env        Collector or Service allocated environment data.
 * @param[in]        conidx     Connection index
 ****************************************************************************************
 */
static void wscs_create(struct prf_task_env *env, uint8_t conidx)
{
    struct wscs_env_tag *wscs_env = (struct wscs_env_tag *) env->env;
    wscs_env->prfl_ind_cfg[conidx] = 0;
    // Change state to IDLE
    ke_state_set(KE_BUILD_ID(env->task, conidx), WSCS_IDLE);
}


/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// WSCS Task interface required by profile manager
const struct prf_task_cbs wscs_itf =
{
    (prf_init_fnct) wscs_init,
    wscs_destroy,
    wscs_create,
    wscs_cleanup,
};

/*
 * EXPORTED FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

const struct prf_task_cbs *wscs_prf_itf_get(void)
{
    return &wscs_itf;
}


void wscs_send_cmp_evt(struct wscs_env_tag *wscs_env, uint8_t conidx, uint8_t operation, uint8_t status)
{

    struct wscs_cmp_evt *evt = KE_MSG_ALLOC(WSCS_CMP_EVT,
                                            prf_dst_task_get(&(wscs_env->prf_env), conidx),
                                            prf_src_task_get(&(wscs_env->prf_env), conidx),
                                            wscs_cmp_evt);
    evt->status     = status;
    evt->operation = operation;
    ke_msg_send(evt);
}

#endif //(BLE_WSC_SERVER)

/// @} WSCS
