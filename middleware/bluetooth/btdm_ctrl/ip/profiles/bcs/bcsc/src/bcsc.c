/**
 ****************************************************************************************
 *
 * @file bcsc.c
 *
 * @brief Body Composition Collector implementation.
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 * $ Rev $
 *
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @addtogroup BCSC
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"

#if BLE_BCS_CLIENT
#include "prf.h"
#include "bcs_common.h"

#include "bcsc_task.h"
#include "bcsc.h"
#include "gap.h"

#include "ke_mem.h"

/*
 * GLOBAL VARIABLES DECLARATION
 ****************************************************************************************
 */


/*
 * GLOBAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Initialization of the BCSC module.
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
static uint8_t bcsc_init(struct prf_task_env *env, uint16_t *start_hdl, uint16_t app_task, uint8_t sec_lvl,  void *params)
{
    uint8_t idx;
    //-------------------- allocate memory required for the profile  ---------------------

    struct bcsc_env_tag *bcsc_env =
        (struct bcsc_env_tag *) ke_malloc(sizeof(struct bcsc_env_tag), KE_MEM_ATT_DB);

    // allocate BCSC required environment variable
    env->env = (prf_env_t *) bcsc_env;

    bcsc_env->prf_env.app_task = app_task
                                 | (PERM_GET(sec_lvl, SVC_MI) ? PERM(PRF_MI, ENABLE) : PERM(PRF_MI, DISABLE));
    bcsc_env->prf_env.prf_task = env->task | PERM(PRF_MI, ENABLE);

    // initialize environment variable
    env->id                     = TASK_ID_BCSC;
    bcsc_task_init(&(env->desc));

    for (idx = 0; idx < BCSC_IDX_MAX ; idx++)
    {
        bcsc_env->env[idx] = NULL;
        // service is ready, go into an Idle state
        ke_state_set(KE_BUILD_ID(env->task, idx), BCSC_FREE);
    }

    return GAP_ERR_NO_ERROR;
}

/**
 ****************************************************************************************
 * @brief Clean-up connection dedicated environment parameters
 * This function performs cleanup of ongoing operations
 * @param[in|out]    env        Collector or Service allocated environment data.
 * @param[in]        conidx     Connection index
 * @param[in]        reason     Detach reason
 ****************************************************************************************
 */
static void bcsc_cleanup(struct prf_task_env *env, uint8_t conidx, uint8_t reason)
{
    struct bcsc_env_tag *bcsc_env = (struct bcsc_env_tag *) env->env;

    // clean-up environment variable allocated for task instance
    if (bcsc_env->env[conidx] != NULL)
    {
        if (bcsc_env->env[conidx]->meas_evt_msg != 0)
        {
            ke_msg_free(bcsc_env->env[conidx]->meas_evt_msg);
        }

        ke_free(bcsc_env->env[conidx]);
        bcsc_env->env[conidx] = NULL;
    }

    /* Put BSC Client in Free state */
    ke_state_set(KE_BUILD_ID(env->task, conidx), BCSC_FREE);
}

/**
 ****************************************************************************************
 * @brief Destruction of the BCSC module - due to a reset for instance.
 * This function clean-up allocated memory (attribute database is destroyed by another
 * procedure)
 *
 * @param[in|out]    env        Collector or Service allocated environment data.
 ****************************************************************************************
 */
static void bcsc_destroy(struct prf_task_env *env)
{
    uint8_t idx;
    struct bcsc_env_tag *bcsc_env = (struct bcsc_env_tag *) env->env;

    // cleanup environment variable for each task instances
    for (idx = 0; idx < BCSC_IDX_MAX ; idx++)
    {
        bcsc_cleanup(env, idx, 0);
    }

    // free profile environment variables
    env->env = NULL;
    ke_free(bcsc_env);
}

/**
 ****************************************************************************************
 * @brief Handles Connection creation
 *
 * @param[in|out]    env        Collector or Service allocated environment data.
 * @param[in]        conidx     Connection index
 ****************************************************************************************
 */
static void bcsc_create(struct prf_task_env *env, uint8_t conidx)
{
    /* Put BCS Client in Idle state */
    ke_state_set(KE_BUILD_ID(env->task, conidx), BCSC_IDLE);

}

/// BCSC Task interface required by profile manager
const struct prf_task_cbs bcsc_itf =
{
    bcsc_init,
    bcsc_destroy,
    bcsc_create,
    bcsc_cleanup,
};

/*
 * GLOBAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

const struct prf_task_cbs *bcsc_prf_itf_get(void)
{
    return &bcsc_itf;
}

/**
 ****************************************************************************************
 * @brief Sends a BCSC_ENABLE_RSP messge.
 *
 * @param[in] bcsc_env -- the profile environment for the connection
 * @param[in] conidx   -- Connection Identifier
 * @param[in] status   -- indicates the outcome of the operation
 *
 * @return none.
 ****************************************************************************************
 */
void bcsc_enable_rsp_send(struct bcsc_env_tag *bcsc_env, uint8_t conidx, uint8_t status)
{
    // Send to APP the details of the discovered attributes on BCSC
    struct  bcsc_enable_rsp *rsp = KE_MSG_ALLOC(
                                       BCSC_ENABLE_RSP,
                                       prf_dst_task_get(&(bcsc_env->prf_env), conidx),
                                       prf_src_task_get(&(bcsc_env->prf_env), conidx),
                                       bcsc_enable_rsp);

    rsp->status = status;
    bcsc_env->env[conidx]->meas_evt_msg = 0;
    if (status == GAP_ERR_NO_ERROR)
    {
        rsp->bcs = bcsc_env->env[conidx]->bcs;

        // Register BCSC task in gatt for indication/notifications
        prf_register_atthdl2gatt(&(bcsc_env->prf_env), conidx, &(bcsc_env->env[conidx]->bcs.svc));
        // Go to connected state
        ke_state_set(prf_src_task_get(&(bcsc_env->prf_env), conidx), BCSC_BUSY);
    }
    else
    {
        memset(&rsp->bcs, 0, sizeof(rsp->bcs));
        // Enabled failed - so go back to the FREE state
        ke_state_set(prf_src_task_get(&(bcsc_env->prf_env), conidx), BCSC_FREE);
    }

    ke_msg_send(rsp);
}

/**
 ****************************************************************************************
 * @brief Sends a Complete_Event to the App
 *
 * @param[in] bcsc_env -- the profile environment
 * @param[in] conidx    -- Connection Identifier
 * @param[in] operation -- Indicates the operation for which the cmp_evt is being sent.
 * @param[in] status -- indicates the outcome of the operation
 *
 * @return Error_Code.
 ****************************************************************************************
 */

void bcsc_send_cmp_evt(struct bcsc_env_tag *bcsc_env, uint8_t conidx, uint8_t operation, uint8_t status)
{

    // Go back to the CONNECTED state if the state is busy
    if (ke_state_get(prf_src_task_get(&(bcsc_env->prf_env), conidx)) == BCSC_BUSY)
    {
        ke_state_set(prf_src_task_get(&(bcsc_env->prf_env), conidx), BCSC_IDLE);
    }

    bcsc_env->env[conidx]->op_pending = 0;
    // Send the message
    struct bcsc_cmp_evt *evt = KE_MSG_ALLOC(BCSC_CMP_EVT,
                                            prf_dst_task_get(&(bcsc_env->prf_env), conidx),
                                            prf_src_task_get(&(bcsc_env->prf_env), conidx),
                                            bcsc_cmp_evt);

    evt->operation  = operation;
    evt->status     = status;

    ke_msg_send(evt);
}

#endif //(BLE_BCS_CLIENT)

/// @} BCS
