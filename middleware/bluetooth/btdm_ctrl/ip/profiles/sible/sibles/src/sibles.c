/**
 ****************************************************************************************
 *
 * @file sibles.c
 *
 * @brief SIFLI BLE Server Implementation.
 *
 * Copyright (C) Sifli 2019-2022
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup SIFLIS
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
#include "attm.h"
#include "sibles.h"
#include "sibles_task.h"
#include "prf_utils.h"
#include "prf.h"

#include "ke_mem.h"
#include "ke_msg.h"

#include "gapm.h"

/*
 * MACROS
 ****************************************************************************************
 */

/*
 * SIBLE ATTRIBUTES
 ****************************************************************************************
 */
struct sibles_env_tag sibles_env;


/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Initialization of the SIBLES module.
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
static uint8_t sibles_init(struct prf_task_env *env, uint16_t *start_hdl, uint16_t app_task, uint8_t sec_lvl)
{
    //------------------ create the attribute database for the profile -------------------

    // DB Creation Statis
    uint8_t status = ATT_ERR_NO_ERROR;

    // Use static memory to put to retention memory.
    //struct sibles_env_tag *sibles_env =
    //    (struct sibles_env_tag *) ke_malloc(sizeof(struct sibles_env_tag), KE_MEM_ATT_DB);
    struct sibles_env_tag *sibles_env_p = &sibles_env;

    memset(sibles_env_p, 0, sizeof(struct sibles_env_tag));

    // allocate SIBLES required environment variable
    env->env = (prf_env_t *) sibles_env_p;
    sibles_env_p->prf_env.app_task = app_task
                                     | (PERM_GET(sec_lvl, SVC_MI) ? PERM(PRF_MI, ENABLE) : PERM(PRF_MI, ENABLE));
    sibles_env_p->prf_env.prf_task = env->task | PERM(PRF_MI, ENABLE);
    sibles_env_p->sec_lvl = sec_lvl;

    // initialize environment variable
    env->id = TASK_ID_SIBLES;
    sibles_task_init(&(env->desc));
    co_list_init(&(sibles_env_p->values));
    sibles_env_p->req_hdl = 0;
    sibles_env_p->req_conidx = 0;
    sibles_env_p->start_hdl = 0;
    // service is ready, go into an Idle state
    ke_state_set(env->task, SIBLES_IDLE);
#ifdef CFG_PRF_SIBLE_FORWARD
    {
#ifndef PLF_UART2_OVERMBOX
        void *no_param = ke_msg_alloc(SIBLES_SVC_READY_IND, 0, 0, 0);
#else
        ke_task_id_t *no_param = ke_msg_alloc(SIBLES_SVC_READY_IND, gapm_get_AHI_task_id(), prf_get_task_from_id(TASK_ID_SIBLES), sizeof(ke_task_id_t));
        *no_param = env->task;
#endif
        sifli_mbox_bcpu2hcpu((uint8_t *)no_param);
    }
#endif

    return (status);
}
/**
 ****************************************************************************************
 * @brief Destruction of the SIBLES module - due to a reset for instance.
 * This function clean-up allocated memory (attribute database is destroyed by another
 * procedure)
 *
 * @param[in|out]    env        Collector or Service allocated environment data.
 ****************************************************************************************
 */
static void sibles_destroy(struct prf_task_env *env)
{
    struct sibles_env_tag *sibles_env_p = (struct sibles_env_tag *) env->env;

    // remove all values present in list
    while (!co_list_is_empty(&(sibles_env_p->values)))
    {
        struct sibles_va_elmt *hdr = (struct sibles_va_elmt *) co_list_pop_front(&(sibles_env_p->values));
        ke_free(hdr);
    }
    // free profile environment variables
    env->env = NULL;
    //ke_free(sibles_env);
}

/**
 ****************************************************************************************
 * @brief Handles Connection creation
 *
 * @param[in|out]    env        Collector or Service allocated environment data.
 * @param[in]        conidx     Connection index
 ****************************************************************************************
 */
static void sibles_create(struct prf_task_env *env, uint8_t conidx)
{
    sibles_env.req_conidx = conidx;
    ke_msg_send_basic(SIBLES_CONNECTED_IND, gapm_get_AHI_task_id(), prf_src_task_get(&(sibles_env.prf_env), conidx));
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


#if (NVDS_SUPPORT)
    #include "sifli_nvds.h"
#endif

static void sibles_cleanup(struct prf_task_env *env, uint8_t conidx, uint8_t reason)
{
    sibles_env.req_conidx = 0;
}

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// SIBLES Task interface required by profile manager
const struct prf_task_cbs sibles_itf =
{
    (prf_init_fnct) sibles_init,
    sibles_destroy,
    sibles_create,
    sibles_cleanup,
};

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

const struct prf_task_cbs *sibles_prf_itf_get(void)
{
    return &sibles_itf;
}

#endif //BLE_SIBLE_SERVER
#endif //(BLE_APP_PRESENT)
/// @} SIBLES
