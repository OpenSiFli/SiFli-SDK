/**
 ****************************************************************************************
 * *
 * @file aes_f4.c
 *
 * @brief LE Secure Connections Confirm Value Generation Function f4
 *
 * Copyright (C) RivieraWaves 2009-2018
 *
 *
 * Algorithm:
 *
 * U     is 256 bits
 * V     is 256 bits
 * X     is 128 bits
 * Z     is   8 bits
 *
 * f4(U, V, X, Z) = AES-CMAC(X, U || V || Z)
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup AES_CMAC
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"
#if (BLE_HOST_PRESENT)

#include "arch.h"     // architecture defines
#include "aes_int.h"  // AES internals
#include <string.h>   // for memcpy function

/*
 * DEFINES
 ****************************************************************************************
 */
#define AES_F4_MSG_LEN  65


/*
 * TYPES DEFINITION
 *****************************************************************************************
 */
/// f4 Function environement variable
struct aes_f4_env
{
    /// f4 requires to perform two rounds of AES CMAC
    struct aes_cmac_env cmac_env;
    /// M = U || V || Z
    uint8_t m[AES_F4_MSG_LEN];
};

/*
 * GLOBAL VARIABLE DEFINITION
 *****************************************************************************************
 */


/*
 * MODULE INTERNAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */


/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Continue f4 algorithm
 *
 * @param[in] env           F4 environment
 * @param[in] aes_res       Result of AES computation
 *
 * @return True if algorithm is over, False else
 ****************************************************************************************
 */
__STATIC bool aes_f4_continue(struct aes_f4_env *env, uint8_t *aes_res)
{
    // Let continue AES CMAC continue
    return (aes_cmac_continue(&(env->cmac_env), aes_res));
}

/*
 * EXPORTED FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void aes_f4(const uint8_t *u, const uint8_t *v, const uint8_t *x, uint8_t z,
            aes_func_result_cb res_cb, uint32_t metainfo)
{
    struct aes_f4_env *env;

    // Allocate AES CMAC environent memory
    env = (struct aes_f4_env *) aes_alloc(sizeof(struct aes_f4_env), (aes_func_continue_cb) aes_f4_continue,
                                          res_cb, metainfo);
    env->m[0] = z;
    memcpy(&(env->m[1]),  v, 32);
    memcpy(&(env->m[33]), u, 32);

    // start execution AES CMAC
    aes_cmac_start(&(env->cmac_env), x, env->m, AES_F4_MSG_LEN);
}

#endif // (BLE_HOST_PRESENT)

/// @} AES_CMAC
