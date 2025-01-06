/**
 ****************************************************************************************
 * *
 * @file aes_f6.c
 *
 * @brief LE Secure Connections Check Value Generation Function f6
 *
 * Copyright (C) RivieraWaves 2009-2018
 *
 * Algorithm:
 *      W     is 128 bits
 *      N1    is 128 bits
 *      N2    is 128 bits
 *      R     is 128 bits
 *      IOcap is  24 bits
 *      A1    is  56 bits
 *      A2    is  56 bits
 *
 * f6(W, N1, N2, R, IOCap, A1, A2) = AES-CMAC(W, N1 || N2 || R || IOCap || A1 || A2)
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
#define AES_F6_MSG_LEN  65


/*
 * TYPES DEFINITION
 *****************************************************************************************
 */
/// f6 Function environment variable
struct aes_f6_env
{
    /// f6 requires to perform two rounds of AES CMAC
    struct aes_cmac_env cmac_env;
    /// M = N1 || N2 || R || IOCap || A1 || A2
    uint8_t m[AES_F6_MSG_LEN];
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
 * @brief Continue f6 algorithm
 *
 * @param[in] env           F4 environment
 * @param[in] aes_res       Result of AES computation
 *
 * @return True if algorithm is over, False else
 ****************************************************************************************
 */
__STATIC bool aes_f6_continue(struct aes_f6_env *env, uint8_t *aes_res)
{
    // Let continue AES CMAC continue
    return (aes_cmac_continue(&(env->cmac_env), aes_res));
}

/*
 * EXPORTED FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void aes_f6(const uint8_t *w, const uint8_t *n1, const uint8_t *n2, const uint8_t *r,  const uint8_t *iocap,
            const uint8_t *a1, const uint8_t *a2, aes_func_result_cb res_cb, uint32_t metainfo)
{
    struct aes_f6_env *env;

    // Allocate AES CMAC environent memory
    env = (struct aes_f6_env *) aes_alloc(sizeof(struct aes_f6_env), (aes_func_continue_cb) aes_f6_continue,
                                          res_cb, metainfo);

    memcpy(&(env->m[49]), n1,    16);
    memcpy(&(env->m[33]), n2,    16);
    memcpy(&(env->m[17]), r,     16);
    memcpy(&(env->m[14]), iocap,  3);
    memcpy(&(env->m[7]),  a1,     7);
    memcpy(&(env->m[0]),  a2,     7);

    // start execution AES CMAC
    aes_cmac_start(&(env->cmac_env), w, env->m, AES_F6_MSG_LEN);
}

#endif // (BLE_HOST_PRESENT)

/// @} AES_CMAC
