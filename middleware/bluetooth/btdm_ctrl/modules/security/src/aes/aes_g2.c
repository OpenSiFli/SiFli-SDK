/**
 ****************************************************************************************
 * *
 * @file aes_g2.c
 *
 * @brief LE Secure Connections Numeric Comparison Value Generation Function g2
 *
 * Copyright (C) RivieraWaves 2009-2018
 *
 * Algorithm:
 *     U is 256bit - 32 bytes
 *     V is 256bit - 32 bytes
 *     X is 128bit - 16 bytes
 *     Y is 128bit - 16 bytes
 *
 * g2(U, V, X, Y) = AES-CMAC(X, U || V || Y) mod 2^32
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
#define AES_G2_MSG_LEN  80


/*
 * TYPES DEFINITION
 *****************************************************************************************
 */
/// g2 Function environment variable
struct aes_g2_env
{
    /// g2 requires to perform two rounds of AES CMAC
    struct aes_cmac_env cmac_env;
    /// M = U || V || Y
    uint8_t m[AES_G2_MSG_LEN];
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
 * @brief Continue g2 algorithm
 *
 * @param[in] env           F4 environment
 * @param[in] aes_res       Result of AES computation
 *
 * @return True if algorithm is over, False else
 ****************************************************************************************
 */
__STATIC bool aes_g2_continue(struct aes_g2_env *env, uint8_t *aes_res)
{
    // Let continue AES CMAC continue
    return (aes_cmac_continue(&(env->cmac_env), aes_res));
}

/*
 * EXPORTED FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void aes_g2(const uint8_t *u, const uint8_t *v, const uint8_t *x, const uint8_t *y,
            aes_func_result_cb res_cb, uint32_t metainfo)
{
    struct aes_g2_env *env;

    // Allocate AES CMAC environent memory
    env = (struct aes_g2_env *) aes_alloc(sizeof(struct aes_g2_env), (aes_func_continue_cb) aes_g2_continue,
                                          res_cb, metainfo);

    memcpy(&(env->m[48]), u, 32);
    memcpy(&(env->m[16]), v, 32);
    memcpy(&(env->m[0]),  y, 16);

    // start execution AES CMAC
    aes_cmac_start(&(env->cmac_env), x, env->m, AES_G2_MSG_LEN);
}

#endif // (BLE_HOST_PRESENT)

/// @} AES_CMAC
