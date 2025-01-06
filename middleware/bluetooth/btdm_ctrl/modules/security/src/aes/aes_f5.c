/**
 ****************************************************************************************
 * *
 * @file aes_f5.c
 *
 * @brief Definition file for LE Secure Connections Key Generation Function f5
 *
 * Copyright (C) RivieraWaves 2009-2018
 *
 * Algorithm:
 *
 * W     is 256 bits
 * N1    is 128 bits
 * N2    is 128 bits
 * A1    is  56 bits
 * A2    is  56 bits
 *
 * T = AES_CMAC(SALT, W)
 *
 * SALT = 0x6C88 8391 AAF5 A538 6037 0BDB 5A60 83BE
 *
 * KeyId = "btle"
 *
 * f5(W, N1, N2, A1, A2) =  AES-CMAC(T, 0 || key_id || N1 || N2 || A1 || A2 || 0x0100)
 *                       || AES-CMAC(T, 1 || key_id || N1 || N2 || A1 || A2 || 0x0100)
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
#define AES_F5_W_LEN    32
#define AES_F5_MSG_LEN  53

/*
 * TYPES DEFINITION
 *****************************************************************************************
 */

/// f5 computation step
enum aes_f5_step
{
    /// Compute the T
    /// > T = AES_CMAC(SALT, W)
    AES_F5_T_GEN,
    /// > MacKey = AES-CMAC(T, 0 || key_id || N1 || N2 || A1 || A2 || 0x0100)
    AES_F5_COUNT_0,
    /// > LTK = AES-CMAC(T, 1 || key_id || N1 || N2 || A1 || A2 || 0x0100)
    AES_F5_COUNT_1,
};


/// f5 Function environment variable
struct aes_f5_env
{
    /// f4 requires to perform two rounds of AES CMAC
    struct aes_cmac_env cmac_env;
    /// Dedicated result callback because result length is 256 bits
    aes_func_result_cb  res_cb;
    /// Computed T
    uint8_t t[KEY_LEN];
    /// M =  counter || key_id || N1 || N2 || A1 || A2 || 0x0100
    uint8_t m[AES_F5_MSG_LEN];
    /// concatenation of CMAC for counter = 0 and counter = 1
    uint8_t res[KEY_LEN * 2];
    /// Execution step (@see enum aes_f5_step)
    uint8_t step;
};

/*
 * GLOBAL VARIABLE DEFINITION
 *****************************************************************************************
 */
/// Key identifier "btle"
__STATIC const uint8_t aes_f5_key_id[4]    = {0x65, 0x6c, 0x74, 0x62};
/// Salted key
__STATIC const uint8_t aes_f5_key_salt[16] = {0xBE, 0x83, 0x60, 0x5A, 0xDB, 0x0B, 0x37, 0x60, 0x38, 0xA5, 0xF5, 0xAA, 0x91, 0x83, 0x88, 0x6C};


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
 * @brief Continue f5 algorithm
 *
 * @param[in] env           f5 environment
 * @param[in] aes_res       Result of AES computation
 *
 * @return True if algorithm is over, False else
 ****************************************************************************************
 */
__STATIC bool aes_f5_continue(struct aes_f5_env *env, uint8_t *aes_res)
{
    bool finished = false;

    // Let continue AES CMAC continue
    if (aes_cmac_continue(&(env->cmac_env), aes_res))
    {
        switch (env->step)
        {
        case AES_F5_T_GEN:
        {
            memcpy(env->t, aes_res, KEY_LEN);

            // start execution AES CMAC
            env->m[52] = 0x00; // Counter = 0x00 for MAC Key
            env->step  = AES_F5_COUNT_0;
            aes_cmac_start(&(env->cmac_env), env->t, env->m, AES_F5_MSG_LEN);

        }
        break;
        case AES_F5_COUNT_0:
        {
            memcpy(&(env->res[0]), aes_res, KEY_LEN);

            // start execution AES CMAC
            env->m[52] = 0x01; // Counter = 0x01 for LTK
            env->step  = AES_F5_COUNT_1;
            aes_cmac_start(&(env->cmac_env), env->t, env->m, AES_F5_MSG_LEN);

        }
        break;
        case AES_F5_COUNT_1:
        {
            memcpy(&(env->res[KEY_LEN]), aes_res, KEY_LEN);

            // inform requester of AES result.
            env->res_cb(CO_ERROR_NO_ERROR, env->res, env->cmac_env.aes_env.metainfo);

            finished = true;
        }
        break;
        default: { /* nothing to do */ } break;
        }
    }

    return finished;
}

/*
 * EXPORTED FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void aes_f5(const uint8_t *w, const uint8_t *n1, const uint8_t *n2, const uint8_t *a1, const uint8_t *a2,
            aes_func_result_cb res_cb, uint32_t metainfo)
{
    struct aes_f5_env *env;

    // Allocate AES CMAC environent memory
    env = (struct aes_f5_env *) aes_alloc(sizeof(struct aes_f5_env), (aes_func_continue_cb) aes_f5_continue,
                                          NULL, metainfo);

    env->step   = AES_F5_T_GEN;
    env->res_cb = res_cb;

    // Length field is 256 - 0x0100
    env->m[0] = 0x00;
    env->m[1] = 0x01;
    memcpy((void *) & (env->m[2]),  a2,            7);
    memcpy((void *) & (env->m[9]),  a1,            7);
    memcpy((void *) & (env->m[16]), n2,           16);
    memcpy((void *) & (env->m[32]), n1,           16);
    memcpy((void *) & (env->m[48]), aes_f5_key_id, 4);

    // start execution AES CMAC
    aes_cmac_start(&(env->cmac_env), aes_f5_key_salt, w, AES_F5_W_LEN);
}

#endif // (BLE_HOST_PRESENT)

/// @} AES_CMAC
