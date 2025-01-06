/**
 ****************************************************************************************
 * *
 * @file aes_h7.c
 *
 * @brief Definition file for h7 key conversion of a given size from one key type to
 * another key type with equivalent strength.
 *
 * Copyright (C) RivieraWaves 2009-2019
 *
 *
 * Algorithm:
 *
 * SALT     is 128 bits
 * W        is 128 bits
 *
 * H7(SALT, W) = AES-CMAC(SALT, W)
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

#if (BLE_EMB_PRESENT || BLE_HOST_PRESENT)

#include "arch.h"     // architecture defines
#include "aes_int.h"  // AES internals
#include <string.h>   // for memcpy function

/*
 * DEFINES
 ****************************************************************************************
 */

/*
 * TYPES DEFINITION
 *****************************************************************************************
 */
/// H7 Function environement variable
struct aes_h7_env
{
    /// H7 requires to perform two rounds of AES CMAC
    struct aes_cmac_env cmac_env;
    /// W must be kept
    uint8_t w[KEY_LEN];
    ///SALTW must be kept
    uint8_t salt[KEY_LEN];
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
 * @brief Continue H7 algorithm
 *
 * @param[in] env           H7 environment
 * @param[in] aes_res       Result of AES computation
 *
 * @return True if algorithm is over, False else
 ****************************************************************************************
 */
__STATIC bool aes_h7_continue(struct aes_h7_env *env, uint8_t *aes_res)
{
    bool finished = false;

    // Let continue AES CMAC continue
    if (aes_cmac_continue(&(env->cmac_env), aes_res))
    {
        // AES CMAC is Finished
        finished = true;
    }

    return finished;
}

/*
 * EXPORTED FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void aes_h7(const uint8_t *salt, const uint8_t *w, aes_func_result_cb res_cb, uint32_t metainfo)
{
    struct aes_h7_env *env;

    // Allocate AES CMAC environent memory
    env = (struct aes_h7_env *) aes_alloc(sizeof(struct aes_h7_env), (aes_func_continue_cb) aes_h7_continue,
                                          res_cb, metainfo);

    memcpy(env->salt, salt, KEY_LEN);
    memcpy(env->w,    w,    KEY_LEN);

    // start execution AES CMAC
    aes_cmac_start(&(env->cmac_env), env->salt, env->w, KEY_LEN);
}


#endif // (BLE_EMB_PRESENT || BLE_HOST_PRESENT)

/// @} AES_CMAC
