/**
 ****************************************************************************************
 * *
 * @file aes_h6.c
 *
 * @brief Definition file for h6 key conversion of a given size from one key type to
 * another key type with equivalent strength.
 *
 * Copyright (C) RivieraWaves 2009-2019
 *
 *
 * Algorithm:
 *
 * W     is 128 bits
 * KeyID is 32 bits
 *
 * H6(W, KeyID) = AES-CMAC(W, KeyID)
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
// size of key id parameter
#define KEY_ID_LEN 4


/*
 * TYPES DEFINITION
 *****************************************************************************************
 */
/// H6 Function environement variable
struct aes_h6_env
{
    /// H6 requires to perform two rounds of AES CMAC
    struct aes_cmac_env cmac_env;
    /// Key id must be kept
    uint8_t key_id[KEY_ID_LEN];
    /// W must be kept
    uint8_t w[KEY_LEN];
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
 * @brief Continue H6 algorithm
 *
 * @param[in] env           H6 environment
 * @param[in] aes_res       Result of AES computation
 *
 * @return True if algorithm is over, False else
 ****************************************************************************************
 */
__STATIC bool aes_h6_continue(struct aes_h6_env *env, uint8_t *aes_res)
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

void aes_h6(const uint8_t *w, const uint8_t *key_id, aes_func_result_cb res_cb, uint32_t metainfo)
{
    struct aes_h6_env *env;

    // Allocate AES CMAC environent memory
    env = (struct aes_h6_env *) aes_alloc(sizeof(struct aes_h6_env), (aes_func_continue_cb) aes_h6_continue,
                                          res_cb, metainfo);

    memcpy(env->key_id, key_id, KEY_ID_LEN);
    memcpy(env->w,      w,      KEY_LEN);

    // start execution AES CMAC
    aes_cmac_start(&(env->cmac_env), env->w, env->key_id, KEY_ID_LEN);
}


#endif // (BLE_EMB_PRESENT || BLE_HOST_PRESENT)

/// @} AES_CMAC
