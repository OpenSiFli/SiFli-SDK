/**
 ****************************************************************************************
 * *
 * @file aes_h8.c
 *
 * @brief Definition file for Group Session Key Derivation Function h8 functions
 *
 * Copyright (C) RivieraWaves 2009-2017
 *
 *
 * Algorithm:
 *
 * K     is 128 bits
 * S     is 128 bits
 * KeyID is 32 bits
 *
 * IK = AES-CMAC(S, K)
 *
 * H8(K, S, KeyID) = AES-CMAC(IK, KeyID)
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

/// H8 computation state
enum aes_h8_state
{
    /// Compute the IK (second key)
    /// > IK = AES-CMAC(S, K)
    AES_H8_IK_GEN,
    /// Finish H8 Computation:
    /// > H8(K, S, KeyID) = AES-CMAC(IK, KeyID)
    AES_H8_FINISH,
};

/*
 * TYPES DEFINITION
 *****************************************************************************************
 */
/// H8 Function environement variable
struct aes_h8_env
{
    /// H8 requires to perform two rounds of AES CMAC
    struct aes_cmac_env cmac_env;
    /// K field must be kept for AES CMAC procedure
    uint8_t k[KEY_LEN];
    /// S field must be kept for AES CMAC procedure
    uint8_t s[KEY_LEN];
    /// Key id must be kept because used only in second round
    uint8_t key_id[KEY_ID_LEN];
    /// H8 computation state (@see enum aes_h8_state)
    uint8_t state;
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
 * @brief Continue H8 algorithm
 *
 * @param[in] env           H8 environment
 * @param[in] aes_res       Result of AES computation
 *
 * @return True if algorithm is over, False else
 ****************************************************************************************
 */
__STATIC bool aes_h8_continue(struct aes_h8_env *env, uint8_t *aes_res)
{
    bool finished = false;

    // Let continue AES CMAC continue
    if (aes_cmac_continue(&(env->cmac_env), aes_res))
    {
        // AES CMAC is Finished

        // IK = AES-CMAC(S, K)
        if (env->state == AES_H8_IK_GEN)
        {
            // finished H8 computation - AES CMAC second round
            env->state = AES_H8_FINISH;

            // put result in s temporary
            memcpy(env->s,      aes_res,      KEY_LEN);
            // IK is value of AES_RES
            aes_cmac_start(&(env->cmac_env), env->s, env->key_id, KEY_ID_LEN);
        }
        // H8(K, S, KeyID) = AES-CMAC(IK, KeyID)
        else
        {
            finished = true;
        }
    }

    return finished;
}

/*
 * EXPORTED FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void aes_h8(const uint8_t *k, const uint8_t *s, const uint8_t *key_id, aes_func_result_cb res_cb, uint32_t metainfo)
{
    struct aes_h8_env *env;

    // Allocate AES CMAC environent memory
    env = (struct aes_h8_env *) aes_alloc(sizeof(struct aes_h8_env), (aes_func_continue_cb) aes_h8_continue,
                                          res_cb, metainfo);

    memcpy(env->k,      k,      KEY_LEN);
    memcpy(env->s,      s,      KEY_LEN);
    memcpy(env->key_id, key_id, KEY_ID_LEN);

    env->state = AES_H8_IK_GEN;

    // start execution AES CMAC
    aes_cmac_start(&(env->cmac_env), env->s, env->k, KEY_LEN);
}


#endif // (BLE_EMB_PRESENT || BLE_HOST_PRESENT)

/// @} AES_CMAC
