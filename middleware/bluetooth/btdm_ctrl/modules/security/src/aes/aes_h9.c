/**
 ****************************************************************************************
 * *
 * @file aes_h9.c
 *
 * @brief Definition file for Group Long Term Key Generation Function h9 functions
 *
 * Copyright (C) RivieraWaves 2009-2017
 *
 *
 * Algorithm:
 *
 * W     is 128 bits
 * KeyID is 32 bits
 *
 * P = AES-CMAC(W, "ISOC")
 *
 * H9(W, KeyID) = AES-CMAC(P, KeyID)
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

/// H9 computation state
enum aes_h9_state
{
    /// Compute P
    /// > P = AES-CMAC(W, K)
    AES_H9_P_GEN,
    /// Finish H9 Computation:
    /// > H9(W, KeyID) = AES-CMAC(P, KeyID)
    AES_H9_FINISH,
};

/*
 * TYPES DEFINITION
 *****************************************************************************************
 */
/// H9 Function environment variable
struct aes_h9_env
{
    /// H9 requires to perform two rounds of AES CMAC
    struct aes_cmac_env cmac_env;
    /// Key id must be kept because used only in second round
    uint8_t key_id[KEY_ID_LEN];
    /// H9 computation state (@see enum aes_h9_state)
    uint8_t state;
};

/*
 * GLOBAL VARIABLE DEFINITION
 *****************************************************************************************
 */
/// h9() algorithm : "ISOC"
__STATIC const uint8_t aes_h9_isoc[] = {'I', 'S', 'O', 'C' };


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
 * @brief Continue H9 algorithm
 *
 * @param[in] env           H9 environment
 * @param[in] aes_res       Result of AES computation
 *
 * @return True if algorithm is over, False else
 ****************************************************************************************
 */
__STATIC bool aes_h9_continue(struct aes_h9_env *env, uint8_t *aes_res)
{
    bool finished = false;

    // Let continue AES CMAC continue
    if (aes_cmac_continue(&(env->cmac_env), aes_res))
    {
        // AES CMAC is Finished

        // P = AES-CMAC(W, "ISOC")
        if (env->state == AES_H9_P_GEN)
        {
            // finished H9 computation - AES CMAC second round
            env->state = AES_H9_FINISH;

            // P is value of AES_RES
            aes_cmac_start(&(env->cmac_env), aes_res, env->key_id, KEY_ID_LEN);
        }
        // H9(W, KeyID) = AES-CMAC(P, KeyID)
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

void aes_h9(const uint8_t *w, const uint8_t *key_id, aes_func_result_cb res_cb, uint32_t metainfo)
{
    struct aes_h9_env *env;

    // Allocate AES CMAC environent memory
    env = (struct aes_h9_env *) aes_alloc(sizeof(struct aes_h9_env), (aes_func_continue_cb) aes_h9_continue,
                                          res_cb, metainfo);

    memcpy(env->key_id, key_id, KEY_ID_LEN);

    env->state = AES_H9_P_GEN;

    // start execution AES CMAC
    aes_cmac_start(&(env->cmac_env), w, aes_h9_isoc, KEY_ID_LEN);
}


#endif // (BLE_EMB_PRESENT || BLE_HOST_PRESENT)

/// @} AES_CMAC
