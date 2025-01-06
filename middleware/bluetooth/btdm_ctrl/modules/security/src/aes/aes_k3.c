/**
 ****************************************************************************************
 * *
 * @file aes_k3.c
 *
 * @brief Definition file for AES K3 crypto module functions
 *
 * Copyright (C) RivieraWaves 2017-2018
 *
 *
 * Algorithm:
 *   The inputs to function k3 are:
 *      - N is 128 bits
 *   The key (T) is computed as follows:
 *      - T = AES-CMAC(SALT, N)
 *   SALT is a 128-bit value computed as follows:
 *      - SALT = s1(“smk3”)
 *   The output of the derivation function k3 is as follows:
 *        k3(N) = AES-CMAC(T, “id64” || 0x01 ) mod 2^64
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup AES
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"
#include "aes_int.h"     // AES internals
#include <string.h>      // for memcpy

/*
 * DEFINES
 ****************************************************************************************
 */

/*
 * TYPES DEFINITION
 *****************************************************************************************
 */

/// Structure definition of the AES-K3 algorithm
struct aes_k3_env
{
    /// AES-CMAC Environment structure
    struct aes_cmac_env cmac_env;
    /// Used to know if T computation is on-going
    uint8_t             compute_t;
    /// T: message authentication code
    uint8_t             t[AES_BLOCK_SIZE];
};

/*
 * GLOBAL VARIABLE DEFINITION
 *****************************************************************************************
 */


/// Pre-computed value of Salted Key s1("smk3")
const uint8_t aes_k3_salt[AES_BLOCK_SIZE] = {0x02, 0xC3, 0x91, 0x62, 0x13, 0x6E, 0x71, 0x8A, 0xCC, 0x95, 0xF1, 0x03, 0x35, 0x44, 0x36, 0x00};

/// "id64" || 0x01
const uint8_t aes_k3_id64[] = {0x01, 0x34, 0x36, 0x64, 0x69};
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
 * @brief Continue AES K3 algorithm
 *
 * @param[in] env       AES K3 Environment
 * @param[in] aes_res   AES Result
 *
 * @return True if algorithm is over, False else
 ****************************************************************************************
 */
__STATIC bool aes_k3_continue(struct aes_k3_env *env, uint8_t *aes_res)
{
    bool finished = false;

    finished = aes_cmac_continue(&(env->cmac_env), aes_res);

    if (finished && env->compute_t)
    {
        // save computed T = AES-CMAC(SALT, N)
        memcpy(env->t, aes_res, AES_BLOCK_SIZE);
        env->compute_t = false;
        finished = false;

        // start execution AES CMAC - Second run to compute K3(N) = AES-CMAC(T, "id64" | 0x01)
        aes_cmac_start(&(env->cmac_env), env->t, aes_k3_id64, sizeof(aes_k3_id64));
    }

    return finished;
}


/*
 * EXPORTED FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void aes_k3(const uint8_t *n, aes_func_result_cb res_cb, uint32_t metainfo)
{
    struct aes_k3_env *aes_k3;

    // Allocate AES-K3 environment memory
    aes_k3 = (struct aes_k3_env *) aes_alloc(sizeof(struct aes_k3_env), (aes_func_continue_cb) aes_k3_continue,
             res_cb, metainfo);

    // Save input parameters
    aes_k3->compute_t = true;

    // start execution AES CMAC - First run to compute T
    aes_cmac_start(&(aes_k3->cmac_env), aes_k3_salt, n, KEY_LEN);
}

/// @} AES
