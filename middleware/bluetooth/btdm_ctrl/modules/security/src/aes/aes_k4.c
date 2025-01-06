/**
 ****************************************************************************************
 * *
 * @file aes_k4.c
 *
 * @brief Definition file for AES K4 crypto module functions
 *
 * Copyright (C) RivieraWaves 2017-2018
 *
 *
 * Algorithm:
 *   The inputs to function k4 are:
 *      - N is 128 bits
 *   The key (T) is computed as follows:
 *      - T = AES-CMAC(SALT, N)
 *   SALT is a 128-bit value computed as follows:
 *      - SALT = s1(“smk4”)
 *   The output of the derivation function k4 is as follows:
 *        k4(N) = AES-CMAC(T, “id6” || 0x01 ) mod 2^6
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

/// Structure definition of the AES-K4 algorithm
struct aes_k4_env
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

/// Pre-computed value of Salted Key s1("smk4")
const uint8_t aes_k4_salt[AES_BLOCK_SIZE] = {0xBE, 0x49, 0x5F, 0xAC, 0x54, 0xEE, 0x97, 0x4C, 0x87, 0x66, 0xFA, 0xCE, 0xB7, 0xC1, 0x9A, 0x0E};

/// "id6" || 0x01
const uint8_t aes_k4_id6[] = {0x01, 0x36, 0x64, 0x69};
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
 * @brief Continue AES K4 algorithm
 *
 * @param[in] env       AES K4 Environment
 * @param[in] aes_res   AES Result
 *
 * @return True if algorithm is over, False else
 ****************************************************************************************
 */
__STATIC bool aes_k4_continue(struct aes_k4_env *env, uint8_t *aes_res)
{
    bool finished = false;

    finished = aes_cmac_continue(&(env->cmac_env), aes_res);

    if (finished)
    {
        if (env->compute_t)
        {
            // save computed T = AES-CMAC(SALT, N)
            memcpy(env->t, aes_res, AES_BLOCK_SIZE);
            env->compute_t = false;
            finished = false;

            // start execution AES CMAC - Second run to compute K4(N) = AES-CMAC(T, "id6" | 0x01)
            aes_cmac_start(&(env->cmac_env), env->t, aes_k4_id6, sizeof(aes_k4_id6));
        }
        else
        {
            // k4(N) = AES-CMAC(T, “id6” || 0x01 ) mod 2^6
            aes_res[0] = aes_res[0] & 0x3F;
        }
    }

    return finished;
}


/*
 * EXPORTED FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void aes_k4(const uint8_t *n, aes_func_result_cb res_cb, uint32_t metainfo)
{
    struct aes_k4_env *aes_k4;

    // Allocate AES-K4 environment memory
    aes_k4 = (struct aes_k4_env *) aes_alloc(sizeof(struct aes_k4_env), (aes_func_continue_cb) aes_k4_continue,
             res_cb, metainfo);

    // Save input parameters
    aes_k4->compute_t = true;

    // start execution AES CMAC - First run to compute T
    aes_cmac_start(&(aes_k4->cmac_env), aes_k4_salt, n, KEY_LEN);
}

/// @} AES
