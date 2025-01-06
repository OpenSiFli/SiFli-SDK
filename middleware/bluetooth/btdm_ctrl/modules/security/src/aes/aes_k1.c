/**
 ****************************************************************************************
 * *
 * @file aes_k1.c
 *
 * @brief Definition file for AES K1 crypto module functions
 *
 * Copyright (C) RivieraWaves 2017-2018
 *
 *
 * Algorithm:
 *   The inputs to function k1 are:
 *       - N is 0 or more octets
 *       - SALT is 128 bits
 *       - P is 0 or more octets
 *   The key (T) is computed as follows:
 *       - T = AES-CMAC(SALT, N)
 *   The output of the key generation function k1 is as follows:
 *
 *       k1(N, SALT, P) = AES-CMAC(T, P)
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

/// Structure definition of the AES-K1 algorithm
struct aes_k1_env
{
    /// AES-CMAC Environment structure
    struct aes_cmac_env cmac_env;
    /// P: Pointer to P input
    const uint8_t      *p;
    /// Length of P
    uint8_t             p_len;
    /// Used to know if T computation is on-going
    uint8_t             compute_t;
    /// T: message authentication code
    uint8_t             t[AES_BLOCK_SIZE];
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
 * @brief Continue AES K1 algorithm
 *
 * @param[in] env       AES K1 Environment
 * @param[in] aes_res   AES Result
 *
 * @return True if algorithm is over, False else
 ****************************************************************************************
 */
__STATIC bool aes_k1_continue(struct aes_k1_env *env, uint8_t *aes_res)
{
    bool finished = false;

    finished = aes_cmac_continue(&(env->cmac_env), aes_res);

    if (finished && env->compute_t)
    {
        // save computed T = AES-CMAC(SALT, N)
        memcpy(env->t, aes_res, AES_BLOCK_SIZE);
        env->compute_t = false;
        finished = false;

        // start execution AES CMAC - Second run to compute K1(N, SALT, P)
        aes_cmac_start(&(env->cmac_env), env->t, env->p, env->p_len);
    }

    return finished;
}


/*
 * EXPORTED FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void aes_k1(const uint8_t *salt, const uint8_t *n, uint8_t n_len, const uint8_t *p, uint8_t p_len,
            aes_func_result_cb res_cb, uint32_t metainfo)
{
    struct aes_k1_env *aes_k1;

    // Allocate AES-K1 environment memory
    aes_k1 = (struct aes_k1_env *) aes_alloc(sizeof(struct aes_k1_env), (aes_func_continue_cb) aes_k1_continue,
             res_cb, metainfo);

    // Save input parameters
    aes_k1->p         = p;
    aes_k1->p_len     = p_len;
    aes_k1->compute_t = true;

    // start execution AES CMAC - First run to compute T
    aes_cmac_start(&(aes_k1->cmac_env), salt, n, n_len);
}

/// @} AES_CMAC
