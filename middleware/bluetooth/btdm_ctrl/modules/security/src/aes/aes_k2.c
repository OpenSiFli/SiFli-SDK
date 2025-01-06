/**
 ****************************************************************************************
 * *
 * @file aes_k2.c
 *
 * @brief Definition file for AES K2 crypto module functions
 *
 * Copyright (C) RivieraWaves 2017-2018
 *
 *
 * Algorithm:
 *
 *   The inputs to function k2 are:
 *       - N is 128 bits
 *       - P is 1 or more octets
 *   The key (T) is computed as follows:
 *       - T = AES-CMACSALT (N)
 *   SALT is the 128-bit value computed as follows
 *       - SALT = s1(“smk2”)
 *   The output of the key generation function k2 is as follows:
 *       - T0 = empty string (zero length)
 *       - T1 = AES-CMACT (T0 || P || 0x01)
 *       - T2 = AES-CMACT (T1 || P || 0x02)
 *       - T3 = AES-CMACT (T2 || P || 0x03)
 *
 *       k2(N, P) = (T1 || T2 || T3) mod 2^263
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
#include "arch.h"        // Platform specific defines
#include "aes_int.h"     // AES internals
#include <string.h>      // for memcpy

/*
 * DEFINES
 ****************************************************************************************
 */

#define AES_K2_T_OFFSET  (0)
#define AES_K2_T1_OFFSET (AES_BLOCK_SIZE*2)
#define AES_K2_T2_OFFSET (AES_BLOCK_SIZE)
#define AES_K2_T3_OFFSET (0)


/// Computation state
enum aes_k2_state
{
    AES_K2_COMPUTE_T,
    AES_K2_COMPUTE_T1,
    AES_K2_COMPUTE_T2,
    AES_K2_COMPUTE_T3,
};



/*
 * TYPES DEFINITION
 *****************************************************************************************
 */

/// Structure definition of the AES-K1 algorithm
struct aes_k2_env
{
    /// AES-CMAC Environment structure
    struct aes_cmac_env cmac_env;
    /// Result callback
    aes_func_result_cb  res_cb;
    /// State of the function computation (@see enum aes_k2_state)
    uint8_t             state;
    /// Length of P + AES_BLOCK_SIZE + 1 (for (T0 || P || 0x01), (T1 || P || 0x02), (T2 || P || 0x03)
    uint8_t             p_in_len;
    /// Result value (T1 || T2 || T3)
    uint8_t             res[AES_BLOCK_SIZE * 3];
    /// P: Copy of P input value
    uint8_t             p_in[__ARRAY_EMPTY];
};

/*
 * GLOBAL VARIABLE DEFINITION
 *****************************************************************************************
 */

/// Pre-computed value of Salted Key s1("smk2")
const uint8_t aes_k2_salt[AES_BLOCK_SIZE] = {0xB1, 0x10, 0x8D, 0x4D, 0x1F, 0x97, 0x16, 0xFD, 0xBF, 0xBF, 0x71, 0x18, 0x0C, 0x48, 0x90, 0x4F};

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
 * @brief Continue AES K2 algorithm
 *
 * @param[in] env       AES K2 Environment
 * @param[in] aes_res   AES Result
 *
 * @return True if algorithm is over, False else
 ****************************************************************************************
 */
__STATIC bool aes_k2_continue(struct aes_k2_env *env, uint8_t *aes_res)
{
    bool finished = false;

    finished = aes_cmac_continue(&(env->cmac_env), aes_res);

    if (finished)
    {
        switch (env->state)
        {
        case  AES_K2_COMPUTE_T:
        {
            // save computed T = AES-CMAC(SALT, N)
            memcpy(&(env->res[AES_K2_T_OFFSET]), aes_res, AES_BLOCK_SIZE);

            // start execution AES CMAC - Second run to compute T0
            aes_cmac_start(&(env->cmac_env), &(env->res[AES_K2_T_OFFSET]), NULL, 0);

            // prepare T1 computation input: (T0 || P || 0x01) where T0 is an empty string
            env->p_in[0] = 0x01;
            // start execution AES CMAC - Compute T1 (P || 0x01)
            aes_cmac_start(&(env->cmac_env), &(env->res[AES_K2_T_OFFSET]), env->p_in, env->p_in_len - AES_BLOCK_SIZE);
            finished = false;
            env->state = AES_K2_COMPUTE_T1;
        }
        break;

        case  AES_K2_COMPUTE_T1:
        {
            // save computed T1 = AES-CMAC(T, (T0 || P || 0x01))
            memcpy(&(env->res[AES_K2_T1_OFFSET]), aes_res, AES_BLOCK_SIZE);

            // and prepare T2 computation input: (T1 || P || 0x02)
            memcpy(&(env->p_in[env->p_in_len - AES_BLOCK_SIZE]), aes_res, AES_BLOCK_SIZE);
            env->p_in[0] = 0x02;

            // start execution AES CMAC - Compute T2
            aes_cmac_start(&(env->cmac_env), &(env->res[AES_K2_T_OFFSET]), env->p_in, env->p_in_len);
            finished = false;
            env->state = AES_K2_COMPUTE_T2;
        }
        break;
        case  AES_K2_COMPUTE_T2:
        {
            // save computed T2 = AES-CMAC(T, (T1 || P || 0x02))
            memcpy(&(env->res[AES_K2_T2_OFFSET]), aes_res, AES_BLOCK_SIZE);

            // and prepare T3 computation input: (T2 || P || 0x03)
            memcpy(&(env->p_in[env->p_in_len - AES_BLOCK_SIZE]), aes_res, AES_BLOCK_SIZE);
            env->p_in[0] = 0x03;

            // start execution AES CMAC - Compute T3
            aes_cmac_start(&(env->cmac_env), &(env->res[AES_K2_T_OFFSET]), env->p_in, env->p_in_len);
            finished = false;
            env->state = AES_K2_COMPUTE_T3;
        }
        break;
        case  AES_K2_COMPUTE_T3:
        {
            // save computed T3 = AES-CMAC(T, (T2 || P || 0x03))
            memcpy(&(env->res[AES_K2_T3_OFFSET]), aes_res, AES_BLOCK_SIZE);

            env->res[AES_K2_T1_OFFSET] = (env->res[AES_K2_T1_OFFSET] & 0x7F);

            // Function is over, send back result
            if (env->res_cb != NULL)
            {
                env->res_cb(CO_ERROR_NO_ERROR, env->res, env->cmac_env.aes_env.metainfo);
            }
        }
        break;
        default:
        {
            ASSERT_INFO_FORCE(0, env->state, 0);
        }
        break;
        }
    }

    return finished;
}


/*
 * EXPORTED FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void aes_k2(const uint8_t *n, const uint8_t *p, uint8_t p_len, aes_func_result_cb res_cb, uint32_t metainfo)
{
    struct aes_k2_env *env;
    uint8_t p_in_len = p_len + AES_BLOCK_SIZE + 1;

    // Allocate AES-K2 environment memory
    env = (struct aes_k2_env *) aes_alloc(sizeof(struct aes_k2_env) + p_in_len, (aes_func_continue_cb) aes_k2_continue,
                                          NULL, metainfo);

    // Save input parameters
    env->res_cb       = res_cb;
    env->p_in_len     = p_in_len;
    env->state        = AES_K2_COMPUTE_T;
    // Store a copy of P
    memcpy(&(env->p_in[1]), p, p_len);

    // start execution AES CMAC - First run to compute T
    aes_cmac_start(&(env->cmac_env), aes_k2_salt, n, KEY_LEN);
}

/// @} AES_CMAC
