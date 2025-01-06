/**
 ****************************************************************************************
 * *
 * @file aes_c1.c
 *
 * @brief Definition file for Confirm value generation function c1 for LE Legacy Pairing
 *
 * Copyright (C) RivieraWaves 2017-2018
 *
 * Algorithm:
 *
 *    k is 128 bits
 *    r is 128 bits
 *    pres is 56 bits
 *    preq is 56 bits
 *    iat is 1 bit
 *    ia is 48 bits
 *    rat is 1 bit
 *    ra is 48 bits
 *    padding is 32 bits or 0
 *
 *    P1 = pres || preq || rat’ || iat’
 *    P2 = padding || ia || ra
 *
 *    c1(k, r, pres, preq, iat, ia, rat, ra) = AES-128(K, AES-128(K, K XOR P1) XOR P2)
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
#if (BLE_HOST_PRESENT)

#include "arch.h"     // For architecture platform defines
#include "aes_int.h"  // AES internals
#include <string.h>   // for memcpy function
#include "co_utils.h" // for Bit field access
#include "co_endian.h"// For endianess requiements

/*
 * DEFINES
 ****************************************************************************************
 */


/*
 * TYPES DEFINITION
 *****************************************************************************************
 */

/// Structure definition of the AES C1 algorithm
struct aes_c1_env
{
    /// AES Environment structure
    struct aes_func_env  aes_env;
    /// (R XOR P1 value) or / (e(R XOR P1) XOR P2)
    uint8_t              xor[KEY_LEN];
    /// P2 value
    uint8_t              p2[KEY_LEN];
    /// used to know if C1 computed
    bool                 c1_computed;
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

__STATIC bool aes_c1_continue(struct aes_c1_env *env, uint8_t *aes_res)
{
    bool finished = false;

    if (!env->c1_computed)
    {
        aes_xor_128(env->xor, aes_res, env->p2, KEY_LEN);
        env->c1_computed = true;

        // C_1 := AES-128(K, VAL_1 XOR P2)
        aes_start(&(env->aes_env), env->aes_env.key, env->xor, true);
    }
    else
    {
        // mark AES-C1 over
        finished = true;
    }

    return finished;
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
void aes_c1(const uint8_t *k, const uint8_t *r, const uint8_t *p1, const uint8_t *p2,
            aes_func_result_cb res_cb, uint32_t metainfo)
{
    struct aes_c1_env *env;

    // Allocate AES CMAC environment memory
    env = (struct aes_c1_env *) aes_alloc(sizeof(struct aes_c1_env),
                                          (aes_func_continue_cb) aes_c1_continue, res_cb, metainfo);

    // prepare parameters
    aes_xor_128(env->xor, r, p1, KEY_LEN);
    memcpy(env->p2, p2, KEY_LEN);
    env->c1_computed = false;

    // VAL_1 := AES-128(K, R XOR P1)
    aes_start(&(env->aes_env), k, env->xor, true);
}

#endif // (BLE_HOST_PRESENT)
/// @} AES_CCM
