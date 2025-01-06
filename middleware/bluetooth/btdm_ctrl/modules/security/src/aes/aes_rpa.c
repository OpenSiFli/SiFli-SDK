/**
 ****************************************************************************************
 * *
 * @file aes_rpa.c
 *
 * @brief Definition file for Confirm value generation function c1 for LE Legacy Pairing
 *
 * Copyright (C) RivieraWaves 2017-2018
 *
 * Algorithm:
 *
 *    irk[i] is 128 bits
 *    addr is 48 bits, composed of prand (24 bits) and hash (24 bits)
 *       addr = prand || hash
 *
 *    prand' is 128 bits
 *       prand' = padding || prand
 *
 *    The algorithm finds the index i, where hash = AES-128(irk[i], prand')
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
#if (BLE_EMB_PRESENT || BLE_HOST_PRESENT)

#include "arch.h"     // For architecture platform defines
#include "aes_int.h"  // AES internals
#include <string.h>   // for memcpy function
#include "co_utils.h" // for Bit field access
#include "co_math.h"  // for Maths operation
#include "co_endian.h"// For endianess requiements

/*
 * DEFINES
 ****************************************************************************************
 */

/// Generation algorithm step (RAND/HASH)
enum aes_addr_gen_step
{
    AES_ADDR_GEN_RAND,
    AES_ADDR_GEN_HASH,
};


/*
 * TYPES DEFINITION
 *****************************************************************************************
 */

/// Structure definition of the AES function for Address generation algorithm
struct aes_gen_rand_addr_env
{
    /// AES Environment structure
    struct aes_func_env  aes_env;
    /// Pointer to key used for generation
    const uint8_t       *key;
    /// Padded random number
    uint8_t              prand_bis[ENC_DATA_LEN];
    /// Generation algorithm step (RAND/HASH)
    uint8_t              step;
    /// Address type
    uint8_t              addr_type;
};

/// Structure definition of the AES RPA resolution algorithm
struct aes_rpa_resolve_env
{
    /// AES Environment structure
    struct aes_func_env  aes_env;
    /// Callback indicating the result of address resolution
    aes_rpa_func_result_cb res_cb;
    /// BD address
    uint8_t addr[KEY_LEN];
    /// Padded random number
    uint8_t prand_bis[ENC_DATA_LEN];
    /// Current index
    uint8_t index;
    /// Number of IRKs
    uint8_t nb_irk;
    /// Pointer to RAL
    aes_key_t irk[__ARRAY_EMPTY];
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

__STATIC bool aes_addr_gen_continue(struct aes_gen_rand_addr_env *env, uint8_t *aes_res)
{
    bool finished = false;

    switch (env->step)
    {
    case AES_ADDR_GEN_RAND:
    {
        memset(&env->prand_bis[0], 0, ENC_DATA_LEN);
        memcpy(&env->prand_bis[0], aes_res, BD_ADDR_PRAND_LEN);

        // Set the address type as resolvable private address (2 msbs of prand)
        env->prand_bis[BD_ADDR_PRAND_LEN - 1] &= 0x3F;
        env->prand_bis[BD_ADDR_PRAND_LEN - 1] |= env->addr_type;

        /*
         * Encrypt PRAND with local IRK, to obtain HASH
         */
        aes_start(&(env->aes_env), env->key, &env->prand_bis[0], true);
        env->step = AES_ADDR_GEN_HASH;
    }
    break;

    case AES_ADDR_GEN_HASH:
    {
        /*
         * Construct RPA, from PRAND and HASH, address provided in LSB->MSB
         *      -----------------------------------------------------------------------------------------
         *      | hash[0:(HASH_LEN-1)] | prand[(HASH_LEN:(BD_ADDR_LEN-1)] |
         *      -----------------------------------------------------------------------------------------
         */
        memcpy(aes_res + BD_ADDR_HASH_LEN, &env->prand_bis[0], BD_ADDR_PRAND_LEN);

        finished = true;
    }
    break;

    default:
    {
        ASSERT_INFO_FORCE(0, env->step, env->aes_env.metainfo);
    }
    break;
    }

    return finished;
}

__STATIC bool aes_rpa_resolve_continue(struct aes_rpa_resolve_env *env, uint8_t *aes_res)
{
    bool finished = false;

    do
    {
        // Compare hash values
        if (memcmp(&env->addr[0], aes_res, BD_ADDR_HASH_LEN))
        {
            // The IRK did not match, jump to next valid IRK
            uint8_t position = env->index + 1;
            for (; position < env->nb_irk; position++)
            {
                if (memcmp(env->irk[position].value, co_null_key, KEY_LEN))
                    break;
            }
            env->index = position;

            // Check that IRKs have not been all used
            if (env->index < env->nb_irk)
            {
                // Start AES-128 on next IRK
                aes_start(&(env->aes_env), &env->irk[env->index].value[0], &env->prand_bis[0], true);
                break;
            }
        }

        // Indicate process completion
        if (env->res_cb != NULL)
        {
            env->res_cb((env->index < env->nb_irk) ? CO_ERROR_NO_ERROR : CO_ERROR_UNSPECIFIED_ERROR,
                        env->index, env->aes_env.metainfo, env->addr, &env->irk[env->index]);
        }

        finished = true;

    }
    while (0);

    return finished;
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void aes_gen_rand_addr(const uint8_t *key, uint8_t addr_type, aes_func_result_cb res_cb, uint32_t metainfo)
{
    struct aes_gen_rand_addr_env *env;

    // Allocate AES RPA environment memory
    env = (struct aes_gen_rand_addr_env *) aes_alloc(sizeof(struct aes_gen_rand_addr_env), (aes_func_continue_cb) aes_addr_gen_continue, res_cb, metainfo);

    // prepare parameters
    env->key       = key;
    env->addr_type = addr_type;

    /*
     * Start by generating a 3-bytes random number PRAND
     */

    // Generate random number
    for (uint8_t i = 0 ; i < KEY_LEN; i++)
    {
        env->prand_bis[i] = co_rand_byte();
    }

    aes_start(&(env->aes_env), env->key, &env->prand_bis[0], true);
    env->step = AES_ADDR_GEN_RAND;

}

void aes_rpa_gen(const aes_key_t *irk, aes_func_result_cb res_cb, uint32_t metainfo)
{
    aes_gen_rand_addr(irk->value, BD_ADDR_RSLV, res_cb, metainfo);
}

void aes_rpa_resolve(uint8_t nb_irk, const aes_key_t *irk, const uint8_t *p_addr, aes_rpa_func_result_cb res_cb, uint32_t metainfo)
{
    uint8_t position = 0;
    struct aes_rpa_resolve_env *env;

    // Allocate AES RPA environment memory
    env = (struct aes_rpa_resolve_env *) aes_alloc(sizeof(struct aes_rpa_resolve_env) + sizeof(aes_key_t) * nb_irk,
            (aes_func_continue_cb) aes_rpa_resolve_continue, NULL, metainfo);

    // prepare parameters
    env->res_cb = res_cb;
    memcpy(env->irk, irk, nb_irk * sizeof(aes_key_t));
    memcpy(&env->addr, p_addr, BD_ADDR_LEN);
    env->nb_irk = nb_irk;

    /*
     * Address provided in LSB->MSB
     *      -----------------------------------------------------------------------------------------
     *      | hash[0:(HASH_LEN-1)] | prand[(HASH_LEN:(BD_ADDR_LEN-1)] |
     *      -----------------------------------------------------------------------------------------
     *
     * prand_bis is LSB->MSB
     *      --------------------------------------------------------------------------------------
     *      | prand[0:(PRAND_LEN-1)] | 0[(PRAND_LEN):(ENC_LEN-1)]  |
     *      --------------------------------------------------------------------------------------
     */
    // Clear prand_bis
    memset(&env->prand_bis[0], 0x00, ENC_DATA_LEN);

    // Copy prand value in prand_bis
    memcpy(&env->prand_bis[0], &p_addr[BD_ADDR_HASH_LEN], BD_ADDR_PRAND_LEN);

    // Find the first valid peer IRK
    for (position = 0; position < (env->nb_irk - 1); position++)
    {
        if (memcmp(env->irk[position].value, co_null_key, KEY_LEN))
            break;
    }

    // Start AES-128 on first valid IRK
    env->index = position;
    aes_start(&(env->aes_env), &env->irk[env->index].value[0], &env->prand_bis[0], true);
}

#endif // (BLE_EMB_PRESENT || BLE_HOST_PRESENT)
/// @} AES_RPA
