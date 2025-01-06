/**
 ****************************************************************************************
 * *
 * @file aes.c
 *
 * @brief Definition file for AES crypto module functions
 *
 * Copyright (C) RivieraWaves 2017-2018
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

#include "aes.h"            // AES function declaration
#include "aes_int.h"        // AES Internal defines
#include "ke_mem.h"         // Used to allocate memory environment
#include "co_list.h"        // List of AES execution to perform
#include "co_math.h"        // For rand functions

#include "rwip.h"           // For HW AES accelerator

#include <string.h>         // for memcpy function
#include "arch.h"
/*
 * DEFINES
 ****************************************************************************************
 */


/*
 * TYPES DEFINITION
 *****************************************************************************************
 */

/// AES environment structure
struct aes_env_tag
{
    /// List of AES function to execute
    struct co_list  queue;
    /// AES under execution
    bool aes_ongoing;
};

/// Structure definition used for AES with a cache
struct aes_cache_env
{
    /// AES Environment structure
    struct aes_func_env aes_env;
    /// Cached Key
    uint8_t key_cache[KEY_LEN];
    /// Cached value
    uint8_t val_cache[KEY_LEN];
};

/*
 * GLOBAL VARIABLE DEFINITION
 *****************************************************************************************
 */
/// AES queue environment
struct aes_env_tag aes_env;

/*
 * MODULE INTERNAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */


/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */


/*
 * EXPORTED FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void aes_init(bool is_reset)
{
    if (is_reset)
    {
        // remove all element present in queue
        while (!co_list_is_empty(&(aes_env.queue)))
        {
            // remove allocated environment
            ke_free(co_list_pop_front(&(aes_env.queue)));
        }
    }

    // initialize queue
    co_list_init(&(aes_env.queue));
    aes_env.aes_ongoing = false;
}

struct aes_func_env *aes_alloc(uint16_t size, aes_func_continue_cb aes_continue_cb, aes_func_result_cb res_cb,
                               uint32_t metainfo)
{
    struct aes_func_env *aes_env;

    // allocate AES memory in non-retention to execute it as soon as possible
    aes_env = ke_malloc_system(size, KE_MEM_NON_RETENTION);

    // initialize AES environment
    aes_env->aes_continue_cb = aes_continue_cb;
    aes_env->aes_res_cb      = res_cb;
    aes_env->metainfo        = metainfo;

    return aes_env;
}

void aes_start(struct aes_func_env *env, const uint8_t *key, const uint8_t *val, bool cipher)
{
    // put function environment at end of list.
    co_list_push_back(&(aes_env.queue), &(env->hdr));

    // store parameter information
    env->key = key;
    env->val = val;

    // AES encryption can be immediately performed
    if (!aes_env.aes_ongoing)
    {
        aes_env.aes_ongoing = true;
        rwip_aes(env->key, env->val, cipher);
    }
}

void aes_encrypt(const uint8_t *key, const uint8_t *val, bool copy, aes_func_result_cb res_cb, uint32_t metainfo)
{
    if (!copy)
    {
        // allocate environment for the AES execution
        struct aes_func_env *env = aes_alloc(sizeof(struct aes_func_env), NULL, res_cb, metainfo);

        // Start AES execution
        aes_start(env, key, val, true);
    }
    else
    {
        // allocate environment for the AES execution
        struct aes_cache_env *env = (struct aes_cache_env *) aes_alloc(sizeof(struct aes_cache_env), NULL, res_cb, metainfo);

        // Copy keys and val
        memcpy(env->key_cache, key, KEY_LEN);
        memcpy(env->val_cache, val, KEY_LEN);

        // Start AES execution
        aes_start(&(env->aes_env), env->key_cache, env->val_cache, true);
    }
}


void aes_decrypt(const uint8_t *key, const uint8_t *val, bool copy, aes_func_result_cb res_cb, uint32_t metainfo)
{
    if (!copy)
    {
        // allocate environment for the AES execution
        struct aes_func_env *env = aes_alloc(sizeof(struct aes_func_env), NULL, res_cb, metainfo);

        // Start AES execution
        aes_start(env, key, val, false);
    }
    else
    {
        // allocate environment for the AES execution
        struct aes_cache_env *env = (struct aes_cache_env *) aes_alloc(sizeof(struct aes_cache_env), NULL, res_cb, metainfo);

        // Copy keys and val
        memcpy(env->key_cache, key, KEY_LEN);
        memcpy(env->val_cache, val, KEY_LEN);

        // Start AES execution
        aes_start(&(env->aes_env), env->key_cache, env->val_cache, false);
    }
}

void aes_rand(aes_func_result_cb res_cb, uint32_t metainfo)
{
    /// Random number that requires more entropy
    uint32_t randn[KEY_LEN / 2];

    /// generate key and value
    for (uint8_t i = 0 ; i < (KEY_LEN / 2) ; i++)
    {
        randn[i] = co_rand_word();
    }

    /// perform encryption
    aes_encrypt((uint8_t *) & (randn[0]), (uint8_t *) & (randn[4]), true, res_cb, metainfo);
}

void aes_result_handler(uint8_t status, uint8_t *result)
{
    // extract first element of the list that contains
    struct aes_func_env *env = (struct aes_func_env *)      co_list_pop_front(&(aes_env.queue));
    struct aes_func_env *env_next = (struct aes_func_env *) co_list_pick(&(aes_env.queue));
    aes_env.aes_ongoing      = false;

    // Prepare new AES Run if requested
    if (env_next != NULL)
    {
        aes_env.aes_ongoing = true;
        rwip_aes(env_next->key, env_next->val, true);
    }

    // Check that AES result has a requester
    if (env)
    {
        bool finished = true;

        // check status of current result
        if (status == CO_ERROR_NO_ERROR)
        {
            // continue function computation
            if (env->aes_continue_cb != NULL)
            {
                finished = env->aes_continue_cb(env, result);
            }
        }

        // if function execution is over
        if (finished)
        {
            // Inform requester of end of AES based algorithm
            if (env->aes_res_cb != NULL)
            {
                env->aes_res_cb(status, result, env->metainfo);
            }

            // remove function environment
            ke_free(env);
        }
    }
}

void aes_xor_128(uint8_t *result, const uint8_t *a, const uint8_t *b, uint8_t size)
{
    int8_t i;

    for (i = (size - 1); i >= 0 ; i--)
    {
        result[i] = a[i] ^ b[i];
    }
}

void aes_shift_left_128(const uint8_t *input, uint8_t *output)
{
    uint8_t i;

    for (i = AES_BLOCK_SIZE - 1 ; i >= 1; i--)
    {
        output[i] = (input[i] << 1) | (input[i - 1] >> 7);
    }

    output[0] = (input[0] << 1);
}

/// @} AES
