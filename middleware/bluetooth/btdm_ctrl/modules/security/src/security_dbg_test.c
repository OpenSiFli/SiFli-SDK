/**
 ****************************************************************************************
 *
 * @file security_dbg_test.c
 *
 * @brief Module use to test security
 *
 * Copyright (C) RivieraWaves 2009-2021
 *
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"
#if (RW_DEBUG)
#include "co_bt.h"
#include "aes.h"
#if (ECC_P256_SUPPORT)
    #include "ecc_p256.h"
#endif // (ECC_P256_SUPPORT)
#if (BT_EMB_PRESENT)
    #include "ecc_p192.h"
#endif // (BT_EMB_PRESENT)

#if defined(CFG_SHA_256_SUPPORT)
    #include "sha.h"
#endif // defined(CFG_SHA_256_SUPPORT)

#if defined(CFG_AES_CTR_SUPPORT)
    #include "co_buf.h"
#endif // defined(CFG_AES_CTR_SUPPORT)

#include  "co_utils.h"
#include <string.h>


/*
 * MACROS
 ****************************************************************************************
 */

/*
 * DEFINES
 ****************************************************************************************
 */

/// List of supported tests
enum security_test_type
{
    /// ECC P256 Private Key Generation
    SECURITY_TEST_ECC_P256_PRIVATE_KEY_GEN,
    /// ECC P256 Public Key Generation
    SECURITY_TEST_ECC_P256_PUBLIC_KEY_GEN,
    /// ECC P256 DH Key Generation
    SECURITY_TEST_ECC_P256_DH_KEY_GEN,
    /// ECC P192 Private Key Generation
    SECURITY_TEST_ECC_P192_PRIVATE_KEY_GEN,
    /// ECC P192 Public Key Generation
    SECURITY_TEST_ECC_P192_PUBLIC_KEY_GEN,
    /// ECC P192 DH Key Generation
    SECURITY_TEST_ECC_P192_DH_KEY_GEN,
    /// SHA-256 Hash Compute
    SECURITY_TEST_SHA_256_HASH,
    /// HMAC-SHA-256 Compute
    SECURITY_TEST_SHA_256_HMAC,
    /// Add a key into a SHA-256 based Bloom filter
    SECURITY_TEST_SHA_BLOOM_FILTER_ADD_KEY,
    /// AES-CTR
    SECURITY_TEST_AES_CTR,
    /// AES-CMAC - H6 function
    SECURITY_TEST_H6,
    /// AES-CMAC - H7 function
    SECURITY_TEST_H7,
};

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */
/// Callback executed to return result
typedef void (*security_test_result_cb)(uint16_t result_size, const void *p_result);


/// Test type
typedef struct security_test_info
{
    /// see enum #security_test_type
    uint8_t test_type;
} security_test_info_t;

#if (ECC_P256_SUPPORT)

/// ECC P256 Private Key Generate Result
typedef struct security_test_ecc_p256_private_key_gen_result
{
    /// see enum #security_test_type
    uint8_t test_type;
    /// Private Key
    uint8_t private_key[ECC_256_KEY_SIZE];
} security_test_ecc_p256_private_key_gen_result_t;

/// ECC P256 Public Key Generate Parameters
typedef struct security_test_ecc_p256_public_key_gen_param
{
    /// see enum #security_test_type
    uint8_t test_type;
    /// Private Key
    uint8_t private_key[ECC_256_KEY_SIZE];
} security_test_ecc_p256_public_key_gen_param_t;

/// ECC P256 Public Key Generate Result
typedef struct security_test_ecc_p256_public_key_gen_result
{
    /// see enum #security_test_type
    uint8_t test_type;
    /// Public Key X
    uint8_t public_key_x[ECC_256_KEY_SIZE];
    /// Public Key Y
    uint8_t public_key_y[ECC_256_KEY_SIZE];
} security_test_ecc_p256_public_key_gen_result_t;

/// ECC P256 DH Key Generate Parameters
typedef struct security_test_ecc_p256_dh_key_gen_param
{
    /// see enum #security_test_type
    uint8_t test_type;
    /// Private Key
    uint8_t private_key[ECC_256_KEY_SIZE];
    /// Public Key X
    uint8_t public_key_x[ECC_256_KEY_SIZE];
    /// Public Key Y
    uint8_t public_key_y[ECC_256_KEY_SIZE];
} security_test_ecc_p256_dh_key_gen_param_t;

/// ECC P256 Public Key Generate Result
typedef struct security_test_ecc_p256_dh_key_gen_result
{
    /// see enum #security_test_type
    uint8_t test_type;
    /// DH-Key
    uint8_t dh_key[ECC_256_KEY_SIZE];
} security_test_ecc_p256_dh_key_gen_result_t;

#endif // (ECC_P256_SUPPORT)


#if (BT_EMB_PRESENT)

/// ECC P192 Private Key Generate Result
typedef struct security_test_ecc_p192_private_key_gen_result
{
    /// see enum #security_test_type
    uint8_t test_type;
    /// Private Key
    uint8_t private_key[PRIV_KEY_192_LEN];
} security_test_ecc_p192_private_key_gen_result_t;

/// ECC P192 Public Key Generate Parameters
typedef struct security_test_ecc_p192_public_key_gen_param
{
    /// see enum #security_test_type
    uint8_t test_type;
    /// Private Key
    uint8_t private_key[PRIV_KEY_192_LEN];
} security_test_ecc_p192_public_key_gen_param_t;

/// ECC P192 Public Key Generate Result
typedef struct security_test_ecc_p192_public_key_gen_result
{
    /// see enum #security_test_type
    uint8_t test_type;
    /// Public Key X
    uint8_t public_key_x[PUB_KEY_192_LEN / 2];
    /// Public Key Y
    uint8_t public_key_y[PUB_KEY_192_LEN / 2];
} security_test_ecc_p192_public_key_gen_result_t;

/// ECC P192 DH Key Generate Parameters
typedef struct security_test_ecc_p192_dh_key_gen_param
{
    /// see enum #security_test_type
    uint8_t test_type;
    /// Private Key
    uint8_t private_key[PRIV_KEY_192_LEN];
    /// Public Key X
    uint8_t public_key_x[PUB_KEY_192_LEN / 2];
    /// Public Key Y
    uint8_t public_key_y[PUB_KEY_192_LEN / 2];
} security_test_ecc_p192_dh_key_gen_param_t;

/// ECC P192 Public Key Generate Result
typedef struct security_test_ecc_p192_dh_key_gen_result
{
    /// see enum #security_test_type
    uint8_t test_type;
    /// DH-Key
    uint8_t dh_key[PUB_KEY_192_LEN / 2];
} security_test_ecc_p192_dh_key_gen_result_t;

#endif // (BT_EMB_PRESENT)

#if defined(CFG_SHA_256_SUPPORT)
/// SHA 256 hash Parameters
typedef struct security_test_sha_256_hash_param
{
    /// see enum #security_test_type
    uint8_t test_type;
    /// Message Length
    uint16_t message_len;
    /// Message
    uint8_t message[__ARRAY_EMPTY];
} security_test_sha_256_hash_param_t;

/// HMAC-SHA 256 Parameters
typedef struct security_test_sha_256_hmac_param
{
    /// see enum #security_test_type
    uint8_t test_type;
    /// Key Length
    uint8_t key_len;
    /// Message Length
    uint16_t message_len;
    /// key + Message
    uint8_t key_message[__ARRAY_EMPTY];
} security_test_sha_256_hmac_param_t;

/// SHA 256 hash / HMAC result
typedef struct security_test_sha_256_hash_result
{
    /// see enum #security_test_type
    uint8_t test_type;
    /// Sha 256 hash
    uint8_t hash[SHA_256_HASH_BYTE_SIZE];
} security_test_sha_256_hash_result_t;

#if defined(CFG_SHA_BLOOM_FILTER_SUPPORT)
/// SHA-BLOOM_FILTER add key Parameters
typedef struct security_test_sha_bloom_filter_add_key_param
{
    /// see enum #security_test_type
    uint8_t test_type;
    /// Key to add
    uint8_t key[AES_KEY_LEN];
    /// Size of salt data
    uint8_t salt_size;
    /// Size of bloom filter
    uint16_t filter_size;
    /// salt + filter
    uint8_t salt_and_filter[__ARRAY_EMPTY];
} security_test_sha_bloom_filter_add_key_param_t;

/// SHA-BLOOM_FILTER add key result
typedef struct security_test_sha_bloom_filter_add_key_result
{
    /// see enum #security_test_type
    uint8_t test_type;
    /// Size of bloom filter
    uint16_t filter_size;
    /// Filter output
    uint8_t filter[__ARRAY_EMPTY];
} security_test_sha_bloom_filter_add_key_result_t;
#endif // defined(CFG_SHA_BLOOM_FILTER_SUPPORT)
#endif // defined(CFG_SHA_256_SUPPORT)



#if defined(CFG_AES_CTR_SUPPORT)
/// AES CTR Parameters
typedef struct security_test_aes_ctr_param
{
    /// see enum #security_test_type
    uint8_t test_type;
    /// Secret key
    uint8_t  secret_key[AES_BLOCK_SIZE];
    /// Nonce value
    uint8_t  nonce[AES_BLOCK_SIZE];
    /// Message Length
    uint16_t message_len;
    /// Message (raw or encrypted data)
    uint8_t  message[__ARRAY_EMPTY];
} security_test_aes_ctr_param_t;

/// AES CTR result
typedef struct security_test_aes_ctr_result
{
    /// see enum #security_test_type
    uint8_t test_type;
    /// Message Length
    uint16_t message_len;
    /// Message (encrypted or raw data)
    uint8_t  message[__ARRAY_EMPTY];
} security_test_aes_ctr_result_t;

/// AES CTR buffer metadata
typedef struct security_test_aes_ctr_buf_metadata
{
    co_buf_t *p_buf_in;
    security_test_result_cb cb_result;
} security_test_aes_ctr_buf_metadata_t;

#endif // defined(CFG_AES_CTR_SUPPORT)

#if (BLE_EMB_PRESENT || BLE_HOST_PRESENT)
/// H6 Parameters
typedef struct security_test_h6_param
{
    /// see enum #security_test_type
    uint8_t  test_type;
    /// W
    uint8_t  w[AES_BLOCK_SIZE];
    /// Key ID
    uint8_t  key_id[4];
} security_test_h6_param_t;

/// H6 result
typedef struct security_test_h6_result
{
    /// see enum #security_test_type
    uint8_t  test_type;
    /// AES result
    uint8_t  result[AES_BLOCK_SIZE];
} security_test_h6_result_t;

/// H7 Parameters
typedef struct security_test_h7_param
{
    /// see enum #security_test_type
    uint8_t  test_type;
    /// W
    uint8_t  w[AES_BLOCK_SIZE];
    /// Key ID
    uint8_t  key_id[AES_BLOCK_SIZE];
} security_test_h7_param_t;

/// H7 result
typedef security_test_h6_result_t security_test_h7_result_t;

#endif // defined(CFG_AES_CTR_SUPPORT)
/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

#if (ECC_P256_SUPPORT)
__STATIC void security_test_ecc_p256_public_key_gen_result_cb(security_test_result_cb cb_result, const ecc_p256_result_t *p_res)
{
    security_test_ecc_p256_public_key_gen_result_t result;
    result.test_type = SECURITY_TEST_ECC_P256_PUBLIC_KEY_GEN;
    memcpy(result.public_key_x, p_res->key_res_x, ECC_256_KEY_SIZE);
    memcpy(result.public_key_y, p_res->key_res_y, ECC_256_KEY_SIZE);
    cb_result(sizeof(result), &result);
}


__STATIC void security_test_ecc_p256_dh_key_gen_result_cb(security_test_result_cb cb_result, const ecc_p256_result_t *p_res)
{
    security_test_ecc_p256_dh_key_gen_result_t result;
    result.test_type = SECURITY_TEST_ECC_P256_DH_KEY_GEN;
    memcpy(result.dh_key, p_res->key_res_x, ECC_256_KEY_SIZE);
    cb_result(sizeof(result), &result);
}
#endif // (ECC_P256_SUPPORT)


#if (BT_EMB_PRESENT)
__STATIC void security_test_ecc_p192_public_key_gen_result_cb(security_test_result_cb cb_result, uint8_t status, const ecc_p192_result_t *p_res)
{
    security_test_ecc_p192_public_key_gen_result_t result;
    result.test_type = SECURITY_TEST_ECC_P192_PUBLIC_KEY_GEN;
    memcpy(result.public_key_x, p_res->key_res_x, PUB_KEY_192_LEN / 2);
    memcpy(result.public_key_y, p_res->key_res_y, PUB_KEY_192_LEN / 2);
    cb_result(sizeof(result), &result);
}


__STATIC void security_test_ecc_p192_dh_key_gen_result_cb(security_test_result_cb cb_result, uint8_t status, const ecc_p192_result_t *p_res)
{
    security_test_ecc_p192_dh_key_gen_result_t result;
    result.test_type = SECURITY_TEST_ECC_P192_DH_KEY_GEN;
    memcpy(result.dh_key, p_res->key_res_x, PUB_KEY_192_LEN / 2);
    cb_result(sizeof(result), &result);
}
#endif // (BT_EMB_PRESENT)


#if defined(CFG_AES_CTR_SUPPORT)
__STATIC void security_test_aes_ctr_result_cb(co_buf_t *p_buf_out)
{
    security_test_aes_ctr_buf_metadata_t *p_meta  = (security_test_aes_ctr_buf_metadata_t *) co_buf_metadata(p_buf_out);
    uint16_t data_len = co_buf_data_len(p_buf_out);
    uint8_t *p_data;
    co_buf_head_reserve(p_buf_out, 4); // Put header data
    p_data = co_buf_data(p_buf_out);
    *(p_data + 1) = 0; // padding byte
    *p_data = SECURITY_TEST_AES_CTR;
    co_write16p(p_data + 2, data_len);

    p_meta->cb_result(co_buf_data_len(p_buf_out), p_data);
    co_buf_release(p_meta->p_buf_in);
    co_buf_release(p_buf_out);
}
#endif // defined(CFG_AES_CTR_SUPPORT)


#if (BLE_EMB_PRESENT || BLE_HOST_PRESENT)
__STATIC void security_test_h6_result_cb(uint8_t status, const uint8_t *aes_res, security_test_result_cb cb_result)
{
    security_test_h6_result_t result;
    result.test_type = SECURITY_TEST_H6;
    memcpy(result.result, aes_res, AES_BLOCK_SIZE);
    cb_result(sizeof(result), &result);
}

__STATIC void security_test_h7_result_cb(uint8_t status, const uint8_t *aes_res, security_test_result_cb cb_result)
{
    security_test_h7_result_t result;
    result.test_type = SECURITY_TEST_H7;
    memcpy(result.result, aes_res, AES_BLOCK_SIZE);
    cb_result(sizeof(result), &result);
}
#endif // (BLE_EMB_PRESENT || BLE_HOST_PRESENT)

/*
 * EXPORTED FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

bool security_dbg_test_execute(const security_test_info_t *p_info, security_test_result_cb cb_result)
{
    bool is_test_executed = true;

    switch (p_info->test_type)
    {
#if (ECC_P256_SUPPORT)
    case SECURITY_TEST_ECC_P256_PRIVATE_KEY_GEN:
    {
        security_test_ecc_p256_private_key_gen_result_t result;
        result.test_type = SECURITY_TEST_ECC_P256_PRIVATE_KEY_GEN;
        ecc_p256_gen_new_secret_key(result.private_key);
        cb_result(sizeof(result), &result);
    }
    break;
    case SECURITY_TEST_ECC_P256_PUBLIC_KEY_GEN:
    {
        security_test_ecc_p256_public_key_gen_param_t *p_param = (security_test_ecc_p256_public_key_gen_param_t *) p_info;
        ecc_p256_gen_new_public_key(p_param->private_key, (uint32_t)cb_result, (ecc_p256_result_cb)security_test_ecc_p256_public_key_gen_result_cb);

    }
    break;
    case SECURITY_TEST_ECC_P256_DH_KEY_GEN:
    {
        security_test_ecc_p256_dh_key_gen_param_t *p_param = (security_test_ecc_p256_dh_key_gen_param_t *) p_info;
        is_test_executed = (ecc_p256_gen_dh_key(p_param->private_key, p_param->public_key_x, p_param->public_key_y,
                                                (uint32_t)cb_result, (ecc_p256_result_cb) security_test_ecc_p256_dh_key_gen_result_cb) == CO_ERROR_NO_ERROR);
    }
    break;
#endif // (ECC_P256_SUPPORT)
#if (BT_EMB_PRESENT)
    case SECURITY_TEST_ECC_P192_PRIVATE_KEY_GEN:
    {
        security_test_ecc_p192_private_key_gen_result_t result;
        result.test_type = SECURITY_TEST_ECC_P192_PRIVATE_KEY_GEN;
        ecc_p192_gen_new_secret_key(result.private_key);
        cb_result(sizeof(result), &result);
    }
    break;
    case SECURITY_TEST_ECC_P192_PUBLIC_KEY_GEN:
    {
        security_test_ecc_p192_public_key_gen_param_t *p_param = (security_test_ecc_p192_public_key_gen_param_t *) p_info;
        ecc_p192_gen_new_public_key(p_param->private_key, (uint32_t)cb_result, (ecc_p192_result_cb)security_test_ecc_p192_public_key_gen_result_cb);

    }
    break;
    case SECURITY_TEST_ECC_P192_DH_KEY_GEN:
    {
        security_test_ecc_p192_dh_key_gen_param_t *p_param = (security_test_ecc_p192_dh_key_gen_param_t *) p_info;
        is_test_executed = (ecc_p192_gen_dh_key(p_param->private_key, p_param->public_key_x, p_param->public_key_y,
                                                (uint32_t)cb_result, (ecc_p192_result_cb) security_test_ecc_p192_dh_key_gen_result_cb) == CO_ERROR_NO_ERROR);
    }
    break;
#endif // (BT_EMB_PRESENT)
#if defined(CFG_SHA_256_SUPPORT)
    case SECURITY_TEST_SHA_256_HASH:
    {
        security_test_sha_256_hash_param_t *p_param = (security_test_sha_256_hash_param_t *) p_info;
        security_test_sha_256_hash_result_t result;
        sha_256_hash_t hash;
        result.test_type = SECURITY_TEST_SHA_256_HASH;
        sha_256(p_param->message, p_param->message_len, &hash);
        memcpy(result.hash, &hash, SHA_256_HASH_BYTE_SIZE);
        cb_result(sizeof(result), &result);
    }
    break;

    case SECURITY_TEST_SHA_256_HMAC:
    {
        security_test_sha_256_hmac_param_t *p_param = (security_test_sha_256_hmac_param_t *) p_info;
        security_test_sha_256_hash_result_t result;
        sha_256_hash_t hash;
        result.test_type = SECURITY_TEST_SHA_256_HMAC;
        sha_256_hmac(&(p_param->key_message[0]), p_param->key_len,
                     &(p_param->key_message[p_param->key_len]), p_param->message_len, &hash);
        memcpy(result.hash, &hash, SHA_256_HASH_BYTE_SIZE);
        cb_result(sizeof(result), &result);
    }
    break;

#if defined(CFG_SHA_BLOOM_FILTER_SUPPORT)
    case SECURITY_TEST_SHA_BLOOM_FILTER_ADD_KEY:
    {
        co_buf_t *p_result_buf;
        security_test_sha_bloom_filter_add_key_param_t *p_param = (security_test_sha_bloom_filter_add_key_param_t *) p_info;
        security_test_sha_bloom_filter_add_key_result_t *p_result;

        if (co_buf_alloc(&p_result_buf, 0,
                         sizeof(security_test_sha_bloom_filter_add_key_result_t) + p_param->filter_size, 0) != CO_BUF_ERR_NO_ERROR)
        {
            is_test_executed = false;
            break;
        }

        p_result = (security_test_sha_bloom_filter_add_key_result_t *) co_buf_data(p_result_buf);
        // ensure that padding bytes are initialized
        DBG_MEM_INIT(p_result, sizeof(security_test_sha_bloom_filter_add_key_result_t));
        p_result->test_type = p_param->test_type;
        p_result->filter_size = p_param->filter_size;
        memcpy(p_result->filter, &(p_param->salt_and_filter[p_param->salt_size]), p_result->filter_size);

        sha_bloom_filter_add_key(p_result->filter_size, p_result->filter, p_param->key,
                                 p_param->salt_size, p_param->salt_and_filter);
        cb_result(sizeof(security_test_sha_bloom_filter_add_key_result_t) + p_param->filter_size, p_result);
        co_buf_release(p_result_buf);
    }
    break;
#endif // defined(CFG_SHA_BLOOM_FILTER_SUPPORT)
#endif // #if defined(CFG_SHA_256_SUPPORT)

#if defined(CFG_AES_CTR_SUPPORT)
    case SECURITY_TEST_AES_CTR:
    {
        co_buf_t *p_buf_in;
        co_buf_t *p_buf_out;

        security_test_aes_ctr_param_t *p_param = (security_test_aes_ctr_param_t *) p_info;
        security_test_aes_ctr_buf_metadata_t *p_meta;

        if (co_buf_alloc(&p_buf_in, 0, p_param->message_len, 0) != CO_BUF_ERR_NO_ERROR)
        {
            is_test_executed = false;
            break;
        }

        memcpy(co_buf_data(p_buf_in), p_param->message, p_param->message_len);

        if (co_buf_alloc(&p_buf_out, 4, p_param->message_len, 0) != CO_BUF_ERR_NO_ERROR)
        {
            co_buf_release(p_buf_in);
            is_test_executed = false;
            break;
        }
        p_meta  = (security_test_aes_ctr_buf_metadata_t *) co_buf_metadata(p_buf_out);
        p_meta->p_buf_in  = p_buf_in;
        p_meta->cb_result = cb_result;
        aes_ctr(p_param->secret_key, p_param->nonce, p_param->message_len,
                co_buf_data(p_buf_in), co_buf_data(p_buf_out),
                (aes_ctr_func_result_cb)security_test_aes_ctr_result_cb, (uint32_t)p_buf_out);
    }
    break;
#endif // defined(CFG_AES_CTR_SUPPORT)

#if (BLE_EMB_PRESENT || BLE_HOST_PRESENT)
    case SECURITY_TEST_H6:
    {
        security_test_h6_param_t *p_param = (security_test_h6_param_t *) p_info;
        aes_h6(p_param->w, p_param->key_id, (aes_func_result_cb) security_test_h6_result_cb, (uint32_t)cb_result);
    }
    break;

    case SECURITY_TEST_H7:
    {
        security_test_h7_param_t *p_param = (security_test_h7_param_t *) p_info;
        aes_h7(p_param->w, p_param->key_id, (aes_func_result_cb) security_test_h7_result_cb, (uint32_t)cb_result);
    }
    break;
#endif // (BLE_EMB_PRESENT || BLE_HOST_PRESENT)

    default:
    {
        is_test_executed = false;
    }
    break;
    }

    return (is_test_executed);
}

#endif // (RW_DEBUG)
