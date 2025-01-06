/**
 ****************************************************************************************
 *
 * @file sha_bloom_filter.c
 *
 * @brief Create a bloom filter. A bloom filter is an algorithm that use HASH function (here sha-256)
 *        to generate an index table helping to find possibility of presence or non presence of an element into a data set.
 *        see https://en.wikipedia.org/wiki/Bloom_filter
 *
 * Copyright (C) RivieraWaves 2009-2022
 *
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "sha.h"       // sha hash function
#include "aes.h"       // Size of key
#include "co_math.h"   // co_bit and co_min
#include "co_endian.h" // swap array of byte
#include <string.h>    // memcopy and memset
/*
 * MACROS
 ****************************************************************************************
 */

/*
 * DEFINES
 ****************************************************************************************
 */

#define SHA_BLOOM_FILTER_MAX_SALT_SIZE  (16)

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */


/*
 * EXPORTED FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

void sha_bloom_filter_init(uint16_t filter_size, uint8_t *p_filter)
{
    memset(p_filter, 0, filter_size);
}


/*
 * Let V be concat(K, salt)
 * Hash V using SHA256, obtaining a 32-byte value H = {H0, …, H31}.
 * Divide H into eight 4-byte unsigned integers in big-endian, X = {X0, …, X7}, where X0 = 0xH0H1H2H3.
 * For each Xi:
 *     Let M be Xi modulo the number of bits in the filter, (s * 8).
 *     Get the byte in F at index (M / 8), rounded down.
 *     Within the byte, set the bit at index (M % 8) to 1.
 *     M = Xi % (s * 8)
 *     F[M/8] = F[M/8] | (1 << (M % 8))
 */
void sha_bloom_filter_add_key(uint16_t filter_size, uint8_t *p_filter, const uint8_t *p_key,
                              uint8_t salt_size, const uint8_t *p_salt)
{
    uint8_t cursor;
    sha_256_hash_t hash_v;
    uint16_t m;
    uint16_t filter_bit_size = filter_size * 8;
    // Prepare V: concat(K, salt)
    uint8_t v[AES_KEY_LEN + SHA_BLOOM_FILTER_MAX_SALT_SIZE];
    // Ensure that Salt size do not exceed supported size
    salt_size = co_min(salt_size, SHA_BLOOM_FILTER_MAX_SALT_SIZE);

    co_bswap_copy(&(v[0]), p_key, AES_KEY_LEN);
    memcpy(&(v[AES_KEY_LEN]), p_salt, salt_size);

    // Hash V using SHA256
    sha_256(v, AES_KEY_LEN + salt_size, &hash_v);

    // For each Xi:
    for (cursor = 0; cursor < SHA_256_HASH_WORD_SIZE ; cursor++)
    {
        // Let M be Xi modulo the number of bits in the filter, (s * 8).
        m = (co_ntohl(hash_v.value[cursor]) % filter_bit_size); // Turn hash word in Little endian
        // Get the byte in F at index (M / 8), rounded down.
        // Within the byte, set the bit at index (M % 8) to 1.
        p_filter[m >> 3] |= CO_BIT(m & 0x7);
    }
}

/// @} sha_bloom
