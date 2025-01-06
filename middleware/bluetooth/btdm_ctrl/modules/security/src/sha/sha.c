/**
 ****************************************************************************************
 *
 * @file sha.c
 *
 * @brief SHA-2 (Secure Hash Algorithm) function definition
 *
 * Copyright (C) RivieraWaves 2009-2021
 *
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "sha.h"
#include "arch.h"
#include "co_math.h"   // for co_divide
#include "co_endian.h" // for co_bswap32
#include "string.h"    // for memcpy
#include "ke_mem.h"

/*
 * MACROS
 ****************************************************************************************
 */

/*
 * DEFINES
 ****************************************************************************************
 */

#define SHA_256_HMAC_IPAD_WORD  (0x36363636)
#define SHA_256_HMAC_OPAD_WORD  (0x5C5C5C5C)
#define SHA_256_CHUNK_BYTE_SIZE (64)

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

#if 0
__INLINE void PRINT_128BIT_KEY(const char *name, const void *p_val)
{
    const uint8_t *val = (uint8_t *) p_val;
    DUMP_STR(0, "%s() = 0x%02X%02X%02X%02X %02X%02X%02X%02X %02X%02X%02X%02X %02X%02X%02X%02X", name,
             val[0], val[1], val[2], val[3], val[4], val[5], val[6], val[7], val[8], val[9], val[10], val[11], val[12], val[13], val[14], val[15]);
}

__INLINE void PRINT_256BIT_KEY(const char *name, const void *p_val)
{
    const uint8_t *val = (uint8_t *) p_val;
    DUMP_STR(0, "%s() = 0x%02X%02X%02X%02X %02X%02X%02X%02X %02X%02X%02X%02X %02X%02X%02X%02X %02X%02X%02X%02X %02X%02X%02X%02X %02X%02X%02X%02X %02X%02X%02X%02X", name,
             val[0], val[1], val[2], val[3], val[4], val[5], val[6], val[7], val[8], val[9], val[10], val[11], val[12], val[13], val[14], val[15],
             val[16], val[17], val[18], val[19], val[20], val[21], val[22], val[23], val[24], val[25], val[26], val[27], val[28], val[29], val[30], val[31]);
}
#else
#define PRINT_128BIT_KEY(...)
#define PRINT_256BIT_KEY(...)
#endif

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// SHA256 constant K value
__STATIC const uint32_t sha_256_k[] =
{
    0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
    0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
    0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
    0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
    0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
    0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
    0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
    0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2
};


/// SHA Initial Hash
__STATIC const sha_256_hash_t sha_256_hash_init_value =
{
    .value = { 0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a, 0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19 },
};


/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @brief This function is used for SHA-256 calculation
 *
 * @param[in] x       coordinate
 * @param[in] y       coordinate
 * @param[in] z       coordinate
 *
 * @return uint32_t
 ****************************************************************************************
 */
__INLINE uint32_t sha_256_ch(uint32_t x, uint32_t y, uint32_t z)
{
    return ((x & y) ^ ((~x)&z));
}

/*
 ****************************************************************************************
 * @brief This function is used for SHA-256 calculation
 *
 * @param[in] x       coordinate
 * @param[in] y       coordinate
 * @param[in] z       coordinate
 *
 * @return uint32_t
 *
 ****************************************************************************************
 */
__INLINE uint32_t sha_256_maj(uint32_t x, uint32_t y, uint32_t z)
{
    return ((x & y) ^ (x & z) ^ (y & z));
}


/*
 ****************************************************************************************
 * @brief This function is used for SHA-256 calculation
 *
 * @param[in] x     coordinate
 *
 * @return uint32_t
 *
 ****************************************************************************************
 */
__INLINE uint32_t sha_256_sigma0(uint32_t x)
{
    uint32_t result = 0;
    result = ((x >> 7) | (x << (32 - 7)));
    result ^= ((x >> 18) | (x << (32 - 18)));
    result ^= (x >> 3);
    return result;
}

/*
 ****************************************************************************************
 * @brief This function is used for SHA-256 calculation
 *
 * @param[in] x    coordinate
 *
 * @return uint32_t
 *
 ****************************************************************************************
 */
__INLINE uint32_t sha_256_sigma1(uint32_t x)
{
    uint32_t result = 0;
    result = ((x >> 17) | (x << (32 - 17)));
    result ^= ((x >> 19) | (x << (32 - 19)));
    result ^= (x >> 10);
    return result;
}

/*
 ****************************************************************************************
 * @brief This function is used for SHA-256 calculation
 *
 * @param[in] x     coordinate
 *
 * @return uint32_t
 *
 ****************************************************************************************
 */
__INLINE uint32_t sha_256_sigma0_1(uint32_t x)
{
    uint32_t result = 0;
    result = ((x >> 2) | (x << (32 - 2)));
    result ^= ((x >> 13) | (x << (32 - 13)));
    result ^= ((x >> 22) | (x << (32 - 22)));
    return result;
}

/*
 ****************************************************************************************
 * @brief This function is used for SHA-256 calculation
 *
 * @param[in] x     coordinate
 *
 * @return uint32_t
 *
 ****************************************************************************************
 */
__INLINE uint32_t sha_256_sigma1_1(uint32_t x)
{
    uint32_t result = 0;
    result = ((x >> 6) | (x << (32 - 6)));
    result ^= ((x >> 11) | (x << (32 - 11)));
    result ^= ((x >> 25) | (x << (32 - 25)));
    return result;
}

/// Initialize Hash
__INLINE void sha_256_hash_init(sha_256_hash_t *p_hash)
{
    memcpy(p_hash, &sha_256_hash_init_value, sizeof(sha_256_hash_init_value));
}

/// Swap Hash Result
__INLINE void sha_256_hash_swap(sha_256_hash_t *p_hash)
{
    uint8_t cursor;
    for (cursor = 0; cursor < SHA_256_HASH_WORD_SIZE; cursor++)
    {
        p_hash->value[cursor] = co_bswap32(p_hash->value[cursor]);
    }
}

/// Prepare data in chunk
__STATIC void sha_256_prepare_chunk(uint32_t *W, uint8_t copy_word_nb,  const uint8_t **pp_message, uint16_t remain_message_len,
                                    uint32_t msg_len_in_bits, bool last, bool *p_do_put_final_bit)
{
    const uint8_t *p_message = *pp_message;
    uint8_t  t;
    for (t = 0 ; t < copy_word_nb ; t++)
    {
        W[t] = ((p_message[0] << 24) | (p_message[1] << 16) | (p_message[2] << 8) | p_message[3]);
        p_message += 4;
    }

    /* ********************************************************************************
     * Padding the message. A 1 is added after the last bit of the message and the
     * remainder of the block is padded with zeros.
     * ********************************************************************************/
    if ((remain_message_len < 4) && *p_do_put_final_bit && (t < 16))
    {
        uint32_t M_last = 0x80;
        p_message += (remain_message_len - 1);
        while (remain_message_len > 0)
        {
            M_last = (M_last << 8) | *(p_message--);
            remain_message_len -= 1;
        }
        W[t] = co_bswap32(M_last);
        *p_do_put_final_bit = false;
        t += 1;
    }

    // Put Padding value
    for (; t < 16 ; t++) W[t] = 0;

    /* ********************************************************************************
     * The length of the message is then inserted in the last 64 bits of the block.
     * Actually we only need to do this for the last chunk of the message
     * ********************************************************************************/
    if (last)
    {
        W[15] = msg_len_in_bits;
    }

    *pp_message = p_message;
}

/// Extend W to sixty-four 32-bit words and compute hash for chunk
__STATIC void sha_256_extends_w_and_compute_hash_for_chunk(uint32_t *W, uint32_t *p_hash_data)
{
    uint32_t a, b, c, d, e, f, g, h;
    uint8_t  t;

    //Extend the sixteen 32-bit words into sixty-four 32-bit words
    for (t = 16; t < 64; t++)
    {
        W[t] = sha_256_sigma1(W[t - 2]) +  W[t - 7] +  sha_256_sigma0(W[t - 15]) + W[t - 16];
    }

    PRINT_256BIT_KEY("sha W[0]", W);
    PRINT_256BIT_KEY("sha W[8]", W + 8);

    // make a copy of current hash value
    a = p_hash_data[0];
    b = p_hash_data[1];
    c = p_hash_data[2];
    d = p_hash_data[3];
    e = p_hash_data[4];
    f = p_hash_data[5];
    g = p_hash_data[6];
    h = p_hash_data[7];
    // Apply the sha-256 constant to the chunk
    for (t = 0; t < 64; t++)
    {
        uint32_t T1, T2;
        T1 = h + sha_256_sigma1_1(e) + sha_256_ch(e, f, g) + sha_256_k[t] + W[t];
        T2 = sha_256_sigma0_1(a) + sha_256_maj(a, b, c);
        h = g;
        g = f;
        f = e;
        e = d + T1;
        d = c;
        c = b;
        b = a;
        a = T1 + T2;
    }

    // increment hash value
    p_hash_data[0] += a;
    p_hash_data[1] += b;
    p_hash_data[2] += c;
    p_hash_data[3] += d;
    p_hash_data[4] += e;
    p_hash_data[5] += f;
    p_hash_data[6] += g;
    p_hash_data[7] += h;
}

/// Execute SHA on chunks
__STATIC void sha_256_continue_compute(const uint8_t *p_message, uint16_t message_len, uint32_t msg_len_in_bits,
                                       bool last, sha_256_hash_t *p_hash)
{
    // Compute number of chunk with end character and size information
    // 8 bytes required for 64-bit MSB length and 1 byte for final message bit
    uint16_t num_chunks = CO_DIVIDE_CEIL(message_len + (last ? 9 : 0), 64);
    uint16_t remain_message_len = message_len;
    bool do_put_final_bit = last;

    // Do full chuncks or contains end of message
    ASSERT_ERR(last || message_len % 64 == 0);

    // Break the message into N * 512 bit blocks. ( 16 * 32bit words)
    uint32_t *W = ke_malloc_user(64 * 4, KE_MEM_NON_RETENTION);
    while (num_chunks > 0)
    {
        uint8_t copy_word_nb = co_min(remain_message_len >> 2, 16);
        remain_message_len -= copy_word_nb << 2;

        sha_256_prepare_chunk(W, copy_word_nb,  &p_message, remain_message_len, msg_len_in_bits,
                              (num_chunks == 1) && last, &do_put_final_bit);

        sha_256_extends_w_and_compute_hash_for_chunk(W, p_hash->value);

        // go to next chunk
        num_chunks--;
    }
    ke_free(W);
}

/// Compute hash for HMAC Key chunk
__STATIC void sha_256_hmac_compute_chunk_for_key(const uint8_t *p_key, uint8_t key_len, uint32_t pad_word, sha_256_hash_t *p_hash)
{
    uint32_t *W = ke_malloc_user(64 * 4, KE_MEM_NON_RETENTION);
    uint8_t  copy_word_nb = key_len >> 2;
    uint8_t  t;

    key_len -= copy_word_nb << 2;
    for (t = 0 ; t < copy_word_nb ; t++)
    {
        W[t] = ((p_key[0] << 24) | (p_key[1] << 16) | (p_key[2] << 8) | p_key[3]) ^ pad_word;
        p_key += 4;
    }

    if (key_len < 4)
    {
        uint32_t M = 0;
        p_key += (key_len - 1);
        while (key_len > 0)
        {
            M = (M << 8) | *(p_key--);
            key_len -= 1;
        }
        W[t] = co_bswap32(M) ^ pad_word;
        t += 1;
    }

    // Put Padding value
    for (; t < 16 ; t++) W[t] = pad_word;

    sha_256_extends_w_and_compute_hash_for_chunk(W, p_hash->value);
    ke_free(W);
}


/// Compute hash for HMAC Key chunk
__STATIC void sha_256_do_hmac_round(const uint8_t *p_key, uint8_t key_len, const uint8_t *p_message, uint16_t message_len,
                                    uint32_t pad_word, sha_256_hash_t *p_hash)
{
    // Initialize the hash
    sha_256_hash_init(p_hash);
    // compute first chunk: Key
    sha_256_hmac_compute_chunk_for_key(p_key, key_len, pad_word, p_hash);
    // compute folowing chunk for message
    sha_256_continue_compute(p_message, message_len, 8 * (SHA_256_CHUNK_BYTE_SIZE + message_len), true, p_hash);
    // Swap result
    sha_256_hash_swap(p_hash);
}

/*
 * EXPORTED FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */
void sha_256(const uint8_t *p_message, uint16_t message_len, sha_256_hash_t *p_hash)
{
    // Initialize the hash
    sha_256_hash_init(p_hash);

    // Compute sha for complete message
    sha_256_continue_compute(p_message, message_len, message_len * 8, true, p_hash);

    // Swap result
    sha_256_hash_swap(p_hash);
}

void sha_256_hmac(const uint8_t *p_key, uint8_t key_len, const uint8_t *p_message, uint16_t message_len, sha_256_hash_t *p_hash)
{
    sha_256_hash_t intermediate_hash;

    // ----------------------------- First Round:  sha256(key xor ipad || msg)
    sha_256_do_hmac_round(p_key, key_len, p_message, message_len, SHA_256_HMAC_IPAD_WORD, &intermediate_hash);

    // ----------------------------- Second Round  sha256(key xor opad || sha256(key xor ipad || msg))
    sha_256_do_hmac_round(p_key, key_len, (uint8_t *)&intermediate_hash, SHA_256_HASH_BYTE_SIZE, SHA_256_HMAC_OPAD_WORD, p_hash);
}

