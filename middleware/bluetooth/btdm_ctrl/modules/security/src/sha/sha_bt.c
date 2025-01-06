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


/*
 * MACROS
 ****************************************************************************************
 */

/*
 * DEFINES
 ****************************************************************************************
 */

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
 * EXPORTED FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

void sha_256_f1(bool p_256, const uint8_t *U, const uint8_t *V, const uint8_t *X, const uint8_t *Z, uint8_t *OutPut)
{
    sha_256_hash_t hash;
    uint8_t X_inv[16];
    uint8_t Message[65];
    uint8_t cursor = 0;
    uint8_t ecdh_len = p_256 ? 32 : 24; // p192 or p256

    // U is 256/192 bits = 32/24 Bytes = 8/6 Words
    // V is 256/192 bits = 32/24 Bytes = 8/6 Words
    // X is 128 bits     = 16 Bytes    = 4 Words  -- The KEY
    // Z is 8 bits       =  1 Byte

    co_bswap_copy(X_inv,               X, 16);
    co_bswap_copy(&(Message[cursor]),  U, ecdh_len);
    cursor += ecdh_len;
    co_bswap_copy(&(Message[cursor]),  V, ecdh_len);
    cursor += ecdh_len;
    Message[cursor] = Z[0];
    cursor += 1;

    sha_256_hmac(X_inv, 16, Message, cursor, &hash);

    // 128-bit result
    co_bswap_copy(OutPut, (uint8_t *) &hash, 16);

    PRINT_128BIT_KEY("f1", OutPut);
}

void sha_256_g(bool p_256, const uint8_t *U, const uint8_t *V, const uint8_t *X, const uint8_t *Y, uint32_t *OutPut)
{
    uint8_t Message[96];
    sha_256_hash_t hash;
    uint8_t cursor = 0;
    uint8_t ecdh_len = p_256 ? 32 : 24; // p192 or p256
    // U is 256/192 bits = 32/24 Bytes = 8/6 Words
    // V is 256/192 bits = 32/24 Bytes = 8/6 Words
    // X is 128 bits     = 16 Bytes    = 4 Words
    // Y is 128 bits     = 16 Bytes    = 4 Words
    //
    // g(U, V, X, Y) = SHA-256 (U || V || X || Y) / 2e32

    co_bswap_copy(&Message[cursor], U, ecdh_len);
    cursor += ecdh_len;
    co_bswap_copy(&Message[cursor], V, ecdh_len);
    cursor += ecdh_len;
    co_bswap_copy(&Message[cursor], X, 16);
    cursor += 16;
    co_bswap_copy(&Message[cursor], Y, 16);
    cursor += 16;

    sha_256(Message, cursor, &hash);

    *OutPut = co_bswap32(hash.value[7]);
    *OutPut = CO_MOD(*OutPut, 0xF4240);
}

void sha_256_f2(bool p_256, const uint8_t *W, const uint8_t *N1, const uint8_t *N2, const uint8_t *KeyId,
                const uint8_t *A1, const uint8_t *A2, uint8_t *OutPut)
{
    sha_256_hash_t hash;
    uint8_t cursor = 0;
    uint8_t W_inv[32];
    uint8_t Message[48];
    uint8_t ecdh_len = p_256 ? 32 : 24; // p192 or p256

    // W  is 256/192 bits = 32/24 Bytes = 8/6 Words
    // N1 is 128 bits = 16 Bytes = 4 Words
    // N2 is 128 bits = 16 Bytes = 4 Words
    // keyID is 32 bits = 4 Bytes = 1 Word
    // A1 is 48 bits = 6 Bytes
    // A2 is 48 bits = 6 Bytes
    co_bswap_copy(&W_inv[0],        W,     ecdh_len);
    co_bswap_copy(&Message[cursor], N1,    16);
    cursor += 16;
    co_bswap_copy(&Message[cursor], N2,    16);
    cursor += 16;
    co_bswap_copy(&Message[cursor], KeyId, 4);
    cursor += 4;
    co_bswap_copy(&Message[cursor], A1,    6);
    cursor += 6;
    co_bswap_copy(&Message[cursor], A2,    6);
    cursor += 6;

    sha_256_hmac(W_inv, ecdh_len, Message, cursor, &hash);

    // 128-bit result
    co_bswap_copy(OutPut, (uint8_t *) &hash, 16);


    PRINT_128BIT_KEY("f2", OutPut);
}

void sha_256_f3(bool p_256, const uint8_t *W, const uint8_t *N1, const uint8_t *N2, const uint8_t *R, const uint8_t *IOcap,
                const uint8_t *A1, const uint8_t *A2, uint8_t *OutPut)
{
    sha_256_hash_t hash;
    uint8_t W_inv[32];
    uint8_t Message[63];
    uint8_t cursor = 0;
    uint8_t ecdh_len = p_256 ? 32 : 24; // p192 or p256
    // W is  256/192 bits = 32/24 Bytes = 8/6 Words
    // N1 is 128 bits = 16 Bytes = 4 Words
    // N2 is 128 bits = 16 Bytes = 4 Words
    // R is  128 bits = 16 Bytes = 4 Words
    // IOcap is 24 bits = 4 Bytes = 1 Word
    // A1 is 48 bits = 6 Bytes
    // A2 is 48 bits = 6 Bytes

    co_bswap_copy(&W_inv[0],        W,     ecdh_len);
    co_bswap_copy(&Message[cursor], N1,    16);
    cursor += 16;
    co_bswap_copy(&Message[cursor], N2,    16);
    cursor += 16;
    co_bswap_copy(&Message[cursor], R,     16);
    cursor += 16;
    co_bswap_copy(&Message[cursor], IOcap, 3);
    cursor += 3;
    co_bswap_copy(&Message[cursor], A1,    6);
    cursor += 6;
    co_bswap_copy(&Message[cursor], A2,    6);
    cursor += 6;


    sha_256_hmac(W_inv, ecdh_len, Message, cursor, &hash);

    // 128-bit result
    co_bswap_copy(OutPut, (uint8_t *) &hash, 16);

    PRINT_128BIT_KEY("f3", OutPut);
}

void sha_256_h3(const uint8_t *T, const uint8_t *A1, const uint8_t *A2, const uint8_t *ACO, uint8_t *pResult)
{
    const uint8_t KeyID[4] = {0x6b, 0x61, 0x74, 0x62};
    sha_256_hash_t hash;
    uint8_t T_inv[16];
    uint8_t Message[24];

    // T = 16 Bytes.
    // KeyId = 4 Bytes
    // A1 = 6 Bytes
    // A2 = 6 Bytes
    // ACO = 8 Bytes
    // h3(W, keyID, A1, A2, ACO) = HMAC-SHA-256T(KeyID || A1 || A2 || ACO) / 2^128
    co_bswap_copy(&T_inv[0],    T,     16);
    co_bswap_copy(&Message[0],  KeyID, 4);
    co_bswap_copy(&Message[4],  A1,    6);
    co_bswap_copy(&Message[10], A2,    6);
    co_bswap_copy(&Message[16], ACO,   8);

    sha_256_hmac(T_inv, 16, Message, 24, &hash);

    // 128-bit result
    co_bswap_copy(pResult, (uint8_t *) &hash, 16);


    PRINT_128BIT_KEY("h3", pResult);
}

void sha_256_h4(const uint8_t *T, const uint8_t *A1, const uint8_t *A2, uint8_t *pResult)
{
    const uint8_t KeyID[4] = {0x6b, 0x64, 0x74, 0x62};
    sha_256_hash_t hash;
    uint8_t T_inv[16];
    uint8_t Message[16];
    // T = 16 Bytes.
    // KeyId = 4 Bytes
    // A1 = 6 Bytes
    // A2 = 6 Bytes
    // h4(W, keyID, A1, A2) = HMAC-SHA-256T(KeyID || A1 || A2) / 2^128

    co_bswap_copy(&T_inv[0],    T,     16);
    co_bswap_copy(&Message[0],  KeyID, 4);
    co_bswap_copy(&Message[4],  A1,    6);
    co_bswap_copy(&Message[10], A2,    6);

    sha_256_hmac(T_inv, 16, Message, 16, &hash);

    // 128-bit result
    co_bswap_copy(pResult, (uint8_t *) &hash, 16);

    PRINT_128BIT_KEY("h4", pResult);
}

void sha_256_h5(const uint8_t *W, const uint8_t *R_1, const uint8_t *R_2, uint8_t *pResult)
{
    sha_256_hash_t hash;
    uint8_t W_Inv[16];
    uint8_t Message[32];
    /*
     * H5 if used for the SRES and ACO calculation
     *
     * NOTE :- The R1 and R2 are the AU_RANDs of the Master and Slave (AU_RANDs and AU_RANDm)
     *
     * However, as both of the AU_RANDs are transmitted with Least Significant Octet on the Left
     * and the most significant octet on the Right - they have to be inverted for HMAC
     */

    // W = 16 Bytes.
    // R1 = 16 Bytes
    // R2 = 16 Bytes
    // h5(W, R1, R2) = HMAC-SHA-256T( R1 || R2) / 2^128

    co_bswap_copy(&W_Inv[0],   W,   16);
    co_bswap_copy(&Message[0], R_1, 16);
    co_bswap_copy(&Message[16], R_2, 16);

    sha_256_hmac(W_Inv, 16, Message, 32, &hash);

    // 128-bit result
    co_bswap_copy(pResult, (uint8_t *) &hash, 16);

    PRINT_128BIT_KEY("h5", pResult);
}
