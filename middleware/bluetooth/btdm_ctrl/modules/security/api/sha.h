/**
 ****************************************************************************************
 *
 * @file sha.h
 *
 * @brief SHA-2 (Secure Hash Algorithm) function declaration
 *
 * Copyright (C) RivieraWaves 2009-2021
 *
 ****************************************************************************************
 */


#ifndef _SHA_H_
#define _SHA_H_

/**
 ****************************************************************************************
 * @addtogroup SHA_API
 * @ingroup MODULE_API
 * @brief  SHA-2 (Secure Hash Algorithm)
 *
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "stdint.h"
#include "stdbool.h"

/*
 * DEFINES
 ****************************************************************************************
 */
#define SHA_256_HASH_BYTE_SIZE  (32)
#define SHA_256_HASH_WORD_SIZE  (SHA_256_HASH_BYTE_SIZE / 4)


/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// Hash value
typedef struct sha_256_hash
{
    uint32_t value[SHA_256_HASH_WORD_SIZE];
} sha_256_hash_t;

/*
 * MACROS
 ****************************************************************************************
 */


/*
 * GLOBAL VARIABLE DECLARATIONS
 ****************************************************************************************
 */

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * Compute message hash using SHA-256 algorithm
 *
 * @param[in]  p_message   Pointer to message value used to compute hash
 * @param[in]  message_len Size of the message
 * @param[out] p_hash      Pointer to the value that contains computed hash -- in MSB Format
 *
 * @return Status of function execution (see enum #co_error)
 ****************************************************************************************
 */
void sha_256(const uint8_t *p_message, uint16_t message_len, sha_256_hash_t *p_hash);

/**
 ****************************************************************************************
 * Compute HMAC-SHA256 of a message
 *
 * @param[in]  p_key       Pointer to the key -- in MSB Format
 * @param[in]  key_len     Size of the key (shall be less or equals 64 bytes)
 * @param[in]  p_message   Pointer to message value used to compute hash
 * @param[in]  message_len Size of the message
 * @param[out] p_hash      Pointer to the value that contains computed hash -- in MSB Format
 *
 * @return Status of function execution (see enum #co_error)
 ****************************************************************************************
 */
void sha_256_hmac(const uint8_t *p_key, uint8_t key_len, const uint8_t *p_message, uint16_t message_len,
                  sha_256_hash_t *p_hash);

/**
 ****************************************************************************************
 * @brief This function is used to calculate the F1 Value
 *
 * @param[in] p256    True if P-256 algo, False for P-192
 * @param[in] U       pointer to 32 byte value U (see BT Spec)
 * @param[in] V       pointer to 32 byte value V (see BT Spec)
 * @param[in] X       pointer to 16 byte value X (see BT Spec)
 * @param[in] Z       pointer to single byte Z (see BT Spec)
 *
 * @param[out] Output  pointer to 16 byte result (see BT Spec)
 *
 ****************************************************************************************
 */
void sha_256_f1(bool p_256, const uint8_t *U, const uint8_t *V, const uint8_t *X, const uint8_t *Z, uint8_t *OutPut);


/**
 ****************************************************************************************
 * @brief This function is used to calculate the G value
 *
 * g(U, V, X, Y) = SHA-256 (U || V || X || Y) / 2e32
 *
 * @param[in] p256    True if P-256 algo, False for P-192
 * @param[in] U       pointer to 32 byte value U (see BT Spec)
 * @param[in] V       pointer to 32 byte value V (see BT Spec)
 * @param[in] X       pointer to 16 byte value X (see BT Spec)
 * @param[in] Y       pointer to 16 byte value Y (see BT Spec)
 *
 * @param[out] Output  pointer to 4 byte result (see BT Spec)
 ****************************************************************************************
 */
void sha_256_g(bool p_256, const uint8_t *U, const uint8_t *V, const uint8_t *X, const uint8_t *Y, uint32_t *OutPut);


/**
 ****************************************************************************************
 * @brief This function is used to calculate the F2 value
 *
 * @param[in] p256    True if P-256 algo, False for P-192
 * @param[in] W       pointer to 32 byte value W (see BT Spec)
 * @param[in] N1      pointer to 16 byte value N1 (see BT Spec)
 * @param[in] N2      pointer to 16 byte value N2 (see BT Spec)
 * @param[in] KeyId   pointer to 4 Byte KeyId(see BT Spec)
 * @param[in] A1      pointer to 6 Address value A1 (see BT Spec)
 * @param[in] A2      pointer to 6 Address value A2 (see BT Spec)
 *
 * @param[out] Output  pointer to 16 byte result (see BT Spec)
 ****************************************************************************************
 */
void sha_256_f2(bool p_256, const uint8_t *W, const uint8_t *N1, const uint8_t *N2, const uint8_t *KeyId, const uint8_t *A1,
                const uint8_t *A2, uint8_t *OutPut);

/**
 ****************************************************************************************
 * @brief This function is used to calculate the F3 value
 *
 * @param[in] p256    True if P-256 algo, False for P-192
 * @param[in] W       pointer to 32 byte value W (see BT Spec)
 * @param[in] N1      pointer to 16 byte value N1 (see BT Spec)
 * @param[in] N2      pointer to 16 byte value N2 (see BT Spec)
 * @param[in] R1      pointer to 16 byte value R1 (see BT Spec)
 * @param[in] IOcap   pointer to 4 Byte IO Capabilities (see BT Spec)
 * @param[in] A1      pointer to 6 Address value A1 (see BT Spec)
 * @param[in] A2      pointer to 6 Address value A2 (see BT Spec)
 *
 * @param[out] Output  pointer to 16 byte result (see BT Spec)
 ****************************************************************************************
 */
void sha_256_f3(bool p_256, const uint8_t *W, const uint8_t *N1, const uint8_t *N2, const uint8_t *R, const uint8_t *IOcap,
                const uint8_t *A1, const uint8_t *A2, uint8_t *OutPut);

/**
 ****************************************************************************************
 * @brief This function is used to calculate the H3 value
 * The AES encryption key is generated using H3.
 *
 * h3(W, keyID, A1, A2, ACO) = HMAC-SHA-256T(KeyID || A1 || A2 || ACO) / 2^128
 *
 * @param[in] T       pointer to the 128 bit Bluetooth Link Key derived from f2
 * @param[in] A1      pointer to 6 Byte BD_ADDR of the master.
 * @param[in] A2      pointer to 6 Byte BD_ADDR of the slave.
 * @param[in] ACO     pointer to 8 byte ACO output from h5
 *
 * @param[out] pResult  pointer to 16 byte result (see BT Spec)
 ****************************************************************************************
 */
void sha_256_h3(const uint8_t *T, const uint8_t *A1, const uint8_t *A2, const uint8_t *ACO, uint8_t *pResult);

/**
 ****************************************************************************************
 * @brief This function is used to calculate the H4 value
 * H4 determines the Device Authentication Key
 *
 * h4(W, keyID, A1, A2) = HMAC-SHA-256T(KeyID || A1 || A2) / 2^128
 *
 * @param[in] T       pointer to the 128 bit Bluetooth Link Key.
 * @param[in] A1      pointer to 6 Byte BD_ADDR of the master.
 * @param[in] A2      pointer to 6 Byte BD_ADDR of the slave.
 *
 * @param[out] pResult  pointer to 16 byte result (see BT Spec)
 *
 * NOTE :- The KeyId is fixed and thus not passed as a parameter.
 ****************************************************************************************
 */
void sha_256_h4(const uint8_t *T, const uint8_t *A1, const uint8_t *A2, uint8_t *pResult);

/**
 ****************************************************************************************
 * @brief This function is used to calculate the H5 value
 * H5 if used for the SRES and ACO calculation. The R1 and R2 are the AU_RANDs of the
 * Master and Slave (AU_RANDs and AU_RANDm)
 *
 *  h5(W, R1, R2) = HMAC-SHA-256T( R1 || R2) / 2^128
 *
 * @param[in] W       pointer to the 16 Byte Device Authentication Key.
 * @param[in] R1      pointer to 16 Byte AU_RAND.
 * @param[in] R2      pointer to 16 Byte AU_RAND.
 *
 * @param[out] pResult  pointer to 16 byte result (see BT Spec)
 *
 ****************************************************************************************
 */
void sha_256_h5(const uint8_t *W, const uint8_t *R_1, const uint8_t *R_2, uint8_t *pResult);

/*
 * BLOOM FILTER
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief This function is used to initialize the bloom filter with zeros
 *
 * @param[in] filter_size   Size of the filter
 * @param[in] p_filter      Content of the filter
 ****************************************************************************************
 */
void sha_bloom_filter_init(uint16_t filter_size, uint8_t *p_filter);


/**
 ****************************************************************************************
 * @brief This function is used to add a key element in bloom filter using SHA-256 hash function
 *
 * @param[in]     filter_size   Size of the filter
 * @param[in|out] p_filter      Filter data to update
 * @param[in]     p_key         16-bit key element to add in the filter (LSB Format)
 * @param[in]     salt_size     Size of salt data
 * @param[in]     p_salt        Pointer to salt data used for hashing
 ****************************************************************************************
 */
void sha_bloom_filter_add_key(uint16_t filter_size, uint8_t *p_filter, const uint8_t *p_key,
                              uint8_t salt_size, const uint8_t *p_salt);


/// @} SHA_256_API

#endif /* _SHA_H_ */
