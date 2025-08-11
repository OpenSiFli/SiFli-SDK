/*
 *  Copyright 2008-2020, 2024 NXP
 *
 *  SPDX-License-Identifier: BSD-3-Clause
 *
 */

/*!\file wmcrypto.h
 *\brief This file provides crypto wrapper interfaces.
 *
 *  This provides crypto wrapper functions that selectively call the WMSDKA or
 *  mbed TLS crypto functions based on the selected configuration.
 */

#ifndef WMCRYPTO_H
#define WMCRYPTO_H

#include <stdint.h>

#include <wmlog.h>

#define crypto_e(...) wmlog_e("crypt", ##__VA_ARGS__)
#define crypto_w(...) wmlog_w("crypt", ##__VA_ARGS__)

#if CONFIG_CRYPTO_DEBUG
    #define crypto_d(...) wmlog("crypt", ##__VA_ARGS__)
#else
    #define crypto_d(...)
#endif /* ! CONFIG_CRYPTO_DEBUG */

/* To do: find a way to get these digest and block size */
/** Digest size */
#define SHA256_DIGEST_SIZE (256 / 8)
/** Block size  */
#define SHA256_BLOCK_SIZE (512 / 8)

#define SHA1_MAC_LEN 20
#define MD5_MAC_LEN  16

/**
 *  * Diffie-Hellman parameters.
 *   */
typedef struct
{
    /** prime */
    unsigned char *prime;
    /** length of prime */
    unsigned int primeLen;
    /** generator */
    unsigned char *generator;
    /** length of generator */
    unsigned int generatorLen;
} DH_PG_PARAMS;

/**********************************************************
 *****     Diffie-Hellman Shared Key Generation       *****
 **********************************************************/
/**
 *  @brief  Sets up Diffie-Hellman key agreement.
 *
 *  @param public_key       Pointers to public key generated
 *  @param public_len       Length of public key
 *  @param private_key      Pointers to private key generated randomly
 *  @param private_len      Length of private key
 *  @param dh_params        Parameters for DH algorithm
 *
 *  @return                 0 on success, -1 on failure
 */
void *nxp_dh_setup_key(
    uint8_t *public_key, uint32_t public_len, uint8_t *private_key, uint32_t private_len, DH_PG_PARAMS *dh_params);

/**
 *  @brief  Computes agreed shared key from the public value,
 *          private value, and Diffie-Hellman parameters.
 *
 *  @param dh           DH key
 *  @param shared_key       Pointer to agreed shared key generated
 *  @param shared_len       Length of agreed shared key
 *  @param public_key       Pointer to public key generated
 *  @param public_len       Length of public key
 *  @param private_key      Pointer to private key generated randomly
 *  @param private_len      Length of private key
 *  @param dh_params        Parameters for DH algorithm
 *
 *  @return                 0 on success, -1 on failure
 */
int nxp_dh_compute_key(void *dh,
                       uint8_t *shared_key,
                       uint32_t shared_len,
                       uint8_t *public_key,
                       uint32_t public_len,
                       uint8_t *private_key,
                       uint32_t private_len,
                       DH_PG_PARAMS *dh_params);

/**
 * @brief  Free allocated DH key
 *
 * @param dh_context       DH key
 *
 * @return         None
 */
void nxp_dh_free(void *dh_context);

/**
 *  @brief  SHA-1 hash for data vector
 *
 *  @param nmsg         Number of elements in the data vector
 *  @param msg          Pointers to the data areas
 *  @param msglen       Lengths of the data blocks
 *  @param mac          Buffer for the hash
 *  @param maclen       Length of hash buffer
 *
 *  @return         0 on success, -1 of failure
 */
uint32_t nxp_sha1_vector(size_t nmsg, const uint8_t *msg[], const size_t msglen[], uint8_t *mac, size_t maclen);

/**********************************************************
 ***   HMAC-SHA256
 **********************************************************/
/**
 *  @brief  SHA-256 hash for data vector
 *
 *  @param nmsg         Number of elements in the data vector
 *  @param msg              Pointers to the data areas
 *  @param msglen           Lengths of the data blocks
 *  @param mac              Buffer for the hash
 *  @param maclen       Length of hash buffer
 *
 *  @return                 0 on success, -1 on failure
 */
uint32_t nxp_sha256_vector(size_t nmsg, const uint8_t *msg[], const size_t msglen[], uint8_t *mac, size_t maclen);

/**
 *  @brief  Wrapper function for SHA256 hash
 *
 *  @param num_elem     Number of elements in the data vector
 *  @param addr             Pointers to the data areas
 *  @param len              Lengths of the data blocks
 *  @param mac          Buffer for the hash
 *
 *  @return                 0 on success, -1 on failure
 */

void nxp_sha256(size_t num_elem, const uint8_t *addr[], const size_t *len, uint8_t *mac);

/**
 *  @brief  Wrapper function for HMAC-SHA256 (RFC 2104)
 *
 *  @param key              Key for HMAC operations
 *  @param keylen          Length of the key in bytes
 *  @param msg          Pointers to the data areas
 *  @param msglen       Lengths of the data blocks
 *  @param mac              Buffer for the hash (32 bytes)
 *  @param maclen           Length of hash buffer
 *
 *  @return                 0 on success, -1 on failure
 */
uint32_t nxp_hmac_sha256(
    const uint8_t *key, uint32_t keylen, uint8_t *msg, uint32_t msglen, uint8_t *mac, uint32_t maclen);

/**
 *  @brief Key derivation function to generate Authentication
 *         Key and Key Wrap Key used in WPS registration protocol
 *
 *  @param key          Input key to generate authentication key (KDK)
 *  @param key_len          Length of input key
 *  @param result           result buffer
 *  @param result_len       Length of result buffer
 *
 * @return          0 on success, 1 otherwise
 */
int nxp_kdf(uint8_t *key, uint32_t key_len, uint8_t *result, uint32_t result_len);

/**********************************************************
 *  ***   AES Key Wrap Key
 **********************************************************/
/**
 *  @brief  Wrap keys with AES Key Wrap Algorithm (128-bit KEK)
 *
 *  @param plain_txt        Plaintext key to be wrapped
 *  @param txt_len          Length of the plain key in bytes (16 bytes)
 *  @param cip_txt          Wrapped key
 *  @param kek              Key encryption key (KEK)
 *  @param kek_len          Length of KEK in bytes (must be divisible by 16)
 *  @param iv               Encryption IV for CBC mode (16 bytes)
 *
 *  @return                 0 on success, -1 on failure
 */
int nxp_aes_wrap(uint8_t *plain_txt, uint32_t txt_len, uint8_t *cip_txt, uint8_t *kek, uint32_t kek_len, uint8_t *iv);

/**
 *  @brief  Unwrap key with AES Key Wrap Algorithm (128-bit KEK)
 *
 *  @param cip_txt          Wrapped key to be unwrapped
 *  @param txt_len          Length of the wrapped key in bytes (16 bytes)
 *  @param plain_txt        Plaintext key
 *  @param kek              Key encryption key (KEK)
 *  @param kek_len          Length of KEK in bytes (must be divisible by 16)
 *  @param iv               Encryption IV for CBC mode (16 bytes)
 *
 *  @return                 0 on success, -1 on failure
 */
int nxp_aes_unwrap(uint8_t *cip_txt, uint32_t txt_len, uint8_t *plain_txt, uint8_t *kek, uint32_t kek_len, uint8_t *iv);

/**********************************************************
 *  ***   Extended AES Key Wrap Key
 **********************************************************/
/**
 *  @brief  Wrap keys with Extended AES Key Wrap Algorithm (128-bit KEK)
 *
 *  @param plain_txt        Plaintext key to be wrapped
 *  @param plain_len       Length of the plain key in bytes (16 bytes)
 *  @param cip_txt          Wrapped key
 *  @param kek              Key encryption key (KEK)
 *  @param kek_Len          Length of KEK in bytes (must be divisible by 16)
 *  @param iv               Encryption IV for CBC mode (16 bytes)
 *
 *  @return                 0 on success, -1 on failure
 */
int nxp_aes_wrap_ext(
    uint8_t *plain_txt, uint32_t plain_len, uint8_t *cip_txt, uint8_t *kek, uint32_t kek_Len, uint8_t *iv);

/**
 *  @brief  Unwrap key with Extended AES Key Wrap Algorithm (128-bit KEK)
 *
 *  @param cip_txt          Wrapped key to be unwrapped
 *  @param cip_Len          Length of the wrapped key in bytes (16 bytes)
 *  @param plain_txt        Plaintext key
 *  @param kek              Key encryption key (KEK)
 *  @param key_len          Length of KEK in bytes (must be divisible by 16)
 *  @param iv               Encryption IV for CBC mode (16 bytes)
 *
 *  @return                 0 on success, -1 on failure
 */
int nxp_aes_unwrap_ext(
    uint8_t *cip_txt, uint32_t cip_Len, uint8_t *plain_txt, uint8_t *kek, uint32_t key_len, uint8_t *iv);

/**
 *  @brief  Wrapper function for HMAC-MD5
 *
 *  @param input            Pointer to input data
 *  @param len              Length of input data
 *  @param hash         Pointer to hash
 *  @param hash_key     Pointer to hash key
 *
 *  @return                 None
 */
void nxp_crypto_hmac_md5(uint8_t *input, int len, uint8_t *hash, char *hash_key);

/**
 *  @brief  Wrapper function for MD5
 *
 *  @param input            Pointer to input data
 *  @param len              Length of input data
 *  @param hash             Pointer to hash
 *  @param hlen         Pointer to hash len
 *
 *  @return                 None
 */
void nxp_crypto_md5(uint8_t *input, int len, uint8_t *hash, int hlen);

/**
 *  @brief  Wrapper function for PBKDF2
 *
 *  @param password         Pointer to password
 *  @param ssid             Pointer to ssid
 *  @param ssidlength       Length of ssid
 *  @param iterations       No if iterations
 *  @param output_len       Length of output data
 *  @param output           Pointer to output data
 *
 *  @return                 None
 */
void nxp_crypto_pass_to_key(
    char *password, unsigned char *ssid, int ssidlength, int iterations, int output_len, unsigned char *output);

int nxp_sha1_prf(const uint8_t *key,
                 size_t key_len,
                 const char *label,
                 const uint8_t *data,
                 size_t data_len,
                 uint8_t *buf,
                 size_t buf_len);

int aes_wrap(const uint8_t *kek, int n, const uint8_t *plain, uint8_t *cipher);

int aes_unwrap(const uint8_t *kek, int n, const uint8_t *cipher, uint8_t *plain);

int hmac_sha1(const uint8_t *key, size_t key_len, const uint8_t *data, size_t data_len, uint8_t *mac, size_t maclen);

int nxp_tls_prf(const uint8_t *secret,
                size_t secret_len,
                const char *label,
                const uint8_t *seed,
                size_t seed_len,
                uint8_t *out,
                size_t outlen);

void nxp_hmac_md5(
    const uint8_t *key, size_t key_len, const uint8_t *data, size_t data_len, uint8_t *mac, size_t maclen);

int nxp_generate_psk(const char *ssid, int ssid_len, const char *passphrase, char *output);

#endif
