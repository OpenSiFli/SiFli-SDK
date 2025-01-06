/**
 ****************************************************************************************
 *
 * @file aes.h
 *
 * @brief Header file for AES crypto module
 *
 * Copyright (C) RivieraWaves 2017-2018
 *
 ****************************************************************************************
 */

#ifndef AES_H_
#define AES_H_

/**
 ****************************************************************************************
 * @defgroup AES_API AES-128 Cryptographic functions
 * @brief  Set of AES-128 based functions used for security
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include <stdint.h>
#include <stdbool.h>

/*
 * Defines
 ****************************************************************************************
 */

/// Size of an a AES Message block in bytes
#define AES_BLOCK_SIZE 16
/// Size of AES Key length
#define AES_KEY_LEN    16

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// AES IRK definition
typedef struct aes_key
{
    ///16-byte key value
    uint8_t value[AES_KEY_LEN];
} aes_key_t;

/**
 ****************************************************************************************
 * @brief Call back definition of the function that can handle result of an AES based algorithm
 *
 * @param[in] status       Execution status
 * @param[in] aes_res      16 bytes block result
 * @param[in] metainfo     Metadata information provided by API user (see \glos{METAINFO})
 ****************************************************************************************
 */
typedef void (*aes_func_result_cb)(uint8_t status, const uint8_t *aes_res, uint32_t metainfo);

/**
 ****************************************************************************************
 * @brief Call back definition of the function that can handle result of AES-CCM Cipher/Decipher
 *
 * @param[in] mic_error  True if a MIC error detected when Decipher, False else
 *                       In case of MIC error output message is considered invalid
 * @param[in] metainfo     Metadata information provided by API user (see \glos{METAINFO})
 ****************************************************************************************
 */
typedef void (*aes_ccm_func_result_cb)(bool mic_error, uint32_t metainfo);

/**
 ****************************************************************************************
 * @brief Call back definition of the function that can handle result of AES-CTR Cipher/Decipher
 *
 * @param[in] metainfo     Metadata information provided by API user (see \glos{METAINFO})
 ****************************************************************************************
 */
typedef void (*aes_ctr_func_result_cb)(uint32_t metainfo);

/**
 ****************************************************************************************
 * @brief Call back definition of the Resolvable Private Address resolution function
 *
 * @param[in] status     Execution status
 * @param[in] index      Index of the IRK used to resolve the provided RPA (number of IRK if not resolved)
 * @param[in] metainfo   Metadata information provided by API user (see \glos{METAINFO})
 * @param[in] p_addr     Pointer to base RPA address
 * @param[in] p_irk      Pointer to valid IRK
 ****************************************************************************************
 */
typedef void (*aes_rpa_func_result_cb)(uint8_t status, uint8_t index, uint32_t metainfo, const uint8_t *p_addr,
                                       const aes_key_t *p_irk);

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Initialize AES function management
 *
 * @param[in] is_reset  True if reset is requested, false for an initialization
 ****************************************************************************************
 */
void aes_init(bool is_reset);

/**
 ****************************************************************************************
 * @brief Handler of AES execution (HW accelerator if BLE controller present, HCI Encrypt for BLE Host Stack)
 *
 * @param[in] status  Status of AES execution
 * @param[in] result  16 bytes result of AES execution
 ****************************************************************************************
 */
void aes_result_handler(uint8_t status, uint8_t *result);

/**
 ****************************************************************************************
 * @brief Perform an AES encryption - result within callback
 * @param[in] key      Key used for the encryption
 * @param[in] val      Value to encrypt using AES
 * @param[in] copy     Copy parameters because source is destroyed
 * @param[in] res_cb   Function that will handle the AES based result (16 bytes)
 * @param[in] metainfo Metadata information that will be returned in procedure callback functions (see \glos{METAINFO})
 ****************************************************************************************
 */
void aes_encrypt(const uint8_t *key, const uint8_t *val, bool copy, aes_func_result_cb res_cb, uint32_t metainfo);

/**
 ****************************************************************************************
 * @brief Perform an AES decryption - result within callback
 * @param[in] key      Key used for the encryption
 * @param[in] val      Value to decrypt using AES
 * @param[in] copy     Copy parameters because source is destroyed
 * @param[in] res_cb   Function that will handle the AES based result (16 bytes)
 * @param[in] metainfo Metadata information that will be returned in procedure callback functions (see \glos{METAINFO})
 ****************************************************************************************
 */
void aes_decrypt(const uint8_t *key, const uint8_t *val, bool copy, aes_func_result_cb res_cb, uint32_t metainfo);

/**
 ****************************************************************************************
 * @brief Generate a random number using AES encryption - result within callback
 *
 * @param[in] res_cb   Function that will handle the AES based result (16 bytes)
 * @param[in] metainfo Metadata information that will be returned in procedure callback functions (see \glos{METAINFO})
 ****************************************************************************************
 */
void aes_rand(aes_func_result_cb res_cb, uint32_t metainfo);

/**
 ****************************************************************************************
 * @brief Perform a XOR of two numbers.
 *
 * @param[out] result Output 128 bits number: result = a ^ b
 * @param[in]  a      first 128 bits operand
 * @param[in]  b      second 128 bits operand
 * @param[in]  size   number of bytes to XOR
 ****************************************************************************************
 */
void aes_xor_128(uint8_t *result, const uint8_t *a, const uint8_t *b, uint8_t size);

/**
 ****************************************************************************************
 * @brief Start the AES S1 crypto function.
 *
 * @param[in] message      Message used to generate Salted key
 * @param[in] message_len  Length (in bytes) of the block of data M
 * @param[in] res_cb       Function that will handle the AES based result (16 bytes)
 * @param[in] metainfo     Metadata information that will be returned in procedure callback functions (see \glos{METAINFO})
 ****************************************************************************************
 */
void aes_s1(const uint8_t *message, uint8_t message_len, aes_func_result_cb res_cb, uint32_t metainfo);

/**
 ****************************************************************************************
 * @brief Start the AES K1 crypto function.
 *
 * @param[in] salt         Salted Key to use
 * @param[in] n            Value of N
 * @param[in] n_len        Length of N
 * @param[in] p            Value of P
 * @param[in] p_len        Length of P
 * @param[in] res_cb       Function that will handle the AES based result (16 bytes)
 * @param[in] metainfo     Metadata information that will be returned in procedure callback functions (see \glos{METAINFO})
 ****************************************************************************************
 */
void aes_k1(const uint8_t *salt, const uint8_t *n, uint8_t n_len, const uint8_t *p, uint8_t p_len,
            aes_func_result_cb res_cb, uint32_t metainfo);

/**
 ****************************************************************************************
 * @brief Compute Confirm value
 *
 * @param[in] k        Key used for aes functions
 * @param[in] r        Random number
 * @param[in] p1       p1 = pres || preq || rat’ || iat’
 * @param[in] p2       p2 = padding || ia || ra
 * @param[in] res_cb   Function that will handle the AES based result (16 bytes)
 * @param[in] metainfo Metadata information that will be returned in procedure callback functions (see \glos{METAINFO})
 ****************************************************************************************
 */
void aes_c1(const uint8_t *k, const uint8_t *r, const uint8_t *p1, const uint8_t *p2,
            aes_func_result_cb res_cb, uint32_t metainfo);

/**
 ****************************************************************************************
 * @brief Compute LE Secure Connections Confirm Value Generation Function f4
 *
 * @param[in] u        U is 256 bits
 * @param[in] v        V is 256 bits
 * @param[in] x        X is 128 bits
 * @param[in] z        Z is   8 bits
 * @param[in] res_cb   Function that will handle the AES based result (16 bytes)
 * @param[in] metainfo Metadata information that will be returned in procedure callback functions (see \glos{METAINFO})
 ****************************************************************************************
 */
void aes_f4(const uint8_t *u, const uint8_t *v, const uint8_t *x, uint8_t z,
            aes_func_result_cb res_cb, uint32_t metainfo);

/**
 ****************************************************************************************
 * @brief Compute LE Secure Connections Key Generation Function f5
 *
 * @param[in] w        W  is 256 bits
 * @param[in] n1       N1 is 128 bits
 * @param[in] n2       N2 is 128 bits
 * @param[in] a1       A1 is  56 bits
 * @param[in] a2       A2 is  56 bits
 * @param[in] res_cb   Function that will handle the AES based result (16 bytes)
 * @param[in] metainfo Metadata information that will be returned in procedure callback functions (see \glos{METAINFO})
 ****************************************************************************************
 */
void aes_f5(const uint8_t *w, const uint8_t *n1, const uint8_t *n2, const uint8_t *a1, const uint8_t *a2,
            aes_func_result_cb res_cb, uint32_t metainfo);


/**
 ****************************************************************************************
 * @brief Compute LE Secure Connections Check Value Generation Function f6
 *
 * @param[in] w        W     is 128 bits
 * @param[in] n1       N1    is 128 bits
 * @param[in] n2       N2    is 128 bits
 * @param[in] r        R     is 128 bits
 * @param[in] iocap    IOcap is  24 bits
 * @param[in] a1       A1    is  56 bits
 * @param[in] a2       A2    is  56 bits
 *
 * @param[in] res_cb   Function that will handle the AES based result (16 bytes)
 * @param[in] metainfo Metadata information that will be returned in procedure callback functions (see \glos{METAINFO})
 ****************************************************************************************
 */
void aes_f6(const uint8_t *w, const uint8_t *n1, const uint8_t *n2, const uint8_t *r,  const uint8_t *iocap,
            const uint8_t *a1, const uint8_t *a2, aes_func_result_cb res_cb, uint32_t metainfo);

/**
 ****************************************************************************************
 * @brief Compute LE Secure Connections Numeric Comparison Value Generation Function g2
 *
 * @param[in] u        U     is 256 bits
 * @param[in] v        V     is 256 bits
 * @param[in] x        X     is 128 bits
 * @param[in] y        Y     is 128 bits
 *
 * @param[in] res_cb   Function that will handle the AES based result (16 bytes)
 * @param[in] metainfo Metadata information that will be returned in procedure callback functions (see \glos{METAINFO})
 ****************************************************************************************
 */
void aes_g2(const uint8_t *u, const uint8_t *v, const uint8_t *x, const uint8_t *y,
            aes_func_result_cb res_cb, uint32_t metainfo);

/**
 ****************************************************************************************
 * @brief Start the AES CMAC crypto function. Allocate memory for the CMAC and
 *        begins the subkey generation
 *
 * @param[in] key          Pointer to the Key to be used
 * @param[in] message      Pointer to the block of data the data on which the CMAC is performed
 * @param[in] message_len  Length (in bytes) of the block of data M
 * @param[in] res_cb       Function that will handle the AES based result (16 bytes)
 * @param[in] metainfo     Metadata information that will be returned in procedure callback functions (see \glos{METAINFO})
 ****************************************************************************************
 */
void aes_cmac(const uint8_t *key, const uint8_t *message, uint16_t message_len,
              aes_func_result_cb res_cb, uint32_t metainfo);


/**
 ****************************************************************************************
 * @brief Start the AES K2 crypto function.
 *
 * @param[in] n            Value of N - 128 bits
 * @param[in] p            Value of P
 * @param[in] p_len        Length of P
 * @param[in] res_cb       Function that will handle the AES based result (33 bytes)
 * @param[in] metainfo     Metadata information that will be returned in procedure callback functions (see \glos{METAINFO})
 ****************************************************************************************
 */
void aes_k2(const uint8_t *n, const uint8_t *p, uint8_t p_len, aes_func_result_cb res_cb, uint32_t metainfo);


/**
 ****************************************************************************************
 * @brief Start the AES K3 crypto function.
 *
 * @param[in] n            Value of N - 128 bits
 * @param[in] res_cb       Function that will handle the AES based result (8 bytes)
 * @param[in] metainfo     Metadata information that will be returned in procedure callback functions (see \glos{METAINFO})
 ****************************************************************************************
 */
void aes_k3(const uint8_t *n, aes_func_result_cb res_cb, uint32_t metainfo);

/**
 ****************************************************************************************
 * @brief Start the AES K4 crypto function.
 *
 * @param[in] n            Value of N - 128 bits
 * @param[in] res_cb       Function that will handle the AES based result (1 byte)
 * @param[in] metainfo     Metadata information that will be returned in procedure callback functions (see \glos{METAINFO})
 ****************************************************************************************
 */
void aes_k4(const uint8_t *n, aes_func_result_cb res_cb, uint32_t metainfo);


/**
 ****************************************************************************************
 * @brief Start the AES CCM crypto function. Allocate memory for the CCM and start processing it
 *        Execute result callback at end of function execution
 *
 * @param[in]  key               Pointer to the Key to be used
 * @param[in]  nonce             13 Bytes Nonce to use for cipher/decipher
 * @param[in]  in_message        Input message for AES-CCM execution
 * @param[out] out_message       Output message that will contain cipher+mic or decipher data
 * @param[in]  message_len       Length of Input/Output message without mic
 * @param[in]  mic_len           Length of the mic to use (2, 4, 6, 8, 10, 12, 14, 16 valid)
 * @param[in]  cipher            True to encrypt message, False to decrypt it.
 * @param[in]  add_auth_data     Additional Authentication data used for computation of MIC
 * @param[in]  add_auth_data_len Length of Additional Authentication data
 * @param[in]  res_cb            Function that will handle the AES CCM result
 * @param[in]  metainfo          Metadata information that will be returned in procedure callback functions (see \glos{METAINFO})
 ****************************************************************************************
 */
void aes_ccm(const uint8_t *key, const uint8_t *nonce, const uint8_t *in_message,
             uint8_t *out_message, uint16_t message_len, uint8_t mic_len, bool cipher,
             const uint8_t *add_auth_data, uint8_t add_auth_data_len, aes_ccm_func_result_cb res_cb, uint32_t metainfo);

/**
 ****************************************************************************************
 * @brief Start the AES CTR crypto function. Allocate memory for the CTR and start processing it
 *        Execute result callback at end of function execution
 *
 * @param[in]  key               Pointer to the Key to be used in LSB Format
 * @param[in]  nonce             16-Bytes Nonce to use for cipher/decipher in LSB Format
 * @param[in]  message_len       Length of Input/Output message without mic
 * @param[in]  p_in_message      Pointer to input message for AES-CCM execution
 * @param[out] p_out_message     Pointer to output message that will contain cipher or decipher data
 * @param[in]  res_cb            Function that will handle the AES CTR result
 * @param[in]  metainfo          Metadata information that will be returned in procedure callback functions (see \glos{METAINFO})
 ****************************************************************************************
 */
void aes_ctr(const uint8_t *key, const uint8_t *nonce, uint16_t message_len, const uint8_t *p_in_message,
             uint8_t *p_out_message, aes_ctr_func_result_cb res_cb, uint32_t metainfo);

/**
 ****************************************************************************************
 * @brief Key Conversion Function h6
 *
 * @param[in] w            W is a 128bits data
 * @param[in] key_id       KeyID is a 32 bits  data
 * @param[in] res_cb       Function that will handle the AES CCM result
 * @param[in] metainfo     Metadata information that will be returned in procedure callback functions (see \glos{METAINFO})
 ****************************************************************************************
 */
void aes_h6(const uint8_t *w, const uint8_t *key_id, aes_func_result_cb res_cb, uint32_t metainfo);

/**
 ****************************************************************************************
 * @brief Key Conversion Function h7
 *
 * @param[in] salt         SALT is a 128bits data
 * @param[in] w            W is a 128bits key
 * @param[in] res_cb       Function that will handle the AES CCM result
 * @param[in] metainfo     Metadata information that will be returned in procedure callback functions (see \glos{METAINFO})
 ****************************************************************************************
 */
void aes_h7(const uint8_t *salt, const uint8_t *w, aes_func_result_cb res_cb, uint32_t metainfo);

/**
 ****************************************************************************************
 * @brief Group Session Key Derivation Function h8
 *
 * @param[in] k            K is a 128bits data
 * @param[in] s            S is a 128bits key
 * @param[in] key_id       KeyID is a 32 bits  data
 * @param[in] res_cb       Function that will handle the AES CCM result
 * @param[in] metainfo     Metadata information that will be returned in procedure callback functions (see \glos{METAINFO})
 ****************************************************************************************
 */
void aes_h8(const uint8_t *k, const uint8_t *s, const uint8_t *key_id, aes_func_result_cb res_cb, uint32_t metainfo);

/**
 ****************************************************************************************
 * @brief Group Long Term Key Generation Function h9
 *
 * @param[in] w            W is a 128bits data
 * @param[in] key_id       KeyID is a 32 bits  data
 * @param[in] res_cb       Function that will handle the AES CCM result
 * @param[in] metainfo     Metadata information that will be returned in procedure callback functions (see \glos{METAINFO})
 ****************************************************************************************
 */
void aes_h9(const uint8_t *w, const uint8_t *key_id, aes_func_result_cb res_cb, uint32_t metainfo);

/**
 ****************************************************************************************
 * @brief Address generation Function
 *
 * @param[in] key          Pointer to 128-bit key used for address generation
 * @param[in] addr_type    Address type (0=public / 1=private random)
 * @param[in] res_cb       Function that will handle the AES RPA generation result (address generated in the 6 LSBs of the returned buffer)
 * @param[in] metainfo     Metadata information that will be returned in procedure callback functions (see \glos{METAINFO})
 ****************************************************************************************
 */
void aes_gen_rand_addr(const uint8_t *key, uint8_t addr_type, aes_func_result_cb res_cb, uint32_t metainfo);

/**
 ****************************************************************************************
 * @brief Resolvable Private Address generation Function
 *
 * @param[in] irk          Pointer to IRK (local IRK to generate a local RPA)
 * @param[in] res_cb       Function that will handle the AES RPA generation result (address generated in the 6 LSBs of the returned buffer)
 * @param[in] metainfo     Metadata information that will be returned in procedure callback functions (see \glos{METAINFO})
 ****************************************************************************************
 */
void aes_rpa_gen(const aes_key_t *irk, aes_func_result_cb res_cb, uint32_t metainfo);

/**
 ****************************************************************************************
 * @brief Resolvable Private Address resolution Function
 *
 * @param[in] nb_irk       Number of IRKs provided
 * @param[in] irk          Table of IRKs (stored internally to AES RPA, caller can destroy the table)
 * @param[in] addr         BD address to resolve
 * @param[in] res_cb       Function that will handle the AES RPA resolution result
 * @param[in] metainfo     Metadata information that will be returned in procedure callback functions (see \glos{METAINFO})
 ****************************************************************************************
 */
void aes_rpa_resolve(uint8_t nb_irk, const aes_key_t *irk, const uint8_t *addr, aes_rpa_func_result_cb res_cb, uint32_t metainfo);


/// @} AES_API
///
#endif /* AES_H_ */
