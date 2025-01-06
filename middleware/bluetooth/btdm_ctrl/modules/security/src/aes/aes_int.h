/**
 ****************************************************************************************
 *
 * @file aes_int.h
 *
 * @brief Header file for AES Internal crypto module
 *
 * Copyright (C) RivieraWaves 2017-2018
 *
 ****************************************************************************************
 */

#ifndef AES_INT_H_
#define AES_INT_H_

/**
 ****************************************************************************************
 * @defgroup AES_INT
 * @ingroup AES
 * @brief  AES_INT Crypto internal definitions
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"
#include "aes.h"


#include "co_bt.h"      // Common defines
#include "co_list.h"    // List usage

/*
 * Defines
 ****************************************************************************************
 */

// Size of an a AES Message block in bytes
#define AES_BLOCK_SIZE 16

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */
struct aes_func_env;

/**
 ****************************************************************************************
 * Callback used to continue execution of the AES based function.
 *
 * @param[in] aes_result Result of the AES (aes(key , val))
 *
 * @return True if function execution is over
 ****************************************************************************************
 */
typedef  bool (*aes_func_continue_cb)(struct aes_func_env *aes_env, uint8_t *aes_result);

/// Environment variable required for an AES based function
/// This structure must be Header of all function environment variables
struct aes_func_env
{
    /// used to put AES function in the AES execution queue
    struct co_list_hdr    hdr;
    /// AES continue callback
    aes_func_continue_cb  aes_continue_cb;
    /// AES End callback
    aes_func_result_cb    aes_res_cb;

    /// Key to use for the AES execution
    const uint8_t       *key;
    /// Value to use for AES Cypher/Decypher
    const uint8_t       *val;
    /// Information put in source id message to retrieve requester
    uint32_t             metainfo;
};

#if (BLE_EMB_PRESENT || BLE_HOST_PRESENT)
// Structure definition of the AES CMAC algorithm
struct aes_cmac_env
{
    /// AES Environment structure
    struct aes_func_env aes_env;

    /// M: Pointer to the message to be authenticated
    const uint8_t      *message; // pointer to memory allocated by calling function

    /// K: authentication key
    const uint8_t      *auth_key;

    /// T: message authentication code
    uint8_t             auth_code[AES_BLOCK_SIZE];

    /// Length of the message
    uint16_t            message_len;
    ///  Number of blocks (1 block = 16 bytes)
    uint8_t             num_blocks;
    /// Current block to process
    uint8_t             cur_block;
};
#endif // (BLE_EMB_PRESENT || BLE_HOST_PRESENT)

/*
 * GLOBAL VARIABLE DECLARATIONS
 ****************************************************************************************
 */

/// zero block
extern const uint8_t aes_cmac_zero[AES_BLOCK_SIZE];

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Allocate environment for the AES based function execution
 *
 * @param[in] size              Size of the environment to allocate (must be >= sizeof(struct aes_func_env))
 * @param[in] aes_continue_cb   Callback used to send the AES result
 * @param[in] res_cb            Function that will handle end of AES Based algorithm
 * @param[in] metainfo          Metadata information that will be returned in procedure callback functions (see \glos{METAINFO})
 *
 * @return Allocated environment variable
 ****************************************************************************************
 */
struct aes_func_env *aes_alloc(uint16_t size, aes_func_continue_cb aes_continue_cb, aes_func_result_cb res_cb,
                               uint32_t metainfo);

/**
 ****************************************************************************************
 * @brief AES Cypher request function.
 *
 * This will queue AES request in the AES execution queue
 * When the AES result is received, the AES continue callback is executed.
 *
 * If AES continue function returns that AES execution is over, a message will be send to destination task
 * with latest AES result.
 *
 * @param[in] env     AES environment
 * @param[in] key     Secret key
 * @param[in] val     Value to cipher / de-cipher
 * @param[in] cipher  True to cipher data; de-cipher data otherwise
 *
 ****************************************************************************************
 */
void aes_start(struct aes_func_env *env, const uint8_t *key, const uint8_t *val, bool cipher);

#if (BLE_EMB_PRESENT || BLE_HOST_PRESENT)
    /**
    ****************************************************************************************
    * @brief Start the AES CMAC crypto function. Allocate memory for the CMAC and
    *        begins the subkey generation
    *
    * @param[in] enc          AES CMAC Environment
    * @param[in] key          Pointer to the Key to be used
    * @param[in] message      Pointer to the block of data the data on which the CMAC is performed
    * @param[in] message_len  Length (in bytes) of the block of data M
    ****************************************************************************************
    */
    void aes_cmac_start(struct aes_cmac_env *env, const uint8_t *key, const uint8_t *message, uint16_t message_len);

    /**
    ****************************************************************************************
    * @brief Continue AES CMAC algorithm
    *
    * @param[in] env       AES CMAC Environment
    * @param[in] aes_res   AES Result
    *
    * @return True if algorithm is over, False else
    ****************************************************************************************
    */
    bool aes_cmac_continue(struct aes_cmac_env *env, uint8_t *aes_res);

    /**
    ****************************************************************************************
    * @brief Perform shift left of a 128 bits numvber
    *
    * @param[in]  input  Input 128 bits number
    * @param[out] output Output 128 bits number: output = input << 1
    ****************************************************************************************
    */
    void aes_shift_left_128(const uint8_t *input, uint8_t *output);
#endif // (BLE_EMB_PRESENT || BLE_HOST_PRESENT)


/**
 ****************************************************************************************
 * @brief Perform a XOR of two numbers (output and first parameters are LSB).
 *
 * @param[out] p_output Output number: result = a ^ b => LSB (128 bits block)
 * @param[in]  p_in_a   First operand LSB (128 bits block)
 * @param[in]  p_in_b   Second operand MSB
 * @param[in]  size     Number of bytes to XOR
 * @param[in]  offset   Position offset in the payload
 ****************************************************************************************
 */
__INLINE void aes_xor_128_lsb(uint8_t *p_output, const uint8_t *p_in_a, const uint8_t *p_in_b, uint8_t size, uint8_t offset)
{
    int8_t i;
    offset = AES_BLOCK_SIZE - offset - 1;
    for (i = 0; i < size ; i++)
    {
        p_output[offset - i] = p_in_a[offset - i] ^ p_in_b[i];
    }
}

/**
 ****************************************************************************************
 * @brief Perform a XOR of two numbers (output and first parameters are MSB).
 *
 * @param[out] p_output Output number: result = a ^ b => MSB (variable size block)
 * @param[in]  p_in_a   First operand MSB (variable size block)
 * @param[in]  p_in_b   Second operand LSB (128 bits block)
 * @param[in]  size     Number of bytes to XOR
 ****************************************************************************************
 */
__INLINE void aes_xor_128_msb(uint8_t *p_output, const uint8_t *p_in_a, const uint8_t *p_in_b, uint8_t size)
{
    int8_t i;
    for (i = 0; i < size ; i++)
    {
        p_output[i] = p_in_a[i] ^ p_in_b[AES_BLOCK_SIZE - i - 1];
    }
}

/// @} AES_INT
///
#endif /* AES_INT_H_ */
