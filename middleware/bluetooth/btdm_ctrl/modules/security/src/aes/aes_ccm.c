/**
 ****************************************************************************************
 * *
 * @file aes_ccm.c
 *
 * @brief Definition file for AES CCM crypto module functions
 *
 * Copyright (C) RivieraWaves 2017-2018
 *
 *
 * Algorithm described in RFC 3610
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

#include "arch.h"     // For architecture platform defines
#include "aes_int.h"  // AES internals
#include <string.h>   // for memcpy function
#include "co_utils.h" // for Bit field access
#include "co_endian.h"// For endianess requiements

/*
 * DEFINES
 ****************************************************************************************
 */
/// Size of supported Nonce size
#define AES_CCM_NONCE_SIZE      (13)

/// Bit Field used to know where we are in AES-CCM execution
///    7     6     5     4     3     2     1     0
/// +-----+-----+-----+-----+-----+-----+-----+-----+
/// |       RFU       |BLK_0|DATA |AUTH |M_GEN|CIPH |
/// +-----+-----+-----+-----+-----+-----+-----+-----+
///
enum aes_ccm_state_field
{
    /// True if ciphering, False if deciphering data.
    AES_CCM_CIPHER_POS      = 0,
    AES_CCM_CIPHER_BIT      = (1 << 0),
    /// True if AES execution is for MAC generation or Encryption
    AES_CCM_MAC_GEN_POS     = 1,
    AES_CCM_MAC_GEN_BIT     = (1 << 1),
    /// True if Authenticated optional data is under process
    AES_CCM_AUTH_POS        = 2,
    AES_CCM_AUTH_BIT        = (1 << 2),
    /// True if data is under process
    AES_CCM_DATA_POS        = 3,
    AES_CCM_DATA_BIT        = (1 << 3),
    /// True if execute B_0 or A_0 block
    AES_CCM_BLOCK_0_POS     = 4,
    AES_CCM_BLOCK_0_BIT     = (1 << 4),
};

/// Bit Field used to know where we are in AES-CCM execution
///    7     6     5     4     3     2     1     0
/// +-----+-----+-----+-----+-----+-----+-----+-----+
/// |RSVD |Adata|        M'       |        L'       |
/// +-----+-----+-----+-----+-----+-----+-----+-----+
///
enum aes_ccm_b_0_flag
{
    /// bit[7] - RSVD
    AES_CCM_B_0_RSVD_POS        = 7,
    AES_CCM_B_0_RSVD_BIT        = (1 << 7),
    /// bit[6] - Adata
    AES_CCM_B_0_ADATA_POS       = 6,
    AES_CCM_B_0_ADATA_BIT       = (1 << 6),
    /// bit[5:3] - M'
    AES_CCM_B_0_M_PRIME_LSB     = 3,
    AES_CCM_B_0_M_PRIME_MASK    = 0x38,
    /// bit[2:0] - L'
    AES_CCM_B_0_L_PRIME_LSB     = 0,
    AES_CCM_B_0_L_PRIME_MASK    = 0x07,
};


/*
 * TYPES DEFINITION
 *****************************************************************************************
 */

/// Structure definition of the AES CCM algorithm
struct aes_ccm_env
{
    /// AES Environment structure
    struct aes_func_env    aes_env;
    /// Function to call at end of AES-CCM algorithm
    aes_ccm_func_result_cb cb_result;
    /// Encryption key
    const uint8_t         *key;
    /// Additional authenticated data
    const uint8_t         *add_auth_data;
    /// Pointer to the input message to be cipher/decipher
    const uint8_t         *in_message;
    /// Pointer to the output message ==> will contain cipher text or data
    uint8_t               *out_message;

    /// Length of the message
    uint16_t               message_len;
    /// Block Cursor of process data
    uint8_t                cursor;

    /// Nonce information (copied in [2:14] used to ease creation of B_0 and A_i)
    uint8_t                nonce[AES_BLOCK_SIZE];
    /// MAC Result of the CBC-MAC computation
    uint8_t                mac[AES_BLOCK_SIZE];

    /// Additional authenticated data length
    uint8_t                add_auth_data_len;
    /// MIC size
    uint8_t                mic_len;
    /// State of AES-CCM execution bit field (@see enum aes_ccm_state_field)
    uint8_t                state;
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


/**
 ****************************************************************************************
 * @brief Continue AES-CCM execution according to current AES-CCM execution state
 *        This function always request a new AES computation to be performed
 *
 * @param[in] env     Pointer to the AES CCM environment variable
 ****************************************************************************************
 */
__STATIC void aes_ccm_continue(struct aes_ccm_env *env)
{
    uint8_t *aes_val_ptr = NULL;

    // CBC-MAC generation step
    if (GETB(env->state, AES_CCM_MAC_GEN))
    {
        // Create B_0 block - to compute X_1 block
        if (GETB(env->state, AES_CCM_BLOCK_0))
        {
            // Set B_0 Flag field
            env->nonce[15] = 0;
            SETB(env->nonce[15], AES_CCM_B_0_RSVD,    0);
            SETB(env->nonce[15], AES_CCM_B_0_ADATA, (env->add_auth_data_len > 0));
            SETF(env->nonce[15], AES_CCM_B_0_M_PRIME, (env->mic_len - 2) / 2);
            SETF(env->nonce[15], AES_CCM_B_0_L_PRIME, 1); // Length field is 2 bytes (L' = L - 1)
            // Set B_0 Message Length field
            co_write16p(&(env->nonce[0]), env->message_len);

            aes_val_ptr = env->nonce;
        }
        // Create B_i blocks - to compute X_i blocks
        else
        {
            aes_val_ptr = env->mac;

            // B_i blocks generated with authenticated data
            if (GETB(env->state, AES_CCM_AUTH))
            {
                uint16_t offset     = (AES_BLOCK_SIZE * env->cursor);
                uint8_t  block_size = AES_BLOCK_SIZE;
                uint8_t  mac_offset = 2;

                // if computed offset is > 0, take in account the first two bytes used for auth data length
                if (offset > 0)
                {
                    offset     -= 2;
                    mac_offset  = 0;
                }
                else
                {
                    block_size -= 2;
                }

                // Two first bytes contains authenticated data
                if (offset == 0)
                {
                    uint16_t auth_len = co_htons(env->add_auth_data_len);

                    // B_1 = X_1  ^ (len || a[0:14])
                    aes_xor_128_lsb(env->mac, env->mac, (uint8_t *) & (auth_len), 2, 0);
                }

                // Size of Auth data block to XOR
                if (block_size > (env->add_auth_data_len - offset))
                {
                    block_size = (env->add_auth_data_len - offset);
                }

                // B_i = X_i  ^ a
                aes_xor_128_lsb(env->mac, env->mac, env->add_auth_data + offset, block_size, mac_offset);
            }
            // B_i blocks generated with plain data
            else // if (GETB(env->state, AES_CCM_DATA))
            {
                // nothing to do
            }
        }
    }
    else
    {
        uint8_t counter = GETB(env->state, AES_CCM_BLOCK_0) ? 0 : (1 + env->cursor);

        // Set A_i Flag field
        env->nonce[15] = 1; // Flag = L'  ==>  Length field is 2 bytes (L' = L - 1)
        // Set Counter value
        co_write16p(&(env->nonce[0]), counter);

        aes_val_ptr = env->nonce;
    }

    // execute next AES operation
    aes_start(&(env->aes_env), env->key, aes_val_ptr, true);
}

/**
 ****************************************************************************************
 * @brief Process result of AES block to continue AES-CCM execution
 *
 * @param[in] env     Pointer to the AES CCM environment variable
 * @param[in] aes_res Result of AES execution
 *
 * @return    True if AES-CCM is over, False else
 ****************************************************************************************
 */
__STATIC bool aes_ccm_process_e(struct aes_ccm_env *env, uint8_t *aes_res)
{
    bool finished = false;
    uint8_t i;

    // CBC-MAC generation step
    if (GETB(env->state, AES_CCM_MAC_GEN))
    {
        // put result in MAC block
        memcpy(env->mac, aes_res, AES_BLOCK_SIZE);

        // Handle X_1 block
        if (GETB(env->state, AES_CCM_BLOCK_0))
        {
            // B_0 processed
            SETB(env->state, AES_CCM_BLOCK_0, false);

            // check if additional auth data present
            if (env->add_auth_data_len > 0)
            {
                // if yes process it for MAC generation
                SETB(env->state, AES_CCM_AUTH,    true);
                env->cursor = 0;
            }
            // process only the data
            else
            {
                // start data processing
                SETB(env->state, AES_CCM_MAC_GEN, false);
                SETB(env->state, AES_CCM_AUTH,    false);
                SETB(env->state, AES_CCM_DATA,    true);
                env->cursor = 0;
            }
        }
        else if (GETB(env->state, AES_CCM_AUTH))
        {
            env->cursor++;

            // check if there is additional data to process
            if (env->add_auth_data_len < ((env->cursor * AES_BLOCK_SIZE) - 2))
            {
                // start data processing
                SETB(env->state, AES_CCM_MAC_GEN, false);
                SETB(env->state, AES_CCM_AUTH,    false);
                SETB(env->state, AES_CCM_DATA,    true);
                env->cursor = 0;
            }
        }
        // Handle X_i Data block
        else // if (GETB(env->state, AES_CCM_DATA))
        {
            uint16_t offset = (AES_BLOCK_SIZE * env->cursor);

            // next step is handled by Encrypt algorithm
            SETB(env->state, AES_CCM_MAC_GEN, false);

            // Check if all the data has been processed
            if (offset >= env->message_len)
            {
                // compute the A_0 block for MIC
                SETB(env->state, AES_CCM_DATA,    false);
                SETB(env->state, AES_CCM_BLOCK_0, true);
            }
        }
    }
    else
    {
        // Handle S_0 block - MIC
        if (GETB(env->state, AES_CCM_BLOCK_0))
        {
            bool mic_error  = false;

            // Cipher: Generate the MIC and return
            if (GETB(env->state, AES_CCM_CIPHER))
            {
                uint8_t *out_mic = env->out_message + env->message_len;

                // MIC = first_M_bytes (T ^ S_0)
                for (i = 0 ; i < env->mic_len ; i++)
                {
                    out_mic[i] = (env->mac[AES_BLOCK_SIZE - i - 1] ^  aes_res[AES_BLOCK_SIZE - i - 1]);
                }
            }
            // Decipher: Retrieve MIC and Check if value is ok
            else
            {
                const uint8_t *in_mic = env->in_message + env->message_len;

                // MAC= first_M_bytes (T ^ S_0)
                for (i = 0 ; i < env->mic_len ; i++)
                {
                    if (env->mac[AES_BLOCK_SIZE - i - 1] != (in_mic[i] ^  aes_res[AES_BLOCK_SIZE - i - 1]))
                    {
                        mic_error = true;
                        break;
                    }
                }
            }

            if (env->cb_result != NULL)
            {
                env->cb_result(mic_error, env->aes_env.metainfo);
            }

            finished = true;
        }
        // Handle S_i Data block
        else // if (GETB(env->state, AES_CCM_DATA))
        {
            uint16_t       offset  = (AES_BLOCK_SIZE * env->cursor);
            uint8_t        len     = (offset + AES_BLOCK_SIZE > env->message_len)
                                     ? (env->message_len - offset)
                                     : AES_BLOCK_SIZE;
            const uint8_t *in_msg  = env->in_message  + offset;
            uint8_t       *out_msg = env->out_message + offset;

            // execute two XOR to prepare MAC algorithm and execute Encryption/Decryption algorithm
            //
            // +--------------------+---------------------+
            // |       CIPHER       |       DECIPHER      |
            // +--------------------+---------------------+
            // | MAC = IN ^ MAC     | OUT = IN  ^ AES_RES |
            // +--------------------+---------------------+
            // | OUT = IN ^ AES_RES | MAC = OUT ^ MAC     |
            // +--------------------+---------------------+
            //
            if (GETB(env->state, AES_CCM_CIPHER))
            {

                // MAC = IN  ^ MAC
                aes_xor_128_lsb(env->mac, env->mac, in_msg, len, 0);

                // OUT = IN  ^ AES_RES
                aes_xor_128_msb(out_msg, in_msg, aes_res, len);
            }
            else
            {
                // OUT = IN  ^ AES_RES
                aes_xor_128_msb(out_msg, in_msg, aes_res, len);
                // MAC = OUT ^ MAC
                aes_xor_128_lsb(env->mac, env->mac, out_msg, len, 0);
            }

            // next step is handled by MAC generation algorithm
            SETB(env->state, AES_CCM_MAC_GEN, true);
            env->cursor++;
        }
    }

    // check if AES-CCM algorithm must be continued
    if (!finished)
    {
        aes_ccm_continue(env);
    }

    return finished;
}


/*
 * EXPORTED FUNCTION DEFINITIONS
 ****************************************************************************************
 */
void aes_ccm(const uint8_t *key, const uint8_t *nonce, const uint8_t *in_message,
             uint8_t *out_message, uint16_t message_len, uint8_t mic_len, bool cipher,
             const uint8_t *add_auth_data, uint8_t add_auth_data_len, aes_ccm_func_result_cb res_cb, uint32_t metainfo)
{
    struct aes_ccm_env *env;
    int i;
    // Allocate AES CCM environent memory
    env = (struct aes_ccm_env *) aes_alloc(sizeof(struct aes_ccm_env), (aes_func_continue_cb)aes_ccm_process_e,  NULL, metainfo);

    // copy information needed for AES-CCM execution
    env->cb_result         = res_cb;
    env->key               = key;
    env->add_auth_data     = add_auth_data;
    env->in_message        = in_message;
    env->out_message       = out_message;
    env->add_auth_data_len = add_auth_data_len;
    env->message_len       = message_len;
    env->mic_len           = mic_len;
    env->state             = 0;

    // Put NONCE in LSB format
    for (i = 0 ; i < AES_CCM_NONCE_SIZE ; i++)
    {
        env->nonce[2 + i] = nonce[AES_CCM_NONCE_SIZE - i - 1];
    }

    // Initialize execution of the AES-CCM function
    SETB(env->state, AES_CCM_CIPHER,  cipher);
    SETB(env->state, AES_CCM_MAC_GEN, true);
    SETB(env->state, AES_CCM_AUTH,    false);
    SETB(env->state, AES_CCM_DATA,    false);
    SETB(env->state, AES_CCM_BLOCK_0, true);

    // Start first execution step
    aes_ccm_continue(env);
}

/// @} AES_CCM
