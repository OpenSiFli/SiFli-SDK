/**
 ****************************************************************************************
 * *
 * @file aes_cmac.c
 *
 * @brief Definition file for AES CMAC crypto module functions
 *
 * Copyright (C) RivieraWaves 2017-2018
 *
 *
 * Algorithm from RFC 4493:
 *
 *   +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *   +                    Algorithm Generate_Subkey                      +
 *   +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *   +                                                                   +
 *   +   Input    : K (128-bit key)                                      +
 *   +   Output   : K1 (128-bit first subkey)                            +
 *   +              K2 (128-bit second subkey)                           +
 *   +-------------------------------------------------------------------+
 *   +                                                                   +
 *   +   Constants: const_Zero is 0x00000000000000000000000000000000     +
 *   +              const_Rb   is 0x00000000000000000000000000000087     +
 *   +   Variables: L          for output of AES-128 applied to 0^128    +
 *   +                                                                   +
 *   +   Step 1.  L := AES-128(K, const_Zero);                           +
 *   +   Step 2.  if MSB(L) is equal to 0                                +
 *   +            then    K1 := L << 1;                                  +
 *   +            else    K1 := (L << 1) XOR const_Rb;                   +
 *   +   Step 3.  if MSB(K1) is equal to 0                               +
 *   +            then    K2 := K1 << 1;                                 +
 *   +            else    K2 := (K1 << 1) XOR const_Rb;                  +
 *   +   Step 4.  return K1, K2;                                         +
 *   +                                                                   +
 *   +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 *
 *   +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *   +                   Algorithm AES-CMAC                              +
 *   +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *   +                                                                   +
 *   +   Input    : K    ( 128-bit key )                                 +
 *   +            : M    ( message to be authenticated )                 +
 *   +            : len  ( length of the message in octets )             +
 *   +   Output   : T    ( message authentication code )                 +
 *   +                                                                   +
 *   +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *   +                                                                   +
 *   +   Constants: const_Zero is 0x00000000000000000000000000000000     +
 *   +              const_Bsize is 16                                    +
 *   +                                                                   +
 *   +   Variables: K1, K2 for 128-bit subkeys                           +
 *   +              M_i is the i-th block (i=1..ceil(len/const_Bsize))   +
 *   +              M_last is the last block xor-ed with K1 or K2        +
 *   +              n      for number of blocks to be processed          +
 *   +              r      for number of octets of last block            +
 *   +              flag   for denoting if last block is complete or not +
 *   +                                                                   +
 *   +   Step 1.  (K1,K2) := Generate_Subkey(K);                         +
 *   +   Step 2.  n := ceil(len/const_Bsize);                            +
 *   +   Step 3.  if n = 0                                               +
 *   +            then                                                   +
 *   +                 n := 1;                                           +
 *   +                 flag := false;                                    +
 *   +            else                                                   +
 *   +                 if len mod const_Bsize is 0                       +
 *   +                 then flag := true;                                +
 *   +                 else flag := false;                               +
 *   +                                                                   +
 *   +   Step 4.  if flag is true                                        +
 *   +            then M_last := M_n XOR K1;                             +
 *   +            else M_last := padding(M_n) XOR K2;                    +
 *   +   Step 5.  X := const_Zero;                                       +
 *   +   Step 6.  for i := 1 to n-1 do                                   +
 *   +                begin                                              +
 *   +                  Y := X XOR M_i;                                  +
 *   +                  X := AES-128(K,Y);                               +
 *   +                end                                                +
 *   +            Y := M_last XOR X;                                     +
 *   +            T := AES-128(K,Y);                                     +
 *   +   Step 7.  return T;                                              +
 *   +                                                                   +
 *   +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
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

#if (BLE_EMB_PRESENT || BLE_HOST_PRESENT)

#include "arch.h"     // architecture defines
#include "aes_int.h"  // AES internals
#include <string.h>   // for memcpy function
#include "co_math.h"

/*
 * DEFINES
 ****************************************************************************************
 */
// const_Rb   is 0x00000000000000000000000000000087
#define AES_CMAC_CONST_RB   0x87
// Get MSB bit a a 128 bits number
#define AES_CMAC_MSB(num)   (num[AES_BLOCK_SIZE-1] >> 7)

#define AES_CMAC_GEN_KEY_START       (0xFE)
#define AES_CMAC_GEN_KEY_FINISH      (0xFF)

/*
 * TYPES DEFINITION
 *****************************************************************************************
 */


/*
 * GLOBAL VARIABLE DEFINITION
 *****************************************************************************************
 */
/// const_Zero
const uint8_t aes_cmac_zero[AES_BLOCK_SIZE] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};


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


void aes_cmac(const uint8_t *key, const uint8_t *message, uint16_t message_len,
              aes_func_result_cb res_cb, uint32_t metainfo)
{
    struct aes_cmac_env *aes_cmac;

    // Allocate AES CMAC environent memory
    aes_cmac = (struct aes_cmac_env *) aes_alloc(sizeof(struct aes_cmac_env),
               (aes_func_continue_cb) aes_cmac_continue, res_cb, metainfo);

    // start execution AES CMAC
    aes_cmac_start(aes_cmac, key, message, message_len);
}

void aes_cmac_start(struct aes_cmac_env *env, const uint8_t *key, const uint8_t *message, uint16_t message_len)
{
    // copy information needed for AES-CMAC execution
    env->message     = message;
    env->message_len = message_len;
    env->auth_key    = key;

    // initialize execution parameters
    // num_blocks = ceil(message_len / block_size)
    env->num_blocks = (env->message_len + AES_BLOCK_SIZE - 1) / AES_BLOCK_SIZE;
    env->cur_block  = 1;

    // Initialize authentication block with zeros (const_Zero)
    memcpy(env->auth_code, aes_cmac_zero, AES_BLOCK_SIZE);

    // start execution of the AES-CMAC computation
    aes_cmac_continue(env, env->auth_code);
}

bool aes_cmac_continue(struct aes_cmac_env *env, uint8_t *aes_res)
{
    bool finished = false;

    //  To optimize the AES-CMAC algorithm, SW is implemented as follow:
    //
    //  +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    //  +   Step 1.  n := ceil(len/block_size); (already done in init)      +
    //  +            X := const_Zero;                                       +
    //  +                                                                   +
    //  +   Step 2.                                                         +
    //  +            for i = 1 , i < n , i++ do                             +
    //  +                begin                                              +
    //  +                  Y := X XOR M_i;                                  +
    //  +                  X := AES-128(K,Y);                               +
    //  +                end                                                +
    //  +                                                                   +
    //  +   Step 3.  len = len mod block_size;                              +
    //  +            flag := true if (len == 0) && (n != 0);                +
    //  +                                                                   +
    //  +   Step 4.  Generate_Subkey(K):                                    +
    //  +                                                                   +
    //  +            L := AES-128(K, const_Zero);                           +
    //  +            subkey := (L << 1);                                    +
    //  +            if MSB(L) is equal to 1                                +
    //  +            then    subkey := subkey XOR const_Rb;                 +
    //  +                                                                   +
    //  +            if flag is true                                        +
    //  +            then    if MSB(subkey) is equal 1                      +
    //  +                    then    subkey := (subkey << 1)                +
    //  +                    else    subkey := (subkey << 1) XOR const_Rb;  +
    //  +                                                                   +
    //  +   Step 5.  if flag is true                                        +
    //  +            then M_last := M_n                                     +
    //  +            else M_last := padding(M_n)                            +
    //  +                                                                   +
    //  +            M_last := M_last XOR subkey;                           +
    //  +                                                                   +
    //  +            Y := M_last XOR X;                                     +
    //  +            T := AES-128(K,Y);                                     +
    //  +                                                                   +
    //  +   Step 6.  return T;                                              +
    //  +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    // Step 2: handle Blocks M_1 ==> M_(n-1)
    if (env->cur_block < env->num_blocks)
    {
        const uint8_t *m_i = &(env->message[env->message_len - (env->cur_block * AES_BLOCK_SIZE)]);

        // Y := X XOR M_i
        aes_xor_128(env->auth_code, aes_res, m_i, AES_BLOCK_SIZE);

        // X := AES-128(K,Y)
        aes_start(&(env->aes_env), env->auth_key, env->auth_code, true);

        env->cur_block++;
    }
    else
    {
        // Step 3: Generate_Subkey(K):
        if (env->cur_block < AES_CMAC_GEN_KEY_START)
        {
            env->cur_block = AES_CMAC_GEN_KEY_START;
            // Save latest value of X
            memcpy(env->auth_code, aes_res, AES_BLOCK_SIZE);

            // L := AES-128(K, const_Zero);
            aes_start(&(env->aes_env), env->auth_key, aes_cmac_zero, true);
        }
        // Step 3: Generate_Subkey(K): continue
        else if (env->cur_block < AES_CMAC_GEN_KEY_FINISH)
        {
            env->cur_block = AES_CMAC_GEN_KEY_FINISH;
            // compute block remaining length
            uint8_t remain_len = CO_MOD(env->message_len, AES_BLOCK_SIZE);
            uint8_t sub_key[AES_BLOCK_SIZE];
            uint8_t m_last[AES_BLOCK_SIZE];

            uint8_t msb = AES_CMAC_MSB(aes_res);

            // sub_key := (L << 1)
            aes_shift_left_128(aes_res, sub_key);

            if (msb == 1)
            {
                // sub_key XOR const_Rb
                sub_key[0] = sub_key[0] ^ AES_CMAC_CONST_RB;
            }

            // compute Flag to know if sub_key equals K2
            if ((remain_len != 0) || (env->num_blocks == 0))
            {
                uint8_t msb = AES_CMAC_MSB(sub_key);

                // sub_key := (sub_key << 1)
                aes_shift_left_128(sub_key, sub_key);

                if (msb == 1)
                {
                    // sub_key XOR const_Rb
                    sub_key[0] = sub_key[0] ^ AES_CMAC_CONST_RB;
                }

                // M_last := padding(M_n)

                /* NOTE :- Padding performed is ISO/IEC 7816-4 compliant.
                 *
                 *  This means in practice that the first byte is a mandatory byte valued '80' (Hexadecimal) followed,
                 * if needed, by 0 to N-1 bytes set to '00', until the end of the block is reached.
                 */
                for (uint8_t i = 0; i < AES_BLOCK_SIZE ; i++)
                {
                    if (i < (remain_len))
                    {
                        m_last[(AES_BLOCK_SIZE - remain_len) + i] = env->message[i];
                    }
                    else if (i == remain_len)
                    {
                        m_last[AES_BLOCK_SIZE - 1 - remain_len]    = 0x80;
                    }
                    else
                    {
                        m_last[AES_BLOCK_SIZE - 1 - i]             = 0x00;
                    }
                }
            }
            else
            {
                memcpy(m_last, env->message, AES_BLOCK_SIZE);
            }
            // Step 5

            // M_last := M_last XOR subkey
            aes_xor_128(m_last, m_last, sub_key, AES_BLOCK_SIZE);

            // Y := M_last XOR X
            aes_xor_128(env->auth_code, m_last, env->auth_code, AES_BLOCK_SIZE);

            // T := AES-128(K,Y)
            aes_start(&(env->aes_env), env->auth_key, env->auth_code, true);
        }
        // Step 6 Return result
        else
        {
            // mark AES-CMAC over
            finished = true;
        }
    }

    return finished;
}

#endif // (BLE_EMB_PRESENT || BLE_HOST_PRESENT)

/// @} AES_CMAC
