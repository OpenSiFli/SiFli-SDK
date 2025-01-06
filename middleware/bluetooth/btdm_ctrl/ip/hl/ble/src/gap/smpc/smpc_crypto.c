/*
 * smpc_crypto.c
 *
 *  Created on: May 12, 2015
 *      Author: garyf
 */

#include "rwip_config.h"
#if (BLE_SMPC)

#include "co_utils.h"
#include "co_endian.h"

#include "smpc_int.h"
#include "smpc_crypto.h"
#include "smpc_api.h"

#include "gapm.h"
#include "gapc.h"
#include "../gapc/gapc_int.h"  // Internal are required

#include "ke_mem.h"
#include <string.h>

#if (SECURE_CONNECTIONS)
void smpc_XOR_16_Byte_Array(uint8_t *a, uint8_t *b, uint8_t *result);
void smpc_ShiftLeft1Bit16ByteNum(uint8_t *input, uint8_t *output);

void smpc_f4_Init(uint8_t conidx, uint8_t *U, uint8_t *V, uint8_t *X, uint8_t Z)
{
    // This Kicks off the f4 Crypto function
    // All inputs are LSO first.

    // U is 256bit - 32 bytes
    // V is 256bit - 32 bytes
    // X is 128bit - 16 bytes
    // Z is 8 bits -  1 byte

    ASSERT_ERR(gapc_env[conidx]->smpc.info.pair != NULL);
    // Allocate structure for f4 - this will be released on f4 completion
    struct smp_f4 *p_f4 = (struct smp_f4 *)ke_malloc(sizeof(struct smp_f4), KE_MEM_KE_MSG);
    gapc_env[conidx]->smpc.info.pair->f4_info = p_f4;


    // All Elements are LSO first.
    memcpy((void *)&p_f4->X[0], (void *)&X[0], 16);

    p_f4->M[0] = Z;
    memcpy((void *)&p_f4->M[1], (void *)&V[0], 32);
    memcpy((void *)&p_f4->M[33], (void *)&U[0], 32);

    // Kick off the AES_CMAC
    smpc_aes_cmac_init(conidx, p_f4->X, p_f4->M, 65);

    // Wait for AES-CMAC to complete
}

void smpc_f4_complete(uint8_t conidx)
{
    // Free the memory previously allocated for AES_CMAC
    ke_free(gapc_env[conidx]->smpc.info.pair->aes_cmac);
    gapc_env[conidx]->smpc.info.pair->aes_cmac = NULL;
    // Free The memory previously allocated for f4
    ke_free(gapc_env[conidx]->smpc.info.pair->f4_info);
    gapc_env[conidx]->smpc.info.pair->f4_info = NULL;
}

void smpc_g2_init(uint8_t conidx, uint8_t *U, uint8_t *V, uint8_t *X, uint8_t *Y)
{
    // U is 256bit - 32 bytes
    // V is 256bit - 32 bytes
    // X is 128bit - 16 bytes
    // Y is 128bit - 16 bytes

    // All inputs are LSO first.

    // M = U || V || Y
    // Mlsc = Y || V || U
    // Key = X
    ASSERT_ERR(gapc_env[conidx]->smpc.info.pair != NULL);
    // Allocate structure for g2 - this will be released on f4 completion
    struct smp_g2 *p_g2 = (struct smp_g2 *)ke_malloc(sizeof(struct smp_g2), KE_MEM_KE_MSG);
    gapc_env[conidx]->smpc.info.pair->g2_info = p_g2;

    memcpy((void *)&p_g2->X[0], (void *)&X[0], 16);

    memcpy((void *)&p_g2->M[48], (void *)&U[0], 32);
    memcpy((void *)&p_g2->M[16], (void *)&V[0], 32);
    memcpy((void *)&p_g2->M[0], (void *)&Y[0], 16);

    smpc_aes_cmac_init(conidx, p_g2->X, p_g2->M, 80);
}

void smpc_g2_complete(uint8_t conidx)
{
    // Free the memory previously allocated for AES_CMAC
    ke_free(gapc_env[conidx]->smpc.info.pair->aes_cmac);
    gapc_env[conidx]->smpc.info.pair->aes_cmac = NULL;

    // Free The memory previously allocated for f4
    ke_free(gapc_env[conidx]->smpc.info.pair->g2_info);
    gapc_env[conidx]->smpc.info.pair->g2_info = NULL;
}

void smpc_f5_init(uint8_t conidx, uint8_t *W, uint8_t *N1, uint8_t *N2, uint8_t *A1, uint8_t *A2)
{
    // W  is 256 bits - 32 bytes
    // N1 is 128 bits - 16 bytes
    // N2 is 128 bits - 16 bytes
    // A1 is  56 bits -  7 bytes
    // A2 is  56 bits -  7 bytes

    // All input are LSO first !

    // LSO First
    uint8_t KeyId[4] = {0x65, 0x6c, 0x74, 0x62};
    uint8_t Salt[16] = {0xBE, 0x83, 0x60, 0x5A, 0xDB, 0x0B, 0x37, 0x60, 0x38, 0xA5, 0xF5, 0xAA, 0x91, 0x83, 0x88, 0x6C};
    ASSERT_ERR(gapc_env[conidx]->smpc.info.pair != NULL);
    // Allocate structure for f5 - this will be released on f5 completion
    struct smp_f5 *p_f5 = (struct smp_f5 *)ke_malloc(sizeof(struct smp_f5), KE_MEM_KE_MSG);
    gapc_env[conidx]->smpc.info.pair->f5_info = p_f5;

    // Setup SALT & KeyId
    memcpy((void *)&p_f5->SALT[0], Salt, 16);
    // Assign pointer for W - Note - W is the Dh_key which is already stored.
    p_f5->W = W;

    // Setup M - LSO first

    // Length field is 256 - 0x0100
    p_f5->M[0] = 0x00;
    p_f5->M[1] = 0x01;
    memcpy((void *)&p_f5->M[2], (void *)A2, 7);
    memcpy((void *)&p_f5->M[9], (void *)A1, 7);
    memcpy((void *)&p_f5->M[16], (void *)N2, 16);
    memcpy((void *)&p_f5->M[32], (void *)N1, 16);
    memcpy((void *)&p_f5->M[48], (void *)KeyId, 4);
    p_f5->M[52] = 0x00; // Counter = 0x00 for MacKey

    smpc_aes_cmac_init(conidx, p_f5->SALT, p_f5->W, 32);
}

void smpc_f5_complete(uint8_t conidx)
{
    // Free the memory previously allocated for AES_CMAC
    if (gapc_env[conidx]->smpc.info.pair->aes_cmac != NULL)
        ke_free(gapc_env[conidx]->smpc.info.pair->aes_cmac);
    gapc_env[conidx]->smpc.info.pair->aes_cmac = NULL;

    // Free The memory previously allocated for f5
    ke_free(gapc_env[conidx]->smpc.info.pair->f5_info);
    gapc_env[conidx]->smpc.info.pair->f5_info = NULL;
}

void smpc_f6_init(uint8_t conidx, uint8_t *W, uint8_t *N1, uint8_t *N2, uint8_t *R, uint8_t *IOcap, uint8_t *A1, uint8_t *A2)
{
    // All inputs are LSO first

    // W  is 128 bits   -- 16 bytes
    // N1 is 128 bits   -- 16 bytes
    // N2 is 128 bits   -- 16 bytes
    // R  is 128 bits   -- 16 bytes
    // IOcap is 24 bits --  3 bytes
    // A1 is 56 bits    --  7 bytes
    // A2 is 56 bits    --  7 bytes
    ASSERT_ERR(gapc_env[conidx]->smpc.info.pair != NULL);
    // Allocate structure for f6 - this will be released on f6 completion
    struct smp_f6 *p_f6 = (struct smp_f6 *)ke_malloc(sizeof(struct smp_f6), KE_MEM_KE_MSG);
    gapc_env[conidx]->smpc.info.pair->f6_info = p_f6;

    memcpy((void *)&p_f6->W[0], (void *)&W[0], 16);

    memcpy((void *)&p_f6->M[49], (void *)N1, 16);
    memcpy((void *)&p_f6->M[33], (void *)N2, 16);
    memcpy((void *)&p_f6->M[17], (void *)R, 16);
    memcpy((void *)&p_f6->M[14], (void *)IOcap, 3);
    memcpy((void *)&p_f6->M[7], (void *)A1, 7);
    memcpy((void *)&p_f6->M[0], (void *)A2, 7);

    smpc_aes_cmac_init(conidx, p_f6->W, p_f6->M, 65);
}

void smpc_f6_complete(uint8_t conidx)
{
    // Free the memory previously allocated for AES_CMAC
    ke_free(gapc_env[conidx]->smpc.info.pair->aes_cmac);
    gapc_env[conidx]->smpc.info.pair->aes_cmac = NULL;

    // Free The memory previously allocated for f6
    ke_free(gapc_env[conidx]->smpc.info.pair->f6_info);
    gapc_env[conidx]->smpc.info.pair->f6_info = NULL;
}

void smpc_aes_cmac_init(uint8_t conidx, uint8_t *K, uint8_t *M, uint8_t M_len)
{
    uint8_t Const_Zero[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    struct smp_aes_cmac *p_aes_cmac;

    ASSERT_ERR(gapc_env[conidx]->smpc.info.pair != NULL);
    p_aes_cmac = (struct smp_aes_cmac *)ke_malloc(sizeof(struct smp_aes_cmac), KE_MEM_KE_MSG);
    gapc_env[conidx]->smpc.info.pair->aes_cmac = p_aes_cmac;

    // Save the Key - K and the block's M
    p_aes_cmac->M = &M[0];
    p_aes_cmac->K = &K[0];
    p_aes_cmac->next_block = 0;
    p_aes_cmac->M_len = M_len;
    // Step 1 - Generate Subkeys
    p_aes_cmac->state = SMP_AES_CMAC_KEY_GENERATION;
    // First Step is to Generate L = AES-128(K,Const_Zero)

    smpc_send_use_enc_block_cmd(conidx, K, Const_Zero);

}

void smpc_aes_cmac_continue(uint8_t conidx, uint8_t *aes_res)
{
    struct smp_aes_cmac *p_aes_cmac;
    ASSERT_ERR(gapc_env[conidx]->smpc.info.pair != NULL);
    p_aes_cmac = gapc_env[conidx]->smpc.info.pair->aes_cmac;

    // Determine if more calls on AES are required to complete the AES_CMAC
    // Remember the block count starts at Zero.
    if (p_aes_cmac->state == SMP_AES_CMAC_KEY_GENERATION)
    {
        uint8_t n = 0;
        uint8_t flag = 0;
        //*******************************************************
        // Complete the SUBKEY generation using the AES Result
        //*******************************************************

        if ((aes_res[15] & 0x80) == 0)
        {
            // The K1 = L << 1
            smpc_ShiftLeft1Bit16ByteNum(aes_res, p_aes_cmac->K1);
        }
        else
        {
            // The K1 = L << 1 XOR const_Rb
            smpc_ShiftLeft1Bit16ByteNum(aes_res, p_aes_cmac->K1);
            p_aes_cmac->K1[0] = p_aes_cmac->K1[0] ^ 0x87;
        }

        if ((p_aes_cmac->K1[15] & 0x80) == 0)
        {
            // The K2 = K1 << 1
            smpc_ShiftLeft1Bit16ByteNum(p_aes_cmac->K1, p_aes_cmac->K2);
        }
        else
        {
            // The K2 = K1 << 1 XOR const_Rb
            smpc_ShiftLeft1Bit16ByteNum(p_aes_cmac->K1, p_aes_cmac->K2);
            p_aes_cmac->K2[0] = p_aes_cmac->K2[0] ^ 0x87;
        }
        //*****************************************************
        // SubKey Generation complete.
        //*****************************************************

        //*****************************************************
        // 1. Calculate the Number of Blocks.
        // 2. Determine Mlast - Padding if required
        //*****************************************************

        // Calculate ceil(len/const_Bsize) to get number of blocks
        if (p_aes_cmac->M_len != 0)
        {
            n = (p_aes_cmac->M_len + 15) / 16;
        }
        // Step 3
        if (n == 0)
        {
            n = 1;
        }
        else if ((p_aes_cmac->M_len % 16) == 0)
        {
            flag = 0x01;
        }

        // Step 4
        if (flag)
        {
            // M_last = M_n XOR K1
            uint8_t *pM_n =  &p_aes_cmac->M[0];
            smpc_XOR_16_Byte_Array(pM_n, p_aes_cmac->K1, p_aes_cmac->M_last);
        }
        else
        {

            /* NOTE :- Padding performed is ISO/IEC 7816-4 compliant.
             *
             *  This means in practice that the first byte is a mandatory byte valued '80' (Hexadecimal) followed,
             * if needed, by 0 to N-1 bytes set to '00', until the end of the block is reached.
             */
            // M_last = padding(M_n) XOR K2
            uint8_t  padded_Mn[16];
            {
                uint8_t *pMn = &p_aes_cmac->M[0];
                uint8_t length = p_aes_cmac->M_len % 16;
                uint8_t i;

                for (i = 0; i < 16; i++)
                {
                    if (i < (length))
                    {
                        padded_Mn[(16 - length) + i] = pMn[i];
                    }
                    else if (i == length)
                    {
                        padded_Mn[15 - length] = 0x80;
                    }
                    else
                    {
                        padded_Mn[15 - i] = 0x00;
                    }
                }
            }
            smpc_XOR_16_Byte_Array(padded_Mn, p_aes_cmac->K2, p_aes_cmac->M_last);
        }
        // Ensure the AES_CMAC structure is populated.
        p_aes_cmac->num_blocks = n;
        memset(p_aes_cmac->X, 0, 16);

        // Perform AES of the first block
        p_aes_cmac->next_block = 1;
        p_aes_cmac->state = SMP_AES_CMAC_BLOCK;
        smpc_XOR_16_Byte_Array(&p_aes_cmac->M[p_aes_cmac->M_len - (p_aes_cmac->next_block * 16)], p_aes_cmac->X, p_aes_cmac->Y);

        // Assume smpc_send_use_enc_block_cmd(Key,PlainText);
        smpc_send_use_enc_block_cmd(conidx, p_aes_cmac->K, p_aes_cmac->Y);
        // Result needs to be copied to p_aes_cmac->X
    }
    else if (p_aes_cmac->state == SMP_AES_CMAC_BLOCK) // Not part of Subkey Generation
    {
        memcpy(p_aes_cmac->X, aes_res, 16);
        p_aes_cmac->next_block++;
        smpc_XOR_16_Byte_Array(&p_aes_cmac->M[p_aes_cmac->M_len - (p_aes_cmac->next_block * 16)], p_aes_cmac->X, p_aes_cmac->Y);
        // Assume smpc_send_use_enc_block_cmd(Key,PlainText);
        smpc_send_use_enc_block_cmd(conidx, p_aes_cmac->K, p_aes_cmac->Y);
        // Result needs to be copied to p_aes_cmac->X
    }
}

void smpc_aes_cmac_complete(uint8_t conIdx, uint8_t *aes_res)
{
    // Memory will be freed after the AES is complete
    struct smp_aes_cmac *p_aes_cmac;

    p_aes_cmac = gapc_env[conIdx]->smpc.info.pair->aes_cmac;
    memcpy(p_aes_cmac->X, aes_res, 16);
    smpc_XOR_16_Byte_Array(&p_aes_cmac->M_last[0], p_aes_cmac->X, p_aes_cmac->Y);
    // Assume smpc_send_use_enc_block_cmd(Key,PlainText);
    smpc_send_use_enc_block_cmd(conIdx, p_aes_cmac->K, p_aes_cmac->Y);
    // Result needs to be copied to p_aes_cmac->X
    p_aes_cmac->next_block++;
}


bool smpc_process_aes_cmac(uint8_t idx, uint8_t *aes_res)
{
    ASSERT_ERR(gapc_env[idx]->smpc.info.pair != NULL);
    struct smp_aes_cmac *p_aes_cmac = gapc_env[idx]->smpc.info.pair->aes_cmac;
    bool aes_cmac_complete = false;

    if (p_aes_cmac->state == SMP_AES_CMAC_KEY_GENERATION)
    {
        // We have done AES for the subkeys. Next step is to complete the subkeys and
        // move onto AES of the first block.
        smpc_aes_cmac_continue(idx, aes_res);
    }
    else
    {
        if (p_aes_cmac->next_block == p_aes_cmac->num_blocks)
        {
            // AES_CMAC complete ! Finish F4
            aes_cmac_complete = true;
        }
        else if (p_aes_cmac->next_block < (p_aes_cmac->num_blocks - 1))
        {
            smpc_aes_cmac_continue(idx, aes_res);
        }
        else if (p_aes_cmac->next_block == (p_aes_cmac->num_blocks - 1))
        {
            smpc_aes_cmac_complete(idx, aes_res);
        }
    }
    return aes_cmac_complete;
}


void smpc_XOR_16_Byte_Array(uint8_t *a, uint8_t *b, uint8_t *result)
{
    uint8_t i;

    for (i = 0; i < 16; i++)
    {
        result[i] = a[i] ^ b[i];
    }
}



void smpc_ShiftLeft1Bit16ByteNum(uint8_t *input, uint8_t *output)
{
    uint8_t i;

    for (i = 15 ; i >= 1; i--)
    {
        output[i] = (input[i] << 1) | (input[i - 1] >> 7);
    }

    output[0] = (input[0] << 1);
}


#endif // (SECURE_CONNECTIONS)


#endif // (BLE_SMPC)
