/**
  ******************************************************************************
  * @file   efuse.c
  * @author Sifli software development team
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2019 - 2022,  Sifli Technology
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Sifli integrated circuit
 *    in a product or a software update for such product, must reproduce the above
 *    copyright notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of Sifli nor the names of its contributors may be used to endorse
 *    or promote products derived from this software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Sifli integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY SIFLI TECHNOLOGY "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL SIFLI TECHNOLOGY OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <string.h>
#include <rtthread.h>
#include "dfu.h"
#include "board.h"
#ifndef BSP_USING_HW_AES
    #include "mbedtls/aes.h"
#endif
#include "mbedtls/cipher.h"
#include "mbedtls/pk.h"
#include "mbedtls/sha256.h"

ALIGN(4)
uint8_t g_uid[DFU_UID_SIZE];
uint8_t root_key[DFU_KEY_SIZE];
uint8_t g_sig_hash[DFU_SIG_HASH_SIZE];
dfu_efuse_read_hook_t g_dfu_efuse_read_hook;

#ifdef HW_EFUSE
#define EFUSE_OFFSET_UID        0
#define EFUSE_OFFSET_SIG_HASH   128
#define EFUSE_OFFSET_SECURE     192
#define EFUSE_OFFSET_ROOT       768
#define EFUSE_BANK_SIZE         32
#define EFUSE_BANK_NUM          (4)

void init_dfu_efuse_read_hook()
{
    g_dfu_efuse_read_hook = NULL;
}

int sifli_hw_efuse_read(uint8_t id, uint8_t *data, int size)
{
    int r;
    if (g_dfu_efuse_read_hook)
    {
        r = g_dfu_efuse_read_hook(id, data, size);
        if (r != 0)
            return r;
    }

    if (id == EFUSE_UID)
        r = HAL_EFUSE_Read(EFUSE_OFFSET_UID, data, DFU_UID_SIZE);
    else if (id == EFUSE_ID_SIG_HASH)
        r = HAL_EFUSE_Read(EFUSE_OFFSET_SIG_HASH, data, DFU_SIG_HASH_SIZE);
    else if (id == EFUSE_ID_ROOT)
        r = HAL_EFUSE_Read(EFUSE_OFFSET_ROOT, data, DFU_KEY_SIZE);
    else if (id == EFUSE_ID_SECURE_ENABLED)
    {
        uint32_t temp_data;
        r = HAL_EFUSE_Read(EFUSE_OFFSET_SECURE, (uint8_t *)&temp_data, 4);
        if (r == 4)
        {
            *data = (uint8_t)(temp_data & 0xFF);
            r = DFU_SECURE_SIZE;
        }
        else
            r = 0;
    }
    else
        r = 0;
    return r;

}


int sifli_hw_efuse_read_init(void)
{
    //HAL_StatusTypeDef ret= HAL_EFUSE_Init();
    //return ret == HAL_OK ? 0 : -1;
    // expected PCLK is 12M and calculate as 18M, while EFUSE_PGM_THPCK_NS (20) , EFUSE_PGM_TCKHP_US  (10), EFUSE_RD_TIM_NS    (500)
    hwp_efusec->TIMR = 0x2D08F;
    return 0;
}


int sifli_hw_efuse_read_all(void)
{
    uint32_t i;
    int32_t r = 0;
    int size;
    static uint8_t data[EFUSE_BANK_SIZE];

    // Only read bank0 and bank3 in A0.
    for (i = 0; i < EFUSE_BANK_NUM; i += 3)
    {
        size = HAL_EFUSE_Read(EFUSE_BANK_SIZE * i * 8, data, EFUSE_BANK_SIZE);
        if (0 == size)
        {
            r = -1;
            break;
        }
    }

    return r;
}

int sifli_hw_efuse_read_bank(uint32_t i)
{
    uint8_t data[EFUSE_BANK_SIZE];
    int32_t r = 0;
    int size;
    if (i >= EFUSE_BANK_NUM)
        return -1;

    size = HAL_EFUSE_Read(EFUSE_BANK_SIZE * i * 8, data, EFUSE_BANK_SIZE);;
    if (size == 0)
        r = -2;

    return r;
}

int sifli_hw_efuse_write(uint8_t id, uint8_t *data, int size)
{
    int r = 0;
    switch (id)
    {
    case EFUSE_UID:
        r = HAL_EFUSE_Write(EFUSE_OFFSET_UID, data, size);
        break;
    case EFUSE_ID_SIG_HASH:
        size = size < DFU_SIG_HASH_SIZE ? size : DFU_SIG_HASH_SIZE;
        r = HAL_EFUSE_Write(EFUSE_OFFSET_SIG_HASH, data, size);
        break;
    case EFUSE_ID_ROOT:
        r = HAL_EFUSE_Write(EFUSE_OFFSET_ROOT, data, size);
        break;
    case EFUSE_ID_SECURE_ENABLED:
    {
        uint32_t temp_data = 0;
        temp_data = (uint32_t)(*data);
        r = HAL_EFUSE_Write(EFUSE_OFFSET_SECURE, (uint8_t *)&temp_data, 4);
        if (r == 4)
            r = 1;
        else
            r = 0;
    }
    break;
    default:
        r = 0;
    }
    return r;
}

#else
int sifli_hw_efuse_read(uint8_t id, uint8_t *data, int size)
{
    int r = size;
    if (id == EFUSE_UID)
        sec_flash_read(DFU_FLASH_SEC_CONFIG, SECFG_UID_OFFSET, data, size);
    else if (id == EFUSE_ID_ROOT)
        sec_flash_read(DFU_FLASH_SEC_CONFIG, SECFG_ROOTKEY_OFFSET, data, size);
    else if (id == EFUSE_ID_SIG_HASH)
        sec_flash_read(DFU_FLASH_SEC_CONFIG, SECFG_SIG_HASH_OFFSET, data, size);
    else
        r = 0;
    return r;
}

int sifli_hw_efuse_write(uint8_t id, uint8_t *data, int size)
{
    int r = size;
    if (id == EFUSE_UID)
        sec_flash_write(DFU_FLASH_SEC_CONFIG, SECFG_UID_OFFSET, data, size);
    else if (id == EFUSE_ID_ROOT)
        sec_flash_write(DFU_FLASH_SEC_CONFIG, SECFG_ROOTKEY_OFFSET, data, size);
    else if (id == EFUSE_ID_SIG_HASH)
        sec_flash_write(DFU_FLASH_SEC_CONFIG, SECFG_SIG_HASH_OFFSET, data, size);
    else
        r = 0;
    return r;
}
#endif


int sifli_hw_dec(uint8_t *key, uint8_t *in_data, uint8_t *out_data, int size, uint32_t init_offset)
{
    uint32_t offset = 0;

    if (g_dfu_efuse_read_hook && !key)
    {
        g_dfu_efuse_read_hook(EFUSE_ID_ROOT, root_key, DFU_KEY_SIZE);
        key = &root_key[0];
    }

#ifdef BSP_USING_HW_AES
    {
#define AES_BLOCK_SIZE 512
        static uint8_t temp[AES_BLOCK_SIZE];
        memset(temp, 0, AES_BLOCK_SIZE);
        while (offset < size)
        {
            int len = (size - offset) < AES_BLOCK_SIZE ? (size - offset) : AES_BLOCK_SIZE;
            memcpy(temp, in_data + offset, len);
            HAL_AES_init((uint32_t *)key, DFU_KEY_SIZE, (uint32_t *)dfu_get_counter(init_offset + offset), AES_MODE_CTR);
            HAL_AES_run(AES_DEC, temp, out_data + offset, len);
            offset += len;
        }
    }
#else
    {
        static mbedtls_aes_context ctx;
        static uint8_t stream_block[16];
        mbedtls_aes_init(&ctx);
        mbedtls_aes_setkey_enc(&ctx, key, DFU_KEY_SIZE * 8);
        mbedtls_aes_crypt_ctr(&ctx, size, &offset, dfu_get_counter(init_offset + offset), stream_block,
                              in_data, out_data);
    }
#endif
    return offset;
}

void sifli_hw_init_xip_key(uint8_t *enc_img_key)
{
    uint8_t *uid;

    /* enable dedicated mode for image key decryption */
    static uint32_t plain_key[DFU_KEY_SIZE >> 2];

    __HAL_SYSCFG_SET_SECURITY();
    uid = &g_uid[0];
    sifli_hw_efuse_read(EFUSE_UID, uid, DFU_UID_SIZE);
    memset(plain_key, 0, sizeof(plain_key));
    HAL_AES_init(NULL, DFU_KEY_SIZE, (uint32_t *)uid, AES_MODE_CBC);
    HAL_AES_run(AES_DEC, enc_img_key, (uint8_t *)plain_key, DFU_KEY_SIZE);
    /* restore to normal mode */
    __HAL_SYSCFG_CLEAR_SECURITY();
}

int sifli_hw_dec_key(uint8_t *in_data, uint8_t *out_data, int size)
{
    uint8_t *uid;
    uint8_t *key = NULL;

    if (size != DFU_KEY_SIZE)
    {
        return -1;
    }
    if (g_dfu_efuse_read_hook)
    {
        g_dfu_efuse_read_hook(EFUSE_ID_ROOT, root_key, DFU_KEY_SIZE);
        key = &root_key[0];
    }

    uid = &g_uid[0];
    sifli_hw_efuse_read(EFUSE_UID, uid, DFU_UID_SIZE);
    HAL_AES_init((uint32_t *)key, DFU_KEY_SIZE, (uint32_t *)uid, AES_MODE_CBC);
    HAL_AES_run(AES_DEC, in_data, out_data, DFU_KEY_SIZE);
#if 0
    uint8_t *key;
    key = &root_key[0];
    sifli_hw_efuse_read(EFUSE_ID_ROOT, key, DFU_KEY_SIZE);


    static mbedtls_aes_context ctx;
    static uint8_t stream_block[16];
    mbedtls_aes_init(&ctx);
    mbedtls_aes_setkey_dec(&ctx, key, DFU_KEY_SIZE * 8);

    mbedtls_aes_crypt_cbc(&ctx, MBEDTLS_AES_DECRYPT, size, &uid[0], in_data, out_data);
#endif


    return 0;
}


void sifli_hw_enc(uint8_t *in_data, uint8_t *out_data, int size)
{
    uint8_t *key = NULL;

    if (g_dfu_efuse_read_hook)
    {
        g_dfu_efuse_read_hook(EFUSE_ID_ROOT, root_key, DFU_KEY_SIZE);
        key = &root_key[0];
    }

#ifdef BSP_USING_HW_AES
    uint8_t *uid;

    uid = &g_uid[0];
    sifli_hw_efuse_read(EFUSE_UID, uid, DFU_UID_SIZE);
    HAL_AES_init((uint32_t *)key, DFU_KEY_SIZE, (uint32_t *)uid, AES_MODE_CBC);
    HAL_AES_run(AES_ENC, in_data, out_data, DFU_KEY_SIZE);
#else

    uint8_t *uid;

    key = &root_key[0];
    sifli_hw_efuse_read(EFUSE_ID_ROOT, key, DFU_KEY_SIZE);
    uid = &g_uid[0];
    sifli_hw_efuse_read(EFUSE_UID, uid, DFU_UID_SIZE);

    static mbedtls_aes_context ctx;
    static uint8_t stream_block[16];
    mbedtls_aes_init(&ctx);
    mbedtls_aes_setkey_enc(&ctx, key, DFU_KEY_SIZE * 8);

    mbedtls_aes_crypt_cbc(&ctx, MBEDTLS_AES_ENCRYPT, size, &uid[0], in_data, out_data);
#endif
}


void sifli_hw_enc_with_key(uint8_t *key, uint8_t *in_data, uint8_t *out_data, int size, uint32_t init_offset)
{
    uint32_t offset = 0;
#ifdef BSP_USING_HW_AES
    {
#define AES_BLOCK_SIZE 512
        static uint8_t temp[AES_BLOCK_SIZE];
        memset(temp, 0, AES_BLOCK_SIZE);
        while (offset < size)
        {
            int len = (size - offset) < AES_BLOCK_SIZE ? (size - offset) : AES_BLOCK_SIZE;
            memcpy(temp, in_data + offset, len);
            HAL_AES_init((uint32_t *)key, DFU_KEY_SIZE, (uint32_t *)dfu_get_counter(init_offset + offset), AES_MODE_CTR);
            HAL_AES_run(AES_ENC, temp, out_data + offset, len);
            offset += len;
        }
    }
#else

#endif
}


/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
