/**
  ******************************************************************************
  * @file   secboot.c
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

#include <rtconfig.h>
#include <string.h>
#include <stdint.h>
#include "../dfu/dfu.h"
#include "board.h"
#include "drv_io.h"
#ifndef HAL_AES_MODULE_ENABLED
    #include "mbedtls/aes.h"
#endif
#include "mbedtls/cipher.h"
#include "mbedtls/pk.h"
#include "mbedtls/sha256.h"
#include "secboot.h"

#define SPLIT_THRESHOLD     (256)
#define SECBOOT_SIGKEY_PUB_ERR      (1)
#define SECBOOT_IMG_HASH_SIG_ERR    (2)


#define IS_MPI_ADDR(addr, i) ((addr >= MPI##i##_MEM_BASE) && (addr < (MPI##i##_MEM_BASE + QSPI##i##_MAX_SIZE)))

/** encoded key */
ALIGN(4)
static uint8_t dfu_key[DFU_KEY_SIZE];
/** key in plaintext */
ALIGN(4)
static uint8_t dfu_key1[DFU_KEY_SIZE];

static flash_read_func secboot_flash_read;

uint8_t sig_pub_key[DFU_SIG_KEY_SIZE] =
{
    0x30, 0x82, 0x01, 0x22, 0x30, 0x0D, 0x06, 0x09, 0x2A, 0x86, 0x48, 0x86,
    0xF7, 0x0D, 0x01, 0x01, 0x01, 0x05, 0x00, 0x03, 0x82, 0x01, 0x0F, 0x00,
    0x30, 0x82, 0x01, 0x0A, 0x02, 0x82, 0x01, 0x01, 0x00, 0xC2, 0x4D, 0x2F,
    0x9E, 0xE7, 0x2A, 0x31, 0x6A, 0xCE, 0x26, 0x23, 0x99, 0xF5, 0xD8, 0x95,
    0x7D, 0x9C, 0x88, 0x96, 0xB3, 0x94, 0x15, 0xD4, 0x71, 0x28, 0xEC, 0xBE,
    0x2E, 0x8D, 0xC5, 0xC8, 0x45, 0xDA, 0x66, 0x4A, 0x5F, 0x9B, 0x23, 0xD7,
    0x4C, 0xD0, 0xAB, 0x2E, 0xA2, 0x30, 0x21, 0xC5, 0x33, 0x51, 0x52, 0xCB,
    0x79, 0x8D, 0xC6, 0x4E, 0xB1, 0x54, 0x13, 0xA7, 0x1C, 0x65, 0x0D, 0x83,
    0xF8, 0x7E, 0x7E, 0xA2, 0xC0, 0xB7, 0x86, 0xE7, 0xEA, 0x42, 0x32, 0x78,
    0xF8, 0x1D, 0x61, 0x65, 0x2B, 0x3D, 0xB4, 0x40, 0x0A, 0x19, 0xE2, 0xDD,
    0x86, 0xBA, 0xD0, 0x4D, 0x0C, 0xD7, 0xC6, 0x94, 0xFC, 0x48, 0x72, 0xE5,
    0xA0, 0x49, 0xE9, 0x6D, 0x44, 0xA8, 0xD3, 0x70, 0x4B, 0x18, 0x8D, 0x92,
    0xED, 0xE7, 0xEA, 0x94, 0x56, 0xBD, 0x05, 0xF2, 0xDF, 0x64, 0xBA, 0xAF,
    0xD2, 0xAE, 0xC4, 0xDF, 0xC5, 0xC0, 0x4C, 0xC4, 0x89, 0xA4, 0x76, 0x5B,
    0x4C, 0x5C, 0x76, 0xB0, 0x2B, 0x0B, 0xED, 0x66, 0xB9, 0x2E, 0xD7, 0xB4,
    0x4F, 0x5A, 0xDC, 0x4C, 0xDB, 0xB5, 0xFC, 0x17, 0x90, 0x90, 0xCC, 0x21,
    0x18, 0x29, 0x77, 0x10, 0x45, 0x53, 0xEB, 0x92, 0xAD, 0xB3, 0x15, 0x5E,
    0xEB, 0x9F, 0xD1, 0x27, 0x24, 0xCC, 0x84, 0x11, 0x58, 0x90, 0x42, 0xF9,
    0xA4, 0xAA, 0x04, 0x4E, 0x3C, 0xDA, 0x8B, 0x5F, 0x5B, 0x75, 0x63, 0x82,
    0x78, 0xA6, 0x79, 0x1B, 0x74, 0x68, 0x7E, 0xEC, 0xAC, 0x63, 0xC4, 0x64,
    0x5B, 0x49, 0xBB, 0x4B, 0xBC, 0x39, 0xB9, 0x40, 0xB1, 0x5E, 0x13, 0x2B,
    0x00, 0x4E, 0x6E, 0x8B, 0x8E, 0x2C, 0x8E, 0x33, 0xA1, 0x94, 0x4C, 0x02,
    0xC0, 0x44, 0xFE, 0xCB, 0x36, 0x22, 0x49, 0x3F, 0x6C, 0x2F, 0x25, 0x19,
    0x6B, 0x9D, 0x1F, 0x1F, 0x60, 0x1A, 0xBE, 0x06, 0x06, 0xE9, 0x04, 0x08,
    0x09, 0x02, 0x03, 0x01, 0x00, 0x01
};


__WEAK uint8_t *sifli_get_sig_pub_key(void)
{
    return sig_pub_key;
}

#ifdef SECBOOT_USING_APP_IMG_SIG_VERIFIY
/* out buf size must more than 32 byte */
static int hash_calculate(uint8_t *in, uint32_t in_size, uint8_t *out, uint8_t algo)
{
#ifndef HAL_HASH_MODULE_ENABLED
    static mbedtls_sha256_context ctx;
#endif /* !HAL_HASH_MODULE_ENABLED */
    int last, i, j;
#ifdef SECBOOT_APP_IMG_SIG_BY_UID_ENABLD
    uint8_t uid[DFU_UID_SIZE] = {0};
#endif /* SECBOOT_APP_IMG_SIG_BY_UID_ENABLD */
    uint32_t ex_data_size = 0;
    uint8_t *ex_data = NULL;
    uint8_t *blk = NULL;
    uint32_t blk_size;
    static uint32_t blk_buf[SPLIT_THRESHOLD / sizeof(uint32_t)];
    int32_t remaining_size;

    if (!in || !in_size || !out || algo > 3)
        return -1;

#ifdef SECBOOT_APP_IMG_SIG_BY_UID_ENABLD
    i = HAL_EFUSE_Read(0, uid, DFU_UID_SIZE);
    if (i < DFU_UID_SIZE)
    {
        return -2;

    }
    ex_data_size = DFU_UID_SIZE;
    ex_data = uid;
#endif /* SECBOOT_APP_IMG_SIG_BY_UID_ENABLD */


#ifdef HAL_HASH_MODULE_ENABLED
    HAL_HASH_reset();
    HAL_HASH_init(NULL, algo, 0);
#else
    mbedtls_sha256_init(&ctx);
    mbedtls_sha256_starts(&ctx, 0);
#endif /* HAL_HASH_MODULE_ENABLED */

    i = 0;
    remaining_size = (in_size + ex_data_size);
    while (remaining_size > 0)
    {
        if (SPLIT_THRESHOLD >= remaining_size)
        {
            /* last block */
            last = 1;
            blk_size = remaining_size;
            remaining_size = 0;
        }
        else
        {
            last = 0;
            blk_size = SPLIT_THRESHOLD;
            remaining_size -= SPLIT_THRESHOLD;
        }

#ifdef HAL_HASH_MODULE_ENABLED
        if (i > 0)
        {
            HAL_HASH_reset();
            HAL_HASH_init((uint32_t *)out, algo, last ? i : 0);
        }
#endif /* HAL_HASH_MODULE_ENABLED */

        if ((i + blk_size) <= in_size)
        {
            /* use all data from in buf */
            blk = &in[i];
        }
        else
        {
            /* combine in and ex_data into blk */
            blk = (uint8_t *)blk_buf;
            j = 0;
            if (i < in_size)
            {
                j = in_size - i;
                /* copy rest data from in */
                memcpy((void *)blk_buf, (void *)&in[i], j);
            }
            /* copy from ex_data*/
            memcpy((void *)((uint32_t)blk_buf + j), (void *)&ex_data[i + j - in_size], blk_size - j);
        }
#ifdef HAL_HASH_MODULE_ENABLED
        HAL_HASH_run(blk, blk_size, last);
        HAL_HASH_result(out);
#else
        mbedtls_sha256_update(&ctx, blk, blk_size);
#endif /* HAL_HASH_MODULE_ENABLED */
        i += blk_size;
    }

#ifndef HAL_HASH_MODULE_ENABLED
    // mbedtls_sha256_update(&ctx, in, in_size);
    mbedtls_sha256_finish(&ctx, out);
#endif /* ！HAL_HASH_MODULE_ENABLED */

    return 0;
}

static int img_sig_hash_verify(uint8_t *img_hash_sig, uint8_t *sig_pub_key, uint8_t *image, uint32_t img_size)
{
    uint8_t img_hash[32] = {0};
    mbedtls_pk_context pk;

    /*1.calculate image hash*/
    if (hash_calculate(image, img_size, img_hash, HASH_ALGO_SHA256))
        return -1;

    /*2.verify image hash digital signature*/
    mbedtls_pk_init(&pk);
    if (mbedtls_pk_parse_public_key(&pk, sig_pub_key, DFU_SIG_KEY_SIZE))
        return -1;

    mbedtls_rsa_set_padding((mbedtls_rsa_context *)pk.pk_ctx, MBEDTLS_RSA_PKCS_V15, MBEDTLS_MD_SHA256);
    if (mbedtls_pk_verify(&pk, MBEDTLS_MD_SHA256, img_hash, DFU_IMG_HASH_SIZE, img_hash_sig, DFU_SIG_SIZE))
        return -1;

    return 0;
}

static void secboot_exception(uint8_t excpt)
{
    switch (excpt)
    {
    case SECBOOT_SIGKEY_PUB_ERR:
        printf("secboot sigkey pub err!");
        break;
    case SECBOOT_IMG_HASH_SIG_ERR:
        printf("secboot img hash sig err!");
        break;
    default:
        printf("secboot excpt null!");
        break;
    }

    HAL_sw_breakpoint();
}

#endif /* SECBOOT_USING_APP_IMG_SIG_VERIFIY  */



static uint8_t is_addr_in_flash(uint32_t addr)
{
    uint8_t is_in_flash = 0;

    if ((addr >= FLASH_BASE_ADDR && addr < HPSYS_RAM0_BASE)
#ifdef BSP_QSPI2_MEM_SIZE
            || (addr >= FLASH2_BASE_ADDR && addr < (FLASH2_BASE_ADDR + FLASH2_SIZE))
#endif
#ifdef BSP_QSPI3_MEM_SIZE
            || (addr >= FLASH3_BASE_ADDR && addr < (FLASH3_BASE_ADDR + FLASH3_SIZE))
#endif
#ifdef BSP_QSPI4_MEM_SIZE
            || (addr >= FLASH4_BASE_ADDR && addr < (FLASH4_BASE_ADDR + FLASH4_SIZE))
#endif
       )
        is_in_flash = 1;
    return is_in_flash;
}


static uint8_t is_addr_in_mpi(uint32_t addr)
{
    uint8_t ret = 0;

    if (IS_MPI_ADDR(addr, 1)
            || IS_MPI_ADDR(addr, 2)
#ifdef BSP_ENABLE_QSPI3
            || IS_MPI_ADDR(addr, 3)
#endif
#ifdef BSP_ENABLE_QSPI4
            || IS_MPI_ADDR(addr, 4)
#endif
#ifdef BSP_ENABLE_QSPI5
            || IS_MPI_ADDR(addr, 5)
#endif
       )
        ret = 1;
    return ret;
}

// Move secure key to flash5
void sec_flash_key_update(void)
{
#ifdef BSP_ENABLE_MPI5
    uint32_t count = 0xFFFFF;
    // Move secure key to lpaon
    hwp_hpsys_rcc->ENR1 |= HPSYS_RCC_ENR1_PTC1;
    hwp_ptc1->TCR0 = PTC_TCR0_SWTRIG | 0x08;

    while (((hwp_ptc1->ISR & PTC_ISR_TCIF0) == 0) && count)
        count--;

    // No need to 2nd move since 1st failed
    if (!count)
        return;

    hwp_ptc1->ICR |= PTC_ICR_CTCIF0;
    count = 0xFFFFF;

    // Move lpaon to mpi5
    hwp_lpsys_rcc->ENR1 |= LPSYS_RCC_ENR1_PTC2;
    hwp_ptc2->TCR0 = PTC_TCR0_SWTRIG | 0x08;

    while (((hwp_ptc2->ISR & PTC_ISR_TCIF0) == 0) && count)
        count--;

    hwp_ptc2->ICR |= PTC_ICR_CTCIF0;
#endif
}

static void boot_copy_img(uint8_t *dest, uint8_t *src, uint32_t len)
{
    secboot_flash_read((uint32_t)src, dest, len);

    SCB_CleanDCache();
}


static void run_img(uint8_t *dest)
{
    __asm("LDR SP, [%0]" :: "r"(dest));
    __asm("LDR PC, [%0, #4]" :: "r"(dest));
}

void boot_set_flash_read_func(flash_read_func read_func)
{
    secboot_flash_read = read_func;
}

void dfu_boot_img_in_flash(int flashid)
{
    struct image_header_enc *img_hdr;
    int coreid = DFU_FLASH_IMG_IDX(flashid);
    FLASH_HandleTypeDef *xip_handle;

    if ((flashid >= DFU_FLASH_PARTITION)
            || (flashid < DFU_FLASH_IMG_LCPU))
    {
        return ;
    }

    img_hdr = &(g_sec_config->imgs[coreid]);

    uint8_t *src, *dest;

    if (img_hdr->flags & DFU_FLAG_SINGLE)
        src = dest = (uint8_t *)DFU_SING_IMG_START;
    else
    {
        src = (uint8_t *)(g_sec_config->ftab[flashid].base);
        dest = (uint8_t *)(g_sec_config->ftab[flashid].xip_base);
    }

    if (img_hdr->length == FLASH_UNINIT_32)
        return;
    if (img_hdr->flags & DFU_FLAG_ENC)
    {
        uint32_t is_flash = 1;
        if (img_hdr->length && (src != dest)
                && (is_addr_in_flash((uint32_t)dest) == 0))
        {
            memcpy(dest, src, img_hdr->length);
            is_flash = 0;
        }

        if (coreid < 2 * CORE_MAX)
        {
            coreid %= CORE_MAX;
            if (coreid == CORE_HCPU || coreid == CORE_BL || coreid == CORE_LCPU)
            {
                if (is_addr_in_mpi((uint32_t)dest))
                {
                    memcpy(dfu_key, img_hdr->key, sizeof(dfu_key));
                    sifli_hw_init_xip_key(dfu_key);
#if !defined(SOC_SF32LB55X) && !defined(SOC_SF32LB52X)
                    // Only MPI5 need copy key
                    if (IS_MPI_ADDR((uint32_t)dest, 5))
                        sec_flash_key_update();
#endif
                    xip_handle = BSP_Flash_get_handle((uint32_t)dest);
                    HAL_ASSERT(xip_handle);

                    // Setup XIP for decoding and running
                    HAL_FLASH_NONCE_CFG(xip_handle, (uint32_t)dest, (uint32_t)dest + img_hdr->length,
                                        dfu_get_counter(0));
                    HAL_ASSERT(src >= dest);

                    if (is_flash)
                        HAL_FLASH_ALIAS_CFG(xip_handle, (uint32_t)dest, img_hdr->length,
                                            (uint32_t)src - (uint32_t)dest);
                    /* enable on-the-fly decoder */
                    HAL_FLASH_AES_CFG(xip_handle, 1);

                }
                else
                {
                    /* copy encrypted key to ram as AES_ACC cannot access flash */
                    memcpy(dfu_key, img_hdr->key, sizeof(dfu_key));
                    sifli_hw_dec_key(dfu_key, dfu_key1, sizeof(dfu_key1));
                    sifli_hw_dec(dfu_key1, src, dest, img_hdr->length, 0);
                }
                run_img(dest);
            }
            else        // For other core, decode to corresponding location
            {
                /* copy encrypted key to ram as AES_ACC cannot access flash */
                memcpy(dfu_key, img_hdr->key, sizeof(dfu_key));
                sifli_hw_dec_key(dfu_key, dfu_key1, sizeof(dfu_key1));
                sifli_hw_dec(dfu_key1, src, dest, img_hdr->length, 0);
                if (coreid == CORE_BOOT)
                {
                    // Install boot patch from flash
                    boot_init_hook hook = (boot_init_hook)((uint32_t)dest | 1);
                    hook();
                }
            }
        }
        else        // For FPGA extension code section, decode to corresponding location
        {
            /* copy encrypted key to ram as AES_ACC cannot access flash */
            memcpy(dfu_key, img_hdr->key, sizeof(dfu_key));
            sifli_hw_dec_key(dfu_key, dfu_key1, sizeof(dfu_key1));
            sifli_hw_dec(dfu_key1, src, dest, img_hdr->length, 0);
        }
    }
    else
    {
        if (img_hdr->length && (src != dest)
                && (is_addr_in_flash((uint32_t)dest) == 0))
        {
#ifdef SECBOOT_USING_APP_IMG_SIG_VERIFIY
            boot_copy_img(dest, src, img_hdr->length + DFU_SIG_SIZE);
#else
            boot_copy_img(dest, src, img_hdr->length);
#endif /* SECBOOT_USING_APP_IMG_SIG_VERIFIY */
        }
        if (coreid < 2 * CORE_MAX)
        {
            coreid %= CORE_MAX;
            if (coreid == CORE_HCPU || coreid == CORE_BL || coreid == CORE_LCPU)
            {
                if (is_addr_in_flash((uint32_t)dest))
                {
                    /* update offset of flash XIP */
                    HAL_ASSERT(src >= dest);
                    xip_handle = BSP_Flash_get_handle((uint32_t)dest);
                    HAL_ASSERT(xip_handle);
                    HAL_FLASH_ALIAS_CFG(xip_handle, (uint32_t)dest, img_hdr->length,
                                        (uint32_t)src - (uint32_t)dest);
                }
#ifdef SECBOOT_USING_APP_IMG_SIG_VERIFIY
                if (coreid == CORE_HCPU)
                {
                    /* verify image hash signature */
                    uint8_t *sig_pubkey = sifli_get_sig_pub_key();
                    uint8_t *img_sig = (uint8_t *)(dest + img_hdr->length);
                    // HAL_sw_breakpoint();
                    if (img_sig_hash_verify(img_sig, sig_pubkey, (uint8_t *)dest, img_hdr->length))
                    {
                        secboot_exception(SECBOOT_IMG_HASH_SIG_ERR);
                    }
                }
#endif /* SECBOOT_USING_APP_IMG_SIG_VERIFIY */

                boot_precheck();

                run_img(dest);
            }
            else if (coreid == CORE_BOOT)
            {
                boot_init_hook hook = (boot_init_hook)((uint32_t)dest | 1);
                hook();
            }

        }
    }
}

int32_t boot_get_uid(uint8_t *uid, uint32_t len)
{
    int32_t i;

    if (len < DFU_UID_SIZE)
    {
        return -1;
    }

    i = HAL_EFUSE_Read(0, uid, DFU_UID_SIZE);
    if (i < DFU_UID_SIZE)
    {
        return -2;
    }

    return 0;
}

__WEAK int32_t boot_read_sig(uint8_t *sig, uint32_t len)
{
    int32_t i;

    if (len < DFU_SIG_SIZE)
    {
        return -1;
    }

    HAL_ASSERT(secboot_flash_read);

#ifdef BOOT_SIG_REGION_START_ADDR
    i = secboot_flash_read((uint32_t)BOOT_SIG_REGION_START_ADDR, sig, DFU_SIG_SIZE);
#else
    i = 0;
#endif /* BOOT_SIG_REGION_START_ADDR */
    if (i < DFU_SIG_SIZE)
    {
        return -2;
    }

    return 0;
}

int32_t boot_sha256_calculate(uint8_t *in, uint32_t in_size, uint8_t *uid, uint32_t uid_size, uint8_t *out, uint32_t out_size)
{
#ifndef HAL_HASH_MODULE_ENABLED
    static mbedtls_sha256_context ctx;
#endif /* !HAL_HASH_MODULE_ENABLED */
    int last, i, j;
    uint32_t ex_data_size = 0;
    uint8_t *ex_data = NULL;
    uint8_t *blk = NULL;
    uint32_t blk_size;
    static uint32_t blk_buf[SPLIT_THRESHOLD / sizeof(uint32_t)];
    int32_t remaining_size;

    if (!in || !in_size || !out || (out_size < 32))
        return -1;

    if (uid)
    {
        ex_data_size = uid_size;
        ex_data = uid;
    }

#ifdef HAL_HASH_MODULE_ENABLED
    HAL_HASH_reset();
    HAL_HASH_init(NULL, algo, 0);
#else
    mbedtls_sha256_init(&ctx);
    mbedtls_sha256_starts(&ctx, 0);
#endif /* HAL_HASH_MODULE_ENABLED */

    i = 0;
    remaining_size = (in_size + ex_data_size);
    while (remaining_size > 0)
    {
        if (SPLIT_THRESHOLD >= remaining_size)
        {
            /* last block */
            last = 1;
            blk_size = remaining_size;
            remaining_size = 0;
        }
        else
        {
            last = 0;
            blk_size = SPLIT_THRESHOLD;
            remaining_size -= SPLIT_THRESHOLD;
        }

#ifdef HAL_HASH_MODULE_ENABLED
        if (i > 0)
        {
            HAL_HASH_reset();
            HAL_HASH_init((uint32_t *)out, algo, last ? i : 0);
        }
#endif /* HAL_HASH_MODULE_ENABLED */

        if ((i + blk_size) <= in_size)
        {
            /* use all data from in buf */
            blk = &in[i];
        }
        else
        {
            /* combine in and ex_data into blk */
            blk = (uint8_t *)blk_buf;
            j = 0;
            if (i < in_size)
            {
                j = in_size - i;
                /* copy rest data from in */
                memcpy((void *)blk_buf, (void *)&in[i], j);
            }
            /* copy from ex_data*/
            memcpy((void *)((uint32_t)blk_buf + j), (void *)&ex_data[i + j - in_size], blk_size - j);
        }
#ifdef HAL_HASH_MODULE_ENABLED
        HAL_HASH_run(blk, blk_size, last);
        HAL_HASH_result(out);
#else
        mbedtls_sha256_update(&ctx, blk, blk_size);
#endif /* HAL_HASH_MODULE_ENABLED */
        i += blk_size;
    }

#ifndef HAL_HASH_MODULE_ENABLED
    // mbedtls_sha256_update(&ctx, in, in_size);
    mbedtls_sha256_finish(&ctx, out);
#endif /* ！HAL_HASH_MODULE_ENABLED */

    return 0;
}


__WEAK void boot_precheck(void)
{
#if 0
    int32_t r;
    uint8_t uid[DFU_UID_SIZE] = {0};
    uint8_t img_hash[32] = {0};
    uint8_t img_hash_sig[DFU_SIG_SIZE];
    mbedtls_pk_context pk;
    uint8_t *sig_pubkey;

    r = boot_get_uid(uid, DFU_UID_SIZE);
    HAL_ASSERT(0 == r);
    /* calculate sha256 by UID */
    r = boot_sha256_calculate((uint8_t *)uid, DFU_UID_SIZE, NULL, 0, img_hash, 32);
    HAL_ASSERT(0 == r);
    r = boot_read_sig(img_hash_sig, DFU_SIG_SIZE);
    HAL_ASSERT(0 == r);

    sig_pubkey = sifli_get_sig_pub_key();
    HAL_ASSERT(sig_pubkey);

    mbedtls_pk_init(&pk);
    if (mbedtls_pk_parse_public_key(&pk, sig_pubkey, DFU_SIG_KEY_SIZE))
    {
        printf("mbedtls parse public key failed!\n");
        while (1);
    }

    mbedtls_rsa_set_padding((mbedtls_rsa_context *)pk.pk_ctx, MBEDTLS_RSA_PKCS_V15, MBEDTLS_MD_SHA256);
    /* verify the signature */
    if (mbedtls_pk_verify(&pk, MBEDTLS_MD_SHA256, img_hash, DFU_IMG_HASH_SIZE, img_hash_sig, DFU_SIG_SIZE))
    {
        printf("signature verification failed!\n");
        while (1);
    }
#endif
}


/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
