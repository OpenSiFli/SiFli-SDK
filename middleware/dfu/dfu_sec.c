/**
  ******************************************************************************
  * @file   dfu_sec.c
  * @author Sifli software development team
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2021 - 2021,  Sifli Technology
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

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <rtthread.h>
#include "board.h"
#include "os_adaptor.h"

#include "dfu.h"
#include "drv_flash.h"
#include "mbedtls/cipher.h"
#include "mbedtls/pk.h"
#include "mbedtls/sha256.h"

#include "dfu_internal.h"

#include "log.h"
struct sec_configuration *g_sec_config;
static uint8_t config_malloc = 0;

static mbedtls_sha256_context ctx2;

DFU_NON_RET_SECT_BEGIN
static uint8_t dfu_hash[DFU_SIG_HASH_SIZE];

/** encoded key */
ALIGN(4)
static uint8_t dfu_key[DFU_KEY_SIZE];


ALIGN(4)
static uint8_t g_aes_ctr_iv[DFU_IV_LEN];
DFU_NON_RET_SECT_END

static const uint8_t g_fake_dfu_efuse_uid[DFU_UID_SIZE] =
{
    0xB7, 0x76, 0x6B, 0x8A, 0xD7, 0xA5, 0xE7, 0xD0, 0x88, 0x52, 0x36, 0xDE, 0xC3, 0x16, 0x36, 0x4C
};

static const uint8_t g_fake_dfu_efuse_sig_hash[DFU_SIG_HASH_SIZE] =
{
    0x56, 0xCE, 0x4E, 0xC4, 0xC9, 0xDC, 0xF4, 0x11
};

static const uint8_t g_fake_dfu_efuse_root_key[DFU_KEY_SIZE] =
{
    0xE6, 0x92, 0xDF, 0xB5, 0xE8, 0xFA, 0xC2, 0x43, 0x78, 0x13, 0x34, 0x5D, 0x1B, 0x4B, 0xE2, 0xB9,
    0x50, 0x9D, 0xE5, 0xC2, 0xF4, 0x05, 0x62, 0xE6, 0xC8, 0x25, 0x85, 0x81, 0x02, 0x59, 0x17, 0x2B,
};

static int dfu_get_efuse_hook(uint8_t id, uint8_t *data, int size)
{
    int ret = 0;

    if (id == EFUSE_UID)
    {
        memcpy(data, (uint8_t *)g_fake_dfu_efuse_uid, DFU_UID_SIZE);
        ret = DFU_UID_SIZE;
    }
    else if (id == EFUSE_ID_SIG_HASH)
    {
        memcpy(data, (uint8_t *)g_fake_dfu_efuse_sig_hash, DFU_SIG_HASH_SIZE);
        ret = DFU_SIG_HASH_SIZE;
    }
    else if (id == EFUSE_ID_ROOT)
    {
        memcpy(data, (uint8_t *)g_fake_dfu_efuse_root_key, DFU_KEY_SIZE);
        ret = DFU_KEY_SIZE;
    }

    return ret;

}


static uint8_t *dfu_get_public_key(void)
{
    return &g_sec_config->sig_pub_key[0];
}


static uint8_t dfu_secure_boot_check()
{
    uint8_t pattern;
    uint32_t ret = 0;
    int len = sifli_hw_efuse_read(EFUSE_ID_SECURE_ENABLED, &pattern, DFU_SECURE_SIZE);
    if (len == DFU_SECURE_SIZE)
        if (pattern == DFU_SECURE_PATTERN)
            ret = 1;

    return ret;
}

static struct image_header_enc *dfu_get_enc_image_header(uint8_t img_id, struct sec_configuration *sec_config)
{
#ifdef OTA_55X
    struct image_header_enc *header = NULL;
    int8_t flashid = dfu_get_flashid_by_imgid(img_id);
    if (flashid < 0)
        return NULL;

    int flash_img_idx = DFU_FLASH_IMG_IDX(flashid);
    if (flash_img_idx > DFU_FLASH_IMG_IDX(DFU_FLASH_PARTITION))
        return NULL;
    if (sec_config == NULL)
        sec_config = g_sec_config;

    header = &sec_config->imgs[flash_img_idx];
    return header;
#else
    return NULL;
#endif
}

void dfu_sec_init(void)
{
    init_dfu_efuse_read_hook();
#ifdef OTA_55X
#ifdef SOC_SF32LB55X
    g_sec_config = (struct sec_configuration *)FLASH_START;
#elif defined (SOC_SF32LB52X)
    g_sec_config = (struct sec_configuration *)0x12000000;
#else
    g_sec_config = (struct sec_configuration *)0x1c000000;
#endif
#endif

#ifdef OTA_56X_NAND
#ifdef OTA_NAND_ONLY
    config_malloc = 0;
#else
    g_sec_config = (struct sec_configuration *)0x1c000000;
#endif
#endif
    if (dfu_secure_boot_check() == 0)
    {
        g_dfu_efuse_read_hook = dfu_get_efuse_hook;
    }
}

void dfu_sec_config_malloc()
{
#ifdef OTA_NAND_ONLY
    LOG_I("dfu_sec_config_malloc");
    if (config_malloc == 0)
    {
        g_sec_config = malloc(sizeof(struct sec_configuration));
        OS_ASSERT(g_sec_config);
        config_malloc = 1;
    }
    dfu_packet_read_flash_ext(0x62000000, 0, (uint8_t *)g_sec_config, sizeof(struct sec_configuration));
#endif
}

void dfu_sec_config_free()
{
#ifdef OTA_NAND_ONLY
    LOG_I("dfu_sec_config_free");
    if (config_malloc == 1)
    {
        config_malloc = 0;
        free(g_sec_config);
    }
#endif
}

void dump_config()
{
    int i, *test;

    LOG_I("Secured images:");
    for (i = 0; i < DFU_FLASH_PARTITION - 2; i++)
    {
        if (g_sec_config->imgs[i].length != FLASH_UNINIT_32)
            LOG_I("\tImg %d: length 0x%x, block size %d, flags:0x%x", i + 1,
                  g_sec_config->imgs[i].length, g_sec_config->imgs[i].blksize, g_sec_config->imgs[i].flags);
    }
    LOG_I("Running images:");
    for (i = 0; i < CORE_MAX; i++)
    {
        if ((uint32_t)g_sec_config->running_imgs[i] != FLASH_UNINIT_32)
            LOG_I("\t %d", g_sec_config->running_imgs[i] - &g_sec_config->imgs[0] + 1);
    }
    LOG_I("Flash table:");
    for (i = 0; i < DFU_FLASH_PARTITION; i++)
    {
        if (g_sec_config->ftab[i].size != FLASH_UNINIT_32)
            LOG_I("\t%d: base=0x%x, size=0x%x, xip_base=0x%x, flags=0x%x", i,
                  g_sec_config->ftab[i].base, g_sec_config->ftab[i].size,
                  g_sec_config->ftab[i].xip_base, g_sec_config->ftab[i].flags);
    }
    uint8_t *dfu_temp;
    dfu_temp = malloc(DFU_MAX_BLK_SIZE);
    OS_ASSERT(dfu_temp);
    for (i = EFUSE_UID; i <= EFUSE_ID_SIG_HASH; i++)
    {
        int size, j;
        switch (i)
        {
        case EFUSE_UID:
            size = DFU_UID_SIZE;
            LOG_I("UID: ");
            break;
        case EFUSE_ID_ROOT:
            size = DFU_KEY_SIZE;
            LOG_I("Root key: ");
            break;
        case EFUSE_ID_SIG_HASH:
            size = DFU_SIG_HASH_SIZE;
            LOG_I("Sig hash: ");
            break;
        default:
            continue;
        }
        sifli_hw_efuse_read(i, dfu_temp, size);
        for (j = 0; j < size; j++)
            if (dfu_temp[i] != FLASH_UNINIT_8)
                break;
        if (j < size)
        {
            for (j = 0; j < size; j++)
            {
                LOG_RAW("%02X", dfu_temp[j]);
            }
            LOG_RAW("\n");
        }
        else
            LOG_W(" empty\n");
    }
    free(dfu_temp);
    LOG_I("SIG: ");
    for (i = 0; i < DFU_SIG_KEY_SIZE; i++)
        if (g_sec_config->sig_pub_key[i] != FLASH_UNINIT_8)
            break;
    if (i < DFU_SIG_KEY_SIZE)
        LOG_I(" received\n");
    else
        LOG_W(" empty\n");
}
MSH_CMD_EXPORT(dump_config, Dump system configuration.);


uint8_t *dfu_get_counter(uint32_t offset)
{
    int i;
    sifli_hw_efuse_read(EFUSE_ID_SIG_HASH, g_aes_ctr_iv, DFU_SIG_HASH_SIZE);
    memset(&g_aes_ctr_iv[8], 0, 8);
    offset >>= 4;   // Counter is increased every 16 bytes
    for (i = 15; i >= 12 && offset > 0; i--, offset >>= 8)
    {
        g_aes_ctr_iv[i] = offset & 0xff;
    }
    return g_aes_ctr_iv;
}


uint8_t *dfu_dec_verify(uint8_t *key, uint32_t offset,
                        uint8_t *in_data, uint8_t *out_data, int size, uint8_t *hash)
{
    uint8_t *r = NULL;

#ifdef SOC_SF32LB52X
    uint8_t *temp_out;
    int align_size = (size / 32 + 1) * 32;
    temp_out = rt_malloc(align_size + 32);
    OS_ASSERT(temp_out);

    uint8_t align_offset = 0;
    if ((uint32_t)temp_out % 32 != 0)
    {
        align_offset = 32 - (uint32_t)temp_out % 32;
    }

    SCB_InvalidateDCache_by_Addr(temp_out + align_offset, align_size);
    SCB_InvalidateICache_by_Addr(temp_out + align_offset, align_size);

    sifli_hw_dec(key, in_data, temp_out + align_offset, size, offset);
    rt_memcpy(out_data, temp_out + align_offset, size);
    rt_free(temp_out);
#else
    sifli_hw_dec(key, in_data, out_data, size, offset);
#endif

    mbedtls_sha256_init(&ctx2);
    mbedtls_sha256_starts(&ctx2, 0); /* SHA-256, not 224 */
    mbedtls_sha256_update(&ctx2, out_data, size);

    uint8_t temp_hash[32] = {0};
    memcpy(temp_hash, dfu_hash, DFU_SIG_HASH_SIZE);
    mbedtls_sha256_finish(&ctx2, temp_hash);
    memcpy(dfu_hash, temp_hash, DFU_SIG_HASH_SIZE);

    if (memcmp(dfu_hash, hash, DFU_SIG_HASH_SIZE) == 0)
        r = out_data;
    return r;
}

int8_t dfu_integrate_verify(uint8_t *in_data, int size, uint8_t *hash)
{
    int8_t ret = -1;

    mbedtls_sha256_init(&ctx2);
    mbedtls_sha256_starts(&ctx2, 0); /* SHA-256, not 224 */
    mbedtls_sha256_update(&ctx2, in_data, size);

    uint8_t temp_hash[32] = {0};
    memcpy(temp_hash, dfu_hash, DFU_SIG_HASH_SIZE);
    mbedtls_sha256_finish(&ctx2, temp_hash);
    memcpy(dfu_hash, temp_hash, DFU_SIG_HASH_SIZE);

    if (memcmp(dfu_hash, hash, DFU_SIG_HASH_SIZE) == 0)
        ret = 0;
    return ret;
}

uint8_t dfu_img_verification(dfu_ctrl_env_t *env)
{
#ifdef OTA_55X
    dfu_img_info_t *curr_info = &env->prog.fw_context.code_img.curr_img_info;
    if (env->prog.dfu_ID == DFU_ID_OTA_MANAGER)
    {
        curr_info = &env->ota_state.fw_context.code_img.curr_img_info;
    }
    dfu_image_header_int_t *img_hdr = curr_info->header;
    int offset = 0;
    int size = 0;
    uint8_t *dfu_temp;
    dfu_temp = malloc(DFU_MAX_BLK_SIZE);
    OS_ASSERT(dfu_temp);
    uint32_t read_size = DFU_MAX_BLK_SIZE;
    uint8_t status = DFU_ERR_FW_INVALID;
    uint8_t *sig_pub_key = dfu_get_public_key();
    {
        // Calculate HASH
        mbedtls_sha256_context ctx2;
        mbedtls_sha256_init(&ctx2);
        mbedtls_sha256_starts(&ctx2, 0); /* SHA-256, not 224 */
        do
        {
            if (offset + read_size > img_hdr->length)
                size = img_hdr->length - offset;
            else
                size = read_size;
            if (size)
            {
                dfu_read_storage_data(img_hdr, offset, dfu_temp, size);
                mbedtls_sha256_update(&ctx2, dfu_temp, size);
            }
            if (size < read_size)
                break;
            offset += size;
        }
        while (1);
        mbedtls_sha256_finish(&ctx2, dfu_temp);
    }

    {
        // Verify signature
        mbedtls_pk_context pk;
        uint8_t *key;
        mbedtls_pk_init(&pk);
        if (mbedtls_pk_parse_public_key(&pk, sig_pub_key, DFU_SIG_KEY_SIZE) == 0)
        {
            mbedtls_rsa_set_padding((mbedtls_rsa_context *)pk.pk_ctx, MBEDTLS_RSA_PKCS_V15, MBEDTLS_MD_SHA256);
            /*Calculate the RSA encryption of the data. */
            if (mbedtls_pk_verify(&pk, MBEDTLS_MD_SHA256, dfu_temp, DFU_IMG_HASH_SIZE,
                                  img_hdr->sig, DFU_SIG_SIZE) == 0)
            {
                status = DFU_ERR_NO_ERR;
            }
        }
    }
    free(dfu_temp);
    return status;
#else /* OTA_55X */

    return 0;
#endif
}

uint8_t dfu_img_verification_ext(dfu_ctrl_ext_env_t *env)
{
    dfu_img_info_t *curr_info = &env->prog.fw_context.code_img.curr_img_info;
    dfu_image_header_int_t *img_hdr = curr_info->header;
    int offset = 0;
    int size = 0;
    uint8_t *dfu_temp;
    dfu_temp = malloc(DFU_MAX_BLK_SIZE);
    OS_ASSERT(dfu_temp);
    uint32_t read_size = DFU_MAX_BLK_SIZE;
    uint8_t status = DFU_ERR_FW_INVALID;
    uint8_t *sig_pub_key = dfu_get_public_key();
    {
        // Calculate HASH
        mbedtls_sha256_context ctx2;
        mbedtls_sha256_init(&ctx2);
        mbedtls_sha256_starts(&ctx2, 0); /* SHA-256, not 224 */
        do
        {
            if (offset + read_size > img_hdr->length)
                size = img_hdr->length - offset;
            else
                size = read_size;
            if (size)
            {
                dfu_packet_read_flash(img_hdr, offset, dfu_temp, size);
                mbedtls_sha256_update(&ctx2, dfu_temp, size);
            }
            if (size < read_size)
                break;
            offset += size;
        }
        while (1);
        mbedtls_sha256_finish(&ctx2, dfu_temp);

    }

    {
        // Verify signature
        mbedtls_pk_context pk;
        uint8_t *key;
        mbedtls_pk_init(&pk);
        if (mbedtls_pk_parse_public_key(&pk, sig_pub_key, DFU_SIG_KEY_SIZE) == 0)
        {
            mbedtls_rsa_set_padding((mbedtls_rsa_context *)pk.pk_ctx, MBEDTLS_RSA_PKCS_V15, MBEDTLS_MD_SHA256);
            /*Calculate the RSA encryption of the data. */
            if (mbedtls_pk_verify(&pk, MBEDTLS_MD_SHA256, dfu_temp, DFU_IMG_HASH_SIZE,
                                  img_hdr->sig, DFU_SIG_SIZE) == 0)
            {
                status = DFU_ERR_NO_ERR;
            }
        }
    }
    free(dfu_temp);
    return status;
}


int dfu_encrypt_packet(dfu_image_header_int_t *header, uint32_t offset, uint8_t *data, uint32_t size, uint8_t *dfu_key)
{
#ifdef OTA_55X
    sifli_hw_enc_with_key(dfu_key, data, data, size, offset);
    dfu_packet_write_flash(header, offset, data, size);
    return DFU_SUCCESS;
#else /* OTA_55X */
    return DFU_SUCCESS;
#endif
}


int8_t dfu_ctrl_ctrl_header_sig_verify(dfu_ctrl_env_t *env, uint8_t *packet, uint16_t total_len, uint8_t *sig)
{
    uint16_t packet_size = total_len - DFU_SIG_SIZE;
    //uint8_t *sig = (uint8_t *)packet + packet_size;
    uint8_t hash[DFU_IMG_HASH_SIZE];
    int8_t ret = -1;

    {
        // Calculate HASH
        mbedtls_sha256_context ctx2;
        mbedtls_sha256_init(&ctx2);
        mbedtls_sha256_starts(&ctx2, 0); /* SHA-256, not 224 */
        mbedtls_sha256_update(&ctx2, (uint8_t *)packet, packet_size);
        mbedtls_sha256_finish(&ctx2, hash);
    }
    {
        // Verify signature
        mbedtls_pk_context pk;
        uint8_t *key;
        uint8_t *sig_pub_key = dfu_get_public_key();
        mbedtls_pk_init(&pk);
        if (mbedtls_pk_parse_public_key(&pk, sig_pub_key, DFU_SIG_KEY_SIZE) == 0)
        {
            mbedtls_rsa_set_padding((mbedtls_rsa_context *)pk.pk_ctx, MBEDTLS_RSA_PKCS_V15, MBEDTLS_MD_SHA256);
            /*Calculate the RSA encryption of the data. */
            if (mbedtls_pk_verify(&pk, MBEDTLS_MD_SHA256, hash, DFU_IMG_HASH_SIZE, sig, DFU_SIG_SIZE) == 0)
            {
                ret = 0;
            }
        }

    }
    return ret;

}

int8_t dfu_ctrl_ctrl_header_sig_verify_ext(uint8_t *packet, uint16_t total_len, uint8_t *sig)
{
    uint16_t packet_size = total_len - DFU_SIG_SIZE;
    //uint8_t *sig = (uint8_t *)packet + packet_size;
    uint8_t hash[DFU_IMG_HASH_SIZE];
    int8_t ret = -1;

    {
        // Calculate HASH
        mbedtls_sha256_context ctx2;
        mbedtls_sha256_init(&ctx2);
        mbedtls_sha256_starts(&ctx2, 0); /* SHA-256, not 224 */
        mbedtls_sha256_update(&ctx2, (uint8_t *)packet, packet_size);
        mbedtls_sha256_finish(&ctx2, hash);
    }
    {
        // Verify signature
        mbedtls_pk_context pk;
        uint8_t *key;
        uint8_t *sig_pub_key = dfu_get_public_key();
        mbedtls_pk_init(&pk);
        if (mbedtls_pk_parse_public_key(&pk, sig_pub_key, DFU_SIG_KEY_SIZE) == 0)
        {
            mbedtls_rsa_set_padding((mbedtls_rsa_context *)pk.pk_ctx, MBEDTLS_RSA_PKCS_V15, MBEDTLS_MD_SHA256);
            /*Calculate the RSA encryption of the data. */
            if (mbedtls_pk_verify(&pk, MBEDTLS_MD_SHA256, hash, DFU_IMG_HASH_SIZE, sig, DFU_SIG_SIZE) == 0)
            {
                ret = 0;
            }
        }

    }
    return ret;

}


static void dfu_update_sec_flash(struct sec_configuration *sec_config)
{
    uint32_t flash_addr = (uint32_t)g_sec_config;
    uint32_t size = (sizeof(struct sec_configuration) + SPI_NOR_SECT_SIZE * 2) & (~(SPI_NOR_SECT_SIZE * 2 - 1));
    rt_flash_erase(flash_addr, size);
    rt_flash_write(flash_addr, (uint8_t *)sec_config, size);
}

void dfu_update_img_header(dfu_ctrl_env_t *env)
{
    dfu_dl_image_header_t *dl_hdr = &env->prog.fw_context.code_img;
    struct image_header_enc *enc_hdr;
    struct sec_configuration *cache = (struct sec_configuration *)malloc(sizeof(struct sec_configuration));
    OS_ASSERT(cache);
    memcpy((uint8_t *)cache, (uint8_t *)g_sec_config, sizeof(struct sec_configuration));

    for (uint32_t i  = 0; i < dl_hdr->img_count; i++)
    {
        enc_hdr = dfu_get_enc_image_header(dl_hdr->img_header[i].img_id, cache);
        if (enc_hdr == NULL)
            continue;
        enc_hdr->blksize = dl_hdr->blk_size;
        enc_hdr->flags = dl_hdr->img_header[i].flag;
        enc_hdr->length = dl_hdr->img_header[i].length;
        memcpy(enc_hdr->key, env->prog.FW_key, DFU_KEY_SIZE);
        memcpy(enc_hdr->sig, dl_hdr->img_header[i].sig, DFU_SIG_SIZE);
        //TODO how to handle ver in flash.
    }
    dfu_update_sec_flash(cache);
    free((uint8_t *)cache);
}

void dfu_update_img_header_ext(dfu_ctrl_ext_env_t *env)
{
    dfu_dl_image_header_t *dl_hdr = &env->prog.fw_context.code_img;
    struct image_header_enc *enc_hdr;
    struct sec_configuration *cache = (struct sec_configuration *)malloc(sizeof(struct sec_configuration));
    OS_ASSERT(cache);
    memcpy((uint8_t *)cache, (uint8_t *)g_sec_config, sizeof(struct sec_configuration));

    for (uint32_t i  = 0; i < dl_hdr->img_count; i++)
    {
        enc_hdr = dfu_get_enc_image_header(dl_hdr->img_header[i].img_id, cache);
        if (enc_hdr == NULL)
            continue;
        enc_hdr->blksize = dl_hdr->blk_size;
        enc_hdr->flags = dl_hdr->img_header[i].flag;
        enc_hdr->length = dl_hdr->img_header[i].length;
        memcpy(enc_hdr->key, env->prog.FW_key, DFU_KEY_SIZE);
        memcpy(enc_hdr->sig, dl_hdr->img_header[i].sig, DFU_SIG_SIZE);
        //TODO how to handle ver in flash.
    }

    // TODO: update nand
    //dfu_update_sec_flash(cache);
    free((uint8_t *)cache);
}


void dfu_bootjump_sec_config(dfu_ctrl_env_t *env, uint8_t *dest)
{
#if OTA_55X
    dfu_dl_image_header_t *bin_header = &env->prog.fw_context.code_img;
    struct image_header_enc *img_hdr = NULL;
    if (env->prog.dfu_ID == DFU_ID_CODE ||
            env->prog.dfu_ID == DFU_ID_CODE_MIX)
        img_hdr = dfu_get_enc_image_header(DFU_IMG_ID_HCPU, NULL);

    if (!img_hdr)
        return;

    if ((img_hdr->flags & DFU_FLAG_ENC) == 0)
        return;

    if (is_addr_in_flash((uint32_t)dest))
    {
        // Setup XIP for decoding and running
        rt_flash_set_ctr_nonce((uint32_t)dest, (uint32_t)dest, (uint32_t)dest + img_hdr->length,
                               dfu_get_counter(0));
        rt_flash_set_alias((uint32_t)dest, (uint32_t)dest, img_hdr->length, 0);

        /* copy encrypted key to ram as AES_ACC cannot access flash */
        memcpy(dfu_key, img_hdr->key, sizeof(dfu_key));
        sifli_hw_init_xip_key(dfu_key);
        /* enable on-the-fly decoder */
        rt_flash_enable_aes((uint32_t)dest, 1);
    }
    else
    {
        /* Not handle yet. */
    }

#endif /* OTA_55X */
}


/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
