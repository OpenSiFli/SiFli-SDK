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
#ifdef PKG_SIFLI_MBEDTLS_BOOT
    #include "mbedtls/cipher.h"
    #include "mbedtls/pk.h"
    #include "mbedtls/sha256.h"
#endif
#include "dfu_internal.h"

#include "log.h"
struct sec_configuration *g_sec_config;

#ifdef PKG_SIFLI_MBEDTLS_BOOT
    static mbedtls_sha256_context ctx2;
#endif

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

static uint8_t g_fake_sig_pub[DFU_SIG_KEY_SIZE] =
{
    0x30, 0x82, 0x01, 0x22, 0x30, 0x0D, 0x06, 0x09, 0x2A, 0x86, 0x48, 0x86,
    0xF7, 0x0D, 0x01, 0x01, 0x01, 0x05, 0x00, 0x03, 0x82, 0x01, 0x0F, 0x00,
    0x30, 0x82, 0x01, 0x0A, 0x02, 0x82, 0x01, 0x01, 0x00, 0x8B, 0x5B, 0x29,
    0x5A, 0x57, 0x3A, 0x7E, 0x98, 0x0A, 0xC5, 0x4E, 0x50, 0x4C, 0x64, 0xFA,
    0xF2, 0x5B, 0x7B, 0x42, 0x4B, 0x70, 0xD7, 0x75, 0x0F, 0xE5, 0x46, 0x45,
    0x83, 0x73, 0xB8, 0xE3, 0x53, 0x57, 0x5F, 0xCD, 0xB3, 0xA0, 0x88, 0x8A,
    0x7E, 0x55, 0xCD, 0xDE, 0x11, 0x51, 0x21, 0x81, 0xE9, 0xB2, 0xAC, 0x28,
    0x00, 0x90, 0x2D, 0x88, 0x76, 0xD1, 0x0C, 0x8C, 0x62, 0xB6, 0xF6, 0x6F,
    0xB8, 0x05, 0x07, 0xAE, 0x86, 0x2A, 0x1D, 0x78, 0x5B, 0xA4, 0x7D, 0x32,
    0xE4, 0x18, 0x03, 0x58, 0xB6, 0xED, 0x54, 0xDC, 0xFB, 0x50, 0xD1, 0x5E,
    0x4C, 0xF5, 0x9C, 0xC3, 0xBC, 0x27, 0x49, 0xBC, 0x9A, 0x2A, 0x5A, 0x5E,
    0x9F, 0xB3, 0xF7, 0x47, 0x54, 0x8F, 0xED, 0xDB, 0xCE, 0x58, 0x23, 0xFC,
    0x27, 0x3D, 0x17, 0xC1, 0x67, 0x80, 0x9A, 0x7D, 0xC2, 0x42, 0xC7, 0x93,
    0x07, 0x59, 0xDE, 0x8C, 0xDD, 0x4F, 0xD3, 0x7E, 0xAA, 0x0B, 0x91, 0xE9,
    0x41, 0x4C, 0xF9, 0x03, 0xB9, 0x19, 0x62, 0x12, 0x6F, 0x92, 0xCB, 0x7E,
    0x45, 0x1E, 0x72, 0xEF, 0xA0, 0x8A, 0x05, 0x14, 0x6B, 0xA0, 0xB9, 0xF8,
    0x47, 0xAD, 0xA8, 0x90, 0xD6, 0xE6, 0xD4, 0x18, 0x0B, 0xFE, 0x2A, 0x50,
    0xD8, 0x74, 0x05, 0x98, 0xAF, 0xC5, 0x29, 0x11, 0x80, 0x29, 0x85, 0xC2,
    0x36, 0xB6, 0xEB, 0x89, 0x3B, 0xDE, 0x37, 0x23, 0xF8, 0xB8, 0xBD, 0xB0,
    0x94, 0x59, 0x86, 0x5E, 0xD0, 0xC7, 0xEF, 0x86, 0x71, 0x22, 0x31, 0x7D,
    0x24, 0x47, 0xF6, 0x0D, 0x61, 0xFE, 0x22, 0x86, 0x8E, 0x37, 0x93, 0x6B,
    0x03, 0xC4, 0x5E, 0xDA, 0x0B, 0xE3, 0x20, 0xFB, 0xD6, 0x0B, 0x90, 0x31,
    0x8D, 0xEA, 0x1A, 0x9C, 0xB7, 0x4C, 0x54, 0x45, 0x4E, 0x27, 0x65, 0xCA,
    0xAF, 0xB1, 0xD4, 0xE0, 0xE9, 0xB8, 0xC5, 0x44, 0x7F, 0xAF, 0xC3, 0x34,
    0xCF, 0x02, 0x03, 0x01, 0x00, 0x01
};

__WEAK uint8_t *dfu_get_public_key_hook()
{
    return &g_fake_sig_pub[0];
}

static uint8_t *dfu_get_public_key(void)
{
    return dfu_get_public_key_hook();
    // return &g_sec_config->sig_pub_key[0];
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
    g_sec_config = malloc(sizeof(struct sec_configuration));
    dfu_packet_read_flash_ext(0x62000000, 0, (uint8_t *)g_sec_config, sizeof(struct sec_configuration));
#else
    g_sec_config = (struct sec_configuration *)0x1c000000;
#endif
#endif
    if (dfu_secure_boot_check() == 0)
    {
        LOG_I("dfu_secure_boot_check");
        g_dfu_efuse_read_hook = dfu_get_efuse_hook;
    }


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
    LOG_HEX("g_aes_ctr_iv", 16, g_aes_ctr_iv, DFU_SIG_HASH_SIZE);
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
#ifdef PKG_SIFLI_MBEDTLS_BOOT
    uint8_t *r = NULL;

    uint8_t *temp_out;
    int align_size = (size / 16 + 1) * 16;
    temp_out = rt_malloc(align_size);
    OS_ASSERT(temp_out);

    SCB_InvalidateDCache_by_Addr(temp_out, align_size);
    SCB_InvalidateICache_by_Addr(temp_out, align_size);

    sifli_hw_dec(key, in_data, temp_out, size, offset);
    memcpy(out_data, temp_out, size);
    rt_free(temp_out);

    mbedtls_sha256_init(&ctx2);
    mbedtls_sha256_starts(&ctx2, 0); /* SHA-256, not 224 */
    mbedtls_sha256_update(&ctx2, out_data, size);

    uint8_t temp_hash[32] = {0};
    mbedtls_sha256_finish(&ctx2, temp_hash);
    memcpy(dfu_hash, temp_hash, DFU_SIG_HASH_SIZE);

    if (memcmp(dfu_hash, hash, DFU_SIG_HASH_SIZE) == 0)
        r = out_data;
    return r;
#else
    return 0;
#endif
}

int8_t dfu_integrate_verify(uint8_t *in_data, int size, uint8_t *hash)
{
#ifdef PKG_SIFLI_MBEDTLS_BOOT
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
#else
    return 0;
#endif
}


uint8_t dfu_img_verification(dfu_ctrl_env_t *env)
{
#ifdef PKG_SIFLI_MBEDTLS_BOOT
#ifdef OTA_55X
    dfu_img_info_t *curr_info = &env->prog.fw_context.code_img.curr_img_info;
    if (env->mode == DFU_CTRL_NORMAL_MODE && env->ota_state.dfu_ID == DFU_ID_OTA_MANAGER)
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
    uint32_t cal_count = 0;
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
            cal_count++;
            if (cal_count % DFU_HASH_VERIFY_WDT_PET_FREQUENCY == 0)
            {
#ifdef RT_USING_WDT
                uint32_t status = rt_hw_watchdog_get_status();
                if (0 != status)
                {
                    rt_hw_watchdog_pet();
                }
#endif
            }
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
#else
    return 0;
#endif
}

uint8_t dfu_img_verification_ext(dfu_ctrl_ext_env_t *env)
{
#ifdef PKG_SIFLI_MBEDTLS_BOOT
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
    uint32_t cal_count = 0;
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
            cal_count++;
            if (cal_count % DFU_HASH_VERIFY_WDT_PET_FREQUENCY == 0)
            {
#ifdef RT_USING_WDT
                uint32_t status = rt_hw_watchdog_get_status();
                if (0 != status)
                {
                    rt_hw_watchdog_pet();
                }
#endif
            }
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
#else
    return 0;
#endif
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
#ifdef PKG_SIFLI_MBEDTLS_BOOT
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
#else
    return 0;
#endif
}

int8_t dfu_ctrl_ctrl_header_sig_verify_ext(uint8_t *packet, uint16_t total_len, uint8_t *sig)
{
#ifdef PKG_SIFLI_MBEDTLS_BOOT
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
#else
    return 0;
#endif
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
#if 0
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
#endif
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

static uint32_t CrcTable[256] =
{
    0x00000000, 0x04c11db7, 0x09823b6e, 0x0d4326d9, 0x130476dc, 0x17c56b6b,
    0x1a864db2, 0x1e475005, 0x2608edb8, 0x22c9f00f, 0x2f8ad6d6, 0x2b4bcb61,
    0x350c9b64, 0x31cd86d3, 0x3c8ea00a, 0x384fbdbd, 0x4c11db70, 0x48d0c6c7,
    0x4593e01e, 0x4152fda9, 0x5f15adac, 0x5bd4b01b, 0x569796c2, 0x52568b75,
    0x6a1936c8, 0x6ed82b7f, 0x639b0da6, 0x675a1011, 0x791d4014, 0x7ddc5da3,
    0x709f7b7a, 0x745e66cd, 0x9823b6e0, 0x9ce2ab57, 0x91a18d8e, 0x95609039,
    0x8b27c03c, 0x8fe6dd8b, 0x82a5fb52, 0x8664e6e5, 0xbe2b5b58, 0xbaea46ef,
    0xb7a96036, 0xb3687d81, 0xad2f2d84, 0xa9ee3033, 0xa4ad16ea, 0xa06c0b5d,
    0xd4326d90, 0xd0f37027, 0xddb056fe, 0xd9714b49, 0xc7361b4c, 0xc3f706fb,
    0xceb42022, 0xca753d95, 0xf23a8028, 0xf6fb9d9f, 0xfbb8bb46, 0xff79a6f1,
    0xe13ef6f4, 0xe5ffeb43, 0xe8bccd9a, 0xec7dd02d, 0x34867077, 0x30476dc0,
    0x3d044b19, 0x39c556ae, 0x278206ab, 0x23431b1c, 0x2e003dc5, 0x2ac12072,
    0x128e9dcf, 0x164f8078, 0x1b0ca6a1, 0x1fcdbb16, 0x018aeb13, 0x054bf6a4,
    0x0808d07d, 0x0cc9cdca, 0x7897ab07, 0x7c56b6b0, 0x71159069, 0x75d48dde,
    0x6b93dddb, 0x6f52c06c, 0x6211e6b5, 0x66d0fb02, 0x5e9f46bf, 0x5a5e5b08,
    0x571d7dd1, 0x53dc6066, 0x4d9b3063, 0x495a2dd4, 0x44190b0d, 0x40d816ba,
    0xaca5c697, 0xa864db20, 0xa527fdf9, 0xa1e6e04e, 0xbfa1b04b, 0xbb60adfc,
    0xb6238b25, 0xb2e29692, 0x8aad2b2f, 0x8e6c3698, 0x832f1041, 0x87ee0df6,
    0x99a95df3, 0x9d684044, 0x902b669d, 0x94ea7b2a, 0xe0b41de7, 0xe4750050,
    0xe9362689, 0xedf73b3e, 0xf3b06b3b, 0xf771768c, 0xfa325055, 0xfef34de2,
    0xc6bcf05f, 0xc27dede8, 0xcf3ecb31, 0xcbffd686, 0xd5b88683, 0xd1799b34,
    0xdc3abded, 0xd8fba05a, 0x690ce0ee, 0x6dcdfd59, 0x608edb80, 0x644fc637,
    0x7a089632, 0x7ec98b85, 0x738aad5c, 0x774bb0eb, 0x4f040d56, 0x4bc510e1,
    0x46863638, 0x42472b8f, 0x5c007b8a, 0x58c1663d, 0x558240e4, 0x51435d53,
    0x251d3b9e, 0x21dc2629, 0x2c9f00f0, 0x285e1d47, 0x36194d42, 0x32d850f5,
    0x3f9b762c, 0x3b5a6b9b, 0x0315d626, 0x07d4cb91, 0x0a97ed48, 0x0e56f0ff,
    0x1011a0fa, 0x14d0bd4d, 0x19939b94, 0x1d528623, 0xf12f560e, 0xf5ee4bb9,
    0xf8ad6d60, 0xfc6c70d7, 0xe22b20d2, 0xe6ea3d65, 0xeba91bbc, 0xef68060b,
    0xd727bbb6, 0xd3e6a601, 0xdea580d8, 0xda649d6f, 0xc423cd6a, 0xc0e2d0dd,
    0xcda1f604, 0xc960ebb3, 0xbd3e8d7e, 0xb9ff90c9, 0xb4bcb610, 0xb07daba7,
    0xae3afba2, 0xaafbe615, 0xa7b8c0cc, 0xa379dd7b, 0x9b3660c6, 0x9ff77d71,
    0x92b45ba8, 0x9675461f, 0x8832161a, 0x8cf30bad, 0x81b02d74, 0x857130c3,
    0x5d8a9099, 0x594b8d2e, 0x5408abf7, 0x50c9b640, 0x4e8ee645, 0x4a4ffbf2,
    0x470cdd2b, 0x43cdc09c, 0x7b827d21, 0x7f436096, 0x7200464f, 0x76c15bf8,
    0x68860bfd, 0x6c47164a, 0x61043093, 0x65c52d24, 0x119b4be9, 0x155a565e,
    0x18197087, 0x1cd86d30, 0x029f3d35, 0x065e2082, 0x0b1d065b, 0x0fdc1bec,
    0x3793a651, 0x3352bbe6, 0x3e119d3f, 0x3ad08088, 0x2497d08d, 0x2056cd3a,
    0x2d15ebe3, 0x29d4f654, 0xc5a92679, 0xc1683bce, 0xcc2b1d17, 0xc8ea00a0,
    0xd6ad50a5, 0xd26c4d12, 0xdf2f6bcb, 0xdbee767c, 0xe3a1cbc1, 0xe760d676,
    0xea23f0af, 0xeee2ed18, 0xf0a5bd1d, 0xf464a0aa, 0xf9278673, 0xfde69bc4,
    0x89b8fd09, 0x8d79e0be, 0x803ac667, 0x84fbdbd0, 0x9abc8bd5, 0x9e7d9662,
    0x933eb0bb, 0x97ffad0c, 0xafb010b1, 0xab710d06, 0xa6322bdf, 0xa2f33668,
    0xbcb4666d, 0xb8757bda, 0xb5365d03, 0xb1f740b4
};

uint32_t dfu_crc32mpeg2(uint8_t *data, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFF;

    for (uint32_t i = 0; i < len; i++)
    {
        crc = (crc << 8) ^ CrcTable[((crc >> 24) ^ *data++) & 0xFF];
    }
    return crc;
}

// calculate CRC32 MPEG2 using part data
uint32_t crc32_update(uint32_t crc, const uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; i++)
    {
        crc = (crc << 8) ^ CrcTable[((crc >> 24) ^ data[i]) & 0xFF];
    }
    return crc;
}

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
