/**
  ******************************************************************************
  * @file   dfu.c
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

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <rtthread.h>
#include "register.h"
#ifdef FINSH_USING_MSH
    #include "shell.h"
    #include "msh.h"
#endif
#include "dfu.h"
#include "drv_flash.h"
#include "drv_mpi.h"
#include "mbedtls/cipher.h"
#include "mbedtls/pk.h"
#include "mbedtls/sha256.h"
#ifdef BSP_USING_DFU_COMPRESS
    #include "zlib.h"
#endif

#ifdef CFG_BOOTLOADER
    #define DBG_ENABLE
#endif /* CFG_BOOTLOADER */

#define LOG_TAG "dfu"
#define DBG_LVL    DBG_INFO
#include "log.h"

#define IS_MPI_ADDR(addr, i) ((addr >= MPI##i##_MEM_BASE) && (addr < (MPI##i##_MEM_BASE + QSPI##i##_MAX_SIZE)))



static uint8_t dfu_temp[DFU_MAX_BLK_SIZE];
static uint8_t dfu_data[DFU_MAX_BLK_SIZE];
static uint8_t dfu_hash[DFU_SIG_HASH_SIZE];
struct sec_configuration sec_config_cache;

#ifdef BSP_USING_DFU_COMPRESS
    struct image_header_compress g_dfu_img_progress;
#endif

volatile struct dfu_compress_configuration *g_dfu_compress_config;

/** encoded key */
ALIGN(4)
static uint8_t dfu_key[DFU_KEY_SIZE];
/** key in plaintext */
ALIGN(4)
static uint8_t dfu_key1[DFU_KEY_SIZE];
struct sec_configuration *g_sec_config;
struct sec_configuration *g_config_cache;
struct dfu_configuration *g_dfu_config;
static uint8_t g_sig_hash_cache[DFU_SIG_HASH_SIZE];
ALIGN(4)
static uint8_t g_aes_ctr_iv[DFU_IV_LEN];
static struct image_header_enc g_boot_patch_img_head;
static uint8_t g_boot_patch_sig_pub_key[DFU_SIG_KEY_SIZE];
static int img_flashid(int coreid)
{
    return coreid + 2;
}
//#define DFU_DEV
#ifdef DFU_DEV
    extern int sifli_hw_efuse_write_din(uint8_t id, uint8_t *data, int size);
#endif

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

static mbedtls_sha256_context ctx2;
uint8_t *sifli_dec_verify(uint8_t *key, uint32_t offset,
                          uint8_t *in_data, uint8_t *out_data, int size, uint8_t *hash)
{
    uint8_t *r = NULL;

    sifli_hw_dec(key, in_data, out_data, size, offset);
    mbedtls_sha256_init(&ctx2);
    mbedtls_sha256_starts(&ctx2, 0); /* SHA-256, not 224 */
    mbedtls_sha256_update(&ctx2, out_data, size);
    mbedtls_sha256_finish(&ctx2, dfu_hash);
    if (memcmp(dfu_hash, hash, DFU_SIG_HASH_SIZE) == 0)
        r = out_data;
    return r;
}


static int8_t sifli_sig_key_verify(uint8_t *sig_pub_key)
{
    int r = DFU_FAIL;
    uint8_t sig_hash[DFU_SIG_HASH_SIZE];
    if (sifli_hw_efuse_read(EFUSE_ID_SIG_HASH, (uint8_t *)&sig_hash, DFU_SIG_HASH_SIZE) == 0)
        return DFU_FAIL;

    mbedtls_sha256_init(&ctx2);
    mbedtls_sha256_starts(&ctx2, 0); /* SHA-256, not 224 */
    mbedtls_sha256_update(&ctx2, sig_pub_key, DFU_SIG_KEY_SIZE);
    mbedtls_sha256_finish(&ctx2, dfu_hash);
    if (memcmp(dfu_hash, sig_hash, DFU_SIG_HASH_SIZE) == 0)
        r = DFU_SUCCESS;
    return r;
}


#ifdef BSP_USING_DFU_COMPRESS
static uint32_t dfu_compress_check_dup_image(uint8_t flashid)
{
    uint32_t index = 0xFF;
    if (g_dfu_compress_config->img_count)
    {
        uint32_t i = g_dfu_compress_config->img_count;
        while (i--)
        {
            if (flashid == g_dfu_compress_config->imgs[i].compress_img_id)
            {
                index = i;
                break;
            }
        }
    }
    return index;
}
#endif

static int dfu_process_hdr_sec(uint8_t flashid, uint8_t *data, int size)
{
    int r = DFU_FAIL;
    uint8_t *d;
    struct image_cfg_hdr *hdr = (struct image_cfg_hdr *)data;
    int img_idx = DFU_FLASH_IMG_IDX(flashid);

#ifdef BSP_USING_DFU_COMPRESS
    uint8_t compress_img_idx = DFU_FLASH_IMG_COMPRESS_IDX(flashid);
    if (DFU_FLASH_IMG_COMPRESS_FLASH(flashid) == DFU_FLASH_COMPRESS)
    {
        flashid = DFU_FLASH_IMG_COMPRESS_FLASH(flashid);
    }
#endif

    if (
#ifdef BSP_USING_DFU_COMPRESS
        (flashid > DFU_FLASH_COMPRESS)
#else
        (flashid >= DFU_FLASH_PARTITION)
#endif
        || (flashid < DFU_FLASH_IMG_LCPU)
    )
    {
        return r;
    }



    data += sizeof(struct image_cfg_hdr);
    size -= sizeof(struct image_cfg_hdr);

    d = sifli_dec_verify(NULL, 0, data, dfu_temp, size, hdr->hash);
    if (d)          // Use root key to decode image header
    {
        struct image_header_enc *hdr = (struct image_header_enc *)d;

        // encrypt image key with root key+UID
        sifli_hw_enc(hdr->key, dfu_key, DFU_KEY_SIZE);
        /* save image_key in plaintext */
        memcpy(dfu_key1, hdr->key, DFU_KEY_SIZE);
        memcpy(hdr->key, dfu_key, DFU_KEY_SIZE);


        if (DFU_FLASH_SINGLE == flashid)
        {
            img_idx = DFU_FLASH_IMG_IDX(DFU_FLASH_IMG_HCPU);
            hdr->flags |= DFU_FLAG_SINGLE;
        }
        if (DFU_FLASH_IMG_BOOT2 == flashid)
        {
            memcpy(&g_boot_patch_img_head, d, sizeof(g_boot_patch_img_head));
        }
#ifdef BSP_USING_DFU_COMPRESS
        else if (DFU_FLASH_COMPRESS == flashid)
        {
            uint32_t index = dfu_compress_check_dup_image(compress_img_idx);


            struct dfu_compress_configuration *config = rt_malloc(sizeof(struct dfu_compress_configuration));
            RT_ASSERT(config);
            //rt_memset(config, 0, sizeof(struct dfu_compress_configuration));
            rt_memcpy(config, (const void *)g_dfu_compress_config, sizeof(struct dfu_compress_configuration));

            rt_memcpy(&g_dfu_img_progress.img.enc_img, hdr, sizeof(struct image_header_enc));
            g_dfu_img_progress.state = DFU_STATE_BIN_DOWNLOADING;
            g_dfu_img_progress.compress_img_id = compress_img_idx;
            g_dfu_img_progress.current_img_len = 0;
            g_dfu_img_progress.current_packet_count = 0;
            if (index != 0xFF)
            {
                // index already existed, just overwrite.
                rt_memcpy((void *)&config->imgs[index], (const void *)&g_dfu_img_progress, sizeof(struct image_header_compress));
            }
            else
            {
                config->ctab[config->img_count].base = DFU_RES_FLASH_CODE_START_ADDR + 0x1000;
                config->ctab[config->img_count].size = DFU_RES_FLASH_CODE_SIZE;
                rt_memcpy((void *)&config->imgs[config->img_count++], (const void *)&g_dfu_img_progress, sizeof(struct image_header_compress));
            }

            //LOG_HEX("dfu_config", 16, (uint8_t *)config, sizeof(struct dfu_compress_configuration));
            sec_flash_erase(DFU_FLASH_COMPRESS_CONFIG, 0, SPI_NOR_SECT_SIZE);
            sec_flash_write(DFU_FLASH_COMPRESS_CONFIG, 0, (uint8_t *)config, sizeof(struct dfu_compress_configuration));
            rt_free(config);
        }
#endif
        else
        {
            sec_flash_update(DFU_FLASH_SEC_CONFIG, SECFG_IMG_OFFSET + img_idx * sizeof(struct image_header_enc), d, size);
        }

        // Save to flash and erase the image flash section.
        sec_flash_erase(flashid, 0, hdr->length);
        r = DFU_SUCCESS;
    }
    return r;
}

static int dfu_process_body_sec(uint8_t flashid, uint8_t *data, int size)
{
    int r = DFU_FAIL;
    struct image_body_hdr *hdr = (struct image_body_hdr *)data;
    uint8_t *key;
    uint8_t *d;
    int flash_img_idx = DFU_FLASH_IMG_IDX(flashid);
    struct image_header_enc *img_hdr;

#ifdef BSP_USING_DFU_COMPRESS
    if (DFU_FLASH_IMG_COMPRESS_FLASH(flashid) == DFU_FLASH_COMPRESS)
    {
        flashid = DFU_FLASH_IMG_COMPRESS_FLASH(flashid);
    }
#endif


    if (
#ifdef BSP_USING_DFU_COMPRESS
        (flashid > DFU_FLASH_COMPRESS)
#else
        (flashid >= DFU_FLASH_PARTITION)
#endif
        || (flashid < DFU_FLASH_IMG_LCPU)
    )
    {
        return r;
    }


    // Read image key
    //key = &(g_sec_config->imgs[coreid].key[0]);
    key = dfu_key1;
    size -= sizeof(struct image_body_hdr);
    data += sizeof(struct image_body_hdr);
    d = sifli_dec_verify(key, hdr->offset, data, dfu_temp, size, hdr->hash);
    if (d)
    {
        if (DFU_FLASH_IMG_BOOT2 == flashid)
        {
            img_hdr = &g_boot_patch_img_head;
        }
#ifdef BSP_USING_DFU_COMPRESS
        else if (DFU_FLASH_COMPRESS == flashid)
        {
            img_hdr = &g_dfu_img_progress.img.enc_img;
            g_dfu_img_progress.current_img_len += size + sizeof(struct image_body_hdr);
            g_dfu_img_progress.current_packet_count++;
            if ((g_dfu_img_progress.current_packet_count % 4) == 0)
            {
                struct dfu_compress_configuration *config = rt_malloc(sizeof(struct dfu_compress_configuration));
                // All other app should stop in this scenario, heap should be enough
                RT_ASSERT(config);
                rt_memcpy(config, (const void *)g_dfu_compress_config, sizeof(struct dfu_compress_configuration));
                rt_memcpy(&config->imgs[config->img_count - 1], &g_dfu_img_progress, sizeof(struct image_header_compress));
                sec_flash_erase(DFU_FLASH_COMPRESS_CONFIG, 0, SPI_NOR_SECT_SIZE);
                sec_flash_write(DFU_FLASH_COMPRESS_CONFIG, 0, (uint8_t *)config, sizeof(struct dfu_compress_configuration));
                rt_free(config);
            }

        }
#endif
        else
        {
            img_hdr = &g_sec_config->imgs[flash_img_idx];
        }
        if (img_hdr->flags & DFU_FLAG_ENC)
            sec_flash_write(flashid, hdr->offset, data, size);
        else
            sec_flash_write(flashid, hdr->offset, d, size);
        LOG_I("%d ", hdr->offset + size);
        r = DFU_SUCCESS;
    }
    else
    {
        LOG_E("body verify fail\n");
    }

    return r;
}

static int dfu_process_config_sec(uint8_t keyid, uint8_t *data, int len)
{
    int r = DFU_FAIL;

    if (keyid < DFU_CONFIG_FLASH_TABLE)
    {
        sifli_hw_efuse_write(keyid, data, len);
        r = DFU_SUCCESS;
    }
    else if ((keyid == DFU_CONFIG_SIG)
             || (keyid == DFU_CONFIG_BOOT_PATCH_SIG))
    {
        int i;
        mbedtls_sha256_context ctx2;
        mbedtls_sha256_init(&ctx2);
        mbedtls_sha256_starts(&ctx2, 0); /* SHA-256, not 224 */
        mbedtls_sha256_update(&ctx2, data, len);
        mbedtls_sha256_finish(&ctx2, dfu_temp);
        sifli_hw_efuse_read(EFUSE_ID_SIG_HASH, g_sig_hash_cache, DFU_SIG_HASH_SIZE);
        for (i = 0; i < DFU_SIG_HASH_SIZE; i++)
            if (g_sig_hash_cache[i] != dfu_temp[i])
                break;
        if (i == DFU_SIG_HASH_SIZE)
        {
            r = DFU_SUCCESS;
            if (keyid == DFU_CONFIG_SIG)
            {
                sec_flash_write(DFU_FLASH_SEC_CONFIG, SECFG_SIGKEY_OFFSET, data, len);
            }
            else
            {
                memcpy(&g_boot_patch_sig_pub_key, data, sizeof(g_boot_patch_sig_pub_key));
            }
        }
    }
    else if (keyid == DFU_CONFIG_FLASH_TABLE)
    {
        struct image_cfg_hdr *hdr = (struct image_cfg_hdr *)data;
        uint8_t *d;

        data += sizeof(struct image_cfg_hdr);
        len -= sizeof(struct image_cfg_hdr);
        d = sifli_dec_verify(NULL, 0, data, dfu_temp, len, hdr->hash);
        if (d)
        {
            sec_flash_update(DFU_FLASH_SEC_CONFIG, SECFG_FTAB_OFFSET, d, len);
            r = DFU_SUCCESS;
        }
    }
    return r;
}

static int dfu_end(uint8_t flashid)
{
    int offset = 0;
    int size = 0;
    int r = DFU_FAIL;
    int img_idx;
    struct image_header_enc *img_hdr;
    uint8_t *sig_pub_key;

#ifdef BSP_USING_DFU_COMPRESS
    if (DFU_FLASH_IMG_COMPRESS_FLASH(flashid) == DFU_FLASH_COMPRESS)
    {
        flashid = DFU_FLASH_IMG_COMPRESS_FLASH(flashid);
    }
#endif



    if (
#ifdef BSP_USING_DFU_COMPRESS
        (flashid > DFU_FLASH_COMPRESS)
#else
        (flashid >= DFU_FLASH_PARTITION)
#endif
        || (flashid < DFU_FLASH_IMG_LCPU)
    )
    {
        return r;
    }


    if (DFU_FLASH_SINGLE == flashid)
    {
        img_idx = DFU_FLASH_IMG_IDX(DFU_FLASH_IMG_HCPU);
    }
    else
    {
        img_idx = DFU_FLASH_IMG_IDX(flashid);
    }

    if (DFU_FLASH_IMG_BOOT2 == flashid)
    {
        img_hdr = &g_boot_patch_img_head;
        sig_pub_key = &g_boot_patch_sig_pub_key[0];
    }
#ifdef BSP_USING_DFU_COMPRESS
    else if (DFU_FLASH_COMPRESS == flashid)
    {
        img_hdr = &g_dfu_img_progress.img.enc_img;
        sig_pub_key = &g_sec_config->sig_pub_key[0];
    }
#endif
    else
    {
        img_hdr = &g_sec_config->imgs[img_idx];
        sig_pub_key = &g_sec_config->sig_pub_key[0];
    }

    {
        // Verify sig key.
        if (sifli_sig_key_verify(sig_pub_key) != DFU_SUCCESS)
            return r;
    }

    {
        // Calculate HASH
        mbedtls_sha256_context ctx2;
        mbedtls_sha256_init(&ctx2);
        mbedtls_sha256_starts(&ctx2, 0); /* SHA-256, not 224 */
        do
        {
            if (offset + sizeof(dfu_temp) > img_hdr->length)
                size = img_hdr->length - offset;
            else
                size = sizeof(dfu_temp);
            if (size)
            {
                sec_flash_read(flashid, offset, dfu_temp, size);
                mbedtls_sha256_update(&ctx2, dfu_temp, size);
            }
            if (size < sizeof(dfu_temp))
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
                if (flashid == DFU_FLASH_IMG_BOOT2) //patch in RAM, install directly
                {
                    boot_init_hook bp = (boot_init_hook)(BOOTLOADER_PATCH_CODE_ADDR | 1);
                    bp();
                }
#ifdef BSP_USING_DFU_COMPRESS
                else if (DFU_FLASH_COMPRESS == flashid)
                {
                    // Verify decompress public key
                    dfu_bin_verify(dfu_key1);
                }
#endif
                else
                {
                    if (img_idx < CORE_MAX * 2)
                    {
                        uint8_t *running_img = (uint8_t *) & (g_sec_config->imgs[img_idx]);

                        sec_flash_update(DFU_FLASH_SEC_CONFIG, SECFG_RUNIMG_OFFSET + (img_idx % CORE_MAX) * sizeof(struct image_header_enc *),
                                         (uint8_t *)&running_img, sizeof(struct image_header_enc *));
                    }
                }
                r = DFU_SUCCESS;
            }
        }
    }

#ifdef BSP_USING_DFU_COMPRESS
    {
        struct dfu_compress_configuration *config = rt_malloc(sizeof(struct dfu_compress_configuration));
        // All other app should stop in this scenario, heap should be enough
        RT_ASSERT(config);
        rt_memcpy(config, (const void *)g_dfu_compress_config, sizeof(struct dfu_compress_configuration));
        g_dfu_img_progress.state = r == DFU_SUCCESS ? DFU_STATE_BIN_DOWNLOADED : DFU_STATE_BIN_READY;
        rt_memcpy(&config->imgs[config->img_count - 1], &g_dfu_img_progress, sizeof(struct image_header_compress));
        sec_flash_erase(DFU_FLASH_COMPRESS_CONFIG, 0, SPI_NOR_SECT_SIZE);
        sec_flash_write(DFU_FLASH_COMPRESS_CONFIG, 0, (uint8_t *)config, sizeof(struct dfu_compress_configuration));
        rt_free(config);
    }
#endif
    return r;
}

#ifdef BSP_USING_DFU_COMPRESS
static void dfu_get_resource_information(uint8_t flashid, uint32_t *addr, uint32_t *size)
{
    uint32_t addr_l;
    uint32_t size_l;
    switch (flashid)
    {
    case DFU_FLASH_IMAGE:
    {
        addr_l = HCPU_FLASH2_IMG_START_ADDR;
        size_l = HCPU_FLASH2_IMG_SIZE;
        break;
    }
    case DFU_FLASH_FONT:
    {
        addr_l = HCPU_FLASH2_FONT_START_ADDR;
        size_l = HCPU_FLASH2_FONT_SIZE;
        break;
    }
    case DFU_FLASH_IMAGE_COMPRESS:
    {
        addr_l = HCPU_FLASH2_IMG_UPGRADE_START_ADDR;
        size_l = HCPU_FLASH2_IMG_UPGRADE_SIZE;
        break;
    }
    case DFU_FLASH_FONT_COMPRESS:
    {
        addr_l = HCPU_FLASH2_FONT_UPGRADE_START_ADDR;
        size_l = HCPU_FLASH2_FONT_UPGRADE_SIZE;
        break;
    }
    default:
    {
        addr_l = NULL;
        size_l = 0;
        break;
    }
    }
    if (addr)
        *addr = addr_l;
    if (size)
        *size = size_l;

}

static int dfu_process_hdr(uint8_t flashid, uint8_t *data, int size)
{
    int r = DFU_FAIL;
    uint8_t *d;

#if 0
#ifdef BSP_USING_DFU_COMPRESS
    uint8_t compress_img_idx;
    if (flashid == DFU_FLASH_IMAGE_COMPRESS)
        compress_img_idx = DFU_FLASH_IMAGE;
    else if (flashid == DFU_FLASH_FONT_COMPRESS)
        compress_img_idx = DFU_FLASH_FONT;
#endif
#endif

    if (flashid < DFU_FLASH_IMAGE ||
#ifndef BSP_USING_DFU_COMPRESS
            flashid > DFU_FLASH_FONT)
#else
            flashid > DFU_FLASH_FONT_COMPRESS)
#endif
    {
        return r;
    }

    //int img_idx = DFU_FLASH_IMG_IDX(flashid);

#if 0
    if (flashid == DFU_FLASH_IMAGE ||
            flashid == DFU_FLASH_FONT)
    {
        // Image and font can overwrite directly.
        return DFU_SUCCESS;
    }
#endif


    struct image_header *hdr = (struct image_header *)data;


    // Save to flash and erase the image flash section.

//#ifdef BSP_USING_DFU_COMPRESS
//    if (DFU_FLASH_IMAGE_COMPRESS == flashid ||
//             DFU_FLASH_COMPRESS_FONT == flashid)
    {


        uint32_t index = dfu_compress_check_dup_image(flashid);

        struct dfu_compress_configuration *config = rt_malloc(sizeof(struct dfu_compress_configuration));
        RT_ASSERT(config);
        rt_memcpy(config, (const void *)g_dfu_compress_config, sizeof(struct dfu_compress_configuration));

        dfu_get_resource_information(flashid, &config->ctab[config->img_count].base,
                                     &config->ctab[config->img_count].size);

        rt_memcpy(&g_dfu_img_progress.img.none_enc_img, hdr, sizeof(struct image_header));
        g_dfu_img_progress.state = DFU_STATE_BIN_DOWNLOADING;
        g_dfu_img_progress.compress_img_id = flashid;
        g_dfu_img_progress.current_img_len = 0;
        g_dfu_img_progress.current_packet_count = 0;


        if (index != 0xFF)
        {
            rt_memcpy((void *)&config->imgs[index], (const void *)&g_dfu_img_progress, sizeof(struct image_header_compress));

        }
        else
        {
            dfu_get_resource_information(flashid, &config->ctab[config->img_count].base,
                                         &config->ctab[config->img_count].size);
            if (flashid == DFU_FLASH_IMAGE_COMPRESS)
                dfu_get_resource_information(DFU_FLASH_IMAGE, &g_dfu_img_progress.img.none_enc_img.target_base, NULL);
            else if (flashid == DFU_FLASH_FONT_COMPRESS)
                dfu_get_resource_information(DFU_FLASH_FONT, &g_dfu_img_progress.img.none_enc_img.target_base, NULL);
            else
                g_dfu_img_progress.img.none_enc_img.target_base = 0;
            rt_memcpy((void *)&config->imgs[config->img_count++], (const void *)&g_dfu_img_progress, sizeof(struct image_header_compress));
        }


        sec_flash_erase(DFU_FLASH_COMPRESS_CONFIG, 0, SPI_NOR_SECT_SIZE);
        sec_flash_write(DFU_FLASH_COMPRESS_CONFIG, 0, (uint8_t *)config, sizeof(struct dfu_compress_configuration));
        rt_free(config);
    }
//#endif
    sec_flash_erase(flashid, 0, hdr->length);

    r = DFU_SUCCESS;
    return r;
}


static int dfu_process_body(uint8_t flashid, uint8_t *data, uint32_t offset, int size)
{
    int r = DFU_FAIL;
    uint8_t *key;
    uint8_t *d;
    int flash_img_idx = DFU_FLASH_IMG_IDX(flashid);
    struct image_header *img_hdr;

    if (flashid < DFU_FLASH_IMAGE ||
#ifndef BSP_USING_DFU_COMPRESS
            flashid > DFU_FLASH_FONT)
#else
            flashid > DFU_FLASH_FONT_COMPRESS)
#endif
    {
        return r;
    }


#ifdef BSP_USING_DFU_COMPRESS
    {
        img_hdr = &g_dfu_img_progress.img.none_enc_img;
        g_dfu_img_progress.current_img_len += size;
        g_dfu_img_progress.current_packet_count++;
        if ((g_dfu_img_progress.current_packet_count % 4) == 0)
        {
            struct dfu_compress_configuration *config = rt_malloc(sizeof(struct dfu_compress_configuration));
            // All other app should stop in this scenario, heap should be enough
            RT_ASSERT(config);
            rt_memcpy(config, (const void *)g_dfu_compress_config, sizeof(struct dfu_compress_configuration));
            rt_memcpy(&config->imgs[config->img_count - 1], &g_dfu_img_progress, sizeof(struct image_header_compress));
            sec_flash_erase(DFU_FLASH_COMPRESS_CONFIG, 0, SPI_NOR_SECT_SIZE);
            sec_flash_write(DFU_FLASH_COMPRESS_CONFIG, 0, (uint8_t *)config, sizeof(struct dfu_compress_configuration));
            rt_free(config);
        }

    }
#endif

    sec_flash_write(flashid, offset, data, size);
    r = DFU_SUCCESS;

    return r;
}

static int dfu_end_wo_enc(uint8_t flashid)
{
    int offset = 0;
    int size = 0;
    int r = DFU_FAIL;
    int img_idx;
    struct image_header *img_hdr;
    uint8_t *sig_pub_key;


    if (flashid < DFU_FLASH_IMAGE ||
#ifndef BSP_USING_DFU_COMPRESS
            flashid > DFU_FLASH_FONT)
#else
            flashid > DFU_FLASH_FONT_COMPRESS)
#endif
    {
        return r;
    }

    if (g_dfu_img_progress.compress_img_id != flashid)
        return r;

#if 0
    if (flashid == DFU_FLASH_IMAGE_COMPRESS || flashid == DFU_FLASH_FONT_COMPRESS)
    {
        img_hdr = &g_dfu_img_progress.img.none_enc_img;
        sig_pub_key = &g_sec_config->sig_pub_key[0];

        {
            // Calculate HASH
            mbedtls_sha256_context ctx2;
            mbedtls_sha256_init(&ctx2);
            mbedtls_sha256_starts(&ctx2, 0); /* SHA-256, not 224 */
            do
            {
                if (offset + sizeof(dfu_temp) > img_hdr->length)
                    size = img_hdr->length - offset;
                else
                    size = sizeof(dfu_temp);
                if (size)
                {
                    sec_flash_read(flashid, offset, dfu_temp, size);
                    mbedtls_sha256_update(&ctx2, dfu_temp, size);
                }
                if (size < sizeof(dfu_temp))
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

                    r = DFU_SUCCESS;
                }
            }
        }
    }
#endif

    r = DFU_SUCCESS;
#ifdef BSP_USING_DFU_COMPRESS
    {
        struct dfu_compress_configuration *config = rt_malloc(sizeof(struct dfu_compress_configuration));
        // All other app should stop in this scenario, heap should be enough
        RT_ASSERT(config);
        rt_memcpy(config, (const void *)g_dfu_compress_config, sizeof(struct dfu_compress_configuration));
        if (flashid == DFU_FLASH_IMAGE || flashid == DFU_FLASH_FONT)
        {
            g_dfu_img_progress.state  = r == DFU_SUCCESS ? DFU_STATE_BIN_INSTALLED : DFU_STATE_BIN_NOT_EXISTED;
        }
        else
            g_dfu_img_progress.state = r == DFU_SUCCESS ? DFU_STATE_BIN_DOWNLOADED : DFU_STATE_BIN_READY;
        rt_memcpy(&config->imgs[config->img_count - 1], &g_dfu_img_progress, sizeof(struct image_header_compress));
        sec_flash_erase(DFU_FLASH_COMPRESS_CONFIG, 0, SPI_NOR_SECT_SIZE);
        sec_flash_write(DFU_FLASH_COMPRESS_CONFIG, 0, (uint8_t *)config, sizeof(struct dfu_compress_configuration));
        rt_free(config);
    }
#endif
    return r;
}
#endif

RT_WEAK uint8_t is_addr_in_flash(uint32_t addr)
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


void run_img(uint8_t *dest)
{
    __asm("LDR SP, [%0]" :: "r"(dest));
    __asm("LDR PC, [%0, #4]" :: "r"(dest));
}

void dfu_boot_img(int flashid)
{
    struct image_header_enc *img_hdr;
    int coreid = DFU_FLASH_IMG_IDX(flashid);

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
        if (coreid < 2 * CORE_MAX)
        {
            coreid %= CORE_MAX;
            if (coreid == CORE_HCPU || coreid == CORE_BL)
            {
                if (is_addr_in_flash((uint32_t)dest))
                {
                    /* copy encrypted key to ram as AES_ACC cannot access flash */
                    memcpy(dfu_key, img_hdr->key, sizeof(dfu_key));
                    sifli_hw_init_xip_key(dfu_key);
#ifndef SOC_SF32LB55X
                    sec_flash_key_update();
#endif
                    // Setup XIP for decoding and running
                    rt_flash_set_ctr_nonce((uint32_t)dest, (uint32_t)dest, (uint32_t)dest + img_hdr->length,
                                           dfu_get_counter(0));
                    RT_ASSERT(src >= dest);
                    rt_flash_set_alias((uint32_t)dest, (uint32_t)dest, img_hdr->length,
                                       (uint32_t)src - (uint32_t)dest);

                    /* enable on-the-fly decoder */
                    rt_flash_enable_aes((uint32_t)dest, 1);

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
            memcpy(dest, src, img_hdr->length);
        }
        if (coreid < 2 * CORE_MAX)
        {
            coreid %= CORE_MAX;
            if (coreid == CORE_HCPU || coreid == CORE_BL)
            {
                if (is_addr_in_flash((uint32_t)dest))
                {
                    /* update offset of flash XIP */
                    RT_ASSERT(src >= dest);
                    rt_flash_set_alias((uint32_t)dest, (uint32_t)dest, img_hdr->length,
                                       (uint32_t)src - (uint32_t)dest);
                }
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

static void dfu_copy_img(uint8_t *dest, uint8_t *src, uint32_t len)
{
    FLASH_HandleTypeDef *fhandle = rt_nand_get_handle((uint32_t)src);
    if (fhandle)
    {
        rt_nand_read((uint32_t)src, dest, len);
    }
    else
    {
        memcpy((void *)dest, (void *)src, len);
    }

    SCB_CleanDCache();
}



void dfu_boot_img_in_flash(int flashid)
{
    struct image_header_enc *img_hdr;
    int coreid = DFU_FLASH_IMG_IDX(flashid);

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
                    // Setup XIP for decoding and running
                    rt_mpi_set_ctr_nonce((uint32_t)dest, (uint32_t)dest, (uint32_t)dest + img_hdr->length,
                                         dfu_get_counter(0));
                    RT_ASSERT(src >= dest);

                    if (is_flash)
                        rt_mpi_set_alias((uint32_t)dest, (uint32_t)dest, img_hdr->length,
                                         (uint32_t)src - (uint32_t)dest);
                    /* enable on-the-fly decoder */
                    rt_mpi_enable_aes((uint32_t)dest, 1);

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
            dfu_copy_img(dest, src, img_hdr->length);
        }
        if (coreid < 2 * CORE_MAX)
        {
            coreid %= CORE_MAX;
            if (coreid == CORE_HCPU || coreid == CORE_BL || coreid == CORE_LCPU)
            {
                if (is_addr_in_flash((uint32_t)dest))
                {
                    /* update offset of flash XIP */
                    RT_ASSERT(src >= dest);
                    rt_flash_set_alias((uint32_t)dest, (uint32_t)dest, img_hdr->length,
                                       (uint32_t)src - (uint32_t)dest);
                }
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

RT_WEAK void sec_flash_func_init(void)
{
}

RT_WEAK void dfu_flash_init()
{
    sec_flash_func_init();
    sec_flash_init();
    if (dfu_secure_boot_check() == 0)
    {
        g_dfu_efuse_read_hook = dfu_get_efuse_hook;
    }
}
int dfu_verify_hdr_sec(uint8_t flashid, uint8_t *data, int size)
{
    int r = DFU_FAIL;
    struct image_cfg_hdr *hdr = (struct image_cfg_hdr *)data;
    uint8_t *d;

    data += sizeof(struct image_cfg_hdr);
    size -= sizeof(struct image_cfg_hdr);
    d = sifli_dec_verify(NULL, 0, data, dfu_temp, size, hdr->hash);
    if (d)
    {
        r = DFU_SUCCESS;
    }
    return r;

}

int dfu_verify_body_sec(uint8_t flashid, uint8_t *data, int size)
{
    int r = DFU_FAIL;
    struct image_body_hdr *hdr = (struct image_body_hdr *)data;
    uint8_t *key;
    uint8_t *d;
    int flash_img_idx = DFU_FLASH_IMG_IDX(flashid);
    struct image_header_enc *img_hdr;

    if ((flashid >= DFU_FLASH_PARTITION)
            || (flashid < DFU_FLASH_IMG_LCPU))
    {
        return r;
    }

    // Read image key
    //key = &(g_sec_config->imgs[coreid].key[0]);
    key = dfu_key1;
    size -= sizeof(struct image_body_hdr);
    data += sizeof(struct image_body_hdr);
    d = sifli_dec_verify(key, hdr->offset, data, dfu_temp, size, hdr->hash);
    if (d)
    {
        r = DFU_SUCCESS;
    }
    return r;
}


int dfu_receive_resume(uint8_t flashid, uint8_t *data, int size)
{
    int r = DFU_FAIL;

#ifdef BSP_USING_DFU_COMPRESS
    uint8_t compress_img_idx = DFU_FLASH_IMG_COMPRESS_IDX(flashid);

    if (DFU_FLASH_IMG_COMPRESS_FLASH(flashid) == DFU_FLASH_COMPRESS)
    {
        flashid = DFU_FLASH_IMG_COMPRESS_FLASH(flashid);
    }

    if (flashid == DFU_FLASH_COMPRESS)
    {
        struct image_header_compress_resume *hdr = (struct image_header_compress_resume *)data;
        volatile struct dfu_compress_configuration *config = g_dfu_compress_config;
        // All other app should stop in this scenario, heap should be enough
        RT_ASSERT(config);

        if (config->imgs[config->img_count - 1].state != DFU_STATE_BIN_DOWNLOADING ||
                config->imgs[config->img_count - 1].compress_img_id != hdr->img_id ||
                config->imgs[config->img_count - 1].img.enc_img.length != hdr->total_len ||
                config->imgs[config->img_count - 1].current_img_len != hdr->offset ||
                rt_memcmp((const void *)config->imgs[config->img_count - 1].img.enc_img.sig, hdr->sig, DFU_SIG_SIZE) != 0 ||
                rt_strncmp((char *)config->imgs[config->img_count - 1].img.enc_img.ver, (const char *)hdr->ver, DFU_VERSION_LEN) != 0)
            return r;

        // Use flash info
        rt_memcpy(&g_dfu_img_progress, (const void *)&config->imgs, sizeof(sizeof(struct image_header_compress)));
        // Recovery dfu key
        memcpy(dfu_key, (const void *)config->imgs[config->img_count - 1].img.enc_img.key, sizeof(dfu_key));
        sifli_hw_dec_key(dfu_key, dfu_key1, sizeof(dfu_key1));
        r = DFU_SUCCESS;
    }
#endif
    return r;

}
int dfu_receive_pkt(int len, uint8_t *data)
{
    int r;
    struct dfu_hdr *hdr = (struct dfu_hdr *)data;

    int img_idx = DFU_FLASH_IMG_IDX(hdr->id);
    if (img_idx < CORE_BOOT)
    {
        if ((uint32_t)g_sec_config->running_imgs[img_idx] != FLASH_UNINIT_32 &&
                (g_sec_config->running_imgs[img_idx] == &(g_sec_config->imgs[img_idx])))
        {
            // For Ping pong images
            if (hdr->command != DFU_CONFIG_ENC)
                hdr->id += CORE_MAX;
        }
    }
    LOG_I("command %d", hdr->command);
    switch (hdr->command)
    {
    case DFU_IMG_HDR_ENC:
        r = dfu_process_hdr_sec(hdr->id, data + sizeof(struct dfu_hdr), len - sizeof(struct dfu_hdr));
        break;
    case DFU_IMG_BODY_ENC:
        r = dfu_process_body_sec(hdr->id, data + sizeof(struct dfu_hdr), len - sizeof(struct dfu_hdr));
        break;
    case DFU_CONFIG_ENC:
        r = dfu_process_config_sec(hdr->id, data + sizeof(struct dfu_hdr), len - sizeof(struct dfu_hdr));
        break;
    case DFU_END:
        r = dfu_end(hdr->id);
        break;
#ifdef BSP_USING_DFU_COMPRESS
    case DFU_IMG_HDR:
    {
        r = dfu_process_hdr(hdr->id, data + sizeof(struct dfu_hdr), len - sizeof(struct dfu_hdr));
        break;
    }
    case DFU_IMG_BODY:
    {
        struct dfu_hdr_body *hdr1 = (struct dfu_hdr_body *)data;
        r = dfu_process_body(hdr1->id, data + sizeof(struct dfu_hdr_body), hdr1->offset, len - sizeof(struct
                             dfu_hdr_body));
        break;

    }
    case DFU_END_NO_ENC:
    {
        r = dfu_end_wo_enc(hdr->id);
        break;
    }
#endif
    default:
        r = DFU_FAIL;
        break;
    }
    return r;
}

struct image_header_enc *dfu_get_img_info(uint8_t img_idx)
{
    return &(g_sec_config->imgs[img_idx]);
}

#ifdef BSP_USING_DFU_COMPRESS

int dfu_bin_verify(uint8_t *dfu_key)
{

    uint8_t img_header_get = 1;
    uint32_t total_len = g_dfu_img_progress.img.enc_img.length, total_uncompress_len, packet_len;
    uint8_t *compress_buf = NULL, *uncompress_buf = NULL;
    uint32_t offset = 0, com_offset = 0, pksize = 0;
    int r = DFU_FAIL;
    uint16_t blksize = g_dfu_img_progress.img.enc_img.blksize, body_size = 0, comp_len;
    mbedtls_sha256_context ctx2;
    uint8_t *sig = NULL;
    uint8_t *enc_data = rt_malloc(blksize);
    // Parser the 1st img
    while (total_len)
    {
        if (blksize >= total_len)
            blksize = total_len;
        sec_flash_read(DFU_FLASH_COMPRESS, offset, enc_data, blksize);
        sifli_hw_dec(dfu_key, enc_data, dfu_temp, blksize, offset);
        total_len -= blksize;
        offset += blksize;

        // paser the 1st part
        if (img_header_get)
        {
            struct image_header_compress_info *hdr;
            //get the first img
            // len is decided by pksize, default is 10K
            hdr = (struct image_header_compress_info *)dfu_temp;
            pksize = hdr->pksize;
            total_uncompress_len = hdr->total_len;
            LOG_I("uncompre len %d \r\n", total_uncompress_len);
            packet_len = ((dfu_compress_packet_header_t *)(dfu_temp + sizeof(struct image_header_compress_info)))->packet_len;
            sig = rt_malloc(DFU_SIG_SIZE);
            RT_ASSERT(sig);
            rt_memcpy(sig, hdr->sig, DFU_SIG_SIZE);

            compress_buf = rt_malloc(packet_len);
            RT_ASSERT(compress_buf);
            uncompress_buf = rt_malloc(pksize);
            RT_ASSERT(uncompress_buf);
            rt_memcpy(compress_buf, dfu_temp + sizeof(struct image_header_compress_info) + sizeof(dfu_compress_packet_header_t),
                      blksize - sizeof(struct image_header_compress_info) - sizeof(dfu_compress_packet_header_t));
            com_offset += blksize - sizeof(struct image_header_compress_info) - sizeof(dfu_compress_packet_header_t);
            // Get compress length
            // Prepare Hash calcualte
            mbedtls_sha256_init(&ctx2);
            mbedtls_sha256_starts(&ctx2, 0); /* SHA-256, not 224 */
            img_header_get = 0;
            continue;
        }

        if (compress_buf)
        {
            //LOG_I("left size %d, offset %d, blkszie %d, pack %d\r\n", total_len, com_offset, blksize, packet_len);
            if (com_offset + blksize >= packet_len)
            {
                // uncompress
                uint32_t left_size = packet_len - com_offset;
                uint32_t left_len = com_offset + blksize - packet_len;
                rt_memcpy(compress_buf + com_offset, dfu_temp, left_size);
                r = uncompress2(uncompress_buf, (uLong *)&pksize, compress_buf, (uLong *)&packet_len);
                rt_free(compress_buf);
                compress_buf = NULL;
                com_offset = 0;
                if (r != Z_OK)
                    break;
                RT_ASSERT(total_uncompress_len >= pksize);
                total_uncompress_len -= pksize;
                mbedtls_sha256_update(&ctx2, uncompress_buf, pksize);

                LOG_D("Before uncompre len %d, total len %d \r\n", total_uncompress_len, total_len);
                //if (!total_len && left_len)
                //  RT_ASSERT(0);
                // Parse new header

                // The last packet
                if (!total_len && !left_len)
                    break;

                if (!total_uncompress_len)
                    break;
                // Always read more if could
                {
                    uint32_t temp_blksize = blksize;
                    if (blksize >= total_len)
                        blksize = total_len;
                    sec_flash_read(DFU_FLASH_COMPRESS, offset, enc_data, blksize);
                    sifli_hw_dec(dfu_key, enc_data, dfu_temp + temp_blksize, blksize, offset);
                    total_len -= blksize;
                    offset += blksize;
                    left_len += blksize;
                }
                packet_len = ((dfu_compress_packet_header_t *)(dfu_temp + left_size))->packet_len;
                compress_buf = rt_malloc(packet_len);
                RT_ASSERT(compress_buf);
                rt_memcpy(compress_buf, dfu_temp + left_size + sizeof(dfu_compress_packet_header_t), left_len - sizeof(dfu_compress_packet_header_t));
                com_offset += left_len - sizeof(dfu_compress_packet_header_t);
                // To avoid the last packet
                LOG_D("After uncompre len %d, total len %d \r\n", total_uncompress_len, total_len);
                if (!total_len)
                {
                    r = uncompress2(uncompress_buf, (uLong *)&pksize, compress_buf, (uLong *)&packet_len);
                    rt_free(compress_buf);
                    compress_buf =  NULL;
                    com_offset = 0;
                    if (r != Z_OK)
                        break;
                    RT_ASSERT(total_uncompress_len >= pksize);
                    total_uncompress_len -= pksize;
                    mbedtls_sha256_update(&ctx2, uncompress_buf, pksize);
                    break;
                }
            }
            else
            {
                rt_memcpy(compress_buf + com_offset, dfu_temp, blksize);
                com_offset += blksize;
            }
        }
    }
    LOG_I("total len %d, uncom len %d, compress buf %x\r\n", total_len, total_uncompress_len, compress_buf);
    //RT_ASSERT(compress_buf == NULL);
    rt_free(uncompress_buf);
    rt_free(enc_data);

    if (r == DFU_SUCCESS)
    {
        // Verify signature.
        r = DFU_FAIL;
        mbedtls_sha256_finish(&ctx2, dfu_temp);
        {
            // Verify signature
            mbedtls_pk_context pk;
            uint8_t *key;
            mbedtls_pk_init(&pk);
            if (mbedtls_pk_parse_public_key(&pk, g_sec_config->sig_pub_key, DFU_SIG_KEY_SIZE) == 0)
            {
                mbedtls_rsa_set_padding((mbedtls_rsa_context *)pk.pk_ctx, MBEDTLS_RSA_PKCS_V15, MBEDTLS_MD_SHA256);
                /*Calculate the RSA encryption of the data. */
                if (mbedtls_pk_verify(&pk, MBEDTLS_MD_SHA256, dfu_temp, DFU_IMG_HASH_SIZE,
                                      sig, DFU_SIG_SIZE) == 0)
                {
                    r = DFU_SUCCESS;
                }
            }
        }
    }

    rt_free(sig);
    LOG_I("verify r %d\r\n", r);
    return r;
}
#endif


#ifdef FINSH_USING_MSH

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


extern struct finsh_shell *shell;
static int dfu_recv(int argc, char **argv)
{
    int r = DFU_FAIL;

    if (argc == 2)
    {
        int len = atoi(argv[1]);
        if (len < DFU_MAX_BLK_SIZE)
        {
            int offset = 0;
            while (1)
            {
#ifdef RT_USING_POSIX
                dfu_data[offset] = getchar();
                offset++;
#else
                int delta = rt_device_read(shell->device, offset, &(dfu_data[offset]), len - offset);
                offset += delta;
#endif
                if (offset < len)
                    rt_sem_take(&shell->rx_sem, RT_WAITING_FOREVER);
                else
                    break;
            }
            r = dfu_receive_pkt(len, dfu_data);
        }
    }
    if (r == DFU_SUCCESS)
        LOG_I("OK\n");
    else
        LOG_E("Fail\n");
    return r;
}
MSH_CMD_EXPORT(dfu_recv, Receiv DFU data.);

#if 0
int lcpu(int argc, char *argv[])
{
    if (argc > 1)
    {
        LOG_I("LCPU before: RSTR2=0x%x\n", hwp_lpsys_rcc->RSTR);

        if (strcmp(argv[1], "start") == 0)
        {
            uint32_t before = hwp_lpsys_rcc->RSTR;
            hwp_lpsys_rcc->RSTR &= ~LPSYS_RCC_RSTR_LCPU;
        }
        if (strcmp(argv[1], "stop") == 0)
        {
            uint32_t before = hwp_lpsys_rcc->RSTR;
            uint32_t *p_dest = (uint32_t *)LCPU_BOOT_ADDR;
            hwp_lpsys_rcc->RSTR |= LPSYS_RCC_RSTR_LCPU;
            *p_dest = 0;

        }
        LOG_I("LCPU After: RSTR2=0x%x\n", hwp_lpsys_rcc->RSTR);
    }
    return RT_EOK;
}
MSH_CMD_EXPORT(lcpu,   Turn on / off CPU.);
#endif

static int reset(int argc, char **argv)
{

    uint32_t magic = SEC_CONFIG_MAGIC;
    sec_flash_erase(DFU_FLASH_SEC_CONFIG, 0, sizeof(struct sec_configuration));
    sec_flash_write(DFU_FLASH_SEC_CONFIG, 0, (uint8_t *)&magic, 4);
    dfu_flash_init();
    return 0;
}
MSH_CMD_EXPORT(reset, Reset secure configuration.);

#ifndef HW_EFUSE
static int reset_efuse(int argc, char **argv)
{
    sec_flash_erase(DFU_FLASH_SEC_CONFIG, EFUSE_OFFSET, sizeof(struct sec_efuse));
    return 0;
}
MSH_CMD_EXPORT(reset_efuse, Reset flash simulated efuse.);
#endif

#endif


/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
