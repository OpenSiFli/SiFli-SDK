/**
  ******************************************************************************
  * @file   dfu_flash.c
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


#include <rtthread.h>
#include <rtdevice.h>
#ifdef BSP_USING_DFU_COMPRESS

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <rtthread.h>
#include "register.h"
#include "dfu.h"
#include "drv_flash.h"
#include "mbedtls/cipher.h"
#include "mbedtls/pk.h"
#include "mbedtls/sha256.h"

#if 0
static uint8_t dfu_temp[DFU_MAX_BLK_SIZE];


struct image_header_compress g_dfu_img_progress;


volatile struct dfu_compress_configuration *g_dfu_compress_config;

static int dfu_process_hdr_compress(uint8_t *data, int size)
{
    struct image_header_compress_info *hdr = (struct image_header_compress_info *)data;
    struct dfu_compress_configuration *config = rt_malloc(sizeof(struct dfu_compress_configuration));
    // All other app should stop in this scenario, heap should be enough
    RT_ASSERT(config);

    rt_memcpy(config, g_dfu_compress_config, sizeof(sizeof(struct dfu_compress_configuration)));
    // Always enter init start state once received DFU_IMG_HDR_COMPRESS command
    // Only imgs info could be modified.
    // It's for img len not including header
    g_dfu_img_progress.current_dl_len = 0;
    g_dfu_img_progress.pksize = hdr->pksize;
    g_dfu_img_progress.state = DFU_STATE_BIN_DOWNLOADING;
    g_dfu_img_progress.total_bin_len = hdr->total_len;
    g_dfu_img_progress.img_id = hdr->img_id;
    rt_memcpy(g_dfu_img_progress.ver, hdr->ver, DFU_VERSION_LEN);
    rt_memcpy(g_dfu_img_progress.sig, hdr->sig, DFU_SIG_SIZE);

    rt_memcpy(&config->imgs, &g_dfu_img_progress, sizeof(struct image_header_compress));

    // Save to flash and erase the image flash section.
    dfu_compress_flash_erase(DFU_FLASH_COMPRESS_CONFIG, 0, sizeof(struct dfu_compress_configuration));
    dfu_compress_flash_write(DFU_FLASH_COMPRESS_CONFIG, 0, &config, sizeof(struct dfu_compress_configuration));

    rt_free(config);

    return DFU_SUCCESS;

}

static int dfu_receive_compress_pkt_resume(uint8_t *data, int size)
{
    int r = DFU_FAIL;
    struct image_header_compress_resume *hdr = (struct image_header_compress_resume *)data;
    struct dfu_compress_configuration *config = g_dfu_compress_config;
    // All other app should stop in this scenario, heap should be enough
    RT_ASSERT(config);

    if (config->imgs.state != DFU_STATE_BIN_DOWNLOADING ||
            config->imgs.total_bin_len != hdr->total_len ||
            config->imgs.total_dl_len != hdr->total_dl_len ||
            config->imgs.current_dl_len != hdr->offset ||
            config->imgs.pksize != hdr->pksize ||
            rt_memcmp(config->imgs.sig, hdr->sig, DFU_SIG_SIZE) != 0 ||
            rt_memcmp(config->imgs.ver, hdr->ver, DFU_VERSION_LEN) != 0)
        return r;

    // Use flash info
    rt_memcpy(&g_dfu_img_progress, &config->imgs, sizeof(sizeof(struct image_header_compress)));

    return DFU_SUCCESS;
}


static int dfu_process_body_compress(uint8_t *data, int size)
{
    int r = DFU_FAIL;

    if (g_dfu_img_progress.state != DFU_STATE_BIN_DOWNLOADING)
        return r;

    if (g_dfu_img_progress.current_dl_len + size > g_dfu_img_progress.total_dl_len)
        return r;

    g_dfu_img_progress.current_dl_len += size;
    dfu_compress_flash_write(DFU_FLASH_COMPRESS, g_dfu_img_progress.current_dl_len, data, size);

    if ((g_dfu_img_progress.current_dl_len % FLASH_SECT_SIZE) == 0)
    {
        struct dfu_compress_configuration *config = rt_malloc(sizeof(struct dfu_compress_configuration));
        // All other app should stop in this scenario, heap should be enough
        RT_ASSERT(config);
        rt_memcpy(config, g_dfu_compress_config, sizeof(sizeof(struct dfu_compress_configuration)));
        rt_memcpy(&config->imgs, &g_dfu_img_progress, sizeof(struct image_header_compress));
        dfu_compress_flash_update(DFU_FLASH_COMPRESS_CONFIG, 0, &config, sizeof(struct dfu_compress_configuration));
        rt_free(config);
    }

    return DFU_SUCCESS;
}



static int dfu_process_download_end(void)
{
    int offset = 0;
    int size = 0;
    int r = DFU_FAIL;
    int img_idx;
    struct image_header_compress *img_hdr;
    uint8_t *sig_pub_key;

    img_hdr = &g_dfu_img_progress;
    sig_pub_key = &g_dfu_compress_config->sig_pub_key[0];

    {
        // Calculate HASH
        mbedtls_sha256_context ctx2;
        mbedtls_sha256_init(&ctx2);
        mbedtls_sha256_starts(&ctx2, 0); /* SHA-256, not 224 */
        do
        {
            if (offset + sizeof(dfu_temp) > img_hdr->total_dl_len)
                size = img_hdr->total_dl_len - offset;
            else
                size = sizeof(dfu_temp);
            if (size)
            {
                dfu_compress_flash_read(DFU_FLASH_COMPRESS, offset, dfu_temp, size);
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
                // Verify uncompress bin
                r = dfu_bin_verify();

                // Hand over to OTA reset handler
            }
        }
    }
    if (r == DFU_SUCCESS)
    {
        g_dfu_img_progress.state = DFU_STATE_BIN_DOWNLOADED;
    }
    else
    {
        // Verify failed, treat no img downloaded
        rt_memset(&g_dfu_img_progress, 0, sizeof(struct image_header_compress));
    }

    struct dfu_compress_configuration *config = rt_malloc(sizeof(struct dfu_compress_configuration));
    // All other app should stop in this scenario, heap should be enough
    RT_ASSERT(config);
    rt_memcpy(config, g_dfu_compress_config, sizeof(sizeof(struct dfu_compress_configuration)));
    rt_memcpy(&config->imgs, &g_dfu_img_progress, sizeof(struct image_header_compress));
    dfu_compress_flash_update(DFU_FLASH_COMPRESS_CONFIG, 0, &config, sizeof(struct dfu_compress_configuration));
    rt_free(config);
    return r;
}

//#endif

int dfu_bin_verify(uint8_t *dfu_key)
{

    uint8_t *src_addr = g_dfu_compress_config->ctab.base;
    uint8_t img_header_get = 1;
    uint32_t total_len = g_dfu_img_progress.img.length, total_uncompress_len, packet_len;
    uint8_t *compress_buf = NULL, *uncompress_buf = NULL;
    uint32_t offset = 0, com_offset = 0;
    int r = DFU_FAIL;
    uint16_t blksize = g_dfu_img_progress.img.blksize, body_size = 0, comp_len, pksize = 0;
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
            packet_len = (dfu_compress_packet_header_t *)(dfu_temp + sizeof(struct image_header_compress_info))->packet_len;
            sig = rt_malloc(DFU_SIG_SIZE);
            rt_memcpy(sig, hdr->sig, DFU_SIG_SIZE);

            compress_buf = rt_malloc(packet_len);
            uncompress_buf = rt_malloc(pksize);
            rt_memcpy(compress_buf, dfu_temp + sizeof(struct image_header_compress_info) + sizeof(dfu_compress_packet_header_t),
                      blksize - sizeof(struct image_header_compress_info) - sizeof(dfu_compress_packet_header_t));
            com_offset += blksize - sizeof(struct image_header_compress_info) - sizeof(dfu_compress_packet_header_t);
            total_len -= blksize;
            // Get compress length
            // Prepare Hash calcualte
            mbedtls_sha256_init(&ctx2);
            mbedtls_sha256_starts(&ctx2, 0); /* SHA-256, not 224 */
            img_header_get = 0;
            continue;
        }

        if (compress_buf)
        {
            if (com_offset + blksize >= packet_len)
            {
                // uncompress
                uint32_t left_size = packet_len - com_offset;
                uint32_t left_len = com_offset + blksize - packet_len;
                rt_memcpy(compress_buf + com_offset, dfu_temp, left_size);
                r = uncompress2(uncompress_buf, &pksize, compress_buf, &packet_len);
                if (r != Z_OK)
                    break;
                mbedtls_sha256_update(&ctx2, uncompress_buf, pksize);
                rt_free(compress_buf);
                if (!total_len && left_len)
                    RT_ASSERT(0);
                // Parse new header

                // The last packet
                if (!total_len && !left_len)
                    break;

                // Always read more if could
                {
                    if (blksize >= total_len)
                        blksize = total_len;
                    sec_flash_read(DFU_FLASH_COMPRESS, offset, enc_data, blksize);
                    sifli_hw_dec(dfu_key, enc_data, dfu_temp + blksize, blksize, offset);
                    total_len -= blksize;
                    offset += blksize;
                    left_len += blksize;
                }
                packet_len = (dfu_compress_packet_header_t *)(dfu_temp + left_size)->packet_len;
                compress_buf = rt_malloc(packet_len);
                rt_memcpy(compress_buf, dfu_temp + left_size + sizeof(dfu_compress_packet_header_t), left_len - sizeof(dfu_compress_packet_header_t));
                com_offset += left_len - sizeof(dfu_compress_packet_header_t);
                // To avoid the last packet
                if (!total_len)
                {
                    r = uncompress2(uncompress_buf, &pksize, compress_buf, &packet_len);
                    if (r != Z_OK)
                        break;
                    mbedtls_sha256_update(&ctx2, uncompress_buf, pksize);
                    rt_free(compress_buf);
                    break;
                }
            }
            else
                rt_memcpy(compress_buf + com_offset, dfu_temp, blksize);
        }
    }

    rt_free(uncompress_buf);

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
    return r;
}

//#if 0
int dfu_receive_compress_pkt(int len, uint8_t *data)
{
    int r;
    struct dfu_compress_hdr *hdr = (struct dfu_compress_hdr *)data;

    // the status should update to according img.
    switch (hdr->command)
    {
    case DFU_IMG_HDR_COMPRESS:
    {
        r = dfu_process_hdr_compress(data + sizeof(struct dfu_compress_hdr), len - sizeof(struct dfu_compress_hdr));
        break;
    }
    case DFU_IMG_BODY_COMPRESS:
    {
        r = dfu_process_body_compress(data + sizeof(struct dfu_compress_hdr), len - sizeof(struct dfu_compress_hdr));
        break;
    }
    case DFU_IMG_END:
    {
        r = dfu_process_download_end();
        break;
    }
    case DFU_DOWNLOAD_END:
    {
        // All img transfer completed, could install the bins
    }

    }
    return r;

}
#endif

#endif // BSP_USING_DFU_COMPRESS

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

