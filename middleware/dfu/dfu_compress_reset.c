/**
  ******************************************************************************
  * @file   dfu_reset.c
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


#if 1
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "board.h"
#include "dfu.h"
#include "drv_flash.h"
#include "zlib.h"

#ifdef BSP_USING_DFU_COMPRESS

static uint8_t dfu_temp[DFU_MAX_BLK_SIZE];


ALIGN(4)
static uint8_t dfu_key[DFU_KEY_SIZE];
ALIGN(4)
static uint8_t dfu_key1[DFU_KEY_SIZE];

#define LOG_TAG "dfu_reset"
#include "log.h"



typedef struct
{
    rt_device_t flash_dev;
    uint32_t total_uncompress_len;
    uint32_t total_count;
    uint32_t current_uncompress_len;
    uint32_t total_compress_len;
    uint32_t current_read_len;
    uint32_t current_count;
    uint8_t *dfu_bin_addr;
    uint8_t *user_bin_addr;
    uint16_t pk_size;
} dfu_reset_env_t;

static dfu_reset_env_t g_dfu_reset_env;

static dfu_reset_env_t *dfu_reset_get_env(void)
{
    return &g_dfu_reset_env;
}

int dfu_encrypt_packet(int flashid, uint32_t offset, uint8_t *data, uint32_t size, uint8_t *dfu_key)
{
    sifli_hw_enc_with_key(dfu_key, data, data, size, offset);
    sec_flash_write(flashid, offset, data, size);
    return DFU_SUCCESS;
}



static int dfu_decompress(uint8_t flashid, uint8_t *dfu_key, uint16_t img_flags, uint8_t *uncompress_buf, uint32_t *pksize, uint8_t *compress_buf, uint32_t *packet_len,
                          uint32_t *total_uncompress_len, uint32_t *uncompress_offset)
{
    int r;
    r = uncompress2(uncompress_buf, (uLong *)pksize, compress_buf, (uLong *)packet_len);
    if (r != Z_OK)
        return r;
    if (img_flags & DFU_FLAG_ENC)
        dfu_encrypt_packet(flashid, *uncompress_offset, uncompress_buf, *pksize, dfu_key);
    else
    {
        if (dfu_key)
            sec_flash_write(flashid, *uncompress_offset, uncompress_buf, *pksize);
        else
            sec_flash_write_target(flashid, *uncompress_offset, uncompress_buf, *pksize);
    }
    LOG_D("pk len %d", *packet_len);
    RT_ASSERT(*total_uncompress_len >= *pksize);
    *total_uncompress_len -= *pksize;
    *uncompress_offset += *pksize;
    return r;

}


int dfu_reset_install(uint8_t *dfu_key, uint8_t flashid, uint16_t img_flags, uint32_t img_index)
{

    //uint8_t *src_addr = (uint8_t *)g_dfu_compress_config->ctab.base;
    struct image_header_compress *img_progress = (struct image_header_compress *)&g_dfu_compress_config->imgs[img_index];
    uint8_t img_header_get = 1;
    uint32_t total_len, total_uncompress_len, packet_len, uncompress_offset = 0;
    uint8_t *compress_buf = NULL, *uncompress_buf = NULL;
    uint32_t offset = 0, com_offset = 0, pksize = 0;
    int r = DFU_SUCCESS;
    uint32_t blksize, body_size = 0, comp_len, blk_offset = 0;

    if (dfu_key)
    {
        total_len = img_progress->img.enc_img.length;
        blksize = img_progress->img.enc_img.blksize;
    }
    else
    {
        total_len = img_progress->img.none_enc_img.length;
        blksize = img_progress->img.none_enc_img.blksize;
    }
    uint8_t *enc_data = rt_malloc(blksize);
    // Parser the 1st img
    while (total_len)
    {
        if (blksize >= total_len)
            blksize = total_len;

        if (dfu_key)
        {
            sec_flash_read(DFU_FLASH_COMPRESS, offset, enc_data, blksize);
            sifli_hw_dec(dfu_key, enc_data, dfu_temp + blk_offset, blksize, offset);
        }
        else
            sec_flash_read(flashid, offset, dfu_temp + blk_offset, blksize);
        total_len -= blksize;
        offset += blksize;

        // paser the 1st part
        if (img_header_get)
        {
            uint32_t temp_left, temp_offset;
            img_header_get = 0;
            struct image_header_compress_info *hdr;
            //get the first img
            // len is decided by pksize, default is 10K
            hdr = (struct image_header_compress_info *)dfu_temp;
            pksize = hdr->pksize;
            total_uncompress_len = hdr->total_len;
            LOG_I("uncompre len %d \r\n", total_uncompress_len);

            if (dfu_key)
                sec_flash_erase(flashid, 0, total_uncompress_len);
            else
                sec_flash_erase_target(flashid, 0, total_uncompress_len);

            packet_len = ((dfu_compress_packet_header_t *)(dfu_temp + sizeof(struct image_header_compress_info)))->packet_len;

            uncompress_buf = rt_malloc(pksize);
            RT_ASSERT(uncompress_buf);

            temp_offset = sizeof(struct image_header_compress_info) + sizeof(dfu_compress_packet_header_t);
            temp_left = blksize - temp_offset;
            // Handle compress pksize smaller than bksize
            while (packet_len < temp_left)
            {
                r = dfu_decompress(flashid, dfu_key, img_flags, uncompress_buf, &pksize, dfu_temp + temp_offset,
                                   &packet_len, &total_uncompress_len, &uncompress_offset);
                if (r != Z_OK)
                    break;
                temp_left -= packet_len;
                temp_offset += packet_len;
                if (temp_left < sizeof(dfu_compress_packet_header_t))
                {
                    memcpy(dfu_temp, dfu_temp + temp_offset, temp_left);
                    blk_offset = temp_left;
                    break;
                }
                packet_len = ((dfu_compress_packet_header_t *)(dfu_temp + temp_offset))->packet_len;
                temp_offset += sizeof(dfu_compress_packet_header_t);
                temp_left -= sizeof(dfu_compress_packet_header_t);

            }

            if (r != Z_OK)
                break;

            // blk_offset has value means packet_len could not get
            if (blk_offset == 0)
            {
                // already make sure packet len is larger than left data in dfu_temp
                compress_buf = rt_malloc(packet_len);
                RT_ASSERT(compress_buf);

                rt_memcpy(compress_buf, dfu_temp + temp_offset, temp_left);
                com_offset += temp_left;
            }

            continue;
            // Get compress length
            // Prepare Hash calcualte
        }

        if (compress_buf)
        {
            //LOG_I("left size %d, offset %d, blkszie %d, pack %d\r\n", total_len, com_offset, blksize, packet_len);
            if (com_offset + blksize >= packet_len)
            {
                // uncompress
                RT_ASSERT(packet_len >= com_offset);

                uint32_t left_size = packet_len - com_offset;
                uint32_t left_len = com_offset + blksize - packet_len;

                // handle the combine buffer : compress_buf
                rt_memcpy(compress_buf + com_offset, dfu_temp, left_size);
                r = dfu_decompress(flashid, dfu_key, img_flags, uncompress_buf, &pksize, compress_buf, &packet_len,
                                   &total_uncompress_len, &uncompress_offset);
                rt_free(compress_buf);
                compress_buf = NULL;
                com_offset = 0;
                if (r != Z_OK)
                    break;

                //LOG_I("Before uncompre len %d, total len %d \r\n", total_uncompress_len, total_len);
                //if (!total_len && left_len)
                //  RT_ASSERT(0);
                // Parse new header

                // The last packet
                if (!total_len && !left_len)
                    break;

                if (!total_uncompress_len)
                    break;
                // Always read more if could


                if (left_len < sizeof(dfu_compress_packet_header_t))
                {
                    memcpy(dfu_temp, dfu_temp + left_size, left_len);
                    blk_offset = left_len;
                    continue;
                }
                packet_len = ((dfu_compress_packet_header_t *)(dfu_temp + left_size))->packet_len;
                // reuse for used of dfu_temp
                left_size += sizeof(dfu_compress_packet_header_t);
                left_len -= sizeof(dfu_compress_packet_header_t);

                while (packet_len < left_len)
                {

                    r = dfu_decompress(flashid, dfu_key, img_flags, uncompress_buf, &pksize, dfu_temp + left_size,
                                       &packet_len, &total_uncompress_len, &uncompress_offset);
                    left_len -= packet_len;
                    left_size += packet_len;

                    if (r != Z_OK)
                        goto END;

                    // The last packet
                    if (!total_len && !left_len)
                        goto END;

                    if (!total_uncompress_len)
                        goto END;

                    if (left_len < sizeof(dfu_compress_packet_header_t))
                    {
                        memcpy(dfu_temp, dfu_temp + left_size, left_len);
                        blk_offset = left_len;
                        break;
                    }
                    packet_len = ((dfu_compress_packet_header_t *)(dfu_temp + left_size))->packet_len;
                    left_size += sizeof(dfu_compress_packet_header_t);
                    left_len -= sizeof(dfu_compress_packet_header_t);


                }

                if (blk_offset == 0)
                {
                    compress_buf = rt_malloc(packet_len);
                    RT_ASSERT(compress_buf);
                    rt_memcpy(compress_buf, dfu_temp + left_size, left_len);
                    com_offset += left_len;
                }
                // To avoid the last packet
                //LOG_I("After uncompre len %d, total len %d \r\n", total_uncompress_len, total_len);
            }
            else
            {
                rt_memcpy(compress_buf + com_offset, dfu_temp, blksize);
                com_offset += blksize;
            }
        }
        else
        {
            // Allocate failed will assert, this case is for not get packet len
            uint32_t temp_left, temp_offset;
            //RT_ASSERT(blk_offset != 0);
            packet_len = ((dfu_compress_packet_header_t *)(dfu_temp))->packet_len;


            temp_offset = sizeof(dfu_compress_packet_header_t);
            temp_left = blksize + blk_offset - temp_offset;
            blk_offset = 0;
            // Handle compress pksize smaller than bksize
            while (packet_len < temp_left)
            {
                r = dfu_decompress(flashid, dfu_key, img_flags, uncompress_buf, &pksize, dfu_temp + temp_offset,
                                   &packet_len, &total_uncompress_len, &uncompress_offset);
                temp_left -= packet_len;
                temp_offset += packet_len;

                if (r != Z_OK)
                    goto END;

                // The last packet
                if (!total_len && !temp_left)
                    goto END;

                if (!total_uncompress_len)
                    goto END;

                if (temp_left < sizeof(dfu_compress_packet_header_t))
                {
                    // move to the dfu beginning
                    memcpy(dfu_temp, dfu_temp + temp_offset, temp_left);
                    blk_offset = temp_left;
                    break;
                }
                packet_len = ((dfu_compress_packet_header_t *)(dfu_temp + temp_offset))->packet_len;
                temp_offset += sizeof(dfu_compress_packet_header_t);
                temp_left -= sizeof(dfu_compress_packet_header_t);

            }

            // blk_offset has value means packet_len could not get
            if (blk_offset == 0)
            {
                // already make sure packet len is larger than left data in dfu_temp
                compress_buf = rt_malloc(packet_len);
                RT_ASSERT(compress_buf);

                rt_memcpy(compress_buf, dfu_temp + temp_offset, temp_left);
                com_offset += temp_left;
            }
        }
    }

END:
    LOG_I("total len %d, uncom len %d, compress buf %x\r\n", total_len, total_uncompress_len, compress_buf);
    //RT_ASSERT(compress_buf == NULL);
    rt_free(uncompress_buf);
    rt_free(enc_data);

    if (r == DFU_SUCCESS)
    {
        struct dfu_compress_configuration *config = rt_malloc(sizeof(struct dfu_compress_configuration));
        // All other app should stop in this scenario, heap should be enough
        RT_ASSERT(config);
        rt_memcpy(config, (const void *)g_dfu_compress_config, sizeof(struct dfu_compress_configuration));
        config->img_count--;
        // the img_index should be decrese from highest index
        rt_memset(&config->imgs[img_index], 0, sizeof(struct image_header_compress));
        rt_memset(&config->ctab[img_index], 0, sizeof(struct dfu_compress_flash_table));
        sec_flash_erase(DFU_FLASH_COMPRESS_CONFIG, 0, SPI_NOR_SECT_SIZE);
        sec_flash_write(DFU_FLASH_COMPRESS_CONFIG, 0, (uint8_t *)config, sizeof(struct dfu_compress_configuration));
        rt_free(config);
    }

    LOG_I("Instasll r %d\r\n", r);
    return r;
}

#define BOOT_LOCATION 1

extern void run_img(uint8_t *dest);

static void dfu_bootjump(void)
{

    uint32_t i;
    for (i = 0; i < 8; i++)
        NVIC->ICER[0] = 0xFFFFFFFF;
    for (i = 0; i < 8; i++)
        NVIC->ICPR[0] = 0xFFFFFFFF;
    SysTick->CTRL = 0;
    SCB->ICSR |= SCB_ICSR_PENDNMICLR_Msk;
    SCB->SHCSR &= ~(SCB_SHCSR_USGFAULTACT_Msk | SCB_SHCSR_BUSFAULTACT_Msk | SCB_SHCSR_MEMFAULTACT_Msk);

    if (CONTROL_SPSEL_Msk & __get_CONTROL())
    {
        __set_MSP(__get_PSP());
        __set_CONTROL(__get_CONTROL() & ~CONTROL_SPSEL_Msk);
    }

    SCB->VTOR = (uint32_t)HCPU_FLASH_CODE_START_ADDR;
    run_img((uint8_t *)HCPU_FLASH_CODE_START_ADDR);

}

int32_t dfu_reset_start(void)
{
    // 1. Get the total len
#if BOOT_LOCATION<1
    g_sec_config = (struct sec_configuration *)FLASH_START_RAM;
    memset(g_sec_config, FLASH_UNINIT_8, sizeof(struct sec_configuration));
    g_sec_config->magic = SEC_CONFIG_MAGIC;
#else
    g_sec_config = (struct sec_configuration *)FLASH_START;
    dfu_flash_init();
#endif
#ifdef BSP_USING_DFU_COMPRESS
    g_dfu_compress_config = (struct dfu_compress_configuration *)FLASH_DFU_COMPRESS_START;
#endif
    // 2. While loop until all data copy completed.

    uint32_t i = g_dfu_compress_config->img_count;
    while (i--)
    {
        if (g_dfu_compress_config->imgs[i].state == DFU_STATE_BIN_DOWNLOADED ||
                g_dfu_compress_config->imgs[i].state == DFU_STATE_BIN_INSTALLING)
        {
            int r;
            uint8_t flashid = g_dfu_compress_config->imgs[i].compress_img_id;
            if (flashid == DFU_FLASH_IMG_HCPU)
            {
                flashid = DFU_FLASH_COMPRESS_IMG_HCPU;
                uint16_t img_flags = g_sec_config->imgs[DFU_FLASH_IMG_IDX(g_dfu_compress_config->imgs[i].compress_img_id)].flags;
                memcpy(dfu_key, (const void *)g_dfu_compress_config->imgs[i].img.enc_img.key, sizeof(dfu_key));
                sifli_hw_dec_key(dfu_key, dfu_key1, sizeof(dfu_key1));
                r = dfu_reset_install(dfu_key1, flashid, img_flags, i);
            }
            else
                r = dfu_reset_install(NULL, flashid, 0, i);
            RT_ASSERT(r == DFU_SUCCESS);
        }
        //else if (g_dfu_compress_config->imgs[i].state == DFU_STATE_BIN_INSTALLED)
        else
        {
            struct dfu_compress_configuration *config = rt_malloc(sizeof(struct dfu_compress_configuration));
            // All other app should stop in this scenario, heap should be enough
            RT_ASSERT(config);
            rt_memcpy(config, (const void *)g_dfu_compress_config, sizeof(struct dfu_compress_configuration));
            config->img_count--;
            // the img_index should be decrese from highest index
            rt_memset(&config->imgs[i], 0, sizeof(struct image_header_compress));
            sec_flash_erase(DFU_FLASH_COMPRESS_CONFIG, 0, SPI_NOR_SECT_SIZE);
            sec_flash_write(DFU_FLASH_COMPRESS_CONFIG, 0, (uint8_t *)config, sizeof(struct dfu_compress_configuration));
            rt_free(config);
        }
    }

    // 3. Jump to user section

    /* update offset of flash XIP */
    //hwp_flashc1->AASAR = (uint32_t)0x100f0000;
    /* ensure length is aligned to 1k boundary */
    //hwp_flashc1->AAEAR = (uint32_t)0x100f0000 + 0x80000 + (1 << FLASHC_AAEAR_EA_Pos) - 1;
    //sifli_hw_set_offset((uint8_t *)0x10020000, (uint8_t *)0x100f0000);

    if (CONTROL_nPRIV_Msk & __get_CONTROL())
    {
        __asm("SVC #0");
    }
    else
    {
        dfu_bootjump();
    }

    return 0;
}





void SVC_Handler_Main(unsigned int *svc_args)
{
    unsigned int svc_number;
    svc_number = ((char *)svc_args[6])[-2];
    switch (svc_number)
    {
    case 0:
    {
        __set_CONTROL(__get_CONTROL() & ~CONTROL_nPRIV_Msk);
        dfu_bootjump();
    }
    break;
    default:
        break;
    }

}

void SVC_Handler(void)
{
    __asm(
        ".global SVC_Handler_Main\n"
        "TST lr, #4\n"
        "ITE EQ\n"
        "MRSEQ r0, MSP\n"
        "MRSNE r0, PSP\n"
        "B SVC_Handler_Main\n"
    );

}

#endif


#endif

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

