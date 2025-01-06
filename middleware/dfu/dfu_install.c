/**
  ******************************************************************************
  * @file   dfu_install.c
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

#include "rtconfig.h"

#ifdef OTA_55X

#include "dfu.h"
#include "dfu_internal.h"
//#ifdef DFU_OTA_MANAGER
#ifdef DFU_DECOMPRESS_USING_SOFTWARE
    #include "zlib.h"
#endif
#include "log.h"

#include "bf0_hal_ezip.h"
#ifdef BSP_USING_EPIC
    #include "drv_epic.h"
#endif /* BSP_USING_EPIC */

#ifdef DFU_DECOMPRESS_USING_SOFTWARE
    #define DFU_DECOM_OK Z_OK
#else
    #define DFU_DECOM_OK HAL_OK
#endif

#ifdef SOC_SF32LB55X
    #define CB_IS_IN_ITCM_RANGE(addr)    ((((addr) >= HPSYS_ITCM_BASE) && ((addr) < HPSYS_ITCM_END)) ? true : false)

    #define CB_IS_IN_RETM_RANGE(addr)    ((((addr) >= HPSYS_RETM_BASE) && ((addr) < HPSYS_RETM_END)) ? true : false)

    #define CB_IS_IN_EZIP_ADDR_RANGE(addr)  (!CB_IS_IN_ITCM_RANGE((addr)) && !CB_IS_IN_RETM_RANGE((addr)))
#else
    #define CB_IS_IN_EZIP_ADDR_RANGE(addr)  (true)
#endif

static int dfu_decompress(dfu_image_header_int_t *header, uint8_t *dfu_key, uint8_t *uncompress_buf, uint32_t *pksize, uint8_t *compress_buf, uint32_t *packet_len,
                          uint32_t *total_uncompress_len, uint32_t *uncompress_offset)
{
    int r;
#ifdef DFU_DECOMPRESS_USING_SOFTWARE
    r = uncompress2(uncompress_buf, (uLong *)pksize, compress_buf, (uLong *)packet_len);
#else
    memcpy(pksize, compress_buf, sizeof(uint32_t));

    // compress_buf may not 4 byte aligned, so malloc new one.
    uint8_t *temp_compress_buf;
    uint32_t compress_len = *packet_len;
    temp_compress_buf = malloc(compress_len);
    OS_ASSERT(temp_compress_buf);
    memcpy(temp_compress_buf, compress_buf + sizeof(uint32_t), compress_len);
#ifdef BSP_USING_EPIC
    drv_epic_take(RT_WAITING_FOREVER);
#endif
    if (CB_IS_IN_EZIP_ADDR_RANGE((uint32_t)uncompress_buf) && CB_IS_IN_EZIP_ADDR_RANGE((uint32_t)temp_compress_buf))
    {
        EZIP_DecodeConfigTypeDef config;
        EZIP_HandleTypeDef ezip_handle = {0};

        config.input = temp_compress_buf;
        config.output = uncompress_buf;
        config.start_x = 0;
        config.start_y = 0;
        config.width = 0;
        config.height = 0;
        config.work_mode = HAL_EZIP_MODE_GZIP;
        config.output_mode = HAL_EZIP_OUTPUT_AHB;
        ezip_handle.Instance = hwp_ezip;
        HAL_EZIP_Init(&ezip_handle);

        // disbale interrupt
        register rt_base_t ret;
        ret = rt_hw_interrupt_disable();
        r = HAL_EZIP_Decode(&ezip_handle, &config);
        rt_hw_interrupt_enable(ret);
        RT_ASSERT(HAL_OK == r);
    }
    else
    {
        OS_ASSERT(0);
    }
#ifdef BSP_USING_EPIC
    drv_epic_release();
#endif
    free(temp_compress_buf);
#endif

    if (r != DFU_DECOM_OK)
        return r;
    if (header->flag & DFU_FLAG_ENC)
        dfu_encrypt_packet(header, *uncompress_offset, uncompress_buf, *pksize, dfu_key);
    else
    {
        dfu_packet_write_flash(header, *uncompress_offset, uncompress_buf, *pksize);
    }
    //LOG_D("pk len %d", *packet_len);
    RT_ASSERT(*total_uncompress_len >= *pksize);
    *total_uncompress_len -= *pksize;
    *uncompress_offset += *pksize;
    return r;

}

uint32_t g_uncompress_len;

static int dfu_img_install_flash(dfu_ctrl_env_t *env, uint8_t *dfu_key, dfu_image_header_int_t *header)
{
    dfu_img_info_t *curr_info = &env->prog.fw_context.code_img.curr_img_info;
    if (env->mode == DFU_CTRL_NORMAL_MODE)
    {
        curr_info = &env->ota_state.fw_context.code_img.curr_img_info;
    }
    uint8_t img_header_get = 1;
    uint32_t total_len, total_uncompress_len, packet_len, total_hdr_len = 0, uncompress_offset = 0;
    uint8_t *compress_buf = NULL, *uncompress_buf = NULL;
    uint8_t *dfu_temp;
    dfu_temp = malloc(DFU_MAX_BLK_SIZE);
    OS_ASSERT(dfu_temp);
    uint32_t offset = 0, com_offset = 0, pksize = 0;
    int r = DFU_SUCCESS;
    uint32_t blksize, body_size = 0, comp_len, blk_offset = 0;
    uint8_t is_last_small_packet = 0;

    curr_info->img_id = header->img_id;
    curr_info->img_state = DFU_CTRL_IMG_STATE_DOWNLOADING;
    curr_info->header = header;

    header->flag &= ~DFU_FLAG_COMPRESS;
    total_len = header->length;
    blksize = env->prog.fw_context.code_img.blk_size;
    if (env->mode == DFU_CTRL_NORMAL_MODE)
    {
        blksize = env->ota_state.fw_context.code_img.blk_size;
    }

    uint8_t *enc_data = malloc(blksize);
    // Parser the 1st img
    while (total_len)
    {
        if (blksize >= total_len)
            blksize = total_len;

        /* Should read from compress section. */
        header->flag |= DFU_FLAG_COMPRESS;
        //header->flag |= DFU_FLAG_ENC;
        if (dfu_key)
        {
            dfu_read_storage_data(header, offset, enc_data, blksize);
            sifli_hw_dec(dfu_key, enc_data, dfu_temp + blk_offset, blksize, offset);
        }
        else
        {
            dfu_read_storage_data(header, offset, dfu_temp + blk_offset, blksize);
        }
        header->flag &= ~DFU_FLAG_COMPRESS;
        //header->flag &= ~DFU_FLAG_ENC;

        total_len -= blksize;
        offset += blksize;

        // paser the 1st part
        if (img_header_get)
        {
            uint32_t temp_left, temp_offset;
            img_header_get = 0;
            struct img_header_compress_info *hdr;
            //get the first img
            // len is decided by pksize, default is 10K
            hdr = (struct img_header_compress_info *)dfu_temp;
            pksize = hdr->pksize;
            total_hdr_len = total_uncompress_len = hdr->total_len;
            g_uncompress_len = total_uncompress_len;
            LOG_I("uncompre len %d \r\n", total_uncompress_len);

            dfu_packet_erase_flash(header, 0, total_uncompress_len);

            packet_len = ((dfu_compress_packet_header_t *)(dfu_temp + sizeof(struct img_header_compress_info)))->packet_len;

            uncompress_buf = malloc(pksize);
            OS_ASSERT(uncompress_buf);

            temp_offset = sizeof(struct img_header_compress_info) + sizeof(dfu_compress_packet_header_t);
            temp_left = blksize - temp_offset;
            // Handle compress pksize smaller than bksize
            while (packet_len < temp_left)
            {
                r = dfu_decompress(header, dfu_key, uncompress_buf, &pksize, dfu_temp + temp_offset,
                                   &packet_len, &total_uncompress_len, &uncompress_offset);

                if (r != DFU_DECOM_OK)
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

            if (r != DFU_DECOM_OK)
                break;

            // blk_offset has value means packet_len could not get
            if (blk_offset == 0)
            {
                // already make sure packet len is larger than left data in dfu_temp
                compress_buf = malloc(packet_len);
                OS_ASSERT(compress_buf);

                memcpy(compress_buf, dfu_temp + temp_offset, temp_left);
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
                OS_ASSERT(packet_len >= com_offset);

                uint32_t left_size = packet_len - com_offset;
                uint32_t left_len = com_offset + blksize - packet_len;

                // handle the combine buffer : compress_buf
                memcpy(compress_buf + com_offset, dfu_temp, left_size);
                r = dfu_decompress(header, dfu_key, uncompress_buf, &pksize, compress_buf, &packet_len,
                                   &total_uncompress_len, &uncompress_offset);
                free(compress_buf);
                compress_buf = NULL;
                com_offset = 0;
                if (r != DFU_DECOM_OK)
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

                    r = dfu_decompress(header, dfu_key, uncompress_buf, &pksize, dfu_temp + left_size,
                                       &packet_len, &total_uncompress_len, &uncompress_offset);
                    left_len -= packet_len;
                    left_size += packet_len;

                    if (r != DFU_DECOM_OK)
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
                    compress_buf = malloc(packet_len);
                    OS_ASSERT(compress_buf);
                    memcpy(compress_buf, dfu_temp + left_size, left_len);
                    com_offset += left_len;
                }
                // To avoid the last packet
                //LOG_I("After uncompre len %d, total len %d \r\n", total_uncompress_len, total_len);
            }
            else
            {
                // if last compress packet is less than blksize,
                if (!is_last_small_packet)
                {
                    if (total_uncompress_len < pksize)
                    {
                        // already last packet
                    }
                    else
                    {
                        if (total_uncompress_len - pksize < blksize)
                        {
                            LOG_I("is_last_small_packet %d, %d, %d", total_uncompress_len, total_uncompress_len - pksize, blksize);
                            is_last_small_packet = 1;
                        }
                    }
                }

                if (total_len < blksize && is_last_small_packet)
                {
                    LOG_I("ADD LAST PACKET");
                    total_len += blksize;
                }

                memcpy(compress_buf + com_offset, dfu_temp, blksize);
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
                r = dfu_decompress(header, dfu_key, uncompress_buf, &pksize, dfu_temp + temp_offset,
                                   &packet_len, &total_uncompress_len, &uncompress_offset);
                temp_left -= packet_len;
                temp_offset += packet_len;

                if (r != DFU_DECOM_OK)
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
                compress_buf = malloc(packet_len);
                OS_ASSERT(compress_buf);

                memcpy(compress_buf, dfu_temp + temp_offset, temp_left);
                com_offset += temp_left;
            }
        }
    }

END:
    LOG_I("total len %d, uncom len %d, compress buf %x\r\n", total_len, total_uncompress_len, compress_buf);
    //RT_ASSERT(compress_buf == NULL);
    free(uncompress_buf);
    free(enc_data);
    free(dfu_temp);

    if (r == DFU_SUCCESS)
    {
        curr_info->img_state = DFU_CTRL_IMG_STATE_DOWNLOADED;
        curr_info->header->length = total_hdr_len;
        dfu_ctrl_update_prog_info(env);
    }

    LOG_I("Instasll r %d\r\n", r);
    return r;
}

//#endif
#ifdef DFU_LCPU_ROM_PATCH_COMPARE
static int dfu_img_lcpu_compare()
{
    uint32_t *addr_lcpu;
    addr_lcpu = (uint32_t *)LCPU_PATCH_START_ADDR;
    uint32_t *addr_lcpu_temp;
    addr_lcpu_temp = (uint32_t *)(DFU_RES_FLASH_CODE_START_ADDR + OTA_NOR_LCPU_ROM_PATCH_SIZE);
    LOG_I("dfu_img_lcpu_compare 0x%x, 0x%x, %d", addr_lcpu, addr_lcpu_temp, g_uncompress_len);

    int ret;
    ret = memcmp(addr_lcpu, addr_lcpu_temp, g_uncompress_len);


    LOG_I("dfu_img_lcpu_compare ret %d", ret);
    return ret;
}
#endif

int dfu_img_install_lcpu_rom_patch(dfu_ctrl_env_t *env)
{
    dfu_dl_image_header_t *dl_header = &env->prog.fw_context.code_img;
    uint8_t key[DFU_KEY_SIZE] = {0};
    int r = DFU_SUCCESS;
    int ret;

    for (uint32_t i = 0; i < dl_header->img_count; i++)
    {
        if (dl_header->img_header[i].flag & DFU_FLAG_COMPRESS)
        {
            if (dl_header->img_header[i].img_id == DFU_IMG_ID_PATCH)
            {
                LOG_I("install lcpu patch");
                if (dl_header->img_header[i].flag & DFU_FLAG_ENC)
                {
#ifdef DFU_LCPU_ROM_PATCH_COMPARE
                    sifli_hw_dec_key(env->prog.FW_key, key, DFU_KEY_SIZE);

                    dl_header->img_header[i].img_id = DFU_IMG_ID_PATCH_TEMP;
                    r = dfu_img_install_flash(env, key, &dl_header->img_header[i]);
                    dl_header->img_header[i].img_id = DFU_IMG_ID_PATCH;
                    ret = dfu_img_lcpu_compare();
#else
                    ret = 1;
#endif
                    if (ret != 0)
                    {
                        LOG_I("lcpu rom patch is different, install");
                        HAL_RCC_Reset_and_Halt_LCPU(0);
                        r = dfu_img_install_flash(env, key, &dl_header->img_header[i]);
                    }

                    if (r != DFU_SUCCESS)
                    {
                        LOG_E("Install failed!(%d)", dl_header->img_header[i].img_id);
                        break;
                    }
                }
                else
                {
#ifdef DFU_LCPU_ROM_PATCH_COMPARE
                    dl_header->img_header[i].img_id = DFU_IMG_ID_PATCH_TEMP;
                    r = dfu_img_install_flash(env, NULL, &dl_header->img_header[i]);
                    dl_header->img_header[i].img_id = DFU_IMG_ID_PATCH;
                    ret = dfu_img_lcpu_compare();
#else
                    ret = 1;
#endif
                    if (ret != 0)
                    {
                        LOG_I("lcpu rom patch is different, install");
                        HAL_RCC_Reset_and_Halt_LCPU(0);
                        r = dfu_img_install_flash(env, NULL, &dl_header->img_header[i]);
                    }

                    if (r != DFU_SUCCESS)
                    {
                        LOG_E("Install failed!(%d)", dl_header->img_header[i].img_id);
                        break;
                    }
                }
            }
        }
    }
    return r;
}

int dfu_img_install(dfu_ctrl_env_t *env)
{
    dfu_dl_image_header_t *dl_header = &env->prog.fw_context.code_img;
    uint8_t key[DFU_KEY_SIZE] = {0};
    int r = DFU_SUCCESS;
#ifdef DFU_OTA_MANAGER
    for (uint32_t i = 0; i < dl_header->img_count; i++)
    {
        if (dl_header->img_header[i].flag & DFU_FLAG_COMPRESS)
        {
            if (dl_header->img_header[i].img_id == DFU_IMG_ID_PATCH)
            {
                // install at last
                continue;
            }

            if (dl_header->img_header[i].flag & DFU_FLAG_ENC)
            {
                sifli_hw_dec_key(env->prog.FW_key, key, DFU_KEY_SIZE);
                r = dfu_img_install_flash(env, key, &dl_header->img_header[i]);
                if (r != DFU_SUCCESS)
                {
                    LOG_E("Install failed!(%d)", dl_header->img_header[i].img_id);
                    break;
                }
            }
            else
            {
                r = dfu_img_install_flash(env, NULL, &dl_header->img_header[i]);
                if (r != DFU_SUCCESS)
                {
                    LOG_E("Install failed!(%d)", dl_header->img_header[i].img_id);
                    break;
                }

            }
        }
    }
#else
    dl_header = (dfu_dl_image_header_t *)&env->ota_state.fw_context.code_img;
    for (uint32_t i = 0; i < dl_header->img_count; i++)
    {
        if (dl_header->img_header[i].flag & DFU_FLAG_COMPRESS)
        {
            if (dl_header->img_header[i].flag & DFU_FLAG_ENC)
            {
                LOG_E("Install ota manager enc %d", dl_header->img_header[i].img_id);

                sifli_hw_dec_key(env->ota_state.FW_key, key, DFU_KEY_SIZE);
                r = dfu_img_install_flash(env, key, &dl_header->img_header[i]);

                if (r != DFU_SUCCESS)
                {
                    LOG_E("Install failed!(%d)", dl_header->img_header[i].img_id);
                }
            }
            else
            {
                LOG_E("Install ota manager no enc %d", dl_header->img_header[i].img_id);

                r = dfu_img_install_flash(env, NULL, &dl_header->img_header[i]);
                if (r != DFU_SUCCESS)
                {
                    LOG_E("Install failed!(%d)", dl_header->img_header[i].img_id);
                }
            }
        }
    }
#endif
    return r;
}



#endif /* OTA_55X */
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
