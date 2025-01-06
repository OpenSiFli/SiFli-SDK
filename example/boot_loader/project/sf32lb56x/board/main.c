/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-06     zylx         first version
 */

#include <rtconfig.h>
#include <board.h>
#include <string.h>
#include "register.h"
#include "../dfu/dfu.h"
#include "boot_flash.h"
#include "../dfu/dfu_protocol.h"

extern flash_read_func g_flash_read_mpi5;
extern flash_write_func g_flash_write_mpi5;
extern flash_erase_func g_flash_erase_mpi5;

extern flash_read_func g_flash_read;

int board_boot_src;
struct sec_configuration sec_config_cache;

typedef void (*ram_hook_handler)(void);
void boot_ram(void)
{
    volatile ram_hook_handler hook = (volatile ram_hook_handler)hwp_hpsys_aon->RESERVE0;
    if (hook)
        hook();
}



/************************Boot *****************************************/

void run_img(uint32_t dest)
{
    __asm("LDR SP, [%0]" :: "r"(dest));
    __asm("LDR PC, [%0, #4]" :: "r"(dest));
}

uint8_t is_addr_in_nor(uint32_t addr)
{
    if (boot_handle && boot_handle->isNand == 0 &&
            addr >= boot_handle->base && addr < boot_handle->base + boot_handle->size)
        return 1;
    else
        return 0;
}

void dfu_boot_img_in_flash(int flashid)
{
    uint32_t src = sec_config_cache.ftab[flashid].base;
    uint32_t dest = sec_config_cache.ftab[flashid].xip_base;
    int coreid = DFU_FLASH_IMG_IDX(flashid);
    struct image_header_enc *img_hdr = &(sec_config_cache.imgs[coreid]);

    if (img_hdr->flags & DFU_FLAG_ENC)
    {
        uint32_t is_flash = 1;

        if (coreid < 2 * CORE_MAX)
        {
            coreid %= CORE_MAX;
            if (coreid == CORE_HCPU || coreid == CORE_BL)
            {
                ALIGN(4)
                static uint8_t dfu_key[DFU_KEY_SIZE];
                /** key in plaintext */
                ALIGN(4)
                static uint8_t dfu_key1[DFU_KEY_SIZE];
                if (is_addr_in_nor(dest))
                {
                    memcpy(dfu_key, img_hdr->key, sizeof(dfu_key));
                    sifli_hw_init_xip_key(dfu_key);

                    // Setup XIP for decoding and running
                    HAL_FLASH_NONCE_CFG(boot_handle, dest, dest + img_hdr->length, dfu_get_counter(0));
                    if (is_flash)
                        HAL_FLASH_ALIAS_CFG(boot_handle, dest, img_hdr->length, src - dest);
                    HAL_FLASH_AES_CFG(boot_handle, 1);          /* enable on-the-fly decoder */
                }
                else
                {
                    /* copy encrypted key to ram as AES_ACC cannot access flash */
                    memcpy(dfu_key, img_hdr->key, sizeof(dfu_key));
                    sifli_hw_dec_key(dfu_key, dfu_key1, sizeof(dfu_key1));
                    g_flash_read(src, (const int8_t *)dest, img_hdr->length);
                    sifli_hw_dec(dfu_key1, (uint8_t *)dest, (uint8_t *)dest, img_hdr->length, 0);
                }
                run_img(dest);
            }
        }
    }
    if (coreid < 2 * CORE_MAX)
    {
        coreid %= CORE_MAX;
        if (coreid == CORE_HCPU || coreid == CORE_BL)
        {
            if (is_addr_in_nor(dest))
                HAL_FLASH_ALIAS_CFG(boot_handle, dest, img_hdr->length, src - dest);
            else if (src != dest)
                g_flash_read(src, (const int8_t *)dest, img_hdr->length);
            run_img(dest);
        }
    }
}

struct sec_configuration g_temp_sec_config;
struct sec_configuration *temp_sec_config;


static void update_sec_flash(struct sec_configuration *sec_config)
{
    uint32_t flash_addr = MPI5_MEM_BASE;
    //uint32_t size = (sizeof(struct sec_configuration) + SPI_NAND_PAGE_SIZE * 2) & (~(SPI_NAND_PAGE_SIZE * 2 - 1));
    uint32_t size = sizeof(struct sec_configuration);
    if (size % 4096 != 0)
    {
        size = (size + 4096) / 4096 * 4096;
    }
    g_flash_erase_mpi5(flash_addr, size);
    g_flash_write_mpi5(flash_addr, (const int8_t *)sec_config, sizeof(struct sec_configuration));

    //flash_config = (struct sec_configuration *)MPI5_MEM_BASE;
}


static void select_boot()
{
    uint8_t hcpu_des = HAL_Get_backup(RTC_BACKUP_NAND_OTA_DES);
    temp_sec_config = &g_temp_sec_config;

    struct sec_configuration *flash_config = (struct sec_configuration *)MPI5_MEM_BASE;
    //memcpy((const int8_t *)temp_sec_config, (uint8_t *)MPI5_MEM_BASE, sizeof(struct sec_configuration));
    g_flash_read_mpi5(MPI5_MEM_BASE, (const int8_t *)temp_sec_config, sizeof(struct sec_configuration));

    int hcpu1_img_idx = DFU_FLASH_IMG_IDX(DFU_FLASH_IMG_HCPU);
    int hcpu2_img_idx = DFU_FLASH_IMG_IDX(DFU_FLASH_IMG_HCPU2);
    //HAL_sw_breakpoint();

    if (sec_config_cache.running_imgs[CORE_HCPU] == &(flash_config->imgs[hcpu1_img_idx]))
    {
        // now we are running on hcpu1
        switch (hcpu_des)
        {
        case DFU_DES_RUNNING_ON_HCPU1:
        {
            // normal power on hcpu1
            break;
        }
        case DFU_DES_SWITCH_TO_HCPU2:
        {
            // HAL_sw_breakpoint();
            // switch to hcpu2

            sec_config_cache.running_imgs[CORE_HCPU] = &(flash_config->imgs[hcpu2_img_idx]);

            hcpu_des = DFU_DES_UPDATE_HCPU2;
            HAL_Set_backup(RTC_BACKUP_NAND_OTA_DES, hcpu_des);

            temp_sec_config->running_imgs[CORE_HCPU] = &(flash_config->imgs[hcpu2_img_idx]);
            update_sec_flash(temp_sec_config);
            break;
        }
        case DFU_DES_UPDATE_HCPU2:
        {
            // switch to hcpu2 fail, back to hcpu1
            sec_config_cache.running_imgs[CORE_HCPU] = &(flash_config->imgs[hcpu1_img_idx]);
            hcpu_des = DFU_DES_RUNNING_ON_HCPU1;
            HAL_Set_backup(RTC_BACKUP_NAND_OTA_DES, hcpu_des);

            temp_sec_config->running_imgs[CORE_HCPU] = &(flash_config->imgs[hcpu1_img_idx]);
            update_sec_flash(temp_sec_config);
            break;
        }
        case DFU_DES_NONE:
        {
            // first power on, use current
            hcpu_des = DFU_DES_RUNNING_ON_HCPU1;
            HAL_Set_backup(RTC_BACKUP_NAND_OTA_DES, hcpu_des);
            break;
        }
        default:
            hcpu_des = DFU_DES_RUNNING_ON_HCPU1;
            HAL_Set_backup(RTC_BACKUP_NAND_OTA_DES, hcpu_des);
        }

    }
    else if (sec_config_cache.running_imgs[CORE_HCPU] == &(flash_config->imgs[hcpu2_img_idx]))
    {
        // now we are running on hcpu2
        switch (hcpu_des)
        {
        case DFU_DES_RUNNING_ON_HCPU2:
        {
            // normal power on hcpu2
            break;
        }
        case DFU_DES_SWITCH_TO_HCPU1:
        {
            // switch to hcpu1
            sec_config_cache.running_imgs[CORE_HCPU] = &(flash_config->imgs[hcpu1_img_idx]);
            hcpu_des = DFU_DES_UPDATE_HCPU1;
            HAL_Set_backup(RTC_BACKUP_NAND_OTA_DES, hcpu_des);

            temp_sec_config->running_imgs[CORE_HCPU] = &(flash_config->imgs[hcpu1_img_idx]);
            update_sec_flash(temp_sec_config);
            break;
        }
        case DFU_DES_UPDATE_HCPU1:
        {
            // switch to hcpu1 fail, back to hcpu2
            sec_config_cache.running_imgs[CORE_HCPU] = &(flash_config->imgs[hcpu2_img_idx]);
            hcpu_des = DFU_DES_RUNNING_ON_HCPU2;
            HAL_Set_backup(RTC_BACKUP_NAND_OTA_DES, hcpu_des);

            temp_sec_config->running_imgs[CORE_HCPU] = &(flash_config->imgs[hcpu2_img_idx]);
            update_sec_flash(temp_sec_config);
            break;
        }
        case DFU_DES_NONE:
        {
            // first power on, use current
            hcpu_des = DFU_DES_RUNNING_ON_HCPU2;
            HAL_Set_backup(RTC_BACKUP_NAND_OTA_DES, hcpu_des);
            break;
        }
        default:
            hcpu_des = DFU_DES_RUNNING_ON_HCPU2;
            HAL_Set_backup(RTC_BACKUP_NAND_OTA_DES, hcpu_des);
        }
    }
}

void boot_images_help()
{
    if (sec_config_cache.magic == SEC_CONFIG_MAGIC)
    {
        select_boot();
        if (sec_config_cache.running_imgs[CORE_HCPU] != (struct image_header_enc *)FLASH_UNINIT_32)
        {
            int flash_id = ((uint32_t)sec_config_cache.running_imgs[CORE_HCPU] - g_config_addr - 0x1000) / sizeof(struct image_header_enc) + DFU_FLASH_IMG_LCPU;
            if (BOOT_FROM_NAND())
            {
                extern void board_init_psram();
                board_init_psram();
#ifdef SD_BL_MODE
                extern uint32_t init_sdnand();
                init_sdnand();
#endif
            }
            dfu_boot_img_in_flash(flash_id);
        }
    }
}

#if defined(__CC_ARM) || defined(__CLANG_ARM)
    int main(void)
#elif defined(__ICCARM__)
    int __low_level_init(void)
#elif defined(__GNUC__)
    int entry(void)
#endif
{
    // 1. Read boot options
    // board_boot_src = board_boot_from();

    // 2. Power on flash.
    board_flash_power_on();

    // 3. Check boot mode.
    HAL_MspInit();

    // 4. Boot images
    if (hwp_hpsys_cfg->BMR == 0)
    {
        /* init AES_ACC as normal mode */
        __HAL_SYSCFG_CLEAR_SECURITY();
        dfu_flash_init();
        boot_images_help();
    }

    while (1);

    return HAL_OK;
}


