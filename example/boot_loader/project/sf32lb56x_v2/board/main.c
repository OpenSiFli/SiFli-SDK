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
#include <boot_board.h>
#include <string.h>
#include "register.h"
#include "../dfu/dfu.h"
#include "boot_flash.h"

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
            if (coreid == CORE_HCPU || coreid == CORE_BL || coreid == CORE_LCPU)
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
        if (coreid == CORE_HCPU || coreid == CORE_BL || coreid == CORE_LCPU)
        {
            if (is_addr_in_nor(dest))
                HAL_FLASH_ALIAS_CFG(boot_handle, dest, img_hdr->length, src - dest);
            else if (src != dest)
                g_flash_read(src, (const int8_t *)dest, img_hdr->length);
            run_img(dest);
        }
    }
}

void boot_images_help()
{
    dfu_install_info info;
    dfu_install_info info_ext;
    if (DFU_DOWNLOAD_REGION_START_ADDR != FLASH_UNINIT_32)
    {
        g_flash_read(DFU_DOWNLOAD_REGION_START_ADDR, (const int8_t *)&info, sizeof(dfu_install_info));
    }
    if (DFU_INFO_REGION_START_ADDR != FLASH_UNINIT_32)
    {
        g_flash_read(DFU_INFO_REGION_START_ADDR, (const int8_t *)&info_ext, sizeof(dfu_install_info));
    }
    if (info.magic == SEC_CONFIG_MAGIC && info_ext.magic == SEC_CONFIG_MAGIC)
    {
        info = info_ext;
    }

    if (DFU_DOWNLOAD_REGION_START_ADDR != FLASH_UNINIT_32)
    {
        if ((HAL_Get_backup(RTC_BAKCUP_OTA_FORCE_MODE) == DFU_FORCE_MODE_REBOOT_TO_PACKAGE_OTA_MANAGER) ||
                (info.magic == SEC_CONFIG_MAGIC) && (info.install_state == DFU_PACKAGE_INSTALL))
        {
            sec_config_cache.running_imgs[CORE_HCPU] = (struct image_header_enc *) & (((struct sec_configuration *)FLASH_TABLE_START_ADDR)->imgs[DFU_FLASH_IMG_IDX(DFU_FLASH_IMG_LCPU)]);
        }
    }

    if (sec_config_cache.magic == SEC_CONFIG_MAGIC)
    {
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


