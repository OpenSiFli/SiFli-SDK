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
#include "secboot.h"

int board_boot_src;
struct sec_configuration sec_config_cache;

typedef void (*ram_hook_handler)(void);
void boot_ram(void)
{
    volatile ram_hook_handler hook = (volatile ram_hook_handler)hwp_hpsys_aon->RESERVE0;
    if (hook)
        hook();
}


//#define BOOT_TEST
#ifdef BOOT_TEST
void boot_test(void)
{
    uint32_t delay = (100 << BOOT_PU_Delay_Pos) | (200 << BOOT_PD_Delay_Pos);
    if (HAL_Get_backup(RTC_BACKUP_BOOTOPT + 1) == 0)
    {
        HAL_Set_backup(RTC_BACKUP_BOOTOPT, delay);
        HAL_Set_backup(RTC_BACKUP_BOOTOPT + 1, 1);
        HAL_PMU_Reboot();
    }
    else
    {
        boot_uart_tx(hwp_usart1, (uint8_t *)"E", 1);
        __asm("B .");
    }
}
#else
#define boot_test()
#endif

/**************************Efuse**************************************************/
#define boot_efuse_init_stage1(void) \
{ \
    hwp_efusec->TIMR = 0x2D08F; \
    /* Read bank0 */ \
    sifli_hw_efuse_read_bank(0); \
}

#define boot_efuse_init_stage2(void) \
{ \
    /* Read bank3 */ \
    sifli_hw_efuse_read_bank(3); \
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
    struct sec_configuration *sec_config = &sec_config_cache;

    if (img_hdr->flags & DFU_FLAG_ENC)
    {
        uint32_t is_flash = 1;

        /* verify public sig_key hash */
        if (sifli_sigkey_pub_verify(sec_config->sig_pub_key, DFU_SIG_KEY_SIZE))
            sifli_secboot_exception(SECBOOT_SIGKEY_PUB_ERR);

        if (coreid < 2 * CORE_MAX)
        {
            coreid %= CORE_MAX;
            // Read Root key
            boot_efuse_init_stage2();
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
#ifdef PKG_SIFLI_MBEDTLS_BOOT
                /* verify image hash signature */
                if (sifli_img_sig_hash_verify(img_hdr->sig, sec_config->sig_pub_key, (uint8_t *)dest, img_hdr->length))
                    sifli_secboot_exception(SECBOOT_IMG_HASH_SIG_ERR);
#endif
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
    if (sec_config_cache.magic == SEC_CONFIG_MAGIC)
    {
#ifdef  CFG_BOOTROM
        if (sec_config_cache.running_imgs[CORE_BL] != (struct image_header_enc *)FLASH_UNINIT_32)
        {
            int flash_id = ((uint32_t)sec_config_cache.running_imgs[CORE_BL] - g_config_addr - 0x1000) / sizeof(struct image_header_enc)
                           + DFU_FLASH_IMG_LCPU;
            dfu_boot_img_in_flash(flash_id);
        }
#else

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

        if (sec_config_cache.running_imgs[CORE_HCPU] != (struct image_header_enc *)FLASH_UNINIT_32)
        {
            int flash_id = ((uint32_t)sec_config_cache.running_imgs[CORE_HCPU] - g_config_addr - 0x1000) / sizeof(struct image_header_enc) + DFU_FLASH_IMG_LCPU;
            board_init_psram();
            dfu_boot_img_in_flash(flash_id);
        }
#endif
    }
}

void hw_preinit0(void)
{
    if (__HAL_SYSCFG_GET_REVID() < HAL_CHIP_REV_ID_A4)
    {
        /* lower power on threshold and set VBAT_LDO output voltage to default 3.3V*/
        MODIFY_REG(hwp_pmuc->AON_LDO, PMUC_AON_LDO_VBAT_POR_TH_Msk | PMUC_AON_LDO_VBAT_LDO_SET_VOUT_Msk,
                   MAKE_REG_VAL(0, PMUC_AON_LDO_VBAT_POR_TH_Msk, PMUC_AON_LDO_VBAT_POR_TH_Pos)
                   | MAKE_REG_VAL(6, PMUC_AON_LDO_VBAT_LDO_SET_VOUT_Msk, PMUC_AON_LDO_VBAT_LDO_SET_VOUT_Pos));

        /* auto power down if VCC is low */
        hwp_pmuc->WER |= PMUC_WER_LOWBAT;
    }

    HAL_Delay_us(0);

    // 1. Read efuse bank0 first to take efuse effect.
    boot_efuse_init_stage1();

    // 2. If ram hook existed, just jump to ram.
    boot_ram();
}

/**************************main**************************************/

#if defined(__CC_ARM) || defined(__CLANG_ARM)
    int main(void)
#elif defined(__ICCARM__)
    int __low_level_init(void)
#elif defined(__GNUC__)
    int entry(void)
#endif
{
    HAL_Delay_us(0);

    if (__HAL_SYSCFG_GET_REVID() >= HAL_CHIP_REV_ID_A4)
    {
        // 3. Power on flash.
        board_flash_power_on();

        // 4. Check boot mode.
        HAL_MspInit();

        // 5. Boot images
#ifdef CFG_BOOTROM
        if (hwp_hpsys_cfg->BMR == 0)
#endif
        {
            // 6. Read boot options
            board_boot_src = board_boot_from();

            /* init AES_ACC as normal mode */
            __HAL_SYSCFG_CLEAR_SECURITY();
            dfu_flash_init();
            boot_images_help();
        }
    }
    else
    {
        // 3. Read boot options
        board_boot_src = board_boot_from();

        // 4. Power on flash.
        board_flash_power_on();

        // 5. Check boot mode.
        HAL_MspInit();

        // 6. Boot images
#ifdef CFG_BOOTROM
        if (hwp_hpsys_cfg->BMR == 0)
#endif
        {
            /* init AES_ACC as normal mode */
            __HAL_SYSCFG_CLEAR_SECURITY();
            dfu_flash_init();
            boot_images_help();
        }
    }

    while (1)
        ;

    return HAL_OK;
}


