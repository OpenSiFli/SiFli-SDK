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

int board_boot_src;
struct sec_configuration sec_config_cache;

typedef void (*ram_hook_handler)(void);
void boot_ram(void)
{
    volatile ram_hook_handler hook = (volatile ram_hook_handler)hwp_hpsys_aon->RESERVE0;
    if (hook)
        hook();
}

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

    if (coreid < 2 * CORE_MAX)
    {
        coreid %= CORE_MAX;
        if (coreid == CORE_HCPU || coreid == CORE_BL)
        {
            if (src != dest)
                memcpy((void *)dest, (uint8_t *)src, img_hdr->length);
            else
                HAL_FLASH_ALIAS_CFG(boot_handle, dest, img_hdr->length, src - dest);
            run_img(dest);
        }
    }
}

void boot_images_help()
{
    if (sec_config_cache.magic == SEC_CONFIG_MAGIC && sec_config_cache.running_imgs[CORE_HCPU] != (struct image_header_enc *)FLASH_UNINIT_32)
    {
        int flash_id = ((uint32_t)sec_config_cache.running_imgs[CORE_HCPU] - g_config_addr - 0x1000) / sizeof(struct image_header_enc) + DFU_FLASH_IMG_LCPU;
        dfu_boot_img_in_flash(flash_id);
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

    // 3. Power on flash.
    board_flash_power_on();

    // 4. Check boot mode.
    HAL_MspInit();

    // 6. Read boot options
    board_boot_src = board_boot_from();

    /* init AES_ACC as normal mode */
    __HAL_SYSCFG_CLEAR_SECURITY();
    dfu_flash_init();
    boot_images_help();

    while (1);
    return HAL_OK;
}


