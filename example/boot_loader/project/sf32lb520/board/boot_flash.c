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
#include <string.h>
#include <register.h>
#include "../dfu/dfu.h"
#include "bf0_hal.h"
#include "board.h"
#include "boot_flash.h"
#include "sifli_bbm.h"

DMA_HandleTypeDef spi_flash_dma_handle[FLASH_MAX_INSTANCE];
QSPI_FLASH_CTX_T spi_flash_handle[FLASH_MAX_INSTANCE];
FLASH_HandleTypeDef *boot_handle;
uint32_t g_config_addr;

static int read_nor(uint32_t addr, const int8_t *buf, uint32_t size)
{
    memcpy((void *)buf, (uint8_t *)addr, size);
    return size;
}

static uint32_t init_mpi1()
{
    // Initialize MPI1
    qspi_configure_t flash_cfg = FLASH1_CONFIG;
    struct dma_config flash_dma = FLASH1_DMA_CONFIG;

    spi_flash_handle[0].dual_mode = 1;
    flash_cfg.line = HAL_FLASH_NOR_MODE;
    if (board_boot_src == BOOT_FROM_SIP_PUYA)
    {
        board_pinmux_mpi1_puya_base();
    }
    else
    {
        board_pinmux_mpi1_gd();
        flash_cfg.line = HAL_FLASH_QMODE;
    }
    HAL_FLASH_Init(&(spi_flash_handle[0]), &flash_cfg, &spi_flash_dma_handle[0], &flash_dma, BSP_GetFlash1DIV());
    if (board_boot_src == BOOT_FROM_SIP_PUYA)
    {
        // check spi_flash_handle[0].dev_id == 0x176085 // 0x856017
        board_pinmux_mpi1_puya_ext(spi_flash_handle[0].dev_id == 0x176085);
        HAL_FLASH_SET_QUAL_SPI((FLASH_HandleTypeDef *) & (spi_flash_handle[0].handle), true);
        spi_flash_handle[0].handle.Mode = HAL_FLASH_QMODE;
    }
    return (spi_flash_handle[0].base_addr);
}

/******************************************************************************/
void dfu_flash_init()
{
    // set clock to high speed for bootloader
    HAL_HPAON_EnableXT48();
    HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_SYS, RCC_SYSCLK_HXT48);
    HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_HP_PERI, RCC_CLK_PERI_HXT48);


    HAL_PMU_EnableDLL(1);

    HAL_RCC_HCPU_ConfigHCLK(144);
    //HAL_RCC_HCPU_EnableDLL1(240000000);
    //HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_SYS, RCC_SYSCLK_DLL1);
    // Reset sysclk used by HAL_Delay_us
    HAL_Delay_us(0);

    switch (board_boot_src)
    {
    case BOOT_FROM_SIP_PUYA:
    case BOOT_FROM_SIP_GD:
        HAL_RCC_HCPU_EnableDLL2(288000000);
        BSP_SetFlash1DIV(6);    // bootloader use DLL2 / 6, 288/6 = 48
        HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_FLASH1, RCC_CLK_FLASH_DLL2);
        g_config_addr = init_mpi1();
        boot_handle = (FLASH_HandleTypeDef *)&spi_flash_handle[0].handle;
        break;
    default:
        boot_error('S');
    }
#define MAX_RETRY 5
    int count = MAX_RETRY;

    while (count)
    {
        read_nor(g_config_addr, (const int8_t *)&sec_config_cache, sizeof(struct sec_configuration));
        if (sec_config_cache.magic == SEC_CONFIG_MAGIC)
            break;
        HAL_Delay_us(1000000);
        count--;
    }
    if (count == 0)
        boot_error('C');
}

