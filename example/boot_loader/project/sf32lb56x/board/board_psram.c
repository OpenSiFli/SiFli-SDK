/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-12-21     zylx         first version
 */

#include "rtconfig.h"
#include "board.h"
#include "boot_flash.h"
#include "string.h"

#include "mem_map.h"

enum
{
    BF0_QPSRAM_RD   = 0x03,   // Read
    BF0_QPSRAM_FRD  = 0x0b,   // Fast Read
    BF0_QPSRAM_QRD  = 0xeb,   // Quad Read
    BF0_QPSRAM_WR   = 0x02,   // Write
    BF0_QPSRAM_QWR  = 0x38,   // Quad Write
    BF0_QPSRAM_WRD  = 0x8b,   // Wrapped Read
    BF0_QPSRAM_WWR  = 0x82,   // Wrapped Write
    BF0_QPSRAM_MRR  = 0xb5,   // Mode Register Read
    BF0_QPSRAM_MRW  = 0xb1,   // Mode Register Write
    BF0_QPSRAM_QEN  = 0x35,   // Quad Mode Enable
    BF0_QPSRAM_QDIS = 0xf5,   // Quad Mode Disable
    BF0_QPSRAM_RSTE = 0x66,   // Reset Enable
    BF0_QPSRAM_RST  = 0x99,   // Reset
    BF0_QPSRAM_BLT  = 0xc0,   // Burst Length Toggle
    BF0_QPSRAM_RDID = 0x9f    // Read ID
};

enum
{
    OPSRAM_RD   = 0x00,   // Read
    OPSRAM_WR   = 0x80,   // Write
    OPSRAM_LRD  = 0x20,   // Linear Read
    OPSRAM_LWR  = 0xa0,   // Linear Write
    OPSRAM_MRR  = 0x40,   // Mode Register Read
    OPSRAM_MRW  = 0xc0,   // Mode Register Write
    OPSRAM_RST  = 0xff    // Global Reset
};

static FLASH_HandleTypeDef psram_hdl;

__STATIC_INLINE void dwtIpInit(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

__STATIC_INLINE void dwtIpDeinit(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk;
}

__STATIC_INLINE void dwtReset(void)
{
    DWT->CYCCNT = 0; /* Clear DWT cycle counter */
}

__STATIC_INLINE uint32_t dwtGetCycles(void)
{
    return DWT->CYCCNT;
}

#define SFLB56X_PID_566     1
#define SFLB56X_PID_567     2
#define SFLB56X_PID_565_VCN 3
#define SFLB56X_PID_565_VBN 4
#define SFLB56X_PID_565_VBB 5

void bootloader_switch_clock(int mpi)
{
    // for hyper bus psram, need switch clock?
    HAL_HPAON_EnableXT48();

    HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_SYS, RCC_SYSCLK_HXT48);
    //HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_HP_PERI, RCC_CLK_PERI_HXT48);

    if (__HAL_SYSCFG_GET_PID() == SFLB56X_PID_567)
    {
        // Enable SA/SB for DLLs
        HAL_PIN_Set(PAD_PA72, GPIO_A72, PIN_PULLUP, 1);               // SB_EN/MPI2_EN
        HAL_PIN_Set(PAD_PA74, GPIO_A74, PIN_PULLUP, 1);               // SA_EN/MPI1_EN
    }

    HAL_PMU_EnableDLL(1);

    __HAL_SYSCFG_HPBG_EN();
    __HAL_SYSCFG_HPBG_VDDPSW_EN();

    dwtIpInit();
    /* wait until 2ms elapse to ensure flash LDO is stable before flash access
     * default clock is 24MHz, maybe slower than 24MHz at startup stage
     *
     */
    while (dwtGetCycles() < (2 * 24000))
    {
    }

    HAL_RCC_HCPU_EnableDLL1(240000000);
    /* set hdiv again to make HCLK run on 240MHz actually
     * Although the default hdiv registe value is 1, hardware would use 2 as the default hdiv.
     * After write the hdiv register, hardware would use the updated value.
     */
    HAL_RCC_HCPU_SetDiv(1, 2, 5);
    HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_SYS, RCC_SYSCLK_DLL1);

    dwtIpInit();
    /* wait until 2ms elapse to ensure flash LDO is stable before flash access
     * default clock is 24MHz, maybe slower than 24MHz at startup stage
     *
     */
    while (dwtGetCycles() < (20 * 2400))
    {
    }

    HAL_RCC_HCPU_EnableDLL2(240000000);
    HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_SDMMC, RCC_CLK_FLASH_DLL2);

    if (mpi == 1)
        HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_FLASH1, RCC_CLK_FLASH_DLL2);
    else
        HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_FLASH2, RCC_CLK_FLASH_DLL2);
}

void boot_psram_init(uint8_t mpi, uint8_t type, uint8_t size)
{
    FLASH_HandleTypeDef *handle;
    qspi_configure_t qspi_cfg;
    uint16_t div;

    /* init as 0 in case reach here before bss is initialized */
    memset(&psram_hdl, 0, sizeof(psram_hdl));
    memset(&qspi_cfg, 0, sizeof(qspi_cfg));

    if (mpi == BOOT_MPI1_PSRAM)
    {
        qspi_cfg.Instance = hwp_mpi1;
        qspi_cfg.base = MPI1_MEM_BASE;
        div = BSP_GetFlash1DIV();
    }
    else if (mpi == BOOT_MPI2_PSRAM)
    {
        qspi_cfg.Instance = hwp_mpi2;
        qspi_cfg.base = MPI2_MEM_BASE;
        div = BSP_GetFlash2DIV();
    }
    else
        HAL_ASSERT(0);

    /* extend MPI1 space to 64Mb for 56x if psram supported */
    hwp_hpsys_cfg->SYSCR |= HPSYS_CFG_SYSCR_REMAP;

    qspi_cfg.SpiMode = type;
    qspi_cfg.msize = size;

    handle = &psram_hdl;
    handle->wakeup = 0;

    // force exit low power mode before init ?
    if (1)
    {
        // only controller instance and mode needed for ext lower power
        FLASH_HandleTypeDef tmp_handle;
        memset(&tmp_handle, 0, sizeof(tmp_handle));
        tmp_handle.Instance = qspi_cfg.Instance;
        HAL_MPI_EXIT_LOWP(&tmp_handle, type);
    }

    HAL_MPI_PSRAM_Init(handle, &qspi_cfg, div);
}

static int board_psram_config_get(uint8_t *mpi, uint8_t *type, uint8_t *size)
{
    int ret = -1;

    //TODO: read from flash OTP first
    BSP_System_Config();
    uint32_t sip1 = BSP_Get_Sip1_Mode();
    uint32_t sip2 = BSP_Get_Sip2_Mode();

    if (sip1 != 0)
    {
        ret = 0;
        *mpi = 1;
        switch (sip1)
        {
        case SFPIN_SIP1_APM_XCA64:
            *size = 8;
            *type = SPI_MODE_OPSRAM;
            break;
        case SFPIN_SIP1_APM_LEG32:
            *size = 4;
            *type = SPI_MODE_LEGPSRAM;
            break;
        case SFPIN_SIP1_WINB_HYP64:
        case SFPIN_SIP1_WINB_HYP32:
            *size = 4;
            *type = SPI_MODE_HBPSRAM;
            break;
        default:
            ret = -1;
        }
    }
    else if (sip2 != 0)
    {
        ret = 0;
        *mpi = 2;
        switch (sip2)
        {
        case SFPIN_SIP2_APM_XCA128:
            *size = 8;
            *type = SPI_MODE_OPSRAM;
            break;
        case SFPIN_SIP2_APM_LEG32:
            *size = 4;
            *type = SPI_MODE_LEGPSRAM;
            break;
        case SFPIN_SIP2_WINB_HYP128:
        case SFPIN_SIP2_WINB_HYP32:
            *size = 4;
            *type = SPI_MODE_HBPSRAM;
            break;
        default:
            ret = -1;
        }
    }
    else    // no valid sip info, use outsid configure
    {
#if defined(PSRAM_BL_MODE) && defined(PSRAM_BL_SIZE) && defined(PSRAM_BL_MPI)
        *mpi = PSRAM_BL_MPI;
        *size = PSRAM_BL_SIZE;
        ret = 0;
        {
#if PSRAM_BL_MODE == 3
            *type = SPI_MODE_OPSRAM;
#elif PSRAM_BL_MODE == 4
            *type = SPI_MODE_HPSRAM;
#elif PSRAM_BL_MODE == 5
            *type = SPI_MODE_LEGPSRAM;
#elif PSRAM_BL_MODE == 6
            *type = SPI_MODE_HBPSRAM;
#else
            ret = -1;
#endif
        }
#endif
    }
    if (ret >= 0)
        bootloader_switch_clock(*mpi);
    return ret;

}


void board_init_psram()
{
    uint8_t mpi, type, size;
    //__asm("B .");
    if (board_psram_config_get(&mpi, &type, &size) != 0)
        HAL_ASSERT(0);

    board_pinmux_psram(mpi, type);
    boot_psram_init(mpi, type, size);
}


