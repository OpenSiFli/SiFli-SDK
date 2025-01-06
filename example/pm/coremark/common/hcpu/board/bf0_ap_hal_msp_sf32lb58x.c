/* Includes ------------------------------------------------------------------*/
#include <rtthread.h>
#include "bf0_hal.h"
#include "drv_io.h"


HAL_RAM_RET_CODE_SECT(HAL_PostMspInit, void HAL_PostMspInit(void))
{
    HAL_StatusTypeDef status;

    if (PM_COLD_BOOT == SystemPowerOnModeGet())
    {
        status = HAL_RCC_CalibrateRC48();
        RT_ASSERT(HAL_OK == status);
    }
    HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_HP_PERI, RCC_CLK_PERI_HRC48);

    HAL_RCC_DisableModule(RCC_MOD_PTC1);
    HAL_RCC_DisableModule(RCC_MOD_DSIPHY);
    HAL_RCC_DisableModule(RCC_MOD_DSIHOST);
    HAL_RCC_DisableModule(RCC_MOD_I2C4);
    HAL_RCC_DisableModule(RCC_MOD_I2C3);
    HAL_RCC_DisableModule(RCC_MOD_I2C2);
    HAL_RCC_DisableModule(RCC_MOD_I2C1);
    HAL_RCC_DisableModule(RCC_MOD_PDM2);
    HAL_RCC_DisableModule(RCC_MOD_PDM1);
    HAL_RCC_DisableModule(RCC_MOD_NNACC1);

    HAL_RCC_DisableModule(RCC_MOD_SPI2);
    HAL_RCC_DisableModule(RCC_MOD_SPI1);
    //HAL_RCC_DisableModule(RCC_MOD_WDT1);

    HAL_RCC_DisableModule(RCC_MOD_BTIM2);
    HAL_RCC_DisableModule(RCC_MOD_BTIM1);
    HAL_RCC_DisableModule(RCC_MOD_GPTIM2);
    HAL_RCC_DisableModule(RCC_MOD_GPTIM1);
    HAL_RCC_DisableModule(RCC_MOD_TRNG);
    HAL_RCC_DisableModule(RCC_MOD_AES);
    HAL_RCC_DisableModule(RCC_MOD_EFUSEC);
    HAL_RCC_DisableModule(RCC_MOD_I2S2);
    HAL_RCC_DisableModule(RCC_MOD_I2S1);
    HAL_RCC_DisableModule(RCC_MOD_LCDC1);
    HAL_RCC_DisableModule(RCC_MOD_EPIC);
    HAL_RCC_DisableModule(RCC_MOD_EZIP);
    HAL_RCC_DisableModule(RCC_MOD_USART3);
    HAL_RCC_DisableModule(RCC_MOD_USART2);

    HAL_RCC_DisableModule(RCC_MOD_BUSMON2);
    HAL_RCC_DisableModule(RCC_MOD_BUSMON1);
    HAL_RCC_DisableModule(RCC_MOD_USBC);
    HAL_RCC_DisableModule(RCC_MOD_SDMMC1);
    HAL_RCC_DisableModule(RCC_MOD_SDMMC2);
    HAL_RCC_DisableModule(RCC_MOD_MPI3);
    //HAL_RCC_DisableModule(RCC_MOD_MPI4);
    HAL_RCC_DisableModule(RCC_MOD_GPIO1);
    HAL_RCC_DisableModule(RCC_MOD_MAILBOX1);
    HAL_RCC_DisableModule(RCC_MOD_ATIM1);
    HAL_RCC_DisableModule(RCC_MOD_ATIM2);
    HAL_RCC_DisableModule(RCC_MOD_AUDPRC);
    HAL_RCC_DisableModule(RCC_MOD_AUDCODEC_HP);
    HAL_RCC_DisableModule(RCC_MOD_FFT1);
    HAL_RCC_DisableModule(RCC_MOD_FACC1);
    HAL_RCC_DisableModule(RCC_MOD_CAN1);
    HAL_RCC_DisableModule(RCC_MOD_CAN2);
    HAL_RCC_DisableModule(RCC_MOD_SCI);
    HAL_RCC_DisableModule(RCC_MOD_GPU);
    HAL_RCC_DisableModule(RCC_MOD_JENC);
    HAL_RCC_DisableModule(RCC_MOD_JDEC);
    HAL_RCC_DisableModule(RCC_MOD_CRC1);
    HAL_RCC_DisableModule(RCC_MOD_DMAC2);
    //HAL_RCC_DisableModule(RCC_MOD_CRC2);
}
