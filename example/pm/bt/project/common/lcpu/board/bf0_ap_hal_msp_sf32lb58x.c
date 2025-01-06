/* Includes ------------------------------------------------------------------*/
#include <rtthread.h>
#include "bf0_hal.h"
#include "drv_io.h"

/**
* Initializes the Global MSP.
*/
void HAL_MspInit(void)
{
    BSP_IO_Init();
    /* remove this line if LCPU use default 48MHz configured in HAL_PreInit */
    if (!HAL_PMU_LXT_DISABLED())
        HAL_RCC_LCPU_SetDiv(2, 1, 3); //using 24M
}


void HAL_PostMspInit(void)
{
    HAL_RCC_DisableModule(RCC_MOD_BUSMON3);
    HAL_RCC_DisableModule(RCC_MOD_LCDC2);
    HAL_RCC_DisableModule(RCC_MOD_PTC2);
    HAL_RCC_DisableModule(RCC_MOD_TSEN);
    HAL_RCC_DisableModule(RCC_MOD_LPCOMP);
    HAL_RCC_DisableModule(RCC_MOD_SDADC);
    //HAL_RCC_DisableModule(RCC_MOD_GPADC);
    //HAL_RCC_DisableModule(RCC_MOD_WDT2);
    HAL_RCC_DisableModule(RCC_MOD_BTIM4);
    HAL_RCC_DisableModule(RCC_MOD_BTIM3);
    HAL_RCC_DisableModule(RCC_MOD_GPTIM5);
    HAL_RCC_DisableModule(RCC_MOD_GPTIM4);
    HAL_RCC_DisableModule(RCC_MOD_GPTIM3);
    HAL_RCC_DisableModule(RCC_MOD_I2C7);
    HAL_RCC_DisableModule(RCC_MOD_I2C6);
    HAL_RCC_DisableModule(RCC_MOD_I2C5);
    //HAL_RCC_DisableModule(RCC_MOD_I2S3);
    HAL_RCC_DisableModule(RCC_MOD_SPI4);
    HAL_RCC_DisableModule(RCC_MOD_SPI3);
    HAL_RCC_DisableModule(RCC_MOD_USART5);
    HAL_RCC_DisableModule(RCC_MOD_USART6);
    //HAL_RCC_DisableModule(RCC_MOD_PATCH);
    //HAL_RCC_DisableModule(RCC_MOD_MAILBOX2);
    //HAL_RCC_DisableModule(RCC_MOD_MAC);
    //HAL_RCC_DisableModule(RCC_MOD_PHY);
    //HAL_RCC_DisableModule(RCC_MOD_RFC);
    //HAL_RCC_DisableModule(RCC_MOD_MPI5);
    //HAL_RCC_DisableModule(RCC_MOD_GPIO2);
    HAL_RCC_DisableModule(RCC_MOD_NNACC2);
    //HAL_RCC_DisableModule(RCC_MOD_AUDCODEC_LP);
    HAL_RCC_DisableModule(RCC_MOD_FFT2);
    HAL_RCC_DisableModule(RCC_MOD_FACC2);


}

