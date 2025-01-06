/* Includes ------------------------------------------------------------------*/
#include <rtthread.h>
#include "bf0_hal.h"
#include "drv_io.h"

/**
* Initializes the Global MSP.
*/
void HAL_MspInit(void)
{
    //HAL_PATCH_install();
#ifndef SOC_SF32LB52X
    BSP_IO_Init();
#endif
}


#ifdef SF32LB56X
void HAL_PostMspInit(void)
{
    HAL_RCC_DisableModule(RCC_MOD_BUSMON2);
    HAL_RCC_DisableModule(RCC_MOD_PTC2);
    HAL_RCC_DisableModule(RCC_MOD_TSEN);
    HAL_RCC_DisableModule(RCC_MOD_LPCOMP);
    HAL_RCC_DisableModule(RCC_MOD_BTIM4);
    HAL_RCC_DisableModule(RCC_MOD_BTIM3);
    HAL_RCC_DisableModule(RCC_MOD_GPTIM5);
    HAL_RCC_DisableModule(RCC_MOD_GPTIM4);
    //HAL_RCC_DisableModule(RCC_MOD_GPTIM3);
    HAL_RCC_DisableModule(RCC_MOD_I2C7);
    HAL_RCC_DisableModule(RCC_MOD_I2C6);
    HAL_RCC_DisableModule(RCC_MOD_I2C5);
    HAL_RCC_DisableModule(RCC_MOD_SPI4);
    HAL_RCC_DisableModule(RCC_MOD_SPI3);
    HAL_RCC_DisableModule(RCC_MOD_USART5);
    HAL_RCC_DisableModule(RCC_MOD_USART6);
}
#endif
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
