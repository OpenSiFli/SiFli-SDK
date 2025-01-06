/* Includes ------------------------------------------------------------------*/
#include <rtthread.h>
#include "bf0_hal.h"
#include "drv_io.h"


void HAL_MspInit(void)
{
}

void HAL_PostMspInit(void)
{
    HAL_RCC_DisableModule(RCC_MOD_PTC2);
    //HAL_RCC_DisableModule(RCC_MOD_WDT2);
    //HAL_RCC_DisableModule(RCC_MOD_BTIM4);
    HAL_RCC_DisableModule(RCC_MOD_BTIM3);
    HAL_RCC_DisableModule(RCC_MOD_SECU2);
}
