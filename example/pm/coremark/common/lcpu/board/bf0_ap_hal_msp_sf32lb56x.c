/* Includes ------------------------------------------------------------------*/
#include <rtthread.h>
#include "bf0_hal.h"
#include "drv_io.h"


void HAL_PostMspInit(void)
{
    HAL_RCC_DisableModule(RCC_MOD_BUSMON2);
    HAL_RCC_DisableModule(RCC_MOD_PTC2);
    HAL_RCC_DisableModule(RCC_MOD_TSEN);
    HAL_RCC_DisableModule(RCC_MOD_LPCOMP);
    HAL_RCC_DisableModule(RCC_MOD_GPADC);
    //HAL_RCC_DisableModule(RCC_MOD_WDT2);
    HAL_RCC_DisableModule(RCC_MOD_BTIM4);
    HAL_RCC_DisableModule(RCC_MOD_BTIM3);
    HAL_RCC_DisableModule(RCC_MOD_GPTIM5);
    HAL_RCC_DisableModule(RCC_MOD_GPTIM4);
    HAL_RCC_DisableModule(RCC_MOD_GPTIM3);
    HAL_RCC_DisableModule(RCC_MOD_I2C7);
    HAL_RCC_DisableModule(RCC_MOD_I2C6);
    HAL_RCC_DisableModule(RCC_MOD_I2C5);
    HAL_RCC_DisableModule(RCC_MOD_SPI4);
    HAL_RCC_DisableModule(RCC_MOD_SPI3);
    HAL_RCC_DisableModule(RCC_MOD_USART5);
    HAL_RCC_DisableModule(RCC_MOD_USART6);
    HAL_RCC_DisableModule(RCC_MOD_PATCH);
    HAL_RCC_DisableModule(RCC_MOD_MAILBOX2);
    HAL_RCC_DisableModule(RCC_MOD_MAC);
    HAL_RCC_DisableModule(RCC_MOD_PHY);
    HAL_RCC_DisableModule(RCC_MOD_RFC);
    HAL_RCC_DisableModule(RCC_MOD_GPIO2);
    HAL_RCC_DisableModule(RCC_MOD_AUDCODEC_LP);

    HAL_RCC_LCPU_ClockSelect(RCC_CLK_MOD_LP_PERI, RCC_CLK_PERI_HRC48);
    HAL_RCC_LCPU_ClockSelect(RCC_CLK_MOD_SYS, RCC_SYSCLK_HRC48);
    HAL_LPAON_DisableXT48();

    /* Disable BLE HXT48 request */
    hwp_lpsys_aon->SLP_CFG |= LPSYS_AON_SLP_CFG_XTAL_FORCE_OFF;
}

