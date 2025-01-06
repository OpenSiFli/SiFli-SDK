/* Includes ------------------------------------------------------------------*/
#include <rtthread.h>
#include "bf0_hal.h"
#include "drv_io.h"

void HAL_PostMspInit(void)
{

    HAL_RCC_DisableModule(RCC_MOD_PTC2);
    //HAL_RCC_DisableModule(RCC_MOD_WDT2);
    HAL_RCC_DisableModule(RCC_MOD_BTIM4);
    HAL_RCC_DisableModule(RCC_MOD_BTIM3);
    HAL_RCC_DisableModule(RCC_MOD_PATCH);
    HAL_RCC_DisableModule(RCC_MOD_MAC);
    HAL_RCC_DisableModule(RCC_MOD_PHY);
    HAL_RCC_DisableModule(RCC_MOD_RFC);
    HAL_RCC_DisableModule(RCC_MOD_GPIO2);
    HAL_RCC_DisableModule(RCC_MOD_SECU2);

    HAL_RCC_LCPU_ClockSelect(RCC_CLK_MOD_LP_PERI, RCC_CLK_PERI_HRC48);
    HAL_RCC_LCPU_ClockSelect(RCC_CLK_MOD_SYS, RCC_SYSCLK_HRC48);

    HAL_LPAON_DisableXT48();

    /* Disable BLE HXT48 request */
    hwp_lpsys_aon->SLP_CFG |= LPSYS_AON_SLP_CFG_XTAL_FORCE_OFF;
}

