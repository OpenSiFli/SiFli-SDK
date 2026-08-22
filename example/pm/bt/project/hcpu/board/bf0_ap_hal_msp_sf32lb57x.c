/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <rtthread.h>
#include "bf0_hal.h"
#include "drv_io.h"
#include "drv_flash.h"


void HAL_PostMspInit(void)
{
    HAL_PIN_Set(PAD_PA24, GPIO_A24, PIN_PULLUP, 1);

#ifndef BSP_USING_PSRAM1
    /* not enable pull down as SIP flash is used */
    HAL_PMU_ConfigPeriLdo(PMU_PERI_LDO_1V8, false, true);
    HAL_PIN_Set_Analog(PAD_SA00, 1);
    HAL_PIN_Set_Analog(PAD_SA01, 1);
    HAL_PIN_Set_Analog(PAD_SA02, 1);
    HAL_PIN_Set_Analog(PAD_SA03, 1);
    HAL_PIN_Set_Analog(PAD_SA04, 1);
    HAL_PIN_Set_Analog(PAD_SA05, 1);
    HAL_PIN_Set_Analog(PAD_SA06, 1);
    HAL_PIN_Set_Analog(PAD_SA07, 1);
    HAL_PIN_Set_Analog(PAD_SA08, 1);
    HAL_PIN_Set_Analog(PAD_SA09, 1);
    HAL_PIN_Set_Analog(PAD_SA10, 1);
    HAL_PIN_Set_Analog(PAD_SA11, 1);
    HAL_PIN_Set_Analog(PAD_SA12, 1);
#endif /* !BSP_USING_PSRAM1 */

#ifndef BSP_USING_PSRAM2
    /* not enable pull down as SIP flash is used */
    HAL_PMU_ConfigPeriLdo(PMU_PERI_LDO_1V8, false, true);
    HAL_PIN_Set_Analog(PAD_SB00, 1);
    HAL_PIN_Set_Analog(PAD_SB01, 1);
    HAL_PIN_Set_Analog(PAD_SB02, 1);
    HAL_PIN_Set_Analog(PAD_SB03, 1);
    HAL_PIN_Set_Analog(PAD_SB04, 1);
    HAL_PIN_Set_Analog(PAD_SB05, 1);
    HAL_PIN_Set_Analog(PAD_SB06, 1);
    HAL_PIN_Set_Analog(PAD_SB07, 1);
    HAL_PIN_Set_Analog(PAD_SB08, 1);
    HAL_PIN_Set_Analog(PAD_SB09, 1);
    HAL_PIN_Set_Analog(PAD_SB10, 1);
    HAL_PIN_Set_Analog(PAD_SB11, 1);
    HAL_PIN_Set_Analog(PAD_SB12, 1);
#endif /* !BSP_USING_PSRAM2 */


}


HAL_RAM_RET_CODE_SECT(BSP_PowerDownCustom, void BSP_PowerDownCustom(int coreid, bool is_deep_sleep))
{
#ifdef SOC_BF0_HCPU

#ifdef BSP_USING_NOR_FLASH3
    FLASH_HandleTypeDef *flash_handle;
    flash_handle = (FLASH_HandleTypeDef *)rt_flash_get_handle_by_addr(MPI3_MEM_BASE);
    HAL_FLASH_DEEP_PWRDOWN(flash_handle);
    HAL_Delay_us(3);
#endif /* BSP_USING_NOR_FLASH3 */


#else
    {
        ;
    }
#endif
}


HAL_RAM_RET_CODE_SECT(BSP_PowerUpCustom, void BSP_PowerUpCustom(bool is_deep_sleep))
{
#ifdef SOC_BF0_HCPU
    if (!is_deep_sleep)
    {
#ifdef BSP_USING_NOR_FLASH3
        FLASH_HandleTypeDef *flash_handle;
        flash_handle = (FLASH_HandleTypeDef *)rt_flash_get_handle_by_addr(MPI3_MEM_BASE);
        HAL_FLASH_RELEASE_DPD(flash_handle);
        HAL_Delay_us(80);
#endif /* BSP_USING_NOR_FLASH3 */
    }
    else if (PM_STANDBY_BOOT == SystemPowerOnModeGet())
    {
    }
#elif defined(SOC_BF0_LCPU)
    {
        ;
    }
#endif
}

