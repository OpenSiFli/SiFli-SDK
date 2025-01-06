/* Includes ------------------------------------------------------------------*/
#include <rtthread.h>
#include "bf0_hal.h"
#include "drv_io.h"
#include "drv_flash.h"


void HAL_PostMspInit(void)
{
    /*  Avoid IO Leakage */
    HAL_PIN_Set(PAD_PA00, GPIO_A0, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA01, GPIO_A1, PIN_PULLDOWN, 1);

    HAL_PIN_Set(PAD_PA04, GPIO_A4, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA03, GPIO_A3, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA05, GPIO_A5, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA06, GPIO_A6, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA07, GPIO_A7, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA08, GPIO_A8, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA02, GPIO_A2, PIN_PULLDOWN, 1);

    HAL_PIN_Set(PAD_PA09, GPIO_A9, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA10, GPIO_A10, PIN_PULLDOWN, 1);

#ifndef BSP_ENABLE_MPI2
    HAL_PIN_Set(PAD_PA16, GPIO_A16, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA12, GPIO_A12, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA15, GPIO_A15, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA13, GPIO_A13, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA14, GPIO_A14, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA17, GPIO_A17, PIN_PULLDOWN, 1);
#endif /* BSP_ENABLE_MPI2 */
    HAL_PIN_Set(PAD_PA20, GPIO_A20, PIN_PULLDOWN, 1);

    //HAL_PIN_Set(PAD_PA24, GPIO_A24, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA25, GPIO_A25, PIN_PULLDOWN, 1);

    HAL_PIN_Set(PAD_PA26, GPIO_A26, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA27, GPIO_A27, PIN_PULLDOWN, 1);

    HAL_PIN_Set(PAD_PA28, GPIO_A28, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA29, GPIO_A29, PIN_PULLDOWN, 1);

    HAL_PIN_Set(PAD_PA30, GPIO_A30, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA31, GPIO_A31, PIN_PULLDOWN, 1);

    HAL_PIN_Set(PAD_PA32, GPIO_A32, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA33, GPIO_A33, PIN_PULLDOWN, 1);

    //HAL_PIN_Set(PAD_PA35, GPIO_A35, PIN_PULLDOWN, 1);
    //HAL_PIN_Set(PAD_PA36, GPIO_A36, PIN_PULLDOWN, 1);

    HAL_PIN_Set(PAD_PA37, GPIO_A37, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA38, GPIO_A38, PIN_PULLDOWN, 1);

    HAL_PIN_Set(PAD_PA39, GPIO_A39, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA40, GPIO_A40, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA41, GPIO_A41, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA42, GPIO_A42, PIN_PULLDOWN, 1);

    HAL_PIN_Set(PAD_PA44, GPIO_A44, PIN_PULLDOWN, 1);

    /* pullup as default to allow sleep */
    //HAL_PIN_Set(PAD_PA30, GPIO_A30, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA24, GPIO_A24, PIN_PULLUP, 1);
    HAL_PBR_ConfigMode(0, false);

    HAL_PMU_ConfigPeriLdo(PMU_PERI_LDO3_3V3, false, true);

#ifndef BSP_USING_NOR_FLASH1
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
#endif /* !BSP_USING_NOR_FLASH1 */

    HAL_RCC_DisableModule(RCC_MOD_PTC1);
    HAL_RCC_DisableModule(RCC_MOD_I2C4);
    HAL_RCC_DisableModule(RCC_MOD_I2C3);
    HAL_RCC_DisableModule(RCC_MOD_I2C2);
    HAL_RCC_DisableModule(RCC_MOD_I2C1);
    HAL_RCC_DisableModule(RCC_MOD_PDM1);

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
    //HAL_RCC_DisableModule(RCC_MOD_I2S1);
    HAL_RCC_DisableModule(RCC_MOD_LCDC1);
    HAL_RCC_DisableModule(RCC_MOD_EPIC);
    HAL_RCC_DisableModule(RCC_MOD_EZIP);
    //HAL_RCC_DisableModule(RCC_MOD_USART3);
    HAL_RCC_DisableModule(RCC_MOD_USART2);

    HAL_RCC_DisableModule(RCC_MOD_USBC);
    HAL_RCC_DisableModule(RCC_MOD_SDMMC1);
    HAL_RCC_DisableModule(RCC_MOD_ATIM1);
    //HAL_RCC_DisableModule(RCC_MOD_AUDPRC);
    //HAL_RCC_DisableModule(RCC_MOD_AUDCODEC_HP);


    HAL_RCC_DisableModule(RCC_MOD_SECU1);
    //HAL_RCC_DisableModule(RCC_MOD_CRC1);
    //HAL_RCC_DisableModule(RCC_MOD_DMAC2);
    //HAL_RCC_DisableModule(RCC_MOD_CRC2);

#ifdef BSP_USING_NOR_FLASH1
    HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_FLASH1, RCC_CLK_SRC_SYS);
#elif defined(BSP_USING_NOR_FLASH2)
    HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_FLASH2, RCC_CLK_SRC_SYS);
#endif /* BSP_USING_NOR_FLASH1 */
    BSP_SetFlash2DIV(4);
    BSP_Flash_Init();

    HAL_RCC_HCPU_DisableDLL2();
}


HAL_RAM_RET_CODE_SECT(BSP_PowerDownCustom, void BSP_PowerDownCustom(int coreid, bool is_deep_sleep))
{
#ifdef SOC_BF0_HCPU
#ifdef BSP_USING_NOR_FLASH2
    HAL_PMU_ConfigPeriLdo(PMU_PERI_LDO2_3V3, false, true);

    HAL_PIN_Set(PAD_PA16, GPIO_A16, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA12, GPIO_A12, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA15, GPIO_A15, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA13, GPIO_A13, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA14, GPIO_A14, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA17, GPIO_A17, PIN_PULLDOWN, 1);

#elif defined(BSP_USING_NOR_FLASH1)
    FLASH_HandleTypeDef *flash_handle;
    flash_handle = (FLASH_HandleTypeDef *)rt_flash_get_handle_by_addr(MPI1_MEM_BASE);
    HAL_FLASH_DEEP_PWRDOWN(flash_handle);
    HAL_Delay_us(3);
#endif /* BSP_USING_NOR_FLASH2 */

    HAL_PIN_Set(PAD_PA35, GPIO_A35, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA36, GPIO_A36, PIN_PULLDOWN, 1);

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
#ifdef BSP_USING_NOR_FLASH2
        HAL_PIN_Set(PAD_PA16, MPI2_CLK,  PIN_NOPULL,   1);
        HAL_PIN_Set(PAD_PA12, MPI2_CS,   PIN_NOPULL,   1);
        HAL_PIN_Set(PAD_PA15, MPI2_DIO0, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_PA13, MPI2_DIO1, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_PA14, MPI2_DIO2, PIN_PULLUP,   1);
        HAL_PIN_Set(PAD_PA17, MPI2_DIO3, PIN_PULLUP, 1);

        HAL_PMU_ConfigPeriLdo(PMU_PERI_LDO2_3V3, true, true);


        BSP_Flash_hw2_init();
#elif defined(BSP_USING_NOR_FLASH1)
        FLASH_HandleTypeDef *flash_handle;
        flash_handle = (FLASH_HandleTypeDef *)rt_flash_get_handle_by_addr(MPI1_MEM_BASE);
        HAL_FLASH_RELEASE_DPD(flash_handle);
        HAL_Delay_us(8);
#endif /* BSP_USING_NOR_FLASH2 */

        HAL_PIN_Set(PAD_PA35, USART3_TXD, PIN_PULLUP, 1);
        HAL_PIN_Set(PAD_PA36, USART3_RXD, PIN_PULLUP, 1);
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

