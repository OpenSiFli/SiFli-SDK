#include "bsp_board.h"

#define SD1_RESET_PIN       (11)
#define SD1_EN_PIN          (2)

#define MPI1_POWER_PIN  (33)
#define MPI2_POWER_PIN  (26)
#define MPI3_POWER_PIN  (43)

void BSP_GPIO_Set(int pin, int val, int is_porta)
{
    GPIO_TypeDef *gpio = (is_porta) ? hwp_gpio1 : hwp_gpio2;
    GPIO_InitTypeDef GPIO_InitStruct;

    // set sensor pin to output mode
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Pin = pin;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(gpio, &GPIO_InitStruct);

    // set sensor pin to high == power on sensor board
    HAL_GPIO_WritePin(gpio, pin, (GPIO_PinState)val);
}


void BSP_Power_Up(bool is_deep_sleep)
{
#ifdef SOC_BF0_HCPU

    if (is_deep_sleep)
    {
        pm_power_on_mode_t pmode = SystemPowerOnModeGet();
        if ((PM_STANDBY_BOOT == pmode) || (PM_COLD_BOOT == pmode))
        {
            BSP_GPIO_Set(MPI1_POWER_PIN, 1, 1);
            BSP_GPIO_Set(MPI2_POWER_PIN, 1, 1);
            BSP_GPIO_Set(MPI3_POWER_PIN, 1, 1);
            /* no need to wait for stable as PAD has not been enabled yet, i.e. power is always present */
            if (PM_COLD_BOOT == pmode)
                HAL_Delay_us(2000);
        }

        // Replace with API that is OS-independent.
        //rt_psram_exit_low_power("psram0");
    }
#elif defined(SOC_BF0_LCPU)
    {
        ;
    }
#endif

}

void BSP_IO_Power_Down(int coreid, bool is_deep_sleep)
{
    int i;
#ifdef SOC_BF0_HCPU
    if (coreid == CORE_ID_HCPU)
    {
        // Replace with API that is OS-independent.
        // if (is_deep_sleep)
        //rt_psram_enter_low_power("psram0");
    }
#else
    {
        ;
    }
#endif
}

void BSP_SD_PowerUp(void)
{
#ifdef PMIC_CTRL_ENABLE
    BSP_PMIC_Control(PMIC_OUT_1V8_LVSW100_5, 1, 1); //LCD_1V8 power
    BSP_PMIC_Control(PMIC_OUT_LDO33_VOUT, 1, 1);    //LCD_3V3 power
#endif /* PMIC_CTRL_ENABLE */

    BSP_GPIO_Set(SD1_EN_PIN, 1, 1);
    BSP_GPIO_Set(SD1_RESET_PIN, 1, 1);
}

void BSP_SD_PowerDown(void)
{
    BSP_GPIO_Set(SD1_EN_PIN, 0, 1);
    BSP_GPIO_Set(SD1_RESET_PIN, 0, 1);
}


