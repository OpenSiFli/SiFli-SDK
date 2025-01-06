#include "bsp_board.h"

// NO SD
// #define SD1_RESET_PIN       (11)
// #define SD1_EN_PIN          (2)

// NO MPI Power pins
// #define MPI1_POWER_PIN  (33)
// #define MPI2_POWER_PIN  (26)
// #define MPI3_POWER_PIN  (43)

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

        if (PM_STANDBY_BOOT == SystemPowerOnModeGet())
        {
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
    //int i;
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
#else /* PMIC_CTRL_ENABLE */

#define WR_REG(a, b)      ( *((uint32_t *)(a)) = (b))
#define RD_REG(a)         (*((uint32_t *)(a)))

#define EUROPA_PMIC_REG_BASE        (0x50070000)
#define EUROPA_PMIC_LDO33_VOUT          (0x20)
#define EUROPA_PMIC_1V8_LVSW100_5       (0x30)

    // enable emmc 3v3
    //WR_REG((EUROPA_PMIC_REG_BASE+EUROPA_PMIC_LDO33_VOUT), (0xd<<3)|1);
    uint32_t data = RD_REG(EUROPA_PMIC_REG_BASE + EUROPA_PMIC_LDO33_VOUT);
    data |= 1;
    WR_REG((EUROPA_PMIC_REG_BASE + EUROPA_PMIC_LDO33_VOUT), data);

    // enable emmc18, LVSW100_5
    data = RD_REG(EUROPA_PMIC_REG_BASE + EUROPA_PMIC_1V8_LVSW100_5);
    //data |= (1 << 5);
    data |= 0xff;
    WR_REG((EUROPA_PMIC_REG_BASE + EUROPA_PMIC_1V8_LVSW100_5), data);
#endif  /* PMIC_CTRL_ENABLE */

#ifdef SD1_RESET_PIN
    BSP_GPIO_Set(SD1_EN_PIN, 1, 1);
#endif
#ifdef SD1_RESET_PIN
    BSP_GPIO_Set(SD1_RESET_PIN, 1, 1);
#endif
}

void BSP_SD_PowerDown(void)
{
#ifdef SD1_RESET_PIN
    BSP_GPIO_Set(SD1_EN_PIN, 0, 1);
#endif
#ifdef SD1_RESET_PIN
    BSP_GPIO_Set(SD1_RESET_PIN, 0, 1);
#endif
}


