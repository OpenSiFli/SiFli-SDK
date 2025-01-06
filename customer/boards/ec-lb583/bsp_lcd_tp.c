#include "bsp_board.h"


#define LCD_VCC_EN              (49)      // GPIO_B49
#define LCD_VIO_EN              (46)      // GPIO_B46
#define LCD_RESET_PIN           (5)       // GPIO_B5


#define TP_VCC_EN           (12)    // GPIO_A12
#define TP_VIO_EN           (13)    // GPIO_A13
#define TP_RESET            (15)    // GPIO_A15
#define TP_INT              (69)    // GPIO_A69


extern void BSP_GPIO_Set(int pin, int val, int is_porta);

void BSP_LCD_Reset(uint8_t high1_low0)
{
    BSP_GPIO_Set(LCD_RESET_PIN, high1_low0, 0);
}

void BSP_LCD_PowerDown(void)
{
#ifdef PMIC_CTRL_ENABLE
    pmic_device_control(PMIC_OUT_1V8_LVSW100_4, 0, 1); //LCD_1V8 power
    pmic_device_control(PMIC_OUT_LDO30_VOUT, 0, 1);    //LCD_3V3 power
#endif /* PMIC_CTRL_ENABLE */

    BSP_GPIO_Set(LCD_VCC_EN, 0, 0);
    BSP_GPIO_Set(LCD_VIO_EN, 0, 0);
    BSP_LCD_Reset(0);
}

void BSP_LCD_PowerUp(void)
{
#ifdef PMIC_CTRL_ENABLE
    pmic_device_control(PMIC_OUT_1V8_LVSW100_4, 1, 1); //LCD_1V8 power
    pmic_device_control(PMIC_OUT_LDO30_VOUT, 1, 1);    //LCD_3V3 power
#endif /* PMIC_CTRL_ENABLE */

    BSP_GPIO_Set(LCD_VCC_EN, 1, 0);
    BSP_GPIO_Set(LCD_VIO_EN, 1, 0);
}

void BSP_TP_PowerUp(void)
{
    BSP_GPIO_Set(TP_VCC_EN, 1, 1);
    BSP_GPIO_Set(TP_VIO_EN, 1, 1);
    BSP_GPIO_Set(TP_RESET,  1, 1);
}

void BSP_TP_PowerDown(void)
{
    BSP_GPIO_Set(TP_VCC_EN, 0, 1);
    BSP_GPIO_Set(TP_VIO_EN, 0, 1);
}


void BSP_TP_Reset(uint8_t high1_low0)
{
    BSP_GPIO_Set(TP_RESET, high1_low0, 1);
}

