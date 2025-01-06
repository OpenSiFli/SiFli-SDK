#include "bsp_board.h"


#define LCD_VCC_EN              (25)        // GPIO_B25
//#define LCD_VIO_EN                        // NO pin
#ifdef BSP_USING_BOARD_EC_SS6700XXX
    #define LCD_RESET_PIN           (4)         // GPIO_B4
#else
    #define LCD_RESET_PIN           (5)         // GPIO_A5
#endif

//#define TP_VCC_EN           (12)          // NO pin
//#define TP_VIO_EN           (13)          // NO pin
#define TP_RESET            (18)            // GPIO_B18
#define TP_INT              (50)            // GPIO_A50


extern void BSP_GPIO_Set(int pin, int val, int is_porta);

void BSP_LCD_Reset(uint8_t high1_low0)
{
#ifdef BSP_USING_BOARD_EC_SS6700XXX
    BSP_GPIO_Set(LCD_RESET_PIN, high1_low0, 0);
#else
    BSP_GPIO_Set(LCD_RESET_PIN, high1_low0, 1);
#endif
}

void BSP_LCD_PowerDown(void)
{
#ifdef PMIC_CTRL_ENABLE
    pmic_device_control(PMIC_OUT_1V8_LVSW100_4, 0, 1); //LCD_1V8 power
    pmic_device_control(PMIC_OUT_LDO30_VOUT, 0, 1);    //LCD_3V3 power
#endif /* PMIC_CTRL_ENABLE */

    BSP_GPIO_Set(LCD_VCC_EN, 0, 0);
#ifdef LCD_VIO_EN
    BSP_GPIO_Set(LCD_VIO_EN, 0, 0);
#endif
    BSP_LCD_Reset(0);
}

void BSP_LCD_PowerUp(void)
{
#ifdef PMIC_CTRL_ENABLE
    pmic_device_control(PMIC_OUT_1V8_LVSW100_4, 1, 1); //LCD_1V8 power
    pmic_device_control(PMIC_OUT_LDO30_VOUT, 1, 1);    //LCD_3V3 power
#endif /* PMIC_CTRL_ENABLE */

    BSP_GPIO_Set(LCD_VCC_EN, 1, 0);
#ifdef LCD_VIO_EN
    BSP_GPIO_Set(LCD_VIO_EN, 1, 0);
#endif
}

void BSP_TP_PowerUp(void)
{
#ifdef TP_VCC_EN
    BSP_GPIO_Set(TP_VCC_EN, 1, 1);
#endif
#ifdef TP_VIO_EN
    BSP_GPIO_Set(TP_VIO_EN, 1, 1);
#endif
    BSP_GPIO_Set(TP_RESET,  1, 0);
}

void BSP_TP_PowerDown(void)
{
#ifdef TP_VCC_EN
    BSP_GPIO_Set(TP_VCC_EN, 0, 1);
#endif
#ifdef TP_VIO_EN
    BSP_GPIO_Set(TP_VIO_EN, 0, 1);
#endif
    BSP_GPIO_Set(TP_RESET,  0, 0);
}

void BSP_TP_Reset(uint8_t high1_low0)
{
    BSP_GPIO_Set(TP_RESET, high1_low0, 0);
}

