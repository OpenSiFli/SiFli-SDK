#include "bsp_board.h"


#ifdef BSP_LCDC_USING_DPI
    #define LCD_RESET_PIN           (54)         // GPIO_A54

#else
    #define LCD_VCC_EN              (47)        // GPIO_A47
    //#define LCD_VIO_EN                        // NO pin
    #define LCD_RESET_PIN           (43)         // GPIO_A43
#endif /* BSP_LCDC_USING_DPI */


//#define TP_VCC_EN           (12)          // NO pin
//#define TP_VIO_EN           (13)          // NO pin
#define TP_RESET            (18)            // GPIO_B18
#define TP_INT              (50)            // GPIO_A50


extern void BSP_GPIO_Set(int pin, int val, int is_porta);

void BSP_LCD_Reset(uint8_t high1_low0)
{
    BSP_GPIO_Set(LCD_RESET_PIN, high1_low0, 1);
}

void BSP_LCD_PowerDown(void)
{
#ifdef PMIC_CTRL_ENABLE
    pmic_device_control(PMIC_OUT_1V8_LVSW100_4, 0, 1); //LCD_1V8 power
    pmic_device_control(PMIC_OUT_LDO30_VOUT, 0, 1);    //LCD_3V3 power
#endif /* PMIC_CTRL_ENABLE */

#ifdef LCD_VCC_EN
    BSP_GPIO_Set(LCD_VCC_EN, 0, 1);
#endif /* LCD_VCC_EN */
#ifdef LCD_VIO_EN
    BSP_GPIO_Set(LCD_VIO_EN, 0, 0);
#endif
    BSP_LCD_Reset(0);
}

void BSP_LCD_PowerUp(void)
{

#ifdef BSP_LCDC_USING_DPI
    HAL_PIN_Set(PAD_PA13, LCDC1_DPI_R1, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA14, LCDC1_DPI_R0, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA16, LCDC1_DPI_R2, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA19, LCDC1_DPI_R4, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA24, LCDC1_DPI_R3, PIN_NOPULL, 1);

    HAL_PIN_Set(PAD_PA28, LCDC1_DPI_G0, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA29, LCDC1_DPI_G5, PIN_NOPULL, 1);
    //HAL_PIN_Set(PAD_PA30, LCDC1_DPI_G1, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA32, LCDC1_DPI_G2, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA33, LCDC1_DPI_G3, PIN_NOPULL, 1);
    //HAL_PIN_Set(PAD_PA34, LCDC1_DPI_G4, PIN_NOPULL, 1);

    HAL_PIN_Set(PAD_PA36, LCDC1_DPI_B0, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA37, LCDC1_DPI_B1, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA38, LCDC1_DPI_B2, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA41, LCDC1_DPI_B4, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA43, LCDC1_DPI_B3, PIN_NOPULL, 1);

    HAL_PIN_Set(PAD_PA42, LCDC1_DPI_VSYNC, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA44, LCDC1_DPI_HSYNC, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA45, LCDC1_DPI_CLK, PIN_NOPULL, 1);

    HAL_PIN_Set(PAD_PA47, LCDC1_DPI_DE, PIN_NOPULL, 1);


    HAL_PIN_Set(PAD_PA48, I2C1_SCL, PIN_PULLUP, 1);  //tp sclk
    HAL_PIN_Set(PAD_PA49, I2C1_SDA, PIN_PULLUP, 1);  //tp sda
    HAL_PIN_Set(PAD_PA46, GPIO_A46, PIN_NOPULL, 1);  //tp reset
    HAL_PIN_Set(PAD_PA54, GPIO_A54, PIN_NOPULL, 1);  //lcd reset
    HAL_PIN_Set(PAD_PA50, GPIO_A50, PIN_PULLUP, 1);  // tp INT,
#endif /* BSP_LCDC_USING_DPI */



#ifdef PMIC_CTRL_ENABLE
    pmic_device_control(PMIC_OUT_1V8_LVSW100_4, 1, 1); //LCD_1V8 power
    pmic_device_control(PMIC_OUT_LDO30_VOUT, 1, 1);    //LCD_3V3 power
#endif /* PMIC_CTRL_ENABLE */

#ifdef LCD_VCC_EN
    BSP_GPIO_Set(LCD_VCC_EN, 1, 1);
#endif /* LCD_VCC_EN */
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

