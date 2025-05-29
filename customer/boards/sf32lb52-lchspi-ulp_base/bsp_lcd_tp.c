#include "bsp_board.h"


#ifdef BSP_USING_LCD
#define LCD_RESET_PIN           (0)         // GPIO_A00
#define TP_RESET                (9)         // GPIO_A09

#ifdef LCD_USING_CO5300
    #define LCD_VADD_EN             (1)
    #define PA_38                   (38)
    #define PA_26                   (26)

#endif



/***************************LCD ***********************************/
extern void BSP_PIN_LCD(void);
void BSP_LCD_Reset(uint8_t high1_low0)
{
    BSP_GPIO_Set(LCD_RESET_PIN, high1_low0, 1);
}

void BSP_LCD_PowerDown(void)
{
    // TODO: LCD power down

#ifdef LCD_USING_CO5300
    BSP_GPIO_Set(LCD_RESET_PIN, 0, 1);//reset
    BSP_GPIO_Set(LCD_VADD_EN, 0, 1); //
    BSP_GPIO_Set(38, 0, 1); //vsys_1
    BSP_GPIO_Set(26, 0, 1); //3v3

    BSP_GPIO_Set(42, 0, 1);  //Audio

    HAL_PIN_Set(PAD_PA00, GPIO_A0, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA01, GPIO_A1, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA02, GPIO_A2, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA03, GPIO_A3, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA04, GPIO_A4, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA05, GPIO_A5, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA06, GPIO_A6, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA07, GPIO_A7, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA08, GPIO_A8, PIN_PULLDOWN, 1);

    HAL_PIN_Set(PAD_PA42, GPIO_A42, PIN_PULLDOWN, 1);
#endif
}

void BSP_LCD_PowerUp(void)
{
    // TODO: LCD power up
    HAL_Delay_us(500);      // lcd power on finish ,need 500us
    BSP_PIN_LCD();
#ifdef LCD_USING_CO5300
    BSP_GPIO_Set(LCD_RESET_PIN, 1, 1);
    BSP_GPIO_Set(LCD_VADD_EN, 1, 1); //POwer up VADD EN
    BSP_GPIO_Set(38, 1, 1);
    BSP_GPIO_Set(26, 1, 1);

    HAL_PIN_Set(PAD_PA00, GPIO_A0, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA01, GPIO_A1, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA38, GPIO_A38, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA26, GPIO_A26, PIN_NOPULL, 1);

    //Audio
    HAL_PIN_Set(PAD_PA42, GPIO_A42, PIN_PULLUP, 1);
    BSP_GPIO_Set(42, 1, 1);  //Audio Power

#endif
}


/***************************Touch ***********************************/
extern void BSP_PIN_Touch(void);
void BSP_TP_PowerUp(void)
{
    // TODO: Setup TP power up pin
    BSP_PIN_Touch();
    BSP_GPIO_Set(TP_RESET,  1, 1);
}

void BSP_TP_PowerDown(void)
{
    // TODO: Setup TP power down pin
    BSP_GPIO_Set(TP_RESET,  0, 1);
    HAL_PIN_Set(PAD_PA09, GPIO_A9, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA41, GPIO_A41, PIN_PULLDOWN, 1);    // CTP_INT
    HAL_PIN_Set(PAD_PA33, GPIO_A33, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA37, GPIO_A37, PIN_PULLDOWN, 1);


}
void BSP_TP_Reset(uint8_t high1_low0)
{
    BSP_GPIO_Set(TP_RESET, high1_low0, 1);
}

#endif
