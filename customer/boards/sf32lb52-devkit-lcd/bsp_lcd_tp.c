#include "bsp_board.h"


#ifdef BSP_USING_LCD
#define LCD_RESET_PIN           (0)         // GPIO_A00
#define TP_RESET                (9)         // GPIO_A09


/***************************LCD ***********************************/
extern void BSP_PIN_LCD(void);
void BSP_LCD_Reset(uint8_t high1_low0)
{
    BSP_GPIO_Set(LCD_RESET_PIN, high1_low0, 1);
}

void BSP_LCD_PowerDown(void)
{
    // TODO: LCD power down
    BSP_GPIO_Set(LCD_RESET_PIN, 0, 1);
}

void BSP_LCD_PowerUp(void)
{
    // TODO: LCD power up
    HAL_Delay_us(500);      // lcd power on finish ,need 500us
    BSP_PIN_LCD();
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
}
void BSP_TP_Reset(uint8_t high1_low0)
{
    BSP_GPIO_Set(TP_RESET, high1_low0, 1);
}

#endif
