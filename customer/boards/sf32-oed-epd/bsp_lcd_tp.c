#include "bsp_board.h"


#ifdef BSP_USING_LCD





/***************************LCD ***********************************/
extern void BSP_PIN_LCD(void);
void BSP_LCD_Reset(uint8_t high1_low0)
{

}

void BSP_LCD_PowerDown(void)
{

}

void BSP_LCD_PowerUp(void)
{
    BSP_PIN_LCD();
}


/***************************Touch ***********************************/
extern void BSP_PIN_Touch(void);
void BSP_TP_PowerUp(void)
{
    BSP_PIN_Touch();
    BSP_GPIO_Set(TOUCH_RESET_PIN,  1, 1);
}

void BSP_TP_PowerDown(void)
{
//     // TODO: Setup TP power down pin
    BSP_GPIO_Set(TOUCH_RESET_PIN,  0, 1);
}
void BSP_TP_Reset(uint8_t high1_low0)
{
    BSP_GPIO_Set(TOUCH_RESET_PIN, high1_low0, 1);
}

#endif
