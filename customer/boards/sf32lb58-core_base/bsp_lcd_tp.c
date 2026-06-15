#include "bsp_board.h"



#define LCD_RESET_PIN           (18)       // GPIO_A18



#define TP_RESET            (16)    // GPIO_A16
#define TP_INT              (17)    // GPIO_A17


extern void BSP_GPIO_Set(int pin, int val, int is_porta);

void BSP_LCD_Reset(uint8_t high1_low0)
{
    BSP_GPIO_Set(LCD_RESET_PIN, high1_low0, 1);
}

void BSP_LCD_PowerDown(void)
{

    BSP_LCD_Reset(0);
}

void BSP_LCD_PowerUp(void)
{
    BSP_LCD_Reset(1);
}

void BSP_TP_PowerUp(void)
{
    BSP_GPIO_Set(TP_RESET,  1, 1);
    HAL_PIN_Set(PAD_PA17, GPIO_A17, PIN_NOPULL, 1);//TP irq
}

void BSP_TP_PowerDown(void)
{
    HAL_PIN_Set(PAD_PA17, GPIO_A17, PIN_PULLDOWN, 1);//TP irq  PULL DOWN
}


void BSP_TP_Reset(uint8_t high1_low0)
{
    BSP_GPIO_Set(TP_RESET, high1_low0, 1);
}

