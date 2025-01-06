#include "bsp_board.h"


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

#define MPI2_POWER_PIN  (11)

__WEAK void BSP_PowerDownCustom(int coreid, bool is_deep_sleep)
{
    BSP_GPIO_Set(MPI2_POWER_PIN, 0, 1);
}

__WEAK void BSP_PowerUpCustom(bool is_deep_sleep)
{
    BSP_GPIO_Set(MPI2_POWER_PIN, 1, 1);
}


void BSP_Power_Up(bool is_deep_sleep)
{
    BSP_PowerUpCustom(is_deep_sleep);
}



void BSP_IO_Power_Down(int coreid, bool is_deep_sleep)
{
    BSP_PowerDownCustom(coreid, is_deep_sleep);
}

void BSP_SDIO_Power_Up(void)
{
#ifdef RT_USING_SDIO
    // TODO: Add SDIO power up

#endif

}
void BSP_SDIO_Power_Down(void)
{
#ifdef RT_USING_SDIO
    // TODO: Add SDIO power down
#endif
}


