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
    //HAL_PMU_ConfigPeriLdo(PMUC_PERI_LDO_EN_VDD33_LDO3_Pos, 1, 1);
    HAL_PIN_Set(PAD_PA15, SD1_CMD, PIN_PULLUP, 1);
    HAL_Delay_us(20);   // add a delay before clock setting to avoid wrong cmd happen

    HAL_PIN_Set(PAD_PA14, SD1_CLK,  PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA16, SD1_DIO0, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA17, SD1_DIO1, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA12, SD1_DIO2, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA13, SD1_DIO3, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA11, GPIO_A11, PIN_PULLUP, 1);
    HAL_Delay_us(20);
    BSP_GPIO_Set(11, 1, 1);
#endif

}
void BSP_SDIO_Power_Down(void)
{
#ifdef RT_USING_SDIO

    HAL_PIN_Set(PAD_PA15, GPIO_A15, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA14, GPIO_A14, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA16, GPIO_A16, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA17, GPIO_A17, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA12, GPIO_A12, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA13, GPIO_A13, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA11, GPIO_A11, PIN_PULLUP, 1);

    HAL_Delay_us(20);
    //BSP_GPIO_Set(11,0,1);
    BSP_GPIO_Set(12, 1, 1);
    BSP_GPIO_Set(13, 1, 1);
    BSP_GPIO_Set(14, 1, 1);
    BSP_GPIO_Set(15, 1, 1);
    BSP_GPIO_Set(16, 1, 1);
    BSP_GPIO_Set(17, 1, 1);

#endif
}


