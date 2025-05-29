#include "bsp_board.h"
#include <stdint.h>

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

}

__WEAK void BSP_PowerUpCustom(bool is_deep_sleep)
{
    // VSYS
    BSP_GPIO_Set(38, 1, 1);

    // VSYS to VCC_3V3
    BSP_GPIO_Set(26, 1, 1);

    // GS_3V3
    BSP_GPIO_Set(30, 1, 1);
    HAL_Delay_us(500); // must delay 500us to make sure power on finish

    // Configure Charger
    const uint8_t charger_addr = 0x49;
    HAL_StatusTypeDef ret;
    I2C_HandleTypeDef i2c_Handle;
    HAL_RCC_EnableModule(RCC_MOD_I2C2);
    i2c_Handle.Instance = I2C2;
    i2c_Handle.Mode = HAL_I2C_MODE_MASTER; // i2c master mode
    i2c_Handle.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT; // i2c 7bits device address mode
    i2c_Handle.Init.ClockSpeed = 400000; // i2c speed (hz)
    i2c_Handle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    i2c_Handle.Init.OwnAddress1 = 0;
    ret = HAL_I2C_Init(&i2c_Handle);
    if (ret != HAL_OK)
    {
        goto exit;
    }

    __HAL_I2C_ENABLE(&i2c_Handle);
    uint8_t data = 0;

    // CEB, address 0x01, bit3
    ret = HAL_I2C_Mem_Read(&i2c_Handle, charger_addr, 0x01, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);
    if (ret != HAL_OK)
    {
        goto exit;
    }
    data &= ~(1 << 3); // clear bit3
    ret = HAL_I2C_Mem_Write(&i2c_Handle, charger_addr, 0x01, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);
    if (ret != HAL_OK)
    {
        goto exit;
    }
    // disable watchdog, address 0x05, bit5-6
    ret = HAL_I2C_Mem_Read(&i2c_Handle, charger_addr, 0x05, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);
    if (ret != HAL_OK)
    {
        return;
    }
    data &= ~(3 << 5);
    ret = HAL_I2C_Mem_Write(&i2c_Handle, charger_addr, 0x05, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);
    if (ret != HAL_OK)
    {
        goto exit;
    }
exit:
    __HAL_I2C_DISABLE(&i2c_Handle);
}


void BSP_Power_Up(bool is_deep_sleep)
{
    BSP_PowerUpCustom(is_deep_sleep);
#ifdef SOC_BF0_HCPU
    if (!is_deep_sleep)
    {
#ifdef BSP_USING_PSRAM1
        bsp_psram_exit_low_power("psram1");
#endif /* BSP_USING_PSRAM1 */
    }
#endif  /* SOC_BF0_HCPU */
}



void BSP_IO_Power_Down(int coreid, bool is_deep_sleep)
{
    BSP_PowerDownCustom(coreid, is_deep_sleep);
#ifdef SOC_BF0_HCPU
    if (coreid == CORE_ID_HCPU)
    {
#ifdef BSP_USING_PSRAM1
        bsp_psram_enter_low_power("psram1");
#endif /* BSP_USING_PSRAM1 */
    }
#endif  /* SOC_BF0_HCPU */
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


