/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rtconfig.h"
#include "bf0_hal.h"
#include "drv_io.h"
#include "stdio.h"
#include "string.h"
#include "rtthread.h"
#include "bf0_hal_i2c.h"


static I2C_HandleTypeDef hi2c;

#define SLAVE_ADDRESS 0x5A
uint8_t master_tx_data[] = {0x11, 0x22, 0x33, 0x44};
uint8_t master_rx_data[4];

void I2C_Master_Init(void)
{
    HAL_StatusTypeDef ret;

#ifdef SF32LB52X
    HAL_RCC_EnableModule(RCC_MOD_I2C2);
    HAL_PIN_Set(PAD_PA41, I2C2_SCL, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA42, I2C2_SDA, PIN_PULLUP, 1);
#define MASTER_I2C I2C2
#elif defined(SF32LB58X)
    HAL_RCC_EnableModule(RCC_MOD_I2C6);
    HAL_PIN_Set(PAD_PB28, I2C6_SCL, PIN_PULLUP, 0);
    HAL_PIN_Set(PAD_PB29, I2C6_SDA, PIN_PULLUP, 0);
#define MASTER_I2C I2C6
#elif defined(SF32LB56X)
    HAL_RCC_EnableModule(RCC_MOD_I2C3);
    HAL_PIN_Set(PAD_PA20, I2C3_SCL, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA12, I2C3_SDA, PIN_PULLUP, 1);
#define MASTER_I2C I2C3
#elif defined(SF32LB57X)
    HAL_RCC_EnableModule(RCC_MOD_I2C2);
    HAL_PIN_Set(PAD_PA22, I2C2_SCL, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA23, I2C2_SDA, PIN_PULLUP, 1);
#define MASTER_I2C I2C2
#endif

    hi2c.Instance = MASTER_I2C;
    hi2c.Mode = HAL_I2C_MODE_MASTER; // i2c master mode
    hi2c.Init.ClockSpeed = 400000;           // 400kHz
    hi2c.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;

    ret = HAL_I2C_Init(&hi2c);
    if (ret != HAL_OK)
    {
        rt_kprintf("I2C Master Init failed: %d\n", ret);
        return;
    }
    rt_kprintf("I2C Master Init Success!\n");
}

HAL_StatusTypeDef I2C_Master_Write_To_Slave(uint16_t slave_addr, uint8_t *pData, uint16_t size)
{
    HAL_StatusTypeDef ret;
    __HAL_I2C_ENABLE(&hi2c);
    rt_kprintf("Master sending %d bytes to slave 0x%02X: ", size, slave_addr);
    for (int i = 0; i < size; i++)
    {
        rt_kprintf("0x%02X ", pData[i]);
    }
    rt_kprintf("\n");
    ret = HAL_I2C_Master_Transmit(&hi2c, slave_addr, pData, size, 1000);

    if (ret == HAL_OK)
    {
        rt_kprintf("Master transmit success!\n");
    }
    else
    {
        rt_kprintf("Master transmit failed: %d\n", ret);
    }
    __HAL_I2C_DISABLE(&hi2c);
    return ret;
}

HAL_StatusTypeDef I2C_Master_Read_From_Slave(uint16_t slave_addr, uint8_t *pData, uint16_t size)
{
    HAL_StatusTypeDef ret;
    __HAL_I2C_ENABLE(&hi2c);

    rt_kprintf("Master reading %d bytes from slave 0x%02X\n", size, slave_addr);
    ret = HAL_I2C_Master_Receive(&hi2c, slave_addr, pData, size, 1000);

    if (ret == HAL_OK)
    {
        rt_kprintf("Master receive success, data: ");
        for (int i = 0; i < size; i++)
        {
            rt_kprintf("0x%02X ", pData[i]);
        }
        rt_kprintf("\n");
    }
    else
    {
        rt_kprintf("Master receive failed: %d\n", ret);
    }
    __HAL_I2C_DISABLE(&hi2c);

    return ret;
}

void I2C_Communication_Test(void)
{
    HAL_StatusTypeDef ret;

    rt_kprintf("=== I2C Communication Test Start ===\n");
    ret = I2C_Master_Write_To_Slave(SLAVE_ADDRESS, master_tx_data, sizeof(master_tx_data));
    if (ret != HAL_OK)
    {
        rt_kprintf("Write test failed\n");
        return;
    }

    rt_thread_delay(100);

    ret = I2C_Master_Read_From_Slave(SLAVE_ADDRESS, master_rx_data, sizeof(master_tx_data));

    if (ret != HAL_OK)
    {
        rt_kprintf("Read test failed\n");
        return;
    }
    rt_kprintf("=== I2C Communication Test End ===\n\n");
}

int main(void)
{
    rt_kprintf("Start I2C Master Demo!\n");

    I2C_Master_Init();

    while (1)
    {
        I2C_Communication_Test();
        rt_thread_delay(2000);
    }

    return 0;
}