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
#define SLAVE_ADDRESS 0x5A //Slave address

uint8_t rx_buffer[4];//
uint8_t tx_data[] = {0xAA, 0xBB, 0xCC, 0xDD};//Send data

void I2C_Slave_Init(void)
{
    HAL_StatusTypeDef ret;
    //pin nux
#ifdef SF32LB52X
    HAL_RCC_EnableModule(RCC_MOD_I2C2); // enable i2c2
    HAL_PIN_Set(PAD_PA41, I2C2_SCL, PIN_PULLUP, 1); // i2c io select
    HAL_PIN_Set(PAD_PA42, I2C2_SDA, PIN_PULLUP, 1);
#define SLAVE_I2C I2C2// i2c number of cpu
#elif defined(SF32LB58X)
    HAL_RCC_EnableModule(RCC_MOD_I2C6);  // enable i2c6
    HAL_PIN_Set(PAD_PB28, I2C6_SCL, PIN_PULLUP, 0); // i2c io select
    HAL_PIN_Set(PAD_PB29, I2C6_SDA, PIN_PULLUP, 0);
#define SLAVE_I2C I2C6// i2c number of cpu
#elif defined(SF32LB56X)
    HAL_RCC_EnableModule(RCC_MOD_I2C3); // enable i2c3
    HAL_PIN_Set(PAD_PA20, I2C3_SCL, PIN_PULLUP, 1); // i2c io select
    HAL_PIN_Set(PAD_PA12, I2C3_SDA, PIN_PULLUP, 1);
#define SLAVE_I2C I2C3// i2c number of cpu
#elif defined(SF32LB57X)
    HAL_RCC_EnableModule(RCC_MOD_I2C2); // enable i2c2
    HAL_PIN_Set(PAD_PA22, I2C2_SCL, PIN_PULLUP, 1); // i2c io select
    HAL_PIN_Set(PAD_PA23, I2C2_SDA, PIN_PULLUP, 1);
#define SLAVE_I2C I2C2// i2c number of cpu
#endif
    //i2c init
    hi2c.Instance = SLAVE_I2C;// i2c slave mode
    hi2c.Mode = HAL_I2C_MODE_SLAVE;
    hi2c.Init.ClockSpeed = 400000;           // i2c speed(hz)
    hi2c.Init.OwnAddress1 = SLAVE_ADDRESS;  // slave address
    hi2c.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;// i2c 7bits device address mode
    hi2c.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;

    ret = HAL_I2C_Init(&hi2c);
    if (ret != HAL_OK)
    {
        rt_kprintf("I2C Init failed: %d\n", ret);
        return;
    }
    rt_kprintf("I2C Slave Init Success!\n");
}

int main(void)
{
    HAL_StatusTypeDef ret;

    rt_kprintf("Start I2C Slave Demo!\n");
    I2C_Slave_Init();
    rt_thread_delay(500);//Give the host device some time to prepare for communication.

    while (1)
    {
        ret = HAL_I2C_Slave_Receive(&hi2c, rx_buffer, sizeof(rx_buffer), HAL_MAX_DELAY);
        if (ret == HAL_OK)
        {
            rt_kprintf("Slave Receive Success, data: ");
            for (int i = 0; i < sizeof(rx_buffer); i++)
            {
                rt_kprintf("0x%02X  ", rx_buffer[i]);
            }
            rt_kprintf("\n");
        }
        else
        {
            rt_kprintf("Slave Receive Failed: %d\n", ret);
        }

        ret = HAL_I2C_Slave_Transmit(&hi2c, tx_data, sizeof(tx_data), HAL_MAX_DELAY);
        if (ret == HAL_OK)
        {
            rt_kprintf("Slave Transmit Success!\n");
        }
        else
        {
            rt_kprintf("Slave Transmit Failed: %d\n", ret);
        }
    }

    return 0;
}

