/**
  ******************************************************************************
  * @file   example_i2c.c
  * @author Sifli software development team
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2019 - 2022,  Sifli Technology
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Sifli integrated circuit
 *    in a product or a software update for such product, must reproduce the above
 *    copyright notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of Sifli nor the names of its contributors may be used to endorse
 *    or promote products derived from this software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Sifli integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY SIFLI TECHNOLOGY "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL SIFLI TECHNOLOGY OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <string.h>
#include <stdlib.h>
#include "utest.h"
#include "bf0_hal.h"

#ifdef HAL_I2C_MODULE_ENABLED

/*
    Example Description:
    This example connect to EVB sensor board, user should replace the I2C instance and
    device address when using different resolution, pin mux and register to access depend
    on the new board.
#ifdef SOC_BF_Z0
    For Z0 EVB, use I2C4 control sensor and set correct pinmux before using it.
#else // SF32LB55X
    For A0 EVB, use I2C4 control sensor and need set pinmux and power pin before using it.
#endif
    There are 2 mode to use I2C: polling and INT mode.
    1. Set correct pin mux.
    2. Initial I2C handler.
    3. read BMP280 id with poling mode.
    4. Write value to register .
    5. Read register value after write.
    6. Enable I2C interrupt.
    7. Read ID with int mode.
    8. Write register with INT mode.
    9. Read back rw register with INT mode and check.

    * Note:
        1. USE I2C4 CONTROL BMP280 sensor for example.
           The sensor bord should be connected to the EVB bord.
        2. R/W register make sure write bits that can be write.
        3. I2C instance and PINMUX should correct
*/

// i2c device address
//BMP280_AD0_HIGH
#define TEST_I2C_7BIT_ADDR          (0X77)

// Read only register
#define TEST_I2C_REG_PRODUCTID      (0XD0)

//MMC36X0KJ_REG_X_THD as R/W register
#define TEST_I2C_RW_REG            (0XF5)

// read only register check value
#define TEST_I2C_PRODUCT_ID        (0X58)

static I2C_HandleTypeDef i2c_Handle = {0};

static rt_err_t utest_tc_init(void)
{
    return RT_EOK;
}

static rt_err_t utest_tc_cleanup(void)
{
    return RT_EOK;
}

void I2C4_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    if (i2c_Handle.XferISR != NULL)
    {
        i2c_Handle.XferISR(&i2c_Handle, 0, 0);
    }

    /* leave interrupt */
    rt_interrupt_leave();
}

static void testcase(int argc, char **argv)
{
    uint8_t slaveAddr = TEST_I2C_7BIT_ADDR;
    uint8_t cmd = TEST_I2C_REG_PRODUCTID;
    uint8_t pid = 0;
    uint8_t value = 0xc0;
    uint8_t buf[2];
    HAL_StatusTypeDef ret;

    //----------------------------------------------
    //1. pin mux
#ifdef SOC_SF32LB52X
#define EXAMPLE_I2C I2C2
#define EXAMPLE_I2C_IRQ I2C2_IRQn
    HAL_PIN_Set(PAD_PA40, I2C2_SCL, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA39, I2C2_SDA, PIN_PULLUP, 1);
#ifdef HAL_USING_HTOL
    LOG_I("HTOL skip I2C test for 52x\n");
    uassert_true(RT_TRUE);
    return;
#endif
#else
#define EXAMPLE_I2C I2C4
#define EXAMPLE_I2C_IRQ I2C4_IRQn
    HAL_PIN_Set(PAD_PB03, GPIO_B3, PIN_PULLUP, 0);           // I2C4 power
#ifdef  BSP_USING_BOARD_EC_LB555XXX
    HAL_PIN_Set(PAD_PB25, GPIO_B25, PIN_PULLUP, 0);          //sensor power
    HAL_Delay(10);  // add a delay to make sure sensor board power up
#endif
    HAL_PIN_Set(PAD_PB04, I2C4_SCL, PIN_NOPULL, 0);          // I2C4
    HAL_PIN_Set(PAD_PB05, I2C4_SDA, PIN_NOPULL, 0);
#endif
    //----------------------------------------------
    // 2. i2c init

    i2c_Handle.Instance = EXAMPLE_I2C;
    i2c_Handle.Mode = HAL_I2C_MODE_MASTER;
    i2c_Handle.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    i2c_Handle.Init.ClockSpeed = 400000;
    i2c_Handle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    ret = HAL_I2C_Init(&i2c_Handle);
    if (ret != HAL_OK)
    {
        uassert_true(RT_FALSE);
        return;
    }

    // 3. read MMC36XOKJ id with poling mode
    __HAL_I2C_ENABLE(&i2c_Handle);  // for master, enable it before transmit
    // write register
    cmd = TEST_I2C_REG_PRODUCTID;
    ret = HAL_I2C_Master_Transmit(&i2c_Handle, slaveAddr, &cmd, 1, 1000);
    uassert_true(ret == HAL_OK);
    // read register value
    ret = HAL_I2C_Master_Receive(&i2c_Handle, slaveAddr, &pid, 1, 1000);
    uassert_true(ret == HAL_OK);

    LOG_I("get pid %d, expect %d", pid, TEST_I2C_PRODUCT_ID);
    uassert_true(pid == TEST_I2C_PRODUCT_ID);

    // 4. Write value to register
    buf[0] = (uint8_t)TEST_I2C_RW_REG;
    buf[1] = value;
    ret = HAL_I2C_Master_Transmit(&i2c_Handle, slaveAddr, (uint8_t *)buf, 2, 1000);

    // 5. Read register value after write
    cmd = TEST_I2C_RW_REG;
    ret = HAL_I2C_Master_Transmit(&i2c_Handle, slaveAddr, &cmd, 1, 1000);
    uassert_true(ret == HAL_OK);
    ret = HAL_I2C_Master_Receive(&i2c_Handle, slaveAddr, &pid, 1, 1000);
    uassert_true(ret == HAL_OK);

    // check if read result same to write
    LOG_I("get value %d, expect %d", pid, value);
    uassert_true(pid == value);
    __HAL_I2C_DISABLE(&i2c_Handle); // for master, disable it after transmit to reduce error status

    // 6. Enable I2C interrupt
    HAL_NVIC_SetPriority(EXAMPLE_I2C_IRQ, 3, 0);
    NVIC_EnableIRQ(EXAMPLE_I2C_IRQ);

    // 7. Read ID with int mode
    __HAL_I2C_ENABLE(&i2c_Handle);  // for master, enable it before transmit
    // write reg address
    cmd = TEST_I2C_REG_PRODUCTID;
    ret = HAL_I2C_Master_Transmit_IT(&i2c_Handle, slaveAddr, &cmd, 1);
    uassert_true(ret == HAL_OK);

    // wait I2C status ready
    while (ret == HAL_OK)
    {
        if (HAL_I2C_GetState(&i2c_Handle) == HAL_I2C_STATE_READY)
        {
            ret = HAL_OK;
            break;
        }
        if (HAL_I2C_GetState(&i2c_Handle) == HAL_I2C_STATE_ERROR)
            ret = HAL_ERROR;
        if (HAL_I2C_GetState(&i2c_Handle) == HAL_I2C_STATE_TIMEOUT)
            ret = HAL_TIMEOUT;
    }
    uassert_true(ret == HAL_OK);
    // read
    ret = HAL_I2C_Master_Receive_IT(&i2c_Handle, slaveAddr, &pid, 1);
    uassert_true(ret == HAL_OK);

    // wait I2C status ready
    while (ret == HAL_OK)
    {
        if (HAL_I2C_GetState(&i2c_Handle) == HAL_I2C_STATE_READY)
        {
            ret = HAL_OK;
            break;
        }
        if (HAL_I2C_GetState(&i2c_Handle) == HAL_I2C_STATE_ERROR)
            ret = HAL_ERROR;
        if (HAL_I2C_GetState(&i2c_Handle) == HAL_I2C_STATE_TIMEOUT)
            ret = HAL_TIMEOUT;
    }
    uassert_true(ret == HAL_OK);

    LOG_I("get pid %d, expect %d", pid, TEST_I2C_PRODUCT_ID);
    uassert_true(pid == TEST_I2C_PRODUCT_ID);

    // 8. Write register with INT mode
    buf[0] = (uint8_t)TEST_I2C_RW_REG;
    buf[1] = value;
    ret = HAL_I2C_Master_Transmit_IT(&i2c_Handle, slaveAddr, (uint8_t *)buf, 2);
    uassert_true(ret == HAL_OK);

    // wait I2C status ready
    while (ret == HAL_OK)
    {
        if (HAL_I2C_GetState(&i2c_Handle) == HAL_I2C_STATE_READY)
        {
            ret = HAL_OK;
            break;
        }
        if (HAL_I2C_GetState(&i2c_Handle) == HAL_I2C_STATE_ERROR)
            ret = HAL_ERROR;
        if (HAL_I2C_GetState(&i2c_Handle) == HAL_I2C_STATE_TIMEOUT)
            ret = HAL_TIMEOUT;
    }
    uassert_true(ret == HAL_OK);

    // 9. Read back rw register with INT mode and check
    cmd = TEST_I2C_RW_REG;
    ret = HAL_I2C_Master_Transmit_IT(&i2c_Handle, slaveAddr, &cmd, 1);
    uassert_true(ret == HAL_OK);

    // wait I2C status ready
    while (ret == HAL_OK)
    {
        if (HAL_I2C_GetState(&i2c_Handle) == HAL_I2C_STATE_READY)
        {
            ret = HAL_OK;
            break;
        }
        if (HAL_I2C_GetState(&i2c_Handle) == HAL_I2C_STATE_ERROR)
            ret = HAL_ERROR;
        if (HAL_I2C_GetState(&i2c_Handle) == HAL_I2C_STATE_TIMEOUT)
            ret = HAL_TIMEOUT;
    }
    uassert_true(ret == HAL_OK);
    // read back
    ret = HAL_I2C_Master_Receive_IT(&i2c_Handle, slaveAddr, &pid, 1);
    uassert_true(ret == HAL_OK);

    // wait I2C status ready
    while (ret == HAL_OK)
    {
        if (HAL_I2C_GetState(&i2c_Handle) == HAL_I2C_STATE_READY)
        {
            ret = HAL_OK;
            break;
        }
        if (HAL_I2C_GetState(&i2c_Handle) == HAL_I2C_STATE_ERROR)
            ret = HAL_ERROR;
        if (HAL_I2C_GetState(&i2c_Handle) == HAL_I2C_STATE_TIMEOUT)
            ret = HAL_TIMEOUT;
    }
    uassert_true(ret == HAL_OK);

    LOG_I("get value %d, expect %d", pid, value);
    uassert_true(pid == value);

    __HAL_I2C_DISABLE(&i2c_Handle); // for master, disable it after transmit to reduce error status
}


UTEST_TC_EXPORT(testcase, "example_i2c", utest_tc_init, utest_tc_cleanup, 10);

#endif /* HAL_I2C_MODULE_ENABLED */


/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
