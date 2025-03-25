/**
  ******************************************************************************
  * @file   LTR303.c
  * @author Sifli software development team
  * @brief   This file includes the LTR303 driver functions
  *
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2025,  Sifli Technology
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

#include "LTR303.h"
#include <rtthread.h>
#include "board.h"

#define DRV_DEBUG
#define LOG_TAG              "drv.als"
#include <drv_log.h>

// TODO: This parameters should be defined by board, move to configure?
//#define LTR303_I2C_NAME        "i2c2"

static struct rt_i2c_bus_device *LTR303_bus;

// Add delay ms
void DEV_Delay_ms(uint32_t delay)
{
    rt_thread_delay(delay);
}

// TODO:  gpio read
int DEV_Digital_Read(uint8_t pin)
{
    return 0;
}

// i2c bus initial
static int LTR303_I2C_Init(const char *name)
{
    /* get i2c bus device */
    LTR303_bus = rt_i2c_bus_device_find(name);
    if (LTR303_bus)
    {
        LOG_D("Find i2c bus device %s\n", name);
    }
    else
    {
        LOG_E("Can not found i2c bus %s, init fail\n", name);
        return -1;
    }

    return 0;
}

void LTR303_PowerOn(void)
{
    uint8_t Reg[1];
    RT_ASSERT(rt_i2c_mem_read(LTR303_bus, LTR303_I2CADDR_DEFAULT, LTR303_ALS_CTRL, 8, Reg, 1) > 0);
    Reg[0] |= 0x01;
    RT_ASSERT(rt_i2c_mem_write(LTR303_bus, LTR303_I2CADDR_DEFAULT, LTR303_ALS_CTRL, 8, Reg, 1) > 0);

    RT_ASSERT(rt_i2c_mem_read(LTR303_bus, LTR303_I2CADDR_DEFAULT, LTR303_ALS_CTRL, 8, Reg, 1) > 0);
}

void LTR303_PowerOff(void)
{
    uint8_t Reg[1];
    RT_ASSERT(rt_i2c_mem_read(LTR303_bus, LTR303_I2CADDR_DEFAULT, LTR303_ALS_CTRL, 8, Reg, 1) > 0);
    Reg[0] &= 0xfe;
    RT_ASSERT(rt_i2c_mem_write(LTR303_bus, LTR303_I2CADDR_DEFAULT, LTR303_ALS_CTRL, 8, Reg, 1) > 0);
}

void LTR303_SetGain(ltr303_gain_t Gain)
{
    uint8_t Reg[1];
    RT_ASSERT(rt_i2c_mem_read(LTR303_bus, LTR303_I2CADDR_DEFAULT, LTR303_ALS_CTRL, 8, Reg, 1) > 0);
    // Gain(4:2)
    Reg[0] &= 0xe3;
    Reg[0] |= (Gain << 2);
    RT_ASSERT(rt_i2c_mem_write(LTR303_bus, LTR303_I2CADDR_DEFAULT, LTR303_ALS_CTRL, 8, Reg, 1) > 0);
}

ltr303_gain_t LTR303_GetGain(void)
{
    uint8_t data[1];
    RT_ASSERT(rt_i2c_mem_read(LTR303_bus, LTR303_I2CADDR_DEFAULT, LTR303_ALS_CTRL, 8, data, 1) > 0);
    uint8_t LTR303_Gain = (data[0] & 0x1c) >> 2;
    return LTR303_Gain;
}

void LT303_SetIntegrationTime(ltr303_integrationtime_t Time)
{
    uint8_t Reg[1];
    RT_ASSERT(rt_i2c_mem_read(LTR303_bus, LTR303_I2CADDR_DEFAULT, LTR303_MEAS_RATE, 8, Reg, 1) > 0);
    rt_kprintf("LTR303_MEAS_RATE Reg[0] = %d\n", Reg[0]);
    // Time(5:3)
    Reg[0] &= 0xc7;
    Reg[0] |= (Time << 3);
    RT_ASSERT(rt_i2c_mem_write(LTR303_bus, LTR303_I2CADDR_DEFAULT, LTR303_ALS_CTRL, 8, Reg, 1) > 0);
}

ltr303_integrationtime_t LTR303_GetIntegrationTime(void)
{
    uint8_t data[1];
    RT_ASSERT(rt_i2c_mem_read(LTR303_bus, LTR303_I2CADDR_DEFAULT, LTR303_MEAS_RATE, 8, data, 1) > 0);
    uint8_t LTR303_Time = (data[0] & 0x38) >> 3;
    return LTR303_Time;
}

void LT303_SetMeasurementRate(ltr303_measurerate_t Rate)
{
    // uint8_t Reg = LTR303_Read_Byte(LTR303_MEAS_RATE);
    // // Rate(2:0)
    // Reg &= 0xf8;
    // Reg |= Rate;
    // LTR303_Write_Byte(LTR303_ALS_CTRL, Reg);
    uint8_t Reg[1];
    RT_ASSERT(rt_i2c_mem_read(LTR303_bus, LTR303_I2CADDR_DEFAULT, LTR303_MEAS_RATE, 8, Reg, 1) > 0);
    // Rate(2:0)
    Reg[0] &= 0xf8;
    Reg[0] |= Rate;
    RT_ASSERT(rt_i2c_mem_write(LTR303_bus, LTR303_I2CADDR_DEFAULT, LTR303_ALS_CTRL, 8, Reg, 1) > 0);
}

ltr303_measurerate_t LTR303_GetMeasurementRate(void)
{
    uint8_t data[1];
    RT_ASSERT(rt_i2c_mem_read(LTR303_bus, LTR303_I2CADDR_DEFAULT, LTR303_MEAS_RATE, 8, data, 1) > 0);
    uint8_t LTR303_Rate = data[0] & 0x07;
    return LTR303_Rate;
}

rt_err_t LTR303_Init(struct rt_sensor_config *cfg)
{
    LTR303_I2C_Init(cfg->intf.dev_name);

    LTR303_SetGain(LTR3XX_GAIN_1);
    LT303_SetIntegrationTime(LTR3XX_INTEGTIME_50);
    LT303_SetMeasurementRate(LTR3XX_MEASRATE_50);

    return RT_EOK;
}

uint16_t LTR303_ReadVisible(void)
{
    uint8_t data[4] = {1,2,3,4};
    RT_ASSERT(rt_i2c_mem_read(LTR303_bus, LTR303_I2CADDR_DEFAULT, LTR303_CH1DATA, 8, data, 4) > 0);
    return ((data[1] << 8) | data[0]);
}


#include <string.h>

int cmd_als(int argc, char *argv[])
{
    uint32_t value;
    // if (argc >= 2)
    // {
    //     if (strcmp(argv[1], "-open") == 0)
    //     {
    //         int res = LTR303_Init();
    //         LOG_I("Open ALS %d\n", res);
    //         //LTR303_Enable();
    //     }
    //     else if (strcmp(argv[1], "-close") == 0)
    //     {
    //         //LTR303_Disable();
    //         LOG_I("Close als\n");
    //     }
    //     else if (strcmp(argv[1], "-visi") == 0)
    //     {
    //         value = LTR303_Read_Visible();
    //         LOG_I("get visible %d\n", value);
    //     }
    //     else if (strcmp(argv[1], "-inf") == 0)
    //     {
    //         value = LTR303_Read_Infrared();
    //         LOG_I("get infrared %d\n", value);
    //     }
    //     else if (strcmp(argv[1], "-full") == 0)
    //     {
    //         value = LTR303_Read_FullSpectrum();
    //         LOG_I("get fullspectrum %d\n", value);
    //     }
    //     else if (strcmp(argv[1], "-lux") == 0)
    //     {
    //         value = LTR303_Read_Lux();
    //         LOG_I("get lux %d\n", value);
    //     }
    //     else
    //     {
    //         LOG_I("Invalid parameter\n");
    //     }
    // }
    // else
    // {
    //     LOG_I("Invalid parameter\n");
    // }
    return 0;
}

FINSH_FUNCTION_EXPORT_ALIAS(cmd_als, __cmd_als, Test hw als);
