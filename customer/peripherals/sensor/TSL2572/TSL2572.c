/**
  ******************************************************************************
  * @file   TSL2572.c
  * @author Sifli software development team
  * @brief   This file includes the TSL2572 driver functions
  *
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

#include "TSL2572.h"
#include <rtthread.h>
#include "board.h"

#define DRV_DEBUG
#define LOG_TAG              "drv.als"
#include <drv_log.h>

// TODO: This parameters should be defined by board, move to configure?
//#define TSL2572_I2C_NAME        "i2c2"

static uint8_t TSL2572_Gain, TSL2572_Time;
static struct rt_i2c_bus_device *tsl2572_bus;
static uint8_t tsl2572_index;

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
int TSL2572_I2C_Init(uint8_t address)
{
    /* get i2c bus device */
    tsl2572_bus = rt_i2c_bus_device_find(TSL2572_I2C_NAME);
    if (tsl2572_bus)
    {
        LOG_D("Find i2c bus device %s\n", TSL2572_I2C_NAME);
    }
    else
    {
        LOG_E("Can not found i2c bus %s, init fail\n", TSL2572_I2C_NAME);
        return -1;
    }

    return 0;
}


/******************************************************************************
function:   Read one byte of data to TSL2572 via I2C
parameter:
            Addr: Register address
Info:
******************************************************************************/
static uint8_t TSL2572_Read_Byte(uint8_t Addr)
{
    //Addr = Addr | COMMAND_BIT;
    struct rt_i2c_msg msgs[2];
    uint8_t RegAddr = Addr | COMMAND_BIT;
    uint8_t value = 0;
    uint32_t res;

    if (tsl2572_bus)
    {
        msgs[0].addr  = TSL2572_ADDRESS;    /* Slave address */
        msgs[0].flags = RT_I2C_WR;        /* Write flag */
        msgs[0].buf   = &RegAddr;         /* Slave register address */
        msgs[0].len   = 1;                /* Number of bytes sent */

        msgs[1].addr  = TSL2572_ADDRESS;    /* Slave address */
        msgs[1].flags = RT_I2C_RD;        /* Read flag */
        msgs[1].buf   = &value;             /* Read data pointer */
        msgs[1].len   = 1;              /* Number of bytes read */

        res = rt_i2c_transfer(tsl2572_bus, msgs, 2);
        if (res != 2)
        {
            LOG_I("TSL2591_Read_Byte fail with res %d\n", res);
        }
    }

    return value;
}

/******************************************************************************
function:   Read one word of data to TSL2572 via I2C
parameter:
            Addr: Register address
Info:
******************************************************************************/
static uint16_t TSL2572_Read_Word(uint8_t Addr)
{
    //Addr = Addr | COMMAND_BIT;
    struct rt_i2c_msg msgs[2];
    uint8_t RegAddr = Addr | COMMAND_BIT;
    uint16_t value = 0;
    uint8_t buf[2];
    uint32_t res;

    if (tsl2572_bus)
    {
        msgs[0].addr  = TSL2572_ADDRESS;    /* Slave address */
        msgs[0].flags = RT_I2C_WR;        /* Write flag */
        msgs[0].buf   = &RegAddr;         /* Slave register address */
        msgs[0].len   = 1;                /* Number of bytes sent */

        msgs[1].addr  = TSL2572_ADDRESS;    /* Slave address */
        msgs[1].flags = RT_I2C_RD;        /* Read flag */
        msgs[1].buf   = buf;             /* Read data pointer */
        msgs[1].len   = 2;              /* Number of bytes read */

        res = rt_i2c_transfer(tsl2572_bus, msgs, 2);
        if (res != 2)
        {
            LOG_I("TSL2572_Read_Byte fail with res %d\n", res);
        }
    }

    value = (buf[1] << 8) | buf[0];
    return value;
}

/******************************************************************************
function:   Send one byte of data to TSL2572 via I2C
parameter:
            Addr: Register address
           Value: Write to the value of the register
Info:
******************************************************************************/
static void TSL2572_Write_Byte(uint8_t Addr, uint8_t Value)
{
    //Addr = Addr | COMMAND_BIT;
    struct rt_i2c_msg msgs[2];
    uint8_t value[2];
    uint32_t res;

    if (tsl2572_bus)
    {
        value[0] = Addr | COMMAND_BIT;
        value[1] = Value;

        msgs[0].addr  = TSL2572_ADDRESS;    /* Slave address */
        msgs[0].flags = RT_I2C_WR;        /* Write flag */
        msgs[0].buf   = value;             /* Slave register address */
        msgs[0].len   = 2;                /* Number of bytes sent */

        res = rt_i2c_transfer(tsl2572_bus, msgs, 1);
        if (res != 1)
        {
            LOG_I("TSL2572_Write_Byte FAIL %d\n", res);
        }
    }
}

/******************************************************************************
function:   Enable TSL2572
parameter:
Info:
******************************************************************************/
void TSL2572_Enable(void)
{
    TSL2572_Write_Byte(ENABLE_REGISTER, \
                       ENABLE_AIEN | ENABLE_POWERON | ENABLE_AEN | ENABLE_WEN);
}

/******************************************************************************
function:   Disable TSL2572
parameter:
Info:
******************************************************************************/
void TSL2572_Disable(void)
{
    TSL2572_Write_Byte(ENABLE_REGISTER, \
                       ENABLE_POWEROFF);
}

/******************************************************************************
function:   Read TSL2572 gain
parameter:
Info:
******************************************************************************/
uint8_t TSL2572_Get_Gain(void)
{
    /*************************************************
        LOW_AGAIN           = (0X00)        (1x)
        MEDIUM_AGAIN        = (0X01)        (8x)
        HIGH_AGAIN          = (0X02)        (16x)
        MAX_AGAIN           = (0x03)        (120x)
    *************************************************/
    uint8_t data;
    data = TSL2572_Read_Byte(CONTROL_REGISTER);
    TSL2572_Gain = data & 0x03;
    return data & 0x03;
}

/******************************************************************************
function:   Set the TSL2572 gain
parameter:
Info:
******************************************************************************/
void TSL2572_Set_Gain(uint8_t Gain)
{
    uint8_t control = 0;
    if (Gain == LOW_AGAIN || Gain == MEDIUM_AGAIN \
            || Gain == HIGH_AGAIN || Gain == MAX_AGAIN)
    {
        control =  TSL2572_Read_Byte(CONTROL_REGISTER);
        //control &= 0xfc; //0b11111100
        control = Gain;
        TSL2572_Write_Byte(CONTROL_REGISTER, control);
        TSL2572_Gain = Gain;
    }
    else
    {
        LOG_I("Gain Parameter Error\r\n");
    }
}

/******************************************************************************
function:   Get the TSL2572 Integral Time
parameter:
Info:
******************************************************************************/
uint8_t TSL2572_Get_IntegralTime()
{
    uint8_t control = 0;
    /************************************************************
        ATIME_699MS           (0x00)//699 millis    MAX COUNT 65535
        ATIME_175MS           (0xc0)//175 millis    MAX COUNT 65535
        ATIME_101MS           (0xdb)//300 millis    MAX COUNT 37888
        ATIME_27D3MS          (0xF6)//27.3 millis   MAX COUNT 10240
        ATIME_2D73MS          (0xFF)//2.73 millis   MAX COUNT 1024
    ************************************************************/
    control = TSL2572_Read_Byte(ATIME_REGISTER);
    TSL2572_Time = control & 0xFF;
    return control & 0xFF;
}

/******************************************************************************
function:   Set the TSL2572 Integral Time
parameter:
Info:
******************************************************************************/
void TSL2572_Set_IntegralTime(uint8_t Time)
{
    TSL2572_Write_Byte(ATIME_REGISTER, Time);
    TSL2572_Time = Time;

}

/******************************************************************************
function:   Read channel data
parameter:
Info:
******************************************************************************/
uint16_t TSL2572_Read_Channel0(void)
{
    return TSL2572_Read_Word(CHAN0_LOW);
}

uint16_t TSL2572_Read_Channel1(void)
{
    return TSL2572_Read_Word(CHAN1_LOW);
}

/******************************************************************************
function:   TSL2572 Initialization
parameter:
Info:
******************************************************************************/
int TSL2572_Init(void)
{
    TSL2572_I2C_Init(TSL2572_ADDRESS);//8-bit address

    tsl2572_index = TSL2572_Read_Byte(ID_REGISTER);
    LOG_I("ID = 0x%X \r\n", tsl2572_index);
    if (tsl2572_index != TSL25723_ID && tsl2572_index != TSL25721_ID)
    {
        LOG_E("Get wrong TSL2572 id 0x%x\n", tsl2572_index);
        return -1;
    }
    TSL2572_Enable();
    TSL2572_Set_Gain(MEDIUM_AGAIN);//8X GAIN
    // atime = 256 - interTime / 2.73 ms
    uint32_t atime = 256 - (100 * 100 + 136) / 273;  // for 50hz light, need set multipule of time, set 100 ms
    TSL2572_Set_IntegralTime((uint8_t)(atime & 0xff)); //200ms Integration time
    TSL2572_Write_Byte(PERSIST_REGISTER, 0x01);//filter
    TSL2572_Disable();
    return 0;
}

/******************************************************************************
function:   Read TSL2572 data to convert to Lux value
parameter:
Info:
******************************************************************************/
uint32_t TSL2572_Read_Lux(void)
{
    uint16_t max_counts, channel_0, channel_1;
    uint32_t atime;
    int i;
    TSL2572_Enable();
    //for (i = 0; i < TSL2572_Time + 2; i++)
    {
        DEV_Delay_ms((256 - TSL2572_Time) * 3);
    }
    //if (DEV_Digital_Read(TSL2572_INT_PIN) == 1)
    //    LOG_I("INT 0\r\n");
    //else
    //    LOG_I("INT 1\r\n");
    channel_0 = TSL2572_Read_Channel0();
    channel_1 = TSL2572_Read_Channel1();
    TSL2572_Disable();
    //TSL2572_Enable();
    //TSL2572_Write_Byte(0xE7, 0x13); // ???
    //TSL2572_Disable();

    atime = (256 - TSL2572_Time) * 273 / 100;
    if (TSL2572_Time == ATIME_699MS)
    {
        max_counts = MAX_COUNT_699MS;
    }
    else if (TSL2572_Time == ATIME_175MS)
    {
        max_counts = MAX_COUNT_175MS;
    }
    else if (TSL2572_Time == ATIME_101MS)
    {
        max_counts = MAX_COUNT_101MS;
    }
    else if (TSL2572_Time == ATIME_27D3MS)
    {
        max_counts = MAX_COUNT_27D3MS;
    }
    else
    {
        max_counts = MAX_COUNT_2D73MS;
    }

    if (channel_0 >= max_counts || channel_1 >= max_counts)
    {
        uint8_t gain_t = TSL2572_Get_Gain();
        if (gain_t != LOW_AGAIN)
        {
            gain_t = gain_t - 1;
            TSL2572_Set_Gain(gain_t);
            channel_0 = 0;
            channel_1 = 0;
            while (channel_0 <= 0 || channel_1 <= 0)
            {
                channel_0 = TSL2572_Read_Channel0();
                channel_1 = TSL2572_Read_Channel1();
            }
            DEV_Delay_ms(100);
        }
        else
        {
            LOG_I("Numerical overflow!/r/n");
            return 0;
        }
    }
    double again;
    again = 1.0;
    if (TSL2572_Gain == MEDIUM_AGAIN)
    {
        again = 8.0;
    }
    else if (TSL2572_Gain == HIGH_AGAIN)
    {
        again = 16.0;
    }
    else if (TSL2572_Gain == MAX_AGAIN)
    {
        again = 120.0;
    }
    double Cpl;
    int lux1, lux2 = 0;

    Cpl = (atime * again) / LUX_DF;
    lux1 = ((channel_0 - (1.87 * channel_1)) / Cpl);
    lux2 = ((0.63 * channel_0) - (channel_1)) / Cpl;
    // This is a two segment lux equation where the first
    // segment (Lux1) covers fluorescent and incandescent light
    // and the second segment (Lux2) covers dimmed incandescent light

    int lux = lux1 > lux2 ? lux1 : lux2;
    if (lux > 0)
        return lux;

    return 0;
}

/******************************************************************************
function:   Set the TSL2572 interrupt
parameter:
        SET_LOW : Interrupt low threshold
        SET_HIGH: Interrupt high threshold
Info:   Is the channel 0 AD data as a comparison
******************************************************************************/
void TSL2572_SET_InterruptThreshold(uint16_t SET_LOW, uint16_t SET_HIGH)
{
    TSL2572_Enable();
    TSL2572_Write_Byte(AILTL_REGISTER, SET_LOW & 0xFF);
    TSL2572_Write_Byte(AILTH_REGISTER, SET_LOW >> 8);

    TSL2572_Write_Byte(AIHTL_REGISTER, SET_HIGH & 0xFF);
    TSL2572_Write_Byte(AIHTH_REGISTER, SET_HIGH >> 8);

    //TSL2572_Write_Byte(NPAILTL_REGISTER, 0 );
    //TSL2572_Write_Byte(NPAILTH_REGISTER, 0 );

    //TSL2572_Write_Byte(NPAIHTL_REGISTER, 0xff );
    //TSL2572_Write_Byte(NPAIHTH_REGISTER, 0xff );
    TSL2572_Disable();
}

/******************************************************************************
function:   Set the TSL2572 interrupt
parameter:
        SET_LOW : Interrupt low threshold
        SET_HIGH: Interrupt high threshold
Info:   Based on Lux as a comparison
        Need to use the function TSL2572_Read_Lux() to update the data
******************************************************************************/
void TSL2572_SET_LuxInterrupt(uint16_t SET_LOW, uint16_t SET_HIGH)
{
    double Cpl;
    double again;
    uint16_t atime, channel_1;
    atime  = (uint16_t)((256 - TSL2572_Time) * 273 / 100);
    again = 1.0;
    if (TSL2572_Gain == MEDIUM_AGAIN)
    {
        again = 8.0;
    }
    else if (TSL2572_Gain == HIGH_AGAIN)
    {
        again = 16.0;
    }
    else if (TSL2572_Gain == MAX_AGAIN)
    {
        again = 120.0;
    }

    Cpl = (atime * again) / LUX_DF;
    channel_1 = TSL2572_Read_Channel1();

    SET_HIGH = (int)(Cpl * SET_HIGH) + 2 * channel_1 - 1;
    SET_LOW = (int)(Cpl * SET_LOW) + 2 * channel_1 + 1;

    TSL2572_Enable();
    TSL2572_Write_Byte(AILTL_REGISTER, SET_LOW & 0xFF);
    TSL2572_Write_Byte(AILTH_REGISTER, SET_LOW >> 8);

    TSL2572_Write_Byte(AIHTL_REGISTER, SET_HIGH & 0xFF);
    TSL2572_Write_Byte(AIHTH_REGISTER, SET_HIGH >> 8);

    //TSL2572_Write_Byte(NPAILTL_REGISTER, 0 );
    //TSL2572_Write_Byte(NPAILTH_REGISTER, 0 );

    //TSL2572_Write_Byte(NPAIHTL_REGISTER, 0xff );
    //TSL2572_Write_Byte(NPAIHTH_REGISTER, 0xff );
    TSL2572_Disable();
}

uint32_t TSL2572_Read_FullSpectrum(void)
{
    uint32_t data;
    uint32_t atime  = ((256 - TSL2572_Time) * 273 / 100);
    //Read the full spectrum (IR + visible) light and return its value
    TSL2572_Enable();

    DEV_Delay_ms(atime);

    data = (TSL2572_Read_Channel1()  << 16) | TSL2572_Read_Channel0();
    TSL2572_Disable();
    return data;
}

uint16_t TSL2572_Read_Infrared()
{
    uint16_t data;
    uint32_t atime  = ((256 - TSL2572_Time) * 273 / 100);
    //Read the infrared light and return its value as a 16-bit unsigned number
    TSL2572_Enable();

    DEV_Delay_ms(atime);

    data = TSL2572_Read_Channel0();
    TSL2572_Disable();
    return data;
}

uint32_t TSL2572_Read_Visible()
{
    uint32_t full;
    uint16_t Ch1, Ch0;
    uint32_t atime  = ((256 - TSL2572_Time) * 273 / 100);
    TSL2572_Enable();

    DEV_Delay_ms(atime);

    Ch1 = TSL2572_Read_Channel1();
    Ch0 = TSL2572_Read_Channel0();
    TSL2572_Disable();
    full = (Ch1 << 16) | Ch0;
    return full - Ch1;
}

uint8_t TSL2572_Get_ID()
{
    return tsl2572_index;
}

void *TSL2572_Get_Bus()
{
    return (void *)tsl2572_bus;
}

// TODO : add thread to get value for fixed period?


// TODO: add local test
#define TSL2572_FUNC_TEST
#ifdef TSL2572_FUNC_TEST

#include <string.h>

int cmd_als(int argc, char *argv[])
{
    uint32_t value;
    if (argc >= 2)
    {
        if (strcmp(argv[1], "-open") == 0)
        {
            int res = TSL2572_Init();
            LOG_I("Open ALS %d\n", res);
            //TSL2572_Enable();
        }
        else if (strcmp(argv[1], "-close") == 0)
        {
            //TSL2572_Disable();
            LOG_I("Close als\n");
        }
        else if (strcmp(argv[1], "-visi") == 0)
        {
            value = TSL2572_Read_Visible();
            LOG_I("get visible %d\n", value);
        }
        else if (strcmp(argv[1], "-inf") == 0)
        {
            value = TSL2572_Read_Infrared();
            LOG_I("get infrared %d\n", value);
        }
        else if (strcmp(argv[1], "-full") == 0)
        {
            value = TSL2572_Read_FullSpectrum();
            LOG_I("get fullspectrum %d\n", value);
        }
        else if (strcmp(argv[1], "-lux") == 0)
        {
            value = TSL2572_Read_Lux();
            LOG_I("get lux %d\n", value);
        }
        else
        {
            LOG_I("Invalid parameter\n");
        }
    }
    else
    {
        LOG_I("Invalid parameter\n");
    }
    return 0;
}

FINSH_FUNCTION_EXPORT_ALIAS(cmd_als, __cmd_als, Test hw als);
#endif  /* TSL2572_FUNC_TEST */


