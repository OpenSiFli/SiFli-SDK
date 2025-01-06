/**
  ******************************************************************************
  * @file   ICM20948.c
  * @author Sifli software development team
  * @brief   This file includes the invsense ICM20948 driver functions
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

#include "board.h"
#include "ICM20948.h"

#ifdef SENSOR_USING_ICM20948

#define DRV_DEBUG
#define LOG_TAG              "drv.icm"
#include <drv_log.h>

ICM20948_ST_SENSOR_DATA gstGyroOffset = {0, 0, 0};
static struct rt_i2c_bus_device *i2cbus = NULL;
static uint8_t icm_slave_addr = I2C_ADD_ICM20948;


static bool invmsICM20948MagCheck(void);
static void invmsICM20948CalAvgValue(uint8_t *pIndex, int16_t *pAvgBuffer, int16_t InVal, int32_t *pOutVal);
static void invmsICM20948GyroOffset(void);

static void invmsICM20948ReadSecondary(uint8_t u8I2CAddr, uint8_t u8RegAddr, uint8_t u8Len, uint8_t *pu8data);
static void invmsICM20948WriteSecondary(uint8_t u8I2CAddr, uint8_t u8RegAddr, uint8_t u8data);

static void Delay_ms(uint32_t ms)
{
    rt_thread_delay(ms);
}
static void I2C_WriteOneByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t Data)
{
#ifdef RT_USING_I2C
    struct rt_i2c_msg msgs[2];
    uint8_t value[2];
    uint32_t res;

    if (i2cbus)
    {
        value[0] = RegAddr;
        value[1] = Data;

        msgs[0].addr  = DevAddr;    /* Slave address */
        msgs[0].flags = RT_I2C_WR;        /* Write flag */
        msgs[0].buf   = value;             /* Slave register address */
        msgs[0].len   = 2;                /* Number of bytes sent */

        //msgs[1].addr  = DevAddr;    /* Slave address */
        //msgs[1].flags = RT_I2C_WR;        /* Read flag */
        //msgs[1].buf   = &Data;              /* Read data pointer */
        //msgs[1].len   = 1;              /* Number of bytes read */
        res = rt_i2c_transfer(i2cbus, msgs, 1);
        if (res == 1)
        {
            //LOG_D("I2C_WriteOneByte OK\n");
        }
        else
        {
            LOG_D("I2C_WriteOneByte FAIL %d\n", res);
        }

    }
#endif
}

static uint8_t I2C_ReadOneByte(uint8_t DevAddr, uint8_t RegAddr)
{
#ifdef RT_USING_I2C
    struct rt_i2c_msg msgs[2];
    uint32_t res;
    uint8_t value;

    if (i2cbus)
    {
        msgs[0].addr  = DevAddr;    /* Slave address */
        msgs[0].flags = RT_I2C_WR;        /* Write flag */
        msgs[0].buf   = &RegAddr;             /* Slave register address */
        msgs[0].len   = 1;                /* Number of bytes sent */

        msgs[1].addr  = DevAddr;    /* Slave address */
        msgs[1].flags = RT_I2C_RD;        /* Read flag */
        msgs[1].buf   = &value;              /* Read data pointer */
        msgs[1].len   = 1;              /* Number of bytes read */

        res = rt_i2c_transfer(i2cbus, msgs, 2);
        if (res == 2)
        {
            //LOG_D("I2C_ReadOneByte OK\n");
            return value;
        }
        else
        {
            LOG_D("I2C_ReadOneByte fail %d\n", res);
            return 0xff;
        }
    }
#endif
    return 0xff;
}


extern uint8_t invmsICM20948Check(void)
{
    uint8_t bRet = 0;
    icm_slave_addr = I2C_ADD_ICM20948_L;
    if (REG_VAL_WIA == I2C_ReadOneByte(icm_slave_addr, REG_ADD_WIA))
    {
        bRet = 1;
        icm_slave_addr = I2C_ADD_ICM20948_L;
    }
    else
    {
        icm_slave_addr = I2C_ADD_ICM20948_H;
        if (REG_VAL_WIA == I2C_ReadOneByte(icm_slave_addr, REG_ADD_WIA))
        {
            bRet = 1;
            icm_slave_addr = I2C_ADD_ICM20948_H;
        }
    }
    return bRet;
}

extern uint8_t invmsICM20948Init(void)
{
    /* get i2c bus device */
    i2cbus = rt_i2c_bus_device_find(SENSOR_I2C_BUS);
    if (i2cbus)
    {
        LOG_D("Find i2c bus device %s\n", SENSOR_I2C_BUS);
    }
    else
    {
        LOG_E("Can not found i2c bus %s, init fail\n", SENSOR_I2C_BUS);
        return false;
    }

    uint8_t res = invmsICM20948Check();
    if (res)
    {
        LOG_I("Check ICM20948 success\n");
    }
    else
    {
        LOG_E("Check ICM20948 fail\n");
        return false;
    }

    /* user bank 0 register */
    I2C_WriteOneByte(icm_slave_addr, REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0);
    I2C_WriteOneByte(icm_slave_addr, REG_ADD_PWR_MIGMT_1,  REG_VAL_ALL_RGE_RESET);
    Delay_ms(10);
    I2C_WriteOneByte(icm_slave_addr, REG_ADD_PWR_MIGMT_1,  REG_VAL_RUN_MODE);

    /* user bank 2 register */
    I2C_WriteOneByte(icm_slave_addr, REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_2);
    I2C_WriteOneByte(icm_slave_addr, REG_ADD_GYRO_SMPLRT_DIV, 0x07);
    I2C_WriteOneByte(icm_slave_addr, REG_ADD_GYRO_CONFIG_1,
                     REG_VAL_BIT_GYRO_DLPCFG_6 | REG_VAL_BIT_GYRO_FS_1000DPS | REG_VAL_BIT_GYRO_DLPF);
    I2C_WriteOneByte(icm_slave_addr, REG_ADD_ACCEL_SMPLRT_DIV_2,  0x07);
    I2C_WriteOneByte(icm_slave_addr, REG_ADD_ACCEL_CONFIG,
                     REG_VAL_BIT_ACCEL_DLPCFG_6 | REG_VAL_BIT_ACCEL_FS_2g | REG_VAL_BIT_ACCEL_DLPF);

    /* user bank 0 register */
    I2C_WriteOneByte(icm_slave_addr, REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0);

    Delay_ms(100);
    /* offset */
    invmsICM20948GyroOffset();

    res = invmsICM20948MagCheck();
    if (res)
    {
        LOG_I("Check ICM20948Mag success\n");
    }
    else
    {
        LOG_E("Check ICM20948Mag fail\n");
    }

    invmsICM20948WriteSecondary(I2C_ADD_ICM20948_AK09916 | I2C_ADD_ICM20948_AK09916_WRITE,
                                REG_ADD_MAG_CNTL2, REG_VAL_MAG_MODE_20HZ);
    return true;
}
extern void invmsICM20948GyroRead(int16_t *ps16X, int16_t *ps16Y, int16_t *ps16Z)
{
    uint8_t u8Buf[6];
    int16_t s16Buf[3] = {0};
    uint8_t i;
    int32_t s32OutBuf[3] = {0};
    static ICM20948_ST_AVG_DATA sstAvgBuf[3];

    u8Buf[0] = I2C_ReadOneByte(icm_slave_addr, REG_ADD_GYRO_XOUT_L);
    u8Buf[1] = I2C_ReadOneByte(icm_slave_addr, REG_ADD_GYRO_XOUT_H);
    s16Buf[0] = (u8Buf[1] << 8) | u8Buf[0];

    u8Buf[0] = I2C_ReadOneByte(icm_slave_addr, REG_ADD_GYRO_YOUT_L);
    u8Buf[1] = I2C_ReadOneByte(icm_slave_addr, REG_ADD_GYRO_YOUT_H);
    s16Buf[1] = (u8Buf[1] << 8) | u8Buf[0];

    u8Buf[0] = I2C_ReadOneByte(icm_slave_addr, REG_ADD_GYRO_ZOUT_L);
    u8Buf[1] = I2C_ReadOneByte(icm_slave_addr, REG_ADD_GYRO_ZOUT_H);
    s16Buf[2] = (u8Buf[1] << 8) | u8Buf[0];

#if 1
    for (i = 0; i < 3; i ++)
    {
        invmsICM20948CalAvgValue(&sstAvgBuf[i].u8Index, sstAvgBuf[i].s16AvgBuffer, s16Buf[i], s32OutBuf + i);
    }
    *ps16X = s32OutBuf[0] - gstGyroOffset.s16X;
    *ps16Y = s32OutBuf[1] - gstGyroOffset.s16Y;
    *ps16Z = s32OutBuf[2] - gstGyroOffset.s16Z;
#else
    *ps16X = s16Buf[0];
    *ps16Y = s16Buf[1];
    *ps16Z = s16Buf[2];
#endif
    return;
}

extern void invmsICM20948AccelRead(int16_t *ps16X, int16_t *ps16Y, int16_t *ps16Z)
{
    uint8_t u8Buf[2];
    int16_t s16Buf[3] = {0};
    uint8_t i;
    int32_t s32OutBuf[3] = {0};
    static ICM20948_ST_AVG_DATA sstAvgBuf[3];

    u8Buf[0] = I2C_ReadOneByte(icm_slave_addr, REG_ADD_ACCEL_XOUT_L);
    u8Buf[1] = I2C_ReadOneByte(icm_slave_addr, REG_ADD_ACCEL_XOUT_H);
    s16Buf[0] = (u8Buf[1] << 8) | u8Buf[0];

    u8Buf[0] = I2C_ReadOneByte(icm_slave_addr, REG_ADD_ACCEL_YOUT_L);
    u8Buf[1] = I2C_ReadOneByte(icm_slave_addr, REG_ADD_ACCEL_YOUT_H);
    s16Buf[1] = (u8Buf[1] << 8) | u8Buf[0];

    u8Buf[0] = I2C_ReadOneByte(icm_slave_addr, REG_ADD_ACCEL_ZOUT_L);
    u8Buf[1] = I2C_ReadOneByte(icm_slave_addr, REG_ADD_ACCEL_ZOUT_H);
    s16Buf[2] = (u8Buf[1] << 8) | u8Buf[0];

#if 1
    for (i = 0; i < 3; i ++)
    {
        invmsICM20948CalAvgValue(&sstAvgBuf[i].u8Index, sstAvgBuf[i].s16AvgBuffer, s16Buf[i], s32OutBuf + i);
    }
    *ps16X = s32OutBuf[0];
    *ps16Y = s32OutBuf[1];
    *ps16Z = s32OutBuf[2];

#else
    *ps16X = s16Buf[0];
    *ps16Y = s16Buf[1];
    *ps16Z = s16Buf[2];
#endif
    return;

}

extern void invmsICM20948MagRead(int16_t *ps16X, int16_t *ps16Y, int16_t *ps16Z)
{
    uint8_t counter = 20;
    uint8_t u8Data[MAG_DATA_LEN];
    int16_t s16Buf[3] = {0};
    uint8_t i;
    int32_t s32OutBuf[3] = {0};
    static ICM20948_ST_AVG_DATA sstAvgBuf[3];
    while (counter > 0)
    {
        Delay_ms(10);
        invmsICM20948ReadSecondary(I2C_ADD_ICM20948_AK09916 | I2C_ADD_ICM20948_AK09916_READ,
                                   REG_ADD_MAG_ST2, 1, u8Data);

        if ((u8Data[0] & 0x01) != 0)
            break;

        counter--;
    }

    if (counter != 0)
    {
        invmsICM20948ReadSecondary(I2C_ADD_ICM20948_AK09916 | I2C_ADD_ICM20948_AK09916_READ,
                                   REG_ADD_MAG_DATA,
                                   MAG_DATA_LEN,
                                   u8Data);
        s16Buf[0] = ((int16_t)u8Data[1] << 8) | u8Data[0];
        s16Buf[1] = ((int16_t)u8Data[3] << 8) | u8Data[2];
        s16Buf[2] = ((int16_t)u8Data[5] << 8) | u8Data[4];
    }
    else
    {
        LOG_D("Mag is bussy \r\n");
    }
#if 1
    for (i = 0; i < 3; i ++)
    {
        invmsICM20948CalAvgValue(&sstAvgBuf[i].u8Index, sstAvgBuf[i].s16AvgBuffer, s16Buf[i], s32OutBuf + i);
    }

    *ps16X =  s32OutBuf[0];
    *ps16Y = -s32OutBuf[1];
    *ps16Z = -s32OutBuf[2];
#else
    *ps16X = s16Buf[0];
    *ps16Y = -s16Buf[1];
    *ps16Z = -s16Buf[2];
#endif
    return;
}

static bool invmsICM20948MagCheck(void)
{
    bool bRet = false;
    uint8_t u8Ret[2];

    invmsICM20948ReadSecondary(I2C_ADD_ICM20948_AK09916 | I2C_ADD_ICM20948_AK09916_READ,
                               REG_ADD_MAG_WIA1, 2, u8Ret);
    if ((u8Ret[0] == REG_VAL_MAG_WIA1) && (u8Ret[1] == REG_VAL_MAG_WIA2))
    {
        bRet = true;
    }

    return bRet;
}

static void invmsICM20948ReadSecondary(uint8_t u8I2CAddr, uint8_t u8RegAddr, uint8_t u8Len, uint8_t *pu8data)
{
    uint8_t i;
    uint8_t u8Temp;

    I2C_WriteOneByte(icm_slave_addr, REG_ADD_REG_BANK_SEL,  REG_VAL_REG_BANK_3); //swtich bank3
    I2C_WriteOneByte(icm_slave_addr, REG_ADD_I2C_SLV0_ADDR, u8I2CAddr);
    I2C_WriteOneByte(icm_slave_addr, REG_ADD_I2C_SLV0_REG,  u8RegAddr);
    I2C_WriteOneByte(icm_slave_addr, REG_ADD_I2C_SLV0_CTRL, REG_VAL_BIT_SLV0_EN | u8Len);

    I2C_WriteOneByte(icm_slave_addr, REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0); //swtich bank0

    u8Temp = I2C_ReadOneByte(icm_slave_addr, REG_ADD_USER_CTRL);
    u8Temp |= REG_VAL_BIT_I2C_MST_EN;
    I2C_WriteOneByte(icm_slave_addr, REG_ADD_USER_CTRL, u8Temp);
    Delay_ms(5);
    u8Temp &= ~REG_VAL_BIT_I2C_MST_EN;
    I2C_WriteOneByte(icm_slave_addr, REG_ADD_USER_CTRL, u8Temp);

    for (i = 0; i < u8Len; i++)
    {
        *(pu8data + i) = I2C_ReadOneByte(icm_slave_addr, REG_ADD_EXT_SENS_DATA_00 + i);

    }
    I2C_WriteOneByte(icm_slave_addr, REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_3); //swtich bank3

    u8Temp = I2C_ReadOneByte(icm_slave_addr, REG_ADD_I2C_SLV0_CTRL);
    u8Temp &= ~((REG_VAL_BIT_I2C_MST_EN) & (REG_VAL_BIT_MASK_LEN));
    I2C_WriteOneByte(icm_slave_addr, REG_ADD_I2C_SLV0_CTRL,  u8Temp);

    I2C_WriteOneByte(icm_slave_addr, REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0); //swtich bank0

}

static void invmsICM20948WriteSecondary(uint8_t u8I2CAddr, uint8_t u8RegAddr, uint8_t u8data)
{
    uint8_t u8Temp;
    I2C_WriteOneByte(icm_slave_addr, REG_ADD_REG_BANK_SEL,  REG_VAL_REG_BANK_3); //swtich bank3
    I2C_WriteOneByte(icm_slave_addr, REG_ADD_I2C_SLV1_ADDR, u8I2CAddr);
    I2C_WriteOneByte(icm_slave_addr, REG_ADD_I2C_SLV1_REG,  u8RegAddr);
    I2C_WriteOneByte(icm_slave_addr, REG_ADD_I2C_SLV1_DO,   u8data);
    I2C_WriteOneByte(icm_slave_addr, REG_ADD_I2C_SLV1_CTRL, REG_VAL_BIT_SLV0_EN | 1);

    I2C_WriteOneByte(icm_slave_addr, REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0); //swtich bank0

    u8Temp = I2C_ReadOneByte(icm_slave_addr, REG_ADD_USER_CTRL);
    u8Temp |= REG_VAL_BIT_I2C_MST_EN;
    I2C_WriteOneByte(icm_slave_addr, REG_ADD_USER_CTRL, u8Temp);
    Delay_ms(5);
    u8Temp &= ~REG_VAL_BIT_I2C_MST_EN;
    I2C_WriteOneByte(icm_slave_addr, REG_ADD_USER_CTRL, u8Temp);

    I2C_WriteOneByte(icm_slave_addr, REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_3); //swtich bank3

    u8Temp = I2C_ReadOneByte(icm_slave_addr, REG_ADD_I2C_SLV0_CTRL);
    u8Temp &= ~((REG_VAL_BIT_I2C_MST_EN) & (REG_VAL_BIT_MASK_LEN));
    I2C_WriteOneByte(icm_slave_addr, REG_ADD_I2C_SLV0_CTRL,  u8Temp);

    I2C_WriteOneByte(icm_slave_addr, REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0); //swtich bank0

    return;
}

static void invmsICM20948CalAvgValue(uint8_t *pIndex, int16_t *pAvgBuffer, int16_t InVal, int32_t *pOutVal)
{
    uint8_t i;

    *(pAvgBuffer + ((*pIndex) ++)) = InVal;
    *pIndex &= 0x07;

    *pOutVal = 0;
    for (i = 0; i < 8; i ++)
    {
        *pOutVal += *(pAvgBuffer + i);
    }
    *pOutVal >>= 3;
}

static void invmsICM20948GyroOffset(void)
{
    uint8_t i;
    int16_t s16Gx = 0, s16Gy = 0, s16Gz = 0;
    int32_t s32TempGx = 0, s32TempGy = 0, s32TempGz = 0;
    for (i = 0; i < 32; i ++)
    {
        invmsICM20948GyroRead(&s16Gx, &s16Gy, &s16Gz);
        s32TempGx += s16Gx;
        s32TempGy += s16Gy;
        s32TempGz += s16Gz;
        Delay_ms(10);
    }
    gstGyroOffset.s16X = s32TempGx >> 5;
    gstGyroOffset.s16Y = s32TempGy >> 5;
    gstGyroOffset.s16Z = s32TempGz >> 5;
    LOG_D("GyroOffset x = %d, y = %d, z = %d\n", gstGyroOffset.s16X,
          gstGyroOffset.s16Y, gstGyroOffset.s16Z);
    return;
}

void *invmsICM20948GetBus(void)
{
    return (void *)i2cbus;
}

uint8_t invmsICM20948GetDevAddr(void)
{
    return icm_slave_addr;
}

uint8_t invmsICM20948GetDevId(void)
{
    return REG_VAL_WIA;
}

void invmsICM20948SetAccelRange(uint32_t range)
{
    /* user bank 2 register */
    I2C_WriteOneByte(icm_slave_addr, REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_2);

    I2C_WriteOneByte(icm_slave_addr, REG_ADD_ACCEL_CONFIG,
                     REG_VAL_BIT_ACCEL_DLPCFG_6 | range | REG_VAL_BIT_ACCEL_DLPF);

    /* user bank 0 register */
    I2C_WriteOneByte(icm_slave_addr, REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0);
}

void invmsICM20948SetGyroRange(uint32_t range)
{
    /* user bank 2 register */
    I2C_WriteOneByte(icm_slave_addr, REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_2);
    I2C_WriteOneByte(icm_slave_addr, REG_ADD_GYRO_CONFIG_1,
                     REG_VAL_BIT_GYRO_DLPCFG_6 | range | REG_VAL_BIT_GYRO_DLPF);

    /* user bank 0 register */
    I2C_WriteOneByte(icm_slave_addr, REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0);

}

//#define DRV_ICM20948_TEST

#ifdef DRV_ICM20948_TEST
#include <string.h>
#include "counter.h"
#include "mcube_pedo_interface.h"

#define FILTER_CNTER        (8)
#define ABSX(x,y)           ((x)>(y)?(x)-(y):(y)-(x))
#define GXT             (100)
#define GYT             (100)
#define GZT             (100)
#define AXT             (1000)
#define AYT             (1000)
#define AZT             (1000)
#define MXT             (50)
#define MYT             (50)
#define MZT             (50)

// get average value after filter
static void ICM20948_get_gyro(int16_t *ps16X, int16_t *ps16Y, int16_t *ps16Z)
{
    int i;
    for (i = 0; i < FILTER_CNTER; i++)
        invmsICM20948GyroRead(ps16X, ps16Y, ps16Z);
}

static void ICM20948_get_accel(int16_t *ps16X, int16_t *ps16Y, int16_t *ps16Z)
{
    int i;
    for (i = 0; i < FILTER_CNTER; i++)
        invmsICM20948AccelRead(ps16X, ps16Y, ps16Z);
}
static void ICM20948_get_mag(int16_t *ps16X, int16_t *ps16Y, int16_t *ps16Z)
{
    int i;
    for (i = 0; i < FILTER_CNTER; i++)
        invmsICM20948MagRead(ps16X, ps16Y, ps16Z);
}


int cmd_icmt(int argc, char *argv[])
{
    int16_t x, y, z;
    float angle[3];
    int i;

    if (argc < 2)
    {
        rt_kprintf("Invalid parameter!\n");
        return 1;
    }
    if (strcmp(argv[1], "open") == 0)
    {
        bool res = invmsICM20948Init();
        if (res)
            rt_kprintf("Open icm20948 success\n");
        else
            rt_kprintf("open icm20948 fail\n");
    }
    if (strcmp(argv[1], "-r") == 0)
    {
        uint8_t deva = icm_slave_addr; // atoi(argv[2]) & 0xff;
        uint8_t rega = atoi(argv[2]) & 0xff;
        uint8_t value = I2C_ReadOneByte(deva, rega);
        rt_kprintf("Reg 0x%x value 0x%x\n", rega, value);
    }
    if (strcmp(argv[1], "-g") == 0)
    {
        x = 0;
        y = 0;
        z = 0;
        // there is an 8 value average, get the lastest one
        for (i = 0; i < FILTER_CNTER; i++)
            invmsICM20948GyroRead(&x, &y, &z);
        rt_kprintf("Get Gyro x=%d, y=%d, z=%d\n", x, y, z);
    }
    if (strcmp(argv[1], "-a") == 0)
    {
        x = 0;
        y = 0;
        z = 0;
        // there is an 8 value average, get the lastest one
        for (i = 0; i < FILTER_CNTER; i++)
            invmsICM20948AccelRead(&x, &y, &z);
        rt_kprintf("Get Accelerate X:%d, Y:%d, Z:%d\n", x, y, z);
    }
    if (strcmp(argv[1], "-m") == 0)
    {
        x = 0;
        y = 0;
        z = 0;
        // there is an 8 value average, get the lastest one
        for (i = 0; i < FILTER_CNTER; i++)
            invmsICM20948MagRead(&x, &y, &z);
        rt_kprintf("Get Magnetic x=%d, y=%d, z=%d\n", x, y, z);
    }
    if (strcmp(argv[1], "-y") == 0)
    {
        IMU_GetYawPitchRoll(angle);
        rt_kprintf("Get Roll: %.2f\n", angle[2]);
        rt_kprintf("Get Pitch: %.2f\n", angle[1]);
        rt_kprintf("Get Yaw: %.2f\n", angle[0]);
    }
    if (strcmp(argv[1], "-l") == 0)
    {
        uint32_t loop = atoi(argv[2]);  // continue check with 20hz
        int16_t gyrox = 0, gyroy = 0, gyroz = 0;
        int16_t gyrox1 = 0, gyroy1 = 0, gyroz1 = 0;
        int16_t accx = 0, accy = 0, accz = 0;
        int16_t accx1 = 0, accy1 = 0, accz1 = 0;
        int16_t magx = 0, magy = 0, magz = 0;
        int16_t magx1 = 0, magy1 = 0, magz1 = 0;
        if (loop > 36000)
            loop = 36000;
        rt_kprintf("Test %d second\n", loop / 20);

        do
        {
            ICM20948_get_gyro(&gyrox, &gyroy, &gyroz);
            ICM20948_get_accel(&accx, &accy, &accz);
            ICM20948_get_mag(&magx, &magy, &magz);
            //IMU_GetYawPitchRoll(angle);

            if (ABSX(gyrox, gyrox1) > GXT || ABSX(gyroy, gyroy1) > GYT || ABSX(gyroz, gyroz1) > GZT)
            {
                gyrox1 = gyrox;
                gyroy1 = gyroy;
                gyroz1 = gyroz;
                rt_kprintf("Get Gyro x=%d, y=%d, z=%d\n", gyrox, gyroy, gyroz);
            }
            if (ABSX(accx, accx1) > AXT || ABSX(accy, accy1) > AYT || ABSX(accz, accz1) > AZT)
            {
                accx1 = accx;
                accy1 = accy;
                accz1 = accz;
                rt_kprintf("Get Accelerate X:%d, Y:%d, Z:%d\n", accx, accy, accz);
            }
            if (ABSX(magx, magx1) > MXT || ABSX(magy, magy1) > MYT || ABSX(magz, magz1) > MZT)
            {
                magx1 = magx;
                magy1 = magy;
                magz1 = magz;
                rt_kprintf("Get Magnetic x=%d, y=%d, z=%d\n", magx, magy, magz);
            }
            loop--;
            rt_thread_delay(50);    // continue check with 20hz
        }
        while (loop > 0);
    }
#if 0
    if (strcmp(argv[1], "-p") == 0) // pedometer lib
    {
        int16_t accx = 0, accy = 0, accz = 0;
        int32_t loop = 50 * 60 * 5;
        int32_t logcnt = 0;
        unsigned long step;
        if (argc >= 3)
            loop = 50 * 60 * atoi(argv[2]);

        unsigned char res = Ped_Open();
        if (res == 0)
        {
            rt_kprintf("Ped_Open fail\n");
            return 1;
        }
        Ped_ResetStepCount();

        do
        {
            //ICM20948_get_accel(&accx, &accy, &accz);
            invmsICM20948AccelRead(&accx, &accy, &accz);
            Ped_ProcessData(accx, accy, accz);  // suppose get value mg based
            logcnt++;
            if (logcnt >= 50 * 2) // output each 2 second
            {
                logcnt = 0;
                step = Ped_GetStepCount();
                rt_kprintf("Step count %d\n", step);
            }

            loop--;
            rt_thread_delay(20);    // continue check with 50hz
        }
        while (loop > 0);
    }
#endif
    if (strcmp(argv[1], "-c") == 0) // pedometer lib-c
    {
        int16_t accx = 0, accy = 0, accz = 0;
        int32_t loop = 50 * 60 * 5;
        int32_t logcnt = 0;
        SportDataType data = {0};
        if (argc >= 3)
            loop = 50 * 60 * atoi(argv[2]);

        Sport_Init();
        Set_Parameter(175, 80);

        do
        {
            //ICM20948_get_accel(&accx, &accy, &accz);
            invmsICM20948AccelRead(&accx, &accy, &accz);
            Sport_Calculator(accx, accy, accz); // suppose parameter mg based
            logcnt++;
            if (logcnt >= 50 * 2)
            {
                logcnt = 0;
                Read_SportData(&data);
                rt_kprintf("Step count %d\n", data.steps);
            }

            loop--;
            rt_thread_delay(20);    // continue check with 50hz
        }
        while (loop > 0);
    }

    return 0;
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_icmt, __cmd_icmt, Test driver icm20948);

#endif //DRV_ICM20948_TEST

#endif  // SENSOR_USING_ICM20948

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
