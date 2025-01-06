/**
  ******************************************************************************
  * @file   BMP280.c
  * @author Sifli software development team
  * @brief   This file includes the BMP280 driver functions
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

#include "BMP280.h"
#include <rtthread.h>
#include <math.h>
#include "stdlib.h"
#include "board.h"

#define DRV_DEBUG
#define LOG_TAG              "drv.bmp"
#include <drv_log.h>


#ifdef SENSOR_USING_BMP280

#define dig_T1 bmp280.T1
#define dig_T2 bmp280.T2
#define dig_T3 bmp280.T3
#define dig_P1 bmp280.P1
#define dig_P2 bmp280.P2
#define dig_P3 bmp280.P3
#define dig_P4 bmp280.P4
#define dig_P5 bmp280.P5
#define dig_P6 bmp280.P6
#define dig_P7 bmp280.P7
#define dig_P8 bmp280.P8
#define dig_P9 bmp280.P9
#define t_fine bmp280.T_fine

BMP280_HandleTypeDef bmp280;
int32_t gs32Pressure0 = MSLP;
static struct rt_i2c_bus_device *i2cbus = NULL;
uint8_t bmp_dev_addr = BMP280_AD0_LOW;
static int32_t press_offset = 0;

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

void BMP280_ReadReg(uint8_t RegAddr, uint8_t Num, uint8_t *pBuffer)
{
#ifdef RT_USING_I2C
    struct rt_i2c_msg msgs[2];
    uint32_t res;

    if (i2cbus)
    {
        msgs[0].addr  = bmp_dev_addr;    /* Slave address */
        msgs[0].flags = RT_I2C_WR;        /* Write flag */
        msgs[0].buf   = &RegAddr;         /* Slave register address */
        msgs[0].len   = 1;                /* Number of bytes sent */

        msgs[1].addr  = bmp_dev_addr;    /* Slave address */
        msgs[1].flags = RT_I2C_RD;        /* Read flag */
        msgs[1].buf   = pBuffer;             /* Read data pointer */
        msgs[1].len   = Num;              /* Number of bytes read */

        res = rt_i2c_transfer(i2cbus, msgs, 2);
        if (res == 2)
        {
            //LOG_D("BMP280_ReadReg OK\n");
        }
        else
        {
            LOG_D("BMP280_ReadReg FAIL %d\n", res);
        }

    }
#endif
}

void BMP280_WriteReg(uint8_t RegAddr, uint8_t Val)
{
    I2C_WriteOneByte(bmp_dev_addr, RegAddr, Val);
}

void BMP280_Read_Calibration(void)
{
    uint8_t lsb, msb;

    /* read the temperature calibration parameters */
    BMP280_ReadReg(BMP280_DIG_T1_LSB_REG, 1, &lsb);
    BMP280_ReadReg(BMP280_DIG_T1_MSB_REG, 1, &msb);
    dig_T1 = msb << 8 | lsb;
    BMP280_ReadReg(BMP280_DIG_T2_LSB_REG, 1, &lsb);
    BMP280_ReadReg(BMP280_DIG_T2_MSB_REG, 1, &msb);
    dig_T2 = msb << 8 | lsb;
    BMP280_ReadReg(BMP280_DIG_T3_LSB_REG, 1, &lsb);
    BMP280_ReadReg(BMP280_DIG_T3_MSB_REG, 1, &msb);
    dig_T3 = msb << 8 | lsb;

    /* read the pressure calibration parameters */
    BMP280_ReadReg(BMP280_DIG_P1_LSB_REG, 1, &lsb);
    BMP280_ReadReg(BMP280_DIG_P1_MSB_REG, 1, &msb);
    dig_P1 = msb << 8 | lsb;
    BMP280_ReadReg(BMP280_DIG_P2_LSB_REG, 1, &lsb);
    BMP280_ReadReg(BMP280_DIG_P2_MSB_REG, 1, &msb);
    dig_P2 = msb << 8 | lsb;
    BMP280_ReadReg(BMP280_DIG_P3_LSB_REG, 1, &lsb);
    BMP280_ReadReg(BMP280_DIG_P3_MSB_REG, 1, &msb);
    dig_P3 = msb << 8 | lsb;
    BMP280_ReadReg(BMP280_DIG_P4_LSB_REG, 1, &lsb);
    BMP280_ReadReg(BMP280_DIG_P4_MSB_REG, 1, &msb);
    dig_P4 = msb << 8 | lsb;
    BMP280_ReadReg(BMP280_DIG_P5_LSB_REG, 1, &lsb);
    BMP280_ReadReg(BMP280_DIG_P5_MSB_REG, 1, &msb);
    dig_P5 = msb << 8 | lsb;
    BMP280_ReadReg(BMP280_DIG_P6_LSB_REG, 1, &lsb);
    BMP280_ReadReg(BMP280_DIG_P6_MSB_REG, 1, &msb);
    dig_P6 = msb << 8 | lsb;
    BMP280_ReadReg(BMP280_DIG_P7_LSB_REG, 1, &lsb);
    BMP280_ReadReg(BMP280_DIG_P7_MSB_REG, 1, &msb);
    dig_P7 = msb << 8 | lsb;
    BMP280_ReadReg(BMP280_DIG_P8_LSB_REG, 1, &lsb);
    BMP280_ReadReg(BMP280_DIG_P8_MSB_REG, 1, &msb);
    dig_P8 = msb << 8 | lsb;
    BMP280_ReadReg(BMP280_DIG_P9_LSB_REG, 1, &lsb);
    BMP280_ReadReg(BMP280_DIG_P9_MSB_REG, 1, &msb);
    dig_P9 = msb << 8 | lsb;

//  LOG_D("dig_T1 = %d\r\n",dig_T1);
//  LOG_D("dig_T2 = %d\r\n",dig_T2);
//  LOG_D("dig_T3 = %d\r\n",dig_T3);
//  LOG_D("dig_P1 = %d\r\n",dig_P1);
//  LOG_D("dig_P2 = %d\r\n",dig_P2);
//  LOG_D("dig_P3 = %d\r\n",dig_P3);
//  LOG_D("dig_P4 = %d\r\n",dig_P4);
//  LOG_D("dig_P5 = %d\r\n",dig_P5);
//  LOG_D("dig_P6 = %d\r\n",dig_P6);
//  LOG_D("dig_P7 = %d\r\n",dig_P7);
//  LOG_D("dig_P8 = %d\r\n",dig_P8);
//  LOG_D("dig_P9 = %d\r\n",dig_P9);

}
/* Returns temperature in DegC, double precision. Output value of "1.23"equals 51.23 DegC. */
double BMP280_Compensate_Temperature(int32_t adc_T)
{
    double var1, var2, temperature;
    var1 = (((double) adc_T) / 16384.0 - ((double) dig_T1) / 1024.0) * ((double) dig_T2);
    var2 = ((((double) adc_T) / 131072.0 - ((double) dig_T1) / 8192.0)  * (((double) adc_T) / 131072.0
            - ((double) dig_T1) / 8192.0)) * ((double) dig_T3);
    t_fine = (int32_t)(var1 + var2);
    temperature = (var1 + var2) / 5120.0;

    return temperature;
}


/* Returns pressure in Pa as double. Output value of "6386.2"equals 96386.2 Pa = 963.862 hPa */
double BMP280_Compensate_Pressure(int32_t adc_P)
{
    double var1, var2, pressure;

    var1 = ((double)t_fine / 2.0) - 64000.0;
    var2 = var1 * var1 * ((double) dig_P6) / 32768.0;
    var2 = var2 + var1 * ((double) dig_P5) * 2.0;
    var2 = (var2 / 4.0) + (((double) dig_P4) * 65536.0);
    var1 = (((double) dig_P3) * var1 * var1 / 524288.0  + ((double) dig_P2) * var1) / 524288.0;
    var1 = (1.0 + var1 / 32768.0) * ((double) dig_P1);

    if (var1 == 0.0)
    {
        return 0; // avoid exception caused by division by zero
    }

    pressure = 1048576.0 - (double) adc_P;
    pressure = (pressure - (var2 / 4096.0)) * 6250.0 / var1;
    var1 = ((double) dig_P9) * pressure * pressure / 2147483648.0;
    var2 = pressure * ((double) dig_P8) / 32768.0;
    pressure = pressure + (var1 + var2 + ((double) dig_P7)) / 16.0;

    return pressure;
}
void BMP280_Get_Temperature_And_Pressure(double *temperature, double *pressure)
{
    uint8_t lsb, msb, xlsb;
    int32_t adc_P, adc_T;

    BMP280_ReadReg(BMP280_TEMP_XLSB_REG, 1, &xlsb);
    BMP280_ReadReg(BMP280_TEMP_LSB_REG,  1, &lsb);
    BMP280_ReadReg(BMP280_TEMP_MSB_REG,  1, &msb);
    adc_T = (msb << 12) | (lsb << 4) | (xlsb >> 4);
    //adc_T = 415148;
    * temperature = BMP280_Compensate_Temperature(adc_T);

    BMP280_ReadReg(BMP280_PRESS_XLSB_REG, 1, &xlsb);
    BMP280_ReadReg(BMP280_PRESS_LSB_REG,  1, &lsb);
    BMP280_ReadReg(BMP280_PRESS_MSB_REG,  1, &msb);
    adc_P = (msb << 12) | (lsb << 4) | (xlsb >> 4);
    //adc_P = 51988;
    * pressure = BMP280_Compensate_Pressure(adc_P);
}

void BMP280_CalAvgValue(uint8_t *pIndex, int32_t *pAvgBuffer, int32_t InVal, int32_t *pOutVal)
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

void BMP280_CalculateAbsoluteAltitude(int32_t *pAltitude, int32_t PressureVal)
{
    float value = pow((PressureVal / (float)gs32Pressure0), 0.1903);
    *pAltitude = 4430000 * (1 - value);
    //LOG_D("PressureVal = %d, gs32Pressure0 = %d\n",PressureVal,gs32Pressure0);
    //LOG_D("Power value = %.6f\n",value);
}

extern uint8_t BMP280_Init()
{
    uint8_t u8Ret = BMP280_RET_OK;
    uint8_t u8ChipID, u8CtrlMod, u8Status;

    /* get i2c bus device */
    i2cbus = rt_i2c_bus_device_find(BMP280_I2C_BUS);
    if (i2cbus)
    {
        LOG_D("Find i2c bus device %s\n", BMP280_I2C_BUS);
        rt_i2c_open(i2cbus, RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_INT_TX);
    }
    else
    {
        LOG_E("Can not found i2c bus %s, BMP280_Init fail\n", BMP280_I2C_BUS);
        return BMP280_RET_NG;
    }

    /* check bmp280 slave address */
    bmp_dev_addr = BMP280_AD0_LOW;
    u8ChipID = 0;
    BMP280_ReadReg(BMP280_REGISTER_CHIPID, 1, &u8ChipID);
    if (u8ChipID != 0x58)
    {
        bmp_dev_addr = BMP280_AD0_HIGH;
        u8ChipID = 0;
        BMP280_ReadReg(BMP280_REGISTER_CHIPID, 1, &u8ChipID);
    }
    if (u8ChipID == BMP280_CHIP_ID)
    {
        LOG_D("BMP280 I2C slave address = 0x%x\n", bmp_dev_addr);
    }

    BMP280_ReadReg(BMP280_REGISTER_CONTROL, 1, &u8CtrlMod);
    BMP280_ReadReg(BMP280_REGISTER_STATUS, 1, &u8Status);

    if (u8ChipID == BMP280_CHIP_ID)
    {
        LOG_D("BMP280 init successful : ChipID [0x%x] CtrlMod [0x%x] Status [0x%x] \r\n", u8ChipID, u8CtrlMod, u8Status);
        u8Ret = BMP280_RET_OK;
    }
    else
    {
        LOG_D("BMP280 init fail\n");
        u8Ret = BMP280_RET_NG;
    }
    return  u8Ret;
}

void BMP280_open(void)
{
    BMP280_WriteReg(BMP280_REGISTER_CONTROL, 0xFF);
    BMP280_WriteReg(BMP280_REGISTER_CONFIG, 0x14);
    BMP280_Read_Calibration();
}

void BMP280_close(void)
{
    BMP280_WriteReg(BMP280_REGISTER_CONTROL, 0);
}

extern void BMP280_CalTemperatureAndPressureAndAltitude(int32_t *temperature, int32_t *pressure, int32_t *Altitude)
{
    double CurPressure, CurTemperature;
    int32_t CurAltitude;
    static BMP280_AvgTypeDef BMP280_Filter[3];

    BMP280_Get_Temperature_And_Pressure(&CurTemperature, &CurPressure);
    BMP280_CalAvgValue(&BMP280_Filter[0].Index, BMP280_Filter[0].AvgBuffer, (int32_t)(CurPressure), pressure);

    // add a offset to correct test
    *pressure -= press_offset;
    BMP280_CalculateAbsoluteAltitude(&CurAltitude, (*pressure));
    BMP280_CalAvgValue(&BMP280_Filter[1].Index, BMP280_Filter[1].AvgBuffer, CurAltitude, Altitude);
    BMP280_CalAvgValue(&BMP280_Filter[2].Index, BMP280_Filter[2].AvgBuffer, (int32_t)CurTemperature * 10, temperature);

    return;
}

void *BMP280GetBus(void)
{
    return (void *)i2cbus;
}

uint8_t BMP280GetDevAddr(void)
{
    return bmp_dev_addr;
}

uint8_t BMP280GetDevId(void)
{
    return BMP280_CHIP_ID;
}

int BMP280_SelfCheck(void)
{
    uint8_t u8ChipID = 0;

    BMP280_ReadReg(BMP280_REGISTER_CHIPID, 1, &u8ChipID);
    if (u8ChipID != BMP280_CHIP_ID)
        return -1;

    return 0;
}

#define DRV_BMP280_TEST

#ifdef DRV_BMP280_TEST
#include <string.h>

int cmd_bmpt(int argc, char *argv[])
{
    int32_t temp, pres, alti;
    if (argc < 2)
    {
        LOG_I("Invalid parameter!\n");
        return 1;
    }
    if (strcmp(argv[1], "-open") == 0)
    {
        HAL_PIN_Set(PAD_PB25, GPIO_B25, PIN_PULLUP, 0);
        rt_thread_delay(2); // ADD a delay to make sure SENSOR BOARD power up
        uint8_t res = BMP280_Init();
        if (BMP280_RET_OK == res)
        {
            BMP280_open();
            LOG_I("Open bmp280 success\n");
        }
        else
            LOG_I("open bmp280 fail\n");
    }
    if (strcmp(argv[1], "-close") == 0)
    {
        BMP280_close();
        LOG_I("BMP280 closed\n");
    }
    if (strcmp(argv[1], "-r") == 0)
    {
        uint8_t rega = atoi(argv[2]) & 0xff;
        uint8_t value;
        BMP280_ReadReg(rega, 1, &value);
        LOG_I("Reg 0x%x value 0x%x\n", rega, value);
    }
    if (strcmp(argv[1], "-tpa") == 0)
    {
        temp = 0;
        pres = 0;
        alti = 0;
        BMP280_CalTemperatureAndPressureAndAltitude(&temp, &pres, &alti);
        LOG_I("Get temperature = %.1f\n", (float)temp / 10);
        LOG_I("Get pressure= %.2f\n", (float)pres / 100);
        LOG_I("Get altitude= %.2f\n", (float)alti / 100);
    }
    if (strcmp(argv[1], "-bps") == 0)
    {
        struct rt_i2c_configuration cfg;
        int bps = atoi(argv[2]);
        cfg.addr = 0;
        cfg.max_hz = bps;
        cfg.mode = 0;
        cfg.timeout = 5000;
        rt_i2c_configure(i2cbus, &cfg);
        LOG_I("Config BMP I2C speed to %d\n", bps);
    }
    if (strcmp(argv[1], "-l") == 0)
    {
        temp = 0;
        pres = 0;
        alti = 0;
        uint32_t loop = atoi(argv[2]);  // continue check with 10hz
        int32_t prev1 = 0, prev2 = 0, prev3 = 0;
        if (loop > 36000)
            loop = 36000;

        do
        {
            BMP280_CalTemperatureAndPressureAndAltitude(&temp, &pres, &alti);
            if (prev1 != temp || prev2 != pres || prev3 != alti)
            {
                prev1 = temp;
                prev2 = pres;
                prev3 = alti;
                LOG_I("Get temperature = %.1f\n", (float)temp / 10);
                LOG_I("Get pressure= %.2f\n", (float)pres / 100);
                LOG_I("Get altitude= %.2f\n", (float)alti / 100);
            }
            loop--;
            rt_thread_delay(100);
        }
        while (loop > 0);
    }

    return 0;
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_bmpt, __cmd_bmpt, Test driver bmp280);

#endif //DRV_BMP280_TEST


#endif  // SENSOR_USING_BMP280

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
