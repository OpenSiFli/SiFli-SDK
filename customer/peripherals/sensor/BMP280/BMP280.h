/**
  ******************************************************************************
  * @file   BMP280.h
  * @author Sifli software development team
  * @brief   Header file for BMP280.c module.
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

#ifndef __BMP280_H
#define __BMP280_H

#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

#define BMP280_RET_OK   0
#define BMP280_RET_NG   1

/*
 *  BMP280 I2c address
 */
//#define BMP280_AD0_LOW     0xEC //address pin low (GND)
//#define BMP280_AD0_HIGH    0xEE //address pin high (VCC)
#define BMP280_AD0_LOW     0x76 //address pin low (GND)
#define BMP280_AD0_HIGH    0x77 //address pin high (VCC)

/*
 *  BMP280 chip id
 */
#define BMP280_CHIP_ID          (0x58)

/*
 *  BMP280 register address
 */
#define BMP280_REGISTER_DIG_T1      0x88
#define BMP280_REGISTER_DIG_T2      0x8A
#define BMP280_REGISTER_DIG_T3      0x8C

#define BMP280_REGISTER_DIG_P1      0x8E
#define BMP280_REGISTER_DIG_P2      0x90
#define BMP280_REGISTER_DIG_P3      0x92
#define BMP280_REGISTER_DIG_P4      0x94
#define BMP280_REGISTER_DIG_P5      0x96
#define BMP280_REGISTER_DIG_P6      0x98
#define BMP280_REGISTER_DIG_P7      0x9A
#define BMP280_REGISTER_DIG_P8      0x9C
#define BMP280_REGISTER_DIG_P9      0x9E

#define BMP280_REGISTER_CHIPID      0xD0
#define BMP280_REGISTER_VERSION     0xD1
#define BMP280_REGISTER_SOFTRESET   0xE0
#define BMP280_REGISTER_STATUS      0xF3
#define BMP280_REGISTER_CONTROL     0xF4
#define BMP280_REGISTER_CONFIG      0xF5

#define BMP280_TEMP_XLSB_REG        0xFC        /*Temperature XLSB Register */
#define BMP280_TEMP_LSB_REG         0xFB        /*Temperature LSB Register  */
#define BMP280_TEMP_MSB_REG         0xFA        /*Temperature LSB Register  */
#define BMP280_PRESS_XLSB_REG       0xF9        /*Pressure XLSB  Register   */
#define BMP280_PRESS_LSB_REG        0xF8        /*Pressure LSB Register     */
#define BMP280_PRESS_MSB_REG        0xF7        /*Pressure MSB Register     */

/*calibration parameters */
#define BMP280_DIG_T1_LSB_REG                0x88
#define BMP280_DIG_T1_MSB_REG                0x89
#define BMP280_DIG_T2_LSB_REG                0x8A
#define BMP280_DIG_T2_MSB_REG                0x8B
#define BMP280_DIG_T3_LSB_REG                0x8C
#define BMP280_DIG_T3_MSB_REG                0x8D
#define BMP280_DIG_P1_LSB_REG                0x8E
#define BMP280_DIG_P1_MSB_REG                0x8F
#define BMP280_DIG_P2_LSB_REG                0x90
#define BMP280_DIG_P2_MSB_REG                0x91
#define BMP280_DIG_P3_LSB_REG                0x92
#define BMP280_DIG_P3_MSB_REG                0x93
#define BMP280_DIG_P4_LSB_REG                0x94
#define BMP280_DIG_P4_MSB_REG                0x95
#define BMP280_DIG_P5_LSB_REG                0x96
#define BMP280_DIG_P5_MSB_REG                0x97
#define BMP280_DIG_P6_LSB_REG                0x98
#define BMP280_DIG_P6_MSB_REG                0x99
#define BMP280_DIG_P7_LSB_REG                0x9A
#define BMP280_DIG_P7_MSB_REG                0x9B
#define BMP280_DIG_P8_LSB_REG                0x9C
#define BMP280_DIG_P8_MSB_REG                0x9D
#define BMP280_DIG_P9_LSB_REG                0x9E
#define BMP280_DIG_P9_MSB_REG                0x9F

typedef struct
{
    uint16_t T1;        /*<calibration T1 data*/
    int16_t T2;         /*<calibration T2 data*/
    int16_t T3;         /*<calibration T3 data*/
    uint16_t P1;        /*<calibration P1 data*/
    int16_t P2;         /*<calibration P2 data*/
    int16_t P3;         /*<calibration P3 data*/
    int16_t P4;         /*<calibration P4 data*/
    int16_t P5;         /*<calibration P5 data*/
    int16_t P6;         /*<calibration P6 data*/
    int16_t P7;         /*<calibration P7 data*/
    int16_t P8;         /*<calibration P8 data*/
    int16_t P9;         /*<calibration P9 data*/
    int32_t T_fine; /*<calibration t_fine data*/
} BMP280_HandleTypeDef;

typedef struct
{
    uint8_t Index;
    int32_t AvgBuffer[8];
} BMP280_AvgTypeDef;

#define MSLP     101325          // Mean Sea Level Pressure = 1013.25 hPA (1hPa = 100Pa = 1mbar)

/*
 *  extern interface
 */
extern uint8_t BMP280_Init(void);
extern void BMP280_CalTemperatureAndPressureAndAltitude(int32_t *temperature, int32_t *pressure, int32_t *Altitude);
void *BMP280GetBus(void);
uint8_t BMP280GetDevAddr(void);
uint8_t BMP280GetDevId(void);

void BMP280_open(void);
void BMP280_close(void);

int BMP280_SelfCheck(void);

#ifdef __cplusplus
}
#endif


#endif /* __BMP280_H */

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
