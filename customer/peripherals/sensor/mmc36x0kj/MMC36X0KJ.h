/**
  ******************************************************************************
  * @file   MMC36X0KJ.h
  * @author Sifli software development team
  * @brief
* This file implement magnetic sensor driver APIs.
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

#if 0
    typedef   signed char  int8_t;      // signed 8-bit number    (-128 to +127)
    typedef unsigned char  uint8_t;     // unsigned 8-bit number  (+0 to +255)
    typedef   signed short int16_t;     // signed 16-bt number    (-32768 to +32767)
    typedef unsigned short uint16_t;    // unsigned 16-bit number (+0 to +65535)
    typedef   signed int   int32_t;     // signed 32-bt number    (-2,147,483,648 to +2,147,483,647)
    typedef unsigned int   uint32_t;    // unsigned 32-bit number (+0 to +4,294,967,295)
#endif
#define MMC36X0KJ_7BITI2C_ADDRESS   0x30

#define MMC36X0KJ_REG_DATA          0x00
#define MMC36X0KJ_REG_XL            0x00
#define MMC36X0KJ_REG_XH            0x01
#define MMC36X0KJ_REG_YL            0x02
#define MMC36X0KJ_REG_YH            0x03
#define MMC36X0KJ_REG_ZL            0x04
#define MMC36X0KJ_REG_ZH            0x05
#define MMC36X0KJ_REG_TEMP          0x06
#define MMC36X0KJ_REG_STATUS        0x07
#define MMC36X0KJ_REG_CTRL0         0x08
#define MMC36X0KJ_REG_CTRL1         0x09
#define MMC36X0KJ_REG_CTRL2         0x0A
#define MMC36X0KJ_REG_X_THD         0x0B
#define MMC36X0KJ_REG_Y_THD         0x0C
#define MMC36X0KJ_REG_Z_THD         0x0D
#define MMC36X0KJ_REG_SELFTEST      0x0E
#define MMC36X0KJ_REG_PASSWORD      0x0F
#define MMC36X0KJ_REG_OTPMODE       0x12
#define MMC36X0KJ_REG_TESTMODE      0x13
#define MMC36X0KJ_REG_SR_PWIDTH     0x20
#define MMC36X0KJ_REG_OTP           0x2A
#define MMC36X0KJ_REG_PRODUCTID     0x2F

#define MMC36X0KJ_CMD_REFILL        0x20
#define MMC36X0KJ_CMD_RESET         0x10
#define MMC36X0KJ_CMD_SET           0x08
#define MMC36X0KJ_CMD_TM_M          0x01
#define MMC36X0KJ_CMD_TM_T          0x02
#define MMC36X0KJ_CMD_START_MDT     0x04
#define MMC36X0KJ_CMD_100HZ         0x00
#define MMC36X0KJ_CMD_200HZ         0x01
#define MMC36X0KJ_CMD_400HZ         0x02
#define MMC36X0KJ_CMD_600HZ         0x03
#define MMC36X0KJ_CMD_CM_14HZ       0x01
#define MMC36X0KJ_CMD_CM_5HZ        0x02
#define MMC36X0KJ_CMD_CM_1HZ        0x04
#define MMC36X0KJ_CMD_SW_RST        0x80
#define MMC36X0KJ_CMD_PASSWORD      0xE1
#define MMC36X0KJ_CMD_OTP_OPER      0x11
#define MMC36X0KJ_CMD_OTP_MR        0x80
#define MMC36X0KJ_CMD_OTP_ACT       0x80
#define MMC36X0KJ_CMD_OTP_NACT      0x00
#define MMC36X0KJ_CMD_STSET_OPEN    0x02
#define MMC36X0KJ_CMD_STRST_OPEN    0x04
#define MMC36X0KJ_CMD_ST_CLOSE      0x00
#define MMC36X0KJ_CMD_INT_MD_EN     0x40
#define MMC36X0KJ_CMD_INT_MDT_EN    0x20

#define MMC36X0KJ_PRODUCT_ID        0x0A
#define MMC36X0KJ_OTP_READ_DONE_BIT 0x10
#define MMC36X0KJ_PUMP_ON_BIT       0x08
#define MMC36X0KJ_MDT_BIT           0x04
#define MMC36X0KJ_MEAS_T_DONE_BIT   0x02
#define MMC36X0KJ_MEAS_M_DONE_BIT   0x01

/* 16-bit mode, null field output (32768) */
#define MMC36X0KJ_OFFSET            32768
#define MMC36X0KJ_SENSITIVITY       1024
#define MMC36X0KJ_T_ZERO            (-75)
#define MMC36X0KJ_T_SENSITIVITY     0.8

/**
 * @brief Initialization
 */
int MMC36X0KJ_Initialization(void);

/**
 * @brief Enable the sensor
 */
void MMC36X0KJ_Enable(void);

/**
 * @brief Disable the sensor
 */
void MMC36X0KJ_Disable(void);

/**
 * @brief SET operation when using dual supply
 */
void MMC36X0KJ_DualPower_SET(void);

/**
 * @brief RESET operation when using dual supply
 */
void MMC36X0KJ_DualPower_RESET(void);

/**
 * @brief SET operation when using single supply
 */
void MMC36X0KJ_SinglePower_SET(void);

/**
 * @brief RESET operation when using single supply
 */
void MMC36X0KJ_SinglePower_RESET(void);

/**
 * @brief Get the temperature output
 * @param t_out[0] is the temperature, unit is degree Celsius
 */
void MMC36X0KJ_GetTemperature(float *t_out);

/**
 * @brief Get sensor data
 * @param mag_out is the magnetic field vector, unit is gauss
 */
void MMC36X0KJ_GetData(float *mag_out);
void MMC36X0KJ_Get_OrgData(int *x, int *y, int *z);
/**
 * @brief Get sensor data with SET and RESET function
 * @param mag_out is the magnetic field vector, unit is gauss
 */
void MMC36X0KJ_GetData_With_SET_RESET(float *mag_out);
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
