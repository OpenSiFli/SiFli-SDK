/**
  ******************************************************************************
  * @file   MMC36X0KJ.c
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

#include "MMC36X0KJ.h"
#include "MMC36X_Customer.h"

#define MMC36X_INT_GPIO_BIT         (9)

/* Default sensor otp compensation matrix */
static float fOtpMatrix[3] = {1.0f, 1.0f, 1.35f};

static int posX, posY, posZ;

/* Function declaration */

/**
 * @brief Reset the sensor by software
 */
void MMC36X0KJ_Software_Reset(void);

/**
 * @brief OTP read done check
 */
int MMC36X0KJ_Check_OTP(void);

/**
 * @brief Check Product ID
 */
int MMC36X0KJ_CheckID(void);

/**
 * @brief Get the sensitivity compensation value
 */
void MMC36X0KJ_GetCompMatrix(void);

/**
 * @brief Change the SET/RESET pulse width
 */
void MMC36X0KJ_SetPulseWidth(void);

/**
 * @brief Set the output resolution
 */
void MMC36X0KJ_SetOutputResolution(unsigned char res);

/**
 * @brief Enable the meas_done interrupt
 */
void MMC36X0KJ_INT_Meas_Done_Enable(void);

/**
 * @brief Enable the MDT interrupt
 */
void MMC36X0KJ_INT_MDT_Enable(void);

/**
 * @brief Clear Meas_T_Done interrupt
 */
void MMC36X0KJ_INT_Meas_T_Done_Clear(void);

/**
 * @brief Clear Meas_M_Done interrupt
 */
void MMC36X0KJ_INT_Meas_M_Done_Clear(void);

/**
 * @brief Clear MDT interrupt
 */
void MMC36X0KJ_INT_MDT_Clear(void);

/*********************************************************************************
* decription: Reset the sensor by software
*********************************************************************************/
void MMC36X0KJ_Software_Reset(void)
{
    /* Write 0x80 to register 0x09, set SW_RST bit high */
    I2C_Write_Reg(MMC36X0KJ_7BITI2C_ADDRESS, MMC36X0KJ_REG_CTRL1, MMC36X0KJ_CMD_SW_RST);

    /* Delay at least 5ms to finish the software reset operation */
    Delay_Ms(50);

    return;
}

/*********************************************************************************
* decription: OTP read done check
*********************************************************************************/
int MMC36X0KJ_Check_OTP(void)
{
    unsigned char reg_val = 0;

    Delay_Ms(5);
    /* Read register 0x07, check OTP_Read_Done bit */
    I2C_Read_Reg(MMC36X0KJ_7BITI2C_ADDRESS, MMC36X0KJ_REG_STATUS, &reg_val);
    if ((reg_val & 0x10) != MMC36X0KJ_OTP_READ_DONE_BIT)
        return -1;

    return 1;
}

/*********************************************************************************
* decription: Check Product ID
*********************************************************************************/
int MMC36X0KJ_CheckID(void)
{
    unsigned char pro_id = 0;

    /* Read register 0x2F, check product ID */
    I2C_Read_Reg(MMC36X0KJ_7BITI2C_ADDRESS, MMC36X0KJ_REG_PRODUCTID, &pro_id);
    if (pro_id != MMC36X0KJ_PRODUCT_ID)
        return -1;

    return 1;
}

/*********************************************************************************
* decription: Read the sensitivity compensation registers 0x2A and 0x2B
*********************************************************************************/
void MMC36X0KJ_GetCompMatrix(void)
{
    uint8_t reg_data[2] = {0};
    uint8_t reg_temp = 0;
    int8_t otp_temp = 0;

    /* Write 0xE1 to register 0x0F, write password */
    I2C_Write_Reg(MMC36X0KJ_7BITI2C_ADDRESS, MMC36X0KJ_REG_PASSWORD, MMC36X0KJ_CMD_PASSWORD);

    /* Write 0x11 to register 0x12 */
    I2C_Write_Reg(MMC36X0KJ_7BITI2C_ADDRESS, MMC36X0KJ_REG_OTPMODE, MMC36X0KJ_CMD_OTP_OPER);

    /* Write 0x80 to register 0x13 */
    I2C_Write_Reg(MMC36X0KJ_7BITI2C_ADDRESS, MMC36X0KJ_REG_TESTMODE, MMC36X0KJ_CMD_OTP_MR);

    /* Write 0x80 to register 0x0A, set ULP_SEL '1' */
    I2C_Write_Reg(MMC36X0KJ_7BITI2C_ADDRESS, MMC36X0KJ_REG_CTRL2, MMC36X0KJ_CMD_OTP_ACT);

    /* Read register 0x2A and 0x2B */
    I2C_MultiRead_Reg(MMC36X0KJ_7BITI2C_ADDRESS, MMC36X0KJ_REG_OTP, 2, reg_data);

    /* Write 0x00 to register 0x0A, set ULP_SEL '0' */
    I2C_Write_Reg(MMC36X0KJ_7BITI2C_ADDRESS, MMC36X0KJ_REG_CTRL2, MMC36X0KJ_CMD_OTP_NACT);

    /* Get sensitivity compensation value */
    fOtpMatrix[0] = 1.0f;

    reg_temp = reg_data[0] & 0x3f;
    if (reg_temp >= 32)
        otp_temp = 32 - reg_temp;
    else
        otp_temp = reg_temp;
    fOtpMatrix[1] = (float)otp_temp * 0.006f + 1.0f;

    reg_temp = (reg_data[1] & 0x0f) << 2 | (reg_data[0] & 0xc0) >> 6;
    if (reg_temp >= 32)
        otp_temp = 32 - reg_temp;
    else
        otp_temp = reg_temp;
    fOtpMatrix[2] = ((float)otp_temp * 0.006f + 1.0f) * 1.35f;

    return;
}

/*********************************************************************************
* decription: Change the SET/RESET pulse width
*********************************************************************************/
void MMC36X0KJ_SetPulseWidth(void)
{
    unsigned char reg_val;

    /* Write 0x00 to register 0x0A, set ULP_SEL bit low */
    I2C_Write_Reg(MMC36X0KJ_7BITI2C_ADDRESS, MMC36X0KJ_REG_CTRL2, MMC36X0KJ_CMD_OTP_NACT);

    /* Write 0xE1 to register 0x0F, write password */
    I2C_Write_Reg(MMC36X0KJ_7BITI2C_ADDRESS, MMC36X0KJ_REG_PASSWORD, MMC36X0KJ_CMD_PASSWORD);

    /* Read and write register 0x20, set SR_PW<1:0> = 00 = 1us */
    I2C_Read_Reg(MMC36X0KJ_7BITI2C_ADDRESS, MMC36X0KJ_REG_SR_PWIDTH, &reg_val);
    reg_val = reg_val & 0xE7;
    I2C_Write_Reg(MMC36X0KJ_7BITI2C_ADDRESS, MMC36X0KJ_REG_SR_PWIDTH, reg_val);

    return;
}

/*********************************************************************************
* decription: SET operation when using dual supply
*********************************************************************************/
void MMC36X0KJ_DualPower_SET(void)
{
    /* Write 0x08 to register 0x08, set SET bit high */
    I2C_Write_Reg(MMC36X0KJ_7BITI2C_ADDRESS, MMC36X0KJ_REG_CTRL0, MMC36X0KJ_CMD_SET);

    /* Delay to finish the SET operation */
    Delay_Ms(1);

    return;
}

/*********************************************************************************
* decription: RESET operation when using dual supply
*********************************************************************************/
void MMC36X0KJ_DualPower_RESET(void)
{
    /* Write 0x10 to register 0x08, set RESET bit high */
    I2C_Write_Reg(MMC36X0KJ_7BITI2C_ADDRESS, MMC36X0KJ_REG_CTRL0, MMC36X0KJ_CMD_RESET);

    /* Delay to finish the RESET operation */
    Delay_Ms(1);

    return;
}

/*********************************************************************************
* decription:  SET when using single supply
*********************************************************************************/
void MMC36X0KJ_SinglePower_SET(void)
{
    unsigned char reg_val = 0;

    /* Write 0x20 to register 0x08, set Refill Cap bit high */
    I2C_Write_Reg(MMC36X0KJ_7BITI2C_ADDRESS, MMC36X0KJ_REG_CTRL0, MMC36X0KJ_CMD_REFILL);

    /* Read register 0x07, check Pump On bit */
    I2C_Read_Reg(MMC36X0KJ_7BITI2C_ADDRESS, MMC36X0KJ_REG_STATUS, &reg_val);
    while ((reg_val & MMC36X0KJ_PUMP_ON_BIT) == MMC36X0KJ_PUMP_ON_BIT)
    {
        Delay_Ms(1);
        I2C_Read_Reg(MMC36X0KJ_7BITI2C_ADDRESS, MMC36X0KJ_REG_STATUS, &reg_val);
    }

    /* Write 0x08 to register 0x08, set Set bit high */
    I2C_Write_Reg(MMC36X0KJ_7BITI2C_ADDRESS, MMC36X0KJ_REG_CTRL0, MMC36X0KJ_CMD_SET);

    /* Delay to finish the SET operation */
    Delay_Ms(1);

    return;
}

/*********************************************************************************
* decription:  RESET when using single supply
*********************************************************************************/
void MMC36X0KJ_SinglePower_RESET(void)
{
    unsigned char reg_val = 0;

    /* Write 0x20 to register 0x08, set Refill Cap bit high */
    I2C_Write_Reg(MMC36X0KJ_7BITI2C_ADDRESS, MMC36X0KJ_REG_CTRL0, MMC36X0KJ_CMD_REFILL);

    /* Read register 0x07, check Pump On bit */
    I2C_Read_Reg(MMC36X0KJ_7BITI2C_ADDRESS, MMC36X0KJ_REG_STATUS, &reg_val);
    while ((reg_val & MMC36X0KJ_PUMP_ON_BIT) == MMC36X0KJ_PUMP_ON_BIT)
    {
        Delay_Ms(1);
        I2C_Read_Reg(MMC36X0KJ_7BITI2C_ADDRESS, MMC36X0KJ_REG_STATUS, &reg_val);
    }

    /* Write 0x10 to register 0x08, set Reset bit high */
    I2C_Write_Reg(MMC36X0KJ_7BITI2C_ADDRESS, MMC36X0KJ_REG_CTRL0, MMC36X0KJ_CMD_RESET);

    /* Delay to finish the RESET operation */
    Delay_Ms(1);

    return;
}

/*********************************************************************************
* decription: Set the output resolution
*********************************************************************************/
void MMC36X0KJ_SetOutputResolution(unsigned char res)
{
    /* Write register 0x09, Set BW<1:0> = 0x00, 0x01, 0x02, or 0x03 */
    I2C_Write_Reg(MMC36X0KJ_7BITI2C_ADDRESS, MMC36X0KJ_REG_CTRL1, res);

    return;
}

/*********************************************************************************
* decription: Enable the int when a mag or temp measuremet event is completed
*********************************************************************************/
void MMC36X0KJ_INT_Meas_Done_Enable(void)
{
    /* Write register 0x0A, Set INT_Meas_Done_EN high */
    I2C_Write_Reg(MMC36X0KJ_7BITI2C_ADDRESS, MMC36X0KJ_REG_CTRL2, MMC36X0KJ_CMD_INT_MD_EN);

    return;
}

/*********************************************************************************
* decription: Enable the int when a motion is detected
*********************************************************************************/
void MMC36X0KJ_INT_MDT_Enable(void)
{
    /* Step size is 4mG, threshold = mdt_threshold*4mG */
    unsigned char mdt_threshold = 0x0E;

    /* Write register 0x0B, Set X, Y and Z threshold */
    I2C_Write_Reg(MMC36X0KJ_7BITI2C_ADDRESS, MMC36X0KJ_REG_X_THD, mdt_threshold);
    I2C_Write_Reg(MMC36X0KJ_7BITI2C_ADDRESS, MMC36X0KJ_REG_Y_THD, mdt_threshold);
    I2C_Write_Reg(MMC36X0KJ_7BITI2C_ADDRESS, MMC36X0KJ_REG_Z_THD, mdt_threshold);

    /* Write register 0x0A, Set CM_Freq [3:0], Set INT_MDT_EN high */
    I2C_Write_Reg(MMC36X0KJ_7BITI2C_ADDRESS, MMC36X0KJ_REG_CTRL2, MMC36X0KJ_CMD_CM_14HZ | MMC36X0KJ_CMD_INT_MDT_EN);

    /* Write register 0x08, Set Start_MDT high, Start the motion detector */
    I2C_Write_Reg(MMC36X0KJ_7BITI2C_ADDRESS, MMC36X0KJ_REG_CTRL0, MMC36X0KJ_CMD_START_MDT);

    return;
}

/*********************************************************************************
* decription: Clear Meas_T_Done interrupt
*********************************************************************************/
void MMC36X0KJ_INT_Meas_T_Done_Clear(void)
{
    /* Write register 0x07, Set Meas_T_Done bit high, clear Meas_T_Done interrupt */
    I2C_Write_Reg(MMC36X0KJ_7BITI2C_ADDRESS, MMC36X0KJ_REG_STATUS, MMC36X0KJ_MEAS_T_DONE_BIT);

    return;
}

/*********************************************************************************
* decription: Clear Meas_M_Done interrupt
*********************************************************************************/
void MMC36X0KJ_INT_Meas_M_Done_Clear(void)
{
    /* Write register 0x07, Set Meas_M_Done bit high, clear Meas_M_Done interrupt */
    I2C_Write_Reg(MMC36X0KJ_7BITI2C_ADDRESS, MMC36X0KJ_REG_STATUS, MMC36X0KJ_MEAS_M_DONE_BIT);

    return;
}

/*********************************************************************************
* decription: Clear MDT interrupt
*********************************************************************************/
void MMC36X0KJ_INT_MDT_Clear(void)
{
    /* Write register 0x07, Set Motion Detected bit high, clear MDT interrupt */
    I2C_Write_Reg(MMC36X0KJ_7BITI2C_ADDRESS, MMC36X0KJ_REG_STATUS, MMC36X0KJ_MDT_BIT);

    return;
}

/*********************************************************************************
* decription: Initial the sensor when power on
*********************************************************************************/
int MMC36X0KJ_Initialization(void)
{
    int ret = 0;
    posX = 0xffffffff;
    posY = 0xffffffff;
    posZ = 0xffffffff;

    /* Initial I2C BUS */
    ret = MMC36X_I2C_Init();
    if (ret < 0)
        return ret;

    /* Check OTP Read status */
    ret = MMC36X0KJ_Check_OTP();
    if (ret < 0)
        return ret;

    /* Check product ID */
    ret = MMC36X0KJ_CheckID();
    if (ret < 0)
        return ret;

    /* Get sensitivity compensation value */
    MMC36X0KJ_GetCompMatrix();

    /* Change the SET/RESET pulse width to 1us */
    MMC36X0KJ_SetPulseWidth();

    /* SET operation when using dual power supply */
    //MMC36X0KJ_DualPower_SET();
    MMC36X0KJ_SinglePower_SET();

    /* Set output resolution */
    MMC36X0KJ_SetOutputResolution(MMC36X0KJ_CMD_100HZ);

    return 1;
}
/*********************************************************************************
* decription: Enable sensor when from pown down mode to normal mode
*********************************************************************************/
void MMC36X0KJ_Enable(void)
{
    /* Write 0x01 to register 0x08, set TM_M bit high */
    I2C_Write_Reg(MMC36X0KJ_7BITI2C_ADDRESS, MMC36X0KJ_REG_CTRL0, MMC36X0KJ_CMD_TM_M);
    Delay_Ms(10);

    return;
}
/*********************************************************************************
* decription: Disable sensor
*********************************************************************************/
void MMC36X0KJ_Disable(void)
{
    return;
}

/*********************************************************************************
* decription: Read the temperature output
*********************************************************************************/
void MMC36X0KJ_GetTemperature(float *t_out)
{
    uint8_t reg_status = 0;
    uint8_t reg_t = 0;

    /* Write 0x02 to register 0x08, set TM_T bit high */
    I2C_Write_Reg(MMC36X0KJ_7BITI2C_ADDRESS, MMC36X0KJ_REG_CTRL0, MMC36X0KJ_CMD_TM_T);
    Delay_Ms(1);

    /* Read register 0x07, check Meas_T_Done bit */
    I2C_Read_Reg(MMC36X0KJ_7BITI2C_ADDRESS, MMC36X0KJ_REG_STATUS, &reg_status);
    while ((reg_status & 0x02) != 0x02)
    {
        Delay_Ms(1);
        I2C_Read_Reg(MMC36X0KJ_7BITI2C_ADDRESS, MMC36X0KJ_REG_STATUS, &reg_status);
    }

    /* Read register 0x06 */
    I2C_Read_Reg(MMC36X0KJ_7BITI2C_ADDRESS, MMC36X0KJ_REG_TEMP, &reg_t);

    /* The temperature output has not been calibrated, can not present the ambient temperature*/
    t_out[0] = (float)reg_t * MMC36X0KJ_T_SENSITIVITY + MMC36X0KJ_T_ZERO; //unit is degree Celsius

    return;
}

/*********************************************************************************
* decription: Read the data register and convert to magnetic field vector
*********************************************************************************/
void MMC36X0KJ_GetData(float *mag_out)
{
    uint8_t reg_status = 0;

    uint8_t data_reg[6] = {0};
    uint16_t data_temp[3] = {0};
    uint16_t mag_rawdata[3] = {0};

    float mag_temp[3] = {0};

    /* Write 0x01 to register 0x08, set TM_M bit high */
    I2C_Write_Reg(MMC36X0KJ_7BITI2C_ADDRESS, MMC36X0KJ_REG_CTRL0, MMC36X0KJ_CMD_TM_M);

    /* Read register 0x07, check Meas_M_Done bit */
    I2C_Read_Reg(MMC36X0KJ_7BITI2C_ADDRESS, MMC36X0KJ_REG_STATUS, &reg_status);
    while ((reg_status & 0x01) != 0x01)
    {
        Delay_Ms(1);
        I2C_Read_Reg(MMC36X0KJ_7BITI2C_ADDRESS, MMC36X0KJ_REG_STATUS, &reg_status);
    }

    /* Read register 0x00-0x05 */
    I2C_MultiRead_Reg(MMC36X0KJ_7BITI2C_ADDRESS, MMC36X0KJ_REG_DATA, 6, data_reg);

    /* The output raw data unit is "count or LSB" */
    data_temp[0] = (uint16_t)(data_reg[1] << 8 | data_reg[0]);
    data_temp[1] = (uint16_t)(data_reg[3] << 8 | data_reg[2]);
    data_temp[2] = (uint16_t)(data_reg[5] << 8 | data_reg[4]);
    posX = (int)data_temp[0];
    posY = (int)data_temp[1];
    posZ = (int)data_temp[2];

    /* Transform register data, get x, y, z output */
    mag_rawdata[0] = data_temp[0];
    mag_rawdata[1] = data_temp[1] - data_temp[2] + 32768;
    mag_rawdata[2] = data_temp[1] + data_temp[2] - 32768;

    /* Transform to unit Gauss */
    mag_temp[0] = ((float)mag_rawdata[0] - MMC36X0KJ_OFFSET) / MMC36X0KJ_SENSITIVITY; //unit Gauss
    mag_temp[1] = ((float)mag_rawdata[1] - MMC36X0KJ_OFFSET) / MMC36X0KJ_SENSITIVITY;
    mag_temp[2] = ((float)mag_rawdata[2] - MMC36X0KJ_OFFSET) / MMC36X0KJ_SENSITIVITY;

    /* Sensitivity compensation */
    mag_out[0] = mag_temp[0] * fOtpMatrix[0]; //unit Gauss
    mag_out[1] = mag_temp[1] * fOtpMatrix[1];
    mag_out[2] = mag_temp[2] * fOtpMatrix[2];

    return;
}

void MMC36X0KJ_Get_OrgData(int *x, int *y, int *z)
{
    *x = posX;
    *y = posY;
    *z = posZ;
}

/*********************************************************************************
* decription: Read the data register with SET and RESET function
*********************************************************************************/
void MMC36X0KJ_GetData_With_SET_RESET(float *mag_out)
{
    float magnetic_set_field[3];
    float magnetic_reset_field[3];

    /* Do RESET operation before TM and read data */
    //MMC36X0KJ_DualPower_RESET();
    MMC36X0KJ_SinglePower_RESET();

    /* TM and read data */
    MMC36X0KJ_GetData(magnetic_reset_field);

    /* Do SET operation before TM and read data */
    //MMC36X0KJ_DualPower_SET();
    MMC36X0KJ_SinglePower_SET();

    /* TM and read data */
    MMC36X0KJ_GetData(magnetic_set_field);

    /* Get external magnetic field with (SET-REST)/2 to remove the sensor bridge offset */
    mag_out[0] = (magnetic_set_field[0] - magnetic_reset_field[0]) / 2;
    mag_out[1] = (magnetic_set_field[1] - magnetic_reset_field[1]) / 2;
    mag_out[2] = (magnetic_set_field[2] - magnetic_reset_field[2]) / 2;

    return;
}

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
