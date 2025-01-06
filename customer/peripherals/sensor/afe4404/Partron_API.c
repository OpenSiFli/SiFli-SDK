/**
  ******************************************************************************
  * @file   Partron_API.c
  * @author Sifli software development team
  * @brief Writes to a HW register by I2C communication
 * @param regAddress: Register address to write to
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
#include "afe4404_hw.h"


/******************************************************************************/
void PPSI26X_enableRead(void);
void PPSI26X_enableWrite(void);
int PPSI26X_readRegWithoutReadEnable(int reg);
int PPSI26X_readRegWithReadEnable(int reg);
void PPSI26X_writeRegWithoutWriteEnable(int reg, int registerValue);
void PPSI26X_writeRegWithWriteEnable(int reg, int registerValue);
void check_inside(int value1, int value2, int value3);

/******************************************************************************/






/**
 * @brief Writes to a HW register by I2C communication
 * @param regAddress: Register address to write to
 * @param wdata : Data to send to the address
 * @return: none
 */
static void I2C_writeReg(int regAddress, int wdata)
{
    /* I2C Write */
    //I2C_writeReg_Internal(regAddress,wdata);
    PPS960_writeReg((uint8_t)(regAddress & 0xff), wdata);
}

/**
 * @brief Read data from a register
 * @param regAddress: Register to read from
 * @return: Return the register data in uint32
 */
static int I2C_readReg(int regAddress)
{
    /* I2C Read */
    return PPS960_readReg((uint8_t)(regAddress & 0xff));
}


/**
 * @brief: Enables the AFE to read to a register
 * @param: None
 * @return: None
 */
void PPSI26X_enableRead(void)
{
    I2C_writeReg(0x00, 0x00000021);
}

/**
 * @brief: Enables the AFE to write to a register
 * @param: None
 * @return: None
 */
void PPSI26X_enableWrite(void)
{
    I2C_writeReg(0x00, 0x00000020);
}

int PPSI26X_readRegWithoutReadEnable(int reg)
{
    return (I2C_readReg(reg));
}

int PPSI26X_readRegWithReadEnable(int reg)
{
    PPSI26X_enableRead();
    return (I2C_readReg(reg));
}

void PPSI26X_writeRegWithoutWriteEnable(int reg, int registerValue)
{
    I2C_writeReg(reg, registerValue);
}

void PPSI26X_writeRegWithWriteEnable(int reg, int registerValue)
{
    PPSI26X_enableWrite();
    I2C_writeReg(reg, registerValue);
}



/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
