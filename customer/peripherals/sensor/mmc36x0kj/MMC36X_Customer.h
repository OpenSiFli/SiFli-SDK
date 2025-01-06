/**
  ******************************************************************************
  * @file   MMC36X_Customer.h
  * @author Sifli software development team
  * @brief Delay function.
  *
  *****************************************************************************
**/
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

#ifndef __MMC36X_HDR_FILE__
#define __MMC36X_HDR_FILE__

#include "board.h"
/**
 * @brief Delay function.
 */
void Delay_Ms(int cnt);

/**
 * @brief Delay function.
 */
void Delay_Us(int cnt);

/**
 * @brief I2C BUS initial.
 */
int MMC36X_I2C_Init();
/**
 * @brief I2C write register.
 */
int I2C_Write_Reg(unsigned char i2c_add, unsigned char reg_add, unsigned char cmd);

/**
 * @brief I2C read register.
 */
int I2C_Read_Reg(unsigned char i2c_add, unsigned char reg_add, unsigned char *data);

/**
 * @brief I2C multi read.
 */
int I2C_MultiRead_Reg(unsigned char i2c_add, unsigned char reg_add, int num, unsigned char *data);

uint32_t MMC36X0KJ_get_bus();
int MMC36X0KJ_self_check(void);

#endif //__MMC36X_HDR_FILE__/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
