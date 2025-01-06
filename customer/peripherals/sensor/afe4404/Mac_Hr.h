/**
  ******************************************************************************
  * @file   Mac_Hr.h
  * @author Sifli software development team
  * @brief Initialize the variables, This function should be called every time the algorithm starts
 * @Param : None
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

#ifndef __MAC_HR_HDR_XX01__
#define __MAC_HR_HDR_XX01__

/* Includes ------------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Extern variables ----------------------------------------------------------*/







/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @brief Initialize the variables, This function should be called every time the algorithm starts
 * @Param : None
 * @return: None
 */

void Mac_Init();


/**
 * @brief Calculate Heart rate function
 * @param GreenLed : Green Led value, AccX : Acc X G value, AccY : Acc Y G value, AccZ : Acc Z G value
 * @return: HR
 */

int Partron_ALG(double LED1, double AccX, double AccY, double AccZ);

/**
 * @brief Calculate BBI function
 * @return: BBI
 */

int BBI_Calculation();

/** * @brief Check that the library link is correct
* @param: None
* @return: Some defined hr values
    {0,0,0,0,0,0,0,0,0,0,21,38,57,75,93,91,96,100,105,110,116,116,116,116,119,122,124,127,130,130,130}
*/

int Mac_HR_Validcheck();


double Adaptive_MACancel_2(double *dc_GREEN1, double *ACC_X1, double *ACC_Y1, double *ACC_Z1);
int Get_snr();

#endif  // __MAC_HR_HDR_XX01__
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
