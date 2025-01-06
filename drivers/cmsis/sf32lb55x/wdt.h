/**
  ******************************************************************************
  * @file   wdt.h
  * @author Sifli software development team
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

#ifndef __WDT_H
#define __WDT_H

typedef struct
{
    __IO uint32_t WDT_CVR0;
    __IO uint32_t WDT_CVR1;
    __IO uint32_t WDT_CR;
    __IO uint32_t WDT_CCR;
    __IO uint32_t WDT_ICR;
    __IO uint32_t WDT_SR;
    __IO uint32_t WDT_WP;
} WDT_TypeDef;


/****************** Bit definition for WDT_WDT_CVR0 register ******************/
#define WDT_WDT_CVR0_COUNT_VALUE_0_Pos  (0U)
#define WDT_WDT_CVR0_COUNT_VALUE_0_Msk  (0xFFFFFFUL << WDT_WDT_CVR0_COUNT_VALUE_0_Pos)
#define WDT_WDT_CVR0_COUNT_VALUE_0      WDT_WDT_CVR0_COUNT_VALUE_0_Msk

/****************** Bit definition for WDT_WDT_CVR1 register ******************/
#define WDT_WDT_CVR1_COUNT_VALUE_1_Pos  (0U)
#define WDT_WDT_CVR1_COUNT_VALUE_1_Msk  (0xFFFFFFUL << WDT_WDT_CVR1_COUNT_VALUE_1_Pos)
#define WDT_WDT_CVR1_COUNT_VALUE_1      WDT_WDT_CVR1_COUNT_VALUE_1_Msk

/******************* Bit definition for WDT_WDT_CR register *******************/
#define WDT_WDT_CR_RESET_LENGTH_Pos     (0U)
#define WDT_WDT_CR_RESET_LENGTH_Msk     (0x7UL << WDT_WDT_CR_RESET_LENGTH_Pos)
#define WDT_WDT_CR_RESET_LENGTH         WDT_WDT_CR_RESET_LENGTH_Msk
#define WDT_WDT_CR_RESPONSE_MODE_Pos    (4U)
#define WDT_WDT_CR_RESPONSE_MODE_Msk    (0x1UL << WDT_WDT_CR_RESPONSE_MODE_Pos)
#define WDT_WDT_CR_RESPONSE_MODE        WDT_WDT_CR_RESPONSE_MODE_Msk

/****************** Bit definition for WDT_WDT_CCR register *******************/
#define WDT_WDT_CCR_COUNTER_CONTROL_Pos  (0U)
#define WDT_WDT_CCR_COUNTER_CONTROL_Msk  (0xFFUL << WDT_WDT_CCR_COUNTER_CONTROL_Pos)
#define WDT_WDT_CCR_COUNTER_CONTROL     WDT_WDT_CCR_COUNTER_CONTROL_Msk

/****************** Bit definition for WDT_WDT_ICR register *******************/
#define WDT_WDT_ICR_INT_CLR_Pos         (0U)
#define WDT_WDT_ICR_INT_CLR_Msk         (0x1UL << WDT_WDT_ICR_INT_CLR_Pos)
#define WDT_WDT_ICR_INT_CLR             WDT_WDT_ICR_INT_CLR_Msk

/******************* Bit definition for WDT_WDT_SR register *******************/
#define WDT_WDT_SR_INT_ASSERT_Pos       (0U)
#define WDT_WDT_SR_INT_ASSERT_Msk       (0x1UL << WDT_WDT_SR_INT_ASSERT_Pos)
#define WDT_WDT_SR_INT_ASSERT           WDT_WDT_SR_INT_ASSERT_Msk
#define WDT_WDT_SR_WDT_ACTIVE_Pos       (1U)
#define WDT_WDT_SR_WDT_ACTIVE_Msk       (0x1UL << WDT_WDT_SR_WDT_ACTIVE_Pos)
#define WDT_WDT_SR_WDT_ACTIVE           WDT_WDT_SR_WDT_ACTIVE_Msk

/******************* Bit definition for WDT_WDT_WP register *******************/
#define WDT_WDT_WP_WRPT_Pos             (0U)
#define WDT_WDT_WP_WRPT_Msk             (0x7FFFFFFFUL << WDT_WDT_WP_WRPT_Pos)
#define WDT_WDT_WP_WRPT                 WDT_WDT_WP_WRPT_Msk
#define WDT_WDT_WP_WRPT_ST_Pos          (31U)
#define WDT_WDT_WP_WRPT_ST_Msk          (0x1UL << WDT_WDT_WP_WRPT_ST_Pos)
#define WDT_WDT_WP_WRPT_ST              WDT_WDT_WP_WRPT_ST_Msk

#endif
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
