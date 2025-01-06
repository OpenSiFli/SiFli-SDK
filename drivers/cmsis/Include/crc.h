/**
  ******************************************************************************
  * @file   crc.h
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

#ifndef __CRC_H
#define __CRC_H

typedef struct
{
    __IO uint32_t DR;
    __IO uint32_t SR;
    __IO uint32_t CR;
    __IO uint32_t RSVD1;
    __IO uint32_t INIT;
    __IO uint32_t POL;
} CRC_TypeDef;


/********************* Bit definition for CRC_DR register *********************/
#define CRC_DR_DR_Pos                   (0U)
#define CRC_DR_DR_Msk                   (0xFFFFFFFFUL << CRC_DR_DR_Pos)
#define CRC_DR_DR                       CRC_DR_DR_Msk

/********************* Bit definition for CRC_SR register *********************/
#define CRC_SR_DONE_Pos                 (0U)
#define CRC_SR_DONE_Msk                 (0x1UL << CRC_SR_DONE_Pos)
#define CRC_SR_DONE                     CRC_SR_DONE_Msk
#define CRC_SR_OVERFLOW_Pos             (1U)
#define CRC_SR_OVERFLOW_Msk             (0x1UL << CRC_SR_OVERFLOW_Pos)
#define CRC_SR_OVERFLOW                 CRC_SR_OVERFLOW_Msk

/********************* Bit definition for CRC_CR register *********************/
#define CRC_CR_RESET_Pos                (0U)
#define CRC_CR_RESET_Msk                (0x1UL << CRC_CR_RESET_Pos)
#define CRC_CR_RESET                    CRC_CR_RESET_Msk
#define CRC_CR_DATASIZE_Pos             (1U)
#define CRC_CR_DATASIZE_Msk             (0x3UL << CRC_CR_DATASIZE_Pos)
#define CRC_CR_DATASIZE                 CRC_CR_DATASIZE_Msk
#define CRC_CR_POLYSIZE_Pos             (3U)
#define CRC_CR_POLYSIZE_Msk             (0x3UL << CRC_CR_POLYSIZE_Pos)
#define CRC_CR_POLYSIZE                 CRC_CR_POLYSIZE_Msk
#define CRC_CR_REV_IN_Pos               (5U)
#define CRC_CR_REV_IN_Msk               (0x3UL << CRC_CR_REV_IN_Pos)
#define CRC_CR_REV_IN                   CRC_CR_REV_IN_Msk
#define CRC_CR_REV_OUT_Pos              (7U)
#define CRC_CR_REV_OUT_Msk              (0x1UL << CRC_CR_REV_OUT_Pos)
#define CRC_CR_REV_OUT                  CRC_CR_REV_OUT_Msk

/******************** Bit definition for CRC_INIT register ********************/
#define CRC_INIT_INIT_Pos               (0U)
#define CRC_INIT_INIT_Msk               (0xFFFFFFFFUL << CRC_INIT_INIT_Pos)
#define CRC_INIT_INIT                   CRC_INIT_INIT_Msk

/******************** Bit definition for CRC_POL register *********************/
#define CRC_POL_POL_Pos                 (0U)
#define CRC_POL_POL_Msk                 (0xFFFFFFFFUL << CRC_POL_POL_Pos)
#define CRC_POL_POL                     CRC_POL_POL_Msk

#endif
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
