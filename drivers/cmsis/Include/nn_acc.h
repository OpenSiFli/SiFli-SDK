/**
  ******************************************************************************
  * @file   nn_acc.h
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
#ifndef __NN_ACC_H
#define __NN_ACC_H

typedef struct
{
    __IO uint32_t CFG1;
    __IO uint32_t CFG2;
    __IO uint32_t CFG3;
    __IO uint32_t CFG4;
    __IO uint32_t CFG5;
    __IO uint32_t CFG6;
    __IO uint32_t CFG7;
#ifndef SF32LB55X
    __IO uint32_t CFG8;
#endif
    __IO uint32_t CR1;
    __IO uint32_t IRQ;
} NN_ACC_TypeDef;


/****************** Bit definition for NN_ACC_CFG1 register *******************/
#define NN_ACC_CFG1_KERH_Pos            (0U)
#define NN_ACC_CFG1_KERH_Msk            (0xFUL << NN_ACC_CFG1_KERH_Pos)
#define NN_ACC_CFG1_KERH                NN_ACC_CFG1_KERH_Msk
#define NN_ACC_CFG1_KERW_Pos            (4U)
#define NN_ACC_CFG1_KERW_Msk            (0xFUL << NN_ACC_CFG1_KERW_Pos)
#define NN_ACC_CFG1_KERW                NN_ACC_CFG1_KERW_Msk
#define NN_ACC_CFG1_OFH_Pos             (8U)
#define NN_ACC_CFG1_OFH_Msk             (0xFFUL << NN_ACC_CFG1_OFH_Pos)
#define NN_ACC_CFG1_OFH                 NN_ACC_CFG1_OFH_Msk
#define NN_ACC_CFG1_OFW_Pos             (16U)
#define NN_ACC_CFG1_OFW_Msk             (0xFFUL << NN_ACC_CFG1_OFW_Pos)
#define NN_ACC_CFG1_OFW                 NN_ACC_CFG1_OFW_Msk
#define NN_ACC_CFG1_ORIENT_Pos          (24U)
#define NN_ACC_CFG1_ORIENT_Msk          (0x3UL << NN_ACC_CFG1_ORIENT_Pos)
#define NN_ACC_CFG1_ORIENT              NN_ACC_CFG1_ORIENT_Msk
#define NN_ACC_CFG1_STRIDEX_Pos         (26U)
#define NN_ACC_CFG1_STRIDEX_Msk         (0x7UL << NN_ACC_CFG1_STRIDEX_Pos)
#define NN_ACC_CFG1_STRIDEX             NN_ACC_CFG1_STRIDEX_Msk
#define NN_ACC_CFG1_STRIDEY_Pos         (29U)
#define NN_ACC_CFG1_STRIDEY_Msk         (0x7UL << NN_ACC_CFG1_STRIDEY_Pos)
#define NN_ACC_CFG1_STRIDEY             NN_ACC_CFG1_STRIDEY_Msk

/****************** Bit definition for NN_ACC_CFG2 register *******************/
#define NN_ACC_CFG2_IFH_Pos             (0U)
#define NN_ACC_CFG2_IFH_Msk             (0xFFUL << NN_ACC_CFG2_IFH_Pos)
#define NN_ACC_CFG2_IFH                 NN_ACC_CFG2_IFH_Msk
#define NN_ACC_CFG2_IFW_Pos             (8U)
#define NN_ACC_CFG2_IFW_Msk             (0xFFUL << NN_ACC_CFG2_IFW_Pos)
#define NN_ACC_CFG2_IFW                 NN_ACC_CFG2_IFW_Msk
#define NN_ACC_CFG2_IFCH_NUM_Pos        (16U)
#define NN_ACC_CFG2_IFCH_NUM_Msk        (0xFFUL << NN_ACC_CFG2_IFCH_NUM_Pos)
#define NN_ACC_CFG2_IFCH_NUM            NN_ACC_CFG2_IFCH_NUM_Msk
#define NN_ACC_CFG2_OFCH_NUM_Pos        (24U)
#define NN_ACC_CFG2_OFCH_NUM_Msk        (0xFFUL << NN_ACC_CFG2_OFCH_NUM_Pos)
#define NN_ACC_CFG2_OFCH_NUM            NN_ACC_CFG2_OFCH_NUM_Msk

/****************** Bit definition for NN_ACC_CFG3 register *******************/
#define NN_ACC_CFG3_ADDRK_Pos           (0U)
#define NN_ACC_CFG3_ADDRK_Msk           (0xFFFFFFFFUL << NN_ACC_CFG3_ADDRK_Pos)
#define NN_ACC_CFG3_ADDRK               NN_ACC_CFG3_ADDRK_Msk

/****************** Bit definition for NN_ACC_CFG4 register *******************/
#define NN_ACC_CFG4_ADDRI_Pos           (0U)
#define NN_ACC_CFG4_ADDRI_Msk           (0xFFFFFFFFUL << NN_ACC_CFG4_ADDRI_Pos)
#define NN_ACC_CFG4_ADDRI               NN_ACC_CFG4_ADDRI_Msk

/****************** Bit definition for NN_ACC_CFG5 register *******************/
#define NN_ACC_CFG5_ADDRO_Pos           (0U)
#define NN_ACC_CFG5_ADDRO_Msk           (0xFFFFFFFFUL << NN_ACC_CFG5_ADDRO_Pos)
#define NN_ACC_CFG5_ADDRO               NN_ACC_CFG5_ADDRO_Msk

/****************** Bit definition for NN_ACC_CFG6 register *******************/
#define NN_ACC_CFG6_ADDRB_Pos           (0U)
#define NN_ACC_CFG6_ADDRB_Msk           (0xFFFFFFFFUL << NN_ACC_CFG6_ADDRB_Pos)
#define NN_ACC_CFG6_ADDRB               NN_ACC_CFG6_ADDRB_Msk

/****************** Bit definition for NN_ACC_CFG7 register *******************/
#define NN_ACC_CFG7_BIASSHIFT_Pos       (0U)
#define NN_ACC_CFG7_BIASSHIFT_Msk       (0xFFUL << NN_ACC_CFG7_BIASSHIFT_Pos)
#define NN_ACC_CFG7_BIASSHIFT           NN_ACC_CFG7_BIASSHIFT_Msk
#define NN_ACC_CFG7_OUTSHIFT_Pos        (8U)
#define NN_ACC_CFG7_OUTSHIFT_Msk        (0xFFUL << NN_ACC_CFG7_OUTSHIFT_Pos)
#define NN_ACC_CFG7_OUTSHIFT            NN_ACC_CFG7_OUTSHIFT_Msk
#define NN_ACC_CFG7_PADX_Pos            (16U)
#define NN_ACC_CFG7_PADX_Msk            (0xFUL << NN_ACC_CFG7_PADX_Pos)
#define NN_ACC_CFG7_PADX                NN_ACC_CFG7_PADX_Msk
#define NN_ACC_CFG7_PADY_Pos            (20U)
#define NN_ACC_CFG7_PADY_Msk            (0xFUL << NN_ACC_CFG7_PADY_Pos)
#define NN_ACC_CFG7_PADY                NN_ACC_CFG7_PADY_Msk
#define NN_ACC_CFG7_KERNUM_Pos          (24U)
#define NN_ACC_CFG7_KERNUM_Msk          (0xFFUL << NN_ACC_CFG7_KERNUM_Pos)
#define NN_ACC_CFG7_KERNUM              NN_ACC_CFG7_KERNUM_Msk

#ifndef SF32LB55X
    /****************** Bit definition for NN_ACC_CFG8 register *******************/
    #define NN_ACC_CFG8_UNSIGNED_KER_Pos    (0U)
    #define NN_ACC_CFG8_UNSIGNED_KER_Msk    (0x1UL << NN_ACC_CFG8_UNSIGNED_KER_Pos)
    #define NN_ACC_CFG8_UNSIGNED_KER        NN_ACC_CFG8_UNSIGNED_KER_Msk
    #define NN_ACC_CFG8_UNSIGNED_IN_Pos     (1U)
    #define NN_ACC_CFG8_UNSIGNED_IN_Msk     (0x1UL << NN_ACC_CFG8_UNSIGNED_IN_Pos)
    #define NN_ACC_CFG8_UNSIGNED_IN         NN_ACC_CFG8_UNSIGNED_IN_Msk
    #define NN_ACC_CFG8_UNSIGNED_OUT_Pos    (2U)
    #define NN_ACC_CFG8_UNSIGNED_OUT_Msk    (0x1UL << NN_ACC_CFG8_UNSIGNED_OUT_Pos)
    #define NN_ACC_CFG8_UNSIGNED_OUT        NN_ACC_CFG8_UNSIGNED_OUT_Msk
#endif

/******************* Bit definition for NN_ACC_CR1 register *******************/
#define NN_ACC_CR1_ENABLE_Pos           (0U)
#define NN_ACC_CR1_ENABLE_Msk           (0x1UL << NN_ACC_CR1_ENABLE_Pos)
#define NN_ACC_CR1_ENABLE               NN_ACC_CR1_ENABLE_Msk
#define NN_ACC_CR1_START_Pos            (1U)
#define NN_ACC_CR1_START_Msk            (0x1UL << NN_ACC_CR1_START_Pos)
#define NN_ACC_CR1_START                NN_ACC_CR1_START_Msk

/******************* Bit definition for NN_ACC_IRQ register *******************/
#define NN_ACC_IRQ_IMR_Pos              (0U)
#define NN_ACC_IRQ_IMR_Msk              (0x1UL << NN_ACC_IRQ_IMR_Pos)
#define NN_ACC_IRQ_IMR                  NN_ACC_IRQ_IMR_Msk
#define NN_ACC_IRQ_ICR_Pos              (1U)
#define NN_ACC_IRQ_ICR_Msk              (0x1UL << NN_ACC_IRQ_ICR_Pos)
#define NN_ACC_IRQ_ICR                  NN_ACC_IRQ_ICR_Msk
#define NN_ACC_IRQ_IRSR_Pos             (2U)
#define NN_ACC_IRQ_IRSR_Msk             (0x1UL << NN_ACC_IRQ_IRSR_Pos)
#define NN_ACC_IRQ_IRSR                 NN_ACC_IRQ_IRSR_Msk
#define NN_ACC_IRQ_ISR_Pos              (3U)
#define NN_ACC_IRQ_ISR_Msk              (0x1UL << NN_ACC_IRQ_ISR_Pos)
#define NN_ACC_IRQ_ISR                  NN_ACC_IRQ_ISR_Msk

#endif
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
