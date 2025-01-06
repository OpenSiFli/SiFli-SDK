/**
  ******************************************************************************
  * @file   btim.h
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

#ifndef __BTIM_H
#define __BTIM_H

typedef struct
{
    __IO uint32_t CR1;
    __IO uint32_t CR2;
    __IO uint32_t SMCR;
    __IO uint32_t DIER;
    __IO uint32_t SR;
    __IO uint32_t EGR;
    __IO uint32_t RSVD1[3];
    __IO uint32_t CNT;
    __IO uint32_t PSC;
    __IO uint32_t ARR;
} BTIM_TypeDef;


/******************** Bit definition for BTIM_CR1 register ********************/
#define BTIM_CR1_CEN_Pos                (0U)
#define BTIM_CR1_CEN_Msk                (0x1UL << BTIM_CR1_CEN_Pos)
#define BTIM_CR1_CEN                    BTIM_CR1_CEN_Msk
#define BTIM_CR1_UDIS_Pos               (1U)
#define BTIM_CR1_UDIS_Msk               (0x1UL << BTIM_CR1_UDIS_Pos)
#define BTIM_CR1_UDIS                   BTIM_CR1_UDIS_Msk
#define BTIM_CR1_URS_Pos                (2U)
#define BTIM_CR1_URS_Msk                (0x1UL << BTIM_CR1_URS_Pos)
#define BTIM_CR1_URS                    BTIM_CR1_URS_Msk
#define BTIM_CR1_OPM_Pos                (3U)
#define BTIM_CR1_OPM_Msk                (0x1UL << BTIM_CR1_OPM_Pos)
#define BTIM_CR1_OPM                    BTIM_CR1_OPM_Msk
#define BTIM_CR1_ARPE_Pos               (7U)
#define BTIM_CR1_ARPE_Msk               (0x1UL << BTIM_CR1_ARPE_Pos)
#define BTIM_CR1_ARPE                   BTIM_CR1_ARPE_Msk

/******************** Bit definition for BTIM_CR2 register ********************/
#define BTIM_CR2_MMS_Pos                (4U)
#define BTIM_CR2_MMS_Msk                (0x3UL << BTIM_CR2_MMS_Pos)
#define BTIM_CR2_MMS                    BTIM_CR2_MMS_Msk

/******************* Bit definition for BTIM_SMCR register ********************/
#define BTIM_SMCR_TS_Pos                (4U)
#define BTIM_SMCR_TS_Msk                (0x3UL << BTIM_SMCR_TS_Pos)
#define BTIM_SMCR_TS                    BTIM_SMCR_TS_Msk
#define BTIM_SMCR_MSM_Pos               (7U)
#define BTIM_SMCR_MSM_Msk               (0x1UL << BTIM_SMCR_MSM_Pos)
#define BTIM_SMCR_MSM                   BTIM_SMCR_MSM_Msk
#define BTIM_SMCR_SMS_Pos               (16U)
#define BTIM_SMCR_SMS_Msk               (0x7UL << BTIM_SMCR_SMS_Pos)
#define BTIM_SMCR_SMS                   BTIM_SMCR_SMS_Msk
#define BTIM_SMCR_GTS_Pos               (20U)
#define BTIM_SMCR_GTS_Msk               (0x3UL << BTIM_SMCR_GTS_Pos)
#define BTIM_SMCR_GTS                   BTIM_SMCR_GTS_Msk
#define BTIM_SMCR_GTP_Pos               (22U)
#define BTIM_SMCR_GTP_Msk               (0x1UL << BTIM_SMCR_GTP_Pos)
#define BTIM_SMCR_GTP                   BTIM_SMCR_GTP_Msk
#define BTIM_SMCR_GM_Pos                (23U)
#define BTIM_SMCR_GM_Msk                (0x1UL << BTIM_SMCR_GM_Pos)
#define BTIM_SMCR_GM                    BTIM_SMCR_GM_Msk

/******************* Bit definition for BTIM_DIER register ********************/
#define BTIM_DIER_UIE_Pos               (0U)
#define BTIM_DIER_UIE_Msk               (0x1UL << BTIM_DIER_UIE_Pos)
#define BTIM_DIER_UIE                   BTIM_DIER_UIE_Msk
#define BTIM_DIER_UDE_Pos               (8U)
#define BTIM_DIER_UDE_Msk               (0x1UL << BTIM_DIER_UDE_Pos)
#define BTIM_DIER_UDE                   BTIM_DIER_UDE_Msk

/******************** Bit definition for BTIM_SR register *********************/
#define BTIM_SR_UIF_Pos                 (0U)
#define BTIM_SR_UIF_Msk                 (0x1UL << BTIM_SR_UIF_Pos)
#define BTIM_SR_UIF                     BTIM_SR_UIF_Msk

/******************** Bit definition for BTIM_EGR register ********************/
#define BTIM_EGR_UG_Pos                 (0U)
#define BTIM_EGR_UG_Msk                 (0x1UL << BTIM_EGR_UG_Pos)
#define BTIM_EGR_UG                     BTIM_EGR_UG_Msk

/******************** Bit definition for BTIM_CNT register ********************/
#define BTIM_CNT_CNT_Pos                (0U)
#define BTIM_CNT_CNT_Msk                (0xFFFFFFFFUL << BTIM_CNT_CNT_Pos)
#define BTIM_CNT_CNT                    BTIM_CNT_CNT_Msk

/******************** Bit definition for BTIM_PSC register ********************/
#define BTIM_PSC_PSC_Pos                (0U)
#define BTIM_PSC_PSC_Msk                (0xFFFFUL << BTIM_PSC_PSC_Pos)
#define BTIM_PSC_PSC                    BTIM_PSC_PSC_Msk

/******************** Bit definition for BTIM_ARR register ********************/
#define BTIM_ARR_ARR_Pos                (0U)
#define BTIM_ARR_ARR_Msk                (0xFFFFFFFFUL << BTIM_ARR_ARR_Pos)
#define BTIM_ARR_ARR                    BTIM_ARR_ARR_Msk

#endif
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
