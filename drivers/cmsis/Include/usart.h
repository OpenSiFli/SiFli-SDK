/**
  ******************************************************************************
  * @file   usart.h
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

#ifndef __USART_H
#define __USART_H

typedef struct
{
    __IO uint32_t CR1;
    __IO uint32_t CR2;
    __IO uint32_t CR3;
    __IO uint32_t BRR;
    __IO uint32_t GTPR;
    __IO uint32_t RTOR;
    __IO uint32_t RQR;
    __IO uint32_t ISR;
    __IO uint32_t ICR;
    __IO uint32_t RDR;
    __IO uint32_t TDR;
    __IO uint32_t MISCR;
#if defined(SF32LB56X)|| defined(SF32LB52X)
    __IO uint32_t DRDR;
    __IO uint32_t DTDR;
    __IO uint32_t EXR;
#endif /* SF32LB56X */
} USART_TypeDef;


/******************* Bit definition for USART_CR1 register ********************/
#define USART_CR1_UE_Pos                (0U)
#define USART_CR1_UE_Msk                (0x1UL << USART_CR1_UE_Pos)
#define USART_CR1_UE                    USART_CR1_UE_Msk
#define USART_CR1_UESM_Pos              (1U)
#define USART_CR1_UESM_Msk              (0x1UL << USART_CR1_UESM_Pos)
#define USART_CR1_UESM                  USART_CR1_UESM_Msk
#define USART_CR1_RE_Pos                (2U)
#define USART_CR1_RE_Msk                (0x1UL << USART_CR1_RE_Pos)
#define USART_CR1_RE                    USART_CR1_RE_Msk
#define USART_CR1_TE_Pos                (3U)
#define USART_CR1_TE_Msk                (0x1UL << USART_CR1_TE_Pos)
#define USART_CR1_TE                    USART_CR1_TE_Msk
#define USART_CR1_IDLEIE_Pos            (4U)
#define USART_CR1_IDLEIE_Msk            (0x1UL << USART_CR1_IDLEIE_Pos)
#define USART_CR1_IDLEIE                USART_CR1_IDLEIE_Msk
#define USART_CR1_RXNEIE_Pos            (5U)
#define USART_CR1_RXNEIE_Msk            (0x1UL << USART_CR1_RXNEIE_Pos)
#define USART_CR1_RXNEIE                USART_CR1_RXNEIE_Msk
#define USART_CR1_TCIE_Pos              (6U)
#define USART_CR1_TCIE_Msk              (0x1UL << USART_CR1_TCIE_Pos)
#define USART_CR1_TCIE                  USART_CR1_TCIE_Msk
#define USART_CR1_TXEIE_Pos             (7U)
#define USART_CR1_TXEIE_Msk             (0x1UL << USART_CR1_TXEIE_Pos)
#define USART_CR1_TXEIE                 USART_CR1_TXEIE_Msk
#define USART_CR1_PEIE_Pos              (8U)
#define USART_CR1_PEIE_Msk              (0x1UL << USART_CR1_PEIE_Pos)
#define USART_CR1_PEIE                  USART_CR1_PEIE_Msk
#define USART_CR1_PS_Pos                (9U)
#define USART_CR1_PS_Msk                (0x1UL << USART_CR1_PS_Pos)
#define USART_CR1_PS                    USART_CR1_PS_Msk
#define USART_CR1_PCE_Pos               (10U)
#define USART_CR1_PCE_Msk               (0x1UL << USART_CR1_PCE_Pos)
#define USART_CR1_PCE                   USART_CR1_PCE_Msk
#define USART_CR1_WAKE_Pos              (11U)
#define USART_CR1_WAKE_Msk              (0x1UL << USART_CR1_WAKE_Pos)
#define USART_CR1_WAKE                  USART_CR1_WAKE_Msk
#define USART_CR1_MME_Pos               (12U)
#define USART_CR1_MME_Msk               (0x1UL << USART_CR1_MME_Pos)
#define USART_CR1_MME                   USART_CR1_MME_Msk
#define USART_CR1_CMIE_Pos              (13U)
#define USART_CR1_CMIE_Msk              (0x1UL << USART_CR1_CMIE_Pos)
#define USART_CR1_CMIE                  USART_CR1_CMIE_Msk
#define USART_CR1_OVER8_Pos             (14U)
#define USART_CR1_OVER8_Msk             (0x1UL << USART_CR1_OVER8_Pos)
#define USART_CR1_OVER8                 USART_CR1_OVER8_Msk
#define USART_CR1_DEDT_Pos              (15U)
#define USART_CR1_DEDT_Msk              (0x1FUL << USART_CR1_DEDT_Pos)
#define USART_CR1_DEDT                  USART_CR1_DEDT_Msk
#define USART_CR1_DEDT_0                (0x01UL << USART_CR1_DEDT_Pos)
#define USART_CR1_DEDT_1                (0x02UL << USART_CR1_DEDT_Pos)
#define USART_CR1_DEDT_2                (0x04UL << USART_CR1_DEDT_Pos)
#define USART_CR1_DEDT_3                (0x08UL << USART_CR1_DEDT_Pos)
#define USART_CR1_DEDT_4                (0x10UL << USART_CR1_DEDT_Pos)
#define USART_CR1_DEAT_Pos              (20U)
#define USART_CR1_DEAT_Msk              (0x1FUL << USART_CR1_DEAT_Pos)
#define USART_CR1_DEAT                  USART_CR1_DEAT_Msk
#define USART_CR1_DEAT_0                (0x01UL << USART_CR1_DEAT_Pos)
#define USART_CR1_DEAT_1                (0x02UL << USART_CR1_DEAT_Pos)
#define USART_CR1_DEAT_2                (0x04UL << USART_CR1_DEAT_Pos)
#define USART_CR1_DEAT_3                (0x08UL << USART_CR1_DEAT_Pos)
#define USART_CR1_DEAT_4                (0x10UL << USART_CR1_DEAT_Pos)
#define USART_CR1_RTOIE_Pos             (25U)
#define USART_CR1_RTOIE_Msk             (0x1UL << USART_CR1_RTOIE_Pos)
#define USART_CR1_RTOIE                 USART_CR1_RTOIE_Msk
#define USART_CR1_EOBIE_Pos             (26U)
#define USART_CR1_EOBIE_Msk             (0x1UL << USART_CR1_EOBIE_Pos)
#define USART_CR1_EOBIE                 USART_CR1_EOBIE_Msk
#define USART_CR1_M_Pos                 (27U)
#define USART_CR1_M_Msk                 (0x3UL << USART_CR1_M_Pos)
#define USART_CR1_M                     USART_CR1_M_Msk

/******************* Bit definition for USART_CR2 register ********************/
#define USART_CR2_ADDM7_Pos             (4U)
#define USART_CR2_ADDM7_Msk             (0x1UL << USART_CR2_ADDM7_Pos)
#define USART_CR2_ADDM7                 USART_CR2_ADDM7_Msk
#define USART_CR2_LBDL_Pos              (5U)
#define USART_CR2_LBDL_Msk              (0x1UL << USART_CR2_LBDL_Pos)
#define USART_CR2_LBDL                  USART_CR2_LBDL_Msk
#define USART_CR2_LBDIE_Pos             (6U)
#define USART_CR2_LBDIE_Msk             (0x1UL << USART_CR2_LBDIE_Pos)
#define USART_CR2_LBDIE                 USART_CR2_LBDIE_Msk
#define USART_CR2_LBCL_Pos              (8U)
#define USART_CR2_LBCL_Msk              (0x1UL << USART_CR2_LBCL_Pos)
#define USART_CR2_LBCL                  USART_CR2_LBCL_Msk
#define USART_CR2_CPHA_Pos              (9U)
#define USART_CR2_CPHA_Msk              (0x1UL << USART_CR2_CPHA_Pos)
#define USART_CR2_CPHA                  USART_CR2_CPHA_Msk
#define USART_CR2_CPOL_Pos              (10U)
#define USART_CR2_CPOL_Msk              (0x1UL << USART_CR2_CPOL_Pos)
#define USART_CR2_CPOL                  USART_CR2_CPOL_Msk
#define USART_CR2_CLKEN_Pos             (11U)
#define USART_CR2_CLKEN_Msk             (0x1UL << USART_CR2_CLKEN_Pos)
#define USART_CR2_CLKEN                 USART_CR2_CLKEN_Msk
#define USART_CR2_STOP_Pos              (12U)
#define USART_CR2_STOP_Msk              (0x3UL << USART_CR2_STOP_Pos)
#define USART_CR2_STOP                  USART_CR2_STOP_Msk
#define USART_CR2_STOP_0                (0x1UL << USART_CR2_STOP_Pos)
#define USART_CR2_STOP_1                (0x2UL << USART_CR2_STOP_Pos)
#define USART_CR2_LINEN_Pos             (14U)
#define USART_CR2_LINEN_Msk             (0x1UL << USART_CR2_LINEN_Pos)
#define USART_CR2_LINEN                 USART_CR2_LINEN_Msk
#define USART_CR2_SWAP_Pos              (15U)
#define USART_CR2_SWAP_Msk              (0x1UL << USART_CR2_SWAP_Pos)
#define USART_CR2_SWAP                  USART_CR2_SWAP_Msk
#define USART_CR2_RXINV_Pos             (16U)
#define USART_CR2_RXINV_Msk             (0x1UL << USART_CR2_RXINV_Pos)
#define USART_CR2_RXINV                 USART_CR2_RXINV_Msk
#define USART_CR2_TXINV_Pos             (17U)
#define USART_CR2_TXINV_Msk             (0x1UL << USART_CR2_TXINV_Pos)
#define USART_CR2_TXINV                 USART_CR2_TXINV_Msk
#define USART_CR2_DATAINV_Pos           (18U)
#define USART_CR2_DATAINV_Msk           (0x1UL << USART_CR2_DATAINV_Pos)
#define USART_CR2_DATAINV               USART_CR2_DATAINV_Msk
#define USART_CR2_MSBFIRST_Pos          (19U)
#define USART_CR2_MSBFIRST_Msk          (0x1UL << USART_CR2_MSBFIRST_Pos)
#define USART_CR2_MSBFIRST              USART_CR2_MSBFIRST_Msk
#define USART_CR2_ABREN_Pos             (20U)
#define USART_CR2_ABREN_Msk             (0x1UL << USART_CR2_ABREN_Pos)
#define USART_CR2_ABREN                 USART_CR2_ABREN_Msk
#define USART_CR2_ABRMOD_Pos            (21U)
#define USART_CR2_ABRMOD_Msk            (0x3UL << USART_CR2_ABRMOD_Pos)
#define USART_CR2_ABRMOD                USART_CR2_ABRMOD_Msk
#define USART_CR2_ABRMOD_0              (0x1UL << USART_CR2_ABRMOD_Pos)
#define USART_CR2_ABRMOD_1              (0x2UL << USART_CR2_ABRMOD_Pos)
#define USART_CR2_RTOEN_Pos             (23U)
#define USART_CR2_RTOEN_Msk             (0x1UL << USART_CR2_RTOEN_Pos)
#define USART_CR2_RTOEN                 USART_CR2_RTOEN_Msk
#define USART_CR2_ADD_Pos               (24U)
#define USART_CR2_ADD_Msk               (0xFFUL << USART_CR2_ADD_Pos)
#define USART_CR2_ADD                   USART_CR2_ADD_Msk

/******************* Bit definition for USART_CR3 register ********************/
#define USART_CR3_EIE_Pos               (0U)
#define USART_CR3_EIE_Msk               (0x1UL << USART_CR3_EIE_Pos)
#define USART_CR3_EIE                   USART_CR3_EIE_Msk
#define USART_CR3_IREN_Pos              (1U)
#define USART_CR3_IREN_Msk              (0x1UL << USART_CR3_IREN_Pos)
#define USART_CR3_IREN                  USART_CR3_IREN_Msk
#define USART_CR3_IRLP_Pos              (2U)
#define USART_CR3_IRLP_Msk              (0x1UL << USART_CR3_IRLP_Pos)
#define USART_CR3_IRLP                  USART_CR3_IRLP_Msk
#define USART_CR3_HDSEL_Pos             (3U)
#define USART_CR3_HDSEL_Msk             (0x1UL << USART_CR3_HDSEL_Pos)
#define USART_CR3_HDSEL                 USART_CR3_HDSEL_Msk
#define USART_CR3_NACK_Pos              (4U)
#define USART_CR3_NACK_Msk              (0x1UL << USART_CR3_NACK_Pos)
#define USART_CR3_NACK                  USART_CR3_NACK_Msk
#define USART_CR3_SCEN_Pos              (5U)
#define USART_CR3_SCEN_Msk              (0x1UL << USART_CR3_SCEN_Pos)
#define USART_CR3_SCEN                  USART_CR3_SCEN_Msk
#define USART_CR3_DMAR_Pos              (6U)
#define USART_CR3_DMAR_Msk              (0x1UL << USART_CR3_DMAR_Pos)
#define USART_CR3_DMAR                  USART_CR3_DMAR_Msk
#define USART_CR3_DMAT_Pos              (7U)
#define USART_CR3_DMAT_Msk              (0x1UL << USART_CR3_DMAT_Pos)
#define USART_CR3_DMAT                  USART_CR3_DMAT_Msk
#define USART_CR3_RTSE_Pos              (8U)
#define USART_CR3_RTSE_Msk              (0x1UL << USART_CR3_RTSE_Pos)
#define USART_CR3_RTSE                  USART_CR3_RTSE_Msk
#define USART_CR3_CTSE_Pos              (9U)
#define USART_CR3_CTSE_Msk              (0x1UL << USART_CR3_CTSE_Pos)
#define USART_CR3_CTSE                  USART_CR3_CTSE_Msk
#define USART_CR3_CTSIE_Pos             (10U)
#define USART_CR3_CTSIE_Msk             (0x1UL << USART_CR3_CTSIE_Pos)
#define USART_CR3_CTSIE                 USART_CR3_CTSIE_Msk
#define USART_CR3_ONEBIT_Pos            (11U)
#define USART_CR3_ONEBIT_Msk            (0x1UL << USART_CR3_ONEBIT_Pos)
#define USART_CR3_ONEBIT                USART_CR3_ONEBIT_Msk
#define USART_CR3_OVRDIS_Pos            (12U)
#define USART_CR3_OVRDIS_Msk            (0x1UL << USART_CR3_OVRDIS_Pos)
#define USART_CR3_OVRDIS                USART_CR3_OVRDIS_Msk
#define USART_CR3_DDRE_Pos              (13U)
#define USART_CR3_DDRE_Msk              (0x1UL << USART_CR3_DDRE_Pos)
#define USART_CR3_DDRE                  USART_CR3_DDRE_Msk
#define USART_CR3_DEM_Pos               (14U)
#define USART_CR3_DEM_Msk               (0x1UL << USART_CR3_DEM_Pos)
#define USART_CR3_DEM                   USART_CR3_DEM_Msk
#define USART_CR3_DEP_Pos               (15U)
#define USART_CR3_DEP_Msk               (0x1UL << USART_CR3_DEP_Pos)
#define USART_CR3_DEP                   USART_CR3_DEP_Msk
#define USART_CR3_SCARCNT_Pos           (17U)
#define USART_CR3_SCARCNT_Msk           (0x7UL << USART_CR3_SCARCNT_Pos)
#define USART_CR3_SCARCNT               USART_CR3_SCARCNT_Msk
#define USART_CR3_SCARCNT_0             (0x1UL << USART_CR3_SCARCNT_Pos)
#define USART_CR3_SCARCNT_1             (0x2UL << USART_CR3_SCARCNT_Pos)
#define USART_CR3_SCARCNT_2             (0x4UL << USART_CR3_SCARCNT_Pos)
#define USART_CR3_WUS_Pos               (20U)
#define USART_CR3_WUS_Msk               (0x3UL << USART_CR3_WUS_Pos)
#define USART_CR3_WUS                   USART_CR3_WUS_Msk
#define USART_CR3_WUS_0                 (0x1UL << USART_CR3_WUS_Pos)
#define USART_CR3_WUS_1                 (0x2UL << USART_CR3_WUS_Pos)
#define USART_CR3_WUFIE_Pos             (22U)
#define USART_CR3_WUFIE_Msk             (0x1UL << USART_CR3_WUFIE_Pos)
#define USART_CR3_WUFIE                 USART_CR3_WUFIE_Msk
#define USART_CR3_UCESM_Pos             (23U)
#define USART_CR3_UCESM_Msk             (0x1UL << USART_CR3_UCESM_Pos)
#define USART_CR3_UCESM                 USART_CR3_UCESM_Msk
#define USART_CR3_TCBGTIE_Pos           (24U)
#define USART_CR3_TCBGTIE_Msk           (0x1UL << USART_CR3_TCBGTIE_Pos)
#define USART_CR3_TCBGTIE               USART_CR3_TCBGTIE_Msk

/******************* Bit definition for USART_BRR register ********************/
#define USART_BRR_FRAC_Pos              (0U)
#define USART_BRR_FRAC_Msk              (0xFUL << USART_BRR_FRAC_Pos)
#define USART_BRR_FRAC                  USART_BRR_FRAC_Msk
#define USART_BRR_INT_Pos               (4U)
#define USART_BRR_INT_Msk               (0xFFFUL << USART_BRR_INT_Pos)
#define USART_BRR_INT                   USART_BRR_INT_Msk

/******************* Bit definition for USART_GTPR register *******************/
#define USART_GTPR_PSC_Pos              (0U)
#define USART_GTPR_PSC_Msk              (0xFFUL << USART_GTPR_PSC_Pos)
#define USART_GTPR_PSC                  USART_GTPR_PSC_Msk
#define USART_GTPR_GT_Pos               (8U)
#define USART_GTPR_GT_Msk               (0xFFUL << USART_GTPR_GT_Pos)
#define USART_GTPR_GT                   USART_GTPR_GT_Msk

/******************* Bit definition for USART_RTOR register *******************/
#define USART_RTOR_RTO_Pos              (0U)
#define USART_RTOR_RTO_Msk              (0xFFFFFFUL << USART_RTOR_RTO_Pos)
#define USART_RTOR_RTO                  USART_RTOR_RTO_Msk
#define USART_RTOR_BLEN_Pos             (24U)
#define USART_RTOR_BLEN_Msk             (0xFFUL << USART_RTOR_BLEN_Pos)
#define USART_RTOR_BLEN                 USART_RTOR_BLEN_Msk

/******************* Bit definition for USART_RQR register ********************/
#define USART_RQR_ABRRQ_Pos             (0U)
#define USART_RQR_ABRRQ_Msk             (0x1UL << USART_RQR_ABRRQ_Pos)
#define USART_RQR_ABRRQ                 USART_RQR_ABRRQ_Msk
#define USART_RQR_SBKRQ_Pos             (1U)
#define USART_RQR_SBKRQ_Msk             (0x1UL << USART_RQR_SBKRQ_Pos)
#define USART_RQR_SBKRQ                 USART_RQR_SBKRQ_Msk
#define USART_RQR_MMRQ_Pos              (2U)
#define USART_RQR_MMRQ_Msk              (0x1UL << USART_RQR_MMRQ_Pos)
#define USART_RQR_MMRQ                  USART_RQR_MMRQ_Msk
#define USART_RQR_RXFRQ_Pos             (3U)
#define USART_RQR_RXFRQ_Msk             (0x1UL << USART_RQR_RXFRQ_Pos)
#define USART_RQR_RXFRQ                 USART_RQR_RXFRQ_Msk
#define USART_RQR_TXFRQ_Pos             (4U)
#define USART_RQR_TXFRQ_Msk             (0x1UL << USART_RQR_TXFRQ_Pos)
#define USART_RQR_TXFRQ                 USART_RQR_TXFRQ_Msk

/******************* Bit definition for USART_ISR register ********************/
#define USART_ISR_PE_Pos                (0U)
#define USART_ISR_PE_Msk                (0x1UL << USART_ISR_PE_Pos)
#define USART_ISR_PE                    USART_ISR_PE_Msk
#define USART_ISR_FE_Pos                (1U)
#define USART_ISR_FE_Msk                (0x1UL << USART_ISR_FE_Pos)
#define USART_ISR_FE                    USART_ISR_FE_Msk
#define USART_ISR_NF_Pos                (2U)
#define USART_ISR_NF_Msk                (0x1UL << USART_ISR_NF_Pos)
#define USART_ISR_NF                    USART_ISR_NF_Msk
#define USART_ISR_ORE_Pos               (3U)
#define USART_ISR_ORE_Msk               (0x1UL << USART_ISR_ORE_Pos)
#define USART_ISR_ORE                   USART_ISR_ORE_Msk
#define USART_ISR_IDLE_Pos              (4U)
#define USART_ISR_IDLE_Msk              (0x1UL << USART_ISR_IDLE_Pos)
#define USART_ISR_IDLE                  USART_ISR_IDLE_Msk
#define USART_ISR_RXNE_Pos              (5U)
#define USART_ISR_RXNE_Msk              (0x1UL << USART_ISR_RXNE_Pos)
#define USART_ISR_RXNE                  USART_ISR_RXNE_Msk
#define USART_ISR_TC_Pos                (6U)
#define USART_ISR_TC_Msk                (0x1UL << USART_ISR_TC_Pos)
#define USART_ISR_TC                    USART_ISR_TC_Msk
#define USART_ISR_TXE_Pos               (7U)
#define USART_ISR_TXE_Msk               (0x1UL << USART_ISR_TXE_Pos)
#define USART_ISR_TXE                   USART_ISR_TXE_Msk
#define USART_ISR_LBDF_Pos              (8U)
#define USART_ISR_LBDF_Msk              (0x1UL << USART_ISR_LBDF_Pos)
#define USART_ISR_LBDF                  USART_ISR_LBDF_Msk
#define USART_ISR_CTSIF_Pos             (9U)
#define USART_ISR_CTSIF_Msk             (0x1UL << USART_ISR_CTSIF_Pos)
#define USART_ISR_CTSIF                 USART_ISR_CTSIF_Msk
#define USART_ISR_CTS_Pos               (10U)
#define USART_ISR_CTS_Msk               (0x1UL << USART_ISR_CTS_Pos)
#define USART_ISR_CTS                   USART_ISR_CTS_Msk
#define USART_ISR_RTOF_Pos              (11U)
#define USART_ISR_RTOF_Msk              (0x1UL << USART_ISR_RTOF_Pos)
#define USART_ISR_RTOF                  USART_ISR_RTOF_Msk
#define USART_ISR_EOBF_Pos              (12U)
#define USART_ISR_EOBF_Msk              (0x1UL << USART_ISR_EOBF_Pos)
#define USART_ISR_EOBF                  USART_ISR_EOBF_Msk
#define USART_ISR_ABREN_Pos             (14U)
#define USART_ISR_ABREN_Msk             (0x1UL << USART_ISR_ABREN_Pos)
#define USART_ISR_ABREN                 USART_ISR_ABREN_Msk
#define USART_ISR_ABRF_Pos              (15U)
#define USART_ISR_ABRF_Msk              (0x1UL << USART_ISR_ABRF_Pos)
#define USART_ISR_ABRF                  USART_ISR_ABRF_Msk
#define USART_ISR_BUSY_Pos              (16U)
#define USART_ISR_BUSY_Msk              (0x1UL << USART_ISR_BUSY_Pos)
#define USART_ISR_BUSY                  USART_ISR_BUSY_Msk
#define USART_ISR_CMF_Pos               (17U)
#define USART_ISR_CMF_Msk               (0x1UL << USART_ISR_CMF_Pos)
#define USART_ISR_CMF                   USART_ISR_CMF_Msk
#define USART_ISR_SBKF_Pos              (18U)
#define USART_ISR_SBKF_Msk              (0x1UL << USART_ISR_SBKF_Pos)
#define USART_ISR_SBKF                  USART_ISR_SBKF_Msk
#define USART_ISR_RWU_Pos               (19U)
#define USART_ISR_RWU_Msk               (0x1UL << USART_ISR_RWU_Pos)
#define USART_ISR_RWU                   USART_ISR_RWU_Msk
#define USART_ISR_WUF_Pos               (20U)
#define USART_ISR_WUF_Msk               (0x1UL << USART_ISR_WUF_Pos)
#define USART_ISR_WUF                   USART_ISR_WUF_Msk
#define USART_ISR_TEACK_Pos             (21U)
#define USART_ISR_TEACK_Msk             (0x1UL << USART_ISR_TEACK_Pos)
#define USART_ISR_TEACK                 USART_ISR_TEACK_Msk
#define USART_ISR_REACK_Pos             (22U)
#define USART_ISR_REACK_Msk             (0x1UL << USART_ISR_REACK_Pos)
#define USART_ISR_REACK                 USART_ISR_REACK_Msk
#define USART_ISR_TCBGT_Pos             (25U)
#define USART_ISR_TCBGT_Msk             (0x1UL << USART_ISR_TCBGT_Pos)
#define USART_ISR_TCBGT                 USART_ISR_TCBGT_Msk

/******************* Bit definition for USART_ICR register ********************/
#define USART_ICR_PECF_Pos              (0U)
#define USART_ICR_PECF_Msk              (0x1UL << USART_ICR_PECF_Pos)
#define USART_ICR_PECF                  USART_ICR_PECF_Msk
#define USART_ICR_FECF_Pos              (1U)
#define USART_ICR_FECF_Msk              (0x1UL << USART_ICR_FECF_Pos)
#define USART_ICR_FECF                  USART_ICR_FECF_Msk
#define USART_ICR_NCF_Pos               (2U)
#define USART_ICR_NCF_Msk               (0x1UL << USART_ICR_NCF_Pos)
#define USART_ICR_NCF                   USART_ICR_NCF_Msk
#define USART_ICR_ORECF_Pos             (3U)
#define USART_ICR_ORECF_Msk             (0x1UL << USART_ICR_ORECF_Pos)
#define USART_ICR_ORECF                 USART_ICR_ORECF_Msk
#define USART_ICR_IDLECF_Pos            (4U)
#define USART_ICR_IDLECF_Msk            (0x1UL << USART_ICR_IDLECF_Pos)
#define USART_ICR_IDLECF                USART_ICR_IDLECF_Msk
#define USART_ICR_TCCF_Pos              (6U)
#define USART_ICR_TCCF_Msk              (0x1UL << USART_ICR_TCCF_Pos)
#define USART_ICR_TCCF                  USART_ICR_TCCF_Msk
#define USART_ICR_TCBGTCF_Pos           (7U)
#define USART_ICR_TCBGTCF_Msk           (0x1UL << USART_ICR_TCBGTCF_Pos)
#define USART_ICR_TCBGTCF               USART_ICR_TCBGTCF_Msk
#define USART_ICR_LBDCF_Pos             (8U)
#define USART_ICR_LBDCF_Msk             (0x1UL << USART_ICR_LBDCF_Pos)
#define USART_ICR_LBDCF                 USART_ICR_LBDCF_Msk
#define USART_ICR_CTSCF_Pos             (9U)
#define USART_ICR_CTSCF_Msk             (0x1UL << USART_ICR_CTSCF_Pos)
#define USART_ICR_CTSCF                 USART_ICR_CTSCF_Msk
#define USART_ICR_RTOCF_Pos             (11U)
#define USART_ICR_RTOCF_Msk             (0x1UL << USART_ICR_RTOCF_Pos)
#define USART_ICR_RTOCF                 USART_ICR_RTOCF_Msk
#define USART_ICR_EOBCF_Pos             (12U)
#define USART_ICR_EOBCF_Msk             (0x1UL << USART_ICR_EOBCF_Pos)
#define USART_ICR_EOBCF                 USART_ICR_EOBCF_Msk
#define USART_ICR_CMCF_Pos              (17U)
#define USART_ICR_CMCF_Msk              (0x1UL << USART_ICR_CMCF_Pos)
#define USART_ICR_CMCF                  USART_ICR_CMCF_Msk
#define USART_ICR_WUCF_Pos              (20U)
#define USART_ICR_WUCF_Msk              (0x1UL << USART_ICR_WUCF_Pos)
#define USART_ICR_WUCF                  USART_ICR_WUCF_Msk

/******************* Bit definition for USART_RDR register ********************/
#define USART_RDR_RDR_Pos               (0U)
#define USART_RDR_RDR_Msk               (0x1FFUL << USART_RDR_RDR_Pos)
#define USART_RDR_RDR                   USART_RDR_RDR_Msk

/******************* Bit definition for USART_TDR register ********************/
#define USART_TDR_TDR_Pos               (0U)
#define USART_TDR_TDR_Msk               (0x1FFUL << USART_TDR_TDR_Pos)
#define USART_TDR_TDR                   USART_TDR_TDR_Msk

/****************** Bit definition for USART_MISCR register *******************/
#define USART_MISCR_SMPLINI_Pos         (0U)
#define USART_MISCR_SMPLINI_Msk         (0xFUL << USART_MISCR_SMPLINI_Pos)
#define USART_MISCR_SMPLINI             USART_MISCR_SMPLINI_Msk
#define USART_MISCR_RTSBIT_Pos          (4U)
#define USART_MISCR_RTSBIT_Msk          (0xFUL << USART_MISCR_RTSBIT_Pos)
#define USART_MISCR_RTSBIT              USART_MISCR_RTSBIT_Msk
#ifndef SF32LB55X
    #define USART_MISCR_AUTOCAL_Pos         (31U)
    #define USART_MISCR_AUTOCAL_Msk         (0x1UL << USART_MISCR_AUTOCAL_Pos)
    #define USART_MISCR_AUTOCAL             USART_MISCR_AUTOCAL_Msk
#endif /* SF32LB55X */

#if defined(SF32LB56X)|| defined(SF32LB52X)
    /******************* Bit definition for USART_DRDR register *******************/
    #define USART_DRDR_DATA_Pos             (0U)
    #define USART_DRDR_DATA_Msk             (0xFFFFFFFFUL << USART_DRDR_DATA_Pos)
    #define USART_DRDR_DATA                 USART_DRDR_DATA_Msk

    /******************* Bit definition for USART_DTDR register *******************/
    #define USART_DTDR_DATA_Pos             (0U)
    #define USART_DTDR_DATA_Msk             (0xFFFFFFFFUL << USART_DTDR_DATA_Pos)
    #define USART_DTDR_DATA                 USART_DTDR_DATA_Msk

    /******************* Bit definition for USART_EXR register ********************/
    #define USART_EXR_BUSY_Pos              (0U)
    #define USART_EXR_BUSY_Msk              (0x1UL << USART_EXR_BUSY_Pos)
    #define USART_EXR_BUSY                  USART_EXR_BUSY_Msk
    #define USART_EXR_ID_Pos                (4U)
    #define USART_EXR_ID_Msk                (0x1UL << USART_EXR_ID_Pos)
    #define USART_EXR_ID                    USART_EXR_ID_Msk
#endif /* SF32LB56X or SF32LB52X */

#endif
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
