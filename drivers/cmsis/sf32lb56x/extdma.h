/**
  ******************************************************************************
  * @file   extdma.h
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

#ifndef __EXTDMA_H
#define __EXTDMA_H

typedef struct
{
    __IO uint32_t ISR;
    __IO uint32_t IFCR;
    __IO uint32_t CCR;
    __IO uint32_t CNDTR;
    __IO uint32_t SRCAR;
    __IO uint32_t DSTAR;
    __IO uint32_t RSVD1[2];
    __IO uint32_t CMPRCR;
    __IO uint32_t CMPRSR;
    __IO uint32_t CMPRNDTR;
    __IO uint32_t CMPRCFG0;
    __IO uint32_t CMPRCFG1;
    __IO uint32_t CMPRQR;
    __IO uint32_t CMPRDR;
} EXTDMA_TypeDef;


/******************* Bit definition for EXTDMA_ISR register *******************/
#define EXTDMA_ISR_GIF_Pos              (0U)
#define EXTDMA_ISR_GIF_Msk              (0x1UL << EXTDMA_ISR_GIF_Pos)
#define EXTDMA_ISR_GIF                  EXTDMA_ISR_GIF_Msk
#define EXTDMA_ISR_TCIF_Pos             (1U)
#define EXTDMA_ISR_TCIF_Msk             (0x1UL << EXTDMA_ISR_TCIF_Pos)
#define EXTDMA_ISR_TCIF                 EXTDMA_ISR_TCIF_Msk
#define EXTDMA_ISR_HTIF_Pos             (2U)
#define EXTDMA_ISR_HTIF_Msk             (0x1UL << EXTDMA_ISR_HTIF_Pos)
#define EXTDMA_ISR_HTIF                 EXTDMA_ISR_HTIF_Msk
#define EXTDMA_ISR_TEIF_Pos             (3U)
#define EXTDMA_ISR_TEIF_Msk             (0x1UL << EXTDMA_ISR_TEIF_Pos)
#define EXTDMA_ISR_TEIF                 EXTDMA_ISR_TEIF_Msk
#define EXTDMA_ISR_OFIF_Pos             (4U)
#define EXTDMA_ISR_OFIF_Msk             (0x1UL << EXTDMA_ISR_OFIF_Pos)
#define EXTDMA_ISR_OFIF                 EXTDMA_ISR_OFIF_Msk

/****************** Bit definition for EXTDMA_IFCR register *******************/
#define EXTDMA_IFCR_CGIF_Pos            (0U)
#define EXTDMA_IFCR_CGIF_Msk            (0x1UL << EXTDMA_IFCR_CGIF_Pos)
#define EXTDMA_IFCR_CGIF                EXTDMA_IFCR_CGIF_Msk
#define EXTDMA_IFCR_CTCIF_Pos           (1U)
#define EXTDMA_IFCR_CTCIF_Msk           (0x1UL << EXTDMA_IFCR_CTCIF_Pos)
#define EXTDMA_IFCR_CTCIF               EXTDMA_IFCR_CTCIF_Msk
#define EXTDMA_IFCR_CHTIF_Pos           (2U)
#define EXTDMA_IFCR_CHTIF_Msk           (0x1UL << EXTDMA_IFCR_CHTIF_Pos)
#define EXTDMA_IFCR_CHTIF               EXTDMA_IFCR_CHTIF_Msk
#define EXTDMA_IFCR_CTEIF_Pos           (3U)
#define EXTDMA_IFCR_CTEIF_Msk           (0x1UL << EXTDMA_IFCR_CTEIF_Pos)
#define EXTDMA_IFCR_CTEIF               EXTDMA_IFCR_CTEIF_Msk
#define EXTDMA_IFCR_COFIF_Pos           (4U)
#define EXTDMA_IFCR_COFIF_Msk           (0x1UL << EXTDMA_IFCR_COFIF_Pos)
#define EXTDMA_IFCR_COFIF               EXTDMA_IFCR_COFIF_Msk

/******************* Bit definition for EXTDMA_CCR register *******************/
#define EXTDMA_CCR_EN_Pos               (0U)
#define EXTDMA_CCR_EN_Msk               (0x1UL << EXTDMA_CCR_EN_Pos)
#define EXTDMA_CCR_EN                   EXTDMA_CCR_EN_Msk
#define EXTDMA_CCR_TCIE_Pos             (1U)
#define EXTDMA_CCR_TCIE_Msk             (0x1UL << EXTDMA_CCR_TCIE_Pos)
#define EXTDMA_CCR_TCIE                 EXTDMA_CCR_TCIE_Msk
#define EXTDMA_CCR_HTIE_Pos             (2U)
#define EXTDMA_CCR_HTIE_Msk             (0x1UL << EXTDMA_CCR_HTIE_Pos)
#define EXTDMA_CCR_HTIE                 EXTDMA_CCR_HTIE_Msk
#define EXTDMA_CCR_TEIE_Pos             (3U)
#define EXTDMA_CCR_TEIE_Msk             (0x1UL << EXTDMA_CCR_TEIE_Pos)
#define EXTDMA_CCR_TEIE                 EXTDMA_CCR_TEIE_Msk
#define EXTDMA_CCR_OFIE_Pos             (4U)
#define EXTDMA_CCR_OFIE_Msk             (0x1UL << EXTDMA_CCR_OFIE_Pos)
#define EXTDMA_CCR_OFIE                 EXTDMA_CCR_OFIE_Msk
#define EXTDMA_CCR_DSTINC_Pos           (6U)
#define EXTDMA_CCR_DSTINC_Msk           (0x1UL << EXTDMA_CCR_DSTINC_Pos)
#define EXTDMA_CCR_DSTINC               EXTDMA_CCR_DSTINC_Msk
#define EXTDMA_CCR_SRCINC_Pos           (7U)
#define EXTDMA_CCR_SRCINC_Msk           (0x1UL << EXTDMA_CCR_SRCINC_Pos)
#define EXTDMA_CCR_SRCINC               EXTDMA_CCR_SRCINC_Msk
#define EXTDMA_CCR_DSTSIZE_Pos          (8U)
#define EXTDMA_CCR_DSTSIZE_Msk          (0x3UL << EXTDMA_CCR_DSTSIZE_Pos)
#define EXTDMA_CCR_DSTSIZE              EXTDMA_CCR_DSTSIZE_Msk
#define EXTDMA_CCR_SRCSIZE_Pos          (10U)
#define EXTDMA_CCR_SRCSIZE_Msk          (0x3UL << EXTDMA_CCR_SRCSIZE_Pos)
#define EXTDMA_CCR_SRCSIZE              EXTDMA_CCR_SRCSIZE_Msk
#define EXTDMA_CCR_DSTBURST_Pos         (16U)
#define EXTDMA_CCR_DSTBURST_Msk         (0x3UL << EXTDMA_CCR_DSTBURST_Pos)
#define EXTDMA_CCR_DSTBURST             EXTDMA_CCR_DSTBURST_Msk
#define EXTDMA_CCR_SRCBURST_Pos         (18U)
#define EXTDMA_CCR_SRCBURST_Msk         (0x3UL << EXTDMA_CCR_SRCBURST_Pos)
#define EXTDMA_CCR_SRCBURST             EXTDMA_CCR_SRCBURST_Msk
#define EXTDMA_CCR_RESET_Pos            (31U)
#define EXTDMA_CCR_RESET_Msk            (0x1UL << EXTDMA_CCR_RESET_Pos)
#define EXTDMA_CCR_RESET                EXTDMA_CCR_RESET_Msk

/****************** Bit definition for EXTDMA_CNDTR register ******************/
#define EXTDMA_CNDTR_NDT_Pos            (0U)
#define EXTDMA_CNDTR_NDT_Msk            (0xFFFFFUL << EXTDMA_CNDTR_NDT_Pos)
#define EXTDMA_CNDTR_NDT                EXTDMA_CNDTR_NDT_Msk

/****************** Bit definition for EXTDMA_SRCAR register ******************/
#define EXTDMA_SRCAR_SRCADDR_Pos        (0U)
#define EXTDMA_SRCAR_SRCADDR_Msk        (0xFFFFFFFFUL << EXTDMA_SRCAR_SRCADDR_Pos)
#define EXTDMA_SRCAR_SRCADDR            EXTDMA_SRCAR_SRCADDR_Msk

/****************** Bit definition for EXTDMA_DSTAR register ******************/
#define EXTDMA_DSTAR_DSTADDR_Pos        (0U)
#define EXTDMA_DSTAR_DSTADDR_Msk        (0xFFFFFFFFUL << EXTDMA_DSTAR_DSTADDR_Pos)
#define EXTDMA_DSTAR_DSTADDR            EXTDMA_DSTAR_DSTADDR_Msk

/***************** Bit definition for EXTDMA_CMPRCR register ******************/
#define EXTDMA_CMPRCR_CMPREN_Pos        (0U)
#define EXTDMA_CMPRCR_CMPREN_Msk        (0x1UL << EXTDMA_CMPRCR_CMPREN_Pos)
#define EXTDMA_CMPRCR_CMPREN            EXTDMA_CMPRCR_CMPREN_Msk
#define EXTDMA_CMPRCR_SRCFMT_Pos        (4U)
#define EXTDMA_CMPRCR_SRCFMT_Msk        (0x3UL << EXTDMA_CMPRCR_SRCFMT_Pos)
#define EXTDMA_CMPRCR_SRCFMT            EXTDMA_CMPRCR_SRCFMT_Msk
#define EXTDMA_CMPRCR_SRCFMT_RGB565     (0UL << EXTDMA_CMPRCR_SRCFMT_Pos)
#define EXTDMA_CMPRCR_SRCFMT_RGB888     (1UL << EXTDMA_CMPRCR_SRCFMT_Pos)
#define EXTDMA_CMPRCR_SRCFMT_ARGB8888   (2UL << EXTDMA_CMPRCR_SRCFMT_Pos)
#define EXTDMA_CMPRCR_SRCPOS_Pos        (6U)
#define EXTDMA_CMPRCR_SRCPOS_Msk        (0x3UL << EXTDMA_CMPRCR_SRCPOS_Pos)
#define EXTDMA_CMPRCR_SRCPOS            EXTDMA_CMPRCR_SRCPOS_Msk

/***************** Bit definition for EXTDMA_CMPRSR register ******************/
#define EXTDMA_CMPRSR_LINESIZE_Pos      (0U)
#define EXTDMA_CMPRSR_LINESIZE_Msk      (0xFFFUL << EXTDMA_CMPRSR_LINESIZE_Pos)
#define EXTDMA_CMPRSR_LINESIZE          EXTDMA_CMPRSR_LINESIZE_Msk
#define EXTDMA_CMPRSR_TGTSIZE_Pos       (16U)
#define EXTDMA_CMPRSR_TGTSIZE_Msk       (0xFFFUL << EXTDMA_CMPRSR_TGTSIZE_Pos)
#define EXTDMA_CMPRSR_TGTSIZE           EXTDMA_CMPRSR_TGTSIZE_Msk

/**************** Bit definition for EXTDMA_CMPRNDTR register *****************/
#define EXTDMA_CMPRNDTR_CMPRNDT_Pos     (0U)
#define EXTDMA_CMPRNDTR_CMPRNDT_Msk     (0xFFFFFUL << EXTDMA_CMPRNDTR_CMPRNDT_Pos)
#define EXTDMA_CMPRNDTR_CMPRNDT         EXTDMA_CMPRNDTR_CMPRNDT_Msk

/**************** Bit definition for EXTDMA_CMPRCFG0 register *****************/
#define EXTDMA_CMPRCFG0_CFG_Pos         (0U)
#define EXTDMA_CMPRCFG0_CFG_Msk         (0xFFFFFFFFUL << EXTDMA_CMPRCFG0_CFG_Pos)
#define EXTDMA_CMPRCFG0_CFG             EXTDMA_CMPRCFG0_CFG_Msk

/**************** Bit definition for EXTDMA_CMPRCFG1 register *****************/
#define EXTDMA_CMPRCFG1_CFG_Pos         (0U)
#define EXTDMA_CMPRCFG1_CFG_Msk         (0xFFFFFFFFUL << EXTDMA_CMPRCFG1_CFG_Pos)
#define EXTDMA_CMPRCFG1_CFG             EXTDMA_CMPRCFG1_CFG_Msk

/***************** Bit definition for EXTDMA_CMPRQR register ******************/
#define EXTDMA_CMPRQR_LQR_Pos           (0U)
#define EXTDMA_CMPRQR_LQR_Msk           (0xFFUL << EXTDMA_CMPRQR_LQR_Pos)
#define EXTDMA_CMPRQR_LQR               EXTDMA_CMPRQR_LQR_Msk
#define EXTDMA_CMPRQR_LQB_Pos           (8U)
#define EXTDMA_CMPRQR_LQB_Msk           (0xFFUL << EXTDMA_CMPRQR_LQB_Pos)
#define EXTDMA_CMPRQR_LQB               EXTDMA_CMPRQR_LQB_Msk
#define EXTDMA_CMPRQR_DUMMY_Pos         (16U)
#define EXTDMA_CMPRQR_DUMMY_Msk         (0x1FFFUL << EXTDMA_CMPRQR_DUMMY_Pos)
#define EXTDMA_CMPRQR_DUMMY             EXTDMA_CMPRQR_DUMMY_Msk

/***************** Bit definition for EXTDMA_CMPRDR register ******************/
#define EXTDMA_CMPRDR_MAXBUF_Pos        (0U)
#define EXTDMA_CMPRDR_MAXBUF_Msk        (0x7FUL << EXTDMA_CMPRDR_MAXBUF_Pos)
#define EXTDMA_CMPRDR_MAXBUF            EXTDMA_CMPRDR_MAXBUF_Msk

#endif
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
