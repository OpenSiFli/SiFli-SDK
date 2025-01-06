/**
  ******************************************************************************
  * @file   sdmmc.h
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
#ifndef __SDMMC_H
#define __SDMMC_H

typedef struct
{
    __IO uint32_t SAR;
    __IO uint32_t BSR;
    __IO uint32_t ARG1;
    __IO uint32_t CR1;
    __IO uint32_t RSP1;
    __IO uint32_t RSP2;
    __IO uint32_t RSP3;
    __IO uint32_t RSP4;
    __IO uint32_t BUF;
    __IO uint32_t SR;
    __IO uint32_t CR2;
    __IO uint32_t CR3;
    __IO uint32_t ISR;
    __IO uint32_t ISER;
    __IO uint32_t IER;
    __IO uint32_t CR4;
    __IO uint32_t CAP1;
    __IO uint32_t CAP2;
    __IO uint32_t CAP3;
    __IO uint32_t RSVD4;
    __IO uint32_t FER;
    __IO uint32_t AESR;
    __IO uint32_t ASAR;
    __IO uint32_t RSVD3;
    __IO uint32_t PVR1;
    __IO uint32_t PVR2;
    __IO uint32_t PVR3;
    __IO uint32_t PVR4;
    __IO uint32_t RSVD2[31];
    __IO uint32_t ABSR;
    __IO uint32_t RSVD1[3];
    __IO uint32_t VER;
} SDMMC_TypeDef;


/******************* Bit definition for SDMMC_SAR register ********************/
#define SDMMC_SAR_ADDR_Pos              (0U)
#define SDMMC_SAR_ADDR_Msk              (0xFFFFFFFFUL << SDMMC_SAR_ADDR_Pos)
#define SDMMC_SAR_ADDR                  SDMMC_SAR_ADDR_Msk

/******************* Bit definition for SDMMC_BSR register ********************/
#define SDMMC_BSR_BSIZE_Pos             (0U)
#define SDMMC_BSR_BSIZE_Msk             (0xFFFUL << SDMMC_BSR_BSIZE_Pos)
#define SDMMC_BSR_BSIZE                 SDMMC_BSR_BSIZE_Msk
#define SDMMC_BSR_SDMABDY_Pos           (12U)
#define SDMMC_BSR_SDMABDY_Msk           (0x7UL << SDMMC_BSR_SDMABDY_Pos)
#define SDMMC_BSR_SDMABDY               SDMMC_BSR_SDMABDY_Msk
#define SDMMC_BSR_BCNT_Pos              (16U)
#define SDMMC_BSR_BCNT_Msk              (0xFFFFUL << SDMMC_BSR_BCNT_Pos)
#define SDMMC_BSR_BCNT                  SDMMC_BSR_BCNT_Msk

/******************* Bit definition for SDMMC_ARG1 register *******************/
#define SDMMC_ARG1_ARG_Pos              (0U)
#define SDMMC_ARG1_ARG_Msk              (0xFFFFFFFFUL << SDMMC_ARG1_ARG_Pos)
#define SDMMC_ARG1_ARG                  SDMMC_ARG1_ARG_Msk

/******************* Bit definition for SDMMC_CR1 register ********************/
#define SDMMC_CR1_DMAEN_Pos             (0U)
#define SDMMC_CR1_DMAEN_Msk             (0x1UL << SDMMC_CR1_DMAEN_Pos)
#define SDMMC_CR1_DMAEN                 SDMMC_CR1_DMAEN_Msk
#define SDMMC_CR1_BCNTEN_Pos            (1U)
#define SDMMC_CR1_BCNTEN_Msk            (0x1UL << SDMMC_CR1_BCNTEN_Pos)
#define SDMMC_CR1_BCNTEN                SDMMC_CR1_BCNTEN_Msk
#define SDMMC_CR1_AUTOCMD_Pos           (2U)
#define SDMMC_CR1_AUTOCMD_Msk           (0x3UL << SDMMC_CR1_AUTOCMD_Pos)
#define SDMMC_CR1_AUTOCMD               SDMMC_CR1_AUTOCMD_Msk
#define SDMMC_CR1_DIR_Pos               (4U)
#define SDMMC_CR1_DIR_Msk               (0x1UL << SDMMC_CR1_DIR_Pos)
#define SDMMC_CR1_DIR                   SDMMC_CR1_DIR_Msk
#define SDMMC_CR1_MULTIBLK_Pos          (5U)
#define SDMMC_CR1_MULTIBLK_Msk          (0x1UL << SDMMC_CR1_MULTIBLK_Pos)
#define SDMMC_CR1_MULTIBLK              SDMMC_CR1_MULTIBLK_Msk
#define SDMMC_CR1_STREAMMODE_Pos        (11U)
#define SDMMC_CR1_STREAMMODE_Msk        (0x1UL << SDMMC_CR1_STREAMMODE_Pos)
#define SDMMC_CR1_STREAMMODE            SDMMC_CR1_STREAMMODE_Msk
#define SDMMC_CR1_SPIMODE_Pos           (12U)
#define SDMMC_CR1_SPIMODE_Msk           (0x1UL << SDMMC_CR1_SPIMODE_Pos)
#define SDMMC_CR1_SPIMODE               SDMMC_CR1_SPIMODE_Msk
#define SDMMC_CR1_BOOTACK_Pos           (13U)
#define SDMMC_CR1_BOOTACK_Msk           (0x1UL << SDMMC_CR1_BOOTACK_Pos)
#define SDMMC_CR1_BOOTACK               SDMMC_CR1_BOOTACK_Msk
#define SDMMC_CR1_ALTERBOOT_Pos         (14U)
#define SDMMC_CR1_ALTERBOOT_Msk         (0x1UL << SDMMC_CR1_ALTERBOOT_Pos)
#define SDMMC_CR1_ALTERBOOT             SDMMC_CR1_ALTERBOOT_Msk
#define SDMMC_CR1_BOOT_Pos              (15U)
#define SDMMC_CR1_BOOT_Msk              (0x1UL << SDMMC_CR1_BOOT_Pos)
#define SDMMC_CR1_BOOT                  SDMMC_CR1_BOOT_Msk
#define SDMMC_CR1_RSPTYPE_Pos           (16U)
#define SDMMC_CR1_RSPTYPE_Msk           (0x3UL << SDMMC_CR1_RSPTYPE_Pos)
#define SDMMC_CR1_RSPTYPE               SDMMC_CR1_RSPTYPE_Msk
#define SDMMC_CR1_CHECKCRC_Pos          (19U)
#define SDMMC_CR1_CHECKCRC_Msk          (0x1UL << SDMMC_CR1_CHECKCRC_Pos)
#define SDMMC_CR1_CHECKCRC              SDMMC_CR1_CHECKCRC_Msk
#define SDMMC_CR1_CHECKIDX_Pos          (20U)
#define SDMMC_CR1_CHECKIDX_Msk          (0x1UL << SDMMC_CR1_CHECKIDX_Pos)
#define SDMMC_CR1_CHECKIDX              SDMMC_CR1_CHECKIDX_Msk
#define SDMMC_CR1_DATAPRESENT_Pos       (21U)
#define SDMMC_CR1_DATAPRESENT_Msk       (0x1UL << SDMMC_CR1_DATAPRESENT_Pos)
#define SDMMC_CR1_DATAPRESENT           SDMMC_CR1_DATAPRESENT_Msk
#define SDMMC_CR1_CMDTYPE_Pos           (22U)
#define SDMMC_CR1_CMDTYPE_Msk           (0x3UL << SDMMC_CR1_CMDTYPE_Pos)
#define SDMMC_CR1_CMDTYPE               SDMMC_CR1_CMDTYPE_Msk
#define SDMMC_CR1_CMDIDX_Pos            (24U)
#define SDMMC_CR1_CMDIDX_Msk            (0x3FUL << SDMMC_CR1_CMDIDX_Pos)
#define SDMMC_CR1_CMDIDX                SDMMC_CR1_CMDIDX_Msk

/******************* Bit definition for SDMMC_RSP1 register *******************/
#define SDMMC_RSP1_RSP_Pos              (0U)
#define SDMMC_RSP1_RSP_Msk              (0xFFFFFFFFUL << SDMMC_RSP1_RSP_Pos)
#define SDMMC_RSP1_RSP                  SDMMC_RSP1_RSP_Msk

/******************* Bit definition for SDMMC_RSP2 register *******************/
#define SDMMC_RSP2_RSP_Pos              (0U)
#define SDMMC_RSP2_RSP_Msk              (0xFFFFFFFFUL << SDMMC_RSP2_RSP_Pos)
#define SDMMC_RSP2_RSP                  SDMMC_RSP2_RSP_Msk

/******************* Bit definition for SDMMC_RSP3 register *******************/
#define SDMMC_RSP3_RSP_Pos              (0U)
#define SDMMC_RSP3_RSP_Msk              (0xFFFFFFFFUL << SDMMC_RSP3_RSP_Pos)
#define SDMMC_RSP3_RSP                  SDMMC_RSP3_RSP_Msk

/******************* Bit definition for SDMMC_RSP4 register *******************/
#define SDMMC_RSP4_RSP_Pos              (0U)
#define SDMMC_RSP4_RSP_Msk              (0xFFFFFFFFUL << SDMMC_RSP4_RSP_Pos)
#define SDMMC_RSP4_RSP                  SDMMC_RSP4_RSP_Msk

/******************* Bit definition for SDMMC_BUF register ********************/
#define SDMMC_BUF_DATA_Pos              (0U)
#define SDMMC_BUF_DATA_Msk              (0xFFFFFFFFUL << SDMMC_BUF_DATA_Pos)
#define SDMMC_BUF_DATA                  SDMMC_BUF_DATA_Msk

/******************** Bit definition for SDMMC_SR register ********************/
#define SDMMC_SR_CMDBUSY_Pos            (0U)
#define SDMMC_SR_CMDBUSY_Msk            (0x1UL << SDMMC_SR_CMDBUSY_Pos)
#define SDMMC_SR_CMDBUSY                SDMMC_SR_CMDBUSY_Msk
#define SDMMC_SR_DATBUSY_Pos            (1U)
#define SDMMC_SR_DATBUSY_Msk            (0x1UL << SDMMC_SR_DATBUSY_Pos)
#define SDMMC_SR_DATBUSY                SDMMC_SR_DATBUSY_Msk
#define SDMMC_SR_DATACTIVE_Pos          (2U)
#define SDMMC_SR_DATACTIVE_Msk          (0x1UL << SDMMC_SR_DATACTIVE_Pos)
#define SDMMC_SR_DATACTIVE              SDMMC_SR_DATACTIVE_Msk
#define SDMMC_SR_RETUNEREQ_Pos          (3U)
#define SDMMC_SR_RETUNEREQ_Msk          (0x1UL << SDMMC_SR_RETUNEREQ_Pos)
#define SDMMC_SR_RETUNEREQ              SDMMC_SR_RETUNEREQ_Msk
#define SDMMC_SR_WACTIVE_Pos            (8U)
#define SDMMC_SR_WACTIVE_Msk            (0x1UL << SDMMC_SR_WACTIVE_Pos)
#define SDMMC_SR_WACTIVE                SDMMC_SR_WACTIVE_Msk
#define SDMMC_SR_RACTIVE_Pos            (9U)
#define SDMMC_SR_RACTIVE_Msk            (0x1UL << SDMMC_SR_RACTIVE_Pos)
#define SDMMC_SR_RACTIVE                SDMMC_SR_RACTIVE_Msk
#define SDMMC_SR_BUFWEN_Pos             (10U)
#define SDMMC_SR_BUFWEN_Msk             (0x1UL << SDMMC_SR_BUFWEN_Pos)
#define SDMMC_SR_BUFWEN                 SDMMC_SR_BUFWEN_Msk
#define SDMMC_SR_BUFREN_Pos             (11U)
#define SDMMC_SR_BUFREN_Msk             (0x1UL << SDMMC_SR_BUFREN_Pos)
#define SDMMC_SR_BUFREN                 SDMMC_SR_BUFREN_Msk
#define SDMMC_SR_CINSERT_Pos            (16U)
#define SDMMC_SR_CINSERT_Msk            (0x1UL << SDMMC_SR_CINSERT_Pos)
#define SDMMC_SR_CINSERT                SDMMC_SR_CINSERT_Msk
#define SDMMC_SR_CSTABLE_Pos            (17U)
#define SDMMC_SR_CSTABLE_Msk            (0x1UL << SDMMC_SR_CSTABLE_Pos)
#define SDMMC_SR_CSTABLE                SDMMC_SR_CSTABLE_Msk
#define SDMMC_SR_CDLVL_Pos              (18U)
#define SDMMC_SR_CDLVL_Msk              (0x1UL << SDMMC_SR_CDLVL_Pos)
#define SDMMC_SR_CDLVL                  SDMMC_SR_CDLVL_Msk
#define SDMMC_SR_WPLVL_Pos              (19U)
#define SDMMC_SR_WPLVL_Msk              (0x1UL << SDMMC_SR_WPLVL_Pos)
#define SDMMC_SR_WPLVL                  SDMMC_SR_WPLVL_Msk
#define SDMMC_SR_DATLVL_Pos             (20U)
#define SDMMC_SR_DATLVL_Msk             (0xFUL << SDMMC_SR_DATLVL_Pos)
#define SDMMC_SR_DATLVL                 SDMMC_SR_DATLVL_Msk
#define SDMMC_SR_CMDLVL_Pos             (24U)
#define SDMMC_SR_CMDLVL_Msk             (0x1UL << SDMMC_SR_CMDLVL_Pos)
#define SDMMC_SR_CMDLVL                 SDMMC_SR_CMDLVL_Msk

/******************* Bit definition for SDMMC_CR2 register ********************/
#define SDMMC_CR2_LEDON_Pos             (0U)
#define SDMMC_CR2_LEDON_Msk             (0x1UL << SDMMC_CR2_LEDON_Pos)
#define SDMMC_CR2_LEDON                 SDMMC_CR2_LEDON_Msk
#define SDMMC_CR2_DATWIDTH_Pos          (1U)
#define SDMMC_CR2_DATWIDTH_Msk          (0x1UL << SDMMC_CR2_DATWIDTH_Pos)
#define SDMMC_CR2_DATWIDTH              SDMMC_CR2_DATWIDTH_Msk
#define SDMMC_CR2_HSMODE_Pos            (2U)
#define SDMMC_CR2_HSMODE_Msk            (0x1UL << SDMMC_CR2_HSMODE_Pos)
#define SDMMC_CR2_HSMODE                SDMMC_CR2_HSMODE_Msk
#define SDMMC_CR2_DMAMODE_Pos           (3U)
#define SDMMC_CR2_DMAMODE_Msk           (0x3UL << SDMMC_CR2_DMAMODE_Pos)
#define SDMMC_CR2_DMAMODE               SDMMC_CR2_DMAMODE_Msk
#define SDMMC_CR2_EXTWIDTH_Pos          (5U)
#define SDMMC_CR2_EXTWIDTH_Msk          (0x1UL << SDMMC_CR2_EXTWIDTH_Pos)
#define SDMMC_CR2_EXTWIDTH              SDMMC_CR2_EXTWIDTH_Msk
#define SDMMC_CR2_CDTSTLVL_Pos          (6U)
#define SDMMC_CR2_CDTSTLVL_Msk          (0x1UL << SDMMC_CR2_CDTSTLVL_Pos)
#define SDMMC_CR2_CDTSTLVL              SDMMC_CR2_CDTSTLVL_Msk
#define SDMMC_CR2_CDTST_Pos             (7U)
#define SDMMC_CR2_CDTST_Msk             (0x1UL << SDMMC_CR2_CDTST_Pos)
#define SDMMC_CR2_CDTST                 SDMMC_CR2_CDTST_Msk
#define SDMMC_CR2_PWRON_Pos             (8U)
#define SDMMC_CR2_PWRON_Msk             (0x1UL << SDMMC_CR2_PWRON_Pos)
#define SDMMC_CR2_PWRON                 SDMMC_CR2_PWRON_Msk
#define SDMMC_CR2_VOLTSEL_Pos           (9U)
#define SDMMC_CR2_VOLTSEL_Msk           (0x7UL << SDMMC_CR2_VOLTSEL_Pos)
#define SDMMC_CR2_VOLTSEL               SDMMC_CR2_VOLTSEL_Msk
#define SDMMC_CR2_PPMODE_Pos            (12U)
#define SDMMC_CR2_PPMODE_Msk            (0x1UL << SDMMC_CR2_PPMODE_Pos)
#define SDMMC_CR2_PPMODE                SDMMC_CR2_PPMODE_Msk
#define SDMMC_CR2_RSTN_Pos              (13U)
#define SDMMC_CR2_RSTN_Msk              (0x1UL << SDMMC_CR2_RSTN_Pos)
#define SDMMC_CR2_RSTN                  SDMMC_CR2_RSTN_Msk
#define SDMMC_CR2_BLKGAPSTOP_Pos        (16U)
#define SDMMC_CR2_BLKGAPSTOP_Msk        (0x1UL << SDMMC_CR2_BLKGAPSTOP_Pos)
#define SDMMC_CR2_BLKGAPSTOP            SDMMC_CR2_BLKGAPSTOP_Msk
#define SDMMC_CR2_CONTREQ_Pos           (17U)
#define SDMMC_CR2_CONTREQ_Msk           (0x1UL << SDMMC_CR2_CONTREQ_Pos)
#define SDMMC_CR2_CONTREQ               SDMMC_CR2_CONTREQ_Msk
#define SDMMC_CR2_RWAITEN_Pos           (18U)
#define SDMMC_CR2_RWAITEN_Msk           (0x1UL << SDMMC_CR2_RWAITEN_Pos)
#define SDMMC_CR2_RWAITEN               SDMMC_CR2_RWAITEN_Msk
#define SDMMC_CR2_BLKGAPIE_Pos          (19U)
#define SDMMC_CR2_BLKGAPIE_Msk          (0x1UL << SDMMC_CR2_BLKGAPIE_Pos)
#define SDMMC_CR2_BLKGAPIE              SDMMC_CR2_BLKGAPIE_Msk
#define SDMMC_CR2_INTWE_Pos             (24U)
#define SDMMC_CR2_INTWE_Msk             (0x1UL << SDMMC_CR2_INTWE_Pos)
#define SDMMC_CR2_INTWE                 SDMMC_CR2_INTWE_Msk
#define SDMMC_CR2_INSERTWE_Pos          (25U)
#define SDMMC_CR2_INSERTWE_Msk          (0x1UL << SDMMC_CR2_INSERTWE_Pos)
#define SDMMC_CR2_INSERTWE              SDMMC_CR2_INSERTWE_Msk
#define SDMMC_CR2_REMOVWE_Pos           (26U)
#define SDMMC_CR2_REMOVWE_Msk           (0x1UL << SDMMC_CR2_REMOVWE_Pos)
#define SDMMC_CR2_REMOVWE               SDMMC_CR2_REMOVWE_Msk

/******************* Bit definition for SDMMC_CR3 register ********************/
#define SDMMC_CR3_INTCLKEN_Pos          (0U)
#define SDMMC_CR3_INTCLKEN_Msk          (0x1UL << SDMMC_CR3_INTCLKEN_Pos)
#define SDMMC_CR3_INTCLKEN              SDMMC_CR3_INTCLKEN_Msk
#define SDMMC_CR3_INTCLKRDY_Pos         (1U)
#define SDMMC_CR3_INTCLKRDY_Msk         (0x1UL << SDMMC_CR3_INTCLKRDY_Pos)
#define SDMMC_CR3_INTCLKRDY             SDMMC_CR3_INTCLKRDY_Msk
#define SDMMC_CR3_CLKEN_Pos             (2U)
#define SDMMC_CR3_CLKEN_Msk             (0x1UL << SDMMC_CR3_CLKEN_Pos)
#define SDMMC_CR3_CLKEN                 SDMMC_CR3_CLKEN_Msk
#define SDMMC_CR3_CLKSEL_Pos            (5U)
#define SDMMC_CR3_CLKSEL_Msk            (0x1UL << SDMMC_CR3_CLKSEL_Pos)
#define SDMMC_CR3_CLKSEL                SDMMC_CR3_CLKSEL_Msk
#define SDMMC_CR3_CLKDIVU_Pos           (6U)
#define SDMMC_CR3_CLKDIVU_Msk           (0x3UL << SDMMC_CR3_CLKDIVU_Pos)
#define SDMMC_CR3_CLKDIVU               SDMMC_CR3_CLKDIVU_Msk
#define SDMMC_CR3_CLKDIVL_Pos           (8U)
#define SDMMC_CR3_CLKDIVL_Msk           (0xFFUL << SDMMC_CR3_CLKDIVL_Pos)
#define SDMMC_CR3_CLKDIVL               SDMMC_CR3_CLKDIVL_Msk
#define SDMMC_CR3_DTOCNT_Pos            (16U)
#define SDMMC_CR3_DTOCNT_Msk            (0xFUL << SDMMC_CR3_DTOCNT_Pos)
#define SDMMC_CR3_DTOCNT                SDMMC_CR3_DTOCNT_Msk
#define SDMMC_CR3_RST_Pos               (24U)
#define SDMMC_CR3_RST_Msk               (0x1UL << SDMMC_CR3_RST_Pos)
#define SDMMC_CR3_RST                   SDMMC_CR3_RST_Msk
#define SDMMC_CR3_RSTCMD_Pos            (25U)
#define SDMMC_CR3_RSTCMD_Msk            (0x1UL << SDMMC_CR3_RSTCMD_Pos)
#define SDMMC_CR3_RSTCMD                SDMMC_CR3_RSTCMD_Msk
#define SDMMC_CR3_RSTDAT_Pos            (26U)
#define SDMMC_CR3_RSTDAT_Msk            (0x1UL << SDMMC_CR3_RSTDAT_Pos)
#define SDMMC_CR3_RSTDAT                SDMMC_CR3_RSTDAT_Msk

/******************* Bit definition for SDMMC_ISR register ********************/
#define SDMMC_ISR_CC_Pos                (0U)
#define SDMMC_ISR_CC_Msk                (0x1UL << SDMMC_ISR_CC_Pos)
#define SDMMC_ISR_CC                    SDMMC_ISR_CC_Msk
#define SDMMC_ISR_TC_Pos                (1U)
#define SDMMC_ISR_TC_Msk                (0x1UL << SDMMC_ISR_TC_Pos)
#define SDMMC_ISR_TC                    SDMMC_ISR_TC_Msk
#define SDMMC_ISR_BLKGAP_Pos            (2U)
#define SDMMC_ISR_BLKGAP_Msk            (0x1UL << SDMMC_ISR_BLKGAP_Pos)
#define SDMMC_ISR_BLKGAP                SDMMC_ISR_BLKGAP_Msk
#define SDMMC_ISR_DMA_Pos               (3U)
#define SDMMC_ISR_DMA_Msk               (0x1UL << SDMMC_ISR_DMA_Pos)
#define SDMMC_ISR_DMA                   SDMMC_ISR_DMA_Msk
#define SDMMC_ISR_BUFWRDY_Pos           (4U)
#define SDMMC_ISR_BUFWRDY_Msk           (0x1UL << SDMMC_ISR_BUFWRDY_Pos)
#define SDMMC_ISR_BUFWRDY               SDMMC_ISR_BUFWRDY_Msk
#define SDMMC_ISR_BUFRRDY_Pos           (5U)
#define SDMMC_ISR_BUFRRDY_Msk           (0x1UL << SDMMC_ISR_BUFRRDY_Pos)
#define SDMMC_ISR_BUFRRDY               SDMMC_ISR_BUFRRDY_Msk
#define SDMMC_ISR_CINSERT_Pos           (6U)
#define SDMMC_ISR_CINSERT_Msk           (0x1UL << SDMMC_ISR_CINSERT_Pos)
#define SDMMC_ISR_CINSERT               SDMMC_ISR_CINSERT_Msk
#define SDMMC_ISR_CREMOV_Pos            (7U)
#define SDMMC_ISR_CREMOV_Msk            (0x1UL << SDMMC_ISR_CREMOV_Pos)
#define SDMMC_ISR_CREMOV                SDMMC_ISR_CREMOV_Msk
#define SDMMC_ISR_CARD_Pos              (8U)
#define SDMMC_ISR_CARD_Msk              (0x1UL << SDMMC_ISR_CARD_Pos)
#define SDMMC_ISR_CARD                  SDMMC_ISR_CARD_Msk
#define SDMMC_ISR_INTA_Pos              (9U)
#define SDMMC_ISR_INTA_Msk              (0x1UL << SDMMC_ISR_INTA_Pos)
#define SDMMC_ISR_INTA                  SDMMC_ISR_INTA_Msk
#define SDMMC_ISR_INTB_Pos              (10U)
#define SDMMC_ISR_INTB_Msk              (0x1UL << SDMMC_ISR_INTB_Pos)
#define SDMMC_ISR_INTB                  SDMMC_ISR_INTB_Msk
#define SDMMC_ISR_INTC_Pos              (11U)
#define SDMMC_ISR_INTC_Msk              (0x1UL << SDMMC_ISR_INTC_Pos)
#define SDMMC_ISR_INTC                  SDMMC_ISR_INTC_Msk
#define SDMMC_ISR_RETUNE_Pos            (12U)
#define SDMMC_ISR_RETUNE_Msk            (0x1UL << SDMMC_ISR_RETUNE_Pos)
#define SDMMC_ISR_RETUNE                SDMMC_ISR_RETUNE_Msk
#define SDMMC_ISR_BOOTACK_Pos           (13U)
#define SDMMC_ISR_BOOTACK_Msk           (0x1UL << SDMMC_ISR_BOOTACK_Pos)
#define SDMMC_ISR_BOOTACK               SDMMC_ISR_BOOTACK_Msk
#define SDMMC_ISR_BOOTDONE_Pos          (14U)
#define SDMMC_ISR_BOOTDONE_Msk          (0x1UL << SDMMC_ISR_BOOTDONE_Pos)
#define SDMMC_ISR_BOOTDONE              SDMMC_ISR_BOOTDONE_Msk
#define SDMMC_ISR_ERR_Pos               (15U)
#define SDMMC_ISR_ERR_Msk               (0x1UL << SDMMC_ISR_ERR_Pos)
#define SDMMC_ISR_ERR                   SDMMC_ISR_ERR_Msk
#define SDMMC_ISR_CTOERR_Pos            (16U)
#define SDMMC_ISR_CTOERR_Msk            (0x1UL << SDMMC_ISR_CTOERR_Pos)
#define SDMMC_ISR_CTOERR                SDMMC_ISR_CTOERR_Msk
#define SDMMC_ISR_CCRCERR_Pos           (17U)
#define SDMMC_ISR_CCRCERR_Msk           (0x1UL << SDMMC_ISR_CCRCERR_Pos)
#define SDMMC_ISR_CCRCERR               SDMMC_ISR_CCRCERR_Msk
#define SDMMC_ISR_CEBERR_Pos            (18U)
#define SDMMC_ISR_CEBERR_Msk            (0x1UL << SDMMC_ISR_CEBERR_Pos)
#define SDMMC_ISR_CEBERR                SDMMC_ISR_CEBERR_Msk
#define SDMMC_ISR_IDXERR_Pos            (19U)
#define SDMMC_ISR_IDXERR_Msk            (0x1UL << SDMMC_ISR_IDXERR_Pos)
#define SDMMC_ISR_IDXERR                SDMMC_ISR_IDXERR_Msk
#define SDMMC_ISR_DTOERR_Pos            (20U)
#define SDMMC_ISR_DTOERR_Msk            (0x1UL << SDMMC_ISR_DTOERR_Pos)
#define SDMMC_ISR_DTOERR                SDMMC_ISR_DTOERR_Msk
#define SDMMC_ISR_DCRCERR_Pos           (21U)
#define SDMMC_ISR_DCRCERR_Msk           (0x1UL << SDMMC_ISR_DCRCERR_Pos)
#define SDMMC_ISR_DCRCERR               SDMMC_ISR_DCRCERR_Msk
#define SDMMC_ISR_DEBERR_Pos            (22U)
#define SDMMC_ISR_DEBERR_Msk            (0x1UL << SDMMC_ISR_DEBERR_Pos)
#define SDMMC_ISR_DEBERR                SDMMC_ISR_DEBERR_Msk
#define SDMMC_ISR_CLERR_Pos             (23U)
#define SDMMC_ISR_CLERR_Msk             (0x1UL << SDMMC_ISR_CLERR_Pos)
#define SDMMC_ISR_CLERR                 SDMMC_ISR_CLERR_Msk
#define SDMMC_ISR_ACERR_Pos             (24U)
#define SDMMC_ISR_ACERR_Msk             (0x1UL << SDMMC_ISR_ACERR_Pos)
#define SDMMC_ISR_ACERR                 SDMMC_ISR_ACERR_Msk
#define SDMMC_ISR_ADMAERR_Pos           (25U)
#define SDMMC_ISR_ADMAERR_Msk           (0x1UL << SDMMC_ISR_ADMAERR_Pos)
#define SDMMC_ISR_ADMAERR               SDMMC_ISR_ADMAERR_Msk
#define SDMMC_ISR_TUNEERR_Pos           (26U)
#define SDMMC_ISR_TUNEERR_Msk           (0x1UL << SDMMC_ISR_TUNEERR_Pos)
#define SDMMC_ISR_TUNEERR               SDMMC_ISR_TUNEERR_Msk

/******************* Bit definition for SDMMC_ISER register *******************/
#define SDMMC_ISER_CCEN_Pos             (0U)
#define SDMMC_ISER_CCEN_Msk             (0x1UL << SDMMC_ISER_CCEN_Pos)
#define SDMMC_ISER_CCEN                 SDMMC_ISER_CCEN_Msk
#define SDMMC_ISER_TCEN_Pos             (1U)
#define SDMMC_ISER_TCEN_Msk             (0x1UL << SDMMC_ISER_TCEN_Pos)
#define SDMMC_ISER_TCEN                 SDMMC_ISER_TCEN_Msk
#define SDMMC_ISER_BLKGAPEN_Pos         (2U)
#define SDMMC_ISER_BLKGAPEN_Msk         (0x1UL << SDMMC_ISER_BLKGAPEN_Pos)
#define SDMMC_ISER_BLKGAPEN             SDMMC_ISER_BLKGAPEN_Msk
#define SDMMC_ISER_DMAEN_Pos            (3U)
#define SDMMC_ISER_DMAEN_Msk            (0x1UL << SDMMC_ISER_DMAEN_Pos)
#define SDMMC_ISER_DMAEN                SDMMC_ISER_DMAEN_Msk
#define SDMMC_ISER_BUFWRDYEN_Pos        (4U)
#define SDMMC_ISER_BUFWRDYEN_Msk        (0x1UL << SDMMC_ISER_BUFWRDYEN_Pos)
#define SDMMC_ISER_BUFWRDYEN            SDMMC_ISER_BUFWRDYEN_Msk
#define SDMMC_ISER_BUFRRDYEN_Pos        (5U)
#define SDMMC_ISER_BUFRRDYEN_Msk        (0x1UL << SDMMC_ISER_BUFRRDYEN_Pos)
#define SDMMC_ISER_BUFRRDYEN            SDMMC_ISER_BUFRRDYEN_Msk
#define SDMMC_ISER_CINSERTEN_Pos        (6U)
#define SDMMC_ISER_CINSERTEN_Msk        (0x1UL << SDMMC_ISER_CINSERTEN_Pos)
#define SDMMC_ISER_CINSERTEN            SDMMC_ISER_CINSERTEN_Msk
#define SDMMC_ISER_CREMOVEN_Pos         (7U)
#define SDMMC_ISER_CREMOVEN_Msk         (0x1UL << SDMMC_ISER_CREMOVEN_Pos)
#define SDMMC_ISER_CREMOVEN             SDMMC_ISER_CREMOVEN_Msk
#define SDMMC_ISER_CARDEN_Pos           (8U)
#define SDMMC_ISER_CARDEN_Msk           (0x1UL << SDMMC_ISER_CARDEN_Pos)
#define SDMMC_ISER_CARDEN               SDMMC_ISER_CARDEN_Msk
#define SDMMC_ISER_INTAEN_Pos           (9U)
#define SDMMC_ISER_INTAEN_Msk           (0x1UL << SDMMC_ISER_INTAEN_Pos)
#define SDMMC_ISER_INTAEN               SDMMC_ISER_INTAEN_Msk
#define SDMMC_ISER_INTBEN_Pos           (10U)
#define SDMMC_ISER_INTBEN_Msk           (0x1UL << SDMMC_ISER_INTBEN_Pos)
#define SDMMC_ISER_INTBEN               SDMMC_ISER_INTBEN_Msk
#define SDMMC_ISER_INTCEN_Pos           (11U)
#define SDMMC_ISER_INTCEN_Msk           (0x1UL << SDMMC_ISER_INTCEN_Pos)
#define SDMMC_ISER_INTCEN               SDMMC_ISER_INTCEN_Msk
#define SDMMC_ISER_RETUNEEN_Pos         (12U)
#define SDMMC_ISER_RETUNEEN_Msk         (0x1UL << SDMMC_ISER_RETUNEEN_Pos)
#define SDMMC_ISER_RETUNEEN             SDMMC_ISER_RETUNEEN_Msk
#define SDMMC_ISER_BOOTACKEN_Pos        (13U)
#define SDMMC_ISER_BOOTACKEN_Msk        (0x1UL << SDMMC_ISER_BOOTACKEN_Pos)
#define SDMMC_ISER_BOOTACKEN            SDMMC_ISER_BOOTACKEN_Msk
#define SDMMC_ISER_BOOTDONEEN_Pos       (14U)
#define SDMMC_ISER_BOOTDONEEN_Msk       (0x1UL << SDMMC_ISER_BOOTDONEEN_Pos)
#define SDMMC_ISER_BOOTDONEEN           SDMMC_ISER_BOOTDONEEN_Msk
#define SDMMC_ISER_CTOERREN_Pos         (16U)
#define SDMMC_ISER_CTOERREN_Msk         (0x1UL << SDMMC_ISER_CTOERREN_Pos)
#define SDMMC_ISER_CTOERREN             SDMMC_ISER_CTOERREN_Msk
#define SDMMC_ISER_CCRCERREN_Pos        (17U)
#define SDMMC_ISER_CCRCERREN_Msk        (0x1UL << SDMMC_ISER_CCRCERREN_Pos)
#define SDMMC_ISER_CCRCERREN            SDMMC_ISER_CCRCERREN_Msk
#define SDMMC_ISER_CEBERREN_Pos         (18U)
#define SDMMC_ISER_CEBERREN_Msk         (0x1UL << SDMMC_ISER_CEBERREN_Pos)
#define SDMMC_ISER_CEBERREN             SDMMC_ISER_CEBERREN_Msk
#define SDMMC_ISER_IDXERREN_Pos         (19U)
#define SDMMC_ISER_IDXERREN_Msk         (0x1UL << SDMMC_ISER_IDXERREN_Pos)
#define SDMMC_ISER_IDXERREN             SDMMC_ISER_IDXERREN_Msk
#define SDMMC_ISER_DTOERREN_Pos         (20U)
#define SDMMC_ISER_DTOERREN_Msk         (0x1UL << SDMMC_ISER_DTOERREN_Pos)
#define SDMMC_ISER_DTOERREN             SDMMC_ISER_DTOERREN_Msk
#define SDMMC_ISER_DCRCERREN_Pos        (21U)
#define SDMMC_ISER_DCRCERREN_Msk        (0x1UL << SDMMC_ISER_DCRCERREN_Pos)
#define SDMMC_ISER_DCRCERREN            SDMMC_ISER_DCRCERREN_Msk
#define SDMMC_ISER_DEBERREN_Pos         (22U)
#define SDMMC_ISER_DEBERREN_Msk         (0x1UL << SDMMC_ISER_DEBERREN_Pos)
#define SDMMC_ISER_DEBERREN             SDMMC_ISER_DEBERREN_Msk
#define SDMMC_ISER_CLERREN_Pos          (23U)
#define SDMMC_ISER_CLERREN_Msk          (0x1UL << SDMMC_ISER_CLERREN_Pos)
#define SDMMC_ISER_CLERREN              SDMMC_ISER_CLERREN_Msk
#define SDMMC_ISER_ACERREN_Pos          (24U)
#define SDMMC_ISER_ACERREN_Msk          (0x1UL << SDMMC_ISER_ACERREN_Pos)
#define SDMMC_ISER_ACERREN              SDMMC_ISER_ACERREN_Msk
#define SDMMC_ISER_ADMAERREN_Pos        (25U)
#define SDMMC_ISER_ADMAERREN_Msk        (0x1UL << SDMMC_ISER_ADMAERREN_Pos)
#define SDMMC_ISER_ADMAERREN            SDMMC_ISER_ADMAERREN_Msk
#define SDMMC_ISER_TUNEERREN_Pos        (26U)
#define SDMMC_ISER_TUNEERREN_Msk        (0x1UL << SDMMC_ISER_TUNEERREN_Pos)
#define SDMMC_ISER_TUNEERREN            SDMMC_ISER_TUNEERREN_Msk

/******************* Bit definition for SDMMC_IER register ********************/
#define SDMMC_IER_CCIE_Pos              (0U)
#define SDMMC_IER_CCIE_Msk              (0x1UL << SDMMC_IER_CCIE_Pos)
#define SDMMC_IER_CCIE                  SDMMC_IER_CCIE_Msk
#define SDMMC_IER_TCIE_Pos              (1U)
#define SDMMC_IER_TCIE_Msk              (0x1UL << SDMMC_IER_TCIE_Pos)
#define SDMMC_IER_TCIE                  SDMMC_IER_TCIE_Msk
#define SDMMC_IER_BLKGAPIE_Pos          (2U)
#define SDMMC_IER_BLKGAPIE_Msk          (0x1UL << SDMMC_IER_BLKGAPIE_Pos)
#define SDMMC_IER_BLKGAPIE              SDMMC_IER_BLKGAPIE_Msk
#define SDMMC_IER_DMAIE_Pos             (3U)
#define SDMMC_IER_DMAIE_Msk             (0x1UL << SDMMC_IER_DMAIE_Pos)
#define SDMMC_IER_DMAIE                 SDMMC_IER_DMAIE_Msk
#define SDMMC_IER_BUFWRDYIE_Pos         (4U)
#define SDMMC_IER_BUFWRDYIE_Msk         (0x1UL << SDMMC_IER_BUFWRDYIE_Pos)
#define SDMMC_IER_BUFWRDYIE             SDMMC_IER_BUFWRDYIE_Msk
#define SDMMC_IER_BUFRRDYIE_Pos         (5U)
#define SDMMC_IER_BUFRRDYIE_Msk         (0x1UL << SDMMC_IER_BUFRRDYIE_Pos)
#define SDMMC_IER_BUFRRDYIE             SDMMC_IER_BUFRRDYIE_Msk
#define SDMMC_IER_CINSERTIE_Pos         (6U)
#define SDMMC_IER_CINSERTIE_Msk         (0x1UL << SDMMC_IER_CINSERTIE_Pos)
#define SDMMC_IER_CINSERTIE             SDMMC_IER_CINSERTIE_Msk
#define SDMMC_IER_CREMOVIE_Pos          (7U)
#define SDMMC_IER_CREMOVIE_Msk          (0x1UL << SDMMC_IER_CREMOVIE_Pos)
#define SDMMC_IER_CREMOVIE              SDMMC_IER_CREMOVIE_Msk
#define SDMMC_IER_CARDIE_Pos            (8U)
#define SDMMC_IER_CARDIE_Msk            (0x1UL << SDMMC_IER_CARDIE_Pos)
#define SDMMC_IER_CARDIE                SDMMC_IER_CARDIE_Msk
#define SDMMC_IER_INTAIE_Pos            (9U)
#define SDMMC_IER_INTAIE_Msk            (0x1UL << SDMMC_IER_INTAIE_Pos)
#define SDMMC_IER_INTAIE                SDMMC_IER_INTAIE_Msk
#define SDMMC_IER_INTBIE_Pos            (10U)
#define SDMMC_IER_INTBIE_Msk            (0x1UL << SDMMC_IER_INTBIE_Pos)
#define SDMMC_IER_INTBIE                SDMMC_IER_INTBIE_Msk
#define SDMMC_IER_INTCIE_Pos            (11U)
#define SDMMC_IER_INTCIE_Msk            (0x1UL << SDMMC_IER_INTCIE_Pos)
#define SDMMC_IER_INTCIE                SDMMC_IER_INTCIE_Msk
#define SDMMC_IER_RETUNEIE_Pos          (12U)
#define SDMMC_IER_RETUNEIE_Msk          (0x1UL << SDMMC_IER_RETUNEIE_Pos)
#define SDMMC_IER_RETUNEIE              SDMMC_IER_RETUNEIE_Msk
#define SDMMC_IER_BOOTACKIE_Pos         (13U)
#define SDMMC_IER_BOOTACKIE_Msk         (0x1UL << SDMMC_IER_BOOTACKIE_Pos)
#define SDMMC_IER_BOOTACKIE             SDMMC_IER_BOOTACKIE_Msk
#define SDMMC_IER_BOOTDONEIE_Pos        (14U)
#define SDMMC_IER_BOOTDONEIE_Msk        (0x1UL << SDMMC_IER_BOOTDONEIE_Pos)
#define SDMMC_IER_BOOTDONEIE            SDMMC_IER_BOOTDONEIE_Msk
#define SDMMC_IER_CTOERRIE_Pos          (16U)
#define SDMMC_IER_CTOERRIE_Msk          (0x1UL << SDMMC_IER_CTOERRIE_Pos)
#define SDMMC_IER_CTOERRIE              SDMMC_IER_CTOERRIE_Msk
#define SDMMC_IER_CCRCERRIE_Pos         (17U)
#define SDMMC_IER_CCRCERRIE_Msk         (0x1UL << SDMMC_IER_CCRCERRIE_Pos)
#define SDMMC_IER_CCRCERRIE             SDMMC_IER_CCRCERRIE_Msk
#define SDMMC_IER_CEBERRIE_Pos          (18U)
#define SDMMC_IER_CEBERRIE_Msk          (0x1UL << SDMMC_IER_CEBERRIE_Pos)
#define SDMMC_IER_CEBERRIE              SDMMC_IER_CEBERRIE_Msk
#define SDMMC_IER_IDXERRIE_Pos          (19U)
#define SDMMC_IER_IDXERRIE_Msk          (0x1UL << SDMMC_IER_IDXERRIE_Pos)
#define SDMMC_IER_IDXERRIE              SDMMC_IER_IDXERRIE_Msk
#define SDMMC_IER_DTOERRIE_Pos          (20U)
#define SDMMC_IER_DTOERRIE_Msk          (0x1UL << SDMMC_IER_DTOERRIE_Pos)
#define SDMMC_IER_DTOERRIE              SDMMC_IER_DTOERRIE_Msk
#define SDMMC_IER_DCRCERRIE_Pos         (21U)
#define SDMMC_IER_DCRCERRIE_Msk         (0x1UL << SDMMC_IER_DCRCERRIE_Pos)
#define SDMMC_IER_DCRCERRIE             SDMMC_IER_DCRCERRIE_Msk
#define SDMMC_IER_DEBERRIE_Pos          (22U)
#define SDMMC_IER_DEBERRIE_Msk          (0x1UL << SDMMC_IER_DEBERRIE_Pos)
#define SDMMC_IER_DEBERRIE              SDMMC_IER_DEBERRIE_Msk
#define SDMMC_IER_CLERRIE_Pos           (23U)
#define SDMMC_IER_CLERRIE_Msk           (0x1UL << SDMMC_IER_CLERRIE_Pos)
#define SDMMC_IER_CLERRIE               SDMMC_IER_CLERRIE_Msk
#define SDMMC_IER_ACERRIE_Pos           (24U)
#define SDMMC_IER_ACERRIE_Msk           (0x1UL << SDMMC_IER_ACERRIE_Pos)
#define SDMMC_IER_ACERRIE               SDMMC_IER_ACERRIE_Msk
#define SDMMC_IER_ADMAERRIE_Pos         (25U)
#define SDMMC_IER_ADMAERRIE_Msk         (0x1UL << SDMMC_IER_ADMAERRIE_Pos)
#define SDMMC_IER_ADMAERRIE             SDMMC_IER_ADMAERRIE_Msk
#define SDMMC_IER_TUNEERRIE_Pos         (26U)
#define SDMMC_IER_TUNEERRIE_Msk         (0x1UL << SDMMC_IER_TUNEERRIE_Pos)
#define SDMMC_IER_TUNEERRIE             SDMMC_IER_TUNEERRIE_Msk

/******************* Bit definition for SDMMC_CR4 register ********************/
#define SDMMC_CR4_ACNE_Pos              (0U)
#define SDMMC_CR4_ACNE_Msk              (0x1UL << SDMMC_CR4_ACNE_Pos)
#define SDMMC_CR4_ACNE                  SDMMC_CR4_ACNE_Msk
#define SDMMC_CR4_ACTOERR_Pos           (1U)
#define SDMMC_CR4_ACTOERR_Msk           (0x1UL << SDMMC_CR4_ACTOERR_Pos)
#define SDMMC_CR4_ACTOERR               SDMMC_CR4_ACTOERR_Msk
#define SDMMC_CR4_ACCRCERR_Pos          (2U)
#define SDMMC_CR4_ACCRCERR_Msk          (0x1UL << SDMMC_CR4_ACCRCERR_Pos)
#define SDMMC_CR4_ACCRCERR              SDMMC_CR4_ACCRCERR_Msk
#define SDMMC_CR4_ACEBERR_Pos           (3U)
#define SDMMC_CR4_ACEBERR_Msk           (0x1UL << SDMMC_CR4_ACEBERR_Pos)
#define SDMMC_CR4_ACEBERR               SDMMC_CR4_ACEBERR_Msk
#define SDMMC_CR4_ACIDXERR_Pos          (4U)
#define SDMMC_CR4_ACIDXERR_Msk          (0x1UL << SDMMC_CR4_ACIDXERR_Pos)
#define SDMMC_CR4_ACIDXERR              SDMMC_CR4_ACIDXERR_Msk
#define SDMMC_CR4_CNIERR_Pos            (7U)
#define SDMMC_CR4_CNIERR_Msk            (0x1UL << SDMMC_CR4_CNIERR_Pos)
#define SDMMC_CR4_CNIERR                SDMMC_CR4_CNIERR_Msk
#define SDMMC_CR4_UHSMODE_Pos           (16U)
#define SDMMC_CR4_UHSMODE_Msk           (0x7UL << SDMMC_CR4_UHSMODE_Pos)
#define SDMMC_CR4_UHSMODE               SDMMC_CR4_UHSMODE_Msk
#define SDMMC_CR4_V1P8EN_Pos            (19U)
#define SDMMC_CR4_V1P8EN_Msk            (0x1UL << SDMMC_CR4_V1P8EN_Pos)
#define SDMMC_CR4_V1P8EN                SDMMC_CR4_V1P8EN_Msk
#define SDMMC_CR4_DRVSTR_Pos            (20U)
#define SDMMC_CR4_DRVSTR_Msk            (0x3UL << SDMMC_CR4_DRVSTR_Pos)
#define SDMMC_CR4_DRVSTR                SDMMC_CR4_DRVSTR_Msk
#define SDMMC_CR4_EXETUNE_Pos           (22U)
#define SDMMC_CR4_EXETUNE_Msk           (0x1UL << SDMMC_CR4_EXETUNE_Pos)
#define SDMMC_CR4_EXETUNE               SDMMC_CR4_EXETUNE_Msk
#define SDMMC_CR4_SAMPCLK_Pos           (23U)
#define SDMMC_CR4_SAMPCLK_Msk           (0x1UL << SDMMC_CR4_SAMPCLK_Pos)
#define SDMMC_CR4_SAMPCLK               SDMMC_CR4_SAMPCLK_Msk
#define SDMMC_CR4_CONSSAMP_Pos          (24U)
#define SDMMC_CR4_CONSSAMP_Msk          (0x3FUL << SDMMC_CR4_CONSSAMP_Pos)
#define SDMMC_CR4_CONSSAMP              SDMMC_CR4_CONSSAMP_Msk
#define SDMMC_CR4_ASYNCIE_Pos           (30U)
#define SDMMC_CR4_ASYNCIE_Msk           (0x1UL << SDMMC_CR4_ASYNCIE_Pos)
#define SDMMC_CR4_ASYNCIE               SDMMC_CR4_ASYNCIE_Msk
#define SDMMC_CR4_PVE_Pos               (31U)
#define SDMMC_CR4_PVE_Msk               (0x1UL << SDMMC_CR4_PVE_Pos)
#define SDMMC_CR4_PVE                   SDMMC_CR4_PVE_Msk

/******************* Bit definition for SDMMC_CAP1 register *******************/
#define SDMMC_CAP1_TOFREQ_Pos           (0U)
#define SDMMC_CAP1_TOFREQ_Msk           (0x3FUL << SDMMC_CAP1_TOFREQ_Pos)
#define SDMMC_CAP1_TOFREQ               SDMMC_CAP1_TOFREQ_Msk
#define SDMMC_CAP1_TOUNIT_Pos           (7U)
#define SDMMC_CAP1_TOUNIT_Msk           (0x1UL << SDMMC_CAP1_TOUNIT_Pos)
#define SDMMC_CAP1_TOUNIT               SDMMC_CAP1_TOUNIT_Msk
#define SDMMC_CAP1_BCLKFREQ_Pos         (8U)
#define SDMMC_CAP1_BCLKFREQ_Msk         (0xFFUL << SDMMC_CAP1_BCLKFREQ_Pos)
#define SDMMC_CAP1_BCLKFREQ             SDMMC_CAP1_BCLKFREQ_Msk
#define SDMMC_CAP1_MAXBLKLEN_Pos        (16U)
#define SDMMC_CAP1_MAXBLKLEN_Msk        (0x3UL << SDMMC_CAP1_MAXBLKLEN_Pos)
#define SDMMC_CAP1_MAXBLKLEN            SDMMC_CAP1_MAXBLKLEN_Msk
#define SDMMC_CAP1_BUS8BIT_Pos          (18U)
#define SDMMC_CAP1_BUS8BIT_Msk          (0x1UL << SDMMC_CAP1_BUS8BIT_Pos)
#define SDMMC_CAP1_BUS8BIT              SDMMC_CAP1_BUS8BIT_Msk
#define SDMMC_CAP1_ADMA_Pos             (19U)
#define SDMMC_CAP1_ADMA_Msk             (0x1UL << SDMMC_CAP1_ADMA_Pos)
#define SDMMC_CAP1_ADMA                 SDMMC_CAP1_ADMA_Msk
#define SDMMC_CAP1_HS_Pos               (21U)
#define SDMMC_CAP1_HS_Msk               (0x1UL << SDMMC_CAP1_HS_Pos)
#define SDMMC_CAP1_HS                   SDMMC_CAP1_HS_Msk
#define SDMMC_CAP1_SDMA_Pos             (22U)
#define SDMMC_CAP1_SDMA_Msk             (0x1UL << SDMMC_CAP1_SDMA_Pos)
#define SDMMC_CAP1_SDMA                 SDMMC_CAP1_SDMA_Msk
#define SDMMC_CAP1_SUSP_Pos             (23U)
#define SDMMC_CAP1_SUSP_Msk             (0x1UL << SDMMC_CAP1_SUSP_Pos)
#define SDMMC_CAP1_SUSP                 SDMMC_CAP1_SUSP_Msk
#define SDMMC_CAP1_V3P3_Pos             (24U)
#define SDMMC_CAP1_V3P3_Msk             (0x1UL << SDMMC_CAP1_V3P3_Pos)
#define SDMMC_CAP1_V3P3                 SDMMC_CAP1_V3P3_Msk
#define SDMMC_CAP1_V3P0_Pos             (25U)
#define SDMMC_CAP1_V3P0_Msk             (0x1UL << SDMMC_CAP1_V3P0_Pos)
#define SDMMC_CAP1_V3P0                 SDMMC_CAP1_V3P0_Msk
#define SDMMC_CAP1_V1P8_Pos             (26U)
#define SDMMC_CAP1_V1P8_Msk             (0x1UL << SDMMC_CAP1_V1P8_Pos)
#define SDMMC_CAP1_V1P8                 SDMMC_CAP1_V1P8_Msk
#define SDMMC_CAP1_V1P2_Pos             (27U)
#define SDMMC_CAP1_V1P2_Msk             (0x1UL << SDMMC_CAP1_V1P2_Pos)
#define SDMMC_CAP1_V1P2                 SDMMC_CAP1_V1P2_Msk
#define SDMMC_CAP1_ASYNCIRQ_Pos         (28U)
#define SDMMC_CAP1_ASYNCIRQ_Msk         (0x1UL << SDMMC_CAP1_ASYNCIRQ_Pos)
#define SDMMC_CAP1_ASYNCIRQ             SDMMC_CAP1_ASYNCIRQ_Msk
#define SDMMC_CAP1_SLOTTYPE_Pos         (30U)
#define SDMMC_CAP1_SLOTTYPE_Msk         (0x3UL << SDMMC_CAP1_SLOTTYPE_Pos)
#define SDMMC_CAP1_SLOTTYPE             SDMMC_CAP1_SLOTTYPE_Msk

/******************* Bit definition for SDMMC_CAP2 register *******************/
#define SDMMC_CAP2_SDR50_Pos            (0U)
#define SDMMC_CAP2_SDR50_Msk            (0x1UL << SDMMC_CAP2_SDR50_Pos)
#define SDMMC_CAP2_SDR50                SDMMC_CAP2_SDR50_Msk
#define SDMMC_CAP2_SDR104_Pos           (1U)
#define SDMMC_CAP2_SDR104_Msk           (0x1UL << SDMMC_CAP2_SDR104_Pos)
#define SDMMC_CAP2_SDR104               SDMMC_CAP2_SDR104_Msk
#define SDMMC_CAP2_DDR550_Pos           (2U)
#define SDMMC_CAP2_DDR550_Msk           (0x1UL << SDMMC_CAP2_DDR550_Pos)
#define SDMMC_CAP2_DDR550               SDMMC_CAP2_DDR550_Msk
#define SDMMC_CAP2_DRVTYPEA_Pos         (4U)
#define SDMMC_CAP2_DRVTYPEA_Msk         (0x1UL << SDMMC_CAP2_DRVTYPEA_Pos)
#define SDMMC_CAP2_DRVTYPEA             SDMMC_CAP2_DRVTYPEA_Msk
#define SDMMC_CAP2_DRVTYPEC_Pos         (5U)
#define SDMMC_CAP2_DRVTYPEC_Msk         (0x1UL << SDMMC_CAP2_DRVTYPEC_Pos)
#define SDMMC_CAP2_DRVTYPEC             SDMMC_CAP2_DRVTYPEC_Msk
#define SDMMC_CAP2_DRVTYPED_Pos         (6U)
#define SDMMC_CAP2_DRVTYPED_Msk         (0x1UL << SDMMC_CAP2_DRVTYPED_Pos)
#define SDMMC_CAP2_DRVTYPED             SDMMC_CAP2_DRVTYPED_Msk
#define SDMMC_CAP2_RETUNECNT_Pos        (8U)
#define SDMMC_CAP2_RETUNECNT_Msk        (0xFUL << SDMMC_CAP2_RETUNECNT_Pos)
#define SDMMC_CAP2_RETUNECNT            SDMMC_CAP2_RETUNECNT_Msk
#define SDMMC_CAP2_SDR50TUNE_Pos        (13U)
#define SDMMC_CAP2_SDR50TUNE_Msk        (0x1UL << SDMMC_CAP2_SDR50TUNE_Pos)
#define SDMMC_CAP2_SDR50TUNE            SDMMC_CAP2_SDR50TUNE_Msk
#define SDMMC_CAP2_RETUNEMODE_Pos       (14U)
#define SDMMC_CAP2_RETUNEMODE_Msk       (0x3UL << SDMMC_CAP2_RETUNEMODE_Pos)
#define SDMMC_CAP2_RETUNEMODE           SDMMC_CAP2_RETUNEMODE_Msk
#define SDMMC_CAP2_CLKMULT_Pos          (16U)
#define SDMMC_CAP2_CLKMULT_Msk          (0xFFUL << SDMMC_CAP2_CLKMULT_Pos)
#define SDMMC_CAP2_CLKMULT              SDMMC_CAP2_CLKMULT_Msk

/******************* Bit definition for SDMMC_CAP3 register *******************/
#define SDMMC_CAP3_MAX3P3_Pos           (0U)
#define SDMMC_CAP3_MAX3P3_Msk           (0xFFUL << SDMMC_CAP3_MAX3P3_Pos)
#define SDMMC_CAP3_MAX3P3               SDMMC_CAP3_MAX3P3_Msk
#define SDMMC_CAP3_MAX3P0_Pos           (8U)
#define SDMMC_CAP3_MAX3P0_Msk           (0xFFUL << SDMMC_CAP3_MAX3P0_Pos)
#define SDMMC_CAP3_MAX3P0               SDMMC_CAP3_MAX3P0_Msk
#define SDMMC_CAP3_MAX1P8_Pos           (16U)
#define SDMMC_CAP3_MAX1P8_Msk           (0xFFUL << SDMMC_CAP3_MAX1P8_Pos)
#define SDMMC_CAP3_MAX1P8               SDMMC_CAP3_MAX1P8_Msk

/******************* Bit definition for SDMMC_FER register ********************/
#define SDMMC_FER_ACNE_Pos              (0U)
#define SDMMC_FER_ACNE_Msk              (0x1UL << SDMMC_FER_ACNE_Pos)
#define SDMMC_FER_ACNE                  SDMMC_FER_ACNE_Msk
#define SDMMC_FER_ACTOERR_Pos           (1U)
#define SDMMC_FER_ACTOERR_Msk           (0x1UL << SDMMC_FER_ACTOERR_Pos)
#define SDMMC_FER_ACTOERR               SDMMC_FER_ACTOERR_Msk
#define SDMMC_FER_ACCRCERR_Pos          (2U)
#define SDMMC_FER_ACCRCERR_Msk          (0x1UL << SDMMC_FER_ACCRCERR_Pos)
#define SDMMC_FER_ACCRCERR              SDMMC_FER_ACCRCERR_Msk
#define SDMMC_FER_ACEBERR_Pos           (3U)
#define SDMMC_FER_ACEBERR_Msk           (0x1UL << SDMMC_FER_ACEBERR_Pos)
#define SDMMC_FER_ACEBERR               SDMMC_FER_ACEBERR_Msk
#define SDMMC_FER_ACIDXERR_Pos          (4U)
#define SDMMC_FER_ACIDXERR_Msk          (0x1UL << SDMMC_FER_ACIDXERR_Pos)
#define SDMMC_FER_ACIDXERR              SDMMC_FER_ACIDXERR_Msk
#define SDMMC_FER_CNIERR_Pos            (7U)
#define SDMMC_FER_CNIERR_Msk            (0x1UL << SDMMC_FER_CNIERR_Pos)
#define SDMMC_FER_CNIERR                SDMMC_FER_CNIERR_Msk
#define SDMMC_FER_CTOERR_Pos            (16U)
#define SDMMC_FER_CTOERR_Msk            (0x1UL << SDMMC_FER_CTOERR_Pos)
#define SDMMC_FER_CTOERR                SDMMC_FER_CTOERR_Msk
#define SDMMC_FER_CCRCERR_Pos           (17U)
#define SDMMC_FER_CCRCERR_Msk           (0x1UL << SDMMC_FER_CCRCERR_Pos)
#define SDMMC_FER_CCRCERR               SDMMC_FER_CCRCERR_Msk
#define SDMMC_FER_CEBERR_Pos            (18U)
#define SDMMC_FER_CEBERR_Msk            (0x1UL << SDMMC_FER_CEBERR_Pos)
#define SDMMC_FER_CEBERR                SDMMC_FER_CEBERR_Msk
#define SDMMC_FER_IDXERR_Pos            (19U)
#define SDMMC_FER_IDXERR_Msk            (0x1UL << SDMMC_FER_IDXERR_Pos)
#define SDMMC_FER_IDXERR                SDMMC_FER_IDXERR_Msk
#define SDMMC_FER_DTOERR_Pos            (20U)
#define SDMMC_FER_DTOERR_Msk            (0x1UL << SDMMC_FER_DTOERR_Pos)
#define SDMMC_FER_DTOERR                SDMMC_FER_DTOERR_Msk
#define SDMMC_FER_DCRCERR_Pos           (21U)
#define SDMMC_FER_DCRCERR_Msk           (0x1UL << SDMMC_FER_DCRCERR_Pos)
#define SDMMC_FER_DCRCERR               SDMMC_FER_DCRCERR_Msk
#define SDMMC_FER_DEBERR_Pos            (22U)
#define SDMMC_FER_DEBERR_Msk            (0x1UL << SDMMC_FER_DEBERR_Pos)
#define SDMMC_FER_DEBERR                SDMMC_FER_DEBERR_Msk
#define SDMMC_FER_CLERR_Pos             (23U)
#define SDMMC_FER_CLERR_Msk             (0x1UL << SDMMC_FER_CLERR_Pos)
#define SDMMC_FER_CLERR                 SDMMC_FER_CLERR_Msk
#define SDMMC_FER_ACERR_Pos             (24U)
#define SDMMC_FER_ACERR_Msk             (0x1UL << SDMMC_FER_ACERR_Pos)
#define SDMMC_FER_ACERR                 SDMMC_FER_ACERR_Msk
#define SDMMC_FER_ADMAERR_Pos           (25U)
#define SDMMC_FER_ADMAERR_Msk           (0x1UL << SDMMC_FER_ADMAERR_Pos)
#define SDMMC_FER_ADMAERR               SDMMC_FER_ADMAERR_Msk

/******************* Bit definition for SDMMC_AESR register *******************/
#define SDMMC_AESR_ERRSTATE_Pos         (0U)
#define SDMMC_AESR_ERRSTATE_Msk         (0x3UL << SDMMC_AESR_ERRSTATE_Pos)
#define SDMMC_AESR_ERRSTATE             SDMMC_AESR_ERRSTATE_Msk
#define SDMMC_AESR_LENERR_Pos           (2U)
#define SDMMC_AESR_LENERR_Msk           (0x1UL << SDMMC_AESR_LENERR_Pos)
#define SDMMC_AESR_LENERR               SDMMC_AESR_LENERR_Msk

/******************* Bit definition for SDMMC_ASAR register *******************/
#define SDMMC_ASAR_ADDR_Pos             (0U)
#define SDMMC_ASAR_ADDR_Msk             (0xFFFFFFFFUL << SDMMC_ASAR_ADDR_Pos)
#define SDMMC_ASAR_ADDR                 SDMMC_ASAR_ADDR_Msk

/******************* Bit definition for SDMMC_PVR1 register *******************/
#define SDMMC_PVR1_INITCLKDIV_Pos       (0U)
#define SDMMC_PVR1_INITCLKDIV_Msk       (0x3FFUL << SDMMC_PVR1_INITCLKDIV_Pos)
#define SDMMC_PVR1_INITCLKDIV           SDMMC_PVR1_INITCLKDIV_Msk
#define SDMMC_PVR1_INITCLKSEL_Pos       (10U)
#define SDMMC_PVR1_INITCLKSEL_Msk       (0x1UL << SDMMC_PVR1_INITCLKSEL_Pos)
#define SDMMC_PVR1_INITCLKSEL           SDMMC_PVR1_INITCLKSEL_Msk
#define SDMMC_PVR1_INITDRVSTR_Pos       (14U)
#define SDMMC_PVR1_INITDRVSTR_Msk       (0x3UL << SDMMC_PVR1_INITDRVSTR_Pos)
#define SDMMC_PVR1_INITDRVSTR           SDMMC_PVR1_INITDRVSTR_Msk
#define SDMMC_PVR1_DSCLKDIV_Pos         (16U)
#define SDMMC_PVR1_DSCLKDIV_Msk         (0x3FFUL << SDMMC_PVR1_DSCLKDIV_Pos)
#define SDMMC_PVR1_DSCLKDIV             SDMMC_PVR1_DSCLKDIV_Msk
#define SDMMC_PVR1_DSCLKSEL_Pos         (26U)
#define SDMMC_PVR1_DSCLKSEL_Msk         (0x1UL << SDMMC_PVR1_DSCLKSEL_Pos)
#define SDMMC_PVR1_DSCLKSEL             SDMMC_PVR1_DSCLKSEL_Msk
#define SDMMC_PVR1_DSDRVSTR_Pos         (30U)
#define SDMMC_PVR1_DSDRVSTR_Msk         (0x3UL << SDMMC_PVR1_DSDRVSTR_Pos)
#define SDMMC_PVR1_DSDRVSTR             SDMMC_PVR1_DSDRVSTR_Msk

/******************* Bit definition for SDMMC_PVR2 register *******************/
#define SDMMC_PVR2_HSCLKDIV_Pos         (0U)
#define SDMMC_PVR2_HSCLKDIV_Msk         (0x3FFUL << SDMMC_PVR2_HSCLKDIV_Pos)
#define SDMMC_PVR2_HSCLKDIV             SDMMC_PVR2_HSCLKDIV_Msk
#define SDMMC_PVR2_HSCLKSEL_Pos         (10U)
#define SDMMC_PVR2_HSCLKSEL_Msk         (0x1UL << SDMMC_PVR2_HSCLKSEL_Pos)
#define SDMMC_PVR2_HSCLKSEL             SDMMC_PVR2_HSCLKSEL_Msk
#define SDMMC_PVR2_HSDRVSTR_Pos         (14U)
#define SDMMC_PVR2_HSDRVSTR_Msk         (0x3UL << SDMMC_PVR2_HSDRVSTR_Pos)
#define SDMMC_PVR2_HSDRVSTR             SDMMC_PVR2_HSDRVSTR_Msk
#define SDMMC_PVR2_SDR12CLKDIV_Pos      (16U)
#define SDMMC_PVR2_SDR12CLKDIV_Msk      (0x3FFUL << SDMMC_PVR2_SDR12CLKDIV_Pos)
#define SDMMC_PVR2_SDR12CLKDIV          SDMMC_PVR2_SDR12CLKDIV_Msk
#define SDMMC_PVR2_SDR12CLKSEL_Pos      (26U)
#define SDMMC_PVR2_SDR12CLKSEL_Msk      (0x1UL << SDMMC_PVR2_SDR12CLKSEL_Pos)
#define SDMMC_PVR2_SDR12CLKSEL          SDMMC_PVR2_SDR12CLKSEL_Msk
#define SDMMC_PVR2_SDR12DRVSTR_Pos      (30U)
#define SDMMC_PVR2_SDR12DRVSTR_Msk      (0x3UL << SDMMC_PVR2_SDR12DRVSTR_Pos)
#define SDMMC_PVR2_SDR12DRVSTR          SDMMC_PVR2_SDR12DRVSTR_Msk

/******************* Bit definition for SDMMC_PVR3 register *******************/
#define SDMMC_PVR3_SDR25CLKDIV_Pos      (0U)
#define SDMMC_PVR3_SDR25CLKDIV_Msk      (0x3FFUL << SDMMC_PVR3_SDR25CLKDIV_Pos)
#define SDMMC_PVR3_SDR25CLKDIV          SDMMC_PVR3_SDR25CLKDIV_Msk
#define SDMMC_PVR3_SDR25CLKSEL_Pos      (10U)
#define SDMMC_PVR3_SDR25CLKSEL_Msk      (0x1UL << SDMMC_PVR3_SDR25CLKSEL_Pos)
#define SDMMC_PVR3_SDR25CLKSEL          SDMMC_PVR3_SDR25CLKSEL_Msk
#define SDMMC_PVR3_SDR25DRVSTR_Pos      (14U)
#define SDMMC_PVR3_SDR25DRVSTR_Msk      (0x3UL << SDMMC_PVR3_SDR25DRVSTR_Pos)
#define SDMMC_PVR3_SDR25DRVSTR          SDMMC_PVR3_SDR25DRVSTR_Msk
#define SDMMC_PVR3_SDR50CLKDIV_Pos      (16U)
#define SDMMC_PVR3_SDR50CLKDIV_Msk      (0x3FFUL << SDMMC_PVR3_SDR50CLKDIV_Pos)
#define SDMMC_PVR3_SDR50CLKDIV          SDMMC_PVR3_SDR50CLKDIV_Msk
#define SDMMC_PVR3_SDR50CLKSEL_Pos      (26U)
#define SDMMC_PVR3_SDR50CLKSEL_Msk      (0x1UL << SDMMC_PVR3_SDR50CLKSEL_Pos)
#define SDMMC_PVR3_SDR50CLKSEL          SDMMC_PVR3_SDR50CLKSEL_Msk
#define SDMMC_PVR3_SDR50DRVSTR_Pos      (30U)
#define SDMMC_PVR3_SDR50DRVSTR_Msk      (0x3UL << SDMMC_PVR3_SDR50DRVSTR_Pos)
#define SDMMC_PVR3_SDR50DRVSTR          SDMMC_PVR3_SDR50DRVSTR_Msk

/******************* Bit definition for SDMMC_PVR4 register *******************/
#define SDMMC_PVR4_SDR104CLKDIV_Pos     (0U)
#define SDMMC_PVR4_SDR104CLKDIV_Msk     (0x3FFUL << SDMMC_PVR4_SDR104CLKDIV_Pos)
#define SDMMC_PVR4_SDR104CLKDIV         SDMMC_PVR4_SDR104CLKDIV_Msk
#define SDMMC_PVR4_SDR104CLKSEL_Pos     (10U)
#define SDMMC_PVR4_SDR104CLKSEL_Msk     (0x1UL << SDMMC_PVR4_SDR104CLKSEL_Pos)
#define SDMMC_PVR4_SDR104CLKSEL         SDMMC_PVR4_SDR104CLKSEL_Msk
#define SDMMC_PVR4_SDR104DRVSTR_Pos     (14U)
#define SDMMC_PVR4_SDR104DRVSTR_Msk     (0x3UL << SDMMC_PVR4_SDR104DRVSTR_Pos)
#define SDMMC_PVR4_SDR104DRVSTR         SDMMC_PVR4_SDR104DRVSTR_Msk
#define SDMMC_PVR4_DDR50CLKDIV_Pos      (16U)
#define SDMMC_PVR4_DDR50CLKDIV_Msk      (0x3FFUL << SDMMC_PVR4_DDR50CLKDIV_Pos)
#define SDMMC_PVR4_DDR50CLKDIV          SDMMC_PVR4_DDR50CLKDIV_Msk
#define SDMMC_PVR4_DDR50CLKSEL_Pos      (26U)
#define SDMMC_PVR4_DDR50CLKSEL_Msk      (0x1UL << SDMMC_PVR4_DDR50CLKSEL_Pos)
#define SDMMC_PVR4_DDR50CLKSEL          SDMMC_PVR4_DDR50CLKSEL_Msk
#define SDMMC_PVR4_DDR50DRVSTR_Pos      (30U)
#define SDMMC_PVR4_DDR50DRVSTR_Msk      (0x3UL << SDMMC_PVR4_DDR50DRVSTR_Pos)
#define SDMMC_PVR4_DDR50DRVSTR          SDMMC_PVR4_DDR50DRVSTR_Msk

/******************* Bit definition for SDMMC_ABSR register *******************/
#define SDMMC_ABSR_BSIZE_Pos            (0U)
#define SDMMC_ABSR_BSIZE_Msk            (0x7FUL << SDMMC_ABSR_BSIZE_Pos)
#define SDMMC_ABSR_BSIZE                SDMMC_ABSR_BSIZE_Msk

/******************* Bit definition for SDMMC_VER register ********************/
#define SDMMC_VER_SLOT_Pos              (0U)
#define SDMMC_VER_SLOT_Msk              (0xFFUL << SDMMC_VER_SLOT_Pos)
#define SDMMC_VER_SLOT                  SDMMC_VER_SLOT_Msk
#define SDMMC_VER_SVER_Pos              (16U)
#define SDMMC_VER_SVER_Msk              (0xFFUL << SDMMC_VER_SVER_Pos)
#define SDMMC_VER_SVER                  SDMMC_VER_SVER_Msk
#define SDMMC_VER_VER_Pos               (24U)
#define SDMMC_VER_VER_Msk               (0xFFUL << SDMMC_VER_VER_Pos)
#define SDMMC_VER_VER                   SDMMC_VER_VER_Msk

#endif
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
