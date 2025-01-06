/**
  ******************************************************************************
  * @file   spi.h
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

#ifndef __SPI_H
#define __SPI_H

typedef struct
{
    __IO uint32_t TOP_CTRL;
    __IO uint32_t FIFO_CTRL;
    __IO uint32_t INTE;
    __IO uint32_t TO;
    __IO uint32_t DATA;
    __IO uint32_t STATUS;
    __IO uint32_t PSP_CTRL;
    __IO uint32_t NW_CTRL;
    __IO uint32_t NW_STATUS;
    __IO uint32_t RWOT_CTRL;
    __IO uint32_t RWOT_CCM;
    __IO uint32_t RWOT_CVWRN;
    __IO uint32_t RSVD2[3];
    __IO uint32_t CLK_CTRL;
    __IO uint32_t RSVD1[5];
    __IO uint32_t TRIWIRE_CTRL;
} SPI_TypeDef;


/****************** Bit definition for SPI_TOP_CTRL register ******************/
#define SPI_TOP_CTRL_SSE_Pos            (0U)
#define SPI_TOP_CTRL_SSE_Msk            (0x1UL << SPI_TOP_CTRL_SSE_Pos)
#define SPI_TOP_CTRL_SSE                SPI_TOP_CTRL_SSE_Msk
#define SPI_TOP_CTRL_FRF_Pos            (1U)
#define SPI_TOP_CTRL_FRF_Msk            (0x3UL << SPI_TOP_CTRL_FRF_Pos)
#define SPI_TOP_CTRL_FRF                SPI_TOP_CTRL_FRF_Msk
#define SPI_TOP_CTRL_SCLKDIR_Pos        (3U)
#define SPI_TOP_CTRL_SCLKDIR_Msk        (0x1UL << SPI_TOP_CTRL_SCLKDIR_Pos)
#define SPI_TOP_CTRL_SCLKDIR            SPI_TOP_CTRL_SCLKDIR_Msk
#define SPI_TOP_CTRL_SFRMDIR_Pos        (4U)
#define SPI_TOP_CTRL_SFRMDIR_Msk        (0x1UL << SPI_TOP_CTRL_SFRMDIR_Pos)
#define SPI_TOP_CTRL_SFRMDIR            SPI_TOP_CTRL_SFRMDIR_Msk
#define SPI_TOP_CTRL_DSS_Pos            (5U)
#define SPI_TOP_CTRL_DSS_Msk            (0x1FUL << SPI_TOP_CTRL_DSS_Pos)
#define SPI_TOP_CTRL_DSS                SPI_TOP_CTRL_DSS_Msk
#define SPI_TOP_CTRL_SPO_Pos            (10U)
#define SPI_TOP_CTRL_SPO_Msk            (0x1UL << SPI_TOP_CTRL_SPO_Pos)
#define SPI_TOP_CTRL_SPO                SPI_TOP_CTRL_SPO_Msk
#define SPI_TOP_CTRL_SPH_Pos            (11U)
#define SPI_TOP_CTRL_SPH_Msk            (0x1UL << SPI_TOP_CTRL_SPH_Pos)
#define SPI_TOP_CTRL_SPH                SPI_TOP_CTRL_SPH_Msk
#define SPI_TOP_CTRL_LBM_Pos            (12U)
#define SPI_TOP_CTRL_LBM_Msk            (0x1UL << SPI_TOP_CTRL_LBM_Pos)
#define SPI_TOP_CTRL_LBM                SPI_TOP_CTRL_LBM_Msk
#define SPI_TOP_CTRL_TRAIL_Pos          (13U)
#define SPI_TOP_CTRL_TRAIL_Msk          (0x1UL << SPI_TOP_CTRL_TRAIL_Pos)
#define SPI_TOP_CTRL_TRAIL              SPI_TOP_CTRL_TRAIL_Msk
#define SPI_TOP_CTRL_HOLD_FRAME_LOW_Pos  (14U)
#define SPI_TOP_CTRL_HOLD_FRAME_LOW_Msk  (0x1UL << SPI_TOP_CTRL_HOLD_FRAME_LOW_Pos)
#define SPI_TOP_CTRL_HOLD_FRAME_LOW     SPI_TOP_CTRL_HOLD_FRAME_LOW_Msk
#define SPI_TOP_CTRL_IFS_Pos            (15U)
#define SPI_TOP_CTRL_IFS_Msk            (0x1UL << SPI_TOP_CTRL_IFS_Pos)
#define SPI_TOP_CTRL_IFS                SPI_TOP_CTRL_IFS_Msk
#define SPI_TOP_CTRL_SCFR_Pos           (16U)
#define SPI_TOP_CTRL_SCFR_Msk           (0x1UL << SPI_TOP_CTRL_SCFR_Pos)
#define SPI_TOP_CTRL_SCFR               SPI_TOP_CTRL_SCFR_Msk
#define SPI_TOP_CTRL_TTE_Pos            (17U)
#define SPI_TOP_CTRL_TTE_Msk            (0x1UL << SPI_TOP_CTRL_TTE_Pos)
#define SPI_TOP_CTRL_TTE                SPI_TOP_CTRL_TTE_Msk
#define SPI_TOP_CTRL_TTELP_Pos          (18U)
#define SPI_TOP_CTRL_TTELP_Msk          (0x1UL << SPI_TOP_CTRL_TTELP_Pos)
#define SPI_TOP_CTRL_TTELP              SPI_TOP_CTRL_TTELP_Msk

/***************** Bit definition for SPI_FIFO_CTRL register ******************/
#define SPI_FIFO_CTRL_TFT_Pos           (0U)
#define SPI_FIFO_CTRL_TFT_Msk           (0x1FUL << SPI_FIFO_CTRL_TFT_Pos)
#define SPI_FIFO_CTRL_TFT               SPI_FIFO_CTRL_TFT_Msk
#define SPI_FIFO_CTRL_RFT_Pos           (5U)
#define SPI_FIFO_CTRL_RFT_Msk           (0x1FUL << SPI_FIFO_CTRL_RFT_Pos)
#define SPI_FIFO_CTRL_RFT               SPI_FIFO_CTRL_RFT_Msk
#define SPI_FIFO_CTRL_TSRE_Pos          (10U)
#define SPI_FIFO_CTRL_TSRE_Msk          (0x1UL << SPI_FIFO_CTRL_TSRE_Pos)
#define SPI_FIFO_CTRL_TSRE              SPI_FIFO_CTRL_TSRE_Msk
#define SPI_FIFO_CTRL_RSRE_Pos          (11U)
#define SPI_FIFO_CTRL_RSRE_Msk          (0x1UL << SPI_FIFO_CTRL_RSRE_Pos)
#define SPI_FIFO_CTRL_RSRE              SPI_FIFO_CTRL_RSRE_Msk
#define SPI_FIFO_CTRL_RXFIFO_RD_ENDIAN_Pos  (12U)
#define SPI_FIFO_CTRL_RXFIFO_RD_ENDIAN_Msk  (0x3UL << SPI_FIFO_CTRL_RXFIFO_RD_ENDIAN_Pos)
#define SPI_FIFO_CTRL_RXFIFO_RD_ENDIAN  SPI_FIFO_CTRL_RXFIFO_RD_ENDIAN_Msk
#define SPI_FIFO_CTRL_TXFIFO_WR_ENDIAN_Pos  (14U)
#define SPI_FIFO_CTRL_TXFIFO_WR_ENDIAN_Msk  (0x3UL << SPI_FIFO_CTRL_TXFIFO_WR_ENDIAN_Pos)
#define SPI_FIFO_CTRL_TXFIFO_WR_ENDIAN  SPI_FIFO_CTRL_TXFIFO_WR_ENDIAN_Msk
#define SPI_FIFO_CTRL_FPCKE_Pos         (16U)
#define SPI_FIFO_CTRL_FPCKE_Msk         (0x1UL << SPI_FIFO_CTRL_FPCKE_Pos)
#define SPI_FIFO_CTRL_FPCKE             SPI_FIFO_CTRL_FPCKE_Msk
#define SPI_FIFO_CTRL_RXFIFO_AUTO_FULL_CTRL_Pos  (17U)
#define SPI_FIFO_CTRL_RXFIFO_AUTO_FULL_CTRL_Msk  (0x1UL << SPI_FIFO_CTRL_RXFIFO_AUTO_FULL_CTRL_Pos)
#define SPI_FIFO_CTRL_RXFIFO_AUTO_FULL_CTRL  SPI_FIFO_CTRL_RXFIFO_AUTO_FULL_CTRL_Msk
#define SPI_FIFO_CTRL_EFWR_Pos          (18U)
#define SPI_FIFO_CTRL_EFWR_Msk          (0x1UL << SPI_FIFO_CTRL_EFWR_Pos)
#define SPI_FIFO_CTRL_EFWR              SPI_FIFO_CTRL_EFWR_Msk
#define SPI_FIFO_CTRL_STRF_Pos          (19U)
#define SPI_FIFO_CTRL_STRF_Msk          (0x1UL << SPI_FIFO_CTRL_STRF_Pos)
#define SPI_FIFO_CTRL_STRF              SPI_FIFO_CTRL_STRF_Msk

/******************** Bit definition for SPI_INTE register ********************/
#define SPI_INTE_PINTE_Pos              (0U)
#define SPI_INTE_PINTE_Msk              (0x1UL << SPI_INTE_PINTE_Pos)
#define SPI_INTE_PINTE                  SPI_INTE_PINTE_Msk
#define SPI_INTE_TINTE_Pos              (1U)
#define SPI_INTE_TINTE_Msk              (0x1UL << SPI_INTE_TINTE_Pos)
#define SPI_INTE_TINTE                  SPI_INTE_TINTE_Msk
#define SPI_INTE_RIE_Pos                (2U)
#define SPI_INTE_RIE_Msk                (0x1UL << SPI_INTE_RIE_Pos)
#define SPI_INTE_RIE                    SPI_INTE_RIE_Msk
#define SPI_INTE_TIE_Pos                (3U)
#define SPI_INTE_TIE_Msk                (0x1UL << SPI_INTE_TIE_Pos)
#define SPI_INTE_TIE                    SPI_INTE_TIE_Msk
#define SPI_INTE_RIM_Pos                (4U)
#define SPI_INTE_RIM_Msk                (0x1UL << SPI_INTE_RIM_Pos)
#define SPI_INTE_RIM                    SPI_INTE_RIM_Msk
#define SPI_INTE_TIM_Pos                (5U)
#define SPI_INTE_TIM_Msk                (0x1UL << SPI_INTE_TIM_Pos)
#define SPI_INTE_TIM                    SPI_INTE_TIM_Msk
#define SPI_INTE_EBCEI_Pos              (6U)
#define SPI_INTE_EBCEI_Msk              (0x1UL << SPI_INTE_EBCEI_Pos)
#define SPI_INTE_EBCEI                  SPI_INTE_EBCEI_Msk

/********************* Bit definition for SPI_TO register *********************/
#define SPI_TO_TIMEOUT_Pos              (0U)
#define SPI_TO_TIMEOUT_Msk              (0xFFFFFFUL << SPI_TO_TIMEOUT_Pos)
#define SPI_TO_TIMEOUT                  SPI_TO_TIMEOUT_Msk

/******************** Bit definition for SPI_DATA register ********************/
#define SPI_DATA_DATA_Pos               (0U)
#define SPI_DATA_DATA_Msk               (0xFFFFFFFFUL << SPI_DATA_DATA_Pos)
#define SPI_DATA_DATA                   SPI_DATA_DATA_Msk

/******************* Bit definition for SPI_STATUS register *******************/
#define SPI_STATUS_BSY_Pos              (0U)
#define SPI_STATUS_BSY_Msk              (0x1UL << SPI_STATUS_BSY_Pos)
#define SPI_STATUS_BSY                  SPI_STATUS_BSY_Msk
#define SPI_STATUS_CSS_Pos              (1U)
#define SPI_STATUS_CSS_Msk              (0x1UL << SPI_STATUS_CSS_Pos)
#define SPI_STATUS_CSS                  SPI_STATUS_CSS_Msk
#define SPI_STATUS_PINT_Pos             (2U)
#define SPI_STATUS_PINT_Msk             (0x1UL << SPI_STATUS_PINT_Pos)
#define SPI_STATUS_PINT                 SPI_STATUS_PINT_Msk
#define SPI_STATUS_TINT_Pos             (3U)
#define SPI_STATUS_TINT_Msk             (0x1UL << SPI_STATUS_TINT_Pos)
#define SPI_STATUS_TINT                 SPI_STATUS_TINT_Msk
#define SPI_STATUS_EOC_Pos              (4U)
#define SPI_STATUS_EOC_Msk              (0x1UL << SPI_STATUS_EOC_Pos)
#define SPI_STATUS_EOC                  SPI_STATUS_EOC_Msk
#define SPI_STATUS_TFS_Pos              (5U)
#define SPI_STATUS_TFS_Msk              (0x1UL << SPI_STATUS_TFS_Pos)
#define SPI_STATUS_TFS                  SPI_STATUS_TFS_Msk
#define SPI_STATUS_TNF_Pos              (6U)
#define SPI_STATUS_TNF_Msk              (0x1UL << SPI_STATUS_TNF_Pos)
#define SPI_STATUS_TNF                  SPI_STATUS_TNF_Msk
#define SPI_STATUS_TFL_Pos              (7U)
#define SPI_STATUS_TFL_Msk              (0x1FUL << SPI_STATUS_TFL_Pos)
#define SPI_STATUS_TFL                  SPI_STATUS_TFL_Msk
#define SPI_STATUS_TUR_Pos              (12U)
#define SPI_STATUS_TUR_Msk              (0x1UL << SPI_STATUS_TUR_Pos)
#define SPI_STATUS_TUR                  SPI_STATUS_TUR_Msk
#define SPI_STATUS_RFS_Pos              (13U)
#define SPI_STATUS_RFS_Msk              (0x1UL << SPI_STATUS_RFS_Pos)
#define SPI_STATUS_RFS                  SPI_STATUS_RFS_Msk
#define SPI_STATUS_RNE_Pos              (14U)
#define SPI_STATUS_RNE_Msk              (0x1UL << SPI_STATUS_RNE_Pos)
#define SPI_STATUS_RNE                  SPI_STATUS_RNE_Msk
#define SPI_STATUS_RFL_Pos              (15U)
#define SPI_STATUS_RFL_Msk              (0x1FUL << SPI_STATUS_RFL_Pos)
#define SPI_STATUS_RFL                  SPI_STATUS_RFL_Msk
#define SPI_STATUS_ROR_Pos              (20U)
#define SPI_STATUS_ROR_Msk              (0x1UL << SPI_STATUS_ROR_Pos)
#define SPI_STATUS_ROR                  SPI_STATUS_ROR_Msk
#define SPI_STATUS_BCE_Pos              (21U)
#define SPI_STATUS_BCE_Msk              (0x1UL << SPI_STATUS_BCE_Pos)
#define SPI_STATUS_BCE                  SPI_STATUS_BCE_Msk
#define SPI_STATUS_TX_OSS_Pos           (22U)
#define SPI_STATUS_TX_OSS_Msk           (0x1UL << SPI_STATUS_TX_OSS_Pos)
#define SPI_STATUS_TX_OSS               SPI_STATUS_TX_OSS_Msk
#define SPI_STATUS_OSS_Pos              (23U)
#define SPI_STATUS_OSS_Msk              (0x1UL << SPI_STATUS_OSS_Pos)
#define SPI_STATUS_OSS                  SPI_STATUS_OSS_Msk

/****************** Bit definition for SPI_PSP_CTRL register ******************/
#define SPI_PSP_CTRL_SCMODE_Pos         (0U)
#define SPI_PSP_CTRL_SCMODE_Msk         (0x3UL << SPI_PSP_CTRL_SCMODE_Pos)
#define SPI_PSP_CTRL_SCMODE             SPI_PSP_CTRL_SCMODE_Msk
#define SPI_PSP_CTRL_ETDS_Pos           (2U)
#define SPI_PSP_CTRL_ETDS_Msk           (0x1UL << SPI_PSP_CTRL_ETDS_Pos)
#define SPI_PSP_CTRL_ETDS               SPI_PSP_CTRL_ETDS_Msk
#define SPI_PSP_CTRL_FSRT_Pos           (3U)
#define SPI_PSP_CTRL_FSRT_Msk           (0x1UL << SPI_PSP_CTRL_FSRT_Pos)
#define SPI_PSP_CTRL_FSRT               SPI_PSP_CTRL_FSRT_Msk
#define SPI_PSP_CTRL_SFRMP_Pos          (4U)
#define SPI_PSP_CTRL_SFRMP_Msk          (0x1UL << SPI_PSP_CTRL_SFRMP_Pos)
#define SPI_PSP_CTRL_SFRMP              SPI_PSP_CTRL_SFRMP_Msk
#define SPI_PSP_CTRL_SFRMDLY_Pos        (5U)
#define SPI_PSP_CTRL_SFRMDLY_Msk        (0x7FUL << SPI_PSP_CTRL_SFRMDLY_Pos)
#define SPI_PSP_CTRL_SFRMDLY            SPI_PSP_CTRL_SFRMDLY_Msk
#define SPI_PSP_CTRL_SFRMWDTH_Pos       (12U)
#define SPI_PSP_CTRL_SFRMWDTH_Msk       (0x3FUL << SPI_PSP_CTRL_SFRMWDTH_Pos)
#define SPI_PSP_CTRL_SFRMWDTH           SPI_PSP_CTRL_SFRMWDTH_Msk
#define SPI_PSP_CTRL_STRTDLY_Pos        (18U)
#define SPI_PSP_CTRL_STRTDLY_Msk        (0x7UL << SPI_PSP_CTRL_STRTDLY_Pos)
#define SPI_PSP_CTRL_STRTDLY            SPI_PSP_CTRL_STRTDLY_Msk
#define SPI_PSP_CTRL_DMYSTRT_Pos        (21U)
#define SPI_PSP_CTRL_DMYSTRT_Msk        (0x3UL << SPI_PSP_CTRL_DMYSTRT_Pos)
#define SPI_PSP_CTRL_DMYSTRT            SPI_PSP_CTRL_DMYSTRT_Msk
#define SPI_PSP_CTRL_EDMYSTRT_Pos       (23U)
#define SPI_PSP_CTRL_EDMYSTRT_Msk       (0x3UL << SPI_PSP_CTRL_EDMYSTRT_Pos)
#define SPI_PSP_CTRL_EDMYSTRT           SPI_PSP_CTRL_EDMYSTRT_Msk
#define SPI_PSP_CTRL_DMYSTOP_Pos        (25U)
#define SPI_PSP_CTRL_DMYSTOP_Msk        (0x3UL << SPI_PSP_CTRL_DMYSTOP_Pos)
#define SPI_PSP_CTRL_DMYSTOP            SPI_PSP_CTRL_DMYSTOP_Msk
#define SPI_PSP_CTRL_EDMYSTOP_Pos       (27U)
#define SPI_PSP_CTRL_EDMYSTOP_Msk       (0x7UL << SPI_PSP_CTRL_EDMYSTOP_Pos)
#define SPI_PSP_CTRL_EDMYSTOP           SPI_PSP_CTRL_EDMYSTOP_Msk

/****************** Bit definition for SPI_NW_CTRL register *******************/
#define SPI_NW_CTRL_NET_WORK_MOD_Pos    (0U)
#define SPI_NW_CTRL_NET_WORK_MOD_Msk    (0x1UL << SPI_NW_CTRL_NET_WORK_MOD_Pos)
#define SPI_NW_CTRL_NET_WORK_MOD        SPI_NW_CTRL_NET_WORK_MOD_Msk
#define SPI_NW_CTRL_FRDC_Pos            (1U)
#define SPI_NW_CTRL_FRDC_Msk            (0x7UL << SPI_NW_CTRL_FRDC_Pos)
#define SPI_NW_CTRL_FRDC                SPI_NW_CTRL_FRDC_Msk
#define SPI_NW_CTRL_TTSA_Pos            (4U)
#define SPI_NW_CTRL_TTSA_Msk            (0xFFUL << SPI_NW_CTRL_TTSA_Pos)
#define SPI_NW_CTRL_TTSA                SPI_NW_CTRL_TTSA_Msk
#define SPI_NW_CTRL_RSTA_Pos            (12U)
#define SPI_NW_CTRL_RSTA_Msk            (0xFFUL << SPI_NW_CTRL_RSTA_Pos)
#define SPI_NW_CTRL_RSTA                SPI_NW_CTRL_RSTA_Msk

/***************** Bit definition for SPI_NW_STATUS register ******************/
#define SPI_NW_STATUS_TSS_Pos           (0U)
#define SPI_NW_STATUS_TSS_Msk           (0x7UL << SPI_NW_STATUS_TSS_Pos)
#define SPI_NW_STATUS_TSS               SPI_NW_STATUS_TSS_Msk
#define SPI_NW_STATUS_NMBSY_Pos         (3U)
#define SPI_NW_STATUS_NMBSY_Msk         (0x1UL << SPI_NW_STATUS_NMBSY_Pos)
#define SPI_NW_STATUS_NMBSY             SPI_NW_STATUS_NMBSY_Msk

/***************** Bit definition for SPI_RWOT_CTRL register ******************/
#define SPI_RWOT_CTRL_RWOT_Pos          (0U)
#define SPI_RWOT_CTRL_RWOT_Msk          (0x1UL << SPI_RWOT_CTRL_RWOT_Pos)
#define SPI_RWOT_CTRL_RWOT              SPI_RWOT_CTRL_RWOT_Msk
#define SPI_RWOT_CTRL_CYCLE_RWOT_EN_Pos  (1U)
#define SPI_RWOT_CTRL_CYCLE_RWOT_EN_Msk  (0x1UL << SPI_RWOT_CTRL_CYCLE_RWOT_EN_Pos)
#define SPI_RWOT_CTRL_CYCLE_RWOT_EN     SPI_RWOT_CTRL_CYCLE_RWOT_EN_Msk
#define SPI_RWOT_CTRL_SET_RWOT_CYCLE_Pos  (2U)
#define SPI_RWOT_CTRL_SET_RWOT_CYCLE_Msk  (0x1UL << SPI_RWOT_CTRL_SET_RWOT_CYCLE_Pos)
#define SPI_RWOT_CTRL_SET_RWOT_CYCLE    SPI_RWOT_CTRL_SET_RWOT_CYCLE_Msk
#define SPI_RWOT_CTRL_CLR_RWOT_CYCLE_Pos  (3U)
#define SPI_RWOT_CTRL_CLR_RWOT_CYCLE_Msk  (0x1UL << SPI_RWOT_CTRL_CLR_RWOT_CYCLE_Pos)
#define SPI_RWOT_CTRL_CLR_RWOT_CYCLE    SPI_RWOT_CTRL_CLR_RWOT_CYCLE_Msk
#define SPI_RWOT_CTRL_MASK_RWOT_LAST_SAMPLE_Pos  (4U)
#define SPI_RWOT_CTRL_MASK_RWOT_LAST_SAMPLE_Msk  (0x1UL << SPI_RWOT_CTRL_MASK_RWOT_LAST_SAMPLE_Pos)
#define SPI_RWOT_CTRL_MASK_RWOT_LAST_SAMPLE  SPI_RWOT_CTRL_MASK_RWOT_LAST_SAMPLE_Msk

/****************** Bit definition for SPI_RWOT_CCM register ******************/
#define SPI_RWOT_CCM_SSPRWOTCCM_Pos     (0U)
#define SPI_RWOT_CCM_SSPRWOTCCM_Msk     (0xFFFFFFFFUL << SPI_RWOT_CCM_SSPRWOTCCM_Pos)
#define SPI_RWOT_CCM_SSPRWOTCCM         SPI_RWOT_CCM_SSPRWOTCCM_Msk

/***************** Bit definition for SPI_RWOT_CVWRN register *****************/
#define SPI_RWOT_CVWRN_SSPRWOTCVWR_Pos  (0U)
#define SPI_RWOT_CVWRN_SSPRWOTCVWR_Msk  (0xFFFFFFFFUL << SPI_RWOT_CVWRN_SSPRWOTCVWR_Pos)
#define SPI_RWOT_CVWRN_SSPRWOTCVWR      SPI_RWOT_CVWRN_SSPRWOTCVWR_Msk

/****************** Bit definition for SPI_CLK_CTRL register ******************/
#define SPI_CLK_CTRL_CLK_DIV_Pos        (0U)
#define SPI_CLK_CTRL_CLK_DIV_Msk        (0x7FUL << SPI_CLK_CTRL_CLK_DIV_Pos)
#define SPI_CLK_CTRL_CLK_DIV            SPI_CLK_CTRL_CLK_DIV_Msk
#define SPI_CLK_CTRL_CLK_SEL_Pos        (7U)
#define SPI_CLK_CTRL_CLK_SEL_Msk        (0x1UL << SPI_CLK_CTRL_CLK_SEL_Pos)
#define SPI_CLK_CTRL_CLK_SEL            SPI_CLK_CTRL_CLK_SEL_Msk

/**************** Bit definition for SPI_TRIWIRE_CTRL register ****************/
#define SPI_TRIWIRE_CTRL_SPI_TRI_WIRE_EN_Pos  (0U)
#define SPI_TRIWIRE_CTRL_SPI_TRI_WIRE_EN_Msk  (0x1UL << SPI_TRIWIRE_CTRL_SPI_TRI_WIRE_EN_Pos)
#define SPI_TRIWIRE_CTRL_SPI_TRI_WIRE_EN  SPI_TRIWIRE_CTRL_SPI_TRI_WIRE_EN_Msk
#define SPI_TRIWIRE_CTRL_TXD_OEN_Pos    (1U)
#define SPI_TRIWIRE_CTRL_TXD_OEN_Msk    (0x1UL << SPI_TRIWIRE_CTRL_TXD_OEN_Pos)
#define SPI_TRIWIRE_CTRL_TXD_OEN        SPI_TRIWIRE_CTRL_TXD_OEN_Msk
#define SPI_TRIWIRE_CTRL_SSP_WORK_WIDTH_DYN_CHANGE_Pos  (2U)
#define SPI_TRIWIRE_CTRL_SSP_WORK_WIDTH_DYN_CHANGE_Msk  (0x1UL << SPI_TRIWIRE_CTRL_SSP_WORK_WIDTH_DYN_CHANGE_Pos)
#define SPI_TRIWIRE_CTRL_SSP_WORK_WIDTH_DYN_CHANGE  SPI_TRIWIRE_CTRL_SSP_WORK_WIDTH_DYN_CHANGE_Msk

#endif
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
