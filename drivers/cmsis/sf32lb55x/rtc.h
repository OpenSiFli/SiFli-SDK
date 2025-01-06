/**
  ******************************************************************************
  * @file   rtc.h
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

#ifndef __RTC_H
#define __RTC_H

typedef struct
{
    __IO uint32_t TR;
    __IO uint32_t DR;
    __IO uint32_t CR;
    __IO uint32_t ISR;
    __IO uint32_t PSCLR;
    __IO uint32_t WUTR;
    __IO uint32_t ALRMTR;
    __IO uint32_t ALRMDR;
    __IO uint32_t SHIFTR;
    __IO uint32_t TSTR;
    __IO uint32_t TSDR;
    __IO uint32_t OR;
    __IO uint32_t BKP0R;
    __IO uint32_t BKP1R;
    __IO uint32_t BKP2R;
    __IO uint32_t BKP3R;
    __IO uint32_t BKP4R;
    __IO uint32_t BKP5R;
    __IO uint32_t BKP6R;
    __IO uint32_t BKP7R;
    __IO uint32_t BKP8R;
    __IO uint32_t BKP9R;
    __IO uint32_t BKP10R;
    __IO uint32_t BKP11R;
    __IO uint32_t BKP12R;
    __IO uint32_t BKP13R;
    __IO uint32_t BKP14R;
    __IO uint32_t BKP15R;
    __IO uint32_t BKP16R;
    __IO uint32_t BKP17R;
    __IO uint32_t BKP18R;
    __IO uint32_t BKP19R;
    __IO uint32_t BKP20R;
    __IO uint32_t BKP21R;
    __IO uint32_t BKP22R;
    __IO uint32_t BKP23R;
    __IO uint32_t BKP24R;
    __IO uint32_t BKP25R;
    __IO uint32_t BKP26R;
    __IO uint32_t BKP27R;
    __IO uint32_t BKP28R;
    __IO uint32_t BKP29R;
    __IO uint32_t BKP30R;
    __IO uint32_t BKP31R;
} RTC_TypeDef;


/********************* Bit definition for RTC_TR register *********************/
#define RTC_TR_SS_Pos                   (0U)
#define RTC_TR_SS_Msk                   (0x3FFUL << RTC_TR_SS_Pos)
#define RTC_TR_SS                       RTC_TR_SS_Msk
#define RTC_TR_SU_Pos                   (11U)
#define RTC_TR_SU_Msk                   (0xFUL << RTC_TR_SU_Pos)
#define RTC_TR_SU                       RTC_TR_SU_Msk
#define RTC_TR_ST_Pos                   (15U)
#define RTC_TR_ST_Msk                   (0x7UL << RTC_TR_ST_Pos)
#define RTC_TR_ST                       RTC_TR_ST_Msk
#define RTC_TR_MNU_Pos                  (18U)
#define RTC_TR_MNU_Msk                  (0xFUL << RTC_TR_MNU_Pos)
#define RTC_TR_MNU                      RTC_TR_MNU_Msk
#define RTC_TR_MNT_Pos                  (22U)
#define RTC_TR_MNT_Msk                  (0x7UL << RTC_TR_MNT_Pos)
#define RTC_TR_MNT                      RTC_TR_MNT_Msk
#define RTC_TR_HU_Pos                   (25U)
#define RTC_TR_HU_Msk                   (0xFUL << RTC_TR_HU_Pos)
#define RTC_TR_HU                       RTC_TR_HU_Msk
#define RTC_TR_HT_Pos                   (29U)
#define RTC_TR_HT_Msk                   (0x3UL << RTC_TR_HT_Pos)
#define RTC_TR_HT                       RTC_TR_HT_Msk
#define RTC_TR_PM_Pos                   (31U)
#define RTC_TR_PM_Msk                   (0x1UL << RTC_TR_PM_Pos)
#define RTC_TR_PM                       RTC_TR_PM_Msk

/********************* Bit definition for RTC_DR register *********************/
#define RTC_DR_DU_Pos                   (0U)
#define RTC_DR_DU_Msk                   (0xFUL << RTC_DR_DU_Pos)
#define RTC_DR_DU                       RTC_DR_DU_Msk
#define RTC_DR_DT_Pos                   (4U)
#define RTC_DR_DT_Msk                   (0x3UL << RTC_DR_DT_Pos)
#define RTC_DR_DT                       RTC_DR_DT_Msk
#define RTC_DR_MU_Pos                   (8U)
#define RTC_DR_MU_Msk                   (0xFUL << RTC_DR_MU_Pos)
#define RTC_DR_MU                       RTC_DR_MU_Msk
#define RTC_DR_MT_Pos                   (12U)
#define RTC_DR_MT_Msk                   (0x1UL << RTC_DR_MT_Pos)
#define RTC_DR_MT                       RTC_DR_MT_Msk
#define RTC_DR_WD_Pos                   (13U)
#define RTC_DR_WD_Msk                   (0x7UL << RTC_DR_WD_Pos)
#define RTC_DR_WD                       RTC_DR_WD_Msk
#define RTC_DR_YU_Pos                   (16U)
#define RTC_DR_YU_Msk                   (0xFUL << RTC_DR_YU_Pos)
#define RTC_DR_YU                       RTC_DR_YU_Msk
#define RTC_DR_YT_Pos                   (20U)
#define RTC_DR_YT_Msk                   (0xFUL << RTC_DR_YT_Pos)
#define RTC_DR_YT                       RTC_DR_YT_Msk
#define RTC_DR_CB_Pos                   (24U)
#define RTC_DR_CB_Msk                   (0x1UL << RTC_DR_CB_Pos)
#define RTC_DR_CB                       RTC_DR_CB_Msk
#define RTC_DR_ERR_Pos                  (31U)
#define RTC_DR_ERR_Msk                  (0x1UL << RTC_DR_ERR_Pos)
#define RTC_DR_ERR                      RTC_DR_ERR_Msk

/********************* Bit definition for RTC_CR register *********************/
#define RTC_CR_WUCKSEL_Pos              (0U)
#define RTC_CR_WUCKSEL_Msk              (0x1UL << RTC_CR_WUCKSEL_Pos)
#define RTC_CR_WUCKSEL                  RTC_CR_WUCKSEL_Msk
#define RTC_CR_TSEDGE_Pos               (3U)
#define RTC_CR_TSEDGE_Msk               (0x1UL << RTC_CR_TSEDGE_Pos)
#define RTC_CR_TSEDGE                   RTC_CR_TSEDGE_Msk
#define RTC_CR_REFCKON_Pos              (4U)
#define RTC_CR_REFCKON_Msk              (0x1UL << RTC_CR_REFCKON_Pos)
#define RTC_CR_REFCKON                  RTC_CR_REFCKON_Msk
#define RTC_CR_BYPSHAD_Pos              (5U)
#define RTC_CR_BYPSHAD_Msk              (0x1UL << RTC_CR_BYPSHAD_Pos)
#define RTC_CR_BYPSHAD                  RTC_CR_BYPSHAD_Msk
#define RTC_CR_FMT_Pos                  (6U)
#define RTC_CR_FMT_Msk                  (0x1UL << RTC_CR_FMT_Pos)
#define RTC_CR_FMT                      RTC_CR_FMT_Msk
#define RTC_CR_ALRME_Pos                (8U)
#define RTC_CR_ALRME_Msk                (0x1UL << RTC_CR_ALRME_Pos)
#define RTC_CR_ALRME                    RTC_CR_ALRME_Msk
#define RTC_CR_WUTE_Pos                 (9U)
#define RTC_CR_WUTE_Msk                 (0x1UL << RTC_CR_WUTE_Pos)
#define RTC_CR_WUTE                     RTC_CR_WUTE_Msk
#define RTC_CR_TSE_Pos                  (10U)
#define RTC_CR_TSE_Msk                  (0x1UL << RTC_CR_TSE_Pos)
#define RTC_CR_TSE                      RTC_CR_TSE_Msk
#define RTC_CR_ALRMIE_Pos               (11U)
#define RTC_CR_ALRMIE_Msk               (0x1UL << RTC_CR_ALRMIE_Pos)
#define RTC_CR_ALRMIE                   RTC_CR_ALRMIE_Msk
#define RTC_CR_WUTIE_Pos                (12U)
#define RTC_CR_WUTIE_Msk                (0x1UL << RTC_CR_WUTIE_Pos)
#define RTC_CR_WUTIE                    RTC_CR_WUTIE_Msk
#define RTC_CR_TSIE_Pos                 (13U)
#define RTC_CR_TSIE_Msk                 (0x1UL << RTC_CR_TSIE_Pos)
#define RTC_CR_TSIE                     RTC_CR_TSIE_Msk
#define RTC_CR_ADD1H_Pos                (14U)
#define RTC_CR_ADD1H_Msk                (0x1UL << RTC_CR_ADD1H_Pos)
#define RTC_CR_ADD1H                    RTC_CR_ADD1H_Msk
#define RTC_CR_SUB1H_Pos                (15U)
#define RTC_CR_SUB1H_Msk                (0x1UL << RTC_CR_SUB1H_Pos)
#define RTC_CR_SUB1H                    RTC_CR_SUB1H_Msk
#define RTC_CR_BKP_Pos                  (16U)
#define RTC_CR_BKP_Msk                  (0x1UL << RTC_CR_BKP_Pos)
#define RTC_CR_BKP                      RTC_CR_BKP_Msk
#define RTC_CR_COSEL_Pos                (17U)
#define RTC_CR_COSEL_Msk                (0x1UL << RTC_CR_COSEL_Pos)
#define RTC_CR_COSEL                    RTC_CR_COSEL_Msk
#define RTC_CR_POL_Pos                  (18U)
#define RTC_CR_POL_Msk                  (0x1UL << RTC_CR_POL_Pos)
#define RTC_CR_POL                      RTC_CR_POL_Msk
#define RTC_CR_OSEL_Pos                 (19U)
#define RTC_CR_OSEL_Msk                 (0x3UL << RTC_CR_OSEL_Pos)
#define RTC_CR_OSEL                     RTC_CR_OSEL_Msk
#define RTC_CR_COE_Pos                  (21U)
#define RTC_CR_COE_Msk                  (0x1UL << RTC_CR_COE_Pos)
#define RTC_CR_COE                      RTC_CR_COE_Msk

/******************** Bit definition for RTC_ISR register *********************/
#define RTC_ISR_ALRMWF_Pos              (0U)
#define RTC_ISR_ALRMWF_Msk              (0x1UL << RTC_ISR_ALRMWF_Pos)
#define RTC_ISR_ALRMWF                  RTC_ISR_ALRMWF_Msk
#define RTC_ISR_ALRMF_Pos               (1U)
#define RTC_ISR_ALRMF_Msk               (0x1UL << RTC_ISR_ALRMF_Pos)
#define RTC_ISR_ALRMF                   RTC_ISR_ALRMF_Msk
#define RTC_ISR_WUTWF_Pos               (2U)
#define RTC_ISR_WUTWF_Msk               (0x1UL << RTC_ISR_WUTWF_Pos)
#define RTC_ISR_WUTWF                   RTC_ISR_WUTWF_Msk
#define RTC_ISR_WUTF_Pos                (3U)
#define RTC_ISR_WUTF_Msk                (0x1UL << RTC_ISR_WUTF_Pos)
#define RTC_ISR_WUTF                    RTC_ISR_WUTF_Msk
#define RTC_ISR_TSF_Pos                 (4U)
#define RTC_ISR_TSF_Msk                 (0x1UL << RTC_ISR_TSF_Pos)
#define RTC_ISR_TSF                     RTC_ISR_TSF_Msk
#define RTC_ISR_TSOVF_Pos               (5U)
#define RTC_ISR_TSOVF_Msk               (0x1UL << RTC_ISR_TSOVF_Pos)
#define RTC_ISR_TSOVF                   RTC_ISR_TSOVF_Msk
#define RTC_ISR_SHPF_Pos                (6U)
#define RTC_ISR_SHPF_Msk                (0x1UL << RTC_ISR_SHPF_Pos)
#define RTC_ISR_SHPF                    RTC_ISR_SHPF_Msk
#define RTC_ISR_RSF_Pos                 (7U)
#define RTC_ISR_RSF_Msk                 (0x1UL << RTC_ISR_RSF_Pos)
#define RTC_ISR_RSF                     RTC_ISR_RSF_Msk
#define RTC_ISR_INITS_Pos               (8U)
#define RTC_ISR_INITS_Msk               (0x1UL << RTC_ISR_INITS_Pos)
#define RTC_ISR_INITS                   RTC_ISR_INITS_Msk
#define RTC_ISR_INITF_Pos               (9U)
#define RTC_ISR_INITF_Msk               (0x1UL << RTC_ISR_INITF_Pos)
#define RTC_ISR_INITF                   RTC_ISR_INITF_Msk
#define RTC_ISR_INIT_Pos                (10U)
#define RTC_ISR_INIT_Msk                (0x1UL << RTC_ISR_INIT_Pos)
#define RTC_ISR_INIT                    RTC_ISR_INIT_Msk

/******************* Bit definition for RTC_PSCLR register ********************/
#define RTC_PSCLR_DIVB_Pos              (0U)
#define RTC_PSCLR_DIVB_Msk              (0x3FFUL << RTC_PSCLR_DIVB_Pos)
#define RTC_PSCLR_DIVB                  RTC_PSCLR_DIVB_Msk
#define RTC_PSCLR_DIVA_FRAC_Pos         (10U)
#define RTC_PSCLR_DIVA_FRAC_Msk         (0x3FFFUL << RTC_PSCLR_DIVA_FRAC_Pos)
#define RTC_PSCLR_DIVA_FRAC             RTC_PSCLR_DIVA_FRAC_Msk
#define RTC_PSCLR_DIVA_INT_Pos          (24U)
#define RTC_PSCLR_DIVA_INT_Msk          (0xFFUL << RTC_PSCLR_DIVA_INT_Pos)
#define RTC_PSCLR_DIVA_INT              RTC_PSCLR_DIVA_INT_Msk

/******************** Bit definition for RTC_WUTR register ********************/
#define RTC_WUTR_WUT_Pos                (0U)
#define RTC_WUTR_WUT_Msk                (0x3FFFFUL << RTC_WUTR_WUT_Pos)
#define RTC_WUTR_WUT                    RTC_WUTR_WUT_Msk

/******************* Bit definition for RTC_ALRMTR register *******************/
#define RTC_ALRMTR_SS_Pos               (0U)
#define RTC_ALRMTR_SS_Msk               (0x3FFUL << RTC_ALRMTR_SS_Pos)
#define RTC_ALRMTR_SS                   RTC_ALRMTR_SS_Msk
#define RTC_ALRMTR_SU_Pos               (11U)
#define RTC_ALRMTR_SU_Msk               (0xFUL << RTC_ALRMTR_SU_Pos)
#define RTC_ALRMTR_SU                   RTC_ALRMTR_SU_Msk
#define RTC_ALRMTR_ST_Pos               (15U)
#define RTC_ALRMTR_ST_Msk               (0x7UL << RTC_ALRMTR_ST_Pos)
#define RTC_ALRMTR_ST                   RTC_ALRMTR_ST_Msk
#define RTC_ALRMTR_MNU_Pos              (18U)
#define RTC_ALRMTR_MNU_Msk              (0xFUL << RTC_ALRMTR_MNU_Pos)
#define RTC_ALRMTR_MNU                  RTC_ALRMTR_MNU_Msk
#define RTC_ALRMTR_MNT_Pos              (22U)
#define RTC_ALRMTR_MNT_Msk              (0x7UL << RTC_ALRMTR_MNT_Pos)
#define RTC_ALRMTR_MNT                  RTC_ALRMTR_MNT_Msk
#define RTC_ALRMTR_HU_Pos               (25U)
#define RTC_ALRMTR_HU_Msk               (0xFUL << RTC_ALRMTR_HU_Pos)
#define RTC_ALRMTR_HU                   RTC_ALRMTR_HU_Msk
#define RTC_ALRMTR_HT_Pos               (29U)
#define RTC_ALRMTR_HT_Msk               (0x3UL << RTC_ALRMTR_HT_Pos)
#define RTC_ALRMTR_HT                   RTC_ALRMTR_HT_Msk
#define RTC_ALRMTR_PM_Pos               (31U)
#define RTC_ALRMTR_PM_Msk               (0x1UL << RTC_ALRMTR_PM_Pos)
#define RTC_ALRMTR_PM                   RTC_ALRMTR_PM_Msk

/******************* Bit definition for RTC_ALRMDR register *******************/
#define RTC_ALRMDR_DU_Pos               (0U)
#define RTC_ALRMDR_DU_Msk               (0xFUL << RTC_ALRMDR_DU_Pos)
#define RTC_ALRMDR_DU                   RTC_ALRMDR_DU_Msk
#define RTC_ALRMDR_DT_Pos               (4U)
#define RTC_ALRMDR_DT_Msk               (0x3UL << RTC_ALRMDR_DT_Pos)
#define RTC_ALRMDR_DT                   RTC_ALRMDR_DT_Msk
#define RTC_ALRMDR_MU_Pos               (8U)
#define RTC_ALRMDR_MU_Msk               (0xFUL << RTC_ALRMDR_MU_Pos)
#define RTC_ALRMDR_MU                   RTC_ALRMDR_MU_Msk
#define RTC_ALRMDR_MT_Pos               (12U)
#define RTC_ALRMDR_MT_Msk               (0x1UL << RTC_ALRMDR_MT_Pos)
#define RTC_ALRMDR_MT                   RTC_ALRMDR_MT_Msk
#define RTC_ALRMDR_WD_Pos               (13U)
#define RTC_ALRMDR_WD_Msk               (0x7UL << RTC_ALRMDR_WD_Pos)
#define RTC_ALRMDR_WD                   RTC_ALRMDR_WD_Msk
#define RTC_ALRMDR_MSKSS_Pos            (20U)
#define RTC_ALRMDR_MSKSS_Msk            (0xFUL << RTC_ALRMDR_MSKSS_Pos)
#define RTC_ALRMDR_MSKSS                RTC_ALRMDR_MSKSS_Msk
#define RTC_ALRMDR_MSKS_Pos             (24U)
#define RTC_ALRMDR_MSKS_Msk             (0x1UL << RTC_ALRMDR_MSKS_Pos)
#define RTC_ALRMDR_MSKS                 RTC_ALRMDR_MSKS_Msk
#define RTC_ALRMDR_MSKMN_Pos            (25U)
#define RTC_ALRMDR_MSKMN_Msk            (0x1UL << RTC_ALRMDR_MSKMN_Pos)
#define RTC_ALRMDR_MSKMN                RTC_ALRMDR_MSKMN_Msk
#define RTC_ALRMDR_MSKH_Pos             (26U)
#define RTC_ALRMDR_MSKH_Msk             (0x1UL << RTC_ALRMDR_MSKH_Pos)
#define RTC_ALRMDR_MSKH                 RTC_ALRMDR_MSKH_Msk
#define RTC_ALRMDR_MSKD_Pos             (27U)
#define RTC_ALRMDR_MSKD_Msk             (0x1UL << RTC_ALRMDR_MSKD_Pos)
#define RTC_ALRMDR_MSKD                 RTC_ALRMDR_MSKD_Msk
#define RTC_ALRMDR_MSKM_Pos             (28U)
#define RTC_ALRMDR_MSKM_Msk             (0x1UL << RTC_ALRMDR_MSKM_Pos)
#define RTC_ALRMDR_MSKM                 RTC_ALRMDR_MSKM_Msk
#define RTC_ALRMDR_MSKWD_Pos            (29U)
#define RTC_ALRMDR_MSKWD_Msk            (0x1UL << RTC_ALRMDR_MSKWD_Pos)
#define RTC_ALRMDR_MSKWD                RTC_ALRMDR_MSKWD_Msk

/******************* Bit definition for RTC_SHIFTR register *******************/
#define RTC_SHIFTR_SUBFS_Pos            (0U)
#ifdef SF32LB55X
    #define RTC_SHIFTR_SUBFS_Msk            (0x7FFFUL << RTC_SHIFTR_SUBFS_Pos)
#else
    #define RTC_SHIFTR_SUBFS_Msk            (0x3FFUL << RTC_SHIFTR_SUBFS_Pos)
#endif /* SF32LB55X */
#define RTC_SHIFTR_SUBFS                RTC_SHIFTR_SUBFS_Msk
#define RTC_SHIFTR_ADD1S_Pos            (31U)
#define RTC_SHIFTR_ADD1S_Msk            (0x1UL << RTC_SHIFTR_ADD1S_Pos)
#define RTC_SHIFTR_ADD1S                RTC_SHIFTR_ADD1S_Msk

/******************** Bit definition for RTC_TSTR register ********************/
#define RTC_TSTR_SS_Pos                 (0U)
#define RTC_TSTR_SS_Msk                 (0x3FFUL << RTC_TSTR_SS_Pos)
#define RTC_TSTR_SS                     RTC_TSTR_SS_Msk
#define RTC_TSTR_SU_Pos                 (11U)
#define RTC_TSTR_SU_Msk                 (0xFUL << RTC_TSTR_SU_Pos)
#define RTC_TSTR_SU                     RTC_TSTR_SU_Msk
#define RTC_TSTR_ST_Pos                 (15U)
#define RTC_TSTR_ST_Msk                 (0x7UL << RTC_TSTR_ST_Pos)
#define RTC_TSTR_ST                     RTC_TSTR_ST_Msk
#define RTC_TSTR_MNU_Pos                (18U)
#define RTC_TSTR_MNU_Msk                (0xFUL << RTC_TSTR_MNU_Pos)
#define RTC_TSTR_MNU                    RTC_TSTR_MNU_Msk
#define RTC_TSTR_MNT_Pos                (22U)
#define RTC_TSTR_MNT_Msk                (0x7UL << RTC_TSTR_MNT_Pos)
#define RTC_TSTR_MNT                    RTC_TSTR_MNT_Msk
#define RTC_TSTR_HU_Pos                 (25U)
#define RTC_TSTR_HU_Msk                 (0xFUL << RTC_TSTR_HU_Pos)
#define RTC_TSTR_HU                     RTC_TSTR_HU_Msk
#define RTC_TSTR_HT_Pos                 (29U)
#define RTC_TSTR_HT_Msk                 (0x3UL << RTC_TSTR_HT_Pos)
#define RTC_TSTR_HT                     RTC_TSTR_HT_Msk
#define RTC_TSTR_PM_Pos                 (31U)
#define RTC_TSTR_PM_Msk                 (0x1UL << RTC_TSTR_PM_Pos)
#define RTC_TSTR_PM                     RTC_TSTR_PM_Msk

/******************** Bit definition for RTC_TSDR register ********************/
#define RTC_TSDR_DU_Pos                 (0U)
#define RTC_TSDR_DU_Msk                 (0xFUL << RTC_TSDR_DU_Pos)
#define RTC_TSDR_DU                     RTC_TSDR_DU_Msk
#define RTC_TSDR_DT_Pos                 (4U)
#define RTC_TSDR_DT_Msk                 (0x3UL << RTC_TSDR_DT_Pos)
#define RTC_TSDR_DT                     RTC_TSDR_DT_Msk
#define RTC_TSDR_MU_Pos                 (8U)
#define RTC_TSDR_MU_Msk                 (0xFUL << RTC_TSDR_MU_Pos)
#define RTC_TSDR_MU                     RTC_TSDR_MU_Msk
#define RTC_TSDR_MT_Pos                 (12U)
#define RTC_TSDR_MT_Msk                 (0x1UL << RTC_TSDR_MT_Pos)
#define RTC_TSDR_MT                     RTC_TSDR_MT_Msk
#define RTC_TSDR_WD_Pos                 (13U)
#define RTC_TSDR_WD_Msk                 (0x7UL << RTC_TSDR_WD_Pos)
#define RTC_TSDR_WD                     RTC_TSDR_WD_Msk

/********************* Bit definition for RTC_OR register *********************/
#define RTC_OR_RTC_ALARM_TYPE_Pos       (0U)
#define RTC_OR_RTC_ALARM_TYPE_Msk       (0x1UL << RTC_OR_RTC_ALARM_TYPE_Pos)
#define RTC_OR_RTC_ALARM_TYPE           RTC_OR_RTC_ALARM_TYPE_Msk
#define RTC_OR_RTC_OUT_RMP_Pos          (1U)
#define RTC_OR_RTC_OUT_RMP_Msk          (0x1UL << RTC_OR_RTC_OUT_RMP_Pos)
#define RTC_OR_RTC_OUT_RMP              RTC_OR_RTC_OUT_RMP_Msk

/******************* Bit definition for RTC_BKP0R register ********************/
#define RTC_BKP0R_BKP_Pos               (0U)
#define RTC_BKP0R_BKP_Msk               (0xFFFFFFFFUL << RTC_BKP0R_BKP_Pos)
#define RTC_BKP0R_BKP                   RTC_BKP0R_BKP_Msk

/******************* Bit definition for RTC_BKP1R register ********************/
#define RTC_BKP1R_BKP_Pos               (0U)
#define RTC_BKP1R_BKP_Msk               (0xFFFFFFFFUL << RTC_BKP1R_BKP_Pos)
#define RTC_BKP1R_BKP                   RTC_BKP1R_BKP_Msk

/******************* Bit definition for RTC_BKP2R register ********************/
#define RTC_BKP2R_BKP_Pos               (0U)
#define RTC_BKP2R_BKP_Msk               (0xFFFFFFFFUL << RTC_BKP2R_BKP_Pos)
#define RTC_BKP2R_BKP                   RTC_BKP2R_BKP_Msk

/******************* Bit definition for RTC_BKP3R register ********************/
#define RTC_BKP3R_BKP_Pos               (0U)
#define RTC_BKP3R_BKP_Msk               (0xFFFFFFFFUL << RTC_BKP3R_BKP_Pos)
#define RTC_BKP3R_BKP                   RTC_BKP3R_BKP_Msk

/******************* Bit definition for RTC_BKP4R register ********************/
#define RTC_BKP4R_BKP_Pos               (0U)
#define RTC_BKP4R_BKP_Msk               (0xFFFFFFFFUL << RTC_BKP4R_BKP_Pos)
#define RTC_BKP4R_BKP                   RTC_BKP4R_BKP_Msk

/******************* Bit definition for RTC_BKP5R register ********************/
#define RTC_BKP5R_BKP_Pos               (0U)
#define RTC_BKP5R_BKP_Msk               (0xFFFFFFFFUL << RTC_BKP5R_BKP_Pos)
#define RTC_BKP5R_BKP                   RTC_BKP5R_BKP_Msk

/******************* Bit definition for RTC_BKP6R register ********************/
#define RTC_BKP6R_BKP_Pos               (0U)
#define RTC_BKP6R_BKP_Msk               (0xFFFFFFFFUL << RTC_BKP6R_BKP_Pos)
#define RTC_BKP6R_BKP                   RTC_BKP6R_BKP_Msk

/******************* Bit definition for RTC_BKP7R register ********************/
#define RTC_BKP7R_BKP_Pos               (0U)
#define RTC_BKP7R_BKP_Msk               (0xFFFFFFFFUL << RTC_BKP7R_BKP_Pos)
#define RTC_BKP7R_BKP                   RTC_BKP7R_BKP_Msk

/******************* Bit definition for RTC_BKP8R register ********************/
#define RTC_BKP8R_BKP_Pos               (0U)
#define RTC_BKP8R_BKP_Msk               (0xFFFFFFFFUL << RTC_BKP8R_BKP_Pos)
#define RTC_BKP8R_BKP                   RTC_BKP8R_BKP_Msk

/******************* Bit definition for RTC_BKP9R register ********************/
#define RTC_BKP9R_BKP_Pos               (0U)
#define RTC_BKP9R_BKP_Msk               (0xFFFFFFFFUL << RTC_BKP9R_BKP_Pos)
#define RTC_BKP9R_BKP                   RTC_BKP9R_BKP_Msk

/******************* Bit definition for RTC_BKP10R register *******************/
#define RTC_BKP10R_BKP_Pos              (0U)
#define RTC_BKP10R_BKP_Msk              (0xFFFFFFFFUL << RTC_BKP10R_BKP_Pos)
#define RTC_BKP10R_BKP                  RTC_BKP10R_BKP_Msk

/******************* Bit definition for RTC_BKP11R register *******************/
#define RTC_BKP11R_BKP_Pos              (0U)
#define RTC_BKP11R_BKP_Msk              (0xFFFFFFFFUL << RTC_BKP11R_BKP_Pos)
#define RTC_BKP11R_BKP                  RTC_BKP11R_BKP_Msk

/******************* Bit definition for RTC_BKP12R register *******************/
#define RTC_BKP12R_BKP_Pos              (0U)
#define RTC_BKP12R_BKP_Msk              (0xFFFFFFFFUL << RTC_BKP12R_BKP_Pos)
#define RTC_BKP12R_BKP                  RTC_BKP12R_BKP_Msk

/******************* Bit definition for RTC_BKP13R register *******************/
#define RTC_BKP13R_BKP_Pos              (0U)
#define RTC_BKP13R_BKP_Msk              (0xFFFFFFFFUL << RTC_BKP13R_BKP_Pos)
#define RTC_BKP13R_BKP                  RTC_BKP13R_BKP_Msk

/******************* Bit definition for RTC_BKP14R register *******************/
#define RTC_BKP14R_BKP_Pos              (0U)
#define RTC_BKP14R_BKP_Msk              (0xFFFFFFFFUL << RTC_BKP14R_BKP_Pos)
#define RTC_BKP14R_BKP                  RTC_BKP14R_BKP_Msk

/******************* Bit definition for RTC_BKP15R register *******************/
#define RTC_BKP15R_BKP_Pos              (0U)
#define RTC_BKP15R_BKP_Msk              (0xFFFFFFFFUL << RTC_BKP15R_BKP_Pos)
#define RTC_BKP15R_BKP                  RTC_BKP15R_BKP_Msk

/******************* Bit definition for RTC_BKP16R register *******************/
#define RTC_BKP16R_BKP_Pos              (0U)
#define RTC_BKP16R_BKP_Msk              (0xFFFFFFFFUL << RTC_BKP16R_BKP_Pos)
#define RTC_BKP16R_BKP                  RTC_BKP16R_BKP_Msk

/******************* Bit definition for RTC_BKP17R register *******************/
#define RTC_BKP17R_BKP_Pos              (0U)
#define RTC_BKP17R_BKP_Msk              (0xFFFFFFFFUL << RTC_BKP17R_BKP_Pos)
#define RTC_BKP17R_BKP                  RTC_BKP17R_BKP_Msk

/******************* Bit definition for RTC_BKP18R register *******************/
#define RTC_BKP18R_BKP_Pos              (0U)
#define RTC_BKP18R_BKP_Msk              (0xFFFFFFFFUL << RTC_BKP18R_BKP_Pos)
#define RTC_BKP18R_BKP                  RTC_BKP18R_BKP_Msk

/******************* Bit definition for RTC_BKP19R register *******************/
#define RTC_BKP19R_BKP_Pos              (0U)
#define RTC_BKP19R_BKP_Msk              (0xFFFFFFFFUL << RTC_BKP19R_BKP_Pos)
#define RTC_BKP19R_BKP                  RTC_BKP19R_BKP_Msk

/******************* Bit definition for RTC_BKP20R register *******************/
#define RTC_BKP20R_BKP_Pos              (0U)
#define RTC_BKP20R_BKP_Msk              (0xFFFFFFFFUL << RTC_BKP20R_BKP_Pos)
#define RTC_BKP20R_BKP                  RTC_BKP20R_BKP_Msk

/******************* Bit definition for RTC_BKP21R register *******************/
#define RTC_BKP21R_BKP_Pos              (0U)
#define RTC_BKP21R_BKP_Msk              (0xFFFFFFFFUL << RTC_BKP21R_BKP_Pos)
#define RTC_BKP21R_BKP                  RTC_BKP21R_BKP_Msk

/******************* Bit definition for RTC_BKP22R register *******************/
#define RTC_BKP22R_BKP_Pos              (0U)
#define RTC_BKP22R_BKP_Msk              (0xFFFFFFFFUL << RTC_BKP22R_BKP_Pos)
#define RTC_BKP22R_BKP                  RTC_BKP22R_BKP_Msk

/******************* Bit definition for RTC_BKP23R register *******************/
#define RTC_BKP23R_BKP_Pos              (0U)
#define RTC_BKP23R_BKP_Msk              (0xFFFFFFFFUL << RTC_BKP23R_BKP_Pos)
#define RTC_BKP23R_BKP                  RTC_BKP23R_BKP_Msk

/******************* Bit definition for RTC_BKP24R register *******************/
#define RTC_BKP24R_BKP_Pos              (0U)
#define RTC_BKP24R_BKP_Msk              (0xFFFFFFFFUL << RTC_BKP24R_BKP_Pos)
#define RTC_BKP24R_BKP                  RTC_BKP24R_BKP_Msk

/******************* Bit definition for RTC_BKP25R register *******************/
#define RTC_BKP25R_BKP_Pos              (0U)
#define RTC_BKP25R_BKP_Msk              (0xFFFFFFFFUL << RTC_BKP25R_BKP_Pos)
#define RTC_BKP25R_BKP                  RTC_BKP25R_BKP_Msk

/******************* Bit definition for RTC_BKP26R register *******************/
#define RTC_BKP26R_BKP_Pos              (0U)
#define RTC_BKP26R_BKP_Msk              (0xFFFFFFFFUL << RTC_BKP26R_BKP_Pos)
#define RTC_BKP26R_BKP                  RTC_BKP26R_BKP_Msk

/******************* Bit definition for RTC_BKP27R register *******************/
#define RTC_BKP27R_BKP_Pos              (0U)
#define RTC_BKP27R_BKP_Msk              (0xFFFFFFFFUL << RTC_BKP27R_BKP_Pos)
#define RTC_BKP27R_BKP                  RTC_BKP27R_BKP_Msk

/******************* Bit definition for RTC_BKP28R register *******************/
#define RTC_BKP28R_BKP_Pos              (0U)
#define RTC_BKP28R_BKP_Msk              (0xFFFFFFFFUL << RTC_BKP28R_BKP_Pos)
#define RTC_BKP28R_BKP                  RTC_BKP28R_BKP_Msk

/******************* Bit definition for RTC_BKP29R register *******************/
#define RTC_BKP29R_BKP_Pos              (0U)
#define RTC_BKP29R_BKP_Msk              (0xFFFFFFFFUL << RTC_BKP29R_BKP_Pos)
#define RTC_BKP29R_BKP                  RTC_BKP29R_BKP_Msk

/******************* Bit definition for RTC_BKP30R register *******************/
#define RTC_BKP30R_BKP_Pos              (0U)
#define RTC_BKP30R_BKP_Msk              (0xFFFFFFFFUL << RTC_BKP30R_BKP_Pos)
#define RTC_BKP30R_BKP                  RTC_BKP30R_BKP_Msk

/******************* Bit definition for RTC_BKP31R register *******************/
#define RTC_BKP31R_BKP_Pos              (0U)
#define RTC_BKP31R_BKP_Msk              (0xFFFFFFFFUL << RTC_BKP31R_BKP_Pos)
#define RTC_BKP31R_BKP                  RTC_BKP31R_BKP_Msk

#endif
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
