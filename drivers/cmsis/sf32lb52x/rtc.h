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
    __IO uint32_t PBRCR;
    __IO uint32_t PBR0R;
    __IO uint32_t PBR1R;
    __IO uint32_t PBR2R;
    __IO uint32_t PBR3R;
    __IO uint32_t PAWK1R;
    __IO uint32_t PAWK2R;
    __IO uint32_t PAWK3R;
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
#define RTC_CR_LPCKSEL_Pos              (0U)
#define RTC_CR_LPCKSEL_Msk              (0x1UL << RTC_CR_LPCKSEL_Pos)
#define RTC_CR_LPCKSEL                  RTC_CR_LPCKSEL_Msk
#define RTC_CR_WUCKSEL_Pos              (1U)
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
#define RTC_SHIFTR_SUBFS_Msk            (0x3FFUL << RTC_SHIFTR_SUBFS_Pos)
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

/******************* Bit definition for RTC_PBRCR register ********************/
#define RTC_PBRCR_RTO_Pos               (0U)
#define RTC_PBRCR_RTO_Msk               (0x1UL << RTC_PBRCR_RTO_Pos)
#define RTC_PBRCR_RTO                   RTC_PBRCR_RTO_Msk
#define RTC_PBRCR_SNS_Pos               (1U)
#define RTC_PBRCR_SNS_Msk               (0x1UL << RTC_PBRCR_SNS_Pos)
#define RTC_PBRCR_SNS                   RTC_PBRCR_SNS_Msk
#define RTC_PBRCR_DBG_SEL_Pos           (4U)
#define RTC_PBRCR_DBG_SEL_Msk           (0xFUL << RTC_PBRCR_DBG_SEL_Pos)
#define RTC_PBRCR_DBG_SEL               RTC_PBRCR_DBG_SEL_Msk

/******************* Bit definition for RTC_PBR0R register ********************/
#define RTC_PBR0R_OUT_Pos               (0U)
#define RTC_PBR0R_OUT_Msk               (0x1UL << RTC_PBR0R_OUT_Pos)
#define RTC_PBR0R_OUT                   RTC_PBR0R_OUT_Msk
#define RTC_PBR0R_OE_Pos                (1U)
#define RTC_PBR0R_OE_Msk                (0x1UL << RTC_PBR0R_OE_Pos)
#define RTC_PBR0R_OE                    RTC_PBR0R_OE_Msk
#define RTC_PBR0R_IE_Pos                (2U)
#define RTC_PBR0R_IE_Msk                (0x1UL << RTC_PBR0R_IE_Pos)
#define RTC_PBR0R_IE                    RTC_PBR0R_IE_Msk
#define RTC_PBR0R_PE_Pos                (3U)
#define RTC_PBR0R_PE_Msk                (0x1UL << RTC_PBR0R_PE_Pos)
#define RTC_PBR0R_PE                    RTC_PBR0R_PE_Msk
#define RTC_PBR0R_PS_Pos                (4U)
#define RTC_PBR0R_PS_Msk                (0x1UL << RTC_PBR0R_PS_Pos)
#define RTC_PBR0R_PS                    RTC_PBR0R_PS_Msk
#define RTC_PBR0R_IS_Pos                (5U)
#define RTC_PBR0R_IS_Msk                (0x1UL << RTC_PBR0R_IS_Pos)
#define RTC_PBR0R_IS                    RTC_PBR0R_IS_Msk
#define RTC_PBR0R_SR_Pos                (6U)
#define RTC_PBR0R_SR_Msk                (0x1UL << RTC_PBR0R_SR_Pos)
#define RTC_PBR0R_SR                    RTC_PBR0R_SR_Msk
#define RTC_PBR0R_DS0_Pos               (7U)
#define RTC_PBR0R_DS0_Msk               (0x1UL << RTC_PBR0R_DS0_Pos)
#define RTC_PBR0R_DS0                   RTC_PBR0R_DS0_Msk
#define RTC_PBR0R_DS1_Pos               (8U)
#define RTC_PBR0R_DS1_Msk               (0x1UL << RTC_PBR0R_DS1_Pos)
#define RTC_PBR0R_DS1                   RTC_PBR0R_DS1_Msk
#define RTC_PBR0R_IN_Pos                (9U)
#define RTC_PBR0R_IN_Msk                (0x1UL << RTC_PBR0R_IN_Pos)
#define RTC_PBR0R_IN                    RTC_PBR0R_IN_Msk
#define RTC_PBR0R_SEL_Pos               (12U)
#define RTC_PBR0R_SEL_Msk               (0x7UL << RTC_PBR0R_SEL_Pos)
#define RTC_PBR0R_SEL                   RTC_PBR0R_SEL_Msk
#define RTC_PBR0R_FORCE1_Pos            (31U)
#define RTC_PBR0R_FORCE1_Msk            (0x1UL << RTC_PBR0R_FORCE1_Pos)
#define RTC_PBR0R_FORCE1                RTC_PBR0R_FORCE1_Msk

/******************* Bit definition for RTC_PBR1R register ********************/
#define RTC_PBR1R_OUT_Pos               (0U)
#define RTC_PBR1R_OUT_Msk               (0x1UL << RTC_PBR1R_OUT_Pos)
#define RTC_PBR1R_OUT                   RTC_PBR1R_OUT_Msk
#define RTC_PBR1R_OE_Pos                (1U)
#define RTC_PBR1R_OE_Msk                (0x1UL << RTC_PBR1R_OE_Pos)
#define RTC_PBR1R_OE                    RTC_PBR1R_OE_Msk
#define RTC_PBR1R_IE_Pos                (2U)
#define RTC_PBR1R_IE_Msk                (0x1UL << RTC_PBR1R_IE_Pos)
#define RTC_PBR1R_IE                    RTC_PBR1R_IE_Msk
#define RTC_PBR1R_PE_Pos                (3U)
#define RTC_PBR1R_PE_Msk                (0x1UL << RTC_PBR1R_PE_Pos)
#define RTC_PBR1R_PE                    RTC_PBR1R_PE_Msk
#define RTC_PBR1R_PS_Pos                (4U)
#define RTC_PBR1R_PS_Msk                (0x1UL << RTC_PBR1R_PS_Pos)
#define RTC_PBR1R_PS                    RTC_PBR1R_PS_Msk
#define RTC_PBR1R_IS_Pos                (5U)
#define RTC_PBR1R_IS_Msk                (0x1UL << RTC_PBR1R_IS_Pos)
#define RTC_PBR1R_IS                    RTC_PBR1R_IS_Msk
#define RTC_PBR1R_SR_Pos                (6U)
#define RTC_PBR1R_SR_Msk                (0x1UL << RTC_PBR1R_SR_Pos)
#define RTC_PBR1R_SR                    RTC_PBR1R_SR_Msk
#define RTC_PBR1R_DS0_Pos               (7U)
#define RTC_PBR1R_DS0_Msk               (0x1UL << RTC_PBR1R_DS0_Pos)
#define RTC_PBR1R_DS0                   RTC_PBR1R_DS0_Msk
#define RTC_PBR1R_DS1_Pos               (8U)
#define RTC_PBR1R_DS1_Msk               (0x1UL << RTC_PBR1R_DS1_Pos)
#define RTC_PBR1R_DS1                   RTC_PBR1R_DS1_Msk
#define RTC_PBR1R_IN_Pos                (9U)
#define RTC_PBR1R_IN_Msk                (0x1UL << RTC_PBR1R_IN_Pos)
#define RTC_PBR1R_IN                    RTC_PBR1R_IN_Msk
#define RTC_PBR1R_SEL_Pos               (12U)
#define RTC_PBR1R_SEL_Msk               (0x7UL << RTC_PBR1R_SEL_Pos)
#define RTC_PBR1R_SEL                   RTC_PBR1R_SEL_Msk

/******************* Bit definition for RTC_PBR2R register ********************/
#define RTC_PBR2R_OUT_Pos               (0U)
#define RTC_PBR2R_OUT_Msk               (0x1UL << RTC_PBR2R_OUT_Pos)
#define RTC_PBR2R_OUT                   RTC_PBR2R_OUT_Msk
#define RTC_PBR2R_OE_Pos                (1U)
#define RTC_PBR2R_OE_Msk                (0x1UL << RTC_PBR2R_OE_Pos)
#define RTC_PBR2R_OE                    RTC_PBR2R_OE_Msk
#define RTC_PBR2R_IE_Pos                (2U)
#define RTC_PBR2R_IE_Msk                (0x1UL << RTC_PBR2R_IE_Pos)
#define RTC_PBR2R_IE                    RTC_PBR2R_IE_Msk
#define RTC_PBR2R_PE_Pos                (3U)
#define RTC_PBR2R_PE_Msk                (0x1UL << RTC_PBR2R_PE_Pos)
#define RTC_PBR2R_PE                    RTC_PBR2R_PE_Msk
#define RTC_PBR2R_PS_Pos                (4U)
#define RTC_PBR2R_PS_Msk                (0x1UL << RTC_PBR2R_PS_Pos)
#define RTC_PBR2R_PS                    RTC_PBR2R_PS_Msk
#define RTC_PBR2R_IS_Pos                (5U)
#define RTC_PBR2R_IS_Msk                (0x1UL << RTC_PBR2R_IS_Pos)
#define RTC_PBR2R_IS                    RTC_PBR2R_IS_Msk
#define RTC_PBR2R_SR_Pos                (6U)
#define RTC_PBR2R_SR_Msk                (0x1UL << RTC_PBR2R_SR_Pos)
#define RTC_PBR2R_SR                    RTC_PBR2R_SR_Msk
#define RTC_PBR2R_DS0_Pos               (7U)
#define RTC_PBR2R_DS0_Msk               (0x1UL << RTC_PBR2R_DS0_Pos)
#define RTC_PBR2R_DS0                   RTC_PBR2R_DS0_Msk
#define RTC_PBR2R_DS1_Pos               (8U)
#define RTC_PBR2R_DS1_Msk               (0x1UL << RTC_PBR2R_DS1_Pos)
#define RTC_PBR2R_DS1                   RTC_PBR2R_DS1_Msk
#define RTC_PBR2R_IN_Pos                (9U)
#define RTC_PBR2R_IN_Msk                (0x1UL << RTC_PBR2R_IN_Pos)
#define RTC_PBR2R_IN                    RTC_PBR2R_IN_Msk
#define RTC_PBR2R_SEL_Pos               (12U)
#define RTC_PBR2R_SEL_Msk               (0x7UL << RTC_PBR2R_SEL_Pos)
#define RTC_PBR2R_SEL                   RTC_PBR2R_SEL_Msk

/******************* Bit definition for RTC_PBR3R register ********************/
#define RTC_PBR3R_OUT_Pos               (0U)
#define RTC_PBR3R_OUT_Msk               (0x1UL << RTC_PBR3R_OUT_Pos)
#define RTC_PBR3R_OUT                   RTC_PBR3R_OUT_Msk
#define RTC_PBR3R_OE_Pos                (1U)
#define RTC_PBR3R_OE_Msk                (0x1UL << RTC_PBR3R_OE_Pos)
#define RTC_PBR3R_OE                    RTC_PBR3R_OE_Msk
#define RTC_PBR3R_IE_Pos                (2U)
#define RTC_PBR3R_IE_Msk                (0x1UL << RTC_PBR3R_IE_Pos)
#define RTC_PBR3R_IE                    RTC_PBR3R_IE_Msk
#define RTC_PBR3R_PE_Pos                (3U)
#define RTC_PBR3R_PE_Msk                (0x1UL << RTC_PBR3R_PE_Pos)
#define RTC_PBR3R_PE                    RTC_PBR3R_PE_Msk
#define RTC_PBR3R_PS_Pos                (4U)
#define RTC_PBR3R_PS_Msk                (0x1UL << RTC_PBR3R_PS_Pos)
#define RTC_PBR3R_PS                    RTC_PBR3R_PS_Msk
#define RTC_PBR3R_IS_Pos                (5U)
#define RTC_PBR3R_IS_Msk                (0x1UL << RTC_PBR3R_IS_Pos)
#define RTC_PBR3R_IS                    RTC_PBR3R_IS_Msk
#define RTC_PBR3R_SR_Pos                (6U)
#define RTC_PBR3R_SR_Msk                (0x1UL << RTC_PBR3R_SR_Pos)
#define RTC_PBR3R_SR                    RTC_PBR3R_SR_Msk
#define RTC_PBR3R_DS0_Pos               (7U)
#define RTC_PBR3R_DS0_Msk               (0x1UL << RTC_PBR3R_DS0_Pos)
#define RTC_PBR3R_DS0                   RTC_PBR3R_DS0_Msk
#define RTC_PBR3R_DS1_Pos               (8U)
#define RTC_PBR3R_DS1_Msk               (0x1UL << RTC_PBR3R_DS1_Pos)
#define RTC_PBR3R_DS1                   RTC_PBR3R_DS1_Msk
#define RTC_PBR3R_IN_Pos                (9U)
#define RTC_PBR3R_IN_Msk                (0x1UL << RTC_PBR3R_IN_Pos)
#define RTC_PBR3R_IN                    RTC_PBR3R_IN_Msk
#define RTC_PBR3R_SEL_Pos               (12U)
#define RTC_PBR3R_SEL_Msk               (0x7UL << RTC_PBR3R_SEL_Pos)
#define RTC_PBR3R_SEL                   RTC_PBR3R_SEL_Msk

/******************* Bit definition for RTC_PAWK1R register *******************/
#define RTC_PAWK1R_PE_Pos               (0U)
#define RTC_PAWK1R_PE_Msk               (0x1FFFFUL << RTC_PAWK1R_PE_Pos)
#define RTC_PAWK1R_PE                   RTC_PAWK1R_PE_Msk

/******************* Bit definition for RTC_PAWK2R register *******************/
#define RTC_PAWK2R_PS_Pos               (0U)
#define RTC_PAWK2R_PS_Msk               (0x1FFFFUL << RTC_PAWK2R_PS_Pos)
#define RTC_PAWK2R_PS                   RTC_PAWK2R_PS_Msk

/******************* Bit definition for RTC_PAWK3R register *******************/
#define RTC_PAWK3R_IS_Pos               (0U)
#define RTC_PAWK3R_IS_Msk               (0x1FFFFUL << RTC_PAWK3R_IS_Pos)
#define RTC_PAWK3R_IS                   RTC_PAWK3R_IS_Msk

#endif
