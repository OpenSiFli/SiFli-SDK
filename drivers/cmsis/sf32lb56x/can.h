#ifndef __CAN_H
#define __CAN_H

typedef struct
{
    __IO uint32_t RBUF;
    __IO uint32_t RSVD2[17];
    __IO uint32_t TBUF;
    __IO uint32_t RSVD1[17];
    __IO uint32_t CR;
    __IO uint32_t IR;
    __IO uint32_t BITTIME;
    __IO uint32_t PRESC;
    __IO uint32_t CNT;
    __IO uint32_t ACFCTRL;
    __IO uint32_t ACF;
    __IO uint32_t VER;
    __IO uint32_t CR2;
} CAN_TypeDef;


/******************** Bit definition for CAN_RBUF register ********************/
#define CAN_RBUF_DATA_Pos               (0U)
#define CAN_RBUF_DATA_Msk               (0xFFFFFFFFUL << CAN_RBUF_DATA_Pos)
#define CAN_RBUF_DATA                   CAN_RBUF_DATA_Msk

/******************** Bit definition for CAN_TBUF register ********************/
#define CAN_TBUF_DATA_Pos               (0U)
#define CAN_TBUF_DATA_Msk               (0xFFFFFFFFUL << CAN_TBUF_DATA_Pos)
#define CAN_TBUF_DATA                   CAN_TBUF_DATA_Msk

/********************* Bit definition for CAN_CR register *********************/
#define CAN_CR_BUSOFF_Pos               (0U)
#define CAN_CR_BUSOFF_Msk               (0x1UL << CAN_CR_BUSOFF_Pos)
#define CAN_CR_BUSOFF                   CAN_CR_BUSOFF_Msk
#define CAN_CR_TACTIVE_Pos              (1U)
#define CAN_CR_TACTIVE_Msk              (0x1UL << CAN_CR_TACTIVE_Pos)
#define CAN_CR_TACTIVE                  CAN_CR_TACTIVE_Msk
#define CAN_CR_RACTIVE_Pos              (2U)
#define CAN_CR_RACTIVE_Msk              (0x1UL << CAN_CR_RACTIVE_Pos)
#define CAN_CR_RACTIVE                  CAN_CR_RACTIVE_Msk
#define CAN_CR_TSSS_Pos                 (3U)
#define CAN_CR_TSSS_Msk                 (0x1UL << CAN_CR_TSSS_Pos)
#define CAN_CR_TSSS                     CAN_CR_TSSS_Msk
#define CAN_CR_TPSS_Pos                 (4U)
#define CAN_CR_TPSS_Msk                 (0x1UL << CAN_CR_TPSS_Pos)
#define CAN_CR_TPSS                     CAN_CR_TPSS_Msk
#define CAN_CR_LBMI_Pos                 (5U)
#define CAN_CR_LBMI_Msk                 (0x1UL << CAN_CR_LBMI_Pos)
#define CAN_CR_LBMI                     CAN_CR_LBMI_Msk
#define CAN_CR_LBME_Pos                 (6U)
#define CAN_CR_LBME_Msk                 (0x1UL << CAN_CR_LBME_Pos)
#define CAN_CR_LBME                     CAN_CR_LBME_Msk
#define CAN_CR_RESET_Pos                (7U)
#define CAN_CR_RESET_Msk                (0x1UL << CAN_CR_RESET_Pos)
#define CAN_CR_RESET                    CAN_CR_RESET_Msk
#define CAN_CR_TSA_Pos                  (8U)
#define CAN_CR_TSA_Msk                  (0x1UL << CAN_CR_TSA_Pos)
#define CAN_CR_TSA                      CAN_CR_TSA_Msk
#define CAN_CR_TSALL_Pos                (9U)
#define CAN_CR_TSALL_Msk                (0x1UL << CAN_CR_TSALL_Pos)
#define CAN_CR_TSALL                    CAN_CR_TSALL_Msk
#define CAN_CR_TSONE_Pos                (10U)
#define CAN_CR_TSONE_Msk                (0x1UL << CAN_CR_TSONE_Pos)
#define CAN_CR_TSONE                    CAN_CR_TSONE_Msk
#define CAN_CR_TPA_Pos                  (11U)
#define CAN_CR_TPA_Msk                  (0x1UL << CAN_CR_TPA_Pos)
#define CAN_CR_TPA                      CAN_CR_TPA_Msk
#define CAN_CR_TPE_Pos                  (12U)
#define CAN_CR_TPE_Msk                  (0x1UL << CAN_CR_TPE_Pos)
#define CAN_CR_TPE                      CAN_CR_TPE_Msk
#define CAN_CR_STBY_Pos                 (13U)
#define CAN_CR_STBY_Msk                 (0x1UL << CAN_CR_STBY_Pos)
#define CAN_CR_STBY                     CAN_CR_STBY_Msk
#define CAN_CR_LOM_Pos                  (14U)
#define CAN_CR_LOM_Msk                  (0x1UL << CAN_CR_LOM_Pos)
#define CAN_CR_LOM                      CAN_CR_LOM_Msk
#define CAN_CR_TBSEL_Pos                (15U)
#define CAN_CR_TBSEL_Msk                (0x1UL << CAN_CR_TBSEL_Pos)
#define CAN_CR_TBSEL                    CAN_CR_TBSEL_Msk
#define CAN_CR_TSSTAT_Pos               (16U)
#define CAN_CR_TSSTAT_Msk               (0x1FUL << CAN_CR_TSSTAT_Pos)
#define CAN_CR_TSSTAT                   CAN_CR_TSSTAT_Msk
#define CAN_CR_TSNEXT_Pos               (22U)
#define CAN_CR_TSNEXT_Msk               (0x1UL << CAN_CR_TSNEXT_Pos)
#define CAN_CR_TSNEXT                   CAN_CR_TSNEXT_Msk
#define CAN_CR_FD_ISO_Pos               (23U)
#define CAN_CR_FD_ISO_Msk               (0x1UL << CAN_CR_FD_ISO_Pos)
#define CAN_CR_FD_ISO                   CAN_CR_FD_ISO_Msk
#define CAN_CR_RSTAT_Pos                (24U)
#define CAN_CR_RSTAT_Msk                (0x3UL << CAN_CR_RSTAT_Pos)
#define CAN_CR_RSTAT                    CAN_CR_RSTAT_Msk
#define CAN_CR_RREL_Pos                 (28U)
#define CAN_CR_RREL_Msk                 (0x1UL << CAN_CR_RREL_Pos)
#define CAN_CR_RREL                     CAN_CR_RREL_Msk
#define CAN_CR_ROV_Pos                  (29U)
#define CAN_CR_ROV_Msk                  (0x1UL << CAN_CR_ROV_Pos)
#define CAN_CR_ROV                      CAN_CR_ROV_Msk

/********************* Bit definition for CAN_IR register *********************/
#define CAN_IR_TSFF_Pos                 (0U)
#define CAN_IR_TSFF_Msk                 (0x1UL << CAN_IR_TSFF_Pos)
#define CAN_IR_TSFF                     CAN_IR_TSFF_Msk
#define CAN_IR_EIE_Pos                  (1U)
#define CAN_IR_EIE_Msk                  (0x1UL << CAN_IR_EIE_Pos)
#define CAN_IR_EIE                      CAN_IR_EIE_Msk
#define CAN_IR_TSIE_Pos                 (2U)
#define CAN_IR_TSIE_Msk                 (0x1UL << CAN_IR_TSIE_Pos)
#define CAN_IR_TSIE                     CAN_IR_TSIE_Msk
#define CAN_IR_TPIE_Pos                 (3U)
#define CAN_IR_TPIE_Msk                 (0x1UL << CAN_IR_TPIE_Pos)
#define CAN_IR_TPIE                     CAN_IR_TPIE_Msk
#define CAN_IR_RAFIE_Pos                (4U)
#define CAN_IR_RAFIE_Msk                (0x1UL << CAN_IR_RAFIE_Pos)
#define CAN_IR_RAFIE                    CAN_IR_RAFIE_Msk
#define CAN_IR_RFIE_Pos                 (5U)
#define CAN_IR_RFIE_Msk                 (0x1UL << CAN_IR_RFIE_Pos)
#define CAN_IR_RFIE                     CAN_IR_RFIE_Msk
#define CAN_IR_ROIE_Pos                 (6U)
#define CAN_IR_ROIE_Msk                 (0x1UL << CAN_IR_ROIE_Pos)
#define CAN_IR_ROIE                     CAN_IR_ROIE_Msk
#define CAN_IR_RIE_Pos                  (7U)
#define CAN_IR_RIE_Msk                  (0x1UL << CAN_IR_RIE_Pos)
#define CAN_IR_RIE                      CAN_IR_RIE_Msk
#define CAN_IR_AIF_Pos                  (8U)
#define CAN_IR_AIF_Msk                  (0x1UL << CAN_IR_AIF_Pos)
#define CAN_IR_AIF                      CAN_IR_AIF_Msk
#define CAN_IR_EIF_Pos                  (9U)
#define CAN_IR_EIF_Msk                  (0x1UL << CAN_IR_EIF_Pos)
#define CAN_IR_EIF                      CAN_IR_EIF_Msk
#define CAN_IR_TSIF_Pos                 (10U)
#define CAN_IR_TSIF_Msk                 (0x1UL << CAN_IR_TSIF_Pos)
#define CAN_IR_TSIF                     CAN_IR_TSIF_Msk
#define CAN_IR_TPIF_Pos                 (11U)
#define CAN_IR_TPIF_Msk                 (0x1UL << CAN_IR_TPIF_Pos)
#define CAN_IR_TPIF                     CAN_IR_TPIF_Msk
#define CAN_IR_RAFIF_Pos                (12U)
#define CAN_IR_RAFIF_Msk                (0x1UL << CAN_IR_RAFIF_Pos)
#define CAN_IR_RAFIF                    CAN_IR_RAFIF_Msk
#define CAN_IR_RFIF_Pos                 (13U)
#define CAN_IR_RFIF_Msk                 (0x1UL << CAN_IR_RFIF_Pos)
#define CAN_IR_RFIF                     CAN_IR_RFIF_Msk
#define CAN_IR_ROIF_Pos                 (14U)
#define CAN_IR_ROIF_Msk                 (0x1UL << CAN_IR_ROIF_Pos)
#define CAN_IR_ROIF                     CAN_IR_ROIF_Msk
#define CAN_IR_RIF_Pos                  (15U)
#define CAN_IR_RIF_Msk                  (0x1UL << CAN_IR_RIF_Pos)
#define CAN_IR_RIF                      CAN_IR_RIF_Msk
#define CAN_IR_BEIF_Pos                 (16U)
#define CAN_IR_BEIF_Msk                 (0x1UL << CAN_IR_BEIF_Pos)
#define CAN_IR_BEIF                     CAN_IR_BEIF_Msk
#define CAN_IR_BEIE_Pos                 (17U)
#define CAN_IR_BEIE_Msk                 (0x1UL << CAN_IR_BEIE_Pos)
#define CAN_IR_BEIE                     CAN_IR_BEIE_Msk
#define CAN_IR_ALIF_Pos                 (18U)
#define CAN_IR_ALIF_Msk                 (0x1UL << CAN_IR_ALIF_Pos)
#define CAN_IR_ALIF                     CAN_IR_ALIF_Msk
#define CAN_IR_ALIE_Pos                 (19U)
#define CAN_IR_ALIE_Msk                 (0x1UL << CAN_IR_ALIE_Pos)
#define CAN_IR_ALIE                     CAN_IR_ALIE_Msk
#define CAN_IR_EPIF_Pos                 (20U)
#define CAN_IR_EPIF_Msk                 (0x1UL << CAN_IR_EPIF_Pos)
#define CAN_IR_EPIF                     CAN_IR_EPIF_Msk
#define CAN_IR_EPIE_Pos                 (21U)
#define CAN_IR_EPIE_Msk                 (0x1UL << CAN_IR_EPIE_Pos)
#define CAN_IR_EPIE                     CAN_IR_EPIE_Msk
#define CAN_IR_EPASS_Pos                (22U)
#define CAN_IR_EPASS_Msk                (0x1UL << CAN_IR_EPASS_Pos)
#define CAN_IR_EPASS                    CAN_IR_EPASS_Msk
#define CAN_IR_EWARN_Pos                (23U)
#define CAN_IR_EWARN_Msk                (0x1UL << CAN_IR_EWARN_Pos)
#define CAN_IR_EWARN                    CAN_IR_EWARN_Msk
#define CAN_IR_EWL_Pos                  (24U)
#define CAN_IR_EWL_Msk                  (0xFUL << CAN_IR_EWL_Pos)
#define CAN_IR_EWL                      CAN_IR_EWL_Msk
#define CAN_IR_AFWL_Pos                 (28U)
#define CAN_IR_AFWL_Msk                 (0xFUL << CAN_IR_AFWL_Pos)
#define CAN_IR_AFWL                     CAN_IR_AFWL_Msk

/****************** Bit definition for CAN_BITTIME register *******************/
#define CAN_BITTIME_S_SEG_1_Pos         (0U)
#define CAN_BITTIME_S_SEG_1_Msk         (0x3FUL << CAN_BITTIME_S_SEG_1_Pos)
#define CAN_BITTIME_S_SEG_1             CAN_BITTIME_S_SEG_1_Msk
#define CAN_BITTIME_F_SJW_Pos           (6U)
#define CAN_BITTIME_F_SJW_Msk           (0x3UL << CAN_BITTIME_F_SJW_Pos)
#define CAN_BITTIME_F_SJW               CAN_BITTIME_F_SJW_Msk
#define CAN_BITTIME_S_SEG_2_Pos         (8U)
#define CAN_BITTIME_S_SEG_2_Msk         (0x1FUL << CAN_BITTIME_S_SEG_2_Pos)
#define CAN_BITTIME_S_SEG_2             CAN_BITTIME_S_SEG_2_Msk
#define CAN_BITTIME_F_SEG_2_Pos         (13U)
#define CAN_BITTIME_F_SEG_2_Msk         (0x7UL << CAN_BITTIME_F_SEG_2_Pos)
#define CAN_BITTIME_F_SEG_2             CAN_BITTIME_F_SEG_2_Msk
#define CAN_BITTIME_S_SJW_Pos           (16U)
#define CAN_BITTIME_S_SJW_Msk           (0xFUL << CAN_BITTIME_S_SJW_Pos)
#define CAN_BITTIME_S_SJW               CAN_BITTIME_S_SJW_Msk
#define CAN_BITTIME_F_SEG_1_Pos         (20U)
#define CAN_BITTIME_F_SEG_1_Msk         (0xFUL << CAN_BITTIME_F_SEG_1_Pos)
#define CAN_BITTIME_F_SEG_1             CAN_BITTIME_F_SEG_1_Msk

/******************* Bit definition for CAN_PRESC register ********************/
#define CAN_PRESC_S_PRESC_Pos           (0U)
#define CAN_PRESC_S_PRESC_Msk           (0xFFUL << CAN_PRESC_S_PRESC_Pos)
#define CAN_PRESC_S_PRESC               CAN_PRESC_S_PRESC_Msk
#define CAN_PRESC_F_PRESC_Pos           (8U)
#define CAN_PRESC_F_PRESC_Msk           (0xFFUL << CAN_PRESC_F_PRESC_Pos)
#define CAN_PRESC_F_PRESC               CAN_PRESC_F_PRESC_Msk
#define CAN_PRESC_SSPOFF_Pos            (16U)
#define CAN_PRESC_SSPOFF_Msk            (0x1FUL << CAN_PRESC_SSPOFF_Pos)
#define CAN_PRESC_SSPOFF                CAN_PRESC_SSPOFF_Msk
#define CAN_PRESC_TDCEN_Pos             (23U)
#define CAN_PRESC_TDCEN_Msk             (0x1UL << CAN_PRESC_TDCEN_Pos)
#define CAN_PRESC_TDCEN                 CAN_PRESC_TDCEN_Msk

/******************** Bit definition for CAN_CNT register *********************/
#define CAN_CNT_ALC_Pos                 (0U)
#define CAN_CNT_ALC_Msk                 (0x1FUL << CAN_CNT_ALC_Pos)
#define CAN_CNT_ALC                     CAN_CNT_ALC_Msk
#define CAN_CNT_KOER_Pos                (5U)
#define CAN_CNT_KOER_Msk                (0x7UL << CAN_CNT_KOER_Pos)
#define CAN_CNT_KOER                    CAN_CNT_KOER_Msk
#define CAN_CNT_RECNT_Pos               (16U)
#define CAN_CNT_RECNT_Msk               (0xFFUL << CAN_CNT_RECNT_Pos)
#define CAN_CNT_RECNT                   CAN_CNT_RECNT_Msk
#define CAN_CNT_TECNT_Pos               (24U)
#define CAN_CNT_TECNT_Msk               (0xFFUL << CAN_CNT_TECNT_Pos)
#define CAN_CNT_TECNT                   CAN_CNT_TECNT_Msk

/****************** Bit definition for CAN_ACFCTRL register *******************/
#define CAN_ACFCTRL_ACFADR_Pos          (0U)
#define CAN_ACFCTRL_ACFADR_Msk          (0xFUL << CAN_ACFCTRL_ACFADR_Pos)
#define CAN_ACFCTRL_ACFADR              CAN_ACFCTRL_ACFADR_Msk
#define CAN_ACFCTRL_SELMASK_Pos         (5U)
#define CAN_ACFCTRL_SELMASK_Msk         (0x1UL << CAN_ACFCTRL_SELMASK_Pos)
#define CAN_ACFCTRL_SELMASK             CAN_ACFCTRL_SELMASK_Msk
#define CAN_ACFCTRL_AE_0_Pos            (16U)
#define CAN_ACFCTRL_AE_0_Msk            (0x1UL << CAN_ACFCTRL_AE_0_Pos)
#define CAN_ACFCTRL_AE_0                CAN_ACFCTRL_AE_0_Msk
#define CAN_ACFCTRL_AE_1_Pos            (17U)
#define CAN_ACFCTRL_AE_1_Msk            (0x1UL << CAN_ACFCTRL_AE_1_Pos)
#define CAN_ACFCTRL_AE_1                CAN_ACFCTRL_AE_1_Msk
#define CAN_ACFCTRL_AE_2_Pos            (18U)
#define CAN_ACFCTRL_AE_2_Msk            (0x1UL << CAN_ACFCTRL_AE_2_Pos)
#define CAN_ACFCTRL_AE_2                CAN_ACFCTRL_AE_2_Msk
#define CAN_ACFCTRL_AE_3_Pos            (19U)
#define CAN_ACFCTRL_AE_3_Msk            (0x1UL << CAN_ACFCTRL_AE_3_Pos)
#define CAN_ACFCTRL_AE_3                CAN_ACFCTRL_AE_3_Msk
#define CAN_ACFCTRL_AE_4_Pos            (20U)
#define CAN_ACFCTRL_AE_4_Msk            (0x1UL << CAN_ACFCTRL_AE_4_Pos)
#define CAN_ACFCTRL_AE_4                CAN_ACFCTRL_AE_4_Msk
#define CAN_ACFCTRL_AE_5_Pos            (21U)
#define CAN_ACFCTRL_AE_5_Msk            (0x1UL << CAN_ACFCTRL_AE_5_Pos)
#define CAN_ACFCTRL_AE_5                CAN_ACFCTRL_AE_5_Msk
#define CAN_ACFCTRL_AE_6_Pos            (22U)
#define CAN_ACFCTRL_AE_6_Msk            (0x1UL << CAN_ACFCTRL_AE_6_Pos)
#define CAN_ACFCTRL_AE_6                CAN_ACFCTRL_AE_6_Msk
#define CAN_ACFCTRL_AE_7_Pos            (23U)
#define CAN_ACFCTRL_AE_7_Msk            (0x1UL << CAN_ACFCTRL_AE_7_Pos)
#define CAN_ACFCTRL_AE_7                CAN_ACFCTRL_AE_7_Msk
#define CAN_ACFCTRL_AE_8_Pos            (24U)
#define CAN_ACFCTRL_AE_8_Msk            (0x1UL << CAN_ACFCTRL_AE_8_Pos)
#define CAN_ACFCTRL_AE_8                CAN_ACFCTRL_AE_8_Msk
#define CAN_ACFCTRL_AE_9_Pos            (25U)
#define CAN_ACFCTRL_AE_9_Msk            (0x1UL << CAN_ACFCTRL_AE_9_Pos)
#define CAN_ACFCTRL_AE_9                CAN_ACFCTRL_AE_9_Msk
#define CAN_ACFCTRL_AE_10_Pos           (26U)
#define CAN_ACFCTRL_AE_10_Msk           (0x1UL << CAN_ACFCTRL_AE_10_Pos)
#define CAN_ACFCTRL_AE_10               CAN_ACFCTRL_AE_10_Msk
#define CAN_ACFCTRL_AE_11_Pos           (27U)
#define CAN_ACFCTRL_AE_11_Msk           (0x1UL << CAN_ACFCTRL_AE_11_Pos)
#define CAN_ACFCTRL_AE_11               CAN_ACFCTRL_AE_11_Msk
#define CAN_ACFCTRL_AE_12_Pos           (28U)
#define CAN_ACFCTRL_AE_12_Msk           (0x1UL << CAN_ACFCTRL_AE_12_Pos)
#define CAN_ACFCTRL_AE_12               CAN_ACFCTRL_AE_12_Msk
#define CAN_ACFCTRL_AE_13_Pos           (29U)
#define CAN_ACFCTRL_AE_13_Msk           (0x1UL << CAN_ACFCTRL_AE_13_Pos)
#define CAN_ACFCTRL_AE_13               CAN_ACFCTRL_AE_13_Msk
#define CAN_ACFCTRL_AE_14_Pos           (30U)
#define CAN_ACFCTRL_AE_14_Msk           (0x1UL << CAN_ACFCTRL_AE_14_Pos)
#define CAN_ACFCTRL_AE_14               CAN_ACFCTRL_AE_14_Msk
#define CAN_ACFCTRL_AE_15_Pos           (31U)
#define CAN_ACFCTRL_AE_15_Msk           (0x1UL << CAN_ACFCTRL_AE_15_Pos)
#define CAN_ACFCTRL_AE_15               CAN_ACFCTRL_AE_15_Msk

/******************** Bit definition for CAN_ACF register *********************/
#define CAN_ACF_ACODE_Pos               (0U)
#define CAN_ACF_ACODE_Msk               (0x1FFFFFFFUL << CAN_ACF_ACODE_Pos)
#define CAN_ACF_ACODE                   CAN_ACF_ACODE_Msk
#define CAN_ACF_AIDE_Pos                (29U)
#define CAN_ACF_AIDE_Msk                (0x1UL << CAN_ACF_AIDE_Pos)
#define CAN_ACF_AIDE                    CAN_ACF_AIDE_Msk
#define CAN_ACF_AIDEE_Pos               (30U)
#define CAN_ACF_AIDEE_Msk               (0x1UL << CAN_ACF_AIDEE_Pos)
#define CAN_ACF_AIDEE                   CAN_ACF_AIDEE_Msk

/******************** Bit definition for CAN_VER register *********************/
#define CAN_VER_VERSION_Pos             (0U)
#define CAN_VER_VERSION_Msk             (0xFFFFUL << CAN_VER_VERSION_Pos)
#define CAN_VER_VERSION                 CAN_VER_VERSION_Msk

/******************** Bit definition for CAN_CR2 register *********************/
#define CAN_CR2_MEMMASK_Pos             (0U)
#define CAN_CR2_MEMMASK_Msk             (0x1UL << CAN_CR2_MEMMASK_Pos)
#define CAN_CR2_MEMMASK                 CAN_CR2_MEMMASK_Msk
#define CAN_CR2_FD_EN_Pos               (1U)
#define CAN_CR2_FD_EN_Msk               (0x1UL << CAN_CR2_FD_EN_Pos)
#define CAN_CR2_FD_EN                   CAN_CR2_FD_EN_Msk

#endif
