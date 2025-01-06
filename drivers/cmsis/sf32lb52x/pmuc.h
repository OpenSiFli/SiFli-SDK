#ifndef __PMUC_H
#define __PMUC_H

typedef struct
{
    __IO uint32_t CR;
    __IO uint32_t WER;
    __IO uint32_t WSR;
    __IO uint32_t WCR;
    __IO uint32_t VRTC_CR;
    __IO uint32_t VRET_CR;
    __IO uint32_t LRC10_CR;
    __IO uint32_t LRC32_CR;
    __IO uint32_t LXT_CR;
    __IO uint32_t AON_BG;
    __IO uint32_t AON_LDO;
    __IO uint32_t BUCK_CR1;
    __IO uint32_t BUCK_CR2;
    __IO uint32_t CHG_CR1;
    __IO uint32_t CHG_CR2;
    __IO uint32_t CHG_CR3;
    __IO uint32_t CHG_CR4;
    __IO uint32_t CHG_CR5;
    __IO uint32_t CHG_SR;
    __IO uint32_t HPSYS_LDO;
    __IO uint32_t LPSYS_LDO;
    __IO uint32_t HPSYS_SWR;
    __IO uint32_t LPSYS_SWR;
    __IO uint32_t PERI_LDO;
    __IO uint32_t PMU_TR;
    __IO uint32_t PMU_RSVD;
    __IO uint32_t HXT_CR1;
    __IO uint32_t HXT_CR2;
    __IO uint32_t HXT_CR3;
    __IO uint32_t HRC_CR;
    __IO uint32_t DBL96_CR;
    __IO uint32_t DBL96_CALR;
    __IO uint32_t CAU_BGR;
    __IO uint32_t CAU_TR;
    __IO uint32_t CAU_RSVD;
    __IO uint32_t WKUP_CNT;
    __IO uint32_t PWRKEY_CNT;
    __IO uint32_t HPSYS_VOUT;
    __IO uint32_t LPSYS_VOUT;
    __IO uint32_t BUCK_VOUT;
} PMUC_TypeDef;


/******************** Bit definition for PMUC_CR register *********************/
#define PMUC_CR_SEL_LPCLK_Pos           (0U)
#define PMUC_CR_SEL_LPCLK_Msk           (0x1UL << PMUC_CR_SEL_LPCLK_Pos)
#define PMUC_CR_SEL_LPCLK               PMUC_CR_SEL_LPCLK_Msk
#define PMUC_CR_HIBER_EN_Pos            (1U)
#define PMUC_CR_HIBER_EN_Msk            (0x1UL << PMUC_CR_HIBER_EN_Pos)
#define PMUC_CR_HIBER_EN                PMUC_CR_HIBER_EN_Msk
#define PMUC_CR_REBOOT_Pos              (2U)
#define PMUC_CR_REBOOT_Msk              (0x1UL << PMUC_CR_REBOOT_Pos)
#define PMUC_CR_REBOOT                  PMUC_CR_REBOOT_Msk
#define PMUC_CR_PIN_RET_Pos             (3U)
#define PMUC_CR_PIN_RET_Msk             (0x1UL << PMUC_CR_PIN_RET_Pos)
#define PMUC_CR_PIN_RET                 PMUC_CR_PIN_RET_Msk
#define PMUC_CR_PIN0_MODE_Pos           (4U)
#define PMUC_CR_PIN0_MODE_Msk           (0x7UL << PMUC_CR_PIN0_MODE_Pos)
#define PMUC_CR_PIN0_MODE               PMUC_CR_PIN0_MODE_Msk
#define PMUC_CR_PIN1_MODE_Pos           (7U)
#define PMUC_CR_PIN1_MODE_Msk           (0x7UL << PMUC_CR_PIN1_MODE_Pos)
#define PMUC_CR_PIN1_MODE               PMUC_CR_PIN1_MODE_Msk
#define PMUC_CR_PIN0_SEL_Pos            (10U)
#define PMUC_CR_PIN0_SEL_Msk            (0x1FUL << PMUC_CR_PIN0_SEL_Pos)
#define PMUC_CR_PIN0_SEL                PMUC_CR_PIN0_SEL_Msk
#define PMUC_CR_PIN1_SEL_Pos            (15U)
#define PMUC_CR_PIN1_SEL_Msk            (0x1FUL << PMUC_CR_PIN1_SEL_Pos)
#define PMUC_CR_PIN1_SEL                PMUC_CR_PIN1_SEL_Msk

/******************** Bit definition for PMUC_WER register ********************/
#define PMUC_WER_RTC_Pos                (0U)
#define PMUC_WER_RTC_Msk                (0x1UL << PMUC_WER_RTC_Pos)
#define PMUC_WER_RTC                    PMUC_WER_RTC_Msk
#define PMUC_WER_WDT1_Pos               (1U)
#define PMUC_WER_WDT1_Msk               (0x1UL << PMUC_WER_WDT1_Pos)
#define PMUC_WER_WDT1                   PMUC_WER_WDT1_Msk
#define PMUC_WER_WDT2_Pos               (2U)
#define PMUC_WER_WDT2_Msk               (0x1UL << PMUC_WER_WDT2_Pos)
#define PMUC_WER_WDT2                   PMUC_WER_WDT2_Msk
#define PMUC_WER_PIN0_Pos               (3U)
#define PMUC_WER_PIN0_Msk               (0x1UL << PMUC_WER_PIN0_Pos)
#define PMUC_WER_PIN0                   PMUC_WER_PIN0_Msk
#define PMUC_WER_PIN1_Pos               (4U)
#define PMUC_WER_PIN1_Msk               (0x1UL << PMUC_WER_PIN1_Pos)
#define PMUC_WER_PIN1                   PMUC_WER_PIN1_Msk
#define PMUC_WER_LOWBAT_Pos             (7U)
#define PMUC_WER_LOWBAT_Msk             (0x1UL << PMUC_WER_LOWBAT_Pos)
#define PMUC_WER_LOWBAT                 PMUC_WER_LOWBAT_Msk
#define PMUC_WER_CHG_Pos                (8U)
#define PMUC_WER_CHG_Msk                (0x1UL << PMUC_WER_CHG_Pos)
#define PMUC_WER_CHG                    PMUC_WER_CHG_Msk

/******************** Bit definition for PMUC_WSR register ********************/
#define PMUC_WSR_RTC_Pos                (0U)
#define PMUC_WSR_RTC_Msk                (0x1UL << PMUC_WSR_RTC_Pos)
#define PMUC_WSR_RTC                    PMUC_WSR_RTC_Msk
#define PMUC_WSR_WDT1_Pos               (1U)
#define PMUC_WSR_WDT1_Msk               (0x1UL << PMUC_WSR_WDT1_Pos)
#define PMUC_WSR_WDT1                   PMUC_WSR_WDT1_Msk
#define PMUC_WSR_WDT2_Pos               (2U)
#define PMUC_WSR_WDT2_Msk               (0x1UL << PMUC_WSR_WDT2_Pos)
#define PMUC_WSR_WDT2                   PMUC_WSR_WDT2_Msk
#define PMUC_WSR_PIN0_Pos               (3U)
#define PMUC_WSR_PIN0_Msk               (0x1UL << PMUC_WSR_PIN0_Pos)
#define PMUC_WSR_PIN0                   PMUC_WSR_PIN0_Msk
#define PMUC_WSR_PIN1_Pos               (4U)
#define PMUC_WSR_PIN1_Msk               (0x1UL << PMUC_WSR_PIN1_Pos)
#define PMUC_WSR_PIN1                   PMUC_WSR_PIN1_Msk
#define PMUC_WSR_IWDT_Pos               (5U)
#define PMUC_WSR_IWDT_Msk               (0x1UL << PMUC_WSR_IWDT_Pos)
#define PMUC_WSR_IWDT                   PMUC_WSR_IWDT_Msk
#define PMUC_WSR_PWRKEY_Pos             (6U)
#define PMUC_WSR_PWRKEY_Msk             (0x1UL << PMUC_WSR_PWRKEY_Pos)
#define PMUC_WSR_PWRKEY                 PMUC_WSR_PWRKEY_Msk
#define PMUC_WSR_LOWBAT_Pos             (7U)
#define PMUC_WSR_LOWBAT_Msk             (0x1UL << PMUC_WSR_LOWBAT_Pos)
#define PMUC_WSR_LOWBAT                 PMUC_WSR_LOWBAT_Msk
#define PMUC_WSR_CHG_Pos                (8U)
#define PMUC_WSR_CHG_Msk                (0x1UL << PMUC_WSR_CHG_Pos)
#define PMUC_WSR_CHG                    PMUC_WSR_CHG_Msk

/******************** Bit definition for PMUC_WCR register ********************/
#define PMUC_WCR_WDT1_Pos               (1U)
#define PMUC_WCR_WDT1_Msk               (0x1UL << PMUC_WCR_WDT1_Pos)
#define PMUC_WCR_WDT1                   PMUC_WCR_WDT1_Msk
#define PMUC_WCR_WDT2_Pos               (2U)
#define PMUC_WCR_WDT2_Msk               (0x1UL << PMUC_WCR_WDT2_Pos)
#define PMUC_WCR_WDT2                   PMUC_WCR_WDT2_Msk
#define PMUC_WCR_PIN0_Pos               (3U)
#define PMUC_WCR_PIN0_Msk               (0x1UL << PMUC_WCR_PIN0_Pos)
#define PMUC_WCR_PIN0                   PMUC_WCR_PIN0_Msk
#define PMUC_WCR_PIN1_Pos               (4U)
#define PMUC_WCR_PIN1_Msk               (0x1UL << PMUC_WCR_PIN1_Pos)
#define PMUC_WCR_PIN1                   PMUC_WCR_PIN1_Msk
#define PMUC_WCR_PWRKEY_Pos             (6U)
#define PMUC_WCR_PWRKEY_Msk             (0x1UL << PMUC_WCR_PWRKEY_Pos)
#define PMUC_WCR_PWRKEY                 PMUC_WCR_PWRKEY_Msk
#define PMUC_WCR_LOWBAT_Pos             (7U)
#define PMUC_WCR_LOWBAT_Msk             (0x1UL << PMUC_WCR_LOWBAT_Pos)
#define PMUC_WCR_LOWBAT                 PMUC_WCR_LOWBAT_Msk
#define PMUC_WCR_AON_Pos                (31U)
#define PMUC_WCR_AON_Msk                (0x1UL << PMUC_WCR_AON_Pos)
#define PMUC_WCR_AON                    PMUC_WCR_AON_Msk

/****************** Bit definition for PMUC_VRTC_CR register ******************/
#define PMUC_VRTC_CR_VRTC_VBIT_Pos      (0U)
#define PMUC_VRTC_CR_VRTC_VBIT_Msk      (0xFUL << PMUC_VRTC_CR_VRTC_VBIT_Pos)
#define PMUC_VRTC_CR_VRTC_VBIT          PMUC_VRTC_CR_VRTC_VBIT_Msk
#define PMUC_VRTC_CR_VRTC_TRIM_Pos      (4U)
#define PMUC_VRTC_CR_VRTC_TRIM_Msk      (0xFUL << PMUC_VRTC_CR_VRTC_TRIM_Pos)
#define PMUC_VRTC_CR_VRTC_TRIM          PMUC_VRTC_CR_VRTC_TRIM_Msk
#define PMUC_VRTC_CR_BOR_EN_Pos         (8U)
#define PMUC_VRTC_CR_BOR_EN_Msk         (0x1UL << PMUC_VRTC_CR_BOR_EN_Pos)
#define PMUC_VRTC_CR_BOR_EN             PMUC_VRTC_CR_BOR_EN_Msk
#define PMUC_VRTC_CR_BOR_VT_TRIM_Pos    (9U)
#define PMUC_VRTC_CR_BOR_VT_TRIM_Msk    (0xFUL << PMUC_VRTC_CR_BOR_VT_TRIM_Pos)
#define PMUC_VRTC_CR_BOR_VT_TRIM        PMUC_VRTC_CR_BOR_VT_TRIM_Msk

/****************** Bit definition for PMUC_VRET_CR register ******************/
#define PMUC_VRET_CR_EN_Pos             (0U)
#define PMUC_VRET_CR_EN_Msk             (0x1UL << PMUC_VRET_CR_EN_Pos)
#define PMUC_VRET_CR_EN                 PMUC_VRET_CR_EN_Msk
#define PMUC_VRET_CR_BM_Pos             (1U)
#define PMUC_VRET_CR_BM_Msk             (0x1UL << PMUC_VRET_CR_BM_Pos)
#define PMUC_VRET_CR_BM                 PMUC_VRET_CR_BM_Msk
#define PMUC_VRET_CR_VBIT_Pos           (2U)
#define PMUC_VRET_CR_VBIT_Msk           (0xFUL << PMUC_VRET_CR_VBIT_Pos)
#define PMUC_VRET_CR_VBIT               PMUC_VRET_CR_VBIT_Msk
#define PMUC_VRET_CR_TRIM_Pos           (10U)
#define PMUC_VRET_CR_TRIM_Msk           (0xFUL << PMUC_VRET_CR_TRIM_Pos)
#define PMUC_VRET_CR_TRIM               PMUC_VRET_CR_TRIM_Msk
#define PMUC_VRET_CR_DLY_Pos            (16U)
#define PMUC_VRET_CR_DLY_Msk            (0x3FUL << PMUC_VRET_CR_DLY_Pos)
#define PMUC_VRET_CR_DLY                PMUC_VRET_CR_DLY_Msk
#define PMUC_VRET_CR_RDY_Pos            (31U)
#define PMUC_VRET_CR_RDY_Msk            (0x1UL << PMUC_VRET_CR_RDY_Pos)
#define PMUC_VRET_CR_RDY                PMUC_VRET_CR_RDY_Msk

/***************** Bit definition for PMUC_LRC10_CR register ******************/
#define PMUC_LRC10_CR_EN_Pos            (0U)
#define PMUC_LRC10_CR_EN_Msk            (0x1UL << PMUC_LRC10_CR_EN_Pos)
#define PMUC_LRC10_CR_EN                PMUC_LRC10_CR_EN_Msk
#define PMUC_LRC10_CR_CMPBM1_Pos        (1U)
#define PMUC_LRC10_CR_CMPBM1_Msk        (0x3UL << PMUC_LRC10_CR_CMPBM1_Pos)
#define PMUC_LRC10_CR_CMPBM1            PMUC_LRC10_CR_CMPBM1_Msk
#define PMUC_LRC10_CR_CMPBM2_Pos        (3U)
#define PMUC_LRC10_CR_CMPBM2_Msk        (0x1UL << PMUC_LRC10_CR_CMPBM2_Pos)
#define PMUC_LRC10_CR_CMPBM2            PMUC_LRC10_CR_CMPBM2_Msk
#define PMUC_LRC10_CR_CHGCRT_Pos        (4U)
#define PMUC_LRC10_CR_CHGCRT_Msk        (0x3UL << PMUC_LRC10_CR_CHGCRT_Pos)
#define PMUC_LRC10_CR_CHGCRT            PMUC_LRC10_CR_CHGCRT_Msk
#define PMUC_LRC10_CR_CHGCAP_Pos        (6U)
#define PMUC_LRC10_CR_CHGCAP_Msk        (0x3UL << PMUC_LRC10_CR_CHGCAP_Pos)
#define PMUC_LRC10_CR_CHGCAP            PMUC_LRC10_CR_CHGCAP_Msk
#define PMUC_LRC10_CR_REFRES_Pos        (8U)
#define PMUC_LRC10_CR_REFRES_Msk        (0x1UL << PMUC_LRC10_CR_REFRES_Pos)
#define PMUC_LRC10_CR_REFRES            PMUC_LRC10_CR_REFRES_Msk
#define PMUC_LRC10_CR_RDY_Pos           (31U)
#define PMUC_LRC10_CR_RDY_Msk           (0x1UL << PMUC_LRC10_CR_RDY_Pos)
#define PMUC_LRC10_CR_RDY               PMUC_LRC10_CR_RDY_Msk

/***************** Bit definition for PMUC_LRC32_CR register ******************/
#define PMUC_LRC32_CR_EN_Pos            (0U)
#define PMUC_LRC32_CR_EN_Msk            (0x1UL << PMUC_LRC32_CR_EN_Pos)
#define PMUC_LRC32_CR_EN                PMUC_LRC32_CR_EN_Msk
#define PMUC_LRC32_CR_CMPBM1_Pos        (1U)
#define PMUC_LRC32_CR_CMPBM1_Msk        (0x3UL << PMUC_LRC32_CR_CMPBM1_Pos)
#define PMUC_LRC32_CR_CMPBM1            PMUC_LRC32_CR_CMPBM1_Msk
#define PMUC_LRC32_CR_CMPBM2_Pos        (3U)
#define PMUC_LRC32_CR_CMPBM2_Msk        (0x1UL << PMUC_LRC32_CR_CMPBM2_Pos)
#define PMUC_LRC32_CR_CMPBM2            PMUC_LRC32_CR_CMPBM2_Msk
#define PMUC_LRC32_CR_CHGCRT_Pos        (4U)
#define PMUC_LRC32_CR_CHGCRT_Msk        (0x3UL << PMUC_LRC32_CR_CHGCRT_Pos)
#define PMUC_LRC32_CR_CHGCRT            PMUC_LRC32_CR_CHGCRT_Msk
#define PMUC_LRC32_CR_RSEL_Pos          (6U)
#define PMUC_LRC32_CR_RSEL_Msk          (0xFUL << PMUC_LRC32_CR_RSEL_Pos)
#define PMUC_LRC32_CR_RSEL              PMUC_LRC32_CR_RSEL_Msk
#define PMUC_LRC32_CR_RDY_Pos           (31U)
#define PMUC_LRC32_CR_RDY_Msk           (0x1UL << PMUC_LRC32_CR_RDY_Pos)
#define PMUC_LRC32_CR_RDY               PMUC_LRC32_CR_RDY_Msk

/****************** Bit definition for PMUC_LXT_CR register *******************/
#define PMUC_LXT_CR_EN_Pos              (0U)
#define PMUC_LXT_CR_EN_Msk              (0x1UL << PMUC_LXT_CR_EN_Pos)
#define PMUC_LXT_CR_EN                  PMUC_LXT_CR_EN_Msk
#define PMUC_LXT_CR_RSN_Pos             (1U)
#define PMUC_LXT_CR_RSN_Msk             (0x1UL << PMUC_LXT_CR_RSN_Pos)
#define PMUC_LXT_CR_RSN                 PMUC_LXT_CR_RSN_Msk
#define PMUC_LXT_CR_BM_Pos              (2U)
#define PMUC_LXT_CR_BM_Msk              (0xFUL << PMUC_LXT_CR_BM_Pos)
#define PMUC_LXT_CR_BM                  PMUC_LXT_CR_BM_Msk
#define PMUC_LXT_CR_AMP_BM_Pos          (6U)
#define PMUC_LXT_CR_AMP_BM_Msk          (0x3UL << PMUC_LXT_CR_AMP_BM_Pos)
#define PMUC_LXT_CR_AMP_BM              PMUC_LXT_CR_AMP_BM_Msk
#define PMUC_LXT_CR_AMPCTRL_ENB_Pos     (8U)
#define PMUC_LXT_CR_AMPCTRL_ENB_Msk     (0x1UL << PMUC_LXT_CR_AMPCTRL_ENB_Pos)
#define PMUC_LXT_CR_AMPCTRL_ENB         PMUC_LXT_CR_AMPCTRL_ENB_Msk
#define PMUC_LXT_CR_BMSEL_Pos           (9U)
#define PMUC_LXT_CR_BMSEL_Msk           (0x1UL << PMUC_LXT_CR_BMSEL_Pos)
#define PMUC_LXT_CR_BMSEL               PMUC_LXT_CR_BMSEL_Msk
#define PMUC_LXT_CR_BMSTART_Pos         (10U)
#define PMUC_LXT_CR_BMSTART_Msk         (0xFUL << PMUC_LXT_CR_BMSTART_Pos)
#define PMUC_LXT_CR_BMSTART             PMUC_LXT_CR_BMSTART_Msk
#define PMUC_LXT_CR_CAP_SEL_Pos         (14U)
#define PMUC_LXT_CR_CAP_SEL_Msk         (0x1UL << PMUC_LXT_CR_CAP_SEL_Pos)
#define PMUC_LXT_CR_CAP_SEL             PMUC_LXT_CR_CAP_SEL_Msk
#define PMUC_LXT_CR_EXT_EN_Pos          (15U)
#define PMUC_LXT_CR_EXT_EN_Msk          (0x1UL << PMUC_LXT_CR_EXT_EN_Pos)
#define PMUC_LXT_CR_EXT_EN              PMUC_LXT_CR_EXT_EN_Msk
#define PMUC_LXT_CR_RDY_Pos             (31U)
#define PMUC_LXT_CR_RDY_Msk             (0x1UL << PMUC_LXT_CR_RDY_Pos)
#define PMUC_LXT_CR_RDY                 PMUC_LXT_CR_RDY_Msk

/****************** Bit definition for PMUC_AON_BG register *******************/
#define PMUC_AON_BG_BUF_VOS_TRIM_Pos    (0U)
#define PMUC_AON_BG_BUF_VOS_TRIM_Msk    (0x7UL << PMUC_AON_BG_BUF_VOS_TRIM_Pos)
#define PMUC_AON_BG_BUF_VOS_TRIM        PMUC_AON_BG_BUF_VOS_TRIM_Msk
#define PMUC_AON_BG_BUF_VOS_STEP_Pos    (3U)
#define PMUC_AON_BG_BUF_VOS_STEP_Msk    (0x3UL << PMUC_AON_BG_BUF_VOS_STEP_Pos)
#define PMUC_AON_BG_BUF_VOS_STEP        PMUC_AON_BG_BUF_VOS_STEP_Msk
#define PMUC_AON_BG_BUF_VOS_POLAR_Pos   (5U)
#define PMUC_AON_BG_BUF_VOS_POLAR_Msk   (0x1UL << PMUC_AON_BG_BUF_VOS_POLAR_Pos)
#define PMUC_AON_BG_BUF_VOS_POLAR       PMUC_AON_BG_BUF_VOS_POLAR_Msk

/****************** Bit definition for PMUC_AON_LDO register ******************/
#define PMUC_AON_LDO_VBAT_LDO_SET_VOUT_Pos  (0U)
#define PMUC_AON_LDO_VBAT_LDO_SET_VOUT_Msk  (0xFUL << PMUC_AON_LDO_VBAT_LDO_SET_VOUT_Pos)
#define PMUC_AON_LDO_VBAT_LDO_SET_VOUT  PMUC_AON_LDO_VBAT_LDO_SET_VOUT_Msk
#define PMUC_AON_LDO_VBAT_POR_TH_Pos    (4U)
#define PMUC_AON_LDO_VBAT_POR_TH_Msk    (0x7UL << PMUC_AON_LDO_VBAT_POR_TH_Pos)
#define PMUC_AON_LDO_VBAT_POR_TH        PMUC_AON_LDO_VBAT_POR_TH_Msk

/***************** Bit definition for PMUC_BUCK_CR1 register ******************/
#define PMUC_BUCK_CR1_EN_Pos            (0U)
#define PMUC_BUCK_CR1_EN_Msk            (0x1UL << PMUC_BUCK_CR1_EN_Pos)
#define PMUC_BUCK_CR1_EN                PMUC_BUCK_CR1_EN_Msk
#define PMUC_BUCK_CR1_CTRL_Pos          (1U)
#define PMUC_BUCK_CR1_CTRL_Msk          (0x1UL << PMUC_BUCK_CR1_CTRL_Pos)
#define PMUC_BUCK_CR1_CTRL              PMUC_BUCK_CR1_CTRL_Msk
#define PMUC_BUCK_CR1_MOT_CTUNE_Pos     (6U)
#define PMUC_BUCK_CR1_MOT_CTUNE_Msk     (0x7UL << PMUC_BUCK_CR1_MOT_CTUNE_Pos)
#define PMUC_BUCK_CR1_MOT_CTUNE         PMUC_BUCK_CR1_MOT_CTUNE_Msk
#define PMUC_BUCK_CR1_COT_CTUNE_Pos     (9U)
#define PMUC_BUCK_CR1_COT_CTUNE_Msk     (0x7UL << PMUC_BUCK_CR1_COT_CTUNE_Pos)
#define PMUC_BUCK_CR1_COT_CTUNE         PMUC_BUCK_CR1_COT_CTUNE_Msk
#define PMUC_BUCK_CR1_COMP_BM_AHI_Pos   (12U)
#define PMUC_BUCK_CR1_COMP_BM_AHI_Msk   (0x1UL << PMUC_BUCK_CR1_COMP_BM_AHI_Pos)
#define PMUC_BUCK_CR1_COMP_BM_AHI       PMUC_BUCK_CR1_COMP_BM_AHI_Msk
#define PMUC_BUCK_CR1_COMP_IQ_TUNE_Pos  (13U)
#define PMUC_BUCK_CR1_COMP_IQ_TUNE_Msk  (0x3UL << PMUC_BUCK_CR1_COMP_IQ_TUNE_Pos)
#define PMUC_BUCK_CR1_COMP_IQ_TUNE      PMUC_BUCK_CR1_COMP_IQ_TUNE_Msk
#define PMUC_BUCK_CR1_COMP_IDYN_TUNE_Pos  (15U)
#define PMUC_BUCK_CR1_COMP_IDYN_TUNE_Msk  (0x3UL << PMUC_BUCK_CR1_COMP_IDYN_TUNE_Pos)
#define PMUC_BUCK_CR1_COMP_IDYN_TUNE    PMUC_BUCK_CR1_COMP_IDYN_TUNE_Msk
#define PMUC_BUCK_CR1_IOCP_TUNE_Pos     (17U)
#define PMUC_BUCK_CR1_IOCP_TUNE_Msk     (0x7UL << PMUC_BUCK_CR1_IOCP_TUNE_Pos)
#define PMUC_BUCK_CR1_IOCP_TUNE         PMUC_BUCK_CR1_IOCP_TUNE_Msk
#define PMUC_BUCK_CR1_SEL_IOCP_HI_Pos   (20U)
#define PMUC_BUCK_CR1_SEL_IOCP_HI_Msk   (0x1UL << PMUC_BUCK_CR1_SEL_IOCP_HI_Pos)
#define PMUC_BUCK_CR1_SEL_IOCP_HI       PMUC_BUCK_CR1_SEL_IOCP_HI_Msk
#define PMUC_BUCK_CR1_SEL_LX22_Pos      (21U)
#define PMUC_BUCK_CR1_SEL_LX22_Msk      (0x1UL << PMUC_BUCK_CR1_SEL_LX22_Pos)
#define PMUC_BUCK_CR1_SEL_LX22          PMUC_BUCK_CR1_SEL_LX22_Msk
#define PMUC_BUCK_CR1_OCP_AON_Pos       (22U)
#define PMUC_BUCK_CR1_OCP_AON_Msk       (0x1UL << PMUC_BUCK_CR1_OCP_AON_Pos)
#define PMUC_BUCK_CR1_OCP_AON           PMUC_BUCK_CR1_OCP_AON_Msk
#define PMUC_BUCK_CR1_ZCD_AON_Pos       (23U)
#define PMUC_BUCK_CR1_ZCD_AON_Msk       (0x1UL << PMUC_BUCK_CR1_ZCD_AON_Pos)
#define PMUC_BUCK_CR1_ZCD_AON           PMUC_BUCK_CR1_ZCD_AON_Msk
#define PMUC_BUCK_CR1_UVLO_X_BIAS_Pos   (24U)
#define PMUC_BUCK_CR1_UVLO_X_BIAS_Msk   (0x1UL << PMUC_BUCK_CR1_UVLO_X_BIAS_Pos)
#define PMUC_BUCK_CR1_UVLO_X_BIAS       PMUC_BUCK_CR1_UVLO_X_BIAS_Msk
#define PMUC_BUCK_CR1_BG_BUF_VOS_TRIM_Pos  (25U)
#define PMUC_BUCK_CR1_BG_BUF_VOS_TRIM_Msk  (0x7UL << PMUC_BUCK_CR1_BG_BUF_VOS_TRIM_Pos)
#define PMUC_BUCK_CR1_BG_BUF_VOS_TRIM   PMUC_BUCK_CR1_BG_BUF_VOS_TRIM_Msk
#define PMUC_BUCK_CR1_BG_BUF_VOS_STEP_Pos  (28U)
#define PMUC_BUCK_CR1_BG_BUF_VOS_STEP_Msk  (0x3UL << PMUC_BUCK_CR1_BG_BUF_VOS_STEP_Pos)
#define PMUC_BUCK_CR1_BG_BUF_VOS_STEP   PMUC_BUCK_CR1_BG_BUF_VOS_STEP_Msk
#define PMUC_BUCK_CR1_BG_BUF_VOS_POLAR_Pos  (30U)
#define PMUC_BUCK_CR1_BG_BUF_VOS_POLAR_Msk  (0x1UL << PMUC_BUCK_CR1_BG_BUF_VOS_POLAR_Pos)
#define PMUC_BUCK_CR1_BG_BUF_VOS_POLAR  PMUC_BUCK_CR1_BG_BUF_VOS_POLAR_Msk
#define PMUC_BUCK_CR1_SS_DONE_Pos       (31U)
#define PMUC_BUCK_CR1_SS_DONE_Msk       (0x1UL << PMUC_BUCK_CR1_SS_DONE_Pos)
#define PMUC_BUCK_CR1_SS_DONE           PMUC_BUCK_CR1_SS_DONE_Msk

/***************** Bit definition for PMUC_BUCK_CR2 register ******************/
#define PMUC_BUCK_CR2_H2M_EN_Pos        (0U)
#define PMUC_BUCK_CR2_H2M_EN_Msk        (0x1UL << PMUC_BUCK_CR2_H2M_EN_Pos)
#define PMUC_BUCK_CR2_H2M_EN            PMUC_BUCK_CR2_H2M_EN_Msk
#define PMUC_BUCK_CR2_H2L_EN_Pos        (1U)
#define PMUC_BUCK_CR2_H2L_EN_Msk        (0x1UL << PMUC_BUCK_CR2_H2L_EN_Pos)
#define PMUC_BUCK_CR2_H2L_EN            PMUC_BUCK_CR2_H2L_EN_Msk
#define PMUC_BUCK_CR2_M2L_EN_Pos        (2U)
#define PMUC_BUCK_CR2_M2L_EN_Msk        (0x1UL << PMUC_BUCK_CR2_M2L_EN_Pos)
#define PMUC_BUCK_CR2_M2L_EN            PMUC_BUCK_CR2_M2L_EN_Msk
#define PMUC_BUCK_CR2_L2M_EN_Pos        (3U)
#define PMUC_BUCK_CR2_L2M_EN_Msk        (0x1UL << PMUC_BUCK_CR2_L2M_EN_Pos)
#define PMUC_BUCK_CR2_L2M_EN            PMUC_BUCK_CR2_L2M_EN_Msk
#define PMUC_BUCK_CR2_M2H_CNT_Pos       (4U)
#define PMUC_BUCK_CR2_M2H_CNT_Msk       (0xFUL << PMUC_BUCK_CR2_M2H_CNT_Pos)
#define PMUC_BUCK_CR2_M2H_CNT           PMUC_BUCK_CR2_M2H_CNT_Msk
#define PMUC_BUCK_CR2_L2H_CNT_Pos       (8U)
#define PMUC_BUCK_CR2_L2H_CNT_Msk       (0xFUL << PMUC_BUCK_CR2_L2H_CNT_Pos)
#define PMUC_BUCK_CR2_L2H_CNT           PMUC_BUCK_CR2_L2H_CNT_Msk
#define PMUC_BUCK_CR2_L2M_CNT_Pos       (12U)
#define PMUC_BUCK_CR2_L2M_CNT_Msk       (0xFUL << PMUC_BUCK_CR2_L2M_CNT_Pos)
#define PMUC_BUCK_CR2_L2M_CNT           PMUC_BUCK_CR2_L2M_CNT_Msk
#define PMUC_BUCK_CR2_BYPASS_PG_Pos     (16U)
#define PMUC_BUCK_CR2_BYPASS_PG_Msk     (0x1UL << PMUC_BUCK_CR2_BYPASS_PG_Pos)
#define PMUC_BUCK_CR2_BYPASS_PG         PMUC_BUCK_CR2_BYPASS_PG_Msk
#define PMUC_BUCK_CR2_BYPASS_OCP_Pos    (17U)
#define PMUC_BUCK_CR2_BYPASS_OCP_Msk    (0x1UL << PMUC_BUCK_CR2_BYPASS_OCP_Pos)
#define PMUC_BUCK_CR2_BYPASS_OCP        PMUC_BUCK_CR2_BYPASS_OCP_Msk
#define PMUC_BUCK_CR2_BYPASS_UVLO_Pos   (18U)
#define PMUC_BUCK_CR2_BYPASS_UVLO_Msk   (0x1UL << PMUC_BUCK_CR2_BYPASS_UVLO_Pos)
#define PMUC_BUCK_CR2_BYPASS_UVLO       PMUC_BUCK_CR2_BYPASS_UVLO_Msk
#define PMUC_BUCK_CR2_FORCE_RDY_Pos     (19U)
#define PMUC_BUCK_CR2_FORCE_RDY_Msk     (0x1UL << PMUC_BUCK_CR2_FORCE_RDY_Pos)
#define PMUC_BUCK_CR2_FORCE_RDY         PMUC_BUCK_CR2_FORCE_RDY_Msk
#define PMUC_BUCK_CR2_SET_VOUT_M_Pos    (20U)
#define PMUC_BUCK_CR2_SET_VOUT_M_Msk    (0xFUL << PMUC_BUCK_CR2_SET_VOUT_M_Pos)
#define PMUC_BUCK_CR2_SET_VOUT_M        PMUC_BUCK_CR2_SET_VOUT_M_Msk
#define PMUC_BUCK_CR2_SET_VOUT_L_Pos    (24U)
#define PMUC_BUCK_CR2_SET_VOUT_L_Msk    (0xFUL << PMUC_BUCK_CR2_SET_VOUT_L_Pos)
#define PMUC_BUCK_CR2_SET_VOUT_L        PMUC_BUCK_CR2_SET_VOUT_L_Msk
#define PMUC_BUCK_CR2_TDIS_Pos          (28U)
#define PMUC_BUCK_CR2_TDIS_Msk          (0xFUL << PMUC_BUCK_CR2_TDIS_Pos)
#define PMUC_BUCK_CR2_TDIS              PMUC_BUCK_CR2_TDIS_Msk

/****************** Bit definition for PMUC_CHG_CR1 register ******************/
#define PMUC_CHG_CR1_EN_Pos             (0U)
#define PMUC_CHG_CR1_EN_Msk             (0x1UL << PMUC_CHG_CR1_EN_Pos)
#define PMUC_CHG_CR1_EN                 PMUC_CHG_CR1_EN_Msk
#define PMUC_CHG_CR1_LOOP_EN_Pos        (1U)
#define PMUC_CHG_CR1_LOOP_EN_Msk        (0x1UL << PMUC_CHG_CR1_LOOP_EN_Pos)
#define PMUC_CHG_CR1_LOOP_EN            PMUC_CHG_CR1_LOOP_EN_Msk
#define PMUC_CHG_CR1_CC_ICTRL_Pos       (2U)
#define PMUC_CHG_CR1_CC_ICTRL_Msk       (0x3FUL << PMUC_CHG_CR1_CC_ICTRL_Pos)
#define PMUC_CHG_CR1_CC_ICTRL           PMUC_CHG_CR1_CC_ICTRL_Msk
#define PMUC_CHG_CR1_CC_VCTRL_Pos       (8U)
#define PMUC_CHG_CR1_CC_VCTRL_Msk       (0x3FUL << PMUC_CHG_CR1_CC_VCTRL_Pos)
#define PMUC_CHG_CR1_CC_VCTRL           PMUC_CHG_CR1_CC_VCTRL_Msk
#define PMUC_CHG_CR1_CC_MP_Pos          (14U)
#define PMUC_CHG_CR1_CC_MP_Msk          (0x1FUL << PMUC_CHG_CR1_CC_MP_Pos)
#define PMUC_CHG_CR1_CC_MP              PMUC_CHG_CR1_CC_MP_Msk
#define PMUC_CHG_CR1_CC_MN_Pos          (19U)
#define PMUC_CHG_CR1_CC_MN_Msk          (0x1FUL << PMUC_CHG_CR1_CC_MN_Pos)
#define PMUC_CHG_CR1_CC_MN              PMUC_CHG_CR1_CC_MN_Msk
#define PMUC_CHG_CR1_CC_RANGE_Pos       (24U)
#define PMUC_CHG_CR1_CC_RANGE_Msk       (0x3UL << PMUC_CHG_CR1_CC_RANGE_Pos)
#define PMUC_CHG_CR1_CC_RANGE           PMUC_CHG_CR1_CC_RANGE_Msk
#define PMUC_CHG_CR1_CV_VCTRL_Pos       (26U)
#define PMUC_CHG_CR1_CV_VCTRL_Msk       (0x3FUL << PMUC_CHG_CR1_CV_VCTRL_Pos)
#define PMUC_CHG_CR1_CV_VCTRL           PMUC_CHG_CR1_CV_VCTRL_Msk

/****************** Bit definition for PMUC_CHG_CR2 register ******************/
#define PMUC_CHG_CR2_BG_PROG_V1P2_Pos   (0U)
#define PMUC_CHG_CR2_BG_PROG_V1P2_Msk   (0xFUL << PMUC_CHG_CR2_BG_PROG_V1P2_Pos)
#define PMUC_CHG_CR2_BG_PROG_V1P2       PMUC_CHG_CR2_BG_PROG_V1P2_Msk
#define PMUC_CHG_CR2_PRECC_RANGE_Pos    (4U)
#define PMUC_CHG_CR2_PRECC_RANGE_Msk    (0x3UL << PMUC_CHG_CR2_PRECC_RANGE_Pos)
#define PMUC_CHG_CR2_PRECC_RANGE        PMUC_CHG_CR2_PRECC_RANGE_Msk
#define PMUC_CHG_CR2_PRECC_ICTRL_Pos    (6U)
#define PMUC_CHG_CR2_PRECC_ICTRL_Msk    (0x3FUL << PMUC_CHG_CR2_PRECC_ICTRL_Pos)
#define PMUC_CHG_CR2_PRECC_ICTRL        PMUC_CHG_CR2_PRECC_ICTRL_Msk
#define PMUC_CHG_CR2_REP_VCTRL_Pos      (12U)
#define PMUC_CHG_CR2_REP_VCTRL_Msk      (0x3FUL << PMUC_CHG_CR2_REP_VCTRL_Pos)
#define PMUC_CHG_CR2_REP_VCTRL          PMUC_CHG_CR2_REP_VCTRL_Msk
#define PMUC_CHG_CR2_HIGH_VCTRL_Pos     (18U)
#define PMUC_CHG_CR2_HIGH_VCTRL_Msk     (0x3FUL << PMUC_CHG_CR2_HIGH_VCTRL_Pos)
#define PMUC_CHG_CR2_HIGH_VCTRL         PMUC_CHG_CR2_HIGH_VCTRL_Msk
#define PMUC_CHG_CR2_BM_EOC_Pos         (24U)
#define PMUC_CHG_CR2_BM_EOC_Msk         (0x7UL << PMUC_CHG_CR2_BM_EOC_Pos)
#define PMUC_CHG_CR2_BM_EOC             PMUC_CHG_CR2_BM_EOC_Msk
#define PMUC_CHG_CR2_RANGE_EOC_Pos      (27U)
#define PMUC_CHG_CR2_RANGE_EOC_Msk      (0x1UL << PMUC_CHG_CR2_RANGE_EOC_Pos)
#define PMUC_CHG_CR2_RANGE_EOC          PMUC_CHG_CR2_RANGE_EOC_Msk
#define PMUC_CHG_CR2_VBAT_RANGE_Pos     (28U)
#define PMUC_CHG_CR2_VBAT_RANGE_Msk     (0xFUL << PMUC_CHG_CR2_VBAT_RANGE_Pos)
#define PMUC_CHG_CR2_VBAT_RANGE         PMUC_CHG_CR2_VBAT_RANGE_Msk

/****************** Bit definition for PMUC_CHG_CR3 register ******************/
#define PMUC_CHG_CR3_DLY1_Pos           (0U)
#define PMUC_CHG_CR3_DLY1_Msk           (0x3FUL << PMUC_CHG_CR3_DLY1_Pos)
#define PMUC_CHG_CR3_DLY1               PMUC_CHG_CR3_DLY1_Msk
#define PMUC_CHG_CR3_DLY2_Pos           (6U)
#define PMUC_CHG_CR3_DLY2_Msk           (0x1FUL << PMUC_CHG_CR3_DLY2_Pos)
#define PMUC_CHG_CR3_DLY2               PMUC_CHG_CR3_DLY2_Msk
#define PMUC_CHG_CR3_FORCE_RST_Pos      (30U)
#define PMUC_CHG_CR3_FORCE_RST_Msk      (0x1UL << PMUC_CHG_CR3_FORCE_RST_Pos)
#define PMUC_CHG_CR3_FORCE_RST          PMUC_CHG_CR3_FORCE_RST_Msk
#define PMUC_CHG_CR3_FORCE_CTRL_Pos     (31U)
#define PMUC_CHG_CR3_FORCE_CTRL_Msk     (0x1UL << PMUC_CHG_CR3_FORCE_CTRL_Pos)
#define PMUC_CHG_CR3_FORCE_CTRL         PMUC_CHG_CR3_FORCE_CTRL_Msk

/****************** Bit definition for PMUC_CHG_CR4 register ******************/
#define PMUC_CHG_CR4_IE_VBUS_RDY_Pos    (0U)
#define PMUC_CHG_CR4_IE_VBUS_RDY_Msk    (0x1UL << PMUC_CHG_CR4_IE_VBUS_RDY_Pos)
#define PMUC_CHG_CR4_IE_VBUS_RDY        PMUC_CHG_CR4_IE_VBUS_RDY_Msk
#define PMUC_CHG_CR4_IE_VBAT_HIGH_Pos   (1U)
#define PMUC_CHG_CR4_IE_VBAT_HIGH_Msk   (0x1UL << PMUC_CHG_CR4_IE_VBAT_HIGH_Pos)
#define PMUC_CHG_CR4_IE_VBAT_HIGH       PMUC_CHG_CR4_IE_VBAT_HIGH_Msk
#define PMUC_CHG_CR4_IE_ABOVE_REP_Pos   (2U)
#define PMUC_CHG_CR4_IE_ABOVE_REP_Msk   (0x1UL << PMUC_CHG_CR4_IE_ABOVE_REP_Pos)
#define PMUC_CHG_CR4_IE_ABOVE_REP       PMUC_CHG_CR4_IE_ABOVE_REP_Msk
#define PMUC_CHG_CR4_IE_ABOVE_CC_Pos    (3U)
#define PMUC_CHG_CR4_IE_ABOVE_CC_Msk    (0x1UL << PMUC_CHG_CR4_IE_ABOVE_CC_Pos)
#define PMUC_CHG_CR4_IE_ABOVE_CC        PMUC_CHG_CR4_IE_ABOVE_CC_Msk
#define PMUC_CHG_CR4_IE_CC_MODE_Pos     (4U)
#define PMUC_CHG_CR4_IE_CC_MODE_Msk     (0x1UL << PMUC_CHG_CR4_IE_CC_MODE_Pos)
#define PMUC_CHG_CR4_IE_CC_MODE         PMUC_CHG_CR4_IE_CC_MODE_Msk
#define PMUC_CHG_CR4_IE_CV_MODE_Pos     (5U)
#define PMUC_CHG_CR4_IE_CV_MODE_Msk     (0x1UL << PMUC_CHG_CR4_IE_CV_MODE_Pos)
#define PMUC_CHG_CR4_IE_CV_MODE         PMUC_CHG_CR4_IE_CV_MODE_Msk
#define PMUC_CHG_CR4_IE_EOC_MODE_Pos    (6U)
#define PMUC_CHG_CR4_IE_EOC_MODE_Msk    (0x1UL << PMUC_CHG_CR4_IE_EOC_MODE_Pos)
#define PMUC_CHG_CR4_IE_EOC_MODE        PMUC_CHG_CR4_IE_EOC_MODE_Msk
#define PMUC_CHG_CR4_IE_EOC_Pos         (7U)
#define PMUC_CHG_CR4_IE_EOC_Msk         (0x1UL << PMUC_CHG_CR4_IE_EOC_Pos)
#define PMUC_CHG_CR4_IE_EOC             PMUC_CHG_CR4_IE_EOC_Msk
#define PMUC_CHG_CR4_IM_VBUS_RDY_Pos    (11U)
#define PMUC_CHG_CR4_IM_VBUS_RDY_Msk    (0x7UL << PMUC_CHG_CR4_IM_VBUS_RDY_Pos)
#define PMUC_CHG_CR4_IM_VBUS_RDY        PMUC_CHG_CR4_IM_VBUS_RDY_Msk
#define PMUC_CHG_CR4_IM_VBAT_HIGH_Pos   (14U)
#define PMUC_CHG_CR4_IM_VBAT_HIGH_Msk   (0x7UL << PMUC_CHG_CR4_IM_VBAT_HIGH_Pos)
#define PMUC_CHG_CR4_IM_VBAT_HIGH       PMUC_CHG_CR4_IM_VBAT_HIGH_Msk
#define PMUC_CHG_CR4_IM_ABOVE_REP_Pos   (17U)
#define PMUC_CHG_CR4_IM_ABOVE_REP_Msk   (0x7UL << PMUC_CHG_CR4_IM_ABOVE_REP_Pos)
#define PMUC_CHG_CR4_IM_ABOVE_REP       PMUC_CHG_CR4_IM_ABOVE_REP_Msk
#define PMUC_CHG_CR4_IM_ABOVE_CC_Pos    (20U)
#define PMUC_CHG_CR4_IM_ABOVE_CC_Msk    (0x7UL << PMUC_CHG_CR4_IM_ABOVE_CC_Pos)
#define PMUC_CHG_CR4_IM_ABOVE_CC        PMUC_CHG_CR4_IM_ABOVE_CC_Msk
#define PMUC_CHG_CR4_IM_CC_MODE_Pos     (23U)
#define PMUC_CHG_CR4_IM_CC_MODE_Msk     (0x7UL << PMUC_CHG_CR4_IM_CC_MODE_Pos)
#define PMUC_CHG_CR4_IM_CC_MODE         PMUC_CHG_CR4_IM_CC_MODE_Msk
#define PMUC_CHG_CR4_IM_CV_MODE_Pos     (26U)
#define PMUC_CHG_CR4_IM_CV_MODE_Msk     (0x7UL << PMUC_CHG_CR4_IM_CV_MODE_Pos)
#define PMUC_CHG_CR4_IM_CV_MODE         PMUC_CHG_CR4_IM_CV_MODE_Msk
#define PMUC_CHG_CR4_IM_EOC_MODE_Pos    (29U)
#define PMUC_CHG_CR4_IM_EOC_MODE_Msk    (0x7UL << PMUC_CHG_CR4_IM_EOC_MODE_Pos)
#define PMUC_CHG_CR4_IM_EOC_MODE        PMUC_CHG_CR4_IM_EOC_MODE_Msk

/****************** Bit definition for PMUC_CHG_CR5 register ******************/
#define PMUC_CHG_CR5_IC_VBUS_RDY_Pos    (0U)
#define PMUC_CHG_CR5_IC_VBUS_RDY_Msk    (0x1UL << PMUC_CHG_CR5_IC_VBUS_RDY_Pos)
#define PMUC_CHG_CR5_IC_VBUS_RDY        PMUC_CHG_CR5_IC_VBUS_RDY_Msk
#define PMUC_CHG_CR5_IC_VBAT_HIGH_Pos   (1U)
#define PMUC_CHG_CR5_IC_VBAT_HIGH_Msk   (0x1UL << PMUC_CHG_CR5_IC_VBAT_HIGH_Pos)
#define PMUC_CHG_CR5_IC_VBAT_HIGH       PMUC_CHG_CR5_IC_VBAT_HIGH_Msk
#define PMUC_CHG_CR5_IC_ABOVE_REP_Pos   (2U)
#define PMUC_CHG_CR5_IC_ABOVE_REP_Msk   (0x1UL << PMUC_CHG_CR5_IC_ABOVE_REP_Pos)
#define PMUC_CHG_CR5_IC_ABOVE_REP       PMUC_CHG_CR5_IC_ABOVE_REP_Msk
#define PMUC_CHG_CR5_IC_ABOVE_CC_Pos    (3U)
#define PMUC_CHG_CR5_IC_ABOVE_CC_Msk    (0x1UL << PMUC_CHG_CR5_IC_ABOVE_CC_Pos)
#define PMUC_CHG_CR5_IC_ABOVE_CC        PMUC_CHG_CR5_IC_ABOVE_CC_Msk
#define PMUC_CHG_CR5_IC_CC_MODE_Pos     (4U)
#define PMUC_CHG_CR5_IC_CC_MODE_Msk     (0x1UL << PMUC_CHG_CR5_IC_CC_MODE_Pos)
#define PMUC_CHG_CR5_IC_CC_MODE         PMUC_CHG_CR5_IC_CC_MODE_Msk
#define PMUC_CHG_CR5_IC_CV_MODE_Pos     (5U)
#define PMUC_CHG_CR5_IC_CV_MODE_Msk     (0x1UL << PMUC_CHG_CR5_IC_CV_MODE_Pos)
#define PMUC_CHG_CR5_IC_CV_MODE         PMUC_CHG_CR5_IC_CV_MODE_Msk
#define PMUC_CHG_CR5_IC_EOC_MODE_Pos    (6U)
#define PMUC_CHG_CR5_IC_EOC_MODE_Msk    (0x1UL << PMUC_CHG_CR5_IC_EOC_MODE_Pos)
#define PMUC_CHG_CR5_IC_EOC_MODE        PMUC_CHG_CR5_IC_EOC_MODE_Msk
#define PMUC_CHG_CR5_IC_EOC_Pos         (7U)
#define PMUC_CHG_CR5_IC_EOC_Msk         (0x1UL << PMUC_CHG_CR5_IC_EOC_Pos)
#define PMUC_CHG_CR5_IC_EOC             PMUC_CHG_CR5_IC_EOC_Msk
#define PMUC_CHG_CR5_IS_VBUS_RDY_Pos    (16U)
#define PMUC_CHG_CR5_IS_VBUS_RDY_Msk    (0x1UL << PMUC_CHG_CR5_IS_VBUS_RDY_Pos)
#define PMUC_CHG_CR5_IS_VBUS_RDY        PMUC_CHG_CR5_IS_VBUS_RDY_Msk
#define PMUC_CHG_CR5_IS_VBAT_HIGH_Pos   (17U)
#define PMUC_CHG_CR5_IS_VBAT_HIGH_Msk   (0x1UL << PMUC_CHG_CR5_IS_VBAT_HIGH_Pos)
#define PMUC_CHG_CR5_IS_VBAT_HIGH       PMUC_CHG_CR5_IS_VBAT_HIGH_Msk
#define PMUC_CHG_CR5_IS_ABOVE_REP_Pos   (18U)
#define PMUC_CHG_CR5_IS_ABOVE_REP_Msk   (0x1UL << PMUC_CHG_CR5_IS_ABOVE_REP_Pos)
#define PMUC_CHG_CR5_IS_ABOVE_REP       PMUC_CHG_CR5_IS_ABOVE_REP_Msk
#define PMUC_CHG_CR5_IS_ABOVE_CC_Pos    (19U)
#define PMUC_CHG_CR5_IS_ABOVE_CC_Msk    (0x1UL << PMUC_CHG_CR5_IS_ABOVE_CC_Pos)
#define PMUC_CHG_CR5_IS_ABOVE_CC        PMUC_CHG_CR5_IS_ABOVE_CC_Msk
#define PMUC_CHG_CR5_IS_CC_MODE_Pos     (20U)
#define PMUC_CHG_CR5_IS_CC_MODE_Msk     (0x1UL << PMUC_CHG_CR5_IS_CC_MODE_Pos)
#define PMUC_CHG_CR5_IS_CC_MODE         PMUC_CHG_CR5_IS_CC_MODE_Msk
#define PMUC_CHG_CR5_IS_CV_MODE_Pos     (21U)
#define PMUC_CHG_CR5_IS_CV_MODE_Msk     (0x1UL << PMUC_CHG_CR5_IS_CV_MODE_Pos)
#define PMUC_CHG_CR5_IS_CV_MODE         PMUC_CHG_CR5_IS_CV_MODE_Msk
#define PMUC_CHG_CR5_IS_EOC_MODE_Pos    (22U)
#define PMUC_CHG_CR5_IS_EOC_MODE_Msk    (0x1UL << PMUC_CHG_CR5_IS_EOC_MODE_Pos)
#define PMUC_CHG_CR5_IS_EOC_MODE        PMUC_CHG_CR5_IS_EOC_MODE_Msk
#define PMUC_CHG_CR5_IS_EOC_Pos         (23U)
#define PMUC_CHG_CR5_IS_EOC_Msk         (0x1UL << PMUC_CHG_CR5_IS_EOC_Pos)
#define PMUC_CHG_CR5_IS_EOC             PMUC_CHG_CR5_IS_EOC_Msk

/****************** Bit definition for PMUC_CHG_SR register *******************/
#define PMUC_CHG_SR_VBUS_RDY_OUT_Pos    (0U)
#define PMUC_CHG_SR_VBUS_RDY_OUT_Msk    (0x1UL << PMUC_CHG_SR_VBUS_RDY_OUT_Pos)
#define PMUC_CHG_SR_VBUS_RDY_OUT        PMUC_CHG_SR_VBUS_RDY_OUT_Msk
#define PMUC_CHG_SR_VBAT_HIGH_OUT_Pos   (1U)
#define PMUC_CHG_SR_VBAT_HIGH_OUT_Msk   (0x1UL << PMUC_CHG_SR_VBAT_HIGH_OUT_Pos)
#define PMUC_CHG_SR_VBAT_HIGH_OUT       PMUC_CHG_SR_VBAT_HIGH_OUT_Msk
#define PMUC_CHG_SR_VBAT_ABOVE_REP_OUT_Pos  (2U)
#define PMUC_CHG_SR_VBAT_ABOVE_REP_OUT_Msk  (0x1UL << PMUC_CHG_SR_VBAT_ABOVE_REP_OUT_Pos)
#define PMUC_CHG_SR_VBAT_ABOVE_REP_OUT  PMUC_CHG_SR_VBAT_ABOVE_REP_OUT_Msk
#define PMUC_CHG_SR_VBAT_ABOVE_CC_OUT_Pos  (3U)
#define PMUC_CHG_SR_VBAT_ABOVE_CC_OUT_Msk  (0x1UL << PMUC_CHG_SR_VBAT_ABOVE_CC_OUT_Pos)
#define PMUC_CHG_SR_VBAT_ABOVE_CC_OUT   PMUC_CHG_SR_VBAT_ABOVE_CC_OUT_Msk
#define PMUC_CHG_SR_CC_MODE_Pos         (4U)
#define PMUC_CHG_SR_CC_MODE_Msk         (0x1UL << PMUC_CHG_SR_CC_MODE_Pos)
#define PMUC_CHG_SR_CC_MODE             PMUC_CHG_SR_CC_MODE_Msk
#define PMUC_CHG_SR_CV_MODE_Pos         (5U)
#define PMUC_CHG_SR_CV_MODE_Msk         (0x1UL << PMUC_CHG_SR_CV_MODE_Pos)
#define PMUC_CHG_SR_CV_MODE             PMUC_CHG_SR_CV_MODE_Msk
#define PMUC_CHG_SR_EOC_MODE_Pos        (6U)
#define PMUC_CHG_SR_EOC_MODE_Msk        (0x1UL << PMUC_CHG_SR_EOC_MODE_Pos)
#define PMUC_CHG_SR_EOC_MODE            PMUC_CHG_SR_EOC_MODE_Msk
#define PMUC_CHG_SR_CHG_STATE_Pos       (8U)
#define PMUC_CHG_SR_CHG_STATE_Msk       (0x7FUL << PMUC_CHG_SR_CHG_STATE_Pos)
#define PMUC_CHG_SR_CHG_STATE           PMUC_CHG_SR_CHG_STATE_Msk

/***************** Bit definition for PMUC_HPSYS_LDO register *****************/
#define PMUC_HPSYS_LDO_EN_Pos           (0U)
#define PMUC_HPSYS_LDO_EN_Msk           (0x1UL << PMUC_HPSYS_LDO_EN_Pos)
#define PMUC_HPSYS_LDO_EN               PMUC_HPSYS_LDO_EN_Msk
#define PMUC_HPSYS_LDO_BP_Pos           (1U)
#define PMUC_HPSYS_LDO_BP_Msk           (0x1UL << PMUC_HPSYS_LDO_BP_Pos)
#define PMUC_HPSYS_LDO_BP               PMUC_HPSYS_LDO_BP_Msk
#define PMUC_HPSYS_LDO_VREF_Pos         (2U)
#define PMUC_HPSYS_LDO_VREF_Msk         (0xFUL << PMUC_HPSYS_LDO_VREF_Pos)
#define PMUC_HPSYS_LDO_VREF             PMUC_HPSYS_LDO_VREF_Msk
#define PMUC_HPSYS_LDO_VREF2_Pos        (6U)
#define PMUC_HPSYS_LDO_VREF2_Msk        (0xFUL << PMUC_HPSYS_LDO_VREF2_Pos)
#define PMUC_HPSYS_LDO_VREF2            PMUC_HPSYS_LDO_VREF2_Msk
#define PMUC_HPSYS_LDO_DLY_Pos          (10U)
#define PMUC_HPSYS_LDO_DLY_Msk          (0x3FUL << PMUC_HPSYS_LDO_DLY_Pos)
#define PMUC_HPSYS_LDO_DLY              PMUC_HPSYS_LDO_DLY_Msk
#define PMUC_HPSYS_LDO_RDY_Pos          (16U)
#define PMUC_HPSYS_LDO_RDY_Msk          (0x1UL << PMUC_HPSYS_LDO_RDY_Pos)
#define PMUC_HPSYS_LDO_RDY              PMUC_HPSYS_LDO_RDY_Msk

/***************** Bit definition for PMUC_LPSYS_LDO register *****************/
#define PMUC_LPSYS_LDO_EN_Pos           (0U)
#define PMUC_LPSYS_LDO_EN_Msk           (0x1UL << PMUC_LPSYS_LDO_EN_Pos)
#define PMUC_LPSYS_LDO_EN               PMUC_LPSYS_LDO_EN_Msk
#define PMUC_LPSYS_LDO_BP_Pos           (1U)
#define PMUC_LPSYS_LDO_BP_Msk           (0x1UL << PMUC_LPSYS_LDO_BP_Pos)
#define PMUC_LPSYS_LDO_BP               PMUC_LPSYS_LDO_BP_Msk
#define PMUC_LPSYS_LDO_VREF_Pos         (2U)
#define PMUC_LPSYS_LDO_VREF_Msk         (0xFUL << PMUC_LPSYS_LDO_VREF_Pos)
#define PMUC_LPSYS_LDO_VREF             PMUC_LPSYS_LDO_VREF_Msk
#define PMUC_LPSYS_LDO_VREF2_Pos        (6U)
#define PMUC_LPSYS_LDO_VREF2_Msk        (0xFUL << PMUC_LPSYS_LDO_VREF2_Pos)
#define PMUC_LPSYS_LDO_VREF2            PMUC_LPSYS_LDO_VREF2_Msk
#define PMUC_LPSYS_LDO_DLY_Pos          (10U)
#define PMUC_LPSYS_LDO_DLY_Msk          (0x3FUL << PMUC_LPSYS_LDO_DLY_Pos)
#define PMUC_LPSYS_LDO_DLY              PMUC_LPSYS_LDO_DLY_Msk
#define PMUC_LPSYS_LDO_RDY_Pos          (16U)
#define PMUC_LPSYS_LDO_RDY_Msk          (0x1UL << PMUC_LPSYS_LDO_RDY_Pos)
#define PMUC_LPSYS_LDO_RDY              PMUC_LPSYS_LDO_RDY_Msk

/***************** Bit definition for PMUC_HPSYS_SWR register *****************/
#define PMUC_HPSYS_SWR_PSW_Pos          (0U)
#define PMUC_HPSYS_SWR_PSW_Msk          (0x3UL << PMUC_HPSYS_SWR_PSW_Pos)
#define PMUC_HPSYS_SWR_PSW              PMUC_HPSYS_SWR_PSW_Msk
#define PMUC_HPSYS_SWR_PSW_RET_Pos      (2U)
#define PMUC_HPSYS_SWR_PSW_RET_Msk      (0x3UL << PMUC_HPSYS_SWR_PSW_RET_Pos)
#define PMUC_HPSYS_SWR_PSW_RET          PMUC_HPSYS_SWR_PSW_RET_Msk
#define PMUC_HPSYS_SWR_DLY_Pos          (4U)
#define PMUC_HPSYS_SWR_DLY_Msk          (0x7UL << PMUC_HPSYS_SWR_DLY_Pos)
#define PMUC_HPSYS_SWR_DLY              PMUC_HPSYS_SWR_DLY_Msk
#define PMUC_HPSYS_SWR_NORET_Pos        (7U)
#define PMUC_HPSYS_SWR_NORET_Msk        (0x1UL << PMUC_HPSYS_SWR_NORET_Pos)
#define PMUC_HPSYS_SWR_NORET            PMUC_HPSYS_SWR_NORET_Msk
#define PMUC_HPSYS_SWR_RDY_Pos          (31U)
#define PMUC_HPSYS_SWR_RDY_Msk          (0x1UL << PMUC_HPSYS_SWR_RDY_Pos)
#define PMUC_HPSYS_SWR_RDY              PMUC_HPSYS_SWR_RDY_Msk

/***************** Bit definition for PMUC_LPSYS_SWR register *****************/
#define PMUC_LPSYS_SWR_PSW_Pos          (0U)
#define PMUC_LPSYS_SWR_PSW_Msk          (0x3UL << PMUC_LPSYS_SWR_PSW_Pos)
#define PMUC_LPSYS_SWR_PSW              PMUC_LPSYS_SWR_PSW_Msk
#define PMUC_LPSYS_SWR_PSW_RET_Pos      (2U)
#define PMUC_LPSYS_SWR_PSW_RET_Msk      (0x3UL << PMUC_LPSYS_SWR_PSW_RET_Pos)
#define PMUC_LPSYS_SWR_PSW_RET          PMUC_LPSYS_SWR_PSW_RET_Msk
#define PMUC_LPSYS_SWR_DLY_Pos          (4U)
#define PMUC_LPSYS_SWR_DLY_Msk          (0x7UL << PMUC_LPSYS_SWR_DLY_Pos)
#define PMUC_LPSYS_SWR_DLY              PMUC_LPSYS_SWR_DLY_Msk
#define PMUC_LPSYS_SWR_NORET_Pos        (7U)
#define PMUC_LPSYS_SWR_NORET_Msk        (0x1UL << PMUC_LPSYS_SWR_NORET_Pos)
#define PMUC_LPSYS_SWR_NORET            PMUC_LPSYS_SWR_NORET_Msk
#define PMUC_LPSYS_SWR_RDY_Pos          (31U)
#define PMUC_LPSYS_SWR_RDY_Msk          (0x1UL << PMUC_LPSYS_SWR_RDY_Pos)
#define PMUC_LPSYS_SWR_RDY              PMUC_LPSYS_SWR_RDY_Msk

/***************** Bit definition for PMUC_PERI_LDO register ******************/
#define PMUC_PERI_LDO_EN_LDO18_Pos      (0U)
#define PMUC_PERI_LDO_EN_LDO18_Msk      (0x1UL << PMUC_PERI_LDO_EN_LDO18_Pos)
#define PMUC_PERI_LDO_EN_LDO18          PMUC_PERI_LDO_EN_LDO18_Msk
#define PMUC_PERI_LDO_LDO18_VREF_SEL_Pos  (1U)
#define PMUC_PERI_LDO_LDO18_VREF_SEL_Msk  (0xFUL << PMUC_PERI_LDO_LDO18_VREF_SEL_Pos)
#define PMUC_PERI_LDO_LDO18_VREF_SEL    PMUC_PERI_LDO_LDO18_VREF_SEL_Msk
#define PMUC_PERI_LDO_LDO18_PD_Pos      (5U)
#define PMUC_PERI_LDO_LDO18_PD_Msk      (0x1UL << PMUC_PERI_LDO_LDO18_PD_Pos)
#define PMUC_PERI_LDO_LDO18_PD          PMUC_PERI_LDO_LDO18_PD_Msk
#define PMUC_PERI_LDO_EN_VDD33_LDO2_Pos  (8U)
#define PMUC_PERI_LDO_EN_VDD33_LDO2_Msk  (0x1UL << PMUC_PERI_LDO_EN_VDD33_LDO2_Pos)
#define PMUC_PERI_LDO_EN_VDD33_LDO2     PMUC_PERI_LDO_EN_VDD33_LDO2_Msk
#define PMUC_PERI_LDO_VDD33_LDO2_SET_VOUT_Pos  (9U)
#define PMUC_PERI_LDO_VDD33_LDO2_SET_VOUT_Msk  (0xFUL << PMUC_PERI_LDO_VDD33_LDO2_SET_VOUT_Pos)
#define PMUC_PERI_LDO_VDD33_LDO2_SET_VOUT  PMUC_PERI_LDO_VDD33_LDO2_SET_VOUT_Msk
#define PMUC_PERI_LDO_VDD33_LDO2_PD_Pos  (13U)
#define PMUC_PERI_LDO_VDD33_LDO2_PD_Msk  (0x1UL << PMUC_PERI_LDO_VDD33_LDO2_PD_Pos)
#define PMUC_PERI_LDO_VDD33_LDO2_PD     PMUC_PERI_LDO_VDD33_LDO2_PD_Msk
#define PMUC_PERI_LDO_EN_VDD33_LDO3_Pos  (16U)
#define PMUC_PERI_LDO_EN_VDD33_LDO3_Msk  (0x1UL << PMUC_PERI_LDO_EN_VDD33_LDO3_Pos)
#define PMUC_PERI_LDO_EN_VDD33_LDO3     PMUC_PERI_LDO_EN_VDD33_LDO3_Msk
#define PMUC_PERI_LDO_VDD33_LDO3_SET_VOUT_Pos  (17U)
#define PMUC_PERI_LDO_VDD33_LDO3_SET_VOUT_Msk  (0xFUL << PMUC_PERI_LDO_VDD33_LDO3_SET_VOUT_Pos)
#define PMUC_PERI_LDO_VDD33_LDO3_SET_VOUT  PMUC_PERI_LDO_VDD33_LDO3_SET_VOUT_Msk
#define PMUC_PERI_LDO_VDD33_LDO3_PD_Pos  (21U)
#define PMUC_PERI_LDO_VDD33_LDO3_PD_Msk  (0x1UL << PMUC_PERI_LDO_VDD33_LDO3_PD_Pos)
#define PMUC_PERI_LDO_VDD33_LDO3_PD     PMUC_PERI_LDO_VDD33_LDO3_PD_Msk

/****************** Bit definition for PMUC_PMU_TR register *******************/
#define PMUC_PMU_TR_PMU_DC_TR_Pos       (0U)
#define PMUC_PMU_TR_PMU_DC_TR_Msk       (0x7UL << PMUC_PMU_TR_PMU_DC_TR_Pos)
#define PMUC_PMU_TR_PMU_DC_TR           PMUC_PMU_TR_PMU_DC_TR_Msk
#define PMUC_PMU_TR_PMU_DC_MR_Pos       (3U)
#define PMUC_PMU_TR_PMU_DC_MR_Msk       (0x7UL << PMUC_PMU_TR_PMU_DC_MR_Pos)
#define PMUC_PMU_TR_PMU_DC_MR           PMUC_PMU_TR_PMU_DC_MR_Msk

/***************** Bit definition for PMUC_PMU_RSVD register ******************/
#define PMUC_PMU_RSVD_RESERVE0_Pos      (0U)
#define PMUC_PMU_RSVD_RESERVE0_Msk      (0xFFUL << PMUC_PMU_RSVD_RESERVE0_Pos)
#define PMUC_PMU_RSVD_RESERVE0          PMUC_PMU_RSVD_RESERVE0_Msk
#define PMUC_PMU_RSVD_RESERVE1_Pos      (8U)
#define PMUC_PMU_RSVD_RESERVE1_Msk      (0xFFUL << PMUC_PMU_RSVD_RESERVE1_Pos)
#define PMUC_PMU_RSVD_RESERVE1          PMUC_PMU_RSVD_RESERVE1_Msk
#define PMUC_PMU_RSVD_RESERVE2_Pos      (16U)
#define PMUC_PMU_RSVD_RESERVE2_Msk      (0xFFUL << PMUC_PMU_RSVD_RESERVE2_Pos)
#define PMUC_PMU_RSVD_RESERVE2          PMUC_PMU_RSVD_RESERVE2_Msk
#define PMUC_PMU_RSVD_RESERVE3_Pos      (24U)
#define PMUC_PMU_RSVD_RESERVE3_Msk      (0xFFUL << PMUC_PMU_RSVD_RESERVE3_Pos)
#define PMUC_PMU_RSVD_RESERVE3          PMUC_PMU_RSVD_RESERVE3_Msk

/****************** Bit definition for PMUC_HXT_CR1 register ******************/
#define PMUC_HXT_CR1_EN_Pos             (0U)
#define PMUC_HXT_CR1_EN_Msk             (0x1UL << PMUC_HXT_CR1_EN_Pos)
#define PMUC_HXT_CR1_EN                 PMUC_HXT_CR1_EN_Msk
#define PMUC_HXT_CR1_BUF_EN_Pos         (1U)
#define PMUC_HXT_CR1_BUF_EN_Msk         (0x1UL << PMUC_HXT_CR1_BUF_EN_Pos)
#define PMUC_HXT_CR1_BUF_EN             PMUC_HXT_CR1_BUF_EN_Msk
#define PMUC_HXT_CR1_BUF_DIG_EN_Pos     (2U)
#define PMUC_HXT_CR1_BUF_DIG_EN_Msk     (0x1UL << PMUC_HXT_CR1_BUF_DIG_EN_Pos)
#define PMUC_HXT_CR1_BUF_DIG_EN         PMUC_HXT_CR1_BUF_DIG_EN_Msk
#define PMUC_HXT_CR1_BUF_DIG_STR_Pos    (3U)
#define PMUC_HXT_CR1_BUF_DIG_STR_Msk    (0x3UL << PMUC_HXT_CR1_BUF_DIG_STR_Pos)
#define PMUC_HXT_CR1_BUF_DIG_STR        PMUC_HXT_CR1_BUF_DIG_STR_Msk
#define PMUC_HXT_CR1_BUF_DLL_EN_Pos     (5U)
#define PMUC_HXT_CR1_BUF_DLL_EN_Msk     (0x1UL << PMUC_HXT_CR1_BUF_DLL_EN_Pos)
#define PMUC_HXT_CR1_BUF_DLL_EN         PMUC_HXT_CR1_BUF_DLL_EN_Msk
#define PMUC_HXT_CR1_BUF_DLL_STR_Pos    (6U)
#define PMUC_HXT_CR1_BUF_DLL_STR_Msk    (0x3UL << PMUC_HXT_CR1_BUF_DLL_STR_Pos)
#define PMUC_HXT_CR1_BUF_DLL_STR        PMUC_HXT_CR1_BUF_DLL_STR_Msk
#define PMUC_HXT_CR1_BUF_AUD_EN_Pos     (8U)
#define PMUC_HXT_CR1_BUF_AUD_EN_Msk     (0x1UL << PMUC_HXT_CR1_BUF_AUD_EN_Pos)
#define PMUC_HXT_CR1_BUF_AUD_EN         PMUC_HXT_CR1_BUF_AUD_EN_Msk
#define PMUC_HXT_CR1_BUF_AUD_STR_Pos    (9U)
#define PMUC_HXT_CR1_BUF_AUD_STR_Msk    (0x3UL << PMUC_HXT_CR1_BUF_AUD_STR_Pos)
#define PMUC_HXT_CR1_BUF_AUD_STR        PMUC_HXT_CR1_BUF_AUD_STR_Msk
#define PMUC_HXT_CR1_BUF_RF_STR_Pos     (11U)
#define PMUC_HXT_CR1_BUF_RF_STR_Msk     (0x3UL << PMUC_HXT_CR1_BUF_RF_STR_Pos)
#define PMUC_HXT_CR1_BUF_RF_STR         PMUC_HXT_CR1_BUF_RF_STR_Msk
#define PMUC_HXT_CR1_LDO_VREF_Pos       (13U)
#define PMUC_HXT_CR1_LDO_VREF_Msk       (0xFUL << PMUC_HXT_CR1_LDO_VREF_Pos)
#define PMUC_HXT_CR1_LDO_VREF           PMUC_HXT_CR1_LDO_VREF_Msk
#define PMUC_HXT_CR1_LDO_FLT_RSEL_Pos   (17U)
#define PMUC_HXT_CR1_LDO_FLT_RSEL_Msk   (0x3UL << PMUC_HXT_CR1_LDO_FLT_RSEL_Pos)
#define PMUC_HXT_CR1_LDO_FLT_RSEL       PMUC_HXT_CR1_LDO_FLT_RSEL_Msk
#define PMUC_HXT_CR1_GM_EN_Pos          (19U)
#define PMUC_HXT_CR1_GM_EN_Msk          (0x1UL << PMUC_HXT_CR1_GM_EN_Pos)
#define PMUC_HXT_CR1_GM_EN              PMUC_HXT_CR1_GM_EN_Msk
#define PMUC_HXT_CR1_CBANK_SEL_Pos      (20U)
#define PMUC_HXT_CR1_CBANK_SEL_Msk      (0x3FFUL << PMUC_HXT_CR1_CBANK_SEL_Pos)
#define PMUC_HXT_CR1_CBANK_SEL          PMUC_HXT_CR1_CBANK_SEL_Msk

/****************** Bit definition for PMUC_HXT_CR2 register ******************/
#define PMUC_HXT_CR2_AGC_EN_Pos         (0U)
#define PMUC_HXT_CR2_AGC_EN_Msk         (0x1UL << PMUC_HXT_CR2_AGC_EN_Pos)
#define PMUC_HXT_CR2_AGC_EN             PMUC_HXT_CR2_AGC_EN_Msk
#define PMUC_HXT_CR2_AGC_ISTART_SEL_Pos  (1U)
#define PMUC_HXT_CR2_AGC_ISTART_SEL_Msk  (0x1UL << PMUC_HXT_CR2_AGC_ISTART_SEL_Pos)
#define PMUC_HXT_CR2_AGC_ISTART_SEL     PMUC_HXT_CR2_AGC_ISTART_SEL_Msk
#define PMUC_HXT_CR2_AGC_VTH_Pos        (2U)
#define PMUC_HXT_CR2_AGC_VTH_Msk        (0xFUL << PMUC_HXT_CR2_AGC_VTH_Pos)
#define PMUC_HXT_CR2_AGC_VTH            PMUC_HXT_CR2_AGC_VTH_Msk
#define PMUC_HXT_CR2_AGC_VINDC_Pos      (6U)
#define PMUC_HXT_CR2_AGC_VINDC_Msk      (0x3UL << PMUC_HXT_CR2_AGC_VINDC_Pos)
#define PMUC_HXT_CR2_AGC_VINDC          PMUC_HXT_CR2_AGC_VINDC_Msk
#define PMUC_HXT_CR2_ACBUF_SEL_Pos      (8U)
#define PMUC_HXT_CR2_ACBUF_SEL_Msk      (0x3UL << PMUC_HXT_CR2_ACBUF_SEL_Pos)
#define PMUC_HXT_CR2_ACBUF_SEL          PMUC_HXT_CR2_ACBUF_SEL_Msk
#define PMUC_HXT_CR2_ACBUF_RSEL_Pos     (10U)
#define PMUC_HXT_CR2_ACBUF_RSEL_Msk     (0x1UL << PMUC_HXT_CR2_ACBUF_RSEL_Pos)
#define PMUC_HXT_CR2_ACBUF_RSEL         PMUC_HXT_CR2_ACBUF_RSEL_Msk
#define PMUC_HXT_CR2_BUF_SEL2_Pos       (11U)
#define PMUC_HXT_CR2_BUF_SEL2_Msk       (0x3UL << PMUC_HXT_CR2_BUF_SEL2_Pos)
#define PMUC_HXT_CR2_BUF_SEL2           PMUC_HXT_CR2_BUF_SEL2_Msk
#define PMUC_HXT_CR2_BUF_SEL3_Pos       (13U)
#define PMUC_HXT_CR2_BUF_SEL3_Msk       (0x3UL << PMUC_HXT_CR2_BUF_SEL3_Pos)
#define PMUC_HXT_CR2_BUF_SEL3           PMUC_HXT_CR2_BUF_SEL3_Msk
#define PMUC_HXT_CR2_IDAC_EN_Pos        (15U)
#define PMUC_HXT_CR2_IDAC_EN_Msk        (0x1UL << PMUC_HXT_CR2_IDAC_EN_Pos)
#define PMUC_HXT_CR2_IDAC_EN            PMUC_HXT_CR2_IDAC_EN_Msk
#define PMUC_HXT_CR2_IDAC_Pos           (16U)
#define PMUC_HXT_CR2_IDAC_Msk           (0x3FFUL << PMUC_HXT_CR2_IDAC_Pos)
#define PMUC_HXT_CR2_IDAC               PMUC_HXT_CR2_IDAC_Msk
#define PMUC_HXT_CR2_SDADC_CLKIN_EN_Pos  (26U)
#define PMUC_HXT_CR2_SDADC_CLKIN_EN_Msk  (0x1UL << PMUC_HXT_CR2_SDADC_CLKIN_EN_Pos)
#define PMUC_HXT_CR2_SDADC_CLKIN_EN     PMUC_HXT_CR2_SDADC_CLKIN_EN_Msk
#define PMUC_HXT_CR2_SDADC_CLKDIV1_SEL_Pos  (27U)
#define PMUC_HXT_CR2_SDADC_CLKDIV1_SEL_Msk  (0x3UL << PMUC_HXT_CR2_SDADC_CLKDIV1_SEL_Pos)
#define PMUC_HXT_CR2_SDADC_CLKDIV1_SEL  PMUC_HXT_CR2_SDADC_CLKDIV1_SEL_Msk
#define PMUC_HXT_CR2_SDADC_CLKDIV2_SEL_Pos  (29U)
#define PMUC_HXT_CR2_SDADC_CLKDIV2_SEL_Msk  (0x3UL << PMUC_HXT_CR2_SDADC_CLKDIV2_SEL_Pos)
#define PMUC_HXT_CR2_SDADC_CLKDIV2_SEL  PMUC_HXT_CR2_SDADC_CLKDIV2_SEL_Msk
#define PMUC_HXT_CR2_SLEEP_EN_Pos       (31U)
#define PMUC_HXT_CR2_SLEEP_EN_Msk       (0x1UL << PMUC_HXT_CR2_SLEEP_EN_Pos)
#define PMUC_HXT_CR2_SLEEP_EN           PMUC_HXT_CR2_SLEEP_EN_Msk

/****************** Bit definition for PMUC_HXT_CR3 register ******************/
#define PMUC_HXT_CR3_BUF_DAC_STR_Pos    (0U)
#define PMUC_HXT_CR3_BUF_DAC_STR_Msk    (0x3UL << PMUC_HXT_CR3_BUF_DAC_STR_Pos)
#define PMUC_HXT_CR3_BUF_DAC_STR        PMUC_HXT_CR3_BUF_DAC_STR_Msk
#define PMUC_HXT_CR3_BUF_OSLO_STR_Pos   (2U)
#define PMUC_HXT_CR3_BUF_OSLO_STR_Msk   (0x3UL << PMUC_HXT_CR3_BUF_OSLO_STR_Pos)
#define PMUC_HXT_CR3_BUF_OSLO_STR       PMUC_HXT_CR3_BUF_OSLO_STR_Msk
#define PMUC_HXT_CR3_DLY_Pos            (4U)
#define PMUC_HXT_CR3_DLY_Msk            (0x3FUL << PMUC_HXT_CR3_DLY_Pos)
#define PMUC_HXT_CR3_DLY                PMUC_HXT_CR3_DLY_Msk

/****************** Bit definition for PMUC_HRC_CR register *******************/
#define PMUC_HRC_CR_EN_Pos              (0U)
#define PMUC_HRC_CR_EN_Msk              (0x1UL << PMUC_HRC_CR_EN_Pos)
#define PMUC_HRC_CR_EN                  PMUC_HRC_CR_EN_Msk
#define PMUC_HRC_CR_LDO_VREF_Pos        (1U)
#define PMUC_HRC_CR_LDO_VREF_Msk        (0xFUL << PMUC_HRC_CR_LDO_VREF_Pos)
#define PMUC_HRC_CR_LDO_VREF            PMUC_HRC_CR_LDO_VREF_Msk
#define PMUC_HRC_CR_FREQ_TRIM_Pos       (5U)
#define PMUC_HRC_CR_FREQ_TRIM_Msk       (0x3FFUL << PMUC_HRC_CR_FREQ_TRIM_Pos)
#define PMUC_HRC_CR_FREQ_TRIM           PMUC_HRC_CR_FREQ_TRIM_Msk
#define PMUC_HRC_CR_TEMP_TRIM_Pos       (15U)
#define PMUC_HRC_CR_TEMP_TRIM_Msk       (0x7UL << PMUC_HRC_CR_TEMP_TRIM_Pos)
#define PMUC_HRC_CR_TEMP_TRIM           PMUC_HRC_CR_TEMP_TRIM_Msk
#define PMUC_HRC_CR_CLK96M_EN_Pos       (18U)
#define PMUC_HRC_CR_CLK96M_EN_Msk       (0x1UL << PMUC_HRC_CR_CLK96M_EN_Pos)
#define PMUC_HRC_CR_CLK96M_EN           PMUC_HRC_CR_CLK96M_EN_Msk
#define PMUC_HRC_CR_CLKHP_EN_Pos        (20U)
#define PMUC_HRC_CR_CLKHP_EN_Msk        (0x1UL << PMUC_HRC_CR_CLKHP_EN_Pos)
#define PMUC_HRC_CR_CLKHP_EN            PMUC_HRC_CR_CLKHP_EN_Msk
#define PMUC_HRC_CR_CLKHP_SEL_Pos       (21U)
#define PMUC_HRC_CR_CLKHP_SEL_Msk       (0x3UL << PMUC_HRC_CR_CLKHP_SEL_Pos)
#define PMUC_HRC_CR_CLKHP_SEL           PMUC_HRC_CR_CLKHP_SEL_Msk
#define PMUC_HRC_CR_CLKHP_STR_Pos       (23U)
#define PMUC_HRC_CR_CLKHP_STR_Msk       (0x3UL << PMUC_HRC_CR_CLKHP_STR_Pos)
#define PMUC_HRC_CR_CLKHP_STR           PMUC_HRC_CR_CLKHP_STR_Msk
#define PMUC_HRC_CR_CLKLP_EN_Pos        (25U)
#define PMUC_HRC_CR_CLKLP_EN_Msk        (0x1UL << PMUC_HRC_CR_CLKLP_EN_Pos)
#define PMUC_HRC_CR_CLKLP_EN            PMUC_HRC_CR_CLKLP_EN_Msk
#define PMUC_HRC_CR_CLKLP_SEL_Pos       (26U)
#define PMUC_HRC_CR_CLKLP_SEL_Msk       (0x3UL << PMUC_HRC_CR_CLKLP_SEL_Pos)
#define PMUC_HRC_CR_CLKLP_SEL           PMUC_HRC_CR_CLKLP_SEL_Msk
#define PMUC_HRC_CR_CLKLP_STR_Pos       (28U)
#define PMUC_HRC_CR_CLKLP_STR_Msk       (0x3UL << PMUC_HRC_CR_CLKLP_STR_Pos)
#define PMUC_HRC_CR_CLKLP_STR           PMUC_HRC_CR_CLKLP_STR_Msk
#define PMUC_HRC_CR_DLY_Pos             (31U)
#define PMUC_HRC_CR_DLY_Msk             (0x1UL << PMUC_HRC_CR_DLY_Pos)
#define PMUC_HRC_CR_DLY                 PMUC_HRC_CR_DLY_Msk

/***************** Bit definition for PMUC_DBL96_CR register ******************/
#define PMUC_DBL96_CR_EN_Pos            (0U)
#define PMUC_DBL96_CR_EN_Msk            (0x1UL << PMUC_DBL96_CR_EN_Pos)
#define PMUC_DBL96_CR_EN                PMUC_DBL96_CR_EN_Msk
#define PMUC_DBL96_CR_OUT_EN_Pos        (1U)
#define PMUC_DBL96_CR_OUT_EN_Msk        (0x1UL << PMUC_DBL96_CR_OUT_EN_Pos)
#define PMUC_DBL96_CR_OUT_EN            PMUC_DBL96_CR_OUT_EN_Msk
#define PMUC_DBL96_CR_TODIG_EN_Pos      (2U)
#define PMUC_DBL96_CR_TODIG_EN_Msk      (0x1UL << PMUC_DBL96_CR_TODIG_EN_Pos)
#define PMUC_DBL96_CR_TODIG_EN          PMUC_DBL96_CR_TODIG_EN_Msk
#define PMUC_DBL96_CR_TODIG_STR_Pos     (3U)
#define PMUC_DBL96_CR_TODIG_STR_Msk     (0x3UL << PMUC_DBL96_CR_TODIG_STR_Pos)
#define PMUC_DBL96_CR_TODIG_STR         PMUC_DBL96_CR_TODIG_STR_Msk
#define PMUC_DBL96_CR_TORF_EN_Pos       (5U)
#define PMUC_DBL96_CR_TORF_EN_Msk       (0x1UL << PMUC_DBL96_CR_TORF_EN_Pos)
#define PMUC_DBL96_CR_TORF_EN           PMUC_DBL96_CR_TORF_EN_Msk
#define PMUC_DBL96_CR_TOOSLO_EN_Pos     (6U)
#define PMUC_DBL96_CR_TOOSLO_EN_Msk     (0x1UL << PMUC_DBL96_CR_TOOSLO_EN_Pos)
#define PMUC_DBL96_CR_TOOSLO_EN         PMUC_DBL96_CR_TOOSLO_EN_Msk
#define PMUC_DBL96_CR_LOOP_RSTB_Pos     (7U)
#define PMUC_DBL96_CR_LOOP_RSTB_Msk     (0x1UL << PMUC_DBL96_CR_LOOP_RSTB_Pos)
#define PMUC_DBL96_CR_LOOP_RSTB         PMUC_DBL96_CR_LOOP_RSTB_Msk
#define PMUC_DBL96_CR_PH_EN_Pos         (8U)
#define PMUC_DBL96_CR_PH_EN_Msk         (0xFUL << PMUC_DBL96_CR_PH_EN_Pos)
#define PMUC_DBL96_CR_PH_EN             PMUC_DBL96_CR_PH_EN_Msk
#define PMUC_DBL96_CR_DLY_EN_Pos        (12U)
#define PMUC_DBL96_CR_DLY_EN_Msk        (0xFUL << PMUC_DBL96_CR_DLY_EN_Pos)
#define PMUC_DBL96_CR_DLY_EN            PMUC_DBL96_CR_DLY_EN_Msk
#define PMUC_DBL96_CR_DLY_EXT_EN_Pos    (16U)
#define PMUC_DBL96_CR_DLY_EXT_EN_Msk    (0x1UL << PMUC_DBL96_CR_DLY_EXT_EN_Pos)
#define PMUC_DBL96_CR_DLY_EXT_EN        PMUC_DBL96_CR_DLY_EXT_EN_Msk
#define PMUC_DBL96_CR_DLY_SEL_EXT_EN_Pos  (17U)
#define PMUC_DBL96_CR_DLY_SEL_EXT_EN_Msk  (0x1UL << PMUC_DBL96_CR_DLY_SEL_EXT_EN_Pos)
#define PMUC_DBL96_CR_DLY_SEL_EXT_EN    PMUC_DBL96_CR_DLY_SEL_EXT_EN_Msk
#define PMUC_DBL96_CR_DLY_SEL_EXT_Pos   (18U)
#define PMUC_DBL96_CR_DLY_SEL_EXT_Msk   (0x7FFUL << PMUC_DBL96_CR_DLY_SEL_EXT_Pos)
#define PMUC_DBL96_CR_DLY_SEL_EXT       PMUC_DBL96_CR_DLY_SEL_EXT_Msk

/**************** Bit definition for PMUC_DBL96_CALR register *****************/
#define PMUC_DBL96_CALR_CAL_EN_Pos      (0U)
#define PMUC_DBL96_CALR_CAL_EN_Msk      (0x1UL << PMUC_DBL96_CALR_CAL_EN_Pos)
#define PMUC_DBL96_CALR_CAL_EN          PMUC_DBL96_CALR_CAL_EN_Msk
#define PMUC_DBL96_CALR_CAL_CLOSE_EXT_EN_Pos  (1U)
#define PMUC_DBL96_CALR_CAL_CLOSE_EXT_EN_Msk  (0x1UL << PMUC_DBL96_CALR_CAL_CLOSE_EXT_EN_Pos)
#define PMUC_DBL96_CALR_CAL_CLOSE_EXT_EN  PMUC_DBL96_CALR_CAL_CLOSE_EXT_EN_Msk
#define PMUC_DBL96_CALR_CAL_OP_Pos      (2U)
#define PMUC_DBL96_CALR_CAL_OP_Msk      (0x7FFUL << PMUC_DBL96_CALR_CAL_OP_Pos)
#define PMUC_DBL96_CALR_CAL_OP          PMUC_DBL96_CALR_CAL_OP_Msk
#define PMUC_DBL96_CALR_CAL_LOCK_Pos    (13U)
#define PMUC_DBL96_CALR_CAL_LOCK_Msk    (0x1UL << PMUC_DBL96_CALR_CAL_LOCK_Pos)
#define PMUC_DBL96_CALR_CAL_LOCK        PMUC_DBL96_CALR_CAL_LOCK_Msk

/****************** Bit definition for PMUC_CAU_BGR register ******************/
#define PMUC_CAU_BGR_HPBG_VDDPSW_EN_Pos  (0U)
#define PMUC_CAU_BGR_HPBG_VDDPSW_EN_Msk  (0x1UL << PMUC_CAU_BGR_HPBG_VDDPSW_EN_Pos)
#define PMUC_CAU_BGR_HPBG_VDDPSW_EN     PMUC_CAU_BGR_HPBG_VDDPSW_EN_Msk
#define PMUC_CAU_BGR_HPBG_EN_Pos        (1U)
#define PMUC_CAU_BGR_HPBG_EN_Msk        (0x1UL << PMUC_CAU_BGR_HPBG_EN_Pos)
#define PMUC_CAU_BGR_HPBG_EN            PMUC_CAU_BGR_HPBG_EN_Msk
#define PMUC_CAU_BGR_LPBG_EN_Pos        (2U)
#define PMUC_CAU_BGR_LPBG_EN_Msk        (0x1UL << PMUC_CAU_BGR_LPBG_EN_Pos)
#define PMUC_CAU_BGR_LPBG_EN            PMUC_CAU_BGR_LPBG_EN_Msk
#define PMUC_CAU_BGR_LPBG_VREF06_Pos    (3U)
#define PMUC_CAU_BGR_LPBG_VREF06_Msk    (0xFUL << PMUC_CAU_BGR_LPBG_VREF06_Pos)
#define PMUC_CAU_BGR_LPBG_VREF06        PMUC_CAU_BGR_LPBG_VREF06_Msk
#define PMUC_CAU_BGR_LPBG_VREF12_Pos    (7U)
#define PMUC_CAU_BGR_LPBG_VREF12_Msk    (0xFUL << PMUC_CAU_BGR_LPBG_VREF12_Pos)
#define PMUC_CAU_BGR_LPBG_VREF12        PMUC_CAU_BGR_LPBG_VREF12_Msk

/****************** Bit definition for PMUC_CAU_TR register *******************/
#define PMUC_CAU_TR_CAU_DC_TR_Pos       (0U)
#define PMUC_CAU_TR_CAU_DC_TR_Msk       (0x7UL << PMUC_CAU_TR_CAU_DC_TR_Pos)
#define PMUC_CAU_TR_CAU_DC_TR           PMUC_CAU_TR_CAU_DC_TR_Msk
#define PMUC_CAU_TR_CAU_DC_BR_Pos       (3U)
#define PMUC_CAU_TR_CAU_DC_BR_Msk       (0x7UL << PMUC_CAU_TR_CAU_DC_BR_Pos)
#define PMUC_CAU_TR_CAU_DC_BR           PMUC_CAU_TR_CAU_DC_BR_Msk
#define PMUC_CAU_TR_CAU_DC_MR_Pos       (6U)
#define PMUC_CAU_TR_CAU_DC_MR_Msk       (0x7UL << PMUC_CAU_TR_CAU_DC_MR_Pos)
#define PMUC_CAU_TR_CAU_DC_MR           PMUC_CAU_TR_CAU_DC_MR_Msk

/***************** Bit definition for PMUC_CAU_RSVD register ******************/
#define PMUC_CAU_RSVD_RESERVE0_Pos      (0U)
#define PMUC_CAU_RSVD_RESERVE0_Msk      (0xFFUL << PMUC_CAU_RSVD_RESERVE0_Pos)
#define PMUC_CAU_RSVD_RESERVE0          PMUC_CAU_RSVD_RESERVE0_Msk
#define PMUC_CAU_RSVD_RESERVE1_Pos      (8U)
#define PMUC_CAU_RSVD_RESERVE1_Msk      (0xFFUL << PMUC_CAU_RSVD_RESERVE1_Pos)
#define PMUC_CAU_RSVD_RESERVE1          PMUC_CAU_RSVD_RESERVE1_Msk
#define PMUC_CAU_RSVD_RESERVE2_Pos      (16U)
#define PMUC_CAU_RSVD_RESERVE2_Msk      (0xFFUL << PMUC_CAU_RSVD_RESERVE2_Pos)
#define PMUC_CAU_RSVD_RESERVE2          PMUC_CAU_RSVD_RESERVE2_Msk

/***************** Bit definition for PMUC_WKUP_CNT register ******************/
#define PMUC_WKUP_CNT_PIN0_CNT_Pos      (0U)
#define PMUC_WKUP_CNT_PIN0_CNT_Msk      (0xFFFFUL << PMUC_WKUP_CNT_PIN0_CNT_Pos)
#define PMUC_WKUP_CNT_PIN0_CNT          PMUC_WKUP_CNT_PIN0_CNT_Msk
#define PMUC_WKUP_CNT_PIN1_CNT_Pos      (16U)
#define PMUC_WKUP_CNT_PIN1_CNT_Msk      (0xFFFFUL << PMUC_WKUP_CNT_PIN1_CNT_Pos)
#define PMUC_WKUP_CNT_PIN1_CNT          PMUC_WKUP_CNT_PIN1_CNT_Msk

/**************** Bit definition for PMUC_PWRKEY_CNT register *****************/
#define PMUC_PWRKEY_CNT_RST_CNT_Pos     (4U)
#define PMUC_PWRKEY_CNT_RST_CNT_Msk     (0xFFFFUL << PMUC_PWRKEY_CNT_RST_CNT_Pos)
#define PMUC_PWRKEY_CNT_RST_CNT         PMUC_PWRKEY_CNT_RST_CNT_Msk

/**************** Bit definition for PMUC_HPSYS_VOUT register *****************/
#define PMUC_HPSYS_VOUT_VOUT_Pos        (0U)
#define PMUC_HPSYS_VOUT_VOUT_Msk        (0xFUL << PMUC_HPSYS_VOUT_VOUT_Pos)
#define PMUC_HPSYS_VOUT_VOUT            PMUC_HPSYS_VOUT_VOUT_Msk

/**************** Bit definition for PMUC_LPSYS_VOUT register *****************/
#define PMUC_LPSYS_VOUT_VOUT_Pos        (0U)
#define PMUC_LPSYS_VOUT_VOUT_Msk        (0xFUL << PMUC_LPSYS_VOUT_VOUT_Pos)
#define PMUC_LPSYS_VOUT_VOUT            PMUC_LPSYS_VOUT_VOUT_Msk

/***************** Bit definition for PMUC_BUCK_VOUT register *****************/
#define PMUC_BUCK_VOUT_VOUT_Pos         (0U)
#define PMUC_BUCK_VOUT_VOUT_Msk         (0xFUL << PMUC_BUCK_VOUT_VOUT_Pos)
#define PMUC_BUCK_VOUT_VOUT             PMUC_BUCK_VOUT_VOUT_Msk

#endif
