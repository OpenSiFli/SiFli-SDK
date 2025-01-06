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
    __IO uint32_t LRC_CR;
    __IO uint32_t LXT_CR;
    __IO uint32_t BG1_CR;
    __IO uint32_t BG2_CR;
    __IO uint32_t BUCK1_CR;
    __IO uint32_t BUCK2_CR;
    __IO uint32_t LDO_CR;
    __IO uint32_t HPSYS_SWR;
    __IO uint32_t LPSYS_SWR;
    __IO uint32_t LPCOMP_CR;
    __IO uint32_t PMU_TR;
    __IO uint32_t PMU_RSVD1;
    __IO uint32_t PMU_RSVD2;
    __IO uint32_t HXT_CR1;
    __IO uint32_t HXT_CR2;
    __IO uint32_t HXT_CR3;
    __IO uint32_t HRC_CR;
    __IO uint32_t RC1M_CR;
    __IO uint32_t DBL96_CR;
    __IO uint32_t DBL96_CALR;
    __IO uint32_t CAU_BGR;
    __IO uint32_t CAU_TR;
    __IO uint32_t CAU_RSVD;
    __IO uint32_t TSTIF_CR;
    __IO uint32_t HFOSC_CR;
    __IO uint32_t ULPVR_CR1;
    __IO uint32_t ULPVR_CR2;
    __IO uint32_t ULPVR_CR3;
    __IO uint32_t ULPVR_CR4;
    __IO uint32_t ULPVR_CR5;
    __IO uint32_t WKUP_CNT;
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
#define PMUC_CR_PIN0_MODE_Pos           (4U)
#define PMUC_CR_PIN0_MODE_Msk           (0x7UL << PMUC_CR_PIN0_MODE_Pos)
#define PMUC_CR_PIN0_MODE               PMUC_CR_PIN0_MODE_Msk
#define PMUC_CR_PIN1_MODE_Pos           (7U)
#define PMUC_CR_PIN1_MODE_Msk           (0x7UL << PMUC_CR_PIN1_MODE_Pos)
#define PMUC_CR_PIN1_MODE               PMUC_CR_PIN1_MODE_Msk
#define PMUC_CR_PIN0_SEL_Pos            (10U)
#define PMUC_CR_PIN0_SEL_Msk            (0xFUL << PMUC_CR_PIN0_SEL_Pos)
#define PMUC_CR_PIN0_SEL                PMUC_CR_PIN0_SEL_Msk
#define PMUC_CR_PIN1_SEL_Pos            (14U)
#define PMUC_CR_PIN1_SEL_Msk            (0xFUL << PMUC_CR_PIN1_SEL_Pos)
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
#define PMUC_VRET_CR_TRIM_SEL_Pos       (14U)
#define PMUC_VRET_CR_TRIM_SEL_Msk       (0x1UL << PMUC_VRET_CR_TRIM_SEL_Pos)
#define PMUC_VRET_CR_TRIM_SEL           PMUC_VRET_CR_TRIM_SEL_Msk
#define PMUC_VRET_CR_TRIM_RSTN_Pos      (15U)
#define PMUC_VRET_CR_TRIM_RSTN_Msk      (0x1UL << PMUC_VRET_CR_TRIM_RSTN_Pos)
#define PMUC_VRET_CR_TRIM_RSTN          PMUC_VRET_CR_TRIM_RSTN_Msk
#define PMUC_VRET_CR_CAL_RDY_Pos        (16U)
#define PMUC_VRET_CR_CAL_RDY_Msk        (0x1UL << PMUC_VRET_CR_CAL_RDY_Pos)
#define PMUC_VRET_CR_CAL_RDY            PMUC_VRET_CR_CAL_RDY_Msk
#define PMUC_VRET_CR_CAL_TRIM_Pos       (17U)
#define PMUC_VRET_CR_CAL_TRIM_Msk       (0xFUL << PMUC_VRET_CR_CAL_TRIM_Pos)
#define PMUC_VRET_CR_CAL_TRIM           PMUC_VRET_CR_CAL_TRIM_Msk
#define PMUC_VRET_CR_DLY_Pos            (21U)
#define PMUC_VRET_CR_DLY_Msk            (0x3FUL << PMUC_VRET_CR_DLY_Pos)
#define PMUC_VRET_CR_DLY                PMUC_VRET_CR_DLY_Msk
#define PMUC_VRET_CR_RDY_Pos            (31U)
#define PMUC_VRET_CR_RDY_Msk            (0x1UL << PMUC_VRET_CR_RDY_Pos)
#define PMUC_VRET_CR_RDY                PMUC_VRET_CR_RDY_Msk

/****************** Bit definition for PMUC_LRC_CR register *******************/
#define PMUC_LRC_CR_EN_Pos              (0U)
#define PMUC_LRC_CR_EN_Msk              (0x1UL << PMUC_LRC_CR_EN_Pos)
#define PMUC_LRC_CR_EN                  PMUC_LRC_CR_EN_Msk
#define PMUC_LRC_CR_CMPBM1_Pos          (1U)
#define PMUC_LRC_CR_CMPBM1_Msk          (0x3UL << PMUC_LRC_CR_CMPBM1_Pos)
#define PMUC_LRC_CR_CMPBM1              PMUC_LRC_CR_CMPBM1_Msk
#define PMUC_LRC_CR_CMPBM2_Pos          (3U)
#define PMUC_LRC_CR_CMPBM2_Msk          (0x1UL << PMUC_LRC_CR_CMPBM2_Pos)
#define PMUC_LRC_CR_CMPBM2              PMUC_LRC_CR_CMPBM2_Msk
#define PMUC_LRC_CR_CHGCRT_Pos          (4U)
#define PMUC_LRC_CR_CHGCRT_Msk          (0x3UL << PMUC_LRC_CR_CHGCRT_Pos)
#define PMUC_LRC_CR_CHGCRT              PMUC_LRC_CR_CHGCRT_Msk
#define PMUC_LRC_CR_CHGCAP_Pos          (6U)
#define PMUC_LRC_CR_CHGCAP_Msk          (0x3UL << PMUC_LRC_CR_CHGCAP_Pos)
#define PMUC_LRC_CR_CHGCAP              PMUC_LRC_CR_CHGCAP_Msk
#define PMUC_LRC_CR_REFRES_Pos          (8U)
#define PMUC_LRC_CR_REFRES_Msk          (0x1UL << PMUC_LRC_CR_REFRES_Pos)
#define PMUC_LRC_CR_REFRES              PMUC_LRC_CR_REFRES_Msk

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
#define PMUC_LXT_CR_RDY_Pos             (31U)
#define PMUC_LXT_CR_RDY_Msk             (0x1UL << PMUC_LXT_CR_RDY_Pos)
#define PMUC_LXT_CR_RDY                 PMUC_LXT_CR_RDY_Msk

/****************** Bit definition for PMUC_BG1_CR register *******************/
#define PMUC_BG1_CR_BG1_EN_Pos          (0U)
#define PMUC_BG1_CR_BG1_EN_Msk          (0x1UL << PMUC_BG1_CR_BG1_EN_Pos)
#define PMUC_BG1_CR_BG1_EN              PMUC_BG1_CR_BG1_EN_Msk
#define PMUC_BG1_CR_BG1_VREF06_Pos      (1U)
#define PMUC_BG1_CR_BG1_VREF06_Msk      (0xFUL << PMUC_BG1_CR_BG1_VREF06_Pos)
#define PMUC_BG1_CR_BG1_VREF06          PMUC_BG1_CR_BG1_VREF06_Msk
#define PMUC_BG1_CR_BG1_VREF12_Pos      (5U)
#define PMUC_BG1_CR_BG1_VREF12_Msk      (0xFUL << PMUC_BG1_CR_BG1_VREF12_Pos)
#define PMUC_BG1_CR_BG1_VREF12          PMUC_BG1_CR_BG1_VREF12_Msk
#define PMUC_BG1_CR_BG1_DLY_Pos         (9U)
#define PMUC_BG1_CR_BG1_DLY_Msk         (0x3UL << PMUC_BG1_CR_BG1_DLY_Pos)
#define PMUC_BG1_CR_BG1_DLY             PMUC_BG1_CR_BG1_DLY_Msk

/****************** Bit definition for PMUC_BG2_CR register *******************/
#define PMUC_BG2_CR_BG2_EN_Pos          (0U)
#define PMUC_BG2_CR_BG2_EN_Msk          (0x1UL << PMUC_BG2_CR_BG2_EN_Pos)
#define PMUC_BG2_CR_BG2_EN              PMUC_BG2_CR_BG2_EN_Msk
#define PMUC_BG2_CR_BG2_VREF06_Pos      (1U)
#define PMUC_BG2_CR_BG2_VREF06_Msk      (0xFUL << PMUC_BG2_CR_BG2_VREF06_Pos)
#define PMUC_BG2_CR_BG2_VREF06          PMUC_BG2_CR_BG2_VREF06_Msk
#define PMUC_BG2_CR_BG2_VREF12_Pos      (5U)
#define PMUC_BG2_CR_BG2_VREF12_Msk      (0xFUL << PMUC_BG2_CR_BG2_VREF12_Pos)
#define PMUC_BG2_CR_BG2_VREF12          PMUC_BG2_CR_BG2_VREF12_Msk
#define PMUC_BG2_CR_BG2_DLY_Pos         (9U)
#define PMUC_BG2_CR_BG2_DLY_Msk         (0x3UL << PMUC_BG2_CR_BG2_DLY_Pos)
#define PMUC_BG2_CR_BG2_DLY             PMUC_BG2_CR_BG2_DLY_Msk

/***************** Bit definition for PMUC_BUCK1_CR register ******************/
#define PMUC_BUCK1_CR_BUCK1_EN_Pos      (0U)
#define PMUC_BUCK1_CR_BUCK1_EN_Msk      (0x1UL << PMUC_BUCK1_CR_BUCK1_EN_Pos)
#define PMUC_BUCK1_CR_BUCK1_EN          PMUC_BUCK1_CR_BUCK1_EN_Msk
#define PMUC_BUCK1_CR_BUCK1_ILIMIT_Pos  (1U)
#define PMUC_BUCK1_CR_BUCK1_ILIMIT_Msk  (0x7UL << PMUC_BUCK1_CR_BUCK1_ILIMIT_Pos)
#define PMUC_BUCK1_CR_BUCK1_ILIMIT      PMUC_BUCK1_CR_BUCK1_ILIMIT_Msk
#define PMUC_BUCK1_CR_BUCK1_CCH_Pos     (4U)
#define PMUC_BUCK1_CR_BUCK1_CCH_Msk     (0xFUL << PMUC_BUCK1_CR_BUCK1_CCH_Pos)
#define PMUC_BUCK1_CR_BUCK1_CCH         PMUC_BUCK1_CR_BUCK1_CCH_Msk
#define PMUC_BUCK1_CR_BUCK1_CS_Pos      (8U)
#define PMUC_BUCK1_CR_BUCK1_CS_Msk      (0xFUL << PMUC_BUCK1_CR_BUCK1_CS_Pos)
#define PMUC_BUCK1_CR_BUCK1_CS          PMUC_BUCK1_CR_BUCK1_CS_Msk
#define PMUC_BUCK1_CR_BUCK1_SEL_LX22_Pos  (12U)
#define PMUC_BUCK1_CR_BUCK1_SEL_LX22_Msk  (0x1UL << PMUC_BUCK1_CR_BUCK1_SEL_LX22_Pos)
#define PMUC_BUCK1_CR_BUCK1_SEL_LX22    PMUC_BUCK1_CR_BUCK1_SEL_LX22_Msk
#define PMUC_BUCK1_CR_BUCK1_MOT_Pos     (13U)
#define PMUC_BUCK1_CR_BUCK1_MOT_Msk     (0x7UL << PMUC_BUCK1_CR_BUCK1_MOT_Pos)
#define PMUC_BUCK1_CR_BUCK1_MOT         PMUC_BUCK1_CR_BUCK1_MOT_Msk
#define PMUC_BUCK1_CR_BUCK1_BM_COTCMP_Pos  (16U)
#define PMUC_BUCK1_CR_BUCK1_BM_COTCMP_Msk  (0x7UL << PMUC_BUCK1_CR_BUCK1_BM_COTCMP_Pos)
#define PMUC_BUCK1_CR_BUCK1_BM_COTCMP   PMUC_BUCK1_CR_BUCK1_BM_COTCMP_Msk
#define PMUC_BUCK1_CR_BUCK1_BM_PWMCMP_Pos  (19U)
#define PMUC_BUCK1_CR_BUCK1_BM_PWMCMP_Msk  (0x7UL << PMUC_BUCK1_CR_BUCK1_BM_PWMCMP_Pos)
#define PMUC_BUCK1_CR_BUCK1_BM_PWMCMP   PMUC_BUCK1_CR_BUCK1_BM_PWMCMP_Msk
#define PMUC_BUCK1_CR_BUCK1_BM_ZCD_Pos  (22U)
#define PMUC_BUCK1_CR_BUCK1_BM_ZCD_Msk  (0x7UL << PMUC_BUCK1_CR_BUCK1_BM_ZCD_Pos)
#define PMUC_BUCK1_CR_BUCK1_BM_ZCD      PMUC_BUCK1_CR_BUCK1_BM_ZCD_Msk
#define PMUC_BUCK1_CR_BUCK1_ZCD_AON_Pos  (25U)
#define PMUC_BUCK1_CR_BUCK1_ZCD_AON_Msk  (0x1UL << PMUC_BUCK1_CR_BUCK1_ZCD_AON_Pos)
#define PMUC_BUCK1_CR_BUCK1_ZCD_AON     PMUC_BUCK1_CR_BUCK1_ZCD_AON_Msk
#define PMUC_BUCK1_CR_BUCK1_RDY_Pos     (26U)
#define PMUC_BUCK1_CR_BUCK1_RDY_Msk     (0x1UL << PMUC_BUCK1_CR_BUCK1_RDY_Pos)
#define PMUC_BUCK1_CR_BUCK1_RDY         PMUC_BUCK1_CR_BUCK1_RDY_Msk
#define PMUC_BUCK1_CR_BUCK1_FORCE_RDY_Pos  (31U)
#define PMUC_BUCK1_CR_BUCK1_FORCE_RDY_Msk  (0x1UL << PMUC_BUCK1_CR_BUCK1_FORCE_RDY_Pos)
#define PMUC_BUCK1_CR_BUCK1_FORCE_RDY   PMUC_BUCK1_CR_BUCK1_FORCE_RDY_Msk

/***************** Bit definition for PMUC_BUCK2_CR register ******************/
#define PMUC_BUCK2_CR_BUCK2_EN_Pos      (0U)
#define PMUC_BUCK2_CR_BUCK2_EN_Msk      (0x1UL << PMUC_BUCK2_CR_BUCK2_EN_Pos)
#define PMUC_BUCK2_CR_BUCK2_EN          PMUC_BUCK2_CR_BUCK2_EN_Msk
#define PMUC_BUCK2_CR_BUCK2_ILIMIT_Pos  (1U)
#define PMUC_BUCK2_CR_BUCK2_ILIMIT_Msk  (0x7UL << PMUC_BUCK2_CR_BUCK2_ILIMIT_Pos)
#define PMUC_BUCK2_CR_BUCK2_ILIMIT      PMUC_BUCK2_CR_BUCK2_ILIMIT_Msk
#define PMUC_BUCK2_CR_BUCK2_CCH_Pos     (4U)
#define PMUC_BUCK2_CR_BUCK2_CCH_Msk     (0xFUL << PMUC_BUCK2_CR_BUCK2_CCH_Pos)
#define PMUC_BUCK2_CR_BUCK2_CCH         PMUC_BUCK2_CR_BUCK2_CCH_Msk
#define PMUC_BUCK2_CR_BUCK2_CS_Pos      (8U)
#define PMUC_BUCK2_CR_BUCK2_CS_Msk      (0xFUL << PMUC_BUCK2_CR_BUCK2_CS_Pos)
#define PMUC_BUCK2_CR_BUCK2_CS          PMUC_BUCK2_CR_BUCK2_CS_Msk
#define PMUC_BUCK2_CR_BUCK2_SEL_LX22_Pos  (12U)
#define PMUC_BUCK2_CR_BUCK2_SEL_LX22_Msk  (0x1UL << PMUC_BUCK2_CR_BUCK2_SEL_LX22_Pos)
#define PMUC_BUCK2_CR_BUCK2_SEL_LX22    PMUC_BUCK2_CR_BUCK2_SEL_LX22_Msk
#define PMUC_BUCK2_CR_BUCK2_MOT_Pos     (13U)
#define PMUC_BUCK2_CR_BUCK2_MOT_Msk     (0x7UL << PMUC_BUCK2_CR_BUCK2_MOT_Pos)
#define PMUC_BUCK2_CR_BUCK2_MOT         PMUC_BUCK2_CR_BUCK2_MOT_Msk
#define PMUC_BUCK2_CR_BUCK2_BM_COTCMP_Pos  (16U)
#define PMUC_BUCK2_CR_BUCK2_BM_COTCMP_Msk  (0x7UL << PMUC_BUCK2_CR_BUCK2_BM_COTCMP_Pos)
#define PMUC_BUCK2_CR_BUCK2_BM_COTCMP   PMUC_BUCK2_CR_BUCK2_BM_COTCMP_Msk
#define PMUC_BUCK2_CR_BUCK2_BM_PWMCMP_Pos  (19U)
#define PMUC_BUCK2_CR_BUCK2_BM_PWMCMP_Msk  (0x7UL << PMUC_BUCK2_CR_BUCK2_BM_PWMCMP_Pos)
#define PMUC_BUCK2_CR_BUCK2_BM_PWMCMP   PMUC_BUCK2_CR_BUCK2_BM_PWMCMP_Msk
#define PMUC_BUCK2_CR_BUCK2_BM_ZCD_Pos  (22U)
#define PMUC_BUCK2_CR_BUCK2_BM_ZCD_Msk  (0x7UL << PMUC_BUCK2_CR_BUCK2_BM_ZCD_Pos)
#define PMUC_BUCK2_CR_BUCK2_BM_ZCD      PMUC_BUCK2_CR_BUCK2_BM_ZCD_Msk
#define PMUC_BUCK2_CR_BUCK2_ZCD_AON_Pos  (25U)
#define PMUC_BUCK2_CR_BUCK2_ZCD_AON_Msk  (0x1UL << PMUC_BUCK2_CR_BUCK2_ZCD_AON_Pos)
#define PMUC_BUCK2_CR_BUCK2_ZCD_AON     PMUC_BUCK2_CR_BUCK2_ZCD_AON_Msk
#define PMUC_BUCK2_CR_BUCK2_RDY_Pos     (26U)
#define PMUC_BUCK2_CR_BUCK2_RDY_Msk     (0x1UL << PMUC_BUCK2_CR_BUCK2_RDY_Pos)
#define PMUC_BUCK2_CR_BUCK2_RDY         PMUC_BUCK2_CR_BUCK2_RDY_Msk
#define PMUC_BUCK2_CR_BUCK2_FORCE_RDY_Pos  (31U)
#define PMUC_BUCK2_CR_BUCK2_FORCE_RDY_Msk  (0x1UL << PMUC_BUCK2_CR_BUCK2_FORCE_RDY_Pos)
#define PMUC_BUCK2_CR_BUCK2_FORCE_RDY   PMUC_BUCK2_CR_BUCK2_FORCE_RDY_Msk

/****************** Bit definition for PMUC_LDO_CR register *******************/
#define PMUC_LDO_CR_HPSYS_LDO_EN_Pos    (0U)
#define PMUC_LDO_CR_HPSYS_LDO_EN_Msk    (0x1UL << PMUC_LDO_CR_HPSYS_LDO_EN_Pos)
#define PMUC_LDO_CR_HPSYS_LDO_EN        PMUC_LDO_CR_HPSYS_LDO_EN_Msk
#define PMUC_LDO_CR_HPSYS_LDO_VREF_Pos  (1U)
#define PMUC_LDO_CR_HPSYS_LDO_VREF_Msk  (0xFUL << PMUC_LDO_CR_HPSYS_LDO_VREF_Pos)
#define PMUC_LDO_CR_HPSYS_LDO_VREF      PMUC_LDO_CR_HPSYS_LDO_VREF_Msk
#define PMUC_LDO_CR_HPSYS_LDO_VREF2_Pos  (5U)
#define PMUC_LDO_CR_HPSYS_LDO_VREF2_Msk  (0xFUL << PMUC_LDO_CR_HPSYS_LDO_VREF2_Pos)
#define PMUC_LDO_CR_HPSYS_LDO_VREF2     PMUC_LDO_CR_HPSYS_LDO_VREF2_Msk
#define PMUC_LDO_CR_HPSYS_LDO_DLY_Pos   (9U)
#define PMUC_LDO_CR_HPSYS_LDO_DLY_Msk   (0x3FUL << PMUC_LDO_CR_HPSYS_LDO_DLY_Pos)
#define PMUC_LDO_CR_HPSYS_LDO_DLY       PMUC_LDO_CR_HPSYS_LDO_DLY_Msk
#define PMUC_LDO_CR_HPSYS_LDO_RDY_Pos   (15U)
#define PMUC_LDO_CR_HPSYS_LDO_RDY_Msk   (0x1UL << PMUC_LDO_CR_HPSYS_LDO_RDY_Pos)
#define PMUC_LDO_CR_HPSYS_LDO_RDY       PMUC_LDO_CR_HPSYS_LDO_RDY_Msk
#define PMUC_LDO_CR_LDOBG_EN_Pos        (31U)
#define PMUC_LDO_CR_LDOBG_EN_Msk        (0x1UL << PMUC_LDO_CR_LDOBG_EN_Pos)
#define PMUC_LDO_CR_LDOBG_EN            PMUC_LDO_CR_LDOBG_EN_Msk

/***************** Bit definition for PMUC_HPSYS_SWR register *****************/
#define PMUC_HPSYS_SWR_PSW_Pos          (0U)
#define PMUC_HPSYS_SWR_PSW_Msk          (0x3UL << PMUC_HPSYS_SWR_PSW_Pos)
#define PMUC_HPSYS_SWR_PSW              PMUC_HPSYS_SWR_PSW_Msk
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
#define PMUC_LPSYS_SWR_PSW_CORE_Pos     (0U)
#define PMUC_LPSYS_SWR_PSW_CORE_Msk     (0xFUL << PMUC_LPSYS_SWR_PSW_CORE_Pos)
#define PMUC_LPSYS_SWR_PSW_CORE         PMUC_LPSYS_SWR_PSW_CORE_Msk
#define PMUC_LPSYS_SWR_PSW_MEM_Pos      (4U)
#define PMUC_LPSYS_SWR_PSW_MEM_Msk      (0xFUL << PMUC_LPSYS_SWR_PSW_MEM_Pos)
#define PMUC_LPSYS_SWR_PSW_MEM          PMUC_LPSYS_SWR_PSW_MEM_Msk
#define PMUC_LPSYS_SWR_DLY_Pos          (8U)
#define PMUC_LPSYS_SWR_DLY_Msk          (0x7UL << PMUC_LPSYS_SWR_DLY_Pos)
#define PMUC_LPSYS_SWR_DLY              PMUC_LPSYS_SWR_DLY_Msk
#define PMUC_LPSYS_SWR_NORET_Pos        (11U)
#define PMUC_LPSYS_SWR_NORET_Msk        (0x1UL << PMUC_LPSYS_SWR_NORET_Pos)
#define PMUC_LPSYS_SWR_NORET            PMUC_LPSYS_SWR_NORET_Msk
#define PMUC_LPSYS_SWR_PSW_CORE_RET_Pos  (12U)
#define PMUC_LPSYS_SWR_PSW_CORE_RET_Msk  (0xFUL << PMUC_LPSYS_SWR_PSW_CORE_RET_Pos)
#define PMUC_LPSYS_SWR_PSW_CORE_RET     PMUC_LPSYS_SWR_PSW_CORE_RET_Msk
#define PMUC_LPSYS_SWR_PSW_MEM_RET_Pos  (16U)
#define PMUC_LPSYS_SWR_PSW_MEM_RET_Msk  (0xFUL << PMUC_LPSYS_SWR_PSW_MEM_RET_Pos)
#define PMUC_LPSYS_SWR_PSW_MEM_RET      PMUC_LPSYS_SWR_PSW_MEM_RET_Msk
#define PMUC_LPSYS_SWR_RDY_Pos          (31U)
#define PMUC_LPSYS_SWR_RDY_Msk          (0x1UL << PMUC_LPSYS_SWR_RDY_Pos)
#define PMUC_LPSYS_SWR_RDY              PMUC_LPSYS_SWR_RDY_Msk

/***************** Bit definition for PMUC_LPCOMP_CR register *****************/
#define PMUC_LPCOMP_CR_EN_Pos           (0U)
#define PMUC_LPCOMP_CR_EN_Msk           (0x1UL << PMUC_LPCOMP_CR_EN_Pos)
#define PMUC_LPCOMP_CR_EN               PMUC_LPCOMP_CR_EN_Msk
#define PMUC_LPCOMP_CR_EN_ATTN_Pos      (1U)
#define PMUC_LPCOMP_CR_EN_ATTN_Msk      (0x1UL << PMUC_LPCOMP_CR_EN_ATTN_Pos)
#define PMUC_LPCOMP_CR_EN_ATTN          PMUC_LPCOMP_CR_EN_ATTN_Msk
#define PMUC_LPCOMP_CR_BM_Pos           (2U)
#define PMUC_LPCOMP_CR_BM_Msk           (0x3UL << PMUC_LPCOMP_CR_BM_Pos)
#define PMUC_LPCOMP_CR_BM               PMUC_LPCOMP_CR_BM_Msk
#define PMUC_LPCOMP_CR_MODE_Pos         (4U)
#define PMUC_LPCOMP_CR_MODE_Msk         (0x3UL << PMUC_LPCOMP_CR_MODE_Pos)
#define PMUC_LPCOMP_CR_MODE             PMUC_LPCOMP_CR_MODE_Msk
#define PMUC_LPCOMP_CR_CH_SEL_Pos       (6U)
#define PMUC_LPCOMP_CR_CH_SEL_Msk       (0x1UL << PMUC_LPCOMP_CR_CH_SEL_Pos)
#define PMUC_LPCOMP_CR_CH_SEL           PMUC_LPCOMP_CR_CH_SEL_Msk
#define PMUC_LPCOMP_CR_HYST_SEL_Pos     (7U)
#define PMUC_LPCOMP_CR_HYST_SEL_Msk     (0x3UL << PMUC_LPCOMP_CR_HYST_SEL_Pos)
#define PMUC_LPCOMP_CR_HYST_SEL         PMUC_LPCOMP_CR_HYST_SEL_Msk
#define PMUC_LPCOMP_CR_VREF_INT_SEL_Pos  (9U)
#define PMUC_LPCOMP_CR_VREF_INT_SEL_Msk  (0x3UL << PMUC_LPCOMP_CR_VREF_INT_SEL_Pos)
#define PMUC_LPCOMP_CR_VREF_INT_SEL     PMUC_LPCOMP_CR_VREF_INT_SEL_Msk
#define PMUC_LPCOMP_CR_VREF_MODE_Pos    (11U)
#define PMUC_LPCOMP_CR_VREF_MODE_Msk    (0x1UL << PMUC_LPCOMP_CR_VREF_MODE_Pos)
#define PMUC_LPCOMP_CR_VREF_MODE        PMUC_LPCOMP_CR_VREF_MODE_Msk
#define PMUC_LPCOMP_CR_COMP_OUT_Pos     (12U)
#define PMUC_LPCOMP_CR_COMP_OUT_Msk     (0x1UL << PMUC_LPCOMP_CR_COMP_OUT_Pos)
#define PMUC_LPCOMP_CR_COMP_OUT         PMUC_LPCOMP_CR_COMP_OUT_Msk

/****************** Bit definition for PMUC_PMU_TR register *******************/
#define PMUC_PMU_TR_PMU_DC_TR_Pos       (0U)
#define PMUC_PMU_TR_PMU_DC_TR_Msk       (0x7UL << PMUC_PMU_TR_PMU_DC_TR_Pos)
#define PMUC_PMU_TR_PMU_DC_TR           PMUC_PMU_TR_PMU_DC_TR_Msk
#define PMUC_PMU_TR_PMU_DC_BR_Pos       (3U)
#define PMUC_PMU_TR_PMU_DC_BR_Msk       (0x7UL << PMUC_PMU_TR_PMU_DC_BR_Pos)
#define PMUC_PMU_TR_PMU_DC_BR           PMUC_PMU_TR_PMU_DC_BR_Msk
#define PMUC_PMU_TR_PMU_DC_MR_Pos       (6U)
#define PMUC_PMU_TR_PMU_DC_MR_Msk       (0x7UL << PMUC_PMU_TR_PMU_DC_MR_Pos)
#define PMUC_PMU_TR_PMU_DC_MR           PMUC_PMU_TR_PMU_DC_MR_Msk

/***************** Bit definition for PMUC_PMU_RSVD1 register *****************/
#define PMUC_PMU_RSVD1_RESERVE0_Pos     (0U)
#define PMUC_PMU_RSVD1_RESERVE0_Msk     (0xFFUL << PMUC_PMU_RSVD1_RESERVE0_Pos)
#define PMUC_PMU_RSVD1_RESERVE0         PMUC_PMU_RSVD1_RESERVE0_Msk
#define PMUC_PMU_RSVD1_RESERVE1_Pos     (8U)
#define PMUC_PMU_RSVD1_RESERVE1_Msk     (0xFFUL << PMUC_PMU_RSVD1_RESERVE1_Pos)
#define PMUC_PMU_RSVD1_RESERVE1         PMUC_PMU_RSVD1_RESERVE1_Msk
#define PMUC_PMU_RSVD1_RESERVE2_Pos     (16U)
#define PMUC_PMU_RSVD1_RESERVE2_Msk     (0xFFUL << PMUC_PMU_RSVD1_RESERVE2_Pos)
#define PMUC_PMU_RSVD1_RESERVE2         PMUC_PMU_RSVD1_RESERVE2_Msk
#define PMUC_PMU_RSVD1_RESERVE3_Pos     (24U)
#define PMUC_PMU_RSVD1_RESERVE3_Msk     (0xFFUL << PMUC_PMU_RSVD1_RESERVE3_Pos)
#define PMUC_PMU_RSVD1_RESERVE3         PMUC_PMU_RSVD1_RESERVE3_Msk

/***************** Bit definition for PMUC_PMU_RSVD2 register *****************/
#define PMUC_PMU_RSVD2_RESERVE0_Pos     (0U)
#define PMUC_PMU_RSVD2_RESERVE0_Msk     (0xFFUL << PMUC_PMU_RSVD2_RESERVE0_Pos)
#define PMUC_PMU_RSVD2_RESERVE0         PMUC_PMU_RSVD2_RESERVE0_Msk
#define PMUC_PMU_RSVD2_RESERVE1_Pos     (8U)
#define PMUC_PMU_RSVD2_RESERVE1_Msk     (0xFFUL << PMUC_PMU_RSVD2_RESERVE1_Pos)
#define PMUC_PMU_RSVD2_RESERVE1         PMUC_PMU_RSVD2_RESERVE1_Msk
#define PMUC_PMU_RSVD2_RESERVE2_Pos     (16U)
#define PMUC_PMU_RSVD2_RESERVE2_Msk     (0xFFUL << PMUC_PMU_RSVD2_RESERVE2_Pos)
#define PMUC_PMU_RSVD2_RESERVE2         PMUC_PMU_RSVD2_RESERVE2_Msk
#define PMUC_PMU_RSVD2_RESERVE3_Pos     (24U)
#define PMUC_PMU_RSVD2_RESERVE3_Msk     (0xFFUL << PMUC_PMU_RSVD2_RESERVE3_Pos)
#define PMUC_PMU_RSVD2_RESERVE3         PMUC_PMU_RSVD2_RESERVE3_Msk

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
#define PMUC_HXT_CR3_OPT_CBANK_SEL_Pos  (0U)
#define PMUC_HXT_CR3_OPT_CBANK_SEL_Msk  (0x3FFUL << PMUC_HXT_CR3_OPT_CBANK_SEL_Pos)
#define PMUC_HXT_CR3_OPT_CBANK_SEL      PMUC_HXT_CR3_OPT_CBANK_SEL_Msk
#define PMUC_HXT_CR3_OPT_IDAC_EN_Pos    (10U)
#define PMUC_HXT_CR3_OPT_IDAC_EN_Msk    (0x1UL << PMUC_HXT_CR3_OPT_IDAC_EN_Pos)
#define PMUC_HXT_CR3_OPT_IDAC_EN        PMUC_HXT_CR3_OPT_IDAC_EN_Msk
#define PMUC_HXT_CR3_OPT_IDAC_Pos       (11U)
#define PMUC_HXT_CR3_OPT_IDAC_Msk       (0x3FFUL << PMUC_HXT_CR3_OPT_IDAC_Pos)
#define PMUC_HXT_CR3_OPT_IDAC           PMUC_HXT_CR3_OPT_IDAC_Msk
#define PMUC_HXT_CR3_BUF_DAC_STR_Pos    (21U)
#define PMUC_HXT_CR3_BUF_DAC_STR_Msk    (0x3UL << PMUC_HXT_CR3_BUF_DAC_STR_Pos)
#define PMUC_HXT_CR3_BUF_DAC_STR        PMUC_HXT_CR3_BUF_DAC_STR_Msk
#define PMUC_HXT_CR3_BUF_OSLO_STR_Pos   (23U)
#define PMUC_HXT_CR3_BUF_OSLO_STR_Msk   (0x3UL << PMUC_HXT_CR3_BUF_OSLO_STR_Pos)
#define PMUC_HXT_CR3_BUF_OSLO_STR       PMUC_HXT_CR3_BUF_OSLO_STR_Msk
#define PMUC_HXT_CR3_DLY_Pos            (25U)
#define PMUC_HXT_CR3_DLY_Msk            (0x3FUL << PMUC_HXT_CR3_DLY_Pos)
#define PMUC_HXT_CR3_DLY                PMUC_HXT_CR3_DLY_Msk

/****************** Bit definition for PMUC_HRC_CR register *******************/
#define PMUC_HRC_CR_EN_Pos              (0U)
#define PMUC_HRC_CR_EN_Msk              (0x1UL << PMUC_HRC_CR_EN_Pos)
#define PMUC_HRC_CR_EN                  PMUC_HRC_CR_EN_Msk
#define PMUC_HRC_CR_OUT_EN_Pos          (1U)
#define PMUC_HRC_CR_OUT_EN_Msk          (0x1UL << PMUC_HRC_CR_OUT_EN_Pos)
#define PMUC_HRC_CR_OUT_EN              PMUC_HRC_CR_OUT_EN_Msk
#define PMUC_HRC_CR_OUT_STR_Pos         (2U)
#define PMUC_HRC_CR_OUT_STR_Msk         (0x3UL << PMUC_HRC_CR_OUT_STR_Pos)
#define PMUC_HRC_CR_OUT_STR             PMUC_HRC_CR_OUT_STR_Msk
#define PMUC_HRC_CR_RST_Pos             (4U)
#define PMUC_HRC_CR_RST_Msk             (0x1UL << PMUC_HRC_CR_RST_Pos)
#define PMUC_HRC_CR_RST                 PMUC_HRC_CR_RST_Msk
#define PMUC_HRC_CR_LDO_VREF_Pos        (5U)
#define PMUC_HRC_CR_LDO_VREF_Msk        (0xFUL << PMUC_HRC_CR_LDO_VREF_Pos)
#define PMUC_HRC_CR_LDO_VREF            PMUC_HRC_CR_LDO_VREF_Msk
#define PMUC_HRC_CR_BM_Pos              (9U)
#define PMUC_HRC_CR_BM_Msk              (0x3UL << PMUC_HRC_CR_BM_Pos)
#define PMUC_HRC_CR_BM                  PMUC_HRC_CR_BM_Msk
#define PMUC_HRC_CR_MODE24M_EN_Pos      (11U)
#define PMUC_HRC_CR_MODE24M_EN_Msk      (0x1UL << PMUC_HRC_CR_MODE24M_EN_Pos)
#define PMUC_HRC_CR_MODE24M_EN          PMUC_HRC_CR_MODE24M_EN_Msk
#define PMUC_HRC_CR_CT_Pos              (12U)
#define PMUC_HRC_CR_CT_Msk              (0x7FFUL << PMUC_HRC_CR_CT_Pos)
#define PMUC_HRC_CR_CT                  PMUC_HRC_CR_CT_Msk
#define PMUC_HRC_CR_DLY_Pos             (23U)
#define PMUC_HRC_CR_DLY_Msk             (0x1UL << PMUC_HRC_CR_DLY_Pos)
#define PMUC_HRC_CR_DLY                 PMUC_HRC_CR_DLY_Msk

/****************** Bit definition for PMUC_RC1M_CR register ******************/
#define PMUC_RC1M_CR_EN_Pos             (0U)
#define PMUC_RC1M_CR_EN_Msk             (0x1UL << PMUC_RC1M_CR_EN_Pos)
#define PMUC_RC1M_CR_EN                 PMUC_RC1M_CR_EN_Msk
#define PMUC_RC1M_CR_OUT_EN_Pos         (1U)
#define PMUC_RC1M_CR_OUT_EN_Msk         (0x1UL << PMUC_RC1M_CR_OUT_EN_Pos)
#define PMUC_RC1M_CR_OUT_EN             PMUC_RC1M_CR_OUT_EN_Msk
#define PMUC_RC1M_CR_RST_Pos            (2U)
#define PMUC_RC1M_CR_RST_Msk            (0x1UL << PMUC_RC1M_CR_RST_Pos)
#define PMUC_RC1M_CR_RST                PMUC_RC1M_CR_RST_Msk
#define PMUC_RC1M_CR_BM_CHG_Pos         (3U)
#define PMUC_RC1M_CR_BM_CHG_Msk         (0x3UL << PMUC_RC1M_CR_BM_CHG_Pos)
#define PMUC_RC1M_CR_BM_CHG             PMUC_RC1M_CR_BM_CHG_Msk
#define PMUC_RC1M_CR_BM_COMP_Pos        (5U)
#define PMUC_RC1M_CR_BM_COMP_Msk        (0x3UL << PMUC_RC1M_CR_BM_COMP_Pos)
#define PMUC_RC1M_CR_BM_COMP            PMUC_RC1M_CR_BM_COMP_Msk
#define PMUC_RC1M_CR_CSEL_Pos           (7U)
#define PMUC_RC1M_CR_CSEL_Msk           (0x7FUL << PMUC_RC1M_CR_CSEL_Pos)
#define PMUC_RC1M_CR_CSEL               PMUC_RC1M_CR_CSEL_Msk
#define PMUC_RC1M_CR_RSEL_Pos           (14U)
#define PMUC_RC1M_CR_RSEL_Msk           (0x3UL << PMUC_RC1M_CR_RSEL_Pos)
#define PMUC_RC1M_CR_RSEL               PMUC_RC1M_CR_RSEL_Msk
#define PMUC_RC1M_CR_DLY_Pos            (16U)
#define PMUC_RC1M_CR_DLY_Msk            (0x1UL << PMUC_RC1M_CR_DLY_Pos)
#define PMUC_RC1M_CR_DLY                PMUC_RC1M_CR_DLY_Msk
#define PMUC_RC1M_CR_SEL_SOURCE_Pos     (17U)
#define PMUC_RC1M_CR_SEL_SOURCE_Msk     (0x1UL << PMUC_RC1M_CR_SEL_SOURCE_Pos)
#define PMUC_RC1M_CR_SEL_SOURCE         PMUC_RC1M_CR_SEL_SOURCE_Msk
#define PMUC_RC1M_CR_RINGOSC_EN_Pos     (18U)
#define PMUC_RC1M_CR_RINGOSC_EN_Msk     (0x1UL << PMUC_RC1M_CR_RINGOSC_EN_Pos)
#define PMUC_RC1M_CR_RINGOSC_EN         PMUC_RC1M_CR_RINGOSC_EN_Msk
#define PMUC_RC1M_CR_RINGOSC_BM_Pos     (19U)
#define PMUC_RC1M_CR_RINGOSC_BM_Msk     (0xFUL << PMUC_RC1M_CR_RINGOSC_BM_Pos)
#define PMUC_RC1M_CR_RINGOSC_BM         PMUC_RC1M_CR_RINGOSC_BM_Msk
#define PMUC_RC1M_CR_RINGOSC_MODE_Pos   (23U)
#define PMUC_RC1M_CR_RINGOSC_MODE_Msk   (0x1UL << PMUC_RC1M_CR_RINGOSC_MODE_Pos)
#define PMUC_RC1M_CR_RINGOSC_MODE       PMUC_RC1M_CR_RINGOSC_MODE_Msk

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
#define PMUC_CAU_RSVD_RESERVE3_Pos      (24U)
#define PMUC_CAU_RSVD_RESERVE3_Msk      (0xFFUL << PMUC_CAU_RSVD_RESERVE3_Pos)
#define PMUC_CAU_RSVD_RESERVE3          PMUC_CAU_RSVD_RESERVE3_Msk

/***************** Bit definition for PMUC_TSTIF_CR register ******************/
#define PMUC_TSTIF_CR_TST_EN_Pos        (0U)
#define PMUC_TSTIF_CR_TST_EN_Msk        (0x1UL << PMUC_TSTIF_CR_TST_EN_Pos)
#define PMUC_TSTIF_CR_TST_EN            PMUC_TSTIF_CR_TST_EN_Msk
#define PMUC_TSTIF_CR_TST_SIGNAL_Pos    (1U)
#define PMUC_TSTIF_CR_TST_SIGNAL_Msk    (0x7UL << PMUC_TSTIF_CR_TST_SIGNAL_Pos)
#define PMUC_TSTIF_CR_TST_SIGNAL        PMUC_TSTIF_CR_TST_SIGNAL_Msk
#define PMUC_TSTIF_CR_TST_MODE_Pos      (4U)
#define PMUC_TSTIF_CR_TST_MODE_Msk      (0xFUL << PMUC_TSTIF_CR_TST_MODE_Pos)
#define PMUC_TSTIF_CR_TST_MODE          PMUC_TSTIF_CR_TST_MODE_Msk

/***************** Bit definition for PMUC_HFOSC_CR register ******************/
#define PMUC_HFOSC_CR_EN_Pos            (0U)
#define PMUC_HFOSC_CR_EN_Msk            (0x1UL << PMUC_HFOSC_CR_EN_Pos)
#define PMUC_HFOSC_CR_EN                PMUC_HFOSC_CR_EN_Msk
#define PMUC_HFOSC_CR_FREQ_TRIM_Pos     (1U)
#define PMUC_HFOSC_CR_FREQ_TRIM_Msk     (0x3FUL << PMUC_HFOSC_CR_FREQ_TRIM_Pos)
#define PMUC_HFOSC_CR_FREQ_TRIM         PMUC_HFOSC_CR_FREQ_TRIM_Msk
#define PMUC_HFOSC_CR_TEMPCO_TRIM_Pos   (7U)
#define PMUC_HFOSC_CR_TEMPCO_TRIM_Msk   (0x1FUL << PMUC_HFOSC_CR_TEMPCO_TRIM_Pos)
#define PMUC_HFOSC_CR_TEMPCO_TRIM       PMUC_HFOSC_CR_TEMPCO_TRIM_Msk
#define PMUC_HFOSC_CR_TST_SIGNAL_Pos    (12U)
#define PMUC_HFOSC_CR_TST_SIGNAL_Msk    (0x7UL << PMUC_HFOSC_CR_TST_SIGNAL_Pos)
#define PMUC_HFOSC_CR_TST_SIGNAL        PMUC_HFOSC_CR_TST_SIGNAL_Msk

/***************** Bit definition for PMUC_ULPVR_CR1 register *****************/
#define PMUC_ULPVR_CR1_EN_Pos           (0U)
#define PMUC_ULPVR_CR1_EN_Msk           (0x1UL << PMUC_ULPVR_CR1_EN_Pos)
#define PMUC_ULPVR_CR1_EN               PMUC_ULPVR_CR1_EN_Msk
#define PMUC_ULPVR_CR1_EN_SLP_DET_Pos   (1U)
#define PMUC_ULPVR_CR1_EN_SLP_DET_Msk   (0x1UL << PMUC_ULPVR_CR1_EN_SLP_DET_Pos)
#define PMUC_ULPVR_CR1_EN_SLP_DET       PMUC_ULPVR_CR1_EN_SLP_DET_Msk
#define PMUC_ULPVR_CR1_VREF_SEL_Pos     (2U)
#define PMUC_ULPVR_CR1_VREF_SEL_Msk     (0x1UL << PMUC_ULPVR_CR1_VREF_SEL_Pos)
#define PMUC_ULPVR_CR1_VREF_SEL         PMUC_ULPVR_CR1_VREF_SEL_Msk
#define PMUC_ULPVR_CR1_PWMD1_Pos        (3U)
#define PMUC_ULPVR_CR1_PWMD1_Msk        (0x3UL << PMUC_ULPVR_CR1_PWMD1_Pos)
#define PMUC_ULPVR_CR1_PWMD1            PMUC_ULPVR_CR1_PWMD1_Msk
#define PMUC_ULPVR_CR1_PWMD2_Pos        (5U)
#define PMUC_ULPVR_CR1_PWMD2_Msk        (0x3UL << PMUC_ULPVR_CR1_PWMD2_Pos)
#define PMUC_ULPVR_CR1_PWMD2            PMUC_ULPVR_CR1_PWMD2_Msk
#define PMUC_ULPVR_CR1_VOUT1_TRIM_Pos   (7U)
#define PMUC_ULPVR_CR1_VOUT1_TRIM_Msk   (0x7FUL << PMUC_ULPVR_CR1_VOUT1_TRIM_Pos)
#define PMUC_ULPVR_CR1_VOUT1_TRIM       PMUC_ULPVR_CR1_VOUT1_TRIM_Msk
#define PMUC_ULPVR_CR1_VOUT2_TRIM_Pos   (14U)
#define PMUC_ULPVR_CR1_VOUT2_TRIM_Msk   (0x7FUL << PMUC_ULPVR_CR1_VOUT2_TRIM_Pos)
#define PMUC_ULPVR_CR1_VOUT2_TRIM       PMUC_ULPVR_CR1_VOUT2_TRIM_Msk
#define PMUC_ULPVR_CR1_CLKH_TRIM_Pos    (21U)
#define PMUC_ULPVR_CR1_CLKH_TRIM_Msk    (0x7UL << PMUC_ULPVR_CR1_CLKH_TRIM_Pos)
#define PMUC_ULPVR_CR1_CLKH_TRIM        PMUC_ULPVR_CR1_CLKH_TRIM_Msk
#define PMUC_ULPVR_CR1_CLKM_TRIM_Pos    (24U)
#define PMUC_ULPVR_CR1_CLKM_TRIM_Msk    (0x7UL << PMUC_ULPVR_CR1_CLKM_TRIM_Pos)
#define PMUC_ULPVR_CR1_CLKM_TRIM        PMUC_ULPVR_CR1_CLKM_TRIM_Msk
#define PMUC_ULPVR_CR1_CLKL_TRIM_Pos    (27U)
#define PMUC_ULPVR_CR1_CLKL_TRIM_Msk    (0x7UL << PMUC_ULPVR_CR1_CLKL_TRIM_Pos)
#define PMUC_ULPVR_CR1_CLKL_TRIM        PMUC_ULPVR_CR1_CLKL_TRIM_Msk

/***************** Bit definition for PMUC_ULPVR_CR2 register *****************/
#define PMUC_ULPVR_CR2_TONP1_TRIM_Pos   (0U)
#define PMUC_ULPVR_CR2_TONP1_TRIM_Msk   (0x7FUL << PMUC_ULPVR_CR2_TONP1_TRIM_Pos)
#define PMUC_ULPVR_CR2_TONP1_TRIM       PMUC_ULPVR_CR2_TONP1_TRIM_Msk
#define PMUC_ULPVR_CR2_TONN1_TRIM_Pos   (7U)
#define PMUC_ULPVR_CR2_TONN1_TRIM_Msk   (0x7FUL << PMUC_ULPVR_CR2_TONN1_TRIM_Pos)
#define PMUC_ULPVR_CR2_TONN1_TRIM       PMUC_ULPVR_CR2_TONN1_TRIM_Msk
#define PMUC_ULPVR_CR2_TONP2_TRIM_Pos   (14U)
#define PMUC_ULPVR_CR2_TONP2_TRIM_Msk   (0x7FUL << PMUC_ULPVR_CR2_TONP2_TRIM_Pos)
#define PMUC_ULPVR_CR2_TONP2_TRIM       PMUC_ULPVR_CR2_TONP2_TRIM_Msk
#define PMUC_ULPVR_CR2_TONN2_TRIM_Pos   (21U)
#define PMUC_ULPVR_CR2_TONN2_TRIM_Msk   (0x7FUL << PMUC_ULPVR_CR2_TONN2_TRIM_Pos)
#define PMUC_ULPVR_CR2_TONN2_TRIM       PMUC_ULPVR_CR2_TONN2_TRIM_Msk
#define PMUC_ULPVR_CR2_TONCAL_CLK_TRIM_Pos  (28U)
#define PMUC_ULPVR_CR2_TONCAL_CLK_TRIM_Msk  (0x3UL << PMUC_ULPVR_CR2_TONCAL_CLK_TRIM_Pos)
#define PMUC_ULPVR_CR2_TONCAL_CLK_TRIM  PMUC_ULPVR_CR2_TONCAL_CLK_TRIM_Msk
#define PMUC_ULPVR_CR2_TONCAL_FORCE_B_Pos  (30U)
#define PMUC_ULPVR_CR2_TONCAL_FORCE_B_Msk  (0x1UL << PMUC_ULPVR_CR2_TONCAL_FORCE_B_Pos)
#define PMUC_ULPVR_CR2_TONCAL_FORCE_B   PMUC_ULPVR_CR2_TONCAL_FORCE_B_Msk

/***************** Bit definition for PMUC_ULPVR_CR3 register *****************/
#define PMUC_ULPVR_CR3_TEMPCO_TRIM_2T_Pos  (0U)
#define PMUC_ULPVR_CR3_TEMPCO_TRIM_2T_Msk  (0x1FUL << PMUC_ULPVR_CR3_TEMPCO_TRIM_2T_Pos)
#define PMUC_ULPVR_CR3_TEMPCO_TRIM_2T   PMUC_ULPVR_CR3_TEMPCO_TRIM_2T_Msk
#define PMUC_ULPVR_CR3_I200N_TRIM_Pos   (5U)
#define PMUC_ULPVR_CR3_I200N_TRIM_Msk   (0x7UL << PMUC_ULPVR_CR3_I200N_TRIM_Pos)
#define PMUC_ULPVR_CR3_I200N_TRIM       PMUC_ULPVR_CR3_I200N_TRIM_Msk
#define PMUC_ULPVR_CR3_ICAL_TRIM_Pos    (8U)
#define PMUC_ULPVR_CR3_ICAL_TRIM_Msk    (0x1FUL << PMUC_ULPVR_CR3_ICAL_TRIM_Pos)
#define PMUC_ULPVR_CR3_ICAL_TRIM        PMUC_ULPVR_CR3_ICAL_TRIM_Msk
#define PMUC_ULPVR_CR3_V300M_TRIM_Pos   (13U)
#define PMUC_ULPVR_CR3_V300M_TRIM_Msk   (0xFUL << PMUC_ULPVR_CR3_V300M_TRIM_Pos)
#define PMUC_ULPVR_CR3_V300M_TRIM       PMUC_ULPVR_CR3_V300M_TRIM_Msk
#define PMUC_ULPVR_CR3_V500M_TRIM_Pos   (17U)
#define PMUC_ULPVR_CR3_V500M_TRIM_Msk   (0xFUL << PMUC_ULPVR_CR3_V500M_TRIM_Pos)
#define PMUC_ULPVR_CR3_V500M_TRIM       PMUC_ULPVR_CR3_V500M_TRIM_Msk
#define PMUC_ULPVR_CR3_V1P2_TRIM_Pos    (21U)
#define PMUC_ULPVR_CR3_V1P2_TRIM_Msk    (0xFUL << PMUC_ULPVR_CR3_V1P2_TRIM_Pos)
#define PMUC_ULPVR_CR3_V1P2_TRIM        PMUC_ULPVR_CR3_V1P2_TRIM_Msk

/***************** Bit definition for PMUC_ULPVR_CR4 register *****************/
#define PMUC_ULPVR_CR4_VCTAT_TRIM_Pos   (0U)
#define PMUC_ULPVR_CR4_VCTAT_TRIM_Msk   (0xFFFUL << PMUC_ULPVR_CR4_VCTAT_TRIM_Pos)
#define PMUC_ULPVR_CR4_VCTAT_TRIM       PMUC_ULPVR_CR4_VCTAT_TRIM_Msk
#define PMUC_ULPVR_CR4_DISABLE1N_Pos    (12U)
#define PMUC_ULPVR_CR4_DISABLE1N_Msk    (0x1UL << PMUC_ULPVR_CR4_DISABLE1N_Pos)
#define PMUC_ULPVR_CR4_DISABLE1N        PMUC_ULPVR_CR4_DISABLE1N_Msk
#define PMUC_ULPVR_CR4_DISABLE200N_Pos  (13U)
#define PMUC_ULPVR_CR4_DISABLE200N_Msk  (0x1UL << PMUC_ULPVR_CR4_DISABLE200N_Pos)
#define PMUC_ULPVR_CR4_DISABLE200N      PMUC_ULPVR_CR4_DISABLE200N_Msk
#define PMUC_ULPVR_CR4_TST_EN_Pos       (16U)
#define PMUC_ULPVR_CR4_TST_EN_Msk       (0x1UL << PMUC_ULPVR_CR4_TST_EN_Pos)
#define PMUC_ULPVR_CR4_TST_EN           PMUC_ULPVR_CR4_TST_EN_Msk
#define PMUC_ULPVR_CR4_TST_BLOCK_Pos    (17U)
#define PMUC_ULPVR_CR4_TST_BLOCK_Msk    (0x7UL << PMUC_ULPVR_CR4_TST_BLOCK_Pos)
#define PMUC_ULPVR_CR4_TST_BLOCK        PMUC_ULPVR_CR4_TST_BLOCK_Msk
#define PMUC_ULPVR_CR4_TST_SIGNAL_Pos   (20U)
#define PMUC_ULPVR_CR4_TST_SIGNAL_Msk   (0x7UL << PMUC_ULPVR_CR4_TST_SIGNAL_Pos)
#define PMUC_ULPVR_CR4_TST_SIGNAL       PMUC_ULPVR_CR4_TST_SIGNAL_Msk

/***************** Bit definition for PMUC_ULPVR_CR5 register *****************/
#define PMUC_ULPVR_CR5_PWMD1_RET_Pos    (0U)
#define PMUC_ULPVR_CR5_PWMD1_RET_Msk    (0x3UL << PMUC_ULPVR_CR5_PWMD1_RET_Pos)
#define PMUC_ULPVR_CR5_PWMD1_RET        PMUC_ULPVR_CR5_PWMD1_RET_Msk
#define PMUC_ULPVR_CR5_PWMD2_RET_Pos    (2U)
#define PMUC_ULPVR_CR5_PWMD2_RET_Msk    (0x3UL << PMUC_ULPVR_CR5_PWMD2_RET_Pos)
#define PMUC_ULPVR_CR5_PWMD2_RET        PMUC_ULPVR_CR5_PWMD2_RET_Msk
#define PMUC_ULPVR_CR5_VOUT1_RET_Pos    (4U)
#define PMUC_ULPVR_CR5_VOUT1_RET_Msk    (0x7FUL << PMUC_ULPVR_CR5_VOUT1_RET_Pos)
#define PMUC_ULPVR_CR5_VOUT1_RET        PMUC_ULPVR_CR5_VOUT1_RET_Msk
#define PMUC_ULPVR_CR5_VOUT2_RET_Pos    (11U)
#define PMUC_ULPVR_CR5_VOUT2_RET_Msk    (0x7FUL << PMUC_ULPVR_CR5_VOUT2_RET_Pos)
#define PMUC_ULPVR_CR5_VOUT2_RET        PMUC_ULPVR_CR5_VOUT2_RET_Msk
#define PMUC_ULPVR_CR5_DISABLE1N_RET_Pos  (18U)
#define PMUC_ULPVR_CR5_DISABLE1N_RET_Msk  (0x1UL << PMUC_ULPVR_CR5_DISABLE1N_RET_Pos)
#define PMUC_ULPVR_CR5_DISABLE1N_RET    PMUC_ULPVR_CR5_DISABLE1N_RET_Msk
#define PMUC_ULPVR_CR5_DISABLE200N_RET_Pos  (19U)
#define PMUC_ULPVR_CR5_DISABLE200N_RET_Msk  (0x1UL << PMUC_ULPVR_CR5_DISABLE200N_RET_Pos)
#define PMUC_ULPVR_CR5_DISABLE200N_RET  PMUC_ULPVR_CR5_DISABLE200N_RET_Msk

/***************** Bit definition for PMUC_WKUP_CNT register ******************/
#define PMUC_WKUP_CNT_PIN0_CNT_Pos      (0U)
#define PMUC_WKUP_CNT_PIN0_CNT_Msk      (0xFFFFUL << PMUC_WKUP_CNT_PIN0_CNT_Pos)
#define PMUC_WKUP_CNT_PIN0_CNT          PMUC_WKUP_CNT_PIN0_CNT_Msk
#define PMUC_WKUP_CNT_PIN1_CNT_Pos      (16U)
#define PMUC_WKUP_CNT_PIN1_CNT_Msk      (0xFFFFUL << PMUC_WKUP_CNT_PIN1_CNT_Pos)
#define PMUC_WKUP_CNT_PIN1_CNT          PMUC_WKUP_CNT_PIN1_CNT_Msk

#endif
