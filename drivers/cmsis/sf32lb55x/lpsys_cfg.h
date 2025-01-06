#ifndef __LPSYS_CFG_H
#define __LPSYS_CFG_H

typedef struct
{
    __IO uint32_t SWCR;
    __IO uint32_t ULPMCR;
    __IO uint32_t RTC_TR;
    __IO uint32_t RTC_DR;
    __IO uint32_t LSDBGR;
    __IO uint32_t DBGR;
    __IO uint32_t DBGCLKR;
    __IO uint32_t BISTR;
    __IO uint32_t SEAR0;
    __IO uint32_t SEAR1;
    __IO uint32_t SEAR2;
    __IO uint32_t SEAR3;
    __IO uint32_t SEAR4;
    __IO uint32_t SEAR5;
    __IO uint32_t SEAR6;
    __IO uint32_t SEAR7;
} LPSYS_CFG_TypeDef;

#define LPSYS_CFG_PINR_SIZE          (0)

/***************** Bit definition for LPSYS_CFG_SWCR register *****************/
#define LPSYS_CFG_SWCR_SWSEL_Pos        (0U)
#define LPSYS_CFG_SWCR_SWSEL_Msk        (0x1UL << LPSYS_CFG_SWCR_SWSEL_Pos)
#define LPSYS_CFG_SWCR_SWSEL            LPSYS_CFG_SWCR_SWSEL_Msk

/**************** Bit definition for LPSYS_CFG_ULPMCR register ****************/
#define LPSYS_CFG_ULPMCR_RAM_RM_Pos     (0U)
#define LPSYS_CFG_ULPMCR_RAM_RM_Msk     (0x7UL << LPSYS_CFG_ULPMCR_RAM_RM_Pos)
#define LPSYS_CFG_ULPMCR_RAM_RM         LPSYS_CFG_ULPMCR_RAM_RM_Msk
#define LPSYS_CFG_ULPMCR_RAM_RME_Pos    (4U)
#define LPSYS_CFG_ULPMCR_RAM_RME_Msk    (0x1UL << LPSYS_CFG_ULPMCR_RAM_RME_Pos)
#define LPSYS_CFG_ULPMCR_RAM_RME        LPSYS_CFG_ULPMCR_RAM_RME_Msk
#define LPSYS_CFG_ULPMCR_RAM_RA_Pos     (5U)
#define LPSYS_CFG_ULPMCR_RAM_RA_Msk     (0x3UL << LPSYS_CFG_ULPMCR_RAM_RA_Pos)
#define LPSYS_CFG_ULPMCR_RAM_RA         LPSYS_CFG_ULPMCR_RAM_RA_Msk
#define LPSYS_CFG_ULPMCR_RAM_WA_Pos     (7U)
#define LPSYS_CFG_ULPMCR_RAM_WA_Msk     (0x7UL << LPSYS_CFG_ULPMCR_RAM_WA_Pos)
#define LPSYS_CFG_ULPMCR_RAM_WA         LPSYS_CFG_ULPMCR_RAM_WA_Msk
#define LPSYS_CFG_ULPMCR_RAM_WPULSE_Pos  (10U)
#define LPSYS_CFG_ULPMCR_RAM_WPULSE_Msk  (0x7UL << LPSYS_CFG_ULPMCR_RAM_WPULSE_Pos)
#define LPSYS_CFG_ULPMCR_RAM_WPULSE     LPSYS_CFG_ULPMCR_RAM_WPULSE_Msk
#define LPSYS_CFG_ULPMCR_ROM_RM_Pos     (16U)
#define LPSYS_CFG_ULPMCR_ROM_RM_Msk     (0xFUL << LPSYS_CFG_ULPMCR_ROM_RM_Pos)
#define LPSYS_CFG_ULPMCR_ROM_RM         LPSYS_CFG_ULPMCR_ROM_RM_Msk
#define LPSYS_CFG_ULPMCR_ROM_RME_Pos    (20U)
#define LPSYS_CFG_ULPMCR_ROM_RME_Msk    (0x1UL << LPSYS_CFG_ULPMCR_ROM_RME_Pos)
#define LPSYS_CFG_ULPMCR_ROM_RME        LPSYS_CFG_ULPMCR_ROM_RME_Msk

/**************** Bit definition for LPSYS_CFG_RTC_TR register ****************/
#define LPSYS_CFG_RTC_TR_SS_Pos         (0U)
#define LPSYS_CFG_RTC_TR_SS_Msk         (0x3FFUL << LPSYS_CFG_RTC_TR_SS_Pos)
#define LPSYS_CFG_RTC_TR_SS             LPSYS_CFG_RTC_TR_SS_Msk
#define LPSYS_CFG_RTC_TR_SU_Pos         (11U)
#define LPSYS_CFG_RTC_TR_SU_Msk         (0xFUL << LPSYS_CFG_RTC_TR_SU_Pos)
#define LPSYS_CFG_RTC_TR_SU             LPSYS_CFG_RTC_TR_SU_Msk
#define LPSYS_CFG_RTC_TR_ST_Pos         (15U)
#define LPSYS_CFG_RTC_TR_ST_Msk         (0x7UL << LPSYS_CFG_RTC_TR_ST_Pos)
#define LPSYS_CFG_RTC_TR_ST             LPSYS_CFG_RTC_TR_ST_Msk
#define LPSYS_CFG_RTC_TR_MNU_Pos        (18U)
#define LPSYS_CFG_RTC_TR_MNU_Msk        (0xFUL << LPSYS_CFG_RTC_TR_MNU_Pos)
#define LPSYS_CFG_RTC_TR_MNU            LPSYS_CFG_RTC_TR_MNU_Msk
#define LPSYS_CFG_RTC_TR_MNT_Pos        (22U)
#define LPSYS_CFG_RTC_TR_MNT_Msk        (0x7UL << LPSYS_CFG_RTC_TR_MNT_Pos)
#define LPSYS_CFG_RTC_TR_MNT            LPSYS_CFG_RTC_TR_MNT_Msk
#define LPSYS_CFG_RTC_TR_HU_Pos         (25U)
#define LPSYS_CFG_RTC_TR_HU_Msk         (0xFUL << LPSYS_CFG_RTC_TR_HU_Pos)
#define LPSYS_CFG_RTC_TR_HU             LPSYS_CFG_RTC_TR_HU_Msk
#define LPSYS_CFG_RTC_TR_HT_Pos         (29U)
#define LPSYS_CFG_RTC_TR_HT_Msk         (0x3UL << LPSYS_CFG_RTC_TR_HT_Pos)
#define LPSYS_CFG_RTC_TR_HT             LPSYS_CFG_RTC_TR_HT_Msk
#define LPSYS_CFG_RTC_TR_PM_Pos         (31U)
#define LPSYS_CFG_RTC_TR_PM_Msk         (0x1UL << LPSYS_CFG_RTC_TR_PM_Pos)
#define LPSYS_CFG_RTC_TR_PM             LPSYS_CFG_RTC_TR_PM_Msk

/**************** Bit definition for LPSYS_CFG_RTC_DR register ****************/
#define LPSYS_CFG_RTC_DR_DU_Pos         (0U)
#define LPSYS_CFG_RTC_DR_DU_Msk         (0xFUL << LPSYS_CFG_RTC_DR_DU_Pos)
#define LPSYS_CFG_RTC_DR_DU             LPSYS_CFG_RTC_DR_DU_Msk
#define LPSYS_CFG_RTC_DR_DT_Pos         (4U)
#define LPSYS_CFG_RTC_DR_DT_Msk         (0x3UL << LPSYS_CFG_RTC_DR_DT_Pos)
#define LPSYS_CFG_RTC_DR_DT             LPSYS_CFG_RTC_DR_DT_Msk
#define LPSYS_CFG_RTC_DR_MU_Pos         (8U)
#define LPSYS_CFG_RTC_DR_MU_Msk         (0xFUL << LPSYS_CFG_RTC_DR_MU_Pos)
#define LPSYS_CFG_RTC_DR_MU             LPSYS_CFG_RTC_DR_MU_Msk
#define LPSYS_CFG_RTC_DR_MT_Pos         (12U)
#define LPSYS_CFG_RTC_DR_MT_Msk         (0x1UL << LPSYS_CFG_RTC_DR_MT_Pos)
#define LPSYS_CFG_RTC_DR_MT             LPSYS_CFG_RTC_DR_MT_Msk
#define LPSYS_CFG_RTC_DR_WD_Pos         (13U)
#define LPSYS_CFG_RTC_DR_WD_Msk         (0x7UL << LPSYS_CFG_RTC_DR_WD_Pos)
#define LPSYS_CFG_RTC_DR_WD             LPSYS_CFG_RTC_DR_WD_Msk
#define LPSYS_CFG_RTC_DR_YU_Pos         (16U)
#define LPSYS_CFG_RTC_DR_YU_Msk         (0xFUL << LPSYS_CFG_RTC_DR_YU_Pos)
#define LPSYS_CFG_RTC_DR_YU             LPSYS_CFG_RTC_DR_YU_Msk
#define LPSYS_CFG_RTC_DR_YT_Pos         (20U)
#define LPSYS_CFG_RTC_DR_YT_Msk         (0xFUL << LPSYS_CFG_RTC_DR_YT_Pos)
#define LPSYS_CFG_RTC_DR_YT             LPSYS_CFG_RTC_DR_YT_Msk
#define LPSYS_CFG_RTC_DR_CB_Pos         (24U)
#define LPSYS_CFG_RTC_DR_CB_Msk         (0x1UL << LPSYS_CFG_RTC_DR_CB_Pos)
#define LPSYS_CFG_RTC_DR_CB             LPSYS_CFG_RTC_DR_CB_Msk
#define LPSYS_CFG_RTC_DR_ERR_Pos        (31U)
#define LPSYS_CFG_RTC_DR_ERR_Msk        (0x1UL << LPSYS_CFG_RTC_DR_ERR_Pos)
#define LPSYS_CFG_RTC_DR_ERR            LPSYS_CFG_RTC_DR_ERR_Msk

/**************** Bit definition for LPSYS_CFG_LSDBGR register ****************/
#define LPSYS_CFG_LSDBGR_LS_RAM0_Pos    (0U)
#define LPSYS_CFG_LSDBGR_LS_RAM0_Msk    (0x1UL << LPSYS_CFG_LSDBGR_LS_RAM0_Pos)
#define LPSYS_CFG_LSDBGR_LS_RAM0        LPSYS_CFG_LSDBGR_LS_RAM0_Msk
#define LPSYS_CFG_LSDBGR_LS_RAM1_Pos    (1U)
#define LPSYS_CFG_LSDBGR_LS_RAM1_Msk    (0x1UL << LPSYS_CFG_LSDBGR_LS_RAM1_Pos)
#define LPSYS_CFG_LSDBGR_LS_RAM1        LPSYS_CFG_LSDBGR_LS_RAM1_Msk
#define LPSYS_CFG_LSDBGR_LS_RAM2_Pos    (2U)
#define LPSYS_CFG_LSDBGR_LS_RAM2_Msk    (0x1UL << LPSYS_CFG_LSDBGR_LS_RAM2_Pos)
#define LPSYS_CFG_LSDBGR_LS_RAM2        LPSYS_CFG_LSDBGR_LS_RAM2_Msk
#define LPSYS_CFG_LSDBGR_LS_RAM3_Pos    (3U)
#define LPSYS_CFG_LSDBGR_LS_RAM3_Msk    (0x1UL << LPSYS_CFG_LSDBGR_LS_RAM3_Pos)
#define LPSYS_CFG_LSDBGR_LS_RAM3        LPSYS_CFG_LSDBGR_LS_RAM3_Msk
#define LPSYS_CFG_LSDBGR_LS_RAM4_Pos    (4U)
#define LPSYS_CFG_LSDBGR_LS_RAM4_Msk    (0x1UL << LPSYS_CFG_LSDBGR_LS_RAM4_Pos)
#define LPSYS_CFG_LSDBGR_LS_RAM4        LPSYS_CFG_LSDBGR_LS_RAM4_Msk
#define LPSYS_CFG_LSDBGR_LS_RAM5_Pos    (5U)
#define LPSYS_CFG_LSDBGR_LS_RAM5_Msk    (0x1UL << LPSYS_CFG_LSDBGR_LS_RAM5_Pos)
#define LPSYS_CFG_LSDBGR_LS_RAM5        LPSYS_CFG_LSDBGR_LS_RAM5_Msk
#define LPSYS_CFG_LSDBGR_LS_ITCM_Pos    (6U)
#define LPSYS_CFG_LSDBGR_LS_ITCM_Msk    (0x1UL << LPSYS_CFG_LSDBGR_LS_ITCM_Pos)
#define LPSYS_CFG_LSDBGR_LS_ITCM        LPSYS_CFG_LSDBGR_LS_ITCM_Msk
#define LPSYS_CFG_LSDBGR_LS_DTCM_Pos    (7U)
#define LPSYS_CFG_LSDBGR_LS_DTCM_Msk    (0x1UL << LPSYS_CFG_LSDBGR_LS_DTCM_Pos)
#define LPSYS_CFG_LSDBGR_LS_DTCM        LPSYS_CFG_LSDBGR_LS_DTCM_Msk
#define LPSYS_CFG_LSDBGR_LS_CACHE_Pos   (8U)
#define LPSYS_CFG_LSDBGR_LS_CACHE_Msk   (0x1UL << LPSYS_CFG_LSDBGR_LS_CACHE_Pos)
#define LPSYS_CFG_LSDBGR_LS_CACHE       LPSYS_CFG_LSDBGR_LS_CACHE_Msk
#define LPSYS_CFG_LSDBGR_LS_ROM_Pos     (9U)
#define LPSYS_CFG_LSDBGR_LS_ROM_Msk     (0x1UL << LPSYS_CFG_LSDBGR_LS_ROM_Pos)
#define LPSYS_CFG_LSDBGR_LS_ROM         LPSYS_CFG_LSDBGR_LS_ROM_Msk

/***************** Bit definition for LPSYS_CFG_DBGR register *****************/

/*************** Bit definition for LPSYS_CFG_DBGCLKR register ****************/
#define LPSYS_CFG_DBGCLKR_CLK_SEL_Pos   (0U)
#define LPSYS_CFG_DBGCLKR_CLK_SEL_Msk   (0x7UL << LPSYS_CFG_DBGCLKR_CLK_SEL_Pos)
#define LPSYS_CFG_DBGCLKR_CLK_SEL       LPSYS_CFG_DBGCLKR_CLK_SEL_Msk
#define LPSYS_CFG_DBGCLKR_CLK_EN_Pos    (3U)
#define LPSYS_CFG_DBGCLKR_CLK_EN_Msk    (0x1UL << LPSYS_CFG_DBGCLKR_CLK_EN_Pos)
#define LPSYS_CFG_DBGCLKR_CLK_EN        LPSYS_CFG_DBGCLKR_CLK_EN_Msk

/**************** Bit definition for LPSYS_CFG_BISTR register *****************/
#define LPSYS_CFG_BISTR_BIST_FAIL_ITCM_Pos  (0U)
#define LPSYS_CFG_BISTR_BIST_FAIL_ITCM_Msk  (0x7FUL << LPSYS_CFG_BISTR_BIST_FAIL_ITCM_Pos)
#define LPSYS_CFG_BISTR_BIST_FAIL_ITCM  LPSYS_CFG_BISTR_BIST_FAIL_ITCM_Msk
#define LPSYS_CFG_BISTR_BIST_FAIL_DTCM_Pos  (7U)
#define LPSYS_CFG_BISTR_BIST_FAIL_DTCM_Msk  (0xFUL << LPSYS_CFG_BISTR_BIST_FAIL_DTCM_Pos)
#define LPSYS_CFG_BISTR_BIST_FAIL_DTCM  LPSYS_CFG_BISTR_BIST_FAIL_DTCM_Msk
#define LPSYS_CFG_BISTR_BIST_FAIL_RAM_Pos  (11U)
#define LPSYS_CFG_BISTR_BIST_FAIL_RAM_Msk  (0xFFUL << LPSYS_CFG_BISTR_BIST_FAIL_RAM_Pos)
#define LPSYS_CFG_BISTR_BIST_FAIL_RAM   LPSYS_CFG_BISTR_BIST_FAIL_RAM_Msk
#define LPSYS_CFG_BISTR_BIST_FAIL_CACHE_Pos  (19U)
#define LPSYS_CFG_BISTR_BIST_FAIL_CACHE_Msk  (0x1UL << LPSYS_CFG_BISTR_BIST_FAIL_CACHE_Pos)
#define LPSYS_CFG_BISTR_BIST_FAIL_CACHE  LPSYS_CFG_BISTR_BIST_FAIL_CACHE_Msk

/**************** Bit definition for LPSYS_CFG_SEAR0 register *****************/
#define LPSYS_CFG_SEAR0_SEAR0_Pos       (0U)
#define LPSYS_CFG_SEAR0_SEAR0_Msk       (0xFFFFFFFFUL << LPSYS_CFG_SEAR0_SEAR0_Pos)
#define LPSYS_CFG_SEAR0_SEAR0           LPSYS_CFG_SEAR0_SEAR0_Msk

/**************** Bit definition for LPSYS_CFG_SEAR1 register *****************/
#define LPSYS_CFG_SEAR1_SEAR1_Pos       (0U)
#define LPSYS_CFG_SEAR1_SEAR1_Msk       (0xFFFFFFFFUL << LPSYS_CFG_SEAR1_SEAR1_Pos)
#define LPSYS_CFG_SEAR1_SEAR1           LPSYS_CFG_SEAR1_SEAR1_Msk

/**************** Bit definition for LPSYS_CFG_SEAR2 register *****************/
#define LPSYS_CFG_SEAR2_SEAR2_Pos       (0U)
#define LPSYS_CFG_SEAR2_SEAR2_Msk       (0xFFFFFFFFUL << LPSYS_CFG_SEAR2_SEAR2_Pos)
#define LPSYS_CFG_SEAR2_SEAR2           LPSYS_CFG_SEAR2_SEAR2_Msk

/**************** Bit definition for LPSYS_CFG_SEAR3 register *****************/
#define LPSYS_CFG_SEAR3_SEAR3_Pos       (0U)
#define LPSYS_CFG_SEAR3_SEAR3_Msk       (0xFFFFFFFFUL << LPSYS_CFG_SEAR3_SEAR3_Pos)
#define LPSYS_CFG_SEAR3_SEAR3           LPSYS_CFG_SEAR3_SEAR3_Msk

/**************** Bit definition for LPSYS_CFG_SEAR4 register *****************/
#define LPSYS_CFG_SEAR4_SEAR4_Pos       (0U)
#define LPSYS_CFG_SEAR4_SEAR4_Msk       (0xFFFFFFFFUL << LPSYS_CFG_SEAR4_SEAR4_Pos)
#define LPSYS_CFG_SEAR4_SEAR4           LPSYS_CFG_SEAR4_SEAR4_Msk

/**************** Bit definition for LPSYS_CFG_SEAR5 register *****************/
#define LPSYS_CFG_SEAR5_SEAR5_Pos       (0U)
#define LPSYS_CFG_SEAR5_SEAR5_Msk       (0xFFFFFFFFUL << LPSYS_CFG_SEAR5_SEAR5_Pos)
#define LPSYS_CFG_SEAR5_SEAR5           LPSYS_CFG_SEAR5_SEAR5_Msk

/**************** Bit definition for LPSYS_CFG_SEAR6 register *****************/
#define LPSYS_CFG_SEAR6_SEAR6_Pos       (0U)
#define LPSYS_CFG_SEAR6_SEAR6_Msk       (0xFFFFFFFFUL << LPSYS_CFG_SEAR6_SEAR6_Pos)
#define LPSYS_CFG_SEAR6_SEAR6           LPSYS_CFG_SEAR6_SEAR6_Msk

/**************** Bit definition for LPSYS_CFG_SEAR7 register *****************/
#define LPSYS_CFG_SEAR7_SEAR7_Pos       (0U)
#define LPSYS_CFG_SEAR7_SEAR7_Msk       (0xFFFFFFFFUL << LPSYS_CFG_SEAR7_SEAR7_Pos)
#define LPSYS_CFG_SEAR7_SEAR7           LPSYS_CFG_SEAR7_SEAR7_Msk

#endif
