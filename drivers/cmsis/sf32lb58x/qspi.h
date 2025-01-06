#ifndef __QSPI_H
#define __QSPI_H

typedef struct
{
    __IO uint32_t CR;
    __IO uint32_t DR;
    __IO uint32_t DCR;
    __IO uint32_t PSCLR;
    __IO uint32_t SR;
    __IO uint32_t SCR;
    __IO uint32_t CMDR1;
    __IO uint32_t AR1;
    __IO uint32_t ABR1;
    __IO uint32_t DLR1;
    __IO uint32_t CCR1;
    __IO uint32_t CMDR2;
    __IO uint32_t AR2;
    __IO uint32_t ABR2;
    __IO uint32_t DLR2;
    __IO uint32_t CCR2;
    __IO uint32_t HCMDR;
    __IO uint32_t HRABR;
    __IO uint32_t HRCCR;
    __IO uint32_t HWABR;
    __IO uint32_t HWCCR;
    __IO uint32_t FIFOCR;
    __IO uint32_t MISCR;
    __IO uint32_t CTRSAR;
    __IO uint32_t CTREAR;
    __IO uint32_t NONCEA;
    __IO uint32_t NONCEB;
    __IO uint32_t AASAR;
    __IO uint32_t AAEAR;
    __IO uint32_t AAOAR;
    __IO uint32_t CIR;
    __IO uint32_t SMR;
    __IO uint32_t SMKR;
    __IO uint32_t TIMR;
    __IO uint32_t WDTR;
    __IO uint32_t PRSAR;
    __IO uint32_t PREAR;
} QSPI_TypeDef;


/******************** Bit definition for QSPI_CR register *********************/
#define QSPI_CR_EN_Pos                  (0U)
#define QSPI_CR_EN_Msk                  (0x1UL << QSPI_CR_EN_Pos)
#define QSPI_CR_EN                      QSPI_CR_EN_Msk
#define QSPI_CR_WPE_Pos                 (1U)
#define QSPI_CR_WPE_Msk                 (0x1UL << QSPI_CR_WPE_Pos)
#define QSPI_CR_WPE                     QSPI_CR_WPE_Msk
#define QSPI_CR_WP_Pos                  (2U)
#define QSPI_CR_WP_Msk                  (0x1UL << QSPI_CR_WP_Pos)
#define QSPI_CR_WP                      QSPI_CR_WP_Msk
#define QSPI_CR_HOLDE_Pos               (3U)
#define QSPI_CR_HOLDE_Msk               (0x1UL << QSPI_CR_HOLDE_Pos)
#define QSPI_CR_HOLDE                   QSPI_CR_HOLDE_Msk
#define QSPI_CR_HOLD_Pos                (4U)
#define QSPI_CR_HOLD_Msk                (0x1UL << QSPI_CR_HOLD_Pos)
#define QSPI_CR_HOLD                    QSPI_CR_HOLD_Msk
#define QSPI_CR_DMAE_Pos                (5U)
#define QSPI_CR_DMAE_Msk                (0x1UL << QSPI_CR_DMAE_Pos)
#define QSPI_CR_DMAE                    QSPI_CR_DMAE_Msk
#define QSPI_CR_CTRE_Pos                (6U)
#define QSPI_CR_CTRE_Msk                (0x1UL << QSPI_CR_CTRE_Pos)
#define QSPI_CR_CTRE                    QSPI_CR_CTRE_Msk
#define QSPI_CR_CTRM_Pos                (7U)
#define QSPI_CR_CTRM_Msk                (0x1UL << QSPI_CR_CTRM_Pos)
#define QSPI_CR_CTRM                    QSPI_CR_CTRM_Msk
#define QSPI_CR_TCIE_Pos                (8U)
#define QSPI_CR_TCIE_Msk                (0x1UL << QSPI_CR_TCIE_Pos)
#define QSPI_CR_TCIE                    QSPI_CR_TCIE_Msk
#define QSPI_CR_FUIE_Pos                (9U)
#define QSPI_CR_FUIE_Msk                (0x1UL << QSPI_CR_FUIE_Pos)
#define QSPI_CR_FUIE                    QSPI_CR_FUIE_Msk
#define QSPI_CR_FOIE_Pos                (10U)
#define QSPI_CR_FOIE_Msk                (0x1UL << QSPI_CR_FOIE_Pos)
#define QSPI_CR_FOIE                    QSPI_CR_FOIE_Msk
#define QSPI_CR_SMIE_Pos                (11U)
#define QSPI_CR_SMIE_Msk                (0x1UL << QSPI_CR_SMIE_Pos)
#define QSPI_CR_SMIE                    QSPI_CR_SMIE_Msk
#define QSPI_CR_CSVIE_Pos               (12U)
#define QSPI_CR_CSVIE_Msk               (0x1UL << QSPI_CR_CSVIE_Pos)
#define QSPI_CR_CSVIE                   QSPI_CR_CSVIE_Msk
#define QSPI_CR_RBXIE_Pos               (13U)
#define QSPI_CR_RBXIE_Msk               (0x1UL << QSPI_CR_RBXIE_Pos)
#define QSPI_CR_RBXIE                   QSPI_CR_RBXIE_Msk
#define QSPI_CR_CMD2E_Pos               (16U)
#define QSPI_CR_CMD2E_Msk               (0x1UL << QSPI_CR_CMD2E_Pos)
#define QSPI_CR_CMD2E                   QSPI_CR_CMD2E_Msk
#define QSPI_CR_SME1_Pos                (17U)
#define QSPI_CR_SME1_Msk                (0x1UL << QSPI_CR_SME1_Pos)
#define QSPI_CR_SME1                    QSPI_CR_SME1_Msk
#define QSPI_CR_SME2_Pos                (18U)
#define QSPI_CR_SME2_Msk                (0x1UL << QSPI_CR_SME2_Pos)
#define QSPI_CR_SME2                    QSPI_CR_SME2_Msk
#define QSPI_CR_SMM_Pos                 (19U)
#define QSPI_CR_SMM_Msk                 (0x1UL << QSPI_CR_SMM_Pos)
#define QSPI_CR_SMM                     QSPI_CR_SMM_Msk
#define QSPI_CR_HWIFE_Pos               (20U)
#define QSPI_CR_HWIFE_Msk               (0x1UL << QSPI_CR_HWIFE_Pos)
#define QSPI_CR_HWIFE                   QSPI_CR_HWIFE_Msk
#define QSPI_CR_OPIE_Pos                (21U)
#define QSPI_CR_OPIE_Msk                (0x1UL << QSPI_CR_OPIE_Pos)
#define QSPI_CR_OPIE                    QSPI_CR_OPIE_Msk
#define QSPI_CR_PREFE_Pos               (22U)
#define QSPI_CR_PREFE_Msk               (0x1UL << QSPI_CR_PREFE_Pos)
#define QSPI_CR_PREFE                   QSPI_CR_PREFE_Msk
#define QSPI_CR_MX16_Pos                (23U)
#define QSPI_CR_MX16_Msk                (0x1UL << QSPI_CR_MX16_Pos)
#define QSPI_CR_MX16                    QSPI_CR_MX16_Msk
#define QSPI_CR_ABORT_Pos               (31U)
#define QSPI_CR_ABORT_Msk               (0x1UL << QSPI_CR_ABORT_Pos)
#define QSPI_CR_ABORT                   QSPI_CR_ABORT_Msk

/******************** Bit definition for QSPI_DR register *********************/
#define QSPI_DR_DATA_Pos                (0U)
#define QSPI_DR_DATA_Msk                (0xFFFFFFFFUL << QSPI_DR_DATA_Pos)
#define QSPI_DR_DATA                    QSPI_DR_DATA_Msk

/******************** Bit definition for QSPI_DCR register ********************/
#define QSPI_DCR_DFM_Pos                (0U)
#define QSPI_DCR_DFM_Msk                (0x1UL << QSPI_DCR_DFM_Pos)
#define QSPI_DCR_DFM                    QSPI_DCR_DFM_Msk
#define QSPI_DCR_RBSIZE_Pos             (1U)
#define QSPI_DCR_RBSIZE_Msk             (0x7UL << QSPI_DCR_RBSIZE_Pos)
#define QSPI_DCR_RBSIZE                 QSPI_DCR_RBSIZE_Msk
#define QSPI_DCR_DQSE_Pos               (4U)
#define QSPI_DCR_DQSE_Msk               (0x1UL << QSPI_DCR_DQSE_Pos)
#define QSPI_DCR_DQSE                   QSPI_DCR_DQSE_Msk
#define QSPI_DCR_HYPER_Pos              (5U)
#define QSPI_DCR_HYPER_Msk              (0x1UL << QSPI_DCR_HYPER_Pos)
#define QSPI_DCR_HYPER                  QSPI_DCR_HYPER_Msk
#define QSPI_DCR_XLEGACY_Pos            (6U)
#define QSPI_DCR_XLEGACY_Msk            (0x1UL << QSPI_DCR_XLEGACY_Pos)
#define QSPI_DCR_XLEGACY                QSPI_DCR_XLEGACY_Msk
#define QSPI_DCR_CSLMAX_Pos             (7U)
#define QSPI_DCR_CSLMAX_Msk             (0xFFFUL << QSPI_DCR_CSLMAX_Pos)
#define QSPI_DCR_CSLMAX                 QSPI_DCR_CSLMAX_Msk
#define QSPI_DCR_CSLMIN_Pos             (19U)
#define QSPI_DCR_CSLMIN_Msk             (0xFUL << QSPI_DCR_CSLMIN_Pos)
#define QSPI_DCR_CSLMIN                 QSPI_DCR_CSLMIN_Msk
#define QSPI_DCR_CSHMIN_Pos             (23U)
#define QSPI_DCR_CSHMIN_Msk             (0x7UL << QSPI_DCR_CSHMIN_Pos)
#define QSPI_DCR_CSHMIN                 QSPI_DCR_CSHMIN_Msk
#define QSPI_DCR_TRCMIN_Pos             (26U)
#define QSPI_DCR_TRCMIN_Msk             (0x1FUL << QSPI_DCR_TRCMIN_Pos)
#define QSPI_DCR_TRCMIN                 QSPI_DCR_TRCMIN_Msk
#define QSPI_DCR_FIXLAT_Pos             (31U)
#define QSPI_DCR_FIXLAT_Msk             (0x1UL << QSPI_DCR_FIXLAT_Pos)
#define QSPI_DCR_FIXLAT                 QSPI_DCR_FIXLAT_Msk

/******************* Bit definition for QSPI_PSCLR register *******************/
#define QSPI_PSCLR_DIV_Pos              (0U)
#define QSPI_PSCLR_DIV_Msk              (0xFFUL << QSPI_PSCLR_DIV_Pos)
#define QSPI_PSCLR_DIV                  QSPI_PSCLR_DIV_Msk

/******************** Bit definition for QSPI_SR register *********************/
#define QSPI_SR_TCF_Pos                 (0U)
#define QSPI_SR_TCF_Msk                 (0x1UL << QSPI_SR_TCF_Pos)
#define QSPI_SR_TCF                     QSPI_SR_TCF_Msk
#define QSPI_SR_FUF_Pos                 (1U)
#define QSPI_SR_FUF_Msk                 (0x1UL << QSPI_SR_FUF_Pos)
#define QSPI_SR_FUF                     QSPI_SR_FUF_Msk
#define QSPI_SR_FOF_Pos                 (2U)
#define QSPI_SR_FOF_Msk                 (0x1UL << QSPI_SR_FOF_Pos)
#define QSPI_SR_FOF                     QSPI_SR_FOF_Msk
#define QSPI_SR_SMF_Pos                 (3U)
#define QSPI_SR_SMF_Msk                 (0x1UL << QSPI_SR_SMF_Pos)
#define QSPI_SR_SMF                     QSPI_SR_SMF_Msk
#define QSPI_SR_CSVF_Pos                (4U)
#define QSPI_SR_CSVF_Msk                (0x1UL << QSPI_SR_CSVF_Pos)
#define QSPI_SR_CSVF                    QSPI_SR_CSVF_Msk
#define QSPI_SR_RBXF_Pos                (5U)
#define QSPI_SR_RBXF_Msk                (0x1UL << QSPI_SR_RBXF_Pos)
#define QSPI_SR_RBXF                    QSPI_SR_RBXF_Msk
#define QSPI_SR_BUSY_Pos                (31U)
#define QSPI_SR_BUSY_Msk                (0x1UL << QSPI_SR_BUSY_Pos)
#define QSPI_SR_BUSY                    QSPI_SR_BUSY_Msk

/******************** Bit definition for QSPI_SCR register ********************/
#define QSPI_SCR_TCFC_Pos               (0U)
#define QSPI_SCR_TCFC_Msk               (0x1UL << QSPI_SCR_TCFC_Pos)
#define QSPI_SCR_TCFC                   QSPI_SCR_TCFC_Msk
#define QSPI_SCR_FUFC_Pos               (1U)
#define QSPI_SCR_FUFC_Msk               (0x1UL << QSPI_SCR_FUFC_Pos)
#define QSPI_SCR_FUFC                   QSPI_SCR_FUFC_Msk
#define QSPI_SCR_FOFC_Pos               (2U)
#define QSPI_SCR_FOFC_Msk               (0x1UL << QSPI_SCR_FOFC_Pos)
#define QSPI_SCR_FOFC                   QSPI_SCR_FOFC_Msk
#define QSPI_SCR_SMFC_Pos               (3U)
#define QSPI_SCR_SMFC_Msk               (0x1UL << QSPI_SCR_SMFC_Pos)
#define QSPI_SCR_SMFC                   QSPI_SCR_SMFC_Msk
#define QSPI_SCR_CSVFC_Pos              (4U)
#define QSPI_SCR_CSVFC_Msk              (0x1UL << QSPI_SCR_CSVFC_Pos)
#define QSPI_SCR_CSVFC                  QSPI_SCR_CSVFC_Msk
#define QSPI_SCR_RBXFC_Pos              (5U)
#define QSPI_SCR_RBXFC_Msk              (0x1UL << QSPI_SCR_RBXFC_Pos)
#define QSPI_SCR_RBXFC                  QSPI_SCR_RBXFC_Msk

/******************* Bit definition for QSPI_CMDR1 register *******************/
#define QSPI_CMDR1_CMD_Pos              (0U)
#define QSPI_CMDR1_CMD_Msk              (0xFFUL << QSPI_CMDR1_CMD_Pos)
#define QSPI_CMDR1_CMD                  QSPI_CMDR1_CMD_Msk

/******************** Bit definition for QSPI_AR1 register ********************/
#define QSPI_AR1_ADDR_Pos               (0U)
#define QSPI_AR1_ADDR_Msk               (0xFFFFFFFFUL << QSPI_AR1_ADDR_Pos)
#define QSPI_AR1_ADDR                   QSPI_AR1_ADDR_Msk

/******************* Bit definition for QSPI_ABR1 register ********************/
#define QSPI_ABR1_ABYTE_Pos             (0U)
#define QSPI_ABR1_ABYTE_Msk             (0xFFFFFFFFUL << QSPI_ABR1_ABYTE_Pos)
#define QSPI_ABR1_ABYTE                 QSPI_ABR1_ABYTE_Msk

/******************* Bit definition for QSPI_DLR1 register ********************/
#define QSPI_DLR1_DLEN_Pos              (0U)
#define QSPI_DLR1_DLEN_Msk              (0xFFFFFUL << QSPI_DLR1_DLEN_Pos)
#define QSPI_DLR1_DLEN                  QSPI_DLR1_DLEN_Msk

/******************* Bit definition for QSPI_CCR1 register ********************/
#define QSPI_CCR1_IMODE_Pos             (0U)
#define QSPI_CCR1_IMODE_Msk             (0x7UL << QSPI_CCR1_IMODE_Pos)
#define QSPI_CCR1_IMODE                 QSPI_CCR1_IMODE_Msk
#define QSPI_CCR1_ADMODE_Pos            (3U)
#define QSPI_CCR1_ADMODE_Msk            (0x7UL << QSPI_CCR1_ADMODE_Pos)
#define QSPI_CCR1_ADMODE                QSPI_CCR1_ADMODE_Msk
#define QSPI_CCR1_ADSIZE_Pos            (6U)
#define QSPI_CCR1_ADSIZE_Msk            (0x3UL << QSPI_CCR1_ADSIZE_Pos)
#define QSPI_CCR1_ADSIZE                QSPI_CCR1_ADSIZE_Msk
#define QSPI_CCR1_ABMODE_Pos            (8U)
#define QSPI_CCR1_ABMODE_Msk            (0x7UL << QSPI_CCR1_ABMODE_Pos)
#define QSPI_CCR1_ABMODE                QSPI_CCR1_ABMODE_Msk
#define QSPI_CCR1_ABSIZE_Pos            (11U)
#define QSPI_CCR1_ABSIZE_Msk            (0x3UL << QSPI_CCR1_ABSIZE_Pos)
#define QSPI_CCR1_ABSIZE                QSPI_CCR1_ABSIZE_Msk
#define QSPI_CCR1_DCYC_Pos              (13U)
#define QSPI_CCR1_DCYC_Msk              (0x1FUL << QSPI_CCR1_DCYC_Pos)
#define QSPI_CCR1_DCYC                  QSPI_CCR1_DCYC_Msk
#define QSPI_CCR1_DMODE_Pos             (18U)
#define QSPI_CCR1_DMODE_Msk             (0x7UL << QSPI_CCR1_DMODE_Pos)
#define QSPI_CCR1_DMODE                 QSPI_CCR1_DMODE_Msk
#define QSPI_CCR1_FMODE_Pos             (21U)
#define QSPI_CCR1_FMODE_Msk             (0x1UL << QSPI_CCR1_FMODE_Pos)
#define QSPI_CCR1_FMODE                 QSPI_CCR1_FMODE_Msk

/******************* Bit definition for QSPI_CMDR2 register *******************/
#define QSPI_CMDR2_CMD_Pos              (0U)
#define QSPI_CMDR2_CMD_Msk              (0xFFUL << QSPI_CMDR2_CMD_Pos)
#define QSPI_CMDR2_CMD                  QSPI_CMDR2_CMD_Msk

/******************** Bit definition for QSPI_AR2 register ********************/
#define QSPI_AR2_ADDR_Pos               (0U)
#define QSPI_AR2_ADDR_Msk               (0xFFFFFFFFUL << QSPI_AR2_ADDR_Pos)
#define QSPI_AR2_ADDR                   QSPI_AR2_ADDR_Msk

/******************* Bit definition for QSPI_ABR2 register ********************/
#define QSPI_ABR2_ABYTE_Pos             (0U)
#define QSPI_ABR2_ABYTE_Msk             (0xFFFFFFFFUL << QSPI_ABR2_ABYTE_Pos)
#define QSPI_ABR2_ABYTE                 QSPI_ABR2_ABYTE_Msk

/******************* Bit definition for QSPI_DLR2 register ********************/
#define QSPI_DLR2_DLEN_Pos              (0U)
#define QSPI_DLR2_DLEN_Msk              (0xFFFFFUL << QSPI_DLR2_DLEN_Pos)
#define QSPI_DLR2_DLEN                  QSPI_DLR2_DLEN_Msk

/******************* Bit definition for QSPI_CCR2 register ********************/
#define QSPI_CCR2_IMODE_Pos             (0U)
#define QSPI_CCR2_IMODE_Msk             (0x7UL << QSPI_CCR2_IMODE_Pos)
#define QSPI_CCR2_IMODE                 QSPI_CCR2_IMODE_Msk
#define QSPI_CCR2_ADMODE_Pos            (3U)
#define QSPI_CCR2_ADMODE_Msk            (0x7UL << QSPI_CCR2_ADMODE_Pos)
#define QSPI_CCR2_ADMODE                QSPI_CCR2_ADMODE_Msk
#define QSPI_CCR2_ADSIZE_Pos            (6U)
#define QSPI_CCR2_ADSIZE_Msk            (0x3UL << QSPI_CCR2_ADSIZE_Pos)
#define QSPI_CCR2_ADSIZE                QSPI_CCR2_ADSIZE_Msk
#define QSPI_CCR2_ABMODE_Pos            (8U)
#define QSPI_CCR2_ABMODE_Msk            (0x7UL << QSPI_CCR2_ABMODE_Pos)
#define QSPI_CCR2_ABMODE                QSPI_CCR2_ABMODE_Msk
#define QSPI_CCR2_ABSIZE_Pos            (11U)
#define QSPI_CCR2_ABSIZE_Msk            (0x3UL << QSPI_CCR2_ABSIZE_Pos)
#define QSPI_CCR2_ABSIZE                QSPI_CCR2_ABSIZE_Msk
#define QSPI_CCR2_DCYC_Pos              (13U)
#define QSPI_CCR2_DCYC_Msk              (0x1FUL << QSPI_CCR2_DCYC_Pos)
#define QSPI_CCR2_DCYC                  QSPI_CCR2_DCYC_Msk
#define QSPI_CCR2_DMODE_Pos             (18U)
#define QSPI_CCR2_DMODE_Msk             (0x7UL << QSPI_CCR2_DMODE_Pos)
#define QSPI_CCR2_DMODE                 QSPI_CCR2_DMODE_Msk
#define QSPI_CCR2_FMODE_Pos             (21U)
#define QSPI_CCR2_FMODE_Msk             (0x1UL << QSPI_CCR2_FMODE_Pos)
#define QSPI_CCR2_FMODE                 QSPI_CCR2_FMODE_Msk

/******************* Bit definition for QSPI_HCMDR register *******************/
#define QSPI_HCMDR_RCMD_Pos             (0U)
#define QSPI_HCMDR_RCMD_Msk             (0xFFUL << QSPI_HCMDR_RCMD_Pos)
#define QSPI_HCMDR_RCMD                 QSPI_HCMDR_RCMD_Msk
#define QSPI_HCMDR_WCMD_Pos             (8U)
#define QSPI_HCMDR_WCMD_Msk             (0xFFUL << QSPI_HCMDR_WCMD_Pos)
#define QSPI_HCMDR_WCMD                 QSPI_HCMDR_WCMD_Msk

/******************* Bit definition for QSPI_HRABR register *******************/
#define QSPI_HRABR_ABYTE_Pos            (0U)
#define QSPI_HRABR_ABYTE_Msk            (0xFFFFFFFFUL << QSPI_HRABR_ABYTE_Pos)
#define QSPI_HRABR_ABYTE                QSPI_HRABR_ABYTE_Msk

/******************* Bit definition for QSPI_HRCCR register *******************/
#define QSPI_HRCCR_IMODE_Pos            (0U)
#define QSPI_HRCCR_IMODE_Msk            (0x7UL << QSPI_HRCCR_IMODE_Pos)
#define QSPI_HRCCR_IMODE                QSPI_HRCCR_IMODE_Msk
#define QSPI_HRCCR_ADMODE_Pos           (3U)
#define QSPI_HRCCR_ADMODE_Msk           (0x7UL << QSPI_HRCCR_ADMODE_Pos)
#define QSPI_HRCCR_ADMODE               QSPI_HRCCR_ADMODE_Msk
#define QSPI_HRCCR_ADSIZE_Pos           (6U)
#define QSPI_HRCCR_ADSIZE_Msk           (0x3UL << QSPI_HRCCR_ADSIZE_Pos)
#define QSPI_HRCCR_ADSIZE               QSPI_HRCCR_ADSIZE_Msk
#define QSPI_HRCCR_ABMODE_Pos           (8U)
#define QSPI_HRCCR_ABMODE_Msk           (0x7UL << QSPI_HRCCR_ABMODE_Pos)
#define QSPI_HRCCR_ABMODE               QSPI_HRCCR_ABMODE_Msk
#define QSPI_HRCCR_ABSIZE_Pos           (11U)
#define QSPI_HRCCR_ABSIZE_Msk           (0x3UL << QSPI_HRCCR_ABSIZE_Pos)
#define QSPI_HRCCR_ABSIZE               QSPI_HRCCR_ABSIZE_Msk
#define QSPI_HRCCR_DCYC_Pos             (13U)
#define QSPI_HRCCR_DCYC_Msk             (0x1FUL << QSPI_HRCCR_DCYC_Pos)
#define QSPI_HRCCR_DCYC                 QSPI_HRCCR_DCYC_Msk
#define QSPI_HRCCR_DMODE_Pos            (18U)
#define QSPI_HRCCR_DMODE_Msk            (0x7UL << QSPI_HRCCR_DMODE_Pos)
#define QSPI_HRCCR_DMODE                QSPI_HRCCR_DMODE_Msk

/******************* Bit definition for QSPI_HWABR register *******************/
#define QSPI_HWABR_ABYTE_Pos            (0U)
#define QSPI_HWABR_ABYTE_Msk            (0xFFFFFFFFUL << QSPI_HWABR_ABYTE_Pos)
#define QSPI_HWABR_ABYTE                QSPI_HWABR_ABYTE_Msk

/******************* Bit definition for QSPI_HWCCR register *******************/
#define QSPI_HWCCR_IMODE_Pos            (0U)
#define QSPI_HWCCR_IMODE_Msk            (0x7UL << QSPI_HWCCR_IMODE_Pos)
#define QSPI_HWCCR_IMODE                QSPI_HWCCR_IMODE_Msk
#define QSPI_HWCCR_ADMODE_Pos           (3U)
#define QSPI_HWCCR_ADMODE_Msk           (0x7UL << QSPI_HWCCR_ADMODE_Pos)
#define QSPI_HWCCR_ADMODE               QSPI_HWCCR_ADMODE_Msk
#define QSPI_HWCCR_ADSIZE_Pos           (6U)
#define QSPI_HWCCR_ADSIZE_Msk           (0x3UL << QSPI_HWCCR_ADSIZE_Pos)
#define QSPI_HWCCR_ADSIZE               QSPI_HWCCR_ADSIZE_Msk
#define QSPI_HWCCR_ABMODE_Pos           (8U)
#define QSPI_HWCCR_ABMODE_Msk           (0x7UL << QSPI_HWCCR_ABMODE_Pos)
#define QSPI_HWCCR_ABMODE               QSPI_HWCCR_ABMODE_Msk
#define QSPI_HWCCR_ABSIZE_Pos           (11U)
#define QSPI_HWCCR_ABSIZE_Msk           (0x3UL << QSPI_HWCCR_ABSIZE_Pos)
#define QSPI_HWCCR_ABSIZE               QSPI_HWCCR_ABSIZE_Msk
#define QSPI_HWCCR_DCYC_Pos             (13U)
#define QSPI_HWCCR_DCYC_Msk             (0x1FUL << QSPI_HWCCR_DCYC_Pos)
#define QSPI_HWCCR_DCYC                 QSPI_HWCCR_DCYC_Msk
#define QSPI_HWCCR_DMODE_Pos            (18U)
#define QSPI_HWCCR_DMODE_Msk            (0x7UL << QSPI_HWCCR_DMODE_Pos)
#define QSPI_HWCCR_DMODE                QSPI_HWCCR_DMODE_Msk

/****************** Bit definition for QSPI_FIFOCR register *******************/
#define QSPI_FIFOCR_RXCLR_Pos           (0U)
#define QSPI_FIFOCR_RXCLR_Msk           (0x1UL << QSPI_FIFOCR_RXCLR_Pos)
#define QSPI_FIFOCR_RXCLR               QSPI_FIFOCR_RXCLR_Msk
#define QSPI_FIFOCR_RXE_Pos             (1U)
#define QSPI_FIFOCR_RXE_Msk             (0x1UL << QSPI_FIFOCR_RXE_Pos)
#define QSPI_FIFOCR_RXE                 QSPI_FIFOCR_RXE_Msk
#define QSPI_FIFOCR_TXCLR_Pos           (8U)
#define QSPI_FIFOCR_TXCLR_Msk           (0x1UL << QSPI_FIFOCR_TXCLR_Pos)
#define QSPI_FIFOCR_TXCLR               QSPI_FIFOCR_TXCLR_Msk
#define QSPI_FIFOCR_TXF_Pos             (9U)
#define QSPI_FIFOCR_TXF_Msk             (0x1UL << QSPI_FIFOCR_TXF_Pos)
#define QSPI_FIFOCR_TXF                 QSPI_FIFOCR_TXF_Msk
#define QSPI_FIFOCR_TXSLOTS_Pos         (10U)
#define QSPI_FIFOCR_TXSLOTS_Msk         (0x1FUL << QSPI_FIFOCR_TXSLOTS_Pos)
#define QSPI_FIFOCR_TXSLOTS             QSPI_FIFOCR_TXSLOTS_Msk

/******************* Bit definition for QSPI_MISCR register *******************/
#define QSPI_MISCR_RXCLKDLY_Pos         (0U)
#define QSPI_MISCR_RXCLKDLY_Msk         (0xFUL << QSPI_MISCR_RXCLKDLY_Pos)
#define QSPI_MISCR_RXCLKDLY             QSPI_MISCR_RXCLKDLY_Msk
#define QSPI_MISCR_RXCLKINV_Pos         (4U)
#define QSPI_MISCR_RXCLKINV_Msk         (0x1UL << QSPI_MISCR_RXCLKINV_Pos)
#define QSPI_MISCR_RXCLKINV             QSPI_MISCR_RXCLKINV_Msk
#define QSPI_MISCR_DTRPRE_Pos           (5U)
#define QSPI_MISCR_DTRPRE_Msk           (0x1UL << QSPI_MISCR_DTRPRE_Pos)
#define QSPI_MISCR_DTRPRE               QSPI_MISCR_DTRPRE_Msk
#define QSPI_MISCR_DQSDLY_Pos           (8U)
#define QSPI_MISCR_DQSDLY_Msk           (0xFUL << QSPI_MISCR_DQSDLY_Pos)
#define QSPI_MISCR_DQSDLY               QSPI_MISCR_DQSDLY_Msk
#define QSPI_MISCR_SCKDLY_Pos           (12U)
#define QSPI_MISCR_SCKDLY_Msk           (0xFUL << QSPI_MISCR_SCKDLY_Pos)
#define QSPI_MISCR_SCKDLY               QSPI_MISCR_SCKDLY_Msk
#define QSPI_MISCR_SCKINV_Pos           (16U)
#define QSPI_MISCR_SCKINV_Msk           (0x1UL << QSPI_MISCR_SCKINV_Pos)
#define QSPI_MISCR_SCKINV               QSPI_MISCR_SCKINV_Msk
#define QSPI_MISCR_DBGSEL_Pos           (28U)
#define QSPI_MISCR_DBGSEL_Msk           (0xFUL << QSPI_MISCR_DBGSEL_Pos)
#define QSPI_MISCR_DBGSEL               QSPI_MISCR_DBGSEL_Msk

/****************** Bit definition for QSPI_CTRSAR register *******************/
#define QSPI_CTRSAR_SA_Pos              (10U)
#define QSPI_CTRSAR_SA_Msk              (0x3FFFFFUL << QSPI_CTRSAR_SA_Pos)
#define QSPI_CTRSAR_SA                  QSPI_CTRSAR_SA_Msk

/****************** Bit definition for QSPI_CTREAR register *******************/
#define QSPI_CTREAR_EA_Pos              (10U)
#define QSPI_CTREAR_EA_Msk              (0x3FFFFFUL << QSPI_CTREAR_EA_Pos)
#define QSPI_CTREAR_EA                  QSPI_CTREAR_EA_Msk

/****************** Bit definition for QSPI_NONCEA register *******************/
#define QSPI_NONCEA_NONCEA_Pos          (0U)
#define QSPI_NONCEA_NONCEA_Msk          (0xFFFFFFFFUL << QSPI_NONCEA_NONCEA_Pos)
#define QSPI_NONCEA_NONCEA              QSPI_NONCEA_NONCEA_Msk

/****************** Bit definition for QSPI_NONCEB register *******************/
#define QSPI_NONCEB_NONCEB_Pos          (0U)
#define QSPI_NONCEB_NONCEB_Msk          (0xFFFFFFFFUL << QSPI_NONCEB_NONCEB_Pos)
#define QSPI_NONCEB_NONCEB              QSPI_NONCEB_NONCEB_Msk

/******************* Bit definition for QSPI_AASAR register *******************/
#define QSPI_AASAR_SA_Pos               (10U)
#define QSPI_AASAR_SA_Msk               (0x3FFFFFUL << QSPI_AASAR_SA_Pos)
#define QSPI_AASAR_SA                   QSPI_AASAR_SA_Msk

/******************* Bit definition for QSPI_AAEAR register *******************/
#define QSPI_AAEAR_EA_Pos               (10U)
#define QSPI_AAEAR_EA_Msk               (0x3FFFFFUL << QSPI_AAEAR_EA_Pos)
#define QSPI_AAEAR_EA                   QSPI_AAEAR_EA_Msk

/******************* Bit definition for QSPI_AAOAR register *******************/
#define QSPI_AAOAR_OA_Pos               (10U)
#define QSPI_AAOAR_OA_Msk               (0x3FFFFFUL << QSPI_AAOAR_OA_Pos)
#define QSPI_AAOAR_OA                   QSPI_AAOAR_OA_Msk

/******************** Bit definition for QSPI_CIR register ********************/
#define QSPI_CIR_INTERVAL1_Pos          (0U)
#define QSPI_CIR_INTERVAL1_Msk          (0xFFFFUL << QSPI_CIR_INTERVAL1_Pos)
#define QSPI_CIR_INTERVAL1              QSPI_CIR_INTERVAL1_Msk
#define QSPI_CIR_INTERVAL2_Pos          (16U)
#define QSPI_CIR_INTERVAL2_Msk          (0xFFFFUL << QSPI_CIR_INTERVAL2_Pos)
#define QSPI_CIR_INTERVAL2              QSPI_CIR_INTERVAL2_Msk

/******************** Bit definition for QSPI_SMR register ********************/
#define QSPI_SMR_STATUS_Pos             (0U)
#define QSPI_SMR_STATUS_Msk             (0xFFFFFFFFUL << QSPI_SMR_STATUS_Pos)
#define QSPI_SMR_STATUS                 QSPI_SMR_STATUS_Msk

/******************* Bit definition for QSPI_SMKR register ********************/
#define QSPI_SMKR_MASK_Pos              (0U)
#define QSPI_SMKR_MASK_Msk              (0xFFFFFFFFUL << QSPI_SMKR_MASK_Pos)
#define QSPI_SMKR_MASK                  QSPI_SMKR_MASK_Msk

/******************* Bit definition for QSPI_TIMR register ********************/
#define QSPI_TIMR_TIMEOUT_Pos           (0U)
#define QSPI_TIMR_TIMEOUT_Msk           (0xFFFFUL << QSPI_TIMR_TIMEOUT_Pos)
#define QSPI_TIMR_TIMEOUT               QSPI_TIMR_TIMEOUT_Msk

/******************* Bit definition for QSPI_WDTR register ********************/
#define QSPI_WDTR_TIMEOUT_Pos           (0U)
#define QSPI_WDTR_TIMEOUT_Msk           (0x3FFUL << QSPI_WDTR_TIMEOUT_Pos)
#define QSPI_WDTR_TIMEOUT               QSPI_WDTR_TIMEOUT_Msk
#define QSPI_WDTR_TOF_Pos               (31U)
#define QSPI_WDTR_TOF_Msk               (0x1UL << QSPI_WDTR_TOF_Pos)
#define QSPI_WDTR_TOF                   QSPI_WDTR_TOF_Msk

/******************* Bit definition for QSPI_PRSAR register *******************/
#define QSPI_PRSAR_SA_Pos               (10U)
#define QSPI_PRSAR_SA_Msk               (0x3FFFFFUL << QSPI_PRSAR_SA_Pos)
#define QSPI_PRSAR_SA                   QSPI_PRSAR_SA_Msk

/******************* Bit definition for QSPI_PREAR register *******************/
#define QSPI_PREAR_EA_Pos               (10U)
#define QSPI_PREAR_EA_Msk               (0x3FFFFFUL << QSPI_PREAR_EA_Pos)
#define QSPI_PREAR_EA                   QSPI_PREAR_EA_Msk

#endif
