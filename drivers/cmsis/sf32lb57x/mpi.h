/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __MPI_H
#define __MPI_H

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
    __IO uint32_t CALCR;
    __IO uint32_t CALRR;
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
    __IO uint32_t CR2;
    __IO uint32_t DCR2;
    __IO uint32_t CTRSAR2;
    __IO uint32_t CTREAR2;
    __IO uint32_t NONCEA2;
    __IO uint32_t NONCEB2;
    __IO uint32_t DBGR;
} MPI_TypeDef;


/********************* Bit definition for MPI_CR register *********************/
#define MPI_CR_EN_Pos                   (0U)
#define MPI_CR_EN_Msk                   (0x1UL << MPI_CR_EN_Pos)
#define MPI_CR_EN                       MPI_CR_EN_Msk
#define MPI_CR_WPE_Pos                  (1U)
#define MPI_CR_WPE_Msk                  (0x1UL << MPI_CR_WPE_Pos)
#define MPI_CR_WPE                      MPI_CR_WPE_Msk
#define MPI_CR_WP_Pos                   (2U)
#define MPI_CR_WP_Msk                   (0x1UL << MPI_CR_WP_Pos)
#define MPI_CR_WP                       MPI_CR_WP_Msk
#define MPI_CR_HOLDE_Pos                (3U)
#define MPI_CR_HOLDE_Msk                (0x1UL << MPI_CR_HOLDE_Pos)
#define MPI_CR_HOLDE                    MPI_CR_HOLDE_Msk
#define MPI_CR_HOLD_Pos                 (4U)
#define MPI_CR_HOLD_Msk                 (0x1UL << MPI_CR_HOLD_Pos)
#define MPI_CR_HOLD                     MPI_CR_HOLD_Msk
#define MPI_CR_DMAE_Pos                 (5U)
#define MPI_CR_DMAE_Msk                 (0x1UL << MPI_CR_DMAE_Pos)
#define MPI_CR_DMAE                     MPI_CR_DMAE_Msk
#define MPI_CR_CTRE_Pos                 (6U)
#define MPI_CR_CTRE_Msk                 (0x1UL << MPI_CR_CTRE_Pos)
#define MPI_CR_CTRE                     MPI_CR_CTRE_Msk
#define MPI_CR_CTRM_Pos                 (7U)
#define MPI_CR_CTRM_Msk                 (0x1UL << MPI_CR_CTRM_Pos)
#define MPI_CR_CTRM                     MPI_CR_CTRM_Msk
#define MPI_CR_TCIE_Pos                 (8U)
#define MPI_CR_TCIE_Msk                 (0x1UL << MPI_CR_TCIE_Pos)
#define MPI_CR_TCIE                     MPI_CR_TCIE_Msk
#define MPI_CR_TOIE_Pos                 (9U)
#define MPI_CR_TOIE_Msk                 (0x1UL << MPI_CR_TOIE_Pos)
#define MPI_CR_TOIE                     MPI_CR_TOIE_Msk
#define MPI_CR_ROIE_Pos                 (10U)
#define MPI_CR_ROIE_Msk                 (0x1UL << MPI_CR_ROIE_Pos)
#define MPI_CR_ROIE                     MPI_CR_ROIE_Msk
#define MPI_CR_SMIE_Pos                 (11U)
#define MPI_CR_SMIE_Msk                 (0x1UL << MPI_CR_SMIE_Pos)
#define MPI_CR_SMIE                     MPI_CR_SMIE_Msk
#define MPI_CR_CSVIE_Pos                (12U)
#define MPI_CR_CSVIE_Msk                (0x1UL << MPI_CR_CSVIE_Pos)
#define MPI_CR_CSVIE                    MPI_CR_CSVIE_Msk
#define MPI_CR_RBXIE_Pos                (13U)
#define MPI_CR_RBXIE_Msk                (0x1UL << MPI_CR_RBXIE_Pos)
#define MPI_CR_RBXIE                    MPI_CR_RBXIE_Msk
#define MPI_CR_CMD2E_Pos                (16U)
#define MPI_CR_CMD2E_Msk                (0x1UL << MPI_CR_CMD2E_Pos)
#define MPI_CR_CMD2E                    MPI_CR_CMD2E_Msk
#define MPI_CR_SME1_Pos                 (17U)
#define MPI_CR_SME1_Msk                 (0x1UL << MPI_CR_SME1_Pos)
#define MPI_CR_SME1                     MPI_CR_SME1_Msk
#define MPI_CR_SME2_Pos                 (18U)
#define MPI_CR_SME2_Msk                 (0x1UL << MPI_CR_SME2_Pos)
#define MPI_CR_SME2                     MPI_CR_SME2_Msk
#define MPI_CR_SMM_Pos                  (19U)
#define MPI_CR_SMM_Msk                  (0x1UL << MPI_CR_SMM_Pos)
#define MPI_CR_SMM                      MPI_CR_SMM_Msk
#define MPI_CR_OPIE_Pos                 (21U)
#define MPI_CR_OPIE_Msk                 (0x1UL << MPI_CR_OPIE_Pos)
#define MPI_CR_OPIE                     MPI_CR_OPIE_Msk
#define MPI_CR_MX16_Pos                 (23U)
#define MPI_CR_MX16_Msk                 (0x1UL << MPI_CR_MX16_Pos)
#define MPI_CR_MX16                     MPI_CR_MX16_Msk
#define MPI_CR_DFM_Pos                  (24U)
#define MPI_CR_DFM_Msk                  (0x1UL << MPI_CR_DFM_Pos)
#define MPI_CR_DFM                      MPI_CR_DFM_Msk
#define MPI_CR_AHBDIS_Pos               (25U)
#define MPI_CR_AHBDIS_Msk               (0x1UL << MPI_CR_AHBDIS_Pos)
#define MPI_CR_AHBDIS                   MPI_CR_AHBDIS_Msk
#define MPI_CR_ABORT_Pos                (31U)
#define MPI_CR_ABORT_Msk                (0x1UL << MPI_CR_ABORT_Pos)
#define MPI_CR_ABORT                    MPI_CR_ABORT_Msk

/********************* Bit definition for MPI_DR register *********************/
#define MPI_DR_DATA_Pos                 (0U)
#define MPI_DR_DATA_Msk                 (0xFFFFFFFFUL << MPI_DR_DATA_Pos)
#define MPI_DR_DATA                     MPI_DR_DATA_Msk

/******************** Bit definition for MPI_DCR register *********************/
#define MPI_DCR_RBSIZE_Pos              (0U)
#define MPI_DCR_RBSIZE_Msk              (0x7UL << MPI_DCR_RBSIZE_Pos)
#define MPI_DCR_RBSIZE                  MPI_DCR_RBSIZE_Msk
#define MPI_DCR_PROT_Pos                (4U)
#define MPI_DCR_PROT_Msk                (0x3UL << MPI_DCR_PROT_Pos)
#define MPI_DCR_PROT                    MPI_DCR_PROT_Msk
#define MPI_DCR_PROT_XCCELA             (0 << MPI_DCR_PROT_Pos)
#define MPI_DCR_PROT_XCCELA_LEGACY      (1 << MPI_DCR_PROT_Pos)
#define MPI_DCR_PROT_HYPER_BUS          (2 << MPI_DCR_PROT_Pos)
#define MPI_DCR_PROT_UPSRAM             (3 << MPI_DCR_PROT_Pos)
#define MPI_DCR_CSLMAX_Pos              (6U)
#define MPI_DCR_CSLMAX_Msk              (0xFFFUL << MPI_DCR_CSLMAX_Pos)
#define MPI_DCR_CSLMAX                  MPI_DCR_CSLMAX_Msk
#define MPI_DCR_CSLMIN_Pos              (18U)
#define MPI_DCR_CSLMIN_Msk              (0xFUL << MPI_DCR_CSLMIN_Pos)
#define MPI_DCR_CSLMIN                  MPI_DCR_CSLMIN_Msk
#define MPI_DCR_CSHMIN_Pos              (22U)
#define MPI_DCR_CSHMIN_Msk              (0xFUL << MPI_DCR_CSHMIN_Pos)
#define MPI_DCR_CSHMIN                  MPI_DCR_CSHMIN_Msk
#define MPI_DCR_TRCMIN_Pos              (26U)
#define MPI_DCR_TRCMIN_Msk              (0x1FUL << MPI_DCR_TRCMIN_Pos)
#define MPI_DCR_TRCMIN                  MPI_DCR_TRCMIN_Msk

/******************* Bit definition for MPI_PSCLR register ********************/
#define MPI_PSCLR_DIV_Pos               (0U)
#define MPI_PSCLR_DIV_Msk               (0xFFUL << MPI_PSCLR_DIV_Pos)
#define MPI_PSCLR_DIV                   MPI_PSCLR_DIV_Msk

/********************* Bit definition for MPI_SR register *********************/
#define MPI_SR_TCF_Pos                  (0U)
#define MPI_SR_TCF_Msk                  (0x1UL << MPI_SR_TCF_Pos)
#define MPI_SR_TCF                      MPI_SR_TCF_Msk
#define MPI_SR_TOF_Pos                  (1U)
#define MPI_SR_TOF_Msk                  (0x1UL << MPI_SR_TOF_Pos)
#define MPI_SR_TOF                      MPI_SR_TOF_Msk
#define MPI_SR_ROF_Pos                  (2U)
#define MPI_SR_ROF_Msk                  (0x1UL << MPI_SR_ROF_Pos)
#define MPI_SR_ROF                      MPI_SR_ROF_Msk
#define MPI_SR_SMF_Pos                  (3U)
#define MPI_SR_SMF_Msk                  (0x1UL << MPI_SR_SMF_Pos)
#define MPI_SR_SMF                      MPI_SR_SMF_Msk
#define MPI_SR_CSVF_Pos                 (4U)
#define MPI_SR_CSVF_Msk                 (0x1UL << MPI_SR_CSVF_Pos)
#define MPI_SR_CSVF                     MPI_SR_CSVF_Msk
#define MPI_SR_RBXF_Pos                 (5U)
#define MPI_SR_RBXF_Msk                 (0x1UL << MPI_SR_RBXF_Pos)
#define MPI_SR_RBXF                     MPI_SR_RBXF_Msk
#define MPI_SR_IDLEF_Pos                (6U)
#define MPI_SR_IDLEF_Msk                (0x1UL << MPI_SR_IDLEF_Pos)
#define MPI_SR_IDLEF                    MPI_SR_IDLEF_Msk
#define MPI_SR_AHBBUSY_Pos              (29U)
#define MPI_SR_AHBBUSY_Msk              (0x1UL << MPI_SR_AHBBUSY_Pos)
#define MPI_SR_AHBBUSY                  MPI_SR_AHBBUSY_Msk
#define MPI_SR_CMDBUSY_Pos              (30U)
#define MPI_SR_CMDBUSY_Msk              (0x1UL << MPI_SR_CMDBUSY_Pos)
#define MPI_SR_CMDBUSY                  MPI_SR_CMDBUSY_Msk
#define MPI_SR_BUSY_Pos                 (31U)
#define MPI_SR_BUSY_Msk                 (0x1UL << MPI_SR_BUSY_Pos)
#define MPI_SR_BUSY                     MPI_SR_BUSY_Msk

/******************** Bit definition for MPI_SCR register *********************/
#define MPI_SCR_TCFC_Pos                (0U)
#define MPI_SCR_TCFC_Msk                (0x1UL << MPI_SCR_TCFC_Pos)
#define MPI_SCR_TCFC                    MPI_SCR_TCFC_Msk
#define MPI_SCR_TOFC_Pos                (1U)
#define MPI_SCR_TOFC_Msk                (0x1UL << MPI_SCR_TOFC_Pos)
#define MPI_SCR_TOFC                    MPI_SCR_TOFC_Msk
#define MPI_SCR_ROFC_Pos                (2U)
#define MPI_SCR_ROFC_Msk                (0x1UL << MPI_SCR_ROFC_Pos)
#define MPI_SCR_ROFC                    MPI_SCR_ROFC_Msk
#define MPI_SCR_SMFC_Pos                (3U)
#define MPI_SCR_SMFC_Msk                (0x1UL << MPI_SCR_SMFC_Pos)
#define MPI_SCR_SMFC                    MPI_SCR_SMFC_Msk
#define MPI_SCR_CSVFC_Pos               (4U)
#define MPI_SCR_CSVFC_Msk               (0x1UL << MPI_SCR_CSVFC_Pos)
#define MPI_SCR_CSVFC                   MPI_SCR_CSVFC_Msk
#define MPI_SCR_RBXFC_Pos               (5U)
#define MPI_SCR_RBXFC_Msk               (0x1UL << MPI_SCR_RBXFC_Pos)
#define MPI_SCR_RBXFC                   MPI_SCR_RBXFC_Msk

/******************* Bit definition for MPI_CMDR1 register ********************/
#define MPI_CMDR1_CMD_Pos               (0U)
#define MPI_CMDR1_CMD_Msk               (0xFFUL << MPI_CMDR1_CMD_Pos)
#define MPI_CMDR1_CMD                   MPI_CMDR1_CMD_Msk

/******************** Bit definition for MPI_AR1 register *********************/
#define MPI_AR1_ADDR_Pos                (0U)
#define MPI_AR1_ADDR_Msk                (0xFFFFFFFFUL << MPI_AR1_ADDR_Pos)
#define MPI_AR1_ADDR                    MPI_AR1_ADDR_Msk

/******************** Bit definition for MPI_ABR1 register ********************/
#define MPI_ABR1_ABYTE_Pos              (0U)
#define MPI_ABR1_ABYTE_Msk              (0xFFFFFFFFUL << MPI_ABR1_ABYTE_Pos)
#define MPI_ABR1_ABYTE                  MPI_ABR1_ABYTE_Msk

/******************** Bit definition for MPI_DLR1 register ********************/
#define MPI_DLR1_DLEN_Pos               (0U)
#define MPI_DLR1_DLEN_Msk               (0xFFFFFUL << MPI_DLR1_DLEN_Pos)
#define MPI_DLR1_DLEN                   MPI_DLR1_DLEN_Msk

/******************** Bit definition for MPI_CCR1 register ********************/
#define MPI_CCR1_IMODE_Pos              (0U)
#define MPI_CCR1_IMODE_Msk              (0x7UL << MPI_CCR1_IMODE_Pos)
#define MPI_CCR1_IMODE                  MPI_CCR1_IMODE_Msk
#define MPI_CCR1_ADMODE_Pos             (3U)
#define MPI_CCR1_ADMODE_Msk             (0x7UL << MPI_CCR1_ADMODE_Pos)
#define MPI_CCR1_ADMODE                 MPI_CCR1_ADMODE_Msk
#define MPI_CCR1_ADSIZE_Pos             (6U)
#define MPI_CCR1_ADSIZE_Msk             (0x3UL << MPI_CCR1_ADSIZE_Pos)
#define MPI_CCR1_ADSIZE                 MPI_CCR1_ADSIZE_Msk
#define MPI_CCR1_ABMODE_Pos             (8U)
#define MPI_CCR1_ABMODE_Msk             (0x7UL << MPI_CCR1_ABMODE_Pos)
#define MPI_CCR1_ABMODE                 MPI_CCR1_ABMODE_Msk
#define MPI_CCR1_ABSIZE_Pos             (11U)
#define MPI_CCR1_ABSIZE_Msk             (0x3UL << MPI_CCR1_ABSIZE_Pos)
#define MPI_CCR1_ABSIZE                 MPI_CCR1_ABSIZE_Msk
#define MPI_CCR1_DCYC_Pos               (13U)
#define MPI_CCR1_DCYC_Msk               (0x1FUL << MPI_CCR1_DCYC_Pos)
#define MPI_CCR1_DCYC                   MPI_CCR1_DCYC_Msk
#define MPI_CCR1_DMODE_Pos              (18U)
#define MPI_CCR1_DMODE_Msk              (0x7UL << MPI_CCR1_DMODE_Pos)
#define MPI_CCR1_DMODE                  MPI_CCR1_DMODE_Msk
#define MPI_CCR1_FMODE_Pos              (21U)
#define MPI_CCR1_FMODE_Msk              (0x1UL << MPI_CCR1_FMODE_Pos)
#define MPI_CCR1_FMODE                  MPI_CCR1_FMODE_Msk

/******************* Bit definition for MPI_CMDR2 register ********************/
#define MPI_CMDR2_CMD_Pos               (0U)
#define MPI_CMDR2_CMD_Msk               (0xFFUL << MPI_CMDR2_CMD_Pos)
#define MPI_CMDR2_CMD                   MPI_CMDR2_CMD_Msk

/******************** Bit definition for MPI_AR2 register *********************/
#define MPI_AR2_ADDR_Pos                (0U)
#define MPI_AR2_ADDR_Msk                (0xFFFFFFFFUL << MPI_AR2_ADDR_Pos)
#define MPI_AR2_ADDR                    MPI_AR2_ADDR_Msk

/******************** Bit definition for MPI_ABR2 register ********************/
#define MPI_ABR2_ABYTE_Pos              (0U)
#define MPI_ABR2_ABYTE_Msk              (0xFFFFFFFFUL << MPI_ABR2_ABYTE_Pos)
#define MPI_ABR2_ABYTE                  MPI_ABR2_ABYTE_Msk

/******************** Bit definition for MPI_DLR2 register ********************/
#define MPI_DLR2_DLEN_Pos               (0U)
#define MPI_DLR2_DLEN_Msk               (0xFFFFFUL << MPI_DLR2_DLEN_Pos)
#define MPI_DLR2_DLEN                   MPI_DLR2_DLEN_Msk

/******************** Bit definition for MPI_CCR2 register ********************/
#define MPI_CCR2_IMODE_Pos              (0U)
#define MPI_CCR2_IMODE_Msk              (0x7UL << MPI_CCR2_IMODE_Pos)
#define MPI_CCR2_IMODE                  MPI_CCR2_IMODE_Msk
#define MPI_CCR2_ADMODE_Pos             (3U)
#define MPI_CCR2_ADMODE_Msk             (0x7UL << MPI_CCR2_ADMODE_Pos)
#define MPI_CCR2_ADMODE                 MPI_CCR2_ADMODE_Msk
#define MPI_CCR2_ADSIZE_Pos             (6U)
#define MPI_CCR2_ADSIZE_Msk             (0x3UL << MPI_CCR2_ADSIZE_Pos)
#define MPI_CCR2_ADSIZE                 MPI_CCR2_ADSIZE_Msk
#define MPI_CCR2_ABMODE_Pos             (8U)
#define MPI_CCR2_ABMODE_Msk             (0x7UL << MPI_CCR2_ABMODE_Pos)
#define MPI_CCR2_ABMODE                 MPI_CCR2_ABMODE_Msk
#define MPI_CCR2_ABSIZE_Pos             (11U)
#define MPI_CCR2_ABSIZE_Msk             (0x3UL << MPI_CCR2_ABSIZE_Pos)
#define MPI_CCR2_ABSIZE                 MPI_CCR2_ABSIZE_Msk
#define MPI_CCR2_DCYC_Pos               (13U)
#define MPI_CCR2_DCYC_Msk               (0x1FUL << MPI_CCR2_DCYC_Pos)
#define MPI_CCR2_DCYC                   MPI_CCR2_DCYC_Msk
#define MPI_CCR2_DMODE_Pos              (18U)
#define MPI_CCR2_DMODE_Msk              (0x7UL << MPI_CCR2_DMODE_Pos)
#define MPI_CCR2_DMODE                  MPI_CCR2_DMODE_Msk
#define MPI_CCR2_FMODE_Pos              (21U)
#define MPI_CCR2_FMODE_Msk              (0x1UL << MPI_CCR2_FMODE_Pos)
#define MPI_CCR2_FMODE                  MPI_CCR2_FMODE_Msk

/******************* Bit definition for MPI_HCMDR register ********************/
#define MPI_HCMDR_RCMD_Pos              (0U)
#define MPI_HCMDR_RCMD_Msk              (0xFFUL << MPI_HCMDR_RCMD_Pos)
#define MPI_HCMDR_RCMD                  MPI_HCMDR_RCMD_Msk
#define MPI_HCMDR_WCMD_Pos              (8U)
#define MPI_HCMDR_WCMD_Msk              (0xFFUL << MPI_HCMDR_WCMD_Pos)
#define MPI_HCMDR_WCMD                  MPI_HCMDR_WCMD_Msk

/******************* Bit definition for MPI_HRABR register ********************/
#define MPI_HRABR_ABYTE_Pos             (0U)
#define MPI_HRABR_ABYTE_Msk             (0xFFFFFFFFUL << MPI_HRABR_ABYTE_Pos)
#define MPI_HRABR_ABYTE                 MPI_HRABR_ABYTE_Msk

/******************* Bit definition for MPI_HRCCR register ********************/
#define MPI_HRCCR_IMODE_Pos             (0U)
#define MPI_HRCCR_IMODE_Msk             (0x7UL << MPI_HRCCR_IMODE_Pos)
#define MPI_HRCCR_IMODE                 MPI_HRCCR_IMODE_Msk
#define MPI_HRCCR_ADMODE_Pos            (3U)
#define MPI_HRCCR_ADMODE_Msk            (0x7UL << MPI_HRCCR_ADMODE_Pos)
#define MPI_HRCCR_ADMODE                MPI_HRCCR_ADMODE_Msk
#define MPI_HRCCR_ADSIZE_Pos            (6U)
#define MPI_HRCCR_ADSIZE_Msk            (0x3UL << MPI_HRCCR_ADSIZE_Pos)
#define MPI_HRCCR_ADSIZE                MPI_HRCCR_ADSIZE_Msk
#define MPI_HRCCR_ABMODE_Pos            (8U)
#define MPI_HRCCR_ABMODE_Msk            (0x7UL << MPI_HRCCR_ABMODE_Pos)
#define MPI_HRCCR_ABMODE                MPI_HRCCR_ABMODE_Msk
#define MPI_HRCCR_ABSIZE_Pos            (11U)
#define MPI_HRCCR_ABSIZE_Msk            (0x3UL << MPI_HRCCR_ABSIZE_Pos)
#define MPI_HRCCR_ABSIZE                MPI_HRCCR_ABSIZE_Msk
#define MPI_HRCCR_DCYC_Pos              (13U)
#define MPI_HRCCR_DCYC_Msk              (0x1FUL << MPI_HRCCR_DCYC_Pos)
#define MPI_HRCCR_DCYC                  MPI_HRCCR_DCYC_Msk
#define MPI_HRCCR_DMODE_Pos             (18U)
#define MPI_HRCCR_DMODE_Msk             (0x7UL << MPI_HRCCR_DMODE_Pos)
#define MPI_HRCCR_DMODE                 MPI_HRCCR_DMODE_Msk

/******************* Bit definition for MPI_HWABR register ********************/
#define MPI_HWABR_ABYTE_Pos             (0U)
#define MPI_HWABR_ABYTE_Msk             (0xFFFFFFFFUL << MPI_HWABR_ABYTE_Pos)
#define MPI_HWABR_ABYTE                 MPI_HWABR_ABYTE_Msk

/******************* Bit definition for MPI_HWCCR register ********************/
#define MPI_HWCCR_IMODE_Pos             (0U)
#define MPI_HWCCR_IMODE_Msk             (0x7UL << MPI_HWCCR_IMODE_Pos)
#define MPI_HWCCR_IMODE                 MPI_HWCCR_IMODE_Msk
#define MPI_HWCCR_ADMODE_Pos            (3U)
#define MPI_HWCCR_ADMODE_Msk            (0x7UL << MPI_HWCCR_ADMODE_Pos)
#define MPI_HWCCR_ADMODE                MPI_HWCCR_ADMODE_Msk
#define MPI_HWCCR_ADSIZE_Pos            (6U)
#define MPI_HWCCR_ADSIZE_Msk            (0x3UL << MPI_HWCCR_ADSIZE_Pos)
#define MPI_HWCCR_ADSIZE                MPI_HWCCR_ADSIZE_Msk
#define MPI_HWCCR_ABMODE_Pos            (8U)
#define MPI_HWCCR_ABMODE_Msk            (0x7UL << MPI_HWCCR_ABMODE_Pos)
#define MPI_HWCCR_ABMODE                MPI_HWCCR_ABMODE_Msk
#define MPI_HWCCR_ABSIZE_Pos            (11U)
#define MPI_HWCCR_ABSIZE_Msk            (0x3UL << MPI_HWCCR_ABSIZE_Pos)
#define MPI_HWCCR_ABSIZE                MPI_HWCCR_ABSIZE_Msk
#define MPI_HWCCR_DCYC_Pos              (13U)
#define MPI_HWCCR_DCYC_Msk              (0x1FUL << MPI_HWCCR_DCYC_Pos)
#define MPI_HWCCR_DCYC                  MPI_HWCCR_DCYC_Msk
#define MPI_HWCCR_DMODE_Pos             (18U)
#define MPI_HWCCR_DMODE_Msk             (0x7UL << MPI_HWCCR_DMODE_Pos)
#define MPI_HWCCR_DMODE                 MPI_HWCCR_DMODE_Msk

/******************* Bit definition for MPI_FIFOCR register *******************/
#define MPI_FIFOCR_RXCLR_Pos            (0U)
#define MPI_FIFOCR_RXCLR_Msk            (0x1UL << MPI_FIFOCR_RXCLR_Pos)
#define MPI_FIFOCR_RXCLR                MPI_FIFOCR_RXCLR_Msk
#define MPI_FIFOCR_RXE_Pos              (1U)
#define MPI_FIFOCR_RXE_Msk              (0x1UL << MPI_FIFOCR_RXE_Pos)
#define MPI_FIFOCR_RXE                  MPI_FIFOCR_RXE_Msk
#define MPI_FIFOCR_TXCLR_Pos            (8U)
#define MPI_FIFOCR_TXCLR_Msk            (0x1UL << MPI_FIFOCR_TXCLR_Pos)
#define MPI_FIFOCR_TXCLR                MPI_FIFOCR_TXCLR_Msk
#define MPI_FIFOCR_TXF_Pos              (9U)
#define MPI_FIFOCR_TXF_Msk              (0x1UL << MPI_FIFOCR_TXF_Pos)
#define MPI_FIFOCR_TXF                  MPI_FIFOCR_TXF_Msk
#define MPI_FIFOCR_TXSLOTS_Pos          (10U)
#define MPI_FIFOCR_TXSLOTS_Msk          (0x1FUL << MPI_FIFOCR_TXSLOTS_Pos)
#define MPI_FIFOCR_TXSLOTS              MPI_FIFOCR_TXSLOTS_Msk

/******************* Bit definition for MPI_MISCR register ********************/
#define MPI_MISCR_DQSDLY_Pos            (0U)
#define MPI_MISCR_DQSDLY_Msk            (0xFFUL << MPI_MISCR_DQSDLY_Pos)
#define MPI_MISCR_DQSDLY                MPI_MISCR_DQSDLY_Msk
#define MPI_MISCR_SCKDLY_Pos            (8U)
#define MPI_MISCR_SCKDLY_Msk            (0xFFUL << MPI_MISCR_SCKDLY_Pos)
#define MPI_MISCR_SCKDLY                MPI_MISCR_SCKDLY_Msk
#define MPI_MISCR_SCKINV_Pos            (16U)
#define MPI_MISCR_SCKINV_Msk            (0x1UL << MPI_MISCR_SCKINV_Pos)
#define MPI_MISCR_SCKINV                MPI_MISCR_SCKINV_Msk
#define MPI_MISCR_LC2DLY_Pos            (17U)
#define MPI_MISCR_LC2DLY_Msk            (0x1UL << MPI_MISCR_LC2DLY_Pos)
#define MPI_MISCR_LC2DLY                MPI_MISCR_LC2DLY_Msk

/******************* Bit definition for MPI_CALCR register ********************/
#define MPI_CALCR_STOP_Pos              (30U)
#define MPI_CALCR_STOP_Msk              (0x1UL << MPI_CALCR_STOP_Pos)
#define MPI_CALCR_STOP                  MPI_CALCR_STOP_Msk
#define MPI_CALCR_START_Pos             (31U)
#define MPI_CALCR_START_Msk             (0x1UL << MPI_CALCR_START_Pos)
#define MPI_CALCR_START                 MPI_CALCR_START_Msk

/******************* Bit definition for MPI_CALRR register ********************/
#define MPI_CALRR_DSMP_Pos              (0U)
#define MPI_CALRR_DSMP_Msk              (0xFFFFUL << MPI_CALRR_DSMP_Pos)
#define MPI_CALRR_DSMP                  MPI_CALRR_DSMP_Msk
#define MPI_CALRR_CSMP_Pos              (16U)
#define MPI_CALRR_CSMP_Msk              (0x1UL << MPI_CALRR_CSMP_Pos)
#define MPI_CALRR_CSMP                  MPI_CALRR_CSMP_Msk
#define MPI_CALRR_RUN_Pos               (31U)
#define MPI_CALRR_RUN_Msk               (0x1UL << MPI_CALRR_RUN_Pos)
#define MPI_CALRR_RUN                   MPI_CALRR_RUN_Msk

/******************* Bit definition for MPI_CTRSAR register *******************/
#define MPI_CTRSAR_SA_Pos               (10U)
#define MPI_CTRSAR_SA_Msk               (0x3FFFFFUL << MPI_CTRSAR_SA_Pos)
#define MPI_CTRSAR_SA                   MPI_CTRSAR_SA_Msk

/******************* Bit definition for MPI_CTREAR register *******************/
#define MPI_CTREAR_EA_Pos               (10U)
#define MPI_CTREAR_EA_Msk               (0x3FFFFFUL << MPI_CTREAR_EA_Pos)
#define MPI_CTREAR_EA                   MPI_CTREAR_EA_Msk

/******************* Bit definition for MPI_NONCEA register *******************/
#define MPI_NONCEA_NONCEA_Pos           (0U)
#define MPI_NONCEA_NONCEA_Msk           (0xFFFFFFFFUL << MPI_NONCEA_NONCEA_Pos)
#define MPI_NONCEA_NONCEA               MPI_NONCEA_NONCEA_Msk

/******************* Bit definition for MPI_NONCEB register *******************/
#define MPI_NONCEB_NONCEB_Pos           (0U)
#define MPI_NONCEB_NONCEB_Msk           (0xFFFFFFFFUL << MPI_NONCEB_NONCEB_Pos)
#define MPI_NONCEB_NONCEB               MPI_NONCEB_NONCEB_Msk

/******************* Bit definition for MPI_AASAR register ********************/
#define MPI_AASAR_SA_Pos                (10U)
#define MPI_AASAR_SA_Msk                (0x3FFFFFUL << MPI_AASAR_SA_Pos)
#define MPI_AASAR_SA                    MPI_AASAR_SA_Msk

/******************* Bit definition for MPI_AAEAR register ********************/
#define MPI_AAEAR_EA_Pos                (10U)
#define MPI_AAEAR_EA_Msk                (0x3FFFFFUL << MPI_AAEAR_EA_Pos)
#define MPI_AAEAR_EA                    MPI_AAEAR_EA_Msk

/******************* Bit definition for MPI_AAOAR register ********************/
#define MPI_AAOAR_OA_Pos                (10U)
#define MPI_AAOAR_OA_Msk                (0x3FFFFFUL << MPI_AAOAR_OA_Pos)
#define MPI_AAOAR_OA                    MPI_AAOAR_OA_Msk

/******************** Bit definition for MPI_CIR register *********************/
#define MPI_CIR_INTERVAL1_Pos           (0U)
#define MPI_CIR_INTERVAL1_Msk           (0xFFFFUL << MPI_CIR_INTERVAL1_Pos)
#define MPI_CIR_INTERVAL1               MPI_CIR_INTERVAL1_Msk
#define MPI_CIR_INTERVAL2_Pos           (16U)
#define MPI_CIR_INTERVAL2_Msk           (0xFFFFUL << MPI_CIR_INTERVAL2_Pos)
#define MPI_CIR_INTERVAL2               MPI_CIR_INTERVAL2_Msk

/******************** Bit definition for MPI_SMR register *********************/
#define MPI_SMR_STATUS_Pos              (0U)
#define MPI_SMR_STATUS_Msk              (0xFFFFFFFFUL << MPI_SMR_STATUS_Pos)
#define MPI_SMR_STATUS                  MPI_SMR_STATUS_Msk

/******************** Bit definition for MPI_SMKR register ********************/
#define MPI_SMKR_MASK_Pos               (0U)
#define MPI_SMKR_MASK_Msk               (0xFFFFFFFFUL << MPI_SMKR_MASK_Pos)
#define MPI_SMKR_MASK                   MPI_SMKR_MASK_Msk

/******************** Bit definition for MPI_TIMR register ********************/
#define MPI_TIMR_RTIM_Pos               (0U)
#define MPI_TIMR_RTIM_Msk               (0xFFFFUL << MPI_TIMR_RTIM_Pos)
#define MPI_TIMR_RTIM                   MPI_TIMR_RTIM_Msk
#define MPI_TIMR_WTIM_Pos               (16U)
#define MPI_TIMR_WTIM_Msk               (0xFFFFUL << MPI_TIMR_WTIM_Pos)
#define MPI_TIMR_WTIM                   MPI_TIMR_WTIM_Msk

/******************** Bit definition for MPI_WDTR register ********************/
#define MPI_WDTR_TIMEOUT_Pos            (0U)
#define MPI_WDTR_TIMEOUT_Msk            (0xFFFFUL << MPI_WDTR_TIMEOUT_Pos)
#define MPI_WDTR_TIMEOUT                MPI_WDTR_TIMEOUT_Msk
#define MPI_WDTR_EN_Pos                 (16U)
#define MPI_WDTR_EN_Msk                 (0x1UL << MPI_WDTR_EN_Pos)
#define MPI_WDTR_EN                     MPI_WDTR_EN_Msk
#define MPI_WDTR_MID_Pos                (24U)
#define MPI_WDTR_MID_Msk                (0xFUL << MPI_WDTR_MID_Pos)
#define MPI_WDTR_MID                    MPI_WDTR_MID_Msk
#define MPI_WDTR_TOF_Pos                (31U)
#define MPI_WDTR_TOF_Msk                (0x1UL << MPI_WDTR_TOF_Pos)
#define MPI_WDTR_TOF                    MPI_WDTR_TOF_Msk

/******************** Bit definition for MPI_CR2 register *********************/
#define MPI_CR2_LOOP_Pos                (0U)
#define MPI_CR2_LOOP_Msk                (0xFFUL << MPI_CR2_LOOP_Pos)
#define MPI_CR2_LOOP                    MPI_CR2_LOOP_Msk
#define MPI_CR2_CS1SIZE_Pos             (8U)
#define MPI_CR2_CS1SIZE_Msk             (0x7UL << MPI_CR2_CS1SIZE_Pos)
#define MPI_CR2_CS1SIZE                 MPI_CR2_CS1SIZE_Msk
#define MPI_CR2_CS2E_Pos                (11U)
#define MPI_CR2_CS2E_Msk                (0x1UL << MPI_CR2_CS2E_Pos)
#define MPI_CR2_CS2E                    MPI_CR2_CS2E_Msk

/******************** Bit definition for MPI_DCR2 register ********************/
#define MPI_DCR2_TCPHR_Pos              (0U)
#define MPI_DCR2_TCPHR_Msk              (0xFUL << MPI_DCR2_TCPHR_Pos)
#define MPI_DCR2_TCPHR                  MPI_DCR2_TCPHR_Msk
#define MPI_DCR2_TCPHW_Pos              (4U)
#define MPI_DCR2_TCPHW_Msk              (0xFUL << MPI_DCR2_TCPHW_Pos)
#define MPI_DCR2_TCPHW                  MPI_DCR2_TCPHW_Msk
#define MPI_DCR2_BTYPE_Pos              (8U)
#define MPI_DCR2_BTYPE_Msk              (0x1UL << MPI_DCR2_BTYPE_Pos)
#define MPI_DCR2_BTYPE                  MPI_DCR2_BTYPE_Msk
#define MPI_DCR2_CFMT_Pos               (9U)
#define MPI_DCR2_CFMT_Msk               (0x1UL << MPI_DCR2_CFMT_Pos)
#define MPI_DCR2_CFMT                   MPI_DCR2_CFMT_Msk

/****************** Bit definition for MPI_CTRSAR2 register *******************/
#define MPI_CTRSAR2_SA_Pos              (10U)
#define MPI_CTRSAR2_SA_Msk              (0x3FFFFFUL << MPI_CTRSAR2_SA_Pos)
#define MPI_CTRSAR2_SA                  MPI_CTRSAR2_SA_Msk

/****************** Bit definition for MPI_CTREAR2 register *******************/
#define MPI_CTREAR2_EA_Pos              (10U)
#define MPI_CTREAR2_EA_Msk              (0x3FFFFFUL << MPI_CTREAR2_EA_Pos)
#define MPI_CTREAR2_EA                  MPI_CTREAR2_EA_Msk

/****************** Bit definition for MPI_NONCEA2 register *******************/
#define MPI_NONCEA2_NONCEA_Pos          (0U)
#define MPI_NONCEA2_NONCEA_Msk          (0xFFFFFFFFUL << MPI_NONCEA2_NONCEA_Pos)
#define MPI_NONCEA2_NONCEA              MPI_NONCEA2_NONCEA_Msk

/****************** Bit definition for MPI_NONCEB2 register *******************/
#define MPI_NONCEB2_NONCEB_Pos          (0U)
#define MPI_NONCEB2_NONCEB_Msk          (0xFFFFFFFFUL << MPI_NONCEB2_NONCEB_Pos)
#define MPI_NONCEB2_NONCEB              MPI_NONCEB2_NONCEB_Msk

/******************** Bit definition for MPI_DBGR register ********************/
#define MPI_DBGR_AHB0_STATE_Pos         (0U)
#define MPI_DBGR_AHB0_STATE_Msk         (0xFUL << MPI_DBGR_AHB0_STATE_Pos)
#define MPI_DBGR_AHB0_STATE             MPI_DBGR_AHB0_STATE_Msk
#define MPI_DBGR_AHB0_MASTER_Pos        (4U)
#define MPI_DBGR_AHB0_MASTER_Msk        (0xFUL << MPI_DBGR_AHB0_MASTER_Pos)
#define MPI_DBGR_AHB0_MASTER            MPI_DBGR_AHB0_MASTER_Msk
#define MPI_DBGR_AHB1_STATE_Pos         (8U)
#define MPI_DBGR_AHB1_STATE_Msk         (0xFUL << MPI_DBGR_AHB1_STATE_Pos)
#define MPI_DBGR_AHB1_STATE             MPI_DBGR_AHB1_STATE_Msk
#define MPI_DBGR_AHB1_MASTER_Pos        (12U)
#define MPI_DBGR_AHB1_MASTER_Msk        (0xFUL << MPI_DBGR_AHB1_MASTER_Pos)
#define MPI_DBGR_AHB1_MASTER            MPI_DBGR_AHB1_MASTER_Msk
#define MPI_DBGR_PHY_STATE_Pos          (16U)
#define MPI_DBGR_PHY_STATE_Msk          (0x1FUL << MPI_DBGR_PHY_STATE_Pos)
#define MPI_DBGR_PHY_STATE              MPI_DBGR_PHY_STATE_Msk
#define MPI_DBGR_DBGSEL_Pos             (28U)
#define MPI_DBGR_DBGSEL_Msk             (0xFUL << MPI_DBGR_DBGSEL_Pos)
#define MPI_DBGR_DBGSEL                 MPI_DBGR_DBGSEL_Msk

#endif
