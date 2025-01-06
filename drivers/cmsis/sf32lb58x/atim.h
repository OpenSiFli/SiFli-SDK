#ifndef __ATIM_H
#define __ATIM_H

typedef struct
{
    __IO uint32_t CR1;
    __IO uint32_t CR2;
    __IO uint32_t SMCR;
    __IO uint32_t DIER;
    __IO uint32_t SR;
    __IO uint32_t EGR;
    __IO uint32_t CCMR1;
    __IO uint32_t CCMR2;
    __IO uint32_t CCER;
    __IO uint32_t CNT;
    __IO uint32_t PSC;
    __IO uint32_t ARR;
    __IO uint32_t RCR;
    __IO uint32_t CCR1;
    __IO uint32_t CCR2;
    __IO uint32_t CCR3;
    __IO uint32_t CCR4;
    __IO uint32_t BDTR;
    __IO uint32_t RSVD2[3];
    __IO uint32_t CCMR3;
    __IO uint32_t CCR5;
    __IO uint32_t CCR6;
    __IO uint32_t AF1;
    __IO uint32_t AF2;
    __IO uint32_t RSVD1[2];
    __IO uint32_t DBGSEL;
} ATIM_TypeDef;


/******************** Bit definition for ATIM_CR1 register ********************/
#define ATIM_CR1_CEN_Pos                (0U)
#define ATIM_CR1_CEN_Msk                (0x1UL << ATIM_CR1_CEN_Pos)
#define ATIM_CR1_CEN                    ATIM_CR1_CEN_Msk
#define ATIM_CR1_UDIS_Pos               (1U)
#define ATIM_CR1_UDIS_Msk               (0x1UL << ATIM_CR1_UDIS_Pos)
#define ATIM_CR1_UDIS                   ATIM_CR1_UDIS_Msk
#define ATIM_CR1_URS_Pos                (2U)
#define ATIM_CR1_URS_Msk                (0x1UL << ATIM_CR1_URS_Pos)
#define ATIM_CR1_URS                    ATIM_CR1_URS_Msk
#define ATIM_CR1_OPM_Pos                (3U)
#define ATIM_CR1_OPM_Msk                (0x1UL << ATIM_CR1_OPM_Pos)
#define ATIM_CR1_OPM                    ATIM_CR1_OPM_Msk
#define ATIM_CR1_DIR_Pos                (4U)
#define ATIM_CR1_DIR_Msk                (0x1UL << ATIM_CR1_DIR_Pos)
#define ATIM_CR1_DIR                    ATIM_CR1_DIR_Msk
#define ATIM_CR1_CMS_Pos                (5U)
#define ATIM_CR1_CMS_Msk                (0x3UL << ATIM_CR1_CMS_Pos)
#define ATIM_CR1_CMS                    ATIM_CR1_CMS_Msk
#define ATIM_CR1_ARPE_Pos               (7U)
#define ATIM_CR1_ARPE_Msk               (0x1UL << ATIM_CR1_ARPE_Pos)
#define ATIM_CR1_ARPE                   ATIM_CR1_ARPE_Msk
#define ATIM_CR1_UIFREMAP_Pos           (11U)
#define ATIM_CR1_UIFREMAP_Msk           (0x1UL << ATIM_CR1_UIFREMAP_Pos)
#define ATIM_CR1_UIFREMAP               ATIM_CR1_UIFREMAP_Msk

/******************** Bit definition for ATIM_CR2 register ********************/
#define ATIM_CR2_CCPC_Pos               (0U)
#define ATIM_CR2_CCPC_Msk               (0x1UL << ATIM_CR2_CCPC_Pos)
#define ATIM_CR2_CCPC                   ATIM_CR2_CCPC_Msk
#define ATIM_CR2_CCUS_Pos               (2U)
#define ATIM_CR2_CCUS_Msk               (0x1UL << ATIM_CR2_CCUS_Pos)
#define ATIM_CR2_CCUS                   ATIM_CR2_CCUS_Msk
#define ATIM_CR2_CCDS_Pos               (3U)
#define ATIM_CR2_CCDS_Msk               (0x1UL << ATIM_CR2_CCDS_Pos)
#define ATIM_CR2_CCDS                   ATIM_CR2_CCDS_Msk
#define ATIM_CR2_MMS_Pos                (4U)
#define ATIM_CR2_MMS_Msk                (0x7UL << ATIM_CR2_MMS_Pos)
#define ATIM_CR2_MMS                    ATIM_CR2_MMS_Msk
#define ATIM_CR2_TI1S_Pos               (7U)
#define ATIM_CR2_TI1S_Msk               (0x1UL << ATIM_CR2_TI1S_Pos)
#define ATIM_CR2_TI1S                   ATIM_CR2_TI1S_Msk
#define ATIM_CR2_OIS1_Pos               (8U)
#define ATIM_CR2_OIS1_Msk               (0x1UL << ATIM_CR2_OIS1_Pos)
#define ATIM_CR2_OIS1                   ATIM_CR2_OIS1_Msk
#define ATIM_CR2_OIS1N_Pos              (9U)
#define ATIM_CR2_OIS1N_Msk              (0x1UL << ATIM_CR2_OIS1N_Pos)
#define ATIM_CR2_OIS1N                  ATIM_CR2_OIS1N_Msk
#define ATIM_CR2_OIS2_Pos               (10U)
#define ATIM_CR2_OIS2_Msk               (0x1UL << ATIM_CR2_OIS2_Pos)
#define ATIM_CR2_OIS2                   ATIM_CR2_OIS2_Msk
#define ATIM_CR2_OIS2N_Pos              (11U)
#define ATIM_CR2_OIS2N_Msk              (0x1UL << ATIM_CR2_OIS2N_Pos)
#define ATIM_CR2_OIS2N                  ATIM_CR2_OIS2N_Msk
#define ATIM_CR2_OIS3_Pos               (12U)
#define ATIM_CR2_OIS3_Msk               (0x1UL << ATIM_CR2_OIS3_Pos)
#define ATIM_CR2_OIS3                   ATIM_CR2_OIS3_Msk
#define ATIM_CR2_OIS3N_Pos              (13U)
#define ATIM_CR2_OIS3N_Msk              (0x1UL << ATIM_CR2_OIS3N_Pos)
#define ATIM_CR2_OIS3N                  ATIM_CR2_OIS3N_Msk
#define ATIM_CR2_OIS4_Pos               (14U)
#define ATIM_CR2_OIS4_Msk               (0x1UL << ATIM_CR2_OIS4_Pos)
#define ATIM_CR2_OIS4                   ATIM_CR2_OIS4_Msk
#define ATIM_CR2_OIS5_Pos               (16U)
#define ATIM_CR2_OIS5_Msk               (0x1UL << ATIM_CR2_OIS5_Pos)
#define ATIM_CR2_OIS5                   ATIM_CR2_OIS5_Msk
#define ATIM_CR2_OIS6_Pos               (18U)
#define ATIM_CR2_OIS6_Msk               (0x1UL << ATIM_CR2_OIS6_Pos)
#define ATIM_CR2_OIS6                   ATIM_CR2_OIS6_Msk
#define ATIM_CR2_MMS2_Pos               (20U)
#define ATIM_CR2_MMS2_Msk               (0xFUL << ATIM_CR2_MMS2_Pos)
#define ATIM_CR2_MMS2                   ATIM_CR2_MMS2_Msk

/******************* Bit definition for ATIM_SMCR register ********************/
#define ATIM_SMCR_TS_Pos                (4U)
#define ATIM_SMCR_TS_Msk                (0x7UL << ATIM_SMCR_TS_Pos)
#define ATIM_SMCR_TS                    ATIM_SMCR_TS_Msk
#define ATIM_SMCR_MSM_Pos               (7U)
#define ATIM_SMCR_MSM_Msk               (0x1UL << ATIM_SMCR_MSM_Pos)
#define ATIM_SMCR_MSM                   ATIM_SMCR_MSM_Msk
#define ATIM_SMCR_ETF_Pos               (8U)
#define ATIM_SMCR_ETF_Msk               (0xFUL << ATIM_SMCR_ETF_Pos)
#define ATIM_SMCR_ETF                   ATIM_SMCR_ETF_Msk
#define ATIM_SMCR_ETPS_Pos              (12U)
#define ATIM_SMCR_ETPS_Msk              (0x3UL << ATIM_SMCR_ETPS_Pos)
#define ATIM_SMCR_ETPS                  ATIM_SMCR_ETPS_Msk
#define ATIM_SMCR_ECE_Pos               (14U)
#define ATIM_SMCR_ECE_Msk               (0x1UL << ATIM_SMCR_ECE_Pos)
#define ATIM_SMCR_ECE                   ATIM_SMCR_ECE_Msk
#define ATIM_SMCR_ETP_Pos               (15U)
#define ATIM_SMCR_ETP_Msk               (0x1UL << ATIM_SMCR_ETP_Pos)
#define ATIM_SMCR_ETP                   ATIM_SMCR_ETP_Msk
#define ATIM_SMCR_SMS_Pos               (16U)
#define ATIM_SMCR_SMS_Msk               (0xFUL << ATIM_SMCR_SMS_Pos)
#define ATIM_SMCR_SMS                   ATIM_SMCR_SMS_Msk

/******************* Bit definition for ATIM_DIER register ********************/
#define ATIM_DIER_UIE_Pos               (0U)
#define ATIM_DIER_UIE_Msk               (0x1UL << ATIM_DIER_UIE_Pos)
#define ATIM_DIER_UIE                   ATIM_DIER_UIE_Msk
#define ATIM_DIER_CC1IE_Pos             (1U)
#define ATIM_DIER_CC1IE_Msk             (0x1UL << ATIM_DIER_CC1IE_Pos)
#define ATIM_DIER_CC1IE                 ATIM_DIER_CC1IE_Msk
#define ATIM_DIER_CC2IE_Pos             (2U)
#define ATIM_DIER_CC2IE_Msk             (0x1UL << ATIM_DIER_CC2IE_Pos)
#define ATIM_DIER_CC2IE                 ATIM_DIER_CC2IE_Msk
#define ATIM_DIER_CC3IE_Pos             (3U)
#define ATIM_DIER_CC3IE_Msk             (0x1UL << ATIM_DIER_CC3IE_Pos)
#define ATIM_DIER_CC3IE                 ATIM_DIER_CC3IE_Msk
#define ATIM_DIER_CC4IE_Pos             (4U)
#define ATIM_DIER_CC4IE_Msk             (0x1UL << ATIM_DIER_CC4IE_Pos)
#define ATIM_DIER_CC4IE                 ATIM_DIER_CC4IE_Msk
#define ATIM_DIER_COMIE_Pos             (5U)
#define ATIM_DIER_COMIE_Msk             (0x1UL << ATIM_DIER_COMIE_Pos)
#define ATIM_DIER_COMIE                 ATIM_DIER_COMIE_Msk
#define ATIM_DIER_TIE_Pos               (6U)
#define ATIM_DIER_TIE_Msk               (0x1UL << ATIM_DIER_TIE_Pos)
#define ATIM_DIER_TIE                   ATIM_DIER_TIE_Msk
#define ATIM_DIER_BIE_Pos               (7U)
#define ATIM_DIER_BIE_Msk               (0x1UL << ATIM_DIER_BIE_Pos)
#define ATIM_DIER_BIE                   ATIM_DIER_BIE_Msk
#define ATIM_DIER_UDE_Pos               (8U)
#define ATIM_DIER_UDE_Msk               (0x1UL << ATIM_DIER_UDE_Pos)
#define ATIM_DIER_UDE                   ATIM_DIER_UDE_Msk
#define ATIM_DIER_CC1DE_Pos             (9U)
#define ATIM_DIER_CC1DE_Msk             (0x1UL << ATIM_DIER_CC1DE_Pos)
#define ATIM_DIER_CC1DE                 ATIM_DIER_CC1DE_Msk
#define ATIM_DIER_CC2DE_Pos             (10U)
#define ATIM_DIER_CC2DE_Msk             (0x1UL << ATIM_DIER_CC2DE_Pos)
#define ATIM_DIER_CC2DE                 ATIM_DIER_CC2DE_Msk
#define ATIM_DIER_CC3DE_Pos             (11U)
#define ATIM_DIER_CC3DE_Msk             (0x1UL << ATIM_DIER_CC3DE_Pos)
#define ATIM_DIER_CC3DE                 ATIM_DIER_CC3DE_Msk
#define ATIM_DIER_CC4DE_Pos             (12U)
#define ATIM_DIER_CC4DE_Msk             (0x1UL << ATIM_DIER_CC4DE_Pos)
#define ATIM_DIER_CC4DE                 ATIM_DIER_CC4DE_Msk
#define ATIM_DIER_COMDE_Pos             (13U)
#define ATIM_DIER_COMDE_Msk             (0x1UL << ATIM_DIER_COMDE_Pos)
#define ATIM_DIER_COMDE                 ATIM_DIER_COMDE_Msk
#define ATIM_DIER_TDE_Pos               (14U)
#define ATIM_DIER_TDE_Msk               (0x1UL << ATIM_DIER_TDE_Pos)
#define ATIM_DIER_TDE                   ATIM_DIER_TDE_Msk
#define ATIM_DIER_CC5IE_Pos             (16U)
#define ATIM_DIER_CC5IE_Msk             (0x1UL << ATIM_DIER_CC5IE_Pos)
#define ATIM_DIER_CC5IE                 ATIM_DIER_CC5IE_Msk
#define ATIM_DIER_CC6IE_Pos             (17U)
#define ATIM_DIER_CC6IE_Msk             (0x1UL << ATIM_DIER_CC6IE_Pos)
#define ATIM_DIER_CC6IE                 ATIM_DIER_CC6IE_Msk

/******************** Bit definition for ATIM_SR register *********************/
#define ATIM_SR_UIF_Pos                 (0U)
#define ATIM_SR_UIF_Msk                 (0x1UL << ATIM_SR_UIF_Pos)
#define ATIM_SR_UIF                     ATIM_SR_UIF_Msk
#define ATIM_SR_CC1IF_Pos               (1U)
#define ATIM_SR_CC1IF_Msk               (0x1UL << ATIM_SR_CC1IF_Pos)
#define ATIM_SR_CC1IF                   ATIM_SR_CC1IF_Msk
#define ATIM_SR_CC2IF_Pos               (2U)
#define ATIM_SR_CC2IF_Msk               (0x1UL << ATIM_SR_CC2IF_Pos)
#define ATIM_SR_CC2IF                   ATIM_SR_CC2IF_Msk
#define ATIM_SR_CC3IF_Pos               (3U)
#define ATIM_SR_CC3IF_Msk               (0x1UL << ATIM_SR_CC3IF_Pos)
#define ATIM_SR_CC3IF                   ATIM_SR_CC3IF_Msk
#define ATIM_SR_CC4IF_Pos               (4U)
#define ATIM_SR_CC4IF_Msk               (0x1UL << ATIM_SR_CC4IF_Pos)
#define ATIM_SR_CC4IF                   ATIM_SR_CC4IF_Msk
#define ATIM_SR_COMIF_Pos               (5U)
#define ATIM_SR_COMIF_Msk               (0x1UL << ATIM_SR_COMIF_Pos)
#define ATIM_SR_COMIF                   ATIM_SR_COMIF_Msk
#define ATIM_SR_TIF_Pos                 (6U)
#define ATIM_SR_TIF_Msk                 (0x1UL << ATIM_SR_TIF_Pos)
#define ATIM_SR_TIF                     ATIM_SR_TIF_Msk
#define ATIM_SR_BIF_Pos                 (7U)
#define ATIM_SR_BIF_Msk                 (0x1UL << ATIM_SR_BIF_Pos)
#define ATIM_SR_BIF                     ATIM_SR_BIF_Msk
#define ATIM_SR_B2IF_Pos                (8U)
#define ATIM_SR_B2IF_Msk                (0x1UL << ATIM_SR_B2IF_Pos)
#define ATIM_SR_B2IF                    ATIM_SR_B2IF_Msk
#define ATIM_SR_CC1OF_Pos               (9U)
#define ATIM_SR_CC1OF_Msk               (0x1UL << ATIM_SR_CC1OF_Pos)
#define ATIM_SR_CC1OF                   ATIM_SR_CC1OF_Msk
#define ATIM_SR_CC2OF_Pos               (10U)
#define ATIM_SR_CC2OF_Msk               (0x1UL << ATIM_SR_CC2OF_Pos)
#define ATIM_SR_CC2OF                   ATIM_SR_CC2OF_Msk
#define ATIM_SR_CC3OF_Pos               (11U)
#define ATIM_SR_CC3OF_Msk               (0x1UL << ATIM_SR_CC3OF_Pos)
#define ATIM_SR_CC3OF                   ATIM_SR_CC3OF_Msk
#define ATIM_SR_CC4OF_Pos               (12U)
#define ATIM_SR_CC4OF_Msk               (0x1UL << ATIM_SR_CC4OF_Pos)
#define ATIM_SR_CC4OF                   ATIM_SR_CC4OF_Msk
#define ATIM_SR_SBIF_Pos                (13U)
#define ATIM_SR_SBIF_Msk                (0x1UL << ATIM_SR_SBIF_Pos)
#define ATIM_SR_SBIF                    ATIM_SR_SBIF_Msk
#define ATIM_SR_CC5IF_Pos               (16U)
#define ATIM_SR_CC5IF_Msk               (0x1UL << ATIM_SR_CC5IF_Pos)
#define ATIM_SR_CC5IF                   ATIM_SR_CC5IF_Msk
#define ATIM_SR_CC6IF_Pos               (17U)
#define ATIM_SR_CC6IF_Msk               (0x1UL << ATIM_SR_CC6IF_Pos)
#define ATIM_SR_CC6IF                   ATIM_SR_CC6IF_Msk

/******************** Bit definition for ATIM_EGR register ********************/
#define ATIM_EGR_UG_Pos                 (0U)
#define ATIM_EGR_UG_Msk                 (0x1UL << ATIM_EGR_UG_Pos)
#define ATIM_EGR_UG                     ATIM_EGR_UG_Msk
#define ATIM_EGR_CC1G_Pos               (1U)
#define ATIM_EGR_CC1G_Msk               (0x1UL << ATIM_EGR_CC1G_Pos)
#define ATIM_EGR_CC1G                   ATIM_EGR_CC1G_Msk
#define ATIM_EGR_CC2G_Pos               (2U)
#define ATIM_EGR_CC2G_Msk               (0x1UL << ATIM_EGR_CC2G_Pos)
#define ATIM_EGR_CC2G                   ATIM_EGR_CC2G_Msk
#define ATIM_EGR_CC3G_Pos               (3U)
#define ATIM_EGR_CC3G_Msk               (0x1UL << ATIM_EGR_CC3G_Pos)
#define ATIM_EGR_CC3G                   ATIM_EGR_CC3G_Msk
#define ATIM_EGR_CC4G_Pos               (4U)
#define ATIM_EGR_CC4G_Msk               (0x1UL << ATIM_EGR_CC4G_Pos)
#define ATIM_EGR_CC4G                   ATIM_EGR_CC4G_Msk
#define ATIM_EGR_COMG_Pos               (5U)
#define ATIM_EGR_COMG_Msk               (0x1UL << ATIM_EGR_COMG_Pos)
#define ATIM_EGR_COMG                   ATIM_EGR_COMG_Msk
#define ATIM_EGR_TG_Pos                 (6U)
#define ATIM_EGR_TG_Msk                 (0x1UL << ATIM_EGR_TG_Pos)
#define ATIM_EGR_TG                     ATIM_EGR_TG_Msk
#define ATIM_EGR_BG_Pos                 (7U)
#define ATIM_EGR_BG_Msk                 (0x1UL << ATIM_EGR_BG_Pos)
#define ATIM_EGR_BG                     ATIM_EGR_BG_Msk
#define ATIM_EGR_B2G_Pos                (8U)
#define ATIM_EGR_B2G_Msk                (0x1UL << ATIM_EGR_B2G_Pos)
#define ATIM_EGR_B2G                    ATIM_EGR_B2G_Msk

/******************* Bit definition for ATIM_CCMR1 register *******************/
#define ATIM_CCMR1_CC1S_Pos             (0U)
#define ATIM_CCMR1_CC1S_Msk             (0x3UL << ATIM_CCMR1_CC1S_Pos)
#define ATIM_CCMR1_CC1S                 ATIM_CCMR1_CC1S_Msk
#define ATIM_CCMR1_IC1PSC_Pos           (2U)
#define ATIM_CCMR1_IC1PSC_Msk           (0x3UL << ATIM_CCMR1_IC1PSC_Pos)
#define ATIM_CCMR1_IC1PSC               ATIM_CCMR1_IC1PSC_Msk
#define ATIM_CCMR1_IC1F_Pos             (4U)
#define ATIM_CCMR1_IC1F_Msk             (0xFUL << ATIM_CCMR1_IC1F_Pos)
#define ATIM_CCMR1_IC1F                 ATIM_CCMR1_IC1F_Msk
#define ATIM_CCMR1_CC2S_Pos             (8U)
#define ATIM_CCMR1_CC2S_Msk             (0x3UL << ATIM_CCMR1_CC2S_Pos)
#define ATIM_CCMR1_CC2S                 ATIM_CCMR1_CC2S_Msk
#define ATIM_CCMR1_IC2PSC_Pos           (10U)
#define ATIM_CCMR1_IC2PSC_Msk           (0x3UL << ATIM_CCMR1_IC2PSC_Pos)
#define ATIM_CCMR1_IC2PSC               ATIM_CCMR1_IC2PSC_Msk
#define ATIM_CCMR1_IC2F_Pos             (12U)
#define ATIM_CCMR1_IC2F_Msk             (0xFUL << ATIM_CCMR1_IC2F_Pos)
#define ATIM_CCMR1_IC2F                 ATIM_CCMR1_IC2F_Msk
#define ATIM_CCMR1_OC1CE_Pos            (16U)
#define ATIM_CCMR1_OC1CE_Msk            (0x1UL << ATIM_CCMR1_OC1CE_Pos)
#define ATIM_CCMR1_OC1CE                ATIM_CCMR1_OC1CE_Msk
#define ATIM_CCMR1_OC1PE_Pos            (19U)
#define ATIM_CCMR1_OC1PE_Msk            (0x1UL << ATIM_CCMR1_OC1PE_Pos)
#define ATIM_CCMR1_OC1PE                ATIM_CCMR1_OC1PE_Msk
#define ATIM_CCMR1_OC1M_Pos             (20U)
#define ATIM_CCMR1_OC1M_Msk             (0xFUL << ATIM_CCMR1_OC1M_Pos)
#define ATIM_CCMR1_OC1M                 ATIM_CCMR1_OC1M_Msk
#define ATIM_CCMR1_OC2CE_Pos            (24U)
#define ATIM_CCMR1_OC2CE_Msk            (0x1UL << ATIM_CCMR1_OC2CE_Pos)
#define ATIM_CCMR1_OC2CE                ATIM_CCMR1_OC2CE_Msk
#define ATIM_CCMR1_OC2PE_Pos            (27U)
#define ATIM_CCMR1_OC2PE_Msk            (0x1UL << ATIM_CCMR1_OC2PE_Pos)
#define ATIM_CCMR1_OC2PE                ATIM_CCMR1_OC2PE_Msk
#define ATIM_CCMR1_OC2M_Pos             (28U)
#define ATIM_CCMR1_OC2M_Msk             (0xFUL << ATIM_CCMR1_OC2M_Pos)
#define ATIM_CCMR1_OC2M                 ATIM_CCMR1_OC2M_Msk

/******************* Bit definition for ATIM_CCMR2 register *******************/
#define ATIM_CCMR2_CC3S_Pos             (0U)
#define ATIM_CCMR2_CC3S_Msk             (0x3UL << ATIM_CCMR2_CC3S_Pos)
#define ATIM_CCMR2_CC3S                 ATIM_CCMR2_CC3S_Msk
#define ATIM_CCMR2_IC3PSC_Pos           (2U)
#define ATIM_CCMR2_IC3PSC_Msk           (0x3UL << ATIM_CCMR2_IC3PSC_Pos)
#define ATIM_CCMR2_IC3PSC               ATIM_CCMR2_IC3PSC_Msk
#define ATIM_CCMR2_IC3F_Pos             (4U)
#define ATIM_CCMR2_IC3F_Msk             (0xFUL << ATIM_CCMR2_IC3F_Pos)
#define ATIM_CCMR2_IC3F                 ATIM_CCMR2_IC3F_Msk
#define ATIM_CCMR2_CC4S_Pos             (8U)
#define ATIM_CCMR2_CC4S_Msk             (0x3UL << ATIM_CCMR2_CC4S_Pos)
#define ATIM_CCMR2_CC4S                 ATIM_CCMR2_CC4S_Msk
#define ATIM_CCMR2_IC4PSC_Pos           (10U)
#define ATIM_CCMR2_IC4PSC_Msk           (0x3UL << ATIM_CCMR2_IC4PSC_Pos)
#define ATIM_CCMR2_IC4PSC               ATIM_CCMR2_IC4PSC_Msk
#define ATIM_CCMR2_IC4F_Pos             (12U)
#define ATIM_CCMR2_IC4F_Msk             (0xFUL << ATIM_CCMR2_IC4F_Pos)
#define ATIM_CCMR2_IC4F                 ATIM_CCMR2_IC4F_Msk
#define ATIM_CCMR2_OC3CE_Pos            (16U)
#define ATIM_CCMR2_OC3CE_Msk            (0x1UL << ATIM_CCMR2_OC3CE_Pos)
#define ATIM_CCMR2_OC3CE                ATIM_CCMR2_OC3CE_Msk
#define ATIM_CCMR2_OC3PE_Pos            (19U)
#define ATIM_CCMR2_OC3PE_Msk            (0x1UL << ATIM_CCMR2_OC3PE_Pos)
#define ATIM_CCMR2_OC3PE                ATIM_CCMR2_OC3PE_Msk
#define ATIM_CCMR2_OC3M_Pos             (20U)
#define ATIM_CCMR2_OC3M_Msk             (0xFUL << ATIM_CCMR2_OC3M_Pos)
#define ATIM_CCMR2_OC3M                 ATIM_CCMR2_OC3M_Msk
#define ATIM_CCMR2_OC4CE_Pos            (24U)
#define ATIM_CCMR2_OC4CE_Msk            (0x1UL << ATIM_CCMR2_OC4CE_Pos)
#define ATIM_CCMR2_OC4CE                ATIM_CCMR2_OC4CE_Msk
#define ATIM_CCMR2_OC4PE_Pos            (27U)
#define ATIM_CCMR2_OC4PE_Msk            (0x1UL << ATIM_CCMR2_OC4PE_Pos)
#define ATIM_CCMR2_OC4PE                ATIM_CCMR2_OC4PE_Msk
#define ATIM_CCMR2_OC4M_Pos             (28U)
#define ATIM_CCMR2_OC4M_Msk             (0xFUL << ATIM_CCMR2_OC4M_Pos)
#define ATIM_CCMR2_OC4M                 ATIM_CCMR2_OC4M_Msk

/******************* Bit definition for ATIM_CCER register ********************/
#define ATIM_CCER_CC1E_Pos              (0U)
#define ATIM_CCER_CC1E_Msk              (0x1UL << ATIM_CCER_CC1E_Pos)
#define ATIM_CCER_CC1E                  ATIM_CCER_CC1E_Msk
#define ATIM_CCER_CC1P_Pos              (1U)
#define ATIM_CCER_CC1P_Msk              (0x1UL << ATIM_CCER_CC1P_Pos)
#define ATIM_CCER_CC1P                  ATIM_CCER_CC1P_Msk
#define ATIM_CCER_CC1NE_Pos             (2U)
#define ATIM_CCER_CC1NE_Msk             (0x1UL << ATIM_CCER_CC1NE_Pos)
#define ATIM_CCER_CC1NE                 ATIM_CCER_CC1NE_Msk
#define ATIM_CCER_CC1NP_Pos             (3U)
#define ATIM_CCER_CC1NP_Msk             (0x1UL << ATIM_CCER_CC1NP_Pos)
#define ATIM_CCER_CC1NP                 ATIM_CCER_CC1NP_Msk
#define ATIM_CCER_CC2E_Pos              (4U)
#define ATIM_CCER_CC2E_Msk              (0x1UL << ATIM_CCER_CC2E_Pos)
#define ATIM_CCER_CC2E                  ATIM_CCER_CC2E_Msk
#define ATIM_CCER_CC2P_Pos              (5U)
#define ATIM_CCER_CC2P_Msk              (0x1UL << ATIM_CCER_CC2P_Pos)
#define ATIM_CCER_CC2P                  ATIM_CCER_CC2P_Msk
#define ATIM_CCER_CC2NE_Pos             (6U)
#define ATIM_CCER_CC2NE_Msk             (0x1UL << ATIM_CCER_CC2NE_Pos)
#define ATIM_CCER_CC2NE                 ATIM_CCER_CC2NE_Msk
#define ATIM_CCER_CC2NP_Pos             (7U)
#define ATIM_CCER_CC2NP_Msk             (0x1UL << ATIM_CCER_CC2NP_Pos)
#define ATIM_CCER_CC2NP                 ATIM_CCER_CC2NP_Msk
#define ATIM_CCER_CC3E_Pos              (8U)
#define ATIM_CCER_CC3E_Msk              (0x1UL << ATIM_CCER_CC3E_Pos)
#define ATIM_CCER_CC3E                  ATIM_CCER_CC3E_Msk
#define ATIM_CCER_CC3P_Pos              (9U)
#define ATIM_CCER_CC3P_Msk              (0x1UL << ATIM_CCER_CC3P_Pos)
#define ATIM_CCER_CC3P                  ATIM_CCER_CC3P_Msk
#define ATIM_CCER_CC3NE_Pos             (10U)
#define ATIM_CCER_CC3NE_Msk             (0x1UL << ATIM_CCER_CC3NE_Pos)
#define ATIM_CCER_CC3NE                 ATIM_CCER_CC3NE_Msk
#define ATIM_CCER_CC3NP_Pos             (11U)
#define ATIM_CCER_CC3NP_Msk             (0x1UL << ATIM_CCER_CC3NP_Pos)
#define ATIM_CCER_CC3NP                 ATIM_CCER_CC3NP_Msk
#define ATIM_CCER_CC4E_Pos              (12U)
#define ATIM_CCER_CC4E_Msk              (0x1UL << ATIM_CCER_CC4E_Pos)
#define ATIM_CCER_CC4E                  ATIM_CCER_CC4E_Msk
#define ATIM_CCER_CC4P_Pos              (13U)
#define ATIM_CCER_CC4P_Msk              (0x1UL << ATIM_CCER_CC4P_Pos)
#define ATIM_CCER_CC4P                  ATIM_CCER_CC4P_Msk
#define ATIM_CCER_CC4NP_Pos             (15U)
#define ATIM_CCER_CC4NP_Msk             (0x1UL << ATIM_CCER_CC4NP_Pos)
#define ATIM_CCER_CC4NP                 ATIM_CCER_CC4NP_Msk
#define ATIM_CCER_CC5E_Pos              (16U)
#define ATIM_CCER_CC5E_Msk              (0x1UL << ATIM_CCER_CC5E_Pos)
#define ATIM_CCER_CC5E                  ATIM_CCER_CC5E_Msk
#define ATIM_CCER_CC5P_Pos              (17U)
#define ATIM_CCER_CC5P_Msk              (0x1UL << ATIM_CCER_CC5P_Pos)
#define ATIM_CCER_CC5P                  ATIM_CCER_CC5P_Msk
#define ATIM_CCER_CC6E_Pos              (20U)
#define ATIM_CCER_CC6E_Msk              (0x1UL << ATIM_CCER_CC6E_Pos)
#define ATIM_CCER_CC6E                  ATIM_CCER_CC6E_Msk
#define ATIM_CCER_CC6P_Pos              (21U)
#define ATIM_CCER_CC6P_Msk              (0x1UL << ATIM_CCER_CC6P_Pos)
#define ATIM_CCER_CC6P                  ATIM_CCER_CC6P_Msk

/******************** Bit definition for ATIM_CNT register ********************/
#define ATIM_CNT_CNT_Pos                (0U)
#define ATIM_CNT_CNT_Msk                (0xFFFFFFFFUL << ATIM_CNT_CNT_Pos)
#define ATIM_CNT_CNT                    ATIM_CNT_CNT_Msk

/******************** Bit definition for ATIM_PSC register ********************/
#define ATIM_PSC_PSC_Pos                (0U)
#define ATIM_PSC_PSC_Msk                (0xFFFFUL << ATIM_PSC_PSC_Pos)
#define ATIM_PSC_PSC                    ATIM_PSC_PSC_Msk

/******************** Bit definition for ATIM_ARR register ********************/
#define ATIM_ARR_ARR_Pos                (0U)
#define ATIM_ARR_ARR_Msk                (0xFFFFFFFFUL << ATIM_ARR_ARR_Pos)
#define ATIM_ARR_ARR                    ATIM_ARR_ARR_Msk

/******************** Bit definition for ATIM_RCR register ********************/
#define ATIM_RCR_REP_Pos                (0U)
#define ATIM_RCR_REP_Msk                (0xFFFFUL << ATIM_RCR_REP_Pos)
#define ATIM_RCR_REP                    ATIM_RCR_REP_Msk

/******************* Bit definition for ATIM_CCR1 register ********************/
#define ATIM_CCR1_CCR1_Pos              (0U)
#define ATIM_CCR1_CCR1_Msk              (0xFFFFFFFFUL << ATIM_CCR1_CCR1_Pos)
#define ATIM_CCR1_CCR1                  ATIM_CCR1_CCR1_Msk

/******************* Bit definition for ATIM_CCR2 register ********************/
#define ATIM_CCR2_CCR2_Pos              (0U)
#define ATIM_CCR2_CCR2_Msk              (0xFFFFFFFFUL << ATIM_CCR2_CCR2_Pos)
#define ATIM_CCR2_CCR2                  ATIM_CCR2_CCR2_Msk

/******************* Bit definition for ATIM_CCR3 register ********************/
#define ATIM_CCR3_CCR3_Pos              (0U)
#define ATIM_CCR3_CCR3_Msk              (0xFFFFFFFFUL << ATIM_CCR3_CCR3_Pos)
#define ATIM_CCR3_CCR3                  ATIM_CCR3_CCR3_Msk

/******************* Bit definition for ATIM_CCR4 register ********************/
#define ATIM_CCR4_CCR4_Pos              (0U)
#define ATIM_CCR4_CCR4_Msk              (0xFFFFFFFFUL << ATIM_CCR4_CCR4_Pos)
#define ATIM_CCR4_CCR4                  ATIM_CCR4_CCR4_Msk

/******************* Bit definition for ATIM_BDTR register ********************/
#define ATIM_BDTR_DTG_Pos               (0U)
#define ATIM_BDTR_DTG_Msk               (0x3FFUL << ATIM_BDTR_DTG_Pos)
#define ATIM_BDTR_DTG                   ATIM_BDTR_DTG_Msk
#define ATIM_BDTR_DTPSC_Pos             (11U)
#define ATIM_BDTR_DTPSC_Msk             (0x1UL << ATIM_BDTR_DTPSC_Pos)
#define ATIM_BDTR_DTPSC                 ATIM_BDTR_DTPSC_Msk
#define ATIM_BDTR_BKE_Pos               (12U)
#define ATIM_BDTR_BKE_Msk               (0x1UL << ATIM_BDTR_BKE_Pos)
#define ATIM_BDTR_BKE                   ATIM_BDTR_BKE_Msk
#define ATIM_BDTR_BKP_Pos               (13U)
#define ATIM_BDTR_BKP_Msk               (0x1UL << ATIM_BDTR_BKP_Pos)
#define ATIM_BDTR_BKP                   ATIM_BDTR_BKP_Msk
#define ATIM_BDTR_AOE_Pos               (14U)
#define ATIM_BDTR_AOE_Msk               (0x1UL << ATIM_BDTR_AOE_Pos)
#define ATIM_BDTR_AOE                   ATIM_BDTR_AOE_Msk
#define ATIM_BDTR_MOE_Pos               (15U)
#define ATIM_BDTR_MOE_Msk               (0x1UL << ATIM_BDTR_MOE_Pos)
#define ATIM_BDTR_MOE                   ATIM_BDTR_MOE_Msk
#define ATIM_BDTR_BKF_Pos               (16U)
#define ATIM_BDTR_BKF_Msk               (0xFUL << ATIM_BDTR_BKF_Pos)
#define ATIM_BDTR_BKF                   ATIM_BDTR_BKF_Msk
#define ATIM_BDTR_BK2F_Pos              (20U)
#define ATIM_BDTR_BK2F_Msk              (0xFUL << ATIM_BDTR_BK2F_Pos)
#define ATIM_BDTR_BK2F                  ATIM_BDTR_BK2F_Msk
#define ATIM_BDTR_BK2E_Pos              (24U)
#define ATIM_BDTR_BK2E_Msk              (0x1UL << ATIM_BDTR_BK2E_Pos)
#define ATIM_BDTR_BK2E                  ATIM_BDTR_BK2E_Msk
#define ATIM_BDTR_BK2P_Pos              (25U)
#define ATIM_BDTR_BK2P_Msk              (0x1UL << ATIM_BDTR_BK2P_Pos)
#define ATIM_BDTR_BK2P                  ATIM_BDTR_BK2P_Msk
#define ATIM_BDTR_BKDSRM_Pos            (26U)
#define ATIM_BDTR_BKDSRM_Msk            (0x1UL << ATIM_BDTR_BKDSRM_Pos)
#define ATIM_BDTR_BKDSRM                ATIM_BDTR_BKDSRM_Msk
#define ATIM_BDTR_BK2DSRM_Pos           (27U)
#define ATIM_BDTR_BK2DSRM_Msk           (0x1UL << ATIM_BDTR_BK2DSRM_Pos)
#define ATIM_BDTR_BK2DSRM               ATIM_BDTR_BK2DSRM_Msk
#define ATIM_BDTR_BKBID_Pos             (28U)
#define ATIM_BDTR_BKBID_Msk             (0x1UL << ATIM_BDTR_BKBID_Pos)
#define ATIM_BDTR_BKBID                 ATIM_BDTR_BKBID_Msk
#define ATIM_BDTR_BK2BID_Pos            (29U)
#define ATIM_BDTR_BK2BID_Msk            (0x1UL << ATIM_BDTR_BK2BID_Pos)
#define ATIM_BDTR_BK2BID                ATIM_BDTR_BK2BID_Msk
#define ATIM_BDTR_OSSI_Pos              (30U)
#define ATIM_BDTR_OSSI_Msk              (0x1UL << ATIM_BDTR_OSSI_Pos)
#define ATIM_BDTR_OSSI                  ATIM_BDTR_OSSI_Msk
#define ATIM_BDTR_OSSR_Pos              (31U)
#define ATIM_BDTR_OSSR_Msk              (0x1UL << ATIM_BDTR_OSSR_Pos)
#define ATIM_BDTR_OSSR                  ATIM_BDTR_OSSR_Msk

/******************* Bit definition for ATIM_CCMR3 register *******************/
#define ATIM_CCMR3_GC5C1_Pos            (13U)
#define ATIM_CCMR3_GC5C1_Msk            (0x1UL << ATIM_CCMR3_GC5C1_Pos)
#define ATIM_CCMR3_GC5C1                ATIM_CCMR3_GC5C1_Msk
#define ATIM_CCMR3_GC5C2_Pos            (14U)
#define ATIM_CCMR3_GC5C2_Msk            (0x1UL << ATIM_CCMR3_GC5C2_Pos)
#define ATIM_CCMR3_GC5C2                ATIM_CCMR3_GC5C2_Msk
#define ATIM_CCMR3_GC5C3_Pos            (15U)
#define ATIM_CCMR3_GC5C3_Msk            (0x1UL << ATIM_CCMR3_GC5C3_Pos)
#define ATIM_CCMR3_GC5C3                ATIM_CCMR3_GC5C3_Msk
#define ATIM_CCMR3_OC5CE_Pos            (16U)
#define ATIM_CCMR3_OC5CE_Msk            (0x1UL << ATIM_CCMR3_OC5CE_Pos)
#define ATIM_CCMR3_OC5CE                ATIM_CCMR3_OC5CE_Msk
#define ATIM_CCMR3_OC5PE_Pos            (19U)
#define ATIM_CCMR3_OC5PE_Msk            (0x1UL << ATIM_CCMR3_OC5PE_Pos)
#define ATIM_CCMR3_OC5PE                ATIM_CCMR3_OC5PE_Msk
#define ATIM_CCMR3_OC5M_Pos             (20U)
#define ATIM_CCMR3_OC5M_Msk             (0xFUL << ATIM_CCMR3_OC5M_Pos)
#define ATIM_CCMR3_OC5M                 ATIM_CCMR3_OC5M_Msk
#define ATIM_CCMR3_OC6CE_Pos            (24U)
#define ATIM_CCMR3_OC6CE_Msk            (0x1UL << ATIM_CCMR3_OC6CE_Pos)
#define ATIM_CCMR3_OC6CE                ATIM_CCMR3_OC6CE_Msk
#define ATIM_CCMR3_OC6PE_Pos            (27U)
#define ATIM_CCMR3_OC6PE_Msk            (0x1UL << ATIM_CCMR3_OC6PE_Pos)
#define ATIM_CCMR3_OC6PE                ATIM_CCMR3_OC6PE_Msk
#define ATIM_CCMR3_OC6M_Pos             (28U)
#define ATIM_CCMR3_OC6M_Msk             (0xFUL << ATIM_CCMR3_OC6M_Pos)
#define ATIM_CCMR3_OC6M                 ATIM_CCMR3_OC6M_Msk

/******************* Bit definition for ATIM_CCR5 register ********************/
#define ATIM_CCR5_CCR5_Pos              (0U)
#define ATIM_CCR5_CCR5_Msk              (0xFFFFFFFFUL << ATIM_CCR5_CCR5_Pos)
#define ATIM_CCR5_CCR5                  ATIM_CCR5_CCR5_Msk

/******************* Bit definition for ATIM_CCR6 register ********************/
#define ATIM_CCR6_CCR6_Pos              (0U)
#define ATIM_CCR6_CCR6_Msk              (0xFFFFFFFFUL << ATIM_CCR6_CCR6_Pos)
#define ATIM_CCR6_CCR6                  ATIM_CCR6_CCR6_Msk

/******************** Bit definition for ATIM_AF1 register ********************/
#define ATIM_AF1_BKINE_Pos              (0U)
#define ATIM_AF1_BKINE_Msk              (0x1UL << ATIM_AF1_BKINE_Pos)
#define ATIM_AF1_BKINE                  ATIM_AF1_BKINE_Msk
#define ATIM_AF1_BKCMP1E_Pos            (1U)
#define ATIM_AF1_BKCMP1E_Msk            (0x1UL << ATIM_AF1_BKCMP1E_Pos)
#define ATIM_AF1_BKCMP1E                ATIM_AF1_BKCMP1E_Msk
#define ATIM_AF1_BKCMP2E_Pos            (2U)
#define ATIM_AF1_BKCMP2E_Msk            (0x1UL << ATIM_AF1_BKCMP2E_Pos)
#define ATIM_AF1_BKCMP2E                ATIM_AF1_BKCMP2E_Msk
#define ATIM_AF1_BKINP_Pos              (9U)
#define ATIM_AF1_BKINP_Msk              (0x1UL << ATIM_AF1_BKINP_Pos)
#define ATIM_AF1_BKINP                  ATIM_AF1_BKINP_Msk
#define ATIM_AF1_BKCMP1P_Pos            (10U)
#define ATIM_AF1_BKCMP1P_Msk            (0x1UL << ATIM_AF1_BKCMP1P_Pos)
#define ATIM_AF1_BKCMP1P                ATIM_AF1_BKCMP1P_Msk
#define ATIM_AF1_BKCMP2P_Pos            (11U)
#define ATIM_AF1_BKCMP2P_Msk            (0x1UL << ATIM_AF1_BKCMP2P_Pos)
#define ATIM_AF1_BKCMP2P                ATIM_AF1_BKCMP2P_Msk
#define ATIM_AF1_ETRSEL_Pos             (14U)
#define ATIM_AF1_ETRSEL_Msk             (0x3UL << ATIM_AF1_ETRSEL_Pos)
#define ATIM_AF1_ETRSEL                 ATIM_AF1_ETRSEL_Msk
#define ATIM_AF1_LOCK_Pos               (30U)
#define ATIM_AF1_LOCK_Msk               (0x3UL << ATIM_AF1_LOCK_Pos)
#define ATIM_AF1_LOCK                   ATIM_AF1_LOCK_Msk

/******************** Bit definition for ATIM_AF2 register ********************/
#define ATIM_AF2_BK2INE_Pos             (0U)
#define ATIM_AF2_BK2INE_Msk             (0x1UL << ATIM_AF2_BK2INE_Pos)
#define ATIM_AF2_BK2INE                 ATIM_AF2_BK2INE_Msk
#define ATIM_AF2_BK2CMP1E_Pos           (1U)
#define ATIM_AF2_BK2CMP1E_Msk           (0x1UL << ATIM_AF2_BK2CMP1E_Pos)
#define ATIM_AF2_BK2CMP1E               ATIM_AF2_BK2CMP1E_Msk
#define ATIM_AF2_BK2CMP2E_Pos           (2U)
#define ATIM_AF2_BK2CMP2E_Msk           (0x1UL << ATIM_AF2_BK2CMP2E_Pos)
#define ATIM_AF2_BK2CMP2E               ATIM_AF2_BK2CMP2E_Msk
#define ATIM_AF2_BK2INP_Pos             (9U)
#define ATIM_AF2_BK2INP_Msk             (0x1UL << ATIM_AF2_BK2INP_Pos)
#define ATIM_AF2_BK2INP                 ATIM_AF2_BK2INP_Msk
#define ATIM_AF2_BK2CMP1P_Pos           (10U)
#define ATIM_AF2_BK2CMP1P_Msk           (0x1UL << ATIM_AF2_BK2CMP1P_Pos)
#define ATIM_AF2_BK2CMP1P               ATIM_AF2_BK2CMP1P_Msk
#define ATIM_AF2_BK2CMP2P_Pos           (11U)
#define ATIM_AF2_BK2CMP2P_Msk           (0x1UL << ATIM_AF2_BK2CMP2P_Pos)
#define ATIM_AF2_BK2CMP2P               ATIM_AF2_BK2CMP2P_Msk

/****************** Bit definition for ATIM_DBGSEL register *******************/
#define ATIM_DBGSEL_DBGSEL_Pos          (0U)
#define ATIM_DBGSEL_DBGSEL_Msk          (0x7UL << ATIM_DBGSEL_DBGSEL_Pos)
#define ATIM_DBGSEL_DBGSEL              ATIM_DBGSEL_DBGSEL_Msk

#endif
