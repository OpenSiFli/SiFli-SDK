#ifndef __GPT_H
#define __GPT_H

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
    __IO uint32_t RSVD1[11];
    __IO uint32_t DBGSEL;
} GPT_TypeDef;


/******************** Bit definition for GPT_CR1 register *********************/
#define GPT_CR1_CEN_Pos                 (0U)
#define GPT_CR1_CEN_Msk                 (0x1UL << GPT_CR1_CEN_Pos)
#define GPT_CR1_CEN                     GPT_CR1_CEN_Msk
#define GPT_CR1_UDIS_Pos                (1U)
#define GPT_CR1_UDIS_Msk                (0x1UL << GPT_CR1_UDIS_Pos)
#define GPT_CR1_UDIS                    GPT_CR1_UDIS_Msk
#define GPT_CR1_URS_Pos                 (2U)
#define GPT_CR1_URS_Msk                 (0x1UL << GPT_CR1_URS_Pos)
#define GPT_CR1_URS                     GPT_CR1_URS_Msk
#define GPT_CR1_OPM_Pos                 (3U)
#define GPT_CR1_OPM_Msk                 (0x1UL << GPT_CR1_OPM_Pos)
#define GPT_CR1_OPM                     GPT_CR1_OPM_Msk
#define GPT_CR1_DIR_Pos                 (4U)
#define GPT_CR1_DIR_Msk                 (0x1UL << GPT_CR1_DIR_Pos)
#define GPT_CR1_DIR                     GPT_CR1_DIR_Msk
#define GPT_CR1_CMS_Pos                 (5U)
#define GPT_CR1_CMS_Msk                 (0x3UL << GPT_CR1_CMS_Pos)
#define GPT_CR1_CMS                     GPT_CR1_CMS_Msk
#define GPT_CR1_ARPE_Pos                (7U)
#define GPT_CR1_ARPE_Msk                (0x1UL << GPT_CR1_ARPE_Pos)
#define GPT_CR1_ARPE                    GPT_CR1_ARPE_Msk
#define GPT_CR1_UIFREMAP_Pos            (11U)
#define GPT_CR1_UIFREMAP_Msk            (0x1UL << GPT_CR1_UIFREMAP_Pos)
#define GPT_CR1_UIFREMAP                GPT_CR1_UIFREMAP_Msk

/******************** Bit definition for GPT_CR2 register *********************/
#define GPT_CR2_CCDS_Pos                (3U)
#define GPT_CR2_CCDS_Msk                (0x1UL << GPT_CR2_CCDS_Pos)
#define GPT_CR2_CCDS                    GPT_CR2_CCDS_Msk
#define GPT_CR2_MMS_Pos                 (4U)
#define GPT_CR2_MMS_Msk                 (0x7UL << GPT_CR2_MMS_Pos)
#define GPT_CR2_MMS                     GPT_CR2_MMS_Msk
#define GPT_CR2_TI1S_Pos                (7U)
#define GPT_CR2_TI1S_Msk                (0x1UL << GPT_CR2_TI1S_Pos)
#define GPT_CR2_TI1S                    GPT_CR2_TI1S_Msk

/******************** Bit definition for GPT_SMCR register ********************/
#define GPT_SMCR_TS_Pos                 (4U)
#define GPT_SMCR_TS_Msk                 (0x7UL << GPT_SMCR_TS_Pos)
#define GPT_SMCR_TS                     GPT_SMCR_TS_Msk
#define GPT_SMCR_MSM_Pos                (7U)
#define GPT_SMCR_MSM_Msk                (0x1UL << GPT_SMCR_MSM_Pos)
#define GPT_SMCR_MSM                    GPT_SMCR_MSM_Msk
#define GPT_SMCR_ETF_Pos                (8U)
#define GPT_SMCR_ETF_Msk                (0xFUL << GPT_SMCR_ETF_Pos)
#define GPT_SMCR_ETF                    GPT_SMCR_ETF_Msk
#define GPT_SMCR_ETPS_Pos               (12U)
#define GPT_SMCR_ETPS_Msk               (0x3UL << GPT_SMCR_ETPS_Pos)
#define GPT_SMCR_ETPS                   GPT_SMCR_ETPS_Msk
#define GPT_SMCR_ECE_Pos                (14U)
#define GPT_SMCR_ECE_Msk                (0x1UL << GPT_SMCR_ECE_Pos)
#define GPT_SMCR_ECE                    GPT_SMCR_ECE_Msk
#define GPT_SMCR_ETP_Pos                (15U)
#define GPT_SMCR_ETP_Msk                (0x1UL << GPT_SMCR_ETP_Pos)
#define GPT_SMCR_ETP                    GPT_SMCR_ETP_Msk
#define GPT_SMCR_SMS_Pos                (16U)
#define GPT_SMCR_SMS_Msk                (0xFUL << GPT_SMCR_SMS_Pos)
#define GPT_SMCR_SMS                    GPT_SMCR_SMS_Msk

/******************** Bit definition for GPT_DIER register ********************/
#define GPT_DIER_UIE_Pos                (0U)
#define GPT_DIER_UIE_Msk                (0x1UL << GPT_DIER_UIE_Pos)
#define GPT_DIER_UIE                    GPT_DIER_UIE_Msk
#define GPT_DIER_CC1IE_Pos              (1U)
#define GPT_DIER_CC1IE_Msk              (0x1UL << GPT_DIER_CC1IE_Pos)
#define GPT_DIER_CC1IE                  GPT_DIER_CC1IE_Msk
#define GPT_DIER_CC2IE_Pos              (2U)
#define GPT_DIER_CC2IE_Msk              (0x1UL << GPT_DIER_CC2IE_Pos)
#define GPT_DIER_CC2IE                  GPT_DIER_CC2IE_Msk
#define GPT_DIER_CC3IE_Pos              (3U)
#define GPT_DIER_CC3IE_Msk              (0x1UL << GPT_DIER_CC3IE_Pos)
#define GPT_DIER_CC3IE                  GPT_DIER_CC3IE_Msk
#define GPT_DIER_CC4IE_Pos              (4U)
#define GPT_DIER_CC4IE_Msk              (0x1UL << GPT_DIER_CC4IE_Pos)
#define GPT_DIER_CC4IE                  GPT_DIER_CC4IE_Msk
#define GPT_DIER_TIE_Pos                (6U)
#define GPT_DIER_TIE_Msk                (0x1UL << GPT_DIER_TIE_Pos)
#define GPT_DIER_TIE                    GPT_DIER_TIE_Msk
#define GPT_DIER_UDE_Pos                (8U)
#define GPT_DIER_UDE_Msk                (0x1UL << GPT_DIER_UDE_Pos)
#define GPT_DIER_UDE                    GPT_DIER_UDE_Msk
#define GPT_DIER_CC1DE_Pos              (9U)
#define GPT_DIER_CC1DE_Msk              (0x1UL << GPT_DIER_CC1DE_Pos)
#define GPT_DIER_CC1DE                  GPT_DIER_CC1DE_Msk
#define GPT_DIER_CC2DE_Pos              (10U)
#define GPT_DIER_CC2DE_Msk              (0x1UL << GPT_DIER_CC2DE_Pos)
#define GPT_DIER_CC2DE                  GPT_DIER_CC2DE_Msk
#define GPT_DIER_CC3DE_Pos              (11U)
#define GPT_DIER_CC3DE_Msk              (0x1UL << GPT_DIER_CC3DE_Pos)
#define GPT_DIER_CC3DE                  GPT_DIER_CC3DE_Msk
#define GPT_DIER_CC4DE_Pos              (12U)
#define GPT_DIER_CC4DE_Msk              (0x1UL << GPT_DIER_CC4DE_Pos)
#define GPT_DIER_CC4DE                  GPT_DIER_CC4DE_Msk
#define GPT_DIER_TDE_Pos                (14U)
#define GPT_DIER_TDE_Msk                (0x1UL << GPT_DIER_TDE_Pos)
#define GPT_DIER_TDE                    GPT_DIER_TDE_Msk

/********************* Bit definition for GPT_SR register *********************/
#define GPT_SR_UIF_Pos                  (0U)
#define GPT_SR_UIF_Msk                  (0x1UL << GPT_SR_UIF_Pos)
#define GPT_SR_UIF                      GPT_SR_UIF_Msk
#define GPT_SR_CC1IF_Pos                (1U)
#define GPT_SR_CC1IF_Msk                (0x1UL << GPT_SR_CC1IF_Pos)
#define GPT_SR_CC1IF                    GPT_SR_CC1IF_Msk
#define GPT_SR_CC2IF_Pos                (2U)
#define GPT_SR_CC2IF_Msk                (0x1UL << GPT_SR_CC2IF_Pos)
#define GPT_SR_CC2IF                    GPT_SR_CC2IF_Msk
#define GPT_SR_CC3IF_Pos                (3U)
#define GPT_SR_CC3IF_Msk                (0x1UL << GPT_SR_CC3IF_Pos)
#define GPT_SR_CC3IF                    GPT_SR_CC3IF_Msk
#define GPT_SR_CC4IF_Pos                (4U)
#define GPT_SR_CC4IF_Msk                (0x1UL << GPT_SR_CC4IF_Pos)
#define GPT_SR_CC4IF                    GPT_SR_CC4IF_Msk
#define GPT_SR_TIF_Pos                  (6U)
#define GPT_SR_TIF_Msk                  (0x1UL << GPT_SR_TIF_Pos)
#define GPT_SR_TIF                      GPT_SR_TIF_Msk
#define GPT_SR_CC1OF_Pos                (9U)
#define GPT_SR_CC1OF_Msk                (0x1UL << GPT_SR_CC1OF_Pos)
#define GPT_SR_CC1OF                    GPT_SR_CC1OF_Msk
#define GPT_SR_CC2OF_Pos                (10U)
#define GPT_SR_CC2OF_Msk                (0x1UL << GPT_SR_CC2OF_Pos)
#define GPT_SR_CC2OF                    GPT_SR_CC2OF_Msk
#define GPT_SR_CC3OF_Pos                (11U)
#define GPT_SR_CC3OF_Msk                (0x1UL << GPT_SR_CC3OF_Pos)
#define GPT_SR_CC3OF                    GPT_SR_CC3OF_Msk
#define GPT_SR_CC4OF_Pos                (12U)
#define GPT_SR_CC4OF_Msk                (0x1UL << GPT_SR_CC4OF_Pos)
#define GPT_SR_CC4OF                    GPT_SR_CC4OF_Msk

/******************** Bit definition for GPT_EGR register *********************/
#define GPT_EGR_UG_Pos                  (0U)
#define GPT_EGR_UG_Msk                  (0x1UL << GPT_EGR_UG_Pos)
#define GPT_EGR_UG                      GPT_EGR_UG_Msk
#define GPT_EGR_CC1G_Pos                (1U)
#define GPT_EGR_CC1G_Msk                (0x1UL << GPT_EGR_CC1G_Pos)
#define GPT_EGR_CC1G                    GPT_EGR_CC1G_Msk
#define GPT_EGR_CC2G_Pos                (2U)
#define GPT_EGR_CC2G_Msk                (0x1UL << GPT_EGR_CC2G_Pos)
#define GPT_EGR_CC2G                    GPT_EGR_CC2G_Msk
#define GPT_EGR_CC3G_Pos                (3U)
#define GPT_EGR_CC3G_Msk                (0x1UL << GPT_EGR_CC3G_Pos)
#define GPT_EGR_CC3G                    GPT_EGR_CC3G_Msk
#define GPT_EGR_CC4G_Pos                (4U)
#define GPT_EGR_CC4G_Msk                (0x1UL << GPT_EGR_CC4G_Pos)
#define GPT_EGR_CC4G                    GPT_EGR_CC4G_Msk
#define GPT_EGR_TG_Pos                  (6U)
#define GPT_EGR_TG_Msk                  (0x1UL << GPT_EGR_TG_Pos)
#define GPT_EGR_TG                      GPT_EGR_TG_Msk

/******************* Bit definition for GPT_CCMR1 register ********************/
#define GPT_CCMR1_CC1S_Pos              (0U)
#define GPT_CCMR1_CC1S_Msk              (0x3UL << GPT_CCMR1_CC1S_Pos)
#define GPT_CCMR1_CC1S                  GPT_CCMR1_CC1S_Msk
#define GPT_CCMR1_IC1PSC_Pos            (2U)
#define GPT_CCMR1_IC1PSC_Msk            (0x3UL << GPT_CCMR1_IC1PSC_Pos)
#define GPT_CCMR1_IC1PSC                GPT_CCMR1_IC1PSC_Msk
#define GPT_CCMR1_IC1F_Pos              (4U)
#define GPT_CCMR1_IC1F_Msk              (0xFUL << GPT_CCMR1_IC1F_Pos)
#define GPT_CCMR1_IC1F                  GPT_CCMR1_IC1F_Msk
#define GPT_CCMR1_CC2S_Pos              (8U)
#define GPT_CCMR1_CC2S_Msk              (0x3UL << GPT_CCMR1_CC2S_Pos)
#define GPT_CCMR1_CC2S                  GPT_CCMR1_CC2S_Msk
#define GPT_CCMR1_IC2PSC_Pos            (10U)
#define GPT_CCMR1_IC2PSC_Msk            (0x3UL << GPT_CCMR1_IC2PSC_Pos)
#define GPT_CCMR1_IC2PSC                GPT_CCMR1_IC2PSC_Msk
#define GPT_CCMR1_IC2F_Pos              (12U)
#define GPT_CCMR1_IC2F_Msk              (0xFUL << GPT_CCMR1_IC2F_Pos)
#define GPT_CCMR1_IC2F                  GPT_CCMR1_IC2F_Msk
#define GPT_CCMR1_OC1CE_Pos             (16U)
#define GPT_CCMR1_OC1CE_Msk             (0x1UL << GPT_CCMR1_OC1CE_Pos)
#define GPT_CCMR1_OC1CE                 GPT_CCMR1_OC1CE_Msk
#define GPT_CCMR1_OC1PE_Pos             (19U)
#define GPT_CCMR1_OC1PE_Msk             (0x1UL << GPT_CCMR1_OC1PE_Pos)
#define GPT_CCMR1_OC1PE                 GPT_CCMR1_OC1PE_Msk
#define GPT_CCMR1_OC1M_Pos              (20U)
#define GPT_CCMR1_OC1M_Msk              (0xFUL << GPT_CCMR1_OC1M_Pos)
#define GPT_CCMR1_OC1M                  GPT_CCMR1_OC1M_Msk
#define GPT_CCMR1_OC2CE_Pos             (24U)
#define GPT_CCMR1_OC2CE_Msk             (0x1UL << GPT_CCMR1_OC2CE_Pos)
#define GPT_CCMR1_OC2CE                 GPT_CCMR1_OC2CE_Msk
#define GPT_CCMR1_OC2PE_Pos             (27U)
#define GPT_CCMR1_OC2PE_Msk             (0x1UL << GPT_CCMR1_OC2PE_Pos)
#define GPT_CCMR1_OC2PE                 GPT_CCMR1_OC2PE_Msk
#define GPT_CCMR1_OC2M_Pos              (28U)
#define GPT_CCMR1_OC2M_Msk              (0xFUL << GPT_CCMR1_OC2M_Pos)
#define GPT_CCMR1_OC2M                  GPT_CCMR1_OC2M_Msk

/******************* Bit definition for GPT_CCMR2 register ********************/
#define GPT_CCMR2_CC3S_Pos              (0U)
#define GPT_CCMR2_CC3S_Msk              (0x3UL << GPT_CCMR2_CC3S_Pos)
#define GPT_CCMR2_CC3S                  GPT_CCMR2_CC3S_Msk
#define GPT_CCMR2_IC3PSC_Pos            (2U)
#define GPT_CCMR2_IC3PSC_Msk            (0x3UL << GPT_CCMR2_IC3PSC_Pos)
#define GPT_CCMR2_IC3PSC                GPT_CCMR2_IC3PSC_Msk
#define GPT_CCMR2_IC3F_Pos              (4U)
#define GPT_CCMR2_IC3F_Msk              (0xFUL << GPT_CCMR2_IC3F_Pos)
#define GPT_CCMR2_IC3F                  GPT_CCMR2_IC3F_Msk
#define GPT_CCMR2_CC4S_Pos              (8U)
#define GPT_CCMR2_CC4S_Msk              (0x3UL << GPT_CCMR2_CC4S_Pos)
#define GPT_CCMR2_CC4S                  GPT_CCMR2_CC4S_Msk
#define GPT_CCMR2_IC4PSC_Pos            (10U)
#define GPT_CCMR2_IC4PSC_Msk            (0x3UL << GPT_CCMR2_IC4PSC_Pos)
#define GPT_CCMR2_IC4PSC                GPT_CCMR2_IC4PSC_Msk
#define GPT_CCMR2_IC4F_Pos              (12U)
#define GPT_CCMR2_IC4F_Msk              (0xFUL << GPT_CCMR2_IC4F_Pos)
#define GPT_CCMR2_IC4F                  GPT_CCMR2_IC4F_Msk
#define GPT_CCMR2_OC3CE_Pos             (16U)
#define GPT_CCMR2_OC3CE_Msk             (0x1UL << GPT_CCMR2_OC3CE_Pos)
#define GPT_CCMR2_OC3CE                 GPT_CCMR2_OC3CE_Msk
#define GPT_CCMR2_OC3PE_Pos             (19U)
#define GPT_CCMR2_OC3PE_Msk             (0x1UL << GPT_CCMR2_OC3PE_Pos)
#define GPT_CCMR2_OC3PE                 GPT_CCMR2_OC3PE_Msk
#define GPT_CCMR2_OC3M_Pos              (20U)
#define GPT_CCMR2_OC3M_Msk              (0xFUL << GPT_CCMR2_OC3M_Pos)
#define GPT_CCMR2_OC3M                  GPT_CCMR2_OC3M_Msk
#define GPT_CCMR2_OC4CE_Pos             (24U)
#define GPT_CCMR2_OC4CE_Msk             (0x1UL << GPT_CCMR2_OC4CE_Pos)
#define GPT_CCMR2_OC4CE                 GPT_CCMR2_OC4CE_Msk
#define GPT_CCMR2_OC4PE_Pos             (27U)
#define GPT_CCMR2_OC4PE_Msk             (0x1UL << GPT_CCMR2_OC4PE_Pos)
#define GPT_CCMR2_OC4PE                 GPT_CCMR2_OC4PE_Msk
#define GPT_CCMR2_OC4M_Pos              (28U)
#define GPT_CCMR2_OC4M_Msk              (0xFUL << GPT_CCMR2_OC4M_Pos)
#define GPT_CCMR2_OC4M                  GPT_CCMR2_OC4M_Msk

/******************** Bit definition for GPT_CCER register ********************/
#define GPT_CCER_CC1E_Pos               (0U)
#define GPT_CCER_CC1E_Msk               (0x1UL << GPT_CCER_CC1E_Pos)
#define GPT_CCER_CC1E                   GPT_CCER_CC1E_Msk
#define GPT_CCER_CC1P_Pos               (1U)
#define GPT_CCER_CC1P_Msk               (0x1UL << GPT_CCER_CC1P_Pos)
#define GPT_CCER_CC1P                   GPT_CCER_CC1P_Msk
#define GPT_CCER_CC1NP_Pos              (3U)
#define GPT_CCER_CC1NP_Msk              (0x1UL << GPT_CCER_CC1NP_Pos)
#define GPT_CCER_CC1NP                  GPT_CCER_CC1NP_Msk
#define GPT_CCER_CC2E_Pos               (4U)
#define GPT_CCER_CC2E_Msk               (0x1UL << GPT_CCER_CC2E_Pos)
#define GPT_CCER_CC2E                   GPT_CCER_CC2E_Msk
#define GPT_CCER_CC2P_Pos               (5U)
#define GPT_CCER_CC2P_Msk               (0x1UL << GPT_CCER_CC2P_Pos)
#define GPT_CCER_CC2P                   GPT_CCER_CC2P_Msk
#define GPT_CCER_CC2NP_Pos              (7U)
#define GPT_CCER_CC2NP_Msk              (0x1UL << GPT_CCER_CC2NP_Pos)
#define GPT_CCER_CC2NP                  GPT_CCER_CC2NP_Msk
#define GPT_CCER_CC3E_Pos               (8U)
#define GPT_CCER_CC3E_Msk               (0x1UL << GPT_CCER_CC3E_Pos)
#define GPT_CCER_CC3E                   GPT_CCER_CC3E_Msk
#define GPT_CCER_CC3P_Pos               (9U)
#define GPT_CCER_CC3P_Msk               (0x1UL << GPT_CCER_CC3P_Pos)
#define GPT_CCER_CC3P                   GPT_CCER_CC3P_Msk
#define GPT_CCER_CC3NP_Pos              (11U)
#define GPT_CCER_CC3NP_Msk              (0x1UL << GPT_CCER_CC3NP_Pos)
#define GPT_CCER_CC3NP                  GPT_CCER_CC3NP_Msk
#define GPT_CCER_CC4E_Pos               (12U)
#define GPT_CCER_CC4E_Msk               (0x1UL << GPT_CCER_CC4E_Pos)
#define GPT_CCER_CC4E                   GPT_CCER_CC4E_Msk
#define GPT_CCER_CC4P_Pos               (13U)
#define GPT_CCER_CC4P_Msk               (0x1UL << GPT_CCER_CC4P_Pos)
#define GPT_CCER_CC4P                   GPT_CCER_CC4P_Msk
#define GPT_CCER_CC4NP_Pos              (15U)
#define GPT_CCER_CC4NP_Msk              (0x1UL << GPT_CCER_CC4NP_Pos)
#define GPT_CCER_CC4NP                  GPT_CCER_CC4NP_Msk

/******************** Bit definition for GPT_CNT register *********************/
#define GPT_CNT_CNT_Pos                 (0U)
#define GPT_CNT_CNT_Msk                 (0xFFFFUL << GPT_CNT_CNT_Pos)
#define GPT_CNT_CNT                     GPT_CNT_CNT_Msk
#define GPT_CNT_UIFCPY_Pos              (31U)
#define GPT_CNT_UIFCPY_Msk              (0x1UL << GPT_CNT_UIFCPY_Pos)
#define GPT_CNT_UIFCPY                  GPT_CNT_UIFCPY_Msk

/******************** Bit definition for GPT_PSC register *********************/
#define GPT_PSC_PSC_Pos                 (0U)
#define GPT_PSC_PSC_Msk                 (0xFFFFUL << GPT_PSC_PSC_Pos)
#define GPT_PSC_PSC                     GPT_PSC_PSC_Msk

/******************** Bit definition for GPT_ARR register *********************/
#define GPT_ARR_ARR_Pos                 (0U)
#define GPT_ARR_ARR_Msk                 (0xFFFFUL << GPT_ARR_ARR_Pos)
#define GPT_ARR_ARR                     GPT_ARR_ARR_Msk

/******************** Bit definition for GPT_RCR register *********************/
#define GPT_RCR_REP_Pos                 (0U)
#define GPT_RCR_REP_Msk                 (0xFFUL << GPT_RCR_REP_Pos)
#define GPT_RCR_REP                     GPT_RCR_REP_Msk

/******************** Bit definition for GPT_CCR1 register ********************/
#define GPT_CCR1_CCR1_Pos               (0U)
#define GPT_CCR1_CCR1_Msk               (0xFFFFUL << GPT_CCR1_CCR1_Pos)
#define GPT_CCR1_CCR1                   GPT_CCR1_CCR1_Msk

/******************** Bit definition for GPT_CCR2 register ********************/
#define GPT_CCR2_CCR2_Pos               (0U)
#define GPT_CCR2_CCR2_Msk               (0xFFFFUL << GPT_CCR2_CCR2_Pos)
#define GPT_CCR2_CCR2                   GPT_CCR2_CCR2_Msk

/******************** Bit definition for GPT_CCR3 register ********************/
#define GPT_CCR3_CCR3_Pos               (0U)
#define GPT_CCR3_CCR3_Msk               (0xFFFFUL << GPT_CCR3_CCR3_Pos)
#define GPT_CCR3_CCR3                   GPT_CCR3_CCR3_Msk

/******************** Bit definition for GPT_CCR4 register ********************/
#define GPT_CCR4_CCR4_Pos               (0U)
#define GPT_CCR4_CCR4_Msk               (0xFFFFUL << GPT_CCR4_CCR4_Pos)
#define GPT_CCR4_CCR4                   GPT_CCR4_CCR4_Msk

/******************* Bit definition for GPT_DBGSEL register *******************/
#define GPT_DBGSEL_DBGSEL_Pos           (0U)
#define GPT_DBGSEL_DBGSEL_Msk           (0x7UL << GPT_DBGSEL_DBGSEL_Pos)
#define GPT_DBGSEL_DBGSEL               GPT_DBGSEL_DBGSEL_Msk

#endif
