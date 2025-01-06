#ifndef __I2C_H
#define __I2C_H

typedef struct
{
    __IO uint32_t CR;
    __IO uint32_t TCR;
    __IO uint32_t IER;
    __IO uint32_t SR;
    __IO uint32_t DBR;
    __IO uint32_t SAR;
    __IO uint32_t LCR;
    __IO uint32_t WCR;
    __IO uint32_t RCCR;
    __IO uint32_t BMR;
    __IO uint32_t DNR;
    __IO uint32_t RSVD1;
    __IO uint32_t FIFO;
} I2C_TypeDef;


/********************* Bit definition for I2C_CR register *********************/
#define I2C_CR_MODE_Pos                 (0U)
#define I2C_CR_MODE_Msk                 (0x3UL << I2C_CR_MODE_Pos)
#define I2C_CR_MODE                     I2C_CR_MODE_Msk
#define I2C_CR_IUE_Pos                  (2U)
#define I2C_CR_IUE_Msk                  (0x1UL << I2C_CR_IUE_Pos)
#define I2C_CR_IUE                      I2C_CR_IUE_Msk
#define I2C_CR_SCLE_Pos                 (3U)
#define I2C_CR_SCLE_Msk                 (0x1UL << I2C_CR_SCLE_Pos)
#define I2C_CR_SCLE                     I2C_CR_SCLE_Msk
#define I2C_CR_DMAEN_Pos                (4U)
#define I2C_CR_DMAEN_Msk                (0x1UL << I2C_CR_DMAEN_Pos)
#define I2C_CR_DMAEN                    I2C_CR_DMAEN_Msk
#define I2C_CR_LASTNACK_Pos             (5U)
#define I2C_CR_LASTNACK_Msk             (0x1UL << I2C_CR_LASTNACK_Pos)
#define I2C_CR_LASTNACK                 I2C_CR_LASTNACK_Msk
#define I2C_CR_LASTSTOP_Pos             (6U)
#define I2C_CR_LASTSTOP_Msk             (0x1UL << I2C_CR_LASTSTOP_Pos)
#define I2C_CR_LASTSTOP                 I2C_CR_LASTSTOP_Msk
#define I2C_CR_GCD_Pos                  (7U)
#define I2C_CR_GCD_Msk                  (0x1UL << I2C_CR_GCD_Pos)
#define I2C_CR_GCD                      I2C_CR_GCD_Msk
#define I2C_CR_MSDE_Pos                 (8U)
#define I2C_CR_MSDE_Msk                 (0x1UL << I2C_CR_MSDE_Pos)
#define I2C_CR_MSDE                     I2C_CR_MSDE_Msk
#define I2C_CR_SCLPP_Pos                (9U)
#define I2C_CR_SCLPP_Msk                (0x1UL << I2C_CR_SCLPP_Pos)
#define I2C_CR_SCLPP                    I2C_CR_SCLPP_Msk
#define I2C_CR_SDAGCD_Pos               (10U)
#define I2C_CR_SDAGCD_Msk               (0x1UL << I2C_CR_SDAGCD_Pos)
#define I2C_CR_SDAGCD                   I2C_CR_SDAGCD_Msk
#define I2C_CR_SLVEN_Pos                (11U)
#define I2C_CR_SLVEN_Msk                (0x1UL << I2C_CR_SLVEN_Pos)
#define I2C_CR_SLVEN                    I2C_CR_SLVEN_Msk
#define I2C_CR_DNF_Pos                  (12U)
#define I2C_CR_DNF_Msk                  (0x7UL << I2C_CR_DNF_Pos)
#define I2C_CR_DNF                      I2C_CR_DNF_Msk
#define I2C_CR_BRGRST_Pos               (29U)
#define I2C_CR_BRGRST_Msk               (0x1UL << I2C_CR_BRGRST_Pos)
#define I2C_CR_BRGRST                   I2C_CR_BRGRST_Msk
#define I2C_CR_RSTREQ_Pos               (30U)
#define I2C_CR_RSTREQ_Msk               (0x1UL << I2C_CR_RSTREQ_Pos)
#define I2C_CR_RSTREQ                   I2C_CR_RSTREQ_Msk
#define I2C_CR_UR_Pos                   (31U)
#define I2C_CR_UR_Msk                   (0x1UL << I2C_CR_UR_Pos)
#define I2C_CR_UR                       I2C_CR_UR_Msk

/******************** Bit definition for I2C_TCR register *********************/
#define I2C_TCR_TB_Pos                  (0U)
#define I2C_TCR_TB_Msk                  (0x1UL << I2C_TCR_TB_Pos)
#define I2C_TCR_TB                      I2C_TCR_TB_Msk
#define I2C_TCR_START_Pos               (1U)
#define I2C_TCR_START_Msk               (0x1UL << I2C_TCR_START_Pos)
#define I2C_TCR_START                   I2C_TCR_START_Msk
#define I2C_TCR_STOP_Pos                (2U)
#define I2C_TCR_STOP_Msk                (0x1UL << I2C_TCR_STOP_Pos)
#define I2C_TCR_STOP                    I2C_TCR_STOP_Msk
#define I2C_TCR_NACK_Pos                (3U)
#define I2C_TCR_NACK_Msk                (0x1UL << I2C_TCR_NACK_Pos)
#define I2C_TCR_NACK                    I2C_TCR_NACK_Msk
#define I2C_TCR_MA_Pos                  (4U)
#define I2C_TCR_MA_Msk                  (0x1UL << I2C_TCR_MA_Pos)
#define I2C_TCR_MA                      I2C_TCR_MA_Msk
#define I2C_TCR_TXREQ_Pos               (5U)
#define I2C_TCR_TXREQ_Msk               (0x1UL << I2C_TCR_TXREQ_Pos)
#define I2C_TCR_TXREQ                   I2C_TCR_TXREQ_Msk
#define I2C_TCR_RXREQ_Pos               (6U)
#define I2C_TCR_RXREQ_Msk               (0x1UL << I2C_TCR_RXREQ_Pos)
#define I2C_TCR_RXREQ                   I2C_TCR_RXREQ_Msk
#define I2C_TCR_ABORTDMA_Pos            (7U)
#define I2C_TCR_ABORTDMA_Msk            (0x1UL << I2C_TCR_ABORTDMA_Pos)
#define I2C_TCR_ABORTDMA                I2C_TCR_ABORTDMA_Msk

/******************** Bit definition for I2C_IER register *********************/
#define I2C_IER_SSDIE_Pos               (4U)
#define I2C_IER_SSDIE_Msk               (0x1UL << I2C_IER_SSDIE_Pos)
#define I2C_IER_SSDIE                   I2C_IER_SSDIE_Msk
#define I2C_IER_ALDIE_Pos               (5U)
#define I2C_IER_ALDIE_Msk               (0x1UL << I2C_IER_ALDIE_Pos)
#define I2C_IER_ALDIE                   I2C_IER_ALDIE_Msk
#define I2C_IER_TEIE_Pos                (6U)
#define I2C_IER_TEIE_Msk                (0x1UL << I2C_IER_TEIE_Pos)
#define I2C_IER_TEIE                    I2C_IER_TEIE_Msk
#define I2C_IER_RFIE_Pos                (7U)
#define I2C_IER_RFIE_Msk                (0x1UL << I2C_IER_RFIE_Pos)
#define I2C_IER_RFIE                    I2C_IER_RFIE_Msk
#define I2C_IER_GCADIE_Pos              (8U)
#define I2C_IER_GCADIE_Msk              (0x1UL << I2C_IER_GCADIE_Pos)
#define I2C_IER_GCADIE                  I2C_IER_GCADIE_Msk
#define I2C_IER_SADIE_Pos               (9U)
#define I2C_IER_SADIE_Msk               (0x1UL << I2C_IER_SADIE_Pos)
#define I2C_IER_SADIE                   I2C_IER_SADIE_Msk
#define I2C_IER_BEDIE_Pos               (10U)
#define I2C_IER_BEDIE_Msk               (0x1UL << I2C_IER_BEDIE_Pos)
#define I2C_IER_BEDIE                   I2C_IER_BEDIE_Msk
#define I2C_IER_MSDIE_Pos               (12U)
#define I2C_IER_MSDIE_Msk               (0x1UL << I2C_IER_MSDIE_Pos)
#define I2C_IER_MSDIE                   I2C_IER_MSDIE_Msk
#define I2C_IER_DMADONEIE_Pos           (13U)
#define I2C_IER_DMADONEIE_Msk           (0x1UL << I2C_IER_DMADONEIE_Pos)
#define I2C_IER_DMADONEIE               I2C_IER_DMADONEIE_Msk
#define I2C_IER_OFIE_Pos                (14U)
#define I2C_IER_OFIE_Msk                (0x1UL << I2C_IER_OFIE_Pos)
#define I2C_IER_OFIE                    I2C_IER_OFIE_Msk
#define I2C_IER_UFIE_Pos                (15U)
#define I2C_IER_UFIE_Msk                (0x1UL << I2C_IER_UFIE_Pos)
#define I2C_IER_UFIE                    I2C_IER_UFIE_Msk

/********************* Bit definition for I2C_SR register *********************/
#define I2C_SR_RWM_Pos                  (0U)
#define I2C_SR_RWM_Msk                  (0x1UL << I2C_SR_RWM_Pos)
#define I2C_SR_RWM                      I2C_SR_RWM_Msk
#define I2C_SR_NACK_Pos                 (1U)
#define I2C_SR_NACK_Msk                 (0x1UL << I2C_SR_NACK_Pos)
#define I2C_SR_NACK                     I2C_SR_NACK_Msk
#define I2C_SR_UB_Pos                   (2U)
#define I2C_SR_UB_Msk                   (0x1UL << I2C_SR_UB_Pos)
#define I2C_SR_UB                       I2C_SR_UB_Msk
#define I2C_SR_IBB_Pos                  (3U)
#define I2C_SR_IBB_Msk                  (0x1UL << I2C_SR_IBB_Pos)
#define I2C_SR_IBB                      I2C_SR_IBB_Msk
#define I2C_SR_SSD_Pos                  (4U)
#define I2C_SR_SSD_Msk                  (0x1UL << I2C_SR_SSD_Pos)
#define I2C_SR_SSD                      I2C_SR_SSD_Msk
#define I2C_SR_ALD_Pos                  (5U)
#define I2C_SR_ALD_Msk                  (0x1UL << I2C_SR_ALD_Pos)
#define I2C_SR_ALD                      I2C_SR_ALD_Msk
#define I2C_SR_TE_Pos                   (6U)
#define I2C_SR_TE_Msk                   (0x1UL << I2C_SR_TE_Pos)
#define I2C_SR_TE                       I2C_SR_TE_Msk
#define I2C_SR_RF_Pos                   (7U)
#define I2C_SR_RF_Msk                   (0x1UL << I2C_SR_RF_Pos)
#define I2C_SR_RF                       I2C_SR_RF_Msk
#define I2C_SR_GCAD_Pos                 (8U)
#define I2C_SR_GCAD_Msk                 (0x1UL << I2C_SR_GCAD_Pos)
#define I2C_SR_GCAD                     I2C_SR_GCAD_Msk
#define I2C_SR_SAD_Pos                  (9U)
#define I2C_SR_SAD_Msk                  (0x1UL << I2C_SR_SAD_Pos)
#define I2C_SR_SAD                      I2C_SR_SAD_Msk
#define I2C_SR_BED_Pos                  (10U)
#define I2C_SR_BED_Msk                  (0x1UL << I2C_SR_BED_Pos)
#define I2C_SR_BED                      I2C_SR_BED_Msk
#define I2C_SR_EBB_Pos                  (11U)
#define I2C_SR_EBB_Msk                  (0x1UL << I2C_SR_EBB_Pos)
#define I2C_SR_EBB                      I2C_SR_EBB_Msk
#define I2C_SR_MSD_Pos                  (12U)
#define I2C_SR_MSD_Msk                  (0x1UL << I2C_SR_MSD_Pos)
#define I2C_SR_MSD                      I2C_SR_MSD_Msk
#define I2C_SR_DMADONE_Pos              (13U)
#define I2C_SR_DMADONE_Msk              (0x1UL << I2C_SR_DMADONE_Pos)
#define I2C_SR_DMADONE                  I2C_SR_DMADONE_Msk
#define I2C_SR_OF_Pos                   (14U)
#define I2C_SR_OF_Msk                   (0x1UL << I2C_SR_OF_Pos)
#define I2C_SR_OF                       I2C_SR_OF_Msk
#define I2C_SR_UF_Pos                   (15U)
#define I2C_SR_UF_Msk                   (0x1UL << I2C_SR_UF_Pos)
#define I2C_SR_UF                       I2C_SR_UF_Msk

/******************** Bit definition for I2C_DBR register *********************/
#define I2C_DBR_DATA_Pos                (0U)
#define I2C_DBR_DATA_Msk                (0xFFUL << I2C_DBR_DATA_Pos)
#define I2C_DBR_DATA                    I2C_DBR_DATA_Msk

/******************** Bit definition for I2C_SAR register *********************/
#define I2C_SAR_ADDR_Pos                (0U)
#define I2C_SAR_ADDR_Msk                (0x7FUL << I2C_SAR_ADDR_Pos)
#define I2C_SAR_ADDR                    I2C_SAR_ADDR_Msk

/******************** Bit definition for I2C_LCR register *********************/
#define I2C_LCR_SLV_Pos                 (0U)
#define I2C_LCR_SLV_Msk                 (0x1FFUL << I2C_LCR_SLV_Pos)
#define I2C_LCR_SLV                     I2C_LCR_SLV_Msk
#define I2C_LCR_FLV_Pos                 (9U)
#define I2C_LCR_FLV_Msk                 (0x1FFUL << I2C_LCR_FLV_Pos)
#define I2C_LCR_FLV                     I2C_LCR_FLV_Msk
#define I2C_LCR_HLVL_Pos                (18U)
#define I2C_LCR_HLVL_Msk                (0x1FFUL << I2C_LCR_HLVL_Pos)
#define I2C_LCR_HLVL                    I2C_LCR_HLVL_Msk
#define I2C_LCR_HLVH_Pos                (27U)
#define I2C_LCR_HLVH_Msk                (0x1FUL << I2C_LCR_HLVH_Pos)
#define I2C_LCR_HLVH                    I2C_LCR_HLVH_Msk

/******************** Bit definition for I2C_WCR register *********************/
#define I2C_WCR_CNT_Pos                 (0U)
#define I2C_WCR_CNT_Msk                 (0xFFUL << I2C_WCR_CNT_Pos)
#define I2C_WCR_CNT                     I2C_WCR_CNT_Msk
#define I2C_WCR_HSCNT1_Pos              (8U)
#define I2C_WCR_HSCNT1_Msk              (0x1FUL << I2C_WCR_HSCNT1_Pos)
#define I2C_WCR_HSCNT1                  I2C_WCR_HSCNT1_Msk
#define I2C_WCR_HSCNT2_Pos              (13U)
#define I2C_WCR_HSCNT2_Msk              (0x1FUL << I2C_WCR_HSCNT2_Pos)
#define I2C_WCR_HSCNT2                  I2C_WCR_HSCNT2_Msk
#define I2C_WCR_DIV_Pos                 (28U)
#define I2C_WCR_DIV_Msk                 (0x3UL << I2C_WCR_DIV_Pos)
#define I2C_WCR_DIV                     I2C_WCR_DIV_Msk

/******************** Bit definition for I2C_RCCR register ********************/
#define I2C_RCCR_RSTCYC_Pos             (0U)
#define I2C_RCCR_RSTCYC_Msk             (0xFUL << I2C_RCCR_RSTCYC_Pos)
#define I2C_RCCR_RSTCYC                 I2C_RCCR_RSTCYC_Msk

/******************** Bit definition for I2C_BMR register *********************/
#define I2C_BMR_SDA_Pos                 (0U)
#define I2C_BMR_SDA_Msk                 (0x1UL << I2C_BMR_SDA_Pos)
#define I2C_BMR_SDA                     I2C_BMR_SDA_Msk
#define I2C_BMR_SCL_Pos                 (1U)
#define I2C_BMR_SCL_Msk                 (0x1UL << I2C_BMR_SCL_Pos)
#define I2C_BMR_SCL                     I2C_BMR_SCL_Msk

/******************** Bit definition for I2C_DNR register *********************/
#define I2C_DNR_NDT_Pos                 (0U)
#define I2C_DNR_NDT_Msk                 (0x1FFUL << I2C_DNR_NDT_Pos)
#define I2C_DNR_NDT                     I2C_DNR_NDT_Msk

/******************** Bit definition for I2C_FIFO register ********************/
#define I2C_FIFO_DATA_Pos               (0U)
#define I2C_FIFO_DATA_Msk               (0xFFUL << I2C_FIFO_DATA_Pos)
#define I2C_FIFO_DATA                   I2C_FIFO_DATA_Msk

#endif
