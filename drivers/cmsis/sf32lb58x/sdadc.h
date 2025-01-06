#ifndef __SDADC_H
#define __SDADC_H

typedef struct
{
    __IO uint32_t CFG0;
    __IO uint32_t CFG1;
    __IO uint32_t CFG2;
    __IO uint32_t CFG3;
    __IO uint32_t CAL;
    __IO uint32_t TRIG;
    __IO uint32_t CH0_CFG;
    __IO uint32_t CH1_CFG;
    __IO uint32_t CH2_CFG;
    __IO uint32_t CH3_CFG;
    __IO uint32_t CH4_CFG;
    __IO uint32_t CH0_DOUT;
    __IO uint32_t CH1_DOUT;
    __IO uint32_t CH2_DOUT;
    __IO uint32_t CH3_DOUT;
    __IO uint32_t CH4_DOUT;
    __IO uint32_t SINGLE_DOUT;
    __IO uint32_t SINC_CFG;
    __IO uint32_t COMP_CFG0;
    __IO uint32_t COMP_CFG1;
    __IO uint32_t RSVD1[6];
    __IO uint32_t LPF_CFG6;
    __IO uint32_t DMA_CFG;
    __IO uint32_t ADC_DATA;
    __IO uint32_t FIFO_ST;
    __IO uint32_t INT_ST;
    __IO uint32_t INT_MSK;
    __IO uint32_t INT_CLR;
    __IO uint32_t FSM_ST;
    __IO uint32_t RSVD;
} SDADC_TypeDef;


/******************* Bit definition for SDADC_CFG0 register *******************/
#define SDADC_CFG0_PU_LV_Pos            (0U)
#define SDADC_CFG0_PU_LV_Msk            (0x1UL << SDADC_CFG0_PU_LV_Pos)
#define SDADC_CFG0_PU_LV                SDADC_CFG0_PU_LV_Msk
#define SDADC_CFG0_CLRLEN_LV_Pos        (1U)
#define SDADC_CFG0_CLRLEN_LV_Msk        (0x3UL << SDADC_CFG0_CLRLEN_LV_Pos)
#define SDADC_CFG0_CLRLEN_LV            SDADC_CFG0_CLRLEN_LV_Msk
#define SDADC_CFG0_RUN_LV_Pos           (3U)
#define SDADC_CFG0_RUN_LV_Msk           (0x1UL << SDADC_CFG0_RUN_LV_Pos)
#define SDADC_CFG0_RUN_LV               SDADC_CFG0_RUN_LV_Msk
#define SDADC_CFG0_FCK_SEL_LV_Pos       (4U)
#define SDADC_CFG0_FCK_SEL_LV_Msk       (0x3UL << SDADC_CFG0_FCK_SEL_LV_Pos)
#define SDADC_CFG0_FCK_SEL_LV           SDADC_CFG0_FCK_SEL_LV_Msk
#define SDADC_CFG0_LDO_SEL_LV_Pos       (6U)
#define SDADC_CFG0_LDO_SEL_LV_Msk       (0xFUL << SDADC_CFG0_LDO_SEL_LV_Pos)
#define SDADC_CFG0_LDO_SEL_LV           SDADC_CFG0_LDO_SEL_LV_Msk
#define SDADC_CFG0_CONTIN_LV_Pos        (10U)
#define SDADC_CFG0_CONTIN_LV_Msk        (0x1UL << SDADC_CFG0_CONTIN_LV_Pos)
#define SDADC_CFG0_CONTIN_LV            SDADC_CFG0_CONTIN_LV_Msk
#define SDADC_CFG0_CLK_INV_LV_Pos       (11U)
#define SDADC_CFG0_CLK_INV_LV_Msk       (0x1UL << SDADC_CFG0_CLK_INV_LV_Pos)
#define SDADC_CFG0_CLK_INV_LV           SDADC_CFG0_CLK_INV_LV_Msk
#define SDADC_CFG0_DSAMPLE_MODE_Pos     (12U)
#define SDADC_CFG0_DSAMPLE_MODE_Msk     (0x1UL << SDADC_CFG0_DSAMPLE_MODE_Pos)
#define SDADC_CFG0_DSAMPLE_MODE         SDADC_CFG0_DSAMPLE_MODE_Msk
#define SDADC_CFG0_SDADC_DATA_RDY_Pos   (13U)
#define SDADC_CFG0_SDADC_DATA_RDY_Msk   (0x1UL << SDADC_CFG0_SDADC_DATA_RDY_Pos)
#define SDADC_CFG0_SDADC_DATA_RDY       SDADC_CFG0_SDADC_DATA_RDY_Msk
#define SDADC_CFG0_DC_TR_LV_Pos         (14U)
#define SDADC_CFG0_DC_TR_LV_Msk         (0x7UL << SDADC_CFG0_DC_TR_LV_Pos)
#define SDADC_CFG0_DC_TR_LV             SDADC_CFG0_DC_TR_LV_Msk
#define SDADC_CFG0_HSUP_LV_Pos          (17U)
#define SDADC_CFG0_HSUP_LV_Msk          (0x1UL << SDADC_CFG0_HSUP_LV_Pos)
#define SDADC_CFG0_HSUP_LV              SDADC_CFG0_HSUP_LV_Msk
#define SDADC_CFG0_PPU_LV_Pos           (18U)
#define SDADC_CFG0_PPU_LV_Msk           (0x1UL << SDADC_CFG0_PPU_LV_Pos)
#define SDADC_CFG0_PPU_LV               SDADC_CFG0_PPU_LV_Msk

/******************* Bit definition for SDADC_CFG1 register *******************/
#define SDADC_CFG1_CHOP_EN_LV_Pos       (0U)
#define SDADC_CFG1_CHOP_EN_LV_Msk       (0x1UL << SDADC_CFG1_CHOP_EN_LV_Pos)
#define SDADC_CFG1_CHOP_EN_LV           SDADC_CFG1_CHOP_EN_LV_Msk
#define SDADC_CFG1_FCHOP_SIG_SEL_LV_Pos  (1U)
#define SDADC_CFG1_FCHOP_SIG_SEL_LV_Msk  (0x3UL << SDADC_CFG1_FCHOP_SIG_SEL_LV_Pos)
#define SDADC_CFG1_FCHOP_SIG_SEL_LV     SDADC_CFG1_FCHOP_SIG_SEL_LV_Msk
#define SDADC_CFG1_CHOP_STOP_EN_LV_Pos  (3U)
#define SDADC_CFG1_CHOP_STOP_EN_LV_Msk  (0x1UL << SDADC_CFG1_CHOP_STOP_EN_LV_Pos)
#define SDADC_CFG1_CHOP_STOP_EN_LV      SDADC_CFG1_CHOP_STOP_EN_LV_Msk
#define SDADC_CFG1_CHOP1_NUM_LV_Pos     (4U)
#define SDADC_CFG1_CHOP1_NUM_LV_Msk     (0x1FFUL << SDADC_CFG1_CHOP1_NUM_LV_Pos)
#define SDADC_CFG1_CHOP1_NUM_LV         SDADC_CFG1_CHOP1_NUM_LV_Msk
#define SDADC_CFG1_CHOP2_NUM_LV_Pos     (13U)
#define SDADC_CFG1_CHOP2_NUM_LV_Msk     (0x1FFUL << SDADC_CFG1_CHOP2_NUM_LV_Pos)
#define SDADC_CFG1_CHOP2_NUM_LV         SDADC_CFG1_CHOP2_NUM_LV_Msk
#define SDADC_CFG1_CHOP3_NUM_LV_Pos     (22U)
#define SDADC_CFG1_CHOP3_NUM_LV_Msk     (0x1FFUL << SDADC_CFG1_CHOP3_NUM_LV_Pos)
#define SDADC_CFG1_CHOP3_NUM_LV         SDADC_CFG1_CHOP3_NUM_LV_Msk

/******************* Bit definition for SDADC_CFG2 register *******************/
#define SDADC_CFG2_SAMPLE_NUM_LV_Pos    (0U)
#define SDADC_CFG2_SAMPLE_NUM_LV_Msk    (0x1FFUL << SDADC_CFG2_SAMPLE_NUM_LV_Pos)
#define SDADC_CFG2_SAMPLE_NUM_LV        SDADC_CFG2_SAMPLE_NUM_LV_Msk
#define SDADC_CFG2_GAIN_NUME_LV_Pos     (9U)
#define SDADC_CFG2_GAIN_NUME_LV_Msk     (0x7UL << SDADC_CFG2_GAIN_NUME_LV_Pos)
#define SDADC_CFG2_GAIN_NUME_LV         SDADC_CFG2_GAIN_NUME_LV_Msk
#define SDADC_CFG2_GAIN_DENO_LV_Pos     (12U)
#define SDADC_CFG2_GAIN_DENO_LV_Msk     (0x7UL << SDADC_CFG2_GAIN_DENO_LV_Pos)
#define SDADC_CFG2_GAIN_DENO_LV         SDADC_CFG2_GAIN_DENO_LV_Msk
#define SDADC_CFG2_DEM_EN_LV_Pos        (15U)
#define SDADC_CFG2_DEM_EN_LV_Msk        (0x1UL << SDADC_CFG2_DEM_EN_LV_Pos)
#define SDADC_CFG2_DEM_EN_LV            SDADC_CFG2_DEM_EN_LV_Msk
#define SDADC_CFG2_SE_DIFF_SEL_LV_Pos   (16U)
#define SDADC_CFG2_SE_DIFF_SEL_LV_Msk   (0x1UL << SDADC_CFG2_SE_DIFF_SEL_LV_Pos)
#define SDADC_CFG2_SE_DIFF_SEL_LV       SDADC_CFG2_SE_DIFF_SEL_LV_Msk
#define SDADC_CFG2_CHOP_REF_NUM_LV_Pos  (17U)
#define SDADC_CFG2_CHOP_REF_NUM_LV_Msk  (0x1FFUL << SDADC_CFG2_CHOP_REF_NUM_LV_Pos)
#define SDADC_CFG2_CHOP_REF_NUM_LV      SDADC_CFG2_CHOP_REF_NUM_LV_Msk
#define SDADC_CFG2_FCHOP_REF_SEL_LV_Pos  (26U)
#define SDADC_CFG2_FCHOP_REF_SEL_LV_Msk  (0x3UL << SDADC_CFG2_FCHOP_REF_SEL_LV_Pos)
#define SDADC_CFG2_FCHOP_REF_SEL_LV     SDADC_CFG2_FCHOP_REF_SEL_LV_Msk

/******************* Bit definition for SDADC_CFG3 register *******************/
#define SDADC_CFG3_SEL_PCH_LV_Pos       (0U)
#define SDADC_CFG3_SEL_PCH_LV_Msk       (0x7UL << SDADC_CFG3_SEL_PCH_LV_Pos)
#define SDADC_CFG3_SEL_PCH_LV           SDADC_CFG3_SEL_PCH_LV_Msk
#define SDADC_CFG3_SEL_NCH_LV_Pos       (3U)
#define SDADC_CFG3_SEL_NCH_LV_Msk       (0x7UL << SDADC_CFG3_SEL_NCH_LV_Pos)
#define SDADC_CFG3_SEL_NCH_LV           SDADC_CFG3_SEL_NCH_LV_Msk
#define SDADC_CFG3_VREF_SEL_LV_Pos      (6U)
#define SDADC_CFG3_VREF_SEL_LV_Msk      (0x3UL << SDADC_CFG3_VREF_SEL_LV_Pos)
#define SDADC_CFG3_VREF_SEL_LV          SDADC_CFG3_VREF_SEL_LV_Msk
#define SDADC_CFG3_INT_VREF_SET_LV_Pos  (8U)
#define SDADC_CFG3_INT_VREF_SET_LV_Msk  (0x7UL << SDADC_CFG3_INT_VREF_SET_LV_Pos)
#define SDADC_CFG3_INT_VREF_SET_LV      SDADC_CFG3_INT_VREF_SET_LV_Msk
#define SDADC_CFG3_REFBUF_BP_LV_Pos     (11U)
#define SDADC_CFG3_REFBUF_BP_LV_Msk     (0x1UL << SDADC_CFG3_REFBUF_BP_LV_Pos)
#define SDADC_CFG3_REFBUF_BP_LV         SDADC_CFG3_REFBUF_BP_LV_Msk
#define SDADC_CFG3_AMP1_BM_LV_Pos       (12U)
#define SDADC_CFG3_AMP1_BM_LV_Msk       (0x7UL << SDADC_CFG3_AMP1_BM_LV_Pos)
#define SDADC_CFG3_AMP1_BM_LV           SDADC_CFG3_AMP1_BM_LV_Msk
#define SDADC_CFG3_REFBUF_BM_LV_Pos     (15U)
#define SDADC_CFG3_REFBUF_BM_LV_Msk     (0x7UL << SDADC_CFG3_REFBUF_BM_LV_Pos)
#define SDADC_CFG3_REFBUF_BM_LV         SDADC_CFG3_REFBUF_BM_LV_Msk
#define SDADC_CFG3_AMP2_BM_LV_Pos       (18U)
#define SDADC_CFG3_AMP2_BM_LV_Msk       (0x7UL << SDADC_CFG3_AMP2_BM_LV_Pos)
#define SDADC_CFG3_AMP2_BM_LV           SDADC_CFG3_AMP2_BM_LV_Msk
#define SDADC_CFG3_REFBUF_AZ_LV_Pos     (21U)
#define SDADC_CFG3_REFBUF_AZ_LV_Msk     (0x1UL << SDADC_CFG3_REFBUF_AZ_LV_Pos)
#define SDADC_CFG3_REFBUF_AZ_LV         SDADC_CFG3_REFBUF_AZ_LV_Msk
#define SDADC_CFG3_REFBUF_CHOP_MX_LV_Pos  (22U)
#define SDADC_CFG3_REFBUF_CHOP_MX_LV_Msk  (0x1UL << SDADC_CFG3_REFBUF_CHOP_MX_LV_Pos)
#define SDADC_CFG3_REFBUF_CHOP_MX_LV    SDADC_CFG3_REFBUF_CHOP_MX_LV_Msk

/******************* Bit definition for SDADC_CAL register ********************/
#define SDADC_CAL_OSCAL_EN_LV_Pos       (0U)
#define SDADC_CAL_OSCAL_EN_LV_Msk       (0x3UL << SDADC_CAL_OSCAL_EN_LV_Pos)
#define SDADC_CAL_OSCAL_EN_LV           SDADC_CAL_OSCAL_EN_LV_Msk
#define SDADC_CAL_OSCAL_RST_LV_Pos      (2U)
#define SDADC_CAL_OSCAL_RST_LV_Msk      (0x1UL << SDADC_CAL_OSCAL_RST_LV_Pos)
#define SDADC_CAL_OSCAL_RST_LV          SDADC_CAL_OSCAL_RST_LV_Msk
#define SDADC_CAL_OS_SET1_LV_Pos        (3U)
#define SDADC_CAL_OS_SET1_LV_Msk        (0xFUL << SDADC_CAL_OS_SET1_LV_Pos)
#define SDADC_CAL_OS_SET1_LV            SDADC_CAL_OS_SET1_LV_Msk
#define SDADC_CAL_OS_SET2_LV_Pos        (7U)
#define SDADC_CAL_OS_SET2_LV_Msk        (0xFUL << SDADC_CAL_OS_SET2_LV_Pos)
#define SDADC_CAL_OS_SET2_LV            SDADC_CAL_OS_SET2_LV_Msk
#define SDADC_CAL_OSCAL_RDY_LV_Pos      (11U)
#define SDADC_CAL_OSCAL_RDY_LV_Msk      (0x1UL << SDADC_CAL_OSCAL_RDY_LV_Pos)
#define SDADC_CAL_OSCAL_RDY_LV          SDADC_CAL_OSCAL_RDY_LV_Msk

/******************* Bit definition for SDADC_TRIG register *******************/
#define SDADC_TRIG_TIMER_TRIG_EN_Pos    (0U)
#define SDADC_TRIG_TIMER_TRIG_EN_Msk    (0x1UL << SDADC_TRIG_TIMER_TRIG_EN_Pos)
#define SDADC_TRIG_TIMER_TRIG_EN        SDADC_TRIG_TIMER_TRIG_EN_Msk
#define SDADC_TRIG_TIMER_TRIG_SRC_SEL_Pos  (1U)
#define SDADC_TRIG_TIMER_TRIG_SRC_SEL_Msk  (0x7UL << SDADC_TRIG_TIMER_TRIG_SRC_SEL_Pos)
#define SDADC_TRIG_TIMER_TRIG_SRC_SEL   SDADC_TRIG_TIMER_TRIG_SRC_SEL_Msk
#define SDADC_TRIG_GPIO_TRIG_EN_Pos     (4U)
#define SDADC_TRIG_GPIO_TRIG_EN_Msk     (0x1UL << SDADC_TRIG_GPIO_TRIG_EN_Pos)
#define SDADC_TRIG_GPIO_TRIG_EN         SDADC_TRIG_GPIO_TRIG_EN_Msk
#define SDADC_TRIG_ADC_START_Pos        (5U)
#define SDADC_TRIG_ADC_START_Msk        (0x1UL << SDADC_TRIG_ADC_START_Pos)
#define SDADC_TRIG_ADC_START            SDADC_TRIG_ADC_START_Msk

/***************** Bit definition for SDADC_CH0_CFG register ******************/
#define SDADC_CH0_CFG_EN_Pos            (0U)
#define SDADC_CH0_CFG_EN_Msk            (0x1UL << SDADC_CH0_CFG_EN_Pos)
#define SDADC_CH0_CFG_EN                SDADC_CH0_CFG_EN_Msk
#define SDADC_CH0_CFG_SEL_PCH_Pos       (1U)
#define SDADC_CH0_CFG_SEL_PCH_Msk       (0x7UL << SDADC_CH0_CFG_SEL_PCH_Pos)
#define SDADC_CH0_CFG_SEL_PCH           SDADC_CH0_CFG_SEL_PCH_Msk
#define SDADC_CH0_CFG_SEL_NCH_Pos       (4U)
#define SDADC_CH0_CFG_SEL_NCH_Msk       (0x7UL << SDADC_CH0_CFG_SEL_NCH_Pos)
#define SDADC_CH0_CFG_SEL_NCH           SDADC_CH0_CFG_SEL_NCH_Msk
#define SDADC_CH0_CFG_SHIFT_NUM_Pos     (7U)
#define SDADC_CH0_CFG_SHIFT_NUM_Msk     (0x3UL << SDADC_CH0_CFG_SHIFT_NUM_Pos)
#define SDADC_CH0_CFG_SHIFT_NUM         SDADC_CH0_CFG_SHIFT_NUM_Msk

/***************** Bit definition for SDADC_CH1_CFG register ******************/
#define SDADC_CH1_CFG_EN_Pos            (0U)
#define SDADC_CH1_CFG_EN_Msk            (0x1UL << SDADC_CH1_CFG_EN_Pos)
#define SDADC_CH1_CFG_EN                SDADC_CH1_CFG_EN_Msk
#define SDADC_CH1_CFG_SEL_PCH_Pos       (1U)
#define SDADC_CH1_CFG_SEL_PCH_Msk       (0x7UL << SDADC_CH1_CFG_SEL_PCH_Pos)
#define SDADC_CH1_CFG_SEL_PCH           SDADC_CH1_CFG_SEL_PCH_Msk
#define SDADC_CH1_CFG_SEL_NCH_Pos       (4U)
#define SDADC_CH1_CFG_SEL_NCH_Msk       (0x7UL << SDADC_CH1_CFG_SEL_NCH_Pos)
#define SDADC_CH1_CFG_SEL_NCH           SDADC_CH1_CFG_SEL_NCH_Msk
#define SDADC_CH1_CFG_SHIFT_NUM_Pos     (7U)
#define SDADC_CH1_CFG_SHIFT_NUM_Msk     (0x3UL << SDADC_CH1_CFG_SHIFT_NUM_Pos)
#define SDADC_CH1_CFG_SHIFT_NUM         SDADC_CH1_CFG_SHIFT_NUM_Msk

/***************** Bit definition for SDADC_CH2_CFG register ******************/
#define SDADC_CH2_CFG_EN_Pos            (0U)
#define SDADC_CH2_CFG_EN_Msk            (0x1UL << SDADC_CH2_CFG_EN_Pos)
#define SDADC_CH2_CFG_EN                SDADC_CH2_CFG_EN_Msk
#define SDADC_CH2_CFG_SEL_PCH_Pos       (1U)
#define SDADC_CH2_CFG_SEL_PCH_Msk       (0x7UL << SDADC_CH2_CFG_SEL_PCH_Pos)
#define SDADC_CH2_CFG_SEL_PCH           SDADC_CH2_CFG_SEL_PCH_Msk
#define SDADC_CH2_CFG_SEL_NCH_Pos       (4U)
#define SDADC_CH2_CFG_SEL_NCH_Msk       (0x7UL << SDADC_CH2_CFG_SEL_NCH_Pos)
#define SDADC_CH2_CFG_SEL_NCH           SDADC_CH2_CFG_SEL_NCH_Msk
#define SDADC_CH2_CFG_SHIFT_NUM_Pos     (7U)
#define SDADC_CH2_CFG_SHIFT_NUM_Msk     (0x3UL << SDADC_CH2_CFG_SHIFT_NUM_Pos)
#define SDADC_CH2_CFG_SHIFT_NUM         SDADC_CH2_CFG_SHIFT_NUM_Msk

/***************** Bit definition for SDADC_CH3_CFG register ******************/
#define SDADC_CH3_CFG_EN_Pos            (0U)
#define SDADC_CH3_CFG_EN_Msk            (0x1UL << SDADC_CH3_CFG_EN_Pos)
#define SDADC_CH3_CFG_EN                SDADC_CH3_CFG_EN_Msk
#define SDADC_CH3_CFG_SEL_PCH_Pos       (1U)
#define SDADC_CH3_CFG_SEL_PCH_Msk       (0x7UL << SDADC_CH3_CFG_SEL_PCH_Pos)
#define SDADC_CH3_CFG_SEL_PCH           SDADC_CH3_CFG_SEL_PCH_Msk
#define SDADC_CH3_CFG_SEL_NCH_Pos       (4U)
#define SDADC_CH3_CFG_SEL_NCH_Msk       (0x7UL << SDADC_CH3_CFG_SEL_NCH_Pos)
#define SDADC_CH3_CFG_SEL_NCH           SDADC_CH3_CFG_SEL_NCH_Msk
#define SDADC_CH3_CFG_SHIFT_NUM_Pos     (7U)
#define SDADC_CH3_CFG_SHIFT_NUM_Msk     (0x3UL << SDADC_CH3_CFG_SHIFT_NUM_Pos)
#define SDADC_CH3_CFG_SHIFT_NUM         SDADC_CH3_CFG_SHIFT_NUM_Msk

/***************** Bit definition for SDADC_CH4_CFG register ******************/
#define SDADC_CH4_CFG_EN_Pos            (0U)
#define SDADC_CH4_CFG_EN_Msk            (0x1UL << SDADC_CH4_CFG_EN_Pos)
#define SDADC_CH4_CFG_EN                SDADC_CH4_CFG_EN_Msk
#define SDADC_CH4_CFG_SEL_PCH_Pos       (1U)
#define SDADC_CH4_CFG_SEL_PCH_Msk       (0x7UL << SDADC_CH4_CFG_SEL_PCH_Pos)
#define SDADC_CH4_CFG_SEL_PCH           SDADC_CH4_CFG_SEL_PCH_Msk
#define SDADC_CH4_CFG_SEL_NCH_Pos       (4U)
#define SDADC_CH4_CFG_SEL_NCH_Msk       (0x7UL << SDADC_CH4_CFG_SEL_NCH_Pos)
#define SDADC_CH4_CFG_SEL_NCH           SDADC_CH4_CFG_SEL_NCH_Msk
#define SDADC_CH4_CFG_SHIFT_NUM_Pos     (7U)
#define SDADC_CH4_CFG_SHIFT_NUM_Msk     (0x3UL << SDADC_CH4_CFG_SHIFT_NUM_Pos)
#define SDADC_CH4_CFG_SHIFT_NUM         SDADC_CH4_CFG_SHIFT_NUM_Msk

/***************** Bit definition for SDADC_CH0_DOUT register *****************/
#define SDADC_CH0_DOUT_DATA_Pos         (0U)
#define SDADC_CH0_DOUT_DATA_Msk         (0xFFFFFFUL << SDADC_CH0_DOUT_DATA_Pos)
#define SDADC_CH0_DOUT_DATA             SDADC_CH0_DOUT_DATA_Msk

/***************** Bit definition for SDADC_CH1_DOUT register *****************/
#define SDADC_CH1_DOUT_DATA_Pos         (0U)
#define SDADC_CH1_DOUT_DATA_Msk         (0xFFFFFFUL << SDADC_CH1_DOUT_DATA_Pos)
#define SDADC_CH1_DOUT_DATA             SDADC_CH1_DOUT_DATA_Msk

/***************** Bit definition for SDADC_CH2_DOUT register *****************/
#define SDADC_CH2_DOUT_DATA_Pos         (0U)
#define SDADC_CH2_DOUT_DATA_Msk         (0xFFFFFFUL << SDADC_CH2_DOUT_DATA_Pos)
#define SDADC_CH2_DOUT_DATA             SDADC_CH2_DOUT_DATA_Msk

/***************** Bit definition for SDADC_CH3_DOUT register *****************/
#define SDADC_CH3_DOUT_DATA_Pos         (0U)
#define SDADC_CH3_DOUT_DATA_Msk         (0xFFFFFFUL << SDADC_CH3_DOUT_DATA_Pos)
#define SDADC_CH3_DOUT_DATA             SDADC_CH3_DOUT_DATA_Msk

/***************** Bit definition for SDADC_CH4_DOUT register *****************/
#define SDADC_CH4_DOUT_DATA_Pos         (0U)
#define SDADC_CH4_DOUT_DATA_Msk         (0xFFFFFFUL << SDADC_CH4_DOUT_DATA_Pos)
#define SDADC_CH4_DOUT_DATA             SDADC_CH4_DOUT_DATA_Msk

/*************** Bit definition for SDADC_SINGLE_DOUT register ****************/
#define SDADC_SINGLE_DOUT_DATA_Pos      (0U)
#define SDADC_SINGLE_DOUT_DATA_Msk      (0xFFFFFFUL << SDADC_SINGLE_DOUT_DATA_Pos)
#define SDADC_SINGLE_DOUT_DATA          SDADC_SINGLE_DOUT_DATA_Msk

/***************** Bit definition for SDADC_SINC_CFG register *****************/
#define SDADC_SINC_CFG_SINC_RATE_Pos    (0U)
#define SDADC_SINC_CFG_SINC_RATE_Msk    (0xFFUL << SDADC_SINC_CFG_SINC_RATE_Pos)
#define SDADC_SINC_CFG_SINC_RATE        SDADC_SINC_CFG_SINC_RATE_Msk
#define SDADC_SINC_CFG_SINC_ORDER_SEL_Pos  (8U)
#define SDADC_SINC_CFG_SINC_ORDER_SEL_Msk  (0x1UL << SDADC_SINC_CFG_SINC_ORDER_SEL_Pos)
#define SDADC_SINC_CFG_SINC_ORDER_SEL   SDADC_SINC_CFG_SINC_ORDER_SEL_Msk

/**************** Bit definition for SDADC_COMP_CFG0 register *****************/
#define SDADC_COMP_CFG0_COMP_BYPASS_Pos  (0U)
#define SDADC_COMP_CFG0_COMP_BYPASS_Msk  (0x1UL << SDADC_COMP_CFG0_COMP_BYPASS_Pos)
#define SDADC_COMP_CFG0_COMP_BYPASS     SDADC_COMP_CFG0_COMP_BYPASS_Msk
#define SDADC_COMP_CFG0_COMP_COEFF0_Pos  (1U)
#define SDADC_COMP_CFG0_COMP_COEFF0_Msk  (0xFFFUL << SDADC_COMP_CFG0_COMP_COEFF0_Pos)
#define SDADC_COMP_CFG0_COMP_COEFF0     SDADC_COMP_CFG0_COMP_COEFF0_Msk

/**************** Bit definition for SDADC_COMP_CFG1 register *****************/
#define SDADC_COMP_CFG1_COMP_COEFF1_Pos  (0U)
#define SDADC_COMP_CFG1_COMP_COEFF1_Msk  (0xFFFUL << SDADC_COMP_CFG1_COMP_COEFF1_Pos)
#define SDADC_COMP_CFG1_COMP_COEFF1     SDADC_COMP_CFG1_COMP_COEFF1_Msk
#define SDADC_COMP_CFG1_COMP_COEFF2_Pos  (12U)
#define SDADC_COMP_CFG1_COMP_COEFF2_Msk  (0xFFFUL << SDADC_COMP_CFG1_COMP_COEFF2_Pos)
#define SDADC_COMP_CFG1_COMP_COEFF2     SDADC_COMP_CFG1_COMP_COEFF2_Msk

/***************** Bit definition for SDADC_LPF_CFG6 register *****************/
#define SDADC_LPF_CFG6_LPF_DS_Pos       (12U)
#define SDADC_LPF_CFG6_LPF_DS_Msk       (0x1UL << SDADC_LPF_CFG6_LPF_DS_Pos)
#define SDADC_LPF_CFG6_LPF_DS           SDADC_LPF_CFG6_LPF_DS_Msk
#define SDADC_LPF_CFG6_LPF_BYPASS_Pos   (13U)
#define SDADC_LPF_CFG6_LPF_BYPASS_Msk   (0x1UL << SDADC_LPF_CFG6_LPF_BYPASS_Pos)
#define SDADC_LPF_CFG6_LPF_BYPASS       SDADC_LPF_CFG6_LPF_BYPASS_Msk

/***************** Bit definition for SDADC_DMA_CFG register ******************/
#define SDADC_DMA_CFG_RX_DMA_MSK_Pos    (0U)
#define SDADC_DMA_CFG_RX_DMA_MSK_Msk    (0x1UL << SDADC_DMA_CFG_RX_DMA_MSK_Pos)
#define SDADC_DMA_CFG_RX_DMA_MSK        SDADC_DMA_CFG_RX_DMA_MSK_Msk

/***************** Bit definition for SDADC_ADC_DATA register *****************/
#define SDADC_ADC_DATA_DMA_ENTRY_Pos    (0U)
#define SDADC_ADC_DATA_DMA_ENTRY_Msk    (0xFFFFFFFFUL << SDADC_ADC_DATA_DMA_ENTRY_Pos)
#define SDADC_ADC_DATA_DMA_ENTRY        SDADC_ADC_DATA_DMA_ENTRY_Msk

/***************** Bit definition for SDADC_FIFO_ST register ******************/
#define SDADC_FIFO_ST_EMPTY_Pos         (0U)
#define SDADC_FIFO_ST_EMPTY_Msk         (0x1UL << SDADC_FIFO_ST_EMPTY_Pos)
#define SDADC_FIFO_ST_EMPTY             SDADC_FIFO_ST_EMPTY_Msk

/****************** Bit definition for SDADC_INT_ST register ******************/
#define SDADC_INT_ST_OVERFLOW_Pos       (0U)
#define SDADC_INT_ST_OVERFLOW_Msk       (0x1UL << SDADC_INT_ST_OVERFLOW_Pos)
#define SDADC_INT_ST_OVERFLOW           SDADC_INT_ST_OVERFLOW_Msk
#define SDADC_INT_ST_DSAMPLE_Pos        (1U)
#define SDADC_INT_ST_DSAMPLE_Msk        (0x1UL << SDADC_INT_ST_DSAMPLE_Pos)
#define SDADC_INT_ST_DSAMPLE            SDADC_INT_ST_DSAMPLE_Msk

/***************** Bit definition for SDADC_INT_MSK register ******************/
#define SDADC_INT_MSK_OVERFLOW_Pos      (0U)
#define SDADC_INT_MSK_OVERFLOW_Msk      (0x1UL << SDADC_INT_MSK_OVERFLOW_Pos)
#define SDADC_INT_MSK_OVERFLOW          SDADC_INT_MSK_OVERFLOW_Msk
#define SDADC_INT_MSK_DSAMPLE_Pos       (1U)
#define SDADC_INT_MSK_DSAMPLE_Msk       (0x1UL << SDADC_INT_MSK_DSAMPLE_Pos)
#define SDADC_INT_MSK_DSAMPLE           SDADC_INT_MSK_DSAMPLE_Msk

/***************** Bit definition for SDADC_INT_CLR register ******************/
#define SDADC_INT_CLR_INT_CLR_Pos       (0U)
#define SDADC_INT_CLR_INT_CLR_Msk       (0x1UL << SDADC_INT_CLR_INT_CLR_Pos)
#define SDADC_INT_CLR_INT_CLR           SDADC_INT_CLR_INT_CLR_Msk

/****************** Bit definition for SDADC_FSM_ST register ******************/
#define SDADC_FSM_ST_ACTIVE_Pos         (0U)
#define SDADC_FSM_ST_ACTIVE_Msk         (0x1UL << SDADC_FSM_ST_ACTIVE_Pos)
#define SDADC_FSM_ST_ACTIVE             SDADC_FSM_ST_ACTIVE_Msk

/******************* Bit definition for SDADC_RSVD register *******************/
#define SDADC_RSVD_RESERVE2_Pos         (0U)
#define SDADC_RSVD_RESERVE2_Msk         (0xFFUL << SDADC_RSVD_RESERVE2_Pos)
#define SDADC_RSVD_RESERVE2             SDADC_RSVD_RESERVE2_Msk
#define SDADC_RSVD_RESERVE1_Pos         (8U)
#define SDADC_RSVD_RESERVE1_Msk         (0xFFFFUL << SDADC_RSVD_RESERVE1_Pos)
#define SDADC_RSVD_RESERVE1             SDADC_RSVD_RESERVE1_Msk

#endif
