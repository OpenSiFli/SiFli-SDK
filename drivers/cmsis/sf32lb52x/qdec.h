#ifndef __QDEC_H
#define __QDEC_H

typedef struct
{
    __IO uint32_t QDEC_CTRL;
    __IO uint32_t SAMP_DIV;
    __IO uint32_t TRIG_NUM;
    __IO uint32_t DBF;
    __IO uint32_t INT_EN;
    __IO uint32_t INT_STA;
    __IO uint32_t QDEC_CLR;
    __IO uint32_t SAMP_VAL;
    __IO uint32_t ACC;
    __IO uint32_t ACCDBT;
    __IO uint32_t ACCZ;
} QDEC_TypeDef;


/***************** Bit definition for QDEC_QDEC_CTRL register *****************/
#define QDEC_QDEC_CTRL_QDEC_CTRL_Pos    (0U)
#define QDEC_QDEC_CTRL_QDEC_CTRL_Msk    (0x1UL << QDEC_QDEC_CTRL_QDEC_CTRL_Pos)
#define QDEC_QDEC_CTRL_QDEC_CTRL        QDEC_QDEC_CTRL_QDEC_CTRL_Msk

/***************** Bit definition for QDEC_SAMP_DIV register ******************/
#define QDEC_SAMP_DIV_SAMP_DIV_Pos      (0U)
#define QDEC_SAMP_DIV_SAMP_DIV_Msk      (0xFFFFFFUL << QDEC_SAMP_DIV_SAMP_DIV_Pos)
#define QDEC_SAMP_DIV_SAMP_DIV          QDEC_SAMP_DIV_SAMP_DIV_Msk

/***************** Bit definition for QDEC_TRIG_NUM register ******************/
#define QDEC_TRIG_NUM_TRIG_NUM_Pos      (0U)
#define QDEC_TRIG_NUM_TRIG_NUM_Msk      (0xFFFFUL << QDEC_TRIG_NUM_TRIG_NUM_Pos)
#define QDEC_TRIG_NUM_TRIG_NUM          QDEC_TRIG_NUM_TRIG_NUM_Msk

/******************** Bit definition for QDEC_DBF register ********************/
#define QDEC_DBF_DBF_TIM_Pos            (0U)
#define QDEC_DBF_DBF_TIM_Msk            (0xFFFFFFUL << QDEC_DBF_DBF_TIM_Pos)
#define QDEC_DBF_DBF_TIM                QDEC_DBF_DBF_TIM_Msk
#define QDEC_DBF_DBF_EN_Pos             (24U)
#define QDEC_DBF_DBF_EN_Msk             (0x1UL << QDEC_DBF_DBF_EN_Pos)
#define QDEC_DBF_DBF_EN                 QDEC_DBF_DBF_EN_Msk

/****************** Bit definition for QDEC_INT_EN register *******************/
#define QDEC_INT_EN_SAMP_EN_Pos         (0U)
#define QDEC_INT_EN_SAMP_EN_Msk         (0x1UL << QDEC_INT_EN_SAMP_EN_Pos)
#define QDEC_INT_EN_SAMP_EN             QDEC_INT_EN_SAMP_EN_Msk
#define QDEC_INT_EN_TRIG_EN_Pos         (1U)
#define QDEC_INT_EN_TRIG_EN_Msk         (0x1UL << QDEC_INT_EN_TRIG_EN_Pos)
#define QDEC_INT_EN_TRIG_EN             QDEC_INT_EN_TRIG_EN_Msk
#define QDEC_INT_EN_ACCOF_EN_Pos        (2U)
#define QDEC_INT_EN_ACCOF_EN_Msk        (0x1UL << QDEC_INT_EN_ACCOF_EN_Pos)
#define QDEC_INT_EN_ACCOF_EN            QDEC_INT_EN_ACCOF_EN_Msk
#define QDEC_INT_EN_DBT_EN_Pos          (3U)
#define QDEC_INT_EN_DBT_EN_Msk          (0x1UL << QDEC_INT_EN_DBT_EN_Pos)
#define QDEC_INT_EN_DBT_EN              QDEC_INT_EN_DBT_EN_Msk
#define QDEC_INT_EN_Z_EN_Pos            (4U)
#define QDEC_INT_EN_Z_EN_Msk            (0x1UL << QDEC_INT_EN_Z_EN_Pos)
#define QDEC_INT_EN_Z_EN                QDEC_INT_EN_Z_EN_Msk

/****************** Bit definition for QDEC_INT_STA register ******************/
#define QDEC_INT_STA_SAMP_STA_Pos       (0U)
#define QDEC_INT_STA_SAMP_STA_Msk       (0x1UL << QDEC_INT_STA_SAMP_STA_Pos)
#define QDEC_INT_STA_SAMP_STA           QDEC_INT_STA_SAMP_STA_Msk
#define QDEC_INT_STA_TRIG_STA_Pos       (1U)
#define QDEC_INT_STA_TRIG_STA_Msk       (0x1UL << QDEC_INT_STA_TRIG_STA_Pos)
#define QDEC_INT_STA_TRIG_STA           QDEC_INT_STA_TRIG_STA_Msk
#define QDEC_INT_STA_ACCOF_STA_Pos      (2U)
#define QDEC_INT_STA_ACCOF_STA_Msk      (0x1UL << QDEC_INT_STA_ACCOF_STA_Pos)
#define QDEC_INT_STA_ACCOF_STA          QDEC_INT_STA_ACCOF_STA_Msk
#define QDEC_INT_STA_DBT_STA_Pos        (3U)
#define QDEC_INT_STA_DBT_STA_Msk        (0x1UL << QDEC_INT_STA_DBT_STA_Pos)
#define QDEC_INT_STA_DBT_STA            QDEC_INT_STA_DBT_STA_Msk
#define QDEC_INT_STA_Z_STA_Pos          (4U)
#define QDEC_INT_STA_Z_STA_Msk          (0x1UL << QDEC_INT_STA_Z_STA_Pos)
#define QDEC_INT_STA_Z_STA              QDEC_INT_STA_Z_STA_Msk

/***************** Bit definition for QDEC_QDEC_CLR register ******************/
#define QDEC_QDEC_CLR_QEDC_CLR_Pos      (0U)
#define QDEC_QDEC_CLR_QEDC_CLR_Msk      (0x1UL << QDEC_QDEC_CLR_QEDC_CLR_Pos)
#define QDEC_QDEC_CLR_QEDC_CLR          QDEC_QDEC_CLR_QEDC_CLR_Msk
#define QDEC_QDEC_CLR_ACC_CLR_Pos       (1U)
#define QDEC_QDEC_CLR_ACC_CLR_Msk       (0x1UL << QDEC_QDEC_CLR_ACC_CLR_Pos)
#define QDEC_QDEC_CLR_ACC_CLR           QDEC_QDEC_CLR_ACC_CLR_Msk
#define QDEC_QDEC_CLR_ACCDBT_CLR_Pos    (2U)
#define QDEC_QDEC_CLR_ACCDBT_CLR_Msk    (0x1UL << QDEC_QDEC_CLR_ACCDBT_CLR_Pos)
#define QDEC_QDEC_CLR_ACCDBT_CLR        QDEC_QDEC_CLR_ACCDBT_CLR_Msk
#define QDEC_QDEC_CLR_ACCZ_CLR_Pos      (3U)
#define QDEC_QDEC_CLR_ACCZ_CLR_Msk      (0x1UL << QDEC_QDEC_CLR_ACCZ_CLR_Pos)
#define QDEC_QDEC_CLR_ACCZ_CLR          QDEC_QDEC_CLR_ACCZ_CLR_Msk

/***************** Bit definition for QDEC_SAMP_VAL register ******************/
#define QDEC_SAMP_VAL_SAMP_VAL_Pos      (0U)
#define QDEC_SAMP_VAL_SAMP_VAL_Msk      (0x3UL << QDEC_SAMP_VAL_SAMP_VAL_Pos)
#define QDEC_SAMP_VAL_SAMP_VAL          QDEC_SAMP_VAL_SAMP_VAL_Msk

/******************** Bit definition for QDEC_ACC register ********************/
#define QDEC_ACC_ACC_Pos                (0U)
#define QDEC_ACC_ACC_Msk                (0xFFFFUL << QDEC_ACC_ACC_Pos)
#define QDEC_ACC_ACC                    QDEC_ACC_ACC_Msk

/****************** Bit definition for QDEC_ACCDBT register *******************/
#define QDEC_ACCDBT_ACCDBT_Pos          (0U)
#define QDEC_ACCDBT_ACCDBT_Msk          (0xFFFFUL << QDEC_ACCDBT_ACCDBT_Pos)
#define QDEC_ACCDBT_ACCDBT              QDEC_ACCDBT_ACCDBT_Msk

/******************* Bit definition for QDEC_ACCZ register ********************/
#define QDEC_ACCZ_ACCZ_Pos              (0U)
#define QDEC_ACCZ_ACCZ_Msk              (0xFFFFUL << QDEC_ACCZ_ACCZ_Pos)
#define QDEC_ACCZ_ACCZ                  QDEC_ACCZ_ACCZ_Msk

#endif
