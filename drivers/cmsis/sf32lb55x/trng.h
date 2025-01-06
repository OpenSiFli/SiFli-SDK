#ifndef __TRNG_H
#define __TRNG_H

typedef struct
{
    __IO uint32_t CTRL;
    __IO uint32_t STAT;
    __IO uint32_t CFG;
    __IO uint32_t IRQ;
    __IO uint32_t RAND_SEED0;
    __IO uint32_t RAND_SEED1;
    __IO uint32_t RAND_SEED2;
    __IO uint32_t RAND_SEED3;
    __IO uint32_t RAND_SEED4;
    __IO uint32_t RAND_SEED5;
    __IO uint32_t RAND_SEED6;
    __IO uint32_t RAND_SEED7;
    __IO uint32_t RAND_NUM0;
    __IO uint32_t RAND_NUM1;
    __IO uint32_t RAND_NUM2;
    __IO uint32_t RAND_NUM3;
    __IO uint32_t RAND_NUM4;
    __IO uint32_t RAND_NUM5;
    __IO uint32_t RAND_NUM6;
    __IO uint32_t RAND_NUM7;
} TRNG_TypeDef;


/******************* Bit definition for TRNG_CTRL register ********************/
#define TRNG_CTRL_GEN_SEED_START_Pos    (0U)
#define TRNG_CTRL_GEN_SEED_START_Msk    (0x1UL << TRNG_CTRL_GEN_SEED_START_Pos)
#define TRNG_CTRL_GEN_SEED_START        TRNG_CTRL_GEN_SEED_START_Msk
#define TRNG_CTRL_GEN_RAND_NUM_START_Pos  (1U)
#define TRNG_CTRL_GEN_RAND_NUM_START_Msk  (0x1UL << TRNG_CTRL_GEN_RAND_NUM_START_Pos)
#define TRNG_CTRL_GEN_RAND_NUM_START    TRNG_CTRL_GEN_RAND_NUM_START_Msk
#define TRNG_CTRL_GEN_SEED_STOP_Pos     (2U)
#define TRNG_CTRL_GEN_SEED_STOP_Msk     (0x1UL << TRNG_CTRL_GEN_SEED_STOP_Pos)
#define TRNG_CTRL_GEN_SEED_STOP         TRNG_CTRL_GEN_SEED_STOP_Msk
#define TRNG_CTRL_GEN_RAND_NUM_STOP_Pos  (3U)
#define TRNG_CTRL_GEN_RAND_NUM_STOP_Msk  (0x1UL << TRNG_CTRL_GEN_RAND_NUM_STOP_Pos)
#define TRNG_CTRL_GEN_RAND_NUM_STOP     TRNG_CTRL_GEN_RAND_NUM_STOP_Msk
#define TRNG_CTRL_GEN_RAND_NUM_SUSPEND_Pos  (4U)
#define TRNG_CTRL_GEN_RAND_NUM_SUSPEND_Msk  (0x1UL << TRNG_CTRL_GEN_RAND_NUM_SUSPEND_Pos)
#define TRNG_CTRL_GEN_RAND_NUM_SUSPEND  TRNG_CTRL_GEN_RAND_NUM_SUSPEND_Msk

/******************* Bit definition for TRNG_STAT register ********************/
#define TRNG_STAT_SEED_GEN_BUSY_Pos     (0U)
#define TRNG_STAT_SEED_GEN_BUSY_Msk     (0x1UL << TRNG_STAT_SEED_GEN_BUSY_Pos)
#define TRNG_STAT_SEED_GEN_BUSY         TRNG_STAT_SEED_GEN_BUSY_Msk
#define TRNG_STAT_SEED_VALID_Pos        (1U)
#define TRNG_STAT_SEED_VALID_Msk        (0x1UL << TRNG_STAT_SEED_VALID_Pos)
#define TRNG_STAT_SEED_VALID            TRNG_STAT_SEED_VALID_Msk
#define TRNG_STAT_RAND_NUM_GEN_BUSY_Pos  (2U)
#define TRNG_STAT_RAND_NUM_GEN_BUSY_Msk  (0x1UL << TRNG_STAT_RAND_NUM_GEN_BUSY_Pos)
#define TRNG_STAT_RAND_NUM_GEN_BUSY     TRNG_STAT_RAND_NUM_GEN_BUSY_Msk
#define TRNG_STAT_RAND_NUM_VALID_Pos    (3U)
#define TRNG_STAT_RAND_NUM_VALID_Msk    (0x1UL << TRNG_STAT_RAND_NUM_VALID_Pos)
#define TRNG_STAT_RAND_NUM_VALID        TRNG_STAT_RAND_NUM_VALID_Msk

/******************** Bit definition for TRNG_CFG register ********************/
#define TRNG_CFG_AUTO_CLOCK_ENABLE_Pos  (0U)
#define TRNG_CFG_AUTO_CLOCK_ENABLE_Msk  (0x1UL << TRNG_CFG_AUTO_CLOCK_ENABLE_Pos)
#define TRNG_CFG_AUTO_CLOCK_ENABLE      TRNG_CFG_AUTO_CLOCK_ENABLE_Msk
#define TRNG_CFG_USE_EXT_SEED_Pos       (1U)
#define TRNG_CFG_USE_EXT_SEED_Msk       (0x1UL << TRNG_CFG_USE_EXT_SEED_Pos)
#define TRNG_CFG_USE_EXT_SEED           TRNG_CFG_USE_EXT_SEED_Msk
#define TRNG_CFG_REJECT_THRESHOLD_Pos   (8U)
#define TRNG_CFG_REJECT_THRESHOLD_Msk   (0xFFUL << TRNG_CFG_REJECT_THRESHOLD_Pos)
#define TRNG_CFG_REJECT_THRESHOLD       TRNG_CFG_REJECT_THRESHOLD_Msk

/******************** Bit definition for TRNG_IRQ register ********************/
#define TRNG_IRQ_SEED_GEN_DONE_Pos      (0U)
#define TRNG_IRQ_SEED_GEN_DONE_Msk      (0x1UL << TRNG_IRQ_SEED_GEN_DONE_Pos)
#define TRNG_IRQ_SEED_GEN_DONE          TRNG_IRQ_SEED_GEN_DONE_Msk
#define TRNG_IRQ_RAND_NUM_AVAIL_Pos     (1U)
#define TRNG_IRQ_RAND_NUM_AVAIL_Msk     (0x1UL << TRNG_IRQ_RAND_NUM_AVAIL_Pos)
#define TRNG_IRQ_RAND_NUM_AVAIL         TRNG_IRQ_RAND_NUM_AVAIL_Msk
#define TRNG_IRQ_PRNG_LOCKUP_Pos        (2U)
#define TRNG_IRQ_PRNG_LOCKUP_Msk        (0x1UL << TRNG_IRQ_PRNG_LOCKUP_Pos)
#define TRNG_IRQ_PRNG_LOCKUP            TRNG_IRQ_PRNG_LOCKUP_Msk
#define TRNG_IRQ_SEED_GEN_DONE_MSK_Pos  (16U)
#define TRNG_IRQ_SEED_GEN_DONE_MSK_Msk  (0x1UL << TRNG_IRQ_SEED_GEN_DONE_MSK_Pos)
#define TRNG_IRQ_SEED_GEN_DONE_MSK      TRNG_IRQ_SEED_GEN_DONE_MSK_Msk
#define TRNG_IRQ_RAND_NUM_AVAIL_MSK_Pos  (17U)
#define TRNG_IRQ_RAND_NUM_AVAIL_MSK_Msk  (0x1UL << TRNG_IRQ_RAND_NUM_AVAIL_MSK_Pos)
#define TRNG_IRQ_RAND_NUM_AVAIL_MSK     TRNG_IRQ_RAND_NUM_AVAIL_MSK_Msk
#define TRNG_IRQ_PRNG_LOCKUP_MSK_Pos    (18U)
#define TRNG_IRQ_PRNG_LOCKUP_MSK_Msk    (0x1UL << TRNG_IRQ_PRNG_LOCKUP_MSK_Pos)
#define TRNG_IRQ_PRNG_LOCKUP_MSK        TRNG_IRQ_PRNG_LOCKUP_MSK_Msk

/**************** Bit definition for TRNG_RAND_SEED0 register *****************/
#define TRNG_RAND_SEED0_VAL_Pos         (0U)
#define TRNG_RAND_SEED0_VAL_Msk         (0xFFFFFFFFUL << TRNG_RAND_SEED0_VAL_Pos)
#define TRNG_RAND_SEED0_VAL             TRNG_RAND_SEED0_VAL_Msk

/**************** Bit definition for TRNG_RAND_SEED1 register *****************/
#define TRNG_RAND_SEED1_VAL_Pos         (0U)
#define TRNG_RAND_SEED1_VAL_Msk         (0xFFFFFFFFUL << TRNG_RAND_SEED1_VAL_Pos)
#define TRNG_RAND_SEED1_VAL             TRNG_RAND_SEED1_VAL_Msk

/**************** Bit definition for TRNG_RAND_SEED2 register *****************/
#define TRNG_RAND_SEED2_VAL_Pos         (0U)
#define TRNG_RAND_SEED2_VAL_Msk         (0xFFFFFFFFUL << TRNG_RAND_SEED2_VAL_Pos)
#define TRNG_RAND_SEED2_VAL             TRNG_RAND_SEED2_VAL_Msk

/**************** Bit definition for TRNG_RAND_SEED3 register *****************/
#define TRNG_RAND_SEED3_VAL_Pos         (0U)
#define TRNG_RAND_SEED3_VAL_Msk         (0xFFFFFFFFUL << TRNG_RAND_SEED3_VAL_Pos)
#define TRNG_RAND_SEED3_VAL             TRNG_RAND_SEED3_VAL_Msk

/**************** Bit definition for TRNG_RAND_SEED4 register *****************/
#define TRNG_RAND_SEED4_VAL_Pos         (0U)
#define TRNG_RAND_SEED4_VAL_Msk         (0xFFFFFFFFUL << TRNG_RAND_SEED4_VAL_Pos)
#define TRNG_RAND_SEED4_VAL             TRNG_RAND_SEED4_VAL_Msk

/**************** Bit definition for TRNG_RAND_SEED5 register *****************/
#define TRNG_RAND_SEED5_VAL_Pos         (0U)
#define TRNG_RAND_SEED5_VAL_Msk         (0xFFFFFFFFUL << TRNG_RAND_SEED5_VAL_Pos)
#define TRNG_RAND_SEED5_VAL             TRNG_RAND_SEED5_VAL_Msk

/**************** Bit definition for TRNG_RAND_SEED6 register *****************/
#define TRNG_RAND_SEED6_VAL_Pos         (0U)
#define TRNG_RAND_SEED6_VAL_Msk         (0xFFFFFFFFUL << TRNG_RAND_SEED6_VAL_Pos)
#define TRNG_RAND_SEED6_VAL             TRNG_RAND_SEED6_VAL_Msk

/**************** Bit definition for TRNG_RAND_SEED7 register *****************/
#define TRNG_RAND_SEED7_VAL_Pos         (0U)
#define TRNG_RAND_SEED7_VAL_Msk         (0xFFFFFFFFUL << TRNG_RAND_SEED7_VAL_Pos)
#define TRNG_RAND_SEED7_VAL             TRNG_RAND_SEED7_VAL_Msk

/***************** Bit definition for TRNG_RAND_NUM0 register *****************/
#define TRNG_RAND_NUM0_VAL_Pos          (0U)
#define TRNG_RAND_NUM0_VAL_Msk          (0xFFFFFFFFUL << TRNG_RAND_NUM0_VAL_Pos)
#define TRNG_RAND_NUM0_VAL              TRNG_RAND_NUM0_VAL_Msk

/***************** Bit definition for TRNG_RAND_NUM1 register *****************/
#define TRNG_RAND_NUM1_VAL_Pos          (0U)
#define TRNG_RAND_NUM1_VAL_Msk          (0xFFFFFFFFUL << TRNG_RAND_NUM1_VAL_Pos)
#define TRNG_RAND_NUM1_VAL              TRNG_RAND_NUM1_VAL_Msk

/***************** Bit definition for TRNG_RAND_NUM2 register *****************/
#define TRNG_RAND_NUM2_VAL_Pos          (0U)
#define TRNG_RAND_NUM2_VAL_Msk          (0xFFFFFFFFUL << TRNG_RAND_NUM2_VAL_Pos)
#define TRNG_RAND_NUM2_VAL              TRNG_RAND_NUM2_VAL_Msk

/***************** Bit definition for TRNG_RAND_NUM3 register *****************/
#define TRNG_RAND_NUM3_VAL_Pos          (0U)
#define TRNG_RAND_NUM3_VAL_Msk          (0xFFFFFFFFUL << TRNG_RAND_NUM3_VAL_Pos)
#define TRNG_RAND_NUM3_VAL              TRNG_RAND_NUM3_VAL_Msk

/***************** Bit definition for TRNG_RAND_NUM4 register *****************/
#define TRNG_RAND_NUM4_VAL_Pos          (0U)
#define TRNG_RAND_NUM4_VAL_Msk          (0xFFFFFFFFUL << TRNG_RAND_NUM4_VAL_Pos)
#define TRNG_RAND_NUM4_VAL              TRNG_RAND_NUM4_VAL_Msk

/***************** Bit definition for TRNG_RAND_NUM5 register *****************/
#define TRNG_RAND_NUM5_VAL_Pos          (0U)
#define TRNG_RAND_NUM5_VAL_Msk          (0xFFFFFFFFUL << TRNG_RAND_NUM5_VAL_Pos)
#define TRNG_RAND_NUM5_VAL              TRNG_RAND_NUM5_VAL_Msk

/***************** Bit definition for TRNG_RAND_NUM6 register *****************/
#define TRNG_RAND_NUM6_VAL_Pos          (0U)
#define TRNG_RAND_NUM6_VAL_Msk          (0xFFFFFFFFUL << TRNG_RAND_NUM6_VAL_Pos)
#define TRNG_RAND_NUM6_VAL              TRNG_RAND_NUM6_VAL_Msk

/***************** Bit definition for TRNG_RAND_NUM7 register *****************/
#define TRNG_RAND_NUM7_VAL_Pos          (0U)
#define TRNG_RAND_NUM7_VAL_Msk          (0xFFFFFFFFUL << TRNG_RAND_NUM7_VAL_Pos)
#define TRNG_RAND_NUM7_VAL              TRNG_RAND_NUM7_VAL_Msk

#endif
