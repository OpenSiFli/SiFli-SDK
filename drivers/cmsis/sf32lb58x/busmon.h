#ifndef __BUSMON_H
#define __BUSMON_H

typedef struct
{
    __IO uint32_t CR1;
    __IO uint32_t CNT1;
    __IO uint32_t MIN1;
    __IO uint32_t MAX1;
    __IO uint32_t CR2;
    __IO uint32_t CNT2;
    __IO uint32_t MIN2;
    __IO uint32_t MAX2;
    __IO uint32_t CR3;
    __IO uint32_t CNT3;
    __IO uint32_t MIN3;
    __IO uint32_t MAX3;
    __IO uint32_t CR4;
    __IO uint32_t CNT4;
    __IO uint32_t MIN4;
    __IO uint32_t MAX4;
    __IO uint32_t CR5;
    __IO uint32_t CNT5;
    __IO uint32_t MIN5;
    __IO uint32_t MAX5;
    __IO uint32_t CR6;
    __IO uint32_t CNT6;
    __IO uint32_t MIN6;
    __IO uint32_t MAX6;
    __IO uint32_t CR7;
    __IO uint32_t CNT7;
    __IO uint32_t MIN7;
    __IO uint32_t MAX7;
    __IO uint32_t CR8;
    __IO uint32_t CNT8;
    __IO uint32_t MIN8;
    __IO uint32_t MAX8;
    __IO uint32_t CER;
    __IO uint32_t CCR;
    __IO uint32_t TOV;
    __IO uint32_t TOF;
    __IO uint32_t TOA;
} BUSMON_TypeDef;


/******************* Bit definition for BUSMON_CR1 register *******************/
#define BUSMON_CR1_SEL_Pos              (0U)
#define BUSMON_CR1_SEL_Msk              (0x7UL << BUSMON_CR1_SEL_Pos)
#define BUSMON_CR1_SEL                  BUSMON_CR1_SEL_Msk
#define BUSMON_CR1_OP_Pos               (4U)
#define BUSMON_CR1_OP_Msk               (0x3UL << BUSMON_CR1_OP_Pos)
#define BUSMON_CR1_OP                   BUSMON_CR1_OP_Msk
#define BUSMON_CR1_AUTOCLR_Pos          (6U)
#define BUSMON_CR1_AUTOCLR_Msk          (0x1UL << BUSMON_CR1_AUTOCLR_Pos)
#define BUSMON_CR1_AUTOCLR              BUSMON_CR1_AUTOCLR_Msk
#define BUSMON_CR1_CMPEN_Pos            (7U)
#define BUSMON_CR1_CMPEN_Msk            (0x1UL << BUSMON_CR1_CMPEN_Pos)
#define BUSMON_CR1_CMPEN                BUSMON_CR1_CMPEN_Msk
#define BUSMON_CR1_CMP_Pos              (8U)
#define BUSMON_CR1_CMP_Msk              (0xFFFFFFUL << BUSMON_CR1_CMP_Pos)
#define BUSMON_CR1_CMP                  BUSMON_CR1_CMP_Msk

/****************** Bit definition for BUSMON_CNT1 register *******************/
#define BUSMON_CNT1_CNT_Pos             (0U)
#define BUSMON_CNT1_CNT_Msk             (0x7FFFFFFFUL << BUSMON_CNT1_CNT_Pos)
#define BUSMON_CNT1_CNT                 BUSMON_CNT1_CNT_Msk
#define BUSMON_CNT1_OF_Pos              (31U)
#define BUSMON_CNT1_OF_Msk              (0x1UL << BUSMON_CNT1_OF_Pos)
#define BUSMON_CNT1_OF                  BUSMON_CNT1_OF_Msk

/****************** Bit definition for BUSMON_MIN1 register *******************/
#define BUSMON_MIN1_ADDR_Pos            (0U)
#define BUSMON_MIN1_ADDR_Msk            (0xFFFFFFFFUL << BUSMON_MIN1_ADDR_Pos)
#define BUSMON_MIN1_ADDR                BUSMON_MIN1_ADDR_Msk

/****************** Bit definition for BUSMON_MAX1 register *******************/
#define BUSMON_MAX1_ADDR_Pos            (0U)
#define BUSMON_MAX1_ADDR_Msk            (0xFFFFFFFFUL << BUSMON_MAX1_ADDR_Pos)
#define BUSMON_MAX1_ADDR                BUSMON_MAX1_ADDR_Msk

/******************* Bit definition for BUSMON_CR2 register *******************/
#define BUSMON_CR2_SEL_Pos              (0U)
#define BUSMON_CR2_SEL_Msk              (0x7UL << BUSMON_CR2_SEL_Pos)
#define BUSMON_CR2_SEL                  BUSMON_CR2_SEL_Msk
#define BUSMON_CR2_OP_Pos               (4U)
#define BUSMON_CR2_OP_Msk               (0x3UL << BUSMON_CR2_OP_Pos)
#define BUSMON_CR2_OP                   BUSMON_CR2_OP_Msk
#define BUSMON_CR2_AUTOCLR_Pos          (6U)
#define BUSMON_CR2_AUTOCLR_Msk          (0x1UL << BUSMON_CR2_AUTOCLR_Pos)
#define BUSMON_CR2_AUTOCLR              BUSMON_CR2_AUTOCLR_Msk
#define BUSMON_CR2_CMPEN_Pos            (7U)
#define BUSMON_CR2_CMPEN_Msk            (0x1UL << BUSMON_CR2_CMPEN_Pos)
#define BUSMON_CR2_CMPEN                BUSMON_CR2_CMPEN_Msk
#define BUSMON_CR2_CMP_Pos              (8U)
#define BUSMON_CR2_CMP_Msk              (0xFFFFFFUL << BUSMON_CR2_CMP_Pos)
#define BUSMON_CR2_CMP                  BUSMON_CR2_CMP_Msk

/****************** Bit definition for BUSMON_CNT2 register *******************/
#define BUSMON_CNT2_CNT_Pos             (0U)
#define BUSMON_CNT2_CNT_Msk             (0x7FFFFFFFUL << BUSMON_CNT2_CNT_Pos)
#define BUSMON_CNT2_CNT                 BUSMON_CNT2_CNT_Msk
#define BUSMON_CNT2_OF_Pos              (31U)
#define BUSMON_CNT2_OF_Msk              (0x1UL << BUSMON_CNT2_OF_Pos)
#define BUSMON_CNT2_OF                  BUSMON_CNT2_OF_Msk

/****************** Bit definition for BUSMON_MIN2 register *******************/
#define BUSMON_MIN2_ADDR_Pos            (0U)
#define BUSMON_MIN2_ADDR_Msk            (0xFFFFFFFFUL << BUSMON_MIN2_ADDR_Pos)
#define BUSMON_MIN2_ADDR                BUSMON_MIN2_ADDR_Msk

/****************** Bit definition for BUSMON_MAX2 register *******************/
#define BUSMON_MAX2_ADDR_Pos            (0U)
#define BUSMON_MAX2_ADDR_Msk            (0xFFFFFFFFUL << BUSMON_MAX2_ADDR_Pos)
#define BUSMON_MAX2_ADDR                BUSMON_MAX2_ADDR_Msk

/******************* Bit definition for BUSMON_CR3 register *******************/
#define BUSMON_CR3_SEL_Pos              (0U)
#define BUSMON_CR3_SEL_Msk              (0x7UL << BUSMON_CR3_SEL_Pos)
#define BUSMON_CR3_SEL                  BUSMON_CR3_SEL_Msk
#define BUSMON_CR3_OP_Pos               (4U)
#define BUSMON_CR3_OP_Msk               (0x3UL << BUSMON_CR3_OP_Pos)
#define BUSMON_CR3_OP                   BUSMON_CR3_OP_Msk
#define BUSMON_CR3_AUTOCLR_Pos          (6U)
#define BUSMON_CR3_AUTOCLR_Msk          (0x1UL << BUSMON_CR3_AUTOCLR_Pos)
#define BUSMON_CR3_AUTOCLR              BUSMON_CR3_AUTOCLR_Msk
#define BUSMON_CR3_CMPEN_Pos            (7U)
#define BUSMON_CR3_CMPEN_Msk            (0x1UL << BUSMON_CR3_CMPEN_Pos)
#define BUSMON_CR3_CMPEN                BUSMON_CR3_CMPEN_Msk
#define BUSMON_CR3_CMP_Pos              (8U)
#define BUSMON_CR3_CMP_Msk              (0xFFFFFFUL << BUSMON_CR3_CMP_Pos)
#define BUSMON_CR3_CMP                  BUSMON_CR3_CMP_Msk

/****************** Bit definition for BUSMON_CNT3 register *******************/
#define BUSMON_CNT3_CNT_Pos             (0U)
#define BUSMON_CNT3_CNT_Msk             (0x7FFFFFFFUL << BUSMON_CNT3_CNT_Pos)
#define BUSMON_CNT3_CNT                 BUSMON_CNT3_CNT_Msk
#define BUSMON_CNT3_OF_Pos              (31U)
#define BUSMON_CNT3_OF_Msk              (0x1UL << BUSMON_CNT3_OF_Pos)
#define BUSMON_CNT3_OF                  BUSMON_CNT3_OF_Msk

/****************** Bit definition for BUSMON_MIN3 register *******************/
#define BUSMON_MIN3_ADDR_Pos            (0U)
#define BUSMON_MIN3_ADDR_Msk            (0xFFFFFFFFUL << BUSMON_MIN3_ADDR_Pos)
#define BUSMON_MIN3_ADDR                BUSMON_MIN3_ADDR_Msk

/****************** Bit definition for BUSMON_MAX3 register *******************/
#define BUSMON_MAX3_ADDR_Pos            (0U)
#define BUSMON_MAX3_ADDR_Msk            (0xFFFFFFFFUL << BUSMON_MAX3_ADDR_Pos)
#define BUSMON_MAX3_ADDR                BUSMON_MAX3_ADDR_Msk

/******************* Bit definition for BUSMON_CR4 register *******************/
#define BUSMON_CR4_SEL_Pos              (0U)
#define BUSMON_CR4_SEL_Msk              (0x7UL << BUSMON_CR4_SEL_Pos)
#define BUSMON_CR4_SEL                  BUSMON_CR4_SEL_Msk
#define BUSMON_CR4_OP_Pos               (4U)
#define BUSMON_CR4_OP_Msk               (0x3UL << BUSMON_CR4_OP_Pos)
#define BUSMON_CR4_OP                   BUSMON_CR4_OP_Msk
#define BUSMON_CR4_AUTOCLR_Pos          (6U)
#define BUSMON_CR4_AUTOCLR_Msk          (0x1UL << BUSMON_CR4_AUTOCLR_Pos)
#define BUSMON_CR4_AUTOCLR              BUSMON_CR4_AUTOCLR_Msk
#define BUSMON_CR4_CMPEN_Pos            (7U)
#define BUSMON_CR4_CMPEN_Msk            (0x1UL << BUSMON_CR4_CMPEN_Pos)
#define BUSMON_CR4_CMPEN                BUSMON_CR4_CMPEN_Msk
#define BUSMON_CR4_CMP_Pos              (8U)
#define BUSMON_CR4_CMP_Msk              (0xFFFFFFUL << BUSMON_CR4_CMP_Pos)
#define BUSMON_CR4_CMP                  BUSMON_CR4_CMP_Msk

/****************** Bit definition for BUSMON_CNT4 register *******************/
#define BUSMON_CNT4_CNT_Pos             (0U)
#define BUSMON_CNT4_CNT_Msk             (0x7FFFFFFFUL << BUSMON_CNT4_CNT_Pos)
#define BUSMON_CNT4_CNT                 BUSMON_CNT4_CNT_Msk
#define BUSMON_CNT4_OF_Pos              (31U)
#define BUSMON_CNT4_OF_Msk              (0x1UL << BUSMON_CNT4_OF_Pos)
#define BUSMON_CNT4_OF                  BUSMON_CNT4_OF_Msk

/****************** Bit definition for BUSMON_MIN4 register *******************/
#define BUSMON_MIN4_ADDR_Pos            (0U)
#define BUSMON_MIN4_ADDR_Msk            (0xFFFFFFFFUL << BUSMON_MIN4_ADDR_Pos)
#define BUSMON_MIN4_ADDR                BUSMON_MIN4_ADDR_Msk

/****************** Bit definition for BUSMON_MAX4 register *******************/
#define BUSMON_MAX4_ADDR_Pos            (0U)
#define BUSMON_MAX4_ADDR_Msk            (0xFFFFFFFFUL << BUSMON_MAX4_ADDR_Pos)
#define BUSMON_MAX4_ADDR                BUSMON_MAX4_ADDR_Msk

/******************* Bit definition for BUSMON_CR5 register *******************/
#define BUSMON_CR5_SEL_Pos              (0U)
#define BUSMON_CR5_SEL_Msk              (0x7UL << BUSMON_CR5_SEL_Pos)
#define BUSMON_CR5_SEL                  BUSMON_CR5_SEL_Msk
#define BUSMON_CR5_OP_Pos               (4U)
#define BUSMON_CR5_OP_Msk               (0x3UL << BUSMON_CR5_OP_Pos)
#define BUSMON_CR5_OP                   BUSMON_CR5_OP_Msk
#define BUSMON_CR5_AUTOCLR_Pos          (6U)
#define BUSMON_CR5_AUTOCLR_Msk          (0x1UL << BUSMON_CR5_AUTOCLR_Pos)
#define BUSMON_CR5_AUTOCLR              BUSMON_CR5_AUTOCLR_Msk
#define BUSMON_CR5_CMPEN_Pos            (7U)
#define BUSMON_CR5_CMPEN_Msk            (0x1UL << BUSMON_CR5_CMPEN_Pos)
#define BUSMON_CR5_CMPEN                BUSMON_CR5_CMPEN_Msk
#define BUSMON_CR5_CMP_Pos              (8U)
#define BUSMON_CR5_CMP_Msk              (0xFFFFFFUL << BUSMON_CR5_CMP_Pos)
#define BUSMON_CR5_CMP                  BUSMON_CR5_CMP_Msk

/****************** Bit definition for BUSMON_CNT5 register *******************/
#define BUSMON_CNT5_CNT_Pos             (0U)
#define BUSMON_CNT5_CNT_Msk             (0x7FFFFFFFUL << BUSMON_CNT5_CNT_Pos)
#define BUSMON_CNT5_CNT                 BUSMON_CNT5_CNT_Msk
#define BUSMON_CNT5_OF_Pos              (31U)
#define BUSMON_CNT5_OF_Msk              (0x1UL << BUSMON_CNT5_OF_Pos)
#define BUSMON_CNT5_OF                  BUSMON_CNT5_OF_Msk

/****************** Bit definition for BUSMON_MIN5 register *******************/
#define BUSMON_MIN5_ADDR_Pos            (0U)
#define BUSMON_MIN5_ADDR_Msk            (0xFFFFFFFFUL << BUSMON_MIN5_ADDR_Pos)
#define BUSMON_MIN5_ADDR                BUSMON_MIN5_ADDR_Msk

/****************** Bit definition for BUSMON_MAX5 register *******************/
#define BUSMON_MAX5_ADDR_Pos            (0U)
#define BUSMON_MAX5_ADDR_Msk            (0xFFFFFFFFUL << BUSMON_MAX5_ADDR_Pos)
#define BUSMON_MAX5_ADDR                BUSMON_MAX5_ADDR_Msk

/******************* Bit definition for BUSMON_CR6 register *******************/
#define BUSMON_CR6_SEL_Pos              (0U)
#define BUSMON_CR6_SEL_Msk              (0x7UL << BUSMON_CR6_SEL_Pos)
#define BUSMON_CR6_SEL                  BUSMON_CR6_SEL_Msk
#define BUSMON_CR6_OP_Pos               (4U)
#define BUSMON_CR6_OP_Msk               (0x3UL << BUSMON_CR6_OP_Pos)
#define BUSMON_CR6_OP                   BUSMON_CR6_OP_Msk
#define BUSMON_CR6_AUTOCLR_Pos          (6U)
#define BUSMON_CR6_AUTOCLR_Msk          (0x1UL << BUSMON_CR6_AUTOCLR_Pos)
#define BUSMON_CR6_AUTOCLR              BUSMON_CR6_AUTOCLR_Msk
#define BUSMON_CR6_CMPEN_Pos            (7U)
#define BUSMON_CR6_CMPEN_Msk            (0x1UL << BUSMON_CR6_CMPEN_Pos)
#define BUSMON_CR6_CMPEN                BUSMON_CR6_CMPEN_Msk
#define BUSMON_CR6_CMP_Pos              (8U)
#define BUSMON_CR6_CMP_Msk              (0xFFFFFFUL << BUSMON_CR6_CMP_Pos)
#define BUSMON_CR6_CMP                  BUSMON_CR6_CMP_Msk

/****************** Bit definition for BUSMON_CNT6 register *******************/
#define BUSMON_CNT6_CNT_Pos             (0U)
#define BUSMON_CNT6_CNT_Msk             (0x7FFFFFFFUL << BUSMON_CNT6_CNT_Pos)
#define BUSMON_CNT6_CNT                 BUSMON_CNT6_CNT_Msk
#define BUSMON_CNT6_OF_Pos              (31U)
#define BUSMON_CNT6_OF_Msk              (0x1UL << BUSMON_CNT6_OF_Pos)
#define BUSMON_CNT6_OF                  BUSMON_CNT6_OF_Msk

/****************** Bit definition for BUSMON_MIN6 register *******************/
#define BUSMON_MIN6_ADDR_Pos            (0U)
#define BUSMON_MIN6_ADDR_Msk            (0xFFFFFFFFUL << BUSMON_MIN6_ADDR_Pos)
#define BUSMON_MIN6_ADDR                BUSMON_MIN6_ADDR_Msk

/****************** Bit definition for BUSMON_MAX6 register *******************/
#define BUSMON_MAX6_ADDR_Pos            (0U)
#define BUSMON_MAX6_ADDR_Msk            (0xFFFFFFFFUL << BUSMON_MAX6_ADDR_Pos)
#define BUSMON_MAX6_ADDR                BUSMON_MAX6_ADDR_Msk

/******************* Bit definition for BUSMON_CR7 register *******************/
#define BUSMON_CR7_SEL_Pos              (0U)
#define BUSMON_CR7_SEL_Msk              (0x7UL << BUSMON_CR7_SEL_Pos)
#define BUSMON_CR7_SEL                  BUSMON_CR7_SEL_Msk
#define BUSMON_CR7_OP_Pos               (4U)
#define BUSMON_CR7_OP_Msk               (0x3UL << BUSMON_CR7_OP_Pos)
#define BUSMON_CR7_OP                   BUSMON_CR7_OP_Msk
#define BUSMON_CR7_AUTOCLR_Pos          (6U)
#define BUSMON_CR7_AUTOCLR_Msk          (0x1UL << BUSMON_CR7_AUTOCLR_Pos)
#define BUSMON_CR7_AUTOCLR              BUSMON_CR7_AUTOCLR_Msk
#define BUSMON_CR7_CMPEN_Pos            (7U)
#define BUSMON_CR7_CMPEN_Msk            (0x1UL << BUSMON_CR7_CMPEN_Pos)
#define BUSMON_CR7_CMPEN                BUSMON_CR7_CMPEN_Msk
#define BUSMON_CR7_CMP_Pos              (8U)
#define BUSMON_CR7_CMP_Msk              (0xFFFFFFUL << BUSMON_CR7_CMP_Pos)
#define BUSMON_CR7_CMP                  BUSMON_CR7_CMP_Msk

/****************** Bit definition for BUSMON_CNT7 register *******************/
#define BUSMON_CNT7_CNT_Pos             (0U)
#define BUSMON_CNT7_CNT_Msk             (0x7FFFFFFFUL << BUSMON_CNT7_CNT_Pos)
#define BUSMON_CNT7_CNT                 BUSMON_CNT7_CNT_Msk
#define BUSMON_CNT7_OF_Pos              (31U)
#define BUSMON_CNT7_OF_Msk              (0x1UL << BUSMON_CNT7_OF_Pos)
#define BUSMON_CNT7_OF                  BUSMON_CNT7_OF_Msk

/****************** Bit definition for BUSMON_MIN7 register *******************/
#define BUSMON_MIN7_ADDR_Pos            (0U)
#define BUSMON_MIN7_ADDR_Msk            (0xFFFFFFFFUL << BUSMON_MIN7_ADDR_Pos)
#define BUSMON_MIN7_ADDR                BUSMON_MIN7_ADDR_Msk

/****************** Bit definition for BUSMON_MAX7 register *******************/
#define BUSMON_MAX7_ADDR_Pos            (0U)
#define BUSMON_MAX7_ADDR_Msk            (0xFFFFFFFFUL << BUSMON_MAX7_ADDR_Pos)
#define BUSMON_MAX7_ADDR                BUSMON_MAX7_ADDR_Msk

/******************* Bit definition for BUSMON_CR8 register *******************/
#define BUSMON_CR8_SEL_Pos              (0U)
#define BUSMON_CR8_SEL_Msk              (0x7UL << BUSMON_CR8_SEL_Pos)
#define BUSMON_CR8_SEL                  BUSMON_CR8_SEL_Msk
#define BUSMON_CR8_OP_Pos               (4U)
#define BUSMON_CR8_OP_Msk               (0x3UL << BUSMON_CR8_OP_Pos)
#define BUSMON_CR8_OP                   BUSMON_CR8_OP_Msk
#define BUSMON_CR8_AUTOCLR_Pos          (6U)
#define BUSMON_CR8_AUTOCLR_Msk          (0x1UL << BUSMON_CR8_AUTOCLR_Pos)
#define BUSMON_CR8_AUTOCLR              BUSMON_CR8_AUTOCLR_Msk
#define BUSMON_CR8_CMPEN_Pos            (7U)
#define BUSMON_CR8_CMPEN_Msk            (0x1UL << BUSMON_CR8_CMPEN_Pos)
#define BUSMON_CR8_CMPEN                BUSMON_CR8_CMPEN_Msk
#define BUSMON_CR8_CMP_Pos              (8U)
#define BUSMON_CR8_CMP_Msk              (0xFFFFFFUL << BUSMON_CR8_CMP_Pos)
#define BUSMON_CR8_CMP                  BUSMON_CR8_CMP_Msk

/****************** Bit definition for BUSMON_CNT8 register *******************/
#define BUSMON_CNT8_CNT_Pos             (0U)
#define BUSMON_CNT8_CNT_Msk             (0x7FFFFFFFUL << BUSMON_CNT8_CNT_Pos)
#define BUSMON_CNT8_CNT                 BUSMON_CNT8_CNT_Msk
#define BUSMON_CNT8_OF_Pos              (31U)
#define BUSMON_CNT8_OF_Msk              (0x1UL << BUSMON_CNT8_OF_Pos)
#define BUSMON_CNT8_OF                  BUSMON_CNT8_OF_Msk

/****************** Bit definition for BUSMON_MIN8 register *******************/
#define BUSMON_MIN8_ADDR_Pos            (0U)
#define BUSMON_MIN8_ADDR_Msk            (0xFFFFFFFFUL << BUSMON_MIN8_ADDR_Pos)
#define BUSMON_MIN8_ADDR                BUSMON_MIN8_ADDR_Msk

/****************** Bit definition for BUSMON_MAX8 register *******************/
#define BUSMON_MAX8_ADDR_Pos            (0U)
#define BUSMON_MAX8_ADDR_Msk            (0xFFFFFFFFUL << BUSMON_MAX8_ADDR_Pos)
#define BUSMON_MAX8_ADDR                BUSMON_MAX8_ADDR_Msk

/******************* Bit definition for BUSMON_CER register *******************/
#define BUSMON_CER_EN1_Pos              (0U)
#define BUSMON_CER_EN1_Msk              (0x1UL << BUSMON_CER_EN1_Pos)
#define BUSMON_CER_EN1                  BUSMON_CER_EN1_Msk
#define BUSMON_CER_EN2_Pos              (1U)
#define BUSMON_CER_EN2_Msk              (0x1UL << BUSMON_CER_EN2_Pos)
#define BUSMON_CER_EN2                  BUSMON_CER_EN2_Msk
#define BUSMON_CER_EN3_Pos              (2U)
#define BUSMON_CER_EN3_Msk              (0x1UL << BUSMON_CER_EN3_Pos)
#define BUSMON_CER_EN3                  BUSMON_CER_EN3_Msk
#define BUSMON_CER_EN4_Pos              (3U)
#define BUSMON_CER_EN4_Msk              (0x1UL << BUSMON_CER_EN4_Pos)
#define BUSMON_CER_EN4                  BUSMON_CER_EN4_Msk
#define BUSMON_CER_EN5_Pos              (4U)
#define BUSMON_CER_EN5_Msk              (0x1UL << BUSMON_CER_EN5_Pos)
#define BUSMON_CER_EN5                  BUSMON_CER_EN5_Msk
#define BUSMON_CER_EN6_Pos              (5U)
#define BUSMON_CER_EN6_Msk              (0x1UL << BUSMON_CER_EN6_Pos)
#define BUSMON_CER_EN6                  BUSMON_CER_EN6_Msk
#define BUSMON_CER_EN7_Pos              (6U)
#define BUSMON_CER_EN7_Msk              (0x1UL << BUSMON_CER_EN7_Pos)
#define BUSMON_CER_EN7                  BUSMON_CER_EN7_Msk
#define BUSMON_CER_EN8_Pos              (7U)
#define BUSMON_CER_EN8_Msk              (0x1UL << BUSMON_CER_EN8_Pos)
#define BUSMON_CER_EN8                  BUSMON_CER_EN8_Msk

/******************* Bit definition for BUSMON_CCR register *******************/
#define BUSMON_CCR_CLR1_Pos             (0U)
#define BUSMON_CCR_CLR1_Msk             (0x1UL << BUSMON_CCR_CLR1_Pos)
#define BUSMON_CCR_CLR1                 BUSMON_CCR_CLR1_Msk
#define BUSMON_CCR_CLR2_Pos             (1U)
#define BUSMON_CCR_CLR2_Msk             (0x1UL << BUSMON_CCR_CLR2_Pos)
#define BUSMON_CCR_CLR2                 BUSMON_CCR_CLR2_Msk
#define BUSMON_CCR_CLR3_Pos             (2U)
#define BUSMON_CCR_CLR3_Msk             (0x1UL << BUSMON_CCR_CLR3_Pos)
#define BUSMON_CCR_CLR3                 BUSMON_CCR_CLR3_Msk
#define BUSMON_CCR_CLR4_Pos             (3U)
#define BUSMON_CCR_CLR4_Msk             (0x1UL << BUSMON_CCR_CLR4_Pos)
#define BUSMON_CCR_CLR4                 BUSMON_CCR_CLR4_Msk
#define BUSMON_CCR_CLR5_Pos             (4U)
#define BUSMON_CCR_CLR5_Msk             (0x1UL << BUSMON_CCR_CLR5_Pos)
#define BUSMON_CCR_CLR5                 BUSMON_CCR_CLR5_Msk
#define BUSMON_CCR_CLR6_Pos             (5U)
#define BUSMON_CCR_CLR6_Msk             (0x1UL << BUSMON_CCR_CLR6_Pos)
#define BUSMON_CCR_CLR6                 BUSMON_CCR_CLR6_Msk
#define BUSMON_CCR_CLR7_Pos             (6U)
#define BUSMON_CCR_CLR7_Msk             (0x1UL << BUSMON_CCR_CLR7_Pos)
#define BUSMON_CCR_CLR7                 BUSMON_CCR_CLR7_Msk
#define BUSMON_CCR_CLR8_Pos             (7U)
#define BUSMON_CCR_CLR8_Msk             (0x1UL << BUSMON_CCR_CLR8_Pos)
#define BUSMON_CCR_CLR8                 BUSMON_CCR_CLR8_Msk

/******************* Bit definition for BUSMON_TOV register *******************/
#define BUSMON_TOV_VAL_Pos              (0U)
#define BUSMON_TOV_VAL_Msk              (0xFFUL << BUSMON_TOV_VAL_Pos)
#define BUSMON_TOV_VAL                  BUSMON_TOV_VAL_Msk

/******************* Bit definition for BUSMON_TOF register *******************/
#define BUSMON_TOF_TO_Pos               (0U)
#define BUSMON_TOF_TO_Msk               (0xFFFFUL << BUSMON_TOF_TO_Pos)
#define BUSMON_TOF_TO                   BUSMON_TOF_TO_Msk

/******************* Bit definition for BUSMON_TOA register *******************/
#define BUSMON_TOA_ADDR_Pos             (0U)
#define BUSMON_TOA_ADDR_Msk             (0xFFFFFFFFUL << BUSMON_TOA_ADDR_Pos)
#define BUSMON_TOA_ADDR                 BUSMON_TOA_ADDR_Msk

#endif
