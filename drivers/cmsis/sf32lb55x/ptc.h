#ifndef __PTC_H
#define __PTC_H

typedef struct
{
    __IO uint32_t ISR;
    __IO uint32_t ICR;
    __IO uint32_t IER;
    __IO uint32_t TCR1;
    __IO uint32_t TAR1;
    __IO uint32_t TDR1;
    __IO uint32_t TCR2;
    __IO uint32_t TAR2;
    __IO uint32_t TDR2;
    __IO uint32_t TCR3;
    __IO uint32_t TAR3;
    __IO uint32_t TDR3;
    __IO uint32_t TCR4;
    __IO uint32_t TAR4;
    __IO uint32_t TDR4;
    __IO uint32_t TCR5;
    __IO uint32_t TAR5;
    __IO uint32_t TDR5;
    __IO uint32_t TCR6;
    __IO uint32_t TAR6;
    __IO uint32_t TDR6;
    __IO uint32_t TCR7;
    __IO uint32_t TAR7;
    __IO uint32_t TDR7;
    __IO uint32_t TCR8;
    __IO uint32_t TAR8;
    __IO uint32_t TDR8;
    __IO uint32_t RSVD1;
    __IO uint32_t MEM1;
    __IO uint32_t MEM2;
    __IO uint32_t MEM3;
    __IO uint32_t MEM4;
    __IO uint32_t GPIO31_0;
    __IO uint32_t GPIO63_32;
    __IO uint32_t GPIO95_64;
} PTC_TypeDef;


/******************** Bit definition for PTC_ISR register *********************/
#define PTC_ISR_TCIF1_Pos               (0U)
#define PTC_ISR_TCIF1_Msk               (0x1UL << PTC_ISR_TCIF1_Pos)
#define PTC_ISR_TCIF1                   PTC_ISR_TCIF1_Msk
#define PTC_ISR_TCIF2_Pos               (1U)
#define PTC_ISR_TCIF2_Msk               (0x1UL << PTC_ISR_TCIF2_Pos)
#define PTC_ISR_TCIF2                   PTC_ISR_TCIF2_Msk
#define PTC_ISR_TCIF3_Pos               (2U)
#define PTC_ISR_TCIF3_Msk               (0x1UL << PTC_ISR_TCIF3_Pos)
#define PTC_ISR_TCIF3                   PTC_ISR_TCIF3_Msk
#define PTC_ISR_TCIF4_Pos               (3U)
#define PTC_ISR_TCIF4_Msk               (0x1UL << PTC_ISR_TCIF4_Pos)
#define PTC_ISR_TCIF4                   PTC_ISR_TCIF4_Msk
#define PTC_ISR_TCIF5_Pos               (4U)
#define PTC_ISR_TCIF5_Msk               (0x1UL << PTC_ISR_TCIF5_Pos)
#define PTC_ISR_TCIF5                   PTC_ISR_TCIF5_Msk
#define PTC_ISR_TCIF6_Pos               (5U)
#define PTC_ISR_TCIF6_Msk               (0x1UL << PTC_ISR_TCIF6_Pos)
#define PTC_ISR_TCIF6                   PTC_ISR_TCIF6_Msk
#define PTC_ISR_TCIF7_Pos               (6U)
#define PTC_ISR_TCIF7_Msk               (0x1UL << PTC_ISR_TCIF7_Pos)
#define PTC_ISR_TCIF7                   PTC_ISR_TCIF7_Msk
#define PTC_ISR_TCIF8_Pos               (7U)
#define PTC_ISR_TCIF8_Msk               (0x1UL << PTC_ISR_TCIF8_Pos)
#define PTC_ISR_TCIF8                   PTC_ISR_TCIF8_Msk
#define PTC_ISR_TEIF1_Pos               (16U)
#define PTC_ISR_TEIF1_Msk               (0x1UL << PTC_ISR_TEIF1_Pos)
#define PTC_ISR_TEIF1                   PTC_ISR_TEIF1_Msk
#define PTC_ISR_TEIF2_Pos               (17U)
#define PTC_ISR_TEIF2_Msk               (0x1UL << PTC_ISR_TEIF2_Pos)
#define PTC_ISR_TEIF2                   PTC_ISR_TEIF2_Msk
#define PTC_ISR_TEIF3_Pos               (18U)
#define PTC_ISR_TEIF3_Msk               (0x1UL << PTC_ISR_TEIF3_Pos)
#define PTC_ISR_TEIF3                   PTC_ISR_TEIF3_Msk
#define PTC_ISR_TEIF4_Pos               (19U)
#define PTC_ISR_TEIF4_Msk               (0x1UL << PTC_ISR_TEIF4_Pos)
#define PTC_ISR_TEIF4                   PTC_ISR_TEIF4_Msk
#define PTC_ISR_TEIF5_Pos               (20U)
#define PTC_ISR_TEIF5_Msk               (0x1UL << PTC_ISR_TEIF5_Pos)
#define PTC_ISR_TEIF5                   PTC_ISR_TEIF5_Msk
#define PTC_ISR_TEIF6_Pos               (21U)
#define PTC_ISR_TEIF6_Msk               (0x1UL << PTC_ISR_TEIF6_Pos)
#define PTC_ISR_TEIF6                   PTC_ISR_TEIF6_Msk
#define PTC_ISR_TEIF7_Pos               (22U)
#define PTC_ISR_TEIF7_Msk               (0x1UL << PTC_ISR_TEIF7_Pos)
#define PTC_ISR_TEIF7                   PTC_ISR_TEIF7_Msk
#define PTC_ISR_TEIF8_Pos               (23U)
#define PTC_ISR_TEIF8_Msk               (0x1UL << PTC_ISR_TEIF8_Pos)
#define PTC_ISR_TEIF8                   PTC_ISR_TEIF8_Msk

/******************** Bit definition for PTC_ICR register *********************/
#define PTC_ICR_CTCIF1_Pos              (0U)
#define PTC_ICR_CTCIF1_Msk              (0x1UL << PTC_ICR_CTCIF1_Pos)
#define PTC_ICR_CTCIF1                  PTC_ICR_CTCIF1_Msk
#define PTC_ICR_CTCIF2_Pos              (1U)
#define PTC_ICR_CTCIF2_Msk              (0x1UL << PTC_ICR_CTCIF2_Pos)
#define PTC_ICR_CTCIF2                  PTC_ICR_CTCIF2_Msk
#define PTC_ICR_CTCIF3_Pos              (2U)
#define PTC_ICR_CTCIF3_Msk              (0x1UL << PTC_ICR_CTCIF3_Pos)
#define PTC_ICR_CTCIF3                  PTC_ICR_CTCIF3_Msk
#define PTC_ICR_CTCIF4_Pos              (3U)
#define PTC_ICR_CTCIF4_Msk              (0x1UL << PTC_ICR_CTCIF4_Pos)
#define PTC_ICR_CTCIF4                  PTC_ICR_CTCIF4_Msk
#define PTC_ICR_CTCIF5_Pos              (4U)
#define PTC_ICR_CTCIF5_Msk              (0x1UL << PTC_ICR_CTCIF5_Pos)
#define PTC_ICR_CTCIF5                  PTC_ICR_CTCIF5_Msk
#define PTC_ICR_CTCIF6_Pos              (5U)
#define PTC_ICR_CTCIF6_Msk              (0x1UL << PTC_ICR_CTCIF6_Pos)
#define PTC_ICR_CTCIF6                  PTC_ICR_CTCIF6_Msk
#define PTC_ICR_CTCIF7_Pos              (6U)
#define PTC_ICR_CTCIF7_Msk              (0x1UL << PTC_ICR_CTCIF7_Pos)
#define PTC_ICR_CTCIF7                  PTC_ICR_CTCIF7_Msk
#define PTC_ICR_CTCIF8_Pos              (7U)
#define PTC_ICR_CTCIF8_Msk              (0x1UL << PTC_ICR_CTCIF8_Pos)
#define PTC_ICR_CTCIF8                  PTC_ICR_CTCIF8_Msk
#define PTC_ICR_CTEIF_Pos               (16U)
#define PTC_ICR_CTEIF_Msk               (0x1UL << PTC_ICR_CTEIF_Pos)
#define PTC_ICR_CTEIF                   PTC_ICR_CTEIF_Msk

/******************** Bit definition for PTC_IER register *********************/
#define PTC_IER_TCIE1_Pos               (0U)
#define PTC_IER_TCIE1_Msk               (0x1UL << PTC_IER_TCIE1_Pos)
#define PTC_IER_TCIE1                   PTC_IER_TCIE1_Msk
#define PTC_IER_TCIE2_Pos               (1U)
#define PTC_IER_TCIE2_Msk               (0x1UL << PTC_IER_TCIE2_Pos)
#define PTC_IER_TCIE2                   PTC_IER_TCIE2_Msk
#define PTC_IER_TCIE3_Pos               (2U)
#define PTC_IER_TCIE3_Msk               (0x1UL << PTC_IER_TCIE3_Pos)
#define PTC_IER_TCIE3                   PTC_IER_TCIE3_Msk
#define PTC_IER_TCIE4_Pos               (3U)
#define PTC_IER_TCIE4_Msk               (0x1UL << PTC_IER_TCIE4_Pos)
#define PTC_IER_TCIE4                   PTC_IER_TCIE4_Msk
#define PTC_IER_TCIE5_Pos               (4U)
#define PTC_IER_TCIE5_Msk               (0x1UL << PTC_IER_TCIE5_Pos)
#define PTC_IER_TCIE5                   PTC_IER_TCIE5_Msk
#define PTC_IER_TCIE6_Pos               (5U)
#define PTC_IER_TCIE6_Msk               (0x1UL << PTC_IER_TCIE6_Pos)
#define PTC_IER_TCIE6                   PTC_IER_TCIE6_Msk
#define PTC_IER_TCIE7_Pos               (6U)
#define PTC_IER_TCIE7_Msk               (0x1UL << PTC_IER_TCIE7_Pos)
#define PTC_IER_TCIE7                   PTC_IER_TCIE7_Msk
#define PTC_IER_TCIE8_Pos               (7U)
#define PTC_IER_TCIE8_Msk               (0x1UL << PTC_IER_TCIE8_Pos)
#define PTC_IER_TCIE8                   PTC_IER_TCIE8_Msk
#define PTC_IER_TEIE_Pos                (16U)
#define PTC_IER_TEIE_Msk                (0x1UL << PTC_IER_TEIE_Pos)
#define PTC_IER_TEIE                    PTC_IER_TEIE_Msk

/******************** Bit definition for PTC_TCR1 register ********************/
#define PTC_TCR1_TRIGSEL_Pos            (0U)
#define PTC_TCR1_TRIGSEL_Msk            (0xFFUL << PTC_TCR1_TRIGSEL_Pos)
#define PTC_TCR1_TRIGSEL                PTC_TCR1_TRIGSEL_Msk
#define PTC_TCR1_OP_Pos                 (16U)
#define PTC_TCR1_OP_Msk                 (0x7UL << PTC_TCR1_OP_Pos)
#define PTC_TCR1_OP                     PTC_TCR1_OP_Msk

/******************** Bit definition for PTC_TAR1 register ********************/
#define PTC_TAR1_ADDR_Pos               (0U)
#define PTC_TAR1_ADDR_Msk               (0xFFFFFFFFUL << PTC_TAR1_ADDR_Pos)
#define PTC_TAR1_ADDR                   PTC_TAR1_ADDR_Msk

/******************** Bit definition for PTC_TDR1 register ********************/
#define PTC_TDR1_DATA_Pos               (0U)
#define PTC_TDR1_DATA_Msk               (0xFFFFFFFFUL << PTC_TDR1_DATA_Pos)
#define PTC_TDR1_DATA                   PTC_TDR1_DATA_Msk

/******************** Bit definition for PTC_TCR2 register ********************/
#define PTC_TCR2_TRIGSEL_Pos            (0U)
#define PTC_TCR2_TRIGSEL_Msk            (0xFFUL << PTC_TCR2_TRIGSEL_Pos)
#define PTC_TCR2_TRIGSEL                PTC_TCR2_TRIGSEL_Msk
#define PTC_TCR2_OP_Pos                 (16U)
#define PTC_TCR2_OP_Msk                 (0x7UL << PTC_TCR2_OP_Pos)
#define PTC_TCR2_OP                     PTC_TCR2_OP_Msk

/******************** Bit definition for PTC_TAR2 register ********************/
#define PTC_TAR2_ADDR_Pos               (0U)
#define PTC_TAR2_ADDR_Msk               (0xFFFFFFFFUL << PTC_TAR2_ADDR_Pos)
#define PTC_TAR2_ADDR                   PTC_TAR2_ADDR_Msk

/******************** Bit definition for PTC_TDR2 register ********************/
#define PTC_TDR2_DATA_Pos               (0U)
#define PTC_TDR2_DATA_Msk               (0xFFFFFFFFUL << PTC_TDR2_DATA_Pos)
#define PTC_TDR2_DATA                   PTC_TDR2_DATA_Msk

/******************** Bit definition for PTC_TCR3 register ********************/
#define PTC_TCR3_TRIGSEL_Pos            (0U)
#define PTC_TCR3_TRIGSEL_Msk            (0xFFUL << PTC_TCR3_TRIGSEL_Pos)
#define PTC_TCR3_TRIGSEL                PTC_TCR3_TRIGSEL_Msk
#define PTC_TCR3_OP_Pos                 (16U)
#define PTC_TCR3_OP_Msk                 (0x7UL << PTC_TCR3_OP_Pos)
#define PTC_TCR3_OP                     PTC_TCR3_OP_Msk

/******************** Bit definition for PTC_TAR3 register ********************/
#define PTC_TAR3_ADDR_Pos               (0U)
#define PTC_TAR3_ADDR_Msk               (0xFFFFFFFFUL << PTC_TAR3_ADDR_Pos)
#define PTC_TAR3_ADDR                   PTC_TAR3_ADDR_Msk

/******************** Bit definition for PTC_TDR3 register ********************/
#define PTC_TDR3_DATA_Pos               (0U)
#define PTC_TDR3_DATA_Msk               (0xFFFFFFFFUL << PTC_TDR3_DATA_Pos)
#define PTC_TDR3_DATA                   PTC_TDR3_DATA_Msk

/******************** Bit definition for PTC_TCR4 register ********************/
#define PTC_TCR4_TRIGSEL_Pos            (0U)
#define PTC_TCR4_TRIGSEL_Msk            (0xFFUL << PTC_TCR4_TRIGSEL_Pos)
#define PTC_TCR4_TRIGSEL                PTC_TCR4_TRIGSEL_Msk
#define PTC_TCR4_OP_Pos                 (16U)
#define PTC_TCR4_OP_Msk                 (0x7UL << PTC_TCR4_OP_Pos)
#define PTC_TCR4_OP                     PTC_TCR4_OP_Msk

/******************** Bit definition for PTC_TAR4 register ********************/
#define PTC_TAR4_ADDR_Pos               (0U)
#define PTC_TAR4_ADDR_Msk               (0xFFFFFFFFUL << PTC_TAR4_ADDR_Pos)
#define PTC_TAR4_ADDR                   PTC_TAR4_ADDR_Msk

/******************** Bit definition for PTC_TDR4 register ********************/
#define PTC_TDR4_DATA_Pos               (0U)
#define PTC_TDR4_DATA_Msk               (0xFFFFFFFFUL << PTC_TDR4_DATA_Pos)
#define PTC_TDR4_DATA                   PTC_TDR4_DATA_Msk

/******************** Bit definition for PTC_TCR5 register ********************/
#define PTC_TCR5_TRIGSEL_Pos            (0U)
#define PTC_TCR5_TRIGSEL_Msk            (0xFFUL << PTC_TCR5_TRIGSEL_Pos)
#define PTC_TCR5_TRIGSEL                PTC_TCR5_TRIGSEL_Msk
#define PTC_TCR5_OP_Pos                 (16U)
#define PTC_TCR5_OP_Msk                 (0x7UL << PTC_TCR5_OP_Pos)
#define PTC_TCR5_OP                     PTC_TCR5_OP_Msk

/******************** Bit definition for PTC_TAR5 register ********************/
#define PTC_TAR5_ADDR_Pos               (0U)
#define PTC_TAR5_ADDR_Msk               (0xFFFFFFFFUL << PTC_TAR5_ADDR_Pos)
#define PTC_TAR5_ADDR                   PTC_TAR5_ADDR_Msk

/******************** Bit definition for PTC_TDR5 register ********************/
#define PTC_TDR5_DATA_Pos               (0U)
#define PTC_TDR5_DATA_Msk               (0xFFFFFFFFUL << PTC_TDR5_DATA_Pos)
#define PTC_TDR5_DATA                   PTC_TDR5_DATA_Msk

/******************** Bit definition for PTC_TCR6 register ********************/
#define PTC_TCR6_TRIGSEL_Pos            (0U)
#define PTC_TCR6_TRIGSEL_Msk            (0xFFUL << PTC_TCR6_TRIGSEL_Pos)
#define PTC_TCR6_TRIGSEL                PTC_TCR6_TRIGSEL_Msk
#define PTC_TCR6_OP_Pos                 (16U)
#define PTC_TCR6_OP_Msk                 (0x7UL << PTC_TCR6_OP_Pos)
#define PTC_TCR6_OP                     PTC_TCR6_OP_Msk

/******************** Bit definition for PTC_TAR6 register ********************/
#define PTC_TAR6_ADDR_Pos               (0U)
#define PTC_TAR6_ADDR_Msk               (0xFFFFFFFFUL << PTC_TAR6_ADDR_Pos)
#define PTC_TAR6_ADDR                   PTC_TAR6_ADDR_Msk

/******************** Bit definition for PTC_TDR6 register ********************/
#define PTC_TDR6_DATA_Pos               (0U)
#define PTC_TDR6_DATA_Msk               (0xFFFFFFFFUL << PTC_TDR6_DATA_Pos)
#define PTC_TDR6_DATA                   PTC_TDR6_DATA_Msk

/******************** Bit definition for PTC_TCR7 register ********************/
#define PTC_TCR7_TRIGSEL_Pos            (0U)
#define PTC_TCR7_TRIGSEL_Msk            (0xFFUL << PTC_TCR7_TRIGSEL_Pos)
#define PTC_TCR7_TRIGSEL                PTC_TCR7_TRIGSEL_Msk
#define PTC_TCR7_OP_Pos                 (16U)
#define PTC_TCR7_OP_Msk                 (0x7UL << PTC_TCR7_OP_Pos)
#define PTC_TCR7_OP                     PTC_TCR7_OP_Msk

/******************** Bit definition for PTC_TAR7 register ********************/
#define PTC_TAR7_ADDR_Pos               (0U)
#define PTC_TAR7_ADDR_Msk               (0xFFFFFFFFUL << PTC_TAR7_ADDR_Pos)
#define PTC_TAR7_ADDR                   PTC_TAR7_ADDR_Msk

/******************** Bit definition for PTC_TDR7 register ********************/
#define PTC_TDR7_DATA_Pos               (0U)
#define PTC_TDR7_DATA_Msk               (0xFFFFFFFFUL << PTC_TDR7_DATA_Pos)
#define PTC_TDR7_DATA                   PTC_TDR7_DATA_Msk

/******************** Bit definition for PTC_TCR8 register ********************/
#define PTC_TCR8_TRIGSEL_Pos            (0U)
#define PTC_TCR8_TRIGSEL_Msk            (0xFFUL << PTC_TCR8_TRIGSEL_Pos)
#define PTC_TCR8_TRIGSEL                PTC_TCR8_TRIGSEL_Msk
#define PTC_TCR8_OP_Pos                 (16U)
#define PTC_TCR8_OP_Msk                 (0x7UL << PTC_TCR8_OP_Pos)
#define PTC_TCR8_OP                     PTC_TCR8_OP_Msk

/******************** Bit definition for PTC_TAR8 register ********************/
#define PTC_TAR8_ADDR_Pos               (0U)
#define PTC_TAR8_ADDR_Msk               (0xFFFFFFFFUL << PTC_TAR8_ADDR_Pos)
#define PTC_TAR8_ADDR                   PTC_TAR8_ADDR_Msk

/******************** Bit definition for PTC_TDR8 register ********************/
#define PTC_TDR8_DATA_Pos               (0U)
#define PTC_TDR8_DATA_Msk               (0xFFFFFFFFUL << PTC_TDR8_DATA_Pos)
#define PTC_TDR8_DATA                   PTC_TDR8_DATA_Msk

/******************** Bit definition for PTC_MEM1 register ********************/
#define PTC_MEM1_DATA_Pos               (0U)
#define PTC_MEM1_DATA_Msk               (0xFFFFFFFFUL << PTC_MEM1_DATA_Pos)
#define PTC_MEM1_DATA                   PTC_MEM1_DATA_Msk

/******************** Bit definition for PTC_MEM2 register ********************/
#define PTC_MEM2_DATA_Pos               (0U)
#define PTC_MEM2_DATA_Msk               (0xFFFFFFFFUL << PTC_MEM2_DATA_Pos)
#define PTC_MEM2_DATA                   PTC_MEM2_DATA_Msk

/******************** Bit definition for PTC_MEM3 register ********************/
#define PTC_MEM3_DATA_Pos               (0U)
#define PTC_MEM3_DATA_Msk               (0xFFFFFFFFUL << PTC_MEM3_DATA_Pos)
#define PTC_MEM3_DATA                   PTC_MEM3_DATA_Msk

/******************** Bit definition for PTC_MEM4 register ********************/
#define PTC_MEM4_DATA_Pos               (0U)
#define PTC_MEM4_DATA_Msk               (0xFFFFFFFFUL << PTC_MEM4_DATA_Pos)
#define PTC_MEM4_DATA                   PTC_MEM4_DATA_Msk

/****************** Bit definition for PTC_GPIO31_0 register ******************/
#define PTC_GPIO31_0_SELA_Pos           (0U)
#define PTC_GPIO31_0_SELA_Msk           (0x1FUL << PTC_GPIO31_0_SELA_Pos)
#define PTC_GPIO31_0_SELA               PTC_GPIO31_0_SELA_Msk
#define PTC_GPIO31_0_SELB_Pos           (8U)
#define PTC_GPIO31_0_SELB_Msk           (0x1FUL << PTC_GPIO31_0_SELB_Pos)
#define PTC_GPIO31_0_SELB               PTC_GPIO31_0_SELB_Msk
#define PTC_GPIO31_0_SELC_Pos           (16U)
#define PTC_GPIO31_0_SELC_Msk           (0x1FUL << PTC_GPIO31_0_SELC_Pos)
#define PTC_GPIO31_0_SELC               PTC_GPIO31_0_SELC_Msk
#define PTC_GPIO31_0_SELD_Pos           (24U)
#define PTC_GPIO31_0_SELD_Msk           (0x1FUL << PTC_GPIO31_0_SELD_Pos)
#define PTC_GPIO31_0_SELD               PTC_GPIO31_0_SELD_Msk

/***************** Bit definition for PTC_GPIO63_32 register ******************/
#define PTC_GPIO63_32_SELA_Pos          (0U)
#define PTC_GPIO63_32_SELA_Msk          (0x1FUL << PTC_GPIO63_32_SELA_Pos)
#define PTC_GPIO63_32_SELA              PTC_GPIO63_32_SELA_Msk
#define PTC_GPIO63_32_SELB_Pos          (8U)
#define PTC_GPIO63_32_SELB_Msk          (0x1FUL << PTC_GPIO63_32_SELB_Pos)
#define PTC_GPIO63_32_SELB              PTC_GPIO63_32_SELB_Msk
#define PTC_GPIO63_32_SELC_Pos          (16U)
#define PTC_GPIO63_32_SELC_Msk          (0x1FUL << PTC_GPIO63_32_SELC_Pos)
#define PTC_GPIO63_32_SELC              PTC_GPIO63_32_SELC_Msk
#define PTC_GPIO63_32_SELD_Pos          (24U)
#define PTC_GPIO63_32_SELD_Msk          (0x1FUL << PTC_GPIO63_32_SELD_Pos)
#define PTC_GPIO63_32_SELD              PTC_GPIO63_32_SELD_Msk

/***************** Bit definition for PTC_GPIO95_64 register ******************/
#define PTC_GPIO95_64_SELA_Pos          (0U)
#define PTC_GPIO95_64_SELA_Msk          (0x1FUL << PTC_GPIO95_64_SELA_Pos)
#define PTC_GPIO95_64_SELA              PTC_GPIO95_64_SELA_Msk
#define PTC_GPIO95_64_SELB_Pos          (8U)
#define PTC_GPIO95_64_SELB_Msk          (0x1FUL << PTC_GPIO95_64_SELB_Pos)
#define PTC_GPIO95_64_SELB              PTC_GPIO95_64_SELB_Msk
#define PTC_GPIO95_64_SELC_Pos          (16U)
#define PTC_GPIO95_64_SELC_Msk          (0x1FUL << PTC_GPIO95_64_SELC_Pos)
#define PTC_GPIO95_64_SELC              PTC_GPIO95_64_SELC_Msk
#define PTC_GPIO95_64_SELD_Pos          (24U)
#define PTC_GPIO95_64_SELD_Msk          (0x1FUL << PTC_GPIO95_64_SELD_Pos)
#define PTC_GPIO95_64_SELD              PTC_GPIO95_64_SELD_Msk

#endif
