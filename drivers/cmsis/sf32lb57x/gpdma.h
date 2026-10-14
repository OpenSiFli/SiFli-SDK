/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __GPDMA_H
#define __GPDMA_H

typedef struct
{
    __IO uint32_t ISR;
    __IO uint32_t IFCR;
    __IO uint32_t CCR1;
    __IO uint32_t CNDTR1;
    __IO uint32_t CPAR1;
    __IO uint32_t CM0AR1;
    __IO uint32_t CSR1;
    __IO uint32_t CCR2;
    __IO uint32_t CNDTR2;
    __IO uint32_t CPAR2;
    __IO uint32_t CM0AR2;
    __IO uint32_t CSR2;
    __IO uint32_t CCR3;
    __IO uint32_t CNDTR3;
    __IO uint32_t CPAR3;
    __IO uint32_t CM0AR3;
    __IO uint32_t CSR3;
    __IO uint32_t CCR4;
    __IO uint32_t CNDTR4;
    __IO uint32_t CPAR4;
    __IO uint32_t CM0AR4;
    __IO uint32_t CSR4;
    __IO uint32_t CCR5;
    __IO uint32_t CNDTR5;
    __IO uint32_t CPAR5;
    __IO uint32_t CM0AR5;
    __IO uint32_t CSR5;
    __IO uint32_t CCR6;
    __IO uint32_t CNDTR6;
    __IO uint32_t CPAR6;
    __IO uint32_t CM0AR6;
    __IO uint32_t CSR6;
    __IO uint32_t CCR7;
    __IO uint32_t CNDTR7;
    __IO uint32_t CPAR7;
    __IO uint32_t CM0AR7;
    __IO uint32_t CSR7;
    __IO uint32_t CCR8;
    __IO uint32_t CNDTR8;
    __IO uint32_t CPAR8;
    __IO uint32_t CM0AR8;
    __IO uint32_t CSR8;
    __IO uint32_t CSELR1;
    __IO uint32_t CSELR2;
    __IO uint32_t DBGSEL;
    __IO uint32_t SELR1;
    __IO uint32_t SELR2;
    __IO uint32_t SELR3;
    __IO uint32_t SELR4;
    __IO uint32_t SELR5;
    __IO uint32_t SELR6;
    __IO uint32_t SELR7;
    __IO uint32_t SELR8;
    __IO uint32_t LISR;
    __IO uint32_t LIFCR;
    __IO uint32_t CREPR1;
    __IO uint32_t CINCR1;
    __IO uint32_t LCR1;
    __IO uint32_t CREPR2;
    __IO uint32_t CINCR2;
    __IO uint32_t LCR2;
    __IO uint32_t LCR3;
    __IO uint32_t LCR4;
    __IO uint32_t LCR5;
    __IO uint32_t LCR6;
    __IO uint32_t LCR7;
    __IO uint32_t LCR8;
} GPDMA_TypeDef;

typedef struct
{
    __IO uint32_t CCR;         /*!< DMA channel x configuration register        */
    __IO uint32_t CNDTR;       /*!< DMA channel x number of data register       */
    __IO uint32_t CPAR;        /*!< DMA channel x peripheral address register   */
    __IO uint32_t CM0AR;       /*!< DMA channel x memory address register       */
    __IO uint32_t CSR;         /*!< DMA channel x status register               */
} GPDMA_Channel_TypeDef;

typedef struct
{
    __IO uint32_t CSELR1;       /*!< DMA channel selection1 register             */
    __IO uint32_t CSELR2;       /*!< DMA channel selection2 register             */
} GPDMA_Request_TypeDef;

/** ChannelX extention registers which supports repetition */
typedef struct
{
    __IO uint32_t CREPR;        /*!< DMA channel x repetition register           */
    __IO uint32_t CINCR;        /*!< DMA channel x increment register            */
    __IO uint32_t LCR;          /*!< DMA channel x link-list control register    */
} GPDMA_Channel_Ext0TypeDef;

/** ChannelX extention registers which doesn't supports repetition */
typedef struct
{
    __IO uint32_t LCR;          /*!< DMA channel x link-list control register     */
} GPDMA_Channel_Ext1TypeDef;

#define GPDMA_REP_CHANNEL_NUM (2)


/******************* Bit definition for GPDMA_ISR register ********************/
#define GPDMA_ISR_GIF1_Pos              (0U)
#define GPDMA_ISR_GIF1_Msk              (0x1UL << GPDMA_ISR_GIF1_Pos)
#define GPDMA_ISR_GIF1                  GPDMA_ISR_GIF1_Msk
#define GPDMA_ISR_TCIF1_Pos             (1U)
#define GPDMA_ISR_TCIF1_Msk             (0x1UL << GPDMA_ISR_TCIF1_Pos)
#define GPDMA_ISR_TCIF1                 GPDMA_ISR_TCIF1_Msk
#define GPDMA_ISR_HTIF1_Pos             (2U)
#define GPDMA_ISR_HTIF1_Msk             (0x1UL << GPDMA_ISR_HTIF1_Pos)
#define GPDMA_ISR_HTIF1                 GPDMA_ISR_HTIF1_Msk
#define GPDMA_ISR_TEIF1_Pos             (3U)
#define GPDMA_ISR_TEIF1_Msk             (0x1UL << GPDMA_ISR_TEIF1_Pos)
#define GPDMA_ISR_TEIF1                 GPDMA_ISR_TEIF1_Msk
#define GPDMA_ISR_GIF2_Pos              (4U)
#define GPDMA_ISR_GIF2_Msk              (0x1UL << GPDMA_ISR_GIF2_Pos)
#define GPDMA_ISR_GIF2                  GPDMA_ISR_GIF2_Msk
#define GPDMA_ISR_TCIF2_Pos             (5U)
#define GPDMA_ISR_TCIF2_Msk             (0x1UL << GPDMA_ISR_TCIF2_Pos)
#define GPDMA_ISR_TCIF2                 GPDMA_ISR_TCIF2_Msk
#define GPDMA_ISR_HTIF2_Pos             (6U)
#define GPDMA_ISR_HTIF2_Msk             (0x1UL << GPDMA_ISR_HTIF2_Pos)
#define GPDMA_ISR_HTIF2                 GPDMA_ISR_HTIF2_Msk
#define GPDMA_ISR_TEIF2_Pos             (7U)
#define GPDMA_ISR_TEIF2_Msk             (0x1UL << GPDMA_ISR_TEIF2_Pos)
#define GPDMA_ISR_TEIF2                 GPDMA_ISR_TEIF2_Msk
#define GPDMA_ISR_GIF3_Pos              (8U)
#define GPDMA_ISR_GIF3_Msk              (0x1UL << GPDMA_ISR_GIF3_Pos)
#define GPDMA_ISR_GIF3                  GPDMA_ISR_GIF3_Msk
#define GPDMA_ISR_TCIF3_Pos             (9U)
#define GPDMA_ISR_TCIF3_Msk             (0x1UL << GPDMA_ISR_TCIF3_Pos)
#define GPDMA_ISR_TCIF3                 GPDMA_ISR_TCIF3_Msk
#define GPDMA_ISR_HTIF3_Pos             (10U)
#define GPDMA_ISR_HTIF3_Msk             (0x1UL << GPDMA_ISR_HTIF3_Pos)
#define GPDMA_ISR_HTIF3                 GPDMA_ISR_HTIF3_Msk
#define GPDMA_ISR_TEIF3_Pos             (11U)
#define GPDMA_ISR_TEIF3_Msk             (0x1UL << GPDMA_ISR_TEIF3_Pos)
#define GPDMA_ISR_TEIF3                 GPDMA_ISR_TEIF3_Msk
#define GPDMA_ISR_GIF4_Pos              (12U)
#define GPDMA_ISR_GIF4_Msk              (0x1UL << GPDMA_ISR_GIF4_Pos)
#define GPDMA_ISR_GIF4                  GPDMA_ISR_GIF4_Msk
#define GPDMA_ISR_TCIF4_Pos             (13U)
#define GPDMA_ISR_TCIF4_Msk             (0x1UL << GPDMA_ISR_TCIF4_Pos)
#define GPDMA_ISR_TCIF4                 GPDMA_ISR_TCIF4_Msk
#define GPDMA_ISR_HTIF4_Pos             (14U)
#define GPDMA_ISR_HTIF4_Msk             (0x1UL << GPDMA_ISR_HTIF4_Pos)
#define GPDMA_ISR_HTIF4                 GPDMA_ISR_HTIF4_Msk
#define GPDMA_ISR_TEIF4_Pos             (15U)
#define GPDMA_ISR_TEIF4_Msk             (0x1UL << GPDMA_ISR_TEIF4_Pos)
#define GPDMA_ISR_TEIF4                 GPDMA_ISR_TEIF4_Msk
#define GPDMA_ISR_GIF5_Pos              (16U)
#define GPDMA_ISR_GIF5_Msk              (0x1UL << GPDMA_ISR_GIF5_Pos)
#define GPDMA_ISR_GIF5                  GPDMA_ISR_GIF5_Msk
#define GPDMA_ISR_TCIF5_Pos             (17U)
#define GPDMA_ISR_TCIF5_Msk             (0x1UL << GPDMA_ISR_TCIF5_Pos)
#define GPDMA_ISR_TCIF5                 GPDMA_ISR_TCIF5_Msk
#define GPDMA_ISR_HTIF5_Pos             (18U)
#define GPDMA_ISR_HTIF5_Msk             (0x1UL << GPDMA_ISR_HTIF5_Pos)
#define GPDMA_ISR_HTIF5                 GPDMA_ISR_HTIF5_Msk
#define GPDMA_ISR_TEIF5_Pos             (19U)
#define GPDMA_ISR_TEIF5_Msk             (0x1UL << GPDMA_ISR_TEIF5_Pos)
#define GPDMA_ISR_TEIF5                 GPDMA_ISR_TEIF5_Msk
#define GPDMA_ISR_GIF6_Pos              (20U)
#define GPDMA_ISR_GIF6_Msk              (0x1UL << GPDMA_ISR_GIF6_Pos)
#define GPDMA_ISR_GIF6                  GPDMA_ISR_GIF6_Msk
#define GPDMA_ISR_TCIF6_Pos             (21U)
#define GPDMA_ISR_TCIF6_Msk             (0x1UL << GPDMA_ISR_TCIF6_Pos)
#define GPDMA_ISR_TCIF6                 GPDMA_ISR_TCIF6_Msk
#define GPDMA_ISR_HTIF6_Pos             (22U)
#define GPDMA_ISR_HTIF6_Msk             (0x1UL << GPDMA_ISR_HTIF6_Pos)
#define GPDMA_ISR_HTIF6                 GPDMA_ISR_HTIF6_Msk
#define GPDMA_ISR_TEIF6_Pos             (23U)
#define GPDMA_ISR_TEIF6_Msk             (0x1UL << GPDMA_ISR_TEIF6_Pos)
#define GPDMA_ISR_TEIF6                 GPDMA_ISR_TEIF6_Msk
#define GPDMA_ISR_GIF7_Pos              (24U)
#define GPDMA_ISR_GIF7_Msk              (0x1UL << GPDMA_ISR_GIF7_Pos)
#define GPDMA_ISR_GIF7                  GPDMA_ISR_GIF7_Msk
#define GPDMA_ISR_TCIF7_Pos             (25U)
#define GPDMA_ISR_TCIF7_Msk             (0x1UL << GPDMA_ISR_TCIF7_Pos)
#define GPDMA_ISR_TCIF7                 GPDMA_ISR_TCIF7_Msk
#define GPDMA_ISR_HTIF7_Pos             (26U)
#define GPDMA_ISR_HTIF7_Msk             (0x1UL << GPDMA_ISR_HTIF7_Pos)
#define GPDMA_ISR_HTIF7                 GPDMA_ISR_HTIF7_Msk
#define GPDMA_ISR_TEIF7_Pos             (27U)
#define GPDMA_ISR_TEIF7_Msk             (0x1UL << GPDMA_ISR_TEIF7_Pos)
#define GPDMA_ISR_TEIF7                 GPDMA_ISR_TEIF7_Msk
#define GPDMA_ISR_GIF8_Pos              (28U)
#define GPDMA_ISR_GIF8_Msk              (0x1UL << GPDMA_ISR_GIF8_Pos)
#define GPDMA_ISR_GIF8                  GPDMA_ISR_GIF8_Msk
#define GPDMA_ISR_TCIF8_Pos             (29U)
#define GPDMA_ISR_TCIF8_Msk             (0x1UL << GPDMA_ISR_TCIF8_Pos)
#define GPDMA_ISR_TCIF8                 GPDMA_ISR_TCIF8_Msk
#define GPDMA_ISR_HTIF8_Pos             (30U)
#define GPDMA_ISR_HTIF8_Msk             (0x1UL << GPDMA_ISR_HTIF8_Pos)
#define GPDMA_ISR_HTIF8                 GPDMA_ISR_HTIF8_Msk
#define GPDMA_ISR_TEIF8_Pos             (31U)
#define GPDMA_ISR_TEIF8_Msk             (0x1UL << GPDMA_ISR_TEIF8_Pos)
#define GPDMA_ISR_TEIF8                 GPDMA_ISR_TEIF8_Msk

/******************* Bit definition for GPDMA_IFCR register *******************/
#define GPDMA_IFCR_CGIF1_Pos            (0U)
#define GPDMA_IFCR_CGIF1_Msk            (0x1UL << GPDMA_IFCR_CGIF1_Pos)
#define GPDMA_IFCR_CGIF1                GPDMA_IFCR_CGIF1_Msk
#define GPDMA_IFCR_CTCIF1_Pos           (1U)
#define GPDMA_IFCR_CTCIF1_Msk           (0x1UL << GPDMA_IFCR_CTCIF1_Pos)
#define GPDMA_IFCR_CTCIF1               GPDMA_IFCR_CTCIF1_Msk
#define GPDMA_IFCR_CHTIF1_Pos           (2U)
#define GPDMA_IFCR_CHTIF1_Msk           (0x1UL << GPDMA_IFCR_CHTIF1_Pos)
#define GPDMA_IFCR_CHTIF1               GPDMA_IFCR_CHTIF1_Msk
#define GPDMA_IFCR_CTEIF1_Pos           (3U)
#define GPDMA_IFCR_CTEIF1_Msk           (0x1UL << GPDMA_IFCR_CTEIF1_Pos)
#define GPDMA_IFCR_CTEIF1               GPDMA_IFCR_CTEIF1_Msk
#define GPDMA_IFCR_CGIF2_Pos            (4U)
#define GPDMA_IFCR_CGIF2_Msk            (0x1UL << GPDMA_IFCR_CGIF2_Pos)
#define GPDMA_IFCR_CGIF2                GPDMA_IFCR_CGIF2_Msk
#define GPDMA_IFCR_CTCIF2_Pos           (5U)
#define GPDMA_IFCR_CTCIF2_Msk           (0x1UL << GPDMA_IFCR_CTCIF2_Pos)
#define GPDMA_IFCR_CTCIF2               GPDMA_IFCR_CTCIF2_Msk
#define GPDMA_IFCR_CHTIF2_Pos           (6U)
#define GPDMA_IFCR_CHTIF2_Msk           (0x1UL << GPDMA_IFCR_CHTIF2_Pos)
#define GPDMA_IFCR_CHTIF2               GPDMA_IFCR_CHTIF2_Msk
#define GPDMA_IFCR_CTEIF2_Pos           (7U)
#define GPDMA_IFCR_CTEIF2_Msk           (0x1UL << GPDMA_IFCR_CTEIF2_Pos)
#define GPDMA_IFCR_CTEIF2               GPDMA_IFCR_CTEIF2_Msk
#define GPDMA_IFCR_CGIF3_Pos            (8U)
#define GPDMA_IFCR_CGIF3_Msk            (0x1UL << GPDMA_IFCR_CGIF3_Pos)
#define GPDMA_IFCR_CGIF3                GPDMA_IFCR_CGIF3_Msk
#define GPDMA_IFCR_CTCIF3_Pos           (9U)
#define GPDMA_IFCR_CTCIF3_Msk           (0x1UL << GPDMA_IFCR_CTCIF3_Pos)
#define GPDMA_IFCR_CTCIF3               GPDMA_IFCR_CTCIF3_Msk
#define GPDMA_IFCR_CHTIF3_Pos           (10U)
#define GPDMA_IFCR_CHTIF3_Msk           (0x1UL << GPDMA_IFCR_CHTIF3_Pos)
#define GPDMA_IFCR_CHTIF3               GPDMA_IFCR_CHTIF3_Msk
#define GPDMA_IFCR_CTEIF3_Pos           (11U)
#define GPDMA_IFCR_CTEIF3_Msk           (0x1UL << GPDMA_IFCR_CTEIF3_Pos)
#define GPDMA_IFCR_CTEIF3               GPDMA_IFCR_CTEIF3_Msk
#define GPDMA_IFCR_CGIF4_Pos            (12U)
#define GPDMA_IFCR_CGIF4_Msk            (0x1UL << GPDMA_IFCR_CGIF4_Pos)
#define GPDMA_IFCR_CGIF4                GPDMA_IFCR_CGIF4_Msk
#define GPDMA_IFCR_CTCIF4_Pos           (13U)
#define GPDMA_IFCR_CTCIF4_Msk           (0x1UL << GPDMA_IFCR_CTCIF4_Pos)
#define GPDMA_IFCR_CTCIF4               GPDMA_IFCR_CTCIF4_Msk
#define GPDMA_IFCR_CHTIF4_Pos           (14U)
#define GPDMA_IFCR_CHTIF4_Msk           (0x1UL << GPDMA_IFCR_CHTIF4_Pos)
#define GPDMA_IFCR_CHTIF4               GPDMA_IFCR_CHTIF4_Msk
#define GPDMA_IFCR_CTEIF4_Pos           (15U)
#define GPDMA_IFCR_CTEIF4_Msk           (0x1UL << GPDMA_IFCR_CTEIF4_Pos)
#define GPDMA_IFCR_CTEIF4               GPDMA_IFCR_CTEIF4_Msk
#define GPDMA_IFCR_CGIF5_Pos            (16U)
#define GPDMA_IFCR_CGIF5_Msk            (0x1UL << GPDMA_IFCR_CGIF5_Pos)
#define GPDMA_IFCR_CGIF5                GPDMA_IFCR_CGIF5_Msk
#define GPDMA_IFCR_CTCIF5_Pos           (17U)
#define GPDMA_IFCR_CTCIF5_Msk           (0x1UL << GPDMA_IFCR_CTCIF5_Pos)
#define GPDMA_IFCR_CTCIF5               GPDMA_IFCR_CTCIF5_Msk
#define GPDMA_IFCR_CHTIF5_Pos           (18U)
#define GPDMA_IFCR_CHTIF5_Msk           (0x1UL << GPDMA_IFCR_CHTIF5_Pos)
#define GPDMA_IFCR_CHTIF5               GPDMA_IFCR_CHTIF5_Msk
#define GPDMA_IFCR_CTEIF5_Pos           (19U)
#define GPDMA_IFCR_CTEIF5_Msk           (0x1UL << GPDMA_IFCR_CTEIF5_Pos)
#define GPDMA_IFCR_CTEIF5               GPDMA_IFCR_CTEIF5_Msk
#define GPDMA_IFCR_CGIF6_Pos            (20U)
#define GPDMA_IFCR_CGIF6_Msk            (0x1UL << GPDMA_IFCR_CGIF6_Pos)
#define GPDMA_IFCR_CGIF6                GPDMA_IFCR_CGIF6_Msk
#define GPDMA_IFCR_CTCIF6_Pos           (21U)
#define GPDMA_IFCR_CTCIF6_Msk           (0x1UL << GPDMA_IFCR_CTCIF6_Pos)
#define GPDMA_IFCR_CTCIF6               GPDMA_IFCR_CTCIF6_Msk
#define GPDMA_IFCR_CHTIF6_Pos           (22U)
#define GPDMA_IFCR_CHTIF6_Msk           (0x1UL << GPDMA_IFCR_CHTIF6_Pos)
#define GPDMA_IFCR_CHTIF6               GPDMA_IFCR_CHTIF6_Msk
#define GPDMA_IFCR_CTEIF6_Pos           (23U)
#define GPDMA_IFCR_CTEIF6_Msk           (0x1UL << GPDMA_IFCR_CTEIF6_Pos)
#define GPDMA_IFCR_CTEIF6               GPDMA_IFCR_CTEIF6_Msk
#define GPDMA_IFCR_CGIF7_Pos            (24U)
#define GPDMA_IFCR_CGIF7_Msk            (0x1UL << GPDMA_IFCR_CGIF7_Pos)
#define GPDMA_IFCR_CGIF7                GPDMA_IFCR_CGIF7_Msk
#define GPDMA_IFCR_CTCIF7_Pos           (25U)
#define GPDMA_IFCR_CTCIF7_Msk           (0x1UL << GPDMA_IFCR_CTCIF7_Pos)
#define GPDMA_IFCR_CTCIF7               GPDMA_IFCR_CTCIF7_Msk
#define GPDMA_IFCR_CHTIF7_Pos           (26U)
#define GPDMA_IFCR_CHTIF7_Msk           (0x1UL << GPDMA_IFCR_CHTIF7_Pos)
#define GPDMA_IFCR_CHTIF7               GPDMA_IFCR_CHTIF7_Msk
#define GPDMA_IFCR_CTEIF7_Pos           (27U)
#define GPDMA_IFCR_CTEIF7_Msk           (0x1UL << GPDMA_IFCR_CTEIF7_Pos)
#define GPDMA_IFCR_CTEIF7               GPDMA_IFCR_CTEIF7_Msk
#define GPDMA_IFCR_CGIF8_Pos            (28U)
#define GPDMA_IFCR_CGIF8_Msk            (0x1UL << GPDMA_IFCR_CGIF8_Pos)
#define GPDMA_IFCR_CGIF8                GPDMA_IFCR_CGIF8_Msk
#define GPDMA_IFCR_CTCIF8_Pos           (29U)
#define GPDMA_IFCR_CTCIF8_Msk           (0x1UL << GPDMA_IFCR_CTCIF8_Pos)
#define GPDMA_IFCR_CTCIF8               GPDMA_IFCR_CTCIF8_Msk
#define GPDMA_IFCR_CHTIF8_Pos           (30U)
#define GPDMA_IFCR_CHTIF8_Msk           (0x1UL << GPDMA_IFCR_CHTIF8_Pos)
#define GPDMA_IFCR_CHTIF8               GPDMA_IFCR_CHTIF8_Msk
#define GPDMA_IFCR_CTEIF8_Pos           (31U)
#define GPDMA_IFCR_CTEIF8_Msk           (0x1UL << GPDMA_IFCR_CTEIF8_Pos)
#define GPDMA_IFCR_CTEIF8               GPDMA_IFCR_CTEIF8_Msk

/******************* Bit definition for GPDMA_CCR1 register *******************/
#define GPDMA_CCR1_EN_Pos               (0U)
#define GPDMA_CCR1_EN_Msk               (0x1UL << GPDMA_CCR1_EN_Pos)
#define GPDMA_CCR1_EN                   GPDMA_CCR1_EN_Msk
#define GPDMA_CCR1_TCIE_Pos             (1U)
#define GPDMA_CCR1_TCIE_Msk             (0x1UL << GPDMA_CCR1_TCIE_Pos)
#define GPDMA_CCR1_TCIE                 GPDMA_CCR1_TCIE_Msk
#define GPDMA_CCR1_HTIE_Pos             (2U)
#define GPDMA_CCR1_HTIE_Msk             (0x1UL << GPDMA_CCR1_HTIE_Pos)
#define GPDMA_CCR1_HTIE                 GPDMA_CCR1_HTIE_Msk
#define GPDMA_CCR1_TEIE_Pos             (3U)
#define GPDMA_CCR1_TEIE_Msk             (0x1UL << GPDMA_CCR1_TEIE_Pos)
#define GPDMA_CCR1_TEIE                 GPDMA_CCR1_TEIE_Msk
#define GPDMA_CCR1_DIR_Pos              (4U)
#define GPDMA_CCR1_DIR_Msk              (0x1UL << GPDMA_CCR1_DIR_Pos)
#define GPDMA_CCR1_DIR                  GPDMA_CCR1_DIR_Msk
#define GPDMA_CCR1_CIRC_Pos             (5U)
#define GPDMA_CCR1_CIRC_Msk             (0x1UL << GPDMA_CCR1_CIRC_Pos)
#define GPDMA_CCR1_CIRC                 GPDMA_CCR1_CIRC_Msk
#define GPDMA_CCR1_PINC_Pos             (6U)
#define GPDMA_CCR1_PINC_Msk             (0x1UL << GPDMA_CCR1_PINC_Pos)
#define GPDMA_CCR1_PINC                 GPDMA_CCR1_PINC_Msk
#define GPDMA_CCR1_MINC_Pos             (7U)
#define GPDMA_CCR1_MINC_Msk             (0x1UL << GPDMA_CCR1_MINC_Pos)
#define GPDMA_CCR1_MINC                 GPDMA_CCR1_MINC_Msk
#define GPDMA_CCR1_PSIZE_Pos            (8U)
#define GPDMA_CCR1_PSIZE_Msk            (0x3UL << GPDMA_CCR1_PSIZE_Pos)
#define GPDMA_CCR1_PSIZE                GPDMA_CCR1_PSIZE_Msk
#define GPDMA_CCR1_MSIZE_Pos            (10U)
#define GPDMA_CCR1_MSIZE_Msk            (0x3UL << GPDMA_CCR1_MSIZE_Pos)
#define GPDMA_CCR1_MSIZE                GPDMA_CCR1_MSIZE_Msk
#define GPDMA_CCR1_PL_Pos               (12U)
#define GPDMA_CCR1_PL_Msk               (0x3UL << GPDMA_CCR1_PL_Pos)
#define GPDMA_CCR1_PL                   GPDMA_CCR1_PL_Msk
#define GPDMA_CCR1_MEM2MEM_Pos          (14U)
#define GPDMA_CCR1_MEM2MEM_Msk          (0x1UL << GPDMA_CCR1_MEM2MEM_Pos)
#define GPDMA_CCR1_MEM2MEM              GPDMA_CCR1_MEM2MEM_Msk
#define GPDMA_CCR1_LCIE_Pos             (15U)
#define GPDMA_CCR1_LCIE_Msk             (0x1UL << GPDMA_CCR1_LCIE_Pos)
#define GPDMA_CCR1_LCIE                 GPDMA_CCR1_LCIE_Msk
#define GPDMA_CCR1_SIGN_Pos             (16U)
#define GPDMA_CCR1_SIGN_Msk             (0x1UL << GPDMA_CCR1_SIGN_Pos)
#define GPDMA_CCR1_SIGN                 GPDMA_CCR1_SIGN_Msk
#define GPDMA_CCR1_PACK_Pos             (17U)
#define GPDMA_CCR1_PACK_Msk             (0x1UL << GPDMA_CCR1_PACK_Pos)
#define GPDMA_CCR1_PACK                 GPDMA_CCR1_PACK_Msk
#define GPDMA_CCR1_DHX_Pos              (18U)
#define GPDMA_CCR1_DHX_Msk              (0x1UL << GPDMA_CCR1_DHX_Pos)
#define GPDMA_CCR1_DHX                  GPDMA_CCR1_DHX_Msk
#define GPDMA_CCR1_DBX_Pos              (19U)
#define GPDMA_CCR1_DBX_Msk              (0x1UL << GPDMA_CCR1_DBX_Pos)
#define GPDMA_CCR1_DBX                  GPDMA_CCR1_DBX_Msk
#define GPDMA_CCR1_SBS_Pos              (20U)
#define GPDMA_CCR1_SBS_Msk              (0x1UL << GPDMA_CCR1_SBS_Pos)
#define GPDMA_CCR1_SBS                  GPDMA_CCR1_SBS_Msk
#define GPDMA_CCR1_SBX_Pos              (21U)
#define GPDMA_CCR1_SBX_Msk              (0x1UL << GPDMA_CCR1_SBX_Pos)
#define GPDMA_CCR1_SBX                  GPDMA_CCR1_SBX_Msk
#define GPDMA_CCR1_TPOL_Pos             (22U)
#define GPDMA_CCR1_TPOL_Msk             (0x1UL << GPDMA_CCR1_TPOL_Pos)
#define GPDMA_CCR1_TPOL                 GPDMA_CCR1_TPOL_Msk
#define GPDMA_CCR1_TEDGE_Pos            (23U)
#define GPDMA_CCR1_TEDGE_Msk            (0x1UL << GPDMA_CCR1_TEDGE_Pos)
#define GPDMA_CCR1_TEDGE                GPDMA_CCR1_TEDGE_Msk
#define GPDMA_CCR1_DBURST_Pos           (24U)
#define GPDMA_CCR1_DBURST_Msk           (0x3UL << GPDMA_CCR1_DBURST_Pos)
#define GPDMA_CCR1_DBURST               GPDMA_CCR1_DBURST_Msk
#define GPDMA_CCR1_SBURST_Pos           (26U)
#define GPDMA_CCR1_SBURST_Msk           (0x3UL << GPDMA_CCR1_SBURST_Pos)
#define GPDMA_CCR1_SBURST               GPDMA_CCR1_SBURST_Msk
#define GPDMA_CCR1_DFIX_Pos             (28U)
#define GPDMA_CCR1_DFIX_Msk             (0x1UL << GPDMA_CCR1_DFIX_Pos)
#define GPDMA_CCR1_DFIX                 GPDMA_CCR1_DFIX_Msk
#define GPDMA_CCR1_SFIX_Pos             (29U)
#define GPDMA_CCR1_SFIX_Msk             (0x1UL << GPDMA_CCR1_SFIX_Pos)
#define GPDMA_CCR1_SFIX                 GPDMA_CCR1_SFIX_Msk

/****************** Bit definition for GPDMA_CNDTR1 register ******************/
#define GPDMA_CNDTR1_NDT_Pos            (0U)
#define GPDMA_CNDTR1_NDT_Msk            (0xFFFFFUL << GPDMA_CNDTR1_NDT_Pos)
#define GPDMA_CNDTR1_NDT                GPDMA_CNDTR1_NDT_Msk

/****************** Bit definition for GPDMA_CPAR1 register *******************/
#define GPDMA_CPAR1_PA_Pos              (0U)
#define GPDMA_CPAR1_PA_Msk              (0xFFFFFFFFUL << GPDMA_CPAR1_PA_Pos)
#define GPDMA_CPAR1_PA                  GPDMA_CPAR1_PA_Msk

/****************** Bit definition for GPDMA_CM0AR1 register ******************/
#define GPDMA_CM0AR1_MA_Pos             (0U)
#define GPDMA_CM0AR1_MA_Msk             (0xFFFFFFFFUL << GPDMA_CM0AR1_MA_Pos)
#define GPDMA_CM0AR1_MA                 GPDMA_CM0AR1_MA_Msk

/******************* Bit definition for GPDMA_CSR1 register *******************/
#define GPDMA_CSR1_DSTNDT_Pos           (0U)
#define GPDMA_CSR1_DSTNDT_Msk           (0xFFFFFUL << GPDMA_CSR1_DSTNDT_Pos)
#define GPDMA_CSR1_DSTNDT               GPDMA_CSR1_DSTNDT_Msk
#define GPDMA_CSR1_BUSERR_Pos           (20U)
#define GPDMA_CSR1_BUSERR_Msk           (0x1UL << GPDMA_CSR1_BUSERR_Pos)
#define GPDMA_CSR1_BUSERR               GPDMA_CSR1_BUSERR_Msk
#define GPDMA_CSR1_CFGERR_Pos           (21U)
#define GPDMA_CSR1_CFGERR_Msk           (0x1UL << GPDMA_CSR1_CFGERR_Pos)
#define GPDMA_CSR1_CFGERR               GPDMA_CSR1_CFGERR_Msk
#define GPDMA_CSR1_LCNT_Pos             (22U)
#define GPDMA_CSR1_LCNT_Msk             (0x3FFUL << GPDMA_CSR1_LCNT_Pos)
#define GPDMA_CSR1_LCNT                 GPDMA_CSR1_LCNT_Msk

/******************* Bit definition for GPDMA_CCR2 register *******************/
#define GPDMA_CCR2_EN_Pos               (0U)
#define GPDMA_CCR2_EN_Msk               (0x1UL << GPDMA_CCR2_EN_Pos)
#define GPDMA_CCR2_EN                   GPDMA_CCR2_EN_Msk
#define GPDMA_CCR2_TCIE_Pos             (1U)
#define GPDMA_CCR2_TCIE_Msk             (0x1UL << GPDMA_CCR2_TCIE_Pos)
#define GPDMA_CCR2_TCIE                 GPDMA_CCR2_TCIE_Msk
#define GPDMA_CCR2_HTIE_Pos             (2U)
#define GPDMA_CCR2_HTIE_Msk             (0x1UL << GPDMA_CCR2_HTIE_Pos)
#define GPDMA_CCR2_HTIE                 GPDMA_CCR2_HTIE_Msk
#define GPDMA_CCR2_TEIE_Pos             (3U)
#define GPDMA_CCR2_TEIE_Msk             (0x1UL << GPDMA_CCR2_TEIE_Pos)
#define GPDMA_CCR2_TEIE                 GPDMA_CCR2_TEIE_Msk
#define GPDMA_CCR2_DIR_Pos              (4U)
#define GPDMA_CCR2_DIR_Msk              (0x1UL << GPDMA_CCR2_DIR_Pos)
#define GPDMA_CCR2_DIR                  GPDMA_CCR2_DIR_Msk
#define GPDMA_CCR2_CIRC_Pos             (5U)
#define GPDMA_CCR2_CIRC_Msk             (0x1UL << GPDMA_CCR2_CIRC_Pos)
#define GPDMA_CCR2_CIRC                 GPDMA_CCR2_CIRC_Msk
#define GPDMA_CCR2_PINC_Pos             (6U)
#define GPDMA_CCR2_PINC_Msk             (0x1UL << GPDMA_CCR2_PINC_Pos)
#define GPDMA_CCR2_PINC                 GPDMA_CCR2_PINC_Msk
#define GPDMA_CCR2_MINC_Pos             (7U)
#define GPDMA_CCR2_MINC_Msk             (0x1UL << GPDMA_CCR2_MINC_Pos)
#define GPDMA_CCR2_MINC                 GPDMA_CCR2_MINC_Msk
#define GPDMA_CCR2_PSIZE_Pos            (8U)
#define GPDMA_CCR2_PSIZE_Msk            (0x3UL << GPDMA_CCR2_PSIZE_Pos)
#define GPDMA_CCR2_PSIZE                GPDMA_CCR2_PSIZE_Msk
#define GPDMA_CCR2_MSIZE_Pos            (10U)
#define GPDMA_CCR2_MSIZE_Msk            (0x3UL << GPDMA_CCR2_MSIZE_Pos)
#define GPDMA_CCR2_MSIZE                GPDMA_CCR2_MSIZE_Msk
#define GPDMA_CCR2_PL_Pos               (12U)
#define GPDMA_CCR2_PL_Msk               (0x3UL << GPDMA_CCR2_PL_Pos)
#define GPDMA_CCR2_PL                   GPDMA_CCR2_PL_Msk
#define GPDMA_CCR2_MEM2MEM_Pos          (14U)
#define GPDMA_CCR2_MEM2MEM_Msk          (0x1UL << GPDMA_CCR2_MEM2MEM_Pos)
#define GPDMA_CCR2_MEM2MEM              GPDMA_CCR2_MEM2MEM_Msk
#define GPDMA_CCR2_LCIE_Pos             (15U)
#define GPDMA_CCR2_LCIE_Msk             (0x1UL << GPDMA_CCR2_LCIE_Pos)
#define GPDMA_CCR2_LCIE                 GPDMA_CCR2_LCIE_Msk
#define GPDMA_CCR2_SIGN_Pos             (16U)
#define GPDMA_CCR2_SIGN_Msk             (0x1UL << GPDMA_CCR2_SIGN_Pos)
#define GPDMA_CCR2_SIGN                 GPDMA_CCR2_SIGN_Msk
#define GPDMA_CCR2_PACK_Pos             (17U)
#define GPDMA_CCR2_PACK_Msk             (0x1UL << GPDMA_CCR2_PACK_Pos)
#define GPDMA_CCR2_PACK                 GPDMA_CCR2_PACK_Msk
#define GPDMA_CCR2_DHX_Pos              (18U)
#define GPDMA_CCR2_DHX_Msk              (0x1UL << GPDMA_CCR2_DHX_Pos)
#define GPDMA_CCR2_DHX                  GPDMA_CCR2_DHX_Msk
#define GPDMA_CCR2_DBX_Pos              (19U)
#define GPDMA_CCR2_DBX_Msk              (0x1UL << GPDMA_CCR2_DBX_Pos)
#define GPDMA_CCR2_DBX                  GPDMA_CCR2_DBX_Msk
#define GPDMA_CCR2_SBS_Pos              (20U)
#define GPDMA_CCR2_SBS_Msk              (0x1UL << GPDMA_CCR2_SBS_Pos)
#define GPDMA_CCR2_SBS                  GPDMA_CCR2_SBS_Msk
#define GPDMA_CCR2_SBX_Pos              (21U)
#define GPDMA_CCR2_SBX_Msk              (0x1UL << GPDMA_CCR2_SBX_Pos)
#define GPDMA_CCR2_SBX                  GPDMA_CCR2_SBX_Msk
#define GPDMA_CCR2_TPOL_Pos             (22U)
#define GPDMA_CCR2_TPOL_Msk             (0x1UL << GPDMA_CCR2_TPOL_Pos)
#define GPDMA_CCR2_TPOL                 GPDMA_CCR2_TPOL_Msk
#define GPDMA_CCR2_TEDGE_Pos            (23U)
#define GPDMA_CCR2_TEDGE_Msk            (0x1UL << GPDMA_CCR2_TEDGE_Pos)
#define GPDMA_CCR2_TEDGE                GPDMA_CCR2_TEDGE_Msk
#define GPDMA_CCR2_DBURST_Pos           (24U)
#define GPDMA_CCR2_DBURST_Msk           (0x3UL << GPDMA_CCR2_DBURST_Pos)
#define GPDMA_CCR2_DBURST               GPDMA_CCR2_DBURST_Msk
#define GPDMA_CCR2_SBURST_Pos           (26U)
#define GPDMA_CCR2_SBURST_Msk           (0x3UL << GPDMA_CCR2_SBURST_Pos)
#define GPDMA_CCR2_SBURST               GPDMA_CCR2_SBURST_Msk
#define GPDMA_CCR2_DFIX_Pos             (28U)
#define GPDMA_CCR2_DFIX_Msk             (0x1UL << GPDMA_CCR2_DFIX_Pos)
#define GPDMA_CCR2_DFIX                 GPDMA_CCR2_DFIX_Msk
#define GPDMA_CCR2_SFIX_Pos             (29U)
#define GPDMA_CCR2_SFIX_Msk             (0x1UL << GPDMA_CCR2_SFIX_Pos)
#define GPDMA_CCR2_SFIX                 GPDMA_CCR2_SFIX_Msk

/****************** Bit definition for GPDMA_CNDTR2 register ******************/
#define GPDMA_CNDTR2_NDT_Pos            (0U)
#define GPDMA_CNDTR2_NDT_Msk            (0xFFFFFUL << GPDMA_CNDTR2_NDT_Pos)
#define GPDMA_CNDTR2_NDT                GPDMA_CNDTR2_NDT_Msk

/****************** Bit definition for GPDMA_CPAR2 register *******************/
#define GPDMA_CPAR2_PA_Pos              (0U)
#define GPDMA_CPAR2_PA_Msk              (0xFFFFFFFFUL << GPDMA_CPAR2_PA_Pos)
#define GPDMA_CPAR2_PA                  GPDMA_CPAR2_PA_Msk

/****************** Bit definition for GPDMA_CM0AR2 register ******************/
#define GPDMA_CM0AR2_MA_Pos             (0U)
#define GPDMA_CM0AR2_MA_Msk             (0xFFFFFFFFUL << GPDMA_CM0AR2_MA_Pos)
#define GPDMA_CM0AR2_MA                 GPDMA_CM0AR2_MA_Msk

/******************* Bit definition for GPDMA_CSR2 register *******************/
#define GPDMA_CSR2_DSTNDT_Pos           (0U)
#define GPDMA_CSR2_DSTNDT_Msk           (0xFFFFFUL << GPDMA_CSR2_DSTNDT_Pos)
#define GPDMA_CSR2_DSTNDT               GPDMA_CSR2_DSTNDT_Msk
#define GPDMA_CSR2_BUSERR_Pos           (20U)
#define GPDMA_CSR2_BUSERR_Msk           (0x1UL << GPDMA_CSR2_BUSERR_Pos)
#define GPDMA_CSR2_BUSERR               GPDMA_CSR2_BUSERR_Msk
#define GPDMA_CSR2_CFGERR_Pos           (21U)
#define GPDMA_CSR2_CFGERR_Msk           (0x1UL << GPDMA_CSR2_CFGERR_Pos)
#define GPDMA_CSR2_CFGERR               GPDMA_CSR2_CFGERR_Msk
#define GPDMA_CSR2_LCNT_Pos             (22U)
#define GPDMA_CSR2_LCNT_Msk             (0x3FFUL << GPDMA_CSR2_LCNT_Pos)
#define GPDMA_CSR2_LCNT                 GPDMA_CSR2_LCNT_Msk

/******************* Bit definition for GPDMA_CCR3 register *******************/
#define GPDMA_CCR3_EN_Pos               (0U)
#define GPDMA_CCR3_EN_Msk               (0x1UL << GPDMA_CCR3_EN_Pos)
#define GPDMA_CCR3_EN                   GPDMA_CCR3_EN_Msk
#define GPDMA_CCR3_TCIE_Pos             (1U)
#define GPDMA_CCR3_TCIE_Msk             (0x1UL << GPDMA_CCR3_TCIE_Pos)
#define GPDMA_CCR3_TCIE                 GPDMA_CCR3_TCIE_Msk
#define GPDMA_CCR3_HTIE_Pos             (2U)
#define GPDMA_CCR3_HTIE_Msk             (0x1UL << GPDMA_CCR3_HTIE_Pos)
#define GPDMA_CCR3_HTIE                 GPDMA_CCR3_HTIE_Msk
#define GPDMA_CCR3_TEIE_Pos             (3U)
#define GPDMA_CCR3_TEIE_Msk             (0x1UL << GPDMA_CCR3_TEIE_Pos)
#define GPDMA_CCR3_TEIE                 GPDMA_CCR3_TEIE_Msk
#define GPDMA_CCR3_DIR_Pos              (4U)
#define GPDMA_CCR3_DIR_Msk              (0x1UL << GPDMA_CCR3_DIR_Pos)
#define GPDMA_CCR3_DIR                  GPDMA_CCR3_DIR_Msk
#define GPDMA_CCR3_CIRC_Pos             (5U)
#define GPDMA_CCR3_CIRC_Msk             (0x1UL << GPDMA_CCR3_CIRC_Pos)
#define GPDMA_CCR3_CIRC                 GPDMA_CCR3_CIRC_Msk
#define GPDMA_CCR3_PINC_Pos             (6U)
#define GPDMA_CCR3_PINC_Msk             (0x1UL << GPDMA_CCR3_PINC_Pos)
#define GPDMA_CCR3_PINC                 GPDMA_CCR3_PINC_Msk
#define GPDMA_CCR3_MINC_Pos             (7U)
#define GPDMA_CCR3_MINC_Msk             (0x1UL << GPDMA_CCR3_MINC_Pos)
#define GPDMA_CCR3_MINC                 GPDMA_CCR3_MINC_Msk
#define GPDMA_CCR3_PSIZE_Pos            (8U)
#define GPDMA_CCR3_PSIZE_Msk            (0x3UL << GPDMA_CCR3_PSIZE_Pos)
#define GPDMA_CCR3_PSIZE                GPDMA_CCR3_PSIZE_Msk
#define GPDMA_CCR3_MSIZE_Pos            (10U)
#define GPDMA_CCR3_MSIZE_Msk            (0x3UL << GPDMA_CCR3_MSIZE_Pos)
#define GPDMA_CCR3_MSIZE                GPDMA_CCR3_MSIZE_Msk
#define GPDMA_CCR3_PL_Pos               (12U)
#define GPDMA_CCR3_PL_Msk               (0x3UL << GPDMA_CCR3_PL_Pos)
#define GPDMA_CCR3_PL                   GPDMA_CCR3_PL_Msk
#define GPDMA_CCR3_MEM2MEM_Pos          (14U)
#define GPDMA_CCR3_MEM2MEM_Msk          (0x1UL << GPDMA_CCR3_MEM2MEM_Pos)
#define GPDMA_CCR3_MEM2MEM              GPDMA_CCR3_MEM2MEM_Msk
#define GPDMA_CCR3_LCIE_Pos             (15U)
#define GPDMA_CCR3_LCIE_Msk             (0x1UL << GPDMA_CCR3_LCIE_Pos)
#define GPDMA_CCR3_LCIE                 GPDMA_CCR3_LCIE_Msk
#define GPDMA_CCR3_SIGN_Pos             (16U)
#define GPDMA_CCR3_SIGN_Msk             (0x1UL << GPDMA_CCR3_SIGN_Pos)
#define GPDMA_CCR3_SIGN                 GPDMA_CCR3_SIGN_Msk
#define GPDMA_CCR3_PACK_Pos             (17U)
#define GPDMA_CCR3_PACK_Msk             (0x1UL << GPDMA_CCR3_PACK_Pos)
#define GPDMA_CCR3_PACK                 GPDMA_CCR3_PACK_Msk
#define GPDMA_CCR3_DHX_Pos              (18U)
#define GPDMA_CCR3_DHX_Msk              (0x1UL << GPDMA_CCR3_DHX_Pos)
#define GPDMA_CCR3_DHX                  GPDMA_CCR3_DHX_Msk
#define GPDMA_CCR3_DBX_Pos              (19U)
#define GPDMA_CCR3_DBX_Msk              (0x1UL << GPDMA_CCR3_DBX_Pos)
#define GPDMA_CCR3_DBX                  GPDMA_CCR3_DBX_Msk
#define GPDMA_CCR3_SBS_Pos              (20U)
#define GPDMA_CCR3_SBS_Msk              (0x1UL << GPDMA_CCR3_SBS_Pos)
#define GPDMA_CCR3_SBS                  GPDMA_CCR3_SBS_Msk
#define GPDMA_CCR3_SBX_Pos              (21U)
#define GPDMA_CCR3_SBX_Msk              (0x1UL << GPDMA_CCR3_SBX_Pos)
#define GPDMA_CCR3_SBX                  GPDMA_CCR3_SBX_Msk
#define GPDMA_CCR3_TPOL_Pos             (22U)
#define GPDMA_CCR3_TPOL_Msk             (0x1UL << GPDMA_CCR3_TPOL_Pos)
#define GPDMA_CCR3_TPOL                 GPDMA_CCR3_TPOL_Msk
#define GPDMA_CCR3_TEDGE_Pos            (23U)
#define GPDMA_CCR3_TEDGE_Msk            (0x1UL << GPDMA_CCR3_TEDGE_Pos)
#define GPDMA_CCR3_TEDGE                GPDMA_CCR3_TEDGE_Msk
#define GPDMA_CCR3_DBURST_Pos           (24U)
#define GPDMA_CCR3_DBURST_Msk           (0x3UL << GPDMA_CCR3_DBURST_Pos)
#define GPDMA_CCR3_DBURST               GPDMA_CCR3_DBURST_Msk
#define GPDMA_CCR3_SBURST_Pos           (26U)
#define GPDMA_CCR3_SBURST_Msk           (0x3UL << GPDMA_CCR3_SBURST_Pos)
#define GPDMA_CCR3_SBURST               GPDMA_CCR3_SBURST_Msk
#define GPDMA_CCR3_DFIX_Pos             (28U)
#define GPDMA_CCR3_DFIX_Msk             (0x1UL << GPDMA_CCR3_DFIX_Pos)
#define GPDMA_CCR3_DFIX                 GPDMA_CCR3_DFIX_Msk
#define GPDMA_CCR3_SFIX_Pos             (29U)
#define GPDMA_CCR3_SFIX_Msk             (0x1UL << GPDMA_CCR3_SFIX_Pos)
#define GPDMA_CCR3_SFIX                 GPDMA_CCR3_SFIX_Msk

/****************** Bit definition for GPDMA_CNDTR3 register ******************/
#define GPDMA_CNDTR3_NDT_Pos            (0U)
#define GPDMA_CNDTR3_NDT_Msk            (0xFFFFFUL << GPDMA_CNDTR3_NDT_Pos)
#define GPDMA_CNDTR3_NDT                GPDMA_CNDTR3_NDT_Msk

/****************** Bit definition for GPDMA_CPAR3 register *******************/
#define GPDMA_CPAR3_PA_Pos              (0U)
#define GPDMA_CPAR3_PA_Msk              (0xFFFFFFFFUL << GPDMA_CPAR3_PA_Pos)
#define GPDMA_CPAR3_PA                  GPDMA_CPAR3_PA_Msk

/****************** Bit definition for GPDMA_CM0AR3 register ******************/
#define GPDMA_CM0AR3_MA_Pos             (0U)
#define GPDMA_CM0AR3_MA_Msk             (0xFFFFFFFFUL << GPDMA_CM0AR3_MA_Pos)
#define GPDMA_CM0AR3_MA                 GPDMA_CM0AR3_MA_Msk

/******************* Bit definition for GPDMA_CSR3 register *******************/
#define GPDMA_CSR3_DSTNDT_Pos           (0U)
#define GPDMA_CSR3_DSTNDT_Msk           (0xFFFFFUL << GPDMA_CSR3_DSTNDT_Pos)
#define GPDMA_CSR3_DSTNDT               GPDMA_CSR3_DSTNDT_Msk
#define GPDMA_CSR3_BUSERR_Pos           (20U)
#define GPDMA_CSR3_BUSERR_Msk           (0x1UL << GPDMA_CSR3_BUSERR_Pos)
#define GPDMA_CSR3_BUSERR               GPDMA_CSR3_BUSERR_Msk
#define GPDMA_CSR3_CFGERR_Pos           (21U)
#define GPDMA_CSR3_CFGERR_Msk           (0x1UL << GPDMA_CSR3_CFGERR_Pos)
#define GPDMA_CSR3_CFGERR               GPDMA_CSR3_CFGERR_Msk
#define GPDMA_CSR3_LCNT_Pos             (22U)
#define GPDMA_CSR3_LCNT_Msk             (0x3FFUL << GPDMA_CSR3_LCNT_Pos)
#define GPDMA_CSR3_LCNT                 GPDMA_CSR3_LCNT_Msk

/******************* Bit definition for GPDMA_CCR4 register *******************/
#define GPDMA_CCR4_EN_Pos               (0U)
#define GPDMA_CCR4_EN_Msk               (0x1UL << GPDMA_CCR4_EN_Pos)
#define GPDMA_CCR4_EN                   GPDMA_CCR4_EN_Msk
#define GPDMA_CCR4_TCIE_Pos             (1U)
#define GPDMA_CCR4_TCIE_Msk             (0x1UL << GPDMA_CCR4_TCIE_Pos)
#define GPDMA_CCR4_TCIE                 GPDMA_CCR4_TCIE_Msk
#define GPDMA_CCR4_HTIE_Pos             (2U)
#define GPDMA_CCR4_HTIE_Msk             (0x1UL << GPDMA_CCR4_HTIE_Pos)
#define GPDMA_CCR4_HTIE                 GPDMA_CCR4_HTIE_Msk
#define GPDMA_CCR4_TEIE_Pos             (3U)
#define GPDMA_CCR4_TEIE_Msk             (0x1UL << GPDMA_CCR4_TEIE_Pos)
#define GPDMA_CCR4_TEIE                 GPDMA_CCR4_TEIE_Msk
#define GPDMA_CCR4_DIR_Pos              (4U)
#define GPDMA_CCR4_DIR_Msk              (0x1UL << GPDMA_CCR4_DIR_Pos)
#define GPDMA_CCR4_DIR                  GPDMA_CCR4_DIR_Msk
#define GPDMA_CCR4_CIRC_Pos             (5U)
#define GPDMA_CCR4_CIRC_Msk             (0x1UL << GPDMA_CCR4_CIRC_Pos)
#define GPDMA_CCR4_CIRC                 GPDMA_CCR4_CIRC_Msk
#define GPDMA_CCR4_PINC_Pos             (6U)
#define GPDMA_CCR4_PINC_Msk             (0x1UL << GPDMA_CCR4_PINC_Pos)
#define GPDMA_CCR4_PINC                 GPDMA_CCR4_PINC_Msk
#define GPDMA_CCR4_MINC_Pos             (7U)
#define GPDMA_CCR4_MINC_Msk             (0x1UL << GPDMA_CCR4_MINC_Pos)
#define GPDMA_CCR4_MINC                 GPDMA_CCR4_MINC_Msk
#define GPDMA_CCR4_PSIZE_Pos            (8U)
#define GPDMA_CCR4_PSIZE_Msk            (0x3UL << GPDMA_CCR4_PSIZE_Pos)
#define GPDMA_CCR4_PSIZE                GPDMA_CCR4_PSIZE_Msk
#define GPDMA_CCR4_MSIZE_Pos            (10U)
#define GPDMA_CCR4_MSIZE_Msk            (0x3UL << GPDMA_CCR4_MSIZE_Pos)
#define GPDMA_CCR4_MSIZE                GPDMA_CCR4_MSIZE_Msk
#define GPDMA_CCR4_PL_Pos               (12U)
#define GPDMA_CCR4_PL_Msk               (0x3UL << GPDMA_CCR4_PL_Pos)
#define GPDMA_CCR4_PL                   GPDMA_CCR4_PL_Msk
#define GPDMA_CCR4_MEM2MEM_Pos          (14U)
#define GPDMA_CCR4_MEM2MEM_Msk          (0x1UL << GPDMA_CCR4_MEM2MEM_Pos)
#define GPDMA_CCR4_MEM2MEM              GPDMA_CCR4_MEM2MEM_Msk
#define GPDMA_CCR4_LCIE_Pos             (15U)
#define GPDMA_CCR4_LCIE_Msk             (0x1UL << GPDMA_CCR4_LCIE_Pos)
#define GPDMA_CCR4_LCIE                 GPDMA_CCR4_LCIE_Msk
#define GPDMA_CCR4_SIGN_Pos             (16U)
#define GPDMA_CCR4_SIGN_Msk             (0x1UL << GPDMA_CCR4_SIGN_Pos)
#define GPDMA_CCR4_SIGN                 GPDMA_CCR4_SIGN_Msk
#define GPDMA_CCR4_PACK_Pos             (17U)
#define GPDMA_CCR4_PACK_Msk             (0x1UL << GPDMA_CCR4_PACK_Pos)
#define GPDMA_CCR4_PACK                 GPDMA_CCR4_PACK_Msk
#define GPDMA_CCR4_DHX_Pos              (18U)
#define GPDMA_CCR4_DHX_Msk              (0x1UL << GPDMA_CCR4_DHX_Pos)
#define GPDMA_CCR4_DHX                  GPDMA_CCR4_DHX_Msk
#define GPDMA_CCR4_DBX_Pos              (19U)
#define GPDMA_CCR4_DBX_Msk              (0x1UL << GPDMA_CCR4_DBX_Pos)
#define GPDMA_CCR4_DBX                  GPDMA_CCR4_DBX_Msk
#define GPDMA_CCR4_SBS_Pos              (20U)
#define GPDMA_CCR4_SBS_Msk              (0x1UL << GPDMA_CCR4_SBS_Pos)
#define GPDMA_CCR4_SBS                  GPDMA_CCR4_SBS_Msk
#define GPDMA_CCR4_SBX_Pos              (21U)
#define GPDMA_CCR4_SBX_Msk              (0x1UL << GPDMA_CCR4_SBX_Pos)
#define GPDMA_CCR4_SBX                  GPDMA_CCR4_SBX_Msk
#define GPDMA_CCR4_TPOL_Pos             (22U)
#define GPDMA_CCR4_TPOL_Msk             (0x1UL << GPDMA_CCR4_TPOL_Pos)
#define GPDMA_CCR4_TPOL                 GPDMA_CCR4_TPOL_Msk
#define GPDMA_CCR4_TEDGE_Pos            (23U)
#define GPDMA_CCR4_TEDGE_Msk            (0x1UL << GPDMA_CCR4_TEDGE_Pos)
#define GPDMA_CCR4_TEDGE                GPDMA_CCR4_TEDGE_Msk
#define GPDMA_CCR4_DBURST_Pos           (24U)
#define GPDMA_CCR4_DBURST_Msk           (0x3UL << GPDMA_CCR4_DBURST_Pos)
#define GPDMA_CCR4_DBURST               GPDMA_CCR4_DBURST_Msk
#define GPDMA_CCR4_SBURST_Pos           (26U)
#define GPDMA_CCR4_SBURST_Msk           (0x3UL << GPDMA_CCR4_SBURST_Pos)
#define GPDMA_CCR4_SBURST               GPDMA_CCR4_SBURST_Msk
#define GPDMA_CCR4_DFIX_Pos             (28U)
#define GPDMA_CCR4_DFIX_Msk             (0x1UL << GPDMA_CCR4_DFIX_Pos)
#define GPDMA_CCR4_DFIX                 GPDMA_CCR4_DFIX_Msk
#define GPDMA_CCR4_SFIX_Pos             (29U)
#define GPDMA_CCR4_SFIX_Msk             (0x1UL << GPDMA_CCR4_SFIX_Pos)
#define GPDMA_CCR4_SFIX                 GPDMA_CCR4_SFIX_Msk

/****************** Bit definition for GPDMA_CNDTR4 register ******************/
#define GPDMA_CNDTR4_NDT_Pos            (0U)
#define GPDMA_CNDTR4_NDT_Msk            (0xFFFFFUL << GPDMA_CNDTR4_NDT_Pos)
#define GPDMA_CNDTR4_NDT                GPDMA_CNDTR4_NDT_Msk

/****************** Bit definition for GPDMA_CPAR4 register *******************/
#define GPDMA_CPAR4_PA_Pos              (0U)
#define GPDMA_CPAR4_PA_Msk              (0xFFFFFFFFUL << GPDMA_CPAR4_PA_Pos)
#define GPDMA_CPAR4_PA                  GPDMA_CPAR4_PA_Msk

/****************** Bit definition for GPDMA_CM0AR4 register ******************/
#define GPDMA_CM0AR4_MA_Pos             (0U)
#define GPDMA_CM0AR4_MA_Msk             (0xFFFFFFFFUL << GPDMA_CM0AR4_MA_Pos)
#define GPDMA_CM0AR4_MA                 GPDMA_CM0AR4_MA_Msk

/******************* Bit definition for GPDMA_CSR4 register *******************/
#define GPDMA_CSR4_DSTNDT_Pos           (0U)
#define GPDMA_CSR4_DSTNDT_Msk           (0xFFFFFUL << GPDMA_CSR4_DSTNDT_Pos)
#define GPDMA_CSR4_DSTNDT               GPDMA_CSR4_DSTNDT_Msk
#define GPDMA_CSR4_BUSERR_Pos           (20U)
#define GPDMA_CSR4_BUSERR_Msk           (0x1UL << GPDMA_CSR4_BUSERR_Pos)
#define GPDMA_CSR4_BUSERR               GPDMA_CSR4_BUSERR_Msk
#define GPDMA_CSR4_CFGERR_Pos           (21U)
#define GPDMA_CSR4_CFGERR_Msk           (0x1UL << GPDMA_CSR4_CFGERR_Pos)
#define GPDMA_CSR4_CFGERR               GPDMA_CSR4_CFGERR_Msk
#define GPDMA_CSR4_LCNT_Pos             (22U)
#define GPDMA_CSR4_LCNT_Msk             (0x3FFUL << GPDMA_CSR4_LCNT_Pos)
#define GPDMA_CSR4_LCNT                 GPDMA_CSR4_LCNT_Msk

/******************* Bit definition for GPDMA_CCR5 register *******************/
#define GPDMA_CCR5_EN_Pos               (0U)
#define GPDMA_CCR5_EN_Msk               (0x1UL << GPDMA_CCR5_EN_Pos)
#define GPDMA_CCR5_EN                   GPDMA_CCR5_EN_Msk
#define GPDMA_CCR5_TCIE_Pos             (1U)
#define GPDMA_CCR5_TCIE_Msk             (0x1UL << GPDMA_CCR5_TCIE_Pos)
#define GPDMA_CCR5_TCIE                 GPDMA_CCR5_TCIE_Msk
#define GPDMA_CCR5_HTIE_Pos             (2U)
#define GPDMA_CCR5_HTIE_Msk             (0x1UL << GPDMA_CCR5_HTIE_Pos)
#define GPDMA_CCR5_HTIE                 GPDMA_CCR5_HTIE_Msk
#define GPDMA_CCR5_TEIE_Pos             (3U)
#define GPDMA_CCR5_TEIE_Msk             (0x1UL << GPDMA_CCR5_TEIE_Pos)
#define GPDMA_CCR5_TEIE                 GPDMA_CCR5_TEIE_Msk
#define GPDMA_CCR5_DIR_Pos              (4U)
#define GPDMA_CCR5_DIR_Msk              (0x1UL << GPDMA_CCR5_DIR_Pos)
#define GPDMA_CCR5_DIR                  GPDMA_CCR5_DIR_Msk
#define GPDMA_CCR5_CIRC_Pos             (5U)
#define GPDMA_CCR5_CIRC_Msk             (0x1UL << GPDMA_CCR5_CIRC_Pos)
#define GPDMA_CCR5_CIRC                 GPDMA_CCR5_CIRC_Msk
#define GPDMA_CCR5_PINC_Pos             (6U)
#define GPDMA_CCR5_PINC_Msk             (0x1UL << GPDMA_CCR5_PINC_Pos)
#define GPDMA_CCR5_PINC                 GPDMA_CCR5_PINC_Msk
#define GPDMA_CCR5_MINC_Pos             (7U)
#define GPDMA_CCR5_MINC_Msk             (0x1UL << GPDMA_CCR5_MINC_Pos)
#define GPDMA_CCR5_MINC                 GPDMA_CCR5_MINC_Msk
#define GPDMA_CCR5_PSIZE_Pos            (8U)
#define GPDMA_CCR5_PSIZE_Msk            (0x3UL << GPDMA_CCR5_PSIZE_Pos)
#define GPDMA_CCR5_PSIZE                GPDMA_CCR5_PSIZE_Msk
#define GPDMA_CCR5_MSIZE_Pos            (10U)
#define GPDMA_CCR5_MSIZE_Msk            (0x3UL << GPDMA_CCR5_MSIZE_Pos)
#define GPDMA_CCR5_MSIZE                GPDMA_CCR5_MSIZE_Msk
#define GPDMA_CCR5_PL_Pos               (12U)
#define GPDMA_CCR5_PL_Msk               (0x3UL << GPDMA_CCR5_PL_Pos)
#define GPDMA_CCR5_PL                   GPDMA_CCR5_PL_Msk
#define GPDMA_CCR5_MEM2MEM_Pos          (14U)
#define GPDMA_CCR5_MEM2MEM_Msk          (0x1UL << GPDMA_CCR5_MEM2MEM_Pos)
#define GPDMA_CCR5_MEM2MEM              GPDMA_CCR5_MEM2MEM_Msk
#define GPDMA_CCR5_LCIE_Pos             (15U)
#define GPDMA_CCR5_LCIE_Msk             (0x1UL << GPDMA_CCR5_LCIE_Pos)
#define GPDMA_CCR5_LCIE                 GPDMA_CCR5_LCIE_Msk
#define GPDMA_CCR5_SIGN_Pos             (16U)
#define GPDMA_CCR5_SIGN_Msk             (0x1UL << GPDMA_CCR5_SIGN_Pos)
#define GPDMA_CCR5_SIGN                 GPDMA_CCR5_SIGN_Msk
#define GPDMA_CCR5_PACK_Pos             (17U)
#define GPDMA_CCR5_PACK_Msk             (0x1UL << GPDMA_CCR5_PACK_Pos)
#define GPDMA_CCR5_PACK                 GPDMA_CCR5_PACK_Msk
#define GPDMA_CCR5_DHX_Pos              (18U)
#define GPDMA_CCR5_DHX_Msk              (0x1UL << GPDMA_CCR5_DHX_Pos)
#define GPDMA_CCR5_DHX                  GPDMA_CCR5_DHX_Msk
#define GPDMA_CCR5_DBX_Pos              (19U)
#define GPDMA_CCR5_DBX_Msk              (0x1UL << GPDMA_CCR5_DBX_Pos)
#define GPDMA_CCR5_DBX                  GPDMA_CCR5_DBX_Msk
#define GPDMA_CCR5_SBS_Pos              (20U)
#define GPDMA_CCR5_SBS_Msk              (0x1UL << GPDMA_CCR5_SBS_Pos)
#define GPDMA_CCR5_SBS                  GPDMA_CCR5_SBS_Msk
#define GPDMA_CCR5_SBX_Pos              (21U)
#define GPDMA_CCR5_SBX_Msk              (0x1UL << GPDMA_CCR5_SBX_Pos)
#define GPDMA_CCR5_SBX                  GPDMA_CCR5_SBX_Msk
#define GPDMA_CCR5_TPOL_Pos             (22U)
#define GPDMA_CCR5_TPOL_Msk             (0x1UL << GPDMA_CCR5_TPOL_Pos)
#define GPDMA_CCR5_TPOL                 GPDMA_CCR5_TPOL_Msk
#define GPDMA_CCR5_TEDGE_Pos            (23U)
#define GPDMA_CCR5_TEDGE_Msk            (0x1UL << GPDMA_CCR5_TEDGE_Pos)
#define GPDMA_CCR5_TEDGE                GPDMA_CCR5_TEDGE_Msk
#define GPDMA_CCR5_DBURST_Pos           (24U)
#define GPDMA_CCR5_DBURST_Msk           (0x3UL << GPDMA_CCR5_DBURST_Pos)
#define GPDMA_CCR5_DBURST               GPDMA_CCR5_DBURST_Msk
#define GPDMA_CCR5_SBURST_Pos           (26U)
#define GPDMA_CCR5_SBURST_Msk           (0x3UL << GPDMA_CCR5_SBURST_Pos)
#define GPDMA_CCR5_SBURST               GPDMA_CCR5_SBURST_Msk
#define GPDMA_CCR5_DFIX_Pos             (28U)
#define GPDMA_CCR5_DFIX_Msk             (0x1UL << GPDMA_CCR5_DFIX_Pos)
#define GPDMA_CCR5_DFIX                 GPDMA_CCR5_DFIX_Msk
#define GPDMA_CCR5_SFIX_Pos             (29U)
#define GPDMA_CCR5_SFIX_Msk             (0x1UL << GPDMA_CCR5_SFIX_Pos)
#define GPDMA_CCR5_SFIX                 GPDMA_CCR5_SFIX_Msk

/****************** Bit definition for GPDMA_CNDTR5 register ******************/
#define GPDMA_CNDTR5_NDT_Pos            (0U)
#define GPDMA_CNDTR5_NDT_Msk            (0xFFFFFUL << GPDMA_CNDTR5_NDT_Pos)
#define GPDMA_CNDTR5_NDT                GPDMA_CNDTR5_NDT_Msk

/****************** Bit definition for GPDMA_CPAR5 register *******************/
#define GPDMA_CPAR5_PA_Pos              (0U)
#define GPDMA_CPAR5_PA_Msk              (0xFFFFFFFFUL << GPDMA_CPAR5_PA_Pos)
#define GPDMA_CPAR5_PA                  GPDMA_CPAR5_PA_Msk

/****************** Bit definition for GPDMA_CM0AR5 register ******************/
#define GPDMA_CM0AR5_MA_Pos             (0U)
#define GPDMA_CM0AR5_MA_Msk             (0xFFFFFFFFUL << GPDMA_CM0AR5_MA_Pos)
#define GPDMA_CM0AR5_MA                 GPDMA_CM0AR5_MA_Msk

/******************* Bit definition for GPDMA_CSR5 register *******************/
#define GPDMA_CSR5_DSTNDT_Pos           (0U)
#define GPDMA_CSR5_DSTNDT_Msk           (0xFFFFFUL << GPDMA_CSR5_DSTNDT_Pos)
#define GPDMA_CSR5_DSTNDT               GPDMA_CSR5_DSTNDT_Msk
#define GPDMA_CSR5_BUSERR_Pos           (20U)
#define GPDMA_CSR5_BUSERR_Msk           (0x1UL << GPDMA_CSR5_BUSERR_Pos)
#define GPDMA_CSR5_BUSERR               GPDMA_CSR5_BUSERR_Msk
#define GPDMA_CSR5_CFGERR_Pos           (21U)
#define GPDMA_CSR5_CFGERR_Msk           (0x1UL << GPDMA_CSR5_CFGERR_Pos)
#define GPDMA_CSR5_CFGERR               GPDMA_CSR5_CFGERR_Msk
#define GPDMA_CSR5_LCNT_Pos             (22U)
#define GPDMA_CSR5_LCNT_Msk             (0x3FFUL << GPDMA_CSR5_LCNT_Pos)
#define GPDMA_CSR5_LCNT                 GPDMA_CSR5_LCNT_Msk

/******************* Bit definition for GPDMA_CCR6 register *******************/
#define GPDMA_CCR6_EN_Pos               (0U)
#define GPDMA_CCR6_EN_Msk               (0x1UL << GPDMA_CCR6_EN_Pos)
#define GPDMA_CCR6_EN                   GPDMA_CCR6_EN_Msk
#define GPDMA_CCR6_TCIE_Pos             (1U)
#define GPDMA_CCR6_TCIE_Msk             (0x1UL << GPDMA_CCR6_TCIE_Pos)
#define GPDMA_CCR6_TCIE                 GPDMA_CCR6_TCIE_Msk
#define GPDMA_CCR6_HTIE_Pos             (2U)
#define GPDMA_CCR6_HTIE_Msk             (0x1UL << GPDMA_CCR6_HTIE_Pos)
#define GPDMA_CCR6_HTIE                 GPDMA_CCR6_HTIE_Msk
#define GPDMA_CCR6_TEIE_Pos             (3U)
#define GPDMA_CCR6_TEIE_Msk             (0x1UL << GPDMA_CCR6_TEIE_Pos)
#define GPDMA_CCR6_TEIE                 GPDMA_CCR6_TEIE_Msk
#define GPDMA_CCR6_DIR_Pos              (4U)
#define GPDMA_CCR6_DIR_Msk              (0x1UL << GPDMA_CCR6_DIR_Pos)
#define GPDMA_CCR6_DIR                  GPDMA_CCR6_DIR_Msk
#define GPDMA_CCR6_CIRC_Pos             (5U)
#define GPDMA_CCR6_CIRC_Msk             (0x1UL << GPDMA_CCR6_CIRC_Pos)
#define GPDMA_CCR6_CIRC                 GPDMA_CCR6_CIRC_Msk
#define GPDMA_CCR6_PINC_Pos             (6U)
#define GPDMA_CCR6_PINC_Msk             (0x1UL << GPDMA_CCR6_PINC_Pos)
#define GPDMA_CCR6_PINC                 GPDMA_CCR6_PINC_Msk
#define GPDMA_CCR6_MINC_Pos             (7U)
#define GPDMA_CCR6_MINC_Msk             (0x1UL << GPDMA_CCR6_MINC_Pos)
#define GPDMA_CCR6_MINC                 GPDMA_CCR6_MINC_Msk
#define GPDMA_CCR6_PSIZE_Pos            (8U)
#define GPDMA_CCR6_PSIZE_Msk            (0x3UL << GPDMA_CCR6_PSIZE_Pos)
#define GPDMA_CCR6_PSIZE                GPDMA_CCR6_PSIZE_Msk
#define GPDMA_CCR6_MSIZE_Pos            (10U)
#define GPDMA_CCR6_MSIZE_Msk            (0x3UL << GPDMA_CCR6_MSIZE_Pos)
#define GPDMA_CCR6_MSIZE                GPDMA_CCR6_MSIZE_Msk
#define GPDMA_CCR6_PL_Pos               (12U)
#define GPDMA_CCR6_PL_Msk               (0x3UL << GPDMA_CCR6_PL_Pos)
#define GPDMA_CCR6_PL                   GPDMA_CCR6_PL_Msk
#define GPDMA_CCR6_MEM2MEM_Pos          (14U)
#define GPDMA_CCR6_MEM2MEM_Msk          (0x1UL << GPDMA_CCR6_MEM2MEM_Pos)
#define GPDMA_CCR6_MEM2MEM              GPDMA_CCR6_MEM2MEM_Msk
#define GPDMA_CCR6_LCIE_Pos             (15U)
#define GPDMA_CCR6_LCIE_Msk             (0x1UL << GPDMA_CCR6_LCIE_Pos)
#define GPDMA_CCR6_LCIE                 GPDMA_CCR6_LCIE_Msk
#define GPDMA_CCR6_SIGN_Pos             (16U)
#define GPDMA_CCR6_SIGN_Msk             (0x1UL << GPDMA_CCR6_SIGN_Pos)
#define GPDMA_CCR6_SIGN                 GPDMA_CCR6_SIGN_Msk
#define GPDMA_CCR6_PACK_Pos             (17U)
#define GPDMA_CCR6_PACK_Msk             (0x1UL << GPDMA_CCR6_PACK_Pos)
#define GPDMA_CCR6_PACK                 GPDMA_CCR6_PACK_Msk
#define GPDMA_CCR6_DHX_Pos              (18U)
#define GPDMA_CCR6_DHX_Msk              (0x1UL << GPDMA_CCR6_DHX_Pos)
#define GPDMA_CCR6_DHX                  GPDMA_CCR6_DHX_Msk
#define GPDMA_CCR6_DBX_Pos              (19U)
#define GPDMA_CCR6_DBX_Msk              (0x1UL << GPDMA_CCR6_DBX_Pos)
#define GPDMA_CCR6_DBX                  GPDMA_CCR6_DBX_Msk
#define GPDMA_CCR6_SBS_Pos              (20U)
#define GPDMA_CCR6_SBS_Msk              (0x1UL << GPDMA_CCR6_SBS_Pos)
#define GPDMA_CCR6_SBS                  GPDMA_CCR6_SBS_Msk
#define GPDMA_CCR6_SBX_Pos              (21U)
#define GPDMA_CCR6_SBX_Msk              (0x1UL << GPDMA_CCR6_SBX_Pos)
#define GPDMA_CCR6_SBX                  GPDMA_CCR6_SBX_Msk
#define GPDMA_CCR6_TPOL_Pos             (22U)
#define GPDMA_CCR6_TPOL_Msk             (0x1UL << GPDMA_CCR6_TPOL_Pos)
#define GPDMA_CCR6_TPOL                 GPDMA_CCR6_TPOL_Msk
#define GPDMA_CCR6_TEDGE_Pos            (23U)
#define GPDMA_CCR6_TEDGE_Msk            (0x1UL << GPDMA_CCR6_TEDGE_Pos)
#define GPDMA_CCR6_TEDGE                GPDMA_CCR6_TEDGE_Msk
#define GPDMA_CCR6_DBURST_Pos           (24U)
#define GPDMA_CCR6_DBURST_Msk           (0x3UL << GPDMA_CCR6_DBURST_Pos)
#define GPDMA_CCR6_DBURST               GPDMA_CCR6_DBURST_Msk
#define GPDMA_CCR6_SBURST_Pos           (26U)
#define GPDMA_CCR6_SBURST_Msk           (0x3UL << GPDMA_CCR6_SBURST_Pos)
#define GPDMA_CCR6_SBURST               GPDMA_CCR6_SBURST_Msk
#define GPDMA_CCR6_DFIX_Pos             (28U)
#define GPDMA_CCR6_DFIX_Msk             (0x1UL << GPDMA_CCR6_DFIX_Pos)
#define GPDMA_CCR6_DFIX                 GPDMA_CCR6_DFIX_Msk
#define GPDMA_CCR6_SFIX_Pos             (29U)
#define GPDMA_CCR6_SFIX_Msk             (0x1UL << GPDMA_CCR6_SFIX_Pos)
#define GPDMA_CCR6_SFIX                 GPDMA_CCR6_SFIX_Msk

/****************** Bit definition for GPDMA_CNDTR6 register ******************/
#define GPDMA_CNDTR6_NDT_Pos            (0U)
#define GPDMA_CNDTR6_NDT_Msk            (0xFFFFFUL << GPDMA_CNDTR6_NDT_Pos)
#define GPDMA_CNDTR6_NDT                GPDMA_CNDTR6_NDT_Msk

/****************** Bit definition for GPDMA_CPAR6 register *******************/
#define GPDMA_CPAR6_PA_Pos              (0U)
#define GPDMA_CPAR6_PA_Msk              (0xFFFFFFFFUL << GPDMA_CPAR6_PA_Pos)
#define GPDMA_CPAR6_PA                  GPDMA_CPAR6_PA_Msk

/****************** Bit definition for GPDMA_CM0AR6 register ******************/
#define GPDMA_CM0AR6_MA_Pos             (0U)
#define GPDMA_CM0AR6_MA_Msk             (0xFFFFFFFFUL << GPDMA_CM0AR6_MA_Pos)
#define GPDMA_CM0AR6_MA                 GPDMA_CM0AR6_MA_Msk

/******************* Bit definition for GPDMA_CSR6 register *******************/
#define GPDMA_CSR6_DSTNDT_Pos           (0U)
#define GPDMA_CSR6_DSTNDT_Msk           (0xFFFFFUL << GPDMA_CSR6_DSTNDT_Pos)
#define GPDMA_CSR6_DSTNDT               GPDMA_CSR6_DSTNDT_Msk
#define GPDMA_CSR6_BUSERR_Pos           (20U)
#define GPDMA_CSR6_BUSERR_Msk           (0x1UL << GPDMA_CSR6_BUSERR_Pos)
#define GPDMA_CSR6_BUSERR               GPDMA_CSR6_BUSERR_Msk
#define GPDMA_CSR6_CFGERR_Pos           (21U)
#define GPDMA_CSR6_CFGERR_Msk           (0x1UL << GPDMA_CSR6_CFGERR_Pos)
#define GPDMA_CSR6_CFGERR               GPDMA_CSR6_CFGERR_Msk
#define GPDMA_CSR6_LCNT_Pos             (22U)
#define GPDMA_CSR6_LCNT_Msk             (0x3FFUL << GPDMA_CSR6_LCNT_Pos)
#define GPDMA_CSR6_LCNT                 GPDMA_CSR6_LCNT_Msk

/******************* Bit definition for GPDMA_CCR7 register *******************/
#define GPDMA_CCR7_EN_Pos               (0U)
#define GPDMA_CCR7_EN_Msk               (0x1UL << GPDMA_CCR7_EN_Pos)
#define GPDMA_CCR7_EN                   GPDMA_CCR7_EN_Msk
#define GPDMA_CCR7_TCIE_Pos             (1U)
#define GPDMA_CCR7_TCIE_Msk             (0x1UL << GPDMA_CCR7_TCIE_Pos)
#define GPDMA_CCR7_TCIE                 GPDMA_CCR7_TCIE_Msk
#define GPDMA_CCR7_HTIE_Pos             (2U)
#define GPDMA_CCR7_HTIE_Msk             (0x1UL << GPDMA_CCR7_HTIE_Pos)
#define GPDMA_CCR7_HTIE                 GPDMA_CCR7_HTIE_Msk
#define GPDMA_CCR7_TEIE_Pos             (3U)
#define GPDMA_CCR7_TEIE_Msk             (0x1UL << GPDMA_CCR7_TEIE_Pos)
#define GPDMA_CCR7_TEIE                 GPDMA_CCR7_TEIE_Msk
#define GPDMA_CCR7_DIR_Pos              (4U)
#define GPDMA_CCR7_DIR_Msk              (0x1UL << GPDMA_CCR7_DIR_Pos)
#define GPDMA_CCR7_DIR                  GPDMA_CCR7_DIR_Msk
#define GPDMA_CCR7_CIRC_Pos             (5U)
#define GPDMA_CCR7_CIRC_Msk             (0x1UL << GPDMA_CCR7_CIRC_Pos)
#define GPDMA_CCR7_CIRC                 GPDMA_CCR7_CIRC_Msk
#define GPDMA_CCR7_PINC_Pos             (6U)
#define GPDMA_CCR7_PINC_Msk             (0x1UL << GPDMA_CCR7_PINC_Pos)
#define GPDMA_CCR7_PINC                 GPDMA_CCR7_PINC_Msk
#define GPDMA_CCR7_MINC_Pos             (7U)
#define GPDMA_CCR7_MINC_Msk             (0x1UL << GPDMA_CCR7_MINC_Pos)
#define GPDMA_CCR7_MINC                 GPDMA_CCR7_MINC_Msk
#define GPDMA_CCR7_PSIZE_Pos            (8U)
#define GPDMA_CCR7_PSIZE_Msk            (0x3UL << GPDMA_CCR7_PSIZE_Pos)
#define GPDMA_CCR7_PSIZE                GPDMA_CCR7_PSIZE_Msk
#define GPDMA_CCR7_MSIZE_Pos            (10U)
#define GPDMA_CCR7_MSIZE_Msk            (0x3UL << GPDMA_CCR7_MSIZE_Pos)
#define GPDMA_CCR7_MSIZE                GPDMA_CCR7_MSIZE_Msk
#define GPDMA_CCR7_PL_Pos               (12U)
#define GPDMA_CCR7_PL_Msk               (0x3UL << GPDMA_CCR7_PL_Pos)
#define GPDMA_CCR7_PL                   GPDMA_CCR7_PL_Msk
#define GPDMA_CCR7_MEM2MEM_Pos          (14U)
#define GPDMA_CCR7_MEM2MEM_Msk          (0x1UL << GPDMA_CCR7_MEM2MEM_Pos)
#define GPDMA_CCR7_MEM2MEM              GPDMA_CCR7_MEM2MEM_Msk
#define GPDMA_CCR7_LCIE_Pos             (15U)
#define GPDMA_CCR7_LCIE_Msk             (0x1UL << GPDMA_CCR7_LCIE_Pos)
#define GPDMA_CCR7_LCIE                 GPDMA_CCR7_LCIE_Msk
#define GPDMA_CCR7_SIGN_Pos             (16U)
#define GPDMA_CCR7_SIGN_Msk             (0x1UL << GPDMA_CCR7_SIGN_Pos)
#define GPDMA_CCR7_SIGN                 GPDMA_CCR7_SIGN_Msk
#define GPDMA_CCR7_PACK_Pos             (17U)
#define GPDMA_CCR7_PACK_Msk             (0x1UL << GPDMA_CCR7_PACK_Pos)
#define GPDMA_CCR7_PACK                 GPDMA_CCR7_PACK_Msk
#define GPDMA_CCR7_DHX_Pos              (18U)
#define GPDMA_CCR7_DHX_Msk              (0x1UL << GPDMA_CCR7_DHX_Pos)
#define GPDMA_CCR7_DHX                  GPDMA_CCR7_DHX_Msk
#define GPDMA_CCR7_DBX_Pos              (19U)
#define GPDMA_CCR7_DBX_Msk              (0x1UL << GPDMA_CCR7_DBX_Pos)
#define GPDMA_CCR7_DBX                  GPDMA_CCR7_DBX_Msk
#define GPDMA_CCR7_SBS_Pos              (20U)
#define GPDMA_CCR7_SBS_Msk              (0x1UL << GPDMA_CCR7_SBS_Pos)
#define GPDMA_CCR7_SBS                  GPDMA_CCR7_SBS_Msk
#define GPDMA_CCR7_SBX_Pos              (21U)
#define GPDMA_CCR7_SBX_Msk              (0x1UL << GPDMA_CCR7_SBX_Pos)
#define GPDMA_CCR7_SBX                  GPDMA_CCR7_SBX_Msk
#define GPDMA_CCR7_TPOL_Pos             (22U)
#define GPDMA_CCR7_TPOL_Msk             (0x1UL << GPDMA_CCR7_TPOL_Pos)
#define GPDMA_CCR7_TPOL                 GPDMA_CCR7_TPOL_Msk
#define GPDMA_CCR7_TEDGE_Pos            (23U)
#define GPDMA_CCR7_TEDGE_Msk            (0x1UL << GPDMA_CCR7_TEDGE_Pos)
#define GPDMA_CCR7_TEDGE                GPDMA_CCR7_TEDGE_Msk
#define GPDMA_CCR7_DBURST_Pos           (24U)
#define GPDMA_CCR7_DBURST_Msk           (0x3UL << GPDMA_CCR7_DBURST_Pos)
#define GPDMA_CCR7_DBURST               GPDMA_CCR7_DBURST_Msk
#define GPDMA_CCR7_SBURST_Pos           (26U)
#define GPDMA_CCR7_SBURST_Msk           (0x3UL << GPDMA_CCR7_SBURST_Pos)
#define GPDMA_CCR7_SBURST               GPDMA_CCR7_SBURST_Msk
#define GPDMA_CCR7_DFIX_Pos             (28U)
#define GPDMA_CCR7_DFIX_Msk             (0x1UL << GPDMA_CCR7_DFIX_Pos)
#define GPDMA_CCR7_DFIX                 GPDMA_CCR7_DFIX_Msk
#define GPDMA_CCR7_SFIX_Pos             (29U)
#define GPDMA_CCR7_SFIX_Msk             (0x1UL << GPDMA_CCR7_SFIX_Pos)
#define GPDMA_CCR7_SFIX                 GPDMA_CCR7_SFIX_Msk

/****************** Bit definition for GPDMA_CNDTR7 register ******************/
#define GPDMA_CNDTR7_NDT_Pos            (0U)
#define GPDMA_CNDTR7_NDT_Msk            (0xFFFFFUL << GPDMA_CNDTR7_NDT_Pos)
#define GPDMA_CNDTR7_NDT                GPDMA_CNDTR7_NDT_Msk

/****************** Bit definition for GPDMA_CPAR7 register *******************/
#define GPDMA_CPAR7_PA_Pos              (0U)
#define GPDMA_CPAR7_PA_Msk              (0xFFFFFFFFUL << GPDMA_CPAR7_PA_Pos)
#define GPDMA_CPAR7_PA                  GPDMA_CPAR7_PA_Msk

/****************** Bit definition for GPDMA_CM0AR7 register ******************/
#define GPDMA_CM0AR7_MA_Pos             (0U)
#define GPDMA_CM0AR7_MA_Msk             (0xFFFFFFFFUL << GPDMA_CM0AR7_MA_Pos)
#define GPDMA_CM0AR7_MA                 GPDMA_CM0AR7_MA_Msk

/******************* Bit definition for GPDMA_CSR7 register *******************/
#define GPDMA_CSR7_DSTNDT_Pos           (0U)
#define GPDMA_CSR7_DSTNDT_Msk           (0xFFFFFUL << GPDMA_CSR7_DSTNDT_Pos)
#define GPDMA_CSR7_DSTNDT               GPDMA_CSR7_DSTNDT_Msk
#define GPDMA_CSR7_BUSERR_Pos           (20U)
#define GPDMA_CSR7_BUSERR_Msk           (0x1UL << GPDMA_CSR7_BUSERR_Pos)
#define GPDMA_CSR7_BUSERR               GPDMA_CSR7_BUSERR_Msk
#define GPDMA_CSR7_CFGERR_Pos           (21U)
#define GPDMA_CSR7_CFGERR_Msk           (0x1UL << GPDMA_CSR7_CFGERR_Pos)
#define GPDMA_CSR7_CFGERR               GPDMA_CSR7_CFGERR_Msk
#define GPDMA_CSR7_LCNT_Pos             (22U)
#define GPDMA_CSR7_LCNT_Msk             (0x3FFUL << GPDMA_CSR7_LCNT_Pos)
#define GPDMA_CSR7_LCNT                 GPDMA_CSR7_LCNT_Msk

/******************* Bit definition for GPDMA_CCR8 register *******************/
#define GPDMA_CCR8_EN_Pos               (0U)
#define GPDMA_CCR8_EN_Msk               (0x1UL << GPDMA_CCR8_EN_Pos)
#define GPDMA_CCR8_EN                   GPDMA_CCR8_EN_Msk
#define GPDMA_CCR8_TCIE_Pos             (1U)
#define GPDMA_CCR8_TCIE_Msk             (0x1UL << GPDMA_CCR8_TCIE_Pos)
#define GPDMA_CCR8_TCIE                 GPDMA_CCR8_TCIE_Msk
#define GPDMA_CCR8_HTIE_Pos             (2U)
#define GPDMA_CCR8_HTIE_Msk             (0x1UL << GPDMA_CCR8_HTIE_Pos)
#define GPDMA_CCR8_HTIE                 GPDMA_CCR8_HTIE_Msk
#define GPDMA_CCR8_TEIE_Pos             (3U)
#define GPDMA_CCR8_TEIE_Msk             (0x1UL << GPDMA_CCR8_TEIE_Pos)
#define GPDMA_CCR8_TEIE                 GPDMA_CCR8_TEIE_Msk
#define GPDMA_CCR8_DIR_Pos              (4U)
#define GPDMA_CCR8_DIR_Msk              (0x1UL << GPDMA_CCR8_DIR_Pos)
#define GPDMA_CCR8_DIR                  GPDMA_CCR8_DIR_Msk
#define GPDMA_CCR8_CIRC_Pos             (5U)
#define GPDMA_CCR8_CIRC_Msk             (0x1UL << GPDMA_CCR8_CIRC_Pos)
#define GPDMA_CCR8_CIRC                 GPDMA_CCR8_CIRC_Msk
#define GPDMA_CCR8_PINC_Pos             (6U)
#define GPDMA_CCR8_PINC_Msk             (0x1UL << GPDMA_CCR8_PINC_Pos)
#define GPDMA_CCR8_PINC                 GPDMA_CCR8_PINC_Msk
#define GPDMA_CCR8_MINC_Pos             (7U)
#define GPDMA_CCR8_MINC_Msk             (0x1UL << GPDMA_CCR8_MINC_Pos)
#define GPDMA_CCR8_MINC                 GPDMA_CCR8_MINC_Msk
#define GPDMA_CCR8_PSIZE_Pos            (8U)
#define GPDMA_CCR8_PSIZE_Msk            (0x3UL << GPDMA_CCR8_PSIZE_Pos)
#define GPDMA_CCR8_PSIZE                GPDMA_CCR8_PSIZE_Msk
#define GPDMA_CCR8_MSIZE_Pos            (10U)
#define GPDMA_CCR8_MSIZE_Msk            (0x3UL << GPDMA_CCR8_MSIZE_Pos)
#define GPDMA_CCR8_MSIZE                GPDMA_CCR8_MSIZE_Msk
#define GPDMA_CCR8_PL_Pos               (12U)
#define GPDMA_CCR8_PL_Msk               (0x3UL << GPDMA_CCR8_PL_Pos)
#define GPDMA_CCR8_PL                   GPDMA_CCR8_PL_Msk
#define GPDMA_CCR8_MEM2MEM_Pos          (14U)
#define GPDMA_CCR8_MEM2MEM_Msk          (0x1UL << GPDMA_CCR8_MEM2MEM_Pos)
#define GPDMA_CCR8_MEM2MEM              GPDMA_CCR8_MEM2MEM_Msk
#define GPDMA_CCR8_LCIE_Pos             (15U)
#define GPDMA_CCR8_LCIE_Msk             (0x1UL << GPDMA_CCR8_LCIE_Pos)
#define GPDMA_CCR8_LCIE                 GPDMA_CCR8_LCIE_Msk
#define GPDMA_CCR8_SIGN_Pos             (16U)
#define GPDMA_CCR8_SIGN_Msk             (0x1UL << GPDMA_CCR8_SIGN_Pos)
#define GPDMA_CCR8_SIGN                 GPDMA_CCR8_SIGN_Msk
#define GPDMA_CCR8_PACK_Pos             (17U)
#define GPDMA_CCR8_PACK_Msk             (0x1UL << GPDMA_CCR8_PACK_Pos)
#define GPDMA_CCR8_PACK                 GPDMA_CCR8_PACK_Msk
#define GPDMA_CCR8_DHX_Pos              (18U)
#define GPDMA_CCR8_DHX_Msk              (0x1UL << GPDMA_CCR8_DHX_Pos)
#define GPDMA_CCR8_DHX                  GPDMA_CCR8_DHX_Msk
#define GPDMA_CCR8_DBX_Pos              (19U)
#define GPDMA_CCR8_DBX_Msk              (0x1UL << GPDMA_CCR8_DBX_Pos)
#define GPDMA_CCR8_DBX                  GPDMA_CCR8_DBX_Msk
#define GPDMA_CCR8_SBS_Pos              (20U)
#define GPDMA_CCR8_SBS_Msk              (0x1UL << GPDMA_CCR8_SBS_Pos)
#define GPDMA_CCR8_SBS                  GPDMA_CCR8_SBS_Msk
#define GPDMA_CCR8_SBX_Pos              (21U)
#define GPDMA_CCR8_SBX_Msk              (0x1UL << GPDMA_CCR8_SBX_Pos)
#define GPDMA_CCR8_SBX                  GPDMA_CCR8_SBX_Msk
#define GPDMA_CCR8_TPOL_Pos             (22U)
#define GPDMA_CCR8_TPOL_Msk             (0x1UL << GPDMA_CCR8_TPOL_Pos)
#define GPDMA_CCR8_TPOL                 GPDMA_CCR8_TPOL_Msk
#define GPDMA_CCR8_TEDGE_Pos            (23U)
#define GPDMA_CCR8_TEDGE_Msk            (0x1UL << GPDMA_CCR8_TEDGE_Pos)
#define GPDMA_CCR8_TEDGE                GPDMA_CCR8_TEDGE_Msk
#define GPDMA_CCR8_DBURST_Pos           (24U)
#define GPDMA_CCR8_DBURST_Msk           (0x3UL << GPDMA_CCR8_DBURST_Pos)
#define GPDMA_CCR8_DBURST               GPDMA_CCR8_DBURST_Msk
#define GPDMA_CCR8_SBURST_Pos           (26U)
#define GPDMA_CCR8_SBURST_Msk           (0x3UL << GPDMA_CCR8_SBURST_Pos)
#define GPDMA_CCR8_SBURST               GPDMA_CCR8_SBURST_Msk
#define GPDMA_CCR8_DFIX_Pos             (28U)
#define GPDMA_CCR8_DFIX_Msk             (0x1UL << GPDMA_CCR8_DFIX_Pos)
#define GPDMA_CCR8_DFIX                 GPDMA_CCR8_DFIX_Msk
#define GPDMA_CCR8_SFIX_Pos             (29U)
#define GPDMA_CCR8_SFIX_Msk             (0x1UL << GPDMA_CCR8_SFIX_Pos)
#define GPDMA_CCR8_SFIX                 GPDMA_CCR8_SFIX_Msk

/****************** Bit definition for GPDMA_CNDTR8 register ******************/
#define GPDMA_CNDTR8_NDT_Pos            (0U)
#define GPDMA_CNDTR8_NDT_Msk            (0xFFFFFUL << GPDMA_CNDTR8_NDT_Pos)
#define GPDMA_CNDTR8_NDT                GPDMA_CNDTR8_NDT_Msk

/****************** Bit definition for GPDMA_CPAR8 register *******************/
#define GPDMA_CPAR8_PA_Pos              (0U)
#define GPDMA_CPAR8_PA_Msk              (0xFFFFFFFFUL << GPDMA_CPAR8_PA_Pos)
#define GPDMA_CPAR8_PA                  GPDMA_CPAR8_PA_Msk

/****************** Bit definition for GPDMA_CM0AR8 register ******************/
#define GPDMA_CM0AR8_MA_Pos             (0U)
#define GPDMA_CM0AR8_MA_Msk             (0xFFFFFFFFUL << GPDMA_CM0AR8_MA_Pos)
#define GPDMA_CM0AR8_MA                 GPDMA_CM0AR8_MA_Msk

/******************* Bit definition for GPDMA_CSR8 register *******************/
#define GPDMA_CSR8_DSTNDT_Pos           (0U)
#define GPDMA_CSR8_DSTNDT_Msk           (0xFFFFFUL << GPDMA_CSR8_DSTNDT_Pos)
#define GPDMA_CSR8_DSTNDT               GPDMA_CSR8_DSTNDT_Msk
#define GPDMA_CSR8_BUSERR_Pos           (20U)
#define GPDMA_CSR8_BUSERR_Msk           (0x1UL << GPDMA_CSR8_BUSERR_Pos)
#define GPDMA_CSR8_BUSERR               GPDMA_CSR8_BUSERR_Msk
#define GPDMA_CSR8_CFGERR_Pos           (21U)
#define GPDMA_CSR8_CFGERR_Msk           (0x1UL << GPDMA_CSR8_CFGERR_Pos)
#define GPDMA_CSR8_CFGERR               GPDMA_CSR8_CFGERR_Msk
#define GPDMA_CSR8_LCNT_Pos             (22U)
#define GPDMA_CSR8_LCNT_Msk             (0x3FFUL << GPDMA_CSR8_LCNT_Pos)
#define GPDMA_CSR8_LCNT                 GPDMA_CSR8_LCNT_Msk

/****************** Bit definition for GPDMA_CSELR1 register ******************/
#define GPDMA_CSELR1_C1S_Pos            (0U)
#define GPDMA_CSELR1_C1S_Msk            (0x7FUL << GPDMA_CSELR1_C1S_Pos)
#define GPDMA_CSELR1_C1S                GPDMA_CSELR1_C1S_Msk
#define GPDMA_CSELR1_C2S_Pos            (8U)
#define GPDMA_CSELR1_C2S_Msk            (0x7FUL << GPDMA_CSELR1_C2S_Pos)
#define GPDMA_CSELR1_C2S                GPDMA_CSELR1_C2S_Msk
#define GPDMA_CSELR1_C3S_Pos            (16U)
#define GPDMA_CSELR1_C3S_Msk            (0x7FUL << GPDMA_CSELR1_C3S_Pos)
#define GPDMA_CSELR1_C3S                GPDMA_CSELR1_C3S_Msk
#define GPDMA_CSELR1_C4S_Pos            (24U)
#define GPDMA_CSELR1_C4S_Msk            (0x7FUL << GPDMA_CSELR1_C4S_Pos)
#define GPDMA_CSELR1_C4S                GPDMA_CSELR1_C4S_Msk

/****************** Bit definition for GPDMA_CSELR2 register ******************/
#define GPDMA_CSELR2_C5S_Pos            (0U)
#define GPDMA_CSELR2_C5S_Msk            (0x7FUL << GPDMA_CSELR2_C5S_Pos)
#define GPDMA_CSELR2_C5S                GPDMA_CSELR2_C5S_Msk
#define GPDMA_CSELR2_C6S_Pos            (8U)
#define GPDMA_CSELR2_C6S_Msk            (0x7FUL << GPDMA_CSELR2_C6S_Pos)
#define GPDMA_CSELR2_C6S                GPDMA_CSELR2_C6S_Msk
#define GPDMA_CSELR2_C7S_Pos            (16U)
#define GPDMA_CSELR2_C7S_Msk            (0x7FUL << GPDMA_CSELR2_C7S_Pos)
#define GPDMA_CSELR2_C7S                GPDMA_CSELR2_C7S_Msk
#define GPDMA_CSELR2_C8S_Pos            (24U)
#define GPDMA_CSELR2_C8S_Msk            (0x7FUL << GPDMA_CSELR2_C8S_Pos)
#define GPDMA_CSELR2_C8S                GPDMA_CSELR2_C8S_Msk

/****************** Bit definition for GPDMA_DBGSEL register ******************/
#define GPDMA_DBGSEL_DBGSEL_Pos         (0U)
#define GPDMA_DBGSEL_DBGSEL_Msk         (0xFUL << GPDMA_DBGSEL_DBGSEL_Pos)
#define GPDMA_DBGSEL_DBGSEL             GPDMA_DBGSEL_DBGSEL_Msk

/****************** Bit definition for GPDMA_SELR1 register *******************/
#define GPDMA_SELR1_CS_Pos              (0U)
#define GPDMA_SELR1_CS_Msk              (0x7FUL << GPDMA_SELR1_CS_Pos)
#define GPDMA_SELR1_CS                  GPDMA_SELR1_CS_Msk
#define GPDMA_SELR1_TS_Pos              (8U)
#define GPDMA_SELR1_TS_Msk              (0xFFUL << GPDMA_SELR1_TS_Pos)
#define GPDMA_SELR1_TS                  GPDMA_SELR1_TS_Msk

/****************** Bit definition for GPDMA_SELR2 register *******************/
#define GPDMA_SELR2_CS_Pos              (0U)
#define GPDMA_SELR2_CS_Msk              (0x7FUL << GPDMA_SELR2_CS_Pos)
#define GPDMA_SELR2_CS                  GPDMA_SELR2_CS_Msk
#define GPDMA_SELR2_TS_Pos              (8U)
#define GPDMA_SELR2_TS_Msk              (0xFFUL << GPDMA_SELR2_TS_Pos)
#define GPDMA_SELR2_TS                  GPDMA_SELR2_TS_Msk

/****************** Bit definition for GPDMA_SELR3 register *******************/
#define GPDMA_SELR3_CS_Pos              (0U)
#define GPDMA_SELR3_CS_Msk              (0x7FUL << GPDMA_SELR3_CS_Pos)
#define GPDMA_SELR3_CS                  GPDMA_SELR3_CS_Msk
#define GPDMA_SELR3_TS_Pos              (8U)
#define GPDMA_SELR3_TS_Msk              (0xFFUL << GPDMA_SELR3_TS_Pos)
#define GPDMA_SELR3_TS                  GPDMA_SELR3_TS_Msk

/****************** Bit definition for GPDMA_SELR4 register *******************/
#define GPDMA_SELR4_CS_Pos              (0U)
#define GPDMA_SELR4_CS_Msk              (0x7FUL << GPDMA_SELR4_CS_Pos)
#define GPDMA_SELR4_CS                  GPDMA_SELR4_CS_Msk
#define GPDMA_SELR4_TS_Pos              (8U)
#define GPDMA_SELR4_TS_Msk              (0xFFUL << GPDMA_SELR4_TS_Pos)
#define GPDMA_SELR4_TS                  GPDMA_SELR4_TS_Msk

/****************** Bit definition for GPDMA_SELR5 register *******************/
#define GPDMA_SELR5_CS_Pos              (0U)
#define GPDMA_SELR5_CS_Msk              (0x7FUL << GPDMA_SELR5_CS_Pos)
#define GPDMA_SELR5_CS                  GPDMA_SELR5_CS_Msk
#define GPDMA_SELR5_TS_Pos              (8U)
#define GPDMA_SELR5_TS_Msk              (0xFFUL << GPDMA_SELR5_TS_Pos)
#define GPDMA_SELR5_TS                  GPDMA_SELR5_TS_Msk

/****************** Bit definition for GPDMA_SELR6 register *******************/
#define GPDMA_SELR6_CS_Pos              (0U)
#define GPDMA_SELR6_CS_Msk              (0x7FUL << GPDMA_SELR6_CS_Pos)
#define GPDMA_SELR6_CS                  GPDMA_SELR6_CS_Msk
#define GPDMA_SELR6_TS_Pos              (8U)
#define GPDMA_SELR6_TS_Msk              (0xFFUL << GPDMA_SELR6_TS_Pos)
#define GPDMA_SELR6_TS                  GPDMA_SELR6_TS_Msk

/****************** Bit definition for GPDMA_SELR7 register *******************/
#define GPDMA_SELR7_CS_Pos              (0U)
#define GPDMA_SELR7_CS_Msk              (0x7FUL << GPDMA_SELR7_CS_Pos)
#define GPDMA_SELR7_CS                  GPDMA_SELR7_CS_Msk
#define GPDMA_SELR7_TS_Pos              (8U)
#define GPDMA_SELR7_TS_Msk              (0xFFUL << GPDMA_SELR7_TS_Pos)
#define GPDMA_SELR7_TS                  GPDMA_SELR7_TS_Msk

/****************** Bit definition for GPDMA_SELR8 register *******************/
#define GPDMA_SELR8_CS_Pos              (0U)
#define GPDMA_SELR8_CS_Msk              (0x7FUL << GPDMA_SELR8_CS_Pos)
#define GPDMA_SELR8_CS                  GPDMA_SELR8_CS_Msk
#define GPDMA_SELR8_TS_Pos              (8U)
#define GPDMA_SELR8_TS_Msk              (0xFFUL << GPDMA_SELR8_TS_Pos)
#define GPDMA_SELR8_TS                  GPDMA_SELR8_TS_Msk

/******************* Bit definition for GPDMA_LISR register *******************/
#define GPDMA_LISR_LCIF1_Pos            (0U)
#define GPDMA_LISR_LCIF1_Msk            (0x1UL << GPDMA_LISR_LCIF1_Pos)
#define GPDMA_LISR_LCIF1                GPDMA_LISR_LCIF1_Msk
#define GPDMA_LISR_LCIF2_Pos            (1U)
#define GPDMA_LISR_LCIF2_Msk            (0x1UL << GPDMA_LISR_LCIF2_Pos)
#define GPDMA_LISR_LCIF2                GPDMA_LISR_LCIF2_Msk
#define GPDMA_LISR_LCIF3_Pos            (2U)
#define GPDMA_LISR_LCIF3_Msk            (0x1UL << GPDMA_LISR_LCIF3_Pos)
#define GPDMA_LISR_LCIF3                GPDMA_LISR_LCIF3_Msk
#define GPDMA_LISR_LCIF4_Pos            (3U)
#define GPDMA_LISR_LCIF4_Msk            (0x1UL << GPDMA_LISR_LCIF4_Pos)
#define GPDMA_LISR_LCIF4                GPDMA_LISR_LCIF4_Msk
#define GPDMA_LISR_LCIF5_Pos            (4U)
#define GPDMA_LISR_LCIF5_Msk            (0x1UL << GPDMA_LISR_LCIF5_Pos)
#define GPDMA_LISR_LCIF5                GPDMA_LISR_LCIF5_Msk
#define GPDMA_LISR_LCIF6_Pos            (5U)
#define GPDMA_LISR_LCIF6_Msk            (0x1UL << GPDMA_LISR_LCIF6_Pos)
#define GPDMA_LISR_LCIF6                GPDMA_LISR_LCIF6_Msk
#define GPDMA_LISR_LCIF7_Pos            (6U)
#define GPDMA_LISR_LCIF7_Msk            (0x1UL << GPDMA_LISR_LCIF7_Pos)
#define GPDMA_LISR_LCIF7                GPDMA_LISR_LCIF7_Msk
#define GPDMA_LISR_LCIF8_Pos            (7U)
#define GPDMA_LISR_LCIF8_Msk            (0x1UL << GPDMA_LISR_LCIF8_Pos)
#define GPDMA_LISR_LCIF8                GPDMA_LISR_LCIF8_Msk

/****************** Bit definition for GPDMA_LIFCR register *******************/
#define GPDMA_LIFCR_CLCIF1_Pos          (0U)
#define GPDMA_LIFCR_CLCIF1_Msk          (0x1UL << GPDMA_LIFCR_CLCIF1_Pos)
#define GPDMA_LIFCR_CLCIF1              GPDMA_LIFCR_CLCIF1_Msk
#define GPDMA_LIFCR_CLCIF2_Pos          (1U)
#define GPDMA_LIFCR_CLCIF2_Msk          (0x1UL << GPDMA_LIFCR_CLCIF2_Pos)
#define GPDMA_LIFCR_CLCIF2              GPDMA_LIFCR_CLCIF2_Msk
#define GPDMA_LIFCR_CLCIF3_Pos          (2U)
#define GPDMA_LIFCR_CLCIF3_Msk          (0x1UL << GPDMA_LIFCR_CLCIF3_Pos)
#define GPDMA_LIFCR_CLCIF3              GPDMA_LIFCR_CLCIF3_Msk
#define GPDMA_LIFCR_CLCIF4_Pos          (3U)
#define GPDMA_LIFCR_CLCIF4_Msk          (0x1UL << GPDMA_LIFCR_CLCIF4_Pos)
#define GPDMA_LIFCR_CLCIF4              GPDMA_LIFCR_CLCIF4_Msk
#define GPDMA_LIFCR_CLCIF5_Pos          (4U)
#define GPDMA_LIFCR_CLCIF5_Msk          (0x1UL << GPDMA_LIFCR_CLCIF5_Pos)
#define GPDMA_LIFCR_CLCIF5              GPDMA_LIFCR_CLCIF5_Msk
#define GPDMA_LIFCR_CLCIF6_Pos          (5U)
#define GPDMA_LIFCR_CLCIF6_Msk          (0x1UL << GPDMA_LIFCR_CLCIF6_Pos)
#define GPDMA_LIFCR_CLCIF6              GPDMA_LIFCR_CLCIF6_Msk
#define GPDMA_LIFCR_CLCIF7_Pos          (6U)
#define GPDMA_LIFCR_CLCIF7_Msk          (0x1UL << GPDMA_LIFCR_CLCIF7_Pos)
#define GPDMA_LIFCR_CLCIF7              GPDMA_LIFCR_CLCIF7_Msk
#define GPDMA_LIFCR_CLCIF8_Pos          (7U)
#define GPDMA_LIFCR_CLCIF8_Msk          (0x1UL << GPDMA_LIFCR_CLCIF8_Pos)
#define GPDMA_LIFCR_CLCIF8              GPDMA_LIFCR_CLCIF8_Msk

/****************** Bit definition for GPDMA_CREPR1 register ******************/
#define GPDMA_CREPR1_CNT_Pos            (0U)
#define GPDMA_CREPR1_CNT_Msk            (0x3FFUL << GPDMA_CREPR1_CNT_Pos)
#define GPDMA_CREPR1_CNT                GPDMA_CREPR1_CNT_Msk

/****************** Bit definition for GPDMA_CINCR1 register ******************/
#define GPDMA_CINCR1_MINC_Pos           (0U)
#define GPDMA_CINCR1_MINC_Msk           (0xFFFFUL << GPDMA_CINCR1_MINC_Pos)
#define GPDMA_CINCR1_MINC               GPDMA_CINCR1_MINC_Msk
#define GPDMA_CINCR1_PINC_Pos           (16U)
#define GPDMA_CINCR1_PINC_Msk           (0xFFFFUL << GPDMA_CINCR1_PINC_Pos)
#define GPDMA_CINCR1_PINC               GPDMA_CINCR1_PINC_Msk

/******************* Bit definition for GPDMA_LCR1 register *******************/
#define GPDMA_LCR1_START_Pos            (0U)
#define GPDMA_LCR1_START_Msk            (0x1UL << GPDMA_LCR1_START_Pos)
#define GPDMA_LCR1_START                GPDMA_LCR1_START_Msk
#define GPDMA_LCR1_BUSY_Pos             (1U)
#define GPDMA_LCR1_BUSY_Msk             (0x1UL << GPDMA_LCR1_BUSY_Pos)
#define GPDMA_LCR1_BUSY                 GPDMA_LCR1_BUSY_Msk
#define GPDMA_LCR1_ADDR_Pos             (2U)
#define GPDMA_LCR1_ADDR_Msk             (0x3FFFFFFFUL << GPDMA_LCR1_ADDR_Pos)
#define GPDMA_LCR1_ADDR                 GPDMA_LCR1_ADDR_Msk

/****************** Bit definition for GPDMA_CREPR2 register ******************/
#define GPDMA_CREPR2_CNT_Pos            (0U)
#define GPDMA_CREPR2_CNT_Msk            (0x3FFUL << GPDMA_CREPR2_CNT_Pos)
#define GPDMA_CREPR2_CNT                GPDMA_CREPR2_CNT_Msk

/****************** Bit definition for GPDMA_CINCR2 register ******************/
#define GPDMA_CINCR2_MINC_Pos           (0U)
#define GPDMA_CINCR2_MINC_Msk           (0xFFFFUL << GPDMA_CINCR2_MINC_Pos)
#define GPDMA_CINCR2_MINC               GPDMA_CINCR2_MINC_Msk
#define GPDMA_CINCR2_PINC_Pos           (16U)
#define GPDMA_CINCR2_PINC_Msk           (0xFFFFUL << GPDMA_CINCR2_PINC_Pos)
#define GPDMA_CINCR2_PINC               GPDMA_CINCR2_PINC_Msk

/******************* Bit definition for GPDMA_LCR2 register *******************/
#define GPDMA_LCR2_START_Pos            (0U)
#define GPDMA_LCR2_START_Msk            (0x1UL << GPDMA_LCR2_START_Pos)
#define GPDMA_LCR2_START                GPDMA_LCR2_START_Msk
#define GPDMA_LCR2_BUSY_Pos             (1U)
#define GPDMA_LCR2_BUSY_Msk             (0x1UL << GPDMA_LCR2_BUSY_Pos)
#define GPDMA_LCR2_BUSY                 GPDMA_LCR2_BUSY_Msk
#define GPDMA_LCR2_ADDR_Pos             (2U)
#define GPDMA_LCR2_ADDR_Msk             (0x3FFFFFFFUL << GPDMA_LCR2_ADDR_Pos)
#define GPDMA_LCR2_ADDR                 GPDMA_LCR2_ADDR_Msk

/******************* Bit definition for GPDMA_LCR3 register *******************/
#define GPDMA_LCR3_START_Pos            (0U)
#define GPDMA_LCR3_START_Msk            (0x1UL << GPDMA_LCR3_START_Pos)
#define GPDMA_LCR3_START                GPDMA_LCR3_START_Msk
#define GPDMA_LCR3_BUSY_Pos             (1U)
#define GPDMA_LCR3_BUSY_Msk             (0x1UL << GPDMA_LCR3_BUSY_Pos)
#define GPDMA_LCR3_BUSY                 GPDMA_LCR3_BUSY_Msk
#define GPDMA_LCR3_ADDR_Pos             (2U)
#define GPDMA_LCR3_ADDR_Msk             (0x3FFFFFFFUL << GPDMA_LCR3_ADDR_Pos)
#define GPDMA_LCR3_ADDR                 GPDMA_LCR3_ADDR_Msk

/******************* Bit definition for GPDMA_LCR4 register *******************/
#define GPDMA_LCR4_START_Pos            (0U)
#define GPDMA_LCR4_START_Msk            (0x1UL << GPDMA_LCR4_START_Pos)
#define GPDMA_LCR4_START                GPDMA_LCR4_START_Msk
#define GPDMA_LCR4_BUSY_Pos             (1U)
#define GPDMA_LCR4_BUSY_Msk             (0x1UL << GPDMA_LCR4_BUSY_Pos)
#define GPDMA_LCR4_BUSY                 GPDMA_LCR4_BUSY_Msk
#define GPDMA_LCR4_ADDR_Pos             (2U)
#define GPDMA_LCR4_ADDR_Msk             (0x3FFFFFFFUL << GPDMA_LCR4_ADDR_Pos)
#define GPDMA_LCR4_ADDR                 GPDMA_LCR4_ADDR_Msk

/******************* Bit definition for GPDMA_LCR5 register *******************/
#define GPDMA_LCR5_START_Pos            (0U)
#define GPDMA_LCR5_START_Msk            (0x1UL << GPDMA_LCR5_START_Pos)
#define GPDMA_LCR5_START                GPDMA_LCR5_START_Msk
#define GPDMA_LCR5_BUSY_Pos             (1U)
#define GPDMA_LCR5_BUSY_Msk             (0x1UL << GPDMA_LCR5_BUSY_Pos)
#define GPDMA_LCR5_BUSY                 GPDMA_LCR5_BUSY_Msk
#define GPDMA_LCR5_ADDR_Pos             (2U)
#define GPDMA_LCR5_ADDR_Msk             (0x3FFFFFFFUL << GPDMA_LCR5_ADDR_Pos)
#define GPDMA_LCR5_ADDR                 GPDMA_LCR5_ADDR_Msk

/******************* Bit definition for GPDMA_LCR6 register *******************/
#define GPDMA_LCR6_START_Pos            (0U)
#define GPDMA_LCR6_START_Msk            (0x1UL << GPDMA_LCR6_START_Pos)
#define GPDMA_LCR6_START                GPDMA_LCR6_START_Msk
#define GPDMA_LCR6_BUSY_Pos             (1U)
#define GPDMA_LCR6_BUSY_Msk             (0x1UL << GPDMA_LCR6_BUSY_Pos)
#define GPDMA_LCR6_BUSY                 GPDMA_LCR6_BUSY_Msk
#define GPDMA_LCR6_ADDR_Pos             (2U)
#define GPDMA_LCR6_ADDR_Msk             (0x3FFFFFFFUL << GPDMA_LCR6_ADDR_Pos)
#define GPDMA_LCR6_ADDR                 GPDMA_LCR6_ADDR_Msk

/******************* Bit definition for GPDMA_LCR7 register *******************/
#define GPDMA_LCR7_START_Pos            (0U)
#define GPDMA_LCR7_START_Msk            (0x1UL << GPDMA_LCR7_START_Pos)
#define GPDMA_LCR7_START                GPDMA_LCR7_START_Msk
#define GPDMA_LCR7_BUSY_Pos             (1U)
#define GPDMA_LCR7_BUSY_Msk             (0x1UL << GPDMA_LCR7_BUSY_Pos)
#define GPDMA_LCR7_BUSY                 GPDMA_LCR7_BUSY_Msk
#define GPDMA_LCR7_ADDR_Pos             (2U)
#define GPDMA_LCR7_ADDR_Msk             (0x3FFFFFFFUL << GPDMA_LCR7_ADDR_Pos)
#define GPDMA_LCR7_ADDR                 GPDMA_LCR7_ADDR_Msk

/******************* Bit definition for GPDMA_LCR8 register *******************/
#define GPDMA_LCR8_START_Pos            (0U)
#define GPDMA_LCR8_START_Msk            (0x1UL << GPDMA_LCR8_START_Pos)
#define GPDMA_LCR8_START                GPDMA_LCR8_START_Msk
#define GPDMA_LCR8_BUSY_Pos             (1U)
#define GPDMA_LCR8_BUSY_Msk             (0x1UL << GPDMA_LCR8_BUSY_Pos)
#define GPDMA_LCR8_BUSY                 GPDMA_LCR8_BUSY_Msk
#define GPDMA_LCR8_ADDR_Pos             (2U)
#define GPDMA_LCR8_ADDR_Msk             (0x3FFFFFFFUL << GPDMA_LCR8_ADDR_Pos)
#define GPDMA_LCR8_ADDR                 GPDMA_LCR8_ADDR_Msk

/******************* Bit definition for GPDMA Link-List Node CONTROL Register (1st word) *******************/
#define GPDMA_LL_CONTROL_NEXT_Pos            (31U)
#define GPDMA_LL_CONTROL_NEXT_Msk            (0x1UL << GPDMA_LL_CONTROL_NEXT_Pos)
#define GPDMA_LL_CONTROL_NEXT                GPDMA_LL_CONTROL_NEXT_Msk
#define GPDMA_LL_CONTROL_DELAY_Pos           (0U)
#define GPDMA_LL_CONTROL_DELAY_Msk           (0xFUL << GPDMA_LL_CONTROL_DELAY_Pos)
#define GPDMA_LL_CONTROL_DELAY               GPDMA_LL_CONTROL_DELAY_Msk



/******************* Bit definition for GPDMA Link-List Node MISC register (4th word) *******************/
#define GPDMA_LL_MISC_NDT_Pos            (0U)
#define GPDMA_LL_MISC_NDT_Msk            (0xFFFFUL << GPDMA_LL_MISC_NDT_Pos)
#define GPDMA_LL_MISC_NDT                GPDMA_LL_MISC_NDT_Msk
#define GPDMA_LL_MISC_CS_Pos             (16U)
#define GPDMA_LL_MISC_CS_Msk             (0x3FUL << GPDMA_LL_MISC_CS_Pos)
#define GPDMA_LL_MISC_CS                 GPDMA_LL_MISC_CS_Msk
#define GPDMA_LL_MISC_TS_Pos             (24U)
#define GPDMA_LL_MISC_TS_Msk             (0xFFUL << GPDMA_LL_MISC_TS_Pos)
#define GPDMA_LL_MISC_TS                 GPDMA_LL_MISC_TS_Msk

#define GPDMA_TS_RSVD_0                  (0UL)
#define GPDMA_TS_HCPU_SLEEPING           (1UL)
#define GPDMA_TS_GPTIM1_UPDATE           (2UL)
#define GPDMA_TS_GPTIM1_TRIG             (3UL)
#define GPDMA_TS_GPTIM1_CH1              (4UL)
#define GPDMA_TS_GPTIM1_CH2              (5UL)
#define GPDMA_TS_GPTIM1_CH3              (6UL)
#define GPDMA_TS_GPTIM1_CH4              (7UL)
#define GPDMA_TS_GPTIM2_UPDATE           (8UL)
#define GPDMA_TS_GPTIM2_TRIG             (9UL)
#define GPDMA_TS_GPTIM2_CH1              (10UL)
#define GPDMA_TS_GPTIM2_CH2              (11UL)
#define GPDMA_TS_RSVD_12                 (12UL)
#define GPDMA_TS_RSVD_13                 (13UL)
#define GPDMA_TS_BTIM1_UPDATE            (14UL)
#define GPDMA_TS_BTIM2_UPDATE            (15UL)
#define GPDMA_TS_ATIM1_UPDATE            (16UL)
#define GPDMA_TS_ATIM1_TRIG              (17UL)
#define GPDMA_TS_ATIM1_CH1               (18UL)
#define GPDMA_TS_ATIM1_CH2               (19UL)
#define GPDMA_TS_ATIM1_CH3               (20UL)
#define GPDMA_TS_ATIM1_CH4               (21UL)
#define GPDMA_TS_ATIM1_COM               (22UL)
#define GPDMA_TS_CAN1_BUSY               (23UL)
#define GPDMA_TS_DMAC1_DONE1             (24UL)
#define GPDMA_TS_DMAC1_DONE2             (25UL)
#define GPDMA_TS_DMAC1_DONE3             (26UL)
#define GPDMA_TS_DMAC1_DONE4             (27UL)
#define GPDMA_TS_DMAC1_DONE5             (28UL)
#define GPDMA_TS_DMAC1_DONE6             (29UL)
#define GPDMA_TS_DMAC1_DONE7             (30UL)
#define GPDMA_TS_DMAC1_DONE8             (31UL)
#define GPDMA_TS_DMAC1_HALF1             (32UL)
#define GPDMA_TS_DMAC1_HALF2             (33UL)
#define GPDMA_TS_DMAC1_HALF3             (34UL)
#define GPDMA_TS_DMAC1_HALF4             (35UL)
#define GPDMA_TS_DMAC1_HALF5             (36UL)
#define GPDMA_TS_DMAC1_HALF6             (37UL)
#define GPDMA_TS_DMAC1_HALF7             (38UL)
#define GPDMA_TS_DMAC1_HALF8             (39UL)
#define GPDMA_TS_DMAC1_LDONE1            (40UL)
#define GPDMA_TS_DMAC1_LDONE2            (41UL)
#define GPDMA_TS_DMAC1_LDONE3            (42UL)
#define GPDMA_TS_DMAC1_LDONE4            (43UL)
#define GPDMA_TS_DMAC1_LDONE5            (44UL)
#define GPDMA_TS_DMAC1_LDONE6            (45UL)
#define GPDMA_TS_DMAC1_LDONE7            (46UL)
#define GPDMA_TS_DMAC1_LDONE8            (47UL)
#define GPDMA_TS_DCMI_VSYNC              (48UL)
#define GPDMA_TS_DCMI_FRAMESYNC          (49UL)
#define GPDMA_TS_BT_RX_EN                (50UL)
#define GPDMA_TS_BT_TX_EN                (51UL)
#define GPDMA_TS_MAILBOX1_C1IRQ7         (52UL)
#define GPDMA_TS_MAILBOX1_C2IRQ7         (53UL)
#define GPDMA_TS_MAILBOX1_C3IRQ7         (54UL)
#define GPDMA_TS_MAILBOX1_C4IRQ7         (55UL)
#define GPDMA_TS_PWM1_UPDATE1            (56UL)
#define GPDMA_TS_PWM1_UPDATE2            (57UL)
#define GPDMA_TS_PWM1_UPDATE3            (58UL)
#define GPDMA_TS_PWM1_UPDATE4            (59UL)
#define GPDMA_TS_PWM1_UPDATE5            (60UL)
#define GPDMA_TS_ATIM2_UPDATE            (61UL)
#define GPDMA_TS_ATIM2_TRIG              (62UL)
#define GPDMA_TS_ATIM2_CH1               (63UL)
#define GPDMA_TS_BT_ACTIVE               (64UL)
#define GPDMA_TS_BT_PRIORITY             (65UL)
#define GPDMA_TS_BT_COLLISION            (66UL)
#define GPDMA_TS_WLAN_ACTIVE             (67UL)
#define GPDMA_TS_I2C1_ATDONE             (68UL)
#define GPDMA_TS_I2C1_DMADONE            (69UL)
#define GPDMA_TS_I2C1_TXEMPTY            (70UL)
#define GPDMA_TS_I2C1_RXFULL             (71UL)
#define GPDMA_TS_I2C2_ATDONE             (72UL)
#define GPDMA_TS_I2C2_DMADONE            (73UL)
#define GPDMA_TS_I2C2_TXEMPTY            (74UL)
#define GPDMA_TS_I2C2_RXFULL             (75UL)
#define GPDMA_TS_I2C3_ATDONE             (76UL)
#define GPDMA_TS_I2C3_DMADONE            (77UL)
#define GPDMA_TS_I2C3_TXEMPTY            (78UL)
#define GPDMA_TS_I2C3_RXFULL             (79UL)
#define GPDMA_TS_SPI1_DONE               (80UL)
#define GPDMA_TS_SPI1_START              (81UL)
#define GPDMA_TS_SPI2_DONE               (82UL)
#define GPDMA_TS_SPI2_START              (83UL)
#define GPDMA_TS_SDMMC1_CMDDONE          (84UL)
#define GPDMA_TS_SDMMC1_DATDONE          (85UL)
#define GPDMA_TS_SDMMC2_CMDDONE          (86UL)
#define GPDMA_TS_SDMMC2_DATDONE          (87UL)
#define GPDMA_TS_USART1_RXBYTE           (88UL)
#define GPDMA_TS_USART1_TXBYTE           (89UL)
#define GPDMA_TS_USART2_RXBYTE           (90UL)
#define GPDMA_TS_USART2_TXBYTE           (91UL)
#define GPDMA_TS_I2C4_ATDONE             (92UL)
#define GPDMA_TS_I2C4_DMADONE            (93UL)
#define GPDMA_TS_I2C4_TXEMPTY            (94UL)
#define GPDMA_TS_I2C4_RXFULL             (95UL)
#define GPDMA_TS_USB_TX                  (96UL)
#define GPDMA_TS_USB_RX                  (97UL)
#define GPDMA_TS_USART3_RXBYTE           (98UL)
#define GPDMA_TS_USART3_TXBYTE           (99UL)
#define GPDMA_TS_EZIP1_DONE              (100UL)
#define GPDMA_TS_EZIP1_ROW               (101UL)
#define GPDMA_TS_TSEN_DONE               (102UL)
#define GPDMA_TS_LCDC1_BUSY              (103UL)
#define GPDMA_TS_LCDC1_DONE              (104UL)
#define GPDMA_TS_LCDC1_FMARK             (105UL)
#define GPDMA_TS_LCDC1_LINEHIT           (106UL)
#define GPDMA_TS_LCDC1_LINE              (107UL)
#define GPDMA_TS_LCDC1_ERR               (108UL)
#define GPDMA_TS_JPEGD_INT               (109UL)
#define GPDMA_TS_EPIC_DONE               (110UL)
#define GPDMA_TS_EPIC_LINEHIT            (111UL)
#define GPDMA_TS_HCPU_SLEEPDEEP          (112UL)
#define GPDMA_TS_TRNG_SEEDGEN            (113UL)
#define GPDMA_TS_TRNG_RANDGEN            (114UL)
#define GPDMA_TS_AES_DONE                (115UL)
#define GPDMA_TS_I2S1_UF                 (116UL)
#define GPDMA_TS_I2S1_OF                 (117UL)
#define GPDMA_TS_I2S2_UF                 (118UL)
#define GPDMA_TS_I2S2_OF                 (119UL)
#define GPDMA_TS_RSVD_120                (120UL)
#define GPDMA_TS_RSVD_121                (121UL)
#define GPDMA_TS_RSVD_122                (122UL)
#define GPDMA_TS_RSVD_123                (123UL)
#define GPDMA_TS_RSVD_124                (124UL)
#define GPDMA_TS_RSVD_125                (125UL)
#define GPDMA_TS_RSVD_126                (126UL)
#define GPDMA_TS_RSVD_127                (127UL)


#endif
