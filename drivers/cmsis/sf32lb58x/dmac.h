/**
  ******************************************************************************
  * @file   dmac.h
  * @author Sifli software development team
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2019 - 2022,  Sifli Technology
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Sifli integrated circuit
 *    in a product or a software update for such product, must reproduce the above
 *    copyright notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of Sifli nor the names of its contributors may be used to endorse
 *    or promote products derived from this software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Sifli integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY SIFLI TECHNOLOGY "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL SIFLI TECHNOLOGY OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef __DMAC_H
#define __DMAC_H

typedef struct
{
    __IO uint32_t ISR;
    __IO uint32_t IFCR;
    __IO uint32_t CCR1;
    __IO uint32_t CNDTR1;
    __IO uint32_t CPAR1;
    __IO uint32_t CM0AR1;
    __IO uint32_t CBSR1;
    __IO uint32_t CCR2;
    __IO uint32_t CNDTR2;
    __IO uint32_t CPAR2;
    __IO uint32_t CM0AR2;
    __IO uint32_t CBSR2;
    __IO uint32_t CCR3;
    __IO uint32_t CNDTR3;
    __IO uint32_t CPAR3;
    __IO uint32_t CM0AR3;
    __IO uint32_t CBSR3;
    __IO uint32_t CCR4;
    __IO uint32_t CNDTR4;
    __IO uint32_t CPAR4;
    __IO uint32_t CM0AR4;
    __IO uint32_t CBSR4;
    __IO uint32_t CCR5;
    __IO uint32_t CNDTR5;
    __IO uint32_t CPAR5;
    __IO uint32_t CM0AR5;
    __IO uint32_t CBSR5;
    __IO uint32_t CCR6;
    __IO uint32_t CNDTR6;
    __IO uint32_t CPAR6;
    __IO uint32_t CM0AR6;
    __IO uint32_t CBSR6;
    __IO uint32_t CCR7;
    __IO uint32_t CNDTR7;
    __IO uint32_t CPAR7;
    __IO uint32_t CM0AR7;
    __IO uint32_t CBSR7;
    __IO uint32_t CCR8;
    __IO uint32_t CNDTR8;
    __IO uint32_t CPAR8;
    __IO uint32_t CM0AR8;
    __IO uint32_t CBSR8;
    __IO uint32_t CSELR1;
    __IO uint32_t CSELR2;
    __IO uint32_t DBGSEL;
} DMAC_TypeDef;

typedef struct
{
    __IO uint32_t CCR;         /*!< DMA channel x configuration register        */
    __IO uint32_t CNDTR;       /*!< DMA channel x number of data register       */
    __IO uint32_t CPAR;        /*!< DMA channel x peripheral address register   */
    __IO uint32_t CM0AR;        /*!< DMA channel x memory address register       */
    __IO uint32_t CBSR;
} DMA_Channel_TypeDef;

typedef struct
{
    __IO uint32_t CSELR1;       /*!< DMA channel selection1 register              */
    __IO uint32_t CSELR2;       /*!< DMA channel selection2 register              */
} DMA_Request_TypeDef;



/****************** Bit definition for DMAC_ISR register ******************/
#define DMAC_ISR_GIF1_Pos           (0U)
#define DMAC_ISR_GIF1_Msk           (0x1UL << DMAC_ISR_GIF1_Pos)
#define DMAC_ISR_GIF1               DMAC_ISR_GIF1_Msk
#define DMA_ISR_GIF1                DMAC_ISR_GIF1
#define DMAC_ISR_TCIF1_Pos          (1U)
#define DMAC_ISR_TCIF1_Msk          (0x1UL << DMAC_ISR_TCIF1_Pos)
#define DMAC_ISR_TCIF1              DMAC_ISR_TCIF1_Msk
#define DMA_ISR_TCIF1               DMAC_ISR_TCIF1
#define DMAC_ISR_HTIF1_Pos          (2U)
#define DMAC_ISR_HTIF1_Msk          (0x1UL << DMAC_ISR_HTIF1_Pos)
#define DMAC_ISR_HTIF1              DMAC_ISR_HTIF1_Msk
#define DMA_ISR_HTIF1               DMAC_ISR_HTIF1
#define DMAC_ISR_TEIF1_Pos          (3U)
#define DMAC_ISR_TEIF1_Msk          (0x1UL << DMAC_ISR_TEIF1_Pos)
#define DMAC_ISR_TEIF1              DMAC_ISR_TEIF1_Msk
#define DMA_ISR_TEIF1               DMAC_ISR_TEIF1
#define DMAC_ISR_GIF2_Pos           (4U)
#define DMAC_ISR_GIF2_Msk           (0x1UL << DMAC_ISR_GIF2_Pos)
#define DMAC_ISR_GIF2               DMAC_ISR_GIF2_Msk
#define DMAC_ISR_TCIF2_Pos          (5U)
#define DMAC_ISR_TCIF2_Msk          (0x1UL << DMAC_ISR_TCIF2_Pos)
#define DMAC_ISR_TCIF2              DMAC_ISR_TCIF2_Msk
#define DMAC_ISR_HTIF2_Pos          (6U)
#define DMAC_ISR_HTIF2_Msk          (0x1UL << DMAC_ISR_HTIF2_Pos)
#define DMAC_ISR_HTIF2              DMAC_ISR_HTIF2_Msk
#define DMAC_ISR_TEIF2_Pos          (7U)
#define DMAC_ISR_TEIF2_Msk          (0x1UL << DMAC_ISR_TEIF2_Pos)
#define DMAC_ISR_TEIF2              DMAC_ISR_TEIF2_Msk
#define DMAC_ISR_GIF3_Pos           (8U)
#define DMAC_ISR_GIF3_Msk           (0x1UL << DMAC_ISR_GIF3_Pos)
#define DMAC_ISR_GIF3               DMAC_ISR_GIF3_Msk
#define DMAC_ISR_TCIF3_Pos          (9U)
#define DMAC_ISR_TCIF3_Msk          (0x1UL << DMAC_ISR_TCIF3_Pos)
#define DMAC_ISR_TCIF3              DMAC_ISR_TCIF3_Msk
#define DMAC_ISR_HTIF3_Pos          (10U)
#define DMAC_ISR_HTIF3_Msk          (0x1UL << DMAC_ISR_HTIF3_Pos)
#define DMAC_ISR_HTIF3              DMAC_ISR_HTIF3_Msk
#define DMAC_ISR_TEIF3_Pos          (11U)
#define DMAC_ISR_TEIF3_Msk          (0x1UL << DMAC_ISR_TEIF3_Pos)
#define DMAC_ISR_TEIF3              DMAC_ISR_TEIF3_Msk
#define DMAC_ISR_GIF4_Pos           (12U)
#define DMAC_ISR_GIF4_Msk           (0x1UL << DMAC_ISR_GIF4_Pos)
#define DMAC_ISR_GIF4               DMAC_ISR_GIF4_Msk
#define DMAC_ISR_TCIF4_Pos          (13U)
#define DMAC_ISR_TCIF4_Msk          (0x1UL << DMAC_ISR_TCIF4_Pos)
#define DMAC_ISR_TCIF4              DMAC_ISR_TCIF4_Msk
#define DMAC_ISR_HTIF4_Pos          (14U)
#define DMAC_ISR_HTIF4_Msk          (0x1UL << DMAC_ISR_HTIF4_Pos)
#define DMAC_ISR_HTIF4              DMAC_ISR_HTIF4_Msk
#define DMAC_ISR_TEIF4_Pos          (15U)
#define DMAC_ISR_TEIF4_Msk          (0x1UL << DMAC_ISR_TEIF4_Pos)
#define DMAC_ISR_TEIF4              DMAC_ISR_TEIF4_Msk
#define DMAC_ISR_GIF5_Pos           (16U)
#define DMAC_ISR_GIF5_Msk           (0x1UL << DMAC_ISR_GIF5_Pos)
#define DMAC_ISR_GIF5               DMAC_ISR_GIF5_Msk
#define DMAC_ISR_TCIF5_Pos          (17U)
#define DMAC_ISR_TCIF5_Msk          (0x1UL << DMAC_ISR_TCIF5_Pos)
#define DMAC_ISR_TCIF5              DMAC_ISR_TCIF5_Msk
#define DMAC_ISR_HTIF5_Pos          (18U)
#define DMAC_ISR_HTIF5_Msk          (0x1UL << DMAC_ISR_HTIF5_Pos)
#define DMAC_ISR_HTIF5              DMAC_ISR_HTIF5_Msk
#define DMAC_ISR_TEIF5_Pos          (19U)
#define DMAC_ISR_TEIF5_Msk          (0x1UL << DMAC_ISR_TEIF5_Pos)
#define DMAC_ISR_TEIF5              DMAC_ISR_TEIF5_Msk
#define DMAC_ISR_GIF6_Pos           (20U)
#define DMAC_ISR_GIF6_Msk           (0x1UL << DMAC_ISR_GIF6_Pos)
#define DMAC_ISR_GIF6               DMAC_ISR_GIF6_Msk
#define DMAC_ISR_TCIF6_Pos          (21U)
#define DMAC_ISR_TCIF6_Msk          (0x1UL << DMAC_ISR_TCIF6_Pos)
#define DMAC_ISR_TCIF6              DMAC_ISR_TCIF6_Msk
#define DMAC_ISR_HTIF6_Pos          (22U)
#define DMAC_ISR_HTIF6_Msk          (0x1UL << DMAC_ISR_HTIF6_Pos)
#define DMAC_ISR_HTIF6              DMAC_ISR_HTIF6_Msk
#define DMAC_ISR_TEIF6_Pos          (23U)
#define DMAC_ISR_TEIF6_Msk          (0x1UL << DMAC_ISR_TEIF6_Pos)
#define DMAC_ISR_TEIF6              DMAC_ISR_TEIF6_Msk
#define DMAC_ISR_GIF7_Pos           (24U)
#define DMAC_ISR_GIF7_Msk           (0x1UL << DMAC_ISR_GIF7_Pos)
#define DMAC_ISR_GIF7               DMAC_ISR_GIF7_Msk
#define DMAC_ISR_TCIF7_Pos          (25U)
#define DMAC_ISR_TCIF7_Msk          (0x1UL << DMAC_ISR_TCIF7_Pos)
#define DMAC_ISR_TCIF7              DMAC_ISR_TCIF7_Msk
#define DMAC_ISR_HTIF7_Pos          (26U)
#define DMAC_ISR_HTIF7_Msk          (0x1UL << DMAC_ISR_HTIF7_Pos)
#define DMAC_ISR_HTIF7              DMAC_ISR_HTIF7_Msk
#define DMAC_ISR_TEIF7_Pos          (27U)
#define DMAC_ISR_TEIF7_Msk          (0x1UL << DMAC_ISR_TEIF7_Pos)
#define DMAC_ISR_TEIF7              DMAC_ISR_TEIF7_Msk
#define DMAC_ISR_GIF8_Pos           (28U)
#define DMAC_ISR_GIF8_Msk           (0x1UL << DMAC_ISR_GIF8_Pos)
#define DMAC_ISR_GIF8               DMAC_ISR_GIF8_Msk
#define DMAC_ISR_TCIF8_Pos          (29U)
#define DMAC_ISR_TCIF8_Msk          (0x1UL << DMAC_ISR_TCIF8_Pos)
#define DMAC_ISR_TCIF8              DMAC_ISR_TCIF8_Msk
#define DMAC_ISR_HTIF8_Pos          (30U)
#define DMAC_ISR_HTIF8_Msk          (0x1UL << DMAC_ISR_HTIF8_Pos)
#define DMAC_ISR_HTIF8              DMAC_ISR_HTIF8_Msk
#define DMAC_ISR_TEIF8_Pos          (31U)
#define DMAC_ISR_TEIF8_Msk          (0x1UL << DMAC_ISR_TEIF8_Pos)
#define DMAC_ISR_TEIF8              DMAC_ISR_TEIF8_Msk

/***************** Bit definition for DMAC_IFCR register ******************/
#define DMAC_IFCR_CGIF1_Pos         (0U)
#define DMAC_IFCR_CGIF1_Msk         (0x1UL << DMAC_IFCR_CGIF1_Pos)
#define DMAC_IFCR_CGIF1             DMAC_IFCR_CGIF1_Msk
#define DMAC_IFCR_CTCIF1_Pos        (1U)
#define DMAC_IFCR_CTCIF1_Msk        (0x1UL << DMAC_IFCR_CTCIF1_Pos)
#define DMAC_IFCR_CTCIF1            DMAC_IFCR_CTCIF1_Msk
#define DMAC_IFCR_CHTIF1_Pos        (2U)
#define DMAC_IFCR_CHTIF1_Msk        (0x1UL << DMAC_IFCR_CHTIF1_Pos)
#define DMAC_IFCR_CHTIF1            DMAC_IFCR_CHTIF1_Msk
#define DMAC_IFCR_CTEIF1_Pos        (3U)
#define DMAC_IFCR_CTEIF1_Msk        (0x1UL << DMAC_IFCR_CTEIF1_Pos)
#define DMAC_IFCR_CTEIF1            DMAC_IFCR_CTEIF1_Msk
#define DMAC_IFCR_CGIF2_Pos         (4U)
#define DMAC_IFCR_CGIF2_Msk         (0x1UL << DMAC_IFCR_CGIF2_Pos)
#define DMAC_IFCR_CGIF2             DMAC_IFCR_CGIF2_Msk
#define DMAC_IFCR_CTCIF2_Pos        (5U)
#define DMAC_IFCR_CTCIF2_Msk        (0x1UL << DMAC_IFCR_CTCIF2_Pos)
#define DMAC_IFCR_CTCIF2            DMAC_IFCR_CTCIF2_Msk
#define DMAC_IFCR_CHTIF2_Pos        (6U)
#define DMAC_IFCR_CHTIF2_Msk        (0x1UL << DMAC_IFCR_CHTIF2_Pos)
#define DMAC_IFCR_CHTIF2            DMAC_IFCR_CHTIF2_Msk
#define DMAC_IFCR_CTEIF2_Pos        (7U)
#define DMAC_IFCR_CTEIF2_Msk        (0x1UL << DMAC_IFCR_CTEIF2_Pos)
#define DMAC_IFCR_CTEIF2            DMAC_IFCR_CTEIF2_Msk
#define DMAC_IFCR_CGIF3_Pos         (8U)
#define DMAC_IFCR_CGIF3_Msk         (0x1UL << DMAC_IFCR_CGIF3_Pos)
#define DMAC_IFCR_CGIF3             DMAC_IFCR_CGIF3_Msk
#define DMAC_IFCR_CTCIF3_Pos        (9U)
#define DMAC_IFCR_CTCIF3_Msk        (0x1UL << DMAC_IFCR_CTCIF3_Pos)
#define DMAC_IFCR_CTCIF3            DMAC_IFCR_CTCIF3_Msk
#define DMAC_IFCR_CHTIF3_Pos        (10U)
#define DMAC_IFCR_CHTIF3_Msk        (0x1UL << DMAC_IFCR_CHTIF3_Pos)
#define DMAC_IFCR_CHTIF3            DMAC_IFCR_CHTIF3_Msk
#define DMAC_IFCR_CTEIF3_Pos        (11U)
#define DMAC_IFCR_CTEIF3_Msk        (0x1UL << DMAC_IFCR_CTEIF3_Pos)
#define DMAC_IFCR_CTEIF3            DMAC_IFCR_CTEIF3_Msk
#define DMAC_IFCR_CGIF4_Pos         (12U)
#define DMAC_IFCR_CGIF4_Msk         (0x1UL << DMAC_IFCR_CGIF4_Pos)
#define DMAC_IFCR_CGIF4             DMAC_IFCR_CGIF4_Msk
#define DMAC_IFCR_CTCIF4_Pos        (13U)
#define DMAC_IFCR_CTCIF4_Msk        (0x1UL << DMAC_IFCR_CTCIF4_Pos)
#define DMAC_IFCR_CTCIF4            DMAC_IFCR_CTCIF4_Msk
#define DMAC_IFCR_CHTIF4_Pos        (14U)
#define DMAC_IFCR_CHTIF4_Msk        (0x1UL << DMAC_IFCR_CHTIF4_Pos)
#define DMAC_IFCR_CHTIF4            DMAC_IFCR_CHTIF4_Msk
#define DMAC_IFCR_CTEIF4_Pos        (15U)
#define DMAC_IFCR_CTEIF4_Msk        (0x1UL << DMAC_IFCR_CTEIF4_Pos)
#define DMAC_IFCR_CTEIF4            DMAC_IFCR_CTEIF4_Msk
#define DMAC_IFCR_CGIF5_Pos         (16U)
#define DMAC_IFCR_CGIF5_Msk         (0x1UL << DMAC_IFCR_CGIF5_Pos)
#define DMAC_IFCR_CGIF5             DMAC_IFCR_CGIF5_Msk
#define DMAC_IFCR_CTCIF5_Pos        (17U)
#define DMAC_IFCR_CTCIF5_Msk        (0x1UL << DMAC_IFCR_CTCIF5_Pos)
#define DMAC_IFCR_CTCIF5            DMAC_IFCR_CTCIF5_Msk
#define DMAC_IFCR_CHTIF5_Pos        (18U)
#define DMAC_IFCR_CHTIF5_Msk        (0x1UL << DMAC_IFCR_CHTIF5_Pos)
#define DMAC_IFCR_CHTIF5            DMAC_IFCR_CHTIF5_Msk
#define DMAC_IFCR_CTEIF5_Pos        (19U)
#define DMAC_IFCR_CTEIF5_Msk        (0x1UL << DMAC_IFCR_CTEIF5_Pos)
#define DMAC_IFCR_CTEIF5            DMAC_IFCR_CTEIF5_Msk
#define DMAC_IFCR_CGIF6_Pos         (20U)
#define DMAC_IFCR_CGIF6_Msk         (0x1UL << DMAC_IFCR_CGIF6_Pos)
#define DMAC_IFCR_CGIF6             DMAC_IFCR_CGIF6_Msk
#define DMAC_IFCR_CTCIF6_Pos        (21U)
#define DMAC_IFCR_CTCIF6_Msk        (0x1UL << DMAC_IFCR_CTCIF6_Pos)
#define DMAC_IFCR_CTCIF6            DMAC_IFCR_CTCIF6_Msk
#define DMAC_IFCR_CHTIF6_Pos        (22U)
#define DMAC_IFCR_CHTIF6_Msk        (0x1UL << DMAC_IFCR_CHTIF6_Pos)
#define DMAC_IFCR_CHTIF6            DMAC_IFCR_CHTIF6_Msk
#define DMAC_IFCR_CTEIF6_Pos        (23U)
#define DMAC_IFCR_CTEIF6_Msk        (0x1UL << DMAC_IFCR_CTEIF6_Pos)
#define DMAC_IFCR_CTEIF6            DMAC_IFCR_CTEIF6_Msk
#define DMAC_IFCR_CGIF7_Pos         (24U)
#define DMAC_IFCR_CGIF7_Msk         (0x1UL << DMAC_IFCR_CGIF7_Pos)
#define DMAC_IFCR_CGIF7             DMAC_IFCR_CGIF7_Msk
#define DMAC_IFCR_CTCIF7_Pos        (25U)
#define DMAC_IFCR_CTCIF7_Msk        (0x1UL << DMAC_IFCR_CTCIF7_Pos)
#define DMAC_IFCR_CTCIF7            DMAC_IFCR_CTCIF7_Msk
#define DMAC_IFCR_CHTIF7_Pos        (26U)
#define DMAC_IFCR_CHTIF7_Msk        (0x1UL << DMAC_IFCR_CHTIF7_Pos)
#define DMAC_IFCR_CHTIF7            DMAC_IFCR_CHTIF7_Msk
#define DMAC_IFCR_CTEIF7_Pos        (27U)
#define DMAC_IFCR_CTEIF7_Msk        (0x1UL << DMAC_IFCR_CTEIF7_Pos)
#define DMAC_IFCR_CTEIF7            DMAC_IFCR_CTEIF7_Msk
#define DMAC_IFCR_CGIF8_Pos         (28U)
#define DMAC_IFCR_CGIF8_Msk         (0x1UL << DMAC_IFCR_CGIF8_Pos)
#define DMAC_IFCR_CGIF8             DMAC_IFCR_CGIF8_Msk
#define DMAC_IFCR_CTCIF8_Pos        (29U)
#define DMAC_IFCR_CTCIF8_Msk        (0x1UL << DMAC_IFCR_CTCIF8_Pos)
#define DMAC_IFCR_CTCIF8            DMAC_IFCR_CTCIF8_Msk
#define DMAC_IFCR_CHTIF8_Pos        (30U)
#define DMAC_IFCR_CHTIF8_Msk        (0x1UL << DMAC_IFCR_CHTIF8_Pos)
#define DMAC_IFCR_CHTIF8            DMAC_IFCR_CHTIF8_Msk
#define DMAC_IFCR_CTEIF8_Pos        (31U)
#define DMAC_IFCR_CTEIF8_Msk        (0x1UL << DMAC_IFCR_CTEIF8_Pos)
#define DMAC_IFCR_CTEIF8            DMAC_IFCR_CTEIF8_Msk

/***************** Bit definition for DMAC_CCR1 register ******************/
#define DMAC_CCR1_EN_Pos            (0U)
#define DMAC_CCR1_EN_Msk            (0x1UL << DMAC_CCR1_EN_Pos)
#define DMAC_CCR1_EN                DMAC_CCR1_EN_Msk
#define DMAC_CCR1_TCIE_Pos          (1U)
#define DMAC_CCR1_TCIE_Msk          (0x1UL << DMAC_CCR1_TCIE_Pos)
#define DMAC_CCR1_TCIE              DMAC_CCR1_TCIE_Msk
#define DMAC_CCR1_HTIE_Pos          (2U)
#define DMAC_CCR1_HTIE_Msk          (0x1UL << DMAC_CCR1_HTIE_Pos)
#define DMAC_CCR1_HTIE              DMAC_CCR1_HTIE_Msk
#define DMAC_CCR1_TEIE_Pos          (3U)
#define DMAC_CCR1_TEIE_Msk          (0x1UL << DMAC_CCR1_TEIE_Pos)
#define DMAC_CCR1_TEIE              DMAC_CCR1_TEIE_Msk
#define DMAC_CCR1_DIR_Pos           (4U)
#define DMAC_CCR1_DIR_Msk           (0x1UL << DMAC_CCR1_DIR_Pos)
#define DMAC_CCR1_DIR               DMAC_CCR1_DIR_Msk
#define DMAC_CCR1_CIRC_Pos          (5U)
#define DMAC_CCR1_CIRC_Msk          (0x1UL << DMAC_CCR1_CIRC_Pos)
#define DMAC_CCR1_CIRC              DMAC_CCR1_CIRC_Msk
#define DMAC_CCR1_PINC_Pos          (6U)
#define DMAC_CCR1_PINC_Msk          (0x1UL << DMAC_CCR1_PINC_Pos)
#define DMAC_CCR1_PINC              DMAC_CCR1_PINC_Msk
#define DMAC_CCR1_MINC_Pos          (7U)
#define DMAC_CCR1_MINC_Msk          (0x1UL << DMAC_CCR1_MINC_Pos)
#define DMAC_CCR1_MINC              DMAC_CCR1_MINC_Msk
#define DMAC_CCR1_PSIZE_Pos         (8U)
#define DMAC_CCR1_PSIZE_Msk         (0x3UL << DMAC_CCR1_PSIZE_Pos)
#define DMAC_CCR1_PSIZE             DMAC_CCR1_PSIZE_Msk
#define DMAC_CCR1_PSIZE_0           (0x1UL << DMAC_CCR1_PSIZE_Pos)                    /*!< 0x00000100 */
#define DMAC_CCR1_PSIZE_1           (0x2UL << DMAC_CCR1_PSIZE_Pos)                    /*!< 0x00000200 */
#define DMAC_CCR1_MSIZE_Pos         (10U)
#define DMAC_CCR1_MSIZE_Msk         (0x3UL << DMAC_CCR1_MSIZE_Pos)
#define DMAC_CCR1_MSIZE             DMAC_CCR1_MSIZE_Msk
#define DMAC_CCR1_MSIZE_0           (0x1UL << DMAC_CCR1_MSIZE_Pos)                    /*!< 0x00000400 */
#define DMAC_CCR1_MSIZE_1           (0x2UL << DMAC_CCR1_MSIZE_Pos)                    /*!< 0x00000800 */

#define DMAC_CCR1_PL_Pos            (12U)
#define DMAC_CCR1_PL_Msk            (0x3UL << DMAC_CCR1_PL_Pos)
#define DMAC_CCR1_PL                DMAC_CCR1_PL_Msk
#define DMAC_CCR1_PL_0              (0x1UL << DMAC_CCR1_PL_Pos)                       /*!< 0x00001000 */
#define DMAC_CCR1_PL_1              (0x2UL << DMAC_CCR1_PL_Pos)                       /*!< 0x00002000 */

#define DMAC_CCR1_MEM2MEM_Pos       (14U)
#define DMAC_CCR1_MEM2MEM_Msk       (0x1UL << DMAC_CCR1_MEM2MEM_Pos)
#define DMAC_CCR1_MEM2MEM           DMAC_CCR1_MEM2MEM_Msk

/**************** Bit definition for DMAC_CNDTR1 register *****************/
#define DMAC_CNDTR1_NDT_Pos         (0U)
#define DMAC_CNDTR1_NDT_Msk         (0xFFFFUL << DMAC_CNDTR1_NDT_Pos)
#define DMAC_CNDTR1_NDT             DMAC_CNDTR1_NDT_Msk

/***************** Bit definition for DMAC_CPAR1 register *****************/
#define DMAC_CPAR1_PA_Pos           (0U)
#define DMAC_CPAR1_PA_Msk           (0xFFFFFFFFUL << DMAC_CPAR1_PA_Pos)
#define DMAC_CPAR1_PA               DMAC_CPAR1_PA_Msk

/**************** Bit definition for DMAC_CM0AR1 register *****************/
#define DMAC_CM0AR1_MA_Pos          (0U)
#define DMAC_CM0AR1_MA_Msk          (0xFFFFFFFFUL << DMAC_CM0AR1_MA_Pos)
#define DMAC_CM0AR1_MA              DMAC_CM0AR1_MA_Msk

/******************* Bit definition for DMAC_CBSR1 register *******************/
#define DMAC_CBSR1_BS_Pos               (0U)
#define DMAC_CBSR1_BS_Msk               (0xFFUL << DMAC_CBSR1_BS_Pos)
#define DMAC_CBSR1_BS                   DMAC_CBSR1_BS_Msk

/***************** Bit definition for DMAC_CCR2 register ******************/
#define DMAC_CCR2_EN_Pos            (0U)
#define DMAC_CCR2_EN_Msk            (0x1UL << DMAC_CCR2_EN_Pos)
#define DMAC_CCR2_EN                DMAC_CCR2_EN_Msk
#define DMAC_CCR2_TCIE_Pos          (1U)
#define DMAC_CCR2_TCIE_Msk          (0x1UL << DMAC_CCR2_TCIE_Pos)
#define DMAC_CCR2_TCIE              DMAC_CCR2_TCIE_Msk
#define DMAC_CCR2_HTIE_Pos          (2U)
#define DMAC_CCR2_HTIE_Msk          (0x1UL << DMAC_CCR2_HTIE_Pos)
#define DMAC_CCR2_HTIE              DMAC_CCR2_HTIE_Msk
#define DMAC_CCR2_TEIE_Pos          (3U)
#define DMAC_CCR2_TEIE_Msk          (0x1UL << DMAC_CCR2_TEIE_Pos)
#define DMAC_CCR2_TEIE              DMAC_CCR2_TEIE_Msk
#define DMAC_CCR2_DIR_Pos           (4U)
#define DMAC_CCR2_DIR_Msk           (0x1UL << DMAC_CCR2_DIR_Pos)
#define DMAC_CCR2_DIR               DMAC_CCR2_DIR_Msk
#define DMAC_CCR2_CIRC_Pos          (5U)
#define DMAC_CCR2_CIRC_Msk          (0x1UL << DMAC_CCR2_CIRC_Pos)
#define DMAC_CCR2_CIRC              DMAC_CCR2_CIRC_Msk
#define DMAC_CCR2_PINC_Pos          (6U)
#define DMAC_CCR2_PINC_Msk          (0x1UL << DMAC_CCR2_PINC_Pos)
#define DMAC_CCR2_PINC              DMAC_CCR2_PINC_Msk
#define DMAC_CCR2_MINC_Pos          (7U)
#define DMAC_CCR2_MINC_Msk          (0x1UL << DMAC_CCR2_MINC_Pos)
#define DMAC_CCR2_MINC              DMAC_CCR2_MINC_Msk
#define DMAC_CCR2_PSIZE_Pos         (8U)
#define DMAC_CCR2_PSIZE_Msk         (0x3UL << DMAC_CCR2_PSIZE_Pos)
#define DMAC_CCR2_PSIZE             DMAC_CCR2_PSIZE_Msk
#define DMAC_CCR2_MSIZE_Pos         (10U)
#define DMAC_CCR2_MSIZE_Msk         (0x3UL << DMAC_CCR2_MSIZE_Pos)
#define DMAC_CCR2_MSIZE             DMAC_CCR2_MSIZE_Msk
#define DMAC_CCR2_PL_Pos            (12U)
#define DMAC_CCR2_PL_Msk            (0x3UL << DMAC_CCR2_PL_Pos)
#define DMAC_CCR2_PL                DMAC_CCR2_PL_Msk
#define DMAC_CCR2_MEM2MEM_Pos       (14U)
#define DMAC_CCR2_MEM2MEM_Msk       (0x1UL << DMAC_CCR2_MEM2MEM_Pos)
#define DMAC_CCR2_MEM2MEM           DMAC_CCR2_MEM2MEM_Msk

/**************** Bit definition for DMAC_CNDTR2 register *****************/
#define DMAC_CNDTR2_NDT_Pos         (0U)
#define DMAC_CNDTR2_NDT_Msk         (0xFFFFUL << DMAC_CNDTR2_NDT_Pos)
#define DMAC_CNDTR2_NDT             DMAC_CNDTR2_NDT_Msk

/***************** Bit definition for DMAC_CPAR2 register *****************/
#define DMAC_CPAR2_PA_Pos           (0U)
#define DMAC_CPAR2_PA_Msk           (0xFFFFFFFFUL << DMAC_CPAR2_PA_Pos)
#define DMAC_CPAR2_PA               DMAC_CPAR2_PA_Msk

/**************** Bit definition for DMAC_CM0AR2 register *****************/
#define DMAC_CM0AR2_MA_Pos          (0U)
#define DMAC_CM0AR2_MA_Msk          (0xFFFFFFFFUL << DMAC_CM0AR2_MA_Pos)
#define DMAC_CM0AR2_MA              DMAC_CM0AR2_MA_Msk

/******************* Bit definition for DMAC_CBSR2 register *******************/
#define DMAC_CBSR2_BS_Pos               (0U)
#define DMAC_CBSR2_BS_Msk               (0xFFUL << DMAC_CBSR2_BS_Pos)
#define DMAC_CBSR2_BS                   DMAC_CBSR2_BS_Msk

/***************** Bit definition for DMAC_CCR3 register ******************/
#define DMAC_CCR3_EN_Pos            (0U)
#define DMAC_CCR3_EN_Msk            (0x1UL << DMAC_CCR3_EN_Pos)
#define DMAC_CCR3_EN                DMAC_CCR3_EN_Msk
#define DMAC_CCR3_TCIE_Pos          (1U)
#define DMAC_CCR3_TCIE_Msk          (0x1UL << DMAC_CCR3_TCIE_Pos)
#define DMAC_CCR3_TCIE              DMAC_CCR3_TCIE_Msk
#define DMAC_CCR3_HTIE_Pos          (2U)
#define DMAC_CCR3_HTIE_Msk          (0x1UL << DMAC_CCR3_HTIE_Pos)
#define DMAC_CCR3_HTIE              DMAC_CCR3_HTIE_Msk
#define DMAC_CCR3_TEIE_Pos          (3U)
#define DMAC_CCR3_TEIE_Msk          (0x1UL << DMAC_CCR3_TEIE_Pos)
#define DMAC_CCR3_TEIE              DMAC_CCR3_TEIE_Msk
#define DMAC_CCR3_DIR_Pos           (4U)
#define DMAC_CCR3_DIR_Msk           (0x1UL << DMAC_CCR3_DIR_Pos)
#define DMAC_CCR3_DIR               DMAC_CCR3_DIR_Msk
#define DMAC_CCR3_CIRC_Pos          (5U)
#define DMAC_CCR3_CIRC_Msk          (0x1UL << DMAC_CCR3_CIRC_Pos)
#define DMAC_CCR3_CIRC              DMAC_CCR3_CIRC_Msk
#define DMAC_CCR3_PINC_Pos          (6U)
#define DMAC_CCR3_PINC_Msk          (0x1UL << DMAC_CCR3_PINC_Pos)
#define DMAC_CCR3_PINC              DMAC_CCR3_PINC_Msk
#define DMAC_CCR3_MINC_Pos          (7U)
#define DMAC_CCR3_MINC_Msk          (0x1UL << DMAC_CCR3_MINC_Pos)
#define DMAC_CCR3_MINC              DMAC_CCR3_MINC_Msk
#define DMAC_CCR3_PSIZE_Pos         (8U)
#define DMAC_CCR3_PSIZE_Msk         (0x3UL << DMAC_CCR3_PSIZE_Pos)
#define DMAC_CCR3_PSIZE             DMAC_CCR3_PSIZE_Msk
#define DMAC_CCR3_MSIZE_Pos         (10U)
#define DMAC_CCR3_MSIZE_Msk         (0x3UL << DMAC_CCR3_MSIZE_Pos)
#define DMAC_CCR3_MSIZE             DMAC_CCR3_MSIZE_Msk
#define DMAC_CCR3_PL_Pos            (12U)
#define DMAC_CCR3_PL_Msk            (0x3UL << DMAC_CCR3_PL_Pos)
#define DMAC_CCR3_PL                DMAC_CCR3_PL_Msk
#define DMAC_CCR3_MEM2MEM_Pos       (14U)
#define DMAC_CCR3_MEM2MEM_Msk       (0x1UL << DMAC_CCR3_MEM2MEM_Pos)
#define DMAC_CCR3_MEM2MEM           DMAC_CCR3_MEM2MEM_Msk

/**************** Bit definition for DMAC_CNDTR3 register *****************/
#define DMAC_CNDTR3_NDT_Pos         (0U)
#define DMAC_CNDTR3_NDT_Msk         (0xFFFFUL << DMAC_CNDTR3_NDT_Pos)
#define DMAC_CNDTR3_NDT             DMAC_CNDTR3_NDT_Msk

/***************** Bit definition for DMAC_CPAR3 register *****************/
#define DMAC_CPAR3_PA_Pos           (0U)
#define DMAC_CPAR3_PA_Msk           (0xFFFFFFFFUL << DMAC_CPAR3_PA_Pos)
#define DMAC_CPAR3_PA               DMAC_CPAR3_PA_Msk

/**************** Bit definition for DMAC_CM0AR3 register *****************/
#define DMAC_CM0AR3_MA_Pos          (0U)
#define DMAC_CM0AR3_MA_Msk          (0xFFFFFFFFUL << DMAC_CM0AR3_MA_Pos)
#define DMAC_CM0AR3_MA              DMAC_CM0AR3_MA_Msk

/******************* Bit definition for DMAC_CBSR3 register *******************/
#define DMAC_CBSR3_BS_Pos               (0U)
#define DMAC_CBSR3_BS_Msk               (0xFFUL << DMAC_CBSR3_BS_Pos)
#define DMAC_CBSR3_BS                   DMAC_CBSR3_BS_Msk

/***************** Bit definition for DMAC_CCR4 register ******************/
#define DMAC_CCR4_EN_Pos            (0U)
#define DMAC_CCR4_EN_Msk            (0x1UL << DMAC_CCR4_EN_Pos)
#define DMAC_CCR4_EN                DMAC_CCR4_EN_Msk
#define DMAC_CCR4_TCIE_Pos          (1U)
#define DMAC_CCR4_TCIE_Msk          (0x1UL << DMAC_CCR4_TCIE_Pos)
#define DMAC_CCR4_TCIE              DMAC_CCR4_TCIE_Msk
#define DMAC_CCR4_HTIE_Pos          (2U)
#define DMAC_CCR4_HTIE_Msk          (0x1UL << DMAC_CCR4_HTIE_Pos)
#define DMAC_CCR4_HTIE              DMAC_CCR4_HTIE_Msk
#define DMAC_CCR4_TEIE_Pos          (3U)
#define DMAC_CCR4_TEIE_Msk          (0x1UL << DMAC_CCR4_TEIE_Pos)
#define DMAC_CCR4_TEIE              DMAC_CCR4_TEIE_Msk
#define DMAC_CCR4_DIR_Pos           (4U)
#define DMAC_CCR4_DIR_Msk           (0x1UL << DMAC_CCR4_DIR_Pos)
#define DMAC_CCR4_DIR               DMAC_CCR4_DIR_Msk
#define DMAC_CCR4_CIRC_Pos          (5U)
#define DMAC_CCR4_CIRC_Msk          (0x1UL << DMAC_CCR4_CIRC_Pos)
#define DMAC_CCR4_CIRC              DMAC_CCR4_CIRC_Msk
#define DMAC_CCR4_PINC_Pos          (6U)
#define DMAC_CCR4_PINC_Msk          (0x1UL << DMAC_CCR4_PINC_Pos)
#define DMAC_CCR4_PINC              DMAC_CCR4_PINC_Msk
#define DMAC_CCR4_MINC_Pos          (7U)
#define DMAC_CCR4_MINC_Msk          (0x1UL << DMAC_CCR4_MINC_Pos)
#define DMAC_CCR4_MINC              DMAC_CCR4_MINC_Msk
#define DMAC_CCR4_PSIZE_Pos         (8U)
#define DMAC_CCR4_PSIZE_Msk         (0x3UL << DMAC_CCR4_PSIZE_Pos)
#define DMAC_CCR4_PSIZE             DMAC_CCR4_PSIZE_Msk
#define DMAC_CCR4_MSIZE_Pos         (10U)
#define DMAC_CCR4_MSIZE_Msk         (0x3UL << DMAC_CCR4_MSIZE_Pos)
#define DMAC_CCR4_MSIZE             DMAC_CCR4_MSIZE_Msk
#define DMAC_CCR4_PL_Pos            (12U)
#define DMAC_CCR4_PL_Msk            (0x3UL << DMAC_CCR4_PL_Pos)
#define DMAC_CCR4_PL                DMAC_CCR4_PL_Msk
#define DMAC_CCR4_MEM2MEM_Pos       (14U)
#define DMAC_CCR4_MEM2MEM_Msk       (0x1UL << DMAC_CCR4_MEM2MEM_Pos)
#define DMAC_CCR4_MEM2MEM           DMAC_CCR4_MEM2MEM_Msk

/**************** Bit definition for DMAC_CNDTR4 register *****************/
#define DMAC_CNDTR4_NDT_Pos         (0U)
#define DMAC_CNDTR4_NDT_Msk         (0xFFFFUL << DMAC_CNDTR4_NDT_Pos)
#define DMAC_CNDTR4_NDT             DMAC_CNDTR4_NDT_Msk

/***************** Bit definition for DMAC_CPAR4 register *****************/
#define DMAC_CPAR4_PA_Pos           (0U)
#define DMAC_CPAR4_PA_Msk           (0xFFFFFFFFUL << DMAC_CPAR4_PA_Pos)
#define DMAC_CPAR4_PA               DMAC_CPAR4_PA_Msk

/**************** Bit definition for DMAC_CM0AR4 register *****************/
#define DMAC_CM0AR4_MA_Pos          (0U)
#define DMAC_CM0AR4_MA_Msk          (0xFFFFFFFFUL << DMAC_CM0AR4_MA_Pos)
#define DMAC_CM0AR4_MA              DMAC_CM0AR4_MA_Msk


/******************* Bit definition for DMAC_CBSR4 register *******************/
#define DMAC_CBSR4_BS_Pos               (0U)
#define DMAC_CBSR4_BS_Msk               (0xFFUL << DMAC_CBSR4_BS_Pos)
#define DMAC_CBSR4_BS                   DMAC_CBSR4_BS_Msk


/***************** Bit definition for DMAC_CCR5 register ******************/
#define DMAC_CCR5_EN_Pos            (0U)
#define DMAC_CCR5_EN_Msk            (0x1UL << DMAC_CCR5_EN_Pos)
#define DMAC_CCR5_EN                DMAC_CCR5_EN_Msk
#define DMAC_CCR5_TCIE_Pos          (1U)
#define DMAC_CCR5_TCIE_Msk          (0x1UL << DMAC_CCR5_TCIE_Pos)
#define DMAC_CCR5_TCIE              DMAC_CCR5_TCIE_Msk
#define DMAC_CCR5_HTIE_Pos          (2U)
#define DMAC_CCR5_HTIE_Msk          (0x1UL << DMAC_CCR5_HTIE_Pos)
#define DMAC_CCR5_HTIE              DMAC_CCR5_HTIE_Msk
#define DMAC_CCR5_TEIE_Pos          (3U)
#define DMAC_CCR5_TEIE_Msk          (0x1UL << DMAC_CCR5_TEIE_Pos)
#define DMAC_CCR5_TEIE              DMAC_CCR5_TEIE_Msk
#define DMAC_CCR5_DIR_Pos           (4U)
#define DMAC_CCR5_DIR_Msk           (0x1UL << DMAC_CCR5_DIR_Pos)
#define DMAC_CCR5_DIR               DMAC_CCR5_DIR_Msk
#define DMAC_CCR5_CIRC_Pos          (5U)
#define DMAC_CCR5_CIRC_Msk          (0x1UL << DMAC_CCR5_CIRC_Pos)
#define DMAC_CCR5_CIRC              DMAC_CCR5_CIRC_Msk
#define DMAC_CCR5_PINC_Pos          (6U)
#define DMAC_CCR5_PINC_Msk          (0x1UL << DMAC_CCR5_PINC_Pos)
#define DMAC_CCR5_PINC              DMAC_CCR5_PINC_Msk
#define DMAC_CCR5_MINC_Pos          (7U)
#define DMAC_CCR5_MINC_Msk          (0x1UL << DMAC_CCR5_MINC_Pos)
#define DMAC_CCR5_MINC              DMAC_CCR5_MINC_Msk
#define DMAC_CCR5_PSIZE_Pos         (8U)
#define DMAC_CCR5_PSIZE_Msk         (0x3UL << DMAC_CCR5_PSIZE_Pos)
#define DMAC_CCR5_PSIZE             DMAC_CCR5_PSIZE_Msk
#define DMAC_CCR5_MSIZE_Pos         (10U)
#define DMAC_CCR5_MSIZE_Msk         (0x3UL << DMAC_CCR5_MSIZE_Pos)
#define DMAC_CCR5_MSIZE             DMAC_CCR5_MSIZE_Msk
#define DMAC_CCR5_PL_Pos            (12U)
#define DMAC_CCR5_PL_Msk            (0x3UL << DMAC_CCR5_PL_Pos)
#define DMAC_CCR5_PL                DMAC_CCR5_PL_Msk
#define DMAC_CCR5_MEM2MEM_Pos       (14U)
#define DMAC_CCR5_MEM2MEM_Msk       (0x1UL << DMAC_CCR5_MEM2MEM_Pos)
#define DMAC_CCR5_MEM2MEM           DMAC_CCR5_MEM2MEM_Msk

/**************** Bit definition for DMAC_CNDTR5 register *****************/
#define DMAC_CNDTR5_NDT_Pos         (0U)
#define DMAC_CNDTR5_NDT_Msk         (0xFFFFUL << DMAC_CNDTR5_NDT_Pos)
#define DMAC_CNDTR5_NDT             DMAC_CNDTR5_NDT_Msk

/***************** Bit definition for DMAC_CPAR5 register *****************/
#define DMAC_CPAR5_PA_Pos           (0U)
#define DMAC_CPAR5_PA_Msk           (0xFFFFFFFFUL << DMAC_CPAR5_PA_Pos)
#define DMAC_CPAR5_PA               DMAC_CPAR5_PA_Msk

/**************** Bit definition for DMAC_CM0AR5 register *****************/
#define DMAC_CM0AR5_MA_Pos          (0U)
#define DMAC_CM0AR5_MA_Msk          (0xFFFFFFFFUL << DMAC_CM0AR5_MA_Pos)
#define DMAC_CM0AR5_MA              DMAC_CM0AR5_MA_Msk

/******************* Bit definition for DMAC_CBSR5 register *******************/
#define DMAC_CBSR5_BS_Pos               (0U)
#define DMAC_CBSR5_BS_Msk               (0xFFUL << DMAC_CBSR5_BS_Pos)
#define DMAC_CBSR5_BS                   DMAC_CBSR5_BS_Msk

/***************** Bit definition for DMAC_CCR6 register ******************/
#define DMAC_CCR6_EN_Pos            (0U)
#define DMAC_CCR6_EN_Msk            (0x1UL << DMAC_CCR6_EN_Pos)
#define DMAC_CCR6_EN                DMAC_CCR6_EN_Msk
#define DMAC_CCR6_TCIE_Pos          (1U)
#define DMAC_CCR6_TCIE_Msk          (0x1UL << DMAC_CCR6_TCIE_Pos)
#define DMAC_CCR6_TCIE              DMAC_CCR6_TCIE_Msk
#define DMAC_CCR6_HTIE_Pos          (2U)
#define DMAC_CCR6_HTIE_Msk          (0x1UL << DMAC_CCR6_HTIE_Pos)
#define DMAC_CCR6_HTIE              DMAC_CCR6_HTIE_Msk
#define DMAC_CCR6_TEIE_Pos          (3U)
#define DMAC_CCR6_TEIE_Msk          (0x1UL << DMAC_CCR6_TEIE_Pos)
#define DMAC_CCR6_TEIE              DMAC_CCR6_TEIE_Msk
#define DMAC_CCR6_DIR_Pos           (4U)
#define DMAC_CCR6_DIR_Msk           (0x1UL << DMAC_CCR6_DIR_Pos)
#define DMAC_CCR6_DIR               DMAC_CCR6_DIR_Msk
#define DMAC_CCR6_CIRC_Pos          (5U)
#define DMAC_CCR6_CIRC_Msk          (0x1UL << DMAC_CCR6_CIRC_Pos)
#define DMAC_CCR6_CIRC              DMAC_CCR6_CIRC_Msk
#define DMAC_CCR6_PINC_Pos          (6U)
#define DMAC_CCR6_PINC_Msk          (0x1UL << DMAC_CCR6_PINC_Pos)
#define DMAC_CCR6_PINC              DMAC_CCR6_PINC_Msk
#define DMAC_CCR6_MINC_Pos          (7U)
#define DMAC_CCR6_MINC_Msk          (0x1UL << DMAC_CCR6_MINC_Pos)
#define DMAC_CCR6_MINC              DMAC_CCR6_MINC_Msk
#define DMAC_CCR6_PSIZE_Pos         (8U)
#define DMAC_CCR6_PSIZE_Msk         (0x3UL << DMAC_CCR6_PSIZE_Pos)
#define DMAC_CCR6_PSIZE             DMAC_CCR6_PSIZE_Msk
#define DMAC_CCR6_MSIZE_Pos         (10U)
#define DMAC_CCR6_MSIZE_Msk         (0x3UL << DMAC_CCR6_MSIZE_Pos)
#define DMAC_CCR6_MSIZE             DMAC_CCR6_MSIZE_Msk
#define DMAC_CCR6_PL_Pos            (12U)
#define DMAC_CCR6_PL_Msk            (0x3UL << DMAC_CCR6_PL_Pos)
#define DMAC_CCR6_PL                DMAC_CCR6_PL_Msk
#define DMAC_CCR6_MEM2MEM_Pos       (14U)
#define DMAC_CCR6_MEM2MEM_Msk       (0x1UL << DMAC_CCR6_MEM2MEM_Pos)
#define DMAC_CCR6_MEM2MEM           DMAC_CCR6_MEM2MEM_Msk

/**************** Bit definition for DMAC_CNDTR6 register *****************/
#define DMAC_CNDTR6_NDT_Pos         (0U)
#define DMAC_CNDTR6_NDT_Msk         (0xFFFFUL << DMAC_CNDTR6_NDT_Pos)
#define DMAC_CNDTR6_NDT             DMAC_CNDTR6_NDT_Msk

/***************** Bit definition for DMAC_CPAR6 register *****************/
#define DMAC_CPAR6_PA_Pos           (0U)
#define DMAC_CPAR6_PA_Msk           (0xFFFFFFFFUL << DMAC_CPAR6_PA_Pos)
#define DMAC_CPAR6_PA               DMAC_CPAR6_PA_Msk

/**************** Bit definition for DMAC_CM0AR6 register *****************/
#define DMAC_CM0AR6_MA_Pos          (0U)
#define DMAC_CM0AR6_MA_Msk          (0xFFFFFFFFUL << DMAC_CM0AR6_MA_Pos)
#define DMAC_CM0AR6_MA              DMAC_CM0AR6_MA_Msk

/******************* Bit definition for DMAC_CBSR6 register *******************/
#define DMAC_CBSR6_BS_Pos               (0U)
#define DMAC_CBSR6_BS_Msk               (0xFFUL << DMAC_CBSR6_BS_Pos)
#define DMAC_CBSR6_BS                   DMAC_CBSR6_BS_Msk

/***************** Bit definition for DMAC_CCR7 register ******************/
#define DMAC_CCR7_EN_Pos            (0U)
#define DMAC_CCR7_EN_Msk            (0x1UL << DMAC_CCR7_EN_Pos)
#define DMAC_CCR7_EN                DMAC_CCR7_EN_Msk
#define DMAC_CCR7_TCIE_Pos          (1U)
#define DMAC_CCR7_TCIE_Msk          (0x1UL << DMAC_CCR7_TCIE_Pos)
#define DMAC_CCR7_TCIE              DMAC_CCR7_TCIE_Msk
#define DMAC_CCR7_HTIE_Pos          (2U)
#define DMAC_CCR7_HTIE_Msk          (0x1UL << DMAC_CCR7_HTIE_Pos)
#define DMAC_CCR7_HTIE              DMAC_CCR7_HTIE_Msk
#define DMAC_CCR7_TEIE_Pos          (3U)
#define DMAC_CCR7_TEIE_Msk          (0x1UL << DMAC_CCR7_TEIE_Pos)
#define DMAC_CCR7_TEIE              DMAC_CCR7_TEIE_Msk
#define DMAC_CCR7_DIR_Pos           (4U)
#define DMAC_CCR7_DIR_Msk           (0x1UL << DMAC_CCR7_DIR_Pos)
#define DMAC_CCR7_DIR               DMAC_CCR7_DIR_Msk
#define DMAC_CCR7_CIRC_Pos          (5U)
#define DMAC_CCR7_CIRC_Msk          (0x1UL << DMAC_CCR7_CIRC_Pos)
#define DMAC_CCR7_CIRC              DMAC_CCR7_CIRC_Msk
#define DMAC_CCR7_PINC_Pos          (6U)
#define DMAC_CCR7_PINC_Msk          (0x1UL << DMAC_CCR7_PINC_Pos)
#define DMAC_CCR7_PINC              DMAC_CCR7_PINC_Msk
#define DMAC_CCR7_MINC_Pos          (7U)
#define DMAC_CCR7_MINC_Msk          (0x1UL << DMAC_CCR7_MINC_Pos)
#define DMAC_CCR7_MINC              DMAC_CCR7_MINC_Msk
#define DMAC_CCR7_PSIZE_Pos         (8U)
#define DMAC_CCR7_PSIZE_Msk         (0x3UL << DMAC_CCR7_PSIZE_Pos)
#define DMAC_CCR7_PSIZE             DMAC_CCR7_PSIZE_Msk
#define DMAC_CCR7_MSIZE_Pos         (10U)
#define DMAC_CCR7_MSIZE_Msk         (0x3UL << DMAC_CCR7_MSIZE_Pos)
#define DMAC_CCR7_MSIZE             DMAC_CCR7_MSIZE_Msk
#define DMAC_CCR7_PL_Pos            (12U)
#define DMAC_CCR7_PL_Msk            (0x3UL << DMAC_CCR7_PL_Pos)
#define DMAC_CCR7_PL                DMAC_CCR7_PL_Msk
#define DMAC_CCR7_MEM2MEM_Pos       (14U)
#define DMAC_CCR7_MEM2MEM_Msk       (0x1UL << DMAC_CCR7_MEM2MEM_Pos)
#define DMAC_CCR7_MEM2MEM           DMAC_CCR7_MEM2MEM_Msk

/**************** Bit definition for DMAC_CNDTR7 register *****************/
#define DMAC_CNDTR7_NDT_Pos         (0U)
#define DMAC_CNDTR7_NDT_Msk         (0xFFFFUL << DMAC_CNDTR7_NDT_Pos)
#define DMAC_CNDTR7_NDT             DMAC_CNDTR7_NDT_Msk

/***************** Bit definition for DMAC_CPAR7 register *****************/
#define DMAC_CPAR7_PA_Pos           (0U)
#define DMAC_CPAR7_PA_Msk           (0xFFFFFFFFUL << DMAC_CPAR7_PA_Pos)
#define DMAC_CPAR7_PA               DMAC_CPAR7_PA_Msk

/**************** Bit definition for DMAC_CM0AR7 register *****************/
#define DMAC_CM0AR7_MA_Pos          (0U)
#define DMAC_CM0AR7_MA_Msk          (0xFFFFFFFFUL << DMAC_CM0AR7_MA_Pos)
#define DMAC_CM0AR7_MA              DMAC_CM0AR7_MA_Msk

/******************* Bit definition for DMAC_CBSR7 register *******************/
#define DMAC_CBSR7_BS_Pos               (0U)
#define DMAC_CBSR7_BS_Msk               (0xFFUL << DMAC_CBSR7_BS_Pos)
#define DMAC_CBSR7_BS                   DMAC_CBSR7_BS_Msk

/***************** Bit definition for DMAC_CCR8 register ******************/
#define DMAC_CCR8_EN_Pos            (0U)
#define DMAC_CCR8_EN_Msk            (0x1UL << DMAC_CCR8_EN_Pos)
#define DMAC_CCR8_EN                DMAC_CCR8_EN_Msk
#define DMAC_CCR8_TCIE_Pos          (1U)
#define DMAC_CCR8_TCIE_Msk          (0x1UL << DMAC_CCR8_TCIE_Pos)
#define DMAC_CCR8_TCIE              DMAC_CCR8_TCIE_Msk
#define DMAC_CCR8_HTIE_Pos          (2U)
#define DMAC_CCR8_HTIE_Msk          (0x1UL << DMAC_CCR8_HTIE_Pos)
#define DMAC_CCR8_HTIE              DMAC_CCR8_HTIE_Msk
#define DMAC_CCR8_TEIE_Pos          (3U)
#define DMAC_CCR8_TEIE_Msk          (0x1UL << DMAC_CCR8_TEIE_Pos)
#define DMAC_CCR8_TEIE              DMAC_CCR8_TEIE_Msk
#define DMAC_CCR8_DIR_Pos           (4U)
#define DMAC_CCR8_DIR_Msk           (0x1UL << DMAC_CCR8_DIR_Pos)
#define DMAC_CCR8_DIR               DMAC_CCR8_DIR_Msk
#define DMAC_CCR8_CIRC_Pos          (5U)
#define DMAC_CCR8_CIRC_Msk          (0x1UL << DMAC_CCR8_CIRC_Pos)
#define DMAC_CCR8_CIRC              DMAC_CCR8_CIRC_Msk
#define DMAC_CCR8_PINC_Pos          (6U)
#define DMAC_CCR8_PINC_Msk          (0x1UL << DMAC_CCR8_PINC_Pos)
#define DMAC_CCR8_PINC              DMAC_CCR8_PINC_Msk
#define DMAC_CCR8_MINC_Pos          (7U)
#define DMAC_CCR8_MINC_Msk          (0x1UL << DMAC_CCR8_MINC_Pos)
#define DMAC_CCR8_MINC              DMAC_CCR8_MINC_Msk
#define DMAC_CCR8_PSIZE_Pos         (8U)
#define DMAC_CCR8_PSIZE_Msk         (0x3UL << DMAC_CCR8_PSIZE_Pos)
#define DMAC_CCR8_PSIZE             DMAC_CCR8_PSIZE_Msk
#define DMAC_CCR8_MSIZE_Pos         (10U)
#define DMAC_CCR8_MSIZE_Msk         (0x3UL << DMAC_CCR8_MSIZE_Pos)
#define DMAC_CCR8_MSIZE             DMAC_CCR8_MSIZE_Msk
#define DMAC_CCR8_PL_Pos            (12U)
#define DMAC_CCR8_PL_Msk            (0x3UL << DMAC_CCR8_PL_Pos)
#define DMAC_CCR8_PL                DMAC_CCR8_PL_Msk
#define DMAC_CCR8_MEM2MEM_Pos       (14U)
#define DMAC_CCR8_MEM2MEM_Msk       (0x1UL << DMAC_CCR8_MEM2MEM_Pos)
#define DMAC_CCR8_MEM2MEM           DMAC_CCR8_MEM2MEM_Msk

/****************** Bit definition for DMAC_CNDTR8 register *******************/
#define DMAC_CNDTR8_NDT_Pos         (0U)
#define DMAC_CNDTR8_NDT_Msk         (0xFFFFUL << DMAC_CNDTR8_NDT_Pos)
#define DMAC_CNDTR8_NDT             DMAC_CNDTR8_NDT_Msk

/***************** Bit definition for DMAC_CPAR8 register *****************/
#define DMAC_CPAR8_PA_Pos           (0U)
#define DMAC_CPAR8_PA_Msk           (0xFFFFFFFFUL << DMAC_CPAR8_PA_Pos)
#define DMAC_CPAR8_PA               DMAC_CPAR8_PA_Msk

/**************** Bit definition for DMAC_CM0AR8 register *****************/
#define DMAC_CM0AR8_MA_Pos          (0U)
#define DMAC_CM0AR8_MA_Msk          (0xFFFFFFFFUL << DMAC_CM0AR8_MA_Pos)
#define DMAC_CM0AR8_MA              DMAC_CM0AR8_MA_Msk

/******************* Bit definition for DMAC_CBSR8 register *******************/
#define DMAC_CBSR8_BS_Pos               (0U)
#define DMAC_CBSR8_BS_Msk               (0xFFUL << DMAC_CBSR8_BS_Pos)
#define DMAC_CBSR8_BS                   DMAC_CBSR8_BS_Msk

/****************** Bit definition for DMAC_CSELR1 register *******************/
#define DMAC_CSELR1_C1S_Pos             (0U)
#define DMAC_CSELR1_C1S_Msk             (0x3FUL << DMAC_CSELR1_C1S_Pos)
#define DMAC_CSELR1_C1S                 DMAC_CSELR1_C1S_Msk
#define DMAC_CSELR1_C2S_Pos             (8U)
#define DMAC_CSELR1_C2S_Msk             (0x3FUL << DMAC_CSELR1_C2S_Pos)
#define DMAC_CSELR1_C2S                 DMAC_CSELR1_C2S_Msk
#define DMAC_CSELR1_C3S_Pos             (16U)
#define DMAC_CSELR1_C3S_Msk             (0x3FUL << DMAC_CSELR1_C3S_Pos)
#define DMAC_CSELR1_C3S                 DMAC_CSELR1_C3S_Msk
#define DMAC_CSELR1_C4S_Pos             (24U)
#define DMAC_CSELR1_C4S_Msk             (0x3FUL << DMAC_CSELR1_C4S_Pos)
#define DMAC_CSELR1_C4S                 DMAC_CSELR1_C4S_Msk

/****************** Bit definition for DMAC_CSELR2 register *******************/
#define DMAC_CSELR2_C5S_Pos             (0U)
#define DMAC_CSELR2_C5S_Msk             (0x3FUL << DMAC_CSELR2_C5S_Pos)
#define DMAC_CSELR2_C5S                 DMAC_CSELR2_C5S_Msk
#define DMAC_CSELR2_C6S_Pos             (8U)
#define DMAC_CSELR2_C6S_Msk             (0x3FUL << DMAC_CSELR2_C6S_Pos)
#define DMAC_CSELR2_C6S                 DMAC_CSELR2_C6S_Msk
#define DMAC_CSELR2_C7S_Pos             (16U)
#define DMAC_CSELR2_C7S_Msk             (0x3FUL << DMAC_CSELR2_C7S_Pos)
#define DMAC_CSELR2_C7S                 DMAC_CSELR2_C7S_Msk
#define DMAC_CSELR2_C8S_Pos             (24U)
#define DMAC_CSELR2_C8S_Msk             (0x3FUL << DMAC_CSELR2_C8S_Pos)
#define DMAC_CSELR2_C8S                 DMAC_CSELR2_C8S_Msk

#define DMA_CSELR_C1S                   DMAC_CSELR1_C1S

/****************** Bit definition for DMAC_DBGSEL register *******************/
#define DMAC_DBGSEL_DBGSEL_Pos          (0U)
#define DMAC_DBGSEL_DBGSEL_Msk          (0xFUL << DMAC_DBGSEL_DBGSEL_Pos)
#define DMAC_DBGSEL_DBGSEL              DMAC_DBGSEL_DBGSEL_Msk

#define DMA_CCR_CIRC                    DMAC_CCR1_CIRC
#define DMA_CCR_PL                      DMAC_CCR1_PL
#define DMA_CCR_MSIZE                   DMAC_CCR1_MSIZE
#define DMA_CCR_PSIZE                   DMAC_CCR1_PSIZE
#define DMA_CCR_MINC                    DMAC_CCR1_MINC
#define DMA_CCR_PINC                    DMAC_CCR1_PINC
#define DMA_CCR_CIRC                    DMAC_CCR1_CIRC
#define DMA_CCR_DIR                     DMAC_CCR1_DIR
#define DMA_CCR_MEM2MEM                 DMAC_CCR1_MEM2MEM



#endif
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
