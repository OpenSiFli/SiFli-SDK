/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __PTM_H
#define __PTM_H

typedef struct
{
    __IO uint32_t ISR;
    __IO uint32_t IER;
    __IO uint32_t CER;
    __IO uint32_t RSVD9;
    __IO uint32_t DOER;
    __IO uint32_t DOR;
    __IO uint32_t DOESR;
    __IO uint32_t DOECR;
    __IO uint32_t DOSR;
    __IO uint32_t DOCR;
    __IO uint32_t RSVD8[2];
    __IO uint32_t CCR0;
    __IO uint32_t ICR0;
    __IO uint32_t OCR0;
    __IO uint32_t X0;
    __IO uint32_t Y0;
    __IO uint32_t A0;
    __IO uint32_t B0;
    __IO uint32_t PC0;
    __IO uint32_t IFIFO0;
    __IO uint32_t OFIFO0;
    __IO uint32_t RSVD7[2];
    __IO uint32_t CCR1;
    __IO uint32_t ICR1;
    __IO uint32_t OCR1;
    __IO uint32_t X1;
    __IO uint32_t Y1;
    __IO uint32_t A1;
    __IO uint32_t B1;
    __IO uint32_t PC1;
    __IO uint32_t IFIFO1;
    __IO uint32_t OFIFO1;
    __IO uint32_t RSVD6[2];
    __IO uint32_t CCR2;
    __IO uint32_t ICR2;
    __IO uint32_t OCR2;
    __IO uint32_t X2;
    __IO uint32_t Y2;
    __IO uint32_t A2;
    __IO uint32_t B2;
    __IO uint32_t PC2;
    __IO uint32_t IFIFO2;
    __IO uint32_t OFIFO2;
    __IO uint32_t RSVD5[2];
    __IO uint32_t CCR3;
    __IO uint32_t ICR3;
    __IO uint32_t OCR3;
    __IO uint32_t X3;
    __IO uint32_t Y3;
    __IO uint32_t A3;
    __IO uint32_t B3;
    __IO uint32_t PC3;
    __IO uint32_t IFIFO3;
    __IO uint32_t OFIFO3;
    __IO uint32_t RSVD4[2];
    __IO uint32_t CCR4;
    __IO uint32_t ICR4;
    __IO uint32_t OCR4;
    __IO uint32_t X4;
    __IO uint32_t Y4;
    __IO uint32_t A4;
    __IO uint32_t B4;
    __IO uint32_t PC4;
    __IO uint32_t IFIFO4;
    __IO uint32_t OFIFO4;
    __IO uint32_t RSVD3[2];
    __IO uint32_t CCR5;
    __IO uint32_t ICR5;
    __IO uint32_t OCR5;
    __IO uint32_t X5;
    __IO uint32_t Y5;
    __IO uint32_t A5;
    __IO uint32_t B5;
    __IO uint32_t PC5;
    __IO uint32_t IFIFO5;
    __IO uint32_t OFIFO5;
    __IO uint32_t RSVD2[2];
    __IO uint32_t CCR6;
    __IO uint32_t ICR6;
    __IO uint32_t OCR6;
    __IO uint32_t X6;
    __IO uint32_t Y6;
    __IO uint32_t A6;
    __IO uint32_t B6;
    __IO uint32_t PC6;
    __IO uint32_t IFIFO6;
    __IO uint32_t OFIFO6;
    __IO uint32_t RSVD1[2];
    __IO uint32_t CCR7;
    __IO uint32_t ICR7;
    __IO uint32_t OCR7;
    __IO uint32_t X7;
    __IO uint32_t Y7;
    __IO uint32_t A7;
    __IO uint32_t B7;
    __IO uint32_t PC7;
    __IO uint32_t IFIFO7;
    __IO uint32_t OFIFO7;
} PTM_TypeDef;

/**
  * @brief One core slice of PTM_TypeDef, i.e. the registers of a single core.
  *
  * The register block of the cores is repeated every 0x30 bytes starting at CCR0,
  * so this structure must stay identical to one core block of PTM_TypeDef,
  * reserved words included: its size is the distance between the registers of two
  * cores (PTM_CORE_REG_STRIDE in bf0_hal_ptm.c).  Keep the two definitions next
  * to each other when the register map changes.
  */
typedef struct
{
    __IO uint32_t CCR;      /*!< 0x00 control: PSC / REP / END0 / END1              */
    __IO uint32_t ICR;      /*!< 0x04 input  FIFO configuration, status in the MSBs */
    __IO uint32_t OCR;      /*!< 0x08 output FIFO configuration, status in the MSBs */
    __IO uint32_t X;        /*!< 0x0C 32 bit general purpose register X             */
    __IO uint32_t Y;        /*!< 0x10 32 bit general purpose register Y             */
    __IO uint32_t A;        /*!< 0x14 16 bit general purpose register A             */
    __IO uint32_t B;        /*!< 0x18 16 bit general purpose register B             */
    __IO uint32_t PC;       /*!< 0x1C program counter                               */
    __IO uint32_t IFIFO;    /*!< 0x20 input  FIFO data                              */
    __IO uint32_t OFIFO;    /*!< 0x24 output FIFO data                              */
    __IO uint32_t RSVD[2];  /*!< 0x28 reserved, pads the slice to 0x30 bytes        */
} PTM_CoreRegTypeDef;

/******************** Bit definition for PTM_ISR register *********************/
#define PTM_ISR_EVT0_Pos                (0U)
#define PTM_ISR_EVT0_Msk                (0x1UL << PTM_ISR_EVT0_Pos)
#define PTM_ISR_EVT0                    PTM_ISR_EVT0_Msk
#define PTM_ISR_EVT1_Pos                (1U)
#define PTM_ISR_EVT1_Msk                (0x1UL << PTM_ISR_EVT1_Pos)
#define PTM_ISR_EVT1                    PTM_ISR_EVT1_Msk
#define PTM_ISR_EVT2_Pos                (2U)
#define PTM_ISR_EVT2_Msk                (0x1UL << PTM_ISR_EVT2_Pos)
#define PTM_ISR_EVT2                    PTM_ISR_EVT2_Msk
#define PTM_ISR_EVT3_Pos                (3U)
#define PTM_ISR_EVT3_Msk                (0x1UL << PTM_ISR_EVT3_Pos)
#define PTM_ISR_EVT3                    PTM_ISR_EVT3_Msk
#define PTM_ISR_EVT4_Pos                (4U)
#define PTM_ISR_EVT4_Msk                (0x1UL << PTM_ISR_EVT4_Pos)
#define PTM_ISR_EVT4                    PTM_ISR_EVT4_Msk
#define PTM_ISR_EVT5_Pos                (5U)
#define PTM_ISR_EVT5_Msk                (0x1UL << PTM_ISR_EVT5_Pos)
#define PTM_ISR_EVT5                    PTM_ISR_EVT5_Msk
#define PTM_ISR_EVT6_Pos                (6U)
#define PTM_ISR_EVT6_Msk                (0x1UL << PTM_ISR_EVT6_Pos)
#define PTM_ISR_EVT6                    PTM_ISR_EVT6_Msk
#define PTM_ISR_EVT7_Pos                (7U)
#define PTM_ISR_EVT7_Msk                (0x1UL << PTM_ISR_EVT7_Pos)
#define PTM_ISR_EVT7                    PTM_ISR_EVT7_Msk
#define PTM_ISR_EVT8_Pos                (8U)
#define PTM_ISR_EVT8_Msk                (0x1UL << PTM_ISR_EVT8_Pos)
#define PTM_ISR_EVT8                    PTM_ISR_EVT8_Msk
#define PTM_ISR_EVT9_Pos                (9U)
#define PTM_ISR_EVT9_Msk                (0x1UL << PTM_ISR_EVT9_Pos)
#define PTM_ISR_EVT9                    PTM_ISR_EVT9_Msk
#define PTM_ISR_EVT10_Pos               (10U)
#define PTM_ISR_EVT10_Msk               (0x1UL << PTM_ISR_EVT10_Pos)
#define PTM_ISR_EVT10                   PTM_ISR_EVT10_Msk
#define PTM_ISR_EVT11_Pos               (11U)
#define PTM_ISR_EVT11_Msk               (0x1UL << PTM_ISR_EVT11_Pos)
#define PTM_ISR_EVT11                   PTM_ISR_EVT11_Msk
#define PTM_ISR_EVT12_Pos               (12U)
#define PTM_ISR_EVT12_Msk               (0x1UL << PTM_ISR_EVT12_Pos)
#define PTM_ISR_EVT12                   PTM_ISR_EVT12_Msk
#define PTM_ISR_EVT13_Pos               (13U)
#define PTM_ISR_EVT13_Msk               (0x1UL << PTM_ISR_EVT13_Pos)
#define PTM_ISR_EVT13                   PTM_ISR_EVT13_Msk
#define PTM_ISR_EVT14_Pos               (14U)
#define PTM_ISR_EVT14_Msk               (0x1UL << PTM_ISR_EVT14_Pos)
#define PTM_ISR_EVT14                   PTM_ISR_EVT14_Msk
#define PTM_ISR_EVT15_Pos               (15U)
#define PTM_ISR_EVT15_Msk               (0x1UL << PTM_ISR_EVT15_Pos)
#define PTM_ISR_EVT15                   PTM_ISR_EVT15_Msk
#define PTM_ISR_ERR0_Pos                (16U)
#define PTM_ISR_ERR0_Msk                (0x1UL << PTM_ISR_ERR0_Pos)
#define PTM_ISR_ERR0                    PTM_ISR_ERR0_Msk
#define PTM_ISR_ERR1_Pos                (17U)
#define PTM_ISR_ERR1_Msk                (0x1UL << PTM_ISR_ERR1_Pos)
#define PTM_ISR_ERR1                    PTM_ISR_ERR1_Msk
#define PTM_ISR_ERR2_Pos                (18U)
#define PTM_ISR_ERR2_Msk                (0x1UL << PTM_ISR_ERR2_Pos)
#define PTM_ISR_ERR2                    PTM_ISR_ERR2_Msk
#define PTM_ISR_ERR3_Pos                (19U)
#define PTM_ISR_ERR3_Msk                (0x1UL << PTM_ISR_ERR3_Pos)
#define PTM_ISR_ERR3                    PTM_ISR_ERR3_Msk
#define PTM_ISR_ERR4_Pos                (20U)
#define PTM_ISR_ERR4_Msk                (0x1UL << PTM_ISR_ERR4_Pos)
#define PTM_ISR_ERR4                    PTM_ISR_ERR4_Msk
#define PTM_ISR_ERR5_Pos                (21U)
#define PTM_ISR_ERR5_Msk                (0x1UL << PTM_ISR_ERR5_Pos)
#define PTM_ISR_ERR5                    PTM_ISR_ERR5_Msk
#define PTM_ISR_ERR6_Pos                (22U)
#define PTM_ISR_ERR6_Msk                (0x1UL << PTM_ISR_ERR6_Pos)
#define PTM_ISR_ERR6                    PTM_ISR_ERR6_Msk
#define PTM_ISR_ERR7_Pos                (23U)
#define PTM_ISR_ERR7_Msk                (0x1UL << PTM_ISR_ERR7_Pos)
#define PTM_ISR_ERR7                    PTM_ISR_ERR7_Msk
#define PTM_ISR_TCM0_Pos                (24U)
#define PTM_ISR_TCM0_Msk                (0x1UL << PTM_ISR_TCM0_Pos)
#define PTM_ISR_TCM0                    PTM_ISR_TCM0_Msk
#define PTM_ISR_TCM1_Pos                (25U)
#define PTM_ISR_TCM1_Msk                (0x1UL << PTM_ISR_TCM1_Pos)
#define PTM_ISR_TCM1                    PTM_ISR_TCM1_Msk
#define PTM_ISR_TCM2_Pos                (26U)
#define PTM_ISR_TCM2_Msk                (0x1UL << PTM_ISR_TCM2_Pos)
#define PTM_ISR_TCM2                    PTM_ISR_TCM2_Msk
#define PTM_ISR_TCM3_Pos                (27U)
#define PTM_ISR_TCM3_Msk                (0x1UL << PTM_ISR_TCM3_Pos)
#define PTM_ISR_TCM3                    PTM_ISR_TCM3_Msk
#define PTM_ISR_TCM4_Pos                (28U)
#define PTM_ISR_TCM4_Msk                (0x1UL << PTM_ISR_TCM4_Pos)
#define PTM_ISR_TCM4                    PTM_ISR_TCM4_Msk
#define PTM_ISR_TCM5_Pos                (29U)
#define PTM_ISR_TCM5_Msk                (0x1UL << PTM_ISR_TCM5_Pos)
#define PTM_ISR_TCM5                    PTM_ISR_TCM5_Msk
#define PTM_ISR_TCM6_Pos                (30U)
#define PTM_ISR_TCM6_Msk                (0x1UL << PTM_ISR_TCM6_Pos)
#define PTM_ISR_TCM6                    PTM_ISR_TCM6_Msk
#define PTM_ISR_TCM7_Pos                (31U)
#define PTM_ISR_TCM7_Msk                (0x1UL << PTM_ISR_TCM7_Pos)
#define PTM_ISR_TCM7                    PTM_ISR_TCM7_Msk

/******************** Bit definition for PTM_IER register *********************/
#define PTM_IER_EVTIE0_Pos              (0U)
#define PTM_IER_EVTIE0_Msk              (0x1UL << PTM_IER_EVTIE0_Pos)
#define PTM_IER_EVTIE0                  PTM_IER_EVTIE0_Msk
#define PTM_IER_EVTIE1_Pos              (1U)
#define PTM_IER_EVTIE1_Msk              (0x1UL << PTM_IER_EVTIE1_Pos)
#define PTM_IER_EVTIE1                  PTM_IER_EVTIE1_Msk
#define PTM_IER_EVTIE2_Pos              (2U)
#define PTM_IER_EVTIE2_Msk              (0x1UL << PTM_IER_EVTIE2_Pos)
#define PTM_IER_EVTIE2                  PTM_IER_EVTIE2_Msk
#define PTM_IER_EVTIE3_Pos              (3U)
#define PTM_IER_EVTIE3_Msk              (0x1UL << PTM_IER_EVTIE3_Pos)
#define PTM_IER_EVTIE3                  PTM_IER_EVTIE3_Msk
#define PTM_IER_EVTIE4_Pos              (4U)
#define PTM_IER_EVTIE4_Msk              (0x1UL << PTM_IER_EVTIE4_Pos)
#define PTM_IER_EVTIE4                  PTM_IER_EVTIE4_Msk
#define PTM_IER_EVTIE5_Pos              (5U)
#define PTM_IER_EVTIE5_Msk              (0x1UL << PTM_IER_EVTIE5_Pos)
#define PTM_IER_EVTIE5                  PTM_IER_EVTIE5_Msk
#define PTM_IER_EVTIE6_Pos              (6U)
#define PTM_IER_EVTIE6_Msk              (0x1UL << PTM_IER_EVTIE6_Pos)
#define PTM_IER_EVTIE6                  PTM_IER_EVTIE6_Msk
#define PTM_IER_EVTIE7_Pos              (7U)
#define PTM_IER_EVTIE7_Msk              (0x1UL << PTM_IER_EVTIE7_Pos)
#define PTM_IER_EVTIE7                  PTM_IER_EVTIE7_Msk
#define PTM_IER_EVTAUTO0_Pos            (8U)
#define PTM_IER_EVTAUTO0_Msk            (0x1UL << PTM_IER_EVTAUTO0_Pos)
#define PTM_IER_EVTAUTO0                PTM_IER_EVTAUTO0_Msk
#define PTM_IER_EVTAUTO1_Pos            (9U)
#define PTM_IER_EVTAUTO1_Msk            (0x1UL << PTM_IER_EVTAUTO1_Pos)
#define PTM_IER_EVTAUTO1                PTM_IER_EVTAUTO1_Msk
#define PTM_IER_EVTAUTO2_Pos            (10U)
#define PTM_IER_EVTAUTO2_Msk            (0x1UL << PTM_IER_EVTAUTO2_Pos)
#define PTM_IER_EVTAUTO2                PTM_IER_EVTAUTO2_Msk
#define PTM_IER_EVTAUTO3_Pos            (11U)
#define PTM_IER_EVTAUTO3_Msk            (0x1UL << PTM_IER_EVTAUTO3_Pos)
#define PTM_IER_EVTAUTO3                PTM_IER_EVTAUTO3_Msk
#define PTM_IER_EVTAUTO4_Pos            (12U)
#define PTM_IER_EVTAUTO4_Msk            (0x1UL << PTM_IER_EVTAUTO4_Pos)
#define PTM_IER_EVTAUTO4                PTM_IER_EVTAUTO4_Msk
#define PTM_IER_EVTAUTO5_Pos            (13U)
#define PTM_IER_EVTAUTO5_Msk            (0x1UL << PTM_IER_EVTAUTO5_Pos)
#define PTM_IER_EVTAUTO5                PTM_IER_EVTAUTO5_Msk
#define PTM_IER_EVTAUTO6_Pos            (14U)
#define PTM_IER_EVTAUTO6_Msk            (0x1UL << PTM_IER_EVTAUTO6_Pos)
#define PTM_IER_EVTAUTO6                PTM_IER_EVTAUTO6_Msk
#define PTM_IER_EVTAUTO7_Pos            (15U)
#define PTM_IER_EVTAUTO7_Msk            (0x1UL << PTM_IER_EVTAUTO7_Pos)
#define PTM_IER_EVTAUTO7                PTM_IER_EVTAUTO7_Msk
#define PTM_IER_ERRIE0_Pos              (16U)
#define PTM_IER_ERRIE0_Msk              (0x1UL << PTM_IER_ERRIE0_Pos)
#define PTM_IER_ERRIE0                  PTM_IER_ERRIE0_Msk
#define PTM_IER_ERRIE1_Pos              (17U)
#define PTM_IER_ERRIE1_Msk              (0x1UL << PTM_IER_ERRIE1_Pos)
#define PTM_IER_ERRIE1                  PTM_IER_ERRIE1_Msk
#define PTM_IER_ERRIE2_Pos              (18U)
#define PTM_IER_ERRIE2_Msk              (0x1UL << PTM_IER_ERRIE2_Pos)
#define PTM_IER_ERRIE2                  PTM_IER_ERRIE2_Msk
#define PTM_IER_ERRIE3_Pos              (19U)
#define PTM_IER_ERRIE3_Msk              (0x1UL << PTM_IER_ERRIE3_Pos)
#define PTM_IER_ERRIE3                  PTM_IER_ERRIE3_Msk
#define PTM_IER_ERRIE4_Pos              (20U)
#define PTM_IER_ERRIE4_Msk              (0x1UL << PTM_IER_ERRIE4_Pos)
#define PTM_IER_ERRIE4                  PTM_IER_ERRIE4_Msk
#define PTM_IER_ERRIE5_Pos              (21U)
#define PTM_IER_ERRIE5_Msk              (0x1UL << PTM_IER_ERRIE5_Pos)
#define PTM_IER_ERRIE5                  PTM_IER_ERRIE5_Msk
#define PTM_IER_ERRIE6_Pos              (22U)
#define PTM_IER_ERRIE6_Msk              (0x1UL << PTM_IER_ERRIE6_Pos)
#define PTM_IER_ERRIE6                  PTM_IER_ERRIE6_Msk
#define PTM_IER_ERRIE7_Pos              (23U)
#define PTM_IER_ERRIE7_Msk              (0x1UL << PTM_IER_ERRIE7_Pos)
#define PTM_IER_ERRIE7                  PTM_IER_ERRIE7_Msk

/******************** Bit definition for PTM_CER register *********************/
#define PTM_CER_EN0_Pos                 (0U)
#define PTM_CER_EN0_Msk                 (0x1UL << PTM_CER_EN0_Pos)
#define PTM_CER_EN0                     PTM_CER_EN0_Msk
#define PTM_CER_EN1_Pos                 (1U)
#define PTM_CER_EN1_Msk                 (0x1UL << PTM_CER_EN1_Pos)
#define PTM_CER_EN1                     PTM_CER_EN1_Msk
#define PTM_CER_EN2_Pos                 (2U)
#define PTM_CER_EN2_Msk                 (0x1UL << PTM_CER_EN2_Pos)
#define PTM_CER_EN2                     PTM_CER_EN2_Msk
#define PTM_CER_EN3_Pos                 (3U)
#define PTM_CER_EN3_Msk                 (0x1UL << PTM_CER_EN3_Pos)
#define PTM_CER_EN3                     PTM_CER_EN3_Msk
#define PTM_CER_EN4_Pos                 (4U)
#define PTM_CER_EN4_Msk                 (0x1UL << PTM_CER_EN4_Pos)
#define PTM_CER_EN4                     PTM_CER_EN4_Msk
#define PTM_CER_EN5_Pos                 (5U)
#define PTM_CER_EN5_Msk                 (0x1UL << PTM_CER_EN5_Pos)
#define PTM_CER_EN5                     PTM_CER_EN5_Msk
#define PTM_CER_EN6_Pos                 (6U)
#define PTM_CER_EN6_Msk                 (0x1UL << PTM_CER_EN6_Pos)
#define PTM_CER_EN6                     PTM_CER_EN6_Msk
#define PTM_CER_EN7_Pos                 (7U)
#define PTM_CER_EN7_Msk                 (0x1UL << PTM_CER_EN7_Pos)
#define PTM_CER_EN7                     PTM_CER_EN7_Msk
#define PTM_CER_STOP0_Pos               (8U)
#define PTM_CER_STOP0_Msk               (0x1UL << PTM_CER_STOP0_Pos)
#define PTM_CER_STOP0                   PTM_CER_STOP0_Msk
#define PTM_CER_STOP1_Pos               (9U)
#define PTM_CER_STOP1_Msk               (0x1UL << PTM_CER_STOP1_Pos)
#define PTM_CER_STOP1                   PTM_CER_STOP1_Msk
#define PTM_CER_STOP2_Pos               (10U)
#define PTM_CER_STOP2_Msk               (0x1UL << PTM_CER_STOP2_Pos)
#define PTM_CER_STOP2                   PTM_CER_STOP2_Msk
#define PTM_CER_STOP3_Pos               (11U)
#define PTM_CER_STOP3_Msk               (0x1UL << PTM_CER_STOP3_Pos)
#define PTM_CER_STOP3                   PTM_CER_STOP3_Msk
#define PTM_CER_STOP4_Pos               (12U)
#define PTM_CER_STOP4_Msk               (0x1UL << PTM_CER_STOP4_Pos)
#define PTM_CER_STOP4                   PTM_CER_STOP4_Msk
#define PTM_CER_STOP5_Pos               (13U)
#define PTM_CER_STOP5_Msk               (0x1UL << PTM_CER_STOP5_Pos)
#define PTM_CER_STOP5                   PTM_CER_STOP5_Msk
#define PTM_CER_STOP6_Pos               (14U)
#define PTM_CER_STOP6_Msk               (0x1UL << PTM_CER_STOP6_Pos)
#define PTM_CER_STOP6                   PTM_CER_STOP6_Msk
#define PTM_CER_STOP7_Pos               (15U)
#define PTM_CER_STOP7_Msk               (0x1UL << PTM_CER_STOP7_Pos)
#define PTM_CER_STOP7                   PTM_CER_STOP7_Msk
#define PTM_CER_RST0_Pos                (16U)
#define PTM_CER_RST0_Msk                (0x1UL << PTM_CER_RST0_Pos)
#define PTM_CER_RST0                    PTM_CER_RST0_Msk
#define PTM_CER_RST1_Pos                (17U)
#define PTM_CER_RST1_Msk                (0x1UL << PTM_CER_RST1_Pos)
#define PTM_CER_RST1                    PTM_CER_RST1_Msk
#define PTM_CER_RST2_Pos                (18U)
#define PTM_CER_RST2_Msk                (0x1UL << PTM_CER_RST2_Pos)
#define PTM_CER_RST2                    PTM_CER_RST2_Msk
#define PTM_CER_RST3_Pos                (19U)
#define PTM_CER_RST3_Msk                (0x1UL << PTM_CER_RST3_Pos)
#define PTM_CER_RST3                    PTM_CER_RST3_Msk
#define PTM_CER_RST4_Pos                (20U)
#define PTM_CER_RST4_Msk                (0x1UL << PTM_CER_RST4_Pos)
#define PTM_CER_RST4                    PTM_CER_RST4_Msk
#define PTM_CER_RST5_Pos                (21U)
#define PTM_CER_RST5_Msk                (0x1UL << PTM_CER_RST5_Pos)
#define PTM_CER_RST5                    PTM_CER_RST5_Msk
#define PTM_CER_RST6_Pos                (22U)
#define PTM_CER_RST6_Msk                (0x1UL << PTM_CER_RST6_Pos)
#define PTM_CER_RST6                    PTM_CER_RST6_Msk
#define PTM_CER_RST7_Pos                (23U)
#define PTM_CER_RST7_Msk                (0x1UL << PTM_CER_RST7_Pos)
#define PTM_CER_RST7                    PTM_CER_RST7_Msk
#define PTM_CER_STEP0_Pos               (24U)
#define PTM_CER_STEP0_Msk               (0x1UL << PTM_CER_STEP0_Pos)
#define PTM_CER_STEP0                   PTM_CER_STEP0_Msk
#define PTM_CER_STEP1_Pos               (25U)
#define PTM_CER_STEP1_Msk               (0x1UL << PTM_CER_STEP1_Pos)
#define PTM_CER_STEP1                   PTM_CER_STEP1_Msk
#define PTM_CER_STEP2_Pos               (26U)
#define PTM_CER_STEP2_Msk               (0x1UL << PTM_CER_STEP2_Pos)
#define PTM_CER_STEP2                   PTM_CER_STEP2_Msk
#define PTM_CER_STEP3_Pos               (27U)
#define PTM_CER_STEP3_Msk               (0x1UL << PTM_CER_STEP3_Pos)
#define PTM_CER_STEP3                   PTM_CER_STEP3_Msk
#define PTM_CER_STEP4_Pos               (28U)
#define PTM_CER_STEP4_Msk               (0x1UL << PTM_CER_STEP4_Pos)
#define PTM_CER_STEP4                   PTM_CER_STEP4_Msk
#define PTM_CER_STEP5_Pos               (29U)
#define PTM_CER_STEP5_Msk               (0x1UL << PTM_CER_STEP5_Pos)
#define PTM_CER_STEP5                   PTM_CER_STEP5_Msk
#define PTM_CER_STEP6_Pos               (30U)
#define PTM_CER_STEP6_Msk               (0x1UL << PTM_CER_STEP6_Pos)
#define PTM_CER_STEP6                   PTM_CER_STEP6_Msk
#define PTM_CER_STEP7_Pos               (31U)
#define PTM_CER_STEP7_Msk               (0x1UL << PTM_CER_STEP7_Pos)
#define PTM_CER_STEP7                   PTM_CER_STEP7_Msk

/******************** Bit definition for PTM_DOER register ********************/
#define PTM_DOER_DOE_Pos                (0U)
#define PTM_DOER_DOE_Msk                (0xFFFFUL << PTM_DOER_DOE_Pos)
#define PTM_DOER_DOE                    PTM_DOER_DOE_Msk

/******************** Bit definition for PTM_DOR register *********************/
#define PTM_DOR_DO_Pos                  (0U)
#define PTM_DOR_DO_Msk                  (0xFFFFUL << PTM_DOR_DO_Pos)
#define PTM_DOR_DO                      PTM_DOR_DO_Msk

/******************* Bit definition for PTM_DOESR register ********************/
#define PTM_DOESR_DOES_Pos              (0U)
#define PTM_DOESR_DOES_Msk              (0xFFFFUL << PTM_DOESR_DOES_Pos)
#define PTM_DOESR_DOES                  PTM_DOESR_DOES_Msk

/******************* Bit definition for PTM_DOECR register ********************/
#define PTM_DOECR_DOEC_Pos              (0U)
#define PTM_DOECR_DOEC_Msk              (0xFFFFUL << PTM_DOECR_DOEC_Pos)
#define PTM_DOECR_DOEC                  PTM_DOECR_DOEC_Msk

/******************** Bit definition for PTM_DOSR register ********************/
#define PTM_DOSR_DOS_Pos                (0U)
#define PTM_DOSR_DOS_Msk                (0xFFFFUL << PTM_DOSR_DOS_Pos)
#define PTM_DOSR_DOS                    PTM_DOSR_DOS_Msk

/******************** Bit definition for PTM_DOCR register ********************/
#define PTM_DOCR_DOC_Pos                (0U)
#define PTM_DOCR_DOC_Msk                (0xFFFFUL << PTM_DOCR_DOC_Pos)
#define PTM_DOCR_DOC                    PTM_DOCR_DOC_Msk

/******************** Bit definition for PTM_CCR0 register ********************/
#define PTM_CCR0_PSC_Pos                (0U)
#define PTM_CCR0_PSC_Msk                (0xFFUL << PTM_CCR0_PSC_Pos)
#define PTM_CCR0_PSC                    PTM_CCR0_PSC_Msk
#define PTM_CCR0_REP_Pos                (8U)
#define PTM_CCR0_REP_Msk                (0xFFUL << PTM_CCR0_REP_Pos)
#define PTM_CCR0_REP                    PTM_CCR0_REP_Msk
#define PTM_CCR0_END0_Pos               (16U)
#define PTM_CCR0_END0_Msk               (0xFFUL << PTM_CCR0_END0_Pos)
#define PTM_CCR0_END0                   PTM_CCR0_END0_Msk
#define PTM_CCR0_END1_Pos               (24U)
#define PTM_CCR0_END1_Msk               (0xFFUL << PTM_CCR0_END1_Pos)
#define PTM_CCR0_END1                   PTM_CCR0_END1_Msk

/******************** Bit definition for PTM_ICR0 register ********************/
#define PTM_ICR0_IFIFO_Pos              (0U)
#define PTM_ICR0_IFIFO_Msk              (0x1UL << PTM_ICR0_IFIFO_Pos)
#define PTM_ICR0_IFIFO                  PTM_ICR0_IFIFO_Msk
#define PTM_ICR0_IAUTO_Pos              (1U)
#define PTM_ICR0_IAUTO_Msk              (0x1UL << PTM_ICR0_IAUTO_Pos)
#define PTM_ICR0_IAUTO                  PTM_ICR0_IAUTO_Msk
#define PTM_ICR0_IRSH_Pos               (2U)
#define PTM_ICR0_IRSH_Msk               (0x1UL << PTM_ICR0_IRSH_Pos)
#define PTM_ICR0_IRSH                   PTM_ICR0_IRSH_Msk
#define PTM_ICR0_IBLOCK_Pos             (3U)
#define PTM_ICR0_IBLOCK_Msk             (0x1UL << PTM_ICR0_IBLOCK_Pos)
#define PTM_ICR0_IBLOCK                 PTM_ICR0_IBLOCK_Msk
#define PTM_ICR0_IBASE_Pos              (8U)
#define PTM_ICR0_IBASE_Msk              (0xFUL << PTM_ICR0_IBASE_Pos)
#define PTM_ICR0_IBASE                  PTM_ICR0_IBASE_Msk
#define PTM_ICR0_ISHIFT_Pos             (16U)
#define PTM_ICR0_ISHIFT_Msk             (0x1FUL << PTM_ICR0_ISHIFT_Pos)
#define PTM_ICR0_ISHIFT                 PTM_ICR0_ISHIFT_Msk
#define PTM_ICR0_ILIMIT_Pos             (24U)
#define PTM_ICR0_ILIMIT_Msk             (0x1FUL << PTM_ICR0_ILIMIT_Pos)
#define PTM_ICR0_ILIMIT                 PTM_ICR0_ILIMIT_Msk
#define PTM_ICR0_IOF_Pos                (29U)
#define PTM_ICR0_IOF_Msk                (0x1UL << PTM_ICR0_IOF_Pos)
#define PTM_ICR0_IOF                    PTM_ICR0_IOF_Msk
#define PTM_ICR0_IEMPTY_Pos             (30U)
#define PTM_ICR0_IEMPTY_Msk             (0x1UL << PTM_ICR0_IEMPTY_Pos)
#define PTM_ICR0_IEMPTY                 PTM_ICR0_IEMPTY_Msk
#define PTM_ICR0_IFULL_Pos              (31U)
#define PTM_ICR0_IFULL_Msk              (0x1UL << PTM_ICR0_IFULL_Pos)
#define PTM_ICR0_IFULL                  PTM_ICR0_IFULL_Msk

/******************** Bit definition for PTM_OCR0 register ********************/
#define PTM_OCR0_OFIFO_Pos              (0U)
#define PTM_OCR0_OFIFO_Msk              (0x1UL << PTM_OCR0_OFIFO_Pos)
#define PTM_OCR0_OFIFO                  PTM_OCR0_OFIFO_Msk
#define PTM_OCR0_OAUTO_Pos              (1U)
#define PTM_OCR0_OAUTO_Msk              (0x1UL << PTM_OCR0_OAUTO_Pos)
#define PTM_OCR0_OAUTO                  PTM_OCR0_OAUTO_Msk
#define PTM_OCR0_ORSH_Pos               (2U)
#define PTM_OCR0_ORSH_Msk               (0x1UL << PTM_OCR0_ORSH_Pos)
#define PTM_OCR0_ORSH                   PTM_OCR0_ORSH_Msk
#define PTM_OCR0_OBLOCK_Pos             (3U)
#define PTM_OCR0_OBLOCK_Msk             (0x1UL << PTM_OCR0_OBLOCK_Pos)
#define PTM_OCR0_OBLOCK                 PTM_OCR0_OBLOCK_Msk
#define PTM_OCR0_OBASE_Pos              (8U)
#define PTM_OCR0_OBASE_Msk              (0xFUL << PTM_OCR0_OBASE_Pos)
#define PTM_OCR0_OBASE                  PTM_OCR0_OBASE_Msk
#define PTM_OCR0_OSHIFT_Pos             (16U)
#define PTM_OCR0_OSHIFT_Msk             (0x1FUL << PTM_OCR0_OSHIFT_Pos)
#define PTM_OCR0_OSHIFT                 PTM_OCR0_OSHIFT_Msk
#define PTM_OCR0_OLIMIT_Pos             (24U)
#define PTM_OCR0_OLIMIT_Msk             (0x1FUL << PTM_OCR0_OLIMIT_Pos)
#define PTM_OCR0_OLIMIT                 PTM_OCR0_OLIMIT_Msk
#define PTM_OCR0_OUF_Pos                (29U)
#define PTM_OCR0_OUF_Msk                (0x1UL << PTM_OCR0_OUF_Pos)
#define PTM_OCR0_OUF                    PTM_OCR0_OUF_Msk
#define PTM_OCR0_OEMPTY_Pos             (30U)
#define PTM_OCR0_OEMPTY_Msk             (0x1UL << PTM_OCR0_OEMPTY_Pos)
#define PTM_OCR0_OEMPTY                 PTM_OCR0_OEMPTY_Msk
#define PTM_OCR0_OFULL_Pos              (31U)
#define PTM_OCR0_OFULL_Msk              (0x1UL << PTM_OCR0_OFULL_Pos)
#define PTM_OCR0_OFULL                  PTM_OCR0_OFULL_Msk

/********************* Bit definition for PTM_X0 register *********************/
#define PTM_X0_VAL_Pos                  (0U)
#define PTM_X0_VAL_Msk                  (0xFFFFFFFFUL << PTM_X0_VAL_Pos)
#define PTM_X0_VAL                      PTM_X0_VAL_Msk

/********************* Bit definition for PTM_Y0 register *********************/
#define PTM_Y0_VAL_Pos                  (0U)
#define PTM_Y0_VAL_Msk                  (0xFFFFFFFFUL << PTM_Y0_VAL_Pos)
#define PTM_Y0_VAL                      PTM_Y0_VAL_Msk

/********************* Bit definition for PTM_A0 register *********************/
#define PTM_A0_VAL_Pos                  (0U)
#define PTM_A0_VAL_Msk                  (0xFFFFUL << PTM_A0_VAL_Pos)
#define PTM_A0_VAL                      PTM_A0_VAL_Msk

/********************* Bit definition for PTM_B0 register *********************/
#define PTM_B0_VAL_Pos                  (0U)
#define PTM_B0_VAL_Msk                  (0xFFFFUL << PTM_B0_VAL_Pos)
#define PTM_B0_VAL                      PTM_B0_VAL_Msk

/******************** Bit definition for PTM_PC0 register *********************/
#define PTM_PC0_VAL_Pos                 (0U)
#define PTM_PC0_VAL_Msk                 (0xFFFFFFFFUL << PTM_PC0_VAL_Pos)
#define PTM_PC0_VAL                     PTM_PC0_VAL_Msk

/******************* Bit definition for PTM_IFIFO0 register *******************/
#define PTM_IFIFO0_DAT_Pos              (0U)
#define PTM_IFIFO0_DAT_Msk              (0xFFFFFFFFUL << PTM_IFIFO0_DAT_Pos)
#define PTM_IFIFO0_DAT                  PTM_IFIFO0_DAT_Msk

/******************* Bit definition for PTM_OFIFO0 register *******************/
#define PTM_OFIFO0_DAT_Pos              (0U)
#define PTM_OFIFO0_DAT_Msk              (0xFFFFFFFFUL << PTM_OFIFO0_DAT_Pos)
#define PTM_OFIFO0_DAT                  PTM_OFIFO0_DAT_Msk

/******************** Bit definition for PTM_CCR1 register ********************/
#define PTM_CCR1_PSC_Pos                (0U)
#define PTM_CCR1_PSC_Msk                (0xFFUL << PTM_CCR1_PSC_Pos)
#define PTM_CCR1_PSC                    PTM_CCR1_PSC_Msk
#define PTM_CCR1_REP_Pos                (8U)
#define PTM_CCR1_REP_Msk                (0xFFUL << PTM_CCR1_REP_Pos)
#define PTM_CCR1_REP                    PTM_CCR1_REP_Msk
#define PTM_CCR1_END0_Pos               (16U)
#define PTM_CCR1_END0_Msk               (0xFFUL << PTM_CCR1_END0_Pos)
#define PTM_CCR1_END0                   PTM_CCR1_END0_Msk
#define PTM_CCR1_END1_Pos               (24U)
#define PTM_CCR1_END1_Msk               (0xFFUL << PTM_CCR1_END1_Pos)
#define PTM_CCR1_END1                   PTM_CCR1_END1_Msk

/******************** Bit definition for PTM_ICR1 register ********************/
#define PTM_ICR1_IFIFO_Pos              (0U)
#define PTM_ICR1_IFIFO_Msk              (0x1UL << PTM_ICR1_IFIFO_Pos)
#define PTM_ICR1_IFIFO                  PTM_ICR1_IFIFO_Msk
#define PTM_ICR1_IAUTO_Pos              (1U)
#define PTM_ICR1_IAUTO_Msk              (0x1UL << PTM_ICR1_IAUTO_Pos)
#define PTM_ICR1_IAUTO                  PTM_ICR1_IAUTO_Msk
#define PTM_ICR1_IRSH_Pos               (2U)
#define PTM_ICR1_IRSH_Msk               (0x1UL << PTM_ICR1_IRSH_Pos)
#define PTM_ICR1_IRSH                   PTM_ICR1_IRSH_Msk
#define PTM_ICR1_IBLOCK_Pos             (3U)
#define PTM_ICR1_IBLOCK_Msk             (0x1UL << PTM_ICR1_IBLOCK_Pos)
#define PTM_ICR1_IBLOCK                 PTM_ICR1_IBLOCK_Msk
#define PTM_ICR1_IBASE_Pos              (8U)
#define PTM_ICR1_IBASE_Msk              (0xFUL << PTM_ICR1_IBASE_Pos)
#define PTM_ICR1_IBASE                  PTM_ICR1_IBASE_Msk
#define PTM_ICR1_ISHIFT_Pos             (16U)
#define PTM_ICR1_ISHIFT_Msk             (0x1FUL << PTM_ICR1_ISHIFT_Pos)
#define PTM_ICR1_ISHIFT                 PTM_ICR1_ISHIFT_Msk
#define PTM_ICR1_ILIMIT_Pos             (24U)
#define PTM_ICR1_ILIMIT_Msk             (0x1FUL << PTM_ICR1_ILIMIT_Pos)
#define PTM_ICR1_ILIMIT                 PTM_ICR1_ILIMIT_Msk
#define PTM_ICR1_IOF_Pos                (29U)
#define PTM_ICR1_IOF_Msk                (0x1UL << PTM_ICR1_IOF_Pos)
#define PTM_ICR1_IOF                    PTM_ICR1_IOF_Msk
#define PTM_ICR1_IEMPTY_Pos             (30U)
#define PTM_ICR1_IEMPTY_Msk             (0x1UL << PTM_ICR1_IEMPTY_Pos)
#define PTM_ICR1_IEMPTY                 PTM_ICR1_IEMPTY_Msk
#define PTM_ICR1_IFULL_Pos              (31U)
#define PTM_ICR1_IFULL_Msk              (0x1UL << PTM_ICR1_IFULL_Pos)
#define PTM_ICR1_IFULL                  PTM_ICR1_IFULL_Msk

/******************** Bit definition for PTM_OCR1 register ********************/
#define PTM_OCR1_OFIFO_Pos              (0U)
#define PTM_OCR1_OFIFO_Msk              (0x1UL << PTM_OCR1_OFIFO_Pos)
#define PTM_OCR1_OFIFO                  PTM_OCR1_OFIFO_Msk
#define PTM_OCR1_OAUTO_Pos              (1U)
#define PTM_OCR1_OAUTO_Msk              (0x1UL << PTM_OCR1_OAUTO_Pos)
#define PTM_OCR1_OAUTO                  PTM_OCR1_OAUTO_Msk
#define PTM_OCR1_ORSH_Pos               (2U)
#define PTM_OCR1_ORSH_Msk               (0x1UL << PTM_OCR1_ORSH_Pos)
#define PTM_OCR1_ORSH                   PTM_OCR1_ORSH_Msk
#define PTM_OCR1_OBLOCK_Pos             (3U)
#define PTM_OCR1_OBLOCK_Msk             (0x1UL << PTM_OCR1_OBLOCK_Pos)
#define PTM_OCR1_OBLOCK                 PTM_OCR1_OBLOCK_Msk
#define PTM_OCR1_OBASE_Pos              (8U)
#define PTM_OCR1_OBASE_Msk              (0xFUL << PTM_OCR1_OBASE_Pos)
#define PTM_OCR1_OBASE                  PTM_OCR1_OBASE_Msk
#define PTM_OCR1_OSHIFT_Pos             (16U)
#define PTM_OCR1_OSHIFT_Msk             (0x1FUL << PTM_OCR1_OSHIFT_Pos)
#define PTM_OCR1_OSHIFT                 PTM_OCR1_OSHIFT_Msk
#define PTM_OCR1_OLIMIT_Pos             (24U)
#define PTM_OCR1_OLIMIT_Msk             (0x1FUL << PTM_OCR1_OLIMIT_Pos)
#define PTM_OCR1_OLIMIT                 PTM_OCR1_OLIMIT_Msk
#define PTM_OCR1_OUF_Pos                (29U)
#define PTM_OCR1_OUF_Msk                (0x1UL << PTM_OCR1_OUF_Pos)
#define PTM_OCR1_OUF                    PTM_OCR1_OUF_Msk
#define PTM_OCR1_OEMPTY_Pos             (30U)
#define PTM_OCR1_OEMPTY_Msk             (0x1UL << PTM_OCR1_OEMPTY_Pos)
#define PTM_OCR1_OEMPTY                 PTM_OCR1_OEMPTY_Msk
#define PTM_OCR1_OFULL_Pos              (31U)
#define PTM_OCR1_OFULL_Msk              (0x1UL << PTM_OCR1_OFULL_Pos)
#define PTM_OCR1_OFULL                  PTM_OCR1_OFULL_Msk

/********************* Bit definition for PTM_X1 register *********************/
#define PTM_X1_VAL_Pos                  (0U)
#define PTM_X1_VAL_Msk                  (0xFFFFFFFFUL << PTM_X1_VAL_Pos)
#define PTM_X1_VAL                      PTM_X1_VAL_Msk

/********************* Bit definition for PTM_Y1 register *********************/
#define PTM_Y1_VAL_Pos                  (0U)
#define PTM_Y1_VAL_Msk                  (0xFFFFFFFFUL << PTM_Y1_VAL_Pos)
#define PTM_Y1_VAL                      PTM_Y1_VAL_Msk

/********************* Bit definition for PTM_A1 register *********************/
#define PTM_A1_VAL_Pos                  (0U)
#define PTM_A1_VAL_Msk                  (0xFFFFUL << PTM_A1_VAL_Pos)
#define PTM_A1_VAL                      PTM_A1_VAL_Msk

/********************* Bit definition for PTM_B1 register *********************/
#define PTM_B1_VAL_Pos                  (0U)
#define PTM_B1_VAL_Msk                  (0xFFFFUL << PTM_B1_VAL_Pos)
#define PTM_B1_VAL                      PTM_B1_VAL_Msk

/******************** Bit definition for PTM_PC1 register *********************/
#define PTM_PC1_VAL_Pos                 (0U)
#define PTM_PC1_VAL_Msk                 (0xFFFFFFFFUL << PTM_PC1_VAL_Pos)
#define PTM_PC1_VAL                     PTM_PC1_VAL_Msk

/******************* Bit definition for PTM_IFIFO1 register *******************/
#define PTM_IFIFO1_DAT_Pos              (0U)
#define PTM_IFIFO1_DAT_Msk              (0xFFFFFFFFUL << PTM_IFIFO1_DAT_Pos)
#define PTM_IFIFO1_DAT                  PTM_IFIFO1_DAT_Msk

/******************* Bit definition for PTM_OFIFO1 register *******************/
#define PTM_OFIFO1_DAT_Pos              (0U)
#define PTM_OFIFO1_DAT_Msk              (0xFFFFFFFFUL << PTM_OFIFO1_DAT_Pos)
#define PTM_OFIFO1_DAT                  PTM_OFIFO1_DAT_Msk

/******************** Bit definition for PTM_CCR2 register ********************/
#define PTM_CCR2_PSC_Pos                (0U)
#define PTM_CCR2_PSC_Msk                (0xFFUL << PTM_CCR2_PSC_Pos)
#define PTM_CCR2_PSC                    PTM_CCR2_PSC_Msk
#define PTM_CCR2_REP_Pos                (8U)
#define PTM_CCR2_REP_Msk                (0xFFUL << PTM_CCR2_REP_Pos)
#define PTM_CCR2_REP                    PTM_CCR2_REP_Msk
#define PTM_CCR2_END0_Pos               (16U)
#define PTM_CCR2_END0_Msk               (0xFFUL << PTM_CCR2_END0_Pos)
#define PTM_CCR2_END0                   PTM_CCR2_END0_Msk
#define PTM_CCR2_END1_Pos               (24U)
#define PTM_CCR2_END1_Msk               (0xFFUL << PTM_CCR2_END1_Pos)
#define PTM_CCR2_END1                   PTM_CCR2_END1_Msk

/******************** Bit definition for PTM_ICR2 register ********************/
#define PTM_ICR2_IFIFO_Pos              (0U)
#define PTM_ICR2_IFIFO_Msk              (0x1UL << PTM_ICR2_IFIFO_Pos)
#define PTM_ICR2_IFIFO                  PTM_ICR2_IFIFO_Msk
#define PTM_ICR2_IAUTO_Pos              (1U)
#define PTM_ICR2_IAUTO_Msk              (0x1UL << PTM_ICR2_IAUTO_Pos)
#define PTM_ICR2_IAUTO                  PTM_ICR2_IAUTO_Msk
#define PTM_ICR2_IRSH_Pos               (2U)
#define PTM_ICR2_IRSH_Msk               (0x1UL << PTM_ICR2_IRSH_Pos)
#define PTM_ICR2_IRSH                   PTM_ICR2_IRSH_Msk
#define PTM_ICR2_IBLOCK_Pos             (3U)
#define PTM_ICR2_IBLOCK_Msk             (0x1UL << PTM_ICR2_IBLOCK_Pos)
#define PTM_ICR2_IBLOCK                 PTM_ICR2_IBLOCK_Msk
#define PTM_ICR2_IBASE_Pos              (8U)
#define PTM_ICR2_IBASE_Msk              (0xFUL << PTM_ICR2_IBASE_Pos)
#define PTM_ICR2_IBASE                  PTM_ICR2_IBASE_Msk
#define PTM_ICR2_ISHIFT_Pos             (16U)
#define PTM_ICR2_ISHIFT_Msk             (0x1FUL << PTM_ICR2_ISHIFT_Pos)
#define PTM_ICR2_ISHIFT                 PTM_ICR2_ISHIFT_Msk
#define PTM_ICR2_ILIMIT_Pos             (24U)
#define PTM_ICR2_ILIMIT_Msk             (0x1FUL << PTM_ICR2_ILIMIT_Pos)
#define PTM_ICR2_ILIMIT                 PTM_ICR2_ILIMIT_Msk
#define PTM_ICR2_IOF_Pos                (29U)
#define PTM_ICR2_IOF_Msk                (0x1UL << PTM_ICR2_IOF_Pos)
#define PTM_ICR2_IOF                    PTM_ICR2_IOF_Msk
#define PTM_ICR2_IEMPTY_Pos             (30U)
#define PTM_ICR2_IEMPTY_Msk             (0x1UL << PTM_ICR2_IEMPTY_Pos)
#define PTM_ICR2_IEMPTY                 PTM_ICR2_IEMPTY_Msk
#define PTM_ICR2_IFULL_Pos              (31U)
#define PTM_ICR2_IFULL_Msk              (0x1UL << PTM_ICR2_IFULL_Pos)
#define PTM_ICR2_IFULL                  PTM_ICR2_IFULL_Msk

/******************** Bit definition for PTM_OCR2 register ********************/
#define PTM_OCR2_OFIFO_Pos              (0U)
#define PTM_OCR2_OFIFO_Msk              (0x1UL << PTM_OCR2_OFIFO_Pos)
#define PTM_OCR2_OFIFO                  PTM_OCR2_OFIFO_Msk
#define PTM_OCR2_OAUTO_Pos              (1U)
#define PTM_OCR2_OAUTO_Msk              (0x1UL << PTM_OCR2_OAUTO_Pos)
#define PTM_OCR2_OAUTO                  PTM_OCR2_OAUTO_Msk
#define PTM_OCR2_ORSH_Pos               (2U)
#define PTM_OCR2_ORSH_Msk               (0x1UL << PTM_OCR2_ORSH_Pos)
#define PTM_OCR2_ORSH                   PTM_OCR2_ORSH_Msk
#define PTM_OCR2_OBLOCK_Pos             (3U)
#define PTM_OCR2_OBLOCK_Msk             (0x1UL << PTM_OCR2_OBLOCK_Pos)
#define PTM_OCR2_OBLOCK                 PTM_OCR2_OBLOCK_Msk
#define PTM_OCR2_OBASE_Pos              (8U)
#define PTM_OCR2_OBASE_Msk              (0xFUL << PTM_OCR2_OBASE_Pos)
#define PTM_OCR2_OBASE                  PTM_OCR2_OBASE_Msk
#define PTM_OCR2_OSHIFT_Pos             (16U)
#define PTM_OCR2_OSHIFT_Msk             (0x1FUL << PTM_OCR2_OSHIFT_Pos)
#define PTM_OCR2_OSHIFT                 PTM_OCR2_OSHIFT_Msk
#define PTM_OCR2_OLIMIT_Pos             (24U)
#define PTM_OCR2_OLIMIT_Msk             (0x1FUL << PTM_OCR2_OLIMIT_Pos)
#define PTM_OCR2_OLIMIT                 PTM_OCR2_OLIMIT_Msk
#define PTM_OCR2_OUF_Pos                (29U)
#define PTM_OCR2_OUF_Msk                (0x1UL << PTM_OCR2_OUF_Pos)
#define PTM_OCR2_OUF                    PTM_OCR2_OUF_Msk
#define PTM_OCR2_OEMPTY_Pos             (30U)
#define PTM_OCR2_OEMPTY_Msk             (0x1UL << PTM_OCR2_OEMPTY_Pos)
#define PTM_OCR2_OEMPTY                 PTM_OCR2_OEMPTY_Msk
#define PTM_OCR2_OFULL_Pos              (31U)
#define PTM_OCR2_OFULL_Msk              (0x1UL << PTM_OCR2_OFULL_Pos)
#define PTM_OCR2_OFULL                  PTM_OCR2_OFULL_Msk

/********************* Bit definition for PTM_X2 register *********************/
#define PTM_X2_VAL_Pos                  (0U)
#define PTM_X2_VAL_Msk                  (0xFFFFFFFFUL << PTM_X2_VAL_Pos)
#define PTM_X2_VAL                      PTM_X2_VAL_Msk

/********************* Bit definition for PTM_Y2 register *********************/
#define PTM_Y2_VAL_Pos                  (0U)
#define PTM_Y2_VAL_Msk                  (0xFFFFFFFFUL << PTM_Y2_VAL_Pos)
#define PTM_Y2_VAL                      PTM_Y2_VAL_Msk

/********************* Bit definition for PTM_A2 register *********************/
#define PTM_A2_VAL_Pos                  (0U)
#define PTM_A2_VAL_Msk                  (0xFFFFUL << PTM_A2_VAL_Pos)
#define PTM_A2_VAL                      PTM_A2_VAL_Msk

/********************* Bit definition for PTM_B2 register *********************/
#define PTM_B2_VAL_Pos                  (0U)
#define PTM_B2_VAL_Msk                  (0xFFFFUL << PTM_B2_VAL_Pos)
#define PTM_B2_VAL                      PTM_B2_VAL_Msk

/******************** Bit definition for PTM_PC2 register *********************/
#define PTM_PC2_VAL_Pos                 (0U)
#define PTM_PC2_VAL_Msk                 (0xFFFFFFFFUL << PTM_PC2_VAL_Pos)
#define PTM_PC2_VAL                     PTM_PC2_VAL_Msk

/******************* Bit definition for PTM_IFIFO2 register *******************/
#define PTM_IFIFO2_DAT_Pos              (0U)
#define PTM_IFIFO2_DAT_Msk              (0xFFFFFFFFUL << PTM_IFIFO2_DAT_Pos)
#define PTM_IFIFO2_DAT                  PTM_IFIFO2_DAT_Msk

/******************* Bit definition for PTM_OFIFO2 register *******************/
#define PTM_OFIFO2_DAT_Pos              (0U)
#define PTM_OFIFO2_DAT_Msk              (0xFFFFFFFFUL << PTM_OFIFO2_DAT_Pos)
#define PTM_OFIFO2_DAT                  PTM_OFIFO2_DAT_Msk

/******************** Bit definition for PTM_CCR3 register ********************/
#define PTM_CCR3_PSC_Pos                (0U)
#define PTM_CCR3_PSC_Msk                (0xFFUL << PTM_CCR3_PSC_Pos)
#define PTM_CCR3_PSC                    PTM_CCR3_PSC_Msk
#define PTM_CCR3_REP_Pos                (8U)
#define PTM_CCR3_REP_Msk                (0xFFUL << PTM_CCR3_REP_Pos)
#define PTM_CCR3_REP                    PTM_CCR3_REP_Msk
#define PTM_CCR3_END0_Pos               (16U)
#define PTM_CCR3_END0_Msk               (0xFFUL << PTM_CCR3_END0_Pos)
#define PTM_CCR3_END0                   PTM_CCR3_END0_Msk
#define PTM_CCR3_END1_Pos               (24U)
#define PTM_CCR3_END1_Msk               (0xFFUL << PTM_CCR3_END1_Pos)
#define PTM_CCR3_END1                   PTM_CCR3_END1_Msk

/******************** Bit definition for PTM_ICR3 register ********************/
#define PTM_ICR3_IFIFO_Pos              (0U)
#define PTM_ICR3_IFIFO_Msk              (0x1UL << PTM_ICR3_IFIFO_Pos)
#define PTM_ICR3_IFIFO                  PTM_ICR3_IFIFO_Msk
#define PTM_ICR3_IAUTO_Pos              (1U)
#define PTM_ICR3_IAUTO_Msk              (0x1UL << PTM_ICR3_IAUTO_Pos)
#define PTM_ICR3_IAUTO                  PTM_ICR3_IAUTO_Msk
#define PTM_ICR3_IRSH_Pos               (2U)
#define PTM_ICR3_IRSH_Msk               (0x1UL << PTM_ICR3_IRSH_Pos)
#define PTM_ICR3_IRSH                   PTM_ICR3_IRSH_Msk
#define PTM_ICR3_IBLOCK_Pos             (3U)
#define PTM_ICR3_IBLOCK_Msk             (0x1UL << PTM_ICR3_IBLOCK_Pos)
#define PTM_ICR3_IBLOCK                 PTM_ICR3_IBLOCK_Msk
#define PTM_ICR3_IBASE_Pos              (8U)
#define PTM_ICR3_IBASE_Msk              (0xFUL << PTM_ICR3_IBASE_Pos)
#define PTM_ICR3_IBASE                  PTM_ICR3_IBASE_Msk
#define PTM_ICR3_ISHIFT_Pos             (16U)
#define PTM_ICR3_ISHIFT_Msk             (0x1FUL << PTM_ICR3_ISHIFT_Pos)
#define PTM_ICR3_ISHIFT                 PTM_ICR3_ISHIFT_Msk
#define PTM_ICR3_ILIMIT_Pos             (24U)
#define PTM_ICR3_ILIMIT_Msk             (0x1FUL << PTM_ICR3_ILIMIT_Pos)
#define PTM_ICR3_ILIMIT                 PTM_ICR3_ILIMIT_Msk
#define PTM_ICR3_IOF_Pos                (29U)
#define PTM_ICR3_IOF_Msk                (0x1UL << PTM_ICR3_IOF_Pos)
#define PTM_ICR3_IOF                    PTM_ICR3_IOF_Msk
#define PTM_ICR3_IEMPTY_Pos             (30U)
#define PTM_ICR3_IEMPTY_Msk             (0x1UL << PTM_ICR3_IEMPTY_Pos)
#define PTM_ICR3_IEMPTY                 PTM_ICR3_IEMPTY_Msk
#define PTM_ICR3_IFULL_Pos              (31U)
#define PTM_ICR3_IFULL_Msk              (0x1UL << PTM_ICR3_IFULL_Pos)
#define PTM_ICR3_IFULL                  PTM_ICR3_IFULL_Msk

/******************** Bit definition for PTM_OCR3 register ********************/
#define PTM_OCR3_OFIFO_Pos              (0U)
#define PTM_OCR3_OFIFO_Msk              (0x1UL << PTM_OCR3_OFIFO_Pos)
#define PTM_OCR3_OFIFO                  PTM_OCR3_OFIFO_Msk
#define PTM_OCR3_OAUTO_Pos              (1U)
#define PTM_OCR3_OAUTO_Msk              (0x1UL << PTM_OCR3_OAUTO_Pos)
#define PTM_OCR3_OAUTO                  PTM_OCR3_OAUTO_Msk
#define PTM_OCR3_ORSH_Pos               (2U)
#define PTM_OCR3_ORSH_Msk               (0x1UL << PTM_OCR3_ORSH_Pos)
#define PTM_OCR3_ORSH                   PTM_OCR3_ORSH_Msk
#define PTM_OCR3_OBLOCK_Pos             (3U)
#define PTM_OCR3_OBLOCK_Msk             (0x1UL << PTM_OCR3_OBLOCK_Pos)
#define PTM_OCR3_OBLOCK                 PTM_OCR3_OBLOCK_Msk
#define PTM_OCR3_OBASE_Pos              (8U)
#define PTM_OCR3_OBASE_Msk              (0xFUL << PTM_OCR3_OBASE_Pos)
#define PTM_OCR3_OBASE                  PTM_OCR3_OBASE_Msk
#define PTM_OCR3_OSHIFT_Pos             (16U)
#define PTM_OCR3_OSHIFT_Msk             (0x1FUL << PTM_OCR3_OSHIFT_Pos)
#define PTM_OCR3_OSHIFT                 PTM_OCR3_OSHIFT_Msk
#define PTM_OCR3_OLIMIT_Pos             (24U)
#define PTM_OCR3_OLIMIT_Msk             (0x1FUL << PTM_OCR3_OLIMIT_Pos)
#define PTM_OCR3_OLIMIT                 PTM_OCR3_OLIMIT_Msk
#define PTM_OCR3_OUF_Pos                (29U)
#define PTM_OCR3_OUF_Msk                (0x1UL << PTM_OCR3_OUF_Pos)
#define PTM_OCR3_OUF                    PTM_OCR3_OUF_Msk
#define PTM_OCR3_OEMPTY_Pos             (30U)
#define PTM_OCR3_OEMPTY_Msk             (0x1UL << PTM_OCR3_OEMPTY_Pos)
#define PTM_OCR3_OEMPTY                 PTM_OCR3_OEMPTY_Msk
#define PTM_OCR3_OFULL_Pos              (31U)
#define PTM_OCR3_OFULL_Msk              (0x1UL << PTM_OCR3_OFULL_Pos)
#define PTM_OCR3_OFULL                  PTM_OCR3_OFULL_Msk

/********************* Bit definition for PTM_X3 register *********************/
#define PTM_X3_VAL_Pos                  (0U)
#define PTM_X3_VAL_Msk                  (0xFFFFFFFFUL << PTM_X3_VAL_Pos)
#define PTM_X3_VAL                      PTM_X3_VAL_Msk

/********************* Bit definition for PTM_Y3 register *********************/
#define PTM_Y3_VAL_Pos                  (0U)
#define PTM_Y3_VAL_Msk                  (0xFFFFFFFFUL << PTM_Y3_VAL_Pos)
#define PTM_Y3_VAL                      PTM_Y3_VAL_Msk

/********************* Bit definition for PTM_A3 register *********************/
#define PTM_A3_VAL_Pos                  (0U)
#define PTM_A3_VAL_Msk                  (0xFFFFUL << PTM_A3_VAL_Pos)
#define PTM_A3_VAL                      PTM_A3_VAL_Msk

/********************* Bit definition for PTM_B3 register *********************/
#define PTM_B3_VAL_Pos                  (0U)
#define PTM_B3_VAL_Msk                  (0xFFFFUL << PTM_B3_VAL_Pos)
#define PTM_B3_VAL                      PTM_B3_VAL_Msk

/******************** Bit definition for PTM_PC3 register *********************/
#define PTM_PC3_VAL_Pos                 (0U)
#define PTM_PC3_VAL_Msk                 (0xFFFFFFFFUL << PTM_PC3_VAL_Pos)
#define PTM_PC3_VAL                     PTM_PC3_VAL_Msk

/******************* Bit definition for PTM_IFIFO3 register *******************/
#define PTM_IFIFO3_DAT_Pos              (0U)
#define PTM_IFIFO3_DAT_Msk              (0xFFFFFFFFUL << PTM_IFIFO3_DAT_Pos)
#define PTM_IFIFO3_DAT                  PTM_IFIFO3_DAT_Msk

/******************* Bit definition for PTM_OFIFO3 register *******************/
#define PTM_OFIFO3_DAT_Pos              (0U)
#define PTM_OFIFO3_DAT_Msk              (0xFFFFFFFFUL << PTM_OFIFO3_DAT_Pos)
#define PTM_OFIFO3_DAT                  PTM_OFIFO3_DAT_Msk

/******************** Bit definition for PTM_CCR4 register ********************/
#define PTM_CCR4_PSC_Pos                (0U)
#define PTM_CCR4_PSC_Msk                (0xFFUL << PTM_CCR4_PSC_Pos)
#define PTM_CCR4_PSC                    PTM_CCR4_PSC_Msk
#define PTM_CCR4_REP_Pos                (8U)
#define PTM_CCR4_REP_Msk                (0xFFUL << PTM_CCR4_REP_Pos)
#define PTM_CCR4_REP                    PTM_CCR4_REP_Msk
#define PTM_CCR4_END0_Pos               (16U)
#define PTM_CCR4_END0_Msk               (0xFFUL << PTM_CCR4_END0_Pos)
#define PTM_CCR4_END0                   PTM_CCR4_END0_Msk
#define PTM_CCR4_END1_Pos               (24U)
#define PTM_CCR4_END1_Msk               (0xFFUL << PTM_CCR4_END1_Pos)
#define PTM_CCR4_END1                   PTM_CCR4_END1_Msk

/******************** Bit definition for PTM_ICR4 register ********************/
#define PTM_ICR4_IFIFO_Pos              (0U)
#define PTM_ICR4_IFIFO_Msk              (0x1UL << PTM_ICR4_IFIFO_Pos)
#define PTM_ICR4_IFIFO                  PTM_ICR4_IFIFO_Msk
#define PTM_ICR4_IAUTO_Pos              (1U)
#define PTM_ICR4_IAUTO_Msk              (0x1UL << PTM_ICR4_IAUTO_Pos)
#define PTM_ICR4_IAUTO                  PTM_ICR4_IAUTO_Msk
#define PTM_ICR4_IRSH_Pos               (2U)
#define PTM_ICR4_IRSH_Msk               (0x1UL << PTM_ICR4_IRSH_Pos)
#define PTM_ICR4_IRSH                   PTM_ICR4_IRSH_Msk
#define PTM_ICR4_IBLOCK_Pos             (3U)
#define PTM_ICR4_IBLOCK_Msk             (0x1UL << PTM_ICR4_IBLOCK_Pos)
#define PTM_ICR4_IBLOCK                 PTM_ICR4_IBLOCK_Msk
#define PTM_ICR4_IBASE_Pos              (8U)
#define PTM_ICR4_IBASE_Msk              (0xFUL << PTM_ICR4_IBASE_Pos)
#define PTM_ICR4_IBASE                  PTM_ICR4_IBASE_Msk
#define PTM_ICR4_ISHIFT_Pos             (16U)
#define PTM_ICR4_ISHIFT_Msk             (0x1FUL << PTM_ICR4_ISHIFT_Pos)
#define PTM_ICR4_ISHIFT                 PTM_ICR4_ISHIFT_Msk
#define PTM_ICR4_ILIMIT_Pos             (24U)
#define PTM_ICR4_ILIMIT_Msk             (0x1FUL << PTM_ICR4_ILIMIT_Pos)
#define PTM_ICR4_ILIMIT                 PTM_ICR4_ILIMIT_Msk
#define PTM_ICR4_IOF_Pos                (29U)
#define PTM_ICR4_IOF_Msk                (0x1UL << PTM_ICR4_IOF_Pos)
#define PTM_ICR4_IOF                    PTM_ICR4_IOF_Msk
#define PTM_ICR4_IEMPTY_Pos             (30U)
#define PTM_ICR4_IEMPTY_Msk             (0x1UL << PTM_ICR4_IEMPTY_Pos)
#define PTM_ICR4_IEMPTY                 PTM_ICR4_IEMPTY_Msk
#define PTM_ICR4_IFULL_Pos              (31U)
#define PTM_ICR4_IFULL_Msk              (0x1UL << PTM_ICR4_IFULL_Pos)
#define PTM_ICR4_IFULL                  PTM_ICR4_IFULL_Msk

/******************** Bit definition for PTM_OCR4 register ********************/
#define PTM_OCR4_OFIFO_Pos              (0U)
#define PTM_OCR4_OFIFO_Msk              (0x1UL << PTM_OCR4_OFIFO_Pos)
#define PTM_OCR4_OFIFO                  PTM_OCR4_OFIFO_Msk
#define PTM_OCR4_OAUTO_Pos              (1U)
#define PTM_OCR4_OAUTO_Msk              (0x1UL << PTM_OCR4_OAUTO_Pos)
#define PTM_OCR4_OAUTO                  PTM_OCR4_OAUTO_Msk
#define PTM_OCR4_ORSH_Pos               (2U)
#define PTM_OCR4_ORSH_Msk               (0x1UL << PTM_OCR4_ORSH_Pos)
#define PTM_OCR4_ORSH                   PTM_OCR4_ORSH_Msk
#define PTM_OCR4_OBLOCK_Pos             (3U)
#define PTM_OCR4_OBLOCK_Msk             (0x1UL << PTM_OCR4_OBLOCK_Pos)
#define PTM_OCR4_OBLOCK                 PTM_OCR4_OBLOCK_Msk
#define PTM_OCR4_OBASE_Pos              (8U)
#define PTM_OCR4_OBASE_Msk              (0xFUL << PTM_OCR4_OBASE_Pos)
#define PTM_OCR4_OBASE                  PTM_OCR4_OBASE_Msk
#define PTM_OCR4_OSHIFT_Pos             (16U)
#define PTM_OCR4_OSHIFT_Msk             (0x1FUL << PTM_OCR4_OSHIFT_Pos)
#define PTM_OCR4_OSHIFT                 PTM_OCR4_OSHIFT_Msk
#define PTM_OCR4_OLIMIT_Pos             (24U)
#define PTM_OCR4_OLIMIT_Msk             (0x1FUL << PTM_OCR4_OLIMIT_Pos)
#define PTM_OCR4_OLIMIT                 PTM_OCR4_OLIMIT_Msk
#define PTM_OCR4_OUF_Pos                (29U)
#define PTM_OCR4_OUF_Msk                (0x1UL << PTM_OCR4_OUF_Pos)
#define PTM_OCR4_OUF                    PTM_OCR4_OUF_Msk
#define PTM_OCR4_OEMPTY_Pos             (30U)
#define PTM_OCR4_OEMPTY_Msk             (0x1UL << PTM_OCR4_OEMPTY_Pos)
#define PTM_OCR4_OEMPTY                 PTM_OCR4_OEMPTY_Msk
#define PTM_OCR4_OFULL_Pos              (31U)
#define PTM_OCR4_OFULL_Msk              (0x1UL << PTM_OCR4_OFULL_Pos)
#define PTM_OCR4_OFULL                  PTM_OCR4_OFULL_Msk

/********************* Bit definition for PTM_X4 register *********************/
#define PTM_X4_VAL_Pos                  (0U)
#define PTM_X4_VAL_Msk                  (0xFFFFFFFFUL << PTM_X4_VAL_Pos)
#define PTM_X4_VAL                      PTM_X4_VAL_Msk

/********************* Bit definition for PTM_Y4 register *********************/
#define PTM_Y4_VAL_Pos                  (0U)
#define PTM_Y4_VAL_Msk                  (0xFFFFFFFFUL << PTM_Y4_VAL_Pos)
#define PTM_Y4_VAL                      PTM_Y4_VAL_Msk

/********************* Bit definition for PTM_A4 register *********************/
#define PTM_A4_VAL_Pos                  (0U)
#define PTM_A4_VAL_Msk                  (0xFFFFUL << PTM_A4_VAL_Pos)
#define PTM_A4_VAL                      PTM_A4_VAL_Msk

/********************* Bit definition for PTM_B4 register *********************/
#define PTM_B4_VAL_Pos                  (0U)
#define PTM_B4_VAL_Msk                  (0xFFFFUL << PTM_B4_VAL_Pos)
#define PTM_B4_VAL                      PTM_B4_VAL_Msk

/******************** Bit definition for PTM_PC4 register *********************/
#define PTM_PC4_VAL_Pos                 (0U)
#define PTM_PC4_VAL_Msk                 (0xFFFFFFFFUL << PTM_PC4_VAL_Pos)
#define PTM_PC4_VAL                     PTM_PC4_VAL_Msk

/******************* Bit definition for PTM_IFIFO4 register *******************/
#define PTM_IFIFO4_DAT_Pos              (0U)
#define PTM_IFIFO4_DAT_Msk              (0xFFFFFFFFUL << PTM_IFIFO4_DAT_Pos)
#define PTM_IFIFO4_DAT                  PTM_IFIFO4_DAT_Msk

/******************* Bit definition for PTM_OFIFO4 register *******************/
#define PTM_OFIFO4_DAT_Pos              (0U)
#define PTM_OFIFO4_DAT_Msk              (0xFFFFFFFFUL << PTM_OFIFO4_DAT_Pos)
#define PTM_OFIFO4_DAT                  PTM_OFIFO4_DAT_Msk

/******************** Bit definition for PTM_CCR5 register ********************/
#define PTM_CCR5_PSC_Pos                (0U)
#define PTM_CCR5_PSC_Msk                (0xFFUL << PTM_CCR5_PSC_Pos)
#define PTM_CCR5_PSC                    PTM_CCR5_PSC_Msk
#define PTM_CCR5_REP_Pos                (8U)
#define PTM_CCR5_REP_Msk                (0xFFUL << PTM_CCR5_REP_Pos)
#define PTM_CCR5_REP                    PTM_CCR5_REP_Msk
#define PTM_CCR5_END0_Pos               (16U)
#define PTM_CCR5_END0_Msk               (0xFFUL << PTM_CCR5_END0_Pos)
#define PTM_CCR5_END0                   PTM_CCR5_END0_Msk
#define PTM_CCR5_END1_Pos               (24U)
#define PTM_CCR5_END1_Msk               (0xFFUL << PTM_CCR5_END1_Pos)
#define PTM_CCR5_END1                   PTM_CCR5_END1_Msk

/******************** Bit definition for PTM_ICR5 register ********************/
#define PTM_ICR5_IFIFO_Pos              (0U)
#define PTM_ICR5_IFIFO_Msk              (0x1UL << PTM_ICR5_IFIFO_Pos)
#define PTM_ICR5_IFIFO                  PTM_ICR5_IFIFO_Msk
#define PTM_ICR5_IAUTO_Pos              (1U)
#define PTM_ICR5_IAUTO_Msk              (0x1UL << PTM_ICR5_IAUTO_Pos)
#define PTM_ICR5_IAUTO                  PTM_ICR5_IAUTO_Msk
#define PTM_ICR5_IRSH_Pos               (2U)
#define PTM_ICR5_IRSH_Msk               (0x1UL << PTM_ICR5_IRSH_Pos)
#define PTM_ICR5_IRSH                   PTM_ICR5_IRSH_Msk
#define PTM_ICR5_IBLOCK_Pos             (3U)
#define PTM_ICR5_IBLOCK_Msk             (0x1UL << PTM_ICR5_IBLOCK_Pos)
#define PTM_ICR5_IBLOCK                 PTM_ICR5_IBLOCK_Msk
#define PTM_ICR5_IBASE_Pos              (8U)
#define PTM_ICR5_IBASE_Msk              (0xFUL << PTM_ICR5_IBASE_Pos)
#define PTM_ICR5_IBASE                  PTM_ICR5_IBASE_Msk
#define PTM_ICR5_ISHIFT_Pos             (16U)
#define PTM_ICR5_ISHIFT_Msk             (0x1FUL << PTM_ICR5_ISHIFT_Pos)
#define PTM_ICR5_ISHIFT                 PTM_ICR5_ISHIFT_Msk
#define PTM_ICR5_ILIMIT_Pos             (24U)
#define PTM_ICR5_ILIMIT_Msk             (0x1FUL << PTM_ICR5_ILIMIT_Pos)
#define PTM_ICR5_ILIMIT                 PTM_ICR5_ILIMIT_Msk
#define PTM_ICR5_IOF_Pos                (29U)
#define PTM_ICR5_IOF_Msk                (0x1UL << PTM_ICR5_IOF_Pos)
#define PTM_ICR5_IOF                    PTM_ICR5_IOF_Msk
#define PTM_ICR5_IEMPTY_Pos             (30U)
#define PTM_ICR5_IEMPTY_Msk             (0x1UL << PTM_ICR5_IEMPTY_Pos)
#define PTM_ICR5_IEMPTY                 PTM_ICR5_IEMPTY_Msk
#define PTM_ICR5_IFULL_Pos              (31U)
#define PTM_ICR5_IFULL_Msk              (0x1UL << PTM_ICR5_IFULL_Pos)
#define PTM_ICR5_IFULL                  PTM_ICR5_IFULL_Msk

/******************** Bit definition for PTM_OCR5 register ********************/
#define PTM_OCR5_OFIFO_Pos              (0U)
#define PTM_OCR5_OFIFO_Msk              (0x1UL << PTM_OCR5_OFIFO_Pos)
#define PTM_OCR5_OFIFO                  PTM_OCR5_OFIFO_Msk
#define PTM_OCR5_OAUTO_Pos              (1U)
#define PTM_OCR5_OAUTO_Msk              (0x1UL << PTM_OCR5_OAUTO_Pos)
#define PTM_OCR5_OAUTO                  PTM_OCR5_OAUTO_Msk
#define PTM_OCR5_ORSH_Pos               (2U)
#define PTM_OCR5_ORSH_Msk               (0x1UL << PTM_OCR5_ORSH_Pos)
#define PTM_OCR5_ORSH                   PTM_OCR5_ORSH_Msk
#define PTM_OCR5_OBLOCK_Pos             (3U)
#define PTM_OCR5_OBLOCK_Msk             (0x1UL << PTM_OCR5_OBLOCK_Pos)
#define PTM_OCR5_OBLOCK                 PTM_OCR5_OBLOCK_Msk
#define PTM_OCR5_OBASE_Pos              (8U)
#define PTM_OCR5_OBASE_Msk              (0xFUL << PTM_OCR5_OBASE_Pos)
#define PTM_OCR5_OBASE                  PTM_OCR5_OBASE_Msk
#define PTM_OCR5_OSHIFT_Pos             (16U)
#define PTM_OCR5_OSHIFT_Msk             (0x1FUL << PTM_OCR5_OSHIFT_Pos)
#define PTM_OCR5_OSHIFT                 PTM_OCR5_OSHIFT_Msk
#define PTM_OCR5_OLIMIT_Pos             (24U)
#define PTM_OCR5_OLIMIT_Msk             (0x1FUL << PTM_OCR5_OLIMIT_Pos)
#define PTM_OCR5_OLIMIT                 PTM_OCR5_OLIMIT_Msk
#define PTM_OCR5_OUF_Pos                (29U)
#define PTM_OCR5_OUF_Msk                (0x1UL << PTM_OCR5_OUF_Pos)
#define PTM_OCR5_OUF                    PTM_OCR5_OUF_Msk
#define PTM_OCR5_OEMPTY_Pos             (30U)
#define PTM_OCR5_OEMPTY_Msk             (0x1UL << PTM_OCR5_OEMPTY_Pos)
#define PTM_OCR5_OEMPTY                 PTM_OCR5_OEMPTY_Msk
#define PTM_OCR5_OFULL_Pos              (31U)
#define PTM_OCR5_OFULL_Msk              (0x1UL << PTM_OCR5_OFULL_Pos)
#define PTM_OCR5_OFULL                  PTM_OCR5_OFULL_Msk

/********************* Bit definition for PTM_X5 register *********************/
#define PTM_X5_VAL_Pos                  (0U)
#define PTM_X5_VAL_Msk                  (0xFFFFFFFFUL << PTM_X5_VAL_Pos)
#define PTM_X5_VAL                      PTM_X5_VAL_Msk

/********************* Bit definition for PTM_Y5 register *********************/
#define PTM_Y5_VAL_Pos                  (0U)
#define PTM_Y5_VAL_Msk                  (0xFFFFFFFFUL << PTM_Y5_VAL_Pos)
#define PTM_Y5_VAL                      PTM_Y5_VAL_Msk

/********************* Bit definition for PTM_A5 register *********************/
#define PTM_A5_VAL_Pos                  (0U)
#define PTM_A5_VAL_Msk                  (0xFFFFUL << PTM_A5_VAL_Pos)
#define PTM_A5_VAL                      PTM_A5_VAL_Msk

/********************* Bit definition for PTM_B5 register *********************/
#define PTM_B5_VAL_Pos                  (0U)
#define PTM_B5_VAL_Msk                  (0xFFFFUL << PTM_B5_VAL_Pos)
#define PTM_B5_VAL                      PTM_B5_VAL_Msk

/******************** Bit definition for PTM_PC5 register *********************/
#define PTM_PC5_VAL_Pos                 (0U)
#define PTM_PC5_VAL_Msk                 (0xFFFFFFFFUL << PTM_PC5_VAL_Pos)
#define PTM_PC5_VAL                     PTM_PC5_VAL_Msk

/******************* Bit definition for PTM_IFIFO5 register *******************/
#define PTM_IFIFO5_DAT_Pos              (0U)
#define PTM_IFIFO5_DAT_Msk              (0xFFFFFFFFUL << PTM_IFIFO5_DAT_Pos)
#define PTM_IFIFO5_DAT                  PTM_IFIFO5_DAT_Msk

/******************* Bit definition for PTM_OFIFO5 register *******************/
#define PTM_OFIFO5_DAT_Pos              (0U)
#define PTM_OFIFO5_DAT_Msk              (0xFFFFFFFFUL << PTM_OFIFO5_DAT_Pos)
#define PTM_OFIFO5_DAT                  PTM_OFIFO5_DAT_Msk

/******************** Bit definition for PTM_CCR6 register ********************/
#define PTM_CCR6_PSC_Pos                (0U)
#define PTM_CCR6_PSC_Msk                (0xFFUL << PTM_CCR6_PSC_Pos)
#define PTM_CCR6_PSC                    PTM_CCR6_PSC_Msk
#define PTM_CCR6_REP_Pos                (8U)
#define PTM_CCR6_REP_Msk                (0xFFUL << PTM_CCR6_REP_Pos)
#define PTM_CCR6_REP                    PTM_CCR6_REP_Msk
#define PTM_CCR6_END0_Pos               (16U)
#define PTM_CCR6_END0_Msk               (0xFFUL << PTM_CCR6_END0_Pos)
#define PTM_CCR6_END0                   PTM_CCR6_END0_Msk
#define PTM_CCR6_END1_Pos               (24U)
#define PTM_CCR6_END1_Msk               (0xFFUL << PTM_CCR6_END1_Pos)
#define PTM_CCR6_END1                   PTM_CCR6_END1_Msk

/******************** Bit definition for PTM_ICR6 register ********************/
#define PTM_ICR6_IFIFO_Pos              (0U)
#define PTM_ICR6_IFIFO_Msk              (0x1UL << PTM_ICR6_IFIFO_Pos)
#define PTM_ICR6_IFIFO                  PTM_ICR6_IFIFO_Msk
#define PTM_ICR6_IAUTO_Pos              (1U)
#define PTM_ICR6_IAUTO_Msk              (0x1UL << PTM_ICR6_IAUTO_Pos)
#define PTM_ICR6_IAUTO                  PTM_ICR6_IAUTO_Msk
#define PTM_ICR6_IRSH_Pos               (2U)
#define PTM_ICR6_IRSH_Msk               (0x1UL << PTM_ICR6_IRSH_Pos)
#define PTM_ICR6_IRSH                   PTM_ICR6_IRSH_Msk
#define PTM_ICR6_IBLOCK_Pos             (3U)
#define PTM_ICR6_IBLOCK_Msk             (0x1UL << PTM_ICR6_IBLOCK_Pos)
#define PTM_ICR6_IBLOCK                 PTM_ICR6_IBLOCK_Msk
#define PTM_ICR6_IBASE_Pos              (8U)
#define PTM_ICR6_IBASE_Msk              (0xFUL << PTM_ICR6_IBASE_Pos)
#define PTM_ICR6_IBASE                  PTM_ICR6_IBASE_Msk
#define PTM_ICR6_ISHIFT_Pos             (16U)
#define PTM_ICR6_ISHIFT_Msk             (0x1FUL << PTM_ICR6_ISHIFT_Pos)
#define PTM_ICR6_ISHIFT                 PTM_ICR6_ISHIFT_Msk
#define PTM_ICR6_ILIMIT_Pos             (24U)
#define PTM_ICR6_ILIMIT_Msk             (0x1FUL << PTM_ICR6_ILIMIT_Pos)
#define PTM_ICR6_ILIMIT                 PTM_ICR6_ILIMIT_Msk
#define PTM_ICR6_IOF_Pos                (29U)
#define PTM_ICR6_IOF_Msk                (0x1UL << PTM_ICR6_IOF_Pos)
#define PTM_ICR6_IOF                    PTM_ICR6_IOF_Msk
#define PTM_ICR6_IEMPTY_Pos             (30U)
#define PTM_ICR6_IEMPTY_Msk             (0x1UL << PTM_ICR6_IEMPTY_Pos)
#define PTM_ICR6_IEMPTY                 PTM_ICR6_IEMPTY_Msk
#define PTM_ICR6_IFULL_Pos              (31U)
#define PTM_ICR6_IFULL_Msk              (0x1UL << PTM_ICR6_IFULL_Pos)
#define PTM_ICR6_IFULL                  PTM_ICR6_IFULL_Msk

/******************** Bit definition for PTM_OCR6 register ********************/
#define PTM_OCR6_OFIFO_Pos              (0U)
#define PTM_OCR6_OFIFO_Msk              (0x1UL << PTM_OCR6_OFIFO_Pos)
#define PTM_OCR6_OFIFO                  PTM_OCR6_OFIFO_Msk
#define PTM_OCR6_OAUTO_Pos              (1U)
#define PTM_OCR6_OAUTO_Msk              (0x1UL << PTM_OCR6_OAUTO_Pos)
#define PTM_OCR6_OAUTO                  PTM_OCR6_OAUTO_Msk
#define PTM_OCR6_ORSH_Pos               (2U)
#define PTM_OCR6_ORSH_Msk               (0x1UL << PTM_OCR6_ORSH_Pos)
#define PTM_OCR6_ORSH                   PTM_OCR6_ORSH_Msk
#define PTM_OCR6_OBLOCK_Pos             (3U)
#define PTM_OCR6_OBLOCK_Msk             (0x1UL << PTM_OCR6_OBLOCK_Pos)
#define PTM_OCR6_OBLOCK                 PTM_OCR6_OBLOCK_Msk
#define PTM_OCR6_OBASE_Pos              (8U)
#define PTM_OCR6_OBASE_Msk              (0xFUL << PTM_OCR6_OBASE_Pos)
#define PTM_OCR6_OBASE                  PTM_OCR6_OBASE_Msk
#define PTM_OCR6_OSHIFT_Pos             (16U)
#define PTM_OCR6_OSHIFT_Msk             (0x1FUL << PTM_OCR6_OSHIFT_Pos)
#define PTM_OCR6_OSHIFT                 PTM_OCR6_OSHIFT_Msk
#define PTM_OCR6_OLIMIT_Pos             (24U)
#define PTM_OCR6_OLIMIT_Msk             (0x1FUL << PTM_OCR6_OLIMIT_Pos)
#define PTM_OCR6_OLIMIT                 PTM_OCR6_OLIMIT_Msk
#define PTM_OCR6_OUF_Pos                (29U)
#define PTM_OCR6_OUF_Msk                (0x1UL << PTM_OCR6_OUF_Pos)
#define PTM_OCR6_OUF                    PTM_OCR6_OUF_Msk
#define PTM_OCR6_OEMPTY_Pos             (30U)
#define PTM_OCR6_OEMPTY_Msk             (0x1UL << PTM_OCR6_OEMPTY_Pos)
#define PTM_OCR6_OEMPTY                 PTM_OCR6_OEMPTY_Msk
#define PTM_OCR6_OFULL_Pos              (31U)
#define PTM_OCR6_OFULL_Msk              (0x1UL << PTM_OCR6_OFULL_Pos)
#define PTM_OCR6_OFULL                  PTM_OCR6_OFULL_Msk

/********************* Bit definition for PTM_X6 register *********************/
#define PTM_X6_VAL_Pos                  (0U)
#define PTM_X6_VAL_Msk                  (0xFFFFFFFFUL << PTM_X6_VAL_Pos)
#define PTM_X6_VAL                      PTM_X6_VAL_Msk

/********************* Bit definition for PTM_Y6 register *********************/
#define PTM_Y6_VAL_Pos                  (0U)
#define PTM_Y6_VAL_Msk                  (0xFFFFFFFFUL << PTM_Y6_VAL_Pos)
#define PTM_Y6_VAL                      PTM_Y6_VAL_Msk

/********************* Bit definition for PTM_A6 register *********************/
#define PTM_A6_VAL_Pos                  (0U)
#define PTM_A6_VAL_Msk                  (0xFFFFUL << PTM_A6_VAL_Pos)
#define PTM_A6_VAL                      PTM_A6_VAL_Msk

/********************* Bit definition for PTM_B6 register *********************/
#define PTM_B6_VAL_Pos                  (0U)
#define PTM_B6_VAL_Msk                  (0xFFFFUL << PTM_B6_VAL_Pos)
#define PTM_B6_VAL                      PTM_B6_VAL_Msk

/******************** Bit definition for PTM_PC6 register *********************/
#define PTM_PC6_VAL_Pos                 (0U)
#define PTM_PC6_VAL_Msk                 (0xFFFFFFFFUL << PTM_PC6_VAL_Pos)
#define PTM_PC6_VAL                     PTM_PC6_VAL_Msk

/******************* Bit definition for PTM_IFIFO6 register *******************/
#define PTM_IFIFO6_DAT_Pos              (0U)
#define PTM_IFIFO6_DAT_Msk              (0xFFFFFFFFUL << PTM_IFIFO6_DAT_Pos)
#define PTM_IFIFO6_DAT                  PTM_IFIFO6_DAT_Msk

/******************* Bit definition for PTM_OFIFO6 register *******************/
#define PTM_OFIFO6_DAT_Pos              (0U)
#define PTM_OFIFO6_DAT_Msk              (0xFFFFFFFFUL << PTM_OFIFO6_DAT_Pos)
#define PTM_OFIFO6_DAT                  PTM_OFIFO6_DAT_Msk

/******************** Bit definition for PTM_CCR7 register ********************/
#define PTM_CCR7_PSC_Pos                (0U)
#define PTM_CCR7_PSC_Msk                (0xFFUL << PTM_CCR7_PSC_Pos)
#define PTM_CCR7_PSC                    PTM_CCR7_PSC_Msk
#define PTM_CCR7_REP_Pos                (8U)
#define PTM_CCR7_REP_Msk                (0xFFUL << PTM_CCR7_REP_Pos)
#define PTM_CCR7_REP                    PTM_CCR7_REP_Msk
#define PTM_CCR7_END0_Pos               (16U)
#define PTM_CCR7_END0_Msk               (0xFFUL << PTM_CCR7_END0_Pos)
#define PTM_CCR7_END0                   PTM_CCR7_END0_Msk
#define PTM_CCR7_END1_Pos               (24U)
#define PTM_CCR7_END1_Msk               (0xFFUL << PTM_CCR7_END1_Pos)
#define PTM_CCR7_END1                   PTM_CCR7_END1_Msk

/******************** Bit definition for PTM_ICR7 register ********************/
#define PTM_ICR7_IFIFO_Pos              (0U)
#define PTM_ICR7_IFIFO_Msk              (0x1UL << PTM_ICR7_IFIFO_Pos)
#define PTM_ICR7_IFIFO                  PTM_ICR7_IFIFO_Msk
#define PTM_ICR7_IAUTO_Pos              (1U)
#define PTM_ICR7_IAUTO_Msk              (0x1UL << PTM_ICR7_IAUTO_Pos)
#define PTM_ICR7_IAUTO                  PTM_ICR7_IAUTO_Msk
#define PTM_ICR7_IRSH_Pos               (2U)
#define PTM_ICR7_IRSH_Msk               (0x1UL << PTM_ICR7_IRSH_Pos)
#define PTM_ICR7_IRSH                   PTM_ICR7_IRSH_Msk
#define PTM_ICR7_IBLOCK_Pos             (3U)
#define PTM_ICR7_IBLOCK_Msk             (0x1UL << PTM_ICR7_IBLOCK_Pos)
#define PTM_ICR7_IBLOCK                 PTM_ICR7_IBLOCK_Msk
#define PTM_ICR7_IBASE_Pos              (8U)
#define PTM_ICR7_IBASE_Msk              (0xFUL << PTM_ICR7_IBASE_Pos)
#define PTM_ICR7_IBASE                  PTM_ICR7_IBASE_Msk
#define PTM_ICR7_ISHIFT_Pos             (16U)
#define PTM_ICR7_ISHIFT_Msk             (0x1FUL << PTM_ICR7_ISHIFT_Pos)
#define PTM_ICR7_ISHIFT                 PTM_ICR7_ISHIFT_Msk
#define PTM_ICR7_ILIMIT_Pos             (24U)
#define PTM_ICR7_ILIMIT_Msk             (0x1FUL << PTM_ICR7_ILIMIT_Pos)
#define PTM_ICR7_ILIMIT                 PTM_ICR7_ILIMIT_Msk
#define PTM_ICR7_IOF_Pos                (29U)
#define PTM_ICR7_IOF_Msk                (0x1UL << PTM_ICR7_IOF_Pos)
#define PTM_ICR7_IOF                    PTM_ICR7_IOF_Msk
#define PTM_ICR7_IEMPTY_Pos             (30U)
#define PTM_ICR7_IEMPTY_Msk             (0x1UL << PTM_ICR7_IEMPTY_Pos)
#define PTM_ICR7_IEMPTY                 PTM_ICR7_IEMPTY_Msk
#define PTM_ICR7_IFULL_Pos              (31U)
#define PTM_ICR7_IFULL_Msk              (0x1UL << PTM_ICR7_IFULL_Pos)
#define PTM_ICR7_IFULL                  PTM_ICR7_IFULL_Msk

/******************** Bit definition for PTM_OCR7 register ********************/
#define PTM_OCR7_OFIFO_Pos              (0U)
#define PTM_OCR7_OFIFO_Msk              (0x1UL << PTM_OCR7_OFIFO_Pos)
#define PTM_OCR7_OFIFO                  PTM_OCR7_OFIFO_Msk
#define PTM_OCR7_OAUTO_Pos              (1U)
#define PTM_OCR7_OAUTO_Msk              (0x1UL << PTM_OCR7_OAUTO_Pos)
#define PTM_OCR7_OAUTO                  PTM_OCR7_OAUTO_Msk
#define PTM_OCR7_ORSH_Pos               (2U)
#define PTM_OCR7_ORSH_Msk               (0x1UL << PTM_OCR7_ORSH_Pos)
#define PTM_OCR7_ORSH                   PTM_OCR7_ORSH_Msk
#define PTM_OCR7_OBLOCK_Pos             (3U)
#define PTM_OCR7_OBLOCK_Msk             (0x1UL << PTM_OCR7_OBLOCK_Pos)
#define PTM_OCR7_OBLOCK                 PTM_OCR7_OBLOCK_Msk
#define PTM_OCR7_OBASE_Pos              (8U)
#define PTM_OCR7_OBASE_Msk              (0xFUL << PTM_OCR7_OBASE_Pos)
#define PTM_OCR7_OBASE                  PTM_OCR7_OBASE_Msk
#define PTM_OCR7_OSHIFT_Pos             (16U)
#define PTM_OCR7_OSHIFT_Msk             (0x1FUL << PTM_OCR7_OSHIFT_Pos)
#define PTM_OCR7_OSHIFT                 PTM_OCR7_OSHIFT_Msk
#define PTM_OCR7_OLIMIT_Pos             (24U)
#define PTM_OCR7_OLIMIT_Msk             (0x1FUL << PTM_OCR7_OLIMIT_Pos)
#define PTM_OCR7_OLIMIT                 PTM_OCR7_OLIMIT_Msk
#define PTM_OCR7_OUF_Pos                (29U)
#define PTM_OCR7_OUF_Msk                (0x1UL << PTM_OCR7_OUF_Pos)
#define PTM_OCR7_OUF                    PTM_OCR7_OUF_Msk
#define PTM_OCR7_OEMPTY_Pos             (30U)
#define PTM_OCR7_OEMPTY_Msk             (0x1UL << PTM_OCR7_OEMPTY_Pos)
#define PTM_OCR7_OEMPTY                 PTM_OCR7_OEMPTY_Msk
#define PTM_OCR7_OFULL_Pos              (31U)
#define PTM_OCR7_OFULL_Msk              (0x1UL << PTM_OCR7_OFULL_Pos)
#define PTM_OCR7_OFULL                  PTM_OCR7_OFULL_Msk

/********************* Bit definition for PTM_X7 register *********************/
#define PTM_X7_VAL_Pos                  (0U)
#define PTM_X7_VAL_Msk                  (0xFFFFFFFFUL << PTM_X7_VAL_Pos)
#define PTM_X7_VAL                      PTM_X7_VAL_Msk

/********************* Bit definition for PTM_Y7 register *********************/
#define PTM_Y7_VAL_Pos                  (0U)
#define PTM_Y7_VAL_Msk                  (0xFFFFFFFFUL << PTM_Y7_VAL_Pos)
#define PTM_Y7_VAL                      PTM_Y7_VAL_Msk

/********************* Bit definition for PTM_A7 register *********************/
#define PTM_A7_VAL_Pos                  (0U)
#define PTM_A7_VAL_Msk                  (0xFFFFUL << PTM_A7_VAL_Pos)
#define PTM_A7_VAL                      PTM_A7_VAL_Msk

/********************* Bit definition for PTM_B7 register *********************/
#define PTM_B7_VAL_Pos                  (0U)
#define PTM_B7_VAL_Msk                  (0xFFFFUL << PTM_B7_VAL_Pos)
#define PTM_B7_VAL                      PTM_B7_VAL_Msk

/******************** Bit definition for PTM_PC7 register *********************/
#define PTM_PC7_VAL_Pos                 (0U)
#define PTM_PC7_VAL_Msk                 (0xFFFFFFFFUL << PTM_PC7_VAL_Pos)
#define PTM_PC7_VAL                     PTM_PC7_VAL_Msk

/******************* Bit definition for PTM_IFIFO7 register *******************/
#define PTM_IFIFO7_DAT_Pos              (0U)
#define PTM_IFIFO7_DAT_Msk              (0xFFFFFFFFUL << PTM_IFIFO7_DAT_Pos)
#define PTM_IFIFO7_DAT                  PTM_IFIFO7_DAT_Msk

/******************* Bit definition for PTM_OFIFO7 register *******************/
#define PTM_OFIFO7_DAT_Pos              (0U)
#define PTM_OFIFO7_DAT_Msk              (0xFFFFFFFFUL << PTM_OFIFO7_DAT_Pos)
#define PTM_OFIFO7_DAT                  PTM_OFIFO7_DAT_Msk


/******************* PTM Trigger **********************************************/
#define PTM_TRIG_RSVD_0                  (0UL)
#define PTM_TRIG_HCPU_SLEEPING           (1UL)
#define PTM_TRIG_GPTIM1_UPDATE           (2UL)
#define PTM_TRIG_GPTIM1_TRIG             (3UL)
#define PTM_TRIG_GPTIM1_CH1              (4UL)
#define PTM_TRIG_GPTIM1_CH2              (5UL)
#define PTM_TRIG_GPTIM1_CH3              (6UL)
#define PTM_TRIG_GPTIM1_CH4              (7UL)
#define PTM_TRIG_GPTIM2_UPDATE           (8UL)
#define PTM_TRIG_GPTIM2_TRIG             (9UL)
#define PTM_TRIG_GPTIM2_CH1              (10UL)
#define PTM_TRIG_GPTIM2_CH2              (11UL)
#define PTM_TRIG_RSVD_12                 (12UL)
#define PTM_TRIG_RSVD_13                 (13UL)
#define PTM_TRIG_BTIM1_UPDATE            (14UL)
#define PTM_TRIG_BTIM2_UPDATE            (15UL)
#define PTM_TRIG_ATIM1_UPDATE            (16UL)
#define PTM_TRIG_ATIM1_TRIG              (17UL)
#define PTM_TRIG_ATIM1_CH1               (18UL)
#define PTM_TRIG_ATIM1_CH2               (19UL)
#define PTM_TRIG_ATIM1_CH3               (20UL)
#define PTM_TRIG_ATIM1_CH4               (21UL)
#define PTM_TRIG_ATIM1_COM               (22UL)
#define PTM_TRIG_CAN1_BUSY               (23UL)
#define PTM_TRIG_DMAC1_DONE1             (24UL)
#define PTM_TRIG_DMAC1_DONE2             (25UL)
#define PTM_TRIG_DMAC1_DONE3             (26UL)
#define PTM_TRIG_DMAC1_DONE4             (27UL)
#define PTM_TRIG_DMAC1_DONE5             (28UL)
#define PTM_TRIG_DMAC1_DONE6             (29UL)
#define PTM_TRIG_DMAC1_DONE7             (30UL)
#define PTM_TRIG_DMAC1_DONE8             (31UL)
#define PTM_TRIG_DMAC1_HALF1             (32UL)
#define PTM_TRIG_DMAC1_HALF2             (33UL)
#define PTM_TRIG_DMAC1_HALF3             (34UL)
#define PTM_TRIG_DMAC1_HALF4             (35UL)
#define PTM_TRIG_DMAC1_HALF5             (36UL)
#define PTM_TRIG_DMAC1_HALF6             (37UL)
#define PTM_TRIG_DMAC1_HALF7             (38UL)
#define PTM_TRIG_DMAC1_HALF8             (39UL)
#define PTM_TRIG_DMAC1_LDONE1            (40UL)
#define PTM_TRIG_DMAC1_LDONE2            (41UL)
#define PTM_TRIG_DMAC1_LDONE3            (42UL)
#define PTM_TRIG_DMAC1_LDONE4            (43UL)
#define PTM_TRIG_DMAC1_LDONE5            (44UL)
#define PTM_TRIG_DMAC1_LDONE6            (45UL)
#define PTM_TRIG_DMAC1_LDONE7            (46UL)
#define PTM_TRIG_DMAC1_LDONE8            (47UL)
#define PTM_TRIG_DCMI_VSYNC              (48UL)
#define PTM_TRIG_DCMI_FRAMESYNC          (49UL)
#define PTM_TRIG_BT_RX_EN                (50UL)
#define PTM_TRIG_BT_TX_EN                (51UL)
#define PTM_TRIG_MAILBOX1_C1IRQ7         (52UL)
#define PTM_TRIG_MAILBOX1_C2IRQ7         (53UL)
#define PTM_TRIG_MAILBOX1_C3IRQ7         (54UL)
#define PTM_TRIG_MAILBOX1_C4IRQ7         (55UL)
#define PTM_TRIG_PWM1_UPDATE1            (56UL)
#define PTM_TRIG_PWM1_UPDATE2            (57UL)
#define PTM_TRIG_PWM1_UPDATE3            (58UL)
#define PTM_TRIG_PWM1_UPDATE4            (59UL)
#define PTM_TRIG_PWM1_UPDATE5            (60UL)
#define PTM_TRIG_ATIM2_UPDATE            (61UL)
#define PTM_TRIG_ATIM2_TRIG              (62UL)
#define PTM_TRIG_ATIM2_CH1               (63UL)
#define PTM_TRIG_BT_ACTIVE               (64UL)
#define PTM_TRIG_BT_PRIORITY             (65UL)
#define PTM_TRIG_BT_COLLISION            (66UL)
#define PTM_TRIG_WLAN_ACTIVE             (67UL)
#define PTM_TRIG_I2C1_ATDONE             (68UL)
#define PTM_TRIG_I2C1_DMADONE            (69UL)
#define PTM_TRIG_I2C1_TXEMPTY            (70UL)
#define PTM_TRIG_I2C1_RXFULL             (71UL)
#define PTM_TRIG_I2C2_ATDONE             (72UL)
#define PTM_TRIG_I2C2_DMADONE            (73UL)
#define PTM_TRIG_I2C2_TXEMPTY            (74UL)
#define PTM_TRIG_I2C2_RXFULL             (75UL)
#define PTM_TRIG_I2C3_ATDONE             (76UL)
#define PTM_TRIG_I2C3_DMADONE            (77UL)
#define PTM_TRIG_I2C3_TXEMPTY            (78UL)
#define PTM_TRIG_I2C3_RXFULL             (79UL)
#define PTM_TRIG_SPI1_DONE               (80UL)
#define PTM_TRIG_SPI1_START              (81UL)
#define PTM_TRIG_SPI2_DONE               (82UL)
#define PTM_TRIG_SPI2_START              (83UL)
#define PTM_TRIG_SDMMC1_CMDDONE          (84UL)
#define PTM_TRIG_SDMMC1_DATDONE          (85UL)
#define PTM_TRIG_SDMMC2_CMDDONE          (86UL)
#define PTM_TRIG_SDMMC2_DATDONE          (87UL)
#define PTM_TRIG_USART1_RXBYTE           (88UL)
#define PTM_TRIG_USART1_TXBYTE           (89UL)
#define PTM_TRIG_USART2_RXBYTE           (90UL)
#define PTM_TRIG_USART2_TXBYTE           (91UL)
#define PTM_TRIG_I2C4_ATDONE             (92UL)
#define PTM_TRIG_I2C4_DMADONE            (93UL)
#define PTM_TRIG_I2C4_TXEMPTY            (94UL)
#define PTM_TRIG_I2C4_RXFULL             (95UL)
#define PTM_TRIG_USB_TX                  (96UL)
#define PTM_TRIG_USB_RX                  (97UL)
#define PTM_TRIG_USART3_RXBYTE           (98UL)
#define PTM_TRIG_USART3_TXBYTE           (99UL)
#define PTM_TRIG_EZIP1_DONE              (100UL)
#define PTM_TRIG_EZIP1_ROW               (101UL)
#define PTM_TRIG_TSEN_DONE               (102UL)
#define PTM_TRIG_LCDC1_BUSY              (103UL)
#define PTM_TRIG_LCDC1_DONE              (104UL)
#define PTM_TRIG_LCDC1_FMARK             (105UL)
#define PTM_TRIG_LCDC1_LINEHIT           (106UL)
#define PTM_TRIG_LCDC1_LINE              (107UL)
#define PTM_TRIG_LCDC1_ERR               (108UL)
#define PTM_TRIG_JPEGD_INT               (109UL)
#define PTM_TRIG_EPIC_DONE               (110UL)
#define PTM_TRIG_EPIC_LINEHIT            (111UL)
#define PTM_TRIG_HCPU_SLEEPDEEP          (112UL)
#define PTM_TRIG_TRNG_SEEDGEN            (113UL)
#define PTM_TRIG_TRNG_RANDGEN            (114UL)
#define PTM_TRIG_AES_DONE                (115UL)
#define PTM_TRIG_I2S1_UF                 (116UL)
#define PTM_TRIG_I2S1_OF                 (117UL)
#define PTM_TRIG_I2S2_UF                 (118UL)
#define PTM_TRIG_I2S2_OF                 (119UL)
#define PTM_TRIG_PTM1_EVT0               (120UL)
#define PTM_TRIG_PTM1_EVT1               (121UL)
#define PTM_TRIG_PTM1_EVT2               (122UL)
#define PTM_TRIG_PTM1_EVT3               (123UL)
#define PTM_TRIG_PTM1_EVT4               (124UL)
#define PTM_TRIG_PTM1_EVT5               (125UL)
#define PTM_TRIG_PTM1_EVT6               (126UL)
#define PTM_TRIG_PTM1_EVT7               (127UL)

#endif
