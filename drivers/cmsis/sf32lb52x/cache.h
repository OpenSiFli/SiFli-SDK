/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __CACHE_H
#define __CACHE_H

typedef struct
{
    __IO uint32_t CCR;
    __IO uint32_t IMCR;
    __IO uint32_t IACR;
    __IO uint32_t ISR;
    __IO uint32_t DMCR;
    __IO uint32_t DACR;
    __IO uint32_t DSR;
} CACHE_TypeDef;


/******************* Bit definition for CACHE_CCR register ********************/
#define CACHE_CCR_CNTEN_Pos             (0U)
#define CACHE_CCR_CNTEN_Msk             (0x1UL << CACHE_CCR_CNTEN_Pos)
#define CACHE_CCR_CNTEN                 CACHE_CCR_CNTEN_Msk
#define CACHE_CCR_CNTCLR_Pos            (1U)
#define CACHE_CCR_CNTCLR_Msk            (0x1UL << CACHE_CCR_CNTCLR_Pos)
#define CACHE_CCR_CNTCLR                CACHE_CCR_CNTCLR_Msk
#define CACHE_CCR_IRANGE_Pos            (16U)
#define CACHE_CCR_IRANGE_Msk            (0x1FUL << CACHE_CCR_IRANGE_Pos)
#define CACHE_CCR_IRANGE                CACHE_CCR_IRANGE_Msk
#define CACHE_CCR_IRANGE_MPI1           (1UL << CACHE_CCR_IRANGE_Pos)
#define CACHE_CCR_IRANGE_MPI2           (2UL << CACHE_CCR_IRANGE_Pos)
#define CACHE_CCR_DRANGE_Pos            (24U)
#define CACHE_CCR_DRANGE_Msk            (0x1FUL << CACHE_CCR_DRANGE_Pos)
#define CACHE_CCR_DRANGE                CACHE_CCR_DRANGE_Msk
#define CACHE_CCR_DRANGE_MPI1           (1UL << CACHE_CCR_DRANGE_Pos)
#define CACHE_CCR_DRANGE_MPI2           (2UL << CACHE_CCR_DRANGE_Pos)

/******************* Bit definition for CACHE_IMCR register *******************/
#define CACHE_IMCR_IMISS_Pos            (0U)
#define CACHE_IMCR_IMISS_Msk            (0xFFFFFFFFUL << CACHE_IMCR_IMISS_Pos)
#define CACHE_IMCR_IMISS                CACHE_IMCR_IMISS_Msk

/******************* Bit definition for CACHE_IACR register *******************/
#define CACHE_IACR_IACCESS_Pos          (0U)
#define CACHE_IACR_IACCESS_Msk          (0xFFFFFFFFUL << CACHE_IACR_IACCESS_Pos)
#define CACHE_IACR_IACCESS              CACHE_IACR_IACCESS_Msk

/******************* Bit definition for CACHE_ISR register ********************/
#define CACHE_ISR_ISNAPSHOT_Pos         (0U)
#define CACHE_ISR_ISNAPSHOT_Msk         (0xFFFFFFFFUL << CACHE_ISR_ISNAPSHOT_Pos)
#define CACHE_ISR_ISNAPSHOT             CACHE_ISR_ISNAPSHOT_Msk

/******************* Bit definition for CACHE_DMCR register *******************/
#define CACHE_DMCR_DMISS_Pos            (0U)
#define CACHE_DMCR_DMISS_Msk            (0xFFFFFFFFUL << CACHE_DMCR_DMISS_Pos)
#define CACHE_DMCR_DMISS                CACHE_DMCR_DMISS_Msk

/******************* Bit definition for CACHE_DACR register *******************/
#define CACHE_DACR_DACCESS_Pos          (0U)
#define CACHE_DACR_DACCESS_Msk          (0xFFFFFFFFUL << CACHE_DACR_DACCESS_Pos)
#define CACHE_DACR_DACCESS              CACHE_DACR_DACCESS_Msk

/******************* Bit definition for CACHE_DSR register ********************/
#define CACHE_DSR_DSNAPSHOT_Pos         (0U)
#define CACHE_DSR_DSNAPSHOT_Msk         (0xFFFFFFFFUL << CACHE_DSR_DSNAPSHOT_Pos)
#define CACHE_DSR_DSNAPSHOT             CACHE_DSR_DSNAPSHOT_Msk

#endif