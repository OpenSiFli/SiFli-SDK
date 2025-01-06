#ifndef __LPCOMP_H
#define __LPCOMP_H

typedef struct
{
    __IO uint32_t CR1;
    __IO uint32_t CR2;
} LPCOMP_TypeDef;


/******************* Bit definition for LPCOMP_CR1 register *******************/
#define LPCOMP_CR1_EN_Pos               (0U)
#define LPCOMP_CR1_EN_Msk               (0x1UL << LPCOMP_CR1_EN_Pos)
#define LPCOMP_CR1_EN                   LPCOMP_CR1_EN_Msk
#define LPCOMP_CR1_POL_Pos              (1U)
#define LPCOMP_CR1_POL_Msk              (0x1UL << LPCOMP_CR1_POL_Pos)
#define LPCOMP_CR1_POL                  LPCOMP_CR1_POL_Msk
#define LPCOMP_CR1_MODE_Pos             (2U)
#define LPCOMP_CR1_MODE_Msk             (0x3UL << LPCOMP_CR1_MODE_Pos)
#define LPCOMP_CR1_MODE                 LPCOMP_CR1_MODE_Msk
#define LPCOMP_CR1_VREFINT_Pos          (4U)
#define LPCOMP_CR1_VREFINT_Msk          (0x3UL << LPCOMP_CR1_VREFINT_Pos)
#define LPCOMP_CR1_VREFINT              LPCOMP_CR1_VREFINT_Msk
#define LPCOMP_CR1_INMSEL_Pos           (6U)
#define LPCOMP_CR1_INMSEL_Msk           (0x1UL << LPCOMP_CR1_INMSEL_Pos)
#define LPCOMP_CR1_INMSEL               LPCOMP_CR1_INMSEL_Msk
#define LPCOMP_CR1_INPSEL_Pos           (7U)
#define LPCOMP_CR1_INPSEL_Msk           (0x1UL << LPCOMP_CR1_INPSEL_Pos)
#define LPCOMP_CR1_INPSEL               LPCOMP_CR1_INPSEL_Msk
#define LPCOMP_CR1_HYST_Pos             (8U)
#define LPCOMP_CR1_HYST_Msk             (0x3UL << LPCOMP_CR1_HYST_Pos)
#define LPCOMP_CR1_HYST                 LPCOMP_CR1_HYST_Msk
#define LPCOMP_CR1_SENS_Pos             (12U)
#define LPCOMP_CR1_SENS_Msk             (0x7UL << LPCOMP_CR1_SENS_Pos)
#define LPCOMP_CR1_SENS                 LPCOMP_CR1_SENS_Msk
#define LPCOMP_CR1_IE_Pos               (15U)
#define LPCOMP_CR1_IE_Msk               (0x1UL << LPCOMP_CR1_IE_Pos)
#define LPCOMP_CR1_IE                   LPCOMP_CR1_IE_Msk
#define LPCOMP_CR1_BM_Pos               (16U)
#define LPCOMP_CR1_BM_Msk               (0x3UL << LPCOMP_CR1_BM_Pos)
#define LPCOMP_CR1_BM                   LPCOMP_CR1_BM_Msk
#define LPCOMP_CR1_ATTN_Pos             (18U)
#define LPCOMP_CR1_ATTN_Msk             (0x1UL << LPCOMP_CR1_ATTN_Pos)
#define LPCOMP_CR1_ATTN                 LPCOMP_CR1_ATTN_Msk
#define LPCOMP_CR1_STAT_Pos             (30U)
#define LPCOMP_CR1_STAT_Msk             (0x1UL << LPCOMP_CR1_STAT_Pos)
#define LPCOMP_CR1_STAT                 LPCOMP_CR1_STAT_Msk
#define LPCOMP_CR1_VAL_Pos              (31U)
#define LPCOMP_CR1_VAL_Msk              (0x1UL << LPCOMP_CR1_VAL_Pos)
#define LPCOMP_CR1_VAL                  LPCOMP_CR1_VAL_Msk

/******************* Bit definition for LPCOMP_CR2 register *******************/
#define LPCOMP_CR2_EN_Pos               (0U)
#define LPCOMP_CR2_EN_Msk               (0x1UL << LPCOMP_CR2_EN_Pos)
#define LPCOMP_CR2_EN                   LPCOMP_CR2_EN_Msk
#define LPCOMP_CR2_POL_Pos              (1U)
#define LPCOMP_CR2_POL_Msk              (0x1UL << LPCOMP_CR2_POL_Pos)
#define LPCOMP_CR2_POL                  LPCOMP_CR2_POL_Msk
#define LPCOMP_CR2_MODE_Pos             (2U)
#define LPCOMP_CR2_MODE_Msk             (0x3UL << LPCOMP_CR2_MODE_Pos)
#define LPCOMP_CR2_MODE                 LPCOMP_CR2_MODE_Msk
#define LPCOMP_CR2_VREFINT_Pos          (4U)
#define LPCOMP_CR2_VREFINT_Msk          (0x3UL << LPCOMP_CR2_VREFINT_Pos)
#define LPCOMP_CR2_VREFINT              LPCOMP_CR2_VREFINT_Msk
#define LPCOMP_CR2_INMSEL_Pos           (6U)
#define LPCOMP_CR2_INMSEL_Msk           (0x1UL << LPCOMP_CR2_INMSEL_Pos)
#define LPCOMP_CR2_INMSEL               LPCOMP_CR2_INMSEL_Msk
#define LPCOMP_CR2_INPSEL_Pos           (7U)
#define LPCOMP_CR2_INPSEL_Msk           (0x1UL << LPCOMP_CR2_INPSEL_Pos)
#define LPCOMP_CR2_INPSEL               LPCOMP_CR2_INPSEL_Msk
#define LPCOMP_CR2_HYST_Pos             (8U)
#define LPCOMP_CR2_HYST_Msk             (0x3UL << LPCOMP_CR2_HYST_Pos)
#define LPCOMP_CR2_HYST                 LPCOMP_CR2_HYST_Msk
#define LPCOMP_CR2_COMBO_Pos            (10U)
#define LPCOMP_CR2_COMBO_Msk            (0x1UL << LPCOMP_CR2_COMBO_Pos)
#define LPCOMP_CR2_COMBO                LPCOMP_CR2_COMBO_Msk
#define LPCOMP_CR2_SENS_Pos             (12U)
#define LPCOMP_CR2_SENS_Msk             (0x7UL << LPCOMP_CR2_SENS_Pos)
#define LPCOMP_CR2_SENS                 LPCOMP_CR2_SENS_Msk
#define LPCOMP_CR2_IE_Pos               (15U)
#define LPCOMP_CR2_IE_Msk               (0x1UL << LPCOMP_CR2_IE_Pos)
#define LPCOMP_CR2_IE                   LPCOMP_CR2_IE_Msk
#define LPCOMP_CR2_BM_Pos               (16U)
#define LPCOMP_CR2_BM_Msk               (0x3UL << LPCOMP_CR2_BM_Pos)
#define LPCOMP_CR2_BM                   LPCOMP_CR2_BM_Msk
#define LPCOMP_CR2_ATTN_Pos             (18U)
#define LPCOMP_CR2_ATTN_Msk             (0x1UL << LPCOMP_CR2_ATTN_Pos)
#define LPCOMP_CR2_ATTN                 LPCOMP_CR2_ATTN_Msk
#define LPCOMP_CR2_STAT_Pos             (30U)
#define LPCOMP_CR2_STAT_Msk             (0x1UL << LPCOMP_CR2_STAT_Pos)
#define LPCOMP_CR2_STAT                 LPCOMP_CR2_STAT_Msk
#define LPCOMP_CR2_VAL_Pos              (31U)
#define LPCOMP_CR2_VAL_Msk              (0x1UL << LPCOMP_CR2_VAL_Pos)
#define LPCOMP_CR2_VAL                  LPCOMP_CR2_VAL_Msk

#endif
