#ifndef __FACC_H
#define __FACC_H

typedef struct
{
    __IO uint32_t FACC_EN;
    __IO uint32_t FACC_PARA;
    __IO uint32_t SRC_ADDR;
    __IO uint32_t DST_ADDR;
    __IO uint32_t BLK_LEN;
    __IO uint32_t BUF_ADDR;
    __IO uint32_t DATA_SEQ;
    __IO uint32_t VECT_PARA;
    __IO uint32_t VECT_B;
    __IO uint32_t VECT_A;
    __IO uint32_t VECT_CLR;
    __IO uint32_t INT_EN;
    __IO uint32_t INT_STA;
    __IO uint32_t DBG_DATA;
} FACC_TypeDef;


/****************** Bit definition for FACC_FACC_EN register ******************/
#define FACC_FACC_EN_FACC_EN_Pos        (0U)
#define FACC_FACC_EN_FACC_EN_Msk        (0x1UL << FACC_FACC_EN_FACC_EN_Pos)
#define FACC_FACC_EN_FACC_EN            FACC_FACC_EN_FACC_EN_Msk

/***************** Bit definition for FACC_FACC_PARA register *****************/
#define FACC_FACC_PARA_GAIN_Pos         (0U)
#define FACC_FACC_PARA_GAIN_Msk         (0xFUL << FACC_FACC_PARA_GAIN_Pos)
#define FACC_FACC_PARA_GAIN             FACC_FACC_PARA_GAIN_Msk
#define FACC_FACC_PARA_LAST_SEL_Pos     (4U)
#define FACC_FACC_PARA_LAST_SEL_Msk     (0x1UL << FACC_FACC_PARA_LAST_SEL_Pos)
#define FACC_FACC_PARA_LAST_SEL         FACC_FACC_PARA_LAST_SEL_Msk
#define FACC_FACC_PARA_FIRST_SEL_Pos    (5U)
#define FACC_FACC_PARA_FIRST_SEL_Msk    (0x1UL << FACC_FACC_PARA_FIRST_SEL_Pos)
#define FACC_FACC_PARA_FIRST_SEL        FACC_FACC_PARA_FIRST_SEL_Msk
#define FACC_FACC_PARA_MOD_SEL_Pos      (6U)
#define FACC_FACC_PARA_MOD_SEL_Msk      (0x1UL << FACC_FACC_PARA_MOD_SEL_Pos)
#define FACC_FACC_PARA_MOD_SEL          FACC_FACC_PARA_MOD_SEL_Msk
#define FACC_FACC_PARA_SYM_SEL_Pos      (7U)
#define FACC_FACC_PARA_SYM_SEL_Msk      (0x1UL << FACC_FACC_PARA_SYM_SEL_Pos)
#define FACC_FACC_PARA_SYM_SEL          FACC_FACC_PARA_SYM_SEL_Msk
#define FACC_FACC_PARA_FP_SEL_Pos       (8U)
#define FACC_FACC_PARA_FP_SEL_Msk       (0x1UL << FACC_FACC_PARA_FP_SEL_Pos)
#define FACC_FACC_PARA_FP_SEL           FACC_FACC_PARA_FP_SEL_Msk
#define FACC_FACC_PARA_CONV_SEL_Pos     (9U)
#define FACC_FACC_PARA_CONV_SEL_Msk     (0x1UL << FACC_FACC_PARA_CONV_SEL_Pos)
#define FACC_FACC_PARA_CONV_SEL         FACC_FACC_PARA_CONV_SEL_Msk
#define FACC_FACC_PARA_EVEN_SEL_Pos     (10U)
#define FACC_FACC_PARA_EVEN_SEL_Msk     (0x1UL << FACC_FACC_PARA_EVEN_SEL_Pos)
#define FACC_FACC_PARA_EVEN_SEL         FACC_FACC_PARA_EVEN_SEL_Msk
#define FACC_FACC_PARA_BUF_SEL_Pos      (11U)
#define FACC_FACC_PARA_BUF_SEL_Msk      (0x1UL << FACC_FACC_PARA_BUF_SEL_Pos)
#define FACC_FACC_PARA_BUF_SEL          FACC_FACC_PARA_BUF_SEL_Msk

/***************** Bit definition for FACC_SRC_ADDR register ******************/
#define FACC_SRC_ADDR_SRC_ADDR_Pos      (0U)
#define FACC_SRC_ADDR_SRC_ADDR_Msk      (0xFFFFFFFFUL << FACC_SRC_ADDR_SRC_ADDR_Pos)
#define FACC_SRC_ADDR_SRC_ADDR          FACC_SRC_ADDR_SRC_ADDR_Msk

/***************** Bit definition for FACC_DST_ADDR register ******************/
#define FACC_DST_ADDR_DST_ADDR_Pos      (0U)
#define FACC_DST_ADDR_DST_ADDR_Msk      (0xFFFFFFFFUL << FACC_DST_ADDR_DST_ADDR_Pos)
#define FACC_DST_ADDR_DST_ADDR          FACC_DST_ADDR_DST_ADDR_Msk

/****************** Bit definition for FACC_BLK_LEN register ******************/
#define FACC_BLK_LEN_BLK_LEN_Pos        (0U)
#define FACC_BLK_LEN_BLK_LEN_Msk        (0xFFFFFFFFUL << FACC_BLK_LEN_BLK_LEN_Pos)
#define FACC_BLK_LEN_BLK_LEN            FACC_BLK_LEN_BLK_LEN_Msk

/***************** Bit definition for FACC_BUF_ADDR register ******************/
#define FACC_BUF_ADDR_BUF_ADDR_Pos      (0U)
#define FACC_BUF_ADDR_BUF_ADDR_Msk      (0xFFFFFFFFUL << FACC_BUF_ADDR_BUF_ADDR_Pos)
#define FACC_BUF_ADDR_BUF_ADDR          FACC_BUF_ADDR_BUF_ADDR_Msk

/***************** Bit definition for FACC_DATA_SEQ register ******************/
#define FACC_DATA_SEQ_DATA_SEQ_Pos      (0U)
#define FACC_DATA_SEQ_DATA_SEQ_Msk      (0xFFFFFFFFUL << FACC_DATA_SEQ_DATA_SEQ_Pos)
#define FACC_DATA_SEQ_DATA_SEQ          FACC_DATA_SEQ_DATA_SEQ_Msk

/***************** Bit definition for FACC_VECT_PARA register *****************/
#define FACC_VECT_PARA_P_Pos            (0U)
#define FACC_VECT_PARA_P_Msk            (0xFFUL << FACC_VECT_PARA_P_Pos)
#define FACC_VECT_PARA_P                FACC_VECT_PARA_P_Msk
#define FACC_VECT_PARA_M_Pos            (8U)
#define FACC_VECT_PARA_M_Msk            (0xFFUL << FACC_VECT_PARA_M_Pos)
#define FACC_VECT_PARA_M                FACC_VECT_PARA_M_Msk

/****************** Bit definition for FACC_VECT_B register *******************/
#define FACC_VECT_B_VECT_B_Pos          (0U)
#define FACC_VECT_B_VECT_B_Msk          (0xFFFFFFFFUL << FACC_VECT_B_VECT_B_Pos)
#define FACC_VECT_B_VECT_B              FACC_VECT_B_VECT_B_Msk

/****************** Bit definition for FACC_VECT_A register *******************/
#define FACC_VECT_A_VECT_A_Pos          (0U)
#define FACC_VECT_A_VECT_A_Msk          (0xFFFFFFFFUL << FACC_VECT_A_VECT_A_Pos)
#define FACC_VECT_A_VECT_A              FACC_VECT_A_VECT_A_Msk

/***************** Bit definition for FACC_VECT_CLR register ******************/
#define FACC_VECT_CLR_VECT_CLR_Pos      (0U)
#define FACC_VECT_CLR_VECT_CLR_Msk      (0x1UL << FACC_VECT_CLR_VECT_CLR_Pos)
#define FACC_VECT_CLR_VECT_CLR          FACC_VECT_CLR_VECT_CLR_Msk

/****************** Bit definition for FACC_INT_EN register *******************/
#define FACC_INT_EN_INT_EN_Pos          (0U)
#define FACC_INT_EN_INT_EN_Msk          (0x1UL << FACC_INT_EN_INT_EN_Pos)
#define FACC_INT_EN_INT_EN              FACC_INT_EN_INT_EN_Msk

/****************** Bit definition for FACC_INT_STA register ******************/
#define FACC_INT_STA_INT_STA_Pos        (0U)
#define FACC_INT_STA_INT_STA_Msk        (0x1UL << FACC_INT_STA_INT_STA_Pos)
#define FACC_INT_STA_INT_STA            FACC_INT_STA_INT_STA_Msk

/***************** Bit definition for FACC_DBG_DATA register ******************/
#define FACC_DBG_DATA_DEBUG_DATA_Pos    (0U)
#define FACC_DBG_DATA_DEBUG_DATA_Msk    (0xFFFFFFFFUL << FACC_DBG_DATA_DEBUG_DATA_Pos)
#define FACC_DBG_DATA_DEBUG_DATA        FACC_DBG_DATA_DEBUG_DATA_Msk

#endif
