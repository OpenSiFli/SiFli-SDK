#ifndef __EZIP_H
#define __EZIP_H

typedef struct
{
    __IO uint32_t EZIP_CTRL;
    __IO uint32_t SRC_ADDR;
    __IO uint32_t DST_ADDR;
    __IO uint32_t EZIP_PARA;
    __IO uint32_t CACHE_CLR;
    __IO uint32_t START_POINT;
    __IO uint32_t END_POINT;
    __IO uint32_t ROW_SIGN;
    __IO uint32_t INT_EN;
    __IO uint32_t INT_STA;
    __IO uint32_t INT_MASK;
    __IO uint32_t NAP_PARA;
    __IO uint32_t SRC_LEN;
    __IO uint32_t AEZIP_CTRL;
    __IO uint32_t FRAME_START;
    __IO uint32_t PLAY_START;
    __IO uint32_t FRAME_NUM;
    __IO uint32_t PLAY_NUM;
    __IO uint32_t SEQ_NUM;
    __IO uint32_t FRAME_AREA;
    __IO uint32_t FRAME_OFFSET;
    __IO uint32_t FRAME_DELAY;
    __IO uint32_t FRAME_TYPE;
    __IO uint32_t FRAME_SIZE;
    __IO uint32_t GREY_PARA;
    __IO uint32_t DB_SEL;
    __IO uint32_t DB_DATA0;
    __IO uint32_t DB_DATA1;
    __IO uint32_t DB_DATA2;
    __IO uint32_t DB_DATA3;
    __IO uint32_t DB_DATA4;
    __IO uint32_t DB_DATA5;
    __IO uint32_t DB_DATA6;
    __IO uint32_t DB_DATA7;
    __IO uint32_t DB_DATA8;
    __IO uint32_t DB_DATA9;
    __IO uint32_t DB_DATA10;
    __IO uint32_t DB_DATA11;
    __IO uint32_t DB_DATA12;
    __IO uint32_t DB_DATA13;
} EZIP_TypeDef;


/***************** Bit definition for EZIP_EZIP_CTRL register *****************/
#define EZIP_EZIP_CTRL_EZIP_CTRL_Pos    (0U)
#define EZIP_EZIP_CTRL_EZIP_CTRL_Msk    (0x1UL << EZIP_EZIP_CTRL_EZIP_CTRL_Pos)
#define EZIP_EZIP_CTRL_EZIP_CTRL        EZIP_EZIP_CTRL_EZIP_CTRL_Msk

/***************** Bit definition for EZIP_SRC_ADDR register ******************/
#define EZIP_SRC_ADDR_SRC_ADDR_Pos      (0U)
#define EZIP_SRC_ADDR_SRC_ADDR_Msk      (0xFFFFFFFFUL << EZIP_SRC_ADDR_SRC_ADDR_Pos)
#define EZIP_SRC_ADDR_SRC_ADDR          EZIP_SRC_ADDR_SRC_ADDR_Msk

/***************** Bit definition for EZIP_DST_ADDR register ******************/
#define EZIP_DST_ADDR_DST_ADDR_Pos      (0U)
#define EZIP_DST_ADDR_DST_ADDR_Msk      (0xFFFFFFFFUL << EZIP_DST_ADDR_DST_ADDR_Pos)
#define EZIP_DST_ADDR_DST_ADDR          EZIP_DST_ADDR_DST_ADDR_Msk

/***************** Bit definition for EZIP_EZIP_PARA register *****************/
#define EZIP_EZIP_PARA_OUT_SEL_Pos      (0U)
#define EZIP_EZIP_PARA_OUT_SEL_Msk      (0x1UL << EZIP_EZIP_PARA_OUT_SEL_Pos)
#define EZIP_EZIP_PARA_OUT_SEL          EZIP_EZIP_PARA_OUT_SEL_Msk
#define EZIP_EZIP_PARA_MOD_SEL_Pos      (1U)
#define EZIP_EZIP_PARA_MOD_SEL_Msk      (0x3UL << EZIP_EZIP_PARA_MOD_SEL_Pos)
#define EZIP_EZIP_PARA_MOD_SEL          EZIP_EZIP_PARA_MOD_SEL_Msk
#define EZIP_EZIP_PARA_CACHE_EN_Pos     (3U)
#define EZIP_EZIP_PARA_CACHE_EN_Msk     (0x1UL << EZIP_EZIP_PARA_CACHE_EN_Pos)
#define EZIP_EZIP_PARA_CACHE_EN         EZIP_EZIP_PARA_CACHE_EN_Msk
#define EZIP_EZIP_PARA_IN_SEL_Pos       (4U)
#define EZIP_EZIP_PARA_IN_SEL_Msk       (0x1UL << EZIP_EZIP_PARA_IN_SEL_Pos)
#define EZIP_EZIP_PARA_IN_SEL           EZIP_EZIP_PARA_IN_SEL_Msk
#define EZIP_EZIP_PARA_SPI_SEL_Pos      (5U)
#define EZIP_EZIP_PARA_SPI_SEL_Msk      (0x1UL << EZIP_EZIP_PARA_SPI_SEL_Pos)
#define EZIP_EZIP_PARA_SPI_SEL          EZIP_EZIP_PARA_SPI_SEL_Msk

/***************** Bit definition for EZIP_CACHE_CLR register *****************/
#define EZIP_CACHE_CLR_CACHE_CLR_Pos    (0U)
#define EZIP_CACHE_CLR_CACHE_CLR_Msk    (0x1UL << EZIP_CACHE_CLR_CACHE_CLR_Pos)
#define EZIP_CACHE_CLR_CACHE_CLR        EZIP_CACHE_CLR_CACHE_CLR_Msk

/**************** Bit definition for EZIP_START_POINT register ****************/
#define EZIP_START_POINT_START_ROW_Pos  (0U)
#define EZIP_START_POINT_START_ROW_Msk  (0xFFFFUL << EZIP_START_POINT_START_ROW_Pos)
#define EZIP_START_POINT_START_ROW      EZIP_START_POINT_START_ROW_Msk
#define EZIP_START_POINT_START_COL_Pos  (16U)
#define EZIP_START_POINT_START_COL_Msk  (0xFFFFUL << EZIP_START_POINT_START_COL_Pos)
#define EZIP_START_POINT_START_COL      EZIP_START_POINT_START_COL_Msk

/***************** Bit definition for EZIP_END_POINT register *****************/
#define EZIP_END_POINT_END_ROW_Pos      (0U)
#define EZIP_END_POINT_END_ROW_Msk      (0xFFFFUL << EZIP_END_POINT_END_ROW_Pos)
#define EZIP_END_POINT_END_ROW          EZIP_END_POINT_END_ROW_Msk
#define EZIP_END_POINT_END_COL_Pos      (16U)
#define EZIP_END_POINT_END_COL_Msk      (0xFFFFUL << EZIP_END_POINT_END_COL_Pos)
#define EZIP_END_POINT_END_COL          EZIP_END_POINT_END_COL_Msk

/***************** Bit definition for EZIP_ROW_SIGN register ******************/
#define EZIP_ROW_SIGN_ROW_SIGN_Pos      (0U)
#define EZIP_ROW_SIGN_ROW_SIGN_Msk      (0xFFFFUL << EZIP_ROW_SIGN_ROW_SIGN_Pos)
#define EZIP_ROW_SIGN_ROW_SIGN          EZIP_ROW_SIGN_ROW_SIGN_Msk

/****************** Bit definition for EZIP_INT_EN register *******************/
#define EZIP_INT_EN_END_INT_EN_Pos      (0U)
#define EZIP_INT_EN_END_INT_EN_Msk      (0x1UL << EZIP_INT_EN_END_INT_EN_Pos)
#define EZIP_INT_EN_END_INT_EN          EZIP_INT_EN_END_INT_EN_Msk
#define EZIP_INT_EN_ROW_INT_EN_Pos      (1U)
#define EZIP_INT_EN_ROW_INT_EN_Msk      (0x1UL << EZIP_INT_EN_ROW_INT_EN_Pos)
#define EZIP_INT_EN_ROW_INT_EN          EZIP_INT_EN_ROW_INT_EN_Msk
#define EZIP_INT_EN_ROW_ERR_EN_Pos      (2U)
#define EZIP_INT_EN_ROW_ERR_EN_Msk      (0x1UL << EZIP_INT_EN_ROW_ERR_EN_Pos)
#define EZIP_INT_EN_ROW_ERR_EN          EZIP_INT_EN_ROW_ERR_EN_Msk
#define EZIP_INT_EN_BTYPE_ERR_EN_Pos    (3U)
#define EZIP_INT_EN_BTYPE_ERR_EN_Msk    (0x1UL << EZIP_INT_EN_BTYPE_ERR_EN_Pos)
#define EZIP_INT_EN_BTYPE_ERR_EN        EZIP_INT_EN_BTYPE_ERR_EN_Msk
#define EZIP_INT_EN_ETYPE_ERR_EN_Pos    (4U)
#define EZIP_INT_EN_ETYPE_ERR_EN_Msk    (0x1UL << EZIP_INT_EN_ETYPE_ERR_EN_Pos)
#define EZIP_INT_EN_ETYPE_ERR_EN        EZIP_INT_EN_ETYPE_ERR_EN_Msk
#define EZIP_INT_EN_AEZIP_INT_EN_Pos    (5U)
#define EZIP_INT_EN_AEZIP_INT_EN_Msk    (0x1UL << EZIP_INT_EN_AEZIP_INT_EN_Pos)
#define EZIP_INT_EN_AEZIP_INT_EN        EZIP_INT_EN_AEZIP_INT_EN_Msk

/****************** Bit definition for EZIP_INT_STA register ******************/
#define EZIP_INT_STA_END_INT_STA_Pos    (0U)
#define EZIP_INT_STA_END_INT_STA_Msk    (0x1UL << EZIP_INT_STA_END_INT_STA_Pos)
#define EZIP_INT_STA_END_INT_STA        EZIP_INT_STA_END_INT_STA_Msk
#define EZIP_INT_STA_ROW_INT_STA_Pos    (1U)
#define EZIP_INT_STA_ROW_INT_STA_Msk    (0x1UL << EZIP_INT_STA_ROW_INT_STA_Pos)
#define EZIP_INT_STA_ROW_INT_STA        EZIP_INT_STA_ROW_INT_STA_Msk
#define EZIP_INT_STA_ROW_ERR_STA_Pos    (2U)
#define EZIP_INT_STA_ROW_ERR_STA_Msk    (0x1UL << EZIP_INT_STA_ROW_ERR_STA_Pos)
#define EZIP_INT_STA_ROW_ERR_STA        EZIP_INT_STA_ROW_ERR_STA_Msk
#define EZIP_INT_STA_BTYPE_ERR_STA_Pos  (3U)
#define EZIP_INT_STA_BTYPE_ERR_STA_Msk  (0x1UL << EZIP_INT_STA_BTYPE_ERR_STA_Pos)
#define EZIP_INT_STA_BTYPE_ERR_STA      EZIP_INT_STA_BTYPE_ERR_STA_Msk
#define EZIP_INT_STA_ETYPE_ERR_STA_Pos  (4U)
#define EZIP_INT_STA_ETYPE_ERR_STA_Msk  (0x1UL << EZIP_INT_STA_ETYPE_ERR_STA_Pos)
#define EZIP_INT_STA_ETYPE_ERR_STA      EZIP_INT_STA_ETYPE_ERR_STA_Msk
#define EZIP_INT_STA_AEZIP_INT_STA_Pos  (5U)
#define EZIP_INT_STA_AEZIP_INT_STA_Msk  (0x1UL << EZIP_INT_STA_AEZIP_INT_STA_Pos)
#define EZIP_INT_STA_AEZIP_INT_STA      EZIP_INT_STA_AEZIP_INT_STA_Msk

/***************** Bit definition for EZIP_INT_MASK register ******************/
#define EZIP_INT_MASK_END_INT_MASK_Pos  (0U)
#define EZIP_INT_MASK_END_INT_MASK_Msk  (0x1UL << EZIP_INT_MASK_END_INT_MASK_Pos)
#define EZIP_INT_MASK_END_INT_MASK      EZIP_INT_MASK_END_INT_MASK_Msk
#define EZIP_INT_MASK_ROW_INT_MASK_Pos  (1U)
#define EZIP_INT_MASK_ROW_INT_MASK_Msk  (0x1UL << EZIP_INT_MASK_ROW_INT_MASK_Pos)
#define EZIP_INT_MASK_ROW_INT_MASK      EZIP_INT_MASK_ROW_INT_MASK_Msk
#define EZIP_INT_MASK_ROW_ERR_MASK_Pos  (2U)
#define EZIP_INT_MASK_ROW_ERR_MASK_Msk  (0x1UL << EZIP_INT_MASK_ROW_ERR_MASK_Pos)
#define EZIP_INT_MASK_ROW_ERR_MASK      EZIP_INT_MASK_ROW_ERR_MASK_Msk
#define EZIP_INT_MASK_BTYPE_ERR_MASK_Pos  (3U)
#define EZIP_INT_MASK_BTYPE_ERR_MASK_Msk  (0x1UL << EZIP_INT_MASK_BTYPE_ERR_MASK_Pos)
#define EZIP_INT_MASK_BTYPE_ERR_MASK    EZIP_INT_MASK_BTYPE_ERR_MASK_Msk
#define EZIP_INT_MASK_ETYPE_ERR_MASK_Pos  (4U)
#define EZIP_INT_MASK_ETYPE_ERR_MASK_Msk  (0x1UL << EZIP_INT_MASK_ETYPE_ERR_MASK_Pos)
#define EZIP_INT_MASK_ETYPE_ERR_MASK    EZIP_INT_MASK_ETYPE_ERR_MASK_Msk
#define EZIP_INT_MASK_AEZIP_INT_MASK_Pos  (5U)
#define EZIP_INT_MASK_AEZIP_INT_MASK_Msk  (0x1UL << EZIP_INT_MASK_AEZIP_INT_MASK_Pos)
#define EZIP_INT_MASK_AEZIP_INT_MASK    EZIP_INT_MASK_AEZIP_INT_MASK_Msk

/***************** Bit definition for EZIP_NAP_PARA register ******************/
#define EZIP_NAP_PARA_NAP_TIM_Pos       (0U)
#define EZIP_NAP_PARA_NAP_TIM_Msk       (0xFUL << EZIP_NAP_PARA_NAP_TIM_Pos)
#define EZIP_NAP_PARA_NAP_TIM           EZIP_NAP_PARA_NAP_TIM_Msk
#define EZIP_NAP_PARA_BURST_NUM_Pos     (4U)
#define EZIP_NAP_PARA_BURST_NUM_Msk     (0xFUL << EZIP_NAP_PARA_BURST_NUM_Pos)
#define EZIP_NAP_PARA_BURST_NUM         EZIP_NAP_PARA_BURST_NUM_Msk

/****************** Bit definition for EZIP_SRC_LEN register ******************/
#define EZIP_SRC_LEN_SRC_LEN_Pos        (0U)
#define EZIP_SRC_LEN_SRC_LEN_Msk        (0xFFFFFFFFUL << EZIP_SRC_LEN_SRC_LEN_Pos)
#define EZIP_SRC_LEN_SRC_LEN            EZIP_SRC_LEN_SRC_LEN_Msk

/**************** Bit definition for EZIP_AEZIP_CTRL register *****************/
#define EZIP_AEZIP_CTRL_AEZIP_CTRL_Pos  (0U)
#define EZIP_AEZIP_CTRL_AEZIP_CTRL_Msk  (0x1UL << EZIP_AEZIP_CTRL_AEZIP_CTRL_Pos)
#define EZIP_AEZIP_CTRL_AEZIP_CTRL      EZIP_AEZIP_CTRL_AEZIP_CTRL_Msk

/**************** Bit definition for EZIP_FRAME_START register ****************/
#define EZIP_FRAME_START_FRAME_START_Pos  (0U)
#define EZIP_FRAME_START_FRAME_START_Msk  (0xFFFFFFFFUL << EZIP_FRAME_START_FRAME_START_Pos)
#define EZIP_FRAME_START_FRAME_START    EZIP_FRAME_START_FRAME_START_Msk

/**************** Bit definition for EZIP_PLAY_START register *****************/
#define EZIP_PLAY_START_PLAY_START_Pos  (0U)
#define EZIP_PLAY_START_PLAY_START_Msk  (0xFFFFFFFFUL << EZIP_PLAY_START_PLAY_START_Pos)
#define EZIP_PLAY_START_PLAY_START      EZIP_PLAY_START_PLAY_START_Msk

/***************** Bit definition for EZIP_FRAME_NUM register *****************/
#define EZIP_FRAME_NUM_FRAME_NUM_Pos    (0U)
#define EZIP_FRAME_NUM_FRAME_NUM_Msk    (0xFFFFFFFFUL << EZIP_FRAME_NUM_FRAME_NUM_Pos)
#define EZIP_FRAME_NUM_FRAME_NUM        EZIP_FRAME_NUM_FRAME_NUM_Msk

/***************** Bit definition for EZIP_PLAY_NUM register ******************/
#define EZIP_PLAY_NUM_PLAY_NUM_Pos      (0U)
#define EZIP_PLAY_NUM_PLAY_NUM_Msk      (0xFFFFFFFFUL << EZIP_PLAY_NUM_PLAY_NUM_Pos)
#define EZIP_PLAY_NUM_PLAY_NUM          EZIP_PLAY_NUM_PLAY_NUM_Msk

/****************** Bit definition for EZIP_SEQ_NUM register ******************/
#define EZIP_SEQ_NUM_SEQ_NUM_Pos        (0U)
#define EZIP_SEQ_NUM_SEQ_NUM_Msk        (0xFFFFFFFFUL << EZIP_SEQ_NUM_SEQ_NUM_Pos)
#define EZIP_SEQ_NUM_SEQ_NUM            EZIP_SEQ_NUM_SEQ_NUM_Msk

/**************** Bit definition for EZIP_FRAME_AREA register *****************/
#define EZIP_FRAME_AREA_FRAME_HEIGHT_Pos  (0U)
#define EZIP_FRAME_AREA_FRAME_HEIGHT_Msk  (0xFFFFUL << EZIP_FRAME_AREA_FRAME_HEIGHT_Pos)
#define EZIP_FRAME_AREA_FRAME_HEIGHT    EZIP_FRAME_AREA_FRAME_HEIGHT_Msk
#define EZIP_FRAME_AREA_FRAME_WIDTH_Pos  (16U)
#define EZIP_FRAME_AREA_FRAME_WIDTH_Msk  (0xFFFFUL << EZIP_FRAME_AREA_FRAME_WIDTH_Pos)
#define EZIP_FRAME_AREA_FRAME_WIDTH     EZIP_FRAME_AREA_FRAME_WIDTH_Msk

/*************** Bit definition for EZIP_FRAME_OFFSET register ****************/
#define EZIP_FRAME_OFFSET_OFFEST_ROW_Pos  (0U)
#define EZIP_FRAME_OFFSET_OFFEST_ROW_Msk  (0xFFFFUL << EZIP_FRAME_OFFSET_OFFEST_ROW_Pos)
#define EZIP_FRAME_OFFSET_OFFEST_ROW    EZIP_FRAME_OFFSET_OFFEST_ROW_Msk
#define EZIP_FRAME_OFFSET_OFFSET_COL_Pos  (16U)
#define EZIP_FRAME_OFFSET_OFFSET_COL_Msk  (0xFFFFUL << EZIP_FRAME_OFFSET_OFFSET_COL_Pos)
#define EZIP_FRAME_OFFSET_OFFSET_COL    EZIP_FRAME_OFFSET_OFFSET_COL_Msk

/**************** Bit definition for EZIP_FRAME_DELAY register ****************/
#define EZIP_FRAME_DELAY_DELAY_DEN_Pos  (0U)
#define EZIP_FRAME_DELAY_DELAY_DEN_Msk  (0xFFFFUL << EZIP_FRAME_DELAY_DELAY_DEN_Pos)
#define EZIP_FRAME_DELAY_DELAY_DEN      EZIP_FRAME_DELAY_DELAY_DEN_Msk
#define EZIP_FRAME_DELAY_DELAY_NUM_Pos  (16U)
#define EZIP_FRAME_DELAY_DELAY_NUM_Msk  (0xFFFFUL << EZIP_FRAME_DELAY_DELAY_NUM_Pos)
#define EZIP_FRAME_DELAY_DELAY_NUM      EZIP_FRAME_DELAY_DELAY_NUM_Msk

/**************** Bit definition for EZIP_FRAME_TYPE register *****************/
#define EZIP_FRAME_TYPE_BLEND_OP_Pos    (0U)
#define EZIP_FRAME_TYPE_BLEND_OP_Msk    (0xFFUL << EZIP_FRAME_TYPE_BLEND_OP_Pos)
#define EZIP_FRAME_TYPE_BLEND_OP        EZIP_FRAME_TYPE_BLEND_OP_Msk
#define EZIP_FRAME_TYPE_DISPOSE_OP_Pos  (8U)
#define EZIP_FRAME_TYPE_DISPOSE_OP_Msk  (0xFFUL << EZIP_FRAME_TYPE_DISPOSE_OP_Pos)
#define EZIP_FRAME_TYPE_DISPOSE_OP      EZIP_FRAME_TYPE_DISPOSE_OP_Msk

/**************** Bit definition for EZIP_FRAME_SIZE register *****************/
#define EZIP_FRAME_SIZE_FRAME_SIZE_Pos  (0U)
#define EZIP_FRAME_SIZE_FRAME_SIZE_Msk  (0xFFFFFFFFUL << EZIP_FRAME_SIZE_FRAME_SIZE_Pos)
#define EZIP_FRAME_SIZE_FRAME_SIZE      EZIP_FRAME_SIZE_FRAME_SIZE_Msk

/***************** Bit definition for EZIP_GREY_PARA register *****************/
#define EZIP_GREY_PARA_GREY_PARA_Pos    (0U)
#define EZIP_GREY_PARA_GREY_PARA_Msk    (0xFFFFFFUL << EZIP_GREY_PARA_GREY_PARA_Pos)
#define EZIP_GREY_PARA_GREY_PARA        EZIP_GREY_PARA_GREY_PARA_Msk

/****************** Bit definition for EZIP_DB_SEL register *******************/
#define EZIP_DB_SEL_DB_SEL_Pos          (0U)
#define EZIP_DB_SEL_DB_SEL_Msk          (0xFFFFUL << EZIP_DB_SEL_DB_SEL_Pos)
#define EZIP_DB_SEL_DB_SEL              EZIP_DB_SEL_DB_SEL_Msk

/***************** Bit definition for EZIP_DB_DATA0 register ******************/
#define EZIP_DB_DATA0_DB_DATA0_Pos      (0U)
#define EZIP_DB_DATA0_DB_DATA0_Msk      (0xFFFFFFFFUL << EZIP_DB_DATA0_DB_DATA0_Pos)
#define EZIP_DB_DATA0_DB_DATA0          EZIP_DB_DATA0_DB_DATA0_Msk

/***************** Bit definition for EZIP_DB_DATA1 register ******************/
#define EZIP_DB_DATA1_DB_DATA1_Pos      (0U)
#define EZIP_DB_DATA1_DB_DATA1_Msk      (0xFFFFFFFFUL << EZIP_DB_DATA1_DB_DATA1_Pos)
#define EZIP_DB_DATA1_DB_DATA1          EZIP_DB_DATA1_DB_DATA1_Msk

/***************** Bit definition for EZIP_DB_DATA2 register ******************/
#define EZIP_DB_DATA2_DB_DATA2_Pos      (0U)
#define EZIP_DB_DATA2_DB_DATA2_Msk      (0xFFFFFFFFUL << EZIP_DB_DATA2_DB_DATA2_Pos)
#define EZIP_DB_DATA2_DB_DATA2          EZIP_DB_DATA2_DB_DATA2_Msk

/***************** Bit definition for EZIP_DB_DATA3 register ******************/
#define EZIP_DB_DATA3_DB_DATA3_Pos      (0U)
#define EZIP_DB_DATA3_DB_DATA3_Msk      (0xFFFFFFFFUL << EZIP_DB_DATA3_DB_DATA3_Pos)
#define EZIP_DB_DATA3_DB_DATA3          EZIP_DB_DATA3_DB_DATA3_Msk

/***************** Bit definition for EZIP_DB_DATA4 register ******************/
#define EZIP_DB_DATA4_DB_DATA4_Pos      (0U)
#define EZIP_DB_DATA4_DB_DATA4_Msk      (0xFFFFFFFFUL << EZIP_DB_DATA4_DB_DATA4_Pos)
#define EZIP_DB_DATA4_DB_DATA4          EZIP_DB_DATA4_DB_DATA4_Msk

/***************** Bit definition for EZIP_DB_DATA5 register ******************/
#define EZIP_DB_DATA5_DB_DATA5_Pos      (0U)
#define EZIP_DB_DATA5_DB_DATA5_Msk      (0xFFFFFFFFUL << EZIP_DB_DATA5_DB_DATA5_Pos)
#define EZIP_DB_DATA5_DB_DATA5          EZIP_DB_DATA5_DB_DATA5_Msk

/***************** Bit definition for EZIP_DB_DATA6 register ******************/
#define EZIP_DB_DATA6_DB_DATA6_Pos      (0U)
#define EZIP_DB_DATA6_DB_DATA6_Msk      (0xFFFFFFFFUL << EZIP_DB_DATA6_DB_DATA6_Pos)
#define EZIP_DB_DATA6_DB_DATA6          EZIP_DB_DATA6_DB_DATA6_Msk

/***************** Bit definition for EZIP_DB_DATA7 register ******************/
#define EZIP_DB_DATA7_DB_DATA7_Pos      (0U)
#define EZIP_DB_DATA7_DB_DATA7_Msk      (0xFFFFFFFFUL << EZIP_DB_DATA7_DB_DATA7_Pos)
#define EZIP_DB_DATA7_DB_DATA7          EZIP_DB_DATA7_DB_DATA7_Msk

/***************** Bit definition for EZIP_DB_DATA8 register ******************/
#define EZIP_DB_DATA8_DB_DATA8_Pos      (0U)
#define EZIP_DB_DATA8_DB_DATA8_Msk      (0xFFFFFFFFUL << EZIP_DB_DATA8_DB_DATA8_Pos)
#define EZIP_DB_DATA8_DB_DATA8          EZIP_DB_DATA8_DB_DATA8_Msk

/***************** Bit definition for EZIP_DB_DATA9 register ******************/
#define EZIP_DB_DATA9_DB_DATA9_Pos      (0U)
#define EZIP_DB_DATA9_DB_DATA9_Msk      (0xFFFFFFFFUL << EZIP_DB_DATA9_DB_DATA9_Pos)
#define EZIP_DB_DATA9_DB_DATA9          EZIP_DB_DATA9_DB_DATA9_Msk

/***************** Bit definition for EZIP_DB_DATA10 register *****************/
#define EZIP_DB_DATA10_DB_DATA10_Pos    (0U)
#define EZIP_DB_DATA10_DB_DATA10_Msk    (0xFFFFFFFFUL << EZIP_DB_DATA10_DB_DATA10_Pos)
#define EZIP_DB_DATA10_DB_DATA10        EZIP_DB_DATA10_DB_DATA10_Msk

/***************** Bit definition for EZIP_DB_DATA11 register *****************/
#define EZIP_DB_DATA11_DB_DATA11_Pos    (0U)
#define EZIP_DB_DATA11_DB_DATA11_Msk    (0xFFFFFFFFUL << EZIP_DB_DATA11_DB_DATA11_Pos)
#define EZIP_DB_DATA11_DB_DATA11        EZIP_DB_DATA11_DB_DATA11_Msk

/***************** Bit definition for EZIP_DB_DATA12 register *****************/
#define EZIP_DB_DATA12_DB_DATA12_Pos    (0U)
#define EZIP_DB_DATA12_DB_DATA12_Msk    (0xFFFFFFFFUL << EZIP_DB_DATA12_DB_DATA12_Pos)
#define EZIP_DB_DATA12_DB_DATA12        EZIP_DB_DATA12_DB_DATA12_Msk

/***************** Bit definition for EZIP_DB_DATA13 register *****************/
#define EZIP_DB_DATA13_DB_DATA13_Pos    (0U)
#define EZIP_DB_DATA13_DB_DATA13_Msk    (0xFFFFFFFFUL << EZIP_DB_DATA13_DB_DATA13_Pos)
#define EZIP_DB_DATA13_DB_DATA13        EZIP_DB_DATA13_DB_DATA13_Msk

#endif
