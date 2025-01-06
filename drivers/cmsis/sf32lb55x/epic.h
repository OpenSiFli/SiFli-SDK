/**
  ******************************************************************************
  * @file   epic.h
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
#ifndef __EPIC_H
#define __EPIC_H

typedef struct
{
    __IO uint32_t CFG;
    __IO uint32_t TL_POS;
    __IO uint32_t BR_POS;
    __IO uint32_t FILTER;
    __IO uint32_t SRC;
} EPIC_LayerxTypeDef;

typedef struct
{
    __IO uint32_t CFG;
    __IO uint32_t TL_POS;
    __IO uint32_t BR_POS;
    __IO uint32_t EXTENTS;
    __IO uint32_t FILTER;
    __IO uint32_t SRC;
    __IO uint32_t ROT;
    __IO uint32_t SCALE_RATIO;
    __IO uint32_t MISC_CFG;
} EPIC_VideoLayerxTypeDef;

typedef struct
{
    __IO uint32_t ROT_M_CFG1;
    __IO uint32_t ROT_M_CFG2;
    __IO uint32_t ROT_M_CFG3;
} EPIC_VideoLayerxTransTypeDef;

typedef struct
{
    __IO uint32_t COMMAND;
    __IO uint32_t STATUS;
    __IO uint32_t EOF_IRQ;
    __IO uint32_t SETTING;
    __IO uint32_t CANVAS_TL_POS;
    __IO uint32_t CANVAS_BR_POS;
    __IO uint32_t CANVAS_BG;
    __IO uint32_t VL_CFG;
    __IO uint32_t VL_TL_POS;
    __IO uint32_t VL_BR_POS;
    __IO uint32_t VL_EXTENTS;
    __IO uint32_t VL_FILTER;
    __IO uint32_t VL_SRC;
    __IO uint32_t VL_ROT;
    __IO uint32_t VL_SCALE_RATIO;
    __IO uint32_t VL_MISC_CFG;
    __IO uint32_t L0_CFG;
    __IO uint32_t L0_TL_POS;
    __IO uint32_t L0_BR_POS;
    __IO uint32_t L0_FILTER;
    __IO uint32_t L0_SRC;
    __IO uint32_t L1_CFG;
    __IO uint32_t L1_TL_POS;
    __IO uint32_t L1_BR_POS;
    __IO uint32_t L1_FILTER;
    __IO uint32_t L1_SRC;
    __IO uint32_t L2_CFG;
    __IO uint32_t L2_TL_POS;
    __IO uint32_t L2_BR_POS;
    __IO uint32_t L2_FILTER;
    __IO uint32_t L2_SRC;
    __IO uint32_t AHB_CTRL;
    __IO uint32_t AHB_MEM;
    __IO uint32_t AHB_STRIDE;
    __IO uint32_t DEBUG;
    __IO uint32_t VL_ROT_M_CFG1;
    __IO uint32_t VL_ROT_M_CFG2;
    __IO uint32_t VL_ROT_M_CFG3;
} EPIC_TypeDef;


/****************** Bit definition for EPIC_COMMAND register ******************/
#define EPIC_COMMAND_START_Pos          (0U)
#define EPIC_COMMAND_START_Msk          (0x1UL << EPIC_COMMAND_START_Pos)
#define EPIC_COMMAND_START              EPIC_COMMAND_START_Msk
#define EPIC_COMMAND_RESET_Pos          (1U)
#define EPIC_COMMAND_RESET_Msk          (0x1UL << EPIC_COMMAND_RESET_Pos)
#define EPIC_COMMAND_RESET              EPIC_COMMAND_RESET_Msk

/****************** Bit definition for EPIC_STATUS register *******************/
#define EPIC_STATUS_IA_BUSY_Pos         (0U)
#define EPIC_STATUS_IA_BUSY_Msk         (0x1UL << EPIC_STATUS_IA_BUSY_Pos)
#define EPIC_STATUS_IA_BUSY             EPIC_STATUS_IA_BUSY_Msk
#define EPIC_STATUS_LCD_BUSY_Pos        (4U)
#define EPIC_STATUS_LCD_BUSY_Msk        (0x1UL << EPIC_STATUS_LCD_BUSY_Pos)
#define EPIC_STATUS_LCD_BUSY            EPIC_STATUS_LCD_BUSY_Msk

/****************** Bit definition for EPIC_EOF_IRQ register ******************/
#define EPIC_EOF_IRQ_IRQ_CAUSE_Pos      (0U)
#define EPIC_EOF_IRQ_IRQ_CAUSE_Msk      (0x1UL << EPIC_EOF_IRQ_IRQ_CAUSE_Pos)
#define EPIC_EOF_IRQ_IRQ_CAUSE          EPIC_EOF_IRQ_IRQ_CAUSE_Msk
#define EPIC_EOF_IRQ_LINE_HIT_CAUSE_Pos  (1U)
#define EPIC_EOF_IRQ_LINE_HIT_CAUSE_Msk  (0x1UL << EPIC_EOF_IRQ_LINE_HIT_CAUSE_Pos)
#define EPIC_EOF_IRQ_LINE_HIT_CAUSE     EPIC_EOF_IRQ_LINE_HIT_CAUSE_Msk
#define EPIC_EOF_IRQ_IRQ_STATUS_Pos     (16U)
#define EPIC_EOF_IRQ_IRQ_STATUS_Msk     (0x1UL << EPIC_EOF_IRQ_IRQ_STATUS_Pos)
#define EPIC_EOF_IRQ_IRQ_STATUS         EPIC_EOF_IRQ_IRQ_STATUS_Msk
#define EPIC_EOF_IRQ_LINE_HIT_STATUS_Pos  (17U)
#define EPIC_EOF_IRQ_LINE_HIT_STATUS_Msk  (0x1UL << EPIC_EOF_IRQ_LINE_HIT_STATUS_Pos)
#define EPIC_EOF_IRQ_LINE_HIT_STATUS    EPIC_EOF_IRQ_LINE_HIT_STATUS_Msk

/****************** Bit definition for EPIC_SETTING register ******************/
#define EPIC_SETTING_EOF_IRQ_MASK_Pos   (0U)
#define EPIC_SETTING_EOF_IRQ_MASK_Msk   (0x1UL << EPIC_SETTING_EOF_IRQ_MASK_Pos)
#define EPIC_SETTING_EOF_IRQ_MASK       EPIC_SETTING_EOF_IRQ_MASK_Msk
#define EPIC_SETTING_AUTO_GATE_EN_Pos   (1U)
#define EPIC_SETTING_AUTO_GATE_EN_Msk   (0x1UL << EPIC_SETTING_AUTO_GATE_EN_Pos)
#define EPIC_SETTING_AUTO_GATE_EN       EPIC_SETTING_AUTO_GATE_EN_Msk

/*************** Bit definition for EPIC_CANVAS_TL_POS register ***************/
#define EPIC_CANVAS_TL_POS_X0_Pos       (0U)
#define EPIC_CANVAS_TL_POS_X0_Msk       (0x3FFUL << EPIC_CANVAS_TL_POS_X0_Pos)
#define EPIC_CANVAS_TL_POS_X0           EPIC_CANVAS_TL_POS_X0_Msk
#define EPIC_CANVAS_TL_POS_Y0_Pos       (16U)
#define EPIC_CANVAS_TL_POS_Y0_Msk       (0x3FFUL << EPIC_CANVAS_TL_POS_Y0_Pos)
#define EPIC_CANVAS_TL_POS_Y0           EPIC_CANVAS_TL_POS_Y0_Msk

/*************** Bit definition for EPIC_CANVAS_BR_POS register ***************/
#define EPIC_CANVAS_BR_POS_X1_Pos       (0U)
#define EPIC_CANVAS_BR_POS_X1_Msk       (0x3FFUL << EPIC_CANVAS_BR_POS_X1_Pos)
#define EPIC_CANVAS_BR_POS_X1           EPIC_CANVAS_BR_POS_X1_Msk
#define EPIC_CANVAS_BR_POS_Y1_Pos       (16U)
#define EPIC_CANVAS_BR_POS_Y1_Msk       (0x3FFUL << EPIC_CANVAS_BR_POS_Y1_Pos)
#define EPIC_CANVAS_BR_POS_Y1           EPIC_CANVAS_BR_POS_Y1_Msk

/***************** Bit definition for EPIC_CANVAS_BG register *****************/
#define EPIC_CANVAS_BG_BLUE_Pos         (0U)
#define EPIC_CANVAS_BG_BLUE_Msk         (0xFFUL << EPIC_CANVAS_BG_BLUE_Pos)
#define EPIC_CANVAS_BG_BLUE             EPIC_CANVAS_BG_BLUE_Msk
#define EPIC_CANVAS_BG_GREEN_Pos        (8U)
#define EPIC_CANVAS_BG_GREEN_Msk        (0xFFUL << EPIC_CANVAS_BG_GREEN_Pos)
#define EPIC_CANVAS_BG_GREEN            EPIC_CANVAS_BG_GREEN_Msk
#define EPIC_CANVAS_BG_RED_Pos          (16U)
#define EPIC_CANVAS_BG_RED_Msk          (0xFFUL << EPIC_CANVAS_BG_RED_Pos)
#define EPIC_CANVAS_BG_RED              EPIC_CANVAS_BG_RED_Msk
#define EPIC_CANVAS_BG_BG_BLENDING_BYPASS_Pos  (24U)
#define EPIC_CANVAS_BG_BG_BLENDING_BYPASS_Msk  (0x1UL << EPIC_CANVAS_BG_BG_BLENDING_BYPASS_Pos)
#define EPIC_CANVAS_BG_BG_BLENDING_BYPASS  EPIC_CANVAS_BG_BG_BLENDING_BYPASS_Msk

/****************** Bit definition for EPIC_VL_CFG register *******************/
#define EPIC_VL_CFG_FORMAT_Pos          (0U)
#define EPIC_VL_CFG_FORMAT_Msk          (0x3UL << EPIC_VL_CFG_FORMAT_Pos)
#define EPIC_VL_CFG_FORMAT              EPIC_VL_CFG_FORMAT_Msk
#define EPIC_VL_CFG_ALPHA_SEL_Pos       (2U)
#define EPIC_VL_CFG_ALPHA_SEL_Msk       (0x1UL << EPIC_VL_CFG_ALPHA_SEL_Pos)
#define EPIC_VL_CFG_ALPHA_SEL           EPIC_VL_CFG_ALPHA_SEL_Msk
#define EPIC_VL_CFG_ALPHA_Pos           (3U)
#define EPIC_VL_CFG_ALPHA_Msk           (0xFFUL << EPIC_VL_CFG_ALPHA_Pos)
#define EPIC_VL_CFG_ALPHA               EPIC_VL_CFG_ALPHA_Msk
#define EPIC_VL_CFG_BLEND_DEPTH_Pos     (11U)
#define EPIC_VL_CFG_BLEND_DEPTH_Msk     (0x3UL << EPIC_VL_CFG_BLEND_DEPTH_Pos)
#define EPIC_VL_CFG_BLEND_DEPTH         EPIC_VL_CFG_BLEND_DEPTH_Msk
#define EPIC_VL_CFG_FILTER_EN_Pos       (13U)
#define EPIC_VL_CFG_FILTER_EN_Msk       (0x1UL << EPIC_VL_CFG_FILTER_EN_Pos)
#define EPIC_VL_CFG_FILTER_EN           EPIC_VL_CFG_FILTER_EN_Msk
#define EPIC_VL_CFG_WIDTH_Pos           (14U)
#define EPIC_VL_CFG_WIDTH_Msk           (0x1FFFUL << EPIC_VL_CFG_WIDTH_Pos)
#define EPIC_VL_CFG_WIDTH               EPIC_VL_CFG_WIDTH_Msk
#define EPIC_VL_CFG_PREFETCH_EN_Pos     (27U)
#define EPIC_VL_CFG_PREFETCH_EN_Msk     (0x1UL << EPIC_VL_CFG_PREFETCH_EN_Pos)
#define EPIC_VL_CFG_PREFETCH_EN         EPIC_VL_CFG_PREFETCH_EN_Msk
#define EPIC_VL_CFG_ACTIVE_Pos          (28U)
#define EPIC_VL_CFG_ACTIVE_Msk          (0x1UL << EPIC_VL_CFG_ACTIVE_Pos)
#define EPIC_VL_CFG_ACTIVE              EPIC_VL_CFG_ACTIVE_Msk
#define EPIC_VL_CFG_ALPHA_BLEND_Pos     (29U)
#define EPIC_VL_CFG_ALPHA_BLEND_Msk     (0x1UL << EPIC_VL_CFG_ALPHA_BLEND_Pos)
#define EPIC_VL_CFG_ALPHA_BLEND         EPIC_VL_CFG_ALPHA_BLEND_Msk
#define EPIC_VL_CFG_EZIP_EN_Pos       (30U)
#define EPIC_VL_CFG_EZIP_EN_Msk       (0x1UL << EPIC_VL_CFG_EZIP_EN_Pos)
#define EPIC_VL_CFG_EZIP_EN           EPIC_VL_CFG_EZIP_EN_Msk


/***************** Bit definition for EPIC_VL_TL_POS register *****************/
#define EPIC_VL_TL_POS_X0_Pos           (0U)
#define EPIC_VL_TL_POS_X0_Msk           (0x3FFUL << EPIC_VL_TL_POS_X0_Pos)
#define EPIC_VL_TL_POS_X0               EPIC_VL_TL_POS_X0_Msk
#define EPIC_VL_TL_POS_Y0_Pos           (16U)
#define EPIC_VL_TL_POS_Y0_Msk           (0x3FFUL << EPIC_VL_TL_POS_Y0_Pos)
#define EPIC_VL_TL_POS_Y0               EPIC_VL_TL_POS_Y0_Msk

/***************** Bit definition for EPIC_VL_BR_POS register *****************/
#define EPIC_VL_BR_POS_X1_Pos           (0U)
#define EPIC_VL_BR_POS_X1_Msk           (0x3FFUL << EPIC_VL_BR_POS_X1_Pos)
#define EPIC_VL_BR_POS_X1               EPIC_VL_BR_POS_X1_Msk
#define EPIC_VL_BR_POS_Y1_Pos           (16U)
#define EPIC_VL_BR_POS_Y1_Msk           (0x3FFUL << EPIC_VL_BR_POS_Y1_Pos)
#define EPIC_VL_BR_POS_Y1               EPIC_VL_BR_POS_Y1_Msk

/**************** Bit definition for EPIC_VL_EXTENTS register *****************/
#define EPIC_VL_EXTENTS_MAX_LINE_Pos    (0U)
#define EPIC_VL_EXTENTS_MAX_LINE_Msk    (0x3FFUL << EPIC_VL_EXTENTS_MAX_LINE_Pos)
#define EPIC_VL_EXTENTS_MAX_LINE        EPIC_VL_EXTENTS_MAX_LINE_Msk
#define EPIC_VL_EXTENTS_MAX_COL_Pos     (16U)
#define EPIC_VL_EXTENTS_MAX_COL_Msk     (0x3FFUL << EPIC_VL_EXTENTS_MAX_COL_Pos)
#define EPIC_VL_EXTENTS_MAX_COL         EPIC_VL_EXTENTS_MAX_COL_Msk

/***************** Bit definition for EPIC_VL_FILTER register *****************/
#define EPIC_VL_FILTER_FILTER_B_Pos     (0U)
#define EPIC_VL_FILTER_FILTER_B_Msk     (0xFFUL << EPIC_VL_FILTER_FILTER_B_Pos)
#define EPIC_VL_FILTER_FILTER_B         EPIC_VL_FILTER_FILTER_B_Msk
#define EPIC_VL_FILTER_FILTER_G_Pos     (8U)
#define EPIC_VL_FILTER_FILTER_G_Msk     (0xFFUL << EPIC_VL_FILTER_FILTER_G_Pos)
#define EPIC_VL_FILTER_FILTER_G         EPIC_VL_FILTER_FILTER_G_Msk
#define EPIC_VL_FILTER_FILTER_R_Pos     (16U)
#define EPIC_VL_FILTER_FILTER_R_Msk     (0xFFUL << EPIC_VL_FILTER_FILTER_R_Pos)
#define EPIC_VL_FILTER_FILTER_R         EPIC_VL_FILTER_FILTER_R_Msk
#define EPIC_VL_FILTER_FILTER_MASK_Pos  (24U)
#define EPIC_VL_FILTER_FILTER_MASK_Msk  (0xFFUL << EPIC_VL_FILTER_FILTER_MASK_Pos)
#define EPIC_VL_FILTER_FILTER_MASK      EPIC_VL_FILTER_FILTER_MASK_Msk

/****************** Bit definition for EPIC_VL_SRC register *******************/
#define EPIC_VL_SRC_ADDR_Pos            (0U)
#define EPIC_VL_SRC_ADDR_Msk            (0xFFFFFFFFUL << EPIC_VL_SRC_ADDR_Pos)
#define EPIC_VL_SRC_ADDR                EPIC_VL_SRC_ADDR_Msk

/****************** Bit definition for EPIC_VL_ROT register *******************/
#define EPIC_VL_ROT_ROT_MAX_LINE_Pos    (0U)
#define EPIC_VL_ROT_ROT_MAX_LINE_Msk    (0x3FFUL << EPIC_VL_ROT_ROT_MAX_LINE_Pos)
#define EPIC_VL_ROT_ROT_MAX_LINE        EPIC_VL_ROT_ROT_MAX_LINE_Msk
#define EPIC_VL_ROT_ROT_MAX_COL_Pos     (10U)
#define EPIC_VL_ROT_ROT_MAX_COL_Msk     (0x3FFUL << EPIC_VL_ROT_ROT_MAX_COL_Pos)
#define EPIC_VL_ROT_ROT_MAX_COL         EPIC_VL_ROT_ROT_MAX_COL_Msk
#define EPIC_VL_ROT_CALC_DONE_Pos       (20U)
#define EPIC_VL_ROT_CALC_DONE_Msk       (0x1UL << EPIC_VL_ROT_CALC_DONE_Pos)
#define EPIC_VL_ROT_CALC_DONE           EPIC_VL_ROT_CALC_DONE_Msk
#define EPIC_VL_ROT_ROT_DEG_Pos         (21U)
#define EPIC_VL_ROT_ROT_DEG_Msk         (0x1FFUL << EPIC_VL_ROT_ROT_DEG_Pos)
#define EPIC_VL_ROT_ROT_DEG             EPIC_VL_ROT_ROT_DEG_Msk
#define EPIC_VL_ROT_CALC_CLR_Pos        (30U)
#define EPIC_VL_ROT_CALC_CLR_Msk        (0x1UL << EPIC_VL_ROT_CALC_CLR_Pos)
#define EPIC_VL_ROT_CALC_CLR            EPIC_VL_ROT_CALC_CLR_Msk
#define EPIC_VL_ROT_CALC_REQ_Pos        (31U)
#define EPIC_VL_ROT_CALC_REQ_Msk        (0x1UL << EPIC_VL_ROT_CALC_REQ_Pos)
#define EPIC_VL_ROT_CALC_REQ            EPIC_VL_ROT_CALC_REQ_Msk

/************** Bit definition for EPIC_VL_SCALE_RATIO register ***************/
#define EPIC_VL_SCALE_RATIO_XPITCH_Pos  (0U)
#define EPIC_VL_SCALE_RATIO_XPITCH_Msk  (0x7FFUL << EPIC_VL_SCALE_RATIO_XPITCH_Pos)
#define EPIC_VL_SCALE_RATIO_XPITCH      EPIC_VL_SCALE_RATIO_XPITCH_Msk

#define EPIC_VL_SCALE_RATIO_YPITCH_Pos  (16U)
#define EPIC_VL_SCALE_RATIO_YPITCH_Msk  (0x7FFUL << EPIC_VL_SCALE_RATIO_YPITCH_Pos)
#define EPIC_VL_SCALE_RATIO_YPITCH      EPIC_VL_SCALE_RATIO_YPITCH_Msk


/**************** Bit definition for EPIC_VL_MISC_CFG register ****************/
#define EPIC_VL_MISC_CFG_MIRROR_Pos     (0U)
#define EPIC_VL_MISC_CFG_MIRROR_Msk     (0x1UL << EPIC_VL_MISC_CFG_MIRROR_Pos)
#define EPIC_VL_MISC_CFG_MIRROR         EPIC_VL_MISC_CFG_MIRROR_Msk
#define EPIC_VL_MISC_CFG_COS_FORCE_VALUE_Pos     (2U)
#define EPIC_VL_MISC_CFG_COS_FORCE_VALUE_Msk     (0x1FFFUL << EPIC_VL_MISC_CFG_COS_FORCE_VALUE_Pos)
#define EPIC_VL_MISC_CFG_COS_FORCE_VALUE         EPIC_VL_MISC_CFG_COS_FORCE_VALUE_Msk
#define EPIC_VL_MISC_CFG_COS_FRAC_BIT            (12)
#define EPIC_VL_MISC_CFG_SIN_FORCE_VALUE_Pos  (15U)
#define EPIC_VL_MISC_CFG_SIN_FORCE_VALUE_Msk  (0x1FFFUL << EPIC_VL_MISC_CFG_SIN_FORCE_VALUE_Pos)
#define EPIC_VL_MISC_CFG_SIN_FORCE_VALUE  EPIC_VL_MISC_CFG_SIN_FORCE_VALUE_Msk
#define EPIC_VL_MISC_CFG_SIN_FRAC_BIT    (EPIC_VL_MISC_CFG_COS_FRAC_BIT)
#define EPIC_VL_MISC_CFG_DEG_FORCE_Pos  (28U)
#define EPIC_VL_MISC_CFG_DEG_FORCE_Msk  (0x1UL << EPIC_VL_MISC_CFG_DEG_FORCE_Pos)
#define EPIC_VL_MISC_CFG_DEG_FORCE      EPIC_VL_MISC_CFG_DEG_FORCE_Msk

/****************** Bit definition for EPIC_L0_CFG register *******************/
#define EPIC_L0_CFG_FORMAT_Pos          (0U)
#define EPIC_L0_CFG_FORMAT_Msk          (0x3UL << EPIC_L0_CFG_FORMAT_Pos)
#define EPIC_L0_CFG_FORMAT              EPIC_L0_CFG_FORMAT_Msk
#define EPIC_L0_CFG_FMT_RGB565          (0 << EPIC_L0_CFG_FORMAT_Pos)
#define EPIC_L0_CFG_FMT_RGB888          (1 << EPIC_L0_CFG_FORMAT_Pos)
#define EPIC_L0_CFG_FMT_ARGB8888        (2 << EPIC_L0_CFG_FORMAT_Pos)
#define EPIC_L0_CFG_FMT_ARGB8565        (3 << EPIC_L0_CFG_FORMAT_Pos)
#define EPIC_L0_CFG_ALPHA_SEL_Pos       (2U)
#define EPIC_L0_CFG_ALPHA_SEL_Msk       (0x1UL << EPIC_L0_CFG_ALPHA_SEL_Pos)
#define EPIC_L0_CFG_ALPHA_SEL           EPIC_L0_CFG_ALPHA_SEL_Msk
#define EPIC_L0_CFG_ALPHA_Pos           (3U)
#define EPIC_L0_CFG_ALPHA_Msk           (0xFFUL << EPIC_L0_CFG_ALPHA_Pos)
#define EPIC_L0_CFG_ALPHA               EPIC_L0_CFG_ALPHA_Msk
#define EPIC_L0_CFG_FILTER_EN_Pos       (13U)
#define EPIC_L0_CFG_FILTER_EN_Msk       (0x1UL << EPIC_L0_CFG_FILTER_EN_Pos)
#define EPIC_L0_CFG_FILTER_EN           EPIC_L0_CFG_FILTER_EN_Msk
#define EPIC_L0_CFG_WIDTH_Pos           (14U)
#define EPIC_L0_CFG_WIDTH_Msk           (0x1FFFUL << EPIC_L0_CFG_WIDTH_Pos)
#define EPIC_L0_CFG_WIDTH               EPIC_L0_CFG_WIDTH_Msk
#define EPIC_L0_CFG_PREFETCH_EN_Pos     (27U)
#define EPIC_L0_CFG_PREFETCH_EN_Msk     (0x1UL << EPIC_L0_CFG_PREFETCH_EN_Pos)
#define EPIC_L0_CFG_PREFETCH_EN         EPIC_L0_CFG_PREFETCH_EN_Msk
#define EPIC_L0_CFG_ACTIVE_Pos          (28U)
#define EPIC_L0_CFG_ACTIVE_Msk          (0x1UL << EPIC_L0_CFG_ACTIVE_Pos)
#define EPIC_L0_CFG_ACTIVE              EPIC_L0_CFG_ACTIVE_Msk
#define EPIC_L0_CFG_ALPHA_BLEND_Pos     (29U)
#define EPIC_L0_CFG_ALPHA_BLEND_Msk     (0x1UL << EPIC_L0_CFG_ALPHA_BLEND_Pos)
#define EPIC_L0_CFG_ALPHA_BLEND         EPIC_L0_CFG_ALPHA_BLEND_Msk
#define EPIC_L0_CFG_EZIP_EN_Pos       (30U)
#define EPIC_L0_CFG_EZIP_EN_Msk       (0x1UL << EPIC_L0_CFG_EZIP_EN_Pos)
#define EPIC_L0_CFG_EZIP_EN           EPIC_L0_CFG_EZIP_EN_Msk

/***************** Bit definition for EPIC_L0_TL_POS register *****************/
#define EPIC_L0_TL_POS_X0_Pos           (0U)
#define EPIC_L0_TL_POS_X0_Msk           (0x3FFUL << EPIC_L0_TL_POS_X0_Pos)
#define EPIC_L0_TL_POS_X0               EPIC_L0_TL_POS_X0_Msk
#define EPIC_L0_TL_POS_Y0_Pos           (16U)
#define EPIC_L0_TL_POS_Y0_Msk           (0x3FFUL << EPIC_L0_TL_POS_Y0_Pos)
#define EPIC_L0_TL_POS_Y0               EPIC_L0_TL_POS_Y0_Msk

/***************** Bit definition for EPIC_L0_BR_POS register *****************/
#define EPIC_L0_BR_POS_X1_Pos           (0U)
#define EPIC_L0_BR_POS_X1_Msk           (0x3FFUL << EPIC_L0_BR_POS_X1_Pos)
#define EPIC_L0_BR_POS_X1               EPIC_L0_BR_POS_X1_Msk
#define EPIC_L0_BR_POS_Y1_Pos           (16U)
#define EPIC_L0_BR_POS_Y1_Msk           (0x3FFUL << EPIC_L0_BR_POS_Y1_Pos)
#define EPIC_L0_BR_POS_Y1               EPIC_L0_BR_POS_Y1_Msk

/***************** Bit definition for EPIC_L0_FILTER register *****************/
#define EPIC_L0_FILTER_FILTER_B_Pos     (0U)
#define EPIC_L0_FILTER_FILTER_B_Msk     (0xFFUL << EPIC_L0_FILTER_FILTER_B_Pos)
#define EPIC_L0_FILTER_FILTER_B         EPIC_L0_FILTER_FILTER_B_Msk
#define EPIC_L0_FILTER_FILTER_G_Pos     (8U)
#define EPIC_L0_FILTER_FILTER_G_Msk     (0xFFUL << EPIC_L0_FILTER_FILTER_G_Pos)
#define EPIC_L0_FILTER_FILTER_G         EPIC_L0_FILTER_FILTER_G_Msk
#define EPIC_L0_FILTER_FILTER_R_Pos     (16U)
#define EPIC_L0_FILTER_FILTER_R_Msk     (0xFFUL << EPIC_L0_FILTER_FILTER_R_Pos)
#define EPIC_L0_FILTER_FILTER_R         EPIC_L0_FILTER_FILTER_R_Msk
#define EPIC_L0_FILTER_FILTER_MASK_Pos  (24U)
#define EPIC_L0_FILTER_FILTER_MASK_Msk  (0xFFUL << EPIC_L0_FILTER_FILTER_MASK_Pos)
#define EPIC_L0_FILTER_FILTER_MASK      EPIC_L0_FILTER_FILTER_MASK_Msk

/****************** Bit definition for EPIC_L0_SRC register *******************/
#define EPIC_L0_SRC_ADDR_Pos            (0U)
#define EPIC_L0_SRC_ADDR_Msk            (0xFFFFFFFFUL << EPIC_L0_SRC_ADDR_Pos)
#define EPIC_L0_SRC_ADDR                EPIC_L0_SRC_ADDR_Msk

/****************** Bit definition for EPIC_L1_CFG register *******************/
#define EPIC_L1_CFG_FORMAT_Pos          (0U)
#define EPIC_L1_CFG_FORMAT_Msk          (0x3UL << EPIC_L1_CFG_FORMAT_Pos)
#define EPIC_L1_CFG_FORMAT              EPIC_L1_CFG_FORMAT_Msk
#define EPIC_L1_CFG_ALPHA_SEL_Pos       (2U)
#define EPIC_L1_CFG_ALPHA_SEL_Msk       (0x1UL << EPIC_L1_CFG_ALPHA_SEL_Pos)
#define EPIC_L1_CFG_ALPHA_SEL           EPIC_L1_CFG_ALPHA_SEL_Msk
#define EPIC_L1_CFG_ALPHA_Pos           (3U)
#define EPIC_L1_CFG_ALPHA_Msk           (0xFFUL << EPIC_L1_CFG_ALPHA_Pos)
#define EPIC_L1_CFG_ALPHA               EPIC_L1_CFG_ALPHA_Msk
#define EPIC_L1_CFG_FILTER_EN_Pos       (13U)
#define EPIC_L1_CFG_FILTER_EN_Msk       (0x1UL << EPIC_L1_CFG_FILTER_EN_Pos)
#define EPIC_L1_CFG_FILTER_EN           EPIC_L1_CFG_FILTER_EN_Msk
#define EPIC_L1_CFG_WIDTH_Pos           (14U)
#define EPIC_L1_CFG_WIDTH_Msk           (0x1FFFUL << EPIC_L1_CFG_WIDTH_Pos)
#define EPIC_L1_CFG_WIDTH               EPIC_L1_CFG_WIDTH_Msk
#define EPIC_L1_CFG_PREFETCH_EN_Pos     (27U)
#define EPIC_L1_CFG_PREFETCH_EN_Msk     (0x1UL << EPIC_L1_CFG_PREFETCH_EN_Pos)
#define EPIC_L1_CFG_PREFETCH_EN         EPIC_L1_CFG_PREFETCH_EN_Msk
#define EPIC_L1_CFG_ACTIVE_Pos          (28U)
#define EPIC_L1_CFG_ACTIVE_Msk          (0x1UL << EPIC_L1_CFG_ACTIVE_Pos)
#define EPIC_L1_CFG_ACTIVE              EPIC_L1_CFG_ACTIVE_Msk
#define EPIC_L1_CFG_ALPHA_BLEND_Pos     (29U)
#define EPIC_L1_CFG_ALPHA_BLEND_Msk     (0x1UL << EPIC_L1_CFG_ALPHA_BLEND_Pos)
#define EPIC_L1_CFG_ALPHA_BLEND         EPIC_L1_CFG_ALPHA_BLEND_Msk
#define EPIC_L1_CFG_EZIP_EN_Pos       (30U)
#define EPIC_L1_CFG_EZIP_EN_Msk       (0x1UL << EPIC_L1_CFG_EZIP_EN_Pos)
#define EPIC_L1_CFG_EZIP_EN           EPIC_L1_CFG_EZIP_EN_Msk

/***************** Bit definition for EPIC_L1_TL_POS register *****************/
#define EPIC_L1_TL_POS_X0_Pos           (0U)
#define EPIC_L1_TL_POS_X0_Msk           (0x3FFUL << EPIC_L1_TL_POS_X0_Pos)
#define EPIC_L1_TL_POS_X0               EPIC_L1_TL_POS_X0_Msk
#define EPIC_L1_TL_POS_Y0_Pos           (16U)
#define EPIC_L1_TL_POS_Y0_Msk           (0x3FFUL << EPIC_L1_TL_POS_Y0_Pos)
#define EPIC_L1_TL_POS_Y0               EPIC_L1_TL_POS_Y0_Msk

/***************** Bit definition for EPIC_L1_BR_POS register *****************/
#define EPIC_L1_BR_POS_X1_Pos           (0U)
#define EPIC_L1_BR_POS_X1_Msk           (0x3FFUL << EPIC_L1_BR_POS_X1_Pos)
#define EPIC_L1_BR_POS_X1               EPIC_L1_BR_POS_X1_Msk
#define EPIC_L1_BR_POS_Y1_Pos           (16U)
#define EPIC_L1_BR_POS_Y1_Msk           (0x3FFUL << EPIC_L1_BR_POS_Y1_Pos)
#define EPIC_L1_BR_POS_Y1               EPIC_L1_BR_POS_Y1_Msk

/***************** Bit definition for EPIC_L1_FILTER register *****************/
#define EPIC_L1_FILTER_FILTER_B_Pos     (0U)
#define EPIC_L1_FILTER_FILTER_B_Msk     (0xFFUL << EPIC_L1_FILTER_FILTER_B_Pos)
#define EPIC_L1_FILTER_FILTER_B         EPIC_L1_FILTER_FILTER_B_Msk
#define EPIC_L1_FILTER_FILTER_G_Pos     (8U)
#define EPIC_L1_FILTER_FILTER_G_Msk     (0xFFUL << EPIC_L1_FILTER_FILTER_G_Pos)
#define EPIC_L1_FILTER_FILTER_G         EPIC_L1_FILTER_FILTER_G_Msk
#define EPIC_L1_FILTER_FILTER_R_Pos     (16U)
#define EPIC_L1_FILTER_FILTER_R_Msk     (0xFFUL << EPIC_L1_FILTER_FILTER_R_Pos)
#define EPIC_L1_FILTER_FILTER_R         EPIC_L1_FILTER_FILTER_R_Msk
#define EPIC_L1_FILTER_FILTER_MASK_Pos  (24U)
#define EPIC_L1_FILTER_FILTER_MASK_Msk  (0xFFUL << EPIC_L1_FILTER_FILTER_MASK_Pos)
#define EPIC_L1_FILTER_FILTER_MASK      EPIC_L1_FILTER_FILTER_MASK_Msk

/****************** Bit definition for EPIC_L1_SRC register *******************/
#define EPIC_L1_SRC_ADDR_Pos            (0U)
#define EPIC_L1_SRC_ADDR_Msk            (0xFFFFFFFFUL << EPIC_L1_SRC_ADDR_Pos)
#define EPIC_L1_SRC_ADDR                EPIC_L1_SRC_ADDR_Msk

/****************** Bit definition for EPIC_L2_CFG register *******************/
#define EPIC_L2_CFG_FORMAT_Pos          (0U)
#define EPIC_L2_CFG_FORMAT_Msk          (0x3UL << EPIC_L2_CFG_FORMAT_Pos)
#define EPIC_L2_CFG_FORMAT              EPIC_L2_CFG_FORMAT_Msk
#define EPIC_L2_CFG_ALPHA_SEL_Pos       (2U)
#define EPIC_L2_CFG_ALPHA_SEL_Msk       (0x1UL << EPIC_L2_CFG_ALPHA_SEL_Pos)
#define EPIC_L2_CFG_ALPHA_SEL           EPIC_L2_CFG_ALPHA_SEL_Msk
#define EPIC_L2_CFG_ALPHA_Pos           (3U)
#define EPIC_L2_CFG_ALPHA_Msk           (0xFFUL << EPIC_L2_CFG_ALPHA_Pos)
#define EPIC_L2_CFG_ALPHA               EPIC_L2_CFG_ALPHA_Msk
#define EPIC_L2_CFG_FILTER_EN_Pos       (13U)
#define EPIC_L2_CFG_FILTER_EN_Msk       (0x1UL << EPIC_L2_CFG_FILTER_EN_Pos)
#define EPIC_L2_CFG_FILTER_EN           EPIC_L2_CFG_FILTER_EN_Msk
#define EPIC_L2_CFG_WIDTH_Pos           (14U)
#define EPIC_L2_CFG_WIDTH_Msk           (0x1FFFUL << EPIC_L2_CFG_WIDTH_Pos)
#define EPIC_L2_CFG_WIDTH               EPIC_L2_CFG_WIDTH_Msk
#define EPIC_L2_CFG_PREFETCH_EN_Pos     (27U)
#define EPIC_L2_CFG_PREFETCH_EN_Msk     (0x1UL << EPIC_L2_CFG_PREFETCH_EN_Pos)
#define EPIC_L2_CFG_PREFETCH_EN         EPIC_L2_CFG_PREFETCH_EN_Msk
#define EPIC_L2_CFG_ACTIVE_Pos          (28U)
#define EPIC_L2_CFG_ACTIVE_Msk          (0x1UL << EPIC_L2_CFG_ACTIVE_Pos)
#define EPIC_L2_CFG_ACTIVE              EPIC_L2_CFG_ACTIVE_Msk
#define EPIC_L2_CFG_ALPHA_BLEND_Pos     (29U)
#define EPIC_L2_CFG_ALPHA_BLEND_Msk     (0x1UL << EPIC_L2_CFG_ALPHA_BLEND_Pos)
#define EPIC_L2_CFG_ALPHA_BLEND         EPIC_L2_CFG_ALPHA_BLEND_Msk
#define EPIC_L2_CFG_EZIP_EN_Pos       (30U)
#define EPIC_L2_CFG_EZIP_EN_Msk       (0x1UL << EPIC_L2_CFG_EZIP_EN_Pos)
#define EPIC_L2_CFG_EZIP_EN           EPIC_L2_CFG_EZIP_EN_Msk

/***************** Bit definition for EPIC_L2_TL_POS register *****************/
#define EPIC_L2_TL_POS_X0_Pos           (0U)
#define EPIC_L2_TL_POS_X0_Msk           (0x3FFUL << EPIC_L2_TL_POS_X0_Pos)
#define EPIC_L2_TL_POS_X0               EPIC_L2_TL_POS_X0_Msk
#define EPIC_L2_TL_POS_Y0_Pos           (16U)
#define EPIC_L2_TL_POS_Y0_Msk           (0x3FFUL << EPIC_L2_TL_POS_Y0_Pos)
#define EPIC_L2_TL_POS_Y0               EPIC_L2_TL_POS_Y0_Msk

/***************** Bit definition for EPIC_L2_BR_POS register *****************/
#define EPIC_L2_BR_POS_X1_Pos           (0U)
#define EPIC_L2_BR_POS_X1_Msk           (0x3FFUL << EPIC_L2_BR_POS_X1_Pos)
#define EPIC_L2_BR_POS_X1               EPIC_L2_BR_POS_X1_Msk
#define EPIC_L2_BR_POS_Y1_Pos           (16U)
#define EPIC_L2_BR_POS_Y1_Msk           (0x3FFUL << EPIC_L2_BR_POS_Y1_Pos)
#define EPIC_L2_BR_POS_Y1               EPIC_L2_BR_POS_Y1_Msk

/***************** Bit definition for EPIC_L2_FILTER register *****************/
#define EPIC_L2_FILTER_FILTER_B_Pos     (0U)
#define EPIC_L2_FILTER_FILTER_B_Msk     (0xFFUL << EPIC_L2_FILTER_FILTER_B_Pos)
#define EPIC_L2_FILTER_FILTER_B         EPIC_L2_FILTER_FILTER_B_Msk
#define EPIC_L2_FILTER_FILTER_G_Pos     (8U)
#define EPIC_L2_FILTER_FILTER_G_Msk     (0xFFUL << EPIC_L2_FILTER_FILTER_G_Pos)
#define EPIC_L2_FILTER_FILTER_G         EPIC_L2_FILTER_FILTER_G_Msk
#define EPIC_L2_FILTER_FILTER_R_Pos     (16U)
#define EPIC_L2_FILTER_FILTER_R_Msk     (0xFFUL << EPIC_L2_FILTER_FILTER_R_Pos)
#define EPIC_L2_FILTER_FILTER_R         EPIC_L2_FILTER_FILTER_R_Msk
#define EPIC_L2_FILTER_FILTER_MASK_Pos  (24U)
#define EPIC_L2_FILTER_FILTER_MASK_Msk  (0xFFUL << EPIC_L2_FILTER_FILTER_MASK_Pos)
#define EPIC_L2_FILTER_FILTER_MASK      EPIC_L2_FILTER_FILTER_MASK_Msk

/****************** Bit definition for EPIC_L2_SRC register *******************/
#define EPIC_L2_SRC_ADDR_Pos            (0U)
#define EPIC_L2_SRC_ADDR_Msk            (0xFFFFFFFFUL << EPIC_L2_SRC_ADDR_Pos)
#define EPIC_L2_SRC_ADDR                EPIC_L2_SRC_ADDR_Msk

/***************** Bit definition for EPIC_AHB_CTRL register ******************/
#define EPIC_AHB_CTRL_DESTINATION_Pos   (0U)
#define EPIC_AHB_CTRL_DESTINATION_Msk   (0x1UL << EPIC_AHB_CTRL_DESTINATION_Pos)
#define EPIC_AHB_CTRL_DESTINATION       EPIC_AHB_CTRL_DESTINATION_Msk
#define EPIC_AHB_CTRL_DEST_RAM          (0UL << EPIC_AHB_CTRL_DESTINATION_Pos)
#define EPIC_AHB_CTRL_DEST_LCD          (1UL << EPIC_AHB_CTRL_DESTINATION_Pos)
#define EPIC_AHB_CTRL_O_FORMAT_Pos      (1U)
#define EPIC_AHB_CTRL_O_FORMAT_Msk      (0x3UL << EPIC_AHB_CTRL_O_FORMAT_Pos)
#define EPIC_AHB_CTRL_O_FORMAT          EPIC_AHB_CTRL_O_FORMAT_Msk
#define EPIC_AHB_CTRL_O_FMT_RGB565      (0 << EPIC_AHB_CTRL_O_FORMAT_Pos)
#define EPIC_AHB_CTRL_O_FMT_RGB888      (1 << EPIC_AHB_CTRL_O_FORMAT_Pos)
#define EPIC_AHB_CTRL_O_FMT_ARGB8888    (2 << EPIC_AHB_CTRL_O_FORMAT_Pos)
#define EPIC_AHB_CTRL_O_FMT_ARGB8565    (3 << EPIC_AHB_CTRL_O_FORMAT_Pos)

/****************** Bit definition for EPIC_AHB_MEM register ******************/
#define EPIC_AHB_MEM_ADDR_Pos           (0U)
#define EPIC_AHB_MEM_ADDR_Msk           (0xFFFFFFFFUL << EPIC_AHB_MEM_ADDR_Pos)
#define EPIC_AHB_MEM_ADDR               EPIC_AHB_MEM_ADDR_Msk


/**************** Bit definition for EPIC_AHB_STRIDE register *****************/
#define EPIC_AHB_STRIDE_OFFSET_Pos      (0U)
#define EPIC_AHB_STRIDE_OFFSET_Msk      (0xFFFFUL << EPIC_AHB_STRIDE_OFFSET_Pos)
#define EPIC_AHB_STRIDE_OFFSET          EPIC_AHB_STRIDE_OFFSET_Msk


/******************* Bit definition for EPIC_DEBUG register *******************/
#define EPIC_DEBUG_DEBUG_OUT_SEL_Pos    (0U)
#define EPIC_DEBUG_DEBUG_OUT_SEL_Msk    (0xFUL << EPIC_DEBUG_DEBUG_OUT_SEL_Pos)
#define EPIC_DEBUG_DEBUG_OUT_SEL        EPIC_DEBUG_DEBUG_OUT_SEL_Msk
#define EPIC_DEBUG_DEBUG_INT_SEL_Pos    (4U)
#define EPIC_DEBUG_DEBUG_INT_SEL_Msk    (0xFUL << EPIC_DEBUG_DEBUG_INT_SEL_Pos)
#define EPIC_DEBUG_DEBUG_INT_SEL        EPIC_DEBUG_DEBUG_INT_SEL_Msk
#define EPIC_DEBUG_DEBUG_INT_DATA_Pos   (16U)
#define EPIC_DEBUG_DEBUG_INT_DATA_Msk   (0xFFFFUL << EPIC_DEBUG_DEBUG_INT_DATA_Pos)
#define EPIC_DEBUG_DEBUG_INT_DATA       EPIC_DEBUG_DEBUG_INT_DATA_Msk

/*************** Bit definition for EPIC_VL_ROT_M_CFG1 register ***************/
#define EPIC_VL_ROT_M_CFG1_M_ROT_MAX_LINE_Pos  (0U)
#define EPIC_VL_ROT_M_CFG1_M_ROT_MAX_LINE_Msk  (0x3FFUL << EPIC_VL_ROT_M_CFG1_M_ROT_MAX_LINE_Pos)
#define EPIC_VL_ROT_M_CFG1_M_ROT_MAX_LINE  EPIC_VL_ROT_M_CFG1_M_ROT_MAX_LINE_Msk
#define EPIC_VL_ROT_M_CFG1_M_ROT_MAX_COL_Pos  (16U)
#define EPIC_VL_ROT_M_CFG1_M_ROT_MAX_COL_Msk  (0x3FFUL << EPIC_VL_ROT_M_CFG1_M_ROT_MAX_COL_Pos)
#define EPIC_VL_ROT_M_CFG1_M_ROT_MAX_COL  EPIC_VL_ROT_M_CFG1_M_ROT_MAX_COL_Msk
#define EPIC_VL_ROT_M_CFG1_M_MODE_Pos   (31U)
#define EPIC_VL_ROT_M_CFG1_M_MODE_Msk   (0x1UL << EPIC_VL_ROT_M_CFG1_M_MODE_Pos)
#define EPIC_VL_ROT_M_CFG1_M_MODE       EPIC_VL_ROT_M_CFG1_M_MODE_Msk

/*************** Bit definition for EPIC_VL_ROT_M_CFG2 register ***************/
#define EPIC_VL_ROT_M_CFG2_M_PIVOT_X_Pos  (0U)
#define EPIC_VL_ROT_M_CFG2_M_PIVOT_X_Msk  (0x3FFUL << EPIC_VL_ROT_M_CFG2_M_PIVOT_X_Pos)
#define EPIC_VL_ROT_M_CFG2_M_PIVOT_X    EPIC_VL_ROT_M_CFG2_M_PIVOT_X_Msk
#define EPIC_VL_ROT_M_CFG2_M_PIVOT_Y_Pos  (16U)
#define EPIC_VL_ROT_M_CFG2_M_PIVOT_Y_Msk  (0x3FFUL << EPIC_VL_ROT_M_CFG2_M_PIVOT_Y_Pos)
#define EPIC_VL_ROT_M_CFG2_M_PIVOT_Y    EPIC_VL_ROT_M_CFG2_M_PIVOT_Y_Msk

/*************** Bit definition for EPIC_VL_ROT_M_CFG3 register ***************/
#define EPIC_VL_ROT_M_CFG3_M_XTL_Pos    (0U)
#define EPIC_VL_ROT_M_CFG3_M_XTL_Msk    (0x7FFUL << EPIC_VL_ROT_M_CFG3_M_XTL_Pos)
#define EPIC_VL_ROT_M_CFG3_M_XTL        EPIC_VL_ROT_M_CFG3_M_XTL_Msk
#define EPIC_VL_ROT_M_CFG3_M_YTL_Pos    (16U)
#define EPIC_VL_ROT_M_CFG3_M_YTL_Msk    (0x7FFUL << EPIC_VL_ROT_M_CFG3_M_YTL_Pos)
#define EPIC_VL_ROT_M_CFG3_M_YTL        EPIC_VL_ROT_M_CFG3_M_YTL_Msk

#define EPIC_MAX_X_SIZE  (1023)
#define EPIC_VL_SCALE_RATIO_XPITCH_MAX  (EPIC_VL_SCALE_RATIO_XPITCH_Msk >> EPIC_VL_SCALE_RATIO_XPITCH_Pos)
#define EPIC_VL_SCALE_RATIO_YPITCH_MAX  (EPIC_VL_SCALE_RATIO_YPITCH_Msk >> EPIC_VL_SCALE_RATIO_YPITCH_Pos)
#endif
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
