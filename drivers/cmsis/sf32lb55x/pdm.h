#ifndef __PDM_H
#define __PDM_H

typedef struct
{
    __IO uint32_t CFG0;
    __IO uint32_t CFG1;
    __IO uint32_t SINC_CFG;
    __IO uint32_t RSVD2[2];
    __IO uint32_t HPF_CFG;
    __IO uint32_t PGA_CFG;
    __IO uint32_t RSVD1[6];
    __IO uint32_t LPF_CFG6;
    __IO uint32_t FIFO_CFG;
    __IO uint32_t DATA_L;
    __IO uint32_t DATA_R;
    __IO uint32_t FIFO_ST;
    __IO uint32_t INT_ST;
    __IO uint32_t INT_MSK;
    __IO uint32_t INT_CLR;
} PDM_TypeDef;


/******************** Bit definition for PDM_CFG0 register ********************/
#define PDM_CFG0_PDMCOREEN_Pos          (0U)
#define PDM_CFG0_PDMCOREEN_Msk          (0x1UL << PDM_CFG0_PDMCOREEN_Pos)
#define PDM_CFG0_PDMCOREEN              PDM_CFG0_PDMCOREEN_Msk
#define PDM_CFG0_CLK_SEL_Pos            (1U)
#define PDM_CFG0_CLK_SEL_Msk            (0x1UL << PDM_CFG0_CLK_SEL_Pos)
#define PDM_CFG0_CLK_SEL                PDM_CFG0_CLK_SEL_Msk
#define PDM_CFG0_CLK_DIV_Pos            (2U)
#define PDM_CFG0_CLK_DIV_Msk            (0xFUL << PDM_CFG0_CLK_DIV_Pos)
#define PDM_CFG0_CLK_DIV                PDM_CFG0_CLK_DIV_Msk
#define PDM_CFG0_LEFT_EN_Pos            (6U)
#define PDM_CFG0_LEFT_EN_Msk            (0x1UL << PDM_CFG0_LEFT_EN_Pos)
#define PDM_CFG0_LEFT_EN                PDM_CFG0_LEFT_EN_Msk
#define PDM_CFG0_RIGHT_EN_Pos           (7U)
#define PDM_CFG0_RIGHT_EN_Msk           (0x1UL << PDM_CFG0_RIGHT_EN_Pos)
#define PDM_CFG0_RIGHT_EN               PDM_CFG0_RIGHT_EN_Msk
#define PDM_CFG0_STEREO_EN_Pos          (8U)
#define PDM_CFG0_STEREO_EN_Msk          (0x1UL << PDM_CFG0_STEREO_EN_Pos)
#define PDM_CFG0_STEREO_EN              PDM_CFG0_STEREO_EN_Msk
#define PDM_CFG0_SWAP_EN_Pos            (9U)
#define PDM_CFG0_SWAP_EN_Msk            (0x1UL << PDM_CFG0_SWAP_EN_Pos)
#define PDM_CFG0_SWAP_EN                PDM_CFG0_SWAP_EN_Msk

/******************** Bit definition for PDM_CFG1 register ********************/
#define PDM_CFG1_BUF_DLY_Pos            (0U)
#define PDM_CFG1_BUF_DLY_Msk            (0x1FUL << PDM_CFG1_BUF_DLY_Pos)
#define PDM_CFG1_BUF_DLY                PDM_CFG1_BUF_DLY_Msk
#define PDM_CFG1_SAMPLE_DLY_L_Pos       (5U)
#define PDM_CFG1_SAMPLE_DLY_L_Msk       (0x7UL << PDM_CFG1_SAMPLE_DLY_L_Pos)
#define PDM_CFG1_SAMPLE_DLY_L           PDM_CFG1_SAMPLE_DLY_L_Msk
#define PDM_CFG1_SAMPLE_DLY_R_Pos       (8U)
#define PDM_CFG1_SAMPLE_DLY_R_Msk       (0x7UL << PDM_CFG1_SAMPLE_DLY_R_Pos)
#define PDM_CFG1_SAMPLE_DLY_R           PDM_CFG1_SAMPLE_DLY_R_Msk

/****************** Bit definition for PDM_SINC_CFG register ******************/
#define PDM_SINC_CFG_SINC_RATE_Pos      (0U)
#define PDM_SINC_CFG_SINC_RATE_Msk      (0xFFUL << PDM_SINC_CFG_SINC_RATE_Pos)
#define PDM_SINC_CFG_SINC_RATE          PDM_SINC_CFG_SINC_RATE_Msk
#define PDM_SINC_CFG_SINC_ORDER_SEL_Pos  (8U)
#define PDM_SINC_CFG_SINC_ORDER_SEL_Msk  (0x1UL << PDM_SINC_CFG_SINC_ORDER_SEL_Pos)
#define PDM_SINC_CFG_SINC_ORDER_SEL     PDM_SINC_CFG_SINC_ORDER_SEL_Msk

/****************** Bit definition for PDM_HPF_CFG register *******************/
#define PDM_HPF_CFG_HPF_COEFF_Pos       (0U)
#define PDM_HPF_CFG_HPF_COEFF_Msk       (0xFUL << PDM_HPF_CFG_HPF_COEFF_Pos)
#define PDM_HPF_CFG_HPF_COEFF           PDM_HPF_CFG_HPF_COEFF_Msk
#define PDM_HPF_CFG_HPF_BYPASS_Pos      (4U)
#define PDM_HPF_CFG_HPF_BYPASS_Msk      (0x1UL << PDM_HPF_CFG_HPF_BYPASS_Pos)
#define PDM_HPF_CFG_HPF_BYPASS          PDM_HPF_CFG_HPF_BYPASS_Msk
#define PDM_HPF_CFG_HPF_RST_Pos         (5U)
#define PDM_HPF_CFG_HPF_RST_Msk         (0x1UL << PDM_HPF_CFG_HPF_RST_Pos)
#define PDM_HPF_CFG_HPF_RST             PDM_HPF_CFG_HPF_RST_Msk

/****************** Bit definition for PDM_PGA_CFG register *******************/
#define PDM_PGA_CFG_PGA_GAIN_L_Pos      (0U)
#define PDM_PGA_CFG_PGA_GAIN_L_Msk      (0x7FUL << PDM_PGA_CFG_PGA_GAIN_L_Pos)
#define PDM_PGA_CFG_PGA_GAIN_L          PDM_PGA_CFG_PGA_GAIN_L_Msk
#define PDM_PGA_CFG_PGA_GAIN_R_Pos      (7U)
#define PDM_PGA_CFG_PGA_GAIN_R_Msk      (0x7FUL << PDM_PGA_CFG_PGA_GAIN_R_Pos)
#define PDM_PGA_CFG_PGA_GAIN_R          PDM_PGA_CFG_PGA_GAIN_R_Msk

/****************** Bit definition for PDM_LPF_CFG6 register ******************/
#define PDM_LPF_CFG6_LPF_DS_Pos         (12U)
#define PDM_LPF_CFG6_LPF_DS_Msk         (0x1UL << PDM_LPF_CFG6_LPF_DS_Pos)
#define PDM_LPF_CFG6_LPF_DS             PDM_LPF_CFG6_LPF_DS_Msk
#define PDM_LPF_CFG6_LPF_BYPASS_Pos     (13U)
#define PDM_LPF_CFG6_LPF_BYPASS_Msk     (0x1UL << PDM_LPF_CFG6_LPF_BYPASS_Pos)
#define PDM_LPF_CFG6_LPF_BYPASS         PDM_LPF_CFG6_LPF_BYPASS_Msk

/****************** Bit definition for PDM_FIFO_CFG register ******************/
#define PDM_FIFO_CFG_BYTE_CON_Pos       (0U)
#define PDM_FIFO_CFG_BYTE_CON_Msk       (0x1UL << PDM_FIFO_CFG_BYTE_CON_Pos)
#define PDM_FIFO_CFG_BYTE_CON           PDM_FIFO_CFG_BYTE_CON_Msk
#define PDM_FIFO_CFG_BYTE_TRUNC_Pos     (1U)
#define PDM_FIFO_CFG_BYTE_TRUNC_Msk     (0x1UL << PDM_FIFO_CFG_BYTE_TRUNC_Pos)
#define PDM_FIFO_CFG_BYTE_TRUNC         PDM_FIFO_CFG_BYTE_TRUNC_Msk
#define PDM_FIFO_CFG_PDM_SHIFT_Pos      (2U)
#define PDM_FIFO_CFG_PDM_SHIFT_Msk      (0x7UL << PDM_FIFO_CFG_PDM_SHIFT_Pos)
#define PDM_FIFO_CFG_PDM_SHIFT          PDM_FIFO_CFG_PDM_SHIFT_Msk
#define PDM_FIFO_CFG_RX_DMA_MSK_R_Pos   (5U)
#define PDM_FIFO_CFG_RX_DMA_MSK_R_Msk   (0x1UL << PDM_FIFO_CFG_RX_DMA_MSK_R_Pos)
#define PDM_FIFO_CFG_RX_DMA_MSK_R       PDM_FIFO_CFG_RX_DMA_MSK_R_Msk
#define PDM_FIFO_CFG_RX_DMA_MSK_L_Pos   (6U)
#define PDM_FIFO_CFG_RX_DMA_MSK_L_Msk   (0x1UL << PDM_FIFO_CFG_RX_DMA_MSK_L_Pos)
#define PDM_FIFO_CFG_RX_DMA_MSK_L       PDM_FIFO_CFG_RX_DMA_MSK_L_Msk
#define PDM_FIFO_CFG_LR_CHG_Pos         (7U)
#define PDM_FIFO_CFG_LR_CHG_Msk         (0x1UL << PDM_FIFO_CFG_LR_CHG_Pos)
#define PDM_FIFO_CFG_LR_CHG             PDM_FIFO_CFG_LR_CHG_Msk

/******************* Bit definition for PDM_DATA_L register *******************/
#define PDM_DATA_L_DMA_ENTRY_L_Pos      (0U)
#define PDM_DATA_L_DMA_ENTRY_L_Msk      (0xFFFFFFFFUL << PDM_DATA_L_DMA_ENTRY_L_Pos)
#define PDM_DATA_L_DMA_ENTRY_L          PDM_DATA_L_DMA_ENTRY_L_Msk

/******************* Bit definition for PDM_DATA_R register *******************/
#define PDM_DATA_R_DMA_ENTRY_R_Pos      (0U)
#define PDM_DATA_R_DMA_ENTRY_R_Msk      (0xFFFFFFFFUL << PDM_DATA_R_DMA_ENTRY_R_Pos)
#define PDM_DATA_R_DMA_ENTRY_R          PDM_DATA_R_DMA_ENTRY_R_Msk

/****************** Bit definition for PDM_FIFO_ST register *******************/
#define PDM_FIFO_ST_ALMOST_EMPTY_R_Pos  (0U)
#define PDM_FIFO_ST_ALMOST_EMPTY_R_Msk  (0x1UL << PDM_FIFO_ST_ALMOST_EMPTY_R_Pos)
#define PDM_FIFO_ST_ALMOST_EMPTY_R      PDM_FIFO_ST_ALMOST_EMPTY_R_Msk
#define PDM_FIFO_ST_ALMOST_FULL_R_Pos   (1U)
#define PDM_FIFO_ST_ALMOST_FULL_R_Msk   (0x1UL << PDM_FIFO_ST_ALMOST_FULL_R_Pos)
#define PDM_FIFO_ST_ALMOST_FULL_R       PDM_FIFO_ST_ALMOST_FULL_R_Msk
#define PDM_FIFO_ST_EMPTY_R_Pos         (2U)
#define PDM_FIFO_ST_EMPTY_R_Msk         (0x1UL << PDM_FIFO_ST_EMPTY_R_Pos)
#define PDM_FIFO_ST_EMPTY_R             PDM_FIFO_ST_EMPTY_R_Msk
#define PDM_FIFO_ST_FULL_R_Pos          (3U)
#define PDM_FIFO_ST_FULL_R_Msk          (0x1UL << PDM_FIFO_ST_FULL_R_Pos)
#define PDM_FIFO_ST_FULL_R              PDM_FIFO_ST_FULL_R_Msk
#define PDM_FIFO_ST_ALMOST_EMPTY_L_Pos  (4U)
#define PDM_FIFO_ST_ALMOST_EMPTY_L_Msk  (0x1UL << PDM_FIFO_ST_ALMOST_EMPTY_L_Pos)
#define PDM_FIFO_ST_ALMOST_EMPTY_L      PDM_FIFO_ST_ALMOST_EMPTY_L_Msk
#define PDM_FIFO_ST_ALMOST_FULL_L_Pos   (5U)
#define PDM_FIFO_ST_ALMOST_FULL_L_Msk   (0x1UL << PDM_FIFO_ST_ALMOST_FULL_L_Pos)
#define PDM_FIFO_ST_ALMOST_FULL_L       PDM_FIFO_ST_ALMOST_FULL_L_Msk
#define PDM_FIFO_ST_EMPTY_L_Pos         (6U)
#define PDM_FIFO_ST_EMPTY_L_Msk         (0x1UL << PDM_FIFO_ST_EMPTY_L_Pos)
#define PDM_FIFO_ST_EMPTY_L             PDM_FIFO_ST_EMPTY_L_Msk
#define PDM_FIFO_ST_FULL_L_Pos          (7U)
#define PDM_FIFO_ST_FULL_L_Msk          (0x1UL << PDM_FIFO_ST_FULL_L_Pos)
#define PDM_FIFO_ST_FULL_L              PDM_FIFO_ST_FULL_L_Msk

/******************* Bit definition for PDM_INT_ST register *******************/
#define PDM_INT_ST_OVERFLOW_R_Pos       (0U)
#define PDM_INT_ST_OVERFLOW_R_Msk       (0x1UL << PDM_INT_ST_OVERFLOW_R_Pos)
#define PDM_INT_ST_OVERFLOW_R           PDM_INT_ST_OVERFLOW_R_Msk
#define PDM_INT_ST_OVERFLOW_L_Pos       (1U)
#define PDM_INT_ST_OVERFLOW_L_Msk       (0x1UL << PDM_INT_ST_OVERFLOW_L_Pos)
#define PDM_INT_ST_OVERFLOW_L           PDM_INT_ST_OVERFLOW_L_Msk

/****************** Bit definition for PDM_INT_MSK register *******************/
#define PDM_INT_MSK_INT_MASK_R_Pos      (0U)
#define PDM_INT_MSK_INT_MASK_R_Msk      (0x1UL << PDM_INT_MSK_INT_MASK_R_Pos)
#define PDM_INT_MSK_INT_MASK_R          PDM_INT_MSK_INT_MASK_R_Msk
#define PDM_INT_MSK_INT_MASK_L_Pos      (1U)
#define PDM_INT_MSK_INT_MASK_L_Msk      (0x1UL << PDM_INT_MSK_INT_MASK_L_Pos)
#define PDM_INT_MSK_INT_MASK_L          PDM_INT_MSK_INT_MASK_L_Msk

/****************** Bit definition for PDM_INT_CLR register *******************/
#define PDM_INT_CLR_INT_CLR_R_Pos       (0U)
#define PDM_INT_CLR_INT_CLR_R_Msk       (0x1UL << PDM_INT_CLR_INT_CLR_R_Pos)
#define PDM_INT_CLR_INT_CLR_R           PDM_INT_CLR_INT_CLR_R_Msk
#define PDM_INT_CLR_INT_CLR_L_Pos       (1U)
#define PDM_INT_CLR_INT_CLR_L_Msk       (0x1UL << PDM_INT_CLR_INT_CLR_L_Pos)
#define PDM_INT_CLR_INT_CLR_L           PDM_INT_CLR_INT_CLR_L_Msk

#endif
