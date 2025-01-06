#ifndef __SCI_H
#define __SCI_H

typedef struct
{
    __IO uint32_t CONFIG;
    __IO uint32_t STATUS;
    __IO uint32_t DATA;
    __IO uint32_t CLKDIV;
    __IO uint32_t RXCNT;
    __IO uint32_t TIMES;
    __IO uint32_t CH_FILT;
    __IO uint32_t DBG;
    __IO uint32_t INT_CAUSE;
    __IO uint32_t INT_CLR;
    __IO uint32_t INT_MASK;
    __IO uint32_t MISC_CTRL;
    __IO uint32_t PA_STATUS;
} SCI_TypeDef;


/******************* Bit definition for SCI_CONFIG register *******************/
#define SCI_CONFIG_ENABLE_Pos           (0U)
#define SCI_CONFIG_ENABLE_Msk           (0x1UL << SCI_CONFIG_ENABLE_Pos)
#define SCI_CONFIG_ENABLE               SCI_CONFIG_ENABLE_Msk
#define SCI_CONFIG_PARITY_Pos           (1U)
#define SCI_CONFIG_PARITY_Msk           (0x1UL << SCI_CONFIG_PARITY_Pos)
#define SCI_CONFIG_PARITY               SCI_CONFIG_PARITY_Msk
#define SCI_CONFIG_PERF_Pos             (2U)
#define SCI_CONFIG_PERF_Msk             (0x1UL << SCI_CONFIG_PERF_Pos)
#define SCI_CONFIG_PERF                 SCI_CONFIG_PERF_Msk
#define SCI_CONFIG_FILTER_DISABLE_Pos   (3U)
#define SCI_CONFIG_FILTER_DISABLE_Msk   (0x1UL << SCI_CONFIG_FILTER_DISABLE_Pos)
#define SCI_CONFIG_FILTER_DISABLE       SCI_CONFIG_FILTER_DISABLE_Msk
#define SCI_CONFIG_CLOCKSTOP_Pos        (4U)
#define SCI_CONFIG_CLOCKSTOP_Msk        (0x1UL << SCI_CONFIG_CLOCKSTOP_Pos)
#define SCI_CONFIG_CLOCKSTOP            SCI_CONFIG_CLOCKSTOP_Msk
#define SCI_CONFIG_AUTOSTOP_EN_H_Pos    (5U)
#define SCI_CONFIG_AUTOSTOP_EN_H_Msk    (0x1UL << SCI_CONFIG_AUTOSTOP_EN_H_Pos)
#define SCI_CONFIG_AUTOSTOP_EN_H        SCI_CONFIG_AUTOSTOP_EN_H_Msk
#define SCI_CONFIG_MSBH_LSBL_Pos        (6U)
#define SCI_CONFIG_MSBH_LSBL_Msk        (0x1UL << SCI_CONFIG_MSBH_LSBL_Pos)
#define SCI_CONFIG_MSBH_LSBL            SCI_CONFIG_MSBH_LSBL_Msk
#define SCI_CONFIG_LLI_Pos              (7U)
#define SCI_CONFIG_LLI_Msk              (0x1UL << SCI_CONFIG_LLI_Pos)
#define SCI_CONFIG_LLI                  SCI_CONFIG_LLI_Msk
#define SCI_CONFIG_PEGEN_LEN_Pos        (8U)
#define SCI_CONFIG_PEGEN_LEN_Msk        (0x1UL << SCI_CONFIG_PEGEN_LEN_Pos)
#define SCI_CONFIG_PEGEN_LEN            SCI_CONFIG_PEGEN_LEN_Msk
#define SCI_CONFIG_PARITY_EN_Pos        (9U)
#define SCI_CONFIG_PARITY_EN_Msk        (0x1UL << SCI_CONFIG_PARITY_EN_Pos)
#define SCI_CONFIG_PARITY_EN            SCI_CONFIG_PARITY_EN_Msk
#define SCI_CONFIG_STOP_LEVEL_Pos       (10U)
#define SCI_CONFIG_STOP_LEVEL_Msk       (0x1UL << SCI_CONFIG_STOP_LEVEL_Pos)
#define SCI_CONFIG_STOP_LEVEL           SCI_CONFIG_STOP_LEVEL_Msk
#define SCI_CONFIG_RX_CLK_CNT_SAMPLE_Pos  (11U)
#define SCI_CONFIG_RX_CLK_CNT_SAMPLE_Msk  (0x1FUL << SCI_CONFIG_RX_CLK_CNT_SAMPLE_Pos)
#define SCI_CONFIG_RX_CLK_CNT_SAMPLE    SCI_CONFIG_RX_CLK_CNT_SAMPLE_Msk
#define SCI_CONFIG_ARG_H_Pos            (16U)
#define SCI_CONFIG_ARG_H_Msk            (0x1UL << SCI_CONFIG_ARG_H_Pos)
#define SCI_CONFIG_ARG_H                SCI_CONFIG_ARG_H_Msk
#define SCI_CONFIG_AFD_EN_H_Pos         (17U)
#define SCI_CONFIG_AFD_EN_H_Msk         (0x1UL << SCI_CONFIG_AFD_EN_H_Pos)
#define SCI_CONFIG_AFD_EN_H             SCI_CONFIG_AFD_EN_H_Msk
#define SCI_CONFIG_TX_RESEND_EN_H_Pos   (18U)
#define SCI_CONFIG_TX_RESEND_EN_H_Msk   (0x1UL << SCI_CONFIG_TX_RESEND_EN_H_Pos)
#define SCI_CONFIG_TX_RESEND_EN_H       SCI_CONFIG_TX_RESEND_EN_H_Msk
#define SCI_CONFIG_RESET_Pos            (20U)
#define SCI_CONFIG_RESET_Msk            (0x1UL << SCI_CONFIG_RESET_Pos)
#define SCI_CONFIG_RESET                SCI_CONFIG_RESET_Msk
#define SCI_CONFIG_DLY_SEL_Pos          (21U)
#define SCI_CONFIG_DLY_SEL_Msk          (0x1UL << SCI_CONFIG_DLY_SEL_Pos)
#define SCI_CONFIG_DLY_SEL              SCI_CONFIG_DLY_SEL_Msk
#define SCI_CONFIG_IN_AVG_EN_Pos        (22U)
#define SCI_CONFIG_IN_AVG_EN_Msk        (0x1UL << SCI_CONFIG_IN_AVG_EN_Pos)
#define SCI_CONFIG_IN_AVG_EN            SCI_CONFIG_IN_AVG_EN_Msk
#define SCI_CONFIG_PAR_CHK_OFFSET_Pos   (24U)
#define SCI_CONFIG_PAR_CHK_OFFSET_Msk   (0x3FUL << SCI_CONFIG_PAR_CHK_OFFSET_Pos)
#define SCI_CONFIG_PAR_CHK_OFFSET       SCI_CONFIG_PAR_CHK_OFFSET_Msk
#define SCI_CONFIG_SCI_MODE_Pos         (30U)
#define SCI_CONFIG_SCI_MODE_Msk         (0x3UL << SCI_CONFIG_SCI_MODE_Pos)
#define SCI_CONFIG_SCI_MODE             SCI_CONFIG_SCI_MODE_Msk

/******************* Bit definition for SCI_STATUS register *******************/
#define SCI_STATUS_RXDATA_RDY_Pos       (0U)
#define SCI_STATUS_RXDATA_RDY_Msk       (0x1UL << SCI_STATUS_RXDATA_RDY_Pos)
#define SCI_STATUS_RXDATA_RDY           SCI_STATUS_RXDATA_RDY_Msk
#define SCI_STATUS_TX_FIFO_RDY_Pos      (1U)
#define SCI_STATUS_TX_FIFO_RDY_Msk      (0x1UL << SCI_STATUS_TX_FIFO_RDY_Pos)
#define SCI_STATUS_TX_FIFO_RDY          SCI_STATUS_TX_FIFO_RDY_Msk
#define SCI_STATUS_FORMAT_DET_Pos       (2U)
#define SCI_STATUS_FORMAT_DET_Msk       (0x1UL << SCI_STATUS_FORMAT_DET_Pos)
#define SCI_STATUS_FORMAT_DET           SCI_STATUS_FORMAT_DET_Msk
#define SCI_STATUS_ARG_DET_Pos          (3U)
#define SCI_STATUS_ARG_DET_Msk          (0x1UL << SCI_STATUS_ARG_DET_Pos)
#define SCI_STATUS_ARG_DET              SCI_STATUS_ARG_DET_Msk
#define SCI_STATUS_RESET_DET_Pos        (4U)
#define SCI_STATUS_RESET_DET_Msk        (0x1UL << SCI_STATUS_RESET_DET_Pos)
#define SCI_STATUS_RESET_DET            SCI_STATUS_RESET_DET_Msk
#define SCI_STATUS_CLK_RDY_H_Pos        (5U)
#define SCI_STATUS_CLK_RDY_H_Msk        (0x1UL << SCI_STATUS_CLK_RDY_H_Pos)
#define SCI_STATUS_CLK_RDY_H            SCI_STATUS_CLK_RDY_H_Msk
#define SCI_STATUS_CLK_OFF_Pos          (6U)
#define SCI_STATUS_CLK_OFF_Msk          (0x1UL << SCI_STATUS_CLK_OFF_Pos)
#define SCI_STATUS_CLK_OFF              SCI_STATUS_CLK_OFF_Msk
#define SCI_STATUS_RX_ERR_Pos           (8U)
#define SCI_STATUS_RX_ERR_Msk           (0x1UL << SCI_STATUS_RX_ERR_Pos)
#define SCI_STATUS_RX_ERR               SCI_STATUS_RX_ERR_Msk
#define SCI_STATUS_TX_ERR_Pos           (9U)
#define SCI_STATUS_TX_ERR_Msk           (0x1UL << SCI_STATUS_TX_ERR_Pos)
#define SCI_STATUS_TX_ERR               SCI_STATUS_TX_ERR_Msk
#define SCI_STATUS_RXOVERFLOW_Pos       (10U)
#define SCI_STATUS_RXOVERFLOW_Msk       (0x1UL << SCI_STATUS_RXOVERFLOW_Pos)
#define SCI_STATUS_RXOVERFLOW           SCI_STATUS_RXOVERFLOW_Msk
#define SCI_STATUS_TXOVERFLOW_Pos       (11U)
#define SCI_STATUS_TXOVERFLOW_Msk       (0x1UL << SCI_STATUS_TXOVERFLOW_Pos)
#define SCI_STATUS_TXOVERFLOW           SCI_STATUS_TXOVERFLOW_Msk
#define SCI_STATUS_AUTOSTOP_STATE_Pos   (30U)
#define SCI_STATUS_AUTOSTOP_STATE_Msk   (0x3UL << SCI_STATUS_AUTOSTOP_STATE_Pos)
#define SCI_STATUS_AUTOSTOP_STATE       SCI_STATUS_AUTOSTOP_STATE_Msk

/******************** Bit definition for SCI_DATA register ********************/
#define SCI_DATA_DATA_IO_Pos            (0U)
#define SCI_DATA_DATA_IO_Msk            (0xFFUL << SCI_DATA_DATA_IO_Pos)
#define SCI_DATA_DATA_IO                SCI_DATA_DATA_IO_Msk

/******************* Bit definition for SCI_CLKDIV register *******************/
#define SCI_CLKDIV_VAL_Pos              (0U)
#define SCI_CLKDIV_VAL_Msk              (0x1FFUL << SCI_CLKDIV_VAL_Pos)
#define SCI_CLKDIV_VAL                  SCI_CLKDIV_VAL_Msk
#define SCI_CLKDIV_BAUD_X8_EN_Pos       (9U)
#define SCI_CLKDIV_BAUD_X8_EN_Msk       (0x1UL << SCI_CLKDIV_BAUD_X8_EN_Pos)
#define SCI_CLKDIV_BAUD_X8_EN           SCI_CLKDIV_BAUD_X8_EN_Msk
#define SCI_CLKDIV_RX_CLK_CNT_LIMIT_Pos  (10U)
#define SCI_CLKDIV_RX_CLK_CNT_LIMIT_Msk  (0x1FUL << SCI_CLKDIV_RX_CLK_CNT_LIMIT_Pos)
#define SCI_CLKDIV_RX_CLK_CNT_LIMIT     SCI_CLKDIV_RX_CLK_CNT_LIMIT_Msk
#define SCI_CLKDIV_CLK_TST_Pos          (15U)
#define SCI_CLKDIV_CLK_TST_Msk          (0x1UL << SCI_CLKDIV_CLK_TST_Pos)
#define SCI_CLKDIV_CLK_TST              SCI_CLKDIV_CLK_TST_Msk
#define SCI_CLKDIV_VAL_16_Pos           (16U)
#define SCI_CLKDIV_VAL_16_Msk           (0xFFUL << SCI_CLKDIV_VAL_16_Pos)
#define SCI_CLKDIV_VAL_16               SCI_CLKDIV_VAL_16_Msk
#define SCI_CLKDIV_MAINDIV_Pos          (24U)
#define SCI_CLKDIV_MAINDIV_Msk          (0x3FUL << SCI_CLKDIV_MAINDIV_Pos)
#define SCI_CLKDIV_MAINDIV              SCI_CLKDIV_MAINDIV_Msk
#define SCI_CLKDIV_CLK_OUT_INV_Pos      (30U)
#define SCI_CLKDIV_CLK_OUT_INV_Msk      (0x1UL << SCI_CLKDIV_CLK_OUT_INV_Pos)
#define SCI_CLKDIV_CLK_OUT_INV          SCI_CLKDIV_CLK_OUT_INV_Msk
#define SCI_CLKDIV_CLK_INV_Pos          (31U)
#define SCI_CLKDIV_CLK_INV_Msk          (0x1UL << SCI_CLKDIV_CLK_INV_Pos)
#define SCI_CLKDIV_CLK_INV              SCI_CLKDIV_CLK_INV_Msk

/******************* Bit definition for SCI_RXCNT register ********************/
#define SCI_RXCNT_VAL_Pos               (0U)
#define SCI_RXCNT_VAL_Msk               (0x3FFUL << SCI_RXCNT_VAL_Pos)
#define SCI_RXCNT_VAL                   SCI_RXCNT_VAL_Msk
#define SCI_RXCNT_CLK_PERSIST_Pos       (31U)
#define SCI_RXCNT_CLK_PERSIST_Msk       (0x1UL << SCI_RXCNT_CLK_PERSIST_Pos)
#define SCI_RXCNT_CLK_PERSIST           SCI_RXCNT_CLK_PERSIST_Msk

/******************* Bit definition for SCI_TIMES register ********************/
#define SCI_TIMES_CHGUARD_Pos           (0U)
#define SCI_TIMES_CHGUARD_Msk           (0xFFUL << SCI_TIMES_CHGUARD_Pos)
#define SCI_TIMES_CHGUARD               SCI_TIMES_CHGUARD_Msk
#define SCI_TIMES_TURNAROUNDGUARD_Pos   (8U)
#define SCI_TIMES_TURNAROUNDGUARD_Msk   (0xFUL << SCI_TIMES_TURNAROUNDGUARD_Pos)
#define SCI_TIMES_TURNAROUNDGUARD       SCI_TIMES_TURNAROUNDGUARD_Msk
#define SCI_TIMES_WI_Pos                (16U)
#define SCI_TIMES_WI_Msk                (0xFFUL << SCI_TIMES_WI_Pos)
#define SCI_TIMES_WI                    SCI_TIMES_WI_Msk
#define SCI_TIMES_TX_PERT_Pos           (24U)
#define SCI_TIMES_TX_PERT_Msk           (0xFFUL << SCI_TIMES_TX_PERT_Pos)
#define SCI_TIMES_TX_PERT               SCI_TIMES_TX_PERT_Msk

/****************** Bit definition for SCI_CH_FILT register *******************/
#define SCI_CH_FILT_VAL_Pos             (0U)
#define SCI_CH_FILT_VAL_Msk             (0xFFUL << SCI_CH_FILT_VAL_Pos)
#define SCI_CH_FILT_VAL                 SCI_CH_FILT_VAL_Msk

/******************** Bit definition for SCI_DBG register *********************/
#define SCI_DBG_FIFO_RX_CLR_Pos         (0U)
#define SCI_DBG_FIFO_RX_CLR_Msk         (0x1UL << SCI_DBG_FIFO_RX_CLR_Pos)
#define SCI_DBG_FIFO_RX_CLR             SCI_DBG_FIFO_RX_CLR_Msk
#define SCI_DBG_FIFO_TX_CLR_Pos         (1U)
#define SCI_DBG_FIFO_TX_CLR_Msk         (0x1UL << SCI_DBG_FIFO_TX_CLR_Pos)
#define SCI_DBG_FIFO_TX_CLR             SCI_DBG_FIFO_TX_CLR_Msk

/***************** Bit definition for SCI_INT_CAUSE register ******************/
#define SCI_INT_CAUSE_RX_DONE_Pos       (0U)
#define SCI_INT_CAUSE_RX_DONE_Msk       (0x1UL << SCI_INT_CAUSE_RX_DONE_Pos)
#define SCI_INT_CAUSE_RX_DONE           SCI_INT_CAUSE_RX_DONE_Msk
#define SCI_INT_CAUSE_RX_HALF_Pos       (1U)
#define SCI_INT_CAUSE_RX_HALF_Msk       (0x1UL << SCI_INT_CAUSE_RX_HALF_Pos)
#define SCI_INT_CAUSE_RX_HALF           SCI_INT_CAUSE_RX_HALF_Msk
#define SCI_INT_CAUSE_WWT_TIMEOUT_Pos   (2U)
#define SCI_INT_CAUSE_WWT_TIMEOUT_Msk   (0x1UL << SCI_INT_CAUSE_WWT_TIMEOUT_Pos)
#define SCI_INT_CAUSE_WWT_TIMEOUT       SCI_INT_CAUSE_WWT_TIMEOUT_Msk
#define SCI_INT_CAUSE_EXTRA_RX_Pos      (3U)
#define SCI_INT_CAUSE_EXTRA_RX_Msk      (0x1UL << SCI_INT_CAUSE_EXTRA_RX_Pos)
#define SCI_INT_CAUSE_EXTRA_RX          SCI_INT_CAUSE_EXTRA_RX_Msk
#define SCI_INT_CAUSE_RESEND_OVFL_Pos   (4U)
#define SCI_INT_CAUSE_RESEND_OVFL_Msk   (0x1UL << SCI_INT_CAUSE_RESEND_OVFL_Pos)
#define SCI_INT_CAUSE_RESEND_OVFL       SCI_INT_CAUSE_RESEND_OVFL_Msk
#define SCI_INT_CAUSE_ARG_END_Pos       (5U)
#define SCI_INT_CAUSE_ARG_END_Msk       (0x1UL << SCI_INT_CAUSE_ARG_END_Pos)
#define SCI_INT_CAUSE_ARG_END           SCI_INT_CAUSE_ARG_END_Msk
#define SCI_INT_CAUSE_SCI_DMA_TX_DONE_Pos  (6U)
#define SCI_INT_CAUSE_SCI_DMA_TX_DONE_Msk  (0x1UL << SCI_INT_CAUSE_SCI_DMA_TX_DONE_Pos)
#define SCI_INT_CAUSE_SCI_DMA_TX_DONE   SCI_INT_CAUSE_SCI_DMA_TX_DONE_Msk
#define SCI_INT_CAUSE_SCI_DMA_RX_DONE_Pos  (7U)
#define SCI_INT_CAUSE_SCI_DMA_RX_DONE_Msk  (0x1UL << SCI_INT_CAUSE_SCI_DMA_RX_DONE_Pos)
#define SCI_INT_CAUSE_SCI_DMA_RX_DONE   SCI_INT_CAUSE_SCI_DMA_RX_DONE_Msk
#define SCI_INT_CAUSE_RX_DONE_R_Pos     (16U)
#define SCI_INT_CAUSE_RX_DONE_R_Msk     (0x1UL << SCI_INT_CAUSE_RX_DONE_R_Pos)
#define SCI_INT_CAUSE_RX_DONE_R         SCI_INT_CAUSE_RX_DONE_R_Msk
#define SCI_INT_CAUSE_RX_HALF_R_Pos     (17U)
#define SCI_INT_CAUSE_RX_HALF_R_Msk     (0x1UL << SCI_INT_CAUSE_RX_HALF_R_Pos)
#define SCI_INT_CAUSE_RX_HALF_R         SCI_INT_CAUSE_RX_HALF_R_Msk
#define SCI_INT_CAUSE_WWT_TIMEOUT_R_Pos  (18U)
#define SCI_INT_CAUSE_WWT_TIMEOUT_R_Msk  (0x1UL << SCI_INT_CAUSE_WWT_TIMEOUT_R_Pos)
#define SCI_INT_CAUSE_WWT_TIMEOUT_R     SCI_INT_CAUSE_WWT_TIMEOUT_R_Msk
#define SCI_INT_CAUSE_EXTRA_RX_R_Pos    (19U)
#define SCI_INT_CAUSE_EXTRA_RX_R_Msk    (0x1UL << SCI_INT_CAUSE_EXTRA_RX_R_Pos)
#define SCI_INT_CAUSE_EXTRA_RX_R        SCI_INT_CAUSE_EXTRA_RX_R_Msk
#define SCI_INT_CAUSE_RESEND_OVFL_R_Pos  (20U)
#define SCI_INT_CAUSE_RESEND_OVFL_R_Msk  (0x1UL << SCI_INT_CAUSE_RESEND_OVFL_R_Pos)
#define SCI_INT_CAUSE_RESEND_OVFL_R     SCI_INT_CAUSE_RESEND_OVFL_R_Msk
#define SCI_INT_CAUSE_ARG_END_R_Pos     (21U)
#define SCI_INT_CAUSE_ARG_END_R_Msk     (0x1UL << SCI_INT_CAUSE_ARG_END_R_Pos)
#define SCI_INT_CAUSE_ARG_END_R         SCI_INT_CAUSE_ARG_END_R_Msk
#define SCI_INT_CAUSE_SCI_DMA_TX_DONE_R_Pos  (22U)
#define SCI_INT_CAUSE_SCI_DMA_TX_DONE_R_Msk  (0x1UL << SCI_INT_CAUSE_SCI_DMA_TX_DONE_R_Pos)
#define SCI_INT_CAUSE_SCI_DMA_TX_DONE_R  SCI_INT_CAUSE_SCI_DMA_TX_DONE_R_Msk
#define SCI_INT_CAUSE_SCI_DMA_RX_DONE_R_Pos  (23U)
#define SCI_INT_CAUSE_SCI_DMA_RX_DONE_R_Msk  (0x1UL << SCI_INT_CAUSE_SCI_DMA_RX_DONE_R_Pos)
#define SCI_INT_CAUSE_SCI_DMA_RX_DONE_R  SCI_INT_CAUSE_SCI_DMA_RX_DONE_R_Msk

/****************** Bit definition for SCI_INT_CLR register *******************/
#define SCI_INT_CLR_RX_DONE_Pos         (0U)
#define SCI_INT_CLR_RX_DONE_Msk         (0x1UL << SCI_INT_CLR_RX_DONE_Pos)
#define SCI_INT_CLR_RX_DONE             SCI_INT_CLR_RX_DONE_Msk
#define SCI_INT_CLR_RX_HALF_Pos         (1U)
#define SCI_INT_CLR_RX_HALF_Msk         (0x1UL << SCI_INT_CLR_RX_HALF_Pos)
#define SCI_INT_CLR_RX_HALF             SCI_INT_CLR_RX_HALF_Msk
#define SCI_INT_CLR_WWT_TIMEOUT_Pos     (2U)
#define SCI_INT_CLR_WWT_TIMEOUT_Msk     (0x1UL << SCI_INT_CLR_WWT_TIMEOUT_Pos)
#define SCI_INT_CLR_WWT_TIMEOUT         SCI_INT_CLR_WWT_TIMEOUT_Msk
#define SCI_INT_CLR_EXTRA_RX_Pos        (3U)
#define SCI_INT_CLR_EXTRA_RX_Msk        (0x1UL << SCI_INT_CLR_EXTRA_RX_Pos)
#define SCI_INT_CLR_EXTRA_RX            SCI_INT_CLR_EXTRA_RX_Msk
#define SCI_INT_CLR_RESEND_OVFL_Pos     (4U)
#define SCI_INT_CLR_RESEND_OVFL_Msk     (0x1UL << SCI_INT_CLR_RESEND_OVFL_Pos)
#define SCI_INT_CLR_RESEND_OVFL         SCI_INT_CLR_RESEND_OVFL_Msk
#define SCI_INT_CLR_ARG_END_Pos         (5U)
#define SCI_INT_CLR_ARG_END_Msk         (0x1UL << SCI_INT_CLR_ARG_END_Pos)
#define SCI_INT_CLR_ARG_END             SCI_INT_CLR_ARG_END_Msk
#define SCI_INT_CLR_SCI_DMA_TX_DONE_Pos  (6U)
#define SCI_INT_CLR_SCI_DMA_TX_DONE_Msk  (0x1UL << SCI_INT_CLR_SCI_DMA_TX_DONE_Pos)
#define SCI_INT_CLR_SCI_DMA_TX_DONE     SCI_INT_CLR_SCI_DMA_TX_DONE_Msk
#define SCI_INT_CLR_SCI_DMA_RX_DONE_Pos  (7U)
#define SCI_INT_CLR_SCI_DMA_RX_DONE_Msk  (0x1UL << SCI_INT_CLR_SCI_DMA_RX_DONE_Pos)
#define SCI_INT_CLR_SCI_DMA_RX_DONE     SCI_INT_CLR_SCI_DMA_RX_DONE_Msk

/****************** Bit definition for SCI_INT_MASK register ******************/
#define SCI_INT_MASK_RX_DONE_Pos        (0U)
#define SCI_INT_MASK_RX_DONE_Msk        (0x1UL << SCI_INT_MASK_RX_DONE_Pos)
#define SCI_INT_MASK_RX_DONE            SCI_INT_MASK_RX_DONE_Msk
#define SCI_INT_MASK_RX_HALF_Pos        (1U)
#define SCI_INT_MASK_RX_HALF_Msk        (0x1UL << SCI_INT_MASK_RX_HALF_Pos)
#define SCI_INT_MASK_RX_HALF            SCI_INT_MASK_RX_HALF_Msk
#define SCI_INT_MASK_WWT_TIMEOUT_Pos    (2U)
#define SCI_INT_MASK_WWT_TIMEOUT_Msk    (0x1UL << SCI_INT_MASK_WWT_TIMEOUT_Pos)
#define SCI_INT_MASK_WWT_TIMEOUT        SCI_INT_MASK_WWT_TIMEOUT_Msk
#define SCI_INT_MASK_EXTRA_RX_Pos       (3U)
#define SCI_INT_MASK_EXTRA_RX_Msk       (0x1UL << SCI_INT_MASK_EXTRA_RX_Pos)
#define SCI_INT_MASK_EXTRA_RX           SCI_INT_MASK_EXTRA_RX_Msk
#define SCI_INT_MASK_RESEND_OVFL_Pos    (4U)
#define SCI_INT_MASK_RESEND_OVFL_Msk    (0x1UL << SCI_INT_MASK_RESEND_OVFL_Pos)
#define SCI_INT_MASK_RESEND_OVFL        SCI_INT_MASK_RESEND_OVFL_Msk
#define SCI_INT_MASK_ARG_END_Pos        (5U)
#define SCI_INT_MASK_ARG_END_Msk        (0x1UL << SCI_INT_MASK_ARG_END_Pos)
#define SCI_INT_MASK_ARG_END            SCI_INT_MASK_ARG_END_Msk
#define SCI_INT_MASK_SCI_DMA_TX_DONE_Pos  (6U)
#define SCI_INT_MASK_SCI_DMA_TX_DONE_Msk  (0x1UL << SCI_INT_MASK_SCI_DMA_TX_DONE_Pos)
#define SCI_INT_MASK_SCI_DMA_TX_DONE    SCI_INT_MASK_SCI_DMA_TX_DONE_Msk
#define SCI_INT_MASK_SCI_DMA_RX_DONE_Pos  (7U)
#define SCI_INT_MASK_SCI_DMA_RX_DONE_Msk  (0x1UL << SCI_INT_MASK_SCI_DMA_RX_DONE_Pos)
#define SCI_INT_MASK_SCI_DMA_RX_DONE    SCI_INT_MASK_SCI_DMA_RX_DONE_Msk

/***************** Bit definition for SCI_MISC_CTRL register ******************/
#define SCI_MISC_CTRL_PA_CLK_EN_Pos     (0U)
#define SCI_MISC_CTRL_PA_CLK_EN_Msk     (0x1UL << SCI_MISC_CTRL_PA_CLK_EN_Pos)
#define SCI_MISC_CTRL_PA_CLK_EN         SCI_MISC_CTRL_PA_CLK_EN_Msk
#define SCI_MISC_CTRL_TX_DMA_EN_Pos     (1U)
#define SCI_MISC_CTRL_TX_DMA_EN_Msk     (0x1UL << SCI_MISC_CTRL_TX_DMA_EN_Pos)
#define SCI_MISC_CTRL_TX_DMA_EN         SCI_MISC_CTRL_TX_DMA_EN_Msk
#define SCI_MISC_CTRL_RX_DMA_EN_Pos     (2U)
#define SCI_MISC_CTRL_RX_DMA_EN_Msk     (0x1UL << SCI_MISC_CTRL_RX_DMA_EN_Pos)
#define SCI_MISC_CTRL_RX_DMA_EN         SCI_MISC_CTRL_RX_DMA_EN_Msk

/***************** Bit definition for SCI_PA_STATUS register ******************/
#define SCI_PA_STATUS_ENABLE_Pos        (0U)
#define SCI_PA_STATUS_ENABLE_Msk        (0x1UL << SCI_PA_STATUS_ENABLE_Pos)
#define SCI_PA_STATUS_ENABLE            SCI_PA_STATUS_ENABLE_Msk

#endif
