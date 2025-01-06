#ifndef __SD_H
#define __SD_H

typedef struct
{
    __IO uint32_t SR;
    __IO uint32_t CCR;
    __IO uint32_t CAR;
    __IO uint32_t RIR;
    __IO uint32_t RAR1;
    __IO uint32_t RAR2;
    __IO uint32_t RAR3;
    __IO uint32_t RAR4;
    __IO uint32_t TOR;
    __IO uint32_t DCR;
    __IO uint32_t DLR;
    __IO uint32_t IER;
    __IO uint32_t CLKCR;
    __IO uint32_t PCR;
    __IO uint32_t RSVD2;
    __IO uint32_t CDR;
    __IO uint32_t DBGR1;
    __IO uint32_t DBGR2;
    __IO uint32_t CEATA;
    __IO uint32_t BLR;
    __IO uint32_t FDLR;
    __IO uint32_t DSR;
    __IO uint32_t CDCR;
    __IO uint32_t CASR;
    __IO uint32_t CACR;
    __IO uint32_t CACNT;
    __IO uint32_t CAOFF;
    __IO uint32_t RSVD1[101];
    __IO uint32_t FIFO;
} SD_TypeDef;


/********************* Bit definition for SD_SR register **********************/
#define SD_SR_CMD_BUSY_Pos              (0U)
#define SD_SR_CMD_BUSY_Msk              (0x1UL << SD_SR_CMD_BUSY_Pos)
#define SD_SR_CMD_BUSY                  SD_SR_CMD_BUSY_Msk
#define SD_SR_CMD_DONE_Pos              (1U)
#define SD_SR_CMD_DONE_Msk              (0x1UL << SD_SR_CMD_DONE_Pos)
#define SD_SR_CMD_DONE                  SD_SR_CMD_DONE_Msk
#define SD_SR_CMD_RSP_CRC_Pos           (2U)
#define SD_SR_CMD_RSP_CRC_Msk           (0x1UL << SD_SR_CMD_RSP_CRC_Pos)
#define SD_SR_CMD_RSP_CRC               SD_SR_CMD_RSP_CRC_Msk
#define SD_SR_CMD_TIMEOUT_Pos           (3U)
#define SD_SR_CMD_TIMEOUT_Msk           (0x1UL << SD_SR_CMD_TIMEOUT_Pos)
#define SD_SR_CMD_TIMEOUT               SD_SR_CMD_TIMEOUT_Msk
#define SD_SR_DATA_BUSY_Pos             (4U)
#define SD_SR_DATA_BUSY_Msk             (0x1UL << SD_SR_DATA_BUSY_Pos)
#define SD_SR_DATA_BUSY                 SD_SR_DATA_BUSY_Msk
#define SD_SR_DATA_DONE_Pos             (5U)
#define SD_SR_DATA_DONE_Msk             (0x1UL << SD_SR_DATA_DONE_Pos)
#define SD_SR_DATA_DONE                 SD_SR_DATA_DONE_Msk
#define SD_SR_DATA_CRC_Pos              (6U)
#define SD_SR_DATA_CRC_Msk              (0x1UL << SD_SR_DATA_CRC_Pos)
#define SD_SR_DATA_CRC                  SD_SR_DATA_CRC_Msk
#define SD_SR_DATA_TIMEOUT_Pos          (7U)
#define SD_SR_DATA_TIMEOUT_Msk          (0x1UL << SD_SR_DATA_TIMEOUT_Pos)
#define SD_SR_DATA_TIMEOUT              SD_SR_DATA_TIMEOUT_Msk
#define SD_SR_STARTBIT_ERROR_Pos        (8U)
#define SD_SR_STARTBIT_ERROR_Msk        (0x1UL << SD_SR_STARTBIT_ERROR_Pos)
#define SD_SR_STARTBIT_ERROR            SD_SR_STARTBIT_ERROR_Msk
#define SD_SR_FIFO_UNDERRUN_Pos         (9U)
#define SD_SR_FIFO_UNDERRUN_Msk         (0x1UL << SD_SR_FIFO_UNDERRUN_Pos)
#define SD_SR_FIFO_UNDERRUN             SD_SR_FIFO_UNDERRUN_Msk
#define SD_SR_FIFO_OVERRUN_Pos          (10U)
#define SD_SR_FIFO_OVERRUN_Msk          (0x1UL << SD_SR_FIFO_OVERRUN_Pos)
#define SD_SR_FIFO_OVERRUN              SD_SR_FIFO_OVERRUN_Msk
#define SD_SR_CMD_SENT_Pos              (12U)
#define SD_SR_CMD_SENT_Msk              (0x1UL << SD_SR_CMD_SENT_Pos)
#define SD_SR_CMD_SENT                  SD_SR_CMD_SENT_Msk
#define SD_SR_CARD_EXIT_INT_Pos         (13U)
#define SD_SR_CARD_EXIT_INT_Msk         (0x1UL << SD_SR_CARD_EXIT_INT_Pos)
#define SD_SR_CARD_EXIT_INT             SD_SR_CARD_EXIT_INT_Msk
#define SD_SR_CARD_REMOVE_Pos           (14U)
#define SD_SR_CARD_REMOVE_Msk           (0x1UL << SD_SR_CARD_REMOVE_Pos)
#define SD_SR_CARD_REMOVE               SD_SR_CARD_REMOVE_Msk
#define SD_SR_CARD_EXIT_Pos             (15U)
#define SD_SR_CARD_EXIT_Msk             (0x1UL << SD_SR_CARD_EXIT_Pos)
#define SD_SR_CARD_EXIT                 SD_SR_CARD_EXIT_Msk
#define SD_SR_SDIO_Pos                  (16U)
#define SD_SR_SDIO_Msk                  (0x1UL << SD_SR_SDIO_Pos)
#define SD_SR_SDIO                      SD_SR_SDIO_Msk
#define SD_SR_CACHE_ERR_Pos             (17U)
#define SD_SR_CACHE_ERR_Msk             (0x1UL << SD_SR_CACHE_ERR_Pos)
#define SD_SR_CACHE_ERR                 SD_SR_CACHE_ERR_Msk

/********************* Bit definition for SD_CCR register *********************/
#define SD_CCR_CMD_START_Pos            (0U)
#define SD_CCR_CMD_START_Msk            (0x1UL << SD_CCR_CMD_START_Pos)
#define SD_CCR_CMD_START                SD_CCR_CMD_START_Msk
#define SD_CCR_CMD_TX_EN_Pos            (8U)
#define SD_CCR_CMD_TX_EN_Msk            (0x1UL << SD_CCR_CMD_TX_EN_Pos)
#define SD_CCR_CMD_TX_EN                SD_CCR_CMD_TX_EN_Msk
#define SD_CCR_CMD_PEND_Pos             (9U)
#define SD_CCR_CMD_PEND_Msk             (0x1UL << SD_CCR_CMD_PEND_Pos)
#define SD_CCR_CMD_PEND                 SD_CCR_CMD_PEND_Msk
#define SD_CCR_CMD_HAS_RSP_Pos          (16U)
#define SD_CCR_CMD_HAS_RSP_Msk          (0x1UL << SD_CCR_CMD_HAS_RSP_Pos)
#define SD_CCR_CMD_HAS_RSP              SD_CCR_CMD_HAS_RSP_Msk
#define SD_CCR_CMD_LONG_RSP_Pos         (17U)
#define SD_CCR_CMD_LONG_RSP_Msk         (0x1UL << SD_CCR_CMD_LONG_RSP_Pos)
#define SD_CCR_CMD_LONG_RSP             SD_CCR_CMD_LONG_RSP_Msk
#define SD_CCR_CMD_INDEX_Pos            (18U)
#define SD_CCR_CMD_INDEX_Msk            (0x3FUL << SD_CCR_CMD_INDEX_Pos)
#define SD_CCR_CMD_INDEX                SD_CCR_CMD_INDEX_Msk
#define SD_CCR_CCS_DISABLE_Pos          (24U)
#define SD_CCR_CCS_DISABLE_Msk          (0x1UL << SD_CCR_CCS_DISABLE_Pos)
#define SD_CCR_CCS_DISABLE              SD_CCR_CCS_DISABLE_Msk

/********************* Bit definition for SD_CAR register *********************/
#define SD_CAR_CMD_ARG_Pos              (0U)
#define SD_CAR_CMD_ARG_Msk              (0xFFFFFFFFUL << SD_CAR_CMD_ARG_Pos)
#define SD_CAR_CMD_ARG                  SD_CAR_CMD_ARG_Msk

/********************* Bit definition for SD_RIR register *********************/
#define SD_RIR_RSP_INDEX_Pos            (0U)
#define SD_RIR_RSP_INDEX_Msk            (0x3FUL << SD_RIR_RSP_INDEX_Pos)
#define SD_RIR_RSP_INDEX                SD_RIR_RSP_INDEX_Msk

/******************** Bit definition for SD_RAR1 register *********************/
#define SD_RAR1_RSP_ARG1_Pos            (0U)
#define SD_RAR1_RSP_ARG1_Msk            (0xFFFFFFFFUL << SD_RAR1_RSP_ARG1_Pos)
#define SD_RAR1_RSP_ARG1                SD_RAR1_RSP_ARG1_Msk

/******************** Bit definition for SD_RAR2 register *********************/
#define SD_RAR2_RSP_ARG2_Pos            (0U)
#define SD_RAR2_RSP_ARG2_Msk            (0xFFFFFFFFUL << SD_RAR2_RSP_ARG2_Pos)
#define SD_RAR2_RSP_ARG2                SD_RAR2_RSP_ARG2_Msk

/******************** Bit definition for SD_RAR3 register *********************/
#define SD_RAR3_RSP_ARG3_Pos            (0U)
#define SD_RAR3_RSP_ARG3_Msk            (0xFFFFFFFFUL << SD_RAR3_RSP_ARG3_Pos)
#define SD_RAR3_RSP_ARG3                SD_RAR3_RSP_ARG3_Msk

/******************** Bit definition for SD_RAR4 register *********************/
#define SD_RAR4_RSP_ARG4_Pos            (0U)
#define SD_RAR4_RSP_ARG4_Msk            (0xFFFFFFUL << SD_RAR4_RSP_ARG4_Pos)
#define SD_RAR4_RSP_ARG4                SD_RAR4_RSP_ARG4_Msk

/********************* Bit definition for SD_TOR register *********************/
#define SD_TOR_TIMEOUT_CNT_Pos          (0U)
#define SD_TOR_TIMEOUT_CNT_Msk          (0xFFFFFFFFUL << SD_TOR_TIMEOUT_CNT_Pos)
#define SD_TOR_TIMEOUT_CNT              SD_TOR_TIMEOUT_CNT_Msk

/********************* Bit definition for SD_DCR register *********************/
#define SD_DCR_DATA_START_Pos           (0U)
#define SD_DCR_DATA_START_Msk           (0x1UL << SD_DCR_DATA_START_Pos)
#define SD_DCR_DATA_START               SD_DCR_DATA_START_Msk
#define SD_DCR_TRAN_DATA_EN_Pos         (8U)
#define SD_DCR_TRAN_DATA_EN_Msk         (0x1UL << SD_DCR_TRAN_DATA_EN_Pos)
#define SD_DCR_TRAN_DATA_EN             SD_DCR_TRAN_DATA_EN_Msk
#define SD_DCR_R_WN_Pos                 (9U)
#define SD_DCR_R_WN_Msk                 (0x1UL << SD_DCR_R_WN_Pos)
#define SD_DCR_R_WN                     SD_DCR_R_WN_Msk
#define SD_DCR_STREAM_MODE_Pos          (10U)
#define SD_DCR_STREAM_MODE_Msk          (0x1UL << SD_DCR_STREAM_MODE_Pos)
#define SD_DCR_STREAM_MODE              SD_DCR_STREAM_MODE_Msk
#define SD_DCR_WIRE_MODE_Pos            (11U)
#define SD_DCR_WIRE_MODE_Msk            (0x3UL << SD_DCR_WIRE_MODE_Pos)
#define SD_DCR_WIRE_MODE                SD_DCR_WIRE_MODE_Msk
#define SD_DCR_BLOCK_SIZE_Pos           (16U)
#define SD_DCR_BLOCK_SIZE_Msk           (0x7FFUL << SD_DCR_BLOCK_SIZE_Pos)
#define SD_DCR_BLOCK_SIZE               SD_DCR_BLOCK_SIZE_Msk

/********************* Bit definition for SD_DLR register *********************/
#define SD_DLR_DATA_LEN_Pos             (0U)
#define SD_DLR_DATA_LEN_Msk             (0xFFFFUL << SD_DLR_DATA_LEN_Pos)
#define SD_DLR_DATA_LEN                 SD_DLR_DATA_LEN_Msk
#define SD_DLR_BLOCK_TRAN_NUM_Pos       (16U)
#define SD_DLR_BLOCK_TRAN_NUM_Msk       (0xFFFFUL << SD_DLR_BLOCK_TRAN_NUM_Pos)
#define SD_DLR_BLOCK_TRAN_NUM           SD_DLR_BLOCK_TRAN_NUM_Msk

/********************* Bit definition for SD_IER register *********************/
#define SD_IER_CMD_DONE_MASK_Pos        (1U)
#define SD_IER_CMD_DONE_MASK_Msk        (0x1UL << SD_IER_CMD_DONE_MASK_Pos)
#define SD_IER_CMD_DONE_MASK            SD_IER_CMD_DONE_MASK_Msk
#define SD_IER_CMD_RSP_CRC_MASK_Pos     (2U)
#define SD_IER_CMD_RSP_CRC_MASK_Msk     (0x1UL << SD_IER_CMD_RSP_CRC_MASK_Pos)
#define SD_IER_CMD_RSP_CRC_MASK         SD_IER_CMD_RSP_CRC_MASK_Msk
#define SD_IER_CMD_TIMEOUT_MASK_Pos     (3U)
#define SD_IER_CMD_TIMEOUT_MASK_Msk     (0x1UL << SD_IER_CMD_TIMEOUT_MASK_Pos)
#define SD_IER_CMD_TIMEOUT_MASK         SD_IER_CMD_TIMEOUT_MASK_Msk
#define SD_IER_DATA_DONE_MASK_Pos       (5U)
#define SD_IER_DATA_DONE_MASK_Msk       (0x1UL << SD_IER_DATA_DONE_MASK_Pos)
#define SD_IER_DATA_DONE_MASK           SD_IER_DATA_DONE_MASK_Msk
#define SD_IER_DATA_CRC_MASK_Pos        (6U)
#define SD_IER_DATA_CRC_MASK_Msk        (0x1UL << SD_IER_DATA_CRC_MASK_Pos)
#define SD_IER_DATA_CRC_MASK            SD_IER_DATA_CRC_MASK_Msk
#define SD_IER_DATA_TIMEOUT_MASK_Pos    (7U)
#define SD_IER_DATA_TIMEOUT_MASK_Msk    (0x1UL << SD_IER_DATA_TIMEOUT_MASK_Pos)
#define SD_IER_DATA_TIMEOUT_MASK        SD_IER_DATA_TIMEOUT_MASK_Msk
#define SD_IER_STARTBIT_ERROR_MASK_Pos  (8U)
#define SD_IER_STARTBIT_ERROR_MASK_Msk  (0x1UL << SD_IER_STARTBIT_ERROR_MASK_Pos)
#define SD_IER_STARTBIT_ERROR_MASK      SD_IER_STARTBIT_ERROR_MASK_Msk
#define SD_IER_FIFO_UNDERRUN_MASK_Pos   (9U)
#define SD_IER_FIFO_UNDERRUN_MASK_Msk   (0x1UL << SD_IER_FIFO_UNDERRUN_MASK_Pos)
#define SD_IER_FIFO_UNDERRUN_MASK       SD_IER_FIFO_UNDERRUN_MASK_Msk
#define SD_IER_FIFO_OVERRUN_MASK_Pos    (10U)
#define SD_IER_FIFO_OVERRUN_MASK_Msk    (0x1UL << SD_IER_FIFO_OVERRUN_MASK_Pos)
#define SD_IER_FIFO_OVERRUN_MASK        SD_IER_FIFO_OVERRUN_MASK_Msk
#define SD_IER_CMD_SENT_MASK_Pos        (12U)
#define SD_IER_CMD_SENT_MASK_Msk        (0x1UL << SD_IER_CMD_SENT_MASK_Pos)
#define SD_IER_CMD_SENT_MASK            SD_IER_CMD_SENT_MASK_Msk
#define SD_IER_CARD_EXIT_INT_MASK_Pos   (13U)
#define SD_IER_CARD_EXIT_INT_MASK_Msk   (0x1UL << SD_IER_CARD_EXIT_INT_MASK_Pos)
#define SD_IER_CARD_EXIT_INT_MASK       SD_IER_CARD_EXIT_INT_MASK_Msk
#define SD_IER_CARD_REMOVE_INT_MASK_Pos  (14U)
#define SD_IER_CARD_REMOVE_INT_MASK_Msk  (0x1UL << SD_IER_CARD_REMOVE_INT_MASK_Pos)
#define SD_IER_CARD_REMOVE_INT_MASK     SD_IER_CARD_REMOVE_INT_MASK_Msk
#define SD_IER_SDIO_MASK_Pos            (16U)
#define SD_IER_SDIO_MASK_Msk            (0x1UL << SD_IER_SDIO_MASK_Pos)
#define SD_IER_SDIO_MASK                SD_IER_SDIO_MASK_Msk
#define SD_IER_CACHE_ERR_INT_MASK_Pos   (17U)
#define SD_IER_CACHE_ERR_INT_MASK_Msk   (0x1UL << SD_IER_CACHE_ERR_INT_MASK_Pos)
#define SD_IER_CACHE_ERR_INT_MASK       SD_IER_CACHE_ERR_INT_MASK_Msk

/******************** Bit definition for SD_CLKCR register ********************/
#define SD_CLKCR_STOP_CLK_Pos           (0U)
#define SD_CLKCR_STOP_CLK_Msk           (0x1UL << SD_CLKCR_STOP_CLK_Pos)
#define SD_CLKCR_STOP_CLK               SD_CLKCR_STOP_CLK_Msk
#define SD_CLKCR_VOID_FIFO_ERROR_Pos    (1U)
#define SD_CLKCR_VOID_FIFO_ERROR_Msk    (0x1UL << SD_CLKCR_VOID_FIFO_ERROR_Pos)
#define SD_CLKCR_VOID_FIFO_ERROR        SD_CLKCR_VOID_FIFO_ERROR_Msk
#define SD_CLKCR_CLK_TUNE_SEL_Pos       (2U)
#define SD_CLKCR_CLK_TUNE_SEL_Msk       (0x3UL << SD_CLKCR_CLK_TUNE_SEL_Pos)
#define SD_CLKCR_CLK_TUNE_SEL           SD_CLKCR_CLK_TUNE_SEL_Msk
#define SD_CLKCR_DIV_Pos                (8U)
#define SD_CLKCR_DIV_Msk                (0x1FFFUL << SD_CLKCR_DIV_Pos)
#define SD_CLKCR_DIV                    SD_CLKCR_DIV_Msk

/********************* Bit definition for SD_PCR register *********************/
#define SD_PCR_PWR_Pos                  (0U)
#define SD_PCR_PWR_Msk                  (0x3UL << SD_PCR_PWR_Pos)
#define SD_PCR_PWR                      SD_PCR_PWR_Msk
#define SD_PCR_VOLTAGE_Pos              (2U)
#define SD_PCR_VOLTAGE_Msk              (0xFFUL << SD_PCR_VOLTAGE_Pos)
#define SD_PCR_VOLTAGE                  SD_PCR_VOLTAGE_Msk
#define SD_PCR_ODRAIN_Pos               (10U)
#define SD_PCR_ODRAIN_Msk               (0x1UL << SD_PCR_ODRAIN_Pos)
#define SD_PCR_ODRAIN                   SD_PCR_ODRAIN_Msk
#define SD_PCR_ROD_Pos                  (11U)
#define SD_PCR_ROD_Msk                  (0x1UL << SD_PCR_ROD_Pos)
#define SD_PCR_ROD                      SD_PCR_ROD_Msk

/********************* Bit definition for SD_CDR register *********************/
#define SD_CDR_SD_DATA3_CD_Pos          (0U)
#define SD_CDR_SD_DATA3_CD_Msk          (0x1UL << SD_CDR_SD_DATA3_CD_Pos)
#define SD_CDR_SD_DATA3_CD              SD_CDR_SD_DATA3_CD_Msk
#define SD_CDR_ITIMING_SEL_Pos          (1U)
#define SD_CDR_ITIMING_SEL_Msk          (0x1UL << SD_CDR_ITIMING_SEL_Pos)
#define SD_CDR_ITIMING_SEL              SD_CDR_ITIMING_SEL_Msk
#define SD_CDR_OTIMING_SEL_Pos          (2U)
#define SD_CDR_OTIMING_SEL_Msk          (0x1UL << SD_CDR_OTIMING_SEL_Pos)
#define SD_CDR_OTIMING_SEL              SD_CDR_OTIMING_SEL_Msk
#define SD_CDR_EN_CD_Pos                (3U)
#define SD_CDR_EN_CD_Msk                (0x1UL << SD_CDR_EN_CD_Pos)
#define SD_CDR_EN_CD                    SD_CDR_EN_CD_Msk
#define SD_CDR_CD_HVALID_Pos            (4U)
#define SD_CDR_CD_HVALID_Msk            (0x1UL << SD_CDR_CD_HVALID_Pos)
#define SD_CDR_CD_HVALID                SD_CDR_CD_HVALID_Msk
#define SD_CDR_CMD_OD_Pos               (5U)
#define SD_CDR_CMD_OD_Msk               (0x1UL << SD_CDR_CMD_OD_Pos)
#define SD_CDR_CMD_OD                   SD_CDR_CMD_OD_Msk
#define SD_CDR_ITIMING_Pos              (6U)
#define SD_CDR_ITIMING_Msk              (0x1FFFUL << SD_CDR_ITIMING_Pos)
#define SD_CDR_ITIMING                  SD_CDR_ITIMING_Msk
#define SD_CDR_OTIMING_Pos              (19U)
#define SD_CDR_OTIMING_Msk              (0x1FFFUL << SD_CDR_OTIMING_Pos)
#define SD_CDR_OTIMING                  SD_CDR_OTIMING_Msk

/******************** Bit definition for SD_DBGR1 register ********************/
#define SD_DBGR1_CMD_ST_Pos             (0U)
#define SD_DBGR1_CMD_ST_Msk             (0xFFFFUL << SD_DBGR1_CMD_ST_Pos)
#define SD_DBGR1_CMD_ST                 SD_DBGR1_CMD_ST_Msk
#define SD_DBGR1_DATA_ST_Pos            (16U)
#define SD_DBGR1_DATA_ST_Msk            (0x7FFFUL << SD_DBGR1_DATA_ST_Pos)
#define SD_DBGR1_DATA_ST                SD_DBGR1_DATA_ST_Msk

/******************** Bit definition for SD_DBGR2 register ********************/
#define SD_DBGR2_HOST_WORD_COUNTER_Pos  (0U)
#define SD_DBGR2_HOST_WORD_COUNTER_Msk  (0x3FFFUL << SD_DBGR2_HOST_WORD_COUNTER_Pos)
#define SD_DBGR2_HOST_WORD_COUNTER      SD_DBGR2_HOST_WORD_COUNTER_Msk
#define SD_DBGR2_VALID_DATA_COU_Pos     (16U)
#define SD_DBGR2_VALID_DATA_COU_Msk     (0x3FFUL << SD_DBGR2_VALID_DATA_COU_Pos)
#define SD_DBGR2_VALID_DATA_COU         SD_DBGR2_VALID_DATA_COU_Msk
#define SD_DBGR2_DBG_SEL_Pos            (30U)
#define SD_DBGR2_DBG_SEL_Msk            (0x3UL << SD_DBGR2_DBG_SEL_Pos)
#define SD_DBGR2_DBG_SEL                SD_DBGR2_DBG_SEL_Msk

/******************** Bit definition for SD_CEATA register ********************/
#define SD_CEATA_ATA_MODE_Pos           (0U)
#define SD_CEATA_ATA_MODE_Msk           (0x1UL << SD_CEATA_ATA_MODE_Pos)
#define SD_CEATA_ATA_MODE               SD_CEATA_ATA_MODE_Msk
#define SD_CEATA_ENABLE_SDIO_IRQ_Pos    (1U)
#define SD_CEATA_ENABLE_SDIO_IRQ_Msk    (0x1UL << SD_CEATA_ENABLE_SDIO_IRQ_Pos)
#define SD_CEATA_ENABLE_SDIO_IRQ        SD_CEATA_ENABLE_SDIO_IRQ_Msk
#define SD_CEATA_SDIO_4WIRES_IRQ_Pos    (2U)
#define SD_CEATA_SDIO_4WIRES_IRQ_Msk    (0x1UL << SD_CEATA_SDIO_4WIRES_IRQ_Pos)
#define SD_CEATA_SDIO_4WIRES_IRQ        SD_CEATA_SDIO_4WIRES_IRQ_Msk
#define SD_CEATA_SDIO_4WIRES_MULTI_IRQ_Pos  (3U)
#define SD_CEATA_SDIO_4WIRES_MULTI_IRQ_Msk  (0x1UL << SD_CEATA_SDIO_4WIRES_MULTI_IRQ_Pos)
#define SD_CEATA_SDIO_4WIRES_MULTI_IRQ  SD_CEATA_SDIO_4WIRES_MULTI_IRQ_Msk

/********************* Bit definition for SD_BLR register *********************/
#define SD_BLR_BURST_LEN_Pos            (0U)
#define SD_BLR_BURST_LEN_Msk            (0x3FFFUL << SD_BLR_BURST_LEN_Pos)
#define SD_BLR_BURST_LEN                SD_BLR_BURST_LEN_Msk

/******************** Bit definition for SD_FDLR register *********************/
#define SD_FDLR_RD_VALID_Pos            (0U)
#define SD_FDLR_RD_VALID_Msk            (0x3FFUL << SD_FDLR_RD_VALID_Pos)
#define SD_FDLR_RD_VALID                SD_FDLR_RD_VALID_Msk
#define SD_FDLR_RD_VALID_EN_Pos         (12U)
#define SD_FDLR_RD_VALID_EN_Msk         (0x1UL << SD_FDLR_RD_VALID_EN_Pos)
#define SD_FDLR_RD_VALID_EN             SD_FDLR_RD_VALID_EN_Msk

/********************* Bit definition for SD_DSR register *********************/
#define SD_DSR_SD_DATA_I_LL_Pos         (0U)
#define SD_DSR_SD_DATA_I_LL_Msk         (0xFFUL << SD_DSR_SD_DATA_I_LL_Pos)
#define SD_DSR_SD_DATA_I_LL             SD_DSR_SD_DATA_I_LL_Msk

/******************** Bit definition for SD_CDCR register *********************/
#define SD_CDCR_CLK_CONFIG_Pos          (0U)
#define SD_CDCR_CLK_CONFIG_Msk          (0x1UL << SD_CDCR_CLK_CONFIG_Pos)
#define SD_CDCR_CLK_CONFIG              SD_CDCR_CLK_CONFIG_Msk

/******************** Bit definition for SD_CASR register *********************/
#define SD_CASR_SD_REQ_Pos              (0U)
#define SD_CASR_SD_REQ_Msk              (0x1UL << SD_CASR_SD_REQ_Pos)
#define SD_CASR_SD_REQ                  SD_CASR_SD_REQ_Msk
#define SD_CASR_SD_BUSY_Pos             (1U)
#define SD_CASR_SD_BUSY_Msk             (0x1UL << SD_CASR_SD_BUSY_Pos)
#define SD_CASR_SD_BUSY                 SD_CASR_SD_BUSY_Msk
#define SD_CASR_CACHE_BUSY_Pos          (2U)
#define SD_CASR_CACHE_BUSY_Msk          (0x1UL << SD_CASR_CACHE_BUSY_Pos)
#define SD_CASR_CACHE_BUSY              SD_CASR_CACHE_BUSY_Msk
#define SD_CASR_CACHE_FLUSH_Pos         (3U)
#define SD_CASR_CACHE_FLUSH_Msk         (0x1UL << SD_CASR_CACHE_FLUSH_Pos)
#define SD_CASR_CACHE_FLUSH             SD_CASR_CACHE_FLUSH_Msk

/******************** Bit definition for SD_CACR register *********************/
#define SD_CACR_READ_INDEX_Pos          (0U)
#define SD_CACR_READ_INDEX_Msk          (0x3FUL << SD_CACR_READ_INDEX_Pos)
#define SD_CACR_READ_INDEX              SD_CACR_READ_INDEX_Msk
#define SD_CACR_READ_HAS_RSP_Pos        (6U)
#define SD_CACR_READ_HAS_RSP_Msk        (0x1UL << SD_CACR_READ_HAS_RSP_Pos)
#define SD_CACR_READ_HAS_RSP            SD_CACR_READ_HAS_RSP_Msk
#define SD_CACR_READ_LONG_RSP_Pos       (7U)
#define SD_CACR_READ_LONG_RSP_Msk       (0x1UL << SD_CACR_READ_LONG_RSP_Pos)
#define SD_CACR_READ_LONG_RSP           SD_CACR_READ_LONG_RSP_Msk
#define SD_CACR_STOP_INDEX_Pos          (8U)
#define SD_CACR_STOP_INDEX_Msk          (0x3FUL << SD_CACR_STOP_INDEX_Pos)
#define SD_CACR_STOP_INDEX              SD_CACR_STOP_INDEX_Msk
#define SD_CACR_STOP_HAS_RSP_Pos        (14U)
#define SD_CACR_STOP_HAS_RSP_Msk        (0x1UL << SD_CACR_STOP_HAS_RSP_Pos)
#define SD_CACR_STOP_HAS_RSP            SD_CACR_STOP_HAS_RSP_Msk
#define SD_CACR_STOP_LONG_RSP_Pos       (15U)
#define SD_CACR_STOP_LONG_RSP_Msk       (0x1UL << SD_CACR_STOP_LONG_RSP_Pos)
#define SD_CACR_STOP_LONG_RSP           SD_CACR_STOP_LONG_RSP_Msk
#define SD_CACR_CACHE_BLOCK_Pos         (16U)
#define SD_CACR_CACHE_BLOCK_Msk         (0x7UL << SD_CACR_CACHE_BLOCK_Pos)
#define SD_CACR_CACHE_BLOCK             SD_CACR_CACHE_BLOCK_Msk
#define SD_CACR_CACHE_PREF_BLOCK_Pos    (20U)
#define SD_CACR_CACHE_PREF_BLOCK_Msk    (0xFUL << SD_CACR_CACHE_PREF_BLOCK_Pos)
#define SD_CACR_CACHE_PREF_BLOCK        SD_CACR_CACHE_PREF_BLOCK_Msk
#define SD_CACR_CACHE_HRESP_Pos         (26U)
#define SD_CACR_CACHE_HRESP_Msk         (0x1UL << SD_CACR_CACHE_HRESP_Pos)
#define SD_CACR_CACHE_HRESP             SD_CACR_CACHE_HRESP_Msk
#define SD_CACR_CACHE_NOCRC_Pos         (27U)
#define SD_CACR_CACHE_NOCRC_Msk         (0x1UL << SD_CACR_CACHE_NOCRC_Pos)
#define SD_CACR_CACHE_NOCRC             SD_CACR_CACHE_NOCRC_Msk
#define SD_CACR_CACHE_SDSC_Pos          (28U)
#define SD_CACR_CACHE_SDSC_Msk          (0x1UL << SD_CACR_CACHE_SDSC_Pos)
#define SD_CACR_CACHE_SDSC              SD_CACR_CACHE_SDSC_Msk
#define SD_CACR_CACHE_FORCE_READ_Pos    (29U)
#define SD_CACR_CACHE_FORCE_READ_Msk    (0x1UL << SD_CACR_CACHE_FORCE_READ_Pos)
#define SD_CACR_CACHE_FORCE_READ        SD_CACR_CACHE_FORCE_READ_Msk
#define SD_CACR_CACHE_TO_EN_Pos         (30U)
#define SD_CACR_CACHE_TO_EN_Msk         (0x1UL << SD_CACR_CACHE_TO_EN_Pos)
#define SD_CACR_CACHE_TO_EN             SD_CACR_CACHE_TO_EN_Msk
#define SD_CACR_CACHE_EN_Pos            (31U)
#define SD_CACR_CACHE_EN_Msk            (0x1UL << SD_CACR_CACHE_EN_Pos)
#define SD_CACR_CACHE_EN                SD_CACR_CACHE_EN_Msk

/******************** Bit definition for SD_CACNT register ********************/
#define SD_CACNT_CACHE_NCC_Pos          (0U)
#define SD_CACNT_CACHE_NCC_Msk          (0xFFUL << SD_CACNT_CACHE_NCC_Pos)
#define SD_CACNT_CACHE_NCC              SD_CACNT_CACHE_NCC_Msk
#define SD_CACNT_CACHE_NDC_Pos          (8U)
#define SD_CACNT_CACHE_NDC_Msk          (0xFFUL << SD_CACNT_CACHE_NDC_Pos)
#define SD_CACNT_CACHE_NDC              SD_CACNT_CACHE_NDC_Msk
#define SD_CACNT_CACHE_TOR_Pos          (16U)
#define SD_CACNT_CACHE_TOR_Msk          (0xFFFFUL << SD_CACNT_CACHE_TOR_Pos)
#define SD_CACNT_CACHE_TOR              SD_CACNT_CACHE_TOR_Msk

/******************** Bit definition for SD_CAOFF register ********************/
#define SD_CAOFF_CACHE_OFFSET_Pos       (0U)
#define SD_CAOFF_CACHE_OFFSET_Msk       (0xFFFFFFFFUL << SD_CAOFF_CACHE_OFFSET_Pos)
#define SD_CAOFF_CACHE_OFFSET           SD_CAOFF_CACHE_OFFSET_Msk

/******************** Bit definition for SD_FIFO register *********************/
#define SD_FIFO_DATA_Pos                (0U)
#define SD_FIFO_DATA_Msk                (0xFFFFFFFFUL << SD_FIFO_DATA_Pos)
#define SD_FIFO_DATA                    SD_FIFO_DATA_Msk

#endif
