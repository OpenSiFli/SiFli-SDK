#ifndef _REG_UART_H_
#define _REG_UART_H_

#include <stdint.h>
#include "_reg_uart.h"
#include "compiler.h"
#include "arch.h"
#include "reg_access.h"

#define REG_UART_COUNT 9

#define REG_UART_DECODING_MASK 0x0000003F

/**
 * @brief CTRL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:24        CLK_DIV_FRACP   0x0
 *  23:16         CLK_DIV_INTP   0x0
 *  15:08              CNT_VAL   0x0
 *     04            CNT_START   0
 *     01        EXT_WAKEUP_EN   0
 *     00            FORCE_RTS   0
 * </pre>
 */
#define UART_CTRL_ADDR   0x10007000
#define UART_CTRL_OFFSET 0x00000000
#define UART_CTRL_INDEX  0x00000000
#define UART_CTRL_RESET  0x00000000

__INLINE uint32_t uart_ctrl_get(void)
{
    return REG_PL_RD(UART_CTRL_ADDR);
}

__INLINE void uart_ctrl_set(uint32_t value)
{
    REG_PL_WR(UART_CTRL_ADDR, value);
}

// field definitions
#define UART_CLK_DIV_FRACP_MASK   ((uint32_t)0xFF000000)
#define UART_CLK_DIV_FRACP_LSB    24
#define UART_CLK_DIV_FRACP_WIDTH  ((uint32_t)0x00000008)
#define UART_CLK_DIV_INTP_MASK    ((uint32_t)0x00FF0000)
#define UART_CLK_DIV_INTP_LSB     16
#define UART_CLK_DIV_INTP_WIDTH   ((uint32_t)0x00000008)
#define UART_CNT_VAL_MASK         ((uint32_t)0x0000FF00)
#define UART_CNT_VAL_LSB          8
#define UART_CNT_VAL_WIDTH        ((uint32_t)0x00000008)
#define UART_CNT_START_BIT        ((uint32_t)0x00000010)
#define UART_CNT_START_POS        4
#define UART_EXT_WAKEUP_EN_BIT    ((uint32_t)0x00000002)
#define UART_EXT_WAKEUP_EN_POS    1
#define UART_FORCE_RTS_BIT        ((uint32_t)0x00000001)
#define UART_FORCE_RTS_POS        0

#define UART_CLK_DIV_FRACP_RST    0x0
#define UART_CLK_DIV_INTP_RST     0x0
#define UART_CNT_VAL_RST          0x0
#define UART_CNT_START_RST        0x0
#define UART_EXT_WAKEUP_EN_RST    0x0
#define UART_FORCE_RTS_RST        0x0

__INLINE void uart_ctrl_pack(uint8_t clkdivfracp, uint8_t clkdivintp, uint8_t cntval, uint8_t cntstart, uint8_t extwakeupen, uint8_t forcerts)
{
    ASSERT_ERR((((uint32_t)clkdivfracp << 24) & ~((uint32_t)0xFF000000)) == 0);
    ASSERT_ERR((((uint32_t)clkdivintp << 16) & ~((uint32_t)0x00FF0000)) == 0);
    ASSERT_ERR((((uint32_t)cntval << 8) & ~((uint32_t)0x0000FF00)) == 0);
    ASSERT_ERR((((uint32_t)cntstart << 4) & ~((uint32_t)0x00000010)) == 0);
    ASSERT_ERR((((uint32_t)extwakeupen << 1) & ~((uint32_t)0x00000002)) == 0);
    ASSERT_ERR((((uint32_t)forcerts << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_PL_WR(UART_CTRL_ADDR, ((uint32_t)clkdivfracp << 24) | ((uint32_t)clkdivintp << 16) | ((uint32_t)cntval << 8) | ((uint32_t)cntstart << 4) | ((uint32_t)extwakeupen << 1) | ((uint32_t)forcerts << 0));
}

__INLINE void uart_ctrl_unpack(uint8_t *clkdivfracp, uint8_t *clkdivintp, uint8_t *cntval, uint8_t *cntstart, uint8_t *extwakeupen, uint8_t *forcerts)
{
    uint32_t localVal = REG_PL_RD(UART_CTRL_ADDR);

    *clkdivfracp = (localVal & ((uint32_t)0xFF000000)) >> 24;
    *clkdivintp = (localVal & ((uint32_t)0x00FF0000)) >> 16;
    *cntval = (localVal & ((uint32_t)0x0000FF00)) >> 8;
    *cntstart = (localVal & ((uint32_t)0x00000010)) >> 4;
    *extwakeupen = (localVal & ((uint32_t)0x00000002)) >> 1;
    *forcerts = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t uart_clk_div_fracp_getf(void)
{
    uint32_t localVal = REG_PL_RD(UART_CTRL_ADDR);
    return ((localVal & ((uint32_t)0xFF000000)) >> 24);
}

__INLINE void uart_clk_div_fracp_setf(uint8_t clkdivfracp)
{
    ASSERT_ERR((((uint32_t)clkdivfracp << 24) & ~((uint32_t)0xFF000000)) == 0);
    REG_PL_WR(UART_CTRL_ADDR, (REG_PL_RD(UART_CTRL_ADDR) & ~((uint32_t)0xFF000000)) | ((uint32_t)clkdivfracp << 24));
}

__INLINE uint8_t uart_clk_div_intp_getf(void)
{
    uint32_t localVal = REG_PL_RD(UART_CTRL_ADDR);
    return ((localVal & ((uint32_t)0x00FF0000)) >> 16);
}

__INLINE void uart_clk_div_intp_setf(uint8_t clkdivintp)
{
    ASSERT_ERR((((uint32_t)clkdivintp << 16) & ~((uint32_t)0x00FF0000)) == 0);
    REG_PL_WR(UART_CTRL_ADDR, (REG_PL_RD(UART_CTRL_ADDR) & ~((uint32_t)0x00FF0000)) | ((uint32_t)clkdivintp << 16));
}

__INLINE uint8_t uart_cnt_val_getf(void)
{
    uint32_t localVal = REG_PL_RD(UART_CTRL_ADDR);
    return ((localVal & ((uint32_t)0x0000FF00)) >> 8);
}

__INLINE void uart_cnt_val_setf(uint8_t cntval)
{
    ASSERT_ERR((((uint32_t)cntval << 8) & ~((uint32_t)0x0000FF00)) == 0);
    REG_PL_WR(UART_CTRL_ADDR, (REG_PL_RD(UART_CTRL_ADDR) & ~((uint32_t)0x0000FF00)) | ((uint32_t)cntval << 8));
}

__INLINE void uart_cnt_start_setf(uint8_t cntstart)
{
    ASSERT_ERR((((uint32_t)cntstart << 4) & ~((uint32_t)0x00000010)) == 0);
    REG_PL_WR(UART_CTRL_ADDR, (REG_PL_RD(UART_CTRL_ADDR) & ~((uint32_t)0x00000010)) | ((uint32_t)cntstart << 4));
}

__INLINE uint8_t uart_ext_wakeup_en_getf(void)
{
    uint32_t localVal = REG_PL_RD(UART_CTRL_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE void uart_ext_wakeup_en_setf(uint8_t extwakeupen)
{
    ASSERT_ERR((((uint32_t)extwakeupen << 1) & ~((uint32_t)0x00000002)) == 0);
    REG_PL_WR(UART_CTRL_ADDR, (REG_PL_RD(UART_CTRL_ADDR) & ~((uint32_t)0x00000002)) | ((uint32_t)extwakeupen << 1));
}

__INLINE uint8_t uart_force_rts_getf(void)
{
    uint32_t localVal = REG_PL_RD(UART_CTRL_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void uart_force_rts_setf(uint8_t forcerts)
{
    ASSERT_ERR((((uint32_t)forcerts << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_PL_WR(UART_CTRL_ADDR, (REG_PL_RD(UART_CTRL_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)forcerts << 0));
}

/**
 * @brief STAT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     09       TX_DMA_STARTED   0
 *     08       RX_DMA_STARTED   0
 *     05           EXT_WAKEUP   0
 *     04              CNT_END   0
 *     03        TX_FIFO_EMPTY   0
 *     02    RX_FIFO_NOT_EMPTY   0
 *     01                  CTS   0
 *     00                  RTS   0
 * </pre>
 */
#define UART_STAT_ADDR   0x10007004
#define UART_STAT_OFFSET 0x00000004
#define UART_STAT_INDEX  0x00000001
#define UART_STAT_RESET  0x00000000

__INLINE uint32_t uart_stat_get(void)
{
    return REG_PL_RD(UART_STAT_ADDR);
}

// field definitions
#define UART_TX_DMA_STARTED_BIT       ((uint32_t)0x00000200)
#define UART_TX_DMA_STARTED_POS       9
#define UART_RX_DMA_STARTED_BIT       ((uint32_t)0x00000100)
#define UART_RX_DMA_STARTED_POS       8
#define UART_EXT_WAKEUP_BIT           ((uint32_t)0x00000020)
#define UART_EXT_WAKEUP_POS           5
#define UART_CNT_END_BIT              ((uint32_t)0x00000010)
#define UART_CNT_END_POS              4
#define UART_TX_FIFO_EMPTY_BIT        ((uint32_t)0x00000008)
#define UART_TX_FIFO_EMPTY_POS        3
#define UART_RX_FIFO_NOT_EMPTY_BIT    ((uint32_t)0x00000004)
#define UART_RX_FIFO_NOT_EMPTY_POS    2
#define UART_CTS_BIT                  ((uint32_t)0x00000002)
#define UART_CTS_POS                  1
#define UART_RTS_BIT                  ((uint32_t)0x00000001)
#define UART_RTS_POS                  0

#define UART_TX_DMA_STARTED_RST       0x0
#define UART_RX_DMA_STARTED_RST       0x0
#define UART_EXT_WAKEUP_RST           0x0
#define UART_CNT_END_RST              0x0
#define UART_TX_FIFO_EMPTY_RST        0x0
#define UART_RX_FIFO_NOT_EMPTY_RST    0x0
#define UART_CTS_RST                  0x0
#define UART_RTS_RST                  0x0

__INLINE void uart_stat_unpack(uint8_t *txdmastarted, uint8_t *rxdmastarted, uint8_t *extwakeup, uint8_t *cntend, uint8_t *txfifoempty, uint8_t *rxfifonotempty, uint8_t *cts, uint8_t *rts)
{
    uint32_t localVal = REG_PL_RD(UART_STAT_ADDR);

    *txdmastarted = (localVal & ((uint32_t)0x00000200)) >> 9;
    *rxdmastarted = (localVal & ((uint32_t)0x00000100)) >> 8;
    *extwakeup = (localVal & ((uint32_t)0x00000020)) >> 5;
    *cntend = (localVal & ((uint32_t)0x00000010)) >> 4;
    *txfifoempty = (localVal & ((uint32_t)0x00000008)) >> 3;
    *rxfifonotempty = (localVal & ((uint32_t)0x00000004)) >> 2;
    *cts = (localVal & ((uint32_t)0x00000002)) >> 1;
    *rts = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t uart_tx_dma_started_getf(void)
{
    uint32_t localVal = REG_PL_RD(UART_STAT_ADDR);
    return ((localVal & ((uint32_t)0x00000200)) >> 9);
}

__INLINE uint8_t uart_rx_dma_started_getf(void)
{
    uint32_t localVal = REG_PL_RD(UART_STAT_ADDR);
    return ((localVal & ((uint32_t)0x00000100)) >> 8);
}

__INLINE uint8_t uart_ext_wakeup_getf(void)
{
    uint32_t localVal = REG_PL_RD(UART_STAT_ADDR);
    return ((localVal & ((uint32_t)0x00000020)) >> 5);
}

__INLINE uint8_t uart_cnt_end_getf(void)
{
    uint32_t localVal = REG_PL_RD(UART_STAT_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE uint8_t uart_tx_fifo_empty_getf(void)
{
    uint32_t localVal = REG_PL_RD(UART_STAT_ADDR);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

__INLINE uint8_t uart_rx_fifo_not_empty_getf(void)
{
    uint32_t localVal = REG_PL_RD(UART_STAT_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE uint8_t uart_cts_getf(void)
{
    uint32_t localVal = REG_PL_RD(UART_STAT_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE uint8_t uart_rts_getf(void)
{
    uint32_t localVal = REG_PL_RD(UART_STAT_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

/**
 * @brief CLK register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00                 FREQ   0x0
 * </pre>
 */
#define UART_CLK_ADDR   0x10007008
#define UART_CLK_OFFSET 0x00000008
#define UART_CLK_INDEX  0x00000002
#define UART_CLK_RESET  0x00000000

__INLINE uint32_t uart_clk_get(void)
{
    return REG_PL_RD(UART_CLK_ADDR);
}

// field definitions
#define UART_FREQ_MASK   ((uint32_t)0xFFFFFFFF)
#define UART_FREQ_LSB    0
#define UART_FREQ_WIDTH  ((uint32_t)0x00000020)

#define UART_FREQ_RST    0x0

__INLINE uint32_t uart_freq_getf(void)
{
    uint32_t localVal = REG_PL_RD(UART_CLK_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0xFFFFFFFF)) == 0);
    return (localVal >> 0);
}

/**
 * @brief ISR_STAT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     06          TX_DMA_DONE   0
 *     05          RX_DMA_DONE   0
 *     04            BREAK_ISR   0
 *     01    TX_FIFO_EMPTY_ISR   0
 *     00   RX_FIFO_NOT_EMPTY_ISR   0
 * </pre>
 */
#define UART_ISR_STAT_ADDR   0x1000700C
#define UART_ISR_STAT_OFFSET 0x0000000C
#define UART_ISR_STAT_INDEX  0x00000003
#define UART_ISR_STAT_RESET  0x00000000

__INLINE uint32_t uart_isr_stat_get(void)
{
    return REG_PL_RD(UART_ISR_STAT_ADDR);
}

// field definitions
#define UART_TX_DMA_DONE_BIT              ((uint32_t)0x00000040)
#define UART_TX_DMA_DONE_POS              6
#define UART_RX_DMA_DONE_BIT              ((uint32_t)0x00000020)
#define UART_RX_DMA_DONE_POS              5
#define UART_BREAK_ISR_BIT                ((uint32_t)0x00000010)
#define UART_BREAK_ISR_POS                4
#define UART_TX_FIFO_EMPTY_ISR_BIT        ((uint32_t)0x00000002)
#define UART_TX_FIFO_EMPTY_ISR_POS        1
#define UART_RX_FIFO_NOT_EMPTY_ISR_BIT    ((uint32_t)0x00000001)
#define UART_RX_FIFO_NOT_EMPTY_ISR_POS    0

#define UART_TX_DMA_DONE_RST              0x0
#define UART_RX_DMA_DONE_RST              0x0
#define UART_BREAK_ISR_RST                0x0
#define UART_TX_FIFO_EMPTY_ISR_RST        0x0
#define UART_RX_FIFO_NOT_EMPTY_ISR_RST    0x0

__INLINE void uart_isr_stat_unpack(uint8_t *txdmadone, uint8_t *rxdmadone, uint8_t *breakisr, uint8_t *txfifoemptyisr, uint8_t *rxfifonotemptyisr)
{
    uint32_t localVal = REG_PL_RD(UART_ISR_STAT_ADDR);

    *txdmadone = (localVal & ((uint32_t)0x00000040)) >> 6;
    *rxdmadone = (localVal & ((uint32_t)0x00000020)) >> 5;
    *breakisr = (localVal & ((uint32_t)0x00000010)) >> 4;
    *txfifoemptyisr = (localVal & ((uint32_t)0x00000002)) >> 1;
    *rxfifonotemptyisr = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t uart_tx_dma_done_getf(void)
{
    uint32_t localVal = REG_PL_RD(UART_ISR_STAT_ADDR);
    return ((localVal & ((uint32_t)0x00000040)) >> 6);
}

__INLINE uint8_t uart_rx_dma_done_getf(void)
{
    uint32_t localVal = REG_PL_RD(UART_ISR_STAT_ADDR);
    return ((localVal & ((uint32_t)0x00000020)) >> 5);
}

__INLINE uint8_t uart_break_isr_getf(void)
{
    uint32_t localVal = REG_PL_RD(UART_ISR_STAT_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE uint8_t uart_tx_fifo_empty_isr_getf(void)
{
    uint32_t localVal = REG_PL_RD(UART_ISR_STAT_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE uint8_t uart_rx_fifo_not_empty_isr_getf(void)
{
    uint32_t localVal = REG_PL_RD(UART_ISR_STAT_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

/**
 * @brief ISR_EN register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     06       TX_DMA_DONE_EN   0
 *     05       RX_DMA_DONE_EN   0
 *     04             BREAK_EN   0
 *     01     TX_FIFO_EMPTY_EN   0
 *     00   RX_FIFO_NOT_EMPTY_EN   0
 * </pre>
 */
#define UART_ISR_EN_ADDR   0x10007010
#define UART_ISR_EN_OFFSET 0x00000010
#define UART_ISR_EN_INDEX  0x00000004
#define UART_ISR_EN_RESET  0x00000000

__INLINE uint32_t uart_isr_en_get(void)
{
    return REG_PL_RD(UART_ISR_EN_ADDR);
}

__INLINE void uart_isr_en_set(uint32_t value)
{
    REG_PL_WR(UART_ISR_EN_ADDR, value);
}

// field definitions
#define UART_TX_DMA_DONE_EN_BIT          ((uint32_t)0x00000040)
#define UART_TX_DMA_DONE_EN_POS          6
#define UART_RX_DMA_DONE_EN_BIT          ((uint32_t)0x00000020)
#define UART_RX_DMA_DONE_EN_POS          5
#define UART_BREAK_EN_BIT                ((uint32_t)0x00000010)
#define UART_BREAK_EN_POS                4
#define UART_TX_FIFO_EMPTY_EN_BIT        ((uint32_t)0x00000002)
#define UART_TX_FIFO_EMPTY_EN_POS        1
#define UART_RX_FIFO_NOT_EMPTY_EN_BIT    ((uint32_t)0x00000001)
#define UART_RX_FIFO_NOT_EMPTY_EN_POS    0

#define UART_TX_DMA_DONE_EN_RST          0x0
#define UART_RX_DMA_DONE_EN_RST          0x0
#define UART_BREAK_EN_RST                0x0
#define UART_TX_FIFO_EMPTY_EN_RST        0x0
#define UART_RX_FIFO_NOT_EMPTY_EN_RST    0x0

__INLINE void uart_isr_en_pack(uint8_t txdmadoneen, uint8_t rxdmadoneen, uint8_t breaken, uint8_t txfifoemptyen, uint8_t rxfifonotemptyen)
{
    ASSERT_ERR((((uint32_t)txdmadoneen << 6) & ~((uint32_t)0x00000040)) == 0);
    ASSERT_ERR((((uint32_t)rxdmadoneen << 5) & ~((uint32_t)0x00000020)) == 0);
    ASSERT_ERR((((uint32_t)breaken << 4) & ~((uint32_t)0x00000010)) == 0);
    ASSERT_ERR((((uint32_t)txfifoemptyen << 1) & ~((uint32_t)0x00000002)) == 0);
    ASSERT_ERR((((uint32_t)rxfifonotemptyen << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_PL_WR(UART_ISR_EN_ADDR, ((uint32_t)txdmadoneen << 6) | ((uint32_t)rxdmadoneen << 5) | ((uint32_t)breaken << 4) | ((uint32_t)txfifoemptyen << 1) | ((uint32_t)rxfifonotemptyen << 0));
}

__INLINE void uart_isr_en_unpack(uint8_t *txdmadoneen, uint8_t *rxdmadoneen, uint8_t *breaken, uint8_t *txfifoemptyen, uint8_t *rxfifonotemptyen)
{
    uint32_t localVal = REG_PL_RD(UART_ISR_EN_ADDR);

    *txdmadoneen = (localVal & ((uint32_t)0x00000040)) >> 6;
    *rxdmadoneen = (localVal & ((uint32_t)0x00000020)) >> 5;
    *breaken = (localVal & ((uint32_t)0x00000010)) >> 4;
    *txfifoemptyen = (localVal & ((uint32_t)0x00000002)) >> 1;
    *rxfifonotemptyen = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t uart_tx_dma_done_en_getf(void)
{
    uint32_t localVal = REG_PL_RD(UART_ISR_EN_ADDR);
    return ((localVal & ((uint32_t)0x00000040)) >> 6);
}

__INLINE void uart_tx_dma_done_en_setf(uint8_t txdmadoneen)
{
    ASSERT_ERR((((uint32_t)txdmadoneen << 6) & ~((uint32_t)0x00000040)) == 0);
    REG_PL_WR(UART_ISR_EN_ADDR, (REG_PL_RD(UART_ISR_EN_ADDR) & ~((uint32_t)0x00000040)) | ((uint32_t)txdmadoneen << 6));
}

__INLINE uint8_t uart_rx_dma_done_en_getf(void)
{
    uint32_t localVal = REG_PL_RD(UART_ISR_EN_ADDR);
    return ((localVal & ((uint32_t)0x00000020)) >> 5);
}

__INLINE void uart_rx_dma_done_en_setf(uint8_t rxdmadoneen)
{
    ASSERT_ERR((((uint32_t)rxdmadoneen << 5) & ~((uint32_t)0x00000020)) == 0);
    REG_PL_WR(UART_ISR_EN_ADDR, (REG_PL_RD(UART_ISR_EN_ADDR) & ~((uint32_t)0x00000020)) | ((uint32_t)rxdmadoneen << 5));
}

__INLINE uint8_t uart_break_en_getf(void)
{
    uint32_t localVal = REG_PL_RD(UART_ISR_EN_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE void uart_break_en_setf(uint8_t breaken)
{
    ASSERT_ERR((((uint32_t)breaken << 4) & ~((uint32_t)0x00000010)) == 0);
    REG_PL_WR(UART_ISR_EN_ADDR, (REG_PL_RD(UART_ISR_EN_ADDR) & ~((uint32_t)0x00000010)) | ((uint32_t)breaken << 4));
}

__INLINE uint8_t uart_tx_fifo_empty_en_getf(void)
{
    uint32_t localVal = REG_PL_RD(UART_ISR_EN_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE void uart_tx_fifo_empty_en_setf(uint8_t txfifoemptyen)
{
    ASSERT_ERR((((uint32_t)txfifoemptyen << 1) & ~((uint32_t)0x00000002)) == 0);
    REG_PL_WR(UART_ISR_EN_ADDR, (REG_PL_RD(UART_ISR_EN_ADDR) & ~((uint32_t)0x00000002)) | ((uint32_t)txfifoemptyen << 1));
}

__INLINE uint8_t uart_rx_fifo_not_empty_en_getf(void)
{
    uint32_t localVal = REG_PL_RD(UART_ISR_EN_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void uart_rx_fifo_not_empty_en_setf(uint8_t rxfifonotemptyen)
{
    ASSERT_ERR((((uint32_t)rxfifonotemptyen << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_PL_WR(UART_ISR_EN_ADDR, (REG_PL_RD(UART_ISR_EN_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)rxfifonotemptyen << 0));
}

/**
 * @brief ISR_CLR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     06      TX_DMA_DONE_CLR   0
 *     05      RX_DMA_DONE_CLR   0
 *     04            BREAK_CLR   0
 *     01    TX_FIFO_EMPTY_CLR   0
 *     00   RX_FIFO_NOT_EMPTY_CLR   0
 * </pre>
 */
#define UART_ISR_CLR_ADDR   0x10007014
#define UART_ISR_CLR_OFFSET 0x00000014
#define UART_ISR_CLR_INDEX  0x00000005
#define UART_ISR_CLR_RESET  0x00000000

__INLINE void uart_isr_clr_set(uint32_t value)
{
    REG_PL_WR(UART_ISR_CLR_ADDR, value);
}

// field definitions
#define UART_TX_DMA_DONE_CLR_BIT          ((uint32_t)0x00000040)
#define UART_TX_DMA_DONE_CLR_POS          6
#define UART_RX_DMA_DONE_CLR_BIT          ((uint32_t)0x00000020)
#define UART_RX_DMA_DONE_CLR_POS          5
#define UART_BREAK_CLR_BIT                ((uint32_t)0x00000010)
#define UART_BREAK_CLR_POS                4
#define UART_TX_FIFO_EMPTY_CLR_BIT        ((uint32_t)0x00000002)
#define UART_TX_FIFO_EMPTY_CLR_POS        1
#define UART_RX_FIFO_NOT_EMPTY_CLR_BIT    ((uint32_t)0x00000001)
#define UART_RX_FIFO_NOT_EMPTY_CLR_POS    0

#define UART_TX_DMA_DONE_CLR_RST          0x0
#define UART_RX_DMA_DONE_CLR_RST          0x0
#define UART_BREAK_CLR_RST                0x0
#define UART_TX_FIFO_EMPTY_CLR_RST        0x0
#define UART_RX_FIFO_NOT_EMPTY_CLR_RST    0x0

__INLINE void uart_isr_clr_pack(uint8_t txdmadoneclr, uint8_t rxdmadoneclr, uint8_t breakclr, uint8_t txfifoemptyclr, uint8_t rxfifonotemptyclr)
{
    ASSERT_ERR((((uint32_t)txdmadoneclr << 6) & ~((uint32_t)0x00000040)) == 0);
    ASSERT_ERR((((uint32_t)rxdmadoneclr << 5) & ~((uint32_t)0x00000020)) == 0);
    ASSERT_ERR((((uint32_t)breakclr << 4) & ~((uint32_t)0x00000010)) == 0);
    ASSERT_ERR((((uint32_t)txfifoemptyclr << 1) & ~((uint32_t)0x00000002)) == 0);
    ASSERT_ERR((((uint32_t)rxfifonotemptyclr << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_PL_WR(UART_ISR_CLR_ADDR, ((uint32_t)txdmadoneclr << 6) | ((uint32_t)rxdmadoneclr << 5) | ((uint32_t)breakclr << 4) | ((uint32_t)txfifoemptyclr << 1) | ((uint32_t)rxfifonotemptyclr << 0));
}

__INLINE void uart_tx_dma_done_clr_setf(uint8_t txdmadoneclr)
{
    ASSERT_ERR((((uint32_t)txdmadoneclr << 6) & ~((uint32_t)0x00000040)) == 0);
    REG_PL_WR(UART_ISR_CLR_ADDR, (REG_PL_RD(UART_ISR_CLR_ADDR) & ~((uint32_t)0x00000040)) | ((uint32_t)txdmadoneclr << 6));
}

__INLINE void uart_rx_dma_done_clr_setf(uint8_t rxdmadoneclr)
{
    ASSERT_ERR((((uint32_t)rxdmadoneclr << 5) & ~((uint32_t)0x00000020)) == 0);
    REG_PL_WR(UART_ISR_CLR_ADDR, (REG_PL_RD(UART_ISR_CLR_ADDR) & ~((uint32_t)0x00000020)) | ((uint32_t)rxdmadoneclr << 5));
}

__INLINE void uart_break_clr_setf(uint8_t breakclr)
{
    ASSERT_ERR((((uint32_t)breakclr << 4) & ~((uint32_t)0x00000010)) == 0);
    REG_PL_WR(UART_ISR_CLR_ADDR, (REG_PL_RD(UART_ISR_CLR_ADDR) & ~((uint32_t)0x00000010)) | ((uint32_t)breakclr << 4));
}

__INLINE void uart_tx_fifo_empty_clr_setf(uint8_t txfifoemptyclr)
{
    ASSERT_ERR((((uint32_t)txfifoemptyclr << 1) & ~((uint32_t)0x00000002)) == 0);
    REG_PL_WR(UART_ISR_CLR_ADDR, (REG_PL_RD(UART_ISR_CLR_ADDR) & ~((uint32_t)0x00000002)) | ((uint32_t)txfifoemptyclr << 1));
}

__INLINE void uart_rx_fifo_not_empty_clr_setf(uint8_t rxfifonotemptyclr)
{
    ASSERT_ERR((((uint32_t)rxfifonotemptyclr << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_PL_WR(UART_ISR_CLR_ADDR, (REG_PL_RD(UART_ISR_CLR_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)rxfifonotemptyclr << 0));
}

/**
 * @brief RX_DMA register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00               RX_PTR   0x0
 * </pre>
 */
#define UART_RX_DMA_ADDR   0x10007018
#define UART_RX_DMA_OFFSET 0x00000018
#define UART_RX_DMA_INDEX  0x00000006
#define UART_RX_DMA_RESET  0x00000000

__INLINE uint32_t uart_rx_dma_get(void)
{
    return REG_PL_RD(UART_RX_DMA_ADDR);
}

__INLINE void uart_rx_dma_set(uint32_t value)
{
    REG_PL_WR(UART_RX_DMA_ADDR, value);
}

// field definitions
#define UART_RX_PTR_MASK   ((uint32_t)0xFFFFFFFF)
#define UART_RX_PTR_LSB    0
#define UART_RX_PTR_WIDTH  ((uint32_t)0x00000020)

#define UART_RX_PTR_RST    0x0

__INLINE uint32_t uart_rx_ptr_getf(void)
{
    uint32_t localVal = REG_PL_RD(UART_RX_DMA_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0xFFFFFFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void uart_rx_ptr_setf(uint32_t rxptr)
{
    ASSERT_ERR((((uint32_t)rxptr << 0) & ~((uint32_t)0xFFFFFFFF)) == 0);
    REG_PL_WR(UART_RX_DMA_ADDR, (uint32_t)rxptr << 0);
}

/**
 * @brief TX_DMA register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00               tx_ptr   0x0
 * </pre>
 */
#define UART_TX_DMA_ADDR   0x1000701C
#define UART_TX_DMA_OFFSET 0x0000001C
#define UART_TX_DMA_INDEX  0x00000007
#define UART_TX_DMA_RESET  0x00000000

__INLINE uint32_t uart_tx_dma_get(void)
{
    return REG_PL_RD(UART_TX_DMA_ADDR);
}

__INLINE void uart_tx_dma_set(uint32_t value)
{
    REG_PL_WR(UART_TX_DMA_ADDR, value);
}

// field definitions
#define UART_TX_PTR_MASK   ((uint32_t)0xFFFFFFFF)
#define UART_TX_PTR_LSB    0
#define UART_TX_PTR_WIDTH  ((uint32_t)0x00000020)

#define UART_TX_PTR_RST    0x0

__INLINE uint32_t uart_tx_ptr_getf(void)
{
    uint32_t localVal = REG_PL_RD(UART_TX_DMA_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0xFFFFFFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void uart_tx_ptr_setf(uint32_t txptr)
{
    ASSERT_ERR((((uint32_t)txptr << 0) & ~((uint32_t)0xFFFFFFFF)) == 0);
    REG_PL_WR(UART_TX_DMA_ADDR, (uint32_t)txptr << 0);
}

/**
 * @brief DMA_CTRL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     28             TX_START   0
 *  27:16              TX_SIZE   0x0
 *     12             RX_START   0
 *  11:00              RX_SIZE   0x0
 * </pre>
 */
#define UART_DMA_CTRL_ADDR   0x10007020
#define UART_DMA_CTRL_OFFSET 0x00000020
#define UART_DMA_CTRL_INDEX  0x00000008
#define UART_DMA_CTRL_RESET  0x00000000

__INLINE uint32_t uart_dma_ctrl_get(void)
{
    return REG_PL_RD(UART_DMA_CTRL_ADDR);
}

__INLINE void uart_dma_ctrl_set(uint32_t value)
{
    REG_PL_WR(UART_DMA_CTRL_ADDR, value);
}

// field definitions
#define UART_TX_START_BIT    ((uint32_t)0x10000000)
#define UART_TX_START_POS    28
#define UART_TX_SIZE_MASK    ((uint32_t)0x0FFF0000)
#define UART_TX_SIZE_LSB     16
#define UART_TX_SIZE_WIDTH   ((uint32_t)0x0000000C)
#define UART_RX_START_BIT    ((uint32_t)0x00001000)
#define UART_RX_START_POS    12
#define UART_RX_SIZE_MASK    ((uint32_t)0x00000FFF)
#define UART_RX_SIZE_LSB     0
#define UART_RX_SIZE_WIDTH   ((uint32_t)0x0000000C)

#define UART_TX_START_RST    0x0
#define UART_TX_SIZE_RST     0x0
#define UART_RX_START_RST    0x0
#define UART_RX_SIZE_RST     0x0

__INLINE void uart_dma_ctrl_pack(uint8_t txstart, uint16_t txsize, uint8_t rxstart, uint16_t rxsize)
{
    ASSERT_ERR((((uint32_t)txstart << 28) & ~((uint32_t)0x10000000)) == 0);
    ASSERT_ERR((((uint32_t)txsize << 16) & ~((uint32_t)0x0FFF0000)) == 0);
    ASSERT_ERR((((uint32_t)rxstart << 12) & ~((uint32_t)0x00001000)) == 0);
    ASSERT_ERR((((uint32_t)rxsize << 0) & ~((uint32_t)0x00000FFF)) == 0);
    REG_PL_WR(UART_DMA_CTRL_ADDR, ((uint32_t)txstart << 28) | ((uint32_t)txsize << 16) | ((uint32_t)rxstart << 12) | ((uint32_t)rxsize << 0));
}

__INLINE void uart_dma_ctrl_unpack(uint8_t *txstart, uint16_t *txsize, uint8_t *rxstart, uint16_t *rxsize)
{
    uint32_t localVal = REG_PL_RD(UART_DMA_CTRL_ADDR);

    *txstart = (localVal & ((uint32_t)0x10000000)) >> 28;
    *txsize = (localVal & ((uint32_t)0x0FFF0000)) >> 16;
    *rxstart = (localVal & ((uint32_t)0x00001000)) >> 12;
    *rxsize = (localVal & ((uint32_t)0x00000FFF)) >> 0;
}

__INLINE void uart_tx_start_setf(uint8_t txstart)
{
    ASSERT_ERR((((uint32_t)txstart << 28) & ~((uint32_t)0x10000000)) == 0);
    REG_PL_WR(UART_DMA_CTRL_ADDR, (REG_PL_RD(UART_DMA_CTRL_ADDR) & ~((uint32_t)0x10000000)) | ((uint32_t)txstart << 28));
}

__INLINE uint16_t uart_tx_size_getf(void)
{
    uint32_t localVal = REG_PL_RD(UART_DMA_CTRL_ADDR);
    return ((localVal & ((uint32_t)0x0FFF0000)) >> 16);
}

__INLINE void uart_tx_size_setf(uint16_t txsize)
{
    ASSERT_ERR((((uint32_t)txsize << 16) & ~((uint32_t)0x0FFF0000)) == 0);
    REG_PL_WR(UART_DMA_CTRL_ADDR, (REG_PL_RD(UART_DMA_CTRL_ADDR) & ~((uint32_t)0x0FFF0000)) | ((uint32_t)txsize << 16));
}

__INLINE void uart_rx_start_setf(uint8_t rxstart)
{
    ASSERT_ERR((((uint32_t)rxstart << 12) & ~((uint32_t)0x00001000)) == 0);
    REG_PL_WR(UART_DMA_CTRL_ADDR, (REG_PL_RD(UART_DMA_CTRL_ADDR) & ~((uint32_t)0x00001000)) | ((uint32_t)rxstart << 12));
}

__INLINE uint16_t uart_rx_size_getf(void)
{
    uint32_t localVal = REG_PL_RD(UART_DMA_CTRL_ADDR);
    return ((localVal & ((uint32_t)0x00000FFF)) >> 0);
}

__INLINE void uart_rx_size_setf(uint16_t rxsize)
{
    ASSERT_ERR((((uint32_t)rxsize << 0) & ~((uint32_t)0x00000FFF)) == 0);
    REG_PL_WR(UART_DMA_CTRL_ADDR, (REG_PL_RD(UART_DMA_CTRL_ADDR) & ~((uint32_t)0x00000FFF)) | ((uint32_t)rxsize << 0));
}


#endif // _REG_UART_H_

