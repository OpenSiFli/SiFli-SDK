#ifndef _REG_RIPPLE_H_
#define _REG_RIPPLE_H_

#include <stdint.h>
#include "_reg_ripple.h"
#include "compiler.h"
#include "arch.h"
#include "reg_access.h"

#define REG_RIPPLE_COUNT 32

#define REG_RIPPLE_DECODING_MASK 0x0000007F

/**
 * @brief BTRFIP_VER register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:24                  TYP   0x1
 *  23:16                  REL   0x1
 *  15:08                  UPG   0x7
 *  07:00                BUILD   0x0
 * </pre>
 */
#define RPL_BTRFIP_VER_ADDR   0x00000000
#define RPL_BTRFIP_VER_OFFSET 0x00000000
#define RPL_BTRFIP_VER_INDEX  0x00000000
#define RPL_BTRFIP_VER_RESET  0x01010700

__INLINE uint32_t rpl_btrfip_ver_get(void)
{
    return REG_RPL_RD(RPL_BTRFIP_VER_ADDR);
}

// field definitions
#define RPL_TYP_MASK     ((uint32_t)0xFF000000)
#define RPL_TYP_LSB      24
#define RPL_TYP_WIDTH    ((uint32_t)0x00000008)
#define RPL_REL_MASK     ((uint32_t)0x00FF0000)
#define RPL_REL_LSB      16
#define RPL_REL_WIDTH    ((uint32_t)0x00000008)
#define RPL_UPG_MASK     ((uint32_t)0x0000FF00)
#define RPL_UPG_LSB      8
#define RPL_UPG_WIDTH    ((uint32_t)0x00000008)
#define RPL_BUILD_MASK   ((uint32_t)0x000000FF)
#define RPL_BUILD_LSB    0
#define RPL_BUILD_WIDTH  ((uint32_t)0x00000008)

#define RPL_TYP_RST      0x1
#define RPL_REL_RST      0x1
#define RPL_UPG_RST      0x7
#define RPL_BUILD_RST    0x0

__INLINE void rpl_btrfip_ver_unpack(uint8_t *typ, uint8_t *rel, uint8_t *upg, uint8_t *build)
{
    uint32_t localVal = REG_RPL_RD(RPL_BTRFIP_VER_ADDR);

    *typ = (localVal & ((uint32_t)0xFF000000)) >> 24;
    *rel = (localVal & ((uint32_t)0x00FF0000)) >> 16;
    *upg = (localVal & ((uint32_t)0x0000FF00)) >> 8;
    *build = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint8_t rpl_typ_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_BTRFIP_VER_ADDR);
    return ((localVal & ((uint32_t)0xFF000000)) >> 24);
}

__INLINE uint8_t rpl_rel_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_BTRFIP_VER_ADDR);
    return ((localVal & ((uint32_t)0x00FF0000)) >> 16);
}

__INLINE uint8_t rpl_upg_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_BTRFIP_VER_ADDR);
    return ((localVal & ((uint32_t)0x0000FF00)) >> 8);
}

__INLINE uint8_t rpl_build_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_BTRFIP_VER_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

/**
 * @brief RFDYNCNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  23:20          TX_EDR_GAIN   0x0
 *  19:16         TX_GFSK_GAIN   0x0
 *  14:08              CHANNEL   0x0
 *     07              BTLE_EN   0
 *     04                RX_EN   0
 *     01               TX_EDR   0
 *     00                TX_EN   0
 * </pre>
 */
#define RPL_RFDYNCNTL_ADDR   0x00000001
#define RPL_RFDYNCNTL_OFFSET 0x00000001
#define RPL_RFDYNCNTL_INDEX  0x00000001
#define RPL_RFDYNCNTL_RESET  0x00000000

__INLINE uint32_t rpl_rfdyncntl_get(void)
{
    return REG_RPL_RD(RPL_RFDYNCNTL_ADDR);
}

__INLINE void rpl_rfdyncntl_set(uint32_t value)
{
    REG_RPL_WR(RPL_RFDYNCNTL_ADDR, value);
}

// field definitions
#define RPL_TX_EDR_GAIN_MASK    ((uint32_t)0x00F00000)
#define RPL_TX_EDR_GAIN_LSB     20
#define RPL_TX_EDR_GAIN_WIDTH   ((uint32_t)0x00000004)
#define RPL_TX_GFSK_GAIN_MASK   ((uint32_t)0x000F0000)
#define RPL_TX_GFSK_GAIN_LSB    16
#define RPL_TX_GFSK_GAIN_WIDTH  ((uint32_t)0x00000004)
#define RPL_CHANNEL_MASK        ((uint32_t)0x00007F00)
#define RPL_CHANNEL_LSB         8
#define RPL_CHANNEL_WIDTH       ((uint32_t)0x00000007)
#define RPL_BTLE_EN_BIT         ((uint32_t)0x00000080)
#define RPL_BTLE_EN_POS         7
#define RPL_RX_EN_BIT           ((uint32_t)0x00000010)
#define RPL_RX_EN_POS           4
#define RPL_TX_EDR_BIT          ((uint32_t)0x00000002)
#define RPL_TX_EDR_POS          1
#define RPL_TX_EN_BIT           ((uint32_t)0x00000001)
#define RPL_TX_EN_POS           0

#define RPL_TX_EDR_GAIN_RST     0x0
#define RPL_TX_GFSK_GAIN_RST    0x0
#define RPL_CHANNEL_RST         0x0
#define RPL_BTLE_EN_RST         0x0
#define RPL_RX_EN_RST           0x0
#define RPL_TX_EDR_RST          0x0
#define RPL_TX_EN_RST           0x0

__INLINE void rpl_rfdyncntl_pack(uint8_t txedrgain, uint8_t txgfskgain, uint8_t channel, uint8_t btleen, uint8_t rxen, uint8_t txedr, uint8_t txen)
{
    ASSERT_ERR((((uint32_t)txedrgain << 20) & ~((uint32_t)0x00F00000)) == 0);
    ASSERT_ERR((((uint32_t)txgfskgain << 16) & ~((uint32_t)0x000F0000)) == 0);
    ASSERT_ERR((((uint32_t)channel << 8) & ~((uint32_t)0x00007F00)) == 0);
    ASSERT_ERR((((uint32_t)btleen << 7) & ~((uint32_t)0x00000080)) == 0);
    ASSERT_ERR((((uint32_t)rxen << 4) & ~((uint32_t)0x00000010)) == 0);
    ASSERT_ERR((((uint32_t)txedr << 1) & ~((uint32_t)0x00000002)) == 0);
    ASSERT_ERR((((uint32_t)txen << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_RPL_WR(RPL_RFDYNCNTL_ADDR, ((uint32_t)txedrgain << 20) | ((uint32_t)txgfskgain << 16) | ((uint32_t)channel << 8) | ((uint32_t)btleen << 7) | ((uint32_t)rxen << 4) | ((uint32_t)txedr << 1) | ((uint32_t)txen << 0));
}

__INLINE void rpl_rfdyncntl_unpack(uint8_t *txedrgain, uint8_t *txgfskgain, uint8_t *channel, uint8_t *btleen, uint8_t *rxen, uint8_t *txedr, uint8_t *txen)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFDYNCNTL_ADDR);

    *txedrgain = (localVal & ((uint32_t)0x00F00000)) >> 20;
    *txgfskgain = (localVal & ((uint32_t)0x000F0000)) >> 16;
    *channel = (localVal & ((uint32_t)0x00007F00)) >> 8;
    *btleen = (localVal & ((uint32_t)0x00000080)) >> 7;
    *rxen = (localVal & ((uint32_t)0x00000010)) >> 4;
    *txedr = (localVal & ((uint32_t)0x00000002)) >> 1;
    *txen = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t rpl_tx_edr_gain_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFDYNCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00F00000)) >> 20);
}

__INLINE void rpl_tx_edr_gain_setf(uint8_t txedrgain)
{
    ASSERT_ERR((((uint32_t)txedrgain << 20) & ~((uint32_t)0x00F00000)) == 0);
    REG_RPL_WR(RPL_RFDYNCNTL_ADDR, (REG_RPL_RD(RPL_RFDYNCNTL_ADDR) & ~((uint32_t)0x00F00000)) | ((uint32_t)txedrgain << 20));
}

__INLINE uint8_t rpl_tx_gfsk_gain_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFDYNCNTL_ADDR);
    return ((localVal & ((uint32_t)0x000F0000)) >> 16);
}

__INLINE void rpl_tx_gfsk_gain_setf(uint8_t txgfskgain)
{
    ASSERT_ERR((((uint32_t)txgfskgain << 16) & ~((uint32_t)0x000F0000)) == 0);
    REG_RPL_WR(RPL_RFDYNCNTL_ADDR, (REG_RPL_RD(RPL_RFDYNCNTL_ADDR) & ~((uint32_t)0x000F0000)) | ((uint32_t)txgfskgain << 16));
}

__INLINE uint8_t rpl_channel_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFDYNCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00007F00)) >> 8);
}

__INLINE void rpl_channel_setf(uint8_t channel)
{
    ASSERT_ERR((((uint32_t)channel << 8) & ~((uint32_t)0x00007F00)) == 0);
    REG_RPL_WR(RPL_RFDYNCNTL_ADDR, (REG_RPL_RD(RPL_RFDYNCNTL_ADDR) & ~((uint32_t)0x00007F00)) | ((uint32_t)channel << 8));
}

__INLINE uint8_t rpl_btle_en_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFDYNCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000080)) >> 7);
}

__INLINE void rpl_btle_en_setf(uint8_t btleen)
{
    ASSERT_ERR((((uint32_t)btleen << 7) & ~((uint32_t)0x00000080)) == 0);
    REG_RPL_WR(RPL_RFDYNCNTL_ADDR, (REG_RPL_RD(RPL_RFDYNCNTL_ADDR) & ~((uint32_t)0x00000080)) | ((uint32_t)btleen << 7));
}

__INLINE uint8_t rpl_rx_en_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFDYNCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE void rpl_rx_en_setf(uint8_t rxen)
{
    ASSERT_ERR((((uint32_t)rxen << 4) & ~((uint32_t)0x00000010)) == 0);
    REG_RPL_WR(RPL_RFDYNCNTL_ADDR, (REG_RPL_RD(RPL_RFDYNCNTL_ADDR) & ~((uint32_t)0x00000010)) | ((uint32_t)rxen << 4));
}

__INLINE uint8_t rpl_tx_edr_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFDYNCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE void rpl_tx_edr_setf(uint8_t txedr)
{
    ASSERT_ERR((((uint32_t)txedr << 1) & ~((uint32_t)0x00000002)) == 0);
    REG_RPL_WR(RPL_RFDYNCNTL_ADDR, (REG_RPL_RD(RPL_RFDYNCNTL_ADDR) & ~((uint32_t)0x00000002)) | ((uint32_t)txedr << 1));
}

__INLINE uint8_t rpl_tx_en_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFDYNCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void rpl_tx_en_setf(uint8_t txen)
{
    ASSERT_ERR((((uint32_t)txen << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_RPL_WR(RPL_RFDYNCNTL_ADDR, (REG_RPL_RD(RPL_RFDYNCNTL_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)txen << 0));
}

/**
 * @brief SEQCNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  27:24        RX_FLAG_CHECK   0x2
 *  20:16        RX_DIV_FACTOR   0x8
 *  11:08        TX_FLAG_CHECK   0x2
 *  04:00        TX_DIV_FACTOR   0x8
 * </pre>
 */
#define RPL_SEQCNTL_ADDR   0x00000002
#define RPL_SEQCNTL_OFFSET 0x00000002
#define RPL_SEQCNTL_INDEX  0x00000002
#define RPL_SEQCNTL_RESET  0x02080208

__INLINE uint32_t rpl_seqcntl_get(void)
{
    return REG_RPL_RD(RPL_SEQCNTL_ADDR);
}

__INLINE void rpl_seqcntl_set(uint32_t value)
{
    REG_RPL_WR(RPL_SEQCNTL_ADDR, value);
}

// field definitions
#define RPL_RX_FLAG_CHECK_MASK   ((uint32_t)0x0F000000)
#define RPL_RX_FLAG_CHECK_LSB    24
#define RPL_RX_FLAG_CHECK_WIDTH  ((uint32_t)0x00000004)
#define RPL_RX_DIV_FACTOR_MASK   ((uint32_t)0x001F0000)
#define RPL_RX_DIV_FACTOR_LSB    16
#define RPL_RX_DIV_FACTOR_WIDTH  ((uint32_t)0x00000005)
#define RPL_TX_FLAG_CHECK_MASK   ((uint32_t)0x00000F00)
#define RPL_TX_FLAG_CHECK_LSB    8
#define RPL_TX_FLAG_CHECK_WIDTH  ((uint32_t)0x00000004)
#define RPL_TX_DIV_FACTOR_MASK   ((uint32_t)0x0000001F)
#define RPL_TX_DIV_FACTOR_LSB    0
#define RPL_TX_DIV_FACTOR_WIDTH  ((uint32_t)0x00000005)

#define RPL_RX_FLAG_CHECK_RST    0x2
#define RPL_RX_DIV_FACTOR_RST    0x8
#define RPL_TX_FLAG_CHECK_RST    0x2
#define RPL_TX_DIV_FACTOR_RST    0x8

__INLINE void rpl_seqcntl_pack(uint8_t rxflagcheck, uint8_t rxdivfactor, uint8_t txflagcheck, uint8_t txdivfactor)
{
    ASSERT_ERR((((uint32_t)rxflagcheck << 24) & ~((uint32_t)0x0F000000)) == 0);
    ASSERT_ERR((((uint32_t)rxdivfactor << 16) & ~((uint32_t)0x001F0000)) == 0);
    ASSERT_ERR((((uint32_t)txflagcheck << 8) & ~((uint32_t)0x00000F00)) == 0);
    ASSERT_ERR((((uint32_t)txdivfactor << 0) & ~((uint32_t)0x0000001F)) == 0);
    REG_RPL_WR(RPL_SEQCNTL_ADDR, ((uint32_t)rxflagcheck << 24) | ((uint32_t)rxdivfactor << 16) | ((uint32_t)txflagcheck << 8) | ((uint32_t)txdivfactor << 0));
}

__INLINE void rpl_seqcntl_unpack(uint8_t *rxflagcheck, uint8_t *rxdivfactor, uint8_t *txflagcheck, uint8_t *txdivfactor)
{
    uint32_t localVal = REG_RPL_RD(RPL_SEQCNTL_ADDR);

    *rxflagcheck = (localVal & ((uint32_t)0x0F000000)) >> 24;
    *rxdivfactor = (localVal & ((uint32_t)0x001F0000)) >> 16;
    *txflagcheck = (localVal & ((uint32_t)0x00000F00)) >> 8;
    *txdivfactor = (localVal & ((uint32_t)0x0000001F)) >> 0;
}

__INLINE uint8_t rpl_rx_flag_check_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_SEQCNTL_ADDR);
    return ((localVal & ((uint32_t)0x0F000000)) >> 24);
}

__INLINE void rpl_rx_flag_check_setf(uint8_t rxflagcheck)
{
    ASSERT_ERR((((uint32_t)rxflagcheck << 24) & ~((uint32_t)0x0F000000)) == 0);
    REG_RPL_WR(RPL_SEQCNTL_ADDR, (REG_RPL_RD(RPL_SEQCNTL_ADDR) & ~((uint32_t)0x0F000000)) | ((uint32_t)rxflagcheck << 24));
}

__INLINE uint8_t rpl_rx_div_factor_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_SEQCNTL_ADDR);
    return ((localVal & ((uint32_t)0x001F0000)) >> 16);
}

__INLINE void rpl_rx_div_factor_setf(uint8_t rxdivfactor)
{
    ASSERT_ERR((((uint32_t)rxdivfactor << 16) & ~((uint32_t)0x001F0000)) == 0);
    REG_RPL_WR(RPL_SEQCNTL_ADDR, (REG_RPL_RD(RPL_SEQCNTL_ADDR) & ~((uint32_t)0x001F0000)) | ((uint32_t)rxdivfactor << 16));
}

__INLINE uint8_t rpl_tx_flag_check_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_SEQCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000F00)) >> 8);
}

__INLINE void rpl_tx_flag_check_setf(uint8_t txflagcheck)
{
    ASSERT_ERR((((uint32_t)txflagcheck << 8) & ~((uint32_t)0x00000F00)) == 0);
    REG_RPL_WR(RPL_SEQCNTL_ADDR, (REG_RPL_RD(RPL_SEQCNTL_ADDR) & ~((uint32_t)0x00000F00)) | ((uint32_t)txflagcheck << 8));
}

__INLINE uint8_t rpl_tx_div_factor_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_SEQCNTL_ADDR);
    return ((localVal & ((uint32_t)0x0000001F)) >> 0);
}

__INLINE void rpl_tx_div_factor_setf(uint8_t txdivfactor)
{
    ASSERT_ERR((((uint32_t)txdivfactor << 0) & ~((uint32_t)0x0000001F)) == 0);
    REG_RPL_WR(RPL_SEQCNTL_ADDR, (REG_RPL_RD(RPL_SEQCNTL_ADDR) & ~((uint32_t)0x0000001F)) | ((uint32_t)txdivfactor << 0));
}

/**
 * @brief RFTXUP_DLY0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:22           TX_UP_DLY2   0x19
 *  20:11           TX_UP_DLY1   0x206
 *  09:00           TX_UP_DLY0   0x1
 * </pre>
 */
#define RPL_RFTXUP_DLY0_ADDR   0x00000003
#define RPL_RFTXUP_DLY0_OFFSET 0x00000003
#define RPL_RFTXUP_DLY0_INDEX  0x00000003
#define RPL_RFTXUP_DLY0_RESET  0x06503001

__INLINE uint32_t rpl_rftxup_dly0_get(void)
{
    return REG_RPL_RD(RPL_RFTXUP_DLY0_ADDR);
}

__INLINE void rpl_rftxup_dly0_set(uint32_t value)
{
    REG_RPL_WR(RPL_RFTXUP_DLY0_ADDR, value);
}

// field definitions
#define RPL_TX_UP_DLY2_MASK   ((uint32_t)0xFFC00000)
#define RPL_TX_UP_DLY2_LSB    22
#define RPL_TX_UP_DLY2_WIDTH  ((uint32_t)0x0000000A)
#define RPL_TX_UP_DLY1_MASK   ((uint32_t)0x001FF800)
#define RPL_TX_UP_DLY1_LSB    11
#define RPL_TX_UP_DLY1_WIDTH  ((uint32_t)0x0000000A)
#define RPL_TX_UP_DLY0_MASK   ((uint32_t)0x000003FF)
#define RPL_TX_UP_DLY0_LSB    0
#define RPL_TX_UP_DLY0_WIDTH  ((uint32_t)0x0000000A)

#define RPL_TX_UP_DLY2_RST    0x19
#define RPL_TX_UP_DLY1_RST    0x206
#define RPL_TX_UP_DLY0_RST    0x1

__INLINE void rpl_rftxup_dly0_pack(uint16_t txupdly2, uint16_t txupdly1, uint16_t txupdly0)
{
    ASSERT_ERR((((uint32_t)txupdly2 << 22) & ~((uint32_t)0xFFC00000)) == 0);
    ASSERT_ERR((((uint32_t)txupdly1 << 11) & ~((uint32_t)0x001FF800)) == 0);
    ASSERT_ERR((((uint32_t)txupdly0 << 0) & ~((uint32_t)0x000003FF)) == 0);
    REG_RPL_WR(RPL_RFTXUP_DLY0_ADDR, ((uint32_t)txupdly2 << 22) | ((uint32_t)txupdly1 << 11) | ((uint32_t)txupdly0 << 0));
}

__INLINE void rpl_rftxup_dly0_unpack(uint16_t *txupdly2, uint16_t *txupdly1, uint16_t *txupdly0)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXUP_DLY0_ADDR);

    *txupdly2 = (localVal & ((uint32_t)0xFFC00000)) >> 22;
    *txupdly1 = (localVal & ((uint32_t)0x001FF800)) >> 11;
    *txupdly0 = (localVal & ((uint32_t)0x000003FF)) >> 0;
}

__INLINE uint16_t rpl_tx_up_dly2_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXUP_DLY0_ADDR);
    return ((localVal & ((uint32_t)0xFFC00000)) >> 22);
}

__INLINE void rpl_tx_up_dly2_setf(uint16_t txupdly2)
{
    ASSERT_ERR((((uint32_t)txupdly2 << 22) & ~((uint32_t)0xFFC00000)) == 0);
    REG_RPL_WR(RPL_RFTXUP_DLY0_ADDR, (REG_RPL_RD(RPL_RFTXUP_DLY0_ADDR) & ~((uint32_t)0xFFC00000)) | ((uint32_t)txupdly2 << 22));
}

__INLINE uint16_t rpl_tx_up_dly1_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXUP_DLY0_ADDR);
    return ((localVal & ((uint32_t)0x001FF800)) >> 11);
}

__INLINE void rpl_tx_up_dly1_setf(uint16_t txupdly1)
{
    ASSERT_ERR((((uint32_t)txupdly1 << 11) & ~((uint32_t)0x001FF800)) == 0);
    REG_RPL_WR(RPL_RFTXUP_DLY0_ADDR, (REG_RPL_RD(RPL_RFTXUP_DLY0_ADDR) & ~((uint32_t)0x001FF800)) | ((uint32_t)txupdly1 << 11));
}

__INLINE uint16_t rpl_tx_up_dly0_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXUP_DLY0_ADDR);
    return ((localVal & ((uint32_t)0x000003FF)) >> 0);
}

__INLINE void rpl_tx_up_dly0_setf(uint16_t txupdly0)
{
    ASSERT_ERR((((uint32_t)txupdly0 << 0) & ~((uint32_t)0x000003FF)) == 0);
    REG_RPL_WR(RPL_RFTXUP_DLY0_ADDR, (REG_RPL_RD(RPL_RFTXUP_DLY0_ADDR) & ~((uint32_t)0x000003FF)) | ((uint32_t)txupdly0 << 0));
}

/**
 * @brief RFTXUP_DLY1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:22           TX_UP_DLY5   0x1
 *  20:11           TX_UP_DLY4   0x1
 *  09:00           TX_UP_DLY3   0x19
 * </pre>
 */
#define RPL_RFTXUP_DLY1_ADDR   0x00000004
#define RPL_RFTXUP_DLY1_OFFSET 0x00000004
#define RPL_RFTXUP_DLY1_INDEX  0x00000004
#define RPL_RFTXUP_DLY1_RESET  0x00400819

__INLINE uint32_t rpl_rftxup_dly1_get(void)
{
    return REG_RPL_RD(RPL_RFTXUP_DLY1_ADDR);
}

__INLINE void rpl_rftxup_dly1_set(uint32_t value)
{
    REG_RPL_WR(RPL_RFTXUP_DLY1_ADDR, value);
}

// field definitions
#define RPL_TX_UP_DLY5_MASK   ((uint32_t)0xFFC00000)
#define RPL_TX_UP_DLY5_LSB    22
#define RPL_TX_UP_DLY5_WIDTH  ((uint32_t)0x0000000A)
#define RPL_TX_UP_DLY4_MASK   ((uint32_t)0x001FF800)
#define RPL_TX_UP_DLY4_LSB    11
#define RPL_TX_UP_DLY4_WIDTH  ((uint32_t)0x0000000A)
#define RPL_TX_UP_DLY3_MASK   ((uint32_t)0x000003FF)
#define RPL_TX_UP_DLY3_LSB    0
#define RPL_TX_UP_DLY3_WIDTH  ((uint32_t)0x0000000A)

#define RPL_TX_UP_DLY5_RST    0x1
#define RPL_TX_UP_DLY4_RST    0x1
#define RPL_TX_UP_DLY3_RST    0x19

__INLINE void rpl_rftxup_dly1_pack(uint16_t txupdly5, uint16_t txupdly4, uint16_t txupdly3)
{
    ASSERT_ERR((((uint32_t)txupdly5 << 22) & ~((uint32_t)0xFFC00000)) == 0);
    ASSERT_ERR((((uint32_t)txupdly4 << 11) & ~((uint32_t)0x001FF800)) == 0);
    ASSERT_ERR((((uint32_t)txupdly3 << 0) & ~((uint32_t)0x000003FF)) == 0);
    REG_RPL_WR(RPL_RFTXUP_DLY1_ADDR, ((uint32_t)txupdly5 << 22) | ((uint32_t)txupdly4 << 11) | ((uint32_t)txupdly3 << 0));
}

__INLINE void rpl_rftxup_dly1_unpack(uint16_t *txupdly5, uint16_t *txupdly4, uint16_t *txupdly3)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXUP_DLY1_ADDR);

    *txupdly5 = (localVal & ((uint32_t)0xFFC00000)) >> 22;
    *txupdly4 = (localVal & ((uint32_t)0x001FF800)) >> 11;
    *txupdly3 = (localVal & ((uint32_t)0x000003FF)) >> 0;
}

__INLINE uint16_t rpl_tx_up_dly5_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXUP_DLY1_ADDR);
    return ((localVal & ((uint32_t)0xFFC00000)) >> 22);
}

__INLINE void rpl_tx_up_dly5_setf(uint16_t txupdly5)
{
    ASSERT_ERR((((uint32_t)txupdly5 << 22) & ~((uint32_t)0xFFC00000)) == 0);
    REG_RPL_WR(RPL_RFTXUP_DLY1_ADDR, (REG_RPL_RD(RPL_RFTXUP_DLY1_ADDR) & ~((uint32_t)0xFFC00000)) | ((uint32_t)txupdly5 << 22));
}

__INLINE uint16_t rpl_tx_up_dly4_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXUP_DLY1_ADDR);
    return ((localVal & ((uint32_t)0x001FF800)) >> 11);
}

__INLINE void rpl_tx_up_dly4_setf(uint16_t txupdly4)
{
    ASSERT_ERR((((uint32_t)txupdly4 << 11) & ~((uint32_t)0x001FF800)) == 0);
    REG_RPL_WR(RPL_RFTXUP_DLY1_ADDR, (REG_RPL_RD(RPL_RFTXUP_DLY1_ADDR) & ~((uint32_t)0x001FF800)) | ((uint32_t)txupdly4 << 11));
}

__INLINE uint16_t rpl_tx_up_dly3_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXUP_DLY1_ADDR);
    return ((localVal & ((uint32_t)0x000003FF)) >> 0);
}

__INLINE void rpl_tx_up_dly3_setf(uint16_t txupdly3)
{
    ASSERT_ERR((((uint32_t)txupdly3 << 0) & ~((uint32_t)0x000003FF)) == 0);
    REG_RPL_WR(RPL_RFTXUP_DLY1_ADDR, (REG_RPL_RD(RPL_RFTXUP_DLY1_ADDR) & ~((uint32_t)0x000003FF)) | ((uint32_t)txupdly3 << 0));
}

/**
 * @brief RFTXUP_DLY2 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:22           TX_UP_DLY8   0x1
 *  20:11           TX_UP_DLY7   0x1
 *  09:00           TX_UP_DLY6   0x1
 * </pre>
 */
#define RPL_RFTXUP_DLY2_ADDR   0x00000005
#define RPL_RFTXUP_DLY2_OFFSET 0x00000005
#define RPL_RFTXUP_DLY2_INDEX  0x00000005
#define RPL_RFTXUP_DLY2_RESET  0x00400801

__INLINE uint32_t rpl_rftxup_dly2_get(void)
{
    return REG_RPL_RD(RPL_RFTXUP_DLY2_ADDR);
}

__INLINE void rpl_rftxup_dly2_set(uint32_t value)
{
    REG_RPL_WR(RPL_RFTXUP_DLY2_ADDR, value);
}

// field definitions
#define RPL_TX_UP_DLY8_MASK   ((uint32_t)0xFFC00000)
#define RPL_TX_UP_DLY8_LSB    22
#define RPL_TX_UP_DLY8_WIDTH  ((uint32_t)0x0000000A)
#define RPL_TX_UP_DLY7_MASK   ((uint32_t)0x001FF800)
#define RPL_TX_UP_DLY7_LSB    11
#define RPL_TX_UP_DLY7_WIDTH  ((uint32_t)0x0000000A)
#define RPL_TX_UP_DLY6_MASK   ((uint32_t)0x000003FF)
#define RPL_TX_UP_DLY6_LSB    0
#define RPL_TX_UP_DLY6_WIDTH  ((uint32_t)0x0000000A)

#define RPL_TX_UP_DLY8_RST    0x1
#define RPL_TX_UP_DLY7_RST    0x1
#define RPL_TX_UP_DLY6_RST    0x1

__INLINE void rpl_rftxup_dly2_pack(uint16_t txupdly8, uint16_t txupdly7, uint16_t txupdly6)
{
    ASSERT_ERR((((uint32_t)txupdly8 << 22) & ~((uint32_t)0xFFC00000)) == 0);
    ASSERT_ERR((((uint32_t)txupdly7 << 11) & ~((uint32_t)0x001FF800)) == 0);
    ASSERT_ERR((((uint32_t)txupdly6 << 0) & ~((uint32_t)0x000003FF)) == 0);
    REG_RPL_WR(RPL_RFTXUP_DLY2_ADDR, ((uint32_t)txupdly8 << 22) | ((uint32_t)txupdly7 << 11) | ((uint32_t)txupdly6 << 0));
}

__INLINE void rpl_rftxup_dly2_unpack(uint16_t *txupdly8, uint16_t *txupdly7, uint16_t *txupdly6)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXUP_DLY2_ADDR);

    *txupdly8 = (localVal & ((uint32_t)0xFFC00000)) >> 22;
    *txupdly7 = (localVal & ((uint32_t)0x001FF800)) >> 11;
    *txupdly6 = (localVal & ((uint32_t)0x000003FF)) >> 0;
}

__INLINE uint16_t rpl_tx_up_dly8_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXUP_DLY2_ADDR);
    return ((localVal & ((uint32_t)0xFFC00000)) >> 22);
}

__INLINE void rpl_tx_up_dly8_setf(uint16_t txupdly8)
{
    ASSERT_ERR((((uint32_t)txupdly8 << 22) & ~((uint32_t)0xFFC00000)) == 0);
    REG_RPL_WR(RPL_RFTXUP_DLY2_ADDR, (REG_RPL_RD(RPL_RFTXUP_DLY2_ADDR) & ~((uint32_t)0xFFC00000)) | ((uint32_t)txupdly8 << 22));
}

__INLINE uint16_t rpl_tx_up_dly7_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXUP_DLY2_ADDR);
    return ((localVal & ((uint32_t)0x001FF800)) >> 11);
}

__INLINE void rpl_tx_up_dly7_setf(uint16_t txupdly7)
{
    ASSERT_ERR((((uint32_t)txupdly7 << 11) & ~((uint32_t)0x001FF800)) == 0);
    REG_RPL_WR(RPL_RFTXUP_DLY2_ADDR, (REG_RPL_RD(RPL_RFTXUP_DLY2_ADDR) & ~((uint32_t)0x001FF800)) | ((uint32_t)txupdly7 << 11));
}

__INLINE uint16_t rpl_tx_up_dly6_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXUP_DLY2_ADDR);
    return ((localVal & ((uint32_t)0x000003FF)) >> 0);
}

__INLINE void rpl_tx_up_dly6_setf(uint16_t txupdly6)
{
    ASSERT_ERR((((uint32_t)txupdly6 << 0) & ~((uint32_t)0x000003FF)) == 0);
    REG_RPL_WR(RPL_RFTXUP_DLY2_ADDR, (REG_RPL_RD(RPL_RFTXUP_DLY2_ADDR) & ~((uint32_t)0x000003FF)) | ((uint32_t)txupdly6 << 0));
}

/**
 * @brief RFTXDN_DLY0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:22           TX_DN_DLY2   0x1
 *  20:11           TX_DN_DLY1   0x19
 *  09:00           TX_DN_DLY0   0x19
 * </pre>
 */
#define RPL_RFTXDN_DLY0_ADDR   0x00000006
#define RPL_RFTXDN_DLY0_OFFSET 0x00000006
#define RPL_RFTXDN_DLY0_INDEX  0x00000006
#define RPL_RFTXDN_DLY0_RESET  0x0040C819

__INLINE uint32_t rpl_rftxdn_dly0_get(void)
{
    return REG_RPL_RD(RPL_RFTXDN_DLY0_ADDR);
}

__INLINE void rpl_rftxdn_dly0_set(uint32_t value)
{
    REG_RPL_WR(RPL_RFTXDN_DLY0_ADDR, value);
}

// field definitions
#define RPL_TX_DN_DLY2_MASK   ((uint32_t)0xFFC00000)
#define RPL_TX_DN_DLY2_LSB    22
#define RPL_TX_DN_DLY2_WIDTH  ((uint32_t)0x0000000A)
#define RPL_TX_DN_DLY1_MASK   ((uint32_t)0x001FF800)
#define RPL_TX_DN_DLY1_LSB    11
#define RPL_TX_DN_DLY1_WIDTH  ((uint32_t)0x0000000A)
#define RPL_TX_DN_DLY0_MASK   ((uint32_t)0x000003FF)
#define RPL_TX_DN_DLY0_LSB    0
#define RPL_TX_DN_DLY0_WIDTH  ((uint32_t)0x0000000A)

#define RPL_TX_DN_DLY2_RST    0x1
#define RPL_TX_DN_DLY1_RST    0x19
#define RPL_TX_DN_DLY0_RST    0x19

__INLINE void rpl_rftxdn_dly0_pack(uint16_t txdndly2, uint16_t txdndly1, uint16_t txdndly0)
{
    ASSERT_ERR((((uint32_t)txdndly2 << 22) & ~((uint32_t)0xFFC00000)) == 0);
    ASSERT_ERR((((uint32_t)txdndly1 << 11) & ~((uint32_t)0x001FF800)) == 0);
    ASSERT_ERR((((uint32_t)txdndly0 << 0) & ~((uint32_t)0x000003FF)) == 0);
    REG_RPL_WR(RPL_RFTXDN_DLY0_ADDR, ((uint32_t)txdndly2 << 22) | ((uint32_t)txdndly1 << 11) | ((uint32_t)txdndly0 << 0));
}

__INLINE void rpl_rftxdn_dly0_unpack(uint16_t *txdndly2, uint16_t *txdndly1, uint16_t *txdndly0)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXDN_DLY0_ADDR);

    *txdndly2 = (localVal & ((uint32_t)0xFFC00000)) >> 22;
    *txdndly1 = (localVal & ((uint32_t)0x001FF800)) >> 11;
    *txdndly0 = (localVal & ((uint32_t)0x000003FF)) >> 0;
}

__INLINE uint16_t rpl_tx_dn_dly2_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXDN_DLY0_ADDR);
    return ((localVal & ((uint32_t)0xFFC00000)) >> 22);
}

__INLINE void rpl_tx_dn_dly2_setf(uint16_t txdndly2)
{
    ASSERT_ERR((((uint32_t)txdndly2 << 22) & ~((uint32_t)0xFFC00000)) == 0);
    REG_RPL_WR(RPL_RFTXDN_DLY0_ADDR, (REG_RPL_RD(RPL_RFTXDN_DLY0_ADDR) & ~((uint32_t)0xFFC00000)) | ((uint32_t)txdndly2 << 22));
}

__INLINE uint16_t rpl_tx_dn_dly1_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXDN_DLY0_ADDR);
    return ((localVal & ((uint32_t)0x001FF800)) >> 11);
}

__INLINE void rpl_tx_dn_dly1_setf(uint16_t txdndly1)
{
    ASSERT_ERR((((uint32_t)txdndly1 << 11) & ~((uint32_t)0x001FF800)) == 0);
    REG_RPL_WR(RPL_RFTXDN_DLY0_ADDR, (REG_RPL_RD(RPL_RFTXDN_DLY0_ADDR) & ~((uint32_t)0x001FF800)) | ((uint32_t)txdndly1 << 11));
}

__INLINE uint16_t rpl_tx_dn_dly0_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXDN_DLY0_ADDR);
    return ((localVal & ((uint32_t)0x000003FF)) >> 0);
}

__INLINE void rpl_tx_dn_dly0_setf(uint16_t txdndly0)
{
    ASSERT_ERR((((uint32_t)txdndly0 << 0) & ~((uint32_t)0x000003FF)) == 0);
    REG_RPL_WR(RPL_RFTXDN_DLY0_ADDR, (REG_RPL_RD(RPL_RFTXDN_DLY0_ADDR) & ~((uint32_t)0x000003FF)) | ((uint32_t)txdndly0 << 0));
}

/**
 * @brief RFTXDN_DLY1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:22           TX_DN_DLY5   0x1
 *  20:11           TX_DN_DLY4   0x1
 *  09:00           TX_DN_DLY3   0x1
 * </pre>
 */
#define RPL_RFTXDN_DLY1_ADDR   0x00000007
#define RPL_RFTXDN_DLY1_OFFSET 0x00000007
#define RPL_RFTXDN_DLY1_INDEX  0x00000007
#define RPL_RFTXDN_DLY1_RESET  0x00400801

__INLINE uint32_t rpl_rftxdn_dly1_get(void)
{
    return REG_RPL_RD(RPL_RFTXDN_DLY1_ADDR);
}

__INLINE void rpl_rftxdn_dly1_set(uint32_t value)
{
    REG_RPL_WR(RPL_RFTXDN_DLY1_ADDR, value);
}

// field definitions
#define RPL_TX_DN_DLY5_MASK   ((uint32_t)0xFFC00000)
#define RPL_TX_DN_DLY5_LSB    22
#define RPL_TX_DN_DLY5_WIDTH  ((uint32_t)0x0000000A)
#define RPL_TX_DN_DLY4_MASK   ((uint32_t)0x001FF800)
#define RPL_TX_DN_DLY4_LSB    11
#define RPL_TX_DN_DLY4_WIDTH  ((uint32_t)0x0000000A)
#define RPL_TX_DN_DLY3_MASK   ((uint32_t)0x000003FF)
#define RPL_TX_DN_DLY3_LSB    0
#define RPL_TX_DN_DLY3_WIDTH  ((uint32_t)0x0000000A)

#define RPL_TX_DN_DLY5_RST    0x1
#define RPL_TX_DN_DLY4_RST    0x1
#define RPL_TX_DN_DLY3_RST    0x1

__INLINE void rpl_rftxdn_dly1_pack(uint16_t txdndly5, uint16_t txdndly4, uint16_t txdndly3)
{
    ASSERT_ERR((((uint32_t)txdndly5 << 22) & ~((uint32_t)0xFFC00000)) == 0);
    ASSERT_ERR((((uint32_t)txdndly4 << 11) & ~((uint32_t)0x001FF800)) == 0);
    ASSERT_ERR((((uint32_t)txdndly3 << 0) & ~((uint32_t)0x000003FF)) == 0);
    REG_RPL_WR(RPL_RFTXDN_DLY1_ADDR, ((uint32_t)txdndly5 << 22) | ((uint32_t)txdndly4 << 11) | ((uint32_t)txdndly3 << 0));
}

__INLINE void rpl_rftxdn_dly1_unpack(uint16_t *txdndly5, uint16_t *txdndly4, uint16_t *txdndly3)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXDN_DLY1_ADDR);

    *txdndly5 = (localVal & ((uint32_t)0xFFC00000)) >> 22;
    *txdndly4 = (localVal & ((uint32_t)0x001FF800)) >> 11;
    *txdndly3 = (localVal & ((uint32_t)0x000003FF)) >> 0;
}

__INLINE uint16_t rpl_tx_dn_dly5_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXDN_DLY1_ADDR);
    return ((localVal & ((uint32_t)0xFFC00000)) >> 22);
}

__INLINE void rpl_tx_dn_dly5_setf(uint16_t txdndly5)
{
    ASSERT_ERR((((uint32_t)txdndly5 << 22) & ~((uint32_t)0xFFC00000)) == 0);
    REG_RPL_WR(RPL_RFTXDN_DLY1_ADDR, (REG_RPL_RD(RPL_RFTXDN_DLY1_ADDR) & ~((uint32_t)0xFFC00000)) | ((uint32_t)txdndly5 << 22));
}

__INLINE uint16_t rpl_tx_dn_dly4_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXDN_DLY1_ADDR);
    return ((localVal & ((uint32_t)0x001FF800)) >> 11);
}

__INLINE void rpl_tx_dn_dly4_setf(uint16_t txdndly4)
{
    ASSERT_ERR((((uint32_t)txdndly4 << 11) & ~((uint32_t)0x001FF800)) == 0);
    REG_RPL_WR(RPL_RFTXDN_DLY1_ADDR, (REG_RPL_RD(RPL_RFTXDN_DLY1_ADDR) & ~((uint32_t)0x001FF800)) | ((uint32_t)txdndly4 << 11));
}

__INLINE uint16_t rpl_tx_dn_dly3_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXDN_DLY1_ADDR);
    return ((localVal & ((uint32_t)0x000003FF)) >> 0);
}

__INLINE void rpl_tx_dn_dly3_setf(uint16_t txdndly3)
{
    ASSERT_ERR((((uint32_t)txdndly3 << 0) & ~((uint32_t)0x000003FF)) == 0);
    REG_RPL_WR(RPL_RFTXDN_DLY1_ADDR, (REG_RPL_RD(RPL_RFTXDN_DLY1_ADDR) & ~((uint32_t)0x000003FF)) | ((uint32_t)txdndly3 << 0));
}

/**
 * @brief RFTXDN_DLY2 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:22           TX_DN_DLY8   0x1
 *  20:11           TX_DN_DLY7   0x1
 *  09:00           TX_DN_DLY6   0x1
 * </pre>
 */
#define RPL_RFTXDN_DLY2_ADDR   0x00000008
#define RPL_RFTXDN_DLY2_OFFSET 0x00000008
#define RPL_RFTXDN_DLY2_INDEX  0x00000008
#define RPL_RFTXDN_DLY2_RESET  0x00400801

__INLINE uint32_t rpl_rftxdn_dly2_get(void)
{
    return REG_RPL_RD(RPL_RFTXDN_DLY2_ADDR);
}

__INLINE void rpl_rftxdn_dly2_set(uint32_t value)
{
    REG_RPL_WR(RPL_RFTXDN_DLY2_ADDR, value);
}

// field definitions
#define RPL_TX_DN_DLY8_MASK   ((uint32_t)0xFFC00000)
#define RPL_TX_DN_DLY8_LSB    22
#define RPL_TX_DN_DLY8_WIDTH  ((uint32_t)0x0000000A)
#define RPL_TX_DN_DLY7_MASK   ((uint32_t)0x001FF800)
#define RPL_TX_DN_DLY7_LSB    11
#define RPL_TX_DN_DLY7_WIDTH  ((uint32_t)0x0000000A)
#define RPL_TX_DN_DLY6_MASK   ((uint32_t)0x000003FF)
#define RPL_TX_DN_DLY6_LSB    0
#define RPL_TX_DN_DLY6_WIDTH  ((uint32_t)0x0000000A)

#define RPL_TX_DN_DLY8_RST    0x1
#define RPL_TX_DN_DLY7_RST    0x1
#define RPL_TX_DN_DLY6_RST    0x1

__INLINE void rpl_rftxdn_dly2_pack(uint16_t txdndly8, uint16_t txdndly7, uint16_t txdndly6)
{
    ASSERT_ERR((((uint32_t)txdndly8 << 22) & ~((uint32_t)0xFFC00000)) == 0);
    ASSERT_ERR((((uint32_t)txdndly7 << 11) & ~((uint32_t)0x001FF800)) == 0);
    ASSERT_ERR((((uint32_t)txdndly6 << 0) & ~((uint32_t)0x000003FF)) == 0);
    REG_RPL_WR(RPL_RFTXDN_DLY2_ADDR, ((uint32_t)txdndly8 << 22) | ((uint32_t)txdndly7 << 11) | ((uint32_t)txdndly6 << 0));
}

__INLINE void rpl_rftxdn_dly2_unpack(uint16_t *txdndly8, uint16_t *txdndly7, uint16_t *txdndly6)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXDN_DLY2_ADDR);

    *txdndly8 = (localVal & ((uint32_t)0xFFC00000)) >> 22;
    *txdndly7 = (localVal & ((uint32_t)0x001FF800)) >> 11;
    *txdndly6 = (localVal & ((uint32_t)0x000003FF)) >> 0;
}

__INLINE uint16_t rpl_tx_dn_dly8_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXDN_DLY2_ADDR);
    return ((localVal & ((uint32_t)0xFFC00000)) >> 22);
}

__INLINE void rpl_tx_dn_dly8_setf(uint16_t txdndly8)
{
    ASSERT_ERR((((uint32_t)txdndly8 << 22) & ~((uint32_t)0xFFC00000)) == 0);
    REG_RPL_WR(RPL_RFTXDN_DLY2_ADDR, (REG_RPL_RD(RPL_RFTXDN_DLY2_ADDR) & ~((uint32_t)0xFFC00000)) | ((uint32_t)txdndly8 << 22));
}

__INLINE uint16_t rpl_tx_dn_dly7_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXDN_DLY2_ADDR);
    return ((localVal & ((uint32_t)0x001FF800)) >> 11);
}

__INLINE void rpl_tx_dn_dly7_setf(uint16_t txdndly7)
{
    ASSERT_ERR((((uint32_t)txdndly7 << 11) & ~((uint32_t)0x001FF800)) == 0);
    REG_RPL_WR(RPL_RFTXDN_DLY2_ADDR, (REG_RPL_RD(RPL_RFTXDN_DLY2_ADDR) & ~((uint32_t)0x001FF800)) | ((uint32_t)txdndly7 << 11));
}

__INLINE uint16_t rpl_tx_dn_dly6_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXDN_DLY2_ADDR);
    return ((localVal & ((uint32_t)0x000003FF)) >> 0);
}

__INLINE void rpl_tx_dn_dly6_setf(uint16_t txdndly6)
{
    ASSERT_ERR((((uint32_t)txdndly6 << 0) & ~((uint32_t)0x000003FF)) == 0);
    REG_RPL_WR(RPL_RFTXDN_DLY2_ADDR, (REG_RPL_RD(RPL_RFTXDN_DLY2_ADDR) & ~((uint32_t)0x000003FF)) | ((uint32_t)txdndly6 << 0));
}

/**
 * @brief RFRXUP_DLY0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:22           RX_UP_DLY2   0x19
 *  20:11           RX_UP_DLY1   0x206
 *  09:00           RX_UP_DLY0   0x1
 * </pre>
 */
#define RPL_RFRXUP_DLY0_ADDR   0x00000009
#define RPL_RFRXUP_DLY0_OFFSET 0x00000009
#define RPL_RFRXUP_DLY0_INDEX  0x00000009
#define RPL_RFRXUP_DLY0_RESET  0x06503001

__INLINE uint32_t rpl_rfrxup_dly0_get(void)
{
    return REG_RPL_RD(RPL_RFRXUP_DLY0_ADDR);
}

__INLINE void rpl_rfrxup_dly0_set(uint32_t value)
{
    REG_RPL_WR(RPL_RFRXUP_DLY0_ADDR, value);
}

// field definitions
#define RPL_RX_UP_DLY2_MASK   ((uint32_t)0xFFC00000)
#define RPL_RX_UP_DLY2_LSB    22
#define RPL_RX_UP_DLY2_WIDTH  ((uint32_t)0x0000000A)
#define RPL_RX_UP_DLY1_MASK   ((uint32_t)0x001FF800)
#define RPL_RX_UP_DLY1_LSB    11
#define RPL_RX_UP_DLY1_WIDTH  ((uint32_t)0x0000000A)
#define RPL_RX_UP_DLY0_MASK   ((uint32_t)0x000003FF)
#define RPL_RX_UP_DLY0_LSB    0
#define RPL_RX_UP_DLY0_WIDTH  ((uint32_t)0x0000000A)

#define RPL_RX_UP_DLY2_RST    0x19
#define RPL_RX_UP_DLY1_RST    0x206
#define RPL_RX_UP_DLY0_RST    0x1

__INLINE void rpl_rfrxup_dly0_pack(uint16_t rxupdly2, uint16_t rxupdly1, uint16_t rxupdly0)
{
    ASSERT_ERR((((uint32_t)rxupdly2 << 22) & ~((uint32_t)0xFFC00000)) == 0);
    ASSERT_ERR((((uint32_t)rxupdly1 << 11) & ~((uint32_t)0x001FF800)) == 0);
    ASSERT_ERR((((uint32_t)rxupdly0 << 0) & ~((uint32_t)0x000003FF)) == 0);
    REG_RPL_WR(RPL_RFRXUP_DLY0_ADDR, ((uint32_t)rxupdly2 << 22) | ((uint32_t)rxupdly1 << 11) | ((uint32_t)rxupdly0 << 0));
}

__INLINE void rpl_rfrxup_dly0_unpack(uint16_t *rxupdly2, uint16_t *rxupdly1, uint16_t *rxupdly0)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFRXUP_DLY0_ADDR);

    *rxupdly2 = (localVal & ((uint32_t)0xFFC00000)) >> 22;
    *rxupdly1 = (localVal & ((uint32_t)0x001FF800)) >> 11;
    *rxupdly0 = (localVal & ((uint32_t)0x000003FF)) >> 0;
}

__INLINE uint16_t rpl_rx_up_dly2_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFRXUP_DLY0_ADDR);
    return ((localVal & ((uint32_t)0xFFC00000)) >> 22);
}

__INLINE void rpl_rx_up_dly2_setf(uint16_t rxupdly2)
{
    ASSERT_ERR((((uint32_t)rxupdly2 << 22) & ~((uint32_t)0xFFC00000)) == 0);
    REG_RPL_WR(RPL_RFRXUP_DLY0_ADDR, (REG_RPL_RD(RPL_RFRXUP_DLY0_ADDR) & ~((uint32_t)0xFFC00000)) | ((uint32_t)rxupdly2 << 22));
}

__INLINE uint16_t rpl_rx_up_dly1_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFRXUP_DLY0_ADDR);
    return ((localVal & ((uint32_t)0x001FF800)) >> 11);
}

__INLINE void rpl_rx_up_dly1_setf(uint16_t rxupdly1)
{
    ASSERT_ERR((((uint32_t)rxupdly1 << 11) & ~((uint32_t)0x001FF800)) == 0);
    REG_RPL_WR(RPL_RFRXUP_DLY0_ADDR, (REG_RPL_RD(RPL_RFRXUP_DLY0_ADDR) & ~((uint32_t)0x001FF800)) | ((uint32_t)rxupdly1 << 11));
}

__INLINE uint16_t rpl_rx_up_dly0_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFRXUP_DLY0_ADDR);
    return ((localVal & ((uint32_t)0x000003FF)) >> 0);
}

__INLINE void rpl_rx_up_dly0_setf(uint16_t rxupdly0)
{
    ASSERT_ERR((((uint32_t)rxupdly0 << 0) & ~((uint32_t)0x000003FF)) == 0);
    REG_RPL_WR(RPL_RFRXUP_DLY0_ADDR, (REG_RPL_RD(RPL_RFRXUP_DLY0_ADDR) & ~((uint32_t)0x000003FF)) | ((uint32_t)rxupdly0 << 0));
}

/**
 * @brief RFRXUP_DLY1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:22           RX_UP_DLY5   0x1
 *  20:11           RX_UP_DLY4   0x1
 *  09:00           RX_UP_DLY3   0x19
 * </pre>
 */
#define RPL_RFRXUP_DLY1_ADDR   0x0000000A
#define RPL_RFRXUP_DLY1_OFFSET 0x0000000A
#define RPL_RFRXUP_DLY1_INDEX  0x0000000A
#define RPL_RFRXUP_DLY1_RESET  0x00400819

__INLINE uint32_t rpl_rfrxup_dly1_get(void)
{
    return REG_RPL_RD(RPL_RFRXUP_DLY1_ADDR);
}

__INLINE void rpl_rfrxup_dly1_set(uint32_t value)
{
    REG_RPL_WR(RPL_RFRXUP_DLY1_ADDR, value);
}

// field definitions
#define RPL_RX_UP_DLY5_MASK   ((uint32_t)0xFFC00000)
#define RPL_RX_UP_DLY5_LSB    22
#define RPL_RX_UP_DLY5_WIDTH  ((uint32_t)0x0000000A)
#define RPL_RX_UP_DLY4_MASK   ((uint32_t)0x001FF800)
#define RPL_RX_UP_DLY4_LSB    11
#define RPL_RX_UP_DLY4_WIDTH  ((uint32_t)0x0000000A)
#define RPL_RX_UP_DLY3_MASK   ((uint32_t)0x000003FF)
#define RPL_RX_UP_DLY3_LSB    0
#define RPL_RX_UP_DLY3_WIDTH  ((uint32_t)0x0000000A)

#define RPL_RX_UP_DLY5_RST    0x1
#define RPL_RX_UP_DLY4_RST    0x1
#define RPL_RX_UP_DLY3_RST    0x19

__INLINE void rpl_rfrxup_dly1_pack(uint16_t rxupdly5, uint16_t rxupdly4, uint16_t rxupdly3)
{
    ASSERT_ERR((((uint32_t)rxupdly5 << 22) & ~((uint32_t)0xFFC00000)) == 0);
    ASSERT_ERR((((uint32_t)rxupdly4 << 11) & ~((uint32_t)0x001FF800)) == 0);
    ASSERT_ERR((((uint32_t)rxupdly3 << 0) & ~((uint32_t)0x000003FF)) == 0);
    REG_RPL_WR(RPL_RFRXUP_DLY1_ADDR, ((uint32_t)rxupdly5 << 22) | ((uint32_t)rxupdly4 << 11) | ((uint32_t)rxupdly3 << 0));
}

__INLINE void rpl_rfrxup_dly1_unpack(uint16_t *rxupdly5, uint16_t *rxupdly4, uint16_t *rxupdly3)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFRXUP_DLY1_ADDR);

    *rxupdly5 = (localVal & ((uint32_t)0xFFC00000)) >> 22;
    *rxupdly4 = (localVal & ((uint32_t)0x001FF800)) >> 11;
    *rxupdly3 = (localVal & ((uint32_t)0x000003FF)) >> 0;
}

__INLINE uint16_t rpl_rx_up_dly5_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFRXUP_DLY1_ADDR);
    return ((localVal & ((uint32_t)0xFFC00000)) >> 22);
}

__INLINE void rpl_rx_up_dly5_setf(uint16_t rxupdly5)
{
    ASSERT_ERR((((uint32_t)rxupdly5 << 22) & ~((uint32_t)0xFFC00000)) == 0);
    REG_RPL_WR(RPL_RFRXUP_DLY1_ADDR, (REG_RPL_RD(RPL_RFRXUP_DLY1_ADDR) & ~((uint32_t)0xFFC00000)) | ((uint32_t)rxupdly5 << 22));
}

__INLINE uint16_t rpl_rx_up_dly4_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFRXUP_DLY1_ADDR);
    return ((localVal & ((uint32_t)0x001FF800)) >> 11);
}

__INLINE void rpl_rx_up_dly4_setf(uint16_t rxupdly4)
{
    ASSERT_ERR((((uint32_t)rxupdly4 << 11) & ~((uint32_t)0x001FF800)) == 0);
    REG_RPL_WR(RPL_RFRXUP_DLY1_ADDR, (REG_RPL_RD(RPL_RFRXUP_DLY1_ADDR) & ~((uint32_t)0x001FF800)) | ((uint32_t)rxupdly4 << 11));
}

__INLINE uint16_t rpl_rx_up_dly3_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFRXUP_DLY1_ADDR);
    return ((localVal & ((uint32_t)0x000003FF)) >> 0);
}

__INLINE void rpl_rx_up_dly3_setf(uint16_t rxupdly3)
{
    ASSERT_ERR((((uint32_t)rxupdly3 << 0) & ~((uint32_t)0x000003FF)) == 0);
    REG_RPL_WR(RPL_RFRXUP_DLY1_ADDR, (REG_RPL_RD(RPL_RFRXUP_DLY1_ADDR) & ~((uint32_t)0x000003FF)) | ((uint32_t)rxupdly3 << 0));
}

/**
 * @brief RFRXUP_DLY2 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:22           RX_UP_DLY8   0x1
 *  20:11           RX_UP_DLY7   0x1
 *  09:00           RX_UP_DLY6   0x1
 * </pre>
 */
#define RPL_RFRXUP_DLY2_ADDR   0x0000000B
#define RPL_RFRXUP_DLY2_OFFSET 0x0000000B
#define RPL_RFRXUP_DLY2_INDEX  0x0000000B
#define RPL_RFRXUP_DLY2_RESET  0x00400801

__INLINE uint32_t rpl_rfrxup_dly2_get(void)
{
    return REG_RPL_RD(RPL_RFRXUP_DLY2_ADDR);
}

__INLINE void rpl_rfrxup_dly2_set(uint32_t value)
{
    REG_RPL_WR(RPL_RFRXUP_DLY2_ADDR, value);
}

// field definitions
#define RPL_RX_UP_DLY8_MASK   ((uint32_t)0xFFC00000)
#define RPL_RX_UP_DLY8_LSB    22
#define RPL_RX_UP_DLY8_WIDTH  ((uint32_t)0x0000000A)
#define RPL_RX_UP_DLY7_MASK   ((uint32_t)0x001FF800)
#define RPL_RX_UP_DLY7_LSB    11
#define RPL_RX_UP_DLY7_WIDTH  ((uint32_t)0x0000000A)
#define RPL_RX_UP_DLY6_MASK   ((uint32_t)0x000003FF)
#define RPL_RX_UP_DLY6_LSB    0
#define RPL_RX_UP_DLY6_WIDTH  ((uint32_t)0x0000000A)

#define RPL_RX_UP_DLY8_RST    0x1
#define RPL_RX_UP_DLY7_RST    0x1
#define RPL_RX_UP_DLY6_RST    0x1

__INLINE void rpl_rfrxup_dly2_pack(uint16_t rxupdly8, uint16_t rxupdly7, uint16_t rxupdly6)
{
    ASSERT_ERR((((uint32_t)rxupdly8 << 22) & ~((uint32_t)0xFFC00000)) == 0);
    ASSERT_ERR((((uint32_t)rxupdly7 << 11) & ~((uint32_t)0x001FF800)) == 0);
    ASSERT_ERR((((uint32_t)rxupdly6 << 0) & ~((uint32_t)0x000003FF)) == 0);
    REG_RPL_WR(RPL_RFRXUP_DLY2_ADDR, ((uint32_t)rxupdly8 << 22) | ((uint32_t)rxupdly7 << 11) | ((uint32_t)rxupdly6 << 0));
}

__INLINE void rpl_rfrxup_dly2_unpack(uint16_t *rxupdly8, uint16_t *rxupdly7, uint16_t *rxupdly6)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFRXUP_DLY2_ADDR);

    *rxupdly8 = (localVal & ((uint32_t)0xFFC00000)) >> 22;
    *rxupdly7 = (localVal & ((uint32_t)0x001FF800)) >> 11;
    *rxupdly6 = (localVal & ((uint32_t)0x000003FF)) >> 0;
}

__INLINE uint16_t rpl_rx_up_dly8_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFRXUP_DLY2_ADDR);
    return ((localVal & ((uint32_t)0xFFC00000)) >> 22);
}

__INLINE void rpl_rx_up_dly8_setf(uint16_t rxupdly8)
{
    ASSERT_ERR((((uint32_t)rxupdly8 << 22) & ~((uint32_t)0xFFC00000)) == 0);
    REG_RPL_WR(RPL_RFRXUP_DLY2_ADDR, (REG_RPL_RD(RPL_RFRXUP_DLY2_ADDR) & ~((uint32_t)0xFFC00000)) | ((uint32_t)rxupdly8 << 22));
}

__INLINE uint16_t rpl_rx_up_dly7_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFRXUP_DLY2_ADDR);
    return ((localVal & ((uint32_t)0x001FF800)) >> 11);
}

__INLINE void rpl_rx_up_dly7_setf(uint16_t rxupdly7)
{
    ASSERT_ERR((((uint32_t)rxupdly7 << 11) & ~((uint32_t)0x001FF800)) == 0);
    REG_RPL_WR(RPL_RFRXUP_DLY2_ADDR, (REG_RPL_RD(RPL_RFRXUP_DLY2_ADDR) & ~((uint32_t)0x001FF800)) | ((uint32_t)rxupdly7 << 11));
}

__INLINE uint16_t rpl_rx_up_dly6_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFRXUP_DLY2_ADDR);
    return ((localVal & ((uint32_t)0x000003FF)) >> 0);
}

__INLINE void rpl_rx_up_dly6_setf(uint16_t rxupdly6)
{
    ASSERT_ERR((((uint32_t)rxupdly6 << 0) & ~((uint32_t)0x000003FF)) == 0);
    REG_RPL_WR(RPL_RFRXUP_DLY2_ADDR, (REG_RPL_RD(RPL_RFRXUP_DLY2_ADDR) & ~((uint32_t)0x000003FF)) | ((uint32_t)rxupdly6 << 0));
}

/**
 * @brief RFRXDN_DLY0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:22           RX_DN_DLY2   0x1
 *  20:11           RX_DN_DLY1   0x19
 *  09:00           RX_DN_DLY0   0x19
 * </pre>
 */
#define RPL_RFRXDN_DLY0_ADDR   0x0000000C
#define RPL_RFRXDN_DLY0_OFFSET 0x0000000C
#define RPL_RFRXDN_DLY0_INDEX  0x0000000C
#define RPL_RFRXDN_DLY0_RESET  0x0040C819

__INLINE uint32_t rpl_rfrxdn_dly0_get(void)
{
    return REG_RPL_RD(RPL_RFRXDN_DLY0_ADDR);
}

__INLINE void rpl_rfrxdn_dly0_set(uint32_t value)
{
    REG_RPL_WR(RPL_RFRXDN_DLY0_ADDR, value);
}

// field definitions
#define RPL_RX_DN_DLY2_MASK   ((uint32_t)0xFFC00000)
#define RPL_RX_DN_DLY2_LSB    22
#define RPL_RX_DN_DLY2_WIDTH  ((uint32_t)0x0000000A)
#define RPL_RX_DN_DLY1_MASK   ((uint32_t)0x001FF800)
#define RPL_RX_DN_DLY1_LSB    11
#define RPL_RX_DN_DLY1_WIDTH  ((uint32_t)0x0000000A)
#define RPL_RX_DN_DLY0_MASK   ((uint32_t)0x000003FF)
#define RPL_RX_DN_DLY0_LSB    0
#define RPL_RX_DN_DLY0_WIDTH  ((uint32_t)0x0000000A)

#define RPL_RX_DN_DLY2_RST    0x1
#define RPL_RX_DN_DLY1_RST    0x19
#define RPL_RX_DN_DLY0_RST    0x19

__INLINE void rpl_rfrxdn_dly0_pack(uint16_t rxdndly2, uint16_t rxdndly1, uint16_t rxdndly0)
{
    ASSERT_ERR((((uint32_t)rxdndly2 << 22) & ~((uint32_t)0xFFC00000)) == 0);
    ASSERT_ERR((((uint32_t)rxdndly1 << 11) & ~((uint32_t)0x001FF800)) == 0);
    ASSERT_ERR((((uint32_t)rxdndly0 << 0) & ~((uint32_t)0x000003FF)) == 0);
    REG_RPL_WR(RPL_RFRXDN_DLY0_ADDR, ((uint32_t)rxdndly2 << 22) | ((uint32_t)rxdndly1 << 11) | ((uint32_t)rxdndly0 << 0));
}

__INLINE void rpl_rfrxdn_dly0_unpack(uint16_t *rxdndly2, uint16_t *rxdndly1, uint16_t *rxdndly0)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFRXDN_DLY0_ADDR);

    *rxdndly2 = (localVal & ((uint32_t)0xFFC00000)) >> 22;
    *rxdndly1 = (localVal & ((uint32_t)0x001FF800)) >> 11;
    *rxdndly0 = (localVal & ((uint32_t)0x000003FF)) >> 0;
}

__INLINE uint16_t rpl_rx_dn_dly2_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFRXDN_DLY0_ADDR);
    return ((localVal & ((uint32_t)0xFFC00000)) >> 22);
}

__INLINE void rpl_rx_dn_dly2_setf(uint16_t rxdndly2)
{
    ASSERT_ERR((((uint32_t)rxdndly2 << 22) & ~((uint32_t)0xFFC00000)) == 0);
    REG_RPL_WR(RPL_RFRXDN_DLY0_ADDR, (REG_RPL_RD(RPL_RFRXDN_DLY0_ADDR) & ~((uint32_t)0xFFC00000)) | ((uint32_t)rxdndly2 << 22));
}

__INLINE uint16_t rpl_rx_dn_dly1_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFRXDN_DLY0_ADDR);
    return ((localVal & ((uint32_t)0x001FF800)) >> 11);
}

__INLINE void rpl_rx_dn_dly1_setf(uint16_t rxdndly1)
{
    ASSERT_ERR((((uint32_t)rxdndly1 << 11) & ~((uint32_t)0x001FF800)) == 0);
    REG_RPL_WR(RPL_RFRXDN_DLY0_ADDR, (REG_RPL_RD(RPL_RFRXDN_DLY0_ADDR) & ~((uint32_t)0x001FF800)) | ((uint32_t)rxdndly1 << 11));
}

__INLINE uint16_t rpl_rx_dn_dly0_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFRXDN_DLY0_ADDR);
    return ((localVal & ((uint32_t)0x000003FF)) >> 0);
}

__INLINE void rpl_rx_dn_dly0_setf(uint16_t rxdndly0)
{
    ASSERT_ERR((((uint32_t)rxdndly0 << 0) & ~((uint32_t)0x000003FF)) == 0);
    REG_RPL_WR(RPL_RFRXDN_DLY0_ADDR, (REG_RPL_RD(RPL_RFRXDN_DLY0_ADDR) & ~((uint32_t)0x000003FF)) | ((uint32_t)rxdndly0 << 0));
}

/**
 * @brief RFRXDN_DLY1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:22           RX_DN_DLY5   0x1
 *  20:11           RX_DN_DLY4   0x1
 *  09:00           RX_DN_DLY3   0x1
 * </pre>
 */
#define RPL_RFRXDN_DLY1_ADDR   0x0000000D
#define RPL_RFRXDN_DLY1_OFFSET 0x0000000D
#define RPL_RFRXDN_DLY1_INDEX  0x0000000D
#define RPL_RFRXDN_DLY1_RESET  0x00400801

__INLINE uint32_t rpl_rfrxdn_dly1_get(void)
{
    return REG_RPL_RD(RPL_RFRXDN_DLY1_ADDR);
}

__INLINE void rpl_rfrxdn_dly1_set(uint32_t value)
{
    REG_RPL_WR(RPL_RFRXDN_DLY1_ADDR, value);
}

// field definitions
#define RPL_RX_DN_DLY5_MASK   ((uint32_t)0xFFC00000)
#define RPL_RX_DN_DLY5_LSB    22
#define RPL_RX_DN_DLY5_WIDTH  ((uint32_t)0x0000000A)
#define RPL_RX_DN_DLY4_MASK   ((uint32_t)0x001FF800)
#define RPL_RX_DN_DLY4_LSB    11
#define RPL_RX_DN_DLY4_WIDTH  ((uint32_t)0x0000000A)
#define RPL_RX_DN_DLY3_MASK   ((uint32_t)0x000003FF)
#define RPL_RX_DN_DLY3_LSB    0
#define RPL_RX_DN_DLY3_WIDTH  ((uint32_t)0x0000000A)

#define RPL_RX_DN_DLY5_RST    0x1
#define RPL_RX_DN_DLY4_RST    0x1
#define RPL_RX_DN_DLY3_RST    0x1

__INLINE void rpl_rfrxdn_dly1_pack(uint16_t rxdndly5, uint16_t rxdndly4, uint16_t rxdndly3)
{
    ASSERT_ERR((((uint32_t)rxdndly5 << 22) & ~((uint32_t)0xFFC00000)) == 0);
    ASSERT_ERR((((uint32_t)rxdndly4 << 11) & ~((uint32_t)0x001FF800)) == 0);
    ASSERT_ERR((((uint32_t)rxdndly3 << 0) & ~((uint32_t)0x000003FF)) == 0);
    REG_RPL_WR(RPL_RFRXDN_DLY1_ADDR, ((uint32_t)rxdndly5 << 22) | ((uint32_t)rxdndly4 << 11) | ((uint32_t)rxdndly3 << 0));
}

__INLINE void rpl_rfrxdn_dly1_unpack(uint16_t *rxdndly5, uint16_t *rxdndly4, uint16_t *rxdndly3)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFRXDN_DLY1_ADDR);

    *rxdndly5 = (localVal & ((uint32_t)0xFFC00000)) >> 22;
    *rxdndly4 = (localVal & ((uint32_t)0x001FF800)) >> 11;
    *rxdndly3 = (localVal & ((uint32_t)0x000003FF)) >> 0;
}

__INLINE uint16_t rpl_rx_dn_dly5_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFRXDN_DLY1_ADDR);
    return ((localVal & ((uint32_t)0xFFC00000)) >> 22);
}

__INLINE void rpl_rx_dn_dly5_setf(uint16_t rxdndly5)
{
    ASSERT_ERR((((uint32_t)rxdndly5 << 22) & ~((uint32_t)0xFFC00000)) == 0);
    REG_RPL_WR(RPL_RFRXDN_DLY1_ADDR, (REG_RPL_RD(RPL_RFRXDN_DLY1_ADDR) & ~((uint32_t)0xFFC00000)) | ((uint32_t)rxdndly5 << 22));
}

__INLINE uint16_t rpl_rx_dn_dly4_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFRXDN_DLY1_ADDR);
    return ((localVal & ((uint32_t)0x001FF800)) >> 11);
}

__INLINE void rpl_rx_dn_dly4_setf(uint16_t rxdndly4)
{
    ASSERT_ERR((((uint32_t)rxdndly4 << 11) & ~((uint32_t)0x001FF800)) == 0);
    REG_RPL_WR(RPL_RFRXDN_DLY1_ADDR, (REG_RPL_RD(RPL_RFRXDN_DLY1_ADDR) & ~((uint32_t)0x001FF800)) | ((uint32_t)rxdndly4 << 11));
}

__INLINE uint16_t rpl_rx_dn_dly3_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFRXDN_DLY1_ADDR);
    return ((localVal & ((uint32_t)0x000003FF)) >> 0);
}

__INLINE void rpl_rx_dn_dly3_setf(uint16_t rxdndly3)
{
    ASSERT_ERR((((uint32_t)rxdndly3 << 0) & ~((uint32_t)0x000003FF)) == 0);
    REG_RPL_WR(RPL_RFRXDN_DLY1_ADDR, (REG_RPL_RD(RPL_RFRXDN_DLY1_ADDR) & ~((uint32_t)0x000003FF)) | ((uint32_t)rxdndly3 << 0));
}

/**
 * @brief RFRXDN_DLY2 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:22           RX_DN_DLY8   0x1
 *  20:11           RX_DN_DLY7   0x1
 *  09:00           RX_DN_DLY6   0x1
 * </pre>
 */
#define RPL_RFRXDN_DLY2_ADDR   0x0000000E
#define RPL_RFRXDN_DLY2_OFFSET 0x0000000E
#define RPL_RFRXDN_DLY2_INDEX  0x0000000E
#define RPL_RFRXDN_DLY2_RESET  0x00400801

__INLINE uint32_t rpl_rfrxdn_dly2_get(void)
{
    return REG_RPL_RD(RPL_RFRXDN_DLY2_ADDR);
}

__INLINE void rpl_rfrxdn_dly2_set(uint32_t value)
{
    REG_RPL_WR(RPL_RFRXDN_DLY2_ADDR, value);
}

// field definitions
#define RPL_RX_DN_DLY8_MASK   ((uint32_t)0xFFC00000)
#define RPL_RX_DN_DLY8_LSB    22
#define RPL_RX_DN_DLY8_WIDTH  ((uint32_t)0x0000000A)
#define RPL_RX_DN_DLY7_MASK   ((uint32_t)0x001FF800)
#define RPL_RX_DN_DLY7_LSB    11
#define RPL_RX_DN_DLY7_WIDTH  ((uint32_t)0x0000000A)
#define RPL_RX_DN_DLY6_MASK   ((uint32_t)0x000003FF)
#define RPL_RX_DN_DLY6_LSB    0
#define RPL_RX_DN_DLY6_WIDTH  ((uint32_t)0x0000000A)

#define RPL_RX_DN_DLY8_RST    0x1
#define RPL_RX_DN_DLY7_RST    0x1
#define RPL_RX_DN_DLY6_RST    0x1

__INLINE void rpl_rfrxdn_dly2_pack(uint16_t rxdndly8, uint16_t rxdndly7, uint16_t rxdndly6)
{
    ASSERT_ERR((((uint32_t)rxdndly8 << 22) & ~((uint32_t)0xFFC00000)) == 0);
    ASSERT_ERR((((uint32_t)rxdndly7 << 11) & ~((uint32_t)0x001FF800)) == 0);
    ASSERT_ERR((((uint32_t)rxdndly6 << 0) & ~((uint32_t)0x000003FF)) == 0);
    REG_RPL_WR(RPL_RFRXDN_DLY2_ADDR, ((uint32_t)rxdndly8 << 22) | ((uint32_t)rxdndly7 << 11) | ((uint32_t)rxdndly6 << 0));
}

__INLINE void rpl_rfrxdn_dly2_unpack(uint16_t *rxdndly8, uint16_t *rxdndly7, uint16_t *rxdndly6)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFRXDN_DLY2_ADDR);

    *rxdndly8 = (localVal & ((uint32_t)0xFFC00000)) >> 22;
    *rxdndly7 = (localVal & ((uint32_t)0x001FF800)) >> 11;
    *rxdndly6 = (localVal & ((uint32_t)0x000003FF)) >> 0;
}

__INLINE uint16_t rpl_rx_dn_dly8_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFRXDN_DLY2_ADDR);
    return ((localVal & ((uint32_t)0xFFC00000)) >> 22);
}

__INLINE void rpl_rx_dn_dly8_setf(uint16_t rxdndly8)
{
    ASSERT_ERR((((uint32_t)rxdndly8 << 22) & ~((uint32_t)0xFFC00000)) == 0);
    REG_RPL_WR(RPL_RFRXDN_DLY2_ADDR, (REG_RPL_RD(RPL_RFRXDN_DLY2_ADDR) & ~((uint32_t)0xFFC00000)) | ((uint32_t)rxdndly8 << 22));
}

__INLINE uint16_t rpl_rx_dn_dly7_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFRXDN_DLY2_ADDR);
    return ((localVal & ((uint32_t)0x001FF800)) >> 11);
}

__INLINE void rpl_rx_dn_dly7_setf(uint16_t rxdndly7)
{
    ASSERT_ERR((((uint32_t)rxdndly7 << 11) & ~((uint32_t)0x001FF800)) == 0);
    REG_RPL_WR(RPL_RFRXDN_DLY2_ADDR, (REG_RPL_RD(RPL_RFRXDN_DLY2_ADDR) & ~((uint32_t)0x001FF800)) | ((uint32_t)rxdndly7 << 11));
}

__INLINE uint16_t rpl_rx_dn_dly6_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFRXDN_DLY2_ADDR);
    return ((localVal & ((uint32_t)0x000003FF)) >> 0);
}

__INLINE void rpl_rx_dn_dly6_setf(uint16_t rxdndly6)
{
    ASSERT_ERR((((uint32_t)rxdndly6 << 0) & ~((uint32_t)0x000003FF)) == 0);
    REG_RPL_WR(RPL_RFRXDN_DLY2_ADDR, (REG_RPL_RD(RPL_RFRXDN_DLY2_ADDR) & ~((uint32_t)0x000003FF)) | ((uint32_t)rxdndly6 << 0));
}

/**
 * @brief SEQTXUP_OUT0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  30:22           TX_UP_OUT2   0x7
 *  19:11           TX_UP_OUT1   0x3
 *  08:00           TX_UP_OUT0   0x1
 * </pre>
 */
#define RPL_SEQTXUP_OUT0_ADDR   0x0000000F
#define RPL_SEQTXUP_OUT0_OFFSET 0x0000000F
#define RPL_SEQTXUP_OUT0_INDEX  0x0000000F
#define RPL_SEQTXUP_OUT0_RESET  0x01C01801

__INLINE uint32_t rpl_seqtxup_out0_get(void)
{
    return REG_RPL_RD(RPL_SEQTXUP_OUT0_ADDR);
}

__INLINE void rpl_seqtxup_out0_set(uint32_t value)
{
    REG_RPL_WR(RPL_SEQTXUP_OUT0_ADDR, value);
}

// field definitions
#define RPL_TX_UP_OUT2_MASK   ((uint32_t)0x7FC00000)
#define RPL_TX_UP_OUT2_LSB    22
#define RPL_TX_UP_OUT2_WIDTH  ((uint32_t)0x00000009)
#define RPL_TX_UP_OUT1_MASK   ((uint32_t)0x000FF800)
#define RPL_TX_UP_OUT1_LSB    11
#define RPL_TX_UP_OUT1_WIDTH  ((uint32_t)0x00000009)
#define RPL_TX_UP_OUT0_MASK   ((uint32_t)0x000001FF)
#define RPL_TX_UP_OUT0_LSB    0
#define RPL_TX_UP_OUT0_WIDTH  ((uint32_t)0x00000009)

#define RPL_TX_UP_OUT2_RST    0x7
#define RPL_TX_UP_OUT1_RST    0x3
#define RPL_TX_UP_OUT0_RST    0x1

__INLINE void rpl_seqtxup_out0_pack(uint16_t txupout2, uint16_t txupout1, uint16_t txupout0)
{
    ASSERT_ERR((((uint32_t)txupout2 << 22) & ~((uint32_t)0x7FC00000)) == 0);
    ASSERT_ERR((((uint32_t)txupout1 << 11) & ~((uint32_t)0x000FF800)) == 0);
    ASSERT_ERR((((uint32_t)txupout0 << 0) & ~((uint32_t)0x000001FF)) == 0);
    REG_RPL_WR(RPL_SEQTXUP_OUT0_ADDR, ((uint32_t)txupout2 << 22) | ((uint32_t)txupout1 << 11) | ((uint32_t)txupout0 << 0));
}

__INLINE void rpl_seqtxup_out0_unpack(uint16_t *txupout2, uint16_t *txupout1, uint16_t *txupout0)
{
    uint32_t localVal = REG_RPL_RD(RPL_SEQTXUP_OUT0_ADDR);

    *txupout2 = (localVal & ((uint32_t)0x7FC00000)) >> 22;
    *txupout1 = (localVal & ((uint32_t)0x000FF800)) >> 11;
    *txupout0 = (localVal & ((uint32_t)0x000001FF)) >> 0;
}

__INLINE uint16_t rpl_tx_up_out2_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_SEQTXUP_OUT0_ADDR);
    return ((localVal & ((uint32_t)0x7FC00000)) >> 22);
}

__INLINE void rpl_tx_up_out2_setf(uint16_t txupout2)
{
    ASSERT_ERR((((uint32_t)txupout2 << 22) & ~((uint32_t)0x7FC00000)) == 0);
    REG_RPL_WR(RPL_SEQTXUP_OUT0_ADDR, (REG_RPL_RD(RPL_SEQTXUP_OUT0_ADDR) & ~((uint32_t)0x7FC00000)) | ((uint32_t)txupout2 << 22));
}

__INLINE uint16_t rpl_tx_up_out1_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_SEQTXUP_OUT0_ADDR);
    return ((localVal & ((uint32_t)0x000FF800)) >> 11);
}

__INLINE void rpl_tx_up_out1_setf(uint16_t txupout1)
{
    ASSERT_ERR((((uint32_t)txupout1 << 11) & ~((uint32_t)0x000FF800)) == 0);
    REG_RPL_WR(RPL_SEQTXUP_OUT0_ADDR, (REG_RPL_RD(RPL_SEQTXUP_OUT0_ADDR) & ~((uint32_t)0x000FF800)) | ((uint32_t)txupout1 << 11));
}

__INLINE uint16_t rpl_tx_up_out0_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_SEQTXUP_OUT0_ADDR);
    return ((localVal & ((uint32_t)0x000001FF)) >> 0);
}

__INLINE void rpl_tx_up_out0_setf(uint16_t txupout0)
{
    ASSERT_ERR((((uint32_t)txupout0 << 0) & ~((uint32_t)0x000001FF)) == 0);
    REG_RPL_WR(RPL_SEQTXUP_OUT0_ADDR, (REG_RPL_RD(RPL_SEQTXUP_OUT0_ADDR) & ~((uint32_t)0x000001FF)) | ((uint32_t)txupout0 << 0));
}

/**
 * @brief SEQTXUP_OUT1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  30:22           TX_UP_OUT5   0x1F
 *  19:11           TX_UP_OUT4   0x1F
 *  08:00           TX_UP_OUT3   0xF
 * </pre>
 */
#define RPL_SEQTXUP_OUT1_ADDR   0x00000010
#define RPL_SEQTXUP_OUT1_OFFSET 0x00000010
#define RPL_SEQTXUP_OUT1_INDEX  0x00000010
#define RPL_SEQTXUP_OUT1_RESET  0x07C0F80F

__INLINE uint32_t rpl_seqtxup_out1_get(void)
{
    return REG_RPL_RD(RPL_SEQTXUP_OUT1_ADDR);
}

__INLINE void rpl_seqtxup_out1_set(uint32_t value)
{
    REG_RPL_WR(RPL_SEQTXUP_OUT1_ADDR, value);
}

// field definitions
#define RPL_TX_UP_OUT5_MASK   ((uint32_t)0x7FC00000)
#define RPL_TX_UP_OUT5_LSB    22
#define RPL_TX_UP_OUT5_WIDTH  ((uint32_t)0x00000009)
#define RPL_TX_UP_OUT4_MASK   ((uint32_t)0x000FF800)
#define RPL_TX_UP_OUT4_LSB    11
#define RPL_TX_UP_OUT4_WIDTH  ((uint32_t)0x00000009)
#define RPL_TX_UP_OUT3_MASK   ((uint32_t)0x000001FF)
#define RPL_TX_UP_OUT3_LSB    0
#define RPL_TX_UP_OUT3_WIDTH  ((uint32_t)0x00000009)

#define RPL_TX_UP_OUT5_RST    0x1F
#define RPL_TX_UP_OUT4_RST    0x1F
#define RPL_TX_UP_OUT3_RST    0xF

__INLINE void rpl_seqtxup_out1_pack(uint16_t txupout5, uint16_t txupout4, uint16_t txupout3)
{
    ASSERT_ERR((((uint32_t)txupout5 << 22) & ~((uint32_t)0x7FC00000)) == 0);
    ASSERT_ERR((((uint32_t)txupout4 << 11) & ~((uint32_t)0x000FF800)) == 0);
    ASSERT_ERR((((uint32_t)txupout3 << 0) & ~((uint32_t)0x000001FF)) == 0);
    REG_RPL_WR(RPL_SEQTXUP_OUT1_ADDR, ((uint32_t)txupout5 << 22) | ((uint32_t)txupout4 << 11) | ((uint32_t)txupout3 << 0));
}

__INLINE void rpl_seqtxup_out1_unpack(uint16_t *txupout5, uint16_t *txupout4, uint16_t *txupout3)
{
    uint32_t localVal = REG_RPL_RD(RPL_SEQTXUP_OUT1_ADDR);

    *txupout5 = (localVal & ((uint32_t)0x7FC00000)) >> 22;
    *txupout4 = (localVal & ((uint32_t)0x000FF800)) >> 11;
    *txupout3 = (localVal & ((uint32_t)0x000001FF)) >> 0;
}

__INLINE uint16_t rpl_tx_up_out5_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_SEQTXUP_OUT1_ADDR);
    return ((localVal & ((uint32_t)0x7FC00000)) >> 22);
}

__INLINE void rpl_tx_up_out5_setf(uint16_t txupout5)
{
    ASSERT_ERR((((uint32_t)txupout5 << 22) & ~((uint32_t)0x7FC00000)) == 0);
    REG_RPL_WR(RPL_SEQTXUP_OUT1_ADDR, (REG_RPL_RD(RPL_SEQTXUP_OUT1_ADDR) & ~((uint32_t)0x7FC00000)) | ((uint32_t)txupout5 << 22));
}

__INLINE uint16_t rpl_tx_up_out4_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_SEQTXUP_OUT1_ADDR);
    return ((localVal & ((uint32_t)0x000FF800)) >> 11);
}

__INLINE void rpl_tx_up_out4_setf(uint16_t txupout4)
{
    ASSERT_ERR((((uint32_t)txupout4 << 11) & ~((uint32_t)0x000FF800)) == 0);
    REG_RPL_WR(RPL_SEQTXUP_OUT1_ADDR, (REG_RPL_RD(RPL_SEQTXUP_OUT1_ADDR) & ~((uint32_t)0x000FF800)) | ((uint32_t)txupout4 << 11));
}

__INLINE uint16_t rpl_tx_up_out3_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_SEQTXUP_OUT1_ADDR);
    return ((localVal & ((uint32_t)0x000001FF)) >> 0);
}

__INLINE void rpl_tx_up_out3_setf(uint16_t txupout3)
{
    ASSERT_ERR((((uint32_t)txupout3 << 0) & ~((uint32_t)0x000001FF)) == 0);
    REG_RPL_WR(RPL_SEQTXUP_OUT1_ADDR, (REG_RPL_RD(RPL_SEQTXUP_OUT1_ADDR) & ~((uint32_t)0x000001FF)) | ((uint32_t)txupout3 << 0));
}

/**
 * @brief SEQTXUP_OUT2 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  30:22           TX_UP_OUT8   0x1F
 *  19:11           TX_UP_OUT7   0x1F
 *  08:00           TX_UP_OUT6   0x1F
 * </pre>
 */
#define RPL_SEQTXUP_OUT2_ADDR   0x00000011
#define RPL_SEQTXUP_OUT2_OFFSET 0x00000011
#define RPL_SEQTXUP_OUT2_INDEX  0x00000011
#define RPL_SEQTXUP_OUT2_RESET  0x07C0F81F

__INLINE uint32_t rpl_seqtxup_out2_get(void)
{
    return REG_RPL_RD(RPL_SEQTXUP_OUT2_ADDR);
}

__INLINE void rpl_seqtxup_out2_set(uint32_t value)
{
    REG_RPL_WR(RPL_SEQTXUP_OUT2_ADDR, value);
}

// field definitions
#define RPL_TX_UP_OUT8_MASK   ((uint32_t)0x7FC00000)
#define RPL_TX_UP_OUT8_LSB    22
#define RPL_TX_UP_OUT8_WIDTH  ((uint32_t)0x00000009)
#define RPL_TX_UP_OUT7_MASK   ((uint32_t)0x000FF800)
#define RPL_TX_UP_OUT7_LSB    11
#define RPL_TX_UP_OUT7_WIDTH  ((uint32_t)0x00000009)
#define RPL_TX_UP_OUT6_MASK   ((uint32_t)0x000001FF)
#define RPL_TX_UP_OUT6_LSB    0
#define RPL_TX_UP_OUT6_WIDTH  ((uint32_t)0x00000009)

#define RPL_TX_UP_OUT8_RST    0x1F
#define RPL_TX_UP_OUT7_RST    0x1F
#define RPL_TX_UP_OUT6_RST    0x1F

__INLINE void rpl_seqtxup_out2_pack(uint16_t txupout8, uint16_t txupout7, uint16_t txupout6)
{
    ASSERT_ERR((((uint32_t)txupout8 << 22) & ~((uint32_t)0x7FC00000)) == 0);
    ASSERT_ERR((((uint32_t)txupout7 << 11) & ~((uint32_t)0x000FF800)) == 0);
    ASSERT_ERR((((uint32_t)txupout6 << 0) & ~((uint32_t)0x000001FF)) == 0);
    REG_RPL_WR(RPL_SEQTXUP_OUT2_ADDR, ((uint32_t)txupout8 << 22) | ((uint32_t)txupout7 << 11) | ((uint32_t)txupout6 << 0));
}

__INLINE void rpl_seqtxup_out2_unpack(uint16_t *txupout8, uint16_t *txupout7, uint16_t *txupout6)
{
    uint32_t localVal = REG_RPL_RD(RPL_SEQTXUP_OUT2_ADDR);

    *txupout8 = (localVal & ((uint32_t)0x7FC00000)) >> 22;
    *txupout7 = (localVal & ((uint32_t)0x000FF800)) >> 11;
    *txupout6 = (localVal & ((uint32_t)0x000001FF)) >> 0;
}

__INLINE uint16_t rpl_tx_up_out8_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_SEQTXUP_OUT2_ADDR);
    return ((localVal & ((uint32_t)0x7FC00000)) >> 22);
}

__INLINE void rpl_tx_up_out8_setf(uint16_t txupout8)
{
    ASSERT_ERR((((uint32_t)txupout8 << 22) & ~((uint32_t)0x7FC00000)) == 0);
    REG_RPL_WR(RPL_SEQTXUP_OUT2_ADDR, (REG_RPL_RD(RPL_SEQTXUP_OUT2_ADDR) & ~((uint32_t)0x7FC00000)) | ((uint32_t)txupout8 << 22));
}

__INLINE uint16_t rpl_tx_up_out7_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_SEQTXUP_OUT2_ADDR);
    return ((localVal & ((uint32_t)0x000FF800)) >> 11);
}

__INLINE void rpl_tx_up_out7_setf(uint16_t txupout7)
{
    ASSERT_ERR((((uint32_t)txupout7 << 11) & ~((uint32_t)0x000FF800)) == 0);
    REG_RPL_WR(RPL_SEQTXUP_OUT2_ADDR, (REG_RPL_RD(RPL_SEQTXUP_OUT2_ADDR) & ~((uint32_t)0x000FF800)) | ((uint32_t)txupout7 << 11));
}

__INLINE uint16_t rpl_tx_up_out6_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_SEQTXUP_OUT2_ADDR);
    return ((localVal & ((uint32_t)0x000001FF)) >> 0);
}

__INLINE void rpl_tx_up_out6_setf(uint16_t txupout6)
{
    ASSERT_ERR((((uint32_t)txupout6 << 0) & ~((uint32_t)0x000001FF)) == 0);
    REG_RPL_WR(RPL_SEQTXUP_OUT2_ADDR, (REG_RPL_RD(RPL_SEQTXUP_OUT2_ADDR) & ~((uint32_t)0x000001FF)) | ((uint32_t)txupout6 << 0));
}

/**
 * @brief SEQTXDN_OUT0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  30:22           TX_DN_OUT2   0x3
 *  19:11           TX_DN_OUT1   0x7
 *  08:00           TX_DN_OUT0   0xF
 * </pre>
 */
#define RPL_SEQTXDN_OUT0_ADDR   0x00000012
#define RPL_SEQTXDN_OUT0_OFFSET 0x00000012
#define RPL_SEQTXDN_OUT0_INDEX  0x00000012
#define RPL_SEQTXDN_OUT0_RESET  0x00C0380F

__INLINE uint32_t rpl_seqtxdn_out0_get(void)
{
    return REG_RPL_RD(RPL_SEQTXDN_OUT0_ADDR);
}

__INLINE void rpl_seqtxdn_out0_set(uint32_t value)
{
    REG_RPL_WR(RPL_SEQTXDN_OUT0_ADDR, value);
}

// field definitions
#define RPL_TX_DN_OUT2_MASK   ((uint32_t)0x7FC00000)
#define RPL_TX_DN_OUT2_LSB    22
#define RPL_TX_DN_OUT2_WIDTH  ((uint32_t)0x00000009)
#define RPL_TX_DN_OUT1_MASK   ((uint32_t)0x000FF800)
#define RPL_TX_DN_OUT1_LSB    11
#define RPL_TX_DN_OUT1_WIDTH  ((uint32_t)0x00000009)
#define RPL_TX_DN_OUT0_MASK   ((uint32_t)0x000001FF)
#define RPL_TX_DN_OUT0_LSB    0
#define RPL_TX_DN_OUT0_WIDTH  ((uint32_t)0x00000009)

#define RPL_TX_DN_OUT2_RST    0x3
#define RPL_TX_DN_OUT1_RST    0x7
#define RPL_TX_DN_OUT0_RST    0xF

__INLINE void rpl_seqtxdn_out0_pack(uint16_t txdnout2, uint16_t txdnout1, uint16_t txdnout0)
{
    ASSERT_ERR((((uint32_t)txdnout2 << 22) & ~((uint32_t)0x7FC00000)) == 0);
    ASSERT_ERR((((uint32_t)txdnout1 << 11) & ~((uint32_t)0x000FF800)) == 0);
    ASSERT_ERR((((uint32_t)txdnout0 << 0) & ~((uint32_t)0x000001FF)) == 0);
    REG_RPL_WR(RPL_SEQTXDN_OUT0_ADDR, ((uint32_t)txdnout2 << 22) | ((uint32_t)txdnout1 << 11) | ((uint32_t)txdnout0 << 0));
}

__INLINE void rpl_seqtxdn_out0_unpack(uint16_t *txdnout2, uint16_t *txdnout1, uint16_t *txdnout0)
{
    uint32_t localVal = REG_RPL_RD(RPL_SEQTXDN_OUT0_ADDR);

    *txdnout2 = (localVal & ((uint32_t)0x7FC00000)) >> 22;
    *txdnout1 = (localVal & ((uint32_t)0x000FF800)) >> 11;
    *txdnout0 = (localVal & ((uint32_t)0x000001FF)) >> 0;
}

__INLINE uint16_t rpl_tx_dn_out2_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_SEQTXDN_OUT0_ADDR);
    return ((localVal & ((uint32_t)0x7FC00000)) >> 22);
}

__INLINE void rpl_tx_dn_out2_setf(uint16_t txdnout2)
{
    ASSERT_ERR((((uint32_t)txdnout2 << 22) & ~((uint32_t)0x7FC00000)) == 0);
    REG_RPL_WR(RPL_SEQTXDN_OUT0_ADDR, (REG_RPL_RD(RPL_SEQTXDN_OUT0_ADDR) & ~((uint32_t)0x7FC00000)) | ((uint32_t)txdnout2 << 22));
}

__INLINE uint16_t rpl_tx_dn_out1_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_SEQTXDN_OUT0_ADDR);
    return ((localVal & ((uint32_t)0x000FF800)) >> 11);
}

__INLINE void rpl_tx_dn_out1_setf(uint16_t txdnout1)
{
    ASSERT_ERR((((uint32_t)txdnout1 << 11) & ~((uint32_t)0x000FF800)) == 0);
    REG_RPL_WR(RPL_SEQTXDN_OUT0_ADDR, (REG_RPL_RD(RPL_SEQTXDN_OUT0_ADDR) & ~((uint32_t)0x000FF800)) | ((uint32_t)txdnout1 << 11));
}

__INLINE uint16_t rpl_tx_dn_out0_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_SEQTXDN_OUT0_ADDR);
    return ((localVal & ((uint32_t)0x000001FF)) >> 0);
}

__INLINE void rpl_tx_dn_out0_setf(uint16_t txdnout0)
{
    ASSERT_ERR((((uint32_t)txdnout0 << 0) & ~((uint32_t)0x000001FF)) == 0);
    REG_RPL_WR(RPL_SEQTXDN_OUT0_ADDR, (REG_RPL_RD(RPL_SEQTXDN_OUT0_ADDR) & ~((uint32_t)0x000001FF)) | ((uint32_t)txdnout0 << 0));
}

/**
 * @brief SEQTXDN_OUT1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  30:22           TX_DN_OUT5   0x0
 *  19:11           TX_DN_OUT4   0x0
 *  08:00           TX_DN_OUT3   0x1
 * </pre>
 */
#define RPL_SEQTXDN_OUT1_ADDR   0x00000013
#define RPL_SEQTXDN_OUT1_OFFSET 0x00000013
#define RPL_SEQTXDN_OUT1_INDEX  0x00000013
#define RPL_SEQTXDN_OUT1_RESET  0x00000001

__INLINE uint32_t rpl_seqtxdn_out1_get(void)
{
    return REG_RPL_RD(RPL_SEQTXDN_OUT1_ADDR);
}

__INLINE void rpl_seqtxdn_out1_set(uint32_t value)
{
    REG_RPL_WR(RPL_SEQTXDN_OUT1_ADDR, value);
}

// field definitions
#define RPL_TX_DN_OUT5_MASK   ((uint32_t)0x7FC00000)
#define RPL_TX_DN_OUT5_LSB    22
#define RPL_TX_DN_OUT5_WIDTH  ((uint32_t)0x00000009)
#define RPL_TX_DN_OUT4_MASK   ((uint32_t)0x000FF800)
#define RPL_TX_DN_OUT4_LSB    11
#define RPL_TX_DN_OUT4_WIDTH  ((uint32_t)0x00000009)
#define RPL_TX_DN_OUT3_MASK   ((uint32_t)0x000001FF)
#define RPL_TX_DN_OUT3_LSB    0
#define RPL_TX_DN_OUT3_WIDTH  ((uint32_t)0x00000009)

#define RPL_TX_DN_OUT5_RST    0x0
#define RPL_TX_DN_OUT4_RST    0x0
#define RPL_TX_DN_OUT3_RST    0x1

__INLINE void rpl_seqtxdn_out1_pack(uint16_t txdnout5, uint16_t txdnout4, uint16_t txdnout3)
{
    ASSERT_ERR((((uint32_t)txdnout5 << 22) & ~((uint32_t)0x7FC00000)) == 0);
    ASSERT_ERR((((uint32_t)txdnout4 << 11) & ~((uint32_t)0x000FF800)) == 0);
    ASSERT_ERR((((uint32_t)txdnout3 << 0) & ~((uint32_t)0x000001FF)) == 0);
    REG_RPL_WR(RPL_SEQTXDN_OUT1_ADDR, ((uint32_t)txdnout5 << 22) | ((uint32_t)txdnout4 << 11) | ((uint32_t)txdnout3 << 0));
}

__INLINE void rpl_seqtxdn_out1_unpack(uint16_t *txdnout5, uint16_t *txdnout4, uint16_t *txdnout3)
{
    uint32_t localVal = REG_RPL_RD(RPL_SEQTXDN_OUT1_ADDR);

    *txdnout5 = (localVal & ((uint32_t)0x7FC00000)) >> 22;
    *txdnout4 = (localVal & ((uint32_t)0x000FF800)) >> 11;
    *txdnout3 = (localVal & ((uint32_t)0x000001FF)) >> 0;
}

__INLINE uint16_t rpl_tx_dn_out5_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_SEQTXDN_OUT1_ADDR);
    return ((localVal & ((uint32_t)0x7FC00000)) >> 22);
}

__INLINE void rpl_tx_dn_out5_setf(uint16_t txdnout5)
{
    ASSERT_ERR((((uint32_t)txdnout5 << 22) & ~((uint32_t)0x7FC00000)) == 0);
    REG_RPL_WR(RPL_SEQTXDN_OUT1_ADDR, (REG_RPL_RD(RPL_SEQTXDN_OUT1_ADDR) & ~((uint32_t)0x7FC00000)) | ((uint32_t)txdnout5 << 22));
}

__INLINE uint16_t rpl_tx_dn_out4_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_SEQTXDN_OUT1_ADDR);
    return ((localVal & ((uint32_t)0x000FF800)) >> 11);
}

__INLINE void rpl_tx_dn_out4_setf(uint16_t txdnout4)
{
    ASSERT_ERR((((uint32_t)txdnout4 << 11) & ~((uint32_t)0x000FF800)) == 0);
    REG_RPL_WR(RPL_SEQTXDN_OUT1_ADDR, (REG_RPL_RD(RPL_SEQTXDN_OUT1_ADDR) & ~((uint32_t)0x000FF800)) | ((uint32_t)txdnout4 << 11));
}

__INLINE uint16_t rpl_tx_dn_out3_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_SEQTXDN_OUT1_ADDR);
    return ((localVal & ((uint32_t)0x000001FF)) >> 0);
}

__INLINE void rpl_tx_dn_out3_setf(uint16_t txdnout3)
{
    ASSERT_ERR((((uint32_t)txdnout3 << 0) & ~((uint32_t)0x000001FF)) == 0);
    REG_RPL_WR(RPL_SEQTXDN_OUT1_ADDR, (REG_RPL_RD(RPL_SEQTXDN_OUT1_ADDR) & ~((uint32_t)0x000001FF)) | ((uint32_t)txdnout3 << 0));
}

/**
 * @brief SEQTXDN_OUT2 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  30:22           TX_DN_OUT8   0x0
 *  19:11           TX_DN_OUT7   0x0
 *  08:00           TX_DN_OUT6   0x0
 * </pre>
 */
#define RPL_SEQTXDN_OUT2_ADDR   0x00000014
#define RPL_SEQTXDN_OUT2_OFFSET 0x00000014
#define RPL_SEQTXDN_OUT2_INDEX  0x00000014
#define RPL_SEQTXDN_OUT2_RESET  0x00000000

__INLINE uint32_t rpl_seqtxdn_out2_get(void)
{
    return REG_RPL_RD(RPL_SEQTXDN_OUT2_ADDR);
}

__INLINE void rpl_seqtxdn_out2_set(uint32_t value)
{
    REG_RPL_WR(RPL_SEQTXDN_OUT2_ADDR, value);
}

// field definitions
#define RPL_TX_DN_OUT8_MASK   ((uint32_t)0x7FC00000)
#define RPL_TX_DN_OUT8_LSB    22
#define RPL_TX_DN_OUT8_WIDTH  ((uint32_t)0x00000009)
#define RPL_TX_DN_OUT7_MASK   ((uint32_t)0x000FF800)
#define RPL_TX_DN_OUT7_LSB    11
#define RPL_TX_DN_OUT7_WIDTH  ((uint32_t)0x00000009)
#define RPL_TX_DN_OUT6_MASK   ((uint32_t)0x000001FF)
#define RPL_TX_DN_OUT6_LSB    0
#define RPL_TX_DN_OUT6_WIDTH  ((uint32_t)0x00000009)

#define RPL_TX_DN_OUT8_RST    0x0
#define RPL_TX_DN_OUT7_RST    0x0
#define RPL_TX_DN_OUT6_RST    0x0

__INLINE void rpl_seqtxdn_out2_pack(uint16_t txdnout8, uint16_t txdnout7, uint16_t txdnout6)
{
    ASSERT_ERR((((uint32_t)txdnout8 << 22) & ~((uint32_t)0x7FC00000)) == 0);
    ASSERT_ERR((((uint32_t)txdnout7 << 11) & ~((uint32_t)0x000FF800)) == 0);
    ASSERT_ERR((((uint32_t)txdnout6 << 0) & ~((uint32_t)0x000001FF)) == 0);
    REG_RPL_WR(RPL_SEQTXDN_OUT2_ADDR, ((uint32_t)txdnout8 << 22) | ((uint32_t)txdnout7 << 11) | ((uint32_t)txdnout6 << 0));
}

__INLINE void rpl_seqtxdn_out2_unpack(uint16_t *txdnout8, uint16_t *txdnout7, uint16_t *txdnout6)
{
    uint32_t localVal = REG_RPL_RD(RPL_SEQTXDN_OUT2_ADDR);

    *txdnout8 = (localVal & ((uint32_t)0x7FC00000)) >> 22;
    *txdnout7 = (localVal & ((uint32_t)0x000FF800)) >> 11;
    *txdnout6 = (localVal & ((uint32_t)0x000001FF)) >> 0;
}

__INLINE uint16_t rpl_tx_dn_out8_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_SEQTXDN_OUT2_ADDR);
    return ((localVal & ((uint32_t)0x7FC00000)) >> 22);
}

__INLINE void rpl_tx_dn_out8_setf(uint16_t txdnout8)
{
    ASSERT_ERR((((uint32_t)txdnout8 << 22) & ~((uint32_t)0x7FC00000)) == 0);
    REG_RPL_WR(RPL_SEQTXDN_OUT2_ADDR, (REG_RPL_RD(RPL_SEQTXDN_OUT2_ADDR) & ~((uint32_t)0x7FC00000)) | ((uint32_t)txdnout8 << 22));
}

__INLINE uint16_t rpl_tx_dn_out7_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_SEQTXDN_OUT2_ADDR);
    return ((localVal & ((uint32_t)0x000FF800)) >> 11);
}

__INLINE void rpl_tx_dn_out7_setf(uint16_t txdnout7)
{
    ASSERT_ERR((((uint32_t)txdnout7 << 11) & ~((uint32_t)0x000FF800)) == 0);
    REG_RPL_WR(RPL_SEQTXDN_OUT2_ADDR, (REG_RPL_RD(RPL_SEQTXDN_OUT2_ADDR) & ~((uint32_t)0x000FF800)) | ((uint32_t)txdnout7 << 11));
}

__INLINE uint16_t rpl_tx_dn_out6_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_SEQTXDN_OUT2_ADDR);
    return ((localVal & ((uint32_t)0x000001FF)) >> 0);
}

__INLINE void rpl_tx_dn_out6_setf(uint16_t txdnout6)
{
    ASSERT_ERR((((uint32_t)txdnout6 << 0) & ~((uint32_t)0x000001FF)) == 0);
    REG_RPL_WR(RPL_SEQTXDN_OUT2_ADDR, (REG_RPL_RD(RPL_SEQTXDN_OUT2_ADDR) & ~((uint32_t)0x000001FF)) | ((uint32_t)txdnout6 << 0));
}

/**
 * @brief SEQRXUP_OUT0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  30:22           RX_UP_OUT2   0x7
 *  19:11           RX_UP_OUT1   0x3
 *  08:00           RX_UP_OUT0   0x1
 * </pre>
 */
#define RPL_SEQRXUP_OUT0_ADDR   0x00000015
#define RPL_SEQRXUP_OUT0_OFFSET 0x00000015
#define RPL_SEQRXUP_OUT0_INDEX  0x00000015
#define RPL_SEQRXUP_OUT0_RESET  0x01C01801

__INLINE uint32_t rpl_seqrxup_out0_get(void)
{
    return REG_RPL_RD(RPL_SEQRXUP_OUT0_ADDR);
}

__INLINE void rpl_seqrxup_out0_set(uint32_t value)
{
    REG_RPL_WR(RPL_SEQRXUP_OUT0_ADDR, value);
}

// field definitions
#define RPL_RX_UP_OUT2_MASK   ((uint32_t)0x7FC00000)
#define RPL_RX_UP_OUT2_LSB    22
#define RPL_RX_UP_OUT2_WIDTH  ((uint32_t)0x00000009)
#define RPL_RX_UP_OUT1_MASK   ((uint32_t)0x000FF800)
#define RPL_RX_UP_OUT1_LSB    11
#define RPL_RX_UP_OUT1_WIDTH  ((uint32_t)0x00000009)
#define RPL_RX_UP_OUT0_MASK   ((uint32_t)0x000001FF)
#define RPL_RX_UP_OUT0_LSB    0
#define RPL_RX_UP_OUT0_WIDTH  ((uint32_t)0x00000009)

#define RPL_RX_UP_OUT2_RST    0x7
#define RPL_RX_UP_OUT1_RST    0x3
#define RPL_RX_UP_OUT0_RST    0x1

__INLINE void rpl_seqrxup_out0_pack(uint16_t rxupout2, uint16_t rxupout1, uint16_t rxupout0)
{
    ASSERT_ERR((((uint32_t)rxupout2 << 22) & ~((uint32_t)0x7FC00000)) == 0);
    ASSERT_ERR((((uint32_t)rxupout1 << 11) & ~((uint32_t)0x000FF800)) == 0);
    ASSERT_ERR((((uint32_t)rxupout0 << 0) & ~((uint32_t)0x000001FF)) == 0);
    REG_RPL_WR(RPL_SEQRXUP_OUT0_ADDR, ((uint32_t)rxupout2 << 22) | ((uint32_t)rxupout1 << 11) | ((uint32_t)rxupout0 << 0));
}

__INLINE void rpl_seqrxup_out0_unpack(uint16_t *rxupout2, uint16_t *rxupout1, uint16_t *rxupout0)
{
    uint32_t localVal = REG_RPL_RD(RPL_SEQRXUP_OUT0_ADDR);

    *rxupout2 = (localVal & ((uint32_t)0x7FC00000)) >> 22;
    *rxupout1 = (localVal & ((uint32_t)0x000FF800)) >> 11;
    *rxupout0 = (localVal & ((uint32_t)0x000001FF)) >> 0;
}

__INLINE uint16_t rpl_rx_up_out2_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_SEQRXUP_OUT0_ADDR);
    return ((localVal & ((uint32_t)0x7FC00000)) >> 22);
}

__INLINE void rpl_rx_up_out2_setf(uint16_t rxupout2)
{
    ASSERT_ERR((((uint32_t)rxupout2 << 22) & ~((uint32_t)0x7FC00000)) == 0);
    REG_RPL_WR(RPL_SEQRXUP_OUT0_ADDR, (REG_RPL_RD(RPL_SEQRXUP_OUT0_ADDR) & ~((uint32_t)0x7FC00000)) | ((uint32_t)rxupout2 << 22));
}

__INLINE uint16_t rpl_rx_up_out1_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_SEQRXUP_OUT0_ADDR);
    return ((localVal & ((uint32_t)0x000FF800)) >> 11);
}

__INLINE void rpl_rx_up_out1_setf(uint16_t rxupout1)
{
    ASSERT_ERR((((uint32_t)rxupout1 << 11) & ~((uint32_t)0x000FF800)) == 0);
    REG_RPL_WR(RPL_SEQRXUP_OUT0_ADDR, (REG_RPL_RD(RPL_SEQRXUP_OUT0_ADDR) & ~((uint32_t)0x000FF800)) | ((uint32_t)rxupout1 << 11));
}

__INLINE uint16_t rpl_rx_up_out0_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_SEQRXUP_OUT0_ADDR);
    return ((localVal & ((uint32_t)0x000001FF)) >> 0);
}

__INLINE void rpl_rx_up_out0_setf(uint16_t rxupout0)
{
    ASSERT_ERR((((uint32_t)rxupout0 << 0) & ~((uint32_t)0x000001FF)) == 0);
    REG_RPL_WR(RPL_SEQRXUP_OUT0_ADDR, (REG_RPL_RD(RPL_SEQRXUP_OUT0_ADDR) & ~((uint32_t)0x000001FF)) | ((uint32_t)rxupout0 << 0));
}

/**
 * @brief SEQRXUP_OUT1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  30:22           RX_UP_OUT5   0x1F
 *  19:11           RX_UP_OUT4   0x1F
 *  08:00           RX_UP_OUT3   0xF
 * </pre>
 */
#define RPL_SEQRXUP_OUT1_ADDR   0x00000016
#define RPL_SEQRXUP_OUT1_OFFSET 0x00000016
#define RPL_SEQRXUP_OUT1_INDEX  0x00000016
#define RPL_SEQRXUP_OUT1_RESET  0x07C0F80F

__INLINE uint32_t rpl_seqrxup_out1_get(void)
{
    return REG_RPL_RD(RPL_SEQRXUP_OUT1_ADDR);
}

__INLINE void rpl_seqrxup_out1_set(uint32_t value)
{
    REG_RPL_WR(RPL_SEQRXUP_OUT1_ADDR, value);
}

// field definitions
#define RPL_RX_UP_OUT5_MASK   ((uint32_t)0x7FC00000)
#define RPL_RX_UP_OUT5_LSB    22
#define RPL_RX_UP_OUT5_WIDTH  ((uint32_t)0x00000009)
#define RPL_RX_UP_OUT4_MASK   ((uint32_t)0x000FF800)
#define RPL_RX_UP_OUT4_LSB    11
#define RPL_RX_UP_OUT4_WIDTH  ((uint32_t)0x00000009)
#define RPL_RX_UP_OUT3_MASK   ((uint32_t)0x000001FF)
#define RPL_RX_UP_OUT3_LSB    0
#define RPL_RX_UP_OUT3_WIDTH  ((uint32_t)0x00000009)

#define RPL_RX_UP_OUT5_RST    0x1F
#define RPL_RX_UP_OUT4_RST    0x1F
#define RPL_RX_UP_OUT3_RST    0xF

__INLINE void rpl_seqrxup_out1_pack(uint16_t rxupout5, uint16_t rxupout4, uint16_t rxupout3)
{
    ASSERT_ERR((((uint32_t)rxupout5 << 22) & ~((uint32_t)0x7FC00000)) == 0);
    ASSERT_ERR((((uint32_t)rxupout4 << 11) & ~((uint32_t)0x000FF800)) == 0);
    ASSERT_ERR((((uint32_t)rxupout3 << 0) & ~((uint32_t)0x000001FF)) == 0);
    REG_RPL_WR(RPL_SEQRXUP_OUT1_ADDR, ((uint32_t)rxupout5 << 22) | ((uint32_t)rxupout4 << 11) | ((uint32_t)rxupout3 << 0));
}

__INLINE void rpl_seqrxup_out1_unpack(uint16_t *rxupout5, uint16_t *rxupout4, uint16_t *rxupout3)
{
    uint32_t localVal = REG_RPL_RD(RPL_SEQRXUP_OUT1_ADDR);

    *rxupout5 = (localVal & ((uint32_t)0x7FC00000)) >> 22;
    *rxupout4 = (localVal & ((uint32_t)0x000FF800)) >> 11;
    *rxupout3 = (localVal & ((uint32_t)0x000001FF)) >> 0;
}

__INLINE uint16_t rpl_rx_up_out5_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_SEQRXUP_OUT1_ADDR);
    return ((localVal & ((uint32_t)0x7FC00000)) >> 22);
}

__INLINE void rpl_rx_up_out5_setf(uint16_t rxupout5)
{
    ASSERT_ERR((((uint32_t)rxupout5 << 22) & ~((uint32_t)0x7FC00000)) == 0);
    REG_RPL_WR(RPL_SEQRXUP_OUT1_ADDR, (REG_RPL_RD(RPL_SEQRXUP_OUT1_ADDR) & ~((uint32_t)0x7FC00000)) | ((uint32_t)rxupout5 << 22));
}

__INLINE uint16_t rpl_rx_up_out4_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_SEQRXUP_OUT1_ADDR);
    return ((localVal & ((uint32_t)0x000FF800)) >> 11);
}

__INLINE void rpl_rx_up_out4_setf(uint16_t rxupout4)
{
    ASSERT_ERR((((uint32_t)rxupout4 << 11) & ~((uint32_t)0x000FF800)) == 0);
    REG_RPL_WR(RPL_SEQRXUP_OUT1_ADDR, (REG_RPL_RD(RPL_SEQRXUP_OUT1_ADDR) & ~((uint32_t)0x000FF800)) | ((uint32_t)rxupout4 << 11));
}

__INLINE uint16_t rpl_rx_up_out3_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_SEQRXUP_OUT1_ADDR);
    return ((localVal & ((uint32_t)0x000001FF)) >> 0);
}

__INLINE void rpl_rx_up_out3_setf(uint16_t rxupout3)
{
    ASSERT_ERR((((uint32_t)rxupout3 << 0) & ~((uint32_t)0x000001FF)) == 0);
    REG_RPL_WR(RPL_SEQRXUP_OUT1_ADDR, (REG_RPL_RD(RPL_SEQRXUP_OUT1_ADDR) & ~((uint32_t)0x000001FF)) | ((uint32_t)rxupout3 << 0));
}

/**
 * @brief SEQRXUP_OUT2 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  30:22           RX_UP_OUT8   0x1F
 *  19:11           RX_UP_OUT7   0x1F
 *  08:00           RX_UP_OUT6   0x1F
 * </pre>
 */
#define RPL_SEQRXUP_OUT2_ADDR   0x00000017
#define RPL_SEQRXUP_OUT2_OFFSET 0x00000017
#define RPL_SEQRXUP_OUT2_INDEX  0x00000017
#define RPL_SEQRXUP_OUT2_RESET  0x07C0F81F

__INLINE uint32_t rpl_seqrxup_out2_get(void)
{
    return REG_RPL_RD(RPL_SEQRXUP_OUT2_ADDR);
}

__INLINE void rpl_seqrxup_out2_set(uint32_t value)
{
    REG_RPL_WR(RPL_SEQRXUP_OUT2_ADDR, value);
}

// field definitions
#define RPL_RX_UP_OUT8_MASK   ((uint32_t)0x7FC00000)
#define RPL_RX_UP_OUT8_LSB    22
#define RPL_RX_UP_OUT8_WIDTH  ((uint32_t)0x00000009)
#define RPL_RX_UP_OUT7_MASK   ((uint32_t)0x000FF800)
#define RPL_RX_UP_OUT7_LSB    11
#define RPL_RX_UP_OUT7_WIDTH  ((uint32_t)0x00000009)
#define RPL_RX_UP_OUT6_MASK   ((uint32_t)0x000001FF)
#define RPL_RX_UP_OUT6_LSB    0
#define RPL_RX_UP_OUT6_WIDTH  ((uint32_t)0x00000009)

#define RPL_RX_UP_OUT8_RST    0x1F
#define RPL_RX_UP_OUT7_RST    0x1F
#define RPL_RX_UP_OUT6_RST    0x1F

__INLINE void rpl_seqrxup_out2_pack(uint16_t rxupout8, uint16_t rxupout7, uint16_t rxupout6)
{
    ASSERT_ERR((((uint32_t)rxupout8 << 22) & ~((uint32_t)0x7FC00000)) == 0);
    ASSERT_ERR((((uint32_t)rxupout7 << 11) & ~((uint32_t)0x000FF800)) == 0);
    ASSERT_ERR((((uint32_t)rxupout6 << 0) & ~((uint32_t)0x000001FF)) == 0);
    REG_RPL_WR(RPL_SEQRXUP_OUT2_ADDR, ((uint32_t)rxupout8 << 22) | ((uint32_t)rxupout7 << 11) | ((uint32_t)rxupout6 << 0));
}

__INLINE void rpl_seqrxup_out2_unpack(uint16_t *rxupout8, uint16_t *rxupout7, uint16_t *rxupout6)
{
    uint32_t localVal = REG_RPL_RD(RPL_SEQRXUP_OUT2_ADDR);

    *rxupout8 = (localVal & ((uint32_t)0x7FC00000)) >> 22;
    *rxupout7 = (localVal & ((uint32_t)0x000FF800)) >> 11;
    *rxupout6 = (localVal & ((uint32_t)0x000001FF)) >> 0;
}

__INLINE uint16_t rpl_rx_up_out8_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_SEQRXUP_OUT2_ADDR);
    return ((localVal & ((uint32_t)0x7FC00000)) >> 22);
}

__INLINE void rpl_rx_up_out8_setf(uint16_t rxupout8)
{
    ASSERT_ERR((((uint32_t)rxupout8 << 22) & ~((uint32_t)0x7FC00000)) == 0);
    REG_RPL_WR(RPL_SEQRXUP_OUT2_ADDR, (REG_RPL_RD(RPL_SEQRXUP_OUT2_ADDR) & ~((uint32_t)0x7FC00000)) | ((uint32_t)rxupout8 << 22));
}

__INLINE uint16_t rpl_rx_up_out7_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_SEQRXUP_OUT2_ADDR);
    return ((localVal & ((uint32_t)0x000FF800)) >> 11);
}

__INLINE void rpl_rx_up_out7_setf(uint16_t rxupout7)
{
    ASSERT_ERR((((uint32_t)rxupout7 << 11) & ~((uint32_t)0x000FF800)) == 0);
    REG_RPL_WR(RPL_SEQRXUP_OUT2_ADDR, (REG_RPL_RD(RPL_SEQRXUP_OUT2_ADDR) & ~((uint32_t)0x000FF800)) | ((uint32_t)rxupout7 << 11));
}

__INLINE uint16_t rpl_rx_up_out6_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_SEQRXUP_OUT2_ADDR);
    return ((localVal & ((uint32_t)0x000001FF)) >> 0);
}

__INLINE void rpl_rx_up_out6_setf(uint16_t rxupout6)
{
    ASSERT_ERR((((uint32_t)rxupout6 << 0) & ~((uint32_t)0x000001FF)) == 0);
    REG_RPL_WR(RPL_SEQRXUP_OUT2_ADDR, (REG_RPL_RD(RPL_SEQRXUP_OUT2_ADDR) & ~((uint32_t)0x000001FF)) | ((uint32_t)rxupout6 << 0));
}

/**
 * @brief SEQRXDN_OUT0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  30:22           RX_DN_OUT2   0x3
 *  19:11           RX_DN_OUT1   0x7
 *  08:00           RX_DN_OUT0   0xF
 * </pre>
 */
#define RPL_SEQRXDN_OUT0_ADDR   0x00000018
#define RPL_SEQRXDN_OUT0_OFFSET 0x00000018
#define RPL_SEQRXDN_OUT0_INDEX  0x00000018
#define RPL_SEQRXDN_OUT0_RESET  0x00C0380F

__INLINE uint32_t rpl_seqrxdn_out0_get(void)
{
    return REG_RPL_RD(RPL_SEQRXDN_OUT0_ADDR);
}

__INLINE void rpl_seqrxdn_out0_set(uint32_t value)
{
    REG_RPL_WR(RPL_SEQRXDN_OUT0_ADDR, value);
}

// field definitions
#define RPL_RX_DN_OUT2_MASK   ((uint32_t)0x7FC00000)
#define RPL_RX_DN_OUT2_LSB    22
#define RPL_RX_DN_OUT2_WIDTH  ((uint32_t)0x00000009)
#define RPL_RX_DN_OUT1_MASK   ((uint32_t)0x000FF800)
#define RPL_RX_DN_OUT1_LSB    11
#define RPL_RX_DN_OUT1_WIDTH  ((uint32_t)0x00000009)
#define RPL_RX_DN_OUT0_MASK   ((uint32_t)0x000001FF)
#define RPL_RX_DN_OUT0_LSB    0
#define RPL_RX_DN_OUT0_WIDTH  ((uint32_t)0x00000009)

#define RPL_RX_DN_OUT2_RST    0x3
#define RPL_RX_DN_OUT1_RST    0x7
#define RPL_RX_DN_OUT0_RST    0xF

__INLINE void rpl_seqrxdn_out0_pack(uint16_t rxdnout2, uint16_t rxdnout1, uint16_t rxdnout0)
{
    ASSERT_ERR((((uint32_t)rxdnout2 << 22) & ~((uint32_t)0x7FC00000)) == 0);
    ASSERT_ERR((((uint32_t)rxdnout1 << 11) & ~((uint32_t)0x000FF800)) == 0);
    ASSERT_ERR((((uint32_t)rxdnout0 << 0) & ~((uint32_t)0x000001FF)) == 0);
    REG_RPL_WR(RPL_SEQRXDN_OUT0_ADDR, ((uint32_t)rxdnout2 << 22) | ((uint32_t)rxdnout1 << 11) | ((uint32_t)rxdnout0 << 0));
}

__INLINE void rpl_seqrxdn_out0_unpack(uint16_t *rxdnout2, uint16_t *rxdnout1, uint16_t *rxdnout0)
{
    uint32_t localVal = REG_RPL_RD(RPL_SEQRXDN_OUT0_ADDR);

    *rxdnout2 = (localVal & ((uint32_t)0x7FC00000)) >> 22;
    *rxdnout1 = (localVal & ((uint32_t)0x000FF800)) >> 11;
    *rxdnout0 = (localVal & ((uint32_t)0x000001FF)) >> 0;
}

__INLINE uint16_t rpl_rx_dn_out2_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_SEQRXDN_OUT0_ADDR);
    return ((localVal & ((uint32_t)0x7FC00000)) >> 22);
}

__INLINE void rpl_rx_dn_out2_setf(uint16_t rxdnout2)
{
    ASSERT_ERR((((uint32_t)rxdnout2 << 22) & ~((uint32_t)0x7FC00000)) == 0);
    REG_RPL_WR(RPL_SEQRXDN_OUT0_ADDR, (REG_RPL_RD(RPL_SEQRXDN_OUT0_ADDR) & ~((uint32_t)0x7FC00000)) | ((uint32_t)rxdnout2 << 22));
}

__INLINE uint16_t rpl_rx_dn_out1_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_SEQRXDN_OUT0_ADDR);
    return ((localVal & ((uint32_t)0x000FF800)) >> 11);
}

__INLINE void rpl_rx_dn_out1_setf(uint16_t rxdnout1)
{
    ASSERT_ERR((((uint32_t)rxdnout1 << 11) & ~((uint32_t)0x000FF800)) == 0);
    REG_RPL_WR(RPL_SEQRXDN_OUT0_ADDR, (REG_RPL_RD(RPL_SEQRXDN_OUT0_ADDR) & ~((uint32_t)0x000FF800)) | ((uint32_t)rxdnout1 << 11));
}

__INLINE uint16_t rpl_rx_dn_out0_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_SEQRXDN_OUT0_ADDR);
    return ((localVal & ((uint32_t)0x000001FF)) >> 0);
}

__INLINE void rpl_rx_dn_out0_setf(uint16_t rxdnout0)
{
    ASSERT_ERR((((uint32_t)rxdnout0 << 0) & ~((uint32_t)0x000001FF)) == 0);
    REG_RPL_WR(RPL_SEQRXDN_OUT0_ADDR, (REG_RPL_RD(RPL_SEQRXDN_OUT0_ADDR) & ~((uint32_t)0x000001FF)) | ((uint32_t)rxdnout0 << 0));
}

/**
 * @brief SEQRXDN_OUT1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  30:22           RX_DN_OUT5   0x0
 *  19:11           RX_DN_OUT4   0x0
 *  08:00           RX_DN_OUT3   0x1
 * </pre>
 */
#define RPL_SEQRXDN_OUT1_ADDR   0x00000019
#define RPL_SEQRXDN_OUT1_OFFSET 0x00000019
#define RPL_SEQRXDN_OUT1_INDEX  0x00000019
#define RPL_SEQRXDN_OUT1_RESET  0x00000001

__INLINE uint32_t rpl_seqrxdn_out1_get(void)
{
    return REG_RPL_RD(RPL_SEQRXDN_OUT1_ADDR);
}

__INLINE void rpl_seqrxdn_out1_set(uint32_t value)
{
    REG_RPL_WR(RPL_SEQRXDN_OUT1_ADDR, value);
}

// field definitions
#define RPL_RX_DN_OUT5_MASK   ((uint32_t)0x7FC00000)
#define RPL_RX_DN_OUT5_LSB    22
#define RPL_RX_DN_OUT5_WIDTH  ((uint32_t)0x00000009)
#define RPL_RX_DN_OUT4_MASK   ((uint32_t)0x000FF800)
#define RPL_RX_DN_OUT4_LSB    11
#define RPL_RX_DN_OUT4_WIDTH  ((uint32_t)0x00000009)
#define RPL_RX_DN_OUT3_MASK   ((uint32_t)0x000001FF)
#define RPL_RX_DN_OUT3_LSB    0
#define RPL_RX_DN_OUT3_WIDTH  ((uint32_t)0x00000009)

#define RPL_RX_DN_OUT5_RST    0x0
#define RPL_RX_DN_OUT4_RST    0x0
#define RPL_RX_DN_OUT3_RST    0x1

__INLINE void rpl_seqrxdn_out1_pack(uint16_t rxdnout5, uint16_t rxdnout4, uint16_t rxdnout3)
{
    ASSERT_ERR((((uint32_t)rxdnout5 << 22) & ~((uint32_t)0x7FC00000)) == 0);
    ASSERT_ERR((((uint32_t)rxdnout4 << 11) & ~((uint32_t)0x000FF800)) == 0);
    ASSERT_ERR((((uint32_t)rxdnout3 << 0) & ~((uint32_t)0x000001FF)) == 0);
    REG_RPL_WR(RPL_SEQRXDN_OUT1_ADDR, ((uint32_t)rxdnout5 << 22) | ((uint32_t)rxdnout4 << 11) | ((uint32_t)rxdnout3 << 0));
}

__INLINE void rpl_seqrxdn_out1_unpack(uint16_t *rxdnout5, uint16_t *rxdnout4, uint16_t *rxdnout3)
{
    uint32_t localVal = REG_RPL_RD(RPL_SEQRXDN_OUT1_ADDR);

    *rxdnout5 = (localVal & ((uint32_t)0x7FC00000)) >> 22;
    *rxdnout4 = (localVal & ((uint32_t)0x000FF800)) >> 11;
    *rxdnout3 = (localVal & ((uint32_t)0x000001FF)) >> 0;
}

__INLINE uint16_t rpl_rx_dn_out5_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_SEQRXDN_OUT1_ADDR);
    return ((localVal & ((uint32_t)0x7FC00000)) >> 22);
}

__INLINE void rpl_rx_dn_out5_setf(uint16_t rxdnout5)
{
    ASSERT_ERR((((uint32_t)rxdnout5 << 22) & ~((uint32_t)0x7FC00000)) == 0);
    REG_RPL_WR(RPL_SEQRXDN_OUT1_ADDR, (REG_RPL_RD(RPL_SEQRXDN_OUT1_ADDR) & ~((uint32_t)0x7FC00000)) | ((uint32_t)rxdnout5 << 22));
}

__INLINE uint16_t rpl_rx_dn_out4_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_SEQRXDN_OUT1_ADDR);
    return ((localVal & ((uint32_t)0x000FF800)) >> 11);
}

__INLINE void rpl_rx_dn_out4_setf(uint16_t rxdnout4)
{
    ASSERT_ERR((((uint32_t)rxdnout4 << 11) & ~((uint32_t)0x000FF800)) == 0);
    REG_RPL_WR(RPL_SEQRXDN_OUT1_ADDR, (REG_RPL_RD(RPL_SEQRXDN_OUT1_ADDR) & ~((uint32_t)0x000FF800)) | ((uint32_t)rxdnout4 << 11));
}

__INLINE uint16_t rpl_rx_dn_out3_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_SEQRXDN_OUT1_ADDR);
    return ((localVal & ((uint32_t)0x000001FF)) >> 0);
}

__INLINE void rpl_rx_dn_out3_setf(uint16_t rxdnout3)
{
    ASSERT_ERR((((uint32_t)rxdnout3 << 0) & ~((uint32_t)0x000001FF)) == 0);
    REG_RPL_WR(RPL_SEQRXDN_OUT1_ADDR, (REG_RPL_RD(RPL_SEQRXDN_OUT1_ADDR) & ~((uint32_t)0x000001FF)) | ((uint32_t)rxdnout3 << 0));
}

/**
 * @brief SEQRXDN_OUT2 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  30:22           RX_DN_OUT8   0x0
 *  19:11           RX_DN_OUT7   0x0
 *  08:00           RX_DN_OUT6   0x0
 * </pre>
 */
#define RPL_SEQRXDN_OUT2_ADDR   0x0000001A
#define RPL_SEQRXDN_OUT2_OFFSET 0x0000001A
#define RPL_SEQRXDN_OUT2_INDEX  0x0000001A
#define RPL_SEQRXDN_OUT2_RESET  0x00000000

__INLINE uint32_t rpl_seqrxdn_out2_get(void)
{
    return REG_RPL_RD(RPL_SEQRXDN_OUT2_ADDR);
}

__INLINE void rpl_seqrxdn_out2_set(uint32_t value)
{
    REG_RPL_WR(RPL_SEQRXDN_OUT2_ADDR, value);
}

// field definitions
#define RPL_RX_DN_OUT8_MASK   ((uint32_t)0x7FC00000)
#define RPL_RX_DN_OUT8_LSB    22
#define RPL_RX_DN_OUT8_WIDTH  ((uint32_t)0x00000009)
#define RPL_RX_DN_OUT7_MASK   ((uint32_t)0x000FF800)
#define RPL_RX_DN_OUT7_LSB    11
#define RPL_RX_DN_OUT7_WIDTH  ((uint32_t)0x00000009)
#define RPL_RX_DN_OUT6_MASK   ((uint32_t)0x000001FF)
#define RPL_RX_DN_OUT6_LSB    0
#define RPL_RX_DN_OUT6_WIDTH  ((uint32_t)0x00000009)

#define RPL_RX_DN_OUT8_RST    0x0
#define RPL_RX_DN_OUT7_RST    0x0
#define RPL_RX_DN_OUT6_RST    0x0

__INLINE void rpl_seqrxdn_out2_pack(uint16_t rxdnout8, uint16_t rxdnout7, uint16_t rxdnout6)
{
    ASSERT_ERR((((uint32_t)rxdnout8 << 22) & ~((uint32_t)0x7FC00000)) == 0);
    ASSERT_ERR((((uint32_t)rxdnout7 << 11) & ~((uint32_t)0x000FF800)) == 0);
    ASSERT_ERR((((uint32_t)rxdnout6 << 0) & ~((uint32_t)0x000001FF)) == 0);
    REG_RPL_WR(RPL_SEQRXDN_OUT2_ADDR, ((uint32_t)rxdnout8 << 22) | ((uint32_t)rxdnout7 << 11) | ((uint32_t)rxdnout6 << 0));
}

__INLINE void rpl_seqrxdn_out2_unpack(uint16_t *rxdnout8, uint16_t *rxdnout7, uint16_t *rxdnout6)
{
    uint32_t localVal = REG_RPL_RD(RPL_SEQRXDN_OUT2_ADDR);

    *rxdnout8 = (localVal & ((uint32_t)0x7FC00000)) >> 22;
    *rxdnout7 = (localVal & ((uint32_t)0x000FF800)) >> 11;
    *rxdnout6 = (localVal & ((uint32_t)0x000001FF)) >> 0;
}

__INLINE uint16_t rpl_rx_dn_out8_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_SEQRXDN_OUT2_ADDR);
    return ((localVal & ((uint32_t)0x7FC00000)) >> 22);
}

__INLINE void rpl_rx_dn_out8_setf(uint16_t rxdnout8)
{
    ASSERT_ERR((((uint32_t)rxdnout8 << 22) & ~((uint32_t)0x7FC00000)) == 0);
    REG_RPL_WR(RPL_SEQRXDN_OUT2_ADDR, (REG_RPL_RD(RPL_SEQRXDN_OUT2_ADDR) & ~((uint32_t)0x7FC00000)) | ((uint32_t)rxdnout8 << 22));
}

__INLINE uint16_t rpl_rx_dn_out7_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_SEQRXDN_OUT2_ADDR);
    return ((localVal & ((uint32_t)0x000FF800)) >> 11);
}

__INLINE void rpl_rx_dn_out7_setf(uint16_t rxdnout7)
{
    ASSERT_ERR((((uint32_t)rxdnout7 << 11) & ~((uint32_t)0x000FF800)) == 0);
    REG_RPL_WR(RPL_SEQRXDN_OUT2_ADDR, (REG_RPL_RD(RPL_SEQRXDN_OUT2_ADDR) & ~((uint32_t)0x000FF800)) | ((uint32_t)rxdnout7 << 11));
}

__INLINE uint16_t rpl_rx_dn_out6_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_SEQRXDN_OUT2_ADDR);
    return ((localVal & ((uint32_t)0x000001FF)) >> 0);
}

__INLINE void rpl_rx_dn_out6_setf(uint16_t rxdnout6)
{
    ASSERT_ERR((((uint32_t)rxdnout6 << 0) & ~((uint32_t)0x000001FF)) == 0);
    REG_RPL_WR(RPL_SEQRXDN_OUT2_ADDR, (REG_RPL_RD(RPL_SEQRXDN_OUT2_ADDR) & ~((uint32_t)0x000001FF)) | ((uint32_t)rxdnout6 << 0));
}

/**
 * @brief RFSWCNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     12           TESTMUX_EN   0
 *  09:08         SEL_INPUTBUF   0x0
 *  06:04           SEL_RFSW32   0x0
 *  02:00           SEL_RFSW10   0x0
 * </pre>
 */
#define RPL_RFSWCNTL_ADDR   0x0000001B
#define RPL_RFSWCNTL_OFFSET 0x0000001B
#define RPL_RFSWCNTL_INDEX  0x0000001B
#define RPL_RFSWCNTL_RESET  0x00000000

__INLINE uint32_t rpl_rfswcntl_get(void)
{
    return REG_RPL_RD(RPL_RFSWCNTL_ADDR);
}

__INLINE void rpl_rfswcntl_set(uint32_t value)
{
    REG_RPL_WR(RPL_RFSWCNTL_ADDR, value);
}

// field definitions
#define RPL_TESTMUX_EN_BIT      ((uint32_t)0x00001000)
#define RPL_TESTMUX_EN_POS      12
#define RPL_SEL_INPUTBUF_MASK   ((uint32_t)0x00000300)
#define RPL_SEL_INPUTBUF_LSB    8
#define RPL_SEL_INPUTBUF_WIDTH  ((uint32_t)0x00000002)
#define RPL_SEL_RFSW32_MASK     ((uint32_t)0x00000070)
#define RPL_SEL_RFSW32_LSB      4
#define RPL_SEL_RFSW32_WIDTH    ((uint32_t)0x00000003)
#define RPL_SEL_RFSW10_MASK     ((uint32_t)0x00000007)
#define RPL_SEL_RFSW10_LSB      0
#define RPL_SEL_RFSW10_WIDTH    ((uint32_t)0x00000003)

#define RPL_TESTMUX_EN_RST      0x0
#define RPL_SEL_INPUTBUF_RST    0x0
#define RPL_SEL_RFSW32_RST      0x0
#define RPL_SEL_RFSW10_RST      0x0

__INLINE void rpl_rfswcntl_pack(uint8_t testmuxen, uint8_t selinputbuf, uint8_t selrfsw32, uint8_t selrfsw10)
{
    ASSERT_ERR((((uint32_t)testmuxen << 12) & ~((uint32_t)0x00001000)) == 0);
    ASSERT_ERR((((uint32_t)selinputbuf << 8) & ~((uint32_t)0x00000300)) == 0);
    ASSERT_ERR((((uint32_t)selrfsw32 << 4) & ~((uint32_t)0x00000070)) == 0);
    ASSERT_ERR((((uint32_t)selrfsw10 << 0) & ~((uint32_t)0x00000007)) == 0);
    REG_RPL_WR(RPL_RFSWCNTL_ADDR, ((uint32_t)testmuxen << 12) | ((uint32_t)selinputbuf << 8) | ((uint32_t)selrfsw32 << 4) | ((uint32_t)selrfsw10 << 0));
}

__INLINE void rpl_rfswcntl_unpack(uint8_t *testmuxen, uint8_t *selinputbuf, uint8_t *selrfsw32, uint8_t *selrfsw10)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFSWCNTL_ADDR);

    *testmuxen = (localVal & ((uint32_t)0x00001000)) >> 12;
    *selinputbuf = (localVal & ((uint32_t)0x00000300)) >> 8;
    *selrfsw32 = (localVal & ((uint32_t)0x00000070)) >> 4;
    *selrfsw10 = (localVal & ((uint32_t)0x00000007)) >> 0;
}

__INLINE uint8_t rpl_testmux_en_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFSWCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00001000)) >> 12);
}

__INLINE void rpl_testmux_en_setf(uint8_t testmuxen)
{
    ASSERT_ERR((((uint32_t)testmuxen << 12) & ~((uint32_t)0x00001000)) == 0);
    REG_RPL_WR(RPL_RFSWCNTL_ADDR, (REG_RPL_RD(RPL_RFSWCNTL_ADDR) & ~((uint32_t)0x00001000)) | ((uint32_t)testmuxen << 12));
}

__INLINE uint8_t rpl_sel_inputbuf_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFSWCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000300)) >> 8);
}

__INLINE void rpl_sel_inputbuf_setf(uint8_t selinputbuf)
{
    ASSERT_ERR((((uint32_t)selinputbuf << 8) & ~((uint32_t)0x00000300)) == 0);
    REG_RPL_WR(RPL_RFSWCNTL_ADDR, (REG_RPL_RD(RPL_RFSWCNTL_ADDR) & ~((uint32_t)0x00000300)) | ((uint32_t)selinputbuf << 8));
}

__INLINE uint8_t rpl_sel_rfsw32_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFSWCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000070)) >> 4);
}

__INLINE void rpl_sel_rfsw32_setf(uint8_t selrfsw32)
{
    ASSERT_ERR((((uint32_t)selrfsw32 << 4) & ~((uint32_t)0x00000070)) == 0);
    REG_RPL_WR(RPL_RFSWCNTL_ADDR, (REG_RPL_RD(RPL_RFSWCNTL_ADDR) & ~((uint32_t)0x00000070)) | ((uint32_t)selrfsw32 << 4));
}

__INLINE uint8_t rpl_sel_rfsw10_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFSWCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000007)) >> 0);
}

__INLINE void rpl_sel_rfsw10_setf(uint8_t selrfsw10)
{
    ASSERT_ERR((((uint32_t)selrfsw10 << 0) & ~((uint32_t)0x00000007)) == 0);
    REG_RPL_WR(RPL_RFSWCNTL_ADDR, (REG_RPL_RD(RPL_RFSWCNTL_ADDR) & ~((uint32_t)0x00000007)) | ((uint32_t)selrfsw10 << 0));
}

/**
 * @brief RPLLCNTL0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     31      FORCE_RPLL_CTRL   0
 *  19:18           RPLL_DELTA   0x0
 *     14              XO32_ON   0
 *  13:12         XO_BIAS_TRIM   0x0
 *     11           XO_TEST_EN   0
 *     10      XO_OPENDRAIN_EN   0
 *     09         XO_REFCLK_EN   0
 *     08                XO_ON   1
 *     05       RPLL_VCOREF_ON   0
 *     04        RPLL_LOCNT_ON   0
 *     03           RPLL_LD_ON   0
 *     02          RPLL_VCO_ON   0
 *     01           RPLL_CP_ON   0
 *     00          RPLL_DIV_ON   0
 * </pre>
 */
#define RPL_RPLLCNTL0_ADDR   0x0000001C
#define RPL_RPLLCNTL0_OFFSET 0x0000001C
#define RPL_RPLLCNTL0_INDEX  0x0000001C
#define RPL_RPLLCNTL0_RESET  0x00000100

__INLINE uint32_t rpl_rpllcntl0_get(void)
{
    return REG_RPL_RD(RPL_RPLLCNTL0_ADDR);
}

__INLINE void rpl_rpllcntl0_set(uint32_t value)
{
    REG_RPL_WR(RPL_RPLLCNTL0_ADDR, value);
}

// field definitions
#define RPL_FORCE_RPLL_CTRL_BIT    ((uint32_t)0x80000000)
#define RPL_FORCE_RPLL_CTRL_POS    31
#define RPL_RPLL_DELTA_MASK        ((uint32_t)0x000C0000)
#define RPL_RPLL_DELTA_LSB         18
#define RPL_RPLL_DELTA_WIDTH       ((uint32_t)0x00000002)
#define RPL_XO32_ON_BIT            ((uint32_t)0x00004000)
#define RPL_XO32_ON_POS            14
#define RPL_XO_BIAS_TRIM_MASK      ((uint32_t)0x00003000)
#define RPL_XO_BIAS_TRIM_LSB       12
#define RPL_XO_BIAS_TRIM_WIDTH     ((uint32_t)0x00000002)
#define RPL_XO_TEST_EN_BIT         ((uint32_t)0x00000800)
#define RPL_XO_TEST_EN_POS         11
#define RPL_XO_OPENDRAIN_EN_BIT    ((uint32_t)0x00000400)
#define RPL_XO_OPENDRAIN_EN_POS    10
#define RPL_XO_REFCLK_EN_BIT       ((uint32_t)0x00000200)
#define RPL_XO_REFCLK_EN_POS       9
#define RPL_XO_ON_BIT              ((uint32_t)0x00000100)
#define RPL_XO_ON_POS              8
#define RPL_RPLL_VCOREF_ON_BIT     ((uint32_t)0x00000020)
#define RPL_RPLL_VCOREF_ON_POS     5
#define RPL_RPLL_LOCNT_ON_BIT      ((uint32_t)0x00000010)
#define RPL_RPLL_LOCNT_ON_POS      4
#define RPL_RPLL_LD_ON_BIT         ((uint32_t)0x00000008)
#define RPL_RPLL_LD_ON_POS         3
#define RPL_RPLL_VCO_ON_BIT        ((uint32_t)0x00000004)
#define RPL_RPLL_VCO_ON_POS        2
#define RPL_RPLL_CP_ON_BIT         ((uint32_t)0x00000002)
#define RPL_RPLL_CP_ON_POS         1
#define RPL_RPLL_DIV_ON_BIT        ((uint32_t)0x00000001)
#define RPL_RPLL_DIV_ON_POS        0

#define RPL_FORCE_RPLL_CTRL_RST    0x0
#define RPL_RPLL_DELTA_RST         0x0
#define RPL_XO32_ON_RST            0x0
#define RPL_XO_BIAS_TRIM_RST       0x0
#define RPL_XO_TEST_EN_RST         0x0
#define RPL_XO_OPENDRAIN_EN_RST    0x0
#define RPL_XO_REFCLK_EN_RST       0x0
#define RPL_XO_ON_RST              0x1
#define RPL_RPLL_VCOREF_ON_RST     0x0
#define RPL_RPLL_LOCNT_ON_RST      0x0
#define RPL_RPLL_LD_ON_RST         0x0
#define RPL_RPLL_VCO_ON_RST        0x0
#define RPL_RPLL_CP_ON_RST         0x0
#define RPL_RPLL_DIV_ON_RST        0x0

__INLINE void rpl_rpllcntl0_pack(uint8_t forcerpllctrl, uint8_t rplldelta, uint8_t xo32on, uint8_t xobiastrim, uint8_t xotesten, uint8_t xoopendrainen, uint8_t xorefclken, uint8_t xoon, uint8_t rpllvcorefon, uint8_t rplllocnton, uint8_t rpllldon, uint8_t rpllvcoon, uint8_t rpllcpon, uint8_t rplldivon)
{
    ASSERT_ERR((((uint32_t)forcerpllctrl << 31) & ~((uint32_t)0x80000000)) == 0);
    ASSERT_ERR((((uint32_t)rplldelta << 18) & ~((uint32_t)0x000C0000)) == 0);
    ASSERT_ERR((((uint32_t)xo32on << 14) & ~((uint32_t)0x00004000)) == 0);
    ASSERT_ERR((((uint32_t)xobiastrim << 12) & ~((uint32_t)0x00003000)) == 0);
    ASSERT_ERR((((uint32_t)xotesten << 11) & ~((uint32_t)0x00000800)) == 0);
    ASSERT_ERR((((uint32_t)xoopendrainen << 10) & ~((uint32_t)0x00000400)) == 0);
    ASSERT_ERR((((uint32_t)xorefclken << 9) & ~((uint32_t)0x00000200)) == 0);
    ASSERT_ERR((((uint32_t)xoon << 8) & ~((uint32_t)0x00000100)) == 0);
    ASSERT_ERR((((uint32_t)rpllvcorefon << 5) & ~((uint32_t)0x00000020)) == 0);
    ASSERT_ERR((((uint32_t)rplllocnton << 4) & ~((uint32_t)0x00000010)) == 0);
    ASSERT_ERR((((uint32_t)rpllldon << 3) & ~((uint32_t)0x00000008)) == 0);
    ASSERT_ERR((((uint32_t)rpllvcoon << 2) & ~((uint32_t)0x00000004)) == 0);
    ASSERT_ERR((((uint32_t)rpllcpon << 1) & ~((uint32_t)0x00000002)) == 0);
    ASSERT_ERR((((uint32_t)rplldivon << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_RPL_WR(RPL_RPLLCNTL0_ADDR, ((uint32_t)forcerpllctrl << 31) | ((uint32_t)rplldelta << 18) | ((uint32_t)xo32on << 14) | ((uint32_t)xobiastrim << 12) | ((uint32_t)xotesten << 11) | ((uint32_t)xoopendrainen << 10) | ((uint32_t)xorefclken << 9) | ((uint32_t)xoon << 8) | ((uint32_t)rpllvcorefon << 5) | ((uint32_t)rplllocnton << 4) | ((uint32_t)rpllldon << 3) | ((uint32_t)rpllvcoon << 2) | ((uint32_t)rpllcpon << 1) | ((uint32_t)rplldivon << 0));
}

__INLINE void rpl_rpllcntl0_unpack(uint8_t *forcerpllctrl, uint8_t *rplldelta, uint8_t *xo32on, uint8_t *xobiastrim, uint8_t *xotesten, uint8_t *xoopendrainen, uint8_t *xorefclken, uint8_t *xoon, uint8_t *rpllvcorefon, uint8_t *rplllocnton, uint8_t *rpllldon, uint8_t *rpllvcoon, uint8_t *rpllcpon, uint8_t *rplldivon)
{
    uint32_t localVal = REG_RPL_RD(RPL_RPLLCNTL0_ADDR);

    *forcerpllctrl = (localVal & ((uint32_t)0x80000000)) >> 31;
    *rplldelta = (localVal & ((uint32_t)0x000C0000)) >> 18;
    *xo32on = (localVal & ((uint32_t)0x00004000)) >> 14;
    *xobiastrim = (localVal & ((uint32_t)0x00003000)) >> 12;
    *xotesten = (localVal & ((uint32_t)0x00000800)) >> 11;
    *xoopendrainen = (localVal & ((uint32_t)0x00000400)) >> 10;
    *xorefclken = (localVal & ((uint32_t)0x00000200)) >> 9;
    *xoon = (localVal & ((uint32_t)0x00000100)) >> 8;
    *rpllvcorefon = (localVal & ((uint32_t)0x00000020)) >> 5;
    *rplllocnton = (localVal & ((uint32_t)0x00000010)) >> 4;
    *rpllldon = (localVal & ((uint32_t)0x00000008)) >> 3;
    *rpllvcoon = (localVal & ((uint32_t)0x00000004)) >> 2;
    *rpllcpon = (localVal & ((uint32_t)0x00000002)) >> 1;
    *rplldivon = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t rpl_force_rpll_ctrl_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RPLLCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE void rpl_force_rpll_ctrl_setf(uint8_t forcerpllctrl)
{
    ASSERT_ERR((((uint32_t)forcerpllctrl << 31) & ~((uint32_t)0x80000000)) == 0);
    REG_RPL_WR(RPL_RPLLCNTL0_ADDR, (REG_RPL_RD(RPL_RPLLCNTL0_ADDR) & ~((uint32_t)0x80000000)) | ((uint32_t)forcerpllctrl << 31));
}

__INLINE uint8_t rpl_rpll_delta_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RPLLCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x000C0000)) >> 18);
}

__INLINE void rpl_rpll_delta_setf(uint8_t rplldelta)
{
    ASSERT_ERR((((uint32_t)rplldelta << 18) & ~((uint32_t)0x000C0000)) == 0);
    REG_RPL_WR(RPL_RPLLCNTL0_ADDR, (REG_RPL_RD(RPL_RPLLCNTL0_ADDR) & ~((uint32_t)0x000C0000)) | ((uint32_t)rplldelta << 18));
}

__INLINE uint8_t rpl_xo32_on_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RPLLCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00004000)) >> 14);
}

__INLINE void rpl_xo32_on_setf(uint8_t xo32on)
{
    ASSERT_ERR((((uint32_t)xo32on << 14) & ~((uint32_t)0x00004000)) == 0);
    REG_RPL_WR(RPL_RPLLCNTL0_ADDR, (REG_RPL_RD(RPL_RPLLCNTL0_ADDR) & ~((uint32_t)0x00004000)) | ((uint32_t)xo32on << 14));
}

__INLINE uint8_t rpl_xo_bias_trim_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RPLLCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00003000)) >> 12);
}

__INLINE void rpl_xo_bias_trim_setf(uint8_t xobiastrim)
{
    ASSERT_ERR((((uint32_t)xobiastrim << 12) & ~((uint32_t)0x00003000)) == 0);
    REG_RPL_WR(RPL_RPLLCNTL0_ADDR, (REG_RPL_RD(RPL_RPLLCNTL0_ADDR) & ~((uint32_t)0x00003000)) | ((uint32_t)xobiastrim << 12));
}

__INLINE uint8_t rpl_xo_test_en_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RPLLCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00000800)) >> 11);
}

__INLINE void rpl_xo_test_en_setf(uint8_t xotesten)
{
    ASSERT_ERR((((uint32_t)xotesten << 11) & ~((uint32_t)0x00000800)) == 0);
    REG_RPL_WR(RPL_RPLLCNTL0_ADDR, (REG_RPL_RD(RPL_RPLLCNTL0_ADDR) & ~((uint32_t)0x00000800)) | ((uint32_t)xotesten << 11));
}

__INLINE uint8_t rpl_xo_opendrain_en_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RPLLCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00000400)) >> 10);
}

__INLINE void rpl_xo_opendrain_en_setf(uint8_t xoopendrainen)
{
    ASSERT_ERR((((uint32_t)xoopendrainen << 10) & ~((uint32_t)0x00000400)) == 0);
    REG_RPL_WR(RPL_RPLLCNTL0_ADDR, (REG_RPL_RD(RPL_RPLLCNTL0_ADDR) & ~((uint32_t)0x00000400)) | ((uint32_t)xoopendrainen << 10));
}

__INLINE uint8_t rpl_xo_refclk_en_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RPLLCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00000200)) >> 9);
}

__INLINE void rpl_xo_refclk_en_setf(uint8_t xorefclken)
{
    ASSERT_ERR((((uint32_t)xorefclken << 9) & ~((uint32_t)0x00000200)) == 0);
    REG_RPL_WR(RPL_RPLLCNTL0_ADDR, (REG_RPL_RD(RPL_RPLLCNTL0_ADDR) & ~((uint32_t)0x00000200)) | ((uint32_t)xorefclken << 9));
}

__INLINE uint8_t rpl_xo_on_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RPLLCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00000100)) >> 8);
}

__INLINE void rpl_xo_on_setf(uint8_t xoon)
{
    ASSERT_ERR((((uint32_t)xoon << 8) & ~((uint32_t)0x00000100)) == 0);
    REG_RPL_WR(RPL_RPLLCNTL0_ADDR, (REG_RPL_RD(RPL_RPLLCNTL0_ADDR) & ~((uint32_t)0x00000100)) | ((uint32_t)xoon << 8));
}

__INLINE uint8_t rpl_rpll_vcoref_on_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RPLLCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00000020)) >> 5);
}

__INLINE void rpl_rpll_vcoref_on_setf(uint8_t rpllvcorefon)
{
    ASSERT_ERR((((uint32_t)rpllvcorefon << 5) & ~((uint32_t)0x00000020)) == 0);
    REG_RPL_WR(RPL_RPLLCNTL0_ADDR, (REG_RPL_RD(RPL_RPLLCNTL0_ADDR) & ~((uint32_t)0x00000020)) | ((uint32_t)rpllvcorefon << 5));
}

__INLINE uint8_t rpl_rpll_locnt_on_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RPLLCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE void rpl_rpll_locnt_on_setf(uint8_t rplllocnton)
{
    ASSERT_ERR((((uint32_t)rplllocnton << 4) & ~((uint32_t)0x00000010)) == 0);
    REG_RPL_WR(RPL_RPLLCNTL0_ADDR, (REG_RPL_RD(RPL_RPLLCNTL0_ADDR) & ~((uint32_t)0x00000010)) | ((uint32_t)rplllocnton << 4));
}

__INLINE uint8_t rpl_rpll_ld_on_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RPLLCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

__INLINE void rpl_rpll_ld_on_setf(uint8_t rpllldon)
{
    ASSERT_ERR((((uint32_t)rpllldon << 3) & ~((uint32_t)0x00000008)) == 0);
    REG_RPL_WR(RPL_RPLLCNTL0_ADDR, (REG_RPL_RD(RPL_RPLLCNTL0_ADDR) & ~((uint32_t)0x00000008)) | ((uint32_t)rpllldon << 3));
}

__INLINE uint8_t rpl_rpll_vco_on_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RPLLCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE void rpl_rpll_vco_on_setf(uint8_t rpllvcoon)
{
    ASSERT_ERR((((uint32_t)rpllvcoon << 2) & ~((uint32_t)0x00000004)) == 0);
    REG_RPL_WR(RPL_RPLLCNTL0_ADDR, (REG_RPL_RD(RPL_RPLLCNTL0_ADDR) & ~((uint32_t)0x00000004)) | ((uint32_t)rpllvcoon << 2));
}

__INLINE uint8_t rpl_rpll_cp_on_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RPLLCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE void rpl_rpll_cp_on_setf(uint8_t rpllcpon)
{
    ASSERT_ERR((((uint32_t)rpllcpon << 1) & ~((uint32_t)0x00000002)) == 0);
    REG_RPL_WR(RPL_RPLLCNTL0_ADDR, (REG_RPL_RD(RPL_RPLLCNTL0_ADDR) & ~((uint32_t)0x00000002)) | ((uint32_t)rpllcpon << 1));
}

__INLINE uint8_t rpl_rpll_div_on_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RPLLCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void rpl_rpll_div_on_setf(uint8_t rplldivon)
{
    ASSERT_ERR((((uint32_t)rplldivon << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_RPL_WR(RPL_RPLLCNTL0_ADDR, (REG_RPL_RD(RPL_RPLLCNTL0_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)rplldivon << 0));
}

/**
 * @brief RPLLCNTL1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  25:24   RPLL_IVCO_BUF_TRIM   0x0
 *     22   RPLL_VCOREFBACKUP_EN   0
 *     20            RPLL_NREF   1
 *  18:16         RPLL_CP_TEST   0x0
 *  14:12      RPLL_ICP_OFFSET   0x5
 *  11:08             RPLL_ICP   0x4
 *  07:04           RPLL_VCOFC   0x0
 *  03:02            RPLL_IVCO   0x3
 *  01:00        RPLL_VCO_MODE   0x0
 * </pre>
 */
#define RPL_RPLLCNTL1_ADDR   0x0000001D
#define RPL_RPLLCNTL1_OFFSET 0x0000001D
#define RPL_RPLLCNTL1_INDEX  0x0000001D
#define RPL_RPLLCNTL1_RESET  0x0010540C

__INLINE uint32_t rpl_rpllcntl1_get(void)
{
    return REG_RPL_RD(RPL_RPLLCNTL1_ADDR);
}

__INLINE void rpl_rpllcntl1_set(uint32_t value)
{
    REG_RPL_WR(RPL_RPLLCNTL1_ADDR, value);
}

// field definitions
#define RPL_RPLL_IVCO_BUF_TRIM_MASK     ((uint32_t)0x03000000)
#define RPL_RPLL_IVCO_BUF_TRIM_LSB      24
#define RPL_RPLL_IVCO_BUF_TRIM_WIDTH    ((uint32_t)0x00000002)
#define RPL_RPLL_VCOREFBACKUP_EN_BIT    ((uint32_t)0x00400000)
#define RPL_RPLL_VCOREFBACKUP_EN_POS    22
#define RPL_RPLL_NREF_BIT               ((uint32_t)0x00100000)
#define RPL_RPLL_NREF_POS               20
#define RPL_RPLL_CP_TEST_MASK           ((uint32_t)0x00070000)
#define RPL_RPLL_CP_TEST_LSB            16
#define RPL_RPLL_CP_TEST_WIDTH          ((uint32_t)0x00000003)
#define RPL_RPLL_ICP_OFFSET_MASK        ((uint32_t)0x00007000)
#define RPL_RPLL_ICP_OFFSET_LSB         12
#define RPL_RPLL_ICP_OFFSET_WIDTH       ((uint32_t)0x00000003)
#define RPL_RPLL_ICP_MASK               ((uint32_t)0x00000F00)
#define RPL_RPLL_ICP_LSB                8
#define RPL_RPLL_ICP_WIDTH              ((uint32_t)0x00000004)
#define RPL_RPLL_VCOFC_MASK             ((uint32_t)0x000000F0)
#define RPL_RPLL_VCOFC_LSB              4
#define RPL_RPLL_VCOFC_WIDTH            ((uint32_t)0x00000004)
#define RPL_RPLL_IVCO_MASK              ((uint32_t)0x0000000C)
#define RPL_RPLL_IVCO_LSB               2
#define RPL_RPLL_IVCO_WIDTH             ((uint32_t)0x00000002)
#define RPL_RPLL_VCO_MODE_MASK          ((uint32_t)0x00000003)
#define RPL_RPLL_VCO_MODE_LSB           0
#define RPL_RPLL_VCO_MODE_WIDTH         ((uint32_t)0x00000002)

#define RPL_RPLL_IVCO_BUF_TRIM_RST      0x0
#define RPL_RPLL_VCOREFBACKUP_EN_RST    0x0
#define RPL_RPLL_NREF_RST               0x1
#define RPL_RPLL_CP_TEST_RST            0x0
#define RPL_RPLL_ICP_OFFSET_RST         0x5
#define RPL_RPLL_ICP_RST                0x4
#define RPL_RPLL_VCOFC_RST              0x0
#define RPL_RPLL_IVCO_RST               0x3
#define RPL_RPLL_VCO_MODE_RST           0x0

__INLINE void rpl_rpllcntl1_pack(uint8_t rpllivcobuftrim, uint8_t rpllvcorefbackupen, uint8_t rpllnref, uint8_t rpllcptest, uint8_t rpllicpoffset, uint8_t rpllicp, uint8_t rpllvcofc, uint8_t rpllivco, uint8_t rpllvcomode)
{
    ASSERT_ERR((((uint32_t)rpllivcobuftrim << 24) & ~((uint32_t)0x03000000)) == 0);
    ASSERT_ERR((((uint32_t)rpllvcorefbackupen << 22) & ~((uint32_t)0x00400000)) == 0);
    ASSERT_ERR((((uint32_t)rpllnref << 20) & ~((uint32_t)0x00100000)) == 0);
    ASSERT_ERR((((uint32_t)rpllcptest << 16) & ~((uint32_t)0x00070000)) == 0);
    ASSERT_ERR((((uint32_t)rpllicpoffset << 12) & ~((uint32_t)0x00007000)) == 0);
    ASSERT_ERR((((uint32_t)rpllicp << 8) & ~((uint32_t)0x00000F00)) == 0);
    ASSERT_ERR((((uint32_t)rpllvcofc << 4) & ~((uint32_t)0x000000F0)) == 0);
    ASSERT_ERR((((uint32_t)rpllivco << 2) & ~((uint32_t)0x0000000C)) == 0);
    ASSERT_ERR((((uint32_t)rpllvcomode << 0) & ~((uint32_t)0x00000003)) == 0);
    REG_RPL_WR(RPL_RPLLCNTL1_ADDR, ((uint32_t)rpllivcobuftrim << 24) | ((uint32_t)rpllvcorefbackupen << 22) | ((uint32_t)rpllnref << 20) | ((uint32_t)rpllcptest << 16) | ((uint32_t)rpllicpoffset << 12) | ((uint32_t)rpllicp << 8) | ((uint32_t)rpllvcofc << 4) | ((uint32_t)rpllivco << 2) | ((uint32_t)rpllvcomode << 0));
}

__INLINE void rpl_rpllcntl1_unpack(uint8_t *rpllivcobuftrim, uint8_t *rpllvcorefbackupen, uint8_t *rpllnref, uint8_t *rpllcptest, uint8_t *rpllicpoffset, uint8_t *rpllicp, uint8_t *rpllvcofc, uint8_t *rpllivco, uint8_t *rpllvcomode)
{
    uint32_t localVal = REG_RPL_RD(RPL_RPLLCNTL1_ADDR);

    *rpllivcobuftrim = (localVal & ((uint32_t)0x03000000)) >> 24;
    *rpllvcorefbackupen = (localVal & ((uint32_t)0x00400000)) >> 22;
    *rpllnref = (localVal & ((uint32_t)0x00100000)) >> 20;
    *rpllcptest = (localVal & ((uint32_t)0x00070000)) >> 16;
    *rpllicpoffset = (localVal & ((uint32_t)0x00007000)) >> 12;
    *rpllicp = (localVal & ((uint32_t)0x00000F00)) >> 8;
    *rpllvcofc = (localVal & ((uint32_t)0x000000F0)) >> 4;
    *rpllivco = (localVal & ((uint32_t)0x0000000C)) >> 2;
    *rpllvcomode = (localVal & ((uint32_t)0x00000003)) >> 0;
}

__INLINE uint8_t rpl_rpll_ivco_buf_trim_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RPLLCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x03000000)) >> 24);
}

__INLINE void rpl_rpll_ivco_buf_trim_setf(uint8_t rpllivcobuftrim)
{
    ASSERT_ERR((((uint32_t)rpllivcobuftrim << 24) & ~((uint32_t)0x03000000)) == 0);
    REG_RPL_WR(RPL_RPLLCNTL1_ADDR, (REG_RPL_RD(RPL_RPLLCNTL1_ADDR) & ~((uint32_t)0x03000000)) | ((uint32_t)rpllivcobuftrim << 24));
}

__INLINE uint8_t rpl_rpll_vcorefbackup_en_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RPLLCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x00400000)) >> 22);
}

__INLINE void rpl_rpll_vcorefbackup_en_setf(uint8_t rpllvcorefbackupen)
{
    ASSERT_ERR((((uint32_t)rpllvcorefbackupen << 22) & ~((uint32_t)0x00400000)) == 0);
    REG_RPL_WR(RPL_RPLLCNTL1_ADDR, (REG_RPL_RD(RPL_RPLLCNTL1_ADDR) & ~((uint32_t)0x00400000)) | ((uint32_t)rpllvcorefbackupen << 22));
}

__INLINE uint8_t rpl_rpll_nref_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RPLLCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x00100000)) >> 20);
}

__INLINE void rpl_rpll_nref_setf(uint8_t rpllnref)
{
    ASSERT_ERR((((uint32_t)rpllnref << 20) & ~((uint32_t)0x00100000)) == 0);
    REG_RPL_WR(RPL_RPLLCNTL1_ADDR, (REG_RPL_RD(RPL_RPLLCNTL1_ADDR) & ~((uint32_t)0x00100000)) | ((uint32_t)rpllnref << 20));
}

__INLINE uint8_t rpl_rpll_cp_test_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RPLLCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x00070000)) >> 16);
}

__INLINE void rpl_rpll_cp_test_setf(uint8_t rpllcptest)
{
    ASSERT_ERR((((uint32_t)rpllcptest << 16) & ~((uint32_t)0x00070000)) == 0);
    REG_RPL_WR(RPL_RPLLCNTL1_ADDR, (REG_RPL_RD(RPL_RPLLCNTL1_ADDR) & ~((uint32_t)0x00070000)) | ((uint32_t)rpllcptest << 16));
}

__INLINE uint8_t rpl_rpll_icp_offset_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RPLLCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x00007000)) >> 12);
}

__INLINE void rpl_rpll_icp_offset_setf(uint8_t rpllicpoffset)
{
    ASSERT_ERR((((uint32_t)rpllicpoffset << 12) & ~((uint32_t)0x00007000)) == 0);
    REG_RPL_WR(RPL_RPLLCNTL1_ADDR, (REG_RPL_RD(RPL_RPLLCNTL1_ADDR) & ~((uint32_t)0x00007000)) | ((uint32_t)rpllicpoffset << 12));
}

__INLINE uint8_t rpl_rpll_icp_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RPLLCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x00000F00)) >> 8);
}

__INLINE void rpl_rpll_icp_setf(uint8_t rpllicp)
{
    ASSERT_ERR((((uint32_t)rpllicp << 8) & ~((uint32_t)0x00000F00)) == 0);
    REG_RPL_WR(RPL_RPLLCNTL1_ADDR, (REG_RPL_RD(RPL_RPLLCNTL1_ADDR) & ~((uint32_t)0x00000F00)) | ((uint32_t)rpllicp << 8));
}

__INLINE uint8_t rpl_rpll_vcofc_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RPLLCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x000000F0)) >> 4);
}

__INLINE void rpl_rpll_vcofc_setf(uint8_t rpllvcofc)
{
    ASSERT_ERR((((uint32_t)rpllvcofc << 4) & ~((uint32_t)0x000000F0)) == 0);
    REG_RPL_WR(RPL_RPLLCNTL1_ADDR, (REG_RPL_RD(RPL_RPLLCNTL1_ADDR) & ~((uint32_t)0x000000F0)) | ((uint32_t)rpllvcofc << 4));
}

__INLINE uint8_t rpl_rpll_ivco_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RPLLCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x0000000C)) >> 2);
}

__INLINE void rpl_rpll_ivco_setf(uint8_t rpllivco)
{
    ASSERT_ERR((((uint32_t)rpllivco << 2) & ~((uint32_t)0x0000000C)) == 0);
    REG_RPL_WR(RPL_RPLLCNTL1_ADDR, (REG_RPL_RD(RPL_RPLLCNTL1_ADDR) & ~((uint32_t)0x0000000C)) | ((uint32_t)rpllivco << 2));
}

__INLINE uint8_t rpl_rpll_vco_mode_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RPLLCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x00000003)) >> 0);
}

__INLINE void rpl_rpll_vco_mode_setf(uint8_t rpllvcomode)
{
    ASSERT_ERR((((uint32_t)rpllvcomode << 0) & ~((uint32_t)0x00000003)) == 0);
    REG_RPL_WR(RPL_RPLLCNTL1_ADDR, (REG_RPL_RD(RPL_RPLLCNTL1_ADDR) & ~((uint32_t)0x00000003)) | ((uint32_t)rpllvcomode << 0));
}

/**
 * @brief RPLLCNTL2 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     21   RPLL_LD_FREF_INV_EN   0
 *     20   RPLL_LD_FDIV_INV_EN   0
 *  19:18      RPLL_EDGES_COMP   0x2
 *     17   RPLL_FE_DETECT_VAL   0
 *     16   RPLL_RE_DETECT_VAL   0
 *  11:08      RPLL_UNLOCK_CNT   0x3
 *  07:00        RPLL_LOCK_CNT   0x46
 * </pre>
 */
#define RPL_RPLLCNTL2_ADDR   0x0000001E
#define RPL_RPLLCNTL2_OFFSET 0x0000001E
#define RPL_RPLLCNTL2_INDEX  0x0000001E
#define RPL_RPLLCNTL2_RESET  0x00080346

__INLINE uint32_t rpl_rpllcntl2_get(void)
{
    return REG_RPL_RD(RPL_RPLLCNTL2_ADDR);
}

__INLINE void rpl_rpllcntl2_set(uint32_t value)
{
    REG_RPL_WR(RPL_RPLLCNTL2_ADDR, value);
}

// field definitions
#define RPL_RPLL_LD_FREF_INV_EN_BIT    ((uint32_t)0x00200000)
#define RPL_RPLL_LD_FREF_INV_EN_POS    21
#define RPL_RPLL_LD_FDIV_INV_EN_BIT    ((uint32_t)0x00100000)
#define RPL_RPLL_LD_FDIV_INV_EN_POS    20
#define RPL_RPLL_EDGES_COMP_MASK       ((uint32_t)0x000C0000)
#define RPL_RPLL_EDGES_COMP_LSB        18
#define RPL_RPLL_EDGES_COMP_WIDTH      ((uint32_t)0x00000002)
#define RPL_RPLL_FE_DETECT_VAL_BIT     ((uint32_t)0x00020000)
#define RPL_RPLL_FE_DETECT_VAL_POS     17
#define RPL_RPLL_RE_DETECT_VAL_BIT     ((uint32_t)0x00010000)
#define RPL_RPLL_RE_DETECT_VAL_POS     16
#define RPL_RPLL_UNLOCK_CNT_MASK       ((uint32_t)0x00000F00)
#define RPL_RPLL_UNLOCK_CNT_LSB        8
#define RPL_RPLL_UNLOCK_CNT_WIDTH      ((uint32_t)0x00000004)
#define RPL_RPLL_LOCK_CNT_MASK         ((uint32_t)0x000000FF)
#define RPL_RPLL_LOCK_CNT_LSB          0
#define RPL_RPLL_LOCK_CNT_WIDTH        ((uint32_t)0x00000008)

#define RPL_RPLL_LD_FREF_INV_EN_RST    0x0
#define RPL_RPLL_LD_FDIV_INV_EN_RST    0x0
#define RPL_RPLL_EDGES_COMP_RST        0x2
#define RPL_RPLL_FE_DETECT_VAL_RST     0x0
#define RPL_RPLL_RE_DETECT_VAL_RST     0x0
#define RPL_RPLL_UNLOCK_CNT_RST        0x3
#define RPL_RPLL_LOCK_CNT_RST          0x46

__INLINE void rpl_rpllcntl2_pack(uint8_t rpllldfrefinven, uint8_t rpllldfdivinven, uint8_t rplledgescomp, uint8_t rpllfedetectval, uint8_t rpllredetectval, uint8_t rpllunlockcnt, uint8_t rplllockcnt)
{
    ASSERT_ERR((((uint32_t)rpllldfrefinven << 21) & ~((uint32_t)0x00200000)) == 0);
    ASSERT_ERR((((uint32_t)rpllldfdivinven << 20) & ~((uint32_t)0x00100000)) == 0);
    ASSERT_ERR((((uint32_t)rplledgescomp << 18) & ~((uint32_t)0x000C0000)) == 0);
    ASSERT_ERR((((uint32_t)rpllfedetectval << 17) & ~((uint32_t)0x00020000)) == 0);
    ASSERT_ERR((((uint32_t)rpllredetectval << 16) & ~((uint32_t)0x00010000)) == 0);
    ASSERT_ERR((((uint32_t)rpllunlockcnt << 8) & ~((uint32_t)0x00000F00)) == 0);
    ASSERT_ERR((((uint32_t)rplllockcnt << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_RPL_WR(RPL_RPLLCNTL2_ADDR, ((uint32_t)rpllldfrefinven << 21) | ((uint32_t)rpllldfdivinven << 20) | ((uint32_t)rplledgescomp << 18) | ((uint32_t)rpllfedetectval << 17) | ((uint32_t)rpllredetectval << 16) | ((uint32_t)rpllunlockcnt << 8) | ((uint32_t)rplllockcnt << 0));
}

__INLINE void rpl_rpllcntl2_unpack(uint8_t *rpllldfrefinven, uint8_t *rpllldfdivinven, uint8_t *rplledgescomp, uint8_t *rpllfedetectval, uint8_t *rpllredetectval, uint8_t *rpllunlockcnt, uint8_t *rplllockcnt)
{
    uint32_t localVal = REG_RPL_RD(RPL_RPLLCNTL2_ADDR);

    *rpllldfrefinven = (localVal & ((uint32_t)0x00200000)) >> 21;
    *rpllldfdivinven = (localVal & ((uint32_t)0x00100000)) >> 20;
    *rplledgescomp = (localVal & ((uint32_t)0x000C0000)) >> 18;
    *rpllfedetectval = (localVal & ((uint32_t)0x00020000)) >> 17;
    *rpllredetectval = (localVal & ((uint32_t)0x00010000)) >> 16;
    *rpllunlockcnt = (localVal & ((uint32_t)0x00000F00)) >> 8;
    *rplllockcnt = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint8_t rpl_rpll_ld_fref_inv_en_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RPLLCNTL2_ADDR);
    return ((localVal & ((uint32_t)0x00200000)) >> 21);
}

__INLINE void rpl_rpll_ld_fref_inv_en_setf(uint8_t rpllldfrefinven)
{
    ASSERT_ERR((((uint32_t)rpllldfrefinven << 21) & ~((uint32_t)0x00200000)) == 0);
    REG_RPL_WR(RPL_RPLLCNTL2_ADDR, (REG_RPL_RD(RPL_RPLLCNTL2_ADDR) & ~((uint32_t)0x00200000)) | ((uint32_t)rpllldfrefinven << 21));
}

__INLINE uint8_t rpl_rpll_ld_fdiv_inv_en_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RPLLCNTL2_ADDR);
    return ((localVal & ((uint32_t)0x00100000)) >> 20);
}

__INLINE void rpl_rpll_ld_fdiv_inv_en_setf(uint8_t rpllldfdivinven)
{
    ASSERT_ERR((((uint32_t)rpllldfdivinven << 20) & ~((uint32_t)0x00100000)) == 0);
    REG_RPL_WR(RPL_RPLLCNTL2_ADDR, (REG_RPL_RD(RPL_RPLLCNTL2_ADDR) & ~((uint32_t)0x00100000)) | ((uint32_t)rpllldfdivinven << 20));
}

__INLINE uint8_t rpl_rpll_edges_comp_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RPLLCNTL2_ADDR);
    return ((localVal & ((uint32_t)0x000C0000)) >> 18);
}

__INLINE void rpl_rpll_edges_comp_setf(uint8_t rplledgescomp)
{
    ASSERT_ERR((((uint32_t)rplledgescomp << 18) & ~((uint32_t)0x000C0000)) == 0);
    REG_RPL_WR(RPL_RPLLCNTL2_ADDR, (REG_RPL_RD(RPL_RPLLCNTL2_ADDR) & ~((uint32_t)0x000C0000)) | ((uint32_t)rplledgescomp << 18));
}

__INLINE uint8_t rpl_rpll_fe_detect_val_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RPLLCNTL2_ADDR);
    return ((localVal & ((uint32_t)0x00020000)) >> 17);
}

__INLINE void rpl_rpll_fe_detect_val_setf(uint8_t rpllfedetectval)
{
    ASSERT_ERR((((uint32_t)rpllfedetectval << 17) & ~((uint32_t)0x00020000)) == 0);
    REG_RPL_WR(RPL_RPLLCNTL2_ADDR, (REG_RPL_RD(RPL_RPLLCNTL2_ADDR) & ~((uint32_t)0x00020000)) | ((uint32_t)rpllfedetectval << 17));
}

__INLINE uint8_t rpl_rpll_re_detect_val_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RPLLCNTL2_ADDR);
    return ((localVal & ((uint32_t)0x00010000)) >> 16);
}

__INLINE void rpl_rpll_re_detect_val_setf(uint8_t rpllredetectval)
{
    ASSERT_ERR((((uint32_t)rpllredetectval << 16) & ~((uint32_t)0x00010000)) == 0);
    REG_RPL_WR(RPL_RPLLCNTL2_ADDR, (REG_RPL_RD(RPL_RPLLCNTL2_ADDR) & ~((uint32_t)0x00010000)) | ((uint32_t)rpllredetectval << 16));
}

__INLINE uint8_t rpl_rpll_unlock_cnt_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RPLLCNTL2_ADDR);
    return ((localVal & ((uint32_t)0x00000F00)) >> 8);
}

__INLINE void rpl_rpll_unlock_cnt_setf(uint8_t rpllunlockcnt)
{
    ASSERT_ERR((((uint32_t)rpllunlockcnt << 8) & ~((uint32_t)0x00000F00)) == 0);
    REG_RPL_WR(RPL_RPLLCNTL2_ADDR, (REG_RPL_RD(RPL_RPLLCNTL2_ADDR) & ~((uint32_t)0x00000F00)) | ((uint32_t)rpllunlockcnt << 8));
}

__INLINE uint8_t rpl_rpll_lock_cnt_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RPLLCNTL2_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

__INLINE void rpl_rpll_lock_cnt_setf(uint8_t rplllockcnt)
{
    ASSERT_ERR((((uint32_t)rplllockcnt << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_RPL_WR(RPL_RPLLCNTL2_ADDR, (REG_RPL_RD(RPL_RPLLCNTL2_ADDR) & ~((uint32_t)0x000000FF)) | ((uint32_t)rplllockcnt << 0));
}

/**
 * @brief RFPLLSTAT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:20       RPLL_LOCNT_VAL   0x0
 *     03      RPLL_LOCNT_DONE   0
 *     01        FIFO_OVERFLOW   0
 *     00          RPLL_LOCKED   0
 * </pre>
 */
#define RPL_RFPLLSTAT_ADDR   0x0000001F
#define RPL_RFPLLSTAT_OFFSET 0x0000001F
#define RPL_RFPLLSTAT_INDEX  0x0000001F
#define RPL_RFPLLSTAT_RESET  0x00000000

__INLINE uint32_t rpl_rfpllstat_get(void)
{
    return REG_RPL_RD(RPL_RFPLLSTAT_ADDR);
}

// field definitions
#define RPL_RPLL_LOCNT_VAL_MASK    ((uint32_t)0xFFF00000)
#define RPL_RPLL_LOCNT_VAL_LSB     20
#define RPL_RPLL_LOCNT_VAL_WIDTH   ((uint32_t)0x0000000C)
#define RPL_RPLL_LOCNT_DONE_BIT    ((uint32_t)0x00000008)
#define RPL_RPLL_LOCNT_DONE_POS    3
#define RPL_FIFO_OVERFLOW_BIT      ((uint32_t)0x00000002)
#define RPL_FIFO_OVERFLOW_POS      1
#define RPL_RPLL_LOCKED_BIT        ((uint32_t)0x00000001)
#define RPL_RPLL_LOCKED_POS        0

#define RPL_RPLL_LOCNT_VAL_RST     0x0
#define RPL_RPLL_LOCNT_DONE_RST    0x0
#define RPL_FIFO_OVERFLOW_RST      0x0
#define RPL_RPLL_LOCKED_RST        0x0

__INLINE void rpl_rfpllstat_unpack(uint16_t *rplllocntval, uint8_t *rplllocntdone, uint8_t *fifooverflow, uint8_t *rplllocked)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFPLLSTAT_ADDR);

    *rplllocntval = (localVal & ((uint32_t)0xFFF00000)) >> 20;
    *rplllocntdone = (localVal & ((uint32_t)0x00000008)) >> 3;
    *fifooverflow = (localVal & ((uint32_t)0x00000002)) >> 1;
    *rplllocked = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint16_t rpl_rpll_locnt_val_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFPLLSTAT_ADDR);
    return ((localVal & ((uint32_t)0xFFF00000)) >> 20);
}

__INLINE uint8_t rpl_rpll_locnt_done_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFPLLSTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

__INLINE uint8_t rpl_fifo_overflow_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFPLLSTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE uint8_t rpl_rpll_locked_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFPLLSTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

/**
 * @brief RPLLSDCNTL0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:08         RPLL_N0FRACT   0x627627
 *  07:00           RPLL_N0INT   0x5C
 * </pre>
 */
#define RPL_RPLLSDCNTL0_ADDR   0x00000020
#define RPL_RPLLSDCNTL0_OFFSET 0x00000020
#define RPL_RPLLSDCNTL0_INDEX  0x00000020
#define RPL_RPLLSDCNTL0_RESET  0x6276275C

__INLINE uint32_t rpl_rpllsdcntl0_get(void)
{
    return REG_RPL_RD(RPL_RPLLSDCNTL0_ADDR);
}

__INLINE void rpl_rpllsdcntl0_set(uint32_t value)
{
    REG_RPL_WR(RPL_RPLLSDCNTL0_ADDR, value);
}

// field definitions
#define RPL_RPLL_N0FRACT_MASK   ((uint32_t)0xFFFFFF00)
#define RPL_RPLL_N0FRACT_LSB    8
#define RPL_RPLL_N0FRACT_WIDTH  ((uint32_t)0x00000018)
#define RPL_RPLL_N0INT_MASK     ((uint32_t)0x000000FF)
#define RPL_RPLL_N0INT_LSB      0
#define RPL_RPLL_N0INT_WIDTH    ((uint32_t)0x00000008)

#define RPL_RPLL_N0FRACT_RST    0x627627
#define RPL_RPLL_N0INT_RST      0x5C

__INLINE void rpl_rpllsdcntl0_pack(uint32_t rplln0fract, uint8_t rplln0int)
{
    ASSERT_ERR((((uint32_t)rplln0fract << 8) & ~((uint32_t)0xFFFFFF00)) == 0);
    ASSERT_ERR((((uint32_t)rplln0int << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_RPL_WR(RPL_RPLLSDCNTL0_ADDR, ((uint32_t)rplln0fract << 8) | ((uint32_t)rplln0int << 0));
}

__INLINE void rpl_rpllsdcntl0_unpack(uint32_t *rplln0fract, uint8_t *rplln0int)
{
    uint32_t localVal = REG_RPL_RD(RPL_RPLLSDCNTL0_ADDR);

    *rplln0fract = (localVal & ((uint32_t)0xFFFFFF00)) >> 8;
    *rplln0int = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint32_t rpl_rpll_n0fract_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RPLLSDCNTL0_ADDR);
    return ((localVal & ((uint32_t)0xFFFFFF00)) >> 8);
}

__INLINE void rpl_rpll_n0fract_setf(uint32_t rplln0fract)
{
    ASSERT_ERR((((uint32_t)rplln0fract << 8) & ~((uint32_t)0xFFFFFF00)) == 0);
    REG_RPL_WR(RPL_RPLLSDCNTL0_ADDR, (REG_RPL_RD(RPL_RPLLSDCNTL0_ADDR) & ~((uint32_t)0xFFFFFF00)) | ((uint32_t)rplln0fract << 8));
}

__INLINE uint8_t rpl_rpll_n0int_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RPLLSDCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

__INLINE void rpl_rpll_n0int_setf(uint8_t rplln0int)
{
    ASSERT_ERR((((uint32_t)rplln0int << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_RPL_WR(RPL_RPLLSDCNTL0_ADDR, (REG_RPL_RD(RPL_RPLLSDCNTL0_ADDR) & ~((uint32_t)0x000000FF)) | ((uint32_t)rplln0int << 0));
}

/**
 * @brief RPLLSDCNTL1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  23:00         RPLL_CHFRACT   0x9D89D
 * </pre>
 */
#define RPL_RPLLSDCNTL1_ADDR   0x00000021
#define RPL_RPLLSDCNTL1_OFFSET 0x00000021
#define RPL_RPLLSDCNTL1_INDEX  0x00000021
#define RPL_RPLLSDCNTL1_RESET  0x0009D89D

__INLINE uint32_t rpl_rpllsdcntl1_get(void)
{
    return REG_RPL_RD(RPL_RPLLSDCNTL1_ADDR);
}

__INLINE void rpl_rpllsdcntl1_set(uint32_t value)
{
    REG_RPL_WR(RPL_RPLLSDCNTL1_ADDR, value);
}

// field definitions
#define RPL_RPLL_CHFRACT_MASK   ((uint32_t)0x00FFFFFF)
#define RPL_RPLL_CHFRACT_LSB    0
#define RPL_RPLL_CHFRACT_WIDTH  ((uint32_t)0x00000018)

#define RPL_RPLL_CHFRACT_RST    0x9D89D

__INLINE uint32_t rpl_rpll_chfract_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RPLLSDCNTL1_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x00FFFFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void rpl_rpll_chfract_setf(uint32_t rpllchfract)
{
    ASSERT_ERR((((uint32_t)rpllchfract << 0) & ~((uint32_t)0x00FFFFFF)) == 0);
    REG_RPL_WR(RPL_RPLLSDCNTL1_ADDR, (uint32_t)rpllchfract << 0);
}

/**
 * @brief RPLLSDCNTL2 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     31         BTLE_DUMO_EN   1
 *  23:16   BTLE_FILTER_SCALING   0x2F
 *  15:08       FILTER_SCALING   0x47
 *     05            FMTX2P_EN   0
 *     04              FMTX_EN   1
 *     03          SDDITHER_EN   1
 *  02:01         RPLL_SDORDER   0x3
 *     00         RPLL_FRAC_EN   1
 * </pre>
 */
#define RPL_RPLLSDCNTL2_ADDR   0x00000022
#define RPL_RPLLSDCNTL2_OFFSET 0x00000022
#define RPL_RPLLSDCNTL2_INDEX  0x00000022
#define RPL_RPLLSDCNTL2_RESET  0x802F471F

__INLINE uint32_t rpl_rpllsdcntl2_get(void)
{
    return REG_RPL_RD(RPL_RPLLSDCNTL2_ADDR);
}

__INLINE void rpl_rpllsdcntl2_set(uint32_t value)
{
    REG_RPL_WR(RPL_RPLLSDCNTL2_ADDR, value);
}

// field definitions
#define RPL_BTLE_DUMO_EN_BIT           ((uint32_t)0x80000000)
#define RPL_BTLE_DUMO_EN_POS           31
#define RPL_BTLE_FILTER_SCALING_MASK   ((uint32_t)0x00FF0000)
#define RPL_BTLE_FILTER_SCALING_LSB    16
#define RPL_BTLE_FILTER_SCALING_WIDTH  ((uint32_t)0x00000008)
#define RPL_FILTER_SCALING_MASK        ((uint32_t)0x0000FF00)
#define RPL_FILTER_SCALING_LSB         8
#define RPL_FILTER_SCALING_WIDTH       ((uint32_t)0x00000008)
#define RPL_FMTX2P_EN_BIT              ((uint32_t)0x00000020)
#define RPL_FMTX2P_EN_POS              5
#define RPL_FMTX_EN_BIT                ((uint32_t)0x00000010)
#define RPL_FMTX_EN_POS                4
#define RPL_SDDITHER_EN_BIT            ((uint32_t)0x00000008)
#define RPL_SDDITHER_EN_POS            3
#define RPL_RPLL_SDORDER_MASK          ((uint32_t)0x00000006)
#define RPL_RPLL_SDORDER_LSB           1
#define RPL_RPLL_SDORDER_WIDTH         ((uint32_t)0x00000002)
#define RPL_RPLL_FRAC_EN_BIT           ((uint32_t)0x00000001)
#define RPL_RPLL_FRAC_EN_POS           0

#define RPL_BTLE_DUMO_EN_RST           0x1
#define RPL_BTLE_FILTER_SCALING_RST    0x2F
#define RPL_FILTER_SCALING_RST         0x47
#define RPL_FMTX2P_EN_RST              0x0
#define RPL_FMTX_EN_RST                0x1
#define RPL_SDDITHER_EN_RST            0x1
#define RPL_RPLL_SDORDER_RST           0x3
#define RPL_RPLL_FRAC_EN_RST           0x1

__INLINE void rpl_rpllsdcntl2_pack(uint8_t btledumoen, uint8_t btlefilterscaling, uint8_t filterscaling, uint8_t fmtx2pen, uint8_t fmtxen, uint8_t sdditheren, uint8_t rpllsdorder, uint8_t rpllfracen)
{
    ASSERT_ERR((((uint32_t)btledumoen << 31) & ~((uint32_t)0x80000000)) == 0);
    ASSERT_ERR((((uint32_t)btlefilterscaling << 16) & ~((uint32_t)0x00FF0000)) == 0);
    ASSERT_ERR((((uint32_t)filterscaling << 8) & ~((uint32_t)0x0000FF00)) == 0);
    ASSERT_ERR((((uint32_t)fmtx2pen << 5) & ~((uint32_t)0x00000020)) == 0);
    ASSERT_ERR((((uint32_t)fmtxen << 4) & ~((uint32_t)0x00000010)) == 0);
    ASSERT_ERR((((uint32_t)sdditheren << 3) & ~((uint32_t)0x00000008)) == 0);
    ASSERT_ERR((((uint32_t)rpllsdorder << 1) & ~((uint32_t)0x00000006)) == 0);
    ASSERT_ERR((((uint32_t)rpllfracen << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_RPL_WR(RPL_RPLLSDCNTL2_ADDR, ((uint32_t)btledumoen << 31) | ((uint32_t)btlefilterscaling << 16) | ((uint32_t)filterscaling << 8) | ((uint32_t)fmtx2pen << 5) | ((uint32_t)fmtxen << 4) | ((uint32_t)sdditheren << 3) | ((uint32_t)rpllsdorder << 1) | ((uint32_t)rpllfracen << 0));
}

__INLINE void rpl_rpllsdcntl2_unpack(uint8_t *btledumoen, uint8_t *btlefilterscaling, uint8_t *filterscaling, uint8_t *fmtx2pen, uint8_t *fmtxen, uint8_t *sdditheren, uint8_t *rpllsdorder, uint8_t *rpllfracen)
{
    uint32_t localVal = REG_RPL_RD(RPL_RPLLSDCNTL2_ADDR);

    *btledumoen = (localVal & ((uint32_t)0x80000000)) >> 31;
    *btlefilterscaling = (localVal & ((uint32_t)0x00FF0000)) >> 16;
    *filterscaling = (localVal & ((uint32_t)0x0000FF00)) >> 8;
    *fmtx2pen = (localVal & ((uint32_t)0x00000020)) >> 5;
    *fmtxen = (localVal & ((uint32_t)0x00000010)) >> 4;
    *sdditheren = (localVal & ((uint32_t)0x00000008)) >> 3;
    *rpllsdorder = (localVal & ((uint32_t)0x00000006)) >> 1;
    *rpllfracen = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t rpl_btle_dumo_en_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RPLLSDCNTL2_ADDR);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE void rpl_btle_dumo_en_setf(uint8_t btledumoen)
{
    ASSERT_ERR((((uint32_t)btledumoen << 31) & ~((uint32_t)0x80000000)) == 0);
    REG_RPL_WR(RPL_RPLLSDCNTL2_ADDR, (REG_RPL_RD(RPL_RPLLSDCNTL2_ADDR) & ~((uint32_t)0x80000000)) | ((uint32_t)btledumoen << 31));
}

__INLINE uint8_t rpl_btle_filter_scaling_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RPLLSDCNTL2_ADDR);
    return ((localVal & ((uint32_t)0x00FF0000)) >> 16);
}

__INLINE void rpl_btle_filter_scaling_setf(uint8_t btlefilterscaling)
{
    ASSERT_ERR((((uint32_t)btlefilterscaling << 16) & ~((uint32_t)0x00FF0000)) == 0);
    REG_RPL_WR(RPL_RPLLSDCNTL2_ADDR, (REG_RPL_RD(RPL_RPLLSDCNTL2_ADDR) & ~((uint32_t)0x00FF0000)) | ((uint32_t)btlefilterscaling << 16));
}

__INLINE uint8_t rpl_filter_scaling_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RPLLSDCNTL2_ADDR);
    return ((localVal & ((uint32_t)0x0000FF00)) >> 8);
}

__INLINE void rpl_filter_scaling_setf(uint8_t filterscaling)
{
    ASSERT_ERR((((uint32_t)filterscaling << 8) & ~((uint32_t)0x0000FF00)) == 0);
    REG_RPL_WR(RPL_RPLLSDCNTL2_ADDR, (REG_RPL_RD(RPL_RPLLSDCNTL2_ADDR) & ~((uint32_t)0x0000FF00)) | ((uint32_t)filterscaling << 8));
}

__INLINE uint8_t rpl_fmtx2p_en_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RPLLSDCNTL2_ADDR);
    return ((localVal & ((uint32_t)0x00000020)) >> 5);
}

__INLINE void rpl_fmtx2p_en_setf(uint8_t fmtx2pen)
{
    ASSERT_ERR((((uint32_t)fmtx2pen << 5) & ~((uint32_t)0x00000020)) == 0);
    REG_RPL_WR(RPL_RPLLSDCNTL2_ADDR, (REG_RPL_RD(RPL_RPLLSDCNTL2_ADDR) & ~((uint32_t)0x00000020)) | ((uint32_t)fmtx2pen << 5));
}

__INLINE uint8_t rpl_fmtx_en_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RPLLSDCNTL2_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE void rpl_fmtx_en_setf(uint8_t fmtxen)
{
    ASSERT_ERR((((uint32_t)fmtxen << 4) & ~((uint32_t)0x00000010)) == 0);
    REG_RPL_WR(RPL_RPLLSDCNTL2_ADDR, (REG_RPL_RD(RPL_RPLLSDCNTL2_ADDR) & ~((uint32_t)0x00000010)) | ((uint32_t)fmtxen << 4));
}

__INLINE uint8_t rpl_sddither_en_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RPLLSDCNTL2_ADDR);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

__INLINE void rpl_sddither_en_setf(uint8_t sdditheren)
{
    ASSERT_ERR((((uint32_t)sdditheren << 3) & ~((uint32_t)0x00000008)) == 0);
    REG_RPL_WR(RPL_RPLLSDCNTL2_ADDR, (REG_RPL_RD(RPL_RPLLSDCNTL2_ADDR) & ~((uint32_t)0x00000008)) | ((uint32_t)sdditheren << 3));
}

__INLINE uint8_t rpl_rpll_sdorder_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RPLLSDCNTL2_ADDR);
    return ((localVal & ((uint32_t)0x00000006)) >> 1);
}

__INLINE void rpl_rpll_sdorder_setf(uint8_t rpllsdorder)
{
    ASSERT_ERR((((uint32_t)rpllsdorder << 1) & ~((uint32_t)0x00000006)) == 0);
    REG_RPL_WR(RPL_RPLLSDCNTL2_ADDR, (REG_RPL_RD(RPL_RPLLSDCNTL2_ADDR) & ~((uint32_t)0x00000006)) | ((uint32_t)rpllsdorder << 1));
}

__INLINE uint8_t rpl_rpll_frac_en_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RPLLSDCNTL2_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void rpl_rpll_frac_en_setf(uint8_t rpllfracen)
{
    ASSERT_ERR((((uint32_t)rpllfracen << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_RPL_WR(RPL_RPLLSDCNTL2_ADDR, (REG_RPL_RD(RPL_RPLLSDCNTL2_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)rpllfracen << 0));
}

/**
 * @brief RPLLSDSTAT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:08          RPLL_NFRACT   0x0
 *  07:00            RPLL_NINT   0x0
 * </pre>
 */
#define RPL_RPLLSDSTAT_ADDR   0x00000023
#define RPL_RPLLSDSTAT_OFFSET 0x00000023
#define RPL_RPLLSDSTAT_INDEX  0x00000023
#define RPL_RPLLSDSTAT_RESET  0x00000000

__INLINE uint32_t rpl_rpllsdstat_get(void)
{
    return REG_RPL_RD(RPL_RPLLSDSTAT_ADDR);
}

// field definitions
#define RPL_RPLL_NFRACT_MASK   ((uint32_t)0xFFFFFF00)
#define RPL_RPLL_NFRACT_LSB    8
#define RPL_RPLL_NFRACT_WIDTH  ((uint32_t)0x00000018)
#define RPL_RPLL_NINT_MASK     ((uint32_t)0x000000FF)
#define RPL_RPLL_NINT_LSB      0
#define RPL_RPLL_NINT_WIDTH    ((uint32_t)0x00000008)

#define RPL_RPLL_NFRACT_RST    0x0
#define RPL_RPLL_NINT_RST      0x0

__INLINE void rpl_rpllsdstat_unpack(uint32_t *rpllnfract, uint8_t *rpllnint)
{
    uint32_t localVal = REG_RPL_RD(RPL_RPLLSDSTAT_ADDR);

    *rpllnfract = (localVal & ((uint32_t)0xFFFFFF00)) >> 8;
    *rpllnint = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint32_t rpl_rpll_nfract_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RPLLSDSTAT_ADDR);
    return ((localVal & ((uint32_t)0xFFFFFF00)) >> 8);
}

__INLINE uint8_t rpl_rpll_nint_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RPLLSDSTAT_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

/**
 * @brief RPLLMASH_INIT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:24            MASH_SDI4   0xD
 *  23:16            MASH_SDI3   0xB
 *  15:08            MASH_SDI2   0x13
 *  07:00            MASH_SDI1   0x11
 * </pre>
 */
#define RPL_RPLLMASH_INIT_ADDR   0x00000024
#define RPL_RPLLMASH_INIT_OFFSET 0x00000024
#define RPL_RPLLMASH_INIT_INDEX  0x00000024
#define RPL_RPLLMASH_INIT_RESET  0x0D0B1311

__INLINE uint32_t rpl_rpllmash_init_get(void)
{
    return REG_RPL_RD(RPL_RPLLMASH_INIT_ADDR);
}

__INLINE void rpl_rpllmash_init_set(uint32_t value)
{
    REG_RPL_WR(RPL_RPLLMASH_INIT_ADDR, value);
}

// field definitions
#define RPL_MASH_SDI4_MASK   ((uint32_t)0xFF000000)
#define RPL_MASH_SDI4_LSB    24
#define RPL_MASH_SDI4_WIDTH  ((uint32_t)0x00000008)
#define RPL_MASH_SDI3_MASK   ((uint32_t)0x00FF0000)
#define RPL_MASH_SDI3_LSB    16
#define RPL_MASH_SDI3_WIDTH  ((uint32_t)0x00000008)
#define RPL_MASH_SDI2_MASK   ((uint32_t)0x0000FF00)
#define RPL_MASH_SDI2_LSB    8
#define RPL_MASH_SDI2_WIDTH  ((uint32_t)0x00000008)
#define RPL_MASH_SDI1_MASK   ((uint32_t)0x000000FF)
#define RPL_MASH_SDI1_LSB    0
#define RPL_MASH_SDI1_WIDTH  ((uint32_t)0x00000008)

#define RPL_MASH_SDI4_RST    0xD
#define RPL_MASH_SDI3_RST    0xB
#define RPL_MASH_SDI2_RST    0x13
#define RPL_MASH_SDI1_RST    0x11

__INLINE void rpl_rpllmash_init_pack(uint8_t mashsdi4, uint8_t mashsdi3, uint8_t mashsdi2, uint8_t mashsdi1)
{
    ASSERT_ERR((((uint32_t)mashsdi4 << 24) & ~((uint32_t)0xFF000000)) == 0);
    ASSERT_ERR((((uint32_t)mashsdi3 << 16) & ~((uint32_t)0x00FF0000)) == 0);
    ASSERT_ERR((((uint32_t)mashsdi2 << 8) & ~((uint32_t)0x0000FF00)) == 0);
    ASSERT_ERR((((uint32_t)mashsdi1 << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_RPL_WR(RPL_RPLLMASH_INIT_ADDR, ((uint32_t)mashsdi4 << 24) | ((uint32_t)mashsdi3 << 16) | ((uint32_t)mashsdi2 << 8) | ((uint32_t)mashsdi1 << 0));
}

__INLINE void rpl_rpllmash_init_unpack(uint8_t *mashsdi4, uint8_t *mashsdi3, uint8_t *mashsdi2, uint8_t *mashsdi1)
{
    uint32_t localVal = REG_RPL_RD(RPL_RPLLMASH_INIT_ADDR);

    *mashsdi4 = (localVal & ((uint32_t)0xFF000000)) >> 24;
    *mashsdi3 = (localVal & ((uint32_t)0x00FF0000)) >> 16;
    *mashsdi2 = (localVal & ((uint32_t)0x0000FF00)) >> 8;
    *mashsdi1 = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint8_t rpl_mash_sdi4_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RPLLMASH_INIT_ADDR);
    return ((localVal & ((uint32_t)0xFF000000)) >> 24);
}

__INLINE void rpl_mash_sdi4_setf(uint8_t mashsdi4)
{
    ASSERT_ERR((((uint32_t)mashsdi4 << 24) & ~((uint32_t)0xFF000000)) == 0);
    REG_RPL_WR(RPL_RPLLMASH_INIT_ADDR, (REG_RPL_RD(RPL_RPLLMASH_INIT_ADDR) & ~((uint32_t)0xFF000000)) | ((uint32_t)mashsdi4 << 24));
}

__INLINE uint8_t rpl_mash_sdi3_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RPLLMASH_INIT_ADDR);
    return ((localVal & ((uint32_t)0x00FF0000)) >> 16);
}

__INLINE void rpl_mash_sdi3_setf(uint8_t mashsdi3)
{
    ASSERT_ERR((((uint32_t)mashsdi3 << 16) & ~((uint32_t)0x00FF0000)) == 0);
    REG_RPL_WR(RPL_RPLLMASH_INIT_ADDR, (REG_RPL_RD(RPL_RPLLMASH_INIT_ADDR) & ~((uint32_t)0x00FF0000)) | ((uint32_t)mashsdi3 << 16));
}

__INLINE uint8_t rpl_mash_sdi2_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RPLLMASH_INIT_ADDR);
    return ((localVal & ((uint32_t)0x0000FF00)) >> 8);
}

__INLINE void rpl_mash_sdi2_setf(uint8_t mashsdi2)
{
    ASSERT_ERR((((uint32_t)mashsdi2 << 8) & ~((uint32_t)0x0000FF00)) == 0);
    REG_RPL_WR(RPL_RPLLMASH_INIT_ADDR, (REG_RPL_RD(RPL_RPLLMASH_INIT_ADDR) & ~((uint32_t)0x0000FF00)) | ((uint32_t)mashsdi2 << 8));
}

__INLINE uint8_t rpl_mash_sdi1_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RPLLMASH_INIT_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

__INLINE void rpl_mash_sdi1_setf(uint8_t mashsdi1)
{
    ASSERT_ERR((((uint32_t)mashsdi1 << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_RPL_WR(RPL_RPLLMASH_INIT_ADDR, (REG_RPL_RD(RPL_RPLLMASH_INIT_ADDR) & ~((uint32_t)0x000000FF)) | ((uint32_t)mashsdi1 << 0));
}

/**
 * @brief GFSK_FILT_COEFF register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00           FILT_COEFF   0x543C1702
 * </pre>
 */
#define RPL_GFSK_FILT_COEFF_ADDR   0x00000025
#define RPL_GFSK_FILT_COEFF_OFFSET 0x00000025
#define RPL_GFSK_FILT_COEFF_INDEX  0x00000025
#define RPL_GFSK_FILT_COEFF_RESET  0x543C1702
#define RPL_GFSK_FILT_COEFF_COUNT  13

__INLINE uint32_t rpl_gfsk_filt_coeff_get(int reg_idx)
{
    ASSERT_ERR(reg_idx <= 12);
    return REG_RPL_RD(RPL_GFSK_FILT_COEFF_ADDR + reg_idx * 1);
}

__INLINE void rpl_gfsk_filt_coeff_set(int reg_idx, uint32_t value)
{
    ASSERT_ERR(reg_idx <= 12);
    REG_RPL_WR(RPL_GFSK_FILT_COEFF_ADDR + reg_idx * 1, value);
}

// field definitions
#define RPL_FILT_COEFF_MASK   ((uint32_t)0xFFFFFFFF)
#define RPL_FILT_COEFF_LSB    0
#define RPL_FILT_COEFF_WIDTH  ((uint32_t)0x00000020)

#define RPL_FILT_COEFF_RST    0x543C1702

__INLINE uint32_t rpl_filt_coeff_getf(int reg_idx)
{
    ASSERT_ERR(reg_idx <= 12);
    uint32_t localVal = REG_RPL_RD(RPL_GFSK_FILT_COEFF_ADDR + reg_idx * 1);
    ASSERT_ERR((localVal & ~((uint32_t)0xFFFFFFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void rpl_filt_coeff_setf(int reg_idx, uint32_t filtcoeff)
{
    ASSERT_ERR(reg_idx <= 12);
    ASSERT_ERR((((uint32_t)filtcoeff << 0) & ~((uint32_t)0xFFFFFFFF)) == 0);
    REG_RPL_WR(RPL_GFSK_FILT_COEFF_ADDR + reg_idx * 1, (uint32_t)filtcoeff << 0);
}

/**
 * @brief DPLLCNTL0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     31      FORCE_DPLL_CTRL   0
 *     03           DPLL_LD_ON   0
 *     02          DPLL_VCO_ON   0
 *     01           DPLL_CP_ON   0
 * </pre>
 */
#define RPL_DPLLCNTL0_ADDR   0x00000032
#define RPL_DPLLCNTL0_OFFSET 0x00000032
#define RPL_DPLLCNTL0_INDEX  0x00000032
#define RPL_DPLLCNTL0_RESET  0x00000000

__INLINE uint32_t rpl_dpllcntl0_get(void)
{
    return REG_RPL_RD(RPL_DPLLCNTL0_ADDR);
}

__INLINE void rpl_dpllcntl0_set(uint32_t value)
{
    REG_RPL_WR(RPL_DPLLCNTL0_ADDR, value);
}

// field definitions
#define RPL_FORCE_DPLL_CTRL_BIT    ((uint32_t)0x80000000)
#define RPL_FORCE_DPLL_CTRL_POS    31
#define RPL_DPLL_LD_ON_BIT         ((uint32_t)0x00000008)
#define RPL_DPLL_LD_ON_POS         3
#define RPL_DPLL_VCO_ON_BIT        ((uint32_t)0x00000004)
#define RPL_DPLL_VCO_ON_POS        2
#define RPL_DPLL_CP_ON_BIT         ((uint32_t)0x00000002)
#define RPL_DPLL_CP_ON_POS         1

#define RPL_FORCE_DPLL_CTRL_RST    0x0
#define RPL_DPLL_LD_ON_RST         0x0
#define RPL_DPLL_VCO_ON_RST        0x0
#define RPL_DPLL_CP_ON_RST         0x0

__INLINE void rpl_dpllcntl0_pack(uint8_t forcedpllctrl, uint8_t dpllldon, uint8_t dpllvcoon, uint8_t dpllcpon)
{
    ASSERT_ERR((((uint32_t)forcedpllctrl << 31) & ~((uint32_t)0x80000000)) == 0);
    ASSERT_ERR((((uint32_t)dpllldon << 3) & ~((uint32_t)0x00000008)) == 0);
    ASSERT_ERR((((uint32_t)dpllvcoon << 2) & ~((uint32_t)0x00000004)) == 0);
    ASSERT_ERR((((uint32_t)dpllcpon << 1) & ~((uint32_t)0x00000002)) == 0);
    REG_RPL_WR(RPL_DPLLCNTL0_ADDR, ((uint32_t)forcedpllctrl << 31) | ((uint32_t)dpllldon << 3) | ((uint32_t)dpllvcoon << 2) | ((uint32_t)dpllcpon << 1));
}

__INLINE void rpl_dpllcntl0_unpack(uint8_t *forcedpllctrl, uint8_t *dpllldon, uint8_t *dpllvcoon, uint8_t *dpllcpon)
{
    uint32_t localVal = REG_RPL_RD(RPL_DPLLCNTL0_ADDR);

    *forcedpllctrl = (localVal & ((uint32_t)0x80000000)) >> 31;
    *dpllldon = (localVal & ((uint32_t)0x00000008)) >> 3;
    *dpllvcoon = (localVal & ((uint32_t)0x00000004)) >> 2;
    *dpllcpon = (localVal & ((uint32_t)0x00000002)) >> 1;
}

__INLINE uint8_t rpl_force_dpll_ctrl_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_DPLLCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE void rpl_force_dpll_ctrl_setf(uint8_t forcedpllctrl)
{
    ASSERT_ERR((((uint32_t)forcedpllctrl << 31) & ~((uint32_t)0x80000000)) == 0);
    REG_RPL_WR(RPL_DPLLCNTL0_ADDR, (REG_RPL_RD(RPL_DPLLCNTL0_ADDR) & ~((uint32_t)0x80000000)) | ((uint32_t)forcedpllctrl << 31));
}

__INLINE uint8_t rpl_dpll_ld_on_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_DPLLCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

__INLINE void rpl_dpll_ld_on_setf(uint8_t dpllldon)
{
    ASSERT_ERR((((uint32_t)dpllldon << 3) & ~((uint32_t)0x00000008)) == 0);
    REG_RPL_WR(RPL_DPLLCNTL0_ADDR, (REG_RPL_RD(RPL_DPLLCNTL0_ADDR) & ~((uint32_t)0x00000008)) | ((uint32_t)dpllldon << 3));
}

__INLINE uint8_t rpl_dpll_vco_on_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_DPLLCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE void rpl_dpll_vco_on_setf(uint8_t dpllvcoon)
{
    ASSERT_ERR((((uint32_t)dpllvcoon << 2) & ~((uint32_t)0x00000004)) == 0);
    REG_RPL_WR(RPL_DPLLCNTL0_ADDR, (REG_RPL_RD(RPL_DPLLCNTL0_ADDR) & ~((uint32_t)0x00000004)) | ((uint32_t)dpllvcoon << 2));
}

__INLINE uint8_t rpl_dpll_cp_on_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_DPLLCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE void rpl_dpll_cp_on_setf(uint8_t dpllcpon)
{
    ASSERT_ERR((((uint32_t)dpllcpon << 1) & ~((uint32_t)0x00000002)) == 0);
    REG_RPL_WR(RPL_DPLLCNTL0_ADDR, (REG_RPL_RD(RPL_DPLLCNTL0_ADDR) & ~((uint32_t)0x00000002)) | ((uint32_t)dpllcpon << 1));
}

/**
 * @brief DPLLCNTL1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  29:28            DPLL_DIVR   0x0
 *  26:24            DPLL_DIVP   0x0
 *  23:08          DPLL_NFRACT   0x0
 *  04:00            DPLL_NINT   0x0
 * </pre>
 */
#define RPL_DPLLCNTL1_ADDR   0x00000033
#define RPL_DPLLCNTL1_OFFSET 0x00000033
#define RPL_DPLLCNTL1_INDEX  0x00000033
#define RPL_DPLLCNTL1_RESET  0x00000000

__INLINE uint32_t rpl_dpllcntl1_get(void)
{
    return REG_RPL_RD(RPL_DPLLCNTL1_ADDR);
}

__INLINE void rpl_dpllcntl1_set(uint32_t value)
{
    REG_RPL_WR(RPL_DPLLCNTL1_ADDR, value);
}

// field definitions
#define RPL_DPLL_DIVR_MASK     ((uint32_t)0x30000000)
#define RPL_DPLL_DIVR_LSB      28
#define RPL_DPLL_DIVR_WIDTH    ((uint32_t)0x00000002)
#define RPL_DPLL_DIVP_MASK     ((uint32_t)0x07000000)
#define RPL_DPLL_DIVP_LSB      24
#define RPL_DPLL_DIVP_WIDTH    ((uint32_t)0x00000003)
#define RPL_DPLL_NFRACT_MASK   ((uint32_t)0x00FFFF00)
#define RPL_DPLL_NFRACT_LSB    8
#define RPL_DPLL_NFRACT_WIDTH  ((uint32_t)0x00000010)
#define RPL_DPLL_NINT_MASK     ((uint32_t)0x0000001F)
#define RPL_DPLL_NINT_LSB      0
#define RPL_DPLL_NINT_WIDTH    ((uint32_t)0x00000005)

#define RPL_DPLL_DIVR_RST      0x0
#define RPL_DPLL_DIVP_RST      0x0
#define RPL_DPLL_NFRACT_RST    0x0
#define RPL_DPLL_NINT_RST      0x0

__INLINE void rpl_dpllcntl1_pack(uint8_t dplldivr, uint8_t dplldivp, uint16_t dpllnfract, uint8_t dpllnint)
{
    ASSERT_ERR((((uint32_t)dplldivr << 28) & ~((uint32_t)0x30000000)) == 0);
    ASSERT_ERR((((uint32_t)dplldivp << 24) & ~((uint32_t)0x07000000)) == 0);
    ASSERT_ERR((((uint32_t)dpllnfract << 8) & ~((uint32_t)0x00FFFF00)) == 0);
    ASSERT_ERR((((uint32_t)dpllnint << 0) & ~((uint32_t)0x0000001F)) == 0);
    REG_RPL_WR(RPL_DPLLCNTL1_ADDR, ((uint32_t)dplldivr << 28) | ((uint32_t)dplldivp << 24) | ((uint32_t)dpllnfract << 8) | ((uint32_t)dpllnint << 0));
}

__INLINE void rpl_dpllcntl1_unpack(uint8_t *dplldivr, uint8_t *dplldivp, uint16_t *dpllnfract, uint8_t *dpllnint)
{
    uint32_t localVal = REG_RPL_RD(RPL_DPLLCNTL1_ADDR);

    *dplldivr = (localVal & ((uint32_t)0x30000000)) >> 28;
    *dplldivp = (localVal & ((uint32_t)0x07000000)) >> 24;
    *dpllnfract = (localVal & ((uint32_t)0x00FFFF00)) >> 8;
    *dpllnint = (localVal & ((uint32_t)0x0000001F)) >> 0;
}

__INLINE uint8_t rpl_dpll_divr_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_DPLLCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x30000000)) >> 28);
}

__INLINE void rpl_dpll_divr_setf(uint8_t dplldivr)
{
    ASSERT_ERR((((uint32_t)dplldivr << 28) & ~((uint32_t)0x30000000)) == 0);
    REG_RPL_WR(RPL_DPLLCNTL1_ADDR, (REG_RPL_RD(RPL_DPLLCNTL1_ADDR) & ~((uint32_t)0x30000000)) | ((uint32_t)dplldivr << 28));
}

__INLINE uint8_t rpl_dpll_divp_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_DPLLCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x07000000)) >> 24);
}

__INLINE void rpl_dpll_divp_setf(uint8_t dplldivp)
{
    ASSERT_ERR((((uint32_t)dplldivp << 24) & ~((uint32_t)0x07000000)) == 0);
    REG_RPL_WR(RPL_DPLLCNTL1_ADDR, (REG_RPL_RD(RPL_DPLLCNTL1_ADDR) & ~((uint32_t)0x07000000)) | ((uint32_t)dplldivp << 24));
}

__INLINE uint16_t rpl_dpll_nfract_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_DPLLCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x00FFFF00)) >> 8);
}

__INLINE void rpl_dpll_nfract_setf(uint16_t dpllnfract)
{
    ASSERT_ERR((((uint32_t)dpllnfract << 8) & ~((uint32_t)0x00FFFF00)) == 0);
    REG_RPL_WR(RPL_DPLLCNTL1_ADDR, (REG_RPL_RD(RPL_DPLLCNTL1_ADDR) & ~((uint32_t)0x00FFFF00)) | ((uint32_t)dpllnfract << 8));
}

__INLINE uint8_t rpl_dpll_nint_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_DPLLCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x0000001F)) >> 0);
}

__INLINE void rpl_dpll_nint_setf(uint8_t dpllnint)
{
    ASSERT_ERR((((uint32_t)dpllnint << 0) & ~((uint32_t)0x0000001F)) == 0);
    REG_RPL_WR(RPL_DPLLCNTL1_ADDR, (REG_RPL_RD(RPL_DPLLCNTL1_ADDR) & ~((uint32_t)0x0000001F)) | ((uint32_t)dpllnint << 0));
}

/**
 * @brief DPLLCNTL2 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  17:16         DPLL_SDORDER   0x1
 *  14:12      DPLL_ICP_OFFSET   0x0
 *     11         DPLL_VCO_CFG   0
 *  10:08         DPLL_PFD_CFG   0x0
 *  07:04             DPLL_ICP   0x0
 *     03         DPLL_TEST_EN   0
 *     02   DPLL_VCO_SPEEDUP_EN   0
 *     01        DPLL_FRACT_EN   0
 *     00          DPLL_DTP_EN   0
 * </pre>
 */
#define RPL_DPLLCNTL2_ADDR   0x00000034
#define RPL_DPLLCNTL2_OFFSET 0x00000034
#define RPL_DPLLCNTL2_INDEX  0x00000034
#define RPL_DPLLCNTL2_RESET  0x00010000

__INLINE uint32_t rpl_dpllcntl2_get(void)
{
    return REG_RPL_RD(RPL_DPLLCNTL2_ADDR);
}

__INLINE void rpl_dpllcntl2_set(uint32_t value)
{
    REG_RPL_WR(RPL_DPLLCNTL2_ADDR, value);
}

// field definitions
#define RPL_DPLL_SDORDER_MASK          ((uint32_t)0x00030000)
#define RPL_DPLL_SDORDER_LSB           16
#define RPL_DPLL_SDORDER_WIDTH         ((uint32_t)0x00000002)
#define RPL_DPLL_ICP_OFFSET_MASK       ((uint32_t)0x00007000)
#define RPL_DPLL_ICP_OFFSET_LSB        12
#define RPL_DPLL_ICP_OFFSET_WIDTH      ((uint32_t)0x00000003)
#define RPL_DPLL_VCO_CFG_BIT           ((uint32_t)0x00000800)
#define RPL_DPLL_VCO_CFG_POS           11
#define RPL_DPLL_PFD_CFG_MASK          ((uint32_t)0x00000700)
#define RPL_DPLL_PFD_CFG_LSB           8
#define RPL_DPLL_PFD_CFG_WIDTH         ((uint32_t)0x00000003)
#define RPL_DPLL_ICP_MASK              ((uint32_t)0x000000F0)
#define RPL_DPLL_ICP_LSB               4
#define RPL_DPLL_ICP_WIDTH             ((uint32_t)0x00000004)
#define RPL_DPLL_TEST_EN_BIT           ((uint32_t)0x00000008)
#define RPL_DPLL_TEST_EN_POS           3
#define RPL_DPLL_VCO_SPEEDUP_EN_BIT    ((uint32_t)0x00000004)
#define RPL_DPLL_VCO_SPEEDUP_EN_POS    2
#define RPL_DPLL_FRACT_EN_BIT          ((uint32_t)0x00000002)
#define RPL_DPLL_FRACT_EN_POS          1
#define RPL_DPLL_DTP_EN_BIT            ((uint32_t)0x00000001)
#define RPL_DPLL_DTP_EN_POS            0

#define RPL_DPLL_SDORDER_RST           0x1
#define RPL_DPLL_ICP_OFFSET_RST        0x0
#define RPL_DPLL_VCO_CFG_RST           0x0
#define RPL_DPLL_PFD_CFG_RST           0x0
#define RPL_DPLL_ICP_RST               0x0
#define RPL_DPLL_TEST_EN_RST           0x0
#define RPL_DPLL_VCO_SPEEDUP_EN_RST    0x0
#define RPL_DPLL_FRACT_EN_RST          0x0
#define RPL_DPLL_DTP_EN_RST            0x0

__INLINE void rpl_dpllcntl2_pack(uint8_t dpllsdorder, uint8_t dpllicpoffset, uint8_t dpllvcocfg, uint8_t dpllpfdcfg, uint8_t dpllicp, uint8_t dplltesten, uint8_t dpllvcospeedupen, uint8_t dpllfracten, uint8_t dplldtpen)
{
    ASSERT_ERR((((uint32_t)dpllsdorder << 16) & ~((uint32_t)0x00030000)) == 0);
    ASSERT_ERR((((uint32_t)dpllicpoffset << 12) & ~((uint32_t)0x00007000)) == 0);
    ASSERT_ERR((((uint32_t)dpllvcocfg << 11) & ~((uint32_t)0x00000800)) == 0);
    ASSERT_ERR((((uint32_t)dpllpfdcfg << 8) & ~((uint32_t)0x00000700)) == 0);
    ASSERT_ERR((((uint32_t)dpllicp << 4) & ~((uint32_t)0x000000F0)) == 0);
    ASSERT_ERR((((uint32_t)dplltesten << 3) & ~((uint32_t)0x00000008)) == 0);
    ASSERT_ERR((((uint32_t)dpllvcospeedupen << 2) & ~((uint32_t)0x00000004)) == 0);
    ASSERT_ERR((((uint32_t)dpllfracten << 1) & ~((uint32_t)0x00000002)) == 0);
    ASSERT_ERR((((uint32_t)dplldtpen << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_RPL_WR(RPL_DPLLCNTL2_ADDR, ((uint32_t)dpllsdorder << 16) | ((uint32_t)dpllicpoffset << 12) | ((uint32_t)dpllvcocfg << 11) | ((uint32_t)dpllpfdcfg << 8) | ((uint32_t)dpllicp << 4) | ((uint32_t)dplltesten << 3) | ((uint32_t)dpllvcospeedupen << 2) | ((uint32_t)dpllfracten << 1) | ((uint32_t)dplldtpen << 0));
}

__INLINE void rpl_dpllcntl2_unpack(uint8_t *dpllsdorder, uint8_t *dpllicpoffset, uint8_t *dpllvcocfg, uint8_t *dpllpfdcfg, uint8_t *dpllicp, uint8_t *dplltesten, uint8_t *dpllvcospeedupen, uint8_t *dpllfracten, uint8_t *dplldtpen)
{
    uint32_t localVal = REG_RPL_RD(RPL_DPLLCNTL2_ADDR);

    *dpllsdorder = (localVal & ((uint32_t)0x00030000)) >> 16;
    *dpllicpoffset = (localVal & ((uint32_t)0x00007000)) >> 12;
    *dpllvcocfg = (localVal & ((uint32_t)0x00000800)) >> 11;
    *dpllpfdcfg = (localVal & ((uint32_t)0x00000700)) >> 8;
    *dpllicp = (localVal & ((uint32_t)0x000000F0)) >> 4;
    *dplltesten = (localVal & ((uint32_t)0x00000008)) >> 3;
    *dpllvcospeedupen = (localVal & ((uint32_t)0x00000004)) >> 2;
    *dpllfracten = (localVal & ((uint32_t)0x00000002)) >> 1;
    *dplldtpen = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t rpl_dpll_sdorder_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_DPLLCNTL2_ADDR);
    return ((localVal & ((uint32_t)0x00030000)) >> 16);
}

__INLINE void rpl_dpll_sdorder_setf(uint8_t dpllsdorder)
{
    ASSERT_ERR((((uint32_t)dpllsdorder << 16) & ~((uint32_t)0x00030000)) == 0);
    REG_RPL_WR(RPL_DPLLCNTL2_ADDR, (REG_RPL_RD(RPL_DPLLCNTL2_ADDR) & ~((uint32_t)0x00030000)) | ((uint32_t)dpllsdorder << 16));
}

__INLINE uint8_t rpl_dpll_icp_offset_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_DPLLCNTL2_ADDR);
    return ((localVal & ((uint32_t)0x00007000)) >> 12);
}

__INLINE void rpl_dpll_icp_offset_setf(uint8_t dpllicpoffset)
{
    ASSERT_ERR((((uint32_t)dpllicpoffset << 12) & ~((uint32_t)0x00007000)) == 0);
    REG_RPL_WR(RPL_DPLLCNTL2_ADDR, (REG_RPL_RD(RPL_DPLLCNTL2_ADDR) & ~((uint32_t)0x00007000)) | ((uint32_t)dpllicpoffset << 12));
}

__INLINE uint8_t rpl_dpll_vco_cfg_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_DPLLCNTL2_ADDR);
    return ((localVal & ((uint32_t)0x00000800)) >> 11);
}

__INLINE void rpl_dpll_vco_cfg_setf(uint8_t dpllvcocfg)
{
    ASSERT_ERR((((uint32_t)dpllvcocfg << 11) & ~((uint32_t)0x00000800)) == 0);
    REG_RPL_WR(RPL_DPLLCNTL2_ADDR, (REG_RPL_RD(RPL_DPLLCNTL2_ADDR) & ~((uint32_t)0x00000800)) | ((uint32_t)dpllvcocfg << 11));
}

__INLINE uint8_t rpl_dpll_pfd_cfg_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_DPLLCNTL2_ADDR);
    return ((localVal & ((uint32_t)0x00000700)) >> 8);
}

__INLINE void rpl_dpll_pfd_cfg_setf(uint8_t dpllpfdcfg)
{
    ASSERT_ERR((((uint32_t)dpllpfdcfg << 8) & ~((uint32_t)0x00000700)) == 0);
    REG_RPL_WR(RPL_DPLLCNTL2_ADDR, (REG_RPL_RD(RPL_DPLLCNTL2_ADDR) & ~((uint32_t)0x00000700)) | ((uint32_t)dpllpfdcfg << 8));
}

__INLINE uint8_t rpl_dpll_icp_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_DPLLCNTL2_ADDR);
    return ((localVal & ((uint32_t)0x000000F0)) >> 4);
}

__INLINE void rpl_dpll_icp_setf(uint8_t dpllicp)
{
    ASSERT_ERR((((uint32_t)dpllicp << 4) & ~((uint32_t)0x000000F0)) == 0);
    REG_RPL_WR(RPL_DPLLCNTL2_ADDR, (REG_RPL_RD(RPL_DPLLCNTL2_ADDR) & ~((uint32_t)0x000000F0)) | ((uint32_t)dpllicp << 4));
}

__INLINE uint8_t rpl_dpll_test_en_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_DPLLCNTL2_ADDR);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

__INLINE void rpl_dpll_test_en_setf(uint8_t dplltesten)
{
    ASSERT_ERR((((uint32_t)dplltesten << 3) & ~((uint32_t)0x00000008)) == 0);
    REG_RPL_WR(RPL_DPLLCNTL2_ADDR, (REG_RPL_RD(RPL_DPLLCNTL2_ADDR) & ~((uint32_t)0x00000008)) | ((uint32_t)dplltesten << 3));
}

__INLINE uint8_t rpl_dpll_vco_speedup_en_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_DPLLCNTL2_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE void rpl_dpll_vco_speedup_en_setf(uint8_t dpllvcospeedupen)
{
    ASSERT_ERR((((uint32_t)dpllvcospeedupen << 2) & ~((uint32_t)0x00000004)) == 0);
    REG_RPL_WR(RPL_DPLLCNTL2_ADDR, (REG_RPL_RD(RPL_DPLLCNTL2_ADDR) & ~((uint32_t)0x00000004)) | ((uint32_t)dpllvcospeedupen << 2));
}

__INLINE uint8_t rpl_dpll_fract_en_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_DPLLCNTL2_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE void rpl_dpll_fract_en_setf(uint8_t dpllfracten)
{
    ASSERT_ERR((((uint32_t)dpllfracten << 1) & ~((uint32_t)0x00000002)) == 0);
    REG_RPL_WR(RPL_DPLLCNTL2_ADDR, (REG_RPL_RD(RPL_DPLLCNTL2_ADDR) & ~((uint32_t)0x00000002)) | ((uint32_t)dpllfracten << 1));
}

__INLINE uint8_t rpl_dpll_dtp_en_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_DPLLCNTL2_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void rpl_dpll_dtp_en_setf(uint8_t dplldtpen)
{
    ASSERT_ERR((((uint32_t)dplldtpen << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_RPL_WR(RPL_DPLLCNTL2_ADDR, (REG_RPL_RD(RPL_DPLLCNTL2_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)dplldtpen << 0));
}

/**
 * @brief DPLLCNTL3 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     21   DPLL_LD_FREF_CLK_INV_EN   0
 *     20   DPLL_LD_FDIV_CLK_INV_EN   0
 *  19:18   DPLL_LD_EDGES_COMB   0x2
 *     17           DPLL_LD_FE   0
 *     16           DPLL_LD_RE   0
 *  15:08   DPLL_LD_UNLOCK_CNT   0x5
 *  07:00     DPLL_LD_LOCK_CNT   0xFF
 * </pre>
 */
#define RPL_DPLLCNTL3_ADDR   0x00000035
#define RPL_DPLLCNTL3_OFFSET 0x00000035
#define RPL_DPLLCNTL3_INDEX  0x00000035
#define RPL_DPLLCNTL3_RESET  0x000805FF

__INLINE uint32_t rpl_dpllcntl3_get(void)
{
    return REG_RPL_RD(RPL_DPLLCNTL3_ADDR);
}

__INLINE void rpl_dpllcntl3_set(uint32_t value)
{
    REG_RPL_WR(RPL_DPLLCNTL3_ADDR, value);
}

// field definitions
#define RPL_DPLL_LD_FREF_CLK_INV_EN_BIT    ((uint32_t)0x00200000)
#define RPL_DPLL_LD_FREF_CLK_INV_EN_POS    21
#define RPL_DPLL_LD_FDIV_CLK_INV_EN_BIT    ((uint32_t)0x00100000)
#define RPL_DPLL_LD_FDIV_CLK_INV_EN_POS    20
#define RPL_DPLL_LD_EDGES_COMB_MASK        ((uint32_t)0x000C0000)
#define RPL_DPLL_LD_EDGES_COMB_LSB         18
#define RPL_DPLL_LD_EDGES_COMB_WIDTH       ((uint32_t)0x00000002)
#define RPL_DPLL_LD_FE_BIT                 ((uint32_t)0x00020000)
#define RPL_DPLL_LD_FE_POS                 17
#define RPL_DPLL_LD_RE_BIT                 ((uint32_t)0x00010000)
#define RPL_DPLL_LD_RE_POS                 16
#define RPL_DPLL_LD_UNLOCK_CNT_MASK        ((uint32_t)0x0000FF00)
#define RPL_DPLL_LD_UNLOCK_CNT_LSB         8
#define RPL_DPLL_LD_UNLOCK_CNT_WIDTH       ((uint32_t)0x00000008)
#define RPL_DPLL_LD_LOCK_CNT_MASK          ((uint32_t)0x000000FF)
#define RPL_DPLL_LD_LOCK_CNT_LSB           0
#define RPL_DPLL_LD_LOCK_CNT_WIDTH         ((uint32_t)0x00000008)

#define RPL_DPLL_LD_FREF_CLK_INV_EN_RST    0x0
#define RPL_DPLL_LD_FDIV_CLK_INV_EN_RST    0x0
#define RPL_DPLL_LD_EDGES_COMB_RST         0x2
#define RPL_DPLL_LD_FE_RST                 0x0
#define RPL_DPLL_LD_RE_RST                 0x0
#define RPL_DPLL_LD_UNLOCK_CNT_RST         0x5
#define RPL_DPLL_LD_LOCK_CNT_RST           0xFF

__INLINE void rpl_dpllcntl3_pack(uint8_t dpllldfrefclkinven, uint8_t dpllldfdivclkinven, uint8_t dpllldedgescomb, uint8_t dpllldfe, uint8_t dpllldre, uint8_t dpllldunlockcnt, uint8_t dpllldlockcnt)
{
    ASSERT_ERR((((uint32_t)dpllldfrefclkinven << 21) & ~((uint32_t)0x00200000)) == 0);
    ASSERT_ERR((((uint32_t)dpllldfdivclkinven << 20) & ~((uint32_t)0x00100000)) == 0);
    ASSERT_ERR((((uint32_t)dpllldedgescomb << 18) & ~((uint32_t)0x000C0000)) == 0);
    ASSERT_ERR((((uint32_t)dpllldfe << 17) & ~((uint32_t)0x00020000)) == 0);
    ASSERT_ERR((((uint32_t)dpllldre << 16) & ~((uint32_t)0x00010000)) == 0);
    ASSERT_ERR((((uint32_t)dpllldunlockcnt << 8) & ~((uint32_t)0x0000FF00)) == 0);
    ASSERT_ERR((((uint32_t)dpllldlockcnt << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_RPL_WR(RPL_DPLLCNTL3_ADDR, ((uint32_t)dpllldfrefclkinven << 21) | ((uint32_t)dpllldfdivclkinven << 20) | ((uint32_t)dpllldedgescomb << 18) | ((uint32_t)dpllldfe << 17) | ((uint32_t)dpllldre << 16) | ((uint32_t)dpllldunlockcnt << 8) | ((uint32_t)dpllldlockcnt << 0));
}

__INLINE void rpl_dpllcntl3_unpack(uint8_t *dpllldfrefclkinven, uint8_t *dpllldfdivclkinven, uint8_t *dpllldedgescomb, uint8_t *dpllldfe, uint8_t *dpllldre, uint8_t *dpllldunlockcnt, uint8_t *dpllldlockcnt)
{
    uint32_t localVal = REG_RPL_RD(RPL_DPLLCNTL3_ADDR);

    *dpllldfrefclkinven = (localVal & ((uint32_t)0x00200000)) >> 21;
    *dpllldfdivclkinven = (localVal & ((uint32_t)0x00100000)) >> 20;
    *dpllldedgescomb = (localVal & ((uint32_t)0x000C0000)) >> 18;
    *dpllldfe = (localVal & ((uint32_t)0x00020000)) >> 17;
    *dpllldre = (localVal & ((uint32_t)0x00010000)) >> 16;
    *dpllldunlockcnt = (localVal & ((uint32_t)0x0000FF00)) >> 8;
    *dpllldlockcnt = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint8_t rpl_dpll_ld_fref_clk_inv_en_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_DPLLCNTL3_ADDR);
    return ((localVal & ((uint32_t)0x00200000)) >> 21);
}

__INLINE void rpl_dpll_ld_fref_clk_inv_en_setf(uint8_t dpllldfrefclkinven)
{
    ASSERT_ERR((((uint32_t)dpllldfrefclkinven << 21) & ~((uint32_t)0x00200000)) == 0);
    REG_RPL_WR(RPL_DPLLCNTL3_ADDR, (REG_RPL_RD(RPL_DPLLCNTL3_ADDR) & ~((uint32_t)0x00200000)) | ((uint32_t)dpllldfrefclkinven << 21));
}

__INLINE uint8_t rpl_dpll_ld_fdiv_clk_inv_en_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_DPLLCNTL3_ADDR);
    return ((localVal & ((uint32_t)0x00100000)) >> 20);
}

__INLINE void rpl_dpll_ld_fdiv_clk_inv_en_setf(uint8_t dpllldfdivclkinven)
{
    ASSERT_ERR((((uint32_t)dpllldfdivclkinven << 20) & ~((uint32_t)0x00100000)) == 0);
    REG_RPL_WR(RPL_DPLLCNTL3_ADDR, (REG_RPL_RD(RPL_DPLLCNTL3_ADDR) & ~((uint32_t)0x00100000)) | ((uint32_t)dpllldfdivclkinven << 20));
}

__INLINE uint8_t rpl_dpll_ld_edges_comb_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_DPLLCNTL3_ADDR);
    return ((localVal & ((uint32_t)0x000C0000)) >> 18);
}

__INLINE void rpl_dpll_ld_edges_comb_setf(uint8_t dpllldedgescomb)
{
    ASSERT_ERR((((uint32_t)dpllldedgescomb << 18) & ~((uint32_t)0x000C0000)) == 0);
    REG_RPL_WR(RPL_DPLLCNTL3_ADDR, (REG_RPL_RD(RPL_DPLLCNTL3_ADDR) & ~((uint32_t)0x000C0000)) | ((uint32_t)dpllldedgescomb << 18));
}

__INLINE uint8_t rpl_dpll_ld_fe_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_DPLLCNTL3_ADDR);
    return ((localVal & ((uint32_t)0x00020000)) >> 17);
}

__INLINE void rpl_dpll_ld_fe_setf(uint8_t dpllldfe)
{
    ASSERT_ERR((((uint32_t)dpllldfe << 17) & ~((uint32_t)0x00020000)) == 0);
    REG_RPL_WR(RPL_DPLLCNTL3_ADDR, (REG_RPL_RD(RPL_DPLLCNTL3_ADDR) & ~((uint32_t)0x00020000)) | ((uint32_t)dpllldfe << 17));
}

__INLINE uint8_t rpl_dpll_ld_re_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_DPLLCNTL3_ADDR);
    return ((localVal & ((uint32_t)0x00010000)) >> 16);
}

__INLINE void rpl_dpll_ld_re_setf(uint8_t dpllldre)
{
    ASSERT_ERR((((uint32_t)dpllldre << 16) & ~((uint32_t)0x00010000)) == 0);
    REG_RPL_WR(RPL_DPLLCNTL3_ADDR, (REG_RPL_RD(RPL_DPLLCNTL3_ADDR) & ~((uint32_t)0x00010000)) | ((uint32_t)dpllldre << 16));
}

__INLINE uint8_t rpl_dpll_ld_unlock_cnt_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_DPLLCNTL3_ADDR);
    return ((localVal & ((uint32_t)0x0000FF00)) >> 8);
}

__INLINE void rpl_dpll_ld_unlock_cnt_setf(uint8_t dpllldunlockcnt)
{
    ASSERT_ERR((((uint32_t)dpllldunlockcnt << 8) & ~((uint32_t)0x0000FF00)) == 0);
    REG_RPL_WR(RPL_DPLLCNTL3_ADDR, (REG_RPL_RD(RPL_DPLLCNTL3_ADDR) & ~((uint32_t)0x0000FF00)) | ((uint32_t)dpllldunlockcnt << 8));
}

__INLINE uint8_t rpl_dpll_ld_lock_cnt_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_DPLLCNTL3_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

__INLINE void rpl_dpll_ld_lock_cnt_setf(uint8_t dpllldlockcnt)
{
    ASSERT_ERR((((uint32_t)dpllldlockcnt << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_RPL_WR(RPL_DPLLCNTL3_ADDR, (REG_RPL_RD(RPL_DPLLCNTL3_ADDR) & ~((uint32_t)0x000000FF)) | ((uint32_t)dpllldlockcnt << 0));
}

/**
 * @brief DPLLSTAT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     01      DPLL_VCO_CLK_OK   0
 *     00     DPLL_LOCK_DETECT   0
 * </pre>
 */
#define RPL_DPLLSTAT_ADDR   0x00000036
#define RPL_DPLLSTAT_OFFSET 0x00000036
#define RPL_DPLLSTAT_INDEX  0x00000036
#define RPL_DPLLSTAT_RESET  0x00000000

__INLINE uint32_t rpl_dpllstat_get(void)
{
    return REG_RPL_RD(RPL_DPLLSTAT_ADDR);
}

// field definitions
#define RPL_DPLL_VCO_CLK_OK_BIT     ((uint32_t)0x00000002)
#define RPL_DPLL_VCO_CLK_OK_POS     1
#define RPL_DPLL_LOCK_DETECT_BIT    ((uint32_t)0x00000001)
#define RPL_DPLL_LOCK_DETECT_POS    0

#define RPL_DPLL_VCO_CLK_OK_RST     0x0
#define RPL_DPLL_LOCK_DETECT_RST    0x0

__INLINE void rpl_dpllstat_unpack(uint8_t *dpllvcoclkok, uint8_t *dplllockdetect)
{
    uint32_t localVal = REG_RPL_RD(RPL_DPLLSTAT_ADDR);

    *dpllvcoclkok = (localVal & ((uint32_t)0x00000002)) >> 1;
    *dplllockdetect = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t rpl_dpll_vco_clk_ok_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_DPLLSTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE uint8_t rpl_dpll_lock_detect_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_DPLLSTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

/**
 * @brief RC_CALIB_CNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:28          FILT_RC_VAL   0x0
 *  27:24          RPLL_RC_VAL   0x0
 *  21:16          DPLL_RC_VAL   0x0
 *     11    FILT_RCCAL_CLK_EN   0
 *     10    RPLL_RCCAL_CLK_EN   0
 *     09    DPLL_RCCAL_CLK_EN   0
 *     08      FILT_RCCAL_NOSC   0
 *     07      RPLL_RCCAL_NOSC   0
 *     06      DPLL_RCCAL_NOSC   0
 *     02        FILT_RCCAL_ON   0
 *     01        RPLL_RCCAL_ON   0
 *     00        DPLL_RCCAL_ON   0
 * </pre>
 */
#define RPL_RC_CALIB_CNTL_ADDR   0x00000037
#define RPL_RC_CALIB_CNTL_OFFSET 0x00000037
#define RPL_RC_CALIB_CNTL_INDEX  0x00000037
#define RPL_RC_CALIB_CNTL_RESET  0x00000000

__INLINE uint32_t rpl_rc_calib_cntl_get(void)
{
    return REG_RPL_RD(RPL_RC_CALIB_CNTL_ADDR);
}

__INLINE void rpl_rc_calib_cntl_set(uint32_t value)
{
    REG_RPL_WR(RPL_RC_CALIB_CNTL_ADDR, value);
}

// field definitions
#define RPL_FILT_RC_VAL_MASK         ((uint32_t)0xF0000000)
#define RPL_FILT_RC_VAL_LSB          28
#define RPL_FILT_RC_VAL_WIDTH        ((uint32_t)0x00000004)
#define RPL_RPLL_RC_VAL_MASK         ((uint32_t)0x0F000000)
#define RPL_RPLL_RC_VAL_LSB          24
#define RPL_RPLL_RC_VAL_WIDTH        ((uint32_t)0x00000004)
#define RPL_DPLL_RC_VAL_MASK         ((uint32_t)0x003F0000)
#define RPL_DPLL_RC_VAL_LSB          16
#define RPL_DPLL_RC_VAL_WIDTH        ((uint32_t)0x00000006)
#define RPL_FILT_RCCAL_CLK_EN_BIT    ((uint32_t)0x00000800)
#define RPL_FILT_RCCAL_CLK_EN_POS    11
#define RPL_RPLL_RCCAL_CLK_EN_BIT    ((uint32_t)0x00000400)
#define RPL_RPLL_RCCAL_CLK_EN_POS    10
#define RPL_DPLL_RCCAL_CLK_EN_BIT    ((uint32_t)0x00000200)
#define RPL_DPLL_RCCAL_CLK_EN_POS    9
#define RPL_FILT_RCCAL_NOSC_BIT      ((uint32_t)0x00000100)
#define RPL_FILT_RCCAL_NOSC_POS      8
#define RPL_RPLL_RCCAL_NOSC_BIT      ((uint32_t)0x00000080)
#define RPL_RPLL_RCCAL_NOSC_POS      7
#define RPL_DPLL_RCCAL_NOSC_BIT      ((uint32_t)0x00000040)
#define RPL_DPLL_RCCAL_NOSC_POS      6
#define RPL_FILT_RCCAL_ON_BIT        ((uint32_t)0x00000004)
#define RPL_FILT_RCCAL_ON_POS        2
#define RPL_RPLL_RCCAL_ON_BIT        ((uint32_t)0x00000002)
#define RPL_RPLL_RCCAL_ON_POS        1
#define RPL_DPLL_RCCAL_ON_BIT        ((uint32_t)0x00000001)
#define RPL_DPLL_RCCAL_ON_POS        0

#define RPL_FILT_RC_VAL_RST          0x0
#define RPL_RPLL_RC_VAL_RST          0x0
#define RPL_DPLL_RC_VAL_RST          0x0
#define RPL_FILT_RCCAL_CLK_EN_RST    0x0
#define RPL_RPLL_RCCAL_CLK_EN_RST    0x0
#define RPL_DPLL_RCCAL_CLK_EN_RST    0x0
#define RPL_FILT_RCCAL_NOSC_RST      0x0
#define RPL_RPLL_RCCAL_NOSC_RST      0x0
#define RPL_DPLL_RCCAL_NOSC_RST      0x0
#define RPL_FILT_RCCAL_ON_RST        0x0
#define RPL_RPLL_RCCAL_ON_RST        0x0
#define RPL_DPLL_RCCAL_ON_RST        0x0

__INLINE void rpl_rc_calib_cntl_pack(uint8_t filtrcval, uint8_t rpllrcval, uint8_t dpllrcval, uint8_t filtrccalclken, uint8_t rpllrccalclken, uint8_t dpllrccalclken, uint8_t filtrccalnosc, uint8_t rpllrccalnosc, uint8_t dpllrccalnosc, uint8_t filtrccalon, uint8_t rpllrccalon, uint8_t dpllrccalon)
{
    ASSERT_ERR((((uint32_t)filtrcval << 28) & ~((uint32_t)0xF0000000)) == 0);
    ASSERT_ERR((((uint32_t)rpllrcval << 24) & ~((uint32_t)0x0F000000)) == 0);
    ASSERT_ERR((((uint32_t)dpllrcval << 16) & ~((uint32_t)0x003F0000)) == 0);
    ASSERT_ERR((((uint32_t)filtrccalclken << 11) & ~((uint32_t)0x00000800)) == 0);
    ASSERT_ERR((((uint32_t)rpllrccalclken << 10) & ~((uint32_t)0x00000400)) == 0);
    ASSERT_ERR((((uint32_t)dpllrccalclken << 9) & ~((uint32_t)0x00000200)) == 0);
    ASSERT_ERR((((uint32_t)filtrccalnosc << 8) & ~((uint32_t)0x00000100)) == 0);
    ASSERT_ERR((((uint32_t)rpllrccalnosc << 7) & ~((uint32_t)0x00000080)) == 0);
    ASSERT_ERR((((uint32_t)dpllrccalnosc << 6) & ~((uint32_t)0x00000040)) == 0);
    ASSERT_ERR((((uint32_t)filtrccalon << 2) & ~((uint32_t)0x00000004)) == 0);
    ASSERT_ERR((((uint32_t)rpllrccalon << 1) & ~((uint32_t)0x00000002)) == 0);
    ASSERT_ERR((((uint32_t)dpllrccalon << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_RPL_WR(RPL_RC_CALIB_CNTL_ADDR, ((uint32_t)filtrcval << 28) | ((uint32_t)rpllrcval << 24) | ((uint32_t)dpllrcval << 16) | ((uint32_t)filtrccalclken << 11) | ((uint32_t)rpllrccalclken << 10) | ((uint32_t)dpllrccalclken << 9) | ((uint32_t)filtrccalnosc << 8) | ((uint32_t)rpllrccalnosc << 7) | ((uint32_t)dpllrccalnosc << 6) | ((uint32_t)filtrccalon << 2) | ((uint32_t)rpllrccalon << 1) | ((uint32_t)dpllrccalon << 0));
}

__INLINE void rpl_rc_calib_cntl_unpack(uint8_t *filtrcval, uint8_t *rpllrcval, uint8_t *dpllrcval, uint8_t *filtrccalclken, uint8_t *rpllrccalclken, uint8_t *dpllrccalclken, uint8_t *filtrccalnosc, uint8_t *rpllrccalnosc, uint8_t *dpllrccalnosc, uint8_t *filtrccalon, uint8_t *rpllrccalon, uint8_t *dpllrccalon)
{
    uint32_t localVal = REG_RPL_RD(RPL_RC_CALIB_CNTL_ADDR);

    *filtrcval = (localVal & ((uint32_t)0xF0000000)) >> 28;
    *rpllrcval = (localVal & ((uint32_t)0x0F000000)) >> 24;
    *dpllrcval = (localVal & ((uint32_t)0x003F0000)) >> 16;
    *filtrccalclken = (localVal & ((uint32_t)0x00000800)) >> 11;
    *rpllrccalclken = (localVal & ((uint32_t)0x00000400)) >> 10;
    *dpllrccalclken = (localVal & ((uint32_t)0x00000200)) >> 9;
    *filtrccalnosc = (localVal & ((uint32_t)0x00000100)) >> 8;
    *rpllrccalnosc = (localVal & ((uint32_t)0x00000080)) >> 7;
    *dpllrccalnosc = (localVal & ((uint32_t)0x00000040)) >> 6;
    *filtrccalon = (localVal & ((uint32_t)0x00000004)) >> 2;
    *rpllrccalon = (localVal & ((uint32_t)0x00000002)) >> 1;
    *dpllrccalon = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t rpl_filt_rc_val_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RC_CALIB_CNTL_ADDR);
    return ((localVal & ((uint32_t)0xF0000000)) >> 28);
}

__INLINE void rpl_filt_rc_val_setf(uint8_t filtrcval)
{
    ASSERT_ERR((((uint32_t)filtrcval << 28) & ~((uint32_t)0xF0000000)) == 0);
    REG_RPL_WR(RPL_RC_CALIB_CNTL_ADDR, (REG_RPL_RD(RPL_RC_CALIB_CNTL_ADDR) & ~((uint32_t)0xF0000000)) | ((uint32_t)filtrcval << 28));
}

__INLINE uint8_t rpl_rpll_rc_val_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RC_CALIB_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x0F000000)) >> 24);
}

__INLINE void rpl_rpll_rc_val_setf(uint8_t rpllrcval)
{
    ASSERT_ERR((((uint32_t)rpllrcval << 24) & ~((uint32_t)0x0F000000)) == 0);
    REG_RPL_WR(RPL_RC_CALIB_CNTL_ADDR, (REG_RPL_RD(RPL_RC_CALIB_CNTL_ADDR) & ~((uint32_t)0x0F000000)) | ((uint32_t)rpllrcval << 24));
}

__INLINE uint8_t rpl_dpll_rc_val_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RC_CALIB_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x003F0000)) >> 16);
}

__INLINE void rpl_dpll_rc_val_setf(uint8_t dpllrcval)
{
    ASSERT_ERR((((uint32_t)dpllrcval << 16) & ~((uint32_t)0x003F0000)) == 0);
    REG_RPL_WR(RPL_RC_CALIB_CNTL_ADDR, (REG_RPL_RD(RPL_RC_CALIB_CNTL_ADDR) & ~((uint32_t)0x003F0000)) | ((uint32_t)dpllrcval << 16));
}

__INLINE uint8_t rpl_filt_rccal_clk_en_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RC_CALIB_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000800)) >> 11);
}

__INLINE void rpl_filt_rccal_clk_en_setf(uint8_t filtrccalclken)
{
    ASSERT_ERR((((uint32_t)filtrccalclken << 11) & ~((uint32_t)0x00000800)) == 0);
    REG_RPL_WR(RPL_RC_CALIB_CNTL_ADDR, (REG_RPL_RD(RPL_RC_CALIB_CNTL_ADDR) & ~((uint32_t)0x00000800)) | ((uint32_t)filtrccalclken << 11));
}

__INLINE uint8_t rpl_rpll_rccal_clk_en_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RC_CALIB_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000400)) >> 10);
}

__INLINE void rpl_rpll_rccal_clk_en_setf(uint8_t rpllrccalclken)
{
    ASSERT_ERR((((uint32_t)rpllrccalclken << 10) & ~((uint32_t)0x00000400)) == 0);
    REG_RPL_WR(RPL_RC_CALIB_CNTL_ADDR, (REG_RPL_RD(RPL_RC_CALIB_CNTL_ADDR) & ~((uint32_t)0x00000400)) | ((uint32_t)rpllrccalclken << 10));
}

__INLINE uint8_t rpl_dpll_rccal_clk_en_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RC_CALIB_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000200)) >> 9);
}

__INLINE void rpl_dpll_rccal_clk_en_setf(uint8_t dpllrccalclken)
{
    ASSERT_ERR((((uint32_t)dpllrccalclken << 9) & ~((uint32_t)0x00000200)) == 0);
    REG_RPL_WR(RPL_RC_CALIB_CNTL_ADDR, (REG_RPL_RD(RPL_RC_CALIB_CNTL_ADDR) & ~((uint32_t)0x00000200)) | ((uint32_t)dpllrccalclken << 9));
}

__INLINE uint8_t rpl_filt_rccal_nosc_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RC_CALIB_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000100)) >> 8);
}

__INLINE void rpl_filt_rccal_nosc_setf(uint8_t filtrccalnosc)
{
    ASSERT_ERR((((uint32_t)filtrccalnosc << 8) & ~((uint32_t)0x00000100)) == 0);
    REG_RPL_WR(RPL_RC_CALIB_CNTL_ADDR, (REG_RPL_RD(RPL_RC_CALIB_CNTL_ADDR) & ~((uint32_t)0x00000100)) | ((uint32_t)filtrccalnosc << 8));
}

__INLINE uint8_t rpl_rpll_rccal_nosc_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RC_CALIB_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000080)) >> 7);
}

__INLINE void rpl_rpll_rccal_nosc_setf(uint8_t rpllrccalnosc)
{
    ASSERT_ERR((((uint32_t)rpllrccalnosc << 7) & ~((uint32_t)0x00000080)) == 0);
    REG_RPL_WR(RPL_RC_CALIB_CNTL_ADDR, (REG_RPL_RD(RPL_RC_CALIB_CNTL_ADDR) & ~((uint32_t)0x00000080)) | ((uint32_t)rpllrccalnosc << 7));
}

__INLINE uint8_t rpl_dpll_rccal_nosc_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RC_CALIB_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000040)) >> 6);
}

__INLINE void rpl_dpll_rccal_nosc_setf(uint8_t dpllrccalnosc)
{
    ASSERT_ERR((((uint32_t)dpllrccalnosc << 6) & ~((uint32_t)0x00000040)) == 0);
    REG_RPL_WR(RPL_RC_CALIB_CNTL_ADDR, (REG_RPL_RD(RPL_RC_CALIB_CNTL_ADDR) & ~((uint32_t)0x00000040)) | ((uint32_t)dpllrccalnosc << 6));
}

__INLINE uint8_t rpl_filt_rccal_on_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RC_CALIB_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE void rpl_filt_rccal_on_setf(uint8_t filtrccalon)
{
    ASSERT_ERR((((uint32_t)filtrccalon << 2) & ~((uint32_t)0x00000004)) == 0);
    REG_RPL_WR(RPL_RC_CALIB_CNTL_ADDR, (REG_RPL_RD(RPL_RC_CALIB_CNTL_ADDR) & ~((uint32_t)0x00000004)) | ((uint32_t)filtrccalon << 2));
}

__INLINE uint8_t rpl_rpll_rccal_on_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RC_CALIB_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE void rpl_rpll_rccal_on_setf(uint8_t rpllrccalon)
{
    ASSERT_ERR((((uint32_t)rpllrccalon << 1) & ~((uint32_t)0x00000002)) == 0);
    REG_RPL_WR(RPL_RC_CALIB_CNTL_ADDR, (REG_RPL_RD(RPL_RC_CALIB_CNTL_ADDR) & ~((uint32_t)0x00000002)) | ((uint32_t)rpllrccalon << 1));
}

__INLINE uint8_t rpl_dpll_rccal_on_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RC_CALIB_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void rpl_dpll_rccal_on_setf(uint8_t dpllrccalon)
{
    ASSERT_ERR((((uint32_t)dpllrccalon << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_RPL_WR(RPL_RC_CALIB_CNTL_ADDR, (REG_RPL_RD(RPL_RC_CALIB_CNTL_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)dpllrccalon << 0));
}

/**
 * @brief PLL_RC_CALIB_STAT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  29:20   RPLL_RCCAL_BTUNESTAT   0x0
 *     16      RPLL_RCCAL_DONE   0
 *  13:04   DPLL_RCCAL_BTUNESTAT   0x0
 *     00      DPLL_RCCAL_DONE   0
 * </pre>
 */
#define RPL_PLL_RC_CALIB_STAT_ADDR   0x00000038
#define RPL_PLL_RC_CALIB_STAT_OFFSET 0x00000038
#define RPL_PLL_RC_CALIB_STAT_INDEX  0x00000038
#define RPL_PLL_RC_CALIB_STAT_RESET  0x00000000

__INLINE uint32_t rpl_pll_rc_calib_stat_get(void)
{
    return REG_RPL_RD(RPL_PLL_RC_CALIB_STAT_ADDR);
}

// field definitions
#define RPL_RPLL_RCCAL_BTUNESTAT_MASK   ((uint32_t)0x3FF00000)
#define RPL_RPLL_RCCAL_BTUNESTAT_LSB    20
#define RPL_RPLL_RCCAL_BTUNESTAT_WIDTH  ((uint32_t)0x0000000A)
#define RPL_RPLL_RCCAL_DONE_BIT         ((uint32_t)0x00010000)
#define RPL_RPLL_RCCAL_DONE_POS         16
#define RPL_DPLL_RCCAL_BTUNESTAT_MASK   ((uint32_t)0x00003FF0)
#define RPL_DPLL_RCCAL_BTUNESTAT_LSB    4
#define RPL_DPLL_RCCAL_BTUNESTAT_WIDTH  ((uint32_t)0x0000000A)
#define RPL_DPLL_RCCAL_DONE_BIT         ((uint32_t)0x00000001)
#define RPL_DPLL_RCCAL_DONE_POS         0

#define RPL_RPLL_RCCAL_BTUNESTAT_RST    0x0
#define RPL_RPLL_RCCAL_DONE_RST         0x0
#define RPL_DPLL_RCCAL_BTUNESTAT_RST    0x0
#define RPL_DPLL_RCCAL_DONE_RST         0x0

__INLINE void rpl_pll_rc_calib_stat_unpack(uint16_t *rpllrccalbtunestat, uint8_t *rpllrccaldone, uint16_t *dpllrccalbtunestat, uint8_t *dpllrccaldone)
{
    uint32_t localVal = REG_RPL_RD(RPL_PLL_RC_CALIB_STAT_ADDR);

    *rpllrccalbtunestat = (localVal & ((uint32_t)0x3FF00000)) >> 20;
    *rpllrccaldone = (localVal & ((uint32_t)0x00010000)) >> 16;
    *dpllrccalbtunestat = (localVal & ((uint32_t)0x00003FF0)) >> 4;
    *dpllrccaldone = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint16_t rpl_rpll_rccal_btunestat_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_PLL_RC_CALIB_STAT_ADDR);
    return ((localVal & ((uint32_t)0x3FF00000)) >> 20);
}

__INLINE uint8_t rpl_rpll_rccal_done_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_PLL_RC_CALIB_STAT_ADDR);
    return ((localVal & ((uint32_t)0x00010000)) >> 16);
}

__INLINE uint16_t rpl_dpll_rccal_btunestat_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_PLL_RC_CALIB_STAT_ADDR);
    return ((localVal & ((uint32_t)0x00003FF0)) >> 4);
}

__INLINE uint8_t rpl_dpll_rccal_done_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_PLL_RC_CALIB_STAT_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

/**
 * @brief FILTER_RC_CALIB_STAT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  13:04   FILT_RCCAL_BTUNESTAT   0x0
 *     00      FILT_RCCAL_DONE   0
 * </pre>
 */
#define RPL_FILTER_RC_CALIB_STAT_ADDR   0x00000039
#define RPL_FILTER_RC_CALIB_STAT_OFFSET 0x00000039
#define RPL_FILTER_RC_CALIB_STAT_INDEX  0x00000039
#define RPL_FILTER_RC_CALIB_STAT_RESET  0x00000000

__INLINE uint32_t rpl_filter_rc_calib_stat_get(void)
{
    return REG_RPL_RD(RPL_FILTER_RC_CALIB_STAT_ADDR);
}

// field definitions
#define RPL_FILT_RCCAL_BTUNESTAT_MASK   ((uint32_t)0x00003FF0)
#define RPL_FILT_RCCAL_BTUNESTAT_LSB    4
#define RPL_FILT_RCCAL_BTUNESTAT_WIDTH  ((uint32_t)0x0000000A)
#define RPL_FILT_RCCAL_DONE_BIT         ((uint32_t)0x00000001)
#define RPL_FILT_RCCAL_DONE_POS         0

#define RPL_FILT_RCCAL_BTUNESTAT_RST    0x0
#define RPL_FILT_RCCAL_DONE_RST         0x0

__INLINE void rpl_filter_rc_calib_stat_unpack(uint16_t *filtrccalbtunestat, uint8_t *filtrccaldone)
{
    uint32_t localVal = REG_RPL_RD(RPL_FILTER_RC_CALIB_STAT_ADDR);

    *filtrccalbtunestat = (localVal & ((uint32_t)0x00003FF0)) >> 4;
    *filtrccaldone = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint16_t rpl_filt_rccal_btunestat_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_FILTER_RC_CALIB_STAT_ADDR);
    return ((localVal & ((uint32_t)0x00003FF0)) >> 4);
}

__INLINE uint8_t rpl_filt_rccal_done_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_FILTER_RC_CALIB_STAT_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

/**
 * @brief VCOFC_ICP_CALIBCNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     16            LUT_LD_IN   0
 *  15:12           LUT_ICP_IN   0x0
 *  11:08         LUT_VCOFC_IN   0x0
 *     07    LUT_VCOFC_ICP_WEN   0
 *  06:00    LUT_VCOFC_ICP_ADD   0x0
 * </pre>
 */
#define RPL_VCOFC_ICP_CALIBCNTL_ADDR   0x0000003A
#define RPL_VCOFC_ICP_CALIBCNTL_OFFSET 0x0000003A
#define RPL_VCOFC_ICP_CALIBCNTL_INDEX  0x0000003A
#define RPL_VCOFC_ICP_CALIBCNTL_RESET  0x00000000

__INLINE uint32_t rpl_vcofc_icp_calibcntl_get(void)
{
    return REG_RPL_RD(RPL_VCOFC_ICP_CALIBCNTL_ADDR);
}

__INLINE void rpl_vcofc_icp_calibcntl_set(uint32_t value)
{
    REG_RPL_WR(RPL_VCOFC_ICP_CALIBCNTL_ADDR, value);
}

// field definitions
#define RPL_LUT_LD_IN_BIT            ((uint32_t)0x00010000)
#define RPL_LUT_LD_IN_POS            16
#define RPL_LUT_ICP_IN_MASK          ((uint32_t)0x0000F000)
#define RPL_LUT_ICP_IN_LSB           12
#define RPL_LUT_ICP_IN_WIDTH         ((uint32_t)0x00000004)
#define RPL_LUT_VCOFC_IN_MASK        ((uint32_t)0x00000F00)
#define RPL_LUT_VCOFC_IN_LSB         8
#define RPL_LUT_VCOFC_IN_WIDTH       ((uint32_t)0x00000004)
#define RPL_LUT_VCOFC_ICP_WEN_BIT    ((uint32_t)0x00000080)
#define RPL_LUT_VCOFC_ICP_WEN_POS    7
#define RPL_LUT_VCOFC_ICP_ADD_MASK   ((uint32_t)0x0000007F)
#define RPL_LUT_VCOFC_ICP_ADD_LSB    0
#define RPL_LUT_VCOFC_ICP_ADD_WIDTH  ((uint32_t)0x00000007)

#define RPL_LUT_LD_IN_RST            0x0
#define RPL_LUT_ICP_IN_RST           0x0
#define RPL_LUT_VCOFC_IN_RST         0x0
#define RPL_LUT_VCOFC_ICP_WEN_RST    0x0
#define RPL_LUT_VCOFC_ICP_ADD_RST    0x0

__INLINE void rpl_vcofc_icp_calibcntl_pack(uint8_t lutldin, uint8_t luticpin, uint8_t lutvcofcin, uint8_t lutvcofcicpwen, uint8_t lutvcofcicpadd)
{
    ASSERT_ERR((((uint32_t)lutldin << 16) & ~((uint32_t)0x00010000)) == 0);
    ASSERT_ERR((((uint32_t)luticpin << 12) & ~((uint32_t)0x0000F000)) == 0);
    ASSERT_ERR((((uint32_t)lutvcofcin << 8) & ~((uint32_t)0x00000F00)) == 0);
    ASSERT_ERR((((uint32_t)lutvcofcicpwen << 7) & ~((uint32_t)0x00000080)) == 0);
    ASSERT_ERR((((uint32_t)lutvcofcicpadd << 0) & ~((uint32_t)0x0000007F)) == 0);
    REG_RPL_WR(RPL_VCOFC_ICP_CALIBCNTL_ADDR, ((uint32_t)lutldin << 16) | ((uint32_t)luticpin << 12) | ((uint32_t)lutvcofcin << 8) | ((uint32_t)lutvcofcicpwen << 7) | ((uint32_t)lutvcofcicpadd << 0));
}

__INLINE void rpl_vcofc_icp_calibcntl_unpack(uint8_t *lutldin, uint8_t *luticpin, uint8_t *lutvcofcin, uint8_t *lutvcofcicpwen, uint8_t *lutvcofcicpadd)
{
    uint32_t localVal = REG_RPL_RD(RPL_VCOFC_ICP_CALIBCNTL_ADDR);

    *lutldin = (localVal & ((uint32_t)0x00010000)) >> 16;
    *luticpin = (localVal & ((uint32_t)0x0000F000)) >> 12;
    *lutvcofcin = (localVal & ((uint32_t)0x00000F00)) >> 8;
    *lutvcofcicpwen = (localVal & ((uint32_t)0x00000080)) >> 7;
    *lutvcofcicpadd = (localVal & ((uint32_t)0x0000007F)) >> 0;
}

__INLINE uint8_t rpl_lut_ld_in_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_VCOFC_ICP_CALIBCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00010000)) >> 16);
}

__INLINE void rpl_lut_ld_in_setf(uint8_t lutldin)
{
    ASSERT_ERR((((uint32_t)lutldin << 16) & ~((uint32_t)0x00010000)) == 0);
    REG_RPL_WR(RPL_VCOFC_ICP_CALIBCNTL_ADDR, (REG_RPL_RD(RPL_VCOFC_ICP_CALIBCNTL_ADDR) & ~((uint32_t)0x00010000)) | ((uint32_t)lutldin << 16));
}

__INLINE uint8_t rpl_lut_icp_in_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_VCOFC_ICP_CALIBCNTL_ADDR);
    return ((localVal & ((uint32_t)0x0000F000)) >> 12);
}

__INLINE void rpl_lut_icp_in_setf(uint8_t luticpin)
{
    ASSERT_ERR((((uint32_t)luticpin << 12) & ~((uint32_t)0x0000F000)) == 0);
    REG_RPL_WR(RPL_VCOFC_ICP_CALIBCNTL_ADDR, (REG_RPL_RD(RPL_VCOFC_ICP_CALIBCNTL_ADDR) & ~((uint32_t)0x0000F000)) | ((uint32_t)luticpin << 12));
}

__INLINE uint8_t rpl_lut_vcofc_in_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_VCOFC_ICP_CALIBCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000F00)) >> 8);
}

__INLINE void rpl_lut_vcofc_in_setf(uint8_t lutvcofcin)
{
    ASSERT_ERR((((uint32_t)lutvcofcin << 8) & ~((uint32_t)0x00000F00)) == 0);
    REG_RPL_WR(RPL_VCOFC_ICP_CALIBCNTL_ADDR, (REG_RPL_RD(RPL_VCOFC_ICP_CALIBCNTL_ADDR) & ~((uint32_t)0x00000F00)) | ((uint32_t)lutvcofcin << 8));
}

__INLINE uint8_t rpl_lut_vcofc_icp_wen_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_VCOFC_ICP_CALIBCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000080)) >> 7);
}

__INLINE void rpl_lut_vcofc_icp_wen_setf(uint8_t lutvcofcicpwen)
{
    ASSERT_ERR((((uint32_t)lutvcofcicpwen << 7) & ~((uint32_t)0x00000080)) == 0);
    REG_RPL_WR(RPL_VCOFC_ICP_CALIBCNTL_ADDR, (REG_RPL_RD(RPL_VCOFC_ICP_CALIBCNTL_ADDR) & ~((uint32_t)0x00000080)) | ((uint32_t)lutvcofcicpwen << 7));
}

__INLINE uint8_t rpl_lut_vcofc_icp_add_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_VCOFC_ICP_CALIBCNTL_ADDR);
    return ((localVal & ((uint32_t)0x0000007F)) >> 0);
}

__INLINE void rpl_lut_vcofc_icp_add_setf(uint8_t lutvcofcicpadd)
{
    ASSERT_ERR((((uint32_t)lutvcofcicpadd << 0) & ~((uint32_t)0x0000007F)) == 0);
    REG_RPL_WR(RPL_VCOFC_ICP_CALIBCNTL_ADDR, (REG_RPL_RD(RPL_VCOFC_ICP_CALIBCNTL_ADDR) & ~((uint32_t)0x0000007F)) | ((uint32_t)lutvcofcicpadd << 0));
}

/**
 * @brief VCOFC_ICP_CALIBCONFIG register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  26:24         KVCOSTEP_DEN   0x6
 *  23:16         KVCOSTEP_NUM   0x3
 *     12           ICPCALSIGN   0
 *  09:00             KVCO_REF   0x1AB
 * </pre>
 */
#define RPL_VCOFC_ICP_CALIBCONFIG_ADDR   0x0000003B
#define RPL_VCOFC_ICP_CALIBCONFIG_OFFSET 0x0000003B
#define RPL_VCOFC_ICP_CALIBCONFIG_INDEX  0x0000003B
#define RPL_VCOFC_ICP_CALIBCONFIG_RESET  0x060301AB

__INLINE uint32_t rpl_vcofc_icp_calibconfig_get(void)
{
    return REG_RPL_RD(RPL_VCOFC_ICP_CALIBCONFIG_ADDR);
}

__INLINE void rpl_vcofc_icp_calibconfig_set(uint32_t value)
{
    REG_RPL_WR(RPL_VCOFC_ICP_CALIBCONFIG_ADDR, value);
}

// field definitions
#define RPL_KVCOSTEP_DEN_MASK   ((uint32_t)0x07000000)
#define RPL_KVCOSTEP_DEN_LSB    24
#define RPL_KVCOSTEP_DEN_WIDTH  ((uint32_t)0x00000003)
#define RPL_KVCOSTEP_NUM_MASK   ((uint32_t)0x00FF0000)
#define RPL_KVCOSTEP_NUM_LSB    16
#define RPL_KVCOSTEP_NUM_WIDTH  ((uint32_t)0x00000008)
#define RPL_ICPCALSIGN_BIT      ((uint32_t)0x00001000)
#define RPL_ICPCALSIGN_POS      12
#define RPL_KVCO_REF_MASK       ((uint32_t)0x000003FF)
#define RPL_KVCO_REF_LSB        0
#define RPL_KVCO_REF_WIDTH      ((uint32_t)0x0000000A)

#define RPL_KVCOSTEP_DEN_RST    0x6
#define RPL_KVCOSTEP_NUM_RST    0x3
#define RPL_ICPCALSIGN_RST      0x0
#define RPL_KVCO_REF_RST        0x1AB

__INLINE void rpl_vcofc_icp_calibconfig_pack(uint8_t kvcostepden, uint8_t kvcostepnum, uint8_t icpcalsign, uint16_t kvcoref)
{
    ASSERT_ERR((((uint32_t)kvcostepden << 24) & ~((uint32_t)0x07000000)) == 0);
    ASSERT_ERR((((uint32_t)kvcostepnum << 16) & ~((uint32_t)0x00FF0000)) == 0);
    ASSERT_ERR((((uint32_t)icpcalsign << 12) & ~((uint32_t)0x00001000)) == 0);
    ASSERT_ERR((((uint32_t)kvcoref << 0) & ~((uint32_t)0x000003FF)) == 0);
    REG_RPL_WR(RPL_VCOFC_ICP_CALIBCONFIG_ADDR, ((uint32_t)kvcostepden << 24) | ((uint32_t)kvcostepnum << 16) | ((uint32_t)icpcalsign << 12) | ((uint32_t)kvcoref << 0));
}

__INLINE void rpl_vcofc_icp_calibconfig_unpack(uint8_t *kvcostepden, uint8_t *kvcostepnum, uint8_t *icpcalsign, uint16_t *kvcoref)
{
    uint32_t localVal = REG_RPL_RD(RPL_VCOFC_ICP_CALIBCONFIG_ADDR);

    *kvcostepden = (localVal & ((uint32_t)0x07000000)) >> 24;
    *kvcostepnum = (localVal & ((uint32_t)0x00FF0000)) >> 16;
    *icpcalsign = (localVal & ((uint32_t)0x00001000)) >> 12;
    *kvcoref = (localVal & ((uint32_t)0x000003FF)) >> 0;
}

__INLINE uint8_t rpl_kvcostep_den_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_VCOFC_ICP_CALIBCONFIG_ADDR);
    return ((localVal & ((uint32_t)0x07000000)) >> 24);
}

__INLINE void rpl_kvcostep_den_setf(uint8_t kvcostepden)
{
    ASSERT_ERR((((uint32_t)kvcostepden << 24) & ~((uint32_t)0x07000000)) == 0);
    REG_RPL_WR(RPL_VCOFC_ICP_CALIBCONFIG_ADDR, (REG_RPL_RD(RPL_VCOFC_ICP_CALIBCONFIG_ADDR) & ~((uint32_t)0x07000000)) | ((uint32_t)kvcostepden << 24));
}

__INLINE uint8_t rpl_kvcostep_num_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_VCOFC_ICP_CALIBCONFIG_ADDR);
    return ((localVal & ((uint32_t)0x00FF0000)) >> 16);
}

__INLINE void rpl_kvcostep_num_setf(uint8_t kvcostepnum)
{
    ASSERT_ERR((((uint32_t)kvcostepnum << 16) & ~((uint32_t)0x00FF0000)) == 0);
    REG_RPL_WR(RPL_VCOFC_ICP_CALIBCONFIG_ADDR, (REG_RPL_RD(RPL_VCOFC_ICP_CALIBCONFIG_ADDR) & ~((uint32_t)0x00FF0000)) | ((uint32_t)kvcostepnum << 16));
}

__INLINE uint8_t rpl_icpcalsign_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_VCOFC_ICP_CALIBCONFIG_ADDR);
    return ((localVal & ((uint32_t)0x00001000)) >> 12);
}

__INLINE void rpl_icpcalsign_setf(uint8_t icpcalsign)
{
    ASSERT_ERR((((uint32_t)icpcalsign << 12) & ~((uint32_t)0x00001000)) == 0);
    REG_RPL_WR(RPL_VCOFC_ICP_CALIBCONFIG_ADDR, (REG_RPL_RD(RPL_VCOFC_ICP_CALIBCONFIG_ADDR) & ~((uint32_t)0x00001000)) | ((uint32_t)icpcalsign << 12));
}

__INLINE uint16_t rpl_kvco_ref_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_VCOFC_ICP_CALIBCONFIG_ADDR);
    return ((localVal & ((uint32_t)0x000003FF)) >> 0);
}

__INLINE void rpl_kvco_ref_setf(uint16_t kvcoref)
{
    ASSERT_ERR((((uint32_t)kvcoref << 0) & ~((uint32_t)0x000003FF)) == 0);
    REG_RPL_WR(RPL_VCOFC_ICP_CALIBCONFIG_ADDR, (REG_RPL_RD(RPL_VCOFC_ICP_CALIBCONFIG_ADDR) & ~((uint32_t)0x000003FF)) | ((uint32_t)kvcoref << 0));
}

/**
 * @brief VCOFC_ICP_CALIBSTAT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     08           LUT_LD_VAL   0
 *  07:04          LUT_ICP_VAL   0x0
 *  03:00        LUT_VCOFC_VAL   0x0
 * </pre>
 */
#define RPL_VCOFC_ICP_CALIBSTAT_ADDR   0x0000003C
#define RPL_VCOFC_ICP_CALIBSTAT_OFFSET 0x0000003C
#define RPL_VCOFC_ICP_CALIBSTAT_INDEX  0x0000003C
#define RPL_VCOFC_ICP_CALIBSTAT_RESET  0x00000000

__INLINE uint32_t rpl_vcofc_icp_calibstat_get(void)
{
    return REG_RPL_RD(RPL_VCOFC_ICP_CALIBSTAT_ADDR);
}

// field definitions
#define RPL_LUT_LD_VAL_BIT       ((uint32_t)0x00000100)
#define RPL_LUT_LD_VAL_POS       8
#define RPL_LUT_ICP_VAL_MASK     ((uint32_t)0x000000F0)
#define RPL_LUT_ICP_VAL_LSB      4
#define RPL_LUT_ICP_VAL_WIDTH    ((uint32_t)0x00000004)
#define RPL_LUT_VCOFC_VAL_MASK   ((uint32_t)0x0000000F)
#define RPL_LUT_VCOFC_VAL_LSB    0
#define RPL_LUT_VCOFC_VAL_WIDTH  ((uint32_t)0x00000004)

#define RPL_LUT_LD_VAL_RST       0x0
#define RPL_LUT_ICP_VAL_RST      0x0
#define RPL_LUT_VCOFC_VAL_RST    0x0

__INLINE void rpl_vcofc_icp_calibstat_unpack(uint8_t *lutldval, uint8_t *luticpval, uint8_t *lutvcofcval)
{
    uint32_t localVal = REG_RPL_RD(RPL_VCOFC_ICP_CALIBSTAT_ADDR);

    *lutldval = (localVal & ((uint32_t)0x00000100)) >> 8;
    *luticpval = (localVal & ((uint32_t)0x000000F0)) >> 4;
    *lutvcofcval = (localVal & ((uint32_t)0x0000000F)) >> 0;
}

__INLINE uint8_t rpl_lut_ld_val_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_VCOFC_ICP_CALIBSTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000100)) >> 8);
}

__INLINE uint8_t rpl_lut_icp_val_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_VCOFC_ICP_CALIBSTAT_ADDR);
    return ((localVal & ((uint32_t)0x000000F0)) >> 4);
}

__INLINE uint8_t rpl_lut_vcofc_val_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_VCOFC_ICP_CALIBSTAT_ADDR);
    return ((localVal & ((uint32_t)0x0000000F)) >> 0);
}

/**
 * @brief TXDC_CALIBCNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  30:28         LUT_TXDC_SEL   0x0
 *  20:16         LUT_TXDCQ_IN   0x0
 *  12:08         LUT_TXDCI_IN   0x0
 *     04         LUT_TXDC_WEN   0
 *  03:00         LUT_TXDC_ADD   0x0
 * </pre>
 */
#define RPL_TXDC_CALIBCNTL_ADDR   0x0000003D
#define RPL_TXDC_CALIBCNTL_OFFSET 0x0000003D
#define RPL_TXDC_CALIBCNTL_INDEX  0x0000003D
#define RPL_TXDC_CALIBCNTL_RESET  0x00000000

__INLINE uint32_t rpl_txdc_calibcntl_get(void)
{
    return REG_RPL_RD(RPL_TXDC_CALIBCNTL_ADDR);
}

__INLINE void rpl_txdc_calibcntl_set(uint32_t value)
{
    REG_RPL_WR(RPL_TXDC_CALIBCNTL_ADDR, value);
}

// field definitions
#define RPL_LUT_TXDC_SEL_MASK   ((uint32_t)0x70000000)
#define RPL_LUT_TXDC_SEL_LSB    28
#define RPL_LUT_TXDC_SEL_WIDTH  ((uint32_t)0x00000003)
#define RPL_LUT_TXDCQ_IN_MASK   ((uint32_t)0x001F0000)
#define RPL_LUT_TXDCQ_IN_LSB    16
#define RPL_LUT_TXDCQ_IN_WIDTH  ((uint32_t)0x00000005)
#define RPL_LUT_TXDCI_IN_MASK   ((uint32_t)0x00001F00)
#define RPL_LUT_TXDCI_IN_LSB    8
#define RPL_LUT_TXDCI_IN_WIDTH  ((uint32_t)0x00000005)
#define RPL_LUT_TXDC_WEN_BIT    ((uint32_t)0x00000010)
#define RPL_LUT_TXDC_WEN_POS    4
#define RPL_LUT_TXDC_ADD_MASK   ((uint32_t)0x0000000F)
#define RPL_LUT_TXDC_ADD_LSB    0
#define RPL_LUT_TXDC_ADD_WIDTH  ((uint32_t)0x00000004)

#define RPL_LUT_TXDC_SEL_RST    0x0
#define RPL_LUT_TXDCQ_IN_RST    0x0
#define RPL_LUT_TXDCI_IN_RST    0x0
#define RPL_LUT_TXDC_WEN_RST    0x0
#define RPL_LUT_TXDC_ADD_RST    0x0

__INLINE void rpl_txdc_calibcntl_pack(uint8_t luttxdcsel, uint8_t luttxdcqin, uint8_t luttxdciin, uint8_t luttxdcwen, uint8_t luttxdcadd)
{
    ASSERT_ERR((((uint32_t)luttxdcsel << 28) & ~((uint32_t)0x70000000)) == 0);
    ASSERT_ERR((((uint32_t)luttxdcqin << 16) & ~((uint32_t)0x001F0000)) == 0);
    ASSERT_ERR((((uint32_t)luttxdciin << 8) & ~((uint32_t)0x00001F00)) == 0);
    ASSERT_ERR((((uint32_t)luttxdcwen << 4) & ~((uint32_t)0x00000010)) == 0);
    ASSERT_ERR((((uint32_t)luttxdcadd << 0) & ~((uint32_t)0x0000000F)) == 0);
    REG_RPL_WR(RPL_TXDC_CALIBCNTL_ADDR, ((uint32_t)luttxdcsel << 28) | ((uint32_t)luttxdcqin << 16) | ((uint32_t)luttxdciin << 8) | ((uint32_t)luttxdcwen << 4) | ((uint32_t)luttxdcadd << 0));
}

__INLINE void rpl_txdc_calibcntl_unpack(uint8_t *luttxdcsel, uint8_t *luttxdcqin, uint8_t *luttxdciin, uint8_t *luttxdcwen, uint8_t *luttxdcadd)
{
    uint32_t localVal = REG_RPL_RD(RPL_TXDC_CALIBCNTL_ADDR);

    *luttxdcsel = (localVal & ((uint32_t)0x70000000)) >> 28;
    *luttxdcqin = (localVal & ((uint32_t)0x001F0000)) >> 16;
    *luttxdciin = (localVal & ((uint32_t)0x00001F00)) >> 8;
    *luttxdcwen = (localVal & ((uint32_t)0x00000010)) >> 4;
    *luttxdcadd = (localVal & ((uint32_t)0x0000000F)) >> 0;
}

__INLINE uint8_t rpl_lut_txdc_sel_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_TXDC_CALIBCNTL_ADDR);
    return ((localVal & ((uint32_t)0x70000000)) >> 28);
}

__INLINE void rpl_lut_txdc_sel_setf(uint8_t luttxdcsel)
{
    ASSERT_ERR((((uint32_t)luttxdcsel << 28) & ~((uint32_t)0x70000000)) == 0);
    REG_RPL_WR(RPL_TXDC_CALIBCNTL_ADDR, (REG_RPL_RD(RPL_TXDC_CALIBCNTL_ADDR) & ~((uint32_t)0x70000000)) | ((uint32_t)luttxdcsel << 28));
}

__INLINE uint8_t rpl_lut_txdcq_in_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_TXDC_CALIBCNTL_ADDR);
    return ((localVal & ((uint32_t)0x001F0000)) >> 16);
}

__INLINE void rpl_lut_txdcq_in_setf(uint8_t luttxdcqin)
{
    ASSERT_ERR((((uint32_t)luttxdcqin << 16) & ~((uint32_t)0x001F0000)) == 0);
    REG_RPL_WR(RPL_TXDC_CALIBCNTL_ADDR, (REG_RPL_RD(RPL_TXDC_CALIBCNTL_ADDR) & ~((uint32_t)0x001F0000)) | ((uint32_t)luttxdcqin << 16));
}

__INLINE uint8_t rpl_lut_txdci_in_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_TXDC_CALIBCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00001F00)) >> 8);
}

__INLINE void rpl_lut_txdci_in_setf(uint8_t luttxdciin)
{
    ASSERT_ERR((((uint32_t)luttxdciin << 8) & ~((uint32_t)0x00001F00)) == 0);
    REG_RPL_WR(RPL_TXDC_CALIBCNTL_ADDR, (REG_RPL_RD(RPL_TXDC_CALIBCNTL_ADDR) & ~((uint32_t)0x00001F00)) | ((uint32_t)luttxdciin << 8));
}

__INLINE uint8_t rpl_lut_txdc_wen_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_TXDC_CALIBCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE void rpl_lut_txdc_wen_setf(uint8_t luttxdcwen)
{
    ASSERT_ERR((((uint32_t)luttxdcwen << 4) & ~((uint32_t)0x00000010)) == 0);
    REG_RPL_WR(RPL_TXDC_CALIBCNTL_ADDR, (REG_RPL_RD(RPL_TXDC_CALIBCNTL_ADDR) & ~((uint32_t)0x00000010)) | ((uint32_t)luttxdcwen << 4));
}

__INLINE uint8_t rpl_lut_txdc_add_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_TXDC_CALIBCNTL_ADDR);
    return ((localVal & ((uint32_t)0x0000000F)) >> 0);
}

__INLINE void rpl_lut_txdc_add_setf(uint8_t luttxdcadd)
{
    ASSERT_ERR((((uint32_t)luttxdcadd << 0) & ~((uint32_t)0x0000000F)) == 0);
    REG_RPL_WR(RPL_TXDC_CALIBCNTL_ADDR, (REG_RPL_RD(RPL_TXDC_CALIBCNTL_ADDR) & ~((uint32_t)0x0000000F)) | ((uint32_t)luttxdcadd << 0));
}

/**
 * @brief TXDC_CALIBSTAT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  29:25   LUT_TXDCQ_8DPSK_VAL   0x0
 *  24:20   LUT_TXDCI_8DPSK_VAL   0x0
 *  19:15   LUT_TXDCQ_DQPSK_VAL   0x0
 *  14:10   LUT_TXDCI_DQPSK_VAL   0x0
 *  09:05   LUT_TXDCQ_GFSK_VAL   0x0
 *  04:00   LUT_TXDCI_GFSK_VAL   0x0
 * </pre>
 */
#define RPL_TXDC_CALIBSTAT_ADDR   0x0000003E
#define RPL_TXDC_CALIBSTAT_OFFSET 0x0000003E
#define RPL_TXDC_CALIBSTAT_INDEX  0x0000003E
#define RPL_TXDC_CALIBSTAT_RESET  0x00000000

__INLINE uint32_t rpl_txdc_calibstat_get(void)
{
    return REG_RPL_RD(RPL_TXDC_CALIBSTAT_ADDR);
}

// field definitions
#define RPL_LUT_TXDCQ_8DPSK_VAL_MASK   ((uint32_t)0x3E000000)
#define RPL_LUT_TXDCQ_8DPSK_VAL_LSB    25
#define RPL_LUT_TXDCQ_8DPSK_VAL_WIDTH  ((uint32_t)0x00000005)
#define RPL_LUT_TXDCI_8DPSK_VAL_MASK   ((uint32_t)0x01F00000)
#define RPL_LUT_TXDCI_8DPSK_VAL_LSB    20
#define RPL_LUT_TXDCI_8DPSK_VAL_WIDTH  ((uint32_t)0x00000005)
#define RPL_LUT_TXDCQ_DQPSK_VAL_MASK   ((uint32_t)0x000F8000)
#define RPL_LUT_TXDCQ_DQPSK_VAL_LSB    15
#define RPL_LUT_TXDCQ_DQPSK_VAL_WIDTH  ((uint32_t)0x00000005)
#define RPL_LUT_TXDCI_DQPSK_VAL_MASK   ((uint32_t)0x00007C00)
#define RPL_LUT_TXDCI_DQPSK_VAL_LSB    10
#define RPL_LUT_TXDCI_DQPSK_VAL_WIDTH  ((uint32_t)0x00000005)
#define RPL_LUT_TXDCQ_GFSK_VAL_MASK    ((uint32_t)0x000003E0)
#define RPL_LUT_TXDCQ_GFSK_VAL_LSB     5
#define RPL_LUT_TXDCQ_GFSK_VAL_WIDTH   ((uint32_t)0x00000005)
#define RPL_LUT_TXDCI_GFSK_VAL_MASK    ((uint32_t)0x0000001F)
#define RPL_LUT_TXDCI_GFSK_VAL_LSB     0
#define RPL_LUT_TXDCI_GFSK_VAL_WIDTH   ((uint32_t)0x00000005)

#define RPL_LUT_TXDCQ_8DPSK_VAL_RST    0x0
#define RPL_LUT_TXDCI_8DPSK_VAL_RST    0x0
#define RPL_LUT_TXDCQ_DQPSK_VAL_RST    0x0
#define RPL_LUT_TXDCI_DQPSK_VAL_RST    0x0
#define RPL_LUT_TXDCQ_GFSK_VAL_RST     0x0
#define RPL_LUT_TXDCI_GFSK_VAL_RST     0x0

__INLINE void rpl_txdc_calibstat_unpack(uint8_t *luttxdcq8dpskval, uint8_t *luttxdci8dpskval, uint8_t *luttxdcqdqpskval, uint8_t *luttxdcidqpskval, uint8_t *luttxdcqgfskval, uint8_t *luttxdcigfskval)
{
    uint32_t localVal = REG_RPL_RD(RPL_TXDC_CALIBSTAT_ADDR);

    *luttxdcq8dpskval = (localVal & ((uint32_t)0x3E000000)) >> 25;
    *luttxdci8dpskval = (localVal & ((uint32_t)0x01F00000)) >> 20;
    *luttxdcqdqpskval = (localVal & ((uint32_t)0x000F8000)) >> 15;
    *luttxdcidqpskval = (localVal & ((uint32_t)0x00007C00)) >> 10;
    *luttxdcqgfskval = (localVal & ((uint32_t)0x000003E0)) >> 5;
    *luttxdcigfskval = (localVal & ((uint32_t)0x0000001F)) >> 0;
}

__INLINE uint8_t rpl_lut_txdcq_8dpsk_val_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_TXDC_CALIBSTAT_ADDR);
    return ((localVal & ((uint32_t)0x3E000000)) >> 25);
}

__INLINE uint8_t rpl_lut_txdci_8dpsk_val_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_TXDC_CALIBSTAT_ADDR);
    return ((localVal & ((uint32_t)0x01F00000)) >> 20);
}

__INLINE uint8_t rpl_lut_txdcq_dqpsk_val_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_TXDC_CALIBSTAT_ADDR);
    return ((localVal & ((uint32_t)0x000F8000)) >> 15);
}

__INLINE uint8_t rpl_lut_txdci_dqpsk_val_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_TXDC_CALIBSTAT_ADDR);
    return ((localVal & ((uint32_t)0x00007C00)) >> 10);
}

__INLINE uint8_t rpl_lut_txdcq_gfsk_val_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_TXDC_CALIBSTAT_ADDR);
    return ((localVal & ((uint32_t)0x000003E0)) >> 5);
}

__INLINE uint8_t rpl_lut_txdci_gfsk_val_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_TXDC_CALIBSTAT_ADDR);
    return ((localVal & ((uint32_t)0x0000001F)) >> 0);
}

/**
 * @brief AGC_RSSI_CONFIG register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     31    RXAGC_SATDET_MODE   1
 *  16:12     RSSI_PRELOAD_CNT   0x0
 *  11:08     RSSI_ON_DLYD_CNT   0x0
 *  07:00      AGC_SETTLE_TIME   0x40
 * </pre>
 */
#define RPL_AGC_RSSI_CONFIG_ADDR   0x0000003F
#define RPL_AGC_RSSI_CONFIG_OFFSET 0x0000003F
#define RPL_AGC_RSSI_CONFIG_INDEX  0x0000003F
#define RPL_AGC_RSSI_CONFIG_RESET  0x80000040

__INLINE uint32_t rpl_agc_rssi_config_get(void)
{
    return REG_RPL_RD(RPL_AGC_RSSI_CONFIG_ADDR);
}

__INLINE void rpl_agc_rssi_config_set(uint32_t value)
{
    REG_RPL_WR(RPL_AGC_RSSI_CONFIG_ADDR, value);
}

// field definitions
#define RPL_RXAGC_SATDET_MODE_BIT    ((uint32_t)0x80000000)
#define RPL_RXAGC_SATDET_MODE_POS    31
#define RPL_RSSI_PRELOAD_CNT_MASK    ((uint32_t)0x0001F000)
#define RPL_RSSI_PRELOAD_CNT_LSB     12
#define RPL_RSSI_PRELOAD_CNT_WIDTH   ((uint32_t)0x00000005)
#define RPL_RSSI_ON_DLYD_CNT_MASK    ((uint32_t)0x00000F00)
#define RPL_RSSI_ON_DLYD_CNT_LSB     8
#define RPL_RSSI_ON_DLYD_CNT_WIDTH   ((uint32_t)0x00000004)
#define RPL_AGC_SETTLE_TIME_MASK     ((uint32_t)0x000000FF)
#define RPL_AGC_SETTLE_TIME_LSB      0
#define RPL_AGC_SETTLE_TIME_WIDTH    ((uint32_t)0x00000008)

#define RPL_RXAGC_SATDET_MODE_RST    0x1
#define RPL_RSSI_PRELOAD_CNT_RST     0x0
#define RPL_RSSI_ON_DLYD_CNT_RST     0x0
#define RPL_AGC_SETTLE_TIME_RST      0x40

__INLINE void rpl_agc_rssi_config_pack(uint8_t rxagcsatdetmode, uint8_t rssipreloadcnt, uint8_t rssiondlydcnt, uint8_t agcsettletime)
{
    ASSERT_ERR((((uint32_t)rxagcsatdetmode << 31) & ~((uint32_t)0x80000000)) == 0);
    ASSERT_ERR((((uint32_t)rssipreloadcnt << 12) & ~((uint32_t)0x0001F000)) == 0);
    ASSERT_ERR((((uint32_t)rssiondlydcnt << 8) & ~((uint32_t)0x00000F00)) == 0);
    ASSERT_ERR((((uint32_t)agcsettletime << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_RPL_WR(RPL_AGC_RSSI_CONFIG_ADDR, ((uint32_t)rxagcsatdetmode << 31) | ((uint32_t)rssipreloadcnt << 12) | ((uint32_t)rssiondlydcnt << 8) | ((uint32_t)agcsettletime << 0));
}

__INLINE void rpl_agc_rssi_config_unpack(uint8_t *rxagcsatdetmode, uint8_t *rssipreloadcnt, uint8_t *rssiondlydcnt, uint8_t *agcsettletime)
{
    uint32_t localVal = REG_RPL_RD(RPL_AGC_RSSI_CONFIG_ADDR);

    *rxagcsatdetmode = (localVal & ((uint32_t)0x80000000)) >> 31;
    *rssipreloadcnt = (localVal & ((uint32_t)0x0001F000)) >> 12;
    *rssiondlydcnt = (localVal & ((uint32_t)0x00000F00)) >> 8;
    *agcsettletime = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint8_t rpl_rxagc_satdet_mode_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_AGC_RSSI_CONFIG_ADDR);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE void rpl_rxagc_satdet_mode_setf(uint8_t rxagcsatdetmode)
{
    ASSERT_ERR((((uint32_t)rxagcsatdetmode << 31) & ~((uint32_t)0x80000000)) == 0);
    REG_RPL_WR(RPL_AGC_RSSI_CONFIG_ADDR, (REG_RPL_RD(RPL_AGC_RSSI_CONFIG_ADDR) & ~((uint32_t)0x80000000)) | ((uint32_t)rxagcsatdetmode << 31));
}

__INLINE uint8_t rpl_rssi_preload_cnt_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_AGC_RSSI_CONFIG_ADDR);
    return ((localVal & ((uint32_t)0x0001F000)) >> 12);
}

__INLINE void rpl_rssi_preload_cnt_setf(uint8_t rssipreloadcnt)
{
    ASSERT_ERR((((uint32_t)rssipreloadcnt << 12) & ~((uint32_t)0x0001F000)) == 0);
    REG_RPL_WR(RPL_AGC_RSSI_CONFIG_ADDR, (REG_RPL_RD(RPL_AGC_RSSI_CONFIG_ADDR) & ~((uint32_t)0x0001F000)) | ((uint32_t)rssipreloadcnt << 12));
}

__INLINE uint8_t rpl_rssi_on_dlyd_cnt_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_AGC_RSSI_CONFIG_ADDR);
    return ((localVal & ((uint32_t)0x00000F00)) >> 8);
}

__INLINE void rpl_rssi_on_dlyd_cnt_setf(uint8_t rssiondlydcnt)
{
    ASSERT_ERR((((uint32_t)rssiondlydcnt << 8) & ~((uint32_t)0x00000F00)) == 0);
    REG_RPL_WR(RPL_AGC_RSSI_CONFIG_ADDR, (REG_RPL_RD(RPL_AGC_RSSI_CONFIG_ADDR) & ~((uint32_t)0x00000F00)) | ((uint32_t)rssiondlydcnt << 8));
}

__INLINE uint8_t rpl_agc_settle_time_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_AGC_RSSI_CONFIG_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

__INLINE void rpl_agc_settle_time_setf(uint8_t agcsettletime)
{
    ASSERT_ERR((((uint32_t)agcsettletime << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_RPL_WR(RPL_AGC_RSSI_CONFIG_ADDR, (REG_RPL_RD(RPL_AGC_RSSI_CONFIG_ADDR) & ~((uint32_t)0x000000FF)) | ((uint32_t)agcsettletime << 0));
}

/**
 * @brief AGC_STAT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  02:00           AGC_RXGAIN   0x0
 * </pre>
 */
#define RPL_AGC_STAT_ADDR   0x00000040
#define RPL_AGC_STAT_OFFSET 0x00000040
#define RPL_AGC_STAT_INDEX  0x00000040
#define RPL_AGC_STAT_RESET  0x00000000

__INLINE uint32_t rpl_agc_stat_get(void)
{
    return REG_RPL_RD(RPL_AGC_STAT_ADDR);
}

// field definitions
#define RPL_AGC_RXGAIN_MASK   ((uint32_t)0x00000007)
#define RPL_AGC_RXGAIN_LSB    0
#define RPL_AGC_RXGAIN_WIDTH  ((uint32_t)0x00000003)

#define RPL_AGC_RXGAIN_RST    0x0

__INLINE uint8_t rpl_agc_rxgain_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_AGC_STAT_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x00000007)) == 0);
    return (localVal >> 0);
}

/**
 * @brief RFRXGAINTABLE register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  09:08           RXLNA_GAIN   0x3
 *  05:04           RXMIX_GAIN   0x3
 *  02:00        RXFILTER_GAIN   0x7
 * </pre>
 */
#define RPL_RFRXGAINTABLE_ADDR   0x00000041
#define RPL_RFRXGAINTABLE_OFFSET 0x00000041
#define RPL_RFRXGAINTABLE_INDEX  0x00000041
#define RPL_RFRXGAINTABLE_RESET  0x00000337
#define RPL_RFRXGAINTABLE_COUNT  7

__INLINE uint32_t rpl_rfrxgaintable_get(int reg_idx)
{
    ASSERT_ERR(reg_idx <= 6);
    return REG_RPL_RD(RPL_RFRXGAINTABLE_ADDR + reg_idx * 1);
}

__INLINE void rpl_rfrxgaintable_set(int reg_idx, uint32_t value)
{
    ASSERT_ERR(reg_idx <= 6);
    REG_RPL_WR(RPL_RFRXGAINTABLE_ADDR + reg_idx * 1, value);
}

// field definitions
#define RPL_RXLNA_GAIN_MASK      ((uint32_t)0x00000300)
#define RPL_RXLNA_GAIN_LSB       8
#define RPL_RXLNA_GAIN_WIDTH     ((uint32_t)0x00000002)
#define RPL_RXMIX_GAIN_MASK      ((uint32_t)0x00000030)
#define RPL_RXMIX_GAIN_LSB       4
#define RPL_RXMIX_GAIN_WIDTH     ((uint32_t)0x00000002)
#define RPL_RXFILTER_GAIN_MASK   ((uint32_t)0x00000007)
#define RPL_RXFILTER_GAIN_LSB    0
#define RPL_RXFILTER_GAIN_WIDTH  ((uint32_t)0x00000003)

#define RPL_RXLNA_GAIN_RST       0x3
#define RPL_RXMIX_GAIN_RST       0x3
#define RPL_RXFILTER_GAIN_RST    0x7

__INLINE void rpl_rfrxgaintable_pack(int reg_idx, uint8_t rxlnagain, uint8_t rxmixgain, uint8_t rxfiltergain)
{
    ASSERT_ERR(reg_idx <= 6);
    ASSERT_ERR((((uint32_t)rxlnagain << 8) & ~((uint32_t)0x00000300)) == 0);
    ASSERT_ERR((((uint32_t)rxmixgain << 4) & ~((uint32_t)0x00000030)) == 0);
    ASSERT_ERR((((uint32_t)rxfiltergain << 0) & ~((uint32_t)0x00000007)) == 0);
    REG_RPL_WR(RPL_RFRXGAINTABLE_ADDR + reg_idx * 1, ((uint32_t)rxlnagain << 8) | ((uint32_t)rxmixgain << 4) | ((uint32_t)rxfiltergain << 0));
}

__INLINE void rpl_rfrxgaintable_unpack(int reg_idx, uint8_t *rxlnagain, uint8_t *rxmixgain, uint8_t *rxfiltergain)
{
    ASSERT_ERR(reg_idx <= 6);
    uint32_t localVal = REG_RPL_RD(RPL_RFRXGAINTABLE_ADDR + reg_idx * 1);

    *rxlnagain = (localVal & ((uint32_t)0x00000300)) >> 8;
    *rxmixgain = (localVal & ((uint32_t)0x00000030)) >> 4;
    *rxfiltergain = (localVal & ((uint32_t)0x00000007)) >> 0;
}

__INLINE uint8_t rpl_rxlna_gain_getf(int reg_idx)
{
    ASSERT_ERR(reg_idx <= 6);
    uint32_t localVal = REG_RPL_RD(RPL_RFRXGAINTABLE_ADDR + reg_idx * 1);
    return ((localVal & ((uint32_t)0x00000300)) >> 8);
}

__INLINE void rpl_rxlna_gain_setf(int reg_idx, uint8_t rxlnagain)
{
    ASSERT_ERR(reg_idx <= 6);
    ASSERT_ERR((((uint32_t)rxlnagain << 8) & ~((uint32_t)0x00000300)) == 0);
    REG_RPL_WR(RPL_RFRXGAINTABLE_ADDR + reg_idx * 1, (REG_RPL_RD(RPL_RFRXGAINTABLE_ADDR + reg_idx * 1) & ~((uint32_t)0x00000300)) | ((uint32_t)rxlnagain << 8));
}

__INLINE uint8_t rpl_rxmix_gain_getf(int reg_idx)
{
    ASSERT_ERR(reg_idx <= 6);
    uint32_t localVal = REG_RPL_RD(RPL_RFRXGAINTABLE_ADDR + reg_idx * 1);
    return ((localVal & ((uint32_t)0x00000030)) >> 4);
}

__INLINE void rpl_rxmix_gain_setf(int reg_idx, uint8_t rxmixgain)
{
    ASSERT_ERR(reg_idx <= 6);
    ASSERT_ERR((((uint32_t)rxmixgain << 4) & ~((uint32_t)0x00000030)) == 0);
    REG_RPL_WR(RPL_RFRXGAINTABLE_ADDR + reg_idx * 1, (REG_RPL_RD(RPL_RFRXGAINTABLE_ADDR + reg_idx * 1) & ~((uint32_t)0x00000030)) | ((uint32_t)rxmixgain << 4));
}

__INLINE uint8_t rpl_rxfilter_gain_getf(int reg_idx)
{
    ASSERT_ERR(reg_idx <= 6);
    uint32_t localVal = REG_RPL_RD(RPL_RFRXGAINTABLE_ADDR + reg_idx * 1);
    return ((localVal & ((uint32_t)0x00000007)) >> 0);
}

__INLINE void rpl_rxfilter_gain_setf(int reg_idx, uint8_t rxfiltergain)
{
    ASSERT_ERR(reg_idx <= 6);
    ASSERT_ERR((((uint32_t)rxfiltergain << 0) & ~((uint32_t)0x00000007)) == 0);
    REG_RPL_WR(RPL_RFRXGAINTABLE_ADDR + reg_idx * 1, (REG_RPL_RD(RPL_RFRXGAINTABLE_ADDR + reg_idx * 1) & ~((uint32_t)0x00000007)) | ((uint32_t)rxfiltergain << 0));
}

/**
 * @brief RFPOWER_CNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  30:28        LDO_RXTX_TRIM   0x0
 *  27:25        LDO_VDIG_TRIM   0x0
 *  24:22        LDO_ADDA_TRIM   0x0
 *  21:19         LDO_VCO_TRIM   0x0
 *  18:16          LDO_XO_TRIM   0x0
 *  15:13         LDO_PLL_TRIM   0x0
 *  12:10        LDO_BGAP_TRIM   0x0
 *     09     LDO_RXTX_LP_MODE   0
 *     08     LDA_ADDA_LP_MODE   0
 *     07      LDO_VCO_LP_MODE   0
 *     06      LDO_PLL_LP_MODE   0
 *     05       LDO_XO_LP_MODE   0
 *     04          LDO_RXTX_ON   0
 *     03          LDO_ADDA_ON   0
 *     02           LDO_VCO_ON   0
 *     01           LDO_PLL_ON   0
 *     00            LDO_XO_ON   1
 * </pre>
 */
#define RPL_RFPOWER_CNTL_ADDR   0x00000048
#define RPL_RFPOWER_CNTL_OFFSET 0x00000048
#define RPL_RFPOWER_CNTL_INDEX  0x00000048
#define RPL_RFPOWER_CNTL_RESET  0x00000001

__INLINE uint32_t rpl_rfpower_cntl_get(void)
{
    return REG_RPL_RD(RPL_RFPOWER_CNTL_ADDR);
}

__INLINE void rpl_rfpower_cntl_set(uint32_t value)
{
    REG_RPL_WR(RPL_RFPOWER_CNTL_ADDR, value);
}

// field definitions
#define RPL_LDO_RXTX_TRIM_MASK      ((uint32_t)0x70000000)
#define RPL_LDO_RXTX_TRIM_LSB       28
#define RPL_LDO_RXTX_TRIM_WIDTH     ((uint32_t)0x00000003)
#define RPL_LDO_VDIG_TRIM_MASK      ((uint32_t)0x0E000000)
#define RPL_LDO_VDIG_TRIM_LSB       25
#define RPL_LDO_VDIG_TRIM_WIDTH     ((uint32_t)0x00000003)
#define RPL_LDO_ADDA_TRIM_MASK      ((uint32_t)0x01C00000)
#define RPL_LDO_ADDA_TRIM_LSB       22
#define RPL_LDO_ADDA_TRIM_WIDTH     ((uint32_t)0x00000003)
#define RPL_LDO_VCO_TRIM_MASK       ((uint32_t)0x00380000)
#define RPL_LDO_VCO_TRIM_LSB        19
#define RPL_LDO_VCO_TRIM_WIDTH      ((uint32_t)0x00000003)
#define RPL_LDO_XO_TRIM_MASK        ((uint32_t)0x00070000)
#define RPL_LDO_XO_TRIM_LSB         16
#define RPL_LDO_XO_TRIM_WIDTH       ((uint32_t)0x00000003)
#define RPL_LDO_PLL_TRIM_MASK       ((uint32_t)0x0000E000)
#define RPL_LDO_PLL_TRIM_LSB        13
#define RPL_LDO_PLL_TRIM_WIDTH      ((uint32_t)0x00000003)
#define RPL_LDO_BGAP_TRIM_MASK      ((uint32_t)0x00001C00)
#define RPL_LDO_BGAP_TRIM_LSB       10
#define RPL_LDO_BGAP_TRIM_WIDTH     ((uint32_t)0x00000003)
#define RPL_LDO_RXTX_LP_MODE_BIT    ((uint32_t)0x00000200)
#define RPL_LDO_RXTX_LP_MODE_POS    9
#define RPL_LDA_ADDA_LP_MODE_BIT    ((uint32_t)0x00000100)
#define RPL_LDA_ADDA_LP_MODE_POS    8
#define RPL_LDO_VCO_LP_MODE_BIT     ((uint32_t)0x00000080)
#define RPL_LDO_VCO_LP_MODE_POS     7
#define RPL_LDO_PLL_LP_MODE_BIT     ((uint32_t)0x00000040)
#define RPL_LDO_PLL_LP_MODE_POS     6
#define RPL_LDO_XO_LP_MODE_BIT      ((uint32_t)0x00000020)
#define RPL_LDO_XO_LP_MODE_POS      5
#define RPL_LDO_RXTX_ON_BIT         ((uint32_t)0x00000010)
#define RPL_LDO_RXTX_ON_POS         4
#define RPL_LDO_ADDA_ON_BIT         ((uint32_t)0x00000008)
#define RPL_LDO_ADDA_ON_POS         3
#define RPL_LDO_VCO_ON_BIT          ((uint32_t)0x00000004)
#define RPL_LDO_VCO_ON_POS          2
#define RPL_LDO_PLL_ON_BIT          ((uint32_t)0x00000002)
#define RPL_LDO_PLL_ON_POS          1
#define RPL_LDO_XO_ON_BIT           ((uint32_t)0x00000001)
#define RPL_LDO_XO_ON_POS           0

#define RPL_LDO_RXTX_TRIM_RST       0x0
#define RPL_LDO_VDIG_TRIM_RST       0x0
#define RPL_LDO_ADDA_TRIM_RST       0x0
#define RPL_LDO_VCO_TRIM_RST        0x0
#define RPL_LDO_XO_TRIM_RST         0x0
#define RPL_LDO_PLL_TRIM_RST        0x0
#define RPL_LDO_BGAP_TRIM_RST       0x0
#define RPL_LDO_RXTX_LP_MODE_RST    0x0
#define RPL_LDA_ADDA_LP_MODE_RST    0x0
#define RPL_LDO_VCO_LP_MODE_RST     0x0
#define RPL_LDO_PLL_LP_MODE_RST     0x0
#define RPL_LDO_XO_LP_MODE_RST      0x0
#define RPL_LDO_RXTX_ON_RST         0x0
#define RPL_LDO_ADDA_ON_RST         0x0
#define RPL_LDO_VCO_ON_RST          0x0
#define RPL_LDO_PLL_ON_RST          0x0
#define RPL_LDO_XO_ON_RST           0x1

__INLINE void rpl_rfpower_cntl_pack(uint8_t ldorxtxtrim, uint8_t ldovdigtrim, uint8_t ldoaddatrim, uint8_t ldovcotrim, uint8_t ldoxotrim, uint8_t ldoplltrim, uint8_t ldobgaptrim, uint8_t ldorxtxlpmode, uint8_t ldaaddalpmode, uint8_t ldovcolpmode, uint8_t ldoplllpmode, uint8_t ldoxolpmode, uint8_t ldorxtxon, uint8_t ldoaddaon, uint8_t ldovcoon, uint8_t ldopllon, uint8_t ldoxoon)
{
    ASSERT_ERR((((uint32_t)ldorxtxtrim << 28) & ~((uint32_t)0x70000000)) == 0);
    ASSERT_ERR((((uint32_t)ldovdigtrim << 25) & ~((uint32_t)0x0E000000)) == 0);
    ASSERT_ERR((((uint32_t)ldoaddatrim << 22) & ~((uint32_t)0x01C00000)) == 0);
    ASSERT_ERR((((uint32_t)ldovcotrim << 19) & ~((uint32_t)0x00380000)) == 0);
    ASSERT_ERR((((uint32_t)ldoxotrim << 16) & ~((uint32_t)0x00070000)) == 0);
    ASSERT_ERR((((uint32_t)ldoplltrim << 13) & ~((uint32_t)0x0000E000)) == 0);
    ASSERT_ERR((((uint32_t)ldobgaptrim << 10) & ~((uint32_t)0x00001C00)) == 0);
    ASSERT_ERR((((uint32_t)ldorxtxlpmode << 9) & ~((uint32_t)0x00000200)) == 0);
    ASSERT_ERR((((uint32_t)ldaaddalpmode << 8) & ~((uint32_t)0x00000100)) == 0);
    ASSERT_ERR((((uint32_t)ldovcolpmode << 7) & ~((uint32_t)0x00000080)) == 0);
    ASSERT_ERR((((uint32_t)ldoplllpmode << 6) & ~((uint32_t)0x00000040)) == 0);
    ASSERT_ERR((((uint32_t)ldoxolpmode << 5) & ~((uint32_t)0x00000020)) == 0);
    ASSERT_ERR((((uint32_t)ldorxtxon << 4) & ~((uint32_t)0x00000010)) == 0);
    ASSERT_ERR((((uint32_t)ldoaddaon << 3) & ~((uint32_t)0x00000008)) == 0);
    ASSERT_ERR((((uint32_t)ldovcoon << 2) & ~((uint32_t)0x00000004)) == 0);
    ASSERT_ERR((((uint32_t)ldopllon << 1) & ~((uint32_t)0x00000002)) == 0);
    ASSERT_ERR((((uint32_t)ldoxoon << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_RPL_WR(RPL_RFPOWER_CNTL_ADDR, ((uint32_t)ldorxtxtrim << 28) | ((uint32_t)ldovdigtrim << 25) | ((uint32_t)ldoaddatrim << 22) | ((uint32_t)ldovcotrim << 19) | ((uint32_t)ldoxotrim << 16) | ((uint32_t)ldoplltrim << 13) | ((uint32_t)ldobgaptrim << 10) | ((uint32_t)ldorxtxlpmode << 9) | ((uint32_t)ldaaddalpmode << 8) | ((uint32_t)ldovcolpmode << 7) | ((uint32_t)ldoplllpmode << 6) | ((uint32_t)ldoxolpmode << 5) | ((uint32_t)ldorxtxon << 4) | ((uint32_t)ldoaddaon << 3) | ((uint32_t)ldovcoon << 2) | ((uint32_t)ldopllon << 1) | ((uint32_t)ldoxoon << 0));
}

__INLINE void rpl_rfpower_cntl_unpack(uint8_t *ldorxtxtrim, uint8_t *ldovdigtrim, uint8_t *ldoaddatrim, uint8_t *ldovcotrim, uint8_t *ldoxotrim, uint8_t *ldoplltrim, uint8_t *ldobgaptrim, uint8_t *ldorxtxlpmode, uint8_t *ldaaddalpmode, uint8_t *ldovcolpmode, uint8_t *ldoplllpmode, uint8_t *ldoxolpmode, uint8_t *ldorxtxon, uint8_t *ldoaddaon, uint8_t *ldovcoon, uint8_t *ldopllon, uint8_t *ldoxoon)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFPOWER_CNTL_ADDR);

    *ldorxtxtrim = (localVal & ((uint32_t)0x70000000)) >> 28;
    *ldovdigtrim = (localVal & ((uint32_t)0x0E000000)) >> 25;
    *ldoaddatrim = (localVal & ((uint32_t)0x01C00000)) >> 22;
    *ldovcotrim = (localVal & ((uint32_t)0x00380000)) >> 19;
    *ldoxotrim = (localVal & ((uint32_t)0x00070000)) >> 16;
    *ldoplltrim = (localVal & ((uint32_t)0x0000E000)) >> 13;
    *ldobgaptrim = (localVal & ((uint32_t)0x00001C00)) >> 10;
    *ldorxtxlpmode = (localVal & ((uint32_t)0x00000200)) >> 9;
    *ldaaddalpmode = (localVal & ((uint32_t)0x00000100)) >> 8;
    *ldovcolpmode = (localVal & ((uint32_t)0x00000080)) >> 7;
    *ldoplllpmode = (localVal & ((uint32_t)0x00000040)) >> 6;
    *ldoxolpmode = (localVal & ((uint32_t)0x00000020)) >> 5;
    *ldorxtxon = (localVal & ((uint32_t)0x00000010)) >> 4;
    *ldoaddaon = (localVal & ((uint32_t)0x00000008)) >> 3;
    *ldovcoon = (localVal & ((uint32_t)0x00000004)) >> 2;
    *ldopllon = (localVal & ((uint32_t)0x00000002)) >> 1;
    *ldoxoon = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t rpl_ldo_rxtx_trim_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFPOWER_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x70000000)) >> 28);
}

__INLINE void rpl_ldo_rxtx_trim_setf(uint8_t ldorxtxtrim)
{
    ASSERT_ERR((((uint32_t)ldorxtxtrim << 28) & ~((uint32_t)0x70000000)) == 0);
    REG_RPL_WR(RPL_RFPOWER_CNTL_ADDR, (REG_RPL_RD(RPL_RFPOWER_CNTL_ADDR) & ~((uint32_t)0x70000000)) | ((uint32_t)ldorxtxtrim << 28));
}

__INLINE uint8_t rpl_ldo_vdig_trim_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFPOWER_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x0E000000)) >> 25);
}

__INLINE void rpl_ldo_vdig_trim_setf(uint8_t ldovdigtrim)
{
    ASSERT_ERR((((uint32_t)ldovdigtrim << 25) & ~((uint32_t)0x0E000000)) == 0);
    REG_RPL_WR(RPL_RFPOWER_CNTL_ADDR, (REG_RPL_RD(RPL_RFPOWER_CNTL_ADDR) & ~((uint32_t)0x0E000000)) | ((uint32_t)ldovdigtrim << 25));
}

__INLINE uint8_t rpl_ldo_adda_trim_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFPOWER_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x01C00000)) >> 22);
}

__INLINE void rpl_ldo_adda_trim_setf(uint8_t ldoaddatrim)
{
    ASSERT_ERR((((uint32_t)ldoaddatrim << 22) & ~((uint32_t)0x01C00000)) == 0);
    REG_RPL_WR(RPL_RFPOWER_CNTL_ADDR, (REG_RPL_RD(RPL_RFPOWER_CNTL_ADDR) & ~((uint32_t)0x01C00000)) | ((uint32_t)ldoaddatrim << 22));
}

__INLINE uint8_t rpl_ldo_vco_trim_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFPOWER_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x00380000)) >> 19);
}

__INLINE void rpl_ldo_vco_trim_setf(uint8_t ldovcotrim)
{
    ASSERT_ERR((((uint32_t)ldovcotrim << 19) & ~((uint32_t)0x00380000)) == 0);
    REG_RPL_WR(RPL_RFPOWER_CNTL_ADDR, (REG_RPL_RD(RPL_RFPOWER_CNTL_ADDR) & ~((uint32_t)0x00380000)) | ((uint32_t)ldovcotrim << 19));
}

__INLINE uint8_t rpl_ldo_xo_trim_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFPOWER_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x00070000)) >> 16);
}

__INLINE void rpl_ldo_xo_trim_setf(uint8_t ldoxotrim)
{
    ASSERT_ERR((((uint32_t)ldoxotrim << 16) & ~((uint32_t)0x00070000)) == 0);
    REG_RPL_WR(RPL_RFPOWER_CNTL_ADDR, (REG_RPL_RD(RPL_RFPOWER_CNTL_ADDR) & ~((uint32_t)0x00070000)) | ((uint32_t)ldoxotrim << 16));
}

__INLINE uint8_t rpl_ldo_pll_trim_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFPOWER_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x0000E000)) >> 13);
}

__INLINE void rpl_ldo_pll_trim_setf(uint8_t ldoplltrim)
{
    ASSERT_ERR((((uint32_t)ldoplltrim << 13) & ~((uint32_t)0x0000E000)) == 0);
    REG_RPL_WR(RPL_RFPOWER_CNTL_ADDR, (REG_RPL_RD(RPL_RFPOWER_CNTL_ADDR) & ~((uint32_t)0x0000E000)) | ((uint32_t)ldoplltrim << 13));
}

__INLINE uint8_t rpl_ldo_bgap_trim_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFPOWER_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x00001C00)) >> 10);
}

__INLINE void rpl_ldo_bgap_trim_setf(uint8_t ldobgaptrim)
{
    ASSERT_ERR((((uint32_t)ldobgaptrim << 10) & ~((uint32_t)0x00001C00)) == 0);
    REG_RPL_WR(RPL_RFPOWER_CNTL_ADDR, (REG_RPL_RD(RPL_RFPOWER_CNTL_ADDR) & ~((uint32_t)0x00001C00)) | ((uint32_t)ldobgaptrim << 10));
}

__INLINE uint8_t rpl_ldo_rxtx_lp_mode_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFPOWER_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000200)) >> 9);
}

__INLINE void rpl_ldo_rxtx_lp_mode_setf(uint8_t ldorxtxlpmode)
{
    ASSERT_ERR((((uint32_t)ldorxtxlpmode << 9) & ~((uint32_t)0x00000200)) == 0);
    REG_RPL_WR(RPL_RFPOWER_CNTL_ADDR, (REG_RPL_RD(RPL_RFPOWER_CNTL_ADDR) & ~((uint32_t)0x00000200)) | ((uint32_t)ldorxtxlpmode << 9));
}

__INLINE uint8_t rpl_lda_adda_lp_mode_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFPOWER_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000100)) >> 8);
}

__INLINE void rpl_lda_adda_lp_mode_setf(uint8_t ldaaddalpmode)
{
    ASSERT_ERR((((uint32_t)ldaaddalpmode << 8) & ~((uint32_t)0x00000100)) == 0);
    REG_RPL_WR(RPL_RFPOWER_CNTL_ADDR, (REG_RPL_RD(RPL_RFPOWER_CNTL_ADDR) & ~((uint32_t)0x00000100)) | ((uint32_t)ldaaddalpmode << 8));
}

__INLINE uint8_t rpl_ldo_vco_lp_mode_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFPOWER_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000080)) >> 7);
}

__INLINE void rpl_ldo_vco_lp_mode_setf(uint8_t ldovcolpmode)
{
    ASSERT_ERR((((uint32_t)ldovcolpmode << 7) & ~((uint32_t)0x00000080)) == 0);
    REG_RPL_WR(RPL_RFPOWER_CNTL_ADDR, (REG_RPL_RD(RPL_RFPOWER_CNTL_ADDR) & ~((uint32_t)0x00000080)) | ((uint32_t)ldovcolpmode << 7));
}

__INLINE uint8_t rpl_ldo_pll_lp_mode_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFPOWER_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000040)) >> 6);
}

__INLINE void rpl_ldo_pll_lp_mode_setf(uint8_t ldoplllpmode)
{
    ASSERT_ERR((((uint32_t)ldoplllpmode << 6) & ~((uint32_t)0x00000040)) == 0);
    REG_RPL_WR(RPL_RFPOWER_CNTL_ADDR, (REG_RPL_RD(RPL_RFPOWER_CNTL_ADDR) & ~((uint32_t)0x00000040)) | ((uint32_t)ldoplllpmode << 6));
}

__INLINE uint8_t rpl_ldo_xo_lp_mode_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFPOWER_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000020)) >> 5);
}

__INLINE void rpl_ldo_xo_lp_mode_setf(uint8_t ldoxolpmode)
{
    ASSERT_ERR((((uint32_t)ldoxolpmode << 5) & ~((uint32_t)0x00000020)) == 0);
    REG_RPL_WR(RPL_RFPOWER_CNTL_ADDR, (REG_RPL_RD(RPL_RFPOWER_CNTL_ADDR) & ~((uint32_t)0x00000020)) | ((uint32_t)ldoxolpmode << 5));
}

__INLINE uint8_t rpl_ldo_rxtx_on_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFPOWER_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE void rpl_ldo_rxtx_on_setf(uint8_t ldorxtxon)
{
    ASSERT_ERR((((uint32_t)ldorxtxon << 4) & ~((uint32_t)0x00000010)) == 0);
    REG_RPL_WR(RPL_RFPOWER_CNTL_ADDR, (REG_RPL_RD(RPL_RFPOWER_CNTL_ADDR) & ~((uint32_t)0x00000010)) | ((uint32_t)ldorxtxon << 4));
}

__INLINE uint8_t rpl_ldo_adda_on_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFPOWER_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

__INLINE void rpl_ldo_adda_on_setf(uint8_t ldoaddaon)
{
    ASSERT_ERR((((uint32_t)ldoaddaon << 3) & ~((uint32_t)0x00000008)) == 0);
    REG_RPL_WR(RPL_RFPOWER_CNTL_ADDR, (REG_RPL_RD(RPL_RFPOWER_CNTL_ADDR) & ~((uint32_t)0x00000008)) | ((uint32_t)ldoaddaon << 3));
}

__INLINE uint8_t rpl_ldo_vco_on_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFPOWER_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE void rpl_ldo_vco_on_setf(uint8_t ldovcoon)
{
    ASSERT_ERR((((uint32_t)ldovcoon << 2) & ~((uint32_t)0x00000004)) == 0);
    REG_RPL_WR(RPL_RFPOWER_CNTL_ADDR, (REG_RPL_RD(RPL_RFPOWER_CNTL_ADDR) & ~((uint32_t)0x00000004)) | ((uint32_t)ldovcoon << 2));
}

__INLINE uint8_t rpl_ldo_pll_on_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFPOWER_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE void rpl_ldo_pll_on_setf(uint8_t ldopllon)
{
    ASSERT_ERR((((uint32_t)ldopllon << 1) & ~((uint32_t)0x00000002)) == 0);
    REG_RPL_WR(RPL_RFPOWER_CNTL_ADDR, (REG_RPL_RD(RPL_RFPOWER_CNTL_ADDR) & ~((uint32_t)0x00000002)) | ((uint32_t)ldopllon << 1));
}

__INLINE uint8_t rpl_ldo_xo_on_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFPOWER_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void rpl_ldo_xo_on_setf(uint8_t ldoxoon)
{
    ASSERT_ERR((((uint32_t)ldoxoon << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_RPL_WR(RPL_RFPOWER_CNTL_ADDR, (REG_RPL_RD(RPL_RFPOWER_CNTL_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)ldoxoon << 0));
}

/**
 * @brief RAMPGENCNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:30             EPC_MODE   0x0
 *  28:20       EDR_SWITCH_CNT   0xD9
 *  15:08             GAIN_DEC   0x1
 *  07:00            GAIN_INCR   0x1
 * </pre>
 */
#define RPL_RAMPGENCNTL_ADDR   0x00000049
#define RPL_RAMPGENCNTL_OFFSET 0x00000049
#define RPL_RAMPGENCNTL_INDEX  0x00000049
#define RPL_RAMPGENCNTL_RESET  0x0D900101

__INLINE uint32_t rpl_rampgencntl_get(void)
{
    return REG_RPL_RD(RPL_RAMPGENCNTL_ADDR);
}

__INLINE void rpl_rampgencntl_set(uint32_t value)
{
    REG_RPL_WR(RPL_RAMPGENCNTL_ADDR, value);
}

// field definitions
#define RPL_EPC_MODE_MASK         ((uint32_t)0xC0000000)
#define RPL_EPC_MODE_LSB          30
#define RPL_EPC_MODE_WIDTH        ((uint32_t)0x00000002)
#define RPL_EDR_SWITCH_CNT_MASK   ((uint32_t)0x1FF00000)
#define RPL_EDR_SWITCH_CNT_LSB    20
#define RPL_EDR_SWITCH_CNT_WIDTH  ((uint32_t)0x00000009)
#define RPL_GAIN_DEC_MASK         ((uint32_t)0x0000FF00)
#define RPL_GAIN_DEC_LSB          8
#define RPL_GAIN_DEC_WIDTH        ((uint32_t)0x00000008)
#define RPL_GAIN_INCR_MASK        ((uint32_t)0x000000FF)
#define RPL_GAIN_INCR_LSB         0
#define RPL_GAIN_INCR_WIDTH       ((uint32_t)0x00000008)

#define RPL_EPC_MODE_RST          0x0
#define RPL_EDR_SWITCH_CNT_RST    0xD9
#define RPL_GAIN_DEC_RST          0x1
#define RPL_GAIN_INCR_RST         0x1

__INLINE void rpl_rampgencntl_pack(uint8_t epcmode, uint16_t edrswitchcnt, uint8_t gaindec, uint8_t gainincr)
{
    ASSERT_ERR((((uint32_t)epcmode << 30) & ~((uint32_t)0xC0000000)) == 0);
    ASSERT_ERR((((uint32_t)edrswitchcnt << 20) & ~((uint32_t)0x1FF00000)) == 0);
    ASSERT_ERR((((uint32_t)gaindec << 8) & ~((uint32_t)0x0000FF00)) == 0);
    ASSERT_ERR((((uint32_t)gainincr << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_RPL_WR(RPL_RAMPGENCNTL_ADDR, ((uint32_t)epcmode << 30) | ((uint32_t)edrswitchcnt << 20) | ((uint32_t)gaindec << 8) | ((uint32_t)gainincr << 0));
}

__INLINE void rpl_rampgencntl_unpack(uint8_t *epcmode, uint16_t *edrswitchcnt, uint8_t *gaindec, uint8_t *gainincr)
{
    uint32_t localVal = REG_RPL_RD(RPL_RAMPGENCNTL_ADDR);

    *epcmode = (localVal & ((uint32_t)0xC0000000)) >> 30;
    *edrswitchcnt = (localVal & ((uint32_t)0x1FF00000)) >> 20;
    *gaindec = (localVal & ((uint32_t)0x0000FF00)) >> 8;
    *gainincr = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint8_t rpl_epc_mode_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RAMPGENCNTL_ADDR);
    return ((localVal & ((uint32_t)0xC0000000)) >> 30);
}

__INLINE void rpl_epc_mode_setf(uint8_t epcmode)
{
    ASSERT_ERR((((uint32_t)epcmode << 30) & ~((uint32_t)0xC0000000)) == 0);
    REG_RPL_WR(RPL_RAMPGENCNTL_ADDR, (REG_RPL_RD(RPL_RAMPGENCNTL_ADDR) & ~((uint32_t)0xC0000000)) | ((uint32_t)epcmode << 30));
}

__INLINE uint16_t rpl_edr_switch_cnt_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RAMPGENCNTL_ADDR);
    return ((localVal & ((uint32_t)0x1FF00000)) >> 20);
}

__INLINE void rpl_edr_switch_cnt_setf(uint16_t edrswitchcnt)
{
    ASSERT_ERR((((uint32_t)edrswitchcnt << 20) & ~((uint32_t)0x1FF00000)) == 0);
    REG_RPL_WR(RPL_RAMPGENCNTL_ADDR, (REG_RPL_RD(RPL_RAMPGENCNTL_ADDR) & ~((uint32_t)0x1FF00000)) | ((uint32_t)edrswitchcnt << 20));
}

__INLINE uint8_t rpl_gain_dec_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RAMPGENCNTL_ADDR);
    return ((localVal & ((uint32_t)0x0000FF00)) >> 8);
}

__INLINE void rpl_gain_dec_setf(uint8_t gaindec)
{
    ASSERT_ERR((((uint32_t)gaindec << 8) & ~((uint32_t)0x0000FF00)) == 0);
    REG_RPL_WR(RPL_RAMPGENCNTL_ADDR, (REG_RPL_RD(RPL_RAMPGENCNTL_ADDR) & ~((uint32_t)0x0000FF00)) | ((uint32_t)gaindec << 8));
}

__INLINE uint8_t rpl_gain_incr_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RAMPGENCNTL_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

__INLINE void rpl_gain_incr_setf(uint8_t gainincr)
{
    ASSERT_ERR((((uint32_t)gainincr << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_RPL_WR(RPL_RAMPGENCNTL_ADDR, (REG_RPL_RD(RPL_RAMPGENCNTL_ADDR) & ~((uint32_t)0x000000FF)) | ((uint32_t)gainincr << 0));
}

/**
 * @brief RFTX_GFSK_GAIN_TABLE register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  10:08   TX_GFSK_PAINT_GAIN   0x0
 *  06:04     TX_GFSK_DRV_GAIN   0x1
 *  02:00   TX_GFSK_FILTER_GAIN   0x3
 * </pre>
 */
#define RPL_RFTX_GFSK_GAIN_TABLE_ADDR   0x0000004A
#define RPL_RFTX_GFSK_GAIN_TABLE_OFFSET 0x0000004A
#define RPL_RFTX_GFSK_GAIN_TABLE_INDEX  0x0000004A
#define RPL_RFTX_GFSK_GAIN_TABLE_RESET  0x00000013
#define RPL_RFTX_GFSK_GAIN_TABLE_COUNT  16

__INLINE uint32_t rpl_rftx_gfsk_gain_table_get(int reg_idx)
{
    ASSERT_ERR(reg_idx <= 15);
    return REG_RPL_RD(RPL_RFTX_GFSK_GAIN_TABLE_ADDR + reg_idx * 1);
}

__INLINE void rpl_rftx_gfsk_gain_table_set(int reg_idx, uint32_t value)
{
    ASSERT_ERR(reg_idx <= 15);
    REG_RPL_WR(RPL_RFTX_GFSK_GAIN_TABLE_ADDR + reg_idx * 1, value);
}

// field definitions
#define RPL_TX_GFSK_PAINT_GAIN_MASK    ((uint32_t)0x00000700)
#define RPL_TX_GFSK_PAINT_GAIN_LSB     8
#define RPL_TX_GFSK_PAINT_GAIN_WIDTH   ((uint32_t)0x00000003)
#define RPL_TX_GFSK_DRV_GAIN_MASK      ((uint32_t)0x00000070)
#define RPL_TX_GFSK_DRV_GAIN_LSB       4
#define RPL_TX_GFSK_DRV_GAIN_WIDTH     ((uint32_t)0x00000003)
#define RPL_TX_GFSK_FILTER_GAIN_MASK   ((uint32_t)0x00000007)
#define RPL_TX_GFSK_FILTER_GAIN_LSB    0
#define RPL_TX_GFSK_FILTER_GAIN_WIDTH  ((uint32_t)0x00000003)

#define RPL_TX_GFSK_PAINT_GAIN_RST     0x0
#define RPL_TX_GFSK_DRV_GAIN_RST       0x1
#define RPL_TX_GFSK_FILTER_GAIN_RST    0x3

__INLINE void rpl_rftx_gfsk_gain_table_pack(int reg_idx, uint8_t txgfskpaintgain, uint8_t txgfskdrvgain, uint8_t txgfskfiltergain)
{
    ASSERT_ERR(reg_idx <= 15);
    ASSERT_ERR((((uint32_t)txgfskpaintgain << 8) & ~((uint32_t)0x00000700)) == 0);
    ASSERT_ERR((((uint32_t)txgfskdrvgain << 4) & ~((uint32_t)0x00000070)) == 0);
    ASSERT_ERR((((uint32_t)txgfskfiltergain << 0) & ~((uint32_t)0x00000007)) == 0);
    REG_RPL_WR(RPL_RFTX_GFSK_GAIN_TABLE_ADDR + reg_idx * 1, ((uint32_t)txgfskpaintgain << 8) | ((uint32_t)txgfskdrvgain << 4) | ((uint32_t)txgfskfiltergain << 0));
}

__INLINE void rpl_rftx_gfsk_gain_table_unpack(int reg_idx, uint8_t *txgfskpaintgain, uint8_t *txgfskdrvgain, uint8_t *txgfskfiltergain)
{
    ASSERT_ERR(reg_idx <= 15);
    uint32_t localVal = REG_RPL_RD(RPL_RFTX_GFSK_GAIN_TABLE_ADDR + reg_idx * 1);

    *txgfskpaintgain = (localVal & ((uint32_t)0x00000700)) >> 8;
    *txgfskdrvgain = (localVal & ((uint32_t)0x00000070)) >> 4;
    *txgfskfiltergain = (localVal & ((uint32_t)0x00000007)) >> 0;
}

__INLINE uint8_t rpl_tx_gfsk_paint_gain_getf(int reg_idx)
{
    ASSERT_ERR(reg_idx <= 15);
    uint32_t localVal = REG_RPL_RD(RPL_RFTX_GFSK_GAIN_TABLE_ADDR + reg_idx * 1);
    return ((localVal & ((uint32_t)0x00000700)) >> 8);
}

__INLINE void rpl_tx_gfsk_paint_gain_setf(int reg_idx, uint8_t txgfskpaintgain)
{
    ASSERT_ERR(reg_idx <= 15);
    ASSERT_ERR((((uint32_t)txgfskpaintgain << 8) & ~((uint32_t)0x00000700)) == 0);
    REG_RPL_WR(RPL_RFTX_GFSK_GAIN_TABLE_ADDR + reg_idx * 1, (REG_RPL_RD(RPL_RFTX_GFSK_GAIN_TABLE_ADDR + reg_idx * 1) & ~((uint32_t)0x00000700)) | ((uint32_t)txgfskpaintgain << 8));
}

__INLINE uint8_t rpl_tx_gfsk_drv_gain_getf(int reg_idx)
{
    ASSERT_ERR(reg_idx <= 15);
    uint32_t localVal = REG_RPL_RD(RPL_RFTX_GFSK_GAIN_TABLE_ADDR + reg_idx * 1);
    return ((localVal & ((uint32_t)0x00000070)) >> 4);
}

__INLINE void rpl_tx_gfsk_drv_gain_setf(int reg_idx, uint8_t txgfskdrvgain)
{
    ASSERT_ERR(reg_idx <= 15);
    ASSERT_ERR((((uint32_t)txgfskdrvgain << 4) & ~((uint32_t)0x00000070)) == 0);
    REG_RPL_WR(RPL_RFTX_GFSK_GAIN_TABLE_ADDR + reg_idx * 1, (REG_RPL_RD(RPL_RFTX_GFSK_GAIN_TABLE_ADDR + reg_idx * 1) & ~((uint32_t)0x00000070)) | ((uint32_t)txgfskdrvgain << 4));
}

__INLINE uint8_t rpl_tx_gfsk_filter_gain_getf(int reg_idx)
{
    ASSERT_ERR(reg_idx <= 15);
    uint32_t localVal = REG_RPL_RD(RPL_RFTX_GFSK_GAIN_TABLE_ADDR + reg_idx * 1);
    return ((localVal & ((uint32_t)0x00000007)) >> 0);
}

__INLINE void rpl_tx_gfsk_filter_gain_setf(int reg_idx, uint8_t txgfskfiltergain)
{
    ASSERT_ERR(reg_idx <= 15);
    ASSERT_ERR((((uint32_t)txgfskfiltergain << 0) & ~((uint32_t)0x00000007)) == 0);
    REG_RPL_WR(RPL_RFTX_GFSK_GAIN_TABLE_ADDR + reg_idx * 1, (REG_RPL_RD(RPL_RFTX_GFSK_GAIN_TABLE_ADDR + reg_idx * 1) & ~((uint32_t)0x00000007)) | ((uint32_t)txgfskfiltergain << 0));
}

/**
 * @brief RFTX_EDR_GAIN_TABLE register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  22:20   TX_DQPSK_PAINT_GAIN   0x0
 *  18:16    TX_DQPSK_DRV_GAIN   0x1
 *  14:12   TX_DQPSK_FILTER_GAIN   0x3
 *  10:08   TX_8DPSK_PAINT_GAIN   0x0
 *  06:04    TX_8DPSK_DRV_GAIN   0x1
 *  02:00   TX_8DPSK_FILTER_GAIN   0x3
 * </pre>
 */
#define RPL_RFTX_EDR_GAIN_TABLE_ADDR   0x0000005A
#define RPL_RFTX_EDR_GAIN_TABLE_OFFSET 0x0000005A
#define RPL_RFTX_EDR_GAIN_TABLE_INDEX  0x0000005A
#define RPL_RFTX_EDR_GAIN_TABLE_RESET  0x00013013
#define RPL_RFTX_EDR_GAIN_TABLE_COUNT  16

__INLINE uint32_t rpl_rftx_edr_gain_table_get(int reg_idx)
{
    ASSERT_ERR(reg_idx <= 15);
    return REG_RPL_RD(RPL_RFTX_EDR_GAIN_TABLE_ADDR + reg_idx * 1);
}

__INLINE void rpl_rftx_edr_gain_table_set(int reg_idx, uint32_t value)
{
    ASSERT_ERR(reg_idx <= 15);
    REG_RPL_WR(RPL_RFTX_EDR_GAIN_TABLE_ADDR + reg_idx * 1, value);
}

// field definitions
#define RPL_TX_DQPSK_PAINT_GAIN_MASK    ((uint32_t)0x00700000)
#define RPL_TX_DQPSK_PAINT_GAIN_LSB     20
#define RPL_TX_DQPSK_PAINT_GAIN_WIDTH   ((uint32_t)0x00000003)
#define RPL_TX_DQPSK_DRV_GAIN_MASK      ((uint32_t)0x00070000)
#define RPL_TX_DQPSK_DRV_GAIN_LSB       16
#define RPL_TX_DQPSK_DRV_GAIN_WIDTH     ((uint32_t)0x00000003)
#define RPL_TX_DQPSK_FILTER_GAIN_MASK   ((uint32_t)0x00007000)
#define RPL_TX_DQPSK_FILTER_GAIN_LSB    12
#define RPL_TX_DQPSK_FILTER_GAIN_WIDTH  ((uint32_t)0x00000003)
#define RPL_TX_8DPSK_PAINT_GAIN_MASK    ((uint32_t)0x00000700)
#define RPL_TX_8DPSK_PAINT_GAIN_LSB     8
#define RPL_TX_8DPSK_PAINT_GAIN_WIDTH   ((uint32_t)0x00000003)
#define RPL_TX_8DPSK_DRV_GAIN_MASK      ((uint32_t)0x00000070)
#define RPL_TX_8DPSK_DRV_GAIN_LSB       4
#define RPL_TX_8DPSK_DRV_GAIN_WIDTH     ((uint32_t)0x00000003)
#define RPL_TX_8DPSK_FILTER_GAIN_MASK   ((uint32_t)0x00000007)
#define RPL_TX_8DPSK_FILTER_GAIN_LSB    0
#define RPL_TX_8DPSK_FILTER_GAIN_WIDTH  ((uint32_t)0x00000003)

#define RPL_TX_DQPSK_PAINT_GAIN_RST     0x0
#define RPL_TX_DQPSK_DRV_GAIN_RST       0x1
#define RPL_TX_DQPSK_FILTER_GAIN_RST    0x3
#define RPL_TX_8DPSK_PAINT_GAIN_RST     0x0
#define RPL_TX_8DPSK_DRV_GAIN_RST       0x1
#define RPL_TX_8DPSK_FILTER_GAIN_RST    0x3

__INLINE void rpl_rftx_edr_gain_table_pack(int reg_idx, uint8_t txdqpskpaintgain, uint8_t txdqpskdrvgain, uint8_t txdqpskfiltergain, uint8_t tx8dpskpaintgain, uint8_t tx8dpskdrvgain, uint8_t tx8dpskfiltergain)
{
    ASSERT_ERR(reg_idx <= 15);
    ASSERT_ERR((((uint32_t)txdqpskpaintgain << 20) & ~((uint32_t)0x00700000)) == 0);
    ASSERT_ERR((((uint32_t)txdqpskdrvgain << 16) & ~((uint32_t)0x00070000)) == 0);
    ASSERT_ERR((((uint32_t)txdqpskfiltergain << 12) & ~((uint32_t)0x00007000)) == 0);
    ASSERT_ERR((((uint32_t)tx8dpskpaintgain << 8) & ~((uint32_t)0x00000700)) == 0);
    ASSERT_ERR((((uint32_t)tx8dpskdrvgain << 4) & ~((uint32_t)0x00000070)) == 0);
    ASSERT_ERR((((uint32_t)tx8dpskfiltergain << 0) & ~((uint32_t)0x00000007)) == 0);
    REG_RPL_WR(RPL_RFTX_EDR_GAIN_TABLE_ADDR + reg_idx * 1, ((uint32_t)txdqpskpaintgain << 20) | ((uint32_t)txdqpskdrvgain << 16) | ((uint32_t)txdqpskfiltergain << 12) | ((uint32_t)tx8dpskpaintgain << 8) | ((uint32_t)tx8dpskdrvgain << 4) | ((uint32_t)tx8dpskfiltergain << 0));
}

__INLINE void rpl_rftx_edr_gain_table_unpack(int reg_idx, uint8_t *txdqpskpaintgain, uint8_t *txdqpskdrvgain, uint8_t *txdqpskfiltergain, uint8_t *tx8dpskpaintgain, uint8_t *tx8dpskdrvgain, uint8_t *tx8dpskfiltergain)
{
    ASSERT_ERR(reg_idx <= 15);
    uint32_t localVal = REG_RPL_RD(RPL_RFTX_EDR_GAIN_TABLE_ADDR + reg_idx * 1);

    *txdqpskpaintgain = (localVal & ((uint32_t)0x00700000)) >> 20;
    *txdqpskdrvgain = (localVal & ((uint32_t)0x00070000)) >> 16;
    *txdqpskfiltergain = (localVal & ((uint32_t)0x00007000)) >> 12;
    *tx8dpskpaintgain = (localVal & ((uint32_t)0x00000700)) >> 8;
    *tx8dpskdrvgain = (localVal & ((uint32_t)0x00000070)) >> 4;
    *tx8dpskfiltergain = (localVal & ((uint32_t)0x00000007)) >> 0;
}

__INLINE uint8_t rpl_tx_dqpsk_paint_gain_getf(int reg_idx)
{
    ASSERT_ERR(reg_idx <= 15);
    uint32_t localVal = REG_RPL_RD(RPL_RFTX_EDR_GAIN_TABLE_ADDR + reg_idx * 1);
    return ((localVal & ((uint32_t)0x00700000)) >> 20);
}

__INLINE void rpl_tx_dqpsk_paint_gain_setf(int reg_idx, uint8_t txdqpskpaintgain)
{
    ASSERT_ERR(reg_idx <= 15);
    ASSERT_ERR((((uint32_t)txdqpskpaintgain << 20) & ~((uint32_t)0x00700000)) == 0);
    REG_RPL_WR(RPL_RFTX_EDR_GAIN_TABLE_ADDR + reg_idx * 1, (REG_RPL_RD(RPL_RFTX_EDR_GAIN_TABLE_ADDR + reg_idx * 1) & ~((uint32_t)0x00700000)) | ((uint32_t)txdqpskpaintgain << 20));
}

__INLINE uint8_t rpl_tx_dqpsk_drv_gain_getf(int reg_idx)
{
    ASSERT_ERR(reg_idx <= 15);
    uint32_t localVal = REG_RPL_RD(RPL_RFTX_EDR_GAIN_TABLE_ADDR + reg_idx * 1);
    return ((localVal & ((uint32_t)0x00070000)) >> 16);
}

__INLINE void rpl_tx_dqpsk_drv_gain_setf(int reg_idx, uint8_t txdqpskdrvgain)
{
    ASSERT_ERR(reg_idx <= 15);
    ASSERT_ERR((((uint32_t)txdqpskdrvgain << 16) & ~((uint32_t)0x00070000)) == 0);
    REG_RPL_WR(RPL_RFTX_EDR_GAIN_TABLE_ADDR + reg_idx * 1, (REG_RPL_RD(RPL_RFTX_EDR_GAIN_TABLE_ADDR + reg_idx * 1) & ~((uint32_t)0x00070000)) | ((uint32_t)txdqpskdrvgain << 16));
}

__INLINE uint8_t rpl_tx_dqpsk_filter_gain_getf(int reg_idx)
{
    ASSERT_ERR(reg_idx <= 15);
    uint32_t localVal = REG_RPL_RD(RPL_RFTX_EDR_GAIN_TABLE_ADDR + reg_idx * 1);
    return ((localVal & ((uint32_t)0x00007000)) >> 12);
}

__INLINE void rpl_tx_dqpsk_filter_gain_setf(int reg_idx, uint8_t txdqpskfiltergain)
{
    ASSERT_ERR(reg_idx <= 15);
    ASSERT_ERR((((uint32_t)txdqpskfiltergain << 12) & ~((uint32_t)0x00007000)) == 0);
    REG_RPL_WR(RPL_RFTX_EDR_GAIN_TABLE_ADDR + reg_idx * 1, (REG_RPL_RD(RPL_RFTX_EDR_GAIN_TABLE_ADDR + reg_idx * 1) & ~((uint32_t)0x00007000)) | ((uint32_t)txdqpskfiltergain << 12));
}

__INLINE uint8_t rpl_tx_8dpsk_paint_gain_getf(int reg_idx)
{
    ASSERT_ERR(reg_idx <= 15);
    uint32_t localVal = REG_RPL_RD(RPL_RFTX_EDR_GAIN_TABLE_ADDR + reg_idx * 1);
    return ((localVal & ((uint32_t)0x00000700)) >> 8);
}

__INLINE void rpl_tx_8dpsk_paint_gain_setf(int reg_idx, uint8_t tx8dpskpaintgain)
{
    ASSERT_ERR(reg_idx <= 15);
    ASSERT_ERR((((uint32_t)tx8dpskpaintgain << 8) & ~((uint32_t)0x00000700)) == 0);
    REG_RPL_WR(RPL_RFTX_EDR_GAIN_TABLE_ADDR + reg_idx * 1, (REG_RPL_RD(RPL_RFTX_EDR_GAIN_TABLE_ADDR + reg_idx * 1) & ~((uint32_t)0x00000700)) | ((uint32_t)tx8dpskpaintgain << 8));
}

__INLINE uint8_t rpl_tx_8dpsk_drv_gain_getf(int reg_idx)
{
    ASSERT_ERR(reg_idx <= 15);
    uint32_t localVal = REG_RPL_RD(RPL_RFTX_EDR_GAIN_TABLE_ADDR + reg_idx * 1);
    return ((localVal & ((uint32_t)0x00000070)) >> 4);
}

__INLINE void rpl_tx_8dpsk_drv_gain_setf(int reg_idx, uint8_t tx8dpskdrvgain)
{
    ASSERT_ERR(reg_idx <= 15);
    ASSERT_ERR((((uint32_t)tx8dpskdrvgain << 4) & ~((uint32_t)0x00000070)) == 0);
    REG_RPL_WR(RPL_RFTX_EDR_GAIN_TABLE_ADDR + reg_idx * 1, (REG_RPL_RD(RPL_RFTX_EDR_GAIN_TABLE_ADDR + reg_idx * 1) & ~((uint32_t)0x00000070)) | ((uint32_t)tx8dpskdrvgain << 4));
}

__INLINE uint8_t rpl_tx_8dpsk_filter_gain_getf(int reg_idx)
{
    ASSERT_ERR(reg_idx <= 15);
    uint32_t localVal = REG_RPL_RD(RPL_RFTX_EDR_GAIN_TABLE_ADDR + reg_idx * 1);
    return ((localVal & ((uint32_t)0x00000007)) >> 0);
}

__INLINE void rpl_tx_8dpsk_filter_gain_setf(int reg_idx, uint8_t tx8dpskfiltergain)
{
    ASSERT_ERR(reg_idx <= 15);
    ASSERT_ERR((((uint32_t)tx8dpskfiltergain << 0) & ~((uint32_t)0x00000007)) == 0);
    REG_RPL_WR(RPL_RFTX_EDR_GAIN_TABLE_ADDR + reg_idx * 1, (REG_RPL_RD(RPL_RFTX_EDR_GAIN_TABLE_ADDR + reg_idx * 1) & ~((uint32_t)0x00000007)) | ((uint32_t)tx8dpskfiltergain << 0));
}

/**
 * @brief RFTX_DACIQ_GAIN register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  22:20    TXDACQ_8DPSK_GAIN   0x0
 *  18:16    TXDACI_8DPSK_GAIN   0x0
 *  14:12    TXDACQ_DQPSK_GAIN   0x0
 *  10:08    TXDACI_DQPSK_GAIN   0x0
 *  06:04     TXDACQ_GFSK_GAIN   0x0
 *  02:00     TXDACI_GFSK_GAIN   0x0
 * </pre>
 */
#define RPL_RFTX_DACIQ_GAIN_ADDR   0x0000006A
#define RPL_RFTX_DACIQ_GAIN_OFFSET 0x0000006A
#define RPL_RFTX_DACIQ_GAIN_INDEX  0x0000006A
#define RPL_RFTX_DACIQ_GAIN_RESET  0x00000000

__INLINE uint32_t rpl_rftx_daciq_gain_get(void)
{
    return REG_RPL_RD(RPL_RFTX_DACIQ_GAIN_ADDR);
}

__INLINE void rpl_rftx_daciq_gain_set(uint32_t value)
{
    REG_RPL_WR(RPL_RFTX_DACIQ_GAIN_ADDR, value);
}

// field definitions
#define RPL_TXDACQ_8DPSK_GAIN_MASK   ((uint32_t)0x00700000)
#define RPL_TXDACQ_8DPSK_GAIN_LSB    20
#define RPL_TXDACQ_8DPSK_GAIN_WIDTH  ((uint32_t)0x00000003)
#define RPL_TXDACI_8DPSK_GAIN_MASK   ((uint32_t)0x00070000)
#define RPL_TXDACI_8DPSK_GAIN_LSB    16
#define RPL_TXDACI_8DPSK_GAIN_WIDTH  ((uint32_t)0x00000003)
#define RPL_TXDACQ_DQPSK_GAIN_MASK   ((uint32_t)0x00007000)
#define RPL_TXDACQ_DQPSK_GAIN_LSB    12
#define RPL_TXDACQ_DQPSK_GAIN_WIDTH  ((uint32_t)0x00000003)
#define RPL_TXDACI_DQPSK_GAIN_MASK   ((uint32_t)0x00000700)
#define RPL_TXDACI_DQPSK_GAIN_LSB    8
#define RPL_TXDACI_DQPSK_GAIN_WIDTH  ((uint32_t)0x00000003)
#define RPL_TXDACQ_GFSK_GAIN_MASK    ((uint32_t)0x00000070)
#define RPL_TXDACQ_GFSK_GAIN_LSB     4
#define RPL_TXDACQ_GFSK_GAIN_WIDTH   ((uint32_t)0x00000003)
#define RPL_TXDACI_GFSK_GAIN_MASK    ((uint32_t)0x00000007)
#define RPL_TXDACI_GFSK_GAIN_LSB     0
#define RPL_TXDACI_GFSK_GAIN_WIDTH   ((uint32_t)0x00000003)

#define RPL_TXDACQ_8DPSK_GAIN_RST    0x0
#define RPL_TXDACI_8DPSK_GAIN_RST    0x0
#define RPL_TXDACQ_DQPSK_GAIN_RST    0x0
#define RPL_TXDACI_DQPSK_GAIN_RST    0x0
#define RPL_TXDACQ_GFSK_GAIN_RST     0x0
#define RPL_TXDACI_GFSK_GAIN_RST     0x0

__INLINE void rpl_rftx_daciq_gain_pack(uint8_t txdacq8dpskgain, uint8_t txdaci8dpskgain, uint8_t txdacqdqpskgain, uint8_t txdacidqpskgain, uint8_t txdacqgfskgain, uint8_t txdacigfskgain)
{
    ASSERT_ERR((((uint32_t)txdacq8dpskgain << 20) & ~((uint32_t)0x00700000)) == 0);
    ASSERT_ERR((((uint32_t)txdaci8dpskgain << 16) & ~((uint32_t)0x00070000)) == 0);
    ASSERT_ERR((((uint32_t)txdacqdqpskgain << 12) & ~((uint32_t)0x00007000)) == 0);
    ASSERT_ERR((((uint32_t)txdacidqpskgain << 8) & ~((uint32_t)0x00000700)) == 0);
    ASSERT_ERR((((uint32_t)txdacqgfskgain << 4) & ~((uint32_t)0x00000070)) == 0);
    ASSERT_ERR((((uint32_t)txdacigfskgain << 0) & ~((uint32_t)0x00000007)) == 0);
    REG_RPL_WR(RPL_RFTX_DACIQ_GAIN_ADDR, ((uint32_t)txdacq8dpskgain << 20) | ((uint32_t)txdaci8dpskgain << 16) | ((uint32_t)txdacqdqpskgain << 12) | ((uint32_t)txdacidqpskgain << 8) | ((uint32_t)txdacqgfskgain << 4) | ((uint32_t)txdacigfskgain << 0));
}

__INLINE void rpl_rftx_daciq_gain_unpack(uint8_t *txdacq8dpskgain, uint8_t *txdaci8dpskgain, uint8_t *txdacqdqpskgain, uint8_t *txdacidqpskgain, uint8_t *txdacqgfskgain, uint8_t *txdacigfskgain)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTX_DACIQ_GAIN_ADDR);

    *txdacq8dpskgain = (localVal & ((uint32_t)0x00700000)) >> 20;
    *txdaci8dpskgain = (localVal & ((uint32_t)0x00070000)) >> 16;
    *txdacqdqpskgain = (localVal & ((uint32_t)0x00007000)) >> 12;
    *txdacidqpskgain = (localVal & ((uint32_t)0x00000700)) >> 8;
    *txdacqgfskgain = (localVal & ((uint32_t)0x00000070)) >> 4;
    *txdacigfskgain = (localVal & ((uint32_t)0x00000007)) >> 0;
}

__INLINE uint8_t rpl_txdacq_8dpsk_gain_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTX_DACIQ_GAIN_ADDR);
    return ((localVal & ((uint32_t)0x00700000)) >> 20);
}

__INLINE void rpl_txdacq_8dpsk_gain_setf(uint8_t txdacq8dpskgain)
{
    ASSERT_ERR((((uint32_t)txdacq8dpskgain << 20) & ~((uint32_t)0x00700000)) == 0);
    REG_RPL_WR(RPL_RFTX_DACIQ_GAIN_ADDR, (REG_RPL_RD(RPL_RFTX_DACIQ_GAIN_ADDR) & ~((uint32_t)0x00700000)) | ((uint32_t)txdacq8dpskgain << 20));
}

__INLINE uint8_t rpl_txdaci_8dpsk_gain_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTX_DACIQ_GAIN_ADDR);
    return ((localVal & ((uint32_t)0x00070000)) >> 16);
}

__INLINE void rpl_txdaci_8dpsk_gain_setf(uint8_t txdaci8dpskgain)
{
    ASSERT_ERR((((uint32_t)txdaci8dpskgain << 16) & ~((uint32_t)0x00070000)) == 0);
    REG_RPL_WR(RPL_RFTX_DACIQ_GAIN_ADDR, (REG_RPL_RD(RPL_RFTX_DACIQ_GAIN_ADDR) & ~((uint32_t)0x00070000)) | ((uint32_t)txdaci8dpskgain << 16));
}

__INLINE uint8_t rpl_txdacq_dqpsk_gain_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTX_DACIQ_GAIN_ADDR);
    return ((localVal & ((uint32_t)0x00007000)) >> 12);
}

__INLINE void rpl_txdacq_dqpsk_gain_setf(uint8_t txdacqdqpskgain)
{
    ASSERT_ERR((((uint32_t)txdacqdqpskgain << 12) & ~((uint32_t)0x00007000)) == 0);
    REG_RPL_WR(RPL_RFTX_DACIQ_GAIN_ADDR, (REG_RPL_RD(RPL_RFTX_DACIQ_GAIN_ADDR) & ~((uint32_t)0x00007000)) | ((uint32_t)txdacqdqpskgain << 12));
}

__INLINE uint8_t rpl_txdaci_dqpsk_gain_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTX_DACIQ_GAIN_ADDR);
    return ((localVal & ((uint32_t)0x00000700)) >> 8);
}

__INLINE void rpl_txdaci_dqpsk_gain_setf(uint8_t txdacidqpskgain)
{
    ASSERT_ERR((((uint32_t)txdacidqpskgain << 8) & ~((uint32_t)0x00000700)) == 0);
    REG_RPL_WR(RPL_RFTX_DACIQ_GAIN_ADDR, (REG_RPL_RD(RPL_RFTX_DACIQ_GAIN_ADDR) & ~((uint32_t)0x00000700)) | ((uint32_t)txdacidqpskgain << 8));
}

__INLINE uint8_t rpl_txdacq_gfsk_gain_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTX_DACIQ_GAIN_ADDR);
    return ((localVal & ((uint32_t)0x00000070)) >> 4);
}

__INLINE void rpl_txdacq_gfsk_gain_setf(uint8_t txdacqgfskgain)
{
    ASSERT_ERR((((uint32_t)txdacqgfskgain << 4) & ~((uint32_t)0x00000070)) == 0);
    REG_RPL_WR(RPL_RFTX_DACIQ_GAIN_ADDR, (REG_RPL_RD(RPL_RFTX_DACIQ_GAIN_ADDR) & ~((uint32_t)0x00000070)) | ((uint32_t)txdacqgfskgain << 4));
}

__INLINE uint8_t rpl_txdaci_gfsk_gain_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTX_DACIQ_GAIN_ADDR);
    return ((localVal & ((uint32_t)0x00000007)) >> 0);
}

__INLINE void rpl_txdaci_gfsk_gain_setf(uint8_t txdacigfskgain)
{
    ASSERT_ERR((((uint32_t)txdacigfskgain << 0) & ~((uint32_t)0x00000007)) == 0);
    REG_RPL_WR(RPL_RFTX_DACIQ_GAIN_ADDR, (REG_RPL_RD(RPL_RFTX_DACIQ_GAIN_ADDR) & ~((uint32_t)0x00000007)) | ((uint32_t)txdacigfskgain << 0));
}

/**
 * @brief CRM_CNTL0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     31         FORCE_RESETS   0
 *     30    FORCE_CLOCK_GATES   0
 *     28      DAP_TXDC_RST_EN   0
 *     27     RPLL_FDIV_RST_EN   0
 *     26           MDM_RST_EN   0
 *     25            DB_RST_EN   0
 *     24           AGC_RST_EN   0
 *     23            SD_RST_EN   0
 *     22   GFSK_PREFILT_RST_EN   0
 *     21     DAP_CALIB_RST_EN   0
 *     20           DAP_RST_EN   0
 *     18   DAP_TXDC_CLK_GATE_EN   0
 *     17   RPLL_FDIV_CLK_GATE_EN   0
 *     16   SAR_ADC_CLK_GATE_EN   0
 *     15   MDM_TXDPSK13_CLK_GATE_EN   0
 *     14   MDM_TXGFSK13_CLK_GATE_EN   0
 *     13   MDM_TX13_CLK_GATE_EN   0
 *     12   MDM_RXDPSK13_CLK_GATE_EN   0
 *     11   MDM_RXDPSK52_CLK_GATE_EN   0
 *     10   MDM_RXDL13_CLK_GATE_EN   0
 *     09   MDM_RXGFSK13_CLK_GATE_EN   0
 *     08   MDM_RXGFSK52_CLK_GATE_EN   0
 *     07   MDM_RX52_CLK_GATE_EN   0
 *     06   MDM_CORE13_CLK_GATE_EN   0
 *     05       DB_CLK_GATE_EN   0
 *     04      AGC_CLK_GATE_EN   0
 *     03       SD_CLK_GATE_EN   0
 *     02   GFSK_PREFILT_CLK_GATE_EN   0
 *     01   DAP_CALIB_CLK_GATE_EN   0
 *     00      DAP_CLK_GATE_EN   0
 * </pre>
 */
#define RPL_CRM_CNTL0_ADDR   0x0000006B
#define RPL_CRM_CNTL0_OFFSET 0x0000006B
#define RPL_CRM_CNTL0_INDEX  0x0000006B
#define RPL_CRM_CNTL0_RESET  0x00000000

__INLINE uint32_t rpl_crm_cntl0_get(void)
{
    return REG_RPL_RD(RPL_CRM_CNTL0_ADDR);
}

__INLINE void rpl_crm_cntl0_set(uint32_t value)
{
    REG_RPL_WR(RPL_CRM_CNTL0_ADDR, value);
}

// field definitions
#define RPL_FORCE_RESETS_BIT                ((uint32_t)0x80000000)
#define RPL_FORCE_RESETS_POS                31
#define RPL_FORCE_CLOCK_GATES_BIT           ((uint32_t)0x40000000)
#define RPL_FORCE_CLOCK_GATES_POS           30
#define RPL_DAP_TXDC_RST_EN_BIT             ((uint32_t)0x10000000)
#define RPL_DAP_TXDC_RST_EN_POS             28
#define RPL_RPLL_FDIV_RST_EN_BIT            ((uint32_t)0x08000000)
#define RPL_RPLL_FDIV_RST_EN_POS            27
#define RPL_MDM_RST_EN_BIT                  ((uint32_t)0x04000000)
#define RPL_MDM_RST_EN_POS                  26
#define RPL_DB_RST_EN_BIT                   ((uint32_t)0x02000000)
#define RPL_DB_RST_EN_POS                   25
#define RPL_AGC_RST_EN_BIT                  ((uint32_t)0x01000000)
#define RPL_AGC_RST_EN_POS                  24
#define RPL_SD_RST_EN_BIT                   ((uint32_t)0x00800000)
#define RPL_SD_RST_EN_POS                   23
#define RPL_GFSK_PREFILT_RST_EN_BIT         ((uint32_t)0x00400000)
#define RPL_GFSK_PREFILT_RST_EN_POS         22
#define RPL_DAP_CALIB_RST_EN_BIT            ((uint32_t)0x00200000)
#define RPL_DAP_CALIB_RST_EN_POS            21
#define RPL_DAP_RST_EN_BIT                  ((uint32_t)0x00100000)
#define RPL_DAP_RST_EN_POS                  20
#define RPL_DAP_TXDC_CLK_GATE_EN_BIT        ((uint32_t)0x00040000)
#define RPL_DAP_TXDC_CLK_GATE_EN_POS        18
#define RPL_RPLL_FDIV_CLK_GATE_EN_BIT       ((uint32_t)0x00020000)
#define RPL_RPLL_FDIV_CLK_GATE_EN_POS       17
#define RPL_SAR_ADC_CLK_GATE_EN_BIT         ((uint32_t)0x00010000)
#define RPL_SAR_ADC_CLK_GATE_EN_POS         16
#define RPL_MDM_TXDPSK13_CLK_GATE_EN_BIT    ((uint32_t)0x00008000)
#define RPL_MDM_TXDPSK13_CLK_GATE_EN_POS    15
#define RPL_MDM_TXGFSK13_CLK_GATE_EN_BIT    ((uint32_t)0x00004000)
#define RPL_MDM_TXGFSK13_CLK_GATE_EN_POS    14
#define RPL_MDM_TX13_CLK_GATE_EN_BIT        ((uint32_t)0x00002000)
#define RPL_MDM_TX13_CLK_GATE_EN_POS        13
#define RPL_MDM_RXDPSK13_CLK_GATE_EN_BIT    ((uint32_t)0x00001000)
#define RPL_MDM_RXDPSK13_CLK_GATE_EN_POS    12
#define RPL_MDM_RXDPSK52_CLK_GATE_EN_BIT    ((uint32_t)0x00000800)
#define RPL_MDM_RXDPSK52_CLK_GATE_EN_POS    11
#define RPL_MDM_RXDL13_CLK_GATE_EN_BIT      ((uint32_t)0x00000400)
#define RPL_MDM_RXDL13_CLK_GATE_EN_POS      10
#define RPL_MDM_RXGFSK13_CLK_GATE_EN_BIT    ((uint32_t)0x00000200)
#define RPL_MDM_RXGFSK13_CLK_GATE_EN_POS    9
#define RPL_MDM_RXGFSK52_CLK_GATE_EN_BIT    ((uint32_t)0x00000100)
#define RPL_MDM_RXGFSK52_CLK_GATE_EN_POS    8
#define RPL_MDM_RX52_CLK_GATE_EN_BIT        ((uint32_t)0x00000080)
#define RPL_MDM_RX52_CLK_GATE_EN_POS        7
#define RPL_MDM_CORE13_CLK_GATE_EN_BIT      ((uint32_t)0x00000040)
#define RPL_MDM_CORE13_CLK_GATE_EN_POS      6
#define RPL_DB_CLK_GATE_EN_BIT              ((uint32_t)0x00000020)
#define RPL_DB_CLK_GATE_EN_POS              5
#define RPL_AGC_CLK_GATE_EN_BIT             ((uint32_t)0x00000010)
#define RPL_AGC_CLK_GATE_EN_POS             4
#define RPL_SD_CLK_GATE_EN_BIT              ((uint32_t)0x00000008)
#define RPL_SD_CLK_GATE_EN_POS              3
#define RPL_GFSK_PREFILT_CLK_GATE_EN_BIT    ((uint32_t)0x00000004)
#define RPL_GFSK_PREFILT_CLK_GATE_EN_POS    2
#define RPL_DAP_CALIB_CLK_GATE_EN_BIT       ((uint32_t)0x00000002)
#define RPL_DAP_CALIB_CLK_GATE_EN_POS       1
#define RPL_DAP_CLK_GATE_EN_BIT             ((uint32_t)0x00000001)
#define RPL_DAP_CLK_GATE_EN_POS             0

#define RPL_FORCE_RESETS_RST                0x0
#define RPL_FORCE_CLOCK_GATES_RST           0x0
#define RPL_DAP_TXDC_RST_EN_RST             0x0
#define RPL_RPLL_FDIV_RST_EN_RST            0x0
#define RPL_MDM_RST_EN_RST                  0x0
#define RPL_DB_RST_EN_RST                   0x0
#define RPL_AGC_RST_EN_RST                  0x0
#define RPL_SD_RST_EN_RST                   0x0
#define RPL_GFSK_PREFILT_RST_EN_RST         0x0
#define RPL_DAP_CALIB_RST_EN_RST            0x0
#define RPL_DAP_RST_EN_RST                  0x0
#define RPL_DAP_TXDC_CLK_GATE_EN_RST        0x0
#define RPL_RPLL_FDIV_CLK_GATE_EN_RST       0x0
#define RPL_SAR_ADC_CLK_GATE_EN_RST         0x0
#define RPL_MDM_TXDPSK13_CLK_GATE_EN_RST    0x0
#define RPL_MDM_TXGFSK13_CLK_GATE_EN_RST    0x0
#define RPL_MDM_TX13_CLK_GATE_EN_RST        0x0
#define RPL_MDM_RXDPSK13_CLK_GATE_EN_RST    0x0
#define RPL_MDM_RXDPSK52_CLK_GATE_EN_RST    0x0
#define RPL_MDM_RXDL13_CLK_GATE_EN_RST      0x0
#define RPL_MDM_RXGFSK13_CLK_GATE_EN_RST    0x0
#define RPL_MDM_RXGFSK52_CLK_GATE_EN_RST    0x0
#define RPL_MDM_RX52_CLK_GATE_EN_RST        0x0
#define RPL_MDM_CORE13_CLK_GATE_EN_RST      0x0
#define RPL_DB_CLK_GATE_EN_RST              0x0
#define RPL_AGC_CLK_GATE_EN_RST             0x0
#define RPL_SD_CLK_GATE_EN_RST              0x0
#define RPL_GFSK_PREFILT_CLK_GATE_EN_RST    0x0
#define RPL_DAP_CALIB_CLK_GATE_EN_RST       0x0
#define RPL_DAP_CLK_GATE_EN_RST             0x0

__INLINE void rpl_crm_cntl0_pack(uint8_t forceresets, uint8_t forceclockgates, uint8_t daptxdcrsten, uint8_t rpllfdivrsten, uint8_t mdmrsten, uint8_t dbrsten, uint8_t agcrsten, uint8_t sdrsten, uint8_t gfskprefiltrsten, uint8_t dapcalibrsten, uint8_t daprsten, uint8_t daptxdcclkgateen, uint8_t rpllfdivclkgateen, uint8_t saradcclkgateen, uint8_t mdmtxdpsk13clkgateen, uint8_t mdmtxgfsk13clkgateen, uint8_t mdmtx13clkgateen, uint8_t mdmrxdpsk13clkgateen, uint8_t mdmrxdpsk52clkgateen, uint8_t mdmrxdl13clkgateen, uint8_t mdmrxgfsk13clkgateen, uint8_t mdmrxgfsk52clkgateen, uint8_t mdmrx52clkgateen, uint8_t mdmcore13clkgateen, uint8_t dbclkgateen, uint8_t agcclkgateen, uint8_t sdclkgateen, uint8_t gfskprefiltclkgateen, uint8_t dapcalibclkgateen, uint8_t dapclkgateen)
{
    ASSERT_ERR((((uint32_t)forceresets << 31) & ~((uint32_t)0x80000000)) == 0);
    ASSERT_ERR((((uint32_t)forceclockgates << 30) & ~((uint32_t)0x40000000)) == 0);
    ASSERT_ERR((((uint32_t)daptxdcrsten << 28) & ~((uint32_t)0x10000000)) == 0);
    ASSERT_ERR((((uint32_t)rpllfdivrsten << 27) & ~((uint32_t)0x08000000)) == 0);
    ASSERT_ERR((((uint32_t)mdmrsten << 26) & ~((uint32_t)0x04000000)) == 0);
    ASSERT_ERR((((uint32_t)dbrsten << 25) & ~((uint32_t)0x02000000)) == 0);
    ASSERT_ERR((((uint32_t)agcrsten << 24) & ~((uint32_t)0x01000000)) == 0);
    ASSERT_ERR((((uint32_t)sdrsten << 23) & ~((uint32_t)0x00800000)) == 0);
    ASSERT_ERR((((uint32_t)gfskprefiltrsten << 22) & ~((uint32_t)0x00400000)) == 0);
    ASSERT_ERR((((uint32_t)dapcalibrsten << 21) & ~((uint32_t)0x00200000)) == 0);
    ASSERT_ERR((((uint32_t)daprsten << 20) & ~((uint32_t)0x00100000)) == 0);
    ASSERT_ERR((((uint32_t)daptxdcclkgateen << 18) & ~((uint32_t)0x00040000)) == 0);
    ASSERT_ERR((((uint32_t)rpllfdivclkgateen << 17) & ~((uint32_t)0x00020000)) == 0);
    ASSERT_ERR((((uint32_t)saradcclkgateen << 16) & ~((uint32_t)0x00010000)) == 0);
    ASSERT_ERR((((uint32_t)mdmtxdpsk13clkgateen << 15) & ~((uint32_t)0x00008000)) == 0);
    ASSERT_ERR((((uint32_t)mdmtxgfsk13clkgateen << 14) & ~((uint32_t)0x00004000)) == 0);
    ASSERT_ERR((((uint32_t)mdmtx13clkgateen << 13) & ~((uint32_t)0x00002000)) == 0);
    ASSERT_ERR((((uint32_t)mdmrxdpsk13clkgateen << 12) & ~((uint32_t)0x00001000)) == 0);
    ASSERT_ERR((((uint32_t)mdmrxdpsk52clkgateen << 11) & ~((uint32_t)0x00000800)) == 0);
    ASSERT_ERR((((uint32_t)mdmrxdl13clkgateen << 10) & ~((uint32_t)0x00000400)) == 0);
    ASSERT_ERR((((uint32_t)mdmrxgfsk13clkgateen << 9) & ~((uint32_t)0x00000200)) == 0);
    ASSERT_ERR((((uint32_t)mdmrxgfsk52clkgateen << 8) & ~((uint32_t)0x00000100)) == 0);
    ASSERT_ERR((((uint32_t)mdmrx52clkgateen << 7) & ~((uint32_t)0x00000080)) == 0);
    ASSERT_ERR((((uint32_t)mdmcore13clkgateen << 6) & ~((uint32_t)0x00000040)) == 0);
    ASSERT_ERR((((uint32_t)dbclkgateen << 5) & ~((uint32_t)0x00000020)) == 0);
    ASSERT_ERR((((uint32_t)agcclkgateen << 4) & ~((uint32_t)0x00000010)) == 0);
    ASSERT_ERR((((uint32_t)sdclkgateen << 3) & ~((uint32_t)0x00000008)) == 0);
    ASSERT_ERR((((uint32_t)gfskprefiltclkgateen << 2) & ~((uint32_t)0x00000004)) == 0);
    ASSERT_ERR((((uint32_t)dapcalibclkgateen << 1) & ~((uint32_t)0x00000002)) == 0);
    ASSERT_ERR((((uint32_t)dapclkgateen << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_RPL_WR(RPL_CRM_CNTL0_ADDR, ((uint32_t)forceresets << 31) | ((uint32_t)forceclockgates << 30) | ((uint32_t)daptxdcrsten << 28) | ((uint32_t)rpllfdivrsten << 27) | ((uint32_t)mdmrsten << 26) | ((uint32_t)dbrsten << 25) | ((uint32_t)agcrsten << 24) | ((uint32_t)sdrsten << 23) | ((uint32_t)gfskprefiltrsten << 22) | ((uint32_t)dapcalibrsten << 21) | ((uint32_t)daprsten << 20) | ((uint32_t)daptxdcclkgateen << 18) | ((uint32_t)rpllfdivclkgateen << 17) | ((uint32_t)saradcclkgateen << 16) | ((uint32_t)mdmtxdpsk13clkgateen << 15) | ((uint32_t)mdmtxgfsk13clkgateen << 14) | ((uint32_t)mdmtx13clkgateen << 13) | ((uint32_t)mdmrxdpsk13clkgateen << 12) | ((uint32_t)mdmrxdpsk52clkgateen << 11) | ((uint32_t)mdmrxdl13clkgateen << 10) | ((uint32_t)mdmrxgfsk13clkgateen << 9) | ((uint32_t)mdmrxgfsk52clkgateen << 8) | ((uint32_t)mdmrx52clkgateen << 7) | ((uint32_t)mdmcore13clkgateen << 6) | ((uint32_t)dbclkgateen << 5) | ((uint32_t)agcclkgateen << 4) | ((uint32_t)sdclkgateen << 3) | ((uint32_t)gfskprefiltclkgateen << 2) | ((uint32_t)dapcalibclkgateen << 1) | ((uint32_t)dapclkgateen << 0));
}

__INLINE void rpl_crm_cntl0_unpack(uint8_t *forceresets, uint8_t *forceclockgates, uint8_t *daptxdcrsten, uint8_t *rpllfdivrsten, uint8_t *mdmrsten, uint8_t *dbrsten, uint8_t *agcrsten, uint8_t *sdrsten, uint8_t *gfskprefiltrsten, uint8_t *dapcalibrsten, uint8_t *daprsten, uint8_t *daptxdcclkgateen, uint8_t *rpllfdivclkgateen, uint8_t *saradcclkgateen, uint8_t *mdmtxdpsk13clkgateen, uint8_t *mdmtxgfsk13clkgateen, uint8_t *mdmtx13clkgateen, uint8_t *mdmrxdpsk13clkgateen, uint8_t *mdmrxdpsk52clkgateen, uint8_t *mdmrxdl13clkgateen, uint8_t *mdmrxgfsk13clkgateen, uint8_t *mdmrxgfsk52clkgateen, uint8_t *mdmrx52clkgateen, uint8_t *mdmcore13clkgateen, uint8_t *dbclkgateen, uint8_t *agcclkgateen, uint8_t *sdclkgateen, uint8_t *gfskprefiltclkgateen, uint8_t *dapcalibclkgateen, uint8_t *dapclkgateen)
{
    uint32_t localVal = REG_RPL_RD(RPL_CRM_CNTL0_ADDR);

    *forceresets = (localVal & ((uint32_t)0x80000000)) >> 31;
    *forceclockgates = (localVal & ((uint32_t)0x40000000)) >> 30;
    *daptxdcrsten = (localVal & ((uint32_t)0x10000000)) >> 28;
    *rpllfdivrsten = (localVal & ((uint32_t)0x08000000)) >> 27;
    *mdmrsten = (localVal & ((uint32_t)0x04000000)) >> 26;
    *dbrsten = (localVal & ((uint32_t)0x02000000)) >> 25;
    *agcrsten = (localVal & ((uint32_t)0x01000000)) >> 24;
    *sdrsten = (localVal & ((uint32_t)0x00800000)) >> 23;
    *gfskprefiltrsten = (localVal & ((uint32_t)0x00400000)) >> 22;
    *dapcalibrsten = (localVal & ((uint32_t)0x00200000)) >> 21;
    *daprsten = (localVal & ((uint32_t)0x00100000)) >> 20;
    *daptxdcclkgateen = (localVal & ((uint32_t)0x00040000)) >> 18;
    *rpllfdivclkgateen = (localVal & ((uint32_t)0x00020000)) >> 17;
    *saradcclkgateen = (localVal & ((uint32_t)0x00010000)) >> 16;
    *mdmtxdpsk13clkgateen = (localVal & ((uint32_t)0x00008000)) >> 15;
    *mdmtxgfsk13clkgateen = (localVal & ((uint32_t)0x00004000)) >> 14;
    *mdmtx13clkgateen = (localVal & ((uint32_t)0x00002000)) >> 13;
    *mdmrxdpsk13clkgateen = (localVal & ((uint32_t)0x00001000)) >> 12;
    *mdmrxdpsk52clkgateen = (localVal & ((uint32_t)0x00000800)) >> 11;
    *mdmrxdl13clkgateen = (localVal & ((uint32_t)0x00000400)) >> 10;
    *mdmrxgfsk13clkgateen = (localVal & ((uint32_t)0x00000200)) >> 9;
    *mdmrxgfsk52clkgateen = (localVal & ((uint32_t)0x00000100)) >> 8;
    *mdmrx52clkgateen = (localVal & ((uint32_t)0x00000080)) >> 7;
    *mdmcore13clkgateen = (localVal & ((uint32_t)0x00000040)) >> 6;
    *dbclkgateen = (localVal & ((uint32_t)0x00000020)) >> 5;
    *agcclkgateen = (localVal & ((uint32_t)0x00000010)) >> 4;
    *sdclkgateen = (localVal & ((uint32_t)0x00000008)) >> 3;
    *gfskprefiltclkgateen = (localVal & ((uint32_t)0x00000004)) >> 2;
    *dapcalibclkgateen = (localVal & ((uint32_t)0x00000002)) >> 1;
    *dapclkgateen = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t rpl_force_resets_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_CRM_CNTL0_ADDR);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE void rpl_force_resets_setf(uint8_t forceresets)
{
    ASSERT_ERR((((uint32_t)forceresets << 31) & ~((uint32_t)0x80000000)) == 0);
    REG_RPL_WR(RPL_CRM_CNTL0_ADDR, (REG_RPL_RD(RPL_CRM_CNTL0_ADDR) & ~((uint32_t)0x80000000)) | ((uint32_t)forceresets << 31));
}

__INLINE uint8_t rpl_force_clock_gates_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_CRM_CNTL0_ADDR);
    return ((localVal & ((uint32_t)0x40000000)) >> 30);
}

__INLINE void rpl_force_clock_gates_setf(uint8_t forceclockgates)
{
    ASSERT_ERR((((uint32_t)forceclockgates << 30) & ~((uint32_t)0x40000000)) == 0);
    REG_RPL_WR(RPL_CRM_CNTL0_ADDR, (REG_RPL_RD(RPL_CRM_CNTL0_ADDR) & ~((uint32_t)0x40000000)) | ((uint32_t)forceclockgates << 30));
}

__INLINE uint8_t rpl_dap_txdc_rst_en_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_CRM_CNTL0_ADDR);
    return ((localVal & ((uint32_t)0x10000000)) >> 28);
}

__INLINE void rpl_dap_txdc_rst_en_setf(uint8_t daptxdcrsten)
{
    ASSERT_ERR((((uint32_t)daptxdcrsten << 28) & ~((uint32_t)0x10000000)) == 0);
    REG_RPL_WR(RPL_CRM_CNTL0_ADDR, (REG_RPL_RD(RPL_CRM_CNTL0_ADDR) & ~((uint32_t)0x10000000)) | ((uint32_t)daptxdcrsten << 28));
}

__INLINE uint8_t rpl_rpll_fdiv_rst_en_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_CRM_CNTL0_ADDR);
    return ((localVal & ((uint32_t)0x08000000)) >> 27);
}

__INLINE void rpl_rpll_fdiv_rst_en_setf(uint8_t rpllfdivrsten)
{
    ASSERT_ERR((((uint32_t)rpllfdivrsten << 27) & ~((uint32_t)0x08000000)) == 0);
    REG_RPL_WR(RPL_CRM_CNTL0_ADDR, (REG_RPL_RD(RPL_CRM_CNTL0_ADDR) & ~((uint32_t)0x08000000)) | ((uint32_t)rpllfdivrsten << 27));
}

__INLINE uint8_t rpl_mdm_rst_en_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_CRM_CNTL0_ADDR);
    return ((localVal & ((uint32_t)0x04000000)) >> 26);
}

__INLINE void rpl_mdm_rst_en_setf(uint8_t mdmrsten)
{
    ASSERT_ERR((((uint32_t)mdmrsten << 26) & ~((uint32_t)0x04000000)) == 0);
    REG_RPL_WR(RPL_CRM_CNTL0_ADDR, (REG_RPL_RD(RPL_CRM_CNTL0_ADDR) & ~((uint32_t)0x04000000)) | ((uint32_t)mdmrsten << 26));
}

__INLINE uint8_t rpl_db_rst_en_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_CRM_CNTL0_ADDR);
    return ((localVal & ((uint32_t)0x02000000)) >> 25);
}

__INLINE void rpl_db_rst_en_setf(uint8_t dbrsten)
{
    ASSERT_ERR((((uint32_t)dbrsten << 25) & ~((uint32_t)0x02000000)) == 0);
    REG_RPL_WR(RPL_CRM_CNTL0_ADDR, (REG_RPL_RD(RPL_CRM_CNTL0_ADDR) & ~((uint32_t)0x02000000)) | ((uint32_t)dbrsten << 25));
}

__INLINE uint8_t rpl_agc_rst_en_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_CRM_CNTL0_ADDR);
    return ((localVal & ((uint32_t)0x01000000)) >> 24);
}

__INLINE void rpl_agc_rst_en_setf(uint8_t agcrsten)
{
    ASSERT_ERR((((uint32_t)agcrsten << 24) & ~((uint32_t)0x01000000)) == 0);
    REG_RPL_WR(RPL_CRM_CNTL0_ADDR, (REG_RPL_RD(RPL_CRM_CNTL0_ADDR) & ~((uint32_t)0x01000000)) | ((uint32_t)agcrsten << 24));
}

__INLINE uint8_t rpl_sd_rst_en_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_CRM_CNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00800000)) >> 23);
}

__INLINE void rpl_sd_rst_en_setf(uint8_t sdrsten)
{
    ASSERT_ERR((((uint32_t)sdrsten << 23) & ~((uint32_t)0x00800000)) == 0);
    REG_RPL_WR(RPL_CRM_CNTL0_ADDR, (REG_RPL_RD(RPL_CRM_CNTL0_ADDR) & ~((uint32_t)0x00800000)) | ((uint32_t)sdrsten << 23));
}

__INLINE uint8_t rpl_gfsk_prefilt_rst_en_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_CRM_CNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00400000)) >> 22);
}

__INLINE void rpl_gfsk_prefilt_rst_en_setf(uint8_t gfskprefiltrsten)
{
    ASSERT_ERR((((uint32_t)gfskprefiltrsten << 22) & ~((uint32_t)0x00400000)) == 0);
    REG_RPL_WR(RPL_CRM_CNTL0_ADDR, (REG_RPL_RD(RPL_CRM_CNTL0_ADDR) & ~((uint32_t)0x00400000)) | ((uint32_t)gfskprefiltrsten << 22));
}

__INLINE uint8_t rpl_dap_calib_rst_en_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_CRM_CNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00200000)) >> 21);
}

__INLINE void rpl_dap_calib_rst_en_setf(uint8_t dapcalibrsten)
{
    ASSERT_ERR((((uint32_t)dapcalibrsten << 21) & ~((uint32_t)0x00200000)) == 0);
    REG_RPL_WR(RPL_CRM_CNTL0_ADDR, (REG_RPL_RD(RPL_CRM_CNTL0_ADDR) & ~((uint32_t)0x00200000)) | ((uint32_t)dapcalibrsten << 21));
}

__INLINE uint8_t rpl_dap_rst_en_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_CRM_CNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00100000)) >> 20);
}

__INLINE void rpl_dap_rst_en_setf(uint8_t daprsten)
{
    ASSERT_ERR((((uint32_t)daprsten << 20) & ~((uint32_t)0x00100000)) == 0);
    REG_RPL_WR(RPL_CRM_CNTL0_ADDR, (REG_RPL_RD(RPL_CRM_CNTL0_ADDR) & ~((uint32_t)0x00100000)) | ((uint32_t)daprsten << 20));
}

__INLINE uint8_t rpl_dap_txdc_clk_gate_en_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_CRM_CNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00040000)) >> 18);
}

__INLINE void rpl_dap_txdc_clk_gate_en_setf(uint8_t daptxdcclkgateen)
{
    ASSERT_ERR((((uint32_t)daptxdcclkgateen << 18) & ~((uint32_t)0x00040000)) == 0);
    REG_RPL_WR(RPL_CRM_CNTL0_ADDR, (REG_RPL_RD(RPL_CRM_CNTL0_ADDR) & ~((uint32_t)0x00040000)) | ((uint32_t)daptxdcclkgateen << 18));
}

__INLINE uint8_t rpl_rpll_fdiv_clk_gate_en_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_CRM_CNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00020000)) >> 17);
}

__INLINE void rpl_rpll_fdiv_clk_gate_en_setf(uint8_t rpllfdivclkgateen)
{
    ASSERT_ERR((((uint32_t)rpllfdivclkgateen << 17) & ~((uint32_t)0x00020000)) == 0);
    REG_RPL_WR(RPL_CRM_CNTL0_ADDR, (REG_RPL_RD(RPL_CRM_CNTL0_ADDR) & ~((uint32_t)0x00020000)) | ((uint32_t)rpllfdivclkgateen << 17));
}

__INLINE uint8_t rpl_sar_adc_clk_gate_en_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_CRM_CNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00010000)) >> 16);
}

__INLINE void rpl_sar_adc_clk_gate_en_setf(uint8_t saradcclkgateen)
{
    ASSERT_ERR((((uint32_t)saradcclkgateen << 16) & ~((uint32_t)0x00010000)) == 0);
    REG_RPL_WR(RPL_CRM_CNTL0_ADDR, (REG_RPL_RD(RPL_CRM_CNTL0_ADDR) & ~((uint32_t)0x00010000)) | ((uint32_t)saradcclkgateen << 16));
}

__INLINE uint8_t rpl_mdm_txdpsk13_clk_gate_en_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_CRM_CNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00008000)) >> 15);
}

__INLINE void rpl_mdm_txdpsk13_clk_gate_en_setf(uint8_t mdmtxdpsk13clkgateen)
{
    ASSERT_ERR((((uint32_t)mdmtxdpsk13clkgateen << 15) & ~((uint32_t)0x00008000)) == 0);
    REG_RPL_WR(RPL_CRM_CNTL0_ADDR, (REG_RPL_RD(RPL_CRM_CNTL0_ADDR) & ~((uint32_t)0x00008000)) | ((uint32_t)mdmtxdpsk13clkgateen << 15));
}

__INLINE uint8_t rpl_mdm_txgfsk13_clk_gate_en_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_CRM_CNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00004000)) >> 14);
}

__INLINE void rpl_mdm_txgfsk13_clk_gate_en_setf(uint8_t mdmtxgfsk13clkgateen)
{
    ASSERT_ERR((((uint32_t)mdmtxgfsk13clkgateen << 14) & ~((uint32_t)0x00004000)) == 0);
    REG_RPL_WR(RPL_CRM_CNTL0_ADDR, (REG_RPL_RD(RPL_CRM_CNTL0_ADDR) & ~((uint32_t)0x00004000)) | ((uint32_t)mdmtxgfsk13clkgateen << 14));
}

__INLINE uint8_t rpl_mdm_tx13_clk_gate_en_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_CRM_CNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00002000)) >> 13);
}

__INLINE void rpl_mdm_tx13_clk_gate_en_setf(uint8_t mdmtx13clkgateen)
{
    ASSERT_ERR((((uint32_t)mdmtx13clkgateen << 13) & ~((uint32_t)0x00002000)) == 0);
    REG_RPL_WR(RPL_CRM_CNTL0_ADDR, (REG_RPL_RD(RPL_CRM_CNTL0_ADDR) & ~((uint32_t)0x00002000)) | ((uint32_t)mdmtx13clkgateen << 13));
}

__INLINE uint8_t rpl_mdm_rxdpsk13_clk_gate_en_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_CRM_CNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00001000)) >> 12);
}

__INLINE void rpl_mdm_rxdpsk13_clk_gate_en_setf(uint8_t mdmrxdpsk13clkgateen)
{
    ASSERT_ERR((((uint32_t)mdmrxdpsk13clkgateen << 12) & ~((uint32_t)0x00001000)) == 0);
    REG_RPL_WR(RPL_CRM_CNTL0_ADDR, (REG_RPL_RD(RPL_CRM_CNTL0_ADDR) & ~((uint32_t)0x00001000)) | ((uint32_t)mdmrxdpsk13clkgateen << 12));
}

__INLINE uint8_t rpl_mdm_rxdpsk52_clk_gate_en_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_CRM_CNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00000800)) >> 11);
}

__INLINE void rpl_mdm_rxdpsk52_clk_gate_en_setf(uint8_t mdmrxdpsk52clkgateen)
{
    ASSERT_ERR((((uint32_t)mdmrxdpsk52clkgateen << 11) & ~((uint32_t)0x00000800)) == 0);
    REG_RPL_WR(RPL_CRM_CNTL0_ADDR, (REG_RPL_RD(RPL_CRM_CNTL0_ADDR) & ~((uint32_t)0x00000800)) | ((uint32_t)mdmrxdpsk52clkgateen << 11));
}

__INLINE uint8_t rpl_mdm_rxdl13_clk_gate_en_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_CRM_CNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00000400)) >> 10);
}

__INLINE void rpl_mdm_rxdl13_clk_gate_en_setf(uint8_t mdmrxdl13clkgateen)
{
    ASSERT_ERR((((uint32_t)mdmrxdl13clkgateen << 10) & ~((uint32_t)0x00000400)) == 0);
    REG_RPL_WR(RPL_CRM_CNTL0_ADDR, (REG_RPL_RD(RPL_CRM_CNTL0_ADDR) & ~((uint32_t)0x00000400)) | ((uint32_t)mdmrxdl13clkgateen << 10));
}

__INLINE uint8_t rpl_mdm_rxgfsk13_clk_gate_en_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_CRM_CNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00000200)) >> 9);
}

__INLINE void rpl_mdm_rxgfsk13_clk_gate_en_setf(uint8_t mdmrxgfsk13clkgateen)
{
    ASSERT_ERR((((uint32_t)mdmrxgfsk13clkgateen << 9) & ~((uint32_t)0x00000200)) == 0);
    REG_RPL_WR(RPL_CRM_CNTL0_ADDR, (REG_RPL_RD(RPL_CRM_CNTL0_ADDR) & ~((uint32_t)0x00000200)) | ((uint32_t)mdmrxgfsk13clkgateen << 9));
}

__INLINE uint8_t rpl_mdm_rxgfsk52_clk_gate_en_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_CRM_CNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00000100)) >> 8);
}

__INLINE void rpl_mdm_rxgfsk52_clk_gate_en_setf(uint8_t mdmrxgfsk52clkgateen)
{
    ASSERT_ERR((((uint32_t)mdmrxgfsk52clkgateen << 8) & ~((uint32_t)0x00000100)) == 0);
    REG_RPL_WR(RPL_CRM_CNTL0_ADDR, (REG_RPL_RD(RPL_CRM_CNTL0_ADDR) & ~((uint32_t)0x00000100)) | ((uint32_t)mdmrxgfsk52clkgateen << 8));
}

__INLINE uint8_t rpl_mdm_rx52_clk_gate_en_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_CRM_CNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00000080)) >> 7);
}

__INLINE void rpl_mdm_rx52_clk_gate_en_setf(uint8_t mdmrx52clkgateen)
{
    ASSERT_ERR((((uint32_t)mdmrx52clkgateen << 7) & ~((uint32_t)0x00000080)) == 0);
    REG_RPL_WR(RPL_CRM_CNTL0_ADDR, (REG_RPL_RD(RPL_CRM_CNTL0_ADDR) & ~((uint32_t)0x00000080)) | ((uint32_t)mdmrx52clkgateen << 7));
}

__INLINE uint8_t rpl_mdm_core13_clk_gate_en_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_CRM_CNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00000040)) >> 6);
}

__INLINE void rpl_mdm_core13_clk_gate_en_setf(uint8_t mdmcore13clkgateen)
{
    ASSERT_ERR((((uint32_t)mdmcore13clkgateen << 6) & ~((uint32_t)0x00000040)) == 0);
    REG_RPL_WR(RPL_CRM_CNTL0_ADDR, (REG_RPL_RD(RPL_CRM_CNTL0_ADDR) & ~((uint32_t)0x00000040)) | ((uint32_t)mdmcore13clkgateen << 6));
}

__INLINE uint8_t rpl_db_clk_gate_en_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_CRM_CNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00000020)) >> 5);
}

__INLINE void rpl_db_clk_gate_en_setf(uint8_t dbclkgateen)
{
    ASSERT_ERR((((uint32_t)dbclkgateen << 5) & ~((uint32_t)0x00000020)) == 0);
    REG_RPL_WR(RPL_CRM_CNTL0_ADDR, (REG_RPL_RD(RPL_CRM_CNTL0_ADDR) & ~((uint32_t)0x00000020)) | ((uint32_t)dbclkgateen << 5));
}

__INLINE uint8_t rpl_agc_clk_gate_en_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_CRM_CNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE void rpl_agc_clk_gate_en_setf(uint8_t agcclkgateen)
{
    ASSERT_ERR((((uint32_t)agcclkgateen << 4) & ~((uint32_t)0x00000010)) == 0);
    REG_RPL_WR(RPL_CRM_CNTL0_ADDR, (REG_RPL_RD(RPL_CRM_CNTL0_ADDR) & ~((uint32_t)0x00000010)) | ((uint32_t)agcclkgateen << 4));
}

__INLINE uint8_t rpl_sd_clk_gate_en_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_CRM_CNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

__INLINE void rpl_sd_clk_gate_en_setf(uint8_t sdclkgateen)
{
    ASSERT_ERR((((uint32_t)sdclkgateen << 3) & ~((uint32_t)0x00000008)) == 0);
    REG_RPL_WR(RPL_CRM_CNTL0_ADDR, (REG_RPL_RD(RPL_CRM_CNTL0_ADDR) & ~((uint32_t)0x00000008)) | ((uint32_t)sdclkgateen << 3));
}

__INLINE uint8_t rpl_gfsk_prefilt_clk_gate_en_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_CRM_CNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE void rpl_gfsk_prefilt_clk_gate_en_setf(uint8_t gfskprefiltclkgateen)
{
    ASSERT_ERR((((uint32_t)gfskprefiltclkgateen << 2) & ~((uint32_t)0x00000004)) == 0);
    REG_RPL_WR(RPL_CRM_CNTL0_ADDR, (REG_RPL_RD(RPL_CRM_CNTL0_ADDR) & ~((uint32_t)0x00000004)) | ((uint32_t)gfskprefiltclkgateen << 2));
}

__INLINE uint8_t rpl_dap_calib_clk_gate_en_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_CRM_CNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE void rpl_dap_calib_clk_gate_en_setf(uint8_t dapcalibclkgateen)
{
    ASSERT_ERR((((uint32_t)dapcalibclkgateen << 1) & ~((uint32_t)0x00000002)) == 0);
    REG_RPL_WR(RPL_CRM_CNTL0_ADDR, (REG_RPL_RD(RPL_CRM_CNTL0_ADDR) & ~((uint32_t)0x00000002)) | ((uint32_t)dapcalibclkgateen << 1));
}

__INLINE uint8_t rpl_dap_clk_gate_en_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_CRM_CNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void rpl_dap_clk_gate_en_setf(uint8_t dapclkgateen)
{
    ASSERT_ERR((((uint32_t)dapclkgateen << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_RPL_WR(RPL_CRM_CNTL0_ADDR, (REG_RPL_RD(RPL_CRM_CNTL0_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)dapclkgateen << 0));
}

/**
 * @brief CRM_CNTL1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     31       FORCE_CRM_CTRL   0
 *  25:24      CLK_SWITCH_CTRL   0x0
 *     23              DPLL_EN   0
 *     12      SAR_ADC_CLK_SEL   0
 *  09:08           BB_CLK_DIV   0x2
 *     00              LP_MODE   0
 * </pre>
 */
#define RPL_CRM_CNTL1_ADDR   0x0000006C
#define RPL_CRM_CNTL1_OFFSET 0x0000006C
#define RPL_CRM_CNTL1_INDEX  0x0000006C
#define RPL_CRM_CNTL1_RESET  0x00000200

__INLINE uint32_t rpl_crm_cntl1_get(void)
{
    return REG_RPL_RD(RPL_CRM_CNTL1_ADDR);
}

__INLINE void rpl_crm_cntl1_set(uint32_t value)
{
    REG_RPL_WR(RPL_CRM_CNTL1_ADDR, value);
}

// field definitions
#define RPL_FORCE_CRM_CTRL_BIT     ((uint32_t)0x80000000)
#define RPL_FORCE_CRM_CTRL_POS     31
#define RPL_CLK_SWITCH_CTRL_MASK   ((uint32_t)0x03000000)
#define RPL_CLK_SWITCH_CTRL_LSB    24
#define RPL_CLK_SWITCH_CTRL_WIDTH  ((uint32_t)0x00000002)
#define RPL_DPLL_EN_BIT            ((uint32_t)0x00800000)
#define RPL_DPLL_EN_POS            23
#define RPL_SAR_ADC_CLK_SEL_BIT    ((uint32_t)0x00001000)
#define RPL_SAR_ADC_CLK_SEL_POS    12
#define RPL_BB_CLK_DIV_MASK        ((uint32_t)0x00000300)
#define RPL_BB_CLK_DIV_LSB         8
#define RPL_BB_CLK_DIV_WIDTH       ((uint32_t)0x00000002)
#define RPL_LP_MODE_BIT            ((uint32_t)0x00000001)
#define RPL_LP_MODE_POS            0

#define RPL_FORCE_CRM_CTRL_RST     0x0
#define RPL_CLK_SWITCH_CTRL_RST    0x0
#define RPL_DPLL_EN_RST            0x0
#define RPL_SAR_ADC_CLK_SEL_RST    0x0
#define RPL_BB_CLK_DIV_RST         0x2
#define RPL_LP_MODE_RST            0x0

__INLINE void rpl_crm_cntl1_pack(uint8_t forcecrmctrl, uint8_t clkswitchctrl, uint8_t dpllen, uint8_t saradcclksel, uint8_t bbclkdiv, uint8_t lpmode)
{
    ASSERT_ERR((((uint32_t)forcecrmctrl << 31) & ~((uint32_t)0x80000000)) == 0);
    ASSERT_ERR((((uint32_t)clkswitchctrl << 24) & ~((uint32_t)0x03000000)) == 0);
    ASSERT_ERR((((uint32_t)dpllen << 23) & ~((uint32_t)0x00800000)) == 0);
    ASSERT_ERR((((uint32_t)saradcclksel << 12) & ~((uint32_t)0x00001000)) == 0);
    ASSERT_ERR((((uint32_t)bbclkdiv << 8) & ~((uint32_t)0x00000300)) == 0);
    ASSERT_ERR((((uint32_t)lpmode << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_RPL_WR(RPL_CRM_CNTL1_ADDR, ((uint32_t)forcecrmctrl << 31) | ((uint32_t)clkswitchctrl << 24) | ((uint32_t)dpllen << 23) | ((uint32_t)saradcclksel << 12) | ((uint32_t)bbclkdiv << 8) | ((uint32_t)lpmode << 0));
}

__INLINE void rpl_crm_cntl1_unpack(uint8_t *forcecrmctrl, uint8_t *clkswitchctrl, uint8_t *dpllen, uint8_t *saradcclksel, uint8_t *bbclkdiv, uint8_t *lpmode)
{
    uint32_t localVal = REG_RPL_RD(RPL_CRM_CNTL1_ADDR);

    *forcecrmctrl = (localVal & ((uint32_t)0x80000000)) >> 31;
    *clkswitchctrl = (localVal & ((uint32_t)0x03000000)) >> 24;
    *dpllen = (localVal & ((uint32_t)0x00800000)) >> 23;
    *saradcclksel = (localVal & ((uint32_t)0x00001000)) >> 12;
    *bbclkdiv = (localVal & ((uint32_t)0x00000300)) >> 8;
    *lpmode = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t rpl_force_crm_ctrl_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_CRM_CNTL1_ADDR);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE void rpl_force_crm_ctrl_setf(uint8_t forcecrmctrl)
{
    ASSERT_ERR((((uint32_t)forcecrmctrl << 31) & ~((uint32_t)0x80000000)) == 0);
    REG_RPL_WR(RPL_CRM_CNTL1_ADDR, (REG_RPL_RD(RPL_CRM_CNTL1_ADDR) & ~((uint32_t)0x80000000)) | ((uint32_t)forcecrmctrl << 31));
}

__INLINE uint8_t rpl_clk_switch_ctrl_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_CRM_CNTL1_ADDR);
    return ((localVal & ((uint32_t)0x03000000)) >> 24);
}

__INLINE void rpl_clk_switch_ctrl_setf(uint8_t clkswitchctrl)
{
    ASSERT_ERR((((uint32_t)clkswitchctrl << 24) & ~((uint32_t)0x03000000)) == 0);
    REG_RPL_WR(RPL_CRM_CNTL1_ADDR, (REG_RPL_RD(RPL_CRM_CNTL1_ADDR) & ~((uint32_t)0x03000000)) | ((uint32_t)clkswitchctrl << 24));
}

__INLINE uint8_t rpl_dpll_en_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_CRM_CNTL1_ADDR);
    return ((localVal & ((uint32_t)0x00800000)) >> 23);
}

__INLINE void rpl_dpll_en_setf(uint8_t dpllen)
{
    ASSERT_ERR((((uint32_t)dpllen << 23) & ~((uint32_t)0x00800000)) == 0);
    REG_RPL_WR(RPL_CRM_CNTL1_ADDR, (REG_RPL_RD(RPL_CRM_CNTL1_ADDR) & ~((uint32_t)0x00800000)) | ((uint32_t)dpllen << 23));
}

__INLINE uint8_t rpl_sar_adc_clk_sel_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_CRM_CNTL1_ADDR);
    return ((localVal & ((uint32_t)0x00001000)) >> 12);
}

__INLINE void rpl_sar_adc_clk_sel_setf(uint8_t saradcclksel)
{
    ASSERT_ERR((((uint32_t)saradcclksel << 12) & ~((uint32_t)0x00001000)) == 0);
    REG_RPL_WR(RPL_CRM_CNTL1_ADDR, (REG_RPL_RD(RPL_CRM_CNTL1_ADDR) & ~((uint32_t)0x00001000)) | ((uint32_t)saradcclksel << 12));
}

__INLINE uint8_t rpl_bb_clk_div_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_CRM_CNTL1_ADDR);
    return ((localVal & ((uint32_t)0x00000300)) >> 8);
}

__INLINE void rpl_bb_clk_div_setf(uint8_t bbclkdiv)
{
    ASSERT_ERR((((uint32_t)bbclkdiv << 8) & ~((uint32_t)0x00000300)) == 0);
    REG_RPL_WR(RPL_CRM_CNTL1_ADDR, (REG_RPL_RD(RPL_CRM_CNTL1_ADDR) & ~((uint32_t)0x00000300)) | ((uint32_t)bbclkdiv << 8));
}

__INLINE uint8_t rpl_lp_mode_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_CRM_CNTL1_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void rpl_lp_mode_setf(uint8_t lpmode)
{
    ASSERT_ERR((((uint32_t)lpmode << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_RPL_WR(RPL_CRM_CNTL1_ADDR, (REG_RPL_RD(RPL_CRM_CNTL1_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)lpmode << 0));
}

/**
 * @brief RFRXCNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     31         RXCTRL_FORCE   0
 *     30         RXGAIN_FORCE   0
 *     29       FORCE_ADDA_RST   0
 *     25      RSSI_CLK_INV_EN   0
 *     24     RXADC_CLK_INV_EN   0
 *     18       N_LDO_ADDA_RST   0
 *     13      RXADC_SATDET_EN   0
 *     12          MODEM_RX_EN   0
 *     11             RXADC_EN   0
 *     10             RXADC_ON   0
 *     09               AGC_EN   0
 *     08              RSSI_ON   0
 *     07            RXFILT_ON   0
 *     06           RXLOBUF_ON   0
 *     05             RXMIX_ON   0
 *     04             RXLNA_ON   0
 *     03         RXRF_BIAS_ON   0
 *  02:00           RXGAIN_SEL   0x0
 * </pre>
 */
#define RPL_RFRXCNTL_ADDR   0x0000006D
#define RPL_RFRXCNTL_OFFSET 0x0000006D
#define RPL_RFRXCNTL_INDEX  0x0000006D
#define RPL_RFRXCNTL_RESET  0x00000000

__INLINE uint32_t rpl_rfrxcntl_get(void)
{
    return REG_RPL_RD(RPL_RFRXCNTL_ADDR);
}

__INLINE void rpl_rfrxcntl_set(uint32_t value)
{
    REG_RPL_WR(RPL_RFRXCNTL_ADDR, value);
}

// field definitions
#define RPL_RXCTRL_FORCE_BIT        ((uint32_t)0x80000000)
#define RPL_RXCTRL_FORCE_POS        31
#define RPL_RXGAIN_FORCE_BIT        ((uint32_t)0x40000000)
#define RPL_RXGAIN_FORCE_POS        30
#define RPL_FORCE_ADDA_RST_BIT      ((uint32_t)0x20000000)
#define RPL_FORCE_ADDA_RST_POS      29
#define RPL_RSSI_CLK_INV_EN_BIT     ((uint32_t)0x02000000)
#define RPL_RSSI_CLK_INV_EN_POS     25
#define RPL_RXADC_CLK_INV_EN_BIT    ((uint32_t)0x01000000)
#define RPL_RXADC_CLK_INV_EN_POS    24
#define RPL_N_LDO_ADDA_RST_BIT      ((uint32_t)0x00040000)
#define RPL_N_LDO_ADDA_RST_POS      18
#define RPL_RXADC_SATDET_EN_BIT     ((uint32_t)0x00002000)
#define RPL_RXADC_SATDET_EN_POS     13
#define RPL_MODEM_RX_EN_BIT         ((uint32_t)0x00001000)
#define RPL_MODEM_RX_EN_POS         12
#define RPL_RXADC_EN_BIT            ((uint32_t)0x00000800)
#define RPL_RXADC_EN_POS            11
#define RPL_RXADC_ON_BIT            ((uint32_t)0x00000400)
#define RPL_RXADC_ON_POS            10
#define RPL_AGC_EN_BIT              ((uint32_t)0x00000200)
#define RPL_AGC_EN_POS              9
#define RPL_RSSI_ON_BIT             ((uint32_t)0x00000100)
#define RPL_RSSI_ON_POS             8
#define RPL_RXFILT_ON_BIT           ((uint32_t)0x00000080)
#define RPL_RXFILT_ON_POS           7
#define RPL_RXLOBUF_ON_BIT          ((uint32_t)0x00000040)
#define RPL_RXLOBUF_ON_POS          6
#define RPL_RXMIX_ON_BIT            ((uint32_t)0x00000020)
#define RPL_RXMIX_ON_POS            5
#define RPL_RXLNA_ON_BIT            ((uint32_t)0x00000010)
#define RPL_RXLNA_ON_POS            4
#define RPL_RXRF_BIAS_ON_BIT        ((uint32_t)0x00000008)
#define RPL_RXRF_BIAS_ON_POS        3
#define RPL_RXGAIN_SEL_MASK         ((uint32_t)0x00000007)
#define RPL_RXGAIN_SEL_LSB          0
#define RPL_RXGAIN_SEL_WIDTH        ((uint32_t)0x00000003)

#define RPL_RXCTRL_FORCE_RST        0x0
#define RPL_RXGAIN_FORCE_RST        0x0
#define RPL_FORCE_ADDA_RST_RST      0x0
#define RPL_RSSI_CLK_INV_EN_RST     0x0
#define RPL_RXADC_CLK_INV_EN_RST    0x0
#define RPL_N_LDO_ADDA_RST_RST      0x0
#define RPL_RXADC_SATDET_EN_RST     0x0
#define RPL_MODEM_RX_EN_RST         0x0
#define RPL_RXADC_EN_RST            0x0
#define RPL_RXADC_ON_RST            0x0
#define RPL_AGC_EN_RST              0x0
#define RPL_RSSI_ON_RST             0x0
#define RPL_RXFILT_ON_RST           0x0
#define RPL_RXLOBUF_ON_RST          0x0
#define RPL_RXMIX_ON_RST            0x0
#define RPL_RXLNA_ON_RST            0x0
#define RPL_RXRF_BIAS_ON_RST        0x0
#define RPL_RXGAIN_SEL_RST          0x0

__INLINE void rpl_rfrxcntl_pack(uint8_t rxctrlforce, uint8_t rxgainforce, uint8_t forceaddarst, uint8_t rssiclkinven, uint8_t rxadcclkinven, uint8_t nldoaddarst, uint8_t rxadcsatdeten, uint8_t modemrxen, uint8_t rxadcen, uint8_t rxadcon, uint8_t agcen, uint8_t rssion, uint8_t rxfilton, uint8_t rxlobufon, uint8_t rxmixon, uint8_t rxlnaon, uint8_t rxrfbiason, uint8_t rxgainsel)
{
    ASSERT_ERR((((uint32_t)rxctrlforce << 31) & ~((uint32_t)0x80000000)) == 0);
    ASSERT_ERR((((uint32_t)rxgainforce << 30) & ~((uint32_t)0x40000000)) == 0);
    ASSERT_ERR((((uint32_t)forceaddarst << 29) & ~((uint32_t)0x20000000)) == 0);
    ASSERT_ERR((((uint32_t)rssiclkinven << 25) & ~((uint32_t)0x02000000)) == 0);
    ASSERT_ERR((((uint32_t)rxadcclkinven << 24) & ~((uint32_t)0x01000000)) == 0);
    ASSERT_ERR((((uint32_t)nldoaddarst << 18) & ~((uint32_t)0x00040000)) == 0);
    ASSERT_ERR((((uint32_t)rxadcsatdeten << 13) & ~((uint32_t)0x00002000)) == 0);
    ASSERT_ERR((((uint32_t)modemrxen << 12) & ~((uint32_t)0x00001000)) == 0);
    ASSERT_ERR((((uint32_t)rxadcen << 11) & ~((uint32_t)0x00000800)) == 0);
    ASSERT_ERR((((uint32_t)rxadcon << 10) & ~((uint32_t)0x00000400)) == 0);
    ASSERT_ERR((((uint32_t)agcen << 9) & ~((uint32_t)0x00000200)) == 0);
    ASSERT_ERR((((uint32_t)rssion << 8) & ~((uint32_t)0x00000100)) == 0);
    ASSERT_ERR((((uint32_t)rxfilton << 7) & ~((uint32_t)0x00000080)) == 0);
    ASSERT_ERR((((uint32_t)rxlobufon << 6) & ~((uint32_t)0x00000040)) == 0);
    ASSERT_ERR((((uint32_t)rxmixon << 5) & ~((uint32_t)0x00000020)) == 0);
    ASSERT_ERR((((uint32_t)rxlnaon << 4) & ~((uint32_t)0x00000010)) == 0);
    ASSERT_ERR((((uint32_t)rxrfbiason << 3) & ~((uint32_t)0x00000008)) == 0);
    ASSERT_ERR((((uint32_t)rxgainsel << 0) & ~((uint32_t)0x00000007)) == 0);
    REG_RPL_WR(RPL_RFRXCNTL_ADDR, ((uint32_t)rxctrlforce << 31) | ((uint32_t)rxgainforce << 30) | ((uint32_t)forceaddarst << 29) | ((uint32_t)rssiclkinven << 25) | ((uint32_t)rxadcclkinven << 24) | ((uint32_t)nldoaddarst << 18) | ((uint32_t)rxadcsatdeten << 13) | ((uint32_t)modemrxen << 12) | ((uint32_t)rxadcen << 11) | ((uint32_t)rxadcon << 10) | ((uint32_t)agcen << 9) | ((uint32_t)rssion << 8) | ((uint32_t)rxfilton << 7) | ((uint32_t)rxlobufon << 6) | ((uint32_t)rxmixon << 5) | ((uint32_t)rxlnaon << 4) | ((uint32_t)rxrfbiason << 3) | ((uint32_t)rxgainsel << 0));
}

__INLINE void rpl_rfrxcntl_unpack(uint8_t *rxctrlforce, uint8_t *rxgainforce, uint8_t *forceaddarst, uint8_t *rssiclkinven, uint8_t *rxadcclkinven, uint8_t *nldoaddarst, uint8_t *rxadcsatdeten, uint8_t *modemrxen, uint8_t *rxadcen, uint8_t *rxadcon, uint8_t *agcen, uint8_t *rssion, uint8_t *rxfilton, uint8_t *rxlobufon, uint8_t *rxmixon, uint8_t *rxlnaon, uint8_t *rxrfbiason, uint8_t *rxgainsel)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFRXCNTL_ADDR);

    *rxctrlforce = (localVal & ((uint32_t)0x80000000)) >> 31;
    *rxgainforce = (localVal & ((uint32_t)0x40000000)) >> 30;
    *forceaddarst = (localVal & ((uint32_t)0x20000000)) >> 29;
    *rssiclkinven = (localVal & ((uint32_t)0x02000000)) >> 25;
    *rxadcclkinven = (localVal & ((uint32_t)0x01000000)) >> 24;
    *nldoaddarst = (localVal & ((uint32_t)0x00040000)) >> 18;
    *rxadcsatdeten = (localVal & ((uint32_t)0x00002000)) >> 13;
    *modemrxen = (localVal & ((uint32_t)0x00001000)) >> 12;
    *rxadcen = (localVal & ((uint32_t)0x00000800)) >> 11;
    *rxadcon = (localVal & ((uint32_t)0x00000400)) >> 10;
    *agcen = (localVal & ((uint32_t)0x00000200)) >> 9;
    *rssion = (localVal & ((uint32_t)0x00000100)) >> 8;
    *rxfilton = (localVal & ((uint32_t)0x00000080)) >> 7;
    *rxlobufon = (localVal & ((uint32_t)0x00000040)) >> 6;
    *rxmixon = (localVal & ((uint32_t)0x00000020)) >> 5;
    *rxlnaon = (localVal & ((uint32_t)0x00000010)) >> 4;
    *rxrfbiason = (localVal & ((uint32_t)0x00000008)) >> 3;
    *rxgainsel = (localVal & ((uint32_t)0x00000007)) >> 0;
}

__INLINE uint8_t rpl_rxctrl_force_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFRXCNTL_ADDR);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE void rpl_rxctrl_force_setf(uint8_t rxctrlforce)
{
    ASSERT_ERR((((uint32_t)rxctrlforce << 31) & ~((uint32_t)0x80000000)) == 0);
    REG_RPL_WR(RPL_RFRXCNTL_ADDR, (REG_RPL_RD(RPL_RFRXCNTL_ADDR) & ~((uint32_t)0x80000000)) | ((uint32_t)rxctrlforce << 31));
}

__INLINE uint8_t rpl_rxgain_force_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFRXCNTL_ADDR);
    return ((localVal & ((uint32_t)0x40000000)) >> 30);
}

__INLINE void rpl_rxgain_force_setf(uint8_t rxgainforce)
{
    ASSERT_ERR((((uint32_t)rxgainforce << 30) & ~((uint32_t)0x40000000)) == 0);
    REG_RPL_WR(RPL_RFRXCNTL_ADDR, (REG_RPL_RD(RPL_RFRXCNTL_ADDR) & ~((uint32_t)0x40000000)) | ((uint32_t)rxgainforce << 30));
}

__INLINE uint8_t rpl_force_adda_rst_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFRXCNTL_ADDR);
    return ((localVal & ((uint32_t)0x20000000)) >> 29);
}

__INLINE void rpl_force_adda_rst_setf(uint8_t forceaddarst)
{
    ASSERT_ERR((((uint32_t)forceaddarst << 29) & ~((uint32_t)0x20000000)) == 0);
    REG_RPL_WR(RPL_RFRXCNTL_ADDR, (REG_RPL_RD(RPL_RFRXCNTL_ADDR) & ~((uint32_t)0x20000000)) | ((uint32_t)forceaddarst << 29));
}

__INLINE uint8_t rpl_rssi_clk_inv_en_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFRXCNTL_ADDR);
    return ((localVal & ((uint32_t)0x02000000)) >> 25);
}

__INLINE void rpl_rssi_clk_inv_en_setf(uint8_t rssiclkinven)
{
    ASSERT_ERR((((uint32_t)rssiclkinven << 25) & ~((uint32_t)0x02000000)) == 0);
    REG_RPL_WR(RPL_RFRXCNTL_ADDR, (REG_RPL_RD(RPL_RFRXCNTL_ADDR) & ~((uint32_t)0x02000000)) | ((uint32_t)rssiclkinven << 25));
}

__INLINE uint8_t rpl_rxadc_clk_inv_en_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFRXCNTL_ADDR);
    return ((localVal & ((uint32_t)0x01000000)) >> 24);
}

__INLINE void rpl_rxadc_clk_inv_en_setf(uint8_t rxadcclkinven)
{
    ASSERT_ERR((((uint32_t)rxadcclkinven << 24) & ~((uint32_t)0x01000000)) == 0);
    REG_RPL_WR(RPL_RFRXCNTL_ADDR, (REG_RPL_RD(RPL_RFRXCNTL_ADDR) & ~((uint32_t)0x01000000)) | ((uint32_t)rxadcclkinven << 24));
}

__INLINE uint8_t rpl_n_ldo_adda_rst_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFRXCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00040000)) >> 18);
}

__INLINE void rpl_n_ldo_adda_rst_setf(uint8_t nldoaddarst)
{
    ASSERT_ERR((((uint32_t)nldoaddarst << 18) & ~((uint32_t)0x00040000)) == 0);
    REG_RPL_WR(RPL_RFRXCNTL_ADDR, (REG_RPL_RD(RPL_RFRXCNTL_ADDR) & ~((uint32_t)0x00040000)) | ((uint32_t)nldoaddarst << 18));
}

__INLINE uint8_t rpl_rxadc_satdet_en_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFRXCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00002000)) >> 13);
}

__INLINE void rpl_rxadc_satdet_en_setf(uint8_t rxadcsatdeten)
{
    ASSERT_ERR((((uint32_t)rxadcsatdeten << 13) & ~((uint32_t)0x00002000)) == 0);
    REG_RPL_WR(RPL_RFRXCNTL_ADDR, (REG_RPL_RD(RPL_RFRXCNTL_ADDR) & ~((uint32_t)0x00002000)) | ((uint32_t)rxadcsatdeten << 13));
}

__INLINE uint8_t rpl_modem_rx_en_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFRXCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00001000)) >> 12);
}

__INLINE void rpl_modem_rx_en_setf(uint8_t modemrxen)
{
    ASSERT_ERR((((uint32_t)modemrxen << 12) & ~((uint32_t)0x00001000)) == 0);
    REG_RPL_WR(RPL_RFRXCNTL_ADDR, (REG_RPL_RD(RPL_RFRXCNTL_ADDR) & ~((uint32_t)0x00001000)) | ((uint32_t)modemrxen << 12));
}

__INLINE uint8_t rpl_rxadc_en_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFRXCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000800)) >> 11);
}

__INLINE void rpl_rxadc_en_setf(uint8_t rxadcen)
{
    ASSERT_ERR((((uint32_t)rxadcen << 11) & ~((uint32_t)0x00000800)) == 0);
    REG_RPL_WR(RPL_RFRXCNTL_ADDR, (REG_RPL_RD(RPL_RFRXCNTL_ADDR) & ~((uint32_t)0x00000800)) | ((uint32_t)rxadcen << 11));
}

__INLINE uint8_t rpl_rxadc_on_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFRXCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000400)) >> 10);
}

__INLINE void rpl_rxadc_on_setf(uint8_t rxadcon)
{
    ASSERT_ERR((((uint32_t)rxadcon << 10) & ~((uint32_t)0x00000400)) == 0);
    REG_RPL_WR(RPL_RFRXCNTL_ADDR, (REG_RPL_RD(RPL_RFRXCNTL_ADDR) & ~((uint32_t)0x00000400)) | ((uint32_t)rxadcon << 10));
}

__INLINE uint8_t rpl_agc_en_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFRXCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000200)) >> 9);
}

__INLINE void rpl_agc_en_setf(uint8_t agcen)
{
    ASSERT_ERR((((uint32_t)agcen << 9) & ~((uint32_t)0x00000200)) == 0);
    REG_RPL_WR(RPL_RFRXCNTL_ADDR, (REG_RPL_RD(RPL_RFRXCNTL_ADDR) & ~((uint32_t)0x00000200)) | ((uint32_t)agcen << 9));
}

__INLINE uint8_t rpl_rssi_on_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFRXCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000100)) >> 8);
}

__INLINE void rpl_rssi_on_setf(uint8_t rssion)
{
    ASSERT_ERR((((uint32_t)rssion << 8) & ~((uint32_t)0x00000100)) == 0);
    REG_RPL_WR(RPL_RFRXCNTL_ADDR, (REG_RPL_RD(RPL_RFRXCNTL_ADDR) & ~((uint32_t)0x00000100)) | ((uint32_t)rssion << 8));
}

__INLINE uint8_t rpl_rxfilt_on_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFRXCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000080)) >> 7);
}

__INLINE void rpl_rxfilt_on_setf(uint8_t rxfilton)
{
    ASSERT_ERR((((uint32_t)rxfilton << 7) & ~((uint32_t)0x00000080)) == 0);
    REG_RPL_WR(RPL_RFRXCNTL_ADDR, (REG_RPL_RD(RPL_RFRXCNTL_ADDR) & ~((uint32_t)0x00000080)) | ((uint32_t)rxfilton << 7));
}

__INLINE uint8_t rpl_rxlobuf_on_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFRXCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000040)) >> 6);
}

__INLINE void rpl_rxlobuf_on_setf(uint8_t rxlobufon)
{
    ASSERT_ERR((((uint32_t)rxlobufon << 6) & ~((uint32_t)0x00000040)) == 0);
    REG_RPL_WR(RPL_RFRXCNTL_ADDR, (REG_RPL_RD(RPL_RFRXCNTL_ADDR) & ~((uint32_t)0x00000040)) | ((uint32_t)rxlobufon << 6));
}

__INLINE uint8_t rpl_rxmix_on_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFRXCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000020)) >> 5);
}

__INLINE void rpl_rxmix_on_setf(uint8_t rxmixon)
{
    ASSERT_ERR((((uint32_t)rxmixon << 5) & ~((uint32_t)0x00000020)) == 0);
    REG_RPL_WR(RPL_RFRXCNTL_ADDR, (REG_RPL_RD(RPL_RFRXCNTL_ADDR) & ~((uint32_t)0x00000020)) | ((uint32_t)rxmixon << 5));
}

__INLINE uint8_t rpl_rxlna_on_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFRXCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE void rpl_rxlna_on_setf(uint8_t rxlnaon)
{
    ASSERT_ERR((((uint32_t)rxlnaon << 4) & ~((uint32_t)0x00000010)) == 0);
    REG_RPL_WR(RPL_RFRXCNTL_ADDR, (REG_RPL_RD(RPL_RFRXCNTL_ADDR) & ~((uint32_t)0x00000010)) | ((uint32_t)rxlnaon << 4));
}

__INLINE uint8_t rpl_rxrf_bias_on_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFRXCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

__INLINE void rpl_rxrf_bias_on_setf(uint8_t rxrfbiason)
{
    ASSERT_ERR((((uint32_t)rxrfbiason << 3) & ~((uint32_t)0x00000008)) == 0);
    REG_RPL_WR(RPL_RFRXCNTL_ADDR, (REG_RPL_RD(RPL_RFRXCNTL_ADDR) & ~((uint32_t)0x00000008)) | ((uint32_t)rxrfbiason << 3));
}

__INLINE uint8_t rpl_rxgain_sel_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFRXCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000007)) >> 0);
}

__INLINE void rpl_rxgain_sel_setf(uint8_t rxgainsel)
{
    ASSERT_ERR((((uint32_t)rxgainsel << 0) & ~((uint32_t)0x00000007)) == 0);
    REG_RPL_WR(RPL_RFRXCNTL_ADDR, (REG_RPL_RD(RPL_RFRXCNTL_ADDR) & ~((uint32_t)0x00000007)) | ((uint32_t)rxgainsel << 0));
}

/**
 * @brief RFRXCONFIG register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     31         RXFILT_IF_EN   1
 *  25:24           RXMIX_TRIM   0x0
 *  23:22           RXLNA_TRIM   0x0
 *  21:20            RXLNA_CFG   0x0
 *  19:16           RXFILT_CFG   0x0
 *  15:08            RXADC_CFG   0x3
 *  07:00             RSSI_CFG   0x3B
 * </pre>
 */
#define RPL_RFRXCONFIG_ADDR   0x0000006E
#define RPL_RFRXCONFIG_OFFSET 0x0000006E
#define RPL_RFRXCONFIG_INDEX  0x0000006E
#define RPL_RFRXCONFIG_RESET  0x8000033B

__INLINE uint32_t rpl_rfrxconfig_get(void)
{
    return REG_RPL_RD(RPL_RFRXCONFIG_ADDR);
}

__INLINE void rpl_rfrxconfig_set(uint32_t value)
{
    REG_RPL_WR(RPL_RFRXCONFIG_ADDR, value);
}

// field definitions
#define RPL_RXFILT_IF_EN_BIT    ((uint32_t)0x80000000)
#define RPL_RXFILT_IF_EN_POS    31
#define RPL_RXMIX_TRIM_MASK     ((uint32_t)0x03000000)
#define RPL_RXMIX_TRIM_LSB      24
#define RPL_RXMIX_TRIM_WIDTH    ((uint32_t)0x00000002)
#define RPL_RXLNA_TRIM_MASK     ((uint32_t)0x00C00000)
#define RPL_RXLNA_TRIM_LSB      22
#define RPL_RXLNA_TRIM_WIDTH    ((uint32_t)0x00000002)
#define RPL_RXLNA_CFG_MASK      ((uint32_t)0x00300000)
#define RPL_RXLNA_CFG_LSB       20
#define RPL_RXLNA_CFG_WIDTH     ((uint32_t)0x00000002)
#define RPL_RXFILT_CFG_MASK     ((uint32_t)0x000F0000)
#define RPL_RXFILT_CFG_LSB      16
#define RPL_RXFILT_CFG_WIDTH    ((uint32_t)0x00000004)
#define RPL_RXADC_CFG_MASK      ((uint32_t)0x0000FF00)
#define RPL_RXADC_CFG_LSB       8
#define RPL_RXADC_CFG_WIDTH     ((uint32_t)0x00000008)
#define RPL_RSSI_CFG_MASK       ((uint32_t)0x000000FF)
#define RPL_RSSI_CFG_LSB        0
#define RPL_RSSI_CFG_WIDTH      ((uint32_t)0x00000008)

#define RPL_RXFILT_IF_EN_RST    0x1
#define RPL_RXMIX_TRIM_RST      0x0
#define RPL_RXLNA_TRIM_RST      0x0
#define RPL_RXLNA_CFG_RST       0x0
#define RPL_RXFILT_CFG_RST      0x0
#define RPL_RXADC_CFG_RST       0x3
#define RPL_RSSI_CFG_RST        0x3B

__INLINE void rpl_rfrxconfig_pack(uint8_t rxfiltifen, uint8_t rxmixtrim, uint8_t rxlnatrim, uint8_t rxlnacfg, uint8_t rxfiltcfg, uint8_t rxadccfg, uint8_t rssicfg)
{
    ASSERT_ERR((((uint32_t)rxfiltifen << 31) & ~((uint32_t)0x80000000)) == 0);
    ASSERT_ERR((((uint32_t)rxmixtrim << 24) & ~((uint32_t)0x03000000)) == 0);
    ASSERT_ERR((((uint32_t)rxlnatrim << 22) & ~((uint32_t)0x00C00000)) == 0);
    ASSERT_ERR((((uint32_t)rxlnacfg << 20) & ~((uint32_t)0x00300000)) == 0);
    ASSERT_ERR((((uint32_t)rxfiltcfg << 16) & ~((uint32_t)0x000F0000)) == 0);
    ASSERT_ERR((((uint32_t)rxadccfg << 8) & ~((uint32_t)0x0000FF00)) == 0);
    ASSERT_ERR((((uint32_t)rssicfg << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_RPL_WR(RPL_RFRXCONFIG_ADDR, ((uint32_t)rxfiltifen << 31) | ((uint32_t)rxmixtrim << 24) | ((uint32_t)rxlnatrim << 22) | ((uint32_t)rxlnacfg << 20) | ((uint32_t)rxfiltcfg << 16) | ((uint32_t)rxadccfg << 8) | ((uint32_t)rssicfg << 0));
}

__INLINE void rpl_rfrxconfig_unpack(uint8_t *rxfiltifen, uint8_t *rxmixtrim, uint8_t *rxlnatrim, uint8_t *rxlnacfg, uint8_t *rxfiltcfg, uint8_t *rxadccfg, uint8_t *rssicfg)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFRXCONFIG_ADDR);

    *rxfiltifen = (localVal & ((uint32_t)0x80000000)) >> 31;
    *rxmixtrim = (localVal & ((uint32_t)0x03000000)) >> 24;
    *rxlnatrim = (localVal & ((uint32_t)0x00C00000)) >> 22;
    *rxlnacfg = (localVal & ((uint32_t)0x00300000)) >> 20;
    *rxfiltcfg = (localVal & ((uint32_t)0x000F0000)) >> 16;
    *rxadccfg = (localVal & ((uint32_t)0x0000FF00)) >> 8;
    *rssicfg = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint8_t rpl_rxfilt_if_en_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFRXCONFIG_ADDR);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE void rpl_rxfilt_if_en_setf(uint8_t rxfiltifen)
{
    ASSERT_ERR((((uint32_t)rxfiltifen << 31) & ~((uint32_t)0x80000000)) == 0);
    REG_RPL_WR(RPL_RFRXCONFIG_ADDR, (REG_RPL_RD(RPL_RFRXCONFIG_ADDR) & ~((uint32_t)0x80000000)) | ((uint32_t)rxfiltifen << 31));
}

__INLINE uint8_t rpl_rxmix_trim_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFRXCONFIG_ADDR);
    return ((localVal & ((uint32_t)0x03000000)) >> 24);
}

__INLINE void rpl_rxmix_trim_setf(uint8_t rxmixtrim)
{
    ASSERT_ERR((((uint32_t)rxmixtrim << 24) & ~((uint32_t)0x03000000)) == 0);
    REG_RPL_WR(RPL_RFRXCONFIG_ADDR, (REG_RPL_RD(RPL_RFRXCONFIG_ADDR) & ~((uint32_t)0x03000000)) | ((uint32_t)rxmixtrim << 24));
}

__INLINE uint8_t rpl_rxlna_trim_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFRXCONFIG_ADDR);
    return ((localVal & ((uint32_t)0x00C00000)) >> 22);
}

__INLINE void rpl_rxlna_trim_setf(uint8_t rxlnatrim)
{
    ASSERT_ERR((((uint32_t)rxlnatrim << 22) & ~((uint32_t)0x00C00000)) == 0);
    REG_RPL_WR(RPL_RFRXCONFIG_ADDR, (REG_RPL_RD(RPL_RFRXCONFIG_ADDR) & ~((uint32_t)0x00C00000)) | ((uint32_t)rxlnatrim << 22));
}

__INLINE uint8_t rpl_rxlna_cfg_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFRXCONFIG_ADDR);
    return ((localVal & ((uint32_t)0x00300000)) >> 20);
}

__INLINE void rpl_rxlna_cfg_setf(uint8_t rxlnacfg)
{
    ASSERT_ERR((((uint32_t)rxlnacfg << 20) & ~((uint32_t)0x00300000)) == 0);
    REG_RPL_WR(RPL_RFRXCONFIG_ADDR, (REG_RPL_RD(RPL_RFRXCONFIG_ADDR) & ~((uint32_t)0x00300000)) | ((uint32_t)rxlnacfg << 20));
}

__INLINE uint8_t rpl_rxfilt_cfg_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFRXCONFIG_ADDR);
    return ((localVal & ((uint32_t)0x000F0000)) >> 16);
}

__INLINE void rpl_rxfilt_cfg_setf(uint8_t rxfiltcfg)
{
    ASSERT_ERR((((uint32_t)rxfiltcfg << 16) & ~((uint32_t)0x000F0000)) == 0);
    REG_RPL_WR(RPL_RFRXCONFIG_ADDR, (REG_RPL_RD(RPL_RFRXCONFIG_ADDR) & ~((uint32_t)0x000F0000)) | ((uint32_t)rxfiltcfg << 16));
}

__INLINE uint8_t rpl_rxadc_cfg_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFRXCONFIG_ADDR);
    return ((localVal & ((uint32_t)0x0000FF00)) >> 8);
}

__INLINE void rpl_rxadc_cfg_setf(uint8_t rxadccfg)
{
    ASSERT_ERR((((uint32_t)rxadccfg << 8) & ~((uint32_t)0x0000FF00)) == 0);
    REG_RPL_WR(RPL_RFRXCONFIG_ADDR, (REG_RPL_RD(RPL_RFRXCONFIG_ADDR) & ~((uint32_t)0x0000FF00)) | ((uint32_t)rxadccfg << 8));
}

__INLINE uint8_t rpl_rssi_cfg_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFRXCONFIG_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

__INLINE void rpl_rssi_cfg_setf(uint8_t rssicfg)
{
    ASSERT_ERR((((uint32_t)rssicfg << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_RPL_WR(RPL_RFRXCONFIG_ADDR, (REG_RPL_RD(RPL_RFRXCONFIG_ADDR) & ~((uint32_t)0x000000FF)) | ((uint32_t)rssicfg << 0));
}

/**
 * @brief RFTXCNTL0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     31         TXCTRL_FORCE   0
 *     30    TXGAIN_GFSK_FORCE   0
 *     29     TXGAIN_EDR_FORCE   0
 *     27     TXDAC_CLK_INV_EN   0
 *     26   TXDC_CAL_TIMEOUT_DSB   0
 *     25          TXDC_CAL_EN   0
 *     24       TXDC_CAL_START   0
 *     21           SD_FIFO_EN   0
 *     20          RAMP_GEN_EN   0
 *     19          MODEM_TX_EN   0
 *     18            FMTX2P_ON   0
 *     17           TXFILTQ_ON   0
 *     16           TXFILTI_ON   0
 *     15            TXDACQ_ON   0
 *     14            TXDACI_ON   0
 *     13              FMTX_ON   0
 *     12              TXPA_ON   0
 *     11             TXPA_SEL   0
 *     10       TXRF_DETECT_ON   0
 *     09             TXMIX_ON   0
 *     08           TXLOBUF_ON   0
 *  07:04       TXGAIN_EDR_SEL   0x0
 *  03:00      TXGAIN_GFSK_SEL   0x0
 * </pre>
 */
#define RPL_RFTXCNTL0_ADDR   0x0000006F
#define RPL_RFTXCNTL0_OFFSET 0x0000006F
#define RPL_RFTXCNTL0_INDEX  0x0000006F
#define RPL_RFTXCNTL0_RESET  0x00000000

__INLINE uint32_t rpl_rftxcntl0_get(void)
{
    return REG_RPL_RD(RPL_RFTXCNTL0_ADDR);
}

__INLINE void rpl_rftxcntl0_set(uint32_t value)
{
    REG_RPL_WR(RPL_RFTXCNTL0_ADDR, value);
}

// field definitions
#define RPL_TXCTRL_FORCE_BIT            ((uint32_t)0x80000000)
#define RPL_TXCTRL_FORCE_POS            31
#define RPL_TXGAIN_GFSK_FORCE_BIT       ((uint32_t)0x40000000)
#define RPL_TXGAIN_GFSK_FORCE_POS       30
#define RPL_TXGAIN_EDR_FORCE_BIT        ((uint32_t)0x20000000)
#define RPL_TXGAIN_EDR_FORCE_POS        29
#define RPL_TXDAC_CLK_INV_EN_BIT        ((uint32_t)0x08000000)
#define RPL_TXDAC_CLK_INV_EN_POS        27
#define RPL_TXDC_CAL_TIMEOUT_DSB_BIT    ((uint32_t)0x04000000)
#define RPL_TXDC_CAL_TIMEOUT_DSB_POS    26
#define RPL_TXDC_CAL_EN_BIT             ((uint32_t)0x02000000)
#define RPL_TXDC_CAL_EN_POS             25
#define RPL_TXDC_CAL_START_BIT          ((uint32_t)0x01000000)
#define RPL_TXDC_CAL_START_POS          24
#define RPL_SD_FIFO_EN_BIT              ((uint32_t)0x00200000)
#define RPL_SD_FIFO_EN_POS              21
#define RPL_RAMP_GEN_EN_BIT             ((uint32_t)0x00100000)
#define RPL_RAMP_GEN_EN_POS             20
#define RPL_MODEM_TX_EN_BIT             ((uint32_t)0x00080000)
#define RPL_MODEM_TX_EN_POS             19
#define RPL_FMTX2P_ON_BIT               ((uint32_t)0x00040000)
#define RPL_FMTX2P_ON_POS               18
#define RPL_TXFILTQ_ON_BIT              ((uint32_t)0x00020000)
#define RPL_TXFILTQ_ON_POS              17
#define RPL_TXFILTI_ON_BIT              ((uint32_t)0x00010000)
#define RPL_TXFILTI_ON_POS              16
#define RPL_TXDACQ_ON_BIT               ((uint32_t)0x00008000)
#define RPL_TXDACQ_ON_POS               15
#define RPL_TXDACI_ON_BIT               ((uint32_t)0x00004000)
#define RPL_TXDACI_ON_POS               14
#define RPL_FMTX_ON_BIT                 ((uint32_t)0x00002000)
#define RPL_FMTX_ON_POS                 13
#define RPL_TXPA_ON_BIT                 ((uint32_t)0x00001000)
#define RPL_TXPA_ON_POS                 12
#define RPL_TXPA_SEL_BIT                ((uint32_t)0x00000800)
#define RPL_TXPA_SEL_POS                11
#define RPL_TXRF_DETECT_ON_BIT          ((uint32_t)0x00000400)
#define RPL_TXRF_DETECT_ON_POS          10
#define RPL_TXMIX_ON_BIT                ((uint32_t)0x00000200)
#define RPL_TXMIX_ON_POS                9
#define RPL_TXLOBUF_ON_BIT              ((uint32_t)0x00000100)
#define RPL_TXLOBUF_ON_POS              8
#define RPL_TXGAIN_EDR_SEL_MASK         ((uint32_t)0x000000F0)
#define RPL_TXGAIN_EDR_SEL_LSB          4
#define RPL_TXGAIN_EDR_SEL_WIDTH        ((uint32_t)0x00000004)
#define RPL_TXGAIN_GFSK_SEL_MASK        ((uint32_t)0x0000000F)
#define RPL_TXGAIN_GFSK_SEL_LSB         0
#define RPL_TXGAIN_GFSK_SEL_WIDTH       ((uint32_t)0x00000004)

#define RPL_TXCTRL_FORCE_RST            0x0
#define RPL_TXGAIN_GFSK_FORCE_RST       0x0
#define RPL_TXGAIN_EDR_FORCE_RST        0x0
#define RPL_TXDAC_CLK_INV_EN_RST        0x0
#define RPL_TXDC_CAL_TIMEOUT_DSB_RST    0x0
#define RPL_TXDC_CAL_EN_RST             0x0
#define RPL_TXDC_CAL_START_RST          0x0
#define RPL_SD_FIFO_EN_RST              0x0
#define RPL_RAMP_GEN_EN_RST             0x0
#define RPL_MODEM_TX_EN_RST             0x0
#define RPL_FMTX2P_ON_RST               0x0
#define RPL_TXFILTQ_ON_RST              0x0
#define RPL_TXFILTI_ON_RST              0x0
#define RPL_TXDACQ_ON_RST               0x0
#define RPL_TXDACI_ON_RST               0x0
#define RPL_FMTX_ON_RST                 0x0
#define RPL_TXPA_ON_RST                 0x0
#define RPL_TXPA_SEL_RST                0x0
#define RPL_TXRF_DETECT_ON_RST          0x0
#define RPL_TXMIX_ON_RST                0x0
#define RPL_TXLOBUF_ON_RST              0x0
#define RPL_TXGAIN_EDR_SEL_RST          0x0
#define RPL_TXGAIN_GFSK_SEL_RST         0x0

__INLINE void rpl_rftxcntl0_pack(uint8_t txctrlforce, uint8_t txgaingfskforce, uint8_t txgainedrforce, uint8_t txdacclkinven, uint8_t txdccaltimeoutdsb, uint8_t txdccalen, uint8_t txdccalstart, uint8_t sdfifoen, uint8_t rampgenen, uint8_t modemtxen, uint8_t fmtx2pon, uint8_t txfiltqon, uint8_t txfiltion, uint8_t txdacqon, uint8_t txdacion, uint8_t fmtxon, uint8_t txpaon, uint8_t txpasel, uint8_t txrfdetecton, uint8_t txmixon, uint8_t txlobufon, uint8_t txgainedrsel, uint8_t txgaingfsksel)
{
    ASSERT_ERR((((uint32_t)txctrlforce << 31) & ~((uint32_t)0x80000000)) == 0);
    ASSERT_ERR((((uint32_t)txgaingfskforce << 30) & ~((uint32_t)0x40000000)) == 0);
    ASSERT_ERR((((uint32_t)txgainedrforce << 29) & ~((uint32_t)0x20000000)) == 0);
    ASSERT_ERR((((uint32_t)txdacclkinven << 27) & ~((uint32_t)0x08000000)) == 0);
    ASSERT_ERR((((uint32_t)txdccaltimeoutdsb << 26) & ~((uint32_t)0x04000000)) == 0);
    ASSERT_ERR((((uint32_t)txdccalen << 25) & ~((uint32_t)0x02000000)) == 0);
    ASSERT_ERR((((uint32_t)txdccalstart << 24) & ~((uint32_t)0x01000000)) == 0);
    ASSERT_ERR((((uint32_t)sdfifoen << 21) & ~((uint32_t)0x00200000)) == 0);
    ASSERT_ERR((((uint32_t)rampgenen << 20) & ~((uint32_t)0x00100000)) == 0);
    ASSERT_ERR((((uint32_t)modemtxen << 19) & ~((uint32_t)0x00080000)) == 0);
    ASSERT_ERR((((uint32_t)fmtx2pon << 18) & ~((uint32_t)0x00040000)) == 0);
    ASSERT_ERR((((uint32_t)txfiltqon << 17) & ~((uint32_t)0x00020000)) == 0);
    ASSERT_ERR((((uint32_t)txfiltion << 16) & ~((uint32_t)0x00010000)) == 0);
    ASSERT_ERR((((uint32_t)txdacqon << 15) & ~((uint32_t)0x00008000)) == 0);
    ASSERT_ERR((((uint32_t)txdacion << 14) & ~((uint32_t)0x00004000)) == 0);
    ASSERT_ERR((((uint32_t)fmtxon << 13) & ~((uint32_t)0x00002000)) == 0);
    ASSERT_ERR((((uint32_t)txpaon << 12) & ~((uint32_t)0x00001000)) == 0);
    ASSERT_ERR((((uint32_t)txpasel << 11) & ~((uint32_t)0x00000800)) == 0);
    ASSERT_ERR((((uint32_t)txrfdetecton << 10) & ~((uint32_t)0x00000400)) == 0);
    ASSERT_ERR((((uint32_t)txmixon << 9) & ~((uint32_t)0x00000200)) == 0);
    ASSERT_ERR((((uint32_t)txlobufon << 8) & ~((uint32_t)0x00000100)) == 0);
    ASSERT_ERR((((uint32_t)txgainedrsel << 4) & ~((uint32_t)0x000000F0)) == 0);
    ASSERT_ERR((((uint32_t)txgaingfsksel << 0) & ~((uint32_t)0x0000000F)) == 0);
    REG_RPL_WR(RPL_RFTXCNTL0_ADDR, ((uint32_t)txctrlforce << 31) | ((uint32_t)txgaingfskforce << 30) | ((uint32_t)txgainedrforce << 29) | ((uint32_t)txdacclkinven << 27) | ((uint32_t)txdccaltimeoutdsb << 26) | ((uint32_t)txdccalen << 25) | ((uint32_t)txdccalstart << 24) | ((uint32_t)sdfifoen << 21) | ((uint32_t)rampgenen << 20) | ((uint32_t)modemtxen << 19) | ((uint32_t)fmtx2pon << 18) | ((uint32_t)txfiltqon << 17) | ((uint32_t)txfiltion << 16) | ((uint32_t)txdacqon << 15) | ((uint32_t)txdacion << 14) | ((uint32_t)fmtxon << 13) | ((uint32_t)txpaon << 12) | ((uint32_t)txpasel << 11) | ((uint32_t)txrfdetecton << 10) | ((uint32_t)txmixon << 9) | ((uint32_t)txlobufon << 8) | ((uint32_t)txgainedrsel << 4) | ((uint32_t)txgaingfsksel << 0));
}

__INLINE void rpl_rftxcntl0_unpack(uint8_t *txctrlforce, uint8_t *txgaingfskforce, uint8_t *txgainedrforce, uint8_t *txdacclkinven, uint8_t *txdccaltimeoutdsb, uint8_t *txdccalen, uint8_t *txdccalstart, uint8_t *sdfifoen, uint8_t *rampgenen, uint8_t *modemtxen, uint8_t *fmtx2pon, uint8_t *txfiltqon, uint8_t *txfiltion, uint8_t *txdacqon, uint8_t *txdacion, uint8_t *fmtxon, uint8_t *txpaon, uint8_t *txpasel, uint8_t *txrfdetecton, uint8_t *txmixon, uint8_t *txlobufon, uint8_t *txgainedrsel, uint8_t *txgaingfsksel)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXCNTL0_ADDR);

    *txctrlforce = (localVal & ((uint32_t)0x80000000)) >> 31;
    *txgaingfskforce = (localVal & ((uint32_t)0x40000000)) >> 30;
    *txgainedrforce = (localVal & ((uint32_t)0x20000000)) >> 29;
    *txdacclkinven = (localVal & ((uint32_t)0x08000000)) >> 27;
    *txdccaltimeoutdsb = (localVal & ((uint32_t)0x04000000)) >> 26;
    *txdccalen = (localVal & ((uint32_t)0x02000000)) >> 25;
    *txdccalstart = (localVal & ((uint32_t)0x01000000)) >> 24;
    *sdfifoen = (localVal & ((uint32_t)0x00200000)) >> 21;
    *rampgenen = (localVal & ((uint32_t)0x00100000)) >> 20;
    *modemtxen = (localVal & ((uint32_t)0x00080000)) >> 19;
    *fmtx2pon = (localVal & ((uint32_t)0x00040000)) >> 18;
    *txfiltqon = (localVal & ((uint32_t)0x00020000)) >> 17;
    *txfiltion = (localVal & ((uint32_t)0x00010000)) >> 16;
    *txdacqon = (localVal & ((uint32_t)0x00008000)) >> 15;
    *txdacion = (localVal & ((uint32_t)0x00004000)) >> 14;
    *fmtxon = (localVal & ((uint32_t)0x00002000)) >> 13;
    *txpaon = (localVal & ((uint32_t)0x00001000)) >> 12;
    *txpasel = (localVal & ((uint32_t)0x00000800)) >> 11;
    *txrfdetecton = (localVal & ((uint32_t)0x00000400)) >> 10;
    *txmixon = (localVal & ((uint32_t)0x00000200)) >> 9;
    *txlobufon = (localVal & ((uint32_t)0x00000100)) >> 8;
    *txgainedrsel = (localVal & ((uint32_t)0x000000F0)) >> 4;
    *txgaingfsksel = (localVal & ((uint32_t)0x0000000F)) >> 0;
}

__INLINE uint8_t rpl_txctrl_force_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE void rpl_txctrl_force_setf(uint8_t txctrlforce)
{
    ASSERT_ERR((((uint32_t)txctrlforce << 31) & ~((uint32_t)0x80000000)) == 0);
    REG_RPL_WR(RPL_RFTXCNTL0_ADDR, (REG_RPL_RD(RPL_RFTXCNTL0_ADDR) & ~((uint32_t)0x80000000)) | ((uint32_t)txctrlforce << 31));
}

__INLINE uint8_t rpl_txgain_gfsk_force_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x40000000)) >> 30);
}

__INLINE void rpl_txgain_gfsk_force_setf(uint8_t txgaingfskforce)
{
    ASSERT_ERR((((uint32_t)txgaingfskforce << 30) & ~((uint32_t)0x40000000)) == 0);
    REG_RPL_WR(RPL_RFTXCNTL0_ADDR, (REG_RPL_RD(RPL_RFTXCNTL0_ADDR) & ~((uint32_t)0x40000000)) | ((uint32_t)txgaingfskforce << 30));
}

__INLINE uint8_t rpl_txgain_edr_force_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x20000000)) >> 29);
}

__INLINE void rpl_txgain_edr_force_setf(uint8_t txgainedrforce)
{
    ASSERT_ERR((((uint32_t)txgainedrforce << 29) & ~((uint32_t)0x20000000)) == 0);
    REG_RPL_WR(RPL_RFTXCNTL0_ADDR, (REG_RPL_RD(RPL_RFTXCNTL0_ADDR) & ~((uint32_t)0x20000000)) | ((uint32_t)txgainedrforce << 29));
}

__INLINE uint8_t rpl_txdac_clk_inv_en_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x08000000)) >> 27);
}

__INLINE void rpl_txdac_clk_inv_en_setf(uint8_t txdacclkinven)
{
    ASSERT_ERR((((uint32_t)txdacclkinven << 27) & ~((uint32_t)0x08000000)) == 0);
    REG_RPL_WR(RPL_RFTXCNTL0_ADDR, (REG_RPL_RD(RPL_RFTXCNTL0_ADDR) & ~((uint32_t)0x08000000)) | ((uint32_t)txdacclkinven << 27));
}

__INLINE uint8_t rpl_txdc_cal_timeout_dsb_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x04000000)) >> 26);
}

__INLINE void rpl_txdc_cal_timeout_dsb_setf(uint8_t txdccaltimeoutdsb)
{
    ASSERT_ERR((((uint32_t)txdccaltimeoutdsb << 26) & ~((uint32_t)0x04000000)) == 0);
    REG_RPL_WR(RPL_RFTXCNTL0_ADDR, (REG_RPL_RD(RPL_RFTXCNTL0_ADDR) & ~((uint32_t)0x04000000)) | ((uint32_t)txdccaltimeoutdsb << 26));
}

__INLINE uint8_t rpl_txdc_cal_en_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x02000000)) >> 25);
}

__INLINE void rpl_txdc_cal_en_setf(uint8_t txdccalen)
{
    ASSERT_ERR((((uint32_t)txdccalen << 25) & ~((uint32_t)0x02000000)) == 0);
    REG_RPL_WR(RPL_RFTXCNTL0_ADDR, (REG_RPL_RD(RPL_RFTXCNTL0_ADDR) & ~((uint32_t)0x02000000)) | ((uint32_t)txdccalen << 25));
}

__INLINE uint8_t rpl_txdc_cal_start_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x01000000)) >> 24);
}

__INLINE void rpl_txdc_cal_start_setf(uint8_t txdccalstart)
{
    ASSERT_ERR((((uint32_t)txdccalstart << 24) & ~((uint32_t)0x01000000)) == 0);
    REG_RPL_WR(RPL_RFTXCNTL0_ADDR, (REG_RPL_RD(RPL_RFTXCNTL0_ADDR) & ~((uint32_t)0x01000000)) | ((uint32_t)txdccalstart << 24));
}

__INLINE uint8_t rpl_sd_fifo_en_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00200000)) >> 21);
}

__INLINE void rpl_sd_fifo_en_setf(uint8_t sdfifoen)
{
    ASSERT_ERR((((uint32_t)sdfifoen << 21) & ~((uint32_t)0x00200000)) == 0);
    REG_RPL_WR(RPL_RFTXCNTL0_ADDR, (REG_RPL_RD(RPL_RFTXCNTL0_ADDR) & ~((uint32_t)0x00200000)) | ((uint32_t)sdfifoen << 21));
}

__INLINE uint8_t rpl_ramp_gen_en_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00100000)) >> 20);
}

__INLINE void rpl_ramp_gen_en_setf(uint8_t rampgenen)
{
    ASSERT_ERR((((uint32_t)rampgenen << 20) & ~((uint32_t)0x00100000)) == 0);
    REG_RPL_WR(RPL_RFTXCNTL0_ADDR, (REG_RPL_RD(RPL_RFTXCNTL0_ADDR) & ~((uint32_t)0x00100000)) | ((uint32_t)rampgenen << 20));
}

__INLINE uint8_t rpl_modem_tx_en_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00080000)) >> 19);
}

__INLINE void rpl_modem_tx_en_setf(uint8_t modemtxen)
{
    ASSERT_ERR((((uint32_t)modemtxen << 19) & ~((uint32_t)0x00080000)) == 0);
    REG_RPL_WR(RPL_RFTXCNTL0_ADDR, (REG_RPL_RD(RPL_RFTXCNTL0_ADDR) & ~((uint32_t)0x00080000)) | ((uint32_t)modemtxen << 19));
}

__INLINE uint8_t rpl_fmtx2p_on_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00040000)) >> 18);
}

__INLINE void rpl_fmtx2p_on_setf(uint8_t fmtx2pon)
{
    ASSERT_ERR((((uint32_t)fmtx2pon << 18) & ~((uint32_t)0x00040000)) == 0);
    REG_RPL_WR(RPL_RFTXCNTL0_ADDR, (REG_RPL_RD(RPL_RFTXCNTL0_ADDR) & ~((uint32_t)0x00040000)) | ((uint32_t)fmtx2pon << 18));
}

__INLINE uint8_t rpl_txfiltq_on_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00020000)) >> 17);
}

__INLINE void rpl_txfiltq_on_setf(uint8_t txfiltqon)
{
    ASSERT_ERR((((uint32_t)txfiltqon << 17) & ~((uint32_t)0x00020000)) == 0);
    REG_RPL_WR(RPL_RFTXCNTL0_ADDR, (REG_RPL_RD(RPL_RFTXCNTL0_ADDR) & ~((uint32_t)0x00020000)) | ((uint32_t)txfiltqon << 17));
}

__INLINE uint8_t rpl_txfilti_on_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00010000)) >> 16);
}

__INLINE void rpl_txfilti_on_setf(uint8_t txfiltion)
{
    ASSERT_ERR((((uint32_t)txfiltion << 16) & ~((uint32_t)0x00010000)) == 0);
    REG_RPL_WR(RPL_RFTXCNTL0_ADDR, (REG_RPL_RD(RPL_RFTXCNTL0_ADDR) & ~((uint32_t)0x00010000)) | ((uint32_t)txfiltion << 16));
}

__INLINE uint8_t rpl_txdacq_on_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00008000)) >> 15);
}

__INLINE void rpl_txdacq_on_setf(uint8_t txdacqon)
{
    ASSERT_ERR((((uint32_t)txdacqon << 15) & ~((uint32_t)0x00008000)) == 0);
    REG_RPL_WR(RPL_RFTXCNTL0_ADDR, (REG_RPL_RD(RPL_RFTXCNTL0_ADDR) & ~((uint32_t)0x00008000)) | ((uint32_t)txdacqon << 15));
}

__INLINE uint8_t rpl_txdaci_on_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00004000)) >> 14);
}

__INLINE void rpl_txdaci_on_setf(uint8_t txdacion)
{
    ASSERT_ERR((((uint32_t)txdacion << 14) & ~((uint32_t)0x00004000)) == 0);
    REG_RPL_WR(RPL_RFTXCNTL0_ADDR, (REG_RPL_RD(RPL_RFTXCNTL0_ADDR) & ~((uint32_t)0x00004000)) | ((uint32_t)txdacion << 14));
}

__INLINE uint8_t rpl_fmtx_on_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00002000)) >> 13);
}

__INLINE void rpl_fmtx_on_setf(uint8_t fmtxon)
{
    ASSERT_ERR((((uint32_t)fmtxon << 13) & ~((uint32_t)0x00002000)) == 0);
    REG_RPL_WR(RPL_RFTXCNTL0_ADDR, (REG_RPL_RD(RPL_RFTXCNTL0_ADDR) & ~((uint32_t)0x00002000)) | ((uint32_t)fmtxon << 13));
}

__INLINE uint8_t rpl_txpa_on_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00001000)) >> 12);
}

__INLINE void rpl_txpa_on_setf(uint8_t txpaon)
{
    ASSERT_ERR((((uint32_t)txpaon << 12) & ~((uint32_t)0x00001000)) == 0);
    REG_RPL_WR(RPL_RFTXCNTL0_ADDR, (REG_RPL_RD(RPL_RFTXCNTL0_ADDR) & ~((uint32_t)0x00001000)) | ((uint32_t)txpaon << 12));
}

__INLINE uint8_t rpl_txpa_sel_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00000800)) >> 11);
}

__INLINE void rpl_txpa_sel_setf(uint8_t txpasel)
{
    ASSERT_ERR((((uint32_t)txpasel << 11) & ~((uint32_t)0x00000800)) == 0);
    REG_RPL_WR(RPL_RFTXCNTL0_ADDR, (REG_RPL_RD(RPL_RFTXCNTL0_ADDR) & ~((uint32_t)0x00000800)) | ((uint32_t)txpasel << 11));
}

__INLINE uint8_t rpl_txrf_detect_on_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00000400)) >> 10);
}

__INLINE void rpl_txrf_detect_on_setf(uint8_t txrfdetecton)
{
    ASSERT_ERR((((uint32_t)txrfdetecton << 10) & ~((uint32_t)0x00000400)) == 0);
    REG_RPL_WR(RPL_RFTXCNTL0_ADDR, (REG_RPL_RD(RPL_RFTXCNTL0_ADDR) & ~((uint32_t)0x00000400)) | ((uint32_t)txrfdetecton << 10));
}

__INLINE uint8_t rpl_txmix_on_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00000200)) >> 9);
}

__INLINE void rpl_txmix_on_setf(uint8_t txmixon)
{
    ASSERT_ERR((((uint32_t)txmixon << 9) & ~((uint32_t)0x00000200)) == 0);
    REG_RPL_WR(RPL_RFTXCNTL0_ADDR, (REG_RPL_RD(RPL_RFTXCNTL0_ADDR) & ~((uint32_t)0x00000200)) | ((uint32_t)txmixon << 9));
}

__INLINE uint8_t rpl_txlobuf_on_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00000100)) >> 8);
}

__INLINE void rpl_txlobuf_on_setf(uint8_t txlobufon)
{
    ASSERT_ERR((((uint32_t)txlobufon << 8) & ~((uint32_t)0x00000100)) == 0);
    REG_RPL_WR(RPL_RFTXCNTL0_ADDR, (REG_RPL_RD(RPL_RFTXCNTL0_ADDR) & ~((uint32_t)0x00000100)) | ((uint32_t)txlobufon << 8));
}

__INLINE uint8_t rpl_txgain_edr_sel_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x000000F0)) >> 4);
}

__INLINE void rpl_txgain_edr_sel_setf(uint8_t txgainedrsel)
{
    ASSERT_ERR((((uint32_t)txgainedrsel << 4) & ~((uint32_t)0x000000F0)) == 0);
    REG_RPL_WR(RPL_RFTXCNTL0_ADDR, (REG_RPL_RD(RPL_RFTXCNTL0_ADDR) & ~((uint32_t)0x000000F0)) | ((uint32_t)txgainedrsel << 4));
}

__INLINE uint8_t rpl_txgain_gfsk_sel_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x0000000F)) >> 0);
}

__INLINE void rpl_txgain_gfsk_sel_setf(uint8_t txgaingfsksel)
{
    ASSERT_ERR((((uint32_t)txgaingfsksel << 0) & ~((uint32_t)0x0000000F)) == 0);
    REG_RPL_WR(RPL_RFTXCNTL0_ADDR, (REG_RPL_RD(RPL_RFTXCNTL0_ADDR) & ~((uint32_t)0x0000000F)) | ((uint32_t)txgaingfsksel << 0));
}

/**
 * @brief RFTXCNTL1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     31           TXDC_FORCE   0
 *  12:08               TXDC_Q   0x0
 *  04:00               TXDC_I   0x0
 * </pre>
 */
#define RPL_RFTXCNTL1_ADDR   0x00000070
#define RPL_RFTXCNTL1_OFFSET 0x00000070
#define RPL_RFTXCNTL1_INDEX  0x00000070
#define RPL_RFTXCNTL1_RESET  0x00000000

__INLINE uint32_t rpl_rftxcntl1_get(void)
{
    return REG_RPL_RD(RPL_RFTXCNTL1_ADDR);
}

__INLINE void rpl_rftxcntl1_set(uint32_t value)
{
    REG_RPL_WR(RPL_RFTXCNTL1_ADDR, value);
}

// field definitions
#define RPL_TXDC_FORCE_BIT    ((uint32_t)0x80000000)
#define RPL_TXDC_FORCE_POS    31
#define RPL_TXDC_Q_MASK       ((uint32_t)0x00001F00)
#define RPL_TXDC_Q_LSB        8
#define RPL_TXDC_Q_WIDTH      ((uint32_t)0x00000005)
#define RPL_TXDC_I_MASK       ((uint32_t)0x0000001F)
#define RPL_TXDC_I_LSB        0
#define RPL_TXDC_I_WIDTH      ((uint32_t)0x00000005)

#define RPL_TXDC_FORCE_RST    0x0
#define RPL_TXDC_Q_RST        0x0
#define RPL_TXDC_I_RST        0x0

__INLINE void rpl_rftxcntl1_pack(uint8_t txdcforce, uint8_t txdcq, uint8_t txdci)
{
    ASSERT_ERR((((uint32_t)txdcforce << 31) & ~((uint32_t)0x80000000)) == 0);
    ASSERT_ERR((((uint32_t)txdcq << 8) & ~((uint32_t)0x00001F00)) == 0);
    ASSERT_ERR((((uint32_t)txdci << 0) & ~((uint32_t)0x0000001F)) == 0);
    REG_RPL_WR(RPL_RFTXCNTL1_ADDR, ((uint32_t)txdcforce << 31) | ((uint32_t)txdcq << 8) | ((uint32_t)txdci << 0));
}

__INLINE void rpl_rftxcntl1_unpack(uint8_t *txdcforce, uint8_t *txdcq, uint8_t *txdci)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXCNTL1_ADDR);

    *txdcforce = (localVal & ((uint32_t)0x80000000)) >> 31;
    *txdcq = (localVal & ((uint32_t)0x00001F00)) >> 8;
    *txdci = (localVal & ((uint32_t)0x0000001F)) >> 0;
}

__INLINE uint8_t rpl_txdc_force_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE void rpl_txdc_force_setf(uint8_t txdcforce)
{
    ASSERT_ERR((((uint32_t)txdcforce << 31) & ~((uint32_t)0x80000000)) == 0);
    REG_RPL_WR(RPL_RFTXCNTL1_ADDR, (REG_RPL_RD(RPL_RFTXCNTL1_ADDR) & ~((uint32_t)0x80000000)) | ((uint32_t)txdcforce << 31));
}

__INLINE uint8_t rpl_txdc_q_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x00001F00)) >> 8);
}

__INLINE void rpl_txdc_q_setf(uint8_t txdcq)
{
    ASSERT_ERR((((uint32_t)txdcq << 8) & ~((uint32_t)0x00001F00)) == 0);
    REG_RPL_WR(RPL_RFTXCNTL1_ADDR, (REG_RPL_RD(RPL_RFTXCNTL1_ADDR) & ~((uint32_t)0x00001F00)) | ((uint32_t)txdcq << 8));
}

__INLINE uint8_t rpl_txdc_i_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x0000001F)) >> 0);
}

__INLINE void rpl_txdc_i_setf(uint8_t txdci)
{
    ASSERT_ERR((((uint32_t)txdci << 0) & ~((uint32_t)0x0000001F)) == 0);
    REG_RPL_WR(RPL_RFTXCNTL1_ADDR, (REG_RPL_RD(RPL_RFTXCNTL1_ADDR) & ~((uint32_t)0x0000001F)) | ((uint32_t)txdci << 0));
}

/**
 * @brief RFTXCONFIG register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     31         TXFILT_IF_EN   1
 *  27:20   TXDC_CAL_TIMEOUT_DELAY   0xFF
 *  19:16   FMTX2P_ON_DLYD_CNT   0x0
 *  15:14            FMTX_TRIM   0x0
 *  13:12           TXMIX_TRIM   0x0
 *  10:08           TXTANK_SEL   0x0
 *  07:04           TXFILT_CFG   0x0
 *  02:00             TXPA_CFG   0x0
 * </pre>
 */
#define RPL_RFTXCONFIG_ADDR   0x00000071
#define RPL_RFTXCONFIG_OFFSET 0x00000071
#define RPL_RFTXCONFIG_INDEX  0x00000071
#define RPL_RFTXCONFIG_RESET  0x8FF00000

__INLINE uint32_t rpl_rftxconfig_get(void)
{
    return REG_RPL_RD(RPL_RFTXCONFIG_ADDR);
}

__INLINE void rpl_rftxconfig_set(uint32_t value)
{
    REG_RPL_WR(RPL_RFTXCONFIG_ADDR, value);
}

// field definitions
#define RPL_TXFILT_IF_EN_BIT              ((uint32_t)0x80000000)
#define RPL_TXFILT_IF_EN_POS              31
#define RPL_TXDC_CAL_TIMEOUT_DELAY_MASK   ((uint32_t)0x0FF00000)
#define RPL_TXDC_CAL_TIMEOUT_DELAY_LSB    20
#define RPL_TXDC_CAL_TIMEOUT_DELAY_WIDTH  ((uint32_t)0x00000008)
#define RPL_FMTX2P_ON_DLYD_CNT_MASK       ((uint32_t)0x000F0000)
#define RPL_FMTX2P_ON_DLYD_CNT_LSB        16
#define RPL_FMTX2P_ON_DLYD_CNT_WIDTH      ((uint32_t)0x00000004)
#define RPL_FMTX_TRIM_MASK                ((uint32_t)0x0000C000)
#define RPL_FMTX_TRIM_LSB                 14
#define RPL_FMTX_TRIM_WIDTH               ((uint32_t)0x00000002)
#define RPL_TXMIX_TRIM_MASK               ((uint32_t)0x00003000)
#define RPL_TXMIX_TRIM_LSB                12
#define RPL_TXMIX_TRIM_WIDTH              ((uint32_t)0x00000002)
#define RPL_TXTANK_SEL_MASK               ((uint32_t)0x00000700)
#define RPL_TXTANK_SEL_LSB                8
#define RPL_TXTANK_SEL_WIDTH              ((uint32_t)0x00000003)
#define RPL_TXFILT_CFG_MASK               ((uint32_t)0x000000F0)
#define RPL_TXFILT_CFG_LSB                4
#define RPL_TXFILT_CFG_WIDTH              ((uint32_t)0x00000004)
#define RPL_TXPA_CFG_MASK                 ((uint32_t)0x00000007)
#define RPL_TXPA_CFG_LSB                  0
#define RPL_TXPA_CFG_WIDTH                ((uint32_t)0x00000003)

#define RPL_TXFILT_IF_EN_RST              0x1
#define RPL_TXDC_CAL_TIMEOUT_DELAY_RST    0xFF
#define RPL_FMTX2P_ON_DLYD_CNT_RST        0x0
#define RPL_FMTX_TRIM_RST                 0x0
#define RPL_TXMIX_TRIM_RST                0x0
#define RPL_TXTANK_SEL_RST                0x0
#define RPL_TXFILT_CFG_RST                0x0
#define RPL_TXPA_CFG_RST                  0x0

__INLINE void rpl_rftxconfig_pack(uint8_t txfiltifen, uint8_t txdccaltimeoutdelay, uint8_t fmtx2pondlydcnt, uint8_t fmtxtrim, uint8_t txmixtrim, uint8_t txtanksel, uint8_t txfiltcfg, uint8_t txpacfg)
{
    ASSERT_ERR((((uint32_t)txfiltifen << 31) & ~((uint32_t)0x80000000)) == 0);
    ASSERT_ERR((((uint32_t)txdccaltimeoutdelay << 20) & ~((uint32_t)0x0FF00000)) == 0);
    ASSERT_ERR((((uint32_t)fmtx2pondlydcnt << 16) & ~((uint32_t)0x000F0000)) == 0);
    ASSERT_ERR((((uint32_t)fmtxtrim << 14) & ~((uint32_t)0x0000C000)) == 0);
    ASSERT_ERR((((uint32_t)txmixtrim << 12) & ~((uint32_t)0x00003000)) == 0);
    ASSERT_ERR((((uint32_t)txtanksel << 8) & ~((uint32_t)0x00000700)) == 0);
    ASSERT_ERR((((uint32_t)txfiltcfg << 4) & ~((uint32_t)0x000000F0)) == 0);
    ASSERT_ERR((((uint32_t)txpacfg << 0) & ~((uint32_t)0x00000007)) == 0);
    REG_RPL_WR(RPL_RFTXCONFIG_ADDR, ((uint32_t)txfiltifen << 31) | ((uint32_t)txdccaltimeoutdelay << 20) | ((uint32_t)fmtx2pondlydcnt << 16) | ((uint32_t)fmtxtrim << 14) | ((uint32_t)txmixtrim << 12) | ((uint32_t)txtanksel << 8) | ((uint32_t)txfiltcfg << 4) | ((uint32_t)txpacfg << 0));
}

__INLINE void rpl_rftxconfig_unpack(uint8_t *txfiltifen, uint8_t *txdccaltimeoutdelay, uint8_t *fmtx2pondlydcnt, uint8_t *fmtxtrim, uint8_t *txmixtrim, uint8_t *txtanksel, uint8_t *txfiltcfg, uint8_t *txpacfg)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXCONFIG_ADDR);

    *txfiltifen = (localVal & ((uint32_t)0x80000000)) >> 31;
    *txdccaltimeoutdelay = (localVal & ((uint32_t)0x0FF00000)) >> 20;
    *fmtx2pondlydcnt = (localVal & ((uint32_t)0x000F0000)) >> 16;
    *fmtxtrim = (localVal & ((uint32_t)0x0000C000)) >> 14;
    *txmixtrim = (localVal & ((uint32_t)0x00003000)) >> 12;
    *txtanksel = (localVal & ((uint32_t)0x00000700)) >> 8;
    *txfiltcfg = (localVal & ((uint32_t)0x000000F0)) >> 4;
    *txpacfg = (localVal & ((uint32_t)0x00000007)) >> 0;
}

__INLINE uint8_t rpl_txfilt_if_en_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXCONFIG_ADDR);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE void rpl_txfilt_if_en_setf(uint8_t txfiltifen)
{
    ASSERT_ERR((((uint32_t)txfiltifen << 31) & ~((uint32_t)0x80000000)) == 0);
    REG_RPL_WR(RPL_RFTXCONFIG_ADDR, (REG_RPL_RD(RPL_RFTXCONFIG_ADDR) & ~((uint32_t)0x80000000)) | ((uint32_t)txfiltifen << 31));
}

__INLINE uint8_t rpl_txdc_cal_timeout_delay_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXCONFIG_ADDR);
    return ((localVal & ((uint32_t)0x0FF00000)) >> 20);
}

__INLINE void rpl_txdc_cal_timeout_delay_setf(uint8_t txdccaltimeoutdelay)
{
    ASSERT_ERR((((uint32_t)txdccaltimeoutdelay << 20) & ~((uint32_t)0x0FF00000)) == 0);
    REG_RPL_WR(RPL_RFTXCONFIG_ADDR, (REG_RPL_RD(RPL_RFTXCONFIG_ADDR) & ~((uint32_t)0x0FF00000)) | ((uint32_t)txdccaltimeoutdelay << 20));
}

__INLINE uint8_t rpl_fmtx2p_on_dlyd_cnt_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXCONFIG_ADDR);
    return ((localVal & ((uint32_t)0x000F0000)) >> 16);
}

__INLINE void rpl_fmtx2p_on_dlyd_cnt_setf(uint8_t fmtx2pondlydcnt)
{
    ASSERT_ERR((((uint32_t)fmtx2pondlydcnt << 16) & ~((uint32_t)0x000F0000)) == 0);
    REG_RPL_WR(RPL_RFTXCONFIG_ADDR, (REG_RPL_RD(RPL_RFTXCONFIG_ADDR) & ~((uint32_t)0x000F0000)) | ((uint32_t)fmtx2pondlydcnt << 16));
}

__INLINE uint8_t rpl_fmtx_trim_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXCONFIG_ADDR);
    return ((localVal & ((uint32_t)0x0000C000)) >> 14);
}

__INLINE void rpl_fmtx_trim_setf(uint8_t fmtxtrim)
{
    ASSERT_ERR((((uint32_t)fmtxtrim << 14) & ~((uint32_t)0x0000C000)) == 0);
    REG_RPL_WR(RPL_RFTXCONFIG_ADDR, (REG_RPL_RD(RPL_RFTXCONFIG_ADDR) & ~((uint32_t)0x0000C000)) | ((uint32_t)fmtxtrim << 14));
}

__INLINE uint8_t rpl_txmix_trim_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXCONFIG_ADDR);
    return ((localVal & ((uint32_t)0x00003000)) >> 12);
}

__INLINE void rpl_txmix_trim_setf(uint8_t txmixtrim)
{
    ASSERT_ERR((((uint32_t)txmixtrim << 12) & ~((uint32_t)0x00003000)) == 0);
    REG_RPL_WR(RPL_RFTXCONFIG_ADDR, (REG_RPL_RD(RPL_RFTXCONFIG_ADDR) & ~((uint32_t)0x00003000)) | ((uint32_t)txmixtrim << 12));
}

__INLINE uint8_t rpl_txtank_sel_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXCONFIG_ADDR);
    return ((localVal & ((uint32_t)0x00000700)) >> 8);
}

__INLINE void rpl_txtank_sel_setf(uint8_t txtanksel)
{
    ASSERT_ERR((((uint32_t)txtanksel << 8) & ~((uint32_t)0x00000700)) == 0);
    REG_RPL_WR(RPL_RFTXCONFIG_ADDR, (REG_RPL_RD(RPL_RFTXCONFIG_ADDR) & ~((uint32_t)0x00000700)) | ((uint32_t)txtanksel << 8));
}

__INLINE uint8_t rpl_txfilt_cfg_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXCONFIG_ADDR);
    return ((localVal & ((uint32_t)0x000000F0)) >> 4);
}

__INLINE void rpl_txfilt_cfg_setf(uint8_t txfiltcfg)
{
    ASSERT_ERR((((uint32_t)txfiltcfg << 4) & ~((uint32_t)0x000000F0)) == 0);
    REG_RPL_WR(RPL_RFTXCONFIG_ADDR, (REG_RPL_RD(RPL_RFTXCONFIG_ADDR) & ~((uint32_t)0x000000F0)) | ((uint32_t)txfiltcfg << 4));
}

__INLINE uint8_t rpl_txpa_cfg_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXCONFIG_ADDR);
    return ((localVal & ((uint32_t)0x00000007)) >> 0);
}

__INLINE void rpl_txpa_cfg_setf(uint8_t txpacfg)
{
    ASSERT_ERR((((uint32_t)txpacfg << 0) & ~((uint32_t)0x00000007)) == 0);
    REG_RPL_WR(RPL_RFTXCONFIG_ADDR, (REG_RPL_RD(RPL_RFTXCONFIG_ADDR) & ~((uint32_t)0x00000007)) | ((uint32_t)txpacfg << 0));
}

/**
 * @brief RFTXSTAT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  16:12            TXDCQ_VAL   0x0
 *  08:04            TXDCI_VAL   0x0
 *     01        TXDC_CAL_DEAD   0
 *     00        TXDC_CAL_DONE   0
 * </pre>
 */
#define RPL_RFTXSTAT_ADDR   0x00000072
#define RPL_RFTXSTAT_OFFSET 0x00000072
#define RPL_RFTXSTAT_INDEX  0x00000072
#define RPL_RFTXSTAT_RESET  0x00000000

__INLINE uint32_t rpl_rftxstat_get(void)
{
    return REG_RPL_RD(RPL_RFTXSTAT_ADDR);
}

// field definitions
#define RPL_TXDCQ_VAL_MASK       ((uint32_t)0x0001F000)
#define RPL_TXDCQ_VAL_LSB        12
#define RPL_TXDCQ_VAL_WIDTH      ((uint32_t)0x00000005)
#define RPL_TXDCI_VAL_MASK       ((uint32_t)0x000001F0)
#define RPL_TXDCI_VAL_LSB        4
#define RPL_TXDCI_VAL_WIDTH      ((uint32_t)0x00000005)
#define RPL_TXDC_CAL_DEAD_BIT    ((uint32_t)0x00000002)
#define RPL_TXDC_CAL_DEAD_POS    1
#define RPL_TXDC_CAL_DONE_BIT    ((uint32_t)0x00000001)
#define RPL_TXDC_CAL_DONE_POS    0

#define RPL_TXDCQ_VAL_RST        0x0
#define RPL_TXDCI_VAL_RST        0x0
#define RPL_TXDC_CAL_DEAD_RST    0x0
#define RPL_TXDC_CAL_DONE_RST    0x0

__INLINE void rpl_rftxstat_unpack(uint8_t *txdcqval, uint8_t *txdcival, uint8_t *txdccaldead, uint8_t *txdccaldone)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXSTAT_ADDR);

    *txdcqval = (localVal & ((uint32_t)0x0001F000)) >> 12;
    *txdcival = (localVal & ((uint32_t)0x000001F0)) >> 4;
    *txdccaldead = (localVal & ((uint32_t)0x00000002)) >> 1;
    *txdccaldone = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t rpl_txdcq_val_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXSTAT_ADDR);
    return ((localVal & ((uint32_t)0x0001F000)) >> 12);
}

__INLINE uint8_t rpl_txdci_val_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXSTAT_ADDR);
    return ((localVal & ((uint32_t)0x000001F0)) >> 4);
}

__INLINE uint8_t rpl_txdc_cal_dead_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXSTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE uint8_t rpl_txdc_cal_done_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXSTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

/**
 * @brief RFTXRXCNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     31       TXRXCTRL_FORCE   0
 *     30   VCOFC_ICP_CAL_FORCE   0
 *     12   VCOFC_CAL_ICP_MODE   0
 *     09      RXMDM_BYPASS_EN   0
 *     08      TXMDM_BYPASS_EN   0
 *     06   RFPLL_CAL_TIMEOUT_DSB   0
 *     05        ICP_CAL_START   0
 *     04      VCOFC_CAL_START   0
 *     02                PD_ON   0
 *     01           BB_BIAS_ON   0
 *     00            LODIV2_ON   0
 * </pre>
 */
#define RPL_RFTXRXCNTL_ADDR   0x00000073
#define RPL_RFTXRXCNTL_OFFSET 0x00000073
#define RPL_RFTXRXCNTL_INDEX  0x00000073
#define RPL_RFTXRXCNTL_RESET  0x00000000

__INLINE uint32_t rpl_rftxrxcntl_get(void)
{
    return REG_RPL_RD(RPL_RFTXRXCNTL_ADDR);
}

__INLINE void rpl_rftxrxcntl_set(uint32_t value)
{
    REG_RPL_WR(RPL_RFTXRXCNTL_ADDR, value);
}

// field definitions
#define RPL_TXRXCTRL_FORCE_BIT           ((uint32_t)0x80000000)
#define RPL_TXRXCTRL_FORCE_POS           31
#define RPL_VCOFC_ICP_CAL_FORCE_BIT      ((uint32_t)0x40000000)
#define RPL_VCOFC_ICP_CAL_FORCE_POS      30
#define RPL_VCOFC_CAL_ICP_MODE_BIT       ((uint32_t)0x00001000)
#define RPL_VCOFC_CAL_ICP_MODE_POS       12
#define RPL_RXMDM_BYPASS_EN_BIT          ((uint32_t)0x00000200)
#define RPL_RXMDM_BYPASS_EN_POS          9
#define RPL_TXMDM_BYPASS_EN_BIT          ((uint32_t)0x00000100)
#define RPL_TXMDM_BYPASS_EN_POS          8
#define RPL_RFPLL_CAL_TIMEOUT_DSB_BIT    ((uint32_t)0x00000040)
#define RPL_RFPLL_CAL_TIMEOUT_DSB_POS    6
#define RPL_ICP_CAL_START_BIT            ((uint32_t)0x00000020)
#define RPL_ICP_CAL_START_POS            5
#define RPL_VCOFC_CAL_START_BIT          ((uint32_t)0x00000010)
#define RPL_VCOFC_CAL_START_POS          4
#define RPL_PD_ON_BIT                    ((uint32_t)0x00000004)
#define RPL_PD_ON_POS                    2
#define RPL_BB_BIAS_ON_BIT               ((uint32_t)0x00000002)
#define RPL_BB_BIAS_ON_POS               1
#define RPL_LODIV2_ON_BIT                ((uint32_t)0x00000001)
#define RPL_LODIV2_ON_POS                0

#define RPL_TXRXCTRL_FORCE_RST           0x0
#define RPL_VCOFC_ICP_CAL_FORCE_RST      0x0
#define RPL_VCOFC_CAL_ICP_MODE_RST       0x0
#define RPL_RXMDM_BYPASS_EN_RST          0x0
#define RPL_TXMDM_BYPASS_EN_RST          0x0
#define RPL_RFPLL_CAL_TIMEOUT_DSB_RST    0x0
#define RPL_ICP_CAL_START_RST            0x0
#define RPL_VCOFC_CAL_START_RST          0x0
#define RPL_PD_ON_RST                    0x0
#define RPL_BB_BIAS_ON_RST               0x0
#define RPL_LODIV2_ON_RST                0x0

__INLINE void rpl_rftxrxcntl_pack(uint8_t txrxctrlforce, uint8_t vcofcicpcalforce, uint8_t vcofccalicpmode, uint8_t rxmdmbypassen, uint8_t txmdmbypassen, uint8_t rfpllcaltimeoutdsb, uint8_t icpcalstart, uint8_t vcofccalstart, uint8_t pdon, uint8_t bbbiason, uint8_t lodiv2on)
{
    ASSERT_ERR((((uint32_t)txrxctrlforce << 31) & ~((uint32_t)0x80000000)) == 0);
    ASSERT_ERR((((uint32_t)vcofcicpcalforce << 30) & ~((uint32_t)0x40000000)) == 0);
    ASSERT_ERR((((uint32_t)vcofccalicpmode << 12) & ~((uint32_t)0x00001000)) == 0);
    ASSERT_ERR((((uint32_t)rxmdmbypassen << 9) & ~((uint32_t)0x00000200)) == 0);
    ASSERT_ERR((((uint32_t)txmdmbypassen << 8) & ~((uint32_t)0x00000100)) == 0);
    ASSERT_ERR((((uint32_t)rfpllcaltimeoutdsb << 6) & ~((uint32_t)0x00000040)) == 0);
    ASSERT_ERR((((uint32_t)icpcalstart << 5) & ~((uint32_t)0x00000020)) == 0);
    ASSERT_ERR((((uint32_t)vcofccalstart << 4) & ~((uint32_t)0x00000010)) == 0);
    ASSERT_ERR((((uint32_t)pdon << 2) & ~((uint32_t)0x00000004)) == 0);
    ASSERT_ERR((((uint32_t)bbbiason << 1) & ~((uint32_t)0x00000002)) == 0);
    ASSERT_ERR((((uint32_t)lodiv2on << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_RPL_WR(RPL_RFTXRXCNTL_ADDR, ((uint32_t)txrxctrlforce << 31) | ((uint32_t)vcofcicpcalforce << 30) | ((uint32_t)vcofccalicpmode << 12) | ((uint32_t)rxmdmbypassen << 9) | ((uint32_t)txmdmbypassen << 8) | ((uint32_t)rfpllcaltimeoutdsb << 6) | ((uint32_t)icpcalstart << 5) | ((uint32_t)vcofccalstart << 4) | ((uint32_t)pdon << 2) | ((uint32_t)bbbiason << 1) | ((uint32_t)lodiv2on << 0));
}

__INLINE void rpl_rftxrxcntl_unpack(uint8_t *txrxctrlforce, uint8_t *vcofcicpcalforce, uint8_t *vcofccalicpmode, uint8_t *rxmdmbypassen, uint8_t *txmdmbypassen, uint8_t *rfpllcaltimeoutdsb, uint8_t *icpcalstart, uint8_t *vcofccalstart, uint8_t *pdon, uint8_t *bbbiason, uint8_t *lodiv2on)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXRXCNTL_ADDR);

    *txrxctrlforce = (localVal & ((uint32_t)0x80000000)) >> 31;
    *vcofcicpcalforce = (localVal & ((uint32_t)0x40000000)) >> 30;
    *vcofccalicpmode = (localVal & ((uint32_t)0x00001000)) >> 12;
    *rxmdmbypassen = (localVal & ((uint32_t)0x00000200)) >> 9;
    *txmdmbypassen = (localVal & ((uint32_t)0x00000100)) >> 8;
    *rfpllcaltimeoutdsb = (localVal & ((uint32_t)0x00000040)) >> 6;
    *icpcalstart = (localVal & ((uint32_t)0x00000020)) >> 5;
    *vcofccalstart = (localVal & ((uint32_t)0x00000010)) >> 4;
    *pdon = (localVal & ((uint32_t)0x00000004)) >> 2;
    *bbbiason = (localVal & ((uint32_t)0x00000002)) >> 1;
    *lodiv2on = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t rpl_txrxctrl_force_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXRXCNTL_ADDR);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE void rpl_txrxctrl_force_setf(uint8_t txrxctrlforce)
{
    ASSERT_ERR((((uint32_t)txrxctrlforce << 31) & ~((uint32_t)0x80000000)) == 0);
    REG_RPL_WR(RPL_RFTXRXCNTL_ADDR, (REG_RPL_RD(RPL_RFTXRXCNTL_ADDR) & ~((uint32_t)0x80000000)) | ((uint32_t)txrxctrlforce << 31));
}

__INLINE uint8_t rpl_vcofc_icp_cal_force_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXRXCNTL_ADDR);
    return ((localVal & ((uint32_t)0x40000000)) >> 30);
}

__INLINE void rpl_vcofc_icp_cal_force_setf(uint8_t vcofcicpcalforce)
{
    ASSERT_ERR((((uint32_t)vcofcicpcalforce << 30) & ~((uint32_t)0x40000000)) == 0);
    REG_RPL_WR(RPL_RFTXRXCNTL_ADDR, (REG_RPL_RD(RPL_RFTXRXCNTL_ADDR) & ~((uint32_t)0x40000000)) | ((uint32_t)vcofcicpcalforce << 30));
}

__INLINE uint8_t rpl_vcofc_cal_icp_mode_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXRXCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00001000)) >> 12);
}

__INLINE void rpl_vcofc_cal_icp_mode_setf(uint8_t vcofccalicpmode)
{
    ASSERT_ERR((((uint32_t)vcofccalicpmode << 12) & ~((uint32_t)0x00001000)) == 0);
    REG_RPL_WR(RPL_RFTXRXCNTL_ADDR, (REG_RPL_RD(RPL_RFTXRXCNTL_ADDR) & ~((uint32_t)0x00001000)) | ((uint32_t)vcofccalicpmode << 12));
}

__INLINE uint8_t rpl_rxmdm_bypass_en_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXRXCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000200)) >> 9);
}

__INLINE void rpl_rxmdm_bypass_en_setf(uint8_t rxmdmbypassen)
{
    ASSERT_ERR((((uint32_t)rxmdmbypassen << 9) & ~((uint32_t)0x00000200)) == 0);
    REG_RPL_WR(RPL_RFTXRXCNTL_ADDR, (REG_RPL_RD(RPL_RFTXRXCNTL_ADDR) & ~((uint32_t)0x00000200)) | ((uint32_t)rxmdmbypassen << 9));
}

__INLINE uint8_t rpl_txmdm_bypass_en_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXRXCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000100)) >> 8);
}

__INLINE void rpl_txmdm_bypass_en_setf(uint8_t txmdmbypassen)
{
    ASSERT_ERR((((uint32_t)txmdmbypassen << 8) & ~((uint32_t)0x00000100)) == 0);
    REG_RPL_WR(RPL_RFTXRXCNTL_ADDR, (REG_RPL_RD(RPL_RFTXRXCNTL_ADDR) & ~((uint32_t)0x00000100)) | ((uint32_t)txmdmbypassen << 8));
}

__INLINE uint8_t rpl_rfpll_cal_timeout_dsb_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXRXCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000040)) >> 6);
}

__INLINE void rpl_rfpll_cal_timeout_dsb_setf(uint8_t rfpllcaltimeoutdsb)
{
    ASSERT_ERR((((uint32_t)rfpllcaltimeoutdsb << 6) & ~((uint32_t)0x00000040)) == 0);
    REG_RPL_WR(RPL_RFTXRXCNTL_ADDR, (REG_RPL_RD(RPL_RFTXRXCNTL_ADDR) & ~((uint32_t)0x00000040)) | ((uint32_t)rfpllcaltimeoutdsb << 6));
}

__INLINE uint8_t rpl_icp_cal_start_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXRXCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000020)) >> 5);
}

__INLINE void rpl_icp_cal_start_setf(uint8_t icpcalstart)
{
    ASSERT_ERR((((uint32_t)icpcalstart << 5) & ~((uint32_t)0x00000020)) == 0);
    REG_RPL_WR(RPL_RFTXRXCNTL_ADDR, (REG_RPL_RD(RPL_RFTXRXCNTL_ADDR) & ~((uint32_t)0x00000020)) | ((uint32_t)icpcalstart << 5));
}

__INLINE uint8_t rpl_vcofc_cal_start_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXRXCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE void rpl_vcofc_cal_start_setf(uint8_t vcofccalstart)
{
    ASSERT_ERR((((uint32_t)vcofccalstart << 4) & ~((uint32_t)0x00000010)) == 0);
    REG_RPL_WR(RPL_RFTXRXCNTL_ADDR, (REG_RPL_RD(RPL_RFTXRXCNTL_ADDR) & ~((uint32_t)0x00000010)) | ((uint32_t)vcofccalstart << 4));
}

__INLINE uint8_t rpl_pd_on_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXRXCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE void rpl_pd_on_setf(uint8_t pdon)
{
    ASSERT_ERR((((uint32_t)pdon << 2) & ~((uint32_t)0x00000004)) == 0);
    REG_RPL_WR(RPL_RFTXRXCNTL_ADDR, (REG_RPL_RD(RPL_RFTXRXCNTL_ADDR) & ~((uint32_t)0x00000004)) | ((uint32_t)pdon << 2));
}

__INLINE uint8_t rpl_bb_bias_on_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXRXCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE void rpl_bb_bias_on_setf(uint8_t bbbiason)
{
    ASSERT_ERR((((uint32_t)bbbiason << 1) & ~((uint32_t)0x00000002)) == 0);
    REG_RPL_WR(RPL_RFTXRXCNTL_ADDR, (REG_RPL_RD(RPL_RFTXRXCNTL_ADDR) & ~((uint32_t)0x00000002)) | ((uint32_t)bbbiason << 1));
}

__INLINE uint8_t rpl_lodiv2_on_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXRXCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void rpl_lodiv2_on_setf(uint8_t lodiv2on)
{
    ASSERT_ERR((((uint32_t)lodiv2on << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_RPL_WR(RPL_RFTXRXCNTL_ADDR, (REG_RPL_RD(RPL_RFTXRXCNTL_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)lodiv2on << 0));
}

/**
 * @brief RFTXRXCONFIG register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     31         DATA_BUS_POL   0
 *     30          ICP_CAL_DSB   0
 *  29:28           CALIB_MODE   0x2
 *  27:20   CALIB_TIMEOUT_DELAY   0xFF
 *  19:14   ICP_CAL_SETTLE_TIME   0x5
 *  13:08   VCOFC_CAL_SETTLE_TIME   0x5
 *  07:06          BB_BIAS_CFG   0x0
 *     04         DAP_CLK_MODE   0
 *  03:02          LODIV2_TRIM   0x0
 *  01:00           LOBUF_TRIM   0x0
 * </pre>
 */
#define RPL_RFTXRXCONFIG_ADDR   0x00000074
#define RPL_RFTXRXCONFIG_OFFSET 0x00000074
#define RPL_RFTXRXCONFIG_INDEX  0x00000074
#define RPL_RFTXRXCONFIG_RESET  0x2FF14500

__INLINE uint32_t rpl_rftxrxconfig_get(void)
{
    return REG_RPL_RD(RPL_RFTXRXCONFIG_ADDR);
}

__INLINE void rpl_rftxrxconfig_set(uint32_t value)
{
    REG_RPL_WR(RPL_RFTXRXCONFIG_ADDR, value);
}

// field definitions
#define RPL_DATA_BUS_POL_BIT             ((uint32_t)0x80000000)
#define RPL_DATA_BUS_POL_POS             31
#define RPL_ICP_CAL_DSB_BIT              ((uint32_t)0x40000000)
#define RPL_ICP_CAL_DSB_POS              30
#define RPL_CALIB_MODE_MASK              ((uint32_t)0x30000000)
#define RPL_CALIB_MODE_LSB               28
#define RPL_CALIB_MODE_WIDTH             ((uint32_t)0x00000002)
#define RPL_CALIB_TIMEOUT_DELAY_MASK     ((uint32_t)0x0FF00000)
#define RPL_CALIB_TIMEOUT_DELAY_LSB      20
#define RPL_CALIB_TIMEOUT_DELAY_WIDTH    ((uint32_t)0x00000008)
#define RPL_ICP_CAL_SETTLE_TIME_MASK     ((uint32_t)0x000FC000)
#define RPL_ICP_CAL_SETTLE_TIME_LSB      14
#define RPL_ICP_CAL_SETTLE_TIME_WIDTH    ((uint32_t)0x00000006)
#define RPL_VCOFC_CAL_SETTLE_TIME_MASK   ((uint32_t)0x00003F00)
#define RPL_VCOFC_CAL_SETTLE_TIME_LSB    8
#define RPL_VCOFC_CAL_SETTLE_TIME_WIDTH  ((uint32_t)0x00000006)
#define RPL_BB_BIAS_CFG_MASK             ((uint32_t)0x000000C0)
#define RPL_BB_BIAS_CFG_LSB              6
#define RPL_BB_BIAS_CFG_WIDTH            ((uint32_t)0x00000002)
#define RPL_DAP_CLK_MODE_BIT             ((uint32_t)0x00000010)
#define RPL_DAP_CLK_MODE_POS             4
#define RPL_LODIV2_TRIM_MASK             ((uint32_t)0x0000000C)
#define RPL_LODIV2_TRIM_LSB              2
#define RPL_LODIV2_TRIM_WIDTH            ((uint32_t)0x00000002)
#define RPL_LOBUF_TRIM_MASK              ((uint32_t)0x00000003)
#define RPL_LOBUF_TRIM_LSB               0
#define RPL_LOBUF_TRIM_WIDTH             ((uint32_t)0x00000002)

#define RPL_DATA_BUS_POL_RST             0x0
#define RPL_ICP_CAL_DSB_RST              0x0
#define RPL_CALIB_MODE_RST               0x2
#define RPL_CALIB_TIMEOUT_DELAY_RST      0xFF
#define RPL_ICP_CAL_SETTLE_TIME_RST      0x5
#define RPL_VCOFC_CAL_SETTLE_TIME_RST    0x5
#define RPL_BB_BIAS_CFG_RST              0x0
#define RPL_DAP_CLK_MODE_RST             0x0
#define RPL_LODIV2_TRIM_RST              0x0
#define RPL_LOBUF_TRIM_RST               0x0

__INLINE void rpl_rftxrxconfig_pack(uint8_t databuspol, uint8_t icpcaldsb, uint8_t calibmode, uint8_t calibtimeoutdelay, uint8_t icpcalsettletime, uint8_t vcofccalsettletime, uint8_t bbbiascfg, uint8_t dapclkmode, uint8_t lodiv2trim, uint8_t lobuftrim)
{
    ASSERT_ERR((((uint32_t)databuspol << 31) & ~((uint32_t)0x80000000)) == 0);
    ASSERT_ERR((((uint32_t)icpcaldsb << 30) & ~((uint32_t)0x40000000)) == 0);
    ASSERT_ERR((((uint32_t)calibmode << 28) & ~((uint32_t)0x30000000)) == 0);
    ASSERT_ERR((((uint32_t)calibtimeoutdelay << 20) & ~((uint32_t)0x0FF00000)) == 0);
    ASSERT_ERR((((uint32_t)icpcalsettletime << 14) & ~((uint32_t)0x000FC000)) == 0);
    ASSERT_ERR((((uint32_t)vcofccalsettletime << 8) & ~((uint32_t)0x00003F00)) == 0);
    ASSERT_ERR((((uint32_t)bbbiascfg << 6) & ~((uint32_t)0x000000C0)) == 0);
    ASSERT_ERR((((uint32_t)dapclkmode << 4) & ~((uint32_t)0x00000010)) == 0);
    ASSERT_ERR((((uint32_t)lodiv2trim << 2) & ~((uint32_t)0x0000000C)) == 0);
    ASSERT_ERR((((uint32_t)lobuftrim << 0) & ~((uint32_t)0x00000003)) == 0);
    REG_RPL_WR(RPL_RFTXRXCONFIG_ADDR, ((uint32_t)databuspol << 31) | ((uint32_t)icpcaldsb << 30) | ((uint32_t)calibmode << 28) | ((uint32_t)calibtimeoutdelay << 20) | ((uint32_t)icpcalsettletime << 14) | ((uint32_t)vcofccalsettletime << 8) | ((uint32_t)bbbiascfg << 6) | ((uint32_t)dapclkmode << 4) | ((uint32_t)lodiv2trim << 2) | ((uint32_t)lobuftrim << 0));
}

__INLINE void rpl_rftxrxconfig_unpack(uint8_t *databuspol, uint8_t *icpcaldsb, uint8_t *calibmode, uint8_t *calibtimeoutdelay, uint8_t *icpcalsettletime, uint8_t *vcofccalsettletime, uint8_t *bbbiascfg, uint8_t *dapclkmode, uint8_t *lodiv2trim, uint8_t *lobuftrim)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXRXCONFIG_ADDR);

    *databuspol = (localVal & ((uint32_t)0x80000000)) >> 31;
    *icpcaldsb = (localVal & ((uint32_t)0x40000000)) >> 30;
    *calibmode = (localVal & ((uint32_t)0x30000000)) >> 28;
    *calibtimeoutdelay = (localVal & ((uint32_t)0x0FF00000)) >> 20;
    *icpcalsettletime = (localVal & ((uint32_t)0x000FC000)) >> 14;
    *vcofccalsettletime = (localVal & ((uint32_t)0x00003F00)) >> 8;
    *bbbiascfg = (localVal & ((uint32_t)0x000000C0)) >> 6;
    *dapclkmode = (localVal & ((uint32_t)0x00000010)) >> 4;
    *lodiv2trim = (localVal & ((uint32_t)0x0000000C)) >> 2;
    *lobuftrim = (localVal & ((uint32_t)0x00000003)) >> 0;
}

__INLINE uint8_t rpl_data_bus_pol_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXRXCONFIG_ADDR);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE void rpl_data_bus_pol_setf(uint8_t databuspol)
{
    ASSERT_ERR((((uint32_t)databuspol << 31) & ~((uint32_t)0x80000000)) == 0);
    REG_RPL_WR(RPL_RFTXRXCONFIG_ADDR, (REG_RPL_RD(RPL_RFTXRXCONFIG_ADDR) & ~((uint32_t)0x80000000)) | ((uint32_t)databuspol << 31));
}

__INLINE uint8_t rpl_icp_cal_dsb_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXRXCONFIG_ADDR);
    return ((localVal & ((uint32_t)0x40000000)) >> 30);
}

__INLINE void rpl_icp_cal_dsb_setf(uint8_t icpcaldsb)
{
    ASSERT_ERR((((uint32_t)icpcaldsb << 30) & ~((uint32_t)0x40000000)) == 0);
    REG_RPL_WR(RPL_RFTXRXCONFIG_ADDR, (REG_RPL_RD(RPL_RFTXRXCONFIG_ADDR) & ~((uint32_t)0x40000000)) | ((uint32_t)icpcaldsb << 30));
}

__INLINE uint8_t rpl_calib_mode_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXRXCONFIG_ADDR);
    return ((localVal & ((uint32_t)0x30000000)) >> 28);
}

__INLINE void rpl_calib_mode_setf(uint8_t calibmode)
{
    ASSERT_ERR((((uint32_t)calibmode << 28) & ~((uint32_t)0x30000000)) == 0);
    REG_RPL_WR(RPL_RFTXRXCONFIG_ADDR, (REG_RPL_RD(RPL_RFTXRXCONFIG_ADDR) & ~((uint32_t)0x30000000)) | ((uint32_t)calibmode << 28));
}

__INLINE uint8_t rpl_calib_timeout_delay_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXRXCONFIG_ADDR);
    return ((localVal & ((uint32_t)0x0FF00000)) >> 20);
}

__INLINE void rpl_calib_timeout_delay_setf(uint8_t calibtimeoutdelay)
{
    ASSERT_ERR((((uint32_t)calibtimeoutdelay << 20) & ~((uint32_t)0x0FF00000)) == 0);
    REG_RPL_WR(RPL_RFTXRXCONFIG_ADDR, (REG_RPL_RD(RPL_RFTXRXCONFIG_ADDR) & ~((uint32_t)0x0FF00000)) | ((uint32_t)calibtimeoutdelay << 20));
}

__INLINE uint8_t rpl_icp_cal_settle_time_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXRXCONFIG_ADDR);
    return ((localVal & ((uint32_t)0x000FC000)) >> 14);
}

__INLINE void rpl_icp_cal_settle_time_setf(uint8_t icpcalsettletime)
{
    ASSERT_ERR((((uint32_t)icpcalsettletime << 14) & ~((uint32_t)0x000FC000)) == 0);
    REG_RPL_WR(RPL_RFTXRXCONFIG_ADDR, (REG_RPL_RD(RPL_RFTXRXCONFIG_ADDR) & ~((uint32_t)0x000FC000)) | ((uint32_t)icpcalsettletime << 14));
}

__INLINE uint8_t rpl_vcofc_cal_settle_time_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXRXCONFIG_ADDR);
    return ((localVal & ((uint32_t)0x00003F00)) >> 8);
}

__INLINE void rpl_vcofc_cal_settle_time_setf(uint8_t vcofccalsettletime)
{
    ASSERT_ERR((((uint32_t)vcofccalsettletime << 8) & ~((uint32_t)0x00003F00)) == 0);
    REG_RPL_WR(RPL_RFTXRXCONFIG_ADDR, (REG_RPL_RD(RPL_RFTXRXCONFIG_ADDR) & ~((uint32_t)0x00003F00)) | ((uint32_t)vcofccalsettletime << 8));
}

__INLINE uint8_t rpl_bb_bias_cfg_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXRXCONFIG_ADDR);
    return ((localVal & ((uint32_t)0x000000C0)) >> 6);
}

__INLINE void rpl_bb_bias_cfg_setf(uint8_t bbbiascfg)
{
    ASSERT_ERR((((uint32_t)bbbiascfg << 6) & ~((uint32_t)0x000000C0)) == 0);
    REG_RPL_WR(RPL_RFTXRXCONFIG_ADDR, (REG_RPL_RD(RPL_RFTXRXCONFIG_ADDR) & ~((uint32_t)0x000000C0)) | ((uint32_t)bbbiascfg << 6));
}

__INLINE uint8_t rpl_dap_clk_mode_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXRXCONFIG_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE void rpl_dap_clk_mode_setf(uint8_t dapclkmode)
{
    ASSERT_ERR((((uint32_t)dapclkmode << 4) & ~((uint32_t)0x00000010)) == 0);
    REG_RPL_WR(RPL_RFTXRXCONFIG_ADDR, (REG_RPL_RD(RPL_RFTXRXCONFIG_ADDR) & ~((uint32_t)0x00000010)) | ((uint32_t)dapclkmode << 4));
}

__INLINE uint8_t rpl_lodiv2_trim_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXRXCONFIG_ADDR);
    return ((localVal & ((uint32_t)0x0000000C)) >> 2);
}

__INLINE void rpl_lodiv2_trim_setf(uint8_t lodiv2trim)
{
    ASSERT_ERR((((uint32_t)lodiv2trim << 2) & ~((uint32_t)0x0000000C)) == 0);
    REG_RPL_WR(RPL_RFTXRXCONFIG_ADDR, (REG_RPL_RD(RPL_RFTXRXCONFIG_ADDR) & ~((uint32_t)0x0000000C)) | ((uint32_t)lodiv2trim << 2));
}

__INLINE uint8_t rpl_lobuf_trim_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXRXCONFIG_ADDR);
    return ((localVal & ((uint32_t)0x00000003)) >> 0);
}

__INLINE void rpl_lobuf_trim_setf(uint8_t lobuftrim)
{
    ASSERT_ERR((((uint32_t)lobuftrim << 0) & ~((uint32_t)0x00000003)) == 0);
    REG_RPL_WR(RPL_RFTXRXCONFIG_ADDR, (REG_RPL_RD(RPL_RFTXRXCONFIG_ADDR) & ~((uint32_t)0x00000003)) | ((uint32_t)lobuftrim << 0));
}

/**
 * @brief RFTXRXSTAT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     03          CAL_TIMEOUT   0
 *     01         ICP_CAL_DONE   0
 *     00       VCOFC_CAL_DONE   0
 * </pre>
 */
#define RPL_RFTXRXSTAT_ADDR   0x00000075
#define RPL_RFTXRXSTAT_OFFSET 0x00000075
#define RPL_RFTXRXSTAT_INDEX  0x00000075
#define RPL_RFTXRXSTAT_RESET  0x00000000

__INLINE uint32_t rpl_rftxrxstat_get(void)
{
    return REG_RPL_RD(RPL_RFTXRXSTAT_ADDR);
}

// field definitions
#define RPL_CAL_TIMEOUT_BIT       ((uint32_t)0x00000008)
#define RPL_CAL_TIMEOUT_POS       3
#define RPL_ICP_CAL_DONE_BIT      ((uint32_t)0x00000002)
#define RPL_ICP_CAL_DONE_POS      1
#define RPL_VCOFC_CAL_DONE_BIT    ((uint32_t)0x00000001)
#define RPL_VCOFC_CAL_DONE_POS    0

#define RPL_CAL_TIMEOUT_RST       0x0
#define RPL_ICP_CAL_DONE_RST      0x0
#define RPL_VCOFC_CAL_DONE_RST    0x0

__INLINE void rpl_rftxrxstat_unpack(uint8_t *caltimeout, uint8_t *icpcaldone, uint8_t *vcofccaldone)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXRXSTAT_ADDR);

    *caltimeout = (localVal & ((uint32_t)0x00000008)) >> 3;
    *icpcaldone = (localVal & ((uint32_t)0x00000002)) >> 1;
    *vcofccaldone = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t rpl_cal_timeout_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXRXSTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

__INLINE uint8_t rpl_icp_cal_done_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXRXSTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE uint8_t rpl_vcofc_cal_done_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RFTXRXSTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

/**
 * @brief SAR_ADC_CNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     31        SAR_ADC_FORCE   0
 *  07:04          SAR_ADC_SEL   0x0
 *     02         SAR_ADC_MODE   0
 *     01        SAR_ADC_START   0
 *     00           SAR_ADC_ON   0
 * </pre>
 */
#define RPL_SAR_ADC_CNTL_ADDR   0x00000076
#define RPL_SAR_ADC_CNTL_OFFSET 0x00000076
#define RPL_SAR_ADC_CNTL_INDEX  0x00000076
#define RPL_SAR_ADC_CNTL_RESET  0x00000000

__INLINE uint32_t rpl_sar_adc_cntl_get(void)
{
    return REG_RPL_RD(RPL_SAR_ADC_CNTL_ADDR);
}

__INLINE void rpl_sar_adc_cntl_set(uint32_t value)
{
    REG_RPL_WR(RPL_SAR_ADC_CNTL_ADDR, value);
}

// field definitions
#define RPL_SAR_ADC_FORCE_BIT    ((uint32_t)0x80000000)
#define RPL_SAR_ADC_FORCE_POS    31
#define RPL_SAR_ADC_SEL_MASK     ((uint32_t)0x000000F0)
#define RPL_SAR_ADC_SEL_LSB      4
#define RPL_SAR_ADC_SEL_WIDTH    ((uint32_t)0x00000004)
#define RPL_SAR_ADC_MODE_BIT     ((uint32_t)0x00000004)
#define RPL_SAR_ADC_MODE_POS     2
#define RPL_SAR_ADC_START_BIT    ((uint32_t)0x00000002)
#define RPL_SAR_ADC_START_POS    1
#define RPL_SAR_ADC_ON_BIT       ((uint32_t)0x00000001)
#define RPL_SAR_ADC_ON_POS       0

#define RPL_SAR_ADC_FORCE_RST    0x0
#define RPL_SAR_ADC_SEL_RST      0x0
#define RPL_SAR_ADC_MODE_RST     0x0
#define RPL_SAR_ADC_START_RST    0x0
#define RPL_SAR_ADC_ON_RST       0x0

__INLINE void rpl_sar_adc_cntl_pack(uint8_t saradcforce, uint8_t saradcsel, uint8_t saradcmode, uint8_t saradcstart, uint8_t saradcon)
{
    ASSERT_ERR((((uint32_t)saradcforce << 31) & ~((uint32_t)0x80000000)) == 0);
    ASSERT_ERR((((uint32_t)saradcsel << 4) & ~((uint32_t)0x000000F0)) == 0);
    ASSERT_ERR((((uint32_t)saradcmode << 2) & ~((uint32_t)0x00000004)) == 0);
    ASSERT_ERR((((uint32_t)saradcstart << 1) & ~((uint32_t)0x00000002)) == 0);
    ASSERT_ERR((((uint32_t)saradcon << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_RPL_WR(RPL_SAR_ADC_CNTL_ADDR, ((uint32_t)saradcforce << 31) | ((uint32_t)saradcsel << 4) | ((uint32_t)saradcmode << 2) | ((uint32_t)saradcstart << 1) | ((uint32_t)saradcon << 0));
}

__INLINE void rpl_sar_adc_cntl_unpack(uint8_t *saradcforce, uint8_t *saradcsel, uint8_t *saradcmode, uint8_t *saradcstart, uint8_t *saradcon)
{
    uint32_t localVal = REG_RPL_RD(RPL_SAR_ADC_CNTL_ADDR);

    *saradcforce = (localVal & ((uint32_t)0x80000000)) >> 31;
    *saradcsel = (localVal & ((uint32_t)0x000000F0)) >> 4;
    *saradcmode = (localVal & ((uint32_t)0x00000004)) >> 2;
    *saradcstart = (localVal & ((uint32_t)0x00000002)) >> 1;
    *saradcon = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t rpl_sar_adc_force_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_SAR_ADC_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE void rpl_sar_adc_force_setf(uint8_t saradcforce)
{
    ASSERT_ERR((((uint32_t)saradcforce << 31) & ~((uint32_t)0x80000000)) == 0);
    REG_RPL_WR(RPL_SAR_ADC_CNTL_ADDR, (REG_RPL_RD(RPL_SAR_ADC_CNTL_ADDR) & ~((uint32_t)0x80000000)) | ((uint32_t)saradcforce << 31));
}

__INLINE uint8_t rpl_sar_adc_sel_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_SAR_ADC_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x000000F0)) >> 4);
}

__INLINE void rpl_sar_adc_sel_setf(uint8_t saradcsel)
{
    ASSERT_ERR((((uint32_t)saradcsel << 4) & ~((uint32_t)0x000000F0)) == 0);
    REG_RPL_WR(RPL_SAR_ADC_CNTL_ADDR, (REG_RPL_RD(RPL_SAR_ADC_CNTL_ADDR) & ~((uint32_t)0x000000F0)) | ((uint32_t)saradcsel << 4));
}

__INLINE uint8_t rpl_sar_adc_mode_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_SAR_ADC_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE void rpl_sar_adc_mode_setf(uint8_t saradcmode)
{
    ASSERT_ERR((((uint32_t)saradcmode << 2) & ~((uint32_t)0x00000004)) == 0);
    REG_RPL_WR(RPL_SAR_ADC_CNTL_ADDR, (REG_RPL_RD(RPL_SAR_ADC_CNTL_ADDR) & ~((uint32_t)0x00000004)) | ((uint32_t)saradcmode << 2));
}

__INLINE uint8_t rpl_sar_adc_start_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_SAR_ADC_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE void rpl_sar_adc_start_setf(uint8_t saradcstart)
{
    ASSERT_ERR((((uint32_t)saradcstart << 1) & ~((uint32_t)0x00000002)) == 0);
    REG_RPL_WR(RPL_SAR_ADC_CNTL_ADDR, (REG_RPL_RD(RPL_SAR_ADC_CNTL_ADDR) & ~((uint32_t)0x00000002)) | ((uint32_t)saradcstart << 1));
}

__INLINE uint8_t rpl_sar_adc_on_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_SAR_ADC_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void rpl_sar_adc_on_setf(uint8_t saradcon)
{
    ASSERT_ERR((((uint32_t)saradcon << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_RPL_WR(RPL_SAR_ADC_CNTL_ADDR, (REG_RPL_RD(RPL_SAR_ADC_CNTL_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)saradcon << 0));
}

/**
 * @brief SAR_ADC_STAT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     31         SAR_ADC_DONE   0
 *  09:00          SAR_ADC_VAL   0x0
 * </pre>
 */
#define RPL_SAR_ADC_STAT_ADDR   0x00000077
#define RPL_SAR_ADC_STAT_OFFSET 0x00000077
#define RPL_SAR_ADC_STAT_INDEX  0x00000077
#define RPL_SAR_ADC_STAT_RESET  0x00000000

__INLINE uint32_t rpl_sar_adc_stat_get(void)
{
    return REG_RPL_RD(RPL_SAR_ADC_STAT_ADDR);
}

// field definitions
#define RPL_SAR_ADC_DONE_BIT    ((uint32_t)0x80000000)
#define RPL_SAR_ADC_DONE_POS    31
#define RPL_SAR_ADC_VAL_MASK    ((uint32_t)0x000003FF)
#define RPL_SAR_ADC_VAL_LSB     0
#define RPL_SAR_ADC_VAL_WIDTH   ((uint32_t)0x0000000A)

#define RPL_SAR_ADC_DONE_RST    0x0
#define RPL_SAR_ADC_VAL_RST     0x0

__INLINE void rpl_sar_adc_stat_unpack(uint8_t *saradcdone, uint16_t *saradcval)
{
    uint32_t localVal = REG_RPL_RD(RPL_SAR_ADC_STAT_ADDR);

    *saradcdone = (localVal & ((uint32_t)0x80000000)) >> 31;
    *saradcval = (localVal & ((uint32_t)0x000003FF)) >> 0;
}

__INLINE uint8_t rpl_sar_adc_done_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_SAR_ADC_STAT_ADDR);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE uint16_t rpl_sar_adc_val_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_SAR_ADC_STAT_ADDR);
    return ((localVal & ((uint32_t)0x000003FF)) >> 0);
}

/**
 * @brief RXADC_SATDET_CONFIG register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:30    RXADC_SATDET_MODE   0x2
 *  14:08     RXADC_SATDET_THR   0x14
 *  06:00   RXADC_SATDET_WINDOW   0x40
 * </pre>
 */
#define RPL_RXADC_SATDET_CONFIG_ADDR   0x00000078
#define RPL_RXADC_SATDET_CONFIG_OFFSET 0x00000078
#define RPL_RXADC_SATDET_CONFIG_INDEX  0x00000078
#define RPL_RXADC_SATDET_CONFIG_RESET  0x80001440

__INLINE uint32_t rpl_rxadc_satdet_config_get(void)
{
    return REG_RPL_RD(RPL_RXADC_SATDET_CONFIG_ADDR);
}

__INLINE void rpl_rxadc_satdet_config_set(uint32_t value)
{
    REG_RPL_WR(RPL_RXADC_SATDET_CONFIG_ADDR, value);
}

// field definitions
#define RPL_RXADC_SATDET_MODE_MASK     ((uint32_t)0xC0000000)
#define RPL_RXADC_SATDET_MODE_LSB      30
#define RPL_RXADC_SATDET_MODE_WIDTH    ((uint32_t)0x00000002)
#define RPL_RXADC_SATDET_THR_MASK      ((uint32_t)0x00007F00)
#define RPL_RXADC_SATDET_THR_LSB       8
#define RPL_RXADC_SATDET_THR_WIDTH     ((uint32_t)0x00000007)
#define RPL_RXADC_SATDET_WINDOW_MASK   ((uint32_t)0x0000007F)
#define RPL_RXADC_SATDET_WINDOW_LSB    0
#define RPL_RXADC_SATDET_WINDOW_WIDTH  ((uint32_t)0x00000007)

#define RPL_RXADC_SATDET_MODE_RST      0x2
#define RPL_RXADC_SATDET_THR_RST       0x14
#define RPL_RXADC_SATDET_WINDOW_RST    0x40

__INLINE void rpl_rxadc_satdet_config_pack(uint8_t rxadcsatdetmode, uint8_t rxadcsatdetthr, uint8_t rxadcsatdetwindow)
{
    ASSERT_ERR((((uint32_t)rxadcsatdetmode << 30) & ~((uint32_t)0xC0000000)) == 0);
    ASSERT_ERR((((uint32_t)rxadcsatdetthr << 8) & ~((uint32_t)0x00007F00)) == 0);
    ASSERT_ERR((((uint32_t)rxadcsatdetwindow << 0) & ~((uint32_t)0x0000007F)) == 0);
    REG_RPL_WR(RPL_RXADC_SATDET_CONFIG_ADDR, ((uint32_t)rxadcsatdetmode << 30) | ((uint32_t)rxadcsatdetthr << 8) | ((uint32_t)rxadcsatdetwindow << 0));
}

__INLINE void rpl_rxadc_satdet_config_unpack(uint8_t *rxadcsatdetmode, uint8_t *rxadcsatdetthr, uint8_t *rxadcsatdetwindow)
{
    uint32_t localVal = REG_RPL_RD(RPL_RXADC_SATDET_CONFIG_ADDR);

    *rxadcsatdetmode = (localVal & ((uint32_t)0xC0000000)) >> 30;
    *rxadcsatdetthr = (localVal & ((uint32_t)0x00007F00)) >> 8;
    *rxadcsatdetwindow = (localVal & ((uint32_t)0x0000007F)) >> 0;
}

__INLINE uint8_t rpl_rxadc_satdet_mode_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RXADC_SATDET_CONFIG_ADDR);
    return ((localVal & ((uint32_t)0xC0000000)) >> 30);
}

__INLINE void rpl_rxadc_satdet_mode_setf(uint8_t rxadcsatdetmode)
{
    ASSERT_ERR((((uint32_t)rxadcsatdetmode << 30) & ~((uint32_t)0xC0000000)) == 0);
    REG_RPL_WR(RPL_RXADC_SATDET_CONFIG_ADDR, (REG_RPL_RD(RPL_RXADC_SATDET_CONFIG_ADDR) & ~((uint32_t)0xC0000000)) | ((uint32_t)rxadcsatdetmode << 30));
}

__INLINE uint8_t rpl_rxadc_satdet_thr_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RXADC_SATDET_CONFIG_ADDR);
    return ((localVal & ((uint32_t)0x00007F00)) >> 8);
}

__INLINE void rpl_rxadc_satdet_thr_setf(uint8_t rxadcsatdetthr)
{
    ASSERT_ERR((((uint32_t)rxadcsatdetthr << 8) & ~((uint32_t)0x00007F00)) == 0);
    REG_RPL_WR(RPL_RXADC_SATDET_CONFIG_ADDR, (REG_RPL_RD(RPL_RXADC_SATDET_CONFIG_ADDR) & ~((uint32_t)0x00007F00)) | ((uint32_t)rxadcsatdetthr << 8));
}

__INLINE uint8_t rpl_rxadc_satdet_window_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_RXADC_SATDET_CONFIG_ADDR);
    return ((localVal & ((uint32_t)0x0000007F)) >> 0);
}

__INLINE void rpl_rxadc_satdet_window_setf(uint8_t rxadcsatdetwindow)
{
    ASSERT_ERR((((uint32_t)rxadcsatdetwindow << 0) & ~((uint32_t)0x0000007F)) == 0);
    REG_RPL_WR(RPL_RXADC_SATDET_CONFIG_ADDR, (REG_RPL_RD(RPL_RXADC_SATDET_CONFIG_ADDR) & ~((uint32_t)0x0000007F)) | ((uint32_t)rxadcsatdetwindow << 0));
}

/**
 * @brief TXDC_DETECT_CONFIG register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     31            TXDC_MODE   0
 *  25:16       TXDC_REF_VALUE   0x26E
 *  07:00     TXDC_SETTLE_TIME   0x17
 * </pre>
 */
#define RPL_TXDC_DETECT_CONFIG_ADDR   0x00000079
#define RPL_TXDC_DETECT_CONFIG_OFFSET 0x00000079
#define RPL_TXDC_DETECT_CONFIG_INDEX  0x00000079
#define RPL_TXDC_DETECT_CONFIG_RESET  0x026E0017

__INLINE uint32_t rpl_txdc_detect_config_get(void)
{
    return REG_RPL_RD(RPL_TXDC_DETECT_CONFIG_ADDR);
}

__INLINE void rpl_txdc_detect_config_set(uint32_t value)
{
    REG_RPL_WR(RPL_TXDC_DETECT_CONFIG_ADDR, value);
}

// field definitions
#define RPL_TXDC_MODE_BIT           ((uint32_t)0x80000000)
#define RPL_TXDC_MODE_POS           31
#define RPL_TXDC_REF_VALUE_MASK     ((uint32_t)0x03FF0000)
#define RPL_TXDC_REF_VALUE_LSB      16
#define RPL_TXDC_REF_VALUE_WIDTH    ((uint32_t)0x0000000A)
#define RPL_TXDC_SETTLE_TIME_MASK   ((uint32_t)0x000000FF)
#define RPL_TXDC_SETTLE_TIME_LSB    0
#define RPL_TXDC_SETTLE_TIME_WIDTH  ((uint32_t)0x00000008)

#define RPL_TXDC_MODE_RST           0x0
#define RPL_TXDC_REF_VALUE_RST      0x26E
#define RPL_TXDC_SETTLE_TIME_RST    0x17

__INLINE void rpl_txdc_detect_config_pack(uint8_t txdcmode, uint16_t txdcrefvalue, uint8_t txdcsettletime)
{
    ASSERT_ERR((((uint32_t)txdcmode << 31) & ~((uint32_t)0x80000000)) == 0);
    ASSERT_ERR((((uint32_t)txdcrefvalue << 16) & ~((uint32_t)0x03FF0000)) == 0);
    ASSERT_ERR((((uint32_t)txdcsettletime << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_RPL_WR(RPL_TXDC_DETECT_CONFIG_ADDR, ((uint32_t)txdcmode << 31) | ((uint32_t)txdcrefvalue << 16) | ((uint32_t)txdcsettletime << 0));
}

__INLINE void rpl_txdc_detect_config_unpack(uint8_t *txdcmode, uint16_t *txdcrefvalue, uint8_t *txdcsettletime)
{
    uint32_t localVal = REG_RPL_RD(RPL_TXDC_DETECT_CONFIG_ADDR);

    *txdcmode = (localVal & ((uint32_t)0x80000000)) >> 31;
    *txdcrefvalue = (localVal & ((uint32_t)0x03FF0000)) >> 16;
    *txdcsettletime = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint8_t rpl_txdc_mode_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_TXDC_DETECT_CONFIG_ADDR);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE void rpl_txdc_mode_setf(uint8_t txdcmode)
{
    ASSERT_ERR((((uint32_t)txdcmode << 31) & ~((uint32_t)0x80000000)) == 0);
    REG_RPL_WR(RPL_TXDC_DETECT_CONFIG_ADDR, (REG_RPL_RD(RPL_TXDC_DETECT_CONFIG_ADDR) & ~((uint32_t)0x80000000)) | ((uint32_t)txdcmode << 31));
}

__INLINE uint16_t rpl_txdc_ref_value_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_TXDC_DETECT_CONFIG_ADDR);
    return ((localVal & ((uint32_t)0x03FF0000)) >> 16);
}

__INLINE void rpl_txdc_ref_value_setf(uint16_t txdcrefvalue)
{
    ASSERT_ERR((((uint32_t)txdcrefvalue << 16) & ~((uint32_t)0x03FF0000)) == 0);
    REG_RPL_WR(RPL_TXDC_DETECT_CONFIG_ADDR, (REG_RPL_RD(RPL_TXDC_DETECT_CONFIG_ADDR) & ~((uint32_t)0x03FF0000)) | ((uint32_t)txdcrefvalue << 16));
}

__INLINE uint8_t rpl_txdc_settle_time_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_TXDC_DETECT_CONFIG_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

__INLINE void rpl_txdc_settle_time_setf(uint8_t txdcsettletime)
{
    ASSERT_ERR((((uint32_t)txdcsettletime << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_RPL_WR(RPL_TXDC_DETECT_CONFIG_ADDR, (REG_RPL_RD(RPL_TXDC_DETECT_CONFIG_ADDR) & ~((uint32_t)0x000000FF)) | ((uint32_t)txdcsettletime << 0));
}

/**
 * @brief TESTMUXCNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:08            MUX1_CTRL   0x1
 *  07:00            MUX0_CTRL   0x0
 * </pre>
 */
#define RPL_TESTMUXCNTL_ADDR   0x0000007A
#define RPL_TESTMUXCNTL_OFFSET 0x0000007A
#define RPL_TESTMUXCNTL_INDEX  0x0000007A
#define RPL_TESTMUXCNTL_RESET  0x00000100

__INLINE uint32_t rpl_testmuxcntl_get(void)
{
    return REG_RPL_RD(RPL_TESTMUXCNTL_ADDR);
}

__INLINE void rpl_testmuxcntl_set(uint32_t value)
{
    REG_RPL_WR(RPL_TESTMUXCNTL_ADDR, value);
}

// field definitions
#define RPL_MUX1_CTRL_MASK   ((uint32_t)0x0000FF00)
#define RPL_MUX1_CTRL_LSB    8
#define RPL_MUX1_CTRL_WIDTH  ((uint32_t)0x00000008)
#define RPL_MUX0_CTRL_MASK   ((uint32_t)0x000000FF)
#define RPL_MUX0_CTRL_LSB    0
#define RPL_MUX0_CTRL_WIDTH  ((uint32_t)0x00000008)

#define RPL_MUX1_CTRL_RST    0x1
#define RPL_MUX0_CTRL_RST    0x0

__INLINE void rpl_testmuxcntl_pack(uint8_t mux1ctrl, uint8_t mux0ctrl)
{
    ASSERT_ERR((((uint32_t)mux1ctrl << 8) & ~((uint32_t)0x0000FF00)) == 0);
    ASSERT_ERR((((uint32_t)mux0ctrl << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_RPL_WR(RPL_TESTMUXCNTL_ADDR, ((uint32_t)mux1ctrl << 8) | ((uint32_t)mux0ctrl << 0));
}

__INLINE void rpl_testmuxcntl_unpack(uint8_t *mux1ctrl, uint8_t *mux0ctrl)
{
    uint32_t localVal = REG_RPL_RD(RPL_TESTMUXCNTL_ADDR);

    *mux1ctrl = (localVal & ((uint32_t)0x0000FF00)) >> 8;
    *mux0ctrl = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint8_t rpl_mux1_ctrl_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_TESTMUXCNTL_ADDR);
    return ((localVal & ((uint32_t)0x0000FF00)) >> 8);
}

__INLINE void rpl_mux1_ctrl_setf(uint8_t mux1ctrl)
{
    ASSERT_ERR((((uint32_t)mux1ctrl << 8) & ~((uint32_t)0x0000FF00)) == 0);
    REG_RPL_WR(RPL_TESTMUXCNTL_ADDR, (REG_RPL_RD(RPL_TESTMUXCNTL_ADDR) & ~((uint32_t)0x0000FF00)) | ((uint32_t)mux1ctrl << 8));
}

__INLINE uint8_t rpl_mux0_ctrl_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_TESTMUXCNTL_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

__INLINE void rpl_mux0_ctrl_setf(uint8_t mux0ctrl)
{
    ASSERT_ERR((((uint32_t)mux0ctrl << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_RPL_WR(RPL_TESTMUXCNTL_ADDR, (REG_RPL_RD(RPL_TESTMUXCNTL_ADDR) & ~((uint32_t)0x000000FF)) | ((uint32_t)mux0ctrl << 0));
}

/**
 * @brief PAD_CNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  23:16          DRVSTR_CNTL   0x0
 *  15:08          PULLDN_CNTL   0x0
 *  07:00          PULLUP_CNTL   0x0
 * </pre>
 */
#define RPL_PAD_CNTL_ADDR   0x0000007B
#define RPL_PAD_CNTL_OFFSET 0x0000007B
#define RPL_PAD_CNTL_INDEX  0x0000007B
#define RPL_PAD_CNTL_RESET  0x00000000

__INLINE uint32_t rpl_pad_cntl_get(void)
{
    return REG_RPL_RD(RPL_PAD_CNTL_ADDR);
}

__INLINE void rpl_pad_cntl_set(uint32_t value)
{
    REG_RPL_WR(RPL_PAD_CNTL_ADDR, value);
}

// field definitions
#define RPL_DRVSTR_CNTL_MASK   ((uint32_t)0x00FF0000)
#define RPL_DRVSTR_CNTL_LSB    16
#define RPL_DRVSTR_CNTL_WIDTH  ((uint32_t)0x00000008)
#define RPL_PULLDN_CNTL_MASK   ((uint32_t)0x0000FF00)
#define RPL_PULLDN_CNTL_LSB    8
#define RPL_PULLDN_CNTL_WIDTH  ((uint32_t)0x00000008)
#define RPL_PULLUP_CNTL_MASK   ((uint32_t)0x000000FF)
#define RPL_PULLUP_CNTL_LSB    0
#define RPL_PULLUP_CNTL_WIDTH  ((uint32_t)0x00000008)

#define RPL_DRVSTR_CNTL_RST    0x0
#define RPL_PULLDN_CNTL_RST    0x0
#define RPL_PULLUP_CNTL_RST    0x0

__INLINE void rpl_pad_cntl_pack(uint8_t drvstrcntl, uint8_t pulldncntl, uint8_t pullupcntl)
{
    ASSERT_ERR((((uint32_t)drvstrcntl << 16) & ~((uint32_t)0x00FF0000)) == 0);
    ASSERT_ERR((((uint32_t)pulldncntl << 8) & ~((uint32_t)0x0000FF00)) == 0);
    ASSERT_ERR((((uint32_t)pullupcntl << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_RPL_WR(RPL_PAD_CNTL_ADDR, ((uint32_t)drvstrcntl << 16) | ((uint32_t)pulldncntl << 8) | ((uint32_t)pullupcntl << 0));
}

__INLINE void rpl_pad_cntl_unpack(uint8_t *drvstrcntl, uint8_t *pulldncntl, uint8_t *pullupcntl)
{
    uint32_t localVal = REG_RPL_RD(RPL_PAD_CNTL_ADDR);

    *drvstrcntl = (localVal & ((uint32_t)0x00FF0000)) >> 16;
    *pulldncntl = (localVal & ((uint32_t)0x0000FF00)) >> 8;
    *pullupcntl = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint8_t rpl_drvstr_cntl_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_PAD_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x00FF0000)) >> 16);
}

__INLINE void rpl_drvstr_cntl_setf(uint8_t drvstrcntl)
{
    ASSERT_ERR((((uint32_t)drvstrcntl << 16) & ~((uint32_t)0x00FF0000)) == 0);
    REG_RPL_WR(RPL_PAD_CNTL_ADDR, (REG_RPL_RD(RPL_PAD_CNTL_ADDR) & ~((uint32_t)0x00FF0000)) | ((uint32_t)drvstrcntl << 16));
}

__INLINE uint8_t rpl_pulldn_cntl_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_PAD_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x0000FF00)) >> 8);
}

__INLINE void rpl_pulldn_cntl_setf(uint8_t pulldncntl)
{
    ASSERT_ERR((((uint32_t)pulldncntl << 8) & ~((uint32_t)0x0000FF00)) == 0);
    REG_RPL_WR(RPL_PAD_CNTL_ADDR, (REG_RPL_RD(RPL_PAD_CNTL_ADDR) & ~((uint32_t)0x0000FF00)) | ((uint32_t)pulldncntl << 8));
}

__INLINE uint8_t rpl_pullup_cntl_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_PAD_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

__INLINE void rpl_pullup_cntl_setf(uint8_t pullupcntl)
{
    ASSERT_ERR((((uint32_t)pullupcntl << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_RPL_WR(RPL_PAD_CNTL_ADDR, (REG_RPL_RD(RPL_PAD_CNTL_ADDR) & ~((uint32_t)0x000000FF)) | ((uint32_t)pullupcntl << 0));
}

/**
 * @brief SPARE_CNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00            SPARE_OUT   0x0
 * </pre>
 */
#define RPL_SPARE_CNTL_ADDR   0x0000007C
#define RPL_SPARE_CNTL_OFFSET 0x0000007C
#define RPL_SPARE_CNTL_INDEX  0x0000007C
#define RPL_SPARE_CNTL_RESET  0x00000000

__INLINE uint32_t rpl_spare_cntl_get(void)
{
    return REG_RPL_RD(RPL_SPARE_CNTL_ADDR);
}

__INLINE void rpl_spare_cntl_set(uint32_t value)
{
    REG_RPL_WR(RPL_SPARE_CNTL_ADDR, value);
}

// field definitions
#define RPL_SPARE_OUT_MASK   ((uint32_t)0xFFFFFFFF)
#define RPL_SPARE_OUT_LSB    0
#define RPL_SPARE_OUT_WIDTH  ((uint32_t)0x00000020)

#define RPL_SPARE_OUT_RST    0x0

__INLINE uint32_t rpl_spare_out_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_SPARE_CNTL_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0xFFFFFFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void rpl_spare_out_setf(uint32_t spareout)
{
    ASSERT_ERR((((uint32_t)spareout << 0) & ~((uint32_t)0xFFFFFFFF)) == 0);
    REG_RPL_WR(RPL_SPARE_CNTL_ADDR, (uint32_t)spareout << 0);
}

/**
 * @brief SPARE_STAT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00             SPARE_IN   0x0
 * </pre>
 */
#define RPL_SPARE_STAT_ADDR   0x0000007D
#define RPL_SPARE_STAT_OFFSET 0x0000007D
#define RPL_SPARE_STAT_INDEX  0x0000007D
#define RPL_SPARE_STAT_RESET  0x00000000

__INLINE uint32_t rpl_spare_stat_get(void)
{
    return REG_RPL_RD(RPL_SPARE_STAT_ADDR);
}

// field definitions
#define RPL_SPARE_IN_MASK   ((uint32_t)0xFFFFFFFF)
#define RPL_SPARE_IN_LSB    0
#define RPL_SPARE_IN_WIDTH  ((uint32_t)0x00000020)

#define RPL_SPARE_IN_RST    0x0

__INLINE uint32_t rpl_spare_in_getf(void)
{
    uint32_t localVal = REG_RPL_RD(RPL_SPARE_STAT_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0xFFFFFFFF)) == 0);
    return (localVal >> 0);
}


#endif // _REG_RIPPLE_H_

