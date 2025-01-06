#ifndef _REG_BTCORE_ESCO_H_
#define _REG_BTCORE_ESCO_H_

#include <stdint.h>
#include "_reg_btcore_esco.h"
#include "compiler.h"
#include "arch.h"
#include "reg_access.h"

#define REG_BTCORE_ESCO_COUNT 140

#define REG_BTCORE_ESCO_DECODING_MASK 0x000003FF

/**
 * @brief E_SCOCHANCNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     31                  TOG   0
 *     15         eSCOCHANSWEN   0
 *     14           eSCOCHANEN   0
 *     13              ITMODE2   0
 *  12:08             INTDELAY   0x0
 *  07:00                TeSCO   0x0
 * </pre>
 */
#define BT_E_SCOCHANCNTL_ADDR   0x40090610
#define BT_E_SCOCHANCNTL_OFFSET 0x00000210
#define BT_E_SCOCHANCNTL_INDEX  0x00000084
#define BT_E_SCOCHANCNTL_RESET  0x00000000

__INLINE uint32_t bt_e_scochancntl_get(int elt_idx)
{
    return REG_BT_RD(BT_E_SCOCHANCNTL_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE);
}

__INLINE void bt_e_scochancntl_set(int elt_idx, uint32_t value)
{
    REG_BT_WR(BT_E_SCOCHANCNTL_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE, value);
}

// field definitions
#define BT_TOG_BIT              ((uint32_t)0x80000000)
#define BT_TOG_POS              31
#define BT_E_SCOCHANSWEN_BIT    ((uint32_t)0x00008000)
#define BT_E_SCOCHANSWEN_POS    15
#define BT_E_SCOCHANEN_BIT      ((uint32_t)0x00004000)
#define BT_E_SCOCHANEN_POS      14
#define BT_ITMODE2_BIT          ((uint32_t)0x00002000)
#define BT_ITMODE2_POS          13
#define BT_INTDELAY_MASK        ((uint32_t)0x00001F00)
#define BT_INTDELAY_LSB         8
#define BT_INTDELAY_WIDTH       ((uint32_t)0x00000005)
#define BT_TE_SCO_MASK          ((uint32_t)0x000000FF)
#define BT_TE_SCO_LSB           0
#define BT_TE_SCO_WIDTH         ((uint32_t)0x00000008)

#define BT_TOG_RST              0x0
#define BT_E_SCOCHANSWEN_RST    0x0
#define BT_E_SCOCHANEN_RST      0x0
#define BT_ITMODE2_RST          0x0
#define BT_INTDELAY_RST         0x0
#define BT_TE_SCO_RST           0x0

__INLINE void bt_e_scochancntl_pack(int elt_idx, uint8_t escochanswen, uint8_t escochanen, uint8_t itmode2, uint8_t intdelay, uint8_t tesco)
{
    ASSERT_ERR((((uint32_t)escochanswen << 15) & ~((uint32_t)0x00008000)) == 0);
    ASSERT_ERR((((uint32_t)escochanen << 14) & ~((uint32_t)0x00004000)) == 0);
    ASSERT_ERR((((uint32_t)itmode2 << 13) & ~((uint32_t)0x00002000)) == 0);
    ASSERT_ERR((((uint32_t)intdelay << 8) & ~((uint32_t)0x00001F00)) == 0);
    ASSERT_ERR((((uint32_t)tesco << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_BT_WR(BT_E_SCOCHANCNTL_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE, ((uint32_t)escochanswen << 15) | ((uint32_t)escochanen << 14) | ((uint32_t)itmode2 << 13) | ((uint32_t)intdelay << 8) | ((uint32_t)tesco << 0));
}

__INLINE void bt_e_scochancntl_unpack(int elt_idx, uint8_t *tog, uint8_t *escochanswen, uint8_t *escochanen, uint8_t *itmode2, uint8_t *intdelay, uint8_t *tesco)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOCHANCNTL_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE);

    *tog = (localVal & ((uint32_t)0x80000000)) >> 31;
    *escochanswen = (localVal & ((uint32_t)0x00008000)) >> 15;
    *escochanen = (localVal & ((uint32_t)0x00004000)) >> 14;
    *itmode2 = (localVal & ((uint32_t)0x00002000)) >> 13;
    *intdelay = (localVal & ((uint32_t)0x00001F00)) >> 8;
    *tesco = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint8_t bt_e_scochancntl_tog_getf(int elt_idx)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOCHANCNTL_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE uint8_t bt_e_scochancntl_e_scochanswen_getf(int elt_idx)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOCHANCNTL_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE);
    return ((localVal & ((uint32_t)0x00008000)) >> 15);
}

__INLINE void bt_e_scochancntl_e_scochanswen_setf(int elt_idx, uint8_t escochanswen)
{
    ASSERT_ERR((((uint32_t)escochanswen << 15) & ~((uint32_t)0x00008000)) == 0);
    REG_BT_WR(BT_E_SCOCHANCNTL_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE, (REG_BT_RD(BT_E_SCOCHANCNTL_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE) & ~((uint32_t)0x00008000)) | ((uint32_t)escochanswen << 15));
}

__INLINE uint8_t bt_e_scochancntl_e_scochanen_getf(int elt_idx)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOCHANCNTL_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE);
    return ((localVal & ((uint32_t)0x00004000)) >> 14);
}

__INLINE void bt_e_scochancntl_e_scochanen_setf(int elt_idx, uint8_t escochanen)
{
    ASSERT_ERR((((uint32_t)escochanen << 14) & ~((uint32_t)0x00004000)) == 0);
    REG_BT_WR(BT_E_SCOCHANCNTL_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE, (REG_BT_RD(BT_E_SCOCHANCNTL_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE) & ~((uint32_t)0x00004000)) | ((uint32_t)escochanen << 14));
}

__INLINE uint8_t bt_e_scochancntl_itmode2_getf(int elt_idx)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOCHANCNTL_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE);
    return ((localVal & ((uint32_t)0x00002000)) >> 13);
}

__INLINE void bt_e_scochancntl_itmode2_setf(int elt_idx, uint8_t itmode2)
{
    ASSERT_ERR((((uint32_t)itmode2 << 13) & ~((uint32_t)0x00002000)) == 0);
    REG_BT_WR(BT_E_SCOCHANCNTL_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE, (REG_BT_RD(BT_E_SCOCHANCNTL_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE) & ~((uint32_t)0x00002000)) | ((uint32_t)itmode2 << 13));
}

__INLINE uint8_t bt_e_scochancntl_intdelay_getf(int elt_idx)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOCHANCNTL_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE);
    return ((localVal & ((uint32_t)0x00001F00)) >> 8);
}

__INLINE void bt_e_scochancntl_intdelay_setf(int elt_idx, uint8_t intdelay)
{
    ASSERT_ERR((((uint32_t)intdelay << 8) & ~((uint32_t)0x00001F00)) == 0);
    REG_BT_WR(BT_E_SCOCHANCNTL_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE, (REG_BT_RD(BT_E_SCOCHANCNTL_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE) & ~((uint32_t)0x00001F00)) | ((uint32_t)intdelay << 8));
}

__INLINE uint8_t bt_e_scochancntl_te_sco_getf(int elt_idx)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOCHANCNTL_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

__INLINE void bt_e_scochancntl_te_sco_setf(int elt_idx, uint8_t tesco)
{
    ASSERT_ERR((((uint32_t)tesco << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_BT_WR(BT_E_SCOCHANCNTL_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE, (REG_BT_RD(BT_E_SCOCHANCNTL_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE) & ~((uint32_t)0x000000FF)) | ((uint32_t)tesco << 0));
}

/**
 * @brief E_SCOMUTECNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     23            MUTE_SINK   0
 *     22          MUTE_SOURCE   0
 *  19:18               INVL_1   0x2
 *  17:16               INVL_0   0x2
 *  15:00             MUTEPATT   0x0
 * </pre>
 */
#define BT_E_SCOMUTECNTL_ADDR   0x40090614
#define BT_E_SCOMUTECNTL_OFFSET 0x00000214
#define BT_E_SCOMUTECNTL_INDEX  0x00000085
#define BT_E_SCOMUTECNTL_RESET  0x000A0000

__INLINE uint32_t bt_e_scomutecntl_get(int elt_idx)
{
    return REG_BT_RD(BT_E_SCOMUTECNTL_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE);
}

__INLINE void bt_e_scomutecntl_set(int elt_idx, uint32_t value)
{
    REG_BT_WR(BT_E_SCOMUTECNTL_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE, value);
}

// field definitions
#define BT_MUTE_SINK_BIT      ((uint32_t)0x00800000)
#define BT_MUTE_SINK_POS      23
#define BT_MUTE_SOURCE_BIT    ((uint32_t)0x00400000)
#define BT_MUTE_SOURCE_POS    22
#define BT_INVL_1_MASK        ((uint32_t)0x000C0000)
#define BT_INVL_1_LSB         18
#define BT_INVL_1_WIDTH       ((uint32_t)0x00000002)
#define BT_INVL_0_MASK        ((uint32_t)0x00030000)
#define BT_INVL_0_LSB         16
#define BT_INVL_0_WIDTH       ((uint32_t)0x00000002)
#define BT_MUTEPATT_MASK      ((uint32_t)0x0000FFFF)
#define BT_MUTEPATT_LSB       0
#define BT_MUTEPATT_WIDTH     ((uint32_t)0x00000010)

#define BT_MUTE_SINK_RST      0x0
#define BT_MUTE_SOURCE_RST    0x0
#define BT_INVL_1_RST         0x2
#define BT_INVL_0_RST         0x2
#define BT_MUTEPATT_RST       0x0

__INLINE void bt_e_scomutecntl_pack(int elt_idx, uint8_t mutesink, uint8_t mutesource, uint8_t invl1, uint8_t invl0, uint16_t mutepatt)
{
    ASSERT_ERR((((uint32_t)mutesink << 23) & ~((uint32_t)0x00800000)) == 0);
    ASSERT_ERR((((uint32_t)mutesource << 22) & ~((uint32_t)0x00400000)) == 0);
    ASSERT_ERR((((uint32_t)invl1 << 18) & ~((uint32_t)0x000C0000)) == 0);
    ASSERT_ERR((((uint32_t)invl0 << 16) & ~((uint32_t)0x00030000)) == 0);
    ASSERT_ERR((((uint32_t)mutepatt << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_BT_WR(BT_E_SCOMUTECNTL_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE, ((uint32_t)mutesink << 23) | ((uint32_t)mutesource << 22) | ((uint32_t)invl1 << 18) | ((uint32_t)invl0 << 16) | ((uint32_t)mutepatt << 0));
}

__INLINE void bt_e_scomutecntl_unpack(int elt_idx, uint8_t *mutesink, uint8_t *mutesource, uint8_t *invl1, uint8_t *invl0, uint16_t *mutepatt)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOMUTECNTL_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE);

    *mutesink = (localVal & ((uint32_t)0x00800000)) >> 23;
    *mutesource = (localVal & ((uint32_t)0x00400000)) >> 22;
    *invl1 = (localVal & ((uint32_t)0x000C0000)) >> 18;
    *invl0 = (localVal & ((uint32_t)0x00030000)) >> 16;
    *mutepatt = (localVal & ((uint32_t)0x0000FFFF)) >> 0;
}

__INLINE uint8_t bt_e_scomutecntl_mute_sink_getf(int elt_idx)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOMUTECNTL_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE);
    return ((localVal & ((uint32_t)0x00800000)) >> 23);
}

__INLINE void bt_e_scomutecntl_mute_sink_setf(int elt_idx, uint8_t mutesink)
{
    ASSERT_ERR((((uint32_t)mutesink << 23) & ~((uint32_t)0x00800000)) == 0);
    REG_BT_WR(BT_E_SCOMUTECNTL_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE, (REG_BT_RD(BT_E_SCOMUTECNTL_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE) & ~((uint32_t)0x00800000)) | ((uint32_t)mutesink << 23));
}

__INLINE uint8_t bt_e_scomutecntl_mute_source_getf(int elt_idx)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOMUTECNTL_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE);
    return ((localVal & ((uint32_t)0x00400000)) >> 22);
}

__INLINE void bt_e_scomutecntl_mute_source_setf(int elt_idx, uint8_t mutesource)
{
    ASSERT_ERR((((uint32_t)mutesource << 22) & ~((uint32_t)0x00400000)) == 0);
    REG_BT_WR(BT_E_SCOMUTECNTL_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE, (REG_BT_RD(BT_E_SCOMUTECNTL_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE) & ~((uint32_t)0x00400000)) | ((uint32_t)mutesource << 22));
}

__INLINE uint8_t bt_e_scomutecntl_invl_1_getf(int elt_idx)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOMUTECNTL_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE);
    return ((localVal & ((uint32_t)0x000C0000)) >> 18);
}

__INLINE void bt_e_scomutecntl_invl_1_setf(int elt_idx, uint8_t invl1)
{
    ASSERT_ERR((((uint32_t)invl1 << 18) & ~((uint32_t)0x000C0000)) == 0);
    REG_BT_WR(BT_E_SCOMUTECNTL_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE, (REG_BT_RD(BT_E_SCOMUTECNTL_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE) & ~((uint32_t)0x000C0000)) | ((uint32_t)invl1 << 18));
}

__INLINE uint8_t bt_e_scomutecntl_invl_0_getf(int elt_idx)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOMUTECNTL_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE);
    return ((localVal & ((uint32_t)0x00030000)) >> 16);
}

__INLINE void bt_e_scomutecntl_invl_0_setf(int elt_idx, uint8_t invl0)
{
    ASSERT_ERR((((uint32_t)invl0 << 16) & ~((uint32_t)0x00030000)) == 0);
    REG_BT_WR(BT_E_SCOMUTECNTL_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE, (REG_BT_RD(BT_E_SCOMUTECNTL_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE) & ~((uint32_t)0x00030000)) | ((uint32_t)invl0 << 16));
}

__INLINE uint16_t bt_e_scomutecntl_mutepatt_getf(int elt_idx)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOMUTECNTL_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE);
    return ((localVal & ((uint32_t)0x0000FFFF)) >> 0);
}

__INLINE void bt_e_scomutecntl_mutepatt_setf(int elt_idx, uint16_t mutepatt)
{
    ASSERT_ERR((((uint32_t)mutepatt << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_BT_WR(BT_E_SCOMUTECNTL_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE, (REG_BT_RD(BT_E_SCOMUTECNTL_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE) & ~((uint32_t)0x0000FFFF)) | ((uint32_t)mutepatt << 0));
}

/**
 * @brief E_SCOCURRENTTXPTR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  29:16           eSCOPTRTX1   0x0
 *  13:00           eSCOPTRTX0   0x0
 * </pre>
 */
#define BT_E_SCOCURRENTTXPTR_ADDR   0x40090618
#define BT_E_SCOCURRENTTXPTR_OFFSET 0x00000218
#define BT_E_SCOCURRENTTXPTR_INDEX  0x00000086
#define BT_E_SCOCURRENTTXPTR_RESET  0x00000000

__INLINE uint32_t bt_e_scocurrenttxptr_get(int elt_idx)
{
    return REG_BT_RD(BT_E_SCOCURRENTTXPTR_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE);
}

__INLINE void bt_e_scocurrenttxptr_set(int elt_idx, uint32_t value)
{
    REG_BT_WR(BT_E_SCOCURRENTTXPTR_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE, value);
}

// field definitions
#define BT_E_SCOPTRTX_1_MASK   ((uint32_t)0x3FFF0000)
#define BT_E_SCOPTRTX_1_LSB    16
#define BT_E_SCOPTRTX_1_WIDTH  ((uint32_t)0x0000000E)
#define BT_E_SCOPTRTX_0_MASK   ((uint32_t)0x00003FFF)
#define BT_E_SCOPTRTX_0_LSB    0
#define BT_E_SCOPTRTX_0_WIDTH  ((uint32_t)0x0000000E)

#define BT_E_SCOPTRTX_1_RST    0x0
#define BT_E_SCOPTRTX_0_RST    0x0

__INLINE void bt_e_scocurrenttxptr_pack(int elt_idx, uint16_t escoptrtx1, uint16_t escoptrtx0)
{
    ASSERT_ERR((((uint32_t)escoptrtx1 << 16) & ~((uint32_t)0x3FFF0000)) == 0);
    ASSERT_ERR((((uint32_t)escoptrtx0 << 0) & ~((uint32_t)0x00003FFF)) == 0);
    REG_BT_WR(BT_E_SCOCURRENTTXPTR_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE, ((uint32_t)escoptrtx1 << 16) | ((uint32_t)escoptrtx0 << 0));
}

__INLINE void bt_e_scocurrenttxptr_unpack(int elt_idx, uint16_t *escoptrtx1, uint16_t *escoptrtx0)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOCURRENTTXPTR_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE);

    *escoptrtx1 = (localVal & ((uint32_t)0x3FFF0000)) >> 16;
    *escoptrtx0 = (localVal & ((uint32_t)0x00003FFF)) >> 0;
}

__INLINE uint16_t bt_e_scocurrenttxptr_e_scoptrtx_1_getf(int elt_idx)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOCURRENTTXPTR_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE);
    return ((localVal & ((uint32_t)0x3FFF0000)) >> 16);
}

__INLINE void bt_e_scocurrenttxptr_e_scoptrtx_1_setf(int elt_idx, uint16_t escoptrtx1)
{
    ASSERT_ERR((((uint32_t)escoptrtx1 << 16) & ~((uint32_t)0x3FFF0000)) == 0);
    REG_BT_WR(BT_E_SCOCURRENTTXPTR_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE, (REG_BT_RD(BT_E_SCOCURRENTTXPTR_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE) & ~((uint32_t)0x3FFF0000)) | ((uint32_t)escoptrtx1 << 16));
}

__INLINE uint16_t bt_e_scocurrenttxptr_e_scoptrtx_0_getf(int elt_idx)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOCURRENTTXPTR_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE);
    return ((localVal & ((uint32_t)0x00003FFF)) >> 0);
}

__INLINE void bt_e_scocurrenttxptr_e_scoptrtx_0_setf(int elt_idx, uint16_t escoptrtx0)
{
    ASSERT_ERR((((uint32_t)escoptrtx0 << 0) & ~((uint32_t)0x00003FFF)) == 0);
    REG_BT_WR(BT_E_SCOCURRENTTXPTR_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE, (REG_BT_RD(BT_E_SCOCURRENTTXPTR_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE) & ~((uint32_t)0x00003FFF)) | ((uint32_t)escoptrtx0 << 0));
}

/**
 * @brief E_SCOCURRENTRXPTR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  29:16           eSCOPTRRX1   0x0
 *  13:00           eSCOPTRRX0   0x0
 * </pre>
 */
#define BT_E_SCOCURRENTRXPTR_ADDR   0x4009061C
#define BT_E_SCOCURRENTRXPTR_OFFSET 0x0000021C
#define BT_E_SCOCURRENTRXPTR_INDEX  0x00000087
#define BT_E_SCOCURRENTRXPTR_RESET  0x00000000

__INLINE uint32_t bt_e_scocurrentrxptr_get(int elt_idx)
{
    return REG_BT_RD(BT_E_SCOCURRENTRXPTR_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE);
}

__INLINE void bt_e_scocurrentrxptr_set(int elt_idx, uint32_t value)
{
    REG_BT_WR(BT_E_SCOCURRENTRXPTR_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE, value);
}

// field definitions
#define BT_E_SCOPTRRX_1_MASK   ((uint32_t)0x3FFF0000)
#define BT_E_SCOPTRRX_1_LSB    16
#define BT_E_SCOPTRRX_1_WIDTH  ((uint32_t)0x0000000E)
#define BT_E_SCOPTRRX_0_MASK   ((uint32_t)0x00003FFF)
#define BT_E_SCOPTRRX_0_LSB    0
#define BT_E_SCOPTRRX_0_WIDTH  ((uint32_t)0x0000000E)

#define BT_E_SCOPTRRX_1_RST    0x0
#define BT_E_SCOPTRRX_0_RST    0x0

__INLINE void bt_e_scocurrentrxptr_pack(int elt_idx, uint16_t escoptrrx1, uint16_t escoptrrx0)
{
    ASSERT_ERR((((uint32_t)escoptrrx1 << 16) & ~((uint32_t)0x3FFF0000)) == 0);
    ASSERT_ERR((((uint32_t)escoptrrx0 << 0) & ~((uint32_t)0x00003FFF)) == 0);
    REG_BT_WR(BT_E_SCOCURRENTRXPTR_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE, ((uint32_t)escoptrrx1 << 16) | ((uint32_t)escoptrrx0 << 0));
}

__INLINE void bt_e_scocurrentrxptr_unpack(int elt_idx, uint16_t *escoptrrx1, uint16_t *escoptrrx0)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOCURRENTRXPTR_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE);

    *escoptrrx1 = (localVal & ((uint32_t)0x3FFF0000)) >> 16;
    *escoptrrx0 = (localVal & ((uint32_t)0x00003FFF)) >> 0;
}

__INLINE uint16_t bt_e_scocurrentrxptr_e_scoptrrx_1_getf(int elt_idx)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOCURRENTRXPTR_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE);
    return ((localVal & ((uint32_t)0x3FFF0000)) >> 16);
}

__INLINE void bt_e_scocurrentrxptr_e_scoptrrx_1_setf(int elt_idx, uint16_t escoptrrx1)
{
    ASSERT_ERR((((uint32_t)escoptrrx1 << 16) & ~((uint32_t)0x3FFF0000)) == 0);
    REG_BT_WR(BT_E_SCOCURRENTRXPTR_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE, (REG_BT_RD(BT_E_SCOCURRENTRXPTR_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE) & ~((uint32_t)0x3FFF0000)) | ((uint32_t)escoptrrx1 << 16));
}

__INLINE uint16_t bt_e_scocurrentrxptr_e_scoptrrx_0_getf(int elt_idx)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOCURRENTRXPTR_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE);
    return ((localVal & ((uint32_t)0x00003FFF)) >> 0);
}

__INLINE void bt_e_scocurrentrxptr_e_scoptrrx_0_setf(int elt_idx, uint16_t escoptrrx0)
{
    ASSERT_ERR((((uint32_t)escoptrrx0 << 0) & ~((uint32_t)0x00003FFF)) == 0);
    REG_BT_WR(BT_E_SCOCURRENTRXPTR_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE, (REG_BT_RD(BT_E_SCOCURRENTRXPTR_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE) & ~((uint32_t)0x00003FFF)) | ((uint32_t)escoptrrx0 << 0));
}

/**
 * @brief E_SCOLTCNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  23:16               RETXNB   0x1
 *     05            eSCOEDRRX   0
 *     04            eSCOEDRTX   0
 *     03              SYNTYPE   0
 *  02:00            SYNLTADDR   0x0
 * </pre>
 */
#define BT_E_SCOLTCNTL_ADDR   0x40090620
#define BT_E_SCOLTCNTL_OFFSET 0x00000220
#define BT_E_SCOLTCNTL_INDEX  0x00000088
#define BT_E_SCOLTCNTL_RESET  0x00010000

__INLINE uint32_t bt_e_scoltcntl_get(int elt_idx)
{
    return REG_BT_RD(BT_E_SCOLTCNTL_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE);
}

__INLINE void bt_e_scoltcntl_set(int elt_idx, uint32_t value)
{
    REG_BT_WR(BT_E_SCOLTCNTL_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE, value);
}

// field definitions
#define BT_RETXNB_MASK       ((uint32_t)0x00FF0000)
#define BT_RETXNB_LSB        16
#define BT_RETXNB_WIDTH      ((uint32_t)0x00000008)
#define BT_E_SCOEDRRX_BIT    ((uint32_t)0x00000020)
#define BT_E_SCOEDRRX_POS    5
#define BT_E_SCOEDRTX_BIT    ((uint32_t)0x00000010)
#define BT_E_SCOEDRTX_POS    4
#define BT_SYNTYPE_BIT       ((uint32_t)0x00000008)
#define BT_SYNTYPE_POS       3
#define BT_SYNLTADDR_MASK    ((uint32_t)0x00000007)
#define BT_SYNLTADDR_LSB     0
#define BT_SYNLTADDR_WIDTH   ((uint32_t)0x00000003)

#define BT_RETXNB_RST        0x1
#define BT_E_SCOEDRRX_RST    0x0
#define BT_E_SCOEDRTX_RST    0x0
#define BT_SYNTYPE_RST       0x0
#define BT_SYNLTADDR_RST     0x0

__INLINE void bt_e_scoltcntl_pack(int elt_idx, uint8_t retxnb, uint8_t escoedrrx, uint8_t escoedrtx, uint8_t syntype, uint8_t synltaddr)
{
    ASSERT_ERR((((uint32_t)retxnb << 16) & ~((uint32_t)0x00FF0000)) == 0);
    ASSERT_ERR((((uint32_t)escoedrrx << 5) & ~((uint32_t)0x00000020)) == 0);
    ASSERT_ERR((((uint32_t)escoedrtx << 4) & ~((uint32_t)0x00000010)) == 0);
    ASSERT_ERR((((uint32_t)syntype << 3) & ~((uint32_t)0x00000008)) == 0);
    ASSERT_ERR((((uint32_t)synltaddr << 0) & ~((uint32_t)0x00000007)) == 0);
    REG_BT_WR(BT_E_SCOLTCNTL_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE, ((uint32_t)retxnb << 16) | ((uint32_t)escoedrrx << 5) | ((uint32_t)escoedrtx << 4) | ((uint32_t)syntype << 3) | ((uint32_t)synltaddr << 0));
}

__INLINE void bt_e_scoltcntl_unpack(int elt_idx, uint8_t *retxnb, uint8_t *escoedrrx, uint8_t *escoedrtx, uint8_t *syntype, uint8_t *synltaddr)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOLTCNTL_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE);

    *retxnb = (localVal & ((uint32_t)0x00FF0000)) >> 16;
    *escoedrrx = (localVal & ((uint32_t)0x00000020)) >> 5;
    *escoedrtx = (localVal & ((uint32_t)0x00000010)) >> 4;
    *syntype = (localVal & ((uint32_t)0x00000008)) >> 3;
    *synltaddr = (localVal & ((uint32_t)0x00000007)) >> 0;
}

__INLINE uint8_t bt_e_scoltcntl_retxnb_getf(int elt_idx)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOLTCNTL_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE);
    return ((localVal & ((uint32_t)0x00FF0000)) >> 16);
}

__INLINE void bt_e_scoltcntl_retxnb_setf(int elt_idx, uint8_t retxnb)
{
    ASSERT_ERR((((uint32_t)retxnb << 16) & ~((uint32_t)0x00FF0000)) == 0);
    REG_BT_WR(BT_E_SCOLTCNTL_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE, (REG_BT_RD(BT_E_SCOLTCNTL_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE) & ~((uint32_t)0x00FF0000)) | ((uint32_t)retxnb << 16));
}

__INLINE uint8_t bt_e_scoltcntl_e_scoedrrx_getf(int elt_idx)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOLTCNTL_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE);
    return ((localVal & ((uint32_t)0x00000020)) >> 5);
}

__INLINE void bt_e_scoltcntl_e_scoedrrx_setf(int elt_idx, uint8_t escoedrrx)
{
    ASSERT_ERR((((uint32_t)escoedrrx << 5) & ~((uint32_t)0x00000020)) == 0);
    REG_BT_WR(BT_E_SCOLTCNTL_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE, (REG_BT_RD(BT_E_SCOLTCNTL_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE) & ~((uint32_t)0x00000020)) | ((uint32_t)escoedrrx << 5));
}

__INLINE uint8_t bt_e_scoltcntl_e_scoedrtx_getf(int elt_idx)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOLTCNTL_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE void bt_e_scoltcntl_e_scoedrtx_setf(int elt_idx, uint8_t escoedrtx)
{
    ASSERT_ERR((((uint32_t)escoedrtx << 4) & ~((uint32_t)0x00000010)) == 0);
    REG_BT_WR(BT_E_SCOLTCNTL_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE, (REG_BT_RD(BT_E_SCOLTCNTL_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE) & ~((uint32_t)0x00000010)) | ((uint32_t)escoedrtx << 4));
}

__INLINE uint8_t bt_e_scoltcntl_syntype_getf(int elt_idx)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOLTCNTL_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

__INLINE void bt_e_scoltcntl_syntype_setf(int elt_idx, uint8_t syntype)
{
    ASSERT_ERR((((uint32_t)syntype << 3) & ~((uint32_t)0x00000008)) == 0);
    REG_BT_WR(BT_E_SCOLTCNTL_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE, (REG_BT_RD(BT_E_SCOLTCNTL_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE) & ~((uint32_t)0x00000008)) | ((uint32_t)syntype << 3));
}

__INLINE uint8_t bt_e_scoltcntl_synltaddr_getf(int elt_idx)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOLTCNTL_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE);
    return ((localVal & ((uint32_t)0x00000007)) >> 0);
}

__INLINE void bt_e_scoltcntl_synltaddr_setf(int elt_idx, uint8_t synltaddr)
{
    ASSERT_ERR((((uint32_t)synltaddr << 0) & ~((uint32_t)0x00000007)) == 0);
    REG_BT_WR(BT_E_SCOLTCNTL_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE, (REG_BT_RD(BT_E_SCOLTCNTL_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE) & ~((uint32_t)0x00000007)) | ((uint32_t)synltaddr << 0));
}

/**
 * @brief E_SCOTRCNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     31               TXSEQN   0
 *  29:20                TXLEN   0x0
 *  19:16               TXTYPE   0x0
 *  13:04                RXLEN   0x0
 *  03:00               RXTYPE   0x0
 * </pre>
 */
#define BT_E_SCOTRCNTL_ADDR   0x40090624
#define BT_E_SCOTRCNTL_OFFSET 0x00000224
#define BT_E_SCOTRCNTL_INDEX  0x00000089
#define BT_E_SCOTRCNTL_RESET  0x00000000

__INLINE uint32_t bt_e_scotrcntl_get(int elt_idx)
{
    return REG_BT_RD(BT_E_SCOTRCNTL_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE);
}

__INLINE void bt_e_scotrcntl_set(int elt_idx, uint32_t value)
{
    REG_BT_WR(BT_E_SCOTRCNTL_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE, value);
}

// field definitions
#define BT_TXSEQN_BIT    ((uint32_t)0x80000000)
#define BT_TXSEQN_POS    31
#define BT_TXLEN_MASK    ((uint32_t)0x3FF00000)
#define BT_TXLEN_LSB     20
#define BT_TXLEN_WIDTH   ((uint32_t)0x0000000A)
#define BT_TXTYPE_MASK   ((uint32_t)0x000F0000)
#define BT_TXTYPE_LSB    16
#define BT_TXTYPE_WIDTH  ((uint32_t)0x00000004)
#define BT_RXLEN_MASK    ((uint32_t)0x00003FF0)
#define BT_RXLEN_LSB     4
#define BT_RXLEN_WIDTH   ((uint32_t)0x0000000A)
#define BT_RXTYPE_MASK   ((uint32_t)0x0000000F)
#define BT_RXTYPE_LSB    0
#define BT_RXTYPE_WIDTH  ((uint32_t)0x00000004)

#define BT_TXSEQN_RST    0x0
#define BT_TXLEN_RST     0x0
#define BT_TXTYPE_RST    0x0
#define BT_RXLEN_RST     0x0
#define BT_RXTYPE_RST    0x0

__INLINE void bt_e_scotrcntl_pack(int elt_idx, uint8_t txseqn, uint16_t txlen, uint8_t txtype, uint16_t rxlen, uint8_t rxtype)
{
    ASSERT_ERR((((uint32_t)txseqn << 31) & ~((uint32_t)0x80000000)) == 0);
    ASSERT_ERR((((uint32_t)txlen << 20) & ~((uint32_t)0x3FF00000)) == 0);
    ASSERT_ERR((((uint32_t)txtype << 16) & ~((uint32_t)0x000F0000)) == 0);
    ASSERT_ERR((((uint32_t)rxlen << 4) & ~((uint32_t)0x00003FF0)) == 0);
    ASSERT_ERR((((uint32_t)rxtype << 0) & ~((uint32_t)0x0000000F)) == 0);
    REG_BT_WR(BT_E_SCOTRCNTL_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE, ((uint32_t)txseqn << 31) | ((uint32_t)txlen << 20) | ((uint32_t)txtype << 16) | ((uint32_t)rxlen << 4) | ((uint32_t)rxtype << 0));
}

__INLINE void bt_e_scotrcntl_unpack(int elt_idx, uint8_t *txseqn, uint16_t *txlen, uint8_t *txtype, uint16_t *rxlen, uint8_t *rxtype)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOTRCNTL_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE);

    *txseqn = (localVal & ((uint32_t)0x80000000)) >> 31;
    *txlen = (localVal & ((uint32_t)0x3FF00000)) >> 20;
    *txtype = (localVal & ((uint32_t)0x000F0000)) >> 16;
    *rxlen = (localVal & ((uint32_t)0x00003FF0)) >> 4;
    *rxtype = (localVal & ((uint32_t)0x0000000F)) >> 0;
}

__INLINE uint8_t bt_e_scotrcntl_txseqn_getf(int elt_idx)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOTRCNTL_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE void bt_e_scotrcntl_txseqn_setf(int elt_idx, uint8_t txseqn)
{
    ASSERT_ERR((((uint32_t)txseqn << 31) & ~((uint32_t)0x80000000)) == 0);
    REG_BT_WR(BT_E_SCOTRCNTL_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE, (REG_BT_RD(BT_E_SCOTRCNTL_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE) & ~((uint32_t)0x80000000)) | ((uint32_t)txseqn << 31));
}

__INLINE uint16_t bt_e_scotrcntl_txlen_getf(int elt_idx)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOTRCNTL_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE);
    return ((localVal & ((uint32_t)0x3FF00000)) >> 20);
}

__INLINE void bt_e_scotrcntl_txlen_setf(int elt_idx, uint16_t txlen)
{
    ASSERT_ERR((((uint32_t)txlen << 20) & ~((uint32_t)0x3FF00000)) == 0);
    REG_BT_WR(BT_E_SCOTRCNTL_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE, (REG_BT_RD(BT_E_SCOTRCNTL_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE) & ~((uint32_t)0x3FF00000)) | ((uint32_t)txlen << 20));
}

__INLINE uint8_t bt_e_scotrcntl_txtype_getf(int elt_idx)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOTRCNTL_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE);
    return ((localVal & ((uint32_t)0x000F0000)) >> 16);
}

__INLINE void bt_e_scotrcntl_txtype_setf(int elt_idx, uint8_t txtype)
{
    ASSERT_ERR((((uint32_t)txtype << 16) & ~((uint32_t)0x000F0000)) == 0);
    REG_BT_WR(BT_E_SCOTRCNTL_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE, (REG_BT_RD(BT_E_SCOTRCNTL_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE) & ~((uint32_t)0x000F0000)) | ((uint32_t)txtype << 16));
}

__INLINE uint16_t bt_e_scotrcntl_rxlen_getf(int elt_idx)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOTRCNTL_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE);
    return ((localVal & ((uint32_t)0x00003FF0)) >> 4);
}

__INLINE void bt_e_scotrcntl_rxlen_setf(int elt_idx, uint16_t rxlen)
{
    ASSERT_ERR((((uint32_t)rxlen << 4) & ~((uint32_t)0x00003FF0)) == 0);
    REG_BT_WR(BT_E_SCOTRCNTL_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE, (REG_BT_RD(BT_E_SCOTRCNTL_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE) & ~((uint32_t)0x00003FF0)) | ((uint32_t)rxlen << 4));
}

__INLINE uint8_t bt_e_scotrcntl_rxtype_getf(int elt_idx)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOTRCNTL_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE);
    return ((localVal & ((uint32_t)0x0000000F)) >> 0);
}

__INLINE void bt_e_scotrcntl_rxtype_setf(int elt_idx, uint8_t rxtype)
{
    ASSERT_ERR((((uint32_t)rxtype << 0) & ~((uint32_t)0x0000000F)) == 0);
    REG_BT_WR(BT_E_SCOTRCNTL_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE, (REG_BT_RD(BT_E_SCOTRCNTL_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE) & ~((uint32_t)0x0000000F)) | ((uint32_t)rxtype << 0));
}

/**
 * @brief E_SCODAYCNT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  10:00           DAYCOUNTER   0x0
 * </pre>
 */
#define BT_E_SCODAYCNT_ADDR   0x40090628
#define BT_E_SCODAYCNT_OFFSET 0x00000228
#define BT_E_SCODAYCNT_INDEX  0x0000008A
#define BT_E_SCODAYCNT_RESET  0x00000000

__INLINE uint32_t bt_e_scodaycnt_get(int elt_idx)
{
    return REG_BT_RD(BT_E_SCODAYCNT_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE);
}

__INLINE void bt_e_scodaycnt_set(int elt_idx, uint32_t value)
{
    REG_BT_WR(BT_E_SCODAYCNT_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE, value);
}

// field definitions
#define BT_DAYCOUNTER_MASK   ((uint32_t)0x000007FF)
#define BT_DAYCOUNTER_LSB    0
#define BT_DAYCOUNTER_WIDTH  ((uint32_t)0x0000000B)

#define BT_DAYCOUNTER_RST    0x0

__INLINE uint16_t bt_e_scodaycnt_daycounter_getf(int elt_idx)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCODAYCNT_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE);
    ASSERT_ERR((localVal & ~((uint32_t)0x000007FF)) == 0);
    return (localVal >> 0);
}

__INLINE void bt_e_scodaycnt_daycounter_setf(int elt_idx, uint16_t daycounter)
{
    ASSERT_ERR((((uint32_t)daycounter << 0) & ~((uint32_t)0x000007FF)) == 0);
    REG_BT_WR(BT_E_SCODAYCNT_ADDR + elt_idx * REG_BTCORE_ESCO_SIZE, (uint32_t)daycounter << 0);
}


#endif // _REG_BTCORE_ESCO_H_

