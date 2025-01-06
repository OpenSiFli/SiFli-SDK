#ifndef _REG_EM_BLE_TX_DESC_H_
#define _REG_EM_BLE_TX_DESC_H_

#include <stdint.h>
#include "_reg_em_ble_tx_desc.h"
#include "compiler.h"
#include "arch.h"
#include "em_map.h"
#include "reg_access.h"

#define REG_EM_BLE_TX_DESC_COUNT 7

#define REG_EM_BLE_TX_DESC_DECODING_MASK 0x0000000F

#define REG_EM_BLE_TX_DESC_ADDR_GET(idx) (EM_BLE_TX_DESC_OFFSET + (idx) * REG_EM_BLE_TX_DESC_SIZE)

/**
 * @brief TXCNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     15               TXDONE   0
 *  14:00              NEXTPTR   0x0
 * </pre>
 */
#define EM_BLE_TXCNTL_ADDR   (0x40110000 + EM_BLE_TX_DESC_OFFSET)
#define EM_BLE_TXCNTL_INDEX  0x00000000
#define EM_BLE_TXCNTL_RESET  0x00000000

__INLINE uint16_t em_ble_txcntl_get(int elt_idx)
{
    return EM_BLE_RD(EM_BLE_TXCNTL_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE);
}

__INLINE void em_ble_txcntl_set(int elt_idx, uint16_t value)
{
    EM_BLE_WR(EM_BLE_TXCNTL_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE, value);
}

// field definitions
#define EM_BLE_TXDONE_BIT     ((uint16_t)0x00008000)
#define EM_BLE_TXDONE_POS     15
#define EM_BLE_NEXTPTR_MASK   ((uint16_t)0x00007FFF)
#define EM_BLE_NEXTPTR_LSB    0
#define EM_BLE_NEXTPTR_WIDTH  ((uint16_t)0x0000000F)

#define EM_BLE_TXDONE_RST     0x0
#define EM_BLE_NEXTPTR_RST    0x0

__INLINE void em_ble_txcntl_pack(int elt_idx, uint8_t txdone, uint16_t nextptr)
{
    ASSERT_ERR((((uint16_t)txdone << 15) & ~((uint16_t)0x00008000)) == 0);
    ASSERT_ERR((((uint16_t)nextptr << 0) & ~((uint16_t)0x00007FFF)) == 0);
    EM_BLE_WR(EM_BLE_TXCNTL_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE, ((uint16_t)txdone << 15) | ((uint16_t)nextptr << 0));
}

__INLINE void em_ble_txcntl_unpack(int elt_idx, uint8_t *txdone, uint16_t *nextptr)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_TXCNTL_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE);

    *txdone = (localVal & ((uint16_t)0x00008000)) >> 15;
    *nextptr = (localVal & ((uint16_t)0x00007FFF)) >> 0;
}

__INLINE uint8_t em_ble_txcntl_txdone_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_TXCNTL_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE);
    return ((localVal & ((uint16_t)0x00008000)) >> 15);
}

__INLINE void em_ble_txcntl_txdone_setf(int elt_idx, uint8_t txdone)
{
    ASSERT_ERR((((uint16_t)txdone << 15) & ~((uint16_t)0x00008000)) == 0);
    EM_BLE_WR(EM_BLE_TXCNTL_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE, (EM_BLE_RD(EM_BLE_TXCNTL_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE) & ~((uint16_t)0x00008000)) | ((uint16_t)txdone << 15));
}

__INLINE uint16_t em_ble_txcntl_nextptr_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_TXCNTL_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE);
    return ((localVal & ((uint16_t)0x00007FFF)) >> 0);
}

__INLINE void em_ble_txcntl_nextptr_setf(int elt_idx, uint16_t nextptr)
{
    ASSERT_ERR((((uint16_t)nextptr << 0) & ~((uint16_t)0x00007FFF)) == 0);
    EM_BLE_WR(EM_BLE_TXCNTL_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE, (EM_BLE_RD(EM_BLE_TXCNTL_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE) & ~((uint16_t)0x00007FFF)) | ((uint16_t)nextptr << 0));
}

/**
 * @brief TXPHCE register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:08                TXLEN   0x0
 *  07:05             TXACLRFU   0x0
 *     04                 TXMD   0
 *     03                 TXSN   0
 *     02               TXNESN   0
 *  01:00               TXLLID   0x0
 * </pre>
 */
#define EM_BLE_TXPHCE_ADDR   (0x40110002 + EM_BLE_TX_DESC_OFFSET)
#define EM_BLE_TXPHCE_INDEX  0x00000001
#define EM_BLE_TXPHCE_RESET  0x00000000

__INLINE uint16_t em_ble_txphce_get(int elt_idx)
{
    return EM_BLE_RD(EM_BLE_TXPHCE_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE);
}

__INLINE void em_ble_txphce_set(int elt_idx, uint16_t value)
{
    EM_BLE_WR(EM_BLE_TXPHCE_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE, value);
}

// field definitions
#define EM_BLE_TXLEN_MASK      ((uint16_t)0x0000FF00)
#define EM_BLE_TXLEN_LSB       8
#define EM_BLE_TXLEN_WIDTH     ((uint16_t)0x00000008)
#define EM_BLE_TXACLRFU_MASK   ((uint16_t)0x000000E0)
#define EM_BLE_TXACLRFU_LSB    5
#define EM_BLE_TXACLRFU_WIDTH  ((uint16_t)0x00000003)
#define EM_BLE_TXMD_BIT        ((uint16_t)0x00000010)
#define EM_BLE_TXMD_POS        4
#define EM_BLE_TXSN_BIT        ((uint16_t)0x00000008)
#define EM_BLE_TXSN_POS        3
#define EM_BLE_TXNESN_BIT      ((uint16_t)0x00000004)
#define EM_BLE_TXNESN_POS      2
#define EM_BLE_TXLLID_MASK     ((uint16_t)0x00000003)
#define EM_BLE_TXLLID_LSB      0
#define EM_BLE_TXLLID_WIDTH    ((uint16_t)0x00000002)

#define EM_BLE_TXLEN_RST       0x0
#define EM_BLE_TXACLRFU_RST    0x0
#define EM_BLE_TXMD_RST        0x0
#define EM_BLE_TXSN_RST        0x0
#define EM_BLE_TXNESN_RST      0x0
#define EM_BLE_TXLLID_RST      0x0

__INLINE void em_ble_txphce_pack(int elt_idx, uint8_t txlen, uint8_t txaclrfu, uint8_t txmd, uint8_t txsn, uint8_t txnesn, uint8_t txllid)
{
    ASSERT_ERR((((uint16_t)txlen << 8) & ~((uint16_t)0x0000FF00)) == 0);
    ASSERT_ERR((((uint16_t)txaclrfu << 5) & ~((uint16_t)0x000000E0)) == 0);
    ASSERT_ERR((((uint16_t)txmd << 4) & ~((uint16_t)0x00000010)) == 0);
    ASSERT_ERR((((uint16_t)txsn << 3) & ~((uint16_t)0x00000008)) == 0);
    ASSERT_ERR((((uint16_t)txnesn << 2) & ~((uint16_t)0x00000004)) == 0);
    ASSERT_ERR((((uint16_t)txllid << 0) & ~((uint16_t)0x00000003)) == 0);
    EM_BLE_WR(EM_BLE_TXPHCE_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE, ((uint16_t)txlen << 8) | ((uint16_t)txaclrfu << 5) | ((uint16_t)txmd << 4) | ((uint16_t)txsn << 3) | ((uint16_t)txnesn << 2) | ((uint16_t)txllid << 0));
}

__INLINE void em_ble_txphce_unpack(int elt_idx, uint8_t *txlen, uint8_t *txaclrfu, uint8_t *txmd, uint8_t *txsn, uint8_t *txnesn, uint8_t *txllid)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_TXPHCE_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE);

    *txlen = (localVal & ((uint16_t)0x0000FF00)) >> 8;
    *txaclrfu = (localVal & ((uint16_t)0x000000E0)) >> 5;
    *txmd = (localVal & ((uint16_t)0x00000010)) >> 4;
    *txsn = (localVal & ((uint16_t)0x00000008)) >> 3;
    *txnesn = (localVal & ((uint16_t)0x00000004)) >> 2;
    *txllid = (localVal & ((uint16_t)0x00000003)) >> 0;
}

__INLINE uint8_t em_ble_txphce_txlen_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_TXPHCE_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE);
    return ((localVal & ((uint16_t)0x0000FF00)) >> 8);
}

__INLINE void em_ble_txphce_txlen_setf(int elt_idx, uint8_t txlen)
{
    ASSERT_ERR((((uint16_t)txlen << 8) & ~((uint16_t)0x0000FF00)) == 0);
    EM_BLE_WR(EM_BLE_TXPHCE_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE, (EM_BLE_RD(EM_BLE_TXPHCE_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE) & ~((uint16_t)0x0000FF00)) | ((uint16_t)txlen << 8));
}

__INLINE uint8_t em_ble_txphce_txaclrfu_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_TXPHCE_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE);
    return ((localVal & ((uint16_t)0x000000E0)) >> 5);
}

__INLINE void em_ble_txphce_txaclrfu_setf(int elt_idx, uint8_t txaclrfu)
{
    ASSERT_ERR((((uint16_t)txaclrfu << 5) & ~((uint16_t)0x000000E0)) == 0);
    EM_BLE_WR(EM_BLE_TXPHCE_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE, (EM_BLE_RD(EM_BLE_TXPHCE_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE) & ~((uint16_t)0x000000E0)) | ((uint16_t)txaclrfu << 5));
}

__INLINE uint8_t em_ble_txphce_txmd_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_TXPHCE_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE);
    return ((localVal & ((uint16_t)0x00000010)) >> 4);
}

__INLINE void em_ble_txphce_txmd_setf(int elt_idx, uint8_t txmd)
{
    ASSERT_ERR((((uint16_t)txmd << 4) & ~((uint16_t)0x00000010)) == 0);
    EM_BLE_WR(EM_BLE_TXPHCE_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE, (EM_BLE_RD(EM_BLE_TXPHCE_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE) & ~((uint16_t)0x00000010)) | ((uint16_t)txmd << 4));
}

__INLINE uint8_t em_ble_txphce_txsn_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_TXPHCE_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE);
    return ((localVal & ((uint16_t)0x00000008)) >> 3);
}

__INLINE void em_ble_txphce_txsn_setf(int elt_idx, uint8_t txsn)
{
    ASSERT_ERR((((uint16_t)txsn << 3) & ~((uint16_t)0x00000008)) == 0);
    EM_BLE_WR(EM_BLE_TXPHCE_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE, (EM_BLE_RD(EM_BLE_TXPHCE_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE) & ~((uint16_t)0x00000008)) | ((uint16_t)txsn << 3));
}

__INLINE uint8_t em_ble_txphce_txnesn_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_TXPHCE_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE);
    return ((localVal & ((uint16_t)0x00000004)) >> 2);
}

__INLINE void em_ble_txphce_txnesn_setf(int elt_idx, uint8_t txnesn)
{
    ASSERT_ERR((((uint16_t)txnesn << 2) & ~((uint16_t)0x00000004)) == 0);
    EM_BLE_WR(EM_BLE_TXPHCE_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE, (EM_BLE_RD(EM_BLE_TXPHCE_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE) & ~((uint16_t)0x00000004)) | ((uint16_t)txnesn << 2));
}

__INLINE uint8_t em_ble_txphce_txllid_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_TXPHCE_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE);
    return ((localVal & ((uint16_t)0x00000003)) >> 0);
}

__INLINE void em_ble_txphce_txllid_setf(int elt_idx, uint8_t txllid)
{
    ASSERT_ERR((((uint16_t)txllid << 0) & ~((uint16_t)0x00000003)) == 0);
    EM_BLE_WR(EM_BLE_TXPHCE_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE, (EM_BLE_RD(EM_BLE_TXPHCE_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE) & ~((uint16_t)0x00000003)) | ((uint16_t)txllid << 0));
}

/**
 * @brief TXPHADV register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:08             TXADVLEN   0x0
 *     07              TXRXADD   0
 *     06              TXTXADD   0
 *     05             TXCHSEL2   0
 *     04             TXADVRFU   0
 *  03:00               TXTYPE   0x0
 * </pre>
 */
#define EM_BLE_TXPHADV_ADDR   (0x40110002 + EM_BLE_TX_DESC_OFFSET)
#define EM_BLE_TXPHADV_INDEX  0x00000001
#define EM_BLE_TXPHADV_RESET  0x00000000

__INLINE uint16_t em_ble_txphadv_get(int elt_idx)
{
    return EM_BLE_RD(EM_BLE_TXPHADV_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE);
}

__INLINE void em_ble_txphadv_set(int elt_idx, uint16_t value)
{
    EM_BLE_WR(EM_BLE_TXPHADV_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE, value);
}

// field definitions
#define EM_BLE_TXADVLEN_MASK   ((uint16_t)0x0000FF00)
#define EM_BLE_TXADVLEN_LSB    8
#define EM_BLE_TXADVLEN_WIDTH  ((uint16_t)0x00000008)
#define EM_BLE_TXRXADD_BIT     ((uint16_t)0x00000080)
#define EM_BLE_TXRXADD_POS     7
#define EM_BLE_TXTXADD_BIT     ((uint16_t)0x00000040)
#define EM_BLE_TXTXADD_POS     6
#define EM_BLE_TXCHSEL2_BIT    ((uint16_t)0x00000020)
#define EM_BLE_TXCHSEL2_POS    5
#define EM_BLE_TXADVRFU_BIT    ((uint16_t)0x00000010)
#define EM_BLE_TXADVRFU_POS    4
#define EM_BLE_TXTYPE_MASK     ((uint16_t)0x0000000F)
#define EM_BLE_TXTYPE_LSB      0
#define EM_BLE_TXTYPE_WIDTH    ((uint16_t)0x00000004)

#define EM_BLE_TXADVLEN_RST    0x0
#define EM_BLE_TXRXADD_RST     0x0
#define EM_BLE_TXTXADD_RST     0x0
#define EM_BLE_TXCHSEL2_RST    0x0
#define EM_BLE_TXADVRFU_RST    0x0
#define EM_BLE_TXTYPE_RST      0x0

__INLINE void em_ble_txphadv_pack(int elt_idx, uint8_t txadvlen, uint8_t txrxadd, uint8_t txtxadd, uint8_t txchsel2, uint8_t txadvrfu, uint8_t txtype)
{
    ASSERT_ERR((((uint16_t)txadvlen << 8) & ~((uint16_t)0x0000FF00)) == 0);
    ASSERT_ERR((((uint16_t)txrxadd << 7) & ~((uint16_t)0x00000080)) == 0);
    ASSERT_ERR((((uint16_t)txtxadd << 6) & ~((uint16_t)0x00000040)) == 0);
    ASSERT_ERR((((uint16_t)txchsel2 << 5) & ~((uint16_t)0x00000020)) == 0);
    ASSERT_ERR((((uint16_t)txadvrfu << 4) & ~((uint16_t)0x00000010)) == 0);
    ASSERT_ERR((((uint16_t)txtype << 0) & ~((uint16_t)0x0000000F)) == 0);
    EM_BLE_WR(EM_BLE_TXPHADV_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE, ((uint16_t)txadvlen << 8) | ((uint16_t)txrxadd << 7) | ((uint16_t)txtxadd << 6) | ((uint16_t)txchsel2 << 5) | ((uint16_t)txadvrfu << 4) | ((uint16_t)txtype << 0));
}

__INLINE void em_ble_txphadv_unpack(int elt_idx, uint8_t *txadvlen, uint8_t *txrxadd, uint8_t *txtxadd, uint8_t *txchsel2, uint8_t *txadvrfu, uint8_t *txtype)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_TXPHADV_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE);

    *txadvlen = (localVal & ((uint16_t)0x0000FF00)) >> 8;
    *txrxadd = (localVal & ((uint16_t)0x00000080)) >> 7;
    *txtxadd = (localVal & ((uint16_t)0x00000040)) >> 6;
    *txchsel2 = (localVal & ((uint16_t)0x00000020)) >> 5;
    *txadvrfu = (localVal & ((uint16_t)0x00000010)) >> 4;
    *txtype = (localVal & ((uint16_t)0x0000000F)) >> 0;
}

__INLINE uint8_t em_ble_txphadv_txadvlen_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_TXPHADV_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE);
    return ((localVal & ((uint16_t)0x0000FF00)) >> 8);
}

__INLINE void em_ble_txphadv_txadvlen_setf(int elt_idx, uint8_t txadvlen)
{
    ASSERT_ERR((((uint16_t)txadvlen << 8) & ~((uint16_t)0x0000FF00)) == 0);
    EM_BLE_WR(EM_BLE_TXPHADV_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE, (EM_BLE_RD(EM_BLE_TXPHADV_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE) & ~((uint16_t)0x0000FF00)) | ((uint16_t)txadvlen << 8));
}

__INLINE uint8_t em_ble_txphadv_txrxadd_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_TXPHADV_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE);
    return ((localVal & ((uint16_t)0x00000080)) >> 7);
}

__INLINE void em_ble_txphadv_txrxadd_setf(int elt_idx, uint8_t txrxadd)
{
    ASSERT_ERR((((uint16_t)txrxadd << 7) & ~((uint16_t)0x00000080)) == 0);
    EM_BLE_WR(EM_BLE_TXPHADV_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE, (EM_BLE_RD(EM_BLE_TXPHADV_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE) & ~((uint16_t)0x00000080)) | ((uint16_t)txrxadd << 7));
}

__INLINE uint8_t em_ble_txphadv_txtxadd_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_TXPHADV_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE);
    return ((localVal & ((uint16_t)0x00000040)) >> 6);
}

__INLINE void em_ble_txphadv_txtxadd_setf(int elt_idx, uint8_t txtxadd)
{
    ASSERT_ERR((((uint16_t)txtxadd << 6) & ~((uint16_t)0x00000040)) == 0);
    EM_BLE_WR(EM_BLE_TXPHADV_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE, (EM_BLE_RD(EM_BLE_TXPHADV_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE) & ~((uint16_t)0x00000040)) | ((uint16_t)txtxadd << 6));
}

__INLINE uint8_t em_ble_txphadv_txchsel2_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_TXPHADV_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE);
    return ((localVal & ((uint16_t)0x00000020)) >> 5);
}

__INLINE void em_ble_txphadv_txchsel2_setf(int elt_idx, uint8_t txchsel2)
{
    ASSERT_ERR((((uint16_t)txchsel2 << 5) & ~((uint16_t)0x00000020)) == 0);
    EM_BLE_WR(EM_BLE_TXPHADV_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE, (EM_BLE_RD(EM_BLE_TXPHADV_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE) & ~((uint16_t)0x00000020)) | ((uint16_t)txchsel2 << 5));
}

__INLINE uint8_t em_ble_txphadv_txadvrfu_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_TXPHADV_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE);
    return ((localVal & ((uint16_t)0x00000010)) >> 4);
}

__INLINE void em_ble_txphadv_txadvrfu_setf(int elt_idx, uint8_t txadvrfu)
{
    ASSERT_ERR((((uint16_t)txadvrfu << 4) & ~((uint16_t)0x00000010)) == 0);
    EM_BLE_WR(EM_BLE_TXPHADV_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE, (EM_BLE_RD(EM_BLE_TXPHADV_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE) & ~((uint16_t)0x00000010)) | ((uint16_t)txadvrfu << 4));
}

__INLINE uint8_t em_ble_txphadv_txtype_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_TXPHADV_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE);
    return ((localVal & ((uint16_t)0x0000000F)) >> 0);
}

__INLINE void em_ble_txphadv_txtype_setf(int elt_idx, uint8_t txtype)
{
    ASSERT_ERR((((uint16_t)txtype << 0) & ~((uint16_t)0x0000000F)) == 0);
    EM_BLE_WR(EM_BLE_TXPHADV_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE, (EM_BLE_RD(EM_BLE_TXPHADV_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE) & ~((uint16_t)0x0000000F)) | ((uint16_t)txtype << 0));
}

/**
 * @brief TXDATAPTR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00            TXDATAPTR   0x0
 * </pre>
 */
#define EM_BLE_TXDATAPTR_ADDR   (0x40110004 + EM_BLE_TX_DESC_OFFSET)
#define EM_BLE_TXDATAPTR_INDEX  0x00000002
#define EM_BLE_TXDATAPTR_RESET  0x00000000

__INLINE uint16_t em_ble_txdataptr_get(int elt_idx)
{
    return EM_BLE_RD(EM_BLE_TXDATAPTR_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE);
}

__INLINE void em_ble_txdataptr_set(int elt_idx, uint16_t value)
{
    EM_BLE_WR(EM_BLE_TXDATAPTR_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE, value);
}

// field definitions
#define EM_BLE_TXDATAPTR_MASK   ((uint16_t)0x0000FFFF)
#define EM_BLE_TXDATAPTR_LSB    0
#define EM_BLE_TXDATAPTR_WIDTH  ((uint16_t)0x00000010)

#define EM_BLE_TXDATAPTR_RST    0x0

__INLINE uint16_t em_ble_txdataptr_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_TXDATAPTR_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE);
    ASSERT_ERR((localVal & ~((uint16_t)0x0000FFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void em_ble_txdataptr_setf(int elt_idx, uint16_t txdataptr)
{
    ASSERT_ERR((((uint16_t)txdataptr << 0) & ~((uint16_t)0x0000FFFF)) == 0);
    EM_BLE_WR(EM_BLE_TXDATAPTR_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE, (uint16_t)txdataptr << 0);
}

/**
 * @brief TXAEHEADER register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     15               TXRSVD   0
 *     14                TXPOW   0
 *     13               TXSYNC   0
 *     12             TXAUXPTR   0
 *     11                TXADI   0
 *     10               TXSUPP   0
 *     09               TXTGTA   0
 *     08               TXADVA   0
 *  07:06             TXAEMODE   0x0
 *  05:00           TXAELENGTH   0x0
 * </pre>
 */
#define EM_BLE_TXAEHEADER_ADDR   (0x40110006 + EM_BLE_TX_DESC_OFFSET)
#define EM_BLE_TXAEHEADER_INDEX  0x00000003
#define EM_BLE_TXAEHEADER_RESET  0x00000000

__INLINE uint16_t em_ble_txaeheader_get(int elt_idx)
{
    return EM_BLE_RD(EM_BLE_TXAEHEADER_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE);
}

__INLINE void em_ble_txaeheader_set(int elt_idx, uint16_t value)
{
    EM_BLE_WR(EM_BLE_TXAEHEADER_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE, value);
}

// field definitions
#define EM_BLE_TXRSVD_BIT        ((uint16_t)0x00008000)
#define EM_BLE_TXRSVD_POS        15
#define EM_BLE_TXPOW_BIT         ((uint16_t)0x00004000)
#define EM_BLE_TXPOW_POS         14
#define EM_BLE_TXSYNC_BIT        ((uint16_t)0x00002000)
#define EM_BLE_TXSYNC_POS        13
#define EM_BLE_TXAUXPTR_BIT      ((uint16_t)0x00001000)
#define EM_BLE_TXAUXPTR_POS      12
#define EM_BLE_TXADI_BIT         ((uint16_t)0x00000800)
#define EM_BLE_TXADI_POS         11
#define EM_BLE_TXSUPP_BIT        ((uint16_t)0x00000400)
#define EM_BLE_TXSUPP_POS        10
#define EM_BLE_TXTGTA_BIT        ((uint16_t)0x00000200)
#define EM_BLE_TXTGTA_POS        9
#define EM_BLE_TXADVA_BIT        ((uint16_t)0x00000100)
#define EM_BLE_TXADVA_POS        8
#define EM_BLE_TXAEMODE_MASK     ((uint16_t)0x000000C0)
#define EM_BLE_TXAEMODE_LSB      6
#define EM_BLE_TXAEMODE_WIDTH    ((uint16_t)0x00000002)
#define EM_BLE_TXAELENGTH_MASK   ((uint16_t)0x0000003F)
#define EM_BLE_TXAELENGTH_LSB    0
#define EM_BLE_TXAELENGTH_WIDTH  ((uint16_t)0x00000006)

#define EM_BLE_TXRSVD_RST        0x0
#define EM_BLE_TXPOW_RST         0x0
#define EM_BLE_TXSYNC_RST        0x0
#define EM_BLE_TXAUXPTR_RST      0x0
#define EM_BLE_TXADI_RST         0x0
#define EM_BLE_TXSUPP_RST        0x0
#define EM_BLE_TXTGTA_RST        0x0
#define EM_BLE_TXADVA_RST        0x0
#define EM_BLE_TXAEMODE_RST      0x0
#define EM_BLE_TXAELENGTH_RST    0x0

__INLINE void em_ble_txaeheader_pack(int elt_idx, uint8_t txrsvd, uint8_t txpow, uint8_t txsync, uint8_t txauxptr, uint8_t txadi, uint8_t txsupp, uint8_t txtgta, uint8_t txadva, uint8_t txaemode, uint8_t txaelength)
{
    ASSERT_ERR((((uint16_t)txrsvd << 15) & ~((uint16_t)0x00008000)) == 0);
    ASSERT_ERR((((uint16_t)txpow << 14) & ~((uint16_t)0x00004000)) == 0);
    ASSERT_ERR((((uint16_t)txsync << 13) & ~((uint16_t)0x00002000)) == 0);
    ASSERT_ERR((((uint16_t)txauxptr << 12) & ~((uint16_t)0x00001000)) == 0);
    ASSERT_ERR((((uint16_t)txadi << 11) & ~((uint16_t)0x00000800)) == 0);
    ASSERT_ERR((((uint16_t)txsupp << 10) & ~((uint16_t)0x00000400)) == 0);
    ASSERT_ERR((((uint16_t)txtgta << 9) & ~((uint16_t)0x00000200)) == 0);
    ASSERT_ERR((((uint16_t)txadva << 8) & ~((uint16_t)0x00000100)) == 0);
    ASSERT_ERR((((uint16_t)txaemode << 6) & ~((uint16_t)0x000000C0)) == 0);
    ASSERT_ERR((((uint16_t)txaelength << 0) & ~((uint16_t)0x0000003F)) == 0);
    EM_BLE_WR(EM_BLE_TXAEHEADER_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE, ((uint16_t)txrsvd << 15) | ((uint16_t)txpow << 14) | ((uint16_t)txsync << 13) | ((uint16_t)txauxptr << 12) | ((uint16_t)txadi << 11) | ((uint16_t)txsupp << 10) | ((uint16_t)txtgta << 9) | ((uint16_t)txadva << 8) | ((uint16_t)txaemode << 6) | ((uint16_t)txaelength << 0));
}

__INLINE void em_ble_txaeheader_unpack(int elt_idx, uint8_t *txrsvd, uint8_t *txpow, uint8_t *txsync, uint8_t *txauxptr, uint8_t *txadi, uint8_t *txsupp, uint8_t *txtgta, uint8_t *txadva, uint8_t *txaemode, uint8_t *txaelength)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_TXAEHEADER_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE);

    *txrsvd = (localVal & ((uint16_t)0x00008000)) >> 15;
    *txpow = (localVal & ((uint16_t)0x00004000)) >> 14;
    *txsync = (localVal & ((uint16_t)0x00002000)) >> 13;
    *txauxptr = (localVal & ((uint16_t)0x00001000)) >> 12;
    *txadi = (localVal & ((uint16_t)0x00000800)) >> 11;
    *txsupp = (localVal & ((uint16_t)0x00000400)) >> 10;
    *txtgta = (localVal & ((uint16_t)0x00000200)) >> 9;
    *txadva = (localVal & ((uint16_t)0x00000100)) >> 8;
    *txaemode = (localVal & ((uint16_t)0x000000C0)) >> 6;
    *txaelength = (localVal & ((uint16_t)0x0000003F)) >> 0;
}

__INLINE uint8_t em_ble_txaeheader_txrsvd_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_TXAEHEADER_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE);
    return ((localVal & ((uint16_t)0x00008000)) >> 15);
}

__INLINE void em_ble_txaeheader_txrsvd_setf(int elt_idx, uint8_t txrsvd)
{
    ASSERT_ERR((((uint16_t)txrsvd << 15) & ~((uint16_t)0x00008000)) == 0);
    EM_BLE_WR(EM_BLE_TXAEHEADER_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE, (EM_BLE_RD(EM_BLE_TXAEHEADER_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE) & ~((uint16_t)0x00008000)) | ((uint16_t)txrsvd << 15));
}

__INLINE uint8_t em_ble_txaeheader_txpow_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_TXAEHEADER_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE);
    return ((localVal & ((uint16_t)0x00004000)) >> 14);
}

__INLINE void em_ble_txaeheader_txpow_setf(int elt_idx, uint8_t txpow)
{
    ASSERT_ERR((((uint16_t)txpow << 14) & ~((uint16_t)0x00004000)) == 0);
    EM_BLE_WR(EM_BLE_TXAEHEADER_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE, (EM_BLE_RD(EM_BLE_TXAEHEADER_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE) & ~((uint16_t)0x00004000)) | ((uint16_t)txpow << 14));
}

__INLINE uint8_t em_ble_txaeheader_txsync_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_TXAEHEADER_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE);
    return ((localVal & ((uint16_t)0x00002000)) >> 13);
}

__INLINE void em_ble_txaeheader_txsync_setf(int elt_idx, uint8_t txsync)
{
    ASSERT_ERR((((uint16_t)txsync << 13) & ~((uint16_t)0x00002000)) == 0);
    EM_BLE_WR(EM_BLE_TXAEHEADER_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE, (EM_BLE_RD(EM_BLE_TXAEHEADER_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE) & ~((uint16_t)0x00002000)) | ((uint16_t)txsync << 13));
}

__INLINE uint8_t em_ble_txaeheader_txauxptr_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_TXAEHEADER_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE);
    return ((localVal & ((uint16_t)0x00001000)) >> 12);
}

__INLINE void em_ble_txaeheader_txauxptr_setf(int elt_idx, uint8_t txauxptr)
{
    ASSERT_ERR((((uint16_t)txauxptr << 12) & ~((uint16_t)0x00001000)) == 0);
    EM_BLE_WR(EM_BLE_TXAEHEADER_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE, (EM_BLE_RD(EM_BLE_TXAEHEADER_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE) & ~((uint16_t)0x00001000)) | ((uint16_t)txauxptr << 12));
}

__INLINE uint8_t em_ble_txaeheader_txadi_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_TXAEHEADER_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE);
    return ((localVal & ((uint16_t)0x00000800)) >> 11);
}

__INLINE void em_ble_txaeheader_txadi_setf(int elt_idx, uint8_t txadi)
{
    ASSERT_ERR((((uint16_t)txadi << 11) & ~((uint16_t)0x00000800)) == 0);
    EM_BLE_WR(EM_BLE_TXAEHEADER_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE, (EM_BLE_RD(EM_BLE_TXAEHEADER_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE) & ~((uint16_t)0x00000800)) | ((uint16_t)txadi << 11));
}

__INLINE uint8_t em_ble_txaeheader_txsupp_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_TXAEHEADER_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE);
    return ((localVal & ((uint16_t)0x00000400)) >> 10);
}

__INLINE void em_ble_txaeheader_txsupp_setf(int elt_idx, uint8_t txsupp)
{
    ASSERT_ERR((((uint16_t)txsupp << 10) & ~((uint16_t)0x00000400)) == 0);
    EM_BLE_WR(EM_BLE_TXAEHEADER_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE, (EM_BLE_RD(EM_BLE_TXAEHEADER_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE) & ~((uint16_t)0x00000400)) | ((uint16_t)txsupp << 10));
}

__INLINE uint8_t em_ble_txaeheader_txtgta_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_TXAEHEADER_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE);
    return ((localVal & ((uint16_t)0x00000200)) >> 9);
}

__INLINE void em_ble_txaeheader_txtgta_setf(int elt_idx, uint8_t txtgta)
{
    ASSERT_ERR((((uint16_t)txtgta << 9) & ~((uint16_t)0x00000200)) == 0);
    EM_BLE_WR(EM_BLE_TXAEHEADER_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE, (EM_BLE_RD(EM_BLE_TXAEHEADER_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE) & ~((uint16_t)0x00000200)) | ((uint16_t)txtgta << 9));
}

__INLINE uint8_t em_ble_txaeheader_txadva_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_TXAEHEADER_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE);
    return ((localVal & ((uint16_t)0x00000100)) >> 8);
}

__INLINE void em_ble_txaeheader_txadva_setf(int elt_idx, uint8_t txadva)
{
    ASSERT_ERR((((uint16_t)txadva << 8) & ~((uint16_t)0x00000100)) == 0);
    EM_BLE_WR(EM_BLE_TXAEHEADER_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE, (EM_BLE_RD(EM_BLE_TXAEHEADER_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE) & ~((uint16_t)0x00000100)) | ((uint16_t)txadva << 8));
}

__INLINE uint8_t em_ble_txaeheader_txaemode_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_TXAEHEADER_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE);
    return ((localVal & ((uint16_t)0x000000C0)) >> 6);
}

__INLINE void em_ble_txaeheader_txaemode_setf(int elt_idx, uint8_t txaemode)
{
    ASSERT_ERR((((uint16_t)txaemode << 6) & ~((uint16_t)0x000000C0)) == 0);
    EM_BLE_WR(EM_BLE_TXAEHEADER_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE, (EM_BLE_RD(EM_BLE_TXAEHEADER_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE) & ~((uint16_t)0x000000C0)) | ((uint16_t)txaemode << 6));
}

__INLINE uint8_t em_ble_txaeheader_txaelength_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_TXAEHEADER_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE);
    return ((localVal & ((uint16_t)0x0000003F)) >> 0);
}

__INLINE void em_ble_txaeheader_txaelength_setf(int elt_idx, uint8_t txaelength)
{
    ASSERT_ERR((((uint16_t)txaelength << 0) & ~((uint16_t)0x0000003F)) == 0);
    EM_BLE_WR(EM_BLE_TXAEHEADER_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE, (EM_BLE_RD(EM_BLE_TXAEHEADER_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE) & ~((uint16_t)0x0000003F)) | ((uint16_t)txaelength << 0));
}

/**
 * @brief TXAUXPTR0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:08      TXAUXOFFSET_LSB   0x0
 *     07     TXAUXOFFSET_UNIT   0
 *     06             TXAUX_CA   0
 *  05:00             TX_LL_CH   0x0
 * </pre>
 */
#define EM_BLE_TXAUXPTR0_ADDR   (0x40110008 + EM_BLE_TX_DESC_OFFSET)
#define EM_BLE_TXAUXPTR0_INDEX  0x00000004
#define EM_BLE_TXAUXPTR0_RESET  0x00000000

__INLINE uint16_t em_ble_txauxptr0_get(int elt_idx)
{
    return EM_BLE_RD(EM_BLE_TXAUXPTR0_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE);
}

__INLINE void em_ble_txauxptr0_set(int elt_idx, uint16_t value)
{
    EM_BLE_WR(EM_BLE_TXAUXPTR0_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE, value);
}

// field definitions
#define EM_BLE_TXAUXOFFSET_LSB_MASK    ((uint16_t)0x0000FF00)
#define EM_BLE_TXAUXOFFSET_LSB_LSB     8
#define EM_BLE_TXAUXOFFSET_LSB_WIDTH   ((uint16_t)0x00000008)
#define EM_BLE_TXAUXOFFSET_UNIT_BIT    ((uint16_t)0x00000080)
#define EM_BLE_TXAUXOFFSET_UNIT_POS    7
#define EM_BLE_TXAUX_CA_BIT            ((uint16_t)0x00000040)
#define EM_BLE_TXAUX_CA_POS            6
#define EM_BLE_TX_LL_CH_MASK           ((uint16_t)0x0000003F)
#define EM_BLE_TX_LL_CH_LSB            0
#define EM_BLE_TX_LL_CH_WIDTH          ((uint16_t)0x00000006)

#define EM_BLE_TXAUXOFFSET_LSB_RST     0x0
#define EM_BLE_TXAUXOFFSET_UNIT_RST    0x0
#define EM_BLE_TXAUX_CA_RST            0x0
#define EM_BLE_TX_LL_CH_RST            0x0

__INLINE void em_ble_txauxptr0_pack(int elt_idx, uint8_t txauxoffsetlsb, uint8_t txauxoffsetunit, uint8_t txauxca, uint8_t txllch)
{
    ASSERT_ERR((((uint16_t)txauxoffsetlsb << 8) & ~((uint16_t)0x0000FF00)) == 0);
    ASSERT_ERR((((uint16_t)txauxoffsetunit << 7) & ~((uint16_t)0x00000080)) == 0);
    ASSERT_ERR((((uint16_t)txauxca << 6) & ~((uint16_t)0x00000040)) == 0);
    ASSERT_ERR((((uint16_t)txllch << 0) & ~((uint16_t)0x0000003F)) == 0);
    EM_BLE_WR(EM_BLE_TXAUXPTR0_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE, ((uint16_t)txauxoffsetlsb << 8) | ((uint16_t)txauxoffsetunit << 7) | ((uint16_t)txauxca << 6) | ((uint16_t)txllch << 0));
}

__INLINE void em_ble_txauxptr0_unpack(int elt_idx, uint8_t *txauxoffsetlsb, uint8_t *txauxoffsetunit, uint8_t *txauxca, uint8_t *txllch)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_TXAUXPTR0_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE);

    *txauxoffsetlsb = (localVal & ((uint16_t)0x0000FF00)) >> 8;
    *txauxoffsetunit = (localVal & ((uint16_t)0x00000080)) >> 7;
    *txauxca = (localVal & ((uint16_t)0x00000040)) >> 6;
    *txllch = (localVal & ((uint16_t)0x0000003F)) >> 0;
}

__INLINE uint8_t em_ble_txauxptr0_txauxoffset_lsb_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_TXAUXPTR0_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE);
    return ((localVal & ((uint16_t)0x0000FF00)) >> 8);
}

__INLINE void em_ble_txauxptr0_txauxoffset_lsb_setf(int elt_idx, uint8_t txauxoffsetlsb)
{
    ASSERT_ERR((((uint16_t)txauxoffsetlsb << 8) & ~((uint16_t)0x0000FF00)) == 0);
    EM_BLE_WR(EM_BLE_TXAUXPTR0_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE, (EM_BLE_RD(EM_BLE_TXAUXPTR0_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE) & ~((uint16_t)0x0000FF00)) | ((uint16_t)txauxoffsetlsb << 8));
}

__INLINE uint8_t em_ble_txauxptr0_txauxoffset_unit_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_TXAUXPTR0_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE);
    return ((localVal & ((uint16_t)0x00000080)) >> 7);
}

__INLINE void em_ble_txauxptr0_txauxoffset_unit_setf(int elt_idx, uint8_t txauxoffsetunit)
{
    ASSERT_ERR((((uint16_t)txauxoffsetunit << 7) & ~((uint16_t)0x00000080)) == 0);
    EM_BLE_WR(EM_BLE_TXAUXPTR0_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE, (EM_BLE_RD(EM_BLE_TXAUXPTR0_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE) & ~((uint16_t)0x00000080)) | ((uint16_t)txauxoffsetunit << 7));
}

__INLINE uint8_t em_ble_txauxptr0_txaux_ca_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_TXAUXPTR0_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE);
    return ((localVal & ((uint16_t)0x00000040)) >> 6);
}

__INLINE void em_ble_txauxptr0_txaux_ca_setf(int elt_idx, uint8_t txauxca)
{
    ASSERT_ERR((((uint16_t)txauxca << 6) & ~((uint16_t)0x00000040)) == 0);
    EM_BLE_WR(EM_BLE_TXAUXPTR0_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE, (EM_BLE_RD(EM_BLE_TXAUXPTR0_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE) & ~((uint16_t)0x00000040)) | ((uint16_t)txauxca << 6));
}

__INLINE uint8_t em_ble_txauxptr0_tx_ll_ch_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_TXAUXPTR0_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE);
    return ((localVal & ((uint16_t)0x0000003F)) >> 0);
}

__INLINE void em_ble_txauxptr0_tx_ll_ch_setf(int elt_idx, uint8_t txllch)
{
    ASSERT_ERR((((uint16_t)txllch << 0) & ~((uint16_t)0x0000003F)) == 0);
    EM_BLE_WR(EM_BLE_TXAUXPTR0_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE, (EM_BLE_RD(EM_BLE_TXAUXPTR0_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE) & ~((uint16_t)0x0000003F)) | ((uint16_t)txllch << 0));
}

/**
 * @brief TXAUXPTR1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  07:05            TXAUX_PHY   0x0
 *  04:00      TXAUXOFFSET_MSB   0x0
 * </pre>
 */
#define EM_BLE_TXAUXPTR1_ADDR   (0x4011000A + EM_BLE_TX_DESC_OFFSET)
#define EM_BLE_TXAUXPTR1_INDEX  0x00000005
#define EM_BLE_TXAUXPTR1_RESET  0x00000000

__INLINE uint16_t em_ble_txauxptr1_get(int elt_idx)
{
    return EM_BLE_RD(EM_BLE_TXAUXPTR1_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE);
}

__INLINE void em_ble_txauxptr1_set(int elt_idx, uint16_t value)
{
    EM_BLE_WR(EM_BLE_TXAUXPTR1_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE, value);
}

// field definitions
#define EM_BLE_TXAUX_PHY_MASK         ((uint16_t)0x000000E0)
#define EM_BLE_TXAUX_PHY_LSB          5
#define EM_BLE_TXAUX_PHY_WIDTH        ((uint16_t)0x00000003)
#define EM_BLE_TXAUXOFFSET_MSB_MASK   ((uint16_t)0x0000001F)
#define EM_BLE_TXAUXOFFSET_MSB_LSB    0
#define EM_BLE_TXAUXOFFSET_MSB_WIDTH  ((uint16_t)0x00000005)

#define EM_BLE_TXAUX_PHY_RST          0x0
#define EM_BLE_TXAUXOFFSET_MSB_RST    0x0

__INLINE void em_ble_txauxptr1_pack(int elt_idx, uint8_t txauxphy, uint8_t txauxoffsetmsb)
{
    ASSERT_ERR((((uint16_t)txauxphy << 5) & ~((uint16_t)0x000000E0)) == 0);
    ASSERT_ERR((((uint16_t)txauxoffsetmsb << 0) & ~((uint16_t)0x0000001F)) == 0);
    EM_BLE_WR(EM_BLE_TXAUXPTR1_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE, ((uint16_t)txauxphy << 5) | ((uint16_t)txauxoffsetmsb << 0));
}

__INLINE void em_ble_txauxptr1_unpack(int elt_idx, uint8_t *txauxphy, uint8_t *txauxoffsetmsb)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_TXAUXPTR1_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE);

    *txauxphy = (localVal & ((uint16_t)0x000000E0)) >> 5;
    *txauxoffsetmsb = (localVal & ((uint16_t)0x0000001F)) >> 0;
}

__INLINE uint8_t em_ble_txauxptr1_txaux_phy_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_TXAUXPTR1_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE);
    return ((localVal & ((uint16_t)0x000000E0)) >> 5);
}

__INLINE void em_ble_txauxptr1_txaux_phy_setf(int elt_idx, uint8_t txauxphy)
{
    ASSERT_ERR((((uint16_t)txauxphy << 5) & ~((uint16_t)0x000000E0)) == 0);
    EM_BLE_WR(EM_BLE_TXAUXPTR1_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE, (EM_BLE_RD(EM_BLE_TXAUXPTR1_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE) & ~((uint16_t)0x000000E0)) | ((uint16_t)txauxphy << 5));
}

__INLINE uint8_t em_ble_txauxptr1_txauxoffset_msb_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_TXAUXPTR1_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE);
    return ((localVal & ((uint16_t)0x0000001F)) >> 0);
}

__INLINE void em_ble_txauxptr1_txauxoffset_msb_setf(int elt_idx, uint8_t txauxoffsetmsb)
{
    ASSERT_ERR((((uint16_t)txauxoffsetmsb << 0) & ~((uint16_t)0x0000001F)) == 0);
    EM_BLE_WR(EM_BLE_TXAUXPTR1_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE, (EM_BLE_RD(EM_BLE_TXAUXPTR1_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE) & ~((uint16_t)0x0000001F)) | ((uint16_t)txauxoffsetmsb << 0));
}

/**
 * @brief TXAEDATAPTR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00          TXAEDATAPTR   0x0
 * </pre>
 */
#define EM_BLE_TXAEDATAPTR_ADDR   (0x4011000C + EM_BLE_TX_DESC_OFFSET)
#define EM_BLE_TXAEDATAPTR_INDEX  0x00000006
#define EM_BLE_TXAEDATAPTR_RESET  0x00000000

__INLINE uint16_t em_ble_txaedataptr_get(int elt_idx)
{
    return EM_BLE_RD(EM_BLE_TXAEDATAPTR_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE);
}

__INLINE void em_ble_txaedataptr_set(int elt_idx, uint16_t value)
{
    EM_BLE_WR(EM_BLE_TXAEDATAPTR_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE, value);
}

// field definitions
#define EM_BLE_TXAEDATAPTR_MASK   ((uint16_t)0x0000FFFF)
#define EM_BLE_TXAEDATAPTR_LSB    0
#define EM_BLE_TXAEDATAPTR_WIDTH  ((uint16_t)0x00000010)

#define EM_BLE_TXAEDATAPTR_RST    0x0

__INLINE uint16_t em_ble_txaedataptr_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_TXAEDATAPTR_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE);
    ASSERT_ERR((localVal & ~((uint16_t)0x0000FFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void em_ble_txaedataptr_setf(int elt_idx, uint16_t txaedataptr)
{
    ASSERT_ERR((((uint16_t)txaedataptr << 0) & ~((uint16_t)0x0000FFFF)) == 0);
    EM_BLE_WR(EM_BLE_TXAEDATAPTR_ADDR + elt_idx * REG_EM_BLE_TX_DESC_SIZE, (uint16_t)txaedataptr << 0);
}


#endif // _REG_EM_BLE_TX_DESC_H_

