#ifndef _REG_EM_BLE_RX_ISO_DESC_H_
#define _REG_EM_BLE_RX_ISO_DESC_H_

#include <stdint.h>
#include "_reg_em_ble_rx_iso_desc.h"
#include "compiler.h"
#include "arch.h"
#include "em_map.h"
#include "reg_access.h"

#define REG_EM_BLE_RX_ISO_DESC_COUNT 6

#define REG_EM_BLE_RX_ISO_DESC_DECODING_MASK 0x0000000F

#define REG_EM_BLE_RX_ISO_DESC_ADDR_GET(idx) (EM_BLE_RX_ISO_DESC_OFFSET + (idx) * REG_EM_BLE_RX_ISO_DESC_SIZE)

/**
 * @brief RXISOPTR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     15               RXDONE   0
 *  13:00              NEXTPTR   0x0
 * </pre>
 */
#define EM_BLE_RXISOPTR_ADDR   (0x20418000 + EM_BLE_RX_ISO_DESC_OFFSET)
#define EM_BLE_RXISOPTR_INDEX  0x00000000
#define EM_BLE_RXISOPTR_RESET  0x00000000

__INLINE uint16_t em_ble_rxisoptr_get(int elt_idx)
{
    return EM_BLE_RD(EM_BLE_RXISOPTR_ADDR + elt_idx * REG_EM_BLE_RX_ISO_DESC_SIZE);
}

__INLINE void em_ble_rxisoptr_set(int elt_idx, uint16_t value)
{
    EM_BLE_WR(EM_BLE_RXISOPTR_ADDR + elt_idx * REG_EM_BLE_RX_ISO_DESC_SIZE, value);
}

// field definitions
#define EM_BLE_RXDONE_BIT     ((uint16_t)0x00008000)
#define EM_BLE_RXDONE_POS     15
#define EM_BLE_NEXTPTR_MASK   ((uint16_t)0x00003FFF)
#define EM_BLE_NEXTPTR_LSB    0
#define EM_BLE_NEXTPTR_WIDTH  ((uint16_t)0x0000000E)

#define EM_BLE_RXDONE_RST     0x0
#define EM_BLE_NEXTPTR_RST    0x0

__INLINE void em_ble_rxisoptr_pack(int elt_idx, uint8_t rxdone, uint16_t nextptr)
{
    ASSERT_ERR((((uint16_t)rxdone << 15) & ~((uint16_t)0x00008000)) == 0);
    ASSERT_ERR((((uint16_t)nextptr << 0) & ~((uint16_t)0x00003FFF)) == 0);
    EM_BLE_WR(EM_BLE_RXISOPTR_ADDR + elt_idx * REG_EM_BLE_RX_ISO_DESC_SIZE, ((uint16_t)rxdone << 15) | ((uint16_t)nextptr << 0));
}

__INLINE void em_ble_rxisoptr_unpack(int elt_idx, uint8_t *rxdone, uint16_t *nextptr)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXISOPTR_ADDR + elt_idx * REG_EM_BLE_RX_ISO_DESC_SIZE);

    *rxdone = (localVal & ((uint16_t)0x00008000)) >> 15;
    *nextptr = (localVal & ((uint16_t)0x00003FFF)) >> 0;
}

__INLINE uint8_t em_ble_rxisoptr_rxdone_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXISOPTR_ADDR + elt_idx * REG_EM_BLE_RX_ISO_DESC_SIZE);
    return ((localVal & ((uint16_t)0x00008000)) >> 15);
}

__INLINE void em_ble_rxisoptr_rxdone_setf(int elt_idx, uint8_t rxdone)
{
    ASSERT_ERR((((uint16_t)rxdone << 15) & ~((uint16_t)0x00008000)) == 0);
    EM_BLE_WR(EM_BLE_RXISOPTR_ADDR + elt_idx * REG_EM_BLE_RX_ISO_DESC_SIZE, (EM_BLE_RD(EM_BLE_RXISOPTR_ADDR + elt_idx * REG_EM_BLE_RX_ISO_DESC_SIZE) & ~((uint16_t)0x00008000)) | ((uint16_t)rxdone << 15));
}

__INLINE uint16_t em_ble_rxisoptr_nextptr_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXISOPTR_ADDR + elt_idx * REG_EM_BLE_RX_ISO_DESC_SIZE);
    return ((localVal & ((uint16_t)0x00003FFF)) >> 0);
}

__INLINE void em_ble_rxisoptr_nextptr_setf(int elt_idx, uint16_t nextptr)
{
    ASSERT_ERR((((uint16_t)nextptr << 0) & ~((uint16_t)0x00003FFF)) == 0);
    EM_BLE_WR(EM_BLE_RXISOPTR_ADDR + elt_idx * REG_EM_BLE_RX_ISO_DESC_SIZE, (EM_BLE_RD(EM_BLE_RXISOPTR_ADDR + elt_idx * REG_EM_BLE_RX_ISO_DESC_SIZE) & ~((uint16_t)0x00003FFF)) | ((uint16_t)nextptr << 0));
}

/**
 * @brief RXISOCNT0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00           RXPLD_CNT0   0x0
 * </pre>
 */
#define EM_BLE_RXISOCNT0_ADDR   (0x20418002 + EM_BLE_RX_ISO_DESC_OFFSET)
#define EM_BLE_RXISOCNT0_INDEX  0x00000001
#define EM_BLE_RXISOCNT0_RESET  0x00000000

__INLINE uint16_t em_ble_rxisocnt0_get(int elt_idx)
{
    return EM_BLE_RD(EM_BLE_RXISOCNT0_ADDR + elt_idx * REG_EM_BLE_RX_ISO_DESC_SIZE);
}

__INLINE void em_ble_rxisocnt0_set(int elt_idx, uint16_t value)
{
    EM_BLE_WR(EM_BLE_RXISOCNT0_ADDR + elt_idx * REG_EM_BLE_RX_ISO_DESC_SIZE, value);
}

// field definitions
#define EM_BLE_RXPLD_CNT0_MASK   ((uint16_t)0x0000FFFF)
#define EM_BLE_RXPLD_CNT0_LSB    0
#define EM_BLE_RXPLD_CNT0_WIDTH  ((uint16_t)0x00000010)

#define EM_BLE_RXPLD_CNT0_RST    0x0

__INLINE uint16_t em_ble_rxisocnt0_rxpld_cnt0_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXISOCNT0_ADDR + elt_idx * REG_EM_BLE_RX_ISO_DESC_SIZE);
    ASSERT_ERR((localVal & ~((uint16_t)0x0000FFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void em_ble_rxisocnt0_rxpld_cnt0_setf(int elt_idx, uint16_t rxpldcnt0)
{
    ASSERT_ERR((((uint16_t)rxpldcnt0 << 0) & ~((uint16_t)0x0000FFFF)) == 0);
    EM_BLE_WR(EM_BLE_RXISOCNT0_ADDR + elt_idx * REG_EM_BLE_RX_ISO_DESC_SIZE, (uint16_t)rxpldcnt0 << 0);
}

/**
 * @brief RXISOCNT1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00           RXPLD_CNT1   0x0
 * </pre>
 */
#define EM_BLE_RXISOCNT1_ADDR   (0x20418004 + EM_BLE_RX_ISO_DESC_OFFSET)
#define EM_BLE_RXISOCNT1_INDEX  0x00000002
#define EM_BLE_RXISOCNT1_RESET  0x00000000

__INLINE uint16_t em_ble_rxisocnt1_get(int elt_idx)
{
    return EM_BLE_RD(EM_BLE_RXISOCNT1_ADDR + elt_idx * REG_EM_BLE_RX_ISO_DESC_SIZE);
}

__INLINE void em_ble_rxisocnt1_set(int elt_idx, uint16_t value)
{
    EM_BLE_WR(EM_BLE_RXISOCNT1_ADDR + elt_idx * REG_EM_BLE_RX_ISO_DESC_SIZE, value);
}

// field definitions
#define EM_BLE_RXPLD_CNT1_MASK   ((uint16_t)0x0000FFFF)
#define EM_BLE_RXPLD_CNT1_LSB    0
#define EM_BLE_RXPLD_CNT1_WIDTH  ((uint16_t)0x00000010)

#define EM_BLE_RXPLD_CNT1_RST    0x0

__INLINE uint16_t em_ble_rxisocnt1_rxpld_cnt1_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXISOCNT1_ADDR + elt_idx * REG_EM_BLE_RX_ISO_DESC_SIZE);
    ASSERT_ERR((localVal & ~((uint16_t)0x0000FFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void em_ble_rxisocnt1_rxpld_cnt1_setf(int elt_idx, uint16_t rxpldcnt1)
{
    ASSERT_ERR((((uint16_t)rxpldcnt1 << 0) & ~((uint16_t)0x0000FFFF)) == 0);
    EM_BLE_WR(EM_BLE_RXISOCNT1_ADDR + elt_idx * REG_EM_BLE_RX_ISO_DESC_SIZE, (uint16_t)rxpldcnt1 << 0);
}

/**
 * @brief RXISOCNT2 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:08       RXFLUSHINSTANT   0x0
 *  06:00           RXPLD_CNT2   0x0
 * </pre>
 */
#define EM_BLE_RXISOCNT2_ADDR   (0x20418006 + EM_BLE_RX_ISO_DESC_OFFSET)
#define EM_BLE_RXISOCNT2_INDEX  0x00000003
#define EM_BLE_RXISOCNT2_RESET  0x00000000

__INLINE uint16_t em_ble_rxisocnt2_get(int elt_idx)
{
    return EM_BLE_RD(EM_BLE_RXISOCNT2_ADDR + elt_idx * REG_EM_BLE_RX_ISO_DESC_SIZE);
}

__INLINE void em_ble_rxisocnt2_set(int elt_idx, uint16_t value)
{
    EM_BLE_WR(EM_BLE_RXISOCNT2_ADDR + elt_idx * REG_EM_BLE_RX_ISO_DESC_SIZE, value);
}

// field definitions
#define EM_BLE_RXFLUSHINSTANT_MASK   ((uint16_t)0x0000FF00)
#define EM_BLE_RXFLUSHINSTANT_LSB    8
#define EM_BLE_RXFLUSHINSTANT_WIDTH  ((uint16_t)0x00000008)
#define EM_BLE_RXPLD_CNT2_MASK       ((uint16_t)0x0000007F)
#define EM_BLE_RXPLD_CNT2_LSB        0
#define EM_BLE_RXPLD_CNT2_WIDTH      ((uint16_t)0x00000007)

#define EM_BLE_RXFLUSHINSTANT_RST    0x0
#define EM_BLE_RXPLD_CNT2_RST        0x0

__INLINE void em_ble_rxisocnt2_pack(int elt_idx, uint8_t rxflushinstant, uint8_t rxpldcnt2)
{
    ASSERT_ERR((((uint16_t)rxflushinstant << 8) & ~((uint16_t)0x0000FF00)) == 0);
    ASSERT_ERR((((uint16_t)rxpldcnt2 << 0) & ~((uint16_t)0x0000007F)) == 0);
    EM_BLE_WR(EM_BLE_RXISOCNT2_ADDR + elt_idx * REG_EM_BLE_RX_ISO_DESC_SIZE, ((uint16_t)rxflushinstant << 8) | ((uint16_t)rxpldcnt2 << 0));
}

__INLINE void em_ble_rxisocnt2_unpack(int elt_idx, uint8_t *rxflushinstant, uint8_t *rxpldcnt2)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXISOCNT2_ADDR + elt_idx * REG_EM_BLE_RX_ISO_DESC_SIZE);

    *rxflushinstant = (localVal & ((uint16_t)0x0000FF00)) >> 8;
    *rxpldcnt2 = (localVal & ((uint16_t)0x0000007F)) >> 0;
}

__INLINE uint8_t em_ble_rxisocnt2_rxflushinstant_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXISOCNT2_ADDR + elt_idx * REG_EM_BLE_RX_ISO_DESC_SIZE);
    return ((localVal & ((uint16_t)0x0000FF00)) >> 8);
}

__INLINE void em_ble_rxisocnt2_rxflushinstant_setf(int elt_idx, uint8_t rxflushinstant)
{
    ASSERT_ERR((((uint16_t)rxflushinstant << 8) & ~((uint16_t)0x0000FF00)) == 0);
    EM_BLE_WR(EM_BLE_RXISOCNT2_ADDR + elt_idx * REG_EM_BLE_RX_ISO_DESC_SIZE, (EM_BLE_RD(EM_BLE_RXISOCNT2_ADDR + elt_idx * REG_EM_BLE_RX_ISO_DESC_SIZE) & ~((uint16_t)0x0000FF00)) | ((uint16_t)rxflushinstant << 8));
}

__INLINE uint8_t em_ble_rxisocnt2_rxpld_cnt2_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXISOCNT2_ADDR + elt_idx * REG_EM_BLE_RX_ISO_DESC_SIZE);
    return ((localVal & ((uint16_t)0x0000007F)) >> 0);
}

__INLINE void em_ble_rxisocnt2_rxpld_cnt2_setf(int elt_idx, uint8_t rxpldcnt2)
{
    ASSERT_ERR((((uint16_t)rxpldcnt2 << 0) & ~((uint16_t)0x0000007F)) == 0);
    EM_BLE_WR(EM_BLE_RXISOCNT2_ADDR + elt_idx * REG_EM_BLE_RX_ISO_DESC_SIZE, (EM_BLE_RD(EM_BLE_RXISOCNT2_ADDR + elt_idx * REG_EM_BLE_RX_ISO_DESC_SIZE) & ~((uint16_t)0x0000007F)) | ((uint16_t)rxpldcnt2 << 0));
}

/**
 * @brief RXISOBUFPTR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  13:00          RXISOBUFPTR   0x0
 * </pre>
 */
#define EM_BLE_RXISOBUFPTR_ADDR   (0x20418008 + EM_BLE_RX_ISO_DESC_OFFSET)
#define EM_BLE_RXISOBUFPTR_INDEX  0x00000004
#define EM_BLE_RXISOBUFPTR_RESET  0x00000000

__INLINE uint16_t em_ble_rxisobufptr_get(int elt_idx)
{
    return EM_BLE_RD(EM_BLE_RXISOBUFPTR_ADDR + elt_idx * REG_EM_BLE_RX_ISO_DESC_SIZE);
}

__INLINE void em_ble_rxisobufptr_set(int elt_idx, uint16_t value)
{
    EM_BLE_WR(EM_BLE_RXISOBUFPTR_ADDR + elt_idx * REG_EM_BLE_RX_ISO_DESC_SIZE, value);
}

// field definitions
#define EM_BLE_RXISOBUFPTR_MASK   ((uint16_t)0x00003FFF)
#define EM_BLE_RXISOBUFPTR_LSB    0
#define EM_BLE_RXISOBUFPTR_WIDTH  ((uint16_t)0x0000000E)

#define EM_BLE_RXISOBUFPTR_RST    0x0

__INLINE uint16_t em_ble_rxisobufptr_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXISOBUFPTR_ADDR + elt_idx * REG_EM_BLE_RX_ISO_DESC_SIZE);
    ASSERT_ERR((localVal & ~((uint16_t)0x00003FFF)) == 0);
    return (localVal >> 0);
}

__INLINE void em_ble_rxisobufptr_setf(int elt_idx, uint16_t rxisobufptr)
{
    ASSERT_ERR((((uint16_t)rxisobufptr << 0) & ~((uint16_t)0x00003FFF)) == 0);
    EM_BLE_WR(EM_BLE_RXISOBUFPTR_ADDR + elt_idx * REG_EM_BLE_RX_ISO_DESC_SIZE, (uint16_t)rxisobufptr << 0);
}

/**
 * @brief RXISORESERVED register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00                 RSVD   0x0
 * </pre>
 */
#define EM_BLE_RXISORESERVED_ADDR   (0x2041800A + EM_BLE_RX_ISO_DESC_OFFSET)
#define EM_BLE_RXISORESERVED_INDEX  0x00000005
#define EM_BLE_RXISORESERVED_RESET  0x00000000

__INLINE uint16_t em_ble_rxisoreserved_get(int elt_idx)
{
    return EM_BLE_RD(EM_BLE_RXISORESERVED_ADDR + elt_idx * REG_EM_BLE_RX_ISO_DESC_SIZE);
}

__INLINE void em_ble_rxisoreserved_set(int elt_idx, uint16_t value)
{
    EM_BLE_WR(EM_BLE_RXISORESERVED_ADDR + elt_idx * REG_EM_BLE_RX_ISO_DESC_SIZE, value);
}

// field definitions
#define EM_BLE_RSVD_MASK   ((uint16_t)0x0000FFFF)
#define EM_BLE_RSVD_LSB    0
#define EM_BLE_RSVD_WIDTH  ((uint16_t)0x00000010)

#define EM_BLE_RSVD_RST    0x0

__INLINE uint16_t em_ble_rxisoreserved_rsvd_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXISORESERVED_ADDR + elt_idx * REG_EM_BLE_RX_ISO_DESC_SIZE);
    ASSERT_ERR((localVal & ~((uint16_t)0x0000FFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void em_ble_rxisoreserved_rsvd_setf(int elt_idx, uint16_t rsvd)
{
    ASSERT_ERR((((uint16_t)rsvd << 0) & ~((uint16_t)0x0000FFFF)) == 0);
    EM_BLE_WR(EM_BLE_RXISORESERVED_ADDR + elt_idx * REG_EM_BLE_RX_ISO_DESC_SIZE, (uint16_t)rsvd << 0);
}


#endif // _REG_EM_BLE_RX_ISO_DESC_H_

