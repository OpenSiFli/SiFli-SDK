#ifndef _REG_EM_BT_TXDESC_H_
#define _REG_EM_BT_TXDESC_H_

#include <stdint.h>
#include "_reg_em_bt_txdesc.h"
#include "compiler.h"
#include "arch.h"
#include "em_map.h"
#include "reg_access.h"

#define REG_EM_BT_TXDESC_COUNT 6

#define REG_EM_BT_TXDESC_DECODING_MASK 0x0000000F

#define REG_EM_BT_TXDESC_ADDR_GET(idx) (EM_BT_TXDESC_OFFSET + (idx) * REG_EM_BT_TXDESC_SIZE)

/**
 * @brief TXPTR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     15               TXDONE   0
 *  13:00              NEXTPTR   0x0
 * </pre>
 */
#define EM_BT_TXPTR_ADDR   (0x20418000 + EM_BT_TXDESC_OFFSET)
#define EM_BT_TXPTR_INDEX  0x00000000
#define EM_BT_TXPTR_RESET  0x00000000

__INLINE uint16_t em_bt_txptr_get(int elt_idx)
{
    return EM_BT_RD(EM_BT_TXPTR_ADDR + elt_idx * REG_EM_BT_TXDESC_SIZE);
}

__INLINE void em_bt_txptr_set(int elt_idx, uint16_t value)
{
    EM_BT_WR(EM_BT_TXPTR_ADDR + elt_idx * REG_EM_BT_TXDESC_SIZE, value);
}

// field definitions
#define EM_BT_TXDONE_BIT     ((uint16_t)0x00008000)
#define EM_BT_TXDONE_POS     15
#define EM_BT_NEXTPTR_MASK   ((uint16_t)0x00003FFF)
#define EM_BT_NEXTPTR_LSB    0
#define EM_BT_NEXTPTR_WIDTH  ((uint16_t)0x0000000E)

#define EM_BT_TXDONE_RST     0x0
#define EM_BT_NEXTPTR_RST    0x0

__INLINE void em_bt_txptr_pack(int elt_idx, uint8_t txdone, uint16_t nextptr)
{
    ASSERT_ERR((((uint16_t)txdone << 15) & ~((uint16_t)0x00008000)) == 0);
    ASSERT_ERR((((uint16_t)nextptr << 0) & ~((uint16_t)0x00003FFF)) == 0);
    EM_BT_WR(EM_BT_TXPTR_ADDR + elt_idx * REG_EM_BT_TXDESC_SIZE, ((uint16_t)txdone << 15) | ((uint16_t)nextptr << 0));
}

__INLINE void em_bt_txptr_unpack(int elt_idx, uint8_t *txdone, uint16_t *nextptr)
{
    uint16_t localVal = EM_BT_RD(EM_BT_TXPTR_ADDR + elt_idx * REG_EM_BT_TXDESC_SIZE);

    *txdone = (localVal & ((uint16_t)0x00008000)) >> 15;
    *nextptr = (localVal & ((uint16_t)0x00003FFF)) >> 0;
}

__INLINE uint8_t em_bt_txptr_txdone_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_TXPTR_ADDR + elt_idx * REG_EM_BT_TXDESC_SIZE);
    return ((localVal & ((uint16_t)0x00008000)) >> 15);
}

__INLINE void em_bt_txptr_txdone_setf(int elt_idx, uint8_t txdone)
{
    ASSERT_ERR((((uint16_t)txdone << 15) & ~((uint16_t)0x00008000)) == 0);
    EM_BT_WR(EM_BT_TXPTR_ADDR + elt_idx * REG_EM_BT_TXDESC_SIZE, (EM_BT_RD(EM_BT_TXPTR_ADDR + elt_idx * REG_EM_BT_TXDESC_SIZE) & ~((uint16_t)0x00008000)) | ((uint16_t)txdone << 15));
}

__INLINE uint16_t em_bt_txptr_nextptr_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_TXPTR_ADDR + elt_idx * REG_EM_BT_TXDESC_SIZE);
    return ((localVal & ((uint16_t)0x00003FFF)) >> 0);
}

__INLINE void em_bt_txptr_nextptr_setf(int elt_idx, uint16_t nextptr)
{
    ASSERT_ERR((((uint16_t)nextptr << 0) & ~((uint16_t)0x00003FFF)) == 0);
    EM_BT_WR(EM_BT_TXPTR_ADDR + elt_idx * REG_EM_BT_TXDESC_SIZE, (EM_BT_RD(EM_BT_TXPTR_ADDR + elt_idx * REG_EM_BT_TXDESC_SIZE) & ~((uint16_t)0x00003FFF)) | ((uint16_t)nextptr << 0));
}

/**
 * @brief TXHEADER register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     15                 TXDV   0
 *     14               TXRSVD   0
 *     09               TXSEQN   0
 *     08               TXARQN   0
 *     07               TXFLOW   0
 *  06:03               TXTYPE   0x0
 *  02:00             TXLTADDR   0x0
 * </pre>
 */
#define EM_BT_TXHEADER_ADDR   (0x20418002 + EM_BT_TXDESC_OFFSET)
#define EM_BT_TXHEADER_INDEX  0x00000001
#define EM_BT_TXHEADER_RESET  0x00000000

__INLINE uint16_t em_bt_txheader_get(int elt_idx)
{
    return EM_BT_RD(EM_BT_TXHEADER_ADDR + elt_idx * REG_EM_BT_TXDESC_SIZE);
}

__INLINE void em_bt_txheader_set(int elt_idx, uint16_t value)
{
    EM_BT_WR(EM_BT_TXHEADER_ADDR + elt_idx * REG_EM_BT_TXDESC_SIZE, value);
}

// field definitions
#define EM_BT_TXDV_BIT        ((uint16_t)0x00008000)
#define EM_BT_TXDV_POS        15
#define EM_BT_TXRSVD_BIT      ((uint16_t)0x00004000)
#define EM_BT_TXRSVD_POS      14
#define EM_BT_TXSEQN_BIT      ((uint16_t)0x00000200)
#define EM_BT_TXSEQN_POS      9
#define EM_BT_TXARQN_BIT      ((uint16_t)0x00000100)
#define EM_BT_TXARQN_POS      8
#define EM_BT_TXFLOW_BIT      ((uint16_t)0x00000080)
#define EM_BT_TXFLOW_POS      7
#define EM_BT_TXTYPE_MASK     ((uint16_t)0x00000078)
#define EM_BT_TXTYPE_LSB      3
#define EM_BT_TXTYPE_WIDTH    ((uint16_t)0x00000004)
#define EM_BT_TXLTADDR_MASK   ((uint16_t)0x00000007)
#define EM_BT_TXLTADDR_LSB    0
#define EM_BT_TXLTADDR_WIDTH  ((uint16_t)0x00000003)

#define EM_BT_TXDV_RST        0x0
#define EM_BT_TXRSVD_RST      0x0
#define EM_BT_TXSEQN_RST      0x0
#define EM_BT_TXARQN_RST      0x0
#define EM_BT_TXFLOW_RST      0x0
#define EM_BT_TXTYPE_RST      0x0
#define EM_BT_TXLTADDR_RST    0x0

__INLINE void em_bt_txheader_pack(int elt_idx, uint8_t txdv, uint8_t txrsvd, uint8_t txseqn, uint8_t txarqn, uint8_t txflow, uint8_t txtype, uint8_t txltaddr)
{
    ASSERT_ERR((((uint16_t)txdv << 15) & ~((uint16_t)0x00008000)) == 0);
    ASSERT_ERR((((uint16_t)txrsvd << 14) & ~((uint16_t)0x00004000)) == 0);
    ASSERT_ERR((((uint16_t)txseqn << 9) & ~((uint16_t)0x00000200)) == 0);
    ASSERT_ERR((((uint16_t)txarqn << 8) & ~((uint16_t)0x00000100)) == 0);
    ASSERT_ERR((((uint16_t)txflow << 7) & ~((uint16_t)0x00000080)) == 0);
    ASSERT_ERR((((uint16_t)txtype << 3) & ~((uint16_t)0x00000078)) == 0);
    ASSERT_ERR((((uint16_t)txltaddr << 0) & ~((uint16_t)0x00000007)) == 0);
    EM_BT_WR(EM_BT_TXHEADER_ADDR + elt_idx * REG_EM_BT_TXDESC_SIZE, ((uint16_t)txdv << 15) | ((uint16_t)txrsvd << 14) | ((uint16_t)txseqn << 9) | ((uint16_t)txarqn << 8) | ((uint16_t)txflow << 7) | ((uint16_t)txtype << 3) | ((uint16_t)txltaddr << 0));
}

__INLINE void em_bt_txheader_unpack(int elt_idx, uint8_t *txdv, uint8_t *txrsvd, uint8_t *txseqn, uint8_t *txarqn, uint8_t *txflow, uint8_t *txtype, uint8_t *txltaddr)
{
    uint16_t localVal = EM_BT_RD(EM_BT_TXHEADER_ADDR + elt_idx * REG_EM_BT_TXDESC_SIZE);

    *txdv = (localVal & ((uint16_t)0x00008000)) >> 15;
    *txrsvd = (localVal & ((uint16_t)0x00004000)) >> 14;
    *txseqn = (localVal & ((uint16_t)0x00000200)) >> 9;
    *txarqn = (localVal & ((uint16_t)0x00000100)) >> 8;
    *txflow = (localVal & ((uint16_t)0x00000080)) >> 7;
    *txtype = (localVal & ((uint16_t)0x00000078)) >> 3;
    *txltaddr = (localVal & ((uint16_t)0x00000007)) >> 0;
}

__INLINE uint8_t em_bt_txheader_txdv_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_TXHEADER_ADDR + elt_idx * REG_EM_BT_TXDESC_SIZE);
    return ((localVal & ((uint16_t)0x00008000)) >> 15);
}

__INLINE void em_bt_txheader_txdv_setf(int elt_idx, uint8_t txdv)
{
    ASSERT_ERR((((uint16_t)txdv << 15) & ~((uint16_t)0x00008000)) == 0);
    EM_BT_WR(EM_BT_TXHEADER_ADDR + elt_idx * REG_EM_BT_TXDESC_SIZE, (EM_BT_RD(EM_BT_TXHEADER_ADDR + elt_idx * REG_EM_BT_TXDESC_SIZE) & ~((uint16_t)0x00008000)) | ((uint16_t)txdv << 15));
}

__INLINE uint8_t em_bt_txheader_txrsvd_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_TXHEADER_ADDR + elt_idx * REG_EM_BT_TXDESC_SIZE);
    return ((localVal & ((uint16_t)0x00004000)) >> 14);
}

__INLINE void em_bt_txheader_txrsvd_setf(int elt_idx, uint8_t txrsvd)
{
    ASSERT_ERR((((uint16_t)txrsvd << 14) & ~((uint16_t)0x00004000)) == 0);
    EM_BT_WR(EM_BT_TXHEADER_ADDR + elt_idx * REG_EM_BT_TXDESC_SIZE, (EM_BT_RD(EM_BT_TXHEADER_ADDR + elt_idx * REG_EM_BT_TXDESC_SIZE) & ~((uint16_t)0x00004000)) | ((uint16_t)txrsvd << 14));
}

__INLINE uint8_t em_bt_txheader_txseqn_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_TXHEADER_ADDR + elt_idx * REG_EM_BT_TXDESC_SIZE);
    return ((localVal & ((uint16_t)0x00000200)) >> 9);
}

__INLINE void em_bt_txheader_txseqn_setf(int elt_idx, uint8_t txseqn)
{
    ASSERT_ERR((((uint16_t)txseqn << 9) & ~((uint16_t)0x00000200)) == 0);
    EM_BT_WR(EM_BT_TXHEADER_ADDR + elt_idx * REG_EM_BT_TXDESC_SIZE, (EM_BT_RD(EM_BT_TXHEADER_ADDR + elt_idx * REG_EM_BT_TXDESC_SIZE) & ~((uint16_t)0x00000200)) | ((uint16_t)txseqn << 9));
}

__INLINE uint8_t em_bt_txheader_txarqn_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_TXHEADER_ADDR + elt_idx * REG_EM_BT_TXDESC_SIZE);
    return ((localVal & ((uint16_t)0x00000100)) >> 8);
}

__INLINE void em_bt_txheader_txarqn_setf(int elt_idx, uint8_t txarqn)
{
    ASSERT_ERR((((uint16_t)txarqn << 8) & ~((uint16_t)0x00000100)) == 0);
    EM_BT_WR(EM_BT_TXHEADER_ADDR + elt_idx * REG_EM_BT_TXDESC_SIZE, (EM_BT_RD(EM_BT_TXHEADER_ADDR + elt_idx * REG_EM_BT_TXDESC_SIZE) & ~((uint16_t)0x00000100)) | ((uint16_t)txarqn << 8));
}

__INLINE uint8_t em_bt_txheader_txflow_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_TXHEADER_ADDR + elt_idx * REG_EM_BT_TXDESC_SIZE);
    return ((localVal & ((uint16_t)0x00000080)) >> 7);
}

__INLINE void em_bt_txheader_txflow_setf(int elt_idx, uint8_t txflow)
{
    ASSERT_ERR((((uint16_t)txflow << 7) & ~((uint16_t)0x00000080)) == 0);
    EM_BT_WR(EM_BT_TXHEADER_ADDR + elt_idx * REG_EM_BT_TXDESC_SIZE, (EM_BT_RD(EM_BT_TXHEADER_ADDR + elt_idx * REG_EM_BT_TXDESC_SIZE) & ~((uint16_t)0x00000080)) | ((uint16_t)txflow << 7));
}

__INLINE uint8_t em_bt_txheader_txtype_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_TXHEADER_ADDR + elt_idx * REG_EM_BT_TXDESC_SIZE);
    return ((localVal & ((uint16_t)0x00000078)) >> 3);
}

__INLINE void em_bt_txheader_txtype_setf(int elt_idx, uint8_t txtype)
{
    ASSERT_ERR((((uint16_t)txtype << 3) & ~((uint16_t)0x00000078)) == 0);
    EM_BT_WR(EM_BT_TXHEADER_ADDR + elt_idx * REG_EM_BT_TXDESC_SIZE, (EM_BT_RD(EM_BT_TXHEADER_ADDR + elt_idx * REG_EM_BT_TXDESC_SIZE) & ~((uint16_t)0x00000078)) | ((uint16_t)txtype << 3));
}

__INLINE uint8_t em_bt_txheader_txltaddr_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_TXHEADER_ADDR + elt_idx * REG_EM_BT_TXDESC_SIZE);
    return ((localVal & ((uint16_t)0x00000007)) >> 0);
}

__INLINE void em_bt_txheader_txltaddr_setf(int elt_idx, uint8_t txltaddr)
{
    ASSERT_ERR((((uint16_t)txltaddr << 0) & ~((uint16_t)0x00000007)) == 0);
    EM_BT_WR(EM_BT_TXHEADER_ADDR + elt_idx * REG_EM_BT_TXDESC_SIZE, (EM_BT_RD(EM_BT_TXHEADER_ADDR + elt_idx * REG_EM_BT_TXDESC_SIZE) & ~((uint16_t)0x00000007)) | ((uint16_t)txltaddr << 0));
}

/**
 * @brief TXPHEADER register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:13              TXUNDEF   0x0
 *  12:03             TXLENGTH   0x0
 *     02              TXPFLOW   0
 *  01:00               TXLLID   0x0
 * </pre>
 */
#define EM_BT_TXPHEADER_ADDR   (0x20418004 + EM_BT_TXDESC_OFFSET)
#define EM_BT_TXPHEADER_INDEX  0x00000002
#define EM_BT_TXPHEADER_RESET  0x00000000

__INLINE uint16_t em_bt_txpheader_get(int elt_idx)
{
    return EM_BT_RD(EM_BT_TXPHEADER_ADDR + elt_idx * REG_EM_BT_TXDESC_SIZE);
}

__INLINE void em_bt_txpheader_set(int elt_idx, uint16_t value)
{
    EM_BT_WR(EM_BT_TXPHEADER_ADDR + elt_idx * REG_EM_BT_TXDESC_SIZE, value);
}

// field definitions
#define EM_BT_TXUNDEF_MASK    ((uint16_t)0x0000E000)
#define EM_BT_TXUNDEF_LSB     13
#define EM_BT_TXUNDEF_WIDTH   ((uint16_t)0x00000003)
#define EM_BT_TXLENGTH_MASK   ((uint16_t)0x00001FF8)
#define EM_BT_TXLENGTH_LSB    3
#define EM_BT_TXLENGTH_WIDTH  ((uint16_t)0x0000000A)
#define EM_BT_TXPFLOW_BIT     ((uint16_t)0x00000004)
#define EM_BT_TXPFLOW_POS     2
#define EM_BT_TXLLID_MASK     ((uint16_t)0x00000003)
#define EM_BT_TXLLID_LSB      0
#define EM_BT_TXLLID_WIDTH    ((uint16_t)0x00000002)

#define EM_BT_TXUNDEF_RST     0x0
#define EM_BT_TXLENGTH_RST    0x0
#define EM_BT_TXPFLOW_RST     0x0
#define EM_BT_TXLLID_RST      0x0

__INLINE void em_bt_txpheader_pack(int elt_idx, uint8_t txundef, uint16_t txlength, uint8_t txpflow, uint8_t txllid)
{
    ASSERT_ERR((((uint16_t)txundef << 13) & ~((uint16_t)0x0000E000)) == 0);
    ASSERT_ERR((((uint16_t)txlength << 3) & ~((uint16_t)0x00001FF8)) == 0);
    ASSERT_ERR((((uint16_t)txpflow << 2) & ~((uint16_t)0x00000004)) == 0);
    ASSERT_ERR((((uint16_t)txllid << 0) & ~((uint16_t)0x00000003)) == 0);
    EM_BT_WR(EM_BT_TXPHEADER_ADDR + elt_idx * REG_EM_BT_TXDESC_SIZE, ((uint16_t)txundef << 13) | ((uint16_t)txlength << 3) | ((uint16_t)txpflow << 2) | ((uint16_t)txllid << 0));
}

__INLINE void em_bt_txpheader_unpack(int elt_idx, uint8_t *txundef, uint16_t *txlength, uint8_t *txpflow, uint8_t *txllid)
{
    uint16_t localVal = EM_BT_RD(EM_BT_TXPHEADER_ADDR + elt_idx * REG_EM_BT_TXDESC_SIZE);

    *txundef = (localVal & ((uint16_t)0x0000E000)) >> 13;
    *txlength = (localVal & ((uint16_t)0x00001FF8)) >> 3;
    *txpflow = (localVal & ((uint16_t)0x00000004)) >> 2;
    *txllid = (localVal & ((uint16_t)0x00000003)) >> 0;
}

__INLINE uint8_t em_bt_txpheader_txundef_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_TXPHEADER_ADDR + elt_idx * REG_EM_BT_TXDESC_SIZE);
    return ((localVal & ((uint16_t)0x0000E000)) >> 13);
}

__INLINE void em_bt_txpheader_txundef_setf(int elt_idx, uint8_t txundef)
{
    ASSERT_ERR((((uint16_t)txundef << 13) & ~((uint16_t)0x0000E000)) == 0);
    EM_BT_WR(EM_BT_TXPHEADER_ADDR + elt_idx * REG_EM_BT_TXDESC_SIZE, (EM_BT_RD(EM_BT_TXPHEADER_ADDR + elt_idx * REG_EM_BT_TXDESC_SIZE) & ~((uint16_t)0x0000E000)) | ((uint16_t)txundef << 13));
}

__INLINE uint16_t em_bt_txpheader_txlength_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_TXPHEADER_ADDR + elt_idx * REG_EM_BT_TXDESC_SIZE);
    return ((localVal & ((uint16_t)0x00001FF8)) >> 3);
}

__INLINE void em_bt_txpheader_txlength_setf(int elt_idx, uint16_t txlength)
{
    ASSERT_ERR((((uint16_t)txlength << 3) & ~((uint16_t)0x00001FF8)) == 0);
    EM_BT_WR(EM_BT_TXPHEADER_ADDR + elt_idx * REG_EM_BT_TXDESC_SIZE, (EM_BT_RD(EM_BT_TXPHEADER_ADDR + elt_idx * REG_EM_BT_TXDESC_SIZE) & ~((uint16_t)0x00001FF8)) | ((uint16_t)txlength << 3));
}

__INLINE uint8_t em_bt_txpheader_txpflow_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_TXPHEADER_ADDR + elt_idx * REG_EM_BT_TXDESC_SIZE);
    return ((localVal & ((uint16_t)0x00000004)) >> 2);
}

__INLINE void em_bt_txpheader_txpflow_setf(int elt_idx, uint8_t txpflow)
{
    ASSERT_ERR((((uint16_t)txpflow << 2) & ~((uint16_t)0x00000004)) == 0);
    EM_BT_WR(EM_BT_TXPHEADER_ADDR + elt_idx * REG_EM_BT_TXDESC_SIZE, (EM_BT_RD(EM_BT_TXPHEADER_ADDR + elt_idx * REG_EM_BT_TXDESC_SIZE) & ~((uint16_t)0x00000004)) | ((uint16_t)txpflow << 2));
}

__INLINE uint8_t em_bt_txpheader_txllid_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_TXPHEADER_ADDR + elt_idx * REG_EM_BT_TXDESC_SIZE);
    return ((localVal & ((uint16_t)0x00000003)) >> 0);
}

__INLINE void em_bt_txpheader_txllid_setf(int elt_idx, uint8_t txllid)
{
    ASSERT_ERR((((uint16_t)txllid << 0) & ~((uint16_t)0x00000003)) == 0);
    EM_BT_WR(EM_BT_TXPHEADER_ADDR + elt_idx * REG_EM_BT_TXDESC_SIZE, (EM_BT_RD(EM_BT_TXPHEADER_ADDR + elt_idx * REG_EM_BT_TXDESC_SIZE) & ~((uint16_t)0x00000003)) | ((uint16_t)txllid << 0));
}

/**
 * @brief TXACLBUFPTR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00          TXACLBUFPTR   0x0
 * </pre>
 */
#define EM_BT_TXACLBUFPTR_ADDR   (0x20418006 + EM_BT_TXDESC_OFFSET)
#define EM_BT_TXACLBUFPTR_INDEX  0x00000003
#define EM_BT_TXACLBUFPTR_RESET  0x00000000

__INLINE uint16_t em_bt_txaclbufptr_get(int elt_idx)
{
    return EM_BT_RD(EM_BT_TXACLBUFPTR_ADDR + elt_idx * REG_EM_BT_TXDESC_SIZE);
}

__INLINE void em_bt_txaclbufptr_set(int elt_idx, uint16_t value)
{
    EM_BT_WR(EM_BT_TXACLBUFPTR_ADDR + elt_idx * REG_EM_BT_TXDESC_SIZE, value);
}

// field definitions
#define EM_BT_TXACLBUFPTR_MASK   ((uint16_t)0x0000FFFF)
#define EM_BT_TXACLBUFPTR_LSB    0
#define EM_BT_TXACLBUFPTR_WIDTH  ((uint16_t)0x00000010)

#define EM_BT_TXACLBUFPTR_RST    0x0

__INLINE uint16_t em_bt_txaclbufptr_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_TXACLBUFPTR_ADDR + elt_idx * REG_EM_BT_TXDESC_SIZE);
    ASSERT_ERR((localVal & ~((uint16_t)0x0000FFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void em_bt_txaclbufptr_setf(int elt_idx, uint16_t txaclbufptr)
{
    ASSERT_ERR((((uint16_t)txaclbufptr << 0) & ~((uint16_t)0x0000FFFF)) == 0);
    EM_BT_WR(EM_BT_TXACLBUFPTR_ADDR + elt_idx * REG_EM_BT_TXDESC_SIZE, (uint16_t)txaclbufptr << 0);
}

/**
 * @brief TXLMBUFPTR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00           TXLMBUFPTR   0x0
 * </pre>
 */
#define EM_BT_TXLMBUFPTR_ADDR   (0x20418008 + EM_BT_TXDESC_OFFSET)
#define EM_BT_TXLMBUFPTR_INDEX  0x00000004
#define EM_BT_TXLMBUFPTR_RESET  0x00000000

__INLINE uint16_t em_bt_txlmbufptr_get(int elt_idx)
{
    return EM_BT_RD(EM_BT_TXLMBUFPTR_ADDR + elt_idx * REG_EM_BT_TXDESC_SIZE);
}

__INLINE void em_bt_txlmbufptr_set(int elt_idx, uint16_t value)
{
    EM_BT_WR(EM_BT_TXLMBUFPTR_ADDR + elt_idx * REG_EM_BT_TXDESC_SIZE, value);
}

// field definitions
#define EM_BT_TXLMBUFPTR_MASK   ((uint16_t)0x0000FFFF)
#define EM_BT_TXLMBUFPTR_LSB    0
#define EM_BT_TXLMBUFPTR_WIDTH  ((uint16_t)0x00000010)

#define EM_BT_TXLMBUFPTR_RST    0x0

__INLINE uint16_t em_bt_txlmbufptr_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_TXLMBUFPTR_ADDR + elt_idx * REG_EM_BT_TXDESC_SIZE);
    ASSERT_ERR((localVal & ~((uint16_t)0x0000FFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void em_bt_txlmbufptr_setf(int elt_idx, uint16_t txlmbufptr)
{
    ASSERT_ERR((((uint16_t)txlmbufptr << 0) & ~((uint16_t)0x0000FFFF)) == 0);
    EM_BT_WR(EM_BT_TXLMBUFPTR_ADDR + elt_idx * REG_EM_BT_TXDESC_SIZE, (uint16_t)txlmbufptr << 0);
}


#endif // _REG_EM_BT_TXDESC_H_

